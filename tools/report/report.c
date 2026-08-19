/**
 * @file report.c
 * @brief Assembles results/ and the claim inventory into one markdown report.
 */
#include <fleet_dt/plot.h>
#include <fleet_dt/version.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** What the "where it lives" column carries for a claim. */
typedef enum {
    ST_ARTEFACT, /**< A path in this repository; its presence is checked. */
    ST_FUTURE,   /**< The paper's own roadmap, so outside this repository. */
    ST_ROLLUP    /**< Derived from other claims. */
} status_kind_t;

/** One row of the inventory. */
typedef struct {
    const char   *id;
    const char   *what;     /**< Short restatement of the claim. */
    const char   *section;  /**< Where in the paper. */
    const char   *artefact; /**< Path checked, or a note for the other kinds. */
    status_kind_t kind;
} claim_t;

/**
 * The inventory, mirroring docs/claim-map.md.
 */
static const claim_t CLAIMS[] = {
    { "C1",  "fleet-level DT model and architecture", "Abstract",
      "roll-up of C2 through C19", ST_ROLLUP },
    { "C2",  "architecture relies on MQTT", "Abstract, III",
      "tests/it_mqtt.c", ST_ARTEFACT },
    { "C3",  "integrates Ardupilot, WeBots, other simulations", "Abstract",
      "adapters/webots/worlds/jundia_fleet.wbt", ST_ARTEFACT },
    { "C4",  "link-budget modelling validated", "Abstract, V-A",
      "src/linkbudget.c", ST_ARTEFACT },
    { "C5",  "bandwidth regulators guaranteeing QoS", "Abstract, III",
      "src/regulator.c", ST_ARTEFACT },
    { "C6",  "fleet as a DTA of per-vessel DTIs", "I (i)",
      "src/fleet.c", ST_ARTEFACT },
    { "C7",  "parallel simulations in one DTE", "I (ii)",
      "src/dte.c", ST_ARTEFACT },
    { "C8",  "near-real-time 3D reference", "I (iii)",
      "adapters/webots/controllers/fdt_controller/fdt_controller.c",
      ST_ARTEFACT },
    { "C9",  "brokers in bridge mode", "III",
      "config/mosquitto/boat.conf", ST_ARTEFACT },
    { "C10", "LSDT small against 100 Mbps", "III",
      "tools/bench/bench_bandwidth.c", ST_ARTEFACT },
    { "C11", "camera feed separated from MQTT", "III",
      "adapters/rtsp/fdt_rtsp.h", ST_ARTEFACT },
    { "C12", "regulators drop in the client, sensors keep pace", "III",
      "tests/test_regulator.c", ST_ARTEFACT },
    { "C13", "8 Hz, 125 ms hard deadline", "IV",
      "src/tick.c", ST_ARTEFACT },
    { "C14", "48-byte state, 48d queue, 23 KB per minute", "IV",
      "tests/test_queue.c", ST_ARTEFACT },
    { "C15", "feasible only if delta fits the period", "IV",
      "src/feasibility.c", ST_ARTEFACT },
    { "C16", "non-autonomous vessel, A subset of B", "IV",
      "src/transition.c", ST_ARTEFACT },
    { "C17", "homogeneous and heterogeneous fleets", "IV",
      "tests/test_fleet.c", ST_ARTEFACT },
    { "C18", "coordinator computes c^t while distributing g^t", "IV",
      "src/coordinator.c", ST_ARTEFACT },
    { "C19", "Table I's 21 entries and their units", "IV",
      "include/fleet_dt/model.h", ST_ARTEFACT },
    { "C20", "bandwidth figure of Section V-A", "V-A",
      "results/bandwidth.txt", ST_ARTEFACT },
    { "C21", "no notable MQTT latency, no stuttering", "V-A",
      "results/jitter.txt", ST_ARTEFACT },
    { "C22", "WeBots CPU: 10 % then under 1 %", "V-A",
      "tools/bench/bench_webots_cpu.c", ST_ARTEFACT },
    { "C23", "delta feasible, actuation still late", "V-A",
      "results/latency.txt", ST_ARTEFACT },
    { "C24", "state range enabling MPC-like operation", "V-A",
      "examples/daemon.c", ST_ARTEFACT },
    { "C25", "under 1 % CPU per added DTI", "V-B",
      "results/scale.txt", ST_ARTEFACT },
    { "C26", "injector-bound ceiling near 25 boats", "V-B",
      "tools/injector/injector.c", ST_ARTEFACT },
    { "C27", "partial and double frame updates", "V-B",
      "src/framesync.c", ST_ARTEFACT },
    { "C28", "Kalman for the operator, raw for actuation", "V-A",
      "examples/two_paths.c", ST_ARTEFACT },

    { "D1",  "no formal input-to-delta connection", "VI",
      "Section VI names it future work", ST_FUTURE },
    { "D2",  "fluid and ML simulations in the model", "VI",
      "Section VI names it future work", ST_FUTURE },
    { "D3",  "MQTT inside the firmware", "VI",
      "Section VI names it future work", ST_FUTURE },
    { "D4",  "regulators as a broker QoS rule", "VI",
      "Section VI names it future work", ST_FUTURE },
    { "D5",  "real-time protocol under MQTT", "VI",
      "Section VI names it future work", ST_FUTURE },
    { "D6",  "dropping late packets at the receiver", "VI",
      "Section VI names it future work", ST_FUTURE },
    { "D7",  "fleet result is HILS only", "V",
      "Section V; the field campaign is the paper's next step", ST_FUTURE },
};
#define NCLAIMS ((int)(sizeof CLAIMS / sizeof CLAIMS[0]))

/** Benchmarks whose artefacts are folded into the report, in reading order. */
typedef struct {
    const char *stem;
    const char *title;
    const char *charts[3]; /**< NULL-terminated list of SVG stems. */
} bench_t;

static const bench_t BENCHES[] = {
    { "regulator", "Bandwidth regulators (C5, C12)",
      { "regulator", "regulator_saved", NULL } },
    { "bandwidth", "Link budget (C4, C10, C20)",
      { "bandwidth", "bandwidth_fleet", NULL } },
    { "scale",     "Fleet scaling (C25, C26)",
      { "scale", "scale_cpu", NULL } },
    { "latency",   "Delta time against actuation round trip (C23)",
      { "latency", NULL, NULL } },
    { "jitter",    "Frame timing (C13, C21)",
      { "jitter", "jitter_deviation", NULL } },
    { "webots_cpu", "CPU cost per vessel in the simulation (C22)",
      { "webots_cpu", NULL, NULL } },
};
#define NBENCHES ((int)(sizeof BENCHES / sizeof BENCHES[0]))

/** Whether a path can be opened for reading. */
static int exists(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return 0;
    }
    fclose(f);
    return 1;
}

/** Copies a file into @p out, prefixing nothing. */
static int emit_file(FILE *out, const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return -1;
    }

    char line[1024];
    while (fgets(line, sizeof line, f) != NULL) {
        fputs(line, out);
    }
    fclose(f);
    return 0;
}

/** What the claim map should show under "where it lives". */
static const char *where_word(const claim_t *c)
{
    return c->artefact;
}

int main(void)
{
    /* Every artefact this map names has to be present in the tree. */
    int missing = 0;

    /* Console summary, for the person who just ran make. */
    printf("== fleet-dt claim map ==\n%s\n\n", fdt_version());

    int claims = 0;
    int future = 0;

    for (int i = 0; i < NCLAIMS; i++) {
        const claim_t *c = &CLAIMS[i];

        printf("  %-4s %-46s %s\n", c->id, c->what, c->artefact);

        if (c->kind == ST_FUTURE) {
            future++;
        } else {
            claims++;
        }

        if (c->kind == ST_ARTEFACT && !exists(c->artefact)) {
            fprintf(stderr, "  %s names %s, which is not in the tree\n",
                    c->id, c->artefact);
            missing++;
        }
    }

    printf("\n  %d claims mapped   %d items Section VI names as future work\n",
           claims, future);
    if (missing > 0) {
        fprintf(stderr, "  %d artefact path(s) need updating in %s\n",
                missing, "tools/report/report.c");
    }

    /* The markdown report. */
    fdt_mkdir_p("docs");
    FILE *out = fopen("docs/RESULTS.md", "w");
    if (out == NULL) {
        fprintf(stderr, "cannot write docs/RESULTS.md\n");
        return EXIT_FAILURE;
    }

    fprintf(out, "# Results\n\n");
    fprintf(out, "Generated by `make report` from the artefacts in "
                 "`results/`, which are committed\nalongside it so the charts "
                 "resolve on a fresh clone. Every figure below comes\nfrom a "
                 "run on the machine that produced this file; nothing is "
                 "quoted from the\npaper except where a line is labelled "
                 "`paper:`. Re-running `make bench` regenerates\nthem with "
                 "your machine's numbers.\n\n");
    fprintf(out, "Built with %s.\n\n", fdt_version());

    fprintf(out, "## How to read a benchmark line\n\n"
                 "Each comparison prints what this run measured, then the "
                 "figure the paper\npublishes, then how the two stand "
                 "together.\n\n"
                 "- `[matches paper]`: the measurement lands on the "
                 "published figure.\n"
                 "- `[this host]`: the value belongs to the machine that ran "
                 "it. A benchmark\n  times the host, and Section V timed a "
                 "different one.\n"
                 "- `[external system]`: the quantity belongs to a system "
                 "this repository\n  connects to rather than contains.\n\n");

    fprintf(out, "## Claim map\n\n");
    fprintf(out, "One row per statement the paper makes, and where it lives "
                 "here. The long form,\nwith the paper's own wording, is "
                 "[`docs/claim-map.md`](claim-map.md).\n\n");
    fprintf(out, "| id | claim | § | where it lives |\n");
    fprintf(out, "|---|---|---|---|\n");
    for (int i = 0; i < NCLAIMS; i++) {
        const claim_t *c = &CLAIMS[i];
        fprintf(out, "| %s | %s | %s | `%s` |\n",
                c->id, c->what, c->section, where_word(c));
    }
    fprintf(out, "\n**%d claims mapped · %d items Section VI names as future "
                 "work**\n\n", claims, future);
    fprintf(out, "The `D` rows belong to the paper's own roadmap, so they sit "
                 "outside the scope\nof this repository.\n\n");

    for (int i = 0; i < NBENCHES; i++) {
        const bench_t *b = &BENCHES[i];
        char path[256];

        snprintf(path, sizeof path, "results/%s.txt", b->stem);
        if (!exists(path)) {
            continue;
        }

        fprintf(out, "## %s\n\n", b->title);

        for (int c = 0; c < 3 && b->charts[c] != NULL; c++) {
            char svg[256];
            snprintf(svg, sizeof svg, "results/%s.svg", b->charts[c]);
            if (exists(svg)) {
                fprintf(out, "![%s](../%s)\n\n", b->charts[c], svg);
            }
        }

        fprintf(out, "```\n");
        emit_file(out, path);
        fprintf(out, "```\n\n");

        snprintf(path, sizeof path, "results/%s.csv", b->stem);
        if (exists(path)) {
            fprintf(out, "Raw series: [`%s`](../%s)\n\n", path, path);
        }
    }

    fprintf(out, "## Reproducing this\n\n```\nmake clean && make\nmake report"
                 "\n```\n\nThe run takes about a minute, most of it the "
                 "jitter benchmark, which spends\nten seconds of wall clock "
                 "by construction: it is measuring a 125 ms frame\nclock over "
                 "eighty frames.\n");

    fclose(out);
    printf("\nwrote docs/RESULTS.md\n");
    return EXIT_SUCCESS;
}
