/**
 * @file report.c
 * @brief Assembles results/ and the claim inventory into one markdown report.
 *
 * Two jobs, one pass:
 *
 * 1. **Claim status.** Every claim in docs/spec/paper-claims.md names the
 *    artefact that discharges it. This walks that list, checks whether the
 *    artefact exists, and prints what the table *should* say. It never edits
 *    the table: a mismatch between the two is exactly the thing a human
 *    should look at, and a tool that silently reconciled them would hide it.
 *
 * 2. **Results assembly.** Every benchmark writes `<name>.txt`, `<name>.csv`
 *    and one or more `<name>*.svg` into results/. This collects them into
 *    docs/RESULTS.md, with the charts embedded and the reports quoted.
 *
 * C1 is a roll-up rather than an artefact of its own: "a fleet-level Digital
 * Twin model and architecture" is the sum of C2 through C19, so it is
 * discharged when each of those is either discharged or a boundary, and
 * reported as held back by whichever one is neither.
 *
 * Existing is not the same as compiling, and compiling is not the same as
 * running. `make syntax` type-checks the two SDK adapters against stub headers
 * on any machine; `make mqtt-test` and `make webots` build and run them for
 * real where the SDKs exist. C2, C3 and C8 moved from boundary to discharged
 * on 2026-08-17, when both toolchains were installed here and the adapters
 * were exercised end to end: a round trip through mosquitto, and the Jundiá
 * world stepping under the coordinator.
 *
 * Run by `make report`, after `make bench`.
 */
#include <fleet_dt/plot.h>
#include <fleet_dt/version.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** How a claim is discharged. */
typedef enum {
    ST_ARTEFACT, /**< An artefact must exist; the path is checked. */
    ST_BOUNDARY, /**< The real system lives outside this repository. */
    ST_DEFERRED, /**< Section VI declares it future work; building it is wrong. */
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
 * The inventory, mirroring docs/spec/paper-claims.md.
 *
 * Kept in the same order as the spec so a reader can diff the two by eye.
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
      "measured; under this host's noise floor", ST_BOUNDARY },
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
      "declared future work", ST_DEFERRED },
    { "D2",  "fluid and ML simulations in the model", "VI",
      "declared future work", ST_DEFERRED },
    { "D3",  "MQTT inside the firmware", "VI",
      "declared future work", ST_DEFERRED },
    { "D4",  "regulators as a broker QoS rule", "VI",
      "declared future work", ST_DEFERRED },
    { "D5",  "real-time protocol under MQTT", "VI",
      "declared future work", ST_DEFERRED },
    { "D6",  "dropping late packets at the receiver", "VI",
      "declared future work", ST_DEFERRED },
    { "D7",  "fleet result is HILS only", "V",
      "field campaign pending", ST_DEFERRED },
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

/** The word the spec's status column should carry for this claim. */
static const char *status_word(const claim_t *c, int rollup_ok)
{
    switch (c->kind) {
    case ST_BOUNDARY: return "fronteira";
    case ST_DEFERRED: return "diferido";
    case ST_ROLLUP:   return rollup_ok ? "quita" : "pendente";
    case ST_ARTEFACT:
    default:          return exists(c->artefact) ? "quita" : "pendente";
    }
}

int main(void)
{
    /* The roll-up: C1 holds once C2 through C19 all do. */
    int rollup_ok = 1;
    const char *rollup_blocker = NULL;

    for (int i = 0; i < NCLAIMS; i++) {
        const claim_t *c = &CLAIMS[i];
        if (c->id[0] != 'C' || c->kind == ST_ROLLUP) {
            continue;
        }
        const int n = atoi(c->id + 1);
        if (n < 2 || n > 19) {
            continue;
        }
        if (c->kind == ST_ARTEFACT && !exists(c->artefact)) {
            rollup_ok = 0;
            if (rollup_blocker == NULL) {
                rollup_blocker = c->id;
            }
        }
    }

    /* Console summary, for the person who just ran make. */
    printf("== fleet-dt claim status ==\n%s\n\n", fdt_version());

    int quita = 0;
    int pendente = 0;
    int fronteira = 0;
    int diferido = 0;

    for (int i = 0; i < NCLAIMS; i++) {
        const claim_t *c = &CLAIMS[i];
        const char *word = status_word(c, rollup_ok);

        printf("  %-4s %-10s %-46s %s\n", c->id, word, c->what, c->artefact);

        if (strcmp(word, "quita") == 0)          { quita++; }
        else if (strcmp(word, "pendente") == 0)  { pendente++; }
        else if (strcmp(word, "fronteira") == 0) { fronteira++; }
        else                                     { diferido++; }
    }

    printf("\n  quita %d   fronteira %d   diferido %d   pendente %d\n",
           quita, fronteira, diferido, pendente);
    if (!rollup_ok && rollup_blocker != NULL) {
        printf("  C1 is held back by %s\n", rollup_blocker);
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
                 "`paper:`. Re-running `make bench` overwrites\nthem with "
                 "your machine's numbers.\n\n");
    fprintf(out, "Built with %s.\n\n", fdt_version());

    fprintf(out, "## How to read the verdicts\n\n"
                 "- `[OK]` — the measurement agrees with the published "
                 "figure.\n"
                 "- `[DIVERGE]` — it does not. This is reported, never "
                 "fatal: a benchmark\n  measures *this* machine, and this "
                 "machine is not the one the paper\n  measured.\n"
                 "- `[BOUNDARY]` — not measurable here, because the real "
                 "system lives outside\n  this repository.\n\n");

    fprintf(out, "## Claim coverage\n\n");
    fprintf(out, "| id | claim | § | status | artefact |\n");
    fprintf(out, "|---|---|---|---|---|\n");
    for (int i = 0; i < NCLAIMS; i++) {
        const claim_t *c = &CLAIMS[i];
        fprintf(out, "| %s | %s | %s | %s | `%s` |\n",
                c->id, c->what, c->section, status_word(c, rollup_ok),
                c->artefact);
    }
    fprintf(out, "\n**quita %d · fronteira %d · diferido %d · pendente %d**\n\n",
            quita, fronteira, diferido, pendente);
    fprintf(out, "`diferido` items are the ones Section VI declares future "
                 "work. Building them\nwould contradict the published text, "
                 "so their absence is the correct state.\n\n");

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
