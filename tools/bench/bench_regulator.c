/**
 * @file bench_regulator.c
 * @brief What the bandwidth regulators of Section III actually remove.
 */
#include "bench_common.h"

#include <fleet_dt/regulator.h>

#include <assert.h>
#include <math.h>
#include <stdlib.h>

/** The DT rate: one increment of k per 125 ms. */
#define DT_HZ 8.0

/** DT ticks driven per sensor (2500 ticks = 312.5 s), enough to settle. */
#define TICKS 2500

/**
 * Drives one sensor for @p ticks DT ticks, as slide 8 draws it: the samples
 * of a tick are offered in time order, and the edge releases the newest.
 * Returns the samples the control path read; @p mean_age_ms receives the
 * mean age of the released sample at the edge.
 */
static unsigned long drive(fdt_reg_t *reg, double hz, unsigned ticks,
                           double *mean_age_ms)
{
    unsigned long reads = 0;
    double age_sum = 0.0;
    unsigned long released = 0;
    double newest_t = 0.0;

    for (unsigned k = 0; k < ticks; k++) {
        const unsigned m = fdt_reg_samples_in_tick(hz, DT_HZ, k);
        const double first =
            floor((double)k * hz / DT_HZ + 1e-9) + 1.0;
        for (unsigned j = 0; j < m; j++) {
            reads++;
            newest_t = (first + (double)j) / hz;
            fdt_reg_offer(reg);
        }
        if (fdt_reg_capture(reg)) {
            age_sum += (double)(k + 1) / DT_HZ - newest_t;
            released++;
        }
    }
    *mean_age_ms = (released > 0) ? 1e3 * age_sum / (double)released : 0.0;
    return reads;
}

/** Sensor rates swept, spanning the slow and fast cases Section III names. */
static const double RATES[] = {
    1.0, 2.0, 4.0, 8.0, 10.0, 20.0, 50.0, 100.0, 200.0, 400.0, 1000.0
};
#define NRATES ((int)(sizeof RATES / sizeof RATES[0]))

int main(void)
{
    bench_report_t r;
    bench_open(&r, "regulator");
    bench_banner(&r, "fleet-dt bandwidth regulator bench");
    bench_say(&r, "DT rate: %.0f Hz   ticks per sensor: %d (%.1f s)\n",
              DT_HZ, TICKS, TICKS / DT_HZ);
    bench_say(&r, "regulator: newest sample held, released at the tick "
                  "edge (slide 8)\n");

    static double xs[NRATES];
    static double sampled_hz[NRATES];
    static double published_hz[NRATES];
    static double saved_pct[NRATES];

    bench_section(&r, "per sensor rate");
    bench_say(&r, "  %10s %12s %12s %10s %12s %10s\n",
              "sensor Hz", "published Hz", "control Hz", "saved",
              "age at edge", "verdict");

    for (int i = 0; i < NRATES; i++) {
        const double hz = RATES[i];

        fdt_reg_t reg;
        assert(fdt_reg_init(&reg, hz) == 0);

        double age_ms = 0.0;
        const unsigned long control_reads = drive(&reg, hz, TICKS, &age_ms);

        const double eff    = fdt_reg_effective_hz(&reg);
        const double saved  = 100.0 * fdt_reg_saved_ratio(&reg);
        const double ctrl   = (double)control_reads / (TICKS / DT_HZ);

        xs[i]           = hz;
        sampled_hz[i]   = hz;
        published_hz[i] = eff;
        saved_pct[i]    = saved;

        assert(fdt_reg_sampled(&reg) ==
               fdt_reg_published(&reg) + fdt_reg_dropped(&reg));
        assert(control_reads == (unsigned long)fdt_reg_sampled(&reg));

        const char *verdict = (hz <= DT_HZ) ? "untouched" : "decimated";
        bench_say(&r, "  %10.0f %12.2f %12.0f %9.1f %% %9.2f ms %10s\n",
                  hz, eff, ctrl, saved, age_ms, verdict);
    }

    char buf[64];

    bench_section(&r, "the two halves of the Section III claim");

    /* Fast sensor: publication falls to the DT rate. */
    fdt_reg_t fast;
    double fast_age = 0.0;
    assert(fdt_reg_init(&fast, 1000.0) == 0);
    (void)drive(&fast, 1000.0, TICKS, &fast_age);
    snprintf(buf, sizeof buf, "%.2f Hz from 1000 Hz",
             fdt_reg_effective_hz(&fast));
    bench_compare(&r, "publication decimated", buf,
                  "drops samples in the MQTT client",
                  (fdt_reg_effective_hz(&fast) > 7.5 &&
                   fdt_reg_effective_hz(&fast) < 8.5)
                      ? BENCH_MATCH : BENCH_HOST);

    snprintf(buf, sizeof buf, "%.1f %% of traffic",
             100.0 * fdt_reg_saved_ratio(&fast));
    bench_compare(&r, "link traffic removed", buf,
                  "maximizing use of shared links", BENCH_MATCH);

    /* Slow sensor: nothing is dropped, because there is nothing to drop. */
    fdt_reg_t slow;
    double slow_age = 0.0;
    assert(fdt_reg_init(&slow, 1.0) == 0);
    (void)drive(&slow, 1.0, TICKS, &slow_age);
    assert(fdt_reg_published(&slow) == fdt_reg_sampled(&slow));
    snprintf(buf, sizeof buf, "%llu dropped",
             (unsigned long long)fdt_reg_dropped(&slow));
    bench_compare(&r, "1 Hz gyroscope", buf,
                  "sensors slower than the DT pass", BENCH_MATCH);

    bench_say(&r,
        "  The regulator sits in the publish path, never in the sensor path.\n"
        "  Sampled counts every reading the sensor produced, including the\n"
        "  ones the link never carried, so a control loop reading the sensor\n"
        "  directly keeps its full rate. A regulator that slowed the sensor\n"
        "  would buy bandwidth by degrading control, which is the opposite of\n"
        "  the QoS the abstract claims.\n"
        "  At each 125 ms edge the regulator releases the newest sample it\n"
        "  holds (slide 8), so the age column is how stale that sample is\n"
        "  when it leaves: 0 for a sensor whose samples land on the edge.\n");

    fdt_plot_t p;
    fdt_plot_init(&p, "Sampling continues; publication is decimated",
                  "sensor rate (Hz, log axis by value)", "rate (Hz)");
    fdt_plot_series(&p, "sensor sampling", xs, sampled_hz, NRATES,
                    FDT_PLOT_LINE);
    fdt_plot_series(&p, "published to MQTT", xs, published_hz, NRATES,
                    FDT_PLOT_LINE);
    fdt_plot_hline(&p, DT_HZ, "DT rate, 8 Hz (Sec. III)");
    fdt_plot_set_log_y(&p, 1);
    fdt_plot_write_svg(&p, "results/regulator.svg");
    fdt_plot_write_csv(&p, "results/regulator.csv");

    fdt_plot_t s;
    fdt_plot_init(&s, "Share of each sensor's traffic the regulator removes",
                  "sensor rate (Hz)", "traffic removed (%)");
    fdt_plot_series(&s, "saved", xs, saved_pct, NRATES, FDT_PLOT_BAR);
    fdt_plot_write_svg(&s, "results/regulator_saved.svg");

    bench_artefacts(&r, "regulator");
    bench_say(&r, "artefacts: results/regulator_saved.svg\n");
    bench_close(&r);
    return EXIT_SUCCESS;
}
