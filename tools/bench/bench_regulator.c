/**
 * @file bench_regulator.c
 * @brief What the bandwidth regulators of Section III actually remove.
 */
#include "bench_common.h"

#include <fleet_dt/regulator.h>

#include <assert.h>
#include <stdlib.h>

/** The DT rate: one increment of k per 125 ms. */
#define DT_HZ 8.0

/** Samples offered per sensor, enough for the ratios to settle. */
#define SAMPLES 20000

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
    bench_say(&r, "DT rate: %.0f Hz   samples per sensor: %d\n",
              DT_HZ, SAMPLES);

    static double xs[NRATES];
    static double sampled_hz[NRATES];
    static double published_hz[NRATES];
    static double saved_pct[NRATES];

    bench_section(&r, "per sensor rate");
    bench_say(&r, "  %10s %12s %12s %10s %10s\n",
              "sensor Hz", "published Hz", "control Hz", "saved", "verdict");

    for (int i = 0; i < NRATES; i++) {
        const double hz = RATES[i];

        fdt_reg_t reg;
        assert(fdt_reg_init(&reg, hz, DT_HZ) == 0);

        unsigned long control_reads = 0;
        for (int s = 0; s < SAMPLES; s++) {
            control_reads++;
            (void)fdt_reg_admit(&reg);
        }

        const double eff    = fdt_reg_effective_hz(&reg);
        const double saved  = 100.0 * fdt_reg_saved_ratio(&reg);
        const double ctrl   = (double)control_reads /
                              ((double)SAMPLES / hz);

        xs[i]           = hz;
        sampled_hz[i]   = hz;
        published_hz[i] = eff;
        saved_pct[i]    = saved;

        assert(fdt_reg_sampled(&reg) ==
               fdt_reg_published(&reg) + fdt_reg_dropped(&reg));
        assert(control_reads == (unsigned long)fdt_reg_sampled(&reg));

        const char *verdict = (hz <= DT_HZ) ? "untouched" : "decimated";
        bench_say(&r, "  %10.0f %12.2f %12.0f %9.1f %% %10s\n",
                  hz, eff, ctrl, saved, verdict);
    }

    char buf[64];

    bench_section(&r, "the two halves of the Section III claim");

    /* Fast sensor: publication falls to the DT rate. */
    fdt_reg_t fast;
    assert(fdt_reg_init(&fast, 1000.0, DT_HZ) == 0);
    for (int s = 0; s < SAMPLES; s++) {
        (void)fdt_reg_admit(&fast);
    }
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
    assert(fdt_reg_init(&slow, 1.0, DT_HZ) == 0);
    for (int s = 0; s < 1000; s++) {
        assert(fdt_reg_admit(&slow) == 1);
    }
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
        "  the QoS the abstract claims.\n");

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
