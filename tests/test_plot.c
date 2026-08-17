/**
 * @file test_plot.c
 * @brief The SVG plotter that turns every benchmark into an artefact.
 *
 * Not a paper claim: this is the machinery the claims are reported through.
 * What is checked here is that a chart is well-formed, self-contained, and
 * carries its reference line — because a chart that silently drops the
 * published figure would make a benchmark look conclusive when it is not.
 */
#include <fleet_dt/plot.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Reads a whole file into a caller-supplied buffer, NUL-terminating it. */
static size_t slurp(const char *path, char *buf, size_t cap)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return 0;
    }
    const size_t n = fread(buf, 1, cap - 1, f);
    buf[n] = '\0';
    fclose(f);
    return n;
}

static void test_init_and_guards(void)
{
    fdt_plot_t p;
    assert(fdt_plot_init(&p, "T", "x", "y") == 0);
    assert(fdt_plot_init(NULL, "T", "x", "y") == -1);
    assert(p.nseries == 0 && p.nrules == 0);
    assert(p.width > 0 && p.height > 0);

    const double xs[3] = { 0.0, 1.0, 2.0 };
    const double ys[3] = { 1.0, 2.0, 3.0 };

    assert(fdt_plot_series(&p, "a", xs, ys, 3, FDT_PLOT_LINE) == 0);
    assert(fdt_plot_series(&p, "b", NULL, ys, 3, FDT_PLOT_LINE) == -1);
    assert(fdt_plot_series(&p, "b", xs, ys, 0, FDT_PLOT_LINE) == -1);
    assert(p.nseries == 1);

    /* The chart fills up rather than overrunning its fixed arrays. */
    for (int i = 1; i < FDT_PLOT_MAX_SERIES; i++) {
        assert(fdt_plot_series(&p, "more", xs, ys, 3, FDT_PLOT_LINE) == 0);
    }
    assert(fdt_plot_series(&p, "overflow", xs, ys, 3, FDT_PLOT_LINE) == -1);

    for (int i = 0; i < FDT_PLOT_MAX_RULES; i++) {
        assert(fdt_plot_hline(&p, 1.0, "rule") == 0);
    }
    assert(fdt_plot_hline(&p, 1.0, "overflow") == -1);

    /* An empty chart is not written: a zero-series SVG would be an empty
     * frame that reads as "measured nothing" rather than "wrote nothing". */
    fdt_plot_t empty;
    assert(fdt_plot_init(&empty, "T", "x", "y") == 0);
    assert(fdt_plot_write_svg(&empty, "results/selftest/should-not-exist.svg") == -1);
    assert(fdt_plot_write_csv(&empty, "results/selftest/should-not-exist.csv") == -1);
}

static void test_svg_output(void)
{
    static double xs[32];
    static double line[32];
    static double bars[32];

    for (int i = 0; i < 32; i++) {
        xs[i]   = (double)i;
        line[i] = 100.0 + (double)(i % 7);
        bars[i] = (double)(i % 5);
    }

    fdt_plot_t p;
    assert(fdt_plot_init(&p, "Frame period & escape <check>",
                         "frame", "period (ms)") == 0);
    assert(fdt_plot_series(&p, "measured", xs, line, 32, FDT_PLOT_LINE) == 0);
    assert(fdt_plot_series(&p, "overruns", xs, bars, 32, FDT_PLOT_BAR) == 0);
    assert(fdt_plot_hline(&p, 125.0, "paper: 125 ms deadline") == 0);

    /* Unit-test output goes to a subdirectory of its own. results/ holds
     * measurements, and synthetic data sitting beside real data is how a
     * reader ends up citing a number that came from a test fixture. */
    assert(fdt_mkdir_p("results") == 0);
    assert(fdt_mkdir_p("results") == 0);  /* idempotent */
    assert(fdt_mkdir_p("results/selftest") == 0);
    assert(fdt_plot_write_svg(&p, "results/selftest/plot.svg") == 0);

    static char buf[65536];
    const size_t n = slurp("results/selftest/plot.svg", buf, sizeof buf);
    assert(n > 0);

    /* Well-formed and self-contained. */
    assert(strncmp(buf, "<svg", 4) == 0);
    assert(strstr(buf, "</svg>") != NULL);
    assert(strstr(buf, "xmlns=\"http://www.w3.org/2000/svg\"") != NULL);
    assert(strstr(buf, "http://") == strstr(buf, "xmlns=\"http://") ||
           strstr(buf, "xlink:href") == NULL);

    /* Readable on either background: the dark-mode rule is present. */
    assert(strstr(buf, "prefers-color-scheme: dark") != NULL);

    /* The reference line and its caption survived. */
    assert(strstr(buf, "paper: 125 ms deadline") != NULL);
    assert(strstr(buf, "class=\"rule\"") != NULL);

    /* Both series were drawn, each in its own form. */
    assert(strstr(buf, "<polyline") != NULL);
    assert(strstr(buf, "<rect") != NULL);
    assert(strstr(buf, "measured") != NULL);

    /* XML metacharacters in a title are escaped, not emitted raw. */
    assert(strstr(buf, "&amp;") != NULL);
    assert(strstr(buf, "&lt;check&gt;") != NULL);
}

static void test_csv_output(void)
{
    static double xs[4]  = { 0.0, 1.0, 2.0, 3.0 };
    static double ya[4]  = { 10.0, 20.0, 30.0, 40.0 };
    static double yb[2]  = { 1.5, 2.5 };

    fdt_plot_t p;
    assert(fdt_plot_init(&p, "csv", "x", "y") == 0);
    assert(fdt_plot_series(&p, "long", xs, ya, 4, FDT_PLOT_LINE) == 0);
    assert(fdt_plot_series(&p, "short", xs, yb, 2, FDT_PLOT_LINE) == 0);

    assert(fdt_mkdir_p("results/selftest") == 0);
    assert(fdt_plot_write_csv(&p, "results/selftest/plot.csv") == 0);

    static char buf[4096];
    assert(slurp("results/selftest/plot.csv", buf, sizeof buf) > 0);

    assert(strncmp(buf, "x,long,short\n", 13) == 0);
    assert(strstr(buf, "0,10,1.5\n") != NULL);
    /* The short series leaves the cell empty rather than writing a zero that
     * would plot as a measurement. */
    assert(strstr(buf, "2,30,\n") != NULL);
    assert(strstr(buf, "3,40,\n") != NULL);
}

static void test_log_axis(void)
{
    static double xs[3] = { 0.0, 1.0, 2.0 };
    static double ys[3] = { 0.001, 1.0, 1000.0 };

    fdt_plot_t p;
    assert(fdt_plot_init(&p, "log", "x", "y") == 0);
    assert(fdt_plot_series(&p, "decades", xs, ys, 3, FDT_PLOT_SCATTER) == 0);
    fdt_plot_set_log_y(&p, 1);
    assert(p.log_y == 1);

    assert(fdt_mkdir_p("results/selftest") == 0);
    assert(fdt_plot_write_svg(&p, "results/selftest/plot_log.svg") == 0);

    static char buf[65536];
    assert(slurp("results/selftest/plot_log.svg", buf, sizeof buf) > 0);
    assert(strstr(buf, "<circle") != NULL);
}

int main(void)
{
    test_init_and_guards();
    test_svg_output();
    test_csv_output();
    test_log_axis();

    printf("wrote results/selftest/plot.svg and .csv\n");
    printf("test_plot: ok\n");
    return 0;
}
