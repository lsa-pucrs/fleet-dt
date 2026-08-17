#ifndef FLEET_DT_BENCH_COMMON_H
#define FLEET_DT_BENCH_COMMON_H

#include <fleet_dt/plot.h>
#include <fleet_dt/version.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/**
 * @file bench_common.h
 * @brief Shared reporting for the measurement campaign.
 *
 * Every benchmark writes the same three artefacts into results/:
 *
 * - `<name>.txt`  the human report, also echoed to stdout;
 * - `<name>.csv`  the raw series, so a reader can re-plot or re-analyse;
 * - `<name>.svg`  the chart, with the paper's figure drawn as a rule.
 *
 * The verdict helper is the important part. A benchmark measures a machine,
 * and a machine is not the machine the paper measured, so a divergence from a
 * published number is information rather than failure. Divergence is labelled
 * and printed; it never aborts a run. What would abort a run is a structural
 * error — a vessel lost, a counter that does not balance — and those are
 * plain assertions in the benchmark itself.
 */

/** Open report: writes to stdout and to results/<name>.txt at once. */
typedef struct {
    FILE *file; /**< The text artefact; may be NULL if it could not open. */
} bench_report_t;

/** Verdict of comparing a measurement against a figure from the paper. */
typedef enum {
    BENCH_OK,      /**< The measurement agrees with the paper. */
    BENCH_DIVERGE, /**< It does not; this is reported, not fatal. */
    BENCH_NA       /**< Not measurable here; the artefact lives elsewhere. */
} bench_verdict_t;

/**
 * @brief Opens a report, creating results/ and printing a header.
 * @param name  Artefact stem, e.g. "scale".
 */
static inline void bench_open(bench_report_t *r, const char *name)
{
    char path[256];

    fdt_mkdir_p("results");
    snprintf(path, sizeof path, "results/%s.txt", name);
    r->file = fopen(path, "w");
}

/** @brief Prints one line to stdout and to the text artefact. */
static inline void bench_say(bench_report_t *r, const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    if (r->file != NULL) {
        va_start(ap, fmt);
        vfprintf(r->file, fmt, ap);
        va_end(ap);
    }
}

/** @brief Prints a section heading. */
static inline void bench_section(bench_report_t *r, const char *title)
{
    bench_say(r, "\n--- %s ---\n", title);
}

/**
 * @brief Prints a measurement beside the paper's figure and a verdict.
 * @param label     What was measured.
 * @param measured  Formatted measured value.
 * @param paper     What the paper says, verbatim enough to check.
 * @param verdict   Whether they agree.
 */
static inline void bench_compare(bench_report_t *r, const char *label,
                                 const char *measured, const char *paper,
                                 bench_verdict_t verdict)
{
    const char *mark = (verdict == BENCH_OK)      ? "[OK]"
                     : (verdict == BENCH_DIVERGE) ? "[DIVERGE]"
                                                  : "[BOUNDARY]";
    bench_say(r, "  %-26s %-22s paper: %-34s %s\n",
              label, measured, paper, mark);
}

/** @brief Closes the report. */
static inline void bench_close(bench_report_t *r)
{
    if (r->file != NULL) {
        fclose(r->file);
        r->file = NULL;
    }
}

/** @brief Prints the banner every benchmark starts with. */
static inline void bench_banner(bench_report_t *r, const char *title)
{
    bench_say(r, "== %s ==\n", title);
    bench_say(r, "%s\n", fdt_version());
}

/** @brief Notes where the artefacts went. */
static inline void bench_artefacts(bench_report_t *r, const char *name)
{
    bench_say(r, "\nartefacts: results/%s.txt, results/%s.csv, "
                 "results/%s.svg\n", name, name, name);
}

#endif /* FLEET_DT_BENCH_COMMON_H */
