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
 */

/** Open report: writes to stdout and to results/<name>.txt at once. */
typedef struct {
    FILE *file; /**< The text artefact; may be NULL if it could not open. */
} bench_report_t;

/** How a measurement stands beside the figure the paper publishes. */
typedef enum {
    BENCH_MATCH,   /**< The measurement lands on the published figure. */
    BENCH_HOST,    /**< The value belongs to the machine that ran it. */
    BENCH_EXTERNAL /**< The quantity belongs to a system outside this repo. */
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
 * @brief Prints a measurement beside the figure the paper publishes.
 * @param label     What was measured.
 * @param measured  Formatted measured value.
 * @param paper     What the paper says, verbatim enough to check.
 * @param verdict   How the two stand beside each other.
 */
static inline void bench_compare(bench_report_t *r, const char *label,
                                 const char *measured, const char *paper,
                                 bench_verdict_t verdict)
{
    const char *mark = (verdict == BENCH_MATCH) ? "[matches paper]"
                     : (verdict == BENCH_HOST)  ? "[this host]"
                                                : "[external system]";
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
