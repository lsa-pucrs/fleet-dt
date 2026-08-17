#ifndef FLEET_DT_FEASIBILITY_H
#define FLEET_DT_FEASIBILITY_H

#include <fleet_dt/tick.h>

#include <stdint.h>

/**
 * @file feasibility.h
 * @brief The feasibility predicate of Section IV, measured per vessel.
 *
 * Section IV: "The DTI is feasible only if delta can be computed in less than
 * |t_k - t_{k-1}| for any arbitrary k; otherwise, it is merely a time-bounded
 * simulation model."
 *
 * Two quantities the paper keeps apart, and so does this type:
 *
 * 1. The time to compute delta, measured here.
 * 2. The latency of delivering the actuation, which Section V-A observes to
 *    be high even when delta is feasible, "as it has to travel back through
 *    the network".
 *
 * This monitor says nothing about the second. Reporting them as one number
 * would misrepresent the paper; tools/bench/bench_latency.c measures them as
 * two.
 *
 * "For any arbitrary k" is a worst case, not an average: a single violation
 * makes the DTI infeasible, and no later fast frame redeems it.
 */
typedef struct {
    long            budget_ns;  /**< |t_k - t_{k-1}|, strictly positive. */
    struct timespec begin;      /**< Start of the delta currently timed. */
    long            last_ns;    /**< Duration of the last delta measured. */
    long            worst_ns;   /**< Worst duration observed so far. */
    long            total_ns;   /**< Sum of durations, for the mean. */
    uint64_t        frames;     /**< Deltas measured. */
    uint64_t        violations; /**< Deltas that met or exceeded the budget. */
} fdt_feas_t;

/**
 * @brief Arms the monitor.
 * @param budget_ns  The frame budget. Zero or less is replaced by
 *                   ::FDT_TICK_NS, the paper's 125 ms deadline.
 */
void fdt_feas_init(fdt_feas_t *fs, long budget_ns);

/** @brief Marks the start of one delta. */
void fdt_feas_begin(fdt_feas_t *fs);

/**
 * @brief Marks the end of one delta and folds it into the statistics.
 * @return The duration in nanoseconds, or -1 when @p fs is NULL.
 */
long fdt_feas_end(fdt_feas_t *fs);

/**
 * @brief Whether the DTI is feasible in the sense of Section IV.
 * @return 1 when no measured delta reached the budget, 0 otherwise. Vacuously
 *         1 before the first frame; 0 for a NULL monitor.
 */
int fdt_feas_ok(const fdt_feas_t *fs);

/** @brief Deltas that met or exceeded the budget. */
uint64_t fdt_feas_violations(const fdt_feas_t *fs);

/** @brief Worst delta duration observed, in nanoseconds. */
long fdt_feas_worst_ns(const fdt_feas_t *fs);

/** @brief Duration of the most recent delta, in nanoseconds. */
long fdt_feas_last_ns(const fdt_feas_t *fs);

/**
 * @brief Mean delta duration, in nanoseconds.
 * @return The mean, or 0.0 before the first frame.
 */
double fdt_feas_mean_ns(const fdt_feas_t *fs);

/** @brief Deltas measured. */
uint64_t fdt_feas_frames(const fdt_feas_t *fs);

#endif /* FLEET_DT_FEASIBILITY_H */
