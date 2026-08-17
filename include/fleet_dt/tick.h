#ifndef FLEET_DT_TICK_H
#define FLEET_DT_TICK_H

#include <stdint.h>
#include <time.h>

/**
 * @file tick.h
 * @brief The 125 ms frame pacer of Sections III and IV.
 *
 * @note Requires POSIX.1-2008 clock_nanosleep() with TIMER_ABSTIME, which
 *       glibc provides and Apple libc does not.
 */

/** The frame period of the paper, in nanoseconds: 125 ms, i.e. 8 Hz. */
#define FDT_TICK_NS 125000000L

/**
 * @brief An absolute-deadline pacer.
 */
typedef struct {
    struct timespec next;     /**< Absolute deadline of the next frame. */
    long            period;   /**< Frame period in nanoseconds, positive. */
    uint64_t        overruns; /**< Frames whose work exceeded the period. */
} fdt_tick_t;

/**
 * @brief Arms the pacer and sets the first deadline one period from now.
 * @param period_ns  Frame period in nanoseconds. A value of zero or less is
 *                   replaced by ::FDT_TICK_NS, so a miswired caller runs at
 *                   the paper's rate rather than spinning.
 */
void fdt_tick_start(fdt_tick_t *tk, long period_ns);

/**
 * @brief Sleeps until the next deadline, then advances it by one period.
 */
void fdt_tick_wait(fdt_tick_t *tk);

/**
 * @brief Frames that missed their deadline since fdt_tick_start().
 * @return The count, or 0 for a NULL pacer.
 */
uint64_t fdt_tick_overruns(const fdt_tick_t *tk);

#endif /* FLEET_DT_TICK_H */
