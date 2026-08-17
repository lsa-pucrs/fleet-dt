/**
 * @file test_feasibility.c
 * @brief The feasibility predicate of Section IV, and what it does not measure.
 */
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/feasibility.h>

#include <assert.h>
#include <stdio.h>
#include <time.h>

/** Burns wall-clock time without burning CPU. */
static void burn_ms(long ms)
{
    struct timespec ts = { .tv_sec  = ms / 1000,
                           .tv_nsec = (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/** A fresh monitor is vacuously feasible and reports nothing. */
static void test_initial_state(void)
{
    fdt_feas_t fs;
    fdt_feas_init(&fs, FDT_TICK_NS);

    assert(fs.budget_ns == FDT_TICK_NS);
    assert(fdt_feas_ok(&fs) == 1);
    assert(fdt_feas_frames(&fs) == 0);
    assert(fdt_feas_violations(&fs) == 0);
    assert(fdt_feas_worst_ns(&fs) == 0);
    assert(fdt_feas_mean_ns(&fs) == 0.0);

    /* An invalid budget falls back to the paper's deadline. */
    fdt_feas_t dflt;
    fdt_feas_init(&dflt, 0);
    assert(dflt.budget_ns == FDT_TICK_NS);

    /* Every accessor is total. */
    assert(fdt_feas_ok(NULL) == 0);
    assert(fdt_feas_end(NULL) == -1);
}

/** A delta well inside the budget keeps the DTI feasible. */
static void test_within_budget(void)
{
    fdt_feas_t fs;
    fdt_feas_init(&fs, FDT_TICK_NS);

    fdt_feas_begin(&fs);
    burn_ms(5);
    const long d = fdt_feas_end(&fs);

    assert(d > 0);
    assert(fdt_feas_frames(&fs) == 1);
    assert(fdt_feas_ok(&fs) == 1);
    assert(fdt_feas_violations(&fs) == 0);
    assert(fdt_feas_worst_ns(&fs) == d);
    assert(fdt_feas_last_ns(&fs) == d);
    assert(fdt_feas_mean_ns(&fs) == (double)d);

    printf("delta worst: %.3f ms, budget %.3f ms\n",
           (double)fdt_feas_worst_ns(&fs) / 1e6,
           (double)FDT_TICK_NS / 1e6);
}

/**
 * "For any arbitrary k" is a worst case.
 */
static void test_worst_case_not_average(void)
{
    fdt_feas_t tight;
    fdt_feas_init(&tight, 1000000L); /* a 1 ms budget */

    fdt_feas_begin(&tight);
    burn_ms(10);
    fdt_feas_end(&tight);

    assert(fdt_feas_ok(&tight) == 0);
    assert(fdt_feas_violations(&tight) == 1);
    assert(fdt_feas_worst_ns(&tight) > 1000000L);

    for (int i = 0; i < 20; i++) {
        fdt_feas_begin(&tight);
        fdt_feas_end(&tight);
    }
    assert(fdt_feas_ok(&tight) == 0);
    assert(fdt_feas_violations(&tight) == 1);
    assert(fdt_feas_frames(&tight) == 21);

    assert(fdt_feas_mean_ns(&tight) < (double)fdt_feas_worst_ns(&tight));
}

int main(void)
{
    test_initial_state();
    test_within_budget();
    test_worst_case_not_average();

    printf("test_feasibility: ok\n");
    return 0;
}
