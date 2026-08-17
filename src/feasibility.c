#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/feasibility.h>

#include <stddef.h>

#define NS_PER_S 1000000000L

void fdt_feas_init(fdt_feas_t *fs, long budget_ns)
{
    if (fs == NULL) {
        return;
    }
    fs->budget_ns     = (budget_ns > 0) ? budget_ns : FDT_TICK_NS;
    fs->begin.tv_sec  = 0;
    fs->begin.tv_nsec = 0;
    fs->last_ns       = 0;
    fs->worst_ns      = 0;
    fs->total_ns      = 0;
    fs->frames        = 0;
    fs->violations    = 0;
}

void fdt_feas_begin(fdt_feas_t *fs)
{
    if (fs == NULL) {
        return;
    }
    clock_gettime(CLOCK_MONOTONIC, &fs->begin);
}

long fdt_feas_end(fdt_feas_t *fs)
{
    if (fs == NULL) {
        return -1;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    const long ns = (long)(now.tv_sec - fs->begin.tv_sec) * NS_PER_S +
                    (now.tv_nsec - fs->begin.tv_nsec);

    fs->last_ns   = ns;
    fs->total_ns += ns;
    fs->frames++;

    if (ns > fs->worst_ns) {
        fs->worst_ns = ns;
    }
    /* "in less than |t_k - t_{k-1}|": meeting the budget exactly is already
     * outside the strict inequality the paper writes. */
    if (ns >= fs->budget_ns) {
        fs->violations++;
    }
    return ns;
}

int fdt_feas_ok(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : (fs->violations == 0);
}

uint64_t fdt_feas_violations(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->violations;
}

long fdt_feas_worst_ns(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->worst_ns;
}

long fdt_feas_last_ns(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->last_ns;
}

double fdt_feas_mean_ns(const fdt_feas_t *fs)
{
    if (fs == NULL || fs->frames == 0) {
        return 0.0;
    }
    return (double)fs->total_ns / (double)fs->frames;
}

uint64_t fdt_feas_frames(const fdt_feas_t *fs)
{
    return (fs == NULL) ? 0 : fs->frames;
}
