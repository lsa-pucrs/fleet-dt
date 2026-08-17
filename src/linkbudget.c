#include <fleet_dt/linkbudget.h>

#include <limits.h>
#include <stddef.h>

double fdt_link_stream_bps(const fdt_stream_t *s)
{
    if (s == NULL) {
        return 0.0;
    }
    return (double)s->payload_bytes * 8.0 * s->hz * (double)s->vessels;
}

double fdt_link_total_bps(const fdt_link_t *link,
                          const fdt_stream_t *streams, size_t n)
{
    if (link == NULL || streams == NULL) {
        return 0.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < n; i++) {
        sum += fdt_link_stream_bps(&streams[i]);
    }
    return sum * (1.0 + link->overhead_ratio);
}

double fdt_link_utilization(const fdt_link_t *link,
                            const fdt_stream_t *streams, size_t n)
{
    if (link == NULL || link->capacity_bps <= 0.0) {
        return -1.0;
    }
    return fdt_link_total_bps(link, streams, n) / link->capacity_bps;
}

double fdt_link_increase(const fdt_link_t *link,
                         const fdt_stream_t *baseline, size_t nb,
                         const fdt_stream_t *added, size_t na)
{
    if (link == NULL) {
        return -1.0;
    }

    const double base = fdt_link_total_bps(link, baseline, nb);
    if (base <= 0.0) {
        return -1.0;
    }
    return fdt_link_total_bps(link, added, na) / base;
}

unsigned fdt_link_max_vessels(const fdt_link_t *link,
                              const fdt_stream_t *streams, size_t n,
                              double budget)
{
    if (link == NULL || streams == NULL ||
        link->capacity_bps <= 0.0 || budget <= 0.0) {
        return 0;
    }

    /* Cost of exactly one vessel, whatever vessel count the profile carries. */
    double per_vessel = 0.0;
    for (size_t i = 0; i < n; i++) {
        const fdt_stream_t one = { .name          = streams[i].name,
                                   .payload_bytes = streams[i].payload_bytes,
                                   .hz            = streams[i].hz,
                                   .vessels       = 1 };
        per_vessel += fdt_link_stream_bps(&one);
    }
    per_vessel *= (1.0 + link->overhead_ratio);

    if (per_vessel <= 0.0) {
        return UINT_MAX;
    }

    const double allowed = link->capacity_bps * budget;
    const double fits    = allowed / per_vessel;
    if (fits >= (double)UINT_MAX) {
        return UINT_MAX;
    }
    return (unsigned)fits;
}
