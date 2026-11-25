#include <fleet_dt/regulator.h>

#include <math.h>
#include <stddef.h>

int fdt_reg_init(fdt_reg_t *r, double sensor_hz)
{
    if (r == NULL || !(sensor_hz > 0.0)) {
        return -1;
    }
    r->sensor_hz = sensor_hz;
    r->held      = 0;
    r->sampled   = 0;
    r->published = 0;
    r->dropped   = 0;
    r->empty     = 0;
    return 0;
}

void fdt_reg_offer(fdt_reg_t *r)
{
    if (r == NULL) {
        return;
    }
    r->sampled++;
    if (r->held) {
        r->dropped++;
    }
    r->held = 1;
}

int fdt_reg_capture(fdt_reg_t *r)
{
    if (r == NULL) {
        return 0;
    }
    if (!r->held) {
        r->empty++;
        return 0;
    }
    r->held = 0;
    r->published++;
    return 1;
}

/** Samples taken in (0, k / tick_hz], with a guard against 0.999... */
static uint64_t samples_until(double sensor_hz, double tick_hz, uint64_t k)
{
    return (uint64_t)floor((double)k * sensor_hz / tick_hz + 1e-9);
}

unsigned fdt_reg_samples_in_tick(double sensor_hz, double tick_hz,
                                 uint64_t k)
{
    if (!(sensor_hz > 0.0) || !(tick_hz > 0.0)) {
        return 0;
    }
    return (unsigned)(samples_until(sensor_hz, tick_hz, k + 1) -
                      samples_until(sensor_hz, tick_hz, k));
}

uint64_t fdt_reg_sampled(const fdt_reg_t *r)
{
    return (r == NULL) ? 0 : r->sampled;
}

uint64_t fdt_reg_published(const fdt_reg_t *r)
{
    return (r == NULL) ? 0 : r->published;
}

uint64_t fdt_reg_dropped(const fdt_reg_t *r)
{
    return (r == NULL) ? 0 : r->dropped;
}

uint64_t fdt_reg_empty_ticks(const fdt_reg_t *r)
{
    return (r == NULL) ? 0 : r->empty;
}

double fdt_reg_effective_hz(const fdt_reg_t *r)
{
    if (r == NULL || r->sampled == 0) {
        return 0.0;
    }
    return ((double)r->published / (double)r->sampled) * r->sensor_hz;
}

double fdt_reg_saved_ratio(const fdt_reg_t *r)
{
    if (r == NULL || r->sampled == 0) {
        return 0.0;
    }
    return (double)r->dropped / (double)r->sampled;
}
