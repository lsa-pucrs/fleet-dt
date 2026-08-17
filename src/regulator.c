#include <fleet_dt/regulator.h>

#include <stddef.h>

int fdt_reg_init(fdt_reg_t *r, double sensor_hz, double publish_hz)
{
    if (r == NULL || sensor_hz <= 0.0 || publish_hz <= 0.0) {
        return -1;
    }
    r->sensor_hz  = sensor_hz;
    r->publish_hz = publish_hz;

    /* Publication credit each sample earns. A sensor slower than the DT earns
     * a credit of one or more and is therefore never decimated. */
    r->step      = publish_hz / sensor_hz;
    r->acc       = 0.0;
    r->sampled   = 0;
    r->published = 0;
    r->dropped   = 0;
    return 0;
}

int fdt_reg_admit(fdt_reg_t *r)
{
    if (r == NULL) {
        return 0;
    }

    r->sampled++;
    r->acc += r->step;

    if (r->acc >= 1.0) {
        r->acc -= 1.0;
        if (r->acc >= 1.0) {
            /* Sensor slower than the DT: the credit exceeds one, but one
             * sample publishes once. The surplus is discarded rather than
             * carried, because carrying it would eventually emit a phantom
             * publication for a sample the sensor never produced. */
            r->acc = 0.0;
        }
        r->published++;
        return 1;
    }

    r->dropped++;
    return 0;
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
