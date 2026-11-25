#include "injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/regulator.h>

#include <math.h>
#include <stddef.h>

/**
 * xorshift32.
 */
static uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/** A deterministic value in [-1, 1] from a seed. */
static float jitter(uint32_t *s)
{
    return (float)((double)xorshift32(s) / 2147483648.0 - 1.0);
}

int fdt_inj_init(fdt_inj_t *inj, fdt_transport_t *tr, unsigned vessels,
                 double hz, uint32_t seed)
{
    if (inj == NULL || tr == NULL || tr->publish == NULL ||
        vessels == 0 || vessels > FDT_INJ_MAX_VESSELS || !(hz > 0.0)) {
        return -1;
    }

    inj->tr               = tr;
    inj->vessels          = vessels;
    inj->hz               = hz;
    inj->raw              = 0;
    inj->seed             = seed;
    inj->tick             = 0;
    inj->published        = 0;
    inj->refused          = 0;
    inj->missed_deadlines = 0;
    inj->sensor_hz[FDT_SENSOR_IMU]   = FDT_INJ_HZ_IMU;
    inj->sensor_hz[FDT_SENSOR_GPS]   = FDT_INJ_HZ_GPS;
    inj->sensor_hz[FDT_SENSOR_BARO]  = FDT_INJ_HZ_BARO;
    inj->sensor_hz[FDT_SENSOR_POWER] = FDT_INJ_HZ_POWER;
    return fdt_inj_configure(inj, NULL, 0);
}

int fdt_inj_configure(fdt_inj_t *inj, const double *sensor_hz, int raw)
{
    if (inj == NULL || inj->tr == NULL) {
        return -1;
    }
    if (sensor_hz != NULL) {
        for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
            if (!(sensor_hz[s] > 0.0)) {
                return -1;
            }
        }
        for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
            inj->sensor_hz[s] = sensor_hz[s];
        }
    }
    inj->raw = raw;
    for (unsigned v = 0; v < inj->vessels; v++) {
        if (fdt_regpub_init(&inj->pub[v], inj->tr, v, inj->sensor_hz,
                            raw) != 0) {
            return -1;
        }
    }
    return 0;
}

void fdt_inj_sample_at(const fdt_inj_t *inj, unsigned vessel, double t_s,
                       fdt_input_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = (fdt_input_t){0};
    if (inj == NULL || vessel >= inj->vessels) {
        return;
    }

    /* The noise is a function of the sample instant, to the microsecond. */
    const uint32_t us = (uint32_t)(int64_t)llround(t_s * 1e6);
    uint32_t s = inj->seed ^ (vessel * 2654435761u) ^ (us * 0x9E3779B9u);
    if (s == 0) {
        s = 0x6D2B79F5u; /* xorshift32 must not start at zero */
    }

    const double phase = t_s / 20.0 + (double)vessel * 0.37;

    out->ax_mps2 = 0.05f * jitter(&s);
    out->ay_mps2 = 0.05f * jitter(&s);
    out->az_mps2 = -9.80665f + 0.05f * jitter(&s);

    out->wx_rps = 0.01f * jitter(&s);
    out->wy_rps = 0.01f * jitter(&s);
    out->wz_rps = (float)(0.15 + 0.05 * sin(phase)) +
                  0.005f * jitter(&s);

    /* Southern hemisphere magnetic field, roughly 23 uT horizontal. */
    out->mx_ut = 18.0f + 0.5f * jitter(&s);
    out->my_ut = 2.0f + 0.5f * jitter(&s);
    out->mz_ut = -14.0f + 0.5f * jitter(&s);

    /* The lagoons of southern Brazil, where the eDNA campaign samples. */
    out->gps_lat_deg = (float)(-30.05 + FDT_INJ_SWEEP_DEG * sin(phase));
    out->gps_lon_deg = (float)(-51.17 + FDT_INJ_SWEEP_DEG * cos(phase) +
                               FDT_INJ_SPACING_DEG * (double)vessel);
    out->gps_alt_m   = 3.0f + 0.2f * jitter(&s);

    out->vn_mps = (float)(1.2 * cos(phase)) + 0.02f * jitter(&s);
    out->ve_mps = (float)(1.2 * sin(phase)) + 0.02f * jitter(&s);
    out->vd_mps = 0.01f * jitter(&s);

    out->press_pa = 101325.0f + 40.0f * jitter(&s);
    out->temp_c   = 22.0f + 0.5f * jitter(&s);

    /* Section II: an 18.5 V, 10 Ah LiPo, sagging slowly under load. */
    out->vbat_v = (float)(18.5 - 0.0032 * t_s) + 0.02f * jitter(&s);
    out->ibat_a = 6.0f + 0.5f * jitter(&s);

    out->x_left  = FDT_VIEW_PRESENT;
    out->x_right = FDT_VIEW_PRESENT;
}

void fdt_inj_sample(const fdt_inj_t *inj, unsigned vessel, fdt_input_t *out)
{
    const double t = (inj == NULL) ? 0.0 : (double)inj->tick / inj->hz;
    fdt_inj_sample_at(inj, vessel, t, out);
}

/** Offers one sensor's samples of the current frame, oldest first. */
static void offer_frame(fdt_inj_t *inj, unsigned v, fdt_sensor_t s,
                        int *sent)
{
    const double   f     = inj->sensor_hz[s];
    const unsigned m     = fdt_reg_samples_in_tick(f, inj->hz, inj->tick);
    /* Index of the last sample before this frame: floor(k f / hz). */
    const uint64_t first = (uint64_t)floor((double)inj->tick * f / inj->hz +
                                           1e-9) + 1u;

    for (unsigned j = 0; j < m; j++) {
        fdt_input_t in;
        fdt_inj_sample_at(inj, v, (double)(first + j) / f, &in);
        const int rc = fdt_regpub_offer(&inj->pub[v], s, &in);
        if (rc == 1) {
            (*sent)++;
        }
    }
}

int fdt_inj_offer_frame(fdt_inj_t *inj)
{
    if (inj == NULL || inj->tr == NULL || inj->tr->publish == NULL) {
        return -1;
    }

    int sent = 0;
    for (unsigned v = 0; v < inj->vessels; v++) {
        for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
            offer_frame(inj, v, (fdt_sensor_t)s, &sent);
        }
    }
    inj->published += (uint64_t)sent;
    return sent;
}

int fdt_inj_release(fdt_inj_t *inj)
{
    if (inj == NULL || inj->tr == NULL || inj->tr->publish == NULL) {
        return -1;
    }

    int sent = 0;
    uint64_t refused = 0;
    for (unsigned v = 0; v < inj->vessels; v++) {
        const int edge = fdt_regpub_tick(&inj->pub[v]);
        if (edge > 0) {
            sent += edge;
        }
        refused += fdt_regpub_refused(&inj->pub[v]);
    }
    inj->published += (uint64_t)sent;
    inj->refused    = refused;
    inj->tick++;
    return sent;
}

int fdt_inj_tick(fdt_inj_t *inj)
{
    const int offered = fdt_inj_offer_frame(inj);
    if (offered < 0) {
        return -1;
    }
    const int released = fdt_inj_release(inj);
    return (released < 0) ? -1 : offered + released;
}

void fdt_inj_note_missed_deadline(fdt_inj_t *inj)
{
    if (inj != NULL) {
        inj->missed_deadlines++;
    }
}

uint64_t fdt_inj_published(const fdt_inj_t *inj)
{
    return (inj == NULL) ? 0 : inj->published;
}

uint64_t fdt_inj_refused(const fdt_inj_t *inj)
{
    return (inj == NULL) ? 0 : inj->refused;
}

uint64_t fdt_inj_missed_deadlines(const fdt_inj_t *inj)
{
    return (inj == NULL) ? 0 : inj->missed_deadlines;
}

const fdt_regpub_t *fdt_inj_client(const fdt_inj_t *inj, unsigned vessel)
{
    if (inj == NULL || vessel >= inj->vessels) {
        return NULL;
    }
    return &inj->pub[vessel];
}
