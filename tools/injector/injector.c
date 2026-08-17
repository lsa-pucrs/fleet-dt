#include "injector.h"

#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>

#include <math.h>
#include <stddef.h>

/**
 * xorshift32.
 *
 * Deterministic and seedable, which is the whole requirement: a scaling run
 * has to be comparable against the previous one, and it would not be if the
 * telemetry differed between them.
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
        vessels == 0 || hz <= 0.0) {
        return -1;
    }

    inj->tr               = tr;
    inj->vessels          = vessels;
    inj->hz               = hz;
    inj->seed             = seed;
    inj->tick             = 0;
    inj->seq              = 1; /* seq 0 would read as "nothing seen yet" */
    inj->published        = 0;
    inj->refused          = 0;
    inj->missed_deadlines = 0;
    return 0;
}

void fdt_inj_sample(const fdt_inj_t *inj, unsigned vessel, fdt_input_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = (fdt_input_t){0};
    if (inj == NULL || vessel >= inj->vessels) {
        return;
    }

    uint32_t s = inj->seed ^ (vessel * 2654435761u) ^ inj->tick;

    /* A slow yaw sweep, so a fleet fans out over a run rather than sitting
     * still. Each vessel turns at its own rate. */
    const double phase = (double)inj->tick / (inj->hz * 20.0) +
                         (double)vessel * 0.37;

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
    /* A circle of FDT_INJ_SWEEP_DEG about the sampling point, with the
     * vessels offset from one another by a quarter of it so a fleet is a
     * fleet rather than a stack. */
    out->gps_lat_deg = (float)(-30.05 + FDT_INJ_SWEEP_DEG * sin(phase) +
                               0.25 * FDT_INJ_SWEEP_DEG * (double)vessel);
    out->gps_lon_deg = (float)(-51.17 + FDT_INJ_SWEEP_DEG * cos(phase) +
                               0.25 * FDT_INJ_SWEEP_DEG * (double)vessel);
    out->gps_alt_m   = 3.0f + 0.2f * jitter(&s);

    out->vn_mps = (float)(1.2 * cos(phase)) + 0.02f * jitter(&s);
    out->ve_mps = (float)(1.2 * sin(phase)) + 0.02f * jitter(&s);
    out->vd_mps = 0.01f * jitter(&s);

    out->press_pa = 101325.0f + 40.0f * jitter(&s);
    out->temp_c   = 22.0f + 0.5f * jitter(&s);

    /* Section II: an 18.5 V, 10 Ah LiPo, sagging slowly under load. */
    out->vbat_v = (float)(18.5 - 0.0004 * (double)inj->tick) +
                  0.02f * jitter(&s);
    out->ibat_a = 6.0f + 0.5f * jitter(&s);

    /* The stereo views exist but do not travel this path: Section III moves
     * the camera feed onto RTSP. */
    out->x_left  = FDT_VIEW_PRESENT;
    out->x_right = FDT_VIEW_PRESENT;
}

int fdt_inj_tick(fdt_inj_t *inj)
{
    if (inj == NULL || inj->tr == NULL || inj->tr->publish == NULL) {
        return -1;
    }

    int sent = 0;
    for (unsigned v = 0; v < inj->vessels; v++) {
        fdt_input_t in;
        fdt_inj_sample(inj, v, &in);

        uint8_t payload[FDT_WIRE_INPUT_BYTES];
        if (fdt_enc_input(&in, payload, sizeof payload) < 0) {
            continue;
        }

        const fdt_env_t env = { .magic = FDT_ENV_MAGIC,
                                .vessel = (uint16_t)v,
                                .kind = FDT_ENV_INPUT,
                                .flags = 0,
                                .seq = inj->seq,
                                .payload_len = FDT_WIRE_INPUT_BYTES };

        uint8_t frame[FDT_ENV_HEADER_BYTES + FDT_WIRE_INPUT_BYTES];
        const long n = fdt_env_encode(&env, payload, sizeof payload,
                                      frame, sizeof frame);
        if (n < 0) {
            continue;
        }

        char topic[64];
        if (fdt_topic(topic, sizeof topic, FDT_TOPIC_LSDT, v) != 0) {
            continue;
        }

        if (inj->tr->publish(inj->tr->self, topic, frame, (size_t)n, 1) == 0) {
            inj->published++;
            sent++;
        } else {
            /* The transport refused. Counted rather than retried: a retry
             * would hide the very congestion the benchmark is measuring. */
            inj->refused++;
        }
    }

    inj->tick++;
    inj->seq++;
    return sent;
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
