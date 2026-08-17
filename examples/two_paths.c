/**
 * @file two_paths.c
 * @brief The filtered state and the raw actuation path of Section V-A.
 *
 * Section V-A, on the single-vessel validation: "Telemetry was collected from
 * a real boat and compared to the pose and attitude estimation, using a
 * Kalman filter to generate B_i^t. Data from sensors (I_i^t) were used
 * unfiltered to achieve the lowest possible latency from sensing to the
 * actuation path."
 *
 * Two paths, one frame, and they are not the same path:
 *
 * - delta^e runs a scalar Kalman filter over yaw, producing the B^t the
 *   operator sees. Filtering costs latency and buys smoothness, which is the
 *   right trade for a display.
 * - pi decides from the raw I^t, stashed in the twin's context, not from the
 *   filtered state. Skipping the filter costs smoothness and buys latency,
 *   which is the right trade for actuation.
 *
 * The run prints both trajectories side by side. The gap between them is
 * lag, not error: the filtered trace is the same motion arriving later.
 *
 * Claim covered: C28 of docs/spec/paper-claims.md.
 */
#include <fleet_dt/tick.h>
#include <fleet_dt/transition.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define QUEUE_CAP 8
#define FRAMES    48
#define RAD2DEG   57.2957795f

/** Per-vessel context: the filter's state, and the raw input pi decides on. */
typedef struct {
    float       estimate_deg;   /**< Kalman posterior for yaw. */
    float       variance;       /**< Posterior variance. */
    float       process_var;    /**< Assumed process noise. */
    float       measure_var;    /**< Assumed measurement noise. */
    fdt_input_t raw;            /**< The unfiltered I^t of this frame. */
    float       raw_yaw_deg;    /**< Yaw integrated from raw input alone. */
} vessel_ctx_t;

/**
 * delta^e with a one-dimensional Kalman filter on yaw.
 *
 * Predict from the previous state and the measured rate, then correct toward
 * the measured heading. The gain is what introduces the lag the operator sees
 * and the actuation path must not inherit.
 */
static void kalman_delta_e(const fdt_queue_t *q, size_t n,
                           const fdt_input_t *in, const fdt_goal_t *g_prev,
                           fdt_state_t *out, void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    (void)fleet_ctx;

    vessel_ctx_t *vc = (vessel_ctx_t *)ctx;
    const fdt_state_t *prev = fdt_window_at(q, n, 0);
    *out = (prev != NULL) ? *prev : (fdt_state_t){0};

    const float dt_s = (float)FDT_TICK_NS / 1e9f;
    const float rate = (in != NULL) ? in->wz_rps : 0.0f;

    /* Predict: carry the estimate forward on the measured rate. Crossing
     * rad/s into a degrees field, as Table I's mixed units require. */
    vc->estimate_deg += rate * dt_s * RAD2DEG;
    vc->variance     += vc->process_var;

    /* Correct: pull toward the heading the magnetometer implies. */
    if (in != NULL) {
        const float measured_deg = atan2f(in->my_ut, in->mx_ut) * RAD2DEG;
        const float gain = vc->variance / (vc->variance + vc->measure_var);
        vc->estimate_deg += gain * (measured_deg - vc->estimate_deg);
        vc->variance     *= (1.0f - gain);
    }

    out->yaw_deg      = vc->estimate_deg;
    out->yaw_rate_rps = rate;

    /* The raw input is stashed here, not consumed: pi reads it next, and the
     * filtered state above never reaches the actuation decision. */
    if (in != NULL) {
        vc->raw = *in;
        vc->raw_yaw_deg += rate * dt_s * RAD2DEG;
    }
}

/**
 * pi deciding from the raw input rather than from the filtered state.
 *
 * This is the half of Section V-A that is easy to get wrong: the obvious
 * implementation reads @p b, which is the filtered state, and quietly adds
 * the filter's lag to the control loop.
 */
static void raw_path_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                        fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)b;
    (void)fleet_ctx;

    const vessel_ctx_t *vc = (const vessel_ctx_t *)ctx;

    /* Raw heading, integrated straight from I^t with no filter in the way. */
    const float err = g_now->yaw_deg - vc->raw_yaw_deg;
    out->cage_rad     = err * 0.01f;
    out->throttle_pct = 55.0f;
}

int main(void)
{
    static fdt_state_t queue[QUEUE_CAP];
    fdt_twin_t twin;
    vessel_ctx_t vc = { .estimate_deg = 0.0f,
                        .variance     = 1.0f,
                        .process_var  = 0.05f,
                        .measure_var  = 4.0f,
                        .raw          = {0},
                        .raw_yaw_deg  = 0.0f };

    if (fdt_twin_init(&twin, queue, QUEUE_CAP,
                      kalman_delta_e, raw_path_pi, &vc) != 0) {
        fprintf(stderr, "twin failed to init\n");
        return EXIT_FAILURE;
    }

    const fdt_state_t b0 = {0};
    if (fdt_twin_seed(&twin, &b0) != 0) {
        fprintf(stderr, "twin failed to seed\n");
        return EXIT_FAILURE;
    }

    const fdt_goal_t goal = { .yaw_deg = 90.0f };

    printf("%-5s %12s %12s %12s %10s\n",
           "k", "raw yaw", "filtered", "lag (deg)", "cage");
    printf("%-5s %12s %12s %12s %10s\n",
           "", "(pi path)", "(operator)", "", "");

    double worst_lag = 0.0;

    for (int frame = 0; frame < FRAMES; frame++) {
        /* Synthetic telemetry with measurement noise in the magnetometer, so
         * the filter has something to smooth. */
        const double t = (double)frame;
        const float turn_rate = 0.20f;
        const float heading   = (float)(turn_rate * t *
                                        (double)FDT_TICK_NS / 1e9) * RAD2DEG;
        const float noise = (float)(2.5 * sin(t * 2.7) + 1.5 * sin(t * 0.9));

        fdt_input_t in = {0};
        in.wz_rps = turn_rate;
        in.mx_ut  = cosf((heading + noise) / RAD2DEG) * 23.0f;
        in.my_ut  = sinf((heading + noise) / RAD2DEG) * 23.0f;

        fdt_state_t b;
        fdt_actuation_t a;
        if (fdt_twin_step(&twin, &in, &goal, &goal, 1, &b, &a) != 0) {
            fprintf(stderr, "step failed at frame %d\n", frame);
            return EXIT_FAILURE;
        }

        const double lag = (double)vc.raw_yaw_deg - (double)b.yaw_deg;
        if (fabs(lag) > worst_lag) {
            worst_lag = fabs(lag);
        }

        if (frame % 4 == 0) {
            printf("%-5d %12.3f %12.3f %12.3f %10.4f\n",
                   frame, (double)vc.raw_yaw_deg, (double)b.yaw_deg,
                   lag, (double)a.cage_rad);
        }
    }

    printf("\nworst lag between the two paths: %.3f deg\n", worst_lag);
    printf("The filtered trace is what the operator sees; the actuation "
           "decision\nnever waited for it. Section V-A keeps the same two "
           "paths apart.\n");

    return EXIT_SUCCESS;
}
