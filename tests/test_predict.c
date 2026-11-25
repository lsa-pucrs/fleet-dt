/**
 * @file test_predict.c
 * @brief The time-series estimator x̂_{k+d} = f(x_k, ..., x_{k-n+1})
 *        (Section IV; talk, backup slide 6).
 */
#include <fleet_dt/predict.h>
#include <fleet_dt/queue.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define CAP 16

static fdt_state_t g_store[CAP];
static fdt_queue_t g_q;

/** Fills the queue with states x(t) for t = 0 .. count-1 ticks. */
static void fill(int count, fdt_state_t (*x)(double t))
{
    assert(fdt_queue_init(&g_q, g_store, CAP) == 0);
    for (int i = 0; i < count; i++) {
        const fdt_state_t s = x((double)i);
        fdt_queue_push(&g_q, &s);
    }
}

/** A boat on a straight course with a constant turn: linear in time. */
static fdt_state_t linear(double t)
{
    fdt_state_t s = {0};
    s.lat_deg   = (float)(-30.05 + 1e-6 * t);  /* ~0.11 m per tick */
    s.lon_deg   = (float)(-51.17 - 2e-6 * t);
    s.surge_mps = (float)(1.0 + 0.1 * t);
    s.yaw_deg   = (float)fmod(170.0 + 4.0 * t, 360.0); /* crosses 180 */
    if (s.yaw_deg > 180.0f) {
        s.yaw_deg -= 360.0f;
    }
    return s;
}

/** Accelerating: quadratic in time. */
static fdt_state_t quadratic(double t)
{
    fdt_state_t s = {0};
    s.alt_m     = (float)(3.0 + 0.5 * t + 0.25 * t * t);
    s.surge_mps = (float)(0.2 * t * t);
    return s;
}

static int near(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

static void test_linear_trend_across_the_wrap(void)
{
    fill(8, linear);
    fdt_state_t est;

    /* Newest state is t = 7; d = 3 ticks ahead is t = 10. */
    assert(fdt_predict(&g_q, 8, 3, 1, &est) == 0);
    const fdt_state_t truth = linear(10.0);
    assert(near(est.surge_mps, truth.surge_mps, 1e-4));
    assert(near(est.yaw_deg, truth.yaw_deg, 1e-3));  /* -150 after the wrap */
    assert(near(est.lat_deg, truth.lat_deg, 2e-7));
    assert(near(est.lon_deg, truth.lon_deg, 2e-7));

    printf("linear: yaw %.1f deg -> %.1f deg three ticks ahead\n",
           (double)linear(7.0).yaw_deg, (double)est.yaw_deg);

    /* Degree 0 holds the mean instead of following the trend. */
    assert(fdt_predict(&g_q, 4, 3, 0, &est) == 0);
    assert(near(est.surge_mps, 1.0 + 0.1 * 5.5, 1e-4));
}

static void test_quadratic(void)
{
    fill(6, quadratic);
    fdt_state_t est;

    assert(fdt_predict(&g_q, 6, 2, 2, &est) == 0);
    const fdt_state_t truth = quadratic(7.0);
    assert(near(est.alt_m, truth.alt_m, 1e-3));
    assert(near(est.surge_mps, truth.surge_mps, 1e-3));

    /* A line through a parabola falls short of it. */
    assert(fdt_predict(&g_q, 6, 2, 1, &est) == 0);
    assert(est.surge_mps < truth.surge_mps - 1.0f);

    /* d = 0 returns the fitted present, which an exact fit reproduces. */
    assert(fdt_predict(&g_q, 6, 0, 2, &est) == 0);
    assert(near(est.alt_m, quadratic(5.0).alt_m, 1e-3));
}

static void test_guards(void)
{
    fill(3, linear);
    fdt_state_t est;

    assert(fdt_predict(NULL, 3, 1, 1, &est) == -1);
    assert(fdt_predict(&g_q, 3, 1, 1, NULL) == -1);
    assert(fdt_predict(&g_q, 3, 1, 3, &est) == -1);    /* degree above 2 */
    assert(fdt_predict(&g_q, 2, 1, 2, &est) == -1);    /* n < degree + 1 */
    assert(fdt_predict(&g_q, 4, 1, 1, &est) == -1);    /* queue too short */
    assert(fdt_predict(&g_q, FDT_PREDICT_MAX_N + 1, 1, 1, &est) == -1);
    assert(fdt_predict(&g_q, 3, 1, 1, &est) == 0);
}

int main(void)
{
    test_linear_trend_across_the_wrap();
    test_quadratic();
    test_guards();

    printf("test_predict: ok\n");
    return 0;
}
