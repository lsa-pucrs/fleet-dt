/**
 * @file test_transition.c
 * @brief Equations (2) and (3): the window, the seed, the goal split.
 */
#include <fleet_dt/transition.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Records the depth the last call received, so the test can pin it. */
static size_t g_seen_n;

/**
 * A probe delta^e: integrates the input's yaw rate, and reports both ends of
 * the window so the test can check the ordering equation (3) implies.
 */
static void probe_delta_e(const fdt_queue_t *q, size_t n,
                          const fdt_input_t *in, const fdt_goal_t *g_prev,
                          fdt_state_t *out, void *ctx, void *fleet_ctx)
{
    (void)g_prev;
    (void)ctx;
    (void)fleet_ctx;

    g_seen_n = n;

    /* k == 0 is the newest state of the window: the B^{t-1} of the bracket. */
    const fdt_state_t *newest = fdt_window_at(q, n, 0);
    *out = (newest != NULL) ? *newest : (fdt_state_t){0};
    out->yaw_deg += (in != NULL) ? in->wz_rps : 0.0f;

    /* k == n-1 is the oldest state of the window: the B^{t-n}. */
    const fdt_state_t *oldest = fdt_window_at(q, n, n - 1);
    out->pitch_deg = (oldest != NULL) ? oldest->yaw_deg : -1.0f;

    /* One past the window is out of the window, whatever the queue holds. */
    assert(fdt_window_at(q, n, n) == NULL);
}

/**
 * A probe pi. It reads fleet_ctx as a throttle ceiling; test_fleet.c reads the
 * same argument as a different struct, which is intentional, c^t is opaque to
 * the library and each application picks its own type.
 */
static void probe_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                     fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)ctx;

    const float *ceiling = (const float *)fleet_ctx;
    out->cage_rad     = g_now->yaw_deg - b->yaw_deg;
    out->throttle_pct = (ceiling != NULL) ? *ceiling : 100.0f;
}

/** The window accessor counts back from the present, not forward from age. */
static void test_window_ordering(void)
{
    fdt_state_t storage[4];
    fdt_queue_t q;
    assert(fdt_queue_init(&q, storage, 4) == 0);

    for (int k = 0; k < 4; k++) {
        fdt_state_t b = { .yaw_deg = (float)k };
        fdt_queue_push(&q, &b);
    }

    assert(fdt_window_at(&q, 4, 0)->yaw_deg == 3.0f);
    assert(fdt_window_at(&q, 4, 3)->yaw_deg == 0.0f);
    assert(fdt_queue_at(&q, 0)->yaw_deg == 0.0f);

    /* A narrower window still ends at the present. */
    assert(fdt_window_at(&q, 2, 0)->yaw_deg == 3.0f);
    assert(fdt_window_at(&q, 2, 1)->yaw_deg == 2.0f);

    /* Out of range in every direction. */
    assert(fdt_window_at(&q, 2, 2) == NULL);
    assert(fdt_window_at(&q, 0, 0) == NULL);
    assert(fdt_window_at(&q, 5, 0) == NULL);
    assert(fdt_window_at(NULL, 1, 0) == NULL);
}

/** Binding guards, and the seed that equation (3) presupposes. */
static void test_init_and_seed(void)
{
    fdt_state_t storage[8];
    fdt_twin_t  tw;
    fdt_state_t b;
    fdt_actuation_t a;
    fdt_goal_t  g  = { .yaw_deg = 10.0f };
    fdt_input_t in = { .wz_rps = 1.0f };

    assert(fdt_twin_init(&tw, storage, 8, probe_delta_e, probe_pi, NULL) == 0);
    assert(fdt_twin_init(&tw, storage, 8, NULL, probe_pi, NULL) == -1);
    assert(fdt_twin_init(&tw, storage, 8, probe_delta_e, NULL, NULL) == -1);
    assert(fdt_twin_init(&tw, storage, 0, probe_delta_e, probe_pi, NULL) == -1);

    /* Rebind, since the failing calls left the twin unusable. */
    assert(fdt_twin_init(&tw, storage, 8, probe_delta_e, probe_pi, NULL) == 0);

    /* Without a seed the recurrence has no B^{t-1}: every n fails. */
    assert(fdt_twin_depth(&tw) == 0);
    assert(fdt_twin_newest(&tw) == NULL);
    assert(fdt_twin_step(&tw, &in, &g, &g, 1, &b, &a) == -1);

    fdt_state_t b0 = {0};
    assert(fdt_twin_seed(&tw, &b0) == 0);
    assert(fdt_twin_seed(&tw, NULL) == -1);
    assert(fdt_twin_depth(&tw) == 1);
    assert(fdt_twin_newest(&tw) != NULL);
}

/** Equation (2) is the n == 1 case; equation (3) is every larger n. */
static void test_step_windows(void)
{
    fdt_state_t storage[8];
    fdt_twin_t  tw;
    fdt_state_t b;
    fdt_actuation_t a;
    fdt_goal_t  g  = { .yaw_deg = 10.0f };
    fdt_input_t in = { .wz_rps = 1.0f };
    fdt_state_t b0 = {0};

    assert(fdt_twin_init(&tw, storage, 8, probe_delta_e, probe_pi, NULL) == 0);
    assert(fdt_twin_seed(&tw, &b0) == 0);

    /* Zero is not a window, and a window wider than the history is rejected. */
    assert(fdt_twin_step(&tw, &in, &g, &g, 0, &b, &a) == -1);
    assert(fdt_twin_step(&tw, &in, &g, &g, 2, &b, &a) == -1);

    /* Equation (2). */
    assert(fdt_twin_step(&tw, &in, &g, &g, 1, &b, &a) == 0);
    assert(g_seen_n == 1);
    assert(fabsf(b.yaw_deg - 1.0f) < 1e-6f);
    assert(fdt_twin_depth(&tw) == 2);

    assert(fdt_twin_step(&tw, &in, &g, &g, 2, &b, &a) == 0);
    assert(g_seen_n == 2);
    assert(fabsf(b.pitch_deg - 0.0f) < 1e-6f);
    assert(fabsf(b.yaw_deg - 2.0f) < 1e-6f);

    /* A rejected step enqueues nothing and writes no output. */
    const size_t before = fdt_twin_depth(&tw);
    const float  keep   = b.yaw_deg;
    assert(fdt_twin_step(&tw, &in, &g, &g, 999, &b, &a) == -1);
    assert(fdt_twin_depth(&tw) == before);
    assert(b.yaw_deg == keep);

    /* A NULL input is the dropped-packet case, and must not be fatal. */
    assert(fdt_twin_step(&tw, NULL, &g, &g, 1, &b, &a) == 0);
    assert(fabsf(b.yaw_deg - 2.0f) < 1e-6f);
}

/** Equation (3) consumes g^{t-1}; equation (2) consumes g^t. */
static void test_goal_split(void)
{
    fdt_state_t storage[8];
    fdt_twin_t  tw;
    fdt_state_t b;
    fdt_actuation_t a;
    fdt_input_t in = { .wz_rps = 1.0f };
    fdt_state_t b0 = {0};

    assert(fdt_twin_init(&tw, storage, 8, probe_delta_e, probe_pi, NULL) == 0);
    assert(fdt_twin_seed(&tw, &b0) == 0);

    fdt_goal_t g_prev = { .yaw_deg = 0.0f };
    fdt_goal_t g_now  = { .yaw_deg = 90.0f };
    assert(fdt_twin_step(&tw, &in, &g_prev, &g_now, 1, &b, &a) == 0);
    assert(fabsf(a.cage_rad - (90.0f - b.yaw_deg)) < 1e-6f);

    float ceiling = 60.0f;
    assert(fdt_twin_step_ctx(&tw, &in, &g_now, &g_now, 1, &b, &a,
                             &ceiling) == 0);
    assert(fabsf(a.throttle_pct - 60.0f) < 1e-6f);

    assert(fdt_twin_step(&tw, &in, &g_now, &g_now, 1, &b, &a) == 0);
    assert(fabsf(a.throttle_pct - 100.0f) < 1e-6f);
}

/** C16: a vessel with no decision function, where A is read out of B. */
static void test_non_autonomous(void)
{
    fdt_state_t storage[4];
    fdt_twin_t  passive;
    fdt_state_t b;
    fdt_actuation_t a;
    fdt_goal_t  g = { .yaw_deg = 10.0f };
    fdt_input_t in = {0};

    assert(fdt_twin_init_passive(&passive, storage, 4,
                                 probe_delta_e, NULL) == 0);

    fdt_state_t b0 = { .surge_mps = 2.0f, .yaw_rate_rps = 0.5f };
    assert(fdt_twin_seed(&passive, &b0) == 0);

    assert(fdt_twin_step(&passive, &in, &g, &g, 1, &b, &a) == 0);
    assert(fabsf(a.throttle_pct - b.surge_mps) < 1e-6f);
    assert(fabsf(a.cage_rad - b.yaw_rate_rps) < 1e-6f);
}

int main(void)
{
    test_window_ordering();
    test_init_and_seed();
    test_step_windows();
    test_goal_split();
    test_non_autonomous();

    printf("test_transition: ok\n");
    return 0;
}
