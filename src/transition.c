#include <fleet_dt/transition.h>

const fdt_state_t *fdt_window_at(const fdt_queue_t *q, size_t n, size_t k)
{
    const size_t len = fdt_queue_len(q);

    if (n == 0 || n > len || k >= n) {
        return NULL;
    }
    return fdt_queue_at(q, len - 1 - k);
}

int fdt_twin_init(fdt_twin_t *tw, fdt_state_t *q_storage, size_t cap,
                  fdt_delta_e_fn delta_e, fdt_pi_fn pi, void *ctx)
{
    if (tw == NULL || delta_e == NULL || pi == NULL) {
        return -1;
    }
    if (fdt_queue_init(&tw->q, q_storage, cap) != 0) {
        return -1;
    }
    tw->delta_e = delta_e;
    tw->pi      = pi;
    tw->ctx     = ctx;
    return 0;
}

/**
 * The projection of A^t_i out of B^t_i for a non-autonomous vessel.
 */
static void absorbed_pi(const fdt_state_t *b, const fdt_goal_t *g_now,
                        fdt_actuation_t *out, void *ctx, void *fleet_ctx)
{
    (void)g_now;
    (void)ctx;
    (void)fleet_ctx;

    out->throttle_pct = b->surge_mps;
    out->cage_rad     = b->yaw_rate_rps;
}

int fdt_twin_init_passive(fdt_twin_t *tw, fdt_state_t *q_storage, size_t cap,
                          fdt_delta_e_fn delta_e, void *ctx)
{
    return fdt_twin_init(tw, q_storage, cap, delta_e, absorbed_pi, ctx);
}

int fdt_twin_seed(fdt_twin_t *tw, const fdt_state_t *b)
{
    if (tw == NULL || tw->delta_e == NULL || tw->pi == NULL || b == NULL) {
        return -1;
    }
    if (fdt_queue_cap(&tw->q) == 0) {
        return -1;
    }

    /* B^1_i, the boundary condition equation (3)'s recurrence starts from. */
    fdt_queue_push(&tw->q, b);
    return 0;
}

int fdt_twin_step_ctx(fdt_twin_t *tw, const fdt_input_t *in,
                      const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                      size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out,
                      void *fleet_ctx)
{
    if (tw == NULL || tw->delta_e == NULL || tw->pi == NULL) {
        return -1;
    }
    if (g_prev == NULL || g_now == NULL || b_out == NULL || a_out == NULL) {
        return -1;
    }
    if (n == 0 || n > fdt_queue_len(&tw->q)) {
        return -1;
    }

    fdt_state_t b = {0};
    tw->delta_e(&tw->q, n, in, g_prev, &b, tw->ctx, fleet_ctx);

    fdt_queue_push(&tw->q, &b);

    /* Equation (2), second half, on the state just produced, under g^t_i. */
    fdt_actuation_t a = {0};
    tw->pi(&b, g_now, &a, tw->ctx, fleet_ctx);

    *b_out = b;
    *a_out = a;
    return 0;
}

int fdt_twin_step(fdt_twin_t *tw, const fdt_input_t *in,
                  const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                  size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out)
{
    return fdt_twin_step_ctx(tw, in, g_prev, g_now, n, b_out, a_out, NULL);
}

size_t fdt_twin_depth(const fdt_twin_t *tw)
{
    return (tw == NULL) ? 0 : fdt_queue_len(&tw->q);
}

const fdt_state_t *fdt_twin_newest(const fdt_twin_t *tw)
{
    return (tw == NULL) ? NULL : fdt_queue_newest(&tw->q);
}
