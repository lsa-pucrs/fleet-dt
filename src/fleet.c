#include <fleet_dt/fleet.h>

int fdt_fleet_init(fdt_fleet_t *f, fdt_twin_t *twins, size_t n, void *ctx)
{
    if (f == NULL || twins == NULL || n == 0) {
        return -1;
    }
    f->twins = twins;
    f->n     = n;
    f->ctx   = ctx;
    return 0;
}

int fdt_fleet_step(fdt_fleet_t *f, const fdt_input_t *ins,
                   const fdt_goal_t *goals_prev, const fdt_goal_t *goals_now,
                   size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out)
{
    if (f == NULL || f->twins == NULL || f->n == 0) {
        return -1;
    }
    if (goals_prev == NULL || goals_now == NULL ||
        b_out == NULL || a_out == NULL || n == 0) {
        return -1;
    }

    /* One window is shared by vessels whose queues may differ in length, so
     * every vessel is checked before any vessel steps. Equation (5) advances
     * the fleet as a single frame; a partially advanced fleet is not a state
     * the model has a name for. */
    for (size_t k = 0; k < f->n; k++) {
        const fdt_twin_t *tw = &f->twins[k];
        if (tw->delta_e == NULL || tw->pi == NULL) {
            return -1;
        }
        if (n > fdt_twin_depth(tw)) {
            return -1;
        }
    }

    for (size_t k = 0; k < f->n; k++) {
        const fdt_input_t *in = (ins != NULL) ? &ins[k] : NULL;
        if (fdt_twin_step_ctx(&f->twins[k], in, &goals_prev[k], &goals_now[k],
                              n, &b_out[k], &a_out[k], f->ctx) != 0) {
            return -1;
        }
    }
    return 0;
}

size_t fdt_fleet_size(const fdt_fleet_t *f)
{
    return (f == NULL) ? 0 : f->n;
}

fdt_twin_t *fdt_fleet_twin(const fdt_fleet_t *f, size_t k)
{
    if (f == NULL || f->twins == NULL || k >= f->n) {
        return NULL;
    }
    return &f->twins[k];
}
