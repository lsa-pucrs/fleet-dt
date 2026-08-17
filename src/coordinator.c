#include <fleet_dt/coordinator.h>

int fdt_store_init(fdt_store_t *st, fdt_state_t *slots, size_t n)
{
    if (st == NULL || slots == NULL || n == 0) {
        return -1;
    }
    st->slots = slots;
    st->n     = n;
    for (size_t k = 0; k < n; k++) {
        st->slots[k] = (fdt_state_t){0};
    }
    return 0;
}

int fdt_store_put(fdt_store_t *st, size_t k, const fdt_state_t *b)
{
    if (st == NULL || st->slots == NULL || b == NULL || k >= st->n) {
        return -1;
    }
    st->slots[k] = *b;
    return 0;
}

const fdt_state_t *fdt_store_get(const fdt_store_t *st, size_t k)
{
    if (st == NULL || st->slots == NULL || k >= st->n) {
        return NULL;
    }
    return &st->slots[k];
}

size_t fdt_store_size(const fdt_store_t *st)
{
    return (st == NULL) ? 0 : st->n;
}

int fdt_coord_init(fdt_coord_t *co, fdt_fleet_t *fleet, fdt_store_t *bt,
                   fdt_ctx_fn ctx_fn, fdt_plan_fn plan_fn, void *user)
{
    if (co == NULL || fleet == NULL || bt == NULL ||
        ctx_fn == NULL || plan_fn == NULL) {
        return -1;
    }
    if (fdt_fleet_size(fleet) != fdt_store_size(bt)) {
        return -1;
    }
    co->fleet   = fleet;
    co->bt      = bt;
    co->ctx_fn  = ctx_fn;
    co->plan_fn = plan_fn;
    co->user    = user;
    return 0;
}

int fdt_coord_step(fdt_coord_t *co, const fdt_input_t *ins, size_t n,
                   fdt_goal_t *goals_prev, fdt_goal_t *goals_now,
                   fdt_state_t *b_out, fdt_actuation_t *a_out)
{
    if (co == NULL || co->fleet == NULL || co->bt == NULL) {
        return -1;
    }
    if (goals_prev == NULL || goals_now == NULL ||
        b_out == NULL || a_out == NULL) {
        return -1;
    }

    if (fdt_fleet_step(co->fleet, ins, goals_prev, goals_now,
                       n, b_out, a_out) != 0) {
        return -1;
    }

    const size_t vessels = fdt_fleet_size(co->fleet);

    /* The cylinder of Figure 4: the states this frame produced. */
    for (size_t k = 0; k < vessels; k++) {
        fdt_store_put(co->bt, k, &b_out[k]);
    }

    co->ctx_fn(co->bt, co->fleet->ctx, co->user);

    for (size_t k = 0; k < vessels; k++) {
        goals_prev[k] = goals_now[k];
    }
    co->plan_fn(co->bt, co->fleet->ctx, goals_now, vessels, co->user);
    return 0;
}
