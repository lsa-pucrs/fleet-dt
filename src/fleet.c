#include <fleet_dt/fleet.h>

#include <string.h>

int fdt_fleet_init(fdt_fleet_t *f, fdt_twin_t *twins, size_t n, void *ctx)
{
    if (f == NULL || twins == NULL || n == 0) {
        return -1;
    }
    f->twins = twins;
    f->n     = n;
    f->ctx   = ctx;
    f->ctx_ring  = NULL;
    f->ctx_cap   = 0;
    f->ctx_bytes = 0;
    f->ctx_count = 0;
    f->ctx_head  = 0;
    return 0;
}

int fdt_fleet_keep_history(fdt_fleet_t *f, void *storage, size_t cap,
                           size_t ctx_bytes)
{
    if (f == NULL || storage == NULL || cap == 0 || ctx_bytes == 0 ||
        f->ctx == NULL) {
        return -1;
    }
    f->ctx_ring  = (unsigned char *)storage;
    f->ctx_cap   = cap;
    f->ctx_bytes = ctx_bytes;
    f->ctx_count = 0;
    f->ctx_head  = 0;
    return 0;
}

int fdt_fleet_record_ctx(fdt_fleet_t *f)
{
    if (f == NULL || f->ctx_ring == NULL || f->ctx == NULL) {
        return -1;
    }
    memcpy(f->ctx_ring + f->ctx_head * f->ctx_bytes, f->ctx, f->ctx_bytes);
    f->ctx_head = (f->ctx_head + 1) % f->ctx_cap;
    if (f->ctx_count < f->ctx_cap) {
        f->ctx_count++;
    }
    return 0;
}

const void *fdt_fleet_ctx_at(const fdt_fleet_t *f, size_t n, size_t k)
{
    if (f == NULL || f->ctx_ring == NULL || k >= n || k >= f->ctx_count) {
        return NULL;
    }
    const size_t idx = (f->ctx_head + f->ctx_cap - 1 - k) % f->ctx_cap;
    return f->ctx_ring + idx * f->ctx_bytes;
}

const fdt_state_t *fdt_fleet_state_at(const fdt_fleet_t *f, size_t n,
                                      size_t k, size_t v)
{
    if (f == NULL || f->twins == NULL || v >= f->n || k >= n) {
        return NULL;
    }
    const fdt_twin_t *tw = &f->twins[v];
    const size_t depth = fdt_twin_depth(tw);
    if (k >= depth) {
        return NULL;
    }
    return fdt_window_at(&tw->q, depth, k);
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
