#include <fleet_dt/dte.h>

#include <stddef.h>

int fdt_dte_init(fdt_dte_t *dte, fdt_sim_slot_t *slots, size_t cap)
{
    if (dte == NULL || slots == NULL || cap == 0) {
        return -1;
    }
    dte->slots = slots;
    dte->cap   = cap;
    dte->n     = 0;
    dte->tick  = 0;
    return 0;
}

int fdt_dte_register(fdt_dte_t *dte, const char *name, fdt_sim_fn fn,
                     void *user)
{
    if (dte == NULL || dte->slots == NULL || name == NULL || fn == NULL) {
        return -1;
    }
    if (dte->n >= dte->cap) {
        return -1;
    }
    dte->slots[dte->n].name = name;
    dte->slots[dte->n].fn   = fn;
    dte->slots[dte->n].user = user;
    dte->n++;
    return 0;
}

size_t fdt_dte_tick(fdt_dte_t *dte)
{
    if (dte == NULL || dte->slots == NULL) {
        return (size_t)-1;
    }

    const size_t t = dte->tick;

    /* One index, every simulation. This is the whole of feature (ii): the
     * fluid solver and the obstacle detector observe the instant the twin
     * observes, rather than each drifting on a clock of its own. */
    for (size_t i = 0; i < dte->n; i++) {
        dte->slots[i].fn(t, dte->slots[i].user);
    }

    dte->tick++;
    return t;
}

size_t fdt_dte_count(const fdt_dte_t *dte)
{
    return (dte == NULL) ? 0 : dte->n;
}

size_t fdt_dte_ticks(const fdt_dte_t *dte)
{
    return (dte == NULL) ? 0 : dte->tick;
}

const char *fdt_dte_name(const fdt_dte_t *dte, size_t i)
{
    if (dte == NULL || dte->slots == NULL || i >= dte->n) {
        return NULL;
    }
    return dte->slots[i].name;
}
