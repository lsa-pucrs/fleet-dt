#ifndef FLEET_DT_FLEET_H
#define FLEET_DT_FLEET_H

#include <fleet_dt/transition.h>

#include <stddef.h>

/**
 * @file fleet.h
 * @brief F^t, Delta and Delta^e, equations (4), (5) and (6).
 */

/**
 * @brief F^t of equation (4): a fleet of digital twin instances.
 */
typedef struct {
    fdt_twin_t *twins; /**< Caller-owned array of @c n initialised twins. */
    size_t      n;     /**< Vessels in the fleet, strictly positive. */
    void       *ctx;   /**< c^t, opaque to this library, may be NULL. */

    /* History of c^t for the window [F^{t-1}; F^{t-n}] of equation (6).
     * B^t already has its window in each twin's queue; this ring holds the
     * matching past c^t. Optional: see fdt_fleet_keep_history(). */
    unsigned char *ctx_ring;  /**< Caller-owned, ctx_cap * ctx_bytes. */
    size_t         ctx_cap;   /**< Entries the ring holds. */
    size_t         ctx_bytes; /**< Size of one c^t. */
    size_t         ctx_count; /**< Entries recorded, at most ctx_cap. */
    size_t         ctx_head;  /**< Next entry to write. */
} fdt_fleet_t;

/**
 * @brief Binds a fleet to caller-owned twins.
 * @param twins  Array of @p n twins, each already bound by fdt_twin_init()
 *               or fdt_twin_init_passive(), and outliving the fleet.
 * @param n      Vessel count; must be strictly positive.
 * @param ctx    c^t, may be NULL.
 * @return 0 on success, -1 when @p twins is NULL or @p n is zero.
 */
int fdt_fleet_init(fdt_fleet_t *f, fdt_twin_t *twins, size_t n, void *ctx);

/**
 * @brief The Delta of equation (5), extended per equation (6): one fleet frame.
 *
 * @param ins         Array of @p n inputs, I^{t-1}. May be NULL when no
 *                    vessel has fresh input, in which case every twin sees
 *                    NULL, the all-vessels-starved case.
 * @param goals_prev  Array of @p n goals g^{t-1}_i, consumed by equation (3).
 * @param goals_now   Array of @p n goals g^t_i, consumed by equation (2).
 *                    Pass the same array twice when the mission does not
 *                    re-target.
 * @param n           Window depth of equation (6), shared by every vessel.
 * @param b_out       Array of @p n states, receiving B^t.
 * @param a_out       Array of @p n actuations, receiving A^t.
 * @return 0 when every vessel stepped, -1 on a rejected argument or when any
 *         vessel holds fewer than @p n states. On -1 no vessel steps.
 *
 * @note Every vessel is validated before any vessel steps. Without that pass,
 *       a window vessel 2 cannot satisfy would already have advanced vessels
 *       0 and 1, and equation (5) advances the fleet as one frame rather than
 *       as n independent frames.
 */
int fdt_fleet_step(fdt_fleet_t *f, const fdt_input_t *ins,
                   const fdt_goal_t *goals_prev, const fdt_goal_t *goals_now,
                   size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out);

/**
 * @brief Keeps the last @p cap values of c^t, for equation (6).
 *
 * Equation (6) steps the fleet from the window [F^{t-1}; F^{t-n}], and
 * F^t = [B^t delta^e c^t] (equation 4). Each twin's queue already keeps its
 * B^t; this ring keeps c^t beside them. The coordinator records c^t once
 * per frame (fdt_fleet_record_ctx), after computing it.
 *
 * @param storage    Caller-owned, at least @p cap * @p ctx_bytes bytes.
 * @param ctx_bytes  Size of the c^t that f->ctx points to.
 * @return 0, or -1 on a NULL argument, a zero size, or a fleet without ctx.
 */
int fdt_fleet_keep_history(fdt_fleet_t *f, void *storage, size_t cap,
                           size_t ctx_bytes);

/**
 * @brief Copies the current c^t into the history.
 * @return 0, or -1 when the fleet keeps no history.
 */
int fdt_fleet_record_ctx(fdt_fleet_t *f);

/**
 * @brief c^t inside the fleet window: k = 0 the newest recorded, k = n-1
 *        the oldest the window of depth @p n reaches.
 * @return A pointer into the history, or NULL when @p k >= @p n or the
 *         history holds fewer than k + 1 entries.
 */
const void *fdt_fleet_ctx_at(const fdt_fleet_t *f, size_t n, size_t k);

/**
 * @brief B^t of vessel @p v inside the fleet window: k = 0 the newest state
 *        its twin holds, k = n-1 the oldest within depth @p n.
 * @return A pointer into the twin's queue, or NULL when out of range.
 */
const fdt_state_t *fdt_fleet_state_at(const fdt_fleet_t *f, size_t n,
                                      size_t k, size_t v);

/**
 * @brief Vessels in the fleet.
 * @return The count, or 0 for a NULL fleet.
 */
size_t fdt_fleet_size(const fdt_fleet_t *f);

/**
 * @brief The twin at an index.
 * @return A pointer into the fleet's twin array, or NULL when @p k is at or
 *         past the fleet size.
 */
fdt_twin_t *fdt_fleet_twin(const fdt_fleet_t *f, size_t k);

#endif /* FLEET_DT_FLEET_H */
