#ifndef FLEET_DT_FLEET_H
#define FLEET_DT_FLEET_H

#include <fleet_dt/transition.h>

#include <stddef.h>

/**
 * @file fleet.h
 * @brief F^t, Delta and Delta^e — equations (4), (5) and (6).
 *
 * Equation (4) reads F^t = [B^t  delta^e  c^t]:
 *
 * - B^t is the set of vessel states, held inside each twin's queue.
 * - delta^e is the set of extended transitions, held per twin. That is why a
 *   heterogeneous fleet needs no extra machinery: Section IV says "a fleet DT
 *   is homogeneous when delta^e is the same for all vessels. Oppositely,
 *   heterogeneous fleet DTs require indexing delta^e", and indexing here just
 *   means giving the twins different function pointers.
 * - c^t is application-specific fleet context, for instance inter-vessel
 *   distance. Section IV puts its computation in the coordinator, which
 *   derives it from the vessel states it receives in the same step in which
 *   it distributes g^t_i.
 *
 * This library never reads c^t. fdt_fleet_step() hands it to every delta^e
 * and every pi it drives, as their fleet_ctx argument. That is how c^t is
 * both an input and an output of one fleet frame in equation (5): the
 * transition can see it, and the application updates it between frames.
 */

/**
 * @brief F^t of equation (4): a fleet of digital twin instances.
 *
 * Treat the fields as private; use the accessors.
 */
typedef struct {
    fdt_twin_t *twins; /**< Caller-owned array of @c n initialised twins. */
    size_t      n;     /**< Vessels in the fleet, strictly positive. */
    void       *ctx;   /**< c^t, opaque to this library, may be NULL. */
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
 * Steps every vessel with its own input and its own goals, over one shared
 * window depth.
 *
 * @param ins         Array of @p n inputs, I^{t-1}. May be NULL when no
 *                    vessel has fresh input, in which case every twin sees
 *                    NULL — the all-vessels-starved case.
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
