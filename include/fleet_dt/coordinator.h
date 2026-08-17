#ifndef FLEET_DT_COORDINATOR_H
#define FLEET_DT_COORDINATOR_H

#include <fleet_dt/fleet.h>

#include <stddef.h>

/**
 * @file coordinator.h
 * @brief The B^t store and the coordinator S of Figure 4.
 *
 * Section IV: "The coordinator (S) processes goals (g) received from the MCS
 * and subsequently distributes g^t_i per vessel DTI... The coordinator S
 * computes c^t from the vessel states it receives, in the same step in which
 * it distributes g^t_i."
 *
 * Figure 4 draws both: a cylinder holding B^t, fed by the newly generated
 * states, and a box S that reads it and emits goals. This header is those two
 * boxes.
 */

/**
 * @brief The B^t database of Figure 4: one slot per vessel, latest state only.
 *
 * Figure 4 shows newly generated states "sent to the MCS and stored in a
 * database". The coordinator only ever needs the most recent state of each
 * vessel to compute c^t, so a slot per vessel is the whole structure; the
 * per-vessel history lives in each twin's queue, bounded at 48d.
 *
 * Storage belongs to the caller, as everywhere in this library.
 */
typedef struct {
    fdt_state_t *slots; /**< Caller-owned array of @c n states. */
    size_t       n;     /**< Vessels, strictly positive. */
} fdt_store_t;

/**
 * @brief Binds a store to caller-owned storage and zeroes every slot.
 * @return 0 on success, -1 when @p slots is NULL or @p n is zero.
 */
int fdt_store_init(fdt_store_t *st, fdt_state_t *slots, size_t n);

/**
 * @brief Records the latest state of one vessel.
 * @return 0 on success, -1 when @p k is out of range or @p b is NULL.
 */
int fdt_store_put(fdt_store_t *st, size_t k, const fdt_state_t *b);

/**
 * @brief Reads the latest state of one vessel.
 * @return A pointer into the store, or NULL when @p k is out of range.
 */
const fdt_state_t *fdt_store_get(const fdt_store_t *st, size_t k);

/** @brief Vessels the store covers. */
size_t fdt_store_size(const fdt_store_t *st);

/**
 * @brief Computes c^t from the states the coordinator has received.
 *
 * The application decides what c^t is; the paper's example is inter-vessel
 * contextual information such as distance. Writes through @p fleet_ctx, which
 * is the same pointer every delta^e and every pi will then see.
 *
 * @param bt         The B^t store, already updated with this frame's states.
 * @param fleet_ctx  The fleet's c^t, to be written.
 * @param user       Coordinator-level context, passed through untouched.
 */
typedef void (*fdt_ctx_fn)(const fdt_store_t *bt, void *fleet_ctx, void *user);

/**
 * @brief Distributes g^t_i, seeing the c^t just computed.
 *
 * Section IV places the two in the same step, so this callback runs after
 * ::fdt_ctx_fn and can read its result.
 *
 * @param bt         The B^t store.
 * @param fleet_ctx  The c^t computed earlier in this same step, read-only.
 * @param goals_out  Array of @p n goals to write.
 * @param n          Vessels in the fleet.
 * @param user       Coordinator-level context.
 */
typedef void (*fdt_plan_fn)(const fdt_store_t *bt, const void *fleet_ctx,
                            fdt_goal_t *goals_out, size_t n, void *user);

/** @brief The S of Figure 4: a fleet, its store, and the two callbacks. */
typedef struct {
    fdt_fleet_t *fleet;   /**< The fleet this coordinator drives. */
    fdt_store_t *bt;      /**< The B^t database of Figure 4. */
    fdt_ctx_fn   ctx_fn;  /**< Computes c^t. Never NULL once bound. */
    fdt_plan_fn  plan_fn; /**< Distributes g^t_i. Never NULL once bound. */
    void        *user;    /**< Coordinator-level context, may be NULL. */
} fdt_coord_t;

/**
 * @brief Binds a coordinator.
 * @return 0 on success, -1 when a required argument is NULL or when the store
 *         and the fleet disagree on the vessel count.
 */
int fdt_coord_init(fdt_coord_t *co, fdt_fleet_t *fleet, fdt_store_t *bt,
                   fdt_ctx_fn ctx_fn, fdt_plan_fn plan_fn, void *user);

/**
 * @brief One coordinated frame, in the order Figure 4 draws it.
 *
 * 1. the fleet steps under g^{t-1} and g^t, equations (5) and (6);
 * 2. the resulting B^t enter the store;
 * 3. S computes c^t from the store;
 * 4. S distributes the next g^t, already seeing that c^t.
 *
 * @param ins         Array of vessel inputs, or NULL when none is fresh.
 * @param n           Window depth for this frame.
 * @param goals_prev  Receives this frame's g^t on return, so that the next
 *                    call consumes it as g^{t-1} with no copying by the
 *                    caller. Must hold the fleet's vessel count.
 * @param goals_now   In: this frame's g^t. Out: the next frame's g^t.
 * @param b_out       Array receiving B^t.
 * @param a_out       Array receiving A^t.
 * @return 0 on success, -1 when an argument is NULL or the fleet step fails.
 *         On -1 the store and the goals are left untouched.
 */
int fdt_coord_step(fdt_coord_t *co, const fdt_input_t *ins, size_t n,
                   fdt_goal_t *goals_prev, fdt_goal_t *goals_now,
                   fdt_state_t *b_out, fdt_actuation_t *a_out);

#endif /* FLEET_DT_COORDINATOR_H */
