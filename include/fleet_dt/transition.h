#ifndef FLEET_DT_TRANSITION_H
#define FLEET_DT_TRANSITION_H

#include <fleet_dt/model.h>
#include <fleet_dt/queue.h>

#include <stddef.h>

/**
 * @file transition.h
 * @brief delta, delta^e and pi — equations (2) and (3) — and the DTI they form.
 */

/**
 * @brief delta^e of equation (3): the extended state transition.
 *
 * @param q          The queue, already holding the history. Never NULL.
 * @param n          Window depth, with 1 <= n <= fdt_queue_len(q). Read the
 *                   window through fdt_window_at(), not through @p q
 *                   directly, so that k counts the way equation (3) does.
 * @param in         I^{t-1}_k. May be NULL when the caller has no fresh
 *                   input, which is the case a dropped or late packet
 *                   produces; an implementation must tolerate it.
 * @param g_prev     g^{t-1}_k, the goal in force at t-1. Never NULL.
 * @param out        B^t_i, written by the implementation. Never NULL, and
 *                   zero-initialised before the call.
 * @param ctx        Per-twin application context, passed through untouched.
 * @param fleet_ctx  c^t of equation (4) when a fleet drives the step, NULL
 *                   when the twin is stepped standalone. Opaque to this
 *                   library: each application chooses its own type.
 */
typedef void (*fdt_delta_e_fn)(const fdt_queue_t *q, size_t n,
                               const fdt_input_t *in, const fdt_goal_t *g_prev,
                               fdt_state_t *out, void *ctx, void *fleet_ctx);

/**
 * @brief pi of equation (2): A^t_i = pi(B^t_i, g^t_i).
 *
 * @param b          B^t_i, the state the transition just produced.
 * @param g_now      g^t_i, the goal in force at t. Not necessarily the
 *                   @c g_prev the transition consumed: on a frame where the
 *                   mission re-targets, the transition still runs under the
 *                   old goal while the decision already decides under the new
 *                   one.
 * @param out        A^t_i, written by the implementation. Never NULL.
 * @param ctx        Per-twin application context.
 * @param fleet_ctx  c^t, exactly as for ::fdt_delta_e_fn.
 */
typedef void (*fdt_pi_fn)(const fdt_state_t *b, const fdt_goal_t *g_now,
                          fdt_actuation_t *out, void *ctx, void *fleet_ctx);

/**
 * @brief Reads the window of equation (3), counting back from the present.
 * @param q  Queue holding the history.
 * @param n  Window depth.
 * @param k  Distance from the present: 0 is B^{t-1}, the newest state, and
 *           n-1 is B^{t-n}, the oldest state inside the window.
 * @return A pointer into the queue's storage, or NULL when @p k is at or past
 *         @p n, when @p n is zero, or when the queue holds fewer than @p n
 *         states.
 *
 * @note The ordering is deliberately the reverse of fdt_queue_at(). The queue
 *       is indexed by absolute age; the window is indexed by distance to the
 *       present, which is how equation (3) writes it. Mixing them up is the
 *       likeliest bug in any delta^e, so the two accessors are named apart.
 */
const fdt_state_t *fdt_window_at(const fdt_queue_t *q, size_t n, size_t k);

/**
 * @brief One Digital Twin Instance: a queue plus the dynamics that drive it.
 */
typedef struct {
    fdt_queue_t    q;       /**< The vessel's state history. */
    fdt_delta_e_fn delta_e; /**< The vessel's delta^e. Never NULL once bound. */
    fdt_pi_fn      pi;      /**< The vessel's pi. Never NULL once bound. */
    void          *ctx;     /**< Per-twin application context, may be NULL. */
} fdt_twin_t;

/**
 * @brief Binds a twin to caller-owned storage and to the application dynamics.
 * @param q_storage  Array of at least @p cap states, outliving the twin.
 * @param cap        Queue capacity in states; bounds the memory at 48*cap.
 * @param delta_e    The vessel's transition. Never NULL.
 * @param pi         The vessel's decision. Never NULL; for a non-autonomous
 *                   vessel use fdt_twin_init_passive() instead.
 * @param ctx        Per-twin context, may be NULL.
 * @return 0 on success, -1 when a required argument is NULL or @p cap is 0.
 */
int fdt_twin_init(fdt_twin_t *tw, fdt_state_t *q_storage, size_t cap,
                  fdt_delta_e_fn delta_e, fdt_pi_fn pi, void *ctx);

/**
 * @brief Binds a non-autonomous twin, where actuation is absorbed by state.
 *
 * @return 0 on success, -1 on the same conditions as fdt_twin_init().
 */
int fdt_twin_init_passive(fdt_twin_t *tw, fdt_state_t *q_storage, size_t cap,
                          fdt_delta_e_fn delta_e, void *ctx);

/**
 * @brief Establishes B^1_i, the known starting state of Section IV.
 *
 * @return 0 on success, -1 when the twin is unbound or @p b is NULL.
 *
 * @note Seeding an already-seeded twin simply appends, which is how a run
 *       that restores a saved history primes its queue.
 */
int fdt_twin_seed(fdt_twin_t *tw, const fdt_state_t *b);

/**
 * @brief One simulation frame with fleet context.
 *
 * @param in         I^{t-1}_k, may be NULL.
 * @param g_prev     g^{t-1}_k, consumed by equation (3).
 * @param g_now      g^t_i, consumed by equation (2). Pass the same goal twice
 *                   when the mission does not re-target.
 * @param n          Window depth, 1 <= n <= fdt_twin_depth(tw).
 * @param b_out      Receives B^t_i. Never NULL.
 * @param a_out      Receives A^t_i. Never NULL.
 * @param fleet_ctx  c^t, or NULL for a standalone twin.
 * @return 0 on success, -1 when the twin is unbound, when a required pointer
 *         is NULL, when @p n is 0, or when @p n exceeds the held state count.
 *         On failure nothing is enqueued and no output is written.
 */
int fdt_twin_step_ctx(fdt_twin_t *tw, const fdt_input_t *in,
                      const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                      size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out,
                      void *fleet_ctx);

/**
 * @brief One simulation frame without fleet context.
 */
int fdt_twin_step(fdt_twin_t *tw, const fdt_input_t *in,
                  const fdt_goal_t *g_prev, const fdt_goal_t *g_now,
                  size_t n, fdt_state_t *b_out, fdt_actuation_t *a_out);

/**
 * @brief States this twin currently holds.
 * @return The count, or 0 for a NULL twin.
 *
 * @note This is not a window size. It is the largest n a step may ask for.
 */
size_t fdt_twin_depth(const fdt_twin_t *tw);

/**
 * @brief The twin's most recent state.
 * @return A pointer into the twin's queue, or NULL when the twin has been
 *         neither seeded nor stepped. Invalidated by the next step.
 */
const fdt_state_t *fdt_twin_newest(const fdt_twin_t *tw);

#endif /* FLEET_DT_TRANSITION_H */
