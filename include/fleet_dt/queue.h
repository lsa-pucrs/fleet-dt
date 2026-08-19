#ifndef FLEET_DT_QUEUE_H
#define FLEET_DT_QUEUE_H

#include <fleet_dt/model.h>

#include <stddef.h>

/**
 * @file queue.h
 * @brief The bounded queue of state frames of Section IV.
 */

/**
 * @brief A ring buffer of states, oldest to newest.
 */
typedef struct {
    fdt_state_t *buf;  /**< Caller-owned array of @c cap entries. */
    size_t       cap;  /**< Capacity in states, strictly positive. */
    size_t       len;  /**< States currently held, at most @c cap. */
    size_t       head; /**< Index into @c buf of the oldest held state. */
} fdt_queue_t;

/**
 * @brief Binds a queue to caller-owned storage.
 * @param q        Queue to initialise.
 * @param storage  Array of at least @p cap states, owned by the caller and
 *                 required to outlive @p q.
 * @param cap      Capacity in states; must be strictly positive.
 * @return 0 on success, -1 when any argument is NULL or @p cap is zero.
 */
int fdt_queue_init(fdt_queue_t *q, fdt_state_t *storage, size_t cap);

/**
 * @brief Appends a state, dropping the oldest one when already at capacity.
 * @param q  Queue to push into; a NULL or unbound queue is a silent no-op.
 * @param b  State to copy in; NULL is a silent no-op.
 */
void fdt_queue_push(fdt_queue_t *q, const fdt_state_t *b);

/**
 * @brief States currently held.
 * @return The count, or 0 for a NULL queue.
 */
size_t fdt_queue_len(const fdt_queue_t *q);

/**
 * @brief Capacity in states.
 * @return The capacity, or 0 for a NULL queue.
 */
size_t fdt_queue_cap(const fdt_queue_t *q);

/**
 * @brief The state at a logical index, counting from the oldest.
 * @param q  Queue to read.
 * @param i  Logical index; 0 is the OLDEST held state.
 * @return A pointer into the queue's storage, or NULL when @p i is at or past
 *         the held count. The pointer is invalidated by the next push.
 *
 * @note Absolute-age indexing. The window accessor of equation (3) counts the
 *       other way, from the present backwards; see fdt_window_at().
 */
const fdt_state_t *fdt_queue_at(const fdt_queue_t *q, size_t i);

/**
 * @brief The most recent state.
 * @return A pointer into the queue's storage, or NULL when the queue is
 *         empty. Invalidated by the next push.
 */
const fdt_state_t *fdt_queue_newest(const fdt_queue_t *q);

/**
 * @brief Bytes a queue of the given capacity occupies.
 * @param cap  Capacity in states.
 * @return @p cap times the size of one state, the 48d of Section IV, which
 *         is how the paper bounds queue growth per vessel.
 */
size_t fdt_queue_bytes(size_t cap);

#endif /* FLEET_DT_QUEUE_H */
