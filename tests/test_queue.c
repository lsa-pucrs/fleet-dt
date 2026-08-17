/**
 * @file test_queue.c
 * @brief Pins the 48d queue bound and the 23 KB per minute of Section IV.
 *
 * Claim covered: C14, second half. See docs/spec/paper-claims.md.
 */
#include <fleet_dt/queue.h>

#include <assert.h>
#include <stdio.h>

/** Rejects malformed bindings before any state is stored. */
static void test_init_guards(void)
{
    fdt_state_t storage[4];
    fdt_queue_t q;

    assert(fdt_queue_init(&q, storage, 4) == 0);
    assert(fdt_queue_init(&q, NULL, 4) == -1);
    assert(fdt_queue_init(&q, storage, 0) == -1);
    assert(fdt_queue_init(NULL, storage, 4) == -1);

    /* An empty queue answers every read with NULL rather than with a stale
     * pointer into uninitialised storage. */
    assert(fdt_queue_len(&q) == 0);
    assert(fdt_queue_cap(&q) == 4);
    assert(fdt_queue_newest(&q) == NULL);
    assert(fdt_queue_at(&q, 0) == NULL);

    /* NULL is answerable, not fatal: the accessors are total. */
    assert(fdt_queue_len(NULL) == 0);
    assert(fdt_queue_cap(NULL) == 0);
    assert(fdt_queue_newest(NULL) == NULL);
}

/** Fills, then overflows, checking that the oldest entry is the one dropped. */
static void test_ring_behaviour(void)
{
    fdt_state_t storage[4];
    fdt_queue_t q;
    assert(fdt_queue_init(&q, storage, 4) == 0);

    for (int k = 0; k < 4; k++) {
        fdt_state_t b = { .yaw_deg = (float)k };
        fdt_queue_push(&q, &b);
    }
    assert(fdt_queue_len(&q) == 4);
    assert(fdt_queue_at(&q, 0)->yaw_deg == 0.0f);  /* index 0 is the oldest */
    assert(fdt_queue_at(&q, 3)->yaw_deg == 3.0f);
    assert(fdt_queue_newest(&q)->yaw_deg == 3.0f);

    /* At capacity a push drops the oldest and the window slides. The length
     * never grows past the capacity, which is what bounds the memory. */
    fdt_state_t b4 = { .yaw_deg = 4.0f };
    fdt_queue_push(&q, &b4);
    assert(fdt_queue_len(&q) == 4);
    assert(fdt_queue_at(&q, 0)->yaw_deg == 1.0f);
    assert(fdt_queue_newest(&q)->yaw_deg == 4.0f);
    assert(fdt_queue_at(&q, 4) == NULL);

    /* Wrapping all the way around leaves the ordering intact. */
    for (int k = 5; k < 20; k++) {
        fdt_state_t b = { .yaw_deg = (float)k };
        fdt_queue_push(&q, &b);
    }
    assert(fdt_queue_len(&q) == 4);
    for (size_t i = 0; i < 4; i++) {
        assert(fdt_queue_at(&q, i)->yaw_deg == (float)(16 + i));
    }

    /* A NULL push is a no-op, not a corruption. */
    fdt_queue_push(&q, NULL);
    assert(fdt_queue_newest(&q)->yaw_deg == 19.0f);
}

/**
 * The two published figures of Section IV.
 *
 * "For a 125 ms period, the queue would grow by 23 KB per minute elapsed,
 * per vessel" is 48 bytes at 8 Hz for 60 seconds.
 */
static void test_published_bounds(void)
{
    for (size_t d = 0; d <= 256; d++) {
        assert(fdt_queue_bytes(d) == 48 * d);
    }

    const size_t frames_per_minute = 8 * 60;
    const size_t per_minute = fdt_queue_bytes(frames_per_minute);
    assert(per_minute == 23040);

    printf("queue storage, depth 4      = %zu bytes\n", fdt_queue_bytes(4));
    printf("queue storage, depth 16     = %zu bytes\n", fdt_queue_bytes(16));
    printf("bytes per minute at 8 Hz    = %zu (paper: 23 KB)\n", per_minute);
}

int main(void)
{
    test_init_guards();
    test_ring_behaviour();
    test_published_bounds();

    printf("test_queue: ok\n");
    return 0;
}
