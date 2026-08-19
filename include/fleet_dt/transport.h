#ifndef FLEET_DT_TRANSPORT_H
#define FLEET_DT_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file transport.h
 * @brief The network boundary of Section III, and an in-process loopback.
 */

/** Topic suffix for I^t, the low-speed telemetry of Section III. */
#define FDT_TOPIC_LSDT "lsdt"
/** Topic suffix for B^t, the twin state. */
#define FDT_TOPIC_STATE "state"
/** Topic suffix for A^t, the actuation travelling back to the boat. */
#define FDT_TOPIC_ACT "act"
/** Topic suffix for g^t, the goal coming down from the MCS. */
#define FDT_TOPIC_GOAL "goal"

/**
 * @brief Builds the topic for one vessel and one kind.
 *
 * @param buf     Destination.
 * @param cap     Capacity of @p buf, including the terminator.
 * @param kind    One of the FDT_TOPIC_* suffixes.
 * @param vessel  Vessel index.
 * @return 0 on success, -1 when an argument is NULL or @p buf is too small.
 */
int fdt_topic(char *buf, size_t cap, const char *kind, unsigned vessel);

/**
 * @brief Delivery callback, invoked from fdt_transport_t::poll.
 * @param topic  Topic the message arrived on.
 * @param buf    Payload, valid only for the duration of the call.
 * @param len    Payload length in bytes.
 * @param user   The pointer registered with the subscription.
 */
typedef void (*fdt_on_msg_fn)(const char *topic, const uint8_t *buf,
                              size_t len, void *user);

/**
 * @brief A transport: publish, subscribe, poll, close.
 */
typedef struct {
    /**
     * Enqueues a message for delivery.
     * @return 0 on success, -1 when the queue is full or the payload is too
     *         large. A full queue is reported rather than silently dropping,
     *         because a silent drop here would look exactly like the packet
     *         loss the benchmarks are trying to measure.
     */
    int (*publish)(void *self, const char *topic, const uint8_t *buf,
                   size_t len, int qos);

    /**
     * Registers a callback for one exact topic.
     * @return 0 on success, -1 when no subscription slot is free.
     */
    int (*subscribe)(void *self, const char *topic, fdt_on_msg_fn cb,
                     void *user);

    /**
     * Delivers what has arrived.
     * @param timeout_ms  Hint for implementations that block; the loopback
     *                    ignores it.
     * @return Messages delivered, or -1 on error. A message with several
     *         subscribers counts once.
     */
    int (*poll)(void *self, int timeout_ms);

    /** Releases whatever the implementation holds. */
    void (*close)(void *self);

    /** Implementation state, opaque to the caller. */
    void *self;
} fdt_transport_t;

/** Messages the loopback holds in flight. */
#define FDT_LOOP_QUEUE 64
/** Simultaneous subscriptions the loopback supports. */
#define FDT_LOOP_SUBS 64
/** Longest topic the loopback stores, including the terminator. */
#define FDT_LOOP_TOPIC 64
/** Largest payload the loopback stores. */
#define FDT_LOOP_PAYLOAD 256

/**
 * @brief In-process transport state: no sockets, no broker, no threads.
 */
typedef struct {
    struct {
        char    topic[FDT_LOOP_TOPIC];
        uint8_t payload[FDT_LOOP_PAYLOAD];
        size_t  len;
        int     deferred; /**< Held back for the next poll. */
    } q[FDT_LOOP_QUEUE];
    size_t qlen;

    unsigned drop_every;  /**< Discard every Nth publish; 0 disables. */
    unsigned defer_every; /**< Hold every Nth publish one poll; 0 disables. */
    unsigned published;   /**< Publishes seen, driving the two counters. */
    uint64_t dropped;     /**< Publishes discarded. */
    uint64_t deferred;    /**< Publishes held back at least one poll. */

    struct {
        char          topic[FDT_LOOP_TOPIC];
        fdt_on_msg_fn cb;
        void         *user;
    } subs[FDT_LOOP_SUBS];
    size_t nsubs;
} fdt_loop_t;

/**
 * @brief Zeroes @p lo and returns a transport backed by it.
 * @return A transport whose @c self points at @p lo. On a NULL argument every
 *         member is NULL, which the caller can test.
 */
fdt_transport_t fdt_loop_transport(fdt_loop_t *lo);

/**
 * @brief Makes the loopback lossy, so the Section V-B updates can be seen.
 *
 * @param drop_every   Discard every Nth publish, producing a partial frame.
 *                     The publisher is told the send succeeded, because on a
 *                     real link it did, the loss happened downstream.
 * @param defer_every  Hold every Nth publish back one poll, so it lands in
 *                     the following frame beside that frame's own message and
 *                     produces the double update.
 */
void fdt_loop_set_loss(fdt_loop_t *lo, unsigned drop_every,
                       unsigned defer_every);

/** @brief Publishes the lossy loopback discarded. */
uint64_t fdt_loop_dropped(const fdt_loop_t *lo);

/** @brief Publishes the lossy loopback held back at least one poll. */
uint64_t fdt_loop_deferred(const fdt_loop_t *lo);

#endif /* FLEET_DT_TRANSPORT_H */
