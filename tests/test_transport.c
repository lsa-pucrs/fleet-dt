/**
 * @file test_transport.c
 * @brief The network seam and its in-process implementation.
 *
 * Claim covered in part: C2 (the architecture relies on MQTT) — this is the
 * interface the MQTT adapter implements. See docs/spec/paper-claims.md.
 */
#include <fleet_dt/transport.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>

/** Counts deliveries and remembers the last payload byte. */
typedef struct {
    unsigned hits;
    uint8_t  last;
} sink_t;

static void on_msg(const char *topic, const uint8_t *buf, size_t len,
                   void *user)
{
    (void)topic;

    sink_t *s = (sink_t *)user;
    s->hits++;
    s->last = (len > 0) ? buf[0] : 0;
}

/** The topic layout, and its refusal to overflow. */
static void test_topic_layout(void)
{
    char t[64];

    assert(fdt_topic(t, sizeof t, FDT_TOPIC_STATE, 0) == 0);
    assert(strcmp(t, "fleet/0/state") == 0);

    assert(fdt_topic(t, sizeof t, FDT_TOPIC_LSDT, 12) == 0);
    assert(strcmp(t, "fleet/12/lsdt") == 0);

    assert(fdt_topic(t, sizeof t, FDT_TOPIC_ACT, 7) == 0);
    assert(strcmp(t, "fleet/7/act") == 0);

    assert(fdt_topic(t, sizeof t, FDT_TOPIC_GOAL, 3) == 0);
    assert(strcmp(t, "fleet/3/goal") == 0);

    /* A truncated topic would silently address the wrong vessel. */
    assert(fdt_topic(t, 4, FDT_TOPIC_STATE, 0) == -1);
    assert(fdt_topic(NULL, sizeof t, FDT_TOPIC_STATE, 0) == -1);
    assert(fdt_topic(t, sizeof t, NULL, 0) == -1);
}

/** Publishing enqueues; polling delivers. The split is the point. */
static void test_publish_then_poll(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    assert(tr.self != NULL && tr.publish != NULL && tr.subscribe != NULL);
    assert(tr.poll != NULL && tr.close != NULL);

    sink_t s0 = {0};
    sink_t s1 = {0};
    assert(tr.subscribe(tr.self, "fleet/0/state", on_msg, &s0) == 0);
    assert(tr.subscribe(tr.self, "fleet/1/state", on_msg, &s1) == 0);

    uint8_t payload[1] = { 0xAB };
    assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);

    /* Nothing has been delivered yet. This is what lets a benchmark hold a
     * packet across a frame boundary on purpose. */
    assert(s0.hits == 0);

    assert(tr.poll(tr.self, 0) == 1);
    assert(s0.hits == 1 && s0.last == 0xAB);
    assert(s1.hits == 0);              /* exact topic match, no wildcard */

    /* The queue drains: a second poll does not redeliver. */
    assert(tr.poll(tr.self, 0) == 0);
    assert(s0.hits == 1);

    tr.close(tr.self);
}

/** Fan-out, unheard topics, and the full queue. */
static void test_delivery_semantics(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    uint8_t payload[1] = { 0x01 };

    sink_t a = {0};
    sink_t b = {0};
    assert(tr.subscribe(tr.self, "fleet/0/state", on_msg, &a) == 0);
    assert(tr.subscribe(tr.self, "fleet/0/state", on_msg, &b) == 0);

    /* Two subscribers, one message: the poll count is messages delivered,
     * not callbacks fired, so it stays comparable across topologies. */
    assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);
    assert(tr.poll(tr.self, 0) == 1);
    assert(a.hits == 1 && b.hits == 1);

    /* A topic nobody listens to is publishable and simply goes nowhere. */
    assert(tr.publish(tr.self, "fleet/9/act", payload, 1, 1) == 0);
    assert(tr.poll(tr.self, 0) == 1);

    /* The queue reports being full instead of dropping quietly: a silent
     * drop here would be indistinguishable from the packet loss the
     * benchmarks exist to measure. */
    for (size_t i = 0; i < FDT_LOOP_QUEUE; i++) {
        assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);
    }
    assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == -1);
    assert(tr.poll(tr.self, 0) == FDT_LOOP_QUEUE);

    /* Oversized payloads and topics are refused rather than truncated. */
    uint8_t big[FDT_LOOP_PAYLOAD + 1] = {0};
    assert(tr.publish(tr.self, "fleet/0/state", big, sizeof big, 1) == -1);

    char long_topic[FDT_LOOP_TOPIC + 8];
    memset(long_topic, 'x', sizeof long_topic - 1);
    long_topic[sizeof long_topic - 1] = '\0';
    assert(tr.publish(tr.self, long_topic, payload, 1, 1) == -1);
    assert(tr.subscribe(tr.self, long_topic, on_msg, &a) == -1);

    tr.close(tr.self);
}

/** Every entry point tolerates a NULL transport. */
static void test_null_safety(void)
{
    fdt_transport_t none = fdt_loop_transport(NULL);
    assert(none.self == NULL && none.publish == NULL);

    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    uint8_t payload[1] = { 0x02 };

    assert(tr.publish(NULL, "t", payload, 1, 1) == -1);
    assert(tr.subscribe(NULL, "t", on_msg, NULL) == -1);
    assert(tr.poll(NULL, 0) == -1);
    tr.close(NULL);

    assert(tr.publish(tr.self, NULL, payload, 1, 1) == -1);
    assert(tr.subscribe(tr.self, "t", NULL, NULL) == -1);
}

/**
 * The lossy modes, without which no benchmark can see the Section V-B
 * pathology: a lossless link never produces a partial or a doubled frame.
 */
static void test_loss_injection(void)
{
    fdt_loop_t loop;
    fdt_transport_t tr = fdt_loop_transport(&loop);
    uint8_t payload[1] = { 0x07 };
    sink_t s = {0};

    assert(tr.subscribe(tr.self, "fleet/0/state", on_msg, &s) == 0);

    /* Drop every second publish. The sender is told it succeeded, because on
     * a real link it did: the loss happened downstream. */
    fdt_loop_set_loss(&loop, 2, 0);
    for (int i = 0; i < 10; i++) {
        assert(tr.publish(tr.self, "fleet/0/state", payload, 1, 1) == 0);
    }
    assert(tr.poll(tr.self, 0) == 5);
    assert(s.hits == 5);
    assert(fdt_loop_dropped(&loop) == 5);

    /* Defer every second publish by one poll: it lands in the frame after
     * the one it belonged to, beside that frame's own message, which is the
     * arrival Section V-B describes. */
    fdt_loop_t d;
    fdt_transport_t td = fdt_loop_transport(&d);
    sink_t sd = {0};
    assert(td.subscribe(td.self, "fleet/0/state", on_msg, &sd) == 0);
    fdt_loop_set_loss(&d, 0, 2);

    assert(td.publish(td.self, "fleet/0/state", payload, 1, 1) == 0);
    assert(td.publish(td.self, "fleet/0/state", payload, 1, 1) == 0);

    /* First poll: only the first message. The second is still in flight. */
    assert(td.poll(td.self, 0) == 1);
    assert(sd.hits == 1);
    assert(fdt_loop_deferred(&d) == 1);

    /* Second poll: the late one arrives, in the wrong frame. */
    assert(td.poll(td.self, 0) == 1);
    assert(sd.hits == 2);

    /* Zero on both restores a lossless link. */
    fdt_loop_set_loss(&d, 0, 0);
    assert(td.publish(td.self, "fleet/0/state", payload, 1, 1) == 0);
    assert(td.poll(td.self, 0) == 1);
    assert(sd.hits == 3);

    fdt_loop_set_loss(NULL, 1, 1);
    assert(fdt_loop_dropped(NULL) == 0);
    assert(fdt_loop_deferred(NULL) == 0);
}

int main(void)
{
    test_topic_layout();
    test_publish_then_poll();
    test_delivery_semantics();
    test_null_safety();
    test_loss_injection();

    printf("test_transport: ok\n");
    return 0;
}
