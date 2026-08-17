#include <fleet_dt/transport.h>

#include <stdio.h>
#include <string.h>

int fdt_topic(char *buf, size_t cap, const char *kind, unsigned vessel)
{
    if (buf == NULL || kind == NULL || cap == 0) {
        return -1;
    }

    const int n = snprintf(buf, cap, "fleet/%u/%s", vessel, kind);
    if (n < 0 || (size_t)n >= cap) {
        return -1;
    }
    return 0;
}

static int loop_publish(void *self, const char *topic, const uint8_t *buf,
                        size_t len, int qos)
{
    (void)qos; /* The loopback has one delivery guarantee: exactly once. */

    fdt_loop_t *lo = (fdt_loop_t *)self;
    if (lo == NULL || topic == NULL || (buf == NULL && len > 0)) {
        return -1;
    }
    if (lo->qlen >= FDT_LOOP_QUEUE || len > FDT_LOOP_PAYLOAD) {
        return -1;
    }
    if (strlen(topic) >= FDT_LOOP_TOPIC) {
        return -1;
    }

    lo->published++;

    /* A dropped publish reports success. On a real link it did succeed: the
     * loss happened downstream, and the boat has no way to know. Returning
     * -1 here would model a different failure, one the sender can react to. */
    if (lo->drop_every != 0 && (lo->published % lo->drop_every) == 0) {
        lo->dropped++;
        return 0;
    }

    const int hold = (lo->defer_every != 0 &&
                      (lo->published % lo->defer_every) == 0);
    if (hold) {
        lo->deferred++;
    }

    snprintf(lo->q[lo->qlen].topic, FDT_LOOP_TOPIC, "%s", topic);
    if (len > 0) {
        memcpy(lo->q[lo->qlen].payload, buf, len);
    }
    lo->q[lo->qlen].len      = len;
    lo->q[lo->qlen].deferred = hold;
    lo->qlen++;
    return 0;
}

static int loop_subscribe(void *self, const char *topic, fdt_on_msg_fn cb,
                          void *user)
{
    fdt_loop_t *lo = (fdt_loop_t *)self;
    if (lo == NULL || topic == NULL || cb == NULL) {
        return -1;
    }
    if (lo->nsubs >= FDT_LOOP_SUBS || strlen(topic) >= FDT_LOOP_TOPIC) {
        return -1;
    }

    snprintf(lo->subs[lo->nsubs].topic, FDT_LOOP_TOPIC, "%s", topic);
    lo->subs[lo->nsubs].cb   = cb;
    lo->subs[lo->nsubs].user = user;
    lo->nsubs++;
    return 0;
}

static int loop_poll(void *self, int timeout_ms)
{
    (void)timeout_ms; /* Nothing to wait for: delivery is local. */

    fdt_loop_t *lo = (fdt_loop_t *)self;
    if (lo == NULL) {
        return -1;
    }

    const size_t pending = lo->qlen;
    size_t       kept    = 0;
    int          delivered = 0;

    for (size_t i = 0; i < pending; i++) {
        if (lo->q[i].deferred) {
            /* Still in the network stack. Clear the flag and compact it to
             * the front, so the next poll delivers it -- inside the frame
             * after the one it belonged to, which is exactly the arrival
             * Section V-B describes. */
            lo->q[i].deferred = 0;
            if (kept != i) {
                lo->q[kept] = lo->q[i];
            }
            kept++;
            continue;
        }

        delivered++;
        for (size_t s = 0; s < lo->nsubs; s++) {
            /* Exact match. Section III's data path uses no wildcard; the
             * bridge does, and that lives in the broker configuration. */
            if (strcmp(lo->subs[s].topic, lo->q[i].topic) == 0) {
                lo->subs[s].cb(lo->q[i].topic, lo->q[i].payload,
                               lo->q[i].len, lo->subs[s].user);
            }
        }
    }

    /* Drained whether or not anyone was listening: an unsubscribed topic is
     * published to the void, which is what a real broker does too. Anything
     * deferred stays queued for the next call. */
    lo->qlen = kept;
    return delivered;
}

static void loop_close(void *self)
{
    fdt_loop_t *lo = (fdt_loop_t *)self;
    if (lo != NULL) {
        memset(lo, 0, sizeof *lo);
    }
}

void fdt_loop_set_loss(fdt_loop_t *lo, unsigned drop_every,
                       unsigned defer_every)
{
    if (lo == NULL) {
        return;
    }
    lo->drop_every  = drop_every;
    lo->defer_every = defer_every;
}

uint64_t fdt_loop_dropped(const fdt_loop_t *lo)
{
    return (lo == NULL) ? 0 : lo->dropped;
}

uint64_t fdt_loop_deferred(const fdt_loop_t *lo)
{
    return (lo == NULL) ? 0 : lo->deferred;
}

fdt_transport_t fdt_loop_transport(fdt_loop_t *lo)
{
    fdt_transport_t tr = { NULL, NULL, NULL, NULL, NULL };
    if (lo == NULL) {
        return tr;
    }

    memset(lo, 0, sizeof *lo);
    tr.publish   = loop_publish;
    tr.subscribe = loop_subscribe;
    tr.poll      = loop_poll;
    tr.close     = loop_close;
    tr.self      = lo;
    return tr;
}
