#define _POSIX_C_SOURCE 200809L

#include "fdt_mqtt.h"
#include "fdt_mqtt_wire.h"

#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

/** Subscriptions one client may hold. */
#define MQTT_SUBS 64

/** Longest topic filter stored, including the terminator. */
#define MQTT_TOPIC 64

/** Receive buffer: the largest packet the fleet exchanges fits many times. */
#define MQTT_RX 65536

/** Keepalive negotiated in CONNECT, seconds. */
#define MQTT_KEEPALIVE_S 60

/** Milliseconds to wait for CONNACK and SUBACK. */
#define MQTT_ACK_MS 3000

/** Per-connection state behind fdt_transport_t::self. */
typedef struct {
    int      fd;
    uint16_t next_id;
    int      delivered;      /**< Messages delivered in the current poll. */
    int      suback;         /**< Last SUBACK reason, -2 while waiting. */
    uint16_t suback_id;
    double   last_tx;        /**< Monotonic seconds of the last send. */
    uint64_t unacked;        /**< QoS 1 publishes without a PUBACK yet. */

    struct {
        char          topic[MQTT_TOPIC];
        fdt_on_msg_fn cb;
        void         *user;
    } subs[MQTT_SUBS];
    size_t nsubs;

    uint8_t rx[MQTT_RX];
    size_t  rxlen;
    uint8_t tx[MQTT_RX];
} mqtt_state_t;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static int send_all(mqtt_state_t *st, const uint8_t *p, size_t n)
{
    while (n > 0) {
        const ssize_t w = send(st->fd, p, n, MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        p += w;
        n -= (size_t)w;
    }
    st->last_tx = now_s();
    return 0;
}

static uint16_t next_id(mqtt_state_t *st)
{
    st->next_id = (uint16_t)(st->next_id + 1u);
    if (st->next_id == 0) {
        st->next_id = 1;
    }
    return st->next_id;
}

/** Hands one PUBLISH to the matching subscriptions and acknowledges it. */
static int on_publish(mqtt_state_t *st, const fdt_mqtt_packet_t *pk)
{
    fdt_mqtt_publish_t m;
    if (fdt_mqtt_parse_publish(pk, &m) != 0) {
        return -1;
    }
    char topic[256];
    if (m.topic_len >= sizeof topic) {
        return -1;
    }
    memcpy(topic, m.topic, m.topic_len);
    topic[m.topic_len] = '\0';

    int matched = 0;
    for (size_t i = 0; i < st->nsubs; i++) {
        if (fdt_topic_match(st->subs[i].topic, topic)) {
            st->subs[i].cb(topic, m.payload, m.payload_len,
                           st->subs[i].user);
            matched = 1;
        }
    }
    if (matched) {
        st->delivered++;
    }

    if (m.qos == 1) {
        uint8_t ack[4];
        const long n = fdt_mqtt_enc_puback(ack, sizeof ack, m.packet_id);
        if (n < 0 || send_all(st, ack, (size_t)n) != 0) {
            return -1;
        }
    }
    return 0;
}

/** Processes every complete packet in the receive buffer. */
static int drain(mqtt_state_t *st)
{
    size_t at = 0;
    for (;;) {
        fdt_mqtt_packet_t pk;
        const int r = fdt_mqtt_frame(st->rx + at, st->rxlen - at, &pk);
        if (r < 0) {
            return -1;
        }
        if (r == 0) {
            break;
        }
        switch (pk.type) {
        case FDT_MQTT_PUBLISH:
            if (on_publish(st, &pk) != 0) {
                return -1;
            }
            break;
        case FDT_MQTT_PUBACK:
            if (st->unacked > 0) {
                st->unacked--;
            }
            break;
        case FDT_MQTT_SUBACK:
            if (pk.body_len >= 2 &&
                (uint16_t)((pk.body[0] << 8) | pk.body[1]) == st->suback_id) {
                st->suback = fdt_mqtt_ack_reason(&pk);
            }
            break;
        default:
            break; /* PINGRESP and anything else carry nothing for us */
        }
        at += pk.total;
    }
    memmove(st->rx, st->rx + at, st->rxlen - at);
    st->rxlen -= at;
    return 0;
}

/**
 * Waits up to @p timeout_ms for bytes, reads what is there, and drains.
 * @return 1 when bytes arrived, 0 on timeout, -1 on a broken connection.
 */
static int pump(mqtt_state_t *st, int timeout_ms)
{
    struct pollfd pfd = { .fd = st->fd, .events = POLLIN, .revents = 0 };
    const int pr = poll(&pfd, 1, timeout_ms);
    if (pr < 0) {
        return (errno == EINTR) ? 0 : -1;
    }
    if (pr == 0) {
        return 0;
    }
    int got = 0;
    for (;;) {
        if (st->rxlen == sizeof st->rx) {
            return -1; /* a packet larger than the buffer */
        }
        const ssize_t n = recv(st->fd, st->rx + st->rxlen,
                               sizeof st->rx - st->rxlen, MSG_DONTWAIT);
        if (n > 0) {
            st->rxlen += (size_t)n;
            got = 1;
            continue;
        }
        if (n == 0) {
            return -1; /* broker closed */
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }
        if (errno != EINTR) {
            return -1;
        }
    }
    return (drain(st) == 0) ? got : -1;
}

static int mqtt_publish(void *self, const char *topic, const uint8_t *buf,
                        size_t len, int qos)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL || st->fd < 0 || topic == NULL) {
        return -1;
    }
    /* QoS 1 by default, matching the `out 1` of the bridge configuration. */
    const int q = (qos > 0) ? 1 : 0;
    const uint16_t id = (q > 0) ? next_id(st) : 0;
    const long n = fdt_mqtt_enc_publish(st->tx, sizeof st->tx, topic, buf,
                                        len, q, id);
    if (n < 0 || send_all(st, st->tx, (size_t)n) != 0) {
        return -1;
    }
    if (q > 0) {
        st->unacked++;
    }
    return 0;
}

static int mqtt_subscribe(void *self, const char *topic, fdt_on_msg_fn cb,
                          void *user)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL || st->fd < 0 || topic == NULL || cb == NULL ||
        st->nsubs >= MQTT_SUBS || strlen(topic) >= MQTT_TOPIC) {
        return -1;
    }

    /* The callback is in place before the SUBACK, so a retained message
     * that arrives right behind it is delivered. */
    snprintf(st->subs[st->nsubs].topic, MQTT_TOPIC, "%s", topic);
    st->subs[st->nsubs].cb   = cb;
    st->subs[st->nsubs].user = user;
    st->nsubs++;

    uint8_t pkt[128];
    st->suback_id = next_id(st);
    st->suback = -2;
    const long n = fdt_mqtt_enc_subscribe(pkt, sizeof pkt, st->suback_id,
                                          topic, 1);
    if (n < 0 || send_all(st, pkt, (size_t)n) != 0) {
        st->nsubs--;
        return -1;
    }

    const double give_up = now_s() + MQTT_ACK_MS / 1000.0;
    while (st->suback == -2 && now_s() < give_up) {
        if (pump(st, 50) < 0) {
            st->nsubs--;
            return -1;
        }
    }
    if (st->suback < 0 || st->suback > 2) {
        st->nsubs--;
        return -1;
    }
    return 0;
}

static int mqtt_poll(void *self, int timeout_ms)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL || st->fd < 0) {
        return -1;
    }
    st->delivered = 0;

    /* Keepalive: a PINGREQ once half the interval has passed in silence. */
    if (now_s() - st->last_tx > MQTT_KEEPALIVE_S / 2.0) {
        uint8_t ping[2];
        const long n = fdt_mqtt_enc_pingreq(ping, sizeof ping);
        if (n < 0 || send_all(st, ping, (size_t)n) != 0) {
            return -1;
        }
    }

    int r = pump(st, (timeout_ms > 0) ? timeout_ms : 0);
    while (r > 0) {
        r = pump(st, 0); /* take whatever else is already queued */
    }
    return (r < 0) ? -1 : st->delivered;
}

static void mqtt_close_self(void *self)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL) {
        return;
    }
    if (st->fd >= 0) {
        uint8_t bye[2];
        const long n = fdt_mqtt_enc_disconnect(bye, sizeof bye);
        if (n > 0) {
            (void)send_all(st, bye, (size_t)n);
        }
        close(st->fd);
        st->fd = -1;
    }
    free(st);
}

/** Opens a TCP connection to host:port. @return The socket, or -1. */
static int dial(const char *host, int port)
{
    char service[16];
    snprintf(service, sizeof service, "%d", port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res = NULL;
    if (getaddrinfo(host, service, &hints, &res) != 0) {
        return -1;
    }
    int fd = -1;
    for (struct addrinfo *a = res; a != NULL; a = a->ai_next) {
        fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (connect(fd, a->ai_addr, a->ai_addrlen) == 0) {
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    if (fd >= 0) {
        /* Small telemetry messages must not wait for Nagle's algorithm. */
        const int one = 1;
        (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    }
    return fd;
}

int fdt_mqtt_open(const char *host, int port, const char *client_id,
                  fdt_transport_t *out)
{
    if (host == NULL || client_id == NULL || out == NULL || port <= 0 ||
        port > 65535) {
        return -1;
    }
    mqtt_state_t *st = (mqtt_state_t *)calloc(1, sizeof *st);
    if (st == NULL) {
        return -1;
    }
    st->fd = dial(host, port);
    if (st->fd < 0) {
        free(st);
        return -1;
    }

    uint8_t pkt[256];
    const long n = fdt_mqtt_enc_connect(pkt, sizeof pkt, client_id,
                                        MQTT_KEEPALIVE_S);
    if (n < 0 || send_all(st, pkt, (size_t)n) != 0) {
        close(st->fd);
        free(st);
        return -1;
    }

    /* CONNACK is the first packet the broker sends. */
    int reason = -1;
    const double give_up = now_s() + MQTT_ACK_MS / 1000.0;
    while (now_s() < give_up) {
        struct pollfd pfd = { .fd = st->fd, .events = POLLIN, .revents = 0 };
        if (poll(&pfd, 1, 50) <= 0) {
            continue;
        }
        const ssize_t r = recv(st->fd, st->rx + st->rxlen,
                               sizeof st->rx - st->rxlen, 0);
        if (r <= 0) {
            break;
        }
        st->rxlen += (size_t)r;
        fdt_mqtt_packet_t pk;
        if (fdt_mqtt_frame(st->rx, st->rxlen, &pk) == 1) {
            if (pk.type == FDT_MQTT_CONNACK) {
                reason = fdt_mqtt_ack_reason(&pk);
            }
            memmove(st->rx, st->rx + pk.total, st->rxlen - pk.total);
            st->rxlen -= pk.total;
            break;
        }
    }
    if (reason != 0) {
        close(st->fd);
        free(st);
        return -1;
    }

    out->publish   = mqtt_publish;
    out->subscribe = mqtt_subscribe;
    out->poll      = mqtt_poll;
    out->close     = mqtt_close_self;
    out->self      = st;
    return 0;
}

void fdt_mqtt_close(fdt_transport_t *tr)
{
    if (tr == NULL || tr->close == NULL) {
        return;
    }
    tr->close(tr->self);
    tr->publish   = NULL;
    tr->subscribe = NULL;
    tr->poll      = NULL;
    tr->close     = NULL;
    tr->self      = NULL;
}
