#include "fdt_mqtt.h"

#include <mosquitto.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** Subscriptions one client may hold. */
#define MQTT_SUBS 64

/** Longest topic stored, including the terminator. */
#define MQTT_TOPIC 64

/** Per-connection state behind fdt_transport_t::self. */
typedef struct {
    struct mosquitto *mosq;
    int               delivered; /**< Messages delivered during the current poll. */

    struct {
        char          topic[MQTT_TOPIC];
        fdt_on_msg_fn cb;
        void         *user;
    } subs[MQTT_SUBS];
    size_t nsubs;
} mqtt_state_t;

/**
 * Dispatches an arriving message to the matching subscriptions.
 */
static void on_message(struct mosquitto *m, void *obj,
                       const struct mosquitto_message *msg)
{
    (void)m;

    mqtt_state_t *st = (mqtt_state_t *)obj;
    if (st == NULL || msg == NULL || msg->topic == NULL) {
        return;
    }

    int matched = 0;
    for (size_t i = 0; i < st->nsubs; i++) {
        if (strcmp(st->subs[i].topic, msg->topic) == 0) {
            st->subs[i].cb(msg->topic, (const uint8_t *)msg->payload,
                           (size_t)msg->payloadlen, st->subs[i].user);
            matched = 1;
        }
    }
    if (matched) {
        st->delivered++;
    }
}

static int mqtt_publish(void *self, const char *topic, const uint8_t *buf,
                        size_t len, int qos)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL || st->mosq == NULL || topic == NULL) {
        return -1;
    }

    /* QoS 1 by default, matching the `out 1` of the bridge configuration. */
    const int q = (qos > 0) ? qos : 1;
    const int rc = mosquitto_publish(st->mosq, NULL, topic, (int)len, buf,
                                     q, false);
    return (rc == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

static int mqtt_subscribe(void *self, const char *topic, fdt_on_msg_fn cb,
                          void *user)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL || st->mosq == NULL || topic == NULL || cb == NULL) {
        return -1;
    }
    if (st->nsubs >= MQTT_SUBS || strlen(topic) >= MQTT_TOPIC) {
        return -1;
    }
    if (mosquitto_subscribe(st->mosq, NULL, topic, 1) != MOSQ_ERR_SUCCESS) {
        return -1;
    }

    snprintf(st->subs[st->nsubs].topic, MQTT_TOPIC, "%s", topic);
    st->subs[st->nsubs].cb   = cb;
    st->subs[st->nsubs].user = user;
    st->nsubs++;
    return 0;
}

static int mqtt_poll(void *self, int timeout_ms)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL || st->mosq == NULL) {
        return -1;
    }

    st->delivered = 0;
    const int rc = mosquitto_loop(st->mosq, timeout_ms, 1);
    if (rc != MOSQ_ERR_SUCCESS) {
        return -1;
    }
    return st->delivered;
}

static void mqtt_close_self(void *self)
{
    mqtt_state_t *st = (mqtt_state_t *)self;
    if (st == NULL) {
        return;
    }
    if (st->mosq != NULL) {
        mosquitto_disconnect(st->mosq);
        mosquitto_destroy(st->mosq);
        st->mosq = NULL;
    }
    free(st);
    mosquitto_lib_cleanup();
}

int fdt_mqtt_open(const char *host, int port, const char *client_id,
                  fdt_transport_t *out)
{
    if (host == NULL || client_id == NULL || out == NULL) {
        return -1;
    }

    mosquitto_lib_init();

    mqtt_state_t *st = (mqtt_state_t *)calloc(1, sizeof *st);
    if (st == NULL) {
        mosquitto_lib_cleanup();
        return -1;
    }

    st->mosq = mosquitto_new(client_id, false, st);
    if (st->mosq == NULL) {
        free(st);
        mosquitto_lib_cleanup();
        return -1;
    }

    mosquitto_message_callback_set(st->mosq, on_message);

    if (mosquitto_connect(st->mosq, host, port, 60) != MOSQ_ERR_SUCCESS) {
        mosquitto_destroy(st->mosq);
        free(st);
        mosquitto_lib_cleanup();
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
