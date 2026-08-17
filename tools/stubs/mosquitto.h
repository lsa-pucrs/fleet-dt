/**
 * @file mosquitto.h
 * @brief Syntax-check stub for libmosquitto.
 */
#ifndef FDT_STUB_MOSQUITTO_H
#define FDT_STUB_MOSQUITTO_H

#include <stdbool.h>

#define MOSQ_ERR_SUCCESS 0

struct mosquitto;

struct mosquitto_message {
    int   mid;
    char *topic;
    void *payload;
    int   payloadlen;
    int   qos;
    bool  retain;
};

int  mosquitto_lib_init(void);
int  mosquitto_lib_cleanup(void);
struct mosquitto *mosquitto_new(const char *id, bool clean_session, void *obj);
void mosquitto_destroy(struct mosquitto *mosq);
int  mosquitto_connect(struct mosquitto *mosq, const char *host, int port,
                       int keepalive);
int  mosquitto_disconnect(struct mosquitto *mosq);
int  mosquitto_publish(struct mosquitto *mosq, int *mid, const char *topic,
                       int payloadlen, const void *payload, int qos,
                       bool retain);
int  mosquitto_subscribe(struct mosquitto *mosq, int *mid, const char *sub,
                         int qos);
int  mosquitto_loop(struct mosquitto *mosq, int timeout, int max_packets);
void mosquitto_message_callback_set(
    struct mosquitto *mosq,
    void (*on_message)(struct mosquitto *, void *,
                       const struct mosquitto_message *));

#endif /* FDT_STUB_MOSQUITTO_H */
