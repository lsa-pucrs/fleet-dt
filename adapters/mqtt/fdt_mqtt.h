#ifndef FLEET_DT_MQTT_H
#define FLEET_DT_MQTT_H

#include <fleet_dt/transport.h>

/**
 * @file fdt_mqtt.h
 * @brief The MQTT transport the abstract says the architecture relies on.
 */

/**
 * @brief Connects to a broker and returns a transport for it.
 *
 * @param host       Broker hostname; on a boat this is the local broker,
 *                   which then bridges to the ground station.
 * @param port       Broker port, normally 1883.
 * @param client_id  MQTT client identifier; must be unique on the broker.
 * @param out        Receives the transport on success.
 * @return 0 on success, -1 on a NULL argument or a failed connection.
 *
 * @note Publishes default to QoS 1, matching the `out 1` of the bridge
 *       configuration: at-least-once delivery is what lets a queued message
 *       survive a dropped link, and the sequence number in the envelope is
 *       what lets the receiver notice the duplicate that at-least-once
 *       permits.
 */
int fdt_mqtt_open(const char *host, int port, const char *client_id,
                  fdt_transport_t *out);

/**
 * @brief Disconnects and releases the transport.
 */
void fdt_mqtt_close(fdt_transport_t *tr);

#endif /* FLEET_DT_MQTT_H */
