#ifndef FLEET_DT_MQTT_H
#define FLEET_DT_MQTT_H

#include <fleet_dt/transport.h>

/**
 * @file fdt_mqtt.h
 * @brief The MQTT transport the abstract says the architecture relies on.
 *
 * Section III: "Communication between the boat and the ground station is
 * simulated over Wi-Fi, and local MQTT brokers are connected in bridge mode
 * to avoid service interruption during temporary connection instability.
 * Brokers serve multiple MQTT clients."
 *
 * This is an implementation of ::fdt_transport_t over libmosquitto. Nothing
 * above the transport seam changes: the same coordinator, the same twins and
 * the same benchmarks run over this or over the in-process loopback, which is
 * what makes a loopback measurement and a broker measurement comparable.
 *
 * Bridging is not this file's job. Section III puts it in the broker, and
 * config/mosquitto/ holds the configuration that implements it. A client that
 * tried to bridge in application code would be solving a problem the broker
 * already solves, and would lose the store-and-forward that survives the link
 * instability the paper describes.
 *
 * Built by `make mqtt`, which skips with a notice when libmosquitto is
 * absent. The core library never links it.
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
 *
 * Equivalent to calling @c tr->close(tr->self); provided so a caller holding
 * only the header does not have to reach through the function pointer.
 */
void fdt_mqtt_close(fdt_transport_t *tr);

#endif /* FLEET_DT_MQTT_H */
