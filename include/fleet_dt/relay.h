#ifndef FLEET_DT_RELAY_H
#define FLEET_DT_RELAY_H

#include <fleet_dt/regpub.h>
#include <fleet_dt/transport.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @file relay.h
 * @brief The regulator of the boat broker: an MQTT client on the boat
 *        broker that turns raw samples into regulated telemetry.
 *
 * The autopilot publishes every sample of every sensor on
 * fleet/<v>/raw/<sensor>, at the sensor's rate, to the local broker. The
 * relay subscribes to ::FDT_TOPIC_RAW_ALL, hands each sample to the
 * regulated stage of its vessel, and at the edge of each 125 ms tick
 * publishes the newest sample of each sensor on fleet/<v>/lsdt/<sensor>.
 * The bridge to the ground station carries only the lsdt level
 * (config/mosquitto/boat.conf), so the raw samples never leave the boat.
 */
typedef struct {
    fdt_transport_t *tr;       /**< The boat broker connection. */
    fdt_regpub_t    *stages;   /**< Caller-owned, one per vessel. */
    size_t           n;        /**< Vessels covered. */
    uint64_t         received; /**< Raw samples handed to a stage. */
    uint64_t         rejected; /**< Malformed or foreign messages. */
} fdt_relay_t;

/**
 * @brief Arms the relay and subscribes it to the raw samples.
 * @param tr         Boat broker connection; publish and subscribe bound.
 * @param stages     Array of @p n stages, armed here in regulated mode.
 * @param sensor_hz  Sampling rate of each sensor, by ::fdt_sensor_t.
 * @return 0 on success, -1 on a bad argument or a refused subscription.
 */
int fdt_relay_init(fdt_relay_t *rl, fdt_transport_t *tr,
                   fdt_regpub_t *stages, size_t n,
                   const double sensor_hz[FDT_SENSOR_COUNT]);

/**
 * @brief Applies one raw message; the subscription callback calls it, and
 *        a caller holding the bytes may call it directly.
 * @return 0 when the sample reached a stage, -1 otherwise.
 */
int fdt_relay_offer(fdt_relay_t *rl, const uint8_t *buf, size_t len);

/**
 * @brief The edge of a DT tick: every stage publishes its newest samples.
 * @return Messages published, or -1 on a NULL relay.
 */
int fdt_relay_tick(fdt_relay_t *rl);

/** @brief Raw samples handed to a stage. */
uint64_t fdt_relay_received(const fdt_relay_t *rl);

/** @brief Messages refused as malformed or foreign. */
uint64_t fdt_relay_rejected(const fdt_relay_t *rl);

#endif /* FLEET_DT_RELAY_H */
