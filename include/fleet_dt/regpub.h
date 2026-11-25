#ifndef FLEET_DT_REGPUB_H
#define FLEET_DT_REGPUB_H

#include <fleet_dt/codec.h>
#include <fleet_dt/model.h>
#include <fleet_dt/regulator.h>
#include <fleet_dt/transport.h>

#include <stdint.h>

/**
 * @file regpub.h
 * @brief The regulated publication stage of one vessel.
 *
 * Section III drops samples "in the MQTT client in the broker", and the
 * talk draws the regulator inside the boat broker (slides 7, 12 and 13):
 * the sensors publish every sample to the boat broker, and only the newest
 * sample of each sensor per 125 ms tick crosses the bridge. This module is
 * the stage that does it. Each sensor of the LSDT group offers every sample;
 * the stage keeps the newest one per sensor, and at the edge of each tick it
 * publishes one message per sensor that sampled since the previous edge, on
 * fleet/<v>/lsdt/<sensor>. fdt_relay_t runs one stage per vessel as an MQTT
 * client on the boat broker; the injector runs the same stages in process.
 *
 * With @c raw set, the stage bypasses the regulator and publishes every
 * sample the moment it is offered. Under ::FDT_TOPIC_LSDT that is the "raw
 * rates" bar of slide 13, a bridge without the regulator; under
 * ::FDT_TOPIC_RAW (fdt_regpub_set_level) it is the autopilot publishing its
 * samples to the boat broker.
 *
 * Envelope sequence numbers count per vessel and per sensor. Regulated, the
 * sequence of a message is the tick that released it, so the four messages
 * of one tick share it and a receiver can group them into one frame. Raw, it
 * counts samples.
 */
typedef struct {
    fdt_transport_t *tr;     /**< Where messages go; outlives the publisher. */
    uint16_t         vessel; /**< Vessel index stamped on every message. */
    int              raw;    /**< Non-zero bypasses the regulators. */
    const char      *level;  /**< Topic level, ::FDT_TOPIC_LSDT by default. */
    fdt_reg_t        reg[FDT_SENSOR_COUNT];  /**< One regulator per sensor. */
    fdt_input_t      held;   /**< Newest sample of each sensor. */
    uint32_t         seq[FDT_SENSOR_COUNT];  /**< Last sequence sent. */
    uint32_t         ticks;  /**< Tick edges seen. */
    uint64_t         messages; /**< Messages the transport accepted. */
    uint64_t         bytes;    /**< Envelope + payload bytes accepted. */
    uint64_t         refused;  /**< Publishes the transport refused. */
} fdt_regpub_t;

/**
 * @brief Arms a publisher.
 * @param tr         Transport; its @c publish must be bound.
 * @param vessel     Vessel index, at most 65535.
 * @param sensor_hz  Sampling rate of each sensor, indexed by ::fdt_sensor_t;
 *                   every entry must be positive.
 * @param raw        Non-zero to bypass the regulators.
 * @return 0 on success, -1 on a NULL or out-of-range argument.
 */
int fdt_regpub_init(fdt_regpub_t *rp, fdt_transport_t *tr, unsigned vessel,
                    const double sensor_hz[FDT_SENSOR_COUNT], int raw);

/**
 * @brief Sets the topic level messages go out on: ::FDT_TOPIC_LSDT (the
 *        default) or ::FDT_TOPIC_RAW.
 * @return 0 on success, -1 on a NULL argument.
 */
int fdt_regpub_set_level(fdt_regpub_t *rp, const char *level);

/**
 * @brief Offers one sample of sensor @p s.
 *
 * Only the fields @p s measures are read from @p sample.
 *
 * @return 1 when the sample went out now (raw mode), 0 when the regulator
 *         holds it for the tick, -1 on a bad argument or a refused publish.
 */
int fdt_regpub_offer(fdt_regpub_t *rp, fdt_sensor_t s,
                     const fdt_input_t *sample);

/**
 * @brief The edge of a DT tick: publishes the newest sample of each sensor
 *        that sampled since the previous edge.
 * @return Messages published, 0 in raw mode, -1 on a NULL argument.
 */
int fdt_regpub_tick(fdt_regpub_t *rp);

/** @brief The regulator of sensor @p s, or NULL. */
const fdt_reg_t *fdt_regpub_reg(const fdt_regpub_t *rp, fdt_sensor_t s);

/** @brief Messages the transport accepted. */
uint64_t fdt_regpub_messages(const fdt_regpub_t *rp);

/** @brief Envelope and payload bytes the transport accepted. */
uint64_t fdt_regpub_bytes(const fdt_regpub_t *rp);

/** @brief Publishes the transport refused. */
uint64_t fdt_regpub_refused(const fdt_regpub_t *rp);

#endif /* FLEET_DT_REGPUB_H */
