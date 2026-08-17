#ifndef FLEET_DT_INJECTOR_H
#define FLEET_DT_INJECTOR_H

#include <fleet_dt/model.h>
#include <fleet_dt/transport.h>

#include <stdint.h>

/**
 * @file injector.h
 * @brief Synthetic telemetry injectors, as used in the Section V-B campaign.
 */

/**
 * Amplitude of the synthetic position sweep, in degrees.
 */
#define FDT_INJ_SWEEP_DEG 0.00002

/**
 * Spacing between vessels, in degrees. About 4 m.
 */
#define FDT_INJ_SPACING_DEG 0.000036

/** One injector driving a whole fleet's worth of synthetic vessels. */
typedef struct {
    fdt_transport_t *tr;      /**< Where samples are published. */
    unsigned         vessels; /**< Vessels this injector impersonates. */
    double           hz;      /**< Nominal publication rate per vessel. */
    uint32_t         seed;    /**< Base seed; vessel k uses seed ^ k. */
    uint32_t         tick;    /**< Ticks issued so far, drives the waveform. */
    uint32_t         seq;     /**< Per-tick sequence, shared by all vessels. */
    uint64_t         published;        /**< Messages successfully published. */
    uint64_t         refused;          /**< Publishes the transport refused. */
    uint64_t         missed_deadlines; /**< Ticks that ran past their period. */
} fdt_inj_t;

/**
 * @brief Arms an injector.
 * @param tr       Transport to publish on; must outlive the injector.
 * @param vessels  Vessels to impersonate; must be positive.
 * @param hz       Nominal publication rate, normally the 8 Hz DT rate.
 * @param seed     Base seed. The same seed yields the same telemetry.
 * @return 0 on success, -1 when an argument is NULL or out of range.
 */
int fdt_inj_init(fdt_inj_t *inj, fdt_transport_t *tr, unsigned vessels,
                 double hz, uint32_t seed);

/**
 * @brief Generates the telemetry of one vessel at the current tick.
 *
 * @param vessel  Vessel index; out-of-range indices leave @p out zeroed.
 * @param out     Receives the sample.
 */
void fdt_inj_sample(const fdt_inj_t *inj, unsigned vessel, fdt_input_t *out);

/**
 * @brief Publishes one I^t per vessel, then advances the tick.
 * @return Messages published this tick, or -1 when the injector is unbound.
 *         A return below the vessel count means the transport refused some
 *         publishes, which is itself a measurement.
 */
int fdt_inj_tick(fdt_inj_t *inj);

/**
 * @brief Records that a tick ran past its period.
 */
void fdt_inj_note_missed_deadline(fdt_inj_t *inj);

/** @brief Messages successfully published. */
uint64_t fdt_inj_published(const fdt_inj_t *inj);

/** @brief Publishes the transport refused. */
uint64_t fdt_inj_refused(const fdt_inj_t *inj);

/** @brief Ticks that ran past their period. */
uint64_t fdt_inj_missed_deadlines(const fdt_inj_t *inj);

#endif /* FLEET_DT_INJECTOR_H */
