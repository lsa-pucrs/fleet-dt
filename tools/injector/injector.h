#ifndef FLEET_DT_INJECTOR_H
#define FLEET_DT_INJECTOR_H

#include <fleet_dt/model.h>
#include <fleet_dt/transport.h>

#include <stdint.h>

/**
 * @file injector.h
 * @brief Synthetic telemetry injectors, as used in the Section V-B campaign.
 *
 * Section V-B: "Fleet DT was validated in simulation using injectors (MQTT
 * clients) that periodically published synthetic telemetry data into the
 * network, mimicking the real boats. The goal was to investigate whether
 * multiple boats could synchronize with the simulation and keep pace with its
 * real-time execution."
 *
 * The same section reports the limit the injectors themselves hit:
 * "hard-programming injectors to inject packets periodically could not keep
 * the simulation pace for larger fleets (> 25 boats, same computer model)".
 * That ceiling belongs to the injector, not to the DTI, and the scaling
 * benchmark reports the two separately for exactly this reason.
 *
 * Telemetry is deterministic given the seed, so a campaign is repeatable
 * across runs and across machines — otherwise a scaling result could not be
 * compared against a previous one.
 */

/**
 * Amplitude of the synthetic position sweep, in degrees.
 *
 * Section II gives the Jundiá hull as 1.2 m long. The sweep has to be on that
 * scale or the fleet is invisible in its own simulation: at 0.001 degrees --
 * 111 m -- the vessels wandered out of the viewport in half a minute, and at
 * 0.0002 they were two dark pixels on dark water. 0.00002 degrees is about
 * 2.2 m, under two hull lengths, which is what a boat holding station over a
 * sampling point actually does -- and it keeps the fleet inside the viewport,
 * because the world's follow does not track a hull the supervisor teleports.
 *
 * The wander is twice this, not once: the WeBots controller anchors its local
 * origin on the first fix it sees, and that lands at an arbitrary phase of the
 * circle. tests/test_geo.c pins the doubled figure against both the water box
 * and the viewport.
 */
#define FDT_INJ_SWEEP_DEG 0.00002

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
 * Fills all 21 entries of Table I with values in the ranges Section II
 * describes for the Jundia boat: an 18.5 V battery, atmospheric pressure near
 * sea level, and a position in the southern Brazilian lagoons the eDNA
 * campaign samples.
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
 *
 * The caller owns the clock, so it is the caller that notices. This is the
 * counter that distinguishes an injector-bound run from a DTI-bound one.
 */
void fdt_inj_note_missed_deadline(fdt_inj_t *inj);

/** @brief Messages successfully published. */
uint64_t fdt_inj_published(const fdt_inj_t *inj);

/** @brief Publishes the transport refused. */
uint64_t fdt_inj_refused(const fdt_inj_t *inj);

/** @brief Ticks that ran past their period. */
uint64_t fdt_inj_missed_deadlines(const fdt_inj_t *inj);

#endif /* FLEET_DT_INJECTOR_H */
