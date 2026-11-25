#ifndef FLEET_DT_INJECTOR_H
#define FLEET_DT_INJECTOR_H

#include <fleet_dt/model.h>
#include <fleet_dt/regpub.h>
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

/** Vessels one injector can impersonate (the presentation sweeps to 150). */
#define FDT_INJ_MAX_VESSELS 512

/** IMU sampling rate of slides 8 and 13, Hz. */
#define FDT_INJ_HZ_IMU 1000.0
/** Barometer sampling rate of slides 8 and 13, Hz. */
#define FDT_INJ_HZ_BARO 50.0
/** GPS sampling rate of slides 8 and 13, Hz. */
#define FDT_INJ_HZ_GPS 10.0
/** Power (V, I) sampling rate of slide 13, Hz. */
#define FDT_INJ_HZ_POWER 10.0

/**
 * One injector driving a whole fleet's worth of synthetic vessels.
 *
 * Each vessel samples its four sensors at their own rates and offers every
 * sample to its own regulated stage (fdt_regpub_t): the "Regulator x N" in
 * the broker of the injector computer, slide 12(b) of the talk. The hop from
 * the injector to its local broker is not carried by the transport; the
 * stages receive the samples in process, and only their regulated output is
 * published. Each tick of the injector is one 125 ms DT frame: the samples
 * falling inside the frame are offered in time order, then the frame edge
 * releases the newest sample of each sensor.
 */
typedef struct {
    fdt_transport_t *tr;      /**< Where samples are published. */
    unsigned         vessels; /**< Vessels this injector impersonates. */
    double           hz;      /**< DT tick rate, normally 8 Hz. */
    double           sensor_hz[FDT_SENSOR_COUNT]; /**< Sampling rates. */
    int              raw;     /**< Non-zero bypasses the regulators. */
    uint32_t         seed;    /**< Base seed; vessel k uses seed ^ k. */
    uint32_t         tick;    /**< Ticks issued so far, drives the waveform. */
    uint64_t         published;        /**< Messages successfully published. */
    uint64_t         refused;          /**< Publishes the transport refused. */
    uint64_t         missed_deadlines; /**< Ticks that ran past their period. */
    fdt_regpub_t     pub[FDT_INJ_MAX_VESSELS]; /**< One client per vessel. */
} fdt_inj_t;

/**
 * @brief Arms an injector.
 * @param tr       Transport to publish on; must outlive the injector.
 * @param vessels  Vessels to impersonate; 1 to ::FDT_INJ_MAX_VESSELS.
 * @param hz       DT tick rate, normally 8 Hz.
 * @param seed     Base seed. The same seed yields the same telemetry.
 * @return 0 on success, -1 when an argument is NULL or out of range.
 */
int fdt_inj_init(fdt_inj_t *inj, fdt_transport_t *tr, unsigned vessels,
                 double hz, uint32_t seed);

/**
 * @brief Re-arms the injector with other sensor rates or in raw mode.
 *
 * Call before the first tick. @p sensor_hz may be NULL to keep the rates.
 * @return 0 on success, -1 on a NULL injector or a non-positive rate.
 */
int fdt_inj_configure(fdt_inj_t *inj, const double *sensor_hz, int raw);

/**
 * @brief Generates the telemetry of one vessel at time @p t_s seconds.
 */
void fdt_inj_sample_at(const fdt_inj_t *inj, unsigned vessel, double t_s,
                       fdt_input_t *out);

/**
 * @brief Generates the telemetry of one vessel at the current tick.
 *
 * @param vessel  Vessel index; out-of-range indices leave @p out zeroed.
 * @param out     Receives the sample.
 */
void fdt_inj_sample(const fdt_inj_t *inj, unsigned vessel, fdt_input_t *out);

/**
 * @brief Runs one DT frame: every vessel offers the sensor samples of the
 *        frame to its regulated client, then the frame edge publishes.
 * @return Messages published this tick: at most four per vessel regulated,
 *         every sample raw; -1 when the injector is unbound. Refused
 *         publishes are counted, never retried.
 */
int fdt_inj_tick(fdt_inj_t *inj);

/**
 * @brief First half of ::fdt_inj_tick: offers the samples of the frame.
 * @return Messages published now (raw mode only), or -1 when unbound.
 */
int fdt_inj_offer_frame(fdt_inj_t *inj);

/**
 * @brief Second half of ::fdt_inj_tick: the frame edge, at which every
 *        regulated client publishes, then the tick advances.
 * @return Messages published at the edge, or -1 when unbound.
 */
int fdt_inj_release(fdt_inj_t *inj);

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

/** @brief The regulated client of @p vessel, or NULL. */
const fdt_regpub_t *fdt_inj_client(const fdt_inj_t *inj, unsigned vessel);

#endif /* FLEET_DT_INJECTOR_H */
