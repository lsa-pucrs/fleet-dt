#ifndef FLEET_DT_REGULATOR_H
#define FLEET_DT_REGULATOR_H

#include <stdint.h>

/**
 * @file regulator.h
 * @brief The bandwidth regulators of Section III.
 *
 * One regulator watches one sensor. The sensor offers every sample it takes,
 * at its own rate, and the control path on the boat reads every one of them.
 * The regulator holds only the newest sample. At the edge of each 125 ms DT
 * tick the pacer calls ::fdt_reg_capture, and the regulator releases the
 * sample it holds to the link. A newer sample replaces an older one before
 * the tick, and the replaced sample counts as dropped.
 *
 * The link therefore carries at most one sample per sensor per tick, the
 * reduction M_i = f_i * T of the presentation, slide 8. A sensor slower than
 * the tick is never decimated: a tick that finds no new sample publishes
 * nothing, and no sample is repeated.
 */
typedef struct {
    double   sensor_hz; /**< Rate the sensor samples at, positive. */
    int      held;      /**< Non-zero while a sample waits for the tick. */
    uint64_t sampled;   /**< Samples offered: published + dropped + held. */
    uint64_t published; /**< Samples released to the link at a tick edge. */
    uint64_t dropped;   /**< Samples a newer one replaced before the tick. */
    uint64_t empty;     /**< Tick edges that found no new sample. */
} fdt_reg_t;

/**
 * @brief Arms a regulator.
 * @param sensor_hz  Rate the sensor samples at; must be positive. The tick
 *                   rate is the pacer's, not the regulator's.
 * @return 0 on success, -1 when @p r is NULL or the rate is not positive.
 */
int fdt_reg_init(fdt_reg_t *r, double sensor_hz);

/**
 * @brief Offers one sample, which becomes the newest one held.
 *
 * The caller stores the sample itself; the regulator decides only whether a
 * sample waits for the tick. When one already waits, the older one is
 * dropped.
 */
void fdt_reg_offer(fdt_reg_t *r);

/**
 * @brief The edge of a DT tick.
 * @return 1 when the newest sample must be published now, 0 when no sample
 *         arrived since the previous edge.
 */
int fdt_reg_capture(fdt_reg_t *r);

/**
 * @brief Samples a sensor takes inside tick @p k, the interval
 *        (k / tick_hz, (k + 1) / tick_hz].
 *
 * Samples fall at j / sensor_hz, j = 1, 2, ... A 10 Hz GPS under the 8 Hz
 * tick yields 1, 1, 1, 2, then repeats: 1.25 per tick on average.
 *
 * @return The count, or 0 when either rate is not positive.
 */
unsigned fdt_reg_samples_in_tick(double sensor_hz, double tick_hz,
                                 uint64_t k);

/** @brief Samples offered so far. */
uint64_t fdt_reg_sampled(const fdt_reg_t *r);

/** @brief Samples released to the link. */
uint64_t fdt_reg_published(const fdt_reg_t *r);

/** @brief Samples a newer sample replaced before the tick. */
uint64_t fdt_reg_dropped(const fdt_reg_t *r);

/** @brief Tick edges that found nothing to publish. */
uint64_t fdt_reg_empty_ticks(const fdt_reg_t *r);

/**
 * @brief The publication rate achieved, in hertz.
 * @return published / sampled times the sensor rate, or 0.0 before the first
 *         sample.
 */
double fdt_reg_effective_hz(const fdt_reg_t *r);

/**
 * @brief The fraction of this sensor's samples kept off the link.
 * @return dropped / sampled, in [0, 1); 0.0 before the first sample.
 */
double fdt_reg_saved_ratio(const fdt_reg_t *r);

#endif /* FLEET_DT_REGULATOR_H */
