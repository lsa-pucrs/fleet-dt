#ifndef FLEET_DT_REGULATOR_H
#define FLEET_DT_REGULATOR_H

#include <stdint.h>

/**
 * @file regulator.h
 * @brief The bandwidth regulators of Section III.
 */
typedef struct {
    double   sensor_hz;  /**< Rate the sensor samples at, positive. */
    double   publish_hz; /**< Rate publication is decimated to, positive. */
    double   step;       /**< Publication credit per sample. */
    double   acc;        /**< Accumulated credit, in [0, 1). */
    uint64_t sampled;    /**< Samples offered, published plus dropped. */
    uint64_t published;  /**< Samples admitted to the link. */
    uint64_t dropped;    /**< Samples the regulator withheld. */
} fdt_reg_t;

/**
 * @brief Arms a regulator.
 * @param sensor_hz   Rate the sensor samples at; must be positive.
 * @param publish_hz  Target publication rate, normally the DT rate of 8 Hz;
 *                    must be positive.
 * @return 0 on success, -1 when either rate is zero or negative.
 *
 * @note A sensor slower than the DT is never decimated: the regulator cannot
 *       invent samples, and must not hide the ones that exist.
 */
int fdt_reg_init(fdt_reg_t *r, double sensor_hz, double publish_hz);

/**
 * @brief Offers one sample to the link.
 * @return 1 when the sample should be published, 0 when it is dropped.
 */
int fdt_reg_admit(fdt_reg_t *r);

/** @brief Samples offered so far, published plus dropped. */
uint64_t fdt_reg_sampled(const fdt_reg_t *r);

/** @brief Samples admitted to the link. */
uint64_t fdt_reg_published(const fdt_reg_t *r);

/** @brief Samples the regulator withheld. */
uint64_t fdt_reg_dropped(const fdt_reg_t *r);

/**
 * @brief The publication rate actually achieved, in hertz.
 * @return published / sampled times the sensor rate, or 0.0 before the first
 *         sample.
 */
double fdt_reg_effective_hz(const fdt_reg_t *r);

/**
 * @brief The fraction of this sensor's traffic the regulator removed.
 * @return dropped / sampled, in [0, 1); 0.0 before the first sample.
 */
double fdt_reg_saved_ratio(const fdt_reg_t *r);

#endif /* FLEET_DT_REGULATOR_H */
