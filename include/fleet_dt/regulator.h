#ifndef FLEET_DT_REGULATOR_H
#define FLEET_DT_REGULATOR_H

#include <stdint.h>

/**
 * @file regulator.h
 * @brief The bandwidth regulators of Section III.
 *
 * Section III: "Bandwidth regulators control the frequency of publishing on
 * MQTT clients. The DTI updates every 125 ms (this paper); each update in the
 * real world corresponds to a single increment in k. As some sensors, e.g., a
 * gyroscope at 1 Hz, sample faster than the DT frequency, part of the
 * bandwidth is wasted. The proposed regulators overcome this problem by
 * dropping the number of samples in the MQTT client in the broker. However,
 * real sensors continue sampling at their own pace, as this is necessary for
 * control algorithms and other applications that neither interact with the
 * DTI nor affect the model."
 *
 * Two halves, and both matter:
 *
 * 1. Publication is decimated to the DT rate, so the link does not carry
 *    samples the twin has nowhere to put.
 * 2. Sampling is untouched. The regulator sits in the publish path, not in
 *    the sensor path, so the control loops that read the sensor directly keep
 *    their full rate. A regulator that slowed the sensor would buy bandwidth
 *    by degrading control, which is the opposite of the QoS claim.
 *
 * The regulator is therefore an admission test on publication, and the
 * sampled counter records everything the sensor produced, including what was
 * dropped — which is what lets the saved ratio be reported honestly.
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
 *
 * Call this once per sample the sensor produces, in the MQTT client. The
 * sensor keeps sampling either way.
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
 *
 * This is the "maximizing the use of shared wireless links" half of the
 * abstract, expressed as a number a benchmark can plot.
 */
double fdt_reg_saved_ratio(const fdt_reg_t *r);

#endif /* FLEET_DT_REGULATOR_H */
