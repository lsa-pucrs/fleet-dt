#ifndef FLEET_DT_MAVLINK_H
#define FLEET_DT_MAVLINK_H

#include <fleet_dt/model.h>

#include <stdint.h>

/**
 * @file fdt_mavlink.h
 * @brief Assembling I^t from Ardupilot, and pushing A^t back.
 */

/** Which message families have been seen since the last frame reset. */
typedef enum {
    FDT_MAV_HAVE_IMU      = 1u << 0,
    FDT_MAV_HAVE_GPS      = 1u << 1,
    FDT_MAV_HAVE_NED      = 1u << 2,
    FDT_MAV_HAVE_PRESSURE = 1u << 3,
    FDT_MAV_HAVE_BATTERY  = 1u << 4
} fdt_mav_have_t;

/** Every family Table I needs. */
#define FDT_MAV_HAVE_ALL                                         \
    (FDT_MAV_HAVE_IMU | FDT_MAV_HAVE_GPS | FDT_MAV_HAVE_NED |    \
     FDT_MAV_HAVE_PRESSURE | FDT_MAV_HAVE_BATTERY)

/** Neutral RC pulse width, in microseconds. */
#define FDT_RC_MID 1500

/** RC pulse width span either side of neutral, in microseconds. */
#define FDT_RC_SPAN 500

/**
 * Mechanical limit of the propulsion cage, in radians.
 */
#define FDT_CAGE_LIMIT_RAD 0.6f

/** Accumulates one frame's worth of MAVLink into an I^t. */
typedef struct {
    fdt_input_t input; /**< The assembled I^t. */
    unsigned    have;  /**< Bitmask of ::fdt_mav_have_t. */
} fdt_mav_ingest_t;

/** @brief Clears the accumulator, ready for a new frame. */
void fdt_mav_init(fdt_mav_ingest_t *ing);

/**
 * @brief Folds in a SCALED_IMU2 message.
 * @param xacc,yacc,zacc     Acceleration in milli-g.
 * @param xgyro,ygyro,zgyro  Angular rate in milliradians per second.
 * @param xmag,ymag,zmag     Magnetic field in milligauss.
 */
void fdt_mav_on_scaled_imu(fdt_mav_ingest_t *ing,
                           int16_t xacc, int16_t yacc, int16_t zacc,
                           int16_t xgyro, int16_t ygyro, int16_t zgyro,
                           int16_t xmag, int16_t ymag, int16_t zmag);

/**
 * @brief Folds in a GPS_RAW_INT message.
 * @param lat,lon  Degrees times 1e7.
 * @param alt_mm   Altitude above mean sea level, millimetres.
 */
void fdt_mav_on_gps_raw(fdt_mav_ingest_t *ing, int32_t lat, int32_t lon,
                        int32_t alt_mm);

/**
 * @brief Folds in a LOCAL_POSITION_NED message.
 * @param vx,vy,vz  North, east and down velocity in m/s.
 */
void fdt_mav_on_local_ned(fdt_mav_ingest_t *ing, float vx, float vy,
                          float vz);

/**
 * @brief Folds in a SCALED_PRESSURE message.
 * @param press_abs_hpa  Absolute pressure in hectopascal.
 * @param temperature_c  Temperature in hundredths of a degree Celsius.
 */
void fdt_mav_on_scaled_pressure(fdt_mav_ingest_t *ing, float press_abs_hpa,
                                int16_t temperature_c);

/**
 * @brief Folds in a BATTERY_STATUS message.
 * @param voltage_mv       Cell or pack voltage in millivolts.
 * @param current_battery  Current in centiamperes; negative means unknown.
 */
void fdt_mav_on_battery(fdt_mav_ingest_t *ing, uint16_t voltage_mv,
                        int16_t current_battery);

/**
 * @brief Attaches the stereo views from the RTSP path.
 */
void fdt_mav_attach_views(fdt_mav_ingest_t *ing, const void *left,
                          const void *right);

/**
 * @brief Whether every family Table I needs has arrived.
 * @return 1 when complete, 0 otherwise.
 */
int fdt_mav_complete(const fdt_mav_ingest_t *ing);

/**
 * @brief The assembled I^t.
 * @return A pointer into @p ing, valid until the next fdt_mav_init().
 */
const fdt_input_t *fdt_mav_input(const fdt_mav_ingest_t *ing);

/**
 * @brief Converts A^t into RC channel overrides for Ardupilot.
 * @param chan_throttle  Receives the throttle pulse width, microseconds.
 * @param chan_cage      Receives the cage pulse width, microseconds.
 */
void fdt_mav_actuation_to_rc(const fdt_actuation_t *a, uint16_t *chan_throttle,
                             uint16_t *chan_cage);

#endif /* FLEET_DT_MAVLINK_H */
