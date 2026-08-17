#ifndef FLEET_DT_MAVLINK_H
#define FLEET_DT_MAVLINK_H

#include <fleet_dt/model.h>

#include <stdint.h>

/**
 * @file fdt_mavlink.h
 * @brief Assembling I^t from Ardupilot, and pushing A^t back.
 *
 * Section II: the Jundia boat carries a NAVIO2 board running Ardupilot/Rover
 * on a Raspberry Pi 4, and "sensor readings are available from Raspberry Pi
 * OS via Ardupilot\\Rover firmware". Section III adds that the low-speed
 * sensors "are attached to the NAVIO2 board and controlled by the Ardupilot
 * firmware".
 *
 * This adapter is the seam between that firmware and the model. It takes the
 * *decoded field values* of the relevant MAVLink messages rather than raw
 * frames, which keeps it free of any dependency on a MAVLink library: the
 * caller uses whichever one it already links, and hands the numbers over.
 * The mapping and the unit conversions are the substance, and they are what
 * this header pins.
 *
 * Table I's 21 entries come from five message families, and the two camera
 * views come from none of them — Section III moves the camera onto RTSP.
 *
 * | Table I            | Unit  | Message              | Conversion            |
 * |--------------------|-------|----------------------|-----------------------|
 * | a_x, a_y, a_z      | m/s^2 | SCALED_IMU2          | mG      -> * 9.80665/1000 |
 * | omega_x..z         | rad/s | SCALED_IMU2          | mrad/s  -> / 1000     |
 * | m_x, m_y, m_z      | uT    | SCALED_IMU2          | mgauss  -> * 0.1      |
 * | phi_gps, lambda_gps| deg   | GPS_RAW_INT          | 1e7 deg -> * 1e-7     |
 * | h_gps              | m     | GPS_RAW_INT          | mm      -> / 1000     |
 * | v_N, v_E, v_D      | m/s   | LOCAL_POSITION_NED   | direct                |
 * | P                  | Pa    | SCALED_PRESSURE      | hPa     -> * 100      |
 * | T                  | deg C | SCALED_PRESSURE      | c-deg C -> / 100      |
 * | V_b                | V     | BATTERY_STATUS       | mV      -> / 1000     |
 * | I_b                | A     | BATTERY_STATUS       | cA      -> / 100      |
 * | X_left, X_right    | --    | none: RTSP, Sec. III | not carried here      |
 * | tau                | %     | RC_CHANNELS_OVERRIDE | 0-100 % -> 1000-2000 us |
 * | alpha              | rad   | RC_CHANNELS_OVERRIDE | +/-0.6 rad -> 1000-2000 us |
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
 *
 * Section II describes steering by rotating the propulsion cage rather than
 * with a rudder, so this is the travel the full RC range maps onto.
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
 *
 * Kept separate because these do not arrive over MAVLink: Section III routes
 * the camera feed outside the telemetry infrastructure.
 */
void fdt_mav_attach_views(fdt_mav_ingest_t *ing, const void *left,
                          const void *right);

/**
 * @brief Whether every family Table I needs has arrived.
 * @return 1 when complete, 0 otherwise.
 *
 * A frame assembled from a partial set would hand delta^e stale zeros for the
 * missing entries, indistinguishable from real readings.
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
 *
 * Throttle spans 0..100 % onto 1000..2000 us; the cage spans
 * +/-::FDT_CAGE_LIMIT_RAD onto the same range, clamped at the mechanical
 * limit rather than wrapping.
 */
void fdt_mav_actuation_to_rc(const fdt_actuation_t *a, uint16_t *chan_throttle,
                             uint16_t *chan_cage);

#endif /* FLEET_DT_MAVLINK_H */
