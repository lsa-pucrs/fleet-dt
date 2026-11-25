#include <fleet_dt/codec.h>

#include <string.h>

/** Writes a float as four little-endian bytes and advances the cursor. */
static void put_f32(uint8_t **p, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof bits);

    (*p)[0] = (uint8_t)(bits & 0xFFu);
    (*p)[1] = (uint8_t)((bits >> 8) & 0xFFu);
    (*p)[2] = (uint8_t)((bits >> 16) & 0xFFu);
    (*p)[3] = (uint8_t)((bits >> 24) & 0xFFu);
    *p += 4;
}

/** Reads four little-endian bytes as a float and advances the cursor. */
static float get_f32(const uint8_t **p)
{
    const uint32_t bits = (uint32_t)(*p)[0] |
                          ((uint32_t)(*p)[1] << 8) |
                          ((uint32_t)(*p)[2] << 16) |
                          ((uint32_t)(*p)[3] << 24);
    float v;
    memcpy(&v, &bits, sizeof v);
    *p += 4;
    return v;
}

long fdt_enc_state(const fdt_state_t *b, uint8_t *buf, size_t cap)
{
    if (b == NULL || buf == NULL || cap < FDT_WIRE_STATE_BYTES) {
        return -1;
    }

    uint8_t *p = buf;
    put_f32(&p, b->lat_deg);
    put_f32(&p, b->lon_deg);
    put_f32(&p, b->alt_m);
    put_f32(&p, b->roll_deg);
    put_f32(&p, b->pitch_deg);
    put_f32(&p, b->yaw_deg);
    put_f32(&p, b->surge_mps);
    put_f32(&p, b->sway_mps);
    put_f32(&p, b->heave_mps);
    put_f32(&p, b->roll_rate_rps);
    put_f32(&p, b->pitch_rate_rps);
    put_f32(&p, b->yaw_rate_rps);
    return FDT_WIRE_STATE_BYTES;
}

long fdt_dec_state(const uint8_t *buf, size_t len, fdt_state_t *b)
{
    if (buf == NULL || b == NULL || len < FDT_WIRE_STATE_BYTES) {
        return -1;
    }

    const uint8_t *p = buf;
    b->lat_deg        = get_f32(&p);
    b->lon_deg        = get_f32(&p);
    b->alt_m          = get_f32(&p);
    b->roll_deg       = get_f32(&p);
    b->pitch_deg      = get_f32(&p);
    b->yaw_deg        = get_f32(&p);
    b->surge_mps      = get_f32(&p);
    b->sway_mps       = get_f32(&p);
    b->heave_mps      = get_f32(&p);
    b->roll_rate_rps  = get_f32(&p);
    b->pitch_rate_rps = get_f32(&p);
    b->yaw_rate_rps   = get_f32(&p);
    return FDT_WIRE_STATE_BYTES;
}

long fdt_enc_input(const fdt_input_t *in, uint8_t *buf, size_t cap)
{
    if (in == NULL || buf == NULL || cap < FDT_WIRE_INPUT_BYTES) {
        return -1;
    }

    uint8_t *p = buf;
    put_f32(&p, in->ax_mps2);
    put_f32(&p, in->ay_mps2);
    put_f32(&p, in->az_mps2);
    put_f32(&p, in->wx_rps);
    put_f32(&p, in->wy_rps);
    put_f32(&p, in->wz_rps);
    put_f32(&p, in->mx_ut);
    put_f32(&p, in->my_ut);
    put_f32(&p, in->mz_ut);
    put_f32(&p, in->gps_lat_deg);
    put_f32(&p, in->gps_lon_deg);
    put_f32(&p, in->gps_alt_m);
    put_f32(&p, in->vn_mps);
    put_f32(&p, in->ve_mps);
    put_f32(&p, in->vd_mps);
    put_f32(&p, in->press_pa);
    put_f32(&p, in->temp_c);
    put_f32(&p, in->vbat_v);
    put_f32(&p, in->ibat_a);

    *p++ = (in->x_left != NULL) ? 1u : 0u;
    *p++ = (in->x_right != NULL) ? 1u : 0u;

    return FDT_WIRE_INPUT_BYTES;
}

long fdt_dec_input(const uint8_t *buf, size_t len, fdt_input_t *in)
{
    if (buf == NULL || in == NULL || len < FDT_WIRE_INPUT_BYTES) {
        return -1;
    }

    const uint8_t *p = buf;
    in->ax_mps2     = get_f32(&p);
    in->ay_mps2     = get_f32(&p);
    in->az_mps2     = get_f32(&p);
    in->wx_rps      = get_f32(&p);
    in->wy_rps      = get_f32(&p);
    in->wz_rps      = get_f32(&p);
    in->mx_ut       = get_f32(&p);
    in->my_ut       = get_f32(&p);
    in->mz_ut       = get_f32(&p);
    in->gps_lat_deg = get_f32(&p);
    in->gps_lon_deg = get_f32(&p);
    in->gps_alt_m   = get_f32(&p);
    in->vn_mps      = get_f32(&p);
    in->ve_mps      = get_f32(&p);
    in->vd_mps      = get_f32(&p);
    in->press_pa    = get_f32(&p);
    in->temp_c      = get_f32(&p);
    in->vbat_v      = get_f32(&p);
    in->ibat_a      = get_f32(&p);

    in->x_left  = (*p++ != 0u) ? FDT_VIEW_PRESENT : NULL;
    in->x_right = (*p++ != 0u) ? FDT_VIEW_PRESENT : NULL;

    return FDT_WIRE_INPUT_BYTES;
}

long fdt_enc_act(const fdt_actuation_t *a, uint8_t *buf, size_t cap)
{
    if (a == NULL || buf == NULL || cap < FDT_WIRE_ACT_BYTES) {
        return -1;
    }

    uint8_t *p = buf;
    put_f32(&p, a->throttle_pct);
    put_f32(&p, a->cage_rad);
    return FDT_WIRE_ACT_BYTES;
}

long fdt_dec_act(const uint8_t *buf, size_t len, fdt_actuation_t *a)
{
    if (buf == NULL || a == NULL || len < FDT_WIRE_ACT_BYTES) {
        return -1;
    }

    const uint8_t *p = buf;
    a->throttle_pct = get_f32(&p);
    a->cage_rad     = get_f32(&p);
    return FDT_WIRE_ACT_BYTES;
}

size_t fdt_sensor_bytes(fdt_sensor_t s)
{
    switch (s) {
    case FDT_SENSOR_IMU:   return FDT_WIRE_IMU_BYTES;
    case FDT_SENSOR_GPS:   return FDT_WIRE_GPS_BYTES;
    case FDT_SENSOR_BARO:  return FDT_WIRE_BARO_BYTES;
    case FDT_SENSOR_POWER: return FDT_WIRE_POWER_BYTES;
    default:               return 0;
    }
}

const char *fdt_sensor_name(fdt_sensor_t s)
{
    switch (s) {
    case FDT_SENSOR_IMU:   return "imu";
    case FDT_SENSOR_GPS:   return "gps";
    case FDT_SENSOR_BARO:  return "baro";
    case FDT_SENSOR_POWER: return "power";
    default:               return NULL;
    }
}

long fdt_enc_sensor(const fdt_input_t *in, fdt_sensor_t s, uint8_t *buf,
                    size_t cap)
{
    const size_t need = fdt_sensor_bytes(s);
    if (in == NULL || buf == NULL || need == 0 || cap < need) {
        return -1;
    }

    uint8_t *p = buf;
    switch (s) {
    case FDT_SENSOR_IMU:
        put_f32(&p, in->ax_mps2);
        put_f32(&p, in->ay_mps2);
        put_f32(&p, in->az_mps2);
        put_f32(&p, in->wx_rps);
        put_f32(&p, in->wy_rps);
        put_f32(&p, in->wz_rps);
        put_f32(&p, in->mx_ut);
        put_f32(&p, in->my_ut);
        put_f32(&p, in->mz_ut);
        break;
    case FDT_SENSOR_GPS:
        put_f32(&p, in->gps_lat_deg);
        put_f32(&p, in->gps_lon_deg);
        put_f32(&p, in->gps_alt_m);
        put_f32(&p, in->vn_mps);
        put_f32(&p, in->ve_mps);
        put_f32(&p, in->vd_mps);
        break;
    case FDT_SENSOR_BARO:
        put_f32(&p, in->press_pa);
        put_f32(&p, in->temp_c);
        break;
    case FDT_SENSOR_POWER:
        put_f32(&p, in->vbat_v);
        put_f32(&p, in->ibat_a);
        break;
    default:
        return -1;
    }
    return (long)need;
}

long fdt_dec_sensor(const uint8_t *buf, size_t len, fdt_sensor_t s,
                    fdt_input_t *in)
{
    const size_t need = fdt_sensor_bytes(s);
    if (buf == NULL || in == NULL || need == 0 || len < need) {
        return -1;
    }

    const uint8_t *p = buf;
    switch (s) {
    case FDT_SENSOR_IMU:
        in->ax_mps2 = get_f32(&p);
        in->ay_mps2 = get_f32(&p);
        in->az_mps2 = get_f32(&p);
        in->wx_rps  = get_f32(&p);
        in->wy_rps  = get_f32(&p);
        in->wz_rps  = get_f32(&p);
        in->mx_ut   = get_f32(&p);
        in->my_ut   = get_f32(&p);
        in->mz_ut   = get_f32(&p);
        break;
    case FDT_SENSOR_GPS:
        in->gps_lat_deg = get_f32(&p);
        in->gps_lon_deg = get_f32(&p);
        in->gps_alt_m   = get_f32(&p);
        in->vn_mps      = get_f32(&p);
        in->ve_mps      = get_f32(&p);
        in->vd_mps      = get_f32(&p);
        break;
    case FDT_SENSOR_BARO:
        in->press_pa = get_f32(&p);
        in->temp_c   = get_f32(&p);
        break;
    case FDT_SENSOR_POWER:
        in->vbat_v = get_f32(&p);
        in->ibat_a = get_f32(&p);
        break;
    default:
        return -1;
    }
    return (long)need;
}
