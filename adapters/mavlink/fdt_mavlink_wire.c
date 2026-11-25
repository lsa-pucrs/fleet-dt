#include "mavlink/fdt_mavlink_wire.h"

#include <string.h>

#define STX_V1 0xFEu
#define STX_V2 0xFDu
#define SIGNED 0x01u /* incompat_flags: a 13-byte signature follows */

/** Full payload length and CRC_EXTRA of the messages known here. */
typedef struct {
    uint32_t msgid;
    uint8_t  len;
    uint8_t  crc_extra;
} known_t;

static const known_t KNOWN[] = {
    { FDT_MAVMSG_HEARTBEAT,             9,  50 },
    { FDT_MAVMSG_GPS_RAW_INT,          52,  24 },
    { FDT_MAVMSG_SCALED_PRESSURE,      16, 115 },
    { FDT_MAVMSG_LOCAL_POSITION_NED,   28, 185 },
    { FDT_MAVMSG_RC_CHANNELS_OVERRIDE, 38, 124 },
    { FDT_MAVMSG_SCALED_IMU2,          24,  76 },
    { FDT_MAVMSG_BATTERY_STATUS,       54, 154 },
};
#define NKNOWN (sizeof KNOWN / sizeof KNOWN[0])

static const known_t *lookup(uint32_t msgid)
{
    for (size_t i = 0; i < NKNOWN; i++) {
        if (KNOWN[i].msgid == msgid) {
            return &KNOWN[i];
        }
    }
    return NULL;
}

uint16_t fdt_mav_crc(uint16_t crc, const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t t = (uint8_t)(p[i] ^ (uint8_t)(crc & 0xFFu));
        t ^= (uint8_t)(t << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^
                         ((uint16_t)t << 3) ^ ((uint16_t)t >> 4));
    }
    return crc;
}

int fdt_mav_crc_extra(uint32_t msgid)
{
    const known_t *k = lookup(msgid);
    return (k != NULL) ? k->crc_extra : -1;
}

/** Header length and whole-frame length of the frame at buf, if known yet. */
static int frame_size(const uint8_t *b, size_t have, size_t *hdr,
                      size_t *total)
{
    if (b[0] == STX_V1) {
        if (have < 2) {
            return 0;
        }
        *hdr = 6;
        *total = 6 + (size_t)b[1] + 2;
        return 1;
    }
    if (have < 3) {
        return 0;
    }
    *hdr = 10;
    *total = 10 + (size_t)b[1] + 2 + ((b[2] & SIGNED) ? 13 : 0);
    return 1;
}

int fdt_mav_parse(fdt_mav_parser_t *p, uint8_t byte, fdt_mav_msg_t *out)
{
    if (p == NULL || out == NULL) {
        return 0;
    }
    if (p->have == 0 && byte != STX_V1 && byte != STX_V2) {
        return 0; /* hunting for a start byte */
    }
    p->buf[p->have++] = byte;

    for (;;) {
        size_t hdr = 0, total = 0;
        if (!frame_size(p->buf, p->have, &hdr, &total) || p->have < total) {
            return 0;
        }

        const uint8_t *b = p->buf;
        const size_t len = b[1];
        const uint32_t msgid = (hdr == 6)
            ? b[5]
            : ((uint32_t)b[7] | ((uint32_t)b[8] << 8) |
               ((uint32_t)b[9] << 16));
        const known_t *k = lookup(msgid);

        int ok = 0;
        if (k == NULL) {
            p->unknown++;
        } else {
            uint16_t crc = fdt_mav_crc(0xFFFFu, b + 1, hdr - 1 + len);
            crc = fdt_mav_crc(crc, &k->crc_extra, 1);
            const uint16_t got = (uint16_t)(b[hdr + len] |
                                            (b[hdr + len + 1] << 8));
            ok = (crc == got) && len <= k->len;
            if (!ok) {
                p->bad_crc++;
            }
        }

        if (ok) {
            memset(out, 0, sizeof *out);
            out->version = (hdr == 6) ? 1 : 2;
            out->len    = (uint8_t)len;
            out->seq    = b[(hdr == 6) ? 2 : 4];
            out->sysid  = b[(hdr == 6) ? 3 : 5];
            out->compid = b[(hdr == 6) ? 4 : 6];
            out->msgid  = msgid;
            memcpy(out->payload, b + hdr, len); /* the rest stays zero */
            p->frames++;
            p->have = 0;
            return 1;
        }

        /* Unknown or corrupt: a known frame consumes its length; a corrupt
         * one gives up its start byte and the scan resumes behind it. */
        const size_t drop = (k == NULL) ? total : 1;
        memmove(p->buf, p->buf + drop, p->have - drop);
        p->have -= drop;
        while (p->have > 0 && p->buf[0] != STX_V1 && p->buf[0] != STX_V2) {
            memmove(p->buf, p->buf + 1, p->have - 1);
            p->have--;
        }
        if (p->have == 0) {
            return 0;
        }
    }
}

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int16_t i16(const uint8_t *p)
{
    return (int16_t)u16(p);
}

static int32_t i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static float f32(const uint8_t *p)
{
    const uint32_t u = (uint32_t)i32(p);
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}

int fdt_mav_dispatch(fdt_mav_ingest_t *ing, const fdt_mav_msg_t *m)
{
    if (ing == NULL || m == NULL) {
        return 0;
    }
    const uint8_t *p = m->payload;
    switch (m->msgid) {
    case FDT_MAVMSG_SCALED_IMU2:
        fdt_mav_on_scaled_imu(ing, i16(p + 4), i16(p + 6), i16(p + 8),
                              i16(p + 10), i16(p + 12), i16(p + 14),
                              i16(p + 16), i16(p + 18), i16(p + 20));
        return 1;
    case FDT_MAVMSG_GPS_RAW_INT:
        fdt_mav_on_gps_raw(ing, i32(p + 8), i32(p + 12), i32(p + 16));
        return 1;
    case FDT_MAVMSG_LOCAL_POSITION_NED:
        fdt_mav_on_local_ned(ing, f32(p + 16), f32(p + 20), f32(p + 24));
        return 1;
    case FDT_MAVMSG_SCALED_PRESSURE:
        fdt_mav_on_scaled_pressure(ing, f32(p + 4), i16(p + 12));
        return 1;
    case FDT_MAVMSG_BATTERY_STATUS: {
        /* Pack voltage: the sum of the cells reported. Ardupilot without
         * cell monitoring puts the pack in voltages[0], the rest UINT16_MAX,
         * and the sum is then the pack itself. */
        uint32_t mv = 0;
        for (int c = 0; c < 10; c++) {
            const uint16_t v = u16(p + 10 + 2 * c);
            if (v != 0xFFFFu) {
                mv += v;
            }
        }
        fdt_mav_on_battery(ing, (uint16_t)((mv > 0xFFFEu) ? 0xFFFEu : mv),
                           i16(p + 30));
        return 1;
    }
    default:
        return 0;
    }
}

/** Wraps a payload as a MAVLink 2 frame, truncating trailing zeros. */
static long frame_v2(uint8_t *buf, size_t cap, uint8_t seq, uint8_t sysid,
                     uint8_t compid, uint32_t msgid, const uint8_t *payload,
                     size_t len)
{
    const known_t *k = lookup(msgid);
    if (buf == NULL || k == NULL) {
        return -1;
    }
    while (len > 1 && payload[len - 1] == 0) {
        len--;
    }
    if (10 + len + 2 > cap) {
        return -1;
    }
    buf[0] = STX_V2;
    buf[1] = (uint8_t)len;
    buf[2] = 0; /* incompat flags: unsigned */
    buf[3] = 0;
    buf[4] = seq;
    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)msgid;
    buf[8] = (uint8_t)(msgid >> 8);
    buf[9] = (uint8_t)(msgid >> 16);
    memcpy(buf + 10, payload, len);
    uint16_t crc = fdt_mav_crc(0xFFFFu, buf + 1, 9 + len);
    crc = fdt_mav_crc(crc, &k->crc_extra, 1);
    buf[10 + len]     = (uint8_t)crc;
    buf[10 + len + 1] = (uint8_t)(crc >> 8);
    return (long)(10 + len + 2);
}

long fdt_mav_enc_heartbeat(uint8_t *buf, size_t cap, uint8_t seq,
                           uint8_t sysid, uint8_t compid)
{
    uint8_t p[9] = {0};
    p[4] = 6; /* MAV_TYPE_GCS */
    p[5] = 8; /* MAV_AUTOPILOT_INVALID */
    p[6] = 0; /* base_mode */
    p[7] = 4; /* MAV_STATE_ACTIVE */
    p[8] = 3; /* mavlink_version */
    return frame_v2(buf, cap, seq, sysid, compid, FDT_MAVMSG_HEARTBEAT, p,
                    sizeof p);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

long fdt_mav_enc_rc_override(uint8_t *buf, size_t cap, uint8_t seq,
                             uint8_t sysid, uint8_t compid,
                             uint8_t target_sys, uint8_t target_comp,
                             const fdt_actuation_t *a)
{
    if (a == NULL) {
        return -1;
    }
    uint16_t throttle = 0, cage = 0;
    fdt_mav_actuation_to_rc(a, &throttle, &cage);

    uint8_t p[38] = {0};
    /* Channels 1 to 8: UINT16_MAX leaves a channel as it is. */
    for (int c = 0; c < 8; c++) {
        put16(p + 2 * c, 0xFFFFu);
    }
    put16(p + 0, cage);      /* channel 1: steering */
    put16(p + 4, throttle);  /* channel 3: throttle */
    p[16] = target_sys;
    p[17] = target_comp;
    /* Channels 9 to 18 stay 0: ignored. */
    return frame_v2(buf, cap, seq, sysid, compid,
                    FDT_MAVMSG_RC_CHANNELS_OVERRIDE, p, sizeof p);
}
