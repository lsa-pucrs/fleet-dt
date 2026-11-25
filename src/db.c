#include <fleet_dt/db.h>

#include <fleet_dt/codec.h>

#include <string.h>

static const uint8_t MAGIC[8] = { 'F', 'D', 'T', 'D', 'B', 0, 0, 1 };
#define VERSION 1u
#define MAX_PAYLOAD FDT_WIRE_STATE_BYTES

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t fdt_db_crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        }
    }
    return ~c;
}

static void make_header(uint8_t h[FDT_DB_HEADER_BYTES])
{
    memcpy(h, MAGIC, sizeof MAGIC);
    put_u32(h + 8, VERSION);
    put_u32(h + 12, 0);
}

static int check_header(FILE *f)
{
    uint8_t got[FDT_DB_HEADER_BYTES];
    uint8_t want[FDT_DB_HEADER_BYTES];
    make_header(want);
    return (fread(got, 1, sizeof got, f) == sizeof got &&
            memcmp(got, want, sizeof got) == 0) ? 0 : -1;
}

int fdt_db_open(fdt_db_t *db, const char *path)
{
    if (db == NULL || path == NULL) {
        return -1;
    }
    *db = (fdt_db_t){0};

    FILE *f = fopen(path, "a+b");
    if (f == NULL) {
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return -1;
    }
    const long size = ftell(f);
    if (size == 0) {
        uint8_t h[FDT_DB_HEADER_BYTES];
        make_header(h);
        if (fwrite(h, 1, sizeof h, f) != sizeof h) {
            fclose(f);
            return -1;
        }
    } else {
        rewind(f);
        if (check_header(f) != 0) {
            fclose(f);
            return -1;
        }
        /* "a" mode appends whatever the position; reading moved it only. */
    }
    db->f = f;
    return 0;
}

static int put(fdt_db_t *db, uint32_t tick, uint16_t vessel,
               fdt_db_kind_t kind, const uint8_t *payload, size_t len)
{
    uint8_t rec[8 + MAX_PAYLOAD + 4];
    put_u32(rec, tick);
    put_u16(rec + 4, vessel);
    rec[6] = (uint8_t)kind;
    rec[7] = (uint8_t)len;
    memcpy(rec + 8, payload, len);
    put_u32(rec + 8 + len, fdt_db_crc32(rec, 8 + len));

    const size_t total = 8 + len + 4;
    if (fwrite(rec, 1, total, db->f) != total) {
        return -1;
    }
    db->records++;
    db->payload_bytes += len;
    db->file_bytes += total;
    return 0;
}

int fdt_db_put_state(fdt_db_t *db, uint32_t tick, uint16_t vessel,
                     const fdt_state_t *b)
{
    uint8_t p[FDT_WIRE_STATE_BYTES];
    if (db == NULL || db->f == NULL || b == NULL ||
        fdt_enc_state(b, p, sizeof p) < 0) {
        return -1;
    }
    return put(db, tick, vessel, FDT_DB_STATE, p, sizeof p);
}

int fdt_db_put_goal(fdt_db_t *db, uint32_t tick, uint16_t vessel,
                    const fdt_goal_t *g)
{
    uint8_t p[FDT_WIRE_STATE_BYTES];
    if (db == NULL || db->f == NULL || g == NULL ||
        fdt_enc_state(g, p, sizeof p) < 0) {
        return -1;
    }
    return put(db, tick, vessel, FDT_DB_GOAL, p, sizeof p);
}

int fdt_db_put_act(fdt_db_t *db, uint32_t tick, uint16_t vessel,
                   const fdt_actuation_t *a)
{
    uint8_t p[FDT_WIRE_ACT_BYTES];
    if (db == NULL || db->f == NULL || a == NULL ||
        fdt_enc_act(a, p, sizeof p) < 0) {
        return -1;
    }
    return put(db, tick, vessel, FDT_DB_ACT, p, sizeof p);
}

int fdt_db_put_frame(fdt_db_t *db, uint32_t tick, const fdt_state_t *bs,
                     const fdt_actuation_t *as, size_t n)
{
    if (db == NULL || bs == NULL || n > 0xFFFFu) {
        return -1;
    }
    for (size_t k = 0; k < n; k++) {
        if (fdt_db_put_state(db, tick, (uint16_t)k, &bs[k]) != 0) {
            return -1;
        }
        if (as != NULL && fdt_db_put_act(db, tick, (uint16_t)k, &as[k]) != 0) {
            return -1;
        }
    }
    return 0;
}

int fdt_db_flush(fdt_db_t *db)
{
    if (db == NULL || db->f == NULL) {
        return -1;
    }
    return (fflush(db->f) == 0) ? 0 : -1;
}

int fdt_db_close(fdt_db_t *db)
{
    if (db == NULL || db->f == NULL) {
        return -1;
    }
    const int rc = fclose(db->f);
    db->f = NULL;
    return (rc == 0) ? 0 : -1;
}

int fdt_db_read_open(fdt_db_reader_t *r, const char *path)
{
    if (r == NULL || path == NULL) {
        return -1;
    }
    *r = (fdt_db_reader_t){0};
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return -1;
    }
    if (check_header(f) != 0) {
        fclose(f);
        return -1;
    }
    r->f = f;
    return 0;
}

int fdt_db_next(fdt_db_reader_t *r, fdt_db_record_t *rec)
{
    if (r == NULL || r->f == NULL || rec == NULL) {
        return -1;
    }

    uint8_t buf[8 + MAX_PAYLOAD + 4];
    const size_t h = fread(buf, 1, 8, r->f);
    if (h == 0) {
        return 0;
    }
    if (h < 8) {
        r->torn = 1;
        return 0;
    }

    const size_t len = buf[7];
    const fdt_db_kind_t kind = (fdt_db_kind_t)buf[6];
    const size_t want = (kind == FDT_DB_ACT) ? FDT_WIRE_ACT_BYTES
                                             : FDT_WIRE_STATE_BYTES;
    if ((kind != FDT_DB_STATE && kind != FDT_DB_ACT && kind != FDT_DB_GOAL) ||
        len != want) {
        return -1;
    }

    const size_t body = fread(buf + 8, 1, len + 4, r->f);
    if (body < len + 4) {
        r->torn = 1;
        return 0;
    }
    if (get_u32(buf + 8 + len) != fdt_db_crc32(buf, 8 + len)) {
        /* A bad CRC with nothing after it is a torn tail, not corruption. */
        const int c = fgetc(r->f);
        if (c == EOF) {
            r->torn = 1;
            return 0;
        }
        return -1;
    }

    *rec = (fdt_db_record_t){0};
    rec->tick   = get_u32(buf);
    rec->vessel = get_u16(buf + 4);
    rec->kind   = kind;
    if (kind == FDT_DB_ACT) {
        (void)fdt_dec_act(buf + 8, len, &rec->act);
    } else {
        (void)fdt_dec_state(buf + 8, len, &rec->state);
    }
    r->records++;
    return 1;
}

void fdt_db_read_close(fdt_db_reader_t *r)
{
    if (r != NULL && r->f != NULL) {
        fclose(r->f);
        r->f = NULL;
    }
}
