/**
 * @file test_db.c
 * @brief The database of states and actuation (Figure 4), and the 23 KB per
 *        minute per vessel it grows by (Section IV).
 */
#define _POSIX_C_SOURCE 200809L

#include <fleet_dt/db.h>
#include <fleet_dt/tick.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define PATH "fleet_dt_test_db.tmp"

static fdt_state_t state_at(uint32_t tick, uint16_t v)
{
    fdt_state_t b = {0};
    b.lat_deg = -30.05f + 1e-6f * (float)tick;
    b.lon_deg = -51.17f - 1e-6f * (float)v;
    b.yaw_deg = (float)(tick % 360);
    b.surge_mps = 1.0f + 0.01f * (float)v;
    return b;
}

/** One minute at 8 Hz, one vessel, states only: the figure of Section IV. */
static void test_growth_per_minute(void)
{
    remove(PATH);
    fdt_db_t db;
    assert(fdt_db_open(&db, PATH) == 0);

    const uint32_t per_minute = (uint32_t)(60LL * 1000000000LL / FDT_TICK_NS);
    assert(per_minute == 480);
    for (uint32_t t = 0; t < per_minute; t++) {
        const fdt_state_t b = state_at(t, 0);
        assert(fdt_db_put_state(&db, t, 0, &b) == 0);
    }
    assert(db.payload_bytes == 23040); /* Section IV: 23 KB per minute */
    assert(db.file_bytes == 23040 + 480 * FDT_DB_RECORD_OVERHEAD);
    assert(fdt_db_close(&db) == 0);

    printf("one vessel, one minute: %llu B of states (paper: 23 KB), "
           "%llu B on disk\n", 23040ULL,
           (unsigned long long)(23040 + 480 * FDT_DB_RECORD_OVERHEAD +
                                FDT_DB_HEADER_BYTES));
}

/** A fleet frame round-trips, and a reopened file keeps appending. */
static void test_round_trip_and_append(void)
{
    remove(PATH);
    fdt_db_t db;
    assert(fdt_db_open(&db, PATH) == 0);

    fdt_state_t bs[3];
    fdt_actuation_t as[3];
    for (uint16_t v = 0; v < 3; v++) {
        bs[v] = state_at(7, v);
        as[v] = (fdt_actuation_t){ .throttle_pct = 10.0f * v,
                                   .cage_rad = -0.1f * v };
    }
    assert(fdt_db_put_frame(&db, 7, bs, as, 3) == 0);
    const fdt_goal_t g = { .yaw_deg = 90.0f };
    assert(fdt_db_put_goal(&db, 7, 1, &g) == 0);
    assert(fdt_db_close(&db) == 0);

    /* Reopen: the header is checked, records go after the old ones. */
    assert(fdt_db_open(&db, PATH) == 0);
    const fdt_state_t late = state_at(8, 2);
    assert(fdt_db_put_state(&db, 8, 2, &late) == 0);
    assert(fdt_db_close(&db) == 0);

    fdt_db_reader_t r;
    fdt_db_record_t rec;
    assert(fdt_db_read_open(&r, PATH) == 0);
    for (uint16_t v = 0; v < 3; v++) {
        assert(fdt_db_next(&r, &rec) == 1);
        assert(rec.kind == FDT_DB_STATE && rec.tick == 7 && rec.vessel == v);
        assert(memcmp(&rec.state, &bs[v], sizeof bs[v]) == 0);
        assert(fdt_db_next(&r, &rec) == 1);
        assert(rec.kind == FDT_DB_ACT && rec.vessel == v);
        assert(rec.act.throttle_pct == as[v].throttle_pct);
        assert(rec.act.cage_rad == as[v].cage_rad);
    }
    assert(fdt_db_next(&r, &rec) == 1);
    assert(rec.kind == FDT_DB_GOAL && rec.state.yaw_deg == 90.0f);
    assert(fdt_db_next(&r, &rec) == 1);
    assert(rec.tick == 8 && rec.vessel == 2);
    assert(fdt_db_next(&r, &rec) == 0 && r.torn == 0);
    assert(r.records == 8);
    fdt_db_read_close(&r);
}

/** A crash mid-write leaves a torn tail; everything before it survives. */
static void test_torn_tail_and_corruption(void)
{
    remove(PATH);
    fdt_db_t db;
    assert(fdt_db_open(&db, PATH) == 0);
    for (uint32_t t = 0; t < 10; t++) {
        const fdt_state_t b = state_at(t, 0);
        assert(fdt_db_put_state(&db, t, 0, &b) == 0);
    }
    assert(fdt_db_close(&db) == 0);

    const long full = FDT_DB_HEADER_BYTES + 10 * (FDT_DB_RECORD_OVERHEAD + 48);
    assert(truncate(PATH, full - 5) == 0);

    fdt_db_reader_t r;
    fdt_db_record_t rec;
    assert(fdt_db_read_open(&r, PATH) == 0);
    int n = 0;
    while (fdt_db_next(&r, &rec) == 1) {
        n++;
    }
    assert(n == 9 && r.torn == 1);
    fdt_db_read_close(&r);

    /* A flipped byte inside an early record is corruption, not a tail. */
    FILE *f = fopen(PATH, "r+b");
    assert(f != NULL);
    assert(fseek(f, FDT_DB_HEADER_BYTES + 20, SEEK_SET) == 0);
    fputc(0x5A, f);
    fclose(f);
    assert(fdt_db_read_open(&r, PATH) == 0);
    assert(fdt_db_next(&r, &rec) == -1);
    fdt_db_read_close(&r);

    /* A foreign file is refused, for reading and for appending. */
    f = fopen(PATH, "wb");
    assert(f != NULL);
    fputs("not a fleet-dt database", f);
    fclose(f);
    assert(fdt_db_read_open(&r, PATH) == -1);
    assert(fdt_db_open(&db, PATH) == -1);
    remove(PATH);
}

static void test_guards_and_crc(void)
{
    /* The CRC-32 check value of "123456789". */
    assert(fdt_db_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);

    fdt_db_t db = {0};
    const fdt_state_t b = {0};
    assert(fdt_db_open(NULL, PATH) == -1);
    assert(fdt_db_open(&db, NULL) == -1);
    assert(fdt_db_put_state(&db, 0, 0, &b) == -1); /* not open */
    assert(fdt_db_put_act(NULL, 0, 0, NULL) == -1);
    assert(fdt_db_put_frame(&db, 0, NULL, NULL, 1) == -1);
    assert(fdt_db_flush(NULL) == -1);
    assert(fdt_db_close(&db) == -1);
}

int main(void)
{
    test_growth_per_minute();
    test_round_trip_and_append();
    test_torn_tail_and_corruption();
    test_guards_and_crc();

    printf("test_db: ok\n");
    return 0;
}
