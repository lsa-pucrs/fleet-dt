/**
 * @file test_linkbudget.c
 * @brief The link-budget model, and the two readings of Section V-A.
 *
 * Claims covered: C4, C10, and the arithmetic behind C20. See
 * docs/spec/paper-claims.md, trap 1 and ambiguity 5.
 *
 * Nothing here asserts the paper's "< 1%". The arithmetic is asserted; the
 * published figure is printed beside it and left for the reader, because
 * asserting it would mean picking one reading of an ambiguous sentence on the
 * authors' behalf.
 */
#include <fleet_dt/linkbudget.h>

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>

/** The link Section III names: 100 Mbps of available bandwidth. */
static const fdt_link_t WIFI = { .capacity_bps = 100e6, .overhead_ratio = 0.0 };

/** Section III's own example of a small packet: 8 bytes per IMU axis. */
static void test_lsdt_is_small(void)
{
    const fdt_stream_t imu = { .name = "imu", .payload_bytes = 8 * 3,
                               .hz = 8.0, .vessels = 1 };

    assert(fabs(fdt_link_stream_bps(&imu) - 24.0 * 8.0 * 8.0) < 1e-9);

    /* "Low network overhead vs. 100 Mbps of available bandwidth": 1536 bps
     * against 100 Mbps, three orders of magnitude below one percent. */
    const double util = fdt_link_utilization(&WIFI, &imu, 1);
    assert(fabs(util - 1536.0 / 100e6) < 1e-15);
    assert(util > 0.0 && util < 1e-4);
    printf("IMU stream, one vessel: %.6f%% of a 100 Mbps link\n",
           100.0 * util);

    /* A stream nobody emits costs nothing. */
    const fdt_stream_t none = { .name = "none", .payload_bytes = 1000,
                                .hz = 8.0, .vessels = 0 };
    assert(fdt_link_stream_bps(&none) == 0.0);
    assert(fdt_link_stream_bps(NULL) == 0.0);
}

/**
 * Trap 1: the 48 KB of Section V-A is not the 48 bytes of Section IV.
 *
 * A test that used 48 bytes here would pass and report a figure three orders
 * of magnitude too small.
 */
static void test_48kb_is_not_48_bytes(void)
{
    const fdt_stream_t lsdt = { .name = "lsdt", .payload_bytes = 48u * 1024u,
                                .hz = 8.0, .vessels = 1 };
    const fdt_stream_t state = { .name = "state", .payload_bytes = 48u,
                                 .hz = 8.0, .vessels = 1 };

    const double kb_bps = fdt_link_stream_bps(&lsdt);
    const double b_bps  = fdt_link_stream_bps(&state);

    assert(fabs(kb_bps - 48.0 * 1024.0 * 8.0 * 8.0) < 1e-6);
    assert(fabs(kb_bps / b_bps - 1024.0) < 1e-9);
    printf("48 KB payload = %.3f Mbps; 48-byte state = %.3f kbps\n",
           kb_bps / 1e6, b_bps / 1e3);
}

/** Ambiguity 5: absolute occupancy and relative increase, side by side. */
static void test_two_readings(void)
{
    const fdt_stream_t lsdt = { .name = "lsdt", .payload_bytes = 48u * 1024u,
                                .hz = 8.0, .vessels = 1 };

    /* Reading A, absolute occupancy: 3.1% of a 100 Mbps link. */
    const double util = fdt_link_utilization(&WIFI, &lsdt, 1);
    assert(fabs(util - fdt_link_stream_bps(&lsdt) / WIFI.capacity_bps) < 1e-12);
    assert(util > 0.031 && util < 0.032);
    printf("reading A, absolute occupancy      : %.3f%%  "
           "(paper says the increase was < 1%%)\n", 100.0 * util);

    /* Reading B, growth over traffic already flowing. The camera feed is the
     * obvious baseline: Section III puts 1080p30 at roughly 250 MB/s before
     * it is moved off MQTT. */
    const fdt_stream_t baseline = { .name = "existing",
                                    .payload_bytes = 1u << 20,
                                    .hz = 8.0, .vessels = 1 };
    const double inc = fdt_link_increase(&WIFI, &baseline, 1, &lsdt, 1);
    assert(fabs(inc - fdt_link_stream_bps(&lsdt) /
                      fdt_link_stream_bps(&baseline)) < 1e-12);
    printf("reading B, increase over baseline  : %.3f%%\n", 100.0 * inc);

    /* An increase over nothing is not a number, and must not silently read
     * as zero growth. */
    assert(fdt_link_increase(&WIFI, NULL, 0, &lsdt, 1) < 0.0);
    assert(fdt_link_increase(NULL, &baseline, 1, &lsdt, 1) < 0.0);
}

/** Framing overhead scales the offered load, not the capacity. */
static void test_overhead(void)
{
    const fdt_stream_t lsdt = { .name = "lsdt", .payload_bytes = 48u * 1024u,
                                .hz = 8.0, .vessels = 1 };
    const fdt_link_t with_oh = { .capacity_bps = 100e6,
                                 .overhead_ratio = 0.20 };

    const double bare = fdt_link_utilization(&WIFI, &lsdt, 1);
    const double oh   = fdt_link_utilization(&with_oh, &lsdt, 1);
    assert(fabs(oh - bare * 1.2) < 1e-9);

    /* A link with no capacity has no defined utilisation. */
    const fdt_link_t dead = { .capacity_bps = 0.0, .overhead_ratio = 0.0 };
    assert(fdt_link_utilization(&dead, &lsdt, 1) < 0.0);
}

/** How many vessels a share of the link holds. */
static void test_max_vessels(void)
{
    const fdt_stream_t lsdt = { .name = "lsdt", .payload_bytes = 48u * 1024u,
                                .hz = 8.0, .vessels = 1 };

    const unsigned half = fdt_link_max_vessels(&WIFI, &lsdt, 1, 0.50);
    printf("vessels fitting in 50%% of the link : %u\n", half);
    assert(half >= 15 && half <= 16);

    /* The vessels field of the profile is ignored: the answer is a count. */
    const fdt_stream_t many = { .name = "lsdt", .payload_bytes = 48u * 1024u,
                                .hz = 8.0, .vessels = 99 };
    assert(fdt_link_max_vessels(&WIFI, &many, 1, 0.50) == half);

    /* A free profile is unbounded; a nonsensical budget is zero. */
    const fdt_stream_t free_stream = { .name = "free", .payload_bytes = 0,
                                       .hz = 8.0, .vessels = 1 };
    assert(fdt_link_max_vessels(&WIFI, &free_stream, 1, 0.5) == UINT_MAX);
    assert(fdt_link_max_vessels(&WIFI, &lsdt, 1, 0.0) == 0);
    assert(fdt_link_max_vessels(NULL, &lsdt, 1, 0.5) == 0);
}

int main(void)
{
    test_lsdt_is_small();
    test_48kb_is_not_48_bytes();
    test_two_readings();
    test_overhead();
    test_max_vessels();

    printf("test_linkbudget: ok\n");
    return 0;
}
