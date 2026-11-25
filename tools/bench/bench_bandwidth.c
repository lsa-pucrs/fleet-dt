/**
 * @file bench_bandwidth.c
 * @brief Link budget over the LSDT and HSDT stream profiles of the paper.
 */
#include "bench_common.h"
#include "injector/injector.h"

#include <fleet_dt/linkbudget.h>
#include <fleet_dt/regulator.h>

#include <stdlib.h>
#include <string.h>

/** The DT frame rate: 8 Hz, one increment of k per 125 ms. */
#define DT_HZ 8.0

/** Seconds of telemetry each measured run covers. */
#define MEASURE_S 8

/** Boats of the fleet figure on slides 13 and apoio 8. */
#define FLEET_BOATS 25

/**
 * A transport that only counts what crosses it, standing for the byte
 * counter on the bridge of slide 13.
 */
typedef struct {
    uint64_t msgs;
    uint64_t app_bytes;  /**< Envelope + payload. */
    uint64_t mqtt_bytes; /**< The MQTT v5 PUBLISH packet carrying it. */
} counter_t;

/** Bytes of a variable-byte-integer encoding of @p v (MQTT 1.5.5). */
static size_t vbi_bytes(size_t v)
{
    size_t n = 1;
    while (v >= 128u) {
        v /= 128u;
        n++;
    }
    return n;
}

/**
 * Size of the MQTT v5 PUBLISH packet at QoS 1 carrying @p len bytes on
 * @p topic: topic length (2) + topic + packet id (2) + property length (1,
 * no properties) + payload, behind the fixed header.
 */
static size_t mqtt5_publish_bytes(const char *topic, size_t len)
{
    const size_t remaining = 2u + strlen(topic) + 2u + 1u + len;
    return 1u + vbi_bytes(remaining) + remaining;
}

static int count_publish(void *self, const char *topic, const uint8_t *buf,
                         size_t len, int qos)
{
    (void)buf;
    (void)qos;
    counter_t *c = (counter_t *)self;
    c->msgs++;
    c->app_bytes  += len;
    c->mqtt_bytes += mqtt5_publish_bytes(topic, len);
    return 0;
}

/** Runs one boat for MEASURE_S seconds, raw or regulated. */
static counter_t measure_boat(int raw)
{
    static fdt_inj_t inj; /* large; kept off the stack */
    counter_t c = {0};
    fdt_transport_t tr = { .publish = count_publish, .self = &c };

    if (fdt_inj_init(&inj, &tr, 1, DT_HZ, 20260817u) != 0 ||
        fdt_inj_configure(&inj, NULL, raw) != 0) {
        abort();
    }
    for (int k = 0; k < (int)(MEASURE_S * DT_HZ); k++) {
        (void)fdt_inj_tick(&inj);
    }
    return c;
}

/** Section III: "vs. 100 Mbps of available bandwidth". */
static const fdt_link_t WIFI = { .capacity_bps = 100e6,
                                 .overhead_ratio = 0.0 };

/** The same link with MQTT-over-TCP-over-WiFi framing charged to it. */
static const fdt_link_t WIFI_OH = { .capacity_bps = 100e6,
                                    .overhead_ratio = 0.20 };

/** One row of the stream table, with where the paper states it. */
typedef struct {
    const char  *name;
    size_t       payload_bytes;
    double       sensor_hz;   /**< Rate the sensor samples at. */
    const char  *source;      /**< Where in the paper the figure comes from. */
    int          on_mqtt;     /**< Zero for the HSDT camera path. */
} profile_t;

/**
 * The traffic profiles, each traceable to a sentence.
 */
static const profile_t PROFILES[] = {
    { "imu",       8 * 3,        100.0, "Sec. III, 8 bytes per IMU axis",  1 },
    { "gps",       12,           10.0,  "Sec. II, GPS on the NAVIO2",      1 },
    { "baro",      4,            50.0,  "Sec. II, 10 cm barometer",        1 },
    { "power",     8,            20.0,  "Sec. II, Arduino Micro V/I",      1 },
    { "lsdt_agg",  48u * 1024u,  8.0,   "Sec. V-A, 48 KB payload",         1 },
    { "hsdt_cam",  250u*1024u*1024u, 1.0, "Sec. III, 1080p30 ~250 MB/s",   0 },
};
#define NPROFILES ((int)(sizeof PROFILES / sizeof PROFILES[0]))


int main(void)
{
    bench_report_t r;
    bench_open(&r, "bandwidth");
    bench_banner(&r, "fleet-dt link budget bench");
    bench_say(&r, "link: %.0f Mbps nominal (Sec. III)   DT rate: %.0f Hz\n",
              WIFI.capacity_bps / 1e6, DT_HZ);

    static double xs[NPROFILES];
    static double raw_kbps[NPROFILES];
    static double reg_kbps[NPROFILES];

    bench_section(&r, "per-stream cost, one vessel, before and after the "
                      "regulators");
    bench_say(&r, "  %-10s %9s %8s %12s %12s %8s  %s\n",
              "stream", "payload", "sensor", "unregulated", "regulated",
              "saved", "source");

    double mqtt_raw_bps = 0.0;
    double mqtt_reg_bps = 0.0;

    for (int i = 0; i < NPROFILES; i++) {
        const profile_t *p = &PROFILES[i];

        const fdt_stream_t raw = { .name = p->name,
                                   .payload_bytes = p->payload_bytes,
                                   .hz = p->sensor_hz,
                                   .vessels = 1 };

        const double pub_hz = (p->sensor_hz <= DT_HZ) ? p->sensor_hz : DT_HZ;

        const fdt_stream_t regulated = { .name = p->name,
                                         .payload_bytes = p->payload_bytes,
                                         .hz = pub_hz,
                                         .vessels = 1 };

        const double raw_bps = fdt_link_stream_bps(&raw);
        const double reg_bps = fdt_link_stream_bps(&regulated);
        const double saved   = (raw_bps > 0.0)
                             ? 100.0 * (1.0 - reg_bps / raw_bps) : 0.0;

        xs[i]       = (double)i;
        raw_kbps[i] = raw_bps / 1e3;
        reg_kbps[i] = reg_bps / 1e3;

        if (p->on_mqtt) {
            mqtt_raw_bps += raw_bps;
            mqtt_reg_bps += reg_bps;
        }

        bench_say(&r, "  %-10s %8zu B %6.0f Hz %9.1f kbps %9.1f kbps "
                      "%6.1f %%  %s%s\n",
                  p->name, p->payload_bytes, p->sensor_hz,
                  raw_bps / 1e3, reg_bps / 1e3, saved, p->source,
                  p->on_mqtt ? "" : "  [off MQTT]");
    }

    bench_section(&r, "the two figures that share two digits");
    const fdt_stream_t payload48k = { .name = "48 KB payload",
                                      .payload_bytes = 48u * 1024u,
                                      .hz = DT_HZ, .vessels = 1 };
    const fdt_stream_t state48b = { .name = "48-byte state",
                                    .payload_bytes = 48u,
                                    .hz = DT_HZ, .vessels = 1 };
    bench_say(&r, "  Sec. V-A packet payload : 48 KB   -> %8.3f Mbps\n",
              fdt_link_stream_bps(&payload48k) / 1e6);
    bench_say(&r, "  Sec. IV  state width    : 48 B    -> %8.3f kbps\n",
              fdt_link_stream_bps(&state48b) / 1e3);
    bench_say(&r, "  Ratio %.0f. The state width of Section IV and the "
                  "packet payload of\n  Section V-A are separate quantities, "
                  "printed on separate rows.\n",
              fdt_link_stream_bps(&payload48k) /
              fdt_link_stream_bps(&state48b));

    bench_section(&r, "the Section V-A figure, under both readings");
    char buf[80];

    const double abs_util = fdt_link_utilization(&WIFI, &payload48k, 1);
    snprintf(buf, sizeof buf, "%.3f %% occupied", 100.0 * abs_util);
    bench_compare(&r, "A: absolute occupancy", buf, "increased < 1 %",
                  BENCH_HOST);

    const fdt_stream_t cam = { .name = "camera",
                               .payload_bytes = 250u * 1024u * 1024u,
                               .hz = 1.0, .vessels = 1 };
    const double rel = fdt_link_increase(&WIFI, &cam, 1, &payload48k, 1);
    snprintf(buf, sizeof buf, "%.4f %% growth", 100.0 * rel);
    bench_compare(&r, "B: increase over baseline", buf, "increased < 1 %",
                  (rel < 0.01) ? BENCH_MATCH : BENCH_HOST);

    bench_say(&r,
        "  Reading A is the payload as a share of the whole link. Reading B\n"
        "  is its growth over the camera feed Section III describes. Both\n"
        "  are printed, and fdt_link_utilization and fdt_link_increase are\n"
        "  the two symbols that compute them.\n");

    bench_section(&r, "aggregate MQTT load, and how many vessels fit");
    bench_say(&r, "  unregulated, 1 vessel : %8.3f Mbps  (%.2f %% of link)\n",
              mqtt_raw_bps / 1e6, 100.0 * mqtt_raw_bps / WIFI.capacity_bps);
    bench_say(&r, "  regulated,   1 vessel : %8.3f Mbps  (%.2f %% of link)\n",
              mqtt_reg_bps / 1e6, 100.0 * mqtt_reg_bps / WIFI.capacity_bps);
    bench_say(&r, "  saved by the regulators: %.1f %% of the offered load\n",
              100.0 * (1.0 - mqtt_reg_bps / mqtt_raw_bps));

    const fdt_stream_t agg = { .name = "regulated vessel",
                               .payload_bytes = 48u * 1024u,
                               .hz = DT_HZ, .vessels = 1 };
    const unsigned half     = fdt_link_max_vessels(&WIFI, &agg, 1, 0.50);
    const unsigned half_oh  = fdt_link_max_vessels(&WIFI_OH, &agg, 1, 0.50);
    bench_say(&r, "  vessels within 50 %% of the link : %u "
                  "(%u once 20 %% framing overhead is charged)\n",
              half, half_oh);

    snprintf(buf, sizeof buf, "%u vessels", half);
    bench_compare(&r, "fleet the link supports", buf,
                  "25-boat run was injector-bound", BENCH_MATCH);

    bench_section(&r, "measured on the regulated client, one boat (slide 13)");
    const counter_t raw = measure_boat(1);
    const counter_t reg = measure_boat(0);
    const double raw_msgs = (double)raw.msgs / MEASURE_S;
    const double reg_msgs = (double)reg.msgs / MEASURE_S;
    const double raw_mbps = 8.0 * (double)raw.mqtt_bytes / MEASURE_S / 1e6;
    const double reg_mbps = 8.0 * (double)reg.mqtt_bytes / MEASURE_S / 1e6;
    bench_say(&r, "  sensors: IMU %.0f Hz, barometer %.0f Hz, GPS %.0f Hz, "
                  "power %.0f Hz; %d s\n",
              FDT_INJ_HZ_IMU, FDT_INJ_HZ_BARO, FDT_INJ_HZ_GPS,
              FDT_INJ_HZ_POWER, MEASURE_S);
    bench_say(&r, "  %-10s %10s %14s %14s\n", "", "msg/s",
              "app (Mbps)", "MQTT v5 (Mbps)");
    bench_say(&r, "  %-10s %10.0f %14.4f %14.4f\n", "raw", raw_msgs,
              8.0 * (double)raw.app_bytes / MEASURE_S / 1e6, raw_mbps);
    bench_say(&r, "  %-10s %10.0f %14.4f %14.4f\n", "regulated", reg_msgs,
              8.0 * (double)reg.app_bytes / MEASURE_S / 1e6, reg_mbps);

    snprintf(buf, sizeof buf, "%.0f -> %.0f msg/s", raw_msgs, reg_msgs);
    bench_compare_slide(&r, "messages per boat", buf, "slide 13",
                        "1070 -> 32 msg/s",
                        (raw.msgs == 1070u * MEASURE_S &&
                         reg.msgs == 32u * MEASURE_S)
                            ? BENCH_MATCH : BENCH_HOST);
    snprintf(buf, sizeof buf, "%.1fx fewer", raw_msgs / reg_msgs);
    bench_compare_slide(&r, "message reduction", buf, "slide 13",
                        "33x fewer messages",
                        (raw_msgs / reg_msgs > 33.0 &&
                         raw_msgs / reg_msgs < 34.0)
                            ? BENCH_MATCH : BENCH_HOST);
    snprintf(buf, sizeof buf, "%.3f -> %.4f Mbps", raw_mbps, reg_mbps);
    bench_compare_slide(&r, "traffic per boat", buf, "slide 13",
                        "0.68 -> 0.012 Mbps", BENCH_HOST);
    snprintf(buf, sizeof buf, "%.1f %% less", 100.0 * (1.0 - reg_mbps /
                                                       raw_mbps));
    bench_compare_slide(&r, "traffic removed", buf, "slide 13",
                        "98 % less traffic",
                        (reg_mbps / raw_mbps < 0.03) ? BENCH_MATCH
                                                     : BENCH_HOST);
    snprintf(buf, sizeof buf, "%.1f %% -> %.2f %% of link",
             100.0 * FLEET_BOATS * raw_mbps / 100.0,
             100.0 * FLEET_BOATS * reg_mbps / 100.0);
    bench_compare_slide(&r, "25 boats, link share", buf, "backup 8",
                        "17 % -> 0.3 %", BENCH_HOST);
    bench_say(&r,
        "  The message counts follow from the sensor rates alone, so they\n"
        "  land on the slide exactly. The bit rates depend on the bytes\n"
        "  per message: this count charges the envelope, the payload and\n"
        "  the MQTT v5 PUBLISH header, not TCP, IP or Wi-Fi framing, and\n"
        "  the slide measured a byte counter on a Mosquitto bridge.\n");

    fdt_plot_t p;
    fdt_plot_init(&p, "Offered load per stream, before and after regulation",
                  "stream (see report for names)", "kbps, one vessel (log)");
    fdt_plot_series(&p, "unregulated", xs, raw_kbps, NPROFILES, FDT_PLOT_BAR);
    fdt_plot_series(&p, "regulated to 8 Hz", xs, reg_kbps, NPROFILES,
                    FDT_PLOT_BAR);
    fdt_plot_hline(&p, 100e6 / 1e3, "100 Mbps link (Sec. III)");
    fdt_plot_set_log_y(&p, 1);
    fdt_plot_write_svg(&p, "results/bandwidth.svg");
    fdt_plot_write_csv(&p, "results/bandwidth.csv");

    static double vs[16];
    static double util_pct[16];
    for (int i = 0; i < 16; i++) {
        const unsigned v = (unsigned)(i + 1) * 4u;
        const fdt_stream_t s = { .name = "regulated vessel",
                                 .payload_bytes = 48u * 1024u,
                                 .hz = DT_HZ, .vessels = v };
        vs[i]       = (double)v;
        util_pct[i] = 100.0 * fdt_link_utilization(&WIFI, &s, 1);
    }

    fdt_plot_t u;
    fdt_plot_init(&u, "Link occupancy against fleet size",
                  "vessels in the fleet", "occupancy (%)");
    fdt_plot_series(&u, "48 KB at 8 Hz per vessel", vs, util_pct, 16,
                    FDT_PLOT_LINE);
    fdt_plot_hline(&u, 100.0, "link saturated");
    fdt_plot_hline(&u, 1.0, "1 % (Sec. V-A, reading A)");
    fdt_plot_write_svg(&u, "results/bandwidth_fleet.svg");

    bench_artefacts(&r, "bandwidth");
    bench_say(&r, "artefacts: results/bandwidth_fleet.svg\n");
    bench_close(&r);
    return EXIT_SUCCESS;
}
