#ifndef FLEET_DT_LINKBUDGET_H
#define FLEET_DT_LINKBUDGET_H

#include <stddef.h>

/**
 * @file linkbudget.h
 * @brief The link-budget model the abstract says is validated.
 *
 * Section III gives the link: "these sensors are attached to the NAVIO2 board
 * and controlled by the Ardupilot firmware... which typically transfer data in
 * small packets (e.g., 8 bytes per IMU axis), resulting in low network
 * overhead (vs. 100 Mbps of available bandwidth)."
 *
 * Section V-A gives the measurement: "Due to the size of packets (48 KB
 * payload plus camera feed frame), the bandwidth usage increased < 1% for an
 * update window of 125 ms (8 samples per second, per sensor)."
 *
 * Two traps live in those two sentences, and this header is shaped around
 * both.
 *
 * **48 KB is not 48 bytes.** Section IV's 48 bytes is the width of one state
 * in memory; Section V-A's 48 KB is the size of a measured packet payload.
 * They are unrelated quantities that happen to share two digits, and a
 * benchmark that conflated them would pass while reporting nonsense.
 *
 * **"Increased" is not "occupied".** 48 KB at 8 Hz is 3.15 Mbps, which is
 * 3.1% of a 100 Mbps link — it does not reconcile with "< 1%" read as
 * absolute occupancy. Either the 48 KB is per second rather than per window,
 * or "increased" means a ratio against the traffic already on the link. The
 * manuscript does not choose, so neither does this header: it offers both
 * readings as separate functions, and nothing here asserts the published
 * figure. See ambiguity 5 in docs/spec/paper-claims.md.
 */

/** One traffic source: a payload published at a rate, by some vessels. */
typedef struct {
    const char *name;          /**< Label for reports and plots. */
    size_t      payload_bytes; /**< Application payload per publication. */
    double      hz;            /**< Publications per second, per vessel. */
    unsigned    vessels;       /**< Vessels emitting this stream. */
} fdt_stream_t;

/** A shared wireless link. */
typedef struct {
    double capacity_bps;   /**< Nominal capacity, bits per second. */
    double overhead_ratio; /**< Framing overhead as a fraction, e.g. 0.20 for
                                MQTT over TCP over Wi-Fi headers. Zero counts
                                application payload only. */
} fdt_link_t;

/**
 * @brief Application bit rate of one stream.
 * @return payload_bytes * 8 * hz * vessels, or 0.0 for a NULL stream. Carries
 *         no framing overhead: that belongs to the link, not the stream.
 */
double fdt_link_stream_bps(const fdt_stream_t *s);

/**
 * @brief Total bit rate of several streams, including the link's overhead.
 * @param streams  Array of @p n streams.
 * @return The sum, scaled by (1 + overhead_ratio); 0.0 on a NULL argument.
 */
double fdt_link_total_bps(const fdt_link_t *link,
                          const fdt_stream_t *streams, size_t n);

/**
 * @brief Fraction of the link the given streams occupy.
 *
 * The absolute reading of Section V-A. A return value above 1.0 means the
 * offered load exceeds the link, which is a real answer and not an error.
 *
 * @return Occupied fraction, or -1.0 when the link capacity is not positive.
 */
double fdt_link_utilization(const fdt_link_t *link,
                            const fdt_stream_t *streams, size_t n);

/**
 * @brief How much the added streams grow the existing usage.
 *
 * The relative reading of Section V-A: "the bandwidth usage increased < 1%"
 * is a ratio against what was already flowing, which is why the baseline is
 * an argument rather than a constant.
 *
 * @param baseline  Traffic already on the link.
 * @param nb        Baseline stream count.
 * @param added     Traffic the digital twin adds.
 * @param na        Added stream count.
 * @return added / baseline as a fraction, or -1.0 when the baseline carries
 *         no traffic — a relative increase over nothing is not a number.
 */
double fdt_link_increase(const fdt_link_t *link,
                         const fdt_stream_t *baseline, size_t nb,
                         const fdt_stream_t *added, size_t na);

/**
 * @brief How many vessels fit inside a share of the link.
 * @param streams  Per-vessel stream profile; the @c vessels field of each is
 *                 ignored, since the answer is a vessel count.
 * @param n        Stream count.
 * @param budget   Share of the link to stay within, e.g. 0.5 for half.
 * @return The largest vessel count that fits, 0 when not even one does, and
 *         UINT_MAX when the profile costs nothing.
 */
unsigned fdt_link_max_vessels(const fdt_link_t *link,
                              const fdt_stream_t *streams, size_t n,
                              double budget);

#endif /* FLEET_DT_LINKBUDGET_H */
