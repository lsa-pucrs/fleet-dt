/*
 * Delivery time of the GS MQTT broker (delta_GS of the paper, Section VI-E; supplementary Section S6):
 * core of the plugin fdt_gsstamp.so, without Mosquitto.
 *
 * The broker calls fdt_gss_in when it has received a PUBLISH (MOSQ_EVT_MESSAGE_IN) and fdt_gss_out when it
 * writes the PUBLISH to a subscriber (MOSQ_EVT_MESSAGE_OUT). The core keys each uplink message of a data
 * bridge by its topic boat/b<i>/tx/<name> and the step header (t: u32, s: u16, big endian) of its payload,
 * so one reception matches every delivery of the same message. Per step t of the step header it counts the
 * receptions, the deliveries, the deliveries without a reception in the table (unmatched), and the largest
 * and the mean delivery time, and writes one JSON line per step:
 *
 *   {"t": 812, "n_in": 120, "n_out": 240, "unmatched": 0, "max_us": 412.3, "mean_us": 51.0}
 *
 * A step line is written when its slot of the ring of FDT_GSS_STEPS steps is needed for a later step, and
 * at fdt_gss_flush_all (plugin cleanup).
 */
#ifndef FDT_GSSTAMP_CORE_H
#define FDT_GSSTAMP_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define FDT_GSS_TAB 16384   /* receptions kept for matching (power of 2) */
#define FDT_GSS_PROBE 8     /* slots probed per key */
#define FDT_GSS_STEPS 16    /* steps aggregated at the same time */

struct fdt_gss_ent {
	uint64_t h;      /* hash of the topic; 0: free */
	int64_t t;       /* step of the step header */
	uint32_t seq;    /* sequence number of the step header */
	int64_t in_ns;   /* reception */
};

struct fdt_gss_step {
	int64_t t;       /* -1: free */
	uint64_t n_in, n_out, unmatched;
	int64_t max_ns;
	double sum_ns;
};

struct fdt_gss {
	struct fdt_gss_ent tab[FDT_GSS_TAB];
	struct fdt_gss_step st[FDT_GSS_STEPS];
	FILE *log;                  /* JSON lines of the steps (NULL: none) */
	uint64_t n_in, n_out, unmatched, lost, lines, log_err;
	int64_t max_ns;             /* largest delivery time of every step */
	int64_t max_t;              /* step of max_ns */
};

void fdt_gss_init(struct fdt_gss *g, FILE *log);

/* 0 and the key of an uplink message of a data bridge (topic boat/b<digits>/tx/<name>, payload with the
 * 6-byte step header); -1 otherwise */
int fdt_gss_key(const char *topic, const void *payload, size_t len, uint64_t *h, int64_t *t, uint32_t *seq);

/* reception and delivery of a message at now_ns (any clock, the same for both) */
void fdt_gss_in(struct fdt_gss *g, const char *topic, const void *payload, size_t len, int64_t now_ns);
void fdt_gss_out(struct fdt_gss *g, const char *topic, const void *payload, size_t len, int64_t now_ns);

/* writes the lines of every step still held, in the order of t */
void fdt_gss_flush_all(struct fdt_gss *g);

#endif
