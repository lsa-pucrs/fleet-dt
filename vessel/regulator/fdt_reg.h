/*
 * Fleet-DT vessel regulator, core (Definition 13 of the paper, SPEC.md Sections 2 and 5).
 * No dependency on Mosquitto: the plugin (fdt_regulator_plugin.c) feeds the core
 * and supplies the publication function, and the unit test drives it with a clock
 * of its own.
 *
 *  - A raw message of topic l belongs to step t when it reaches the regulator in
 *    [start(t-1) + T1 + T2, start(t) + T1 + T2).
 *  - At start(t) + T1 + T2 (phase 3 of step t) the regulator forwards the messages
 *    of step t: the topics of level 2 (QoS 2), then the topics of level 1 (QoS 1),
 *    each group in the DTP order; topics of level 0 are withheld.
 *  - Policing (eq. 5): for each forwarded message of topic l with x bytes (step
 *    header included) the regulator adds c_b x + c_m; the message that would raise
 *    the sum above d_{i,l}(t), and the later messages of topic l in step t, are
 *    withheld, so the messages of a topic leave in arrival order with no gap.
 *  - Without the priority table or the demand table of step t, every message of
 *    step t is withheld (the vessel is outside M(t) or p(t) missed the deadline).
 *  - Each forwarded message carries the 6-byte step header t:u32, s:u16, with s
 *    the sequence number of the message in its topic and step.
 */
#ifndef FDT_REG_H
#define FDT_REG_H

#include <stddef.h>
#include <stdint.h>

#include "fdt_common.h"

/* publication of one forwarded message: topic index l, QoS, payload with the step header */
typedef int (*fdt_reg_publish_fn)(void *ctx, int l, int qos, const uint8_t *payload, size_t len);

struct fdt_reg_msg {
	uint8_t *buf;     /* FDT_STEP_HEADER bytes of room + content */
	uint32_t len;     /* content bytes */
};

struct fdt_reg_queue {
	int n, cap;
	struct fdt_reg_msg *m;
};

/* record of one step: what phase 3 of step t did */
struct fdt_reg_log {
	uint32_t t;
	int have_d, have_p;
	int64_t late_us;               /* delay of the phase-3 tick after start(t) + T1 + T2 */
	int64_t outq;                  /* unsent bytes of the data bridge socket, -1 unknown */
	uint8_t p[FDT_L_MAX];
	uint32_t d[FDT_L_MAX];
	uint32_t rx[FDT_L_MAX];        /* raw messages of step t */
	uint32_t fwd[FDT_L_MAX];       /* forwarded messages */
	uint32_t held[FDT_L_MAX];      /* withheld messages */
	uint32_t drop[FDT_L_MAX];      /* raw messages over the queue capacity */
	uint64_t xfwd[FDT_L_MAX];      /* forwarded bytes, step header included */
	double cost[FDT_L_MAX];        /* sum of c_b x + c_m of the forwarded messages */
};

struct fdt_reg_stats {
	uint64_t raw, forwarded, withheld, dropped, stale, steps, no_d, no_p, bad_tables;
	int64_t late_max_us;
};

struct fdt_reg {
	const struct fdt_dtp *dtp;
	int L;
	int max_msgs;                       /* queue capacity per topic and step */
	/* queues of the two steps that can be open: slot t & 1 */
	int64_t qt[2];
	struct fdt_reg_queue q[2][FDT_L_MAX];
	uint32_t drop[2][FDT_L_MAX];
	uint32_t rx[2][FDT_L_MAX];
	/* demand and priority tables of the last steps: slot t & 3 */
	struct { int64_t t; uint32_t d[FDT_L_MAX]; } dem[4];
	struct { int64_t t; uint8_t p[FDT_L_MAX]; } lev[4];
	int64_t next;                       /* next step whose phase 3 is due */
	int64_t log_offset_us;              /* the log of step t is due at start(t+1) + log_offset */
	struct fdt_reg_log log;
	int log_pending;
	struct fdt_reg_stats st;
	fdt_reg_publish_fn publish;
	void *ctx;
};

int fdt_reg_init(struct fdt_reg *r, const struct fdt_dtp *dtp, int max_msgs, int64_t now_us,
                 fdt_reg_publish_fn publish, void *ctx);
void fdt_reg_free(struct fdt_reg *r);
/* raw message of topic l (0-based) with len content bytes, arrived at now */
int fdt_reg_on_raw(struct fdt_reg *r, int l, const void *payload, size_t len, int64_t now_us);
/* demand table D_i(t) and priority table p(t) of the vessel (wire formats of SPEC.md) */
int fdt_reg_on_demand(struct fdt_reg *r, const void *payload, size_t len);
int fdt_reg_on_levels(struct fdt_reg *r, const void *payload, size_t len);
/* Runs the phases that are due at now. outq() is read once per step (may be NULL).
 * Returns 1 when a step log became due and was copied to *out, 0 otherwise.
 * *next_us receives the next instant at which the core needs a tick. */
int fdt_reg_tick(struct fdt_reg *r, int64_t now_us, int64_t (*outq)(void *), void *outq_ctx,
                 struct fdt_reg_log *out, int64_t *next_us);
/* JSON of a step log (boat/b<i>/regulator); returns the length written */
size_t fdt_reg_log_json(const struct fdt_reg *r, const struct fdt_reg_log *lg, char *buf, size_t n);

#endif
