/*
 * Fleet-DT vessel regulator, mode paper: the "Vessel regulator" Definition of the paper (Section IV),
 * with the wire formats of fleetdt/model.py and the 0-based step clock of fleetdt/as_service.py.
 * No dependency on Mosquitto: fdt_regulator_plugin.c feeds the core with the raw messages, the priority
 * tables, the demand tables of an external DTI program and the ticks; tests/c/test_reg_paper.c drives it
 * with its own clock.
 *
 * Step t (vessel clock): start(t) = epoch + t Ts, phase 3 at v(t) = start(t) + T1 + T2, end at start(t+1).
 * A raw message of topic l belongs to step t when it reaches the regulator in [v(t-1), v(t)).
 *
 *  start(t) + off  demand table D_i(t) = t:u32, L:u8, d_1..d_L:u32 on boat/b<i>/demand (QoS 1), with
 *                  d_l = ceil(c_b (x_l + 6 n_l) + c_m,l n_l), n_l = ceil(r_l (Ts + J_l)), x_l = n_l s_l
 *                  (the jitter allowance J_l is inside d_l), and d_l = 0 for a topic with no raw message
 *                  in the last `idle` steps. off = 0 by default (no phase-1 offset). With demand
 *                  "external" the regulator publishes nothing and takes D_i(t) from the DTI program
 *                  (vessel/dti/fdt_dti) as it passes through the vessel broker on boat/b<i>/demand.
 *  p(t)            t:u32, L:u8, p_1..p_L:u8 on boat/b<i>/levels; stored when it arrives before v(t),
 *                  ignored (and counted) later.
 *  v(t)            without p(t), forwards nothing of step t. Otherwise, per topic l in arrival order:
 *                  level 0 holds every message; levels 1 and 2 forward the messages while the sum of
 *                  c_b (x + 6) + c_m,l stays within the d_l(t) of D_i(t) (the value sent is the value
 *                  policed); the message that would exceed it and the later messages of topic l in
 *                  step t stay. Then it publishes the level-2 messages with QoS 2 and the level-1
 *                  messages with QoS 1, each level in order of reception, all in the same tick (no
 *                  pacing), on boat/b<i>/tx/<topic> with payload t:u32, s:u16 + content, s = index of
 *                  the message in its topic and step from 0.
 *  start(t+1)      end of step t: every message of step t not forwarded is logged (one JSON line per
 *                  message in the vessel log file: step, topic, sequence number, bytes, reason) and
 *                  is never forwarded later; the step summary goes to boat/b<i>/regulator (QoS 0).
 *
 * Ablations: paper_nopolice (B5a) forwards every message of a topic of level 1 or 2; paper_noeos (B5b)
 * has no end-of-step rule: a message not forwarded in its step stays queued and is offered again at
 * v(t+1), v(t+2), ... before the newer messages of its topic, under the table and the policing of that
 * step, with its own step index in the header (so the GS sees it as late).
 */
#ifndef FDT_REG_PAPER_H
#define FDT_REG_PAPER_H

#include <stdint.h>
#include <stdio.h>

#include "fdt_paper.h"
#include "fdt_reg.h"   /* struct fdt_io, struct fdt_sock */

#define FDT_P_STEPS 4

enum fdt_pmode { FDT_PMODE_PAPER = 0, FDT_PMODE_NOPOLICE = 1, FDT_PMODE_NOEOS = 2 };

/* reasons of the vessel log */
enum fdt_preason { FDT_PR_NONE = 0, FDT_PR_FWD, FDT_PR_NO_TABLE, FDT_PR_NO_DEMAND, FDT_PR_LEVEL0, FDT_PR_POLICE,
                   FDT_PR_OVERFLOW, FDT_PR_RESYNC, FDT_PR_EOS, FDT_PR_SHUTDOWN, FDT_PR_N };

int fdt_pmode_parse(const char *s);          /* paper | paper_nopolice | paper_noeos; -1 otherwise */
const char *fdt_pmode_name(int mode);
const char *fdt_preason_name(int reason);

struct fdt_pmsg {
	uint8_t *buf;          /* FDT_STEP_HEADER bytes of room + content */
	uint32_t len;          /* content bytes */
	int64_t t;             /* step of the message */
	uint16_t seq;          /* index of the message in its topic and step */
	uint8_t reason;        /* enum fdt_preason, set at v(t) */
	uint64_t arr;          /* order of reception over every topic */
};

struct fdt_pq {
	int n, cap;
	struct fdt_pmsg *m;    /* FIFO in order of reception */
};

struct fdt_pstep {
	int64_t t;             /* -1: free slot */
	int have_d, have_p, applied;
	uint32_t d[FDT_L_MAX];
	uint8_t p[FDT_L_MAX];
	int64_t p_at, late_p1, late_v, outq, nsd;
	uint32_t seq[FDT_L_MAX];       /* next sequence number of the topic in the step */
	uint32_t rx[FDT_L_MAX];        /* raw messages of the step */
	uint32_t fwd[FDT_L_MAX];       /* messages of the step forwarded in the step */
	uint32_t held[FDT_L_MAX];      /* messages of the step logged at the end of the step */
	uint32_t police[FDT_L_MAX];    /* of these, held by the policing */
	uint32_t drop[FDT_L_MAX];      /* raw messages over the queue capacity (logged at once) */
	uint32_t late_fwd[FDT_L_MAX];  /* paper_noeos: messages of earlier steps forwarded at v(t) */
	uint32_t carried[FDT_L_MAX];   /* paper_noeos: messages of step <= t still queued at the end of t */
	uint64_t xfwd[FDT_L_MAX];      /* forwarded bytes at v(t), step header included */
	double cost[FDT_L_MAX];        /* channel bytes of the messages forwarded at v(t) */
};

struct fdt_preg_stats {
	uint64_t raw, fwd, late_fwd, logged, overflow, steps, tab_ok, tab_none, tab_late, tab_bad, dem_pub,
	    dem_ext, dem_late, dem_bad, dem_foreign, resync, pub_err, log_err;
	uint64_t reason[FDT_PR_N];
	int64_t late_max_us;
};

struct fdt_preg {
	const struct fdt_pdtp *d;
	int vessel, mode, demand_ext, max_msgs, idle;
	int64_t p1_off_us;
	double jitter_s[FDT_L_MAX];
	uint32_t dmax[FDT_L_MAX];       /* d_l of an active topic */
	struct fdt_io io;
	char tx_topic[FDT_L_MAX][128];
	char demand_topic[64], log_topic[64];
	struct fdt_pq q[FDT_L_MAX];
	int64_t last_raw[FDT_L_MAX];    /* step of the last raw message of topic l */
	struct fdt_pstep st[FDT_P_STEPS];
	int64_t next_p1, next_v, next_end;
	uint64_t arr;
	FILE *log;                      /* vessel log, one JSON line per message not forwarded; NULL: off */
	struct fdt_preg_stats s;
	char *json;
	size_t json_cap;
};

/* Options of the core; fdt_preg_opts_default() fills the defaults. */
struct fdt_preg_opts {
	int mode;                /* enum fdt_pmode */
	int demand_ext;          /* 0: the regulator publishes D_i(t); 1: it takes D_i(t) from boat/b<i>/demand */
	int max_msgs;            /* messages per topic and step (4096) */
	int idle;                /* steps without raw messages after which d_l = 0 (8; 0: never) */
	double jitter_s;         /* J_l of the topics without "jitter" in the DTP (0.002 s) */
	int64_t p1_off_us;       /* offset of the demand after start(t) (0) */
	FILE *log;               /* vessel log (NULL: off); the core does not close it */
};

void fdt_preg_opts_default(struct fdt_preg_opts *o);
int fdt_preg_init(struct fdt_preg *r, const struct fdt_pdtp *d, int vessel, const struct fdt_preg_opts *o,
                  int64_t now_us, const struct fdt_io *io);
/* logs the queued messages (reason shutdown) and frees the core */
void fdt_preg_free(struct fdt_preg *r);
/* raw message of topic l (0-based); returns 0 queued, 1 over the capacity (logged), -1 invalid */
int fdt_preg_on_raw(struct fdt_preg *r, int l, const void *payload, size_t len, int64_t now_us);
/* boat/b<i>/levels; returns 0 stored, 1 late (ignored), -1 malformed */
int fdt_preg_on_levels(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us);
/* boat/b<i>/demand seen on the vessel broker; returns 0 stored (external) or own table (internal),
 * 1 late (external), 2 a table that the regulator did not publish (internal), -1 malformed */
int fdt_preg_on_demand(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us);
/* runs the events due at now; returns the next instant at which a tick is needed */
int64_t fdt_preg_tick(struct fdt_preg *r, int64_t now_us);
/* record of step t in the ring, or NULL */
struct fdt_pstep *fdt_preg_step(struct fdt_preg *r, int64_t t);

#endif
