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
 *  start(t) + off  demand table D_i(t) = t:u32, L:u8, Q_i:u32, d_1..d_L:u32 on boat/b<i>/demand (QoS 0),
 *                  with d_l = ceil(c_b (x_l + 6 n_l) + c_m,l n_l), n_l = ceil(r_l (Ts + J_l)), x_l = n_l s_l
 *                  (the jitter allowance J_l is inside d_l), and d_l = 0 for a topic with no raw message
 *                  in the last `idle` steps. off = 0 by default (no phase-1 offset). With demand
 *                  "external" the regulator publishes nothing and takes D_i(t) from the DTI program
 *                  (vessel/dti/fdt_dti) as it passes through the vessel broker on boat/b<i>/demand.
 *                  Backlog: Q_i(t) = ceil(c_b o_i(t) + c_k n_i(t)) (fdt_preg_backlog), o_i = SIOCOUTQ of the
 *                  data-bridge socket + the bytes that the broker still holds in its output queue for the
 *                  data-bridge client (bq of io.sock; patch 0004), read at the start of phase 1, and n_i the
 *                  PUBLISH packets with at least one byte among those o_i bytes: the core keeps a ring of the
 *                  lengths of the PUBLISH packets written to each data-bridge socket in write order
 *                  (fdt_preg_on_out, from MOSQ_EVT_MESSAGE_OUT) and walks it from the newest packet back
 *                  until the lengths sum to o_i or more; n_i = the packets visited (all of the ring when
 *                  it holds fewer bytes, 0 when o_i = 0). Q_i = 0 without a socket.
 *                  The DTI program is another process and cannot read that socket, so it writes Q_i = 0
 *                  and the plugin overwrites the field in place (fdt_preg_stamp_demand) when the table
 *                  enters the vessel broker, before the control bridge carries it to the GS.
 *  p(t)            t:u32, L:u8, p_1..p_L:u8 on boat/b<i>/levels; stored when it arrives before v(t),
 *                  ignored (and counted) later.
 *  v(t)            without p(t), forwards nothing of step t. Otherwise, per topic l in arrival order:
 *                  level 0 holds every message; levels 1 and 2 forward the messages while the sum of
 *                  c_b (x + 6) + c_m,l stays within the d_l(t) of D_i(t) (the value sent is the value
 *                  policed); the message that would exceed it and the later messages of topic l in
 *                  step t stay. Then it publishes the level-2 messages, then the level-1 messages
 *                  (strict priority), each level in order of reception, all in the same tick (no
 *                  pacing), on boat/b<i>/tx/<topic> with payload t:u32, s:u16 + content, s = index of
 *                  the message in its topic and step from 0. Both levels use MQTT QoS 0 (fdt_preg_qos), and
 *                  the data bridges subscribe with QoS 0: no MQTT acknowledgment crosses the channel; the
 *                  GS still removes repeated copies by the pair (t, s) (the bridges keep cleansession
 *                  true).
 *  start(t+1)      end of step t: every message of step t not forwarded is logged (one JSON line per
 *                  message in the vessel log file: step, topic, sequence number, bytes, reason) and
 *                  is never forwarded later; the step summary goes to boat/b<i>/regulator (QoS 0) and,
 *                  with a step log, to one JSON line of that file (the vessel-side record of p(t), d,
 *                  the received, forwarded and held messages and the forwarded cost of each topic).
 *
 * Ablations: paper_nopolice (B5a) forwards every message of a topic of level 1 or 2; paper_noeos (B5b)
 * has no end-of-step rule: a message not forwarded in its step stays queued and is offered again at
 * v(t+1), v(t+2), ... before the newer messages of its topic, under the table and the policing of that
 * step, with its own step index in the header (so the GS sees it as late).
 *
 * Baselines of the paper stack (Table S3 of the paper; the mode names are internal). They keep the clock, the payload t:u32, s:u16, the
 * collection windows, the end-of-step rule, the vessel log and the step summary of mode paper, take no
 * priority table (a table that arrives is stored and ignored), and publish every message with QoS 1 on
 * the vessel broker; the data bridges of the paper stack subscribe with QoS 0 (fleetdt/gsconf.py), so the
 * messages cross the channel with QoS 0 as those of the levels do, and the GS records repeated copies
 * (by t, s) in the log of the operation and excludes them from the step (Corollary 2). The modes take the names paper_<x> because b1, b2, b3 and tdma name the modes of the older
 * core (fdt_reg.c, as2 stack: DTP v2, 1-based clock, JSON tables).
 *  paper_b1    (B1, and B9 with one data bridge per uplink topic, fleetdt/gsconf.py --per-topic) no
 *              regulation: each raw message is forwarded when it reaches the regulator, with the step
 *              t = floor((now - epoch) / Ts) of that instant in its header.
 *  paper_b2    (B2) paper_b1 on boat/b<i>/tx2/<class>/<topic>, one data bridge per EDCA access category
 *              (fleetdt/gsconf.py --b2, system/nftables-fdt.nft): ef (AC_VO) for the topic named "state"
 *              (without one: the first topic of "order"), be (AC_BE) for the video stripes (the layer
 *              topics of "video_layers"), af41 (AC_VI) for the other telemetry topics.
 *  paper_b3    (B3) strict priority without phases 1 and 2: at v(t) every message of step t, topic by topic
 *              in sigma_t, each topic in order of reception.
 *  b8          (B7, rate control from y(t-1) as the DCC) paper_b3 with the budget
 *              B(t) = U y(t-1) T3 / (Ts m): the messages leave in the order of paper_b3 while the sum of
 *              c_b (x + 6) + c_m,l stays within B(t); the first message that does not fit and every
 *              message after it in that order stay (reason budget: no message of a later topic passes
 *              it). y, U and m come from the last fleet feedback (below), y = "C0" of the DTP before the
 *              first one, U = plugin_opt_U (default: "U" of the DTP), m = m(t) of the feedback (the vessels
 *              of the DTP before the first one).
 *  paper_tdma  (B8, equal slots of phase 3 above the MAC) fixed equal slots of phase 3 in the order of
 *              "vessels" of the DTP: vessel at position k of M sends in [v(t) + k T3 / M, v(t) + (k + 1) T3 / M);
 *              in its slot it releases the messages of step t in the order of paper_b3 at every tick
 *              (tick_us, 1 ms) while the unsent bytes of the data bridge (SIOCOUTQNSD of the socket plus
 *              the bytes that the broker still holds for the bridge client, bq) plus the bytes released in
 *              the tick stay below lw_bytes (16384); the messages left at the end of the slot stay (reason
 *              slot).
 * Fleet feedback (B3, B7, B8: paper_b3, b8, paper_tdma): fleet/feedback, published by the GS once per step
 * at start(t) + T1 (fleetdt/feedback.py) and carried by the control bridge: t:u32 (the step t that the
 * feedback serves), y:f64 (y(t - 1), bytes per step; NaN when it did not reach the GS by start(t) + T1),
 * U:f32 (0: keep the own U), m:u8 (m(t) = |M(t)|, the vessels whose demand table reached the GS in phase 1
 * of step t; 0: keep), L:u8, sigma_1..sigma_L:u8 (sigma_t, 1-based topics). The last feedback received
 * before v(t) (normally that of step t) sets sigma_t and the y and m of B(t); a missing feedback keeps the
 * last values (sigma of the DTP, y = C0 and m = the vessels of the DTP before the first one).
 */
#ifndef FDT_REG_PAPER_H
#define FDT_REG_PAPER_H

#include <stdint.h>
#include <stdio.h>

#include "fdt_paper.h"
#include "fdt_reg.h"   /* struct fdt_io, struct fdt_sock */

#define FDT_P_STEPS 4

/* modes and the paper label they run: PAPER Fleet-DT and B4, B6; NOPOLICE B5a; NOEOS B5b; B1 B1 and B9; B2 B2;
 * B3 B3; B8 (mode b8) B7; TDMA B8 */
enum fdt_pmode { FDT_PMODE_PAPER = 0, FDT_PMODE_NOPOLICE = 1, FDT_PMODE_NOEOS = 2, FDT_PMODE_B1 = 3,
                 FDT_PMODE_B2 = 4, FDT_PMODE_B3 = 5, FDT_PMODE_B8 = 6, FDT_PMODE_TDMA = 7, FDT_PMODE_N };

/* reasons of the vessel log */
enum fdt_preason { FDT_PR_NONE = 0, FDT_PR_FWD, FDT_PR_NO_TABLE, FDT_PR_NO_DEMAND, FDT_PR_LEVEL0, FDT_PR_POLICE,
                   FDT_PR_OVERFLOW, FDT_PR_RESYNC, FDT_PR_EOS, FDT_PR_SHUTDOWN, FDT_PR_BUDGET, FDT_PR_SLOT,
                   FDT_PR_N };

/* MQTT QoS of a forwarded message of level 1 or 2: 0 for both levels (the order of the levels in phase 3
 * gives the priority; the GS deduplicates by (t, s)) */
#define FDT_P_QOS_LEVEL2 0
#define FDT_P_QOS_LEVEL1 0
/* PUBLISH packets per data-bridge socket in the ring of the backlog (n_i) */
#define FDT_P_RING 16384
int fdt_preg_qos(int level);

/* paper | paper_nopolice | paper_noeos | paper_b1 | paper_b2 | paper_b3 | b8 (alias paper_b8) | paper_tdma;
 * -1 otherwise */
int fdt_pmode_parse(const char *s);
const char *fdt_pmode_name(int mode);
const char *fdt_preason_name(int reason);
/* the mode takes the priority table (paper and its ablations) */
int fdt_pmode_table(int mode);
/* traffic class of topic l in paper_b2 (B2): 2 ef (AC_VO) for the topic named "state" (without one, the first
 * topic of "order"), 0 be (AC_BE) for a video stripe, 1 af41 (AC_VI) for the other topics (the order of
 * fdt_class) */
int fdt_pclass(const struct fdt_pdtp *d, int l);

#define FDT_FEEDBACK_MIN 18   /* t:u32, y:f64, U:f32, m:u8, L:u8 */

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
	int64_t p_late_at;             /* arrival of a table after v(t), before the end of the step (-1: none) */
	int have_Q;                    /* Q_i(t) computed at the start of phase 1 */
	uint32_t Q;                    /* backlog Q_i(t) of the demand table */
	int64_t bo, bn;                /* its o_i(t) and n_i(t) (-1: no data-bridge socket) */
	int64_t bq;                    /* the part of o_i(t) in the output queue of the broker (-1: unknown) */
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
	double budget;                 /* b8: B(t) (-1: not b8) */
	double y_used;                 /* b8: the y(t-1) of B(t) */
	int64_t fb_t;                  /* step of the feedback used at v(t) (-1: none) */
	int64_t gate_s, gate_e;        /* paper_tdma: slot of the vessel */
};

struct fdt_preg_stats {
	uint64_t raw, fwd, late_fwd, logged, overflow, steps, tab_ok, tab_none, tab_late, tab_bad, dem_pub,
	    dem_ext, dem_late, dem_bad, dem_foreign, resync, pub_err, log_err, fb_ok, fb_old, fb_bad;
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
	FILE *step_log;                 /* step summaries (the JSON of boat/b<i>/regulator), one line per step */
	struct fdt_preg_stats s;
	char *json;
	size_t json_cap;
	/* baselines */
	uint8_t sigma[FDT_L_MAX];       /* sigma_t, 0-based topic indices */
	double U;                       /* b8 (B7): own U */
	int have_fb;
	int64_t fb_t;                   /* step of the last feedback */
	double fb_y, fb_U;              /* y of the last feedback with a measurement (C0 before), its U */
	int fb_m;
	int64_t lw_bytes, tick_us;      /* paper_tdma (B8) */
	int slot_pos, slot_n;
	int64_t gate_t;                 /* paper_tdma: step whose slot is open or ahead (-1: none) */
	int gate_left;                  /* messages of that step not yet released */
	uint8_t *scratch;               /* paper_b1, paper_b2: header + content of one message */
	size_t scratch_cap;
	char tx2_topic[FDT_L_MAX][160];
	/* backlog: per data-bridge socket, a ring of the lengths of the PUBLISH packets written to it */
	uint32_t *ring;                 /* FDT_DATA_MAX x FDT_P_RING */
	int ring_head[FDT_DATA_MAX];    /* next slot */
	int ring_n[FDT_DATA_MAX];       /* packets held (<= FDT_P_RING) */
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
	FILE *step_log;          /* step summaries, one JSON line per step (NULL: off); not closed by the core */
	double U;                /* b8: U before a feedback with U (<= 0: "U" of the DTP) */
	int64_t lw_bytes;        /* paper_tdma: L_w (16384) */
	int64_t tick_us;         /* paper_tdma: release period in the slot (1000) */
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
/* backlog Q_i(t) = ceil(c_b o + c_k k) at this instant (o: SIOCOUTQ of the data-bridge sockets, k: QoS 1
 * messages without PUBACK, both from io.sock; Q = 0 without a socket); kept in the record of step t when the
 * ring holds it */
uint32_t fdt_preg_backlog(struct fdt_preg *r, int64_t t, int64_t now_us);
/* a PUBLISH packet of len bytes written to data-bridge socket k (MOSQ_EVT_MESSAGE_OUT), in write order */
void fdt_preg_on_out(struct fdt_preg *r, int k, uint32_t len);
/* data-bridge socket k connected or disconnected: empties its ring */
void fdt_preg_ring_reset(struct fdt_preg *r, int k);
/* n: the packets of the ring of socket k, newest first, whose lengths first sum to o or more (all of the ring
 * when it holds fewer bytes; 0 when o <= 0) */
int64_t fdt_preg_ring_count(const struct fdt_preg *r, int k, int64_t o);
/* demand table of the DTI program entering the vessel broker (demand external): writes Q_i(t) of
 * fdt_preg_backlog into its Q field in place (the table keeps its length); returns 0, or -1 malformed */
int fdt_preg_stamp_demand(struct fdt_preg *r, void *payload, size_t len, int64_t now_us);
/* fleet/feedback; returns 0 stored, 1 older than the last one (ignored), -1 malformed */
int fdt_preg_on_feedback(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us);
/* runs the events due at now; returns the next instant at which a tick is needed */
int64_t fdt_preg_tick(struct fdt_preg *r, int64_t now_us);
/* record of step t in the ring, or NULL */
struct fdt_pstep *fdt_preg_step(struct fdt_preg *r, int64_t t);

#endif
