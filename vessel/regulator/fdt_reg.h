/*
 * Fleet-DT v2 vessel regulator and DTI program, core (paper Defs. demand, table, regulator;
 * SPEC_v2.md Sections 3, 4.1, 4.2 and 6). No dependency on Mosquitto: the plugin
 * (fdt_regulator_plugin.c) feeds the core with the raw messages, the tables, the MESSAGE_OUT
 * events of the data bridge and the ticks, and supplies the publication function and the state
 * of the data-bridge sockets; the unit test (tests/c/test_reg.c) drives the core with its own
 * clock and socket values.
 *
 * Step t (vessel clock): phase 1 at start(t) = epoch + (t-1) Ts, phase 3 at v(t) = start(t) + T1 + T2,
 * end at start(t+1). Collection window C(t) = [v(t-1), v(t)).
 *
 *  raw message      stream: appended to the step whose collection window holds its arrival;
 *                   output: added to its open output (by the header out:u32, k:u16, n:u16); the output
 *                   completes when its n messages arrived and joins the step whose collection window
 *                   holds its last message. At v(t), open outputs whose first message arrived before
 *                   v(t-1) are dropped and counted (inc).
 *  start(t)         closes step t-1 (discards its unreleased messages and counts the discarded messages
 *                   of admitted units), publishes boat/b<i>/demand of step t with the report of t-1 and
 *                   boat/b<i>/regulator with the log of t-1.
 *  levels of t      stored when it arrives before v(t); ignored (and counted) later.
 *  v(t)             applies the table of t or the default table (R only, arrival bound over Ts) and
 *                   builds the release list: level 2 units, then level 1 units, each by the topic
 *                   order "o", outputs by index (first k complete outputs, whole outputs only), stream
 *                   units up to their demand (x, n).
 *  tick             in phase 3, while admitted messages remain and the unsent bytes of the data
 *                   bridge plus the bytes released in this tick stay below L_w, releases the next
 *                   message on boat/b<i>/tx/<topic> with QoS 2 (R) or 0, payload t:u32, seq:u16 + content.
 *
 * Held bytes B: the plugin reports, for each released message that the broker writes to a data
 * bridge socket (MOSQ_EVT_MESSAGE_OUT), its QoS and the exact length of its PUBLISH packet; the core
 * gives the packet the stream offsets [start, end) with start = max(end of the previous packet,
 * bytes written to the socket so far = tcpi_bytes_acked + SIOCOUTQ). In phase 1, the packets with
 * end > tcpi_bytes_acked are unacknowledged; their unacknowledged content bytes and their number,
 * split by QoS, form B. Without tcpi_bytes_acked the core walks back from the newest packet over
 * the SIOCOUTQ bytes.
 */
#ifndef FDT_REG_H
#define FDT_REG_H

#include <stddef.h>
#include <stdint.h>

#include "fdt_dtp2.h"

#define FDT_U_MAX 32          /* units (outputs) per topic and step */
#define FDT_OPEN_MAX 16       /* open outputs per topic */
#define FDT_DATA_MAX 3        /* data bridge sockets (3 in mode b2) */
#define FDT_TRK_MAX 16384     /* tracked unacknowledged packets per socket */
#define FDT_STEPS 4           /* step ring */

enum fdt_mode { FDT_MODE_FLEETDT = 0, FDT_MODE_B1, FDT_MODE_B2, FDT_MODE_B3, FDT_MODE_B5, FDT_MODE_SLOTTED,
                FDT_MODE_TDMA };
enum fdt_class { FDT_CLASS_BE = 0, FDT_CLASS_AF41 = 1, FDT_CLASS_EF = 2 };

int fdt_mode_parse(const char *s);           /* -1 when unknown */
const char *fdt_mode_name(int mode);
const char *fdt_class_name(int cls);
/* traffic class of topic l in mode b2: R -> EF, other streams -> AF41, outputs -> BE */
int fdt_topic_class(const struct fdt_dtp2 *d, int l);

/* state of the data-bridge sockets, read by the plugin (or injected by the test) */
struct fdt_sock {
	int n;                       /* sockets; 0 when no data bridge is connected */
	struct {
		int64_t nsd;             /* SIOCOUTQNSD: bytes not yet sent */
		int64_t outq;            /* SIOCOUTQ: bytes not yet acknowledged */
		uint64_t acked;          /* tcpi_bytes_acked */
		int has_acked;
	} s[FDT_DATA_MAX];
};

struct fdt_io {
	void *ctx;
	/* publication on the vessel broker; payload copied by the callee */
	int (*publish)(void *ctx, const char *topic, int qos, const void *payload, size_t len);
	/* reads the data-bridge sockets; may be NULL (no socket) */
	void (*sock)(void *ctx, struct fdt_sock *s);
};

struct fdt_msg {
	uint8_t *buf;     /* FDT2_STEP_HEADER bytes of room + content */
	uint32_t len;     /* content bytes */
};

struct fdt_out {
	uint32_t id;
	uint16_t n, got;
	int bad;               /* a message over s_l or a header that disagrees */
	int64_t first_step;    /* collection step of the first message */
	uint64_t x;            /* content bytes */
	struct fdt_msg *m;     /* n slots by message index */
};

struct fdt_tstep {
	int nm, cap;
	struct fdt_msg *m;     /* stream messages in arrival order */
	uint64_t x;
	int no;
	struct fdt_out o[FDT_U_MAX];   /* complete outputs in completion order */
	uint32_t rx;           /* raw messages of the step (stream, or of the complete outputs) */
	uint64_t xrx;          /* content bytes of the messages counted in rx */
	uint32_t over;         /* raw messages dropped over the capacity */
	uint32_t inc;          /* incomplete outputs dropped at v(t) */
	uint32_t seq;          /* next sequence number of the topic in the step */
};

struct fdt_unit {
	uint64_t x;            /* content bytes */
	uint32_t n;            /* messages */
};

struct fdt_table {
	int64_t t;             /* -1: none */
	uint8_t p[FDT_L_MAX];
	uint16_t k[FDT_L_MAX];
	uint8_t o[FDT_L_MAX];
	int has_slot;
	int64_t slot_s, slot_e;
	int64_t at_us;
};

struct fdt_rel {
	uint8_t l, unit;
	uint32_t bytes;        /* PUBLISH packet bytes */
	struct fdt_msg *m;
};

struct fdt_step {
	int64_t t;             /* -1: free slot */
	struct fdt_tstep tp[FDT_L_MAX];
	/* phase 1 */
	int have_dem;
	int nu[FDT_L_MAX], held[FDT_L_MAX];
	struct fdt_unit u[FDT_L_MAX][FDT_U_MAX];
	int64_t late_p1, outq_p1;
	/* phase 3 */
	int applied, have_tab, closed;
	struct fdt_table tb;
	int nadm[FDT_L_MAX];
	struct fdt_rel *rl;
	int nrl, irl, caprl;
	uint32_t relu[FDT_L_MAX][FDT_U_MAX];
	uint32_t rel[FDT_L_MAX], disc[FDT_L_MAX], wh[FDT_L_MAX];
	uint64_t xrel[FDT_L_MAX];
	int64_t late_v, nsd_max, tick_us_max, last_tick;
	int64_t gate_s, gate_e;   /* tdma: own slot (from the table when it holds "slot") */
};

struct fdt_trk_ent {
	uint64_t end;
	uint32_t size, content;
	uint8_t qos;
};

struct fdt_trk {
	uint64_t model_end;
	int head, n;
	struct fdt_trk_ent *e;     /* ring of FDT_TRK_MAX */
	uint64_t lost;             /* entries dropped at the ring capacity */
};

struct fdt_b {
	uint64_t x0, n0, x2, n2, other;   /* other: unacknowledged bytes outside the tracked packets */
};

struct fdt_reg_stats {
	uint64_t raw, bad_raw, over, inc, released, discarded, withheld, stale, steps, tab_ok, tab_default,
	    tab_late, tab_bad, pub_err, outs, resync;
	int64_t late_max_us, tick_us_max;
};

struct fdt_reg {
	const struct fdt_dtp2 *d;
	int vessel, mode, max_msgs;
	struct fdt_io io;
	char tx_topic[FDT_L_MAX][128];
	uint32_t tx_len[FDT_L_MAX];
	char demand_topic[64], log_topic[64];
	struct fdt_step st[FDT_STEPS];
	struct fdt_out open[FDT_L_MAX][FDT_OPEN_MAX];
	int nopen[FDT_L_MAX];
	struct fdt_table tab[FDT_STEPS];
	int64_t next_p1, next_v;
	int64_t p1_off_us;     /* phase 1 of this vessel starts p1_off_us after start(t): i (T1 - eps - 2 T_tick) / (M + 1) */
	int64_t pending;           /* released bytes the broker has not yet written (no MESSAGE_OUT yet) */
	struct fdt_trk trk[FDT_DATA_MAX];
	struct fdt_reg_stats st_;
	char *json;                /* output buffer */
	size_t json_cap;
};

int fdt_reg_init(struct fdt_reg *r, const struct fdt_dtp2 *d, int vessel, int mode, int max_msgs,
                 int64_t now_us, const struct fdt_io *io);
void fdt_reg_free(struct fdt_reg *r);
/* offset of phase 1 after the step start (us); spacing_us < 0: the default, min(2000, T1 / (2M)) times the
 * position of the vessel in the DTP list, so that the demands of the vessels do not start together (hidden
 * vessels would collide); 0: every vessel at start(t) */
void fdt_reg_set_p1_offset(struct fdt_reg *r, int64_t spacing_us);
void fdt_reg_set_p1_offset_guard(struct fdt_reg *r, int64_t spacing_us, int64_t guard_us);
/* raw message of topic l (0-based) */
int fdt_reg_on_raw(struct fdt_reg *r, int l, const void *payload, size_t len, int64_t now_us);
/* boat/b<i>/levels; returns 0 stored, 1 late (ignored), -1 malformed */
int fdt_reg_on_levels(struct fdt_reg *r, const void *payload, size_t len, int64_t now_us);
/* MESSAGE_OUT of a PUBLISH of pkt_bytes bytes (payload_len bytes of payload) to data socket k;
 * written_end = tcpi_bytes_acked + SIOCOUTQ just before the write (has_written = 0 when unknown) */
void fdt_reg_on_out(struct fdt_reg *r, int k, int qos, uint32_t pkt_bytes, uint32_t payload_len,
                    uint64_t written_end, int has_written);
/* the data socket k (re)connected or disconnected: forget its packets */
void fdt_reg_sock_reset(struct fdt_reg *r, int k);
/* runs the events due at now and the pacing; returns the next instant at which a tick is needed */
int64_t fdt_reg_tick(struct fdt_reg *r, int64_t now_us);
/* held bytes of the data sockets from the tracked packets (SPEC_v2.md 4.1 "B") */
void fdt_reg_held(struct fdt_reg *r, const struct fdt_sock *s, struct fdt_b *b);

/* slot of step t in the ring, or NULL */
struct fdt_step *fdt_reg_step(struct fdt_reg *r, int64_t t);

#endif
