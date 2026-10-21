#define _GNU_SOURCE
#include "fdt_reg_paper.h"

#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ names */

static const char *const pmode_names[FDT_PMODE_N] = {"paper", "paper_nopolice", "paper_noeos", "paper_b1",
                                                      "paper_b2", "paper_b3", "b8", "paper_tdma"};
static const char *const reason_names[FDT_PR_N] = {"none", "forwarded", "no_table", "no_demand", "level0",
                                                   "police", "overflow", "resync", "eos", "shutdown",
                                                   "budget", "slot"};
static const char *const pclass_names[3] = {"be", "af41", "ef"};

int fdt_pmode_parse(const char *s)
{
	for (int k = 0; k < FDT_PMODE_N; k++)
		if (!strcmp(s, pmode_names[k]))
			return k;
	if (!strcmp(s, "paper_b8"))
		return FDT_PMODE_B8;
	return -1;
}

int fdt_preg_qos(int level)
{
	return level >= 2 ? FDT_P_QOS_LEVEL2 : level == 1 ? FDT_P_QOS_LEVEL1 : 0;
}

const char *fdt_pmode_name(int mode)
{
	return mode >= 0 && mode < FDT_PMODE_N ? pmode_names[mode] : "?";
}

int fdt_pmode_table(int mode)
{
	return mode == FDT_PMODE_PAPER || mode == FDT_PMODE_NOPOLICE || mode == FDT_PMODE_NOEOS;
}

int fdt_pclass(const struct fdt_pdtp *d, int l)
{
	if (d->topic[l].required || (d->n_required == 0 && d->L > 0 && d->order[0] == l))
		return 2;
	return d->topic[l].video ? 0 : 1;
}

/* paper_b1, paper_b2: forwarded when the raw message arrives */
static int immediate(const struct fdt_preg *r)
{
	return r->mode == FDT_PMODE_B1 || r->mode == FDT_PMODE_B2;
}

/* paper_b3, b8, paper_tdma: strict priority in sigma_t at v(t), without the table */
static int prio_mode(const struct fdt_preg *r)
{
	return r->mode == FDT_PMODE_B3 || r->mode == FDT_PMODE_B8 || r->mode == FDT_PMODE_TDMA;
}

const char *fdt_preason_name(int reason)
{
	return reason >= 0 && reason < FDT_PR_N ? reason_names[reason] : "?";
}

static int policing(const struct fdt_preg *r)
{
	return r->mode != FDT_PMODE_NOPOLICE;
}

static int end_of_step_rule(const struct fdt_preg *r)
{
	return r->mode != FDT_PMODE_NOEOS;
}

/* ------------------------------------------------------------------ steps */

static int slot_of(int64_t t)
{
	return (int)(((t % FDT_P_STEPS) + FDT_P_STEPS) % FDT_P_STEPS);
}

struct fdt_pstep *fdt_preg_step(struct fdt_preg *r, int64_t t)
{
	struct fdt_pstep *s = &r->st[slot_of(t)];
	return s->t == t ? s : NULL;
}

static void step_reset(struct fdt_pstep *s, int64_t t)
{
	memset(s, 0, sizeof *s);
	s->t = t;
	s->late_p1 = s->late_v = -1;
	s->p_late_at = -1;
	s->outq = s->nsd = -1;
	s->bo = s->bn = s->bq = -1;
	s->budget = -1;
	s->fb_t = -1;
	s->gate_s = s->gate_e = -1;
}

static struct fdt_pstep *step_get(struct fdt_preg *r, int64_t t)
{
	struct fdt_pstep *s = &r->st[slot_of(t)];
	if (s->t != t)
		step_reset(s, t);
	return s;
}

/* ------------------------------------------------------------------ output */

static void pub(struct fdt_preg *r, const char *topic, int qos, const void *p, size_t n)
{
	if (r->io.publish && r->io.publish(r->io.ctx, topic, qos, p, n) != 0)
		r->s.pub_err++;
}

struct jbuf {
	char *p;
	size_t n, cap;
};

static void jp(struct jbuf *j, const char *fmt, ...)
{
	if (j->n >= j->cap)
		return;
	va_list ap;
	va_start(ap, fmt);
	int w = vsnprintf(j->p + j->n, j->cap - j->n, fmt, ap);
	va_end(ap);
	if (w > 0)
		j->n += (size_t)w;
	if (j->n >= j->cap)
		j->n = j->cap;
}

#define JARR(j, name, L, fmt, expr) do { \
		jp(j, ",\"%s\":[", name); \
		for (int l_ = 0; l_ < (L); l_++) \
			jp(j, "%s" fmt, l_ ? "," : "", expr); \
		jp(j, "]"); \
	} while (0)

/* one line of the vessel log: a message of step m->t that the regulator did not forward */
static void log_msg(struct fdt_preg *r, int l, const struct fdt_pmsg *m, int reason, int64_t at_step)
{
	r->s.logged++;
	r->s.reason[reason < FDT_PR_N ? reason : 0]++;
	if (!r->log)
		return;
	if (fprintf(r->log,
	            "{\"t\":%" PRId64 ",\"v\":%d,\"l\":%d,\"topic\":\"%s\",\"seq\":%u,\"bytes\":%u,\"reason\":\"%s\","
	            "\"at\":%" PRId64 "}\n",
	            m->t, r->vessel, l + 1, r->d->topic[l].name, (unsigned)m->seq, m->len, fdt_preason_name(reason),
	            at_step) < 0)
		r->s.log_err++;
}

static void log_flush(struct fdt_preg *r)
{
	if (r->log && fflush(r->log))
		r->s.log_err++;
}

/* ------------------------------------------------------------------ queues */

static void q_remove_marked(struct fdt_pq *q)
{
	int w = 0;
	for (int k = 0; k < q->n; k++) {
		if (!q->m[k].buf)
			continue;
		if (w != k)
			q->m[w] = q->m[k];
		w++;
	}
	q->n = w;
}

/* logs and frees the queued messages of steps <= t (every step when t is INT64_MAX) */
static void q_log_upto(struct fdt_preg *r, int64_t t, int force_reason, int64_t at_step)
{
	for (int l = 0; l < r->d->L; l++) {
		struct fdt_pq *q = &r->q[l];
		for (int k = 0; k < q->n; k++) {
			struct fdt_pmsg *m = &q->m[k];
			if (m->t > t)
				continue;
			int reason = force_reason ? force_reason
			                          : (m->reason == FDT_PR_NONE || m->reason == FDT_PR_FWD) ? FDT_PR_EOS : m->reason;
			log_msg(r, l, m, reason, at_step);
			struct fdt_pstep *s = fdt_preg_step(r, m->t);
			if (s) {
				s->held[l]++;
				if (reason == FDT_PR_POLICE)
					s->police[l]++;
			}
			free(m->buf);
			m->buf = NULL;
		}
		q_remove_marked(q);
	}
}

/* ------------------------------------------------------------------ init */

void fdt_preg_opts_default(struct fdt_preg_opts *o)
{
	memset(o, 0, sizeof *o);
	o->mode = FDT_PMODE_PAPER;
	o->max_msgs = 4096;
	o->idle = 8;
	o->jitter_s = 0.002;
	o->U = -1;
	o->lw_bytes = 16384;
	o->tick_us = 1000;
}

static void reset_clock(struct fdt_preg *r, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	int64_t cur = fdt_p_step_at(d, now_us);
	/* the first demand due is the next one whose instant is ahead; the first v(t) due is the end of the
	 * collection window that holds now; the first end due is that of the step before it */
	r->next_p1 = now_us < fdt_p_start_us(d, cur) + r->p1_off_us ? cur : cur + 1;
	r->next_v = fdt_p_collect_step(d, now_us);
	r->next_end = r->next_v - 1;
}

int fdt_preg_init(struct fdt_preg *r, const struct fdt_pdtp *d, int vessel, const struct fdt_preg_opts *o,
                  int64_t now_us, const struct fdt_io *io)
{
	struct fdt_preg_opts def;
	if (!o) {
		fdt_preg_opts_default(&def);
		o = &def;
	}
	memset(r, 0, sizeof *r);
	r->d = d;
	r->vessel = vessel;
	r->mode = o->mode;
	r->demand_ext = o->demand_ext;
	r->max_msgs = o->max_msgs > 0 ? o->max_msgs : 4096;
	if (r->max_msgs > 65536)
		r->max_msgs = 65536;   /* the sequence number is a u16 */
	r->idle = o->idle;
	r->p1_off_us = o->p1_off_us > 0 && o->p1_off_us < d->T1_us ? o->p1_off_us : 0;
	r->log = o->log;
	r->step_log = o->step_log;
	if (io)
		r->io = *io;
	for (int k = 0; k < FDT_P_STEPS; k++)
		step_reset(&r->st[k], -1);
	for (int l = 0; l < d->L; l++) {
		snprintf(r->tx_topic[l], sizeof r->tx_topic[l], "boat/b%d/tx/%s", vessel, d->topic[l].name);
		r->jitter_s[l] = d->topic[l].has_jitter ? d->topic[l].jitter : (o->jitter_s > 0 ? o->jitter_s : 0);
		r->dmax[l] = fdt_p_demand(d, l, r->jitter_s[l]);
		r->last_raw[l] = INT64_MIN / 2;
	}
	for (int j = 0; j < d->L; j++)
		r->sigma[j] = (uint8_t)d->order[j];
	if (r->mode == FDT_PMODE_B2)
		for (int l = 0; l < d->L; l++)
			snprintf(r->tx2_topic[l], sizeof r->tx2_topic[l], "boat/b%d/tx2/%s/%s", vessel,
			         pclass_names[fdt_pclass(d, l)], d->topic[l].name);
	r->U = o->U > 0 ? o->U : d->U;
	r->fb_t = -1;
	r->fb_y = d->C0;
	r->lw_bytes = o->lw_bytes > 0 ? o->lw_bytes : 16384;
	r->tick_us = o->tick_us > 0 ? o->tick_us : 1000;
	r->slot_n = d->M > 0 ? d->M : 1;
	r->slot_pos = 0;
	for (int k = 0; k < d->M; k++)
		if (d->vessels[k] == vessel)
			r->slot_pos = k;
	r->gate_t = -1;
	snprintf(r->demand_topic, sizeof r->demand_topic, "boat/b%d/demand", vessel);
	snprintf(r->log_topic, sizeof r->log_topic, "boat/b%d/regulator", vessel);
	r->json_cap = 16384 + (size_t)d->L * 256;
	r->json = malloc(r->json_cap);
	r->ring = calloc((size_t)FDT_DATA_MAX * FDT_P_RING, sizeof *r->ring);
	if (!r->json || !r->ring) {
		free(r->json);
		free(r->ring);
		r->json = NULL;
		r->ring = NULL;
		return -1;
	}
	reset_clock(r, now_us);
	return 0;
}

void fdt_preg_free(struct fdt_preg *r)
{
	if (r->d) {
		q_log_upto(r, INT64_MAX, FDT_PR_SHUTDOWN, r->next_v);
		log_flush(r);
	}
	for (int l = 0; l < FDT_L_MAX; l++) {
		for (int k = 0; k < r->q[l].n; k++)
			free(r->q[l].m[k].buf);
		free(r->q[l].m);
		r->q[l].m = NULL;
		r->q[l].n = r->q[l].cap = 0;
	}
	free(r->json);
	r->json = NULL;
	free(r->ring);
	r->ring = NULL;
	free(r->scratch);
	r->scratch = NULL;
	r->scratch_cap = 0;
}

/* paper_b1, paper_b2: forwards the raw message at once, in step floor((now - epoch) / Ts) */
static int forward_now(struct fdt_preg *r, int l, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	int64_t t = fdt_p_step_at(d, now_us);
	struct fdt_pstep *s = step_get(r, t);
	s->rx[l]++;
	if (s->seq[l] >= (uint32_t)r->max_msgs) {
		struct fdt_pmsg m = {NULL, (uint32_t)len, t, (uint16_t)s->seq[l], FDT_PR_OVERFLOW, 0};
		s->drop[l]++;
		r->s.overflow++;
		log_msg(r, l, &m, FDT_PR_OVERFLOW, t);
		return 1;
	}
	size_t n = FDT_STEP_HEADER + len;
	if (n > r->scratch_cap) {
		uint8_t *b = realloc(r->scratch, n);
		if (!b)
			return -1;
		r->scratch = b;
		r->scratch_cap = n;
	}
	uint16_t seq = (uint16_t)s->seq[l]++;
	fdt_put_u32(r->scratch, (uint32_t)t);
	fdt_put_u16(r->scratch + 4, seq);
	memcpy(r->scratch + FDT_STEP_HEADER, payload, len);
	pub(r, r->mode == FDT_PMODE_B2 ? r->tx2_topic[l] : r->tx_topic[l], 1, r->scratch, n);
	s->fwd[l]++;
	s->xfwd[l] += n;
	s->cost[l] += fdt_p_cost(d, l, len);
	r->s.fwd++;
	return 0;
}

/* ------------------------------------------------------------------ inputs */

int fdt_preg_on_raw(struct fdt_preg *r, int l, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	if (l < 0 || l >= d->L || len > 16u * 1024 * 1024)
		return -1;
	r->s.raw++;
	int64_t c = fdt_p_collect_step(d, now_us);
	if (c < r->next_v)   /* never into a step already applied (clock stepped back) */
		c = r->next_v;
	r->last_raw[l] = c;
	if (immediate(r))
		return forward_now(r, l, payload, len, now_us);
	struct fdt_pstep *s = step_get(r, c);
	struct fdt_pq *q = &r->q[l];
	s->rx[l]++;
	if (s->seq[l] >= (uint32_t)r->max_msgs || q->n >= r->max_msgs * FDT_P_STEPS) {
		struct fdt_pmsg m = {NULL, (uint32_t)len, c, (uint16_t)s->seq[l], FDT_PR_OVERFLOW, 0};
		s->drop[l]++;
		r->s.overflow++;
		log_msg(r, l, &m, FDT_PR_OVERFLOW, c);
		return 1;
	}
	if (q->n >= q->cap) {
		int cap = q->cap ? 2 * q->cap : 32;
		struct fdt_pmsg *nm = realloc(q->m, (size_t)cap * sizeof *nm);
		if (!nm)
			return -1;
		q->m = nm;
		q->cap = cap;
	}
	struct fdt_pmsg *m = &q->m[q->n];
	m->buf = malloc(FDT_STEP_HEADER + len + 1);
	if (!m->buf)
		return -1;
	memcpy(m->buf + FDT_STEP_HEADER, payload, len);
	m->len = (uint32_t)len;
	m->t = c;
	m->seq = (uint16_t)s->seq[l]++;
	m->reason = FDT_PR_NONE;
	m->arr = r->arr++;
	q->n++;
	return 0;
}

int fdt_preg_on_levels(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	uint32_t t32;
	uint8_t p[FDT_L_MAX];
	if (fdt_levels_decode(payload, len, d->L, &t32, p)) {
		r->s.tab_bad++;
		return -1;
	}
	int64_t t = (int64_t)t32;
	if (t < r->next_v || now_us >= fdt_p_v_us(d, t)) {   /* after v(t): phase 3 already started without it */
		r->s.tab_late++;
		/* its arrival goes into the step log while the ring holds step t (w_2(t) of the calibration) */
		struct fdt_pstep *ls = fdt_preg_step(r, t);
		if (ls && !ls->have_p && ls->p_late_at < 0)
			ls->p_late_at = now_us;
		return 1;
	}
	if (t > fdt_p_step_at(d, now_us) + 2) {   /* a table for a step that the ring cannot hold yet */
		r->s.tab_bad++;
		return -1;
	}
	struct fdt_pstep *s = step_get(r, t);
	memcpy(s->p, p, (size_t)d->L);
	s->have_p = 1;
	s->p_at = now_us;
	return 0;
}

/* the ring may hold step t: not applied yet and at most two steps ahead */
static int ring_holds(const struct fdt_preg *r, int64_t t, int64_t now_us)
{
	return t >= r->next_v && t <= fdt_p_step_at(r->d, now_us) + 2;
}

void fdt_preg_on_out(struct fdt_preg *r, int k, uint32_t len)
{
	if (!r->ring || k < 0 || k >= FDT_DATA_MAX)
		return;
	uint32_t *ring = r->ring + (size_t)k * FDT_P_RING;
	ring[r->ring_head[k]] = len;
	r->ring_head[k] = (r->ring_head[k] + 1) % FDT_P_RING;
	if (r->ring_n[k] < FDT_P_RING)
		r->ring_n[k]++;   /* full: the newest packet replaces the oldest */
}

void fdt_preg_ring_reset(struct fdt_preg *r, int k)
{
	if (k >= 0 && k < FDT_DATA_MAX)
		r->ring_head[k] = r->ring_n[k] = 0;
}

int64_t fdt_preg_ring_count(const struct fdt_preg *r, int k, int64_t o)
{
	if (o <= 0 || !r->ring || k < 0 || k >= FDT_DATA_MAX)
		return 0;
	const uint32_t *ring = r->ring + (size_t)k * FDT_P_RING;
	int64_t sum = 0, n = 0;
	int pos = r->ring_head[k];
	while (sum < o && n < r->ring_n[k]) {
		pos = (pos + FDT_P_RING - 1) % FDT_P_RING;   /* newest first */
		sum += ring[pos];
		n++;
	}
	return n;
}

uint32_t fdt_preg_backlog(struct fdt_preg *r, int64_t t, int64_t now_us)
{
	int64_t o = -1, n = -1, bq = -1;
	if (r->io.sock) {
		struct fdt_sock sk;
		memset(&sk, 0, sizeof sk);
		r->io.sock(r->io.ctx, &sk);
		if (sk.n > 0) {
			o = n = 0;
			for (int j = 0; j < sk.n && j < FDT_DATA_MAX; j++) {
				/* o_i: the bytes of the connection that the channel has not carried yet, those in the socket
				 * (SIOCOUTQ: unsent or unacknowledged) and those that the broker still holds in its output
				 * queue for the client (the tail of a full socket) */
				int64_t oj = (sk.s[j].outq > 0 ? sk.s[j].outq : 0) + (sk.s[j].bq > 0 ? sk.s[j].bq : 0);
				if (sk.s[j].bq >= 0)
					bq = (bq < 0 ? 0 : bq) + sk.s[j].bq;
				o += oj;
				n += fdt_preg_ring_count(r, j, oj);
			}
		}
	}
	uint32_t Q = fdt_p_backlog(r->d, o, n);   /* 0 without a socket (o = n = -1) */
	if (ring_holds(r, t, now_us)) {
		struct fdt_pstep *s = step_get(r, t);
		s->have_Q = 1;
		s->Q = Q;
		s->bo = o;
		s->bn = n;
		s->bq = bq;
	}
	return Q;
}

int fdt_preg_stamp_demand(struct fdt_preg *r, void *payload, size_t len, int64_t now_us)
{
	uint32_t t32, Q, dv[FDT_L_MAX];
	if (fdt_demand_decode(payload, len, r->d->L, &t32, &Q, dv))
		return -1;
	fdt_put_u32((uint8_t *)payload + FDT_DEMAND_Q_OFF, fdt_preg_backlog(r, (int64_t)t32, now_us));
	return 0;
}

int fdt_preg_on_demand(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	uint32_t t32, Q, dv[FDT_L_MAX];
	(void)now_us;
	if (fdt_demand_decode(payload, len, d->L, &t32, &Q, dv)) {
		r->s.dem_bad++;
		return -1;
	}
	int64_t t = (int64_t)t32;
	if (!r->demand_ext) {
		/* the regulator publishes D_i(t) itself: a copy of its own table is fine; another table on the
		 * same topic would give the AS a value that the regulator does not police */
		struct fdt_pstep *s = fdt_preg_step(r, t);
		if (s && s->have_d && !memcmp(s->d, dv, sizeof(uint32_t) * (size_t)d->L))
			return 0;
		r->s.dem_foreign++;
		return 2;
	}
	if (!ring_holds(r, t, now_us)) {
		r->s.dem_late++;
		return 1;
	}
	struct fdt_pstep *s = step_get(r, t);
	memcpy(s->d, dv, sizeof(uint32_t) * (size_t)d->L);
	s->have_d = 1;
	if (!s->have_Q) {   /* a table that the plugin did not stamp: the Q_i that it carries */
		s->have_Q = 1;
		s->Q = Q;
	}
	r->s.dem_ext++;
	return 0;
}

static double get_f64(const uint8_t *p)
{
	uint64_t v = (uint64_t)fdt_get_u32(p) << 32 | fdt_get_u32(p + 4);
	double x;
	memcpy(&x, &v, sizeof x);
	return x;
}

static float get_f32(const uint8_t *p)
{
	uint32_t v = fdt_get_u32(p);
	float x;
	memcpy(&x, &v, sizeof x);
	return x;
}

int fdt_preg_on_feedback(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	const uint8_t *p = payload;
	(void)now_us;
	if (len < FDT_FEEDBACK_MIN || p[17] != d->L || len != FDT_FEEDBACK_MIN + (size_t)d->L) {
		r->s.fb_bad++;
		return -1;
	}
	int seen[FDT_L_MAX] = {0};
	for (int j = 0; j < d->L; j++) {
		int l = p[FDT_FEEDBACK_MIN + j];
		if (l < 1 || l > d->L || seen[l - 1]) {
			r->s.fb_bad++;
			return -1;
		}
		seen[l - 1] = 1;
	}
	int64_t t = (int64_t)fdt_get_u32(p);
	if (r->have_fb && t < r->fb_t) {
		r->s.fb_old++;
		return 1;
	}
	double y = get_f64(p + 4);
	double U = (double)get_f32(p + 12);
	r->have_fb = 1;
	r->fb_t = t;
	if (isfinite(y) && y > 0)
		r->fb_y = y;
	if (isfinite(U) && U > 0)
		r->fb_U = U;
	if (p[16] > 0)
		r->fb_m = p[16];
	for (int j = 0; j < d->L; j++)
		r->sigma[j] = (uint8_t)(p[FDT_FEEDBACK_MIN + j] - 1);
	r->s.fb_ok++;
	return 0;
}

/* ------------------------------------------------------------------ events */

/* phase 1 of step t: D_i(t) */
static void do_p1(struct fdt_preg *r, int64_t t, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	if (r->demand_ext)
		return;
	struct fdt_pstep *s = step_get(r, t);
	s->late_p1 = now_us - (fdt_p_start_us(d, t) + r->p1_off_us);
	for (int l = 0; l < d->L; l++)
		s->d[l] = (r->idle <= 0 || t - r->last_raw[l] <= r->idle) ? r->dmax[l] : 0;
	s->have_d = 1;
	uint32_t Q = fdt_preg_backlog(r, t, now_us);   /* Q_i(t) at the start of phase 1 */
	uint8_t buf[FDT_DEMAND_HDR + 4 * FDT_L_MAX];
	size_t n = fdt_demand_encode(buf, (uint32_t)t, Q, d->L, s->d);
	pub(r, r->demand_topic, 0, buf, n);
	r->s.dem_pub++;
}

struct fwd_ent {
	uint64_t arr;
	int l, k;
};

static int cmp_arr(const void *a, const void *b)
{
	const struct fwd_ent *x = a, *y = b;
	return x->arr < y->arr ? -1 : x->arr > y->arr;
}

/* publishes message k of topic l (QoS qos) and frees it; counts it in step s */
static void fwd_msg(struct fdt_preg *r, struct fdt_pstep *s, int l, int k, int qos)
{
	const struct fdt_pdtp *d = r->d;
	struct fdt_pmsg *m = &r->q[l].m[k];
	fdt_put_u32(m->buf, (uint32_t)m->t);
	fdt_put_u16(m->buf + 4, m->seq);
	pub(r, r->tx_topic[l], qos, m->buf, FDT_STEP_HEADER + (size_t)m->len);
	if (m->t == s->t) {
		s->fwd[l]++;
	} else {
		s->late_fwd[l]++;
		r->s.late_fwd++;
	}
	s->xfwd[l] += FDT_STEP_HEADER + (uint64_t)m->len;
	s->cost[l] += fdt_p_cost(d, l, m->len);
	r->s.fwd++;
	free(m->buf);
	m->buf = NULL;
}

/* paper_tdma: releases the messages of the step of the open slot while the unsent bytes of the data
 * bridge plus the bytes released in this tick stay below lw_bytes */
static void tdma_release(struct fdt_preg *r, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	struct fdt_pstep *s = r->gate_t >= 0 ? fdt_preg_step(r, r->gate_t) : NULL;
	if (!s || r->gate_left <= 0 || now_us < s->gate_s || now_us >= s->gate_e)
		return;
	int64_t unsent = 0;
	if (r->io.sock) {
		struct fdt_sock sk;
		memset(&sk, 0, sizeof sk);
		r->io.sock(r->io.ctx, &sk);
		for (int k = 0; k < sk.n && k < FDT_DATA_MAX; k++)
			unsent += sk.s[k].nsd > 0 ? sk.s[k].nsd : 0;
	}
	int64_t rel = 0;
	for (int j = 0; j < d->L && unsent + rel < r->lw_bytes; j++) {
		int l = r->sigma[j];
		struct fdt_pq *q = &r->q[l];
		int any = 0;
		for (int k = 0; k < q->n && unsent + rel < r->lw_bytes; k++) {
			struct fdt_pmsg *m = &q->m[k];
			if (!m->buf || m->t != r->gate_t || m->reason != FDT_PR_SLOT)
				continue;
			rel += FDT_STEP_HEADER + (int64_t)m->len;
			m->reason = FDT_PR_FWD;
			fwd_msg(r, s, l, k, 1);
			r->gate_left--;
			any = 1;
		}
		if (any)
			q_remove_marked(q);
	}
}

/* paper_b3, b8, paper_tdma at v(t): the messages of step t in sigma_t, each topic in order of reception */
static void do_v_prio(struct fdt_preg *r, int64_t t, struct fdt_pstep *s, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	double B = INFINITY, cum = 0;
	int stop = 0;
	s->fb_t = r->have_fb ? r->fb_t : -1;
	if (r->mode == FDT_PMODE_B8) {
		double U = r->fb_U > 0 ? r->fb_U : r->U;
		int m = r->fb_m > 0 ? r->fb_m : (d->M > 0 ? d->M : 1);
		B = U * r->fb_y * (d->Ts - d->T1 - d->T2) / d->Ts / m;
		s->budget = B;
		s->y_used = r->fb_y;
	}
	if (r->mode == FDT_PMODE_TDMA) {
		int64_t v = fdt_p_v_us(d, t), T3 = d->Ts_us - d->T1_us - d->T2_us;
		s->gate_s = v + T3 * r->slot_pos / r->slot_n;
		s->gate_e = v + T3 * (r->slot_pos + 1) / r->slot_n;
		r->gate_t = t;
		r->gate_left = 0;
	}
	for (int j = 0; j < d->L; j++) {
		int l = r->sigma[j];
		struct fdt_pq *q = &r->q[l];
		int any = 0;
		for (int k = 0; k < q->n; k++) {
			struct fdt_pmsg *m = &q->m[k];
			if (m->t > t)   /* arrived after v(t): step t + 1 */
				continue;
			if (r->mode == FDT_PMODE_TDMA) {
				m->reason = FDT_PR_SLOT;   /* released in the slot, else logged as slot */
				r->gate_left++;
				continue;
			}
			double c = fdt_p_cost(d, l, m->len);
			if (stop || cum + c > B + 1e-6) {
				stop = 1;   /* strict priority: nothing after it in sigma_t leaves */
				m->reason = FDT_PR_BUDGET;
				continue;
			}
			cum += c;
			m->reason = FDT_PR_FWD;
			fwd_msg(r, s, l, k, 1);
			any = 1;
		}
		if (any)
			q_remove_marked(q);
	}
	if (r->mode == FDT_PMODE_TDMA)
		tdma_release(r, now_us);
}

/* phase 3 of step t */
static void do_v(struct fdt_preg *r, int64_t t, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	struct fdt_pstep *s = step_get(r, t);
	s->applied = 1;
	s->late_v = now_us - fdt_p_v_us(d, t);
	if (s->late_v > r->s.late_max_us)
		r->s.late_max_us = s->late_v;
	/* unsent and unacknowledged bytes of the data bridge at the start of phase 3 (Prop. 2 needs empty
	 * send buffers) */
	if (r->io.sock) {
		struct fdt_sock sk;
		memset(&sk, 0, sizeof sk);
		r->io.sock(r->io.ctx, &sk);
		if (sk.n > 0) {
			s->outq = s->nsd = 0;
			for (int k = 0; k < sk.n && k < FDT_DATA_MAX; k++) {
				s->outq += sk.s[k].outq > 0 ? sk.s[k].outq : 0;
				s->nsd += sk.s[k].nsd > 0 ? sk.s[k].nsd : 0;
			}
		}
	}
	if (s->have_p)
		r->s.tab_ok++;
	else
		r->s.tab_none++;
	if (immediate(r))
		return;   /* forwarded on arrival: nothing is queued */
	if (prio_mode(r)) {
		do_v_prio(r, t, s, now_us);
		return;
	}
	int police = policing(r);
	int block = !s->have_p ? FDT_PR_NO_TABLE : (police && !s->have_d) ? FDT_PR_NO_DEMAND : FDT_PR_NONE;
	size_t nf = 0, capf = 0;
	struct fwd_ent *fe = NULL;
	for (int l = 0; l < d->L; l++) {
		struct fdt_pq *q = &r->q[l];
		int lev = s->have_p ? s->p[l] : 0;
		double cum = 0;
		int stop = 0;
		for (int k = 0; k < q->n; k++) {
			struct fdt_pmsg *m = &q->m[k];
			if (m->t > t)   /* arrived after v(t): step t + 1 */
				continue;
			if (block) {
				m->reason = (uint8_t)block;
				continue;
			}
			if (lev == 0) {
				m->reason = FDT_PR_LEVEL0;
				continue;
			}
			/* Policing (C17), the rule of the "Vessel regulator" Definition of the paper: the regulator
			 * "withholds the first message that would raise the sum above d_{i,l}(t) and every later message
			 * of the topic in step t". The sum is that of c_b (x + 6) + c_m,l over the messages of topic l of
			 * step t in arrival order; once one message does not fit, `stop` keeps every later message of l in
			 * step t (a smaller later message that would still fit is not forwarded either), so the forwarded
			 * messages of a topic are a prefix of its arrivals and the GS sees no gap. */
			double c = fdt_p_cost(d, l, m->len);
			if (stop || (police && cum + c > (double)s->d[l] + 1e-6)) {
				stop = 1;   /* no gap: the later messages of the topic stay too */
				m->reason = FDT_PR_POLICE;
				continue;
			}
			cum += c;
			m->reason = FDT_PR_FWD;
			if (nf >= capf) {
				size_t cap = capf ? 2 * capf : 256;
				struct fwd_ent *n = realloc(fe, cap * sizeof *n);
				if (!n) {   /* out of memory: the message stays and is logged at the end of the step */
					m->reason = FDT_PR_EOS;
					continue;
				}
				fe = n;
				capf = cap;
			}
			fe[nf].arr = m->arr;
			fe[nf].l = l;
			fe[nf].k = k;
			nf++;
		}
	}
	/* strict priority: level 2 before level 1, each level in order of reception; QoS 0 for both levels */
	if (nf > 1)
		qsort(fe, nf, sizeof *fe, cmp_arr);
	for (int level = 2; level >= 1; level--)
		for (size_t j = 0; j < nf; j++) {
			int l = fe[j].l;
			if (s->p[l] != level)
				continue;
			struct fdt_pmsg *m = &r->q[l].m[fe[j].k];
			fdt_put_u32(m->buf, (uint32_t)m->t);
			fdt_put_u16(m->buf + 4, m->seq);
			pub(r, r->tx_topic[l], fdt_preg_qos(level), m->buf, FDT_STEP_HEADER + (size_t)m->len);
			if (m->t == t) {
				s->fwd[l]++;
			} else {
				s->late_fwd[l]++;
				r->s.late_fwd++;
			}
			s->xfwd[l] += FDT_STEP_HEADER + (uint64_t)m->len;
			s->cost[l] += fdt_p_cost(d, l, m->len);
			r->s.fwd++;
			free(m->buf);
			m->buf = NULL;
		}
	free(fe);
	for (int l = 0; l < d->L; l++)
		q_remove_marked(&r->q[l]);
}

static void summary_json(struct fdt_preg *r, struct jbuf *j, const struct fdt_pstep *s)
{
	const struct fdt_pdtp *d = r->d;
	jp(j, "{\"t\":%" PRId64 ",\"v\":%d,\"mode\":\"%s\",\"have_d\":%s,\"have_p\":%s,\"late_us\":%" PRId64
	   ",\"late_p1_us\":%" PRId64 ",\"outq\":%" PRId64 ",\"nsd\":%" PRId64,
	   s->t, r->vessel, fdt_pmode_name(r->mode), s->have_d ? "true" : "false", s->have_p ? "true" : "false",
	   s->late_v, s->late_p1, s->outq, s->nsd);
	/* arrival of p(t) on the vessel clock, microseconds after start(t): before v(t) (p_rx_us) or after it,
	 * withheld (p_late_us); null without a table in the step */
	if (s->have_p)
		jp(j, ",\"p_rx_us\":%" PRId64, s->p_at - fdt_p_start_us(d, s->t));
	else
		jp(j, ",\"p_rx_us\":null");
	if (s->p_late_at >= 0)
		jp(j, ",\"p_late_us\":%" PRId64, s->p_late_at - fdt_p_start_us(d, s->t));
	else
		jp(j, ",\"p_late_us\":null");
	if (s->have_Q)
		jp(j, ",\"Q\":%u,\"o\":%" PRId64 ",\"bq\":%" PRId64 ",\"n\":%" PRId64, s->Q, s->bo, s->bq, s->bn);
	else
		jp(j, ",\"Q\":null");
	if (s->have_p)
		JARR(j, "p", d->L, "%u", (unsigned)s->p[l_]);
	else
		jp(j, ",\"p\":null");
	if (s->have_d)
		JARR(j, "d", d->L, "%u", s->d[l_]);
	else
		jp(j, ",\"d\":null");
	JARR(j, "rx", d->L, "%u", s->rx[l_]);
	JARR(j, "fwd", d->L, "%u", s->fwd[l_]);
	JARR(j, "held", d->L, "%u", s->held[l_]);
	JARR(j, "police", d->L, "%u", s->police[l_]);
	JARR(j, "drop", d->L, "%u", s->drop[l_]);
	JARR(j, "x", d->L, "%" PRIu64, s->xfwd[l_]);
	JARR(j, "cost", d->L, "%.0f", s->cost[l_]);
	JARR(j, "late_fwd", d->L, "%u", s->late_fwd[l_]);
	JARR(j, "carried", d->L, "%u", s->carried[l_]);
	if (prio_mode(r)) {
		jp(j, ",\"fb_t\":%" PRId64, s->fb_t);
		JARR(j, "sigma", d->L, "%u", (unsigned)r->sigma[l_] + 1);
	}
	if (r->mode == FDT_PMODE_B8)
		jp(j, ",\"budget\":%.0f,\"y\":%.0f", s->budget, s->y_used);
	if (r->mode == FDT_PMODE_TDMA)
		jp(j, ",\"slot\":[%" PRId64 ",%" PRId64 "]", s->gate_s, s->gate_e);
	jp(j, "}");
}

/* end of step t, at start(t + 1) */
static void do_end(struct fdt_preg *r, int64_t t)
{
	const struct fdt_pdtp *d = r->d;
	if (r->gate_t >= 0 && r->gate_t <= t) {
		r->gate_t = -1;
		r->gate_left = 0;
	}
	if (end_of_step_rule(r)) {
		q_log_upto(r, t, 0, t);   /* never forwarded later */
	} else {
		struct fdt_pstep *s = fdt_preg_step(r, t);
		for (int l = 0; l < d->L; l++)
			for (int k = 0; k < r->q[l].n; k++)
				if (r->q[l].m[k].t <= t && s)
					s->carried[l]++;
	}
	log_flush(r);
	struct fdt_pstep *s = fdt_preg_step(r, t);
	if (s) {
		struct jbuf j = {r->json, 0, r->json_cap};
		summary_json(r, &j, s);
		pub(r, r->log_topic, 0, j.p, j.n);
		if (r->step_log && (fwrite(j.p, 1, j.n, r->step_log) != j.n || fputc('\n', r->step_log) == EOF ||
		                    fflush(r->step_log)))
			r->s.log_err++;
		step_reset(s, -1);
	}
	r->s.steps++;
}

/* the clock moved by more than a few steps: drop the steps and start again at the current one */
static void resync(struct fdt_preg *r, int64_t now_us)
{
	int64_t cur = fdt_p_step_at(r->d, now_us);
	if (r->next_p1 >= cur - 2 && r->next_p1 <= cur + 4)
		return;
	q_log_upto(r, INT64_MAX, FDT_PR_RESYNC, cur);
	log_flush(r);
	r->gate_t = -1;
	r->gate_left = 0;
	for (int k = 0; k < FDT_P_STEPS; k++)
		step_reset(&r->st[k], -1);
	reset_clock(r, now_us);
	r->s.resync++;
}

int64_t fdt_preg_tick(struct fdt_preg *r, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	resync(r, now_us);
	for (;;) {
		/* the earliest event due; at the same instant: the end of t, then phase 1, then v. The end of
		 * a step comes only after its phase 3 (next_end < next_v). */
		int64_t at[3] = {r->next_end < r->next_v ? fdt_p_start_us(d, r->next_end + 1) : INT64_MAX,
		                 fdt_p_start_us(d, r->next_p1) + r->p1_off_us, fdt_p_v_us(d, r->next_v)};
		int ev = 0;
		for (int k = 1; k < 3; k++)
			if (at[k] < at[ev])
				ev = k;
		if (at[ev] > now_us) {
			if (r->mode == FDT_PMODE_TDMA && r->gate_t >= 0) {
				tdma_release(r, now_us);
				const struct fdt_pstep *s = fdt_preg_step(r, r->gate_t);
				if (s && r->gate_left > 0 && now_us < s->gate_e) {
					int64_t nx = now_us < s->gate_s ? s->gate_s : now_us + r->tick_us;
					if (nx > s->gate_e)
						nx = s->gate_e;
					if (nx < at[ev])
						return nx;
				}
			}
			return at[ev];
		}
		if (ev == 0) {
			do_end(r, r->next_end);
			r->next_end++;
		} else if (ev == 1) {
			do_p1(r, r->next_p1, now_us);
			r->next_p1++;
		} else {
			do_v(r, r->next_v, now_us);
			r->next_v++;
		}
	}
}
