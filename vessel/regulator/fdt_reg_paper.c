#define _GNU_SOURCE
#include "fdt_reg_paper.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ names */

static const char *const pmode_names[] = {"paper", "paper_nopolice", "paper_noeos"};
static const char *const reason_names[FDT_PR_N] = {"none", "forwarded", "no_table", "no_demand", "level0",
                                                   "police", "overflow", "resync", "eos", "shutdown"};

int fdt_pmode_parse(const char *s)
{
	for (int k = 0; k < 3; k++)
		if (!strcmp(s, pmode_names[k]))
			return k;
	return -1;
}

const char *fdt_pmode_name(int mode)
{
	return mode >= 0 && mode < 3 ? pmode_names[mode] : "?";
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
	s->outq = s->nsd = -1;
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
	snprintf(r->demand_topic, sizeof r->demand_topic, "boat/b%d/demand", vessel);
	snprintf(r->log_topic, sizeof r->log_topic, "boat/b%d/regulator", vessel);
	r->json_cap = 16384 + (size_t)d->L * 256;
	r->json = malloc(r->json_cap);
	if (!r->json)
		return -1;
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

int fdt_preg_on_demand(struct fdt_preg *r, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_pdtp *d = r->d;
	uint32_t t32, dv[FDT_L_MAX];
	(void)now_us;
	if (fdt_demand_decode(payload, len, d->L, &t32, dv)) {
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
	if (t < r->next_v || t > fdt_p_step_at(d, now_us) + 2) {
		r->s.dem_late++;
		return 1;
	}
	struct fdt_pstep *s = step_get(r, t);
	memcpy(s->d, dv, sizeof(uint32_t) * (size_t)d->L);
	s->have_d = 1;
	r->s.dem_ext++;
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
	uint8_t buf[5 + 4 * FDT_L_MAX];
	size_t n = fdt_demand_encode(buf, (uint32_t)t, d->L, s->d);
	pub(r, r->demand_topic, 1, buf, n);
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
	/* strict priority: level 2 (QoS 2) before level 1 (QoS 1), each level in order of reception */
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
			pub(r, r->tx_topic[l], level, m->buf, FDT_STEP_HEADER + (size_t)m->len);
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
	jp(j, "}");
}

/* end of step t, at start(t + 1) */
static void do_end(struct fdt_preg *r, int64_t t)
{
	const struct fdt_pdtp *d = r->d;
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
		if (at[ev] > now_us)
			return at[ev];
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
