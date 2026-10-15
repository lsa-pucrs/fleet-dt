#define _GNU_SOURCE
#include "fdt_reg.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cjson/cJSON.h>

/* ------------------------------------------------------------------ modes */

static const char *const mode_names[] = {"fleetdt", "b1", "b2", "b3", "b5", "slotted", "tdma"};
static const char *const class_names[] = {"be", "af41", "ef"};

int fdt_mode_parse(const char *s)
{
	for (int k = 0; k < (int)(sizeof mode_names / sizeof *mode_names); k++)
		if (!strcmp(s, mode_names[k]))
			return k;
	return -1;
}

const char *fdt_mode_name(int mode)
{
	return mode >= 0 && mode < (int)(sizeof mode_names / sizeof *mode_names) ? mode_names[mode] : "?";
}

const char *fdt_class_name(int cls)
{
	return cls >= 0 && cls < 3 ? class_names[cls] : "be";
}

int fdt_topic_class(const struct fdt_dtp2 *d, int l)
{
	if (d->topic[l].required)
		return FDT_CLASS_EF;
	return d->topic[l].kind == FDT2_STREAM ? FDT_CLASS_AF41 : FDT_CLASS_BE;
}

static int uses_table(int mode)
{
	return mode == FDT_MODE_FLEETDT || mode == FDT_MODE_B5 || mode == FDT_MODE_SLOTTED;
}

static int immediate(int mode)
{
	return mode == FDT_MODE_B1 || mode == FDT_MODE_B2;
}

static int policing(int mode)
{
	return mode == FDT_MODE_FLEETDT || mode == FDT_MODE_SLOTTED;
}

/* ------------------------------------------------------------------ memory */

static void out_free(struct fdt_out *o)
{
	if (o->m) {
		for (int k = 0; k < o->n; k++)
			free(o->m[k].buf);
		free(o->m);
	}
	memset(o, 0, sizeof *o);
}

static void step_clear(struct fdt_step *s)
{
	for (int l = 0; l < FDT_L_MAX; l++) {
		struct fdt_tstep *ts = &s->tp[l];
		for (int k = 0; k < ts->nm; k++)
			free(ts->m[k].buf);
		free(ts->m);
		for (int k = 0; k < ts->no; k++)
			out_free(&ts->o[k]);
	}
	free(s->rl);
	memset(s, 0, sizeof *s);
	s->t = -1;
	s->tb.t = -1;
}

static int slot_of(int64_t t)
{
	return (int)(((t % FDT_STEPS) + FDT_STEPS) % FDT_STEPS);
}

struct fdt_step *fdt_reg_step(struct fdt_reg *r, int64_t t)
{
	struct fdt_step *s = &r->st[slot_of(t)];
	return s->t == t ? s : NULL;
}

/* the slot of step t; a step that still holds the slot is stale and its messages are dropped */
static struct fdt_step *step_get(struct fdt_reg *r, int64_t t)
{
	struct fdt_step *s = &r->st[slot_of(t)];
	if (s->t == t)
		return s;
	if (s->t >= 0) {
		r->st_.stale += (uint64_t)(s->nrl - s->irl);
		step_clear(s);
	}
	s->t = t;
	s->tb.t = -1;
	s->late_p1 = s->late_v = -1;
	s->outq_p1 = -1;
	return s;
}

/* ------------------------------------------------------------------ init */

int fdt_reg_init(struct fdt_reg *r, const struct fdt_dtp2 *d, int vessel, int mode, int max_msgs,
                 int64_t now_us, const struct fdt_io *io)
{
	memset(r, 0, sizeof *r);
	r->d = d;
	r->vessel = vessel;
	r->mode = mode;
	r->max_msgs = max_msgs > 0 ? max_msgs : 4096;
	if (io)
		r->io = *io;
	for (int k = 0; k < FDT_STEPS; k++) {
		r->st[k].t = -1;
		r->st[k].tb.t = -1;
		r->tab[k].t = -1;
	}
	for (int l = 0; l < d->L; l++) {
		int n;
		if (mode == FDT_MODE_B2)
			n = snprintf(r->tx_topic[l], sizeof r->tx_topic[l], "boat/b%d/tx2/%s/%s", vessel,
			             fdt_class_name(fdt_topic_class(d, l)), d->topic[l].name);
		else
			n = snprintf(r->tx_topic[l], sizeof r->tx_topic[l], "boat/b%d/tx/%s", vessel, d->topic[l].name);
		r->tx_len[l] = (uint32_t)n;
	}
	snprintf(r->demand_topic, sizeof r->demand_topic, "boat/b%d/demand", vessel);
	snprintf(r->log_topic, sizeof r->log_topic, "boat/b%d/regulator", vessel);
	for (int k = 0; k < FDT_DATA_MAX; k++) {
		r->trk[k].e = calloc(FDT_TRK_MAX, sizeof(struct fdt_trk_ent));
		if (!r->trk[k].e)
			return -1;
	}
	r->json_cap = 65536 + (size_t)d->L * FDT_U_MAX * 48;
	r->json = malloc(r->json_cap);
	if (!r->json)
		return -1;
	/* the first phase 1 due is the next step start; the first v(t) due is the end of the
	 * collection window that holds now */
	r->next_p1 = fdt2_step_at(d, now_us) + 1;
	r->next_v = fdt2_collect_step(d, now_us);
	return 0;
}

void fdt_reg_set_p1_offset_guard(struct fdt_reg *r, int64_t spacing_us, int64_t guard_us)
{
	/* the vessel at position k (0-based) of the DTP list, i = k + 1, publishes its demand
	 * i * (T1 - g1) / (M + 1) after start(t) (spacing_us < 0), or i * spacing_us (spacing_us >= 0);
	 * g1 = eps + 2 T_tick (guard_us < 0, the rule of the paper) or guard_us (>= 0, an override) */
	const struct fdt_dtp2 *d = r->d;
	int M = d->M > 0 ? d->M : 1, pos = fdt2_vessel_pos(d, r->vessel);
	if (pos < 0)
		pos = 0;
	if (guard_us < 0)
		guard_us = d->eps_us + 2 * d->T_tick_us;
	if (guard_us >= d->T1_us)
		guard_us = 0;
	if (spacing_us < 0)
		r->p1_off_us = (int64_t)(pos + 1) * (d->T1_us - guard_us) / (M + 1);
	else
		r->p1_off_us = spacing_us * (pos + 1);
	if (r->p1_off_us >= d->T1_us)
		r->p1_off_us = d->T1_us / 2;
}

void fdt_reg_set_p1_offset(struct fdt_reg *r, int64_t spacing_us)
{
	fdt_reg_set_p1_offset_guard(r, spacing_us, -1);
}

void fdt_reg_free(struct fdt_reg *r)
{
	for (int k = 0; k < FDT_STEPS; k++)
		if (r->st[k].t >= 0)
			step_clear(&r->st[k]);
	for (int l = 0; l < FDT_L_MAX; l++)
		for (int k = 0; k < r->nopen[l]; k++)
			out_free(&r->open[l][k]);
	for (int k = 0; k < FDT_DATA_MAX; k++) {
		free(r->trk[k].e);
		r->trk[k].e = NULL;
	}
	free(r->json);
	r->json = NULL;
}

/* ------------------------------------------------------------------ publication */

static void pub(struct fdt_reg *r, const char *topic, int qos, const void *p, size_t n)
{
	if (r->io.publish && r->io.publish(r->io.ctx, topic, qos, p, n) != 0)
		r->st_.pub_err++;
}

static void read_sock(struct fdt_reg *r, struct fdt_sock *sk)
{
	memset(sk, 0, sizeof *sk);
	if (r->io.sock)
		r->io.sock(r->io.ctx, sk);
	if (sk->n > FDT_DATA_MAX)
		sk->n = FDT_DATA_MAX;
}

/* release one message: step header, publication, counters; returns the PUBLISH bytes */
static uint32_t release(struct fdt_reg *r, struct fdt_step *s, int l, int unit, struct fdt_msg *m)
{
	struct fdt_tstep *ts = &s->tp[l];
	uint16_t seq = (uint16_t)ts->seq++;
	fdt_put_u32(m->buf, (uint32_t)s->t);
	fdt_put_u16(m->buf + 4, seq);
	pub(r, r->tx_topic[l], r->d->topic[l].qos, m->buf, FDT2_STEP_HEADER + (size_t)m->len);
	uint32_t bytes = fdt2_publish_bytes(r->tx_len[l], r->d->topic[l].qos, FDT2_STEP_HEADER + (size_t)m->len);
	s->rel[l]++;
	s->xrel[l] += m->len;
	if (unit >= 0 && unit < FDT_U_MAX)
		s->relu[l][unit]++;
	r->pending += bytes;
	r->st_.released++;
	return bytes;
}

/* ------------------------------------------------------------------ raw messages */

static struct fdt_msg *msg_new(struct fdt_msg *m, const void *payload, size_t len)
{
	m->buf = malloc(FDT2_STEP_HEADER + len + 1);
	if (!m->buf)
		return NULL;
	memcpy(m->buf + FDT2_STEP_HEADER, payload, len);
	m->len = (uint32_t)len;
	return m;
}

int fdt_reg_on_raw(struct fdt_reg *r, int l, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	if (l < 0 || l >= d->L || len > 16u * 1024 * 1024)
		return -1;
	r->st_.raw++;
	int64_t c = fdt2_collect_step(d, now_us);
	if (c < r->next_v)   /* never into a step that is applied (clock stepped back) */
		c = r->next_v;
	const struct fdt2_topic *tp = &d->topic[l];

	if (immediate(r->mode)) {   /* b1, b2: forward at once */
		struct fdt_step *s = step_get(r, c);
		struct fdt_msg m;
		if (!msg_new(&m, payload, len))
			return -1;
		s->tp[l].rx++;
		s->tp[l].xrx += len;
		s->nadm[l] = 1;
		release(r, s, l, 0, &m);
		r->pending = 0;
		free(m.buf);
		return 0;
	}
	if (tp->kind == FDT2_STREAM) {
		struct fdt_step *s = step_get(r, c);
		struct fdt_tstep *ts = &s->tp[l];
		if (ts->nm >= r->max_msgs) {
			ts->over++;
			r->st_.over++;
			return 1;
		}
		if (ts->nm >= ts->cap) {
			int cap = ts->cap ? 2 * ts->cap : 16;
			struct fdt_msg *nm = realloc(ts->m, (size_t)cap * sizeof *nm);
			if (!nm)
				return -1;
			ts->m = nm;
			ts->cap = cap;
		}
		if (!msg_new(&ts->m[ts->nm], payload, len))
			return -1;
		ts->nm++;
		ts->x += len;
		ts->rx++;
		ts->xrx += len;
		return 0;
	}
	/* output topic: header out:u32, k:u16, n:u16 */
	const uint8_t *p = payload;
	if (len < FDT2_OUT_HEADER) {
		r->st_.bad_raw++;
		return -1;
	}
	uint32_t id = fdt_get_u32(p);
	uint16_t k = fdt_get_u16(p + 4), n = fdt_get_u16(p + 6);
	if (n == 0 || n > tp->n || k >= n) {
		r->st_.bad_raw++;
		return -1;
	}
	struct fdt_out *o = NULL;
	for (int j = 0; j < r->nopen[l]; j++)
		if (r->open[l][j].id == id) {
			o = &r->open[l][j];
			break;
		}
	if (o && o->n != n) {
		r->st_.bad_raw++;
		return -1;
	}
	if (!o) {
		if (r->nopen[l] >= FDT_OPEN_MAX) {   /* evict the oldest open output */
			int old = 0;
			for (int j = 1; j < r->nopen[l]; j++)
				if (r->open[l][j].first_step < r->open[l][old].first_step)
					old = j;
			step_get(r, c)->tp[l].inc++;
			r->st_.inc++;
			out_free(&r->open[l][old]);
			r->open[l][old] = r->open[l][--r->nopen[l]];
			memset(&r->open[l][r->nopen[l]], 0, sizeof(struct fdt_out));
		}
		o = &r->open[l][r->nopen[l]];
		memset(o, 0, sizeof *o);
		o->m = calloc(n, sizeof(struct fdt_msg));
		if (!o->m)
			return -1;
		o->id = id;
		o->n = n;
		o->first_step = c;
		r->nopen[l]++;
	}
	if (o->m[k].buf) {   /* repeated message */
		r->st_.bad_raw++;
		return 1;
	}
	if (!msg_new(&o->m[k], payload, len))
		return -1;
	o->got++;
	o->x += len;
	if (len > (size_t)tp->s)
		o->bad = 1;
	if (o->got == o->n) {   /* complete: joins the step of its last message */
		struct fdt_step *s = step_get(r, c);
		struct fdt_tstep *ts = &s->tp[l];
		int j = (int)(o - r->open[l]);
		if (ts->no >= FDT_U_MAX) {
			ts->over += o->n;
			r->st_.over += o->n;
			out_free(o);
		} else {
			ts->o[ts->no++] = *o;
			ts->rx += o->n;
			ts->xrx += o->x;
		}
		r->open[l][j] = r->open[l][--r->nopen[l]];
		memset(&r->open[l][r->nopen[l]], 0, sizeof(struct fdt_out));
	}
	return 0;
}

/* ------------------------------------------------------------------ table */

int fdt_reg_on_levels(struct fdt_reg *r, const void *payload, size_t len, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	struct fdt_table tb;
	memset(&tb, 0, sizeof tb);
	cJSON *o = cJSON_ParseWithLength(payload, len);
	int rc = -1;
	if (!o || !cJSON_IsObject(o))
		goto out;
	const cJSON *t = cJSON_GetObjectItemCaseSensitive(o, "t");
	const cJSON *p = cJSON_GetObjectItemCaseSensitive(o, "p");
	const cJSON *k = cJSON_GetObjectItemCaseSensitive(o, "k");
	const cJSON *ord = cJSON_GetObjectItemCaseSensitive(o, "o");
	const cJSON *slot = cJSON_GetObjectItemCaseSensitive(o, "slot");
	if (!cJSON_IsNumber(t) || t->valuedouble < 1 || t->valuedouble != (double)(int64_t)t->valuedouble ||
	    !cJSON_IsArray(p) || cJSON_GetArraySize(p) != d->L || !cJSON_IsArray(k) || cJSON_GetArraySize(k) != d->L)
		goto out;
	tb.t = (int64_t)t->valuedouble;
	for (int l = 0; l < d->L; l++) {
		const cJSON *pv = cJSON_GetArrayItem(p, l), *kv = cJSON_GetArrayItem(k, l);
		if (!cJSON_IsNumber(pv) || !cJSON_IsNumber(kv))
			goto out;
		double pl = pv->valuedouble, kl = kv->valuedouble;
		if ((pl != 0 && pl != 1 && pl != 2) || kl < 0 || kl > 65535 || kl != (double)(int)kl)
			goto out;
		tb.p[l] = (uint8_t)pl;
		tb.k[l] = (uint16_t)kl;
	}
	if (ord && !cJSON_IsNull(ord)) {
		int seen[FDT_L_MAX] = {0};
		if (!cJSON_IsArray(ord) || cJSON_GetArraySize(ord) != d->L)
			goto out;
		for (int j = 0; j < d->L; j++) {
			const cJSON *e = cJSON_GetArrayItem(ord, j);
			if (!cJSON_IsNumber(e) || e->valuedouble < 0 || e->valuedouble >= d->L ||
			    e->valuedouble != (double)(int)e->valuedouble || seen[(int)e->valuedouble])
				goto out;
			seen[(int)e->valuedouble] = 1;
			tb.o[j] = (uint8_t)e->valuedouble;
		}
	} else {
		for (int j = 0; j < d->L; j++)
			tb.o[j] = (uint8_t)d->order[j];
	}
	if (slot && !cJSON_IsNull(slot)) {
		const cJSON *a, *b;
		if (!cJSON_IsArray(slot) || cJSON_GetArraySize(slot) != 2 || !cJSON_IsNumber(a = cJSON_GetArrayItem(slot, 0)) ||
		    !cJSON_IsNumber(b = cJSON_GetArrayItem(slot, 1)))
			goto out;
		tb.has_slot = 1;
		tb.slot_s = (int64_t)a->valuedouble;
		tb.slot_e = (int64_t)b->valuedouble;
	}
	tb.at_us = now_us;
	if (tb.t < r->next_v || now_us >= fdt2_v_us(d, tb.t)) {   /* after v_i(t): ignored */
		r->st_.tab_late++;
		rc = 1;
		goto out_ok;
	}
	r->tab[slot_of(tb.t)] = tb;
	rc = 0;
	goto out_ok;
out:
	r->st_.tab_bad++;
out_ok:
	cJSON_Delete(o);
	return rc;
}

/* ------------------------------------------------------------------ offsets of the released packets */

void fdt_reg_sock_reset(struct fdt_reg *r, int k)
{
	if (k < 0 || k >= FDT_DATA_MAX)
		return;
	struct fdt_trk *tk = &r->trk[k];
	tk->model_end = 0;
	tk->head = tk->n = 0;
	r->pending = 0;
}

void fdt_reg_on_out(struct fdt_reg *r, int k, int qos, uint32_t pkt_bytes, uint32_t payload_len,
                    uint64_t written_end, int has_written)
{
	if (k < 0 || k >= FDT_DATA_MAX)
		return;
	struct fdt_trk *tk = &r->trk[k];
	uint64_t start = tk->model_end;
	if (has_written && written_end > start)
		start = written_end;   /* bytes written outside the tracked packets (PUBREL, PINGRESP, ...) */
	tk->model_end = start + pkt_bytes;
	if (tk->n == FDT_TRK_MAX) {
		tk->head = (tk->head + 1) % FDT_TRK_MAX;
		tk->n--;
		tk->lost++;
	}
	struct fdt_trk_ent *e = &tk->e[(tk->head + tk->n) % FDT_TRK_MAX];
	e->end = tk->model_end;
	e->size = pkt_bytes;
	e->content = payload_len > FDT2_STEP_HEADER ? payload_len - FDT2_STEP_HEADER : 0;
	e->qos = (uint8_t)qos;
	tk->n++;
	r->pending -= pkt_bytes;
	if (r->pending < 0)
		r->pending = 0;
	r->st_.outs++;
}

static void count_tail(struct fdt_b *b, const struct fdt_trk_ent *e, uint64_t tail)
{
	if (tail > e->size)
		tail = e->size;
	uint64_t c = tail < e->content ? tail : e->content;   /* the content is the end of the packet */
	if (e->qos > 0) {
		b->x2 += c;
		b->n2++;
	} else {
		b->x0 += c;
		b->n0++;
	}
}

void fdt_reg_held(struct fdt_reg *r, const struct fdt_sock *sk, struct fdt_b *b)
{
	memset(b, 0, sizeof *b);
	for (int k = 0; k < sk->n && k < FDT_DATA_MAX; k++) {
		struct fdt_trk *tk = &r->trk[k];
		uint64_t outq = sk->s[k].outq > 0 ? (uint64_t)sk->s[k].outq : 0;
		if (sk->s[k].has_acked) {
			uint64_t acked = sk->s[k].acked, tracked = 0;
			while (tk->n && tk->e[tk->head].end <= acked) {   /* acknowledged packets */
				tk->head = (tk->head + 1) % FDT_TRK_MAX;
				tk->n--;
			}
			for (int j = 0; j < tk->n; j++) {
				const struct fdt_trk_ent *e = &tk->e[(tk->head + j) % FDT_TRK_MAX];
				uint64_t tail = e->end - acked;
				count_tail(b, e, tail);
				tracked += tail > e->size ? e->size : tail;
			}
			b->other += outq > tracked ? outq - tracked : 0;
		} else {
			/* SIOCOUTQ holds the last outq bytes of the stream: walk back from the newest packet */
			uint64_t rem = outq;
			int j = tk->n - 1;
			for (; j >= 0 && rem > 0; j--) {
				const struct fdt_trk_ent *e = &tk->e[(tk->head + j) % FDT_TRK_MAX];
				uint64_t tail = rem < e->size ? rem : e->size;
				count_tail(b, e, tail);
				rem -= tail;
			}
			b->other += rem;
			/* the packets before the walk are acknowledged */
			int drop = j + 1;
			tk->head = (tk->head + drop) % FDT_TRK_MAX;
			tk->n -= drop;
		}
	}
}

/* ------------------------------------------------------------------ JSON */

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
		j->n = j->cap;   /* truncated */
}

#define JARR(j, name, L, fmt, expr) do { \
		jp(j, ",\"%s\":[", name); \
		for (int l_ = 0; l_ < (L); l_++) \
			jp(j, "%s" fmt, l_ ? "," : "", expr); \
		jp(j, "]"); \
	} while (0)

/* ------------------------------------------------------------------ phase 1 */

static void units_of_phase1(struct fdt_reg *r, struct fdt_step *s)
{
	const struct fdt_dtp2 *d = r->d;
	int64_t rest = d->T1_us + d->T2_us - r->p1_off_us;   /* the rest of C_i(t) after the demand */
	for (int l = 0; l < d->L; l++) {
		const struct fdt2_topic *tp = &d->topic[l];
		struct fdt_tstep *ts = &s->tp[l];
		uint32_t a = fdt2_arrival_outputs(d, l, rest);
		if (tp->kind == FDT2_STREAM) {
			s->u[l][0].x = ts->x + (uint64_t)a * (uint64_t)tp->n * (uint64_t)tp->s;
			s->u[l][0].n = (uint32_t)ts->nm + a * (uint32_t)tp->n;
			s->nu[l] = 1;
			s->held[l] = a == 0;
		} else {
			int nu = 0;
			for (int j = 0; j < ts->no && nu < FDT_U_MAX; j++, nu++) {
				s->u[l][nu].x = ts->o[j].x;
				s->u[l][nu].n = ts->o[j].got;
			}
			s->held[l] = nu;
			for (uint32_t j = 0; j < a && nu < FDT_U_MAX; j++, nu++) {
				s->u[l][nu].x = (uint64_t)tp->n * (uint64_t)tp->s;
				s->u[l][nu].n = (uint32_t)tp->n;
			}
			s->nu[l] = nu;
		}
	}
	s->have_dem = 1;
}

static void demand_json(struct fdt_reg *r, struct jbuf *j, const struct fdt_step *s, const struct fdt_step *prev,
                        int64_t t, const struct fdt_b *b)
{
	const struct fdt_dtp2 *d = r->d;
	jp(j, "{\"t\":%" PRId64 ",\"v\":%d,\"d\":[", t, r->vessel);
	for (int l = 0; l < d->L; l++) {
		jp(j, "%s[", l ? "," : "");
		for (int u = 0; u < s->nu[l]; u++)
			jp(j, "%s[%" PRIu64 ",%u]", u ? "," : "", s->u[l][u].x, s->u[l][u].n);
		jp(j, "]");
	}
	jp(j, "]");
	JARR(j, "held", d->L, "%d", s->held[l_]);
	jp(j, ",\"B\":{\"x0\":%" PRIu64 ",\"n0\":%" PRIu64 ",\"x2\":%" PRIu64 ",\"n2\":%" PRIu64 "}", b->x0, b->n0,
	   b->x2, b->n2);
	jp(j, ",\"rep\":{\"t\":%" PRId64 ",\"rel\":[", t - 1);
	uint64_t disc = 0;
	for (int l = 0; l < d->L; l++) {
		jp(j, "%s[", l ? "," : "");
		int na = prev ? prev->nadm[l] : 0;
		if (na > FDT_U_MAX)
			na = FDT_U_MAX;
		for (int u = 0; u < na; u++)
			jp(j, "%s%u", u ? "," : "", prev->relu[l][u]);
		jp(j, "]");
		if (prev)
			disc += prev->disc[l];
	}
	jp(j, "],\"disc\":%" PRIu64 ",\"tab\":%s}", disc, prev && prev->have_tab ? "true" : "false");
	jp(j, ",\"late_us\":%" PRId64 "}", s->late_p1);
}

static void log_json(struct fdt_reg *r, struct jbuf *j, const struct fdt_step *s)
{
	const struct fdt_dtp2 *d = r->d;
	jp(j, "{\"t\":%" PRId64 ",\"have_tab\":%s,\"late_us\":%" PRId64 ",\"nsd_max\":%" PRId64
	   ",\"outq_p1\":%" PRId64, s->t, s->have_tab ? "true" : "false", s->late_v, s->nsd_max, s->outq_p1);
	JARR(j, "rx", d->L, "%u", s->tp[l_].rx);
	JARR(j, "xrx", d->L, "%" PRIu64, s->tp[l_].xrx);
	JARR(j, "rel", d->L, "%u", s->rel[l_]);
	JARR(j, "disc", d->L, "%u", s->disc[l_]);
	JARR(j, "xrel", d->L, "%" PRIu64, s->xrel[l_]);
	jp(j, ",\"tick_us_max\":%" PRId64, s->tick_us_max);
	/* diagnostics outside SPEC_v2.md 4.2 */
	jp(j, ",\"mode\":\"%s\",\"late_p1_us\":%" PRId64, fdt_mode_name(r->mode), s->late_p1);
	JARR(j, "wh", d->L, "%u", s->wh[l_]);
	JARR(j, "inc", d->L, "%u", s->tp[l_].inc);
	JARR(j, "over", d->L, "%u", s->tp[l_].over);
	jp(j, "}");
}

/* end of step s: the unreleased messages of admitted units are discarded (b5: kept) */
static void close_step(struct fdt_reg *r, struct fdt_step *s)
{
	if (s->closed)
		return;
	s->closed = 1;
	if (r->mode == FDT_MODE_B5)
		return;
	for (int i = s->irl; i < s->nrl; i++) {
		s->disc[s->rl[i].l]++;
		r->st_.discarded++;
	}
	s->irl = s->nrl;
}

static void do_p1(struct fdt_reg *r, int64_t t, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	struct fdt_step *prev = fdt_reg_step(r, t - 1);
	if (prev)
		close_step(r, prev);
	struct fdt_step *s = step_get(r, t);
	s->late_p1 = now_us - (fdt2_start_us(d, t) + r->p1_off_us);
	if (s->late_p1 > r->st_.late_max_us)
		r->st_.late_max_us = s->late_p1;
	struct fdt_sock sk;
	struct fdt_b b;
	read_sock(r, &sk);
	fdt_reg_held(r, &sk, &b);
	if (sk.n) {
		s->outq_p1 = 0;
		for (int k = 0; k < sk.n; k++)
			s->outq_p1 += sk.s[k].outq > 0 ? sk.s[k].outq : 0;
	}
	units_of_phase1(r, s);
	struct jbuf j = {r->json, 0, r->json_cap};
	demand_json(r, &j, s, prev, t, &b);
	pub(r, r->demand_topic, 1, j.p, j.n);
	if (prev) {
		j.n = 0;
		log_json(r, &j, prev);
		pub(r, r->log_topic, 0, j.p, j.n);
		r->st_.steps++;
		if (!(r->mode == FDT_MODE_B5 && prev->irl < prev->nrl))
			step_clear(prev);
	}
	/* b5 carries unreleased messages; steps older than t-1 are stale */
	for (int k = 0; k < FDT_STEPS; k++) {
		struct fdt_step *o = &r->st[k];
		if (o->t >= 0 && o->t < t - 2) {
			r->st_.stale += (uint64_t)(o->nrl - o->irl);
			step_clear(o);
		}
	}
	r->pending = 0;
}

/* ------------------------------------------------------------------ phase 3 */

static int rl_add(struct fdt_step *s, int l, int unit, struct fdt_msg *m, uint32_t bytes)
{
	if (s->nrl >= s->caprl) {
		int cap = s->caprl ? 2 * s->caprl : 64;
		struct fdt_rel *n = realloc(s->rl, (size_t)cap * sizeof *n);
		if (!n)
			return -1;
		s->rl = n;
		s->caprl = cap;
	}
	s->rl[s->nrl].l = (uint8_t)l;
	s->rl[s->nrl].unit = (uint8_t)unit;
	s->rl[s->nrl].bytes = bytes;
	s->rl[s->nrl].m = m;
	s->nrl++;
	return 0;
}

static void default_table(const struct fdt_dtp2 *d, struct fdt_table *tb, int64_t t)
{
	memset(tb, 0, sizeof *tb);
	tb->t = t;
	for (int l = 0; l < d->L; l++) {
		tb->p[l] = d->topic[l].required ? 2 : 0;
		tb->k[l] = d->topic[l].required ? (uint16_t)fdt2_arrival_outputs(d, l, d->Ts_us) : 0;
	}
	for (int j = 0; j < d->L; j++)
		tb->o[j] = (uint8_t)d->order[j];
}

/* arrival-curve bound over Ts as the demand of one unit of topic l */
static struct fdt_unit unit_bound(const struct fdt_dtp2 *d, int l, int whole_step)
{
	const struct fdt2_topic *tp = &d->topic[l];
	struct fdt_unit u;
	uint32_t a = whole_step ? fdt2_arrival_outputs(d, l, d->Ts_us) : 1;
	u.x = (uint64_t)a * (uint64_t)tp->n * (uint64_t)tp->s;
	u.n = a * (uint32_t)tp->n;
	return u;
}

static void add_topic(struct fdt_reg *r, struct fdt_step *s, int l, int K, int police, int dflt)
{
	const struct fdt_dtp2 *d = r->d;
	const struct fdt2_topic *tp = &d->topic[l];
	struct fdt_tstep *ts = &s->tp[l];
	if (tp->kind == FDT2_STREAM) {
		struct fdt_unit u = (dflt || !s->have_dem) ? unit_bound(d, l, 1) : s->u[l][0];
		uint64_t cx = 0;
		uint32_t cn = 0;
		int i = 0;
		s->nadm[l] = 1;
		for (; i < ts->nm; i++) {
			struct fdt_msg *m = &ts->m[i];
			if (police && (cx + m->len > u.x || cn + 1 > u.n))
				break;
			cx += m->len;
			cn++;
			rl_add(s, l, 0, m, fdt2_publish_bytes(r->tx_len[l], tp->qos, FDT2_STEP_HEADER + (size_t)m->len));
		}
		s->wh[l] += (uint32_t)(ts->nm - i);
		return;
	}
	if (K > FDT_U_MAX)
		K = FDT_U_MAX;
	s->nadm[l] = K;
	for (int j = 0; j < ts->no; j++) {
		struct fdt_out *o = &ts->o[j];
		if (j >= K) {
			s->wh[l] += o->got;
			continue;
		}
		struct fdt_unit u = (dflt || !s->have_dem || j >= s->nu[l]) ? unit_bound(d, l, 0) : s->u[l][j];
		if (police && (o->x > u.x || o->got > u.n)) {   /* whole outputs only */
			s->disc[l] += o->got;
			r->st_.discarded += o->got;
			continue;
		}
		for (int k = 0; k < o->n; k++)
			rl_add(s, l, j, &o->m[k],
			       fdt2_publish_bytes(r->tx_len[l], tp->qos, FDT2_STEP_HEADER + (size_t)o->m[k].len));
	}
}

static uint32_t topic_msgs(const struct fdt_tstep *ts)
{
	uint32_t n = (uint32_t)ts->nm;
	for (int j = 0; j < ts->no; j++)
		n += ts->o[j].got;
	return n;
}

static void do_v(struct fdt_reg *r, int64_t t, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	struct fdt_step *s = step_get(r, t);
	s->late_v = now_us - fdt2_v_us(d, t);
	if (s->late_v > r->st_.late_max_us)
		r->st_.late_max_us = s->late_v;
	/* open outputs whose first message arrived before v(t-1) are dropped */
	for (int l = 0; l < d->L; l++)
		for (int j = 0; j < r->nopen[l];) {
			if (r->open[l][j].first_step < t) {
				s->tp[l].inc++;
				r->st_.inc++;
				out_free(&r->open[l][j]);
				r->open[l][j] = r->open[l][--r->nopen[l]];
				memset(&r->open[l][r->nopen[l]], 0, sizeof(struct fdt_out));
			} else {
				j++;
			}
		}
	s->applied = 1;
	s->last_tick = 0;
	if (immediate(r->mode))
		return;
	int admitted[FDT_L_MAX] = {0};
	if (uses_table(r->mode)) {
		struct fdt_table *tb = &r->tab[slot_of(t)];
		int dflt = tb->t != t;
		if (dflt) {
			default_table(d, &s->tb, t);
			r->st_.tab_default++;
		} else {
			s->tb = *tb;
			s->have_tab = 1;
			r->st_.tab_ok++;
		}
		int police = policing(r->mode);
		for (int level = 2; level >= 1; level--)
			for (int j = 0; j < d->L; j++) {
				int l = s->tb.o[j];
				if (s->tb.p[l] != level)
					continue;
				admitted[l] = 1;
				add_topic(r, s, l, s->tb.k[l], police, dflt);
			}
	} else {   /* b3, tdma: every unit, strict priority by the order "o" of the table of t when it
		    * arrived before v(t) (b3: sigma_t), else by the DTP order */
		struct fdt_table *tb = &r->tab[slot_of(t)];
		const uint8_t *ord = NULL;
		uint8_t dord[FDT_L_MAX];
		if (tb->t == t) {
			ord = tb->o;
		} else {
			for (int j = 0; j < d->L; j++)
				dord[j] = (uint8_t)d->order[j];
			ord = dord;
		}
		for (int j = 0; j < d->L; j++) {
			int l = ord[j];
			admitted[l] = 1;
			add_topic(r, s, l, s->tp[l].no, 0, 0);
		}
		if (r->mode == FDT_MODE_B3 && tb->t == t) {
			s->tb = *tb;
			s->have_tab = 1;
			r->st_.tab_ok++;
		}
		if (r->mode == FDT_MODE_TDMA) {
			/* the slot of the table of t when it arrived before v(t) (the AS of mode tdma sends one),
			 * else the own slot of M equal slots of W(t) in the order of "vessels" */
			struct fdt_table *tb = &r->tab[slot_of(t)];
			int M = d->M > 0 ? d->M : 1, pos = fdt2_vessel_pos(d, r->vessel);
			int64_t w0 = fdt2_w_start_us(d, t), tw = fdt2_w_end_us(d, t) - w0;
			if (pos < 0)
				pos = 0;
			s->gate_s = w0 + tw * pos / M;
			s->gate_e = w0 + tw * (pos + 1) / M;
			if (tb->t == t) {
				s->tb = *tb;
				s->have_tab = 1;
				r->st_.tab_ok++;
				if (tb->has_slot) {
					s->gate_s = tb->slot_s;
					s->gate_e = tb->slot_e;
				}
			}
		}
	}
	for (int l = 0; l < d->L; l++)
		if (!admitted[l])
			s->wh[l] += topic_msgs(&s->tp[l]);
	for (int l = 0; l < d->L; l++)
		r->st_.withheld += s->wh[l];
}

/* 0: the entry may leave now; else the instant at which its gate opens (INT64_MAX: not in this step) */
static int64_t gate(struct fdt_reg *r, const struct fdt_step *s, const struct fdt_rel *e, int64_t now_us)
{
	if (r->mode == FDT_MODE_SLOTTED && !r->d->topic[e->l].required) {
		if (!s->tb.has_slot || now_us >= s->tb.slot_e)
			return INT64_MAX;
		return now_us >= s->tb.slot_s ? 0 : s->tb.slot_s;
	}
	if (r->mode == FDT_MODE_TDMA) {
		if (now_us >= s->gate_e)
			return INT64_MAX;
		return now_us >= s->gate_s ? 0 : s->gate_s;
	}
	return 0;
}

static int64_t pace(struct fdt_reg *r, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	int64_t nxt = fdt2_start_us(d, r->next_p1) + r->p1_off_us, tv = fdt2_v_us(d, r->next_v);
	if (tv < nxt)
		nxt = tv;
	if (immediate(r->mode))
		return nxt;
	/* a step ends at start(t + 1) for every vessel: the phase 1 offset of the demand (up to
	 * M T1 / (M + 1)) does not extend the release of step t */
	for (int k = 0; k < FDT_STEPS; k++)
		if (r->st[k].t >= 0 && !r->st[k].closed && now_us >= fdt2_start_us(d, r->st[k].t + 1))
			close_step(r, &r->st[k]);
	/* releasing steps in increasing t: the applied step of phase 3 (and, in b5, older steps) */
	struct fdt_step *order[FDT_STEPS];
	int ns = 0;
	for (int k = 0; k < FDT_STEPS; k++) {
		struct fdt_step *s = &r->st[k];
		if (s->t < 0 || !s->applied || s->irl >= s->nrl || (s->closed && r->mode != FDT_MODE_B5))
			continue;
		int at = ns++;
		while (at > 0 && order[at - 1]->t > s->t) {
			order[at] = order[at - 1];
			at--;
		}
		order[at] = s;
	}
	if (!ns)
		return nxt;
	struct fdt_sock sk;
	read_sock(r, &sk);
	int64_t unsent = 0, nsd = 0;
	for (int k = 0; k < sk.n; k++) {
		nsd += sk.s[k].nsd > 0 ? sk.s[k].nsd : 0;
		if (sk.s[k].has_acked) {   /* packets that the broker holds and has not written yet */
			uint64_t wr = sk.s[k].acked + (uint64_t)(sk.s[k].outq > 0 ? sk.s[k].outq : 0);
			if (r->trk[k].model_end > wr)
				unsent += (int64_t)(r->trk[k].model_end - wr);
		}
	}
	unsent += nsd;
	if (sk.n == 0)
		r->pending = 0;
	unsent += r->pending;   /* released in earlier ticks, not yet written */
	int64_t rel_tick = 0, wait = INT64_MAX;
	int blocked = 0;
	for (int i = 0; i < ns && !blocked; i++) {
		struct fdt_step *s = order[i];
		if (s->last_tick) {
			int64_t gap = now_us - s->last_tick;
			if (gap > s->tick_us_max)
				s->tick_us_max = gap;
			if (gap > r->st_.tick_us_max)
				r->st_.tick_us_max = gap;
		}
		s->last_tick = now_us;
		if (nsd > s->nsd_max)
			s->nsd_max = nsd;
		while (s->irl < s->nrl) {
			struct fdt_rel *e = &s->rl[s->irl];
			int64_t g = gate(r, s, e, now_us);
			if (g) {
				if (g != INT64_MAX && g < wait)
					wait = g;
				blocked = 1;   /* strict order: nothing overtakes a gated message */
				break;
			}
			if (unsent + rel_tick >= d->L_w_bytes) {
				wait = now_us + d->T_tick_us;
				blocked = 1;
				break;
			}
			rel_tick += release(r, s, e->l, e->unit, e->m);
			s->irl++;
		}
	}
	if (!blocked) {
		/* the steps released everything: nothing to do until the next event */
	} else if (wait < nxt) {
		nxt = wait;
	}
	return nxt;
}

/* the clock moved by more than a few steps (a step of the clock, a stopped broker): the regulator drops
 * its steps and starts again at the current step, instead of replaying every step in between */
static void resync(struct fdt_reg *r, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	int64_t cur = fdt2_step_at(d, now_us);
	if (r->next_p1 >= cur - 2 && r->next_p1 <= cur + 4)
		return;
	for (int k = 0; k < FDT_STEPS; k++)
		if (r->st[k].t >= 0) {
			r->st_.stale += (uint64_t)(r->st[k].nrl - r->st[k].irl);
			step_clear(&r->st[k]);
		}
	for (int l = 0; l < FDT_L_MAX; l++) {
		for (int k = 0; k < r->nopen[l]; k++)
			out_free(&r->open[l][k]);
		r->nopen[l] = 0;
	}
	r->next_p1 = cur + 1;
	r->next_v = fdt2_collect_step(d, now_us);
	r->pending = 0;
	r->st_.resync++;
}

int64_t fdt_reg_tick(struct fdt_reg *r, int64_t now_us)
{
	const struct fdt_dtp2 *d = r->d;
	resync(r, now_us);
	for (;;) {
		int64_t tp1 = fdt2_start_us(d, r->next_p1) + r->p1_off_us, tv = fdt2_v_us(d, r->next_v);
		if (tp1 <= tv && tp1 <= now_us) {
			do_p1(r, r->next_p1, now_us);
			r->next_p1++;
		} else if (tv < tp1 && tv <= now_us) {
			do_v(r, r->next_v, now_us);
			r->next_v++;
		} else {
			break;
		}
	}
	return pace(r, now_us);
}
