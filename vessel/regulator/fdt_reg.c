#include "fdt_reg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void queue_clear(struct fdt_reg_queue *q)
{
	for (int k = 0; k < q->n; k++)
		free(q->m[k].buf);
	q->n = 0;
}

static void slot_reset(struct fdt_reg *r, int s, int64_t t)
{
	for (int l = 0; l < r->L; l++) {
		queue_clear(&r->q[s][l]);
		r->drop[s][l] = 0;
		r->rx[s][l] = 0;
	}
	r->qt[s] = t;
}

int fdt_reg_init(struct fdt_reg *r, const struct fdt_dtp *dtp, int max_msgs, int64_t now_us,
                 fdt_reg_publish_fn publish, void *ctx)
{
	memset(r, 0, sizeof *r);
	r->dtp = dtp;
	r->L = dtp->L;
	r->max_msgs = max_msgs > 0 ? max_msgs : 1024;
	r->publish = publish;
	r->ctx = ctx;
	r->log_offset_us = 1000;
	r->st.late_max_us = 0;
	for (int s = 0; s < 2; s++) {
		r->qt[s] = -1;
		for (int l = 0; l < r->L; l++) {
			r->q[s][l].m = calloc((size_t)r->max_msgs, sizeof(struct fdt_reg_msg));
			if (!r->q[s][l].m)
				return -1;
			r->q[s][l].cap = r->max_msgs;
		}
	}
	for (int k = 0; k < 4; k++)
		r->dem[k].t = r->lev[k].t = -1;
	/* the first phase 3 due is the one of the step that owns a message arriving now */
	r->next = fdt_raw_step(dtp, now_us);
	return 0;
}

void fdt_reg_free(struct fdt_reg *r)
{
	for (int s = 0; s < 2; s++)
		for (int l = 0; l < r->L; l++) {
			queue_clear(&r->q[s][l]);
			free(r->q[s][l].m);
			r->q[s][l].m = NULL;
		}
}

int fdt_reg_on_raw(struct fdt_reg *r, int l, const void *payload, size_t len, int64_t now_us)
{
	if (l < 0 || l >= r->L || len > 0xFFFFFFF0u)
		return -1;
	int64_t t = fdt_raw_step(r->dtp, now_us);
	if (t < r->next) {
		/* the phase-3 tick of step t is late and already ran: the message joins step next */
		t = r->next;
	}
	int s = (int)(t & 1);
	if (r->qt[s] != t)
		slot_reset(r, s, t);
	r->st.raw++;
	r->rx[s][l]++;
	struct fdt_reg_queue *q = &r->q[s][l];
	if (q->n >= q->cap) {
		r->drop[s][l]++;
		r->st.dropped++;
		return 1;
	}
	uint8_t *b = malloc(FDT_STEP_HEADER + len);
	if (!b) {
		r->drop[s][l]++;
		r->st.dropped++;
		return -1;
	}
	memcpy(b + FDT_STEP_HEADER, payload, len);
	q->m[q->n].buf = b;
	q->m[q->n].len = (uint32_t)len;
	q->n++;
	return 0;
}

int fdt_reg_on_demand(struct fdt_reg *r, const void *payload, size_t len)
{
	uint32_t t, d[FDT_L_MAX];
	if (fdt_demand_decode(payload, len, r->L, &t, d)) {
		r->st.bad_tables++;
		return -1;
	}
	int k = (int)(t & 3);
	r->dem[k].t = t;
	memcpy(r->dem[k].d, d, sizeof(uint32_t) * (size_t)r->L);
	return 0;
}

int fdt_reg_on_levels(struct fdt_reg *r, const void *payload, size_t len)
{
	uint32_t t;
	uint8_t p[FDT_L_MAX];
	if (fdt_levels_decode(payload, len, r->L, &t, p)) {
		r->st.bad_tables++;
		return -1;
	}
	int k = (int)(t & 3);
	r->lev[k].t = t;
	memcpy(r->lev[k].p, p, (size_t)r->L);
	return 0;
}

/* phase 3 of step t */
static void forward(struct fdt_reg *r, int64_t t, int64_t now_us, int64_t outq, struct fdt_reg_log *lg)
{
	const struct fdt_dtp *dtp = r->dtp;
	int s = (int)(t & 1);
	int have_q = r->qt[s] == t;
	const uint32_t *d = r->dem[t & 3].t == t ? r->dem[t & 3].d : NULL;
	const uint8_t *p = r->lev[t & 3].t == t ? r->lev[t & 3].p : NULL;

	memset(lg, 0, sizeof *lg);
	lg->t = (uint32_t)t;
	lg->have_d = d != NULL;
	lg->have_p = p != NULL;
	lg->late_us = now_us - (fdt_start_us(dtp, t) + dtp->T1_us + dtp->T2_us);
	lg->outq = outq;
	if (lg->late_us > r->st.late_max_us)
		r->st.late_max_us = lg->late_us;
	if (d)
		memcpy(lg->d, d, sizeof(uint32_t) * (size_t)r->L);
	if (p)
		memcpy(lg->p, p, (size_t)r->L);
	r->st.steps++;
	if (!d)
		r->st.no_d++;
	if (!p)
		r->st.no_p++;
	if (have_q)
		for (int l = 0; l < r->L; l++) {
			lg->rx[l] = r->rx[s][l];
			lg->drop[l] = r->drop[s][l];
		}
	for (int level = 2; level >= 1 && have_q && d && p; level--) {
		for (int k = 0; k < dtp->n_order; k++) {
			int l = dtp->order[k];
			if (p[l] != level)
				continue;
			struct fdt_reg_queue *q = &r->q[s][l];
			double used = 0;
			int j = 0;
			for (; j < q->n; j++) {
				size_t x = FDT_STEP_HEADER + q->m[j].len;
				double c = fdt_msg_cost(dtp, x);
				if (used + c > (double)d[l] + 1e-9)
					break;
				used += c;
				uint8_t *b = q->m[j].buf;
				fdt_put_u32(b, (uint32_t)t);
				fdt_put_u16(b + 4, (uint16_t)j);
				if (r->publish)
					r->publish(r->ctx, l, level == 2 ? 2 : 1, b, x);
				lg->fwd[l]++;
				lg->xfwd[l] += x;
			}
			lg->cost[l] = used;
			lg->held[l] += (uint32_t)(q->n - j);
		}
	}
	for (int l = 0; l < r->L; l++) {
		if (have_q && (!d || !p || p[l] == 0))
			lg->held[l] = (uint32_t)r->q[s][l].n;
		r->st.forwarded += lg->fwd[l];
		r->st.withheld += lg->held[l];
	}
	if (have_q)
		slot_reset(r, s, -1);
}

int fdt_reg_tick(struct fdt_reg *r, int64_t now_us, int64_t (*outq)(void *), void *outq_ctx,
                 struct fdt_reg_log *out, int64_t *next_us)
{
	const struct fdt_dtp *dtp = r->dtp;
	int have = 0;
	/* the log of the last forwarded step is due at start(t+1) + log_offset */
	if (r->log_pending && now_us >= fdt_start_us(dtp, (int64_t)r->log.t + 1) + r->log_offset_us) {
		*out = r->log;
		r->log_pending = 0;
		have = 1;
	}
	int64_t due = fdt_start_us(dtp, r->next) + dtp->T1_us + dtp->T2_us;
	if (now_us >= due) {
		/* a tick later than one step: the older steps are stale and their messages are dropped */
		int64_t last = fdt_floordiv(now_us - dtp->epoch_us - dtp->T1_us - dtp->T2_us, dtp->Ts_us);
		for (int64_t t = r->next; t < last; t++) {
			int s = (int)(t & 1);
			if (r->qt[s] == t) {
				for (int l = 0; l < r->L; l++)
					r->st.stale += (uint64_t)r->q[s][l].n;
				slot_reset(r, s, -1);
			}
		}
		if (have == 0 && r->log_pending) {   /* an undelivered log is replaced */
			*out = r->log;
			have = 1;
		}
		forward(r, last, now_us, outq ? outq(outq_ctx) : -1, &r->log);
		r->log_pending = 1;
		r->next = last + 1;
	}
	int64_t nxt = fdt_start_us(dtp, r->next) + dtp->T1_us + dtp->T2_us;
	if (r->log_pending) {
		int64_t lt = fdt_start_us(dtp, (int64_t)r->log.t + 1) + r->log_offset_us;
		if (lt < nxt)
			nxt = lt;
	}
	*next_us = nxt;
	return have;
}

size_t fdt_reg_log_json(const struct fdt_reg *r, const struct fdt_reg_log *lg, char *buf, size_t n)
{
	size_t k = 0;
#define OUT(...) do { int w_ = snprintf(buf + k, k < n ? n - k : 0, __VA_ARGS__); if (w_ > 0) k += (size_t)w_; } while (0)
#define ARR(name, fmt, expr) do { \
		OUT(",\"" name "\":["); \
		for (int l_ = 0; l_ < r->L; l_++) { \
			OUT("%s" fmt, l_ ? "," : "", expr); \
		} \
		OUT("]"); \
	} while (0)
	OUT("{\"t\":%u,\"have_d\":%d,\"have_p\":%d,\"late_us\":%lld,\"outq\":%lld", lg->t, lg->have_d,
	    lg->have_p, (long long)lg->late_us, (long long)lg->outq);
	ARR("p", "%u", lg->p[l_]);
	ARR("d", "%u", lg->d[l_]);
	ARR("rx", "%u", lg->rx[l_]);
	ARR("fwd", "%u", lg->fwd[l_]);
	ARR("held", "%u", lg->held[l_]);
	ARR("drop", "%u", lg->drop[l_]);
	ARR("x", "%llu", (unsigned long long)lg->xfwd[l_]);
	OUT("}");
#undef ARR
#undef OUT
	if (k >= n && n)
		buf[n - 1] = 0;
	return k < n ? k : n - 1;
}
