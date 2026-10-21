/* Delivery time of the GS MQTT broker: core of fdt_gsstamp.so (see fdt_gsstamp_core.h). */
#include "fdt_gsstamp_core.h"

#include <inttypes.h>
#include <string.h>

void fdt_gss_init(struct fdt_gss *g, FILE *log)
{
	memset(g, 0, sizeof *g);
	for (int k = 0; k < FDT_GSS_STEPS; k++)
		g->st[k].t = -1;
	g->log = log;
	g->max_t = -1;
}

int fdt_gss_key(const char *topic, const void *payload, size_t len, uint64_t *h, int64_t *t, uint32_t *seq)
{
	if (!topic || !payload || len < 6 || strncmp(topic, "boat/b", 6) != 0)
		return -1;
	const char *p = topic + 6;
	if (*p < '0' || *p > '9')
		return -1;
	while (*p >= '0' && *p <= '9')
		p++;
	if (strncmp(p, "/tx/", 4) != 0 || p[4] == 0)
		return -1;
	/* FNV-1a of the topic; never 0, the mark of a free slot */
	uint64_t x = 1469598103934665603ULL;
	for (const unsigned char *c = (const unsigned char *)topic; *c; c++) {
		x ^= *c;
		x *= 1099511628211ULL;
	}
	*h = x ? x : 1;
	const uint8_t *b = payload;
	*t = (int64_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]);
	*seq = ((uint32_t)b[4] << 8) | b[5];
	return 0;
}

static size_t home(uint64_t h, int64_t t, uint32_t seq)
{
	uint64_t x = h ^ ((uint64_t)t * 0x9E3779B97F4A7C15ULL) ^ ((uint64_t)seq * 0xC2B2AE3D27D4EB4FULL);
	x ^= x >> 29;
	x *= 0xBF58476D1CE4E5B9ULL;
	x ^= x >> 32;
	return (size_t)(x & (FDT_GSS_TAB - 1));
}

static void write_step(struct fdt_gss *g, const struct fdt_gss_step *s)
{
	g->lines++;
	if (!g->log)
		return;
	uint64_t matched = s->n_out - s->unmatched;
	int rc = fprintf(g->log, "{\"t\":%" PRId64 ",\"n_in\":%" PRIu64 ",\"n_out\":%" PRIu64 ",\"unmatched\":%" PRIu64,
	                 s->t, s->n_in, s->n_out, s->unmatched);
	if (rc >= 0)   /* no matched delivery: no delivery time */
		rc = matched ? fprintf(g->log, ",\"max_us\":%.1f,\"mean_us\":%.1f}\n", (double)s->max_ns / 1e3,
		                       s->sum_ns / 1e3 / (double)matched)
		             : fprintf(g->log, ",\"max_us\":null,\"mean_us\":null}\n");
	if (rc < 0)
		g->log_err++;
}

/* the aggregate of step t; create: take the slot (writing the step it held when that step is older) */
static struct fdt_gss_step *step_of(struct fdt_gss *g, int64_t t, int create)
{
	struct fdt_gss_step *s = &g->st[(size_t)(((t % FDT_GSS_STEPS) + FDT_GSS_STEPS) % FDT_GSS_STEPS)];
	if (s->t == t)
		return s;
	if (!create || (s->t >= 0 && s->t > t))
		return NULL;   /* a step older than the ring */
	if (s->t >= 0) {
		write_step(g, s);
		if (g->log)
			fflush(g->log);
	}
	memset(s, 0, sizeof *s);
	s->t = t;
	return s;
}

void fdt_gss_in(struct fdt_gss *g, const char *topic, const void *payload, size_t len, int64_t now_ns)
{
	uint64_t h;
	int64_t t;
	uint32_t seq;
	if (fdt_gss_key(topic, payload, len, &h, &t, &seq))
		return;
	g->n_in++;
	struct fdt_gss_step *s = step_of(g, t, 1);
	if (s)
		s->n_in++;
	else
		g->lost++;
	size_t k0 = home(h, t, seq), pick = k0;
	int64_t oldest = INT64_MAX;
	for (int j = 0; j < FDT_GSS_PROBE; j++) {
		size_t k = (k0 + (size_t)j) & (FDT_GSS_TAB - 1);
		struct fdt_gss_ent *e = &g->tab[k];
		if (e->h == 0 || (e->h == h && e->t == t && e->seq == seq)) {
			pick = k;
			break;
		}
		if (e->in_ns < oldest) {   /* no free slot: replace the oldest reception of the probe */
			oldest = e->in_ns;
			pick = k;
		}
	}
	struct fdt_gss_ent *e = &g->tab[pick];
	e->h = h;
	e->t = t;
	e->seq = seq;
	e->in_ns = now_ns;
}

void fdt_gss_out(struct fdt_gss *g, const char *topic, const void *payload, size_t len, int64_t now_ns)
{
	uint64_t h;
	int64_t t;
	uint32_t seq;
	if (fdt_gss_key(topic, payload, len, &h, &t, &seq))
		return;
	g->n_out++;
	struct fdt_gss_step *s = step_of(g, t, 1);
	if (!s) {
		g->lost++;
		return;
	}
	s->n_out++;
	size_t k0 = home(h, t, seq);
	for (int j = 0; j < FDT_GSS_PROBE; j++) {
		const struct fdt_gss_ent *e = &g->tab[(k0 + (size_t)j) & (FDT_GSS_TAB - 1)];
		if (e->h == h && e->t == t && e->seq == seq) {
			int64_t d = now_ns - e->in_ns;
			if (d < 0)
				d = 0;
			if (d > s->max_ns)
				s->max_ns = d;
			s->sum_ns += (double)d;
			if (d > g->max_ns) {
				g->max_ns = d;
				g->max_t = t;
			}
			return;
		}
	}
	s->unmatched++;
	g->unmatched++;
}

void fdt_gss_flush_all(struct fdt_gss *g)
{
	for (;;) {
		struct fdt_gss_step *best = NULL;
		for (int k = 0; k < FDT_GSS_STEPS; k++)
			if (g->st[k].t >= 0 && (!best || g->st[k].t < best->t))
				best = &g->st[k];
		if (!best)
			break;
		write_step(g, best);
		best->t = -1;
	}
	if (g->log)
		fflush(g->log);
}
