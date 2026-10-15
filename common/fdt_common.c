#define _GNU_SOURCE
#include "fdt_common.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <cjson/cJSON.h>

static int fail(char *err, size_t n, const char *fmt, ...)
{
	if (err && n) {
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(err, n, fmt, ap);
		va_end(ap);
	}
	return -1;
}

static int num(const cJSON *o, const char *k, double def, int required, double *out, char *err, size_t n)
{
	const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
	if (!v || cJSON_IsNull(v)) {
		if (required)
			return fail(err, n, "DTP: field %s missing", k);
		*out = def;
		return 0;
	}
	if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble))
		return fail(err, n, "DTP: field %s is not a number", k);
	*out = v->valuedouble;
	return 0;
}

int fdt_dtp_index(const struct fdt_dtp *d, const char *name, size_t n)
{
	for (int l = 0; l < d->L; l++)
		if (strlen(d->topic[l].name) == n && !memcmp(d->topic[l].name, name, n))
			return l;
	return -1;
}

int fdt_dtp_parse(const char *text, size_t len, struct fdt_dtp *d, char *err, size_t errlen)
{
	memset(d, 0, sizeof *d);
	cJSON *o = cJSON_ParseWithLength(text, len);
	if (!o || !cJSON_IsObject(o)) {
		cJSON_Delete(o);
		return fail(err, errlen, "DTP: invalid JSON");
	}
	int rc = -1;
	if (num(o, "epoch", FDT_DEFAULT_EPOCH, 0, &d->epoch, err, errlen) ||
	    num(o, "Ts", 0, 1, &d->Ts, err, errlen) || num(o, "T1", 0, 1, &d->T1, err, errlen) ||
	    num(o, "T2", 0, 1, &d->T2, err, errlen) || num(o, "alpha", 0.125, 0, &d->alpha, err, errlen) ||
	    num(o, "U", 0.8, 0, &d->U, err, errlen) || num(o, "C0", 0, 0, &d->C0, err, errlen) ||
	    num(o, "c_b", 1.0, 0, &d->c_b, err, errlen) || num(o, "c_m", 0.0, 0, &d->c_m, err, errlen) ||
	    num(o, "R_PHY", 24e6, 0, &d->R_PHY, err, errlen) || num(o, "h_W", 0.0, 0, &d->h_W, err, errlen))
		goto out;
	if (!(d->Ts > 0) || d->T1 < 0 || d->T2 < 0 || d->T1 + d->T2 >= d->Ts) {
		fail(err, errlen, "DTP: Ts, T1, T2 must satisfy 0 <= T1 + T2 < Ts");
		goto out;
	}
	const cJSON *topics = cJSON_GetObjectItemCaseSensitive(o, "topics");
	if (!cJSON_IsArray(topics) || cJSON_GetArraySize(topics) < 1 || cJSON_GetArraySize(topics) > FDT_L_MAX) {
		fail(err, errlen, "DTP: topics must be an array of 1 to %d topics", FDT_L_MAX);
		goto out;
	}
	const cJSON *t;
	cJSON_ArrayForEach(t, topics) {
		const cJSON *nm = cJSON_GetObjectItemCaseSensitive(t, "name");
		struct fdt_topic *tp = &d->topic[d->L];
		double bound;
		if (!cJSON_IsString(nm) || !*nm->valuestring || strlen(nm->valuestring) >= FDT_NAME_MAX ||
		    strchr(nm->valuestring, '/') || strchr(nm->valuestring, '+') || strchr(nm->valuestring, '#')) {
			fail(err, errlen, "DTP: topic %d has an invalid name", d->L + 1);
			goto out;
		}
		snprintf(tp->name, sizeof tp->name, "%s", nm->valuestring);
		if (num(t, "rate", 0, 1, &tp->rate, err, errlen) || num(t, "bound", 0, 1, &bound, err, errlen))
			goto out;
		if (tp->rate < 0 || bound < 1 || bound > 16 * 1024 * 1024) {
			fail(err, errlen, "DTP: topic %s: rate >= 0 and 1 <= bound <= 16 MiB", tp->name);
			goto out;
		}
		tp->bound = (long)bound;
		if (fdt_dtp_index(d, tp->name, strlen(tp->name)) >= 0) {
			fail(err, errlen, "DTP: topic %s repeated", tp->name);
			goto out;
		}
		d->L++;
	}
	const cJSON *ord = cJSON_GetObjectItemCaseSensitive(o, "order");
	int seen[FDT_L_MAX] = {0};
	if (cJSON_IsArray(ord)) {
		cJSON_ArrayForEach(t, ord) {
			int l = cJSON_IsString(t) ? fdt_dtp_index(d, t->valuestring, strlen(t->valuestring)) : -1;
			if (l < 0 || seen[l]) {
				fail(err, errlen, "DTP: order is not a permutation of the topics");
				goto out;
			}
			seen[l] = 1;
			d->order[d->n_order++] = l;
		}
	}
	for (int l = 0; l < d->L; l++)   /* topics left out of "order" go last, in DTP order */
		if (!seen[l])
			d->order[d->n_order++] = l;
	d->epoch_us = llround(d->epoch * 1e6);
	d->Ts_us = llround(d->Ts * 1e6);
	d->T1_us = llround(d->T1 * 1e6);
	d->T2_us = llround(d->T2 * 1e6);
	rc = 0;
out:
	cJSON_Delete(o);
	return rc;
}

int fdt_dtp_load(const char *path, struct fdt_dtp *d, char *err, size_t errlen)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return fail(err, errlen, "%s: %s", path, strerror(errno));
	char *buf = malloc(1 << 20);
	size_t n = buf ? fread(buf, 1, (1 << 20) - 1, f) : 0;
	fclose(f);
	if (!buf)
		return fail(err, errlen, "out of memory");
	int rc = fdt_dtp_parse(buf, n, d, err, errlen);
	free(buf);
	return rc;
}

uint32_t fdt_msgs_per_step(const struct fdt_dtp *d, int l)
{
	return (uint32_t)ceil(d->topic[l].rate * d->Ts - 1e-9);
}

uint32_t fdt_demand(const struct fdt_dtp *d, int l)
{
	double n = fdt_msgs_per_step(d, l);
	double x = n * (double)d->topic[l].bound;
	double v = ceil(d->c_b * (x + FDT_STEP_HEADER * n) + d->c_m * n - 1e-9);
	return v > 4294967295.0 ? 4294967295u : (uint32_t)v;
}

double fdt_msg_cost(const struct fdt_dtp *d, size_t x)
{
	return d->c_b * (double)x + d->c_m;
}

int64_t fdt_now_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

int64_t fdt_floordiv(int64_t a, int64_t b)
{
	int64_t q = a / b;
	return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

int64_t fdt_start_us(const struct fdt_dtp *d, int64_t t)
{
	return d->epoch_us + t * d->Ts_us;
}

int64_t fdt_step_at(const struct fdt_dtp *d, int64_t now_us)
{
	return fdt_floordiv(now_us - d->epoch_us, d->Ts_us);
}

int64_t fdt_raw_step(const struct fdt_dtp *d, int64_t now_us)
{
	return fdt_floordiv(now_us - d->epoch_us - d->T1_us - d->T2_us, d->Ts_us) + 1;
}

int fdt_sleep_until(int64_t at_us)
{
	struct timespec ts = {(time_t)(at_us / 1000000), (long)(at_us % 1000000) * 1000};
	int rc;
	while ((rc = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL)) == EINTR)
		;
	return rc;
}

size_t fdt_demand_encode(uint8_t *buf, uint32_t t, int L, const uint32_t *d)
{
	fdt_put_u32(buf, t);
	buf[4] = (uint8_t)L;
	for (int l = 0; l < L; l++)
		fdt_put_u32(buf + 5 + 4 * l, d[l]);
	return 5 + 4 * (size_t)L;
}

int fdt_demand_decode(const uint8_t *buf, size_t len, int L, uint32_t *t, uint32_t *d)
{
	if (len < 5 || buf[4] != L || len != 5 + 4 * (size_t)L)
		return -1;
	*t = fdt_get_u32(buf);
	for (int l = 0; l < L; l++)
		d[l] = fdt_get_u32(buf + 5 + 4 * l);
	return 0;
}

int fdt_levels_decode(const uint8_t *buf, size_t len, int L, uint32_t *t, uint8_t *p)
{
	if (len < 5 || buf[4] != L || len != 5 + (size_t)L)
		return -1;
	for (int l = 0; l < L; l++)
		if (buf[5 + l] > 2)
			return -1;
	*t = fdt_get_u32(buf);
	memcpy(p, buf + 5, (size_t)L);
	return 0;
}
