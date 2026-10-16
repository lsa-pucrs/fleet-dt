#define _GNU_SOURCE
#include "fdt_paper.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

/* number field k of o (o may be NULL); def when absent and not required; *has = 1 when present */
static int num(const cJSON *o, const char *k, double def, int required, double *out, int *has, char *err, size_t n)
{
	const cJSON *v = o ? cJSON_GetObjectItemCaseSensitive(o, k) : NULL;
	if (has)
		*has = 0;
	if (!v || cJSON_IsNull(v)) {
		if (required)
			return fail(err, n, "DTP: field %s missing", k);
		*out = def;
		return 0;
	}
	if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble))
		return fail(err, n, "DTP: field %s is not a number", k);
	*out = v->valuedouble;
	if (has)
		*has = 1;
	return 0;
}

int fdt_pdtp_index(const struct fdt_pdtp *d, const char *name, size_t n)
{
	for (int l = 0; l < d->L; l++)
		if (strlen(d->topic[l].name) == n && !memcmp(d->topic[l].name, name, n))
			return l;
	return -1;
}

static int varint_bytes(uint32_t v)
{
	return v < 128 ? 1 : v < 16384 ? 2 : v < 2097152 ? 3 : 4;
}

/* PUBLISH header of a QoS 2 message of s + 6 payload bytes on boat/b<V>/tx/<name>: the packet minus
 * the payload */
static double publish_header(int V, const char *name, long s)
{
	char topic[160];
	int tl = snprintf(topic, sizeof topic, "boat/b%d/tx/%s", V > 0 ? V : 1, name);
	uint32_t rem = (uint32_t)(2 + tl + 2 + FDT_STEP_HEADER + s);
	return 1.0 + varint_bytes(rem) + 2 + tl + 2;
}

int fdt_pdtp_parse(const char *text, size_t len, struct fdt_pdtp *d, char *err, size_t errlen)
{
	memset(d, 0, sizeof *d);
	cJSON *o = cJSON_ParseWithLength(text, len);
	if (!o || !cJSON_IsObject(o)) {
		cJSON_Delete(o);
		return fail(err, errlen, "DTP: invalid JSON");
	}
	int rc = -1, has_cb, has_cm;
	double c_m, h_W, mss, h_T, h_A;
	const cJSON *tcp = cJSON_GetObjectItemCaseSensitive(o, "tcp");
	const cJSON *mqtt = cJSON_GetObjectItemCaseSensitive(o, "mqtt");
	if (num(o, "U", 0.8, 0, &d->U, NULL, err, errlen) || num(o, "C0", 300000.0, 0, &d->C0, NULL, err, errlen) ||
	    num(o, "epoch", FDT_DEFAULT_EPOCH, 0, &d->epoch, NULL, err, errlen) ||
	    num(o, "Ts", 0, 1, &d->Ts, NULL, err, errlen) || num(o, "T1", 0, 1, &d->T1, NULL, err, errlen) ||
	    num(o, "T2", 0, 1, &d->T2, NULL, err, errlen) || num(o, "c_b", 1.0, 0, &d->c_b, &has_cb, err, errlen) ||
	    num(o, "c_m", 0.0, 0, &c_m, &has_cm, err, errlen) || num(o, "h_W", 0.0, 0, &h_W, NULL, err, errlen) ||
	    num(tcp, "mss", 1448, 0, &mss, NULL, err, errlen) || num(tcp, "h_T", 80, 0, &h_T, NULL, err, errlen) ||
	    num(mqtt, "h_A", 4, 0, &h_A, NULL, err, errlen))
		goto out;
	if (!(d->Ts > 0) || d->T1 < 0 || d->T2 < 0 || d->T1 + d->T2 >= d->Ts) {
		fail(err, errlen, "DTP: Ts, T1, T2 must satisfy 0 <= T1 + T2 < Ts");
		goto out;
	}
	if (!(mss >= 1) || d->c_b < 0 || c_m < 0 || h_W < 0) {
		fail(err, errlen, "DTP: c_b, c_m, h_W must be >= 0 and mss >= 1");
		goto out;
	}
	const cJSON *vs = cJSON_GetObjectItemCaseSensitive(o, "vessels"), *e;
	if (cJSON_IsArray(vs))
		cJSON_ArrayForEach(e, vs) {
			if (!cJSON_IsNumber(e) || e->valuedouble < 1 || d->M >= FDT_P_V_MAX) {
				fail(err, errlen, "DTP: vessels must hold up to %d indices >= 1", FDT_P_V_MAX);
				goto out;
			}
			d->vessels[d->M++] = (int)e->valuedouble;
		}
	int V = 1;
	for (int k = 0; k < d->M; k++)
		if (d->vessels[k] > V)
			V = d->vessels[k];
	if (!has_cb)
		d->c_b = 1.0 + 2.0 * (h_T + h_W) / mss;
	/* video layer topics: <base>_<j>, 1 <= j <= layers, of "video_layers" (base "video", 16 layers) */
	char vbase[FDT_NAME_MAX] = "video";
	double vlayers = 16;
	const cJSON *vl = cJSON_GetObjectItemCaseSensitive(o, "video_layers");
	if (cJSON_IsObject(vl)) {
		const cJSON *vb = cJSON_GetObjectItemCaseSensitive(vl, "base");
		if (cJSON_IsString(vb) && strlen(vb->valuestring) < sizeof vbase)
			snprintf(vbase, sizeof vbase, "%s", vb->valuestring);
		if (num(vl, "layers", 16, 0, &vlayers, NULL, err, errlen))
			goto out;
	}
	const cJSON *topics = cJSON_GetObjectItemCaseSensitive(o, "topics");
	if (!cJSON_IsArray(topics) || cJSON_GetArraySize(topics) < 1 || cJSON_GetArraySize(topics) > FDT_L_MAX) {
		fail(err, errlen, "DTP: topics must be an array of 1 to %d topics", FDT_L_MAX);
		goto out;
	}
	cJSON_ArrayForEach(e, topics) {
		const cJSON *nm = cJSON_GetObjectItemCaseSensitive(e, "name");
		struct fdt_ptopic *tp = &d->topic[d->L];
		double bound, s, n;
		int has_bound, has_s;
		if (!cJSON_IsString(nm) || !*nm->valuestring || strlen(nm->valuestring) >= FDT_NAME_MAX ||
		    strpbrk(nm->valuestring, "/+#")) {
			fail(err, errlen, "DTP: topic %d has an invalid name", d->L + 1);
			goto out;
		}
		snprintf(tp->name, sizeof tp->name, "%s", nm->valuestring);
		if (fdt_pdtp_index(d, tp->name, strlen(tp->name)) >= 0) {
			fail(err, errlen, "DTP: topic %s repeated", tp->name);
			goto out;
		}
		if (num(e, "rate", 0, 1, &tp->rate, NULL, err, errlen) ||
		    num(e, "bound", 0, 0, &bound, &has_bound, err, errlen) || num(e, "s", 0, 0, &s, &has_s, err, errlen) ||
		    num(e, "n", 1, 0, &n, NULL, err, errlen) ||
		    num(e, "jitter", 0, 0, &tp->jitter, &tp->has_jitter, err, errlen))
			goto out;
		if (!has_bound && !has_s) {
			fail(err, errlen, "DTP: topic %s needs bound (v1) or s (v2)", tp->name);
			goto out;
		}
		double b = has_bound ? bound : s;
		if (tp->rate < 0 || tp->jitter < 0 || b < 1 || b > 16 * 1024 * 1024 || n < 1 || n > 65535) {
			fail(err, errlen, "DTP: topic %s: rate >= 0, jitter >= 0, 1 <= bound <= 16 MiB, 1 <= n <= 65535",
			     tp->name);
			goto out;
		}
		tp->s = (long)b;
		if (!has_bound)   /* v2: rate counts outputs of n messages */
			tp->rate *= n;
		const cJSON *kind = cJSON_GetObjectItemCaseSensitive(e, "kind");
		size_t bl = strlen(vbase);
		if (cJSON_IsString(kind) && !strcmp(kind->valuestring, "output")) {
			tp->video = 1;
		} else if (!strncmp(tp->name, vbase, bl) && tp->name[bl] == '_' && tp->name[bl + 1]) {
			char *endp = NULL;
			long j = strtol(tp->name + bl + 1, &endp, 10);
			tp->video = endp && !*endp && j >= 1 && j <= (long)vlayers;
		}
		tp->c_m = has_cm ? c_m
		                 : d->c_b * publish_header(V, tp->name, tp->s) + 2.0 * (h_T + h_W) + 3.0 * (h_A + 2.0 * h_W);
		d->L++;
	}
	/* order (sigma of the DTP) and required: names of topics; unknown or repeated names are errors */
	int seen[FDT_L_MAX] = {0}, n_ord = 0;
	const cJSON *ord = cJSON_GetObjectItemCaseSensitive(o, "order");
	if (cJSON_IsArray(ord))
		cJSON_ArrayForEach(e, ord) {
			int l = cJSON_IsString(e) ? fdt_pdtp_index(d, e->valuestring, strlen(e->valuestring)) : -1;
			if (l < 0 || seen[l]) {
				fail(err, errlen, "DTP: order is not a list of distinct topics");
				goto out;
			}
			seen[l] = 1;
			d->order[n_ord++] = l;
		}
	for (int l = 0; l < d->L; l++)
		if (!seen[l])
			d->order[n_ord++] = l;
	const cJSON *req = cJSON_GetObjectItemCaseSensitive(o, "required");
	if (cJSON_IsArray(req))
		cJSON_ArrayForEach(e, req) {
			int l = cJSON_IsString(e) ? fdt_pdtp_index(d, e->valuestring, strlen(e->valuestring)) : -1;
			if (l < 0) {
				fail(err, errlen, "DTP: required lists an unknown topic");
				goto out;
			}
			if (!d->topic[l].required)
				d->n_required++;
			d->topic[l].required = 1;
		}
	d->epoch_us = llround(d->epoch * 1e6);
	d->Ts_us = llround(d->Ts * 1e6);
	d->T1_us = llround(d->T1 * 1e6);
	d->T2_us = llround(d->T2 * 1e6);
	rc = 0;
out:
	cJSON_Delete(o);
	return rc;
}

int fdt_pdtp_load(const char *path, struct fdt_pdtp *d, char *err, size_t errlen)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return fail(err, errlen, "%s: %s", path, strerror(errno));
	char *buf = malloc(1 << 20);
	size_t n = buf ? fread(buf, 1, (1 << 20) - 1, f) : 0;
	fclose(f);
	if (!buf)
		return fail(err, errlen, "out of memory");
	int rc = fdt_pdtp_parse(buf, n, d, err, errlen);
	free(buf);
	return rc;
}

/* ------------------------------------------------------------ clock */

int64_t fdt_p_start_us(const struct fdt_pdtp *d, int64_t t)
{
	return d->epoch_us + t * d->Ts_us;
}

int64_t fdt_p_v_us(const struct fdt_pdtp *d, int64_t t)
{
	return fdt_p_start_us(d, t) + d->T1_us + d->T2_us;
}

int64_t fdt_p_step_at(const struct fdt_pdtp *d, int64_t now_us)
{
	return fdt_floordiv(now_us - d->epoch_us, d->Ts_us);
}

int64_t fdt_p_collect_step(const struct fdt_pdtp *d, int64_t now_us)
{
	/* v(t-1) <= now < v(t)  <=>  t - 1 <= (now - epoch - T1 - T2) / Ts < t */
	return fdt_floordiv(now_us - d->epoch_us - d->T1_us - d->T2_us, d->Ts_us) + 1;
}

/* ------------------------------------------------------------ demand */

uint32_t fdt_p_msgs(const struct fdt_pdtp *d, int l, double jitter_s)
{
	double v = ceil(d->topic[l].rate * (d->Ts + (jitter_s > 0 ? jitter_s : 0)) - 1e-9);
	return v < 0 ? 0 : v > 1e6 ? 1000000u : (uint32_t)v;
}

uint32_t fdt_p_demand(const struct fdt_pdtp *d, int l, double jitter_s)
{
	double n = fdt_p_msgs(d, l, jitter_s);
	double x = n * (double)d->topic[l].s;
	double v = ceil(d->c_b * (x + FDT_STEP_HEADER * n) + d->topic[l].c_m * n - 1e-9);
	return v > 4294967295.0 ? 4294967295u : (uint32_t)v;
}

double fdt_p_cost(const struct fdt_pdtp *d, int l, size_t x)
{
	return d->c_b * (double)(x + FDT_STEP_HEADER) + d->topic[l].c_m;
}
