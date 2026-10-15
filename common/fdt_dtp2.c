#define _GNU_SOURCE
#include "fdt_dtp2.h"

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

/* number field k of o; def when absent and not required */
static int num(const cJSON *o, const char *k, double def, int required, double *out, char *err, size_t n)
{
	const cJSON *v = o ? cJSON_GetObjectItemCaseSensitive(o, k) : NULL;
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

static int is_int(double v)
{
	return v == floor(v) && fabs(v) < 2147483647.0;
}

int fdt2_index(const struct fdt_dtp2 *d, const char *name, size_t n)
{
	for (int l = 0; l < d->L; l++)
		if (strlen(d->topic[l].name) == n && !memcmp(d->topic[l].name, name, n))
			return l;
	return -1;
}

int fdt2_vessel_pos(const struct fdt_dtp2 *d, int vessel)
{
	for (int k = 0; k < d->M; k++)
		if (d->vessels[k] == vessel)
			return k;
	return -1;
}

int fdt2_vessel_max(const struct fdt_dtp2 *d)
{
	int m = 0;
	for (int k = 0; k < d->M; k++)
		if (d->vessels[k] > m)
			m = d->vessels[k];
	return m;
}

/* list of topic names -> indices; every name must exist and appear once */
static int name_list(const struct fdt_dtp2 *d, const cJSON *arr, const char *field, int *idx, int *cnt,
                     char *err, size_t n)
{
	int seen[FDT_L_MAX] = {0};
	const cJSON *e;
	*cnt = 0;
	if (!arr || cJSON_IsNull(arr))
		return 0;
	if (!cJSON_IsArray(arr))
		return fail(err, n, "DTP: %s must be an array of topic names", field);
	cJSON_ArrayForEach(e, arr) {
		int l = cJSON_IsString(e) ? fdt2_index(d, e->valuestring, strlen(e->valuestring)) : -1;
		if (l < 0)
			return fail(err, n, "DTP: %s holds an unknown topic", field);
		if (seen[l])
			return fail(err, n, "DTP: %s repeats topic %s", field, d->topic[l].name);
		seen[l] = 1;
		idx[(*cnt)++] = l;
	}
	return 0;
}

static int parse_phy(const cJSON *p, struct fdt2_phy *ph, char *err, size_t n)
{
	if (num(p, "slot", 9e-6, 0, &ph->slot, err, n) || num(p, "sifs", 10e-6, 0, &ph->sifs, err, n) ||
	    num(p, "t_pre", 16e-6, 0, &ph->t_pre, err, n) || num(p, "t_sig", 4e-6, 0, &ph->t_sig, err, n) ||
	    num(p, "t_sym", 4e-6, 0, &ph->t_sym, err, n) || num(p, "t_se", 6e-6, 0, &ph->t_se, err, n) ||
	    num(p, "h_mac", 38, 0, &ph->h_mac, err, n) || num(p, "aifsn_sta", 3, 0, &ph->aifsn_sta, err, n) ||
	    num(p, "cwmin_sta", 15, 0, &ph->cwmin_sta, err, n) || num(p, "cwmax_sta", 1023, 0, &ph->cwmax_sta, err, n) ||
	    num(p, "m_b", 6, 0, &ph->m_b, err, n) || num(p, "aifsn_ap", 1, 0, &ph->aifsn_ap, err, n) ||
	    num(p, "cwmin_ap", 7, 0, &ph->cwmin_ap, err, n) || num(p, "cwmax_ap", 1023, 0, &ph->cwmax_ap, err, n) ||
	    num(p, "rate_init", 24e6, 0, &ph->rate_init, err, n))
		return -1;
	const cJSON *br = p ? cJSON_GetObjectItemCaseSensitive(p, "basic_rates") : NULL, *e;
	ph->n_basic = 0;
	if (br && cJSON_IsArray(br))
		cJSON_ArrayForEach(e, br) {
			if (!cJSON_IsNumber(e) || ph->n_basic >= 8)
				return fail(err, n, "DTP: phy.basic_rates must hold up to 8 numbers");
			ph->basic_rates[ph->n_basic++] = e->valuedouble;
		}
	return 0;
}

int fdt2_dtp_parse(const char *text, size_t len, struct fdt_dtp2 *d, char *err, size_t errlen)
{
	memset(d, 0, sizeof *d);
	cJSON *o = cJSON_ParseWithLength(text, len);
	if (!o || !cJSON_IsObject(o)) {
		cJSON_Delete(o);
		return fail(err, errlen, "DTP: invalid JSON");
	}
	int rc = -1;
	double mss, h_T, h_A, gs;
	const cJSON *tcp = cJSON_GetObjectItemCaseSensitive(o, "tcp");
	const cJSON *mqtt = cJSON_GetObjectItemCaseSensitive(o, "mqtt");
	const cJSON *goals = cJSON_GetObjectItemCaseSensitive(o, "goals");
	if (num(o, "epoch", FDT_DEFAULT_EPOCH, 0, &d->epoch, err, errlen) ||
	    num(o, "Ts", 0, 1, &d->Ts, err, errlen) || num(o, "T1", 0, 1, &d->T1, err, errlen) ||
	    num(o, "T2", 0, 1, &d->T2, err, errlen) || num(o, "eps", 0.002, 0, &d->eps, err, errlen) ||
	    num(o, "d_rx", 0.002, 0, &d->d_rx, err, errlen) || num(o, "d_m", 0.001, 0, &d->d_m, err, errlen) ||
	    num(o, "T_tick", 0.001, 0, &d->T_tick, err, errlen) || num(o, "L_w", 16384, 0, &d->L_w, err, errlen) ||
	    num(o, "B_cap", 1048576, 0, &d->B_cap, err, errlen) || num(o, "T_min", 0.005, 0, &d->T_min, err, errlen) ||
	    num(o, "beta", 0.05, 0, &d->beta, err, errlen) || num(o, "gamma", 0.01, 0, &d->gamma, err, errlen) ||
	    num(o, "alpha1", 0.5, 0, &d->alpha1, err, errlen) || num(o, "lambda", 0.25, 0, &d->lambda, err, errlen) ||
	    num(tcp, "mss", 1448, 0, &mss, err, errlen) || num(tcp, "h_T", 80, 0, &h_T, err, errlen) ||
	    num(mqtt, "h_A", 4, 0, &h_A, err, errlen) || num(goals, "rate", 1, 0, &d->goals_rate, err, errlen) ||
	    num(goals, "s", 200, 0, &gs, err, errlen) ||
	    parse_phy(cJSON_GetObjectItemCaseSensitive(o, "phy"), &d->phy, err, errlen))
		goto out;
	d->mss = (int)mss;
	d->h_T = (int)h_T;
	d->h_A = (int)h_A;
	d->goals_s = (int)gs;
	if (!(d->Ts > 0) || !(d->T1 > 0) || !(d->T2 > 0) || d->T1 + d->T2 >= d->Ts) {
		fail(err, errlen, "DTP: Ts, T1, T2 must satisfy T1 > 0, T2 > 0, T1 + T2 < Ts");
		goto out;
	}
	if (d->eps < 0 || d->d_rx < 0 || d->d_m < 0 || d->Ts - d->T1 - d->T2 - 2 * d->eps - d->d_rx <= 0) {
		fail(err, errlen, "DTP: T_W = T3 - 2 eps - d_rx must be positive");
		goto out;
	}
	if (!(d->T_tick > 0) || d->T_tick > d->Ts || !(d->L_w >= 1) || !(d->B_cap >= 0) || mss < 1) {
		fail(err, errlen, "DTP: T_tick in (0, Ts], L_w >= 1, B_cap >= 0, mss >= 1");
		goto out;
	}
	/* vessels */
	const cJSON *vs = cJSON_GetObjectItemCaseSensitive(o, "vessels"), *e;
	if (vs && !cJSON_IsNull(vs)) {
		if (!cJSON_IsArray(vs)) {
			fail(err, errlen, "DTP: vessels must be an array of indices");
			goto out;
		}
		cJSON_ArrayForEach(e, vs) {
			if (!cJSON_IsNumber(e) || !is_int(e->valuedouble) || e->valuedouble < 1 || d->M >= FDT2_V_MAX ||
			    fdt2_vessel_pos(d, (int)e->valuedouble) >= 0) {
				fail(err, errlen, "DTP: vessels must hold up to %d distinct indices >= 1", FDT2_V_MAX);
				goto out;
			}
			d->vessels[d->M++] = (int)e->valuedouble;
		}
	}
	/* topics */
	const cJSON *topics = cJSON_GetObjectItemCaseSensitive(o, "topics");
	if (!cJSON_IsArray(topics) || cJSON_GetArraySize(topics) < 1 || cJSON_GetArraySize(topics) > FDT_L_MAX) {
		fail(err, errlen, "DTP: topics must be an array of 1 to %d topics", FDT_L_MAX);
		goto out;
	}
	cJSON_ArrayForEach(e, topics) {
		const cJSON *nm = cJSON_GetObjectItemCaseSensitive(e, "name");
		const cJSON *kd = cJSON_GetObjectItemCaseSensitive(e, "kind");
		struct fdt2_topic *tp = &d->topic[d->L];
		double nn, ss, qos;
		if (!cJSON_IsString(nm) || !*nm->valuestring || strlen(nm->valuestring) >= FDT_NAME_MAX ||
		    strpbrk(nm->valuestring, "/+#")) {
			fail(err, errlen, "DTP: topic %d has an invalid name", d->L + 1);
			goto out;
		}
		snprintf(tp->name, sizeof tp->name, "%s", nm->valuestring);
		if (fdt2_index(d, tp->name, strlen(tp->name)) >= 0) {
			fail(err, errlen, "DTP: topic %s repeated", tp->name);
			goto out;
		}
		if (num(e, "rate", 0, 1, &tp->rate, err, errlen) || num(e, "jitter", 0, 0, &tp->jitter, err, errlen) ||
		    num(e, "n", 1, 0, &nn, err, errlen) || num(e, "s", 0, 1, &ss, err, errlen) ||
		    num(e, "qos", 0, 1, &qos, err, errlen))
			goto out;
		if (tp->rate < 0 || tp->jitter < 0 || !is_int(nn) || nn < 1 || nn > 65535 || !is_int(ss) || ss < 1 ||
		    ss > 1048576 || (qos != 0 && qos != 2)) {
			fail(err, errlen, "DTP: topic %s: rate >= 0, jitter >= 0, 1 <= n <= 65535, 1 <= s <= 1 MiB, qos 0 or 2",
			     tp->name);
			goto out;
		}
		tp->n = (int)nn;
		tp->s = (int)ss;
		tp->qos = (int)qos;
		if (!cJSON_IsString(kd) || (strcmp(kd->valuestring, "stream") && strcmp(kd->valuestring, "output"))) {
			fail(err, errlen, "DTP: topic %s: kind must be stream or output", tp->name);
			goto out;
		}
		tp->kind = strcmp(kd->valuestring, "output") ? FDT2_STREAM : FDT2_OUTPUT;
		if (tp->kind == FDT2_OUTPUT && tp->s <= FDT2_OUT_HEADER) {
			fail(err, errlen, "DTP: output topic %s: s must exceed the %d-byte output header", tp->name,
			     FDT2_OUT_HEADER);
			goto out;
		}
		d->L++;
	}
	if (name_list(d, cJSON_GetObjectItemCaseSensitive(o, "required"), "required", d->required, &d->n_required,
	              err, errlen) ||
	    name_list(d, cJSON_GetObjectItemCaseSensitive(o, "order"), "order", d->order, &d->n_order, err, errlen) ||
	    name_list(d, cJSON_GetObjectItemCaseSensitive(o, "needed"), "needed", d->needed, &d->n_needed, err, errlen))
		goto out;
	for (int k = 0; k < d->n_required; k++)
		d->topic[d->required[k]].required = 1;
	for (int k = 0; k < d->n_needed; k++)
		d->topic[d->needed[k]].needed = 1;
	/* rule: required => qos 2, others qos 0 */
	for (int l = 0; l < d->L; l++)
		if (d->topic[l].qos != (d->topic[l].required ? 2 : 0)) {
			fail(err, errlen, "DTP: topic %s: qos must be %d (%s required)", d->topic[l].name,
			     d->topic[l].required ? 2 : 0, d->topic[l].required ? "in" : "not in");
			goto out;
		}
	/* order: a permutation of the topics that starts with the topics of required; absent: R first,
	 * then the other topics in DTP order */
	if (d->n_order == 0) {
		for (int k = 0; k < d->n_required; k++)
			d->order[d->n_order++] = d->required[k];
		for (int l = 0; l < d->L; l++)
			if (!d->topic[l].required)
				d->order[d->n_order++] = l;
	} else if (d->n_order != d->L) {
		fail(err, errlen, "DTP: order must list every topic once");
		goto out;
	}
	for (int k = 0; k < d->n_required; k++)
		if (!d->topic[d->order[k]].required) {
			fail(err, errlen, "DTP: order must start with the topics of required");
			goto out;
		}
	/* N_i(t) contains R */
	if (d->n_needed > 0)
		for (int k = 0; k < d->n_required; k++)
			if (!d->topic[d->required[k]].needed) {
				fail(err, errlen, "DTP: needed must contain the topics of required");
				goto out;
			}
	d->epoch_us = llround(d->epoch * 1e6);
	d->Ts_us = llround(d->Ts * 1e6);
	d->T1_us = llround(d->T1 * 1e6);
	d->T2_us = llround(d->T2 * 1e6);
	d->eps_us = llround(d->eps * 1e6);
	d->d_rx_us = llround(d->d_rx * 1e6);
	d->d_m_us = llround(d->d_m * 1e6);
	d->T_tick_us = llround(d->T_tick * 1e6);
	if (d->T_tick_us < 1)
		d->T_tick_us = 1;
	d->L_w_bytes = llround(d->L_w);
	d->B_cap_bytes = llround(d->B_cap);
	rc = 0;
out:
	cJSON_Delete(o);
	return rc;
}

int fdt2_dtp_load(const char *path, struct fdt_dtp2 *d, char *err, size_t errlen)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return fail(err, errlen, "%s: %s", path, strerror(errno));
	char *buf = malloc(1 << 20);
	size_t n = buf ? fread(buf, 1, (1 << 20) - 1, f) : 0;
	fclose(f);
	if (!buf)
		return fail(err, errlen, "out of memory");
	int rc = fdt2_dtp_parse(buf, n, d, err, errlen);
	free(buf);
	return rc;
}

uint32_t fdt2_arrival_outputs(const struct fdt_dtp2 *d, int l, int64_t dur_us)
{
	const struct fdt2_topic *tp = &d->topic[l];
	if (tp->rate <= 0)
		return 0;
	double v = ceil(tp->rate * ((double)dur_us * 1e-6 + tp->jitter) - 1e-9);
	return v < 0 ? 0 : v > 65535 ? 65535 : (uint32_t)v;
}

int fdt2_varint_bytes(uint32_t v)
{
	return v < 128 ? 1 : v < 16384 ? 2 : v < 2097152 ? 3 : 4;
}

uint32_t fdt2_publish_bytes(size_t topic_len, int qos, size_t payload_len)
{
	uint32_t rem = (uint32_t)(2 + topic_len + (qos > 0 ? 2 : 0) + payload_len);
	return 1 + (uint32_t)fdt2_varint_bytes(rem) + rem;
}

uint32_t fdt2_h_P(const struct fdt_dtp2 *d, int l)
{
	char topic[160];
	int V = fdt2_vessel_max(d);
	int tl = snprintf(topic, sizeof topic, "boat/b%d/tx/%s", V > 0 ? V : 1, d->topic[l].name);
	uint32_t total = fdt2_publish_bytes((size_t)tl, d->topic[l].qos, FDT2_STEP_HEADER + (size_t)d->topic[l].s);
	return total - (uint32_t)d->topic[l].s;   /* = fixed header + 2 + topic + mid + 6 */
}

int64_t fdt2_start_us(const struct fdt_dtp2 *d, int64_t t)
{
	return d->epoch_us + (t - 1) * d->Ts_us;
}

int64_t fdt2_v_us(const struct fdt_dtp2 *d, int64_t t)
{
	return fdt2_start_us(d, t) + d->T1_us + d->T2_us;
}

int64_t fdt2_step_at(const struct fdt_dtp2 *d, int64_t now_us)
{
	return fdt_floordiv(now_us - d->epoch_us, d->Ts_us) + 1;
}

int64_t fdt2_collect_step(const struct fdt_dtp2 *d, int64_t now_us)
{
	/* v(t-1) <= now < v(t)  <=>  t - 2 <= (now - epoch - T1 - T2) / Ts < t - 1 */
	return fdt_floordiv(now_us - d->epoch_us - d->T1_us - d->T2_us, d->Ts_us) + 2;
}

int64_t fdt2_w_start_us(const struct fdt_dtp2 *d, int64_t t)
{
	return fdt2_start_us(d, t) + d->T1_us + d->T2_us + d->eps_us;
}

int64_t fdt2_w_end_us(const struct fdt_dtp2 *d, int64_t t)
{
	return fdt2_start_us(d, t) + d->Ts_us - d->eps_us - d->d_rx_us;
}
