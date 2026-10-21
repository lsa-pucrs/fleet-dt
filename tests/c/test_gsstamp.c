/* Unit test of the delivery-time core of the GS broker plugin (mosquitto/plugins/fdt_gsstamp_core.c). */
#define _GNU_SOURCE
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cjson/cJSON.h>

#include "fdt_gsstamp_core.h"

static int checks, failures;
#define CHECK(c) do { checks++; if (!(c)) { failures++; fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); } } while (0)

static size_t msg(uint8_t *b, uint32_t t, uint16_t s, size_t content)
{
	b[0] = (uint8_t)(t >> 24);
	b[1] = (uint8_t)(t >> 16);
	b[2] = (uint8_t)(t >> 8);
	b[3] = (uint8_t)t;
	b[4] = (uint8_t)(s >> 8);
	b[5] = (uint8_t)s;
	memset(b + 6, 0x5a, content);
	return 6 + content;
}

/* field of the line of step t (NAN when absent or null) */
static double field(const char *text, long t, const char *key)
{
	char head[40];
	snprintf(head, sizeof head, "{\"t\":%ld,", t);
	const char *line = text ? strstr(text, head) : NULL;
	if (!line)
		return NAN;
	const char *e = strchr(line, '\n');
	cJSON *o = cJSON_ParseWithLength(line, e ? (size_t)(e - line) : strlen(line));
	const cJSON *x = o ? cJSON_GetObjectItem(o, key) : NULL;
	double v = (x && cJSON_IsNumber(x)) ? x->valuedouble : NAN;
	cJSON_Delete(o);
	return v;
}

static int lines(const char *text)
{
	int n = 0;
	for (const char *p = text; p && *p; p++)
		n += *p == '\n';
	return n;
}

static void test_key(void)
{
	uint8_t b[64];
	uint64_t h, h2;
	int64_t t;
	uint32_t s;
	size_t n = msg(b, 0x01020304u, 0x0506, 10);
	CHECK(fdt_gss_key("boat/b1/tx/state", b, n, &h, &t, &s) == 0 && t == 0x01020304 && s == 0x0506 && h != 0);
	CHECK(fdt_gss_key("boat/b12/tx/video_3", b, n, &h2, &t, &s) == 0 && h2 != h);
	CHECK(fdt_gss_key("boat/b1/demand", b, n, &h, &t, &s) == -1);
	CHECK(fdt_gss_key("boat/bx/tx/state", b, n, &h, &t, &s) == -1);
	CHECK(fdt_gss_key("boat/b1/tx/", b, n, &h, &t, &s) == -1);
	CHECK(fdt_gss_key("fleet/as", b, n, &h, &t, &s) == -1);
	CHECK(fdt_gss_key("boat/b1/tx/state", b, 5, &h, &t, &s) == -1);   /* shorter than the step header */
}

static void test_delivery(void)
{
	char *buf = NULL;
	size_t len = 0;
	FILE *f = open_memstream(&buf, &len);
	struct fdt_gss *g = malloc(sizeof *g);
	fdt_gss_init(g, f);
	uint8_t b[64];
	/* step 100: two messages of vessel 1, the first one delivered to two subscribers */
	size_t n0 = msg(b, 100, 0, 20);
	fdt_gss_in(g, "boat/b1/tx/state", b, n0, 1000000);
	fdt_gss_out(g, "boat/b1/tx/state", b, n0, 1000300);       /* 0.3 us */
	fdt_gss_out(g, "boat/b1/tx/state", b, n0, 1250000);       /* 250 us */
	size_t n1 = msg(b, 100, 1, 20);
	fdt_gss_in(g, "boat/b1/tx/state", b, n1, 2000000);
	fdt_gss_out(g, "boat/b1/tx/state", b, n1, 2100000);       /* 100 us */
	/* the same step and sequence number on another topic is another message */
	size_t n2 = msg(b, 100, 0, 30);
	fdt_gss_in(g, "boat/b2/tx/state", b, n2, 3000000);
	fdt_gss_out(g, "boat/b2/tx/state", b, n2, 3040000);       /* 40 us */
	/* a delivery without its reception counts as unmatched; other topics are ignored */
	size_t n3 = msg(b, 100, 9, 10);
	fdt_gss_out(g, "boat/b1/tx/imu", b, n3, 3100000);
	fdt_gss_in(g, "boat/b1/demand", b, n3, 3100000);
	fdt_gss_out(g, "fleet/as", b, n3, 3200000);
	/* step 101 */
	size_t n4 = msg(b, 101, 0, 10);
	fdt_gss_in(g, "boat/b1/tx/gps", b, n4, 126000000);
	fdt_gss_out(g, "boat/b1/tx/gps", b, n4, 126000500);
	fflush(f);
	CHECK(lines(buf) == 0);   /* nothing written while the steps are in the ring */
	/* step 100 + FDT_GSS_STEPS takes the slot of step 100: its line is written */
	size_t n5 = msg(b, 100 + FDT_GSS_STEPS, 0, 10);
	fdt_gss_in(g, "boat/b1/tx/gps", b, n5, 2000000000);
	fflush(f);
	CHECK(lines(buf) == 1);
	CHECK(field(buf, 100, "n_in") == 3 && field(buf, 100, "n_out") == 5 && field(buf, 100, "unmatched") == 1);
	CHECK(fabs(field(buf, 100, "max_us") - 250.0) < 1e-9);
	CHECK(fabs(field(buf, 100, "mean_us") - (0.3 + 250 + 100 + 40) / 4.0) < 0.06);
	/* a message of step 100 now falls outside the ring */
	n0 = msg(b, 100, 0, 20);
	fdt_gss_out(g, "boat/b1/tx/state", b, n0, 2000000100);
	CHECK(g->lost == 1);
	fdt_gss_flush_all(g);
	CHECK(lines(buf) == 3);
	CHECK(fabs(field(buf, 101, "max_us") - 0.5) < 1e-9 && field(buf, 101, "n_out") == 1);
	/* the last step has a reception and no delivery: no delivery time */
	CHECK(field(buf, 100 + FDT_GSS_STEPS, "n_in") == 1 && isnan(field(buf, 100 + FDT_GSS_STEPS, "max_us")));
	CHECK(strstr(buf, "\"max_us\":null") != NULL);
	CHECK(g->max_ns == 250000 && g->max_t == 100 && g->n_in == 5 && g->unmatched == 1);
	fclose(f);
	free(buf);
	free(g);
}

/* many receptions: the table keeps the recent ones; a delivery right after its reception always matches */
static void test_table(void)
{
	struct fdt_gss *g = malloc(sizeof *g);
	fdt_gss_init(g, NULL);
	uint8_t b[64];
	int64_t now = 0;
	for (int k = 0; k < 4 * FDT_GSS_TAB; k++) {
		uint32_t t = (uint32_t)(k / 1000);
		size_t n = msg(b, t, (uint16_t)(k % 1000), 8);
		fdt_gss_in(g, k % 2 ? "boat/b1/tx/imu" : "boat/b2/tx/imu", b, n, now);
		fdt_gss_out(g, k % 2 ? "boat/b1/tx/imu" : "boat/b2/tx/imu", b, n, now + 7000);
		now += 10000;
	}
	CHECK(g->unmatched == 0 && g->max_ns == 7000 && g->lost == 0);
	fdt_gss_flush_all(g);
	CHECK(g->lines == (uint64_t)(4 * FDT_GSS_TAB + 999) / 1000);
	free(g);
}

int main(void)
{
	test_key();
	test_delivery();
	test_table();
	if (failures) {
		fprintf(stderr, "test_gsstamp: %d of %d checks failed\n", failures, checks);
		return 1;
	}
	printf("test_gsstamp: %d checks passed\n", checks);
	return 0;
}
