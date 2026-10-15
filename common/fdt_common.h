/*
 * Fleet-DT: definitions shared by the C programs (vessel regulator, DTI program,
 * channel monitor). Reads the DTP file (dtp.json, SPEC.md Section 2) with cJSON
 * and holds the step clock and the wire formats (SPEC.md Section 3).
 *
 * Times are integer microseconds of CLOCK_REALTIME (Unix time), synchronized by
 * chrony on every host.
 */
#ifndef FDT_COMMON_H
#define FDT_COMMON_H

#include <stddef.h>
#include <stdint.h>

#define FDT_L_MAX 32
#define FDT_NAME_MAX 48
#define FDT_STEP_HEADER 6     /* t:u32, s:u16 */
#define FDT_VIDEO_HEADER 8    /* frame:u32, k:u16, n:u16 */
/* epoch of a DTP file with "epoch": null: 2026-01-01T00:00:00Z. Every program
 * uses the same value, so the step index t agrees on every host. */
#define FDT_DEFAULT_EPOCH 1767225600.0

struct fdt_topic {
	char name[FDT_NAME_MAX];
	double rate;   /* messages per second */
	long bound;    /* largest content of one message, bytes, before the step header */
};

struct fdt_dtp {
	double epoch, Ts, T1, T2, alpha, U, C0, c_b, c_m, R_PHY, h_W;
	int L;
	struct fdt_topic topic[FDT_L_MAX];
	int n_order;
	int order[FDT_L_MAX];   /* 0-based topic indices of "order" (sigma of the AS) */
	/* the same times in microseconds */
	int64_t epoch_us, Ts_us, T1_us, T2_us;
};

/* Reads a DTP file. Returns 0, or -1 with a message in err. */
int fdt_dtp_load(const char *path, struct fdt_dtp *d, char *err, size_t errlen);
/* Parses a DTP held in memory (len bytes). */
int fdt_dtp_parse(const char *text, size_t len, struct fdt_dtp *d, char *err, size_t errlen);
/* 0-based index of a topic name of n bytes, or -1. */
int fdt_dtp_index(const struct fdt_dtp *d, const char *name, size_t n);
/* n_l = ceil(r_l Ts): messages of topic l in one step. */
uint32_t fdt_msgs_per_step(const struct fdt_dtp *d, int l);
/* d_l = ceil(c_b (x_l + 6 n_l) + c_m n_l) with x_l = n_l s_l (SPEC.md Section 5). */
uint32_t fdt_demand(const struct fdt_dtp *d, int l);
/* channel bytes c_b x + c_m of one forwarded message with x bytes (step header included) */
double fdt_msg_cost(const struct fdt_dtp *d, size_t x);

/* clock */
int64_t fdt_now_us(void);
/* floor division of microseconds */
int64_t fdt_floordiv(int64_t a, int64_t b);
/* start(t) = epoch + t Ts */
int64_t fdt_start_us(const struct fdt_dtp *d, int64_t t);
/* step t with start(t) <= now < start(t+1) */
int64_t fdt_step_at(const struct fdt_dtp *d, int64_t now_us);
/* step of a raw message that reaches the vessel regulator at now:
 * [start(t-1) + T1 + T2, start(t) + T1 + T2) belongs to step t */
int64_t fdt_raw_step(const struct fdt_dtp *d, int64_t now_us);
/* sleeps until the absolute Unix time at_us (clock_nanosleep, TIMER_ABSTIME) */
int fdt_sleep_until(int64_t at_us);

/* wire formats, network byte order */
static inline void fdt_put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static inline void fdt_put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}
static inline uint32_t fdt_get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static inline uint16_t fdt_get_u16(const uint8_t *p)
{
	return (uint16_t)(p[0] << 8 | p[1]);
}
/* demand table t:u32, L:u8, d:u32*L; returns the length (5 + 4L) */
size_t fdt_demand_encode(uint8_t *buf, uint32_t t, int L, const uint32_t *d);
/* returns 0 when the table holds L entries */
int fdt_demand_decode(const uint8_t *buf, size_t len, int L, uint32_t *t, uint32_t *d);
/* priority table t:u32, L:u8, p:u8*L; returns 0 when it holds L levels in {0,1,2} */
int fdt_levels_decode(const uint8_t *buf, size_t len, int L, uint32_t *t, uint8_t *p);

#endif
