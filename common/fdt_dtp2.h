/*
 * Fleet-DT v2: DTP v2 reader, step clock and wire formats of the vessel programs
 * (SPEC_v2.md Sections 2, 3 and 4; paper Sections III to V).
 *
 * Times are integer microseconds of CLOCK_REALTIME (Unix time), which chrony keeps
 * within eps of the GS clock. The v1 definitions (fdt_common.h) stay for the v1
 * programs (fdt_dti, fdt_chmon); the v2 programs use this header.
 */
#ifndef FDT_DTP2_H
#define FDT_DTP2_H

#include <stddef.h>
#include <stdint.h>

#include "fdt_common.h"   /* FDT_L_MAX, FDT_NAME_MAX, fdt_put_u32 & co., FDT_DEFAULT_EPOCH */

#define FDT2_STEP_HEADER 6      /* t:u32, seq:u16 in front of each released message */
#define FDT2_OUT_HEADER 8       /* out:u32, k:u16, n:u16 at the start of each output message */
#define FDT2_V_MAX 64           /* vessels */

enum fdt2_kind { FDT2_STREAM = 0, FDT2_OUTPUT = 1 };

struct fdt2_topic {
	char name[FDT_NAME_MAX];
	double rate;      /* r_l, outputs per second */
	double jitter;    /* J_l, s */
	int n;            /* n_l, messages per output (1 for a stream with one message per output) */
	int s;            /* s_l, bytes of content per message */
	int kind;         /* enum fdt2_kind */
	int qos;          /* 2 for the topics of R, 0 otherwise */
	int required;     /* l in R */
	int needed;       /* l in the initial needed set */
};

struct fdt2_phy {
	double slot, sifs, t_pre, t_sig, t_sym, t_se;
	double h_mac, aifsn_sta, cwmin_sta, cwmax_sta, m_b, aifsn_ap, cwmin_ap, cwmax_ap;
	double rate_init;
	int n_basic;
	double basic_rates[8];
};

struct fdt_dtp2 {
	double epoch, Ts, T1, T2, eps, d_rx, d_m, T_tick, L_w, B_cap, T_min;
	double beta, gamma, alpha1, lambda;
	struct fdt2_phy phy;
	int mss, h_T, h_A;
	int M;                      /* vessels */
	int vessels[FDT2_V_MAX];
	int L;
	struct fdt2_topic topic[FDT_L_MAX];
	int n_required;
	int required[FDT_L_MAX];    /* 0-based topic indices of "required" (R) */
	int n_order;
	int order[FDT_L_MAX];       /* 0-based topic indices of "order" (sigma), R first */
	int n_needed;
	int needed[FDT_L_MAX];
	double goals_rate;
	int goals_s;
	/* the same times in microseconds */
	int64_t epoch_us, Ts_us, T1_us, T2_us, eps_us, d_rx_us, d_m_us, T_tick_us;
	int64_t L_w_bytes, B_cap_bytes;
};

/* Reads and validates a DTP v2 file. Returns 0, or -1 with a message in err. */
int fdt2_dtp_load(const char *path, struct fdt_dtp2 *d, char *err, size_t errlen);
int fdt2_dtp_parse(const char *text, size_t len, struct fdt_dtp2 *d, char *err, size_t errlen);
/* 0-based index of the topic name of n bytes, or -1 */
int fdt2_index(const struct fdt_dtp2 *d, const char *name, size_t n);
/* position of vessel i in "vessels" (0-based), or -1 */
int fdt2_vessel_pos(const struct fdt_dtp2 *d, int vessel);
int fdt2_vessel_max(const struct fdt_dtp2 *d);

/* arrival-curve bound: the number of outputs of topic l whose last message can reach the regulator
 * in an interval of dur_us, ceil(r_l (dur + J_l)) */
uint32_t fdt2_arrival_outputs(const struct fdt_dtp2 *d, int l, int64_t dur_us);

/* MQTT 3.1.1 PUBLISH packet: fixed header (1 + remaining-length bytes) + 2 + topic + (2 if qos > 0)
 * + payload. Returns the bytes of the packet. */
uint32_t fdt2_publish_bytes(size_t topic_len, int qos, size_t payload_len);
/* bytes of the MQTT remaining-length field of the value v */
int fdt2_varint_bytes(uint32_t v);
/* h_P of topic l (SPEC_v2.md Section 2): fixed header + 2 + len(boat/b<V>/tx/<name>) + (2 if qos > 0)
 * + 6, with V the largest vessel index and the remaining length of a message of s_l bytes */
uint32_t fdt2_h_P(const struct fdt_dtp2 *d, int l);

/* step clock (SPEC_v2.md Section 3), vessel clock */
/* start(t) = epoch + (t - 1) Ts, step t >= 1 */
int64_t fdt2_start_us(const struct fdt_dtp2 *d, int64_t t);
/* v(t) = start(t) + T1 + T2 (phase 3) */
int64_t fdt2_v_us(const struct fdt_dtp2 *d, int64_t t);
/* step t with start(t) <= now < start(t + 1) */
int64_t fdt2_step_at(const struct fdt_dtp2 *d, int64_t now_us);
/* step whose collection window [v(t-1), v(t)) holds now */
int64_t fdt2_collect_step(const struct fdt_dtp2 *d, int64_t now_us);
/* data window W(t) on the GS clock: [start + T1 + T2 + eps, start + Ts - eps - d_rx) */
int64_t fdt2_w_start_us(const struct fdt_dtp2 *d, int64_t t);
int64_t fdt2_w_end_us(const struct fdt_dtp2 *d, int64_t t);

#endif
