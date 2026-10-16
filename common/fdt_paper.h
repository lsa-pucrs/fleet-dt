/*
 * Fleet-DT, paper stack: DTP reader, 0-based step clock and demand bound of the vessel regulator in
 * mode paper (paper Section IV, "Vessel regulator" Definition; SPEC.md Sections 2, 3 and 5).
 *
 * The AS of the paper (fleetdt/as_service.py) and the VE (fleetdt/ve/collector.py) number the steps
 * from 0: start(t) = epoch + t Ts. This header holds the same clock for the C side of the paper stack;
 * the v2 clock of fdt_dtp2.h (start(t) = epoch + (t - 1) Ts) stays for the other modes.
 *
 * The reader accepts both DTP formats, so the topics and L always come from the file:
 *   v1 (dtp.json, SPEC.md):     topics [{"name", "rate" (messages/s), "bound" (bytes), ["jitter" (s)]}],
 *                               "c_b", "c_m"
 *   v2 (SPEC_v2.md):            topics [{"name", "rate" (outputs/s), "n" (messages/output), "s" (bytes),
 *                               "jitter" (s), ...}], "tcp": {"mss", "h_T"}, "mqtt": {"h_A"}
 * When the file holds no "c_b" (or no "c_m"), the reader computes the cost bound of the paper:
 *   c_b = 1 + 2 (h_T + h_W) / MSS,  c_m,l = c_b h_P,l + 2 (h_T + h_W) + 3 (h_A + 2 h_W)
 * with h_P,l the PUBLISH header of boat/b<V>/tx/<name> (QoS 2) and h_W the top-level "h_W" (0 when absent).
 *
 * The reader also keeps what the baseline modes of the regulator use (fdt_reg_paper.h): "U" and "C0"
 * (defaults 0.8 and 300000), the order "order" (topics left out go last, in file order), "required"
 * (v2), and per topic whether it carries video (a layer topic <base>_<j> of "video_layers", default base
 * "video", or a v2 topic of kind output).
 */
#ifndef FDT_PAPER_H
#define FDT_PAPER_H

#include <stddef.h>
#include <stdint.h>

#include "fdt_common.h"   /* FDT_L_MAX, FDT_NAME_MAX, FDT_STEP_HEADER, FDT_DEFAULT_EPOCH, wire helpers */

#define FDT_P_V_MAX 64

struct fdt_ptopic {
	char name[FDT_NAME_MAX];
	double rate;      /* messages per second (v2: outputs per second times n) */
	long s;           /* largest content of one message, bytes, before the 6-byte step header */
	double jitter;    /* J_l, s (0 when absent) */
	int has_jitter;   /* the file gives "jitter" for this topic */
	double c_m;       /* c_m of this topic */
	int required;     /* listed in "required" */
	int video;        /* a video layer topic, or a v2 output topic */
};

struct fdt_pdtp {
	double epoch, Ts, T1, T2;
	double c_b;
	double U, C0;     /* "U" (0.8) and "C0" (300000 bytes per step) */
	int L;
	struct fdt_ptopic topic[FDT_L_MAX];
	int n_required;
	int order[FDT_L_MAX];   /* 0-based topic indices of "order", then the topics left out */
	int M;
	int vessels[FDT_P_V_MAX];
	int64_t epoch_us, Ts_us, T1_us, T2_us;
};

/* Reads a DTP file in either format. Returns 0, or -1 with a message in err. */
int fdt_pdtp_load(const char *path, struct fdt_pdtp *d, char *err, size_t errlen);
int fdt_pdtp_parse(const char *text, size_t len, struct fdt_pdtp *d, char *err, size_t errlen);
/* 0-based index of the topic name of n bytes, or -1 */
int fdt_pdtp_index(const struct fdt_pdtp *d, const char *name, size_t n);

/* ------------------------------------------------------------ 0-based step clock (as_service.py) */

/* start(t) = epoch + t Ts */
int64_t fdt_p_start_us(const struct fdt_pdtp *d, int64_t t);
/* v(t) = start(t) + T1 + T2: phase 3 of step t */
int64_t fdt_p_v_us(const struct fdt_pdtp *d, int64_t t);
/* step t with start(t) <= now < start(t + 1), floor((now - epoch) / Ts) */
int64_t fdt_p_step_at(const struct fdt_pdtp *d, int64_t now_us);
/* step of a raw message that reaches the regulator at now: [v(t-1), v(t)) belongs to step t */
int64_t fdt_p_collect_step(const struct fdt_pdtp *d, int64_t now_us);

/* ------------------------------------------------------------ demand bound and cost */

/* messages of topic l in one collection window: ceil(r_l (Ts + J_l)), J_l = jitter_s */
uint32_t fdt_p_msgs(const struct fdt_pdtp *d, int l, double jitter_s);
/* d_l = ceil(c_b (x_l + 6 n_l) + c_m,l n_l), n_l = fdt_p_msgs, x_l = n_l s_l (SPEC.md Section 5) */
uint32_t fdt_p_demand(const struct fdt_pdtp *d, int l, double jitter_s);
/* channel bytes of one forwarded message of topic l with x content bytes: c_b (x + 6) + c_m,l */
double fdt_p_cost(const struct fdt_pdtp *d, int l, size_t x);

#endif
