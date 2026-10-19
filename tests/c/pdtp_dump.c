/*
 * Test helper: prints what the DTP reader of the paper stack (common/fdt_paper.c) computes from a DTP file,
 * so tests/test_paper_harness.py can compare it with fleetdt/as_model.py and vessel/publishers/video_layers.py
 * (cost bound of eq. cons: c_b, c_k and c_m,l; the demand d_l with the jitter allowance J).
 *
 *   pdtp_dump DTP_FILE [J_SECONDS]
 *   -> {"c_b": ..., "topics": [{"name": ..., "c_m": ..., "c_k": ..., "d": ...}, ...]}   (%.17g)
 */
#include <stdio.h>
#include <stdlib.h>

#include "fdt_paper.h"

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: pdtp_dump DTP_FILE [J_SECONDS]\n");
		return 2;
	}
	static struct fdt_pdtp d;
	char err[256];
	if (fdt_pdtp_load(argv[1], &d, err, sizeof err)) {
		fprintf(stderr, "%s\n", err);
		return 1;
	}
	double J = argc > 2 ? atof(argv[2]) : 0.0;
	printf("{\"c_b\": %.17g, \"topics\": [", d.c_b);
	for (int l = 0; l < d.L; l++)
		printf("%s{\"name\": \"%s\", \"c_m\": %.17g, \"c_k\": %.17g, \"d\": %u}", l ? ", " : "", d.topic[l].name,
		       d.topic[l].c_m, d.c_k, fdt_p_demand(&d, l, d.topic[l].has_jitter ? d.topic[l].jitter : J));
	printf("]}\n");
	return 0;
}
