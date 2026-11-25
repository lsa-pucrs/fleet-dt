/**
 * @file main.c
 * @brief wbtgen: writes worlds/jundia_n<NN>.wbt for each vessel count given.
 *
 *     tools/worldgen/wbtgen 5 10 20 30 40 50 60
 *
 * Run from the repository root; `make webots-worlds` does.
 */
#include "worldgen/worldgen.h"

#include <stdio.h>
#include <stdlib.h>

#define WORLD_DIR "adapters/webots/worlds/"
#define TEMPLATE  WORLD_DIR "jundia_fleet.wbt"

static char g_tmpl[64 * 1024];
static char g_out[1024 * 1024];

int main(int argc, char **argv)
{
    FILE *f = fopen(TEMPLATE, "r");
    if (f == NULL) {
        fprintf(stderr, "wbtgen: cannot open %s; run from the repository "
                        "root\n", TEMPLATE);
        return EXIT_FAILURE;
    }
    const size_t n = fread(g_tmpl, 1, sizeof g_tmpl - 1, f);
    fclose(f);
    g_tmpl[n] = '\0';

    for (int i = 1; i < argc; i++) {
        const long vessels = strtol(argv[i], NULL, 10);
        const long len = (vessels > 0)
            ? fdt_wbtgen(g_tmpl, (unsigned)vessels, g_out, sizeof g_out)
            : -1;
        if (len < 0) {
            fprintf(stderr, "wbtgen: cannot build a world with %s vessels\n",
                    argv[i]);
            return EXIT_FAILURE;
        }

        char path[256];
        snprintf(path, sizeof path, WORLD_DIR "jundia_n%02ld.wbt", vessels);
        FILE *o = fopen(path, "w");
        if (o == NULL || fwrite(g_out, 1, (size_t)len, o) != (size_t)len) {
            fprintf(stderr, "wbtgen: cannot write %s\n", path);
            if (o != NULL) {
                fclose(o);
            }
            return EXIT_FAILURE;
        }
        fclose(o);
        printf("wrote %s (%ld vessels)\n", path, vessels);
    }
    return EXIT_SUCCESS;
}
