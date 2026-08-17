/**
 * @file test_world.c
 * @brief Checks the WeBots world against what the controller assumes of it.
 */
#include <fleet_dt/tick.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define WORLD_DIR "adapters/webots/worlds/"

/**
 * The DEF names the controller resolves, in the order it tries them. It stops
 * at the first miss, so a world holding a prefix of this list yields exactly
 * that many vessels.
 */
static const char *const VESSEL_DEF[] = { "PINTADO", "TILAPIA" };

/**
 * The three worlds, differing only in vessel count.
 */
typedef struct {
    const char *file;
    int         vessels;
} world_t;

static const world_t WORLDS[] = {
    { "jundia_empty.wbt",  0 },
    { "jundia_single.wbt", 1 },
    { "jundia_fleet.wbt",  2 },
};
#define NWORLDS ((int)(sizeof WORLDS / sizeof WORLDS[0]))

/** The world under test, and how many vessels it should hold. */
static int  g_expect;
static char g_world[64 * 1024];

/** Loads one world into g_world, NUL-terminated. */
static void load_world(const world_t *w)
{
    char path[512];
    snprintf(path, sizeof path, "%s%s", WORLD_DIR, w->file);
    g_expect = w->vessels;

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s -- run make from the repository "
                        "root\n", path);
        exit(EXIT_FAILURE);
    }
    const size_t n = fread(g_world, 1, sizeof g_world - 1, f);
    g_world[n] = '\0';
    fclose(f);
    assert(n > 0);
}

/** Occurrences of @p needle in the world. */
static int count_of(const char *needle)
{
    int n = 0;
    const char *p = g_world;
    while ((p = strstr(p, needle)) != NULL) {
        n++;
        p += strlen(needle);
    }
    return n;
}

/** Whether a path relative to the world directory exists on disk. */
static int mesh_exists(const char *relative)
{
    char path[512];
    snprintf(path, sizeof path, "%s%s", WORLD_DIR, relative);

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return 0;
    }
    fclose(f);
    return 1;
}

/**
 * The frame of Section IV has to be a whole number of physics steps.
 */
static void test_time_step_divides_the_frame(void)
{
    const char *info = strstr(g_world, "WorldInfo {");
    assert(info != NULL);

    const char *p = strstr(info, "basicTimeStep");
    assert(p != NULL);

    const long basic_ms = strtol(p + strlen("basicTimeStep"), NULL, 10);
    assert(basic_ms > 0);

    const long frame_ms = FDT_TICK_NS / 1000000L;
    assert(frame_ms == 125);
    assert(frame_ms % basic_ms == 0);

    (void)0; /* the ratio is asserted above; printing it once per world is noise */
}

/** Every vessel the controller resolves has to exist, exactly once. */
static void test_vessel_defs(void)
{
    for (int k = 0; k < (int)(sizeof VESSEL_DEF / sizeof VESSEL_DEF[0]); k++) {
        char def[64];
        snprintf(def, sizeof def, "DEF %s Robot", VESSEL_DEF[k]);

        assert(count_of(def) == (k < g_expect ? 1 : 0));
    }
    assert(count_of("Robot {") == g_expect);
}

/**
 * One supervisor, not one per hull.
 */
static void test_single_coordinator(void)
{
    /* A world with no vessels has no coordinator to be single. */
    const int coordinators = (g_expect > 0) ? 1 : 0;

    assert(count_of("supervisor TRUE") == coordinators);
    assert(count_of("controller \"fdt_controller\"") == coordinators);

    /* Every vessel after the first is driven, not autonomous. */
    assert(count_of("controller \"<none>\"") ==
           (g_expect > 0 ? g_expect - 1 : 0));
}

/** Mesh URLs have to resolve, or the hulls render as nothing. */
static void test_meshes_resolve(void)
{
    static const char *const MESHES[] = {
        "../3d_models/drone_boat/full_model/boat_model.obj",
        "../3d_models/drone_boat/bounding_box/boat_bounding_box.obj",
    };
    const int n = (int)(sizeof MESHES / sizeof MESHES[0]);

    for (int i = 0; i < n; i++) {
        /* Referenced once per vessel... */
        assert(count_of(MESHES[i]) == g_expect);
        /* ...and present on disk regardless, since another world needs it. */
        assert(mesh_exists(MESHES[i]));
    }

    assert(mesh_exists("../3d_models/drone_boat/full_model/boat_model.mtl"));
    assert(mesh_exists("../3d_models/drone_boat/full_model/boat_uv.png"));
}

/**
 * The world is ENU, and the controller has to agree with it.
 */
static void test_enu_convention(void)
{
    if (g_expect == 0) {
        return;
    }

    const char *p = strstr(g_world, "DEF PINTADO Robot");
    assert(p != NULL);

    p = strstr(p, "translation");
    assert(p != NULL);

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    assert(sscanf(p + strlen("translation"), "%lf %lf %lf",
                  &east, &north, &up) == 3);

    assert(fabs(up) < 0.35);

    assert(fabs(north) > 1.0);
    assert(fabs(up) < fabs(north));

    /* Yaw turns about up, which is the third axis. */
    const char *rot = strstr(p, "rotation");
    assert(rot != NULL);

    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    assert(sscanf(rot + strlen("rotation"), "%lf %lf %lf",
                  &ax, &ay, &az) == 3);
    assert(fabs(ax) < 1e-9 && fabs(ay) < 1e-9);
    assert(fabs(az - 1.0) < 1e-9);
}

/**
 * The pieces of Section III that have to be in the world rather than in code.
 */
static void test_section_iii_furniture(void)
{
    /* The fluid simulator Section III lists as sharing the DTE. */
    assert(count_of("Fluid {") == 1);
    assert(count_of("fluidName \"fluid\"") >= g_expect);

    assert(count_of("dragForceCoefficients") == g_expect);

    assert(count_of("Camera {") == g_expect);
}

int main(void)
{
    for (int i = 0; i < NWORLDS; i++) {
        load_world(&WORLDS[i]);

        test_time_step_divides_the_frame();
        test_vessel_defs();
        test_single_coordinator();
        test_meshes_resolve();
        test_section_iii_furniture();
        test_enu_convention();

        printf("%-20s %d vessel(s), consistent with the controller\n",
               WORLDS[i].file, WORLDS[i].vessels);
    }

    printf("test_world: ok\n");
    return 0;
}
