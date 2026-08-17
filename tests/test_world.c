/**
 * @file test_world.c
 * @brief Checks the WeBots world against what the controller assumes of it.
 *
 * The controller cannot be linked on a machine without the WeBots SDK, so its
 * assumptions about the world would otherwise go unchecked until someone with
 * the SDK opened the simulation and found a DEF renamed or a mesh path stale.
 *
 * The world file is text, and every assumption the controller makes about it
 * is textual: a DEF name it resolves, a mesh URL it renders through, a time
 * step it divides. Those are checkable here, in the ordinary suite, on any
 * machine. This does not prove the simulation runs; it proves the controller
 * and the world still agree about what they are.
 *
 * Claims touched: C3 and C8, which remain boundaries — the artefacts are
 * complete and mutually consistent, but neither has been through the real
 * WeBots toolchain. See docs/spec/paper-claims.md.
 */
#include <fleet_dt/tick.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
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
 *
 * They exist so the CPU claim of Section V-A can be measured as a difference:
 * the empty world is the renderer's own cost, and each vessel after that is
 * an increment. tools/bench/bench_webots_cpu.c runs all three.
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
 *
 * The source world this one derives from left basicTimeStep at the 32 ms
 * default, and 125/32 is not an integer: wb_robot_step() would have rounded
 * the DT frame to something other than the deadline the model is defined by,
 * quietly. This is the check that keeps that from coming back.
 */
static void test_time_step_divides_the_frame(void)
{
    /* Search from WorldInfo, not from the top: the header comment explains
     * the setting by name, and a plain search finds the prose first. */
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

        /* Present exactly once below the vessel count, absent above it. Two
         * nodes sharing a DEF would make the lookup return whichever the
         * parser reached first, silently; a gap in the middle would truncate
         * the fleet, because the controller stops at the first miss. */
        assert(count_of(def) == (k < g_expect ? 1 : 0));
    }
    assert(count_of("Robot {") == g_expect);
}

/**
 * One supervisor, not one per hull.
 *
 * Figure 4 has a single coordinator computing c^t across the fleet. A
 * controller per vessel could not compute it, because no instance would see
 * more than its own state.
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

    /* The material and its texture travel with the hull; an .obj whose .mtl
     * is missing renders untextured rather than failing, which is the kind of
     * breakage nobody notices in a screenshot. */
    assert(mesh_exists("../3d_models/drone_boat/full_model/boat_model.mtl"));
    assert(mesh_exists("../3d_models/drone_boat/full_model/boat_uv.png"));
}

/**
 * The pieces of Section III that have to be in the world rather than in code.
 */
static void test_section_iii_furniture(void)
{
    /* The fluid simulator Section III lists as sharing the DTE. */
    assert(count_of("Fluid {") == 1);
    assert(count_of("fluidName \"fluid\"") >= g_expect);

    /* Drag has to be configured per hull, or the boats slide frictionlessly
     * across a fluid that is only decorative. */
    assert(count_of("dragForceCoefficients") == g_expect);

    /* One stereo view per vessel. The image never travels over MQTT; the
     * codec carries a presence flag and RTSP carries the frame. */
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

        printf("%-20s %d vessel(s), consistent with the controller\n",
               WORLDS[i].file, WORLDS[i].vessels);
    }

    printf("test_world: ok\n");
    return 0;
}
