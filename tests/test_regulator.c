/**
 * @file test_regulator.c
 * @brief The bandwidth regulators of Section III.
 */
#include <fleet_dt/regulator.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_init_guards(void)
{
    fdt_reg_t r;

    assert(fdt_reg_init(&r, 100.0, 8.0) == 0);
    assert(fdt_reg_init(&r, 0.0, 8.0) == -1);
    assert(fdt_reg_init(&r, 100.0, 0.0) == -1);
    assert(fdt_reg_init(&r, -1.0, 8.0) == -1);
    assert(fdt_reg_init(NULL, 100.0, 8.0) == -1);

    assert(fdt_reg_init(&r, 100.0, 8.0) == 0);
    assert(fdt_reg_sampled(&r) == 0);
    assert(fdt_reg_effective_hz(&r) == 0.0);
    assert(fdt_reg_saved_ratio(&r) == 0.0);
    assert(fdt_reg_admit(NULL) == 0);
}

/**
 * The case Section III names: a sensor faster than the DT.
 */
static void test_decimation(void)
{
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 100.0, 8.0) == 0);

    for (int i = 0; i < 1000; i++) {
        (void)fdt_reg_admit(&r);
    }

    assert(fdt_reg_sampled(&r) == 1000);
    assert(fdt_reg_published(&r) >= 79 && fdt_reg_published(&r) <= 81);
    assert(fdt_reg_published(&r) + fdt_reg_dropped(&r) == 1000);

    assert(fabs(fdt_reg_effective_hz(&r) - 8.0) < 0.2);
    assert(fdt_reg_saved_ratio(&r) > 0.90 && fdt_reg_saved_ratio(&r) < 0.93);

    printf("100 Hz sensor -> %.2f Hz published, %.1f%% of its traffic saved\n",
           fdt_reg_effective_hz(&r), 100.0 * fdt_reg_saved_ratio(&r));
}

/** A sensor at or below the DT rate is never decimated. */
static void test_no_false_decimation(void)
{
    /* The paper's own example of a slow sensor: a gyroscope at 1 Hz. */
    fdt_reg_t slow;
    assert(fdt_reg_init(&slow, 1.0, 8.0) == 0);
    for (int i = 0; i < 50; i++) {
        assert(fdt_reg_admit(&slow) == 1);
    }
    assert(fdt_reg_dropped(&slow) == 0);
    assert(fdt_reg_saved_ratio(&slow) == 0.0);
    assert(fabs(fdt_reg_effective_hz(&slow) - 1.0) < 1e-9);

    /* Matched rates pass everything: no sample is spent on rounding. */
    fdt_reg_t same;
    assert(fdt_reg_init(&same, 8.0, 8.0) == 0);
    for (int i = 0; i < 40; i++) {
        assert(fdt_reg_admit(&same) == 1);
    }
    assert(fdt_reg_dropped(&same) == 0);
}

/** Decimation is regular, not bursty: the gaps between publications are even. */
static void test_even_spacing(void)
{
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 40.0, 8.0) == 0); /* exactly one in five */

    int gap = 0;
    int worst_gap = 0;
    int publications = 0;

    for (int i = 0; i < 200; i++) {
        if (fdt_reg_admit(&r) == 1) {
            publications++;
            if (gap > worst_gap) {
                worst_gap = gap;
            }
            gap = 0;
        } else {
            gap++;
        }
    }

    assert(publications == 40);
    assert(worst_gap == 4);
}

/**
 * C12: the regulator governs publication, not sampling.
 */
static void test_control_path_untouched(void)
{
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 200.0, 8.0) == 0);

    int control_loop_reads = 0;
    for (int i = 0; i < 400; i++) {
        /* The control loop reads every sample, whatever the regulator says. */
        control_loop_reads++;
        (void)fdt_reg_admit(&r);
    }

    assert(control_loop_reads == 400);
    assert(fdt_reg_sampled(&r) == 400);
    assert(fdt_reg_sampled(&r) ==
           fdt_reg_published(&r) + fdt_reg_dropped(&r));
    assert(fdt_reg_published(&r) < 20);   /* the link saw almost none */
    assert(control_loop_reads > (int)fdt_reg_published(&r) * 10);
}

int main(void)
{
    test_init_guards();
    test_decimation();
    test_no_false_decimation();
    test_even_spacing();
    test_control_path_untouched();

    printf("test_regulator: ok\n");
    return 0;
}
