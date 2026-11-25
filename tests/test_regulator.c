/**
 * @file test_regulator.c
 * @brief The bandwidth regulators of Section III and slide 8.
 */
#include <fleet_dt/regulator.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>

/** The DT tick rate: 8 Hz, T = 125 ms. */
#define DT_HZ 8.0

/**
 * Runs @p ticks DT ticks of one sensor: offers the samples each tick holds,
 * then captures at the edge. Returns the samples the control path read.
 */
static unsigned long drive(fdt_reg_t *r, double sensor_hz, unsigned ticks,
                           unsigned *published_ticks)
{
    unsigned long reads = 0;
    *published_ticks = 0;
    for (unsigned k = 0; k < ticks; k++) {
        const unsigned m = fdt_reg_samples_in_tick(sensor_hz, DT_HZ, k);
        for (unsigned j = 0; j < m; j++) {
            reads++; /* the control loop reads every sample */
            fdt_reg_offer(r);
        }
        *published_ticks += (unsigned)fdt_reg_capture(r);
    }
    return reads;
}

static void test_init_guards(void)
{
    fdt_reg_t r;

    assert(fdt_reg_init(&r, 100.0) == 0);
    assert(fdt_reg_init(&r, 0.0) == -1);
    assert(fdt_reg_init(&r, -1.0) == -1);
    assert(fdt_reg_init(NULL, 100.0) == -1);

    assert(fdt_reg_init(&r, 100.0) == 0);
    assert(fdt_reg_sampled(&r) == 0);
    assert(fdt_reg_effective_hz(&r) == 0.0);
    assert(fdt_reg_saved_ratio(&r) == 0.0);
    assert(fdt_reg_capture(NULL) == 0);
    fdt_reg_offer(NULL);
}

/** Samples per tick of the slide 8 sensors: 125, 6.25 and 1.25. */
static void test_samples_in_tick(void)
{
    unsigned imu = 0, baro = 0, gps = 0;
    for (unsigned k = 0; k < 8; k++) {
        assert(fdt_reg_samples_in_tick(1000.0, DT_HZ, k) == 125);
        imu  += fdt_reg_samples_in_tick(1000.0, DT_HZ, k);
        baro += fdt_reg_samples_in_tick(50.0, DT_HZ, k);
        gps  += fdt_reg_samples_in_tick(10.0, DT_HZ, k);
    }
    assert(imu == 1000 && baro == 50 && gps == 10);

    /* The 10 Hz GPS: 1, 1, 1, 2 per tick. */
    assert(fdt_reg_samples_in_tick(10.0, DT_HZ, 0) == 1);
    assert(fdt_reg_samples_in_tick(10.0, DT_HZ, 3) == 2);
    assert(fdt_reg_samples_in_tick(0.0, DT_HZ, 0) == 0);
    assert(fdt_reg_samples_in_tick(10.0, 0.0, 0) == 0);
}

/** Slide 8: the IMU at 1 kHz reaches the link once per tick, 125x less. */
static void test_one_sample_per_tick(void)
{
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 1000.0) == 0);

    unsigned pub_ticks = 0;
    const unsigned long reads = drive(&r, 1000.0, 80, &pub_ticks);

    assert(reads == 10000);
    assert(pub_ticks == 80);
    assert(fdt_reg_published(&r) == 80);
    assert(fdt_reg_dropped(&r) == 10000 - 80);
    assert(fabs(fdt_reg_effective_hz(&r) - DT_HZ) < 1e-9);
    assert(fdt_reg_sampled(&r) / fdt_reg_published(&r) == 125);

    printf("1 kHz IMU -> %.2f Hz on the link, %.1f%% of its samples kept off\n",
           fdt_reg_effective_hz(&r), 100.0 * fdt_reg_saved_ratio(&r));
}

/** The sample released at the edge is the newest one, not the first. */
static void test_newest_sample_wins(void)
{
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 50.0) == 0);

    int value = 0;
    int held_value = -1;
    for (int j = 0; j < 6; j++) {
        value = j;
        fdt_reg_offer(&r);
        held_value = value; /* the caller overwrites its single slot */
    }
    assert(fdt_reg_capture(&r) == 1);
    assert(held_value == 5);
    assert(fdt_reg_dropped(&r) == 5);

    /* Nothing new: the next edge publishes nothing and repeats nothing. */
    assert(fdt_reg_capture(&r) == 0);
    assert(fdt_reg_empty_ticks(&r) == 1);
    assert(fdt_reg_published(&r) == 1);
}

/** A sensor slower than the tick is never decimated. */
static void test_no_false_decimation(void)
{
    fdt_reg_t slow;
    assert(fdt_reg_init(&slow, 1.0) == 0);
    unsigned pub_ticks = 0;
    (void)drive(&slow, 1.0, 80, &pub_ticks);
    assert(fdt_reg_sampled(&slow) == 10);
    assert(fdt_reg_published(&slow) == 10);
    assert(fdt_reg_dropped(&slow) == 0);
    assert(fdt_reg_empty_ticks(&slow) == 70);
    assert(fabs(fdt_reg_effective_hz(&slow) - 1.0) < 1e-9);

    /* Matched rates pass everything. */
    fdt_reg_t same;
    assert(fdt_reg_init(&same, DT_HZ) == 0);
    (void)drive(&same, DT_HZ, 40, &pub_ticks);
    assert(pub_ticks == 40);
    assert(fdt_reg_dropped(&same) == 0);

    /* The 10 Hz GPS keeps one sample per tick and drops the 2nd of every 4th. */
    fdt_reg_t gps;
    assert(fdt_reg_init(&gps, 10.0) == 0);
    (void)drive(&gps, 10.0, 80, &pub_ticks);
    assert(pub_ticks == 80);
    assert(fdt_reg_dropped(&gps) == 20);
}

/** C12: the regulator governs publication, not sampling. */
static void test_control_path_untouched(void)
{
    fdt_reg_t r;
    assert(fdt_reg_init(&r, 200.0) == 0);

    unsigned pub_ticks = 0;
    const unsigned long reads = drive(&r, 200.0, 16, &pub_ticks);

    assert(reads == 400);
    assert(fdt_reg_sampled(&r) == 400);
    assert(fdt_reg_sampled(&r) ==
           fdt_reg_published(&r) + fdt_reg_dropped(&r));
    assert(fdt_reg_published(&r) == 16);
    assert(reads > fdt_reg_published(&r) * 10);
}

int main(void)
{
    test_init_guards();
    test_samples_in_tick();
    test_one_sample_per_tick();
    test_newest_sample_wins();
    test_no_false_decimation();
    test_control_path_untouched();

    printf("test_regulator: ok\n");
    return 0;
}
