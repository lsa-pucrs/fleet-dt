/**
 * @file test_relay.c
 * @brief The regulator of the boat broker (Section III; talk, slides 7, 12
 *        and 13), and the regulated stage it runs per vessel.
 */
#include <fleet_dt/assembler.h>
#include <fleet_dt/codec.h>
#include <fleet_dt/envelope.h>
#include <fleet_dt/regpub.h>
#include <fleet_dt/relay.h>
#include <fleet_dt/transport.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>

static fdt_loop_t loop;

/** Rates of slide 13, indexed by fdt_sensor_t: IMU, GPS, baro, power. */
static const double HZ[FDT_SENSOR_COUNT] = { 1000.0, 10.0, 50.0, 10.0 };

/** The twin on the ground side: counts and assembles regulated messages. */
typedef struct {
    fdt_rx_t      rx;
    fdt_rx_slot_t slots[2];
    unsigned      msgs;
} twin_t;

static void on_lsdt(const char *topic, const uint8_t *buf, size_t len,
                    void *user)
{
    (void)topic;
    twin_t *t = (twin_t *)user;
    t->msgs++;
    (void)fdt_rx_accept(&t->rx, buf, len);
}

/** Offers the samples sensor @p s takes in tick @p k, value = sample index. */
static void autopilot_tick(fdt_regpub_t *ap, unsigned k)
{
    for (int s = 0; s < FDT_SENSOR_COUNT; s++) {
        const unsigned m = fdt_reg_samples_in_tick(HZ[s], 8.0, k);
        for (unsigned j = 1; j <= m; j++) {
            fdt_input_t in = {0};
            in.wz_rps      = (float)(k * 1000u + j);
            in.gps_lat_deg = (float)(k * 1000u + j);
            in.press_pa    = (float)(k * 1000u + j);
            in.vbat_v      = (float)(k * 1000u + j);
            assert(fdt_regpub_offer(ap, (fdt_sensor_t)s, &in) == 1);
        }
    }
}

/**
 * Autopilot -> boat broker (raw, every sample) -> regulator in the broker
 * -> lsdt, one message per sensor per tick -> twin.
 */
static void test_regulator_in_the_boat_broker(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop);

    static twin_t twin;
    assert(fdt_rx_init(&twin.rx, twin.slots, 2, NULL) == 0);
    assert(tr.subscribe(tr.self, FDT_TOPIC_LSDT_ALL, on_lsdt, &twin) == 0);

    static fdt_regpub_t stages[2];
    fdt_relay_t relay;
    assert(fdt_relay_init(&relay, &tr, stages, 2, HZ) == 0);

    /* The autopilot of boat 1 publishes raw on fleet/1/raw/<sensor>. */
    fdt_regpub_t ap;
    assert(fdt_regpub_init(&ap, &tr, 1, HZ, 1) == 0);
    assert(fdt_regpub_set_level(&ap, FDT_TOPIC_RAW) == 0);

    for (unsigned k = 0; k < 8; k++) {
        autopilot_tick(&ap, k);
        assert(tr.poll(tr.self, 0) > 125); /* raw samples reach the relay */
        const int out = fdt_relay_tick(&relay);
        assert(out == FDT_SENSOR_COUNT); /* boat 0 is silent */
        (void)tr.poll(tr.self, 0);
    }

    /* One second: 1070 samples reached the broker, 32 left it. */
    assert(fdt_relay_received(&relay) == 1070);
    assert(fdt_relay_rejected(&relay) == 0);
    assert(fdt_regpub_messages(&ap) == 1070);
    assert(twin.msgs == 32);
    assert(fdt_regpub_messages(&stages[1]) == 32);
    assert(fdt_regpub_messages(&stages[0]) == 0);

    /* The twin holds the newest sample of each sensor of tick 7. */
    const fdt_input_t *got = fdt_rx_input(&twin.rx, 1);
    assert(got->wz_rps == 7125.0f);     /* IMU: 125th sample of tick 7 */
    assert(got->gps_lat_deg == 7002.0f); /* GPS: 2 samples in tick 7 */
    assert(got->press_pa == 7007.0f);   /* baro: 7 samples in tick 7 */
    assert(got->vbat_v == 7002.0f);

    printf("boat broker: 1070 raw samples in, %u regulated messages out\n",
           twin.msgs);

    /* Foreign traffic on the raw level is refused, not forwarded. */
    uint8_t junk[3] = { 1, 2, 3 };
    assert(fdt_relay_offer(&relay, junk, sizeof junk) == -1);
    assert(fdt_relay_rejected(&relay) == 1);
    tr.close(tr.self);
}

static void test_guards(void)
{
    fdt_transport_t tr = fdt_loop_transport(&loop);
    fdt_regpub_t rp;
    fdt_relay_t rl;
    static fdt_regpub_t stages[1];
    const double bad[FDT_SENSOR_COUNT] = { 1000.0, 0.0, 50.0, 10.0 };
    fdt_input_t in = {0};

    assert(fdt_regpub_init(NULL, &tr, 0, HZ, 0) == -1);
    assert(fdt_regpub_init(&rp, NULL, 0, HZ, 0) == -1);
    assert(fdt_regpub_init(&rp, &tr, 0, NULL, 0) == -1);
    assert(fdt_regpub_init(&rp, &tr, 0x10000u, HZ, 0) == -1);
    assert(fdt_regpub_init(&rp, &tr, 0, bad, 0) == -1);
    assert(fdt_regpub_init(&rp, &tr, 0, HZ, 0) == 0);
    assert(fdt_regpub_offer(&rp, FDT_SENSOR_COUNT, &in) == -1);
    assert(fdt_regpub_offer(&rp, FDT_SENSOR_IMU, NULL) == -1);
    assert(fdt_regpub_set_level(&rp, NULL) == -1);
    assert(fdt_regpub_tick(NULL) == -1);
    assert(fdt_regpub_reg(&rp, FDT_SENSOR_COUNT) == NULL);

    /* Nothing offered: the tick publishes nothing. */
    assert(fdt_regpub_tick(&rp) == 0);

    /* Raw mode counts a refused publish and reports it. */
    assert(fdt_regpub_init(&rp, &tr, 0, HZ, 1) == 0);
    for (size_t i = 0; i < FDT_LOOP_QUEUE; i++) {
        assert(fdt_regpub_offer(&rp, FDT_SENSOR_BARO, &in) == 1);
    }
    assert(fdt_regpub_offer(&rp, FDT_SENSOR_BARO, &in) == -1);
    assert(fdt_regpub_refused(&rp) == 1);
    assert(fdt_regpub_messages(&rp) == FDT_LOOP_QUEUE);
    assert(fdt_regpub_bytes(&rp) ==
           FDT_LOOP_QUEUE * (FDT_ENV_HEADER_BYTES + FDT_WIRE_BARO_BYTES));

    assert(fdt_relay_init(NULL, &tr, stages, 1, HZ) == -1);
    assert(fdt_relay_init(&rl, &tr, stages, 0, HZ) == -1);
    assert(fdt_relay_init(&rl, &tr, NULL, 1, HZ) == -1);
    assert(fdt_relay_init(&rl, &tr, stages, 1, bad) == -1);
    assert(fdt_relay_tick(NULL) == -1);
    assert(fdt_relay_offer(NULL, NULL, 0) == -1);
    tr.close(tr.self);
}

int main(void)
{
    test_regulator_in_the_boat_broker();
    test_guards();

    printf("test_relay: ok\n");
    return 0;
}
