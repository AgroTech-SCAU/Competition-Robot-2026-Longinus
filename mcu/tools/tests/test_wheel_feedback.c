/* Run from the repository root with tools/run_host_tests.ps1.
 * Standalone build links this file, mcu/src/service/chassis.c, and Standard's
 * sdks/domain/steer_wheel_kine.c plus sdks/infra/log.c; no hardware is used.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../../mcu/src/service/chassis.h"

static uint32_t test_now, test_stamp, test_sequence;
static float test_vx;
static float test_steer_feedback[4], test_steer_position[4], test_drive_command[4];
static double test_drive_position[4];
static float test_drive_velocity_offset[4];
static bool test_enabled, test_bad_feedback, test_tx_fail, test_prepare_busy;
static bool test_position_failure;
static unsigned test_writes, test_steer_writes;
static unsigned test_position_reads;

static uint32_t clock_mock(void) { return test_now; }
static bool timing(uint32_t* stamp, uint32_t* sequence) {
    *stamp = test_stamp;
    *sequence = test_sequence;
    return true;
}
static bool healthy(void) { return (uint32_t)(test_now - test_stamp) < 100u; }
static bool valid(void) { return healthy() && test_enabled; }
static bool drive_positions(double positions[4]) {
    ++test_position_reads;
    if(test_position_failure) return false;
    for(unsigned i = 0; i < 4; ++i) positions[i] = test_drive_position[i];
    return true;
}
static BusMotorStatus okay(void) { return MOTOR_STATUS_OK; }
static BusMotorStatus update(void) {
    return test_tx_fail ? MOTOR_STATUS_PORT_ERROR : MOTOR_STATUS_OK;
}
static BusMotorStatus write_bank(void) { ++test_writes; return MOTOR_STATUS_OK; }
static BusMotorStatus prepare(BusMotorId id) {
    (void)id;
    return test_prepare_busy ? MOTOR_STATUS_BUSY : MOTOR_STATUS_OK;
}
static BusMotorStatus write_id(BusMotorId id) {
    (void)id;
    ++test_writes;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus steer(BusMotorId id, float position, float velocity) {
    (void)velocity;
    assert(id >= 1u && id <= 4u);
    test_steer_position[id - 1u] = position;
    ++test_steer_writes;
    ++test_writes;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus require(BusMotorId id, BusMotorProfile profile) {
    (void)id;
    (void)profile;
    return MOTOR_STATUS_OK;
}
static void clear(void) {}
static const char* status_name(BusMotorStatus status) { (void)status; return "mock"; }
static BusMotorStatus feedback(BusMotorId id, BusMotorFeedback* output) {
    const int signs[4] = { 1, -1, -1, 1 };
    *output = (BusMotorFeedback){
        .id = id,
        .valid = BUS_MOTOR_FEEDBACK_STATE | BUS_MOTOR_FEEDBACK_POSITION | BUS_MOTOR_FEEDBACK_VELOCITY,
        .state = test_enabled ? BUS_MOTOR_STATE_ENABLED : BUS_MOTOR_STATE_DISABLED,
    };
    if(id >= 5u) {
        output->velocity = test_vx / 0.035276f * signs[id - 5u] + test_drive_velocity_offset[id - 5u];
        output->position = (float)test_drive_position[id - 5u];
    }
    else output->position = test_steer_feedback[id - 1u];
    if(test_bad_feedback && id == 1u) output->valid = 0u;
    return MOTOR_STATUS_OK;
}
static BusMotorStatus group(BusMotorGroupId id, const BusMotorCommand* commands,
                            uint8_t count, BusMotorGroupPolicy policy) {
    (void)id;
    (void)policy;
    assert(count == 4u);
    for(unsigned i = 0; i < count; ++i) test_drive_command[i] = commands[i].data.scalar;
    ++test_writes;
    return MOTOR_STATUS_OK;
}
const BusMotorInterface bus_motor = {
    .status_str = status_name,
    .basic = { .disable = write_id, .brake = write_id },
    .profile = { .require = require },
    .feedback = { .all = feedback },
    .group = { .cmd = group },
};

static void sample(float vx, uint32_t timestamp) {
    test_vx = vx;
    test_now = test_stamp = timestamp;
    ++test_sequence;
}
static void near(float actual, float expected) {
    if(!(fabsf(actual - expected) < 1e-5f))
        fprintf(stderr, "near failed at %u ms: actual=%.9g expected=%.9g\n", test_now, actual, expected);
    assert(fabsf(actual - expected) < 1e-5f);
}

static void expect_zero_targets(const float expected[4]) {
    for(unsigned i = 0; i < 4; ++i) {
        near(test_steer_position[i], expected[i]);
        near(test_drive_command[i], 0.0f);
        near(chassis.get_control()->wheels[i].wheel_omega, 0.0f);
    }
}

static void move_steer_feedback(float delta, float expected[4]) {
    for(unsigned i = 0; i < 4; ++i) {
        test_steer_feedback[i] += delta;
        expected[i] = test_steer_feedback[i];
    }
}

static void test_zero_steer_hold(const ChassisConfig* config) {
    ChassisObservation observation;
    float held[4];
    test_bad_feedback = test_tx_fail = false;
    assert(chassis.init(config) == CHASSIS_OK);
    test_enabled = true;
    test_prepare_busy = true;
    test_steer_feedback[0] = -12.1f;
    test_steer_feedback[1] = 12.0f;
    test_steer_feedback[2] = -6.8f;
    test_steer_feedback[3] = 7.1f;
    sample(0.0f, 200u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    const unsigned before_prepare = test_steer_writes;
    assert(chassis.set_velocity(0.0f, 0.0f, 0.0f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    assert(test_steer_writes == before_prepare && !chassis.is_ready());

    /* Preparation must not capture an old angle or emit ordinary targets.
       First zero holds current absolute positions, including multiple turns. */
    move_steer_feedback(0.1f, held);
    sample(0.0f, 202u);
    assert(chassis.process() == CHASSIS_OK && test_steer_writes == before_prepare);
    test_prepare_busy = false;
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK && chassis.is_ready());
    expect_zero_targets(held);

    /* Feedback changes independently of the latched command; repeated zero
       requests cannot ratchet the target, and FK must still see real motion. */
    for(unsigned step = 0; step < 3; ++step) {
        for(unsigned i = 0; i < 4; ++i) test_steer_feedback[i] += 0.02f;
        sample(0.1f, 204u + 2u * step);
        assert(chassis.set_velocity(0.0f, 0.0f, 0.0f) == CHASSIS_OK);
        assert(chassis.process() == CHASSIS_OK);
        expect_zero_targets(held);
        assert(chassis_get_observation(&observation));
        float expected_vx = 0.0f, expected_vy = 0.0f;
        for(unsigned i = 0; i < 4; ++i) {
            expected_vx += 0.025f * cosf(-test_steer_feedback[i]);
            expected_vy += 0.025f * sinf(-test_steer_feedback[i]);
        }
        near(observation.raw_velocity.vx, expected_vx);
        near(observation.raw_velocity.vy, expected_vy);
    }

    /* Motion releases the hold; stopping captures the latest angle rather
       than retaining either the old hold or an unfinished movement target. */
    assert(chassis.set_velocity(0.0f, 0.0f, 0.3f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    assert(fabsf(test_steer_position[0] - held[0]) > 0.1f);
    move_steer_feedback(0.11f, held);
    sample(0.0f, 212u);
    assert(chassis.set_velocity(0.0f, 0.0f, 0.0f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);

    /* X parking owns its own target, then release to ordinary zero captures
       the current position instead of reviving the pre-parking hold. */
    assert(chassis.brake() == CHASSIS_OK && chassis.process() == CHASSIS_OK);
    assert(fabsf(test_steer_position[0] - held[0]) > 0.1f);
    for(unsigned i = 0; i < 4; ++i) test_steer_feedback[i] = test_steer_position[i];
    sample(0.0f, 214u);
    assert(chassis.process() == CHASSIS_OK);
    assert(chassis.get_chassis()->brake_latched);
    move_steer_feedback(0.05f, held);
    sample(0.0f, 216u);
    assert(chassis.set_velocity(0.0f, 0.0f, 0.0f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);

    assert(chassis.stop() == CHASSIS_OK);
    move_steer_feedback(0.1f, held);
    sample(0.0f, 218u);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);

    /* Disable/restart, invalid feedback, and send-failure recovery all discard
       a prior hold; observation validity remains independent of the target. */
    assert(chassis_set_output_enabled(false) == CHASSIS_OK);
    move_steer_feedback(0.1f, held);
    sample(0.0f, 220u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);
    test_bad_feedback = true;
    const unsigned before_invalid = test_steer_writes;
    sample(0.0f, 222u);
    assert(chassis.process() == CHASSIS_MOTOR_COMMAND_FAILED);
    assert(test_steer_writes == before_invalid && !chassis_get_observation(&observation));
    test_bad_feedback = false;
    move_steer_feedback(0.1f, held);
    sample(0.0f, 224u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);
    test_tx_fail = true;
    assert(chassis.process() == CHASSIS_MOTOR_COMMAND_FAILED);
    test_tx_fail = false;
    move_steer_feedback(0.1f, held);
    sample(0.0f, 226u);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);

    assert(chassis.init(config) == CHASSIS_OK);
    move_steer_feedback(-0.2f, held);
    sample(0.0f, 228u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    expect_zero_targets(held);
}

static void zero_cycle(uint32_t timestamp) {
    sample(0.0f, timestamp);
    assert(chassis.set_velocity(0.0f, 0.0f, 0.0f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
}

static void expect_drive_zero(void) {
    for(unsigned i = 0; i < 4; ++i) near(test_drive_command[i], 0.0f);
}

static void test_zero_drive_hold(const ChassisConfig* config) {
    const double initial[4] = { 1000.0, -2000.0, 3000.0, 100000000.0 };
    double held[4];
    test_enabled = true;
    test_bad_feedback = test_tx_fail = test_prepare_busy = test_position_failure = false;
    for(unsigned i = 0; i < 4; ++i) {
        test_steer_feedback[i] = 0.0f;
        test_drive_velocity_offset[i] = 0.0f;
        test_drive_position[i] = initial[i];
    }
    assert(chassis.init(config) == CHASSIS_OK);
    sample(0.0f, 1000u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    assert(chassis.set_velocity(0.2f, 0.0f, 0.0f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);

    /* The chassis first brakes with the existing zero-speed command; position
       gained during braking must never become a request to reverse. */
    sample(0.1f, 1002u);
    assert(chassis.set_velocity(0.0f, 0.0f, 0.0f) == CHASSIS_OK);
    assert(chassis.process() == CHASSIS_OK);
    expect_drive_zero();
    for(unsigned i = 0; i < 4; ++i) test_drive_position[i] += 0.5;
    sample(0.1f, 1502u);
    assert(chassis.process() == CHASSIS_OK);
    expect_drive_zero();
    zero_cycle(1504u);
    for(unsigned repeat = 0; repeat < 300; ++repeat) {
        test_drive_position[0] += 0.001;
        zero_cycle(1504u); /* Calls alone cannot satisfy the 200 ms condition. */
        expect_drive_zero();
    }
    test_drive_velocity_offset[3] = 0.02f; /* One wheel moving restarts the dwell. */
    zero_cycle(1690u);
    expect_drive_zero();
    test_drive_velocity_offset[3] = 0.0f;
    zero_cycle(1692u);
    for(unsigned i = 0; i < 4; ++i) test_drive_position[i] += 0.25;
    zero_cycle(1891u);
    expect_drive_zero();
    for(unsigned i = 0; i < 4; ++i) held[i] = test_drive_position[i] += 0.01;
    zero_cycle(1892u);
    expect_drive_zero();

    /* Error uses native motor signs, including the two reversed drive mounts.
       Repeated zero requests retain the same anchor; both limits are exercised. */
    test_drive_position[0] = held[0] + 0.025;
    test_drive_position[1] = held[1] - 0.025;
    test_drive_position[2] = held[2] + 0.2;
    test_drive_position[3] = held[3] - 0.2;
    for(unsigned repeat = 0; repeat < 3; ++repeat) {
        zero_cycle(1894u + repeat * 2u);
        near(test_drive_command[0], -0.05f);
        near(test_drive_command[1], 0.05f);
        near(test_drive_command[2], -0.10f);
        near(test_drive_command[3], 0.10f);
    }
    test_drive_position[3] = held[3] + 0.00001;
    zero_cycle(1900u);
    assert(test_drive_command[3] < -0.00001f && test_drive_command[3] > -0.00003f);
    ChassisObservation observation;
    assert(chassis_get_observation(&observation));
    near(observation.raw_velocity.vx, 0.0f); /* Report the actual RPM feedback. */
    near(observation.raw_velocity.vy, 0.0f);

    /* A motion request releases the anchor even if zero follows before process. */
    assert(chassis.set_velocity(0.2f, 0.0f, 0.0f) == CHASSIS_OK);
    zero_cycle(2000u);
    expect_drive_zero();
    zero_cycle(2200u);
    expect_drive_zero();

    /* X parking is unchanged: it neither reads nor applies position hold. */
    assert(chassis.brake() == CHASSIS_OK);
    test_position_failure = true;
    const unsigned before_brake = test_position_reads;
    sample(0.0f, 2202u);
    assert(chassis.process() == CHASSIS_OK && test_position_reads == before_brake);
    expect_drive_zero();
    test_position_failure = false;
    test_drive_position[0] += 0.2;
    zero_cycle(2204u);
    expect_drive_zero();
    zero_cycle(2404u);
    expect_drive_zero();
    test_drive_position[0] += 0.01;
    zero_cycle(2406u);
    near(test_drive_command[0], -0.02f);

    assert(chassis.stop() == CHASSIS_OK);
    test_drive_position[0] += 0.1;
    zero_cycle(2408u);
    expect_drive_zero();
    zero_cycle(2608u);
    expect_drive_zero();
    assert(chassis_set_output_enabled(false) == CHASSIS_OK);
    test_prepare_busy = true;
    sample(0.0f, 2610u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    const unsigned before_prepare = test_position_reads;
    zero_cycle(2610u);
    zero_cycle(3010u);
    assert(test_position_reads == before_prepare);
    test_prepare_busy = false;
    assert(chassis_set_output_enabled(true) == CHASSIS_OK);
    test_drive_position[0] += 1.0;
    zero_cycle(3012u);
    expect_drive_zero();
    zero_cycle(3212u);
    expect_drive_zero();

    /* Callback failure or a nonfinite position takes the existing stop path,
       with zero drive commands, and recovery must earn a fresh dwell. */
    test_drive_position[0] += 0.01;
    zero_cycle(3214u);
    near(test_drive_command[0], -0.02f);
    test_position_failure = true;
    assert(chassis.process() == CHASSIS_MOTOR_COMMAND_FAILED);
    expect_drive_zero();
    test_position_failure = false;
    zero_cycle(3216u);
    expect_drive_zero();
    const double saved = test_drive_position[2];
    test_drive_position[2] = NAN;
    sample(0.0f, 3218u);
    assert(chassis.process() == CHASSIS_MOTOR_COMMAND_FAILED);
    expect_drive_zero();
    test_drive_position[2] = INFINITY;
    sample(0.0f, 3220u);
    assert(chassis.process() == CHASSIS_MOTOR_COMMAND_FAILED);
    expect_drive_zero();
    test_drive_position[2] = saved;

    /* Wall-clock dwell remains correct across HAL's uint32 millisecond wrap. */
    assert(chassis.stop() == CHASSIS_OK);
    zero_cycle(UINT32_MAX - 100u);
    test_drive_position[0] += 0.1;
    zero_cycle(98u);
    expect_drive_zero();
    zero_cycle(99u);
    expect_drive_zero();
    test_drive_position[0] += 0.01;
    zero_cycle(101u);
    near(test_drive_command[0], -0.02f);
}

int main(void) {
    const ChassisConfig config = {
        .prepare_steer_motor = prepare, .prepare_drive_motor = prepare,
        .set_steer_target = steer, .motor_update = update, .motor_poll_feedback = okay,
        .cancel_pending_commands = okay, .disable_drive_motors = write_bank,
        .clear_motor_fault = clear, .drive_group = 1,
        .steer_ids = { 1, 2, 3, 4 }, .drive_ids = { 5, 6, 7, 8 },
        .drive_sign = { 1, -1, -1, 1 },
        .feedback_is_valid = valid, .feedback_is_healthy = healthy,
        .now_ms = clock_mock, .feedback_sample_time_ms = timing,
        .read_drive_positions = drive_positions,
        .model = { .length = 0.129f, .width = 0.129f, .wheel_radius = 0.035276f,
                   .max_wheel_linear_speed = 2.0f },
        .wheel_drive_ratio = 1.0f,
        .yaw_bias = { .enabled = true, .k_vx = -0.096f, .k_vy = 0.058f, .v_deadband = 0.01f },
    };
    ChassisObservation observation;
    assert(chassis.init(&config) == CHASSIS_OK);
    assert(!chassis_get_observation(&observation));

    /* A disabled chassis can be pushed: passive polling observes actual motion
       without sending any motor commands. The yaw correction remains active. */
    sample(0.4f, 10u);
    assert(chassis_poll_feedback() == CHASSIS_OK && test_writes == 0u);
    assert(chassis_get_observation(&observation));
    near(observation.raw_velocity.vx, 0.4f);
    near(observation.velocity.wz, -0.0384f);
    const uint32_t previous_sequence = observation.sample_sequence;
    assert(chassis_poll_feedback() == CHASSIS_OK && chassis_get_observation(&observation));
    assert(observation.sample_sequence == previous_sequence && test_writes == 0u);

    /* Stopping changes the command, while the next actual zero feedback updates
       the measured velocity; the last nonzero state must not remain frozen. */
    assert(chassis_set_output_enabled(false) == CHASSIS_OK);
    sample(0.0f, 20u);
    assert(chassis_poll_feedback() == CHASSIS_OK && chassis_get_observation(&observation));
    near(observation.velocity.vx, 0.0f);

    test_enabled = true;
    sample(0.2f, 30u);
    assert(chassis_set_output_enabled(true) == CHASSIS_OK && chassis_is_output_enabled());
    assert(chassis.set_velocity(0.5f, 0.0f, 0.0f) == CHASSIS_OK);
    near(chassis.get_control()->vx, 0.5f);
    assert(chassis.brake() == CHASSIS_OK && chassis.process() == CHASSIS_OK);
    assert(chassis_get_observation(&observation));
    near(observation.raw_velocity.vx, 0.2f); /* Braking must not fabricate zero speed. */

    test_tx_fail = true;
    sample(0.3f, 40u);
    assert(chassis.process() == CHASSIS_MOTOR_COMMAND_FAILED);
    assert(chassis_get_observation(&observation));
    near(observation.raw_velocity.vx, 0.3f); /* Good feedback survives a send failure. */
    near(chassis.get_control()->vx, 0.0f);
    test_tx_fail = false;

    test_now = 140u;
    assert(!chassis_get_observation(&observation) && observation.age_ms == 100u);
    assert(chassis_poll_feedback() == CHASSIS_OK && !chassis_get_observation(&observation));
    near(observation.raw_velocity.vx, 0.3f); /* Invalid diagnostics retain their last value. */

    test_stamp = UINT32_MAX - 5u;
    test_now = 4u;
    ++test_sequence;
    assert(chassis_poll_feedback() == CHASSIS_OK && chassis_get_observation(&observation));
    assert(observation.age_ms == 10u);
    ++test_sequence;
    assert(!chassis_get_observation(&observation)); /* Cache changed before FK refresh. */
    assert(chassis_poll_feedback() == CHASSIS_OK && chassis_get_observation(&observation));
    test_bad_feedback = true;
    assert(chassis_poll_feedback() == CHASSIS_OK && !chassis_get_observation(&observation));

    test_zero_steer_hold(&config);
    test_zero_drive_hold(&config);
    puts("PASS wheel: passive FK, freshness, zero-angle hold, settled position hold and lifecycle");
    return 0;
}
