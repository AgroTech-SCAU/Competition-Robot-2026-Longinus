/* Real runtime/FSM with mocked remote, odometry and motor operations.
 * Run with tools/run_host_tests.ps1; this test never accesses hardware. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "app_runtime.h"
#include "app_control.h"
#include "app_odom_test.h"
#include "chassis.h"
#include "delay.h"
#include "laser_odom.h"
#include "odom.h"
#include "remote.h"

static uint32_t test_now;
static bool test_online, test_clear_event, test_clear_active;
static bool test_laser_online;
static bool test_output_enabled, test_motor_ready;
static unsigned test_enable_calls, test_zero_calls, test_manual_calls, test_auto_calls;
static FsIa10bData test_remote;
static Odom test_odom;

ms_t delay_now_ms(void) { return test_now; }
bool laser_odom_is_online(void) { return test_laser_online; }
bool remote_is_online(uint32_t timeout_ms) { (void)timeout_ms; return test_online; }
bool remote_is_manual_requested(void) {
    return test_online && test_remote.channel[REMOTE_CH_SWD] == REMOTE_SW_HIGH;
}
bool remote_is_auto_requested(void) {
    return test_online && test_remote.channel[REMOTE_CH_SWD] == REMOTE_SW_LOW;
}
bool remote_take_clear_reset_event(void) {
    const bool pending = test_online && test_clear_event;
    test_clear_event = false;
    return pending;
}
bool remote_is_clear_reset_active(void) { return test_online && test_clear_active; }
const FsIa10bData* remote_get_raw_data(void) { return test_online ? &test_remote : NULL; }

ChassisErrorCode chassis_set_output_enabled(bool enabled) {
    test_output_enabled = enabled;
    if(enabled) ++test_enable_calls;
    return CHASSIS_OK;
}
ChassisErrorCode chassis_poll_feedback(void) { return CHASSIS_OK; }
ChassisErrorCode chassis_clear_motor_fault(void) {
    assert(!test_output_enabled);
    return CHASSIS_OK;
}
static ChassisErrorCode process_chassis(void) { return CHASSIS_OK; }
static bool chassis_ready(void) { return test_output_enabled && test_motor_ready; }
static const char* chassis_status(ChassisErrorCode status) { (void)status; return "mock"; }
const struct ChassisInterface chassis_interface = {
    .process = process_chassis,
    .is_ready = chassis_ready,
    .is_self_check_passed = chassis_ready,
    .error_code_to_str = chassis_status,
};
static const Odom* odom_state(void) { return &test_odom; }
static bool odom_ready(void) {
    return test_odom.imu_ready && test_odom.timing_valid && test_odom.fusion_ready;
}
const struct OdomInterface odom_interface = { .get_state = odom_state, .is_ready = odom_ready };

void app_control_init(void) {}
void app_control_reset_chassis(void) {}
void app_odom_test_reset(void) {}
AppControlResult app_control_hold_zero_chassis(void) {
    assert(test_output_enabled);
    ++test_zero_calls;
    return APP_CONTROL_RESULT_OK;
}
AppControlResult app_control_apply_manual_chassis(void) {
    assert(test_output_enabled && chassis_ready() && odom_ready());
    ++test_manual_calls;
    return APP_CONTROL_RESULT_OK;
}
AppControlResult app_odom_test_process(void) {
    assert(test_output_enabled && chassis_ready() && odom_ready());
    ++test_auto_calls;
    return APP_CONTROL_RESULT_OK;
}

static void reset_fixture(void) {
    test_now = 0u;
    test_online = true;
    test_laser_online = true;
    test_clear_event = test_clear_active = false;
    test_output_enabled = test_motor_ready = false;
    test_enable_calls = test_zero_calls = test_manual_calls = test_auto_calls = 0u;
    memset(&test_odom, 0, sizeof(test_odom));
    memset(&test_remote, 0, sizeof(test_remote));
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) test_remote.channel[i] = REMOTE_CENTER;
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_HIGH; /* CH8 negative selects Manual. */
    app_runtime_init();
}
static void step(uint32_t elapsed_ms) {
    test_now += elapsed_ms;
    app_runtime_process();
}
static void imu_ready(void) { test_odom.imu_ready = test_odom.timing_valid = true; }
static void all_ready(void) { imu_ready(); test_motor_ready = test_odom.fusion_ready = true; }

static void test_level_selection_wait_and_prepare(void) {
    reset_fixture();
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL);
    assert(!test_output_enabled && test_enable_calls == 0u && !app_runtime_has_fault());
    step(10000u); /* Calibration waiting is not the 3-second motor preparation. */
    assert(!test_output_enabled && !app_runtime_has_fault());
    test_odom.imu_ready = true;
    step(2u); /* Valid timing is also required before any enable. */
    assert(!test_output_enabled);
    test_odom.timing_valid = true;
    test_remote.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER + REMOTE_DEADBAND + 1u;
    step(10000u);
    assert(!test_output_enabled && !app_runtime_has_fault());
    test_remote.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER;
    step(2u);
    assert(test_output_enabled && test_zero_calls == 1u && test_manual_calls == 0u);
    step(1000u);
    assert(!app_runtime_has_fault() && test_zero_calls == 2u);
    test_motor_ready = true;
    step(2u);
    assert(test_manual_calls == 0u); /* Motor-ready alone must not release zero hold. */
    test_odom.fusion_ready = true;
    step(2u);
    assert(test_manual_calls == 0u);
    step(2u);
    assert(test_manual_calls == 1u && app_runtime_get_state() == APP_FSM_STATE_MANUAL);
    assert(app_runtime_is_self_check_passed());
    test_laser_online = false;
    assert(!app_runtime_is_self_check_passed());
    step(2u);
    assert(test_manual_calls == 2u && !app_runtime_has_fault());
}

static void test_actual_preparation_timeout_and_nonzero_stick(void) {
    reset_fixture();
    step(12000u);
    imu_ready();
    step(2u);
    step(2999u);
    assert(!app_runtime_has_fault());
    step(1u);
    assert(app_runtime_get_state() == APP_FSM_STATE_FAULT && !test_output_enabled);
    reset_fixture();
    imu_ready();
    step(2u);
    test_remote.channel[REMOTE_CH_LEFT_X] = 1600u;
    step(2u);
    assert(app_runtime_has_fault() && !test_output_enabled && test_manual_calls == 0u);
}

static void test_fault_clear_and_held_stop(void) {
    reset_fixture();
    all_ready();
    step(2u);
    step(2u);
    test_odom.imu_ready = false;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_FAULT && !test_output_enabled);
    const unsigned enable_count = test_enable_calls;
    imu_ready();
    step(10000u);
    assert(app_runtime_has_fault() && test_enable_calls == enable_count);
    test_clear_event = test_clear_active = true;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !app_runtime_has_fault());
    step(10000u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    assert(test_enable_calls == enable_count);
    test_clear_active = false;
    step(2u); /* Releasing Stop restores the selected level; no new start gesture. */
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL && test_output_enabled);

    reset_fixture();
    all_ready();
    test_clear_active = true; /* A combination held at boot may have no rising-edge event. */
    step(2u);
    step(10000u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);
    test_clear_active = false;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL && test_output_enabled);
}

static void test_offline_and_existing_auto_flow(void) {
    reset_fixture();
    all_ready();
    step(2u);
    test_online = false;
    step(2u);
    assert(app_runtime_has_fault() && !test_output_enabled);
    test_online = true;
    step(2u);
    assert(app_runtime_has_fault() && !test_output_enabled);

    reset_fixture();
    all_ready();
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_output_enabled);
    assert(test_zero_calls == 1u && test_auto_calls == 0u);
    step(2u);
    assert(test_auto_calls == 1u);
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_HIGH;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL && !test_output_enabled);
    step(2u);
    assert(test_output_enabled && test_manual_calls == 0u);
    step(2u);
    assert(test_manual_calls == 1u);
}

int main(void) {
    test_level_selection_wait_and_prepare();
    test_actual_preparation_timeout_and_nonzero_stick();
    test_fault_clear_and_held_stop();
    test_offline_and_existing_auto_flow();
    puts("PASS runtime: level Manual, calibration/centering wait, zero preparation, held Stop, fault latch and Auto");
    return 0;
}
