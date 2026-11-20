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
static bool test_key_pressed;
static bool test_laser_online;
static bool test_output_enabled, test_motor_ready;
static bool test_pump_enabled;
static uint8_t test_pump_relays;
static unsigned test_enable_calls, test_zero_calls, test_manual_calls, test_auto_calls;
static FsIa10bData test_remote;
static Odom test_odom;
static LaserOdomSample test_laser;
static AppControlResult test_auto_result;

ms_t delay_now_ms(void) { return test_now; }
const LaserOdomSample* laser_odom_get_sample(void) { return &test_laser; }
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
    return test_odom.wheel_valid && test_odom.timing_valid && test_odom.fusion_ready;
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
    assert(test_output_enabled && chassis_ready());
    ++test_auto_calls;
    return test_auto_result;
}

static void set_test_pump_output(bool enabled, uint8_t relays) {
    assert(!enabled || !test_output_enabled);
    assert(enabled || relays == 0u);
    test_pump_enabled = enabled;
    test_pump_relays = relays;
}

static void reset_fixture(void) {
    test_now = 0u;
    test_online = true;
    test_key_pressed = false;
    test_laser_online = true;
    test_clear_event = test_clear_active = false;
    test_output_enabled = test_motor_ready = false;
    test_auto_result = APP_CONTROL_RESULT_OK;
    test_enable_calls = test_zero_calls = test_manual_calls = test_auto_calls = 0u;
    memset(&test_odom, 0, sizeof(test_odom));
    memset(&test_laser, 0, sizeof(test_laser));
    memset(&test_remote, 0, sizeof(test_remote));
    for(unsigned i = 0u; i < FS_IA10B_CHANNEL_COUNT; ++i) test_remote.channel[i] = REMOTE_CENTER;
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_HIGH; /* CH8 negative selects Manual. */
    app_runtime_set_pump_output(set_test_pump_output);
    app_runtime_init();
}
static void step(uint32_t elapsed_ms) {
    test_now += elapsed_ms;
    app_runtime_process(test_key_pressed);
}
static void all_ready(void) {
    test_odom.wheel_valid = test_odom.timing_valid = true;
    test_motor_ready = test_odom.fusion_ready = true;
}

static void side_laser_frame(uint16_t left_mm, uint16_t right_mm) {
    test_laser.distance_mm[LASER_LEFT] = left_mm;
    test_laser.distance_mm[LASER_RIGHT] = right_mm;
    test_laser.valid_mask = (left_mm <= LASER_MAX_RANGE_MM ? 1u << LASER_LEFT : 0u) |
                            (right_mm <= LASER_MAX_RANGE_MM ? 1u << LASER_RIGHT : 0u);
    test_laser.stamp_ms = test_now;
    ++test_laser.received;
    step(2u);
}

static void start_radio_auto(void) {
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    step(2u);
    for(unsigned i = 0; i < 3u; ++i) side_laser_frame(39u, 35u);
}

static void test_side_lasers_start_radio_auto(void) {
    reset_fixture();
    all_ready();
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_CENTER;
    for(unsigned i = 0; i < 3u; ++i) side_laser_frame(49u, 48u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);

    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    step(2u);
    side_laser_frame(49u, 48u);
    side_laser_frame(49u, 48u);
    for(unsigned i = 0; i < 4u; ++i) step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);

    side_laser_frame(86u, 48u);
    side_laser_frame(49u, 48u);
    side_laser_frame(49u, 48u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);
    side_laser_frame(49u, 29u);
    side_laser_frame(49u, 48u);
    side_laser_frame(49u, 48u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);

    step(121u);
    side_laser_frame(30u, 85u);
    side_laser_frame(85u, 30u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);
    side_laser_frame(85u, 85u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_zero_calls == 1u);
    assert(test_auto_calls == 0u);
    step(2u);
    assert(test_auto_calls == 1u && test_output_enabled && !app_runtime_has_fault());

    test_clear_event = test_clear_active = true;
    step(2u);
    test_clear_active = false;
    for(unsigned i = 0; i < 3u; ++i) side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);

    test_remote.channel[REMOTE_CH_SWD] = REMOTE_CENTER;
    step(2u);
    start_radio_auto();
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_zero_calls == 2u);
}

static void test_side_laser_start_requires_online_radio(void) {
    reset_fixture();
    all_ready();
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    test_online = false;
    for(unsigned i = 0; i < 3u; ++i) side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);

    test_online = true;
    step(2u);
    side_laser_frame(39u, 35u);
    side_laser_frame(39u, 35u);
    test_online = false;
    step(2u);
    test_online = true;
    step(2u);
    side_laser_frame(39u, 35u);
    side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && test_enable_calls == 0u);
    side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_zero_calls == 1u);
}

static void test_level_selection_wait_and_prepare(void) {
    reset_fixture();
    test_remote.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER + REMOTE_DEADBAND + 1u;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL);
    assert(!test_output_enabled && test_enable_calls == 0u && !app_runtime_has_fault());
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
    all_ready(); /* This firmware has no IMU even after motor preparation. */
    step(2u);
    assert(test_manual_calls == 0u);
    step(2u);
    assert(test_manual_calls == 1u && app_runtime_get_state() == APP_FSM_STATE_MANUAL);
    assert(app_runtime_is_self_check_passed());
    test_laser_online = false;
    assert(app_runtime_is_self_check_passed()); /* Wheel-only Auto treats laser as telemetry. */
    step(2u);
    assert(test_manual_calls == 2u && !app_runtime_has_fault());
}

static void test_actual_preparation_timeout_and_nonzero_stick(void) {
    reset_fixture();
    step(12000u);
    step(2999u);
    assert(!app_runtime_has_fault());
    step(1u);
    assert(app_runtime_get_state() == APP_FSM_STATE_FAULT && !test_output_enabled);
    reset_fixture();
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
    test_odom.wheel_valid = false;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_FAULT && !test_output_enabled);
    const unsigned enable_count = test_enable_calls;
    all_ready();
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
    start_radio_auto();
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

    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_LOW;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    step(2u);
    side_laser_frame(39u, 35u);
    side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_output_enabled);
}

static void key_level(bool pressed, uint32_t duration_ms) {
    test_key_pressed = pressed;
    step(2u);
    step(duration_ms);
}

static void test_local_start_and_stop(void) {
    reset_fixture();
    all_ready();
    test_online = false;
    key_level(true, 500u); /* Held across power on. */
    key_level(false, 3u);
    key_level(true, 40u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    key_level(false, 31u);
    key_level(true, 5u);
    key_level(false, 3u);
    key_level(true, 29u);
    assert(!test_output_enabled);
    step(1u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_zero_calls == 1u);
    step(2u);
    assert(test_auto_calls == 1u && app_runtime_is_self_check_passed());
    key_level(true, 100u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_manual_calls == 0u);
    key_level(false, 35u);
    key_level(true, 35u); /* Local EStop remains available while the radio is offline. */
    assert(app_runtime_get_state() == APP_FSM_STATE_ESTOP && !test_output_enabled);
    key_level(false, 35u);
    key_level(true, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_ESTOP && !test_output_enabled);
}

static void test_radio_overrides_local_auto(void) {
    reset_fixture();
    all_ready();
    test_online = false;
    key_level(true, 500u);
    key_level(false, 3u);
    key_level(true, 40u);
    key_level(false, 31u);
    key_level(true, 5u);
    key_level(false, 3u);
    key_level(true, 29u);
    step(1u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO);
    test_online = true; /* Default fixture requests Manual: radio takes over immediately. */
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL && !test_output_enabled);
    step(2u);
    assert(test_output_enabled && test_manual_calls == 0u);
    step(2u);
    assert(test_manual_calls == 1u && !app_runtime_has_fault());
}

static void test_key_ownership_and_auto_odom_error(void) {
    reset_fixture();
    all_ready();
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_CENTER;
    key_level(false, 50u);
    key_level(true, 50u); /* Online Idle belongs to the radio. */
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    test_online = false;
    step(100u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE);
    key_level(false, 50u);
    key_level(true, 50u);
    step(2u);
    assert(test_auto_calls == 1u);
    test_odom.wheel_valid = false;
    test_auto_result = APP_CONTROL_RESULT_ODOM_ERROR;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_output_enabled);
    assert(!app_runtime_has_fault());
    all_ready();
    test_auto_result = APP_CONTROL_RESULT_OK;
    test_online = test_clear_event = test_clear_active = true;
    step(2u); /* Online radio stop/clear overrides local-race ownership. */
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    assert(!app_runtime_has_fault());
}

static void test_auto_errors_do_not_fault_or_insert_zero(void) {
    reset_fixture();
    all_ready();
    start_radio_auto();
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_auto_calls == 1u);
    const unsigned zero_calls = test_zero_calls;

    test_auto_result = APP_CONTROL_RESULT_ODOM_ERROR;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_output_enabled);
    assert(!app_runtime_has_fault() && test_zero_calls == zero_calls);
    step(2u);
    assert(!app_runtime_has_fault() && test_zero_calls == zero_calls);

    test_auto_result = APP_CONTROL_RESULT_COMMAND_INVALID;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_output_enabled);
    assert(!app_runtime_has_fault() && test_zero_calls == zero_calls);

    test_auto_result = APP_CONTROL_RESULT_OK;
    step(2u);
    assert(!app_runtime_has_fault() && test_zero_calls == zero_calls);

    test_auto_result = APP_CONTROL_RESULT_ODOM_ERROR;
    test_remote.channel[REMOTE_CH_SWD] = REMOTE_SW_HIGH;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_MANUAL && !test_output_enabled);
    step(2u);
    assert(test_output_enabled && test_manual_calls == 0u);
    step(2u);
    assert(test_manual_calls == 1u && !app_runtime_has_fault());

    reset_fixture();
    all_ready();
    start_radio_auto();
    step(2u);
    test_auto_result = APP_CONTROL_RESULT_CHASSIS_ERROR;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_FAULT && !test_output_enabled);
}

static void test_pump_mode(void) {
    reset_fixture();
    all_ready();
    start_radio_auto();
    step(2u);
    assert(test_output_enabled && test_auto_calls == 1u && !test_pump_enabled);

    test_remote.channel[REMOTE_CH_SWA] = REMOTE_SW_LOW;
    test_remote.channel[REMOTE_CH_SWC] = REMOTE_SW_CENTER;
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE);
    assert(!test_output_enabled && test_auto_calls == 1u);
    assert(test_pump_enabled && test_pump_relays == 0u);

    test_remote.channel[REMOTE_CH_LEFT_X] = REMOTE_CENTER - REMOTE_DEADBAND - 1u;
    test_remote.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER + REMOTE_DEADBAND + 1u;
    step(2u);
    assert(test_pump_relays == (APP_PUMP_RELAY_1 | APP_PUMP_RELAY_4));
    test_remote.channel[REMOTE_CH_LEFT_X] = REMOTE_CENTER + REMOTE_DEADBAND + 1u;
    test_remote.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER - REMOTE_DEADBAND - 1u;
    step(2u);
    assert(test_pump_relays == (APP_PUMP_RELAY_2 | APP_PUMP_RELAY_3));
    test_remote.channel[REMOTE_CH_LEFT_X] = REMOTE_CENTER + REMOTE_DEADBAND;
    test_remote.channel[REMOTE_CH_RIGHT_X] = REMOTE_CENTER - REMOTE_DEADBAND;
    step(2u);
    assert(test_pump_enabled && test_pump_relays == 0u);

    test_remote.channel[REMOTE_CH_LEFT_X] = REMOTE_SW_HIGH;
    test_remote.channel[REMOTE_CH_SWC] = REMOTE_SW_HIGH;
    step(2u);
    assert(!test_pump_enabled && test_pump_relays == 0u && !test_output_enabled);
    test_remote.channel[REMOTE_CH_SWC] = REMOTE_SW_LOW;
    step(2u);
    assert(!test_pump_enabled && test_pump_relays == 0u && !test_output_enabled);
    test_remote.channel[REMOTE_CH_SWC] = REMOTE_SW_CENTER;
    step(2u);
    assert(test_pump_enabled && test_pump_relays == APP_PUMP_RELAY_1);

    test_online = false;
    step(102u);
    assert(!test_pump_enabled && test_pump_relays == 0u);
    test_online = true;
    step(2u);
    assert(test_pump_enabled);
    test_remote.channel[REMOTE_CH_SWA] = REMOTE_SW_HIGH;
    step(2u);
    assert(!test_pump_enabled && test_pump_relays == 0u);
    assert(app_runtime_get_state() == APP_FSM_STATE_IDLE && !test_output_enabled);
    for(unsigned i = 0; i < 3u; ++i) side_laser_frame(39u, 35u);
    assert(app_runtime_get_state() == APP_FSM_STATE_AUTO && test_output_enabled);

    test_remote.channel[REMOTE_CH_SWA] = REMOTE_SW_LOW;
    assert(app_fsm_request_estop());
    step(2u);
    assert(app_runtime_get_state() == APP_FSM_STATE_ESTOP);
    assert(!test_pump_enabled && test_pump_relays == 0u && !test_output_enabled);
}

int main(void) {
    test_side_lasers_start_radio_auto();
    test_side_laser_start_requires_online_radio();
    test_local_start_and_stop();
    test_radio_overrides_local_auto();
    test_key_ownership_and_auto_odom_error();
    test_level_selection_wait_and_prepare();
    test_actual_preparation_timeout_and_nonzero_stick();
    test_fault_clear_and_held_stop();
    test_offline_and_existing_auto_flow();
    test_auto_errors_do_not_fault_or_insert_zero();
    test_pump_mode();
    puts("PASS runtime: radio Auto laser start, local key, EStop, RC safety, Manual faults, Auto override and pump mode");
    return 0;
}
