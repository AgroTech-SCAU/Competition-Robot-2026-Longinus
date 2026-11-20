/**
 * @file app_runtime.c
 * @brief 遥控底盘运行时与安全仲裁
 */

#include "app_runtime.h"
#include "app_control.h"
#include "app_odom_test.h"
#include "chassis.h"
#include "delay.h"
#include "laser_odom.h"
#include "log.h"
#include "odom.h"
#include "remote.h"

#include <stdlib.h>

#define APP_RUNTIME_STARTUP_TIMEOUT_MS 3000u
#define APP_RUNTIME_STOP_RETRY_MS 100u
#define APP_RUNTIME_KEY_DEBOUNCE_MS 30u
#define APP_RUNTIME_LASER_START_MIN_MM 30u
#define APP_RUNTIME_LASER_START_MAX_MM 85u
#define APP_RUNTIME_LASER_START_FRAMES 3u
#define APP_RUNTIME_LASER_START_MAX_AGE_MS 120u

/* A locally started race owns the controls only while the radio is offline. */
static bool s_local_race;
static bool s_key_candidate_pressed;
static bool s_key_armed;
static ms_t s_key_changed_ms;
static bool s_laser_start_waiting;
static bool s_laser_start_consumed;
static uint8_t s_laser_start_count;
static uint32_t s_laser_start_frame;
static ms_t s_laser_start_stamp_ms;
static bool s_waiting_ready = false;
static bool s_motor_starting = false;
static ms_t s_motor_start_since_ms = 0u;
static bool s_stop_requested = false;
static bool s_stop_retry_pending = false;
static ms_t s_stop_last_attempt_ms = 0u;
static bool s_stop_attempted_this_cycle = false;
static AppPumpOutput s_pump_output;
/* Preserve the actual triggering frame; stopping motors makes later wheel
   feedback unavailable and would otherwise hide the readiness failure. */
static volatile Odom s_odom_fault_snapshot;
static volatile bool s_odom_fault_was_starting;

static bool app_runtime_sticks_centered(void);
static void app_runtime_raise_fault(AppFaultSource source, int32_t code);
static void app_runtime_stop(bool force);

void app_runtime_set_pump_output(AppPumpOutput output) {
    s_pump_output = output;
}

void app_runtime_init(void) {
    s_local_race = false;
    s_key_candidate_pressed = true;
    s_key_armed = false; /* Observe a stable release before the first press. */
    s_key_changed_ms = delay_now_ms();
    s_laser_start_waiting = false;
    s_laser_start_consumed = false;
    s_laser_start_count = 0u;
    s_laser_start_frame = 0u;
    s_laser_start_stamp_ms = 0u;
    s_waiting_ready = false;
    s_motor_starting = false;
    s_motor_start_since_ms = 0u;
    s_stop_requested = false;
    s_stop_retry_pending = false;
    s_stop_last_attempt_ms = 0u;
    s_stop_attempted_this_cycle = false;
    app_odom_test_reset();
    app_control_init();
    app_fsm_init();
    if(s_pump_output != NULL) s_pump_output(false, 0u);
}

static uint8_t app_runtime_pump_relays(const FsIa10bData* radio) {
    const uint16_t low = REMOTE_CENTER - REMOTE_DEADBAND;
    const uint16_t high = REMOTE_CENTER + REMOTE_DEADBAND;
    const uint16_t left = radio->channel[REMOTE_CH_LEFT_X];
    const uint16_t right = radio->channel[REMOTE_CH_RIGHT_X];
    uint8_t relays = 0u;

    if(left < low) relays |= APP_PUMP_RELAY_1;
    else if(left > high) relays |= APP_PUMP_RELAY_2;
    if(right < low) relays |= APP_PUMP_RELAY_3;
    else if(right > high) relays |= APP_PUMP_RELAY_4;
    return relays;
}

static bool app_runtime_key_press(bool pressed, ms_t now) {
    if(pressed != s_key_candidate_pressed) {
        s_key_candidate_pressed = pressed;
        s_key_changed_ms = now;
    }
    if((ms_t)(now - s_key_changed_ms) < APP_RUNTIME_KEY_DEBOUNCE_MS) return false;
    if(!pressed) {
        s_key_armed = true;
        return false;
    }
    if(!s_key_armed) return false;
    s_key_armed = false;
    return true;
}

static bool app_runtime_laser_start_ready(bool waiting, ms_t now) {
    const LaserOdomSample* sample = laser_odom_get_sample();
    const unsigned side_mask = (1u << LASER_LEFT) | (1u << LASER_RIGHT);
    const uint16_t left = sample->distance_mm[LASER_LEFT];
    const uint16_t right = sample->distance_mm[LASER_RIGHT];

    /* Only count frames received after the Auto switch arms an idle vehicle. */
    if(!waiting || !s_laser_start_waiting) {
        s_laser_start_waiting = waiting;
        s_laser_start_frame = sample->received;
        s_laser_start_count = 0u;
        return false;
    }
    if(sample->received == 0u || (ms_t)(now - sample->stamp_ms) > APP_RUNTIME_LASER_START_MAX_AGE_MS ||
       (sample->valid_mask & side_mask) != side_mask ||
       left < APP_RUNTIME_LASER_START_MIN_MM || left > APP_RUNTIME_LASER_START_MAX_MM ||
       right < APP_RUNTIME_LASER_START_MIN_MM || right > APP_RUNTIME_LASER_START_MAX_MM) {
        s_laser_start_count = 0u;
        return false;
    }
    if(sample->received != s_laser_start_frame) {
        if((ms_t)(sample->stamp_ms - s_laser_start_stamp_ms) > APP_RUNTIME_LASER_START_MAX_AGE_MS) {
            s_laser_start_count = 0u;
        }
        s_laser_start_frame = sample->received;
        s_laser_start_stamp_ms = sample->stamp_ms;
        if(s_laser_start_count < APP_RUNTIME_LASER_START_FRAMES) ++s_laser_start_count;
    }
    return s_laser_start_count >= APP_RUNTIME_LASER_START_FRAMES;
}

void app_runtime_process(bool local_key_pressed) {
    const ms_t now = delay_now_ms();
    const bool key_event = app_runtime_key_press(local_key_pressed, now);
    const bool online = remote_is_online(100u);
    const FsIa10bData* radio = online ? remote_get_raw_data() : NULL;
    const bool pump_requested = radio != NULL && radio->channel[REMOTE_CH_SWA] == REMOTE_SW_LOW;
    const bool pump_enabled_requested = pump_requested &&
                                        radio->channel[REMOTE_CH_SWC] == REMOTE_SW_CENTER;
    const bool clear_reset_event = online && remote_take_clear_reset_event();
    const bool clear_reset_active = online && remote_is_clear_reset_active();
    const bool manual_request = remote_is_manual_requested();
    const bool auto_request = remote_is_auto_requested();
    if(!online || !auto_request) s_laser_start_consumed = false;
    const bool waiting_for_laser = online && auto_request && !pump_requested && !clear_reset_event &&
                                   !clear_reset_active && !s_laser_start_consumed &&
                                   app_fsm_get_state() == APP_FSM_STATE_IDLE;
    const bool laser_start = app_runtime_laser_start_ready(waiting_for_laser, now);
    AppControlResult control_result;
    ChassisErrorCode status;

    s_stop_attempted_this_cycle = false;
    if(s_pump_output != NULL && (!pump_enabled_requested || clear_reset_event || clear_reset_active)) {
        s_pump_output(false, 0u);
    }
    /* Radio ownership always overrides a locally started race. */
    if(online && s_local_race) {
        s_local_race = false;
        log_info("AUTO radio override");
    }
    if(s_local_race) {
        if(key_event && app_fsm_get_state() == APP_FSM_STATE_AUTO) {
            (void)app_fsm_request_estop();
            app_fsm_process();
            app_runtime_stop(true);
            return;
        }
    }
    else if(!online) {
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL || app_fsm_get_state() == APP_FSM_STATE_AUTO) {
            app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
            return;
        }
        if(key_event && app_fsm_get_state() == APP_FSM_STATE_IDLE &&
           app_fsm_post(APP_FSM_EVENT_SWITCH_TO_AUTO)) {
            s_local_race = true;
            s_waiting_ready = true;
            log_info("AUTO local key start");
        }
    }
    else if(clear_reset_event) {
        app_runtime_stop(false);
        if(s_stop_retry_pending) return;
        if(app_fsm_get_state() == APP_FSM_STATE_FAULT) {
            if(chassis_clear_motor_fault() == CHASSIS_OK) {
                (void)app_fsm_clear_fault();
            }
        }
        else {
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
        }
        app_fsm_process();
        return;
    }
    else if(clear_reset_active) {
        /* The existing stop/clear combination holds output disabled until
           released; its one-shot clear event must not become an auto-restart. */
        app_runtime_stop(false);
        if(s_stop_retry_pending) return;
        (void)app_fsm_post(APP_FSM_EVENT_STOP);
        app_fsm_process();
        return;
    }
    else if(pump_requested) {
        app_runtime_stop(false);
        const AppFsmStateId state = app_fsm_get_state();
        if(state == APP_FSM_STATE_MANUAL || state == APP_FSM_STATE_AUTO) {
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
        }
        app_fsm_process();
        if(s_pump_output != NULL && pump_enabled_requested) {
            const bool enabled = !s_stop_retry_pending && app_fsm_get_state() == APP_FSM_STATE_IDLE;
            s_pump_output(enabled, enabled ? app_runtime_pump_relays(radio) : 0u);
        }
        return;
    }
    else if(auto_request) {
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
            app_runtime_stop(false);
            if(s_stop_retry_pending) return;
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
            app_fsm_process();
            return;
        }
        if(laser_start && app_fsm_post(APP_FSM_EVENT_SWITCH_TO_AUTO)) {
            s_laser_start_consumed = true;
            s_waiting_ready = true;
            log_info("AUTO radio laser start");
        }
    }
    else if(!manual_request) {
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL || app_fsm_get_state() == APP_FSM_STATE_AUTO) {
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
        }
    }
    else {
        bool stopped_for_switch = false;
        if(app_fsm_get_state() == APP_FSM_STATE_AUTO) {
            app_runtime_stop(false);
            if(s_stop_retry_pending) return;
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
            app_fsm_process();
            stopped_for_switch = true;
        }
        if(app_fsm_get_state() == APP_FSM_STATE_IDLE) {
            app_runtime_stop(false);
            if(s_stop_retry_pending) return;
            s_waiting_ready = true;
            (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_MANUAL);
            if(stopped_for_switch) {
                app_fsm_process();
                return;
            }
        }
    }

    app_fsm_process();
    const AppFsmStateId state = app_fsm_get_state();
    const bool manual_mode = state == APP_FSM_STATE_MANUAL;
    const bool auto_mode = state == APP_FSM_STATE_AUTO;
    if(!manual_mode && !auto_mode) {
        app_runtime_stop(false);
        status = chassis_poll_feedback();
        if(status != CHASSIS_OK) {
            app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, status);
        }
        return;
    }

    /* Disabled steering motors may be silent. Zero-target preparation enables
       their feedback; driving starts after wheel odometry has advanced. */
    if(s_waiting_ready) {
        if(manual_mode && !app_runtime_sticks_centered()) {
            status = chassis_poll_feedback();
            if(status != CHASSIS_OK) app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, status);
            return;
        }
        s_waiting_ready = false;
        s_motor_starting = true;
        s_motor_start_since_ms = delay_now_ms();
    }
    if(manual_mode && !s_motor_starting && !odom.is_ready()) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_ODOM, 1);
        return;
    }

    /* Preparation now advances one nonblocking step per cycle. Check every
       startup precondition before that step can issue a hardware command */
    if((manual_mode && !remote_is_manual_requested()) ||
       (auto_mode && !s_local_race && !remote_is_auto_requested())) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
        return;
    }
    if(s_motor_starting &&
       ((manual_mode && !app_runtime_sticks_centered()) ||
        (ms_t)(delay_now_ms() - s_motor_start_since_ms) >= APP_RUNTIME_STARTUP_TIMEOUT_MS)) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, 1);
        return;
    }

    /* Entering active control invalidates the previous safe-stop completion */
    s_stop_requested = false;
    s_stop_retry_pending = false;
    status = chassis_set_output_enabled(true);
    if(status != CHASSIS_OK) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, status);
        return;
    }

    if((manual_mode && !remote_is_manual_requested()) ||
       (auto_mode && !s_local_race && !remote_is_auto_requested())) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
        return;
    }

    if(s_motor_starting) {
        /* Startup is bounded and keeps zero targets until chassis feedback is ready. */
        control_result = app_control_hold_zero_chassis();
    }
    else if(!chassis.is_ready()) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, 1);
        return;
    }
    else if(manual_mode) {
        control_result = app_control_apply_manual_chassis();
    }
    else {
        control_result = app_odom_test_process();
    }

    /* Auto route/state-estimation errors never enter Fault or insert a zero
       command. The previous chassis target remains owned by chassis.process();
       the radio can still switch to Manual. Hardware errors remain fatal. */
    const bool auto_navigation_error = auto_mode &&
        (control_result == APP_CONTROL_RESULT_ODOM_ERROR ||
         control_result == APP_CONTROL_RESULT_COMMAND_INVALID);
    if(!auto_navigation_error && control_result != APP_CONTROL_RESULT_OK &&
       control_result != APP_CONTROL_RESULT_SKIPPED) {
        app_runtime_raise_fault(control_result == APP_CONTROL_RESULT_ODOM_ERROR
                                    ? APP_FAULT_SOURCE_ODOM : APP_FAULT_SOURCE_CHASSIS,
                                control_result);
        return;
    }

    status = chassis.process();
    if(status != CHASSIS_OK) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, status);
        return;
    }
    if(chassis.is_ready() && (auto_mode || odom.is_ready())) {
        s_motor_starting = false;
    }
    else if(!s_motor_starting) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, 1);
    }
}

AppFsmStateId app_runtime_get_state(void) {
    return app_fsm_get_state();
}

const AppFault* app_runtime_get_fault(void) {
    return app_fsm_get_fault();
}

bool app_runtime_has_fault(void) {
    return app_fsm_has_fault();
}

bool app_runtime_is_self_check_passed(void) {
    /* Laser ranging is optional telemetry during open-field wheel-only runs. */
    return !app_fsm_has_fault() && app_fsm_get_state() != APP_FSM_STATE_ESTOP &&
           (s_local_race || remote_is_online(100u)) && odom.is_ready() && chassis.is_self_check_passed();
}

static bool app_runtime_sticks_centered(void) {
    const FsIa10bData* data = remote_get_raw_data();

    return data != NULL &&
           abs((int)data->channel[REMOTE_CH_RIGHT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_RIGHT_Y] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_LEFT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND;
}

static void app_runtime_stop(bool force) {
    ChassisErrorCode status;

    app_odom_test_reset();
    s_waiting_ready = false;
    s_motor_starting = false;
    if(s_stop_attempted_this_cycle) return;
    if(!force && s_stop_requested &&
       (!s_stop_retry_pending ||
        (ms_t)(delay_now_ms() - s_stop_last_attempt_ms) < APP_RUNTIME_STOP_RETRY_MS)) {
        return;
    }

    app_control_reset_chassis();
    s_stop_requested = true;
    s_stop_attempted_this_cycle = true;
    /* Disabling output already performs the complete chassis stop once */
    status = chassis_set_output_enabled(false);
    s_stop_last_attempt_ms = delay_now_ms();
    s_stop_retry_pending = status != CHASSIS_OK;
    if(status != CHASSIS_OK) {
        log_warn("APP_RUNTIME safe stop failed: %s", chassis.error_code_to_str(status));
        if(!app_fsm_has_fault()) {
            AppFault fault = { APP_FAULT_SOURCE_CHASSIS, APP_FAULT_LEVEL_RECOVERABLE,
                               status, delay_now_ms() };
            (void)app_fsm_raise_fault(&fault);
            app_fsm_process();
        }
    }
}

static void app_runtime_raise_fault(AppFaultSource source, int32_t code) {
    const bool new_fault = !app_fsm_has_fault();

    if(new_fault) {
        if(source == APP_FAULT_SOURCE_ODOM) {
            s_odom_fault_snapshot = *odom.get_state();
            s_odom_fault_was_starting = s_motor_starting;
        }
        AppFault fault = { source, APP_FAULT_LEVEL_RECOVERABLE, code, delay_now_ms() };
        log_warn("APP_RUNTIME fault: source=%u code=%ld", (unsigned int)source, (long)code);
        (void)app_fsm_raise_fault(&fault);
        app_fsm_process();
    }
    app_runtime_stop(new_fault);
    if(new_fault && source == APP_FAULT_SOURCE_ODOM && code == 1) {
        log_warn("WHEEL ODOM prep=%u tim=%u pose=%u wh=%u age_ms=%lu dt_ms=%.1f gap=%lu",
                 s_odom_fault_was_starting ? 1u : 0u, s_odom_fault_snapshot.timing_valid ? 1u : 0u,
                 s_odom_fault_snapshot.fusion_ready ? 1u : 0u, s_odom_fault_snapshot.wheel_valid ? 1u : 0u,
                 (unsigned long)s_odom_fault_snapshot.wheel_age_ms,
                 (double)s_odom_fault_snapshot.process_dt_s * 1000.0,
                 (unsigned long)s_odom_fault_snapshot.discontinuity_count);
    }
}
