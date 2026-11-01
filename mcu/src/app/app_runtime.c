/**
 * @file app_runtime.c
 * @brief 遥控底盘运行时与安全仲裁
 */

#include "app_runtime.h"
#include "app_control.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "remote.h"

#include <stdlib.h>

#define APP_RUNTIME_STARTUP_TIMEOUT_MS 3000u
#define APP_RUNTIME_STOP_RETRY_MS 100u

static bool s_manual_edge_armed = false;
static bool s_motor_starting = false;
static ms_t s_motor_start_since_ms = 0u;
static bool s_stop_requested = false;
static bool s_stop_retry_pending = false;
static ms_t s_stop_last_attempt_ms = 0u;
static bool s_stop_attempted_this_cycle = false;

static bool app_runtime_sticks_centered(void);
static void app_runtime_raise_fault(AppFaultSource source, int32_t code);
static void app_runtime_stop(bool force);

void app_runtime_init(void) {
    s_manual_edge_armed = false;
    s_motor_starting = false;
    s_motor_start_since_ms = 0u;
    s_stop_requested = false;
    s_stop_retry_pending = false;
    s_stop_last_attempt_ms = 0u;
    s_stop_attempted_this_cycle = false;
    app_control_init();
    app_fsm_init();
}

void app_runtime_process(void) {
    const bool online = remote_is_online(100u);
    const bool manual_request = remote_is_manual_requested();
    const bool auto_request = remote_is_auto_requested();
    AppControlResult control_result;
    ChassisErrorCode status;

    s_stop_attempted_this_cycle = false;
    if(!online) {
        s_manual_edge_armed = false;
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL || app_fsm_get_state() == APP_FSM_STATE_AUTO) {
            app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
            return;
        }
    }
    else if(remote_take_clear_reset_event()) {
        s_manual_edge_armed = false;
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
    else if(auto_request) {
        s_manual_edge_armed = true;
        bool stopped_for_switch = false;
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
            app_runtime_stop(false);
            if(s_stop_retry_pending) return;
            stopped_for_switch = true;
        }
        if(app_fsm_get_state() == APP_FSM_STATE_IDLE || app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
            s_motor_starting = true;
            s_motor_start_since_ms = delay_now_ms();
            (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_AUTO);
            if(stopped_for_switch) {
                app_fsm_process();
                return;
            }
        }
    }
    else if(!manual_request) {
        /* Only a fresh non-manual selection can arm the next manual request */
        s_manual_edge_armed = true;
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
        if(app_fsm_get_state() == APP_FSM_STATE_IDLE && s_manual_edge_armed &&
           app_runtime_sticks_centered()) {
            s_manual_edge_armed = false;
            s_motor_starting = true;
            s_motor_start_since_ms = delay_now_ms();
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

    /* Check IMU/fusion readiness before any motor preparation can enable output */
    if(!odom.is_ready()) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_ODOM, 1);
        return;
    }

    /* Preparation now advances one nonblocking step per cycle. Check every
       startup precondition before that step can issue a hardware command */
    if((manual_mode && !remote_is_manual_requested()) ||
       (auto_mode && !remote_is_auto_requested())) {
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
       (auto_mode && !remote_is_auto_requested())) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
        return;
    }

    if(s_motor_starting) {
        /* Startup is bounded and keeps zero targets until all feedback is ready */
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
        /* Autonomous tasks own chassis service targets after zero-speed startup */
        control_result = APP_CONTROL_RESULT_OK;
    }

    if(control_result != APP_CONTROL_RESULT_OK && control_result != APP_CONTROL_RESULT_SKIPPED) {
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
    if(chassis.is_ready()) {
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
    return !app_fsm_has_fault() && app_fsm_get_state() != APP_FSM_STATE_ESTOP &&
           remote_is_online(100u) && odom.is_ready() && chassis.is_self_check_passed();
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

    s_manual_edge_armed = false;
    if(new_fault) {
        AppFault fault = { source, APP_FAULT_LEVEL_RECOVERABLE, code, delay_now_ms() };
        log_warn("APP_RUNTIME fault: source=%u code=%ld", (unsigned int)source, (long)code);
        (void)app_fsm_raise_fault(&fault);
        app_fsm_process();
    }
    app_runtime_stop(new_fault);
}
