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

static bool s_manual_edge_armed = false;
static bool s_motor_starting = false;
static ms_t s_motor_start_since_ms = 0u;

static bool app_runtime_sticks_centered(void);
static void app_runtime_raise_fault(AppFaultSource source, int32_t code);
static void app_runtime_stop(void);

void app_runtime_init(void) {
    s_manual_edge_armed = false;
    s_motor_starting = false;
    s_motor_start_since_ms = 0u;
    app_control_init();
    app_fsm_init();
}

void app_runtime_process(void) {
    const bool online = remote_is_online(100u);
    const bool manual_request = remote_is_manual_requested();
    AppControlResult control_result;
    ChassisErrorCode status;

    if(!online) {
        s_manual_edge_armed = false;
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
            app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
        }
    }
    else if(remote_take_clear_reset_event()) {
        s_manual_edge_armed = false;
        app_runtime_stop();
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
    else if(!manual_request) {
        /* Only a fresh non-manual selection can arm the next manual request */
        s_manual_edge_armed = true;
        if(app_fsm_get_state() == APP_FSM_STATE_MANUAL) {
            (void)app_fsm_post(APP_FSM_EVENT_STOP);
        }
    }
    else if(app_fsm_get_state() == APP_FSM_STATE_IDLE && s_manual_edge_armed &&
            app_runtime_sticks_centered()) {
        s_manual_edge_armed = false;
        s_motor_starting = true;
        s_motor_start_since_ms = delay_now_ms();
        (void)app_fsm_post(APP_FSM_EVENT_SWITCH_TO_MANUAL);
    }

    app_fsm_process();
    if(app_fsm_get_state() != APP_FSM_STATE_MANUAL) {
        app_runtime_stop();
        return;
    }

    status = chassis_set_output_enabled(true);
    if(status != CHASSIS_OK) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, status);
        return;
    }

    if(!remote_is_manual_requested()) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_REMOTE, 1);
        return;
    }

    if(!odom.is_ready()) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_ODOM, 1);
        return;
    }

    if(s_motor_starting) {
        /* Startup is bounded and keeps zero targets until all feedback is ready */
        if(!app_runtime_sticks_centered() ||
           (ms_t)(delay_now_ms() - s_motor_start_since_ms) >= APP_RUNTIME_STARTUP_TIMEOUT_MS) {
            app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, 1);
            return;
        }
        control_result = app_control_stop_all();
    }
    else if(!chassis.is_ready()) {
        app_runtime_raise_fault(APP_FAULT_SOURCE_CHASSIS, 1);
        return;
    }
    else {
        control_result = app_control_apply_manual_chassis();
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

static bool app_runtime_sticks_centered(void) {
    const FsIa10bData* data = remote_get_raw_data();

    return data != NULL &&
           abs((int)data->channel[REMOTE_CH_RIGHT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_RIGHT_Y] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_LEFT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND;
}

static void app_runtime_stop(void) {
    ChassisErrorCode status;
    AppControlResult control_result;

    s_motor_starting = false;
    status = chassis_set_output_enabled(false);
    control_result = app_control_stop_all();
    if(control_result != APP_CONTROL_RESULT_OK && status == CHASSIS_OK) {
        status = CHASSIS_MOTOR_COMMAND_FAILED;
    }
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
    s_manual_edge_armed = false;
    if(!app_fsm_has_fault()) {
        AppFault fault = { source, APP_FAULT_LEVEL_RECOVERABLE, code, delay_now_ms() };
        log_warn("APP_RUNTIME fault: source=%u code=%ld", (unsigned int)source, (long)code);
        (void)app_fsm_raise_fault(&fault);
        app_fsm_process();
    }
    app_runtime_stop();
}
