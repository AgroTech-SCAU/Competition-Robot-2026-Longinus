/**
 * @file app_status.c
 * @brief 底盘状态显示与低频日志
 */

#include "app_status.h"
#include "app_fsm.h"
#include "app_runtime.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "remote.h"
#include "rgb_led.h"

#include <stddef.h>

typedef enum {
    APP_LED_STATE_NOT_READY = 0,
    APP_LED_STATE_READY,
    APP_LED_STATE_MANUAL,
    APP_LED_STATE_FAULT,
    APP_LED_STATE_ESTOP
} AppLedState;

static ms_t s_log_timer = 0u;
static ms_t s_heartbeat_timer = 0u;
static AppLedState s_led_state = APP_LED_STATE_NOT_READY;

static void app_status_update_led(void);

void app_status_init(void) {
    s_log_timer = 0u;
    s_heartbeat_timer = 0u;
    s_led_state = APP_LED_STATE_NOT_READY;
}

void app_status_process(void) {
    const AppFault* fault = app_runtime_get_fault();

    app_status_update_led();
    if(delay_nb_ms(&s_log_timer, 1000u)) {
        log_info("Heartbeat state=%s remote=%u ready=%u active=%u fault=%u src=%u code=%ld",
                 app_fsm_state_str(app_runtime_get_state()),
                 remote_is_online(100u) ? 1u : 0u,
                 app_runtime_is_self_check_passed() ? 1u : 0u,
                 chassis.is_ready() ? 1u : 0u,
                 app_runtime_has_fault() ? 1u : 0u,
                 fault != NULL ? (unsigned int)fault->source : 0u,
                 fault != NULL ? (long)fault->code : 0l);
        assemble_chassis_log_diagnostics();
        assemble_remote_log_diagnostics();
        assemble_scheduler_log_diagnostics();
    }
}

static void app_status_update_led(void) {
    AppLedState target_state;
    const AppFsmStateId state = app_runtime_get_state();

    if(!delay_nb_ms(&s_heartbeat_timer, 200u)) {
        return;
    }

    switch(state) {
        case APP_FSM_STATE_ESTOP:
            target_state = APP_LED_STATE_ESTOP;
            break;

        case APP_FSM_STATE_FAULT:
            target_state = APP_LED_STATE_FAULT;
            break;

        case APP_FSM_STATE_MANUAL:
            target_state = APP_LED_STATE_MANUAL;
            break;

        case APP_FSM_STATE_AUTO:
        case APP_FSM_STATE_IDLE:
        default:
            if(!app_runtime_is_self_check_passed()) {
                target_state = APP_LED_STATE_NOT_READY;
            }
            else {
                target_state = APP_LED_STATE_READY;
            }
            break;
    }

    if(target_state == s_led_state) {
        return;
    }

    switch(target_state) {
        case APP_LED_STATE_NOT_READY:
            rgb_led.fill(255u, 0u, 0u);
            break;

        case APP_LED_STATE_READY:
            rgb_led.fill(0u, 255u, 0u);
            break;

        case APP_LED_STATE_MANUAL:
            rgb_led.fill(0u, 0u, 255u);
            break;

        case APP_LED_STATE_FAULT:
            rgb_led.fill(255u, 128u, 0u);
            break;

        case APP_LED_STATE_ESTOP:
        default:
            rgb_led.fill(255u, 0u, 255u);
            break;
    }

    if(rgb_led.show() == RGB_LED_STATUS_OK) {
        s_led_state = target_state;
    }
}
