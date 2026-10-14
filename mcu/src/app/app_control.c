/**
 * @file app_control.c
 * @brief MCU 控制映射与安全执行实现
 */

#include "app_control.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "remote.h"

#include <math.h>

// ! ========================= 宏 定 义 声 明 ========================= ! //

#define REMOTE_CONTROL_PERIOD_S 0.002f

#define REMOTE_FAST_MAX_VX_MPS 2.0f
#define REMOTE_FAST_MAX_VY_MPS 2.0f
#define REMOTE_FAST_MAX_WZ_RAD_S 8.0f
#define REMOTE_MID_MAX_VX_MPS 1.0f
#define REMOTE_MID_MAX_VY_MPS 1.0f
#define REMOTE_MID_MAX_WZ_RAD_S 4.0f
#define REMOTE_SLOW_MAX_VX_MPS 0.5f
#define REMOTE_SLOW_MAX_VY_MPS 0.5f
#define REMOTE_SLOW_MAX_WZ_RAD_S 2.0f

// ! ========================= 类 型 声 明 ========================= ! //

typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

static bool s_remote_yaw_target_valid = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static AppControlResult app_control_merge_result(AppControlResult lhs, AppControlResult rhs);
static uint8_t app_control_result_priority(AppControlResult result);
static AppControlResult app_control_result_from_chassis(ChassisErrorCode status, const char* action);
static float app_control_limit_abs(float value, float limit);
static float app_control_channel_to_norm(uint16_t value, uint16_t deadband);
static RemoteSpeedLimit app_control_get_speed_limit(uint16_t swb);
static bool app_control_get_remote_state(RemoteState* state);
static AppControlResult app_control_apply_remote_chassis(const RemoteState* state);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_control_init(void) {
    ChassisYawHoldConfig yaw_hold_config = chassis_yaw_hold_default_config();

    yaw_hold_config.kp = 48.0f;
    yaw_hold_config.kd = 2.0f;
    yaw_hold_config.v_deadband = 0.01f;
    yaw_hold_config.wz_deadband = 0.08f;
    yaw_hold_config.wz_limit = 1.0f;
    chassis_yaw_hold_init(&yaw_hold_config);
    s_remote_yaw_target_valid = false;
}

AppControlResult app_control_apply_manual_chassis(void) {
    RemoteState state;

    if(!app_control_get_remote_state(&state) || !state.manual_request) {
        return app_control_stop_all();
    }

    return app_control_apply_remote_chassis(&state);
}

AppControlResult app_control_brake_chassis(void) {
    chassis_yaw_hold_reset();
    s_remote_yaw_target_valid = false;
    return app_control_result_from_chassis(chassis.brake(), "brake chassis");
}

AppControlResult app_control_stop_all(void) {
    chassis_yaw_hold_reset();
    s_remote_yaw_target_valid = false;
    return app_control_result_from_chassis(chassis.stop(), "stop chassis");
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static AppControlResult app_control_merge_result(AppControlResult lhs, AppControlResult rhs) {
    if(app_control_result_priority(lhs) >= app_control_result_priority(rhs)) {
        return lhs;
    }

    return rhs;
}

static uint8_t app_control_result_priority(AppControlResult result) {
    switch(result) {
        case APP_CONTROL_RESULT_CHASSIS_ERROR:
        case APP_CONTROL_RESULT_ODOM_ERROR:
            return 4u;

        case APP_CONTROL_RESULT_COMMAND_INVALID:
        case APP_CONTROL_RESULT_UNSUPPORTED:
            return 3u;

        case APP_CONTROL_RESULT_REJECTED:
            return 2u;

        case APP_CONTROL_RESULT_SKIPPED:
            return 1u;

        case APP_CONTROL_RESULT_OK:
        default:
            return 0u;
    }
}

static AppControlResult app_control_result_from_chassis(ChassisErrorCode status, const char* action) {
    if(status == CHASSIS_OK) {
        return APP_CONTROL_RESULT_OK;
    }

    log_warn("APP_CONTROL %s failed: %s", action, chassis.error_code_to_str(status));
    return APP_CONTROL_RESULT_CHASSIS_ERROR;
}

static float app_control_limit_abs(float value, float limit) {
    if(value > limit) {
        return limit;
    }
    if(value < -limit) {
        return -limit;
    }
    return value;
}

static float app_control_channel_to_norm(uint16_t value, uint16_t deadband) {
    int32_t diff = (int32_t)value - (int32_t)REMOTE_CENTER;
    float normalized;

    if(diff < 0) {
        if((uint32_t)(-diff) <= deadband) {
            return 0.0f;
        }
    }
    else if((uint32_t)diff <= deadband) {
        return 0.0f;
    }

    normalized = (float)diff / REMOTE_SPAN;
    return app_control_limit_abs(normalized, 1.0f);
}

static RemoteSpeedLimit app_control_get_speed_limit(uint16_t swb) {
    RemoteSpeedLimit limit;

    if(swb == REMOTE_SW_LOW) {
        limit.max_vx = REMOTE_FAST_MAX_VX_MPS;
        limit.max_vy = REMOTE_FAST_MAX_VY_MPS;
        limit.max_wz = REMOTE_FAST_MAX_WZ_RAD_S;
    }
    else if(swb == REMOTE_SW_HIGH) {
        limit.max_vx = REMOTE_SLOW_MAX_VX_MPS;
        limit.max_vy = REMOTE_SLOW_MAX_VY_MPS;
        limit.max_wz = REMOTE_SLOW_MAX_WZ_RAD_S;
    }
    else {
        limit.max_vx = REMOTE_MID_MAX_VX_MPS;
        limit.max_vy = REMOTE_MID_MAX_VY_MPS;
        limit.max_wz = REMOTE_MID_MAX_WZ_RAD_S;
    }

    return limit;
}

static bool app_control_get_remote_state(RemoteState* state) {
    if(state == NULL || !remote_get_state(state)) {
        return false;
    }

    return state->online && remote_is_online(100u);
}

static AppControlResult app_control_apply_remote_chassis(const RemoteState* state) {
    const uint16_t swb = state->rc_data.channel[REMOTE_CH_SWB];
    const uint16_t swc = state->rc_data.channel[REMOTE_CH_SWC];
    const uint16_t vra = state->rc_data.channel[REMOTE_CH_VRA];
    const uint16_t vrb = state->rc_data.channel[REMOTE_CH_VRB];
    const float ch_right_y = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND);
    const float ch_right_x = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND);
    const float ch_left_x = app_control_channel_to_norm(state->rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND);
    float vx;
    float vy;
    float wz;
    RemoteSpeedLimit speed_limit;
    AppControlResult result = APP_CONTROL_RESULT_OK;

    if(swc == REMOTE_SW_LOW) {
        result = app_control_merge_result(result,
            app_control_result_from_chassis(chassis.set_steer_then_drive_enabled(false),
                "remote set_steer_then_drive disable"));
    }
    else if(swc == REMOTE_SW_CENTER) {
        result = app_control_merge_result(result,
            app_control_result_from_chassis(chassis.set_steer_then_drive_enabled(true),
                "remote set_steer_then_drive enable"));
    }

    if(swc == REMOTE_SW_HIGH || vra <= REMOTE_VR_LOW_THRESHOLD) {
        return app_control_merge_result(result, app_control_brake_chassis());
    }

    if(vrb > REMOTE_VR_LOW_THRESHOLD) {
        chassis_yaw_hold_reset();
        s_remote_yaw_target_valid = false;
        return app_control_merge_result(result,
            app_control_result_from_chassis(chassis.set_velocity(0.0f, 0.0f, 0.0f),
                "remote zero chassis velocity"));
    }

    speed_limit = app_control_get_speed_limit(swb);
    vx = ch_right_y * speed_limit.max_vx;
    vy = -ch_right_x * speed_limit.max_vy;
    wz = -ch_left_x * speed_limit.max_wz;

    {
        Vector3 angle = { 0 };
        Vector3 gyro_corrected = { 0 };

        if(odom.get_angle(&angle) != ODOM_OK || odom.get_gyro_corrected(&gyro_corrected) != ODOM_OK ||
           !isfinite(angle.z) || !isfinite(gyro_corrected.z)) {
            log_warn("APP_CONTROL remote chassis yaw_hold skipped: odom not ready");
            return APP_CONTROL_RESULT_ODOM_ERROR;
        }

        /* A fresh manual heading replaces the former external yaw target */
        if(!s_remote_yaw_target_valid || fabsf(wz) > 0.08f ||
           (fabsf(vx) <= 0.01f && fabsf(vy) <= 0.01f)) {
            chassis_yaw_hold_set_target(angle.z);
            s_remote_yaw_target_valid = true;
        }
        wz = chassis_yaw_hold_apply(vx, vy, wz, angle.z, gyro_corrected.z, REMOTE_CONTROL_PERIOD_S);
    }

    return app_control_merge_result(result,
        app_control_result_from_chassis(chassis.set_velocity(vx, vy, wz),
            "remote chassis set_velocity"));
}
