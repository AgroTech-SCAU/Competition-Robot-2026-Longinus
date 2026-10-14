#include "assemble.h"

#include "bus_motor/dji_motor.h"
#include "bus_motor/dm_motor.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "remote.h"

#include <stdlib.h>
#include "stm32_hal_can.h"

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_prepare_allowed(void);
static bool chassis_feedback_is_valid(void);
static BusMotorStatus chassis_prepare_steer_motor(uint16_t id);
static BusMotorStatus chassis_prepare_drive_motor(uint16_t id);
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);

// ! ========================= 变 量 声 明 ========================= ! //

#define CHASSIS_FEEDBACK_TIMEOUT_MS 100u

static volatile uint32_t s_steer_feedback_ms[CHASSIS_MODULE_COUNT] = { 0 };
static volatile uint32_t s_drive_feedback_ms[CHASSIS_MODULE_COUNT] = { 0 };
static volatile bool s_steer_feedback_seen[CHASSIS_MODULE_COUNT] = { false };
static volatile bool s_drive_feedback_seen[CHASSIS_MODULE_COUNT] = { false };
static volatile uint8_t s_steer_feedback_status[CHASSIS_MODULE_COUNT] = { 0 };


static const BusMotorPortOps chassis_steer_motor_ops = {
    .send = chassis_steer_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

static const BusMotorPortOps chassis_drive_motor_ops = {
    .send = chassis_drive_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_chassis(void) {
    ChassisConfig chassis_config = {
        .steer_motor_interface = &dm_motor_instance,
        .drive_motor_interface = &dji_motor_instance,
        .steer_ops = &chassis_steer_motor_ops,
        .drive_ops = &chassis_drive_motor_ops,
        .prepare_steer_motor = chassis_prepare_steer_motor,
        .prepare_drive_motor = chassis_prepare_drive_motor,
        .feedback_is_valid = chassis_feedback_is_valid,
        .model = {
            .length = 0.26572986916f,
            .width = 0.26572986916f,
            .wheel_radius = 0.035276f,
            .max_wheel_linear_speed = 2.0f },
        .wheel_drive_ratio = 1.0f,
        .steer_target_mode = CHASSIS_STEER_TARGET_ABS_NEAREST,
        .yaw_bias = {
            .enabled = true,
            .k_vx = -0.049f,
            .k_vy = 0.055f,
            .v_deadband = 0.01f,
        }
    };

    log_info("CHASSIS assemble begin");
    if(can_register_rx_callback(&hfdcan1, chassis_steer_can_rx_callback, NULL) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    if(can_register_rx_callback(&hfdcan2, chassis_drive_can_rx_callback, NULL) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;

    if(can_filter_init() != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS CAN filter ok");
    if(can_start(&hfdcan1) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS FDCAN1 start ok");
    if(can_start(&hfdcan2) != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS FDCAN2 start ok");

    delay_ms(100);

    if(chassis_init(&chassis_config) != chassis.OK) {
        return SYSTEM_STATUS_ERROR;
    }

    log_info("CHASSIS service init ok");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
}

static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan2, id, data, len) == STM32_HAL_CAN_OK;
}

static bool chassis_feedback_is_valid(void) {
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const uint32_t steer_ms = s_steer_feedback_ms[i];
        const uint32_t drive_ms = s_drive_feedback_ms[i];
        const uint32_t now_ms = HAL_GetTick();

        if(!s_steer_feedback_seen[i] || !s_drive_feedback_seen[i] ||
           (uint32_t)(now_ms - steer_ms) > CHASSIS_FEEDBACK_TIMEOUT_MS ||
           (uint32_t)(now_ms - drive_ms) > CHASSIS_FEEDBACK_TIMEOUT_MS ||
           s_steer_feedback_status[i] > 1u) {
            return false;
        }
    }
    return true;
}

static bool chassis_prepare_allowed(void) {
    const FsIa10bData* data;

    /* Refresh input during the original blocking preparation delays */
    remote_process();
    data = remote_get_raw_data();
    return remote_is_manual_requested() && data != NULL &&
           abs((int)data->channel[REMOTE_CH_RIGHT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_RIGHT_Y] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_LEFT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND;
}

static BusMotorStatus chassis_prepare_steer_motor(uint16_t id) {
    if(!chassis_prepare_allowed()) {
        return MOTOR_STATUS_ERROR;
    }
    if(dm_motor_clear_error(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    delay_ms(100);
    if(!chassis_prepare_allowed()) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.enable(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    delay_ms(100);
    if(!chassis_prepare_allowed()) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.switch_mode(id, DM_MOTOR_MODE_POS_VEL) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.set_spd(id, 0.0f) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.set_tor(id, 0.0f) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(steer_motor.brake(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    delay_ms(100);
    if(!chassis_prepare_allowed()) {
        return MOTOR_STATUS_ERROR;
    }
    return MOTOR_STATUS_OK;
}

static BusMotorStatus chassis_prepare_drive_motor(uint16_t id) {
    if(!chassis_prepare_allowed()) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.enable(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.switch_mode(id, DJI_MOTOR_MODE_SPEED) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }
    if(drive_motor.stop(id) != MOTOR_STATUS_OK) {
        return MOTOR_STATUS_ERROR;
    }

    return MOTOR_STATUS_OK;
}

static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                          const FDCAN_RxHeaderTypeDef* header,
                                          const uint8_t data[8],
                                          void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    BusMotorFeedback feedback;
    if(header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
       header->DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }
    if(dm_motor_parse_feedback_frame(header->Identifier, data, &feedback) == MOTOR_STATUS_OK &&
       feedback.id >= 1u && feedback.id <= CHASSIS_MODULE_COUNT) {
        const uint16_t index = feedback.id - 1u;
        s_steer_feedback_status[index] = feedback.error_code;
        s_steer_feedback_ms[index] = HAL_GetTick();
        s_steer_feedback_seen[index] = true;
    }
}

static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                          const FDCAN_RxHeaderTypeDef* header,
                                          const uint8_t data[8],
                                          void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    BusMotorFeedback feedback;
    if(header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
       header->DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }
    if(dji_motor_parse_feedback_frame(header->Identifier, data, &feedback) == MOTOR_STATUS_OK &&
       feedback.id >= 1u && feedback.id <= CHASSIS_MODULE_COUNT) {
        const uint16_t index = feedback.id - 1u;
        s_drive_feedback_ms[index] = HAL_GetTick();
        s_drive_feedback_seen[index] = true;
    }
}
