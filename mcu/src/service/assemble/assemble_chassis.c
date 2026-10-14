#include "assemble.h"

#include "bus_motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "remote.h"
#include "stm32_hal_can.h"

#include <stdlib.h>
#include <string.h>

#define CHASSIS_FEEDBACK_TIMEOUT_MS 100u
#define CHASSIS_DRIVE_GROUP 1u
#define CHASSIS_RX_QUEUE_COUNT 16u

/* Retain the Atlas physical IDs, DM codec limits and register protocol */
static const DmMotorConfig s_steer_config[CHASSIS_MODULE_COUNT] = {
#define STEER_CONFIG(id) { .can_id = (id), .master_id = 0u, .model = DM_MOTOR_MODEL_DM4310, \
    .firmware = { .major = DM_MOTOR_FIRMWARE_V3 }, .default_mode = DM_MOTOR_MODE_POS_VEL, \
    .feedback_timeout_ms = CHASSIS_FEEDBACK_TIMEOUT_MS }
    STEER_CONFIG(1u), STEER_CONFIG(2u), STEER_CONFIG(3u), STEER_CONFIG(4u),
#undef STEER_CONFIG
};

static DmMotorInstance s_steer_instances[CHASSIS_MODULE_COUNT];
static DjiMotorInstance s_drive_instances[CHASSIS_MODULE_COUNT];
static const BusMotorId s_drive_ids[CHASSIS_MODULE_COUNT] = { 5u, 6u, 7u, 8u };

typedef struct {
    uint32_t id;
    uint32_t stamp_ms;
    uint8_t data[8];
} ChassisRxFrame;

typedef struct {
    ChassisRxFrame frames[CHASSIS_RX_QUEUE_COUNT];
    volatile uint8_t read;
    volatile uint8_t write;
    volatile bool overflow;
} ChassisRxQueue;

static ChassisRxQueue s_steer_rx;
static ChassisRxQueue s_drive_rx;
static uint32_t s_feedback_ms[8];
static bool s_feedback_seen[8];
static volatile bool s_registry_ready = false;
static BusMotorStatus s_receive_error = MOTOR_STATUS_OK;

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static void chassis_motor_delay(uint32_t ms);
static bool chassis_prepare_allowed(void);
static bool chassis_feedback_is_valid(void);
static BusMotorStatus chassis_motor_update(void);
static void chassis_reset_transport(void);
static BusMotorStatus chassis_receive_feedback(void);
static BusMotorStatus chassis_prepare_steer_motor(BusMotorId id);
static BusMotorStatus chassis_prepare_drive_motor(BusMotorId id);
static BusMotorStatus chassis_wait_feedback(BusMotorId id, BusMotorFeedback* feedback);
static BusMotorStatus chassis_set_steer_target(BusMotorId id, float position, float velocity);
static void chassis_queue_push(ChassisRxQueue* queue, uint32_t id, const uint8_t data[8]);
static bool chassis_queue_pop(ChassisRxQueue* queue, ChassisRxFrame* frame);
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);

static const BusMotorPortOps s_steer_ops = {
    .send = chassis_steer_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = chassis_motor_delay,
};
static const BusMotorPortOps s_drive_ops = {
    .send = chassis_drive_can_send,
    .now_ms = HAL_GetTick,
};

SystemStatus assemble_chassis(void) {
    ChassisConfig config = {
        .prepare_steer_motor = chassis_prepare_steer_motor,
        .prepare_drive_motor = chassis_prepare_drive_motor,
        .set_steer_target = chassis_set_steer_target,
        .motor_update = chassis_motor_update,
        .clear_motor_fault = chassis_reset_transport,
        .drive_group = CHASSIS_DRIVE_GROUP,
        .drive_sign = { 1, -1, -1, 1 },
        .feedback_is_valid = chassis_feedback_is_valid,
        .model = { .length = 0.26572986916f, .width = 0.26572986916f,
                   .wheel_radius = 0.035276f, .max_wheel_linear_speed = 2.0f },
        .wheel_drive_ratio = 1.0f,
        .steer_target_mode = CHASSIS_STEER_TARGET_ABS_NEAREST,
        .yaw_bias = { .enabled = true, .k_vx = -0.049f, .k_vy = 0.055f, .v_deadband = 0.01f },
    };
    BusMotorStatus status;

    log_info("CHASSIS assemble begin");
    s_registry_ready = false;
    if(bus_motor.init() != MOTOR_STATUS_OK ||
       dm_motor_init(&s_steer_ops, s_steer_instances, CHASSIS_MODULE_COUNT) != MOTOR_STATUS_OK ||
       dji_motor_init(&s_drive_ops, s_drive_instances, CHASSIS_MODULE_COUNT) != MOTOR_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const DjiMotorConfig drive_config = {
            .can_id = i + 1u,
            .model = DJI_MOTOR_MODEL_M2006,
            .feedback_timeout_ms = CHASSIS_FEEDBACK_TIMEOUT_MS,
            .current_limit_a = 10.0f,
            .velocity_limit_rad_s = 60.0f,
            .control_period_s = 0.002f,
            /* Atlas RPM/current-command gains converted to output rad/s and A */
            .kp = 8.0f * 36.0f * 60.0f / (6.28318530718f * 1000.0f),
            .ki = 0.1f * 36.0f * 60.0f / (6.28318530718f * 1000.0f * 0.002f),
            .kd = 0.0f,
            .feedback_current_a_per_lsb = 0.0f,
        };
        status = dm_motor_bind(i + 1u, &s_steer_config[i]);
        if(status == MOTOR_STATUS_OK) status = dji_motor_bind(s_drive_ids[i], &drive_config);
        if(status != MOTOR_STATUS_OK) {
            log_error("CHASSIS bind failed: wheel=%u status=%s", i, bus_motor.status_str(status));
            return SYSTEM_STATUS_ERROR;
        }
    }
    status = bus_motor.group.bind(CHASSIS_DRIVE_GROUP, s_drive_ids, CHASSIS_MODULE_COUNT);
    if(status != MOTOR_STATUS_OK) return SYSTEM_STATUS_ERROR;

    /* All instances exist before RX can deliver parameter ACKs or feedback */
    if(can_register_rx_callback(&hfdcan1, chassis_steer_can_rx_callback, NULL) != STM32_HAL_CAN_OK ||
       can_register_rx_callback(&hfdcan2, chassis_drive_can_rx_callback, NULL) != STM32_HAL_CAN_OK ||
       can_filter_init() != STM32_HAL_CAN_OK) return SYSTEM_STATUS_ERROR;
    s_registry_ready = true;
    if(can_start(&hfdcan1) != STM32_HAL_CAN_OK || can_start(&hfdcan2) != STM32_HAL_CAN_OK) {
        s_registry_ready = false;
        return SYSTEM_STATUS_ERROR;
    }
    /* Attempt every disable even if one device or port reports failure */
    status = MOTOR_STATUS_OK;
    for(BusMotorId id = 1u; id <= 8u; ++id) {
        const BusMotorStatus disabled = bus_motor.basic.disable(id);
        if(disabled != MOTOR_STATUS_OK) {
            log_error("CHASSIS boot disable failed: id=%u status=%s", id, bus_motor.status_str(disabled));
            status = disabled;
        }
    }
    if(status != MOTOR_STATUS_OK) return SYSTEM_STATUS_ERROR;
    if(chassis_init(&config) != CHASSIS_OK || chassis_set_output_enabled(false) != CHASSIS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    log_info("CHASSIS service init ok");
    return SYSTEM_STATUS_OK;
}

static bool chassis_steer_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
}
static bool chassis_drive_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan2, id, data, len) == STM32_HAL_CAN_OK;
}

static void chassis_queue_push(ChassisRxQueue* queue, uint32_t id, const uint8_t data[8]) {
    const uint8_t next = (queue->write + 1u) % CHASSIS_RX_QUEUE_COUNT;
    if(next == queue->read) {
        queue->overflow = true;
        return;
    }
    ChassisRxFrame* frame = &queue->frames[queue->write];
    frame->id = id;
    frame->stamp_ms = HAL_GetTick();
    memcpy(frame->data, data, 8u);
    __DMB();
    queue->write = next;
}

static bool chassis_queue_pop(ChassisRxQueue* queue, ChassisRxFrame* frame) {
    const uint32_t irq_state = can_critical_enter();
    if(queue->read == queue->write) {
        can_critical_exit(irq_state);
        return false;
    }
    *frame = queue->frames[queue->read];
    queue->read = (queue->read + 1u) % CHASSIS_RX_QUEUE_COUNT;
    can_critical_exit(irq_state);
    return true;
}

static BusMotorStatus chassis_receive_feedback(void) {
    ChassisRxFrame frame;
    BusMotorFeedback feedback;
    BusMotorStatus status;

    if(s_steer_rx.overflow || s_drive_rx.overflow) s_receive_error = MOTOR_STATUS_PORT_ERROR;
    for(uint8_t bus = 0u; bus < 2u; ++bus) {
        ChassisRxQueue* queue = bus == 0u ? &s_steer_rx : &s_drive_rx;
        while(chassis_queue_pop(queue, &frame)) {
            if((uint32_t)(HAL_GetTick() - frame.stamp_ms) >= CHASSIS_FEEDBACK_TIMEOUT_MS) {
                s_receive_error = MOTOR_STATUS_TIMEOUT;
                continue;
            }
            status = bus == 0u ? dm_motor_parse_feedback_frame(frame.id, frame.data, &feedback)
                              : dji_motor_parse_feedback_frame(frame.id, frame.data, &feedback);
            if(status == MOTOR_STATUS_OK && feedback.id >= 1u && feedback.id <= 8u) {
                s_feedback_ms[feedback.id - 1u] = frame.stamp_ms;
                s_feedback_seen[feedback.id - 1u] = true;
            }
            else if(status != MOTOR_STATUS_ID_MISMATCH) {
                s_receive_error = status;
            }
        }
    }
    return s_receive_error;
}

static BusMotorStatus chassis_motor_update(void) {
    const BusMotorStatus received = chassis_receive_feedback();
    const BusMotorStatus updated = dji_motor_update();
    return received != MOTOR_STATUS_OK ? received : updated;
}

static bool chassis_feedback_is_valid(void) {
    BusMotorFeedback feedback;
    if(s_receive_error != MOTOR_STATUS_OK) return false;
    for(BusMotorId id = 1u; id <= 8u; ++id) {
        if(!s_feedback_seen[id - 1u] ||
           (uint32_t)(HAL_GetTick() - s_feedback_ms[id - 1u]) >= CHASSIS_FEEDBACK_TIMEOUT_MS ||
           bus_motor.feedback.all(id, &feedback) != MOTOR_STATUS_OK || (id <= 4u && feedback.error_code != 1u) ||
           bus_motor.profile.current(id) != (id <= 4u ? BUS_MOTOR_PROFILE_POSITION : BUS_MOTOR_PROFILE_VELOCITY)) return false;
    }
    return true;
}

static void chassis_motor_delay(uint32_t ms) {
    const uint32_t start = HAL_GetTick();
    while((uint32_t)(HAL_GetTick() - start) < ms) {
        delay_ms(1u);
        remote_process();
        (void)chassis_receive_feedback();
    }
}

static bool chassis_prepare_allowed(void) {
    const FsIa10bData* data;
    remote_process();
    data = remote_get_raw_data();
    return s_receive_error == MOTOR_STATUS_OK && remote_is_manual_requested() && data != NULL &&
           abs((int)data->channel[REMOTE_CH_RIGHT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_RIGHT_Y] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND &&
           abs((int)data->channel[REMOTE_CH_LEFT_X] - (int)REMOTE_CENTER) <= REMOTE_DEADBAND;
}

static BusMotorStatus chassis_wait_feedback(BusMotorId id, BusMotorFeedback* feedback) {
    const uint32_t start = HAL_GetTick();
    BusMotorStatus status;
    do {
        if(!chassis_prepare_allowed()) return MOTOR_STATUS_ERROR;
        status = chassis_receive_feedback();
        if(status != MOTOR_STATUS_OK) return status;
        status = bus_motor.feedback.all(id, feedback);
        if(status == MOTOR_STATUS_OK && s_feedback_seen[id - 1u] &&
           (uint32_t)(HAL_GetTick() - s_feedback_ms[id - 1u]) < CHASSIS_FEEDBACK_TIMEOUT_MS) {
            return feedback->error_code > 1u ? MOTOR_STATUS_ERROR : status;
        }
        if(status != MOTOR_STATUS_OK && status != MOTOR_STATUS_NO_FEEDBACK && status != MOTOR_STATUS_TIMEOUT) return status;
        chassis_motor_delay(1u);
    } while((uint32_t)(HAL_GetTick() - start) < CHASSIS_FEEDBACK_TIMEOUT_MS);
    return status == MOTOR_STATUS_OK ? MOTOR_STATUS_NO_FEEDBACK : status;
}

static BusMotorStatus chassis_prepare_steer_motor(BusMotorId id) {
    BusMotorFeedback feedback;
    BusMotorStatus status;
    if(!chassis_prepare_allowed()) return MOTOR_STATUS_ERROR;
    status = dm_motor_clear_error(id);
    if(status == MOTOR_STATUS_OK) status = bus_motor.basic.disable(id);
    if(status == MOTOR_STATUS_OK) status = bus_motor.profile.activate(id, BUS_MOTOR_PROFILE_POSITION);
    if(status == MOTOR_STATUS_OK) status = chassis_wait_feedback(id, &feedback);
    if(status == MOTOR_STATUS_OK) status = dm_motor_set_pos_vel(id, feedback.position, 0.0f);
    if(status == MOTOR_STATUS_OK && !chassis_prepare_allowed()) status = MOTOR_STATUS_ERROR;
    if(status == MOTOR_STATUS_OK) status = bus_motor.basic.enable(id);
    if(status == MOTOR_STATUS_OK) {
        const uint32_t start = HAL_GetTick();
        do {
            chassis_motor_delay(1u);
            if(!chassis_prepare_allowed()) { status = MOTOR_STATUS_ERROR; break; }
            status = bus_motor.feedback.all(id, &feedback);
            if(status != MOTOR_STATUS_OK || feedback.error_code > 1u) {
                if(status == MOTOR_STATUS_OK) status = MOTOR_STATUS_ERROR;
                break;
            }
            if(feedback.error_code == 1u) break;
        } while((uint32_t)(HAL_GetTick() - start) < CHASSIS_FEEDBACK_TIMEOUT_MS);
        if(status == MOTOR_STATUS_OK && feedback.error_code != 1u) status = MOTOR_STATUS_TIMEOUT;
    }
    if(status != MOTOR_STATUS_OK) log_warn("CHASSIS steer prepare failed: id=%u status=%s", id, bus_motor.status_str(status));
    return status;
}

static BusMotorStatus chassis_prepare_drive_motor(BusMotorId id) {
    BusMotorFeedback feedback;
    if(!chassis_prepare_allowed()) return MOTOR_STATUS_ERROR;
    BusMotorStatus status = chassis_wait_feedback(id, &feedback);
    if(status == MOTOR_STATUS_OK) status = bus_motor.profile.activate(id, BUS_MOTOR_PROFILE_VELOCITY);
    if(status == MOTOR_STATUS_OK) status = bus_motor.basic.enable(id);
    if(status != MOTOR_STATUS_OK) log_warn("CHASSIS drive prepare failed: id=%u status=%s", id, bus_motor.status_str(status));
    return status;
}

static BusMotorStatus chassis_set_steer_target(BusMotorId id, float position, float velocity) {
    return dm_motor_set_pos_vel(id, position, velocity);
}

static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user) {
    (void)hcan;
    (void)user;
    if(!s_registry_ready || header == NULL || header->IdType != FDCAN_STANDARD_ID ||
       header->RxFrameType != FDCAN_DATA_FRAME || header->DataLength != FDCAN_DLC_BYTES_8) return;
    bool known_master = false;
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        if(header->Identifier == s_steer_config[i].master_id) known_master = true;
    }
    if(!known_master) return;
    /* ACK parsing updates only the volatile transaction state needed while main waits */
    if(!dm_motor_parse_parameter_frame(header->Identifier, data)) {
        chassis_queue_push(&s_steer_rx, header->Identifier, data);
    }
}

static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user) {
    (void)hcan;
    (void)user;
    if(!s_registry_ready || header == NULL || header->IdType != FDCAN_STANDARD_ID ||
       header->RxFrameType != FDCAN_DATA_FRAME || header->DataLength != FDCAN_DLC_BYTES_8 ||
       header->Identifier < 0x201u || header->Identifier > 0x204u) return;
    chassis_queue_push(&s_drive_rx, header->Identifier, data);
}

static void chassis_reset_transport(void) {
    const uint32_t irq_state = can_critical_enter();
    s_steer_rx.read = s_steer_rx.write;
    s_drive_rx.read = s_drive_rx.write;
    s_steer_rx.overflow = false;
    s_drive_rx.overflow = false;
    s_receive_error = MOTOR_STATUS_OK;
    memset(s_feedback_seen, 0, sizeof(s_feedback_seen));
    can_critical_exit(irq_state);
}
