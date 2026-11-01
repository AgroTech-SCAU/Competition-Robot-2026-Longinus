#include "assemble.h"

#include "bus_motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"

#include <string.h>

#define CHASSIS_FEEDBACK_TIMEOUT_MS 100u
#define CHASSIS_DRIVE_GROUP 1u
#define CHASSIS_RX_QUEUE_COUNT 64u
#define CHASSIS_MOTOR_COUNT (2u * CHASSIS_MODULE_COUNT)

/* Proven Atlas wire ranges are independent of the POS_VEL speed command */
static const DmMotorLimitParam s_steer_codec_limits = {
    .q_max = 12.5f, .dq_max = 10.0f, .tau_max = 28.0f,
};
static const DmMotorConfig s_steer_config[CHASSIS_MODULE_COUNT] = {
#define STEER_CONFIG(id) { .can_id = (id), .master_id = (0x10u + (id)), .model = DM_MOTOR_MODEL_CUSTOM, .firmware = { .major = DM_MOTOR_FIRMWARE_V3 }, .default_mode = DM_MOTOR_MODE_POS_VEL, .feedback_timeout_ms = CHASSIS_FEEDBACK_TIMEOUT_MS, .limits_override = &s_steer_codec_limits, .pos_vel_max_speed_rad_s = 12.56f }
    STEER_CONFIG(1u), STEER_CONFIG(2u), STEER_CONFIG(3u), STEER_CONFIG(4u),
#undef STEER_CONFIG
};
static const BusMotorId s_steer_ids[CHASSIS_MODULE_COUNT] = { 1u, 2u, 3u, 4u };
static const BusMotorId s_drive_ids[CHASSIS_MODULE_COUNT] = { 5u, 6u, 7u, 8u };
static DmMotorInstance s_steer_instances[CHASSIS_MODULE_COUNT];
static DjiMotorInstance s_drive_instances[CHASSIS_MODULE_COUNT];
static DmMotorBus s_steer_bus;
static DjiMotorBus s_drive_bus;

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

typedef enum {
    PREPARE_CLEAR = 0, PREPARE_DISABLE, PREPARE_ACTIVATE,
    PREPARE_POSITION, PREPARE_ZERO_TARGET, PREPARE_ENABLE,
    PREPARE_ENABLED_FEEDBACK, PREPARE_DONE,
} ChassisPreparePhase;
typedef struct {
    ChassisPreparePhase phase;
    uint32_t since_ms;
    uint32_t feedback_sequence;
    float position;
} ChassisPrepareState;

static ChassisRxQueue s_steer_rx;
static ChassisRxQueue s_drive_rx;
static ChassisPrepareState s_steer_prepare[CHASSIS_MODULE_COUNT];
static ChassisPrepareState s_drive_prepare[CHASSIS_MODULE_COUNT];
/* Raw startup trace is diagnostic data; the ISR never parses a protocol */
static volatile struct { uint32_t count; ChassisRxFrame frames[16]; } s_steer_rx_trace;
static uint32_t s_feedback_ms[CHASSIS_MOTOR_COUNT];
static uint32_t s_feedback_sequence[CHASSIS_MOTOR_COUNT];
static bool s_feedback_seen[CHASSIS_MOTOR_COUNT];
static volatile bool s_registry_ready;
static BusMotorStatus s_receive_error = MOTOR_STATUS_OK;
static uint32_t s_steer_rx_mismatch_count;

static BusMotorStatus chassis_can_status(BspCanStatus status);
static BusMotorStatus chassis_can_send(void* context, uint32_t id, const uint8_t* data, uint8_t len);
static uint32_t chassis_now_ms(void* context);
static BusMotorStatus chassis_receive_feedback(void);
static BusMotorStatus chassis_motor_update(void);
static BusMotorStatus chassis_disable_drive_motors(void);
static bool chassis_feedback_is_valid(void);
static bool chassis_feedback_is_healthy(void);
static BusMotorStatus chassis_cancel_pending_commands(void);
static void chassis_reset_transport(void);
static BusMotorStatus chassis_prepare_steer_motor(BusMotorId id);
static BusMotorStatus chassis_prepare_drive_motor(BusMotorId id);
static BusMotorStatus chassis_set_steer_target(BusMotorId id, float position, float velocity);
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef*, const FDCAN_RxHeaderTypeDef*, const uint8_t data[8], void*);
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef*, const FDCAN_RxHeaderTypeDef*, const uint8_t data[8], void*);

static const BusMotorPortOps s_steer_ops = {
    .context = &hfdcan1, .send_frame = chassis_can_send, .clock_ms = chassis_now_ms,
};
static const BusMotorPortOps s_drive_ops = {
    .context = &hfdcan2, .send_frame = chassis_can_send, .clock_ms = chassis_now_ms,
};

/* Boot may wait for board settling, explicitly servicing the RX queue
 * This is not a driver delay callback and never runs application policy */
static BusMotorStatus chassis_boot_settle(uint32_t ms) {
    const uint32_t start = HAL_GetTick();
    while((uint32_t)(HAL_GetTick() - start) < ms) {
        delay_ms(1u);
        BusMotorStatus status = chassis_receive_feedback();
        if(status != MOTOR_STATUS_OK) return status;
    }
    return MOTOR_STATUS_OK;
}

SystemStatus assemble_chassis(void) {
    ChassisConfig config = {
        .prepare_steer_motor = chassis_prepare_steer_motor,
        .prepare_drive_motor = chassis_prepare_drive_motor,
        .set_steer_target = chassis_set_steer_target,
        .motor_update = chassis_motor_update,
        .motor_poll_feedback = chassis_receive_feedback,
        .cancel_pending_commands = chassis_cancel_pending_commands,
        .disable_drive_motors = chassis_disable_drive_motors,
        .clear_motor_fault = chassis_reset_transport,
        .drive_group = CHASSIS_DRIVE_GROUP,
        .drive_sign = { 1, -1, -1, 1 },
        .feedback_is_valid = chassis_feedback_is_valid,
        .feedback_is_healthy = chassis_feedback_is_healthy,
        .model = { .length = 0.129f, .width = 0.129f, .wheel_radius = 0.035276f, .max_wheel_linear_speed = 2.0f },
        .wheel_drive_ratio = 1.0f,
        .steer_target_mode = CHASSIS_STEER_TARGET_ABS_NEAREST,
        .yaw_bias = { .enabled = true, .k_vx = -0.049f, .k_vy = 0.055f, .v_deadband = 0.01f },
    };
    BusMotorStatus status;
    log_info("CHASSIS assemble begin");
    s_registry_ready = false;
    memcpy(config.steer_ids, s_steer_ids, sizeof(s_steer_ids));
    memcpy(config.drive_ids, s_drive_ids, sizeof(s_drive_ids));
    if(bus_motor.init() != MOTOR_STATUS_OK ||
       dm_motor_bus_init(&s_steer_bus, &s_steer_ops, s_steer_instances, CHASSIS_MODULE_COUNT) != MOTOR_STATUS_OK ||
       dji_motor_bus_init(&s_drive_bus, &s_drive_ops, s_drive_instances, CHASSIS_MODULE_COUNT) != MOTOR_STATUS_OK)
        return SYSTEM_STATUS_ERROR;
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const DjiMotorConfig drive_config = {
            .can_id = i + 1u, .model = DJI_MOTOR_MODEL_M2006,
            .feedback_timeout_ms = CHASSIS_FEEDBACK_TIMEOUT_MS,
            .current_limit_a = 10.0f, .velocity_limit_rad_s = 60.0f,
            .control_period_s = 0.002f,
            /* Atlas RPM/current gains converted to output rad/s and A */
            .kp = 8.0f * 36.0f * 60.0f / (6.28318530718f * 1000.0f),
            .ki = 0.1f * 36.0f * 60.0f / (6.28318530718f * 1000.0f * 0.002f),
            .kd = 0.0f, .feedback_current_a_per_lsb = 0.0f,
        };
        status = dm_motor_bus_bind(&s_steer_bus, s_steer_ids[i], &s_steer_config[i]);
        if(status == MOTOR_STATUS_OK)
            status = dji_motor_bus_bind(&s_drive_bus, s_drive_ids[i], &drive_config);
        if(status != MOTOR_STATUS_OK) {
            log_error("CHASSIS bind failed: wheel=%u status=%s", i, bus_motor.status_str(status));
            return SYSTEM_STATUS_ERROR;
        }
    }
    if(bus_motor.group.bind(CHASSIS_DRIVE_GROUP, s_drive_ids, CHASSIS_MODULE_COUNT) != MOTOR_STATUS_OK)
        return SYSTEM_STATUS_ERROR;
    if(can_register_rx_callback(&hfdcan1, chassis_steer_can_rx_callback, NULL) != STM32_HAL_CAN_OK ||
       can_register_rx_callback(&hfdcan2, chassis_drive_can_rx_callback, NULL) != STM32_HAL_CAN_OK ||
       can_filter_init() != STM32_HAL_CAN_OK)
        return SYSTEM_STATUS_ERROR;
    s_registry_ready = true;
    if(can_start(&hfdcan1) != STM32_HAL_CAN_OK || can_start(&hfdcan2) != STM32_HAL_CAN_OK) {
        s_registry_ready = false;
        return SYSTEM_STATUS_ERROR;
    }
    if(chassis_boot_settle(100u) != MOTOR_STATUS_OK) return SYSTEM_STATUS_ERROR;
    status = MOTOR_STATUS_OK;
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const BusMotorStatus disabled = bus_motor.basic.disable(s_steer_ids[i]);
        if(disabled != MOTOR_STATUS_OK) {
            log_error("CHASSIS boot disable failed: id=%u status=%s", s_steer_ids[i], bus_motor.status_str(disabled));
            if(status == MOTOR_STATUS_OK) status = disabled;
        }
    }
    const BusMotorStatus drive_disabled = chassis_disable_drive_motors();
    if(status != MOTOR_STATUS_OK || drive_disabled != MOTOR_STATUS_OK) return SYSTEM_STATUS_ERROR;
    if(chassis_init(&config) != CHASSIS_OK || chassis_set_output_enabled(false) != CHASSIS_OK)
        return SYSTEM_STATUS_ERROR;
    log_info("CHASSIS service init ok");
    return SYSTEM_STATUS_OK;
}

static BusMotorStatus chassis_can_status(BspCanStatus status) {
    switch(status) {
        case STM32_HAL_CAN_OK: return MOTOR_STATUS_OK;
        case STM32_HAL_CAN_INVALID_PARAM:
        case STM32_HAL_CAN_INVALID_DLC: return MOTOR_STATUS_INVALID_PARAM;
        case STM32_HAL_CAN_TX_FIFO_FULL: return MOTOR_STATUS_TX_FULL;
        case STM32_HAL_CAN_TX_MAILBOX_TIMEOUT: return MOTOR_STATUS_TIMEOUT;
        case STM32_HAL_CAN_TX_CANCEL_FAILED: return MOTOR_STATUS_CANCELLED;
        default: return MOTOR_STATUS_PORT_ERROR;
    }
}
static BusMotorStatus chassis_can_send(void* context, uint32_t id, const uint8_t* data, uint8_t len) {
    FDCAN_HandleTypeDef* hcan = (FDCAN_HandleTypeDef*)context;
    const BspCanStatus sent = can_send(hcan, id, data, len);
    if(sent != STM32_HAL_CAN_OK) {
        BspCanDiagnostics diagnostic = { 0 };
        if(can_get_diagnostics(hcan, &diagnostic) == STM32_HAL_CAN_OK && diagnostic.bus_off)
            return MOTOR_STATUS_BUS_OFF;
    }
    return chassis_can_status(sent);
}
static uint32_t chassis_now_ms(void* context) { (void)context; return HAL_GetTick(); }

static int chassis_motor_slot(BusMotorId id) {
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        if(id == s_steer_ids[i]) return i;
        if(id == s_drive_ids[i]) return CHASSIS_MODULE_COUNT + i;
    }
    return -1;
}
static int chassis_role_index(const BusMotorId ids[CHASSIS_MODULE_COUNT], BusMotorId id) {
    for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) if(ids[i] == id) return i;
    return -1;
}
static void chassis_queue_push(ChassisRxQueue* queue, uint32_t id, const uint8_t data[8]) {
    const uint8_t next = (queue->write + 1u) % CHASSIS_RX_QUEUE_COUNT;
    if(next == queue->read) { queue->overflow = true; return; }
    ChassisRxFrame* frame = &queue->frames[queue->write];
    frame->id = id;
    frame->stamp_ms = HAL_GetTick();
    memcpy(frame->data, data, 8u);
    __DMB();
    queue->write = next;
}
static bool chassis_queue_pop(ChassisRxQueue* queue, ChassisRxFrame* frame) {
    const uint32_t irq_state = can_critical_enter();
    if(queue->read == queue->write) { can_critical_exit(irq_state); return false; }
    *frame = queue->frames[queue->read];
    queue->read = (queue->read + 1u) % CHASSIS_RX_QUEUE_COUNT;
    can_critical_exit(irq_state);
    return true;
}
static BusMotorStatus chassis_receive_feedback(void) {
    ChassisRxFrame frame;
    if(s_steer_rx.overflow || s_drive_rx.overflow) s_receive_error = MOTOR_STATUS_PORT_ERROR;
    for(uint8_t bus = 0u; bus < 2u; ++bus) {
        ChassisRxQueue* queue = bus == 0u ? &s_steer_rx : &s_drive_rx;
        while(chassis_queue_pop(queue, &frame)) {
            BusMotorFeedback feedback;
            bool is_feedback = true;
            if((uint32_t)(HAL_GetTick() - frame.stamp_ms) >= CHASSIS_FEEDBACK_TIMEOUT_MS) {
                s_receive_error = MOTOR_STATUS_TIMEOUT;
                continue;
            }
            BusMotorStatus status = bus == 0u
                ? dm_motor_bus_receive_at(&s_steer_bus, frame.id, frame.data, sizeof(frame.data), frame.stamp_ms, &feedback, &is_feedback)
                : dji_motor_bus_parse_feedback_frame_at(&s_drive_bus, frame.id, frame.data, frame.stamp_ms, &feedback);
            if(status == MOTOR_STATUS_OK && is_feedback) {
                const int slot = chassis_motor_slot(feedback.id);
                if(slot >= 0) {
                    s_feedback_ms[slot] = frame.stamp_ms;
                    s_feedback_seen[slot] = true;
                    ++s_feedback_sequence[slot];
                }
            }
            else if(status == MOTOR_STATUS_ID_MISMATCH || status == MOTOR_STATUS_NOT_FOUND) {
                if(bus == 0u) ++s_steer_rx_mismatch_count;
            }
            else if(status != MOTOR_STATUS_OK) s_receive_error = status;
        }
    }
    return s_receive_error;
}
static BusMotorStatus chassis_motor_update(void) {
    const BusMotorStatus received = chassis_receive_feedback();
    const BusMotorStatus updated = dji_motor_bus_update(&s_drive_bus);
    return received != MOTOR_STATUS_OK ? received : updated;
}
static BusMotorStatus chassis_disable_drive_motors(void) { return dji_motor_bus_disable_all(&s_drive_bus); }
static bool chassis_feedback_is_healthy(void) {
    if(s_receive_error != MOTOR_STATUS_OK || s_steer_rx.overflow || s_drive_rx.overflow)
        return false;
    for(uint8_t slot = 0u; slot < CHASSIS_MOTOR_COUNT; ++slot) {
        if(!s_feedback_seen[slot] ||
           (uint32_t)(HAL_GetTick() - s_feedback_ms[slot]) >= CHASSIS_FEEDBACK_TIMEOUT_MS)
            return false;
    }
    return true;
}
static bool chassis_feedback_is_valid(void) {
    if(s_receive_error != MOTOR_STATUS_OK) return false;
    for(uint8_t role = 0u; role < 2u; ++role) {
        const BusMotorId* ids = role == 0u ? s_steer_ids : s_drive_ids;
        const BusMotorProfile profile = role == 0u ? BUS_MOTOR_PROFILE_POSITION : BUS_MOTOR_PROFILE_VELOCITY;
        for(uint8_t i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            const uint8_t slot = role * CHASSIS_MODULE_COUNT + i;
            BusMotorFeedback feedback;
            if(!s_feedback_seen[slot] ||
               (uint32_t)(HAL_GetTick() - s_feedback_ms[slot]) >= CHASSIS_FEEDBACK_TIMEOUT_MS ||
               bus_motor.feedback.all(ids[i], &feedback) != MOTOR_STATUS_OK ||
               (feedback.valid & BUS_MOTOR_FEEDBACK_STATE) == 0u || feedback.state != BUS_MOTOR_STATE_ENABLED ||
               bus_motor.profile.current(ids[i]) != profile) return false;
        }
    }
    return true;
}

/* No callback below blocks or services unrelated application work */
static BusMotorStatus chassis_prepare_feedback(BusMotorId id, BusMotorFeedback* feedback) {
    const int slot = chassis_motor_slot(id);
    if(s_receive_error != MOTOR_STATUS_OK) return s_receive_error;
    if(slot < 0) return MOTOR_STATUS_NOT_FOUND;
    BusMotorStatus status = bus_motor.feedback.all(id, feedback);
    if(status != MOTOR_STATUS_OK) return status;
    if(!s_feedback_seen[slot] ||
       (uint32_t)(HAL_GetTick() - s_feedback_ms[slot]) >= CHASSIS_FEEDBACK_TIMEOUT_MS)
        return MOTOR_STATUS_NO_FEEDBACK;
    if((feedback->valid & BUS_MOTOR_FEEDBACK_STATE) == 0u || feedback->state == BUS_MOTOR_STATE_UNKNOWN)
        return MOTOR_STATUS_NO_FEEDBACK;
    return feedback->state == BUS_MOTOR_STATE_FAULT ? MOTOR_STATUS_ERROR : MOTOR_STATUS_OK;
}
static BusMotorStatus chassis_prepare_wait_status(ChassisPrepareState* state, BusMotorStatus status) {
    if(status != MOTOR_STATUS_OK && status != MOTOR_STATUS_NO_FEEDBACK && status != MOTOR_STATUS_TIMEOUT)
        return status;
    return (uint32_t)(HAL_GetTick() - state->since_ms) >= CHASSIS_FEEDBACK_TIMEOUT_MS
        ? MOTOR_STATUS_TIMEOUT : MOTOR_STATUS_BUSY;
}
static BusMotorStatus chassis_prepare_steer_motor(BusMotorId id) {
    const int index = chassis_role_index(s_steer_ids, id);
    if(index < 0) return MOTOR_STATUS_NOT_FOUND;
    ChassisPrepareState* state = &s_steer_prepare[index];
    BusMotorStatus status;
    BusMotorFeedback feedback;
    switch(state->phase) {
        case PREPARE_CLEAR:
            status = dm_motor_clear_error(id);
            if(status == MOTOR_STATUS_OK) state->phase = PREPARE_DISABLE;
            break;
        case PREPARE_DISABLE:
            status = bus_motor.basic.disable(id);
            if(status == MOTOR_STATUS_OK) state->phase = PREPARE_ACTIVATE;
            break;
        case PREPARE_ACTIVATE:
            status = bus_motor.profile.activate(id, BUS_MOTOR_PROFILE_POSITION);
            if(status == MOTOR_STATUS_OK) {
                state->phase = PREPARE_POSITION;
                state->since_ms = HAL_GetTick();
            }
            break;
        case PREPARE_POSITION:
            status = chassis_prepare_feedback(id, &feedback);
            if(status == MOTOR_STATUS_OK && feedback.state == BUS_MOTOR_STATE_DISABLED &&
               (feedback.valid & BUS_MOTOR_FEEDBACK_POSITION) != 0u) {
                state->position = feedback.position;
                state->phase = PREPARE_ZERO_TARGET;
            } else return chassis_prepare_wait_status(state, status);
            break;
        case PREPARE_ZERO_TARGET:
            status = dm_motor_set_pos_vel(id, state->position, 0.0f);
            if(status == MOTOR_STATUS_OK) state->phase = PREPARE_ENABLE;
            break;
        case PREPARE_ENABLE:
            status = bus_motor.basic.enable(id);
            if(status == MOTOR_STATUS_OK) {
                state->phase = PREPARE_ENABLED_FEEDBACK;
                state->since_ms = HAL_GetTick();
                state->feedback_sequence = s_feedback_sequence[index];
            }
            break;
        case PREPARE_ENABLED_FEEDBACK:
            status = chassis_prepare_feedback(id, &feedback);
            if(status == MOTOR_STATUS_OK && feedback.state == BUS_MOTOR_STATE_ENABLED &&
               s_feedback_sequence[index] != state->feedback_sequence) {
                state->phase = PREPARE_DONE;
                return MOTOR_STATUS_OK;
            }
            return chassis_prepare_wait_status(state, status);
        case PREPARE_DONE: return MOTOR_STATUS_OK;
        default: return MOTOR_STATUS_ERROR;
    }
    return status == MOTOR_STATUS_OK ? MOTOR_STATUS_BUSY : status;
}
static BusMotorStatus chassis_prepare_drive_motor(BusMotorId id) {
    const int index = chassis_role_index(s_drive_ids, id);
    if(index < 0) return MOTOR_STATUS_NOT_FOUND;
    ChassisPrepareState* state = &s_drive_prepare[index];
    BusMotorFeedback feedback;
    BusMotorStatus status;
    switch(state->phase) {
        case PREPARE_CLEAR:
            state->since_ms = HAL_GetTick();
            state->phase = PREPARE_POSITION;
            return MOTOR_STATUS_BUSY;
        case PREPARE_POSITION:
            status = chassis_prepare_feedback(id, &feedback);
            if(status != MOTOR_STATUS_OK) return chassis_prepare_wait_status(state, status);
            state->phase = PREPARE_ACTIVATE;
            return MOTOR_STATUS_BUSY;
        case PREPARE_ACTIVATE:
            status = bus_motor.profile.activate(id, BUS_MOTOR_PROFILE_VELOCITY);
            if(status == MOTOR_STATUS_OK) state->phase = PREPARE_ENABLE;
            break;
        case PREPARE_ENABLE:
            status = bus_motor.basic.enable(id);
            if(status == MOTOR_STATUS_OK) { state->phase = PREPARE_DONE; return MOTOR_STATUS_OK; }
            break;
        case PREPARE_DONE: return MOTOR_STATUS_OK;
        default: return MOTOR_STATUS_ERROR;
    }
    return status == MOTOR_STATUS_OK ? MOTOR_STATUS_BUSY : status;
}
static BusMotorStatus chassis_set_steer_target(BusMotorId id, float position, float velocity) {
    return dm_motor_set_pos_vel(id, position, velocity);
}
static bool chassis_rx_header_valid(const FDCAN_RxHeaderTypeDef* header) {
    return s_registry_ready && header != NULL && header->IdType == FDCAN_STANDARD_ID &&
        header->RxFrameType == FDCAN_DATA_FRAME && header->DataLength == FDCAN_DLC_BYTES_8;
}
static void chassis_steer_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header,
                                         const uint8_t data[8], void* user) {
    (void)hcan; (void)user;
    if(!chassis_rx_header_valid(header) || data == NULL) return;
    if(s_steer_rx_trace.count < 16u) {
        volatile ChassisRxFrame* trace = &s_steer_rx_trace.frames[s_steer_rx_trace.count];
        trace->id = header->Identifier;
        trace->stamp_ms = HAL_GetTick();
        for(uint8_t i = 0u; i < 8u; ++i) trace->data[i] = data[i];
        __DMB();
        ++s_steer_rx_trace.count;
    }
    chassis_queue_push(&s_steer_rx, header->Identifier, data);
}
static void chassis_drive_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header,
                                         const uint8_t data[8], void* user) {
    (void)hcan; (void)user;
    if(chassis_rx_header_valid(header) && data != NULL)
        chassis_queue_push(&s_drive_rx, header->Identifier, data);
}
static void chassis_reset_prepare(void) {
    memset(s_steer_prepare, 0, sizeof(s_steer_prepare));
    memset(s_drive_prepare, 0, sizeof(s_drive_prepare));
}
static void chassis_reset_transport(void) {
    const uint32_t irq_state = can_critical_enter();
    s_steer_rx.read = s_steer_rx.write;
    s_drive_rx.read = s_drive_rx.write;
    s_steer_rx.overflow = s_drive_rx.overflow = false;
    can_critical_exit(irq_state);
    s_receive_error = MOTOR_STATUS_OK;
    memset(s_feedback_seen, 0, sizeof(s_feedback_seen));
    chassis_reset_prepare();
}

void assemble_chassis_log_diagnostics(void) {
    BspCanDiagnostics steer = { 0 };
    BspCanDiagnostics drive = { 0 };
    can_refresh_diagnostics(&hfdcan1);
    can_refresh_diagnostics(&hfdcan2);
    if(can_get_diagnostics(&hfdcan1, &steer) != STM32_HAL_CAN_OK ||
       can_get_diagnostics(&hfdcan2, &drive) != STM32_HAL_CAN_OK)
        return;
    log_info("CAN RX: 1=%lu id=%lx; 2=%lu id=%lx",
             (unsigned long)steer.received_frames, (unsigned long)steer.last_rx_id,
             (unsigned long)drive.received_frames, (unsigned long)drive.last_rx_id);
    if(steer.tx_failure_count == 0u && drive.tx_failure_count == 0u &&
       steer.last_error_code == 0u && drive.last_error_code == 0u &&
       !steer.bus_off && !drive.bus_off)
        return;
    log_info("CAN diag: 1=%s id=%lx free=%lu lec=%lu tec=%lu rec=%lu bo=%u; 2=%s id=%lx free=%lu lec=%lu tec=%lu rec=%lu bo=%u",
             can_error_code_to_str(steer.last_failure), (unsigned long)steer.last_failure_id,
             (unsigned long)steer.tx_fifo_free, (unsigned long)steer.last_error_code,
             (unsigned long)steer.tx_error_count, (unsigned long)steer.rx_error_count, steer.bus_off ? 1u : 0u,
             can_error_code_to_str(drive.last_failure), (unsigned long)drive.last_failure_id,
             (unsigned long)drive.tx_fifo_free, (unsigned long)drive.last_error_code,
             (unsigned long)drive.tx_error_count, (unsigned long)drive.rx_error_count, drive.bus_off ? 1u : 0u);
}


static BusMotorStatus chassis_cancel_pending_commands(void) {
    chassis_reset_prepare();
    const BspCanStatus steer = can_cancel_tx(&hfdcan1);
    const BspCanStatus drive = can_cancel_tx(&hfdcan2);
    const BusMotorStatus first = chassis_can_status(steer);
    return first != MOTOR_STATUS_OK ? first : chassis_can_status(drive);
}
