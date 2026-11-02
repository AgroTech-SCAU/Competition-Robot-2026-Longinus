#include "opticalflow.h"
#include "opticalflow/upixels_flow.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define OPTICALFLOW_RX_CAPACITY 256u

typedef struct {
    uint8_t byte;
    uint32_t timestamp_ms;
} OpticalflowRxByte;

static OpticalflowConfig s_config;
static OpticalflowState s_state;
static upixels_parser_t s_parser;
static volatile OpticalflowRxByte s_queue[OPTICALFLOW_RX_CAPACITY];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile bool s_restart_pending;
static volatile bool s_stream_broken;
static volatile uint32_t s_received_bytes;
static volatile uint32_t s_uart_errors;
static volatile uint32_t s_queue_overflows;
static uint8_t s_rx_byte;
static bool s_initialized;
static bool s_new_sample;
static float s_cos_yaw;
static float s_sin_yaw;
static bool s_tilt_reference_valid;
static float s_reference_roll;
static float s_reference_pitch;

OpticalflowConfig opticalflow_default_config(void) {
    OpticalflowConfig config = {
        .flow_sign = 1.0f,
        .min_height_m = 0.02f,
        .max_height_m = 4.0f,
        .max_speed_m_s = 5.0f,
        .max_tilt_rad = 0.35f,
        .sample_timeout_ms = 100u,
        .distance_timeout_ms = 100u,
    };
    return config;
}

bool opticalflow_init(const OpticalflowConfig* config) {
    if(config == NULL || config->ops == NULL ||
       config->ops->start_receive == NULL || config->ops->abort_receive == NULL ||
       config->ops->now_ms == NULL || config->ops->critical_enter == NULL ||
       config->ops->critical_exit == NULL || !isfinite(config->sensor_yaw_rad) ||
       !isfinite(config->sensor_x_m) || !isfinite(config->sensor_y_m) ||
       !isfinite(config->flow_sign) || !isfinite(config->fixed_height_m) ||
       config->fixed_height_m < 0.0f || !(config->min_height_m > 0.0f) ||
       !isfinite(config->max_height_m) || !(config->max_height_m > config->min_height_m) ||
       !isfinite(config->max_speed_m_s) || !(config->max_speed_m_s > 0.0f) ||
       !isfinite(config->max_tilt_rad) || !(config->max_tilt_rad > 0.0f) ||
       config->sample_timeout_ms == 0u || config->distance_timeout_ms == 0u ||
       (config->fixed_height_m > 0.0f && (config->fixed_height_m < config->min_height_m ||
                                        config->fixed_height_m > config->max_height_m))) {
        return false;
    }
    s_config = *config;
    s_config.flow_sign = config->flow_sign < 0.0f ? -1.0f : 1.0f;
    s_cos_yaw = cosf(config->sensor_yaw_rad);
    s_sin_yaw = sinf(config->sensor_yaw_rad);
    memset(&s_state, 0, sizeof(s_state));
    upixels_parser_init(&s_parser);
    s_head = s_tail = 0u;
    s_received_bytes = s_uart_errors = s_queue_overflows = 0u;
    s_stream_broken = false;
    s_new_sample = false;
    s_tilt_reference_valid = false;
    s_initialized = true;
    s_restart_pending = !s_config.ops->start_receive(s_config.context, &s_rx_byte, 1u);
    /* An absent module does not prevent the original odometry from booting. */
    return true;
}

void opticalflow_on_rx_complete(void) {
    if(!s_initialized) return;
    const uint16_t next = (uint16_t)((s_head + 1u) % OPTICALFLOW_RX_CAPACITY);
    ++s_received_bytes;
    if(next == s_tail) {
        ++s_queue_overflows;
        s_stream_broken = true;
    }
    else if(!s_stream_broken) {
        s_queue[s_head].byte = s_rx_byte;
        s_queue[s_head].timestamp_ms = s_config.ops->now_ms(s_config.context);
        s_head = next;
    }
    if(!s_config.ops->start_receive(s_config.context, &s_rx_byte, 1u)) {
        s_restart_pending = true;
        s_stream_broken = true;
    }
}

void opticalflow_on_rx_error(void) {
    if(!s_initialized) return;
    ++s_uart_errors;
    s_restart_pending = true;
    s_stream_broken = true;
}

static void opticalflow_handle_frame(const upixels_frame_t* frame, uint32_t now_ms) {
    float vx;
    float vy;
    float height = s_config.fixed_height_m;
    if(frame->event != UPIXELS_EVENT_FLOW) return;

    s_state.ready = false;
    s_new_sample = false;
    if(!(height > 0.0f)) {
        if(!s_parser.has_last_distance ||
           (uint32_t)(now_ms - s_parser.last_distance_timestamp_ms) > s_config.distance_timeout_ms) {
            ++s_state.invalid_samples;
            return;
        }
        height = s_parser.last_distance_m;
    }
    if((uint32_t)(now_ms - frame->timestamp_ms) > s_config.sample_timeout_ms ||
       !(frame->dt_s > 0.0f) || frame->dt_s > 0.1f ||
       height < s_config.min_height_m || height > s_config.max_height_m ||
       !upixels_frame_velocity(frame, height, &vx, &vy) ||
       !isfinite(vx) || !isfinite(vy) ||
       hypotf(vx, vy) > s_config.max_speed_m_s) {
        ++s_state.invalid_samples;
        return;
    }
    s_state.vx_sensor_m_s = vx;
    s_state.vy_sensor_m_s = vy;
    s_state.height_m = height;
    s_state.sample_timestamp_ms = frame->timestamp_ms;
    s_state.ready = true;
    s_new_sample = true;
}

void opticalflow_process(void) {
    upixels_frame_t frame;
    OpticalflowRxByte item;
    if(!s_initialized) return;
    uint32_t irq_state = s_config.ops->critical_enter(s_config.context);
    if(s_stream_broken) {
        s_tail = s_head;
        s_stream_broken = false;
        s_parser.length = 0u;
        s_parser.has_last_distance = false;
        s_state.ready = false;
        s_new_sample = false;
    }
    if(s_restart_pending) {
        (void)s_config.ops->abort_receive(s_config.context);
        s_restart_pending = !s_config.ops->start_receive(s_config.context, &s_rx_byte, 1u);
    }
    /* Bound work to the bytes already queued on entry. ISR can keep receiving. */
    const uint16_t end = s_head;
    const uint32_t now_ms = s_config.ops->now_ms(s_config.context);
    s_config.ops->critical_exit(s_config.context, irq_state);
    while(true) {
        irq_state = s_config.ops->critical_enter(s_config.context);
        if(s_tail == end || s_stream_broken) {
            s_config.ops->critical_exit(s_config.context, irq_state);
            break;
        }
        item.byte = s_queue[s_tail].byte;
        item.timestamp_ms = s_queue[s_tail].timestamp_ms;
        s_tail = (uint16_t)((s_tail + 1u) % OPTICALFLOW_RX_CAPACITY);
        s_config.ops->critical_exit(s_config.context, irq_state);
        if(upixels_parser_feed(&s_parser, item.byte, item.timestamp_ms, &frame)) {
            opticalflow_handle_frame(&frame, now_ms);
        }
        while(upixels_parser_poll(&s_parser, item.timestamp_ms, &frame)) {
            opticalflow_handle_frame(&frame, now_ms);
        }
    }
    if((uint32_t)(now_ms - s_state.sample_timestamp_ms) > s_config.sample_timeout_ms ||
       (!(s_config.fixed_height_m > 0.0f) &&
        (!s_parser.has_last_distance || (uint32_t)(now_ms - s_parser.last_distance_timestamp_ms) >
                                       s_config.distance_timeout_ms))) {
        s_state.ready = false;
        s_new_sample = false;
    }
    s_state.frames_ok = s_parser.frames_ok;
    s_state.crc_errors = s_parser.crc_errors;
    irq_state = s_config.ops->critical_enter(s_config.context);
    if(s_stream_broken) {
        s_state.ready = false;
        s_new_sample = false;
    }
    s_state.received_bytes = s_received_bytes;
    s_state.uart_errors = s_uart_errors;
    s_state.queue_overflows = s_queue_overflows;
    s_config.ops->critical_exit(s_config.context, irq_state);
}

ChassisImuOdomFlow opticalflow_take_sample(float roll_rad, float pitch_rad) {
    ChassisImuOdomFlow sample = { 0 };
    if(!s_initialized) return sample;
    /* Capture on the same first valid IMU cycle as chassis_update_imu_tilt(),
       even if no optical-flow frame has arrived yet. */
    if(!s_tilt_reference_valid && isfinite(roll_rad) && isfinite(pitch_rad)) {
        s_reference_roll = roll_rad;
        s_reference_pitch = pitch_rad;
        s_tilt_reference_valid = true;
    }
    if(!s_state.ready) return sample;
    const float vx = s_config.flow_sign * s_state.vx_sensor_m_s;
    const float vy = s_config.flow_sign * s_state.vy_sensor_m_s;
    sample.vx = s_cos_yaw * vx - s_sin_yaw * vy;
    sample.vy = s_sin_yaw * vx + s_cos_yaw * vy;
    sample.sensor_x_m = s_config.sensor_x_m;
    sample.sensor_y_m = s_config.sensor_y_m;
    sample.valid = isfinite(roll_rad) && isfinite(pitch_rad) &&
                   fabsf(roll_rad) <= s_config.max_tilt_rad &&
                   fabsf(pitch_rad) <= s_config.max_tilt_rad;
    if(sample.valid && s_tilt_reference_valid) {
        /* Match chassis.get_state()->cur_vx/vy: XY block of R0^T * R.
           This is the existing reference-plane convention, not new 3D flow compensation. */
        const float cp = cosf(pitch_rad - s_reference_pitch);
        const float sp = sinf(pitch_rad - s_reference_pitch);
        const float sr = sinf(roll_rad);
        const float sr0 = sinf(s_reference_roll);
        const float cr0 = cosf(s_reference_roll);
        const float xx = cp;
        const float xy = sp * sr;
        const float yx = -sr0 * sp;
        const float yy = cr0 * cosf(roll_rad) + sr0 * cp * sr;
        const float body_vx = sample.vx;
        const float body_vy = sample.vy;
        sample.vx = xx * body_vx + xy * body_vy;
        sample.vy = yx * body_vx + yy * body_vy;
        sample.sensor_x_m = xx * s_config.sensor_x_m + xy * s_config.sensor_y_m;
        sample.sensor_y_m = yx * s_config.sensor_x_m + yy * s_config.sensor_y_m;
    }
    sample.updated = s_new_sample && sample.valid;
    s_new_sample = false;
    return sample;
}

const OpticalflowState* opticalflow_get_state(void) {
    return &s_state;
}
