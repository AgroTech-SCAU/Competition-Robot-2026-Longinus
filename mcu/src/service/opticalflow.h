#ifndef SERVICE_OPTICALFLOW_H
#define SERVICE_OPTICALFLOW_H

#include <stdbool.h>
#include <stdint.h>
#include "kalman/chassis_imu_odom.h"

/* UART operations are supplied by assemble_odom; this service has no HAL dependency. */
typedef struct {
    bool (*start_receive)(void* context, uint8_t* data, uint16_t length);
    bool (*abort_receive)(void* context);
    uint32_t (*now_ms)(void* context);
    uint32_t (*critical_enter)(void* context);
    void (*critical_exit)(void* context, uint32_t state);
} OpticalflowPortOps;

typedef struct {
    const OpticalflowPortOps* ops;
    void* context;
    float sensor_yaw_rad;       /* sensor +X relative to body +X, CCW positive */
    float flow_sign;            /* +1 or -1, both axes */
    float sensor_x_m;           /* body origin -> sensor, forward positive */
    float sensor_y_m;           /* left positive */
    float fixed_height_m;       /* 0: require fresh TOF; >0: explicitly fixed height */
    float min_height_m;
    float max_height_m;
    float max_speed_m_s;
    float max_tilt_rad;         /* reject planar-model measurements beyond this tilt */
    uint32_t sample_timeout_ms;
    uint32_t distance_timeout_ms;
} OpticalflowConfig;

typedef struct {
    float vx_sensor_m_s;
    float vy_sensor_m_s;
    float height_m;
    uint32_t sample_timestamp_ms;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t invalid_samples;
    uint32_t received_bytes;
    uint32_t uart_errors;
    uint32_t queue_overflows;
    bool ready;
} OpticalflowState;

OpticalflowConfig opticalflow_default_config(void);
bool opticalflow_init(const OpticalflowConfig* config);
/* Called only from the selected UART's ISR; enqueue and rearm, no filter work. */
void opticalflow_on_rx_complete(void);
void opticalflow_on_rx_error(void);
/* Called once per existing 250Hz odom cycle, including IMU calibration cycles. */
void opticalflow_process(void);
/* Consume each new sample at most once; cached samples only veto false zero-speed. */
ChassisImuOdomFlow opticalflow_take_sample(float roll_rad, float pitch_rad);
const OpticalflowState* opticalflow_get_state(void);

#endif
