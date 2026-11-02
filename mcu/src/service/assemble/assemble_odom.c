#include "assemble.h"

#include "log.h"
#include "odom.h"
#include "opticalflow.h"
#include "stm32_hal_dwt.h"
#include "stm32_hal_uart.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

#define ATLAS_ODOM_YAW_RATE_SCALE 0.988f

static uint32_t odom_now_us(void* context) {
    (void)context;
    return dwt_get_us();
}

static bool flow_start_receive(void* context, uint8_t* data, uint16_t length) {
    return uart_receive_it((UART_HandleTypeDef*)context, data, length);
}

static bool flow_abort_receive(void* context) {
    return uart_abort_receive((UART_HandleTypeDef*)context);
}

static uint32_t flow_now_ms(void* context) {
    (void)context;
    return HAL_GetTick();
}

static const OpticalflowPortOps flow_ops = {
    .start_receive = flow_start_receive,
    .abort_receive = flow_abort_receive,
    .now_ms = flow_now_ms,
    .critical_enter = uart_critical_enter,
    .critical_exit = uart_critical_exit,
};

SystemStatus assemble_odom(void) {
    OdomConfig config = odom.default_config();
    OpticalflowConfig flow_config = opticalflow_default_config();

    config.yaw_rate_scale = ATLAS_ODOM_YAW_RATE_SCALE;
    config.now_us = odom_now_us;

    if(odom.init(&config) != odom.OK) {
        log_error("ODOM service init failed");
        return SYSTEM_STATUS_ERROR;
    }

    /* UART7: PE7 RX <- module TX, PE8 TX -> module RX, 115200 8N1. */
    flow_config.ops = &flow_ops;
    flow_config.context = &huart7;
    flow_config.sensor_yaw_rad = 0.0f; /* sensor +X forward, +Y left: verify on hardware */
    flow_config.flow_sign = 1.0f;
    flow_config.sensor_x_m = 0.0f;    /* user installation: bottom centre */
    flow_config.sensor_y_m = 0.0f;
    flow_config.fixed_height_m = 0.0f; /* require fresh measured height */

    const uint32_t irq_state = uart_critical_enter(NULL);
    uart_register_rx_complete_callback(&huart7, opticalflow_on_rx_complete);
    uart_register_error_callback(&huart7, opticalflow_on_rx_error);
    const bool flow_ok = opticalflow_init(&flow_config);
    uart_critical_exit(NULL, irq_state);
    if(!flow_ok) {
        log_error("ODOM optical flow config invalid");
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ODOM service init ok: optical flow UART7, centre mount");
    return SYSTEM_STATUS_OK;
}
