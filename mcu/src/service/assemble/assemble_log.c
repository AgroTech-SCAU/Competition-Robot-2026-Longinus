#include "assemble.h"

#include "app_odom_test.h"
#include "chassis.h"
#include "delay.h"
#include "laser_odom.h"
#include "log.h"
#include "odom.h"
#include "stm32_hal_uart.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

static bool s_nonblocking_requested;
static uint8_t s_plot_tx[LOG_BUFFER_SIZE];

/* Callers are in the main loop. The UART IRQ only drains the TX buffer. */
static bool log_uart_write(const char* data, uint32_t len) {
    if(s_nonblocking_requested) {
        if(huart1.gState != HAL_UART_STATE_READY || len > sizeof(s_plot_tx)) {
            return false;
        }
        memcpy(s_plot_tx, data, len);
        return HAL_UART_Transmit_IT(&huart1, s_plot_tx, (uint16_t)len) == HAL_OK;
    }

    /* Preserve the existing text log path and give it priority over a plot frame. */
    if(huart1.gState == HAL_UART_STATE_BUSY_TX) {
        if(HAL_UART_AbortTransmit(&huart1) != HAL_OK) return false;
        (void)uart1_write_blocking("\r\n", 2u);
    }
    return uart1_write_blocking(data, len);
}

static const LogPortOps log_ops = {
    .write = log_uart_write,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = false,
    };

    if(log_init(&log_config) != LOG_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    return SYSTEM_STATUS_OK;
}

void assemble_log_tuning(float rem) {
    if(!odom.is_ready()) return;

    const Odom* estimate = odom.get_state();
    const SteerWheelState* wheel = chassis.get_state();
    const SteerWheelControl* target = chassis.get_control();
    const bool active = chassis.is_ready();
    const ms_t ms = delay_now_ms();
    const float x = estimate->odom.x;
    const float y = estimate->odom.y;
    const float yaw = estimate->angle.z;
    const float vx_cmd = active ? target->vx : 0.0f;
    const float vy_cmd = active ? target->vy : 0.0f;
    const float vx_fk = wheel->cur_vx;
    const float vy_fk = wheel->cur_vy;
    const float wz_fk = wheel->cur_wz;
    const float gz = estimate->gyro_corrected.z;
    const float vx = estimate->velocity.x;
    const float vy = estimate->velocity.y;
    const float wz = estimate->velocity.z;

    s_nonblocking_requested = true;
    (void)log_vofa(ms, x, y, yaw, vx_cmd, vy_cmd, vx_fk, vy_fk, wz_fk, gz, vx, vy, wz, rem, active);
    s_nonblocking_requested = false;
}

void assemble_log_laser(void) {
    const LaserOdomSample* sample = laser_odom_get_sample();
    if(!laser_odom_is_online()) return;

    const AppLaserFix fix = app_odom_test_get_laser_fix();
    s_nonblocking_requested = true;
    log_info("LASER POS seg=%u frame=%lu F=%u B=%u L=%u R=%u mm x=%.3f y=%.3f m fix=0x%X",
             fix.segment, (unsigned long)sample->received,
             (unsigned int)sample->distance_mm[LASER_FRONT], (unsigned int)sample->distance_mm[LASER_REAR],
             (unsigned int)sample->distance_mm[LASER_LEFT], (unsigned int)sample->distance_mm[LASER_RIGHT],
             (double)((fix.mask & APP_LASER_X) ? fix.x : NAN),
             (double)((fix.mask & APP_LASER_Y) ? fix.y : NAN), fix.mask);
    s_nonblocking_requested = false;
}
