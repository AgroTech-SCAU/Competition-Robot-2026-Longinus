#include "assemble.h"

#include "log.h"
#include "odom.h"
#include "stm32_hal_dwt.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

static uint32_t odom_now_us(void* context) {
    (void)context;
    return dwt_get_us();
}

SystemStatus assemble_odom(void) {
    OdomConfig config = odom.default_config();

    /* Keep wheel geometry and gyro scale independent; calibrate each against
       measured motion instead of applying a shared empirical 0.988 factor. */
    config.wheel_yaw_rate_scale = 1.0f;
    config.imu_yaw_rate_scale = 1.0f;
    /* Conservative warm-static gyro variance; wheel noise retains its relative
       prior and Q retains the response. These are not motion-calibrated values. */
    config.fusion.imu_gyro_noise = 3.0e-5f;
    config.fusion.chassis_velocity_noise_wz = 1.0e-5f;
    config.fusion.velocity_process_noise_wz = 2.5e-5f;
    /* Warm, enabled steering loops produce about 0.26-0.33 m/s^2 of
       accelerometer RMS noise; retain the sustained slow-turn threshold. */
    config.fusion.acc_change_threshold = 0.35f;
    config.fusion.acc_norm_tolerance = 0.55f;
    config.fusion.imu_gyro_mean_time_constant_s = 0.20f;
    config.now_us = odom_now_us;

    if(odom.init(&config) != odom.OK) {
        log_error("ODOM service init failed");
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ODOM service init ok");
    return SYSTEM_STATUS_OK;
}
