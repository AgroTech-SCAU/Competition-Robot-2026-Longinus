#include "assemble.h"

#include "log.h"
#include "odom.h"
#include "stm32_hal_dwt.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

#define ATLAS_ODOM_YAW_RATE_SCALE 0.988f

static uint32_t odom_now_us(void* context) {
    (void)context;
    return dwt_get_us();
}

SystemStatus assemble_odom(void) {
    OdomConfig config = odom.default_config();

    config.yaw_rate_scale = ATLAS_ODOM_YAW_RATE_SCALE;
    config.now_us = odom_now_us;

    if(odom.init(&config) != odom.OK) {
        log_error("ODOM service init failed");
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ODOM service init ok");
    return SYSTEM_STATUS_OK;
}
