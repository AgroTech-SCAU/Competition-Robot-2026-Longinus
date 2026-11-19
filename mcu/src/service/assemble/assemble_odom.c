#include "assemble.h"

#include "log.h"
#include "odom.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_odom(void) {
    OdomConfig config = odom.default_config();
    config.wheel_only = true;

    if(odom.init(&config) != odom.OK) {
        log_error("ODOM service init failed");
        return SYSTEM_STATUS_ERROR;
    }

    log_info("ODOM wheel-only, waiting for four steering and four drive feedbacks");
    return SYSTEM_STATUS_OK;
}
