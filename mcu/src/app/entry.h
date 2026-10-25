#ifndef _app_entry_h_
#define _app_entry_h_

/**
 * @file entry.h
 * @brief CubeMX `main.c` 的应用入口替代层
 */

#include "app_runtime.h"
#include "app_status.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "remote.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 系统初始化完成标记
 */
static bool init_ok = false;
static bool s_entry_log_ready = false;
static const char* s_entry_failure_stage = "delay";
static ms_t s_entry_failure_log_ms = 0u;

/**
 * @brief 250Hz 节拍计数器
 */
static uint8_t s_entry_250hz_tick = 0u;

/**
 * @brief 100Hz 节拍计数器
 */
static uint8_t s_entry_100hz_tick = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 判断当前 500Hz 周期是否命中 250Hz 节拍
 * @return bool `true` 表示应执行 250Hz 任务
 */
static inline bool entry_tick_250hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 100Hz 节拍
 * @return bool `true` 表示应执行 100Hz 任务
 */
static inline bool entry_tick_100hz(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 系统初始化入口
 * @details 按照启动依赖顺序完成底层装配, 应用层初始化, 最后启动 500Hz 定时调度
 */
static inline bool entry_boot_check(SystemStatus status, const char* stage) {
    if(status == SYSTEM_STATUS_OK) return true;
    s_entry_failure_stage = stage;
    if(s_entry_log_ready) log_error("BOOT failed at %s", stage);
    return false;
}

static inline bool entry_chassis_settle(ms_t interval_ms) {
    const ms_t start = delay_now_ms();
    while((ms_t)(delay_now_ms() - start) < interval_ms) {
        delay_ms(1u);
        if(chassis_poll_feedback() != CHASSIS_OK) {
            (void)entry_boot_check(SYSTEM_STATUS_ERROR, "chassis feedback");
            return false;
        }
    }
    return true;
}

static inline void entry_init(void) {
    if(!entry_boot_check(assemble_delay(), "delay"))
        return;

    if(!entry_boot_check(assemble_log(), "log"))
        return;
    s_entry_log_ready = true;
    log_info("BOOT log ready");
    delay_ms(100u);

    if(!entry_boot_check(assemble_rgb(), "rgb"))
        return;
    log_info("BOOT rgb ready");
    delay_ms(100u);

    if(!entry_boot_check(assemble_imu(), "imu"))
        return;
    log_info("BOOT imu ready");
    delay_ms(100u);

    if(!entry_boot_check(assemble_odom(), "odom"))
        return;
    log_info("BOOT odom ready");
    delay_ms(100u);

    if(!entry_boot_check(assemble_chassis(), "chassis"))
        return;
    log_info("BOOT chassis ready");
    if(!entry_chassis_settle(100u)) return;

    if(!entry_boot_check(assemble_remote(), "remote"))
        return;
    log_info("BOOT remote ready");
    if(!entry_chassis_settle(100u)) return;

    {
        remote_init();

        app_runtime_init();
        app_status_init();
    }
    log_info("BOOT app ready");
    if(!entry_chassis_settle(100u)) return;

    if(!entry_boot_check(assemble_tim6_500hz(), "tim6_500hz"))
        return;
    log_info("BOOT tim6 500hz ready");
    if(!entry_chassis_settle(100u)) return;

    if(!entry_chassis_settle(500u)) return;
    tim6_500hz_flag = false;
    init_ok = true;
    log_info("System initialized successfully");
}

/**
 * @brief 主循环调度入口
 * @details 100Hz 遥控输入和 250Hz 里程计更新后，500Hz 运行时先检查安全再执行底盘输出
 */
static inline void entry_loop(void) {
    if(!init_ok) {
        if(s_entry_log_ready && delay_nb_ms(&s_entry_failure_log_ms, 1000u)) {
            log_error("BOOT incomplete: stage=%s", s_entry_failure_stage);
        }
        return;
    }
    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;
        if(entry_tick_250hz()) {
            odom.process();
        }
        if(entry_tick_100hz()) {
            remote_process();
        }
        app_runtime_process();
    }
    app_status_process();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static inline bool entry_tick_250hz(void) {
    s_entry_250hz_tick = (uint8_t)((s_entry_250hz_tick + 1u) % 2u);
    return s_entry_250hz_tick == 0u;
}

static inline bool entry_tick_100hz(void) {
    s_entry_100hz_tick = (uint8_t)((s_entry_100hz_tick + 1u) % 5u);
    return s_entry_100hz_tick == 0u;
}

#endif
