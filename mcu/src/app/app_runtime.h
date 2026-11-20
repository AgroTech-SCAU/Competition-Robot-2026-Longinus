#ifndef _app_runtime_h_
#define _app_runtime_h_

/**
 * @file app_runtime.h
 * @brief MCU 应用运行时主控接口
 */

#include "app_fsm.h"

#include <stdbool.h>
#include <stdint.h>

enum {
    APP_PUMP_RELAY_1 = 1u << 0,
    APP_PUMP_RELAY_2 = 1u << 1,
    APP_PUMP_RELAY_3 = 1u << 2,
    APP_PUMP_RELAY_4 = 1u << 3,
};

typedef void (*AppPumpOutput)(bool enabled, uint8_t relay_mask);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化 MCU 应用运行时主控
 * @details 负责初始化系统级状态机, 控制映射模块和运行时缓存
 */
void app_runtime_init(void);

/** @brief Register the board output for pump enable and relays 1 through 4. */
void app_runtime_set_pump_output(AppPumpOutput output);

/**
 * @brief 执行一次 MCU 应用运行时轮询
 * @details 在 500Hz 主循环中统一完成输入更新, 模式仲裁, 安全检查和控制输出
 * @param local_key_pressed 本机启动键当前是否按下，消抖和边沿检测由运行时完成
 */
void app_runtime_process(bool local_key_pressed);

/**
 * @brief 获取当前系统级状态
 * @return AppFsmStateId 当前状态
 */
AppFsmStateId app_runtime_get_state(void);

/**
 * @brief 获取当前锁存故障
 * @return const AppFault* 故障只读指针
 */
const AppFault* app_runtime_get_fault(void);

/**
 * @brief 判断当前是否存在锁存故障
 * @return bool `true` 表示存在锁存故障
 */
bool app_runtime_has_fault(void);

/** @brief 初始化与设备自检已通过，不代表电机输出已经使能 */
bool app_runtime_is_self_check_passed(void);

#endif
