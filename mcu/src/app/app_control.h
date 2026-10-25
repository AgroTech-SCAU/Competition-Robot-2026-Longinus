#ifndef _app_control_h_
#define _app_control_h_

/**
 * @file app_control.h
 * @brief MCU 控制映射与安全执行接口
 */

// ! ========================= 类 型 声 明 ========================= ! //

/**
 * @brief 控制执行结果
 * @details 供 `app_runtime` 判断当前周期的控制输出是正常执行, 被安全策略跳过, 还是执行失败
 */
typedef enum {
    APP_CONTROL_RESULT_OK = 0,
    APP_CONTROL_RESULT_SKIPPED,
    APP_CONTROL_RESULT_REJECTED,
    APP_CONTROL_RESULT_CHASSIS_ERROR,
    APP_CONTROL_RESULT_ODOM_ERROR,
    APP_CONTROL_RESULT_COMMAND_INVALID,
    APP_CONTROL_RESULT_UNSUPPORTED
} AppControlResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化控制映射模块
 * @details 在 `app_runtime_init()` 中调用一次, 完成 yaw hold 等辅助控制器初始化
 */
void app_control_init(void);

/**
 * @brief 执行 FS-iA10B 手动底盘控制
 */
AppControlResult app_control_apply_manual_chassis(void);

/**
 * @brief 请求底盘进入驻车刹车状态
 */
AppControlResult app_control_brake_chassis(void);

/**
 * @brief 清除航向保持和遥控目标缓存，不访问电机
 */
void app_control_reset_chassis(void);

/**
 * @brief 仅设置零速目标，由本周期 chassis.process() 保持当前舵角并执行
 */
AppControlResult app_control_hold_zero_chassis(void);

/**
 * @brief 清零目标并立即请求电机停止
 * @details 与驻车刹车和硬件急停分别处理
 */
AppControlResult app_control_stop_all(void);

#endif
