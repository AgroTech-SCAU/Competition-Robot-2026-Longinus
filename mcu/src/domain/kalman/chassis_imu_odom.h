#ifndef _chassis_imu_odom_h_
#define _chassis_imu_odom_h_

#include <stdbool.h>

#include "kalman.h"
#include "matrix.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘 + IMU 里程融合接口单例别名
 */
#define chassis_imu_odom chassis_imu_odom_interface

/**
 * @brief 底盘 + IMU 里程融合状态码表
 *
 * @param OK 操作成功
 * @param INVALID_PARAM 输入参数无效
 * @param KALMAN_FAILED 底层卡尔曼滤波运算失败
 * @param NOT_INITIALIZE 实例尚未初始化
 */
#define CHASSIS_IMU_ODOM_STATUS_TABLE     \
    X(OK, "OK")                           \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(KALMAN_FAILED, "Kalman Failed")     \
    X(NOT_INITIALIZE, "Not Initialize")

#define X(name, str) CHASSIS_IMU_ODOM_##name,
/**
 * @brief 底盘 + IMU 里程融合错误码
 */
typedef enum {
    CHASSIS_IMU_ODOM_STATUS_TABLE
} ChassisImuOdomErrorCode;
#undef X

/**
 * @brief 融合滤波器状态向量索引
 *
 * 当前模型用于平面底盘里程计：
 * - X/Y 由底盘速度按 yaw 积分得到
 * - Z 为预留状态，当前没有高度观测或预测输入，因此保持初始化/重置值
 * - roll/pitch 使用上游姿态解算结果，不重复进行重力融合
 * - yaw 由 wz 积分并用底盘/IMU z 轴角速度约束
 */
typedef enum {
    CHASSIS_IMU_ODOM_STATE_X = 0,
    CHASSIS_IMU_ODOM_STATE_Y,
    CHASSIS_IMU_ODOM_STATE_Z,
    CHASSIS_IMU_ODOM_STATE_ROLL,
    CHASSIS_IMU_ODOM_STATE_PITCH,
    CHASSIS_IMU_ODOM_STATE_YAW,
    CHASSIS_IMU_ODOM_STATE_VX,
    CHASSIS_IMU_ODOM_STATE_VY,
    CHASSIS_IMU_ODOM_STATE_WZ,
    CHASSIS_IMU_ODOM_STATE_DIM
} ChassisImuOdomStateIndex;

/**
 * @brief 底盘平面速度输入
 */
typedef struct {
    float vx; /**< 底盘 x 方向线速度，单位 m/s */
    float vy; /**< 底盘 y 方向线速度，单位 m/s */
    float wz; /**< 底盘 z 轴角速度，单位 rad/s */
    bool valid; /**< 输入：反馈有效且未超龄；输出：估计已更新 */
    bool new_sample; /**< 输入：本次包含未被融合过的轮速样本 */
    bool stationary_allowed; /**< 输入：控制命令允许进行静止确认；不影响正常轮速融合 */
} ChassisImuOdomChassis;

/**
 * @brief IMU 六轴输入
 */
typedef struct {
    Vector3 acc;  /**< 三轴加速度，单位 m/s^2 */
    Vector3 gyro; /**< 三轴角速度，单位 rad/s */
    Vector3 angle; /**< 上游姿态角，x/y 为 roll/pitch；不采用其 yaw */
    bool gyro_valid; /**< gyro 有效且未超龄 */
    bool acc_valid; /**< acc 有效且未超龄 */
    bool angle_valid; /**< 上游 roll/pitch 有效 */
    bool gyro_new_sample; /**< 本次包含未被融合过的陀螺样本 */
    bool acc_new_sample; /**< 本次包含未被统计过的加速度样本 */
} ChassisImuOdomImu;

/**
 * @brief 融合输出
 */
typedef struct {
    Vector3 angle; /**< 三轴姿态角，x/y/z 分别为 roll/pitch/yaw，单位 rad */
    Vector3 odom;  /**< 三轴里程，x/y 单位 m；z 当前为预留高度状态 */
    ChassisImuOdomChassis velocity; /**< 融合后的底盘速度估计 */
    bool stationary; /**< 有效轮速、三轴陀螺及加速度持续满足静止条件 */
} ChassisImuOdomOutput;

/**
 * @brief 融合滤波器噪声配置
 */
typedef struct {
    float position_process_noise_x; /**< x 随机游走方差率，m²/s */
    float position_process_noise_y; /**< y 随机游走方差率，m²/s */
    float yaw_process_noise;        /**< yaw 随机游走方差率，rad²/s */
    float velocity_process_noise_x; /**< vx 随机游走方差率，(m/s)²/s */
    float velocity_process_noise_y; /**< vy 随机游走方差率，(m/s)²/s */
    float velocity_process_noise_wz; /**< wz 随机游走方差率，(rad/s)²/s */
    float chassis_velocity_noise_x; /**< 轮速 vx 观测方差，(m/s)² */
    float chassis_velocity_noise_y; /**< 轮速 vy 观测方差，(m/s)² */
    float chassis_velocity_noise_wz; /**< 轮速 wz 观测方差，(rad/s)² */
    float imu_gyro_noise;         /**< IMU 航向角速度观测方差，(rad/s)² */
    float gravity;                /**< 静止重力加速度参考值，单位 m/s^2 */
    float acc_norm_tolerance;     /**< 静止检测 |acc模长-g| 上限，m/s² */
    float chassis_linear_static_threshold;  /**< 静止判定的底盘线速度阈值，单位 m/s */
    float chassis_angular_static_threshold; /**< 静止判定的底盘角速度阈值，单位 rad/s */
    float imu_gyro_static_threshold;        /**< 瞬时三轴陀螺模长上限，rad/s；容忍静止噪声峰值 */
    float imu_gyro_mean_static_threshold;   /**< 三轴陀螺低通均值模长上限，rad/s；进入和退出共用 */
    float imu_gyro_mean_time_constant_s;    /**< 静止判定陀螺均值低通时间常数，s */
    float stationary_enter_time_s; /**< 连续静止候选时长，s */
    float stationary_exit_ratio; /**< 瞬时 gyro、轮速和 acc 的退出倍率，>=1；不放宽 gyro 均值门限 */
    float acc_change_threshold; /**< 静止窗三轴加速度合成 RMS 变化上限，m/s² */
} ChassisImuOdomConfig;

/**
 * @brief 底盘 + IMU 里程融合实例
 */
typedef struct {
    KalmanFilter filter; /**< 底层通用卡尔曼滤波器 */
    ChassisImuOdomConfig config; /**< 当前融合参数 */
    ChassisImuOdomOutput output; /**< 最近一次融合输出 */
    Vector3 stationary_acc_mean;
    float stationary_acc_m2;
    float stationary_candidate_s;
    float stationary_acc_variance;
    float stationary_acc_weight_s;
    float stationary_acc_elapsed_s;
    uint32_t stationary_acc_samples;
    Vector3 stationary_gyro_mean;
    float stationary_gyro_elapsed_s;
    bool stationary_gyro_mean_initialized;
    bool stationary;
    bool initialized; /**< true 表示实例已初始化 */
} ChassisImuOdom;

/**
 * @brief 底盘 + IMU 里程融合接口表
 */
#define X(name, str) ChassisImuOdomErrorCode name;
extern const struct ChassisImuOdomInterface {
    struct {
        CHASSIS_IMU_ODOM_STATUS_TABLE
    };
    /**
     * @brief 初始化融合实例
     * @param odom 融合实例
     * @param config 配置指针；NULL 表示使用默认配置
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*init)(ChassisImuOdom* odom, const ChassisImuOdomConfig* config);
    /**
     * @brief 使用一帧底盘速度和 IMU 数据更新里程融合
     * @param odom 融合实例
     * @param chassis 底盘速度输入
     * @param imu IMU 六轴输入
     * @param dt 更新周期，单位 s
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*update)(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
    /**
     * @brief 获取当前三轴姿态角
     * @param odom 融合实例
     * @param angle 输出三轴角度
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*get_angle)(const ChassisImuOdom* odom, Vector3* angle);
    /**
     * @brief 获取当前三轴里程
     * @param odom 融合实例
     * @param odom_out 输出三轴里程
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*get_odom)(const ChassisImuOdom* odom, Vector3* odom_out);
    /**
     * @brief 获取完整融合输出
     * @param odom 融合实例
     * @param output 输出缓存
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*get_output)(const ChassisImuOdom* odom, ChassisImuOdomOutput* output);
    /**
     * @brief 重置三轴里程状态
     * @param odom 融合实例
     * @param odom_value 新的里程值
     * @return ChassisImuOdomErrorCode 状态码
     */
    ChassisImuOdomErrorCode (*reset_odom)(ChassisImuOdom* odom, Vector3 odom_value);
    /**
     * @brief 将状态码转换为静态字符串
     * @param status 状态码
     * @return const char* 状态码说明
     */
    const char* (*error_code_to_str)(ChassisImuOdomErrorCode status);
} chassis_imu_odom_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/** @brief 获取按物理量拆分的默认配置；噪声数值需以实测方差标定 */
ChassisImuOdomConfig chassis_imu_odom_default_config(void);
/** @brief 显式数据中断：取消静止确认并重置速度，保留累计位置与航向 */
void chassis_imu_odom_invalidate(ChassisImuOdom* odom);

/**
 * @brief 初始化融合实例
 * @param odom 融合实例
 * @param config 配置指针；NULL 表示使用默认配置
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_init(ChassisImuOdom* odom, const ChassisImuOdomConfig* config);
/**
 * @brief 使用一帧底盘速度和 IMU 数据更新里程融合
 * @param odom 融合实例
 * @param chassis 底盘速度输入
 * @param imu IMU 六轴输入
 * @param dt 更新周期，单位 s
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_update(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
/**
 * @brief 获取当前三轴姿态角
 * @param odom 融合实例
 * @param angle 输出三轴角度
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_angle(const ChassisImuOdom* odom, Vector3* angle);
/**
 * @brief 获取当前三轴里程
 * @param odom 融合实例
 * @param odom_out 输出三轴里程
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_odom(const ChassisImuOdom* odom, Vector3* odom_out);
/**
 * @brief 获取完整融合输出
 * @param odom 融合实例
 * @param output 输出缓存
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_output(const ChassisImuOdom* odom, ChassisImuOdomOutput* output);
/**
 * @brief 重置三轴里程状态
 * @param odom 融合实例
 * @param odom_value 新的里程值
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_reset_odom(ChassisImuOdom* odom, Vector3 odom_value);
/**
 * @brief 将状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
const char* chassis_imu_odom_error_code_to_str(ChassisImuOdomErrorCode status);

#endif
