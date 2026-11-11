#include "odom.h"

#include "chassis.h"
#include "imu.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 里程计服务接口单例的文件内短别名
 */
#define od odom_interface

/**
 * @brief 默认里程计服务周期
 *
 * 当前由 250Hz 任务调用，周期为 0.004s
 */
#define ODOM_DEFAULT_PROCESS_PERIOD_S 0.004f
/**
 * @brief 里程计服务周期上限
 *
 * 超过该间隔时标记数据不连续，不将缺失区间伪装成一个正常周期
 */
#define ODOM_MAX_PROCESS_DT_S 0.05f
#define ODOM_IMU_MAX_AGE_US 20000u
#define ODOM_TEMPERATURE_MAX_AGE_US 2000000u

/**
 * @brief 里程计服务对外状态缓存
 */
static Odom s_odom = { 0 };
/**
 * @brief 底盘 + IMU 融合算法实例
 */
static ChassisImuOdom s_fusion = { 0 };
/**
 * @brief 当前里程计服务配置
 */
static OdomConfig s_config = { 0 };
/**
 * @brief 上次 IMU 样本时间戳，单位微秒
 */
static uint32_t s_last_gyro_timestamp_us = 0u;
static bool s_gyro_timestamp_seen = false;
static uint32_t s_last_wheel_sequence;
static uint32_t s_last_acc_timestamp_us;
static bool s_wheel_sequence_seen;
static bool s_acc_timestamp_seen;
static bool s_yaw_integral_seeded;
static float s_previous_wheel_yaw_rate;
static float s_previous_imu_yaw_rate;

/**
 * @brief 里程计服务接口单例
 */
#define X(name, str) .name = ODOM_##name,
const struct OdomInterface odom_interface = {
    { ODOM_STATUS_TABLE },
    .default_config = odom_default_config,
    .init = odom_init,
    .process = odom_process,
    .get_acc = odom_get_acc,
    .get_gyro = odom_get_gyro,
    .get_gyro_bias = odom_get_gyro_bias,
    .get_gyro_corrected = odom_get_gyro_corrected,
    .get_angle = odom_get_angle,
    .get_odom = odom_get_odom,
    .get_velocity = odom_get_velocity,
    .get_state = odom_get_state,
    .is_ready = odom_is_ready,
    .status_str = odom_status_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将 IMU 加速度结构转换为通用 Vector3
 */
static Vector3 odom_acc_to_vec3(ImuAcc acc);
/**
 * @brief 将 IMU 角速度结构转换为通用 Vector3
 */
static Vector3 odom_gyro_to_vec3(ImuGyro gyro);
/**
 * @brief 将 IMU 姿态角结构转换为通用 Vector3
 */
static Vector3 odom_angle_to_vec3(ImuAngle angle);
/**
 * @brief 检查数据是否可用并拷贝 Vector3 输出
 */
static OdomStatus odom_copy_vec3(bool ready, const Vector3* src, Vector3* out);
/**
 * @brief 解析 IMU 样本时间戳，计算本次里程计服务周期
 */
static bool odom_resolve_process_dt_s(const ImuSample* sample, float* dt_s);
static bool odom_finite_vec3(Vector3 value);
static void odom_invalidate(void);
static void odom_integrate_yaw_inputs(const ChassisImuOdomChassis* wheel,
                                     const ChassisImuOdomImu* imu_input, float dt_s);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取默认里程计配置
 * @return OdomConfig 默认配置
 */
OdomConfig odom_default_config(void) {
    OdomConfig config = {
        .wheel_yaw_rate_scale = 1.0f,
        .imu_yaw_rate_scale = 1.0f,
        .process_period_s = ODOM_DEFAULT_PROCESS_PERIOD_S,
    };
    config.fusion = chassis_imu_odom_default_config();
    return config;
}

/**
 * @brief 初始化里程计服务
 * @param config 配置指针；NULL 表示使用默认配置
 * @return OdomStatus 状态码
 */
OdomStatus odom_init(const OdomConfig* config) {
    ChassisImuOdomErrorCode status;

    memset(&s_odom, 0, sizeof(s_odom));
    memset(&s_fusion, 0, sizeof(s_fusion));
    s_config = (config != NULL) ? *config : odom_default_config();
    s_last_gyro_timestamp_us = 0u;
    s_last_wheel_sequence = 0u;
    s_last_acc_timestamp_us = 0u;
    s_gyro_timestamp_seen = false;
    s_wheel_sequence_seen = false;
    s_acc_timestamp_seen = false;
    s_yaw_integral_seeded = false;
    if(!isfinite(s_config.process_period_s) || s_config.process_period_s <= 0.0f ||
       s_config.process_period_s > ODOM_MAX_PROCESS_DT_S) {
        s_config.process_period_s = ODOM_DEFAULT_PROCESS_PERIOD_S;
    }
    if(!isfinite(s_config.wheel_yaw_rate_scale) || s_config.wheel_yaw_rate_scale <= 0.0f) {
        s_config.wheel_yaw_rate_scale = 1.0f;
    }
    if(!isfinite(s_config.imu_yaw_rate_scale) || s_config.imu_yaw_rate_scale <= 0.0f) {
        s_config.imu_yaw_rate_scale = 1.0f;
    }

    status = chassis_imu_odom.init(&s_fusion, &s_config.fusion);
    if(status != chassis_imu_odom.OK) {
        return od.FUSION_FAILED;
    }

    s_odom.initialized = true;
    return od.OK;
}

/**
 * @brief 执行一次里程计服务流程
 *
 * 该函数统一刷新 IMU，读取底盘速度，并更新底盘 + IMU 融合里程
 * 上层任务不再直接调用 imu.update()
 *
 * @return OdomStatus 状态码
 */
OdomStatus odom_process(void) {
    ImuSample sample = { 0 };
    ChassisObservation wheel = { 0 };
    ChassisImuOdomChassis wheel_input = { 0 };
    ChassisImuOdomImu imu_input = { 0 };
    const bool was_stationary = s_odom.static_window;
    float dt_s = 0.0f;

    if(!s_odom.initialized) return od.NOT_INITIALIZED;
    s_odom.imu_ready = false;
    s_odom.fusion_ready = false;
    s_odom.static_window = false;
    s_odom.timing_valid = false;
    s_odom.acc_valid = false;
    s_odom.temperature_valid = false;
    s_odom.wheel_valid = chassis_get_observation(&wheel);
    s_odom.wheel_age_ms = wheel.age_ms;
    s_odom.wheel_sample_sequence = wheel.sample_sequence;
    s_odom.wheel_raw = (Vector3){ wheel.raw_velocity.vx, wheel.raw_velocity.vy, wheel.raw_velocity.wz };
    s_odom.wheel_corrected = (Vector3){ wheel.velocity.vx, wheel.velocity.vy, wheel.velocity.wz };
    s_odom.wheel_fusion = s_odom.wheel_corrected;
    s_odom.wheel_fusion.z *= s_config.wheel_yaw_rate_scale;

    const SteerWheelControl* command = chassis.get_control();
    const bool command_stopped = command != NULL && command->vx == 0.0f &&
        command->vy == 0.0f && command->wz == 0.0f;
    const bool wheel_stopped = wheel.valid &&
        fabsf(wheel.velocity.vx) <= s_fusion.config.chassis_linear_static_threshold &&
        fabsf(wheel.velocity.vy) <= s_fusion.config.chassis_linear_static_threshold &&
        fabsf(wheel.velocity.wz) <= s_fusion.config.chassis_angular_static_threshold;
    /* Startup still needs the IMU's complete statistical calibration window.
       After calibration only the fusion's confirmed stationary state may learn bias. */
    const bool boot_candidate = command_stopped && (wheel.valid ? wheel_stopped :
        !chassis_is_output_enabled());
    (void)imu_set_stationary(imu_is_calibrated()
        ? command_stopped && wheel_stopped && was_stationary : boot_candidate);

    if(imu.update() != IMU_STATUS_OK || imu_get_sample(&sample) != IMU_STATUS_OK) {
        odom_invalidate();
        return od.NOT_READY;
    }
    const uint32_t now_us = s_config.now_us != NULL
        ? s_config.now_us(s_config.clock_context) : sample.gyro_timestamp_us;
    s_odom.sample_time_us = sample.gyro_timestamp_us;
    s_odom.acc = odom_acc_to_vec3(sample.acc);
    s_odom.gyro = odom_gyro_to_vec3(sample.gyro);
    s_odom.gyro_bias = odom_gyro_to_vec3(imu_get_gyro_bias());
    s_odom.gyro_corrected = odom_gyro_to_vec3(imu_get_gyro_corrected());
    s_odom.gyro_bias_effective = (Vector3){
        s_odom.gyro.x - s_odom.gyro_corrected.x,
        s_odom.gyro.y - s_odom.gyro_corrected.y,
        s_odom.gyro.z - s_odom.gyro_corrected.z,
    };
    s_odom.imu_calibrated = imu_is_calibrated();
    s_odom.acc_valid = (sample.flags & IMU_SAMPLE_ACC_VALID) != 0u &&
        (uint32_t)(now_us - sample.acc_timestamp_us) <= ODOM_IMU_MAX_AGE_US &&
        odom_finite_vec3(s_odom.acc);
    s_odom.temperature_c = sample.temperature;
    s_odom.temperature_valid = (sample.flags & IMU_SAMPLE_TEMP_VALID) != 0u &&
        (uint32_t)(now_us - sample.temp_timestamp_us) <= ODOM_TEMPERATURE_MAX_AGE_US &&
        isfinite(sample.temperature);
    const bool gyro_valid = (sample.flags & (IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_GYRO_NEW)) ==
        (IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_GYRO_NEW) &&
        (uint32_t)(now_us - sample.gyro_timestamp_us) <= ODOM_IMU_MAX_AGE_US &&
        odom_finite_vec3(s_odom.gyro) && odom_finite_vec3(s_odom.gyro_corrected);
    s_odom.imu_ready = gyro_valid && s_odom.imu_calibrated;
    if(!s_odom.imu_ready) {
        /* Do not clear the startup candidate while calibration is accumulating. */
        chassis_imu_odom_invalidate(&s_fusion);
        chassis_update_imu_tilt(NAN, NAN);
        if(!gyro_valid) (void)imu_set_stationary(false);
        s_gyro_timestamp_seen = false;
        s_yaw_integral_seeded = false;
        return od.NOT_READY;
    }
    s_odom.timing_valid = odom_resolve_process_dt_s(&sample, &dt_s);
    s_odom.process_dt_s = dt_s;
    if(!s_odom.timing_valid) {
        ++s_odom.discontinuity_count;
        odom_invalidate();
        return od.NOT_READY;
    }

    imu_input.acc = s_odom.acc;
    imu_input.gyro = s_odom.gyro_corrected;
    imu_input.gyro.z *= s_config.imu_yaw_rate_scale;
    s_odom.gyro_yaw_fusion = imu_input.gyro.z;
    imu_input.angle = odom_angle_to_vec3(imu.get_angle());
    imu_input.gyro_valid = true;
    imu_input.gyro_new_sample = true;
    imu_input.acc_valid = s_odom.acc_valid;
    imu_input.acc_new_sample = s_odom.acc_valid && (sample.flags & IMU_SAMPLE_ACC_NEW) != 0u &&
        (!s_acc_timestamp_seen || sample.acc_timestamp_us != s_last_acc_timestamp_us);
    if(imu_input.acc_new_sample) {
        s_last_acc_timestamp_us = sample.acc_timestamp_us;
        s_acc_timestamp_seen = true;
    }
    /* Mahony retains a valid attitude while briefly coasting without accelerometer correction. */
    imu_input.angle_valid = odom_finite_vec3(imu_input.angle);
    if(imu_input.angle_valid && s_odom.acc_valid) {
        chassis_update_imu_tilt(imu_input.angle.x, imu_input.angle.y);
    }
    else {
        chassis_update_imu_tilt(NAN, NAN);
    }
    wheel_input.vx = s_odom.wheel_fusion.x;
    wheel_input.vy = s_odom.wheel_fusion.y;
    wheel_input.wz = s_odom.wheel_fusion.z;
    wheel_input.valid = wheel.valid && odom_finite_vec3(s_odom.wheel_fusion);
    wheel_input.stationary_allowed = command_stopped;
    if(!wheel_input.valid) {
        /* No position source remains. Preserve the last pose, invalidate the
           velocity prior and require a fresh observation before integrating. */
        odom_invalidate();
        return od.NOT_READY;
    }
    wheel_input.new_sample = wheel_input.valid &&
        (!s_wheel_sequence_seen || wheel.sample_sequence != s_last_wheel_sequence);
    if(chassis_imu_odom.update(&s_fusion, wheel_input, imu_input, dt_s) != chassis_imu_odom.OK) {
        odom_invalidate();
        return od.FUSION_FAILED;
    }
    odom_integrate_yaw_inputs(&wheel_input, &imu_input, dt_s);
    if(wheel_input.new_sample) {
        s_last_wheel_sequence = wheel.sample_sequence;
        s_wheel_sequence_seen = true;
    }
    s_odom.angle = s_fusion.output.angle;
    (void)chassis_imu_odom.get_odom(&s_fusion, &s_odom.odom);
    s_odom.velocity = (Vector3){ s_fusion.output.velocity.vx,
        s_fusion.output.velocity.vy, s_fusion.output.velocity.wz };
    s_odom.static_window = s_fusion.output.stationary && command_stopped;
    (void)imu_set_stationary(s_odom.static_window);
    s_odom.fusion_ready = imu_input.angle_valid;
    return s_odom.fusion_ready ? od.OK : od.NOT_READY;
}

/**
 * @brief 获取最近一次三轴加速度
 * @param acc 输出加速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_acc(Vector3* acc) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.acc, acc);
}

/**
 * @brief 获取最近一次原始三轴角速度
 * @param gyro 输出角速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro(Vector3* gyro) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro, gyro);
}

/**
 * @brief 获取最近一次陀螺零偏估计
 * @param gyro_bias 输出零偏
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro_bias(Vector3* gyro_bias) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro_bias, gyro_bias);
}

/**
 * @brief 获取最近一次修正后三轴角速度
 * @param gyro_corrected 输出修正角速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_gyro_corrected(Vector3* gyro_corrected) {
    return odom_copy_vec3(s_odom.imu_ready, &s_odom.gyro_corrected, gyro_corrected);
}

/**
 * @brief 获取融合后的三轴姿态角
 * @param angle 输出姿态角
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_angle(Vector3* angle) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.angle, angle);
}

/**
 * @brief 获取融合后的三轴里程
 * @param odom_out 输出里程
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_odom(Vector3* odom_out) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.odom, odom_out);
}

/**
 * @brief 获取融合后的底盘速度
 * @details x/y/z 分别表示 base_link 坐标系下的 vx/vy/wz
 * @param velocity_out 输出速度
 * @return OdomStatus 状态码
 */
OdomStatus odom_get_velocity(Vector3* velocity_out) {
    return odom_copy_vec3(s_odom.fusion_ready, &s_odom.velocity, velocity_out);
}

/**
 * @brief 获取里程计服务只读状态快照
 * @return const Odom* 状态快照指针
 */
const Odom* odom_get_state(void) {
    return &s_odom;
}

/**
 * @brief 判断里程计服务是否已有可用数据
 * @return true 数据可用
 * @return false 数据尚未可用
 */
bool odom_is_ready(void) {
    return s_odom.initialized && s_odom.imu_ready && s_odom.fusion_ready;
}

/**
 * @brief 将里程计服务状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
#define X(name, str)  \
    case ODOM_##name: \
        return str;
const char* odom_status_str(OdomStatus status) {
    switch(status) {
        ODOM_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将 IMU 加速度结构转换为通用 Vector3
 * @param acc IMU 加速度
 * @return Vector3 通用三轴向量
 */
static Vector3 odom_acc_to_vec3(ImuAcc acc) {
    Vector3 out = {
        .x = acc.x,
        .y = acc.y,
        .z = acc.z,
    };

    return out;
}

/**
 * @brief 将 IMU 角速度结构转换为通用 Vector3
 * @param gyro IMU 角速度
 * @return Vector3 通用三轴向量
 */
static Vector3 odom_gyro_to_vec3(ImuGyro gyro) {
    Vector3 out = {
        .x = gyro.x,
        .y = gyro.y,
        .z = gyro.z,
    };

    return out;
}

/**
 * @brief 将 IMU 姿态角结构转换为通用 Vector3
 * @param angle IMU 姿态角
 * @return Vector3 x/y/z 分别为 roll/pitch/yaw
 */
static Vector3 odom_angle_to_vec3(ImuAngle angle) {
    Vector3 out = {
        .x = angle.roll,
        .y = angle.pitch,
        .z = angle.yaw,
    };

    return out;
}

/**
 * @brief 检查数据是否可用并拷贝 Vector3 输出
 * @param ready 对应数据是否已经更新成功
 * @param src 数据源
 * @param out 输出缓存
 * @return OdomStatus 状态码
 */
static OdomStatus odom_copy_vec3(bool ready, const Vector3* src, Vector3* out) {
    if(src == NULL || out == NULL) {
        return od.INVALID_PARAM;
    }
    if(!s_odom.initialized) {
        return od.NOT_INITIALIZED;
    }
    if(!ready) {
        return od.NOT_READY;
    }

    *out = *src;
    return od.OK;
}

/**
 * @brief 解析 IMU 样本时间戳，计算本次里程计服务周期
 * @param sample IMU 样本指针；NULL 表示使用 fallback 时间戳
 * @return float 里程计服务周期，单位秒
 */
static bool odom_resolve_process_dt_s(const ImuSample* sample, float* dt_s) {
    *dt_s = s_config.process_period_s;
    if(s_gyro_timestamp_seen) {
        *dt_s = (float)(uint32_t)(sample->gyro_timestamp_us - s_last_gyro_timestamp_us) * 1.0e-6f;
    }
    s_last_gyro_timestamp_us = sample->gyro_timestamp_us;
    s_gyro_timestamp_seen = true;
    return isfinite(*dt_s) && *dt_s > 0.0f && *dt_s <= ODOM_MAX_PROCESS_DT_S;
}

static bool odom_finite_vec3(Vector3 value) {
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

/* Diagnostics use the real, scaled inputs even while fusion holds a stationary
   pose. A missing endpoint breaks the trapezoid; retained totals never bridge it. */
static void odom_integrate_yaw_inputs(const ChassisImuOdomChassis* wheel,
                                     const ChassisImuOdomImu* imu_input, float dt_s) {
    const float cp = cosf(imu_input->angle.y);
    if(!imu_input->angle_valid || !(fabsf(cp) > 0.2f)) {
        s_yaw_integral_seeded = false;
        return;
    }
    const float yaw_rate = (imu_input->gyro.y * sinf(imu_input->angle.x) +
                            imu_input->gyro.z * cosf(imu_input->angle.x)) / cp;
    if(!isfinite(yaw_rate)) {
        s_yaw_integral_seeded = false;
        return;
    }
    if(s_yaw_integral_seeded) {
        s_odom.wheel_yaw_integral_rad +=
            0.5 * ((double)s_previous_wheel_yaw_rate + wheel->wz) * dt_s;
        s_odom.imu_yaw_integral_rad +=
            0.5 * ((double)s_previous_imu_yaw_rate + yaw_rate) * dt_s;
        s_odom.yaw_integrated_time_s += dt_s;
        ++s_odom.yaw_integrated_samples;
    }
    s_previous_wheel_yaw_rate = wheel->wz;
    s_previous_imu_yaw_rate = yaw_rate;
    s_yaw_integral_seeded = true;
}

static void odom_invalidate(void) {
    s_odom.fusion_ready = false;
    s_odom.static_window = false;
    (void)imu_set_stationary(false);
    chassis_update_imu_tilt(NAN, NAN);
    chassis_imu_odom_invalidate(&s_fusion);
    s_odom.velocity = (Vector3){ 0.0f, 0.0f, 0.0f };
    s_wheel_sequence_seen = false;
    s_yaw_integral_seeded = false;
}
