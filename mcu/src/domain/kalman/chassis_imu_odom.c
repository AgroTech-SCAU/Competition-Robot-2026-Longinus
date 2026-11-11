#include "chassis_imu_odom.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 底盘 + IMU 里程融合接口单例的文件内短别名
 */
#define cio chassis_imu_odom_interface

/**
 * @brief 圆周率常量
 */
#define CHASSIS_IMU_ODOM_PI 3.14159265358979323846f
/**
 * @brief 角度完整周期
 */
#define CHASSIS_IMU_ODOM_2PI (2.0f * CHASSIS_IMU_ODOM_PI)
/**
 * @brief 使用标量观测依次更新 vx、vy、wheel wz、IMU 航向角速度
 */
#define CHASSIS_IMU_ODOM_MEAS_DIM 1u

/**
 * @brief 底盘 + IMU 里程融合接口单例
 */
#define X(name, str) .name = CHASSIS_IMU_ODOM_##name,
const struct ChassisImuOdomInterface chassis_imu_odom_interface = {
    { CHASSIS_IMU_ODOM_STATUS_TABLE },
    .init = chassis_imu_odom_init,
    .update = chassis_imu_odom_update,
    .get_angle = chassis_imu_odom_get_angle,
    .get_odom = chassis_imu_odom_get_odom,
    .get_output = chassis_imu_odom_get_output,
    .reset_odom = chassis_imu_odom_reset_odom,
    .error_code_to_str = chassis_imu_odom_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将矩阵清零
 */
static void chassis_imu_odom_matrix_zero(Matrix* matrix);
/**
 * @brief 将矩阵置为单位阵；非方阵时只填充主对角线
 */
static void chassis_imu_odom_matrix_identity(Matrix* matrix);
/**
 * @brief 将角度归一化到 (-pi, pi]
 */
static float chassis_imu_odom_wrap_pi(float angle);
/**
 * @brief 获取有效噪声参数；输入无效时使用默认值
 */
static float chassis_imu_odom_noise(float value, float fallback);
/**
 * @brief 判断底盘和 IMU 是否处于静止窗口
 */
static bool chassis_imu_odom_is_static(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt);
static void chassis_imu_odom_reset_static(ChassisImuOdom* odom);
static void chassis_imu_odom_reset_static_detector(ChassisImuOdom* odom);
static bool chassis_imu_odom_vec_finite(Vector3 value);
static float chassis_imu_odom_vec_norm(Vector3 value);
static KalmanErrorCode chassis_imu_odom_observe(KalmanFilter* filter, uint8_t state, float value, float variance);
/**
 * @brief 将底层卡尔曼状态码映射到融合状态码
 */
static ChassisImuOdomErrorCode chassis_imu_odom_from_kalman(KalmanErrorCode status);
/**
 * @brief 将滤波器状态同步到输出缓存
 */
static void chassis_imu_odom_sync_output(ChassisImuOdom* odom);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化底盘 + IMU 里程融合实例
 * @param odom 融合实例
 * @param config 配置指针；NULL 表示使用默认配置
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_init(ChassisImuOdom* odom, const ChassisImuOdomConfig* config) {
    ChassisImuOdomConfig use_config;
    KalmanErrorCode status;

    if(odom == NULL) {
        return cio.INVALID_PARAM;
    }

    memset(odom, 0, sizeof(ChassisImuOdom));
    use_config = (config != NULL) ? *config : chassis_imu_odom_default_config();
    use_config.position_process_noise_x = chassis_imu_odom_noise(use_config.position_process_noise_x, 0.01f);
    use_config.position_process_noise_y = chassis_imu_odom_noise(use_config.position_process_noise_y, 0.01f);
    use_config.yaw_process_noise = chassis_imu_odom_noise(use_config.yaw_process_noise, 0.02f);
    use_config.velocity_process_noise_x = chassis_imu_odom_noise(use_config.velocity_process_noise_x, 0.1f);
    use_config.velocity_process_noise_y = chassis_imu_odom_noise(use_config.velocity_process_noise_y, 0.1f);
    use_config.velocity_process_noise_wz = chassis_imu_odom_noise(use_config.velocity_process_noise_wz, 0.1f);
    use_config.chassis_velocity_noise_x = chassis_imu_odom_noise(use_config.chassis_velocity_noise_x, 0.04f);
    use_config.chassis_velocity_noise_y = chassis_imu_odom_noise(use_config.chassis_velocity_noise_y, 0.04f);
    use_config.chassis_velocity_noise_wz = chassis_imu_odom_noise(use_config.chassis_velocity_noise_wz, 0.04f);
    use_config.imu_gyro_noise = chassis_imu_odom_noise(use_config.imu_gyro_noise, 0.12f);
    use_config.gravity = chassis_imu_odom_noise(use_config.gravity, 9.80665f);
    use_config.acc_norm_tolerance = chassis_imu_odom_noise(use_config.acc_norm_tolerance, 0.35f);
    use_config.chassis_linear_static_threshold = chassis_imu_odom_noise(use_config.chassis_linear_static_threshold, 0.003f);
    use_config.chassis_angular_static_threshold = chassis_imu_odom_noise(use_config.chassis_angular_static_threshold, 0.02f);
    use_config.imu_gyro_static_threshold = chassis_imu_odom_noise(use_config.imu_gyro_static_threshold, 0.020f);
    use_config.imu_gyro_mean_static_threshold = chassis_imu_odom_noise(use_config.imu_gyro_mean_static_threshold, 0.003f);
    use_config.imu_gyro_mean_time_constant_s = chassis_imu_odom_noise(use_config.imu_gyro_mean_time_constant_s, 0.1f);
    use_config.stationary_enter_time_s = chassis_imu_odom_noise(use_config.stationary_enter_time_s, 0.5f);
    use_config.stationary_exit_ratio = chassis_imu_odom_noise(use_config.stationary_exit_ratio, 1.5f);
    use_config.acc_change_threshold = chassis_imu_odom_noise(use_config.acc_change_threshold, 0.2f);
    if(use_config.stationary_exit_ratio < 1.0f) use_config.stationary_exit_ratio = 1.0f;

    status = kalman.filter_init(&odom->filter, CHASSIS_IMU_ODOM_STATE_DIM, CHASSIS_IMU_ODOM_MEAS_DIM, 0u);
    if(status != kalman.OK) {
        return chassis_imu_odom_from_kalman(status);
    }

    /* The odometry frame is defined at startup; only the initial speeds are uncertain. */
    for(uint8_t i = 0u; i < CHASSIS_IMU_ODOM_STATE_VX; ++i) {
        odom->filter.P_data[i * CHASSIS_IMU_ODOM_STATE_DIM + i] = 0.0f;
    }
    odom->config = use_config;
    odom->initialized = true;
    chassis_imu_odom_sync_output(odom);

    return cio.OK;
}

/**
 * @brief 使用底盘速度和 IMU 六轴数据更新一次融合状态
 *
 * 当前预测模型只积分平面 x/y 和 yaw；z 状态没有高度输入，因此保持初始化或重置值
 *
 * @param odom 融合实例
 * @param chassis 底盘速度输入
 * @param imu IMU 六轴输入
 * @param dt 更新周期，单位 s
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_update(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt) {
    if(odom == NULL) return cio.INVALID_PARAM;
    if(!odom->initialized || !odom->filter.initialized) return cio.NOT_INITIALIZE;
    if(!isfinite(dt) || dt <= 0.0f || dt > 0.05f) return cio.INVALID_PARAM;

    chassis.valid = chassis.valid && isfinite(chassis.vx) && isfinite(chassis.vy) && isfinite(chassis.wz);
    imu.gyro_valid = imu.gyro_valid && chassis_imu_odom_vec_finite(imu.gyro);
    imu.acc_valid = imu.acc_valid && chassis_imu_odom_vec_finite(imu.acc);
    imu.angle_valid = imu.angle_valid && isfinite(imu.angle.x) && isfinite(imu.angle.y);
    KalmanFilter* filter = &odom->filter;
    bool stationary = chassis_imu_odom_is_static(odom, chassis, imu, dt);
    if(stationary) {
        /* A confirmed zero-motion interval uses its own deterministic model:
         * retain the accumulated pose and its uncertainty, map velocities to
         * zero, and introduce no dynamic process noise or pseudo sensor data.
         * This is conditional on the stationary detector, not absolute pose
         * knowledge. The next nonstationary frame restores the dynamic model. */
        chassis_imu_odom_matrix_identity(&filter->F);
        chassis_imu_odom_matrix_zero(&filter->Q);
        for(uint8_t i = CHASSIS_IMU_ODOM_STATE_VX; i < CHASSIS_IMU_ODOM_STATE_DIM; ++i) {
            filter->x_data[i] = 0.0f;
            filter->F_data[i * filter->state_dim + i] = 0.0f;
            for(uint8_t j = 0u; j < CHASSIS_IMU_ODOM_STATE_DIM; ++j) {
                filter->P_data[i * filter->state_dim + j] = 0.0f;
                filter->P_data[j * filter->state_dim + i] = 0.0f;
            }
        }
        if(imu.angle_valid) {
            filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL] = imu.angle.x;
            filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH] = imu.angle.y;
        }
        /* H/R/z remain the last real observation; none is applied here. */
        chassis_imu_odom_sync_output(odom);
        return cio.OK;
    }

    const float vx = filter->x_data[CHASSIS_IMU_ODOM_STATE_VX];
    const float vy = filter->x_data[CHASSIS_IMU_ODOM_STATE_VY];
    const float wz = filter->x_data[CHASSIS_IMU_ODOM_STATE_WZ];
    const float yaw = filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW];
    const float midpoint_yaw = yaw + 0.5f * wz * dt;
    const float c = cosf(midpoint_yaw);
    const float s = sinf(midpoint_yaw);
    float predicted[CHASSIS_IMU_ODOM_STATE_DIM];
    memcpy(predicted, filter->x_data, sizeof(predicted));
    predicted[CHASSIS_IMU_ODOM_STATE_X] += (c * vx - s * vy) * dt;
    predicted[CHASSIS_IMU_ODOM_STATE_Y] += (s * vx + c * vy) * dt;
    predicted[CHASSIS_IMU_ODOM_STATE_YAW] += wz * dt;
    if(imu.angle_valid) {
        predicted[CHASSIS_IMU_ODOM_STATE_ROLL] = imu.angle.x;
        predicted[CHASSIS_IMU_ODOM_STATE_PITCH] = imu.angle.y;
    }

    /* F is the Jacobian of the nonlinear midpoint prediction, never F*x. */
    chassis_imu_odom_matrix_identity(&filter->F);
    chassis_imu_odom_matrix_zero(&filter->Q);
    const uint8_t n = filter->state_dim;
    const float dx_dyaw = (-s * vx - c * vy) * dt;
    const float dy_dyaw = ( c * vx - s * vy) * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_X * n + CHASSIS_IMU_ODOM_STATE_YAW] = dx_dyaw;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_Y * n + CHASSIS_IMU_ODOM_STATE_YAW] = dy_dyaw;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_X * n + CHASSIS_IMU_ODOM_STATE_VX] = c * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_X * n + CHASSIS_IMU_ODOM_STATE_VY] = -s * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_Y * n + CHASSIS_IMU_ODOM_STATE_VX] = s * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_Y * n + CHASSIS_IMU_ODOM_STATE_VY] = c * dt;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_X * n + CHASSIS_IMU_ODOM_STATE_WZ] = 0.5f * dt * dx_dyaw;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_Y * n + CHASSIS_IMU_ODOM_STATE_WZ] = 0.5f * dt * dy_dyaw;
    filter->F_data[CHASSIS_IMU_ODOM_STATE_YAW * n + CHASSIS_IMU_ODOM_STATE_WZ] = dt;
    filter->Q_data[CHASSIS_IMU_ODOM_STATE_X * n + CHASSIS_IMU_ODOM_STATE_X] = odom->config.position_process_noise_x * dt;
    filter->Q_data[CHASSIS_IMU_ODOM_STATE_Y * n + CHASSIS_IMU_ODOM_STATE_Y] = odom->config.position_process_noise_y * dt;
    filter->Q_data[CHASSIS_IMU_ODOM_STATE_YAW * n + CHASSIS_IMU_ODOM_STATE_YAW] = odom->config.yaw_process_noise * dt;
    filter->Q_data[CHASSIS_IMU_ODOM_STATE_VX * n + CHASSIS_IMU_ODOM_STATE_VX] = odom->config.velocity_process_noise_x * dt;
    filter->Q_data[CHASSIS_IMU_ODOM_STATE_VY * n + CHASSIS_IMU_ODOM_STATE_VY] = odom->config.velocity_process_noise_y * dt;
    filter->Q_data[CHASSIS_IMU_ODOM_STATE_WZ * n + CHASSIS_IMU_ODOM_STATE_WZ] = odom->config.velocity_process_noise_wz * dt;
    KalmanErrorCode status = kalman.filter_predict_state(filter, predicted);
    if(status != kalman.OK) return chassis_imu_odom_from_kalman(status);

    /* Independent diagonal-R observations are equivalent to one joint update.
     * Scalar updates avoid inverting nearly dependent wheel/gyro wz rows. */
    if(chassis.valid && chassis.new_sample) {
        status = chassis_imu_odom_observe(filter, CHASSIS_IMU_ODOM_STATE_VX, chassis.vx, odom->config.chassis_velocity_noise_x);
        if(status != kalman.OK) return chassis_imu_odom_from_kalman(status);
        status = chassis_imu_odom_observe(filter, CHASSIS_IMU_ODOM_STATE_VY, chassis.vy, odom->config.chassis_velocity_noise_y);
        if(status != kalman.OK) return chassis_imu_odom_from_kalman(status);
        status = chassis_imu_odom_observe(filter, CHASSIS_IMU_ODOM_STATE_WZ, chassis.wz, odom->config.chassis_velocity_noise_wz);
        if(status != kalman.OK) return chassis_imu_odom_from_kalman(status);
    }
    if(imu.gyro_valid && imu.gyro_new_sample && imu.angle_valid) {
        /* Body q/r are not Euler yaw rate on a slope. The planar model is not
         * used near the Euler singularity. R assumes equal y/z gyro variance. */
        const float cp = cosf(imu.angle.y);
        if(fabsf(cp) > 0.2f) {
            const float yaw_rate = (imu.gyro.y * sinf(imu.angle.x) + imu.gyro.z * cosf(imu.angle.x)) / cp;
            status = chassis_imu_odom_observe(filter, CHASSIS_IMU_ODOM_STATE_WZ, yaw_rate, odom->config.imu_gyro_noise / (cp * cp));
            if(status != kalman.OK) return chassis_imu_odom_from_kalman(status);
        }
    }
    filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW] = chassis_imu_odom_wrap_pi(filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW]);
    chassis_imu_odom_sync_output(odom);
    return cio.OK;
}

void chassis_imu_odom_invalidate(ChassisImuOdom* odom) {
    if(odom == NULL || !odom->initialized) return;
    chassis_imu_odom_reset_static_detector(odom);
    for(uint8_t i = CHASSIS_IMU_ODOM_STATE_VX; i < CHASSIS_IMU_ODOM_STATE_DIM; ++i) {
        odom->filter.x_data[i] = 0.0f;
        for(uint8_t j = 0u; j < CHASSIS_IMU_ODOM_STATE_DIM; ++j) {
            odom->filter.P_data[i * CHASSIS_IMU_ODOM_STATE_DIM + j] = 0.0f;
            odom->filter.P_data[j * CHASSIS_IMU_ODOM_STATE_DIM + i] = 0.0f;
        }
        odom->filter.P_data[i * CHASSIS_IMU_ODOM_STATE_DIM + i] = 1.0f;
    }
    chassis_imu_odom_sync_output(odom);
    odom->output.velocity.valid = false;
}

/**
 * @brief 获取当前三轴姿态角
 * @param odom 融合实例
 * @param angle 输出三轴角度
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_angle(const ChassisImuOdom* odom, Vector3* angle) {
    if(odom == NULL || angle == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    *angle = odom->output.angle;
    return cio.OK;
}

/**
 * @brief 获取当前三轴里程
 * @param odom 融合实例
 * @param odom_out 输出三轴里程
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_odom(const ChassisImuOdom* odom, Vector3* odom_out) {
    if(odom == NULL || odom_out == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    *odom_out = odom->output.odom;
    return cio.OK;
}

/**
 * @brief 获取完整融合输出
 * @param odom 融合实例
 * @param output 输出缓存
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_get_output(const ChassisImuOdom* odom, ChassisImuOdomOutput* output) {
    if(odom == NULL || output == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    *output = odom->output;
    return cio.OK;
}

/**
 * @brief 重置三轴里程状态
 * @param odom 融合实例
 * @param odom_value 新的里程值
 * @return ChassisImuOdomErrorCode 状态码
 */
ChassisImuOdomErrorCode chassis_imu_odom_reset_odom(ChassisImuOdom* odom, Vector3 odom_value) {
    if(odom == NULL) {
        return cio.INVALID_PARAM;
    }
    if(!odom->initialized) {
        return cio.NOT_INITIALIZE;
    }

    odom->filter.x_data[CHASSIS_IMU_ODOM_STATE_X] = odom_value.x;
    odom->filter.x_data[CHASSIS_IMU_ODOM_STATE_Y] = odom_value.y;
    odom->filter.x_data[CHASSIS_IMU_ODOM_STATE_Z] = odom_value.z;
    chassis_imu_odom_sync_output(odom);

    return cio.OK;
}

/**
 * @brief 将融合状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码说明
 */
#define X(name, str)              \
    case CHASSIS_IMU_ODOM_##name: \
        return str;
const char* chassis_imu_odom_error_code_to_str(ChassisImuOdomErrorCode status) {
    switch(status) {
        CHASSIS_IMU_ODOM_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将矩阵清零
 */
static void chassis_imu_odom_matrix_zero(Matrix* matrix) {
    unsigned int i;

    if(matrix == NULL || matrix->pdata == NULL) {
        return;
    }

    for(i = 0u; i < matrix->row * matrix->col; ++i) {
        matrix->pdata[i] = 0.0f;
    }
}

/**
 * @brief 将矩阵置为单位阵；非方阵时只填充主对角线
 */
static void chassis_imu_odom_matrix_identity(Matrix* matrix) {
    unsigned int i;
    unsigned int j;

    chassis_imu_odom_matrix_zero(matrix);
    if(matrix == NULL || matrix->pdata == NULL) {
        return;
    }

    for(i = 0u; i < matrix->row; ++i) {
        for(j = 0u; j < matrix->col; ++j) {
            matrix->pdata[i * matrix->col + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 将角度归一化到 (-pi, pi]
 * @param angle 输入角度，单位 rad
 * @return float 归一化后的角度，单位 rad
 */
static float chassis_imu_odom_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, CHASSIS_IMU_ODOM_2PI);
    if(angle > CHASSIS_IMU_ODOM_PI) {
        angle -= CHASSIS_IMU_ODOM_2PI;
    }
    else if(angle <= -CHASSIS_IMU_ODOM_PI) {
        angle += CHASSIS_IMU_ODOM_2PI;
    }

    return angle;
}

/**
 * @brief 获取有效噪声参数；输入无效时使用默认值
 * @param value 输入配置值
 * @param fallback 默认值
 * @return float 最终使用的配置值
 */
static float chassis_imu_odom_noise(float value, float fallback) {
    return (value > 0.0f && isfinite(value)) ? value : fallback;
}

/**
 * @brief 获取默认融合配置
 * @return ChassisImuOdomConfig 默认配置
 */
ChassisImuOdomConfig chassis_imu_odom_default_config(void) {
    ChassisImuOdomConfig config = {
        .position_process_noise_x = 0.01f,
        .position_process_noise_y = 0.01f,
        .yaw_process_noise = 0.02f,
        .velocity_process_noise_x = 0.1f,
        .velocity_process_noise_y = 0.1f,
        .velocity_process_noise_wz = 0.1f,
        .chassis_velocity_noise_x = 0.04f,
        .chassis_velocity_noise_y = 0.04f,
        .chassis_velocity_noise_wz = 0.04f,
        .imu_gyro_noise = 0.12f,
        .gravity = 9.80665f,
        .acc_norm_tolerance = 0.35f,
        .chassis_linear_static_threshold = 0.003f,
        .chassis_angular_static_threshold = 0.02f,
        .imu_gyro_static_threshold = 0.020f,
        .imu_gyro_mean_static_threshold = 0.003f,
        .imu_gyro_mean_time_constant_s = 0.1f,
        .stationary_enter_time_s = 0.5f,
        .stationary_exit_ratio = 1.5f,
        .acc_change_threshold = 0.2f,
    };
    return config;
}

static bool chassis_imu_odom_vec_finite(Vector3 value) {
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static float chassis_imu_odom_vec_norm(Vector3 value) {
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

static void chassis_imu_odom_reset_static(ChassisImuOdom* odom) {
    odom->stationary = false;
    odom->stationary_candidate_s = 0.0f;
    odom->stationary_acc_samples = 0u;
    odom->stationary_acc_m2 = 0.0f;
    odom->stationary_acc_variance = 0.0f;
    odom->stationary_acc_weight_s = 0.0f;
    odom->stationary_acc_elapsed_s = 0.0f;
    odom->stationary_acc_mean = (Vector3){ 0.0f, 0.0f, 0.0f };
}

static void chassis_imu_odom_reset_static_detector(ChassisImuOdom* odom) {
    chassis_imu_odom_reset_static(odom);
    odom->stationary_gyro_mean = (Vector3){ 0.0f, 0.0f, 0.0f };
    odom->stationary_gyro_elapsed_s = 0.0f;
    odom->stationary_gyro_mean_initialized = false;
}

static bool chassis_imu_odom_is_static(ChassisImuOdom* odom, ChassisImuOdomChassis chassis, ChassisImuOdomImu imu, float dt) {
    const ChassisImuOdomConfig* config = &odom->config;
    const float ratio = odom->stationary ? config->stationary_exit_ratio : 1.0f;
    if(!chassis.stationary_allowed || !chassis.valid || !imu.gyro_valid || !imu.acc_valid) {
        chassis_imu_odom_reset_static_detector(odom);
        return false;
    }
    odom->stationary_gyro_elapsed_s += dt;
    if(imu.gyro_new_sample) {
        if(!odom->stationary_gyro_mean_initialized) {
            odom->stationary_gyro_mean = imu.gyro;
            odom->stationary_gyro_mean_initialized = true;
        }
        else {
            const float alpha = 1.0f - expf(-odom->stationary_gyro_elapsed_s / config->imu_gyro_mean_time_constant_s);
            odom->stationary_gyro_mean.x += alpha * (imu.gyro.x - odom->stationary_gyro_mean.x);
            odom->stationary_gyro_mean.y += alpha * (imu.gyro.y - odom->stationary_gyro_mean.y);
            odom->stationary_gyro_mean.z += alpha * (imu.gyro.z - odom->stationary_gyro_mean.z);
        }
        odom->stationary_gyro_elapsed_s = 0.0f;
    }
    /* Keep the mean across ordinary motion/noise gate failures so it can settle.
     * A sustained slow turn must fail the same mean threshold even after entry;
     * only instantaneous gyro, wheel, and acceleration gates use hysteresis. */
    if(!odom->stationary_gyro_mean_initialized ||
       chassis_imu_odom_vec_norm(odom->stationary_gyro_mean) > config->imu_gyro_mean_static_threshold ||
       hypotf(chassis.vx, chassis.vy) > config->chassis_linear_static_threshold * ratio ||
       fabsf(chassis.wz) > config->chassis_angular_static_threshold * ratio ||
       chassis_imu_odom_vec_norm(imu.gyro) > config->imu_gyro_static_threshold * ratio ||
       fabsf(chassis_imu_odom_vec_norm(imu.acc) - config->gravity) > config->acc_norm_tolerance * ratio) {
        chassis_imu_odom_reset_static(odom);
        return false;
    }
    odom->stationary_candidate_s += dt;
    odom->stationary_acc_elapsed_s += dt;
    if(imu.acc_new_sample) {
        Vector3 delta = {
            imu.acc.x - odom->stationary_acc_mean.x,
            imu.acc.y - odom->stationary_acc_mean.y,
            imu.acc.z - odom->stationary_acc_mean.z,
        };
        const float sample_dt = odom->stationary_acc_elapsed_s;
        odom->stationary_acc_elapsed_s = 0.0f;
        float alpha;
        if(odom->stationary) {
            /* Once accepted, track a duration-weighted rolling variance. */
            alpha = 1.0f - expf(-sample_dt / config->stationary_enter_time_s);
        }
        else {
            odom->stationary_acc_weight_s += sample_dt;
            alpha = sample_dt / odom->stationary_acc_weight_s;
            ++odom->stationary_acc_samples;
        }
        odom->stationary_acc_mean.x += alpha * delta.x;
        odom->stationary_acc_mean.y += alpha * delta.y;
        odom->stationary_acc_mean.z += alpha * delta.z;
        const float delta_squared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
        if(odom->stationary) {
            odom->stationary_acc_variance = (1.0f - alpha) * (odom->stationary_acc_variance + alpha * delta_squared);
        }
        else {
            odom->stationary_acc_m2 += sample_dt * (1.0f - alpha) * delta_squared;
            odom->stationary_acc_variance = odom->stationary_acc_m2 / odom->stationary_acc_weight_s;
        }
        const float variance_limit = config->acc_change_threshold * ratio;
        if(odom->stationary_acc_variance > variance_limit * variance_limit) {
            chassis_imu_odom_reset_static(odom);
            return false;
        }
    }
    if(odom->stationary_acc_samples >= 3u && odom->stationary_candidate_s >= config->stationary_enter_time_s) {
        odom->stationary = true;
    }
    return odom->stationary;
}

static KalmanErrorCode chassis_imu_odom_observe(KalmanFilter* filter, uint8_t state, float value, float variance) {
    chassis_imu_odom_matrix_zero(&filter->H);
    filter->H_data[state] = 1.0f;
    filter->z_data[0] = value;
    filter->R_data[0] = variance;
    return kalman.filter_update(filter);
}

/**
 * @brief 将底层卡尔曼状态码映射到融合状态码
 * @param status 卡尔曼状态码
 * @return ChassisImuOdomErrorCode 融合状态码
 */
static ChassisImuOdomErrorCode chassis_imu_odom_from_kalman(KalmanErrorCode status) {
    if(status == kalman.OK) {
        return cio.OK;
    }
    if(status == kalman.INVALID_PARAM || status == kalman.INVALID_DIM) {
        return cio.INVALID_PARAM;
    }
    if(status == kalman.NOT_INITIALIZE) {
        return cio.NOT_INITIALIZE;
    }

    return cio.KALMAN_FAILED;
}

/**
 * @brief 将滤波器状态同步到输出缓存
 * @param odom 融合实例
 */
static void chassis_imu_odom_sync_output(ChassisImuOdom* odom) {
    KalmanFilter* filter;

    if(odom == NULL || !odom->initialized) {
        return;
    }

    filter = &odom->filter;
    odom->output.odom.x = filter->x_data[CHASSIS_IMU_ODOM_STATE_X];
    odom->output.odom.y = filter->x_data[CHASSIS_IMU_ODOM_STATE_Y];
    odom->output.odom.z = filter->x_data[CHASSIS_IMU_ODOM_STATE_Z];
    odom->output.angle.x = filter->x_data[CHASSIS_IMU_ODOM_STATE_ROLL];
    odom->output.angle.y = filter->x_data[CHASSIS_IMU_ODOM_STATE_PITCH];
    odom->output.angle.z = filter->x_data[CHASSIS_IMU_ODOM_STATE_YAW];
    odom->output.velocity.vx = filter->x_data[CHASSIS_IMU_ODOM_STATE_VX];
    odom->output.velocity.vy = filter->x_data[CHASSIS_IMU_ODOM_STATE_VY];
    odom->output.velocity.wz = filter->x_data[CHASSIS_IMU_ODOM_STATE_WZ];
    odom->output.velocity.valid = true;
    odom->output.velocity.new_sample = true;
    odom->output.stationary = odom->stationary;
}
