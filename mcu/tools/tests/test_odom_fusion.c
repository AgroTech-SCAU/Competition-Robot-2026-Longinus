#include "chassis_imu_odom.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const float dt = 0.004f;

static ChassisImuOdomImu imu_sample(void) {
    return (ChassisImuOdomImu){
        .acc = { 0.0f, 0.0f, 9.80665f },
        .gyro_valid = true, .acc_valid = true, .angle_valid = true,
        .gyro_new_sample = true, .acc_new_sample = true,
    };
}

static ChassisImuOdomChassis wheel_sample(void) {
    return (ChassisImuOdomChassis){ .valid = true, .new_sample = true, .stationary_allowed = true };
}

static void step(ChassisImuOdom* odom, ChassisImuOdomChassis wheel, ChassisImuOdomImu imu) {
    assert(chassis_imu_odom_update(odom, wheel, imu, dt) == CHASSIS_IMU_ODOM_OK);
}

static void test_small_variance(void) {
    KalmanFilter filter;
    assert(kalman_filter_init(&filter, 1u, 1u, 0u) == KALMAN_OK);
    filter.P_data[0] = 1e-10f;
    filter.H_data[0] = 1.0f;
    filter.R_data[0] = 1e-10f;
    filter.z_data[0] = 1.0f;
    assert(kalman_filter_update(&filter) == KALMAN_OK);
    assert(fabsf(filter.x_data[0] - 0.5f) < 1e-6f);
    assert(fabsf(filter.P_data[0] - 5e-11f) < 1e-16f);

    ChassisImuOdom odom;
    ChassisImuOdomConfig config = chassis_imu_odom_default_config();
    config.imu_gyro_noise = 1e-10f;
    config.chassis_velocity_noise_wz = 2e-10f;
    assert(chassis_imu_odom_init(&odom, &config) == CHASSIS_IMU_ODOM_OK);
    assert(odom.config.imu_gyro_noise == config.imu_gyro_noise);
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    wheel.vx = 1.0f;
    for(int i = 0; i < 100; ++i) step(&odom, wheel, imu);
    assert(isfinite(odom.output.angle.z));
}

static void test_nonlinear_mean_and_yaw_covariance(void) {
    ChassisImuOdom odom;
    assert(chassis_imu_odom_init(&odom, NULL) == CHASSIS_IMU_ODOM_OK);
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_YAW] = 1.0f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_VX] = 1.0f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_VY] = 0.3f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_WZ] = 0.4f;
    memset(odom.filter.P_data, 0, sizeof(odom.filter.P_data));
    odom.filter.P_data[CHASSIS_IMU_ODOM_STATE_YAW * CHASSIS_IMU_ODOM_STATE_DIM + CHASSIS_IMU_ODOM_STATE_YAW] = 0.25f;
    ChassisImuOdomChassis wheel = { 0 };
    ChassisImuOdomImu imu = { 0 };
    step(&odom, wheel, imu);
    const float heading = 1.0f + 0.5f * 0.4f * dt;
    const float expected_x = (cosf(heading) - 0.3f * sinf(heading)) * dt;
    const float expected_y = (sinf(heading) + 0.3f * cosf(heading)) * dt;
    assert(fabsf(odom.output.odom.x - expected_x) < 1e-8f);
    assert(fabsf(odom.output.odom.y - expected_y) < 1e-8f);
    assert(fabsf(odom.output.angle.z - (1.0f + 0.4f * dt)) < 1e-7f);
    assert(fabsf(odom.filter.P_data[CHASSIS_IMU_ODOM_STATE_Y * CHASSIS_IMU_ODOM_STATE_DIM + CHASSIS_IMU_ODOM_STATE_YAW]) > 1e-5f);
}

static void test_motion_and_observation_weighting(void) {
    ChassisImuOdom odom;
    assert(chassis_imu_odom_init(&odom, NULL) == CHASSIS_IMU_ODOM_OK);
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    wheel.vx = 1.0f;
    for(int i = 0; i < 2500; ++i) step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    assert(fabsf(odom.output.odom.x - 10.0f) < 0.02f);
    assert(fabsf(odom.output.odom.y) < 1e-6f);
    assert(fabsf(odom.output.angle.z) < 1e-6f);

    wheel.wz = 0.4f;
    imu.gyro.z = 0.8f;
    for(int i = 0; i < 1000; ++i) step(&odom, wheel, imu);
    assert(fabsf(odom.output.velocity.wz - 0.5f) < 2e-5f);

    /* Repeated feedback must not be counted as a new independent measurement. */
    const float before_vx = odom.output.velocity.vx;
    const float before_wz = odom.output.velocity.wz;
    const unsigned vv = CHASSIS_IMU_ODOM_STATE_VX * CHASSIS_IMU_ODOM_STATE_DIM + CHASSIS_IMU_ODOM_STATE_VX;
    const float before_variance = odom.filter.P_data[vv];
    wheel.new_sample = false;
    wheel.vx = 99.0f;
    wheel.wz = 99.0f;
    imu.gyro_new_sample = false;
    imu.gyro.z = 99.0f;
    step(&odom, wheel, imu);
    assert(odom.output.velocity.vx == before_vx);
    assert(odom.output.velocity.wz == before_wz);
    assert(fabsf(odom.filter.P_data[vv] - before_variance - odom.config.velocity_process_noise_x * dt) < 1e-7f);

    /* Invalid inputs are omitted, including NaN; they never become zero observations. */
    wheel.new_sample = true;
    wheel.valid = false;
    wheel.vx = NAN;
    imu.gyro_new_sample = true;
    imu.gyro_valid = false;
    imu.gyro.z = NAN;
    step(&odom, wheel, imu);
    assert(odom.output.velocity.vx == before_vx);
    assert(odom.output.velocity.wz == before_wz);
}

static void test_stationary_window_and_invalidation(void) {
    ChassisImuOdom odom;
    assert(chassis_imu_odom_init(&odom, NULL) == CHASSIS_IMU_ODOM_OK);
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    for(int i = 0; i < 100; ++i) step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    for(int i = 0; i < 30; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    wheel.stationary_allowed = false;
    step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    wheel.stationary_allowed = true;
    for(int i = 0; i < 130; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    wheel.vx = 0.004f; /* Between enter and exit thresholds. */
    wheel.new_sample = false;
    step(&odom, wheel, imu);
    assert(odom.output.stationary);
    assert(odom.output.velocity.vx == 0.0f);
    imu.gyro.x = 0.04f; /* Exceeds the instantaneous exit gate; z still appears stationary. */
    step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    imu.gyro.x = 0.0f;
    wheel.vx = 0.0f;
    for(int i = 0; i < 130; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    wheel.valid = false;
    step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    wheel.valid = true;
    for(int i = 0; i < 130; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    chassis_imu_odom_invalidate(&odom);
    assert(!odom.output.stationary);
    assert(!odom.output.velocity.valid);
    assert(!odom.stationary_gyro_mean_initialized);
    step(&odom, wheel, imu);
    assert(!odom.output.stationary);

    /* Reusing one accelerometer sample cannot satisfy a statistical window. */
    imu.acc_new_sample = false;
    for(int i = 0; i < 250; ++i) step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    chassis_imu_odom_invalidate(&odom);
    imu.acc_new_sample = true;
    for(int i = 0; i < 250; ++i) {
        imu.acc.x = i % 2 ? 1.0f : -1.0f;
        step(&odom, wheel, imu);
        assert(!odom.output.stationary);
    }
}

static float deterministic_normal(uint32_t* state) {
    /* Sum of twelve uniform draws: reproducible zero-mean, unit-variance noise. */
    float sum = -6.0f;
    for(int i = 0; i < 12; ++i) {
        *state = *state * 1664525u + 1013904223u;
        sum += (float)(*state >> 8) / 16777216.0f;
    }
    return sum;
}

static Vector3 measured_scale_noise(uint32_t* state) {
    /* Baseline static capture: std xyz = .00367/.00449/.00413 rad/s,
     * with a small .00035 rad/s residual z mean. This is synthetic 250 Hz
     * noise at the measured scale, not a replay of the 16 Hz capture. */
    return (Vector3){
        0.00367f * deterministic_normal(state),
        0.00449f * deterministic_normal(state),
        0.00413f * deterministic_normal(state) + 0.00035f,
    };
}

static void test_noisy_stationary_and_slow_turn(float mean_tau) {
    ChassisImuOdom odom;
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    ChassisImuOdomConfig config = chassis_imu_odom_default_config();
    config.imu_gyro_mean_time_constant_s = mean_tau;
    assert(config.imu_gyro_mean_static_threshold == 0.003f);
    assert(chassis_imu_odom_init(&odom, &config) == CHASSIS_IMU_ODOM_OK);
    uint32_t noise_state = 0x3b84e329u;
    int old_gate_exceeded = 0;
    int stationary_samples = 0;
    int first_stationary = -1;
    for(int i = 0; i < 2000; ++i) {
        imu.gyro = measured_scale_noise(&noise_state);
        const float norm_squared = imu.gyro.x * imu.gyro.x + imu.gyro.y * imu.gyro.y + imu.gyro.z * imu.gyro.z;
        if(norm_squared > 0.01f * 0.01f) ++old_gate_exceeded;
        step(&odom, wheel, imu);
        if(odom.output.stationary && first_stationary < 0) first_stationary = i;
        if(i >= 250 && odom.output.stationary) ++stationary_samples;
    }
    assert(old_gate_exceeded > 100); /* Noise would repeatedly reset the old gate. */
    assert(first_stationary >= 124 && first_stationary < 500);
    assert(stationary_samples > 1662); /* >95% stationary after one-second warm-up. */
    for(int i = 0; i < 500; ++i) {
        imu.gyro = measured_scale_noise(&noise_state);
        imu.gyro.z += 0.006f; /* Slow actual rotation; wheel and command remain zero. */
        step(&odom, wheel, imu);
        if(i >= 75) assert(!odom.output.stationary);
    }

    /* Starting during a slow turn cannot enter a stationary window. */
    chassis_imu_odom_invalidate(&odom);
    imu.gyro = (Vector3){ 0.0f, 0.0f, 0.006f };
    for(int i = 0; i < 500; ++i) {
        step(&odom, wheel, imu);
        assert(!odom.output.stationary);
    }
    assert(fabsf(odom.stationary_gyro_mean.z - 0.006f) < 1e-6f);

    /* The mean exit threshold is not widened by the 1.5 hysteresis ratio. */
    chassis_imu_odom_invalidate(&odom);
    imu.gyro.z = 0.0f;
    for(int i = 0; i < 130; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    imu.gyro.z = 0.006f;
    /* A zero-to-0.006 step crosses the unchanged 0.003 gate at tau * ln(2).
     * Allow up to 0.8 * tau: 80 ms for the default, 160 ms for the candidate. */
    const int exit_deadline = (int)ceilf(0.8f * mean_tau / dt);
    int first_exit = -1;
    for(int i = 0; i < exit_deadline; ++i) {
        step(&odom, wheel, imu);
        if(!odom.output.stationary && first_exit < 0) first_exit = i + 1;
    }
    assert(first_exit > 0 && first_exit <= exit_deadline);
    assert(!odom.output.stationary);

    /* Commands and unusable sensor data clear both the mean and candidate. */
    wheel.stationary_allowed = false;
    step(&odom, wheel, imu);
    assert(!odom.stationary_gyro_mean_initialized && odom.stationary_candidate_s == 0.0f);
    wheel.stationary_allowed = true;
    imu.gyro.z = 0.0f;
    for(int i = 0; i < 130; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    imu.gyro.z = NAN;
    step(&odom, wheel, imu);
    assert(!odom.output.stationary && !odom.stationary_gyro_mean_initialized);
    imu.gyro.z = 0.0f;
    for(int i = 0; i < 100; ++i) step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    for(int i = 0; i < 30; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    imu.acc_valid = false;
    step(&odom, wheel, imu);
    assert(!odom.output.stationary && !odom.stationary_gyro_mean_initialized);
    printf("PASS: gyro mean tau=%.2fs, noisy static %d/1750, slow-turn exit %.3fs, motion-start rejection and detector resets\n",
           (double)mean_tau, stationary_samples, (double)(first_exit * dt));
}

static Vector3 enabled_acc_noise(uint32_t* state) {
    /* Enabled capture means and per-axis std in m/s^2. This synthetic
     * independent, approximately Gaussian sequence does not reproduce the
     * measured vibration's temporal correlation or heavier distribution tails. */
    return (Vector3){
        0.5895f + 0.135f * deterministic_normal(state),
        -0.0529f + 0.143f * deterministic_normal(state),
        9.7367f + 0.149f * deterministic_normal(state),
    };
}

static void test_enabled_noise_threshold_candidate(float rms_limit, float norm_tolerance, float mean_tau) {
    ChassisImuOdom odom;
    ChassisImuOdomConfig config = chassis_imu_odom_default_config();
    config.acc_change_threshold = rms_limit;
    config.acc_norm_tolerance = norm_tolerance;
    config.imu_gyro_mean_time_constant_s = mean_tau;
    assert(config.imu_gyro_mean_static_threshold == 0.003f);
    assert(chassis_imu_odom_init(&odom, &config) == CHASSIS_IMU_ODOM_OK);
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    uint32_t noise_state = 0x6e9714b3u;
    const int samples = 10000;
    const int warmup = 500;
    int first_stationary = -1;
    int stationary_samples = 0;
    int entries = 0;
    bool previously_stationary = false;
    for(int i = 0; i < samples; ++i) {
        imu.gyro = measured_scale_noise(&noise_state);
        imu.acc = enabled_acc_noise(&noise_state);
        step(&odom, wheel, imu);
        if(odom.output.stationary) {
            if(first_stationary < 0) first_stationary = i;
            if(!previously_stationary) ++entries;
            if(i >= warmup) ++stationary_samples;
        }
        previously_stationary = odom.output.stationary;
    }
    /* This scenario reports the candidate tradeoff, not a hardware acceptance
     * result or a production-default choice. Both candidates must recognize
     * at least some sustained stationary periods in the fixed sequence. */
    assert(first_stationary >= 124 && first_stationary < samples);
    assert(stationary_samples > 0);
    printf("INFO: synthetic enabled noise, acc RMS gate=%.2f, norm gate=%.2f, gyro mean tau=%.2fs: first static %.3fs, static %.2f%% after 2s, entries=%d\n",
           (double)rms_limit, (double)norm_tolerance, (double)mean_tau, (double)((first_stationary + 1) * dt),
           100.0 * stationary_samples / (samples - warmup), entries);

    /* Increasing the candidate RMS limit still must reject strong vibration. */
    for(int i = 0; i < 500; ++i) {
        imu.gyro = measured_scale_noise(&noise_state);
        imu.acc = enabled_acc_noise(&noise_state);
        imu.acc.x += i % 2 ? 1.0f : -1.0f;
        step(&odom, wheel, imu);
        if(i >= 125) assert(!odom.output.stationary);
    }

    /* Even at the larger candidate accelerometer limits, steady slow yaw
     * must exit a confirmed stationary state and never be learned as bias. */
    chassis_imu_odom_invalidate(&odom);
    imu = imu_sample();
    for(int i = 0; i < 130; ++i) step(&odom, wheel, imu);
    assert(odom.output.stationary);
    for(int i = 0; i < 500; ++i) {
        imu.gyro = measured_scale_noise(&noise_state);
        imu.gyro.z += 0.006f;
        imu.acc = enabled_acc_noise(&noise_state);
        step(&odom, wheel, imu);
        if(i >= 75) assert(!odom.output.stationary);
    }
}

static void assert_finite_psd(const KalmanFilter* filter) {
    double l[CHASSIS_IMU_ODOM_STATE_DIM][CHASSIS_IMU_ODOM_STATE_DIM] = { { 0 } };
    for(int i = 0; i < CHASSIS_IMU_ODOM_STATE_DIM; ++i) {
        for(int j = 0; j <= i; ++j) {
            double p = filter->P_data[i * CHASSIS_IMU_ODOM_STATE_DIM + j];
            double transpose = filter->P_data[j * CHASSIS_IMU_ODOM_STATE_DIM + i];
            assert(isfinite(p) && fabs(p - transpose) < 1e-6);
            /* Tiny diagonal regularization admits the exact zero eigenvalues
             * of the deterministic velocity constraint. */
            if(i == j) p += 1e-9;
            for(int m = 0; m < j; ++m) p -= l[i][m] * l[j][m];
            if(i == j) { assert(p > 0.0); l[i][j] = sqrt(p); }
            else l[i][j] = p / l[j][j];
        }
    }
}

static void test_stationary_covariance_hold_and_release(void) {
    ChassisImuOdom odom;
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    assert(chassis_imu_odom_init(&odom, NULL) == CHASSIS_IMU_ODOM_OK);
    imu.gyro.z = 0.001f;
    for(int i = 0; i < 124; ++i) step(&odom, wheel, imu);
    assert(!odom.output.stationary);

    /* Simulate accumulated position/yaw error and correlated nonzero speed
     * uncertainty immediately before the detector confirms standstill. */
    const int n = CHASSIS_IMU_ODOM_STATE_DIM;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_X] = 2.5f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_Y] = -1.2f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_Z] = 0.2f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_YAW] = 0.4f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_VX] = 0.02f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_VY] = -0.01f;
    odom.filter.x_data[CHASSIS_IMU_ODOM_STATE_WZ] = 0.03f;
    for(int i = 0; i < n; ++i) {
        for(int j = 0; j < n; ++j) {
            odom.filter.P_data[i * n + j] = 0.0001f * (i + 1) * (j + 1)
                + (i == j ? 0.1f * (i + 1) : 0.0f);
        }
    }
    float old_p[CHASSIS_IMU_ODOM_STATE_DIM * CHASSIS_IMU_ODOM_STATE_DIM];
    memcpy(old_p, odom.filter.P_data, sizeof(old_p));
    const float last_observation = odom.filter.z_data[0];
    const float last_variance = odom.filter.R_data[0];
    imu.angle = (Vector3){ 0.2f, -0.1f, 0.0f };
    imu.gyro.z = 0.002f;
    for(int sample = 0; sample < 250; ++sample) {
        imu.angle.x = 0.2f + 0.01f * sinf(sample * 0.01f);
        step(&odom, wheel, imu);
        assert(odom.output.stationary);
        assert(odom.output.odom.x == 2.5f && odom.output.odom.y == -1.2f && odom.output.odom.z == 0.2f);
        assert(odom.output.angle.z == 0.4f);
        assert(odom.output.angle.x == imu.angle.x && odom.output.angle.y == imu.angle.y);
        assert(odom.output.velocity.vx == 0.0f && odom.output.velocity.vy == 0.0f && odom.output.velocity.wz == 0.0f);
        assert(odom.filter.z_data[0] == last_observation && odom.filter.R_data[0] == last_variance);
        for(int i = 0; i < n; ++i) {
            for(int j = 0; j < n; ++j) {
                const bool pose_pair = i < CHASSIS_IMU_ODOM_STATE_VX && j < CHASSIS_IMU_ODOM_STATE_VX;
                assert(odom.filter.P_data[i * n + j] == (pose_pair ? old_p[i * n + j] : 0.0f));
                assert(odom.filter.Q_data[i * n + j] == 0.0f);
                assert(odom.filter.F_data[i * n + j] == (pose_pair && i == j ? 1.0f : 0.0f));
            }
        }
        assert_finite_psd(&odom.filter);
    }
    const float last_roll = odom.output.angle.x;
    imu.angle_valid = false;
    imu.angle.x = NAN;
    step(&odom, wheel, imu);
    assert(odom.output.stationary && odom.output.angle.x == last_roll);

    /* A motion command releases the constraint. Restored dynamic Q must give
     * new sensor data nonzero gain immediately, even though velocity P was zero. */
    wheel.stationary_allowed = false;
    wheel.vx = 0.5f;
    wheel.vy = -0.2f;
    wheel.wz = 0.4f;
    imu.angle_valid = true;
    imu.angle = (Vector3){ 0.0f, 0.0f, 0.0f };
    imu.gyro.z = 0.4f;
    step(&odom, wheel, imu);
    assert(!odom.output.stationary);
    assert(odom.output.velocity.vx > 0.0f && odom.output.velocity.vy < 0.0f && odom.output.velocity.wz > 0.0f);
    assert(odom.filter.Q_data[0] == odom.config.position_process_noise_x * dt);
    assert(odom.filter.Q_data[50] == odom.config.yaw_process_noise * dt);
    assert(odom.filter.Q_data[60] == odom.config.velocity_process_noise_x * dt);
    assert(odom.filter.Q_data[70] == odom.config.velocity_process_noise_y * dt);
    assert(odom.filter.Q_data[80] == odom.config.velocity_process_noise_wz * dt);
    assert(odom.filter.P_data[60] > 0.0f && odom.filter.P_data[70] > 0.0f && odom.filter.P_data[80] > 0.0f);
    assert(odom.filter.F_data[60] == 1.0f && odom.filter.F_data[53] == dt);
    assert_finite_psd(&odom.filter);
    for(int i = 0; i < 100; ++i) {
        step(&odom, wheel, imu);
        assert_finite_psd(&odom.filter);
    }
    assert(hypotf(odom.output.odom.x - 2.5f, odom.output.odom.y + 1.2f) > 0.05f);
    assert(odom.output.angle.z > 0.42f);
    assert(fabsf(odom.output.velocity.vx - wheel.vx) < 0.01f);
    puts("PASS: stationary pose/P hold, deterministic velocity covariance, dynamic release");
}

static void test_angular_noise_scale_equivalence(void) {
    ChassisImuOdom reference, scaled;
    ChassisImuOdomConfig reference_config = chassis_imu_odom_default_config();
    ChassisImuOdomConfig scaled_config = reference_config;
    scaled_config.imu_gyro_noise = 3.0e-5f;
    scaled_config.chassis_velocity_noise_wz = 1.0e-5f;
    scaled_config.velocity_process_noise_wz = 2.5e-5f;
    const float scale = scaled_config.imu_gyro_noise / reference_config.imu_gyro_noise;
    assert(chassis_imu_odom_init(&reference, &reference_config) == CHASSIS_IMU_ODOM_OK);
    assert(chassis_imu_odom_init(&scaled, &scaled_config) == CHASSIS_IMU_ODOM_OK);
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    for(int i = 0; i < 130; ++i) {
        step(&reference, wheel, imu);
        step(&scaled, wheel, imu);
    }
    assert(reference.output.stationary && scaled.output.stationary);
    assert(reference.filter.P_data[80] == 0.0f && scaled.filter.P_data[80] == 0.0f);

    float time_s = 0.0f;
    float max_state_difference = 0.0f;
    float max_covariance_scale_error = 0.0f;
    bool stopped_again = false;
    bool released_again = false;
    for(int i = 0; i < 2500; ++i) {
        const float sample_dt = i % 3 == 0 ? 0.0037f : (i % 3 == 1 ? 0.0040f : 0.0043f);
        time_s += sample_dt;
        wheel.stationary_allowed = i >= 1500 && i < 1900;
        wheel.vx = 0.0f;
        wheel.vy = 0.0f;
        wheel.wz = 0.0f;
        float gyro_yaw_rate = 0.0f;
        if(i < 500) {
            wheel.vx = 0.4f + 0.2f * sinf(time_s);
            gyro_yaw_rate = 0.002f * cosf(7.0f * time_s);
        }
        else if(i < 1000) {
            wheel.vy = 0.3f + 0.1f * cosf(time_s);
            gyro_yaw_rate = 0.002f * sinf(9.0f * time_s);
        }
        else if(i < 1500 || i >= 1900) {
            wheel.vx = 0.5f;
            wheel.vy = 0.1f * sinf(time_s);
            wheel.wz = 0.4f * sinf(0.7f * time_s) + 0.08f;
            gyro_yaw_rate = 0.42f * sinf(0.7f * time_s) + 0.005f * cosf(11.0f * time_s);
        }
        imu.angle.x = 0.04f * sinf(0.1f * time_s);
        imu.angle.y = -0.06f;
        imu.gyro.y = wheel.stationary_allowed ? 0.0f : 0.001f * sinf(3.0f * time_s);
        imu.gyro.z = (gyro_yaw_rate * cosf(imu.angle.y) - imu.gyro.y * sinf(imu.angle.x)) / cosf(imu.angle.x);
        /* Independent sensor cadences exercise sequential updates, not just
         * the steady-state two-observation scalar formula. */
        wheel.new_sample = i % 3 != 1;
        imu.gyro_new_sample = i % 11 != 3;
        imu.acc_new_sample = i % 2 == 0;
        assert(chassis_imu_odom_update(&reference, wheel, imu, sample_dt) == CHASSIS_IMU_ODOM_OK);
        assert(chassis_imu_odom_update(&scaled, wheel, imu, sample_dt) == CHASSIS_IMU_ODOM_OK);
        assert(reference.output.stationary == scaled.output.stationary);
        if(reference.output.stationary) stopped_again = true;
        if(i >= 1900 && stopped_again && !reference.output.stationary && fabsf(scaled.output.velocity.wz) > 0.01f) released_again = true;
        for(int state = 0; state < CHASSIS_IMU_ODOM_STATE_DIM; ++state) {
            const float difference = fabsf(reference.filter.x_data[state] - scaled.filter.x_data[state]);
            if(difference > max_state_difference) max_state_difference = difference;
            assert(difference < 2e-5f);
        }
        const float expected_pww = scale * reference.filter.P_data[80];
        const float scale_error = fabsf(scaled.filter.P_data[80] - expected_pww);
        if(scale_error > max_covariance_scale_error) max_covariance_scale_error = scale_error;
        assert(scale_error <= 1e-10f + fabsf(expected_pww) * 2e-4f);
        assert(scaled.filter.Q_data[0] == reference.filter.Q_data[0]);
        assert(scaled.filter.Q_data[50] == reference.filter.Q_data[50]);
        assert(scaled.filter.Q_data[60] == reference.filter.Q_data[60]);
        assert(scaled.filter.Q_data[70] == reference.filter.Q_data[70]);
        assert_finite_psd(&reference.filter);
        assert_finite_psd(&scaled.filter);
        if(i == 0) {
            assert(!scaled.output.stationary && scaled.output.velocity.vx > 0.0f);
            assert(scaled.filter.Q_data[80] > 0.0f && scaled.filter.P_data[80] > 0.0f);
        }
    }
    assert(stopped_again && released_again);
    printf("PASS: angular noise scale equivalence in C float; max state difference %.3g, Pww scale error %.3g\n",
           (double)max_state_difference, (double)max_covariance_scale_error);
}

static void test_attitude_source_and_covariance(void) {
    ChassisImuOdom odom;
    assert(chassis_imu_odom_init(&odom, NULL) == CHASSIS_IMU_ODOM_OK);
    ChassisImuOdomChassis wheel = wheel_sample();
    ChassisImuOdomImu imu = imu_sample();
    imu.angle.x = 0.2f;
    imu.angle.y = -0.3f;
    wheel.vx = 0.8f;
    const int states[] = { 0, 1, 5, 6, 7, 8 };
    for(int k = 0; k < 2000; ++k) {
        wheel.vy = 0.3f * sinf(k * 0.01f);
        wheel.wz = 0.4f * sinf(k * 0.02f);
        imu.gyro.z = wheel.wz * cosf(imu.angle.y) / cosf(imu.angle.x);
        step(&odom, wheel, imu);
        assert(odom.output.angle.x == imu.angle.x);
        assert(odom.output.angle.y == imu.angle.y);
        double l[6][6] = { { 0 } };
        for(int i = 0; i < 6; ++i) {
            for(int j = 0; j <= i; ++j) {
                double p = odom.filter.P_data[states[i] * CHASSIS_IMU_ODOM_STATE_DIM + states[j]];
                double transpose = odom.filter.P_data[states[j] * CHASSIS_IMU_ODOM_STATE_DIM + states[i]];
                assert(isfinite(p) && fabs(p - transpose) < 1e-6);
                if(i == j) p += 1e-9;
                for(int m = 0; m < j; ++m) p -= l[i][m] * l[j][m];
                if(i == j) { assert(p > 0.0); l[i][j] = sqrt(p); }
                else l[i][j] = p / l[j][j];
            }
        }
    }
}

int main(void) {
    test_small_variance();
    test_nonlinear_mean_and_yaw_covariance();
    test_motion_and_observation_weighting();
    test_stationary_window_and_invalidation();
    test_noisy_stationary_and_slow_turn(0.10f);
    test_noisy_stationary_and_slow_turn(0.20f);
    test_enabled_noise_threshold_candidate(0.30f, 0.35f, 0.10f);
    test_enabled_noise_threshold_candidate(0.30f, 0.55f, 0.10f);
    test_enabled_noise_threshold_candidate(0.30f, 0.55f, 0.20f);
    test_enabled_noise_threshold_candidate(0.35f, 0.55f, 0.20f);
    test_stationary_covariance_hold_and_release();
    test_angular_noise_scale_equivalence();
    test_attitude_source_and_covariance();
    puts("PASS: odometry prediction, fresh observations, covariance, and stationary windows");
    return 0;
}
