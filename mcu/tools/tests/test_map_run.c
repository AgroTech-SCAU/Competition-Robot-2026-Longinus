/* Replay the 16-leg route through the production Auto entry point. The wheel
 * pose follows the commanded body twist on a level floor. Target laser frames
 * exercise settling; missing/off-target frames must block required switches. */
#include "app_odom_test.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "imu.h"
#include "laser_odom.h"
#include "log.h"
#include "odom.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct { float x, y; } Point;
static const Point waypoints[] = {
    {0.2f, -0.5f}, {0.2f, 4.4f}, {2.8f, 4.4f}, {0.2f, 4.4f},
    {0.2f, 3.7f},  {2.8f, 3.7f}, {2.8f, 3.0f}, {0.2f, 3.0f},
    {0.2f, 2.3f},  {2.8f, 2.3f}, {2.8f, 1.6f}, {0.2f, 1.6f},
    {0.2f, 0.9f},  {2.8f, 0.9f}, {2.8f, 0.2f}, {0.2f, 0.2f}, {0.2f, -0.5f}
};
enum { LEG_COUNT = sizeof(waypoints) / sizeof(waypoints[0]) - 1 };

static uint32_t now_ms;
static Vector3 position, velocity, angle, command;
static ImuAngle imu_angle;
static Odom odom_state;
static bool wheel_online, imu_online;
static float translation_lag_s;

ms_t delay_now_ms(void) { return now_ms; }
LogStatus log_info(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
LogStatus log_warn(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
static OdomStatus get_position(Vector3* out) {
    *out = position;
    return wheel_online ? ODOM_OK : ODOM_NOT_READY;
}
OdomStatus odom_relocalize_xy(float x, float y) {
    (void)x; (void)y;
    assert(!"wheel-only route must not relocalize from laser observations");
    return ODOM_NOT_READY;
}
static OdomStatus get_angle(Vector3* out) { *out = angle; return wheel_online ? ODOM_OK : ODOM_NOT_READY; }
static OdomStatus get_velocity(Vector3* out) { *out = velocity; return wheel_online ? ODOM_OK : ODOM_NOT_READY; }
static const Odom* get_odom_state(void) {
    odom_state.imu_ready = imu_online;
    return &odom_state;
}
const struct OdomInterface odom_interface = {
    .get_odom = get_position, .get_angle = get_angle, .get_velocity = get_velocity,
    .get_state = get_odom_state,
};
static ImuAngle get_imu_angle(void) { return imu_angle; }
static ImuGyro get_imu_gyro(void) { return (ImuGyro){ 0.0f, 0.0f, velocity.z }; }
static const ImuInterface mock_imu = {
    .get_angle = get_imu_angle, .get_gyro_corrected = get_imu_gyro,
};
static ChassisErrorCode set_velocity(float vx, float vy, float wz) {
    assert(isfinite(vx) && isfinite(vy) && isfinite(wz));
    command = (Vector3){ vx, vy, wz };
    return CHASSIS_OK;
}
const struct ChassisInterface chassis_interface = { .set_velocity = set_velocity };
AppControlResult app_control_hold_zero_chassis(void) {
    chassis_yaw_hold_reset();
    set_velocity(0.0f, 0.0f, 0.0f);
    return APP_CONTROL_RESULT_OK;
}

/* World origin of the wheel odometer is deliberately unrelated to the field. */
static Point field_position(void) {
    return (Point){0.2f - (position.y + 0.4f), -0.5f - (position.x - 1.3f)};
}
static void advance(void) {
    const float dt = 0.002f;
    const float response = translation_lag_s > dt ? dt / translation_lag_s : 1.0f;
    velocity.x += response * (command.x - velocity.x);
    velocity.y += response * (command.y - velocity.y);
    velocity.z += (command.z - velocity.z) * (dt / 0.040f);
    const float heading = angle.z + 0.5f * velocity.z * dt;
    position.x += dt * (cosf(heading) * velocity.x - sinf(heading) * velocity.y);
    position.y += dt * (sinf(heading) * velocity.x + cosf(heading) * velocity.y);
    angle.z += velocity.z * dt;
    imu_angle.yaw += velocity.z * dt;
    now_ms += 2u;
}
static void send_ranges(uint16_t front, uint16_t rear, uint16_t left, uint16_t right) {
    LaserOdomSample frame = {
        .distance_mm = { front, rear, left, right },
        .received = laser_odom_get_sample()->received + 1u, .stamp_ms = now_ms,
    };
    laser_odom_receive(&frame);
}
static void send_ideal_frame(void) { send_ranges(70u, 45u, 45u, 45u); }
static void reset(void) {
    app_odom_test_reset();
    const ChassisYawHoldConfig yaw_config = chassis_yaw_hold_wheel_config();
    chassis_yaw_hold_init(&yaw_config);
    assert(imu_set_instance(&mock_imu) == IMU_STATUS_OK);
    now_ms = 1000u;
    position = (Vector3){ 1.3f, -0.4f, 0.0f };
    velocity = angle = command = (Vector3){ 0 };
    imu_angle = (ImuAngle){ 0 };
    odom_state = (Odom){ 0 };
    wheel_online = imu_online = true;
    translation_lag_s = 0.0f;
    laser_odom_receive(&(LaserOdomSample){ 0 });
}
static void process(void) { assert(app_odom_test_process() == APP_CONTROL_RESULT_OK); }
static void run_ideal_cycles(unsigned cycles) {
    for(unsigned i = 0; i < cycles; ++i) {
        advance();
        if(i % 10u == 0u) send_ideal_frame();
        process();
    }
}
static void test_laser_offline_holds_controlled_route(void) {
    reset();
    const Point start = field_position();
    for(unsigned i = 0; i < 1000u; ++i) { advance(); process(); }
    const Point p = field_position();
    assert(command.x == 0.0f && command.y == 0.0f);
    assert(fabsf(p.x - start.x) < 1e-6f && fabsf(p.y - start.y) < 1e-6f);
    assert(app_odom_test_get_laser_fix().segment == 1u);
}
static void test_start_alignment_is_mandatory(void) {
    reset();
    /* Reach the 01 endpoint with no usable LEFT. It must settle and wait. */
    for(unsigned i = 0; i < 15000u; ++i) {
        advance();
        if(i % 10u == 0u) send_ranges(70u, 45u, UINT16_MAX, 45u);
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 1u);
    assert(app_odom_test_remaining_m() > 0.10f);
    assert(command.x == 0.0f && command.y == 0.0f);

    /* A distant LEFT approaches via -vy, but cannot authorize the switch. */
    for(unsigned i = 0; i < 100u; ++i) {
        advance();
        if(i % 10u == 0u) send_ranges(70u, 45u, 60u, 45u);
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 1u && command.y < 0.0f);

    /* FRONT is an independent second condition: too far means -vx and wait. */
    for(unsigned i = 0; i < 100u; ++i) {
        advance();
        if(i % 10u == 0u) send_ranges(80u, 45u, 45u, 45u);
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 1u);
    assert(command.x < 0.0f && command.y == 0.0f);

    /* Losing FRONT also waits; fresh 70/45 frames then release leg 02. */
    for(unsigned i = 0; i < 100u; ++i) {
        advance();
        if(i % 10u == 0u) send_ranges(UINT16_MAX, 45u, 45u, 45u);
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 1u);
    assert(command.x == 0.0f && command.y == 0.0f);
    run_ideal_cycles(150u);
    assert(app_odom_test_get_laser_fix().segment == 2u);
}
static void test_connectors_do_not_track_previous_front_wall(void) {
    reset();
    for(unsigned i = 0; i < 60000u && app_odom_test_get_laser_fix().segment < 4u; ++i) {
        advance();
        if(i % 10u == 0u) send_ideal_frame();
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 4u);

    /* After 700 mm of +vx travel, the previous front wall is about 770 mm
     * behind. Leg 04 must finish by wheel X + LEFT Y, never drag that wall
     * back to 70 mm with -vx. */
    for(unsigned i = 0; i < 10000u && app_odom_test_get_laser_fix().segment == 4u; ++i) {
        advance();
        if(i % 10u == 0u) send_ranges(770u, 45u, 45u, 45u);
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 5u);
    assert(command.x == 0.0f);
}
static void test_wheel_feedback_is_mandatory(void) {
    reset();
    run_ideal_cycles(500u);
    assert(command.x < -0.05f);
    wheel_online = false;
    assert(app_odom_test_process() == APP_CONTROL_RESULT_ODOM_ERROR);
}
static void test_imu_is_the_only_heading_feedback(void) {
    reset();
    run_ideal_cycles(500u);
    const float wheel_only_wz = command.z;
    angle.z += 0.2f; /* Wheel yaw disturbance must not enter the Auto angle loop. */
    advance();
    process();
    assert(fabsf(command.z - wheel_only_wz) < 1e-7f);
    imu_online = false;
    assert(app_odom_test_process() == APP_CONTROL_RESULT_ODOM_ERROR);
}
static void test_route(float lag_s) {
    reset();
    translation_lag_s = lag_s;
    unsigned leg = 1u, seen = 0u;
    unsigned direction_mask = 0u;
    bool heading_recovered = false;
    uint32_t yaw_disturbance_ms = 0u;
    float max_stop_error = 0.0f;
    Point p = field_position();
    for(unsigned i = 0; i < 140000u && !app_odom_test_is_done(); ++i) {
        advance();
        p = field_position();
        if(i % 10u == 0u) {
            /* Keep 01 on its wheel approach until near the planned endpoint,
             * then expose the 70 mm front target used by production settling. */
            const uint16_t front = leg == 1u && 4.4f - p.y > 0.020f ? 500u : 70u;
            send_ranges(front, 45u, 45u, 45u);
        }
        process();
        const float dx = waypoints[leg].x - waypoints[leg - 1u].x;
        const float dy = waypoints[leg].y - waypoints[leg - 1u].y;
        const float main_command = dx != 0.0f ? copysignf(command.y, dx) : -copysignf(command.x, dy);
        if(main_command > 0.05f) direction_mask |= 1u << (leg - 1u);
        if(leg < LEG_COUNT && app_odom_test_get_laser_fix().segment == leg + 1u) {
            const float endpoint_error = hypotf(p.x - waypoints[leg].x, p.y - waypoints[leg].y);
            max_stop_error = fmaxf(max_stop_error, endpoint_error);
            assert(endpoint_error < 0.035f);
            ++seen;
            ++leg;
            if(leg == 3u) {
                imu_angle.yaw += 0.012f;
                yaw_disturbance_ms = now_ms;
            }
        }
        if(leg == 3u && now_ms - yaw_disturbance_ms > 1000u) {
            assert(fabsf(imu_angle.yaw) < 0.002f && fabsf(command.z) < 0.015f);
            heading_recovered = true;
        }
    }
    assert(app_odom_test_is_done() && seen == LEG_COUNT - 1u);
    assert(direction_mask == (1u << LEG_COUNT) - 1u && heading_recovered);
    assert(fabsf(p.x - waypoints[0].x) < 0.035f);
    assert(fabsf(p.y - waypoints[0].y) < 0.035f);
    assert(app_odom_test_remaining_m() == 0.0f);
    for(unsigned i = 0; i < 100u; ++i) {
        advance();
        send_ideal_frame();
        process();
        assert(app_odom_test_is_done());
        assert(command.x == 0.0f && command.y == 0.0f && command.z == 0.0f);
    }
    printf("PASS route: 16 legs, lag=%.0f ms, target lasers, %.2f s, "
           "max end error %.3f m, parked (%.3f,%.3f) m\n",
           1000.0 * lag_s, (now_ms - 1200u) * 0.001,
           (double)max_stop_error, (double)p.x, (double)p.y);
}
int main(void) {
    test_laser_offline_holds_controlled_route();
    test_start_alignment_is_mandatory();
    test_connectors_do_not_track_previous_front_wall();
    test_wheel_feedback_is_mandatory();
    test_imu_is_the_only_heading_feedback();
    test_route(0.0f);
    test_route(0.060f);
    puts("PASS Auto: wheel position, IMU-only yaw hold, no connector front-wall pullback");
    return 0;
}
