/* Replay the 16-leg route through the production Auto entry point. The wheel
 * pose follows the commanded body twist on a level floor. Laser frames are
 * deliberately extreme: with guidance disabled they must not affect motion. */
#include "app_odom_test.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
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
static bool wheel_online;
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
const struct OdomInterface odom_interface = {
    .get_odom = get_position, .get_angle = get_angle, .get_velocity = get_velocity,
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
    now_ms += 2u;
}
static void send_frame(LaserBeam beam, uint16_t range) {
    LaserOdomSample frame = {
        .distance_mm = { UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX },
        .received = laser_odom_get_sample()->received + 1u, .stamp_ms = now_ms,
    };
    if(beam != LASER_NO_BEAM) frame.distance_mm[beam] = range;
    laser_odom_receive(&frame);
}
static void reset(void) {
    app_odom_test_reset();
    const ChassisYawHoldConfig yaw_config = chassis_yaw_hold_wheel_config();
    chassis_yaw_hold_init(&yaw_config);
    now_ms = 1000u;
    position = (Vector3){ 1.3f, -0.4f, 0.0f };
    velocity = angle = command = (Vector3){ 0 };
    wheel_online = true;
    translation_lag_s = 0.0f;
    laser_odom_receive(&(LaserOdomSample){ 0 });
}
static void process(void) {
    assert(app_odom_test_process() == APP_CONTROL_RESULT_OK);
    const AppLaserFix fix = app_odom_test_get_laser_fix();
    assert(fix.mask == 0u && fix.track_mask == 0u && fix.anchor_mask == 0u);
}
static void opening_run(bool inject_laser, Point* final_position, Vector3* final_command) {
    reset();
    for(unsigned i = 0; i < 1000u; ++i) {
        advance();
        if(inject_laser && i % 10u == 0u)
            send_frame(LASER_LEFT, i % 20u == 0u ? 17u : 7000u);
        process();
    }
    *final_position = field_position();
    *final_command = command;
}
static void test_laser_frames_are_inert(void) {
    Point without_laser, with_laser;
    Vector3 command_without_laser, command_with_laser;
    opening_run(false, &without_laser, &command_without_laser);
    opening_run(true, &with_laser, &command_with_laser);
    assert(command_without_laser.x < -0.05f);
    assert(fabsf(with_laser.x - without_laser.x) < 1e-6f);
    assert(fabsf(with_laser.y - without_laser.y) < 1e-6f);
    assert(fabsf(command_with_laser.x - command_without_laser.x) < 1e-6f);
    assert(fabsf(command_with_laser.y - command_without_laser.y) < 1e-6f);
    assert(fabsf(command_with_laser.z - command_without_laser.z) < 1e-6f);
}
static void test_wheel_feedback_is_mandatory(void) {
    reset();
    for(unsigned i = 0; i < 500u; ++i) { advance(); process(); }
    assert(command.x < -0.05f);
    wheel_online = false;
    assert(app_odom_test_process() == APP_CONTROL_RESULT_ODOM_ERROR);
}
static void test_route(float lag_s, bool inject_laser) {
    reset();
    translation_lag_s = lag_s;
    unsigned leg = 1u, seen = 0u;
    unsigned direction_mask = 0u;
    bool heading_recovered = false;
    uint32_t yaw_disturbance_ms = 0u;
    float max_stop_error = 0.0f;
    Point farthest = waypoints[0];
    Point p = field_position();
    for(unsigned i = 0; i < 140000u && !app_odom_test_is_done(); ++i) {
        advance();
        p = field_position();
        farthest.x = fmaxf(farthest.x, p.x);
        farthest.y = fmaxf(farthest.y, p.y);
        if(inject_laser && i % 10u == 0u) {
            const LaserBeam beam = (LaserBeam)(leg % LASER_BEAM_COUNT);
            send_frame(beam, (uint16_t)(30u + (i % 5000u)));
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
                angle.z += 0.012f;
                yaw_disturbance_ms = now_ms;
            }
        }
        if(leg == 3u && now_ms - yaw_disturbance_ms > 1000u) {
            assert(fabsf(angle.z) < 0.002f && fabsf(command.z) < 0.015f);
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
        send_frame(LASER_FRONT, 67u);
        process();
        assert(app_odom_test_is_done());
        assert(command.x == 0.0f && command.y == 0.0f && command.z == 0.0f);
    }
    printf("PASS route: 16 legs, lag=%.0f ms, laser=%s, %.2f s, "
           "max end error %.3f m, parked (%.3f,%.3f) m\n",
           1000.0 * lag_s, inject_laser ? "ignored" : "absent",
           (now_ms - 1200u) * 0.001, (double)max_stop_error, (double)p.x, (double)p.y);
}
int main(void) {
    test_laser_frames_are_inert();
    test_wheel_feedback_is_mandatory();
    test_route(0.0f, false);
    test_route(0.0f, true);
    test_route(0.060f, true);
    puts("PASS wheel-only Auto: laser inert, wheel faults retained, all directions covered");
    return 0;
}
