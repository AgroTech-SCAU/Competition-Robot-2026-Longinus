/* Replay the competition through the production Auto entry point. Wheel pose
 * follows the commanded body twist on a level floor. Laser ranges are made
 * from the physical map, independently of the controller's route table. */
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
static const LaserBeam end_beams[LEG_COUNT + 1] = {
    LASER_NO_BEAM, LASER_FRONT, LASER_RIGHT, LASER_LEFT, LASER_NO_BEAM,
    LASER_RIGHT, LASER_NO_BEAM, LASER_LEFT, LASER_NO_BEAM, LASER_RIGHT,
    LASER_NO_BEAM, LASER_LEFT, LASER_NO_BEAM, LASER_RIGHT, LASER_REAR, LASER_LEFT, LASER_NO_BEAM
};

static uint32_t now_ms;
static Vector3 position, velocity, angle, command;
static Point odom_offset;
static bool wheel_online;
static float translation_lag_s;

ms_t delay_now_ms(void) { return now_ms; }
LogStatus log_info(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
LogStatus log_warn(const char* format, ...) { (void)format; return LOG_STATUS_OK; }
static OdomStatus get_position(Vector3* out) {
    *out = position;
    out->x += odom_offset.x;
    out->y += odom_offset.y;
    return wheel_online ? ODOM_OK : ODOM_NOT_READY;
}
OdomStatus odom_relocalize_xy(float x, float y) {
    assert(isfinite(x) && isfinite(y));
    odom_offset = (Point){x - position.x, y - position.y};
    return wheel_online ? ODOM_OK : ODOM_NOT_READY;
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
    odom_offset = (Point){ 0 };
    velocity = angle = command = (Vector3){ 0 };
    wheel_online = true;
    translation_lag_s = 0.0f;
    laser_odom_receive(&(LaserOdomSample){ 0 });
}
static void process(void) { assert(app_odom_test_process() == APP_CONTROL_RESULT_OK); }

static void test_left_follow(void) {
    reset();
    for(unsigned i = 0; i < 1450u; ++i) {
        advance();
        const float y = field_position().y;
        if(i % 10u == 0u) send_frame(LASER_LEFT, y < 0.0f || y > 0.55f ? 45u : 54u);
        process();
        if(y < 0.0f) {
            assert(app_odom_test_get_laser_fix().track_mask == 0u);
            assert(fabsf(command.y) < 0.00001f);
        }
    }
    const AppLaserFix fix = app_odom_test_get_laser_fix();
    assert(fix.track_mask & (1u << LASER_LEFT));
    assert(command.x < -0.05f && command.y < 0.0f && command.y >= -0.035f);
}

static void test_single_cross_wall(LaserBeam beam) {
    reset();
    bool followed = false;
    for(unsigned i = 0; i < 15000u && !followed; ++i) {
        advance();
        if(i % 10u == 0u) {
            const bool crossing = app_odom_test_get_laser_fix().segment == 2u;
            send_frame(crossing ? beam : LASER_NO_BEAM, beam == LASER_FRONT ? 57u : 60u);
        }
        process();
        const AppLaserFix fix = app_odom_test_get_laser_fix();
        if(fix.segment == 2u && fix.track_mask == (1u << beam) && fabsf(command.x) > 0.001f) {
            assert(command.y < 0.0f);
            assert(beam == LASER_FRONT ? command.x > 0.0f : command.x < 0.0f);
            followed = true;
        }
    }
    assert(followed);
}

static void test_unavailable_sensors(void) {
    reset();
    for(unsigned i = 0; i < 190u; ++i) { advance(); process(); }
    assert(command.x == 0.0f && command.y == 0.0f);
    for(unsigned i = 0; i < 450u; ++i) {
        advance();
        if(i % 10u == 0u) send_frame(LASER_LEFT, 17u);
        process();
    }
    assert(command.x < -0.05f);
    for(unsigned i = 0; i < 190u; ++i) { advance(); process(); }
    assert(command.x == 0.0f && command.y == 0.0f);
    Point stopped = field_position();
    for(unsigned i = 0; i < 50u; ++i) { advance(); process(); }
    assert(fabsf(field_position().y - stopped.y) < 0.001f);
    for(unsigned i = 0; i < 150u; ++i) {
        advance();
        if(i % 10u == 0u) send_frame(LASER_NO_BEAM, 0u);
        process();
    }
    assert(command.x < -0.05f);
    wheel_online = false;
    assert(app_odom_test_process() == APP_CONTROL_RESULT_ODOM_ERROR);
}

static void test_route(bool provide_landmark, float lag_s) {
    reset();
    translation_lag_s = lag_s;
    unsigned leg = 1u, seen = 0u, eligible_frames = 0u, accepted_beams = 0u;
    bool heading_recovered = false;
    uint32_t yaw_disturbance_ms = 0u;
    float max_stop_error = 0.0f;
    Point farthest = waypoints[0];
    /* The second field is 2.5 percent longer and its two lane endpoints
     * are 3 percent farther apart. The parking box stays outside the gate. */
    const float length_scale = provide_landmark ? 1.025f : 1.0f;
    const float width_scale = provide_landmark ? 1.030f : 1.0f;
    Point p = field_position();
    for(unsigned i = 0; i < 140000u && !app_odom_test_is_done(); ++i) {
        advance();
        p = field_position();
        farthest.x = fmaxf(farthest.x, p.x);
        farthest.y = fmaxf(farthest.y, p.y);
        const float end_x = 0.2f + (waypoints[leg].x - 0.2f) * width_scale;
        const float end_y = waypoints[leg].y > 0.0f ? waypoints[leg].y * length_scale : waypoints[leg].y;
        const LaserBeam end = end_beams[leg];
        float remaining = 0.0f;
        if(end == LASER_FRONT) remaining = end_y - p.y;
        if(end == LASER_REAR) remaining = p.y - end_y;
        if(end == LASER_RIGHT) remaining = end_x - p.x;
        if(end == LASER_LEFT) remaining = p.x - end_x;
        if(i % 10u == 0u) {
            if(provide_landmark && leg == 1u && p.y < 1.0f) {
                send_frame(LASER_FRONT, 67u); /* convincing range at the wrong map location */
            }
            else if(provide_landmark && leg == 1u && remaining < 0.78f && remaining >= 0.60f) {
                send_frame(LASER_FRONT, 710u); /* a blocked beam holds constant inside the gate */
            }
            else if(provide_landmark && end != LASER_NO_BEAM && remaining < 0.60f) {
                static const float endpoint_mm[] = {67.0f, 65.0f, 54.0f, 65.0f};
                const float raw = endpoint_mm[end] + 1000.0f * remaining;
                send_frame(end, (uint16_t)fmaxf(0.0f, raw + 0.5f));
                ++eligible_frames;
            }
            else {
                send_frame(LASER_NO_BEAM, 0u);
            }
        }
        process();
        const AppLaserFix fix = app_odom_test_get_laser_fix();
        if(fix.segment == leg && fix.anchor_mask) {
            assert(end != LASER_NO_BEAM && fix.anchor_mask == (1u << end));
            assert(eligible_frames >= 3u && remaining < 0.60f);
            accepted_beams |= fix.anchor_mask;
        }
        if(leg < LEG_COUNT && fix.segment == leg + 1u) {
            /* A lane end must be reached before the connector can start. */
            const float endpoint_error = hypotf(p.x - end_x, p.y - end_y);
            max_stop_error = fmaxf(max_stop_error, endpoint_error);
            assert(endpoint_error < 0.035f);
            ++seen;
            ++leg;
            eligible_frames = 0u;
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
    assert(accepted_beams == (provide_landmark ? 15u : 0u) && heading_recovered);
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
    printf("16 legs, lag=%.0f ms, anchors=0x%X, %.2f s, maximum field X/Y %.3f/%.3f m, "
           "max end error %.3f m, parked (%.3f,%.3f) m\n", 1000.0 * lag_s, accepted_beams,
           (now_ms - 1200u) * 0.001, (double)farthest.x, (double)farthest.y,
           (double)max_stop_error, (double)p.x, (double)p.y);
}
int main(void) {
    test_unavailable_sensors();
    test_left_follow();
    test_single_cross_wall(LASER_FRONT);
    test_single_cross_wall(LASER_REAR);
    test_route(false, 0.0f);
    test_route(true, 0.0f);
    test_route(true, 0.060f);
    return 0;
}
