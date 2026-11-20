/* Replay the 16-leg route through the production Auto entry point, with the
 * mock wheel pose following the commanded body twist on a level floor. */
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
enum {
    FURROW_FRONT_MM = 56u,
    FURROW_REAR_MM = 53u,
    NARROW_FRONT_MM = 59u,
    NARROW_REAR_MM = 41u,
    LAUNCH_LEFT_MM = 36u,
    LEFT_TARGET_MM = 41u,
    RIGHT_TARGET_MM = 45u,
    FAR_WALL_MM = 500u,
};

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
    return (Point){0.2f + (position.y + 0.4f), -0.5f - (position.x - 1.3f)};
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
static void send_route_ranges(unsigned leg, uint16_t front, uint16_t rear) {
    const float dx = waypoints[leg].x - waypoints[leg - 1u].x;
    uint16_t left = dx < 0.0f ? FAR_WALL_MM :
        (leg == 1u && field_position().y < 0.0f ? LAUNCH_LEFT_MM : LEFT_TARGET_MM);
    uint16_t right = dx > 0.0f ? FAR_WALL_MM : RIGHT_TARGET_MM;
    if(dx < 0.0f) {
        const float range_mm = LEFT_TARGET_MM + 1000.0f * (field_position().x - waypoints[leg].x);
        left = (uint16_t)lroundf(fmaxf(1.0f, fminf(FAR_WALL_MM, range_mm)));
    }
    else if(dx > 0.0f && leg != 2u) {
        const float range_mm = RIGHT_TARGET_MM + 1000.0f * (waypoints[leg].x - field_position().x);
        right = (uint16_t)lroundf(fmaxf(1.0f, fminf(FAR_WALL_MM, range_mm)));
    }
    send_ranges(front, rear, left, right);
}
static void send_ideal_frame(void) {
    const unsigned segment = app_odom_test_get_laser_fix().segment;
    const unsigned leg = segment == 0u ? 1u : segment;
    send_route_ranges(leg, leg == 1u ? FAR_WALL_MM : FURROW_FRONT_MM, FURROW_REAR_MM);
}
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
    app_odom_test_set_work_range(false, 0u, 0u);
}
static void process(void) { assert(app_odom_test_process() == APP_CONTROL_RESULT_OK); }
static void run_ideal_cycles(unsigned cycles) {
    for(unsigned i = 0; i < cycles; ++i) {
        advance();
        if(i % 10u == 0u) send_ideal_frame();
        process();
    }
}

static void step_with_ranges(uint16_t front, uint16_t rear, uint16_t left, uint16_t right) {
    advance();
    send_ranges(front, rear, left, right);
    process();
}

static void run_without_new_ranges(unsigned cycles) {
    for(unsigned i = 0; i < cycles; ++i) {
        advance();
        process();
    }
}

static void test_crossing_laser_stability(void) {
    reset();
    for(unsigned i = 0; i < 30000u && app_odom_test_get_laser_fix().segment < 7u; ++i) {
        advance();
        if(i % 10u == 0u) send_ideal_frame();
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 7u);
    const unsigned dual_beams = (1u << LASER_FRONT) | (1u << LASER_REAR);

    /* Three distinct frames are required, even if the loop reads one frame many times. */
    step_with_ranges(68u, UINT16_MAX, FAR_WALL_MM, RIGHT_TARGET_MM);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    run_without_new_ranges(10u);
    assert(app_odom_test_get_laser_fix().track_mask == 0u && command.x == 0.0f);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == 0u);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams);
    run_without_new_ranges(20u);
    assert(fabsf(command.x + 0.010f) < 1e-6f);

    /* An isolated long rear return cannot turn guidance off or change its output. */
    step_with_ranges(68u, 2000u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams);
    assert(fabsf(command.x + 0.010f) < 1e-6f);

    /* The open gate tolerates a sustained sum of 127 mm. */
    step_with_ranges(70u, 57u, FAR_WALL_MM, RIGHT_TARGET_MM);
    step_with_ranges(70u, 57u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams);
    run_without_new_ranges(20u);
    assert(fabsf(command.x + 0.004f) < 1e-6f);

    /* In the open window, changing direction and returning to zero each take a ramp. */
    step_with_ranges(56u, 68u, FAR_WALL_MM, RIGHT_TARGET_MM);
    const float before_reversal = command.x;
    step_with_ranges(56u, 68u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(command.x < 0.0f && fabsf(command.x - before_reversal - 0.0006f) < 1e-6f);
    run_without_new_ranges(40u);
    assert(command.x > 0.005f);
    step_with_ranges(56u, 53u, FAR_WALL_MM, RIGHT_TARGET_MM);
    const float before_zero = command.x;
    step_with_ranges(56u, 53u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(command.x > 0.0f && fabsf(command.x - before_zero + 0.0006f) < 1e-6f);
    run_without_new_ranges(20u);
    assert(command.x == 0.0f);

    /* Both measured furrows stay within the new tolerance at their own width. */
    step_with_ranges(NARROW_FRONT_MM, NARROW_REAR_MM, 2147u, 579u);
    step_with_ranges(NARROW_FRONT_MM, NARROW_REAR_MM, 2147u, 579u);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams && command.x == 0.0f);

    /* After closing outside the hysteresis band, 127 mm alone cannot reacquire. */
    step_with_ranges(91u, 44u, FAR_WALL_MM, RIGHT_TARGET_MM);
    step_with_ranges(91u, 44u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == 0u);
    step_with_ranges(70u, 57u, FAR_WALL_MM, RIGHT_TARGET_MM);
    step_with_ranges(70u, 57u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == 0u);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams);
    run_without_new_ranges(61u);
    assert(app_odom_test_get_laser_fix().track_mask == 0u && command.x == 0.0f);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == 0u);
    step_with_ranges(68u, 41u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams);

    /* Raw safety takes effect on the first frame. Leg 07 drives toward LEFT. */
    run_ideal_cycles(900u);
    assert(app_odom_test_get_laser_fix().segment == 7u && command.y < -0.40f);
    step_with_ranges(FURROW_FRONT_MM, 0u, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().track_mask == dual_beams);
    assert(app_odom_test_get_laser_fix().jump_mask == (1u << LASER_REAR));
    assert(command.x < -0.07f && command.y < -0.40f);
    step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, 20u, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().jump_mask == (1u << LASER_LEFT));
    assert(command.y > 0.0f);
    step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, FAR_WALL_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().jump_mask == 0u);
    assert(command.y < 0.0f && command.y > -0.002f);
    run_ideal_cycles(200u);
    assert(command.y < -0.09f && command.y > -0.12f);
    puts("PASS crossing: distinct median frames, hysteresis, slew, fresh reacquisition and raw safety");
}

static void test_launch_and_field_left_targets(void) {
    reset();
    step_with_ranges(1440u, 521u, 42u, 31u);
    step_with_ranges(1440u, 521u, 42u, 31u);
    step_with_ranges(1440u, 521u, 42u, 31u);
    assert(app_odom_test_get_laser_fix().track_mask == (1u << LASER_LEFT));
    assert(command.y < 0.0f);

    /* At field Y=0 the same LEFT range is within 1 mm of the field target. */
    position.x = 1.3f - 0.50f;
    step_with_ranges(1142u, 147u, 42u, 776u);
    assert(app_odom_test_get_laser_fix().track_mask == (1u << LASER_LEFT));
    assert(command.y == 0.0f);
    step_with_ranges(1142u, 147u, 41u, 776u);
    assert(command.y == 0.0f);
    step_with_ranges(1142u, 147u, 43u, 776u);
    assert(command.y == 0.0f);
    step_with_ranges(1142u, 147u, 43u, 776u);
    assert(command.y < 0.0f);
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
static void test_wheel_finishes_entry_despite_laser_noise(void) {
    reset();
    run_ideal_cycles(150u);
    position.x = 1.3f - 4.50f;
    step_with_ranges(68u, FURROW_REAR_MM, LEFT_TARGET_MM, RIGHT_TARGET_MM);
    assert(app_odom_test_remaining_m() > 0.08f);
    assert(app_odom_test_get_laser_fix().anchor_mask == 0u && command.x < 0.0f);

    /* Normal wheel arrival never trims raw FRONT/LEFT values into a ±1 mm gate. */
    position.x = 1.3f - 4.90f;
    velocity = (Vector3){ 0 };
    send_ranges(62u, FURROW_REAR_MM, 39u, RIGHT_TARGET_MM);
    process();
    assert(command.x == 0.0f && command.y == 0.0f);
    for(unsigned i = 0; i < 60u && app_odom_test_get_laser_fix().segment == 1u; ++i) {
        step_with_ranges(i % 2u ? 62u : 108u, FURROW_REAR_MM,
                         i % 2u ? 39u : 43u, RIGHT_TARGET_MM);
        assert(command.x == 0.0f);
    }
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
        if(i % 10u == 0u) send_ranges(770u, FURROW_REAR_MM, LEFT_TARGET_MM, RIGHT_TARGET_MM);
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

static void test_body_right_safety_after_heading_rotation(void) {
    reset();
    imu_angle.yaw = 1.0f;
    run_ideal_cycles(600u);
    assert(command.x < -0.20f && fabsf(command.y) < 1e-6f);

    /* A heading disturbance would rotate forward motion into CAN LEFT. */
    imu_angle.yaw -= 0.20f;
    step_with_ranges(FAR_WALL_MM, FURROW_REAR_MM, 20u, RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().jump_mask == (1u << LASER_LEFT));
    assert(command.y > 0.0f);
}

static void test_work_stop_and_resume(void) {
    reset();
    send_ideal_frame();
    app_odom_test_set_work_range(true, 350u, now_ms);
    process();
    run_ideal_cycles(500u);
    assert(command.x < 0.0f && !app_odom_test_work_pump_enabled());

    /* Place the replay at the first waypoint, then start the crossing. */
    position.x = 1.3f - 4.9f;
    velocity = (Vector3){ 0 };
    process();
    run_ideal_cycles(60u);
    assert(app_odom_test_get_laser_fix().segment == 2u);
    app_odom_test_set_work_range(true, 350u, now_ms);
    process();
    assert(command.y > 0.0f && !app_odom_test_work_pump_enabled());

    position.y += 1.0f; /* Leg 02 travels along +vy. */
    velocity.y = 0.20f;
    app_odom_test_set_work_range(true, 350u, now_ms - 201u);
    process();
    assert(command.y > 0.0f);
    app_odom_test_set_work_range(true, 350u, now_ms);
    process();
    assert(command.y == 0.0f && !app_odom_test_work_pump_enabled());
    velocity = (Vector3){ 0 };
    process();
    now_ms += 100u;
    process();
    assert(command.y == 0.0f && app_odom_test_work_pump_enabled());

    now_ms += 1999u;
    process();
    assert(command.y == 0.0f && app_odom_test_work_pump_enabled());
    ++now_ms;
    process();
    assert(command.y == 0.0f && !app_odom_test_work_pump_enabled());

    now_ms += 2u;
    send_ideal_frame();
    app_odom_test_set_work_range(true, 350u, now_ms);
    process();
    assert(command.y > 0.0f && app_odom_test_get_laser_fix().segment == 2u);
}

static void test_first_furrow_return_and_right_wall(void) {
    reset();
    for(unsigned i = 0; i < 20000u && app_odom_test_get_laser_fix().segment < 3u; ++i) {
        advance();
        if(i % 10u == 0u) send_ideal_frame();
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == 3u);
    run_ideal_cycles(1000u);
    assert(command.y < -0.40f);
    for(unsigned i = 0; i < 70u; ++i) {
        app_odom_test_set_work_range(true, 350u, now_ms);
        step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, FAR_WALL_MM, RIGHT_TARGET_MM);
        assert(command.y < -0.40f && !app_odom_test_work_pump_enabled());
    }

    /* Two of three recent frames see the body-right wall: slow with no raw escape. */
    step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, 120u, RIGHT_TARGET_MM);
    assert(command.y < -0.40f);
    step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, 120u, RIGHT_TARGET_MM);
    assert(command.y < -0.15f && command.y > -0.23f);
    assert(app_odom_test_get_laser_fix().jump_mask == 0u);

    for(unsigned i = 0; i < 60u; ++i) {
        step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, 48u, RIGHT_TARGET_MM);
    }
    assert(command.y == 0.0f && app_odom_test_remaining_m() > 0.08f);
    assert(app_odom_test_get_laser_fix().segment == 3u);

    /* The near-wall stop can begin in the wheel end window. The west side
     * still has to reach its own target before starting 04. */
    position.y -= app_odom_test_remaining_m() - 0.060f;
    for(unsigned i = 0; i < 60u; ++i) {
        step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, 48u, RIGHT_TARGET_MM);
        assert(app_odom_test_get_laser_fix().segment == 3u);
    }
    assert(command.y < 0.0f);
    for(unsigned i = 0; i < 60u && app_odom_test_get_laser_fix().segment == 3u; ++i) {
        step_with_ranges(FURROW_FRONT_MM, FURROW_REAR_MM, LEFT_TARGET_MM, RIGHT_TARGET_MM);
    }
    assert(app_odom_test_get_laser_fix().segment == 4u);

    for(unsigned i = 0; i < 15u; ++i) step_with_ranges(770u, FURROW_REAR_MM, 34u, RIGHT_TARGET_MM);
    assert(command.x == 0.0f && command.y > 0.0f);
    for(unsigned i = 0; i < 15u; ++i) step_with_ranges(770u, FURROW_REAR_MM, 37u, RIGHT_TARGET_MM);
    assert(command.x == 0.0f);
    for(unsigned i = 0; i < 15u; ++i) step_with_ranges(770u, FURROW_REAR_MM, 41u, RIGHT_TARGET_MM);
    assert(command.x > 0.0f && command.y == 0.0f);
    for(unsigned i = 0; i < 15u; ++i) step_with_ranges(770u, FURROW_REAR_MM, 776u, RIGHT_TARGET_MM);
    assert(command.x > 0.0f && command.y == 0.0f);
    puts("PASS furrow 1 return: transit only, protected wall arrival and connector restart");
}

static void step_east_wall(uint16_t range_mm) {
    /* Keep front/rear off center to detect any unwanted orthogonal correction. */
    step_with_ranges(68u, 41u, LEFT_TARGET_MM, range_mm);
}

static void prepare_eastbound_approach(unsigned leg, float remaining_m) {
    reset();
    for(unsigned i = 0; i < 30000u && app_odom_test_get_laser_fix().segment < leg; ++i) {
        advance();
        if(i % 10u == 0u) send_ideal_frame();
        process();
    }
    assert(app_odom_test_get_laser_fix().segment == leg);
    run_ideal_cycles(30u);
    position.y += app_odom_test_remaining_m() - remaining_m;
    velocity = (Vector3){ 0 };
}

static void reach_eastbound_endpoint(unsigned leg, uint16_t range_mm) {
    prepare_eastbound_approach(leg, 0.0f);
    send_ranges(68u, 41u, LEFT_TARGET_MM, range_mm);
    process();
    assert(command.x == 0.0f && command.y == 0.0f && app_odom_test_remaining_m() < 0.00001f);
}

static void test_east_wall_endpoint_trim(void) {
    const uint16_t test_range_mm[] = { 165u, 39u };
    for(unsigned test = 0; test < sizeof(test_range_mm) / sizeof(test_range_mm[0]); ++test) {
        const uint16_t initial_mm = test_range_mm[test];
        reach_eastbound_endpoint(5u, initial_mm);
        const float wheel_endpoint = position.y;
        const uint32_t stopped_ms = now_ms;
        uint32_t last_move_ms = 0u;
        for(unsigned i = 0; i < 3000u && app_odom_test_get_laser_fix().segment == 5u; ++i) {
            const uint16_t wall = (uint16_t)lroundf(initial_mm - 1000.0f * (position.y - wheel_endpoint));
            step_east_wall(wall);
            assert(command.x == 0.0f && fabsf(command.y) <= 0.04f);
            assert(initial_mm > RIGHT_TARGET_MM ? command.y >= 0.0f : command.y <= 0.0f);
            if(command.y != 0.0f) {
                assert(now_ms - stopped_ms >= 100u);
                last_move_ms = now_ms;
            }
        }
        assert(app_odom_test_get_laser_fix().segment == 6u);
        assert(last_move_ms != 0u && now_ms - last_move_ms >= 100u);
        const float final_range_mm = initial_mm - 1000.0f * (position.y - wheel_endpoint);
        assert(fabsf(final_range_mm - RIGHT_TARGET_MM) <= 1.5f);
    }
}

static void test_east_wall_trim_limits(void) {
    for(unsigned stalled = 0; stalled <= 1u; ++stalled) {
        reach_eastbound_endpoint(5u, 220u);
        const float wheel_endpoint = position.y;
        for(unsigned i = 0; i < 3000u; ++i) {
            if(stalled) position.y = wheel_endpoint;
            step_east_wall(220u);
            assert(app_odom_test_get_laser_fix().segment == 5u);
            assert(command.x == 0.0f && command.y >= 0.0f && command.y <= 0.04f);
        }
        assert(command.y == 0.0f);
        if(stalled) {
            assert(position.y - wheel_endpoint < 0.001f);
        }
        else {
            assert(position.y - wheel_endpoint > 0.140f && position.y - wheel_endpoint <= 0.150f);
        }
        /* Neither limit releases the route or retries motion. Real alignment still can. */
        for(unsigned i = 0; i < 60u; ++i) {
            step_east_wall(60u);
            assert(app_odom_test_get_laser_fix().segment == 5u && command.y == 0.0f);
        }
        for(unsigned i = 0; i < 60u && app_odom_test_get_laser_fix().segment == 5u; ++i) {
            step_east_wall(RIGHT_TARGET_MM);
        }
        assert(app_odom_test_get_laser_fix().segment == 6u);
    }
}

static void test_east_wall_trim_requires_confirmation(void) {
    /* The first crossing reverses within the same furrow after 02. */
    reach_eastbound_endpoint(2u, 104u);
    for(unsigned i = 0; i < 60u && app_odom_test_get_laser_fix().segment == 2u; ++i) {
        step_east_wall(104u);
    }
    assert(app_odom_test_get_laser_fix().segment == 3u);

    const uint16_t unavailable_mm[] = { 221u, UINT16_MAX };
    for(unsigned test = 0; test < sizeof(unavailable_mm) / sizeof(unavailable_mm[0]); ++test) {
        reach_eastbound_endpoint(5u, unavailable_mm[test]);
        for(unsigned i = 0; i < 95u; ++i) {
            step_east_wall(unavailable_mm[test]);
            assert(command.x == 0.0f && command.y == 0.0f);
            assert(app_odom_test_get_laser_fix().segment == 5u);
        }
        for(unsigned i = 0; i < 5u; ++i) step_east_wall(104u);
        assert(command.x == 0.0f && command.y > 0.0f);
    }

    /* A near-wall anchor can bring 05 into settle early, but 53 mm cannot release 06. */
    prepare_eastbound_approach(5u, 0.04f);
    for(unsigned i = 0; i < 3u; ++i) step_east_wall(53u);
    assert(app_odom_test_remaining_m() > 0.035f);
    for(unsigned i = 0; i < 65u; ++i) step_east_wall(53u);
    assert(app_odom_test_get_laser_fix().segment == 5u && command.y > 0.0f);

    /* The same in-band median can remain fresh for 100 ms. Reobserving
     * that one copy cannot stand in for multiple new CAN frames. */
    step_east_wall(RIGHT_TARGET_MM);
    step_east_wall(RIGHT_TARGET_MM);
    run_without_new_ranges(54u);
    assert(app_odom_test_get_laser_fix().segment == 5u && command.y == 0.0f);
    step_east_wall(RIGHT_TARGET_MM);
    process();
    assert(app_odom_test_get_laser_fix().segment == 5u);
    step_east_wall(RIGHT_TARGET_MM);
    process();
    assert(app_odom_test_get_laser_fix().segment == 6u);
}

static void test_east_wall_trim_safety_and_resume(void) {
    for(unsigned fault = 0; fault < 3u; ++fault) {
        reach_eastbound_endpoint(5u, 104u);
        for(unsigned i = 0; i < 60u && command.y == 0.0f; ++i) step_east_wall(104u);
        assert(command.y > 0.0f);
        if(fault == 0u) {
            run_without_new_ranges(61u);
            assert(command.x == 0.0f && command.y == 0.0f);
        }
        else if(fault == 1u) {
            /* One raw sample can brake before the median has changed. */
            step_east_wall(44u);
            assert(command.y == 0.0f && app_odom_test_get_laser_fix().jump_mask == 0u);
            step_east_wall(20u);
            assert(command.y < 0.0f && app_odom_test_get_laser_fix().jump_mask == (1u << LASER_RIGHT));
        }
        else {
            wheel_online = false;
            assert(app_odom_test_process() == APP_CONTROL_RESULT_ODOM_ERROR);
            assert(command.x == 0.0f && command.y == 0.0f);
            wheel_online = true;
        }
        for(unsigned i = 0; i < 70u; ++i) {
            step_east_wall(104u);
            assert(app_odom_test_get_laser_fix().segment == 5u);
        }
        assert(command.x == 0.0f && command.y > 0.0f);
        for(unsigned i = 0; i < 49u; ++i) {
            step_east_wall(RIGHT_TARGET_MM);
            assert(app_odom_test_get_laser_fix().segment == 5u);
        }
        for(unsigned i = 0; i < 10u && app_odom_test_get_laser_fix().segment == 5u; ++i) {
            step_east_wall(RIGHT_TARGET_MM);
        }
        assert(app_odom_test_get_laser_fix().segment == 6u);
    }
    puts("PASS east-wall endpoint: bidirectional alignment, live confirmation, safety and limits");
}

static void test_east_wall_trim_needs_a_fresh_release_frame(void) {
    reach_eastbound_endpoint(5u, 104u);
    for(unsigned i = 0; i < 60u && command.y == 0.0f; ++i) step_east_wall(104u);
    assert(command.y > 0.0f);
    for(unsigned i = 0; i < 8u; ++i) step_east_wall(RIGHT_TARGET_MM);
    assert(app_odom_test_get_laser_fix().segment == 5u && command.y == 0.0f);

    /* Multiple fresh in-band frames arrived before 100 ms. Wait past the
     * dwell without advancing the CAN stream or expiring its latest frame. */
    run_without_new_ranges(54u);
    assert(app_odom_test_get_laser_fix().segment == 5u && command.y == 0.0f);
    step_east_wall(RIGHT_TARGET_MM);
    process();
    assert(app_odom_test_get_laser_fix().segment == 6u);
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
            /* The nominal run includes 05's east wall and one narrow furrow. */
            const uint16_t front = leg == 7u ? NARROW_FRONT_MM : FURROW_FRONT_MM;
            const uint16_t rear = leg == 7u ? NARROW_REAR_MM : FURROW_REAR_MM;
            send_route_ranges(leg, leg == 1u ? FAR_WALL_MM : front, rear);
        }
        process();
        const float dx = waypoints[leg].x - waypoints[leg - 1u].x;
        const float dy = waypoints[leg].y - waypoints[leg - 1u].y;
        const float main_command = dx != 0.0f ? command.y * copysignf(1.0f, dx) :
                                                -command.x * copysignf(1.0f, dy);
        if(main_command > 0.05f) direction_mask |= 1u << (leg - 1u);
        if(leg < LEG_COUNT && app_odom_test_get_laser_fix().segment == leg + 1u) {
            const float endpoint_error = hypotf(p.x - waypoints[leg].x, p.y - waypoints[leg].y);
            max_stop_error = fmaxf(max_stop_error, endpoint_error);
            assert(endpoint_error < 0.035f);
            ++seen;
            ++leg;
            if(leg == 3u) {
                /* The simulated chassis and its IMU see the same physical yaw kick. */
                angle.z += 0.012f;
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
    test_launch_and_field_left_targets();
    test_wheel_finishes_entry_despite_laser_noise();
    test_connectors_do_not_track_previous_front_wall();
    test_crossing_laser_stability();
    test_wheel_feedback_is_mandatory();
    test_imu_is_the_only_heading_feedback();
    test_body_right_safety_after_heading_rotation();
    test_work_stop_and_resume();
    test_first_furrow_return_and_right_wall();
    test_east_wall_endpoint_trim();
    test_east_wall_trim_limits();
    test_east_wall_trim_requires_confirmation();
    test_east_wall_trim_safety_and_resume();
    test_east_wall_trim_needs_a_fresh_release_frame();
    test_route(0.0f);
    test_route(0.060f);
    puts("PASS Auto: wheel position, IMU-only yaw hold, no connector front-wall pullback");
    return 0;
}
