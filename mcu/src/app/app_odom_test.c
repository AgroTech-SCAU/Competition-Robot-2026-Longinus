#include "app_odom_test.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "laser_odom.h"
#include "log.h"
#include "odom.h"

#include <math.h>
#include <stddef.h>

#define APP_ODOM_TEST_CONTROL_PERIOD_S 0.002f
#define APP_ODOM_TEST_STOP_SPEED_MPS 0.02f

#define LASER_ODOM_FUSION_ENABLE 0
#define LASER_FUSION_GAIN 0.10f
#define LASER_CAPTURE_ERROR_MM 150.0f
#define LASER_DUAL_AGREEMENT_MM 10.0f
#define LASER_MAX_POSITION_RESIDUAL_M 0.30f
#define LASER_CORRECTION_KP 1.0f
#define LASER_CORRECTION_LIMIT_MPS 0.12f
#define LASER_CORRECTION_ACCEL_MPS2 0.6f
#define LASER_SETTLE_MS 150u
#define LASER_RELOCALIZE_TIMEOUT_MS 650u
#define LASER_STOP_TIMEOUT_MS 2000u

/* Raw range = body clearance + sensor setback from body edge.
 * Order: front, rear, left, right. Zero means the route uses raw range. */
static const float s_sensor_setback_mm[LASER_BEAM_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

typedef struct {
    float distance_m;
    float vx;
    float vy;
    LaserBeam side;
    float y_min_mm;
    float y_max_mm;
    float x_min_mm;
    float x_max_mm;
    float x_gate_mm;
    float x_landmark_offset_m;
} AppOdomTestSegment;

/* {distance_m, vx_mps, vy_mps, side, y_min_mm, y_max_mm, x_min_mm,
 *  x_max_mm, x_gate_mm, x_landmark_offset_m}. The X reference is the sum of
 *  signed X route lengths plus the landmark offset. On a VY segment, x_gate_mm enables
 *  longitudinal correction when either end face is within that clearance.
 *  Zero disables in-motion X correction. */
static const AppOdomTestSegment s_route[] = {
    { 4.773f, -1.0f,  0.00f, LASER_LEFT,  12, 12, 12, 12,  0, 0.0f }, /* 01 */
    { 2.660f,  0.0f,  0.80f, LASER_RIGHT, 11, 12, 11, 12,  0, 0.0f }, /* 02 */
    { 0.713f,  0.5f,  0.00f, LASER_RIGHT, 11, 12, 11, 12,  0, 0.0f }, /* 03 */
    { 2.660f,  0.0f, -0.80f, LASER_LEFT,  11, 12, 11, 12, 15, 0.0f }, /* 04 */
    { 0.720f,  0.5f,  0.00f, LASER_LEFT,  11, 12, 11, 12,  0, 0.0f }, /* 05 */
    { 2.660f,  0.0f,  0.80f, LASER_RIGHT, 11, 12, 11, 12, 15, 0.0f }, /* 06 */
    { 0.720f,  0.5f,  0.00f, LASER_RIGHT, 11, 12, 11, 12,  0, 0.0f }, /* 07 */
    { 2.660f,  0.0f,  0.80f, LASER_LEFT,  11, 12, 11, 12, 15, 0.0f }, /* 08 */
    { 0.720f,  0.5f,  0.00f, LASER_LEFT,  11, 12, 11, 12,  0, 0.0f }, /* 09 */
    { 2.660f,  0.0f,  0.80f, LASER_RIGHT, 11, 12, 11, 12, 15, 0.0f }, /* 10 */
    { 0.720f,  0.5f,  0.00f, LASER_RIGHT, 11, 12, 11, 12,  0, 0.0f }, /* 11 */
    { 2.660f,  0.0f, -0.80f, LASER_LEFT,  11, 12, 11, 12, 15, 0.0f }, /* 12 */
    { 0.720f,  0.5f,  0.00f, LASER_LEFT,  11, 12, 11, 12,  0, 0.0f }, /* 13 */
    { 2.550f,  0.0f,  0.80f, LASER_NO_BEAM, 0, 0, 11, 12, 15, 0.0f }, /* 14 */
    { 2.660f,  0.0f, -0.80f, LASER_NO_BEAM, 0, 0, 11, 12, 15, 0.0f }, /* 15 */
};

#define APP_ODOM_TEST_ROUTE_COUNT (sizeof(s_route) / sizeof(s_route[0]))

typedef struct {
    size_t segment_index;
    bool segment_started;
    bool segment_stopping;
    bool stationary;
    bool config_checked;
    bool config_valid;
    uint32_t stop_since_ms;
    uint32_t stationary_since_ms;
    uint32_t last_fused_frame;
    Vector3 origin;
    Vector3 route_origin;
    float waypoint_x;
    float waypoint_y;
    float heading_cos;
    float heading_sin;
    float progress_m;
    float correction_mps;
    AppLaserFix laser;
} AppOdomTestState;

static AppOdomTestState s_odom_test;

void app_odom_test_reset(void) {
    s_odom_test = (AppOdomTestState){ 0 };
}

AppLaserFix app_odom_test_get_laser_fix(void) {
    return s_odom_test.laser;
}

float app_odom_test_remaining_m(void) {
    if(s_odom_test.segment_index >= APP_ODOM_TEST_ROUTE_COUNT) {
        return 0.0f;
    }
    const float distance_m = s_route[s_odom_test.segment_index].distance_m;
    return s_odom_test.segment_started ? fmaxf(0.0f, distance_m - s_odom_test.progress_m) : distance_m;
}

static bool app_odom_test_range_config(LaserBeam beam, float min_mm, float max_mm) {
    if((unsigned int)beam >= LASER_BEAM_COUNT) return false;
    const float setback = s_sensor_setback_mm[beam];
    return isfinite(setback) && setback >= 0.0f && min_mm >= 0.0f && max_mm >= min_mm &&
           min_mm + setback >= LASER_MIN_RANGE_MM && max_mm + setback <= LASER_MAX_RANGE_MM;
}

static bool app_odom_test_check_laser_config(void) {
    for(size_t i = 0; i < APP_ODOM_TEST_ROUTE_COUNT; ++i) {
        const AppOdomTestSegment* segment = &s_route[i];
        if((segment->side != LASER_NO_BEAM &&
            !app_odom_test_range_config(segment->side, segment->y_min_mm, segment->y_max_mm)) ||
           !app_odom_test_range_config(LASER_FRONT, segment->x_min_mm, segment->x_max_mm) ||
           (i != 0u && !app_odom_test_range_config(LASER_REAR, segment->x_min_mm, segment->x_max_mm)) ||
           !isfinite(segment->x_landmark_offset_m) || !isfinite(segment->x_gate_mm) ||
           (segment->x_gate_mm != 0.0f && segment->x_gate_mm < segment->x_max_mm)) {
            log_error("LASER CONFIG segment=%u target+setback must be %u..%u mm; AUTO held",
                      (unsigned int)(i + 1u), LASER_MIN_RANGE_MM, LASER_MAX_RANGE_MM);
            return false;
        }
    }
    return true;
}

static bool app_odom_test_observe(const LaserOdomSample* sample, LaserBeam beam,
                                 float waypoint_m, float reference_mm, float gate_mm, float* position_m) {
    const float clearance_mm = (float)sample->distance_mm[beam] - s_sensor_setback_mm[beam];
    if(!(sample->valid_mask & (1u << beam)) || clearance_mm < 0.0f ||
       fabsf(clearance_mm - reference_mm) > LASER_CAPTURE_ERROR_MM ||
       (gate_mm > 0.0f && clearance_mm > gate_mm)) {
        return false;
    }
    *position_m = laser_odom_axis_position(beam, waypoint_m, reference_mm, clearance_mm);
    return true;
}

static AppLaserFix app_odom_test_laser_fix(const AppOdomTestSegment* segment,
                                         const LaserOdomSample* sample, Vector3 position) {
    AppLaserFix fix = { .segment = (unsigned int)s_odom_test.segment_index + 1u };
    const float x_ref = 0.5f * (segment->x_min_mm + segment->x_max_mm);
    const float y_ref = 0.5f * (segment->y_min_mm + segment->y_max_mm);
    const float x_landmark_m = s_odom_test.waypoint_x + segment->x_landmark_offset_m;
    const float dx = position.x - s_odom_test.route_origin.x;
    const float dy = position.y - s_odom_test.route_origin.y;
    const float x_pred = s_odom_test.heading_cos * dx + s_odom_test.heading_sin * dy;
    const float y_pred = -s_odom_test.heading_sin * dx + s_odom_test.heading_cos * dy;
    float front_m = 0.0f;
    float rear_m = 0.0f;
    const bool front = app_odom_test_observe(sample, LASER_FRONT, x_landmark_m,
                                            x_ref, segment->x_gate_mm, &front_m);
    const bool rear = s_odom_test.segment_index != 0u &&
        app_odom_test_observe(sample, LASER_REAR, x_landmark_m,
                              x_ref, segment->x_gate_mm, &rear_m);
    if(front && rear) {
        if(fabsf(front_m - rear_m) <= LASER_DUAL_AGREEMENT_MM * 0.001f) {
            fix.x = 0.5f * (front_m + rear_m);
            fix.mask |= APP_LASER_X;
        }
    }
    else if(front || rear) {
        fix.x = front ? front_m : rear_m;
        fix.mask |= APP_LASER_X;
    }
    if(segment->side != LASER_NO_BEAM &&
       app_odom_test_observe(sample, segment->side, s_odom_test.waypoint_y, y_ref, 0.0f, &fix.y)) {
        fix.mask |= APP_LASER_Y;
    }
    if(fabsf(fix.x - x_pred) > LASER_MAX_POSITION_RESIDUAL_M) fix.mask &= ~APP_LASER_X;
    if(fabsf(fix.y - y_pred) > LASER_MAX_POSITION_RESIDUAL_M) fix.mask &= ~APP_LASER_Y;
    return fix;
}

static OdomStatus app_odom_test_apply_fix(Vector3 position, AppLaserFix fix, float gain) {
    const float c = s_odom_test.heading_cos;
    const float s = s_odom_test.heading_sin;
    const float dx = position.x - s_odom_test.route_origin.x;
    const float dy = position.y - s_odom_test.route_origin.y;
    const float ex = (fix.mask & APP_LASER_X) ? fix.x - (c * dx + s * dy) : 0.0f;
    const float ey = (fix.mask & APP_LASER_Y) ? fix.y - (-s * dx + c * dy) : 0.0f;
    return odom_relocalize_xy(position.x + gain * (c * ex - s * ey),
                              position.y + gain * (s * ex + c * ey));
}

static AppControlResult app_odom_test_finish_segment(const AppOdomTestSegment* segment,
                                                     Vector3 position, const LaserOdomSample* sample) {
    const AppControlResult stop = app_control_hold_zero_chassis();
    if(stop != APP_CONTROL_RESULT_OK) return stop;

    Vector3 velocity = { 0 };
    if(odom.get_velocity(&velocity) != ODOM_OK || !isfinite(velocity.x) || !isfinite(velocity.y)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    const uint32_t now = delay_now_ms();
    if(hypotf(velocity.x, velocity.y) > APP_ODOM_TEST_STOP_SPEED_MPS) {
        s_odom_test.stationary = false;
        return now - s_odom_test.stop_since_ms >= LASER_STOP_TIMEOUT_MS ? APP_CONTROL_RESULT_ODOM_ERROR : stop;
    }
    if(!s_odom_test.stationary) {
        s_odom_test.stationary = true;
        s_odom_test.stationary_since_ms = now;
        return stop;
    }

    AppLaserFix fix = s_odom_test.laser;
    if((int32_t)(sample->stamp_ms - s_odom_test.stationary_since_ms) < (int32_t)LASER_SETTLE_MS) {
        fix.mask = 0u;
    }
    const unsigned int expected_mask = segment->side == LASER_NO_BEAM ? APP_LASER_X : APP_LASER_X | APP_LASER_Y;
    if(fix.mask != expected_mask &&
       now - s_odom_test.stationary_since_ms < LASER_RELOCALIZE_TIMEOUT_MS) {
        return stop;
    }
    if(fix.mask == 0u) {
        log_warn("LASER RELOC seg=%u timed out without a usable axis", fix.segment);
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    if(app_odom_test_apply_fix(position, fix, 1.0f) != ODOM_OK) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    log_info("LASER RELOC seg=%u mask=0x%X x=%.3f y=%.3f progress=%.3f",
             fix.segment, fix.mask, (double)((fix.mask & APP_LASER_X) ? fix.x : NAN),
             (double)((fix.mask & APP_LASER_Y) ? fix.y : NAN), (double)s_odom_test.progress_m);
    ++s_odom_test.segment_index;
    s_odom_test.segment_started = false;
    s_odom_test.segment_stopping = false;
    s_odom_test.stationary = false;
    s_odom_test.correction_mps = 0.0f;
    return stop;
}

static float app_odom_test_correction(float error_m, float deadband_mm) {
    const float excess = fmaxf(0.0f, fabsf(error_m) - deadband_mm * 0.001f);
    return -copysignf(fminf(LASER_CORRECTION_KP * excess, LASER_CORRECTION_LIMIT_MPS), error_m);
}

AppControlResult app_odom_test_process(void) {
    Vector3 position = { 0 };
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };

    if(s_odom_test.segment_index >= APP_ODOM_TEST_ROUTE_COUNT) {
        s_odom_test.laser.mask = 0u;
        return app_control_hold_zero_chassis();
    }
    if(!s_odom_test.config_checked) {
        s_odom_test.config_valid = app_odom_test_check_laser_config();
        s_odom_test.config_checked = true;
    }
    if(!s_odom_test.config_valid) return app_control_hold_zero_chassis();
    if(!laser_odom_is_online()) {
        s_odom_test.laser.mask = 0u;
        return s_odom_test.segment_started ? APP_CONTROL_RESULT_ODOM_ERROR : app_control_hold_zero_chassis();
    }

    const AppOdomTestSegment* segment = &s_route[s_odom_test.segment_index];
    const float distance_m = segment->distance_m;
    const float vx = segment->vx;
    const float vy = segment->vy;
    const bool test_vy = vy != 0.0f;
    const float speed_mps = test_vy ? vy : vx;

    if(!isfinite(distance_m) || distance_m <= 0.0f ||
       !isfinite(speed_mps) || speed_mps == 0.0f || (test_vy && vx != 0.0f)) {
        return APP_CONTROL_RESULT_COMMAND_INVALID;
    }
    if(odom.get_odom(&position) != ODOM_OK || odom.get_angle(&angle) != ODOM_OK ||
       odom.get_gyro_corrected(&gyro) != ODOM_OK ||
       !isfinite(position.x) || !isfinite(position.y) ||
       !isfinite(angle.z) || !isfinite(gyro.z)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }

    if(!s_odom_test.segment_started) {
        if(s_odom_test.segment_index == 0u) {
            s_odom_test.heading_cos = cosf(angle.z);
            s_odom_test.heading_sin = sinf(angle.z);
            s_odom_test.route_origin = position;
            chassis_yaw_hold_set_target(angle.z);
        }
        s_odom_test.origin = position;
        if(test_vy) {
            s_odom_test.waypoint_y += copysignf(distance_m, vy);
        }
        else {
            s_odom_test.waypoint_x += copysignf(distance_m, vx);
        }
        s_odom_test.segment_started = true;
        log_info("ODOM ROUTE %u/%u start: d=%.2fm vx=%.2fm/s vy=%.2fm/s",
                 (unsigned int)(s_odom_test.segment_index + 1u), (unsigned int)APP_ODOM_TEST_ROUTE_COUNT,
                 (double)distance_m, (double)vx, (double)vy);
    }

    const LaserOdomSample* sample = laser_odom_get_sample();
    s_odom_test.laser = app_odom_test_laser_fix(segment, sample, position);
    if(s_odom_test.segment_stopping) return app_odom_test_finish_segment(segment, position, sample);

    /* Project the displacement onto the route's initial body axes. */
    if(test_vy) {
        s_odom_test.progress_m = -(position.x - s_odom_test.origin.x) * s_odom_test.heading_sin +
                                  (position.y - s_odom_test.origin.y) * s_odom_test.heading_cos;
    }
    else {
        s_odom_test.progress_m = (position.x - s_odom_test.origin.x) * s_odom_test.heading_cos +
                                 (position.y - s_odom_test.origin.y) * s_odom_test.heading_sin;
    }
    s_odom_test.progress_m *= speed_mps > 0.0f ? 1.0f : -1.0f;
    if(!isfinite(s_odom_test.progress_m)) return APP_CONTROL_RESULT_ODOM_ERROR;
    if(s_odom_test.progress_m >= distance_m) {
        s_odom_test.segment_stopping = true;
        s_odom_test.stop_since_ms = delay_now_ms();
        return app_odom_test_finish_segment(segment, position, sample);
    }
    if(!test_vy && !(s_odom_test.laser.mask & APP_LASER_Y)) {
        s_odom_test.correction_mps = 0.0f;
        return app_control_hold_zero_chassis();
    }

    if(LASER_ODOM_FUSION_ENABLE && sample->received != s_odom_test.last_fused_frame) {
        s_odom_test.last_fused_frame = sample->received;
        const AppLaserFix fix = s_odom_test.laser;
        if(fix.mask != 0u && app_odom_test_apply_fix(position, fix, LASER_FUSION_GAIN) != ODOM_OK) {
            return APP_CONTROL_RESULT_ODOM_ERROR;
        }
    }

    float correction = 0.0f;
    if(test_vy && segment->x_gate_mm > 0.0f && (s_odom_test.laser.mask & APP_LASER_X)) {
        const float x_landmark_m = s_odom_test.waypoint_x + segment->x_landmark_offset_m;
        correction = app_odom_test_correction(s_odom_test.laser.x - x_landmark_m,
                                              0.5f * (segment->x_max_mm - segment->x_min_mm));
    }
    if(!test_vy && (s_odom_test.laser.mask & APP_LASER_Y)) {
        correction = app_odom_test_correction(s_odom_test.laser.y - s_odom_test.waypoint_y,
                                              0.5f * (segment->y_max_mm - segment->y_min_mm));
    }
    const float step = LASER_CORRECTION_ACCEL_MPS2 * APP_ODOM_TEST_CONTROL_PERIOD_S;
    const float previous = s_odom_test.correction_mps;
    s_odom_test.correction_mps = fmaxf(previous - step, fminf(previous + step, correction));
    const float vx_cmd = vx + (test_vy ? s_odom_test.correction_mps : 0.0f);
    const float vy_cmd = vy + (test_vy ? 0.0f : s_odom_test.correction_mps);
    const float wz = chassis_yaw_hold_apply(vx_cmd, vy_cmd, 0.0f, angle.z, gyro.z,
                                             APP_ODOM_TEST_CONTROL_PERIOD_S);
    return chassis.set_velocity(vx_cmd, vy_cmd, wz) == CHASSIS_OK
        ? APP_CONTROL_RESULT_OK : APP_CONTROL_RESULT_CHASSIS_ERROR;
}
