#include "app_odom_test.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "laser_field.h"
#include "laser_odom.h"
#include "log.h"
#include "odom.h"

#include <math.h>
#include <stddef.h>

#define APP_ODOM_TEST_CONTROL_PERIOD_S 0.002f
#define APP_ODOM_TEST_STOP_SPEED_MPS 0.02f
#define APP_ODOM_TEST_CORRECTION_START_M 0.01f
#define LASER_VY_DELAY_MS 10u

#define LASER_MM_PER_CM 10.0f
#define LASER_CAPTURE_ERROR_MM 150.0f
#define LASER_DUAL_AGREEMENT_MM 10.0f
#define LASER_MAX_POSITION_RESIDUAL_M 0.20f
#define LASER_TRACK_JUMP_MM 12.0f
#define LASER_TRACK_STABLE_MM 5.0f
#define LASER_TRACK_CONFIRM_FRAMES 3u
#define LASER_TRACK_CONFIRM_MS 100u
#define LASER_TRACK_GAIN 0.5f
#define LASER_TRACK_MAX_AGE_MS 120u
#define ODOM_LINE_DEADBAND_M 0.005f
#define LASER_SETTLE_MS 150u
#define LASER_RELOCALIZE_TIMEOUT_MS 650u
#define LASER_STOP_TIMEOUT_MS 2000u
#define LASER_RELOCALIZE_GAIN 0.15f
#define LASER_RELOCALIZE_LIMIT_M 0.005f
#define LASER_END_TRIM_LIMIT_M 0.05f
#define LASER_END_TRIM_INSET_M 0.01f
#define LASER_END_TRIM_TIMEOUT_MS 2000u
#define LASER_END_YAW_LIMIT_RAD_S 0.15f

typedef struct {
    float kp;
    float limit_mps;
    float accel_mps2;
} AppLaserCorrection;

/* Gain, speed limit and acceleration limit, named by the corrected axis. */
static const AppLaserCorrection s_x_correction = { 2.2f, 0.08f, 0.5f }; /* vy travel */
static const AppLaserCorrection s_y_correction = { 2.2f, 0.05f, 0.3f }; /* vx travel */
static const AppLaserCorrection s_odom_correction = { 2.1f, 0.03f, 0.3f };
static const AppLaserCorrection s_end_y_correction = { 1.5f, 0.04f, 0.20f };

/* Raw range = body clearance + sensor setback from body edge.
 * Order: front, rear, left, right. Zero means the route uses raw range. */
static const float s_sensor_setback_mm[LASER_BEAM_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

typedef struct {
    float min_mm;
    float max_mm;   
    float capture_max_mm;
    float direction;
} AppLaserBand;

/* Moving clearance bands in mm. Direction is the sign of the beam on its axis. */
static const AppLaserBand s_side_band = { 45.0f, 50.0f, 197.5f, 1.0f };
static const AppLaserBand s_front_band = { 30.0f, 35.0f, 65.0f, -1.0f };
static const AppLaserBand s_rear_band = { 25.0f, 30.0f, 60.0f,  1.0f };

typedef struct {
    float wall_mm;
    float accepted_wall_mm;
    float candidate_mm;
    uint32_t candidate_since_ms;
    unsigned int confirmations;
    bool anchored;
    bool usable;
} AppLaserTrack;

typedef struct {
    float distance_m;
    float vx;
    float vy;
    LaserBeam side;
    float y_min_cm;
    float y_max_cm;
    float x_min_cm;
    float x_max_cm;
    float x_gate_cm;
    float x_landmark_offset_m;
} AppOdomTestSegment;

/* End-trim and stopped-localization bands are in cm, as is the optional X gate.
 * A zero X gate uses the ordinary capture window. The X landmark is the
 * cumulative signed X distance plus x_landmark_offset_m. */
static const AppOdomTestSegment s_route[] = {
    { 4.77f, -0.45f,  0.00f, LASER_LEFT,  52, 55, 45, 50, 0, 0.0f }, /* 01 */
    { 1.78f,  0.0f,  0.30f, LASER_RIGHT, 50, 55, 21, 25,  0, 0.0f }, /* 02 */
    { 1.72f,  0.0f,  -0.30f, LASER_RIGHT, 45, 50, 36, 40,  0, 0.0f }, /* 03 */
    { 0.71f,  0.3f, 0.00f, LASER_LEFT,  60, 63, 50, 55, 0, 0.0f }, /* 04 */
    { 2.78f,  0.0f,  0.30f, LASER_LEFT,  30, 35, 45, 50, 0, 0.0f }, /* 05 */
    { 0.71f,  0.3f,  0.00f, LASER_RIGHT, 25, 30, 50, 55, 0, 0.0f }, /* 06 */
    { 2.765f,  0.0f,  -0.30f, LASER_RIGHT, 45, 58, 46, 51,  0, 0.0f }, /* 07 */
    { 0.72f,  0.3f, 0.00f, LASER_LEFT,  80, 83, 50, 55, 0, 0.0f }, /* 08 */
    { 2.76f,  0.0f,  0.30f, LASER_LEFT,  45, 58, 35, 39,  0, 0.0f }, /* 09 */
    { 0.72f,  0.3f,  0.00f, LASER_RIGHT, 25, 30, 50, 55, 0, 0.0f }, /* 10 */
    { 2.76f,  0.0f,  -0.30f, LASER_RIGHT, 45, 58, 45, 48,  0, 0.0f }, /* 11 */
    { 0.72f,  0.3f, 0.00f, LASER_LEFT,  26, 30, 50, 55, 0, 0.0f }, /* 12 */
    { 2.75f,  0.0f,  0.30f, LASER_LEFT,  48, 58, 36, 40,  0, 0.0f }, /* 13 */
    { 0.72f,  0.3f,  0.00f, LASER_RIGHT, 28, 33, 48, 55, 0, 0.0f }, /* 14 */
    { 2.75f,  0.0f, -0.30f, LASER_RIGHT, 45, 50, 40, 45, 0, 0.0f }, /* 15 */
    { 0.8f,  0.0f, -0.50f, LASER_LEFT, 45, 50, 36, 45, 0, 0.0f }, /* 16 */
};

#define APP_ODOM_TEST_ROUTE_COUNT (sizeof(s_route) / sizeof(s_route[0]))

typedef struct {
    size_t segment_index;
    bool segment_started;
    bool segment_stopping;
    bool stationary;
    bool motion_started;
    bool segment_moving;
    bool laser_enabled;
    bool end_trim_started;
    bool end_trim_done;
    uint32_t motion_since_ms;
    uint32_t frame_at_motion_start;
    uint32_t stop_since_ms;
    uint32_t stationary_since_ms;
    uint32_t last_tracked_frame;
    uint32_t last_control_ms;
    uint32_t end_trim_since_ms;
    uint32_t end_trim_accepted_ms;
    Vector3 origin;
    Vector3 route_origin;
    float waypoint_x;
    float waypoint_y;
    float heading_cos;
    float heading_sin;
    float progress_m;
    float correction_mps;
    float line_cross_m;
    float end_trim_origin_y_m;
    bool following_laser;
    AppLaserTrack tracks[LASER_BEAM_COUNT];
    AppLaserFix laser;
} AppOdomTestState;

static AppOdomTestState s_odom_test;

void app_odom_test_reset(void) {
    s_odom_test = (AppOdomTestState){ 0 };
    laser_field_reset();
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

static bool app_odom_test_observe(const LaserOdomSample* sample, LaserBeam beam,
                                 float waypoint_m, float min_mm, float max_mm, float gate_mm, float* position_m) {
    if(!app_odom_test_range_config(beam, min_mm, max_mm) || !isfinite(waypoint_m) ||
       !isfinite(gate_mm) || (gate_mm != 0.0f && gate_mm < max_mm)) {
        return false;
    }
    const float reference_mm = 0.5f * (min_mm + max_mm);
    const float clearance_mm = (float)sample->distance_mm[beam] - s_sensor_setback_mm[beam];
    if(!(sample->valid_mask & (1u << beam)) || clearance_mm < 0.0f ||
       fabsf(clearance_mm - reference_mm) > LASER_CAPTURE_ERROR_MM ||
       (gate_mm > 0.0f && clearance_mm > gate_mm)) {
        return false;
    }
    /* VOFA F faces -vx and B faces +vx. The legacy helper uses +front. */
    LaserBeam direction = beam;
    if(beam == LASER_FRONT) direction = LASER_REAR;
    else if(beam == LASER_REAR) direction = LASER_FRONT;
    *position_m = laser_odom_axis_position(direction, waypoint_m, reference_mm, clearance_mm);
    return true;
}

static AppLaserFix app_odom_test_laser_fix(const AppOdomTestSegment* segment,
                                         const LaserOdomSample* sample, Vector3 position) {
    AppLaserFix fix = { .segment = (unsigned int)s_odom_test.segment_index + 1u };
    const bool first_x = s_odom_test.segment_index == 0u && segment->vx != 0.0f;
    const float x_min_mm = segment->x_min_cm * LASER_MM_PER_CM;
    const float x_max_mm = segment->x_max_cm * LASER_MM_PER_CM;
    const float x_gate_mm = segment->x_gate_cm * LASER_MM_PER_CM;
    const float x_landmark_m = s_odom_test.waypoint_x + segment->x_landmark_offset_m;
    const float dx = position.x - s_odom_test.route_origin.x;
    const float dy = position.y - s_odom_test.route_origin.y;
    const float x_pred = s_odom_test.heading_cos * dx + s_odom_test.heading_sin * dy;
    const float y_pred = -s_odom_test.heading_sin * dx + s_odom_test.heading_cos * dy;
    float front_m = 0.0f;
    float rear_m = 0.0f;
    const bool front = app_odom_test_observe(sample, LASER_FRONT, x_landmark_m,
                                            x_min_mm, x_max_mm, x_gate_mm, &front_m);
    const bool rear = !first_x &&
        app_odom_test_observe(sample, LASER_REAR, x_landmark_m, x_min_mm, x_max_mm, x_gate_mm, &rear_m);
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
       app_odom_test_observe(sample, segment->side, s_odom_test.waypoint_y,
                             segment->y_min_cm * LASER_MM_PER_CM, segment->y_max_cm * LASER_MM_PER_CM,
                             0.0f, &fix.y)) {
        fix.mask |= APP_LASER_Y;
    }
    if(fabsf(fix.x - x_pred) > LASER_MAX_POSITION_RESIDUAL_M) fix.mask &= ~APP_LASER_X;
    if(fabsf(fix.y - y_pred) > LASER_MAX_POSITION_RESIDUAL_M) fix.mask &= ~APP_LASER_Y;
    return fix;
}

static OdomStatus app_odom_test_apply_fix(Vector3 position, AppLaserFix fix) {
    const float c = s_odom_test.heading_cos;
    const float s = s_odom_test.heading_sin;
    const float dx = position.x - s_odom_test.route_origin.x;
    const float dy = position.y - s_odom_test.route_origin.y;
    const float ex = (fix.mask & APP_LASER_X) ? LASER_RELOCALIZE_GAIN * (fix.x - (c * dx + s * dy)) : 0.0f;
    const float ey = (fix.mask & APP_LASER_Y) ? LASER_RELOCALIZE_GAIN * (fix.y - (-s * dx + c * dy)) : 0.0f;
    const float x = fmaxf(-LASER_RELOCALIZE_LIMIT_M, fminf(LASER_RELOCALIZE_LIMIT_M, ex));
    const float y = fmaxf(-LASER_RELOCALIZE_LIMIT_M, fminf(LASER_RELOCALIZE_LIMIT_M, ey));
    return odom_relocalize_xy(position.x + c * x - s * y, position.y + s * x + c * y);
}

static AppControlResult app_odom_test_finish_segment(const AppOdomTestSegment* segment,
                                                     Vector3 position, const LaserOdomSample* sample,
                                                     bool side_filtered) {
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
    const unsigned int expected_mask = segment->side == LASER_NO_BEAM || side_filtered
        ? APP_LASER_X : APP_LASER_X | APP_LASER_Y;
    if(fix.mask != expected_mask &&
       now - s_odom_test.stationary_since_ms < LASER_RELOCALIZE_TIMEOUT_MS) {
        return stop;
    }
    if(fix.mask != 0u) {
        if(app_odom_test_apply_fix(position, fix) != ODOM_OK) return APP_CONTROL_RESULT_ODOM_ERROR;
        log_info("LASER RELOC seg=%u mask=0x%X x=%.3f y=%.3f progress=%.3f",
                 fix.segment, fix.mask, (double)((fix.mask & APP_LASER_X) ? fix.x : NAN),
                 (double)((fix.mask & APP_LASER_Y) ? fix.y : NAN), (double)s_odom_test.progress_m);
    }
    else {
        log_warn("LASER RELOC seg=%u no usable axis; odom retained", fix.segment);
    }
    ++s_odom_test.segment_index;
    s_odom_test.segment_started = false;
    s_odom_test.segment_stopping = false;
    s_odom_test.stationary = false;
    s_odom_test.correction_mps = 0.0f;
    return stop;
}

static void app_odom_test_track(AppLaserTrack* track, const LaserOdomSample* sample,
                                LaserBeam beam, AppLaserBand band, float axis_m) {
    const float raw_mm = (float)sample->distance_mm[beam];
    const float clearance_mm = raw_mm - s_sensor_setback_mm[beam];
    track->usable = false;
    if(raw_mm < LASER_FIELD_MIN_RANGE_MM || clearance_mm < 0.0f || raw_mm > LASER_MAX_RANGE_MM) {
        track->anchored = false;
        track->confirmations = 0u;
        return;
    }

    /* A stationary reflector has constant axis position even as the chassis moves. */
    const float wall_mm = axis_m * 1000.0f + band.direction * clearance_mm;
    if(track->anchored && fabsf(wall_mm - track->wall_mm) <= LASER_TRACK_JUMP_MM) {
        track->wall_mm += LASER_TRACK_GAIN * (wall_mm - track->wall_mm);
        track->accepted_wall_mm = wall_mm;
        track->confirmations = 0u;
        track->usable = true;
        return;
    }
    if(clearance_mm > band.capture_max_mm) {
        track->confirmations = 0u;
        return;
    }

    if(track->confirmations == 0u || fabsf(wall_mm - track->candidate_mm) > LASER_TRACK_STABLE_MM) {
        track->candidate_mm = wall_mm;
        track->candidate_since_ms = sample->stamp_ms;
        track->confirmations = 1u;
    }
    else if(track->confirmations < LASER_TRACK_CONFIRM_FRAMES) {
        ++track->confirmations;
    }
    if(track->confirmations >= LASER_TRACK_CONFIRM_FRAMES &&
       sample->stamp_ms - track->candidate_since_ms >= LASER_TRACK_CONFIRM_MS) {
        track->wall_mm = wall_mm;
        track->accepted_wall_mm = wall_mm;
        track->anchored = track->usable = true;
        track->confirmations = 0u;
    }
}

static void app_odom_test_motion_interval(AppLaserFix* fix, LaserBeam beam, AppLaserBand band,
                                          float axis_m, float* low_mm, float* high_mm) {
    const AppLaserTrack* track = &s_odom_test.tracks[beam];
    if(track->anchored && track->confirmations != 0u) fix->jump_mask |= 1u << beam;
    if(!track->usable) return;

    const float clearance_mm = band.direction * (track->accepted_wall_mm - axis_m * 1000.0f);
    const float lower = band.direction * (clearance_mm - band.max_mm);
    const float upper = band.direction * (clearance_mm - band.min_mm);
    *low_mm = fmaxf(*low_mm, fminf(lower, upper));
    *high_mm = fminf(*high_mm, fmaxf(lower, upper));
    fix->track_mask |= 1u << beam;
}

static float app_odom_test_motion_error(AppLaserFix* fix, bool test_vy, float cross_m,
                                        float low_mm, float high_mm) {
    if(fix->track_mask == 0u || low_mm - high_mm > 0.001f) return 0.0f;

    const float error_m = (low_mm > 0.0f ? low_mm : fminf(0.0f, high_mm)) * 0.001f;
    fix->mask = test_vy ? APP_LASER_X : APP_LASER_Y;
    const float center_m = 0.0005f * (low_mm + high_mm);
    const float dx = s_odom_test.origin.x - s_odom_test.route_origin.x;
    const float dy = s_odom_test.origin.y - s_odom_test.route_origin.y;
    if(test_vy) fix->x = s_odom_test.heading_cos * dx + s_odom_test.heading_sin * dy + cross_m + center_m;
    else fix->y = -s_odom_test.heading_sin * dx + s_odom_test.heading_cos * dy + cross_m + center_m;
    return error_m;
}

static float app_odom_test_motion_fix(AppLaserFix* fix, const LaserOdomSample* sample, bool test_vy, LaserBeam side,
                                      float cross_m, float sample_cross_m, uint32_t age_ms) {
    if(age_ms > LASER_TRACK_MAX_AGE_MS) {
        s_odom_test.last_tracked_frame = 0u;
        for(unsigned int beam = 0u; beam < LASER_BEAM_COUNT; ++beam) {
            s_odom_test.tracks[beam] = (AppLaserTrack){ 0 };
        }
        return 0.0f;
    }
    const bool has_side = side == LASER_LEFT || side == LASER_RIGHT;
    AppLaserBand side_band = s_side_band;
    if(side == LASER_RIGHT) side_band.direction = -1.0f;
    if(sample->received != s_odom_test.last_tracked_frame) {
        s_odom_test.last_tracked_frame = sample->received;
        if(test_vy) {
            app_odom_test_track(&s_odom_test.tracks[LASER_FRONT], sample, LASER_FRONT, s_front_band, sample_cross_m);
            app_odom_test_track(&s_odom_test.tracks[LASER_REAR], sample, LASER_REAR, s_rear_band, sample_cross_m);
        }
        else if(has_side) {
            app_odom_test_track(&s_odom_test.tracks[side], sample, side, side_band, sample_cross_m);
        }
    }

    float low_mm = -INFINITY;
    float high_mm = INFINITY;
    if(test_vy) {
        app_odom_test_motion_interval(fix, LASER_FRONT, s_front_band, cross_m, &low_mm, &high_mm);
        app_odom_test_motion_interval(fix, LASER_REAR, s_rear_band, cross_m, &low_mm, &high_mm);
        const unsigned int required = (1u << LASER_FRONT) | (1u << LASER_REAR);
        if((fix->track_mask & required) != required) return 0.0f;
    }
    else if(has_side) {
        app_odom_test_motion_interval(fix, side, side_band, cross_m, &low_mm, &high_mm);
    }
    return app_odom_test_motion_error(fix, test_vy, cross_m, low_mm, high_mm);
}

static float app_odom_test_slew_correction(float error_m, AppLaserCorrection tuning, float previous, uint32_t now) {
    const float target_mps = fminf(tuning.limit_mps, tuning.kp * fabsf(error_m));
    const float elapsed_s = fminf(0.020f, (float)(now - s_odom_test.last_control_ms) * 0.001f);
    const float step = tuning.accel_mps2 * elapsed_s;
    /* Release the correction immediately when it enters the band or changes sign. */
    const float same_direction_mps = previous * error_m > 0.0f ? fabsf(previous) : 0.0f;
    s_odom_test.last_control_ms = now;
    return copysignf(fminf(target_mps, same_direction_mps + step), error_m);
}

static AppControlResult app_odom_test_finish_trim(const LaserOdomSample* sample, LaserBeam side, const char* reason) {
    s_odom_test.end_trim_done = true;
    s_odom_test.correction_mps = 0.0f;
    log_info("LASER END seg=%u side=%c raw=%u mm result=%s", s_odom_test.laser.segment,
             side == LASER_LEFT ? 'L' : 'R', (unsigned int)sample->distance_mm[side], reason);
    return APP_CONTROL_RESULT_SKIPPED;
}

static bool app_odom_test_end_trim_range(const AppOdomTestSegment* segment, const LaserOdomSample* sample) {
    const LaserBeam side = segment->side;
    const float raw_mm = (float)sample->distance_mm[side];
    const float clearance_mm = raw_mm - s_sensor_setback_mm[side];
    const float margin_mm = LASER_END_TRIM_LIMIT_M * 1000.0f;
    return raw_mm >= LASER_FIELD_MIN_RANGE_MM && raw_mm <= LASER_MAX_RANGE_MM && clearance_mm >= 0.0f &&
           clearance_mm >= segment->y_min_cm * LASER_MM_PER_CM - margin_mm &&
           clearance_mm <= segment->y_max_cm * LASER_MM_PER_CM + margin_mm;
}

static AppControlResult app_odom_test_trim_end(const AppOdomTestSegment* segment, const LaserOdomSample* sample,
                                               float cross_m, float yaw, float gyro_z, uint32_t now) {
    const LaserBeam side = segment->side;
    Vector3 velocity = { 0 };
    if(odom.get_velocity(&velocity) != ODOM_OK || !isfinite(velocity.x) || !isfinite(velocity.y)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    const bool stopped = hypotf(velocity.x, velocity.y) <= APP_ODOM_TEST_STOP_SPEED_MPS;
    AppLaserTrack* track = &s_odom_test.tracks[side];
    if(!s_odom_test.end_trim_started) {
        if(!stopped) {
            const AppControlResult stop = app_control_hold_zero_chassis();
            return stop == APP_CONTROL_RESULT_OK && now - s_odom_test.stop_since_ms >= LASER_STOP_TIMEOUT_MS
                ? APP_CONTROL_RESULT_ODOM_ERROR : stop;
        }
        s_odom_test.end_trim_started = true;
        s_odom_test.end_trim_since_ms = now;
        s_odom_test.end_trim_origin_y_m = cross_m;
        s_odom_test.stationary = true;
        s_odom_test.stationary_since_ms = now;
        s_odom_test.last_tracked_frame = sample->received;
        s_odom_test.last_control_ms = now;
        s_odom_test.correction_mps = 0.0f;
        *track = (AppLaserTrack){ 0 };
        return app_control_hold_zero_chassis();
    }

    if(now - s_odom_test.end_trim_since_ms >= LASER_END_TRIM_TIMEOUT_MS ||
       fabsf(cross_m - s_odom_test.end_trim_origin_y_m) >= LASER_END_TRIM_LIMIT_M) {
        return app_odom_test_finish_trim(sample, side, "limit");
    }

    const float min_mm = segment->y_min_cm * LASER_MM_PER_CM;
    const float max_mm = segment->y_max_cm * LASER_MM_PER_CM;
    const AppLaserBand band = { min_mm, max_mm, max_mm + 1000.0f * LASER_END_TRIM_LIMIT_M,
                               side == LASER_LEFT ? 1.0f : -1.0f };
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);
    const float c = cy * s_odom_test.heading_cos + sy * s_odom_test.heading_sin;
    const float s = sy * s_odom_test.heading_cos - cy * s_odom_test.heading_sin;
    const uint32_t age_ms = now - sample->stamp_ms;
    if((track->usable && now - s_odom_test.end_trim_accepted_ms > LASER_TRACK_MAX_AGE_MS) ||
       age_ms > LASER_TRACK_MAX_AGE_MS) {
        *track = (AppLaserTrack){ 0 };
    }
    if(age_ms <= LASER_TRACK_MAX_AGE_MS &&
       (int32_t)(sample->stamp_ms - s_odom_test.end_trim_since_ms) >= (int32_t)LASER_SETTLE_MS) {
        if(!app_odom_test_end_trim_range(segment, sample)) {
            /* A rejected frame cannot extend the lifetime of the accepted reference. */
            if(!track->usable) return app_odom_test_finish_trim(sample, side, "filtered");
        }
        else if(sample->received != s_odom_test.last_tracked_frame) {
            const float sample_cross_m = cross_m - (s * velocity.x + c * velocity.y) * ((float)age_ms * 0.001f);
            app_odom_test_track(track, sample, side, band, sample_cross_m);
            if(track->usable) s_odom_test.end_trim_accepted_ms = sample->stamp_ms;
            s_odom_test.last_tracked_frame = sample->received;
        }
    }

    float low_mm = -INFINITY;
    float high_mm = INFINITY;
    AppLaserFix* fix = &s_odom_test.laser;
    app_odom_test_motion_interval(fix, side, band, cross_m, &low_mm, &high_mm);
    float error_m = app_odom_test_motion_error(fix, false, cross_m, low_mm, high_mm);
    if(fabsf(error_m) > LASER_END_TRIM_LIMIT_M) {
        return app_odom_test_finish_trim(sample, side, "filtered");
    }
    if(error_m == 0.0f && stopped) {
        if(!s_odom_test.stationary) {
            s_odom_test.stationary = true;
            s_odom_test.stationary_since_ms = now;
        }
        if(fix->mask != 0u &&
           (int32_t)(s_odom_test.end_trim_accepted_ms - s_odom_test.stationary_since_ms) >= (int32_t)LASER_SETTLE_MS) {
            return app_odom_test_finish_trim(sample, side, "in-band");
        }
    }
    else {
        s_odom_test.stationary = false;
    }

    if(error_m != 0.0f) {
        /* Aim just inside the band so the command can overcome stiction near its edge. */
        error_m += copysignf(fminf(LASER_END_TRIM_INSET_M, (max_mm - min_mm) * 0.0005f), error_m);
    }
    s_odom_test.correction_mps = app_odom_test_slew_correction(error_m, s_end_y_correction,
                                                              s_odom_test.correction_mps, now);
    fix->correction_mps = s_odom_test.correction_mps;
    if(s_odom_test.correction_mps == 0.0f) return app_control_hold_zero_chassis();

    /* Keep the trim perpendicular to the route even while the yaw loop is correcting. */
    const float vx_cmd = s * s_odom_test.correction_mps;
    const float vy_cmd = c * s_odom_test.correction_mps;
    float wz = chassis_yaw_hold_apply(vx_cmd, vy_cmd, 0.0f, yaw, gyro_z, APP_ODOM_TEST_CONTROL_PERIOD_S);
    wz = fmaxf(-LASER_END_YAW_LIMIT_RAD_S, fminf(LASER_END_YAW_LIMIT_RAD_S, wz));
    return chassis.set_velocity(vx_cmd, vy_cmd, wz) == CHASSIS_OK
        ? APP_CONTROL_RESULT_OK : APP_CONTROL_RESULT_CHASSIS_ERROR;
}

AppControlResult app_odom_test_process(void) {
    Vector3 position = { 0 };
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };

    if(s_odom_test.segment_index >= APP_ODOM_TEST_ROUTE_COUNT) {
        s_odom_test.laser.mask = 0u;
        return app_control_hold_zero_chassis();
    }
    if(s_odom_test.motion_started && !laser_odom_is_online()) {
        s_odom_test.laser.mask = 0u;
        return s_odom_test.segment_started ? APP_CONTROL_RESULT_ODOM_ERROR : app_control_hold_zero_chassis();
    }

    const AppOdomTestSegment* segment = &s_route[s_odom_test.segment_index];
    const float distance_m = segment->distance_m;
    const float vx = segment->vx;
    const float vy = segment->vy;
    const bool test_vy = vy != 0.0f;
    const float speed_mps = test_vy ? vy : vx;
    const uint32_t now = delay_now_ms();

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
        s_odom_test.line_cross_m = 0.0f;
        s_odom_test.segment_moving = false;
        s_odom_test.laser_enabled = false;
        s_odom_test.end_trim_started = false;
        s_odom_test.end_trim_done = false;
        s_odom_test.following_laser = false;
        s_odom_test.last_tracked_frame = 0u;
        s_odom_test.last_control_ms = now;
        for(unsigned int beam = 0u; beam < LASER_BEAM_COUNT; ++beam) {
            s_odom_test.tracks[beam] = (AppLaserTrack){ 0 };
        }
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

    const float dx = position.x - s_odom_test.origin.x;
    const float dy = position.y - s_odom_test.origin.y;
    const float x_m = dx * s_odom_test.heading_cos + dy * s_odom_test.heading_sin;
    const float y_m = -dx * s_odom_test.heading_sin + dy * s_odom_test.heading_cos;
    const float cross_m = test_vy ? x_m : y_m;
    const LaserOdomSample* sample = laser_odom_get_sample();
    if(!s_odom_test.segment_stopping) {
        const float travel_m = test_vy ? y_m : x_m;
        s_odom_test.progress_m = speed_mps > 0.0f ? travel_m : -travel_m;
        if(!isfinite(s_odom_test.progress_m) || !isfinite(cross_m)) return APP_CONTROL_RESULT_ODOM_ERROR;
        if(!s_odom_test.segment_moving && s_odom_test.progress_m >= APP_ODOM_TEST_CORRECTION_START_M) {
            s_odom_test.frame_at_motion_start = sample->received;
            s_odom_test.motion_since_ms = now;
            s_odom_test.segment_moving = true;
            s_odom_test.motion_started = true;
        }
        if(s_odom_test.progress_m >= distance_m) {
            s_odom_test.segment_stopping = true;
            s_odom_test.stop_since_ms = now;
        }
    }

    if(s_odom_test.segment_moving && !s_odom_test.laser_enabled && sample->received != 0u &&
       sample->received != s_odom_test.frame_at_motion_start &&
       (!test_vy || s_odom_test.segment_stopping ||
        (int32_t)(sample->stamp_ms - s_odom_test.motion_since_ms) >= (int32_t)LASER_VY_DELAY_MS)) {
        s_odom_test.laser_enabled = true;
        log_info("LASER ENABLE seg=%u frame=%lu progress=%.3f",
                 (unsigned int)(s_odom_test.segment_index + 1u), (unsigned long)sample->received,
                 (double)s_odom_test.progress_m);
    }
    s_odom_test.laser = (AppLaserFix){ .segment = (unsigned int)s_odom_test.segment_index + 1u };
    const LaserOdomSample* filtered = s_odom_test.laser_enabled ? laser_field_filter(sample) : NULL;
    if(s_odom_test.segment_stopping) {
        const bool trim_side = !test_vy && (segment->side == LASER_LEFT || segment->side == LASER_RIGHT);
        if(trim_side && !s_odom_test.end_trim_done) {
            const AppControlResult trim = app_odom_test_trim_end(segment, sample, cross_m, angle.z, gyro.z, now);
            if(trim != APP_CONTROL_RESULT_SKIPPED) return trim;
            s_odom_test.stop_since_ms = now;
        }
        if(filtered) s_odom_test.laser = app_odom_test_laser_fix(segment, filtered, position);
        const bool side_filtered = trim_side && (!app_odom_test_end_trim_range(segment, sample) ||
            (filtered && !app_odom_test_end_trim_range(segment, filtered)));
        if(side_filtered) s_odom_test.laser.mask &= ~APP_LASER_Y;
        return app_odom_test_finish_segment(segment, position, sample, side_filtered);
    }
    float error_m = 0.0f;
    if(s_odom_test.laser_enabled && s_odom_test.progress_m >= APP_ODOM_TEST_CORRECTION_START_M) {
        const uint32_t age_ms = now - sample->stamp_ms;
        float sample_cross_m = cross_m;
        if(age_ms > 0u && age_ms <= LASER_TRACK_MAX_AGE_MS && sample->received != s_odom_test.last_tracked_frame) {
            Vector3 velocity = { 0 };
            if(odom.get_velocity(&velocity) != ODOM_OK || !isfinite(velocity.x) || !isfinite(velocity.y)) {
                return APP_CONTROL_RESULT_ODOM_ERROR;
            }
            const float cy = cosf(angle.z);
            const float sy = sinf(angle.z);
            const float c = cy * s_odom_test.heading_cos + sy * s_odom_test.heading_sin;
            const float s = sy * s_odom_test.heading_cos - cy * s_odom_test.heading_sin;
            const float normal_mps = test_vy ? c * velocity.x - s * velocity.y : s * velocity.x + c * velocity.y;
            sample_cross_m -= normal_mps * ((float)age_ms * 0.001f);
        }
        error_m = app_odom_test_motion_fix(&s_odom_test.laser, sample, test_vy, segment->side, cross_m, sample_cross_m, age_ms);
    }

    const bool following_laser = s_odom_test.laser.mask != 0u;
    AppLaserCorrection tuning = test_vy ? s_x_correction : s_y_correction;
    if(following_laser) {
        /* Carry the accepted lateral position into the odometry fallback. */
        s_odom_test.line_cross_m = cross_m;
    }
    else {
        const float line_error_m = s_odom_test.line_cross_m - cross_m;
        error_m = copysignf(fmaxf(0.0f, fabsf(line_error_m) - ODOM_LINE_DEADBAND_M), line_error_m);
        tuning = s_odom_correction;
    }
    const float previous = s_odom_test.following_laser == following_laser ? s_odom_test.correction_mps : 0.0f;
    s_odom_test.correction_mps = app_odom_test_slew_correction(error_m, tuning, previous, now);
    s_odom_test.following_laser = following_laser;
    s_odom_test.laser.correction_mps = s_odom_test.correction_mps;
    float vx_cmd = vx;
    float vy_cmd = vy;
    if(test_vy) {
        /* Route X already uses the software sign: positive points toward B. */
        const float cy = cosf(angle.z);
        const float sy = sinf(angle.z);
        const float c = cy * s_odom_test.heading_cos + sy * s_odom_test.heading_sin;
        const float s = sy * s_odom_test.heading_cos - cy * s_odom_test.heading_sin;
        /* Rotate the route command into body axes before applying yaw hold. */
        vx_cmd = c * s_odom_test.correction_mps + s * vy;
        vy_cmd = c * vy - s * s_odom_test.correction_mps;
    }
    else {
        vy_cmd += s_odom_test.correction_mps;
    }
    const float wz = chassis_yaw_hold_apply(vx_cmd, vy_cmd, 0.0f, angle.z, gyro.z,
                                             APP_ODOM_TEST_CONTROL_PERIOD_S);
    return chassis.set_velocity(vx_cmd, vy_cmd, wz) == CHASSIS_OK
        ? APP_CONTROL_RESULT_OK : APP_CONTROL_RESULT_CHASSIS_ERROR;
}
