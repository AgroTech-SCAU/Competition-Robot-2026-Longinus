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
#define APP_ODOM_TEST_STOP_SETTLE_MS 150u
#define APP_ODOM_TEST_STOP_TIMEOUT_MS 2000u
#define APP_ODOM_TEST_ODOM_DEADBAND_M 0.005f
#define APP_ODOM_TEST_YAW_SLOW_RAD 0.035f
#define APP_ODOM_TEST_YAW_STOP_RAD 0.087f
#define LASER_SIDE_CAPTURE_MAX_MM 250.0f
#define LASER_FRONT_REAR_CAPTURE_MAX_MM 80.0f
#define LASER_SIDE_RESERVE_MM 8.0f
#define LASER_LEFT_SETBACK_MM (6.0f + 20.0f)
#define LASER_START_LEFT_APPROACH_WINDOW_MM 20.0f
#define LASER_START_LEFT_LOOKAHEAD_S 0.35f
#define LASER_START_LEFT_ESCAPE_HALF_SPEED_ERROR_M 0.010f
#define LASER_START_LEFT_ESCAPE_LIMIT_MPS 0.014f
#define LASER_START_LEFT_ESCAPE_ACCEL_MPS2 0.085f
#define LASER_START_LEFT_RESPONSE_WINDOW_MS 100u
#define LASER_START_LEFT_DRIFT_FILTER_S 0.16f
#define LASER_START_LEFT_DRIFT_LIMIT_MPS 0.024f
#define LASER_START_LEFT_AWAY_LIMIT_MPS 0.034f
#define LASER_START_LEFT_RETURN_LIMIT_MPS 0.026f
#define LASER_START_LEFT_RETURN_BASE_MPS 0.018f
#define LASER_START_LEFT_RESPONSE_S 0.085f
#define LASER_START_LEFT_JERK_MPS3 0.55f
#define LASER_SIDE_LOOKAHEAD_S 0.25f
#define LASER_SIDE_LOOKAHEAD_MAX_MM 12.0f
#define LASER_SIDE_NEAR_CONFIRM_MS 40u
#define LASER_TRACK_JUMP_MM 12.0f
#define LASER_TRACK_STABLE_MM 5.0f
#define LASER_TRACK_CONFIRM_FRAMES 3u
#define LASER_TRACK_CONFIRM_MS 100u
#define LASER_TRACK_MAX_AGE_MS 120u
#define LASER_TRACK_HOLD_MS 250u
#define LASER_TRACK_RELOCK_MS 300u
#define LASER_BEAM_SWITCH_MARGIN_MM 5.0f
#define LASER_Y_SLOWDOWN_M 0.010f
#define LASER_Y_RESPONSE_TIME_S 0.07f
#define LASER_Y_JERK_LIMIT_MPS3 1.35f

typedef struct {
    float kp;
    float limit_mps;
    float accel_mps2;
} AppLaserCorrection;

/* Gain, speed limit and acceleration limit, named by the corrected axis. */
static const AppLaserCorrection s_x_correction = { 1.0f, 0.03f, 0.15f }; /* vy travel */
static const AppLaserCorrection s_y_correction = { 0.26f, 0.012f, 0.10f }; /* vx travel */
static const AppLaserCorrection s_y_escape_correction = { 0.90f, 0.024f, 0.15f }; /* away from L/R */
static const AppLaserCorrection s_odom_correction = { 1.0f, 0.015f, 0.10f };
static const AppLaserCorrection s_y_odom_correction = { 0.70f, 0.010f, 0.08f };

/* Distance from each sensor to the outermost edge on that side of the body.
 * Clearance = raw range - (mount setback from the base body + protrusion). */
static const float s_sensor_setback_mm[LASER_BEAM_COUNT] = {
    [LASER_FRONT] = 16.0f,
    [LASER_REAR] = 16.0f + 40.0f, /* Battery */
    [LASER_LEFT] = LASER_LEFT_SETBACK_MM, /* Air pump */
    [LASER_RIGHT] = 16.0f,
};

typedef struct {
    LaserBeam beam;
    float min_mm;
    float max_mm;
} AppLaserBand;

typedef struct {
    float accepted_wall_mm;
    float candidate_mm;
    uint32_t accepted_ms;
    uint32_t candidate_since_ms;
    unsigned int confirmations;
    bool anchored;
    bool usable;
} AppLaserTrack;

/* Distances and commands refer to the same interval between two CAN timestamps.
 * Positive velocity closes the left gap. Discard this history on loss of the face. */
typedef struct {
    bool near_wall;
    uint32_t frame_ms;
    uint32_t window_ms;
    float offset_mm;
    float commanded_m;
    float closing_mps;
    float drift_mps;
} AppStartLeftMotion;

typedef struct {
    float distance_m;
    float vx;
    float vy;
    AppLaserBand cruise[2];
} AppOdomTestSegment;

/* Distance in m, velocities in m/s, then each cruise beam and its body clearance in mm.
 * Use L/R for vx, and F/B for vy. LASER_NO_BEAM means there is no second reference.
 * Start-left raw target is 53..55 mm. Other raw-range centres: L=60.5, F=67, B/R=65 mm.
 * R=65 is the explicit boundary measurement; ridge-obscured R photos are not calibration points.
 * Bands below subtract the existing F/B/L/R setbacks of 16/56/26/16 mm. */
static const AppOdomTestSegment s_route[] = {
    {  4.75f,  -0.5f,  0.00f, { { LASER_LEFT, 53.0f - LASER_LEFT_SETBACK_MM,
                                              55.0f - LASER_LEFT_SETBACK_MM }, { .beam = LASER_NO_BEAM } } }, /* 01 */
    {  1.78f,   0.0f,  0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 02 */
    {  1.78f,   0.0f, -0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 03 */
    {  0.72f,   0.3f,  0.00f, { { LASER_LEFT , 33, 36 }, { .beam = LASER_NO_BEAM } } },  /* 04 */
    {   2.72f,   0.0f,  0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 05 */
    { 0.715f,   0.3f,  0.00f, { { LASER_RIGHT, 47, 51 }, { .beam = LASER_NO_BEAM } } },  /* 06 */
    {   2.64f,   0.0f, -0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 07 */
    {  0.72f,   0.3f,  0.00f, { { LASER_LEFT , 33, 36 }, { .beam = LASER_NO_BEAM } } },  /* 08 */
    {  2.76f,   0.0f,  0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 09 */
    {  0.72f,   0.3f,  0.00f, { { LASER_RIGHT, 47, 51 }, { .beam = LASER_NO_BEAM } } },  /* 10 */
    {  2.76f,   0.0f, -0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 11 */
    {  0.72f,   0.3f,  0.00f, { { LASER_LEFT , 33, 36 }, { .beam = LASER_NO_BEAM } } },  /* 12 */
    {  2.75f,   0.0f,  0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 13 */
    {  0.72f,   0.3f,  0.00f, { { LASER_RIGHT, 47, 51 }, { .beam = LASER_NO_BEAM } } },  /* 14 */
    {  2.75f,   0.0f, -0.30f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 15 */
    {   0.8f,   0.5f, 0.00f, { { LASER_FRONT, 49, 53 }, { LASER_REAR,  7, 11 } } },     /* 16 */
};

#define APP_ODOM_TEST_ROUTE_COUNT (sizeof(s_route) / sizeof(s_route[0]))

typedef struct {
    size_t segment_index;
    bool segment_started;
    bool segment_stopping;
    bool stationary;
    bool laser_waiting;
    bool following_laser;
    uint32_t stop_since_ms;
    uint32_t stationary_since_ms;
    uint32_t last_tracked_frame;
    uint32_t last_control_ms;
    Vector3 origin;
    float heading_cos;
    float heading_sin;
    float progress_m;
    float correction_mps;
    float correction_accel_mps2;
    float line_cross_m;
    LaserBeam preferred_beam;
    AppStartLeftMotion start_left_motion;
    AppLaserTrack tracks[LASER_BEAM_COUNT];
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

static bool app_odom_test_cruise_band_valid(const AppLaserBand* band, bool test_vy) {
    const LaserBeam beam = band->beam;
    const bool matches_axis = test_vy ? beam == LASER_FRONT || beam == LASER_REAR
                                      : beam == LASER_LEFT || beam == LASER_RIGHT;
    return matches_axis && isfinite(band->min_mm) && isfinite(band->max_mm) &&
           band->min_mm >= 0.0f && band->max_mm >= band->min_mm &&
           band->min_mm + s_sensor_setback_mm[beam] >= LASER_FIELD_MIN_RANGE_MM &&
           band->max_mm + s_sensor_setback_mm[beam] <= LASER_MAX_RANGE_MM;
}

static AppControlResult app_odom_test_finish_segment(uint32_t now) {
    const AppControlResult stop = app_control_hold_zero_chassis();
    if(stop != APP_CONTROL_RESULT_OK) return stop;

    Vector3 velocity = { 0 };
    if(odom.get_velocity(&velocity) != ODOM_OK || !isfinite(velocity.x) || !isfinite(velocity.y)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    if(hypotf(velocity.x, velocity.y) > APP_ODOM_TEST_STOP_SPEED_MPS) {
        s_odom_test.stationary = false;
        return now - s_odom_test.stop_since_ms >= APP_ODOM_TEST_STOP_TIMEOUT_MS
            ? APP_CONTROL_RESULT_ODOM_ERROR : stop;
    }
    if(!s_odom_test.stationary) {
        s_odom_test.stationary = true;
        s_odom_test.stationary_since_ms = now;
    }
    if(now - s_odom_test.stationary_since_ms < APP_ODOM_TEST_STOP_SETTLE_MS) return stop;

    ++s_odom_test.segment_index;
    s_odom_test.segment_started = false;
    s_odom_test.segment_stopping = false;
    s_odom_test.stationary = false;
    s_odom_test.correction_mps = 0.0f;
    s_odom_test.correction_accel_mps2 = 0.0f;
    return stop;
}

static void app_odom_test_track(AppLaserTrack* track, const LaserOdomSample* sample,
                                LaserBeam beam, float direction, float capture_max_mm,
                                float sample_cross_m) {
    const float raw_mm = (float)sample->distance_mm[beam];
    const float clearance_mm = raw_mm - s_sensor_setback_mm[beam];
    track->usable = false;
    if(raw_mm < LASER_FIELD_MIN_RANGE_MM || clearance_mm < 0.0f || raw_mm > LASER_MAX_RANGE_MM) {
        track->confirmations = 0u;
        return;
    }

    /* Compare the reflecting face, not range changes caused by our own correction. */
    const float wall_mm = sample_cross_m * 1000.0f + direction * clearance_mm;
    if(track->anchored && sample->stamp_ms - track->accepted_ms <= LASER_TRACK_HOLD_MS &&
       fabsf(wall_mm - track->accepted_wall_mm) <= LASER_TRACK_JUMP_MM) {
        track->accepted_wall_mm = wall_mm;
        track->accepted_ms = sample->stamp_ms;
        track->confirmations = 0u;
        track->usable = true;
        return;
    }
    if(clearance_mm > capture_max_mm) {
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
    const uint32_t confirm_ms = track->anchored ? LASER_TRACK_RELOCK_MS : LASER_TRACK_CONFIRM_MS;
    if(track->confirmations >= LASER_TRACK_CONFIRM_FRAMES &&
       sample->stamp_ms - track->candidate_since_ms >= confirm_ms) {
        track->accepted_wall_mm = wall_mm;
        track->accepted_ms = sample->stamp_ms;
        track->anchored = track->usable = true;
        track->confirmations = 0u;
    }
}

static float app_odom_test_motion_fix(AppLaserFix* fix, const LaserOdomSample* sample,
                                      const AppOdomTestSegment* segment, bool test_vy,
                                       float cross_m, float sample_cross_m, float reserve_mm, uint32_t now) {
    if(sample->received == 0u) return 0.0f;
    const uint32_t age_ms = now - sample->stamp_ms;
    const bool new_frame = sample->received != s_odom_test.last_tracked_frame;
    s_odom_test.last_tracked_frame = sample->received;
    const float capture_max_mm = test_vy ? LASER_FRONT_REAR_CAPTURE_MAX_MM : LASER_SIDE_CAPTURE_MAX_MM;
    for(size_t i = 0; i < sizeof(segment->cruise) / sizeof(segment->cruise[0]); ++i) {
        const AppLaserBand* band = &segment->cruise[i];
        const LaserBeam beam = band->beam;
        if(beam == LASER_NO_BEAM) continue;

        const float direction = beam == LASER_LEFT || beam == LASER_REAR ? 1.0f : -1.0f;
        AppLaserTrack* track = &s_odom_test.tracks[beam];
        if(age_ms > LASER_TRACK_MAX_AGE_MS) {
            track->usable = false;
            track->confirmations = 0u;
        }
        else if(new_frame) {
            app_odom_test_track(track, sample, beam, direction, capture_max_mm, sample_cross_m);
        }
        if(track->usable && now - track->accepted_ms <= LASER_TRACK_MAX_AGE_MS) {
            fix->track_mask |= 1u << beam;
        }
        else if(track->anchored) {
            fix->jump_mask |= 1u << beam;
        }
    }

    float low_mm = -INFINITY;
    float high_mm = INFINITY;
    float preferred_error_mm = 0.0f;
    float smallest_margin_mm = INFINITY;
    float previous_error_mm = 0.0f;
    float previous_margin_mm = INFINITY;
    LaserBeam preferred_beam = LASER_NO_BEAM;
    bool available = false;
    for(size_t i = 0; i < sizeof(segment->cruise) / sizeof(segment->cruise[0]); ++i) {
        const AppLaserBand* band = &segment->cruise[i];
        const LaserBeam beam = band->beam;
        if(beam == LASER_NO_BEAM) continue;
        const AppLaserTrack* track = &s_odom_test.tracks[beam];
        /* Fresh beams take priority; predict old faces only during a short all-beam dropout. */
        if(fix->track_mask != 0u ? !(fix->track_mask & (1u << beam)) :
           !track->anchored || now - track->accepted_ms > LASER_TRACK_HOLD_MS) continue;

        const float direction = beam == LASER_LEFT || beam == LASER_REAR ? 1.0f : -1.0f;
        const float clearance_mm = direction * (track->accepted_wall_mm - cross_m * 1000.0f);
        const float min_mm = band->min_mm + reserve_mm;
        const float max_mm = band->max_mm + reserve_mm;
        const float lower = direction * (clearance_mm - max_mm);
        const float upper = direction * (clearance_mm - min_mm);
        const float beam_low_mm = fminf(lower, upper);
        const float beam_high_mm = fmaxf(lower, upper);
        low_mm = fmaxf(low_mm, beam_low_mm);
        high_mm = fminf(high_mm, beam_high_mm);
        const float margin_mm = clearance_mm - min_mm;
        const float beam_error_mm = beam_low_mm <= 0.0f && beam_high_mm >= 0.0f
            ? 0.0f : 0.5f * (beam_low_mm + beam_high_mm);
        if(beam == s_odom_test.preferred_beam) {
            previous_margin_mm = margin_mm;
            previous_error_mm = beam_error_mm;
        }
        if(margin_mm < smallest_margin_mm) {
            smallest_margin_mm = margin_mm;
            preferred_error_mm = beam_error_mm;
            preferred_beam = beam;
        }
        available = true;
    }
    if(!available) return 0.0f;
    fix->mask = test_vy ? APP_LASER_X : APP_LASER_Y;
    /* Avoid switching sides on millimetre noise when both targets cannot be met. */
    if(previous_margin_mm <= smallest_margin_mm + LASER_BEAM_SWITCH_MARGIN_MM) {
        preferred_error_mm = previous_error_mm;
    }
    else {
        s_odom_test.preferred_beam = preferred_beam;
    }
    /* Disjoint target bands must not veto a trusted single-beam avoidance. */
    if(low_mm > high_mm) return preferred_error_mm * 0.001f;
    /* Zero displacement is feasible inside the band. Aim inside it when recovering. */
    if(low_mm <= 0.0f && high_mm >= 0.0f) return 0.0f;
    return 0.0005f * (low_mm + high_mm);
}

static float app_odom_test_side_lead_mm(float closing_mps, float lookahead_s) {
    return fminf(LASER_SIDE_LOOKAHEAD_MAX_MM, fmaxf(0.0f, closing_mps) * lookahead_s * 1000.0f);
}

static float app_odom_test_guard_side(const AppLaserBand* band, const LaserOdomSample* sample,
                                      float direction, float cross_m, float lead_mm,
                                      float min_clearance_mm, float error_m, uint32_t now) {
    const AppLaserTrack* track = &s_odom_test.tracks[band->beam];
    const float cross_mm = cross_m * 1000.0f;
    float clearance_mm = INFINITY;
    if(track->anchored && now - track->accepted_ms <= LASER_TRACK_HOLD_MS) {
        clearance_mm = direction * (track->accepted_wall_mm - cross_mm);
    }
    /* A stable near return can request escape before it is trusted as a new wall. */
    if(track->confirmations >= LASER_TRACK_CONFIRM_FRAMES &&
       sample->stamp_ms - track->candidate_since_ms >= LASER_SIDE_NEAR_CONFIRM_MS &&
       now - sample->stamp_ms <= LASER_TRACK_MAX_AGE_MS) {
        const float candidate_clearance_mm = direction * (track->candidate_mm - cross_mm);
        clearance_mm = fminf(clearance_mm, candidate_clearance_mm);
    }

    const float predicted_margin_mm = clearance_mm - lead_mm - min_clearance_mm;
    if(predicted_margin_mm >= 0.0f) return error_m;
    /* Only add an away command. Keep the stored line based on the accepted wall. */
    return direction * fminf(direction * error_m, predicted_margin_mm * 0.001f);
}

static float app_odom_test_start_left_speed(float error_m, float limit_mps) {
    const float magnitude_m = fabsf(error_m);
    const float speed = limit_mps * magnitude_m / (magnitude_m + LASER_START_LEFT_ESCAPE_HALF_SPEED_ERROR_M);
    return copysignf(speed, error_m);
}

static float app_odom_test_start_left_target(const AppLaserBand* band, const LaserOdomSample* sample,
                                            float cross_m, float normal_mps, float applied_mps,
                                            float escape_error_m, uint32_t now) {
    AppStartLeftMotion* motion = &s_odom_test.start_left_motion;
    const uint16_t raw_mm = sample->distance_mm[LASER_LEFT];
    const float raw_gap = (float)raw_mm - LASER_LEFT_SETBACK_MM;
    if(!(s_odom_test.laser.track_mask & (1u << LASER_LEFT))) {
        const AppLaserTrack* track = &s_odom_test.tracks[LASER_LEFT];
        const uint32_t age = now - track->accepted_ms;
        /* Fade only confirmed away compensation over the existing wall hold. */
        const float away_drift = track->anchored && age < LASER_TRACK_HOLD_MS
            ? fmaxf(0.0f, motion->drift_mps) : 0.0f;
        *motion = (AppStartLeftMotion){ .drift_mps = away_drift };
        const float held = away_drift * (1.0f - fminf(1.0f, (float)age / LASER_TRACK_HOLD_MS));
        const float away = app_odom_test_start_left_speed(fminf(0.0f, escape_error_m), LASER_START_LEFT_ESCAPE_LIMIT_MPS);
        return fmaxf(-LASER_START_LEFT_AWAY_LIMIT_MPS, away - held);
    }

    const float age_s = (float)(now - sample->stamp_ms) * 0.001f;
    const float after_sample_m = applied_mps * age_s;
    /* Changes entirely inside the OLED target band carry no resolved displacement. */
    const float offset_mm = raw_gap - fmaxf(band->min_mm, fminf(band->max_mm, raw_gap));
    if(!motion->near_wall || sample->stamp_ms - motion->frame_ms > LASER_TRACK_MAX_AGE_MS) {
        *motion = (AppStartLeftMotion){ 0 };
        if(raw_gap > band->max_mm + LASER_START_LEFT_APPROACH_WINDOW_MM) return 0.0f;
        motion->near_wall = true;
        motion->window_ms = sample->stamp_ms;
        motion->offset_mm = offset_mm;
        motion->commanded_m = after_sample_m;
        motion->closing_mps = normal_mps;
    }
    else {
        motion->commanded_m += applied_mps * (float)(now - s_odom_test.last_control_ms) * 0.001f;
        const uint32_t window_ms = sample->stamp_ms - motion->window_ms;
        if(window_ms >= LASER_START_LEFT_RESPONSE_WINDOW_MS) {
            const float dt = (float)window_ms * 0.001f;
            const float measured_m = (motion->offset_mm - offset_mm) * 0.001f;
            const float residual_m = measured_m - (motion->commanded_m - after_sample_m);
            /* Reject a step in the reflecting face before learning it as steady slip. */
            if(fabsf(residual_m) > 2.0f * LASER_START_LEFT_DRIFT_LIMIT_MPS * dt + 0.002f) {
                *motion = (AppStartLeftMotion){ 0 };
                return app_odom_test_start_left_speed(fminf(0.0f, escape_error_m), LASER_START_LEFT_ESCAPE_LIMIT_MPS);
            }
            const float alpha = dt / (LASER_START_LEFT_DRIFT_FILTER_S + dt);
            const float drift = fmaxf(-LASER_START_LEFT_DRIFT_LIMIT_MPS,
                                      fminf(LASER_START_LEFT_DRIFT_LIMIT_MPS, residual_m / dt));
            motion->drift_mps += alpha * (drift - motion->drift_mps);
            motion->closing_mps += alpha * (measured_m / dt - motion->closing_mps);
            motion->commanded_m = after_sample_m;
            motion->window_ms = sample->stamp_ms;
            motion->offset_mm = offset_mm;
        }
    }
    motion->frame_ms = sample->stamp_ms;

    /* The encoder projection accounts for travel since CAN reception. D3 supplies
     * the wall-relative rate, including slip and motion along a sloping face. */
    const float gap = s_odom_test.tracks[LASER_LEFT].accepted_wall_mm - cross_m * 1000.0f;
    const float closing = normal_mps > 0.0f ? fmaxf(normal_mps + fminf(0.0f, motion->drift_mps),
                                                    motion->closing_mps) : motion->closing_mps;
    const float lead_mm = copysignf(app_odom_test_side_lead_mm(fabsf(closing), LASER_START_LEFT_LOOKAHEAD_S), closing);
    const float predicted = gap - lead_mm;
    const float error_mm = predicted < band->min_mm || predicted > band->max_mm
        ? predicted - 0.5f * (band->min_mm + band->max_mm) : 0.0f;
    const float base_limit = error_mm < 0.0f ? LASER_START_LEFT_ESCAPE_LIMIT_MPS : LASER_START_LEFT_RETURN_BASE_MPS;
    const float error_m = error_mm * 0.001f;
    const float drift_mps = raw_gap < band->min_mm ? fmaxf(0.0f, motion->drift_mps) : motion->drift_mps;
    const float target = app_odom_test_start_left_speed(error_m, base_limit) - drift_mps;
    /* An old outward-drift estimate must not keep steering left on re-entry. */
    const float room_m = (fminf(raw_gap, predicted) - band->min_mm) * 0.001f;
    float toward_limit = 0.0f;
    if(room_m > 0.0f && raw_gap > band->max_mm && escape_error_m >= 0.0f) {
        toward_limit = fmaxf(0.0f, fminf(LASER_START_LEFT_RETURN_LIMIT_MPS, room_m - motion->drift_mps));
    }
    return fmaxf(-LASER_START_LEFT_AWAY_LIMIT_MPS, fminf(toward_limit, target));
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

static float app_odom_test_band_speed(float error_m, AppLaserCorrection tuning) {
    const float magnitude_m = fabsf(error_m);
    const float u = fminf(1.0f, magnitude_m / LASER_Y_SLOWDOWN_M);
    const float blend = u * u * (3.0f - 2.0f * u);
    return copysignf(fminf(tuning.limit_mps, tuning.kp * magnitude_m) * blend, error_m);
}

static float app_odom_test_curve_y(float target, float accel_cap, bool start_left, uint32_t now) {
    const float dt = fminf(0.020f, (float)(now - s_odom_test.last_control_ms) * 0.001f);
    s_odom_test.last_control_ms = now;
    const float previous_accel = s_odom_test.correction_accel_mps2;
    const float tau = start_left ? LASER_START_LEFT_RESPONSE_S : LASER_Y_RESPONSE_TIME_S;
    const float jerk_limit = start_left ? LASER_START_LEFT_JERK_MPS3 : LASER_Y_JERK_LIMIT_MPS3;
    /* Critically damped speed tracking with bounded jerk gives an S-shaped rise and fall.
     * Keep both states through zero error, reversal and laser/odometry handover. */
    const float jerk_request = (target - s_odom_test.correction_mps) / (tau * tau)
        - 2.0f * previous_accel / tau;
    const float jerk = fmaxf(-jerk_limit, fminf(jerk_limit, jerk_request));
    /* A tighter tuning limit must also be approached without an acceleration jump. */
    const float accel_limit = fmaxf(accel_cap, fabsf(previous_accel) - jerk_limit * dt);
    const float accel = fmaxf(-accel_limit, fminf(accel_limit, previous_accel + jerk * dt));
    s_odom_test.correction_accel_mps2 = accel;
    return s_odom_test.correction_mps + 0.5f * (previous_accel + accel) * dt;
}

AppControlResult app_odom_test_process(void) {
    Vector3 position = { 0 };
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };

    if(s_odom_test.segment_index >= APP_ODOM_TEST_ROUTE_COUNT) {
        s_odom_test.laser.mask = 0u;
        return app_control_hold_zero_chassis();
    }
    const AppOdomTestSegment* segment = &s_route[s_odom_test.segment_index];
    const float distance_m = segment->distance_m;
    const float vx = segment->vx;
    const float vy = segment->vy;
    const bool test_vy = vy != 0.0f;
    const bool start_left = s_odom_test.segment_index == 0u && segment->cruise[0].beam == LASER_LEFT;
    const float reserve_mm = test_vy || start_left ? 0.0f : LASER_SIDE_RESERVE_MM;
    const float speed_mps = test_vy ? vy : vx;
    uint32_t now = delay_now_ms();
    const LaserOdomSample* sample = laser_odom_get_sample();

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
        const AppLaserBand* primary = &segment->cruise[0];
        const AppLaserBand* secondary = &segment->cruise[1];
        if(!app_odom_test_cruise_band_valid(primary, test_vy) ||
           (secondary->beam != LASER_NO_BEAM &&
            (!test_vy || secondary->beam == primary->beam || !app_odom_test_cruise_band_valid(secondary, test_vy)))) {
            return APP_CONTROL_RESULT_COMMAND_INVALID;
        }

        if(s_odom_test.segment_index == 0u) {
            s_odom_test.heading_cos = cosf(angle.z);
            s_odom_test.heading_sin = sinf(angle.z);
            chassis_yaw_hold_set_target(angle.z);
        }
        s_odom_test.origin = position;
        s_odom_test.line_cross_m = 0.0f;
        s_odom_test.preferred_beam = LASER_NO_BEAM;
        s_odom_test.laser_waiting = false;
        s_odom_test.following_laser = false;
        s_odom_test.last_tracked_frame = sample->received;
        s_odom_test.last_control_ms = now;
        for(unsigned int beam = 0u; beam < LASER_BEAM_COUNT; ++beam) {
            s_odom_test.tracks[beam] = (AppLaserTrack){ 0 };
        }
        s_odom_test.segment_started = true;
        log_info("ODOM ROUTE %u/%u start: d=%.2fm vx=%.2fm/s vy=%.2fm/s",
                 (unsigned int)(s_odom_test.segment_index + 1u), (unsigned int)APP_ODOM_TEST_ROUTE_COUNT,
                 (double)distance_m, (double)vx, (double)vy);
        static const char beam_labels[] = "FBLR";
        for(size_t i = 0; i < sizeof(segment->cruise) / sizeof(segment->cruise[0]); ++i) {
            const AppLaserBand* band = &segment->cruise[i];
            if(band->beam == LASER_NO_BEAM) continue;
            log_info("LASER CRUISE seg=%u %c net=%.1f..%.1f mm",
                     (unsigned int)(s_odom_test.segment_index + 1u), beam_labels[band->beam],
                     (double)(band->min_mm + reserve_mm), (double)(band->max_mm + reserve_mm));
        }
        now = delay_now_ms();
    }

    const float applied_mps = s_odom_test.laser.correction_mps;
    s_odom_test.laser = (AppLaserFix){ .x = NAN, .y = NAN, .segment = (unsigned int)s_odom_test.segment_index + 1u };
    if(s_odom_test.segment_stopping) return app_odom_test_finish_segment(now);

    const float dx = position.x - s_odom_test.origin.x;
    const float dy = position.y - s_odom_test.origin.y;
    const float x_m = dx * s_odom_test.heading_cos + dy * s_odom_test.heading_sin;
    const float y_m = -dx * s_odom_test.heading_sin + dy * s_odom_test.heading_cos;
    const float travel_m = test_vy ? y_m : x_m;
    const float cross_m = test_vy ? x_m : y_m;
    s_odom_test.progress_m = speed_mps > 0.0f ? travel_m : -travel_m;
    if(!isfinite(s_odom_test.progress_m) || !isfinite(cross_m)) return APP_CONTROL_RESULT_ODOM_ERROR;
    if(s_odom_test.progress_m >= distance_m) {
        s_odom_test.segment_stopping = true;
        s_odom_test.stop_since_ms = now;
        return app_odom_test_finish_segment(now);
    }

    const float cy = cosf(angle.z);
    const float sy = sinf(angle.z);
    const float c = cy * s_odom_test.heading_cos + sy * s_odom_test.heading_sin;
    const float s = sy * s_odom_test.heading_cos - cy * s_odom_test.heading_sin;
    const uint32_t age_ms = now - sample->stamp_ms;
    float sample_cross_m = cross_m;
    float normal_mps = 0.0f;
    const bool compensate_sample = age_ms > 0u && age_ms <= LASER_TRACK_MAX_AGE_MS &&
        sample->received != s_odom_test.last_tracked_frame;
    if(!test_vy || compensate_sample) {
        Vector3 velocity = { 0 };
        if(odom.get_velocity(&velocity) == ODOM_OK && isfinite(velocity.x) && isfinite(velocity.y)) {
            normal_mps = test_vy ? c * velocity.x - s * velocity.y : s * velocity.x + c * velocity.y;
        }
        else if(compensate_sample) {
            return APP_CONTROL_RESULT_ODOM_ERROR;
        }
    }
    if(compensate_sample) sample_cross_m -= normal_mps * ((float)age_ms * 0.001f);
    float error_m = app_odom_test_motion_fix(&s_odom_test.laser, sample, segment, test_vy,
                                             cross_m, sample_cross_m, reserve_mm, now);
    const bool following_laser = s_odom_test.laser.mask != 0u;
    AppLaserCorrection tuning = test_vy ? s_odom_correction : s_y_odom_correction;
    if(following_laser) {
        tuning = test_vy ? s_x_correction : s_y_correction;
        /* Preserve the desired line, including any correction not yet completed. */
        s_odom_test.line_cross_m = cross_m + error_m;
    }
    else {
        const float line_error_m = s_odom_test.line_cross_m - cross_m;
        error_m = copysignf(fmaxf(0.0f, fabsf(line_error_m) - APP_ODOM_TEST_ODOM_DEADBAND_M), line_error_m);
        s_odom_test.laser.mask = 0u;
    }
    if(test_vy) {
        const float previous = following_laser == s_odom_test.following_laser ? s_odom_test.correction_mps : 0.0f;
        s_odom_test.correction_mps = app_odom_test_slew_correction(error_m, tuning, previous, now);
    }
    else {
        const AppLaserBand* side = &segment->cruise[0];
        const float direction = side->beam == LASER_LEFT ? 1.0f : -1.0f;
        const float lookahead_s = start_left ? LASER_START_LEFT_LOOKAHEAD_S : LASER_SIDE_LOOKAHEAD_S;
        const float lead_mm = app_odom_test_side_lead_mm(direction * normal_mps, lookahead_s);
        error_m = app_odom_test_guard_side(side, sample, direction, cross_m, lead_mm,
                                          side->min_mm + reserve_mm, error_m, now);
        if(error_m * direction < 0.0f) tuning = s_y_escape_correction;
        const float target = start_left ? app_odom_test_start_left_target(side, sample, cross_m, normal_mps,
                                                                          applied_mps, error_m, now)
                                        : app_odom_test_band_speed(error_m, tuning);
        const float accel_cap = start_left ? LASER_START_LEFT_ESCAPE_ACCEL_MPS2 : tuning.accel_mps2;
        s_odom_test.correction_mps = app_odom_test_curve_y(target, accel_cap, start_left, now);
        if(start_left && target <= 0.0f && s_odom_test.correction_mps >= 0.0f) {
            /* Never carry an old leftward command into the target band or through a dropout. */
            s_odom_test.correction_mps = 0.0f;
            s_odom_test.correction_accel_mps2 = 0.0f;
        }
    }
    s_odom_test.following_laser = following_laser;

    /* Filtered/obscured beams only select laser or odometry correction, never along-track speed.
     * Keep the existing CAN communication-loss stop separate from measurement rejection. */
    const bool wait_laser = !laser_odom_is_online();
    float motion_scale = wait_laser ? 0.0f : 1.0f;
    if(wait_laser != s_odom_test.laser_waiting) {
        log_info("LASER %s seg=%u gap_ms=%lu", wait_laser ? "WAIT" : "RESUME",
                 s_odom_test.laser.segment, (unsigned long)age_ms);
        s_odom_test.laser_waiting = wait_laser;
    }

    /* The route owns along-track speed. The correction only changes the normal component. */
    const float vx_route = test_vy ? s_odom_test.correction_mps : vx;
    const float vy_route = test_vy ? vy : s_odom_test.correction_mps;
    const float vx_cmd = c * vx_route + s * vy_route;
    const float vy_cmd = c * vy_route - s * vx_route;
    const float wz = chassis_yaw_hold_apply(vx_cmd, vy_cmd, 0.0f, angle.z, gyro.z,
                                             APP_ODOM_TEST_CONTROL_PERIOD_S);
    /* Keep heading control active while translation slows or waits for a reference. */
    const float yaw_error = fabsf(chassis_yaw_hold_get_yaw_error());
    const float heading_scale = fminf(1.0f, fmaxf(0.0f,
        (APP_ODOM_TEST_YAW_STOP_RAD - yaw_error) / (APP_ODOM_TEST_YAW_STOP_RAD - APP_ODOM_TEST_YAW_SLOW_RAD)));
    motion_scale = fminf(motion_scale, heading_scale);
    if(motion_scale == 0.0f) {
        s_odom_test.correction_mps = 0.0f;
        s_odom_test.correction_accel_mps2 = 0.0f;
        if(start_left) s_odom_test.start_left_motion = (AppStartLeftMotion){ 0 };
    }
    s_odom_test.laser.correction_mps = motion_scale * s_odom_test.correction_mps;
    if(chassis.set_velocity(motion_scale * vx_cmd, motion_scale * vy_cmd, wz) != CHASSIS_OK) {
        return APP_CONTROL_RESULT_CHASSIS_ERROR;
    }
    return APP_CONTROL_RESULT_OK;
}
