#include "app_odom_test.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "imu.h"
#include "laser_odom.h"
#include "log.h"
#include "map_route.h"
#include "odom.h"

#include <math.h>
#include <stddef.h>

#define APP_ODOM_TEST_CONTROL_PERIOD_S 0.002f
#define APP_ODOM_TEST_STOP_SPEED_MPS 0.02f
#define APP_ODOM_TEST_SETTLE_READY_MS 100u
#define ROUTE_ACCEL_DISTANCE_M 0.400f
#define ROUTE_DECEL_DISTANCE_M 0.400f
#define ROUTE_PROFILE_START_SPEED_MPS 0.02f
#define ROUTE_END_TOLERANCE_M 0.005f
#define ROUTE_WALL_END_WINDOW_M 0.08f
#define ROUTE_WALL_RECOVERY_WINDOW_M 0.25f
#define ROUTE_WALL_TRIM_MAX_M 0.15f
#define ROUTE_WALL_TRIM_MAX_MS 5500u
#define ROUTE_WALL_TRIM_SPEED_MPS 0.04f
#define ROUTE_WALL_TRIM_CRAWL_MPS 0.008f
#define LASER_WALL_TRIM_CAPTURE_MM 220u
#define LASER_WALL_TRIM_TOLERANCE_MM 1.0f
#define LASER_WALL_TRIM_CONFIRM_FRAMES 3u
#define LASER_WALL_ACCEPT_MARGIN_MM 3.0f
#define LASER_WALL_BRAKE_START_MM 400.0f
#define LASER_WALL_BRAKE_MPS2 0.35f
#define LASER_FRONT_WALL_RESERVE_MM 70.0f
#define LASER_LEFT_WALL_RESERVE_MM 48.0f
#define LASER_RIGHT_WALL_RESERVE_MM 52.0f
#define LASER_CONNECT_HOLD_MM 35.0f
#define LASER_CONNECT_RESUME_MM 38.0f
/* The launch measured 42 mm on the left with less clearance on the right. */
#define LASER_LAUNCH_LEFT_TARGET_MM 36.0f
#define LASER_LEFT_TARGET_MM 41.0f
#define LASER_RIGHT_TARGET_MM 45.0f
#define LASER_DUAL_TOLERANCE_MM 1.0f
#define LASER_START_ACQUIRE_MM 105.0f
#define LASER_SIDE_GUIDANCE_MAX_MM 120.0f
#define LASER_CONTROL_DEADBAND_MM 1.0f
#define LASER_FURROW_SUM_MIN_MM 95.0f
#define LASER_FURROW_SUM_MAX_MM 125.0f
#define LASER_FURROW_HYSTERESIS_MM 5.0f
#define LASER_FURROW_NARROW_SUM_MM 100.0f
#define LASER_FURROW_NARROW_BIAS_MM 18.0f
#define LASER_FURROW_CENTER_SUM_MM 109.0f
#define LASER_FURROW_CENTER_BIAS_MM 3.0f
#define LASER_SIDE_CONTROL_GAIN 2.5f
#define LASER_SIDE_CORRECTION_LIMIT_MPS 0.09f
#define LASER_DUAL_CONTROL_GAIN 1.0f
#define LASER_DUAL_CORRECTION_LIMIT_MPS 0.06f
#define LASER_CORRECTION_ACCEL_MPS2 0.30f
#define LASER_FRAME_MAX_AGE_MS 120u
#define LASER_FRONT_SAFETY_MM 55.0f
#define LASER_SAFETY_MM 30.0f
#define LASER_ESCAPE_GAIN 2.5f
#define LASER_ESCAPE_LIMIT_MPS 0.09f
#define LASER_SIDE_ESCAPE_MIN_MPS 0.015f
#define ROUTE_COMMAND_LIMIT_MPS 0.65f
#define ROUTE_ALIGN_LOG_PERIOD_MS 250u
#define ROUTE_ENTRY_PUMP_PRIME_MS 1000u
#define ROUTE_ENTRY_PA0_MS 2000u
#define ROUTE_ENTRY_PUMP_MS (ROUTE_ENTRY_PUMP_PRIME_MS + ROUTE_ENTRY_PA0_MS)

/* UART9 work laser. Leave both ends of a crossing clear of work stops. */
#define ROUTE_WORK_TRIGGER_MM 350u
#define ROUTE_WORK_REARM_HYSTERESIS_MM 30u
#define ROUTE_WORK_REARM_MS 150u
#define ROUTE_WORK_END_MARGIN_M 0.40f
#define ROUTE_WORK_RANGE_MAX_AGE_MS 200u
#define ROUTE_WORK_PUMP_MS 1000u

typedef enum {
    ROUTE_START_LEG = 0,
    ROUTE_TRAVEL,
    ROUTE_WORK_STOPPING,
    ROUTE_WORK_PUMPING,
    ROUTE_SETTLE,
    ROUTE_PARKED,
    ROUTE_WALL_TRIM
} RoutePhase;

typedef enum {
    ENTRY_PUMP_PENDING = 0,
    ENTRY_PUMP_ACTIVE,
    ENTRY_PUMP_DONE
} RouteEntryPumpState;

typedef enum {
    ROUTE_SPEED_STOP = 0,
    ROUTE_SPEED_ACCEL,
    ROUTE_SPEED_CRUISE,
    ROUTE_SPEED_DECEL
} RouteSpeedPhase;

typedef struct {
    uint16_t ranges[LASER_BEAM_COUNT][3];
    uint32_t last_frame;
    uint32_t last_stamp_ms;
    unsigned int frame_count;
} RouteLaserHistory;

typedef struct {
    size_t segment_index;
    RoutePhase phase;
    RouteSpeedPhase speed_phase;
    RouteEntryPumpState entry_pump_state;
    float remaining_m;
    float route_speed_mps;
    float correction_mps;
    float wall_trim_start_m;
    float accel_mps2;
    float decel_mps2;
    float accel_distance_m;
    float decel_distance_m;
    Vector3 origin;
    float heading_cos;
    float heading_sin;
    float imu_heading_ref;
    uint32_t travel_ms;
    uint32_t settle_ready_since_ms;
    uint32_t entry_pump_since_ms;
    uint32_t wall_trim_started_ms;
    uint32_t wall_trim_confirm_frame;
    uint32_t last_control_ms;
    uint32_t work_pump_since_ms;
    uint32_t work_stationary_since_ms;
    uint32_t work_clear_since_ms;
    bool laser_waiting;
    bool guidance_acquired;
    bool furrow_gate_open;
    bool connector_hold;
    bool wall_trim_started;
    bool wall_trim_motion_exhausted;
    bool wall_trim_recovery_pending;
    bool wall_trim_recovery_used;
    unsigned int wall_trim_confirm_count;
    bool yaw_target_valid;
    bool work_target_latched;
    bool work_stationary_seen;
    bool work_clear_seen;
    RouteLaserHistory laser_history;
    unsigned int wall_anchor_mask;
    AppLaserFix laser;
} AppOdomTestState;

typedef struct {
    float vx;
    float vy;
    unsigned int mask;
} RouteEscape;

static AppOdomTestState s_odom_test;
static uint16_t s_work_distance_mm;
static uint32_t s_work_range_ms;
static bool s_work_frame_received;

static bool route_finite_vec3(Vector3 value) {
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static float route_clampf(float value, float limit) {
    return fmaxf(-limit, fminf(limit, value));
}

void app_odom_test_reset(void) {
    s_odom_test = (AppOdomTestState){ 0 };
    chassis_yaw_hold_disable();
}

void app_odom_test_set_work_range(bool frame_received, uint16_t distance_mm, uint32_t stamp_ms) {
    s_work_frame_received = frame_received;
    s_work_distance_mm = distance_mm;
    s_work_range_ms = stamp_ms;
}

bool app_odom_test_work_pump_enabled(void) {
    return s_odom_test.phase == ROUTE_WORK_PUMPING;
}

bool app_odom_test_entry_pump_enabled(void) {
    if(s_odom_test.entry_pump_state == ENTRY_PUMP_ACTIVE &&
       delay_now_ms() - s_odom_test.entry_pump_since_ms >= ROUTE_ENTRY_PUMP_MS) {
        s_odom_test.entry_pump_state = ENTRY_PUMP_DONE;
    }
    return s_odom_test.entry_pump_state == ENTRY_PUMP_ACTIVE;
}

bool app_odom_test_entry_pa0_enabled(void) {
    return app_odom_test_entry_pump_enabled() &&
           delay_now_ms() - s_odom_test.entry_pump_since_ms >= ROUTE_ENTRY_PUMP_PRIME_MS;
}

static bool route_work_active(void) {
    return s_odom_test.phase == ROUTE_WORK_STOPPING || app_odom_test_work_pump_enabled();
}

static bool route_work_target_detected(bool work_zone, uint32_t now) {
    const bool fresh = s_work_frame_received && now - s_work_range_ms <= ROUTE_WORK_RANGE_MAX_AGE_MS;
    if(!work_zone) {
        s_odom_test.work_target_latched = false;
        s_odom_test.work_clear_seen = false;
        return false;
    }
    if(s_odom_test.work_target_latched) {
        const bool clear = fresh && (s_work_distance_mm == 0u ||
                           s_work_distance_mm >= ROUTE_WORK_TRIGGER_MM + ROUTE_WORK_REARM_HYSTERESIS_MM);
        if(!clear) s_odom_test.work_clear_seen = false;
        else if(!s_odom_test.work_clear_seen) {
            s_odom_test.work_clear_seen = true;
            s_odom_test.work_clear_since_ms = s_work_range_ms;
        }
        else if(s_work_range_ms - s_odom_test.work_clear_since_ms >= ROUTE_WORK_REARM_MS) {
            s_odom_test.work_target_latched = false;
            s_odom_test.work_clear_seen = false;
        }
        return false;
    }
    return fresh && s_work_distance_mm != 0u && s_work_distance_mm <= ROUTE_WORK_TRIGGER_MM;
}

static AppControlResult route_work_step(uint32_t now, Vector3 velocity) {
    const AppControlResult stop = app_control_hold_zero_chassis();
    if(stop != APP_CONTROL_RESULT_OK) return stop;

    if(s_odom_test.phase == ROUTE_WORK_STOPPING) {
        const bool stationary = hypotf(velocity.x, velocity.y) <= APP_ODOM_TEST_STOP_SPEED_MPS &&
                                fabsf(velocity.z) <= 0.08f;
        if(!stationary) {
            s_odom_test.work_stationary_seen = false;
        }
        else if(!s_odom_test.work_stationary_seen) {
            s_odom_test.work_stationary_seen = true;
            s_odom_test.work_stationary_since_ms = now;
        }
        else if(now - s_odom_test.work_stationary_since_ms >= APP_ODOM_TEST_SETTLE_READY_MS) {
            s_odom_test.work_pump_since_ms = now;
            s_odom_test.phase = ROUTE_WORK_PUMPING;
            log_info("MAP WORK pump on");
        }
    }
    else if(now - s_odom_test.work_pump_since_ms >= ROUTE_WORK_PUMP_MS) {
        s_odom_test.travel_ms = now;
        s_odom_test.phase = ROUTE_TRAVEL;
        log_info("MAP WORK pump off, resume");
    }
    return stop;
}

static void route_wall_trim_reset_confirmation(uint32_t frame) {
    s_odom_test.wall_trim_confirm_frame = frame;
    s_odom_test.wall_trim_confirm_count = 0u;
    s_odom_test.settle_ready_since_ms = 0u;
}

static AppControlResult route_odom_error(void) {
    if(app_odom_test_entry_pump_enabled()) {
        s_odom_test.entry_pump_state = ENTRY_PUMP_PENDING;
        s_odom_test.settle_ready_since_ms = 0u;
    }
    const bool wall_trim = s_odom_test.phase == ROUTE_WALL_TRIM;
    if(route_work_active() || wall_trim) {
        if(wall_trim) {
            s_odom_test.route_speed_mps = 0.0f;
            route_wall_trim_reset_confirmation(laser_odom_get_sample()->received);
        }
        else {
            s_odom_test.phase = ROUTE_TRAVEL;
        }
        s_odom_test.travel_ms = delay_now_ms();
        const AppControlResult stop = app_control_hold_zero_chassis();
        if(stop != APP_CONTROL_RESULT_OK) return stop;
    }
    return APP_CONTROL_RESULT_ODOM_ERROR;
}

AppLaserFix app_odom_test_get_laser_fix(void) {
    return s_odom_test.laser;
}

float app_odom_test_remaining_m(void) {
    return s_odom_test.remaining_m;
}

bool app_odom_test_is_done(void) {
    return s_odom_test.phase == ROUTE_PARKED;
}

static bool route_frame_fresh(const LaserOdomSample* sample, uint32_t now) {
    return sample->received != 0u && now - sample->stamp_ms <= LASER_FRAME_MAX_AGE_MS;
}

static bool route_laser_valid(const LaserOdomSample* sample, LaserBeam beam, uint32_t now) {
    if((unsigned int)beam >= LASER_BEAM_COUNT || !route_frame_fresh(sample, now)) {
        return false;
    }
    const uint16_t raw = sample->distance_mm[beam];
    return raw >= LASER_MIN_RANGE_MM && raw <= LASER_MAX_RANGE_MM;
}

static void route_update_guidance_acquisition(const LaserOdomSample* sample, uint32_t now) {
    if(s_odom_test.segment_index != 0u || s_odom_test.guidance_acquired ||
       !route_laser_valid(sample, LASER_LEFT, now) ||
       sample->distance_mm[LASER_LEFT] > LASER_START_ACQUIRE_MM) return;
    s_odom_test.guidance_acquired = true;
    log_info("MAP ACQUIRE seg=1 LEFT raw=%u: lateral guidance latched",
             (unsigned int)sample->distance_mm[LASER_LEFT]);
}

static uint16_t route_median_range(const uint16_t ranges[3]) {
    const uint16_t low = ranges[0] < ranges[1] ? ranges[0] : ranges[1];
    const uint16_t high = ranges[0] > ranges[1] ? ranges[0] : ranges[1];
    if(ranges[2] < low) return low;
    return ranges[2] > high ? high : ranges[2];
}

/* Use distinct CAN frames for guidance and braking. Raw safety bypasses this filter. */
static LaserOdomSample route_filter_laser(const LaserOdomSample* sample, uint32_t now) {
    RouteLaserHistory* history = &s_odom_test.laser_history;
    LaserOdomSample filtered = { 0 };
    if(!route_frame_fresh(sample, now)) {
        *history = (RouteLaserHistory){ 0 };
        return filtered;
    }

    if(sample->received != history->last_frame) {
        if(history->frame_count != 0u &&
           sample->stamp_ms - history->last_stamp_ms > LASER_FRAME_MAX_AGE_MS) {
            *history = (RouteLaserHistory){ 0 };
        }
        history->last_frame = sample->received;
        history->last_stamp_ms = sample->stamp_ms;
        for(unsigned int beam = 0; beam < LASER_BEAM_COUNT; ++beam) {
            uint16_t* ranges = history->ranges[beam];
            ranges[0] = ranges[1];
            ranges[1] = ranges[2];
            ranges[2] = sample->distance_mm[beam];
        }
        if(history->frame_count < 3u) ++history->frame_count;
    }
    if(history->frame_count < 3u) return filtered;

    filtered.received = sample->received;
    filtered.stamp_ms = sample->stamp_ms;
    for(unsigned int beam = 0; beam < LASER_BEAM_COUNT; ++beam) {
        const uint16_t* ranges = history->ranges[beam];
        const uint16_t median = route_median_range(ranges);
        filtered.distance_mm[beam] = median != 0u && ranges[0] <= LASER_MAX_RANGE_MM &&
            ranges[1] <= LASER_MAX_RANGE_MM && ranges[2] <= LASER_MAX_RANGE_MM ? median : UINT16_MAX;
    }
    return filtered;
}

static float route_target_error_mm(uint16_t raw, float target_mm) {
    float error_mm = (float)raw - target_mm;
    if(fabsf(error_mm) <= LASER_CONTROL_DEADBAND_MM) return 0.0f;
    return error_mm - copysignf(LASER_CONTROL_DEADBAND_MM, error_mm);
}

/* Interpolate the measured front-minus-rear bias: 59/41 in the narrow
 * furrow and 56/53 in the regular furrow. Keep the full tolerance band
 * clear of the raw safety thresholds. The rear target is the remainder. */
static float route_furrow_front_target_mm(float sum_mm) {
    const float blend = fminf(1.0f, fmaxf(0.0f,
        (sum_mm - LASER_FURROW_NARROW_SUM_MM) /
        (LASER_FURROW_CENTER_SUM_MM - LASER_FURROW_NARROW_SUM_MM)));
    const float bias_mm = LASER_FURROW_NARROW_BIAS_MM +
                          blend * (LASER_FURROW_CENTER_BIAS_MM - LASER_FURROW_NARROW_BIAS_MM);
    const float front_min = LASER_FRONT_SAFETY_MM + LASER_DUAL_TOLERANCE_MM + 1.0f;
    const float rear_min = LASER_SAFETY_MM + LASER_DUAL_TOLERANCE_MM + 1.0f;
    return fminf(sum_mm - rear_min, fmaxf(front_min, 0.5f * (sum_mm + bias_mm)));
}

static float route_side_target_mm(LaserBeam beam) {
    return beam == LASER_LEFT ? LASER_LEFT_TARGET_MM : LASER_RIGHT_TARGET_MM;
}

static float route_lateral_target(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                  uint32_t now, bool launch_zone) {
    const LaserBeam beam = leg->control_beam;
    s_odom_test.laser.track_mask = 0u;
    s_odom_test.laser.mask = 0u;
    if(MAP_ROUTE_LASER_GUIDANCE_ENABLED == 0 || beam == LASER_NO_BEAM) return 0.0f;
    /* Ignore the distant left wall until the first <=105 mm acquisition. */
    if(s_odom_test.segment_index == 0u && s_odom_test.phase == ROUTE_TRAVEL &&
       !s_odom_test.guidance_acquired) return 0.0f;

    float target;
    float correction_limit = LASER_SIDE_CORRECTION_LIMIT_MPS;
    if(beam == LASER_FRONT && leg->furrow_gate) {
        const uint16_t front_mm = sample->distance_mm[LASER_FRONT];
        const uint16_t rear_mm = sample->distance_mm[LASER_REAR];
        const float sum = (float)front_mm + (float)rear_mm;
        const float margin = s_odom_test.furrow_gate_open ? LASER_FURROW_HYSTERESIS_MM : 0.0f;
        s_odom_test.furrow_gate_open = route_laser_valid(sample, LASER_FRONT, now) &&
            route_laser_valid(sample, LASER_REAR, now) &&
            sum >= LASER_FURROW_SUM_MIN_MM - margin && sum <= LASER_FURROW_SUM_MAX_MM + margin;
        if(!s_odom_test.furrow_gate_open) return 0.0f;
        const float front_target_mm = route_furrow_front_target_mm(sum);
        /* FRONT=-vx and REAR=+vx. Both targets share the measured clearance. */
        const float front_error = route_target_error_mm(front_mm, front_target_mm);
        const float rear_error = route_target_error_mm(rear_mm, sum - front_target_mm);
        target = 0.5f * (rear_error - front_error) * LASER_DUAL_CONTROL_GAIN * 0.001f;
        correction_limit = LASER_DUAL_CORRECTION_LIMIT_MPS;
        s_odom_test.laser.track_mask = (1u << LASER_FRONT) | (1u << LASER_REAR);
        s_odom_test.laser.mask = APP_LASER_X;
    }
    else {
        if(!route_laser_valid(sample, beam, now) ||
           sample->distance_mm[beam] > LASER_SIDE_GUIDANCE_MAX_MM) return 0.0f;
        const float target_mm = launch_zone ? LASER_LAUNCH_LEFT_TARGET_MM :
                                              route_side_target_mm(beam);
        const float error_mm = route_target_error_mm(sample->distance_mm[beam], target_mm);
        /* LEFT=-vy, RIGHT=+vy: positive range error approaches that wall. */
        const float beam_sign = beam == LASER_RIGHT ? 1.0f : -1.0f;
        target = beam_sign * LASER_SIDE_CONTROL_GAIN * error_mm * 0.001f;
        s_odom_test.laser.track_mask = 1u << beam;
        s_odom_test.laser.mask = APP_LASER_Y;
    }
    return route_clampf(target, correction_limit);
}

static float route_correction_step(uint32_t* last_control_ms, uint32_t now) {
    const float dt = fminf(APP_ODOM_TEST_CONTROL_PERIOD_S * 10.0f,
                           (float)(now - *last_control_ms) * 0.001f);
    *last_control_ms = now;
    return LASER_CORRECTION_ACCEL_MPS2 * dt;
}

static float route_slew_correction(float target, float* correction_mps,
                                   uint32_t* last_control_ms, uint32_t now) {
    const float step = route_correction_step(last_control_ms, now);
    if(target == 0.0f || target * *correction_mps < 0.0f) return 0.0f;
    const float previous = fabsf(*correction_mps);
    return copysignf(fminf(fabsf(target), previous + step), target);
}

static float route_acceleration_speed(float progress_m) {
    const float profile = sqrtf(2.0f * s_odom_test.accel_mps2 * fmaxf(0.0f, progress_m));
    /* At exactly zero progress a sqrt(position) cap would lock the vehicle at
     * zero forever. The rate limiter still controls the actual acceleration. */
    return fmaxf(ROUTE_PROFILE_START_SPEED_MPS, profile);
}

static float route_braking_speed(float remaining_m) {
    return sqrtf(2.0f * s_odom_test.decel_mps2 * fmaxf(0.0f, remaining_m));
}

static float route_wall_speed_limit(uint16_t wall_mm, float reserve_mm, float cruise_mps) {
    /* Constant-deceleration speed cap from 400 mm down to the retained gap. */
    const float fraction = ((float)wall_mm - reserve_mm) / (LASER_WALL_BRAKE_START_MM - reserve_mm);
    return cruise_mps * sqrtf(fminf(1.0f, fmaxf(0.0f, fraction)));
}

static float route_update_speed(float target_mps, float dt_s) {
    const float previous = s_odom_test.route_speed_mps;
    const float accel_step = s_odom_test.accel_mps2 * dt_s;
    const float decel_step = s_odom_test.decel_mps2 * dt_s;
    if(previous < target_mps) return fminf(target_mps, previous + accel_step);
    return fmaxf(target_mps, previous - decel_step);
}

static void route_log_speed_phase(RouteSpeedPhase next, float remaining_m, uint32_t now) {
    if(next == s_odom_test.speed_phase) return;
    static const char* const names[] = { "STOP", "ACCEL", "CRUISE", "DECEL" };
    s_odom_test.speed_phase = next;
    log_info("MAP SPEED seg=%u phase=%s v=%.3f rem=%.3f t=%lu",
             (unsigned int)s_odom_test.segment_index + 1u, names[next],
             (double)s_odom_test.route_speed_mps, (double)remaining_m, (unsigned long)now);
}

static float route_escape_speed(uint16_t raw, float safety_mm) {
    const float error_m = (safety_mm - (float)raw) * 0.001f;
    return fminf(LASER_ESCAPE_LIMIT_MPS, LASER_ESCAPE_GAIN * error_m);
}

/* FRONT uses 55 mm; LEFT/RIGHT/REAR use 30 mm. Escape is independent of target
 * control and remains active while travelling and settling. */
static RouteEscape route_safety_escape(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                       uint32_t now, float vx, float vy) {
    const bool front = route_laser_valid(sample, LASER_FRONT, now) &&
                       sample->distance_mm[LASER_FRONT] < LASER_FRONT_SAFETY_MM;
    const bool left = route_laser_valid(sample, LASER_LEFT, now) &&
                      sample->distance_mm[LASER_LEFT] < LASER_SAFETY_MM;
    const bool right = route_laser_valid(sample, LASER_RIGHT, now) &&
                       sample->distance_mm[LASER_RIGHT] < LASER_SAFETY_MM;
    const bool rear = leg->rear_escape && route_laser_valid(sample, LASER_REAR, now) &&
                      sample->distance_mm[LASER_REAR] < LASER_SAFETY_MM;
    RouteEscape escape = { vx, vy, 0u };

    /* Cancel every unsafe base-motion component before adding escape speeds. */
    if((front && escape.vx < 0.0f) || (rear && escape.vx > 0.0f)) escape.vx = 0.0f;
    if((left && escape.vy < 0.0f) || (right && escape.vy > 0.0f)) escape.vy = 0.0f;

    /* Opposing dangers on one axis hold that axis at zero instead of letting
     * the later beam overwrite the earlier escape command. */
    if(front != rear) {
        if(front) {
            escape.vx += route_escape_speed(sample->distance_mm[LASER_FRONT],
                                            LASER_FRONT_SAFETY_MM);
            escape.mask |= 1u << LASER_FRONT;
        }
        else {
            escape.vx -= route_escape_speed(sample->distance_mm[LASER_REAR], LASER_SAFETY_MM);
            escape.mask |= 1u << LASER_REAR;
        }
    }
    else if(front) {
        escape.vx = 0.0f;
        escape.mask |= 1u << LASER_FRONT | 1u << LASER_REAR;
    }
    if(left != right) {
        if(left) {
            escape.vy += fmaxf(LASER_SIDE_ESCAPE_MIN_MPS,
                               route_escape_speed(sample->distance_mm[LASER_LEFT], LASER_SAFETY_MM));
            escape.mask |= 1u << LASER_LEFT;
        }
        else {
            escape.vy -= fmaxf(LASER_SIDE_ESCAPE_MIN_MPS,
                               route_escape_speed(sample->distance_mm[LASER_RIGHT], LASER_SAFETY_MM));
            escape.mask |= 1u << LASER_RIGHT;
        }
    }
    else if(left) {
        escape.vy = 0.0f;
        escape.mask |= 1u << LASER_LEFT | 1u << LASER_RIGHT;
    }
    escape.vx = route_clampf(escape.vx, ROUTE_COMMAND_LIMIT_MPS);
    escape.vy = route_clampf(escape.vy, ROUTE_COMMAND_LIMIT_MPS);
    return escape;
}

static AppControlResult route_send_command(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                           uint32_t now, float yaw, float measured_wz,
                                           float vx_route, float vy_route, bool* safety_active) {
    /* Wheel yaw defines the odometry axes. IMU-relative yaw rotates a route
     * command into the present body axes, where the laser beams are mounted. */
    const float heading_delta = yaw - s_odom_test.imu_heading_ref;
    const float c = cosf(heading_delta);
    const float s = sinf(heading_delta);
    const RouteEscape escape = route_safety_escape(leg, sample, now,
        c * vx_route + s * vy_route, c * vy_route - s * vx_route);
    s_odom_test.laser.jump_mask = escape.mask;
    *safety_active = escape.mask != 0u;
    const float vx_cmd = escape.vx;
    const float vy_cmd = escape.vy;
    /* The route never requests rotation; wz only corrects the latched heading. */
    const float wz_cmd = chassis_yaw_hold_apply(vx_cmd, vy_cmd, 0.0f, yaw, measured_wz,
                                                APP_ODOM_TEST_CONTROL_PERIOD_S);
    if(chassis.set_velocity(vx_cmd, vy_cmd, wz_cmd) != CHASSIS_OK) {
        return APP_CONTROL_RESULT_CHASSIS_ERROR;
    }
    return APP_CONTROL_RESULT_OK;
}

static void route_start_leg(const MapRouteLeg* leg, float distance_m, Vector3 pose,
                            float yaw, uint32_t now) {
    if(s_odom_test.segment_index == 0u) {
        s_odom_test.heading_cos = cosf(yaw);
        s_odom_test.heading_sin = sinf(yaw);
    }
    s_odom_test.origin = pose;
    s_odom_test.laser_history = (RouteLaserHistory){ 0 };
    s_odom_test.furrow_gate_open = false;
    s_odom_test.connector_hold = false;
    s_odom_test.wall_anchor_mask = 0u;
    s_odom_test.phase = ROUTE_TRAVEL;
    s_odom_test.speed_phase = ROUTE_SPEED_STOP;
    s_odom_test.accel_distance_m = fminf(ROUTE_ACCEL_DISTANCE_M, 0.5f * distance_m);
    s_odom_test.decel_distance_m = fminf(ROUTE_DECEL_DISTANCE_M, 0.5f * distance_m);
    s_odom_test.accel_mps2 = leg->speed_mps * leg->speed_mps /
                             (2.0f * s_odom_test.accel_distance_m);
    s_odom_test.decel_mps2 = leg->speed_mps * leg->speed_mps /
                             (2.0f * s_odom_test.decel_distance_m);
    s_odom_test.travel_ms = now;
    s_odom_test.last_control_ms = now;
    s_odom_test.settle_ready_since_ms = 0u;
    s_odom_test.laser_waiting = false;
    s_odom_test.guidance_acquired = false;
    s_odom_test.route_speed_mps = 0.0f;
    s_odom_test.correction_mps = 0.0f;
    log_info("MAP TRAVEL %u/%u goal=(%.2f,%.2f) vmax=%.2f",
             (unsigned int)s_odom_test.segment_index + 1u, (unsigned int)map_route_count,
             (double)leg->x_m, (double)leg->y_m, (double)leg->speed_mps);
    log_info("MAP PROFILE seg=%u accel_d=%.3f decel_d=%.3f a=%.3f d=%.3f",
             (unsigned int)s_odom_test.segment_index + 1u,
             (double)s_odom_test.accel_distance_m, (double)s_odom_test.decel_distance_m,
             (double)s_odom_test.accel_mps2, (double)s_odom_test.decel_mps2);
}

static void route_begin_settle(uint32_t now, LaserBeam wall) {
    s_odom_test.phase = ROUTE_SETTLE;
    s_odom_test.settle_ready_since_ms = 0u;
    s_odom_test.route_speed_mps = 0.0f;
    s_odom_test.correction_mps = 0.0f;
    route_log_speed_phase(ROUTE_SPEED_STOP, 0.0f, now);
    s_odom_test.wall_anchor_mask = wall == LASER_NO_BEAM ? 0u : 1u << wall;
    log_info("MAP SETTLE %u xy=(%.3f,%.3f) wall=0x%X",
             (unsigned int)s_odom_test.segment_index + 1u,
             (double)s_odom_test.laser.x, (double)s_odom_test.laser.y, s_odom_test.wall_anchor_mask);
}

static AppControlResult route_next_or_park(uint32_t now) {
    const AppControlResult stop = app_control_hold_zero_chassis();
    if(stop != APP_CONTROL_RESULT_OK) return stop;
    log_info("MAP ALIGNED seg=%u t=%lu", (unsigned int)s_odom_test.segment_index + 1u,
             (unsigned long)now);
    if(s_odom_test.segment_index + 1u < map_route_count) {
        ++s_odom_test.segment_index;
        s_odom_test.phase = ROUTE_START_LEG;
        s_odom_test.speed_phase = ROUTE_SPEED_STOP;
        s_odom_test.route_speed_mps = 0.0f;
        s_odom_test.correction_mps = 0.0f;
        s_odom_test.settle_ready_since_ms = 0u;
        log_info("MAP NEXT %u/%u", (unsigned int)s_odom_test.segment_index + 1u,
                 (unsigned int)map_route_count);
        return stop;
    }

    ++s_odom_test.segment_index;
    s_odom_test.phase = ROUTE_PARKED;
    s_odom_test.remaining_m = 0.0f;
    s_odom_test.route_speed_mps = 0.0f;
    s_odom_test.correction_mps = 0.0f;
    s_odom_test.settle_ready_since_ms = now;
    log_info("MAP PARKED");
    return stop;
}

static bool route_is_stationary(Vector3 velocity) {
    return hypotf(velocity.x, velocity.y) <= APP_ODOM_TEST_STOP_SPEED_MPS &&
           fabsf(velocity.z) <= 0.08f;
}

static AppControlResult route_settle(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                     LaserBeam wall, float progress_m, Vector3 velocity, float imu_yaw,
                                     float imu_wz, uint32_t now) {
    s_odom_test.laser.anchor_mask = s_odom_test.wall_anchor_mask;
    bool safety_active = false;
    const AppControlResult command = route_send_command(leg, sample, now, imu_yaw, imu_wz,
                                                        0.0f, 0.0f, &safety_active);
    if(command != APP_CONTROL_RESULT_OK) return command;

    const bool safety_ready = leg->control_beam == LASER_NO_BEAM || route_frame_fresh(sample, now);
    if(s_odom_test.entry_pump_state == ENTRY_PUMP_ACTIVE) {
        const bool pump_running = app_odom_test_entry_pump_enabled();
        if(safety_active || !safety_ready) {
            s_odom_test.settle_ready_since_ms = 0u;
            if(pump_running) s_odom_test.entry_pump_state = ENTRY_PUMP_PENDING;
        }
        /* Apply the output change before the next control tick can advance. */
        return command;
    }
    if(!route_is_stationary(velocity) || safety_active || !safety_ready) {
        s_odom_test.settle_ready_since_ms = 0u;
        return command;
    }
    if(s_odom_test.settle_ready_since_ms == 0u) s_odom_test.settle_ready_since_ms = now;
    if(now - s_odom_test.settle_ready_since_ms < APP_ODOM_TEST_SETTLE_READY_MS) return command;

    if(s_odom_test.segment_index == 0u && s_odom_test.entry_pump_state == ENTRY_PUMP_PENDING) {
        log_info("MAP ENTRY PUMP seg=1 prime=%u ms PA0=%u ms",
                 (unsigned int)ROUTE_ENTRY_PUMP_PRIME_MS, (unsigned int)ROUTE_ENTRY_PA0_MS);
        s_odom_test.entry_pump_since_ms = delay_now_ms();
        s_odom_test.entry_pump_state = ENTRY_PUMP_ACTIVE;
        return command;
    }
    if(!leg->endpoint_side_trim) return route_next_or_park(now);

    /* A stop at the endpoint starts the side-wall alignment. */
    s_odom_test.wall_trim_start_m = progress_m;
    s_odom_test.wall_trim_started = false;
    s_odom_test.wall_trim_motion_exhausted = false;
    s_odom_test.wall_trim_recovery_pending = false;
    s_odom_test.wall_trim_recovery_used = false;
    route_wall_trim_reset_confirmation(sample->received);
    s_odom_test.travel_ms = now;
    s_odom_test.phase = ROUTE_WALL_TRIM;
    log_info("MAP WALL ALIGN seg=%u beam=%u target=%.0f", s_odom_test.laser.segment,
             (unsigned int)wall, (double)route_side_target_mm(wall));
    return command;
}

static bool route_wall_trim_confirmed(const LaserOdomSample* sample, bool ready, uint32_t now) {
    if(!ready) {
        route_wall_trim_reset_confirmation(sample->received);
        return false;
    }
    if(sample->received == s_odom_test.wall_trim_confirm_frame) return false;

    s_odom_test.wall_trim_confirm_frame = sample->received;
    if(s_odom_test.wall_trim_confirm_count == 0u) s_odom_test.settle_ready_since_ms = now;
    if(s_odom_test.wall_trim_confirm_count < LASER_WALL_TRIM_CONFIRM_FRAMES) {
        ++s_odom_test.wall_trim_confirm_count;
    }
    return s_odom_test.wall_trim_confirm_count >= LASER_WALL_TRIM_CONFIRM_FRAMES &&
           now - s_odom_test.settle_ready_since_ms >= APP_ODOM_TEST_SETTLE_READY_MS;
}

/* Bound the correction with wheel travel and time. A live wall measurement
 * while the vehicle is stationary confirms alignment for the next leg. */
static AppControlResult route_wall_trim(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                        const LaserOdomSample* filtered, LaserBeam wall, float progress_m,
                                        Vector3 velocity, float imu_yaw, float imu_wz, uint32_t now) {
    const uint16_t wall_mm = filtered->distance_mm[wall];
    const float target_mm = route_side_target_mm(wall);
    const bool range_valid = route_laser_valid(filtered, wall, now) &&
                             wall_mm <= LASER_WALL_TRIM_CAPTURE_MM;
    const float error_mm = (float)wall_mm - target_mm;
    const bool in_band = range_valid && fabsf(error_mm) <= LASER_WALL_TRIM_TOLERANCE_MM;
    bool safety_active = route_safety_escape(leg, sample, now, 0.0f, 0.0f).mask != 0u;
    const uint16_t raw_mm = sample->distance_mm[wall];
    const float raw_error_mm = (float)raw_mm - target_mm;
    const bool can_correct = range_valid && !in_band && !safety_active &&
                             route_laser_valid(sample, wall, now) && raw_error_mm * error_mm > 0.0f &&
                             fabsf(raw_error_mm) > LASER_WALL_TRIM_TOLERANCE_MM;
    if(safety_active && !s_odom_test.wall_trim_recovery_used) {
        s_odom_test.wall_trim_recovery_pending = true;
    }
    else if(!safety_active && s_odom_test.wall_trim_recovery_pending && range_valid &&
            wall_mm >= LASER_SAFETY_MM) {
        /* Give one new bounded attempt after the safety escape has cleared. */
        s_odom_test.wall_trim_recovery_pending = false;
        s_odom_test.wall_trim_recovery_used = true;
        s_odom_test.wall_trim_start_m = progress_m;
        s_odom_test.wall_trim_started = false;
        s_odom_test.wall_trim_motion_exhausted = false;
        route_wall_trim_reset_confirmation(sample->received);
        log_info("MAP WALL ALIGN RESUME seg=%u beam=%u", s_odom_test.laser.segment, (unsigned int)wall);
    }
    const float extra_m = progress_m - s_odom_test.wall_trim_start_m;
    const float wheel_budget_m = ROUTE_WALL_TRIM_MAX_M - fabsf(extra_m);
    const bool distance_limit = wheel_budget_m <= ROUTE_END_TOLERANCE_M;
    const bool time_limit = s_odom_test.wall_trim_started &&
                            now - s_odom_test.wall_trim_started_ms >= ROUTE_WALL_TRIM_MAX_MS;
    if(!s_odom_test.wall_trim_motion_exhausted && (distance_limit || time_limit)) {
        s_odom_test.wall_trim_motion_exhausted = true;
        log_info("MAP WALL ALIGN LIMIT seg=%u reason=%s extra=%.3f raw=%u filtered=%u",
                 s_odom_test.laser.segment, distance_limit ? "distance" : "time", (double)extra_m,
                 (unsigned int)raw_mm, (unsigned int)wall_mm);
    }

    float target = 0.0f;
    if(can_correct && !s_odom_test.wall_trim_motion_exhausted) {
        /* A raw sample may stop a correction at the edge of the band, while
         * only the filtered range can choose its direction or confirm arrival. */
        const float budget_speed = sqrtf(2.0f * LASER_WALL_BRAKE_MPS2 * wheel_budget_m);
        const float speed = fmaxf(ROUTE_WALL_TRIM_CRAWL_MPS,
                                 LASER_SIDE_CONTROL_GAIN * fabsf(error_mm) * 0.001f);
        const float direction = wall == LASER_LEFT ? -error_mm : error_mm;
        target = copysignf(fminf(speed, fminf(ROUTE_WALL_TRIM_SPEED_MPS, budget_speed)), direction);
        if(!s_odom_test.wall_trim_started) {
            s_odom_test.wall_trim_started = true;
            s_odom_test.wall_trim_started_ms = now;
        }
    }

    const float dt = fminf(APP_ODOM_TEST_CONTROL_PERIOD_S * 10.0f,
                           (float)(now - s_odom_test.travel_ms) * 0.001f);
    s_odom_test.travel_ms = now;
    if(target * s_odom_test.route_speed_mps < 0.0f) s_odom_test.route_speed_mps = 0.0f;
    s_odom_test.route_speed_mps = route_clampf(route_update_speed(target, dt), fabsf(target));
    if(range_valid) {
        s_odom_test.laser.mask = APP_LASER_Y;
        s_odom_test.laser.track_mask = 1u << wall;
    }
    s_odom_test.laser.correction_mps = s_odom_test.route_speed_mps;
    const AppControlResult command = route_send_command(leg, sample, now, imu_yaw, imu_wz,
        0.0f, s_odom_test.route_speed_mps, &safety_active);
    if(command != APP_CONTROL_RESULT_OK) return command;
    if(route_wall_trim_confirmed(sample, in_band && !safety_active && route_is_stationary(velocity), now)) {
        s_odom_test.laser.anchor_mask = 1u << wall;
        log_info("MAP WALL ALIGNED seg=%u beam=%u range=%u extra=%.3f", s_odom_test.laser.segment,
                 (unsigned int)wall, (unsigned int)wall_mm, (double)extra_m);
        return route_next_or_park(now);
    }
    return command;
}

AppControlResult app_odom_test_process(void) {
    const uint32_t now = delay_now_ms();
    if(s_odom_test.phase == ROUTE_PARKED) return app_control_hold_zero_chassis();

    Vector3 position, angle, velocity;
    if(odom.get_odom(&position) != ODOM_OK || odom.get_angle(&angle) != ODOM_OK ||
       odom.get_velocity(&velocity) != ODOM_OK || !route_finite_vec3(position) ||
       !isfinite(angle.z) || !route_finite_vec3(velocity) || !odom.get_state()->imu_ready) {
        return route_odom_error();
    }

    /* Auto heading feedback comes only from the IMU, never from wheel yaw. */
    const ImuAngle imu_angle = imu.get_angle();
    const ImuGyro imu_gyro = imu_get_gyro_corrected();
    const float cos_pitch = cosf(imu_angle.pitch);
    if(!isfinite(imu_angle.roll) || !isfinite(imu_angle.pitch) || !isfinite(imu_angle.yaw) ||
       !isfinite(imu_gyro.x) || !isfinite(imu_gyro.y) || !isfinite(imu_gyro.z) ||
       fabsf(cos_pitch) <= 0.2f) {
        return route_odom_error();
    }
    const float imu_yaw = imu_angle.yaw;
    const float imu_wz = (imu_gyro.y * sinf(imu_angle.roll) +
                          imu_gyro.z * cosf(imu_angle.roll)) / cos_pitch;
    if(!isfinite(imu_wz)) return route_odom_error();
    if(!s_odom_test.yaw_target_valid) {
        /* First valid IMU sample in Auto owns the reference for the whole route. */
        chassis_yaw_hold_set_target(imu_yaw);
        s_odom_test.imu_heading_ref = imu_yaw;
        s_odom_test.yaw_target_valid = true;
        log_info("AUTO IMU YAW LOCK ref=%.5f", (double)imu_yaw);
    }

    const size_t index = s_odom_test.segment_index;
    if(index >= map_route_count) return APP_CONTROL_RESULT_COMMAND_INVALID;
    const MapRouteLeg* leg = &map_route[index];
    const float from_x = index ? map_route[index - 1u].x_m : MAP_START_X_M;
    const float from_y = index ? map_route[index - 1u].y_m : MAP_START_Y_M;
    const bool test_vy = leg->x_m != from_x;
    const float delta = test_vy ? leg->x_m - from_x : leg->y_m - from_y;
    const float distance_m = fabsf(delta) + leg->wheel_extra_m;
    const float sign = copysignf(1.0f, delta);
    const LaserOdomSample* sample = laser_odom_get_sample();

    if(!isfinite(delta) || delta == 0.0f || (test_vy && leg->y_m != from_y) ||
       (leg->endpoint_side_trim && !test_vy) ||
       !isfinite(leg->speed_mps) || leg->speed_mps <= 0.0f || leg->control_beam == LASER_REAR) {
        return APP_CONTROL_RESULT_COMMAND_INVALID;
    }
    if(s_odom_test.phase == ROUTE_START_LEG) route_start_leg(leg, distance_m, position, angle.z, now);
    const LaserOdomSample filtered = route_filter_laser(sample, now);
    route_update_guidance_acquisition(&filtered, now);

    const float dx = position.x - s_odom_test.origin.x;
    const float dy = position.y - s_odom_test.origin.y;
    /* Proper rotation at launch: field +X=+body Y, field +Y=-body X. */
    const float field_x = -dx * s_odom_test.heading_sin + dy * s_odom_test.heading_cos;
    const float field_y = -dx * s_odom_test.heading_cos - dy * s_odom_test.heading_sin;
    const float along = test_vy ? field_x : field_y;
    const float progress_m = sign * along;
    const float remaining = distance_m - progress_m;
    s_odom_test.remaining_m = fmaxf(0.0f, remaining);
    s_odom_test.laser = (AppLaserFix){
        .x = map_route_coordinate(true, field_x),
        .y = map_route_coordinate(false, field_y),
        .segment = (unsigned int)index + 1u,
    };

    if(route_work_active()) return route_work_step(now, velocity);

    const bool work_zone = s_odom_test.phase == ROUTE_TRAVEL && test_vy && leg->work_stop &&
                           progress_m >= ROUTE_WORK_END_MARGIN_M &&
                           remaining >= ROUTE_WORK_END_MARGIN_M;
    if(route_work_target_detected(work_zone, now)) {
        s_odom_test.work_target_latched = true;
        s_odom_test.work_stationary_seen = false;
        s_odom_test.route_speed_mps = 0.0f;
        s_odom_test.correction_mps = 0.0f;
        s_odom_test.phase = ROUTE_WORK_STOPPING;
        log_info("MAP WORK stop seg=%u range=%u", (unsigned int)index + 1u,
                 (unsigned int)s_work_distance_mm);
        return route_work_step(now, velocity);
    }

    const LaserBeam end_wall = index == 0u ? LASER_FRONT :
        (test_vy ? (sign > 0.0f ? LASER_RIGHT : LASER_LEFT) : LASER_NO_BEAM);
    if(s_odom_test.phase == ROUTE_SETTLE) {
        return route_settle(leg, sample, end_wall, progress_m, velocity, imu_yaw, imu_wz, now);
    }
    if(s_odom_test.phase == ROUTE_WALL_TRIM) {
        return route_wall_trim(leg, sample, &filtered, end_wall, progress_m, velocity, imu_yaw, imu_wz, now);
    }

    /* Near-wall braking may finish normal travel in the last 80 mm. */
    const float wall_reserve = end_wall == LASER_FRONT ? LASER_FRONT_WALL_RESERVE_MM :
        (end_wall == LASER_LEFT ? LASER_LEFT_WALL_RESERVE_MM : LASER_RIGHT_WALL_RESERVE_MM);
    const bool wall_valid = route_laser_valid(&filtered, end_wall, now);
    const float wall_accept_mm = wall_reserve + LASER_WALL_ACCEPT_MARGIN_MM;
    const bool near_wall = wall_valid && (float)filtered.distance_mm[end_wall] <= wall_accept_mm;
    /* If wheel odometry is short, confirm a stopped crossing at its expected
     * side wall before accepting the bounded recovery window. */
    const bool recovery_wall = test_vy && near_wall && remaining > ROUTE_WALL_END_WINDOW_M &&
        remaining <= ROUTE_WALL_RECOVERY_WINDOW_M && filtered.distance_mm[end_wall] >= LASER_SAFETY_MM &&
        filtered.distance_mm[end_wall] <= wall_reserve && route_laser_valid(sample, end_wall, now) &&
        sample->distance_mm[end_wall] >= LASER_SAFETY_MM && sample->distance_mm[end_wall] <= wall_accept_mm &&
        route_is_stationary(velocity);
    const bool recovered_arrival = test_vy && route_wall_trim_confirmed(sample, recovery_wall, now);
    const bool wall_arrival = (near_wall && remaining <= ROUTE_WALL_END_WINDOW_M) || recovered_arrival;
    const bool wheel_arrival = remaining <= ROUTE_END_TOLERANCE_M;
    if(wheel_arrival || wall_arrival) {
        const bool early_anchor = !leg->endpoint_side_trim && wall_arrival && remaining > ROUTE_END_TOLERANCE_M;
        route_begin_settle(now, early_anchor ? end_wall : LASER_NO_BEAM);
        return route_settle(leg, sample, end_wall, progress_m, velocity, imu_yaw, imu_wz, now);
    }

    /* The start lane ends at field Y=0, 0.50 m from the marked launch. */
    const bool launch_zone = index == 0u && progress_m < -MAP_START_Y_M;
    const float correction_target = route_lateral_target(leg, &filtered, now, launch_zone);
    const bool dual_travel = test_vy && leg->control_beam == LASER_FRONT && leg->furrow_gate;
    if(dual_travel) {
        const float step = route_correction_step(&s_odom_test.last_control_ms, now);
        const float delta_mps = correction_target - s_odom_test.correction_mps;
        s_odom_test.correction_mps += route_clampf(delta_mps, step);
    }
    else {
        s_odom_test.correction_mps = route_slew_correction(correction_target,
                                                           &s_odom_test.correction_mps,
                                                           &s_odom_test.last_control_ms, now);
    }
    if(s_odom_test.laser.mask == 0u) s_odom_test.correction_mps = 0.0f;
    const bool wait_laser = MAP_ROUTE_LASER_GUIDANCE_ENABLED != 0 &&
                            leg->control_beam != LASER_NO_BEAM && !route_frame_fresh(sample, now);
    if(wait_laser != s_odom_test.laser_waiting) {
        log_info("LASER %s seg=%u", wait_laser ? "WAIT" : "RESUME", (unsigned int)index + 1u);
        s_odom_test.laser_waiting = wait_laser;
    }
    const LaserBeam side = leg->control_beam;
    if(index != 0u && !test_vy && (side == LASER_LEFT || side == LASER_RIGHT)) {
        const bool side_valid = route_laser_valid(&filtered, side, now);
        const bool raw_close = route_laser_valid(sample, side, now) &&
                               sample->distance_mm[side] < LASER_SAFETY_MM;
        const bool too_close = s_odom_test.connector_hold ?
            filtered.distance_mm[side] < LASER_CONNECT_RESUME_MM :
            filtered.distance_mm[side] <= LASER_CONNECT_HOLD_MM;
        s_odom_test.connector_hold = raw_close || !side_valid || too_close;
        if(side_valid && too_close) {
            const float escape_speed = fmaxf(LASER_SIDE_ESCAPE_MIN_MPS, fabsf(s_odom_test.correction_mps));
            s_odom_test.correction_mps = side == LASER_LEFT ? escape_speed : -escape_speed;
        }
    }

    /* Position-based trapezoid: acceleration uses 400 mm from the leg start,
     * braking uses 400 mm to the wheel endpoint. A 700 mm connector uses two
     * 350 mm ramps because two 400 mm zero-end ramps cannot physically fit. */
    const float accel_speed = route_acceleration_speed(progress_m);
    const float braking_speed = route_braking_speed(remaining);
    float speed_limit = fminf(leg->speed_mps, fminf(accel_speed, braking_speed));
    const float wall_speed = wall_valid ?
        route_wall_speed_limit(filtered.distance_mm[end_wall], wall_reserve, leg->speed_mps) : leg->speed_mps;
    speed_limit = fminf(speed_limit, wall_speed);
    if(s_odom_test.connector_hold) speed_limit = 0.0f;
    RouteSpeedPhase speed_phase = ROUTE_SPEED_CRUISE;
    if(wait_laser || speed_limit <= 0.0f) speed_phase = ROUTE_SPEED_STOP;
    else if(speed_limit < s_odom_test.route_speed_mps - 0.001f) speed_phase = ROUTE_SPEED_DECEL;
    else if(s_odom_test.route_speed_mps < speed_limit - 0.001f) speed_phase = ROUTE_SPEED_ACCEL;
    else if(speed_limit < leg->speed_mps - 0.001f) speed_phase = ROUTE_SPEED_DECEL;
    route_log_speed_phase(speed_phase, remaining, now);

    const float dt = fminf(APP_ODOM_TEST_CONTROL_PERIOD_S * 10.0f,
                           (float)(now - s_odom_test.travel_ms) * 0.001f);
    s_odom_test.travel_ms = now;
    s_odom_test.route_speed_mps = route_update_speed(wait_laser ? 0.0f : speed_limit, dt);
    s_odom_test.route_speed_mps = fminf(s_odom_test.route_speed_mps, wall_speed);
    if(s_odom_test.connector_hold) s_odom_test.route_speed_mps = 0.0f;
    if(wait_laser) s_odom_test.route_speed_mps = s_odom_test.correction_mps = 0.0f;

    /* Field +X=+vy and field +Y=-vx. */
    const float drive = (test_vy ? sign : -sign) * s_odom_test.route_speed_mps;
    const float vx_route = test_vy ? s_odom_test.correction_mps : drive;
    const float vy_route = test_vy ? drive : s_odom_test.correction_mps;
    bool safety_active = false;
    const AppControlResult command = route_send_command(leg, sample, now, imu_yaw, imu_wz,
                                                        vx_route, vy_route, &safety_active);
    if(command != APP_CONTROL_RESULT_OK) return command;

    s_odom_test.laser.correction_mps = s_odom_test.correction_mps;
    if(safety_active) {
        const unsigned int escape_mask = s_odom_test.laser.jump_mask;
        const LaserBeam forward_body = test_vy ? end_wall : (sign > 0.0f ? LASER_FRONT : LASER_REAR);
        const unsigned int cross_axis = test_vy ? (1u << LASER_FRONT) | (1u << LASER_REAR) :
                                                (1u << LASER_LEFT) | (1u << LASER_RIGHT);
        /* Re-ramp the interrupted axes after the raw safety override clears. */
        if(escape_mask & (1u << forward_body)) s_odom_test.route_speed_mps = 0.0f;
        if(escape_mask & cross_axis) s_odom_test.correction_mps = 0.0f;
    }
    return APP_CONTROL_RESULT_OK;
}
