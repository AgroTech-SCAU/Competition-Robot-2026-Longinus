#include "app_odom_test.h"

#include "chassis.h"
#include "delay.h"
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
#define ROUTE01_FINAL_APPROACH_MPS 0.04f
#define ROUTE_END_TOLERANCE_M 0.005f
#define ROUTE_LASER01_WINDOW_M 0.85f
#define LASER_TARGET_MM 65.0f
#define LASER_TARGET_TOLERANCE_MM 8.0f
#define LASER_CONTROL_DEADBAND_MM 3.0f
#define LASER_FURROW_SUM_MIN_MM 100.0f
#define LASER_FURROW_SUM_MAX_MM 140.0f
#define LASER_CONTROL_GAIN 1.0f
#define LASER_CORRECTION_LIMIT_MPS 0.06f
#define LASER_CORRECTION_ACCEL_MPS2 0.30f
#define LASER_FRAME_MAX_AGE_MS 120u
#define LASER_SAFETY_MM 55.0f
#define LASER_ESCAPE_GAIN 4.0f
#define LASER_ESCAPE_LIMIT_MPS 0.12f
#define ROUTE_COMMAND_LIMIT_MPS 0.65f
#define ROUTE_ALIGN_LOG_PERIOD_MS 250u

typedef enum {
    ROUTE_START_LEG = 0,
    ROUTE_TRAVEL,
    ROUTE_SETTLE,
    ROUTE_PARKED
} RoutePhase;

typedef enum {
    ROUTE_SPEED_STOP = 0,
    ROUTE_SPEED_ACCEL,
    ROUTE_SPEED_CRUISE,
    ROUTE_SPEED_DECEL
} RouteSpeedPhase;

typedef struct {
    size_t segment_index;
    RoutePhase phase;
    RouteSpeedPhase speed_phase;
    float remaining_m;
    float route_speed_mps;
    float correction_mps;
    float accel_mps2;
    float decel_mps2;
    float accel_distance_m;
    float decel_distance_m;
    Vector3 origin;
    float heading_cos;
    float heading_sin;
    uint32_t travel_ms;
    uint32_t progress_ms;
    uint32_t settle_since_ms;
    uint32_t settle_ready_since_ms;
    uint32_t align_log_ms;
    uint32_t last_control_ms;
    float last_progress_position_m;
    bool laser_waiting;
    bool following_laser;
    bool final_approach;
    bool alignment_seen;
    bool align_skip_logged;
    AppLaserFix laser;
} AppOdomTestState;

typedef struct {
    float vx;
    float vy;
    unsigned int mask;
} RouteEscape;

static AppOdomTestState s_odom_test;

static bool route_finite_vec3(Vector3 value) {
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static float route_clampf(float value, float limit) {
    return fmaxf(-limit, fminf(limit, value));
}

void app_odom_test_reset(void) {
    s_odom_test = (AppOdomTestState){ 0 };
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

static bool route_laser_valid(const LaserOdomSample* sample, LaserBeam beam, uint32_t now) {
    if(sample->received == 0u || beam == LASER_NO_BEAM ||
       (unsigned int)beam >= LASER_BEAM_COUNT || now - sample->stamp_ms > LASER_FRAME_MAX_AGE_MS) {
        return false;
    }
    const uint16_t raw = sample->distance_mm[beam];
    return raw >= LASER_MIN_RANGE_MM && raw <= LASER_MAX_RANGE_MM;
}

static bool route_furrow_gate(const LaserOdomSample* sample, uint32_t now) {
    if(!route_laser_valid(sample, LASER_FRONT, now) ||
       !route_laser_valid(sample, LASER_REAR, now)) return false;
    const float sum = (float)sample->distance_mm[LASER_FRONT] +
                      (float)sample->distance_mm[LASER_REAR];
    return sum >= LASER_FURROW_SUM_MIN_MM && sum <= LASER_FURROW_SUM_MAX_MM;
}

static float route_lateral_target(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                  bool test_vy, uint32_t now) {
    const LaserBeam beam = leg->control_beam;
    s_odom_test.laser.track_mask = 0u;
    s_odom_test.laser.mask = 0u;
    if(MAP_ROUTE_LASER_GUIDANCE_ENABLED == 0 || beam == LASER_NO_BEAM ||
       beam == LASER_REAR || !route_laser_valid(sample, beam, now)) return 0.0f;
    if(leg->furrow_gate && !route_furrow_gate(sample, now)) return 0.0f;

    float error_mm = (float)sample->distance_mm[beam] - LASER_TARGET_MM;
    if(fabsf(error_mm) <= LASER_CONTROL_DEADBAND_MM) error_mm = 0.0f;
    else error_mm -= copysignf(LASER_CONTROL_DEADBAND_MM, error_mm);

    /* FRONT=-vx, LEFT=-vy, RIGHT=+vy. Positive range error approaches the
     * wall and negative range error moves away; REAR is never a target beam. */
    const float beam_sign = beam == LASER_RIGHT ? 1.0f : -1.0f;
    const float target = beam_sign * LASER_CONTROL_GAIN * error_mm * 0.001f;
    s_odom_test.laser.track_mask = 1u << beam;
    s_odom_test.laser.mask = test_vy ? APP_LASER_X : APP_LASER_Y;
    return route_clampf(target, LASER_CORRECTION_LIMIT_MPS);
}

static float route_slew_correction(float target, uint32_t now) {
    const float dt = fminf(APP_ODOM_TEST_CONTROL_PERIOD_S * 10.0f,
                           (float)(now - s_odom_test.last_control_ms) * 0.001f);
    s_odom_test.last_control_ms = now;
    if(target == 0.0f || target * s_odom_test.correction_mps < 0.0f) return 0.0f;
    const float step = LASER_CORRECTION_ACCEL_MPS2 * dt;
    const float previous = fabsf(s_odom_test.correction_mps);
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

static float route_escape_speed(uint16_t raw) {
    const float error_m = (LASER_SAFETY_MM - (float)raw) * 0.001f;
    return fminf(LASER_ESCAPE_LIMIT_MPS, LASER_ESCAPE_GAIN * error_m);
}

/* Safety escape is independent of the 65 mm target controller. It can cancel
 * motion toward a wall and always remains active, including while settling. */
static RouteEscape route_safety_escape(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                       uint32_t now, float vx, float vy) {
    const bool front = route_laser_valid(sample, LASER_FRONT, now) &&
                       sample->distance_mm[LASER_FRONT] < LASER_SAFETY_MM;
    const bool left = route_laser_valid(sample, LASER_LEFT, now) &&
                      sample->distance_mm[LASER_LEFT] < LASER_SAFETY_MM;
    const bool right = route_laser_valid(sample, LASER_RIGHT, now) &&
                       sample->distance_mm[LASER_RIGHT] < LASER_SAFETY_MM;
    const bool rear = leg->furrow_gate && route_laser_valid(sample, LASER_REAR, now) &&
                      sample->distance_mm[LASER_REAR] < LASER_SAFETY_MM;
    RouteEscape escape = { vx, vy, 0u };

    /* Cancel every unsafe base-motion component before adding escape speeds. */
    if((front && escape.vx < 0.0f) || (rear && escape.vx > 0.0f)) escape.vx = 0.0f;
    if((left && escape.vy < 0.0f) || (right && escape.vy > 0.0f)) escape.vy = 0.0f;

    /* Opposing dangers on one axis hold that axis at zero instead of letting
     * the later beam overwrite the earlier escape command. */
    if(front != rear) {
        if(front) {
            escape.vx += route_escape_speed(sample->distance_mm[LASER_FRONT]);
            escape.mask |= 1u << LASER_FRONT;
        }
        else {
            escape.vx -= route_escape_speed(sample->distance_mm[LASER_REAR]);
            escape.mask |= 1u << LASER_REAR;
        }
    }
    else if(front) {
        escape.vx = 0.0f;
        escape.mask |= 1u << LASER_FRONT | 1u << LASER_REAR;
    }
    if(left != right) {
        if(left) {
            escape.vy += route_escape_speed(sample->distance_mm[LASER_LEFT]);
            escape.mask |= 1u << LASER_LEFT;
        }
        else {
            escape.vy -= route_escape_speed(sample->distance_mm[LASER_RIGHT]);
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
                                           uint32_t now, float yaw, float vx_route, float vy_route,
                                           bool* safety_active) {
    const RouteEscape escape = route_safety_escape(leg, sample, now, vx_route, vy_route);
    s_odom_test.laser.jump_mask = escape.mask;
    *safety_active = escape.mask != 0u;

    const float c = cosf(yaw) * s_odom_test.heading_cos + sinf(yaw) * s_odom_test.heading_sin;
    const float s = sinf(yaw) * s_odom_test.heading_cos - cosf(yaw) * s_odom_test.heading_sin;
    const float vx_cmd = c * escape.vx + s * escape.vy;
    const float vy_cmd = c * escape.vy - s * escape.vx;
    /* Auto is translation-only under every condition: wz is always zero. */
    if(chassis.set_velocity(vx_cmd, vy_cmd, 0.0f) != CHASSIS_OK) {
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
    s_odom_test.phase = ROUTE_TRAVEL;
    s_odom_test.speed_phase = ROUTE_SPEED_STOP;
    s_odom_test.accel_distance_m = fminf(ROUTE_ACCEL_DISTANCE_M, 0.5f * distance_m);
    s_odom_test.decel_distance_m = fminf(ROUTE_DECEL_DISTANCE_M, 0.5f * distance_m);
    s_odom_test.accel_mps2 = leg->speed_mps * leg->speed_mps /
                             (2.0f * s_odom_test.accel_distance_m);
    s_odom_test.decel_mps2 = leg->speed_mps * leg->speed_mps /
                             (2.0f * s_odom_test.decel_distance_m);
    s_odom_test.travel_ms = s_odom_test.progress_ms = now;
    s_odom_test.last_control_ms = now;
    s_odom_test.last_progress_position_m = -INFINITY;
    s_odom_test.settle_ready_since_ms = 0u;
    s_odom_test.laser_waiting = false;
    s_odom_test.following_laser = false;
    s_odom_test.final_approach = false;
    s_odom_test.alignment_seen = false;
    s_odom_test.align_skip_logged = false;
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

static void route_begin_settle(uint32_t now) {
    s_odom_test.phase = ROUTE_SETTLE;
    s_odom_test.settle_since_ms = now;
    s_odom_test.settle_ready_since_ms = 0u;
    s_odom_test.align_log_ms = now;
    s_odom_test.route_speed_mps = 0.0f;
    s_odom_test.correction_mps = 0.0f;
    s_odom_test.alignment_seen = false;
    s_odom_test.align_skip_logged = false;
    s_odom_test.following_laser = false;
    route_log_speed_phase(ROUTE_SPEED_STOP, 0.0f, now);
    s_odom_test.laser.anchor_mask = s_odom_test.segment_index == 0u ? 1u << LASER_FRONT : 0u;
    log_info("MAP SETTLE %u xy=(%.3f,%.3f)", (unsigned int)s_odom_test.segment_index + 1u,
             (double)s_odom_test.laser.x, (double)s_odom_test.laser.y);
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

static AppControlResult route_settle(const MapRouteLeg* leg, const LaserOdomSample* sample,
                                     bool test_vy, Vector3 velocity, float yaw, uint32_t now) {
    const LaserBeam beam = leg->control_beam;
    const bool gate_open = !leg->furrow_gate || route_furrow_gate(sample, now);
    const bool alignment_required = MAP_ROUTE_LASER_GUIDANCE_ENABLED != 0 &&
                                    beam != LASER_NO_BEAM && beam != LASER_REAR && gate_open;
    const bool alignment_valid = alignment_required && route_laser_valid(sample, beam, now);
    if(alignment_valid) s_odom_test.alignment_seen = true;

    /* Wheel odometry owns the waypoint and segment switch. A missing lateral
     * observation skips alignment with a warning instead of deadlocking the
     * route; once seen, a lost observation cannot count as aligned. */
    const bool aligned = !alignment_required ||
        (alignment_valid ? fabsf((float)sample->distance_mm[beam] - LASER_TARGET_MM) <=
                               LASER_TARGET_TOLERANCE_MM : !s_odom_test.alignment_seen);
    if(alignment_required && !alignment_valid && !s_odom_test.alignment_seen &&
       !s_odom_test.align_skip_logged) {
        s_odom_test.align_skip_logged = true;
        log_warn("MAP ALIGN SKIP seg=%u beam=%u: no fresh wall, use wheel waypoint",
                 (unsigned int)s_odom_test.segment_index + 1u, (unsigned int)beam);
    }

    const float correction_target = route_lateral_target(leg, sample, test_vy, now);
    s_odom_test.correction_mps = route_slew_correction(correction_target, now);
    if(s_odom_test.laser.mask == 0u) s_odom_test.correction_mps = 0.0f;
    s_odom_test.following_laser = s_odom_test.laser.mask != 0u;
    if(now - s_odom_test.align_log_ms >= ROUTE_ALIGN_LOG_PERIOD_MS) {
        s_odom_test.align_log_ms = now;
        if(alignment_valid) {
            log_info("MAP ALIGN seg=%u beam=%u raw=%u target=65 corr=%.3f",
                     (unsigned int)s_odom_test.segment_index + 1u, (unsigned int)beam,
                     (unsigned int)sample->distance_mm[beam], (double)s_odom_test.correction_mps);
        }
        else if(alignment_required && s_odom_test.alignment_seen) {
            log_warn("MAP ALIGN WAIT seg=%u beam=%u: waiting for fresh frame",
                     (unsigned int)s_odom_test.segment_index + 1u, (unsigned int)beam);
        }
    }
    const float vx_route = test_vy ? s_odom_test.correction_mps : 0.0f;
    const float vy_route = test_vy ? 0.0f : s_odom_test.correction_mps;

    bool safety_active = false;
    const AppControlResult command = route_send_command(leg, sample, now, yaw,
                                                        vx_route, vy_route, &safety_active);
    if(command != APP_CONTROL_RESULT_OK) return command;

    const bool stationary = hypotf(velocity.x, velocity.y) <= APP_ODOM_TEST_STOP_SPEED_MPS;
    if(aligned && stationary && !safety_active) {
        if(s_odom_test.settle_ready_since_ms == 0u) s_odom_test.settle_ready_since_ms = now;
        if(now - s_odom_test.settle_ready_since_ms >= APP_ODOM_TEST_SETTLE_READY_MS) {
            return route_next_or_park(now);
        }
    }
    else {
        s_odom_test.settle_ready_since_ms = 0u;
    }

    /* No alignment timeout: a valid distant wall is corrected for as long as
     * needed. A lost frame holds motion and waits for recovery without fault. */
    return APP_CONTROL_RESULT_OK;
}

AppControlResult app_odom_test_process(void) {
    const uint32_t now = delay_now_ms();
    if(s_odom_test.phase == ROUTE_PARKED) return app_control_hold_zero_chassis();

    Vector3 position, angle, velocity;
    if(odom.get_odom(&position) != ODOM_OK || odom.get_angle(&angle) != ODOM_OK ||
       odom.get_velocity(&velocity) != ODOM_OK || !route_finite_vec3(position) ||
       !isfinite(angle.z) || !route_finite_vec3(velocity)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }

    const size_t index = s_odom_test.segment_index;
    if(index >= map_route_count) return APP_CONTROL_RESULT_COMMAND_INVALID;
    const MapRouteLeg* leg = &map_route[index];
    const float from_x = index ? map_route[index - 1u].x_m : MAP_START_X_M;
    const float from_y = index ? map_route[index - 1u].y_m : MAP_START_Y_M;
    const bool test_vy = leg->x_m != from_x;
    const float delta = test_vy ? leg->x_m - from_x : leg->y_m - from_y;
    const float distance_m = fabsf(delta);
    const float sign = copysignf(1.0f, delta);
    const LaserOdomSample* sample = laser_odom_get_sample();

    if(!isfinite(delta) || delta == 0.0f || (test_vy && leg->y_m != from_y) ||
       !isfinite(leg->speed_mps) || leg->speed_mps <= 0.0f || leg->control_beam == LASER_REAR) {
        return APP_CONTROL_RESULT_COMMAND_INVALID;
    }
    if(s_odom_test.phase == ROUTE_START_LEG) route_start_leg(leg, distance_m, position, angle.z, now);

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

    if(s_odom_test.phase == ROUTE_SETTLE) {
        return route_settle(leg, sample, test_vy, velocity, angle.z, now);
    }

    /* 01 alone uses a fresh front range for immediate endpoint acquisition.
     * All other legs use only wheel odometry; there is no laser minimum speed,
     * laser remaining-distance takeover, multi-frame vote or overshoot error. */
    bool endpoint_ready = false;
    if(index == 0u) {
        const float endpoint_window = fminf(ROUTE_LASER01_WINDOW_M, 0.45f * distance_m);
        endpoint_ready = remaining <= endpoint_window &&
                         route_laser_valid(sample, LASER_FRONT, now) &&
                         sample->distance_mm[LASER_FRONT] <=
                             LASER_TARGET_MM + LASER_TARGET_TOLERANCE_MM;
    }
    else {
        endpoint_ready = remaining <= ROUTE_END_TOLERANCE_M;
    }
    if(endpoint_ready) {
        route_begin_settle(now);
        return route_settle(leg, sample, test_vy, velocity, angle.z, now);
    }

    const float correction_target = route_lateral_target(leg, sample, test_vy, now);
    s_odom_test.correction_mps = route_slew_correction(correction_target, now);
    if(s_odom_test.laser.mask == 0u) s_odom_test.correction_mps = 0.0f;
    s_odom_test.following_laser = s_odom_test.laser.mask != 0u;

    const bool front01_valid = index == 0u && route_laser_valid(sample, LASER_FRONT, now);
    const bool final_approach = front01_valid && remaining <= ROUTE_END_TOLERANCE_M &&
                                sample->distance_mm[LASER_FRONT] >
                                    LASER_TARGET_MM + LASER_TARGET_TOLERANCE_MM;
    const bool wait_laser = MAP_ROUTE_LASER_GUIDANCE_ENABLED != 0 &&
                            (index == 0u || leg->control_beam != LASER_NO_BEAM) &&
                            (!laser_odom_is_online() ||
                             (index == 0u && remaining <= ROUTE_END_TOLERANCE_M && !front01_valid));
    if(wait_laser != s_odom_test.laser_waiting) {
        log_info("LASER %s seg=%u", wait_laser ? "WAIT" : "RESUME", (unsigned int)index + 1u);
        s_odom_test.laser_waiting = wait_laser;
    }
    if(final_approach != s_odom_test.final_approach) {
        s_odom_test.final_approach = final_approach;
        if(final_approach) {
            log_info("MAP FINAL APPROACH seg=1 F=%u v=0.040",
                     (unsigned int)sample->distance_mm[LASER_FRONT]);
        }
    }

    /* Position-based trapezoid: acceleration uses 400 mm from the leg start,
     * braking uses 400 mm to the wheel endpoint. A 700 mm connector uses two
     * 350 mm ramps because two 400 mm zero-end ramps cannot physically fit. */
    const float accel_speed = route_acceleration_speed(progress_m);
    const float braking_speed = route_braking_speed(remaining);
    float speed_limit = fminf(leg->speed_mps, fminf(accel_speed, braking_speed));
    /* Wheel placement can stop slightly before the front wall acquires 73 mm.
     * Continue only leg 01 at a bounded 0.04 m/s; the 55 mm escape still cuts
     * forward motion. This replaces the former no-progress yellow fault. */
    if(final_approach) speed_limit = ROUTE01_FINAL_APPROACH_MPS;
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
    if(wait_laser) s_odom_test.route_speed_mps = s_odom_test.correction_mps = 0.0f;

    /* Field +X=+vy and field +Y=-vx. */
    const float drive = (test_vy ? sign : -sign) * s_odom_test.route_speed_mps;
    const float vx_route = test_vy ? s_odom_test.correction_mps : drive;
    const float vy_route = test_vy ? drive : s_odom_test.correction_mps;
    bool safety_active = false;
    const AppControlResult command = route_send_command(leg, sample, now, angle.z,
                                                        vx_route, vy_route, &safety_active);
    if(command != APP_CONTROL_RESULT_OK) return command;

    if(wait_laser || safety_active || sign * along >= s_odom_test.last_progress_position_m + 0.008f) {
        s_odom_test.last_progress_position_m = sign * along;
        s_odom_test.progress_ms = now;
    }
    else if(now - s_odom_test.progress_ms > 2000u) {
        log_warn("MAP no progress seg=%u rem=%.3f", (unsigned int)index + 1u, (double)remaining);
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    s_odom_test.laser.correction_mps = s_odom_test.correction_mps;
    return APP_CONTROL_RESULT_OK;
}
