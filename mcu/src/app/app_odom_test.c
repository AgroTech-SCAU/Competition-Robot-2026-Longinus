#include "app_odom_test.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "log.h"
#include "odom.h"

#include <math.h>
#include <stddef.h>

#define APP_ODOM_TEST_CONTROL_PERIOD_S 0.002f
#define APP_ODOM_TEST_STOP_SPEED_MPS 0.02f

typedef struct {
    float distance_m;
    float vx;
    float vy;
} AppOdomTestSegment;

/* Tune segments 01-14 here. Each row is { distance_m, vx_mps, vy_mps }.
 * Use a positive distance and tune only the nonzero speed in each row.
 * Keep the speed sign to preserve direction (+X forward, +Y left).
 */
static const AppOdomTestSegment s_route[] = {
    { 4.773f,  -1.0f,  0.0f }, /* 01: enter */
    { 2.66f, 0.0f,   0.80f }, /* 02: right */
    { 0.713f, 0.5f,  0.0f }, /* 03: forward */
    { 2.66f,  0.0f,  -0.80f }, /* 04: left */
    { 0.72f,  0.5f,  0.0f }, /* 05: forward */
    { 2.66f,  0.0f,   0.80f }, /* 06: right */
    { 0.72f,  0.5f,  0.0f }, /* 07: forward */
    { 2.66f,  0.0f,  0.80f }, /* 08: left */
    { 0.72f,  0.5f,  0.0f }, /* 09: forward */
    { 2.66f,  0.0f,  0.80f }, /* 10: left */
    { 0.72f,  0.5f,  0.0f }, /* 11: forward */
    { 2.66f,  0.0f,   -0.80f }, /* 12: right */
    { 0.72f,   0.5f,  0.0f }, /* 13: return */
    { 2.55f,  0.0f,   0.80f }, /* 14: right */
    { 2.66f,   0.0f,  -0.8f }, /* 15: fixed exit */
};

#define APP_ODOM_TEST_ROUTE_COUNT (sizeof(s_route) / sizeof(s_route[0]))

static struct {
    size_t segment_index;
    bool segment_started;
    Vector3 origin;
    float heading_cos;
    float heading_sin;
    float progress_m;
} s_odom_test;

void app_odom_test_reset(void) {
    s_odom_test.segment_index = 0;
    s_odom_test.segment_started = false;
    s_odom_test.progress_m = 0.0f;
}

float app_odom_test_remaining_m(void) {
    if(s_odom_test.segment_index >= APP_ODOM_TEST_ROUTE_COUNT) {
        return 0.0f;
    }
    const float distance_m = s_route[s_odom_test.segment_index].distance_m;
    return s_odom_test.segment_started ? fmaxf(0.0f, distance_m - s_odom_test.progress_m) : distance_m;
}

AppControlResult app_odom_test_process(void) {
    Vector3 position = { 0 };
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };
    float wz;

    if(s_odom_test.segment_index >= APP_ODOM_TEST_ROUTE_COUNT) {
        return app_control_hold_zero_chassis();
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
        if(s_odom_test.segment_index == 0) {
            s_odom_test.heading_cos = cosf(angle.z);
            s_odom_test.heading_sin = sinf(angle.z);
            chassis_yaw_hold_set_target(angle.z);
        }
        else {
            Vector3 velocity = { 0 };
            if(odom.get_velocity(&velocity) != ODOM_OK ||
               !isfinite(velocity.x) || !isfinite(velocity.y)) {
                return APP_CONTROL_RESULT_ODOM_ERROR;
            }
            if(hypotf(velocity.x, velocity.y) > APP_ODOM_TEST_STOP_SPEED_MPS) {
                return app_control_hold_zero_chassis();
            }
        }
        s_odom_test.origin = position;
        s_odom_test.segment_started = true;
        log_info("ODOM ROUTE %u/%u start: d=%.2fm vx=%.2fm/s vy=%.2fm/s",
                 (unsigned int)(s_odom_test.segment_index + 1), (unsigned int)APP_ODOM_TEST_ROUTE_COUNT,
                 (double)distance_m, (double)vx, (double)vy);
    }

    /* Project this segment's displacement onto the route's initial body axes. */
    if(test_vy) {
        s_odom_test.progress_m = -(position.x - s_odom_test.origin.x) * s_odom_test.heading_sin +
                                 (position.y - s_odom_test.origin.y) * s_odom_test.heading_cos;
    }
    else {
        s_odom_test.progress_m = (position.x - s_odom_test.origin.x) * s_odom_test.heading_cos +
                                 (position.y - s_odom_test.origin.y) * s_odom_test.heading_sin;
    }
    /* Count travel along the commanded direction; opposite motion is negative. */
    s_odom_test.progress_m *= speed_mps > 0.0f ? 1.0f : -1.0f;
    if(!isfinite(s_odom_test.progress_m)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }
    if(s_odom_test.progress_m >= distance_m) {
        AppControlResult result = app_control_hold_zero_chassis();
        if(result == APP_CONTROL_RESULT_OK) {
            ++s_odom_test.segment_index;
            s_odom_test.segment_started = false;
            log_info("ODOM ROUTE %u/%u done: s=%.3fm",
                     (unsigned int)s_odom_test.segment_index, (unsigned int)APP_ODOM_TEST_ROUTE_COUNT,
                     (double)s_odom_test.progress_m);
        }
        return result;
    }

    wz = chassis_yaw_hold_apply(vx, vy, 0.0f,
                                angle.z, gyro.z, APP_ODOM_TEST_CONTROL_PERIOD_S);
    return chassis.set_velocity(vx, vy, wz) == CHASSIS_OK
        ? APP_CONTROL_RESULT_OK : APP_CONTROL_RESULT_CHASSIS_ERROR;
}
