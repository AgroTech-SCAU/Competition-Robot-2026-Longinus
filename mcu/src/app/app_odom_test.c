#include "app_odom_test.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "log.h"
#include "odom.h"

#include <math.h>

#define APP_ODOM_TEST_CONTROL_PERIOD_S 0.002f

static struct {
    bool started;
    bool done;
    Vector3 origin;
    float heading_cos;
    float heading_sin;
    float progress_m;
} s_odom_test;

void app_odom_test_reset(void) {
    s_odom_test.started = false;
    s_odom_test.done = false;
    s_odom_test.progress_m = 0.0f;
}

AppControlResult app_odom_test_process(void) {
    Vector3 position = { 0 };
    Vector3 angle = { 0 };
    Vector3 gyro = { 0 };
    const bool test_vy = APP_ODOM_TEST_AXIS == APP_ODOM_TEST_AXIS_VY;
    const float distance_m = test_vy ? APP_ODOM_TEST_VY_DISTANCE_M : APP_ODOM_TEST_DISTANCE_M;
    const float speed_mps = test_vy ? APP_ODOM_TEST_VY_SPEED_MPS : APP_ODOM_TEST_SPEED_MPS;
    const float vx = test_vy ? 0.0f : speed_mps;
    const float vy = test_vy ? speed_mps : 0.0f;
    float wz;

    if((APP_ODOM_TEST_AXIS != APP_ODOM_TEST_AXIS_VX && APP_ODOM_TEST_AXIS != APP_ODOM_TEST_AXIS_VY) ||
       !isfinite(distance_m) || distance_m <= 0.0f ||
       !isfinite(speed_mps) || speed_mps == 0.0f) {
        return APP_CONTROL_RESULT_COMMAND_INVALID;
    }
    if(s_odom_test.done) {
        return app_control_hold_zero_chassis();
    }
    if(odom.get_odom(&position) != ODOM_OK || odom.get_angle(&angle) != ODOM_OK ||
       odom.get_gyro_corrected(&gyro) != ODOM_OK ||
       !isfinite(position.x) || !isfinite(position.y) ||
       !isfinite(angle.z) || !isfinite(gyro.z)) {
        return APP_CONTROL_RESULT_ODOM_ERROR;
    }

    if(!s_odom_test.started) {
        s_odom_test.origin = position;
        s_odom_test.heading_cos = cosf(angle.z);
        s_odom_test.heading_sin = sinf(angle.z);
        chassis_yaw_hold_set_target(angle.z);
        s_odom_test.started = true;
        log_info("ODOM TEST start: d=%.2fm vx=%.2fm/s vy=%.2fm/s",
                 (double)distance_m, (double)vx, (double)vy);
    }

    /* Project world-frame displacement onto the selected starting body axis. */
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
            s_odom_test.done = true;
            log_info("ODOM TEST done: s=%.3fm", (double)s_odom_test.progress_m);
        }
        return result;
    }

    wz = chassis_yaw_hold_apply(vx, vy, 0.0f,
                                angle.z, gyro.z, APP_ODOM_TEST_CONTROL_PERIOD_S);
    return chassis.set_velocity(vx, vy, wz) == CHASSIS_OK
        ? APP_CONTROL_RESULT_OK : APP_CONTROL_RESULT_CHASSIS_ERROR;
}
