#ifndef _app_odom_test_h_
#define _app_odom_test_h_

#include "app_control.h"

/* Select one autonomous test axis; remote control behavior is unchanged. */
#define APP_ODOM_TEST_AXIS_VX 1u
#define APP_ODOM_TEST_AXIS_VY 0u
#define APP_ODOM_TEST_AXIS APP_ODOM_TEST_AXIS_VX

/* X-axis test: distance > 0 (m); speed > 0 forward, speed < 0 backward (m/s). */
#define APP_ODOM_TEST_DISTANCE_M 0.72f  //进场vx=0.32f
#define APP_ODOM_TEST_SPEED_MPS 0.50f

/* Y-axis test: distance > 0 (m); speed > 0 leftward, speed < 0 rightward (m/s). */
#define APP_ODOM_TEST_VY_DISTANCE_M 2.65f
#define APP_ODOM_TEST_VY_SPEED_MPS -0.30f

/* Reset on runtime initialization or leaving active control. */
void app_odom_test_reset(void);

/* Called only in Auto after motor preparation; completion holds zero speed. */
AppControlResult app_odom_test_process(void);

#endif
