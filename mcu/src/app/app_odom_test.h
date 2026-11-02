#ifndef _app_odom_test_h_
#define _app_odom_test_h_

#include "app_control.h"

/* Autonomous odometry test parameters: forward distance (m), speed (m/s). */
#define APP_ODOM_TEST_DISTANCE_M 2.20f
#define APP_ODOM_TEST_SPEED_MPS 0.20f

/* Reset on runtime initialization or leaving active control. */
void app_odom_test_reset(void);

/* Called only in Auto after motor preparation; completion holds zero speed. */
AppControlResult app_odom_test_process(void);

#endif
