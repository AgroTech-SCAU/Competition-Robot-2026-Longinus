#ifndef _app_odom_test_h_
#define _app_odom_test_h_

#include "app_control.h"

enum { APP_LASER_X = 1u, APP_LASER_Y = 2u };

typedef struct {
    float x;
    float y;
    unsigned int mask;
    unsigned int segment;
} AppLaserFix;

AppLaserFix app_odom_test_get_laser_fix(void);

/* Reset on runtime initialization or leaving active control. */
void app_odom_test_reset(void);

/* Remaining distance of the current segment, for telemetry. */
float app_odom_test_remaining_m(void);

/* Run the route once in Auto after motor preparation, then hold zero speed. */
AppControlResult app_odom_test_process(void);

#endif
