#ifndef _app_odom_test_h_
#define _app_odom_test_h_

#include "app_control.h"

/* Reset on runtime initialization or leaving active control. */
void app_odom_test_reset(void);

/* Remaining distance of the current segment, for telemetry. */
float app_odom_test_remaining_m(void);

/* Run the route once in Auto after motor preparation, then hold zero speed. */
AppControlResult app_odom_test_process(void);

#endif
