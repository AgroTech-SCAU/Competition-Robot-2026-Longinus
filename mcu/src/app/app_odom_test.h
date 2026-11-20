#ifndef _app_odom_test_h_
#define _app_odom_test_h_

#include "app_control.h"
#include <stdbool.h>
#include <stdint.h>

enum { APP_LASER_X = 1u, APP_LASER_Y = 2u };

typedef struct {
    float x, y; /* Running position in the competition diagram, metres. */
    unsigned int mask;
    unsigned int segment;
    unsigned int track_mask;
    unsigned int jump_mask;
    unsigned int anchor_mask; /* Fresh confirmed endpoint fence, F/B/L/R bits. */
    float correction_mps;
} AppLaserFix;

AppLaserFix app_odom_test_get_laser_fix(void);

/* Reset on runtime initialization or leaving active control. */
void app_odom_test_reset(void);

/* Remaining distance of the current segment, for telemetry. */
float app_odom_test_remaining_m(void);

bool app_odom_test_is_done(void);

void app_odom_test_set_work_range(bool frame_received, uint16_t distance_mm, uint32_t stamp_ms);
bool app_odom_test_work_pump_enabled(void);
bool app_odom_test_entry_pump_enabled(void);
bool app_odom_test_entry_pa0_enabled(void);

/* Run the route once in Auto after motor preparation, then hold zero speed. */
AppControlResult app_odom_test_process(void);

#endif
