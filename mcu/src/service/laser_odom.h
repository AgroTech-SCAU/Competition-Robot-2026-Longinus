#ifndef _service_laser_odom_h_
#define _service_laser_odom_h_

#include <stdbool.h>
#include <stdint.h>

#define LASER_MIN_RANGE_MM 70u
#define LASER_MAX_RANGE_MM 7500u

typedef enum {
    LASER_FRONT,
    LASER_REAR,
    LASER_LEFT,
    LASER_RIGHT,
    LASER_BEAM_COUNT,
    LASER_NO_BEAM = LASER_BEAM_COUNT
} LaserBeam;

typedef struct {
    uint16_t distance_mm[LASER_BEAM_COUNT];
    uint32_t received;
    uint32_t stamp_ms;
    uint8_t valid_mask;
} LaserOdomSample;

void laser_odom_receive(const LaserOdomSample* sample);
const LaserOdomSample* laser_odom_get_sample(void);
bool laser_odom_is_online(void);

/* The wall coordinate is calibrated by the measured range at the waypoint.
   Front/left face the positive axes, rear/right face the negative axes. */
float laser_odom_axis_position(LaserBeam beam, float waypoint_m,
                              float reference_mm, float measured_mm);

#endif
