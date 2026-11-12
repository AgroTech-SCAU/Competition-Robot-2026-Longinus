#include "laser_odom.h"

#include "delay.h"

#define LASER_CAN_TIMEOUT_MS 250u

static LaserOdomSample s_laser_sample;

void laser_odom_receive(const LaserOdomSample* sample) {
    s_laser_sample = *sample;
    s_laser_sample.valid_mask = 0u;
    for(unsigned int beam = 0; beam < LASER_BEAM_COUNT; ++beam) {
        const uint16_t distance = sample->distance_mm[beam];
        if(distance >= LASER_MIN_RANGE_MM && distance <= LASER_MAX_RANGE_MM) {
            s_laser_sample.valid_mask |= (uint8_t)(1u << beam);
        }
    }
}

const LaserOdomSample* laser_odom_get_sample(void) {
    return &s_laser_sample;
}

bool laser_odom_is_online(void) {
    return s_laser_sample.received != 0u &&
           (uint32_t)(delay_now_ms() - s_laser_sample.stamp_ms) <= LASER_CAN_TIMEOUT_MS;
}

float laser_odom_axis_position(LaserBeam beam, float waypoint_m,
                              float reference_mm, float measured_mm) {
    const float difference_m = (reference_mm - measured_mm) * 0.001f;
    const bool points_positive = beam == LASER_FRONT || beam == LASER_LEFT;
    return waypoint_m + (points_positive ? difference_m : -difference_m);
}
