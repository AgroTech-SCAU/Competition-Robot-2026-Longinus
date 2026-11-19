#ifndef _service_laser_field_h_
#define _service_laser_field_h_

#include "laser_odom.h"

/* Near-range floor for the Auto adapter, in millimetres. */
#define LASER_FIELD_MIN_RANGE_MM 30u

typedef struct {
    float x_m;
    float y_m;
    uint32_t quality;
    int furrow;
} LaserFieldEstimate;

void laser_field_reset(void);
const LaserOdomSample* laser_field_filter(const LaserOdomSample* sample);
bool laser_field_get_estimate(LaserFieldEstimate* estimate);

#endif
