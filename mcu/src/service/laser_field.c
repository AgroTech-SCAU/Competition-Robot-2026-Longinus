#include "laser_field.h"

/* Compile the original module here to isolate its init symbol from wheel odometry. */
#define odom_init field_odom_init
#include "../../vendor/odom_laser/odom_laser.c"
#undef odom_init

static odom_t s_field_odom;
static LaserOdomSample s_filtered_sample;
static bool s_configured;
static bool s_estimate_valid;

/* Vendor axes: F/B measure field +X/-X and L/R measure field +Y/-Y.
 * Vehicle: field +X=+vy=LASER_RIGHT, field +Y=-vx=LASER_FRONT. */
static const LaserBeam s_field_beams[LASER_BEAM_COUNT] = {
    LASER_RIGHT, LASER_LEFT, LASER_FRONT, LASER_REAR,
};

static bool laser_field_init(void) {
    odom_config_t config;
    odom_config_default(&config);

    /* Nominal map for telemetry. Use measured reflecting faces for absolute XY. */
    config.lx = 3.0f;
    config.af = config.ar = config.bl = config.br = 0.096f;
    config.min_range = LASER_FIELD_MIN_RANGE_MM / 1000.0f;
    config.max_range = LASER_MAX_RANGE_MM / 1000.0f;
    config.n_furrow = 7;
    for(int i = 0; i < config.n_furrow; ++i) {
        config.furrow_center[i] = 0.20f + 0.70f * (float)i;
        config.furrow_width[i] = 0.40f;
        config.x_left[i] = 0.0f;
        config.x_right[i] = config.lx;
    }
    return field_odom_init(&s_field_odom, &config);
}

void laser_field_reset(void) {
    s_configured = false;
    s_estimate_valid = false;
    s_filtered_sample.received = 0u;
}

const LaserOdomSample* laser_field_filter(const LaserOdomSample* sample) {
    if(sample->received == s_filtered_sample.received) return &s_filtered_sample;
    s_filtered_sample = *sample;
    s_filtered_sample.valid_mask = 0u;

    float range_m[LASER_BEAM_COUNT];
    uint8_t valid = 0u;
    for(unsigned int i = 0u; i < LASER_BEAM_COUNT; ++i) {
        const LaserBeam beam = s_field_beams[i];
        const uint16_t distance_mm = sample->distance_mm[beam];
        range_m[i] = (float)distance_mm / 1000.0f;
        if(distance_mm >= LASER_FIELD_MIN_RANGE_MM && distance_mm <= LASER_MAX_RANGE_MM) {
            valid |= (uint8_t)(1u << i);
            s_filtered_sample.valid_mask |= (uint8_t)(1u << beam);
        }
    }

    if(!s_configured) s_configured = laser_field_init();
    if(!s_configured) return &s_filtered_sample;
    s_estimate_valid = odom_update_ex(&s_field_odom, range_m[0], range_m[1], range_m[2], range_m[3],
                                     valid, sample->stamp_ms);
    if(s_estimate_valid) {
        s_filtered_sample.valid_mask = 0u;
        for(unsigned int i = 0u; i < LASER_BEAM_COUNT; ++i) {
            if(!(s_field_odom.last_valid & (1u << i))) continue;
            const LaserBeam beam = s_field_beams[i];
            s_filtered_sample.distance_mm[beam] = (uint16_t)(s_field_odom.last_d[i] * 1000.0f + 0.5f);
            s_filtered_sample.valid_mask |= (uint8_t)(1u << beam);
        }
    }
    return &s_filtered_sample;
}

bool laser_field_get_estimate(LaserFieldEstimate* estimate) {
    if(!s_estimate_valid) return false;
    const float* xy = odom_get_xy(&s_field_odom);
    *estimate = (LaserFieldEstimate){ xy[0], xy[1], odom_get_quality(&s_field_odom), odom_get_furrow(&s_field_odom) };
    return true;
}
