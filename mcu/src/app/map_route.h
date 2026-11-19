#ifndef MAP_ROUTE_H
#define MAP_ROUTE_H

#include "laser_odom.h"
#include <stddef.h>

/* Right-handed base_link: +vx is the actual front, +vy is the physical left.
 * The vehicle reverses into the field: field +Y = -vx, field +X = +vy.
 * LASER_REAR=+vx, LASER_FRONT=-vx, LASER_RIGHT=+vy, LASER_LEFT=-vy. */
#define MAP_START_X_M 0.20f
#define MAP_START_Y_M (-0.50f)
#define MAP_START_LEFT_SETBACK_MM 26.0f

/* Open-field direction validation: laser telemetry remains available, but no
 * laser reading may affect guidance, braking, readiness or segment progress. */
#define MAP_ROUTE_LASER_GUIDANCE_ENABLED 0

typedef struct {
    LaserBeam beam;
    float min_mm;
    float max_mm;
} AppLaserBand;

typedef struct {
    float x_m;
    float y_m;
    float speed_mps;
    AppLaserBand cruise[2]; /* Clearance from the corresponding body extremity. */
    LaserBeam end_beam;     /* Outside fence facing the direction of travel. */
} MapRouteLeg;

extern const MapRouteLeg map_route[];
extern const size_t map_route_count;

typedef struct {
    float scale;   /* Physical wheel metres per nominal field metre. */
    float bias_m;  /* Physical offset from subsequent fence observations. */
} MapRouteAxis;

typedef struct {
    uint32_t frame, first_ms, last_ms;
    float face_m, first_wheel_m, first_range_m;
    unsigned int samples;
    bool locked, frame_accepted;
} MapFenceTrack;

/* The origin of wheel travel is the marked launch position. Scale Y inside
 * the field only, so that the parking target stays 0.50 m outside the gate. */
float map_route_wheel_target(bool along_x, float coordinate, float scale);
float map_route_coordinate(bool along_x, float corrected_travel, float scale);
bool map_route_wall_visible(float lane_y, float field_x, LaserBeam beam);
/* Update only the approaching axis, on independent frames near a known fence.
 * Wheel travel and speed have already been transformed into field directions. */
bool map_route_observe_fence(MapFenceTrack* track, MapRouteAxis* axis,
                            const MapRouteLeg* leg, size_t index, bool along_x, float sign,
                            float wheel_travel, float field_speed, const LaserOdomSample* sample,
                            uint32_t now, float yaw_error);

#endif
