#ifndef MAP_ROUTE_H
#define MAP_ROUTE_H

#include "laser_odom.h"
#include <stddef.h>

/* Right-handed base_link: +vx is the actual front, +vy is the physical left.
 * The vehicle reverses into the field: field +Y = -vx, field +X = +vy.
 * CAN distance 1..4 remain LASER_FRONT/REAR/LEFT/RIGHT. */
#define MAP_START_X_M 0.20f
#define MAP_START_Y_M (-0.50f)
#define MAP_ROUTE_LASER_GUIDANCE_ENABLED 1

typedef struct {
    float x_m;
    float y_m;
    float speed_mps;
    LaserBeam control_beam; /* Bidirectional 65 mm lateral target. */
    bool furrow_gate;       /* Front+rear=100..140 mm; also enables rear escape. */
} MapRouteLeg;

extern const MapRouteLeg map_route[];
extern const size_t map_route_count;

/* Wheel travel is relative to the marked launch position. */
float map_route_wheel_target(bool along_x, float coordinate);
float map_route_coordinate(bool along_x, float corrected_travel);

#endif
