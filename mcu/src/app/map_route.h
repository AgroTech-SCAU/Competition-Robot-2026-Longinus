#ifndef MAP_ROUTE_H
#define MAP_ROUTE_H

#include "laser_odom.h"
#include <stddef.h>

/* Right-handed base_link: +vx is the actual front, +vy is the physical left.
 * The vehicle reverses into the field: field +Y = -vx, field +X = +vy.
 * CAN distance 1..4 remain LASER_FRONT/REAR/LEFT/RIGHT. The CAN LEFT points
 * along -vy toward the vehicle's physical right and the map west wall. */
#define MAP_START_X_M 0.20f
#define MAP_START_Y_M (-0.50f)
#define MAP_ROUTE_LASER_GUIDANCE_ENABLED 1

typedef struct {
    float x_m;
    float y_m;
    float speed_mps;
    LaserBeam control_beam; /* Travel target; FRONT+REAR are combined when gated. */
    bool furrow_gate;       /* Front+rear=95..125 mm acquires dual-beam centering. */
    bool rear_escape;       /* Rear 30 mm protection only on 05/07/.../15. */
    bool work_stop;         /* Enable the mid-furrow laser work stop. */
    bool endpoint_side_trim; /* Require side-wall alignment before changing legs. */
    float wheel_extra_m;    /* Extra wheel travel before alignment at the same wall. */
} MapRouteLeg;

extern const MapRouteLeg map_route[];
extern const size_t map_route_count;

/* Wheel travel is relative to the marked launch position. */
float map_route_wheel_target(bool along_x, float coordinate);
float map_route_coordinate(bool along_x, float corrected_travel);

#endif
