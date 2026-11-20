#include "map_route.h"

#define WEST 0.20f
#define EAST 2.80f
#define ROW(n) (0.20f + 0.70f * ((n) - 1))
#define ROUTE_START_SPEED_MPS 0.50f
#define ROUTE_CROSS_SPEED_MPS 0.45f
#define ROUTE_CONNECT_SPEED_MPS 0.32f
#define ROUTE_RETURN_SPEED_MPS 0.45f

#define CROSS(x, row, rear_escape, work_stop, endpoint_trim) \
    { x, ROW(row), ROUTE_CROSS_SPEED_MPS, LASER_FRONT, true, rear_escape, work_stop, endpoint_trim }
#define CONNECT(x, row, side) \
    { x, ROW(row), ROUTE_CONNECT_SPEED_MPS, side, false, false, false, false }

/* Every crossing acquires front+rear centering inside the 95..125 mm furrow gate.
 * Connectors use wheel distance along +vx and the corresponding side beam.
 * 02 and 03 pass through furrow 1. The return pass 03 is transit only. */
const MapRouteLeg map_route[] = {
    { WEST, ROW(7), ROUTE_START_SPEED_MPS, LASER_LEFT,
      false, false, false, false },                               /* 01 */
    CROSS(EAST, 7, false, true, false),                            /* 02 */
    CROSS(WEST, 7, false, false, true),                            /* 03: return through furrow 1 */
    CONNECT(WEST, 6, LASER_LEFT),                                 /* 04 */
    CROSS(EAST, 6, true, true, true),                             /* 05 */
    CONNECT(EAST, 5, LASER_RIGHT),                                /* 06 */
    CROSS(WEST, 5, true, true, true),                             /* 07 */
    CONNECT(WEST, 4, LASER_LEFT),                                 /* 08 */
    CROSS(EAST, 4, true, true, true),                             /* 09 */
    CONNECT(EAST, 3, LASER_RIGHT),                                /* 10 */
    CROSS(WEST, 3, true, true, true),                             /* 11 */
    CONNECT(WEST, 2, LASER_LEFT),                                 /* 12 */
    CROSS(EAST, 2, true, true, true),                             /* 13 */
    CONNECT(EAST, 1, LASER_RIGHT),                                /* 14 */
    CROSS(WEST, 1, true, true, true),                             /* 15 */
    { WEST, MAP_START_Y_M, ROUTE_RETURN_SPEED_MPS,
      LASER_NO_BEAM, false, false, false, false },                /* 16: wheel only */
};
const size_t map_route_count = sizeof(map_route) / sizeof(map_route[0]);

float map_route_wheel_target(bool along_x, float coordinate) {
    return along_x ? coordinate - MAP_START_X_M : coordinate - MAP_START_Y_M;
}

float map_route_coordinate(bool along_x, float corrected_travel) {
    return (along_x ? MAP_START_X_M : MAP_START_Y_M) + corrected_travel;
}
