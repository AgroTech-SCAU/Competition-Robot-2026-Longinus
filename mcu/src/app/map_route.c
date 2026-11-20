#include "map_route.h"

#define WEST 0.20f
#define EAST 2.80f
#define ROW(n) (0.20f + 0.70f * ((n) - 1))
#define ROUTE_START_SPEED_MPS 0.50f
#define ROUTE_CROSS_SPEED_MPS 0.45f
#define ROUTE_CONNECT_SPEED_MPS 0.32f
#define ROUTE_RETURN_SPEED_MPS 0.45f

#define CROSS(x, row, rear_escape) \
    { x, ROW(row), ROUTE_CROSS_SPEED_MPS, LASER_FRONT, LASER_NO_BEAM, true, rear_escape }
#define CONNECT(x, row, side) \
    { x, ROW(row), ROUTE_CONNECT_SPEED_MPS, side, LASER_NO_BEAM, false, false }

/* Every crossing uses front+rear centering inside the 50..125 mm furrow gate.
 * Only leg 01 additionally settles FRONT=70 mm. Later +vx connectors keep wheel
 * X because FRONT looks back at the previous furrow and must not pull the car back.
 * Leg 15 instead requires LEFT=45 mm during its final settle before leg 16.
 * Leg 16 has no guidance or alignment and exits directly by wheel odometry. */
const MapRouteLeg map_route[] = {
    { WEST, ROW(7), ROUTE_START_SPEED_MPS, LASER_LEFT, LASER_NO_BEAM,
      false, false },                                             /* 01 */
    CROSS(EAST, 7, false),                                        /* 02 */
    CROSS(WEST, 7, false),                                        /* 03 */
    CONNECT(WEST, 6, LASER_LEFT),                                 /* 04 */
    CROSS(EAST, 6, true),                                         /* 05 */
    CONNECT(EAST, 5, LASER_RIGHT),                                /* 06 */
    CROSS(WEST, 5, true),                                         /* 07 */
    CONNECT(WEST, 4, LASER_LEFT),                                 /* 08 */
    CROSS(EAST, 4, true),                                         /* 09 */
    CONNECT(EAST, 3, LASER_RIGHT),                                /* 10 */
    CROSS(WEST, 3, true),                                         /* 11 */
    CONNECT(WEST, 2, LASER_LEFT),                                 /* 12 */
    CROSS(EAST, 2, true),                                         /* 13 */
    CONNECT(EAST, 1, LASER_RIGHT),                                /* 14 */
    { WEST, ROW(1), ROUTE_CROSS_SPEED_MPS, LASER_FRONT, LASER_LEFT,
      true, true },                                               /* 15: settle LEFT */
    { WEST, MAP_START_Y_M, ROUTE_RETURN_SPEED_MPS,
      LASER_NO_BEAM, LASER_NO_BEAM, false, false },               /* 16: wheel only */
};
const size_t map_route_count = sizeof(map_route) / sizeof(map_route[0]);

float map_route_wheel_target(bool along_x, float coordinate) {
    return along_x ? coordinate - MAP_START_X_M : coordinate - MAP_START_Y_M;
}

float map_route_coordinate(bool along_x, float corrected_travel) {
    return (along_x ? MAP_START_X_M : MAP_START_Y_M) + corrected_travel;
}
