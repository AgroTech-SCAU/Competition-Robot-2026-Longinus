#include "map_route.h"
#include <math.h>

#define WEST 0.20f
#define EAST 2.80f
#define ROW(n) (0.20f + 0.70f * ((n) - 1))
/* Open-field wheel-odometry validation speeds. Raise only after the full
 * route direction and stopping distances have been verified on the vehicle. */
#define ROUTE_START_SPEED_MPS 0.35f
#define ROUTE_CROSS_SPEED_MPS 0.30f
#define ROUTE_CONNECT_SPEED_MPS 0.25f
#define ROUTE_RETURN_SPEED_MPS 0.30f
#define LEFT_WALL  { LASER_LEFT, 53 - MAP_START_LEFT_SETBACK_MM, 55 - MAP_START_LEFT_SETBACK_MM }
#define RIGHT_WALL { LASER_RIGHT, 47, 51 }
#define FRONT_WALL { LASER_FRONT, 49, 53 }
#define REAR_WALL  { LASER_REAR, 7, 11 }
#define NO_WALL    { .beam = LASER_NO_BEAM }
#define CROSS(x, row, end) { x, ROW(row), ROUTE_CROSS_SPEED_MPS, { FRONT_WALL, REAR_WALL }, end }
#define CONNECT(x, row, wall, end) { x, ROW(row), ROUTE_CONNECT_SPEED_MPS, { wall, NO_WALL }, end }

/* One row per direction change. The seventh (far) furrow is traversed twice
 * to return through the single gate after seven complete furrows. The short
 * ridge only widens the headlands; the same straight tracks clear its ends. */
const MapRouteLeg map_route[] = {
    { WEST, ROW(7), ROUTE_START_SPEED_MPS, { LEFT_WALL, NO_WALL }, LASER_FRONT },
    CROSS(EAST, 7, LASER_RIGHT),
    CROSS(WEST, 7, LASER_LEFT),
    CONNECT(WEST, 6, LEFT_WALL, LASER_NO_BEAM),
    CROSS(EAST, 6, LASER_RIGHT),
    CONNECT(EAST, 5, RIGHT_WALL, LASER_NO_BEAM),
    CROSS(WEST, 5, LASER_LEFT),
    CONNECT(WEST, 4, LEFT_WALL, LASER_NO_BEAM),
    CROSS(EAST, 4, LASER_RIGHT),
    CONNECT(EAST, 3, RIGHT_WALL, LASER_NO_BEAM),
    CROSS(WEST, 3, LASER_LEFT),
    CONNECT(WEST, 2, LEFT_WALL, LASER_NO_BEAM),
    CROSS(EAST, 2, LASER_RIGHT),
    CONNECT(EAST, 1, RIGHT_WALL, LASER_REAR),
    CROSS(WEST, 1, LASER_LEFT),
    { WEST, MAP_START_Y_M, ROUTE_RETURN_SPEED_MPS, { LEFT_WALL, NO_WALL }, LASER_NO_BEAM }
};
const size_t map_route_count = sizeof(map_route) / sizeof(map_route[0]);

float map_route_wheel_target(bool along_x, float coordinate, float scale) {
    if(along_x) return (coordinate - MAP_START_X_M) * scale;
    return coordinate * (coordinate > 0.0f ? scale : 1.0f) - MAP_START_Y_M;
}

float map_route_coordinate(bool along_x, float corrected_travel, float scale) {
    if(along_x) return MAP_START_X_M + corrected_travel / scale;
    const float y = MAP_START_Y_M + corrected_travel;
    return y > 0.0f ? y / scale : y;
}

bool map_route_wall_visible(float lane_y, float field_x, LaserBeam beam) {
    /* For a cross-field pass, the front beam looks at the northern neighbour
     * and the rear at the southern neighbour. Omit ambiguous ridge tips. */
    if(beam != LASER_FRONT && beam != LASER_REAR) return true;
    const int lane = (int)lroundf((lane_y - ROW(1)) / 0.70f);
    const int ridge = lane + (beam == LASER_FRONT ? 1 : 0);
    if(ridge == 7) return true;                 /* Solid northern perimeter. */
    if(ridge == 0) return field_x > 0.42f;      /* Opening in southern fence. */
    const float edge = ridge == 4 ? 0.62f : 0.42f;
    return field_x >= edge && field_x <= 3.0f - edge;
}

bool map_route_observe_fence(MapFenceTrack* track, MapRouteAxis* axis,
                            const MapRouteLeg* leg, size_t index, bool along_x, float sign,
                            float wheel_travel, float field_speed, const LaserOdomSample* sample,
                            uint32_t now, float yaw_error) {
    /* OLED readings at the chosen endpoints; these include the sensor mounting offsets. */
    static const float reference_mm[LASER_BEAM_COUNT] = { 67, 65, 54, 65 };
    const LaserBeam beam = leg->end_beam;
    const uint32_t age = now - sample->stamp_ms;
    if(beam == LASER_NO_BEAM || sample->received == 0u || age > 120u || fabsf(yaw_error) > 0.052f) {
        track->samples = 0u;
        track->frame_accepted = false;
        return false;
    }
    if(track->frame == sample->received) return track->frame_accepted;
    track->frame = sample->received;
    track->frame_accepted = false;
    const float goal = along_x ? leg->x_m : leg->y_m;
    const float at_sample = wheel_travel - field_speed * (0.001f * (float)age);
    const float remaining = sign * (map_route_wheel_target(along_x, goal, axis->scale) -
                                     (at_sample + axis->bias_m));
    const float raw = (float)sample->distance_mm[beam];
    const float laser_remaining = 0.001f * (raw - reference_mm[beam]);
    const float tolerance = along_x ? 0.18f : 0.26f; /* ±5% map plus placement. */
    if(raw < 30.0f || remaining > 0.85f || remaining < -0.04f ||
       fabsf(remaining - laser_remaining) > tolerance) {
        track->samples = 0u;
        return false;
    }

    /* A stationary fence predicts the same wheel-coordinate endpoint even as
     * the robot approaches. Reject range steps and stale/repeated frames. */
    const float face = at_sample + sign * laser_remaining;
    const bool continuous = track->samples && sample->stamp_ms - track->last_ms <= 120u &&
                            fabsf(face - track->face_m) <= 0.012f;
    if(track->locked && fabsf(remaining - laser_remaining) > 0.018f) {
        track->samples = 0u;
        return false;
    }
    if(!continuous) {
        track->samples = 0u;
        track->first_ms = sample->stamp_ms;
        track->face_m = face;
        track->first_wheel_m = at_sample;
        track->first_range_m = laser_remaining;
    }
    track->last_ms = sample->stamp_ms;
    if(track->samples < 3u) ++track->samples;
    if(track->samples < 3u || sample->stamp_ms - track->first_ms < 40u) return false;
    /* A carpet edge or blocked beam can give a perfectly stable short range
     * while the robot moves. Acquire only after seeing the range actually
     * close with the wheels. Once locked, still allow a stationary update. */
    if(!track->locked) {
        const float advance = sign * (at_sample - track->first_wheel_m);
        if(advance < 0.022f || track->first_range_m - laser_remaining < 0.65f * advance) return false;
    }
    if(!track->locked && (index == 0u || index == 1u)) {
        /* First sight of the far fence measures the full span on this axis.
         * Apply that scale to subsequent row centres and connector lengths. */
        const float measured = along_x ? face : MAP_START_Y_M + face;
        const float nominal = along_x ? goal - MAP_START_X_M : goal;
        axis->scale = fmaxf(0.95f, fminf(1.05f, measured / nominal));
    }
    track->locked = track->frame_accepted = true;
    track->face_m = face;
    axis->bias_m = map_route_wheel_target(along_x, goal, axis->scale) - face;
    return true;
}
