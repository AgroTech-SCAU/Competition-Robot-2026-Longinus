/**
 * odom_laser.h
 *
 * Four-beam laser odometry for a no-yaw vehicle in a ridged field.
 *
 * Geometry (all lengths in metres, times in milliseconds):
 *   x : along the ridges, end walls at x_left[i] / x_right[i]
 *   y : across the ridges, furrow centre = furrow_center[i]
 *
 *   front beam -> +x, rear beam -> -x : absolute x from the end walls
 *   left  beam -> +y, right beam -> -y: differential y inside a furrow
 *
 * The caller supplies the four ranges from the 4-in-1 kit.  This module does
 * NOT own or configure any UART.  No malloc, no RTOS.  The core has no
 * printf; the optional odom_log() formatter uses integer snprintf only.
 *
 * Start condition:
 *   Call odom_reset() while the vehicle is at the known corner/furrow 0.
 *   The first valid frame initialises x/y; later frames update the filter.
 *
 * Known limitations (documented, do not silently ignore):
 *   1. During a ridge crossing y is not measured; the filter predicts it and
 *      ODOM_Q_CROSSING is set.  Do not use y as a high-accuracy value then.
 *   2. A channel that is stuck at a plausible value is not fully detected by
 *      this baseline.  Pass a real health/valid mask to odom_update_ex(); a
 *      separate sensor-health check is recommended.
 *   3. Staggered ridge ends must be entered in x_left[]/x_right[]; the default
 *      table assumes the outer end walls are visible.
 *   4. The default geometry is a placeholder.  Fill odom_config_t for the
 *      actual field before use.
 */

#ifndef ODOM_LASER_H
#define ODOM_LASER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ODOM_MAX_FURROW 8

/* Optional debug log formatting (integer-only snprintf; no float printf).
   Define ODOM_ENABLE_LOG=0 in the build to remove it. */
#ifndef ODOM_ENABLE_LOG
#define ODOM_ENABLE_LOG 1
#endif
#define ODOM_LOG_BUF_SIZE 192

/* Quality bits returned by odom_get_quality(). */
#define ODOM_Q_X_OK        (1u << 0)
#define ODOM_Q_X_SINGLE    (1u << 1)
#define ODOM_Q_X_SUSPECT   (1u << 2)
#define ODOM_Q_X_REJECT    (1u << 3)
#define ODOM_Q_X_TIMEOUT   (1u << 4)
#define ODOM_Q_Y_OK        (1u << 5)
#define ODOM_Q_Y_SINGLE    (1u << 6)
#define ODOM_Q_Y_GUARD     (1u << 7)
#define ODOM_Q_Y_REJECT    (1u << 8)
#define ODOM_Q_Y_TIMEOUT   (1u << 9)
#define ODOM_Q_CROSSING    (1u << 10)
#define ODOM_Q_DIR_UNKNOWN (1u << 11)
#define ODOM_Q_SPIKE       (1u << 12)

/* valid_mask bits for odom_update_ex(). */
#define ODOM_VALID_F (1u << 0)
#define ODOM_VALID_B (1u << 1)
#define ODOM_VALID_L (1u << 2)
#define ODOM_VALID_R (1u << 3)
#define ODOM_VALID_ALL (ODOM_VALID_F | ODOM_VALID_B | ODOM_VALID_L | ODOM_VALID_R)

typedef void (*odom_log_fn)(const char *line);

typedef enum {
    ODOM_FSM_FURROW = 0,
    ODOM_FSM_CROSSING = 1,
    ODOM_FSM_FAULT = 2
} odom_fsm_t;

typedef struct {
    /* Field geometry. */
    float lx;                 /* field length along x */
    float af;                 /* front sensor x offset from vehicle centre */
    float ar;                 /* rear  sensor x offset from vehicle centre */
    float bl;                 /* left  sensor y offset from vehicle centre */
    float br;                 /* right sensor y offset from vehicle centre */
    float min_range;          /* sensor minimum range */
    float max_range;          /* sensor maximum range */

    int n_furrow;             /* 1 .. ODOM_MAX_FURROW */
    float furrow_center[ODOM_MAX_FURROW];
    float furrow_width[ODOM_MAX_FURROW];

    /* x of the left/right end wall or ridge end for each furrow. */
    float x_left[ODOM_MAX_FURROW];
    float x_right[ODOM_MAX_FURROW];

    /* Thresholds. */
    float tol_x;              /* dF+dB sum tolerance */
    float tol_y_valid;        /* tight band: y is used only here */
    float tol_y_exit;         /* wide band: crossing starts here */
    float t_cross_min_s;      /* minimum crossing time */
    float t_cross_max_s;      /* maximum crossing time before FAULT */
    uint8_t n_stable;         /* stable frames in the next furrow before count */
    float vy_eps;             /* |vy| direction threshold */
    float dy_eps;             /* dL-dR change threshold */
    float end_guard;          /* x single-beam end zone [m] */

    /* 4-state constant-velocity Kalman filter. */
    float kf_q_pos;
    float kf_q_vel;
    float kf_r;               /* measurement standard deviation */
    float kf_gate;            /* innovation gate, e.g. 4.0 */
    uint8_t kf_n_accept;      /* force-accept after N rejected frames */
} odom_config_t;

typedef struct {
    odom_config_t cfg;

    uint8_t initialized;
    uint8_t fsm;              /* odom_fsm_t */
    int furrow;
    int cross_dir;
    uint8_t stable;
    uint32_t cross_t0;
    uint32_t last_t;
    float prev_dy;
    float dy_sign;

    uint8_t x_reject;
    uint8_t y_reject;

    /* 3-sample median / spike filter, one history per channel. */
    float hist[4][3];
    uint8_t hist_n[4];
    uint8_t hist_idx[4];
    float last_good[4];
    uint8_t last_good_valid[4];

    /* KF state: x, y, vx, vy. */
    float s[4];
    float P[4][4];

    float x_meas;
    float y_meas;
    bool x_ok;
    bool y_ok;

    /* x/y result.  xy[0]=x, xy[1]=y, metres. */
    float xy[2];
    uint32_t quality;

    /* Latest filtered ranges / valid mask, for odom_log(). */
    float last_d[4];
    uint8_t last_valid;

    /* Optional debug log sink.  Set with odom_set_logger(). */
    odom_log_fn log_fn;
} odom_t;

/** Fill cfg with the built-in placeholder geometry/thresholds. */
void odom_config_default(odom_config_t *cfg);

/**
 * Initialise the module.  cfg may be NULL to use odom_config_default().
 * Returns false if cfg is invalid.
 */
bool odom_init(odom_t *o, const odom_config_t *cfg);

/**
 * Clear the state while keeping the configuration.  Call at the known
 * corner/furrow 0; the next valid frame re-initialises x/y.
 */
void odom_reset(odom_t *o);

/**
 * Update with one four-range frame.
 * valid_mask: ODOM_VALID_F/B/L/R.  A channel is also rejected if its range is
 * outside [min_range, max_range].
 * Returns true when a frame was consumed (the first valid frame initialises).
 */
bool odom_update_ex(odom_t *o,
                    float d_f, float d_b, float d_l, float d_r,
                    uint8_t valid_mask, uint32_t now_ms);

/**
 * Convenience wrapper: treats a range as valid when it is finite and inside
 * [min_range, max_range].  Returns a pointer to the internal [x,y] array.
 * The pointer is valid until the next update.  Returns NULL if o is NULL.
 */
const float *odom_update(odom_t *o,
                         float d_f, float d_b, float d_l, float d_r,
                         uint32_t now_ms);

/** Returns the internal [x,y] array (metres), or NULL. */
const float *odom_get_xy(const odom_t *o);

/** Last quality bits (ODOM_Q_*). */
uint32_t odom_get_quality(const odom_t *o);

/** Current furrow index, or -1. */
int odom_get_furrow(const odom_t *o);

/**
 * Set the debug log sink.  The callback receives one null-terminated line;
 * it may block briefly (e.g. UART TX).  Pass NULL to disable.
 * Call odom_log() at 1-10 Hz, not at the full 100 Hz update rate.
 */
void odom_set_logger(odom_t *o, odom_log_fn fn);

/**
 * Format and send one diagnostic line through the logger set above.
 * No-op if no logger is set or ODOM_ENABLE_LOG=0.
 */
void odom_log(const odom_t *o);

/**
 * Format one diagnostic line into buf.  x/y/d/xm/ym are mm, vx/vy mm/s;
 * q/v are hex, xo/yo are the x/y measurement-valid flags.  No float printf.
 * Returns the snprintf() result (may exceed len if truncated).
 */
int odom_log_format(const odom_t *o, char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ODOM_LASER_H */
