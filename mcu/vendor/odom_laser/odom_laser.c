/**
 * odom_laser.c
 *
 * Portable C99 implementation of the four-beam odometry baseline.
 * No UART, no dynamic allocation, no RTOS.  The core has no printf; the
 * optional odom_log() formatter uses integer snprintf only.
 *
 * See odom_laser.h for the geometry, API and known limitations.
 */

#include "odom_laser.h"

#if ODOM_ENABLE_LOG
#include <stdio.h>
#endif

/* ------------------------------------------------------------------------- */
/* Small helpers                                                             */
/* ------------------------------------------------------------------------- */

static float odom_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float odom_median3(float a, float b, float c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) {
        return b;
    }
    if ((b <= a && a <= c) || (c <= a && a <= b)) {
        return a;
    }
    return c;
}

/* ------------------------------------------------------------------------- */
/* Default configuration: PLACEHOLDERS, fill with measured values             */
/* ------------------------------------------------------------------------- */

void odom_config_default(odom_config_t *cfg)
{
    int i;

    if (cfg == 0) {
        return;
    }

    cfg->lx = 2.50f;
    cfg->af = 0.30f;
    cfg->ar = 0.30f;
    cfg->bl = 0.20f;
    cfg->br = 0.20f;
    cfg->min_range = 0.10f;
    cfg->max_range = 8.00f;

    cfg->n_furrow = 3;
    for (i = 0; i < ODOM_MAX_FURROW; ++i) {
        cfg->furrow_center[i] = 0.0f;
        cfg->furrow_width[i] = 0.0f;
        cfg->x_left[i] = 0.0f;
        cfg->x_right[i] = 0.0f;
    }
    cfg->furrow_center[0] = 0.60f;
    cfg->furrow_center[1] = 1.80f;
    cfg->furrow_center[2] = 3.00f;
    cfg->furrow_width[0] = 0.80f;
    cfg->furrow_width[1] = 0.80f;
    cfg->furrow_width[2] = 0.80f;
    for (i = 0; i < 3; ++i) {
        cfg->x_left[i] = 0.0f;
        cfg->x_right[i] = cfg->lx;
    }

    cfg->tol_x = 0.10f;
    cfg->tol_y_valid = 0.07f;
    cfg->tol_y_exit = 0.12f;
    cfg->t_cross_min_s = 0.15f;
    cfg->t_cross_max_s = 6.00f;
    cfg->n_stable = 5;
    cfg->vy_eps = 0.02f;
    cfg->dy_eps = 0.004f;
    cfg->end_guard = 0.15f;

    cfg->kf_q_pos = 1e-6f;
    cfg->kf_q_vel = 2e-2f;
    cfg->kf_r = 0.02f;   /* std dev, R = 0.02^2, matches the simulation */
    cfg->kf_gate = 4.0f;
    cfg->kf_n_accept = 5;
}

/* ------------------------------------------------------------------------- */
/* Init / reset                                                              */
/* ------------------------------------------------------------------------- */

bool odom_init(odom_t *o, const odom_config_t *cfg)
{
    if (o == 0) {
        return false;
    }

    if (cfg != 0) {
        o->cfg = *cfg;
    } else {
        odom_config_default(&o->cfg);
    }

    if (o->cfg.n_furrow < 1 || o->cfg.n_furrow > ODOM_MAX_FURROW) {
        return false;
    }
    if (o->cfg.lx <= 0.0f) {
        return false;
    }

    odom_reset(o);
    o->log_fn = 0;
    return true;
}

void odom_reset(odom_t *o)
{
    int ch;
    int i;
    int j;

    if (o == 0) {
        return;
    }

    o->initialized = 0;
    o->fsm = (uint8_t)ODOM_FSM_FURROW;
    o->furrow = 0;
    o->cross_dir = 0;
    o->stable = 0;
    o->cross_t0 = 0;
    o->last_t = 0;
    o->prev_dy = 0.0f;
    o->dy_sign = 0.0f;
    o->x_reject = 0;
    o->y_reject = 0;

    for (ch = 0; ch < 4; ++ch) {
        o->hist_n[ch] = 0;
        o->hist_idx[ch] = 0;
        o->last_good[ch] = 0.0f;
        o->last_good_valid[ch] = 0;
        for (i = 0; i < 3; ++i) {
            o->hist[ch][i] = 0.0f;
        }
    }

    for (i = 0; i < 4; ++i) {
        o->s[i] = 0.0f;
        for (j = 0; j < 4; ++j) {
            o->P[i][j] = 0.0f;
        }
    }
    o->P[0][0] = 0.02f * 0.02f;
    o->P[1][1] = 0.02f * 0.02f;
    o->P[2][2] = 0.20f * 0.20f;
    o->P[3][3] = 0.20f * 0.20f;

    o->x_meas = 0.0f;
    o->y_meas = 0.0f;
    o->x_ok = false;
    o->y_ok = false;
    o->xy[0] = 0.0f;
    o->xy[1] = 0.0f;
    o->quality = 0;

    for (i = 0; i < 4; ++i) {
        o->last_d[i] = 0.0f;
    }
    o->last_valid = 0;
    /* log_fn is intentionally kept: odom_reset() is called at the corner and
       the debug sink should survive it. */
}

/* ------------------------------------------------------------------------- */
/* Median / spike filter                                                     */
/* ------------------------------------------------------------------------- */

static void odom_filter(odom_t *o, float d[4], bool valid[4])
{
    int ch;

    for (ch = 0; ch < 4; ++ch) {
        float med;

        if (!valid[ch]) {
            continue;
        }

        o->hist[ch][o->hist_idx[ch]] = d[ch];
        o->hist_idx[ch] = (uint8_t)((o->hist_idx[ch] + 1u) % 3u);
        if (o->hist_n[ch] < 3u) {
            o->hist_n[ch]++;
        }

        if (o->hist_n[ch] >= 3u) {
            med = odom_median3(o->hist[ch][0], o->hist[ch][1], o->hist[ch][2]);
            if (odom_absf(d[ch] - med) > 0.25f && o->last_good_valid[ch]) {
                d[ch] = med;
                o->quality |= ODOM_Q_SPIKE;
            } else {
                o->last_good[ch] = d[ch];
                o->last_good_valid[ch] = 1;
            }
        } else {
            o->last_good[ch] = d[ch];
            o->last_good_valid[ch] = 1;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Kalman filter: state = [x, y, vx, vy]                                     */
/* ------------------------------------------------------------------------- */

static void odom_kf_predict(odom_t *o, float dt)
{
    float fp[4][4];
    float np[4][4];
    float qp = o->cfg.kf_q_pos * dt;
    float qv = o->cfg.kf_q_vel * dt;
    int i;
    int j;

    /* state: x += vx*dt, y += vy*dt */
    o->s[0] += dt * o->s[2];
    o->s[1] += dt * o->s[3];

    /* fp = F*P, F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]] */
    for (j = 0; j < 4; ++j) {
        fp[0][j] = o->P[0][j] + dt * o->P[2][j];
        fp[1][j] = o->P[1][j] + dt * o->P[3][j];
        fp[2][j] = o->P[2][j];
        fp[3][j] = o->P[3][j];
    }

    /* np = fp * F^T
       F row0 = [1,0,dt,0]
       F row1 = [0,1,0,dt]
       F row2 = [0,0,1,0]
       F row3 = [0,0,0,1] */
    for (i = 0; i < 4; ++i) {
        np[i][0] = fp[i][0] + dt * fp[i][2];
        np[i][1] = fp[i][1] + dt * fp[i][3];
        np[i][2] = fp[i][2];
        np[i][3] = fp[i][3];
    }

    np[0][0] += qp;
    np[1][1] += qp;
    np[2][2] += qv;
    np[3][3] += qv;

    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            o->P[i][j] = np[i][j];
        }
    }
}

static bool odom_kf_update(odom_t *o, float z, int axis, float gate)
{
    float s_innov;
    float nu;
    float k[4];
    float row[4];
    int i;
    int j;

    if (axis != 0 && axis != 1) {
        return false;
    }

    s_innov = o->P[axis][axis] + o->cfg.kf_r * o->cfg.kf_r;
    if (s_innov < 1e-9f) {
        return false;
    }

    nu = z - o->s[axis];
    if (gate > 0.0f && nu * nu > gate * gate * s_innov) {
        return false;
    }

    for (i = 0; i < 4; ++i) {
        k[i] = o->P[i][axis] / s_innov;
        o->s[i] += k[i] * nu;
    }

    for (j = 0; j < 4; ++j) {
        row[j] = o->P[axis][j];
    }
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            o->P[i][j] -= k[i] * row[j];
        }
    }

    /* Keep P symmetric. */
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < i; ++j) {
            float v = 0.5f * (o->P[i][j] + o->P[j][i]);
            o->P[i][j] = v;
            o->P[j][i] = v;
        }
    }

    return true;
}

/* ------------------------------------------------------------------------- */
/* Measurement models                                                        */
/* ------------------------------------------------------------------------- */

static float odom_side_single(const odom_t *o, const float d[4], const bool valid[4])
{
    int i = o->furrow;

    if (valid[2] && !valid[3]) {
        return o->cfg.furrow_center[i] + 0.5f * o->cfg.furrow_width[i]
               - o->cfg.bl - d[2];
    }
    if (valid[3] && !valid[2]) {
        return o->cfg.furrow_center[i] - 0.5f * o->cfg.furrow_width[i]
               + o->cfg.br + d[3];
    }
    return 0.0f;
}

static void odom_x_measure(odom_t *o, const float d[4], const bool valid[4])
{
    int i = o->furrow;
    float xl = o->cfg.x_left[i];
    float xr = o->cfg.x_right[i];
    float xf = xr - o->cfg.af - d[0];
    float xb = xl + o->cfg.ar + d[1];
    float both = 0.5f * (d[1] - d[0] + xr + xl + o->cfg.ar - o->cfg.af);
    float sx = d[0] + d[1];
    float sx_exp = xr - xl - o->cfg.af - o->cfg.ar;
    bool both_valid = valid[0] && valid[1];

    o->x_ok = false;
    o->x_meas = 0.0f;

    /* One beam is near a field end: use the other beam alone. */
    if (both_valid &&
        (d[0] < o->cfg.min_range + o->cfg.end_guard ||
         d[1] < o->cfg.min_range + o->cfg.end_guard)) {
        o->x_meas = (d[0] < d[1]) ? xb : xf;
        o->x_ok = true;
        o->quality |= ODOM_Q_X_SINGLE;
        return;
    }

    if (both_valid && odom_absf(sx - sx_exp) < o->cfg.tol_x) {
        o->x_meas = both;
        o->x_ok = true;
        return;
    }

    if (both_valid) {
        /* Sum failed: likely an end cap/edge. Pick the beam closest to prediction. */
        o->x_meas = (odom_absf(xf - o->s[0]) <= odom_absf(xb - o->s[0])) ? xf : xb;
        o->x_ok = true;
        o->quality |= ODOM_Q_X_SINGLE | ODOM_Q_X_SUSPECT;
        return;
    }

    if (valid[0]) {
        o->x_meas = xf;
        o->x_ok = true;
        o->quality |= ODOM_Q_X_SINGLE;
        return;
    }
    if (valid[1]) {
        o->x_meas = xb;
        o->x_ok = true;
        o->quality |= ODOM_Q_X_SINGLE;
    }
}

static void odom_y_measure(odom_t *o, const float d[4], const bool valid[4],
                           uint32_t now_ms)
{
    int i = o->furrow;
    bool both = valid[2] && valid[3];
    float sy = both ? (d[2] + d[3]) : 0.0f;
    float dy = both ? (d[2] - d[3]) : 0.0f;
    float sy_f = o->cfg.furrow_width[i] - o->cfg.bl - o->cfg.br;
    bool in_furrow = both && (odom_absf(sy - sy_f) < o->cfg.tol_y_valid);
    bool wide = both && (odom_absf(sy - sy_f) > o->cfg.tol_y_exit);
    uint32_t t_min_ms = (uint32_t)(o->cfg.t_cross_min_s * 1000.0f + 0.5f);
    uint32_t t_max_ms = (uint32_t)(o->cfg.t_cross_max_s * 1000.0f + 0.5f);

    if (both && odom_absf(dy - o->prev_dy) > o->cfg.dy_eps) {
        /* y = const - diff/2: diff decreasing means y increasing. */
        o->dy_sign = (dy < o->prev_dy) ? 1.0f : -1.0f;
    }

    o->y_ok = false;
    o->y_meas = 0.0f;

    switch (o->fsm) {
    case (uint8_t)ODOM_FSM_FURROW:
        if (both) {
            if (in_furrow) {
                o->y_meas = o->cfg.furrow_center[i]
                            - 0.5f * (dy + o->cfg.bl - o->cfg.br);
                o->y_ok = true;
                o->stable = 0;
            } else if (wide) {
                o->fsm = (uint8_t)ODOM_FSM_CROSSING;
                o->cross_t0 = now_ms;
                o->stable = 0;
                if (o->s[3] > o->cfg.vy_eps) {
                    o->cross_dir = 1;
                } else if (o->s[3] < -o->cfg.vy_eps) {
                    o->cross_dir = -1;
                } else {
                    o->cross_dir = (o->dy_sign > 0.0f) ? 1 :
                                   (o->dy_sign < 0.0f) ? -1 : 0;
                }
            } else {
                o->quality |= ODOM_Q_Y_GUARD;
            }
        } else if (valid[2] || valid[3]) {
            o->y_meas = odom_side_single(o, d, valid);
            o->y_ok = true;
            o->quality |= ODOM_Q_Y_SINGLE;
        } else {
            o->quality |= ODOM_Q_Y_TIMEOUT;
        }
        break;

    case (uint8_t)ODOM_FSM_CROSSING:
        if (both) {
            if (o->cross_dir == 0) {
                if (o->s[3] > o->cfg.vy_eps) {
                    o->cross_dir = 1;
                } else if (o->s[3] < -o->cfg.vy_eps) {
                    o->cross_dir = -1;
                } else if (o->dy_sign > 0.0f) {
                    o->cross_dir = 1;
                } else if (o->dy_sign < 0.0f) {
                    o->cross_dir = -1;
                }
            }

            if (o->cross_dir != 0) {
                int next = o->furrow + o->cross_dir;
                if (next >= 0 && next < o->cfg.n_furrow) {
                    float next_exp = o->cfg.furrow_width[next]
                                     - o->cfg.bl - o->cfg.br;
                    if (odom_absf(sy - next_exp) < o->cfg.tol_y_valid) {
                        o->stable++;
                        if (o->stable >= o->cfg.n_stable &&
                            (uint32_t)(now_ms - o->cross_t0) > t_min_ms) {
                            o->furrow = next;
                            o->fsm = (uint8_t)ODOM_FSM_FURROW;
                            o->y_meas = o->cfg.furrow_center[o->furrow]
                                        - 0.5f * (dy + o->cfg.bl - o->cfg.br);
                            o->y_ok = true;
                            o->stable = 0;
                        }
                    } else {
                        o->stable = 0;
                    }
                } else {
                    o->fsm = (uint8_t)ODOM_FSM_FAULT;
                    o->quality |= ODOM_Q_Y_TIMEOUT;
                }
            } else {
                o->quality |= ODOM_Q_DIR_UNKNOWN;
            }
        }

        if ((uint32_t)(now_ms - o->cross_t0) > t_max_ms) {
            o->fsm = (uint8_t)ODOM_FSM_FAULT;
            o->quality |= ODOM_Q_Y_TIMEOUT;
        }
        if (both && o->fsm == (uint8_t)ODOM_FSM_CROSSING) {
            o->quality |= ODOM_Q_CROSSING;
        }
        break;

    case (uint8_t)ODOM_FSM_FAULT:
        if (valid[2] || valid[3]) {
            o->y_meas = odom_side_single(o, d, valid);
            o->y_ok = true;
            o->quality |= ODOM_Q_Y_SINGLE;
        } else {
            o->quality |= ODOM_Q_Y_TIMEOUT;
        }
        break;

    default:
        o->fsm = (uint8_t)ODOM_FSM_FAULT;
        o->quality |= ODOM_Q_Y_TIMEOUT;
        break;
    }

    if (both) {
        o->prev_dy = dy;
    }
}

/* ------------------------------------------------------------------------- */
/* Update                                                                    */
/* ------------------------------------------------------------------------- */

bool odom_update_ex(odom_t *o,
                    float d_f, float d_b, float d_l, float d_r,
                    uint8_t valid_mask, uint32_t now_ms)
{
    float d[4];
    bool valid[4];
    int i;

    if (o == 0) {
        return false;
    }

    d[0] = d_f;
    d[1] = d_b;
    d[2] = d_l;
    d[3] = d_r;

    for (i = 0; i < 4; ++i) {
        valid[i] = ((valid_mask & (uint8_t)(1u << i)) != 0u) &&
                   (d[i] >= o->cfg.min_range) &&
                   (d[i] <= o->cfg.max_range);
    }

    o->quality = 0;

    /* First valid frame: initialise at the known corner/furrow 0. */
    if (!o->initialized) {
        bool x_ok = valid[0] || valid[1];
        bool y_ok = valid[2] || valid[3];
        float xm;
        float ym;

        if (!x_ok || !y_ok) {
            return false;
        }

        {
            float xl = o->cfg.x_left[0];
            float xr = o->cfg.x_right[0];

            if (valid[0] && valid[1]) {
                xm = 0.5f * (d[1] - d[0] + xr + xl + o->cfg.ar - o->cfg.af);
            } else if (valid[0]) {
                xm = xr - o->cfg.af - d[0];
            } else {
                xm = xl + o->cfg.ar + d[1];
            }
        }

        if (valid[2] && valid[3]) {
            ym = o->cfg.furrow_center[0]
                 - 0.5f * (d[2] - d[3] + o->cfg.bl - o->cfg.br);
        } else if (valid[2]) {
            ym = o->cfg.furrow_center[0] + 0.5f * o->cfg.furrow_width[0]
                 - o->cfg.bl - d[2];
        } else {
            ym = o->cfg.furrow_center[0] - 0.5f * o->cfg.furrow_width[0]
                 + o->cfg.br + d[3];
        }

        o->s[0] = xm;
        o->s[1] = ym;
        o->s[2] = 0.0f;
        o->s[3] = 0.0f;
        o->P[0][0] = 0.02f * 0.02f;
        o->P[1][1] = 0.02f * 0.02f;
        o->P[2][2] = 0.20f * 0.20f;
        o->P[3][3] = 0.20f * 0.20f;
        for (i = 0; i < 4; ++i) {
            int j;
            for (j = 0; j < 4; ++j) {
                if (i != j) {
                    o->P[i][j] = 0.0f;
                }
            }
        }

        o->initialized = 1;
        o->fsm = (uint8_t)ODOM_FSM_FURROW;
        o->furrow = 0;
        o->last_t = now_ms;
        o->xy[0] = xm;
        o->xy[1] = ym;
        o->quality = ODOM_Q_X_OK | ODOM_Q_Y_OK;
        for (i = 0; i < 4; ++i) {
            o->last_d[i] = d[i];
        }
        o->last_valid = (uint8_t)((valid[0] ? ODOM_VALID_F : 0) |
                                  (valid[1] ? ODOM_VALID_B : 0) |
                                  (valid[2] ? ODOM_VALID_L : 0) |
                                  (valid[3] ? ODOM_VALID_R : 0));
        if (!(valid[0] && valid[1])) {
            o->quality |= ODOM_Q_X_SINGLE;
        }
        if (!(valid[2] && valid[3])) {
            o->quality |= ODOM_Q_Y_SINGLE;
        }
        return true;
    }

    {
        float dt = (float)(now_ms - o->last_t) * 0.001f;
        o->last_t = now_ms;
        if (dt <= 0.0f || dt > 0.2f) {
            dt = 0.01f;
        }

        odom_filter(o, d, valid);

        for (i = 0; i < 4; ++i) {
            o->last_d[i] = d[i];
        }
        o->last_valid = (uint8_t)((valid[0] ? ODOM_VALID_F : 0) |
                                  (valid[1] ? ODOM_VALID_B : 0) |
                                  (valid[2] ? ODOM_VALID_L : 0) |
                                  (valid[3] ? ODOM_VALID_R : 0));

        odom_x_measure(o, d, valid);
        odom_y_measure(o, d, valid, now_ms);

        if (o->x_ok) {
            o->quality |= ODOM_Q_X_OK;
        }
        if (o->y_ok) {
            o->quality |= ODOM_Q_Y_OK;
        }

        odom_kf_predict(o, dt);

        if (o->x_ok) {
            if (!odom_kf_update(o, o->x_meas, 0, o->cfg.kf_gate)) {
                o->x_reject++;
                o->quality |= ODOM_Q_X_REJECT;
                if (o->x_reject >= o->cfg.kf_n_accept) {
                    (void)odom_kf_update(o, o->x_meas, 0, 0.0f);
                    o->x_reject = 0;
                }
            } else {
                o->x_reject = 0;
            }
        } else {
            o->quality |= ODOM_Q_X_TIMEOUT;
        }

        if (o->y_ok) {
            if (!odom_kf_update(o, o->y_meas, 1, o->cfg.kf_gate)) {
                o->y_reject++;
                o->quality |= ODOM_Q_Y_REJECT;
                if (o->y_reject >= o->cfg.kf_n_accept) {
                    (void)odom_kf_update(o, o->y_meas, 1, 0.0f);
                    o->y_reject = 0;
                }
            } else {
                o->y_reject = 0;
            }
        }
        /* Do NOT add Q_Y_TIMEOUT here: during CROSSING/guard y is expected to
           be absent; odom_y_measure() already sets the specific quality bits. */

        o->xy[0] = o->s[0];
        o->xy[1] = o->s[1];
    }

    return true;
}

const float *odom_update(odom_t *o,
                         float d_f, float d_b, float d_l, float d_r,
                         uint32_t now_ms)
{
    uint8_t mask = 0;
    float d[4];
    int i;

    if (o == 0) {
        return 0;
    }

    d[0] = d_f;
    d[1] = d_b;
    d[2] = d_l;
    d[3] = d_r;

    for (i = 0; i < 4; ++i) {
        if (d[i] >= o->cfg.min_range && d[i] <= o->cfg.max_range) {
            mask |= (uint8_t)(1u << i);
        }
    }

    (void)odom_update_ex(o, d_f, d_b, d_l, d_r, mask, now_ms);
    return o->xy;
}

const float *odom_get_xy(const odom_t *o)
{
    return (o == 0) ? 0 : o->xy;
}

uint32_t odom_get_quality(const odom_t *o)
{
    return (o == 0) ? 0u : o->quality;
}

int odom_get_furrow(const odom_t *o)
{
    return (o == 0) ? -1 : o->furrow;
}

/* ------------------------------------------------------------------------- */
/* Optional debug log                                                        */
/* ------------------------------------------------------------------------- */

#if ODOM_ENABLE_LOG
static long odom_mm(float v)
{
    float x = v * 1000.0f;
    return (long)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}
#endif

void odom_set_logger(odom_t *o, odom_log_fn fn)
{
    if (o != 0) {
        o->log_fn = fn;
    }
}

int odom_log_format(const odom_t *o, char *buf, size_t len)
{
#if ODOM_ENABLE_LOG
    if (o == 0 || buf == 0 || len == 0u) {
        return 0;
    }

    return snprintf(buf, len,
                    "ODOM t=%lu init=%u fsm=%u i=%d q=0x%04lX v=0x%02X xo=%u yo=%u "
                    "x=%ld y=%ld vx=%ld vy=%ld "
                    "dF=%ld dB=%ld dL=%ld dR=%ld xm=%ld ym=%ld\n",
                    (unsigned long)o->last_t,
                    (unsigned)o->initialized,
                    (unsigned)o->fsm,
                    o->furrow,
                    (unsigned long)o->quality,
                    (unsigned)o->last_valid,
                    (unsigned)o->x_ok,
                    (unsigned)o->y_ok,
                    odom_mm(o->s[0]), odom_mm(o->s[1]),
                    odom_mm(o->s[2]), odom_mm(o->s[3]),
                    odom_mm(o->last_d[0]), odom_mm(o->last_d[1]),
                    odom_mm(o->last_d[2]), odom_mm(o->last_d[3]),
                    odom_mm(o->x_meas), odom_mm(o->y_meas));
#else
    (void)o;
    (void)buf;
    (void)len;
    return 0;
#endif
}

void odom_log(const odom_t *o)
{
#if ODOM_ENABLE_LOG
    char buf[ODOM_LOG_BUF_SIZE];

    if (o == 0 || o->log_fn == 0) {
        return;
    }
    (void)odom_log_format(o, buf, sizeof(buf));
    o->log_fn(buf);
#else
    (void)o;
#endif
}
