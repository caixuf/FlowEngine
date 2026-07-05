/**
 * kalman_tracker.c — Multi-object Kalman tracker with Hungarian assignment.
 *
 * Core: 4D constant-velocity Kalman filter per track.
 * Association: Hungarian algorithm on Mahalanobis distance cost matrix.
 */

#include "kalman_tracker.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>

/* ══════════════════════════════════════════════════════════ */
/*  内部: Kalman Filter (单轨迹, 4D CV model)                  */
/* ══════════════════════════════════════════════════════════ */

/* 状态转移矩阵 F (4×4): x'=x+vx*dt, y'=y+vy*dt, vx'=vx, vy'=vy */
static void kf_predict(KTrack* t, double dt,
                        double q_pos, double q_vel) {
    /* 预测状态 */
    double x_new[4];
    x_new[0] = t->x[0] + t->x[2] * dt;
    x_new[1] = t->x[1] + t->x[3] * dt;
    x_new[2] = t->x[2];
    x_new[3] = t->x[3];

    /* F: 4×4 */
    double F[16] = {
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1,  0,
        0, 0, 0,  1,
    };

    /* P' = F * P * Fᵀ + Q */
    /* Step 1: FP = F * P */
    double FP[16] = {0};
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 4; j++)
                FP[i*4 + j] += F[i*4 + k] * t->P[k*4 + j];

    /* Step 2: P' = FP * Fᵀ */
    memset(t->P, 0, 16 * sizeof(double));
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 4; j++)
                t->P[i*4 + j] += FP[i*4 + k] * F[j*4 + k];

    /* Step 3: +Q */
    double dt2 = dt * dt;
    double dt3 = dt2 * dt / 2.0;
    double dt4 = dt2 * dt2 / 4.0;
    t->P[0*4 + 0] += dt4 * q_pos;
    t->P[0*4 + 2] += dt3 * q_pos;
    t->P[2*4 + 0] += dt3 * q_pos;
    t->P[2*4 + 2] += dt2 * q_pos;
    t->P[1*4 + 1] += dt4 * q_pos;
    t->P[1*4 + 3] += dt3 * q_pos;
    t->P[3*4 + 1] += dt3 * q_pos;
    t->P[3*4 + 3] += dt2 * q_pos;

    /* 速度过程噪声 */
    t->P[2*4 + 2] += dt2 * q_vel;
    t->P[3*4 + 3] += dt2 * q_vel;

    memcpy(t->x, x_new, sizeof(x_new));
    t->age++;
}

/* 用位置测量 [zx, zy] 更新 */
static void kf_update(KTrack* t, double zx, double zy, double r_pos) {
    /* H = [[1,0,0,0], [0,1,0,0]] */
    /* Innovation y = z - H*x */
    double y[2] = {
        zx - t->x[0],
        zy - t->x[1],
    };

    /* S = H*P*Hᵀ + R */
    /* P*Hᵀ = columns 0,1 of P → 4×2 */
    double PHT[8] = {
        t->P[0], t->P[1],
        t->P[4], t->P[5],
        t->P[8], t->P[9],
        t->P[12],t->P[13],
    };

    double S[4] = {
        PHT[0] + r_pos,  PHT[1],
        PHT[4],           PHT[5] + r_pos,
    };

    /* S⁻¹ (2×2) */
    double det = S[0]*S[3] - S[1]*S[2];
    if (fabs(det) < 1e-12) return;
    double Sinv[4] = {
         S[3]/det, -S[1]/det,
        -S[2]/det,  S[0]/det,
    };

    /* K = PHT * Sinv (4×2) */
    double K[8] = {0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
                K[i*2 + j] += PHT[i*2 + k] * Sinv[k*2 + j];

    /* x' = x + K*y */
    for (int i = 0; i < 4; i++)
        t->x[i] += K[i*2]*y[0] + K[i*2+1]*y[1];

    /* P' = (I - K*H) * P */
    /* I-KH = I - K*[1 0 0 0; 0 1 0 0] */
    double IminusKH[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    };
    IminusKH[0] -= K[0]; IminusKH[1] -= K[1];
    IminusKH[4] -= K[2]; IminusKH[5] -= K[3];
    IminusKH[8] -= K[4]; IminusKH[9] -= K[5];
    IminusKH[12]-= K[6]; IminusKH[13]-= K[7];

    double Pnew[16] = {0};
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 4; j++)
                Pnew[i*4 + j] += IminusKH[i*4 + k] * t->P[k*4 + j];
    memcpy(t->P, Pnew, sizeof(Pnew));
}

/* Mahalanobis 距离: sqrt(yᵀ * S⁻¹ * y) */
static double mahalanobis_dist(const KTrack* t, const KTrackDetection* d,
                                double r_pos) {
    double y[2] = { d->x - t->x[0], d->y - t->x[1] };

    double S[4] = {
        t->P[0] + r_pos, t->P[1],
        t->P[4],          t->P[5] + r_pos,
    };
    double det = S[0]*S[3] - S[1]*S[2];
    if (det < 1e-10) return 1e9;

    /* y * S⁻¹ * yᵀ = (S[3]*y0² - 2*S[1]*y0*y1 + S[0]*y1²) / det */
    double m2 = (S[3]*y[0]*y[0] - 2.0*S[1]*y[0]*y[1] + S[0]*y[1]*y[1]) / det;
    return sqrt(m2);
}

/* ══════════════════════════════════════════════════════════ */
/*  内部: Hungarian Algorithm (Munkres)                       */
/* ══════════════════════════════════════════════════════════ */

#define HUNGARIAN_MAX 32

static double hungarian_solve(double cost[HUNGARIAN_MAX][HUNGARIAN_MAX],
                               int n_rows, int n_cols,
                               int assignment[HUNGARIAN_MAX]) {
    /* 初始化: 每行减行最小值 */
    double u[HUNGARIAN_MAX] = {0};
    double v[HUNGARIAN_MAX] = {0};
    int    p[HUNGARIAN_MAX]  = {0};  /* p[j] = 匹配到列 j 的行 */
    int    way[HUNGARIAN_MAX] = {0};

    for (int i = 1; i <= n_rows; i++) {
        p[0] = i;
        int j0 = 0;
        double minv[HUNGARIAN_MAX];
        for (int j = 0; j <= n_cols; j++) minv[j] = 1e12;
        int used[HUNGARIAN_MAX] = {0};

        do {
            used[j0] = 1;
            int i0 = p[j0];
            double delta = 1e12;
            int j1 = 0;

            for (int j = 1; j <= n_cols; j++) {
                if (!used[j]) {
                    double cur = cost[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int j = 0; j <= n_cols; j++) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j]    -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    /* 输出 assignment: assignment[row] = col */
    for (int j = 1; j <= n_cols; j++) {
        if (p[j] > 0)
            assignment[p[j] - 1] = j - 1;
    }
    /* 未匹配的行标记为 -1 */
    for (int i = 0; i < n_rows; i++) {
        int matched = 0;
        for (int j = 1; j <= n_cols; j++) {
            if (p[j] == i + 1) { matched = 1; break; }
        }
        if (!matched) assignment[i] = -1;
    }

    return -v[0]; /* 总代价 */
}

/* ══════════════════════════════════════════════════════════ */
/*  Public API                                                */
/* ══════════════════════════════════════════════════════════ */

void ktracker_init(KalmanTracker* kt, double dt) {
    memset(kt, 0, sizeof(*kt));
    kt->dt                    = dt;
    kt->next_id               = 1;
    kt->tent_hits_to_confirm  = 3;
    kt->max_misses_before_delete = 10;
    kt->max_assoc_dist        = 4.0f;    /* 4m Mahalanobis 门限 (已确认轨迹) */
    kt->max_assoc_dist_init   = 8.0f;    /* 8m (新建) */
    kt->q_pos                 = 0.5;     /* 位置过程噪声 (m²/s⁴) */
    kt->q_vel                 = 1.0;     /* 速度过程噪声 (m²/s²) */
    kt->r_pos                 = 0.25;    /* 位置测量噪声 (m²) */
}

void ktracker_predict(KalmanTracker* kt) {
    for (int i = 0; i < kt->n_tracks; i++) {
        if (kt->tracks[i].state == TRACK_CONFIRMED ||
            kt->tracks[i].state == TRACK_TENTATIVE) {
            kf_predict(&kt->tracks[i], kt->dt, kt->q_pos, kt->q_vel);
        }
    }
}

void ktracker_associate_and_update(KalmanTracker* kt,
                                    const KTrackDetection* dets, int n_dets) {
    if (n_dets > KTRACKER_MAX_DETS) n_dets = KTRACKER_MAX_DETS;

    /* ── Step 1: 构建关联代价矩阵 ── */
    int n_trk = kt->n_tracks;
    if (n_trk == 0 && n_dets == 0) return;

    /* 如果无轨迹，所有检测都创建新轨迹 */
    if (n_trk == 0) {
        for (int d = 0; d < n_dets; d++) {
            if (kt->n_tracks >= KTRACKER_MAX_TRACKS) break;
            KTrack* t = &kt->tracks[kt->n_tracks++];
            memset(t, 0, sizeof(*t));
            t->id = kt->next_id++;
            t->state = TRACK_TENTATIVE;
            t->x[0] = dets[d].x;  t->x[1] = dets[d].y;
            t->x[2] = dets[d].vx; t->x[3] = dets[d].vy;
            /* 初始协方差 */
            memset(t->P, 0, 16 * sizeof(double));
            t->P[0] = kt->r_pos;  t->P[5] = kt->r_pos;
            t->P[10] = 100.0;     t->P[15] = 100.0;
            t->width  = dets[d].width;
            t->length = dets[d].length;
            t->cls    = dets[d].cls;
            t->confidence = dets[d].confidence;
            t->hits = 1;
            t->hits_streak = 1;
        }
        return;
    }

    /* 如果无检测，所有轨迹都记为 miss */
    if (n_dets == 0) {
        for (int t = 0; t < n_trk; t++) {
            kt->tracks[t].misses++;
            if (kt->tracks[t].misses > 0)
                kt->tracks[t].state = TRACK_COASTING;
        }
        /* 删除 COASTING 过久的 */
        int write = 0;
        for (int i = 0; i < kt->n_tracks; i++) {
            if (kt->tracks[i].misses >= kt->max_misses_before_delete) continue;
            if (i != write) kt->tracks[write] = kt->tracks[i];
            write++;
        }
        kt->n_tracks = write;
        return;
    }

    /* 代价矩阵: track行 × detection列 */
    double cost[HUNGARIAN_MAX][HUNGARIAN_MAX];
    for (int t = 0; t < n_trk; t++) {
        double max_d = (kt->tracks[t].state == TRACK_TENTATIVE)
                        ? kt->max_assoc_dist_init : kt->max_assoc_dist;
        for (int d = 0; d < n_dets; d++) {
            double mdist = mahalanobis_dist(&kt->tracks[t], &dets[d], kt->r_pos);
            if (mdist > max_d) {
                cost[t][d] = 1e9;  /* 不可匹配 */
            } else {
                cost[t][d] = mdist;
            }
        }
    }

    /* Hungarian 求解 */
    int n_match = (n_trk > n_dets) ? n_trk : n_dets;
    /* 方形化: 填充大代价 */
    for (int t = 0; t < n_match; t++) {
        for (int d = 0; d < n_match; d++) {
            if (t >= n_trk || d >= n_dets)
                cost[t][d] = 1e9;
        }
    }

    int assignment[HUNGARIAN_MAX];
    hungarian_solve(cost, n_match, n_match, assignment);

    /* ── Step 2: 处理匹配对 ── */
    bool det_matched[KTRACKER_MAX_DETS] = {0};

    for (int t = 0; t < n_trk; t++) {
        int d = assignment[t];
        if (d >= 0 && d < n_dets && cost[t][d] < 1e8) {
            /* 匹配成功: Kalman 更新 */
            kf_update(&kt->tracks[t], dets[d].x, dets[d].y, kt->r_pos);
            kt->tracks[t].width  = 0.7f * kt->tracks[t].width  + 0.3f * dets[d].width;
            kt->tracks[t].length = 0.7f * kt->tracks[t].length + 0.3f * dets[d].length;
            kt->tracks[t].confidence = 0.8f * kt->tracks[t].confidence
                                       + 0.2f * dets[d].confidence;
            kt->tracks[t].misses = 0;
            kt->tracks[t].hits++;
            kt->tracks[t].hits_streak++;
            if (kt->tracks[t].hits_streak >= kt->tent_hits_to_confirm)
                kt->tracks[t].state = TRACK_CONFIRMED;
            det_matched[d] = true;
        } else {
            /* 未匹配: miss */
            kt->tracks[t].misses++;
            kt->tracks[t].hits_streak = 0;
            if (kt->tracks[t].state == TRACK_CONFIRMED && kt->tracks[t].misses > 0) {
                kt->tracks[t].state = TRACK_COASTING;
            }
        }
    }

    /* ── Step 3: 删除过期轨迹 ── */
    int write = 0;
    for (int i = 0; i < kt->n_tracks; i++) {
        if (kt->tracks[i].state == TRACK_TENTATIVE && kt->tracks[i].misses >= 2) continue;
        if (kt->tracks[i].misses >= kt->max_misses_before_delete) continue;
        if (i != write) kt->tracks[write] = kt->tracks[i];
        write++;
    }
    kt->n_tracks = write;

    /* ── Step 4: 为新检测创建轨迹 ── */
    for (int d = 0; d < n_dets; d++) {
        if (det_matched[d]) continue;
        if (kt->n_tracks >= KTRACKER_MAX_TRACKS) break;

        KTrack* t = &kt->tracks[kt->n_tracks++];
        memset(t, 0, sizeof(*t));
        t->id    = kt->next_id++;
        t->state = TRACK_TENTATIVE;
        t->x[0]  = dets[d].x;   t->x[1] = dets[d].y;
        t->x[2]  = dets[d].vx;  t->x[3] = dets[d].vy;
        memset(t->P, 0, 16 * sizeof(double));
        t->P[0] = kt->r_pos;  t->P[5]  = kt->r_pos;
        t->P[10] = 100.0;     t->P[15] = 100.0;
        t->width  = dets[d].width;
        t->length = dets[d].length;
        t->cls    = dets[d].cls;
        t->confidence = dets[d].confidence;
        t->hits   = 1;
        t->hits_streak = 1;
    }
}

int ktracker_confirmed_count(const KalmanTracker* kt) {
    int count = 0;
    for (int i = 0; i < kt->n_tracks; i++)
        if (kt->tracks[i].state == TRACK_CONFIRMED) count++;
    return count;
}

int ktracker_total_count(const KalmanTracker* kt) {
    return kt->n_tracks;
}

void ktracker_reset(KalmanTracker* kt) {
    kt->n_tracks = 0;
    kt->next_id  = 1;
}
