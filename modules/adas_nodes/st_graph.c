/**
 * st_graph.c — ST 图 + DP 速度规划器实现
 * 见 st_graph.h 头注释。算法 1:1 移植自 tools/speed_planner_sim.py 的
 * dp_speed_profile（11/11 场景 PASS 后冻结）。
 *
 * 内存：dp 表 90 格 × 101 候选 × 16B ≈ 142KB → 用静态数组（节点单线程）。
 */

#include "st_graph.h"
#include <math.h>
#include <string.h>

double stg_curve_v_limit(double kappa)
{
    if (kappa < 1e-9 && kappa > -1e-9) return STG_V_MAX;
    return STG_CURVE_SAFETY * sqrt(STG_A_LAT_MAX / fabs(kappa));
}

/* ── 占据检查 ─────────────────────────────────────────────── */

static int occupied_at(const StgInput* in, double s, double t)
{
    /* 红灯墙：变绿后（t>=t_red）消失 */
    for (int i = 0; i < in->n_walls; i++) {
        const StgRedWall* w = &in->walls[i];
        if (w->t_red >= 0.0 && t >= w->t_red) continue;
        double ws = w->stopline_s - w->wall_margin;  /* 墙中心 */
        if (fabs(s - ws) <= 0.3) return 1;           /* 墙半宽 0.3 */
    }
    for (int i = 0; i < in->n_obstacles; i++) {
        const StgObstacle* o = &in->obstacles[i];
        if (fabs(s - (o->s0 + o->v * t)) <= o->half_len) return 1;
    }
    return 0;
}

/* ── 静态约束 v_lim(s) ────────────────────────────────────── */

static void build_v_lim(const StgInput* in, int n, const double* s_list,
                        double* v_lim)
{
    for (int i = 0; i < n; i++) {
        double s = s_list[i];
        double v = STG_V_MAX;
        if (in->kappa_fn) {
            double vc = stg_curve_v_limit(in->kappa_fn(s, in->kappa_user));
            if (vc < v) v = vc;
        }
        if (in->stop_s >= 0.0) {
            double d = in->stop_s - s;
            if (d > 0.0) {
                double vb = sqrt(2.0 * STG_A_MAX * d);
                if (vb < v) v = vb;
            } else {
                v = 0.0;
            }
        }
        v_lim[i] = v;
    }
}

/* ── DP 状态 ──────────────────────────────────────────────── */

#define STG_MAX_CAND 101  /* 0..20.0 步长 0.2 → 101 候选 */
#define STG_INF 1e18

typedef struct {
    double cost;
    int    prev_k;   /* -1 = 起点 */
    double t;
    double v;
} StgDpState;

/* 静态 dp 表（节点单线程，安全） */
static StgDpState s_dp[STG_MAX_GRID][STG_MAX_CAND];
static double    s_cand[STG_MAX_GRID][STG_MAX_CAND];
static int       s_cand_n[STG_MAX_GRID];

static double accel_between(double vj, double vk)
{
    return (vk * vk - vj * vj) / (2.0 * STG_S_RES);
}

int st_graph_plan(const StgInput* in, StgResult* out)
{
    if (!in || !out) return -1;

    /* 视界动态扩展：max(50, 停点+5) */
    double horizon = STG_S_HORIZON;
    if (in->stop_s >= 0.0) {
        double h2 = in->stop_s + 5.0;
        if (h2 > horizon) horizon = h2;
    }
    int n = (int)(horizon / STG_S_RES) + 1;
    if (n > STG_MAX_GRID) n = STG_MAX_GRID;
    out->n = n;
    out->horizon = (n - 1) * STG_S_RES;

    /* s 格与 v_lim */
    double s_list[STG_MAX_GRID];
    double v_lim[STG_MAX_GRID];
    for (int i = 0; i < n; i++) s_list[i] = (double)i * STG_S_RES;
    build_v_lim(in, n, s_list, v_lim);

    /* 每格候选速度 */
    for (int i = 0; i < n; i++) {
        double lim = v_lim[i];
        if (lim < 0.0) lim = 0.0;
        if (lim > STG_V_MAX) lim = STG_V_MAX;
        int m = (int)(lim / STG_V_CAND_STEP) + 1;
        if (m > STG_MAX_CAND) m = STG_MAX_CAND;
        s_cand_n[i] = m;
        for (int k = 0; k < m; k++) s_cand[i][k] = (double)k * STG_V_CAND_STEP;
    }

    /* 第 0 列：起点固定 v0（clamp 到 v_lim[0]） */
    {
        double start_v = in->v0;
        if (start_v > v_lim[0]) start_v = v_lim[0];
        if (start_v < 0.0) start_v = 0.0;
        int m0 = s_cand_n[0];
        int k0 = 0;
        {
            double best_d = 1e18;
            for (int k = 0; k < m0; k++) {
                double d = fabs(s_cand[0][k] - start_v);
                if (d < best_d) { best_d = d; k0 = k; }
            }
        }
        for (int k = 0; k < m0; k++) {
            double v = s_cand[0][k];
            s_dp[0][k].v = v;
            s_dp[0][k].t = in->t0;
            s_dp[0][k].prev_k = -1;
            if (occupied_at(in, s_list[0], in->t0) && v > 0.0) {
                s_dp[0][k].cost = STG_INF;
            } else {
                s_dp[0][k].cost = fabs(v - in->v_target) * STG_W1;
            }
        }
        /* 覆盖 k0 为实际起点（最低 cost 化：起点状态本身 cost 允许非最低） */
        s_dp[0][k0].cost = fabs(s_cand[0][k0] - in->v_target) * STG_W1;
        s_dp[0][k0].prev_k = -1;
        s_dp[0][k0].t = in->t0;
        s_dp[0][k0].v = s_cand[0][k0];
    }

    /* 逐列转移（全量存 dp 表，回溯用） */
    for (int i = 1; i < n; i++) {
        int ck_n = s_cand_n[i];
        int cj_n = s_cand_n[i - 1];
        for (int k = 0; k < ck_n; k++) {
            double vk = s_cand[i][k];
            double best_cost = STG_INF;
            int    best_j = -1;
            double best_t = 0.0;
            for (int j = 0; j < cj_n; j++) {
                const StgDpState* p = &s_dp[i - 1][j];
                if (p->cost >= STG_INF) continue;
                double vj = s_cand[i - 1][j];
                double a = accel_between(vj, vk);
                if (fabs(a) > STG_A_MAX + 1e-9) continue;
                double dt;
                if (vj + vk > 1e-6) dt = 2.0 * STG_S_RES / (vj + vk);
                else dt = 0.05;  /* 静止点时间下限 */
                double t = p->t + dt;
                if (occupied_at(in, s_list[i], t) && vk > 0.0) continue;
                double cost = p->cost
                            + STG_W1 * (vk - in->v_target) * (vk - in->v_target)
                            + STG_W2 * a * a;
                if (cost < best_cost) {
                    best_cost = cost; best_j = j; best_t = t;
                }
            }
            if (best_j >= 0) {
                s_dp[i][k].cost = best_cost;
                s_dp[i][k].prev_k = best_j;
                s_dp[i][k].t = best_t;
                s_dp[i][k].v = vk;
            } else {
                s_dp[i][k].cost = STG_INF;
                s_dp[i][k].prev_k = -1;
                s_dp[i][k].t = s_dp[i - 1][0].t;
                s_dp[i][k].v = vk;
            }
        }
        /* 整列不可达兜底：取前一列最低 cost 状态延续（理论上不会发生） */
        int all_dead = 1;
        for (int k = 0; k < ck_n; k++)
            if (s_dp[i][k].cost < STG_INF) { all_dead = 0; break; }
        if (all_dead) {
            int j_min = 0;
            for (int j = 1; j < cj_n; j++)
                if (s_dp[i - 1][j].cost < s_dp[i - 1][j_min].cost) j_min = j;
            for (int k = 0; k < ck_n; k++) {
                s_dp[i][k].cost = s_dp[i - 1][j_min].cost
                                + STG_W1 * s_cand[i][k] * s_cand[i][k];
                s_dp[i][k].prev_k = j_min;
                s_dp[i][k].t = s_dp[i - 1][j_min].t;
                s_dp[i][k].v = s_cand[i][k];
            }
        }
    }

    /* 回溯：末列最低 cost 状态出发，沿 prev_k 链反向走 */
    int last = 0;
    for (int k = 1; k < s_cand_n[n - 1]; k++)
        if (s_dp[n - 1][k].cost < s_dp[n - 1][last].cost) last = k;

    memset(out->v_out, 0, sizeof(double) * (size_t)STG_MAX_GRID);
    memset(out->t_out, 0, sizeof(double) * (size_t)STG_MAX_GRID);
    for (int i = n - 1; i >= 0; i--) {
        if (last < 0 || last >= s_cand_n[i]) break;
        out->v_out[i] = s_dp[i][last].v;
        out->t_out[i] = s_dp[i][last].t;
        last = s_dp[i][last].prev_k;
    }
    return 0;
}
