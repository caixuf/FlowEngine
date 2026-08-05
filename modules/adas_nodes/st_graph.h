/**
 * st_graph.h — ST 图 + DP 速度规划器（planning 重生 M1，纯 C，无第三方依赖）
 *
 * 设计文档：docs/PLANNING_SPEED_UPGRADE_DESIGN.md
 * Python 仿真：tools/speed_planner_sim.py（11/11 场景全 PASS 后移植）
 *
 * 职责：给定路径曲率剖面 + 障碍物 + 红绿灯硬停点 + 目标速度，
 * 输出沿 s 的速度剖面 spd_out[]。替换 planning_node 旧「线性斜坡 +
 * 红绿灯 override 堆」的机制（原 spd_out[i] = v0*(1-t)+command_speed*t
 * 只有 2s 固定斜坡，无曲率/制动/红灯距离自适应）。
 *
 * 算法（与 speed_planner_sim.py 的 dp_speed_profile 严格一致）：
 *   1. 静态约束 v_lim(s) = min(限速, 0.95·sqrt(a_lat_max/|κ(s)|), 制动自洽)
 *   2. ST 占据：静止/移动障碍物时空轨迹 + 红灯墙（t 全局时间，变绿消失）
 *   3. DP：沿 s 网格离散化，候选速度 v ∈ {0, 0.2, ..., v_lim}，
 *      cost = ω1·(v-v_target)² + ω2·a²，转移约束 |a| ≤ a_max
 *
 * 关键设计（仿真验证出的坑）：
 *   - 视界动态扩展：ST 图范围 = max(50m, 最近停点+5m) —— 红灯墙 60m 外
 *     也能从 42m 开始压速（等价 C 旧 brake_dist+20 提前触发）
 *   - 全局时间 t0：墙变绿才消失 —— 局部时间会在停稳后重规划时重置为 0，
 *     墙永远"没到变绿时刻" → v=0 闭锁死锁
 *   - 0.95 安全系数：DP 步长取整 + 闭环瞬时速度会略超曲率极限
 */

#ifndef FLOW_ST_GRAPH_H
#define FLOW_ST_GRAPH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 配置（与 Python 仿真常量一致）────────────────────────── */
#define STG_S_HORIZON      50.0   /* 轨迹最小长度 m（视界随停点扩展） */
#define STG_S_RES           1.0   /* s 分辨率 m */
#define STG_A_MAX           4.0   /* 最大减速度 m/s² */
#define STG_A_LAT_MAX       5.0   /* 横向加速度上限 m/s²（与 §8.5 一致） */
#define STG_V_MAX          20.0   /* 限速 m/s */
#define STG_V_CAND_STEP     0.2   /* DP 候选速度离散步长 m/s */
#define STG_W1              1.0   /* cost: (v-v_target)² */
#define STG_W2              2.0   /* cost: a² */
#define STG_CURVE_SAFETY    0.95  /* 曲率约束安全系数 */
#define STG_MAX_GRID        90    /* 视界扩展后的最大格数（50+40 停点） */
#define STG_MAX_OBS          8    /* 本车道 ST 占据障碍物上限 */

/* ST 占据体：s0=起始弧长(ego 系), v=沿向速度, 占据半宽 */
typedef struct {
    double s0;
    double v;
    double half_len;
} StgObstacle;

/* 红灯墙：停止线前 wall_margin 处静止占据；t_red<0 表示一直红（无变绿） */
typedef struct {
    double stopline_s;   /* 停止线位置（ego 系，绝对坐标投影后） */
    double t_red;        /* 变绿时刻（全局时间）；<0 = 一直红 */
    double wall_margin;  /* 墙在停止线前距离（默认 1.0） */
} StgRedWall;

/* 规划输入 */
typedef struct {
    double v0;            /* 当前车速 m/s */
    double v_target;      /* 目标速度（behavior 指令；红灯时也允许 >0，
                             墙/制动约束强制刹停，绿灯后自然恢复 —— 非硬约束） */
    double t0;            /* 当前全局时间 s（墙变绿判定） */
    double stop_s;        /* 最近的硬停点（红灯墙位置，ego 系）；<0=无 */
    /* 曲率剖面：kappa_fn 逐 s 查询，返回 κ(s)；NULL=直道 */
    double (*kappa_fn)(double s, void* user);
    void*  kappa_user;
    int    n_obstacles;   /* 本车道 ST 占据障碍物数量 */
    StgObstacle obstacles[STG_MAX_OBS];
    int    n_walls;
    StgRedWall walls[4];
} StgInput;

/* 规划输出 */
typedef struct {
    double v_out[STG_MAX_GRID];  /* 每 s 格的速度 */
    double t_out[STG_MAX_GRID];  /* 每 s 格的累计时间 */
    int    n;                    /* 格数（视界/S_RES+1） */
    double horizon;              /* 实际视界 m */
} StgResult;

/**
 * 执行 ST 图 + DP 速度规划。
 * 返回 0 成功；v_out[i] 对应 s=i*S_RES 的速度剖面。
 * 调用方取 v_out[min(n-1, int(lookahead_s/S_RES))] 作为目标速度，
 * 其中 lookahead_s = 0.3·v0 + 前视（与 control 0.5s 前视点语义一致）。
 */
int st_graph_plan(const StgInput* in, StgResult* out);

/* 工具：曲率约束速度上限（与 Python curve_v_limit 一致） */
double stg_curve_v_limit(double kappa);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_ST_GRAPH_H */
