#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

/**
 * @file traffic_light.h
 * @brief 共享红绿灯相位状态机（纯函数，无状态无副作用）
 *
 * 仿照 road_geometry.h 的"单一数据源共享"模式：sim_world / planning /
 * monitor 三方 include 同一份计算，保证仿真发布、规划决策、可视化对
 * "当前是什么灯"的理解完全一致。
 *
 * 相位循环：GREEN → YELLOW → RED → GREEN → ...
 *   green_s    绿灯时长（s）
 *   yellow_s   黄灯时长（s）
 *   red_s      红灯时长（s）
 *   phase_offset_s  初始相位偏移（s），用于让多个灯错峰（默认 0）
 *
 * 总周期 T = green_s + yellow_s + red_s。给定仿真时刻 t（秒）：
 *   t' = (t + phase_offset_s) mod T
 *   t' ∈ [0, green_s)            → GREEN,  remain = green_s - t'
 *   t' ∈ [green_s, green_s+yellow_s) → YELLOW, remain = green_s+yellow_s - t'
 *   t' ∈ [green_s+yellow_s, T)       → RED,    remain = T - t'
 *
 * 任一时长 <= 0 时视为"无红绿灯"（恒返回 GREEN），与不含 traffic_lights
 * 字段的既有场景完全兼容（向后兼容，默认零风险）。
 */

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 灯状态枚举 ─────────────────────────────────────────── */

typedef enum {
    TL_GREEN  = 0,
    TL_YELLOW = 1,
    TL_RED    = 2
} TrafficLightState;

/* ── 相位计算结果 ───────────────────────────────────────── */

typedef struct {
    TrafficLightState state;   /**< 当前灯色 */
    double            remain_s;/**< 距下次切换的剩余时间（s） */
} TrafficLightPhase;

/**
 * 计算给定时刻 t 的红绿灯相位。
 *
 * @param t              仿真时刻（秒，从场景启动起算）
 * @param green_s        绿灯时长（s，<=0 视为无灯，恒返回 GREEN）
 * @param yellow_s       黄灯时长（s）
 * @param red_s          红灯时长（s）
 * @param phase_offset_s 初始相位偏移（s，默认 0）
 * @return TrafficLightPhase {state, remain_s}
 */
static inline TrafficLightPhase traffic_light_phase_at(double t,
                                                        double green_s,
                                                        double yellow_s,
                                                        double red_s,
                                                        double phase_offset_s) {
    TrafficLightPhase ph = { TL_GREEN, green_s };
    /* 无效时长 → 恒绿灯（等价于无红绿灯，向后兼容） */
    if (green_s <= 0.0 || yellow_s <= 0.0 || red_s <= 0.0) {
        ph.remain_s = 1e9;  /* "永远绿灯" */
        return ph;
    }

    double T = green_s + yellow_s + red_s;
    double tp = t + phase_offset_s;
    /* fmod 可能返回负数（t 为负时），修正到 [0, T) */
    double tp_mod = fmod(tp, T);
    if (tp_mod < 0.0) tp_mod += T;

    if (tp_mod < green_s) {
        ph.state    = TL_GREEN;
        ph.remain_s = green_s - tp_mod;
    } else if (tp_mod < green_s + yellow_s) {
        ph.state    = TL_YELLOW;
        ph.remain_s = green_s + yellow_s - tp_mod;
    } else {
        ph.state    = TL_RED;
        ph.remain_s = T - tp_mod;
    }
    return ph;
}

/**
 * 灯状态 → 字符串（用于 JSON 序列化 / 日志）。
 */
static inline const char* traffic_light_state_str(TrafficLightState s) {
    switch (s) {
        case TL_GREEN:  return "green";
        case TL_YELLOW: return "yellow";
        case TL_RED:    return "red";
        default:        return "green";
    }
}

#ifdef __cplusplus
}
#endif

#endif /* TRAFFIC_LIGHT_H */
