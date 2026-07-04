#ifndef ALGORITHM_PLUGIN_H
#define ALGORITHM_PLUGIN_H

/**
 * @file algorithm_plugin.h
 * @brief 算法插件接口 — 第三方算法接入标准
 *
 * FlowEngine 定位：中间件框架（调度、通信、状态机、监控）
 * 算法定位：    第三方库/开源项目（感知数学、融合数学、规划数学、控制数学）
 *
 * 接入方式：实现 AlgorithmInterface → 编译为 .so → launcher dlopen 加载
 *
 * 典型集成：
 *   感知:  OpenCV + TensorRT → PerceptionPlugin
 *   融合:  Eigen + Kalman → FusionPlugin
 *   规划:  OMPL / Apollo Planning → PlanningPlugin
 *   控制:  PID / MPC → ControlPlugin
 *   定位:  GTSAM / Ceres → LocalizationPlugin
 *
 * 状态机编排：
 *   驾驶模式 (NA→ACC→CP→NP→NOA) → 激活/停用对应算法插件
 *   ACC_ACTIVE  → 激活 ACC_LongitudinalControl + PerceptionPlugin
 *   CP_ACTIVE   → 激活 SteeringControl + LaneDetection
 *   NP_ACTIVE   → 激活 MotionPlanning + HDMapPlugin
 */

#include "task_interface.h"
#include "message_bus.h"
#include "state_machine.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 算法插件类型 ────────────────────────────────────────── */

typedef enum {
    ALGO_PERCEPTION    = 0,  /**< 感知: 目标检测/跟踪/分类 */
    ALGO_FUSION        = 1,  /**< 融合: 多传感器数据融合 */
    ALGO_LOCALIZATION  = 2,  /**< 定位: 自车状态估计 */
    ALGO_PREDICTION    = 3,  /**< 预测: 障碍物轨迹预测 */
    ALGO_PLANNING      = 4,  /**< 规划: 路径/速度规划 */
    ALGO_CONTROL       = 5,  /**< 控制: 纵向/横向控制 */
    ALGO_CUSTOM        = 99, /**< 自定义算法 */
} AlgorithmType;

/* ── 算法能力描述 ────────────────────────────────────────── */

typedef struct {
    const char* name;            /**< 算法名称 (如 "EKF_Fusion") */
    const char* version;         /**< 版本号 */
    AlgorithmType type;          /**< 算法类型 */
    const char* description;     /**< 一句话描述 */
    const char** dependencies;   /**< 依赖的第三方库 (NULL-terminated) */
    const char** input_topics;   /**< 输入的 topic (NULL-terminated) */
    const char** output_topics;  /**< 输出的 topic (NULL-terminated) */
} AlgorithmInfo;

/* ── 算法生命周期回调 ────────────────────────────────────── */

/**
 * 算法插件接口。
 *
 * 每个 .so 导出两个符号:
 *   AlgorithmInterface* get_algorithm_interface(void);
 *   const char*         get_algorithm_version(void);
 */
typedef struct {
    /** 获取算法元信息 */
    const AlgorithmInfo* (*get_info)(void);

    /**
     * 初始化算法。
     * @param bus    消息总线（用于 publish/subscribe）
     * @param config 算法配置 (JSON string)
     * @param params 参数 (key=value pairs)
     * @return 算法实例句柄（opaque pointer），失败返回 NULL
     */
    void* (*initialize)(MessageBus* bus, const char* config, const char** params);

    /**
     * 执行一次算法迭代。
     * @param handle      initialize 返回的句柄
     * @param input_data  输入数据 (由框架从订阅 topic 中收集)
     * @return 0 成功, 负数=错误
     */
    int (*process)(void* handle, const void* input_data);

    /**
     * 获取算法内部状态（用于监控/可视化）。
     * @return JSON 字符串, 调用者不需要 free
     */
    const char* (*get_state_json)(void* handle);

    /**
     * 更新运行时参数。
     * @param key   参数名
     * @param value 新值
     * @return 0 成功, -1 参数不存在或值非法
     */
    int (*set_param)(void* handle, const char* key, const char* value);

    /**
     * 获取当前驾驶模式（用于状态机联动）。
     * 算法可以报告自己推荐的模式切换。
     */
    StateId (*get_recommended_mode)(void* handle);

    /** 清理资源 */
    void (*destroy)(void* handle);
} AlgorithmInterface;

/* ── 状态机 → 算法联动 ───────────────────────────────────── */

/**
 * 根据驾驶模式激活/停用算法插件。
 *
 * 模式切换规则:
 *   NA  → 停用所有算法（待机）
 *   ACC → 激活: LongitudinalControl + PerceptionPlugin(障碍物)
 *   CP  → 激活: ACC集合 + SteeringControl + LaneDetection
 *   NP  → 激活: CP集合 + MotionPlanning + Prediction
 *   NOA → 激活: NP集合 + HDMapLocalization
 *
 * @param sm         状态机
 * @param mode       当前驾驶模式
 * @param plugins    算法插件表 (NULL-terminated)
 */
void algorithm_activate_for_mode(ReflectiveStateMachine* sm,
                                 StateId mode,
                                 AlgorithmInterface** plugins);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_PLUGIN_H */
