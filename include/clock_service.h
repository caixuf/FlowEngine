#ifndef CLOCK_SERVICE_H
#define CLOCK_SERVICE_H

/**
 * @file clock_service.h
 * @brief 统一时钟服务（纯头文件 + 单 .c 实现）
 *
 * 正常模式：返回 CLOCK_MONOTONIC 单调时间（微秒）。
 * 仿真模式：由外部注入时间戳（bag 回放时使用录制时间）。
 *
 * 用法：
 *   // 获取当前时间（替代各处散落的 clock_gettime）
 *   uint64_t now = clock_now_us();
 *
 *   // 切换到仿真时间（bag 回放）
 *   clock_set_sim_mode(true);
 *   clock_set_sim_time(recorded_timestamp_us);
 *   // ... replay loop ...
 *   clock_set_sim_mode(false);   // 恢复真实时间
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 获取当前时间（微秒）
 * 仿真模式下返回 clock_set_sim_time() 最后设置的值。
 * 正常模式返回 CLOCK_MONOTONIC 单调时间。
 */
uint64_t clock_now_us(void);

/**
 * monotonic_us() — 已废弃的过渡别名。
 *
 * 历史上各模块各自实现 `static uint64_t monotonic_us(void) {
 * clock_gettime(CLOCK_MONOTONIC, &ts); ... }`，绕过了仿真时钟，导致
 * bag 回放时时间不一致。clock_service 成为唯一时间源后，本函数作为
 * 迁移桥梁：保留旧名字，但内部直接转发到 clock_now_us()，编译期触发
 * 废弃告警，强制调用方改用 clock_now_us()。
 *
 * 严禁新增调用；现有调用应替换为 clock_now_us()。
 */
uint64_t monotonic_us(void)
    __attribute__((deprecated("clock_service is the only time source; use clock_now_us()")));

/**
 * 获取当前绝对时间（微秒，CLOCK_REALTIME）
 * 不受仿真模式影响，始终返回真实墙钟时间。
 * 用于训练样本时间戳、仪表盘时间戳等需要绝对时间的场景。
 */
uint64_t clock_now_realtime_us(void);

/**
 * 启用/禁用仿真时间模式
 * @param enable  true = 使用仿真时间；false = 使用真实 CLOCK_MONOTONIC
 */
void clock_set_sim_mode(bool enable);

/**
 * 设置仿真时间（仅在 sim_mode=true 时生效）
 * @param timestamp_us  目标时间（微秒，与录制时的 CLOCK_MONOTONIC 对齐）
 */
void clock_set_sim_time(uint64_t timestamp_us);

/**
 * 查询当前是否处于仿真时间模式
 */
bool clock_is_sim_mode(void);

/**
 * 在仿真模式下将逻辑时钟推进 delta_us 微秒。
 * 等价于 clock_set_sim_time(clock_now_us() + delta_us)。
 * 非仿真模式下调用本函数为空操作。
 *
 * 使用方式（仿真主循环）：
 *   clock_set_sim_mode(true);
 *   clock_set_sim_time(0);
 *   while (running) {
 *       // ... 执行一步仿真 ...
 *       clock_advance_us(10000);   // 逻辑时间 +10 ms
 *   }
 */
void clock_advance_us(uint64_t delta_us);

/**
 * 获取仿真步长（微秒）— 由 sim_world_node 初始化时配置。
 * 非仿真模式下返回 0。
 */
uint64_t clock_get_step_us(void);

/**
 * 设置仿真步长（微秒），供 sim_world_node 在 init 时调用。
 * @param step_us  每个 tick 推进的逻辑时间（例如 50000 = 50 ms @20 Hz）
 */
void clock_set_step_us(uint64_t step_us);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_SERVICE_H */
