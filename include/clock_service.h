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
 */
uint64_t clock_now_us(void);

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

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_SERVICE_H */
