#ifndef FAKE_CONTROL_TASK_H
#define FAKE_CONTROL_TASK_H

/**
 * @file fake_control_task.h
 * @brief 假控制节点公共 API
 *
 * 该任务模拟自动驾驶控制模块：
 *   订阅：perception/obstacles、perception/ego_state
 *   发布：control/cmd  (每收到一帧障碍物数据触发一次)
 *
 * 内置决策策略：
 *   dist >= 50m : 正常巡航  throttle=0.4
 *   25m~50m     : 松油门减速 throttle=0~0.3
 *   10m~25m     : 制动      brake=0~0.8
 *   < 10m       : 紧急制动  brake=1.0, emergency_stop=true
 */

#include "task_interface.h"
#include "message_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FakeControlTask FakeControlTask;

/**
 * 创建假控制任务（同时自动订阅感知话题）
 * @param config  任务配置
 * @param bus     关联的消息总线
 * @return 任务指针，失败返回 NULL
 */
FakeControlTask* fake_control_task_create(const TaskConfig* config, MessageBus* bus);

/** 销毁假控制任务 */
void fake_control_task_destroy(FakeControlTask* task);

/** 获取基类指针（用于 task_start / task_stop）*/
TaskBase* fake_control_task_get_base(FakeControlTask* task);

#ifdef __cplusplus
}
#endif

#endif /* FAKE_CONTROL_TASK_H */
