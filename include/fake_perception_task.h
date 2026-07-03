#ifndef FAKE_PERCEPTION_TASK_H
#define FAKE_PERCEPTION_TASK_H

/**
 * @file fake_perception_task.h
 * @brief 假感知节点公共 API
 *
 * 该任务模拟自动驾驶感知模块，向消息总线发布：
 *   sensor/lidar         — LidarFrame   (10 Hz)
 *   sensor/gps           — GpsData      (5  Hz)
 *   perception/obstacles — ObstacleList (10 Hz)
 *   perception/ego_state — EgoState     (10 Hz)
 *
 * 内置场景：
 *   t=0.5s  前方 50m 出现一辆车，以 5m/s 缓慢驶近
 *   t=3.0s  左侧出现行人向路中横穿（持续约 4 秒）
 */

#include "task_interface.h"
#include "message_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FakePerceptionTask FakePerceptionTask;

/**
 * 创建假感知任务
 * @param config  任务配置（name / priority 等）
 * @param bus     关联的消息总线
 * @return 任务指针，失败返回 NULL
 */
FakePerceptionTask* fake_perception_task_create(const TaskConfig* config, MessageBus* bus);

/** 销毁假感知任务 */
void fake_perception_task_destroy(FakePerceptionTask* task);

/** 获取基类指针（用于 task_start / task_stop）*/
TaskBase* fake_perception_task_get_base(FakePerceptionTask* task);

#ifdef __cplusplus
}
#endif

#endif /* FAKE_PERCEPTION_TASK_H */
