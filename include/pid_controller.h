#pragma once
#include "task_interface.h"

/** 获取 PID 控制器内部的车辆遥测状态。
 *  返回 -1 表示 task 无效，0 成功。 */
int pid_control_get_vehicle_state(TaskBase* task,
                                  double* speed, double* target,
                                  double* throttle, double* brake,
                                  double* x, double* y);
