#ifndef FLOWENGINE_H
#define FLOWENGINE_H

/**
 * @file flowengine.h
 * @brief FlowEngine 单一入口汇总头（umbrella header）。
 *
 * 第三方集成时只需 `#include "flowengine.h"` 即可获得核心公共 API，
 * 无需逐个引入内部头文件。同时提供库版本与插件 ABI 版本宏，便于做
 * 编译期/运行期兼容性检查。
 *
 * 用法:
 * @code
 *   #include "flowengine.h"
 *   #if FLOWENGINE_VERSION < FLOWENGINE_VERSION_ENCODE(1, 0, 0)
 *   #  error "FlowEngine >= 1.0.0 required"
 *   #endif
 * @endcode
 */

/* ── 版本号 ──────────────────────────────────────────────────
 * 与 CMake `project(FlowEngine VERSION x.y.z)` 保持一致。 */
#define FLOWENGINE_VERSION_MAJOR 1
#define FLOWENGINE_VERSION_MINOR 0
#define FLOWENGINE_VERSION_PATCH 0

/** 将 (major, minor, patch) 编码为单个可比较整数。 */
#define FLOWENGINE_VERSION_ENCODE(major, minor, patch) \
    ((major) * 10000 + (minor) * 100 + (patch))

/** 当前库版本的可比较整数形式。 */
#define FLOWENGINE_VERSION \
    FLOWENGINE_VERSION_ENCODE(FLOWENGINE_VERSION_MAJOR, \
                              FLOWENGINE_VERSION_MINOR, \
                              FLOWENGINE_VERSION_PATCH)

/** 人类可读版本字符串。 */
#define FLOWENGINE_VERSION_STRING "1.0.0"

/* ── 核心公共 API ────────────────────────────────────────────── */
#include "error_codes.h"
#include "logger.h"
#include "message_bus.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include "serializer.h"
#include "state_machine.h"
#include "fusion.h"
#include "clock_service.h"
#include "bag.h"

/* ── 插件开发接口（含 NODE_PLUGIN_API_VERSION） ── */
#include "node_plugin.h"
#include "task_interface.h"

#endif /* FLOWENGINE_H */
