/**
 * node_context.c — NodeContext 实现
 *
 * 提供统一的节点运行时上下文，封装基础设施访问和生命周期管理。
 * 每个便捷函数都在创建时绑定，避免节点直接操作底层 API。
 */

#include "node_context.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include "health.h"
#include "clock_service.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>

/* ── 内部状态 ────────────────────────────────────────────── */

typedef struct {
    int    task_id;
    bool   health_registered;
    bool   auto_tuner_registered;
    bool   initialized;
} NodeContextInternal;

/* ── 便捷函数实现 ────────────────────────────────────────── */

static int _ctx_subscribe(NodeContext* ctx, const char* topic,
                          NodeMsgCallback cb, void* user) {
    if (!ctx || !ctx->transport || !topic) return ERR_INVALID_PARAM;
    return transport_subscribe(ctx->transport, topic,
                               (MessageCallback)cb, user);
}

static int _ctx_publish(NodeContext* ctx, const char* topic,
                        const uint8_t* data, uint32_t len) {
    if (!ctx || !ctx->transport || !topic) return ERR_INVALID_PARAM;
    return transport_publish(ctx->transport, topic, data, len);
}

static int _ctx_advertise(NodeContext* ctx, const char* topic,
                          uint32_t type_id, uint32_t caps, double max_hz) {
    if (!ctx || !ctx->discovery || !topic) return ERR_INVALID_PARAM;
    return discovery_advertise(ctx->discovery, topic, type_id, caps, max_hz);
}

static void _ctx_heartbeat(NodeContext* ctx) {
    if (!ctx || !ctx->node_name) return;
    health_heartbeat(ctx->node_name);
}

static void _ctx_record_error(NodeContext* ctx, const char* err_msg) {
    if (!ctx || !ctx->node_name) return;
    health_record_error(ctx->node_name, err_msg);
    LOG_ERROR(ctx->node_name, "%s", err_msg ? err_msg : "(unknown error)");
}

static void _ctx_record_latency(NodeContext* ctx, uint64_t latency_us) {
    if (!ctx || !ctx->node_name) return;
    health_record_latency(ctx->node_name, latency_us);
}

static void _ctx_record_stall(NodeContext* ctx) {
    if (!ctx || !ctx->node_name) return;
    health_record_stall(ctx->node_name);
}

/* ── 公开 API ────────────────────────────────────────────── */

int node_context_init(NodeContext* ctx,
                      MessageBus* bus, Transport* transport,
                      DiscoveryManager* discovery, Scheduler* scheduler,
                      TaskBase* task, const NodePlugin* plugin,
                      const char* node_name,
                      double freq_hz, double max_hz) {
    if (!ctx || !node_name) return ERR_INVALID_PARAM;

    memset(ctx, 0, sizeof(*ctx));

    /* 基础设施 */
    ctx->bus       = bus;
    ctx->transport = transport;
    ctx->discovery = discovery;
    ctx->scheduler = scheduler;
    ctx->task      = task;
    ctx->plugin    = plugin;

    /* 配置 */
    ctx->node_name       = node_name;
    ctx->frequency_hz    = freq_hz;
    ctx->max_frequency_hz = max_hz;

    /* 便捷函数绑定 */
    ctx->subscribe      = _ctx_subscribe;
    ctx->publish        = _ctx_publish;
    ctx->advertise      = _ctx_advertise;
    ctx->heartbeat      = _ctx_heartbeat;
    ctx->record_error   = _ctx_record_error;
    ctx->record_latency = _ctx_record_latency;
    ctx->record_stall   = _ctx_record_stall;

    /* 内部状态 */
    NodeContextInternal* internal = (NodeContextInternal*)calloc(1, sizeof(NodeContextInternal));
    if (!internal) return -1;  /* OOM */
    internal->initialized = true;
    ctx->_internal = internal;

    /* 注册到 health 系统 */
    HealthCapability caps = HEALTH_CAP_NONE;
    health_register(node_name, caps);
    internal->health_registered = true;

    return 0;
}

void node_context_destroy(NodeContext* ctx) {
    if (!ctx || !ctx->_internal) return;
    NodeContextInternal* internal = (NodeContextInternal*)ctx->_internal;
    free(internal);
    ctx->_internal = NULL;
}