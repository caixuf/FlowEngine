/**
 * fusion_cpp.cpp — FusionNodeCpp 协程基类 (FlowEngine Phase 4B)
 */

#include "fusion.h"
#include "coroutine_task.h"
#include <memory>
#include <vector>
#include <string>
#include <cstring>

struct FusionNodeCpp::Impl {
    FusionNode*           c_node;
    MessageBus*           bus;
    FusionPolicy          policy;
    std::vector<std::string> input_topics;
};

FusionNodeCpp::FusionNodeCpp(MessageBus* bus, const FusionPolicy& policy)
    : impl_(std::make_unique<Impl>()) {
    impl_->bus    = bus;
    impl_->policy = policy;
    impl_->c_node = nullptr;
}

FusionNodeCpp::~FusionNodeCpp() {
    if (impl_->c_node) {
        fusion_node_stop(impl_->c_node);
        fusion_node_destroy(impl_->c_node);
    }
}

void FusionNodeCpp::AddSensorInput(const std::string& topic, uint32_t type_id,
                                   uint32_t buffer_capacity) {
    impl_->input_topics.push_back(topic);

    if (!impl_->c_node) {
        impl_->c_node = fusion_node_create("fusion_cpp", impl_->bus, &impl_->policy);
    }
    fusion_node_add_input(impl_->c_node, topic.c_str(), type_id, buffer_capacity);
}

void FusionNodeCpp::SetOutputTopic(const std::string& topic, uint32_t type_id) {
    if (!impl_->c_node) {
        impl_->c_node = fusion_node_create("fusion_cpp", impl_->bus, &impl_->policy);
    }
    fusion_node_set_output(impl_->c_node, topic.c_str(), type_id);
}

void FusionNodeCpp::Start() {
    if (impl_->c_node) {
        fusion_node_start(impl_->c_node);
    }
    /* Also start the coroutine on RtExecutor */
    flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1, .idle_sleep_us=200 }};
    g_node_exec = &ex;
    ex.spawn(run(), "fusion_cpp");
    while (!should_stop()) ex.run();
    ex.shutdown();
    g_node_exec = nullptr;
}

Task FusionNodeCpp::run() {
    /* The fusion processing is handled by callbacks from the message bus.
     * This coroutine serves as a lifecycle monitor — if stop is requested,
     * we clean up and exit. */

    while (!should_stop()) {
        /* Wait for messages via BusChannel or similar pattern.
         * For now, we yield periodically to check the stop flag. */
        co_await std::suspend_always{};
    }

    co_return;
}

/* ── Fusion output helper (can be used from Fuse()) ──────── */

void fusion_publish_frame(const SyncedFrame& frame, MessageBus* bus,
                          const char* topic, uint32_t type_id,
                          const void* fused_data, size_t fused_size) {
    if (!bus || !topic) return;

    Message msg;
    msg_init_typed(&msg, topic, "fusion", type_id, 1, fused_data, fused_size);
    msg.timestamp_us = frame.reference_ts;
    message_bus_publish(bus, topic, "fusion", fused_data, (uint32_t)fused_size);
}
