/**
 * test_flowsim_node_plugin.cpp — flowsim_node 插件 dlopen 冒烟测试
 *
 * 验证 Phase 2.1 产出 libflowsim_node.so 能被 dlopen 正确加载：
 *   1. dlopen 成功（node_get_plugin 符号可解析；其它框架符号按需延迟解析）
 *   2. node_get_plugin 符号存在且可调用
 *   3. NodePlugin 描述符字段正确（api_version / name / input_topics / output_topics / 回调非空）
 *
 * 说明：libflowsim_node.so 引用 flowengine_core 的若干符号（net_transport_start、
 * transport_advertise 等），这些符号在真实运行时由 launcher 进程静态链接
 * libflowengine_core.a 提供。本冒烟测试只校验 NodePlugin 描述符 ABI（不调用
 * init/start/stop/cleanup），故采用 RTLD_LAZY 延迟解析未定义符号——node_get_plugin
 * 仅返回静态结构指针，不触发任何外部符号解析。Phase 2.3 回归测试会以 launcher
 * 实际加载方式验证 init/start/stop/cleanup 全流程。
 */

#include <dlfcn.h>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <unistd.h>

/* NodePlugin 结构定义（从 node_plugin.h 复制最小子集，避免依赖整个框架） */
typedef struct NodePlugin {
    unsigned int     api_version;
    const char*      name;
    const char*      version;
    const char*      description;
    const char**     input_topics;
    const char**     output_topics;
    void*            init;
    void*            start;
    void*            stop;
    void*            cleanup;
    void*            health;
    void*            taskbase;
} NodePlugin;

typedef NodePlugin* (*NodeGetPluginFn)(void);

int main(int argc, char** argv) {
    const char* libpath = (argc > 1) ? argv[1]
                        : "/workspace/build/lib/libflowsim_node.so";
    printf("[smoke] dlopen %s\n", libpath);

    /* RTLD_LAZY: 延迟解析符号。libflowsim_node.so 引用 flowengine_core 的符号
     * (net_transport_start/transport_advertise/...)，这些由 launcher 进程在真实
     * 运行时静态链接提供。冒烟测试只校验 NodePlugin 描述符，不调用 init/start/
     * stop/cleanup，故 RTLD_LAZY 足够——node_get_plugin 仅返回静态结构指针。 */
    void* lib = dlopen(libpath, RTLD_LAZY | RTLD_LOCAL);
    if (!lib) {
        fprintf(stderr, "[smoke] dlopen failed: %s\n", dlerror());
        assert(lib && "dlopen failed — see dlerror above");
    }
    printf("[smoke] dlopen OK\n");

    /* node_get_plugin 符号 */
    NodeGetPluginFn get_plugin = (NodeGetPluginFn)dlsym(lib, "node_get_plugin");
    assert(get_plugin && "node_get_plugin symbol not found");
    printf("[smoke] node_get_plugin symbol found\n");

    NodePlugin* p = get_plugin();
    assert(p && "node_get_plugin returned NULL");

    /* 描述符字段校验 */
    assert(p->api_version == 2 && "api_version should be 2 (v2 ABI)");
    assert(p->name && strcmp(p->name, "flowsim") == 0 && "name should be 'flowsim'");
    assert(p->version && "version should not be NULL");
    assert(p->description && "description should not be NULL");
    assert(p->input_topics && p->input_topics[0] && "input_topics should have control/cmd");
    assert(strcmp(p->input_topics[0], "control/cmd") == 0 && "first input should be control/cmd");
    assert(p->input_topics[1] == nullptr && "should have exactly 1 input topic");
    assert(p->output_topics && p->output_topics[0] && "output_topics should not be empty");
    /* 验证 6 个输出 topic：
     *   vehicle/state, road/geometry, road/traffic_lights,
     *   sim/tick, sim/collision, scene/frame (Phase 2.2 新增) */
    int n_out = 0;
    for (int i = 0; p->output_topics[i]; i++) n_out++;
    assert(n_out == 6 && "should have 6 output topics (vehicle/state, road/geometry, "
                          "road/traffic_lights, sim/tick, sim/collision, scene/frame)");
    /* 验证 scene/frame 在输出列表中（Phase 2.2） */
    bool has_scene_frame = false;
    for (int i = 0; p->output_topics[i]; i++) {
        if (strcmp(p->output_topics[i], "scene/frame") == 0) {
            has_scene_frame = true; break;
        }
    }
    assert(has_scene_frame && "scene/frame should be in output topics (Phase 2.2)");
    printf("[smoke] descriptor OK: name=%s ver=%s api=%u inputs=1 outputs=%d\n",
           p->name, p->version, p->api_version, n_out);

    /* 回调非空 */
    assert(p->init && "init callback should not be NULL");
    assert(p->start && "start callback should not be NULL");
    assert(p->stop && "stop callback should not be NULL");
    assert(p->cleanup && "cleanup callback should not be NULL");
    assert(p->health && "health callback should not be NULL");
    printf("[smoke] all callbacks non-null\n");

    /* taskbase 应为 NULL（自管线程模式，非托管模式） */
    assert(p->taskbase == nullptr && "taskbase should be NULL (self-managed mode)");
    printf("[smoke] taskbase=NULL (self-managed mode)\n");

    dlclose(lib);
    printf("[smoke] dlclose OK\n");

    printf("\n[PASS] flowsim_node plugin smoke test — all %d checks passed\n", 14);
    return 0;
}
