#ifndef PARAM_BRIDGE_H
#define PARAM_BRIDGE_H

/**
 * @file param_bridge.h
 * @brief 跨进程参数读写通道 —— flowctl ↔ 运行中的 flow_launcher
 *
 * param_registry 是进程内的。此前 `flowctl param set control.pid_kp 1.2`
 * 只改了 flowctl 自己那份拷贝，跑着的 flow_launcher 一无所知 —— 想调参
 * 只能改 pipeline.json 重启进程，"边跑边调"做不到。本模块补上这条通道。
 *
 * 架构：
 *   flowctl ── LIST/GET/SET 行协议 ──► AF_UNIX ──► flow_launcher 内的服务线程
 *                                                  └─► param_set_*() → 节点逐帧
 *                                                      param_get_float() 读到新值
 *
 * 为什么用 AF_UNIX 而不复用 ipc_channel（stats/dashboard 桥接那套 shm 环）：
 * 那是单向广播，且要求 publisher 先于 subscriber 启动。参数调用是请求/响应，
 * 客户端后启动、发一条收一条就退出，socket 的连接语义天然匹配。
 *
 * 为什么服务端放在 flow_launcher 而不是各节点进程：
 * 节点是 dlopen 进来的 .so，其 param_get_float / param_register_* 经符号
 * interposition 绑定到主程序那份实现（已实测确认），所以 launcher 的 registry
 * 就是全部节点参数的唯一实例 —— 一个服务端即可覆盖全部节点。
 *
 * 线协议（文本行，便于 nc/socat 手工调试）：
 *   请求:  "LIST\n"                        响应: "OK <n>\n<name> <type> <value> <min> <max>\n"×n
 *          "GET <name>\n"                  响应: "OK <value>\n"
 *          "SET <name> <value>\n"          响应: "OK <value>\n"
 *   失败:  "ERR <errno> <message>\n"
 *
 * 用法（服务端，flow_launcher）：
 *   ParamBridgeServer* s = param_bridge_server_start(NULL);
 *   ... 运行 ...
 *   param_bridge_server_stop(s);
 *
 * 用法（客户端，flowctl）：
 *   char out[256];
 *   if (param_bridge_client_request("GET control.pid_kp", out, sizeof(out)) == ERR_OK)
 *       puts(out);
 */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 默认 socket 路径。可用环境变量 FLOW_PARAM_SOCK 覆盖（多实例并行调试用）。 */
#define PARAM_BRIDGE_DEFAULT_SOCK  "/tmp/flow_param.sock"
#define PARAM_BRIDGE_SOCK_ENV      "FLOW_PARAM_SOCK"

/** 单条请求/响应上限。LIST 响应按需分块写，不受此限。 */
#define PARAM_BRIDGE_MAX_LINE      512

typedef struct ParamBridgeServer ParamBridgeServer;

/**
 * 启动参数服务线程（非阻塞，立即返回）。
 *
 * @param sock_path  socket 路径；NULL = 取 $FLOW_PARAM_SOCK，再回退到默认值
 * @return 句柄，失败返回 NULL（失败不致命，调用方应继续运行，仅失去远程调参能力）
 */
ParamBridgeServer* param_bridge_server_start(const char* sock_path);

/**
 * 停止服务线程并 unlink socket 文件。传 NULL 是安全的空操作。
 */
void param_bridge_server_stop(ParamBridgeServer* s);

/**
 * 发一条请求、收一行响应（客户端，短连接）。
 *
 * @param request   请求行，不含结尾换行
 * @param out       响应缓冲；成功时写入 "OK " 之后的内容，失败时写入错误说明
 * @param out_size  out 容量
 * @return ERR_OK 成功；ERR_NOT_FOUND 服务端未运行（launcher 没起来）；
 *         ERR_IO 通信失败；ERR_INVALID_PARAM 服务端拒绝（越界/未知参数等）
 */
int param_bridge_client_request(const char* request, char* out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_BRIDGE_H */
