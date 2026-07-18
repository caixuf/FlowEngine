# 3D 仪表盘"加载失败"排查与修复（2026-07-18）

> 现象：`demo.sh` 启动后浏览器打开 `http://localhost:8800`，3D 视图（及图表）
> 一直显示 **"Waiting for data... / No data for Ns"**，看起来像 3D 加载失败。
> 实际上 Three.js、模型、Bloom 全部加载正常 —— 真正的故障是 **SSE 数据通道**。

## 结论（TL;DR）

| 层级 | 问题 | 处置 |
|------|------|------|
| 操作层 | 终端里对前台运行的 demo.sh 按了 Ctrl+Z，**整个进程组被挂起**（flowmond `T (stopped)`），8800 端口 accept 队列塞满，所有请求挂死 | 杀掉旧进程组重启 demo；见下文"操作注意" |
| 代码层（真 bug） | monitor 的仪表盘 JSON 是 **cJSON_Print 多行格式化**输出，`handle_sse` 用单个 `data: %s\n\n` 帧原样发送；SSE 协议按行解析，浏览器 EventSource 只收到第一行 **45 字节**，其余行全部丢弃 → 前端 JSON.parse 失败 → 永远"无数据" | `monitor_server.c` 新增 `sse_flatten_payload()`，SSE 发送前把 payload 压平为单行 |

## 根因链（代码层）

```
b012cb6 统一 API 迁移（2026-07 cJSON 重构）
  └─ monitor_node.c:755  cJSON_Print(root)          ← 多行 + \t 缩进（文件写入规范如此）
       └─ /tmp/flow_topology.json（文件桥接）
       └─ dashboard_bridge IPC（SHM 桥接）
            └─ flowmond 缓存 cached_json（仍是多行）
                 └─ handle_sse:  snprintf("data: %s\n\n", buf)   ← 把 30KB 多行 JSON 塞进单个 data: 帧
                      └─ SSE 协议按行分帧：
                         第 1 行 `data: {"source":"live","stale":false,"age_sec":0.0,` → 45 字节消息
                         后续行（\t"self": ...）没有 data: 前缀 → EventSource 全部静默丢弃
                              └─ 前端收到半截 JSON → parse 失败 → 3D "Waiting for data"
```

**为什么难查：** `curl -N /api/stream` 看到的是完整 JSON（curl 不解析 SSE 帧），
demo.sh 的自检也只测 HTTP 200 —— 只有真实浏览器的 EventSource 才会暴露按行丢弃行为。
诊断的关键一步是在无头浏览器里 hook `EventSource`，发现每条消息恰好 45 字节 ——
正好是 `build_cached_dashboard_json` 注入前缀 `{"source":"live","stale":false,"age_sec":0.0,` 的长度。

## 修复

`src/core/monitor_server.c`（handle_sse 之前）：

```c
static void sse_flatten_payload(char* s) {
    char* wr = s;
    for (char* rd = s; *rd; rd++) {
        if (*rd != '\n' && *rd != '\r' && *rd != '\t') *wr++ = *rd;
    }
    *wr = '\0';
}
```

调用点（handle_sse 内）：

```c
build_sse_json(ms, buf, sizeof(buf));
sse_flatten_payload(buf);              /* ← 新增：SSE 发送前压平为单行 */
int fl = snprintf(frame, sizeof(frame), "data: %s\n\n", buf);
```

安全性依据：合法 JSON 的字符串内部不可能出现裸 `\n`/`\r`/`\t`（cJSON 一律转义为
`\n` 等两字符序列），因此这些裸字节必为结构性空白，剥离是无损的。
顺带把 30KB payload 压缩到 ~22KB。

## 验证证据

1. **SSE 实测**（`curl -N` 采样 12s）：修复前每帧浏览器仅获 45 字节；修复后
   **每 500ms 一帧、每帧 ~22KB、`age_sec` 0.0–0.1、`source:"live"`**，满速率无断流。
2. **Puppeteer 无头浏览器**：canvas 正常创建（THREE r128 + Bloom CDN 均加载）、
   EventSource 正常收满帧、"Waiting for data" 遮罩消失、无页面错误。
3. **WebGL readPixels**：紧跟渲染帧读回帧缓冲，采样 31,277 像素 **100% 非黑**
   （中心像素 [178,199,166]）→ 3D 真实在渲染。
   注意：无头截图呈黑屏是 SwiftShader 软件合成的已知伪影，真实浏览器（GPU 合成）显示正常。

## 操作注意（第一现场）

demo.sh 是**前台**脚本（监控循环持续打印）。在终端按 **Ctrl+Z** 会把 demo.sh
及其全部子进程（flow_launcher / flowmond / foxglove_bridge）挂起为 `T (stopped)` ——
8800 端口还在监听但无法 accept，浏览器所有请求挂死，表现与"3D 加载失败"完全相同。

- 想结束 demo：用 **Ctrl+C**（脚本有完整 cleanup trap）。
- 想拿回终端：改用 `bash scripts/demo.sh --no-browser 300 &` 后台运行。
- 快速自检：`ps -o stat -C flowmond` 出现 `T` 即为挂起，`kill -9 -<demo.sh的PID>` 清理后重跑。

## 已知边界与后续建议（按优先级）

1. **64KB 静默截断风险（同类症状会复发）**：`build_cached_dashboard_json` 复制上限
   为 `MONITOR_HTTP_BUF_SIZE`(64KB)；场景/节点继续增长导致 JSON 超限时会截断成非法
   JSON，前端又会回到"无数据"。建议后续在注入时 minify（可再省 ~30%）或超限时丢弃
   本帧保留上一帧完整 JSON，并打 WARN 日志。
2. **更深层修法（可选重构）**：SSE 标准做法是按行 `data: ` 前缀分帧（任意 payload
   均安全）；或让 monitor_node 对桥接通道发 `cJSON_PrintUnformatted`（文件保持
   formatted 供人读）。当前修法选择 SSE 出口单点压平，影响面最小。
3. **微优化**：flatten 目前在每客户端每帧执行（~22KB 单遍，代价极小）；如未来
   客户端数量多，可挪到 `monitor_server_inject_dashboard_json` 注入时一次完成。
4. `favicon.ico` 404：无害噪音，可在 handle_client 加空 204 路由消除。

## 相关文件

| 文件 | 角色 |
|------|------|
| `src/core/monitor_server.c` | 修复点：`sse_flatten_payload()` + handle_sse 调用 |
| `modules/adas_nodes/monitor_node.c:755` | 多行 JSON 源头（`cJSON_Print`，文件写入规范） |
| `src/flowmond.c` | IPC/文件双桥接，注入 cached_json |
| `tools/flowboard/js/app.js:472` | 前端 EventSource 消费端 |
