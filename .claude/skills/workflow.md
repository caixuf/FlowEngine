# Workflow — FlowEngine 开发工作流

完整开发流程：设计 → 执行 → 测试 → 迭代 → 清理 → 文档。

## 一、设计（Design）

在写任何代码之前，先明确：

1. **要解决什么问题** — 一句话描述
2. **影响哪些文件** — 列出改动范围
3. **对现有功能的影响** — 向后兼容？破坏性变更？
4. **验证方法** — 如何证明改动生效

复杂任务使用 `EnterPlanMode` 写实施计划，经确认后再执行。

## 二、执行（Implementation）

### 2.1 必须使用项目基础设施

以下 API 是唯一合法入口，**禁止绕过**：

| 需求 | 使用 | 禁止 |
|------|------|------|
| JSON 序列化/反序列化 | `cjson/cJSON.h` | snprintf 手拼 JSON, strstr+sscanf 手写解析 |
| 时间 | `clock_service.h` → `clock_now_us()` | 裸 `clock_gettime()` |
| 参数解析 | `cJSON_Parse(params_json)` | `strstr(params_json, "key")` |
| 错误码 | `error_codes.h` → `ERR_*` | `return -1` |
| 日志 | `LOG_INFO/WARN/ERROR(module, fmt, ...)` | `printf/fprintf` |

### 2.2 代码风格

- 新代码匹配周围代码的注释密度、命名风格、缩进
- C 文件用 `//` 单行注释，多行文档用 `/* */`
- 结构体字段对齐，指针声明 `type* ptr` 星号靠类型

### 2.3 场景文件

- 新场景用 `road_network` 格式（edges + junctions），NPC 用 `s/l/segment_id`
- 旧格式 `x/y` 逐步淘汰，不再新增

## 三、测试（Testing）

### 3.1 端到端验证（必须）

```bash
# 跑 demo 验证全链路
bash scripts/demo.sh --scenario scenarios/<target>.json --no-browser <duration>

# 运行时抓拓扑数据
cp /tmp/flow_topology.json /tmp/verify_<name>.json
```

### 3.2 回归测试

```bash
cd build && ctest -LE "benchmark|manual|stability|integration" --output-on-failure -j$(nproc)
```

### 3.3 使用 /verify skill

改完 pipeline 链路节点后，跑 `/verify` 做系统性验证。

## 四、持续迭代（Iterate）

1. 验证暴露问题 → 分析根因 → 修复 → 再验证
2. 每轮迭代只改一个关注点，避免大杂烩 commit
3. 发现的问题即使不修也要记录到规划文档

## 五、清理（Cleanup）

改动完成后检查：

- [ ] 是否遗留了调试用的 `printf`/`fprintf`？
- [ ] 是否新增了不再需要的兼容代码路径？
- [ ] 旧方案的文件/函数是否可删除？
- [ ] 是否有重复代码可以提取复用？
- [ ] 用户明确说"不改旧格式"时，是否真的删了旧路径？

使用 `/simplify` skill 自动检查冗余。

## 六、更新文档（Document）

- [ ] 规划文档（`docs/NOA_SCENARIO_PLAN.md` 等）的状态和进度是否更新？
- [ ] `CLAUDE.md` 是否需要补充新发现？
- [ ] `memory/` 是否需要记录关键决策？
- [ ] git commit message 是否描述了"做了什么、为什么、影响范围"？

## 七、提交（Commit）

```bash
git add -A
git commit -m "<type>: <简短描述>

<详细说明改了什么、为什么、怎么验证的>

Co-Authored-By: Claude <noreply@anthropic.com>"
```

Commit type: `feat` / `fix` / `docs` / `refactor` / `test` / `chore`
