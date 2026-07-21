# 「减法纪律」执行计划 — 让重构铁律从软清单升级为硬规范

## 一、问题陈述

仓库长期累积「新旧并存」现状（已通过 Phase 1 探索确认）：

| 维度 | 现状 | 证据 |
|------|------|------|
| 训练脚本 | 已收敛到 2 份 | `tools/train_demo_model.py`（顶层入口）、`tools/train_e2e/train.py`（canonical）；`tools/train/train.py` 已删除 |
| 学习闭环文档 | 已收敛到 2 份 | `docs/LEARNING_LOOP.md`、`skills/13_e2e_learning_loop.md`；`docs/E2E_LEARNING_V3_PLAN.md` 已删除 |
| CLAUDE.md 漂移 | 索引写 `.c` 实际 `.cpp` | line 49 `inference_node.c` → 实际 `inference_node.cpp`（已确认） |
| 3D 可视化 | 已收敛到 `vis/`（成功案例） | 4 代→1 代，依赖 `tools/flowboard/js/vis/main.js` choke point |
| 场景文件 | 已清到 2 份 | 删 16 份 + zhongkai_road_full.json 等 |

**根因**（你已诊断）：现有「清理」是软清单（自评、不阻塞、无产物），跟 cJSON 那条硬规范（违反不合并、可 grep）不在一个档次。AI 顺着「加法比减法安全」默认走。

**目标**：把删旧从 agent 自觉 → 自动化硬约束（commit 必填 + 文件系统可 grep + choke point 结构强制）。

## 二、当前状态分析（来自 Phase 1 探索）

### 2.1 CLAUDE.md（[file:///home/caixuf/code/FlowEngine/CLAUDE.md](file:///home/caixuf/code/FlowEngine/CLAUDE.md)）
- 第 80-146 行「编码规范」是**有牙齿的档**（line 146 明确「违反以上规范的代码不会被合并」）
- 第 23-56 行「关键文件表」存在 `.c` vs `.cpp` 漂移（line 49 写 inference_node.c，实际 .cpp）
- **无任何"重构/替代 → 必须删旧"规则**

### 2.2 workflow.md（[file:///home/caixuf/code/FlowEngine/.claude/skills/workflow.md](file:///home/caixuf/code/FlowEngine/.claude/skills/workflow.md)）
- 第五节「清理」(line 69-79) 是 checklist 软步骤，无产出物
- 位于"测试→迭代→清理→文档"末尾，无 gate 拦
- **commit 第七节 (line 88-98) 无 Removed 必填字段**

### 2.3 skills/13_e2e_learning_loop.md（[file:///home/caixuf/code/FlowEngine/skills/13_e2e_learning_loop.md](file:///home/caixuf/code/FlowEngine/skills/13_e2e_learning_loop.md)）
- 13 号 skill 描述学习闭环，未提 `/simplify` 联动
- **整个 skills/ 目录无 `SKILL.md` 索引文件**（Glob 找不到）

### 2.4 训练脚本分布（已清理）
```
tools/train_demo_model.py            # 顶层一键入口，调度 train_e2e/
tools/train_e2e/train.py              # 当前 canonical（tiny-MLP + PyTorch）
tools/train_e2e/torch_train.py        # PyTorch artifact（已并入 train.py）
tools/train_e2e/temporal_train.py     # 时序模型（已并入 train.py）
tools/train/train.py                  # 已删除（deprecated，被 train_e2e/train.py 取代）
```

## 三、具体改动（4 个文件）

### 3.1 [CLAUDE.md](file:///home/caixuf/code/FlowEngine/CLAUDE.md) — 三处改动

#### 改动 ①（line 146 之前）：新增「重构/替代 → 同一 commit 删旧」硬规范
插入位置：第 144 行 `### 错误码 → error_codes.h` 之后，第 146 行 `### 违反以上规范的代码不会被合并` 之前。

插入内容：
```markdown
### 重构/替代 → 同一 commit 删旧

- 任何「重写 / v2 / 换实现」类改动，新实现与被替代的旧实现必须在**同一个 commit** 内，旧的物理删除
- ❌ 禁止 `_v2` / `_v3` / `_new` / `_old` / `_bak` 后缀文件并存超过 1 个 commit
- ❌ 禁止新增与现有入口同名不同实现的第二份脚本（如已有 `tools/train/train.py`，禁止再添加第二个 `tools/train_e2e/train.py` 做相同事情——必须删旧）
- ✅ 确实要暂留旧路径 → 文件头第一行加注释：
  ```python
  # @deprecated superseded-by=tools/train_e2e/train.py remove-by=2026-08-01
  ```
  ```c
  // @deprecated superseded-by=modules/adas_nodes/inference_node.cpp remove-by=2026-08-01
  ```
  无 `remove-by` 日期视为违规。

可 grep 性：
- `git diff HEAD~1 --name-only | grep -E '_v[0-9]+\.|_new\.|_old\.|_bak\.'` 出现多个同名前缀 → 违规
- `git grep -l "@deprecated" -- ':!*.md' | xargs -I{} grep -L "remove-by" {}` → 缺日期违规
- 任何重构类 commit 的 `git show --stat HEAD` 必须显示 `D <旧文件>` 至少一行
```

#### 改动 ②：修正 line 49 漂移（顺手修）
```diff
-| `modules/adas_nodes/inference_node.c` | tiny-MLP 影子推理（Learning Loop Stage 2） |
+| `modules/adas_nodes/inference_node.cpp` | tiny-MLP 影子推理（Learning Loop Stage 2） |
```

#### 改动 ③：第 6 行补一句"清理是硬规范"
在第 6 行「改完代码后必跑」后面加：
```markdown
> **重构类改动** commit body 必含 `Removed:` 段，详见 `.claude/skills/workflow.md` 第五节。
```

### 3.2 [.claude/skills/workflow.md](file:///home/caixuf/code/FlowEngine/.claude/skills/workflow.md) — 第七节 commit 模板加 Removed 必填

改动位置：line 88-98 commit 模板。

```diff
-## 七、提交（Commit）
-
-```bash
-git add -A
-git commit -m "<type>: <简短描述>
+## 七、提交（Commit）— Refactor 类必含 `Removed:` 段
+
+```bash
+git add -A
+git commit -m "<type>: <简短描述>
 
 <详细说明改什么、为什么、怎么验证的>
 
+Removed:
+- <被物理删除的旧文件/函数>  (superseded-by <新实现>)
+
 Co-Authored-By: Claude <noreply@anthropic.com>"
 ```
 
+Removed: 段规则：
+- 重构/替代类（commit type 含 `refactor` 或含 `v2`/`v3`/`rewrite`/`replace`）**必填**：
+  - 至少一行 `Removed: - <path> (superseded-by <new>)`
+  - 纯新增无替代 → 必须显式写 `Removed: none (纯新增, 无替代)` 并给理由
+  - 留空 → review 打回
+- 纯 `feat` / `fix` / `chore` / `docs` / `test` → 段可省略
+
+review 检查：
+  ```
+  git log --format=%B -1 HEAD | grep -E '^Removed:|^Co-Authored' | head -1
+  ```
+  refactor 类若只有 `Co-Authored` 而无 `Removed` → 打回。
```

### 3.3 [skills/13_e2e_learning_loop.md](file:///home/caixuf/code/FlowEngine/skills/13_e2e_learning_loop.md) — 增「canonical 入口」声明

改动位置：第 17-26 行「核心路径」表之后插入：

```markdown
## Canonical 入口（其他脚本禁止并存）

| 用途 | canonical 路径 | 替代关系 |
|------|----------------|----------|
| 端到端训练 | `tools/train_e2e/train.py` | supersedes `tools/train/train.py`（sklearn 版本） |
| PyTorch artifact | `tools/train_e2e/torch_train.py` | — |
| 时序模型 | `tools/train_e2e/temporal_train.py` | — |
| demo 训练 | `tools/train_e2e/train.py` (同端到端) | supersedes `tools/train_demo_model.py`（独立 demo 训练，已被端到端包含） |
| C 端推理 | `modules/adas_nodes/inference_node.cpp` | — |

> 任何新训练入口必须在 canonical 目录 `tools/train_e2e/` 下。`tools/train/` 与 `tools/train_demo_model.py` 已 deprecated，按 CLAUDE.md 重构铁律处理。
```

### 3.4 [tools/train/train.py](file:///home/caixuf/code/FlowEngine/tools/train/train.py) + [tools/train_demo_model.py](file:///home/caixuf/code/FlowEngine/tools/train_demo_model.py) — 标 @deprecated（暂不删，给一个月缓冲）

两个文件头第一行加：
```python
# @deprecated superseded-by=tools/train_e2e/train.py remove-by=2026-08-21
```

**暂不物理删除的原因**：
- 你列的 4 个落地优先级里说"趁热清一次"是步骤 2，本计划只做步骤 1+3（便宜+立即止血）
- 删除属于**单独的清理 PR**，不应混在「纪律建设 PR」里
- 一个月后到期未删 → 由 lint 脚本自动 fail，强制走单独清理 PR

## 四、落地分步（4 步，本计划范围仅前 3 步）

| 步 | 内容 | 工时 | 风险 |
|----|------|------|------|
| 1 | 3 个文件改动（CLAUDE.md / workflow.md / skills/13） | 30min | 0 — 加规则不改代码 |
| 2 | 2 个 train 脚本加 @deprecated 注释 | 5min | 0 — 不删 |
| 3 | 提交纪律 PR（含 Removed: 段） | 5min | 0 — 用自己的规范演示 |
| 4 | （不在本计划）单独 PR 物理删 train.py / train_demo_model.py + E2E_LEARNING_V3_PLAN.md 归档 | 后续 | 中 — 需确认无 pipeline 引用 |

## 五、验证（4 步走完后）

### 5.1 文档 grep 验证
```bash
# 规则可被 grep 出
grep -E "^### 重构/替代|^### 违反以上规范" CLAUDE.md      # 期望 2 行
grep -E "Removed: \-" .claude/skills/workflow.md             # 期望至少 1 行模板
grep -E "superseded-by|remove-by" tools/train/train.py      # 期望 1 行
grep "inference_node.cpp" CLAUDE.md                          # 漂移已修
grep "inference_node.c" CLAUDE.md                            # 期望 0 行
```

### 5.2 commit message 验证（自我演示）
完成第 3 步后：
```bash
git log --format=%B -1 HEAD
```
输出应包含：
```
Removed:
- tools/train/train.py (added @deprecated, 实际删除留给后续 PR)
- tools/train_demo_model.py (added @deprecated, 实际删除留给后续 PR)
```

### 5.3 Phase 1 探索结论回归
- [ ] CLAUDE.md 关键文件表 `inference_node.c` 漂移已修
- [ ] workflow.md 第五节「清理」已升级为产出 `Removed:` 段
- [ ] CLAUDE.md 「违反以上规范的代码不会被合并」前一节已加「重构/替代 → 同一 commit 删旧」
- [ ] skills/13 明确声明 canonical 入口表
- [ ] `tools/train/train.py` 和 `tools/train_demo_model.py` 文件头有 `@deprecated superseded-by=... remove-by=2026-08-21`

### 5.4 未来 lint 脚本（不在本计划范围）
下一 PR 应建 `tools/lint_entrypoints.py`：
- 扫描 `tools/lint_orphans.py`：列出所有 `*_v[0-9].*` / `*_old.*` / `*_new.*` / `*_bak.*` 文件，CI fail
- 扫描 `tools/lint_dead_refs.py`：CLAUDE.md 关键文件表里的每个路径，断言文件存在
- 扫描 `tools/lint_deprecated.py`：所有 `@deprecated` 注释必须有 `remove-by` 且 ≤ 30 天

本 PR 不做（属于"立即止血"之外的"中期"步骤）。

## 六、不做的事（明确边界）

❌ **不**写 lint 脚本 — 你说"云端执行"是后续 PR 范围
❌ **不**物理删 5 份 train 脚本 — 用 `@deprecated` 标记，单独清理 PR 处理
❌ **不**归档 E2E_LEARNING_V3_PLAN.md — 留待文档真相源 PR
❌ **不**新建 CI 配置文件 — 仓库目前无 `.github/workflows/` 或 `.gitlab-ci.yml`，属基础设施决定，本计划只动纪律
❌ **不**碰 vis/ 或 3D 渲染 — 你说"3D 那块已经收敛了"，本计划专注学习闭环 + 通用纪律

## 七、风险评估

| 风险 | 概率 | 影响 | 缓解 |
|------|------|------|------|
| 强制 `Removed:` 段让非重构 commit 也被卡 | 低 | 中 | 明确写"feat/fix/chore/docs/test 可省略" |
| `@deprecated remove-by` 被 agent 故意漏写 | 中 | 低 | 给 30 天宽限期，lint 脚本只 warn 不 fail（下一 PR） |
| 删旧被激进执行、误删正在用的文件 | 中 | 高 | 本计划**不**删任何文件，只标 @deprecated；删除留独立 PR |
| 其他文档（LEARNING_LOOP.md）漂移本计划未处理 | 中 | 低 | 在 skills/13 增 canonical 入口表是软约束；下一 PR 集中处理 |

## 八、决策待确认

按你的工作流约定，**本计划需要你 review 后才能执行**。如果同意，我马上动手 3 个文件改动 + 2 个 @deprecated 注释 + 1 个 commit。

如有要调整的：
- `remove-by` 日期用 `2026-08-21`（今天 7-21 + 30 天）OK 吗？
- CLAUDE.md 关键文件表 line 49 改 `.c → .cpp` 同意吗？还是要我做更全的扫描（其它行可能也有漂移）？
- skills/13 的 canonical 表是放在第 17 行后还是新加一节"## Canonical 入口"？
