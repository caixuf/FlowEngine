#!/usr/bin/env python3
"""pipeline_check.py — 秒级离线管道完整性检查。

读取 /tmp/flow_topology.json（或已保存的 JSON），不启动 demo，秒级验证
算法链路的完整性。适合改完代码后快速确认数据管道没有断。

用法:
    python3 tools/pipeline_check.py                    # 检查当前运行实例
    python3 tools/pipeline_check.py --json /tmp/snap.json  # 检查已保存快照
    python3 tools/pipeline_check.py --focus topology    # 只查拓扑完整性
    python3 tools/pipeline_check.py --focus perception  # 只查感知链路
    python3 tools/pipeline_check.py --focus behavior    # 只查行为规划
    python3 tools/pipeline_check.py --verbose           # 显示所有检查细节

退出码: 0=全部PASS, 1=有WARN, 2=有FAIL
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_JSON = Path("/tmp/flow_topology.json")
PIPELINE_JSON = ROOT / "config" / "pipeline.json"

# 每个 topic 的最低可接受频率（Hz）
TOPIC_MIN_FREQ = {
    "vehicle/state": 15.0,
    "sensor/lidar": 15.0,
    "sensor/gps": 7.0,
    "fusion/localization": 17.0,
    "perception/obstacles": 5.0,
    "planning/trajectory": 5.0,
    "planning/behavior": 5.0,
    "control/raw_cmd": 10.0,
    "control/cmd": 10.0,
    # 注意：behavior/state 是 transport topic（非 bus topic），
    # 不在 metrics.topics 中，改用 metrics.behavior 字段检查
}

# 期望的行为状态机状态集合
BEHAVIOR_STATES = {"NA", "READY", "CRUISE", "FOLLOW", "LEFT_CHANGE",
                   "RIGHT_CHANGE", "STOP", "YIELD", "EMERGENCY"}

# 车道数量来源：感知/规划期望的合理范围
LANE_COUNT_RANGE = (1, 8)
LANE_WIDTH_RANGE = (2.5, 4.0)

# 速度合理范围（m/s）
SPEED_SANE_MAX = 30.0
STEER_SANE_MAX = 0.5

# 静态拓扑边（发布节点→topic→订阅节点）
EXPECTED_EDGES: list[tuple[str, str, str]] = [
    ("flowsim", "vehicle/state", "sensor_model"),
    ("flowsim", "vehicle/state", "monitor"),
    ("flowsim", "scene/frame", "monitor"),
    ("sensor_model", "sensor/lidar", "perception"),
    ("sensor_model", "sensor/gps", "fusion"),
    ("perception", "perception/obstacles", "object_tracker"),
    ("perception", "perception/obstacles", "behavior_planner"),
    ("perception", "perception/obstacles", "planning"),
    ("perception", "perception/obstacles", "safety_control"),
    ("object_tracker", "perception/tracked_objects", "behavior_planner"),
    ("fusion", "fusion/localization", "behavior_planner"),
    ("fusion", "fusion/localization", "planning"),
    ("fusion", "fusion/localization", "control"),
    ("fusion", "fusion/localization", "safety_control"),
    ("behavior_planner", "planning/behavior", "planning"),
    ("planning", "planning/trajectory", "control"),
    ("control", "control/raw_cmd", "safety_control"),
    ("safety_control", "control/cmd", "flowsim"),
]


# ── helpers ────────────────────────────────────────────────────────

TRUTH_TYPE_VEHICLE = {"car", "truck", "suv"}
TRUTH_TYPE_VRU = {"pedestrian"}


def load_json(path: Path) -> dict | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except (json.JSONDecodeError, OSError, FileNotFoundError):
        return None


def lane_idx_from_y(y: float, lane_count: int, lane_width: float,
                    center: float = 0.0) -> int:
    y_rel = y - center
    offset = -y_rel / lane_width + (lane_count - 1) * 0.5
    return max(0, min(lane_count - 1, int(round(offset))))


def lane_center_y(idx: int, lane_count: int, lane_width: float,
                  center: float = 0.0) -> float:
    return -(idx - (lane_count - 1) * 0.5) * lane_width + center


def load_pipeline_topics() -> set[str]:
    """从 pipeline.json 读取注册 topic 列表."""
    topics: set[str] = set()
    if not PIPELINE_JSON.exists():
        return topics
    try:
        cfg = json.loads(PIPELINE_JSON.read_text(encoding="utf-8"))
        for proc in cfg.get("processes", []):
            for p in proc.get("publish", []):
                t = p.get("topic", "")
                if t:
                    topics.add(t)
            for s in proc.get("subscribe", []):
                if isinstance(s, str) and s:
                    topics.add(s)
    except (json.JSONDecodeError, OSError):
        pass
    return topics


# ── 检查器基类 ─────────────────────────────────────────────────────

class _Result:
    """单一检查结果."""
    def __init__(self, ok: bool, msg: str):
        self.ok = ok
        self.msg = msg


class _Section:
    """一组检查项."""
    def __init__(self, name: str, description: str = ""):
        self.name = name
        self.description = description
        self.results: list[_Result] = []
        self.pass_count = 0
        self.fail_count = 0
        self.warn_count = 0

    def ok(self, msg: str) -> None:
        self.results.append(_Result(True, msg))

    def fail(self, msg: str) -> None:
        self.results.append(_Result(False, f"FAIL: {msg}"))

    def warn(self, msg: str) -> None:
        self.results.append(_Result(False, f"WARN: {msg}"))

    def summarize(self) -> None:
        for r in self.results:
            if r.ok:
                self.pass_count += 1
            elif r.msg.startswith("WARN"):
                self.warn_count += 1
            else:
                self.fail_count += 1

    def has_fail(self) -> bool:
        return any(not r.ok and not r.msg.startswith("WARN") for r in self.results)

    def has_warn(self) -> bool:
        return any(not r.ok and r.msg.startswith("WARN") for r in self.results)

    def print_report(self, verbose: bool = False) -> None:
        header = f"── {self.name}"
        if self.description:
            header += f" — {self.description}"
        print(f"\n{header}")
        print("─" * 60)
        for r in self.results:
            if r.ok:
                if verbose:
                    print(f"  ✓ {r.msg}")
            else:
                symbol = "⚠" if r.msg.startswith("WARN") else "✗"
                print(f"  {symbol} {r.msg}")
        total = self.pass_count + self.fail_count + self.warn_count
        if total > 0:
            print(f"  ── {self.pass_count} pass / {self.warn_count} warn / {self.fail_count} fail")


# ── 检查函数 ───────────────────────────────────────────────────────

def check_topology(data: dict, section: _Section) -> None:
    """检查节点拓扑完整性."""
    nodes = data.get("nodes", [])
    if not nodes or not isinstance(nodes, list):
        section.fail("nodes 数组为空或缺失")
        return
    section.ok(f"{len(nodes)} 个节点注册")

    # 节点名称列表
    node_names = set()
    for n in nodes:
        nm = n.get("name", "")
        if nm:
            node_names.add(nm)
            alive = n.get("alive", True)
            if not alive:
                section.fail(f"节点 {nm} 标记为不可用 (alive=false)")

    # 检查 pipeline.json 注册的所有节点是否都在拓扑中
    if PIPELINE_JSON.exists():
        try:
            cfg = json.loads(PIPELINE_JSON.read_text(encoding="utf-8"))
            for proc in cfg.get("processes", []):
                pn = proc.get("name", "")
                if pn and pn not in node_names:
                    section.warn(f"pipeline 注册节点 '{pn}' 不在拓扑中（可能未启动）")
        except (json.JSONDecodeError, OSError):
            pass

    # 检查话题发布-订阅边
    registry = data.get("registry", {})
    tasks = registry.get("tasks", [])
    if tasks:
        found_edges = 0
        missing_edges = 0
        for pub_node, topic, sub_node in EXPECTED_EDGES:
            # 检查发布节点是否注册为该 topic 的 publisher
            pub_ok = False
            sub_ok = False
            for t in tasks:
                tn = t.get("name", "")
                outputs = t.get("outputs", [])
                inputs = t.get("inputs", [])
                if tn == pub_node and topic in outputs:
                    pub_ok = True
                if tn == sub_node and topic in inputs:
                    sub_ok = True
            if pub_ok and sub_ok:
                found_edges += 1
            else:
                missing_edges += 1
                if not pub_ok:
                    section.warn(f"边缺失: {pub_node} 未发布 {topic}")
                if not sub_ok:
                    section.warn(f"边缺失: {sub_node} 未订阅 {topic}")
        if missing_edges == 0:
            section.ok(f"全部 {len(EXPECTED_EDGES)} 条拓扑边完整")
        else:
            section.ok(f"{found_edges}/{len(EXPECTED_EDGES)} 条拓扑边存在")


def check_topic_frequencies(data: dict, section: _Section) -> None:
    """检查 topic 发布频率."""
    topics_raw = data.get("metrics", {}).get("topics", [])
    if not topics_raw:
        section.fail("metrics.topics 数组为空或缺失")
        return

    topic_freqs = {}
    for t in topics_raw:
        name = t.get("topic", "")
        if name:
            topic_freqs[name] = t.get("freq", 0.0)

    section.ok(f"共 {len(topic_freqs)} 个 topic")
    for topic, min_freq in TOPIC_MIN_FREQ.items():
        actual = topic_freqs.get(topic, 0.0)
        if actual < min_freq:
            section.fail(f"{topic} 频率过低: {actual:.1f} Hz < {min_freq:.1f} Hz")
        else:
            section.ok(f"{topic}: {actual:.1f} Hz (≥{min_freq:.1f} Hz)")


def check_data_integrity(data: dict, section: _Section) -> None:
    """检查 metrics 中各段数据的完整性."""
    metrics = data.get("metrics", {})
    scene = metrics.get("scene", {})

    # scene.ego
    ego = scene.get("ego", {})
    if not ego:
        section.fail("metrics.scene.ego 缺失")
    else:
        required = ["x", "y", "heading", "speed", "steer"]
        missing = [k for k in required if k not in ego]
        if missing:
            section.fail(f"scene.ego 缺少字段: {missing}")
        else:
            x = float(ego.get("x", 0))
            y = float(ego.get("y", 0))
            speed = float(ego.get("speed", 0))
            steer = float(ego.get("steer", 0))
            if abs(x) < 0.001 and abs(y) < 0.001:
                section.fail(f"ego 位置恒定零 (x={x}, y={y})，可能未初始化")
            else:
                section.ok(f"ego 位置正常 x={x:.1f} y={y:.1f}")
            if speed < 0 or speed > SPEED_SANE_MAX:
                section.warn(f"ego 速度异常: {speed:.1f} m/s")
            else:
                section.ok(f"ego 速度 {speed:.1f} m/s")
            if abs(steer) > STEER_SANE_MAX:
                section.warn(f"ego steer 超限: {steer:.3f} > {STEER_SANE_MAX}")

    # scene.lane
    lane = scene.get("lane", {})
    lane_w = float(lane.get("width", 3.5))
    lane_c = int(lane.get("count", 2))
    if lane_w < LANE_WIDTH_RANGE[0] or lane_w > LANE_WIDTH_RANGE[1]:
        section.warn(f"车道宽度异常: {lane_w} m (合理范围 {LANE_WIDTH_RANGE})")
    else:
        section.ok(f"车道宽 {lane_w} m x {lane_c} 车道")
    if lane_c < LANE_COUNT_RANGE[0] or lane_c > LANE_COUNT_RANGE[1]:
        section.warn(f"车道数异常: {lane_c}")

    # scene.entities
    entities = scene.get("entities", [])
    if not entities or not isinstance(entities, list):
        section.fail("metrics.scene.entities 数组为空或缺失")
    else:
        section.ok(f"scene.entities: {len(entities)} 个物体（含 ego）")

    # scene.obstacles
    obstacles = scene.get("obstacles", [])
    if not isinstance(obstacles, list):
        section.fail("metrics.scene.obstacles 不是数组")
    else:
        section.ok(f"scene.obstacles: {len(obstacles)} 个障碍物")

    # scene.trajectory_path
    traj = scene.get("trajectory_path", [])
    if traj and isinstance(traj, list):
        section.ok(f"轨迹路径 {len(traj)} 点")
    else:
        section.warn("trajectory_path 为空（可能 planning 未输出）")

    # metrics.behavior
    behavior = metrics.get("behavior", {})
    if not behavior:
        section.warn("metrics.behavior 缺失（behavior/state 可能尚未发布）")
    else:
        state = behavior.get("state", "?")
        section.ok(f"behavior state: {state}")


def check_perception(data: dict, section: _Section) -> None:
    """检查感知识别率与类型透传."""
    scene = data.get("metrics", {}).get("scene", {})
    entities = scene.get("entities", [])
    obstacles = scene.get("obstacles", [])

    if not isinstance(entities, list) or not isinstance(obstacles, list):
        section.fail("entities/obstacles 数据不可用")
        return

    # 按类型统计真值与感知结果
    truth_counts: dict[str, int] = {}
    perceived_counts: dict[str, int] = {}

    for ent in entities:
        etype = ent.get("type", "")
        if etype in ("ego", "tl", "etc_gate", "stop_line"):
            continue
        if etype:
            truth_counts[etype] = truth_counts.get(etype, 0) + 1

    for obs in obstacles:
        otype = obs.get("type", "car")
        perceived_counts[otype] = perceived_counts.get(otype, 0) + 1

    if not truth_counts:
        section.warn("entities 中无非基础设施物体（无 NPC/行人）")
        return

    section.ok(f"场景真值: {truth_counts}")
    section.ok(f"感知结果: {perceived_counts}")

    # 逐类型计算识别率
    for etype, count in sorted(truth_counts.items()):
        perceived = perceived_counts.get(etype, 0)
        rate = perceived / count if count > 0 else 1.0
        if rate < 0.5:
            section.fail(f"{etype} 识别率 {rate*100:.0f}% ({perceived}/{count}) < 50%")
        elif rate < 0.8:
            section.warn(f"{etype} 识别率 {rate*100:.0f}% ({perceived}/{count}) < 80%")
        else:
            section.ok(f"{etype} 识别率 {rate*100:.0f}% ({perceived}/{count})")

    # 行人特殊检查（要保证 type=pedestrian 透传到 obstacles）
    pedestrians_truth = truth_counts.get("pedestrian", 0)
    pedestrians_perceived = perceived_counts.get("pedestrian", 0)
    if pedestrians_truth > 0 and pedestrians_perceived == 0:
        section.fail(f"场景有 {pedestrians_truth} 个行人但感知未识别出任何行人 — type 透传可能断裂")


def check_behavior(data: dict, section: _Section) -> None:
    """检查行为规划状态."""
    behavior = data.get("metrics", {}).get("behavior", {})
    if not behavior:
        section.warn("metrics.behavior 缺失，行为规划未激活或未发布")
        return

    state = behavior.get("state", "")
    if state not in BEHAVIOR_STATES:
        section.warn(f"behavior state '{state}' 不在预期集合中")
    else:
        section.ok(f"状态机: {state}")

    committed = behavior.get("committed_lane", -1)
    if committed >= 0:
        section.ok(f"当前车道: {committed}")
    target = behavior.get("target_lane", -1)
    if target >= 0 and target != committed:
        section.ok(f"目标车道: {target} (变道中)")

    best_gap = behavior.get("best_gap", -1.0)
    if isinstance(best_gap, (int, float)) and best_gap > 0:
        section.ok(f"前车间距: {best_gap:.1f} m")
    lead_speed = behavior.get("lead_speed", 0)
    if lead_speed > 0:
        section.ok(f"前车速度: {lead_speed:.1f} m/s")

    desired_gap = behavior.get("desired_gap", 0)
    if desired_gap > 0:
        section.ok(f"期望间距: {desired_gap:.1f} m")

    blocked = behavior.get("blocked", False)
    worthwhile = behavior.get("worthwhile", False)
    if blocked:
        section.ok("前方封堵")
    if worthwhile:
        section.ok("超车可行")

    obs_count = int(behavior.get("obs_count", 0))
    if obs_count > 0:
        section.ok(f"行为规划可见障碍物: {obs_count}")
    else:
        section.warn("behavior 障碍物计数为 0，可能 on_raw_obstacles 未收到数据")


def check_collisions(data: dict, section: _Section) -> None:
    """检查碰撞事件."""
    topics = data.get("metrics", {}).get("topics", [])
    collision_pub = 0
    for t in topics:
        if t.get("topic") == "sim/collision":
            collision_pub = int(t.get("pub", 0))
            break
    if collision_pub > 0:
        section.fail(f"检测到碰撞: sim/collision pub={collision_pub}")
        # 附加检查：collision 有 pub 但 del=0，说明可能仅发生在启动瞬间
        for t in topics:
            if t.get("topic") == "sim/collision":
                delivered = int(t.get("del", 0))
                if delivered == 0:
                    section.warn("collision 消息零投递，可能为启动阶段瞬间接触")
                break
    else:
        section.ok("无碰撞事件")


def check_registry(data: dict, section: _Section) -> None:
    """检查注册表和参数一致性."""
    registry = data.get("metrics", {}).get("registry", {})
    if not registry:
        section.fail("metrics.registry 缺失")
        return

    tasks = registry.get("tasks", [])
    plugins = registry.get("plugins", [])
    types = registry.get("types", [])
    params = registry.get("params", [])

    section.ok(f"tasks={len(tasks)} plugins={len(plugins)} "
               f"types={len(types)} params={len(params)}")

    # 检查 tasks 与 plugins 的一致性
    task_names = {t.get("name", "") for t in tasks}
    plugin_task_names = set()
    for p in plugins:
        for pt_name_attr in ["tasks", "task"]:
            pt = p.get(pt_name_attr, 0)
            if isinstance(pt, int) and pt > 0:
                plugin_task_names.add(p.get("name", ""))
            elif isinstance(pt, list):
                for item in pt:
                    if isinstance(item, str):
                        plugin_task_names.add(item)

    # 检查 planner 必需节点
    required_nodes = {"flowsim", "sensor_model", "perception", "fusion",
                      "behavior_planner", "planning", "control", "safety_control"}
    missing_nodes = required_nodes - task_names
    if missing_nodes:
        section.fail(f"注册表缺少必需任务节点: {missing_nodes}")
    else:
        section.ok("全部 8 个核心节点在注册表中")

    # 参数值范围检查
    for p in params:
        name = p.get("name", "")
        ptype = p.get("type", "")
        value = p.get("value", "")
        # 对已知参数进行范围校验
        if name == "control.cruise_speed" and value:
            try:
                v = float(value)
                if v < 1 or v > 50:
                    section.warn(f"{name}={v} 超出合理范围 [1, 50]")
            except ValueError:
                pass
        elif name == "control.pid_kp" and value:
            try:
                v = float(value)
                if v > 5000:
                    section.warn(f"{name}={v} 偏大")
            except ValueError:
                pass


# ── 主流程 ─────────────────────────────────────────────────────────

SECTIONS = {
    "topology": ("拓扑完整性", check_topology),
    "topics": ("Topic 频率", check_topic_frequencies),
    "integrity": ("数据完整性", check_data_integrity),
    "perception": ("感知识别率", check_perception),
    "behavior": ("行为规划状态", check_behavior),
    "collision": ("碰撞检查", check_collisions),
    "registry": ("注册表一致性", check_registry),
}


def main() -> int:
    ap = argparse.ArgumentParser(
        description="FlowEngine 离线管道完整性检查")
    ap.add_argument("--json", type=Path, default=DEFAULT_JSON,
                    help=f"拓扑 JSON 路径 (默认 {DEFAULT_JSON})")
    ap.add_argument("--focus", type=str, default=None,
                    choices=list(SECTIONS.keys()) + [None],
                    help="只检查某一类")
    ap.add_argument("--verbose", "-v", action="store_true",
                    help="显示所有检查项")
    ap.add_argument("--quiet", "-q", action="store_true",
                    help="仅显示 FAIL/WARN")

    args = ap.parse_args()

    data = load_json(args.json)
    if not data:
        print(f"✗ 无法读取 {args.json} — 文件不存在或不是合法 JSON", file=sys.stderr)
        print("  提示: 先运行 demo.sh 生成数据，或 --json 指定已保存快照", file=sys.stderr)
        return 2

    # 确定要跑的检查段
    sections_to_run: list[tuple[str, str, callable]] = []
    if args.focus:
        desc, func = SECTIONS[args.focus]
        sections_to_run.append((args.focus, desc, func))
    else:
        sections_to_run = [(k, v[0], v[1]) for k, v in SECTIONS.items()]

    all_results: list[_Section] = []
    for section_key, desc, func in sections_to_run:
        sec = _Section(section_key, desc)
        func(data, sec)
        sec.summarize()
        if not args.quiet or sec.has_fail() or sec.has_warn():
            sec.print_report(verbose=args.verbose)
        all_results.append(sec)

    # 全局汇总
    total_pass = sum(s.pass_count for s in all_results)
    total_warn = sum(s.warn_count for s in all_results)
    total_fail = sum(s.fail_count for s in all_results)
    has_any_fail = any(s.has_fail() for s in all_results)
    has_any_warn = any(s.has_warn() for s in all_results)

    print()
    print("=" * 60)
    if has_any_fail:
        print(f"结果: FAIL  ({total_pass} pass / {total_warn} warn / {total_fail} fail)")
    elif has_any_warn:
        print(f"结果: WARN  ({total_pass} pass / {total_warn} warn / {total_fail} fail)")
    else:
        print(f"结果: PASS  ({total_pass} pass / {total_warn} warn / {total_fail} fail)")

    if has_any_fail:
        return 2
    if has_any_warn:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
