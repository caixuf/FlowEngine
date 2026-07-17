#!/usr/bin/env python3
"""
json_to_xodr.py — 场景 JSON → OpenDRIVE (.xodr) 转换器

FlowSim v2 用 esmini RoadManager 处理道路网络，esmini 吃 OpenDRIVE (.xodr)。
本工具把 FlowEngine 场景 JSON 的道路描述转成合法的 xodr，避免手写 XML。

支持两种输入格式：

1. 旧格式 (现有 14 个场景):
   "road": { "curve_start_x": 200, "curve_length_m": 120, "curve_offset_m": 8 }
   → 2 个 road：直线段 (0~curve_start_x) + 弯道段 (单 arc 近似)
   弯道曲率 k ≈ 2*offset/L² (小角度近似：横向偏移 = L²·k/2)

2. 新格式 (FlowSim v2):
   "road_network": {
     "edges": [
       { "id": 0, "type": "urban", "length_m": 200, "lanes": 3, "lane_width": 3.5,
         "speed_limit": 11.11 },
       { "id": 4, "type": "ramp_curve", "length_m": 250, "lanes": 1,
         "curvature_profile": [{"radius": 45, "arc": 130}, ...] }
     ]
   }
   → 每条 edge 一个 road，curvature_profile 拆成多个 <arc> geometry

向后兼容：无 road/road_network 字段时输出单条 1000m 直道（ego 默认场景）。

用法:
  python3 tools/json_to_xodr.py scenarios/highway_exit.json -o /tmp/highway_exit.xodr
  python3 tools/json_to_xodr.py scenarios/city_to_highway.json   # 输出到 stdout
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional


# ── 几何累积器 ──────────────────────────────────────────────

@dataclass
class RoadState:
    """沿 planView 累积的全局起点坐标/朝向，保证段间端点连续。"""
    x: float = 0.0
    y: float = 0.0
    hdg: float = 0.0  # 弧度

    def advance(self, length: float, curvature: float = 0.0) -> "RoadState":
        """沿当前朝向前进 length 米；curvature≠0 时为圆弧，返回段终点状态。"""
        if abs(curvature) < 1e-9:
            # 直线
            nx = self.x + length * math.cos(self.hdg)
            ny = self.y + length * math.sin(self.hdg)
            return RoadState(nx, ny, self.hdg)
        # 圆弧: 转角 dtheta = length * k, 半径 R = 1/k
        k = curvature
        dtheta = length * k
        R = 1.0 / k
        # 圆心在朝向法线方向
        cx = self.x - R * math.sin(self.hdg)
        cy = self.y + R * math.cos(self.hdg)
        nh = self.hdg + dtheta
        nx = cx + R * math.sin(nh)
        ny = cy - R * math.cos(nh)
        return RoadState(nx, ny, nh)


# ── Road 构建器 ─────────────────────────────────────────────

@dataclass
class GeometrySeg:
    s: float          # 沿 road 的起点里程
    x: float          # 全局起点 x
    y: float          # 全局起点 y
    hdg: float        # 起点朝向
    length: float
    curvature: float  # 0 = line, ≠0 = arc


@dataclass
class Road:
    id: int
    name: str
    length: float
    lane_count: int
    lane_width: float
    speed_limit: float
    geoms: list[GeometrySeg] = field(default_factory=list)
    # OpenDRIVE junction linkage (NOA Phase 1: 分叉/汇入路网支持)
    # junction_id >= 0 表示本 road 是某 junction 的 connecting road；
    # predecessor/successor >= 0 时生成 <link>，描述与 incoming/target road 的连接。
    junction_id: int = -1
    predecessor: int = -1   # elementId of predecessor road (<link><predecessor>)
    successor: int = -1     # elementId of successor road (<link><successor>)


@dataclass
class Junction:
    """OpenDRIVE <junction> 描述：道路分叉(fork)或汇入(merge)。
    incoming_road 是分叉前/汇入前的主路 id；connections 中每项为
    (connecting_road_id, target_road_id_or_None)，target 为 None 时
    表示该 connecting road 终点不再接续（fork 的分支末端）。"""
    id: int
    name: str
    incoming_road: int
    connections: list  # list[tuple[int, Optional[int]]]


def build_straight_road(rid: int, name: str, length: float, lanes: int,
                        lane_width: float, speed: float, state: RoadState) -> tuple[Road, RoadState]:
    """单直线 road，从 state 起，返回 road 和终点 state。"""
    end = state.advance(length, 0.0)
    road = Road(rid, name, length, lanes, lane_width, speed, [
        GeometrySeg(0.0, state.x, state.y, state.hdg, length, 0.0)
    ])
    return road, end


def build_curve_road_from_profile(rid: int, name: str, profile: list[dict],
                                  lanes: int, lane_width: float, speed: float,
                                  state: RoadState) -> tuple[Road, RoadState]:
    """按 curvature_profile 构建弯道 road，每段一个 geometry（line 或 arc）。
    profile 项: {"radius": R, "arc": L}  radius>0 右弯, <0 左弯, 0/缺省 直线。"""
    geoms: list[GeometrySeg] = []
    s_acc = 0.0
    cur = state
    total = 0.0
    for seg in profile:
        L = float(seg.get("arc", 0.0))
        R = float(seg.get("radius", 0.0))
        k = 0.0 if abs(R) < 1e-9 else 1.0 / R
        geoms.append(GeometrySeg(s_acc, cur.x, cur.y, cur.hdg, L, k))
        cur = cur.advance(L, k)
        s_acc += L
        total += L
    road = Road(rid, name, total, lanes, lane_width, speed, geoms)
    return road, cur


# ── 场景 JSON → Road 列表 ──────────────────────────────────

DEFAULT_LANE_WIDTH = 3.5
DEFAULT_SPEED = 13.89  # 50 km/h

def roads_from_legacy_road(road_cfg: dict) -> list[Road]:
    """旧格式 road{curve_start_x, curve_length_m, curve_offset_m} → 2 个 road。"""
    cstart = float(road_cfg.get("curve_start_x", 0.0))
    clen = float(road_cfg.get("curve_length_m", 0.0))
    coff = float(road_cfg.get("curve_offset_m", 0.0))
    lanes = 2
    lw = DEFAULT_LANE_WIDTH
    sp = DEFAULT_SPEED
    state = RoadState(0.0, 0.0, 0.0)
    roads: list[Road] = []
    if cstart > 0:
        r, state = build_straight_road(0, "urban", cstart, lanes, lw, sp, state)
        roads.append(r)
    if clen > 0:
        # 小角度近似: offset = L²·k/2  →  k = 2·offset/L²
        k = (2.0 * coff / (clen * clen)) if clen > 0 else 0.0
        end = state.advance(clen, k)
        roads.append(Road(
            id=len(roads), name="curve", length=clen, lane_count=lanes,
            lane_width=lw, speed_limit=sp,
            geoms=[GeometrySeg(0.0, state.x, state.y, state.hdg, clen, k)]
        ))
        state = end
    if not roads:
        # 纯直道
        r, _ = build_straight_road(0, "urban", 1000.0, lanes, lw, sp, state)
        roads.append(r)
    return roads


def roads_from_road_network(rn_cfg: dict) -> tuple[list[Road], list[Junction]]:
    """新格式 road_network{edges:[...], junctions:[...]} → (roads, junctions)。

    每个 edge 一个 road；edges 之间默认顺序拼接（端点连续）。
    junctions 数组描述道路分叉(fork)/汇入(merge)：
      - fork: incoming_road 终点分叉出多条 connecting_roads，每条从 incoming_road
              终点状态起独立构建，标记 junction_id 并设 predecessor=incoming_road。
      - merge: incoming_road（加速车道）汇入 target_road，把 incoming_road 标记为
              junction 的 connecting road，successor=target_road。
    无 junctions 时退化为既有顺序拼接逻辑（完全向后兼容）。"""
    edges = rn_cfg.get("edges") or rn_cfg.get("segments") or []
    state = RoadState(0.0, 0.0, 0.0)
    roads: list[Road] = []
    end_states: dict[int, RoadState] = {}   # road id → 该 road 终点的全局状态

    # 第一遍：顺序构建所有 edge（主路串联），记录每段终点状态供 junction 分支起算。
    for i, e in enumerate(edges):
        eid = int(e.get("id", i))
        etype = str(e.get("type", "road"))
        length = float(e.get("length_m", 0.0))
        lanes = int(e.get("lanes", e.get("lane_count", 2)))
        lw = float(e.get("lane_width", DEFAULT_LANE_WIDTH))
        sp = float(e.get("speed_limit", DEFAULT_SPEED))
        profile = e.get("curvature_profile")
        if profile:
            r, state = build_curve_road_from_profile(
                eid, etype, profile, lanes, lw, sp, state)
        else:
            r, state = build_straight_road(eid, etype, length, lanes, lw, sp, state)
        roads.append(r)
        end_states[eid] = state

    junctions: list[Junction] = []
    junctions_cfg = rn_cfg.get("junctions") or []
    for jcfg in junctions_cfg:
        jid = int(jcfg.get("id", 100 + len(junctions)))
        jtype = str(jcfg.get("type", "fork"))
        incoming = int(jcfg.get("incoming_road", -1))
        if incoming < 0:
            continue
        start_state = end_states.get(incoming, RoadState())

        if jtype == "fork":
            # 分叉：每条 connecting_road 从 incoming_road 终点起独立构建。
            conns: list[tuple[int, Optional[int]]] = []
            for c in jcfg.get("connecting_roads", []):
                cid = int(c.get("id", -1))
                if cid < 0:
                    continue
                cname = str(c.get("name", f"conn_{cid}"))
                clen = float(c.get("length_m", 0.0))
                clanes = int(c.get("lanes", c.get("lane_count", 1)))
                clw = float(c.get("lane_width", DEFAULT_LANE_WIDTH))
                csp = float(c.get("speed_limit", DEFAULT_SPEED))
                cprofile = c.get("curvature_profile")
                # 分支各自从分叉点起算，互不影响：拷贝起点状态
                # 如果连接道路 id 已存在（已在 edges 中定义），则更新它的
                # junction_id/predecessor 而非重新构建。
                existing = None
                for r in roads:
                    if r.id == cid:
                        existing = r
                        break
                if existing:
                    cr = existing
                    cr.junction_id = jid
                    cr.predecessor = incoming
                    cend = end_states.get(cid, start_state)
                else:
                    if cprofile:
                        cr, cend = build_curve_road_from_profile(
                            cid, cname, cprofile, clanes, clw, csp, start_state)
                    else:
                        cr, cend = build_straight_road(cid, cname, clen, clanes, clw, csp, start_state)
                    cr.junction_id = jid
                    cr.predecessor = incoming
                    roads.append(cr)
                # 若配置了 target_road，分支末端接续到目标主路
                tgt = c.get("target_road")
                if tgt is not None:
                    cr.successor = int(tgt)
                    conns.append((cid, int(tgt)))
                else:
                    conns.append((cid, None))
                end_states[cid] = cend
            junctions.append(Junction(jid, f"fork_{jid}", incoming, conns))

        elif jtype == "merge":
            # 汇入：incoming_road（加速车道）作为 connecting road 汇入 target_road。
            # target_s（汇入点）几何上按端点接续，不单独建模。
            target = int(jcfg.get("target_road", -1))
            # 找到已构建的 incoming road，标记为 junction connecting road 并接续 target
            for rd in roads:
                if rd.id == incoming:
                    rd.junction_id = jid
                    if target >= 0:
                        rd.successor = target
                    break
            conns2: list[tuple[int, Optional[int]]] = [(incoming, target if target >= 0 else None)]
            junctions.append(Junction(jid, f"merge_{jid}", incoming, conns2))

    if not roads:
        r, _ = build_straight_road(0, "urban", 1000.0, 2, DEFAULT_LANE_WIDTH,
                                   DEFAULT_SPEED, RoadState())
        roads.append(r)
    return roads, junctions


# ── Road 列表 → xodr XML ───────────────────────────────────

def road_to_xml(road: Road) -> ET.Element:
    r = ET.Element("road", {
        "name": road.name,
        "length": f"{road.length:.6f}",
        "id": str(road.id),
        "junction": str(road.junction_id),
    })
    # NOA Phase 1: <link> 描述 connecting road 与 incoming/target road 的拓扑连接，
    # esmini RoadManager 据此构建分叉/汇入路网。
    if road.predecessor >= 0 or road.successor >= 0:
        link = ET.SubElement(r, "link")
        if road.predecessor >= 0:
            ET.SubElement(link, "predecessor", {
                "elementId": str(road.predecessor),
                "elementType": "road",
                "contactPoint": "end",
            })
        if road.successor >= 0:
            ET.SubElement(link, "successor", {
                "elementId": str(road.successor),
                "elementType": "road",
                "contactPoint": "start",
            })
    pv = ET.SubElement(r, "planView")
    for g in road.geoms:
        geom = ET.SubElement(pv, "geometry", {
            "s": f"{g.s:.6f}",
            "x": f"{g.x:.6f}",
            "y": f"{g.y:.6f}",
            "hdg": f"{g.hdg:.6f}",
            "length": f"{g.length:.6f}",
        })
        if abs(g.curvature) < 1e-9:
            ET.SubElement(geom, "line")
        else:
            ET.SubElement(geom, "arc", {"curvature": f"{g.curvature:.9f}"})

    # 车道：center(参考线 width=0) + right N 条 driving 车道
    lanes = ET.SubElement(r, "lanes")
    ls = ET.SubElement(lanes, "laneSection", {"s": "0.0"})
    center = ET.SubElement(ls, "center")
    cl = ET.SubElement(center, "lane", {"id": "0", "type": "none"})
    ET.SubElement(cl, "width", {"a": "0", "b": "0", "c": "0", "d": "0"})
    right = ET.SubElement(ls, "right")
    for li in range(1, road.lane_count + 1):
        ln = ET.SubElement(right, "lane", {"id": f"-{li}", "type": "driving"})
        ET.SubElement(ln, "width", {
            "a": f"{road.lane_width:.4f}", "b": "0", "c": "0", "d": "0"
        })
    # 限速 via type/speed (OpenDRIVE 用 road type 元素)
    t = ET.SubElement(r, "type", {"s": "0.0", "type": "town"})
    ET.SubElement(t, "speed", {"max": f"{road.speed_limit:.4f}", "unit": "m/s"})
    return r


def build_xodr(roads: list[Road], junctions: list[Junction] | None = None) -> ET.Element:
    root = ET.Element("OpenDRIVE")
    ET.SubElement(root, "header", {
        "revMajor": "1", "revMinor": "4", "name": "FlowEngine",
        "version": "1.4", "date": "",
        "north": "0", "south": "0", "east": "0", "west": "0",
    })
    for rd in roads:
        root.append(road_to_xml(rd))
    # NOA Phase 1: <junction> 元素声明分叉/汇入拓扑。esmini 用其解析 connecting
    # road 与 incoming road 的连接关系，是表达匝道分叉/加速车道汇入的关键。
    for j in (junctions or []):
        je = ET.SubElement(root, "junction", {
            "name": j.name,
            "id": str(j.id),
        })
        for ci, (conn_road, _target) in enumerate(j.connections):
            # target 仅用于 connecting road 的 <successor>，已在 road_to_xml 的 <link> 中表达
            ET.SubElement(je, "connection", {
                "id": str(ci),
                "incomingRoad": str(j.incoming_road),
                "connectingRoad": str(conn_road),
                "contactPoint": "start",
            })
    return root


# ── 主入口 ─────────────────────────────────────────────────

def convert(scenario: dict) -> str:
    """场景 dict → xodr XML 字符串。"""
    junctions: list[Junction] = []
    if "road_network" in scenario:
        roads, junctions = roads_from_road_network(scenario["road_network"])
    elif "road" in scenario:
        roads = roads_from_legacy_road(scenario["road"])
    else:
        r, _ = build_straight_road(0, "urban", 1000.0, 2, DEFAULT_LANE_WIDTH,
                                   DEFAULT_SPEED, RoadState())
        roads = [r]
    root = build_xodr(roads, junctions)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="FlowEngine 场景 JSON → OpenDRIVE xodr")
    ap.add_argument("scenario", help="场景 JSON 文件路径")
    ap.add_argument("-o", "--output", help="输出 xodr 路径 (默认 stdout)")
    args = ap.parse_args()

    with open(args.scenario, "r", encoding="utf-8") as f:
        scenario = json.load(f)
    xml = convert(scenario)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(xml)
        print(f"✓ {args.scenario} → {args.output} ({len(xml)} bytes)", file=sys.stderr)
    else:
        print(xml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
