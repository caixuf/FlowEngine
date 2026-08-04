"""json_to_xodr.py nodes→XODR 转换回归测试。

根因（2026-08-04）：roads_from_road_network 只认 curvature_profile / length_m，
完全忽略 road_network.edges[].nodes → curve_road 这类 nodes 场景被生成成
一条直线 XODR → ref_path 全 y=0 → 车不跟弯（S 弯直开）。

修复后 nodes 折线 → 三次 Hermite 密采样 → 每段 <line> geometry，必须：
- 穿过每个节点（前端 scene3d / 评估器 demo_evaluator 消费同一份 nodes）
- 几何连续（相邻 geometry 起点 = 上段终点，无 gap）
- 段间 heading 转角小（无 polyline kink，车可跟）
- 直道段保持直道（不把 y=0 的起步段拱弯）
- 2 节点直道（既有场景）行为不变
"""
from __future__ import annotations

import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import json_to_xodr as X  # noqa: E402

# curve_road.json 的 nodes（S 弯，13 点，直道↔弯道切向不连续）
S_CURVE_NODES = [
    [0, 0, 0], [200, 0, 0], [400, 20, 0], [600, 60, 0], [800, 100, 0],
    [1000, 120, 0], [1200, 120, 0], [1400, 100, 0], [1600, 50, 0],
    [1800, -20, 0], [2000, -60, 0], [2400, -60, 0], [3000, -60, 0],
]


def _build_geoms(edges_cfg: list) -> list[ET.Element]:
    rn = {"edges": edges_cfg}
    roads, _ = X.roads_from_road_network(rn)
    root = X.build_xodr(roads)
    out = []
    for road in root.findall("road"):
        out.extend(road.find("planView").findall("geometry"))
    return out


def _s_curve_edge() -> dict:
    return {
        "id": 0, "type": "highway", "lanes": 4, "lane_width": 3.5,
        "speed_limit": 22.0, "nodes": S_CURVE_NODES,
    }


class JsonToXodrTest(unittest.TestCase):
    def test_nodes_converted_to_multiple_geometries(self):
        """nodes 折线必须被转成多条 geometry，而不是一条直线。"""
        geoms = _build_geoms([_s_curve_edge()])
        self.assertGreater(len(geoms), 10, "S 弯 nodes 应被密采样成多条 geometry")

    def test_geometry_passes_through_nodes(self):
        """起点穿过 node[0]，末段终点落在末节点附近。"""
        geoms = _build_geoms([_s_curve_edge()])
        pts = S_CURVE_NODES
        g0 = geoms[0]
        self.assertAlmostEqual(float(g0.get("x")), float(pts[0][0]), delta=0.5)
        self.assertAlmostEqual(float(g0.get("y")), float(pts[0][1]), delta=0.5)
        gl = geoms[-1]
        x, y, h, l = (float(gl.get(k)) for k in ("x", "y", "hdg", "length"))
        ex, ey = x + l * math.cos(h), y + l * math.sin(h)
        self.assertAlmostEqual(ex, float(pts[-1][0]), delta=2.0)
        self.assertAlmostEqual(ey, float(pts[-1][1]), delta=2.0)

    def test_geometry_continuous_no_gap(self):
        """相邻 geometry：下段起点 = 上段终点（esmini 连续求值的前提）。"""
        geoms = _build_geoms([_s_curve_edge()])
        prev_end = None
        for g in geoms:
            x, y, h, l = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
            ex, ey = x + l * math.cos(h), y + l * math.sin(h)
            if prev_end is not None:
                gap = math.hypot(x - prev_end[0], y - prev_end[1])
                self.assertLess(gap, 1e-3, f"geometry 起点与上段终点不连续: {gap}")
            prev_end = (ex, ey)

    def test_heading_kink_bounded(self):
        """段间 heading 转角 <3° → 无 kink，车可跟（R≈500m 的 5m 段转角 ~0.6°）。"""
        geoms = _build_geoms([_s_curve_edge()])
        max_kink = 0.0
        prev_h = None
        for g in geoms:
            h = float(g.get("hdg"))
            if prev_h is not None:
                d = abs(h - prev_h)
                while d > math.pi:
                    d -= 2.0 * math.pi
                max_kink = max(max_kink, abs(d))
            prev_h = h
        self.assertLess(math.degrees(max_kink), 3.0,
                        f"段间 heading 转角 {math.degrees(max_kink):.2f}° 过大（kink）")

    def test_straight_section_stays_straight(self):
        """直道段 (0,0)→(200,0) 保持 y≈0，不被 Hermite 过冲拱弯（CR 过冲 ~3m，门禁 2.5m）。"""
        geoms = _build_geoms([_s_curve_edge()])
        max_dev = 0.0
        for g in geoms:
            x, y = float(g.get("x")), float(g.get("y"))
            if 5.0 < x < 190.0:
                max_dev = max(max_dev, abs(y))
        self.assertLess(max_dev, 2.5, f"直道段被拱弯 {max_dev:.2f}m")

    def test_two_node_straight_regression(self):
        """2 节点直道（既有场景，如 straight_road.json）行为不变：1 条直线。"""
        geoms = _build_geoms([{
            "id": 0, "type": "highway", "lanes": 2, "lane_width": 3.5,
            "nodes": [[0, 0, 0], [3000, 0, 0]],
        }])
        self.assertEqual(len(geoms), 1)
        g = geoms[0]
        self.assertAlmostEqual(float(g.get("hdg")), 0.0, delta=1e-9)
        self.assertAlmostEqual(float(g.get("length")), 3000.0, delta=1e-6)

    def test_curvature_profile_still_works(self):
        """curvature_profile 路径不被破坏（回归）。"""
        roads, _ = X.roads_from_road_network({"edges": [{
            "id": 0, "type": "ramp", "lanes": 1, "lane_width": 3.5,
            "curvature_profile": [{"radius": 45.0, "arc": 130.0}],
        }]})
        self.assertEqual(len(roads), 1)
        self.assertEqual(len(roads[0].geoms), 1)
        self.assertAlmostEqual(roads[0].geoms[0].curvature, 1.0 / 45.0)


if __name__ == "__main__":
    unittest.main()
