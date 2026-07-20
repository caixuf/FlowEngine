#!/usr/bin/env python3
"""Behavior tests for demo_evaluator configuration-driven checks."""

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_evaluator():
    spec = importlib.util.spec_from_file_location("demo_evaluator", ROOT / "tools/demo_evaluator.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class DemoEvaluatorTest(unittest.TestCase):
    def test_expected_edges_are_generated_from_pipeline(self):
        evaluator = load_evaluator()
        pipeline = {
            "processes": [
                {"name": "producer", "publish": [{"topic": "topic/a", "type": "A"}]},
                {"name": "consumer", "subscribe": ["topic/a"]},
                {"name": "observer", "subscribe": ["topic/missing"]},
            ]
        }

        self.assertEqual(
            evaluator.expected_edges_from_pipeline(pipeline),
            [("producer", "topic/a", "consumer")],
        )

    def test_lane_change_requirement_is_scenario_criteria(self):
        evaluator = load_evaluator()
        sample = {
            "timestamp": 1.0,
            "metrics": {
                "topics": [
                    {"topic": "vehicle/state", "freq": 20.0},
                    {"topic": "sensor/lidar", "freq": 20.0},
                    {"topic": "sensor/gps", "freq": 10.0},
                    {"topic": "fusion/localization", "freq": 20.0},
                    {"topic": "planning/trajectory", "freq": 10.0},
                    {"topic": "control/raw_cmd", "freq": 10.0},
                    {"topic": "control/cmd", "freq": 10.0},
                ],
                "vehicle": {"speed": 10.0, "x": 100.0},
                "scene": {"ego": {"x": 100.0, "y": -1.75, "speed": 10.0}, "obstacles": []},
            },
            "nodes": [],
        }

        failures, _, _ = evaluator.score(
            [sample, sample],
            ROOT / "does-not-exist.log",
            criteria={"min_distance_m": 0.0, "min_avg_speed_mps": 0.0},
            expected_edges=[],
        )
        self.assertFalse(any("lane" in failure for failure in failures))

        failures, _, _ = evaluator.score(
            [sample, sample],
            ROOT / "does-not-exist.log",
            criteria={"required_lane_changes": 1, "min_distance_m": 0.0, "min_avg_speed_mps": 0.0},
            expected_edges=[],
        )
        self.assertTrue(any("lane changes too few" in failure for failure in failures))

    def test_perception_metrics_layered_recognition_rate(self):
        """Task 5: 分层识别率按 vehicle/vru 分层统计，漏检行人时 vru 识别率为 0。"""
        evaluator = load_evaluator()
        # 3 帧：每帧 2 car + 1 pedestrian 真值；perceived 仅匹配 car，漏检 ped
        series = []
        for _ in range(3):
            series.append({
                "x": 0.0, "speed": 10.0,
                "entities": [
                    {"id": 1, "type": "car", "x": 20.0, "y": -1.75},
                    {"id": 2, "type": "car", "x": 40.0, "y": 1.75},
                    {"id": 3, "type": "pedestrian", "x": 5.0, "y": 5.0},
                    {"id": 0, "type": "ego", "x": 0.0, "y": 0.0},
                ],
                "obs_world": [
                    {"id": 100, "x": 20.5, "y": -1.75},  # 命中 car 1
                    {"id": 101, "x": 39.5, "y": 1.75},   # 命中 car 2
                    # 行人漏检
                ],
            })
        result = evaluator._compute_perception_metrics(series, [0.0, 1.0, 2.0])
        self.assertAlmostEqual(result["recognition_rate_vehicle"], 1.0)
        self.assertAlmostEqual(result["recognition_rate_vru"], 0.0)
        # 3 帧 × (2 car 命中 + 1 ped 漏) = 6 命中 / 9 真值 = 2/3
        self.assertAlmostEqual(result["recognition_rate_overall"], 2.0 / 3.0)
        self.assertEqual(result["truth_count_vehicle"], 6)
        self.assertEqual(result["truth_count_vru"], 3)
        self.assertEqual(result["truth_count_overall"], 9)
        # by_type 细粒度
        self.assertAlmostEqual(result["recognition_rate_by_type"]["car"], 1.0)
        self.assertAlmostEqual(result["recognition_rate_by_type"]["pedestrian"], 0.0)

    def test_perception_metrics_warning_lead_time(self):
        """Task 5: 预警提前量 = TTC 跌破临界时刻 - 首次检测时刻。"""
        evaluator = load_evaluator()
        # 固定障碍 x=50；ego 从 x=0 匀速 10m/s，10 帧每秒 1 帧。
        # TTC < 3s 时 ego_x > 20 → 第 5 帧（ego_x=25, TTC=2.5）首次跌破。
        # 首次检测在第 0 帧 → warning_lead = 5.0s。
        series = []
        timestamps = []
        for i in range(10):
            series.append({
                "x": float(i * 5), "speed": 10.0,
                "entities": [{"id": 1, "type": "car", "x": 50.0, "y": 0.0}],
                "obs_world": [{"id": 100, "x": 50.0, "y": 0.0}],
            })
            timestamps.append(float(i))
        result = evaluator._compute_perception_metrics(series, timestamps)
        self.assertEqual(result["critical_event_count"], 1)
        self.assertAlmostEqual(result["warning_lead_avg_s"], 5.0, places=2)
        self.assertAlmostEqual(result["warning_lead_min_s"], 5.0, places=2)
        # 末帧 ego_x=45, TTC=5/10=0.5
        self.assertAlmostEqual(result["min_ttc_s"], 0.5, places=2)

    def test_perception_metrics_no_critical_event(self):
        """无临界事件时 critical_event_count=0，不抛异常。"""
        evaluator = load_evaluator()
        series = [{
            "x": 0.0, "speed": 1.0,
            "entities": [{"id": 1, "type": "car", "x": 100.0, "y": 0.0}],
            "obs_world": [{"id": 100, "x": 100.0, "y": 0.0}],
        }]
        result = evaluator._compute_perception_metrics(series, [0.0])
        self.assertEqual(result["critical_event_count"], 0)
        self.assertEqual(result["warning_lead_avg_s"], 0.0)
        self.assertEqual(result["warning_lead_min_s"], 0.0)
        # TTC = 100/1 = 100s，远大于临界 3s，min_ttc_s 应为 100
        self.assertAlmostEqual(result["min_ttc_s"], 100.0, places=2)


if __name__ == "__main__":
    unittest.main()
