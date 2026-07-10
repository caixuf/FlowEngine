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


if __name__ == "__main__":
    unittest.main()
