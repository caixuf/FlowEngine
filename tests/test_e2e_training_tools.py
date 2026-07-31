#!/usr/bin/env python3
"""Behavior tests for the lightweight E2E training bridge tools."""

import json
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class E2ETrainingToolsTest(unittest.TestCase):
    def test_export_train_and_eval_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            source = work / "samples.jsonl"
            source.write_text(
                "\n".join(
                    json.dumps(
                        {
                            "t": 1000 + i,
                            "schema_version": "flowengine.e2e_sample.v3",
                            "features_v3": [
                                10.0 + 0.1 * i,          # ego_v
                                -1.5 + 0.1 * i,           # ego_y
                                0.0,                       # ego_heading
                                0.0,                       # ego_yaw_rate
                                30.0 + 0.1 * i,            # front0_x
                                0.0,                       # front0_y
                                8.0,                       # front0_vx
                                1.0,                       # front0_type (car)
                                1.0,                       # front0_confidence
                                0.0, 0.0, 0.0, 0.0, 0.0,  # front1 (none)
                                0.0, 0.0,                  # control_brake, emergency_stop
                                # v3 场景上下文 (7维)
                                0.0,                       # tl_state (green)
                                -1.0,                      # tl_distance (no light)
                                0.0,                       # road_curvature
                                30.0,                      # road_speed_limit
                                2.0,                       # lane_count
                                3.5,                       # lane_width
                                -1.5 + 0.1 * i,            # ego_lane_offset
                            ],
                            "scene_context": {
                                "tl_state": 0.0,
                                "tl_distance": -1.0,
                                "curvature": 0.0,
                                "speed_limit": 30.0,
                                "lane_count": 2.0,
                                "lane_width": 3.5,
                                "ego_lane_offset": -1.5 + 0.1 * i,
                            },
                            "label": 8.0 + 0.2 * (i % 5),
                            "control": {"throttle": 0.1 + 0.01 * i, "brake": 0.0, "steer": 0.0, "lane_change": 0, "confidence": 1.0},
                            "ego": {"x": 0.5 * i, "y": -1.5 + 0.1 * i, "v": 10.0 + 0.1 * i},
                        }
                    )
                    for i in range(40)
                )
                + "\n",
                encoding="utf-8",
            )

            dataset = work / "dataset"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/dataset/export_e2e_dataset.py"),
                    "--input",
                    str(source),
                    "--output",
                    str(dataset),
                    "--scenario",
                    "unit_test",
                ],
                cwd=ROOT,
                check=True,
            )

            metadata = json.loads((dataset / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["sample_count"], 40)
            self.assertEqual(metadata["schema_version"], "flowengine.e2e_dataset.v3")

            artifact = work / "artifact"
            artifact.mkdir(parents=True, exist_ok=True)
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/train_e2e/temporal_train.py"),
                    "--input",
                    str(dataset / "samples.jsonl"),
                    "--output",
                    str(artifact / "model.txt"),
                    "--epochs",
                    "3",
                    "--hidden",
                    "4",
                ],
                cwd=ROOT,
                check=True,
            )

            self.assertTrue((artifact / "model.txt").exists())

    def make_sample_source(self, work, start_t=2000):
        source = work / "samples.jsonl"
        source.write_text(
            "\n".join(
                json.dumps(
                    {
                        "t": start_t + i,
                        "schema_version": "flowengine.e2e_sample.v3",
                        "features_v3": [
                            10.0 + 0.1 * i,          # ego_v
                            -1.0 + 0.05 * i,          # ego_y
                            0.0,                       # ego_heading
                            0.0,                       # ego_yaw_rate
                            30.0 + 0.1 * i,            # front0_x
                            0.0,                       # front0_y
                            8.0,                       # front0_vx
                            1.0,                       # front0_type (car)
                            1.0,                       # front0_confidence
                            0.0, 0.0, 0.0, 0.0, 0.0,  # front1 (none)
                            0.0, 0.0,                  # control_brake, emergency_stop
                            # v3 场景上下文 (7维)
                            0.0,                       # tl_state (green)
                            -1.0,                      # tl_distance (no light)
                            0.0,                       # road_curvature
                            30.0,                      # road_speed_limit
                            2.0,                       # lane_count
                            3.5,                       # lane_width
                            -1.0 + 0.05 * i,           # ego_lane_offset
                        ],
                        "scene_context": {
                            "tl_state": 0.0,
                            "tl_distance": -1.0,
                            "curvature": 0.0,
                            "speed_limit": 30.0,
                            "lane_count": 2.0,
                            "lane_width": 3.5,
                            "ego_lane_offset": -1.0 + 0.05 * i,
                        },
                        "label": 6.0 + 0.15 * (i % 7),
                        "control": {"throttle": 0.1 + 0.01 * i, "brake": 0.0, "steer": 0.0, "lane_change": 0, "confidence": 1.0},
                        "ego": {"x": 0.25 * i, "y": -1.0 + 0.05 * i, "v": 10.0 + 0.1 * i},
                    }
                )
                for i in range(40)
            )
            + "\n",
            encoding="utf-8",
        )
        return source

    def export_sample_dataset(self, work):
        source = self.make_sample_source(work)
        dataset = work / "dataset"
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/dataset/export_e2e_dataset.py"),
                "--input",
                str(source),
                "--output",
                str(dataset),
            ],
            cwd=ROOT,
            check=True,
        )
        return dataset

    def train_torch_artifact(self, dataset, artifact):
        return subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/train_e2e/torch_train.py"),
                "--dataset",
                str(dataset),
                "--output",
                str(artifact),
                "--epochs",
                "2",
                "--hidden",
                "4",
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def train_torch_artifact_from(self, dataset, artifact, init_from):
        return subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/train_e2e/torch_train.py"),
                "--dataset",
                str(dataset),
                "--output",
                str(artifact),
                "--epochs",
                "1",
                "--hidden",
                "4",
                "--init-from",
                str(init_from),
            ],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_torch_train_reports_optional_dependency_or_writes_checkpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            dataset = self.export_sample_dataset(work)
            artifact = work / "torch_artifact"
            result = self.train_torch_artifact(dataset, artifact)

            if importlib.util.find_spec("torch") is None:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("PyTorch is not installed", result.stderr)
                return

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((artifact / "model.pt").exists())
            manifest = json.loads((artifact / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["backend"], "pytorch")
            self.assertEqual(manifest["dataset"]["sample_count"], 40)

            finetuned = work / "torch_artifact_finetuned"
            finetune_result = self.train_torch_artifact_from(dataset, finetuned, artifact)
            self.assertEqual(finetune_result.returncode, 0, finetune_result.stderr)
            finetuned_manifest = json.loads((finetuned / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(finetuned_manifest["training"]["init_from"], str(artifact))
            self.assertTrue((finetuned / "model.pt").exists())

    def test_torch_sidecar_reports_optional_dependency_or_writes_shadow_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            dataset = self.export_sample_dataset(work)
            model_dir = work / "torch_artifact"
            train_result = self.train_torch_artifact(dataset, model_dir)
            if importlib.util.find_spec("torch") is None:
                self.assertNotEqual(train_result.returncode, 0)
                self.assertIn("PyTorch is not installed", train_result.stderr)
                return
            self.assertEqual(train_result.returncode, 0, train_result.stderr)

            state_file = work / "flow_topology.json"
            state_file.write_text(
                json.dumps(
                    {
                        "timestamp": 1.0,
                        "metrics": {"vehicle": {"speed": 4.0}},
                        "scene": {"ego": {"x": 1.0, "y": -1.2, "heading": 0.05, "speed": 4.0}},
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            output = work / "shadow.json"

            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/train_e2e/torch_sidecar.py"),
                    "--model",
                    str(model_dir),
                    "--state-file",
                    str(state_file),
                    "--output",
                    str(output),
                    "--once",
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            if importlib.util.find_spec("torch") is None:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("PyTorch is not installed", result.stderr)
                return

            self.assertEqual(result.returncode, 0, result.stderr)
            shadow = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(shadow["topic"], "inference/trajectory")
            self.assertTrue(shadow["shadow"])
            self.assertIn("target_speed", shadow)

    def test_v3_dataset_torch_and_sidecar_share_scene_features(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            source = work / "samples_v3.jsonl"
            source.write_text(
                "\n".join(
                    json.dumps(
                        {
                            "schema_version": "flowengine.e2e_sample.v3",
                            "t": 4000 + i,
                            "features_v3": [
                                float(i % 10),            # ego_v
                                -0.8 + 0.04 * i,          # ego_y
                                0.01 * i,                  # ego_heading
                                0.02,                      # ego_yaw_rate
                                20.0 + 0.2 * i,            # front0_x
                                0.5,                       # front0_y
                                -1.0,                      # front0_vx
                                1.0,                       # front0_type
                                0.8,                       # front0_confidence
                                10.0 + 0.05 * i,           # front1_x
                                0.2,                       # front1_y
                                0.0,                       # front1_vx
                                1.0,                       # front1_type
                                0.0,                       # front1_confidence
                                0.0,                       # control_brake
                                0.0,                       # control_emergency_stop
                                # v3 场景上下文 (7维)
                                0.0,                       # tl_state
                                -1.0,                      # tl_distance
                                0.0,                       # road_curvature
                                30.0,                      # road_speed_limit
                                2.0,                       # lane_count
                                3.5,                       # lane_width
                                -0.8 + 0.04 * i,           # ego_lane_offset
                            ],
                            "scene_context": {
                                "tl_state": 0.0,
                                "tl_distance": -1.0,
                                "curvature": 0.0,
                                "speed_limit": 30.0,
                                "lane_count": 2.0,
                                "lane_width": 3.5,
                                "ego_lane_offset": -0.8 + 0.04 * i,
                            },
                            "label": 7.0 + 0.1 * (i % 6),
                            "ego": {"x": 0.3 * i, "y": -0.8 + 0.04 * i},
                            "planning": {"target_speed": 7.0 + 0.1 * (i % 6)},
                            "control": {"throttle": 0.2, "brake": 0.0, "steering": 0.01 * i},
                            "obstacles": [
                                {"id": 1, "x": 20.0 + 0.2 * i, "y": 0.5, "vx": -1.0, "vy": 0.0, "type": 1, "confidence": 0.8},
                                {"id": 2, "x": 10.0 + 0.05 * i, "y": -0.2, "vx": 0.0, "vy": 0.0, "type": 2, "confidence": 0.7},
                            ],
                        }
                    )
                    for i in range(40)
                )
                + "\n",
                encoding="utf-8",
            )

            dataset = work / "dataset_v3"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/dataset/export_e2e_dataset.py"),
                    "--input",
                    str(source),
                    "--output",
                    str(dataset),
                ],
                cwd=ROOT,
                check=True,
            )

            metadata = json.loads((dataset / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["schema_version"], "flowengine.e2e_dataset.v3")
            self.assertEqual(metadata["sample_count"], 40)
            self.assertIn("front0_x", metadata["feature_names"])
            sample = json.loads((dataset / "samples.jsonl").read_text(encoding="utf-8").splitlines()[0])
            self.assertEqual(len(sample["features"]), len(metadata["feature_names"]))
            self.assertIn("obstacles", sample)
            self.assertIn("control", sample)

            artifact = work / "torch_artifact_v3"
            train_result = self.train_torch_artifact(dataset, artifact)
            if importlib.util.find_spec("torch") is None:
                self.assertNotEqual(train_result.returncode, 0)
                self.assertIn("PyTorch is not installed", train_result.stderr)
                return
            self.assertEqual(train_result.returncode, 0, train_result.stderr)

            manifest = json.loads((artifact / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["input_schema"]["features"], metadata["feature_names"])

            state_file = work / "flow_topology.json"
            state_file.write_text(
                json.dumps(
                    {
                        "timestamp": 1.0,
                        "metrics": {"vehicle": {"speed": 4.0, "target_speed": 8.0}},
                        "scene": {
                            "ego": {"x": 1.0, "y": -1.2, "heading": 0.05, "speed": 4.0},
                            "obstacles": [
                                {"id": 1, "x": 20.0, "y": 0.5, "vx": -1.0, "vy": 0.0, "type": 1, "confidence": 0.8},
                                {"id": 2, "x": 10.0, "y": -0.2, "vx": 0.0, "vy": 0.0, "type": 2, "confidence": 0.7},
                            ],
                        },
                        "control": {"throttle": 0.2, "brake": 0.0, "steering": 0.1, "emergency_stop": False},
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            output = work / "shadow_v3.json"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/train_e2e/torch_sidecar.py"),
                    "--model",
                    str(artifact),
                    "--state-file",
                    str(state_file),
                    "--output",
                    str(output),
                    "--once",
                ],
                cwd=ROOT,
                check=True,
            )
            shadow = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(len(shadow["features"]), len(metadata["feature_names"]))
            self.assertEqual(shadow["feature_names"], metadata["feature_names"])

    def test_modelctl_lists_and_promotes_tiny_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            runtime = work / "runtime" / "model.txt"
            runtime.parent.mkdir(parents=True)
            runtime.write_text("runtime-model\n", encoding="utf-8")

            artifact = work / "models" / "tiny_v001"
            artifact.mkdir(parents=True)
            (artifact / "model.txt").write_text("artifact-model\n", encoding="utf-8")
            (artifact / "manifest.json").write_text(
                json.dumps(
                    {
                        "backend": "tiny_mlp",
                        "model_format": "flowengine-tinymlp-v3",
                        "model_path": "model.txt",
                        "dataset": {"sample_count": 12, "scenario": "unit"},
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            list_result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/modelctl.py"),
                    "list",
                    "--runtime-model",
                    str(runtime),
                    "--models-dir",
                    str(work / "models"),
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(list_result.returncode, 0, list_result.stderr)
            self.assertIn("runtime tiny", list_result.stdout)
            self.assertIn("tiny_v001", list_result.stdout)

            promote_result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/modelctl.py"),
                    "promote",
                    str(artifact),
                    "--runtime-model",
                    str(runtime),
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(promote_result.returncode, 0, promote_result.stderr)
            self.assertEqual(runtime.read_text(encoding="utf-8"), "artifact-model\n")
            self.assertIn("promoted", promote_result.stdout)

    def test_train_demo_model_exports_trains_and_evaluates_tiny(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            source = self.make_sample_source(work, start_t=5000)
            datasets_dir = work / "datasets"
            models_dir = work / "models"

            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/train_demo_model.py"),
                    "--input",
                    str(source),
                    "--datasets-dir",
                    str(datasets_dir),
                    "--models-dir",
                    str(models_dir),
                    "--name",
                    "unit_tiny",
                    "--backend",
                    "tiny",
                    "--epochs",
                    "3",
                    "--hidden",
                    "4",
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((models_dir / "unit_tiny" / "model.txt").exists())
            self.assertIn("model: ", result.stdout)
            self.assertIn("next: python3 tools/modelctl.py list", result.stdout)

    def test_train_demo_model_can_finetune_from_existing_torch_artifact(self):
        if importlib.util.find_spec("torch") is None:
            self.skipTest("PyTorch is not installed")
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            source = self.make_sample_source(work, start_t=7000)
            datasets_dir = work / "datasets"
            models_dir = work / "models"

            base = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/train_demo_model.py"),
                    "--input",
                    str(source),
                    "--datasets-dir",
                    str(datasets_dir),
                    "--models-dir",
                    str(models_dir),
                    "--name",
                    "base_torch",
                    "--backend",
                    "torch",
                    "--epochs",
                    "1",
                    "--hidden",
                    "4",
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(base.returncode, 0, base.stderr)

            tuned = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/train_demo_model.py"),
                    "--input",
                    str(source),
                    "--datasets-dir",
                    str(datasets_dir),
                    "--models-dir",
                    str(models_dir),
                    "--name",
                    "tuned_torch",
                    "--backend",
                    "torch",
                    "--epochs",
                    "1",
                    "--hidden",
                    "4",
                    "--init-from",
                    str(models_dir / "base_torch"),
                ],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self.assertEqual(tuned.returncode, 0, tuned.stderr)
            manifest = json.loads((models_dir / "tuned_torch" / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["training"]["init_from"], str(models_dir / "base_torch"))

    def test_onnx_export_parity_with_tiny_mlp(self):
        """export_onnx.py 导出的 .onnx 与 tiny_mlp_forward 数值等价 (|Δ|<1e-3)。

        依赖 onnx + numpy + onnxruntime；缺任一即 skip（本机/CI 未预装 ORT 时不失败，
        与 ENABLE_ONNX 默认关、运行时降级 tiny-MLP 的策略一致）。
        """
        try:
            import numpy as np  # noqa: F401
            import onnx  # noqa: F401
            import onnxruntime as ort
        except ImportError as e:
            self.skipTest(f"onnx/onnxruntime/numpy 未安装，跳过 ONNX parity: {e}")

        import math
        sys.path.insert(0, str(ROOT / "tools/train_e2e"))
        from export_onnx import parse_tiny_mlp, build_onnx

        # 手写一个 in=4 / hidden=[3] / out=2 的 tiny-MLP，带非平凡归一化。
        def fmt(vals):
            return " ".join(f"{v:.8f}" for v in vals)

        w1 = [0.10, -0.20, 0.30, -0.05,   # 隐元0
              0.02, 0.40, -0.15, 0.25,    # 隐元1
              -0.30, 0.12, 0.08, -0.22]   # 隐元2
        b1 = [0.01, -0.02, 0.03]
        w2 = [0.5, -0.4, 0.2,   # 输出0
              -0.1, 0.3, 0.6]   # 输出1
        b2 = [0.05, -0.05]
        norm_mean = [10.0, -1.0, 0.0, 0.0]
        norm_scale = [5.0, 2.0, 1.0, 0.5]
        out_mean = [8.0, 0.0]
        out_scale = [3.0, 1.5]

        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            model_txt = work / "model.txt"
            model_txt.write_text(
                "# flowengine-tinymlp v2 (parity test)\n"
                "in 4\nhidden 3\nout 2\n"
                f"norm_mean {fmt(norm_mean)}\n"
                f"norm_scale {fmt(norm_scale)}\n"
                f"out_mean {fmt(out_mean)}\n"
                f"out_scale {fmt(out_scale)}\n"
                f"w1 {fmt(w1)}\nb1 {fmt(b1)}\n"
                f"w2 {fmt(w2)}\nb2 {fmt(b2)}\n",
                encoding="utf-8",
            )

            onnx_path = work / "model.onnx"
            build_onnx(parse_tiny_mlp(model_txt), onnx_path)
            self.assertTrue(onnx_path.exists())

            # 参考：tiny_mlp.h::tiny_mlp_forward 的纯 Python 复刻（norm→tanh 层→gemm→denorm）
            def tiny_mlp_ref(x):
                xn = [(x[i] - norm_mean[i]) / (norm_scale[i] or 1.0) for i in range(4)]
                h = []
                for j in range(3):
                    acc = b1[j] + sum(w1[j * 4 + i] * xn[i] for i in range(4))
                    h.append(math.tanh(acc))
                y = []
                for k in range(2):
                    acc = b2[k] + sum(w2[k * 3 + j] * h[j] for j in range(3))
                    y.append(acc * out_scale[k] + out_mean[k])
                return y

            sess = ort.InferenceSession(str(onnx_path),
                                        providers=["CPUExecutionProvider"])
            in_name = sess.get_inputs()[0].name
            for x in ([12.0, -0.5, 0.1, 0.2],
                      [8.0, -2.0, -0.3, 0.0],
                      [15.0, 1.0, 0.05, -0.1]):
                ref = tiny_mlp_ref(x)
                got = sess.run(None, {in_name: np.array([x], dtype=np.float32)})[0][0]
                for k in range(2):
                    self.assertLess(abs(float(got[k]) - ref[k]), 1e-3,
                                    f"onnx vs tiny_mlp mismatch @out{k}: "
                                    f"{got[k]} vs {ref[k]} (x={x})")


if __name__ == "__main__":
    unittest.main()