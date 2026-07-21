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
                            "features": [float(i % 8), -1.5 + 0.1 * i, 0.01 * i, 0.0],
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
            self.assertEqual(metadata["schema_version"], "flowengine.e2e_dataset.v1")

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
                        "features": [float(i % 10), -1.0 + 0.05 * i, 0.005 * i, 0.01],
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

    def test_v2_dataset_torch_and_sidecar_share_scene_features(self):
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            source = work / "samples_v2.jsonl"
            source.write_text(
                "\n".join(
                    json.dumps(
                        {
                            "schema_version": "flowengine.e2e_sample.v2",
                            "t": 4000 + i,
                            "features": [float(i % 10), -0.8 + 0.04 * i, 0.01 * i, 0.02],
                            "features_v2": [
                                float(i % 10),
                                -0.8 + 0.04 * i,
                                0.01 * i,
                                0.02,
                                20.0 + 0.2 * i,
                                0.5,
                                -1.0,
                                1.0,
                                0.8,
                                10.0 + 0.05 * i,
                                0.2,
                                0.0,
                                1.0,
                                0.0,
                                0.0,
                            ],
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

            dataset = work / "dataset_v2"
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
            self.assertEqual(metadata["schema_version"], "flowengine.e2e_dataset.v2")
            self.assertEqual(metadata["sample_count"], 40)
            self.assertIn("front0_x", metadata["feature_names"])
            sample = json.loads((dataset / "samples.jsonl").read_text(encoding="utf-8").splitlines()[0])
            self.assertEqual(len(sample["features"]), len(metadata["feature_names"]))
            self.assertIn("obstacles", sample)
            self.assertIn("control", sample)

            artifact = work / "torch_artifact_v2"
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
            output = work / "shadow_v2.json"
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


if __name__ == "__main__":
    unittest.main()