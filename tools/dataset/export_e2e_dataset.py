#!/usr/bin/env python3
"""Export recorder JSONL into a versioned E2E dataset directory."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "train_e2e"))

from feature_schema import (  # noqa: E402
    DATASET_SCHEMA_V1,
    DATASET_SCHEMA_V2,
    FEATURE_NAMES_V1,
    FEATURE_NAMES_V2,
    LABEL_NAMES,
    features_from_sample,
    sample_feature_names,
)


def iter_samples(path: Path):
    with path.open("r", encoding="utf-8") as src:
        for line_no, line in enumerate(src, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            label = obj.get("label")
            if label is None:
                continue
            feature_names = sample_feature_names(obj)
            try:
                features = features_from_sample(obj, feature_names)
            except ValueError:
                continue
            yield {
                "t": int(obj.get("t", 0) or 0),
                "schema_version": obj.get("schema_version", "flowengine.e2e_sample.v1"),
                "features": features,
                "label": float(label),
                "ego": obj.get("ego", {}),
                "planning": obj.get("planning", {}),
                "control": obj.get("control", {}),
                "obstacles": obj.get("obstacles", []),
                "source_line": line_no,
            }


def export_dataset(input_path: Path, output_dir: Path, scenario: str | None) -> dict:
    samples = list(iter_samples(input_path))
    if not samples:
        raise SystemExit(f"error: no valid samples in {input_path}")
    feature_names = FEATURE_NAMES_V1
    schema_version = DATASET_SCHEMA_V1
    if len(samples[0]["features"]) != len(FEATURE_NAMES_V1):
        feature_names = FEATURE_NAMES_V2
        schema_version = DATASET_SCHEMA_V2

    output_dir.mkdir(parents=True, exist_ok=True)
    samples_path = output_dir / "samples.jsonl"
    with samples_path.open("w", encoding="utf-8") as dst:
        for sample in samples:
            dst.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")

    metadata = {
        "schema_version": schema_version,
        "created_unix_ms": int(time.time() * 1000),
        "source": str(input_path),
        "sample_count": len(samples),
        "feature_names": feature_names,
        "label_names": LABEL_NAMES,
        "scenario": scenario or "unknown",
        "format": "jsonl",
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    # Keep a raw copy for reproducibility/debugging. Ignore same-file copies.
    raw_path = output_dir / "raw_samples.jsonl"
    if input_path.resolve() != raw_path.resolve():
        shutil.copyfile(input_path, raw_path)

    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(description="Export FlowEngine recorder JSONL to an E2E dataset")
    parser.add_argument("--input", default="/tmp/flow_train_samples.jsonl")
    parser.add_argument("--output", required=True)
    parser.add_argument("--scenario", default=None)
    args = parser.parse_args()

    metadata = export_dataset(Path(args.input), Path(args.output), args.scenario)
    print(f"dataset exported: {args.output} ({metadata['sample_count']} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())