#!/usr/bin/env python3
"""make_torch_dataset.py — 采集 JSONL → PyTorch dataset 目录

把 data_recorder_node 的采集样本（/tmp/flow_train_samples.jsonl）转成
torch_train.py 的 load_dataset 目录格式（samples.jsonl + metadata.json）。

用法:
  python3 tools/train_e2e/make_torch_dataset.py \
      --input /tmp/flow_train_samples.jsonl --output /tmp/torch_ds
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from feature_schema import FEATURE_NAMES_V3, LABEL_NAMES  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="JSONL → torch dataset")
    ap.add_argument("--input", required=True, help="采集 JSONL")
    ap.add_argument("--output", required=True, help="输出 dataset 目录")
    ap.add_argument("--features", default="v3", choices=["v2", "v3"],
                    help="特征集（v3=23 维含场景上下文）")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    feature_names = FEATURE_NAMES_V3 if args.features == "v3" else None
    n_feat = len(feature_names)
    n = 0
    with in_path.open("r", encoding="utf-8") as src, \
         (out_dir / "samples.jsonl").open("w", encoding="utf-8") as dst:
        for line in src:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if args.features == "v3":
                x = obj.get("features_v3")
            else:
                x = obj.get("features_v2") or obj.get("features")
            y = obj.get("label")
            if not isinstance(x, list) or len(x) != n_feat or y is None:
                continue
            dst.write(json.dumps({"features": x, "label": float(y)}) + "\n")
            n += 1

    if n < 10:
        print(f"error: 有效样本仅 {n} 条", file=sys.stderr)
        return 1

    metadata = {
        "schema_version": "flowengine.e2e_dataset.v1",
        "feature_names": feature_names,
        "label_names": LABEL_NAMES,
        "sample_count": n,
        "source": str(in_path),
    }
    (out_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"dataset: {n} samples → {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
