#!/usr/bin/env python3
"""export_onnx.py — 把 tiny-MLP 权重文件 (model.txt) 导出成等价 ONNX 图。

设计目标：**同权重、同特征契约、同数值**。导出的 .onnx 接收与 tiny_mlp_forward
完全相同的原始特征 x（未归一化），输出相同的原始 y（已反归一化）——归一化 /
反归一化都折进计算图，故 inference_node 的 onnx_backend 与 tiny-MLP 后端喂同一份
特征、取同一份输出，无需在 C 侧区分。

图结构逐位镜像 tiny_mlp.h::tiny_mlp_forward：
    xn      = (x - norm_mean) / norm_scale          # Sub, Div
    h_0     = Tanh(xn · W1^T + b1)                   # Gemm(transB=1), Tanh
    h_i     = Tanh(h_{i-1} · W{i+1}^T + b{i+1})      # 每隐层
    y_norm  = h_last · W_out^T + b_out               # Gemm(transB=1)
    y       = y_norm * out_scale + out_mean          # Mul, Add

依赖：onnx + numpy（`pip install onnx numpy`）。仅导出需要；运行时推理由 C 侧
onnxruntime 负责。未安装时给出明确提示并非零退出（不静默产出坏文件）。

用法：
    python3 tools/train_e2e/export_onnx.py --model models/xxx/model.txt \
            --output models/xxx/model.onnx
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def parse_tiny_mlp(path: Path):
    """解析 model.txt，返回与 tiny_mlp.h 语义一致的权重字典。

    关键兼容点（同 tiny_mlp.h）：hidden_count==1 时 `w2/b2` 标签解读为输出层，
    否则 `w{i+1}` 为隐层 i、`w_out/b_out` 为输出层。
    """
    in_dim = out_dim = 0
    hidden_dims: list[int] = []
    data: dict[str, list[float]] = {}
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0]
        if key == "in":
            in_dim = int(parts[1])
        elif key == "out":
            out_dim = int(parts[1])
        elif key == "hidden":
            hidden_dims = [int(v) for v in parts[1:]]
        else:
            data[key] = [float(v) for v in parts[1:]]

    if in_dim <= 0 or out_dim <= 0 or not hidden_dims:
        raise SystemExit(f"error: 非法模型头 (in={in_dim} out={out_dim} hidden={hidden_dims})")

    hidden_count = len(hidden_dims)

    def take(key: str, n: int) -> list[float]:
        # 与 tiny_mlp.h 一致：缺失/过短的权重按 0 补齐（loader memset 为 0），
        # 过长则截断。全零占位模型 (w1/b1 空行) 因此合法。
        v = data.get(key, [])
        return (v + [0.0] * n)[:n]

    norm_mean = data.get("norm_mean", [0.0] * in_dim)[:in_dim]
    norm_scale = data.get("norm_scale", [1.0] * in_dim)[:in_dim]
    out_mean = data.get("out_mean", [0.0] * out_dim)[:out_dim]
    out_scale = data.get("out_scale", [1.0] * out_dim)[:out_dim]
    # 补齐 + 把 0 scale 视作 1（与 C 的 scale!=0?scale:1 一致）
    norm_mean = (norm_mean + [0.0] * in_dim)[:in_dim]
    norm_scale = [(s if s != 0.0 else 1.0) for s in (norm_scale + [1.0] * in_dim)[:in_dim]]
    out_mean = (out_mean + [0.0] * out_dim)[:out_dim]
    out_scale = [(s if s != 0.0 else 1.0) for s in (out_scale + [1.0] * out_dim)[:out_dim]]

    # 隐层权重 [hid][prev]
    layers = []
    prev = in_dim
    for li in range(hidden_count):
        hd = hidden_dims[li]
        w = take(f"w{li+1}", hd * prev)
        b = take(f"b{li+1}", hd)
        layers.append((hd, prev, w, b))
        prev = hd

    # 输出层：单隐层兼容 w2/b2，多隐层用 w_out/b_out
    if hidden_count == 1 and "w_out" not in data:
        w_out = take("w2", out_dim * prev)
        b_out = take("b2", out_dim)
    else:
        w_out = take("w_out", out_dim * prev)
        b_out = take("b_out", out_dim)

    return {
        "in_dim": in_dim, "out_dim": out_dim, "hidden_dims": hidden_dims,
        "norm_mean": norm_mean, "norm_scale": norm_scale,
        "out_mean": out_mean, "out_scale": out_scale,
        "layers": layers, "w_out": w_out, "b_out": b_out, "last_hid": prev,
    }


def build_onnx(m: dict, output: Path) -> None:
    try:
        import numpy as np
        import onnx
        from onnx import TensorProto, helper, numpy_helper
    except ImportError:
        raise SystemExit(
            "error: 需要 onnx + numpy 才能导出（pip install onnx numpy）。\n"
            "       运行时推理由 C 侧 onnxruntime 负责，此依赖仅导出用。"
        )

    in_dim, out_dim = m["in_dim"], m["out_dim"]
    inits = []
    nodes = []

    def const(name: str, arr) -> str:
        t = numpy_helper.from_array(np.asarray(arr, dtype=np.float32), name)
        inits.append(t)
        return name

    # 归一化: xn = (x - norm_mean) / norm_scale
    const("norm_mean", m["norm_mean"])
    const("norm_scale", m["norm_scale"])
    nodes.append(helper.make_node("Sub", ["input", "norm_mean"], ["xn_sub"]))
    nodes.append(helper.make_node("Div", ["xn_sub", "norm_scale"], ["h0"]))

    prev_name = "h0"
    for li, (hd, prev, w, b) in enumerate(m["layers"]):
        W = np.asarray(w, dtype=np.float32).reshape(hd, prev)   # [hid][prev]
        const(f"W{li}", W)
        const(f"B{li}", np.asarray(b, dtype=np.float32))
        gemm_out = f"gemm{li}"
        # Gemm transB=1: X[1,prev]·W^T[prev,hid] + B  → [1,hid]
        nodes.append(helper.make_node(
            "Gemm", [prev_name, f"W{li}", f"B{li}"], [gemm_out], transB=1))
        act = f"act{li}"
        nodes.append(helper.make_node("Tanh", [gemm_out], [act]))
        prev_name = act

    # 输出层 (无激活)
    last_hid = m["last_hid"]
    Wo = np.asarray(m["w_out"], dtype=np.float32).reshape(out_dim, last_hid)
    const("Wout", Wo)
    const("Bout", np.asarray(m["b_out"], dtype=np.float32))
    nodes.append(helper.make_node(
        "Gemm", [prev_name, "Wout", "Bout"], ["ynorm"], transB=1))

    # 反归一化: y = ynorm * out_scale + out_mean
    const("out_scale", m["out_scale"])
    const("out_mean", m["out_mean"])
    nodes.append(helper.make_node("Mul", ["ynorm", "out_scale"], ["ymul"]))
    nodes.append(helper.make_node("Add", ["ymul", "out_mean"], ["output"]))

    x_in = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, in_dim])
    y_out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, out_dim])
    graph = helper.make_graph(nodes, "tiny_mlp", [x_in], [y_out], inits)
    model = helper.make_model(graph, producer_name="flowengine-export-onnx",
                              opset_imports=[helper.make_opsetid("", 13)])
    # 新版 onnx 默认 IR 版本可能超出已发布 onnxruntime 的上限（如 onnx1.19→IR12
    # 而 ort1.19 仅支持到 IR10），显式降到 10 以最大化运行时兼容。opset13 的算子
    # （Gemm/Tanh/Sub/Div/Mul/Add）在 IR10 下均合法。
    model.ir_version = 10
    onnx.checker.check_model(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output))
    print(f"ONNX exported: {output} (in={in_dim} out={out_dim} "
          f"hidden={m['hidden_dims']})", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description="tiny-MLP model.txt → ONNX 导出（等价数值）")
    ap.add_argument("--model", required=True, help="输入 tiny-MLP model.txt")
    ap.add_argument("--output", required=True, help="输出 model.onnx")
    args = ap.parse_args()

    m = parse_tiny_mlp(Path(args.model))
    build_onnx(m, Path(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
