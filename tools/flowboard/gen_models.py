#!/usr/bin/env python3
"""
gen_models.py — FlowEngine 3D 车辆模型生成器

生成 glTF 2.0 格式的简化车辆/行人模型，用于 flowboard 3D 场景。
产物为 tools/flowboard/models/*.gltf，可直接被 Three.js GLTFLoader 加载。

模型风格：低多边形 + PBR 材质，适合 ADAS 俯视/追尾视角。
面数：每车 ~200-500 三角面，64 辆车同屏 GPU 无压力。

用法:
  python3 tools/flowboard/gen_models.py              # 生成所有模型
  python3 tools/flowboard/gen_models.py --validate    # 检查已有模型完整性
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import struct
import sys
from pathlib import Path

MODELS_DIR = Path(__file__).parent / "models"


# ── 极简二进制缓存区构建器 ──────────────────────────────────

class BufferBuilder:
    """按 float32 组块构建二进制缓冲区，输出为 base64 字符串。"""

    def __init__(self):
        self.data = bytearray()

    def add_floats(self, vals: list[float]) -> int:
        offset = len(self.data)
        for v in vals:
            self.data.extend(struct.pack("<f", v))
        return offset

    def add_u16s(self, vals: list[int]) -> int:
        offset = len(self.data)
        for v in vals:
            self.data.extend(struct.pack("<H", v))
        return offset

    def to_base64(self) -> str:
        return base64.b64encode(bytes(self.data)).decode("ascii")

    def byte_length(self) -> int:
        return len(self.data)


# ── 几何体构造 ──────────────────────────────────────────────

def box_vertices(cx: float, cy: float, cz: float,
                 sx: float, sy: float, sz: float) -> list[float]:
    """以 (cx,cy,cz) 为中心，(sx,sy,sz) 为半边的长方体顶点 (24个顶点, 6面×4)"""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    # 每个面4个顶点，6个面
    verts = []
    # 面法线: +x, -x, +y, -y, +z, -z
    faces = [
        ([1,1,1, 1,-1,1, 1,-1,-1, 1,1,-1], [1,0,0]),   # +x
        ([-1,1,-1, -1,-1,-1, -1,-1,1, -1,1,1], [-1,0,0]), # -x
        ([1,1,-1, 1,1,1, -1,1,1, -1,1,-1], [0,1,0]),     # +y
        ([1,-1,1, 1,-1,-1, -1,-1,-1, -1,-1,1], [0,-1,0]), # -y
        ([-1,1,1, -1,-1,1, 1,-1,1, 1,1,1], [0,0,1]),     # +z
        ([1,1,-1, 1,-1,-1, -1,-1,-1, -1,1,-1], [0,0,-1]), # -z
    ]
    for signs, normal in faces:
        for i in range(0, 12, 3):
            vx = cx + signs[i] * hx
            vy = cy + signs[i+1] * hy
            vz = cz + signs[i+2] * hz
            verts.extend([vx, vy, vz, normal[0], normal[1], normal[2]])
    return verts  # 144 floats = 24 vertices × 6 components

def box_indices(base: int) -> list[int]:
    """长方体三角形索引（每个面2个三角形, 6个面 = 12个三角形 = 36 索引）"""
    tris = []
    for f in range(6):
        b = base + f * 4
        tris.extend([b, b+1, b+2,  b, b+2, b+3])
    return tris  # 36 u16s


def cylinder_vertices(cx: float, cy: float, cz: float,
                      radius: float, height: float, axis: str = "z",
                      segments: int = 16) -> tuple[list[float], list[int]]:
    """圆柱体顶点 + 索引（侧面 + 两端面），轴沿 z（车轮横向）。

    返回 (vertices, indices)，vertices 格式与 box_vertices 一致：
    每顶点 6 float (x,y,z, nx,ny,nz)。
    axis='z' 时圆柱轴沿 Z（适合车轮，旋转 rotation.x 实现滚动）。
    """
    sx, sy, sz = 0.0, 0.0, 0.0
    if axis == "z":
        sz = 1.0
    elif axis == "x":
        sx = 1.0
    else:
        sy = 1.0
    hz = height / 2
    verts: list[float] = []
    # 侧面顶点：2 * segments 个（每端 segments 个）
    for end in (-1, 1):  # -1 = 负端, +1 = 正端
        for i in range(segments):
            ang = (i / segments) * 2 * 3.14159265358979
            # 在垂直于 axis 的平面内取圆
            if axis == "z":
                px = math.cos(ang) * radius
                py = math.sin(ang) * radius
                pz = end * hz
                nx, ny, nz = math.cos(ang), math.sin(ang), 0.0
            elif axis == "x":
                py = math.cos(ang) * radius
                pz = math.sin(ang) * radius
                px = end * hz
                nx, ny, nz = 0.0, math.cos(ang), math.sin(ang)
            else:  # y
                px = math.cos(ang) * radius
                pz = math.sin(ang) * radius
                py = end * hz
                nx, ny, nz = math.cos(ang), 0.0, math.sin(ang)
            verts.extend([cx + px, cy + py, cz + pz, nx, ny, nz])
    base = 0  # 相对索引，调用方需加偏移
    idx: list[int] = []
    # 侧面：每段 2 三角形
    for i in range(segments):
        i0 = i
        i1 = (i + 1) % segments
        i2 = segments + i
        i3 = segments + ((i + 1) % segments)
        # 两端绕向相反以保证外法线朝外
        idx.extend([i0, i2, i1,  i1, i2, i3])
    # 端面 1（负端，顶点 0..segments-1）三角扇
    center_neg = segments * 2
    verts.extend([cx - sx * hz, cy - sy * hz, cz - sz * hz,
                  -sx, -sy, -sz])
    for i in range(segments):
        i0 = i
        i1 = (i + 1) % segments
        idx.extend([center_neg, i1, i0])
    # 端面 2（正端，顶点 segments..2*segments-1）三角扇
    center_pos = segments * 2 + 1
    verts.extend([cx + sx * hz, cy + sy * hz, cz + sz * hz,
                  sx, sy, sz])
    for i in range(segments):
        i0 = segments + i
        i1 = segments + ((i + 1) % segments)
        idx.extend([center_pos, i0, i1])
    return verts, idx

def merge_meshes(parts: list[tuple[list[float], list[int]]]) -> tuple[list[float], list[int]]:
    """合并多个 (vertices, indices) 为一个。"""
    all_verts, all_idx, offset = [], [], 0
    for verts, idx in parts:
        vc = len(verts) // 6
        all_verts.extend(verts)
        all_idx.extend([i + offset for i in idx])
        offset += vc
    return all_verts, all_idx


def build_pedestrian() -> tuple[list[float], list[int]]:
    """行人: 躯干 + 头部 + 双腿"""
    parts = []
    # 躯干 (0.5m宽 × 0.6m高 × 0.3m厚)
    parts.append(box_vertices(0.0, 0.6, 0.0, 0.3, 0.6, 0.5))
    # 头部 (0.25m球) → 方块近似
    parts.append(box_vertices(0.0, 1.1, 0.0, 0.28, 0.28, 0.28))
    # 腿 x 2
    for lx in [-0.12, 0.12]:
        parts.append(box_vertices(lx, 0.2, 0.0, 0.12, 0.4, 0.14))
    return merge_meshes(parts)


# ── 材质定义 ──────────────────────────────────────────────────

MATERIALS = {
    "car_paint": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.2, 0.4, 0.85, 1.0],    # 蓝色金属漆
            "metallicFactor": 0.85,
            "roughnessFactor": 0.3,
        },
    },
    "truck_paint": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.85, 0.2, 0.15, 1.0],   # 红色
            "metallicFactor": 0.6,
            "roughnessFactor": 0.5,
        },
    },
    "suv_paint": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.2, 0.2, 0.2, 1.0],     # 黑色
            "metallicFactor": 0.8,
            "roughnessFactor": 0.25,
        },
    },
    # ── Task 6：小米 SU7 专属车漆 ──
    # 海湾蓝（SU7 招牌色）：深蓝高金属感，低粗糙度（光泽漆面）。
    # 与 sedan 的普通蓝色金属漆区分，凸显 ego 切换为 SU7 后的视觉差异。
    "su7_paint": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.10, 0.32, 0.55, 1.0],   # 海湾蓝
            "metallicFactor": 0.9,
            "roughnessFactor": 0.20,
        },
    },
    # SU7 贯穿式尾灯条：单一连续 LED 光带，跨整个车尾。
    # 复用 brakelight emissive 调控，但单独定义以便 baseColor 微调（更深红）。
    "su7_taillight_bar": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.45, 0.04, 0.04, 1.0],
            "metallicFactor": 0.1,
            "roughnessFactor": 0.30,
        },
        "emissiveFactor": [0.15, 0.0, 0.0],
    },
    "glass": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.7, 0.8, 0.95, 0.5],    # 半透明浅蓝
            "metallicFactor": 0.1,
            "roughnessFactor": 0.05,
        },
        "alphaMode": "BLEND",
        "doubleSided": True,
    },
    "tire": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.08, 0.08, 0.08, 1.0],  # 深灰
            "metallicFactor": 0.0,
            "roughnessFactor": 0.95,
        },
    },
    "pedestrian_body": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.6, 0.5, 0.4, 1.0],     # 棕色
            "metallicFactor": 0.0,
            "roughnessFactor": 0.8,
        },
    },
    "pedestrian_head": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.9, 0.8, 0.7, 1.0],     # 浅肤色
            "metallicFactor": 0.0,
            "roughnessFactor": 0.6,
        },
    },
    # ── 灯光材质（运行时通过 material.emissiveIntensity 切换亮灭）──
    # 默认 emissiveFactor 较暗（灭灯态），VehicleView.js 点亮时拉高 emissiveIntensity
    "headlight": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.85, 0.85, 0.8, 1.0],
            "metallicFactor": 0.2,
            "roughnessFactor": 0.2,
        },
        "emissiveFactor": [0.5, 0.5, 0.45],
    },
    "brakelight": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.5, 0.05, 0.05, 1.0],
            "metallicFactor": 0.1,
            "roughnessFactor": 0.35,
        },
        "emissiveFactor": [0.12, 0.0, 0.0],
    },
    "turnsignal": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.85, 0.50, 0.05, 1.0],
            "metallicFactor": 0.1,
            "roughnessFactor": 0.25,
        },
        "emissiveFactor": [0.35, 0.18, 0.0],
    },
    # 自动驾驶小蓝灯（车尾 x2，量产 ADS 指示灯，始终亮）
    "ads_indicator": {
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.15, 0.45, 0.95, 1.0],
            "metallicFactor": 0.2,
            "roughnessFactor": 0.10,
        },
        "emissiveFactor": [0.15, 0.50, 0.95],
    },
}


# ── 模型规格 ──────────────────────────────────────────────────

MODEL_SPECS = [
    # ── sedan 精致版：车身件 + 4 车门 + 充电盖 + 雨刮器 + 后备箱盖 +
    #    前大灯/后刹车灯/4 转向灯 + 圆柱车轮。车头朝 +x（与 _buildSedan 一致）。
    #    灯节点命名约定被 models.js 扫描建立 userData.brakeLights /
    #    turnSignals / headlights，VehicleView.js 通过 emissiveIntensity 切换亮灭。
    #    Task A: 压低车身/座舱，目标 L=4.6 H=1.45 W=1.8 L:H≈3.2。
    {
        "name": "sedan",
        "materials": ["car_paint", "glass", "tire", "headlight", "brakelight", "turnsignal", "ads_indicator"],
        "parts": [
            # 车身件 (car_paint, mat 0) — 目标 L=4.6 H=1.45 W=1.80 L:H≈3.2
            # body: cy=0.53 sy=0.52 → top=0.79 bottom=0.27
            {"name": "body",     "mat": 0, "build_fn": lambda: box_vertices(0.0,   0.53,  0.0,  4.6,  0.52, 1.80)},
            # cabin: cy=1.19 sy=0.52 → top=1.45 (= 目标 H)
            {"name": "cabin",    "mat": 0, "build_fn": lambda: box_vertices(0.05,  1.19,  0.0,  2.2,  0.52, 1.50)},
            {"name": "hood",     "mat": 0, "build_fn": lambda: box_vertices(1.50,  0.80,  0.0,  1.4,  0.06, 1.50)},
            {"name": "trunklid", "mat": 0, "build_fn": lambda: box_vertices(-1.60, 0.80,  0.0,  1.1,  0.05, 1.48)},
            # 4 车门（薄板贴车身侧面）
            {"name": "door_FL",  "mat": 0, "build_fn": lambda: box_vertices(0.7,   0.53,  0.91, 1.1,  0.62, 0.04)},
            {"name": "door_FR",  "mat": 0, "build_fn": lambda: box_vertices(0.7,   0.53, -0.91, 1.1,  0.62, 0.04)},
            {"name": "door_RL",  "mat": 0, "build_fn": lambda: box_vertices(-0.5,  0.53,  0.91, 1.0,  0.62, 0.04)},
            {"name": "door_RR",  "mat": 0, "build_fn": lambda: box_vertices(-0.5,  0.53, -0.91, 1.0,  0.62, 0.04)},
            # 充电口盖板（前保险杠左侧小盖板，EV 风格）
            {"name": "chargeport_cover", "mat": 0, "build_fn": lambda: box_vertices(2.15, 0.53,  0.62, 0.18, 0.16, 0.02)},
            # 雨刮器臂 x 2（前挡风玻璃上，压低）
            {"name": "wiper_L",  "mat": 0, "build_fn": lambda: box_vertices(1.05,  1.05,  0.35, 0.5,  0.02, 0.04)},
            {"name": "wiper_R",  "mat": 0, "build_fn": lambda: box_vertices(1.05,  1.05, -0.35, 0.5,  0.02, 0.04)},
            # 玻璃 (glass, mat 1)
            {"name": "windshield",  "mat": 1, "build_fn": lambda: box_vertices(1.10,  1.00, 0.0, 0.06, 0.36, 1.45)},
            {"name": "rear_window",  "mat": 1, "build_fn": lambda: box_vertices(-1.05, 0.96, 0.0, 0.06, 0.30, 1.38)},
            # 前大灯 (headlight, mat 3) — 白色发光
            {"name": "headlight_L", "mat": 3, "build_fn": lambda: box_vertices(2.18, 0.55,  0.58, 0.10, 0.16, 0.42)},
            {"name": "headlight_R", "mat": 3, "build_fn": lambda: box_vertices(2.18, 0.55, -0.58, 0.10, 0.16, 0.42)},
            # 后刹车灯 (brakelight, mat 4) — 红色发光
            {"name": "brakelight_L", "mat": 4, "build_fn": lambda: box_vertices(-2.18, 0.60,  0.55, 0.10, 0.14, 0.42)},
            {"name": "brakelight_R", "mat": 4, "build_fn": lambda: box_vertices(-2.18, 0.60, -0.55, 0.10, 0.14, 0.42)},
            # 转向灯 (turnsignal, mat 5) — 橙色发光，前 + 后 × 左右
            {"name": "turnsignal_FL", "mat": 5, "build_fn": lambda: box_vertices(2.16,  0.55,  0.82, 0.08, 0.14, 0.12)},
            {"name": "turnsignal_FR", "mat": 5, "build_fn": lambda: box_vertices(2.16,  0.55, -0.82, 0.08, 0.14, 0.12)},
            {"name": "turnsignal_RL", "mat": 5, "build_fn": lambda: box_vertices(-2.16, 0.60,  0.82, 0.08, 0.14, 0.12)},
            {"name": "turnsignal_RR", "mat": 5, "build_fn": lambda: box_vertices(-2.16, 0.60, -0.82, 0.08, 0.14, 0.12)},
            # 自动驾驶小蓝灯 ×2 (ads_indicator, mat 6) — 车尾左右，始终亮
            {"name": "ads_indicator_L", "mat": 6, "build_fn": lambda: cylinder_vertices(-1.75, 0.80,  0.48, 0.07, 0.08, "y", 12)},
            {"name": "ads_indicator_R", "mat": 6, "build_fn": lambda: cylinder_vertices(-1.75, 0.80, -0.48, 0.07, 0.08, "y", 12)},
            # ── 车辆转向系统：前/后轴 Group（pivot 在轴心，无 mesh）──
            {"name": "axle_front", "translation": [ 1.35, 0.33, 0.0]},
            {"name": "axle_rear",  "translation": [-1.35, 0.33, 0.0]},
            # 车轮（圆柱，轴沿 Z 横向，24 段→更圆滑；几何居中在原点 cylinder_vertices(0,0,0,...)，
            #       挂在 axle 节点下，translation = 相对轴心的偏移）。
            {"name": "wheel_FL", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0,  0.85],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.33, 0.26, "z", 24)},
            {"name": "wheel_FR", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0, -0.85],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.33, 0.26, "z", 24)},
            {"name": "wheel_RL", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0,  0.85],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.33, 0.26, "z", 24)},
            {"name": "wheel_RR", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0, -0.85],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.33, 0.26, "z", 24)},
        ],
    },
    # ── Task 6：su7 — 小米 SU7（运动轿车，低趴溜背造型 + 贯穿尾灯 + 车顶激光雷达凸起）
    # 真实尺寸 4997×1963×1455mm，轴距 3000mm。此处按 1:1 米制构建。
    # Task A: 目标 L=5.0 H=1.45 W=1.9 L:H≈3.4
    {
        "name": "su7",
        "materials": ["su7_paint", "glass", "tire", "headlight", "brakelight", "turnsignal",
                      "ads_indicator", "su7_taillight_bar"],
        "parts": [
            # 车身件 (su7_paint, mat 0) — 低趴运动造型，L=5.0
            {"name": "body",       "mat": 0, "build_fn": lambda: box_vertices( 0.00, 0.45,  0.00, 5.00, 0.55, 1.92)},
            # 引擎盖（下压式）
            {"name": "hood",       "mat": 0, "build_fn": lambda: box_vertices( 1.65, 0.75,  0.00, 1.40, 0.06, 1.78)},
            # 驾驶舱（溜背，cy=1.20 sy=0.50 → top=1.45 = 目标 H）
            {"name": "cabin",      "mat": 0, "build_fn": lambda: box_vertices(-0.05, 1.20,  0.00, 2.30, 0.50, 1.55)},
            # 后备厢盖（fastback 溜背）
            {"name": "trunklid",   "mat": 0, "build_fn": lambda: box_vertices(-1.95, 0.78,  0.00, 0.85, 0.06, 1.78)},
            # 后扰流板唇（小鸭尾）
            {"name": "spoiler",    "mat": 0, "build_fn": lambda: box_vertices(-2.40, 0.82,  0.00, 0.10, 0.04, 1.55)},
            # 前唇 splitter
            {"name": "splitter",   "mat": 0, "build_fn": lambda: box_vertices( 2.48, 0.22,  0.00, 0.08, 0.04, 1.85)},
            # 车顶 LiDAR 凸起（压扁到 2cm，不超 cabin 顶）
            {"name": "lidar_bump", "mat": 0, "build_fn": lambda: box_vertices(-0.05, 1.43,  0.00, 0.18, 0.02, 0.20)},
            # 侧裙 (左右)
            {"name": "side_skirt_L", "mat": 0, "build_fn": lambda: box_vertices( 0.00, 0.25,  0.96, 3.20, 0.05, 0.04)},
            {"name": "side_skirt_R", "mat": 0, "build_fn": lambda: box_vertices( 0.00, 0.25, -0.96, 3.20, 0.05, 0.04)},
            # 玻璃 (glass, mat 1)
            {"name": "windshield",  "mat": 1, "build_fn": lambda: box_vertices( 1.05, 0.95,  0.00, 0.06, 0.38, 1.48)},
            {"name": "rear_window",  "mat": 1, "build_fn": lambda: box_vertices(-1.20, 0.90,  0.00, 0.06, 0.32, 1.42)},
            {"name": "side_window_L", "mat": 1, "build_fn": lambda: box_vertices( 0.10, 1.05,  0.78, 1.85, 0.22, 0.04)},
            {"name": "side_window_R", "mat": 1, "build_fn": lambda: box_vertices( 0.10, 1.05, -0.78, 1.85, 0.22, 0.04)},
            # 前大灯 (headlight, mat 3)
            {"name": "headlight_L", "mat": 3, "build_fn": lambda: box_vertices( 2.48, 0.65,  0.62, 0.10, 0.14, 0.52)},
            {"name": "headlight_R", "mat": 3, "build_fn": lambda: box_vertices( 2.48, 0.65, -0.62, 0.10, 0.14, 0.52)},
            # 后刹车灯 (brakelight, mat 4)
            {"name": "brakelight_L", "mat": 4, "build_fn": lambda: box_vertices(-2.48, 0.78,  0.55, 0.06, 0.14, 0.40)},
            {"name": "brakelight_R", "mat": 4, "build_fn": lambda: box_vertices(-2.48, 0.78, -0.55, 0.06, 0.14, 0.40)},
            # SU7 贯穿式尾灯条
            {"name": "brakelight_bar", "mat": 7, "build_fn": lambda: box_vertices(-2.50, 0.78, 0.00, 0.06, 0.10, 1.65)},
            # 转向灯 (turnsignal, mat 5)
            {"name": "turnsignal_FL", "mat": 5, "build_fn": lambda: box_vertices( 2.48, 0.66,  0.85, 0.06, 0.10, 0.08)},
            {"name": "turnsignal_FR", "mat": 5, "build_fn": lambda: box_vertices( 2.48, 0.66, -0.85, 0.06, 0.10, 0.08)},
            {"name": "turnsignal_RL", "mat": 5, "build_fn": lambda: box_vertices(-2.48, 0.74,  0.85, 0.06, 0.10, 0.08)},
            {"name": "turnsignal_RR", "mat": 5, "build_fn": lambda: box_vertices(-2.48, 0.74, -0.85, 0.06, 0.10, 0.08)},
            # 自动驾驶小蓝灯 ×2
            {"name": "ads_indicator_L", "mat": 6, "build_fn": lambda: cylinder_vertices(-1.95, 0.92,  0.50, 0.07, 0.08, "y", 12)},
            {"name": "ads_indicator_R", "mat": 6, "build_fn": lambda: cylinder_vertices(-1.95, 0.92, -0.50, 0.07, 0.08, "y", 12)},
            # ── 车辆转向系统 ──
            {"name": "axle_front", "translation": [ 1.50, 0.34, 0.0]},
            {"name": "axle_rear",  "translation": [-1.50, 0.34, 0.0]},
            # 车轮（24 段圆柱）
            {"name": "wheel_FL", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0,  1.00],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.34, 0.26, "z", 24)},
            {"name": "wheel_FR", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0, -1.00],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.34, 0.26, "z", 24)},
            {"name": "wheel_RL", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0,  1.00],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.34, 0.26, "z", 24)},
            {"name": "wheel_RR", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0, -1.00],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.34, 0.26, "z", 24)},
        ],
    },
    # ── truck：车头朝 +x，货箱在 -x；圆柱车轮 + 灯节点。目标 L=7.5 H=2.60 W=2.5
    {
        "name": "truck",
        "materials": ["truck_paint", "glass", "tire", "headlight", "brakelight", "turnsignal"],
        "parts": [
            {"name": "cab",        "mat": 0, "build_fn": lambda: box_vertices( 1.0,  0.55, 0.0,  2.0, 0.9,  2.4)},
            # cargo: cy=1.60 sy=2.00 → top=2.60 = 目标 H
            {"name": "cargo",      "mat": 0, "build_fn": lambda: box_vertices(-2.5,  1.60, 0.0,  5.5, 2.0,  2.5)},
            {"name": "windshield", "mat": 1, "build_fn": lambda: box_vertices( 0.5,  0.95, 0.0,  0.08, 0.6, 1.6)},
            {"name": "headlight_L",  "mat": 3, "build_fn": lambda: box_vertices( 1.95, 0.55,  0.70, 0.10, 0.18, 0.40)},
            {"name": "headlight_R",  "mat": 3, "build_fn": lambda: box_vertices( 1.95, 0.55, -0.70, 0.10, 0.18, 0.40)},
            {"name": "brakelight_L", "mat": 4, "build_fn": lambda: box_vertices(-5.20, 0.90,  0.70, 0.10, 0.18, 0.40)},
            {"name": "brakelight_R", "mat": 4, "build_fn": lambda: box_vertices(-5.20, 0.90, -0.70, 0.10, 0.18, 0.40)},
            {"name": "turnsignal_FL", "mat": 5, "build_fn": lambda: box_vertices( 1.95, 0.55,  0.95, 0.08, 0.14, 0.10)},
            {"name": "turnsignal_FR", "mat": 5, "build_fn": lambda: box_vertices( 1.95, 0.55, -0.95, 0.08, 0.14, 0.10)},
            {"name": "turnsignal_RL", "mat": 5, "build_fn": lambda: box_vertices(-5.20, 0.90,  0.95, 0.08, 0.14, 0.10)},
            {"name": "turnsignal_RR", "mat": 5, "build_fn": lambda: box_vertices(-5.20, 0.90, -0.95, 0.08, 0.14, 0.10)},
            # ── 车辆转向系统 ──
            {"name": "axle_front", "translation": [ 1.50, 0.50, 0.0]},
            {"name": "axle_rear",  "translation": [-1.50, 0.50, 0.0]},
            {"name": "wheel_FL", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0,  1.10],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.50, 0.40, "z", 14)},
            {"name": "wheel_FR", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0, -1.10],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.50, 0.40, "z", 14)},
            {"name": "wheel_RL", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0,  1.10],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.50, 0.40, "z", 14)},
            {"name": "wheel_RR", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0, -1.10],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.50, 0.40, "z", 14)},
        ],
    },
    # ── suv：高车身 + 圆柱车轮 + 灯节点。目标 L=4.6 H=1.70 W=1.9
    {
        "name": "suv",
        "materials": ["suv_paint", "glass", "tire", "headlight", "brakelight", "turnsignal"],
        "parts": [
            {"name": "body",       "mat": 0, "build_fn": lambda: box_vertices( 0.0,  0.58,  0.0,  4.6, 0.95, 1.90)},
            # cabin: cy=1.40 sy=0.60 → top=1.70 = 目标 H
            {"name": "cabin",      "mat": 0, "build_fn": lambda: box_vertices(-0.3,  1.40,  0.0,  1.8, 0.60, 1.50)},
            {"name": "rear",       "mat": 0, "build_fn": lambda: box_vertices( 1.0,  1.35,  0.0,  1.2, 0.55, 1.50)},
            {"name": "windshield", "mat": 1, "build_fn": lambda: box_vertices( 1.10, 1.12,  0.0,  0.06, 0.48, 1.45)},
            {"name": "headlight_L",  "mat": 3, "build_fn": lambda: box_vertices( 2.28, 0.62,  0.60, 0.10, 0.18, 0.42)},
            {"name": "headlight_R",  "mat": 3, "build_fn": lambda: box_vertices( 2.28, 0.62, -0.60, 0.10, 0.18, 0.42)},
            {"name": "brakelight_L", "mat": 4, "build_fn": lambda: box_vertices(-2.28, 0.72,  0.60, 0.10, 0.16, 0.42)},
            {"name": "brakelight_R", "mat": 4, "build_fn": lambda: box_vertices(-2.28, 0.72, -0.60, 0.10, 0.16, 0.42)},
            {"name": "turnsignal_FL", "mat": 5, "build_fn": lambda: box_vertices( 2.25, 0.66,  0.80, 0.08, 0.14, 0.10)},
            {"name": "turnsignal_FR", "mat": 5, "build_fn": lambda: box_vertices( 2.25, 0.66, -0.80, 0.08, 0.14, 0.10)},
            {"name": "turnsignal_RL", "mat": 5, "build_fn": lambda: box_vertices(-2.25, 0.72,  0.80, 0.08, 0.14, 0.10)},
            {"name": "turnsignal_RR", "mat": 5, "build_fn": lambda: box_vertices(-2.25, 0.72, -0.80, 0.08, 0.14, 0.10)},
            # ── 车辆转向系统 ──
            {"name": "axle_front", "translation": [ 1.40, 0.38, 0.0]},
            {"name": "axle_rear",  "translation": [-1.40, 0.38, 0.0]},
            {"name": "wheel_FL", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0,  0.93],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.38, 0.28, "z", 14)},
            {"name": "wheel_FR", "mat": 2, "parent": "axle_front", "translation": [0.0, 0.0, -0.93],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.38, 0.28, "z", 14)},
            {"name": "wheel_RL", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0,  0.93],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.38, 0.28, "z", 14)},
            {"name": "wheel_RR", "mat": 2, "parent": "axle_rear",  "translation": [0.0, 0.0, -0.93],
             "build_fn": lambda: cylinder_vertices(0.0, 0.0, 0.0, 0.38, 0.28, "z", 14)},
        ],
    },
    {
        "name": "pedestrian",
        "materials": ["pedestrian_body", "pedestrian_head"],
        "parts": [
            {"name": "torso", "mat": 0,
             "build_fn": lambda: box_vertices(0.0, 0.6, 0.0, 0.3, 0.6, 0.5)},
            {"name": "head", "mat": 1,
             "build_fn": lambda: box_vertices(0.0, 1.1, 0.0, 0.28, 0.28, 0.28)},
            {"name": "leg_L", "mat": 0,
             "build_fn": lambda: box_vertices(-0.12, 0.2, 0.0, 0.12, 0.4, 0.14)},
            {"name": "leg_R", "mat": 0,
             "build_fn": lambda: box_vertices(0.12, 0.2, 0.0, 0.12, 0.4, 0.14)},
        ],
    },
]


# ── glTF 构建器 ──────────────────────────────────────────────

def build_gltf(spec: dict) -> dict:
    """从规格构建完整 glTF JSON（嵌入 base64 buffers）。

    支持节点层级（车辆转向系统）：
      - part 可带 "translation"（节点平移）和 "parent"（父节点名）。
      - 无 "build_fn" 的 part 视为 Group 节点（仅 transform，无 mesh），
        例如 axle_front / axle_rear —— 车轮挂在轴下，translation 为相对轴心的偏移。
      - 有 "build_fn" 的 part 仍是 mesh 节点（保留原行为）。
    scene.nodes 只列顶层节点（无 parent），子节点通过父节点 "children" 引用。
    """
    buf = BufferBuilder()
    meshes = []
    materials_list = []

    # 引用全局材质
    for mat_name in spec["materials"]:
        mat = MATERIALS.get(mat_name)
        if mat:
            materials_list.append(mat)

    # ── 第一阶段：为带 build_fn 的 part 构建 mesh primitive（写入 buffer）──
    # 没有 build_fn 的 part（如 axle_front/axle_rear）只贡献 transform 节点，不建 mesh。
    mesh_primitives = []
    mesh_idx_by_name: dict[str, int] = {}
    for part in spec["parts"]:
        build_fn = part.get("build_fn")
        if not build_fn:
            continue  # group-only node（axle_front / axle_rear 等）
        result = build_fn()
        # build_fn 可返回两种格式：
        #   - list[float]  : 仅顶点（box 风格，每 4 顶点 = 2 三角形，自动生成索引）
        #   - (verts, idx) : 顶点 + 自定义索引（cylinder 等非 box 几何）
        if isinstance(result, tuple):
            verts, tris = result
        else:
            verts = result
            nv = len(verts) // 6
            # 三角形索引：每 4 个顶点 = 2 个三角形（box 风格）
            tris = []
            for f in range(nv // 4):
                b = f * 4
                tris.extend([b, b + 1, b + 2, b, b + 2, b + 3])

        v_offset = buf.add_floats(verts)
        # 每个 part 有独立 bufferView（pos/normal 共享一个，indices 独立一个），
        # bufferView.byteOffset 已指向该 part 在大 buffer 中的起始位置，
        # 所以索引保持相对值（从 0 开始）即可。
        idx_offset = buf.add_u16s(tris)

        mat_idx = part["mat"]
        mesh_idx = len(meshes)
        # pos_accessor_idx / idx_accessor_idx 占位，第二阶段填入真实 accessor 索引
        pos_accessor_idx = mesh_idx * 2
        idx_accessor_idx = pos_accessor_idx + 1
        prim = {
            "attributes": {"POSITION": pos_accessor_idx, "NORMAL": pos_accessor_idx + 1},
            "indices": idx_accessor_idx,
            "material": mat_idx,
        }
        meshes.append({
            "primitives": [prim],
            "name": part["name"],
        })
        mesh_primitives.append({
            "pos_accessor": {"count": len(verts) // 6, "min": [], "max": [], "offset": v_offset},
            "idx_accessor": {"count": len(tris), "offset": idx_offset},
        })
        mesh_idx_by_name[part["name"]] = mesh_idx

    # 构建 buffer view 和 accessor
    b64 = buf.to_base64()
    total_len = buf.byte_length()

    # 每个 part 独立的一组 vertices + indices
    buffer_views = []
    accessors = []
    _bo = 0

    for mesh_idx, mp in enumerate(mesh_primitives):
        # Position & Normal: 每个顶点 6 floats (x3 + nx3)
        n_verts = mp["pos_accessor"]["count"]
        pos_bytes = n_verts * 6 * 4  # 6 floats × 4 bytes
        bv = {
            "buffer": 0,
            "byteOffset": mp["pos_accessor"]["offset"],
            "byteLength": pos_bytes,
            "byteStride": 24,  # 6 floats × 4 bytes
        }
        buffer_views.append(bv)

        # Position accessor (float, 3 components)
        xs = []; ys = []; zs = []
        for i in range(n_verts):
            off = mp["pos_accessor"]["offset"] + i * 24
            x = struct.unpack("<f", bytes(buf.data[off:off+4]))[0]
            y = struct.unpack("<f", bytes(buf.data[off+4:off+8]))[0]
            z = struct.unpack("<f", bytes(buf.data[off+8:off+12]))[0]
            xs.append(x); ys.append(y); zs.append(z)
        accessors.append({
            "bufferView": len(buffer_views) - 1,
            "componentType": 5126,  # FLOAT
            "count": n_verts,
            "type": "VEC3",
            "min": [min(xs), min(ys), min(zs)],
            "max": [max(xs), max(ys), max(zs)],
        })

        # Normal accessor (float, 3 components) — same buffer, offset by 12 bytes
        nxs = []; nys = []; nzs = []
        for i in range(n_verts):
            off = mp["pos_accessor"]["offset"] + i * 24 + 12
            x = struct.unpack("<f", bytes(buf.data[off:off+4]))[0]
            y = struct.unpack("<f", bytes(buf.data[off+4:off+8]))[0]
            z = struct.unpack("<f", bytes(buf.data[off+8:off+12]))[0]
            nxs.append(x); nys.append(y); nzs.append(z)
        accessors.append({
            "bufferView": len(buffer_views) - 1,
            "componentType": 5126,  # FLOAT
            "count": n_verts,
            "type": "VEC3",
            "byteOffset": 12,
            "min": [min(nxs), min(nys), min(nzs)],
            "max": [max(nxs), max(nys), max(nzs)],
        })

        # Index accessor (u16)
        idx_count = mp["idx_accessor"]["count"]
        idx_bytes = idx_count * 2
        bv_idx = {
            "buffer": 0,
            "byteOffset": mp["idx_accessor"]["offset"],
            "byteLength": idx_bytes,
        }
        buffer_views.append(bv_idx)
        accessors.append({
            "bufferView": len(buffer_views) - 1,
            "componentType": 5123,  # UNSIGNED_SHORT
            "count": idx_count,
            "type": "SCALAR",
        })

        # 更新 mesh 中的 accessor 索引
        # 每个 mesh 有 1 个 primitive，引用 3 个 accessors
        prim = meshes[mesh_idx]["primitives"][0]
        prim["attributes"]["POSITION"] = len(accessors) - 3
        prim["attributes"]["NORMAL"] = len(accessors) - 2
        prim["indices"] = len(accessors) - 1

    # ── 第二阶段：组装节点树（支持 parent/translation 层级）──
    # 每个 part 对应一个 node：有 build_fn 的带 mesh 引用，否则只是 transform Group。
    # 子节点（part 带 "parent"）通过父节点 "children" 数组引用。
    nodes: list[dict] = []
    node_idx_by_name: dict[str, int] = {}
    for part in spec["parts"]:
        node: dict = {"name": part["name"]}
        if "translation" in part:
            node["translation"] = list(part["translation"])
        if part.get("build_fn"):
            node["mesh"] = mesh_idx_by_name[part["name"]]
        nodes.append(node)
        node_idx_by_name[part["name"]] = len(nodes) - 1

    top_level: list[int] = []
    for part in spec["parts"]:
        parent_name = part.get("parent")
        if parent_name:
            parent_idx = node_idx_by_name[parent_name]
            child_idx = node_idx_by_name[part["name"]]
            nodes[parent_idx].setdefault("children", []).append(child_idx)
        else:
            top_level.append(node_idx_by_name[part["name"]])

    gltf = {
        "asset": {"version": "2.0", "generator": "FlowEngine gen_models.py"},
        "scene": 0,
        "scenes": [{"nodes": top_level}],
        "nodes": nodes,
        "meshes": meshes,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": total_len, "uri": f"data:application/octet-stream;base64,{b64}"}],
        "materials": materials_list,
    }

    return gltf


# ── 主入口 ──────────────────────────────────────────────────

def measure_gltf_bbox(gltf: dict) -> dict:
    """从 glTF JSON 计算世界空间包围盒 (L×H×W, 米)。
    
    遍历节点层级，对每个带 mesh 的节点：
    1. 沿父链累加 translation 得到世界位移
    2. 取 mesh 的 POSITION accessor 的 min/max（局部空间）
    3. 加世界位移得到世界空间 bbox
    4. 合并所有 mesh 的 bbox
    
    返回 {"L": x_range, "H": y_range, "W": z_range, "min_x", "max_x", ...}
    """
    nodes = gltf.get("nodes", [])
    meshes = gltf.get("meshes", [])
    accessors = gltf.get("accessors", [])
    
    # 构建 parent 映射
    parent_of = {}
    for ni, node in enumerate(nodes):
        for ci in node.get("children", []):
            parent_of[ci] = ni
    
    # 计算每个节点的世界位移（只累加 translation）
    def world_translation(node_idx):
        tx, ty, tz = 0.0, 0.0, 0.0
        while node_idx is not None:
            t = nodes[node_idx].get("translation", [0, 0, 0])
            tx += t[0]; ty += t[1]; tz += t[2]
            node_idx = parent_of.get(node_idx)
        return tx, ty, tz
    
    all_min = [float("inf")] * 3
    all_max = [float("-inf")] * 3
    
    for ni, node in enumerate(nodes):
        mesh_idx = node.get("mesh")
        if mesh_idx is None:
            continue
        mesh = meshes[mesh_idx]
        wt_x, wt_y, wt_z = world_translation(ni)
        
        for prim in mesh.get("primitives", []):
            pos_acc_idx = prim.get("attributes", {}).get("POSITION")
            if pos_acc_idx is None:
                continue
            acc = accessors[pos_acc_idx]
            local_min = acc.get("min", [0, 0, 0])
            local_max = acc.get("max", [0, 0, 0])
            
            world_min = [local_min[0] + wt_x, local_min[1] + wt_y, local_min[2] + wt_z]
            world_max = [local_max[0] + wt_x, local_max[1] + wt_y, local_max[2] + wt_z]
            
            for d in range(3):
                if world_min[d] < all_min[d]: all_min[d] = world_min[d]
                if world_max[d] > all_max[d]: all_max[d] = world_max[d]
    
    if all_min[0] == float("inf"):
        return {"L": 0, "H": 0, "W": 0}
    
    L = all_max[0] - all_min[0]
    H = all_max[1] - all_min[1]
    W = all_max[2] - all_min[2]
    return {
        "L": L, "H": H, "W": W,
        "min_x": all_min[0], "max_x": all_max[0],
        "min_y": all_min[1], "max_y": all_max[1],
        "min_z": all_min[2], "max_z": all_max[2],
    }


# 目标规格：(L, H, W, min_LH_ratio)
# sedan/su7 要求 L:H ≥ 3.0；suv/truck 放宽 ≥ 2.6
MODEL_TARGETS = {
    "sedan":      (4.6, 1.45, 1.80, 3.0),
    "su7":        (5.0, 1.45, 1.90, 3.0),
    "suv":        (4.6, 1.70, 1.90, 2.6),
    "truck":      (7.5, 2.60, 2.50, 2.6),
}


def check_model_proportions(output_dir: Path) -> int:
    """自检：读回生成的 glTF 文件，验证 bbox 尺寸和 L:H 比例。
    返回 0 = 全部通过，1 = 有失败。"""
    errors = 0
    for name, (target_L, target_H, target_W, min_lh) in MODEL_TARGETS.items():
        path = output_dir / f"{name}.gltf"
        if not path.exists():
            print(f"  ✗ {name}.gltf 缺失，跳过自检", file=sys.stderr)
            errors += 1
            continue
        
        with open(path) as f:
            gltf = json.load(f)
        bbox = measure_gltf_bbox(gltf)
        L, H, W = bbox["L"], bbox["H"], bbox["W"]
        lh_ratio = L / H if H > 0 else 0
        
        h_ok = abs(H - target_H) <= 0.1
        lh_ok = lh_ratio >= min_lh
        
        status = "✓" if (h_ok and lh_ok) else "✗"
        print(f"  {status} {name}: L={L:.2f} H={H:.2f} W={W:.2f}  L:H={lh_ratio:.2f}  "
              f"(目标 H={target_H}±0.1 L:H≥{min_lh})")
        
        if not h_ok:
            print(f"    ✗ H={H:.2f} 超出 [{target_H-0.1:.2f}, {target_H+0.1:.2f}]", file=sys.stderr)
            errors += 1
        if not lh_ok:
            print(f"    ✗ L:H={lh_ratio:.2f} < {min_lh}", file=sys.stderr)
            errors += 1
    
    return errors


def generate_all(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for spec in MODEL_SPECS:
        path = output_dir / f"{spec['name']}.gltf"
        gltf = build_gltf(spec)
        with open(path, "w") as f:
            json.dump(gltf, f, indent=2)
        size_kb = path.stat().st_size / 1024
        def _face_count(part):
            if not part.get("build_fn"):
                return 0
            r = part["build_fn"]()
            if isinstance(r, tuple):
                return len(r[1]) // 3
            return (len(r) // 6 // 4) * 2
        faces = sum(_face_count(p) for p in spec["parts"])
        print(f"  ✓ {spec['name']}.gltf  ({size_kb:.1f} KB, {len(spec['parts'])} 部件, ~{faces} 三角面)")
    print(f"\n  总计 {len(MODEL_SPECS)} 个模型 → {output_dir}")
    
    # ── 自检：验证车辆模型比例 ──
    print(f"\n自检模型比例:")
    errs = check_model_proportions(output_dir)
    if errs > 0:
        print(f"\n  {errs} 项自检失败，退出", file=sys.stderr)
        sys.exit(1)
    print(f"  全部通过 ✓")


def validate_all(output_dir: Path) -> int:
    """检查所有模型文件是否完整。返回 0=成功。"""
    errors = 0
    names = [spec["name"] for spec in MODEL_SPECS]
    for name in names:
        path = output_dir / f"{name}.gltf"
        if not path.exists():
            print(f"  ✗ {name}.gltf 缺失", file=sys.stderr)
            errors += 1
            continue
        try:
            with open(path) as f:
                data = json.load(f)
            assert data["asset"]["version"] == "2.0"
            assert len(data["meshes"]) > 0
            print(f"  ✓ {name}.gltf  ({path.stat().st_size // 1024} KB, {len(data['meshes'])} meshes)")
        except Exception as e:
            print(f"  ✗ {name}.gltf 读取失败: {e}", file=sys.stderr)
            errors += 1
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description="FlowEngine 3D 车辆模型生成器")
    ap.add_argument("--validate", action="store_true", help="仅验证已有模型")
    args = ap.parse_args()

    if args.validate:
        print(f"验证模型: {MODELS_DIR}")
        errs = validate_all(MODELS_DIR)
        if errs:
            print(f"  {errs} 个错误", file=sys.stderr)
        return 0 if errs == 0 else 1

    print(f"生成 glTF 模型到 {MODELS_DIR} ...")
    generate_all(MODELS_DIR)
    return 0


if __name__ == "__main__":
    sys.exit(main())
