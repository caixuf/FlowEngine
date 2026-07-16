#!/usr/bin/env python3
"""
waypoint_player.py — 航点文件查看/统计/编辑工具 (L2 Pure Pursuit 配套)

查看 waypoint_record.py 录制的航点文件，统计路径长度、相邻间距，提供简单的 ASCII 可视化。
也能从已有航点 JSON 转换/合并/重采样。

用法:
    # 查看航点文件
    python3 tools/waypoint_player.py /tmp/waypoints.json

    # 显示 ASCII 路径图
    python3 tools/waypoint_player.py /tmp/waypoints.json --plot

    # 重采样（每 2 米一个航点）
    python3 tools/waypoint_player.py /tmp/waypoints.json --resample 2.0 -o /tmp/waypoints_resampled.json
"""

import argparse
import json
import math
import sys


def load_waypoints(path):
    with open(path, "r") as f:
        return json.load(f)


def path_length(waypoints):
    total = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i]["x"] - waypoints[i - 1]["x"]
        dy = waypoints[i]["y"] - waypoints[i - 1]["y"]
        total += math.sqrt(dx * dx + dy * dy)
    return total


def nearest_spacing(waypoints):
    if len(waypoints) < 2:
        return 0, 0
    distances = []
    for i in range(1, len(waypoints)):
        dx = waypoints[i]["x"] - waypoints[i - 1]["x"]
        dy = waypoints[i]["y"] - waypoints[i - 1]["y"]
        distances.append(math.sqrt(dx * dx + dy * dy))
    return min(distances), max(distances)


def resample(waypoints, target_spacing):
    """按目标间距重采样航点"""
    if len(waypoints) < 2:
        return waypoints
    out = [waypoints[0]]
    accum = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i]["x"] - waypoints[i - 1]["x"]
        dy = waypoints[i]["y"] - waypoints[i - 1]["y"]
        seg_len = math.sqrt(dx * dx + dy * dy)
        if seg_len < 1e-6:
            continue
        accum += seg_len
        if accum >= target_spacing:
            out.append(waypoints[i])
            accum = 0.0
    # 确保最后一个航点被包含
    if out[-1] != waypoints[-1]:
        out.append(waypoints[-1])
    return out


def ascii_plot(waypoints, width=70, height=20):
    """简易 ASCII 路径图"""
    if not waypoints:
        return "(空)"
    xs = [w["x"] for w in waypoints]
    ys = [w["y"] for w in waypoints]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    if maxx - minx < 1e-6:
        maxx = minx + 1
    if maxy - miny < 1e-6:
        maxy = miny + 1

    grid = [[" "] * width for _ in range(height)]
    for i, w in enumerate(waypoints):
        col = int((w["x"] - minx) / (maxx - minx) * (width - 1))
        row = int((maxy - w["y"]) / (maxy - miny) * (height - 1))
        col = max(0, min(width - 1, col))
        row = max(0, min(height - 1, row))
        ch = "S" if i == 0 else ("E" if i == len(waypoints) - 1 else "*")
        grid[row][col] = ch

    lines = []
    lines.append(f"y:+{maxy:+.1f}m " + "+" + "-" * (width - 2) + "+")
    for row in range(height):
        lines.append(" " * 10 + "|" + "".join(grid[row]) + "|")
    lines.append(f"y:-{miny:+.1f}m " + "+" + "-" * (width - 2) + "+")
    lines.append(" " * 10 + f"x: {minx:+.1f}m" + " " * (width - 20) + f"x: {maxx:+.1f}m")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="航点文件查看/统计/编辑工具")
    ap.add_argument("file", help="航点 JSON 文件路径")
    ap.add_argument("--plot", action="store_true", help="显示 ASCII 路径图")
    ap.add_argument("--resample", type=float, help="按目标间距重采样（米），写回 --out")
    ap.add_argument("--out", "-o", help="重采样后的输出路径")
    args = ap.parse_args()

    data = load_waypoints(args.file)
    wps = data.get("waypoints", [])

    print(f"=== {args.file} ===")
    print(f"航点数: {len(wps)}")
    if "lookahead_m" in data:
        print(f"前瞻距离: {data['lookahead_m']}m")
    if "cruise_speed" in data:
        print(f"巡航速度: {data['cruise_speed']}m/s")
    if "origin_lat" in data:
        print(f"原点 GPS: ({data['origin_lat']}, {data['origin_lon']})")

    if wps:
        total = path_length(wps)
        min_d, max_d = nearest_spacing(wps)
        print(f"路径总长: {total:.2f}m")
        print(f"相邻间距: min={min_d:.2f}m max={max_d:.2f}m")
        print(f"起点: x={wps[0]['x']:+.2f} y={wps[0]['y']:+.2f}")
        print(f"终点: x={wps[-1]['x']:+.2f} y={wps[-1]['y']:+.2f}")

    if args.plot:
        print("\n路径图:")
        print(ascii_plot(wps))

    if args.resample:
        new_wps = resample(wps, args.resample)
        print(f"\n重采样: {len(wps)} → {len(new_wps)} 航点 (间距 {args.resample}m)")
        if args.out:
            data["waypoints"] = new_wps
            with open(args.out, "w") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            print(f"[OK] 写入 {args.out}")
        else:
            print("(未指定 --out, 仅显示不写回)")


if __name__ == "__main__":
    main()
