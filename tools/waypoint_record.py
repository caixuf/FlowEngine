#!/usr/bin/env python3
"""
waypoint_record.py — GPS 航点录制工具 (L2 Pure Pursuit 配套)

读串口 GPS NMEA → 转局部笛卡尔坐标 → 写航点 JSON 文件，供 waypoint_follower_node 使用。

用法:
    # 录制：跟着小车走一圈，每 1 秒采一个航点
    python3 tools/waypoint_record.py /dev/ttyUSB0 --out /tmp/waypoints.json --interval 1.0

    # 也可以从已有 GPS NMEA 日志回放录制
    python3 tools/waypoint_record.py --replay gps_log.txt --out /tmp/waypoints.json

输出格式 (waypoint_follower_node 期望的 JSON):
    {
      "lookahead_m": 1.5,
      "cruise_speed": 2.0,
      "origin_lat": 31.2304,
      "origin_lon": 121.4737,
      "waypoints": [
        {"x": 0.0, "y": 0.0, "lat": 31.2304, "lon": 121.4737},
        {"x": 5.2, "y": 0.1, "lat": ..., "lon": ...}
      ]
    }

坐标转换: 以第一个 GPS 点为原点，用平面近似（小范围 <1km 内误差 <1%）
    x = (lon - origin_lon) * cos(origin_lat) * 111320  (米，东为 +)
    y = (lat - origin_lat) * 110540                     (米，北为 +)
"""

import argparse
import json
import math
import sys
import time

# ── NMEA 最小解析器 ──────────────────────────────────────────

def parse_nmea_gga(line):
    """解析 $GPGGA / $GNGGA，返回 (lat, lon) 或 None"""
    if not (line.startswith("$GPGGA") or line.startswith("$GNGGA")):
        return None
    parts = line.strip().split(",")
    if len(parts) < 6:
        return None
    try:
        lat_raw = parts[2]
        lat_dir = parts[3]
        lon_raw = parts[4]
        lon_dir = parts[5]
        if not lat_raw or not lon_raw:
            return None
        # ddmm.mmmm → dd.dddddd
        lat_deg = int(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        lat = lat_deg + lat_min / 60.0
        if lat_dir == "S":
            lat = -lat
        lon_deg = int(lon_raw[:3])
        lon_min = float(lon_raw[3:])
        lon = lon_deg + lon_min / 60.0
        if lon_dir == "W":
            lon = -lon
        return (lat, lon)
    except (ValueError, IndexError):
        return None


def parse_nmea_rmc(line):
    """解析 $GPRMC / $GNRMC，返回 (lat, lon, speed_knots, heading_deg) 或 None"""
    if not (line.startswith("$GPRMC") or line.startswith("$GNRMC")):
        return None
    parts = line.strip().split(",")
    if len(parts) < 9:
        return None
    try:
        if parts[2] != "A":  # V = 无效定位
            return None
        lat_raw = parts[3]
        lat_dir = parts[4]
        lon_raw = parts[5]
        lon_dir = parts[6]
        speed_knots = float(parts[7]) if parts[7] else 0.0
        heading = float(parts[8]) if parts[8] else 0.0
        lat_deg = int(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        lat = lat_deg + lat_min / 60.0
        if lat_dir == "S":
            lat = -lat
        lon_deg = int(lon_raw[:3])
        lon_min = float(lon_raw[3:])
        lon = lon_deg + lon_min / 60.0
        if lon_dir == "W":
            lon = -lon
        return (lat, lon, speed_knots, heading)
    except (ValueError, IndexError):
        return None


# ── GPS 经纬度 → 局部笛卡尔坐标 ─────────────────────────────

EARTH_M_PER_DEG_LAT = 110540.0  # 纬度 1° ≈ 110.54km

def gps_to_local(lat, lon, origin_lat, origin_lon):
    """以 (origin_lat, origin_lon) 为原点的平面近似投影（米）"""
    x = (lon - origin_lon) * math.cos(math.radians(origin_lat)) * 111320.0
    y = (lat - origin_lat) * EARTH_M_PER_DEG_LAT
    return (x, y)


# ── 串口 GPS 读取 ────────────────────────────────────────────

def read_gps_from_serial(port, baud, duration_s=None, interval_s=1.0, on_point=None):
    """从串口读 NMEA，每 interval_s 秒采一个航点"""
    try:
        import serial
    except ImportError:
        print("[ERROR] 需要 pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    ser = serial.Serial(port, baud, timeout=1.0)
    print(f"[INFO] 打开 {port} @ {baud}bps, 按 Ctrl-C 停止录制", file=sys.stderr)

    origin = None
    last_sample_time = 0
    waypoints = []

    try:
        while True:
            line = ser.readline().decode("ascii", errors="ignore")
            if not line:
                continue
            res = parse_nmea_rmc(line) or parse_nmea_gga(line)
            if res is None:
                continue
            lat, lon = res[0], res[1]

            now = time.time()
            if now - last_sample_time < interval_s:
                continue

            if origin is None:
                origin = (lat, lon)
                print(f"[INFO] 原点: lat={lat:.6f} lon={lon:.6f}", file=sys.stderr)

            x, y = gps_to_local(lat, lon, origin[0], origin[1])
            waypoints.append({"x": round(x, 3), "y": round(y, 3),
                              "lat": round(lat, 6), "lon": round(lon, 6)})
            last_sample_time = now
            print(f"[WP {len(waypoints):3d}] x={x:+.2f}m y={y:+.2f}m "
                  f"(lat={lat:.6f} lon={lon:.6f})", file=sys.stderr)

            if on_point:
                on_point(waypoints[-1])

            if duration_s and (now - (origin[0] and last_sample_time or now)) > duration_s:
                break

    except KeyboardInterrupt:
        print(f"\n[INFO] 停止录制，共 {len(waypoints)} 个航点", file=sys.stderr)
    finally:
        ser.close()

    return waypoints, origin


def read_gps_from_log(logfile, interval_s=1.0):
    """从 NMEA 日志文件回放录制"""
    origin = None
    waypoints = []
    sim_time = 0
    last_sample_time = 0

    with open(logfile, "r") as f:
        for line in f:
            sim_time += 0.1  # 假设日志 10Hz
            res = parse_nmea_rmc(line) or parse_nmea_gga(line)
            if res is None:
                continue
            lat, lon = res[0], res[1]
            if sim_time - last_sample_time < interval_s:
                continue
            if origin is None:
                origin = (lat, lon)
            x, y = gps_to_local(lat, lon, origin[0], origin[1])
            waypoints.append({"x": round(x, 3), "y": round(y, 3),
                              "lat": round(lat, 6), "lon": round(lon, 6)})
            last_sample_time = sim_time

    return waypoints, origin


# ── 主入口 ───────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="GPS 航点录制工具 (L2 Pure Pursuit 配套)")
    ap.add_argument("port", nargs="?", help="串口设备路径 (如 /dev/ttyUSB0)")
    ap.add_argument("--replay", help="从 NMEA 日志文件回放录制")
    ap.add_argument("--out", "-o", default="/tmp/waypoints.json", help="输出 JSON 路径")
    ap.add_argument("--baud", type=int, default=9600, help="串口波特率")
    ap.add_argument("--interval", type=float, default=1.0, help="采样间隔 (秒)")
    ap.add_argument("--lookahead", type=float, default=1.5, help="Pure Pursuit 前瞻距离 (米)")
    ap.add_argument("--cruise", type=float, default=2.0, help="巡航速度 (m/s)")
    args = ap.parse_args()

    if args.replay:
        print(f"[INFO] 从日志回放: {args.replay}", file=sys.stderr)
        waypoints, origin = read_gps_from_log(args.replay, args.interval)
    elif args.port:
        waypoints, origin = read_gps_from_serial(args.port, args.baud,
                                                  interval_s=args.interval)
    else:
        ap.error("需要提供串口路径或 --replay 日志文件")

    if len(waypoints) < 2:
        print(f"[ERROR] 航点不足 2 个 (实际 {len(waypoints)})，无法录制", file=sys.stderr)
        sys.exit(1)

    # 写 JSON
    out = {
        "lookahead_m": args.lookahead,
        "cruise_speed": args.cruise,
        "origin_lat": round(origin[0], 6) if origin else 0.0,
        "origin_lon": round(origin[1], 6) if origin else 0.0,
        "waypoints": waypoints,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"[OK] 写入 {len(waypoints)} 个航点到 {args.out}", file=sys.stderr)
    print(f"     原点: ({out['origin_lat']}, {out['origin_lon']})", file=sys.stderr)
    print(f"     lookahead={args.lookahead}m cruise={args.cruise}m/s", file=sys.stderr)


if __name__ == "__main__":
    main()
