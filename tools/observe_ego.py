#!/usr/bin/env python3
"""持续采样 ego 轨迹 + 斜走角，用于掉头/返程行为观测。

用法：启动 demo 后运行本脚本，每 2s 采一个 (x,y,heading,v)。
输出：每行一个采样，含位置增量方向 vs 车头（斜走角）、车道估计。
  位置斜走角 = |heading − 位置增量方向|（>5° 即明显斜走）
  车头斜走角 = |heading − atan2(vy,vx)|（vx/vy 由 heading 派生时为 0，仅参考）
"""

import json
import math
import time

TOP = '/tmp/flow_topology.json'


def snap():
    d = json.load(open(TOP))
    e = d['metrics']['scene']['ego']
    return (e.get('x', 0), e.get('y', 0), e.get('heading', 0), e.get('speed', 0),
            e.get('vx', 0), e.get('vy', 0))


def main(duration=220):
    last = snap()
    t0 = time.time()
    while time.time() - t0 < duration:
        time.sleep(2.0)
        try:
            cur = snap()
        except Exception:
            continue
        dx, dy = cur[0] - last[0], cur[1] - last[1]
        d = math.hypot(dx, dy)
        if d > 0.5:
            posdir = math.atan2(dy, dx)
        else:
            posdir = float('nan')
        crab = abs((cur[2] - posdir + math.pi) % (2 * math.pi) - math.pi) if not math.isnan(posdir) else float('nan')
        # 车头 vs vx/vy 方向（vx/vy 由 heading 派生，恒 0，仅对照）
        vdir = math.atan2(cur[5], cur[4]) if math.hypot(cur[4], cur[5]) > 0.01 else float('nan')
        vcrab = abs((cur[2] - vdir + math.pi) % (2 * math.pi) - math.pi) if not math.isnan(vdir) else float('nan')
        lane = '?'  # 按 y 粗估：lane center ±1.75/±5.25
        for lc, cy in [(-2, -5.25), (-1, -1.75), (1, 1.75), (2, 5.25)]:
            if abs(cur[1] - cy) < 1.0:
                lane = lc
                break
        t = time.time() - t0
        print(f"t={t:5.0f} x={cur[0]:7.0f} y={cur[1]:6.2f} v={cur[3]:4.1f} "
              f"h={math.degrees(cur[2]):7.1f} posdir={math.degrees(posdir):7.1f} "
              f"crab={math.degrees(crab) if not math.isnan(crab) else float('nan'):6.1f} "
              f"lane={lane}", flush=True)
        last = cur


if __name__ == '__main__':
    import sys
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 220)
