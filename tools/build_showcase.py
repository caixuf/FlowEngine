#!/usr/bin/env python3
"""build_showcase.py — 生成综合 3D 展示页所需的场景清单。

读取 `scenarios/*.json`（顺序 + 时长参考 `scenarios/suite.json`），把每个场景
的原始定义打包进 `tools/flowboard/showcase/scenes.json`，供 showcase.html 在
浏览器里 fetch。

设计要点：
  * 清单里保存的是**原始 scenario 定义**（raw），不做转换。
    真正的「scenario → vis scene frame」转换由 JS 单一事实源
    `tools/flowboard/js/showcase/sceneAdapter.js` 在浏览器/门禁里完成，
    避免 Python 再写一份会漂移的转换逻辑。
  * 浏览器无法直接 fetch 仓库根的 scenarios/ 目录（flowmond 只托管
    tools/ 下的静态资源），所以把 raw 定义内联进 scenes.json。
  * 门禁测试 tests/vis_showcase_scenes.test.mjs 直接从磁盘读 scenarios/
    做真值校验，同时校验本清单未过期（raw 与磁盘一致）。

用法：
  python3 tools/build_showcase.py            # 生成清单
  python3 tools/build_showcase.py --check    # 只校验清单是否最新（CI 用），不写文件
"""
import argparse
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIOS_DIR = os.path.join(REPO_ROOT, "scenarios")
SUITE_PATH = os.path.join(SCENARIOS_DIR, "suite.json")
OUT_PATH = os.path.join(REPO_ROOT, "tools", "flowboard", "showcase", "scenes.json")


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def ordered_scenario_files():
    """按 suite.json 顺序返回场景文件名，未在 suite 里列出的追加在后。"""
    files = []
    seen = set()
    if os.path.exists(SUITE_PATH):
        suite = load_json(SUITE_PATH)
        for s in suite.get("scenarios", []):
            rel = s.get("file", "")
            name = os.path.basename(rel)
            path = os.path.join(SCENARIOS_DIR, name)
            if name and os.path.exists(path) and name not in seen:
                files.append((name, s.get("duration_s"), s.get("enabled", True)))
                seen.add(name)
    # 追加 suite 未列出的其它场景（suite.json 自身除外）
    for name in sorted(os.listdir(SCENARIOS_DIR)):
        if not name.endswith(".json") or name == "suite.json" or name in seen:
            continue
        files.append((name, None, True))
        seen.add(name)
    return files


def build_manifest():
    scenes = []
    for name, duration_s, enabled in ordered_scenario_files():
        raw = load_json(os.path.join(SCENARIOS_DIR, name))
        scenes.append({
            "file": "scenarios/" + name,
            "name": raw.get("name", name),
            "description": raw.get("description", ""),
            "duration_s": duration_s if duration_s is not None else raw.get("duration_s"),
            "enabled": bool(enabled),
            "raw": raw,
        })
    return {
        "generated_by": "tools/build_showcase.py",
        "count": len(scenes),
        "scenarios": scenes,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="只校验现有 scenes.json 是否最新，不写文件")
    args = ap.parse_args()

    manifest = build_manifest()
    new_text = json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"

    if args.check:
        if not os.path.exists(OUT_PATH):
            print("[FAIL] scenes.json 不存在，请运行 python3 tools/build_showcase.py", file=sys.stderr)
            return 1
        with open(OUT_PATH, "r", encoding="utf-8") as f:
            cur = f.read()
        if cur != new_text:
            print("[FAIL] scenes.json 已过期，请重新运行 python3 tools/build_showcase.py", file=sys.stderr)
            return 1
        print("[OK] scenes.json 最新（%d 个场景）" % manifest["count"])
        return 0

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(new_text)
    print("[OK] 写入 %s（%d 个场景）" % (os.path.relpath(OUT_PATH, REPO_ROOT), manifest["count"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
