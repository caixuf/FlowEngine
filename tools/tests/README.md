# tools/tests — 算法回归测试

针对控制节点等核心算法的 pytest 回归测试（纯 Python，镜像 C 端数学实现，
便于在 CI 中快速捕捉数值回归）。当前覆盖 `control_node` 的 DATA_TIMEOUT
曲线回退分支（`test_phase5_curve_fallback.py`）。

## 运行

```bash
pip install -r tools/tests/requirements-dev.txt
python3 -m pytest tools/tests -v
```

仅使用 dev 依赖(`pytest`);生产运行零 pip 依赖。
