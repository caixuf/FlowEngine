# FlowBoard 可视化层测试

针对 `tools/flowboard_server.py` / `tools/flowboard_normalize.py` / 数据契约的 pytest 测试。

## 运行

```bash
pip install -r tools/tests/requirements-dev.txt
python3 -m pytest tools/tests -v
```

仅使用 dev 依赖(`pytest` + `jsonschema`);生产运行仍是零 pip 依赖。
