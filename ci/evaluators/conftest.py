"""Shared pytest fixtures: make tools/ importable."""
import os
import sys

import pytest

TOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)


@pytest.fixture()
def schema_available():
    """Skip a test unless the jsonschema package is installed."""
    pytest.importorskip("jsonschema")
    return True
