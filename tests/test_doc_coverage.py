"""Every function, class, field, property and constant must have a description (shown when hovering in the IDE)."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
"""Repository root."""


def test_every_symbol_is_documented():
    """tools/doc_coverage.py finds no undocumented symbol in the Python or C# code."""
    result = subprocess.run([sys.executable, str(ROOT / "tools" / "doc_coverage.py")], capture_output=True, text=True)
    assert result.returncode == 0, "Add a description (docstring or /// <summary>) for:\n" + result.stdout
