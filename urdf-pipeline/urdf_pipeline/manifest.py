"""manifest.json: the record of everything the pipeline changed.

It is what `rebuild` uses to restore the original robot and what a Unity script reads to
reconnect parts and re-create collapsed frames (values in Unity coordinates included).
"""

from __future__ import annotations

import json
from pathlib import Path

from .errors import PipelineError

TOOL_NAME = "urdf_pipeline"
SCHEMA_VERSION = 1
FILE_NAME = "manifest.json"
REQUIRED_KEYS = ("tool", "schema_version", "source", "parts", "bridge_joints", "collapsed_frames", "mesh_chunks")


def write_manifest(data: dict, path: str | Path) -> None:
    Path(path).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")


def read_manifest(path: str | Path) -> dict:
    path = Path(path)
    if path.is_dir():
        path = path / FILE_NAME
    if not path.is_file():
        raise PipelineError(f"Manifest not found: {path}", hint="Point to the manifest.json inside a prepared output folder.")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise PipelineError(f"{path} is not valid JSON: {exc.msg} (line {exc.lineno}).") from None
    if not isinstance(data, dict) or data.get("tool") != TOOL_NAME:
        raise PipelineError(f"{path} was not written by {TOOL_NAME}.")
    if data.get("schema_version") != SCHEMA_VERSION:
        raise PipelineError(
            f"{path} uses manifest version {data.get('schema_version')}, but this tool reads version {SCHEMA_VERSION}.",
            hint="Run 'prepare' again with this version of the tool.",
        )
    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        raise PipelineError(f"{path} is incomplete (missing: {', '.join(missing)}).")
    return data


def is_pipeline_manifest(path: Path) -> bool:
    try:
        return json.loads(path.read_text(encoding="utf-8")).get("tool") == TOOL_NAME
    except (OSError, ValueError, AttributeError):
        return False
