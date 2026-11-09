"""Pipeline settings, loadable from a JSON file.

Keys starting with "_" are ignored, so a config file can carry "_help" notes for
non-coders (JSON has no comments).
"""

from __future__ import annotations

import difflib
import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

from .errors import PipelineError

UNITY_LINK_LIMIT = 64
COLLAPSE_MODES = ("auto", "always", "never")
VERTEX_MODES = ("raw", "welded")


@dataclass
class Config:
    max_links: int = 60
    collapse_frames: str = "auto"
    split_meshes: bool = False
    max_vertices: int = 65535
    mesh_vertex_mode: str = "raw"
    keep_together: list[list[str]] = field(default_factory=list)
    cut_cost: dict[str, int] = field(default_factory=dict)
    mesh_roots: list[str] = field(default_factory=list)
    ignore_missing_meshes: bool = False

    def check(self) -> Config:
        def fail(message: str) -> None:
            raise PipelineError(f"Invalid setting: {message}", hint="Fix the value in your config file or command.")

        def is_int(value) -> bool:
            return isinstance(value, int) and not isinstance(value, bool)

        if not is_int(self.max_links) or not 1 <= self.max_links <= UNITY_LINK_LIMIT:
            fail(f"max_links must be a whole number from 1 to {UNITY_LINK_LIMIT} (Unity's limit); got {self.max_links!r}.")
        if self.collapse_frames not in COLLAPSE_MODES:
            fail(f"collapse_frames must be one of {', '.join(COLLAPSE_MODES)}; got {self.collapse_frames!r}.")
        if not isinstance(self.split_meshes, bool):
            fail(f"split_meshes must be true or false; got {self.split_meshes!r}.")
        if not is_int(self.max_vertices) or self.max_vertices < 3:
            fail(f"max_vertices must be a whole number of at least 3; got {self.max_vertices!r}.")
        if self.mesh_vertex_mode not in VERTEX_MODES:
            fail(f"mesh_vertex_mode must be one of {', '.join(VERTEX_MODES)}; got {self.mesh_vertex_mode!r}.")
        if not isinstance(self.keep_together, list) or not all(
            isinstance(g, list) and all(isinstance(n, str) for n in g) for g in self.keep_together
        ):
            fail('keep_together must be a list of lists of link names, e.g. [["arm_link1", "arm_link2"]].')
        if not isinstance(self.cut_cost, dict) or not all(
            isinstance(k, str) and is_int(v) for k, v in self.cut_cost.items()
        ):
            fail('cut_cost must map joint types to whole numbers, e.g. {"fixed": 1}.')
        if not isinstance(self.mesh_roots, list) or not all(isinstance(p, str) for p in self.mesh_roots):
            fail("mesh_roots must be a list of folder paths.")
        if not isinstance(self.ignore_missing_meshes, bool):
            fail(f"ignore_missing_meshes must be true or false; got {self.ignore_missing_meshes!r}.")
        return self

    @classmethod
    def from_dict(cls, data: dict, base_dir: Path | None = None) -> Config:
        if not isinstance(data, dict):
            raise PipelineError("The config file must contain a JSON object ({ ... }).")
        known = {f.name for f in fields(cls)}
        values = {}
        for key, value in data.items():
            if key.startswith("_"):
                continue
            if key not in known:
                close = difflib.get_close_matches(key, known, n=1)
                raise PipelineError(
                    f"Unknown setting '{key}' in the config file.",
                    hint=f"Did you mean '{close[0]}'?" if close else f"Valid settings: {', '.join(sorted(known))}.",
                )
            values[key] = value
        config = cls(**values).check()
        if base_dir is not None:
            config.mesh_roots = [str((base_dir / p).resolve()) for p in config.mesh_roots]
        return config

    def to_dict(self) -> dict:
        return asdict(self)


def load_config(path: str | Path) -> Config:
    path = Path(path)
    if not path.is_file():
        raise PipelineError(f"Config file not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise PipelineError(
            f"{path.name} is not valid JSON (line {exc.lineno}, column {exc.colno}): {exc.msg}",
            hint="Check for a missing comma or quote near that line. JSON does not allow comments.",
        ) from None
    return Config.from_dict(data, base_dir=path.parent)


EXAMPLE_CONFIG = {
    "_help": "Settings for urdf_pipeline. Keys starting with '_' are notes and are ignored.",
    "_max_links": "Most links allowed in one Unity articulation (Unity's hard limit is 64).",
    "max_links": 60,
    "_collapse_frames": "auto = remove frame-only links only when the robot is over max_links; always; never.",
    "collapse_frames": "auto",
    "_split_meshes": "true = cut big STL meshes into smaller files. Unity's URDF Importer already does this, so it is off by default.",
    "split_meshes": False,
    "max_vertices": 65535,
    "_mesh_vertex_mode": "raw = 3 vertices per triangle (what Unity does); welded = shared corners counted once.",
    "mesh_vertex_mode": "raw",
    "_keep_together": "Groups of links that should never be split into different parts, e.g. one wheel module.",
    "keep_together": [],
    "_cut_cost": "Optional: cost of cutting each joint type when splitting (lower = cut first).",
    "cut_cost": {},
    "_mesh_roots": "Extra folders to search for meshes (relative to this file).",
    "mesh_roots": [],
    "ignore_missing_meshes": False,
}
