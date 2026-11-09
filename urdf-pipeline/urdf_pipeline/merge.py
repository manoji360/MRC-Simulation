"""Rebuild the original robot from a prepared output folder (the inverse of prepare)."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

from .compare import diff_robots
from .errors import PipelineError
from .frames import restore_frames
from .manifest import FILE_NAME, read_manifest
from .model import Robot
from .parse import element_from_string, load_urdf, write_urdf


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def rebuild_robot(manifest: dict, out_dir: Path) -> Robot:
    """Undo every stage recorded in the manifest, reading the part URDFs from out_dir."""
    out_dir = Path(out_dir)
    source = manifest["source"]
    robot = Robot(name=source["robot_name"], attrib=dict(source["robot_attrib"]))
    for part in manifest["parts"]:
        part_file = out_dir / part["file"]
        if not part_file.is_file():
            raise PipelineError(f"Part file missing from the output folder: {part_file}")
        part_robot = load_urdf(part_file)
        for link in part_robot.links.values():
            robot.add_link(link.element)
        for joint in part_robot.joints.values():
            robot.add_joint(joint.element)
        robot.extras.extend(part_robot.extras)

    # 1. articulation split: re-add the bridge joints
    for bridge in manifest["bridge_joints"]:
        robot.add_joint(element_from_string(bridge["joint_xml"]))
    # 2. frame collapse: re-add frame links and their fixed joints
    restore_frames(robot, manifest["collapsed_frames"])
    # 3. mesh split: put each original <visual>/<collision> back where its chunks are
    for entry in manifest["mesh_chunks"]:
        link = robot.links[entry["link"]].element
        chunk_uris = set(entry["chunk_uris"])
        holders = [
            h
            for h in link
            if h.tag == entry["kind"]
            and h.find("geometry/mesh") is not None
            and h.find("geometry/mesh").get("filename") in chunk_uris
        ]
        if not holders:
            raise PipelineError(f"Mesh chunks for link '{entry['link']}' are missing from the output URDF.")
        position = list(link).index(holders[0])
        for holder in holders:
            link.remove(holder)
        link.insert(position, element_from_string(entry["original_xml"]))
    return robot


@dataclass
class RebuildResult:
    robot: Robot
    output: Path | None
    compared_with: Path | None
    differences: list[str]


def rebuild(manifest_path: str | Path, output: str | Path | None = None) -> RebuildResult:
    manifest_path = Path(manifest_path)
    if manifest_path.is_dir():
        manifest_path = manifest_path / FILE_NAME
    manifest = read_manifest(manifest_path)
    robot = rebuild_robot(manifest, manifest_path.parent)
    out_path = Path(output) if output else None
    if out_path:
        write_urdf(robot, out_path)

    source = Path(manifest["source"]["file"])
    compared_with = None
    differences: list[str] = []
    if source.is_file() and sha256_file(source) == manifest["source"]["sha256"]:
        compared_with = source
        differences = diff_robots(load_urdf(source), robot)
    return RebuildResult(robot, out_path, compared_with, differences)
