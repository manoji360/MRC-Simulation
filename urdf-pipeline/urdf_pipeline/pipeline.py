"""The two user-facing operations: check() and prepare()."""

from __future__ import annotations

import shutil
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from . import __version__
from .compare import diff_robots
from .config import Config
from .coordinates import pose_blocks_from_matrix, unity_block_from_origin
from .errors import PipelineError, ValidationError
from .frames import collapse_frames, frame_only_links
from .kinematics import max_difference, parse_axis, parse_origin, world_transforms
from .manifest import FILE_NAME, SCHEMA_VERSION, TOOL_NAME, is_pipeline_manifest, write_manifest
from .merge import rebuild_robot, sha256_file
from .mesh_split import needs_split, split_mesh, triangles_match
from .meshes import chunk_uri, companion_files, is_absolute, mesh_uses, output_key, resolve_mesh
from .model import Robot
from .parse import element_to_string, load_urdf, validate, write_urdf
from .split import build_part_robots, plan_split, single_part_plan
from .stl import read_stl, triangle_count, write_stl

Log = Callable[[str], None]
UNITY_MESH_VERTEX_LIMIT = 65535


def _quiet(_message: str) -> None:
    pass


# -- check -------------------------------------------------------------------------------
@dataclass
class CheckResult:
    file: str
    robot: str
    links: int
    joints: int
    joint_types: dict[str, int]
    roots: list[str]
    problems: list[str]
    frame_only: int = 0
    meshes_total: int = 0
    missing_meshes: list[str] = field(default_factory=list)
    absolute_meshes: list[str] = field(default_factory=list)
    unplaceable_meshes: list[str] = field(default_factory=list)
    non_stl_meshes: list[str] = field(default_factory=list)
    heavy_meshes: list[tuple[str, int]] = field(default_factory=list)  # (uri, triangles)
    will_collapse: bool = False
    links_after: int = 0
    parts: int = 1

    @property
    def ok(self) -> bool:
        return not self.problems and not self.missing_meshes


def _mesh_roots(config: Config) -> list[Path]:
    return [Path(p) for p in config.mesh_roots]


def check(urdf_path: str | Path, config: Config | None = None) -> CheckResult:
    config = (config or Config()).check()
    urdf_path = Path(urdf_path)
    robot = load_urdf(urdf_path)
    problems = validate(robot)
    result = CheckResult(
        file=urdf_path.name,
        robot=robot.name,
        links=len(robot.links),
        joints=len(robot.joints),
        joint_types=dict(sorted(Counter(j.type for j in robot.joints.values()).items())),
        roots=robot.roots(),
        problems=problems,
    )

    uris = sorted({u.uri for u in mesh_uses(robot)})
    result.meshes_total = len(uris)
    for uri in uris:
        path = resolve_mesh(uri, urdf_path.parent, _mesh_roots(config))
        if is_absolute(uri):
            result.absolute_meshes.append(uri)
        elif output_key(uri) is None:
            result.unplaceable_meshes.append(uri)
        if path is None:
            result.missing_meshes.append(uri)
        elif path.suffix.lower() != ".stl":
            result.non_stl_meshes.append(uri)
        else:
            try:
                tris = triangle_count(path)
            except PipelineError:
                result.problems.append(f"Mesh '{uri}' is not a readable STL file.")
                continue
            if 3 * tris > UNITY_MESH_VERTEX_LIMIT:
                result.heavy_meshes.append((uri, tris))
    result.heavy_meshes.sort(key=lambda item: -item[1])
    if config.ignore_missing_meshes:
        result.missing_meshes = []

    if not problems:
        result.frame_only = len(frame_only_links(robot))
        work = robot.copy()
        result.will_collapse = _should_collapse(work, config)
        if result.will_collapse:
            collapse_frames(work)
        result.links_after = len(work.links)
        if len(work.links) > config.max_links:
            result.parts = len(plan_split(work, config.max_links, config.keep_together, config.cut_cost).parts)
    return result


def _should_collapse(robot: Robot, config: Config) -> bool:
    return config.collapse_frames == "always" or (
        config.collapse_frames == "auto" and len(robot.links) > config.max_links
    )


# -- prepare -----------------------------------------------------------------------------
@dataclass
class PrepareResult:
    out_dir: Path
    urdf_files: list[Path]
    manifest_path: Path
    report_path: Path
    manifest: dict
    warnings: list[str]


def _prepare_output_dir(out_dir: Path, urdf_path: Path) -> None:
    if out_dir.exists():
        if not out_dir.is_dir():
            raise PipelineError(f"The output path {out_dir} is a file, not a folder.")
        if urdf_path.is_relative_to(out_dir):
            raise PipelineError(
                "The output folder contains the input URDF.",
                hint="Choose a different output folder with -o.",
            )
        if any(out_dir.iterdir()):
            if is_pipeline_manifest(out_dir / FILE_NAME):
                shutil.rmtree(out_dir)  # our own previous output: safe to replace
            else:
                raise PipelineError(
                    f"The output folder {out_dir} already contains files that this tool did not create.",
                    hint="Choose an empty or new folder with -o (nothing was changed).",
                )
    out_dir.mkdir(parents=True, exist_ok=True)


def _split_meshes(robot: Robot, resolved: dict[str, Path | None], out_dir: Path,
                  config: Config, log: Log, warnings: list[str]) -> tuple[list[dict], dict]:
    mesh_files: dict[str, dict] = {}
    for uri in sorted({u.uri for u in mesh_uses(robot)}):
        path = resolved.get(uri)
        if path is None or path.suffix.lower() != ".stl":
            continue
        mesh = read_stl(path)
        if not needs_split(mesh, config.max_vertices, config.mesh_vertex_mode):
            continue
        if output_key(uri) is None:
            warnings.append(f"Mesh '{uri}' is too big but has an absolute path, so it was not split.")
            continue
        chunks = split_mesh(mesh, config.max_vertices, config.mesh_vertex_mode)
        if not triangles_match(mesh, chunks):
            raise PipelineError(f"Internal error: splitting {path.name} lost triangles. Please report this bug.")
        log(f"Split {path.name} ({len(mesh.triangles):,} triangles) into {len(chunks)} chunks")
        chunk_info = []
        for index, chunk in enumerate(chunks):
            curi = chunk_uri(uri, index)
            write_stl(chunk, out_dir / output_key(curi))
            chunk_info.append({"uri": curi, "triangles": len(chunk.triangles),
                               "vertices": chunk.raw_vertex_count if config.mesh_vertex_mode == "raw"
                               else chunk.welded_vertex_count})
        mesh_files[uri] = {"sha256": sha256_file(path), "triangles": len(mesh.triangles), "chunks": chunk_info}

    entries = []
    for use in mesh_uses(robot):
        if use.uri not in mesh_files:
            continue
        link = robot.links[use.link].element
        position = list(link).index(use.holder)
        base_name = use.holder.get("name") or use.kind
        chunk_uris = [c["uri"] for c in mesh_files[use.uri]["chunks"]]
        entries.append({"link": use.link, "kind": use.kind, "source_uri": use.uri,
                        "original_xml": element_to_string(use.holder), "chunk_uris": chunk_uris})
        link.remove(use.holder)
        for index, curi in enumerate(chunk_uris):
            clone = element_from_holder(use.holder)
            clone.find("geometry/mesh").set("filename", curi)
            clone.set("name", f"{base_name}_chunk{index:02d}")
            link.insert(position + index, clone)
    return entries, mesh_files


def element_from_holder(holder):
    import copy

    clone = copy.deepcopy(holder)
    clone.tail = None
    return clone


def _copy_meshes(parts: list[Robot], resolved: dict[str, Path | None], skip: set[str],
                 out_dir: Path, warnings: list[str]) -> int:
    copied = 0
    placed: dict[str, Path] = {}
    for uri in sorted({u.uri for robot in parts for u in mesh_uses(robot)}):
        if uri in skip:
            continue
        key = output_key(uri)
        source = resolved.get(uri)
        if key is None:
            warnings.append(f"Mesh '{uri}' uses an absolute path; it was not copied and will only work on this computer.")
            continue
        if source is None:
            continue
        if key in placed and placed[key] != source:
            warnings.append(f"Two different files map to '{key}'; kept {placed[key]}.")
            continue
        destination = out_dir / key
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        placed[key] = source
        copied += 1
        for rel in companion_files(source):  # textures / material files the mesh needs
            target = destination.parent / rel
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source.parent / rel, target)
                copied += 1
    return copied


def _safe_stem(name: str) -> str:
    cleaned = "".join(c if c.isalnum() or c in "-_" else "_" for c in name).strip("_")
    return cleaned or "robot"


def prepare(urdf_path: str | Path, out_dir: str | Path, config: Config | None = None,
            log: Log = _quiet) -> PrepareResult:
    """Turn a URDF into a validated, Unity-ready folder, and prove it is lossless."""
    config = (config or Config()).check()
    urdf_path = Path(urdf_path).resolve()
    out_dir = Path(out_dir).resolve()

    log(f"Reading {urdf_path.name}")
    original = load_urdf(urdf_path)
    problems = validate(original)
    if problems:
        raise ValidationError(problems, urdf_path.name)

    log("Finding mesh files")
    resolved = {u.uri: resolve_mesh(u.uri, urdf_path.parent, _mesh_roots(config)) for u in mesh_uses(original)}
    missing = sorted(uri for uri, path in resolved.items() if path is None)
    warnings: list[str] = []
    if missing and not config.ignore_missing_meshes:
        shown = "\n  ".join(missing[:15]) + (f"\n  ... and {len(missing) - 15} more" if len(missing) > 15 else "")
        raise PipelineError(
            f"{len(missing)} mesh file(s) used by the URDF could not be found:\n  {shown}",
            hint=("Put the mesh folders next to the .urdf (package://NAME/... looks for a NAME folder "
                  "beside it), or add the folder that contains them with --mesh-root / \"mesh_roots\"."),
        )
    for uri in missing:
        warnings.append(f"Mesh not found (ignored): {uri}")

    _prepare_output_dir(out_dir, urdf_path)
    robot = original.copy()

    mesh_chunks: list[dict] = []
    mesh_files: dict = {}
    if config.split_meshes:
        log("Checking mesh sizes")
        mesh_chunks, mesh_files = _split_meshes(robot, resolved, out_dir, config, log, warnings)

    frames: list[dict] = []
    if _should_collapse(robot, config):
        frames = collapse_frames(robot)
        log(f"Removed {len(frames)} frame-only links from the physics tree ({len(robot.links)} links left)")

    if len(robot.links) > config.max_links:
        plan = plan_split(robot, config.max_links, config.keep_together, config.cut_cost)
        warnings.extend(plan.warnings)
        log(f"Split the robot into {len(plan.parts)} parts of at most {config.max_links} links")
    else:
        plan = single_part_plan(robot)
    parts = build_part_robots(robot, plan)

    stem = _safe_stem(urdf_path.stem)
    urdf_files = []
    part_entries = []
    for part, part_robot in zip(plan.parts, parts):
        file_name = f"{stem}.urdf" if len(parts) == 1 else f"{stem}_{part.id}.urdf"
        write_urdf(part_robot, out_dir / file_name)
        urdf_files.append(out_dir / file_name)
        part_entries.append({"id": part.id, "file": file_name, "root_link": part.root_link, "links": part.links})

    part_of = {name: p["id"] for p in part_entries for name in p["links"]}
    bridges = []
    for joint_name in plan.cut_joints:
        joint = robot.joints[joint_name]
        xyz, rpy = parse_origin(joint.element)
        axis = parse_axis(joint.element)
        limit = joint.element.find("limit")
        bridges.append({
            "name": joint.name, "type": joint.type,
            "parent_link": joint.parent, "parent_part": part_of[joint.parent],
            "child_link": joint.child, "child_part": part_of[joint.child],
            "origin": {"xyz": list(xyz), "rpy": list(rpy)},
            "axis": list(axis) if axis else None,
            "limit": dict(limit.attrib) if limit is not None else None,
            "unity": unity_block_from_origin(xyz, rpy, axis),
            "joint_xml": element_to_string(joint.element),
        })

    log("Copying mesh files")
    chunk_uris = {c["uri"] for info in mesh_files.values() for c in info["chunks"]}
    copied = _copy_meshes(parts, resolved, chunk_uris, out_dir, warnings)

    poses = world_transforms(original)
    manifest = {
        "tool": TOOL_NAME,
        "tool_version": __version__,
        "schema_version": SCHEMA_VERSION,
        "coordinate_note": ("URDF/STL files stay in ROS coordinates (the Unity URDF Importer converts them). "
                            "Blocks named 'unity' are already converted for Unity scripts."),
        "source": {"file": str(urdf_path), "sha256": sha256_file(urdf_path),
                   "robot_name": original.name, "robot_attrib": original.attrib},
        "config": config.to_dict(),
        "stats": {
            "links_in": len(original.links), "joints_in": len(original.joints),
            "links_out": len(robot.links), "parts": len(parts),
            "frames_collapsed": len(frames), "meshes_split": len(mesh_files),
            "mesh_files_copied": copied, "mesh_chunks_written": len(chunk_uris),
        },
        "outputs": [p.name for p in urdf_files],
        "parts": part_entries,
        "bridge_joints": bridges,
        "collapsed_frames": frames,
        "mesh_files": mesh_files,
        "mesh_chunks": mesh_chunks,
        "reference_poses": {name: pose_blocks_from_matrix(t) for name, t in poses.items()},
        "warnings": warnings,
    }
    manifest_path = out_dir / FILE_NAME
    write_manifest(manifest, manifest_path)

    log("Self-check: rebuilding the original from the output")
    rebuilt = rebuild_robot(manifest, out_dir)
    differences = diff_robots(original, rebuilt)
    drift = max_difference(poses, world_transforms(rebuilt))
    if differences or drift > 1e-9:
        details = "; ".join(differences) or f"link poses moved by {drift}"
        raise PipelineError(f"Internal self-check failed ({details}). The output may be wrong; please report this bug.")

    from .report import render_prepare_report  # local import avoids a cycle

    report_path = out_dir / "REPORT.md"
    report_path.write_text(render_prepare_report(manifest), encoding="utf-8", newline="\n")
    return PrepareResult(out_dir, urdf_files, manifest_path, report_path, manifest, warnings)
