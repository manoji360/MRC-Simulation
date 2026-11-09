"""Command line interface: python -m urdf_pipeline <command> ...

Exit codes: 0 success, 1 user-fixable error, 2 invalid URDF / check failed, 3 internal bug.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import traceback
from pathlib import Path

from . import __version__
from .config import EXAMPLE_CONFIG, Config, load_config
from .errors import PipelineError, ValidationError


def _log(message: str) -> None:
    print(f"  - {message}", flush=True)


def _config_from_args(args: argparse.Namespace) -> Config:
    config = load_config(args.config) if getattr(args, "config", None) else Config()
    overrides = {}
    if getattr(args, "max_links", None) is not None:
        overrides["max_links"] = args.max_links
    if getattr(args, "split_meshes", False):
        overrides["split_meshes"] = True
    if getattr(args, "collapse_frames", None):
        overrides["collapse_frames"] = args.collapse_frames
    if getattr(args, "ignore_missing_meshes", False):
        overrides["ignore_missing_meshes"] = True
    if getattr(args, "mesh_root", None):
        overrides["mesh_roots"] = list(config.mesh_roots) + [str(Path(p).resolve()) for p in args.mesh_root]
    return dataclasses.replace(config, **overrides).check()


def _cmd_check(args: argparse.Namespace) -> int:
    from .pipeline import check
    from .report import render_check

    config = _config_from_args(args)
    result = check(args.urdf, config)
    print("\n".join(render_check(result, config)))
    return 0 if result.ok else 2


def _cmd_prepare(args: argparse.Namespace) -> int:
    from .pipeline import prepare

    config = _config_from_args(args)
    urdf = Path(args.urdf)
    out_dir = Path(args.output) if args.output else Path.cwd() / "output" / urdf.stem
    print(f"Preparing {urdf.name} for Unity")
    result = prepare(urdf, out_dir, config, log=_log)
    stats = result.manifest["stats"]
    print()
    print(f"Done. Unity-ready folder: {result.out_dir}")
    for path in result.urdf_files:
        print(f"  {path.name}")
    print(f"  {stats['mesh_files_copied']} mesh files copied, {stats['mesh_chunks_written']} mesh chunks written")
    print("  Self-check passed: the output rebuilds to exactly the original URDF.")
    for warning in result.warnings:
        print(f"[WARN] {warning}")
    print(f"Next: open {result.report_path.name} in that folder for the Unity import steps.")
    return 0


def _cmd_rebuild(args: argparse.Namespace) -> int:
    from .merge import rebuild

    result = rebuild(args.manifest, args.output)
    if result.output:
        print(f"Rebuilt URDF written to {result.output}")
    if result.compared_with is None:
        print("[INFO] Original URDF not found (or changed since prepare); comparison skipped.")
        return 0
    if result.differences:
        print(f"[FAIL] Rebuilt robot differs from {result.compared_with.name}:")
        for diff in result.differences:
            print(f"         - {diff}")
        return 2
    print(f"[OK]   Rebuilt robot is identical to {result.compared_with.name}.")
    return 0


def _cmd_init_config(args: argparse.Namespace) -> int:
    path = Path(args.path)
    if path.exists() and not args.force:
        raise PipelineError(f"{path} already exists.", hint="Use --force to overwrite it.")
    path.write_text(json.dumps(EXAMPLE_CONFIG, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote example config to {path}. Edit it, then pass it with --config {path}.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="urdf_pipeline",
        description="Check a URDF robot model and prepare it for import into Unity.",
    )
    parser.add_argument("--version", action="version", version=f"urdf_pipeline {__version__}")
    sub = parser.add_subparsers(dest="command", required=True, metavar="COMMAND")

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("urdf", help="path to the .urdf file")
        p.add_argument("--config", help="JSON settings file (see init-config)")
        p.add_argument("--max-links", type=int, help="most links per Unity articulation (default 60, max 64)")
        p.add_argument("--mesh-root", action="append", metavar="FOLDER", help="extra folder to search for meshes (repeatable)")
        p.add_argument("--ignore-missing-meshes", action="store_true", help="continue even if mesh files are missing")

    p_check = sub.add_parser("check", help="report problems and what prepare would do (changes nothing)")
    add_common(p_check)
    p_check.set_defaults(func=_cmd_check)

    p_prep = sub.add_parser("prepare", help="build a Unity-ready folder from a URDF")
    add_common(p_prep)
    p_prep.add_argument("-o", "--output", help="output folder (default: ./output/<urdf name>)")
    p_prep.add_argument("--split-meshes", action="store_true", help="cut big STL meshes into smaller chunk files")
    p_prep.add_argument("--collapse-frames", choices=["auto", "always", "never"],
                        help="remove shape-less, mass-less links from physics (default auto: only when over the limit)")
    p_prep.set_defaults(func=_cmd_prepare)

    p_rebuild = sub.add_parser("rebuild", help="rebuild the original URDF from a prepared folder and compare")
    p_rebuild.add_argument("manifest", help="manifest.json (or the prepared folder)")
    p_rebuild.add_argument("-o", "--output", help="also write the rebuilt URDF here")
    p_rebuild.set_defaults(func=_cmd_rebuild)

    p_init = sub.add_parser("init-config", help="write an example settings file")
    p_init.add_argument("path", nargs="?", default="pipeline_config.json")
    p_init.add_argument("--force", action="store_true", help="overwrite an existing file")
    p_init.set_defaults(func=_cmd_init_config)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except ValidationError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        for problem in exc.problems:
            print(f"         - {problem}", file=sys.stderr)
        print(f"Fix: {exc.hint}", file=sys.stderr)
        return 2
    except PipelineError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        if exc.hint:
            print(f"Fix: {exc.hint}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        return 1
    except Exception:  # noqa: BLE001 - last-resort handler for bugs
        print("[BUG] Something went wrong inside urdf_pipeline. Please report this with the text below:", file=sys.stderr)
        traceback.print_exc()
        return 3
