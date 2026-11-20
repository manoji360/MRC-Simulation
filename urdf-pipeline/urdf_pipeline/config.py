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
"""Unity's hard limit on links (ArticulationBodies) in one articulation."""
COLLAPSE_MODES = ("auto", "always", "never")
"""Allowed values of collapse_frames."""
VERTEX_MODES = ("raw", "welded")
"""Allowed values of mesh_vertex_mode."""
DRIVE_LIMIT_KEYS = (
    "max_linear_speed_mps",      # top forward/backward speed
    "max_angular_speed_radps",   # top turn rate
    "max_linear_accel_mps2",     # speeding up
    "max_linear_decel_mps2",     # slowing down (defaults to max_linear_accel_mps2)
    "max_angular_accel_radps2",  # turn-rate change
    "wheel_accel_rev_s2",        # per-wheel motor ramp (e.g. ODrive vel_ramp_rate / gear ratio)
    "controller_rate_hz",        # how often the drive controller updates its command
    "controller_wheel_separation_m",  # wheel_separation the controller uses to mix turning (skid steer: tuned)
    "controller_wheel_radius_m",      # wheel_radius the controller uses to turn m/s into wheel speed
)
"""Allowed keys of the drive_limits setting (see the comment on each)."""


@dataclass
class Config:
    """Every setting prepare/check understand. Load from JSON with load_config(); validate with check()."""
    max_links: int = 60
    """Most links allowed in one Unity articulation (1-64; default 60 leaves headroom)."""
    collapse_frames: str = "auto"
    """Remove frame-only links: 'auto' (only when over max_links), 'always' or 'never'."""
    split_meshes: bool = False
    """Cut big STL meshes into chunk files (off: Unity's importer already splits them)."""
    max_vertices: int = 65535
    """Vertex limit per mesh chunk when split_meshes is on."""
    mesh_vertex_mode: str = "raw"
    """How vertices are counted for the limit: 'raw' (3 per triangle, like Unity) or 'welded'."""
    keep_together: list[list[str]] = field(default_factory=list)
    """Groups of link names that must stay in one articulation part."""
    cut_cost: dict[str, int] = field(default_factory=dict)
    """Cost of cutting each joint type when splitting the robot (lower = cut first)."""
    mesh_roots: list[str] = field(default_factory=list)
    """Extra folders searched for mesh files (relative to the config file)."""
    ignore_missing_meshes: bool = False
    """Continue even if some mesh files cannot be found."""
    joint_roles: dict[str, str] = field(default_factory=dict)
    """Corrections to the guessed joint roles: joint name -> wheel, suspension, arm or passive."""
    massless_link_mass_kg: float | None = 0.001
    """Placeholder mass (kg) for links without <inertial>; None leaves them alone (Unity would use 1 kg)."""
    fix_inertia: bool = False
    """Replace inertia that disagrees with the link's mesh by more than 3x with mesh-derived values."""
    # Closed linkages URDF cannot describe (four-bars, differentials): follower joint -> how it follows.
    #   {"bogie_joint": {"leader": "link_joint", "multiplier": "opposite_rotation"}}
    # multiplier: a number, or "same_rotation" / "opposite_rotation" (sign worked out from the joint axes).
    joint_couplings: dict[str, dict] = field(default_factory=dict)
    """Closed linkages URDF cannot express: follower joint -> {leader, multiplier, offset_deg}."""
    # Springs/dampers URDF cannot describe (suspension shocks): joint -> settings ("auto" = from the robot's weight).
    #   {"link_joint": {"stiffness_nm_per_rad": "auto", "damping_nms_per_rad": "auto", "rest_deg": 0, "max_sag_deg": 2}}
    joint_springs: dict[str, dict] = field(default_factory=dict)
    """Springs/dampers URDF cannot express: joint -> {stiffness_nm_per_rad, damping_nms_per_rad, rest_deg, max_sag_deg}."""
    # Joints that cannot really move (e.g. a "linkage" whose bars meet at one pin): held fixed at zero in Unity.
    locked_joints: list[str] = field(default_factory=list)
    """Joints held fixed at zero in Unity (the URDF itself is not changed)."""
    # How the real drive controller limits speed and acceleration (URDF has no such data). Empty = no limits.
    drive_limits: dict[str, float | None] = field(default_factory=dict)
    """The real drive controller's speed/acceleration limits (keys in DRIVE_LIMIT_KEYS)."""
    # Real per-joint motion limits (e.g. MoveIt joint_limits.yaml): {"joint": {"max_velocity": rad/s, "max_acceleration": rad/s^2}}
    joint_limits: dict[str, dict] = field(default_factory=dict)
    """Real joint motion limits: joint -> {max_velocity rad/s, max_acceleration rad/s^2}."""
    # Every physical number the simulation depends on, with its source and certainty (see parameters.py).
    physical_parameters: dict[str, dict] = field(default_factory=dict)
    """Physical-parameter registry: name -> {value, unit, status, source, uncertainty, applies_to} (see parameters.py)."""

    def check(self) -> Config:
        """Validate every setting; raise PipelineError with a plain-English message on the first bad one. Returns self."""
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
        if self.massless_link_mass_kg is not None and (
            isinstance(self.massless_link_mass_kg, bool)
            or not isinstance(self.massless_link_mass_kg, (int, float))
            or self.massless_link_mass_kg <= 0
        ):
            fail(f"massless_link_mass_kg must be a positive number (kg) or null; got {self.massless_link_mass_kg!r}.")
        if not isinstance(self.fix_inertia, bool):
            fail(f"fix_inertia must be true or false; got {self.fix_inertia!r}.")
        if not isinstance(self.joint_couplings, dict):
            fail('joint_couplings must map follower joints to {"leader": ..., "multiplier": ...}.')
        for follower, spec in self.joint_couplings.items():
            multiplier = spec.get("multiplier", 1) if isinstance(spec, dict) else None
            if (not isinstance(spec, dict) or not isinstance(spec.get("leader"), str)
                    or not (multiplier in ("same_rotation", "opposite_rotation")
                            or (isinstance(multiplier, (int, float)) and not isinstance(multiplier, bool)))
                    or not isinstance(spec.get("offset_deg", 0), (int, float))):
                fail(f'joint_couplings["{follower}"] must look like {{"leader": "joint_name", '
                     '"multiplier": 1.0 | "same_rotation" | "opposite_rotation", "offset_deg": 0}}.')
        if not isinstance(self.joint_springs, dict):
            fail('joint_springs must map joint names to {"stiffness_nm_per_rad": "auto" | number, ...}.')
        def number(v):
            return isinstance(v, (int, float)) and not isinstance(v, bool)
        for name, spec in self.joint_springs.items():
            if (not isinstance(spec, dict)
                    or set(spec) - {"stiffness_nm_per_rad", "damping_nms_per_rad", "rest_deg", "max_sag_deg"}
                    or not all(v == "auto" or (number(v) and v >= 0)
                               for k, v in spec.items() if k in ("stiffness_nm_per_rad", "damping_nms_per_rad"))
                    or not number(spec.get("rest_deg", 0))
                    or not (number(spec.get("max_sag_deg", 2)) and spec.get("max_sag_deg", 2) > 0)):
                fail(f'joint_springs["{name}"] must look like {{"stiffness_nm_per_rad": "auto" | number, '
                     '"damping_nms_per_rad": "auto" | number, "rest_deg": 0, "max_sag_deg": 2}}.')
        if not isinstance(self.locked_joints, list) or not all(isinstance(j, str) for j in self.locked_joints):
            fail('locked_joints must be a list of joint names, e.g. ["front_link_joint"].')
        if not isinstance(self.drive_limits, dict):
            fail("drive_limits must map limit names to numbers, e.g. {\"max_linear_accel_mps2\": 2.0}.")
        for key, value in self.drive_limits.items():
            if key not in DRIVE_LIMIT_KEYS:
                close = difflib.get_close_matches(key, DRIVE_LIMIT_KEYS, n=1)
                fail(f"unknown drive_limits entry '{key}'" + (f" (did you mean '{close[0]}'?)" if close else
                                                             f"; valid: {', '.join(DRIVE_LIMIT_KEYS)}") + ".")
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0):
                fail(f"drive_limits['{key}'] must be a positive number or null; got {value!r}.")
        if not isinstance(self.joint_limits, dict):
            fail('joint_limits must map joint names to {"max_velocity": rad/s, "max_acceleration": rad/s^2}.')
        for name, spec in self.joint_limits.items():
            if (not isinstance(spec, dict) or set(spec) - {"max_velocity", "max_acceleration"}
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0 for v in spec.values())):
                fail(f'joint_limits["{name}"] must look like {{"max_velocity": 0.5, "max_acceleration": 2.0}} (positive, rad/s and rad/s^2).')
        from .parameters import check_parameters
        check_parameters(self.physical_parameters)
        roles = ("wheel", "suspension", "arm", "passive")
        if not isinstance(self.joint_roles, dict) or not all(
            isinstance(k, str) and v in roles for k, v in self.joint_roles.items()
        ):
            fail(f'joint_roles must map joint names to one of {", ".join(roles)}, e.g. {{"wrist_joint": "arm"}}.')
        return self

    @classmethod
    def from_dict(cls, data: dict, base_dir: Path | None = None) -> Config:
        """Build a Config from parsed JSON. Keys starting with '_' are notes; unknown keys get a 'did you mean' hint.
        mesh_roots are resolved relative to base_dir (the config file's folder).
        """
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
        """All settings as a plain dict (written into the manifest)."""
        return asdict(self)


def load_config(path: str | Path) -> Config:
    """Read and validate a JSON config file; friendly errors for a missing file or bad JSON."""
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
    "_joint_roles": "Optional: correct a guessed joint role (wheel, suspension, arm, passive), used by Unity physics tests.",
    "joint_roles": {},
    "_massless_link_mass_kg": "Unity gives links without <inertial> a mass of 1 kg; prepare gives them this tiny mass instead (null = leave them alone).",
    "massless_link_mass_kg": 0.001,
    "_fix_inertia": "true = replace inertia values that disagree with the link's collision mesh by more than 3x "
                    "(computed from the mesh, keeping the URDF mass and centre of mass). Changes physics data; off by default.",
    "fix_inertia": False,
    "_joint_couplings": "Optional: linkages URDF cannot describe. Example: keep a bogie parallel to the body in a "
                        "parallel four-bar: {\"bogie_joint\": {\"leader\": \"front_link_joint\", \"multiplier\": \"opposite_rotation\"}}. "
                        "URDF <mimic> joints are added automatically.",
    "joint_couplings": {},
    "_joint_springs": "Optional: springs/dampers URDF cannot describe, e.g. a suspension shock. \"auto\" works out the "
                      "stiffness that holds the joint within max_sag_deg of rest_deg under the robot's weight, "
                      "and critical damping. Example: {\"link_joint\": {\"stiffness_nm_per_rad\": \"auto\", "
                      "\"damping_nms_per_rad\": \"auto\", \"rest_deg\": 0, \"max_sag_deg\": 2}}",
    "joint_springs": {},
    "_locked_joints": "Optional: joints that should not move in simulation (held fixed at their zero position in Unity; "
                      "the URDF itself is not changed). Example: [\"front_link_joint\"]",
    "locked_joints": [],
    "_drive_limits": "Optional: the real drive controller's limits, so Unity accelerates and brakes like the robot "
                     "(URDF wheel joints usually have none). Keys: " + ", ".join(DRIVE_LIMIT_KEYS) + ". "
                     "Example (diff_drive_controller): {\"max_linear_speed_mps\": 1.0, \"max_linear_accel_mps2\": 0.5}",
    "drive_limits": {},
    "_joint_limits": "Optional: real joint speed/acceleration limits (rad/s, rad/s^2), e.g. from MoveIt joint_limits.yaml. "
                     "Unity moves those joints along a trapezoidal profile within them.",
    "joint_limits": {},
    "_physical_parameters": "Optional: every physical number with its source, e.g. {\"total_mass_kg\": {\"value\": 50.6, "
                            "\"unit\": \"kg\", \"status\": \"measured\", \"source\": \"wheel scales 2026-11-20\", "
                            "\"uncertainty\": \"±0.2 kg\", \"applies_to\": \"all link masses\"}}. status: measured, "
                            "rover_code, urdf, cad, estimate or assumed. check prints everything not yet measured.",
    "physical_parameters": {},
}
"""The file written by 'init-config': every setting with its default and a '_' help note."""
