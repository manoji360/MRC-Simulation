"""Physical-parameter registry: every number the simulation's accuracy depends on, with its source.

A robot config can list `physical_parameters`, each:
    {"value": ..., "unit": "...", "status": "measured|rover_code|urdf|cad|estimate|assumed",
     "source": "where the number came from", "uncertainty": "e.g. ±10 %", "applies_to": "what uses it"}

* `check` and REPORT.md print a fidelity table of everything not yet `measured` / `rover_code`.
* The values reach Unity through manifest["physics"]["parameters"] (Unity can override them with
  `-param name=value` for experiments, e.g. the sensitivity sweep).
* Two of them change the URDF data in `prepare` (recorded, undone by `rebuild`):
    total_mass_kg            - scale every link's mass and inertia so the robot weighs this much
    center_of_mass_offset_m  - move the whole-robot centre of mass by [dx, dy, dz] (ROS root frame) by
                               shifting every link's centre of mass by that offset
"""

from __future__ import annotations

import difflib
import math

from .errors import PipelineError
from .kinematics import world_transforms
from .model import Robot

STATUSES = ("measured", "rover_code", "urdf", "cad", "estimate", "assumed")
"""Allowed values of a parameter's 'status', from most to least trusted (roughly)."""
TRUSTED = ("measured", "rover_code")
"""Statuses that count as real data (not listed in the fidelity table)."""

# Known parameters and their units. Unity reads the ones it uses by these names.
PARAMETER_SPECS: dict[str, str] = {
    # whole robot
    "total_mass_kg": "kg",
    "center_of_mass_offset_m": "m",           # [dx, dy, dz], ROS root frame
    "wheel_radius_m": "m",
    # traction and drivetrain
    "tyre_mu_concrete_static": "-", "tyre_mu_concrete_dynamic": "-",
    "tyre_mu_sand_static": "-", "tyre_mu_sand_dynamic": "-",
    "tyre_mu_rock_static": "-", "tyre_mu_rock_dynamic": "-",
    "wheel_max_torque_nm": "N*m",
    # passive joints (bogies)
    "passive_joint_friction_nm": "N*m",       # null = per-joint URDF <dynamics friction>
    "passive_joint_damping_nms": "N*m*s/rad", # null = per-joint URDF <dynamics damping>
    # arm
    "arm_joint_limits": "rad/s, rad/s^2",     # documents the joint_limits setting (values live there)
    "arm_max_torque_nm": "N*m",
    # sensors (used by the sensor simulation)
    "imu_rate_hz": "Hz", "gps_rate_hz": "Hz", "gps_noise_m": "m",
    "camera_rate_hz": "Hz", "camera_resolution": "px", "camera_hfov_rad": "rad",
    "lidar_model": "-",
}
"""Every known parameter name -> its unit. Unity reads the ones it uses by these names."""


def check_parameters(params: dict) -> None:
    """Raise PipelineError with a friendly message if the registry is malformed."""
    def fail(message: str) -> None:
        raise PipelineError(f"Invalid setting: physical_parameters {message}",
                            hint="Fix the entry in your config file (see init-config for the format).")

    if not isinstance(params, dict):
        fail("must map parameter names to {value, unit, status, source, uncertainty, applies_to}.")
    for name, entry in params.items():
        if name.startswith("_"):
            continue
        if name not in PARAMETER_SPECS:
            close = difflib.get_close_matches(name, PARAMETER_SPECS, n=1)
            fail(f"has an unknown parameter '{name}'" + (f" (did you mean '{close[0]}'?)." if close else "."))
        if not isinstance(entry, dict) or "value" not in entry or "status" not in entry:
            fail(f"['{name}'] must be an object with at least \"value\" and \"status\".")
        if entry["status"] not in STATUSES:
            fail(f"['{name}'].status must be one of {', '.join(STATUSES)}; got {entry['status']!r}.")
        unit = entry.get("unit", PARAMETER_SPECS[name])
        if unit != PARAMETER_SPECS[name]:
            fail(f"['{name}'].unit must be '{PARAMETER_SPECS[name]}'; got {unit!r}.")
        value = entry["value"]
        if name == "total_mass_kg" and value is not None and (not _number(value) or value <= 0):
            fail("['total_mass_kg'].value must be a positive number of kg.")
        if name == "center_of_mass_offset_m" and value is not None and not (
                isinstance(value, list) and len(value) == 3 and all(_number(v) for v in value)):
            fail("['center_of_mass_offset_m'].value must be [dx, dy, dz] in metres.")


def _number(v) -> bool:
    """True for int/float values (bool excluded)."""
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def parameters_block(params: dict) -> dict:
    """The registry as written to the manifest (notes dropped, unit filled in)."""
    return {name: {**entry, "unit": entry.get("unit", PARAMETER_SPECS[name])}
            for name, entry in (params or {}).items() if not name.startswith("_")}


def fidelity_rows(params: dict) -> list[dict]:
    """Parameters that are not measured or taken from the rover's own code: what limits accuracy."""
    rows = []
    for name, entry in parameters_block(params).items():
        if entry["status"] not in TRUSTED:
            rows.append({"name": name, "value": entry["value"], "unit": entry["unit"], "status": entry["status"],
                         "uncertainty": entry.get("uncertainty", "?"), "source": entry.get("source", "")})
    order = {s: i for i, s in enumerate(("assumed", "estimate", "cad", "urdf"))}
    return sorted(rows, key=lambda r: (order.get(r["status"], 9), r["name"]))


# -- mass adjustment ----------------------------------------------------------------------------
def _inertial_parts(link):
    """(inertial, mass, origin) elements of a link, or (None, None, None) if it has no <inertial><mass>."""
    inertial = link.element.find("inertial")
    if inertial is None or inertial.find("mass") is None:
        return None, None, None
    origin = inertial.find("origin")
    return inertial, inertial.find("mass"), origin


def mass_adjustment(robot: Robot, params: dict) -> dict | None:
    """What `prepare` must change for total_mass_kg / center_of_mass_offset_m, or None for nothing.

    Returns {"scale": s, "offset_ros": [dx, dy, dz]} (s = 1 / offset 0 mean no change)."""
    from .physics import total_mass

    entries = parameters_block(params)
    target = (entries.get("total_mass_kg") or {}).get("value")
    offset = (entries.get("center_of_mass_offset_m") or {}).get("value") or [0.0, 0.0, 0.0]
    current = total_mass(robot)
    scale = 1.0 if target is None or current <= 0 else target / current
    if abs(scale - 1.0) < 1e-4:   # within 0.01 %: a rounded copy of the URDF value, not a new measurement
        scale = 1.0
    if scale == 1.0 and all(abs(v) < 1e-12 for v in offset):
        return None
    return {"scale": scale, "offset_ros": [float(v) for v in offset]}


def apply_mass_adjustment(robot: Robot, adjustment: dict) -> dict[str, str]:
    """Scale masses/inertia and shift centres of mass in `robot`. Returns original <inertial> XML per link
    (for the manifest; `rebuild` puts it back)."""
    from .parse import element_to_string

    scale = adjustment["scale"]
    offset = adjustment["offset_ros"]
    poses = world_transforms(robot)
    originals: dict[str, str] = {}
    for name, link in robot.links.items():
        inertial, mass, origin = _inertial_parts(link)
        if inertial is None or link.mass is None or link.mass <= 0:
            continue
        if inertial.get("urdf_pipeline_placeholder") is not None:
            continue  # tiny placeholder masses stay as they are
        originals[name] = element_to_string(inertial)
        mass.set("value", f"{link.mass * scale:.9g}")
        node = inertial.find("inertia")
        if node is not None:
            for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                node.set(key, f"{float(node.get(key, '0')) * scale:.9g}")
        if any(offset):
            rot = [row[:3] for row in poses[name][:3]]   # link -> root rotation; offset is in the root frame
            local = [sum(rot[r][c] * offset[r] for r in range(3)) for c in range(3)]  # R^T * offset
            if origin is None:
                import xml.etree.ElementTree as ET
                origin = ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
                inertial.remove(origin)
                inertial.insert(0, origin)
            xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
            origin.set("xyz", " ".join(f"{xyz[i] + local[i]:.9g}" for i in range(3)))
    return originals


def restore_inertials(robot: Robot, originals: dict[str, str]) -> None:
    """Undo apply_mass_adjustment (used by rebuild)."""
    from .parse import element_from_string

    for name, xml in originals.items():
        link = robot.links[name].element
        old = link.find("inertial")
        position = list(link).index(old)
        link.remove(old)
        link.insert(position, element_from_string(xml))


def com_shift_check(before: list[float], after: list[float], offset: list[float]) -> float:
    """Distance between the intended and the achieved centre-of-mass shift (for tests/self-checks)."""
    return math.dist([a - b for a, b in zip(after, before)], offset)
