"""Build small synthetic URDFs and STL meshes for tests (no real robot data needed)."""

from __future__ import annotations

import math
import struct
from pathlib import Path


def _box(size: str) -> str:
    return f'<geometry><box size="{size}"/></geometry>'


def link(name: str, mass: float | None = None, box: str | None = None, mesh: str | None = None) -> str:
    body = ""
    if mass is not None:
        body += (f'<inertial><mass value="{mass}"/>'
                 f'<inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>')
    if mesh:
        body += f'<visual><geometry><mesh filename="{mesh}" scale="0.001 0.001 0.001"/></geometry></visual>'
    if box:
        body += f"<collision>{_box(box)}</collision>"
    return f'<link name="{name}">{body}</link>'


def joint(name: str, kind: str, parent: str, child: str, xyz: str = "0 0 0", rpy: str = "0 0 0",
          axis: str | None = "1 0 0", limit: str | None = None, dynamics: str | None = None,
          mimic: str | None = None) -> str:
    parts = [f'<parent link="{parent}"/>', f'<child link="{child}"/>', f'<origin xyz="{xyz}" rpy="{rpy}"/>']
    if axis is not None and kind != "fixed":
        parts.append(f'<axis xyz="{axis}"/>')
    if limit:
        parts.append(f"<limit {limit}/>")
    if dynamics:
        parts.append(f"<dynamics {dynamics}/>")
    if mimic:
        parts.append(f'<mimic joint="{mimic}"/>')
    return f'<joint name="{name}" type="{kind}">{"".join(parts)}</joint>'


def robot(name: str, items: list[str], extras: str = "") -> str:
    return f'<?xml version="1.0"?>\n<robot name="{name}">{"".join(items)}{extras}</robot>\n'


ARM_LIMIT = 'lower="-1.5" upper="1.5" effort="50" velocity="1.0"'
BOGIE_LIMIT = 'lower="-0.5" upper="0.5" effort="100" velocity="10"'


def rover_like_items(frame_count: int = 4) -> list[str]:
    """A rover shaped like the OSU one: bogies + 6 wheels, a passive link, an arm, camera frames."""
    items = [link("base_link", 20, box="0.6 0.4 0.2", mesh="package://test_pkg/meshes/body.stl")]
    for side, y in (("left", 0.3), ("right", -0.3), ("rear", 0.0)):
        x = -0.3 if side == "rear" else 0.1
        items += [link(f"{side}_bogie", 2, box="0.4 0.05 0.05"),
                  joint(f"{side}_bogie_joint", "revolute", "base_link", f"{side}_bogie", xyz=f"{x} {y} 0",
                        axis="0 1 0", limit=BOGIE_LIMIT, dynamics='damping="2.0" friction="0.2"')]
        for end, dx in (("front", 0.15), ("back", -0.15)):
            items += [link(f"{side}_{end}_wheel", 1, box="0.2 0.08 0.2", mesh="meshes/wheel.stl"),
                      joint(f"{side}_{end}_wheel_joint", "continuous", f"{side}_bogie", f"{side}_{end}_wheel",
                            xyz=f"{dx} 0 -0.1", axis="0 1 0")]
    # passive linkage piece: continuous, no axis, no limit, no damping (like the rover's back links)
    items += [link("back_link", 0.5, box="0.2 0.02 0.02"),
              joint("back_link_joint", "continuous", "base_link", "back_link", xyz="-0.2 0.2 0", axis=None)]
    # arm: fixed mount + 5 revolute + 1 continuous wrist (with effort) + gripper with no inertial
    items += [link("arm_base", 1.0, box="0.1 0.1 0.1"),
              joint("arm_mount", "fixed", "base_link", "arm_base", xyz="0.2 0 0.15")]
    parent = "arm_base"
    for i in range(1, 6):
        items += [link(f"arm_{i}", 0.5, box="0.05 0.05 0.2"),
                  joint(f"arm_{i}_joint", "revolute", parent, f"arm_{i}", xyz="0 0 0.2",
                        axis="1 0 0" if i % 2 else "0 0 1", limit=ARM_LIMIT)]
        parent = f"arm_{i}"
    items += [link("wrist", 0.2, box="0.04 0.04 0.04"),
              joint("wrist_joint", "continuous", parent, "wrist", xyz="0 0 0.1", axis="0 0 1",
                    limit='effort="20" velocity="2"'),
              link("jaws", None, box="0.05 0.02 0.02"),  # shape but no mass
              joint("jaws_joint", "fixed", "wrist", "jaws", xyz="0 0 0.05")]
    # camera: a body with geometry + mass, and frame-only children (no shape, no mass)
    items += [link("camera_link", 0.1, box="0.02 0.1 0.02"),
              joint("camera_joint", "fixed", "base_link", "camera_link", xyz="0.3 0 0.2")]
    for i in range(frame_count):
        items += [link(f"camera_frame_{i}"),
                  joint(f"camera_frame_{i}_joint", "fixed", "camera_link", f"camera_frame_{i}",
                        xyz=f"0 {0.01 * i} 0", rpy="-1.5708 0 -1.5708")]
    # a massed sensor without geometry: NOT frame-only
    items += [link("imu", 0.05), joint("imu_joint", "fixed", "base_link", "imu", xyz="0 0 0.1")]
    return items


ROVER_EXTRAS = (
    '<material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>'
    '<gazebo reference="camera_frame_0"><sensor name="cam" type="camera"/></gazebo>'
)

ROVER_LIKE_TOTAL_MASS = 20 + 3 * 2 + 6 * 1 + 0.5 + 1.0 + 5 * 0.5 + 0.2 + 0.1 + 0.05


def write_stl(path: Path, triangles: list[tuple]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write(b"test mesh".ljust(80, b" "))
        handle.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            handle.write(struct.pack("<12fH", 0, 0, 1, *tri[0], *tri[1], *tri[2], 0))


def grid_triangles(n: int) -> list[tuple]:
    """n deterministic triangles on a grid (2 per cell), shared corners."""
    side = math.ceil(math.sqrt(n / 2)) + 1
    tris = []
    for i in range(side):
        for j in range(side):
            a, b, c, d = (i, j, 0.0), (i + 1, j, 0.0), (i, j + 1, 0.0), (i + 1, j + 1, 0.0)
            tris += [(a, b, c), (b, d, c)]
            if len(tris) >= n:
                return tris[:n]
    return tris[:n]


def write_rover_like(folder: Path, heavy_wheel: bool = False, frame_count: int = 4) -> Path:
    """Write rover_like URDF + its meshes into `folder`; returns the .urdf path."""
    folder.mkdir(parents=True, exist_ok=True)
    write_stl(folder / "test_pkg" / "meshes" / "body.stl", grid_triangles(200))
    write_stl(folder / "meshes" / "wheel.stl", grid_triangles(22000 if heavy_wheel else 100))
    path = folder / "rover_like.urdf"
    path.write_text(robot("rover_like", rover_like_items(frame_count), ROVER_EXTRAS), encoding="utf-8")
    return path


def chain(n: int) -> str:
    items = [link("link_0", 1.0)]
    for i in range(1, n):
        items += [link(f"link_{i}", 1.0),
                  joint(f"joint_{i}", "revolute", f"link_{i - 1}", f"link_{i}", xyz="0 0 0.1", limit=ARM_LIMIT)]
    return robot(f"chain_{n}", items)
