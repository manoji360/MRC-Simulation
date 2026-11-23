"""Tests of mesh mass properties, inertia plausibility checks and --fix-inertia."""

import math
import random

import pytest

from builders import write_stl
from urdf_pipeline.config import Config
from urdf_pipeline.mass_properties import eigen_symmetric, mesh_solid_properties, principal_axes
from urdf_pipeline.merge import rebuild
from urdf_pipeline.pipeline import check, prepare

IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
"""3x3 identity rotation."""


def cube_triangles(size=1.0):
    """Triangles of a closed axis-aligned cube from (0,0,0) to (size,size,size), outward-facing."""
    s = size
    v = [(0, 0, 0), (s, 0, 0), (s, s, 0), (0, s, 0), (0, 0, s), (s, 0, s), (s, s, s), (0, s, s)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def test_cube_mass_properties(tmp_path):
    """Volume, centroid and inertia of a cube match the textbook values."""
    path = tmp_path / "cube.stl"
    write_stl(path, cube_triangles(2.0))
    volume, centroid, inertia = mesh_solid_properties(path, [1, 1, 1], IDENTITY, [0, 0, 0])
    assert volume == pytest.approx(8.0)
    assert centroid == pytest.approx([1.0, 1.0, 1.0])
    # solid cube, unit density: I = m * (a^2 + a^2) / 12 = 8 * 8 / 12
    assert inertia[0][0] == pytest.approx(64 / 12) and inertia[0][1] == pytest.approx(0.0, abs=1e-9)


def test_principal_axes_reconstruct_tensor():
    """principal_axes() rebuilds random tensors exactly (R diag(I) R^T == tensor)."""
    rng = random.Random(4)
    for _ in range(50):
        a = [[rng.uniform(-1, 1) for _ in range(3)] for _ in range(3)]
        tensor = [[sum(a[i][k] * a[j][k] for k in range(3)) + (0.5 if i == j else 0) for j in range(3)] for i in range(3)]
        moments, (x, y, z, w) = principal_axes(tensor)
        r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        rebuilt = [[sum(r[i][k] * moments[k] * r[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        assert sum(rebuilt, []) == pytest.approx(sum(tensor, []), abs=1e-9)
        assert sorted(moments) == pytest.approx(sorted(eigen_symmetric(tensor)[0]))


def _robot_with_bad_inertia(folder):
    """A two-link robot (good and 100x-too-large inertia) using one cube mesh; returns the URDF path."""
    write_stl(folder / "meshes" / "block.stl", cube_triangles(0.2))  # 0.2 m cube
    good = 'ixx="0.0067" iyy="0.0067" izz="0.0067"'                  # 1 kg 0.2 m cube: 1*(0.04+0.04)/12
    bad = 'ixx="0.67" iyy="0.67" izz="0.67"'                          # 100x too large
    def link(name, inertia):
        return (f'<link name="{name}"><inertial><origin xyz="0.1 0.1 0.1"/><mass value="1"/>'
                f'<inertia {inertia} ixy="0" ixz="0" iyz="0"/></inertial>'
                f'<collision><geometry><mesh filename="meshes/block.stl"/></geometry></collision></link>')
    text = ('<?xml version="1.0"?><robot name="blocks">' + link("good", good) + link("bad", bad)
            + '<joint name="j" type="continuous"><parent link="good"/><child link="bad"/>'
              '<origin xyz="0 0 0.3"/><axis xyz="0 0 1"/></joint></robot>')
    path = folder / "blocks.urdf"
    path.write_text(text)
    return path


def test_check_flags_implausible_inertia(tmp_path):
    """check() flags only the link with implausible inertia, and says it is physically impossible."""
    result = check(_robot_with_bad_inertia(tmp_path))
    warning = next(m for level, m in result.physics if "disagrees with their shape" in m)
    assert "bad (100" in warning and "good" not in warning.split(":")[1]
    assert "1 of them are physically impossible" in warning


def test_hard_bound_and_visual_fallback(tmp_path):
    """The hard physical bound is exact for a cube; visual meshes are used when there is no collision mesh; multiple meshes are combined."""
    from urdf_pipeline.mass_properties import inertia_plausibility
    from urdf_pipeline.parse import load_urdf
    urdf = _robot_with_bad_inertia(tmp_path)
    robot = load_urdf(urdf)
    resolved = {"meshes/block.stl": tmp_path / "meshes" / "block.stl"}
    report = inertia_plausibility(robot, resolved)
    # 0.2 m cube, COM at its centre: no mass can sit farther than 0.1*sqrt(2) m from a centre axis -> I <= 1 kg * 0.02
    assert report["bad"]["max_possible_ratio"] == pytest.approx(0.67 / 0.02, abs=0.01) and report["bad"]["impossible"]
    assert report["good"]["max_possible_ratio"] == pytest.approx(0.0067 / 0.02, abs=0.01) and report["good"]["plausible"]
    # A link with only a visual mesh is still checked; two collision meshes are combined (twice the volume).
    bad = robot.links["bad"].element
    bad.find("collision").tag = "visual"
    assert inertia_plausibility(robot, resolved)["bad"]["shape_from"] == "1 visual mesh(es)"
    good = robot.links["good"].element
    import copy
    second = copy.deepcopy(good.find("collision"))
    second.insert(0, __import__("xml.etree.ElementTree", fromlist=["Element"]).Element("origin", {"xyz": "0.2 0 0"}))
    good.append(second)
    two = inertia_plausibility(robot, resolved)["good"]
    assert two["shape_from"] == "2 collision mesh(es)" and two["implied_density_kg_m3"] == pytest.approx(1 / 0.016)


def test_fix_inertia_replaces_and_rebuild_restores(tmp_path):
    """--fix-inertia replaces only the bad link with mesh-derived values, and rebuild restores the original."""
    urdf = _robot_with_bad_inertia(tmp_path / "in")
    m = prepare(urdf, tmp_path / "out", Config(fix_inertia=True)).manifest
    assert list(m["inertia_from_meshes"]) == ["bad"]
    fixed = m["physics"]["links"]["bad"]["principal_moments_unity"]
    assert fixed == pytest.approx([0.2 ** 2 * 2 / 12] * 3, rel=1e-3)
    assert m["physics"]["inertia_check"]["bad"]["ratio"] == pytest.approx(100.5, rel=0.01)
    assert rebuild(tmp_path / "out").differences == []
    # without the option, nothing changes
    m2 = prepare(urdf, tmp_path / "out2").manifest
    assert m2["inertia_from_meshes"] == {}
    assert m2["physics"]["links"]["bad"]["principal_moments_unity"] == pytest.approx([0.67] * 3)
