"""Mass properties from meshes, and plausibility checks of URDF <inertial> data.

CAD exports often get inertia wrong (unit mix-ups, default densities). We estimate each link's
inertia from its collision mesh, assuming uniform density and the URDF's own mass, and compare.
A uniform solid is an approximation (real wheels carry more mass at the rim, roughly up to 2x),
so only large disagreements (default: more than 3x) are flagged.

Also: principal axes of an inertia tensor (used to give Unity exact values, because the URDF
Importer's own eigen-decomposition mis-rotates non-diagonal tensors).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .coordinates import matrix_to_quaternion
from .kinematics import parse_origin, rpy_matrix
from .model import Link, Robot
from .stl import read_stl

PLAUSIBLE_RATIO = 3.0
"""URDF vs mesh inertia may differ by up to this factor before a link is flagged (real parts are not uniform solids)."""


def mesh_solid_properties(path: Path, scale, rotation, translation):
    """Volume, centroid and inertia about the centroid (per unit density) of a closed mesh,
    expressed in the link frame. None if the mesh is not a closed solid."""
    volume = 0.0
    centroid = [0.0, 0.0, 0.0]
    second = [[0.0] * 3 for _ in range(3)]
    for tri in read_stl(path).triangles:
        pts = []
        for k in (3, 6, 9):
            v = (tri[k] * scale[0], tri[k + 1] * scale[1], tri[k + 2] * scale[2])
            pts.append([sum(rotation[i][j] * v[j] for j in range(3)) + translation[i] for i in range(3)])
        a, b, c = pts
        det = (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
               + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        volume += det
        for i in range(3):
            centroid[i] += det * (a[i] + b[i] + c[i]) / 4.0
            for j in range(3):
                second[i][j] += det / 20.0 * (2 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j])
                                              + a[i] * b[j] + a[j] * b[i] + a[i] * c[j]
                                              + a[j] * c[i] + b[i] * c[j] + b[j] * c[i])
    if abs(volume) < 1e-12:
        return None
    sign = 1.0 if volume > 0 else -1.0  # inward-facing normals give negative volume
    volume *= sign
    centroid = [sign * c / volume for c in centroid]
    second = [[sign * s for s in row] for row in second]
    cov = [[second[i][j] - volume * centroid[i] * centroid[j] for j in range(3)] for i in range(3)]
    trace = cov[0][0] + cov[1][1] + cov[2][2]
    inertia = [[(trace if i == j else 0.0) - cov[i][j] for j in range(3)] for i in range(3)]
    return volume, centroid, inertia


def urdf_inertia_tensor(link: Link):
    """URDF inertia tensor rotated into the link frame, and the inertial origin xyz."""
    inertial = link.element.find("inertial")
    node = None if inertial is None else inertial.find("inertia")
    if node is None:
        return None, None
    v = {k: float(node.get(k, "0")) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    tensor = [[v["ixx"], v["ixy"], v["ixz"]], [v["ixy"], v["iyy"], v["iyz"]], [v["ixz"], v["iyz"], v["izz"]]]
    xyz, rpy = parse_origin(inertial)
    r = rpy_matrix(rpy)
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    tensor = [[sum(r[i][k] * sum(tensor[k][m] * rt[m][j] for m in range(3)) for k in range(3))
               for j in range(3)] for i in range(3)]
    return tensor, xyz


def link_meshes(link: Link, resolved: dict) -> tuple[str, list[tuple[Path, list, list, list]]]:
    """All STL meshes describing the link's shape, as (path, scale, rotation, translation).

    Uses every collision mesh; falls back to the visual meshes when the link has no STL collision
    (e.g. camera models that only ship a visual mesh). Returns (source, meshes)."""
    for tag in ("collision", "visual"):
        found = []
        for element in link.element.findall(tag):
            mesh = element.find("geometry/mesh")
            if mesh is None:
                continue
            path = resolved.get(mesh.get("filename"))
            if path is None or path.suffix.lower() != ".stl":
                continue
            scale = [float(s) for s in (mesh.get("scale") or "1 1 1").split()]
            xyz, rpy = parse_origin(element)
            found.append((path, scale, rpy_matrix(rpy), list(xyz)))
        if found:
            return tag, found
    return "", []


def combined_solid_properties(meshes):
    """Volume, centroid and inertia about the combined centroid (per unit density) of several solids."""
    parts = [p for p in (mesh_solid_properties(*m) for m in meshes) if p is not None]
    if not parts:
        return None
    volume = sum(p[0] for p in parts)
    centroid = [sum(p[0] * p[1][i] for p in parts) / volume for i in range(3)]
    inertia = [[0.0] * 3 for _ in range(3)]
    for v, c, unit in parts:  # parallel-axis theorem to the combined centroid
        d = [c[i] - centroid[i] for i in range(3)]
        d2 = sum(x * x for x in d)
        for i in range(3):
            for j in range(3):
                inertia[i][j] += unit[i][j] + v * ((d2 if i == j else 0.0) - d[i] * d[j])
    return volume, centroid, inertia


def max_possible_ratio(tensor, com, mass: float, meshes) -> float:
    """URDF principal moment / the largest moment ANY mass distribution inside the shape could have.

    For each principal axis, all the mass at the mesh point farthest from that axis (through the URDF
    centre of mass) gives the upper bound m * r^2. Above 1 the value is physically impossible,
    whatever the density inside the part."""
    moments, vectors = eigen_symmetric(tensor)
    axes = [[vectors[r][c] for r in range(3)] for c in range(3)]
    reach = [0.0, 0.0, 0.0]
    for path, scale, rotation, translation in meshes:
        for tri in read_stl(path).triangles:
            for k in (3, 6, 9):
                v = (tri[k] * scale[0], tri[k + 1] * scale[1], tri[k + 2] * scale[2])
                d = [sum(rotation[i][j] * v[j] for j in range(3)) + translation[i] - com[i] for i in range(3)]
                r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
                for n, a in enumerate(axes):
                    along = d[0] * a[0] + d[1] * a[1] + d[2] * a[2]
                    reach[n] = max(reach[n], r2 - along * along)
    ratios = [moments[n] / (mass * reach[n]) if reach[n] > 0 else float("inf") for n in range(3) if moments[n] > 0]
    return max(ratios) if ratios else 0.0


def inertia_plausibility(robot: Robot, resolved: dict, ratio_limit: float = PLAUSIBLE_RATIO) -> dict:
    """Per link with mass + STL meshes: URDF vs mesh-estimated inertia and centre of mass.

    ratio: URDF / uniform-solid estimate (approximate; flagged outside 1/ratio_limit..ratio_limit).
    max_possible_ratio: URDF / hard physical upper bound (above 1 = impossible for any density)."""
    report = {}
    for link in robot.links.values():
        mass = link.mass
        tensor, com = urdf_inertia_tensor(link)
        if not mass or tensor is None:
            continue
        source, meshes = link_meshes(link, resolved)
        props = combined_solid_properties(meshes) if meshes else None
        if props is None:
            continue
        volume, centroid, unit_inertia = props
        density = mass / volume
        mesh_inertia = [[x * density for x in row] for row in unit_inertia]
        urdf_moments = sorted(eigen_symmetric(tensor)[0])
        mesh_moments = sorted(eigen_symmetric(mesh_inertia)[0])
        ratio = max(urdf_moments) / max(mesh_moments) if max(mesh_moments) > 0 else float("inf")
        bound = max_possible_ratio(tensor, com, mass, meshes)
        report[link.name] = {
            "mass": mass,
            "shape_from": f"{len(meshes)} {source} mesh(es)",
            "urdf_principal_moments": [round(m, 6) for m in urdf_moments],
            "mesh_principal_moments": [round(m, 6) for m in mesh_moments],
            "ratio": round(ratio, 2),
            "max_possible_ratio": round(bound, 2),
            "impossible": bound > 1.0,
            "plausible": 1.0 / ratio_limit <= ratio <= ratio_limit and bound <= 1.0,
            "implied_density_kg_m3": round(density, 1),
            "com_offset_m": round(math.dist(com, centroid), 4),
            "mesh_inertia_about_centroid": [[round(x, 9) for x in row] for row in mesh_inertia],
        }
    return report


def replace_inertia(link: Link, tensor) -> None:
    """Write `tensor` (link frame) into the link's <inertia>, zeroing the inertial rotation.
    Keeps the URDF mass and centre of mass (inertial origin xyz)."""
    inertial = link.element.find("inertial")
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.SubElement(inertial, "origin", {"xyz": "0 0 0"})
    origin.set("rpy", "0 0 0")
    node = inertial.find("inertia")
    for key, (i, j) in {"ixx": (0, 0), "iyy": (1, 1), "izz": (2, 2), "ixy": (0, 1), "ixz": (0, 2), "iyz": (1, 2)}.items():
        node.set(key, f"{tensor[i][j]:.9g}")


# -- principal axes ------------------------------------------------------------------------
def eigen_symmetric(m, sweeps: int = 50):
    """Eigenvalues and eigenvectors (as columns of a matrix) of a symmetric 3x3 matrix (Jacobi)."""
    a = [row[:] for row in m]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(sweeps):
        off = sum(a[i][j] ** 2 for i in range(3) for j in range(3) if i != j)
        if off < 1e-30:
            break
        for p in range(3):
            for q in range(p + 1, 3):
                if abs(a[p][q]) < 1e-300:
                    continue
                theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
                t = (1 if theta >= 0 else -1) / (abs(theta) + math.sqrt(theta * theta + 1))
                c = 1 / math.sqrt(t * t + 1)
                s = t * c
                for k in range(3):
                    akp, akq = a[k][p], a[k][q]
                    a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
                for k in range(3):
                    apk, aqk = a[p][k], a[q][k]
                    a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
                for k in range(3):
                    vkp, vkq = v[k][p], v[k][q]
                    v[k][p], v[k][q] = c * vkp - s * vkq, s * vkp + c * vkq
    return [a[0][0], a[1][1], a[2][2]], v


def principal_axes(tensor) -> tuple[list[float], tuple[float, float, float, float]]:
    """Principal moments and the rotation (quaternion x, y, z, w) whose matrix has the principal
    axes as columns, so that tensor = R diag(moments) R^T (Unity's inertiaTensorRotation)."""
    moments, vectors = eigen_symmetric(tensor)
    det = (vectors[0][0] * (vectors[1][1] * vectors[2][2] - vectors[1][2] * vectors[2][1])
           - vectors[0][1] * (vectors[1][0] * vectors[2][2] - vectors[1][2] * vectors[2][0])
           + vectors[0][2] * (vectors[1][0] * vectors[2][1] - vectors[1][1] * vectors[2][0]))
    if det < 0:  # make it a proper rotation
        for row in vectors:
            row[2] = -row[2]
    return moments, matrix_to_quaternion(vectors)
