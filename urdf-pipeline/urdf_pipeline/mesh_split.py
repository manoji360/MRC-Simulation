"""Split a heavy STL mesh into chunks that each stay under a vertex limit.

Triangles are ordered along a Morton (Z-order) curve of their centroids first, so each
chunk is one compact region of the part. Vertex coordinates are never modified.
"""

from __future__ import annotations

from collections import Counter

from .stl import StlMesh

VERTEX_MODES = ("raw", "welded")

# Spread the low 10 bits of an integer so they occupy every third bit.
_SPREAD = []
for _i in range(1024):
    _v = 0
    for _bit in range(10):
        if _i & (1 << _bit):
            _v |= 1 << (3 * _bit)
    _SPREAD.append(_v)


def vertex_count(mesh: StlMesh, mode: str) -> int:
    return mesh.raw_vertex_count if mode == "raw" else mesh.welded_vertex_count


def needs_split(mesh: StlMesh, max_vertices: int, mode: str = "raw") -> bool:
    return vertex_count(mesh, mode) > max_vertices


def _morton_order(triangles: list[tuple]) -> list[int]:
    if not triangles:
        return []
    cx = [(t[3] + t[6] + t[9]) / 3 for t in triangles]
    cy = [(t[4] + t[7] + t[10]) / 3 for t in triangles]
    cz = [(t[5] + t[8] + t[11]) / 3 for t in triangles]

    def quantize(values):
        lo, hi = min(values), max(values)
        scale = 1023 / (hi - lo) if hi > lo else 0.0
        return [int((v - lo) * scale) for v in values]

    qx, qy, qz = quantize(cx), quantize(cy), quantize(cz)
    keys = [_SPREAD[x] | (_SPREAD[y] << 1) | (_SPREAD[z] << 2) for x, y, z in zip(qx, qy, qz)]
    return sorted(range(len(triangles)), key=lambda i: (keys[i], i))


def split_mesh(mesh: StlMesh, max_vertices: int, mode: str = "raw") -> list[StlMesh]:
    """Return chunks whose vertex count (per `mode`) is <= max_vertices.

    mode "raw":    3 vertices per triangle - how Unity's STL importer builds meshes.
    mode "welded": shared corners counted once.
    """
    if mode not in VERTEX_MODES:
        raise ValueError(f"mode must be one of {VERTEX_MODES}")
    if max_vertices < 3:
        raise ValueError("max_vertices must be at least 3")
    tris = mesh.triangles
    order = _morton_order(tris)
    groups: list[list[int]] = []

    if mode == "raw":
        per_chunk = max_vertices // 3
        groups = [order[i : i + per_chunk] for i in range(0, len(order), per_chunk)]
    else:
        current: list[int] = []
        seen: set = set()
        for index in order:
            t = tris[index]
            corners = (t[3:6], t[6:9], t[9:12])
            new = {c for c in corners if c not in seen}
            if current and len(seen) + len(new) > max_vertices:
                groups.append(current)
                current, seen = [], set()
                new = set(corners)
            current.append(index)
            seen |= new
        if current:
            groups.append(current)

    return [StlMesh(header=mesh.header, triangles=[tris[i] for i in group]) for group in groups]


def triangles_match(original: StlMesh, chunks: list[StlMesh]) -> bool:
    """True if the chunks contain exactly the original triangles (none lost or changed)."""
    combined: Counter = Counter()
    for chunk in chunks:
        combined.update(chunk.triangles)
    return combined == Counter(original.triangles)
