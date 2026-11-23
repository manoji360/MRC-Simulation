"""Minimal STL reader/writer (binary and ASCII), standard library only.

Triangles are kept exactly as stored (normal, three vertices, attribute bytes), so
writing a mesh back - or splitting it into chunks - never changes a coordinate.
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass
from pathlib import Path

from .errors import PipelineError

# normal(3f) + v1(3f) + v2(3f) + v3(3f) + attribute byte count (H) = 50 bytes
_TRIANGLE = struct.Struct("<12fH")
"""One binary STL triangle: normal (3 floats), 3 vertices (9 floats), attribute byte count."""
_HEADER_SIZE = 80
"""Size of the binary STL header in bytes."""


@dataclass
class StlMesh:
    """A triangle mesh exactly as stored in an STL file."""
    header: bytes
    """The 80-byte header (binary) or 'solid <name>' line (ASCII), kept for writing back."""
    triangles: list[tuple]  # 13-tuples: nx, ny, nz, x1, y1, z1, x2, y2, z2, x3, y3, z3, attr
    """Triangles as 13-tuples: nx, ny, nz, x1, y1, z1, x2, y2, z2, x3, y3, z3, attribute."""

    @property
    def raw_vertex_count(self) -> int:
        """Vertices as Unity's STL importer creates them: 3 per triangle, no sharing."""
        return 3 * len(self.triangles)

    @property
    def welded_vertex_count(self) -> int:
        """Vertices if shared corners are counted once (welded)."""
        return len(unique_vertices(self.triangles))


def unique_vertices(triangles) -> set:
    """Set of distinct vertex positions (x, y, z) in the triangles."""
    seen = set()
    for t in triangles:
        seen.add(t[3:6])
        seen.add(t[6:9])
        seen.add(t[9:12])
    return seen


def _is_binary(data: bytes) -> bool:
    """True if the bytes are a binary STL (the header's triangle count matches the file size)."""
    if len(data) < _HEADER_SIZE + 4:
        return False
    (count,) = struct.unpack_from("<I", data, _HEADER_SIZE)
    return _HEADER_SIZE + 4 + count * _TRIANGLE.size == len(data)


def triangle_count(path: str | Path) -> int:
    """Triangle count without loading the whole mesh (binary); parses ASCII files."""
    path = Path(path)
    size = path.stat().st_size
    with path.open("rb") as handle:
        head = handle.read(_HEADER_SIZE + 4)
    if len(head) == _HEADER_SIZE + 4:
        (count,) = struct.unpack_from("<I", head, _HEADER_SIZE)
        if _HEADER_SIZE + 4 + count * _TRIANGLE.size == size:
            return count
    return len(read_stl(path).triangles)


_FLOAT = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
"""Regex for one floating-point number in an ASCII STL."""
_FACET_RE = re.compile(
    rf"facet\s+normal\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})\s+outer\s+loop"
    rf"\s+vertex\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})"
    rf"\s+vertex\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})"
    rf"\s+vertex\s+({_FLOAT})\s+({_FLOAT})\s+({_FLOAT})",
    re.IGNORECASE,
)
"""Regex for one ASCII STL facet (normal + three vertices)."""


def read_stl(path: str | Path) -> StlMesh:
    """Read a binary or ASCII STL file. Raises PipelineError if it is neither."""
    path = Path(path)
    data = path.read_bytes()
    if _is_binary(data):
        (count,) = struct.unpack_from("<I", data, _HEADER_SIZE)
        body = memoryview(data)[_HEADER_SIZE + 4 : _HEADER_SIZE + 4 + count * _TRIANGLE.size]
        return StlMesh(header=bytes(data[:_HEADER_SIZE]), triangles=list(_TRIANGLE.iter_unpack(body)))

    text = data.decode("ascii", errors="ignore")
    if text.lstrip()[:5].lower() == "solid":
        triangles = [
            tuple(struct.unpack("<12f", struct.pack("<12f", *(float(v) for v in m.groups())))) + (0,)
            for m in _FACET_RE.finditer(text)
        ]
        if triangles or "facet" not in text.lower():
            name = text.lstrip()[5:].split("\n", 1)[0].strip()
            header = ("solid " + name).encode("ascii", errors="ignore")[:_HEADER_SIZE]
            return StlMesh(header=header.ljust(_HEADER_SIZE, b" "), triangles=triangles)
    raise PipelineError(
        f"{path.name} is not a readable STL file (neither valid binary nor ASCII STL).",
        hint="Re-export the mesh from your CAD tool as binary STL.",
    )


def write_stl(mesh: StlMesh, path: str | Path) -> None:
    """Write as binary STL (the format Unity and ROS tools read fastest)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = mesh.header[:_HEADER_SIZE].ljust(_HEADER_SIZE, b"\0")
    if header[:5].lower() == b"solid":
        # A binary file starting with "solid" confuses some readers.
        header = b"binary" + header[6:]
    with path.open("wb") as handle:
        handle.write(header)
        handle.write(struct.pack("<I", len(mesh.triangles)))
        handle.write(b"".join(_TRIANGLE.pack(*t) for t in mesh.triangles))
