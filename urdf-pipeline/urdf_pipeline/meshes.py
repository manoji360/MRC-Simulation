"""Finding, resolving and laying out the mesh files a URDF references.

Output layout: a mesh is copied to the location its URI resolves to *relative to the
output .urdf*. `package://pkg/meshes/a.stl` -> `<out>/pkg/meshes/a.stl`, which is how the
Unity URDF Importer resolves package:// paths (folders next to the .urdf file).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from urllib.parse import unquote, urlparse

from .model import Robot

PACKAGE = "package://"


@dataclass(frozen=True)
class MeshUse:
    link: str
    kind: str  # "visual" or "collision"
    holder: ET.Element  # the <visual>/<collision> element
    uri: str


def mesh_uses(robot: Robot) -> list[MeshUse]:
    uses = []
    for link in robot.links.values():
        for holder in link.element:
            if holder.tag in ("visual", "collision"):
                mesh = holder.find("geometry/mesh")
                if mesh is not None and mesh.get("filename"):
                    uses.append(MeshUse(link.name, holder.tag, holder, mesh.get("filename")))
    return uses


def is_absolute(uri: str) -> bool:
    return (
        uri.startswith("file://")
        or uri.startswith("/")
        or PureWindowsPath(uri).is_absolute()
    )


def output_key(uri: str) -> str | None:
    """Relative path (posix) where this mesh goes inside the output folder.

    None when the URI can't be placed portably (absolute path, '..', other schemes).
    """
    if uri.startswith(PACKAGE):
        rel = uri[len(PACKAGE):]
    elif "://" in uri or is_absolute(uri):
        return None
    else:
        rel = uri
    parts = PurePosixPath(rel.replace("\\", "/")).parts
    if not parts or ".." in parts or parts[0] == "/":
        return None
    return PurePosixPath(*parts).as_posix()


def _file_uri_path(uri: str) -> Path:
    path = unquote(urlparse(uri).path)
    # file:///C:/x -> "/C:/x" on Windows
    if len(path) > 2 and path[0] == "/" and path[2] == ":":
        path = path[1:]
    return Path(path)


def resolve_mesh(uri: str, urdf_dir: Path, mesh_roots: list[Path]) -> Path | None:
    """Find the file a mesh URI points to, or None."""
    if uri.startswith(PACKAGE):
        rel = uri[len(PACKAGE):]
        without_pkg = rel.split("/", 1)[1] if "/" in rel else rel
        for root in [urdf_dir, *mesh_roots, urdf_dir.parent]:
            # <root>/<pkg>/<path>, or a root that *is* the package folder: <root>/<path>
            for candidate in (root / rel, root / without_pkg):
                if candidate.is_file():
                    return candidate
        return None
    if uri.startswith("file://"):
        path = _file_uri_path(uri)
    elif "://" in uri:
        return None
    else:
        path = Path(uri)
        if not path.is_absolute():
            path = urdf_dir / path
    return path if path.is_file() else None


_IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".tga", ".bmp", ".tif", ".tiff", ".dds", ".gif")
_MTL_MAP_KEYS = ("map_", "bump", "disp", "decal", "refl", "norm")


def _safe_relative(ref: str) -> str | None:
    ref = unquote(ref.strip())
    if ref.startswith("file://"):
        ref = ref[len("file://"):]
    if not ref or is_absolute(ref) or "://" in ref:
        return None
    parts = PurePosixPath(ref.replace("\\", "/")).parts
    if ".." in parts:
        return None
    return PurePosixPath(*parts).as_posix()


def companion_files(mesh_path: Path) -> list[str]:
    """Files a mesh needs next to it (relative to the mesh's folder): textures referenced by
    a COLLADA (.dae) file, or the material library and texture maps of an OBJ file."""
    suffix = mesh_path.suffix.lower()
    refs: list[str] = []
    try:
        if suffix == ".dae":
            for element in ET.parse(mesh_path).getroot().iter():
                if isinstance(element.tag, str) and element.tag.endswith("init_from") and element.text:
                    if element.text.strip().lower().endswith(_IMAGE_EXTS):
                        refs.append(element.text)
        elif suffix == ".obj":
            for line in mesh_path.read_text(encoding="utf-8", errors="ignore").splitlines():
                if line.startswith("mtllib "):
                    mtl = line[len("mtllib "):].strip()
                    refs.append(mtl)
                    mtl_path = mesh_path.parent / mtl
                    if mtl_path.is_file():
                        for mline in mtl_path.read_text(encoding="utf-8", errors="ignore").splitlines():
                            tokens = mline.strip().split()
                            if tokens and tokens[0].lower().startswith(_MTL_MAP_KEYS):
                                # texture paths are relative to the .mtl file
                                refs.append(str(PurePosixPath(mtl.replace("\\", "/")).parent / tokens[-1]))
    except (ET.ParseError, OSError):
        return []
    result = []
    for ref in refs:
        rel = _safe_relative(ref)
        if rel and (mesh_path.parent / rel).is_file() and rel not in result:
            result.append(rel)
    return result


def chunk_uri(uri: str, index: int) -> str:
    base, dot, ext = uri.rpartition(".")
    if not dot:
        return f"{uri}_chunk{index:02d}"
    return f"{base}_chunk{index:02d}.{ext}"
