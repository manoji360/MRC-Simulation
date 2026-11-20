"""Documentation coverage checker: every function, class, field, property and constant must have a description.

Why: hovering over any name in VS Code (Pylance / C# Dev Kit) shows its description only if it has one.

  Python  - module docstring; docstring on every class, function and method; a docstring line right after
            every module-level constant (NAME = ...) and every dataclass / class-level field, which Pylance
            shows on hover ("attribute docstring").
  C#      - a /// <summary> comment on every class, struct, enum, constructor, method, property, field,
            constant and event. (Local variables and local functions cannot carry hover documentation in C#;
            they are named for what they hold and commented inline.)

Usage:  python tools/doc_coverage.py            (lists everything missing; exit code 1 if anything is)
        python tools/doc_coverage.py --summary  (counts per file)
"""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
"""Repository root."""
PY_ROOTS = [REPO / "urdf-pipeline" / "urdf_pipeline", REPO / "urdf-pipeline" / "tests",
            REPO / "ros2" / "src" / "mrc_waypoint_demo", REPO / "tools", REPO / "tests"]
"""Python folders checked."""
CS_ROOTS = [REPO / "unity" / "MRC-Rover" / "Assets"]
"""C# folders checked."""


def _has_attr_doc(body: list, index: int) -> bool:
    """True if body[index] is followed by a bare string expression (an attribute docstring)."""
    nxt = body[index + 1] if index + 1 < len(body) else None
    return isinstance(nxt, ast.Expr) and isinstance(nxt.value, ast.Constant) and isinstance(nxt.value.value, str)


def python_missing(path: Path) -> list[str]:
    """Undocumented Python symbols in one file, as 'path:line kind name'."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    rel = path.relative_to(REPO).as_posix()
    missing = []
    if path.name != "__init__.py" or path.read_text(encoding="utf-8").strip():
        if not ast.get_docstring(tree):
            missing.append(f"{rel}:1 module")

    def visit(body: list, owner: str, in_class: bool) -> None:
        for i, node in enumerate(body):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                kind = "class" if isinstance(node, ast.ClassDef) else ("method" if in_class else "function")
                if not ast.get_docstring(node):
                    missing.append(f"{rel}:{node.lineno} {kind} {owner}{node.name}")
                if isinstance(node, ast.ClassDef):
                    visit(node.body, f"{owner}{node.name}.", True)
            elif isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                names = [t.id for t in targets if isinstance(t, ast.Name)]
                if not names or names[0] in ("__all__", "__version__"):
                    continue
                # module level: UPPER_CASE constants; class level: every field
                public_alias = not in_class and names[0][:1].isupper()   # UPPER_CASE constants and CamelCase type aliases
                if (in_class or public_alias or names[0].lstrip("_").isupper()) and not _has_attr_doc(body, i):
                    missing.append(f"{rel}:{node.lineno} {'field' if in_class else 'constant'} {owner}{names[0]}")

    visit(tree.body, "", False)
    return missing


CS_MEMBER = re.compile(
    r"^\s*(?:\[[^\]]*\]\s*)*"
    r"(?:(?:public|private|protected|internal|static|readonly|const|override|virtual|abstract|sealed|partial|new|event)\s+)+"
    r"(?!return\b|throw\b|var\b)[\w<>\[\],.?() ]+?\b(\w+)\s*(?:\(|=|;|\{|=>)")
"""Regex for a C# member declaration with modifiers (method, property, field, constant, event)."""
CS_TYPE = re.compile(r"^\s*(?:\[[^\]]*\]\s*)*(?:(?:public|private|protected|internal|static|sealed|abstract|partial)\s+)*"
                     r"(class|struct|enum|interface)\s+(\w+)")
"""Regex for a C# type declaration (class, struct, enum, interface)."""
CS_CTOR = re.compile(r"^\s*(?:(?:public|private|protected|internal|static)\s+)+(\w+)\s*\(")
"""Regex for a C# constructor (modifiers directly followed by Name( - no return type)."""
CS_MULTI_FIELD = re.compile(r"^\s*(?:(?:public|private|protected|internal|readonly|static)\s+)*[\w?.\[\]<>]+\s+\w+(?:\s*=[^,;()]+)?\s*,\s*\w+.*;")
"""Regex for a C# field declaration naming several fields (int a, b;) - each needs its own description."""
CS_METHOD_NOMOD = re.compile(r"^\s*(?!(?:return|new|if|for|foreach|while|switch|lock|using|catch|yield|throw|else|await)\b)"
                             r"(?:[\w<>\[\],.?()]+\s+)+(\w+)\s*\([^;]*\)?\s*(?:\{|=>|$)")
"""Regex for a C# method without an access modifier (private by default, e.g. Unity's void Update())."""
CS_FIELD_NOMOD = re.compile(r"^\s{8}(?:[\w<>\[\],.?]+\s+)+(\w+)\s*(?:=[^=>]|;)")   # private fields without a modifier
"""Regex for a C# field without an access modifier (private by default)."""


def csharp_missing(path: Path) -> list[str]:
    """Undocumented C# members in one file (a member counts as documented if /// lines precede it)."""
    lines = path.read_text(encoding="utf-8").splitlines()
    rel = path.relative_to(REPO).as_posix()
    missing = []
    depth = 0                     # brace depth: members live at depth 1 (namespace) or 2 (type) or 3 (nested type)
    type_depths: list[int] = []   # depths at which type bodies start
    pending_type = False          # a type declaration whose "{" is on the next line
    for n, line in enumerate(lines):
        stripped = line.strip()
        code = re.sub(r'"(?:\\.|[^"\\])*"', '""', stripped.split("//")[0])
        in_type_body = bool(type_depths) and depth == type_depths[-1]
        documented = n > 0 and lines[n - 1].strip().startswith(("///", "[")) and _doc_above(lines, n)
        type_match = CS_TYPE.match(line)
        if type_match and not stripped.startswith("//"):
            if not documented:
                missing.append(f"{rel}:{n + 1} {type_match.group(1)} {type_match.group(2)}")
        elif in_type_body and stripped and not stripped.startswith(("//", "[", "#", "}", "{")):
            member = CS_CTOR.match(line) or CS_MEMBER.match(line) or CS_METHOD_NOMOD.match(line) or (CS_FIELD_NOMOD.match(line) if not stripped.startswith(("return", "var ", "if", "for", "while", "yield", "using")) else None)
            if member and not documented:
                missing.append(f"{rel}:{n + 1} member {member.group(1)}")
            if CS_MULTI_FIELD.match(line):
                missing.append(f"{rel}:{n + 1} split: one field per line, so each name can have its own description")
        opens, closes = code.count("{"), code.count("}")
        if type_match and "{" in code:
            type_depths.append(depth + 1)
        elif type_match:
            pending_type = True
        elif pending_type and code.startswith("{"):
            type_depths.append(depth + 1)
            pending_type = False
        depth += opens - closes
        while type_depths and depth < type_depths[-1]:
            type_depths.pop()
    return missing



def _doc_above(lines: list[str], n: int) -> bool:
    """Walk up over attribute lines; true if a /// comment is directly above the declaration."""
    i = n - 1
    while i >= 0 and lines[i].strip().startswith("["):
        i -= 1
    return i >= 0 and lines[i].strip().startswith("///")


def main() -> int:
    """Print every undocumented symbol; return 1 if any."""
    results: dict[str, list[str]] = {}
    for root in PY_ROOTS:
        for path in sorted(root.rglob("*.py")) if root.exists() else []:
            if "__pycache__" in path.parts:
                continue
            results[path.relative_to(REPO).as_posix()] = python_missing(path)
    for root in CS_ROOTS:
        for path in sorted(root.rglob("*.cs")):
            results[path.relative_to(REPO).as_posix()] = csharp_missing(path)
    total = sum(len(v) for v in results.values())
    if "--summary" in sys.argv:
        for file, items in results.items():
            if items:
                print(f"{len(items):4d}  {file}")
    else:
        for items in results.values():
            for item in items:
                print(item)
    print(f"\n{total} undocumented symbol(s) in {sum(1 for v in results.values() if v)} file(s).")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
