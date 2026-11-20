"""Repository checks: the required planning documents exist and stay consistent with the project direction."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
"""Repository root."""


def test_required_docs_exist():
    """README, CONTRIBUTING, AGENTS, CLAUDE and the docs/ requirements, design and risks files exist."""
    required = [
        ROOT / "README.md",
        ROOT / "CONTRIBUTING.md",
        ROOT / "AGENTS.md",
        ROOT / "CLAUDE.md",
        ROOT / "docs" / "requirements.md",
        ROOT / "docs" / "design.md",
        ROOT / "docs" / "risks.md",
    ]

    for path in required:
        assert path.exists(), f"Missing required project document: {path.name}"


def test_repo_direction_mentions_unity():
    """The README names Unity as the direction (and Gazebo is mentioned as the alternative)."""
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "Unity" in readme
    assert "Gazebo" in readme or "Gazebo" in (ROOT / "docs" / "design.md").read_text(encoding="utf-8")


def test_repository_is_not_claiming_runtime_implementation():
    """The docs do not claim a finished simulation (startup / planning phase)."""
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    design = (ROOT / "docs" / "design.md").read_text(encoding="utf-8")

    assert "not yet" in readme.lower()
    assert "startup-phase" in design.lower() or "planning" in design.lower()
