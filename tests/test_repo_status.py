from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_required_docs_exist():
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
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "Unity" in readme
    assert "Gazebo" in readme or "Gazebo" in (ROOT / "docs" / "design.md").read_text(encoding="utf-8")


def test_repository_is_not_claiming_runtime_implementation():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    design = (ROOT / "docs" / "design.md").read_text(encoding="utf-8")

    assert "not yet" in readme.lower()
    assert "startup-phase" in design.lower() or "planning" in design.lower()
