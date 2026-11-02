# Contributing Guide

This repository is currently in the planning and setup phase. The project is not yet implementing a full rover system, and the work tracked here is focused on documenting the project direction, requirements, design decisions, risks, and contribution process.

## Project direction

The team has chosen a Unity-first approach for the prototype and simulation work. This means the repo may eventually contain Unity scenes, scripts, and tooling, but there is currently no active simulation implementation. Gazebo and ROS are intentionally not the lead runtime choice at this checkpoint.

## Getting started

Clone the repository:

```bash
git clone https://github.com/manoji360/MRC-Simulation.git
cd MRC-Simulation
```

Set up the local Python toolchain used for repository checks:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .[dev]
pytest -q
```

At this stage, the project is documentation-led and does not yet include a production runtime or game build.

## Branching and workflow

`main` is the default integration branch. Do not commit directly to `main`.

Use a short-lived branch for each change:

```bash
git switch main
git pull --ff-only
git switch -c <type>/<issue-number>-<short-description>
```

Suggested branch prefixes:

- `feature/`
- `docs/`
- `fix/`
- `refactor/`
- `chore/`

Open a pull request to `main` when the work is ready for review and update the branch from `main` before merge.

## Planning and issue tracking

Every change should be associated with a GitHub Issue. Include:

- the problem being solved or the decision being made
- acceptance criteria or review criteria
- the owner and any blockers
- how the change will be verified

Keep work small enough to review in one iteration.

## Commit messages

Use Conventional Commits:

```text
<type>(<optional-scope>): <short description>
```

Examples:

```text
docs(requirements): draft startup requirements
feat(unity): add prototype scene plan
docs(risk): update project risks
chore(ci): add repository health checks
```

Reference the relevant issue when applicable.

## Testing and validation

This repository currently validates the planning and documentation scaffolding rather than a production simulation stack.

Run the repository checks locally:

```bash
pytest -q
```

The CI workflow runs the same checks on pull requests and on pushes to `main`.

## Definition of done

A change is ready to merge when:

- the issue or task is completed and reviewed
- the relevant documentation reflects the current state accurately
- the test suite still passes
- no secrets or restricted materials are committed
- at least one teammate has reviewed the change

This is intentionally a lightweight definition for the early-stage repository. Product-level simulation testing will be added when the project moves beyond the planning phase.

## Contribution expectations

All team members should contribute to the repository through clear, reviewable updates. The repository should stay consistent with the current project status: planning-oriented, well-documented, and easy for a new teammate to understand.
