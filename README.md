# MRC Simulation

This repository is in the project-start phase. At this point, there is no production simulation implementation yet; the repository is being used to document the initial planning decisions, requirements, risks, and contribution workflow for a Unity-first Mars rover simulation. The project is not yet implemented and should not be described as a working runtime.

## Project direction

The team has decided to prioritize Unity for the prototype and visual simulation work rather than Gazebo for the initial implementation. The current repo is therefore a planning and documentation scaffold, not a running rover simulation.

## Quickstart

```bash
git clone https://github.com/manoji360/MRC-Simulation.git
cd MRC-Simulation
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .[dev]
pytest -q
```

This does not launch a simulator yet. It validates that the repository’s required planning documents and safety checks are present and consistent.

## Repository structure

- `docs/requirements.md` — problem statement, requirements, constraints, and success metrics
- `docs/design.md` — system design for the early-phase Unity-oriented project
- `docs/risks.md` — risk register and external prerequisites
- `CONTRIBUTING.md` — contribution workflow and review expectations
- `AGENTS.md` and `CLAUDE.md` — AI assistant guidance
- `.github/workflows/ci.yml` — automated repository health check on pull requests and pushes to `main`
- `tests/` — lightweight verification for required project documentation and repo configuration

## Current project status

The project has not started implementation work yet. The repo is intentionally minimal and reflects the beginning of the design phase, with emphasis on a Unity-friendly architecture, defined constraints, and a clean contribution process.

## Planned target stack

- Unity 2022 LTS or newer for simulation, interaction, and prototype development
- Windows/macOS/Linux developer workstations depending on team tooling
- A future production pipeline may include a lightweight build/test workflow as the project matures

No Gazebo runtime, ROS 2 workspace, or production game/simulation code is expected in this checkpoint.
