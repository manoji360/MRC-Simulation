# AI Assistant Guidance

## Project Context

This repository is for the DAM Robotics Mars Rover simulation. The team's
target environment is Ubuntu 24.04 LTS, ROS 2 Jazzy, and Gazebo Harmonic, with
C++ and Python 3 in a `colcon` workspace.

At the time this file was written, the repository has no ROS packages,
simulation launch files, tests, formatter/linter configuration, or CI
workflow. Do not invent project APIs, package names, file paths, or existing
checks. Once implementation begins, inspect the repository and follow its
actual structure and configuration.

Read [CONTRIBUTING.md](CONTRIBUTING.md) for setup, local commands, review,
Definition of Done, and contribution expectations. Keep that guide and this
file consistent.

## Conventions and Planned Context

- Prefer ROS 2 Jazzy conventions and the repository's committed configuration.
- Use the documented branch, issue, and pull-request workflow; do not suggest
  direct commits to `main`.
- Do not describe planned Sprint 2 CI gates as if they already exist.
- When code is added, Quality Owner Michael Iu adds the actual build, test,
  lint, and simulator commands/configuration to `CONTRIBUTING.md` and updates
  this file with verified project conventions.
- When architecture or ROS interfaces are established, document their source
  of truth and relevant constraints here; Project Manager Manoji Kumaar
  Vijaya Kumar ensures partner and project constraints are accurately
  represented.
- AI Coordinator Austin Hu owns this file and updates the AI-tool guidance as
  the team's approved practices change.

## Confidentiality and AI Tool Use

The team reports that it has no NDA. This is not blanket permission to disclose
project information. Claude and Codex may be used only with public,
non-sensitive project code and documentation.

Never submit any of the following to an AI tool:

- Passwords, API keys, access tokens, private keys, or other credentials
- Personal data
- Partner-supplied data or material marked restricted
- Private meeting content, including private notes or transcripts

Do not put secrets or restricted information in source code, prompts, logs, or
repository documentation. If unsure whether material is safe to share, stop
and ask AI Coordinator Austin Hu before entering it into any tool.

Treat AI-generated output as unverified. Check it against the project,
understand it, run the relevant tests, and take responsibility for it before
proposing it for merge.
