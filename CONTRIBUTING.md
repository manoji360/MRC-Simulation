# Contributing Guide

This guide describes how to set up, change, test, review, and release the DAM
Robotics Mars Rover simulation in this repository.

## Code of Conduct

Treat teammates, partners, and other contributors respectfully. Raise ordinary
team concerns with the Project Manager. Safety, harassment, or course-conduct
concerns may be taken directly to the course instruction team.

## Getting Started

### Supported environment

The project targets Ubuntu 24.04 LTS, ROS 2 Jazzy, and Gazebo Harmonic. The
planned implementation uses C++ and Python 3 in a ROS 2 workspace built with
`colcon`. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
using their official instructions before building the project.

Clone the repository:

```bash
git clone https://github.com/manoji360/MRC-Simulation.git
cd MRC-Simulation
```

At the time of writing, this repository does not yet contain ROS packages,
launch files, or a test suite. The following commands are the team's intended
workspace workflow and become runnable once packages have been added under
`src/`.

In a terminal, source ROS 2 and install declared package dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace and source its overlay:

```bash
colcon build --symlink-install
source install/setup.bash
```

If `rosdep` has not been initialized on the machine, follow the ROS 2
installation guide's one-time `rosdep init` step. Do not commit local
environment files, credentials, or machine-specific settings.

## Branching and Workflow

`main` is the shared integration branch. Do not commit directly to `main`.
Track planned work in GitHub Issues, with an outcome, acceptance criteria,
owner, and any blockers. Create one short-lived branch per issue:

```bash
git switch main
git pull --ff-only
git switch -c <type>/<issue-number>-<short-description>
```

Use `feature/`, `fix/`, `test/`, `docs/`, `refactor/`, or `chore/` as the
branch type. Keep each branch focused. Open a pull request (PR) to `main` when
the issue's acceptance criteria are met. Update your branch from `main` before
merging; do not rewrite another contributor's branch.

## Issues and Planning

Every planned change should have a GitHub Issue. Include:

- The problem or outcome and verifiable acceptance criteria
- An owner, dependencies, and blockers
- How the result will be tested or reviewed

Keep issues small enough to produce an accepted result within one sprint. Split
larger efforts into independently reviewable issues. The Project Manager
maintains the team's view of issue status.

## Commit Messages

Use Conventional Commits:

```text
<type>(<optional-scope>): <short description>
```

Examples:

```text
feat(sensor): add simulated GPS publisher
fix(robot): correct wheel joint axis
test(control): cover differential drive limits
docs(setup): describe the ROS workspace
```

Reference the issue when applicable (for example, `Closes #42`). Keep each
commit focused on one logical change.

## Code Style, Linting, and Formatting

Follow ROS 2 Jazzy conventions and the formatter/linter configuration committed
with the package. No formatter, linter configuration, or automated lint gate
exists in this repository yet. The Quality Owner will select and configure the
ROS 2 ament lint checks in Sprint 2; until then, do not claim that formatting or
lint has been automatically verified.

Once ROS packages and their lint configuration are present, run the configured
checks with:

```bash
source /opt/ros/jazzy/setup.bash
colcon test
colcon test-result --verbose
```

The team will record any separate formatter command and configuration path here
when the Sprint 2 lint setup is added.

## Testing

Every behavioral change must add or update an automated test where practical.
A bug fix should include a regression test. If an automated test is not
practical, explain why in the PR and provide repeatable manual verification
steps and evidence.

Run the workspace tests after building:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Depending on the change, tests may cover calculations and message processing,
ROS node startup and configuration, topics/services/actions/TF, or a Gazebo
simulation smoke test. There are no project tests yet; the initial test suite
and CI test job are planned for Sprint 2.

## Definition of Done

A change is ready to merge only when all applicable requirements below are
satisfied. A check described as planned is not an existing gate: until it is
implemented, the team must verify the requirement manually and must not claim
that CI enforces it.

| Requirement | Evidence or enforcement | Status and owner |
| --- | --- | --- |
| Issue acceptance criteria are met | Issue and PR description; reviewer verifies the result | Team policy; manual |
| Workspace builds on Ubuntu 24.04 with ROS 2 Jazzy | `colcon build --symlink-install`; planned CI build job in `.github/workflows/` | Planned Sprint 2; Quality Owner Michael Iu |
| Relevant tests pass | `colcon test --event-handlers console_direct+` and `colcon test-result --verbose`; planned CI test job | Planned Sprint 2; Quality Owner Michael Iu |
| Formatting and lint checks pass | ROS 2 ament lint checks run through `colcon test`; exact configuration/formatter to be recorded here | Planned Sprint 2; Quality Owner Michael Iu |
| Documentation and interfaces are updated | Reviewer checks setup, launch, ROS interfaces, parameters, and TF documentation affected by the change | Team policy; manual |
| No credentials or restricted data are committed | Author and reviewer inspect the diff; planned secret-scanning CI job | Planned Sprint 2; Quality Owner Michael Iu |
| At least one teammate approves the PR | One approval from a reviewer other than the author; planned `main` branch protection | Manual now; branch protection planned Sprint 2 by Quality Owner Michael Iu |
| Required checks pass before merge | PR checks; planned required-status-check configuration | Planned Sprint 2; Quality Owner Michael Iu |
| Author understands and can explain submitted code, including AI-assisted code | Author answers reviewer questions and verifies generated suggestions | Team policy; applies to every PR |
| Change leaves the project in a demonstrable state | Relevant launch/smoke test; initial Gazebo smoke test planned with CI | Planned Sprint 2; Quality Owner Michael Iu |

If a requirement does not apply, say why in the PR. The Quality Owner maintains
this Definition of Done and calls out missing or failing gates.

## Contribution Norm

Every student owns at least one piece of accepted work per sprint. Track every
piece of work in a GitHub Issue. Code, tests, and repository documentation live
in this repository; link team research or design documents from their issue.
If work is tracked or accepted in a partner system, record its reference and
acceptance evidence in the GitHub Issue and the team's course Sprint Notes.
Record sprint contributions in those Sprint Notes.

Accepted work means:

- Code or tests are merged through an approved PR and meet the Definition of
  Done.
- Documentation is reviewed, published in its agreed location, and meets its
  issue's acceptance criteria.
- Research or design work is reviewed by the team and results in an accepted
  document, experiment, or recorded decision linked from its issue.
- Partner-system work is accepted by the responsible reviewer or partner, with
  objective evidence linked from its issue.

The team makes this norm workable by sizing issues for one sprint, assigning an
owner and acceptance criteria before work starts, and allowing pairing while
recording each student's contribution. A ready PR should receive a review
response within two weekdays. If work is blocked or at risk, raise it in the
team Discord before the sprint ends so it can be resized, paired, or
reassigned. The Project Manager records accepted work in the Sprint Notes.

## Pull Requests and Reviews

Every change to `main` must use a PR. A PR should:

- Link its GitHub Issue and summarize the problem and resulting behavior
- State how it was tested, or explain why automated testing was impractical
- Include useful logs, screenshots, or simulation evidence
- Identify affected ROS topics, services, actions, messages, parameters, TF
  frames, launch files, or dependencies
- Update documentation when setup or behavior changes
- Confirm the applicable Definition of Done checks

Keep PRs focused and small enough to review. Reviewers should respond within
two weekdays, verify the acceptance criteria, inspect correctness, tests,
interfaces, and documentation, and ask the author to explain any unclear or
AI-assisted code. The author may not approve their own PR. One teammate
approval and passing required checks are required before merge. Use squash
merge. Do not merge while a required check is failing or a requested change is
unresolved.

## CI and Releases

CI workflows will live in `.github/workflows/`. The initial build, test, lint,
secret-scan, and Gazebo smoke-test gates are planned for Sprint 2 and owned by
Quality Owner Michael Iu. Until those workflows and branch protections exist,
the PR approval rule remains a team commitment rather than a server-enforced
setting.

The release and packaging process has not yet been established. Do not publish
or tag a release until the Quality Owner has confirmed the release checks and
the team has documented the process here.

## Security and AI-Assisted Work

Never commit passwords, API keys, access tokens, private keys, personal data,
or partner-supplied/restricted material. If a credential is exposed, notify
the Quality Owner and Project Manager promptly so it can be revoked or
rotated; do not repeat the credential in an issue or chat.

The team has no NDA, but that does not make all project information safe to
share. Claude and Codex may be used only with public, non-sensitive project
code and documentation. Never enter credentials, personal data,
partner-supplied data, private meeting content, or anything marked restricted
into an AI tool. When uncertain, ask the AI Coordinator before sharing. See
[AGENTS.md](AGENTS.md) for the complete AI-tool instructions.

## Support and Contact

Use the Capstone channel in the DAM Robotics team Discord for contributor
questions once that channel is available. Tag the relevant owner:

- Project Manager Manoji Kumaar Vijaya Kumar: planning, issue priority, or
  partner-facing coordination
- AI Coordinator Austin Hu: AI tools and confidentiality
- Quality Owner Michael Iu: builds, tests, CI, and releases
- Software lead Nolan Kessler: technical project guidance through the DAM
  Robotics Discord

An urgent blocker is an issue stopping agreed sprint work or putting a
demo/deadline at risk, such as a broken build or unavailable required access.
For one, tag the relevant owner; also tag Manoji when timing or scope is at
risk. A ready PR should receive a review response within two weekdays.
