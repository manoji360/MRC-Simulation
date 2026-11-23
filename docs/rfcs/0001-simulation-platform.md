# RFC 0001: Platform Selection for the Mars Rover Simulation Environment

**Status: Accepted (conditional).** Unity 6.3 LTS is the working platform for the prototype. Two verification gates stay open: an automated ROS 2 round trip and an independent setup reproduction by a second teammate. This revision replaces the October 25, 2026 draft, which provisionally recommended Gazebo Harmonic.

| Field | Value |
| --- | --- |
| Project | Mars Rover Simulation (Oregon State University CS Capstone) |
| First draft | October 25, 2026 |
| This revision | November 22, 2026 |
| Decision authority | Project lead, with review by the three-person project team |
| Author | Manoji Kumaar |
| Decision record | `docs/rfcs/0001-simulation-platform.md` |

## Problem and Context

Our team must choose the engine that every simulator component will be built on. Three of us are building a simulation environment where university rover teams can test autonomy software before field trials. The setting is the University Rover Challenge (URC) desert site near Hanksville, Utah. We build the environment: terrain, a drivable rover, sensors, and ROS 2 command and feedback topics. ROS 2 (Robot Operating System 2) is the messaging framework rover software uses to exchange commands and sensor data. We do not build the navigation algorithms.

The first user is the Oregon State University Robotics Club. Its rover is described in a URDF, the standard robot description file listing parts (links), the joints between them, masses and 3D meshes. The club's URDF has 55 links, 54 joints and 34 meshes, and comes from its public `Rover_2026_2027` repository.

The decision touches every layer above the robot description. The scene, rover rig, physics tuning, sensor models and automated tests are written against one engine's API. Switching engines later means rewriting all of them. Two layers carry over to any engine: the ROS 2 nodes, which speak standard messages, and our URDF preparation pipeline, which is written in Python. We treat the decision as hard to reverse once sensors and full terrain exist, and only moderately costly to reverse today.

The decision was needed in early November, before substantial implementation. The deadline is a working simulator by the end of winter term 2027. The ROS distribution is ROS 2 Jazzy on Ubuntu 24.04 under WSL2, and it was installed and verified on November 19, 2026.

Two facts shaped the first draft. First, an earlier Unity attempt by a previous team reportedly fell back to a hand-built rover "because of import-size restrictions." Second, nobody had tested Unity with Jazzy. Neither fact was supported by evidence at the time, so the October draft leaned toward Gazebo Harmonic, the documented Jazzy pairing.

If the team does nothing, no simulator exists and the compatibility risks stay hidden until winter term. Doing nothing also compresses the build and handoff period that algorithm teams depend on.

## Options

### Option A: Unity 6.3 LTS with ROS-TCP-Connector

Unity hosts the world and runs PhysX articulation physics, a solver for chains of jointed bodies. Unity's URDF Importer (v0.5.2) brings in the rover. ROS-TCP-Connector (v0.7.0) talks over TCP to a ROS 2 node, `ros_tcp_endpoint`, that runs in the Jazzy workspace. Rover assets pass through our Python pipeline first, which reshapes the URDF to fit Unity's limits.

- **Costs:** Learning the editor. A TCP bridge process between the engine and ROS rather than native ROS integration. Maintaining the pipeline. Unity's robotics packages have had few releases, so fixes may fall to us.
- **Right choice when:** the team needs fast scene iteration and good rendering for cameras, and the TCP bridge holds up under automated testing.

### Option B: Gazebo Harmonic

Gazebo Harmonic is the simulator version documented for ROS 2 Jazzy. It loads the URDF through `ros_gz` and exposes topics natively.

- **Costs:** Learning SDF and world files. Building terrain from scratch. Proving that WSL2 graphics run Gazebo at an acceptable rate. Unlike Unity, it has no project prototype, so its costs are estimates.
- **Right choice when:** the Unity ROS bridge fails, or algorithm teams need `ros2_control` and Nav2 to run exactly as they do on the rover. The club's URDF already contains a `<gazebo>` block (for example, a 100 Hz IMU plugin), which lowers Gazebo's sensor setup cost.

### Option C: Do nothing and defer

Keep the verified Jazzy setup and select no engine until assets and user requirements are confirmed.

- **Costs:** No simulator for the winter deadline. Engine risks surface late.
- **Right choice when:** the rover description or the user requirements are about to change enough to invalidate any engine work. Neither is true now: the URDF is pinned to commit `1e79992`, and the topic interface is documented.

## Analysis

This comparison is uneven. Unity was prototyped between November 8 and 22 and has measured results. Gazebo was not built, so the claims about it come from its documentation. All figures below come from files under `unity/MRC-Rover/FeasibilityResults/` and the commits named in the References.

### The import restriction now has a known cause

The first draft treated the predecessor's "import-size restriction" as unexplained. Inspecting the club's rover found two concrete Unity limits:

- **Mesh vertex limit:** a Unity mesh with 16-bit indices holds 65,535 vertices. Nine of the rover's 34 meshes exceed it.
- **Articulation link limit:** a PhysX articulation tree holds 64 links. The rover has 55, or 30 after removing frame-only links.

The pipeline (`urdf-pipeline`) solves both limits without editing the source files. It splits heavy meshes into chunks, can collapse frame-only links, and records every change in a manifest. Rebuilding the original URDF from that output is a self-check. We cannot prove that these limits are what stopped the previous team, but they would produce the reported symptom.

The pipeline also found defects that would affect either engine. Of the URDF's 26 inertia tensors, 15 are physically impossible: larger than if all of a link's mass sat at its outer edge. All 26 were replaced with values computed from the collision meshes. Gazebo would have read the same faulty data.

### Performance and fidelity

With the tuned rover (55 bodies, 125 colliders), Unity imports the model in 13.8 s with zero errors. The physics benchmark runs 1,000 steps at 100 Hz in 0.20 ms per step, 50.6 times faster than real time. That figure excludes rendering and sensors, and the evaluation computer was not recorded, so it is not yet the ten-minute workload test this RFC requires.

| Check (tuned rover, 1 m/s unless noted) | Result |
| --- | --- |
| Imported link poses vs URDF | 55 links, 0.0 mm and 0.0° error |
| Total mass vs URDF | 50.642 kg, exactly the expected value (URDF 50.613 kg plus pipeline placeholder masses) |
| Drive straight, 0.5 m/s for 10 s | 5.00 m (4.94 m expected), 0.21° heading drift |
| Acceleration / braking | 2.09 / 2.16 m/s² measured vs 2.0 m/s² limit |
| Ramps 10°, 20°, 30° | Crossed, crossed, **flipped** |
| Steps 5, 10, 15 cm; rubble; rough ground | All crossed |
| Utah desert, 90 s | 89 m driven, 4.2 m elevation change, no instability |

The 30° ramp flip is a model finding, not an engine fault. The URDF has no suspension springs, so the bogies (the pivoting wheel carriers) swing freely and the body over-pitches. Many inputs are still assumptions, including tyre friction (±40%) and the 30 N·m wheel torque cap. These are listed in `docs/assumed-data.md`. Neither engine can claim validated tyre–soil behaviour, and the desert terrain is a stylised procedural model, not survey data.

### Complexity and maintainability

Gazebo's main advantage remains its documented Jazzy pairing and native topics. Unity requires a bridge, and that bridge is the weakest point of the prototype. The only recorded automated ROS run (November 17) failed: Unity published 600 odometry messages but never connected to the endpoint and received no commands. The in-editor demo, the Makefile targets and 12 ROS-side unit tests exist, but a headless run that passes has not yet been recorded.

On the other side, Unity's scene is generated from code, nothing generated is committed, and PhysX enhanced determinism makes test runs repeatable. Every script carries documentation, enforced by `tools/doc_coverage.py`. A Gazebo build would start these practices from zero.

### Security and cost

The bridge listens on TCP port 10000 on `127.0.0.1` only. Nothing is exposed beyond the machine; remote use would need its own review. Dependencies are pinned to git tags (URDF Importer v0.5.2, ROS-TCP-Connector v0.7.0, endpoint branch `main-ros2`).

Gazebo is free and open source. Unity's cost depends on licence eligibility, which is still unconfirmed (see Constraints). The larger cost is team time: the Unity prototype spans 6 commits from November 8 to 19, plus uncommitted work through November 22, though hours were not logged. A switch now would discard the scene, rig and tests, but keep the pipeline and the ROS nodes.

## Recommendation and Verification Plan

Adopt Unity (Option A) as the platform, on the condition that gates 4 and 6 below pass by the end of week 2 of winter term. The analysis supports Unity because its main risk, rover import, was the one the October draft could not assess, and it is now resolved with evidence. Gazebo's remaining advantage, the documented Jazzy pairing, reduces to one testable question: does the TCP bridge work reliably? That does not justify abandoning working physics, terrain and tests.

Gazebo stays the named fallback. If the bridge cannot pass gate 4 after the team diagnoses it, we build gates 1–4 in Gazebo Harmonic before adding sensors to Unity. The URDF pipeline and the ROS nodes would carry over.

The seven gates from the first draft still apply. Their status on November 22, 2026:

| # | Gate | Pass condition | Status | Evidence |
| --- | --- | --- | --- | --- |
| 1 | Setup | Second teammate launches the baseline from written instructions alone | Open | Instructions in `unity/README.md`, `ros2/README.md`; not yet reproduced |
| 2 | Environment | Ground, incline and obstacle load with verified scale and collisions | Passed | Test course and desert generated from code; 8 lanes driven |
| 3 | Rover | Links, joints, dimensions and collisions match the source URDF | Passed, with documented simplifications | 0.0 mm pose error over 55 links; inertia and locked four-bar joints listed in `docs/assumed-data.md` |
| 4 | Commands and feedback | Scripted `/cmd_vel` moves the rover, stop stops it, `/odom` uses agreed types, frames and timestamps | Open, failing | Nov 17 headless run never connected |
| 5 | Failure detection | A removed or misconfigured stream is reported, not passed | Partial | Unity stops after 0.5 s without `/cmd_vel`; the test reports a missing connection as a failure |
| 6 | Performance | Agreed scene and sensors run 10 minutes at real time on a recorded computer | Open | Physics-only benchmark 50.6× real time; no sensors yet; computer not recorded |
| 7 | Maintainability | Teammate edits one obstacle and reruns tests from repository instructions | Open | Obstacles are defined in code (`TestCourse.cs`); not yet exercised |

Gates 2 and 3 are verified by automated PlayMode tests. The latest regression run (15 tests) and arm-pose run (3 tests) passed on November 22. One `FrictionAndTorque` failure appeared in a November 17 run; it must be rechecked as part of the gate 4 work.

Across the whole project, the decision has worked if algorithm teams can drive the simulated rover through ROS 2 topics alone, and if a teammate other than the lead can extend the scene. This simulator verifies the environment and its interfaces, not the quality of any team's algorithm.

## Constraints

Three limits apply to this decision. None of them has completed legal review.

| Constraint | What we know | What it changes |
| --- | --- | --- |
| Rover assets (IP) | The URDF and meshes belong to the OSU Robotics Club repository. Public visibility is not permission to redistribute. | The meshes live in a git-ignored `input/` folder and are never committed. A fresh clone regenerates them with `generate_input.sh`. A public release of the simulator needs the club's written permission. |
| Unity editor licence | The URDF Importer and ROS-TCP-Connector are Apache-2.0, but that says nothing about the editor. Unity Personal eligibility depends on Unity's editor terms and our institutional arrangement, not on student status. | Eligibility must be confirmed before winter-term handoff. If it fails, Gazebo becomes the fallback, independent of the technical gates. |
| Data and confidentiality | No personal data is used, and no confidentiality restrictions have been communicated. Terrain is procedural, not survey data. | AI tools may see the code and public rover data, but never credentials or any material later marked restricted. |

## Delegation and Validation

AI tools did substantial work on this decision, so most of our checks are mechanical, with human review at named points.

**What was handed to AI.** For the first draft, an AI assistant organised the RFC, researched documentation, proposed comparison criteria and gates, and drafted text. For the prototype, an AI coding agent (Claude Code) wrote and ran much of the URDF pipeline, the Unity scripts and tests, and the ROS 2 nodes, working inside this repository. For this revision, the agent read the repository and result files and drafted the updated sections. The project lead supplied scope, deadlines, team context and every decision.

**What makes that reasonable.** The agent's output is checked by machines, not taken on trust:

- 48 pipeline tests and 12 ROS-side tests run under `pytest`, and the GitHub Actions CI workflow runs `pytest` on the repository.
- Every pipeline run rebuilds the original URDF from its output and compares the two.
- Unity PlayMode tests compare the imported rover with the URDF numerically (pose, mass, inertia, wheel roundness).
- Physics behaviour is logged to JSON, so results are read from data rather than from the agent's summary.
- Every number in this RFC was traced to a result file or commit before inclusion. The failing ROS run is reported because the file says so.

**Where the lead stopped and validated.**

1. Corrected the scope in the first draft: we build the environment, not the algorithms.
2. Ran the feasibility, terrain, regression and arm-pose suites on the lead's own machine (the Unity logs record the lead's licence session) and read the result files.
3. Made every prototype commit under the lead's own git account (six commits, November 8–19), so every AI-written change entered the branch through the lead.
4. Chose which model simplifications to accept (locked four-bar joints, mesh-derived inertia, placeholder masses); each is recorded with its reason and source in `docs/assumed-data.md`.
5. Watched the rover drive the course and desert in the editor, which exposed the 30° ramp flip and the instant-stop kickback that the drive-chain work later fixed.
6. Reviewed every assumed value in `docs/assumed-data.md` and decided which to measure first.

**Still to validate before acceptance is final:** a second teammate reproduces setup (gate 1); the lead personally checks the cited Gazebo version guidance and the Unity editor terms; and the locked four-bar reading of the suspension is checked against the physical rover.

## Decision

**Accepted (conditional), November 2026.** The team adopted Unity 6.3 LTS for the prototype. Deciders: the project lead, with review by the three-person team.

Conditions: gate 4 (automated ROS 2 round trip) and gate 6 (ten-minute performance run on a recorded computer) pass by the end of week 2 of winter term, and Unity editor eligibility is confirmed. If gate 4 cannot pass, this RFC moves to **Superseded**, and a follow-up RFC records the move to Gazebo Harmonic. Both Status and this section will be updated in this file when the conditions resolve.

## References

- Gazebo. ["Installing Gazebo with ROS"](https://gazebosim.org/docs/harmonic/ros_installation/), Harmonic documentation.
- Unity Technologies. [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer), package documentation and Apache-2.0 licence (v0.5.2 used).
- Unity Technologies. [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) (v0.7.0) and [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (branch `main-ros2`).
- Unity. [Unity Editor Software Terms](https://unity.com/legal/editor-terms-of-service/software).
- OSU Robotics Club. [Rover_2026_2027](https://github.com/OSURoboticsClub/Rover_2026_2027), rover URDF at commit `1e79992`.
- CS Capstone Handbook. [RFC template](https://capstone.alexulbrich.com/rfc-template.md).

**Project evidence** (this repository, branch `feature/unity-physics-feasibility`):

- Commits `c60cd53`, `13ffb6a`, `66499c9`, `8801b7c`, `e90e091`, `e8bf1b9` (November 8–19, 2026).
- `urdf-pipeline/design.md`: the Unity mesh and articulation limits and the rover's numbers against them.
- `unity/MRC-Rover/FeasibilityResults/tuned/`: `import.json`, `physics.json`, `terrain_1.0mps.json`, `ros.json`, `regression.xml`, `armposes.xml`, `tests.xml`.
- `docs/assumed-data.md`: every assumed, derived or invented simulation value.

## Revision Log

No outside peer review has happened yet. The changes below respond to new project evidence, not to a reviewer.

| Date | Section | Change | Reason |
| --- | --- | --- | --- |
| 2026-11-22 | Recommendation, Decision | Gazebo Harmonic (provisional) replaced by Unity (conditional acceptance) | The Unity import risk was diagnosed and solved; Gazebo's remaining advantage reduces to one testable bridge question |
| 2026-11-22 | Analysis | Added measured import, physics, terrain and ROS results, including the failing ROS run | The first draft had no project evidence |
| 2026-11-22 | Recommendation | Gate table with current status | Shows which checks are passed and which are open |
| 2026-11-22 | Constraints | Recorded that rover meshes are kept out of the repository | Asset ownership was confirmed as the club's |
| 2026-11-22 | Delegation and Validation | Added the AI coding work and the checks that cover it | The first draft predated the code |
| 2026-10-25 | All | First draft | — |
