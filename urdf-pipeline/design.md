# URDF Split/Rebuild Pipeline — Design

> **Status: planning only.** Nothing in this folder is implemented yet. This document describes
> the intended design so the team can review it before any code, environment setup, or Unity
> work begins. It does not change the project's Unity-first direction.

## 1. Purpose

We want the OSU Robotics Club rover model to import into Unity cleanly. The source model is
in [OSURoboticsClub/Rover_2026_2027](https://github.com/OSURoboticsClub/Rover_2026_2027/tree/main),
in `software/ros_packages/rover_urdf` and `rover_arm_urdf`.

Two Unity limits stand in the way:

| Limit | What it means | Status for the rover (see §3) |
| --- | --- | --- |
| **Mesh vertex limit** | A Unity mesh with the default 16-bit index format holds at most 65,535 vertices | **Already exceeded**: 9 meshes (8 arm + the D455 camera) are over it |
| **Articulation link limit** | One `ArticulationBody` tree holds at most 64 links (PhysX) | **Close**: 55 links with the cameras (30 after collapsing frame-only links) |

The pipeline **splits** the model into pieces that fit within both limits. A **manifest**
records exactly how the pieces fit together. A **rebuild** step reassembles the pieces and
compares the result to the original, which proves the split lost nothing. Unity then
reassembles the same pieces using the manifest.

The core of the pipeline is Python data processing, so we can test it thoroughly in CI without
Unity or ROS installed.

## 2. Scope

| In scope (first implementation) | Out of scope (later) |
| --- | --- |
| URDF parsing and validation | Changing the rover's xacro sources upstream |
| Splitting STL meshes into chunks under the vertex limit | Mesh decimation (lossy; optional offline step) |
| Collapsing frame-only links | COLLADA (`.dae`) meshes, which only legacy files use |
| Splitting articulations, only if still over 64 links | Unity Editor reassembly script |
| Manifest + rebuild + automated equivalence tests | ROS–Unity runtime bridge, physics tuning |
| CLI: `check`, `split`, `merge` | |

## 3. Findings from the rover repository

I surveyed the files on 2026-11-07 (branch `main`).

### 3.1 How the full model is assembled

The model the rover actually runs is **not** a single committed URDF. xacro assembles it:

```text
rover2_arm/config/rover.urdf.xacro                 ← top-level entry point
└── rover_urdf/urdf/rover2_urdf.xacro              (macro "rover", arg attachment:=arm)
    ├── rover_urdf/urdf/rover2_body.urdf.xacro     chassis, ebox, tower, 6-wheel triple-bogie, gps/imu/lidar frames
    ├── rover_arm_urdf/urdf/rover2_arm.urdf.xacro  5-DOF arm + gripper  (attachment == arm)
    ├── realsense2_description/_d455.urdf.xacro    chassis depth camera
    └── realsense2_description/_d405.urdf.xacro    gripper depth camera  (attachment == arm)
+ ros2_control xacros (no links; hardware interfaces only)
+ <gazebo> sensor blocks (no links)
```

Because the model needs xacro and `realsense2_description`, the canonical input has to be
produced in the ROS 2 Jazzy environment (§11):

```bash
xacro rover2_arm/config/rover.urdf.xacro attachment:=arm > rover.urdf
```

### 3.2 Link count

| Component | Links | Notes |
| --- | --- | --- |
| Body (`rover2_body.urdf.xacro`) | 21 | 7 fixed, 5 revolute (bogies), 8 continuous (wheels) |
| Arm (`rover2_arm.urdf.xacro`) | 9 | 3 fixed, 5 revolute, 1 continuous |
| D455 + D405 cameras | 25 | almost all frame-only (optical frames, IMU frames) |
| **Total** | **55** (54 joints) | 64 is the hard limit |

Confirmed on 2026-11-08 from the xacro-generated model (upstream commit `1e79992`,
`attachment:=arm`): 55 links, a single root `rover_base_origin`, and `check_urdf` passes.
**25 of these links have no geometry and no mass.** They exist only as coordinate frames
attached by `fixed` joints. The Unity URDF Importer still creates an `ArticulationBody` for
each one, which uses up the link budget. Adding another camera or the science attachment would
push the model over 64.

### 3.3 Meshes

The current xacros reference only binary STL files. Triangle count = (bytes − 84) / 50.

| Meshes | Count | Triangles | Size | Over 65,535 vertices? |
| --- | --- | --- | --- | --- |
| Body (visual + collision share files) | 16 | 96 k total, max 9.8 k | 4.8 MB | No |
| Arm collision | 7 | 21 k total | 0.4 MB | No |
| Arm visual (`*-URDF_Mass.stl`) | 8 | **1.58 M total** | **79 MB** | **Yes, all 8** (39 k–386 k triangles each) |
| RealSense `d455.stl` (from `realsense2_description`) | 1 | 51 k | 2.6 MB | **Yes** |

The generated model references **34 meshes**, and **9 of them** are over the limit.

The heaviest are `mr2526-arm-Base_Link-URDF_Mass.stl` (386 k triangles, 19.3 MB) and
`mr2526-arm-Shoulder_Link-URDF_Mass.stl` (373 k triangles, 18.6 MB). STL stores 3 vertices
per triangle, so even after welding duplicate vertices, these are several times over the
65,535-vertex limit.

### 3.4 Data-quality issues to handle

- **`rover_urdf/urdf/rover2_body.urdf` (the committed, pre-expanded file) is invalid.** It has
  three root links (`rover_base_origin`, plus orphan links `camera_left_chassis` and
  `camera_right_chassis`), and joint `camera_right_chassis` points to a child link `camera2`
  that doesn't exist. It is also out of date compared with the xacro. **Do not use it as
  input.** Generate the input from the xacro instead.
- Some joints and links share a name (e.g. joint and link `camera_left_chassis`). URDF allows
  this, but the pipeline must keep link and joint names in separate namespaces.
- The xacro files use two different xacro namespace URIs (`http://www.ros.org/wiki/xacro` and
  `https://ros.org/wiki/xacro`). xacro handles both; we never parse the xacro ourselves.
- Mesh paths use `package://rover_urdf/...` and `package://rover_arm_urdf/...`. Unity resolves
  these from folders with the same names, so the output keeps that layout.
- Visual meshes are in millimetres with `scale="0.001 0.001 0.001"`. Mesh chunks must keep the
  scale exactly.
- **Licensing:** the meshes and URDFs belong to the OSU Robotics Club repository. We keep them
  in a git-ignored `input/` folder and do not commit them into this repo. Commit them only
  once the team has confirmed permission.

## 4. Pipeline overview

```text
 Rover_2026_2027 (xacro + STL)
          │  xacro (ROS 2 Jazzy, WSL2)
          ▼
   input/rover.urdf + input/<package>/urdf/stl_files/*.stl
          │
          ▼
   ┌──────────────┐  single root, no cycles, no dangling references,
   │ 1. validate  │  mesh files exist, report links and mesh vertex counts
   └──────┬───────┘
          ▼
   ┌──────────────┐  any mesh > max_vertices → N chunk STLs;
   │ 2. mesh split│  one <visual> becomes N <visual> elements, same origin/scale/material
   └──────┬───────┘
          ▼
   ┌──────────────┐  frame-only links on fixed joints → removed from the articulation,
   │ 3. collapse  │  recorded in the manifest as plain Unity transforms
   │    frames    │
   └──────┬───────┘
          ▼
   ┌──────────────┐  only if links > max_links: cut into several articulation parts
   │ 4. artic.    │  (skipped for the current rover; kept for future growth)
   │    split     │
   └──────┬───────┘
          ▼
   output/rover_unity.urdf (or output/parts/part_NN.urdf)
   output/meshes/<package>/urdf/stl_files/*_chunkNN.stl
   output/manifest.json
          │
          ├──► merge → output/rebuilt.urdf + rebuilt meshes → compared with input  (tests)
          └──► Unity: import, then reattach frames and bridge joints from the manifest  (future)
```

Each stage switches itself off when it isn't needed. For example, a model with 40 links and
light meshes passes through unchanged and produces an empty manifest.

## 5. Folder structure

Everything for this pipeline lives in `urdf-pipeline/`. Entries marked *(future)* do not
exist yet.

```text
urdf-pipeline/
├── design.md                        # this document
├── README.md                        # (future) quickstart: install, CLI usage, examples
│
├── urdf_pipeline/                   # (future) Python package — stdlib only at runtime
│   ├── __init__.py
│   ├── model.py                     # dataclasses: Robot, Link, Joint, Visual, Collision, Origin, Extra
│   ├── parse.py                     # URDF XML → Robot; validation (roots, cycles, dangling refs)
│   ├── stl.py                       # binary + ASCII STL read/write, vertex welding
│   ├── mesh_split.py                # stage 2: split heavy meshes into chunks
│   ├── frames.py                    # stage 3: collapse frame-only links
│   ├── split.py                     # stage 4: articulation partitioning + bridge joints
│   ├── merge.py                     # rebuild: manifest + outputs → original URDF + meshes
│   ├── manifest.py                  # manifest schema, load/save, version checks
│   ├── kinematics.py                # zero-configuration forward kinematics (4×4 transforms)
│   ├── compare.py                   # canonical model equality (float tolerance)
│   ├── report.py                    # `check` output: link budget, heavy meshes, problems
│   └── cli.py                       # `python -m urdf_pipeline check|split|merge`
│
├── tests/                           # (future) pytest suite
│   ├── conftest.py                  # shared fixtures, temp output dirs
│   ├── test_parse.py                # parsing + validation errors
│   ├── test_stl.py                  # STL read/write round trip, welding
│   ├── test_mesh_split.py           # chunk vertex limits, triangle preservation
│   ├── test_frames.py               # frame collapse + restore
│   ├── test_split.py                # articulation partitioning, cut preferences
│   ├── test_merge_roundtrip.py      # merge(split(x)) == x, for every stage
│   ├── test_kinematics.py           # world transforms identical before/after
│   ├── test_manifest.py             # schema/version handling
│   ├── test_cli.py                  # CLI end-to-end on fixtures
│   └── test_real_rover.py           # integration; skipped if input/rover.urdf is absent
│
├── fixtures/                        # (future) small synthetic data committed to git
│   ├── generate_fixtures.py         # regenerates every fixture deterministically
│   ├── rover_like.urdf              # same shape as the real rover: 6-wheel bogie + arm + camera frames
│   ├── chain_150.urdf               # forces articulation splitting
│   ├── boundary_59.urdf … boundary_65.urdf
│   ├── edge_extras.urdf             # <gazebo>, <transmission>, <ros2_control>
│   ├── edge_shared_names.urdf       # joint and link with the same name
│   ├── meshes/
│   │   ├── sphere_200k.stl          # generated heavy mesh (> 65,535 vertices)
│   │   ├── cube_ascii.stl           # ASCII STL input
│   │   └── tiny.stl                 # below the limit, must pass through untouched
│   └── invalid/                     # cycle, multiple roots, dangling child link, missing mesh file
│
├── config/                          # (future)
│   └── pipeline_config.example.json # limits, keep-together groups, cut weights
│
├── env/                             # dev environment for ROS 2 Jazzy (exists)
│   ├── wsl2-setup.md                # step-by-step guide for teammates
│   ├── wslconfig.example            # copy to %UserProfile%\.wslconfig
│   ├── setup_ros_jazzy.sh           # one-time Ubuntu 24.04 install: ROS 2 Jazzy + URDF tooling
│   ├── generate_input.sh            # clone the rover repo, run xacro, copy URDF + meshes into input/
│   └── verify_env.sh                # PASS/FAIL checklist for tools and generated input
│
├── unity/                           # (future) copied into the Unity project later
│   └── Editor/
│       └── UrdfManifestAssembler.cs # reads manifest.json: reattaches frames + bridge joints
│
├── input/                           # (future, gitignored) generated from Rover_2026_2027
│   ├── rover.urdf
│   ├── rover_urdf/urdf/stl_files/*.stl
│   └── rover_arm_urdf/urdf/stl_files/*.stl
└── output/                          # (future, gitignored) generated artifacts
    ├── rover_unity.urdf             # or parts/part_NN.urdf when articulation splitting runs
    ├── meshes/rover_arm_urdf/urdf/stl_files/*_chunkNN.stl
    ├── manifest.json
    └── rebuilt.urdf
```

**Repository integration (when implementation starts):**

- Add `urdf-pipeline/tests` to `testpaths` in the root `pyproject.toml` so the existing CI
  job (`pytest -q`, Python 3.12 on Ubuntu) runs these tests with no workflow changes.
- Add `urdf-pipeline/input/` and `urdf-pipeline/output/` to `.gitignore`.
- Runtime code uses the Python standard library only (`xml.etree`, `struct`, `json`,
  `hashlib`). Tests use `pytest`, which is already a dev dependency.
- CI never needs the real rover data, because the synthetic fixtures cover every stage.

## 6. Data model

| Element | Fields kept |
| --- | --- |
| `Robot` | name, links, joints, extras (robot-level elements that are not links or joints) |
| `Link` | name, `inertial`, `visual[]`, `collision[]`; unknown child XML kept raw |
| `Visual` / `Collision` | name, origin, geometry (mesh filename + scale, or box/cylinder/sphere), material |
| `Joint` | name, type (`fixed`, `revolute`, `continuous`, `prismatic`, `floating`, `planar`), parent, child, origin (xyz, rpy), axis, limit, dynamics, mimic, safety_controller, calibration |
| `Extra` | raw XML for `<gazebo>`, `<transmission>`, `<ros2_control>`, robot-level `<material>`, plus the link/joint names it references |

Link names and joint names are separate namespaces, since the rover reuses names across them.

**Validation rules:** exactly one root link; no cycles; each link has at most one parent
joint; every joint's parent and child exist; names are unique within their namespace; mimic
joints reference existing joints; every `package://` mesh resolves to a file under `input/`.
Validation errors list *all* problems at once, not just the first one.

## 7. Stages

### 7.1 Mesh split (stage 2)

For each `<visual>` or `<collision>` mesh whose welded vertex count is above `max_vertices`
(default **65,000**, a small margin under 65,535):

1. Read the STL, either binary or ASCII.
2. **Weld** vertices with identical coordinates. STL repeats each shared vertex for every
   triangle, so welding typically reduces the count about 6×.
3. Sort triangles spatially by Morton code of the centroid, so each chunk is one compact region
   of the part. That keeps chunks culled and lit well in Unity.
4. Fill chunks greedily. Keep adding triangles while the chunk's unique-vertex count stays at
   or below `max_vertices`, then start a new chunk.
5. Write each chunk as a binary STL: `<original>_chunk00.stl`, `_chunk01.stl`, and so on.
6. In the URDF, replace the single `<visual>` with N `<visual>` elements. Each gets the same
   `origin`, `scale` and `material`, and the name `<visual name>_chunkNN`.
7. Record the mapping in the manifest (`mesh_chunks`).

Collision meshes in the current rover are all small, so they pass through untouched. The rule
still applies to them if that changes.

**Not done here: decimation.** Reducing triangle counts (e.g. 386 k → 40 k) would make Unity
faster, but it is lossy, so it can't be proven by a round-trip test. If we need it, it runs as a
separate offline step (Blender or MeshLab in WSL) before this pipeline, and its output becomes
the new input.

### 7.2 Frame collapse (stage 3)

A link is **frame-only** when all three are true: it has no `visual`, no `collision` and no
`inertial` (or zero mass); its parent joint is `fixed`; and every link below it is also
frame-only.

- Each frame-only link is removed from the URDF. The manifest records it with its parent link
  and full origin.
- Extras that reference a collapsed frame (e.g. a `<gazebo reference="d455_depth_frame">`
  sensor) are kept in the manifest with that reference.
- In Unity, the frames come back as plain child `GameObject`s/`Transform`s, which are not
  `ArticulationBody` links and so don't count against the 64-link limit.
- For the current rover, this should bring the articulation from about 54 links down to about 30.

### 7.3 Articulation split (stage 4: only if still over `max_links`)

This stage is skipped for the current rover, but it is designed now so that adding attachments
can't silently break the import. Default `max_links` is **60**.

1. Compute subtree sizes with a post-order traversal.
2. Partition bottom-up. When adding a child's open part would push its parent over `max_links`,
   close the part at that child. The child's parent joint becomes a **bridge joint**.
3. Choose cuts by cost:
   - `fixed` joints are the cheapest.
   - Never cut inside a **keep-together group**, such as a bogie and its wheels, or the arm
     chain, unless nothing else works. In that case, emit a warning.
   - Avoid cuts that separate a mimic joint from the joint it follows.
   - Ties are broken by name, so output is deterministic.
4. Each part becomes a standalone, valid URDF whose root is the first link below a cut.

`config/pipeline_config.example.json` (planned):

```json
{
  "max_vertices": 65000,
  "max_links": 60,
  "collapse_frames": true,
  "keep_together": [
    ["rover_left_bogie", "rover_front_left_link", "rover_front_left_wheel", "rover_middle_left_wheel"],
    ["rover_rear_bogie", "rover_back_left_link", "rover_back_left_wheel", "rover_back_right_link", "rover_back_right_wheel"],
    ["rover_arm_base_link", "rover_arm_shoulder", "rover_arm_long_arm", "rover_arm_elbow_roll",
     "rover_arm_short_arm", "rover_arm_wrist_roll", "rover_arm_tool0", "rover_arm_gripper", "rover_arm_jaws_link"]
  ],
  "cut_cost": { "fixed": 1, "continuous": 5, "revolute": 5, "prismatic": 5, "mimic_crossing": 50 }
}
```

The bogie groups above are illustrative. Confirm the real link-to-bogie mapping against the
xacro before using them.

## 8. Manifest schema (v1)

```json
{
  "schema_version": 1,
  "source": {
    "file": "input/rover.urdf",
    "sha256": "<hash of input URDF>",
    "robot_name": "rover",
    "upstream": "OSURoboticsClub/Rover_2026_2027@<commit sha>"
  },
  "config": { "max_vertices": 65000, "max_links": 60, "collapse_frames": true },

  "mesh_chunks": [
    {
      "link": "rover_arm_base_link",
      "element": "visual", "index": 0,
      "source_mesh": "package://rover_arm_urdf/urdf/stl_files/mr2526-arm-Base_Link-URDF_Mass.stl",
      "source_sha256": "<hash>",
      "source_triangles": 385874,
      "chunks": [
        { "file": "package://rover_arm_urdf/urdf/stl_files/mr2526-arm-Base_Link-URDF_Mass_chunk00.stl",
          "triangles": 108000, "vertices": 64990 }
      ]
    }
  ],

  "collapsed_frames": [
    { "link": "d455_depth_optical_frame", "parent_link": "d455_depth_frame",
      "joint": "d455_depth_optical_joint",
      "origin": { "xyz": [0, 0, 0], "rpy": [-1.5708, 0, -1.5708] } }
  ],

  "parts": [
    { "id": "part_00", "file": "rover_unity.urdf", "root_link": "rover_base_origin", "links": ["..."] }
  ],
  "bridge_joints": [],

  "unassigned_extras": ["<raw XML not tied to one part>"],
  "warnings": []
}
```

- Hashes let `merge` and the Unity script detect an output that is stale relative to its input.
- Each section is empty when its stage did nothing.
- A bridge joint stores every field of the original joint (parent/child part and link, origin,
  axis, limit, dynamics, mimic), so the merge step can restore it exactly.
- The mesh-chunk example above is shortened. A real entry lists every chunk.

## 9. Merge (rebuild)

`merge` runs the stages in reverse:

1. Load the manifest, check `schema_version`, and verify the source hashes.
2. **Articulation:** graft each child part under its parent link using the bridge joints.
3. **Frames:** re-insert the collapsed links and their fixed joints.
4. **Meshes:** replace each set of chunk `<visual>` elements with the original single element.
   Optionally, concatenate the chunk STLs back into one mesh.
5. Restore the extras and write `rebuilt.urdf`.

This is the test oracle. It also documents exactly what the Unity reassembly script has to do.

## 10. Unity reassembly (future, separate plan)

- Import `output/rover_unity.urdf` (or each `part_NN.urdf`) with the Unity URDF Importer
  (`com.unity.robotics.urdf-importer`). Place `output/meshes/<package>/...` so `package://`
  paths resolve.
- Mesh chunks need no special handling. They import as several visual children of the same
  link, so they move together.
- `UrdfManifestAssembler.cs` (an Editor script) reads `manifest.json` and:
  - creates an empty child `GameObject` for each collapsed frame at its stored origin, so
    sensors and cameras can attach to `d455_color_optical_frame` and so on;
  - for each bridge joint (if any), adds a `FixedJoint` or `ConfigurableJoint` on the child
    part's root, sets `connectedArticulationBody` to the parent link, sets anchors from the
    origin, and sets motion locks from the joint type, axis and limits.
- **Frame conversion:** URDF is right-handed and Z-up, while Unity is left-handed and Y-up. The
  script must use the same conversion as the importer.
- **Physics trade-off:** joints between parts are less stable than articulation joints. That's
  why stage 4 is a last resort and prefers `fixed` cuts.

## 11. Development environment: Ubuntu 24.04 + ROS 2 Jazzy on Windows 11

ROS 2 Jazzy targets Ubuntu 24.04 (Noble). We need it for three things: expanding the rover
xacro (which needs `realsense2_description`), checking the result with `check_urdf`, and
reviewing it in RViz. Later it will also run the ROS–Unity bridge. Unity stays on Windows.

### Decision: WSL2 (recommended) vs a virtual machine

| Factor | WSL2 + Ubuntu 24.04 | VM (VirtualBox / VMware / Hyper-V) |
| --- | --- | --- |
| Jazzy install | Standard Ubuntu apt install | Same |
| GUI / RViz | WSLg gives windows and GPU-accelerated OpenGL | 3D acceleration is weak; RViz is often slow or broken |
| Performance / RAM | Near-native; memory grows and shrinks as needed | Fixed allocation, heavier |
| Files with Windows | `/mnt/c/...` and `\\wsl$\...` | Shared folders, slower and fiddly |
| Networking to Unity on Windows | `networkingMode=mirrored`, so `localhost` works both ways | Needs NAT/bridged port configuration |
| Conflicts | Uses the built-in Hyper-V platform | VirtualBox can conflict with Hyper-V/WSL2 |

**Choose WSL2.** Fall back to a VM only if a needed tool fails under WSL2, for example DDS
multicast discovery across several physical machines.

### Setup steps

The step-by-step guide is in [env/wsl2-setup.md](env/wsl2-setup.md). It uses these scripts:

| Step | Where | Command |
| --- | --- | --- |
| 1. Install WSL2 + Ubuntu 24.04 | Admin PowerShell | `wsl --install -d Ubuntu-24.04`, reboot, create a Linux user |
| 2. Configure WSL | PowerShell | copy [env/wslconfig.example](env/wslconfig.example) to `%UserProfile%\.wslconfig`, `wsl --shutdown` |
| 3. Install ROS 2 Jazzy + tooling | Ubuntu | `bash urdf-pipeline/env/setup_ros_jazzy.sh` |
| 4. Generate `input/` | Ubuntu | `bash urdf-pipeline/env/generate_input.sh` |
| 5. Verify | Ubuntu | `bash urdf-pipeline/env/verify_env.sh` |

What `generate_input.sh` does:

- builds only `rover_urdf` and `rover_arm_urdf`;
- passes the config files from `rover2_arm` and `rover2_control` to xacro as explicit
  source-tree paths, so MoveIt and the hardware packages never need to be built;
- runs `check_urdf`;
- copies `rover.urdf`, plus every mesh it references (including the RealSense meshes), into
  `input/<package>/...`;
- records the upstream commit in `input/SOURCE.txt`.

Keep this repo on the Windows side, since Unity needs it. WSL reaches it through `/mnt/c/...`.
The rover workspace lives in `~/rover_ws`, because builds under `/mnt/c` are slow.

**Later, for the ROS ↔ Unity bridge:** ROS-TCP-Connector (Unity package, Windows) plus
ROS-TCP-Endpoint (colcon workspace in WSL). **Open item:** confirm that ROS-TCP-Endpoint
builds on Jazzy. The upstream ROS 2 branch targeted older distributions, so a fork or patch
may be needed.

### Who runs what

| Step | Where |
| --- | --- |
| Clone the rover repo, xacro → URDF, `check_urdf`, RViz review | WSL2 (ROS 2 Jazzy) |
| `check` / `split` / `merge` + tests | Windows or WSL2 (plain Python 3.10+), and CI |
| Import + reassemble | Unity on Windows |

## 12. Test plan

### Synthetic fixtures

Fixtures are generated by `fixtures/generate_fixtures.py`, so CI doesn't depend on the rover repository.

| Fixture | Purpose |
| --- | --- |
| `rover_like.urdf` | Same structure as the real rover: 6 wheels on bogies, a 9-link arm, ~25 camera frame links |
| `meshes/sphere_200k.stl` | Heavy mesh, several times over the vertex limit |
| `meshes/cube_ascii.stl` | ASCII STL parsing |
| `meshes/tiny.stl` | Below the limit; must pass through untouched |
| `chain_150.urdf` | Forces several articulation cuts |
| `boundary_59` … `boundary_65` | Exactly at, below and above the link thresholds |
| `edge_extras.urdf` | `<gazebo>` blocks that reference collapsed frames; `<ros2_control>` |
| `edge_shared_names.urdf` | Joint and link with the same name (as in the rover) |
| `invalid/*` | Cycle, multiple roots, dangling child (like `camera2`), missing mesh file |

### What the tests assert

**Mesh split**
1. Every chunk has at most `max_vertices` unique vertices.
2. The chunks together hold exactly the original triangles: the multiset of welded triangles is
   equal, with none lost, duplicated or changed.
3. The bounding box and total surface area of the chunks match the original (tolerance 1e-9
   relative).
4. Each chunk `<visual>` keeps the original origin, scale and material.
5. Meshes under the limit, and non-mesh geometry, are left untouched (byte-identical).

**Frame collapse**

6. Only links that meet the frame-only definition are removed. Links with mass or geometry are never removed.
7. Every collapsed frame's world transform, computed from the manifest, equals its original world transform.

**Articulation split**

8. Every part has at most `max_links` links and is a valid single-root URDF.
9. Every link appears in exactly one part, and `len(bridge_joints) == len(parts) - 1`.
10. Keep-together groups are never split unless that is unavoidable (warning asserted).

**Whole pipeline**

11. **Round trip:** `merge(split(x))` is canonically equal to `x`, for every combination of
    stages switched on or off.
12. **Kinematic equivalence:** world transforms of all links at zero configuration are
    identical before and after.
13. **Determinism:** two runs produce byte-identical outputs and manifest.
14. **Validation:** every invalid fixture produces a clear error that names the bad element,
    and the CLI exits non-zero. The real committed `rover2_body.urdf` is rejected with
    "multiple roots" and "missing link camera2".
15. **Integration:** `test_real_rover.py` runs the full pipeline on `input/rover.urdf`. It is
    skipped automatically when the file is absent (e.g. in CI). It asserts that all 9 heavy
    visual meshes are chunked, that the articulation ends up at ≤ 60 links, and that the round
    trip is exact.

### Running (once implemented)

```bash
pytest -q                                            # whole repo, including pipeline tests
python -m urdf_pipeline check input/rover.urdf       # link budget, heavy meshes, validation report
python -m urdf_pipeline split input/rover.urdf --out output
python -m urdf_pipeline merge output/manifest.json --out output/rebuilt.urdf
```

## 13. Milestones

| # | Milestone | Done when |
| --- | --- | --- |
| 0 | **Baseline experiment** | The unmodified xacro-generated rover is imported into Unity, and exactly what fails is recorded (meshes, link count, import time) |
| 1 | Dev environment | WSL2 + Jazzy installed; `xacro` generates `rover.urdf`; `check_urdf` and RViz work |
| 2 | Model + parser + `check` | `test_parse.py` passes; `check` reports the real rover's budget correctly |
| 3 | Mesh split | `test_stl.py` and `test_mesh_split.py` pass; the arm imports into Unity without mesh errors |
| 4 | Frame collapse | `test_frames.py` passes; the articulation shrinks to ~30 links |
| 5 | Merge + equivalence | Round-trip and kinematic tests pass in CI |
| 6 | Articulation split | `test_split.py` passes on the synthetic large models |
| 7 | Unity reassembly | The manifest script restores frames (and bridges, if any); the rover drives in a test scene (separate plan) |

Milestone 0 matters: it confirms which limit actually breaks the import in our Unity version
before we build more than we need.

## 14. Risks

| Risk | Impact | Mitigation |
| --- | --- | --- |
| The binding limit differs from what we assume (e.g. the importer already splits meshes, or uses 32-bit indices) | Wasted effort | Milestone 0 first; each stage switches itself off when not needed |
| Even after chunking, 1.6 M visual triangles make Unity slow | Low frame rate | Optional offline decimation before the pipeline; Unity LODs |
| xacro needs extra workspace packages (`ros2_control` xacros) | Can't generate input | Build only the needed packages; pass `xacro:arg` values; record steps in `generate_input.sh` |
| Upstream rover repo changes | Stale input | Record the upstream commit SHA and hashes in the manifest; `merge` and Unity check them |
| Joints between parts are less stable than articulation joints | Jitter or separation in Unity | Frame collapse first; articulation split as last resort with `fixed`-joint preference |
| ROS-TCP-Endpoint may not build on Jazzy | Bridge work blocked | Verify early; use a community fork or patch it |
| WSL2 GPU/network quirks | RViz or bridge issues | Mirrored networking, update WSL; a VM is the fallback |
| Use of another team's CAD assets | Licensing/permission | Keep the assets git-ignored; get confirmation before committing them |

## 15. Open questions

- What does Milestone 0 show: is the mesh limit, the link limit, or something else the first failure?
- Which attachment does the simulation need first: `arm`, `northrop_arm`, or `science` (currently empty)?
- Do we have permission to commit the OSU rover meshes into this repository?
- Which Unity version and URDF Importer version will the team standardise on?
- Do we need the ROS ↔ Unity bridge for the Week 10 demo, or only the imported model?
