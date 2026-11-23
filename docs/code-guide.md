# Code guide: URDF pipeline and Unity rover simulation

This guide explains every file and function in the two code bases:

- **`urdf-pipeline/`**: Python, standard library only. It checks a URDF robot model and prepares it for Unity.
- **`unity/`**: the Unity 6.3 project `MRC-Rover` plus its runner scripts. It imports the prepared robot, runs physics tests and provides a drivable test environment.

Each part is ordered **bottom-up**: first the small building blocks that depend on nothing, then the code that uses them, ending with the commands you actually run. Read top-down to learn the system; jump to a module's table when you need to change something.

---

## 1. The big picture

```
 rover.urdf (from the OSU rover repo, via xacro)
      │
      ▼
 ┌─────────────────────────── urdf-pipeline (Python) ───────────────────────────┐
 │ check   : validate + report problems (changes nothing)                        │
 │ prepare : copy + fix for Unity + record every change ──► output/<name>/       │
 │             rover.urdf, meshes/, manifest.json, REPORT.md                     │
 │ rebuild : undo everything from manifest.json ──► identical original URDF      │
 └───────────────────────────────────────────────────────────────────────────────┘
      │  prepared folder (URDF still in ROS coordinates; manifest has Unity values)
      ▼
 ┌─────────────────────────── unity/MRC-Rover (C#) ──────────────────────────────┐
 │ Editor/RoverImport      : URDF Importer → prefab in Assets/Robots/<variant>/   │
 │ Scripts/Feasibility     : RoverRig + DriveController = the robot at runtime   │
 │ Scripts/Environment     : Utah desert, test course, keyboard driving, HUD      │
 │ Tests/PlayMode          : audit, physics and terrain tests → FeasibilityResults│
 └───────────────────────────────────────────────────────────────────────────────┘
      ▲
 unity/run_feasibility.ps1 / .sh  (runs prepare → import → tests, prints a summary)
 unity/physics_sweep.ps1          (re-runs the tests under different physics settings)
```

**The manifest is the contract between the two halves.** Python works out everything that needs physics knowledge: masses, inertia, joint units, couplings, springs, drive limits. It writes these into `manifest.json`, already converted to Unity units and coordinates. The C# code only reads those values and never converts units itself. To change robot-specific behaviour, change the **config file** (`urdf-pipeline/config/*.json`) and re-run `prepare`. Editing C# is rarely needed.

### Every name has a description
Hovering over any function, class, field, property or constant in VS Code shows what it is.
- **Python** uses docstrings. Module-level constants, type aliases and dataclass fields get a docstring line directly below them, which Pylance shows on hover.
- **C#** uses `/// <summary>` comments, one declaration per line, so each field has its own description.
- Local variables can't carry hover documentation in either language, so they're named for what they hold and commented where it isn't obvious.

`python tools/doc_coverage.py` lists anything undocumented, and the test `tests/test_doc_coverage.py` fails if something is. When you add code, add its description too.

### Units and coordinates (cheat sheet)

| | ROS / URDF | Unity |
|---|---|---|
| Axes | X forward, Y left, Z up (right-handed) | X right, Y up, Z forward (left-handed) |
| Position | `(x, y, z)` | `(-y, z, x)` |
| Quaternion | `(x, y, z, w)` | `(y, -z, -x, w)` |
| Joint angles | radians | degrees (`ArticulationDrive.target`, limits) |
| Drive stiffness / damping | — | **per radian / rad/s** (Unity passes them to PhysX unscaled) |
| Inertia tensor | link frame | rotated with `M I Mᵀ`; principal moments plus rotation are given to Unity |

---

## 2. URDF pipeline (`urdf-pipeline/urdf_pipeline/`)

### 2.1 How the layers depend on each other

```
Layer 6  cli.py, __main__.py                      ← you run these
Layer 5  pipeline.py (check, prepare)  merge.py (rebuild)  report.py
Layer 4  config.py   manifest.py   parameters.py
Layer 3  physics.py
Layer 2  meshes.py  mesh_split.py  mass_properties.py  compat.py  frames.py  split.py
Layer 1  parse.py  coordinates.py  kinematics.py  compare.py
Layer 0  errors.py  model.py  stl.py
```
A module only imports from its own layer or the layers below it. If you add a module, keep it that way.

### 2.2 Layer 0: foundations

#### `errors.py`: errors shown to end users
| Name | What it does |
|---|---|
| `PipelineError(message, hint)` | A problem the user can fix (missing file, bad setting). `hint` is printed as "Fix: …". The CLI exits with code 1. |
| `ValidationError(problems, source)` | The URDF is structurally invalid; `problems` lists **every** problem found. The CLI exits with code 2. |

#### `model.py`: the in-memory robot
The URDF XML is kept as `xml.etree` elements, so nothing is lost when it is written back.

| Name | What it does |
|---|---|
| `Link(element)` | Wraps a `<link>`. `.name`; `.has_geometry` (has a visual or collision); `.mass` (from `<inertial><mass>`, or `None`). |
| `Joint(element)` | Wraps a `<joint>`. `.name`, `.type`, `.parent`, `.child` (link names), `.mimic` (name of the joint it mimics, or `None`). |
| `Robot` | `links`, `joints` (dicts by name), `extras` (every other top-level element, e.g. `<gazebo>`, `<ros2_control>`), `name`, `attrib`. |
| `Robot.add_link / add_joint / remove_link / remove_joint` | Edit the tree. |
| `Robot.copy()` | Deep copy (all pipeline stages work on copies). |
| `Robot.parent_joints()` | child link → the joint that attaches it. |
| `Robot.child_joints()` | parent link → its joints, sorted by name (deterministic). |
| `Robot.roots()` | Links with no parent joint (a valid robot has exactly one). |
| `Robot.tree_order(start)` | Link names from the root down, parents before children. |

#### `stl.py`: minimal STL reader and writer
| Name | What it does |
|---|---|
| `StlMesh` | `header` and `triangles` (each a 13-tuple: normal, 3 vertices, attribute). `.raw_vertex_count` = 3 × triangles (what Unity creates); `.welded_vertex_count` = shared corners counted once. |
| `read_stl(path)` | Reads binary or ASCII STL. Raises `PipelineError` if it is neither. |
| `write_stl(mesh, path)` | Writes binary STL, keeping every value bit-for-bit. |
| `triangle_count(path)` | Fast count from the binary header, without loading the mesh. |
| `unique_vertices(triangles)` | Set of distinct vertices (used for the welded count). |
| `_is_binary(data)` | Detects the binary format by checking the size implied by the triangle count. |

### 2.3 Layer 1: parsing and maths

#### `parse.py`: read, validate and write URDF files
| Name | What it does |
|---|---|
| `load_urdf(path)` | Parses a file into a `Robot`. Rejects unreadable files, non-URDF XML and un-expanded xacro, with a hint to run `xacro`. Does **not** check the tree structure. |
| `is_xacro(data)` | True if the file still needs xacro. Ignores xacro text inside comments, which xacro leaves in its own output. |
| `parse_urdf_string(data, source)` | Same as `load_urdf`, but from a string (used by tests). |
| `validate(robot)` | Returns **every** structural problem: no links; unknown joint type; missing or undefined parent/child; self-joint; mimic of an unknown joint; a link with two parents; zero or several roots; loops or disconnected links. An empty list means valid. |
| `load_valid_urdf(path)` | `load_urdf` + `validate`; raises `ValidationError` if anything is wrong. |
| `element_to_string / element_from_string` | Store an XML element in the manifest as text, and get it back. |
| `to_xml_string(robot) / write_urdf(robot, path)` | Write the robot back out (links, joints, then extras, in the original order). |

#### `coordinates.py`: ROS ↔ Unity conversion
| Name | What it does |
|---|---|
| `ros_to_unity_position(v)` / `ros_to_unity_direction(v)` | `(x, y, z)` → `(-y, z, x)`. |
| `ros_to_unity_scale(s)` | `(x, y, z)` → `(y, z, x)` (scales have no sign). |
| `ros_to_unity_quaternion(q)` | `(x, y, z, w)` → `(y, -z, -x, w)`. |
| `ros_rpy_to_unity_euler_deg(rpy)` | The Euler angles exactly as the URDF Importer passes them to `Transform.Rotate`. |
| `rpy_to_quaternion(rpy)` | URDF roll/pitch/yaw (fixed axes X, then Y, then Z) → quaternion. |
| `quaternion_multiply`, `rotate_vector`, `unity_euler_to_quaternion`, `_axis_angle` | Quaternion helpers. |
| `matrix_to_quaternion(r)` | 3×3 (or 4×4) rotation matrix → quaternion (numerically stable). |
| `same_rotation(a, b, tol)` | True if two quaternions are the same rotation (`q` and `-q` count as equal). |
| `unity_block_from_origin(xyz, rpy, axis)` | A joint or frame origin as a `"unity"` manifest block: position, rotation, Euler, axis. |
| `pose_blocks_from_matrix(t)` | A 4×4 pose → `{"ros": …, "unity": …}` (used for `reference_poses`). |
| `_clean(values)` | Rounds for readable JSON and turns `-0.0` into `0.0`. |

#### `kinematics.py`: forward kinematics (ROS frame)
| Name | What it does |
|---|---|
| `parse_origin(element)` | `(xyz, rpy)` of an element's `<origin>` (zeros when absent). |
| `parse_axis(element)` | A joint's `<axis xyz>`, or `None` (URDF default `1 0 0`). |
| `rpy_matrix(rpy)` | rpy → 3×3 rotation `Rz·Ry·Rx`. |
| `transform(xyz, rpy)`, `matmul(a, b)` | Build and multiply 4×4 transforms. |
| `joint_motion(type, axis, value)` | Transform of a joint moved by `value`: radians about the axis (revolute/continuous) or metres along it (prismatic). |
| `world_transforms(robot, positions=None)` | Pose of every link relative to the root. Joints are at zero unless given in `positions` (joint → value). Used for reference poses, centre of mass and spring sizing. |
| `max_difference(a, b)` | Largest element difference between two pose sets (the self-check uses it to prove nothing moved). |

#### `compare.py`: are two robots identical?
| Name | What it does |
|---|---|
| `canonical(element)` | Order-insensitive, number-format-insensitive form of an XML element. |
| `diff_robots(a, b, limit)` | Human-readable differences (links, joints, extras); an empty list means identical. Used by the self-check, `rebuild` and the tests. |

### 2.4 Layer 2: geometry and model fixes

#### `meshes.py`: finding the mesh files a URDF uses
| Name | What it does |
|---|---|
| `MeshUse` | One `<mesh>` reference: link, kind (`visual`/`collision`), URI, and the holder element. |
| `mesh_uses(robot)` | Every mesh reference in the robot. |
| `resolve_mesh(uri, urdf_dir, mesh_roots)` | Finds the file. `package://pkg/path` is looked up as `<root>/pkg/path` or `<root>/path`, trying the URDF folder, each `mesh_roots` folder, then the URDF's parent folder. Also handles `file://` and relative paths. Returns `None` if not found. |
| `is_absolute(uri)` | True for absolute paths or `file://` URIs (these can't be copied portably). |
| `output_key(uri)` | Where the mesh goes inside the output folder (`package://pkg/a/b.stl` → `pkg/a/b.stl`); `None` if it can't be placed safely. |
| `companion_files(mesh_path)` | Extra files a mesh needs: textures a `.dae` references, the `.mtl` and texture maps of an `.obj`. |
| `chunk_uri(uri, index)` | URI of mesh chunk *n* (`name_chunk00.stl`). |

#### `mesh_split.py`: cut a heavy STL into chunks (optional, `split_meshes`)
| Name | What it does |
|---|---|
| `vertex_count(mesh, mode)` / `needs_split(mesh, max_vertices, mode)` | Count vertices the way Unity does (`raw`) or welded, and compare with the limit. |
| `_morton_order(triangles)` | Sorts triangles along a space-filling curve, so each chunk is a compact region. |
| `split_mesh(mesh, max_vertices, mode)` | Splits into chunks under the limit. |
| `triangles_match(original, chunks)` | Proves no triangle was lost or changed. |

Unity's URDF Importer already splits meshes above 65,535 vertices, so this stage is off by default.

#### `mass_properties.py`: inertia from shapes, and plausibility checks
| Name | What it does |
|---|---|
| `mesh_solid_properties(path, scale, rotation, translation)` | Volume, centroid and inertia (per unit density) of a closed mesh, by signed-tetrahedron integration, in the link frame. |
| `link_meshes(link, resolved)` | All of a link's STL **collision** meshes, falling back to **visual** meshes (e.g. camera models). |
| `combined_solid_properties(meshes)` | Combines several solids (parallel-axis theorem). |
| `urdf_inertia_tensor(link)` | The URDF inertia rotated into the link frame, plus the centre of mass. |
| `max_possible_ratio(tensor, com, mass, meshes)` | URDF moment ÷ the **largest moment physically possible** (all mass at the part's farthest point). Above 1 = impossible for any density. |
| `inertia_plausibility(robot, resolved, ratio_limit=3)` | Per link: URDF vs uniform-solid estimate (`ratio`), `max_possible_ratio`, `impossible`, `plausible`, implied density, centre-of-mass offset, and the mesh-derived tensor. |
| `replace_inertia(link, tensor)` | Writes a new tensor into `<inertia>` (keeps the URDF mass and centre of mass). Used by `fix_inertia`. |
| `eigen_symmetric(m)` | Eigenvalues/eigenvectors of a symmetric 3×3 matrix (Jacobi). |
| `principal_axes(tensor)` | Principal moments plus the rotation quaternion, the form Unity's `inertiaTensor` / `inertiaTensorRotation` need. |

#### `compat.py`: fixes for Unity's URDF Importer (meaning unchanged, all undone by `rebuild`)
| Name | What it does |
|---|---|
| `joints_missing_axis(robot)` / `add_default_axes(robot)` / `remove_added_axes(robot, names)` | The importer crashes on a moving joint with no `<axis>`. `prepare` writes the URDF default `1 0 0` explicitly, and `rebuild` removes it. |
| `links_missing_inertial(robot)` / `add_placeholder_inertials(robot, mass)` / `remove_placeholder_inertials(robot, names)` | The importer gives links without `<inertial>` a mass of **1 kg**. `prepare` gives them a tiny placeholder (default 1 g), marked with the attribute `urdf_pipeline_placeholder`. |

#### `frames.py`: remove pure coordinate frames from the physics tree
A *frame-only* link has no shape and no mass, is attached by a fixed joint, and everything below it is frame-only too (camera optical frames, IMU frames). Each one would cost one of Unity's 64 articulation links.

| Name | What it does |
|---|---|
| `frame_only_links(robot)` | Lists them, parents first. |
| `collapse_frames(robot)` | Removes them and returns manifest entries (ROS and Unity offsets, plus the original XML). |
| `restore_frames(robot, entries)` | Puts them back (`rebuild`). |

#### `split.py`: split a robot into several articulations (over 64 links)
| Name | What it does |
|---|---|
| `Part`, `SplitPlan` | A part's id, root link and links; the plan's parts, the cut joints and warnings. |
| `single_part_plan(robot)` | Everything in one part (the usual case). |
| `plan_split(robot, max_links, keep_together, cut_cost)` | Greedy bottom-up: walking from the leaves, whenever a subtree would exceed `max_links`, it cuts the cheapest child joint. Fixed joints are cheapest; mimic joints and joints inside a `keep_together` group are expensive. |
| `build_part_robots(robot, plan)` | One standalone `Robot` per part. Each cut joint becomes a *bridge joint* in the manifest. |

### 2.5 Layer 3: physics (`physics.py`)
This module is the physics knowledge of the pipeline: checks, plus every value the Unity side reads.

| Name | What it does |
|---|---|
| `total_mass(robot)`, `link_mass(link)` | Mass sums. |
| `joint_roles(robot, overrides)` | Guesses each moving joint's role. **wheel**: continuous joint with wheel/tire/tyre in its name. **suspension**: has a wheel below it. **arm**: has an effort limit. **passive**: anything else. Override any guess with the `joint_roles` setting. |
| `physics_issues(robot, overrides)` | Warnings for `check`: links with a shape but no mass, bad or unreadable inertia, large mass ratios between neighbours, free-swinging continuous joints, the total mass. |
| `link_inertials(robot)` | Per link: mass, centre of mass and full tensor in ROS **and** Unity coordinates, plus Unity principal moments and rotation. |
| `center_of_mass(robot)` | Whole-robot centre of mass (all joints at zero). |
| `physics_block(robot, overrides)` | The manifest `physics` block: total mass, links, centre of mass, and every moving joint with its role, child link, `unity_drive` limits in **Unity units**, URDF damping/friction, and its axis in the root frame. |
| `joint_couplings(robot, configured)` | Linkages URDF can't describe: follower = multiplier × leader + offset. Reads URDF `<mimic>` tags and the `joint_couplings` setting. `"same_rotation"` / `"opposite_rotation"` are turned into ±1 from the joint axes. |
| `joint_springs(robot, configured, overrides, couplings)` | Springs/dampers (e.g. shocks). `"auto"` stiffness holds the joint within `max_sag_deg` under the robot's weight, worked out by moving the joint (and its coupled joints) and measuring the wheels' vertical travel. `"auto"` damping is critical damping. |
| `_static_wheel_loads(robot, wheels, weight)` | Weight on each wheel standing on flat ground (balance of forces and moments about the centre of mass). |
| `locked_joints(robot, names)` | Validates the `locked_joints` setting (Unity makes them fixed). |
| `drive_limits_block(limits)` | The `drive_limits` setting for Unity; deceleration defaults to acceleration. |
| `unlimited_wheels(robot, overrides)` | Wheel joints with no effort/velocity limit (triggers the `check` hint about `drive_limits`). |
| `_joint_axis_in_root(...)` | A joint axis in the root frame (Unity coordinates). |
| `_limit`, `_dynamics`, `_inertia`, `_movable_below`, matrix helpers | Small readers and helpers. |

### 2.6 Layer 4: settings and the manifest

#### `config.py`: every setting `prepare` understands
`Config` is a dataclass. `Config.check()` validates every value with a friendly message. `Config.from_dict` / `load_config(path)` read JSON: keys starting with `_` are notes, and unknown keys get a "did you mean …?" suggestion. `EXAMPLE_CONFIG` is what `init-config` writes.

| Setting | Default | Meaning |
|---|---|---|
| `max_links` | 60 | Most links per Unity articulation (hard limit 64). |
| `collapse_frames` | `auto` | Remove frame-only links: `auto` (only when over `max_links`), `always`, or `never`. |
| `split_meshes`, `max_vertices`, `mesh_vertex_mode` | `false`, 65535, `raw` | Optional STL chunking. |
| `keep_together`, `cut_cost` | `[]`, `{}` | Steer the articulation split. |
| `mesh_roots`, `ignore_missing_meshes` | `[]`, `false` | Where to look for meshes; whether to continue without them. |
| `joint_roles` | `{}` | Correct a guessed role, e.g. `{"wrist_joint": "arm"}`. |
| `massless_link_mass_kg` | 0.001 | Placeholder mass (`null` = leave such links alone). |
| `fix_inertia` | `false` | Replace implausible inertia with mesh-derived values (recorded, undone by `rebuild`). |
| `joint_couplings` | `{}` | `{"follower": {"leader": "...", "multiplier": 1 \| "same_rotation" \| "opposite_rotation", "offset_deg": 0}}` |
| `joint_springs` | `{}` | `{"joint": {"stiffness_nm_per_rad": "auto" \| n, "damping_nms_per_rad": "auto" \| n, "rest_deg": 0, "max_sag_deg": 2}}` |
| `locked_joints` | `[]` | Joints held fixed at zero in Unity. |
| `joint_limits` | `{}` | Real joint speed/acceleration limits (rad/s, rad/s²), e.g. from MoveIt's `joint_limits.yaml`; Unity follows them with a trapezoidal profile. |
| `physical_parameters` | `{}` | The registry (see `parameters.py`): `{name: {value, unit, status, source, uncertainty, applies_to}}`. |
| `drive_limits` | `{}` | The real drive controller's limits: `max_linear_speed_mps`, `max_angular_speed_radps`, `max_linear_accel_mps2`, `max_linear_decel_mps2`, `max_angular_accel_radps2`, `wheel_accel_rev_s2`, `controller_rate_hz`, `controller_wheel_separation_m`, `controller_wheel_radius_m`. |

**`config/osu_rover.json`** is the OSU rover's settings. Every value carries a `_note` saying where it came from:
- `fix_inertia`: the URDF inertia is impossible on 15 links.
- `locked_joints`: the front "four-bar" bars meet at one bogie pin, so the links can't move.
- `drive_limits`: from `rover_drive.yaml`, `joy_to_drive.py` and `drive_can_control.py`.

#### `parameters.py`: the physical-parameter registry
Every physical number the simulation's accuracy depends on, with its source and certainty (the config's `physical_parameters`). See [rover-measurement-guide.md](rover-measurement-guide.md).

| Name | What it does |
|---|---|
| `PARAMETER_SPECS`, `STATUSES` | Known parameter names with their units; allowed statuses `measured`, `rover_code`, `urdf`, `cad`, `estimate`, `assumed`. |
| `check_parameters(params)` | Validation (called by `Config.check`): known names, required `value`/`status`, matching unit, value shapes. |
| `parameters_block(params)` | The registry as written to `manifest.physics.parameters` (notes dropped, units filled in). |
| `fidelity_rows(params)` | Everything not `measured` / `rover_code`, least trusted first; printed by `check` and `REPORT.md`. |
| `mass_adjustment(robot, params)` | Whether `total_mass_kg` / `center_of_mass_offset_m` differ from the URDF (differences under 0.01 % count as rounding). |
| `apply_mass_adjustment(robot, adj)` / `restore_inertials(robot, originals)` | Scale every real link's mass and inertia, and shift every link's centre of mass by the offset (root frame → link frame), so the robot's centre of mass moves by exactly that amount. Undone by `rebuild`. |

#### `manifest.py`: reading and writing `manifest.json`
| Name | What it does |
|---|---|
| `write_manifest(data, path)` / `read_manifest(path)` | JSON in and out, with a schema check (`REQUIRED_KEYS`, `SCHEMA_VERSION`). |
| `is_pipeline_manifest(path)` | True if a folder holds this tool's own output (so `prepare` may overwrite it). |

**Manifest keys and who reads them:**

| Key | Contents | Read by |
|---|---|---|
| `source`, `config`, `stats`, `outputs`, `warnings` | Input file and hash, settings used, counts, written files | humans, `rebuild`, `RoverImport` |
| `parts`, `bridge_joints` | Articulation split | `rebuild`; Unity reads `parts[0].root_link` |
| `collapsed_frames`, `mesh_chunks`, `mesh_files`, `unity_compat`, `inertia_from_meshes` | Everything `prepare` changed | `rebuild` (to undo it) |
| `reference_poses` | Every link's pose relative to the root, in ROS and Unity coordinates | `CoordinatesMatchManifest` test |
| `physics.links`, `physics.center_of_mass`, `physics.total_mass_kg`, `physics.unity_expected_mass_kg` | Mass properties in Unity coordinates | `RoverImport.ApplyExactInertia`, `ModelAuditTests`, `MassMatchesUrdf` |
| `physics.joints` | Role, child link, `unity_drive` limits, URDF damping, axis | `RoverRig` |
| `physics.couplings`, `physics.springs`, `physics.locked_joints`, `physics.drive_limits` | Behaviour URDF can't express | `RoverRig`, `DriveController` |
| `physics.joint_limits` | Real joint speed/accel limits, in radians and Unity degrees | `ArmController` |
| `physics.parameters` | The physical-parameter registry | `SimParameters` (all of `RoverRig`) |
| `mass_adjustment` | Mass scale, centre-of-mass offset, original `<inertial>` XML | `rebuild` |
| `physics.inertia_check` | Plausibility per link | humans, `check` |

### 2.7 Layer 5: the operations

#### `pipeline.py`: `check()` and `prepare()`
| Name | What it does |
|---|---|
| `check(urdf_path, config)` → `CheckResult` | Validates, finds meshes (missing, absolute, non-STL, too heavy), runs the physics checks (inertia plausibility, no drive limits, missing axes, massless links), and predicts frame collapse and the number of parts. Changes nothing. `.ok` is false on structure problems or missing meshes. |
| `prepare(urdf_path, out_dir, config, log)` → `PrepareResult` | Builds the Unity-ready folder. The stages are listed below. |
| `_prepare_output_dir` | Output-folder safety: refuses a folder containing the input or files it didn't create; replaces its own earlier output. |
| `_split_meshes`, `_copy_meshes`, `element_from_holder`, `_safe_stem`, `_should_collapse`, `_mesh_roots` | Stage helpers. |

**`prepare` stage order.** The order matters; `rebuild` undoes them in reverse.
1. Load and validate the URDF (`ValidationError` lists every problem).
2. Resolve meshes (fail with a hint if any are missing, unless `ignore_missing_meshes`).
3. Check the output folder is safe, then work on a copy of the robot.
4. Write default axes explicitly (`compat`).
5. `fix_inertia` (optional): replace implausible tensors. The original XML goes to `inertia_from_meshes`. A separate "physics source" copy gets the same fix, so the manifest's values match the file.
5b. Registry mass data (`total_mass_kg`, `center_of_mass_offset_m`), if it differs from the URDF. The original XML goes to `mass_adjustment`.
6. Split meshes (optional).
7. Collapse frame-only links (`auto` / `always`).
8. Add placeholder masses. This happens **after** collapsing, otherwise the frames would gain mass and stop counting as frame-only.
9. Plan the articulation split; write one URDF per part.
10. Copy meshes and their companion files.
11. Write `manifest.json` (all blocks in §2.6).
12. **Self-check:** rebuild from the output and compare with the original. Any difference is an internal error and nothing is reported as successful.
13. Write `REPORT.md`.

#### `merge.py`: `rebuild` (the inverse of `prepare`)
| Name | What it does |
|---|---|
| `rebuild_robot(manifest, out_dir)` | Re-assembles the parts, then undoes in order: bridge joints, collapsed frames, added axes, placeholder masses, registry mass adjustment, replaced inertia, mesh chunks. |
| `rebuild(manifest_path, output)` → `RebuildResult` | Optionally writes the rebuilt URDF, and compares it with the original if the original is still there with the same hash. |
| `sha256_file(path)` | File hash (detects a changed source). |

#### `report.py`: plain-English output
| Name | What it does |
|---|---|
| `render_check(result, config)` | The `check` text: summary, `[FAIL]`/`[WARN]`/`[INFO]` lines, what `prepare` would do. |
| `render_prepare_report(manifest)` | `REPORT.md`: what changed and the Unity import steps. |
| `_name`, `_list` | Formatting helpers. |

### 2.8 Layer 6: the command line (`cli.py`, `__main__.py`)
`python -m urdf_pipeline <command>`, or `mrc-urdf <command>` after `pip install -e .`.

| Command | Function | Does |
|---|---|---|
| `check URDF [--config F] [--max-links N] [--mesh-root DIR]… [--ignore-missing-meshes]` | `_cmd_check` | Prints the report. Exit 0 = ready, 2 = problems. |
| `prepare URDF [-o DIR] [--config F] [--split-meshes] [--collapse-frames auto\|always\|never] [--fix-inertia]` | `_cmd_prepare` | Builds the folder (default `./output/<name>`). |
| `rebuild MANIFEST [-o FILE]` | `_cmd_rebuild` | Rebuilds and compares. Exit 2 if different. |
| `init-config [PATH] [--force]` | `_cmd_init_config` | Writes an example settings file. |

Other functions:
- `_config_from_args` merges the `--config` file with command-line overrides.
- `build_parser` defines the commands.
- `main` maps errors to exit codes: 1 = user-fixable, 2 = invalid / check failed, 3 = internal bug (prints a traceback).

### 2.9 Pipeline tests (`urdf-pipeline/tests/`)
Run all tests with `python -m pytest -q urdf-pipeline/tests`.

| File | Covers |
|---|---|
| `builders.py` | Helpers that write small test robots (`robot`, `link`, `joint`, `rover_like_items`, `write_rover_like`, `write_stl`). |
| `test_parse.py` | Loading, xacro detection, validation messages. |
| `test_coordinates.py` | ROS↔Unity conversions against known values. |
| `test_physics_and_compat.py` | Roles, physics warnings, Unity-unit conversion, axis and placeholder round trips, frame collapse, couplings, springs, locked joints, drive limits. |
| `test_mass_properties.py` | Cube inertia, principal axes, the hard physical bound, the visual-mesh fallback, `fix_inertia` with rebuild. |
| `test_pipeline.py` | `check` output, lossless `prepare` with each setting, mesh split, articulation split, output-folder safety, CLI exit codes, the real rover. |

### 2.10 Common pipeline edits
- **A new robot needs a correction:** add it to a config JSON (copy `config/osu_rover.json`) rather than to code.
- **Add a new setting:**
  1. Add a field to `Config`.
  2. Validate it in `Config.check`.
  3. Add a help entry to `EXAMPLE_CONFIG`.
  4. Use it in `pipeline.prepare`. If it changes the URDF, record the change in the manifest and undo it in `merge.rebuild_robot`.
  5. Add a test.
- **Change a plausibility threshold:** `PLAUSIBLE_RATIO` in `mass_properties.py`.
- **Change a role guess:** `joint_roles` in `physics.py`. Users can also override roles per robot with the `joint_roles` setting.

---

## 3. Unity project (`unity/MRC-Rover/Assets/`)

### 3.1 Folders, assemblies and dependencies
```
Assets/
  Scripts/Feasibility/  (assembly MRC.Feasibility: no dependencies; runtime)
      FeasibilityPaths  PhysicsTuning  SimParameters  SurfaceFriction
      DriveController  ArmController  RoverRig
  Scripts/Environment/  (assembly MRC.Environment → MRC.Feasibility, Unity.InputSystem)
      EnvMaterials  ProceduralMeshes  DesertTerrain  TestCourse
      EnvironmentGenerator  RoverTeleop  FollowCamera  RoverHud
  Editor/               (editor-only; uses the URDF Importer package)
      RoverImport  EnvironmentSceneBuilder  EnvironmentSnapshot
  Tests/PlayMode/       (assembly MRC.Feasibility.Tests → both assemblies + Test Runner)
      TestRobot  ModelAuditTests  RoverPhysicsTests  TerrainCourseTests
  Scenes/RoverTestEnvironment.unity   (almost empty; everything is built when you press Play)
  Robots/<variant>/      (generated by RoverImport; not in git)
FeasibilityResults/      (generated test results; not in git)
```
Packages: URDF Importer v0.5.2, ROS-TCP-Connector v0.7.0 (an importer dependency), Newtonsoft JSON, Test Framework, Input System.

### 3.2 What happens every physics step
This order is the same in tests (`RoverRig.Simulate`) and in the scene (`RoverTeleop.FixedUpdate`):
```
RoverRig.StepControllers(dt):
  1. DriveControl.Step(dt)     speed/acceleration limits → wheel motor ramps → wheel drive targets
  2. ArmControl.Step(dt)       arm setpoints move along trapezoidal profiles within the real joint limits
  3. ApplyPassiveDamping()     viscous damping on bogie pivots (their Coulomb friction is a capped drive)
  4. UpdateCouplings()         linkage followers track their leaders (none for the OSU rover now)
Physics.Simulate(dt)           PhysX (tests: script mode, 100 Hz; scene: FixedUpdate at 100 Hz)
```

### 3.3 Layer 0: shared helpers (`Scripts/Feasibility/`)

#### `FeasibilityPaths.cs`
| Member | What it does |
|---|---|
| `ProjectRoot`, `ResultsRoot` | Project folder, and `FeasibilityResults/`. |
| `Arg(name)` | The value after `name` on Unity's command line (e.g. `-variant tuned`), or `null`. |
| `WriteResult(variant, file, json)` | Writes `FeasibilityResults/<variant>/<file>`. |
| `SetCurrentVariant(variant, prefab, manifest)` / `CurrentVariant()` | Records which robot was imported last (`current.json`), so tests know what to load when `-variant` isn't given. |

#### `PhysicsTuning.cs`
| Member | What it does |
|---|---|
| `Timestep` | `-dt` from the command line, else `DefaultTimestep` (0.01 s = 100 Hz). |
| `ApplySolverIterations(rig)` | Gives every body the project's default solver iteration counts. |
| `Describe()` | Snapshot of the physics settings (solver, iterations, friction, determinism…) saved with every result file. |

Solver settings live in **ProjectSettings/DynamicsManager.asset**: PGS, 6 position / 1 velocity iterations, enhanced determinism on. `physics_sweep.ps1` compares alternatives.

#### `SimParameters.cs`: the registry at runtime
| Member | What it does |
|---|---|
| Constructor `(registry, commandLine)` | Reads `manifest.physics.parameters`, plus every `-param name=value` on Unity's command line. |
| `Float(key)`, `Vector(key)` | The override if given, else the registry value, else `null`. |
| `IsOverridden(key)`, `HasAny(prefix)`, `RegistryVector(key)` | Helpers (overrides are applied as differences from the registry). |
| `Snapshot()` | Values, statuses and overrides, saved as `parameters_used` in every result file. |

Experiment-only keys: `tyre_mu_all`, `arm_accel_scale`, `controller_wheel_radius_m`.

#### `SurfaceFriction.cs`: tyre-on-surface friction
Wheels get `Tyre` (μ = 1, combine **Multiply**). Unity uses the higher-priority combine mode, so contact μ = 1 × the ground's μ, and each ground material's μ *is* the tyre-on-that-surface value. `Register(material, "concrete"|"sand"|"rock")` is called by `EnvMaterials` and `RoverRig.CreateGround`. `Configure(parameters)` writes `tyre_mu_<surface>_static/_dynamic` into them. Robots without tyre parameters keep the old averaged friction.

### 3.4 Layer 1: the robot at runtime

#### `DriveController.cs`: the rover's real drive chain
It copies the real rover's chain: **operator command → `diff_drive_controller` limits → ODrive motor ramp → wheel**. All limits come from `manifest.physics.drive_limits`; with none, commands pass straight through (instant).

| Member | What it does |
|---|---|
| Constructor `(rig, limits)` | Reads `max_linear_speed_mps`, accelerations, `wheel_accel_rev_s2`, `controller_rate_hz`, and the controller's wheel separation and radius. |
| `Command(linear, angular)` | The operator's target in m/s and rad/s (+ = turn left), clamped to the max speeds. The first call activates the controller. Nothing touches the wheels before that, so tests can still set wheels directly. |
| `Step(dt)` | Once per physics step: (1) at `controller_rate_hz`, move the body speed toward the command within the accel/decel limits; (2) mix forward speed and turn rate into left/right wheel speeds using the controller's separation and radius; (3) ramp each wheel's target at `wheel_accel_rev_s2`; (4) `rig.SetWheelSpeed`. |
| `Limit(current, target, accel, decel, h)` | The `diff_drive_controller`-style limiter: uses `decel` when the change opposes the motion, and stops at zero before reversing. |
| `Release()` | Stops writing wheel targets (used after a teleport, and by tests that set wheels directly). |
| `Linear`, `Angular` / `CommandLinear`, `CommandAngular` | The limited output, and the requested input (shown on the HUD). |
| `Describe()` | One-line summary of the limits. |

#### `ArmController.cs`: real arm motion limits
Each arm joint's target is followed along a **trapezoidal velocity profile** within `manifest.physics.joint_limits` (falling back to the URDF velocity limit). The position drive tracks the moving setpoint with velocity feed-forward, like the real geared position control. `SetTarget`, `Step(dt)`, `Reset()`; `MaxVelocity`/`MaxAcceleration`/`Setpoint` for tests and the HUD. `RoverRig.SetArmTarget` goes through it for managed joints.

#### `RoverRig.cs`: spawn, configure and step one imported robot
Constants (top of the file; units are per **radian**, see §1):

| Constant | Value | Used for |
|---|---|---|
| `WheelDamping` | 50 | Wheel velocity drive gain (N·m per rad/s). |
| `WheelDefaultForce` | 30 | Wheel torque cap when neither the registry (`wheel_max_torque_nm`) nor the URDF gives one. |
| `ArmStiffness`, `ArmDamping` | 10000, 100 | Arm position drives. |
| `CouplingStiffness`, `CouplingDamping`, `CouplingForce` | 5000, 100, 10000 | Linkage drives. |
| `PlaceholderMassKg` | 0.01 | Links lighter than this are pipeline placeholders and are ignored by the at-rest check. |
| `CouplingReactionSign` | 1 | Diagnostics only (`-couplingReaction on\|off\|flip`). |

Setting up:

| Member | What it does |
|---|---|
| `RoverRig(manifest)` | Holds the parsed manifest. `Joints` is `manifest.physics.joints`; `RootLinkName` comes from `parts[0]`. |
| `Spawn(prefab)` | Instantiates the prefab and calls `Attach`. |
| `Attach(robot)` | Indexes links and transforms, finds the root, and measures its height above the lowest collider. Then, in order: `ResetImporterDynamics`; creates `Parameters`; `ApplyMassOverrides`; `ApplyTyreFriction`; `LockJoints`; `IgnoreAssembledOverlaps`; creates `DriveControl` and `ArmControl`; `LoadCouplings`; `LoadSprings`. |
| `ResetImporterDynamics()` | **Importer bug fix.** URDF joint `<dynamics damping>` becomes the child *body's* linear and angular damping (world drag, about 80 N on the OSU bogies at 3.7 m/s). `<dynamics friction>` becomes PhysX `jointFriction` (a load-proportional coefficient, not N·m). Both are reset to 0; the real values are applied in `ConfigureDrives`. Listed in `importer_dynamics_reset`. |
| `IgnoreAssembledOverlaps()` | Collider pairs of different bodies that already overlap in the assembled pose (parts pinned together, e.g. a bogie and its linkage's second bar) stop colliding. Otherwise they jam the joint. Listed in `assembled_overlap_bodies`. |
| `ApplyMassOverrides()` | `-param total_mass_kg` / `center_of_mass_offset_m` (relative to the registry): scale real bodies' mass and inertia; shift every body's centre of mass. |
| `ApplyTyreFriction()` | With `tyre_mu_*` in the registry: wheel colliders get `SurfaceFriction.Tyre`. |
| `LockJoints()` | Joints in `locked_joints` become `FixedJoint`s. |
| `LoadCouplings()` | Reads `couplings`. Fixes the sign if the importer reversed one joint's axis, and sets up reaction-torque routing for nested linkages. |
| `IgnoreLinkageCollisions()` | Bodies of the same linkage (and its mount) stop colliding with each other, because the overlapping parts at a missing pin would fight the linkage. |
| `LoadSprings()` | Reads `springs`; flips `rest_deg` if the importer reversed the axis. |
| `ConfigureDrives()` | Sets every drive:<br>• wheels: velocity drives, torque cap `wheel_max_torque_nm`;<br>• arm: stiff position drives through `ArmControl`, URDF effort cap;<br>• passive joints: **Coulomb friction** as a velocity-to-zero drive capped at the friction torque (`passive_joint_friction_nm`, or URDF friction), plus damping;<br>• springs: shock drives;<br>• coupled followers: linkage drives.<br>Call after spawning. |
| `PassiveJoints`, `ApplyPassiveDamping()` | The free joints with friction/damping; explicit viscous torque on the joint and its reaction on the parent. |
| `StepControllers(dt)` | The per-step order in §3.2. Called by `Simulate`, `RoverTeleop.FixedUpdate` and `RosRoverBridge.FixedUpdate`. |

Placing and the ground:

| Member | What it does |
|---|---|
| `CreateGround(static, dynamic)` | A 500 m test plane with a friction material. |
| `LowestPoint()` | World Y of the lowest collider point. |
| `PlaceAboveGround(clearance)` | Lifts the rover so its lowest point is `clearance` above y = 0. |
| `PlaceAt(position, yaw, clearance)` | Teleports onto whatever ground is below (raycast), facing `yaw` (0 = +Z), and zeros all velocities. |

Driving and joints:

| Member | What it does |
|---|---|
| `JointsWithRole(role)`, `JointBody(joint)` | Look up joints by role, and the body a joint moves (its child link). |
| `WheelGeometry()` | Wheel joints, average radius, left-right track. |
| `IsLeftWheel(joint)`, `ForwardSign(joint, forward)`, `WheelRadius(joint)` | Which side a wheel is on; which spin direction pushes the rover forward (the importer reverses wheel axes); the radius from collider bounds. |
| `Drive(speed, turn, track, radius, wheels)` | **Raw** skid steer: sets wheel targets instantly. Use `DriveControl` for realistic driving. |
| `SetWheelSpeed(joint, deg/s)`, `SetArmTarget(joint, deg)` | Set one drive's target. |
| `JointDegrees(joint)` | Current angle in degrees (0 for locked joints). |
| `JointAxisWorld(body)` | A joint's world spin axis (the anchor's X axis). |
| `UpdateCouplings()` | Points each follower drive at multiplier × leader + offset, and routes its reaction torque to the leader's parent (as the missing closing link would). |
| `Simulate(steps, dt)` | The per-step loop in §3.2. **Always step through this**, never `Physics.Simulate` directly. |

Measurements and clean-up:

| Member | What it does |
|---|---|
| `AnyNaN()`, `MaxLinkSpeed(realPartsOnly)`, `FastestLink(realPartsOnly)`, `TiltDegrees()`, `TotalMass()` | Measurements used by the tests. |
| `Destroy()` | Removes the robot and ground immediately (tests run back to back). |

### 3.5 Layer 2: editor tools (`Editor/`)

#### `RoverImport.cs`
`Unity -batchmode -projectPath unity/MRC-Rover -executeMethod RoverImport.ImportFromCommandLine -prepared <folder> -variant <name> [-convex vhacd|unity]`

| Member | What it does |
|---|---|
| `ImportFromCommandLine()` | Reads the arguments, calls `Import`, and exits with 0 or 1. |
| `Import(preparedDir, variant, convex)` | Copies the prepared folder into `Assets/Robots/<variant>/` and runs the URDF Importer (Y-up, VHACD or Unity convex decomposition). It then disables the importer's `Controller`, because it would overwrite all drives in `Start`. Next it calls `ApplyExactInertia`, saves the prefab, records the variant as current, and writes `import.json` (time, bodies, colliders, vertices, errors and warnings). |
| `ApplyExactInertia(robot, manifest)` | Sets each body's centre of mass, principal moments and rotation from the manifest. The importer mis-rotates non-diagonal tensors; this fixes that. |
| `CopyDirectory`, `Capture` | Helpers (log capture during the import). |

#### `EnvironmentSceneBuilder.cs`
Menu **MRC → Create Test Environment Scene**, or `-executeMethod EnvironmentSceneBuilder.CreateFromCommandLine [-variant tuned]`. `Create(variant)` saves `Scenes/RoverTestEnvironment.unity` containing one `EnvironmentGenerator` set to that robot variant.

#### `EnvironmentSnapshot.cs`
`-executeMethod EnvironmentSnapshot.CaptureFromCommandLine` (needs graphics, so no `-nographics`). `Capture(variant)` builds the environment without Play mode and saves preview PNGs to `FeasibilityResults/screenshots/`.

### 3.6 Layer 3: the test environment (`Scripts/Environment/`)

| File | Members | What it does |
|---|---|---|
| `EnvMaterials.cs` | `Concrete` (μ 0.9/0.8), `Sand` (0.65/0.55), `Rock` (0.8/0.7); `Color(c, smoothness)`; `NoiseTexture(...)`; `TileNoise` | Shared physics materials (friction values here) and simple materials/textures. |
| `ProceduralMeshes.cs` | `Wedge`, `Rock(seed)`, `Heightfield(w, l, cell, height)`, `Flat`, `Icosahedron`, `Subdivide` | Generated meshes: ramps, rocks, bumpy ground. |
| `DesertTerrain.cs` | `Build(parent, flatCenter, flatSize, seed)`, `HeightAt`, `Layers`, `Splat`, `ScatterRocks`, `Noise.Fbm` | A 400 m Utah (Hanksville/MDRS-style) desert: sandy flats, bentonite mounds, banded mesas and dry washes. It keeps a flat area for the course and scatters rocks. `Size`, `HeightRange`, `BaseLevel` and the colours are constants at the top. |
| `TestCourse.cs` | `Build(parent, origin)` → lanes; `Ramp`, `Wedge`, `Step`, `Rubble`, `Rough`, `RockObject`, `Box` | Eight 30 m lanes, 5 m apart: ramps of 10/20/30° (0.6 m rise), steps of 5/10/15 cm, rubble, rough ground. `CourseLane` = name, start, `EndZ`. Lane sizes are constants at the top. |
| `EnvironmentGenerator.cs` | `Awake`, `Build(parent, desert, course, seed)`, `SetUpLighting`, `SpawnRover`; fields `robotVariant` (default `tuned`), `buildDesert`, `buildCourse`, `spawnRover`, `terrainSeed` | On Play it sets 100 Hz physics and Earth gravity, builds sun, sky, desert and course, then spawns the rover from `Assets/Robots/<robotVariant>/` and adds teleop, the follow camera and the HUD. `EnvironmentInfo` carries the lanes, start pad and desert start. |
| `RoverTeleop.cs` | `Init`, `Update`, `FixedUpdate`, `Teleport`; fields `speedScale`, `fallbackMaxSpeed/TurnRate`, `armDegreesPerSecond` | Keyboard: W/S/A/D drive, `[` `]` speed scale, Space stop, 1–9 + Q/E arm, Z arm home, T next lane, G desert, R reset. `Update` reads keys and calls `DriveControl.Command`; `FixedUpdate` steps the drive chain and couplings. Top speeds come from `drive_limits` (OSU: 3.7 m/s, 4 rad/s). |
| `FollowCamera.cs` | `LateUpdate`; `distance`, `height`, `smoothing` | Chase camera: right mouse orbits, scroll zooms, C switches chase/free. |
| `RosRoverBridge.cs` | `Init(rig, teleop)`, `ResetOrigin`, `OnCmdVel`, `FixedUpdate`, `Update`, `BuildOdometry`, `RequestedOnCommandLine`; fields for topics, `odomRateHz` (30), `cmdTimeout` (0.5 s), `rosIp`/`rosPort` | ROS 2 link through ROS-TCP-Connector. `/cmd_vel` (Twist) goes into `DriveControl.Command`. `/odom` (Odometry, ROS axes, origin = start or last teleport) is published at 30 Hz. `/waypoint_demo/status` is shown on the HUD. Stops the rover if `/cmd_vel` is quiet for 0.5 s. Holding a drive key overrides ROS (`RoverTeleop.ManualActive`). Turned on by `EnvironmentGenerator.enableRos` or `-ros`. Needs the `ROS2` scripting define (Project Settings → Player). |
| `RoverHud.cs` | `Update`, `OnGUI`; `Message` | On-screen readout: actual / controller / commanded speed, drive limits, pitch, roll, position, arm joint, physics rate, FPS. H hides the help. |

### 3.7 Layer 4: tests (`Tests/PlayMode/`)
Every test writes its measurements to `FeasibilityResults/<variant>/*.json`, **pass or fail**. The runner scripts print them as tables.

| File | Test | Passes if |
|---|---|---|
| `TestRobot.cs` | `Load()` | Loads the variant (`-variant`, or the last imported), its prefab and its manifest. |
| `ModelAuditTests.cs` → `audit.json` | `InertialsMatchUrdf` | Every link's mass, centre of mass (< 1 mm) and full inertia (< 1 %) match the manifest. |
| | `RobotCenterOfMassMatchesUrdf` | Whole-robot centre of mass within 5 mm. |
| | `WheelAxesMatchUrdf` | Wheel spin axes within 0.5° (a reversed direction is allowed and reported). |
| | `WheelCollidersAreRound` | Wheel collision hulls less than 5 % out of round. |
| `RoverPhysicsTests.cs` → `physics.json` | `CoordinatesMatchManifest` | Every link within 1 mm and 0.5° of `reference_poses` (proves the ROS→Unity conversion). |
| | `MassMatchesUrdf` | Total mass within 1 % of `unity_expected_mass_kg`. |
| | `SettlesOnGround` | Dropped from 0.2 m: no NaN, at rest (every real part < 0.05 m/s for 0.5 s) within 15 s, tilt < 5°, linkages hold, springs off their stops. Records the settling time. |
| | `DrivesStraight` | 0.5 m/s for 10 s through `DriveControl`: distance within 30 % of what the controller asked for, heading drift < 10°. |
| | `ArmReachesTargets` | Each arm joint reaches +20° and back within 2°. Its peak speed (50 ms average) and its time to target are within 10 % of the trapezoidal profile from the real joint limits. |
| | `FrictionAndTorque` | Rover held in the air: every wheel drive saturates at exactly `wheel_max_torque_nm` (±2 %); every bogie pivot moves when pushed, while resisting with exactly its friction torque (±10 %). |
| | `PerformanceBudget` | 1000 steps run faster than real time. |
| | `AccelerationAndBraking` | Full command from rest: acceleration and braking within 25 % of the `drive_limits` values, pitch change when stopping < 10°, no tip. Also records an *instant stop* for comparison. |
| `RosWaypointTests.cs` → `ros.json` | `FollowsWaypointsFromRos` | Only with `-ros` (otherwise ignored): ROS sends `/cmd_vel` within 20 s, no NaN, and the ROS `waypoint_follower` reports `done` within 180 s. Records message counts and rates, time, and the path. |
| `TerrainCourseTests.cs` → `terrain.json` | `CourseLane(lane)` × 8, `DesertTraverse` | Fails only on instability (NaN, a link faster than 20 m/s). Whether the rover crossed is a measurement: progress, stuck/flip, tilt, speed. `DriveStraight(...)` steers back to the lane centre through `DriveControl`. |

Test constants are at the top of each file (`SettleSeconds`, `MaxSettleSeconds`, `RestSpeed`, `DriveSpeed`, `LaneTimeout`, `StuckWindow`, …).

### 3.8 Layer 5: runner scripts (`unity/`)
Keep the Unity editor closed while these run: batch mode can't open a project the editor already has open.

| Script | What it does |
|---|---|
| `run_feasibility.ps1 [-Variants a,b] [-Urdf F] [-UnityPath P] [-Python P]` (Windows) / `run_feasibility.sh [variants…]` (Linux/macOS) | For each variant: `prepare` → `RoverImport` → all PlayMode tests. Prints a summary (result, bodies, import time, mass, settling time, accel/decel, drive, arm, speed) and a terrain table. |
| `sensitivity_sweep.ps1 [-Only a,b] [-ReportOnly]` | Runs the physics and terrain tests at the registry values (baseline), then at a low and a high value of each uncertain parameter (`-param`). Prints how much each metric moved and ranks the parameters; that ranking orders `docs/rover-measurement-guide.md`. Results go to `FeasibilityResults/sensitivity/` and `ranking.json`. About 9 min per run. |
| `run_ros_demo.ps1 [-Variant tuned]` | Runs `RosWaypointTests` headless with `-ros` (the ROS 2 launch must already be running in WSL) and prints the `ros.json` summary. |
| `physics_sweep.ps1 [-Profiles a,b] [-Variant tuned]` | For each solver profile (PGS/TGS, iterations, 50/100/200 Hz): edits `DynamicsManager.asset`, runs the tests with `-dt`, saves to `FeasibilityResults/sweep/<profile>/`, and **always restores** the settings file. |

**Variants** (in the `$variantSettings` table of the runner):

| Variant | `prepare` options |
|---|---|
| `tuned` (default) | `--config urdf-pipeline/config/osu_rover.json` |
| `default` | none |
| `collapsed` | `--collapse-frames always` |
| `split-meshes` | `--split-meshes` |
| `convex-unity` | none, but Unity's own convex decomposition |
| `fixed-inertia` | `--fix-inertia` |

### 3.9 Common Unity edits
- **Feel / handling:** change `drive_limits` in the config JSON and re-run the runner. Avoid hard-coding speeds in C#.
- **Wheel grip:** friction in `EnvMaterials.cs` (terrain) and `RoverRig.CreateGround` (test plane).
- **Drive stiffness:** the constants at the top of `RoverRig.cs` (per radian!).
- **Physics rate or solver:** `EnvironmentGenerator.FixedTimestep`, `PhysicsTuning.DefaultTimestep`, and `ProjectSettings/DynamicsManager.asset`. Check with `physics_sweep.ps1`.
- **A new test lane:**
  1. Add a builder in `TestCourse.Build`.
  2. Add its name to `TerrainCourseTests.Lanes`.
- **A new per-robot behaviour:**
  1. Compute it in `physics.py` and write it into the manifest.
  2. Read it in `RoverRig.Attach`.
  3. Apply it in `ConfigureDrives` or `Simulate`.
- **Driving from a script:** call `rig.DriveControl.Command(v, w)` and step with `rig.Simulate`. Calling `SetWheelSpeed` directly bypasses the realistic limits.

---

## 4. ROS 2 side (`ros2/`)
See [ros2/README.md](../ros2/README.md) for running it.

| File | Members | What it does |
|---|---|---|
| `src/mrc_waypoint_demo/mrc_waypoint_demo/controller.py` | `WaypointController.step(x, y, yaw)` → `Command`; `parse_waypoints`, `wrap_angle`, `yaw_from_quaternion` | The algorithm, with no ROS dependency: advance to the next waypoint inside `tolerance`, turn rate = `k_heading` × heading error, forward speed = `max_speed` × cos(error) (zero above `turn_in_place_deg`), slow down near the last waypoint. |
| `…/waypoint_follower.py` | `WaypointFollower` node (`on_odom`, `on_timer`, `destroy_node`), `main` | `/odom` → controller → `/cmd_vel` at `rate_hz`; publishes `/waypoint_demo/status`; optional CSV log. Parameters: `waypoints`, `max_speed`, `max_turn`, `k_heading`, `tolerance`, `slowdown_radius`, `rate_hz`, `log_csv`. |
| `…/launch/waypoint_demo.launch.py` | `generate_launch_description` | Starts Unity's `ros_tcp_endpoint` (port 10000) and the follower; every parameter is a launch argument. |
| `…/test/test_controller.py` | | Drives a simple model of the rover (with its acceleration limits) around the square; runs in the normal `pytest`. |
| `setup_ros2_bridge.sh` | | Clones Unity's ROS-TCP-Endpoint (`main-ros2`) into `~/mrc_ros_ws`, links this package, and runs `colcon build`. |

## 5. Known limits of the current model
- Many physical inputs are not measured yet (`check` prints the fidelity table); see [rover-measurement-guide.md](rover-measurement-guide.md).
- No motor torque data, so the wheel torque cap (`WheelDefaultForce` = 30 N·m) is an estimate. With the drive limits the drives need about 2.5 N·m on the flat and about 7 N·m on a 20° slope.
- Wheel radius: the simulation uses the collision mesh (0.154 m); the rover software uses 0.146 m (odometry-tuned) and 0.15 m (CAN node).
- Gripper jaws and the lidar IMU have no mass in the URDF; they get 1 g placeholders.
- Open-loop straight driving drifts by a varying amount (the rover's URDF isn't left/right symmetric). The terrain tests steer like an operator.
