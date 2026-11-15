# Unity physics feasibility test

This folder holds a Unity 6.3 LTS project (`MRC-Rover/`) that checks whether **Unity can be
the physics backend** for a robot prepared by `urdf-pipeline`. The test imports the robot, then
checks that it lands, drives and moves its arm correctly in Unity's physics engine.

## Run it

You need:
- Unity `6000.3.28f1`, installed through Unity Hub;
- the repo's Python environment (`pip install -e .[dev]`);
- a URDF. The default is `urdf-pipeline/input/rover.urdf`, made by `urdf-pipeline/env/generate_input.sh`.

**Windows:**

```powershell
powershell -ExecutionPolicy Bypass -File unity\run_feasibility.ps1
# another robot / variant:
powershell -ExecutionPolicy Bypass -File unity\run_feasibility.ps1 -Urdf C:\robots\my.urdf -Variants collapsed
```

**Ubuntu / macOS:**

```bash
bash unity/run_feasibility.sh                    # or: URDF=/path/my.urdf bash unity/run_feasibility.sh collapsed
```

For each variant, the script:
1. runs `urdf_pipeline prepare`;
2. imports the result into Unity in batch mode;
3. runs the PlayMode physics tests;
4. prints a summary table.

Details are written to `MRC-Rover/FeasibilityResults/<variant>/` (`import.json`, `physics.json`, logs).

You can also open `MRC-Rover` in Unity Hub and run the tests from **Window → General → Test Runner → PlayMode**,
after importing a robot once from the command line.

## Variants

| Variant | Pipeline options | Purpose |
| --- | --- | --- |
| `default` | none | The robot as-is |
| `collapsed` | `--collapse-frames always` | Pure coordinate frames removed from physics (fewer bodies) |
| `split-meshes` | `--split-meshes` | Big STL meshes pre-split into chunks |
| `convex-unity` | Unity convex hulls instead of VHACD | Comparison only. Loses most colliders on the OSU rover |

## Tests

| Test | Checks | Pass if |
| --- | --- | --- |
| CoordinatesMatchManifest | Imported link poses vs the pipeline's ROS→Unity converted poses | ≤ 1 mm, ≤ 0.5° |
| MassMatchesUrdf | Total Unity mass vs the URDF mass (plus the pipeline's 1 g placeholders) | within 1% |
| SettlesOnGround | Drop from 0.2 m onto flat ground, then 5 s | no NaN; still < 0.05 m/s; tilt < 5° |
| DrivesStraight | All wheels driven for 0.5 m/s for 10 s | distance within 30%; heading drift < 10° |
| ArmReachesTargets | Each arm joint to +20° and back to 0° | error < 2° |
| PerformanceBudget | 1000 physics steps at 100 Hz | faster than real time |

Wheel, suspension and arm joints are identified from `manifest.json → physics.joints[*].role`.
If a role is guessed wrong, set `joint_roles` in a pipeline config file (`mrc-urdf init-config`).

## Test environment: course + Utah desert

`Assets/Scenes/RoverTestEnvironment.unity` is a baseline 3D environment for driving the rover in
Unity physics (Earth gravity, 100 Hz). The scene is almost empty: when you press Play,
`EnvironmentGenerator` builds everything from code, so nothing generated is committed.

- **Test course** on a concrete pad, with one lane per challenge: ramps of 10°, 20° and 30° (0.6 m rise),
  steps of 5, 10 and 15 cm, a 10 m rubble field (rocks 5–20 cm) and a 10 m rough-ground patch.
- **Utah desert**: a 400 m × 400 m terrain modelled on the University Rover Challenge site near
  the Mars Desert Research Station (Hanksville, UT). It has red sandy flats, grey bentonite mounds,
  dry washes, flat-topped mesas with layered red, purple and grey cliffs, and about 700 rocks. It is
  a stylised, procedural approximation, not survey data.

**Drive it yourself:**
1. Import a robot once (`run_feasibility`).
2. Open `MRC-Rover` in Unity Hub, then open `Assets/Scenes/RoverTestEnvironment.unity`.
3. Press **Play**.

| Keys | Action |
| --- | --- |
| W/S, A/D (or arrows) | Drive / turn (skid steer); Shift = faster; Space = stop |
| 1–9, Q/E, Z | Select an arm joint, move it, return the arm home |
| T / G / R | Next test lane / Utah desert / reset to the current start |
| Right mouse, scroll, C | Orbit, zoom, switch between chase and free camera; H hides the help |

To use a different imported variant, select the `Environment` object and change **Robot Variant**.
**MRC → Capture Environment Screenshots** saves preview images to `FeasibilityResults/screenshots/`.
**MRC → Create Test Environment Scene** rebuilds the scene file.

**Automated terrain tests** (`TerrainCourseTests`) drive the rover at 0.5 m/s over each lane and
90 s into the desert, steering to hold a straight line. A test only *fails* if the physics goes
unstable. Whether the rover crossed each feature, got stuck or flipped is recorded in
`terrain.json` and shown in the runner's summary. PhysX "enhanced determinism" is enabled, so
repeated runs give identical results.

## What is committed

The project settings, packages and scripts are committed. Generated files are git-ignored:
`Library/`, imported robots (`Assets/Robots/`), and results.
