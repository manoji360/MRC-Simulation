# Rover measurement guide

The simulation is only as accurate as its inputs. The URDF comes from a CAD export, and several of its numbers have already turned out to be wrong. Every physical number the simulation uses sits in one place, the `physical_parameters` section of [`urdf-pipeline/config/osu_rover.json`](../urdf-pipeline/config/osu_rover.json). Each one records where it came from (`status`):

| status | meaning |
|---|---|
| `measured` | measured on the real rover (the goal) |
| `rover_code` | taken from the rover's own software (controller configs) |
| `urdf` / `cad` | taken from the URDF / CAD export: unverified |
| `estimate` / `assumed` | educated guess: replace first |

`mrc-urdf check … --config urdf-pipeline/config/osu_rover.json` lists every parameter that isn't yet `measured` or `rover_code`.

## Measurement priority
Ranked by the sensitivity sweep (`unity/sensitivity_sweep.ps1`, November 2026). Each unmeasured parameter was run at the low and high end of its plausible range, and its effect on driving, terrain and arm results was measured. Measure in this order:

| # | Parameter | Range tested | Effect in simulation | Procedure |
|---|---|---|---|---|
| 1 | **Centre-of-mass height** (`center_of_mass_offset_m` z) | ±5 cm | +5 cm: 30° ramp progress halves (13.4 → 6.6 m) and other lanes reach 36° tilt (near tip-over); acceleration −7 % | §2 |
| 2 | **Wheel torque limit** (`wheel_max_torque_nm`) | 10–50 N·m | At 10 N·m the 15 cm step and the 30° ramp can't be climbed (lanes 7 → 6) | §4 |
| 3 | **Tyre friction** (`tyre_mu_*`) | 0.4–0.9 | Changes slope behaviour (more grip climbs the 30° ramp further into the tipping zone); low grip triples heading drift | §3 |
| 4 | **Effective wheel radius** (`wheel_radius_m`) | 0.146–0.154 m | Ground speed and odometry scale by about 5 %; acceleration about 6 % | §5 |
| 5 | Arm acceleration (`joint_limits`, shoulder) | ±25 % | Arm move times ±10 %; nothing else | §7 |
| 6 | Total mass (`total_mass_kg`) | ±10 % | Small: the drive is speed-controlled and its acceleration comes from the controller | §1 |
| 7 | Bogie pivot friction (`passive_joint_friction_nm`) | 0–0.5 N·m | Negligible | §6 |

Notes:
- Items 1 and 6 come from the same weighing session (§1 and §2), so do them together.
- Item 2 needs only a laptop and `odrivetool`: the quickest high-value measurement.
- Re-run the sweep after measuring; the ranking changes as uncertainties shrink.

## How to enter a measurement
1. Edit the parameter's entry in `osu_rover.json`:
   ```json
   "total_mass_kg": { "value": 48.9, "unit": "kg", "status": "measured",
                      "source": "6 wheel scales, 2026-11-20, rover in competition config with arm stowed",
                      "uncertainty": "+/-0.2 kg", "applies_to": "all link masses (prepare scales them when this changes)" }
   ```
   Always write **how, when and in what configuration** you measured (battery fitted? arm pose? tyre pressure?).
2. Rebuild and re-test (Unity closed):
   ```powershell
   powershell -ExecutionPolicy Bypass -File unity\run_feasibility.ps1 -Variants tuned
   ```
   `prepare` applies `total_mass_kg` and `center_of_mass_offset_m` to the URDF data (recorded, and undone by `rebuild`). Unity reads the other values at start-up.
3. Commit the config change, so everyone tests against the same rover.

To try a value without editing the config, start Unity with an override, e.g. `-param wheel_max_torque_nm=22` (see `unity/sensitivity_sweep.ps1`).

## Procedures

### 1. Total mass and horizontal centre of mass: `total_mass_kg`, `center_of_mass_offset_m` [x, y]
**Tools:** 6 bathroom or platform scales (or 3, one per bogie pair), level floor, tape measure.
1. Put the rover in its test configuration (battery in, arm in its zero/stowed pose, as in the URDF).
2. Put one scale under each wheel; check all wheels sit level (shim the scales if needed).
3. Record each wheel's reading *Wᵢ* (kg). Total mass *M* = Σ *Wᵢ*.
4. Centre of mass along the rover (*x*, forward) and across it (*y*, left), measured from the base origin:
   *x* = Σ *Wᵢ·xᵢ* / *M*, *y* = Σ *Wᵢ·yᵢ* / *M*. The wheel contact positions *xᵢ, yᵢ* are in `urdf-pipeline/output/tuned/manifest.json` (`reference_poses`, the `ros` block of each wheel link), or measure them with a tape.
5. `center_of_mass_offset_m` = measured − CAD. The CAD centre of mass is `physics.center_of_mass.ros` in the manifest: currently (0.018, −0.016, 0.310) m from the base origin.

### 2. Centre-of-mass height: `center_of_mass_offset_m` [z]
This sets the tip-over angle (currently 37°) and the braking stability.
**Tools:** the scales from step 1, a jack or blocks, angle gauge (a phone works).
1. Lock the bogie pivots (strap or wedge them) so the body can't rotate on them.
2. With the rear wheels still on their scales, lift the front wheels by a measured height, to a tilt angle *θ* of 15–25°.
3. Read the rear-wheel total before (*W_level*) and after (*W_tilt*).
4. Height of the centre of mass above the wheel-axle line: *h* = *L* · (*W_tilt* − *W_level*) / (*M* · tan *θ*), where *L* is the horizontal distance between the front and rear axles when level. Height above the ground = *h* + wheel radius.
5. Enter the difference from the CAD value (0.50 m above the ground) as the *z* of `center_of_mass_offset_m`.

### 3. Tyre friction: `tyre_mu_<surface>_static` / `_dynamic`
**Tools:** spring scale or force gauge (≥ 500 N), tow strap. Surfaces: concrete, loose sand, rock (or the real test site).
1. Hold the wheels: set the drive to hold zero speed (powered, at 0) so the tyres slide rather than roll.
2. Pull the rover horizontally with the force gauge, slowly increasing the force.
3. Static μ = peak force just before it moves ÷ (*M* · 9.81). Dynamic μ = steady force while sliding ÷ (*M* · 9.81).
4. Repeat 3× per surface and use the average; enter the spread as the uncertainty.

### 4. Wheel torque limit: `wheel_max_torque_nm`
**Tools:** laptop with `odrivetool` (or the CAN parameter scripts in `software/can_odrive_configure/`).
1. For each drive ODrive (CAN nodes 0–5), read `axis0.config.motor.current_soft_max` / `current_hard_max` (or `motor.config.current_lim` on older firmware) and `axis0.config.motor.torque_constant` (N·m/A).
2. Wheel torque = current limit × torque constant × 50 (gear ratio) × gearbox efficiency (use 0.8 unless measured).
3. While connected, also record `controller.config.vel_ramp_rate` and `vel_limit`. The simulation assumes 200 turns/s² (from `drive_can_control.py`).

### 5. Effective wheel radius: `wheel_radius_m` / `drive_limits.controller_wheel_radius_m`
**Tools:** tape measure, the rover's `/joint_states` (or the ODrive position counter).
1. Mark the floor and a wheel. Drive straight slowly for at least 10 wheel revolutions (read the wheel-turn count from the encoder).
2. Effective radius = distance ÷ (2π × revolutions). Use it for `wheel_radius_m`. If odometry is tuned against it, also check `rover_drive.yaml`'s `wheel_radius`, currently 0.146 m against a 0.154 m CAD wheel.

### 6. Bogie pivot friction and damping: `passive_joint_friction_nm`, `passive_joint_damping_nms`
**Tools:** spring scale, ruler, phone video (60 fps or more).
1. Lift the rover so the wheels hang free.
2. Friction: hook the spring scale to a bogie at a known distance *d* from the pivot. Pull perpendicular to the bogie and note the force *F* when it starts to rotate. Friction torque = *F* · *d* (subtract the gravity torque by measuring in both directions and averaging).
3. Damping: deflect the bogie about 10° and release. From video, measure how fast the swing decays; viscous damping ≈ 2 · ζ · √(*k* · *I*), from the logarithmic decrement (ask the simulation team to help fit it).

### 7. Arm torque and speed limits: `arm_max_torque_nm`, `joint_limits`
1. `odrivetool` on the arm ODrives (CAN `can_arm`, nodes 0–5): current limit × torque constant × gear ratio (400, 640, 393.6, 120, 370, 100 from the URDF `ros2_control` block) = joint torque limit.
2. The shoulder's speed limit is marked *"should be by measured"* in `joint_limits.yaml`: time a full-speed shoulder move and update both files.

### 8. Sensors: `imu_*`, `gps_*`, camera values
- **IMU / GPS noise:** record 5 minutes stationary (`ros2 bag record /imu/data /gps/raw`) and use the standard deviation of each axis.
- **Camera intrinsics:** `ros2 topic echo --once /camera/d455/color/camera_info`.

## Validation drive (when the rover is available)
Record a short drive so the simulation can be checked against reality (phase 3):
```bash
ros2 bag record /cmd_vel /drive_controller/cmd_vel_unstamped /joint_states /imu/data /odometry/local /gps/raw /unilidar/imu
```
Drive on flat concrete:
1. 5 m straight at 1 m/s, then stop.
2. 5 m straight at full speed, then stop.
3. Two 360° turns on the spot, one each way.
4. Drive up and down a measured ramp.
5. 30 s standing still.

Note the surface, battery level and tyre condition. The bag is replayed into Unity and the two are compared.
