// Unity-as-physics-backend feasibility tests for a robot imported from urdf_pipeline output.
// Run via unity/run_feasibility.ps1 (or the Test Runner window, PlayMode tab).
// Every test records its measurements in FeasibilityResults/<variant>/physics.json,
// whether it passes or fails, so the numbers are always available for the report.
using System.Diagnostics;
using System.IO;
using System.Linq;
using MRC.Feasibility;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace MRC.Feasibility.Tests
{
    /// <summary>Physics tests of the imported robot on flat ground; every measurement goes to physics.json, pass or fail.</summary>
    public class RoverPhysicsTests
    {
        /// <summary>Physics step: 0.01 s (100 Hz) unless -dt is given.</summary>
        static readonly float Dt = PhysicsTuning.Timestep;   // 100 Hz unless -dt is given
        /// <summary>Simulated time to let the rover settle before the driving and arm tests (s).</summary>
        const float SettleSeconds = 5f;        // settle time before the driving / arm tests
        /// <summary>SettlesOnGround: the rover must come to rest within this time (s).</summary>
        const float MaxSettleSeconds = 15f;    // SettlesOnGround: must come to rest within this
        /// <summary>Speed (m/s) below which every real part counts as at rest.</summary>
        const float RestSpeed = 0.05f;         // m/s, every real part
        /// <summary>Time (s) every real part must stay below RestSpeed to count as at rest.</summary>
        const float RestHold = 0.5f;           // s below RestSpeed to count as at rest
        /// <summary>Forward speed for DrivesStraight (m/s).</summary>
        const float DriveSpeed = 0.5f;         // m/s
        /// <summary>How long DrivesStraight drives (s).</summary>
        const float DriveSeconds = 10f;
        /// <summary>Arm test move size (degrees).</summary>
        const float ArmStepDegrees = 20f;
        /// <summary>Time allowed per arm move (s).</summary>
        const float ArmSeconds = 3f;

        /// <summary>Robot variant under test.</summary>
        static string variant;
        /// <summary>The variant's pipeline manifest.</summary>
        static JObject manifest;
        /// <summary>The variant's imported prefab.</summary>
        static GameObject prefab;
        /// <summary>Measurements written to physics.json.</summary>
        static JObject results;

        /// <summary>The rover spawned for the current test.</summary>
        RoverRig rig;
        /// <summary>Physics simulation mode before the test (restored afterwards).</summary>
        SimulationMode previousMode;

        /// <summary>Once before all tests: load the robot and start the results.</summary>
        [OneTimeSetUp]
        public void LoadRobot()
        {
            (variant, manifest, prefab) = TestRobot.Load();
            results = new JObject { ["variant"] = variant, ["dt"] = Dt, ["physics_settings"] = PhysicsTuning.Describe() };
            parametersRecorded = false;
        }

        /// <summary>Once after all tests: write physics.json.</summary>
        [OneTimeTearDown]
        public void SaveResults()
        {
            if (results != null) FeasibilityPaths.WriteResult(variant, "physics.json", results);
        }

        /// <summary>Before each test: manual physics stepping, spawn the rover on a fresh test ground, record the parameter set once.</summary>
        [SetUp]
        public void SpawnRover()
        {
            previousMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            rig = new RoverRig(manifest);
            rig.Spawn(prefab);
            PhysicsTuning.ApplySolverIterations(rig);
            rig.CreateGround();
            if (!parametersRecorded)
            {
                results["parameters_used"] = rig.Parameters.Snapshot();
                results["parameter_overrides_applied"] = rig.AppliedOverrides;
                parametersRecorded = true;
            }
        }

        /// <summary>True once parameters_used has been written for this run.</summary>
        static bool parametersRecorded;

        /// <summary>After each test: remove the rover and ground, restore the physics mode.</summary>
        [TearDown]
        public void Cleanup()
        {
            rig?.Destroy();
            Physics.simulationMode = previousMode;
        }

        /// <summary>Configure drives, drop from 0.2 m and settle for SettleSeconds.</summary>
        void Settle()
        {
            rig.ConfigureDrives();
            rig.PlaceAboveGround(0.2f);
            rig.Simulate(Mathf.RoundToInt(SettleSeconds / Dt), Dt);
        }

        /// <summary>[x, y, z] JSON array -&gt; Vector3.</summary>
        static Vector3 V3(JToken t) => new Vector3((float)t[0], (float)t[1], (float)t[2]);
        /// <summary>[x, y, z, w] JSON array -&gt; Quaternion.</summary>
        static Quaternion Q(JToken t) => new Quaternion((float)t[0], (float)t[1], (float)t[2], (float)t[3]);
        /// <summary>Round for readable JSON.</summary>
        static float R(float v, int digits = 4) => (float)System.Math.Round(v, digits);

        /// <summary>Every link is within 1 mm and 0.5 deg of the manifest's reference_poses (proves the ROS -&gt; Unity conversion).</summary>
        [Test, Order(1)]
        public void CoordinatesMatchManifest()
        {
            // Proves the ROS -> Unity conversion: link poses (relative to the root link) right after
            // import must equal the pipeline's reference_poses.unity values.
            Transform root = rig.Links[rig.RootLinkName].transform;
            float maxPos = 0f, maxAngle = 0f;
            int matched = 0, missing = 0;
            string worst = null;
            foreach (var pose in ((JObject)manifest["reference_poses"]).Properties())
            {
                if (!rig.Links.TryGetValue(pose.Name, out var body)) { missing++; continue; }
                Vector3 actual = Quaternion.Inverse(root.rotation) * (body.transform.position - root.position);
                Quaternion actualRot = Quaternion.Inverse(root.rotation) * body.transform.rotation;
                float posError = (actual - V3(pose.Value["unity"]["position"])).magnitude;
                float angError = Quaternion.Angle(actualRot, Q(pose.Value["unity"]["rotation_quaternion_xyzw"]));
                if (posError > maxPos) { maxPos = posError; worst = pose.Name; }
                maxAngle = Mathf.Max(maxAngle, angError);
                matched++;
            }
            results["coordinates"] = new JObject
            {
                ["links_compared"] = matched, ["links_not_in_unity"] = missing,
                ["max_position_error_mm"] = R(maxPos * 1000f, 3), ["max_angle_error_deg"] = R(maxAngle, 3),
                ["worst_link"] = worst,
            };
            Assert.Greater(matched, 0, "No links matched between Unity and the manifest.");
            Assert.LessOrEqual(maxPos, 0.001f, $"Link '{worst}' is {maxPos * 1000f:F2} mm from its expected position.");
            Assert.LessOrEqual(maxAngle, 0.5f, $"A link is rotated {maxAngle:F2} deg from its expected rotation.");
        }

        /// <summary>Unity's total mass is within 1 % of the manifest's expected mass.</summary>
        [Test, Order(2)]
        public void MassMatchesUrdf()
        {
            // URDF mass plus the pipeline's tiny placeholder masses for links without <inertial>.
            float urdfMass = (float)manifest["physics"]["total_mass_kg"];
            float expected = (float?)manifest["physics"]["unity_expected_mass_kg"] ?? urdfMass;
            float actual = rig.TotalMass();
            var invented = new JObject();
            foreach (var link in manifest["physics"]["links_without_mass"])
                if (rig.Links.TryGetValue((string)link, out var body)) invented[(string)link] = R(body.mass);
            results["mass"] = new JObject
            {
                ["urdf_total_kg"] = R(urdfMass), ["unity_expected_kg"] = R(expected), ["unity_total_kg"] = R(actual),
                ["difference_percent"] = R(100f * (actual - expected) / expected, 2),
                ["unity_mass_of_links_without_urdf_mass"] = invented,
            };
            Assert.That(actual, Is.EqualTo(expected).Within(1).Percent,
                $"Unity total mass {actual:F2} kg differs from the expected {expected:F2} kg (URDF {urdfMass:F2} kg).");
        }

        /// <summary>Dropped from 0.2 m: no NaN, comes to rest within MaxSettleSeconds, level, linkages and springs hold.</summary>
        [Test, Order(3)]
        public void SettlesOnGround()
        {
            rig.ConfigureDrives();
            rig.PlaceAboveGround(0.2f);
            bool nan = false;
            float peakSpeed = 0f;
            // The rover sways on its free bogies after the drop (URDF pivot damping only), so measure how long
            // it takes to come to rest: every real part below RestSpeed for RestHold seconds.
            float settleTime = float.NaN, quietSince = 0f, t = 0f;
            int steps = Mathf.RoundToInt(MaxSettleSeconds / Dt);
            for (int i = 0; i < steps; i++)
            {
                rig.Simulate(1, Dt);
                t += Dt;
                if (rig.AnyNaN()) { nan = true; break; }
                float speed = rig.MaxLinkSpeed(realPartsOnly: true);
                peakSpeed = Mathf.Max(peakSpeed, rig.MaxLinkSpeed());
                if (speed >= RestSpeed) quietSince = t;
                else if (t - quietSince >= RestHold) { settleTime = quietSince; break; }
            }
            var suspension = new JObject();
            foreach (var joint in rig.JointsWithRole("suspension").Concat(rig.JointsWithRole("passive")))
                if (rig.JointBody(joint) != null) suspension[joint] = R(rig.JointDegrees(joint), 2);
            // Real parts only: massless URDF links get a 1 g placeholder, and a 1 g body jitters on any drive.
            float finalSpeed = nan ? float.NaN : rig.MaxLinkSpeed(realPartsOnly: true);
            float finalSpeedAll = nan ? float.NaN : rig.MaxLinkSpeed();
            float tilt = nan ? float.NaN : rig.TiltDegrees();
            // How well each linkage (e.g. four-bar) holds: follower angle vs multiplier * leader + offset.
            var couplings = new JObject();
            float worstCoupling = 0f;
            foreach (var c in rig.Couplings)
            {
                float error = Mathf.Abs(rig.JointDegrees(c.Follower) - (c.Multiplier * rig.JointDegrees(c.Leader) + c.OffsetDegrees));
                worstCoupling = Mathf.Max(worstCoupling, error);
                couplings[c.Follower] = new JObject { ["leader"] = c.Leader, ["multiplier"] = c.Multiplier, ["error_deg"] = R(error, 3),
                                                     ["drive_torque_nm"] = R(rig.JointBody(c.Follower).driveForce[0], 3),
                                                     ["reaction_moved_to"] = c.ReactionTo != null ? c.ReactionTo.name : null };
            }
            // Springs (suspension shocks): how far each sags from its rest angle under the rover's weight.
            var springs = new JObject();
            float worstTravelUsed = 0f;
            foreach (var sp in rig.Springs)
            {
                float sag = Mathf.Abs(rig.JointDegrees(sp.Joint) - sp.RestDegrees);
                // Fraction of the travel from rest to the joint stop on the side it sagged towards.
                var limits = rig.Joints[sp.Joint]["unity_drive"];
                float stop = rig.JointDegrees(sp.Joint) < sp.RestDegrees
                    ? Mathf.Abs(sp.RestDegrees - ((float?)limits["lower_limit"] ?? -180f))
                    : Mathf.Abs(((float?)limits["upper_limit"] ?? 180f) - sp.RestDegrees);
                float travelUsed = stop > 0f ? sag / stop : 1f;
                worstTravelUsed = Mathf.Max(worstTravelUsed, travelUsed);
                springs[sp.Joint] = new JObject { ["rest_deg"] = sp.RestDegrees, ["sag_deg"] = R(sag, 3),
                                                  ["design_max_sag_deg"] = sp.MaxSagDegrees, ["sag_vs_design"] = R(sag / sp.MaxSagDegrees, 2),
                                                  ["travel_used"] = R(travelUsed, 2), ["stiffness_nm_per_rad"] = sp.StiffnessPerRad,
                                                  ["spring_torque_nm"] = R(rig.JointBody(sp.Joint).driveForce[0], 3) };
            }
            results["settle"] = new JObject
            {
                ["seconds_simulated"] = R(t, 2), ["settle_time_s"] = float.IsNaN(settleTime) ? null : (JToken)R(settleTime, 2),
                ["max_settle_seconds"] = MaxSettleSeconds, ["nan"] = nan,
                ["peak_link_speed_mps"] = R(peakSpeed), ["final_max_link_speed_mps"] = R(finalSpeed),
                ["fastest_link"] = nan ? null : rig.FastestLink(realPartsOnly: true),
                ["final_max_speed_incl_placeholders_mps"] = R(finalSpeedAll),
                ["fastest_incl_placeholders"] = nan ? null : rig.FastestLink(),
                ["moving_links_mps"] = nan ? null : new JObject(rig.Links.Values.Where(b => b.linearVelocity.magnitude > 0.02f)
                    .Select(b => new JProperty(b.name, R(b.linearVelocity.magnitude, 3)))),
                ["arm_joint_speed_dps"] = nan ? null : new JObject(rig.JointsWithRole("arm").Where(j => rig.JointBody(j) != null && rig.JointBody(j).dofCount > 0)
                    .Select(j => new JProperty(j, R(rig.JointBody(j).jointVelocity[0] * Mathf.Rad2Deg, 2)))),
                ["root_speed_mps"] = nan ? float.NaN : R(rig.Root.linearVelocity.magnitude),
                ["chassis_tilt_deg"] = R(tilt, 2),
                ["root_height_m"] = R(rig.Root.transform.position.y),
                ["suspension_and_passive_joint_deg"] = suspension,
                ["couplings"] = couplings,
                ["worst_coupling_error_deg"] = R(worstCoupling, 3),
                ["springs"] = springs,
                ["linkage_collider_pairs_ignored"] = rig.IgnoredLinkagePairs,
                ["assembled_overlap_pairs_ignored"] = rig.IgnoredOverlapPairs,
                ["assembled_overlap_bodies"] = rig.IgnoredOverlapBodies,
                ["importer_dynamics_reset"] = rig.ImporterDynamicsReset,
                ["coupling_reaction_sign"] = RoverRig.CouplingReactionSign,
                ["locked_joints"] = new JArray(rig.LockedJoints),
            };
            Assert.IsFalse(nan, "Physics produced NaN (the simulation exploded).");
            Assert.IsFalse(float.IsNaN(settleTime),
                $"The rover did not come to rest within {MaxSettleSeconds} s (still {finalSpeed:F3} m/s, part {rig.FastestLink(true)}).");
            Assert.Less(tilt, 5f, "The chassis is tilted after settling on flat ground.");
            Assert.Less(worstCoupling, 1f, "A coupled linkage (four-bar) drifted more than 1 degree from its leader.");
            // "auto" springs are an estimate (a six-wheeler on flat ground is over-constrained, so how the weight
            // splits between a bogie's wheels - and the sag - varies). The firm requirement: the suspension
            // rides on its spring, not on its joint stop.
            Assert.Less(worstTravelUsed, 0.5f, "A suspension spring sagged more than half-way to its joint stop (spring too soft?).");
        }

        /// <summary>Drives at DriveSpeed for DriveSeconds through the drive chain: distance and heading drift within limits.</summary>
        [Test, Order(4)]
        public void DrivesStraight()
        {
            Settle();
            var wheels = rig.JointsWithRole("wheel").Where(j => rig.JointBody(j) != null).ToList();
            Assert.IsNotEmpty(wheels, "No wheel joints found in the manifest.");

            Vector3 forward = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            float radius = wheels.Average(w => rig.WheelRadius(w));
            rig.DriveControl.Command(DriveSpeed, 0f); // through the drive chain, like the real rover

            Vector3 start = rig.Root.transform.position;
            bool nan = false;
            float expected = 0f; // distance the controller asked for (includes its acceleration ramp)
            int steps = Mathf.RoundToInt(DriveSeconds / Dt);
            for (int i = 0; i < steps && !nan; i++)
            {
                rig.Simulate(1, Dt);
                expected += rig.DriveControl.Linear * Dt;
                nan = rig.AnyNaN();
            }
            Vector3 moved = rig.Root.transform.position - start;
            float along = Vector3.Dot(moved, forward);
            float sideways = Vector3.Dot(moved, Vector3.Cross(Vector3.up, forward));
            Vector3 heading = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            float headingDrift = Vector3.Angle(forward, heading);
            results["drive"] = new JObject
            {
                ["wheels"] = wheels.Count, ["wheel_radius_m"] = R(radius),
                ["commanded_speed_mps"] = DriveSpeed, ["seconds"] = DriveSeconds, ["nan"] = nan,
                ["expected_distance_m"] = R(expected), ["distance_forward_m"] = R(along),
                ["distance_sideways_m"] = R(sideways), ["heading_drift_deg"] = R(headingDrift, 2),
                ["slip_percent"] = R(100f * (1f - along / expected), 1),
                ["chassis_tilt_deg"] = R(rig.TiltDegrees(), 2),
            };
            Assert.IsFalse(nan, "Physics produced NaN while driving.");
            Assert.That(along, Is.InRange(0.7f * expected, 1.3f * expected),
                $"Drove {along:F2} m forward; expected about {expected:F2} m.");
            Assert.Less(headingDrift, 10f, $"Heading drifted {headingDrift:F1} deg while driving straight.");
        }

        /// <summary>Full command from rest and back to zero: measured acceleration and braking match drive_limits; no tip-over.</summary>
        [Test, Order(7)]
        public void AccelerationAndBraking()
        {
            Settle();
            var dc = rig.DriveControl;
            float top = dc.MaxLinearSpeed ?? 1.0f;
            float expectedAccel = dc.LinearAccel ?? float.NaN, expectedDecel = dc.LinearDecel ?? float.NaN;
            if (dc.WheelAccelRevS2.HasValue)  // the motor ramp can be the tighter limit
            {
                float rim = dc.WheelAccelRevS2.Value * 2f * Mathf.PI * (dc.ControllerRadius ?? rig.WheelGeometry().radius);
                expectedAccel = float.IsNaN(expectedAccel) ? rim : Mathf.Min(expectedAccel, rim);
                expectedDecel = float.IsNaN(expectedDecel) ? rim : Mathf.Min(expectedDecel, rim);
            }
            Vector3 forward = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            float Speed() => Vector3.Dot(rig.Root.linearVelocity, forward);
            float Pitch() => Mathf.DeltaAngle(0f, rig.Root.transform.eulerAngles.x);

            // Time between crossing `from` and `to` of `top` (fractions), stepping until done or `limit` s.
            (float seconds, float peakPitchChange, float maxTilt, bool nan) Run(System.Func<float, bool> done, float limit,
                                                                                    float lo, float hi, bool rising)
            {
                float t = 0f, tLo = float.NaN, tHi = float.NaN, pitch0 = Pitch(), peak = 0f, tilt = 0f;
                for (int i = 0; i < Mathf.RoundToInt(limit / Dt); i++)
                {
                    rig.Simulate(1, Dt);
                    t += Dt;
                    if (rig.AnyNaN()) return (float.NaN, float.NaN, float.NaN, true);
                    float v = Speed() / top;
                    if (rising) { if (float.IsNaN(tLo) && v >= lo) tLo = t; if (float.IsNaN(tHi) && v >= hi) tHi = t; }
                    else { if (float.IsNaN(tHi) && v <= hi) tHi = t; if (float.IsNaN(tLo) && v <= lo) tLo = t; }
                    peak = Mathf.Max(peak, Mathf.Abs(Mathf.DeltaAngle(pitch0, Pitch())));
                    tilt = Mathf.Max(tilt, rig.TiltDegrees());
                    if (done(t)) break;
                }
                return (Mathf.Abs(tHi - tLo), peak, tilt, false);
            }

            // 1. Full command from rest, through the drive chain.
            dc.Command(top, 0f);
            var up = Run(t => Speed() >= 0.95f * top && t > 1f, 8f, 0.1f, 0.9f, true);
            float accel = 0.8f * top / up.seconds;
            float reached = Speed();
            Run(t => t >= 1f, 1f, 0f, 0f, true);                 // cruise 1 s
            // 2. Command zero: the controller ramps down.
            dc.Command(0f, 0f);
            var down = Run(t => Mathf.Abs(Speed()) < 0.02f && t > 0.5f, 8f, 0.1f, 0.9f, false);
            float decel = 0.8f * top / down.seconds;

            // 3. Reference only: the old behaviour (wheels told to stop instantly) from the same speed.
            dc.Command(top, 0f);
            Run(t => Speed() >= 0.95f * top && t > 1f, 8f, 0f, 0f, true);
            Run(t => t >= 1f, 1f, 0f, 0f, true);
            dc.Release();
            foreach (var w in rig.JointsWithRole("wheel").Where(j => rig.JointBody(j) != null)) rig.SetWheelSpeed(w, 0f);
            var instant = Run(t => Mathf.Abs(Speed()) < 0.02f && t > 0.5f, 8f, 0.1f, 0.9f, false);
            float instantDecel = 0.8f * top / instant.seconds;

            results["acceleration"] = new JObject
            {
                ["drive_limits"] = dc.Describe(),
                ["top_speed_command_mps"] = R(top), ["top_speed_reached_mps"] = R(reached),
                ["expected_accel_mps2"] = R(expectedAccel, 3), ["measured_accel_mps2"] = R(accel, 3),
                ["expected_decel_mps2"] = R(expectedDecel, 3), ["measured_decel_mps2"] = R(decel, 3),
                ["accel_pitch_change_deg"] = R(up.peakPitchChange, 2),
                ["ramped_stop_pitch_change_deg"] = R(down.peakPitchChange, 2), ["ramped_stop_max_tilt_deg"] = R(down.maxTilt, 2),
                ["reference_instant_stop_decel_mps2"] = R(instantDecel, 3),
                ["reference_instant_stop_pitch_change_deg"] = R(instant.peakPitchChange, 2),
                ["reference_instant_stop_max_tilt_deg"] = R(instant.maxTilt, 2),
            };
            Assert.IsFalse(up.nan || down.nan, "Physics produced NaN while accelerating or braking.");
            if (!dc.HasLimits) Assert.Inconclusive("No drive_limits in the manifest: acceleration is not limited (instant).");
            Assert.That(reached, Is.GreaterThan(0.9f * top), $"Reached only {reached:F2} of {top:F2} m/s.");
            Assert.That(accel, Is.EqualTo(expectedAccel).Within(25).Percent, $"Accelerated at {accel:F2} m/s2; expected {expectedAccel:F2}.");
            Assert.That(decel, Is.EqualTo(expectedDecel).Within(25).Percent, $"Braked at {decel:F2} m/s2; expected {expectedDecel:F2}.");
            Assert.Less(down.peakPitchChange, 10f, $"The rover pitched {down.peakPitchChange:F1} deg when stopping.");
            Assert.Less(down.maxTilt, 70f, "The rover tipped over when stopping.");
        }

        /// <summary>Each arm joint reaches +20 deg and back; its speed and timing match the real joint-limit profile.</summary>
        [Test, Order(5)]
        public void ArmReachesTargets()
        {
            Settle();
            var arm = rig.JointsWithRole("arm").Where(j => rig.JointBody(j) != null && rig.JointBody(j).dofCount > 0).ToList();
            if (arm.Count == 0) Assert.Ignore("No arm joints in this robot.");
            var perJoint = new JObject();
            float worst = 0f, worstPeak = 0f, worstTime = 0f;
            string worstPeakJoint = null, worstTimeJoint = null;
            int steps = Mathf.RoundToInt(ArmSeconds / Dt);
            foreach (var joint in arm)
            {
                var drive = rig.Joints[joint]["unity_drive"];
                float upper = (float?)drive["upper_limit"] ?? float.MaxValue;
                float target = Mathf.Min(ArmStepDegrees, upper - 1f);
                float start = rig.JointDegrees(joint);
                rig.SetArmTarget(joint, target);
                // Peak speed of a 50 ms moving average (single-step spikes from contacts would dominate a raw max).
                float peak = 0f, rawPeak = 0f, arrived = float.NaN;
                int window = Mathf.Max(1, Mathf.RoundToInt(0.05f / Dt));
                var recent = new System.Collections.Generic.Queue<float>();
                float sum = 0f;
                for (int i = 0; i < steps; i++)
                {
                    rig.Simulate(1, Dt);
                    float speed = Mathf.Abs(rig.JointBody(joint).jointVelocity[0] * Mathf.Rad2Deg);
                    rawPeak = Mathf.Max(rawPeak, speed);
                    recent.Enqueue(speed); sum += speed;
                    if (recent.Count > window) sum -= recent.Dequeue();
                    if (recent.Count == window) peak = Mathf.Max(peak, sum / window);
                    if (float.IsNaN(arrived) && Mathf.Abs(rig.JointDegrees(joint) - target) < 0.5f) arrived = (i + 1) * Dt;
                }
                float reached = rig.JointDegrees(joint);
                rig.SetArmTarget(joint, 0f);
                rig.Simulate(steps, Dt);
                float back = rig.JointDegrees(joint);
                float error = Mathf.Max(Mathf.Abs(reached - target), Mathf.Abs(back));
                worst = Mathf.Max(worst, error);
                var entry = new JObject { ["target_deg"] = R(target, 2), ["reached_deg"] = R(reached, 2), ["returned_deg"] = R(back, 2),
                                          ["peak_speed_deg_s"] = R(peak, 2), ["raw_peak_speed_deg_s"] = R(rawPeak, 2), ["time_to_target_s"] = float.IsNaN(arrived) ? null : (JToken)R(arrived, 3) };
                if (rig.ArmControl.Manages(joint))
                {
                    // Trapezoidal profile: cruise at vmax if the move is long enough, else a triangle peaking at sqrt(a*d).
                    float d = Mathf.Abs(target - start), v = rig.ArmControl.MaxVelocity(joint), a = rig.ArmControl.MaxAcceleration(joint);
                    float expectedPeak = float.IsInfinity(a) ? v : Mathf.Min(v, Mathf.Sqrt(a * d));
                    float expectedTime = float.IsInfinity(a) ? d / v : (d >= v * v / a ? d / v + v / a : 2f * Mathf.Sqrt(d / a));
                    // "Arrived" = within 0.5 deg, reached before the profile's final slow-down tail of sqrt(2 * 0.5 / a).
                    if (!float.IsInfinity(a)) expectedTime -= Mathf.Sqrt(2f * 0.5f / a);
                    float peakError = Mathf.Abs(peak - expectedPeak) / expectedPeak;
                    float timeError = float.IsNaN(arrived) ? 1f : Mathf.Abs(arrived - expectedTime) / expectedTime;
                    if (peakError > worstPeak) { worstPeak = peakError; worstPeakJoint = joint; }
                    if (timeError > worstTime) { worstTime = timeError; worstTimeJoint = joint; }
                    entry["limit_speed_deg_s"] = R(v, 2);
                    entry["limit_accel_deg_s2"] = float.IsInfinity(a) ? null : (JToken)R(a, 2);
                    entry["expected_peak_deg_s"] = R(expectedPeak, 2);
                    entry["expected_time_s"] = R(expectedTime, 3);
                }
                perJoint[joint] = entry;
            }
            results["arm"] = new JObject
            {
                ["joints"] = perJoint, ["worst_error_deg"] = R(worst, 3), ["seconds_per_move"] = ArmSeconds,
                ["worst_peak_speed_error"] = R(worstPeak, 3), ["worst_peak_speed_joint"] = worstPeakJoint,
                ["worst_motion_time_error"] = R(worstTime, 3), ["worst_motion_time_joint"] = worstTimeJoint,
            };
            Assert.Less(worst, 2f, $"An arm joint missed its target by {worst:F2} deg.");
            Assert.Less(worstPeak, 0.10f, $"'{worstPeakJoint}' peak speed is {worstPeak:P0} off the real joint limit profile.");
            Assert.Less(worstTime, 0.10f, $"'{worstTimeJoint}' took {worstTime:P0} longer/shorter than the real limit profile.");
        }

        /// <summary>Rover held in the air: wheel drives saturate at the torque cap; bogie pivots resist with exactly their friction torque.</summary>
        [Test, Order(8)]
        public void FrictionAndTorque()
        {
            Settle();
            // Hang the rover in the air, held still: wheels and bogies can move freely.
            rig.PlaceAboveGround(1.0f);
            rig.Root.immovable = true;
            rig.Simulate(Mathf.RoundToInt(1f / Dt), Dt);
            float heightAfterHang = rig.LowestPoint();   // should still be ~1 m if the root is really held

            // 1. Wheel torque cap: command an unreachable speed; the drive must saturate at the cap.
            var wheels = rig.JointsWithRole("wheel").Where(j => rig.JointBody(j) != null).ToList();
            foreach (var w in wheels) rig.SetWheelSpeed(w, 1e5f);
            rig.Simulate(Mathf.RoundToInt(0.5f / Dt), Dt);
            var wheelTorques = new JObject();
            float cap = rig.JointBody(wheels[0]).xDrive.forceLimit, worstCap = 0f;
            foreach (var w in wheels)
            {
                float torque = Mathf.Abs(rig.JointBody(w).driveForce[0]);
                wheelTorques[w] = R(torque, 3);
                worstCap = Mathf.Max(worstCap, Mathf.Abs(torque - cap) / cap);
            }
            foreach (var w in wheels) rig.SetWheelSpeed(w, 0f);

            // 2. Passive-joint (bogie) Coulomb friction: push with 3x the friction torque; the joint must move while
            //    the friction drive resists with exactly the friction torque.
            var bogies = new JObject();
            float worstFriction = 0f;
            bool allMoved = true;
            foreach (var pj in rig.PassiveJoints.Where(p => p.Parent != null))
            {
                float before = rig.JointDegrees(pj.Joint), resisting = 0f, peakOmega = 0f;
                // Push away from the nearer joint stop (a hanging bogie can rest against one).
                float lo = (float?)rig.Joints[pj.Joint]["unity_drive"]["lower_limit"] ?? -180f;
                float hi = (float?)rig.Joints[pj.Joint]["unity_drive"]["upper_limit"] ?? 180f;
                float sign = (hi - before) >= (before - lo) ? 1f : -1f;
                for (int i = 0; i < Mathf.RoundToInt(0.3f / Dt); i++)
                {
                    Vector3 push = RoverRig.JointAxisWorld(pj.Body) * (sign * 3f * Mathf.Max(pj.FrictionNm, 0.2f));
                    pj.Body.WakeUp();   // a still, held articulation sleeps; external torques alone do not wake it
                    pj.Body.AddTorque(push);
                    pj.Parent.AddTorque(-push);
                    rig.Simulate(1, Dt);
                    peakOmega = Mathf.Max(peakOmega, Mathf.Abs(pj.Body.jointVelocity[0]));
                    if (Mathf.Abs(pj.Body.jointVelocity[0]) > 0.01f) resisting = Mathf.Abs(pj.Body.driveForce[0]);
                }
                float moved = Mathf.Abs(rig.JointDegrees(pj.Joint) - before);
                // Diagnostic: a large push (20 N*m) must move any free joint.
                bool sleepingBefore = pj.Body.IsSleeping();
                float bigBefore = rig.JointDegrees(pj.Joint);
                for (int i = 0; i < Mathf.RoundToInt(0.2f / Dt); i++)
                {
                    Vector3 push = RoverRig.JointAxisWorld(pj.Body) * (sign * 20f);
                    pj.Body.WakeUp();
                    pj.Body.AddTorque(push);
                    pj.Parent.AddTorque(-push);
                    rig.Simulate(1, Dt);
                    // A bogie hanging against its stop under gravity may need more than 3x friction to start moving;
                    // what matters is the resisting torque while it moves.
                    if (resisting == 0f && Mathf.Abs(pj.Body.jointVelocity[0]) > 0.01f) resisting = Mathf.Abs(pj.Body.driveForce[0]);
                }
                float bigMoved = rig.JointDegrees(pj.Joint) - bigBefore;

                float error = pj.FrictionNm > 0f ? Mathf.Abs(resisting - pj.FrictionNm) / pj.FrictionNm : 0f;
                worstFriction = Mathf.Max(worstFriction, error);
                allMoved &= moved > 0.5f || Mathf.Abs(bigMoved) > 0.5f;
                bogies[pj.Joint] = new JObject { ["friction_nm"] = pj.FrictionNm, ["resisting_torque_nm"] = R(resisting, 4),
                                                 ["moved_deg"] = R(moved, 2), ["damping_nms"] = pj.DampingNms,
                                                 ["start_deg"] = R(before, 2), ["peak_speed_rad_s"] = R(peakOmega, 4),
                                                 ["big_push_moved_deg"] = R(bigMoved, 2), ["sleeping_before_big_push"] = sleepingBefore,
                                                 ["axis_world"] = new JArray(R(RoverRig.JointAxisWorld(pj.Body).x, 3), R(RoverRig.JointAxisWorld(pj.Body).y, 3), R(RoverRig.JointAxisWorld(pj.Body).z, 3)),
                                                 ["body"] = pj.Body.name, ["parent"] = pj.Parent.name,
                                                 ["lower_limit_deg"] = rig.Joints[pj.Joint]["unity_drive"]["lower_limit"],
                                                 ["upper_limit_deg"] = rig.Joints[pj.Joint]["unity_drive"]["upper_limit"] };
                rig.Simulate(Mathf.RoundToInt(1f / Dt), Dt);   // let it come back to rest before the next joint
            }
            rig.Root.immovable = false;

            results["friction_and_torque"] = new JObject
            {
                ["lowest_point_while_hanging_m"] = R(heightAfterHang, 3), ["root_immovable"] = rig.Root.immovable,
                ["wheel_torque_cap_nm"] = cap, ["wheel_saturated_torque_nm"] = wheelTorques, ["worst_wheel_cap_error"] = R(worstCap, 4),
                ["passive_joints"] = bogies, ["worst_friction_error"] = R(worstFriction, 4),
            };
            Assert.Less(worstCap, 0.02f, $"A wheel drive saturated {worstCap:P1} away from its torque cap ({cap} N*m).");
            if (bogies.Count == 0) return;
            Assert.IsTrue(allMoved, "A passive joint did not move even when pushed with 20 N*m.");
            Assert.Less(worstFriction, 0.10f, $"Passive-joint friction torque is {worstFriction:P0} off its configured value.");
        }

        /// <summary>1000 physics steps run faster than real time.</summary>
        [Test, Order(6)]
        public void PerformanceBudget()
        {
            Settle();
            var wheels = rig.JointsWithRole("wheel").Where(j => rig.JointBody(j) != null).ToList();
            Vector3 forward = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            foreach (var wheel in wheels) rig.SetWheelSpeed(wheel, rig.ForwardSign(wheel, forward) * 90f);
            rig.Simulate(100, Dt); // warm up

            const int samples = 1000;
            var timer = Stopwatch.StartNew();
            rig.Simulate(samples, Dt);
            timer.Stop();
            double msPerStep = timer.Elapsed.TotalMilliseconds / samples;
            double realTimeFactor = (Dt * 1000.0) / msPerStep;
            results["performance"] = new JObject
            {
                ["steps"] = samples, ["ms_per_step"] = System.Math.Round(msPerStep, 4),
                ["real_time_factor"] = System.Math.Round(realTimeFactor, 1),
                ["articulation_bodies"] = rig.Links.Count,
            };
            Assert.GreaterOrEqual(realTimeFactor, 1.0, $"Physics runs slower than real time ({realTimeFactor:F2}x).");
        }
    }
}
