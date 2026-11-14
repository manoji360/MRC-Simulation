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
    public class RoverPhysicsTests
    {
        const float Dt = 0.01f;                // 100 Hz physics
        const float SettleSeconds = 5f;
        const float DriveSpeed = 0.5f;         // m/s
        const float DriveSeconds = 10f;
        const float ArmStepDegrees = 20f;
        const float ArmSeconds = 3f;

        static string variant;
        static JObject manifest;
        static GameObject prefab;
        static JObject results;

        RoverRig rig;
        SimulationMode previousMode;

        [OneTimeSetUp]
        public void LoadRobot()
        {
            var current = FeasibilityPaths.CurrentVariant();
            if (current == null) Assert.Ignore("No imported robot. Run unity/run_feasibility first.");
            variant = (string)current["variant"];
            manifest = JObject.Parse(File.ReadAllText(Path.Combine(FeasibilityPaths.ProjectRoot, (string)current["manifest"])));
#if UNITY_EDITOR
            string prefabPath = (string)current["prefab"] ?? UnityEditor.AssetDatabase
                .FindAssets("t:Prefab", new[] { $"Assets/Robots/{variant}" })
                .Select(UnityEditor.AssetDatabase.GUIDToAssetPath)
                .FirstOrDefault(p => Path.GetDirectoryName(p).Replace('\\', '/') == $"Assets/Robots/{variant}");
            prefab = prefabPath == null ? null : UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
#endif
            if (prefab == null) Assert.Ignore($"Prefab for variant '{variant}' not found. Run the import step first.");
            results = new JObject { ["variant"] = variant, ["dt"] = Dt };
        }

        [OneTimeTearDown]
        public void SaveResults()
        {
            if (results != null) FeasibilityPaths.WriteResult(variant, "physics.json", results);
        }

        [SetUp]
        public void SpawnRover()
        {
            previousMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            rig = new RoverRig(manifest);
            rig.Spawn(prefab);
            rig.CreateGround();
        }

        [TearDown]
        public void Cleanup()
        {
            rig?.Destroy();
            Physics.simulationMode = previousMode;
        }

        void Settle()
        {
            rig.ConfigureDrives();
            rig.PlaceAboveGround(0.2f);
            RoverRig.Step(Mathf.RoundToInt(SettleSeconds / Dt), Dt);
        }

        static Vector3 V3(JToken t) => new Vector3((float)t[0], (float)t[1], (float)t[2]);
        static Quaternion Q(JToken t) => new Quaternion((float)t[0], (float)t[1], (float)t[2], (float)t[3]);
        static float R(float v, int digits = 4) => (float)System.Math.Round(v, digits);

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

        [Test, Order(3)]
        public void SettlesOnGround()
        {
            rig.ConfigureDrives();
            rig.PlaceAboveGround(0.2f);
            bool nan = false;
            float peakSpeed = 0f;
            int steps = Mathf.RoundToInt(SettleSeconds / Dt);
            for (int i = 0; i < steps; i++)
            {
                RoverRig.Step(1, Dt);
                if (rig.AnyNaN()) { nan = true; break; }
                peakSpeed = Mathf.Max(peakSpeed, rig.MaxLinkSpeed());
            }
            var suspension = new JObject();
            foreach (var joint in rig.JointsWithRole("suspension").Concat(rig.JointsWithRole("passive")))
                if (rig.JointBody(joint) != null) suspension[joint] = R(rig.JointDegrees(joint), 2);
            float finalSpeed = nan ? float.NaN : rig.MaxLinkSpeed();
            float tilt = nan ? float.NaN : rig.TiltDegrees();
            results["settle"] = new JObject
            {
                ["seconds"] = SettleSeconds, ["nan"] = nan,
                ["peak_link_speed_mps"] = R(peakSpeed), ["final_max_link_speed_mps"] = R(finalSpeed),
                ["chassis_tilt_deg"] = R(tilt, 2),
                ["root_height_m"] = R(rig.Root.transform.position.y),
                ["suspension_and_passive_joint_deg"] = suspension,
            };
            Assert.IsFalse(nan, "Physics produced NaN (the simulation exploded).");
            Assert.Less(finalSpeed, 0.05f, "The rover is still moving after settling.");
            Assert.Less(tilt, 5f, "The chassis is tilted after settling on flat ground.");
        }

        [Test, Order(4)]
        public void DrivesStraight()
        {
            Settle();
            var wheels = rig.JointsWithRole("wheel").Where(j => rig.JointBody(j) != null).ToList();
            Assert.IsNotEmpty(wheels, "No wheel joints found in the manifest.");

            Vector3 forward = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            float radius = wheels.Average(w => rig.WheelRadius(w));
            float wheelDegPerSec = DriveSpeed / radius * Mathf.Rad2Deg;
            foreach (var wheel in wheels)
                rig.SetWheelSpeed(wheel, rig.ForwardSign(wheel, forward) * wheelDegPerSec);

            Vector3 start = rig.Root.transform.position;
            bool nan = false;
            int steps = Mathf.RoundToInt(DriveSeconds / Dt);
            for (int i = 0; i < steps && !nan; i++)
            {
                RoverRig.Step(1, Dt);
                nan = rig.AnyNaN();
            }
            Vector3 moved = rig.Root.transform.position - start;
            float along = Vector3.Dot(moved, forward);
            float sideways = Vector3.Dot(moved, Vector3.Cross(Vector3.up, forward));
            Vector3 heading = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            float headingDrift = Vector3.Angle(forward, heading);
            float expected = DriveSpeed * DriveSeconds;
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

        [Test, Order(5)]
        public void ArmReachesTargets()
        {
            Settle();
            var arm = rig.JointsWithRole("arm").Where(j => rig.JointBody(j) != null).ToList();
            if (arm.Count == 0) Assert.Ignore("No arm joints in this robot.");
            var perJoint = new JObject();
            float worst = 0f;
            int steps = Mathf.RoundToInt(ArmSeconds / Dt);
            foreach (var joint in arm)
            {
                var drive = rig.Joints[joint]["unity_drive"];
                float upper = (float?)drive["upper_limit"] ?? float.MaxValue;
                float target = Mathf.Min(ArmStepDegrees, upper - 1f);
                rig.SetArmTarget(joint, target);
                RoverRig.Step(steps, Dt);
                float reached = rig.JointDegrees(joint);
                rig.SetArmTarget(joint, 0f);
                RoverRig.Step(steps, Dt);
                float back = rig.JointDegrees(joint);
                float error = Mathf.Max(Mathf.Abs(reached - target), Mathf.Abs(back));
                worst = Mathf.Max(worst, error);
                perJoint[joint] = new JObject { ["target_deg"] = R(target, 2), ["reached_deg"] = R(reached, 2), ["returned_deg"] = R(back, 2) };
            }
            results["arm"] = new JObject { ["joints"] = perJoint, ["worst_error_deg"] = R(worst, 3), ["seconds_per_move"] = ArmSeconds };
            Assert.Less(worst, 2f, $"An arm joint missed its target by {worst:F2} deg.");
        }

        [Test, Order(6)]
        public void PerformanceBudget()
        {
            Settle();
            var wheels = rig.JointsWithRole("wheel").Where(j => rig.JointBody(j) != null).ToList();
            Vector3 forward = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
            foreach (var wheel in wheels) rig.SetWheelSpeed(wheel, rig.ForwardSign(wheel, forward) * 90f);
            RoverRig.Step(100, Dt); // warm up

            const int samples = 1000;
            var timer = Stopwatch.StartNew();
            RoverRig.Step(samples, Dt);
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
