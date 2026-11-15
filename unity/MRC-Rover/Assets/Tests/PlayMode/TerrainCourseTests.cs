// Drives the rover over each feature of the test course and across the Utah desert terrain.
// A test FAILS only if the physics becomes unstable (NaN / explosion). Whether the rover actually
// crossed each feature is a capability measurement, written to FeasibilityResults/<variant>/terrain.json
// and shown in the run_feasibility summary.
using System.Linq;
using MRC.Environment;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace MRC.Feasibility.Tests
{
    public class TerrainCourseTests
    {
        static readonly float Dt = PhysicsTuning.Timestep;
        const float Speed = 0.5f;               // m/s
        const float LaneTimeout = 60f;          // s (20 m at 0.5 m/s is 40 s; slack for slow obstacles)
        const float DesertSeconds = 90f;
        const float StuckWindow = 6f;           // s without 10 cm of progress = stuck

        public static readonly string[] Lanes = { "ramp_10", "ramp_20", "ramp_30", "step_05", "step_10", "step_15", "rubble", "rough" };

        static string variant;
        static JObject manifest;
        static GameObject prefab;
        static JObject results;

        GameObject world;
        RoverRig rig;
        SimulationMode previousMode;

        [OneTimeSetUp]
        public void LoadRobot()
        {
            (variant, manifest, prefab) = TestRobot.Load();
            results = new JObject
            {
                ["variant"] = variant, ["speed_mps"] = Speed, ["physics_settings"] = PhysicsTuning.Describe(),
                ["lanes"] = new JObject(),
            };
        }

        [OneTimeTearDown]
        public void SaveResults()
        {
            if (results != null) FeasibilityPaths.WriteResult(variant, "terrain.json", results);
        }

        [SetUp]
        public void CreateWorld()
        {
            previousMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            Physics.gravity = new Vector3(0f, -9.81f, 0f);
            world = new GameObject("TestWorld");
        }

        [TearDown]
        public void DestroyWorld()
        {
            rig?.Destroy();
            rig = null;
            Object.DestroyImmediate(world);
            Physics.simulationMode = previousMode;
        }

        [TestCaseSource(nameof(Lanes))]
        public void CourseLane(string laneName)
        {
            var info = EnvironmentGenerator.Build(world.transform, desert: false, course: true);
            var lane = info.Lanes.First(l => l.Name == laneName);
            var run = DriveStraight(lane.Start, 0f, lane.EndZ - lane.Start.z, LaneTimeout);
            run["description"] = lane.Description;
            ((JObject)results["lanes"])[laneName] = run;
            AssertStable(run);
        }

        [Test]
        public void DesertTraverse()
        {
            var info = EnvironmentGenerator.Build(world.transform, desert: true, course: true);
            var run = DriveStraight(info.DesertStart, 90f, float.PositiveInfinity, DesertSeconds);
            results["desert"] = run;
            AssertStable(run);
        }

        /// <summary>
        /// Spawn, settle, then drive along `yaw` at Speed, steering to hold the start line, until
        /// `goal` metres of progress, timeout, or stuck. Returns the measurements.
        /// </summary>
        JObject DriveStraight(Vector3 start, float yaw, float goal, float timeout)
        {
            rig = new RoverRig(manifest);
            rig.Spawn(prefab);
            PhysicsTuning.ApplySolverIterations(rig);
            rig.ConfigureDrives();
            rig.PlaceAt(start, yaw);
            rig.Simulate(Mathf.RoundToInt(2f / Dt), Dt); // settle 2 s
            var stepTimer = System.Diagnostics.Stopwatch.StartNew();
            int simulatedSteps = 0;

            var (wheels, track, radius) = rig.WheelGeometry();
            Vector3 dir = Quaternion.Euler(0f, yaw, 0f) * Vector3.forward;
            Vector3 right = Vector3.Cross(Vector3.up, dir);
            Vector3 origin = rig.Root.transform.position;
            float startHeight = origin.y;
            float progress = 0f, maxTilt = 0f, maxPitch = 0f, maxLinkSpeed = 0f, minH = startHeight, maxH = startHeight;
            float lastProgressMark = 0f, lastProgressTime = 0f, t = 0f;
            bool nan = false, stuck = false, reached = false;

            int steps = Mathf.RoundToInt(timeout / Dt);
            for (int i = 0; i < steps; i++)
            {
                Vector3 pos = rig.Root.transform.position;
                Vector3 fwd = Vector3.ProjectOnPlane(rig.Root.transform.forward, Vector3.up).normalized;
                float heading = Vector3.SignedAngle(dir, fwd, Vector3.up);          // + = rotated right
                float lateral = Vector3.Dot(pos - origin, right);                   // + = right of the line
                float desired = -Mathf.Atan2(lateral, 2f) * Mathf.Rad2Deg;          // steer back to the line
                float turn = Mathf.Clamp(2f * (heading - desired) * Mathf.Deg2Rad, -0.6f, 0.6f); // + = turn left
                rig.Drive(Speed, turn, track, radius, wheels);

                rig.Simulate(1, Dt);
                simulatedSteps++;
                t += Dt;
                if (rig.AnyNaN()) { nan = true; break; }
                pos = rig.Root.transform.position;
                progress = Vector3.Dot(pos - origin, dir);
                maxTilt = Mathf.Max(maxTilt, rig.TiltDegrees());
                maxPitch = Mathf.Max(maxPitch, Mathf.Abs(Mathf.DeltaAngle(0f, rig.Root.transform.eulerAngles.x)));
                maxLinkSpeed = Mathf.Max(maxLinkSpeed, rig.MaxLinkSpeed());
                minH = Mathf.Min(minH, pos.y);
                maxH = Mathf.Max(maxH, pos.y);
                if (progress >= goal) { reached = true; break; }
                if (progress > lastProgressMark + 0.1f) { lastProgressMark = progress; lastProgressTime = t; }
                else if (t - lastProgressTime > StuckWindow) { stuck = true; break; }
            }

            stepTimer.Stop();
            float tilt = nan ? float.NaN : rig.TiltDegrees();
            return new JObject
            {
                ["ms_per_step"] = simulatedSteps > 0 ? Round((float)(stepTimer.Elapsed.TotalMilliseconds / simulatedSteps), 4) : 0f,
                ["crossed"] = reached,
                ["stuck"] = stuck,
                ["flipped"] = !nan && tilt > 70f,
                ["nan"] = nan,
                ["progress_m"] = Round(progress),
                ["goal_m"] = float.IsInfinity(goal) ? null : (JToken)Round(goal),
                ["seconds"] = Round(t, 1),
                ["avg_speed_mps"] = t > 0 ? Round(progress / t) : 0,
                ["max_tilt_deg"] = Round(maxTilt, 1),
                ["max_pitch_deg"] = Round(maxPitch, 1),
                ["elevation_change_m"] = Round(maxH - minH),
                ["max_link_speed_mps"] = Round(maxLinkSpeed),
                ["final_lateral_offset_m"] = nan ? null : (JToken)Round(Vector3.Dot(rig.Root.transform.position - origin, right)),
            };
        }

        static void AssertStable(JObject run)
        {
            Assert.IsFalse((bool)run["nan"], "Physics produced NaN (simulation exploded).");
            Assert.Less((float)run["max_link_speed_mps"], 20f, "A link moved faster than 20 m/s (physics instability).");
        }

        static float Round(float v, int digits = 3) => (float)System.Math.Round(v, digits);
    }
}
