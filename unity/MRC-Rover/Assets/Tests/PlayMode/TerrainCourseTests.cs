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
    /// <summary>Drives over each course lane and across the desert; fails only on instability. Results in terrain.json.</summary>
    public class TerrainCourseTests
    {
        /// <summary>Physics step: 0.01 s unless -dt is given.</summary>
        static readonly float Dt = PhysicsTuning.Timestep;
        /// <summary>Driving speed on the course and in the desert (m/s).</summary>
        const float Speed = 0.5f;               // m/s
        /// <summary>Longest a lane may take (s); 20 m at 0.5 m/s is 40 s.</summary>
        const float LaneTimeout = 60f;          // s (20 m at 0.5 m/s is 40 s; slack for slow obstacles)
        /// <summary>How long the desert run drives (s).</summary>
        const float DesertSeconds = 90f;
        /// <summary>Seconds without 10 cm of progress that count as stuck.</summary>
        const float StuckWindow = 6f;           // s without 10 cm of progress = stuck

        /// <summary>Lane names, one test case each.</summary>
        public static readonly string[] Lanes = { "ramp_10", "ramp_20", "ramp_30", "step_05", "step_10", "step_15", "rubble", "rough" };

        /// <summary>Robot variant under test.</summary>
        static string variant;
        /// <summary>The variant's pipeline manifest.</summary>
        static JObject manifest;
        /// <summary>The variant's imported prefab.</summary>
        static GameObject prefab;
        /// <summary>Measurements written to terrain.json.</summary>
        static JObject results;

        /// <summary>The generated course and desert.</summary>
        GameObject world;
        /// <summary>The rover spawned for the current run.</summary>
        RoverRig rig;
        /// <summary>Physics simulation mode before the test (restored afterwards).</summary>
        SimulationMode previousMode;

        /// <summary>Once before all tests: load the robot and start the results.</summary>
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

        /// <summary>Once after all tests: write terrain.json.</summary>
        [OneTimeTearDown]
        public void SaveResults()
        {
            if (results != null) FeasibilityPaths.WriteResult(variant, "terrain.json", results);
        }

        /// <summary>Before each test: manual physics stepping and a fresh course + desert.</summary>
        [SetUp]
        public void CreateWorld()
        {
            previousMode = Physics.simulationMode;
            Physics.simulationMode = SimulationMode.Script;
            Physics.gravity = new Vector3(0f, -9.81f, 0f);
            world = new GameObject("TestWorld");
        }

        /// <summary>After each test: remove the rover and world, restore the physics mode.</summary>
        [TearDown]
        public void DestroyWorld()
        {
            rig?.Destroy();
            rig = null;
            Object.DestroyImmediate(world);
            Physics.simulationMode = previousMode;
        }

        /// <summary>Drive one lane from its start until past its end (or stuck / timeout) and record the run.</summary>
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

        /// <summary>Drive east across the desert from DesertStart for DesertSeconds and record the run.</summary>
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
            if (results["parameters_used"] == null)
            {
                results["parameters_used"] = rig.Parameters.Snapshot();
                results["parameter_overrides_applied"] = rig.AppliedOverrides;
            }
            PhysicsTuning.ApplySolverIterations(rig);
            rig.ConfigureDrives();
            rig.PlaceAt(start, yaw);
            rig.Simulate(Mathf.RoundToInt(2f / Dt), Dt); // settle 2 s
            var stepTimer = System.Diagnostics.Stopwatch.StartNew();
            int simulatedSteps = 0;

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
                rig.DriveControl.Command(Speed, turn); // through the real drive chain (accel limits, motor ramps)

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

        /// <summary>Fail only if physics went unstable: NaN, or a link faster than 20 m/s.</summary>
        static void AssertStable(JObject run)
        {
            Assert.IsFalse((bool)run["nan"], "Physics produced NaN (simulation exploded).");
            Assert.Less((float)run["max_link_speed_mps"], 20f, "A link moved faster than 20 m/s (physics instability).");
        }

        /// <summary>Round for readable JSON.</summary>
        static float Round(float v, int digits = 3) => (float)System.Math.Round(v, digits);
    }
}
