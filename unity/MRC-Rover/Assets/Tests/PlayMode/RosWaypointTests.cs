// End-to-end ROS 2 test: the waypoint_follower node (ros2/, running in WSL) drives the simulated rover
// through Unity's ROS-TCP-Connector. Runs only when Unity is started with -ros (otherwise ignored), e.g.
//   1. WSL:  ros2 launch mrc_waypoint_demo waypoint_demo.launch.py
//   2. Win:  unity/run_ros_demo.ps1    (or Unity -batchmode -runTests ... -testFilter RosWaypointTests -ros)
// Results: FeasibilityResults/<variant>/ros.json (messages exchanged, time, path, final position).
using System.Collections;
using MRC.Environment;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace MRC.Feasibility.Tests
{
    /// <summary>End-to-end ROS 2 test: the waypoint_follower node in WSL drives the rover; runs only with -ros.</summary>
    public class RosWaypointTests
    {
        /// <summary>Longest the waypoint run may take (s); the default 5 m square takes about 40 s.</summary>
        const float TimeoutSeconds = 180f;     // the default 5 m square takes ~40 s
        /// <summary>If no /cmd_vel arrives within this many seconds, the ROS side is not running.</summary>
        const float ConnectSeconds = 20f;      // no /cmd_vel by then = ROS side not running

        /// <summary>Settle the rover, connect to ROS, wait for the node to report 'done', record messages and the path to ros.json.</summary>
        [UnityTest]
        public IEnumerator FollowsWaypointsFromRos()
        {
            if (!RosRoverBridge.RequestedOnCommandLine())
            {
                Assert.Ignore("ROS test: start the ROS 2 demo (ros2/README.md) and run Unity with -ros.");
                yield break;
            }
            var (variant, manifest, prefab) = TestRobot.Load();
            var previousMode = Physics.simulationMode;
            var previousDt = Time.fixedDeltaTime;
            Physics.simulationMode = SimulationMode.FixedUpdate;   // real-time physics: ROS runs on the wall clock
            Time.fixedDeltaTime = PhysicsTuning.Timestep;

            var rig = new RoverRig(manifest);
            GameObject host = null;
            var results = new JObject { ["variant"] = variant };
            try
            {
                rig.Spawn(prefab);
                PhysicsTuning.ApplySolverIterations(rig);
                rig.CreateGround();
                rig.ConfigureDrives();
                rig.PlaceAboveGround(0.05f);
                yield return new WaitForSeconds(2f);                 // settle before connecting

                host = new GameObject("RosRoverBridge");
                var bridge = host.AddComponent<RosRoverBridge>();
                bridge.Init(rig);

                float start = Time.time, firstCmd = float.NaN, lastSample = -1f;
                bool sawRunning = false;
                var path = new JArray();
                while (Time.time - start < TimeoutSeconds)
                {
                    if (float.IsNaN(firstCmd) && bridge.CmdCount > 0) firstCmd = Time.time - start;
                    if (float.IsNaN(firstCmd) && Time.time - start > ConnectSeconds) break;
                    sawRunning |= bridge.Status.StartsWith("running");
                    if (sawRunning && bridge.Status.StartsWith("done")) break;   // a fresh run finished
                    if (Time.time - lastSample >= 0.5f)
                    {
                        lastSample = Time.time;
                        var p = bridge.BuildOdometry().pose.pose.position;
                        path.Add(new JArray(System.Math.Round(p.x, 3), System.Math.Round(p.y, 3)));
                    }
                    if (rig.AnyNaN()) break;
                    yield return null;
                }

                var final = bridge.BuildOdometry().pose.pose.position;
                float seconds = Time.time - start;
                results["connected"] = bridge.Connected;
                results["seconds"] = System.Math.Round(seconds, 1);
                results["first_cmd_vel_after_s"] = float.IsNaN(firstCmd) ? null : (JToken)System.Math.Round(firstCmd, 2);
                results["cmd_vel_received"] = bridge.CmdCount;
                results["cmd_vel_rate_hz"] = System.Math.Round(bridge.CmdCount / Mathf.Max(seconds - (float.IsNaN(firstCmd) ? 0f : firstCmd), 1e-3f), 1);
                results["odom_published"] = bridge.OdomCount;
                results["final_odom_xy"] = new JArray(System.Math.Round(final.x, 3), System.Math.Round(final.y, 3));
                results["node_status"] = bridge.Status;
                results["path_xy_every_0_5s"] = path;
                results["nan"] = rig.AnyNaN();
                FeasibilityPaths.WriteResult(variant, "ros.json", results);

                Assert.IsFalse(float.IsNaN(firstCmd),
                    $"No /cmd_vel from ROS within {ConnectSeconds} s. Is 'ros2 launch mrc_waypoint_demo waypoint_demo.launch.py' running?");
                Assert.IsFalse((bool)results["nan"], "Physics produced NaN.");
                Assert.That(bridge.Status, Does.StartWith("done"),
                    $"The waypoint follower did not finish within {TimeoutSeconds} s (last status: '{bridge.Status}').");
            }
            finally
            {
                if (host != null) Object.DestroyImmediate(host);
                rig.Destroy();
                Physics.simulationMode = previousMode;
                Time.fixedDeltaTime = previousDt;
            }
        }
    }
}
