using System.Linq;
using MRC.Feasibility;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace MRC.Environment
{
    /// <summary>
    /// Connects the simulated rover to ROS 2 through Unity's ROS-TCP-Connector
    /// (ROS side: ros_tcp_endpoint, see ros2/README.md).
    ///   /cmd_vel  geometry_msgs/Twist   in : linear.x (m/s, + forward), angular.z (rad/s, + left)
    ///                                        -> the rover's drive chain (same acceleration limits as the real rover)
    ///   /odom     nav_msgs/Odometry     out: pose in the "odom" frame (ROS axes: x forward, y left, z up; origin =
    ///                                        where the bridge started or the rover was last teleported) and body-frame velocity
    ///   /waypoint_demo/status  std_msgs/String  in : progress text from the demo node (shown on the HUD, read by the test)
    /// Safety: if /cmd_vel goes quiet for cmdTimeout seconds the rover is commanded to stop (like a real watchdog).
    /// Keyboard driving overrides ROS while a drive key is held.
    /// </summary>
    public class RosRoverBridge : MonoBehaviour
    {
        /// <summary>IP address of ROS (the TCP endpoint); with WSL2 mirrored networking ROS is on localhost.</summary>
        public string rosIp = "127.0.0.1";   // WSL2 mirrored networking: ROS is on localhost
        /// <summary>TCP port of the ROS endpoint (ros_tcp_endpoint's ROS_TCP_PORT).</summary>
        public int rosPort = 10000;
        /// <summary>Topic the rover takes drive commands from (geometry_msgs/Twist).</summary>
        public string cmdVelTopic = "/cmd_vel";
        /// <summary>Topic the rover's odometry is published on (nav_msgs/Odometry).</summary>
        public string odomTopic = "/odom";
        /// <summary>Topic with the demo node's progress text (std_msgs/String).</summary>
        public string statusTopic = "/waypoint_demo/status";
        /// <summary>frame_id of the odometry (the odom frame).</summary>
        public string odomFrame = "odom";
        /// <summary>child_frame_id of the odometry: base_frame_id in the rover's rover_drive.yaml.</summary>
        public string baseFrame = "rover_base_origin";   // base_frame_id in the rover's rover_drive.yaml
        /// <summary>How often /odom is published (Hz); the real drive controller publishes at 30 Hz.</summary>
        public float odomRateHz = 30f;                    // the real drive controller publishes at 30 Hz
        /// <summary>Seconds without /cmd_vel before the rover is told to stop (watchdog).</summary>
        public float cmdTimeout = 0.5f;                   // s without /cmd_vel before stopping

        /// <summary>The rover being driven.</summary>
        public RoverRig Rig { get; private set; }
        /// <summary>Keyboard driving in the scene, or null in tests (then this component steps the drive chain itself).</summary>
        public RoverTeleop Teleop { get; private set; }
        /// <summary>The ROS-TCP-Connector connection.</summary>
        public ROSConnection Connection { get; private set; }

        /// <summary>Number of /cmd_vel messages received.</summary>
        public int CmdCount { get; private set; }
        /// <summary>Number of /odom messages published.</summary>
        public int OdomCount { get; private set; }
        /// <summary>Latest text from /waypoint_demo/status.</summary>
        public string Status { get; private set; } = "";
        /// <summary>Latest commanded forward speed from /cmd_vel (m/s).</summary>
        public float CmdLinear { get; private set; }
        /// <summary>Latest commanded turn rate from /cmd_vel (rad/s, + = left).</summary>
        public float CmdAngular { get; private set; }
        /// <summary>True while ROS is steering (a /cmd_vel arrived within cmdTimeout).</summary>
        public bool ExternalActive => CmdCount > 0 && Time.time - lastCmdTime < cmdTimeout;
        /// <summary>True while the TCP connection to ROS is up without errors.</summary>
        public bool Connected => Connection != null && Connection.HasConnectionThread && !Connection.HasConnectionError;

        /// <summary>Time.time of the last /cmd_vel (for the watchdog).</summary>
        float lastCmdTime = float.NegativeInfinity;
        /// <summary>Time.time of the last /odom publish (for the publish rate).</summary>
        float lastOdomTime = float.NegativeInfinity;
        /// <summary>Whether ROS was driving on the previous step (to stop the rover once when /cmd_vel goes quiet).</summary>
        bool wasActive;
        /// <summary>World position of the odom origin (where the bridge started or the rover was last teleported).</summary>
        Vector3 originPosition;
        /// <summary>Heading of the odom origin (yaw only, so the odom frame stays level).</summary>
        Quaternion originRotation = Quaternion.identity;

        /// <summary>Attach to a rover. `teleop` is optional: without it this component also steps the drive chain.</summary>
        public void Init(RoverRig rig, RoverTeleop teleop = null)
        {
            Rig = rig;
            Teleop = teleop;
            if (Teleop != null) Teleop.Teleported += ResetOrigin;
            ResetOrigin();

            Connection = ROSConnection.GetOrCreateInstance();
            Connection.RosIPAddress = rosIp;   // set before its Start() connects
            Connection.RosPort = rosPort;
            Connection.Subscribe<TwistMsg>(cmdVelTopic, OnCmdVel);
            Connection.Subscribe<StringMsg>(statusTopic, msg => Status = msg.data);
            Connection.RegisterPublisher<OdometryMsg>(odomTopic);
        }

        /// <summary>Make the rover's current position and heading the odom origin (0, 0, heading 0).</summary>
        public void ResetOrigin()
        {
            if (Rig == null) return;
            originPosition = Rig.Root.transform.position;
            originRotation = Quaternion.Euler(0f, Rig.Root.transform.eulerAngles.y, 0f); // heading only, keep odom level
        }

        /// <summary>/cmd_vel callback (runs on Unity's main thread): store the command and refresh the watchdog.</summary>
        void OnCmdVel(TwistMsg msg)   // called on the main thread by ROSConnection
        {
            CmdLinear = (float)msg.linear.x;
            CmdAngular = (float)msg.angular.z;
            lastCmdTime = Time.time;
            CmdCount++;
        }

        /// <summary>Each physics step: pass the ROS command to the drive chain (unless a key is held); stop once if /cmd_vel went quiet; step the controllers when there is no teleop.</summary>
        void FixedUpdate()
        {
            if (Rig == null) return;
            bool manual = Teleop != null && Teleop.ManualActive;
            if (!manual)
            {
                if (ExternalActive) Rig.DriveControl.Command(CmdLinear, CmdAngular);
                else if (wasActive) Rig.DriveControl.Command(0f, 0f);   // watchdog: /cmd_vel stopped
            }
            wasActive = ExternalActive;
            if (Teleop == null)   // no teleop in the scene (tests): step the drive chain here
            {
                Rig.StepControllers(Time.fixedDeltaTime);
            }
        }

        /// <summary>Each frame: publish /odom at odomRateHz.</summary>
        void Update()
        {
            if (Rig == null || Connection == null) return;
            if (Time.time - lastOdomTime < 1f / odomRateHz) return;
            lastOdomTime = Time.time;
            Connection.Publish(odomTopic, BuildOdometry());
            OdomCount++;
        }

        /// <summary>The rover's odometry in ROS conventions (FLU axes, odom frame = origin pose).</summary>
        public OdometryMsg BuildOdometry()
        {
            var root = Rig.Root;
            Quaternion toOdom = Quaternion.Inverse(originRotation);
            Vector3 position = FLU.ConvertFromRUF(toOdom * (root.transform.position - originPosition));
            Quaternion rotation = FLU.ConvertFromRUF(toOdom * root.transform.rotation);
            Quaternion toBody = Quaternion.Inverse(root.transform.rotation);
            Vector3 linear = FLU.ConvertFromRUF(toBody * root.linearVelocity);
            Vector3 angular = FLU.ConvertAngularVelocityFromRUF(toBody * root.angularVelocity);

            double t = Time.timeAsDouble;
            int sec = (int)System.Math.Floor(t);
            var header = new HeaderMsg(new TimeMsg(sec, (uint)((t - sec) * 1e9)), odomFrame);
            return new OdometryMsg(
                header, baseFrame,
                new PoseWithCovarianceMsg(
                    new PoseMsg(new PointMsg(position.x, position.y, position.z),
                                new QuaternionMsg(rotation.x, rotation.y, rotation.z, rotation.w)),
                    new double[36]),
                new TwistWithCovarianceMsg(
                    new TwistMsg(new Vector3Msg(linear.x, linear.y, linear.z), new Vector3Msg(angular.x, angular.y, angular.z)),
                    new double[36]));
        }

        /// <summary>Stop listening to ROS topics when this component is removed.</summary>
        void OnDestroy()
        {
            if (Teleop != null) Teleop.Teleported -= ResetOrigin;
            if (Connection != null)
            {
                Connection.Unsubscribe(cmdVelTopic);
                Connection.Unsubscribe(statusTopic);
            }
        }

        /// <summary>True if Unity was started with -ros on the command line.</summary>
        public static bool RequestedOnCommandLine() =>
            System.Environment.GetCommandLineArgs().Any(a => string.Equals(a, "-ros", System.StringComparison.OrdinalIgnoreCase));
    }
}
