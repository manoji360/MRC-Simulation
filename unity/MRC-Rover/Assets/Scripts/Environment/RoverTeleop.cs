using System.Collections.Generic;
using System.Linq;
using MRC.Feasibility;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MRC.Environment
{
    /// <summary>
    /// Keyboard driving (Input System package):
    ///   W/S or Up/Down  forward / reverse      A/D or Left/Right  turn (skid steer)
    ///   [ / ]           speed scale -/+        Space              stop (ramps down, like the real rover)
    ///   1-9             select arm joint       Q / E              move selected arm joint
    ///   Z               arm to zero pose       R                  reset to the current start point
    ///   T               next test lane         G                  desert start point
    /// </summary>
    public class RoverTeleop : MonoBehaviour
    {
        // Top speeds come from the robot's drive_limits (real controller config); these are used only without them.
        /// <summary>Top speed (m/s) used only when the robot has no drive_limits.</summary>
        public float fallbackMaxSpeed = 0.8f;      // m/s
        /// <summary>Top turn rate (rad/s) used only when the robot has no drive_limits.</summary>
        public float fallbackMaxTurnRate = 0.8f;   // rad/s
        /// <summary>How fast Q/E move the selected arm joint's target (deg/s); the arm itself moves within its real limits.</summary>
        [Range(0.1f, 1f)] public float speedScale = 1f; // like joy_to_drive's linear/angular_vel_scalar
        public float armDegreesPerSecond = 40f;

        /// <summary>The rover being driven.</summary>
        public RoverRig Rig { get; private set; }
        /// <summary>The environment (lanes, start points) for the T / G / R keys.</summary>
        public EnvironmentInfo Info { get; private set; }
        /// <summary>Arm joints selectable with keys 1-9.</summary>
        public List<string> ArmJoints { get; private set; } = new List<string>();
        /// <summary>Index of the selected arm joint in ArmJoints.</summary>
        public int SelectedArmJoint { get; private set; }
        /// <summary>Where the rover is (shown on the HUD).</summary>
        public string Location { get; private set; } = "Start pad";
        /// <summary>Forward speed the keyboard is asking for (m/s).</summary>
        public float CommandSpeed { get; private set; }
        /// <summary>True while a drive key (W/A/S/D, arrows, Space) is held: the keyboard overrides ROS.</summary>
        public bool ManualActive { get; private set; }
        /// <summary>Optional ROS 2 bridge (EnvironmentGenerator "Enable Ros"); it drives when no key is held.</summary>
        public RosRoverBridge Ros;
        /// <summary>Raised after R / T / G teleports (the ROS bridge resets its odom origin).</summary>
        public event System.Action Teleported;
        /// <summary>Turn rate the keyboard is asking for (rad/s, + = left).</summary>
        public float CommandTurn { get; private set; }

        /// <summary>Top speed (m/s): drive_limits.max_linear_speed_mps, or fallbackMaxSpeed.</summary>
        float maxSpeed;
        /// <summary>Top turn rate (rad/s): drive_limits.max_angular_speed_radps, or fallbackMaxTurnRate.</summary>
        float maxTurnRate;
        /// <summary>Target angle (degrees) of each arm joint, moved by Q/E.</summary>
        readonly Dictionary<string, float> armTargets = new Dictionary<string, float>();
        /// <summary>Lane the T key teleports to next.</summary>
        int laneIndex;
        /// <summary>Where R resets the rover to.</summary>
        Vector3 resetPoint;
        /// <summary>Heading (degrees) the rover faces after R.</summary>
        float resetYaw;

        /// <summary>Attach to a rover and environment; read the top speeds and arm joints.</summary>
        public void Init(RoverRig rig, EnvironmentInfo info)
        {
            Rig = rig;
            Info = info;
            maxSpeed = rig.DriveControl.MaxLinearSpeed ?? fallbackMaxSpeed;
            maxTurnRate = rig.DriveControl.MaxAngularSpeed ?? fallbackMaxTurnRate;
            ArmJoints = rig.JointsWithRole("arm").Where(j => rig.JointBody(j) != null).ToList();
            foreach (var joint in ArmJoints) armTargets[joint] = 0f;
            resetPoint = info.StartPad;
        }

        /// <summary>Each frame: read the keyboard and command the drive chain, the arm and the teleport keys.</summary>
        void Update()
        {
            var kb = Keyboard.current;
            if (Rig == null || kb == null) return;
            float forward = Axis(kb, Key.W, Key.UpArrow) - Axis(kb, Key.S, Key.DownArrow);
            float turn = Axis(kb, Key.A, Key.LeftArrow) - Axis(kb, Key.D, Key.RightArrow);
            if (kb.leftBracketKey.wasPressedThisFrame) speedScale = Mathf.Max(0.1f, Mathf.Round((speedScale - 0.1f) * 10f) / 10f);
            if (kb.rightBracketKey.wasPressedThisFrame) speedScale = Mathf.Min(1f, Mathf.Round((speedScale + 0.1f) * 10f) / 10f);
            CommandSpeed = kb.spaceKey.isPressed ? 0f : forward * maxSpeed * speedScale;
            CommandTurn = kb.spaceKey.isPressed ? 0f : turn * maxTurnRate * speedScale;
            ManualActive = forward != 0f || turn != 0f || kb.spaceKey.isPressed;
            if (ManualActive || Ros == null || !Ros.ExternalActive)
                Rig.DriveControl.Command(CommandSpeed, CommandTurn); // the drive chain ramps it, every physics step

            for (int i = 0; i < Mathf.Min(9, ArmJoints.Count); i++)
                if (kb[Key.Digit1 + i].wasPressedThisFrame) SelectedArmJoint = i;
            if (ArmJoints.Count > 0)
            {
                string joint = ArmJoints[SelectedArmJoint];
                float delta = (Axis(kb, Key.E) - Axis(kb, Key.Q)) * armDegreesPerSecond * Time.deltaTime;
                if (delta != 0f)
                {
                    var drive = Rig.Joints[joint]["unity_drive"];
                    float lo = (float?)drive["lower_limit"] ?? -360f, hi = (float?)drive["upper_limit"] ?? 360f;
                    armTargets[joint] = Mathf.Clamp(armTargets[joint] + delta, lo, hi);
                    Rig.SetArmTarget(joint, armTargets[joint]);
                }
                if (kb.zKey.wasPressedThisFrame)
                    foreach (var j in ArmJoints) { armTargets[j] = 0f; Rig.SetArmTarget(j, 0f); }
            }

            if (kb.rKey.wasPressedThisFrame) Teleport(resetPoint, resetYaw, Location);
            if (kb.tKey.wasPressedThisFrame && Info.Lanes.Count > 0)
            {
                laneIndex = (laneIndex + 1) % Info.Lanes.Count;
                var lane = Info.Lanes[laneIndex];
                Teleport(lane.Start, 0f, $"Lane: {lane.Name} ({lane.Description})");
            }
            if (kb.gKey.wasPressedThisFrame && Info.Terrain != null) Teleport(Info.DesertStart, 90f, "Utah desert");
        }

        // Every physics step: the drive chain (acceleration limits, motor ramps), then linkages.
        /// <summary>Each physics step: run the drive chain, arm profiles, joint damping and linkages.</summary>
        void FixedUpdate()
        {
            if (Rig == null) return;
            Rig.StepControllers(Time.fixedDeltaTime);   // drive chain, arm profiles, joint damping, linkages
        }

        /// <summary>Move the rover to a point facing `yaw` (degrees), remember it for R, and stop the drive chain.</summary>
        public void Teleport(Vector3 point, float yaw, string label)
        {
            resetPoint = point;
            resetYaw = yaw;
            Location = label;
            Rig.DriveControl.Release(); // a teleport stops the rover; start the drive chain from rest
            Rig.PlaceAt(point, yaw);
            Teleported?.Invoke();
        }

        /// <summary>1 if any of the keys is held, else 0.</summary>
        static float Axis(Keyboard kb, params Key[] keys) => keys.Any(k => kb[k].isPressed) ? 1f : 0f;
    }
}
