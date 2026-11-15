using System.Collections.Generic;
using System.Linq;
using MRC.Feasibility;
using UnityEngine;

namespace MRC.Environment
{
    /// <summary>
    /// Keyboard driving (legacy Input Manager):
    ///   W/S or Up/Down  forward / reverse      A/D or Left/Right  turn (skid steer)
    ///   Left Shift      faster                 Space              stop
    ///   1-9             select arm joint       Q / E              move selected arm joint
    ///   Z               arm to zero pose       R                  reset to the current start point
    ///   T               next test lane         G                  desert start point
    /// </summary>
    public class RoverTeleop : MonoBehaviour
    {
        public float maxSpeed = 0.8f;          // m/s
        public float boost = 2f;
        public float maxTurnRate = 0.8f;       // rad/s
        public float armDegreesPerSecond = 40f;

        public RoverRig Rig { get; private set; }
        public EnvironmentInfo Info { get; private set; }
        public List<string> ArmJoints { get; private set; } = new List<string>();
        public int SelectedArmJoint { get; private set; }
        public string Location { get; private set; } = "Start pad";
        public float CommandSpeed { get; private set; }
        public float CommandTurn { get; private set; }

        List<string> wheels;
        float trackWidth, wheelRadius;
        readonly Dictionary<string, float> armTargets = new Dictionary<string, float>();
        int laneIndex;
        Vector3 resetPoint;
        float resetYaw;

        public void Init(RoverRig rig, EnvironmentInfo info)
        {
            Rig = rig;
            Info = info;
            (wheels, trackWidth, wheelRadius) = rig.WheelGeometry();
            ArmJoints = rig.JointsWithRole("arm").Where(j => rig.JointBody(j) != null).ToList();
            foreach (var joint in ArmJoints) armTargets[joint] = 0f;
            resetPoint = info.StartPad;
        }

        void Update()
        {
            if (Rig == null) return;
            float forward = Axis(KeyCode.W, KeyCode.UpArrow) - Axis(KeyCode.S, KeyCode.DownArrow);
            float turn = Axis(KeyCode.A, KeyCode.LeftArrow) - Axis(KeyCode.D, KeyCode.RightArrow);
            float scale = Input.GetKey(KeyCode.LeftShift) ? boost : 1f;
            CommandSpeed = Input.GetKey(KeyCode.Space) ? 0f : forward * maxSpeed * scale;
            CommandTurn = Input.GetKey(KeyCode.Space) ? 0f : turn * maxTurnRate;
            if (wheels.Count > 0) Rig.Drive(CommandSpeed, CommandTurn, trackWidth, wheelRadius, wheels);

            for (int i = 0; i < Mathf.Min(9, ArmJoints.Count); i++)
                if (Input.GetKeyDown(KeyCode.Alpha1 + i)) SelectedArmJoint = i;
            if (ArmJoints.Count > 0)
            {
                string joint = ArmJoints[SelectedArmJoint];
                float delta = (Axis(KeyCode.E) - Axis(KeyCode.Q)) * armDegreesPerSecond * Time.deltaTime;
                if (delta != 0f)
                {
                    var drive = Rig.Joints[joint]["unity_drive"];
                    float lo = (float?)drive["lower_limit"] ?? -360f, hi = (float?)drive["upper_limit"] ?? 360f;
                    armTargets[joint] = Mathf.Clamp(armTargets[joint] + delta, lo, hi);
                    Rig.SetArmTarget(joint, armTargets[joint]);
                }
                if (Input.GetKeyDown(KeyCode.Z))
                    foreach (var j in ArmJoints) { armTargets[j] = 0f; Rig.SetArmTarget(j, 0f); }
            }

            if (Input.GetKeyDown(KeyCode.R)) Teleport(resetPoint, resetYaw, Location);
            if (Input.GetKeyDown(KeyCode.T) && Info.Lanes.Count > 0)
            {
                laneIndex = (laneIndex + 1) % Info.Lanes.Count;
                var lane = Info.Lanes[laneIndex];
                Teleport(lane.Start, 0f, $"Lane: {lane.Name} ({lane.Description})");
            }
            if (Input.GetKeyDown(KeyCode.G) && Info.Terrain != null) Teleport(Info.DesertStart, 90f, "Utah desert");
        }

        public void Teleport(Vector3 point, float yaw, string label)
        {
            resetPoint = point;
            resetYaw = yaw;
            Location = label;
            Rig.PlaceAt(point, yaw);
        }

        static float Axis(params KeyCode[] keys) => keys.Any(Input.GetKey) ? 1f : 0f;
    }
}
