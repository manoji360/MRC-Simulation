using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MRC.Feasibility
{
    /// <summary>
    /// Moves arm joints the way the real arm controller does: each joint follows its target along a
    /// trapezoidal velocity profile within the real limits (manifest.physics.joint_limits, from MoveIt's
    /// joint_limits.yaml; falls back to the URDF velocity limit). The position drive then tracks that
    /// moving setpoint (with velocity feed-forward), like the geared ODrive position control.
    /// Joints without any limit are not managed: SetArmTarget moves them directly.
    /// </summary>
    public class ArmController
    {
        /// <summary>Motion state of one managed arm joint.</summary>
        class JointState
        {
            /// <summary>The joint's ArticulationBody (its child link).</summary>
            public ArticulationBody Body;
            /// <summary>Where the joint should end up (degrees).</summary>
            public float Target;
            /// <summary>Current setpoint along the profile (degrees), written to the drive target each step.</summary>
            public float Position;
            /// <summary>Current setpoint velocity (deg/s), written to the drive as velocity feed-forward.</summary>
            public float Velocity;
            /// <summary>Maximum joint speed (deg/s), from joint_limits or the URDF velocity limit.</summary>
            public float MaxVelocity;
            /// <summary>Maximum joint acceleration (deg/s^2); infinity when no limit is known.</summary>
            public float MaxAcceleration;
        }

        /// <summary>Managed arm joints by joint name.</summary>
        readonly Dictionary<string, JointState> joints = new Dictionary<string, JointState>();

        /// <summary>Collect every arm joint with a known speed limit. accelScale scales all accelerations (experiments: -param arm_accel_scale).</summary>
        public ArmController(RoverRig rig, JObject jointLimits, float accelScale = 1f)
        {
            foreach (var name in rig.JointsWithRole("arm"))
            {
                var body = rig.JointBody(name);
                if (body == null || body.dofCount == 0) continue;
                var limits = jointLimits?[name];
                float? v = (float?)limits?["max_velocity_unity"] ?? (float?)rig.Joints[name]["unity_drive"]?["max_velocity"];
                float? a = (float?)limits?["max_acceleration_unity"];
                if (!v.HasValue) continue;
                joints[name] = new JointState
                {
                    Body = body,
                    MaxVelocity = v.Value,
                    MaxAcceleration = a.HasValue ? a.Value * accelScale : float.PositiveInfinity,
                };
            }
        }

        /// <summary>True if this joint follows a limited profile (otherwise SetArmTarget moves it directly).</summary>
        public bool Manages(string joint) => joints.ContainsKey(joint);
        /// <summary>Names of the managed joints.</summary>
        public IEnumerable<string> Joints => joints.Keys;
        /// <summary>The joint's speed limit in deg/s.</summary>
        public float MaxVelocity(string joint) => joints[joint].MaxVelocity;
        /// <summary>The joint's acceleration limit in deg/s^2 (infinity = none).</summary>
        public float MaxAcceleration(string joint) => joints[joint].MaxAcceleration;
        /// <summary>The joint's current profile setpoint in degrees.</summary>
        public float Setpoint(string joint) => joints[joint].Position;
        /// <summary>The joint's current profile velocity in deg/s.</summary>
        public float SetpointVelocity(string joint) => joints[joint].Velocity;

        /// <summary>Start from the joints' current positions, at rest.</summary>
        public void Reset()
        {
            foreach (var js in joints.Values)
            {
                js.Position = js.Target = js.Body.jointPosition[0] * Mathf.Rad2Deg;
                js.Velocity = 0f;
            }
        }

        /// <summary>Set where the joint should move to (degrees); it gets there along the profile.</summary>
        public void SetTarget(string joint, float degrees) => joints[joint].Target = degrees;

        /// <summary>Advance every joint's setpoint by one physics step and write it to its drive.</summary>
        public void Step(float dt)
        {
            foreach (var js in joints.Values)
            {
                float remaining = js.Target - js.Position;
                if (Mathf.Abs(remaining) < 1e-4f && Mathf.Abs(js.Velocity) <= js.MaxAcceleration * dt)
                {
                    js.Position = js.Target;
                    js.Velocity = 0f;
                }
                else
                {
                    float direction = Mathf.Sign(remaining);
                    float stopping = float.IsInfinity(js.MaxAcceleration) ? 0f : js.Velocity * js.Velocity / (2f * js.MaxAcceleration);
                    bool brake = Mathf.Sign(js.Velocity) == direction && stopping >= Mathf.Abs(remaining);
                    float desired = brake ? 0f : direction * js.MaxVelocity;
                    js.Velocity = float.IsInfinity(js.MaxAcceleration) ? desired
                        : Mathf.MoveTowards(js.Velocity, desired, js.MaxAcceleration * dt);
                    float step = js.Velocity * dt;
                    if (Mathf.Sign(step) == direction && Mathf.Abs(step) >= Mathf.Abs(remaining))
                    {
                        js.Position = js.Target;   // arrived this step
                        js.Velocity = 0f;
                    }
                    else js.Position += step;
                }
                var drive = js.Body.xDrive;
                drive.target = js.Position;
                drive.targetVelocity = js.Velocity;
                js.Body.xDrive = drive;
            }
        }
    }
}
