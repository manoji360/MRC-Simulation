using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MRC.Feasibility
{
    /// <summary>
    /// The robot's drive chain, as on the real rover (limits from manifest.physics.drive_limits):
    ///   operator command (m/s, rad/s)
    ///   -> drive controller (ros2 diff_drive_controller): clamps speed, limits acceleration and
    ///      deceleration, updates at controller_rate_hz, mixes turning with its own wheel separation/radius
    ///   -> motor drivers (ODrive ramped velocity mode): each wheel's speed ramps at wheel_accel_rev_s2
    ///   -> Unity wheel velocity drives.
    /// Without drive_limits everything passes straight through (instant changes).
    /// Nothing is written to the wheels until the first Command(), so tests may still set wheels directly.
    /// </summary>
    public class DriveController
    {
        /// <summary>The robot whose wheels this controller drives.</summary>
        readonly RoverRig rig;

        /// <summary>Top forward speed (m/s), or null for no limit (operator commands are clamped to it).</summary>
        public float? MaxLinearSpeed;
        /// <summary>Top turn rate (rad/s), or null for no limit.</summary>
        public float? MaxAngularSpeed;
        /// <summary>Forward acceleration limit (m/s^2), or null for instant.</summary>
        public float? LinearAccel;
        /// <summary>Forward deceleration limit (m/s^2); defaults to LinearAccel, as in diff_drive_controller.</summary>
        public float? LinearDecel;
        /// <summary>Turn-rate acceleration limit (rad/s^2), or null for instant.</summary>
        public float? AngularAccel;
        /// <summary>Per-wheel motor ramp (wheel revolutions/s^2), e.g. ODrive vel_ramp_rate / gear ratio; null for none.</summary>
        public float? WheelAccelRevS2;
        /// <summary>How often the controller updates its command (Hz), like the real controller_manager update_rate.</summary>
        public float? RateHz;
        /// <summary>Wheel separation (m) used to mix turning into wheel speeds; null = the measured track width.</summary>
        public float? ControllerSeparation;
        /// <summary>Wheel radius (m) used to turn m/s into wheel speed; null = the measured collider radius.</summary>
        public float? ControllerRadius;
        /// <summary>True if the manifest gave any drive limits (otherwise everything is instant).</summary>
        public bool HasLimits { get; }

        /// <summary>What the operator asked for (after the speed clamp).</summary>
        public float CommandLinear { get; private set; }
        /// <summary>Turn rate the operator asked for (rad/s, + = left), after the clamp.</summary>
        public float CommandAngular { get; private set; }
        /// <summary>The controller's output after acceleration limiting (what the wheels are told to do).</summary>
        public float Linear { get; private set; }
        /// <summary>Turn rate the controller is currently commanding (rad/s), after acceleration limiting.</summary>
        public float Angular { get; private set; }
        /// <summary>True after the first Command(): only then are the wheels driven by this controller.</summary>
        public bool Active { get; private set; }

        /// <summary>Each wheel's current target speed in deg/s, after the motor ramp.</summary>
        readonly Dictionary<string, float> wheelTarget = new Dictionary<string, float>(); // deg/s, after the motor ramp
        /// <summary>Wheel joint names driven by this controller.</summary>
        List<string> wheels;
        /// <summary>Measured left-right track width (m), used when no controller separation is given.</summary>
        float geometricTrack;
        /// <summary>Measured average wheel collider radius (m), used when no controller radius is given.</summary>
        float geometricRadius;
        /// <summary>Seconds since the last controller update (the controller runs at RateHz).</summary>
        float sinceUpdate;

        /// <summary>Read the limits from manifest.physics.drive_limits (and -param overrides via rig.Parameters).</summary>
        public DriveController(RoverRig rig, JObject limits)
        {
            this.rig = rig;
            float? Get(string key) => limits?[key] is JValue v && v.Type != JTokenType.Null ? (float?)v : null;
            MaxLinearSpeed = Get("max_linear_speed_mps");
            MaxAngularSpeed = Get("max_angular_speed_radps");
            LinearAccel = Get("max_linear_accel_mps2");
            LinearDecel = Get("max_linear_decel_mps2") ?? LinearAccel;
            AngularAccel = Get("max_angular_accel_radps2");
            WheelAccelRevS2 = Get("wheel_accel_rev_s2");
            RateHz = Get("controller_rate_hz");
            ControllerSeparation = Get("controller_wheel_separation_m");
            ControllerRadius = rig.Parameters?.Float("controller_wheel_radius_m") ?? Get("controller_wheel_radius_m");
            HasLimits = limits != null && limits.HasValues;
        }

        /// <summary>Set the operator's target: forward speed (m/s) and turn rate (rad/s, + = left).</summary>
        public void Command(float linear, float angular)
        {
            CommandLinear = MaxLinearSpeed.HasValue ? Mathf.Clamp(linear, -MaxLinearSpeed.Value, MaxLinearSpeed.Value) : linear;
            CommandAngular = MaxAngularSpeed.HasValue ? Mathf.Clamp(angular, -MaxAngularSpeed.Value, MaxAngularSpeed.Value) : angular;
            if (!Active)
            {
                Active = true;
                (wheels, geometricTrack, geometricRadius) = rig.WheelGeometry();
                wheelTarget.Clear();
                foreach (var w in wheels) wheelTarget[w] = rig.JointBody(w).xDrive.targetVelocity;
                sinceUpdate = float.PositiveInfinity; // first controller update happens on the next step
            }
        }

        /// <summary>Stop driving the wheels (they keep their last target). Used by tests that drive wheels directly.</summary>
        public void Release()
        {
            Active = false;
            Linear = Angular = CommandLinear = CommandAngular = 0f;
        }

        /// <summary>Advance the drive chain by one physics step.</summary>
        public void Step(float dt)
        {
            if (!Active || wheels == null || wheels.Count == 0) return;

            // 1. Drive controller: runs at its own rate, limiting the change of the body twist.
            sinceUpdate += dt;
            float period = RateHz.HasValue ? 1f / RateHz.Value : 0f;
            if (sinceUpdate >= period - 1e-6f)
            {
                float h = float.IsInfinity(sinceUpdate) || period == 0f ? Mathf.Max(period, dt) : sinceUpdate;
                Linear = Limit(Linear, CommandLinear, LinearAccel, LinearDecel, h);
                Angular = Limit(Angular, CommandAngular, AngularAccel, AngularAccel, h);
                sinceUpdate = 0f;
            }

            // 2. Mix into wheel speeds, as the controller does (its own separation/radius if given).
            float separation = ControllerSeparation ?? geometricTrack;
            float radius = ControllerRadius ?? geometricRadius;
            Vector3 forward = Vector3.ProjectOnPlane(rig.Root.transform.forward, rig.Root.transform.up).normalized;
            float maxStep = WheelAccelRevS2.HasValue ? WheelAccelRevS2.Value * 360f * dt : float.PositiveInfinity;
            foreach (var wheel in wheels)
            {
                float side = rig.IsLeftWheel(wheel) ? -1f : 1f; // left wheels slower when turning left
                float rim = Linear + side * Angular * separation / 2f;
                float desired = rig.ForwardSign(wheel, forward) * rim / radius * Mathf.Rad2Deg;
                // 3. Motor driver ramp (per wheel).
                float next = Mathf.MoveTowards(wheelTarget[wheel], desired, maxStep);
                wheelTarget[wheel] = next;
                rig.SetWheelSpeed(wheel, next);
            }
        }

        /// <summary>diff_drive_controller-style limiter: decelerate at `decel` when the change opposes the motion.</summary>
        static float Limit(float current, float target, float? accel, float? decel, float h)
        {
            float change = target - current;
            bool slowing = current != 0f && Mathf.Sign(change) != Mathf.Sign(current);
            float? rate = slowing ? decel : accel;
            if (!rate.HasValue) return target;
            float max = rate.Value * h;
            // When slowing through zero, stop at zero first (then accelerate the other way on later updates).
            float next = current + Mathf.Clamp(change, -max, max);
            if (slowing && Mathf.Sign(next) != Mathf.Sign(current) && next != 0f) next = 0f;
            return next;
        }

        /// <summary>One-line summary of the active limits, for the HUD and result files.</summary>
        public string Describe() => !HasLimits ? "no drive limits (instant)" :
            $"max {MaxLinearSpeed:0.##} m/s, {MaxAngularSpeed:0.##} rad/s | accel {LinearAccel:0.##} / decel {LinearDecel:0.##} m/s2, " +
            $"turn {AngularAccel:0.##} rad/s2 | wheel ramp {WheelAccelRevS2:0.##} rev/s2 | {RateHz:0} Hz";
    }
}
