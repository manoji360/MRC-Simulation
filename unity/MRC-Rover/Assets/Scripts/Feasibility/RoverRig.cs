using System;
using System.Collections.Generic;
using System.Linq;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MRC.Feasibility
{
    /// <summary>
    /// Spawns an imported rover on flat ground, configures its joint drives from the
    /// pipeline manifest (Unity units, so no conversion here) and steps physics manually.
    /// </summary>
    public class RoverRig
    {
        // Drive tuning used by the feasibility test (documented in the report).
        public const float WheelDamping = 50f;          // N*m per deg/s of speed error
        public const float WheelDefaultForce = 30f;     // N*m per wheel when the URDF gives no effort
        public const float ArmStiffness = 10000f;       // N*m per degree of position error
        public const float ArmDamping = 100f;

        public GameObject Robot { get; private set; }
        public ArticulationBody Root { get; private set; }
        public JObject Manifest { get; }
        public Dictionary<string, ArticulationBody> Links { get; } = new Dictionary<string, ArticulationBody>();
        public Dictionary<string, Transform> Frames { get; } = new Dictionary<string, Transform>();
        public GameObject Ground { get; private set; }

        public RoverRig(JObject manifest) { Manifest = manifest; }

        public string RootLinkName => (string)Manifest["parts"][0]["root_link"];
        public JObject Joints => (JObject)Manifest["physics"]["joints"];

        public IEnumerable<string> JointsWithRole(string role) =>
            Joints.Properties().Where(p => (string)p.Value["role"] == role).Select(p => p.Name);

        /// <summary>The body a joint drives (the joint's child link).</summary>
        public ArticulationBody JointBody(string joint) =>
            Links.TryGetValue((string)Joints[joint]["child_link"], out var body) ? body : null;

        public void Spawn(GameObject prefab)
        {
            Robot = UnityEngine.Object.Instantiate(prefab, Vector3.zero, Quaternion.identity);
            Robot.name = prefab.name;
            foreach (var t in Robot.GetComponentsInChildren<Transform>(true))
                Frames[t.name] = t;
            foreach (var body in Robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                Links[body.name] = body;
                if (body.isRoot) Root = body;
            }
            if (Root == null) throw new InvalidOperationException("Imported robot has no root ArticulationBody.");
        }

        public void CreateGround(float staticFriction = 1.0f, float dynamicFriction = 0.8f)
        {
            Ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            Ground.name = "FeasibilityGround";
            Ground.transform.localScale = new Vector3(50f, 1f, 50f); // 500 m x 500 m
            var material = new PhysicsMaterial("Ground")
            {
                staticFriction = staticFriction,
                dynamicFriction = dynamicFriction,
                frictionCombine = PhysicsMaterialCombine.Average,
            };
            Ground.GetComponent<Collider>().sharedMaterial = material;
        }

        /// <summary>Lowest point of the robot's colliders (world Y).</summary>
        public float LowestPoint()
        {
            Physics.SyncTransforms();
            return Robot.GetComponentsInChildren<Collider>().Where(c => c.enabled).Min(c => c.bounds.min.y);
        }

        public void PlaceAboveGround(float clearance)
        {
            float lift = clearance - LowestPoint();
            Root.TeleportRoot(Root.transform.position + Vector3.up * lift, Root.transform.rotation);
            Physics.SyncTransforms();
        }

        /// <summary>Wheels: velocity drives. Arm: stiff position drives. Suspension / passive: left free.</summary>
        public void ConfigureDrives()
        {
            foreach (var name in JointsWithRole("wheel"))
            {
                var body = JointBody(name);
                if (body == null) continue;
                var drive = body.xDrive;
                float? force = (float?)Joints[name]["unity_drive"]["force_limit"];
                drive.stiffness = 0f;
                drive.damping = WheelDamping;
                drive.forceLimit = force ?? WheelDefaultForce;
                drive.targetVelocity = 0f;
                body.xDrive = drive;
            }
            foreach (var name in JointsWithRole("arm"))
            {
                var body = JointBody(name);
                if (body == null) continue;
                var drive = body.xDrive;
                float? force = (float?)Joints[name]["unity_drive"]["force_limit"];
                drive.stiffness = ArmStiffness;
                drive.damping = ArmDamping;
                drive.forceLimit = force ?? 100f;
                drive.target = 0f;
                body.xDrive = drive;
            }
        }

        public void SetWheelSpeed(string joint, float degreesPerSecond)
        {
            var body = JointBody(joint);
            var drive = body.xDrive;
            drive.targetVelocity = degreesPerSecond;
            body.xDrive = drive;
        }

        public void SetArmTarget(string joint, float degrees)
        {
            var body = JointBody(joint);
            var drive = body.xDrive;
            drive.target = degrees;
            body.xDrive = drive;
        }

        /// <summary>Joint position of a 1-DOF joint, in degrees (Unity stores radians).</summary>
        public float JointDegrees(string joint) => JointBody(joint).jointPosition[0] * Mathf.Rad2Deg;

        /// <summary>World-space spin axis of a revolute joint (Unity articulations spin about the anchor X axis).</summary>
        public static Vector3 JointAxisWorld(ArticulationBody body) =>
            (body.transform.rotation * (body.anchorRotation * Vector3.right)).normalized;

        /// <summary>+1 or -1: the wheel speed sign that pushes the rover along `forward`.</summary>
        public int ForwardSign(string wheelJoint, Vector3 forward)
        {
            Vector3 axis = JointAxisWorld(JointBody(wheelJoint));
            // A wheel spinning with angular velocity w on the ground moves the vehicle along w x up.
            return Vector3.Dot(Vector3.Cross(axis, Vector3.up), forward) >= 0f ? 1 : -1;
        }

        /// <summary>Approximate wheel radius from the collider bounds (vertical extent / 2).</summary>
        public float WheelRadius(string wheelJoint)
        {
            Physics.SyncTransforms();
            var colliders = JointBody(wheelJoint).GetComponentsInChildren<Collider>();
            if (colliders.Length == 0) return 0f;
            var bounds = colliders[0].bounds;
            foreach (var c in colliders.Skip(1)) bounds.Encapsulate(c.bounds);
            return bounds.size.y / 2f;
        }

        public bool AnyNaN() =>
            Links.Values.Any(b => !IsFinite(b.transform.position) || !IsFinite(b.linearVelocity));

        public float MaxLinkSpeed() => Links.Values.Max(b => b.linearVelocity.magnitude);

        /// <summary>Angle in degrees between the chassis "up" and world up.</summary>
        public float TiltDegrees() => Vector3.Angle(Root.transform.up, Vector3.up);

        public float TotalMass() => Links.Values.Sum(b => b.mass);

        public static void Step(int steps, float dt)
        {
            for (int i = 0; i < steps; i++) Physics.Simulate(dt);
        }

        public void Destroy()
        {
            // Immediate: tests run back to back in one frame, and a deferred Destroy would leave
            // the previous robot in the physics scene.
            if (Robot != null) UnityEngine.Object.DestroyImmediate(Robot);
            if (Ground != null) UnityEngine.Object.DestroyImmediate(Ground);
        }

        static bool IsFinite(Vector3 v) =>
            !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
              float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
    }
}
