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
        // Couplings (four-bar linkages etc.) are stiff position drives that keep a follower joint at
        // multiplier * leader + offset. Stiff enough to hold the linkage, soft enough to stay stable.
        public const float CouplingStiffness = 5000f;   // N*m per degree
        public const float CouplingDamping = 100f;
        public const float CouplingForce = 10000f;

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

        /// <summary>One closed-linkage constraint from manifest.physics.couplings.</summary>
        public class Coupling
        {
            public string Follower, Leader;
            public float Multiplier, OffsetDegrees;
            // Closed-loop emulation: where the follower drive's reaction torque lands (null = nowhere to move it).
            public ArticulationBody ReactionFrom, ReactionTo;
        }

        public List<Coupling> Couplings { get; } = new List<Coupling>();

        /// <summary>Reaction routing for closed linkages: 1 = on (default), 0 = off, -1 = reversed (diagnostics only;
        /// command line: -couplingReaction on|off|flip).</summary>
        public static float CouplingReactionSign =
            FeasibilityPaths.Arg("-couplingReaction") == "off" ? 0f : FeasibilityPaths.Arg("-couplingReaction") == "flip" ? -1f : 1f;

        /// <summary>A spring/damper from manifest.physics.springs (e.g. a suspension shock).</summary>
        public class Spring
        {
            public string Joint;
            public float RestDegrees, MaxSagDegrees, StiffnessPerRad, DampingPerRad;
        }

        public List<Spring> Springs { get; } = new List<Spring>();

        // Unity's ArticulationDrive takes targets in degrees but passes stiffness and damping to PhysX
        // unchanged, i.e. per radian - the same units as the manifest (checked by the settle test's sag).
        public const float DriveGainPerManifestUnit = 1f;

        public void Spawn(GameObject prefab)
        {
            var robot = UnityEngine.Object.Instantiate(prefab, Vector3.zero, Quaternion.identity);
            robot.name = prefab.name;
            Attach(robot);
        }

        /// <summary>Use a robot that is already in the scene.</summary>
        public void Attach(GameObject robot)
        {
            Robot = robot;
            Links.Clear();
            Frames.Clear();
            foreach (var t in Robot.GetComponentsInChildren<Transform>(true))
                Frames[t.name] = t;
            foreach (var body in Robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                Links[body.name] = body;
                if (body.isRoot) Root = body;
            }
            if (Root == null) throw new InvalidOperationException("Imported robot has no root ArticulationBody.");
            // Measured once, while the robot is upright and its transforms are current:
            // how far the root sits above the robot's lowest collider point.
            rootAboveLowest = Root.transform.position.y - LowestPoint();
            LockJoints();
            LoadCouplings();
            LoadSprings();
        }

        public List<string> LockedJoints { get; } = new List<string>();

        /// <summary>Joints listed in manifest.physics.locked_joints become fixed (at their zero pose).</summary>
        void LockJoints()
        {
            LockedJoints.Clear();
            if (!(Manifest["physics"]?["locked_joints"] is JArray list)) return;
            foreach (var name in list.Select(t => (string)t))
            {
                var body = JointBody(name);
                if (body == null) continue;
                body.jointType = ArticulationJointType.FixedJoint;
                LockedJoints.Add(name);
            }
        }

        void LoadSprings()
        {
            Springs.Clear();
            if (!(Manifest["physics"]?["springs"] is JArray list)) return;
            foreach (var s in list)
            {
                var spring = new Spring
                {
                    Joint = (string)s["joint"],
                    RestDegrees = (float?)s["rest_deg"] ?? 0f,
                    MaxSagDegrees = (float?)s["max_sag_deg"] ?? 2f,
                    StiffnessPerRad = (float)s["stiffness_nm_per_rad"],
                    DampingPerRad = (float)s["damping_nms_per_rad"],
                };
                if (JointBody(spring.Joint) == null) continue;
                // rest_deg is in the URDF joint's direction; flip it if the importer reversed this joint's axis.
                Vector3 urdfAxis = AxisFromManifest(spring.Joint);
                Vector3 unityAxis = Root.transform.InverseTransformDirection(JointAxisWorld(JointBody(spring.Joint)));
                if (Vector3.Dot(urdfAxis, unityAxis) < 0f) spring.RestDegrees = -spring.RestDegrees;
                Springs.Add(spring);
            }
        }

        void LoadCouplings()
        {
            Couplings.Clear();
            if (!(Manifest["physics"]?["couplings"] is JArray list)) return;
            foreach (var c in list)
            {
                var coupling = new Coupling
                {
                    Follower = (string)c["follower"],
                    Leader = (string)c["leader"],
                    Multiplier = (float)c["multiplier"],
                    OffsetDegrees = (float?)c["offset"] ?? 0f,
                };
                var follower = JointBody(coupling.Follower);
                var leader = JointBody(coupling.Leader);
                if (follower == null || leader == null) continue;
                // The multiplier was worked out from the URDF axes. If the importer flipped one joint's
                // axis but not the other, flip the sign so the linkage still moves the same way.
                Vector3 urdfF = AxisFromManifest(coupling.Follower), urdfL = AxisFromManifest(coupling.Leader);
                float urdfDot = Vector3.Dot(urdfF, urdfL);
                float unityDot = Vector3.Dot(JointAxisWorld(follower), JointAxisWorld(leader));
                if (Mathf.Abs(urdfDot) > 0.5f && Mathf.Abs(unityDot) > 0.5f && Mathf.Sign(urdfDot) != Mathf.Sign(unityDot))
                    coupling.Multiplier = -coupling.Multiplier;
                // In the real mechanism the closing link (e.g. a four-bar's second bar) carries the follower's
                // torque back to the leader's parent. Without it, the coupling drive's reaction would load the
                // follower's parent instead (e.g. twist a four-bar's crank against its shock). Move it.
                var followerParent = ParentBody(follower);
                var leaderParent = ParentBody(leader);
                if (followerParent != null && leaderParent != null && followerParent != leaderParent)
                {
                    coupling.ReactionFrom = followerParent;
                    coupling.ReactionTo = leaderParent;
                }
                Couplings.Add(coupling);
            }
            IgnoreLinkageCollisions();
        }

        /// <summary>Pairs of colliders in the same linkage that were told to ignore each other.</summary>
        public int IgnoredLinkagePairs { get; private set; }

        /// <summary>
        /// Parts of one closed linkage overlap where the missing joint would pin them together (e.g. a four-bar's
        /// back bar and its bogie). Unity only skips parent-child contacts, so they would collide and fight the
        /// linkage. Every body moved by a coupling (leader, followers and what they carry, except wheels) stops
        /// colliding with the others and with the body the linkage is mounted on.
        /// </summary>
        void IgnoreLinkageCollisions()
        {
            var wheels = new HashSet<ArticulationBody>(JointsWithRole("wheel").Select(JointBody).Where(b => b != null));
            var groups = new Dictionary<ArticulationBody, HashSet<ArticulationBody>>();
            foreach (var c in Couplings)
            {
                var leader = JointBody(c.Leader);
                if (!groups.TryGetValue(leader, out var group)) groups[leader] = group = new HashSet<ArticulationBody>();
                var mount = ParentBody(leader);           // the body the linkage is pinned to (e.g. the chassis)
                if (mount != null) group.Add(mount);
                foreach (var name in new[] { c.Leader, c.Follower })
                    foreach (var body in JointBody(name).GetComponentsInChildren<ArticulationBody>())
                        if (!wheels.Contains(body)) group.Add(body);
            }
            IgnoredLinkagePairs = 0;
            foreach (var group in groups.Values)
            {
                var colliders = group.SelectMany(b => b.GetComponentsInChildren<Collider>()
                                                       .Where(col => col.GetComponentInParent<ArticulationBody>() == b)
                                                       .Select(col => (b, col))).ToList();
                for (int i = 0; i < colliders.Count; i++)
                    for (int j = i + 1; j < colliders.Count; j++)
                        if (colliders[i].b != colliders[j].b)
                        {
                            Physics.IgnoreCollision(colliders[i].col, colliders[j].col);
                            IgnoredLinkagePairs++;
                        }
            }
        }

        static ArticulationBody ParentBody(ArticulationBody body) =>
            body.transform.parent == null ? null : body.transform.parent.GetComponentInParent<ArticulationBody>();

        Vector3 AxisFromManifest(string joint)
        {
            var a = Joints[joint]?["axis_unity_root"] as JArray;
            return a == null ? Vector3.zero : new Vector3((float)a[0], (float)a[1], (float)a[2]);
        }

        /// <summary>Point every coupled joint's drive at its leader's current angle, and route the drive's
        /// reaction torque the way the real closed linkage would. Call before each physics step.</summary>
        public void UpdateCouplings()
        {
            foreach (var c in Couplings)
            {
                var body = JointBody(c.Follower);
                var drive = body.xDrive;
                drive.target = c.Multiplier * JointDegrees(c.Leader) + c.OffsetDegrees;
                body.xDrive = drive;
                if (c.ReactionFrom != null)
                {
                    // Torque the drive applied last step (on the follower; the reaction -torque went to its parent).
                    Vector3 torque = JointAxisWorld(body) * (body.driveForce[0] * CouplingReactionSign);
                    c.ReactionFrom.AddTorque(torque);   // cancel the reaction on the follower's parent...
                    c.ReactionTo.AddTorque(-torque);    // ...and put it on the leader's parent, like the closing link
                }
            }
        }

        /// <summary>Step physics, keeping linkages coupled.</summary>
        public void Simulate(int steps, float dt)
        {
            for (int i = 0; i < steps; i++)
            {
                UpdateCouplings();
                Physics.Simulate(dt);
            }
        }

        float rootAboveLowest;

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

        /// <summary>
        /// Move the rover to (x, z), facing `yawDegrees` (0 = +Z, the rover's forward), with its lowest
        /// point `clearance` above whatever ground is below it (works on uneven terrain).
        /// </summary>
        public void PlaceAt(Vector3 position, float yawDegrees, float clearance = 0.05f)
        {
            // One teleport, computed from the ground height and the offset measured at spawn.
            // (After TeleportRoot, Transform and collider positions are not reliably updated until
            // the next simulation step, so nothing is read back in between.)
            Physics.SyncTransforms(); // make newly created ground colliders visible to raycasts
            var ownColliders = new HashSet<Collider>(Robot.GetComponentsInChildren<Collider>());
            float ground = position.y;
            float highest = float.NegativeInfinity;
            foreach (var hit in Physics.RaycastAll(new Vector3(position.x, position.y + 200f, position.z), Vector3.down, 400f))
                if (!ownColliders.Contains(hit.collider) && hit.point.y > highest) highest = hit.point.y;
            if (!float.IsNegativeInfinity(highest)) ground = highest;
            var target = new Vector3(position.x, ground + clearance + rootAboveLowest, position.z);
            Root.TeleportRoot(target, Quaternion.Euler(0f, yawDegrees, 0f));
            foreach (var body in Links.Values)
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }

        /// <summary>Wheel joints on the left / right side of the rover (by position relative to the root).</summary>
        public bool IsLeftWheel(string wheelJoint) =>
            Root.transform.InverseTransformPoint(JointBody(wheelJoint).transform.position).x < 0f;

        /// <summary>Driven wheels, their average radius and the left-right track width.</summary>
        public (List<string> wheels, float trackWidth, float radius) WheelGeometry()
        {
            var wheels = JointsWithRole("wheel").Where(j => JointBody(j) != null).ToList();
            if (wheels.Count == 0) return (wheels, 0.5f, 0.15f);
            float radius = wheels.Average(WheelRadius);
            float SideX(string w) => Root.transform.InverseTransformPoint(JointBody(w).transform.position).x;
            float left = wheels.Where(IsLeftWheel).Select(SideX).DefaultIfEmpty(-0.25f).Average();
            float right = wheels.Where(w => !IsLeftWheel(w)).Select(SideX).DefaultIfEmpty(0.25f).Average();
            return (wheels, Mathf.Max(0.1f, right - left), radius);
        }

        /// <summary>Skid-steer drive: forward speed (m/s) and turn rate (rad/s, positive = left).</summary>
        public void Drive(float speed, float turnRate, float trackWidth, float wheelRadius, IReadOnlyList<string> wheels)
        {
            Vector3 forward = Vector3.ProjectOnPlane(Root.transform.forward, Root.transform.up).normalized;
            foreach (var wheel in wheels)
            {
                float side = IsLeftWheel(wheel) ? -1f : 1f; // left wheels slower when turning left
                float wheelSpeed = speed + side * turnRate * trackWidth / 2f;
                SetWheelSpeed(wheel, ForwardSign(wheel, forward) * wheelSpeed / wheelRadius * Mathf.Rad2Deg);
            }
        }

        /// <summary>Wheels: velocity drives. Arm: stiff position drives. Passive: URDF damping. Springs: shock drives. Coupled joints: linkage drives.</summary>
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
            // Passive joints (free bogies etc.): the URDF's own <dynamics damping>, in N*m*s/rad like Unity's drive.
            var managed = new HashSet<string>(LockedJoints.Concat(Springs.Select(sp => sp.Joint)).Concat(Couplings.Select(c => c.Follower)));
            foreach (var name in JointsWithRole("suspension").Concat(JointsWithRole("passive")))
            {
                var body = JointBody(name);
                float damping = (float?)Joints[name]["urdf_dynamics"]?["damping"] ?? 0f;
                if (body == null || managed.Contains(name) || damping <= 0f) continue;
                var drive = body.xDrive;
                drive.driveType = ArticulationDriveType.Force;
                drive.stiffness = 0f;
                drive.damping = damping;
                drive.forceLimit = float.MaxValue;
                drive.targetVelocity = 0f;
                body.xDrive = drive;
            }
            foreach (var spring in Springs)
            {
                var body = JointBody(spring.Joint);
                var drive = body.xDrive;
                drive.driveType = ArticulationDriveType.Force;
                drive.stiffness = spring.StiffnessPerRad * DriveGainPerManifestUnit;
                drive.damping = spring.DampingPerRad * DriveGainPerManifestUnit;
                drive.forceLimit = float.MaxValue;
                drive.target = spring.RestDegrees;
                drive.targetVelocity = 0f;
                body.xDrive = drive;
            }
            foreach (var c in Couplings)
            {
                var body = JointBody(c.Follower);
                var drive = body.xDrive;
                drive.driveType = ArticulationDriveType.Force;
                drive.stiffness = CouplingStiffness;
                drive.damping = CouplingDamping;
                drive.forceLimit = CouplingForce;
                drive.target = c.Multiplier * JointDegrees(c.Leader) + c.OffsetDegrees;
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
        public float JointDegrees(string joint)
        {
            var body = JointBody(joint);
            return body.dofCount == 0 ? 0f : body.jointPosition[0] * Mathf.Rad2Deg; // locked (fixed) joints: 0
        }

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

        public string FastestLink() => Links.Values.OrderByDescending(b => b.linearVelocity.magnitude).First().name;

        /// <summary>Angle in degrees between the chassis "up" and world up.</summary>
        public float TiltDegrees() => Vector3.Angle(Root.transform.up, Vector3.up);

        public float TotalMass() => Links.Values.Sum(b => b.mass);

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
