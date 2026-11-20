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
        // Drive tuning used by the feasibility test (see docs/code-guide.md).
        // Units: ArticulationDrive targets are in degrees (deg/s), but Unity passes stiffness and damping
        // to PhysX unscaled, so they act PER RADIAN (rad/s) of error - measured by the spring sag test.
        /// <summary>Wheel velocity-drive gain: N*m per rad/s of wheel speed error.</summary>
        public const float WheelDamping = 50f;          // N*m per rad/s of wheel speed error
        /// <summary>Wheel torque cap (N*m) when neither the registry (wheel_max_torque_nm) nor the URDF gives one.</summary>
        public const float WheelDefaultForce = 30f;     // N*m wheel torque cap when neither the registry nor the URDF gives one
        /// <summary>Arm position-drive stiffness: N*m per rad of position error (stiff, like geared position control).</summary>
        public const float ArmStiffness = 10000f;       // N*m per rad of arm position error
        /// <summary>Arm position-drive damping: N*m per rad/s.</summary>
        public const float ArmDamping = 100f;           // N*m per rad/s
        // Couplings (four-bar linkages etc.) are stiff position drives that keep a follower joint at
        // multiplier * leader + offset. Stiff enough to hold the linkage, soft enough to stay stable.
        /// <summary>Linkage (coupling) drive stiffness: N*m per rad of error between follower and leader.</summary>
        public const float CouplingStiffness = 5000f;   // N*m per rad
        /// <summary>Linkage drive damping: N*m per rad/s.</summary>
        public const float CouplingDamping = 100f;      // N*m per rad/s
        /// <summary>Linkage drive torque cap (N*m).</summary>
        public const float CouplingForce = 10000f;      // N*m torque cap

        /// <summary>The spawned robot GameObject (the prefab instance).</summary>
        public GameObject Robot { get; private set; }
        /// <summary>The robot's root ArticulationBody (the base link).</summary>
        public ArticulationBody Root { get; private set; }
        /// <summary>The pipeline manifest of this robot (manifest.json).</summary>
        public JObject Manifest { get; }
        /// <summary>Every ArticulationBody (link) by link name.</summary>
        public Dictionary<string, ArticulationBody> Links { get; } = new Dictionary<string, ArticulationBody>();
        /// <summary>Every Transform in the robot by name (links, collision and visual children).</summary>
        public Dictionary<string, Transform> Frames { get; } = new Dictionary<string, Transform>();
        /// <summary>The flat test ground created by CreateGround(), if any.</summary>
        public GameObject Ground { get; private set; }

        /// <summary>Create a rig for a robot described by this manifest (call Spawn or Attach next).</summary>
        public RoverRig(JObject manifest) { Manifest = manifest; }

        /// <summary>Name of the root link (manifest parts[0].root_link).</summary>
        public string RootLinkName => (string)Manifest["parts"][0]["root_link"];
        /// <summary>manifest.physics.joints: every moving joint with its role, child link, Unity drive limits and axis.</summary>
        public JObject Joints => (JObject)Manifest["physics"]["joints"];

        /// <summary>Names of the moving joints with this role (wheel, suspension, arm or passive).</summary>
        public IEnumerable<string> JointsWithRole(string role) =>
            Joints.Properties().Where(p => (string)p.Value["role"] == role).Select(p => p.Name);

        /// <summary>The body a joint drives (the joint's child link).</summary>
        public ArticulationBody JointBody(string joint) =>
            Links.TryGetValue((string)Joints[joint]["child_link"], out var body) ? body : null;

        /// <summary>One closed-linkage constraint from manifest.physics.couplings.</summary>
        public class Coupling
        {
            /// <summary>Joint that follows (its drive is pointed at the leader).</summary>
            public string Follower;
            /// <summary>Joint it follows.</summary>
            public string Leader;
            /// <summary>follower = Multiplier * leader + OffsetDegrees (sign already corrected for the importer's axes).</summary>
            public float Multiplier;
            /// <summary>Constant offset in degrees.</summary>
            public float OffsetDegrees;
            // Closed-loop emulation: where the follower drive's reaction torque lands (null = nowhere to move it).
            /// <summary>Body whose drive reaction is cancelled (the follower's parent), or null.</summary>
            public ArticulationBody ReactionFrom;
            /// <summary>Body that receives that reaction instead (the leader's parent, as the real closing link would), or null.</summary>
            public ArticulationBody ReactionTo;
        }

        /// <summary>Linkage couplings from manifest.physics.couplings.</summary>
        public List<Coupling> Couplings { get; } = new List<Coupling>();

        /// <summary>The robot's drive chain (speed/acceleration limits from manifest.physics.drive_limits).</summary>
        public DriveController DriveControl { get; private set; }
        /// <summary>Arm joints follow their targets within the real joint limits (manifest.physics.joint_limits).</summary>
        public ArmController ArmControl { get; private set; }
        /// <summary>Physical-parameter registry (manifest.physics.parameters) plus -param overrides.</summary>
        public SimParameters Parameters { get; private set; }
        /// <summary>What Attach changed because of parameter overrides (for result files).</summary>
        public JObject AppliedOverrides { get; } = new JObject();

        /// <summary>Reaction routing for closed linkages: 1 = on (default), 0 = off, -1 = reversed (diagnostics only;
        /// command line: -couplingReaction on|off|flip).</summary>
        public static float CouplingReactionSign =
            FeasibilityPaths.Arg("-couplingReaction") == "off" ? 0f : FeasibilityPaths.Arg("-couplingReaction") == "flip" ? -1f : 1f;

        /// <summary>A spring/damper from manifest.physics.springs (e.g. a suspension shock).</summary>
        public class Spring
        {
            /// <summary>Joint the spring acts on.</summary>
            public string Joint;
            /// <summary>Angle the spring holds (degrees, sign corrected for the importer's axis).</summary>
            public float RestDegrees;
            /// <summary>Design sag under the robot's weight (degrees), used by the settle test.</summary>
            public float MaxSagDegrees;
            /// <summary>Spring stiffness (N*m per rad).</summary>
            public float StiffnessPerRad;
            /// <summary>Spring damping (N*m per rad/s).</summary>
            public float DampingPerRad;
        }

        /// <summary>Springs from manifest.physics.springs.</summary>
        public List<Spring> Springs { get; } = new List<Spring>();

        // Unity's ArticulationDrive takes targets in degrees but passes stiffness and damping to PhysX
        // unchanged, i.e. per radian - the same units as the manifest (checked by the settle test's sag).
        /// <summary>Factor between manifest gains (per radian) and Unity drive gains: 1, because Unity drives are per radian too.</summary>
        public const float DriveGainPerManifestUnit = 1f;

        /// <summary>Instantiate the robot prefab at the origin and Attach to it.</summary>
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
            ResetImporterDynamics();
            Parameters = new SimParameters(Manifest["physics"]?["parameters"] as JObject);
            ApplyMassOverrides();
            ApplyTyreFriction();
            LockJoints();
            IgnoreAssembledOverlaps();
            DriveControl = new DriveController(this, Manifest["physics"]?["drive_limits"] as JObject);
            ArmControl = new ArmController(this, Manifest["physics"]?["joint_limits"] as JObject,
                                           Parameters.Float("arm_accel_scale") ?? 1f);
            LoadCouplings();
            LoadSprings();
        }

        /// <summary>
        /// -param total_mass_kg / center_of_mass_offset_m differing from the registry (which the pipeline already
        /// applied to the URDF): scale every real body's mass and inertia, and shift every body's centre of mass by
        /// the extra offset (ROS root frame), so the whole robot's centre of mass moves by exactly that much.
        /// </summary>
        void ApplyMassOverrides()
        {
            var real = Links.Values.Where(b => b.mass >= PlaceholderMassKg).ToList();
            float manifestTotal = (float?)Manifest["physics"]?["total_mass_kg"] ?? 0f;
            float? total = Parameters.IsOverridden("total_mass_kg") ? Parameters.Float("total_mass_kg") : null;
            if (total.HasValue && manifestTotal > 0f)
            {
                float scale = total.Value / manifestTotal;
                foreach (var b in real) { b.mass *= scale; b.inertiaTensor *= scale; }
                AppliedOverrides["mass_scale"] = scale;
            }
            if (Parameters.IsOverridden("center_of_mass_offset_m"))
            {
                float[] wanted = Parameters.Vector("center_of_mass_offset_m");
                float[] applied = Parameters.RegistryVector("center_of_mass_offset_m") ?? new float[3];
                var deltaRos = new Vector3(wanted[0] - applied[0], wanted[1] - applied[1], wanted[2] - applied[2]);
                var deltaUnity = new Vector3(-deltaRos.y, deltaRos.z, deltaRos.x);       // ROS -> Unity axes
                Vector3 world = Root.transform.TransformDirection(deltaUnity);
                foreach (var b in real) b.centerOfMass += b.transform.InverseTransformDirection(world);
                AppliedOverrides["center_of_mass_shift_ros_m"] = new JArray(deltaRos.x, deltaRos.y, deltaRos.z);
            }
        }

        /// <summary>Wheels get the tyre material when the registry gives tyre friction (see SurfaceFriction).</summary>
        void ApplyTyreFriction()
        {
            if (!Parameters.HasAny("tyre_mu_")) return;
            SurfaceFriction.Configure(Parameters);
            foreach (var name in JointsWithRole("wheel"))
            {
                var body = JointBody(name);
                if (body == null) continue;
                foreach (var c in body.GetComponentsInChildren<Collider>())
                    if (c.GetComponentInParent<ArticulationBody>() == body) c.sharedMaterial = SurfaceFriction.Tyre;
            }
        }

        /// <summary>Bodies whose importer-set damping/friction were reset (see ResetImporterDynamics).</summary>
        public JArray ImporterDynamicsReset { get; } = new JArray();

        /// <summary>
        /// The URDF Importer (v0.5.2) maps a joint's &lt;dynamics&gt; wrongly: damping becomes the child BODY's
        /// linearDamping/angularDamping (drag on its motion through the world, as if in syrup - about 80 N on the
        /// OSU rover's bogies at 3.7 m/s), and friction becomes PhysX jointFriction (a unitless coefficient times the
        /// joint load, not N*m). Reset both to zero; the URDF's joint damping and friction are applied correctly
        /// (N*m*s/rad, N*m) by ConfigureDrives / ApplyPassiveDamping. Aerodynamic drag is negligible at rover speeds.
        /// </summary>
        void ResetImporterDynamics()
        {
            ImporterDynamicsReset.Clear();
            foreach (var b in Links.Values)
            {
                if (b.linearDamping != 0f || b.angularDamping != 0f || b.jointFriction != 0f)
                    ImporterDynamicsReset.Add($"{b.name}: linearDamping {b.linearDamping}, angularDamping {b.angularDamping}, jointFriction {b.jointFriction}");
                b.linearDamping = 0f;
                b.angularDamping = 0f;
                b.jointFriction = 0f;
            }
        }

        /// <summary>Collider pairs (different bodies) that overlap in the assembled pose and were told to ignore each other.</summary>
        public int IgnoredOverlapPairs { get; private set; }
        /// <summary>'bodyA / bodyB' names of every body pair whose overlapping colliders were told to ignore each other.</summary>
        public JArray IgnoredOverlapBodies { get; } = new JArray();

        /// <summary>
        /// Parts that already overlap in the robot's assembled (zero) pose are pinned or bolted together on the real
        /// robot (e.g. a bogie and the second bar of its linkage share one pin), not touching. Unity only ignores
        /// contacts between a joint's parent and child, so such pairs would jam against each other. Ignore every
        /// collider pair of two different bodies that is penetrating at spawn.
        /// </summary>
        void IgnoreAssembledOverlaps()
        {
            Physics.SyncTransforms();
            var colliders = Robot.GetComponentsInChildren<Collider>().Where(c => c.enabled)
                .Select(c => (collider: c, body: c.GetComponentInParent<ArticulationBody>())).Where(x => x.body != null).ToList();
            var bodyPairs = new HashSet<string>();
            IgnoredOverlapPairs = 0;
            for (int i = 0; i < colliders.Count; i++)
                for (int j = i + 1; j < colliders.Count; j++)
                {
                    var (a, ba) = colliders[i];
                    var (b, bb) = colliders[j];
                    if (ba == bb || !a.bounds.Intersects(b.bounds)) continue;
                    if (!Physics.ComputePenetration(a, a.transform.position, a.transform.rotation,
                                                    b, b.transform.position, b.transform.rotation, out _, out float depth) || depth <= 1e-4f)
                        continue;
                    Physics.IgnoreCollision(a, b);
                    IgnoredOverlapPairs++;
                    string key = string.CompareOrdinal(ba.name, bb.name) < 0 ? $"{ba.name} / {bb.name}" : $"{bb.name} / {ba.name}";
                    if (bodyPairs.Add(key)) IgnoredOverlapBodies.Add(key);
                }
        }

        /// <summary>Joints made fixed because the manifest lists them in locked_joints.</summary>
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

        /// <summary>Read manifest.physics.springs; flip each rest angle if the importer reversed that joint's axis.</summary>
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

        /// <summary>Read manifest.physics.couplings; fix multiplier signs for axes the importer reversed; set up reaction-torque routing for nested linkages.</summary>
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

        /// <summary>The ArticulationBody above this one in the hierarchy (its joint's parent link), or null for the root.</summary>
        static ArticulationBody ParentBody(ArticulationBody body) =>
            body.transform.parent == null ? null : body.transform.parent.GetComponentInParent<ArticulationBody>();

        /// <summary>A joint's axis relative to the root (Unity coordinates) from manifest.physics.joints, or zero if unknown.</summary>
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
                StepControllers(dt);
                Physics.Simulate(dt);
            }
        }

        /// <summary>
        /// Everything that runs before each physics step: drive chain, arm profiles, passive-joint damping,
        /// linkages. The scene (RoverTeleop / RosRoverBridge FixedUpdate) and tests (Simulate) both call this.
        /// </summary>
        public void StepControllers(float dt)
        {
            DriveControl.Step(dt);
            ArmControl.Step(dt);
            ApplyPassiveDamping();
            UpdateCouplings();
        }

        /// <summary>Height of the root above the robot's lowest collider point, measured at spawn (used by PlaceAt).</summary>
        float rootAboveLowest;

        /// <summary>Create a 500 m x 500 m flat test ground (its friction comes from the registry's tyre-on-concrete values when present).</summary>
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
            SurfaceFriction.Register(material, "concrete");   // registry tyre-on-concrete friction, if the robot has one
            Ground.GetComponent<Collider>().sharedMaterial = material;
        }

        /// <summary>Lowest point of the robot's colliders (world Y).</summary>
        public float LowestPoint()
        {
            Physics.SyncTransforms();
            return Robot.GetComponentsInChildren<Collider>().Where(c => c.enabled).Min(c => c.bounds.min.y);
        }

        /// <summary>Lift the rover so its lowest collider point is `clearance` metres above y = 0.</summary>
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

        /// <summary>Raw skid-steer drive: sets wheel targets instantly, with no limits (DriveControl is the realistic path).</summary>
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
                float? urdfEffort = (float?)Joints[name]["unity_drive"]["force_limit"];
                drive.stiffness = 0f;
                drive.damping = WheelDamping;
                // Registry (or -param) first: the URDF has no wheel effort for the OSU rover.
                drive.forceLimit = Parameters.Float("wheel_max_torque_nm") ?? urdfEffort ?? WheelDefaultForce;
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
                drive.forceLimit = (Parameters.IsOverridden("arm_max_torque_nm") ? Parameters.Float("arm_max_torque_nm") : null)
                                   ?? force ?? 100f;
                drive.target = 0f;
                body.xDrive = drive;
            }
            ArmControl.Reset();
            // Passive joints (free bogies etc.): URDF <dynamics> friction (Coulomb, N*m) and damping (N*m*s/rad),
            // or the registry's passive_joint_friction_nm / passive_joint_damping_nms when given.
            // Coulomb friction = a velocity drive to 0 whose torque is capped at the friction torque: it holds the
            // joint still below that torque (stiction) and resists with exactly that torque when moving.
            // Viscous damping is then added as an explicit torque each step (ApplyPassiveDamping).
            PassiveJoints.Clear();
            var managed = new HashSet<string>(LockedJoints.Concat(Springs.Select(sp => sp.Joint)).Concat(Couplings.Select(c => c.Follower)));
            foreach (var name in JointsWithRole("suspension").Concat(JointsWithRole("passive")))
            {
                var body = JointBody(name);
                if (body == null || managed.Contains(name) || body.dofCount == 0) continue;
                float friction = Parameters.Float("passive_joint_friction_nm") ?? (float?)Joints[name]["urdf_dynamics"]?["friction"] ?? 0f;
                float damping = Parameters.Float("passive_joint_damping_nms") ?? (float?)Joints[name]["urdf_dynamics"]?["damping"] ?? 0f;
                if (friction <= 0f && damping <= 0f) continue;
                var drive = body.xDrive;
                drive.driveType = ArticulationDriveType.Force;
                drive.stiffness = 0f;
                drive.targetVelocity = 0f;
                if (friction > 0f) { drive.damping = CoulombDriveDamping; drive.forceLimit = friction; }
                else { drive.damping = damping; drive.forceLimit = float.MaxValue; }
                body.xDrive = drive;
                PassiveJoints.Add(new PassiveJoint { Joint = name, Body = body, Parent = ParentBody(body),
                                                     FrictionNm = friction, DampingNms = friction > 0f ? damping : 0f });
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

        /// <summary>One free joint with friction/damping (see ConfigureDrives).</summary>
        public class PassiveJoint
        {
            /// <summary>Name of the passive joint.</summary>
            public string Joint;
            /// <summary>The joint's body (child link).</summary>
            public ArticulationBody Body;
            /// <summary>The joint's parent body (receives the damping reaction).</summary>
            public ArticulationBody Parent;
            /// <summary>Coulomb friction torque (N*m): the drive's torque cap.</summary>
            public float FrictionNm;
            /// <summary>Viscous damping (N*m per rad/s) applied explicitly; 0 when the drive itself does the damping.</summary>
            public float DampingNms;
        }

        /// <summary>Every free joint with friction or damping (bogie pivots), set up by ConfigureDrives.</summary>
        public List<PassiveJoint> PassiveJoints { get; } = new List<PassiveJoint>();
        /// <summary>Drive damping used for Coulomb friction: high enough that the torque cap (the friction) always decides.</summary>
        const float CoulombDriveDamping = 1e4f;   // N*m per rad/s: stiff enough that the torque cap (friction) decides

        /// <summary>Viscous damping on passive joints that also have Coulomb friction (their drive is the friction).</summary>
        void ApplyPassiveDamping()
        {
            foreach (var pj in PassiveJoints)
            {
                if (pj.DampingNms <= 0f || pj.Parent == null) continue;
                float omega = pj.Body.jointVelocity[0];            // rad/s
                Vector3 torque = JointAxisWorld(pj.Body) * (-pj.DampingNms * omega);
                pj.Body.AddTorque(torque);
                pj.Parent.AddTorque(-torque);
            }
        }

        /// <summary>Set a wheel's target speed in deg/s (Unity's unit). Prefer DriveControl.Command for realistic driving.</summary>
        public void SetWheelSpeed(string joint, float degreesPerSecond)
        {
            var body = JointBody(joint);
            var drive = body.xDrive;
            drive.targetVelocity = degreesPerSecond;
            body.xDrive = drive;
        }

        /// <summary>Move an arm joint to `degrees`: through the real joint limits when known (ArmControl).</summary>
        public void SetArmTarget(string joint, float degrees)
        {
            if (ArmControl != null && ArmControl.Manages(joint)) { ArmControl.SetTarget(joint, degrees); return; }
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

        /// <summary>True if any link has a NaN or infinite position or velocity (the simulation blew up).</summary>
        public bool AnyNaN() =>
            Links.Values.Any(b => !IsFinite(b.transform.position) || !IsFinite(b.linearVelocity));

        /// <summary>Links lighter than this are pipeline placeholders (URDF links without mass), not real parts.</summary>
        public const float PlaceholderMassKg = 0.01f;

        /// <summary>Fastest link speed (m/s); realPartsOnly ignores 1 g placeholder links.</summary>
        public float MaxLinkSpeed(bool realPartsOnly = false) =>
            Links.Values.Where(b => !realPartsOnly || b.mass >= PlaceholderMassKg).Max(b => b.linearVelocity.magnitude);

        /// <summary>Name of the fastest-moving link; realPartsOnly ignores 1 g placeholder links.</summary>
        public string FastestLink(bool realPartsOnly = false) => Links.Values
            .Where(b => !realPartsOnly || b.mass >= PlaceholderMassKg)
            .OrderByDescending(b => b.linearVelocity.magnitude).First().name;

        /// <summary>Angle in degrees between the chassis "up" and world up.</summary>
        public float TiltDegrees() => Vector3.Angle(Root.transform.up, Vector3.up);

        /// <summary>Sum of every body's mass (kg), as Unity simulates it.</summary>
        public float TotalMass() => Links.Values.Sum(b => b.mass);

        /// <summary>Remove the robot and the ground immediately (tests run back to back in one frame).</summary>
        public void Destroy()
        {
            // Immediate: tests run back to back in one frame, and a deferred Destroy would leave
            // the previous robot in the physics scene.
            if (Robot != null) UnityEngine.Object.DestroyImmediate(Robot);
            if (Ground != null) UnityEngine.Object.DestroyImmediate(Ground);
        }

        /// <summary>True if no component of v is NaN or infinite.</summary>
        static bool IsFinite(Vector3 v) =>
            !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
              float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
    }
}
