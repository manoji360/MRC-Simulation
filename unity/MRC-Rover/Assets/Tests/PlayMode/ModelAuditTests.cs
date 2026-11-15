// Audits the imported robot against the original URDF data (via the pipeline manifest):
// per-link mass, centre of mass and full inertia tensor, whole-robot centre of mass, wheel spin
// axes, and how round the wheel colliders are. Results: FeasibilityResults/<variant>/audit.json.
using System.Collections.Generic;
using System.Linq;
using MRC.Feasibility;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace MRC.Feasibility.Tests
{
    public class ModelAuditTests
    {
        static string variant;
        static JObject manifest;
        static GameObject prefab;
        static JObject results;
        RoverRig rig;

        [OneTimeSetUp]
        public void Load()
        {
            (variant, manifest, prefab) = TestRobot.Load();
            if (manifest["physics"]?["links"] == null)
                Assert.Ignore("Manifest has no inertial data; re-run prepare with the current pipeline.");
            results = new JObject { ["variant"] = variant };
        }

        [OneTimeTearDown]
        public void Save()
        {
            if (results != null) FeasibilityPaths.WriteResult(variant, "audit.json", results);
        }

        [SetUp]
        public void Spawn()
        {
            rig = new RoverRig(manifest);
            rig.Spawn(prefab);
        }

        [TearDown]
        public void Cleanup() => rig?.Destroy();

        static Vector3 V3(JToken t) => new Vector3((float)t[0], (float)t[1], (float)t[2]);
        static float R(float v, int d = 4) => (float)System.Math.Round(v, d);

        static float[,] Tensor(JToken rows)
        {
            var m = new float[3, 3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i, j] = (float)rows[i][j];
            return m;
        }

        /// <summary>Full inertia tensor of a body in its own local frame: R diag(I) R^T.</summary>
        static float[,] UnityTensor(ArticulationBody body)
        {
            var r = Matrix4x4.Rotate(body.inertiaTensorRotation);
            var d = body.inertiaTensor;
            var m = new float[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    m[i, j] = r[i, 0] * d.x * r[j, 0] + r[i, 1] * d.y * r[j, 1] + r[i, 2] * d.z * r[j, 2];
            return m;
        }

        static float RelativeError(float[,] a, float[,] b)
        {
            float diff = 0f, norm = 0f;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) { diff += (a[i, j] - b[i, j]) * (a[i, j] - b[i, j]); norm += b[i, j] * b[i, j]; }
            return Mathf.Sqrt(diff) / Mathf.Max(Mathf.Sqrt(norm), 1e-9f);
        }

        [Test]
        public void InertialsMatchUrdf()
        {
            var perLink = new JObject();
            float worstTensor = 0f, worstCom = 0f;
            string worstTensorLink = null, worstComLink = null;
            foreach (var link in ((JObject)manifest["physics"]["links"]).Properties())
            {
                if (!rig.Links.TryGetValue(link.Name, out var body)) continue;
                float tensorError = RelativeError(UnityTensor(body), Tensor(link.Value["inertia_unity"]));
                float comError = (body.centerOfMass - V3(link.Value["com_unity"])).magnitude;
                float massError = Mathf.Abs(body.mass - (float)link.Value["mass"]);
                perLink[link.Name] = new JObject
                {
                    ["mass_urdf"] = R((float)link.Value["mass"]), ["mass_unity"] = R(body.mass),
                    ["com_error_mm"] = R(comError * 1000f, 2), ["inertia_rel_error"] = R(tensorError),
                    ["unity_inertia_diag"] = new JArray(R(body.inertiaTensor.x, 6), R(body.inertiaTensor.y, 6), R(body.inertiaTensor.z, 6)),
                    ["automatic_com"] = body.automaticCenterOfMass, ["automatic_inertia"] = body.automaticInertiaTensor,
                    ["mass_error_kg"] = R(massError),
                };
                if (tensorError > worstTensor) { worstTensor = tensorError; worstTensorLink = link.Name; }
                if (comError > worstCom) { worstCom = comError; worstComLink = link.Name; }
            }
            results["inertials"] = new JObject
            {
                ["worst_inertia_rel_error"] = R(worstTensor), ["worst_inertia_link"] = worstTensorLink,
                ["worst_com_error_mm"] = R(worstCom * 1000f, 2), ["worst_com_link"] = worstComLink,
                ["links"] = perLink,
            };
            Assert.Less(worstCom, 0.001f, $"Centre of mass of '{worstComLink}' is {worstCom * 1000f:F1} mm from the URDF value.");
            Assert.Less(worstTensor, 0.01f, $"Inertia of '{worstTensorLink}' differs from the URDF by {worstTensor:P1}.");
        }

        [Test]
        public void RobotCenterOfMassMatchesUrdf()
        {
            Transform root = rig.Links[rig.RootLinkName].transform;
            float total = 0f;
            Vector3 sum = Vector3.zero;
            foreach (var body in rig.Links.Values)
            {
                sum += body.mass * root.InverseTransformPoint(body.worldCenterOfMass);
                total += body.mass;
            }
            Vector3 unity = sum / total;
            Vector3 expected = V3(manifest["physics"]["center_of_mass"]["unity"]);
            float error = (unity - expected).magnitude;
            results["center_of_mass"] = new JObject
            {
                ["unity"] = new JArray(R(unity.x), R(unity.y), R(unity.z)),
                ["urdf"] = new JArray(R(expected.x), R(expected.y), R(expected.z)),
                ["error_mm"] = R(error * 1000f, 2),
                ["height_above_lowest_point_m"] = R(root.TransformPoint(unity).y - rig.LowestPoint()),
            };
            Assert.Less(error, 0.005f, $"Robot centre of mass is {error * 1000f:F1} mm from the URDF value.");
        }

        [Test]
        public void WheelAxesMatchUrdf()
        {
            Transform root = rig.Links[rig.RootLinkName].transform;
            var perWheel = new JObject();
            float worst = 0f;
            foreach (var joint in rig.JointsWithRole("wheel"))
            {
                var body = rig.JointBody(joint);
                var expected = manifest["physics"]["joints"][joint]["axis_unity_root"];
                if (body == null || expected == null || expected.Type == JTokenType.Null) continue;
                Vector3 unityAxis = Quaternion.Inverse(root.rotation) * RoverRig.JointAxisWorld(body);
                float dot = Vector3.Dot(unityAxis.normalized, V3(expected).normalized);
                float angle = Mathf.Acos(Mathf.Clamp(Mathf.Abs(dot), 0f, 1f)) * Mathf.Rad2Deg;
                worst = Mathf.Max(worst, angle);
                perWheel[joint] = new JObject
                {
                    ["axis_error_deg"] = R(angle, 3),
                    ["direction"] = dot >= 0 ? "same" : "reversed (importer uses -axis; handled by drive sign)",
                    ["joint_type"] = body.jointType.ToString(),
                    ["max_angular_velocity"] = R(body.maxAngularVelocity, 2),
                };
            }
            results["wheel_axes"] = new JObject { ["worst_axis_error_deg"] = R(worst, 3), ["wheels"] = perWheel };
            Assert.Less(worst, 0.5f, $"A wheel spins about an axis {worst:F2} deg away from the URDF axis.");
        }

        [Test]
        public void WheelCollidersAreRound()
        {
            // Unity wheels collide through convex hulls made by VHACD. A coarse hull makes the
            // wheel polygonal, which feels like driving on bumps. Measure the outline radius.
            var perWheel = new JObject();
            float worst = 0f;
            Physics.SyncTransforms();
            foreach (var joint in rig.JointsWithRole("wheel"))
            {
                var body = rig.JointBody(joint);
                if (body == null) continue;
                Vector3 axis = RoverRig.JointAxisWorld(body);
                Vector3 center = body.transform.position;
                Vector3 u = Vector3.Cross(axis, Mathf.Abs(axis.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
                Vector3 w = Vector3.Cross(axis, u);
                var points = new List<Vector3>();
                int hulls = 0;
                foreach (var collider in body.GetComponentsInChildren<MeshCollider>())
                {
                    if (collider.sharedMesh == null || collider.attachedArticulationBody != body) continue;
                    hulls++;
                    foreach (var v in collider.sharedMesh.vertices)
                    {
                        Vector3 p = collider.transform.TransformPoint(v) - center;
                        points.Add(p - Vector3.Dot(p, axis) * axis); // projection onto the wheel plane
                    }
                }
                if (points.Count == 0) continue;
                // Support function: how far the (convex) collision outline reaches in each direction.
                // A circle gives the same value everywhere; flats and corners make it vary.
                var reach = new float[360];
                for (int a = 0; a < 360; a++)
                {
                    float rad = a * Mathf.Deg2Rad;
                    Vector3 dir = Mathf.Cos(rad) * u + Mathf.Sin(rad) * w;
                    reach[a] = points.Max(p => Vector3.Dot(p, dir));
                }
                float mean = reach.Average(), outOfRound = (reach.Max() - reach.Min()) / mean;
                worst = Mathf.Max(worst, outOfRound);
                perWheel[joint] = new JObject
                {
                    ["convex_hulls"] = hulls, ["hull_vertices"] = points.Count,
                    ["mean_radius_m"] = R(mean), ["min_radius_m"] = R(reach.Min()), ["max_radius_m"] = R(reach.Max()),
                    ["out_of_round_percent"] = R(outOfRound * 100f, 1),
                };
            }
            results["wheel_colliders"] = new JObject { ["worst_out_of_round_percent"] = R(worst * 100f, 1), ["wheels"] = perWheel };
            Assert.Less(worst, 0.05f, $"A wheel collider is {worst * 100f:F1}% out of round (bumpy rolling).");
        }
    }
}
