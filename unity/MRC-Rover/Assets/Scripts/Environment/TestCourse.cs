using System.Collections.Generic;
using UnityEngine;

namespace MRC.Environment
{
    /// <summary>One straight test lane: the rover starts at Start (facing +Z) and must pass EndZ.</summary>
    public struct CourseLane
    {
        /// <summary>Lane id, e.g. 'ramp_20' (used as the test case name).</summary>
        public string Name;
        /// <summary>What the lane tests, e.g. '20 deg ramp, 0.6 m rise'.</summary>
        public string Description;
        /// <summary>World point where the rover starts (facing +Z).</summary>
        public Vector3 Start;
        /// <summary>World Z the rover must pass to cross the lane.</summary>
        public float EndZ;
    }

    /// <summary>
    /// Standard rover challenge lanes on a concrete pad, side by side along +X, each driven along +Z:
    /// ramps 10/20/30 deg (0.6 m rise), steps 5/10/15 cm, a rubble field and a rough-ground patch.
    /// </summary>
    public static class TestCourse
    {
        /// <summary>Width of a lane (m).</summary>
        public const float LaneWidth = 3.5f;
        /// <summary>Distance between lane centres (m).</summary>
        public const float LaneSpacing = 5f;
        /// <summary>Length of a lane's concrete pad (m).</summary>
        public const float LaneLength = 30f;
        /// <summary>Distance (m) from a lane's start to its feature (ramp, step, rubble ...).</summary>
        public const float FeatureStartZ = 6f;
        /// <summary>Height (m) every ramp climbs.</summary>
        public const float RampRise = 0.6f;

        /// <summary>Colour of the concrete lane pads.</summary>
        static readonly Color PadColor = new Color(0.46f, 0.45f, 0.43f);
        /// <summary>Colour of ramps and steps.</summary>
        static readonly Color FeatureColor = new Color(0.60f, 0.57f, 0.52f);
        /// <summary>Colour of the lane start/finish stripes.</summary>
        static readonly Color StripeColor = new Color(0.95f, 0.75f, 0.1f);
        /// <summary>Colour of the rocks.</summary>
        static readonly Color RockColor = new Color(0.52f, 0.36f, 0.27f);

        /// <summary>Build all eight lanes under parent, starting at origin; returns their start/end points.</summary>
        public static List<CourseLane> Build(Transform parent, Vector3 origin)
        {
            var root = new GameObject("TestCourse").transform;
            root.SetParent(parent, false);
            root.position = origin;
            var lanes = new List<CourseLane>();
            var features = new (string name, string description, System.Action<Transform> build)[]
            {
                ("ramp_10", "10 deg ramp, 0.6 m rise", t => Ramp(t, 10f)),
                ("ramp_20", "20 deg ramp, 0.6 m rise", t => Ramp(t, 20f)),
                ("ramp_30", "30 deg ramp, 0.6 m rise", t => Ramp(t, 30f)),
                ("step_05", "5 cm step up and down", t => Step(t, 0.05f)),
                ("step_10", "10 cm step up and down", t => Step(t, 0.10f)),
                ("step_15", "15 cm step up and down", t => Step(t, 0.15f)),
                ("rubble", "10 m of fixed rocks 5-20 cm", t => Rubble(t, 1234)),
                ("rough", "10 m of bumpy ground (bumps up to ~10 cm)", t => Rough(t)),
            };

            // Concrete pad under every lane (top surface at y = 0).
            float padWidth = features.Length * LaneSpacing + 4f;
            var pad = Box("Pad", root, new Vector3(padWidth, 0.5f, LaneLength + 10f),
                          new Vector3((features.Length - 1) * LaneSpacing / 2f, -0.25f, LaneLength / 2f - 5f), PadColor);
            pad.GetComponent<Collider>().sharedMaterial = EnvMaterials.Concrete;

            for (int i = 0; i < features.Length; i++)
            {
                var lane = new GameObject($"Lane_{features[i].name}").transform;
                lane.SetParent(root, false);
                lane.localPosition = new Vector3(i * LaneSpacing, 0f, 0f);
                features[i].build(lane);
                // Start line stripe (visual only).
                var stripe = Box("StartLine", lane, new Vector3(LaneWidth, 0.005f, 0.1f), new Vector3(0, 0.0025f, 0f), StripeColor);
                Object.DestroyImmediate(stripe.GetComponent<Collider>());
                lanes.Add(new CourseLane
                {
                    Name = features[i].name,
                    Description = features[i].description,
                    Start = lane.position + new Vector3(0f, 0f, -2f),
                    EndZ = lane.position.z + FeatureStartZ + 12f,
                });
            }
            return lanes;
        }

        /// <summary>A ramp up `degrees` to RampRise, a flat top and a ramp back down.</summary>
        static void Ramp(Transform lane, float degrees)
        {
            float run = RampRise / Mathf.Tan(degrees * Mathf.Deg2Rad);
            float z = FeatureStartZ;
            Wedge("RampUp", lane, run, new Vector3(0f, 0f, z), 0f);
            Box("RampTop", lane, new Vector3(LaneWidth, RampRise, 3f), new Vector3(0f, RampRise / 2f, z + run + 1.5f), FeatureColor)
                .GetComponent<Collider>().sharedMaterial = EnvMaterials.Concrete;
            Wedge("RampDown", lane, run, new Vector3(0f, 0f, z + 2f * run + 3f), 180f);
        }

        /// <summary>One wedge (ramp) mesh with a concrete collider.</summary>
        static void Wedge(string name, Transform lane, float run, Vector3 localPos, float yaw)
        {
            var go = new GameObject(name);
            go.transform.SetParent(lane, false);
            go.transform.localPosition = localPos;
            go.transform.localRotation = Quaternion.Euler(0f, yaw, 0f);
            var mesh = ProceduralMeshes.Wedge(LaneWidth, run, RampRise);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = EnvMaterials.Color(FeatureColor);
            var collider = go.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            collider.convex = true;
            collider.sharedMaterial = EnvMaterials.Concrete;
        }

        /// <summary>A step of `height` metres, 1.5 m long, up and back down.</summary>
        static void Step(Transform lane, float height)
        {
            Box("Step", lane, new Vector3(LaneWidth, height, 1.5f), new Vector3(0f, height / 2f, FeatureStartZ + 0.75f), FeatureColor)
                .GetComponent<Collider>().sharedMaterial = EnvMaterials.Concrete;
        }

        /// <summary>90 random rocks (5-20 cm) over 10 m, deterministic for a seed.</summary>
        static void Rubble(Transform lane, int seed)
        {
            var rng = new System.Random(seed);
            var material = EnvMaterials.Color(RockColor);
            var meshes = new Mesh[8];
            for (int i = 0; i < meshes.Length; i++) meshes[i] = ProceduralMeshes.Rock(seed + i);
            for (int i = 0; i < 90; i++)
            {
                float size = 0.05f + (float)rng.NextDouble() * 0.15f;
                var pos = new Vector3(((float)rng.NextDouble() - 0.5f) * (LaneWidth - 0.4f), size * 0.3f,
                                      FeatureStartZ + (float)rng.NextDouble() * 10f);
                var rock = RockObject(lane, meshes[rng.Next(meshes.Length)], material, pos,
                                      new Vector3(size * (0.8f + (float)rng.NextDouble() * 0.6f), size, size * (0.8f + (float)rng.NextDouble() * 0.6f)),
                                      (float)rng.NextDouble() * 360f);
                rock.name = $"Rubble_{i:00}";
            }
        }

        /// <summary>A 10 m patch of bumpy ground (noise heightfield, up to about 10 cm).</summary>
        static void Rough(Transform lane)
        {
            float H(float x, float z) =>
                0.08f * (Mathf.PerlinNoise(x * 0.9f + 3.1f, z * 0.9f + 7.7f) - 0.5f) * 2f
                + 0.03f * (Mathf.PerlinNoise(x * 3f + 11f, z * 3f + 5f) - 0.5f) * 2f;
            var go = new GameObject("RoughGround");
            go.transform.SetParent(lane, false);
            go.transform.localPosition = new Vector3(0f, 0f, FeatureStartZ);
            // Taper to zero at both ends so the patch starts and ends flush with the pad;
            // dips below y = 0 are filled by the pad, so the rover meets bumps only.
            var mesh = ProceduralMeshes.Heightfield(LaneWidth, 10f, 0.1f,
                (x, z) => H(x, z) * Mathf.Clamp01(Mathf.Min(z, 10f - z) / 1.5f));
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = EnvMaterials.Color(FeatureColor * 0.9f);
            var collider = go.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            collider.sharedMaterial = EnvMaterials.Concrete;
        }

        /// <summary>A rock with a convex mesh collider and rock friction.</summary>
        public static GameObject RockObject(Transform parent, Mesh mesh, Material material, Vector3 localPos, Vector3 scale, float yaw)
        {
            var rock = new GameObject("Rock");
            rock.transform.SetParent(parent, false);
            rock.transform.localPosition = localPos;
            rock.transform.localRotation = Quaternion.Euler(0f, yaw, 0f);
            rock.transform.localScale = scale;
            rock.AddComponent<MeshFilter>().sharedMesh = mesh;
            rock.AddComponent<MeshRenderer>().sharedMaterial = material;
            var collider = rock.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            collider.convex = true;
            collider.sharedMaterial = EnvMaterials.Rock;
            return rock;
        }

        /// <summary>A coloured box with a collider.</summary>
        static GameObject Box(string name, Transform parent, Vector3 size, Vector3 localPos, Color color)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            box.transform.SetParent(parent, false);
            box.transform.localPosition = localPos;
            box.transform.localScale = size;
            box.GetComponent<MeshRenderer>().sharedMaterial = EnvMaterials.Color(color);
            return box;
        }
    }
}
