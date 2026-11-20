using System.Collections.Generic;
using UnityEngine;

namespace MRC.Environment
{
    /// <summary>
    /// Procedural desert modelled on the University Rover Challenge site near the Mars Desert Research
    /// Station (Hanksville, Utah): red sandy flats, rounded grey bentonite mounds, colour-banded
    /// shale/sandstone cliffs, dry washes, flat-topped mesas and buttes, and scattered rocks.
    /// It is a stylised approximation for testing, not survey data.
    /// </summary>
    public static class DesertTerrain
    {
        /// <summary>Terrain size in metres along X and Z.</summary>
        public const float Size = 400f;           // metres (X and Z)
        /// <summary>Vertical range of the heightmap in metres.</summary>
        public const float HeightRange = 45f;     // metres of vertical range in the heightmap
        /// <summary>Height (m) of world y = 0 above the terrain's lowest point; the flat course area sits here.</summary>
        public const float BaseLevel = 8f;        // metres above the terrain's lowest point = world y 0
        /// <summary>Heightmap resolution (samples per side; Unity needs 2^n + 1).</summary>
        const int Resolution = 513;
        /// <summary>Resolution of the texture-blend (splat) map.</summary>
        const int SplatResolution = 512;

        // Mesas/buttes in terrain-local metres: (x, z, radius, height).
        /// <summary>Mesas and buttes in terrain-local metres: (x, z, radius, height).</summary>
        static readonly Vector4[] Mesas =
        {
            new Vector4(55, 335, 48, 26), new Vector4(340, 315, 40, 22), new Vector4(320, 55, 55, 18),
            new Vector4(95, 70, 13, 20), new Vector4(250, 365, 16, 24),
        };

        /// <summary>Pale sand colour of the flats.</summary>
        public static readonly Color PaleSand = new Color(0.72f, 0.58f, 0.43f);
        /// <summary>Red soil colour (Morrison/Entrada formations).</summary>
        public static readonly Color RedSoil = new Color(0.56f, 0.31f, 0.20f);
        /// <summary>Grey bentonite colour of the badland mounds.</summary>
        public static readonly Color Bentonite = new Color(0.50f, 0.48f, 0.47f);
        /// <summary>Purple shale colour of the banded mesa slopes.</summary>
        public static readonly Color PurpleShale = new Color(0.40f, 0.27f, 0.30f);

        /// <summary>
        /// Build the terrain so that world (flatCenter.x, 0, flatCenter.z) is the centre of a flat,
        /// level area of `flatSize` (for the test course). Returns the Terrain.
        /// </summary>
        public static Terrain Build(Transform parent, Vector3 flatCenter, Vector2 flatSize, int seed = 7)
        {
            var data = new TerrainData { heightmapResolution = Resolution, alphamapResolution = SplatResolution };
            data.size = new Vector3(Size, HeightRange, Size);

            var heights = new float[Resolution, Resolution];
            var washDepth = new float[Resolution, Resolution];
            var mound = new float[Resolution, Resolution];
            var noise = new Noise(seed);
            Vector2 flat = new Vector2(Size / 2f, Size / 2f); // flat area is at the terrain centre
            for (int zi = 0; zi < Resolution; zi++)
                for (int xi = 0; xi < Resolution; xi++)
                {
                    float x = xi * Size / (Resolution - 1), z = zi * Size / (Resolution - 1);
                    float h = HeightAt(x, z, noise, out washDepth[zi, xi], out mound[zi, xi]);
                    // Flatten the course area (slightly below y = 0 so the course pad sits on top).
                    float dx = Mathf.Max(0f, Mathf.Abs(x - flat.x) - flatSize.x / 2f);
                    float dz = Mathf.Max(0f, Mathf.Abs(z - flat.y) - flatSize.y / 2f);
                    float blend = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(Mathf.Sqrt(dx * dx + dz * dz) / 25f));
                    h = Mathf.Lerp(BaseLevel - 0.05f, h, blend);
                    heights[zi, xi] = Mathf.Clamp01(h / HeightRange);
                }
            data.SetHeights(0, 0, heights);
            data.terrainLayers = Layers(seed);
            data.SetAlphamaps(0, 0, Splat(data, washDepth, mound, noise));

            var go = Terrain.CreateTerrainGameObject(data);
            go.name = "UtahDesertTerrain";
            go.transform.SetParent(parent, false);
            go.transform.position = new Vector3(flatCenter.x - Size / 2f, -BaseLevel, flatCenter.z - Size / 2f);
            var terrain = go.GetComponent<Terrain>();
            terrain.heightmapPixelError = 3f;
            terrain.basemapDistance = 300f;
            go.GetComponent<TerrainCollider>().sharedMaterial = EnvMaterials.Sand;

            // Visual-only desert floor beyond the terrain edge, so the world doesn't end in a cliff.
            var outer = GameObject.CreatePrimitive(PrimitiveType.Plane);
            outer.name = "OuterDesertFloor";
            outer.transform.SetParent(parent, false);
            outer.transform.position = new Vector3(flatCenter.x, -1.2f, flatCenter.z);
            outer.transform.localScale = new Vector3(400f, 1f, 400f); // 4 km
            Object.DestroyImmediate(outer.GetComponent<Collider>());
            outer.GetComponent<MeshRenderer>().sharedMaterial = EnvMaterials.Color(RedSoil * 0.95f);

            ScatterRocks(terrain, parent, flatCenter, flatSize, seed);
            return terrain;
        }

        /// <summary>Terrain height (m) at terrain-local (x, z); also returns how much of a dry wash (0-1) and mound height is there (for texturing).</summary>
        static float HeightAt(float x, float z, Noise n, out float wash, out float moundHeight)
        {
            float h = BaseLevel;
            h += 2.5f * n.Fbm(x * 0.006f, z * 0.006f, 4);                 // broad rolling ground
            h += 0.12f * n.Fbm(x * 0.15f, z * 0.15f, 2);                  // small ripples

            // Rounded bentonite badland mounds, only in some regions.
            float region = Mathf.Clamp01((n.Value(x * 0.004f + 40f, z * 0.004f) - 0.52f) * 6f);
            float ridged = 1f - Mathf.Abs(n.Value(x * 0.02f, z * 0.02f) * 2f - 1f);
            moundHeight = region * 5f * ridged * ridged;
            h += moundHeight;

            // Mesas and buttes: flat tops, steep noisy cliffs.
            foreach (var m in Mesas)
            {
                float d = Vector2.Distance(new Vector2(x, z), new Vector2(m.x, m.y));
                float edge = m.z + 6f * (n.Value(x * 0.05f + 7f, z * 0.05f) - 0.5f);
                float t = Mathf.Clamp01((edge + 16f - d) / 16f);              // 16 m cliff + talus slope
                h = Mathf.Max(h, BaseLevel + m.w * Mathf.SmoothStep(0f, 1f, t * t));
            }

            // Two dry washes (sinuous channels), 2 m deep, ~6 m wide.
            float w1 = Mathf.Abs(z - (140f + 22f * Mathf.Sin(x * 0.025f) + 9f * Mathf.Sin(x * 0.07f + 1f)));
            float w2 = Mathf.Abs(x - (285f + 18f * Mathf.Sin(z * 0.03f + 2f)));
            wash = 2f * Mathf.Exp(-(w1 * w1) / 36f) + 1.6f * Mathf.Exp(-(w2 * w2) / 30f);
            h -= wash;
            return h;
        }

        /// <summary>The four texture layers (sand, red soil, bentonite, purple shale) with seeded noise textures.</summary>
        static TerrainLayer[] Layers(int seed)
        {
            TerrainLayer Layer(Color c, float variation, float tile, int s) => new TerrainLayer
            {
                diffuseTexture = EnvMaterials.NoiseTexture(c, variation, 6f, seed + s),
                tileSize = new Vector2(tile, tile),
                smoothness = 0.05f,
            };
            return new[]
            {
                Layer(PaleSand, 0.10f, 6f, 1),   // 0 wash bottoms, sandy flats
                Layer(RedSoil, 0.14f, 8f, 2),    // 1 red sandy soil (most of the ground)
                Layer(Bentonite, 0.08f, 10f, 3), // 2 grey bentonite mounds / strata
                Layer(PurpleShale, 0.12f, 9f, 4) // 3 purple shale strata
            };
        }

        /// <summary>Texture-blend weights per splat cell: washes get sand, mounds bentonite, steep slopes shale, the rest red soil.</summary>
        static float[,,] Splat(TerrainData data, float[,] wash, float[,] mound, Noise n)
        {
            var map = new float[SplatResolution, SplatResolution, 4];
            for (int zi = 0; zi < SplatResolution; zi++)
                for (int xi = 0; xi < SplatResolution; xi++)
                {
                    float u = (float)xi / (SplatResolution - 1), v = (float)zi / (SplatResolution - 1);
                    int hx = Mathf.RoundToInt(u * (Resolution - 1)), hz = Mathf.RoundToInt(v * (Resolution - 1));
                    float height = data.GetInterpolatedHeight(u, v);
                    float slope = data.GetSteepness(u, v);
                    var w = new float[4];
                    w[1] = 1f;                                                                   // red soil
                    // Pale sand in washes, plus soft wind-blown sand drifts.
                    w[0] = Mathf.Clamp01(wash[hz, hx] * 0.9f) + 0.8f * Mathf.Clamp01((n.Value(u * 12f + 9f, v * 12f) - 0.58f) * 2.5f);
                    w[2] = Mathf.Clamp01(mound[hz, hx] / 2f);                                    // bentonite mounds
                    if (slope > 22f)
                    {
                        // Horizontal colour bands on steep faces, like layered Morrison/Mancos strata.
                        // Bands depend on height (horizontal layers); low-frequency noise only wobbles them.
                        int band = Mathf.FloorToInt((height + 0.8f * n.Value(u * 4f, v * 4f)) / 2.2f) % 3;
                        float steep = Mathf.Clamp01((slope - 22f) / 12f);
                        float[] strata = band == 0 ? new[] { 0f, 1f, 0f, 0f } : band == 1 ? new[] { 0f, 0f, 0f, 1f } : new[] { 0f, 0f, 1f, 0f };
                        for (int k = 0; k < 4; k++) w[k] = Mathf.Lerp(w[k], strata[k] * 2f, steep);
                    }
                    float sum = w[0] + w[1] + w[2] + w[3];
                    for (int k = 0; k < 4; k++) map[zi, xi, k] = w[k] / sum;
                }
            return map;
        }

        /// <summary>Scatter rocks with colliders over the terrain, keeping the flat course area clear.</summary>
        static void ScatterRocks(Terrain terrain, Transform parent, Vector3 flatCenter, Vector2 flatSize, int seed)
        {
            var root = new GameObject("DesertRocks").transform;
            root.SetParent(parent, false);
            var rng = new System.Random(seed * 31 + 5);
            var meshes = new Mesh[12];
            for (int i = 0; i < meshes.Length; i++) meshes[i] = ProceduralMeshes.Rock(seed * 100 + i);
            var materials = new[]
            {
                EnvMaterials.Color(new Color(0.55f, 0.38f, 0.28f)), EnvMaterials.Color(new Color(0.45f, 0.40f, 0.37f)),
                EnvMaterials.Color(new Color(0.70f, 0.58f, 0.44f)),
            };
            var data = terrain.terrainData;
            Vector3 origin = terrain.transform.position;
            int placed = 0;
            for (int attempt = 0; attempt < 6000 && placed < 700; attempt++)
            {
                float u = (float)rng.NextDouble(), v = (float)rng.NextDouble();
                Vector3 world = origin + new Vector3(u * Size, 0f, v * Size);
                if (Mathf.Abs(world.x - flatCenter.x) < flatSize.x / 2f + 3f && Mathf.Abs(world.z - flatCenter.z) < flatSize.y / 2f + 3f)
                    continue; // keep the course area clear
                float slope = data.GetSteepness(u, v);
                // More rocks on slopes and below cliffs (talus), fewer on open flats.
                float chance = 0.12f + Mathf.Clamp01(slope / 35f) * 0.8f;
                if (rng.NextDouble() > chance) continue;
                bool boulder = rng.NextDouble() < 0.06;
                float size = boulder ? 0.9f + (float)rng.NextDouble() * 1.4f : 0.08f + (float)(rng.NextDouble() * rng.NextDouble()) * 0.6f;
                world.y = terrain.SampleHeight(world) + origin.y + size * 0.15f;
                var scale = new Vector3(size * (0.8f + (float)rng.NextDouble() * 0.6f), size * (0.6f + (float)rng.NextDouble() * 0.4f), size * (0.8f + (float)rng.NextDouble() * 0.6f));
                var rock = TestCourse.RockObject(root, meshes[rng.Next(meshes.Length)], materials[rng.Next(materials.Length)],
                                                 root.InverseTransformPoint(world), scale, (float)rng.NextDouble() * 360f);
                rock.name = boulder ? $"Boulder_{placed:000}" : $"Rock_{placed:000}";
                placed++;
            }
        }

        /// <summary>Seeded value noise / fBm (independent of UnityEngine.Random state).</summary>
        sealed class Noise
        {
            /// <summary>Random X offset into the Perlin noise field (from the seed).</summary>
            readonly float ox;
            /// <summary>Random Z offset into the Perlin noise field (from the seed).</summary>
            readonly float oz;
            /// <summary>Noise field for a seed (same seed = same terrain).</summary>
            public Noise(int seed)
            {
                var rng = new System.Random(seed);
                ox = (float)rng.NextDouble() * 1000f;
                oz = (float)rng.NextDouble() * 1000f;
            }
            /// <summary>Perlin noise in 0..1 at (x, z).</summary>
            public float Value(float x, float z) => Mathf.PerlinNoise(ox + x, oz + z);
            /// <summary>Fractal Brownian motion: `octaves` layers of noise, each at double frequency and half amplitude; result about -1..1.</summary>
            public float Fbm(float x, float z, int octaves)
            {
                float sum = 0f, amp = 1f, norm = 0f;
                for (int i = 0; i < octaves; i++)
                {
                    sum += amp * (Value(x, z) - 0.5f) * 2f;
                    norm += amp;
                    amp *= 0.5f; x *= 2.03f; z *= 2.03f;
                }
                return sum / norm;
            }
        }
    }
}
