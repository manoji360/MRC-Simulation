using System.Collections.Generic;
using UnityEngine;

namespace MRC.Environment
{
    /// <summary>Small procedural meshes for the test environment (no art assets needed).</summary>
    public static class ProceduralMeshes
    {
        /// <summary>A ramp wedge: `run` long (+Z), `rise` high at the far end, `width` wide (X), base at y = 0.</summary>
        public static Mesh Wedge(float width, float run, float rise)
        {
            float w = width / 2f;
            var v = new[]
            {
                new Vector3(-w, 0, 0), new Vector3(w, 0, 0),        // front bottom edge
                new Vector3(-w, 0, run), new Vector3(w, 0, run),    // back bottom
                new Vector3(-w, rise, run), new Vector3(w, rise, run), // back top
            };
            var tris = new[]
            {
                0, 4, 1, 1, 4, 5,   // slope
                2, 3, 4, 3, 5, 4,   // back face
                0, 1, 2, 1, 3, 2,   // bottom
                0, 2, 4,            // left side
                1, 5, 3,            // right side
            };
            return Flat(v, tris, "Wedge");
        }

        /// <summary>Low-poly irregular rock (unit size, centred), deterministic for a seed.</summary>
        public static Mesh Rock(int seed)
        {
            var rng = new System.Random(seed);
            var (vertices, triangles) = Icosahedron();
            triangles = Subdivide(vertices, triangles);
            float ox = (float)rng.NextDouble() * 100f, oz = (float)rng.NextDouble() * 100f;
            for (int i = 0; i < vertices.Count; i++)
            {
                Vector3 n = vertices[i].normalized;
                float bump = 0.75f + 0.45f * Mathf.PerlinNoise(ox + n.x * 1.7f + n.y, oz + n.z * 1.7f - n.y);
                vertices[i] = n * bump * 0.5f;
            }
            // Flatten the bottom a little so rocks sit like real ones.
            for (int i = 0; i < vertices.Count; i++)
                if (vertices[i].y < -0.25f) vertices[i] = new Vector3(vertices[i].x, -0.25f + (vertices[i].y + 0.25f) * 0.3f, vertices[i].z);
            return Flat(vertices.ToArray(), triangles.ToArray(), "Rock");
        }

        /// <summary>Bumpy ground patch: `width` (X) x `length` (Z), heights from `height(x, z)`.</summary>
        public static Mesh Heightfield(float width, float length, float cell, System.Func<float, float, float> height)
        {
            int nx = Mathf.Max(2, Mathf.RoundToInt(width / cell) + 1);
            int nz = Mathf.Max(2, Mathf.RoundToInt(length / cell) + 1);
            var v = new Vector3[nx * nz];
            var uv = new Vector2[nx * nz];
            for (int z = 0; z < nz; z++)
                for (int x = 0; x < nx; x++)
                {
                    float px = -width / 2f + x * width / (nx - 1), pz = z * length / (nz - 1);
                    v[z * nx + x] = new Vector3(px, height(px, pz), pz);
                    uv[z * nx + x] = new Vector2(px, pz);
                }
            var t = new List<int>();
            for (int z = 0; z < nz - 1; z++)
                for (int x = 0; x < nx - 1; x++)
                {
                    int i = z * nx + x;
                    t.AddRange(new[] { i, i + nx, i + 1, i + 1, i + nx, i + nx + 1 });
                }
            var mesh = new Mesh { name = "Heightfield", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            mesh.SetVertices(v);
            mesh.SetUVs(0, uv);
            mesh.SetTriangles(t, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        static Mesh Flat(Vector3[] vertices, int[] triangles, string name)
        {
            // Unshared vertices give crisp faceted shading.
            var v = new Vector3[triangles.Length];
            var t = new int[triangles.Length];
            for (int i = 0; i < triangles.Length; i++) { v[i] = vertices[triangles[i]]; t[i] = i; }
            var mesh = new Mesh { name = name, vertices = v, triangles = t };
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        static (List<Vector3>, List<int>) Icosahedron()
        {
            float p = (1f + Mathf.Sqrt(5f)) / 2f;
            var v = new List<Vector3>
            {
                new Vector3(-1, p, 0), new Vector3(1, p, 0), new Vector3(-1, -p, 0), new Vector3(1, -p, 0),
                new Vector3(0, -1, p), new Vector3(0, 1, p), new Vector3(0, -1, -p), new Vector3(0, 1, -p),
                new Vector3(p, 0, -1), new Vector3(p, 0, 1), new Vector3(-p, 0, -1), new Vector3(-p, 0, 1),
            };
            var t = new List<int>
            {
                0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1,
            };
            return (v, t);
        }

        /// <summary>Split each triangle into 4; adds midpoint vertices to `v`, returns the new triangles.</summary>
        static List<int> Subdivide(List<Vector3> v, List<int> t)
        {
            var cache = new Dictionary<long, int>();
            int Mid(int a, int b)
            {
                long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                if (cache.TryGetValue(key, out int i)) return i;
                v.Add((v[a] + v[b]) * 0.5f);
                return cache[key] = v.Count - 1;
            }
            var result = new List<int>();
            for (int i = 0; i < t.Count; i += 3)
            {
                int a = t[i], b = t[i + 1], c = t[i + 2];
                int ab = Mid(a, b), bc = Mid(b, c), ca = Mid(c, a);
                result.AddRange(new[] { a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca });
            }
            return result;
        }
    }
}
