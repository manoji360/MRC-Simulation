using UnityEngine;

namespace MRC.Environment
{
    /// <summary>Shared render and physics materials (Built-in render pipeline, created at runtime).</summary>
    public static class EnvMaterials
    {
        // Friction values: typical rover-testing estimates (rubber/plastic wheels on each surface).
        public static readonly PhysicsMaterial Concrete = Physics("Concrete", 0.9f, 0.8f);
        public static readonly PhysicsMaterial Sand = Physics("Desert sand", 0.65f, 0.55f);
        public static readonly PhysicsMaterial Rock = Physics("Rock", 0.8f, 0.7f);

        static PhysicsMaterial Physics(string name, float staticFriction, float dynamicFriction) =>
            new PhysicsMaterial(name)
            {
                staticFriction = staticFriction,
                dynamicFriction = dynamicFriction,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Average,
            };

        public static Material Color(Color color, float smoothness = 0.1f)
        {
            var shader = Shader.Find("Standard") ?? Shader.Find("Diffuse");
            var material = new Material(shader) { color = color };
            if (material.HasProperty("_Glossiness")) material.SetFloat("_Glossiness", smoothness);
            return material;
        }

        /// <summary>A tileable noisy texture around a base colour (for terrain layers).</summary>
        public static Texture2D NoiseTexture(Color baseColor, float variation, float scale, int seed, int size = 256)
        {
            var texture = new Texture2D(size, size, TextureFormat.RGBA32, true) { wrapMode = TextureWrapMode.Repeat };
            var pixels = new Color[size * size];
            float o = seed * 13.37f;
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    // Tileable noise: sample a torus in 4D via two offset 2D lookups.
                    float u = (float)x / size, v = (float)y / size;
                    float n = 0.6f * TileNoise(u, v, scale, o) + 0.4f * TileNoise(u, v, scale * 4f, o + 50f);
                    float k = 1f + variation * (n - 0.5f) * 2f;
                    // Alpha = smoothness for Unity's terrain shader: keep it low so sand looks matte, not wet.
                    pixels[y * size + x] = new Color(baseColor.r * k, baseColor.g * k, baseColor.b * k, 0.04f);
                }
            texture.SetPixels(pixels);
            texture.Apply(true);
            return texture;
        }

        static float TileNoise(float u, float v, float scale, float offset)
        {
            float a = Mathf.PerlinNoise(offset + u * scale, offset + v * scale);
            float b = Mathf.PerlinNoise(offset + (u - 1f) * scale, offset + v * scale);
            float c = Mathf.PerlinNoise(offset + u * scale, offset + (v - 1f) * scale);
            float d = Mathf.PerlinNoise(offset + (u - 1f) * scale, offset + (v - 1f) * scale);
            return Mathf.Lerp(Mathf.Lerp(a, b, u), Mathf.Lerp(c, d, u), v);
        }
    }
}
