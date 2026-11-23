using System.Collections.Generic;
using UnityEngine;

namespace MRC.Feasibility
{
    /// <summary>
    /// Tyre-on-surface friction from the parameter registry (tyre_mu_concrete_static, ..._sand_..., ..._rock_...).
    /// The wheels get a material with friction 1 and combine mode Multiply. Unity uses the highest-priority
    /// combine mode of the two colliders (Multiply beats Average), so the contact friction is exactly
    /// 1 x the ground's value, which therefore *is* the tyre-on-that-surface coefficient.
    /// Ground materials register themselves with a surface name; Configure() sets their friction.
    /// </summary>
    public static class SurfaceFriction
    {
        /// <summary>Every registered ground material and its surface name ('concrete', 'sand', 'rock').</summary>
        static readonly Dictionary<PhysicsMaterial, string> surfaces = new Dictionary<PhysicsMaterial, string>();
        /// <summary>Parameters of the configured robot, or null when no robot has tyre friction data.</summary>
        static SimParameters parameters;

        /// <summary>Material for wheel colliders (friction 1, Multiply).</summary>
        public static readonly PhysicsMaterial Tyre = new PhysicsMaterial("Tyre")
        {
            staticFriction = 1f, dynamicFriction = 1f, frictionCombine = PhysicsMaterialCombine.Multiply,
            bounciness = 0f, bounceCombine = PhysicsMaterialCombine.Minimum,
        };

        /// <summary>True once a robot with tyre friction parameters has been configured.</summary>
        public static bool Active => parameters != null;

        /// <summary>Declare that `material` is a ground surface ("concrete", "sand", "rock").</summary>
        public static void Register(PhysicsMaterial material, string surface)
        {
            surfaces[material] = surface;
            Apply(material, surface);
        }

        /// <summary>Use these parameters for every registered (and future) surface.</summary>
        public static void Configure(SimParameters p)
        {
            parameters = p;
            foreach (var kv in surfaces) Apply(kv.Key, kv.Value);
        }

        /// <summary>(static, dynamic) tyre-on-surface friction for a surface from the parameters, or null if unknown.</summary>
        public static (float staticMu, float dynamicMu)? For(string surface)
        {
            if (parameters == null) return null;
            float? all = parameters.Float("tyre_mu_all");
            float? s = all ?? parameters.Float($"tyre_mu_{surface}_static");
            float? d = all.HasValue ? all * 0.93f : parameters.Float($"tyre_mu_{surface}_dynamic");
            if (!s.HasValue) return null;
            return (s.Value, d ?? s.Value);
        }

        /// <summary>Write the surface's tyre friction into a ground material (no change if the parameters do not cover it).</summary>
        static void Apply(PhysicsMaterial material, string surface)
        {
            var mu = For(surface);
            if (!mu.HasValue || material == null) return;
            material.staticFriction = mu.Value.staticMu;
            material.dynamicFriction = mu.Value.dynamicMu;
        }
    }
}
