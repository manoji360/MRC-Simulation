using System.Globalization;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MRC.Feasibility
{
    /// <summary>
    /// Physics settings used by the feasibility tests. The timestep can be overridden on the command
    /// line (-dt 0.005); solver type, iterations and friction model come from
    /// ProjectSettings/DynamicsManager.asset (unity/physics_sweep.ps1 edits it per profile).
    /// </summary>
    public static class PhysicsTuning
    {
        /// <summary>Physics step used when no -dt is given: 0.01 s (100 Hz).</summary>
        public const float DefaultTimestep = 0.01f;

        /// <summary>Fixed physics timestep in seconds (-dt argument, default 0.01 = 100 Hz).</summary>
        public static float Timestep
        {
            get
            {
                string arg = FeasibilityPaths.Arg("-dt");
                return arg != null && float.TryParse(arg, NumberStyles.Float, CultureInfo.InvariantCulture, out float dt) && dt > 0f
                    ? dt : DefaultTimestep;
            }
        }

        /// <summary>Make every body of a robot use the project's default solver iteration counts.</summary>
        public static void ApplySolverIterations(RoverRig rig)
        {
            foreach (var body in rig.Links.Values)
            {
                body.solverIterations = Physics.defaultSolverIterations;
                body.solverVelocityIterations = Physics.defaultSolverVelocityIterations;
            }
        }

        /// <summary>Snapshot of the active physics configuration, stored with every result file.</summary>
        public static JObject Describe()
        {
            var info = new JObject
            {
                ["timestep_s"] = Timestep,
                ["rate_hz"] = Mathf.Round(1f / Timestep),
                ["gravity"] = Physics.gravity.y,
                ["solver_iterations"] = Physics.defaultSolverIterations,
                ["solver_velocity_iterations"] = Physics.defaultSolverVelocityIterations,
                ["default_contact_offset"] = Physics.defaultContactOffset,
                ["sleep_threshold"] = Physics.sleepThreshold,
                ["bounce_threshold"] = Physics.bounceThreshold,
                ["max_depenetration_velocity"] = Physics.defaultMaxDepenetrationVelocity,
            };
#if UNITY_EDITOR
            // Settings that are only readable through the project asset.
            var assets = UnityEditor.AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/DynamicsManager.asset");
            if (assets.Length > 0)
            {
                var settings = new UnityEditor.SerializedObject(assets[0]);
                string[] solverNames = { "PGS", "TGS" };
                string[] frictionNames = { "Patch", "OneDirectional", "TwoDirectional", "Strong" };
                var solver = settings.FindProperty("m_SolverType");
                var friction = settings.FindProperty("m_FrictionType");
                info["solver_type"] = solver == null ? null : solverNames[Mathf.Clamp(solver.intValue, 0, 1)];
                info["friction_type"] = friction == null ? null : frictionNames[Mathf.Clamp(friction.intValue, 0, 3)];
                info["improved_patch_friction"] = settings.FindProperty("m_ImprovedPatchFriction")?.boolValue;
                info["enhanced_determinism"] = settings.FindProperty("m_EnableEnhancedDeterminism")?.boolValue;
                info["broadphase_type"] = settings.FindProperty("m_BroadphaseType")?.intValue;
            }
#endif
            return info;
        }
    }
}
