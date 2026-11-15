using System.IO;
using System.Linq;
using MRC.Feasibility;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace MRC.Feasibility.Tests
{
    /// <summary>Loads the imported robot (prefab + pipeline manifest) that the tests run against.</summary>
    public static class TestRobot
    {
        public static (string variant, JObject manifest, GameObject prefab) Load()
        {
            var current = FeasibilityPaths.CurrentVariant();
            if (current == null) Assert.Ignore("No imported robot. Run unity/run_feasibility first.");
            string variant = (string)current["variant"];
            var manifest = JObject.Parse(File.ReadAllText(Path.Combine(FeasibilityPaths.ProjectRoot, (string)current["manifest"])));
            GameObject prefab = null;
#if UNITY_EDITOR
            string folder = $"Assets/Robots/{variant}";
            string prefabPath = (string)current["prefab"] ?? UnityEditor.AssetDatabase
                .FindAssets("t:Prefab", new[] { folder })
                .Select(UnityEditor.AssetDatabase.GUIDToAssetPath)
                .FirstOrDefault(p => Path.GetDirectoryName(p).Replace('\\', '/') == folder);
            prefab = prefabPath == null ? null : UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
#endif
            if (prefab == null) Assert.Ignore($"Prefab for variant '{variant}' not found. Run the import step first.");
            return (variant, manifest, prefab);
        }
    }
}
