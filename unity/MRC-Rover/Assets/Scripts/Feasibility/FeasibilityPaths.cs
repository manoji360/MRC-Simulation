using System;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MRC.Feasibility
{
    /// <summary>Command-line arguments and result-file locations shared by import and tests.</summary>
    public static class FeasibilityPaths
    {
        public static string ProjectRoot => Path.GetDirectoryName(Application.dataPath);
        public static string ResultsRoot => Path.Combine(ProjectRoot, "FeasibilityResults");
        static string CurrentFile => Path.Combine(ResultsRoot, "current.json");

        /// <summary>Value after `name` on the command line, or null.</summary>
        public static string Arg(string name)
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase)) return args[i + 1];
            return null;
        }

        public static void WriteResult(string variant, string fileName, JObject data)
        {
            string dir = Path.Combine(ResultsRoot, variant);
            Directory.CreateDirectory(dir);
            File.WriteAllText(Path.Combine(dir, fileName), data.ToString());
        }

        public static void SetCurrentVariant(string variant, string prefabPath, string manifestPath)
        {
            Directory.CreateDirectory(ResultsRoot);
            var current = new JObject { ["variant"] = variant, ["prefab"] = prefabPath, ["manifest"] = manifestPath };
            File.WriteAllText(CurrentFile, current.ToString());
        }

        /// <summary>The variant to test: -variant argument, else the last imported one.</summary>
        public static JObject CurrentVariant()
        {
            if (!File.Exists(CurrentFile)) return null;
            var current = JObject.Parse(File.ReadAllText(CurrentFile));
            string requested = Arg("-variant");
            if (requested != null && requested != (string)current["variant"])
            {
                current["variant"] = requested;
                current["prefab"] = null; // resolved by the test from Assets/Robots/<variant>/
                current["manifest"] = $"Assets/Robots/{requested}/manifest.json";
            }
            return current;
        }
    }
}
