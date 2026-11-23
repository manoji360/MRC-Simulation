using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Newtonsoft.Json.Linq;

namespace MRC.Feasibility
{
    /// <summary>
    /// The robot's physical-parameter registry (manifest.physics.parameters, written by the pipeline from
    /// the config's physical_parameters), plus command-line overrides for experiments:
    ///   Unity ... -param wheel_max_torque_nm=20 -param center_of_mass_offset_m=0,0,0.05
    /// Extra experiment-only keys: tyre_mu_all (every surface), arm_accel_scale, controller_wheel_radius_m.
    /// </summary>
    public class SimParameters
    {
        /// <summary>manifest.physics.parameters: name -&gt; {value, unit, status, source, ...}.</summary>
        readonly JObject registry;
        /// <summary>-param name=value overrides from the command line.</summary>
        readonly Dictionary<string, string> overrides;

        /// <summary>Wrap the registry; commandLine defaults to Unity's own command-line arguments.</summary>
        public SimParameters(JObject registry, IEnumerable<string> commandLine = null)
        {
            this.registry = registry ?? new JObject();
            overrides = ParseOverrides(commandLine ?? Environment.GetCommandLineArgs());
        }

        /// <summary>`-param name=value` pairs from the command line.</summary>
        public static Dictionary<string, string> ParseOverrides(IEnumerable<string> args)
        {
            var result = new Dictionary<string, string>();
            var list = args.ToList();
            for (int i = 0; i < list.Count - 1; i++)
            {
                if (!string.Equals(list[i], "-param", StringComparison.OrdinalIgnoreCase)) continue;
                string pair = list[i + 1];
                int eq = pair.IndexOf('=');
                if (eq > 0) result[pair.Substring(0, eq).Trim()] = pair.Substring(eq + 1).Trim();
            }
            return result;
        }

        /// <summary>True if -param gave this key on the command line.</summary>
        public bool IsOverridden(string key) => overrides.ContainsKey(key);
        /// <summary>True if any registry entry or override starts with prefix (e.g. 'tyre_mu_').</summary>
        public bool HasAny(string prefix) =>
            registry.Properties().Any(p => p.Name.StartsWith(prefix)) || overrides.Keys.Any(k => k.StartsWith(prefix));

        /// <summary>A number: the override if given, else the registry value, else null.</summary>
        public float? Float(string key)
        {
            if (overrides.TryGetValue(key, out var text))
                return float.Parse(text, CultureInfo.InvariantCulture);
            var value = registry[key]?["value"];
            return value != null && (value.Type == JTokenType.Float || value.Type == JTokenType.Integer) ? (float?)value : null;
        }

        /// <summary>A vector like [dx, dy, dz]: the override ("a,b,c") if given, else the registry value, else null.</summary>
        public float[] Vector(string key)
        {
            if (overrides.TryGetValue(key, out var text))
                return text.Split(',').Select(s => float.Parse(s.Trim(), CultureInfo.InvariantCulture)).ToArray();
            return registry[key]?["value"] is JArray array ? array.Select(v => (float)v).ToArray() : null;
        }

        /// <summary>The registry value (ignoring overrides) as a vector, for working out override deltas.</summary>
        public float[] RegistryVector(string key) =>
            registry[key]?["value"] is JArray array ? array.Select(v => (float)v).ToArray() : null;

        /// <summary>What was used, for result files: every registry entry's value and status, and the overrides.</summary>
        public JObject Snapshot()
        {
            var snapshot = new JObject();
            foreach (var p in registry.Properties())
                snapshot[p.Name] = new JObject { ["value"] = p.Value["value"], ["status"] = p.Value["status"] };
            foreach (var kv in overrides)
                snapshot[kv.Key] = new JObject { ["value"] = kv.Value, ["status"] = "override (-param)" };
            return snapshot;
        }
    }
}
