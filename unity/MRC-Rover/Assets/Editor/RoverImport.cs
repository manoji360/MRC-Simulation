// Imports a folder produced by `python -m urdf_pipeline prepare` into this project and
// saves it as a prefab, recording import metrics for the feasibility report.
//
// Command line (batch mode):
//   Unity -batchmode -projectPath unity/MRC-Rover -executeMethod RoverImport.ImportFromCommandLine
//         -prepared <prepared folder> -variant <name> [-convex vhacd|unity] -logFile <log>
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using MRC.Feasibility;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

public static class RoverImport
{
    public static void ImportFromCommandLine()
    {
        int exitCode = 0;
        try
        {
            string prepared = FeasibilityPaths.Arg("-prepared")
                ?? throw new ArgumentException("Missing -prepared <folder produced by urdf_pipeline prepare>");
            string variant = FeasibilityPaths.Arg("-variant") ?? "default";
            string convex = FeasibilityPaths.Arg("-convex") ?? "vhacd";
            Import(prepared, variant, convex);
        }
        catch (Exception e)
        {
            Debug.LogException(e);
            exitCode = 1;
        }
        EditorApplication.Exit(exitCode);
    }

    public static void Import(string preparedDir, string variant, string convex)
    {
        var errors = new List<string>();
        var warnings = new List<string>();
        void Capture(string message, string stack, LogType type)
        {
            if (type == LogType.Error || type == LogType.Exception || type == LogType.Assert) errors.Add(message);
            else if (type == LogType.Warning) warnings.Add(message);
        }

        preparedDir = Path.GetFullPath(preparedDir);
        var manifest = JObject.Parse(File.ReadAllText(Path.Combine(preparedDir, "manifest.json")));
        string[] outputs = manifest["outputs"].ToObject<string[]>();
        if (outputs.Length != 1)
            throw new NotSupportedException("The feasibility test supports single-part robots only (run prepare without articulation splitting).");

        // Copy the prepared folder into Assets/Robots/<variant>/ (the URDF Importer needs it inside Assets).
        string assetFolder = $"Assets/Robots/{variant}";
        string destination = Path.Combine(FeasibilityPaths.ProjectRoot, assetFolder);
        if (Directory.Exists(destination))
        {
            AssetDatabase.DeleteAsset(assetFolder);
        }
        CopyDirectory(preparedDir, destination);
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);

        var settings = new ImportSettings
        {
            chosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = convex == "unity" ? ImportSettings.convexDecomposer.unity : ImportSettings.convexDecomposer.vHACD,
        };

        Application.logMessageReceived += Capture;
        var timer = Stopwatch.StartNew();
        GameObject robot = null;
        try
        {
            var creator = UrdfRobotExtensions.Create(Path.Combine(destination, outputs[0]), settings, false);
            while (creator.MoveNext())
            {
                robot = creator.Current;
            }
        }
        finally
        {
            timer.Stop();
            Application.logMessageReceived -= Capture;
        }
        if (robot == null) throw new Exception("URDF Importer did not create a robot (see errors above).");

        // The importer's Controller overwrites every joint drive in Start(); the tests set drives themselves.
        var controller = robot.GetComponent<Controller>();
        if (controller != null) controller.enabled = false;

        int fixedInertials = ApplyExactInertia(robot, manifest);

        string prefabPath = $"{assetFolder}/{robot.name}.prefab";
        PrefabUtility.SaveAsPrefabAsset(robot, prefabPath);
        UnityEngine.Object.DestroyImmediate(robot);
        AssetDatabase.SaveAssets();

        var saved = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        var meshFilters = saved.GetComponentsInChildren<MeshFilter>(true);
        var metrics = new JObject
        {
            ["variant"] = variant,
            ["convex_method"] = convex,
            ["urdf"] = outputs[0],
            ["prefab"] = prefabPath,
            ["manifest"] = $"{assetFolder}/manifest.json",
            ["import_seconds"] = Math.Round(timer.Elapsed.TotalSeconds, 2),
            ["articulation_bodies"] = saved.GetComponentsInChildren<ArticulationBody>(true).Length,
            ["colliders"] = saved.GetComponentsInChildren<Collider>(true).Length,
            ["mesh_filters"] = meshFilters.Length,
            ["render_vertices"] = meshFilters.Where(m => m.sharedMesh != null).Sum(m => (long)m.sharedMesh.vertexCount),
            ["inertia_set_from_manifest"] = fixedInertials,
            ["error_count"] = errors.Count,
            ["warning_count"] = warnings.Count,
            ["errors"] = new JArray(errors.Distinct().Take(20)),
            ["warnings"] = new JArray(warnings.Distinct().Take(20)),
        };
        FeasibilityPaths.WriteResult(variant, "import.json", metrics);
        FeasibilityPaths.SetCurrentVariant(variant, prefabPath, $"{assetFolder}/manifest.json");
        Debug.Log($"[Feasibility] Imported {outputs[0]} as {prefabPath} in {timer.Elapsed.TotalSeconds:F1}s " +
                  $"({errors.Count} errors, {warnings.Count} warnings)");
    }

    /// <summary>
    /// The URDF Importer (v0.5.2) mis-rotates inertia tensors that are not diagonal in the link frame
    /// (its eigenvector-to-quaternion conversion). The pipeline computes principal moments and axes
    /// in Unity coordinates, so set them directly - on the body and on UrdfInertial, which re-applies
    /// its stored values in Start().
    /// </summary>
    static int ApplyExactInertia(GameObject robot, JObject manifest)
    {
        var links = manifest["physics"]?["links"] as JObject;
        if (links == null) return 0;
        int count = 0;
        foreach (var body in robot.GetComponentsInChildren<ArticulationBody>(true))
        {
            if (!(links[body.name] is JObject link) || link["principal_moments_unity"] == null) continue;
            var m = link["principal_moments_unity"];
            var q = link["principal_rotation_unity_xyzw"];
            var c = link["com_unity"];
            var moments = new Vector3((float)m[0], (float)m[1], (float)m[2]);
            var rotation = new Quaternion((float)q[0], (float)q[1], (float)q[2], (float)q[3]);
            var com = new Vector3((float)c[0], (float)c[1], (float)c[2]);
            body.centerOfMass = com;
            body.inertiaTensor = moments;
            body.inertiaTensorRotation = rotation;
            var inertial = body.GetComponent<UrdfInertial>();
            if (inertial != null)
            {
                inertial.useUrdfData = true;
                inertial.centerOfMass = com;
                inertial.inertiaTensor = moments;
                inertial.inertiaTensorRotation = rotation;
                inertial.inertialAxisRotation = Quaternion.identity;
            }
            count++;
        }
        return count;
    }

    static void CopyDirectory(string source, string destination)
    {
        foreach (string dir in Directory.GetDirectories(source, "*", SearchOption.AllDirectories))
            Directory.CreateDirectory(dir.Replace(source, destination));
        Directory.CreateDirectory(destination);
        foreach (string file in Directory.GetFiles(source, "*", SearchOption.AllDirectories))
            File.Copy(file, file.Replace(source, destination), true);
    }
}
