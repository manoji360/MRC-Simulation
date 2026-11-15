// Renders preview images of the generated test environment (no Play mode needed).
// Batch (needs graphics, so no -nographics):
//   Unity -batchmode -projectPath unity/MRC-Rover -executeMethod EnvironmentSnapshot.CaptureFromCommandLine [-variant tuned]
// Images go to FeasibilityResults/screenshots/.
using System.IO;
using System.Linq;
using MRC.Environment;
using MRC.Feasibility;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class EnvironmentSnapshot
{
    [MenuItem("MRC/Capture Environment Screenshots")]
    public static void CaptureFromMenu() => Capture(FeasibilityPaths.Arg("-variant") ?? "tuned");

    public static void CaptureFromCommandLine()
    {
        int code = 0;
        try { Capture(FeasibilityPaths.Arg("-variant") ?? "tuned"); }
        catch (System.Exception e) { Debug.LogException(e); code = 1; }
        EditorApplication.Exit(code);
    }

    public static void Capture(string variant)
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        EnvironmentGenerator.SetUpLighting();
        var world = new GameObject("Environment");
        var info = EnvironmentGenerator.Build(world.transform, desert: true, course: true);

        // Place the imported rover (if any) on the start pad, purely for scale in the pictures.
        string folder = $"Assets/Robots/{variant}";
        var prefab = AssetDatabase.FindAssets("t:Prefab", new[] { folder })
            .Select(AssetDatabase.GUIDToAssetPath)
            .Where(p => Path.GetDirectoryName(p).Replace('\\', '/') == folder)
            .Select(AssetDatabase.LoadAssetAtPath<GameObject>).FirstOrDefault();
        if (prefab != null)
        {
            var rover = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            rover.transform.position = info.Lanes[2].Start + new Vector3(0f, 0.35f, 4f);
        }

        var camera = new GameObject("SnapshotCamera").AddComponent<Camera>();
        camera.farClipPlane = 2000f;
        camera.fieldOfView = 55f;
        string outDir = Path.Combine(FeasibilityPaths.ResultsRoot, "screenshots");
        Directory.CreateDirectory(outDir);

        var course = info.Lanes[0].Start;
        var shots = new (string name, Vector3 position, Vector3 lookAt)[]
        {
            ("01_course_overview", course + new Vector3(-12f, 9f, -10f), course + new Vector3(17f, 0f, 10f)),
            ("02_ramps_and_rover", info.Lanes[2].Start + new Vector3(-4f, 2.5f, -3f), info.Lanes[2].Start + new Vector3(2f, 0.4f, 8f)),
            ("03_rubble_and_rough", info.Lanes[6].Start + new Vector3(-3f, 2.5f, 0f), info.Lanes[7].Start + new Vector3(0f, 0f, 10f)),
            ("04_desert_ground", info.DesertStart + new Vector3(-6f, 3f, -6f), info.DesertStart + new Vector3(25f, 0f, 5f)),
            ("05_desert_mesas", info.DesertStart + new Vector3(0f, 25f, 0f), info.DesertStart + new Vector3(80f, 5f, 110f)),
            ("06_aerial", course + new Vector3(-150f, 160f, -150f), course + new Vector3(30f, 0f, 30f)),
        };
        var texture = new RenderTexture(1600, 900, 24);
        var image = new Texture2D(1600, 900, TextureFormat.RGB24, false);
        foreach (var shot in shots)
        {
            camera.transform.position = shot.position;
            camera.transform.LookAt(shot.lookAt);
            camera.targetTexture = texture;
            camera.Render();
            RenderTexture.active = texture;
            image.ReadPixels(new Rect(0, 0, 1600, 900), 0, 0);
            image.Apply();
            File.WriteAllBytes(Path.Combine(outDir, shot.name + ".png"), image.EncodeToPNG());
        }
        RenderTexture.active = null;
        camera.targetTexture = null;
        Debug.Log($"[Environment] Saved {shots.Length} screenshots to {outDir}");
    }
}
