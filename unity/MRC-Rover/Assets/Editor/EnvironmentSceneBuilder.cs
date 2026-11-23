// Creates Assets/Scenes/RoverTestEnvironment.unity: an almost empty scene whose EnvironmentGenerator
// builds the Utah desert, test course and rover when you press Play. Keeping generation at runtime
// means the scene is tiny and nothing generated needs to be committed.
//
// Menu: MRC > Create Test Environment Scene
// Batch: Unity -batchmode -projectPath unity/MRC-Rover -executeMethod EnvironmentSceneBuilder.CreateFromCommandLine [-variant tuned]
using System.Linq;
using MRC.Environment;
using MRC.Feasibility;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>Creates the (almost empty) test-environment scene; its EnvironmentGenerator builds everything when you press Play.</summary>
public static class EnvironmentSceneBuilder
{
    /// <summary>Where the scene is saved.</summary>
    public const string ScenePath = "Assets/Scenes/RoverTestEnvironment.unity";

    /// <summary>Menu MRC &gt; Create Test Environment Scene (robot variant 'tuned').</summary>
    [MenuItem("MRC/Create Test Environment Scene")]
    public static void CreateFromMenu()
    {
        if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
        Create("tuned");
        EditorSceneManager.OpenScene(ScenePath);
    }

    /// <summary>Batch entry point: -executeMethod EnvironmentSceneBuilder.CreateFromCommandLine [-variant name]; exits Unity with 0 or 1.</summary>
    public static void CreateFromCommandLine()
    {
        int code = 0;
        try { Create(FeasibilityPaths.Arg("-variant") ?? "tuned"); }
        catch (System.Exception e) { Debug.LogException(e); code = 1; }
        EditorApplication.Exit(code);
    }

    /// <summary>Create and save the scene with one EnvironmentGenerator set to load Assets/Robots/&lt;variant&gt;/.</summary>
    public static void Create(string variant)
    {
        System.IO.Directory.CreateDirectory("Assets/Scenes");
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        var environment = new GameObject("Environment");
        var generator = environment.AddComponent<EnvironmentGenerator>();
        generator.robotVariant = variant;

        var camera = new GameObject("Main Camera") { tag = "MainCamera" };
        camera.AddComponent<Camera>().farClipPlane = 1500f;
        camera.AddComponent<AudioListener>();
        camera.transform.position = new Vector3(0f, 3f, -8f);
        camera.transform.LookAt(Vector3.zero);

        EditorSceneManager.SaveScene(scene, ScenePath);
        var scenes = EditorBuildSettings.scenes.Where(s => s.path != ScenePath).ToList();
        scenes.Insert(0, new EditorBuildSettingsScene(ScenePath, true));
        EditorBuildSettings.scenes = scenes.ToArray();
        AssetDatabase.SaveAssets();
        Debug.Log($"[Environment] Created {ScenePath} (robot variant '{variant}'). Open it and press Play.");
    }
}
