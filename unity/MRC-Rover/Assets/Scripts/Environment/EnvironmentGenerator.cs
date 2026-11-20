using System.Collections.Generic;
using System.Linq;
using MRC.Feasibility;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace MRC.Environment
{
    /// <summary>Everything the environment builder created, for drivers, tests and the HUD.</summary>
    public class EnvironmentInfo
    {
        /// <summary>The test-course lanes (empty if the course was not built).</summary>
        public List<CourseLane> Lanes = new List<CourseLane>();
        /// <summary>The desert terrain (null if not built).</summary>
        public Terrain Terrain;
        /// <summary>Where the rover starts: the start of the first lane.</summary>
        public Vector3 StartPad;
        /// <summary>Desert start point east of the course (drive facing +X, across a dry wash).</summary>
        public Vector3 DesertStart;
    }

    /// <summary>
    /// Builds the baseline rover test environment at runtime (so nothing generated needs to be
    /// committed): Utah-desert terrain, a test course, sun/sky, and (in the editor) the imported rover
    /// with keyboard driving, a follow camera and a HUD. Earth gravity, 100 Hz physics.
    /// </summary>
    public class EnvironmentGenerator : MonoBehaviour
    {
        /// <summary>Folder under Assets/Robots/ created by RoverImport (run_feasibility or the import command).</summary>
        [Tooltip("Folder under Assets/Robots/ created by RoverImport (run_feasibility or the import command).")]
        public string robotVariant = "tuned";
        /// <summary>Build the Utah desert terrain.</summary>
        public bool buildDesert = true;
        /// <summary>Build the test course.</summary>
        public bool buildCourse = true;
        /// <summary>Spawn the imported rover with keyboard driving, camera and HUD.</summary>
        public bool spawnRover = true;
        /// <summary>Seed for the terrain (same seed = same terrain).</summary>
        public int terrainSeed = 7;
        /// <summary>Connect the rover to ROS 2 (ros2/README.md): /cmd_vel in, /odom out. Also enabled by the -ros command-line flag.</summary>
        [Tooltip("Connect the rover to ROS 2 (ros2/README.md): /cmd_vel in, /odom out. Also enabled by the -ros command-line flag.")]
        public bool enableRos = false;

        /// <summary>Physics step of the scene: 0.01 s (100 Hz).</summary>
        public const float FixedTimestep = 0.01f;
        /// <summary>World position of the test course's first lane.</summary>
        public static readonly Vector3 CourseOrigin = Vector3.zero;

        /// <summary>What Build() created (lanes, terrain, start points).</summary>
        public EnvironmentInfo Info { get; private set; }

        /// <summary>On Play: 100 Hz physics, Earth gravity, lighting, the environment, then the rover.</summary>
        void Awake()
        {
            Time.fixedDeltaTime = FixedTimestep;
            Physics.gravity = new Vector3(0f, -9.81f, 0f);
            SetUpLighting();
            Info = Build(transform, buildDesert, buildCourse, terrainSeed);
            if (spawnRover) SpawnRover();
        }

        /// <summary>Build terrain and/or course under `parent`. Also used by the automated tests.</summary>
        public static EnvironmentInfo Build(Transform parent, bool desert, bool course, int seed = 7)
        {
            var info = new EnvironmentInfo();
            float courseWidth = 8 * TestCourse.LaneSpacing + 4f;
            var courseCenter = CourseOrigin + new Vector3((7 * TestCourse.LaneSpacing) / 2f, 0f, TestCourse.LaneLength / 2f - 5f);
            var flatSize = new Vector2(courseWidth + 16f, TestCourse.LaneLength + 26f);
            if (course) info.Lanes = TestCourse.Build(parent, CourseOrigin);
            if (desert) info.Terrain = DesertTerrain.Build(parent, courseCenter, flatSize, seed);
            if (!desert && !course)
            {
                var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
                ground.transform.SetParent(parent, false);
                ground.transform.localScale = new Vector3(50f, 1f, 50f);
                ground.GetComponent<Collider>().sharedMaterial = EnvMaterials.Concrete;
            }
            info.StartPad = info.Lanes.Count > 0 ? info.Lanes[0].Start : CourseOrigin;
            // Past the flattened course area and its 25 m blend, ~18 m before the eastern dry wash.
            info.DesertStart = courseCenter + new Vector3(courseWidth / 2f + 45f, 0f, 0f);
            return info;
        }

        /// <summary>Sun, procedural desert sky, ambient light and haze.</summary>
        public static void SetUpLighting()
        {
            var sun = new GameObject("Sun").AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.color = new Color(1f, 0.95f, 0.86f);
            sun.intensity = 1.0f;
            sun.shadows = LightShadows.Soft;
            sun.transform.rotation = Quaternion.Euler(48f, -35f, 0f);
            RenderSettings.sun = sun;

            var sky = Shader.Find("Skybox/Procedural");
            if (sky != null)
            {
                var skybox = new Material(sky);
                skybox.SetColor("_SkyTint", new Color(0.55f, 0.65f, 0.85f));
                skybox.SetColor("_GroundColor", new Color(0.62f, 0.45f, 0.33f));
                skybox.SetFloat("_AtmosphereThickness", 0.8f);
                RenderSettings.skybox = skybox;
            }
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.48f, 0.53f, 0.62f);
            RenderSettings.ambientEquatorColor = new Color(0.52f, 0.43f, 0.36f);
            RenderSettings.ambientGroundColor = new Color(0.32f, 0.23f, 0.18f);
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogColor = new Color(0.78f, 0.70f, 0.62f);
            RenderSettings.fogDensity = 0.003f;
        }

        /// <summary>Load the imported rover (prefab + manifest) from Assets/Robots/&lt;robotVariant&gt;/, place it on the start pad, and add driving, camera, HUD and optionally ROS.</summary>
        void SpawnRover()
        {
            GameObject prefab = null;
            JObject manifest = null;
#if UNITY_EDITOR
            string folder = $"Assets/Robots/{robotVariant}";
            prefab = UnityEditor.AssetDatabase.FindAssets("t:Prefab", new[] { folder })
                .Select(UnityEditor.AssetDatabase.GUIDToAssetPath)
                .Where(p => System.IO.Path.GetDirectoryName(p).Replace('\\', '/') == folder)
                .Select(UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>)
                .FirstOrDefault();
            var manifestAsset = UnityEditor.AssetDatabase.LoadAssetAtPath<TextAsset>($"{folder}/manifest.json");
            if (manifestAsset != null) manifest = JObject.Parse(manifestAsset.text);
#endif
            if (prefab == null || manifest == null)
            {
                Debug.LogWarning($"[Environment] No imported robot in Assets/Robots/{robotVariant}/. " +
                                 "Run unity/run_feasibility (or RoverImport) first; the environment loads without a rover.");
                gameObject.AddComponent<RoverHud>().Message = $"No rover found in Assets/Robots/{robotVariant}/ - run unity/run_feasibility first.";
                return;
            }

            var rig = new RoverRig(manifest);
            rig.Spawn(prefab);
            rig.ConfigureDrives();
            rig.PlaceAt(Info.StartPad, 0f);

            var teleop = gameObject.AddComponent<RoverTeleop>();
            teleop.Init(rig, Info);
            var cam = Camera.main != null ? Camera.main : new GameObject("Main Camera") { tag = "MainCamera" }.AddComponent<Camera>();
            cam.farClipPlane = 1500f;
            cam.gameObject.AddComponent<FollowCamera>().Target = rig.Root.transform;
            if (enableRos || RosRoverBridge.RequestedOnCommandLine())
            {
                teleop.Ros = gameObject.AddComponent<RosRoverBridge>();
                teleop.Ros.Init(rig, teleop);
            }
            gameObject.AddComponent<RoverHud>().Teleop = teleop;
        }
    }
}
