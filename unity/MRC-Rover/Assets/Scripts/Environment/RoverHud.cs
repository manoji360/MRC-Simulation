using UnityEngine;
using UnityEngine.InputSystem;

namespace MRC.Environment
{
    /// <summary>Simple on-screen readout (IMGUI, no UI packages needed). Press H to hide the help.</summary>
    public class RoverHud : MonoBehaviour
    {
        /// <summary>The keyboard driver whose rover this HUD describes.</summary>
        public RoverTeleop Teleop;
        /// <summary>If set, the HUD shows only this message (e.g. 'no rover found').</summary>
        public string Message;

        /// <summary>Show the controls help box (toggle with H).</summary>
        bool showHelp = true;
        /// <summary>Smoothed frames per second.</summary>
        float fps;
        /// <summary>Text box style (created on first draw).</summary>
        GUIStyle style;

        /// <summary>Update the FPS estimate and toggle the help box with H.</summary>
        void Update()
        {
            fps = Mathf.Lerp(fps, 1f / Mathf.Max(Time.unscaledDeltaTime, 1e-4f), 0.05f);
            if (Keyboard.current != null && Keyboard.current.hKey.wasPressedThisFrame) showHelp = !showHelp;
        }

        /// <summary>Draw the readout (speeds, drive limits, pose, arm joint, physics rate, ROS status) and the controls help.</summary>
        void OnGUI()
        {
            style ??= new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, fontSize = 14, richText = true };
            if (!string.IsNullOrEmpty(Message))
            {
                GUI.Box(new Rect(10, 10, 620, 34), Message, style);
                return;
            }
            if (Teleop == null || Teleop.Rig == null) return;
            var root = Teleop.Rig.Root;
            Vector3 velocity = root.linearVelocity;
            float speed = Vector3.Dot(velocity, root.transform.forward);
            Vector3 e = root.transform.eulerAngles;
            float pitch = Mathf.DeltaAngle(0f, e.x), roll = Mathf.DeltaAngle(0f, e.z);
            string arm = Teleop.ArmJoints.Count > 0
                ? $"{Teleop.ArmJoints[Teleop.SelectedArmJoint]}  {Teleop.Rig.JointDegrees(Teleop.ArmJoints[Teleop.SelectedArmJoint]):F1} deg"
                : "none";
            string text =
                $"<b>{Teleop.Location}</b>\n" +
                $"Speed {speed:F2} m/s   controller {Teleop.Rig.DriveControl.Linear:F2}   command {Teleop.CommandSpeed:F2}   (scale {Teleop.speedScale:F1})\n" +
                $"Turn  controller {Teleop.Rig.DriveControl.Angular:F2}   command {Teleop.CommandTurn:F2} rad/s\n" +
                $"Drive: {Teleop.Rig.DriveControl.Describe()}\n" +
                $"Pitch {pitch:F1} deg   Roll {roll:F1} deg   Heading {e.y:F0} deg\n" +
                $"Position {root.transform.position.x:F1}, {root.transform.position.y:F2}, {root.transform.position.z:F1}\n" +
                $"Arm joint [{Teleop.SelectedArmJoint + 1}] {arm}\n" +
                $"Gravity {Physics.gravity.y:F2} m/s2   Physics {1f / Time.fixedDeltaTime:F0} Hz   {fps:F0} FPS" +
                (Teleop.Ros == null ? "" :
                    $"\nROS {(Teleop.Ros.Connected ? "connected" : "not connected")}   " +
                    $"{(Teleop.Ros.ExternalActive ? "<b>ROS driving</b>" : "keyboard")}   cmd_vel {Teleop.Ros.CmdCount}   odom {Teleop.Ros.OdomCount}   {Teleop.Ros.Status}");
            GUI.Box(new Rect(10, 10, 640, Teleop.Ros == null ? 160 : 180), text, style);
            if (showHelp)
            {
                const string help =
                    "<b>Controls</b> (H hides)\n" +
                    "W/S drive   A/D turn   [ ] speed scale   Space stop\n" +
                    "1-9 pick arm joint   Q/E move it   Z arm home\n" +
                    "T next test lane   G Utah desert   R reset\n" +
                    "Right mouse orbit   Scroll zoom   C chase/free camera";
                GUI.Box(new Rect(10, Teleop.Ros == null ? 176 : 196, 470, 100), help, style);
            }
        }
    }
}
