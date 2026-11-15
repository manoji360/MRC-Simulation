using UnityEngine;

namespace MRC.Environment
{
    /// <summary>Simple on-screen readout (IMGUI, no UI packages needed). Press H to hide the help.</summary>
    public class RoverHud : MonoBehaviour
    {
        public RoverTeleop Teleop;
        public string Message;

        bool showHelp = true;
        float fps;
        GUIStyle style;

        void Update()
        {
            fps = Mathf.Lerp(fps, 1f / Mathf.Max(Time.unscaledDeltaTime, 1e-4f), 0.05f);
            if (Input.GetKeyDown(KeyCode.H)) showHelp = !showHelp;
        }

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
                $"Speed {speed:F2} m/s   (command {Teleop.CommandSpeed:F2}, turn {Teleop.CommandTurn:F2} rad/s)\n" +
                $"Pitch {pitch:F1} deg   Roll {roll:F1} deg   Heading {e.y:F0} deg\n" +
                $"Position {root.transform.position.x:F1}, {root.transform.position.y:F2}, {root.transform.position.z:F1}\n" +
                $"Arm joint [{Teleop.SelectedArmJoint + 1}] {arm}\n" +
                $"Gravity {Physics.gravity.y:F2} m/s2   Physics {1f / Time.fixedDeltaTime:F0} Hz   {fps:F0} FPS";
            GUI.Box(new Rect(10, 10, 470, 120), text, style);
            if (showHelp)
            {
                const string help =
                    "<b>Controls</b> (H hides)\n" +
                    "W/S drive   A/D turn   Shift faster   Space stop\n" +
                    "1-9 pick arm joint   Q/E move it   Z arm home\n" +
                    "T next test lane   G Utah desert   R reset\n" +
                    "Right mouse orbit   Scroll zoom   C chase/free camera";
                GUI.Box(new Rect(10, 136, 470, 100), help, style);
            }
        }
    }
}
