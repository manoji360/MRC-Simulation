using UnityEngine;
using UnityEngine.InputSystem;

namespace MRC.Environment
{
    /// <summary>
    /// Chase camera that follows the rover. Hold the right mouse button to orbit, scroll to zoom,
    /// press C to switch between chase (turns with the rover) and free orbit (fixed to the world).
    /// </summary>
    public class FollowCamera : MonoBehaviour
    {
        public Transform Target;
        public float distance = 4f;
        public float minDistance = 1.5f, maxDistance = 40f;
        public float height = 1.6f;
        public float smoothing = 6f;

        bool chase = true;
        float yaw, pitch = 18f;

        void LateUpdate()
        {
            if (Target == null) return;
            var kb = Keyboard.current;
            var mouse = Mouse.current;
            if (kb != null && kb.cKey.wasPressedThisFrame) chase = !chase;
            if (mouse != null)
            {
                if (mouse.rightButton.isPressed)
                {
                    Vector2 delta = mouse.delta.ReadValue();     // pixels this frame
                    yaw += delta.x * 0.2f;
                    pitch = Mathf.Clamp(pitch - delta.y * 0.15f, -5f, 80f);
                }
                float notches = mouse.scroll.ReadValue().y / 120f; // one wheel notch = 120 on Windows
                distance = Mathf.Clamp(distance * (1f - Mathf.Clamp(notches, -3f, 3f) * 0.1f), minDistance, maxDistance);
            }

            float baseYaw = chase ? Target.eulerAngles.y : 0f;
            var rotation = Quaternion.Euler(pitch, baseYaw + yaw, 0f);
            Vector3 focus = Target.position + Vector3.up * 0.4f;
            Vector3 wanted = focus - rotation * Vector3.forward * distance + Vector3.up * (height - 1f) * 0.3f;
            float k = 1f - Mathf.Exp(-smoothing * Time.deltaTime);
            transform.position = Vector3.Lerp(transform.position, wanted, k);
            transform.rotation = Quaternion.LookRotation(focus - transform.position, Vector3.up);
        }
    }
}
