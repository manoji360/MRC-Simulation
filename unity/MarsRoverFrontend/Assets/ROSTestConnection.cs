using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSTestConnection : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<StringMsg>("/unity_test");

        Debug.Log("ROS Publisher registered!");
    }

    void Update()
    {
        if (Keyboard.current != null &&
            Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            StringMsg message = new StringMsg("Hello from Unity!");

            ros.Publish("/unity_test", message);

            Debug.Log("Message sent to ROS 2!");
        }
    }
}
