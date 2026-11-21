using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class RoverKeyboardController : MonoBehaviour
{
    private ROSConnection ros;

    private const string TopicName = "/drive_controller/cmd_vel";

    public float linearSpeed = 0.5f;
    public float angularSpeed = 0.5f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<TwistStampedMsg>(TopicName);

        Debug.Log("Rover keyboard controller initialized!");
    }

    void Update()
    {
        if (Keyboard.current == null)
            return;

        float linear = 0f;
        float angular = 0f;

        if (Keyboard.current.wKey.isPressed)
            linear = linearSpeed;

        if (Keyboard.current.sKey.isPressed)
            linear = -linearSpeed;

        if (Keyboard.current.aKey.isPressed)
            angular = angularSpeed;

        if (Keyboard.current.dKey.isPressed)
            angular = -angularSpeed;

        TwistStampedMsg message = new TwistStampedMsg();

        message.header = new HeaderMsg();
        message.header.frame_id = "rover_base_origin";

        message.twist = new TwistMsg(
            new Vector3Msg(linear, 0, 0),
            new Vector3Msg(0, 0, angular)
        );

        ros.Publish(TopicName, message);
    }
}