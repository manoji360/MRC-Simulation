// RoverTeleop.cs
// Drive the Gazebo rover from Unity with WASD.
//
// How it works:
//   Unity (this script) --WebSocket JSON--> rosbridge --/cmd_vel--> joy_to_drive --> drive_controller --> Gazebo
//
// /cmd_vel values are a -1..1 fraction of joy_to_drive's max speed
// (3.7 m/s forward, 4 rad/s turning). joy_to_drive stops the rover if it
// gets nothing for 2 seconds, so this script publishes continuously.
//
// Controls (click the Game view first so it gets the keys):
//   W / S   forward / backward
//   A / D   turn left / right
//   W + A etc. work together
//   Space   stop immediately

using System;
using System.Globalization;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class RoverTeleop : MonoBehaviour
{
    [Header("rosbridge connection")]
    public string rosbridgeUrl = "ws://localhost:9090";
    public string topic = "/cmd_vel";

    [Header("Top speed (fraction of max: 1.0 = 3.7 m/s or 4 rad/s)")]
    [Range(0f, 1f)] public float maxLinear = 0.3f;    // about 1.1 m/s
    [Range(0f, 1f)] public float maxAngular = 0.3f;   // about 1.2 rad/s

    [Header("Smoothing (how fast speed changes, fraction per second)")]
    public float linearAccel = 0.3f;    // about 1 s to reach top speed
    public float angularAccel = 0.5f;   // about 0.6 s to reach top speed

    [Header("Publishing")]
    public float publishHz = 20f;

    private ClientWebSocket ws;
    private CancellationTokenSource cts;
    private bool sending = false;
    private float linear = 0f;
    private float angular = 0f;
    private float nextPublishTime = 0f;
    private string status = "not connected";
    private int sentCount = 0;          // how many publish messages actually went out
    private string lastReply = "(none)"; // last message rosbridge sent back to us

    // Runs once when you press Play.
    async void Start()
    {
        // Keep the connection alive when Unity is not the focused window.
        Application.runInBackground = true;
        await Connect();
    }

    async Task Connect()
    {
        ws = new ClientWebSocket();
        cts = new CancellationTokenSource();
        status = "connecting to " + rosbridgeUrl + " ...";
        try
        {
            await ws.ConnectAsync(new Uri(rosbridgeUrl), cts.Token);
            status = "connected";
            Debug.Log("[RoverTeleop] connected to rosbridge at " + rosbridgeUrl);

            // Listen for anything rosbridge sends back (errors show up here).
            ReceiveLoop();

            // Tell rosbridge which topic and message type we will publish.
            await Send("{\"op\":\"advertise\",\"topic\":\"" + topic +
                       "\",\"type\":\"geometry_msgs/msg/Twist\"}");
        }
        catch (Exception e)
        {
            status = "connect failed: " + e.Message;
            Debug.LogWarning("[RoverTeleop] " + status +
                             "  (is rosbridge running?)");
        }
    }

    // Runs every frame.
    void Update()
    {
        // 1. Read the keyboard and work out the target speed.
        float targetLinear = 0f;
        float targetAngular = 0f;
        if (Input.GetKey(KeyCode.W)) targetLinear += maxLinear;
        if (Input.GetKey(KeyCode.S)) targetLinear -= maxLinear;
        if (Input.GetKey(KeyCode.A)) targetAngular += maxAngular;   // positive = turn left
        if (Input.GetKey(KeyCode.D)) targetAngular -= maxAngular;

        // 2. Move the current speed toward the target a little each frame,
        //    so the rover speeds up and slows down smoothly.
        linear = Mathf.MoveTowards(linear, targetLinear, linearAccel * Time.deltaTime);
        angular = Mathf.MoveTowards(angular, targetAngular, angularAccel * Time.deltaTime);

        // Space skips the smoothing and stops right away.
        if (Input.GetKey(KeyCode.Space))
        {
            linear = 0f;
            angular = 0f;
        }

        // 3. Send to ROS publishHz times per second.
        if (Time.time >= nextPublishTime)
        {
            nextPublishTime = Time.time + 1f / publishHz;
            Publish(linear, angular);
        }
    }

    async void Publish(float lin, float ang)
    {
        if (ws == null || ws.State != WebSocketState.Open || sending) return;

        // Build the JSON by plain string joining (no string.Format), because
        // Unity's C# runtime handles "{2:F3}}}" differently from desktop .NET.
        // InvariantCulture makes sure decimals use "." and not ",".
        string linStr = lin.ToString("F3", CultureInfo.InvariantCulture);
        string angStr = ang.ToString("F3", CultureInfo.InvariantCulture);
        string json =
            "{\"op\":\"publish\",\"topic\":\"" + topic + "\",\"msg\":{" +
            "\"linear\":{\"x\":" + linStr + ",\"y\":0.0,\"z\":0.0}," +
            "\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":" + angStr + "}}}";
        if (sentCount == 0) Debug.Log("[RoverTeleop] first publish: " + json);
        await Send(json);
        sentCount++;
    }

    // Print every message rosbridge sends back to the Unity Console.
    async void ReceiveLoop()
    {
        var buffer = new byte[8192];
        try
        {
            while (ws != null && ws.State == WebSocketState.Open)
            {
                var result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cts.Token);
                if (result.MessageType == WebSocketMessageType.Close) break;
                lastReply = Encoding.UTF8.GetString(buffer, 0, result.Count);
                Debug.LogWarning("[RoverTeleop] rosbridge says: " + lastReply);
            }
        }
        catch (Exception e)
        {
            if (!cts.IsCancellationRequested)
                Debug.LogWarning("[RoverTeleop] receive stopped: " + e.Message);
        }
        status = "disconnected";
    }

    async Task Send(string json)
    {
        sending = true;
        try
        {
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            await ws.SendAsync(new ArraySegment<byte>(bytes),
                               WebSocketMessageType.Text, true, cts.Token);
        }
        catch (Exception e)
        {
            status = "send failed: " + e.Message;
            Debug.LogWarning("[RoverTeleop] " + status);
        }
        finally
        {
            sending = false;
        }
    }

    // Shows the status in the top-left corner of the Game view.
    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 800, 25), "rosbridge: " + status);
        GUI.Label(new Rect(10, 30, 800, 25),
            "linear " + linear.ToString("F2", CultureInfo.InvariantCulture) +
            "   angular " + angular.ToString("F2", CultureInfo.InvariantCulture) +
            "      WASD to drive, Space to stop");
        GUI.Label(new Rect(10, 50, 800, 25), "messages sent: " + sentCount);
        GUI.Label(new Rect(10, 70, 1200, 25), "last reply from rosbridge: " + lastReply);
    }

    // Runs when you press Stop: send one zero command, then close.
    void OnApplicationQuit()
    {
        if (ws == null) return;
        try
        {
            if (ws.State == WebSocketState.Open)
            {
                string stop = "{\"op\":\"publish\",\"topic\":\"" + topic + "\",\"msg\":{" +
                              "\"linear\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}," +
                              "\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}}";
                byte[] bytes = Encoding.UTF8.GetBytes(stop);
                ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text,
                             true, CancellationToken.None).Wait(300);
            }
        }
        catch (Exception) { }
        cts?.Cancel();
        ws.Dispose();
    }
}