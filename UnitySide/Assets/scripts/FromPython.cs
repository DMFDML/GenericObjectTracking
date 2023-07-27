using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine.UI;
using System.Collections.Generic;


public class FromPython : MonoBehaviour
{
    public string ipAddress = "127.0.0.1";  // Replace with the actual IP address
    public int port = 5065;  // Replace with the actual port number

    private UdpClient client;

    public Text text;
    private int frame = 0;
    private float totalTime = 0f;
    private float targetTime = 1f;

    private Vector3 centre;
    private Quaternion rotation;


    private List<Vector3> positions = new List<Vector3>(6);
    private List<Vector3> rotations = new List<Vector3>(6);

    private Quaternion average_rotation;
    private Vector3 average_position;
    private float[] x = new float[6];

    private void Start()
    {
        // Create a UDP client and start receiving data asynchronously
        client = new UdpClient(port);
        client.BeginReceive(ReceiveCallback, null);
    }

    private void OnDestroy()
    {
        // Close the UDP client when the script is destroyed
        if (client != null)
        {
            client.Close();
        }
    }

    private void rollingAverage(){
        if(rotations.Count > 6){
            rotations.RemoveAt(rotations.Count - 1);
            positions.RemoveAt(positions.Count - 1);
        }
        
        var rotation_sum = new Vector3(0,0,0);
        var position_sum = new Vector3(0,0,0);
        for(int i = 0; i < rotations.Count; i++){
            rotation_sum += rotations[i];
            position_sum += positions[i];
        }
        average_rotation = Quaternion.Euler(rotation_sum / 10);
        average_position = position_sum / 10;
    }

    private void ReceiveCallback(IAsyncResult result)
    {
        // Get the received data and remote endpoint
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        byte[] receivedData = client.EndReceive(result, ref remoteEndPoint);

        // Start receiving data again
        client.BeginReceive(ReceiveCallback, null);

        // Process the received data
        string json = Encoding.UTF8.GetString(receivedData);
        DataPacket data = JsonUtility.FromJson<DataPacket>(json);

        
        rotation = new Quaternion(data.box.rotation[1], data.box.rotation[2], data.box.rotation[3], data.box.rotation[0]);
        centre = new Vector3(data.box.centre[0], data.box.centre[1], data.box.centre[2]);

        // print(new Vector3(data.box.rotation[0], data.box.rotation[1], data.box.rotation[2]));
        // rollingAverage(rotation, centre);
        // rotations.Insert(0, rotation);
        // positions.Insert(0, centre);
    
        // Process the received data
        frame++;
        
        
    }

    void Update(){
        // Debug.Log(average_rotation);
        // rollingAverage();
        
        transform.rotation = rotation;
        transform.position = centre;

        totalTime += Time.deltaTime;

        if (totalTime >= targetTime)
        {
            float fps = frame / totalTime;
            text.text = "FPS: " + fps.ToString("F2");

            // Reset counters for the next interval
            totalTime = 0f;
            frame = 0;
        }
    }
}



[System.Serializable]
public class Box {
    public float[] rotation;
    public float[] centre;
}

[System.Serializable]
public class DataPacket
{
    public Box box;
}

