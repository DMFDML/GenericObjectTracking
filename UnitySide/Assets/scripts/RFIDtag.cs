using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class RFIDtag : MonoBehaviour
{
    public InputField input;
    public Text points;
    private int noPoints;
    public bool activate = true;
    private int[] rfidUsed = {0, 0, 0};
    void Start()
    {
        // input.Select();
        // input.ActivateInputField();
        points.text = "Points: 0";
        noPoints = 0;
    }
    void Update()
    {
        if(input.text == "1437055369" && rfidUsed[0] < 3) {
            noPoints += 10;
            points.text = "Points: " + noPoints.ToString();
            input.text = null;
            // input.Select();
            input.ActivateInputField();
            rfidUsed[0]++;
        } 
        else if(input.text == "1437198297" && rfidUsed[1] < 3) {
            noPoints += 20;
            points.text = "Points: " + noPoints.ToString();
            input.text = null;
            // input.Select();
            input.ActivateInputField();
            rfidUsed[1]++;
        } 
        else if(input.text == "1513049860" && rfidUsed[2] < 3) {
            noPoints += 30;
            points.text = "Points: " + noPoints.ToString();
            input.text = null;
            // input.Select();
            input.ActivateInputField();
            rfidUsed[2]++;
        } 
        if(!input.isFocused && activate){
            input.ActivateInputField();
        }
    }
}
