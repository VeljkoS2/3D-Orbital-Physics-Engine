using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using System.Collections.Generic;

public class ApplyOffset : MonoBehaviour
{
    public double distanceFromFocus = 1.0;
    Properties focusP = null;
    Transform bodyT = null;
    Properties bodyP = null;
    // Start is called once before the first execution of Update afters the MonoBehaviour is created
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (GlobalProperties.focused)
        {
            focusP = GlobalProperties.focus.GetComponent<Properties>();
            double3 tempPositionD = GlobalProperties.offset + focusP.worldPosition;
            Vector3 tempPosition = new Vector3((float)(tempPositionD.x/GlobalProperties.scale), (float)(tempPositionD.y/GlobalProperties.scale), (float)(tempPositionD.z/GlobalProperties.scale));
            if (tempPosition.magnitude < focusP.radius * 1.1/GlobalProperties.scale)
            {
                distanceFromFocus = focusP.radius * 1.11;
                GlobalProperties.offset = Camera.main.GetComponent<Move>().direction * distanceFromFocus - focusP.worldPosition;
                tempPositionD = GlobalProperties.offset + focusP.worldPosition;
                tempPosition = new Vector3((float)(tempPositionD.x / GlobalProperties.scale), (float)(tempPositionD.y / GlobalProperties.scale), (float)(tempPositionD.z / GlobalProperties.scale));
            }
            if (math.abs(tempPosition.magnitude - distanceFromFocus) > 0.01 && !GlobalProperties.offsetChanged)
            {  
                GlobalProperties.offset = Camera.main.GetComponent<Move>().direction  * distanceFromFocus - focusP.worldPosition;
            }
        }


        foreach (GameObject b in GlobalProperties.bodies)
        {
            bodyT = b.transform;
            bodyP = b.GetComponent<Properties>();
            bodyT.position = new Vector3((float)((GlobalProperties.offset.x + bodyP.worldPosition.x) / GlobalProperties.scale), (float)((GlobalProperties.offset.y + bodyP.worldPosition.y) / GlobalProperties.scale), (float)((GlobalProperties.offset.z + bodyP.worldPosition.z) / GlobalProperties.scale));
        }

        if (GlobalProperties.focused)
        {
            distanceFromFocus = math.length(GlobalProperties.offset + focusP.worldPosition);
        }
        GlobalProperties.offsetChanged = false;
        foreach (var body in GlobalProperties.bodies)
        {
            if (body.GetComponent<Properties>().dominantBody == null) continue;
            GlobalProperties.orbitRenderers[body].startWidth = OrbitRenderer.ComputeLineThickness(OrbitRenderer.orbitThickness, body.transform.position.magnitude, Camera.main);
            GlobalProperties.orbitRenderers[body].endWidth = OrbitRenderer.ComputeLineThickness(2f, body.transform.position.magnitude, Camera.main);
            GlobalProperties.orbitRenderers[body].SetPositions(body.GetComponent<Properties>().GetOrbitPoint());
        }
    }
}
