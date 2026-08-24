using UnityEngine;

public class OrbitRenderer : MonoBehaviour
{
    public static float orbitThickness = 7f;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        foreach(var body in GlobalProperties.bodies)
        {
            if (body.GetComponent<Properties>().dominantBody == null) continue;
            GlobalProperties.orbitRenderers[body].startWidth = ComputeLineThickness(orbitThickness, body.transform.position.magnitude, Camera.main);
            GlobalProperties.orbitRenderers[body].endWidth = ComputeLineThickness(2f, body.transform.position.magnitude, Camera.main);
            GlobalProperties.orbitRenderers[body].SetPositions(body.GetComponent<Properties>().GetOrbitPoint());
        }
    }

    // Update is called once per frame
    void Update()
    {
        foreach (var body in GlobalProperties.bodies)
        {
            if (body.transform.position.magnitude > Camera.main.farClipPlane*2) GlobalProperties.orbitRenderers[body].enabled = false;
            else GlobalProperties.orbitRenderers[body].enabled = true;
        }
    }

    public static float ComputeLineThickness(float t, float distance, Camera cam)
    {
        float frustumHeightAtDistance = 2.0f * distance * Mathf.Tan(cam.fieldOfView * 0.5f * Mathf.Deg2Rad);
        float worldUnitsPerPixel = frustumHeightAtDistance / Screen.height;
        return t * worldUnitsPerPixel;
    }
}
