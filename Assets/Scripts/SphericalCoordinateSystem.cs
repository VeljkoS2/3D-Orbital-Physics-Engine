using System;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using static UnityEditor.ShaderGraph.Internal.KeywordDependentCollection;

public class SphericalCoordinateSystem : MonoBehaviour
{
    GameObject centerBody;
    public GameObject chosenBody;
    GameObject chosenBodyInstance;
    public UnityEngine.UI.Slider timeScaleSlider;
    public GameObject coordinateSphere;
    GameObject coordinateSphereInstance;
    GameObject placedBodySphereInstance;
    public GameObject placedBodySphere;
    Properties centerBodyProperties;
    public bool placing = false;
    double pxToWorld;
    bool settingSize;
    bool placingBody;
    bool leftPressed;
    bool decidingVelocityMag;
    bool decidingVelocityDir;
    double chosenVelMag;
    public bool finished;
    double velMagConst;
    Vector3 chosenVelDir;
    RaycastHit hit;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (GlobalProperties.focused && Keyboard.current.eKey.wasPressedThisFrame && !placing)
        {
            placing = true;
            if (timeScaleSlider.value != 0)
                GlobalProperties.prevTimeScaleSliderValue = timeScaleSlider.value;
            timeScaleSlider.value = 0;
            timeScaleSlider.enabled = false;
            centerBody = GlobalProperties.focus;
            centerBodyProperties = centerBody.GetComponent<Properties>();
            coordinateSphereInstance = Instantiate(coordinateSphere, centerBody.transform);
            coordinateSphereInstance.transform.localScale = new Vector3(1.0f, 1.0f, 1.0f); 
            settingSize = true;
            placedBodySphereInstance = Instantiate(placedBodySphere, centerBody.transform);
            placedBodySphereInstance.GetComponent<MeshRenderer>().enabled = false;
            placedBodySphereInstance.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
            chosenVelMag = 0;
            chosenVelDir = new Vector3();
        }
        else if ((GlobalProperties.focused && Keyboard.current.eKey.wasPressedThisFrame && placing) || (GlobalProperties.focus != centerBody && GlobalProperties.focus != chosenBodyInstance && placing) || finished)
        {
            Destroy(coordinateSphereInstance);
            Destroy(placedBodySphereInstance);
            timeScaleSlider.value = GlobalProperties.prevTimeScaleSliderValue;
            if ((GlobalProperties.focus == centerBody || GlobalProperties.focus == chosenBodyInstance) && !finished)
            {
                GlobalProperties.focus = centerBody;
            }
            centerBody = null;
            centerBodyProperties = null;
            placing = false;
            settingSize = false;
            placingBody = false;
            leftPressed = false;
            decidingVelocityDir = false;
            decidingVelocityMag = false;
            pxToWorld = 1.0;
            velMagConst = 1.0;
            if (!finished)
            {
                Destroy(GlobalProperties.crosshairs[chosenBodyInstance]);
                GlobalProperties.bodies.Remove(chosenBodyInstance);
                GlobalProperties.blackHoles.Remove(chosenBodyInstance);
                GlobalProperties.stars.Remove(chosenBodyInstance.transform);
                GlobalProperties.crosshairs.Remove(chosenBodyInstance);
                GlobalProperties.orbitRenderers.Remove(chosenBodyInstance);
                Destroy(chosenBodyInstance);
            }
            chosenBodyInstance = null;
            coordinateSphereInstance = null;
            placedBodySphereInstance = null;
            timeScaleSlider.enabled = true;
            finished = false;
        }
        if (placing && settingSize)
        {
            pxToWorld = math.length(centerBody.transform.position) / (Screen.height / 2.0 * Mathf.Tan(Camera.main.fieldOfView * Mathf.Deg2Rad));
            double radiusToPx = centerBodyProperties.radius / GlobalProperties.scale / pxToWorld;
            float mouseDistancePx = math.length(new Vector2(Mouse.current.position.value.x - Screen.width / 2.0f, Mouse.current.position.value.y - Screen.height / 2.0f));
            double newSphereScale = mouseDistancePx / radiusToPx;
            coordinateSphereInstance.transform.localScale = new Vector3((float)newSphereScale, (float)newSphereScale, (float)newSphereScale);
            double circularOrbitVelocityMagnitude = math.sqrt(Constants.G * centerBodyProperties.mass / (centerBodyProperties.radius * coordinateSphereInstance.transform.localScale.x));
            chosenVelMag = circularOrbitVelocityMagnitude;
            placedBodySphereInstance.transform.localScale = new Vector3(coordinateSphereInstance.transform.localScale.x / 2.0f, coordinateSphereInstance.transform.localScale.x / 2.0f, coordinateSphereInstance.transform.localScale.x / 2.0f);
            velMagConst = coordinateSphereInstance.transform.localScale.x / 2.0 * centerBodyProperties.radius / circularOrbitVelocityMagnitude;
        }
        if (placing && settingSize && Mouse.current.leftButton.wasPressedThisFrame && !leftPressed)
        {
            settingSize = false;
            placingBody = true;
            leftPressed = true;
            placedBodySphereInstance.GetComponent<MeshRenderer>().enabled = true;
            coordinateSphereInstance.GetComponent<Renderer>().material.SetFloat("_alpha", 0.2f);
        }
        if (placing && placingBody)
        {
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Mouse.current.position.value), out hit))
            {
                placedBodySphereInstance.transform.position = hit.point;
                placedBodySphereInstance.GetComponent<Properties>().worldPosition = new double3(hit.point.x * GlobalProperties.scale - GlobalProperties.offset.x, hit.point.y * GlobalProperties.scale - GlobalProperties.offset.y, hit.point.z * GlobalProperties.scale - GlobalProperties.offset.z);                
            }
        }
        if (placing && placingBody && Mouse.current.leftButton.wasPressedThisFrame && !leftPressed)
        {
            placingBody = false;
            leftPressed = true;
            decidingVelocityDir = true;
            coordinateSphereInstance.GetComponent<SphereCollider>().enabled = false;
            chosenBodyInstance = Instantiate(chosenBody);
            chosenBodyInstance.GetComponent<Properties>().worldPosition = placedBodySphereInstance.GetComponent<Properties>().worldPosition;
            chosenBodyInstance.GetComponent<MeshRenderer>().enabled = false;
            placedBodySphereInstance.GetComponent<SphereCollider>().enabled = true;
            placedBodySphereInstance.GetComponent<Renderer>().material.SetFloat("_alpha", 0.2f);
            GlobalProperties.focus = chosenBodyInstance;
            GlobalProperties.BodyAdded = true;
        }
        if (placing && decidingVelocityDir)
        {
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Mouse.current.position.value), out hit))
            {
                placedBodySphereInstance.GetComponent<LineRenderer>().SetPosition(0, placedBodySphereInstance.transform.position);
                placedBodySphereInstance.GetComponent<LineRenderer>().SetPosition(1, hit.point);
                float frustumHeightAtDistance = 2.0f * math.length(placedBodySphereInstance.transform.position) * Mathf.Tan(Camera.main.fieldOfView * 0.5f * Mathf.Deg2Rad);
                float worldUnitsPerPixel = frustumHeightAtDistance / Screen.height;
                placedBodySphereInstance.GetComponent<LineRenderer>().startWidth = (float)(4.0 * worldUnitsPerPixel);
                placedBodySphereInstance.GetComponent<LineRenderer>().endWidth = (float)(4.0 * worldUnitsPerPixel);
                chosenVelDir = math.normalize(placedBodySphereInstance.GetComponent<LineRenderer>().GetPosition(1) - placedBodySphereInstance.GetComponent<LineRenderer>().GetPosition(0));
                chosenBodyInstance.GetComponent<Properties>().velocity = new double3(chosenVelDir.x * chosenVelMag, chosenVelDir.y * chosenVelMag, chosenVelDir.z * chosenVelMag) + centerBodyProperties.velocity;
            }
        }
        if (placing && decidingVelocityDir && Mouse.current.leftButton.wasPressedThisFrame && !leftPressed)
        {
            decidingVelocityDir = false;
            leftPressed = true;
            decidingVelocityMag = true;
            placedBodySphereInstance.GetComponent<SphereCollider>().enabled = false;
            placedBodySphereInstance.GetComponent<Renderer>().material.SetFloat("_alpha", 1.0f);
        }
        if (placing && decidingVelocityMag)
        {
            pxToWorld = math.length(centerBody.transform.position) / (Screen.height / 2.0 * Mathf.Tan(Camera.main.fieldOfView * Mathf.Deg2Rad));
            double radiusToPx = centerBodyProperties.radius / GlobalProperties.scale / pxToWorld;
            float mouseDistancePx = math.length(new Vector2(Mouse.current.position.value.x - Screen.width / 2.0f, Mouse.current.position.value.y - Screen.height / 2.0f));
            double newSphereScale = mouseDistancePx / radiusToPx;
            placedBodySphereInstance.transform.localScale = new Vector3((float)newSphereScale, (float)newSphereScale, (float)newSphereScale);
            chosenVelMag = placedBodySphereInstance.transform.localScale.x * centerBodyProperties.radius / velMagConst;
            chosenBodyInstance.GetComponent<Properties>().velocity = new double3(chosenVelDir.x * chosenVelMag, chosenVelDir.y * chosenVelMag, chosenVelDir.z * chosenVelMag) + centerBodyProperties.velocity;
        }
        if (placing && decidingVelocityMag && Mouse.current.leftButton.wasPressedThisFrame && !leftPressed)
        {
            decidingVelocityMag = false;
            leftPressed = true;
            if(chosenBodyInstance.GetComponent<Properties>().type != 3)
                chosenBodyInstance.GetComponent<MeshRenderer>().enabled = true;
            finished = true;
            if (chosenBodyInstance.GetComponent<Properties>().type == 3)
            {
                GlobalProperties.blackHoles.Add(chosenBodyInstance);
            }
        }
        leftPressed = false;
    }
}