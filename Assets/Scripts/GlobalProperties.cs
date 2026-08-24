using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.UI;
using UnityEngine.VFX;

public class GlobalProperties : MonoBehaviour
{
    public static double scale = 10_000_000;
    public static List<GameObject> bodies = new List<GameObject>();
    public static Dictionary<GameObject, Image> crosshairs = new Dictionary<GameObject, Image>();
    public static Dictionary<GameObject, LineRenderer> orbitRenderers = new Dictionary<GameObject, LineRenderer>();
    public static List<Transform> stars = new List<Transform>();
    public static List<GameObject> blackHoles = new List<GameObject>();
    public static double3 offset = new double3(0,0,0);
    public static double timeScale = 1;
    public static GameObject focus = null;
    public static bool focused = false;
    public static bool offsetChanged = false;
    public static bool BodyAdded = true;
    public static GameObject closestBody = null;
    public static GameObject closestStar = null;
    public static double distanceToClosestBody = double.PositiveInfinity;
    public static double distanceToClosestStar = double.PositiveInfinity;
    public double userSetTimeScale { get; set; } = 1;
    public static bool paused = false;
    public Image Crosshair = null;
    Transform overlayTransform = null;
    public static double totalTime = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        userSetTimeScale = 1;
        overlayTransform = GameObject.FindGameObjectWithTag("Overlay").transform;
        distanceToClosestBody = double.PositiveInfinity;
        distanceToClosestStar = double.PositiveInfinity;
        List<GameObject> newBodies = new List<GameObject>(GameObject.FindGameObjectsWithTag("Body"));
        foreach (GameObject body in newBodies)
        {
            if (!bodies.Contains(body))
            {
                bodies.Add(body);
                if(body.GetComponent<Properties>().type == 2)
                {
                    Light starLight = body.AddComponent<Light>();
                    starLight.type = LightType.Directional;
                    starLight.intensity = 0.001f;
                    starLight.shadows = LightShadows.Soft;
                }
                if (body.GetComponent<Properties>().type == 2 && closestStar == null) closestStar = body; 
            }
            if(!crosshairs.ContainsKey(body))
            {
                Image crosshair = Instantiate(Crosshair, overlayTransform);
                crosshair.GetComponent<Crosshair>().body = body;
                crosshairs.Add(body, crosshair);
            }
            if(!orbitRenderers.ContainsKey(body))
            {
                LineRenderer lr = body.AddComponent<LineRenderer>();
                lr.enabled = false;
                lr.loop = true;
                lr.useWorldSpace = true;
                lr.positionCount = 200;
                lr.alignment = LineAlignment.View;
                lr.material = new Material(Shader.Find("Sprites/Default"));
                Gradient colorGradient = new Gradient();
                GradientColorKey[] colorKeys = new GradientColorKey[2];
                colorKeys[0] = new GradientColorKey(UnityEngine.Color.white, 0.0f);
                colorKeys[1] = new GradientColorKey(UnityEngine.Color.white, 1.0f);
                GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
                alphaKeys[0] = new GradientAlphaKey(1.0f, 0.0f);
                alphaKeys[1] = new GradientAlphaKey(0.05f, 1.0f);
                colorGradient.SetKeys(colorKeys, alphaKeys);
                lr.colorGradient = colorGradient;

                orbitRenderers.Add(body, lr);
            }
        }
        if (bodies.Count > 0)
        {
            focus = bodies[0];
            focused = true;
            gameObject.GetComponent<ApplyOffset>().distanceFromFocus = focus.GetComponent<Properties>().radius * 10;
            closestBody = bodies[0];
        }
    }

    // Update is called once per frame
    void Update()
    {

    }

    void LateUpdate()
    {
        if (BodyAdded)
        {
            List<GameObject> newBodies = new List<GameObject>(GameObject.FindGameObjectsWithTag("Body"));
            foreach (GameObject body in newBodies)
            {
                if (!bodies.Contains(body))
                {
                    bodies.Add(body);
                    if (body.GetComponent<Properties>().type == 2)
                    {
                        stars.Add(body.transform);
                    }
                }
                if (!crosshairs.ContainsKey(body))
                {
                    Image crosshair = Instantiate(Crosshair, overlayTransform);
                    crosshairs.Add(body, crosshair);
                }
            }
            BodyAdded = false;
        }
        distanceToClosestBody = double.PositiveInfinity;
        foreach (GameObject body in bodies)
        {
            if (distanceToClosestBody >= body.transform.position.magnitude)
            {
                distanceToClosestBody = body.transform.position.magnitude;
                closestBody = body;
            }
            crosshairs[body].transform.position = Camera.main.WorldToScreenPoint(body.transform.position);
            if (crosshairs[body].transform.position.z >= 0)
            {
                crosshairs[body].enabled = true;
                crosshairs[body].transform.position = new Vector3(crosshairs[body].transform.position.x, crosshairs[body].transform.position.y, 0);
            }
            else
            {
                crosshairs[body].enabled=false;
            }
        }
        distanceToClosestStar = double.PositiveInfinity;
        foreach (Transform t in stars)
        {
            if (distanceToClosestStar >= t.position.magnitude)
            {
                distanceToClosestStar = t.position.magnitude;
                closestStar = t.gameObject;
            }
        }
        timeScale = userSetTimeScale;
        if (timeScale == 0) paused = true;
        else paused = false;
        foreach(var b in bodies)
        {
            if(b.GetComponent<Properties>().dominantBody != null)
                orbitRenderers[b].enabled = true;
            else orbitRenderers[b].enabled=false;
            orbitRenderers[b].enabled = false;
        }
        totalTime += Time.deltaTime * timeScale;
    }

    public void SetUserSetTimeScale(float value)
    {
        userSetTimeScale = math.pow(10, value);
        GameObject[] rings = GameObject.FindGameObjectsWithTag("Ring");
        foreach (var r in rings)
        {
            if(r.TryGetComponent<VisualEffect>(out VisualEffect VE))
            VE.SetFloat("TimeScale", (float)userSetTimeScale);
        }
    }
}
