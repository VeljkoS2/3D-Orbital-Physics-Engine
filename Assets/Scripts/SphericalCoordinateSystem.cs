using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Mathematics;

public class SphericalCoordinateSystem : MonoBehaviour
{
    GameObject centerBody;
    public UnityEngine.UI.Slider timeScaleSlider;
    public GameObject coordinateSphere;
    GameObject coordinateSphereInstance;
    Properties centerBodyProperties;
    Material coordinateSphereMaterial;
    bool placing = false;
    float prevTimeScaleSliderValue;

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
            prevTimeScaleSliderValue = timeScaleSlider.value;
            timeScaleSlider.value = 0;
            centerBody = GlobalProperties.focus;
            centerBodyProperties = centerBody.GetComponent<Properties>();
            coordinateSphereInstance = Instantiate(coordinateSphere, centerBody.transform);
            coordinateSphereInstance.transform.localScale = new Vector3(1.0f, 1.0f, 1.0f); //new Vector3((float)(centerBodyProperties.radius / GlobalProperties.scale) * 2.0f, (float)(centerBodyProperties.radius / GlobalProperties.scale) * 2.0f, (float)(centerBodyProperties.radius / GlobalProperties.scale) * 2.0f);
        }
        else if ((GlobalProperties.focused && Keyboard.current.eKey.wasPressedThisFrame && placing) || (GlobalProperties.focus != centerBody && placing))
        {
            Destroy(coordinateSphereInstance);
            timeScaleSlider.value = prevTimeScaleSliderValue;
            centerBody = null;
            centerBodyProperties = null;
            coordinateSphereMaterial = null;
            placing = false;
        }
        /*
        if(placing)
        {
            centerBodyProperties = centerBody.GetComponent<Properties>();
            coordinateSphereMaterial = coordinateSphereInstance.GetComponent<Renderer>().material;
            coordinateSphereMaterial.SetFloat("_cellSize", math.pow(2, math.round(math.log2(1.0f / (math.length(coordinateSphereInstance.transform.position) - (float)(centerBodyProperties.radius * 5.0f / GlobalProperties.scale))))));
        }*/
    }
}
