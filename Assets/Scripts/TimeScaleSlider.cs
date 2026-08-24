using Unity.Mathematics;
using UnityEngine;

public class TimeScaleSlider : MonoBehaviour
{
    GameObject gameManager = null;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        gameManager = GameObject.FindGameObjectWithTag("GameManager");
        gameObject.GetComponent<UnityEngine.UI.Slider>().value = math.log10((float)gameManager.GetComponent<GlobalProperties>().userSetTimeScale);
    }

    // Update is called once per frame
    void Update()
    {
        if (GlobalProperties.paused) gameObject.GetComponent<UnityEngine.UI.Slider>().value = 0;
    }
}
