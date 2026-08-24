using UnityEngine;

public class Atmosphere : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(!GlobalProperties.paused)
            transform.Rotate(0, -(float)(Time.deltaTime*GlobalProperties.timeScale/100), 0);
    }
}
