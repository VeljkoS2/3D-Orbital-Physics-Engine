using UnityEngine;

public class RingScaleSetter : MonoBehaviour
{
    public double outerRadius;
    public Transform parent;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        float scale = (float)((outerRadius * 2) / (GlobalProperties.scale * parent.localScale.x * 10));
        transform.localScale = new Vector3(scale, scale, scale);
    }
}
