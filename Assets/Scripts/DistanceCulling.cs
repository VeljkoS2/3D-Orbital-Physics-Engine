using UnityEngine;

public class DistanceCulling : MonoBehaviour
{
    public GameObject mainBody;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        GameObject[] rings2D = GameObject.FindGameObjectsWithTag("2DRing");
        foreach (GameObject ring in rings2D)
        {
            if(ring.GetComponent<DistanceCulling>().mainBody.transform.position.magnitude> ring.GetComponent<DistanceCulling>().mainBody.GetComponent<Properties>().ring.y/GlobalProperties.scale*2*0.8) ring.GetComponent<MeshRenderer>().enabled = true;
            else ring.GetComponent<MeshRenderer>().enabled = false;
        }
    }
}
