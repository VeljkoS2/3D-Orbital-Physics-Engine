using UnityEngine;

public class ClickManager : MonoBehaviour
{
    GameObject gameManager = null;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        gameManager = GameObject.FindGameObjectWithTag("GameManager");
    }

    // Update is called once per frame
    void Update()
    {

    }
}
