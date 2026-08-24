using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;

public class Crosshair : MonoBehaviour, IPointerClickHandler
{
    public GameObject body = null;
    GameObject gameManager = null;

    public void OnPointerClick(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Middle)
        {
            if(GlobalProperties.focus != body)
            {
                Camera.main.GetComponent<Move>().ResetView();
                GlobalProperties.focus = body;
                GlobalProperties.focused = true;
                gameManager.GetComponent<ApplyOffset>().distanceFromFocus = GlobalProperties.focus.GetComponent<Properties>().radius * 10;
            }
            else
            {
                GlobalProperties.focus = null;
                GlobalProperties.focused = false;
            }
        }
    }
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
