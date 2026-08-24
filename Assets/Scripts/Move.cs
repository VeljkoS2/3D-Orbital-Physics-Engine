using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Mathematics;

public class Move : MonoBehaviour
{
    public float speed { get; set; } = 0.1f;
    public float baseSpeed = 0;
    public float rotateSpeed = 0.3f;
    float yaw, pitch;
    GameObject gameManager = null;
    public double3 direction { get; set; } = new double3(0,0,-1);
    int focusIndex = 0;

    public void ResetView()
    {
        direction = new double3(0,0,-1);
        yaw = 0;
        pitch = 0;
    }
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        gameManager = GameObject.FindGameObjectWithTag("GameManager");
        baseSpeed = speed;
        direction = new double3(0,0,-1);
    }

    // Update is called once per frame
    void Update()
    {
        if(Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            GlobalProperties.timeScale = 0;
            GlobalProperties.paused = !GlobalProperties.paused;
        }
        speed = (float)math.abs(GlobalProperties.distanceToClosestBody*GlobalProperties.scale- GlobalProperties.closestBody.GetComponent<Properties>().radius) / 100.0f;
        if (GlobalProperties.focus != null)
        {
            GlobalProperties.focused = true;
            speed = (float)(gameManager.GetComponent<ApplyOffset>().distanceFromFocus / 100);
        }
        else GlobalProperties.focused = false;

        if (Keyboard.current.leftArrowKey.wasPressedThisFrame)
        {
            focusIndex = GlobalProperties.bodies.IndexOf(GlobalProperties.focus);
            if (focusIndex == 0) focusIndex = GlobalProperties.bodies.Count - 1;
            else focusIndex--;
            GlobalProperties.focus = GlobalProperties.bodies[focusIndex];
            gameManager.GetComponent<ApplyOffset>().distanceFromFocus = GlobalProperties.focus.GetComponent<Properties>().radius * 10;
            ResetView();
        }

        if (Keyboard.current.rightArrowKey.wasPressedThisFrame)
        {
            focusIndex = GlobalProperties.bodies.IndexOf(GlobalProperties.focus);
            if (focusIndex == GlobalProperties.bodies.Count-1) focusIndex = 0;
            else focusIndex++;
            GlobalProperties.focus = GlobalProperties.bodies[focusIndex];
            gameManager.GetComponent<ApplyOffset>().distanceFromFocus = GlobalProperties.focus.GetComponent<Properties>().radius * 10;
            ResetView();
        }
        

        Vector3 move = Vector3.zero;

        if(!GlobalProperties.focused)
        {
            if (Keyboard.current.wKey.isPressed)
            {
                move -= transform.forward;
            }
            if (Keyboard.current.sKey.isPressed)
            {
                move += transform.forward;
            }
            if (Keyboard.current.aKey.isPressed)
            {
                move += transform.right;
            }
            if (Keyboard.current.dKey.isPressed)
            {
                move -= transform.right;
            }
            if (Keyboard.current.spaceKey.isPressed)
            {
                move -= transform.up;
            }
            if (Keyboard.current.ctrlKey.isPressed)
            {
                move += transform.up;
            }
            GlobalProperties.offset += new double3(move * speed);
        }

        move = Vector3.zero;
        if (Mouse.current.scroll.y.value < 0)
        {
            if (GlobalProperties.focused)
            {
                gameManager.GetComponent<ApplyOffset>().distanceFromFocus *= 1.2;
            }
            else
            {
                move += transform.forward * 20;
                GlobalProperties.offsetChanged = true;
            }
        }
        if (Mouse.current.scroll.y.value > 0)
        {
            if (GlobalProperties.focused)
            {
                gameManager.GetComponent<ApplyOffset>().distanceFromFocus *= 0.8;
                if (gameManager.GetComponent<ApplyOffset>().distanceFromFocus < GlobalProperties.focus.GetComponent<Properties>().radius * 1.1)
                    gameManager.GetComponent<ApplyOffset>().distanceFromFocus = GlobalProperties.focus.GetComponent<Properties>().radius * 1.11;
            }
            else
            {
                move -= transform.forward * 20;
                GlobalProperties.offsetChanged = true;
            }
        }

        GlobalProperties.offset += new double3(move * speed);

        move = Vector3.zero;
        if (Mouse.current.rightButton.isPressed)
        {
            if (!GlobalProperties.focused)
            {
                float x = Mouse.current.delta.x.ReadValue() * rotateSpeed;
                float y = Mouse.current.delta.y.ReadValue() * rotateSpeed;

                transform.Rotate(Vector3.up, x, Space.World);
                transform.Rotate(Vector3.right, -y, Space.Self);
            }
            else
            {
                float x = Mouse.current.delta.x.ReadValue() * rotateSpeed;
                float y = Mouse.current.delta.y.ReadValue() * rotateSpeed;

                yaw += x;
                pitch = math.clamp(pitch + y, -89f, 89f);

                Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);

                double3 focusWorldPos = GlobalProperties.focus.GetComponent<Properties>().worldPosition;

                direction = new double3(rotation * Vector3.back);

                GlobalProperties.offset = direction * gameManager.GetComponent<ApplyOffset>().distanceFromFocus - focusWorldPos;
            }
        }
        if (GlobalProperties.focused)
        {
            GlobalProperties.offset = direction * gameManager.GetComponent<ApplyOffset>().distanceFromFocus - GlobalProperties.focus.GetComponent<Properties>().worldPosition;
            transform.LookAt(GlobalProperties.focus.transform, Vector3.up);
        }
        
    }
    private void LateUpdate()
    {
        Camera.main.nearClipPlane = (float)math.clamp((GlobalProperties.distanceToClosestBody - GlobalProperties.closestBody.GetComponent<Properties>().radius/GlobalProperties.scale)/10, 1e-5f, 1000f);
        Camera.main.farClipPlane = (float)(GlobalProperties.distanceToClosestBody*1000);
    }
}
