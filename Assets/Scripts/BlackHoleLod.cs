using UnityEngine;
using System.Collections.Generic;
public class BlackHoleLod : MonoBehaviour
{
    List<Vector4> positions;
    List<float> radii;
    float count;
    Properties properties;
    Renderer _renderer;
    MaterialPropertyBlock _mpb;
    public float nearMultiplierTreshold = 5f;
    public float mediumMultiplierTreshold = 50f;
    public int nearStepCount = 150;
    public int mediumStepCount = 100;
    public float diskAngularVelocity = 0.001f; // tune to taste
    private float _diskRotationAngle = 0f;
    int frame = 0;
    void Start()
    {
        _renderer = GetComponent<Renderer>();
        _mpb = new MaterialPropertyBlock();
        properties = GetComponent<Properties>();
    }
    void Update()
    {
        positions = new List<Vector4>();
        radii = new List<float>();
        count = 0;
        for (int i = 0; i < GlobalProperties.blackHoles.Count; i++)
        {
            var props = GlobalProperties.blackHoles[i].GetComponent<Properties>();
            if (props == null)
            {
                Debug.LogError($"Black hole at index {i} ({GlobalProperties.blackHoles[i].name}) has no Properties component!");
                continue;
            }
            positions.Add(GlobalProperties.blackHoles[i].transform.position);
            radii.Add(props.blackHoleParamaters.schwarzschildRadiusUnits);
            count++;
        }
        BlackHoleParamaters bp = properties.blackHoleParamaters;
        float visualRadius = (float)(bp.schwarzschildRadiusUnits * 6.0);
        float dist = transform.position.magnitude;
        int stepCount;
        if (dist < visualRadius * nearMultiplierTreshold)
            stepCount = nearStepCount;
        else if (dist < visualRadius * mediumMultiplierTreshold)
            stepCount = mediumStepCount;
        else
            stepCount = 24;

        frame = (frame + 1) % 8;

        _diskRotationAngle += diskAngularVelocity * (float)GlobalProperties.timeScale * Time.deltaTime;
        _diskRotationAngle %= (2f * Mathf.PI);
        _renderer.GetPropertyBlock(_mpb);
        _mpb.SetFloat("_StepCount", stepCount);
        _mpb.SetVector("_BlackHolePosition", transform.position);
        _mpb.SetFloat("_DiskRotationOffset", _diskRotationAngle);
        _mpb.SetFloat("_TimeScale", (float)(GlobalProperties.totalTime+1e9f));
        _mpb.SetFloat("_FrameIndex", frame);
        _mpb.SetFloat("_BlackHoleMassKg", (float)bp.mass);
        _renderer.SetPropertyBlock(_mpb);
        Shader.SetGlobalFloat("_OtherBHCount", count);
    }
}