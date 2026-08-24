using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class BlackHoleGlobalManager : MonoBehaviour
{
    public const int MaxBlackHoles = 8;

    public List<GameObject> blackHoles = GlobalProperties.blackHoles;

    static readonly int BHCountId = Shader.PropertyToID("_BHCount");
    static readonly int BHPositionsId = Shader.PropertyToID("_BHPositionsCamRelative");
    static readonly int BHParamsId = Shader.PropertyToID("_BHParams"); // x=rs, y=diskInner, z=diskOuter, w=escapeRadius
    static readonly int BHDiskNormalsId = Shader.PropertyToID("_BHDiskNormals");
    static readonly int BHRotationOffsetsId = Shader.PropertyToID("_BHRotationOffsets");
    public float diskAngularVelocity = 0.001f; // tune to taste
    private float _diskRotationAngle = 0f;
    Vector4[] diskNormals = new Vector4[MaxBlackHoles];
    Vector4[] rotationOffsets = new Vector4[MaxBlackHoles];

    public Cubemap skybox;
    public Texture2D noiseTexture;
    public Texture2D BlackHoleLut;
    public Texture2D BlackHoleLutCaptured;
    public Texture3D KerrRadialLut, KerrPolarLut; 
    public Texture2D KerrCritHeader;
    Vector4[] positions = new Vector4[MaxBlackHoles];
    Vector4[] parameters = new Vector4[MaxBlackHoles];

    void OnEnable() => RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
    void OnDisable() => RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;

    void OnBeginCameraRendering(ScriptableRenderContext context, Camera cam)
    {
        int count = Mathf.Min(blackHoles.Count, MaxBlackHoles);
        cam = Camera.main; // swap for your actual active camera reference if different
        _diskRotationAngle += diskAngularVelocity * (float)GlobalProperties.timeScale * Time.deltaTime;
        _diskRotationAngle %= (2f * Mathf.PI);

        for (int i = 0; i < count; i++)
        {
            var bh = blackHoles[i].GetComponent<Properties>().blackHoleParamaters;
            if (blackHoles[i].transform == null) continue;

            // Camera-relative position, matching your raymarch function's existing convention
            positions[i] = new Vector4(blackHoles[i].transform.position.x, blackHoles[i].transform.position.y, blackHoles[i].transform.position.z, 0);
            parameters[i] = new Vector4(bh.schwarzschildRadiusUnits, bh.diskInnerRadiusNormalized, bh.diskOuterRadiusNormalized, bh.escapeRadiusNormalized);
            diskNormals[i] = blackHoles[i].transform.up; // or whatever your disk normal convention is
            rotationOffsets[i] = new Vector4(_diskRotationAngle, (float)GlobalProperties.totalTime, 0, 0);
        }

        Shader.SetGlobalInt(BHCountId, count);
        Shader.SetGlobalVectorArray(BHPositionsId, positions);
        Shader.SetGlobalVectorArray(BHParamsId, parameters);
        Shader.SetGlobalVectorArray(BHDiskNormalsId, diskNormals);
        Shader.SetGlobalVectorArray(BHRotationOffsetsId, rotationOffsets);
        Shader.SetGlobalTexture("_BHSkybox", skybox);
        Shader.SetGlobalTexture("_BHNoise", noiseTexture);
        Shader.SetGlobalTexture("_BHLUT", BlackHoleLut);
        Shader.SetGlobalTexture("_BHCapturedLUT", BlackHoleLutCaptured);
        Shader.SetGlobalTexture("_KerrRadialLUT", KerrRadialLut);
        Shader.SetGlobalTexture("_KerrPolarLUT", KerrPolarLut);
        Shader.SetGlobalTexture("_KerrCritHeader", KerrCritHeader);
    }
}