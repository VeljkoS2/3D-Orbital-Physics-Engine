using UnityEngine;

[System.Serializable]
public struct BlackHoleAppearanceProperties
{
    public float diskPeakTemperature;
    public float diskDopplerColor;
    public float diskDopplerBeaming;
    public float diskRadialContrast;
    public float diskBrightness;
    public float diskISCOStress;
    public float diskTurbulance;
    public float diskHigherOrderFade;
    [Header("Hot spots")]
    [Range(0, 6)] public int hotSpotCount;
    public float hotSpotStrength;
    public float hotSpotRMin; 
    public float hotSpotRMax;
    [Header("Accretion Disk Color Based On Mass")]
    public bool physicalDiskTemperature;
    [Range(1e-9f, 1.0f)] public float eddingtonRatio;
    [Header("Disk Color Tint")]
    public Color diskTint;
    [Range(0f, 1f)] public float diskTintStrength;
}
