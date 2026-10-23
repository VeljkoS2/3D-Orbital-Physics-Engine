using UnityEngine;

[System.Serializable]
public struct BlackHoleAppearanceProperties
{
    public float diskPeakTemperature;
    public float diskDopplerColor;
    public float diskDopplerBeaming;
    public float diskRadialContrast;
    [Tooltip("How the disk's brightness is weighed: 0 = by the total power the gas emits (bolometric, as before), " +
             "1 = by the visible light in it, as the eye or a camera sees it. Hot gas (tens of thousands of K and up) " +
             "emits mostly ultraviolet, so in visible light the inner disk outshines the cooler parts far less, and a " +
             "very massive hole's cool outer disk shows its warm colours. Lower Disk Brightness when raising this " +
             "for a hot disk.")]
    [Range(0f, 1f)] public float diskVisibleLight;
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
