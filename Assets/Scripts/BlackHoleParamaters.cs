using UnityEngine;

[System.Serializable]
public struct BlackHoleParamaters
{
    public double mass;
    public double schwarzschildRadiusWorld;
    public float schwarzschildRadiusUnits;
    public float diskInnerRadiusNormalized;
    public float diskOuterRadiusNormalized;
    public float escapeRadiusNormalized;
    public Vector3 blackHolePositionUnits;
}
