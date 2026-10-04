using UnityEngine;
using System;

[System.Serializable]
public struct BlackHoleParamaters
{
    public double mass;
    public double schwarzschildRadiusWorld;
    public float schwarzschildRadiusUnits;
    public float diskInnerRadiusNormalized;
    public float diskOuterRadiusNormalized;
    public float escapeRadiusNormalized;
    [Range(-0.998f, 0.998f)] public float spin;
    public float diskRotation;
    public bool accretionDisk;
    public Vector3 blackHolePositionUnits;
    /*
    public void CalculateBlackHoleISCO()
    {
        double Z1 = 1 + Math.Pow((1 - spin * spin), 1 / 3) * (Math.Pow((1 + spin), 1 / 3) + Math.Pow((1 - spin), 1 / 3));
        double Z2 = Math.Sqrt(3 * spin * spin + Z1 * Z1);

        diskInnerRadiusNormalized = (Math.Sign(diskRotation) == Math.Sign(spin)) ? (float)(3 + Z2 - Math.Sqrt((3 - Z1) * (3 + Z1 + 2 * Z2)) * mass) : (float)(3 + Z2 + Math.Sqrt((3 - Z1) * (3 + Z1 + 2 * Z2)) * mass);
    }*/
}
