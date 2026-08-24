#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.IO;
using System.Threading.Tasks;

public class BlackHoleLUTGenerator
{
    [MenuItem("Tools/Generate Black Hole LUT")]
    public static void GenerateLUT()
    {
        int width = 2048;  // b axis (log-packed near bCritical)
        int height = 2048; // phi-fraction axis, 0 = turning point, 1 = escaped to r_start

        Texture2D lut = new Texture2D(width, height, TextureFormat.RGBAFloat, false);
        Color[] pixelData = new Color[width * height];

        float bCrit = 2.598076211f;
        float bMax = 200.0f;   // must match B_MAX in the shader
        float bSplit = 20.0f;   // must match B_SPLIT in the shader
        float uSplit = 0.6f;   // must match U_SPLIT in the shader
        float k = 8.0f;
        float rStart = 100000.0f;

        Parallel.For(0, width, x =>
        {
            float u = (float)x / (width - 1);
            float b;
            if (u <= uSplit)
            {
                float u1 = u / uSplit;
                float frac = Mathf.Pow(10f, k * (u1 - 1f));
                b = bCrit + frac * (bSplit - bCrit);
            }
            else
            {
                float s = (u - uSplit) / (1f - uSplit);
                float weakLogK = 50f; // must match WEAK_LOG_K in the shader
                float t = (Mathf.Pow(weakLogK, s) - 1f) / (weakLogK - 1f);
                b = bSplit + t * (bMax - bSplit);
            }

            // Find the turning point (closest approach) for this b via bisection —
            // robust, no derivative issues, no risk of overshooting near bCritical.
            float uMin = FindTurningPoint(b);

            // Integrate OUTWARD from the turning point (v=0 there) out to r = rStart.
            // This makes phi=0 canonically the turning point for every b, so the table
            // is indexed by a physically meaningful, camera-distance-independent quantity.
            float uu = uMin;
            float vv = 0f;
            float phi = 0f;
            float stepSize = 0.00001f;
            int maxSteps = 2000000; // generous — this runs once, offline, cost doesn't matter

            float[] phiAtStep = new float[height];
            float[] uAtStep = new float[height];

            // We don't know total phi in advance, so integrate first to find it,
            // then resample onto a fixed grid.
            System.Collections.Generic.List<float> phiList = new System.Collections.Generic.List<float>();
            System.Collections.Generic.List<float> uList = new System.Collections.Generic.List<float>();

            int steps = 0;
            while (steps < maxSteps)
            {
                float ku1 = stepSize * vv;
                float kv1 = stepSize * Acceleration(uu);
                float ku2 = stepSize * (vv + 0.5f * kv1);
                float kv2 = stepSize * Acceleration(uu + 0.5f * ku1);
                float ku3 = stepSize * (vv + 0.5f * kv2);
                float kv3 = stepSize * Acceleration(uu + 0.5f * ku2);
                float ku4 = stepSize * (vv + kv3);
                float kv4 = stepSize * Acceleration(uu + ku3);

                uu += (ku1 + 2f * ku2 + 2f * ku3 + ku4) / 6f;
                vv += (kv1 + 2f * kv2 + 2f * kv3 + kv4) / 6f;
                phi += stepSize;

                phiList.Add(phi);
                uList.Add(uu);

                if (uu <= 1f / rStart) break; // reached r_start going outward
                steps++;
            }

            float totalPhi = phiList.Count > 0 ? phiList[phiList.Count - 1] : 0f;

            float smallPhiTarget = totalPhi * 0.001f;
            float rAtSmallPhi = rStart;
            float phiAtSmallPhi = smallPhiTarget;
            for (int s = 0; s < phiList.Count; s++)
            {
                if (phiList[s] >= smallPhiTarget)
                {
                    rAtSmallPhi = 1f / Mathf.Max(uList[s], 1f / rStart);
                    phiAtSmallPhi = phiList[s];
                    break;
                }
            }
            float rTurn = 1f / Mathf.Max(uMin, 1f / rStart);
            float K = (phiAtSmallPhi > 1e-9f) ? (rAtSmallPhi - rTurn) / (phiAtSmallPhi * phiAtSmallPhi) : 0f;

            if (x == width / 2) // just log the middle column once, avoid spam
            {
                Debug.Log($"x={x}  b={b:F4}  uMin={uMin:F6}  totalPhi={totalPhi:F6}  phiList.Count={phiList.Count}  first r={1f / uList[0]:F3}  last r={1f / uList[uList.Count - 1]:F3}");
            }

            for (int y = 0; y < height; y++)
            {
                float t = (float)y / (height - 1);
                float targetPhi = totalPhi * t * t;
                // find nearest recorded sample (linear search fine — offline, runs once)
                float rVal = rStart;
                for (int s = 0; s < phiList.Count; s++)
                {
                    if (phiList[s] >= targetPhi)
                    {
                        rVal = 1f / Mathf.Max(uList[s], 1f / rStart);
                        break;
                    }
                }
                int index = y * width + x;
                pixelData[index] = new Color(rVal, totalPhi, uMin, K);
            }
        });

        lut.SetPixels(pixelData);
        lut.Apply();
        byte[] bytes = lut.EncodeToEXR(Texture2D.EXRFlags.CompressZIP);
        string path = Application.dataPath + "/BlackHoleLensingLUT.exr";
        File.WriteAllBytes(path, bytes);
        AssetDatabase.Refresh();
        Debug.Log("Black Hole LUT successfully generated at: " + path);
    }

    private static float FindTurningPoint(float b)
    {
        // bisection on u in (0, 2/3): f(u) = u^3 - u^2 + 1/b^2, find smallest positive root
        float lo = 0.0001f, hi = 0.6666f;
        for (int i = 0; i < 60; i++)
        {
            float mid = 0.5f * (lo + hi);
            float f = mid * mid * mid - mid * mid + 1f / (b * b);
            if (f > 0f) lo = mid; else hi = mid;
        }
        return 0.5f * (lo + hi);
    }

    private static float Acceleration(float currentU)
    {
        return 1.5f * (currentU * currentU) - currentU;
    }
}
#endif