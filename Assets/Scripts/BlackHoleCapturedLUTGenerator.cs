#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.IO;
using System.Threading.Tasks;
using System.Collections.Generic;

public class BlackHoleCapturedLUTGenerator
{
    // Must match BH_CRITICAL in the shader
    const float bCrit = 2.598076211f;

    // b-axis packing for the captured side (b in [0, bCrit))
    const float bSplitCap = 1.5f;   // must match B_SPLIT_CAP in the shader
    const float uSplitCap = 0.5f;   // must match U_SPLIT_CAP in the shader
    const float logKCap = 8.0f;   // must match LOG_K_CAP in the shader

    [MenuItem("Tools/Generate Black Hole Captured LUT")]
    public static void GenerateLUT()
    {
        int width = 2048;
        int height = 2048;

        Texture2D lut = new Texture2D(width, height, TextureFormat.RGBAFloat, false);
        Color[] pixelData = new Color[width * height];

        float rStart = 100000.0f;

        Parallel.For(0, width, x =>
        {
            float u = (float)x / (width - 1);
            float b;
            if (u <= uSplitCap)
            {
                float t = u / uSplitCap;
                b = t * bSplitCap;                       // linear, 0 .. bSplitCap
            }
            else
            {
                float t = (u - uSplitCap) / (1f - uSplitCap);
                float frac = Mathf.Pow(10f, -logKCap * t); // 1 -> ~1e-8 as t: 0 -> 1
                b = bCrit - frac * (bCrit - bSplitCap);     // approaches bCrit, never reaches it
            }
            b = Mathf.Max(b, 1e-4f); // avoid b=0 exactly (divide-by-zero in v0)

            // Start exactly at the horizon (u=1, r=1 in r_s units), phi=0.
            // Known analytic starting slope: v0 = dv/dphi = -1/b (moving outward).
            float uu = 1.0f;
            float vv = -1.0f / b;
            float phi = 0f;
            float stepSize = 0.00001f;
            int maxSteps = 2000000;

            List<float> phiList = new List<float>();
            List<float> uList = new List<float>();

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

                if (uu <= 1f / rStart) break; // reached r_start
                if (uu >= 1f) break;           // safety: fell back into horizon numerically
                steps++;
            }

            float totalPhi = phiList.Count > 0 ? phiList[phiList.Count - 1] : 0f;

            if (x == width / 2)
            {
                Debug.Log($"[Captured] x={x}  b={b:F4}  totalPhi={totalPhi:F6}  steps={phiList.Count}");
            }

            for (int y = 0; y < height; y++)
            {
                float targetPhi = totalPhi * ((float)y / (height - 1));
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
                // r, totalPhi, v0 (debug/unused by shader), unused
                pixelData[index] = new Color(rVal, totalPhi, vv, 0f);
            }
        });

        lut.SetPixels(pixelData);
        lut.Apply();
        byte[] bytes = lut.EncodeToEXR(Texture2D.EXRFlags.CompressZIP);
        string path = Application.dataPath + "/BlackHoleCapturedLUT.exr";
        File.WriteAllBytes(path, bytes);
        AssetDatabase.Refresh();
        Debug.Log("Black Hole Captured LUT successfully generated at: " + path);
    }

    private static float Acceleration(float currentU)
    {
        return 1.5f * (currentU * currentU) - currentU;
    }
}
#endif
