#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.IO;
using System.Threading.Tasks;
using System.Collections.Generic;

public class BlackHoleUnifiedLUTGenerator
{
    [MenuItem("Tools/Generate Black Hole Unified LUT")]
    public static void GenerateLUT()
    {
        int width = 4096;
        int height = 4096;

        Texture2D lut = new Texture2D(width, height, TextureFormat.RGBAFloat, false);
        Color[] pixelData = new Color[width * height];

        float bCrit = 2.598076211f;
        float bSplitCap = 1.5f;   // must match B_SPLIT_CAP in the shader
        float bSplit = 20.0f;     // must match B_SPLIT in the shader
        float bMax = 200.0f;      // must match B_MAX in the shader
        float logKCap = 8.0f;     // must match LOG_K_CAP in the shader
        float k = 8.0f;           // must match LOG_K in the shader
        float weakLogK = 50f;     // must match WEAK_LOG_K in the shader
        float rStart = 100000.0f;

        // b-axis: four segments meeting continuously, dense on BOTH sides of
        // b_crit (u=0.5 is exactly b_crit) instead of two separately-anchored
        // packings that have to be reconciled at a boundary.
        Parallel.For(0, width, x =>
        {
            float u = (float)x / (width - 1);
            float b;
            if (u <= 0.15f)
            {
                float t = u / 0.15f;
                b = t * bSplitCap;
            }
            else if (u <= 0.5f)
            {
                float t = (u - 0.15f) / 0.35f;
                float frac = Mathf.Pow(10f, -logKCap * t);
                b = bCrit - frac * (bCrit - bSplitCap);
            }
            else if (u <= 0.85f)
            {
                float t = (u - 0.5f) / 0.35f;
                float frac = Mathf.Pow(10f, k * (t - 1f));
                b = bCrit + frac * (bSplit - bCrit);
            }
            else
            {
                float t = (u - 0.85f) / 0.15f;
                float tt = (Mathf.Pow(weakLogK, t) - 1f) / (weakLogK - 1f);
                b = bSplit + tt * (bMax - bSplit);
            }
            b = Mathf.Max(b, 1e-4f);

            // Winding count near the unstable photon orbit diverges like
            // -ln|b - bCrit| as b -> bCrit -- that's real physics, not a numeric
            // quirk. The old fixed 30-radian budget silently truncated those
            // columns instead of ever hitting a real stop condition. Scale the
            // step budget with the log distance to bCrit instead of a flat cutoff
            // (a flat b-distance threshold covers most of the log-packed columns
            // near critical and would balloon bake time far more than needed).
            float distFromCritical = Mathf.Max(Mathf.Abs(b - bCrit), 1e-7f);
            float logCloseness = Mathf.Max(0f, -Mathf.Log(distFromCritical));
            int extraSteps = (int)(logCloseness * 150000f); // tune this constant, see below
            int dynamicMaxSteps = Mathf.Min(3000000 + extraSteps, 3000000 * 12); // hard cost ceiling

            float u0 = 1f / rStart;
            float uu = u0;
            float vv = Mathf.Sqrt(Mathf.Max(0f, 1f / (b * b) - uu * uu + uu * uu * uu));
            float phi = 0f;
            float stepSize = 0.00001f;

            List<float> phiList = new List<float>();
            List<float> uList = new List<float>();

            bool reachedHorizon = false;

            int steps = 0;
            while (steps < dynamicMaxSteps)
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

                if (vv <= 0f) break;                            // turning point -- escapes
                if (uu >= 1f) { reachedHorizon = true; break; }  // horizon -- captured
                steps++;
            }

            bool truncated = steps >= dynamicMaxSteps - 1;
            float totalPhi = phiList.Count > 0 ? phiList[phiList.Count - 1] : 0f;

            if (x == width / 2 || truncated)
            {
                Debug.Log($"[Unified] x={x}  b={b:F4}  totalPhi={totalPhi:F6}  reachedHorizon={reachedHorizon}  steps={phiList.Count}  truncated={truncated}  final r={1f / uList[uList.Count - 1]:F4}");
            }

            for (int y = 0; y < height; y++)
            {
                float t = (float)y / (height - 1);
                float oneMinusT = 1f - t;
                float targetPhi = totalPhi * (1f - oneMinusT * oneMinusT);

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
                pixelData[index] = new Color(rVal, totalPhi, reachedHorizon ? 1f : 0f, 0f);
            }
        });

        lut.SetPixels(pixelData);
        lut.Apply();
        byte[] bytes = lut.EncodeToEXR(Texture2D.EXRFlags.CompressZIP | Texture2D.EXRFlags.OutputAsFloat);
        string path = Application.dataPath + "/BlackHoleUnifiedLUT.exr";
        File.WriteAllBytes(path, bytes);
        AssetDatabase.Refresh();
        Debug.Log("Black Hole Unified LUT successfully generated at: " + path);
    }

    private static float Acceleration(float currentU)
    {
        return 1.5f * (currentU * currentU) - currentU;
    }
}
#endif