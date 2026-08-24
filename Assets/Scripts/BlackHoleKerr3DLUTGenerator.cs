#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Threading.Tasks;

public static class BlackHoleKerr3DLUTGenerator
{
    const float M = 0.5f;
    const float Spin = 0.9f;
    const int WidthXi = 192;
    const int HeightEta = 96;
    const int DepthFraction = 128;
    const float RStart = 500f;
    const float EtaMax = 60f;
    const float XiLogK = 8.0f;
    const float XiFarMax = 40f;

    [MenuItem("Tools/Black Hole/Generate Kerr 3D LUT")]
    public static void Generate()
    {
        float a = Spin * M;
        float rHorizon = M + Mathf.Sqrt(Mathf.Max(M * M - a * a, 0f));

        var radial = new Color[WidthXi * HeightEta * DepthFraction];
        var polar = new Color[WidthXi * HeightEta * DepthFraction];
        var critHeader = new Color[HeightEta];

        Parallel.For(0, HeightEta, ye =>
        {
            float vEta = (ye + 0.5f) / HeightEta;
            float eta = EtaMax * vEta * vEta * vEta;

            float xiCritPro = GetAnalyticXiCrit(+1f, eta, a);
            float xiCritRetro = GetAnalyticXiCrit(-1f, eta, a);
            critHeader[ye] = new Color(xiCritPro, xiCritRetro, 0f, 0f);

            for (int xx = 0; xx < WidthXi; xx++)
            {
                float u = (xx + 0.5f) / WidthXi;
                float xi = XAxisToXiContinuous(u, xiCritPro, xiCritRetro);

                // ---- Radial Marching (Strictly Inbound Leg: RStart -> Turning Point / Horizon) ----
                var rList = new List<float>(2048);
                float r = RStart;
                float tau = 0f;
                bool captured = false;

                for (int step = 0; step < 20000; step++)
                {
                    float R = GetR(r, xi, eta, a);

                    if (R <= 0f)
                    {
                        captured = false;
                        break; // Reached turning point r_min
                    }

                    float dr_dtau = -Mathf.Sqrt(R) / (r * r + a * a); // Strictly inbound
                    float h = Mathf.Clamp(0.005f * r / Mathf.Max(Mathf.Abs(dr_dtau), 1e-3f), 1e-4f, r * 0.02f);

                    r = StepRK4R(r, -1f, xi, eta, a, h);
                    tau += h;
                    rList.Add(r);

                    if (r <= rHorizon * 1.0005f)
                    {
                        captured = true;
                        break; // Reached horizon
                    }
                }

                float totalTauInbound = Mathf.Max(tau, 1e-4f);

                // ---- Polar Marching ----
                var muList = new List<float>(1024);
                float muVal = 0f, tauPolar = 0f;

                for (int step = 0; step < 20000; step++)
                {
                    float Theta = GetTheta(muVal, xi, eta, a);
                    float dMu = Mathf.Sqrt(Mathf.Max(Theta, 0f));
                    float h = Mathf.Clamp(0.01f / Mathf.Max(dMu, 1e-3f), 1e-4f, 0.05f);

                    muVal = StepRK4Mu(muVal, xi, eta, a, h);
                    tauPolar += h;
                    muList.Add(Mathf.Clamp(muVal, 0f, 1f));

                    if (Theta <= 1e-6f || muVal >= 0.9999f) break;
                }
                float quarterPeriod = Mathf.Max(tauPolar, 1e-4f);

                // ---- Write to 3D Volume ----
                for (int f = 0; f < DepthFraction; f++)
                {
                    float t = (float)f / Mathf.Max(DepthFraction - 1, 1);
                    float oneMinusT = 1f - t;
                    float targetTauR = totalTauInbound * (1f - oneMinusT * oneMinusT);
                    float rVal = (rList.Count > 0) ? rList[rList.Count - 1] : rHorizon;

                    for (int s = 0; s < rList.Count; s++)
                    {
                        float tauAtS = (float)s / Mathf.Max(rList.Count - 1, 1) * totalTauInbound;
                        if (tauAtS >= targetTauR) { rVal = rList[s]; break; }
                    }

                    float targetTauMu = quarterPeriod * t;
                    float muOut = 1f;

                    for (int s = 0; s < muList.Count; s++)
                    {
                        float tauAtS = (float)s / Mathf.Max(muList.Count - 1, 1) * quarterPeriod;
                        if (tauAtS >= targetTauMu) { muOut = muList[s]; break; }
                    }

                    int idx = (f * HeightEta + ye) * WidthXi + xx;
                    radial[idx] = new Color(rVal, totalTauInbound, captured ? 1f : 0f, 0f);
                    polar[idx] = new Color(muOut, quarterPeriod, 0f, 0f);
                }
            }
        });

        SaveTexture3D(radial, "BlackHoleKerrRadialLUT", WidthXi, HeightEta, DepthFraction);
        SaveTexture3D(polar, "BlackHoleKerrPolarLUT", WidthXi, HeightEta, DepthFraction);
        SaveCritHeader(critHeader, "BlackHoleKerrCritHeader", HeightEta);

        Shader.SetGlobalFloat("_KerrSpin", Spin);
        Shader.SetGlobalFloat("_KerrMass", M);
        Shader.SetGlobalFloat("_KerrHorizon", rHorizon);
        Shader.SetGlobalFloat("_KerrEtaMax", EtaMax);
        Shader.SetGlobalFloat("_KerrXiFarMax", XiFarMax);
        Shader.SetGlobalFloat("_KerrXiLogK", XiLogK);
        Debug.Log("Kerr 3D LUT updated with monotonic inbound radial profiles.");
    }

    static float XAxisToXiContinuous(float u, float xiCritPro, float xiCritRetro)
    {
        float centered = (u - 0.5f) * 2f;

        if (centered >= 0f)
        {
            if (centered < 0.5f)
            {
                float w = centered / 0.5f;
                return xiCritPro * (w * w * w);
            }
            else
            {
                float w = (centered - 0.5f) / 0.5f;
                float t = (Mathf.Exp(w * XiLogK) - 1f) / (Mathf.Exp(XiLogK) - 1f);
                return xiCritPro + XiFarMax * t;
            }
        }
        else
        {
            float absC = -centered;
            if (absC < 0.5f)
            {
                float w = absC / 0.5f;
                return -xiCritRetro * (w * w * w);
            }
            else
            {
                float w = (absC - 0.5f) / 0.5f;
                float t = (Mathf.Exp(w * XiLogK) - 1f) / (Mathf.Exp(XiLogK) - 1f);
                return -(xiCritRetro + XiFarMax * t);
            }
        }
    }

    static float GetAnalyticXiCrit(float sign, float eta, float a)
    {
        float rPh = 3f * M + sign * 2f * M * Mathf.Cos(2f / 3f * Mathf.Acos(-sign * a / M));
        float xi = (M * (rPh * rPh - a * a) - rPh * (rPh * rPh - 2f * M * rPh + a * a)) / (a * (rPh - M));
        return Mathf.Abs(xi);
    }

    static float GetR(float r, float xi, float eta, float a)
    {
        float Delta = r * r - 2f * M * r + a * a;
        float Xi = r * r + a * a - a * xi;
        return Xi * Xi - Delta * (eta + (xi - a) * (xi - a));
    }

    static float GetTheta(float mu, float xi, float eta, float a)
    {
        float sinThSq = Mathf.Max(1f - mu * mu, 1e-8f);
        return eta + mu * mu * (a * a - xi * xi / sinThSq);
    }

    static float StepRK4R(float r, float sigmaR, float xi, float eta, float a, float h)
    {
        float k1 = sigmaR * Mathf.Sqrt(Mathf.Max(GetR(r, xi, eta, a), 0f)) / (r * r + a * a);
        float r2 = r + 0.5f * h * k1;
        float k2 = sigmaR * Mathf.Sqrt(Mathf.Max(GetR(r2, xi, eta, a), 0f)) / (r2 * r2 + a * a);
        float r3 = r + 0.5f * h * k2;
        float k3 = sigmaR * Mathf.Sqrt(Mathf.Max(GetR(r3, xi, eta, a), 0f)) / (r3 * r3 + a * a);
        float r4 = r + h * k3;
        float k4 = sigmaR * Mathf.Sqrt(Mathf.Max(GetR(r4, xi, eta, a), 0f)) / (r4 * r4 + a * a);

        return r + (h / 6f) * (k1 + 2f * k2 + 2f * k3 + k4);
    }

    static float StepRK4Mu(float mu, float xi, float eta, float a, float h)
    {
        float k1 = Mathf.Sqrt(Mathf.Max(GetTheta(mu, xi, eta, a), 0f));
        float mu2 = Mathf.Clamp01(mu + 0.5f * h * k1);
        float k2 = Mathf.Sqrt(Mathf.Max(GetTheta(mu2, xi, eta, a), 0f));
        float mu3 = Mathf.Clamp01(mu + 0.5f * h * k2);
        float k3 = Mathf.Sqrt(Mathf.Max(GetTheta(mu3, xi, eta, a), 0f));
        float mu4 = Mathf.Clamp01(mu + h * k3);
        float k4 = Mathf.Sqrt(Mathf.Max(GetTheta(mu4, xi, eta, a), 0f));

        return Mathf.Clamp01(mu + (h / 6f) * (k1 + 2f * k2 + 2f * k3 + k4));
    }

    static void SaveTexture3D(Color[] data, string name, int w, int h, int d)
    {
        var tex = new Texture3D(w, h, d, TextureFormat.RGBAFloat, false);
        tex.wrapMode = TextureWrapMode.Clamp;
        tex.filterMode = FilterMode.Bilinear;
        tex.SetPixels(data);
        tex.Apply();
        string path = $"Assets/{name}.asset";
        AssetDatabase.CreateAsset(tex, path);
        AssetDatabase.SaveAssets();
    }

    static void SaveCritHeader(Color[] data, string name, int h)
    {
        var tex = new Texture2D(1, h, TextureFormat.RGFloat, false, true);
        tex.wrapMode = TextureWrapMode.Clamp;
        tex.filterMode = FilterMode.Bilinear;
        tex.SetPixels(data);
        tex.Apply();
        string path = $"Assets/{name}.asset";
        AssetDatabase.CreateAsset(tex, path);
        AssetDatabase.SaveAssets();
    }
}
#endif