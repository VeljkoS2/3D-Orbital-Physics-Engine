// KerrEllipticLUTGenerator.cs
// Bakes the three LUTs used by BlackHoleKerrAnalytic.hlsl.
//
// Layout (all spin-independent, all float32, all "endpoint-exact": texel i sits
// exactly at coordinate i/(N-1), the shader samples with (c*(N-1)+0.5)/N):
//
//   Modulus axis x in [0,1]:  mc = 1 - m = 2^(x * log2(MC_MIN)),  MC_MIN = 1e-8
//     x = 0 -> m = 0,  x = 1 -> m = 1 - 1e-8.  Log spacing in mc resolves the
//     m -> 1 region that near-critical rays live in (plunge-side 1-m ~ (b_c - b)).
//
//   KerrSnCnLUT (RGFloat, SNCN_X x SNCN_S): (sn, cn) at u = 4K(m) * s,
//     s in [0, 0.25] (quarter period; the shader folds everything else).
//   KerrKLUT    (RFloat,  SNCN_X x 1):      K(m).
//   KerrFLUT    (RFloat,  F_X x F_T):       F(psi|m) / (4K), psi in [0, psi*],
//     tan(psi*) = mc^(-1/4); row coordinate t = asinh(tan psi) / asinh(mc^(-1/4)).
//     The shader maps psi > psi* through F(psi) + F(chi) = K, sqrt(mc) tan psi tan chi = 1,
//     so the log singularity of F at psi -> pi/2, m -> 1 is never sampled.
//
// Textures are saved as .asset (exact RGFloat/RFloat, no EXR import conversion).

#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System;

public class KerrEllipticLUTGenerator : EditorWindow
{
    // Experiment: store the SnCn LUT as 16-bit unorm (RG32) instead of RGFloat.
    // Half the memory/bandwidth and full-rate bilinear filtering on most GPUs;
    // quantisation step 1.5e-5. A/B test before keeping it.
    public bool snCnUnorm16 = true;
    // Keep these in sync with the KERR_LUT_* defines in BlackHoleKerrAnalytic.hlsl
    public int snCnResX = 1024;
    public int snCnResS = 1024;
    public int fResX = 512;
    public int fResT = 256;

    const double MC_MIN = 1e-8;

    [MenuItem("Tools/Kerr Elliptic LUT Generator")]
    public static void ShowWindow()
    {
        GetWindow<KerrEllipticLUTGenerator>("Kerr LUT Generator");
    }

    private void OnGUI()
    {
        GUILayout.Label("SnCn + K LUT", EditorStyles.boldLabel);
        snCnResX = EditorGUILayout.IntField("Modulus Res (KERR_LUT_X)", snCnResX);
        snCnResS = EditorGUILayout.IntField("Phase Res (KERR_LUT_S)", snCnResS);
        snCnUnorm16 = EditorGUILayout.Toggle("SnCn as 16-bit unorm (experiment)", snCnUnorm16);
        EditorGUILayout.Space(10);
        GUILayout.Label("Forward Phase F/(4K) LUT", EditorStyles.boldLabel);
        fResX = EditorGUILayout.IntField("Modulus Res (KERR_LUT_FX)", fResX);
        fResT = EditorGUILayout.IntField("Angle Res (KERR_LUT_FT)", fResT);

        EditorGUILayout.Space(20);
        if (GUILayout.Button("Bake Kerr LUTs", GUILayout.Height(35)))
            Bake();
    }

    // ------------------------------------------------------------------
    // Math (double precision)
    // ------------------------------------------------------------------
    static double McOfX(double x) => Math.Pow(2.0, x * Math.Log(MC_MIN, 2.0));

    static double CarlsonRF(double x, double y, double z)
    {
        for (int i = 0; i < 100; i++)
        {
            double A = (x + y + z) / 3.0;
            double dx = 1.0 - x / A, dy = 1.0 - y / A, dz = 1.0 - z / A;
            if (Math.Max(Math.Abs(dx), Math.Max(Math.Abs(dy), Math.Abs(dz))) < 1e-4)
            {
                double E2 = dx * dy - dz * dz;
                double E3 = dx * dy * dz;
                return (1.0 - E2 / 10.0 + E3 / 14.0 + E2 * E2 / 24.0 - 3.0 * E2 * E3 / 44.0) / Math.Sqrt(A);
            }
            double sx = Math.Sqrt(x), sy = Math.Sqrt(y), sz = Math.Sqrt(z);
            double lam = sx * sy + sy * sz + sz * sx;
            x = 0.25 * (x + lam);
            y = 0.25 * (y + lam);
            z = 0.25 * (z + lam);
        }
        return 1.0 / Math.Sqrt((x + y + z) / 3.0);
    }

    static double CompleteKc(double mc) => CarlsonRF(0.0, mc, 1.0);

    // sn, cn at argument u for parameter m = 1 - mc (descending AGM / Landen)
    static void SnCn(double u, double mc, out double sn, out double cn)
    {
        const int N = 40;
        double[] a = new double[N];
        double[] c = new double[N];
        a[0] = 1.0;
        c[0] = Math.Sqrt(1.0 - mc);
        double b = Math.Sqrt(mc);
        int n = 0;
        while (n < N - 1 && Math.Abs(c[n]) > 1e-15)
        {
            a[n + 1] = 0.5 * (a[n] + b);
            c[n + 1] = 0.5 * (a[n] - b);
            b = Math.Sqrt(a[n] * b);
            n++;
        }
        double phi = Math.Pow(2.0, n) * a[n] * u;
        for (int i = n; i >= 1; i--)
            phi = 0.5 * (phi + Math.Asin(Math.Max(-1.0, Math.Min(1.0, c[i] / a[i] * Math.Sin(phi)))));
        sn = Math.Sin(phi);
        cn = Math.Cos(phi);
    }

    static double Asinh(double v) => Math.Log(v + Math.Sqrt(1.0 + v * v));

    // ------------------------------------------------------------------
    // Bake
    // ------------------------------------------------------------------
    public void Bake()
    {
        try
        {
            EditorUtility.DisplayProgressBar("Kerr LUTs", "SnCn...", 0.0f);
            BakeSnCnAndK();
            EditorUtility.DisplayProgressBar("Kerr LUTs", "F/(4K)...", 0.8f);
            BakeF();
            Debug.Log("<color=green>Kerr LUTs baked.</color> Resolutions: SnCn " + snCnResX + "x" + snCnResS +
                      ", K " + snCnResX + "x1, F " + fResX + "x" + fResT + " -- match the KERR_LUT_* defines.");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Kerr LUT bake failed: {ex.Message}\n{ex.StackTrace}");
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }
    }

    void BakeSnCnAndK()
    {
        var snc = new Texture2D(snCnResX, snCnResS,
                                        snCnUnorm16 ? TextureFormat.RG32 : TextureFormat.RGFloat, false, true);
        var kTex = new Texture2D(snCnResX, 1, TextureFormat.RFloat, false, true);
        var sncPix = new Color[snCnResX * snCnResS];
        var kPix = new Color[snCnResX];

        for (int ix = 0; ix < snCnResX; ix++)
        {
            double mc = McOfX(ix / (double)(snCnResX - 1));
            double K = CompleteKc(mc);
            kPix[ix] = new Color((float)K, 0f, 0f, 1f);

            for (int iS = 0; iS < snCnResS; iS++)
            {
                double s = 0.25 * iS / (snCnResS - 1);
                SnCn(4.0 * K * s, mc, out double sn, out double cn);
                sncPix[iS * snCnResX + ix] = new Color((float)sn, (float)cn, 0f, 1f);
            }

            if ((ix & 63) == 0)
                EditorUtility.DisplayProgressBar("Kerr LUTs", "SnCn...", 0.8f * ix / snCnResX);
        }

        snc.SetPixels(sncPix);
        snc.Apply(false, false);
        kTex.SetPixels(kPix);
        kTex.Apply(false, false);
        SaveAsset(snc, "KerrSnCnLUT");
        SaveAsset(kTex, "KerrKLUT");
    }

    void BakeF()
    {
        var tex = new Texture2D(fResX, fResT, TextureFormat.RFloat, false, true);
        var pix = new Color[fResX * fResT];

        for (int ix = 0; ix < fResX; ix++)
        {
            double mc = McOfX(ix / (double)(fResX - 1));
            double K = CompleteKc(mc);
            double yStar = Asinh(Math.Pow(mc, -0.25));

            for (int it = 0; it < fResT; it++)
            {
                double t = it / (double)(fResT - 1);
                double psi = Math.Atan(Math.Sinh(t * yStar));
                double s = Math.Sin(psi), c = Math.Cos(psi);
                double F = s * CarlsonRF(c * c, c * c + mc * s * s, 1.0);
                pix[it * fResX + ix] = new Color((float)(F / (4.0 * K)), 0f, 0f, 1f);
            }
        }

        tex.SetPixels(pix);
        tex.Apply(false, false);
        SaveAsset(tex, "KerrFLUT");
    }

    static void SaveAsset(Texture2D tex, string name)
    {
        tex.wrapMode = TextureWrapMode.Clamp;
        tex.filterMode = FilterMode.Bilinear;
        tex.anisoLevel = 0;
        tex.name = name;

        string path = "Assets/" + name + ".asset";
        if (AssetDatabase.LoadAssetAtPath<Texture2D>(path) != null)
            AssetDatabase.DeleteAsset(path);
        AssetDatabase.CreateAsset(tex, path);
        AssetDatabase.SaveAssets();
        Debug.Log($"Baked {name} -> {path}");
    }
}
#endif