using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class BlackHoleGlobalManager : MonoBehaviour
{
    public const int MaxBlackHoles = 8;

    // ---- must match BlackHoleKerrAnalytic.hlsl ----------------------------------
    const double DiskNoiseLifeOrbits = 2.0;   // DISK_NOISE_LIFE_ORBITS
    const double DiskDrift = 0.02;            // KERR_DISK_DRIFT
    const double KerrM = 0.5;                 // KERR_M
    public const int PlungeLutWidth = 64;     // BH_PLUNGE_LUT_W (height = MaxBlackHoles)
    const float ObserverShiftStrength = 1.0f;
    // The exact sphere never disappears: it shrinks continuously toward this floor
    // (isotropic Rs), inside the photon sphere, where everything is captured anyway.
    const float KerrFloorRs = 0.55f;

    public List<GameObject> blackHoles = GlobalProperties.blackHoles;

    [Header("Multi-hole lensing")]
    [Tooltip("Exact Kerr sphere radius (Rs) at spin 0 and at full spin.")]
    public float innerKerrRadiusSlowRs = 3f;
    public float innerKerrRadiusFastRs = 12f;
    [Tooltip("Minimum radius (Rs) of the combined-field region around a group (40: ~0.15 px at its edge, 30: ~0.5 px).")]
    public float groupRadiusRs = 40f;
    [Tooltip("Debug: send single holes through the combined-field integrator too.")]
    public bool debugForceMedium = false;

    [Header("Circumbinary disk")]
    [Tooltip("Inner edge = this x separation (a binary clears a cavity about twice its separation).")]
    public float cbdCavity = 2.0f;
    [Tooltip("How far the gas stripped from the holes' own disks spreads: the shared disk's area is this^2 x the area they lost.")]
    public float cbdOuterScale = 1.0f;
    public float cbdBrightness = 1.0f;
    [Tooltip("Speed multiplier for the shared disk's structure (1 = physical).")]
    public float cbdSpeed = 1f;

    [Header("Merger")]
    [Tooltip("Hand-over (remnant spin builds up), in units of the remnant's mass G M / c^3 " +
             "(simulation time). A real ringdown lasts a few tens of M.")]
    public float mergerRingdownM = 100f;
    [Tooltip("Stretches the plunge and the hand-over in simulation time. 1 = physical.")]
    public float mergerTimeStretch = 1f;
    [Tooltip("How the shared disk's look moves from the early (dim, cool) look to the merged hole's " +
         "disk as the cavity closes. 1 = even; >1 keeps the early look longer, <1 brightens sooner.")]
    public float cbdLookCurve = 1f;

    static readonly int BHCBD6Id = Shader.PropertyToID("_BHCBD6");
    static readonly int BHCBD7Id = Shader.PropertyToID("_BHCBD7");
    static readonly int BHCBD8Id = Shader.PropertyToID("_BHCBD8");
    Vector4 cbd6, cbd7, cbd8;
    double cbdGenPhase2 = 0.0;       // final-look clock (= the merged hole's disk clock)
    double cbdLifeSeconds2 = 0.0;

    const int DiskSlots = MaxBlackHoles + 1;   // + the merged hole's future disk
    const int FinalSlot = MaxBlackHoles;       // = BH_FINAL_SLOT in the shader

    static readonly int BHCBD9Id = Shader.PropertyToID("_BHCBD9");
    static readonly int BHCBD10Id = Shader.PropertyToID("_BHCBD10");
    static readonly int BHCBD11Id = Shader.PropertyToID("_BHCBD11");
    Vector4 cbd9, cbd10, cbd11;
    readonly HoleState finalDisk = new HoleState();
    int finalRowVersion = -1;

    static readonly int BHSpinAxisId = Shader.PropertyToID("_BHSpinAxis");
    readonly Vector4[] spinAxis = new Vector4[MaxBlackHoles];

    static readonly int BHCountId = Shader.PropertyToID("_BHCount");
    static readonly int BHPositionsId = Shader.PropertyToID("_BHPositionsCamRelative");
    static readonly int BHParamsId = Shader.PropertyToID("_BHParams");
    static readonly int BHDiskNormalsId = Shader.PropertyToID("_BHDiskNormals");
    static readonly int BHRotationOffsetsId = Shader.PropertyToID("_BHRotationOffsets");
    static readonly int BHSpinsId = Shader.PropertyToID("_BHSpins");
    static readonly int BHObserverGId = Shader.PropertyToID("_BHObserverG");
    static readonly int BHObserverTintId = Shader.PropertyToID("_BHObserverTint");
    static readonly int BHAppearance1Id = Shader.PropertyToID("_BHAppearanceProperties1");
    static readonly int BHAppearance2Id = Shader.PropertyToID("_BHAppearanceProperties2");
    static readonly int BHDiskConst0Id = Shader.PropertyToID("_BHDiskConst0");
    static readonly int BHDiskConst1Id = Shader.PropertyToID("_BHDiskConst1");
    static readonly int BHNoiseMeanId = Shader.PropertyToID("_BHNoiseMean");
    static readonly int BHPlungeLUTId = Shader.PropertyToID("_BHPlungeLUT");
    static readonly int BHHotSpotsId = Shader.PropertyToID("_BHHotSpots");
    static readonly int BHHotSpotBoundsId = Shader.PropertyToID("_BHHotSpotBounds");
    static readonly int BHSkyboxId = Shader.PropertyToID("_BHSkybox");
    static readonly int BHNoiseId = Shader.PropertyToID("_BHNoise");
    static readonly int KerrSnCnLUTId = Shader.PropertyToID("_KerrSnCnLUT");
    static readonly int KerrKLUTId = Shader.PropertyToID("_KerrKLUT");
    static readonly int KerrFLUTId = Shader.PropertyToID("_KerrFLUT");
    static readonly int BHNoiseSizeId = Shader.PropertyToID("_BHNoiseSize");
    static readonly int BHDiskTintId = Shader.PropertyToID("_BHDiskTint");
    static readonly int BHSpinJId = Shader.PropertyToID("_BHSpinJ");
    static readonly int BHDiskCutId = Shader.PropertyToID("_BHDiskCut");
    static readonly int BHCBD0Id = Shader.PropertyToID("_BHCBD0");
    static readonly int BHCBD1Id = Shader.PropertyToID("_BHCBD1");
    static readonly int BHCBD2Id = Shader.PropertyToID("_BHCBD2");
    static readonly int BHCBD3Id = Shader.PropertyToID("_BHCBD3");
    static readonly int BHCBD4Id = Shader.PropertyToID("_BHCBD4");
    static readonly int BHCBD5Id = Shader.PropertyToID("_BHCBD5");
    static readonly int BHCBDBlendId = Shader.PropertyToID("_BHCBDBlend");
    static readonly int BHLensCoverageId = Shader.PropertyToID("_BHLensCoverage");
    static readonly int BHSkyboxSizeId = Shader.PropertyToID("_BHSkyboxSize");

    const float MinAnalyticRadiusRs = 10f;
    const float MaxAnalyticRadiusRs = 300f;
    const float TidalTruncation = 0.7f;

    readonly Vector4[] diskNormals = new Vector4[MaxBlackHoles];
    readonly Vector4[] rotationOffsets = new Vector4[MaxBlackHoles];
    readonly Vector4[] spins = new Vector4[MaxBlackHoles];
    readonly Vector4[] positions = new Vector4[MaxBlackHoles];
    readonly Vector4[] parameters = new Vector4[MaxBlackHoles];
    readonly Vector4[] appearanceProperties1 = new Vector4[MaxBlackHoles];
    readonly Vector4[] appearanceProperties2 = new Vector4[MaxBlackHoles];
    readonly Vector4[] diskConst0 = new Vector4[DiskSlots];
    readonly Vector4[] diskConst1 = new Vector4[DiskSlots];
    readonly Vector4[] diskTints = new Vector4[DiskSlots];
    readonly Vector4[] spinJ = new Vector4[MaxBlackHoles];
    readonly Vector4[] diskCut = new Vector4[DiskSlots];

    const int HotSpotsMax = 6; // must match DISK_HOTSPOTS_MAX in the shader
    readonly Vector4[] hotSpots = new Vector4[DiskSlots * HotSpotsMax];
    readonly Vector4[] hotSpotBounds = new Vector4[DiskSlots];

    readonly float[] holeRs = new float[MaxBlackHoles];
    readonly Vector3[] holePos = new Vector3[MaxBlackHoles];
    readonly float[] holeInner = new float[MaxBlackHoles];
    readonly float[] holeMedium = new float[MaxBlackHoles];
    readonly float[] holeDiskOuter = new float[MaxBlackHoles];
    readonly float[] holeDiskVis = new float[MaxBlackHoles];
    readonly float[] holeSplit = new float[MaxBlackHoles];
    readonly float[] holeIsolated = new float[MaxBlackHoles];
    readonly float[] holePeakT = new float[MaxBlackHoles];
    readonly bool[] holeMerger = new bool[MaxBlackHoles];

    class HoleState
    {
        public GameObject go;
        public Properties props;
        public double genPhase;
        public float textureAngle;

        public float keySpin = float.NaN, keyRot = float.NaN, keyStress = float.NaN;
        public double rIsco, omegaIn, life, drdtIsco, eIsco, lIsco, fluxNorm, rLow;
        public readonly Color[] plungeRow = new Color[PlungeLutWidth];
        public int version;

        // disk outer edge, Unity units: as created, and as cut by tides so far (never regrows)
        public float origOuterW, permOuterW;

        // what is rendered this frame (differs from the physics during a merger)
        public Vector3 renderPos, renderUp;
        public float renderRs, renderSpin;
        public BlackHoleParamaters renderBh;
    }
    readonly Dictionary<GameObject, HoleState> states = new Dictionary<GameObject, HoleState>();
    readonly HoleState[] packed = new HoleState[MaxBlackHoles];

    readonly HoleState[] rowOwner = new HoleState[MaxBlackHoles];
    readonly int[] rowVersion = new int[MaxBlackHoles];
    Texture2D plungeLut;

    float noiseMean = 0.5f;
    Texture noiseMeanSource;
    GlobalKeyword singleHoleKeyword;

    Vector4 cbd0, cbd1, cbd2, cbd3, cbd4, cbd5;
    double cbdGenPhase = 0.0;
    double cbdLifeSeconds = 0.0;
    Vector3 cbdCenter;
    float cbdOuterWorld = 0f;

    // ---- merger animation --------------------------------------------------------
    class MergeAnim
    {
        public HoleState survivor, ghost;
        public int phase;                  // 0 plunge, 1 hand-over
        public Vector3 offA, offB, lHat;   // Unity units from the centre of mass, orbital axis
        public float rsA, rsB, spinA, spinB, massKeep, spinF;
        public Vector3 upA, upB, upF;
        public bool blendCbd;
        public double simT;        // simulation seconds since the merger
        public double plungeT;     // remaining inspiral time at the trigger (Peters)
        public double omega0;      // orbital angular velocity at the trigger (rad / sim s)
        public double phase1SimT;  // simT when the hand-over started
        public double handoverT;   // hand-over length (sim s)
    }
    MergeAnim anim;

    public Cubemap skybox;
    public Texture2D noiseTexture;

    public Texture2D kerrSnCnLUT;
    public Texture2D kerrKLUT;
    public Texture2D kerrFLUT;

    void OnEnable()
    {
        singleHoleKeyword = GlobalKeyword.Create("BH_SINGLE");
        if (plungeLut == null)
        {
            plungeLut = new Texture2D(PlungeLutWidth, DiskSlots, TextureFormat.RGFloat, false, true)
            {
                name = "BHPlungeLUT",
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                hideFlags = HideFlags.DontSave
            };
            System.Array.Clear(rowOwner, 0, rowOwner.Length);
        }
        RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
    }

    void OnDisable() => RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;

    void OnDestroy()
    {
        if (plungeLut != null) Destroy(plungeLut);
    }

    HoleState GetState(GameObject hole)
    {
        if (!states.TryGetValue(hole, out var st))
        {
            var rng = new System.Random(hole.GetEntityId());
            var props = hole.GetComponent<Properties>();
            var bh = props.blackHoleParamaters;
            st = new HoleState
            {
                go = hole,
                props = props,
                genPhase = rng.NextDouble() * 1024.0,
                textureAngle = (float)(rng.NextDouble() * 2.0 * System.Math.PI)
            };
            st.origOuterW = bh.accretionDisk ? bh.diskOuterRadiusNormalized * bh.schwarzschildRadiusUnits : 0f;
            st.permOuterW = st.origOuterW;
            states[hole] = st;
        }
        return st;
    }

    // ---------------------------------------------------------------------------
    // Kerr disk math (double). Same formulas as the shader.
    // ---------------------------------------------------------------------------
    static double PeakDiskTemperature(double massKg, double rIscoRs, double eIsco,
                                      double fluxNorm, double eddRatio)
    {
        const double SigmaSB = 5.670374419e-8;
        const double ProtonMass = 1.67262192e-27;
        const double SigmaThomson = 6.6524587e-29;
        double G = Constants.G, c = Constants.C;

        double lEdd = 4.0 * System.Math.PI * G * massKg * ProtonMass * c / SigmaThomson;
        double eta = System.Math.Max(1.0 - eIsco, 0.01);
        double mdot = eddRatio * lEdd / (eta * c * c);
        double rs = 2.0 * G * massKg / (c * c);
        double rIn = rIscoRs * rs;
        double t4 = 3.0 * G * massKg * mdot / (8.0 * System.Math.PI * SigmaSB * rIn * rIn * rIn) / fluxNorm;
        return System.Math.Pow(System.Math.Max(t4, 0.0), 0.25);
    }

    static double DiskOmega(double r, double spin, double keplerFactor)
    {
        double aPhys = System.Math.Clamp(spin, -0.998, 0.998) * KerrM;
        double sgn = keplerFactor < 0.0 ? -1.0 : 1.0;
        double sqrtM = System.Math.Sqrt(KerrM);
        double kepler = sgn * sqrtM / (r * System.Math.Sqrt(r) + sgn * aPhys * sqrtM);
        return System.Math.Abs(keplerFactor) * kepler;
    }

    static double DiskIscoRadius(double spin, double keplerFactor)
    {
        double sgn = keplerFactor < 0.0 ? -1.0 : 1.0;
        double chi = System.Math.Clamp(sgn * System.Math.Clamp(spin, -0.998, 0.998), -1.0, 1.0);
        double ac = System.Math.Abs(chi);
        double z1 = 1.0 + System.Math.Cbrt(1.0 - ac * ac) * (System.Math.Cbrt(1.0 + ac) + System.Math.Cbrt(1.0 - ac));
        double z2 = System.Math.Sqrt(3.0 * ac * ac + z1 * z1);
        double root = System.Math.Sqrt(System.Math.Max((3.0 - z1) * (3.0 + z1 + 2.0 * z2), 0.0));
        return KerrM * (3.0 + z2 + (chi >= 0.0 ? -root : root));
    }

    static void KerrIscoEL(double ae, double rIsco, out double E, out double L)
    {
        double M = KerrM;
        double sM = System.Math.Sqrt(M);
        double sri = System.Math.Sqrt(rIsco);
        double ri15 = rIsco * sri;
        double den = sri * System.Math.Sqrt(sri) * System.Math.Sqrt(System.Math.Max(ri15 - 3.0 * M * sri + 2.0 * ae * sM, 1e-8));
        E = (ri15 - 2.0 * M * sri + ae * sM) / den;
        L = sM * (rIsco * rIsco - 2.0 * ae * System.Math.Sqrt(M * rIsco) + ae * ae) / den;
    }

    static void PlungeVel(double r, double ae, double E, double L, out double ut, out double uph, out double ur)
    {
        double M = KerrM;
        double D = r * r - 2.0 * M * r + ae * ae;
        ut = ((r * r + ae * ae + 2.0 * M * ae * ae / r) * E - (2.0 * M * ae / r) * L) / D;
        uph = ((2.0 * M * ae / r) * E + (1.0 - 2.0 * M / r) * L) / D;
        double X = E * (r * r + ae * ae) - ae * L;
        double Rm = X * X - D * (r * r + (L - ae * E) * (L - ae * E));
        double uFall = System.Math.Sqrt(System.Math.Max(Rm, 0.0)) / (r * r);
        ur = System.Math.Sqrt(DiskDrift * DiskDrift + uFall * uFall);
    }

    static void BuildPlungeRow(HoleState st, double ae, double orbitSign)
    {
        double W = System.Math.Max(st.rIsco - st.rLow, 1e-4);
        const int Sub = 16;
        double h = 1.0 / ((PlungeLutWidth - 1) * (double)Sub);
        double P = 0.0, T = 0.0;
        st.plungeRow[0] = new Color(0f, 0f, 0f, 0f);
        for (int j = 1; j < PlungeLutWidth; j++)
        {
            double s0 = (j - 1) / (double)(PlungeLutWidth - 1);
            double sumP = 0.0, sumT = 0.0;
            for (int k = 0; k <= Sub; k++)
            {
                double s = s0 + k * h;
                double w = (k == 0 || k == Sub) ? 1.0 : ((k & 1) == 1 ? 4.0 : 2.0);
                double r = st.rIsco - W * s * s * s;
                double jac = 3.0 * W * s * s;
                PlungeVel(r, ae, st.eIsco, st.lIsco, out double ut, out double uph, out double ur);
                sumP += w * jac * uph / ur;
                sumT += w * jac * ut / ur;
            }
            P += sumP * h / 3.0;
            T += sumT * h / 3.0;
            st.plungeRow[j] = new Color((float)(orbitSign * P), (float)T, 0f, 0f);
        }
    }

    static void UpdateDerived(HoleState st, BlackHoleParamaters bh, BlackHoleAppearanceProperties bp)
    {
        if (bh.spin == st.keySpin && bh.diskRotation == st.keyRot && bp.diskISCOStress == st.keyStress)
            return;
        st.keySpin = bh.spin;
        st.keyRot = bh.diskRotation;
        st.keyStress = bp.diskISCOStress;

        const double M = KerrM;
        double aPhys = System.Math.Clamp((double)bh.spin, -0.998, 0.998) * M;
        double orbitSign = bh.diskRotation < 0f ? -1.0 : 1.0;

        st.rIsco = DiskIscoRadius(bh.spin, bh.diskRotation);
        st.omegaIn = DiskOmega(st.rIsco, bh.spin, bh.diskRotation);
        st.life = DiskNoiseLifeOrbits * 2.0 * System.Math.PI / System.Math.Max(System.Math.Abs(st.omegaIn), 1e-6);

        double r = st.rIsco, om = st.omegaIn;
        double delta = r * r - 2.0 * M * r + aPhys * aPhys;
        double gtt = -(1.0 - 2.0 * M / r);
        double gtp = -2.0 * M * aPhys / r;
        double gpp = r * r + aPhys * aPhys + 2.0 * M * aPhys * aPhys / r;
        double norm = -(gtt + 2.0 * gtp * om + gpp * om * om);
        double ut = (norm > 0.0 && delta > 0.0)
            ? System.Math.Sqrt((1.0 + r * r / delta * DiskDrift * DiskDrift) / norm)
            : 1.0;
        st.drdtIsco = DiskDrift / System.Math.Max(ut, 1e-4);

        double c = System.Math.Max(1.0 - bp.diskISCOStress, 1e-3);
        double xPk = (7.0 * c / 6.0) * (7.0 * c / 6.0);
        double fPk = (1.0 - c / System.Math.Sqrt(xPk)) / (xPk * xPk * xPk);
        st.fluxNorm = 1.0 / fPk;

        double ae = orbitSign * aPhys;
        KerrIscoEL(ae, st.rIsco, out st.eIsco, out st.lIsco);
        st.rLow = M + System.Math.Sqrt(System.Math.Max(M * M - aPhys * aPhys, 0.0)) + 0.05;
        BuildPlungeRow(st, ae, orbitSign);
        st.version++;
    }

    static Vector3 Blackbody(float T)
    {
        float t = (Mathf.Log10(Mathf.Clamp(T, 1000f, 40000f)) - 3.8f) * 1.25f;
        float r = ((((((-0.318020f * t + 0.307137f) * t + 0.483045f) * t - 1.126463f) * t + 1.291352f) * t - 0.997409f) * t + 1.059097f);
        float g = ((((((0.126835f * t - 0.133110f) * t - 0.214311f) * t + 0.457298f) * t - 0.342339f) * t + 0.107313f) * t + 0.982236f);
        float b = ((((((-0.319972f * t + 0.414174f) * t + 0.700556f) * t - 1.212946f) * t - 0.411255f) * t + 1.873784f) * t + 1.002553f);
        return new Vector3(Mathf.Max(r, 0f), Mathf.Max(g, 0f), Mathf.Max(b, 0f));
    }

    static Vector4 ObserverTint(float gObs)
    {
        float gO = Mathf.Pow(Mathf.Max(gObs, 1f), ObserverShiftStrength);
        if (gO < 1.0005f)
            return new Vector4(1f, 1f, 1f, 1f);
        Vector3 shifted = Blackbody(6000f * gO);
        Vector3 rest = Blackbody(6000f);
        float g4 = gO * gO * gO * gO;
        return new Vector4(shifted.x / rest.x * g4, shifted.y / rest.y * g4, shifted.z / rest.z * g4, 1f);
    }

    static float ComputeMeanRed(Texture tex)
    {
        var rt = RenderTexture.GetTemporary(tex.width, tex.height, 0,
            RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
        Graphics.Blit(tex, rt);
        var prev = RenderTexture.active;
        RenderTexture.active = rt;
        var tmp = new Texture2D(tex.width, tex.height, TextureFormat.RGBAFloat, false, true);
        tmp.ReadPixels(new Rect(0, 0, tex.width, tex.height), 0, 0, false);
        tmp.Apply(false);
        RenderTexture.active = prev;
        RenderTexture.ReleaseTemporary(rt);

        var px = tmp.GetPixels();
        double sum = 0.0;
        for (int i = 0; i < px.Length; i++) sum += px[i].r;
        Destroy(tmp);
        return (float)(sum / System.Math.Max(px.Length, 1));
    }

    // ---------------------------------------------------------------------------
    // Clocks: once per frame. (The disk constants are refreshed while rendering, from the
    // rendered spin, which differs from the physics during a merger.)
    // ---------------------------------------------------------------------------
    void Update()
    {
        if (noiseTexture != null && noiseTexture != noiseMeanSource)
        {
            noiseMean = ComputeMeanRed(noiseTexture);
            noiseMeanSource = noiseTexture;
        }

        double simDt = Time.deltaTime * (double)GlobalProperties.timeScale;
        if (anim != null) anim.simT += simDt;

        foreach (var hole in blackHoles)
        {
            if (hole == null) continue;
            var st = GetState(hole);
            var bh = st.props.blackHoleParamaters;

            double rsOverC = 2.0 * Constants.G * bh.mass / (Constants.C * Constants.C) / Constants.C;
            if (System.Math.Abs(st.omegaIn) < 1e-12 || rsOverC <= 0.0) continue;

            double lifeSeconds = st.life * rsOverC;
            st.genPhase += simDt / lifeSeconds;
            if (st.genPhase >= 1024.0) st.genPhase -= 1024.0;
        }

        if (cbdLifeSeconds > 0.0)
        {
            cbdGenPhase += simDt / cbdLifeSeconds;
            if (cbdGenPhase >= 1024.0) cbdGenPhase -= 1024.0;
        }

        if (cbdLifeSeconds2 > 0.0)
        {
            cbdGenPhase2 += simDt / cbdLifeSeconds2;
            if (cbdGenPhase2 >= 1024.0) cbdGenPhase2 -= 1024.0;
        }
    }

    static float AccuracyRadiusRs(Camera cam, float spin)
    {
        float pixAngle = 2f * Mathf.Tan(0.5f * cam.fieldOfView * Mathf.Deg2Rad) / Mathf.Max(cam.pixelHeight, 1);
        float tol = 0.25f * pixAngle;
        float chi = Mathf.Abs(spin);
        float bSeries = Mathf.Pow(26f / tol, 0.2f);
        float bSpin = Mathf.Max(Mathf.Pow(1.7f * chi * chi / tol, 1f / 3f), Mathf.Pow(28f * chi / tol, 0.25f));
        return Mathf.Clamp(Mathf.Max(bSeries, bSpin), MinAnalyticRadiusRs, MaxAnalyticRadiusRs);
    }

    static Vector2 GenRandom(double n)
    {
        unchecked
        {
            uint h = (uint)(((long)System.Math.Floor(n)) & 1023) * 747796405u + 2891336453u;
            h = ((h >> (int)((h >> 28) + 4u)) ^ h) * 277803737u;
            h = (h >> 22) ^ h;
            uint h2 = h * 747796405u + 2891336453u;
            h2 = ((h2 >> (int)((h2 >> 28) + 4u)) ^ h2) * 277803737u;
            h2 = (h2 >> 22) ^ h2;
            return new Vector2((h & 0xFFFFu) / 65536f, (h2 & 0xFFFFu) / 65536f);
        }
    }

    void FillHotSpots(int i, BlackHoleParamaters bh, BlackHoleAppearanceProperties bp, HoleState st)
    {
        double rIsco = st.rIsco;
        double life = st.life;
        int n = Mathf.Clamp(bp.hotSpotCount, 0, HotSpotsMax);
        float rLo = float.MaxValue, rHi = -1f;

        for (int s = 0; s < HotSpotsMax; s++)
        {
            Vector4 v = new Vector4(1f, 0f, 0f, 1f);
            if (s < n && bh.accretionDisk)
            {
                double g = st.genPhase + (double)s / n;
                double age = g - System.Math.Floor(g);
                Vector2 rnd = GenRandom(System.Math.Floor(g) + 131.0 * (s + 1));
                double rS = rIsco * Mathf.Lerp(bp.hotSpotRMin, bp.hotSpotRMax, rnd.x);
                double angle = rnd.y * 2.0 * System.Math.PI
                             + DiskOmega(rS, bh.spin, bh.diskRotation) * age * life
                             + st.textureAngle;
                angle %= 2.0 * System.Math.PI;
                double env = System.Math.Sin(System.Math.PI * age);
                float amp = (float)(env * env) * bp.hotSpotStrength;
                float sigma = (float)(0.18 * rS);
                v = new Vector4((float)rS, (float)angle, amp, sigma);
                if (amp > 1e-3f)
                {
                    rLo = Mathf.Min(rLo, (float)rS - 3f * sigma);
                    rHi = Mathf.Max(rHi, (float)rS + 3f * sigma);
                }
            }
            hotSpots[i * HotSpotsMax + s] = v;
        }
        hotSpotBounds[i] = (rHi > 0f)
            ? new Vector4(Mathf.Max(rLo, (float)rIsco), rHi, 0f, 0f)
            : new Vector4(1e9f, -1f, 0f, 0f);
    }

    // ---------------------------------------------------------------------------
    // Merger animation
    //  phase 0 (plunge):   the physics already has the remnant; on screen the two holes
    //                      spiral into each other, their mass ramps down by the radiated
    //                      fraction and their spins to 0. When they coincide, two holes at one
    //                      point ARE one Schwarzschild hole: the switch is invisible.
    //  phase 1 (hand-over): the remnant's spin builds up from 0, and the frozen shared disk,
    //                      drawn at the remnant's own disk crossings, cross-fades into the
    //                      remnant's disk.
    // ---------------------------------------------------------------------------
    public void OnMerger(GameObject keep, GameObject gone, Vector3 offKeep, Vector3 offGone, Vector3 lHat,
                         float rsKeep, float rsGone, float spinKeep, float spinGone,
                         Vector3 upKeep, Vector3 upGone, float massKeep, float spinF, Vector3 upF)
    {
        if (anim != null)
        {
            if (anim.phase == 0) FinishPlunge();
            anim = null;
        }
        // remaining inspiral from this separation, as the 2.5PN physics would continue it:
        //   D^4 falls linearly, T = (5/32) D0^4 / (c Rs1 Rs2 (Rs1 + Rs2)), omega0 = Kepler
        var bhK = GetState(keep).props.blackHoleParamaters;
        double mPerUnit = (double)bhK.schwarzschildRadiusWorld / System.Math.Max(bhK.schwarzschildRadiusUnits, 1e-30);
        double D0 = (offGone - offKeep).magnitude * mPerUnit;
        double r1 = rsKeep * mPerUnit, r2 = rsGone * mPerUnit;
        double c = Constants.C;
        double plungeT = (D0 > 0.0 && r1 > 0.0 && r2 > 0.0)
            ? (5.0 / 32.0) * D0 * D0 * D0 * D0 / (c * r1 * r2 * (r1 + r2)) * mergerTimeStretch
            : 0.0;
        double omega0 = D0 > 0.0 ? System.Math.Sqrt(0.5 * c * c * (r1 + r2) / (D0 * D0 * D0)) / mergerTimeStretch : 0.0;

        anim = new MergeAnim
        {
            survivor = GetState(keep),
            ghost = GetState(gone),
            phase = 0,
            simT = 0.0,
            plungeT = plungeT,
            omega0 = omega0,
            offA = offKeep,
            offB = offGone,
            lHat = lHat.normalized,
            rsA = rsKeep,
            rsB = rsGone,
            spinA = spinKeep,
            spinB = spinGone,
            upA = upKeep,
            upB = upGone,
            massKeep = massKeep,
            spinF = spinF,
            upF = upF
        };
    }

    void AdvanceMergeAnim()
    {
        if (anim == null) return;
        if (anim.phase == 0 && anim.simT >= anim.plungeT)
            FinishPlunge();
        if (anim != null && anim.phase == 1 && anim.simT - anim.phase1SimT >= anim.handoverT)
            anim = null;
    }

    void FinishPlunge()
    {
        // the ghost goes
        var g = anim.ghost;
        if (g != null && g.go != null)
        {
            GlobalProperties.blackHoles.Remove(g.go);
            for (int k = 0; k < MaxBlackHoles; k++)
                if (rowOwner[k] == g) rowOwner[k] = null;
            states.Remove(g.go);
            g.go.SetActive(false);
            Destroy(g.go);
        }

        // the shared disk becomes the remnant's own disk: same outer edge, permanently
        var s = anim.survivor;
        var bh = s.props.blackHoleParamaters;
        anim.blendCbd = false;
        if (bh.accretionDisk && cbdOuterWorld > 0f && bh.schwarzschildRadiusUnits > 0f)
        {
            bh.diskOuterRadiusNormalized = cbdOuterWorld / bh.schwarzschildRadiusUnits;
            bh.escapeRadiusNormalized = bh.diskOuterRadiusNormalized * 2f;
            s.props.blackHoleParamaters = bh;
            s.origOuterW = cbdOuterWorld;
            s.permOuterW = cbdOuterWorld;                
            s.genPhase = cbdGenPhase2;
            s.textureAngle = 0f;
        }
        else if (bh.accretionDisk)
        {
            s.origOuterW = s.permOuterW = bh.diskOuterRadiusNormalized * bh.schwarzschildRadiusUnits;
        }
        anim.phase = 1;
        anim.phase1SimT = anim.simT;
        // ringdown time in units of the remnant's mass: t = N G M / c^3 = N (Rs / 2) / c
        double rsFm = (double)s.props.blackHoleParamaters.schwarzschildRadiusWorld;
        anim.handoverT = mergerRingdownM * 0.5 * rsFm / Constants.C * mergerTimeStretch;
    }

    // D / D0 on the inspiral (D^4 falls linearly in time): 1 at the trigger, 0 at the merger
    float PlungeFrac()
    {
        if (anim == null || anim.plungeT <= 0.0) return 0f;
        return (float)System.Math.Pow(System.Math.Max(1.0 - anim.simT / anim.plungeT, 0.0), 0.25);
    }

    // orbital angle since the trigger: integral of omega0 (D0/D)^1.5 dt
    float PlungeAngleDeg()
    {
        if (anim == null || anim.plungeT <= 0.0) return 0f;
        double tau = System.Math.Min(anim.simT / anim.plungeT, 1.0);
        double ang = anim.omega0 * anim.plungeT * 1.6 * (1.0 - System.Math.Pow(1.0 - tau, 0.625));
        return (float)(ang * (180.0 / System.Math.PI) % 360.0);
    }

    float HandoverT() => (anim == null || anim.handoverT <= 0.0) ? 1f
        : Mathf.Clamp01((float)((anim.simT - anim.phase1SimT) / anim.handoverT));

    void RenderState(HoleState st)
    {
        var bh = st.props.blackHoleParamaters;
        st.renderPos = st.go.transform.position;
        st.renderRs = bh.schwarzschildRadiusUnits;
        st.renderSpin = bh.spin;
        st.renderUp = st.go.transform.up;

        if (anim != null)
        {
            if (anim.phase == 0 && (st == anim.survivor || st == anim.ghost))
            {
                float frac = PlungeFrac();          // D / D0
                float x = 1f - frac;                 // progress by separation
                bool isS = st == anim.survivor;
                Quaternion q = Quaternion.AngleAxis(PlungeAngleDeg(), anim.lHat);
                Vector3 off = q * (isS ? anim.offA : anim.offB) * frac;
                st.renderPos = anim.survivor.go.transform.position + off;   // survivor = centre of mass
                st.renderRs = (isS ? anim.rsA : anim.rsB) * Mathf.Lerp(1f, anim.massKeep, x);
                st.renderSpin = Mathf.Lerp(isS ? anim.spinA : anim.spinB, 0f, x);
                st.renderUp = isS ? anim.upA : anim.upB;
            }
            else if (anim.phase == 1 && st == anim.survivor)
            {
                // start in the shared disk's plane (the orbit's), then tilt with the spin to its
                // final direction: the frozen shared disk is drawn at this hole's disk crossings,
                // so at the start of the hand-over it is exactly where it was a moment before
                float v = Mathf.SmoothStep(0f, 1f, HandoverT());
                st.renderSpin = anim.spinF * v;
                st.renderUp = Vector3.Slerp(anim.lHat, anim.upF, v).normalized;
            }
        }
        bh.spin = st.renderSpin;
        bh.schwarzschildRadiusUnits = st.renderRs;
        st.renderBh = bh;
    }

    // ---------------------------------------------------------------------------
    // Shared disk. Made of the gas the holes' own disks lost to tides: its area is the
    // area they lost (x cbdOuterScale^2), around the central cavity.
    // ---------------------------------------------------------------------------
    bool CircumbinaryGeometry(int a, int b, out Vector3 c, out Vector3 nrm,
                              out float rIn, out float rOut, out float w)
    {
        c = Vector3.zero; nrm = Vector3.up; rIn = 0f; rOut = 0f; w = 0f;
        if (a < 0) return false;

        var A = packed[a];
        var B = packed[b];
        var bhA = A.renderBh;
        var bhB = B.renderBh;
        float rsA = holeRs[a], rsB = holeRs[b];
        float wa = rsA / (rsA + rsB), wb = 1f - wa;
        float rsTot = rsA + rsB;

        Vector3 rel = holePos[b] - holePos[a];
        float D = rel.magnitude;

        float spinRem = Mathf.Clamp(wa * bhA.spin + wb * bhB.spin, -0.998f, 0.998f);
        float kRem = bhA.accretionDisk ? bhA.diskRotation : bhB.diskRotation;
        float rIscoRem = (float)DiskIscoRadius(spinRem, kRem) * rsTot;
        rIn = Mathf.Max(cbdCavity * D, rIscoRem);

        float stripped = Mathf.Max(A.origOuterW * A.origOuterW - A.permOuterW * A.permOuterW, 0f)
                       + Mathf.Max(B.origOuterW * B.origOuterW - B.permOuterW * B.permOuterW, 0f);
        rOut = Mathf.Sqrt(rIn * rIn + cbdOuterScale * cbdOuterScale * stripped);

        w = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01((rOut - rIn) / (0.4f * rOut)));
        if (w <= 0f) return false;

        c = holePos[a] * wa + holePos[b] * wb;
        bool plungePair = anim != null && anim.phase == 0 &&
                          ((A == anim.survivor && B == anim.ghost) || (A == anim.ghost && B == anim.survivor));
        if (plungePair)
        {
            nrm = Vector3.Slerp(anim.lHat, anim.upF, 1f - PlungeFrac()).normalized;
        }
        else
        {
            // orbital angular momentum direction
            var va = A.props.velocity;
            var vb = B.props.velocity;
            Vector3 vrel = new Vector3((float)(vb.x - va.x), (float)(vb.y - va.y), (float)(vb.z - va.z));
            Vector3 lv = Vector3.Cross(rel, vrel);
            if (vrel.sqrMagnitude <= 0f || lv.sqrMagnitude <= 1e-12f * rel.sqrMagnitude * vrel.sqrMagnitude)
                lv = A.renderUp * wa + B.renderUp * wb;
            lv = lv.sqrMagnitude > 0f ? lv.normalized : Vector3.up;

            // Lie in the plane the merged hole's disk will have: perpendicular to its spin
            // axis, predicted from the current orbit and spins (same formula as the merger),
            // on the orbit's side. Then nothing has to tilt when the merged hole takes over.
            var L = new Unity.Mathematics.double3(lv.x, lv.y, lv.z);
            var a1 = new Unity.Mathematics.double3(A.renderUp.x, A.renderUp.y, A.renderUp.z) * bhA.spin;
            var a2 = new Unity.Mathematics.double3(B.renderUp.x, B.renderUp.y, B.renderUp.z) * bhB.spin;
            var af = BlackHoleMerger.FinalSpin(rsA, rsB, a1, a2, L);
            nrm = lv;
            if (Unity.Mathematics.math.lengthsq(af) > 1e-6)
            {
                var ax = Unity.Mathematics.math.normalize(af);
                if (Unity.Mathematics.math.dot(ax, L) < 0) ax = -ax;
                nrm = new Vector3((float)ax.x, (float)ax.y, (float)ax.z);
            }
        }
        return true;
    }

    void FillCircumbinaryDisk(int a, int b)
    {
        // hand-over after a merger: frozen (final look), drawn only by the remnant's exact trace
        if (anim != null && anim.phase == 1 && anim.blendCbd)
        {
            cbd2.w = 0f;
            cbd7.w = 1f;
            double g11 = System.Math.Floor(cbdGenPhase), g21 = System.Math.Floor(cbdGenPhase2);
            cbd5 = new Vector4(0f, (float)(cbdGenPhase - g11), (float)g11, 0f);
            cbd8 = new Vector4(0f, (float)(cbdGenPhase2 - g21), (float)g21, 0f);
            cbdOuterWorld = 0f;
            return;
        }

        cbd2 = Vector4.zero;
        cbdLifeSeconds = 0.0;
        cbdLifeSeconds2 = 0.0;
        cbdOuterWorld = 0f;
        if (!CircumbinaryGeometry(a, b, out Vector3 c, out Vector3 nrm, out float rIn, out float rOut, out float w))
            return;

        var A = packed[a];
        var B = packed[b];
        var bhA = A.props.blackHoleParamaters;
        var bhB = B.props.blackHoleParamaters;
        var apA = A.props.blackHoleAppearanceProperties;
        var apB = B.props.blackHoleAppearanceProperties;
        float rsA = holeRs[a], rsB = holeRs[b];
        float wa = rsA / (rsA + rsB), wb = 1f - wa;
        float rsTot = rsA + rsB;

        // ---- how far the cavity has closed: 0 when the shared disk appears, 1 at the merger
        float spinRem = Mathf.Clamp(wa * A.renderBh.spin + wb * B.renderBh.spin, -0.998f, 0.998f);
        float kRem = A.renderBh.accretionDisk ? A.renderBh.diskRotation : B.renderBh.diskRotation;
        float rIscoRem = (float)DiskIscoRadius(spinRem, kRem) * rsTot;
        float p = 1f - Mathf.Clamp01((rIn - rIscoRem) / Mathf.Max(rOut - rIscoRem, 1e-6f));
        p = Mathf.Pow(Mathf.SmoothStep(0f, 1f, p), Mathf.Max(cbdLookCurve, 1e-3f));

        // ---- shared by both looks -------------------------------------------------
        float contrast = wa * apA.diskRadialContrast + wb * apB.diskRadialContrast;
        cbd0 = new Vector4(c.x, c.y, c.z, rIn);
        cbd1 = new Vector4(nrm.x, nrm.y, nrm.z, rOut);
        cbd3 = new Vector4(wa * apA.diskDopplerColor + wb * apB.diskDopplerColor,
                           wa * apA.diskDopplerBeaming + wb * apB.diskDopplerBeaming,
                           contrast,
                           wa * apA.diskTurbulance + wb * apB.diskTurbulance);
        cbd4 = diskTints[a] * wa + diskTints[b] * wb;

        // ---- early look (as before) --------------------------------------------------
        double QA = System.Math.Pow(holePeakT[a], 4.0) * System.Math.Pow(A.rIsco * rsA, 3.0) * A.fluxNorm;
        double QB = System.Math.Pow(holePeakT[b], 4.0) * System.Math.Pow(B.rIsco * rsB, 3.0) * B.fluxNorm;
        double tIn = System.Math.Sqrt(System.Math.Sqrt(QA) + System.Math.Sqrt(QB)) / System.Math.Pow(rIn, 0.75);
        double t4Avg = wa * System.Math.Pow(holePeakT[a], 4.0) + wb * System.Math.Pow(holePeakT[b], 4.0);
        float brightAvg = wa * apA.diskBrightness + wb * apB.diskBrightness;
        float brightEarly = brightAvg * cbdBrightness
                          * (float)System.Math.Pow(System.Math.Pow(tIn, 4.0) / System.Math.Max(t4Avg, 1e-30), contrast);

        double mPerUnit = (double)bhA.schwarzschildRadiusWorld / System.Math.Max(bhA.schwarzschildRadiusUnits, 1e-30);
        double rInM = rIn * mPerUnit;
        double gm = Constants.G * (bhA.mass + bhB.mass);
        cbdLifeSeconds = DiskNoiseLifeOrbits * 2.0 * System.Math.PI * System.Math.Sqrt(rInM * rInM * rInM / gm)
                       / System.Math.Max(cbdSpeed, 1e-3);

        // ---- final look: the merged hole's future disk ---------------------------------
        bool plungePair = anim != null && anim.phase == 0 &&
                          ((A == anim.survivor && B == anim.ghost) || (A == anim.ghost && B == anim.survivor));
        double rsF;
        if (plungePair)
            rsF = (anim.rsA + anim.rsB) * anim.massKeep;   // already merged in the physics
        else
        {
            double eta = (double)rsA * rsB / ((double)rsTot * rsTot);
            rsF = rsTot * (1.0 - BlackHoleMerger.RadiatedFraction(eta));
        }
        double rsFm = rsF * mPerUnit;
        double massF = rsFm * Constants.C * Constants.C / (2.0 * Constants.G);

        var heavy = wa >= 0.5f ? apA : apB;
        float stress = wa * apA.diskISCOStress + wb * apB.diskISCOStress;
        float eddRatio = wa * apA.eddingtonRatio + wb * apB.eddingtonRatio;
        const double rIscoF = 3.0;   // spin 0
        double cS = System.Math.Max(1.0 - stress, 1e-3);
        double xPk = (7.0 * cS / 6.0) * (7.0 * cS / 6.0);
        double fluxNorm = 1.0 / ((1.0 - cS / System.Math.Sqrt(xPk)) / (xPk * xPk * xPk));
        float peakF = heavy.physicalDiskTemperature
            ? (float)PeakDiskTemperature(massF, rIscoF, System.Math.Sqrt(8.0 / 9.0), fluxNorm, eddRatio)
            : wa * apA.diskPeakTemperature + wb * apB.diskPeakTemperature;
        double lifeRs = DiskNoiseLifeOrbits * 2.0 * System.Math.PI / DiskOmega(rIscoF, 0.0, 1.0);
        cbdLifeSeconds2 = lifeRs * rsFm / Constants.C;   // the merged hole's disk clock

        // ---- pack ---------------------------------------------------------------------
        float massOrbit = Mathf.Lerp(0.5f * rsTot, (float)(0.5 * rsF), p);   // total -> remnant
        cbd2 = new Vector4(massOrbit, (float)tIn, brightEarly, w);
        double g1 = System.Math.Floor(cbdGenPhase), g2 = System.Math.Floor(cbdGenPhase2);
        cbd5 = new Vector4(0f, (float)(cbdGenPhase - g1), (float)g1, 0f);
        cbd6 = new Vector4(peakF, brightAvg, stress, (float)fluxNorm);
        cbd7 = new Vector4((float)rsF, (float)rIscoF, (float)lifeRs, p);
        cbd8 = new Vector4(0f, (float)(cbdGenPhase2 - g2), (float)g2, 0f);
        FillFinalDiskSlot(plungePair ? anim.survivor : (wa >= 0.5f ? A : B), massF, (float)rsF, rOut);
        // the final look's cavity: the same cavity·separation the early look's inner edge
        // uses (CircumbinaryGeometry), without its ISCO floor, so it closes to 0 at the end
        cbd11.z = cbdCavity * Vector3.Distance(holePos[a], holePos[b]);

        cbdCenter = c;
        cbdOuterWorld = rOut;
    }

    // Disk slot FinalSlot = the disk the merged hole will have when the plunge ends:
    // the surviving hole's settings, the remnant's mass, spin 0, the final-look clock.
    // Built with the same functions the survivor's own disk uses afterwards.
    void FillFinalDiskSlot(HoleState heavy, double massF, float rsF, float outerWorld)
    {
        var bh = heavy.props.blackHoleParamaters;
        var bp = heavy.props.blackHoleAppearanceProperties;
        bh.spin = 0f;
        bh.schwarzschildRadiusUnits = rsF;
        UpdateDerived(finalDisk, bh, bp);
        finalDisk.genPhase = cbdGenPhase2;
        finalDisk.textureAngle = 0f;
        FillHotSpots(FinalSlot, bh, bp, finalDisk);

        diskConst0[FinalSlot] = new Vector4((float)finalDisk.rIsco, (float)finalDisk.life, (float)finalDisk.omegaIn, (float)finalDisk.drdtIsco);
        diskConst1[FinalSlot] = new Vector4((float)finalDisk.eIsco, (float)finalDisk.lIsco, (float)finalDisk.fluxNorm, (float)finalDisk.rLow);
        diskCut[FinalSlot] = Vector4.zero;
        Color lin = bp.diskTint.linear;
        float tintLum = 0.2126f * lin.r + 0.7152f * lin.g + 0.0722f * lin.b;
        diskTints[FinalSlot] = tintLum > 1e-4f
            ? new Vector4(lin.r / tintLum, lin.g / tintLum, lin.b / tintLum, bp.diskTintStrength)
            : new Vector4(1f, 1f, 1f, 0f);

        if (finalRowVersion != finalDisk.version)
        {
            plungeLut.SetPixels(0, FinalSlot, PlungeLutWidth, 1, finalDisk.plungeRow);
            plungeLut.Apply(false, false);
            finalRowVersion = finalDisk.version;
        }

        float peakT = bp.physicalDiskTemperature
            ? (float)PeakDiskTemperature(massF, finalDisk.rIsco, finalDisk.eIsco, finalDisk.fluxNorm, bp.eddingtonRatio)
            : bp.diskPeakTemperature;
        cbd9 = new Vector4(peakT, bp.diskDopplerColor, bp.diskDopplerBeaming, bp.diskRadialContrast);
        cbd10 = new Vector4(bp.diskBrightness, bp.diskISCOStress, bp.diskTurbulance, bp.diskHigherOrderFade);
        // outer edge exactly as FinishPlunge will write it (diskOuterRadiusNormalized)
        float outerN = Mathf.Max(outerWorld / Mathf.Max(rsF, 1e-6f), (float)finalDisk.rIsco * 1.05f);
        cbd11 = new Vector4(bh.diskRotation, outerN, 0f, 0f);
    }

    [Header("Cameras")]
    [Tooltip("Match BlackHoleRendererFeature's renderInSceneView.")]
    public bool updateForSceneView = false;

    void OnBeginCameraRendering(ScriptableRenderContext context, Camera cam)
    {
        if (cam.cameraType == CameraType.Preview || cam.cameraType == CameraType.Reflection) return;
        if (cam.cameraType == CameraType.SceneView && !updateForSceneView) return;
        AdvanceMergeAnim();

        int count = 0;
        foreach (var hole in blackHoles)
        {
            if (hole == null) continue;
            if (count >= MaxBlackHoles) break;
            var st = GetState(hole);
            RenderState(st);
            UpdateDerived(st, st.renderBh, st.props.blackHoleAppearanceProperties);
            packed[count++] = st;
        }

        for (int i = 0; i < count; i++)
        {
            holeRs[i] = packed[i].renderRs;
            holePos[i] = packed[i].renderPos;
            holeMerger[i] = false;
        }

        // ---- pairs: merger mode (minimal spheres no longer fit), closest disk pair ------
        int cbdA = -1, cbdB = -1;
        float cbdSep = float.PositiveInfinity;
        for (int i = 0; i < count; i++)
            for (int j = i + 1; j < count; j++)
            {
                float D = Vector3.Distance(holePos[i], holePos[j]);
                float sqI = Mathf.Sqrt(holeRs[i]), sqJ = Mathf.Sqrt(holeRs[j]);
                float splitI = D * sqI / (sqI + sqJ) / Mathf.Max(holeRs[i], 1e-6f);
                float splitJ = D * sqJ / (sqI + sqJ) / Mathf.Max(holeRs[j], 1e-6f);
                if (0.85f * Mathf.Min(splitI, splitJ) < 1.05f * KerrFloorRs)
                    holeMerger[i] = holeMerger[j] = true;

                float sep = D / (holeRs[i] + holeRs[j]);
                bool anyDisk = packed[i].renderBh.accretionDisk || packed[j].renderBh.accretionDisk;
                if (anyDisk && sep < cbdSep)
                {
                    cbdSep = sep;
                    cbdA = i;
                    cbdB = j;
                }
            }

        // ---- pass A: geometry of every hole's regions (Rs units) ------------------
        for (int i = 0; i < count; i++)
        {
            var st = packed[i];
            var bh = st.renderBh;
            float rs = holeRs[i];

            float split = float.PositiveInfinity;
            for (int j = 0; j < count; j++)
            {
                if (j == i) continue;
                float D = Vector3.Distance(holePos[i], holePos[j]);
                float di = D * Mathf.Sqrt(rs) / (Mathf.Sqrt(rs) + Mathf.Sqrt(holeRs[j]));
                split = Mathf.Min(split, di / Mathf.Max(rs, 1e-6f));
            }
            holeSplit[i] = split;

            // tides cut the disk; what they cut stays cut (it becomes the shared disk)
            if (bh.accretionDisk && rs > 0f)
            {
                st.permOuterW = Mathf.Min(st.permOuterW, TidalTruncation * split * rs);
                float physRs = st.props.blackHoleParamaters.schwarzschildRadiusUnits;
                if (physRs > 0f)
                {
                    float newNorm = st.permOuterW / physRs;
                    var pbh = st.props.blackHoleParamaters;
                    if (newNorm < pbh.diskOuterRadiusNormalized - 1e-4f)
                    {
                        pbh.diskOuterRadiusNormalized = newNorm;
                        pbh.escapeRadiusNormalized = newNorm * 2f;
                        st.props.blackHoleParamaters = pbh;
                    }
                }
            }
            float diskNorm = (bh.accretionDisk && rs > 0f) ? st.permOuterW / rs : 0f;

            float isolatedRs = bh.accretionDisk
                ? Mathf.Max(AccuracyRadiusRs(cam, bh.spin), diskNorm * 1.5f)
                : AccuracyRadiusRs(cam, bh.spin);
            holeIsolated[i] = isolatedRs;

            float diskOuter = 0f, rIn = 0f, diskVis = 0f;
            if (!holeMerger[i])
            {
                float rSpin = Mathf.Lerp(innerKerrRadiusSlowRs, innerKerrRadiusFastRs, Mathf.Abs(bh.spin));
                rIn = Mathf.Max(Mathf.Min(rSpin, 0.3f * split), KerrFloorRs);

                if (bh.accretionDisk)
                {
                    diskOuter = Mathf.Max(Mathf.Min(diskNorm, (0.85f * split - 0.5f) / 1.05f), 0f);
                    float rIscoF = Mathf.Max((float)st.rIsco, 1e-3f);
                    diskVis = Mathf.Clamp01((diskOuter - 1.3f * rIscoF) / rIscoF);
                    rIn = Mathf.Lerp(rIn, Mathf.Max(rIn, diskOuter * 1.05f + 0.5f), diskVis);
                    diskOuter = Mathf.Min(diskOuter, Mathf.Max((rIn - 0.5f) / 1.05f, 0f));
                }
            }

            holeInner[i] = rIn;
            holeDiskOuter[i] = diskOuter;
            holeDiskVis[i] = diskVis;
            holeMedium[i] = Mathf.Max(isolatedRs, rIn * 1.5f);
        }

        bool cbdOn = CircumbinaryGeometry(cbdA, cbdB, out Vector3 cbdC, out _, out _, out float cbdROut, out _);

        // ---- pass B: membership, redshift, disk fade, shader data -----------------
        bool lutDirty = false;
        int survivorIndex = -1;
        for (int i = 0; i < count; i++)
        {
            var st = packed[i];
            var bh = st.renderBh;
            var bp = st.props.blackHoleAppearanceProperties;
            float rs = holeRs[i];
            if (anim != null && st == anim.survivor) survivorIndex = i;

            bool member = debugForceMedium || holeMerger[i];
            double S = 0.0;
            for (int j = 0; j < count; j++)
            {
                if (j == i) continue;
                float D = Vector3.Distance(holePos[i], holePos[j]);
                if (D < holeIsolated[i] * rs + holeIsolated[j] * holeRs[j])
                    member = true;
                S += 0.5 * holeRs[j] / Mathf.Max(D, holeRs[j]);
            }
            float gExt = (float)((1.0 - 0.5 * S) / (1.0 + 0.5 * S));

            if (member)
            {
                float rMed = Mathf.Max(Mathf.Max(holeIsolated[i], holeInner[i] * 1.5f), groupRadiusRs);
                if (cbdOn && (i == cbdA || i == cbdB))
                    rMed = Mathf.Max(rMed, (Vector3.Distance(holePos[i], cbdC) + 1.05f * cbdROut) / Mathf.Max(rs, 1e-6f));
                holeMedium[i] = rMed;
            }
            else
                holeMedium[i] = holeIsolated[i];

            float diskVis = holeDiskVis[i];
            diskCut[i] = new Vector4(1f - diskVis, 0f, 0f, 0f);
            bool diskOn = diskVis > 0f;

            Vector3 p = holePos[i];
            positions[i] = new Vector4(p.x, p.y, p.z, 0);
            parameters[i] = new Vector4(rs, holeInner[i], holeDiskOuter[i], holeMedium[i]);
            Vector3 up = st.renderUp;
            diskNormals[i] = up;
            double genInt = System.Math.Floor(st.genPhase);
            rotationOffsets[i] = new Vector4(st.textureAngle, (float)(st.genPhase - genInt), (float)genInt, 0);
            FillHotSpots(i, bh, bp, st);
            spins[i] = new Vector4(bh.spin, bh.diskRotation, member ? 1f : 0f, diskOn ? 1f : 0f);

            float m = 0.5f * rs;
            float jMag = Mathf.Clamp(bh.spin, -0.998f, 0.998f) * m * m;
            spinJ[i] = new Vector4(up.x * jMag, up.y * jMag, up.z * jMag, m);
            float sClamped = Mathf.Clamp(bh.spin, -0.998f, 0.998f);
            spinAxis[i] = new Vector4(up.x, up.y, up.z, sClamped * sClamped * m * m * m);   // M a^2 = J^2 / M

            float peakT = bp.physicalDiskTemperature
                ? (float)PeakDiskTemperature(bh.mass, st.rIsco, st.eIsco, st.fluxNorm, bp.eddingtonRatio)
                : bp.diskPeakTemperature;
            holePeakT[i] = peakT;
            peakT *= Mathf.Lerp(1f, gExt, bp.diskDopplerColor);
            float brightness = bp.diskBrightness * Mathf.Pow(gExt, 4f * bp.diskDopplerBeaming);
            appearanceProperties1[i] = new Vector4(peakT, bp.diskDopplerColor, bp.diskDopplerBeaming, bp.diskRadialContrast);
            appearanceProperties2[i] = new Vector4(brightness, bp.diskISCOStress, bp.diskTurbulance, bp.diskHigherOrderFade);

            diskConst0[i] = new Vector4((float)st.rIsco, (float)st.life, (float)st.omegaIn, (float)st.drdtIsco);
            diskConst1[i] = new Vector4((float)st.eIsco, (float)st.lIsco, (float)st.fluxNorm, (float)st.rLow);

            if (rowOwner[i] != st || rowVersion[i] != st.version)
            {
                plungeLut.SetPixels(0, i, PlungeLutWidth, 1, st.plungeRow);
                rowOwner[i] = st;
                rowVersion[i] = st.version;
                lutDirty = true;
            }

            Color lin = bp.diskTint.linear;
            float tintLum = 0.2126f * lin.r + 0.7152f * lin.g + 0.0722f * lin.b;
            diskTints[i] = tintLum > 1e-4f
                ? new Vector4(lin.r / tintLum, lin.g / tintLum, lin.b / tintLum, bp.diskTintStrength)
                : new Vector4(1f, 1f, 1f, 0f);
        }
        if (lutDirty)
            plungeLut.Apply(false, false);

        FillCircumbinaryDisk(cbdA, cbdB);

        // the shared disk handing over to the remnant's own disk (after a merger)
        Vector4 cbdBlend = new Vector4(0f, -1f, 0f, 0f);
        if (anim != null && anim.phase == 1 && anim.blendCbd && survivorIndex >= 0)
            cbdBlend = new Vector4(1f - Mathf.SmoothStep(0f, 1f, HandoverT()), survivorIndex, holeRs[survivorIndex], 0f);

        float tanHalf = Mathf.Tan(0.5f * cam.fieldOfView * Mathf.Deg2Rad);
        Vector3 camPos = cam.transform.position;
        float coverage = 0f;
        for (int i = 0; i <= count; i++)
        {
            Vector3 cpos;
            float R;
            if (i < count) { cpos = holePos[i]; R = holeMedium[i] * holeRs[i]; }
            else if (cbdOuterWorld > 0f) { cpos = cbdCenter; R = cbdOuterWorld; }
            else break;
            float dist = Vector3.Distance(camPos, cpos);
            float c = (dist <= R)
                ? 1f
                : Mathf.Clamp01(Mathf.Pow(R / Mathf.Sqrt(dist * dist - R * R) / tanHalf, 2f));
            coverage = Mathf.Max(coverage, c);
        }
        Shader.SetGlobalFloat(BHLensCoverageId, coverage);

        float potential = 0f;
        for (int i = 0; i < count; i++)
        {
            float d = ((Vector3)positions[i]).magnitude;
            potential += parameters[i].x / Mathf.Max(d, parameters[i].x * 1.05f);
        }
        float gObs = 1f / Mathf.Sqrt(Mathf.Max(1f - potential, 0.05f));
        Shader.SetGlobalFloat(BHObserverGId, gObs);
        Shader.SetGlobalVector(BHObserverTintId, ObserverTint(gObs));

        Shader.SetKeyword(singleHoleKeyword, count == 1 && !debugForceMedium);

        Shader.SetGlobalVectorArray(BHSpinJId, spinJ);
        Shader.SetGlobalVectorArray(BHSpinAxisId, spinAxis);
        Shader.SetGlobalVectorArray(BHDiskCutId, diskCut);
        Shader.SetGlobalVector(BHCBD0Id, cbd0);
        Shader.SetGlobalVector(BHCBD1Id, cbd1);
        Shader.SetGlobalVector(BHCBD2Id, cbd2);
        Shader.SetGlobalVector(BHCBD3Id, cbd3);
        Shader.SetGlobalVector(BHCBD4Id, cbd4);
        Shader.SetGlobalVector(BHCBD5Id, cbd5);
        Shader.SetGlobalVector(BHCBD6Id, cbd6);
        Shader.SetGlobalVector(BHCBD7Id, cbd7);
        Shader.SetGlobalVector(BHCBD8Id, cbd8);
        Shader.SetGlobalVector(BHCBD9Id, cbd9);
        Shader.SetGlobalVector(BHCBD10Id, cbd10);
        Shader.SetGlobalVector(BHCBD11Id, cbd11);
        Shader.SetGlobalVector(BHCBDBlendId, cbdBlend);

        Shader.SetGlobalVectorArray(BHDiskTintId, diskTints);
        Shader.SetGlobalVectorArray(BHHotSpotsId, hotSpots);
        Shader.SetGlobalVectorArray(BHHotSpotBoundsId, hotSpotBounds);
        Shader.SetGlobalInt(BHCountId, count);
        Shader.SetGlobalVectorArray(BHPositionsId, positions);
        Shader.SetGlobalVectorArray(BHParamsId, parameters);
        Shader.SetGlobalVectorArray(BHDiskNormalsId, diskNormals);
        Shader.SetGlobalVectorArray(BHRotationOffsetsId, rotationOffsets);
        Shader.SetGlobalVectorArray(BHSpinsId, spins);
        Shader.SetGlobalVectorArray(BHAppearance1Id, appearanceProperties1);
        Shader.SetGlobalVectorArray(BHAppearance2Id, appearanceProperties2);
        Shader.SetGlobalVectorArray(BHDiskConst0Id, diskConst0);
        Shader.SetGlobalVectorArray(BHDiskConst1Id, diskConst1);
        Shader.SetGlobalFloat(BHNoiseMeanId, noiseMean);
        Shader.SetGlobalTexture(BHPlungeLUTId, plungeLut);

        Shader.SetGlobalTexture(BHSkyboxId, skybox);
        Shader.SetGlobalTexture(BHNoiseId, noiseTexture);
        if (noiseTexture != null)
            Shader.SetGlobalVector(BHNoiseSizeId, new Vector4(noiseTexture.width, noiseTexture.height, 0f, 0f));
        Shader.SetGlobalVector(BHSkyboxSizeId, new Vector4(skybox != null ? skybox.width : 1024f, 0f, 0f, 0f));
        Shader.SetGlobalTexture(KerrSnCnLUTId, kerrSnCnLUT);
        Shader.SetGlobalTexture(KerrKLUTId, kerrKLUT);
        Shader.SetGlobalTexture(KerrFLUTId, kerrFLUT);
    }
}