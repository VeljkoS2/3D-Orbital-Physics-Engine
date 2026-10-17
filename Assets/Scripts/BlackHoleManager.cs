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
    public float groupRadiusRs = 35f;
    [Tooltip("Debug: send single holes through the combined-field integrator too.")]
    public bool debugForceMedium = false;

    [Header("Merger")]
    [Tooltip("Hand-over (remnant spin builds up), in units of the remnant's mass G M / c^3 " +
             "(simulation time). A real ringdown lasts a few tens of M.")]
    public float mergerRingdownM = 100f;
    [Tooltip("Stretches the plunge and the hand-over in simulation time. 1 = physical.")]
    public float mergerTimeStretch = 1f;
    [Tooltip("The holes' disks are torn up and swallowed in the plunge, and the gas " +
             "the tides flung out falls back afterwards, re-forming the remnant's disk from its outer edge " +
             "inward (it reaches the hot inner disk last). How long that takes, in orbits at the outer edge " +
             "(simulation time, times Merger Time Stretch). 0 = at once.")]
    [Min(0f)] public float diskRefillOrbits = 10f;
    [Tooltip("A much lighter hole entering a big hole's disk loses its own disk on the way in: 0 = gone right at the " +
             "big disk's edge, 1 = only at its inner edge. In between: shrinks from the edge, gone at that fraction of " +
             "the way in (by depth: an eccentric orbit's deepest point counts, and what is lost stays lost).")]
    [Range(0f, 1f)] public float embeddedDiskStrip = 0.25f;

    [Header("Jets")]
    [Tooltip("Relativistic jets along each hole's spin axis: magnetic fields threading the spinning hole, wound up " +
             "by frame dragging, launch them from its rotation (Blandford-Znajek): power ~ spin^2. Analytic glow, " +
             "Doppler beamed (the approaching jet brighter, the receding one dimmer), lensed like the disk.")]
    public bool jets = true;
    [Tooltip("Physical switch: a jet is powered by accretion -- the infalling gas carries in the magnetic field " +
             "that launches it, so its power follows the accretion rate (~ Eddington ratio x spin^2): it fades as the " +
             "disk drains, fades as the disk is torn down toward the hole (below ~5 ISCO radii), is off without a " +
             "disk, and comes back after a merger once the re-forming disk reaches the hole. Brightness 1 is set " +
             "for an Eddington ratio of 0.1. Off: every spinning hole has its jets.")]
    public bool jetPhysicalSwitch = true;
    public Color jetColor = new Color(0.55f, 0.7f, 1f);
    [Min(0f)] public float jetBrightness = 1f;
    [Tooltip("Bulk Lorentz factor of the flow (how strongly it is beamed). Kpc-scale jets ~2-3, blazars ~10-20.")]
    [Range(1.01f, 20f)] public float jetLorentz = 3f;
    [Tooltip("Full opening angle, degrees.")]
    [Range(0.5f, 30f)] public float jetOpeningDeg = 6f;
    [Tooltip("Length of each jet, in Schwarzschild radii.")]
    [Min(10f)] public float jetLengthRs = 400f;
    [Tooltip("Where the jets start glowing, in Schwarzschild radii above the hole.")]
    [Min(1f)] public float jetBaseRs = 5f;
    static readonly int BHJetId = Shader.PropertyToID("_BHJet");
    static readonly int BHJetParamsId = Shader.PropertyToID("_BHJetParams");
    static readonly int BHJetColorId = Shader.PropertyToID("_BHJetColor");
    readonly Vector4[] jetData = new Vector4[MaxBlackHoles];

    [Header("Disk draining")]
    [Tooltip("With no new gas, a disk is swallowed by its hole. The gas flows in on the viscous time R^2 / nu " +
             "(nu = alpha c_s H) of a Shakura-Sunyaev thin disk at the disk's outer edge, worked out for the CURRENT " +
             "accretion rate. The angular momentum is carried off by a disk wind, so the disk shrinks as it drains " +
             "(keeping its surface-density profile, Sigma ~ R^-3/4: outer edge ~ M_disk^0.8), faster and faster, until " +
             "it is gone into the hole. The swallowed gas adds to the hole's mass (minus what it radiated). " +
             "The disk's mass is the one its accretion rate (Eddington ratio) implies. The simulation writes the " +
             "outer radius, Eddington ratio, mass and Accretion Disk into the parameters; change any of them and " +
             "it carries on from your values.")]
    public bool diskDraining = true;
    [Tooltip("Turbulent viscosity (Shakura-Sunyaev alpha). Typical 0.01 - 0.1; larger drains faster.")]
    [Range(0.001f, 1f)] public float drainAlpha = 0.1f;
    [Tooltip("Speeds the draining up (simulation time). 1 = physical.")]
    [Min(0f)] public float drainSpeed = 1f;

    [Header("Volumetric disk")]
    [Tooltip("Off: infinitely thin sheet. On: a disk with real thickness, marched with radiative transfer.")]
    public bool volumetricDisk = false;
    [Tooltip("1 = physical thickness from each hole's Eddington ratio, spin and ISCO stress " +
             "(radiation-pressure supported thin disk). Scales the radiation-pressure part only.")]
    [Min(0f)] public float volumetricThicknessScale = 1f;
    [Tooltip("Gas-pressure floor of H/r (takes over far out or at low Eddington ratio). " +
             "~0.001-0.005 for supermassive holes, ~0.01-0.03 for stellar-mass ones.")]
    [Range(0.0005f, 0.05f)] public float volumetricGasThickness = 0.003f;
    [Tooltip("Vertical optical depth through the disk outside the ISCO. Real disks: 1e3-1e5 (opaque). " +
             "Below ~10 the disk turns translucent (stylised). The plunging gas thins out on its own.")]
    [Min(0.1f)] public float volumetricOpticalDepth = 1000f;
    [Tooltip("Samples through the local disk thickness (face-on); inclined views use more, up to the budget.")]
    [Range(2, 24)] public int volumetricSteps = 8;
    [Tooltip("Sample budget per disk image (edge-on views use the most).")]
    [Range(8, 64)] public int volumetricMaxSteps = 32;

    [Header("Volumetric disk structure")]
    [Tooltip("DiskNoise3D.bytes: tileable 128^3 turbulence noise (built into a 3D texture at startup). " +
             "Without it the gas is smooth.")]
    public TextAsset diskNoise3D;
    [Tooltip("How clumpy the gas is (lognormal density contrast of the 3D turbulence). 0 = smooth.")]
    [Range(0f, 1.5f)] public float volumetricClumpiness = 0.8f;
    [Tooltip("Height scale of the turbulence: noise tiles per disk half-thickness. Smaller = taller structures.")]
    [Range(0.05f, 1f)] public float volumetricNoiseHeightScale = 0.3f;
    [Tooltip("Optical depth of the turbulent atmosphere above the dense disk (both sides together). " +
             "0 = none; ~0.5-3 = soft, wispy, see-through outer layers.")]
    [Range(0f, 10f)] public float atmosphereOpticalDepth = 1.5f;
    [Tooltip("Scale height of the atmosphere, in disk half-thicknesses (it extends ~5x this).")]
    [Range(0.1f, 1.5f)] public float atmosphereScaleHeight = 0.6f;
    [Tooltip("Brightness of the atmosphere relative to the disk surface.")]
    [Range(0f, 3f)] public float atmosphereBrightness = 1f;

    static readonly int BHVolume2Id = Shader.PropertyToID("_BHVolume2");
    static readonly int BHDiskNoise3DId = Shader.PropertyToID("_BHDiskNoise3D");
    Texture3D diskNoise3DTex;
    TextAsset diskNoise3DSource;

    // Builds the 3D noise texture from the raw bytes (x = azimuth fastest, then radius, then height).
    void EnsureDiskNoise3D()
    {
        if (diskNoise3D == diskNoise3DSource && (diskNoise3DTex != null || diskNoise3D == null))
            return;
        if (diskNoise3DTex != null)
        {
            if (Application.isPlaying) Destroy(diskNoise3DTex); else DestroyImmediate(diskNoise3DTex);
            diskNoise3DTex = null;
        }
        diskNoise3DSource = diskNoise3D;
        if (diskNoise3D == null)
            return;
        byte[] data = diskNoise3D.bytes;
        int n = Mathf.RoundToInt(Mathf.Pow(data.Length, 1f / 3f));
        if (n * n * n != data.Length)
        {
            Debug.LogWarning($"diskNoise3D: {data.Length} bytes is not a cube; expected 128^3 = 2097152.");
            return;
        }
        diskNoise3DTex = new Texture3D(n, n, n, TextureFormat.R8, true)
        {
            name = "BHDiskNoise3D",
            wrapMode = TextureWrapMode.Repeat,
            filterMode = FilterMode.Trilinear,
            anisoLevel = 0,
            hideFlags = HideFlags.DontSave
        };
        diskNoise3DTex.SetPixelData(data, 0);
        diskNoise3DTex.Apply(true, true);
    }

    static readonly int BHVolumeId = Shader.PropertyToID("_BHVolume");
    static readonly int BHDiskThickId = Shader.PropertyToID("_BHDiskThick");
    GlobalKeyword volumetricKeyword;
    readonly Vector4[] diskThick = new Vector4[DiskSlots];
    const double VolumeHCap = 0.3;   // VOL_H_CAP in BlackHoleKerrVolume.hlsl

    const int DiskSlots = MaxBlackHoles + 1;   // = the shader's disk arrays (the last slot is unused)

    readonly int[] groupRoot = new int[MaxBlackHoles];
    int FindRoot(int i) { while (groupRoot[i] != i) i = groupRoot[i]; return i; }

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
    static readonly int BHLensCoverageId = Shader.PropertyToID("_BHLensCoverage");
    static readonly int BHSkyboxSizeId = Shader.PropertyToID("_BHSkyboxSize");

    const float MinAnalyticRadiusRs = 10f;
    const float MaxAnalyticRadiusRs = 300f;
    const float TidalTruncation = 0.7f;
    [Tooltip("Companions lighter than this fraction of a hole's mass pass through its disk (nested, no gap); heavier ones clear a gap and the disks shrink.")]
    public float GapMassRatio = 1e-4f;   // lighter companions leave a disk alone (no gap: see pass A)

    // j is a light companion of i (below GapMassRatio of its mass): it neither clears a gap in i's disk nor needs
    // room in the renderer -- i's exact region and disk stay whole around it, and j's own small region sits inside
    // (nested: the shader traces j first wherever a ray reaches it, then i from where the ray leaves j)
    bool Light(int j, int i) => holeRs[j] < GapMassRatio * holeRs[i];

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
    readonly float[] holeGasOuter = new float[MaxBlackHoles];   // the gas's extent (Rs): tides / draining, not the drawing limit
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

        // disk outer edge, Unity units: as set in the inspector, and as cut by tides so far
        public float origOuterW, permOuterW;
        // the parameters' disk as last seen or written by the simulation (a different value there = the user
        // changed it: a new disk, the tidal cut and the draining start over from it)
        public float tideCapW = float.PositiveInfinity;
        // the disk as it was set up (Unity units): not cut by tides, not shrunk by draining. A merger's
        // remnant disk is made of both holes' gas: this is what it is sized from.
        public float createdOuterW;
        // draining: disk mass / starting mass, accretion rate / starting rate, starting rate (kg/s), viscous
        // time (s), disk mass (kg), outer edge (m) and Eddington ratio; reset = start over from the parameters
        public double drainM = 1.0, drainF = 1.0, drainMdot0, drainTnu0, drainMd0, drainR0W;
        public float drainEdd0;
        public bool drainReset = true;
        public float lastUserEdd = float.NaN;
        // this frame's accretion-rate factor for the renderer (brightness)
        public float drainMdot = 1f;
        public float lastUserOuterN = float.NaN;
        public bool lastUserDisk;

        // after a merger: the disk re-forming from the outside in
        public double refillAge = -1.0, refillT;   // sim s; age < 0: not re-forming
        public float refillOutW;                   // its outer edge, Unity units

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

    // ---- merger animation --------------------------------------------------------
    class MergeAnim
    {
        public HoleState survivor, ghost;
        public int phase;                  // 0 plunge, 1 hand-over
        public Vector3 offA, offB, lHat;   // Unity units from the centre of mass, orbital axis
        public float rsA, rsB, spinA, spinB, massKeep, spinF;
        public Vector3 upA, upB, upF;
        public double simT;        // simulation seconds since the merger
        public double plungeT;     // remaining inspiral time at the trigger (Peters)
        public double omega0;      // orbital angular velocity at the trigger (rad / sim s)
        public double phase1SimT;  // simT when the hand-over started
        public double handoverT;   // hand-over length (sim s)
        public float createdA, createdB;   // both disks as set up, captured at the merger (Unity units)
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
        volumetricKeyword = GlobalKeyword.Create("BH_VOLUMETRIC_DISK");
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
        if (diskNoise3DTex != null) Destroy(diskNoise3DTex);
    }

    HoleState GetState(GameObject hole)
    {
        if (!states.TryGetValue(hole, out var st))
        {
            var rng = new System.Random(hole.GetInstanceID());
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

    // H / r of the disk at r (Rs units): radiation-pressure supported thin disk with the
    // relativistic vertical gravity, plus a gas-pressure floor. Same formula as DiskAspectAt
    // in BlackHoleKerrVolume.hlsl.
    static double DiskAspect(double r, double rIsco, double ae, double H0, double c, double hFloor)
    {
        double rr = System.Math.Max(r, rIsco);                 // inside the ISCO: aspect ratio at the ISCO
        double J = System.Math.Max(1.0 - c * System.Math.Sqrt(rIsco / rr), 0.0);
        KerrIscoEL(ae, rr, out double E, out double L);        // circular-orbit E, L at rr
        double G = KerrM * rr / System.Math.Max(L * L + ae * ae * (1.0 - E * E), 1e-6);
        double hRad = H0 * J * G / rr;
        return System.Math.Min(System.Math.Sqrt(hRad * hRad + hFloor * hFloor), VolumeHCap);
    }

    // isotropic -> Boyer-Lindquist radius (Rs units), = IsoToBL in the shader

    // (H0 in Rs, gas floor, envelope = max H/r over the disk) for one hole's disk
    Vector4 DiskThickness(HoleState st, BlackHoleParamaters bh, BlackHoleAppearanceProperties bp)
    {
        double aPhys = System.Math.Clamp((double)bh.spin, -0.998, 0.998) * KerrM;
        double ae = (bh.diskRotation < 0f ? -1.0 : 1.0) * aPhys;
        double eta = System.Math.Max(1.0 - st.eIsco, 0.01);   // radiative efficiency
        double H0 = volumetricThicknessScale * 0.75 * System.Math.Max(bp.eddingtonRatio, 0f) / eta;
        double c = 1.0 - bp.diskISCOStress;
        double hFloor = volumetricGasThickness;
        double hMax = hFloor;
        for (int k = 0; k < 64; k++)   // rIsco .. 64 rIsco, log spaced (the peak is near 2.5 rIsco)
            hMax = System.Math.Max(hMax, DiskAspect(st.rIsco * System.Math.Pow(64.0, k / 63.0),
                                                    st.rIsco, ae, H0, c, hFloor));
        return new Vector4((float)H0, (float)hFloor, (float)System.Math.Min(hMax * 1.03, VolumeHCap), 0f);
    }

    // ---- Draining: a physical thin disk ---------------------------------------------------
    const double SigmaSB = 5.670374419e-8, KBoltz = 1.380649e-23, MProton = 1.67262192e-27, SigmaThomson = 6.6524587e-29;

    // Viscous time R^2 / nu (s) of a Shakura-Sunyaev disk at radius R (m) around mass M (kg), inner edge
    // Rin (m), accretion rate mdot (kg/s). The stress is alpha x the GAS pressure (nu = alpha c_gas^2 / Omega):
    // with the total pressure, a radiation-dominated disk is unstable (Lightman & Eardley 1974) -- the
    // fainter it got, the faster it would drain. Thickness from hydrostatic balance with gas + radiation
    // pressure; midplane temperature T_c^4 = (3 tau / 8 + 1) T_eff^4 (damped iteration); opacity = electron
    // scattering + Kramers. Bisection on the sound speed.
    static double ViscousTime(double M, double R, double Rin, double mdot, double alpha)
    {
        double G = Constants.G, c = Constants.C;
        double om = System.Math.Sqrt(G * M / (R * R * R));
        double vK = om * R;
        double fR = System.Math.Max(1.0 - System.Math.Sqrt(Rin / R), 1e-6);
        double teff4 = System.Math.Max(3.0 * G * M * mdot * fR / (8.0 * System.Math.PI * SigmaSB * R * R * R), 1e-30);
        double aRad = 4.0 * SigmaSB / c;
        double gasK = KBoltz / (0.62 * MProton);            // P_gas / (rho T)
        double nu = 0.0, T = 0.0, rho = 0.0;
        double lo = System.Math.Log(1e-9 * vK), hi = System.Math.Log(vK);
        double lT = 0.25 * System.Math.Log(teff4);   // (warm-started across the bisection)
        for (int it = 0; it <= 40; it++)
        {
            double mid = 0.5 * (lo + hi);
            double cs = System.Math.Exp(mid);
            double H = cs / om;
            int passes = (it == 0) ? 14 : 4;
            for (int k = 0; k < passes; k++)
            {
                T = System.Math.Exp(lT);
                nu = alpha * gasK * T / om;
                double sigma = mdot * fR / (3.0 * System.Math.PI * nu);
                rho = sigma / (2.0 * H);
                double kap = 0.034 + 5e20 * rho * System.Math.Pow(T, -3.5);
                double lTn = 0.25 * System.Math.Log((3.0 / 16.0 * kap * sigma + 1.0) * teff4);
                lT += 0.5 * (lTn - lT);
            }
            if (it == 40) break;   // (the last pass: the structure at the solution)
            double P = rho * gasK * T + aRad * T * T * T * T / 3.0;
            if (cs * cs < P / rho) lo = mid; else hi = mid;
        }
        return R * R / nu;
    }

    // One frame of draining (simulation time). The disk (mass m, a fraction of its starting mass) feeds the
    // hole at mdot = 1.5 M_disk / t_visc(R_out). A wind carries the angular momentum off, so the disk keeps its
    // profile (Sigma ~ R^-3/4, M(<R) ~ R^5/4) and shrinks: R_out = R0 m^0.8 -- t_visc falls with it, so it goes
    // faster and faster; at ~1.3 ISCO the rest is swallowed. The swallowed mass (x (1 - efficiency)) goes to the
    // hole. Writes outer radius, Eddington ratio, mass and Accretion Disk into the parameters.
    // The parameters as the user may have changed them: a value different from the last one seen / written by
    // the simulation is the user's change, and the simulation carries on from it -- a new disk (tidal cut and
    // draining start over, a disk still re-forming after a merger is done) or a new accretion rate.
    // Called before anything is written (the draining runs before the rendering's own check).
    void SyncUserEdits(HoleState st)
    {
        var ubh = st.props.blackHoleParamaters;
        if (ubh.diskOuterRadiusNormalized != st.lastUserOuterN || ubh.accretionDisk != st.lastUserDisk)
        {
            if (!float.IsNaN(st.lastUserOuterN))
                st.refillAge = -1.0;
            st.lastUserOuterN = ubh.diskOuterRadiusNormalized;
            st.lastUserDisk = ubh.accretionDisk;
            st.tideCapW = float.PositiveInfinity;
            st.drainReset = true;
            st.createdOuterW = ubh.accretionDisk ? ubh.diskOuterRadiusNormalized * ubh.schwarzschildRadiusUnits : 0f;
        }
        float uEdd = st.props.blackHoleAppearanceProperties.eddingtonRatio;
        if (uEdd != st.lastUserEdd)
        {
            st.lastUserEdd = uEdd;
            st.drainReset = true;
        }
    }

    void DrainStep(HoleState st, double dt)
    {
        SyncUserEdits(st);
        if (!diskDraining || drainSpeed <= 0f) return;
        var props = st.props;
        var bh = props.blackHoleParamaters;
        var bp = props.blackHoleAppearanceProperties;
        if (!bh.accretionDisk || bh.mass <= 0.0 || bh.schwarzschildRadiusWorld <= 0.0) return;
        double M = bh.mass, Rs = bh.schwarzschildRadiusWorld;
        double rIsco = System.Math.Max(st.rIsco, 1.0);
        double Rin = rIsco * Rs;
        double eta = System.Math.Max(1.0 - st.eIsco, 0.01);
        double alpha = drainAlpha;
        if (st.drainReset)
        {
            // from the parameters as they are now: rate from the Eddington ratio, L_Edd / (eta c^2)
            double lEdd = 4.0 * System.Math.PI * Constants.G * M * MProton * Constants.C / SigmaThomson;
            st.drainEdd0 = Mathf.Max(bp.eddingtonRatio, 0f);
            st.drainMdot0 = st.drainEdd0 * lEdd / (eta * Constants.C * Constants.C);
            st.drainR0W = System.Math.Max(bh.diskOuterRadiusNormalized, rIsco * 1.05) * Rs;
            st.drainTnu0 = st.drainMdot0 > 0.0 ? ViscousTime(M, st.drainR0W, Rin, st.drainMdot0, alpha) : 0.0;
            st.drainMd0 = st.drainMdot0 * st.drainTnu0 / 1.5;
            st.drainM = 1.0;
            st.drainF = 1.0;
            st.drainReset = false;
        }
        if (!(st.drainMdot0 > 0.0) || !(st.drainTnu0 > 0.0)) return;
        double rFloor = 1.05 * Rin;

        // the tides cut the disk (its outer edge in the parameters): the gas beyond is gone, and so is its mass
        // (spreading disk: R_out ~ m^0.8). The rate at the hole does not drop at once -- the inner disk is as it
        // was -- but the smaller disk drains on its own, shorter, viscous time: the jets and the light follow.
        double rCut = System.Math.Max(bh.diskOuterRadiusNormalized * Rs, rFloor);
        if (rCut < st.drainR0W * System.Math.Pow(st.drainM, 0.8) * (1.0 - 1e-6))
        {
            st.drainM = System.Math.Pow(rCut / st.drainR0W, 1.25);
            for (int it = 0; it < 3; it++)
                st.drainF = st.drainM * st.drainTnu0 / ViscousTime(M, rCut, Rin, st.drainMdot0 * st.drainF, alpha);
        }

        double mBefore = st.drainM;   // (what the tides cut is not swallowed)
        double t = dt * drainSpeed;
        for (int n = 0; n < 8 && t > 0.0 && st.drainM > 1e-9; n++)
        {
            double R = System.Math.Max(st.drainR0W * System.Math.Pow(st.drainM, 0.8), rFloor);
            double tnu = ViscousTime(M, R, Rin, st.drainMdot0 * st.drainF, alpha);
            double rate = 1.5 / tnu;                              // d ln m / dt
            double h = (n == 7) ? t : System.Math.Min(t, 0.05 / rate);   // <= 5 % of the mass per step
            st.drainM *= System.Math.Exp(-rate * h);
            t -= h;
            // the rate this mass gives: mdot = 1.5 M_disk / t_visc(R_out, mdot), i.e. f = m t_visc0 / t_visc(f)
            double R1 = System.Math.Max(st.drainR0W * System.Math.Pow(st.drainM, 0.8), rFloor);
            st.drainF = st.drainM * st.drainTnu0 / ViscousTime(M, R1, Rin, st.drainMdot0 * st.drainF, alpha);
        }
        double rOut = st.drainR0W * System.Math.Pow(st.drainM, 0.8);
        bool gone = rOut <= 1.3 * Rin || st.drainM < 1e-4;   // into the plunge region: the rest falls in
        if (gone)
            st.drainM = 0.0;

        // the hole gains what was swallowed, less what was radiated
        double dM = st.drainMd0 * (mBefore - st.drainM) * (1.0 - eta);
        if (dM > 0.0)
        {
            bh.mass += dM;
            bh.schwarzschildRadiusWorld = 2.0 * Constants.G * bh.mass / (Constants.C * Constants.C);
            bh.schwarzschildRadiusUnits = (float)(bh.schwarzschildRadiusWorld / GlobalProperties.scale);
            props.mass += dM;
            props.radius = bh.schwarzschildRadiusWorld * 0.9;
        }

        // write the disk: outer edge (never larger than what is there: the tides may have cut it more),
        // current Eddington ratio, gone = no disk
        if (gone)
        {
            bh.accretionDisk = false;
        }
        else
        {
            float nOut = (float)(System.Math.Max(rOut, rFloor) / bh.schwarzschildRadiusWorld);
            bh.diskOuterRadiusNormalized = Mathf.Min(bh.diskOuterRadiusNormalized, nOut);
            bh.escapeRadiusNormalized = bh.diskOuterRadiusNormalized * 2f;
            bp.eddingtonRatio = Mathf.Clamp(st.drainEdd0 * (float)st.drainF, 1e-9f, 1f);
        }
        props.blackHoleParamaters = bh;
        props.blackHoleAppearanceProperties = bp;
        st.lastUserOuterN = bh.diskOuterRadiusNormalized;
        st.lastUserDisk = bh.accretionDisk;
        st.lastUserEdd = bp.eddingtonRatio;
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
        foreach (var kv in states)
        {
            var rs0 = kv.Value;
            if (rs0.refillAge < 0.0) continue;
            rs0.refillAge += simDt;
            if (rs0.refillAge >= rs0.refillT) rs0.refillAge = -1.0;
        }
        foreach (var kv in states)
            if (kv.Value.refillAge < 0.0   // (re-forming after a merger: drains once formed)
                && !(anim != null && anim.phase == 0 && (kv.Value == anim.survivor || kv.Value == anim.ghost)))
                DrainStep(kv.Value, simDt);

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
    }

    // The straight view past a hole bends rays by a series (the shader's WeakKick) plus a near-camera term
    // for the camera's finite distance; inside the hole's region they are traced exactly. Where the two meet
    // (impact parameter = the region's radius) they differed by ~5e-5 rad -- invisible alone, but another
    // hole's lensing behind magnifies it into a visible seam. This scales the near-camera term so the series
    // equals the exact bend there (Schwarzschild, static observer at r0, Gauss-Legendre 32 in u = 1/r).
    // r0, R in Rs. 1 = unscaled (camera inside the region: no straight view past it).
    static double[] GL32x, GL32w;
    static void BuildGL32()
    {
        const int n = 32;
        var x = new double[n]; var w = new double[n];
        for (int i = 0; i < n; i++)
        {
            double z = System.Math.Cos(System.Math.PI * (i + 0.75) / (n + 0.5)), dp = 1.0;
            for (int it = 0; it < 100; it++)
            {
                double p0 = 1.0, p1 = 0.0;
                for (int j = 0; j < n; j++) { double p2 = p1; p1 = p0; p0 = ((2 * j + 1) * z * p1 - j * p2) / (j + 1); }
                dp = n * (z * p0 - p1) / (z * z - 1.0);
                double dz = p0 / dp; z -= dz;
                if (System.Math.Abs(dz) < 1e-15) break;
            }
            x[i] = 0.5 * (z + 1.0);                       // on [0, 1]
            w[i] = 1.0 / ((1.0 - z * z) * dp * dp);       // (2 / ((1 - z^2) P'^2)) / 2
        }
        GL32x = x; GL32w = w;
    }
    static float NearCameraScale(float r0f, float Rf)
    {
        double r0 = r0f, R = Rf;
        if (!(R >= 3.0) || !(r0 > R * 1.0001)) return 1f;
        if (GL32x == null) BuildGL32();
        double sa = R / r0, ca = System.Math.Sqrt(1.0 - sa * sa);
        double b = r0 * sa / System.Math.Sqrt(1.0 - 1.0 / r0);   // conserved impact parameter (static observer)
        double rp = b;                                           // periapsis: r^3 - b^2 r + b^2 = 0
        for (int it = 0; it < 60; it++)
        {
            double f = rp * rp * rp - b * b * rp + b * b, fp = 3.0 * rp * rp - b * b;
            double d = f / fp; rp -= d;
            if (System.Math.Abs(d) < 1e-14 * rp) break;
        }
        double up = 1.0 / rp, I = 0.0;
        for (int seg = 0; seg < 2; seg++)
        {
            double ua = seg == 0 ? 0.0 : 1.0 / r0;   // infinity -> periapsis, camera -> periapsis
            for (int n = 0; n < 32; n++)
            {
                double s = GL32x[n], u = up - (up - ua) * s * s;
                double P = 1.0 - b * b * u * u * (1.0 - u);
                if (P > 0.0) I += GL32w[n] * 2.0 * (up - ua) * s * b / System.Math.Sqrt(P);
            }
        }
        double bendExact = I - (System.Math.PI - System.Math.Asin(sa));
        double x = 1.0 / R;
        double C = x * (1.0 + x * (15.0 * System.Math.PI / 32.0 + x * (8.0 / 3.0 + x * (3465.0 * System.Math.PI / 2048.0))));
        double nc = 0.5 * (1.0 + ca) / (R * r0);
        double k = (C * (1.0 + ca) - bendExact) / nc;
        return (float)System.Math.Max(0.0, System.Math.Min(k, 3.0));
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
    //  phase 1 (hand-over): the remnant's spin builds up from 0; its disk re-forms from the
    //                      gas the tides flung out (diskRefillOrbits).
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
            upF = upF,
            createdA = GetState(keep).createdOuterW,
            createdB = GetState(gone).createdOuterW
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

        var s = anim.survivor;
        var bh = s.props.blackHoleParamaters;
        // (the tides may have torn both disks down to nothing on the way in -- Accretion Disk off -- but the gas
        // they flung out is still there: the remnant's disk forms from it whenever either hole had one)
        bool hadDisk = bh.accretionDisk || anim.createdA > 0f || anim.createdB > 0f;
        if (hadDisk)
        {
            if (!bh.accretionDisk)
            {
                bh.accretionDisk = true;
                s.props.blackHoleParamaters = bh;
                s.lastUserDisk = true;
            }
            // the new disk feeds at the rate the disks had before they drained / were torn
            var bpS = s.props.blackHoleAppearanceProperties;
            if (s.drainEdd0 > bpS.eddingtonRatio)
            {
                bpS.eddingtonRatio = s.drainEdd0;
                s.props.blackHoleAppearanceProperties = bpS;
                s.lastUserEdd = bpS.eddingtonRatio;
            }
            // Both disks were torn up and swallowed in the plunge; the gas the tides
            // flung out (both disks as created: same total area) falls back and forms the remnant's
            // disk again, from the outer edge inward.
            // (both disks as they were set up -- not as cut by the tides on the way in)
            float rOutW = Mathf.Sqrt(anim.createdA * anim.createdA + anim.createdB * anim.createdB);
            if (rOutW > 0f && bh.schwarzschildRadiusUnits > 0f)
            {
                bh.diskOuterRadiusNormalized = rOutW / bh.schwarzschildRadiusUnits;
                bh.escapeRadiusNormalized = bh.diskOuterRadiusNormalized * 2f;
                s.props.blackHoleParamaters = bh;
                s.origOuterW = s.permOuterW = rOutW;
                s.createdOuterW = rOutW;
                s.lastUserOuterN = bh.diskOuterRadiusNormalized;   // (the merger's, not an edit)
                s.lastUserDisk = bh.accretionDisk;
                s.tideCapW = float.PositiveInfinity;
                s.drainReset = true;   // a new disk: drains once it has formed
                // orbital period at the outer edge: 2 pi (Rs / c) sqrt(2 x^3), x = r / Rs
                double x = bh.diskOuterRadiusNormalized;
                double period = 2.0 * System.Math.PI * (double)bh.schwarzschildRadiusWorld / Constants.C
                              * System.Math.Sqrt(2.0 * x * x * x);
                s.refillT = diskRefillOrbits * period * mergerTimeStretch;
                s.refillOutW = rOutW;
                s.refillAge = s.refillT > 0.0 ? 0.0 : -1.0;
            }
            else
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
                // start in the orbit's plane, then tilt with the spin to its final direction
                float v = Mathf.SmoothStep(0f, 1f, HandoverT());
                st.renderSpin = anim.spinF * v;
                st.renderUp = Vector3.Slerp(anim.lHat, anim.upF, v).normalized;
            }
        }
        bh.spin = st.renderSpin;
        bh.schwarzschildRadiusUnits = st.renderRs;
        st.renderBh = bh;
    }

    void OnBeginCameraRendering(ScriptableRenderContext context, Camera cam)
    {
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

        // ---- pairs: merger mode (minimal spheres no longer fit) -------------------------
        for (int i = 0; i < count; i++)
            for (int j = i + 1; j < count; j++)
            {
                if (Light(i, j) || Light(j, i)) continue;   // (nested, not merged regions)
                float D = Vector3.Distance(holePos[i], holePos[j]);
                float sqI = Mathf.Sqrt(holeRs[i]), sqJ = Mathf.Sqrt(holeRs[j]);
                float splitI = D * sqI / (sqI + sqJ) / Mathf.Max(holeRs[i], 1e-6f);
                float splitJ = D * sqJ / (sqI + sqJ) / Mathf.Max(holeRs[j], 1e-6f);
                if (0.85f * Mathf.Min(splitI, splitJ) < 1.05f * KerrFloorRs)
                    holeMerger[i] = holeMerger[j] = true;
            }

        // ---- pass A: geometry of every hole's regions (Rs units) ------------------
        for (int i = 0; i < count; i++)
        {
            var st = packed[i];
            var bh = st.renderBh;
            float rs = holeRs[i];

            float split = float.PositiveInfinity;
            float tideSplit = float.PositiveInfinity;   // (only companions heavy enough to clear the gas)
            for (int j = 0; j < count; j++)
            {
                if (j == i) continue;
                float D = Vector3.Distance(holePos[i], holePos[j]);
                float di = D * Mathf.Sqrt(rs) / (Mathf.Sqrt(rs) + Mathf.Sqrt(holeRs[j]));
                if (!Light(j, i))   // (a light one sits inside this region: no room needed)
                    split = Mathf.Min(split, di / Mathf.Max(rs, 1e-6f));
                // a companion below ~1e-4 of the mass cannot open a gap in the disk (its Hill sphere is thinner
                // than the disk): it moves through the gas like a pebble and leaves it be
                if (holeRs[j] >= GapMassRatio * rs)
                    tideSplit = Mathf.Min(tideSplit, di / Mathf.Max(rs, 1e-6f));
            }
            holeSplit[i] = split;

            SyncUserEdits(st);
            var ubh = st.props.blackHoleParamaters;
            st.drainMdot = diskDraining ? (float)st.drainF : 1f;
            st.origOuterW = ubh.accretionDisk ? ubh.diskOuterRadiusNormalized * ubh.schwarzschildRadiusUnits : 0f;

            // a light hole inside a heavy one's disk cannot keep its own: its little disk is torn into the big one's
            // gas -- shrinking from the big disk's edge, gone at embeddedDiskStrip of the way to its inner edge, within
            // the gas's height; the deepest it has been counts (eccentric orbits: a close pass leaves it cut), and
            // what goes stays gone (the gas joins the big disk)
            for (int j = 0; j < count; j++)
            {
                if (j == i || !Light(i, j)) continue;
                var hs = packed[j];
                float hostOut = hs.permOuterW;   // the big disk's gas (world), as last set
                if (!(hostOut > 0f) || !hs.renderBh.accretionDisk) continue;
                Vector3 rel = holePos[i] - holePos[j];
                Vector3 nH = hs.renderUp;
                float z = Vector3.Dot(rel, nH);
                float rho = (rel - z * nH).magnitude;
                float hEnv = Mathf.Max(Mathf.Max(diskThick[j].z, diskThick[j].y), 0.02f);
                float hostIn = Mathf.Min((float)hs.rIsco * holeRs[j], 0.9f * hostOut);
                float depth = (hostOut - rho) / Mathf.Max(hostOut - hostIn, 1e-6f);
                float inR = embeddedDiskStrip <= 1e-4f ? (depth > 0f ? 1f : 0f)
                          : Mathf.SmoothStep(0f, 1f, depth / embeddedDiskStrip);
                float inZ = 1f - Mathf.SmoothStep(0f, 1f, Mathf.Abs(z) / Mathf.Max(hEnv * rho, 1e-6f) - 1f);
                float f = inR * inZ;
                if (f > 0f)
                    st.tideCapW = Mathf.Min(st.tideCapW, st.origOuterW * (1f - f));
            }

            // tides cut the disk; what they cut stays cut (swallowed / flung out): written into the parameters
            if (bh.accretionDisk && rs > 0f && tideSplit < float.PositiveInfinity)
                st.tideCapW = Mathf.Min(st.tideCapW, TidalTruncation * tideSplit * rs);
            st.permOuterW = Mathf.Min(st.origOuterW, st.tideCapW);
            if (ubh.accretionDisk && ubh.schwarzschildRadiusUnits > 0f && st.permOuterW < st.origOuterW - 1e-6f)
            {
                float newNorm = st.permOuterW / ubh.schwarzschildRadiusUnits;
                ubh.diskOuterRadiusNormalized = newNorm;
                ubh.escapeRadiusNormalized = newNorm * 2f;
                st.props.blackHoleParamaters = ubh;
                st.lastUserOuterN = newNorm;
            }
            float diskNorm = (bh.accretionDisk && rs > 0f) ? st.permOuterW / rs : 0f;
            holeGasOuter[i] = diskNorm;

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


        // ---- pass B: membership, redshift, disk fade, shader data -----------------
        bool lutDirty = false;
        // close groups: holes linked by the same region-overlap test as membership
        for (int i = 0; i < count; i++) groupRoot[i] = i;
        for (int i = 0; i < count; i++)
            for (int j = i + 1; j < count; j++)
                if (!Light(i, j) && !Light(j, i)
                    && Vector3.Distance(holePos[i], holePos[j]) < holeIsolated[i] * holeRs[i] + holeIsolated[j] * holeRs[j])
                {
                    int a = FindRoot(i), b = FindRoot(j);
                    if (a != b) groupRoot[Mathf.Max(a, b)] = Mathf.Min(a, b);
                }
        for (int i = 0; i < count; i++)
        {
            var st = packed[i];
            var bh = st.renderBh;
            var bp = st.props.blackHoleAppearanceProperties;
            float rs = holeRs[i];

            bool member = debugForceMedium || holeMerger[i];
            double S = 0.0;
            for (int j = 0; j < count; j++)
            {
                if (j == i) continue;
                float D = Vector3.Distance(holePos[i], holePos[j]);
                if (D < holeIsolated[i] * rs + holeIsolated[j] * holeRs[j] && !Light(i, j) && !Light(j, i))
                    member = true;
                S += 0.5 * holeRs[j] / Mathf.Max(D, holeRs[j]);
            }
            float gExt = (float)((1.0 - 0.5 * S) / (1.0 + 0.5 * S));

            if (member)
            {
                float rMed = Mathf.Max(Mathf.Max(holeIsolated[i], holeInner[i] * 1.5f), groupRadiusRs);
                holeMedium[i] = rMed;
            }
            else
                holeMedium[i] = holeIsolated[i];

            float diskVis = holeDiskVis[i];
            // re-forming after a merger: an inner edge (Rs) moving in from beyond the outer edge to 0,
            // ever faster (the inflow time falls with radius, ~r^1.5); the shaders fade the gas over
            // 0.5..1.1 of it
            float refillCutRs = 0f;
            if (st.refillAge >= 0.0 && st.refillT > 0.0 && rs > 0f)
            {
                double left = System.Math.Max(1.0 - st.refillAge / st.refillT, 0.0);
                refillCutRs = (float)(2.0 * st.refillOutW * System.Math.Pow(left, 2.0 / 3.0)) / rs;
            }
            diskCut[i] = new Vector4(1f - diskVis, refillCutRs, 0f, 0f);
            bool diskOn = diskVis > 0f;

            Vector3 p = holePos[i];
            positions[i] = new Vector4(p.x, p.y, p.z, 0);
            parameters[i] = new Vector4(rs, holeInner[i], holeDiskOuter[i], holeMedium[i]);
            Vector3 up = st.renderUp;
            diskNormals[i] = up;
            double genInt = System.Math.Floor(st.genPhase);
            rotationOffsets[i] = new Vector4(st.textureAngle, (float)(st.genPhase - genInt), (float)genInt, 0);
            FillHotSpots(i, bh, bp, st);
            spins[i] = new Vector4(bh.spin, bh.diskRotation, member ? FindRoot(i) + 1f : 0f, diskOn ? 1f : 0f);

            float m = 0.5f * rs;
            float jMag = Mathf.Clamp(bh.spin, -0.998f, 0.998f) * m * m;
            spinJ[i] = new Vector4(up.x * jMag, up.y * jMag, up.z * jMag, m);

            // (draining: the Eddington ratio is the current one; manual temperature T^4 and the light follow
            // the rate too)
            float peakT = bp.physicalDiskTemperature
                ? (float)PeakDiskTemperature(bh.mass, st.rIsco, st.eIsco, st.fluxNorm, bp.eddingtonRatio)
                : bp.diskPeakTemperature * Mathf.Pow(st.drainMdot, 0.25f);
            holePeakT[i] = peakT;
            peakT *= Mathf.Lerp(1f, gExt, bp.diskDopplerColor);
            float brightness = bp.diskBrightness * Mathf.Pow(gExt, 4f * bp.diskDopplerBeaming) * st.drainMdot;
            appearanceProperties1[i] = new Vector4(peakT, bp.diskDopplerColor, bp.diskDopplerBeaming, bp.diskRadialContrast);
            appearanceProperties2[i] = new Vector4(brightness, bp.diskISCOStress, bp.diskTurbulance, bp.diskHigherOrderFade);

            // jet strength: ~ spin^2; physical switch: needs accretion
            float jetS = 0f;
            if (jets)
            {
                float a = Mathf.Clamp(bh.spin, -0.998f, 0.998f);
                jetS = jetBrightness * a * a;
                if (jetPhysicalSwitch)
                {
                    // power ~ the accretion rate (the current Eddington ratio: drains with the disk);
                    // re-forming after a merger: on as the gas reaches the hole
                    float edd = bh.accretionDisk ? Mathf.Max(bp.eddingtonRatio, 0f) : 0f;
                    float refill = 1f;
                    if (st.refillAge >= 0.0 && st.refillT > 0.0)
                        refill = 1f - Mathf.SmoothStep(0f, 1f, (float)(1.0 - st.refillAge / st.refillT) / 0.15f);
                    // and with the disk itself: torn down toward the hole (tides, the inspiral), the inner
                    // disk that carries the field in goes -- fading out from ~5 ISCO radii to where it is gone
                    float rIscoJ = Mathf.Max((float)st.rIsco, 1e-3f);
                    float diskJ = Mathf.SmoothStep(0f, 1f, (holeGasOuter[i] - 1.3f * rIscoJ) / (4f * rIscoJ));
                    jetS *= edd / 0.1f * refill * diskJ;
                }
            }
            // y: the near-camera term's scale for this hole (shader, straight view past it)
            float rCam = rs > 0f ? Vector3.Distance(cam.transform.position, holePos[i]) / rs : 0f;
            jetData[i] = new Vector4(jetS, NearCameraScale(rCam, holeMedium[i]), 0f, 0f);

            diskConst0[i] = new Vector4((float)st.rIsco, (float)st.life, (float)st.omegaIn, (float)st.drdtIsco);
            diskConst1[i] = new Vector4((float)st.eIsco, (float)st.lIsco, (float)st.fluxNorm, (float)st.rLow);
            diskThick[i] = bh.accretionDisk ? DiskThickness(st, bh, bp) : Vector4.zero;

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

        float tanHalf = Mathf.Tan(0.5f * cam.fieldOfView * Mathf.Deg2Rad);
        Vector3 camPos = cam.transform.position;
        float coverage = 0f;
        for (int i = 0; i < count; i++)
        {
            Vector3 cpos = holePos[i];
            float R = holeMedium[i] * holeRs[i];
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
        Shader.SetKeyword(volumetricKeyword, volumetricDisk);
        EnsureDiskNoise3D();
        float clump = diskNoise3DTex != null ? volumetricClumpiness : 0f; // no texture: smooth gas
        Shader.SetGlobalVector(BHVolumeId, new Vector4(clump, volumetricOpticalDepth,
            volumetricSteps, Mathf.Max(volumetricMaxSteps, volumetricSteps)));
        Shader.SetGlobalVector(BHVolume2Id, new Vector4(atmosphereOpticalDepth, Mathf.Max(atmosphereScaleHeight, 0.05f),
            volumetricNoiseHeightScale, atmosphereBrightness));
        if (diskNoise3DTex != null)
            Shader.SetGlobalTexture(BHDiskNoise3DId, diskNoise3DTex);
        Shader.SetGlobalVectorArray(BHDiskThickId, diskThick);

        Shader.SetGlobalVectorArray(BHSpinJId, spinJ);
        Shader.SetGlobalVectorArray(BHDiskCutId, diskCut);

        Shader.SetGlobalVectorArray(BHDiskTintId, diskTints);
        Shader.SetGlobalVectorArray(BHHotSpotsId, hotSpots);
        Shader.SetGlobalVectorArray(BHHotSpotBoundsId, hotSpotBounds);
        Shader.SetGlobalInt(BHCountId, count);
        Shader.SetGlobalVectorArray(BHPositionsId, positions);
        Shader.SetGlobalVectorArray(BHParamsId, parameters);
        Shader.SetGlobalVectorArray(BHDiskNormalsId, diskNormals);
        for (int j = count; j < MaxBlackHoles; j++) jetData[j] = Vector4.zero;
        Shader.SetGlobalVectorArray(BHJetId, jetData);
        Shader.SetGlobalVector(BHJetParamsId, new Vector4(Mathf.Tan(0.5f * jetOpeningDeg * Mathf.Deg2Rad),
                                                          Mathf.Max(jetLorentz, 1.0001f), jetLengthRs, jetBaseRs));
        Color jl = jetColor.linear;
        Shader.SetGlobalVector(BHJetColorId, new Vector4(jl.r, jl.g, jl.b, 0f));
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
