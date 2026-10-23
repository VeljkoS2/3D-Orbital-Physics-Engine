// BlackHoleKerrAnalytic.hlsl
// Analytic Kerr renderer with proper radial turning-point handling
// and continuous Mino-time trajectory reconstruction.

#pragma use_dxc
#ifndef BLACKHOLE_KERR_ANALYTIC_INCLUDED
#define BLACKHOLE_KERR_ANALYTIC_INCLUDED
#define KERR_M           0.5
#define KERR_MAX_CROSSINGS 4
#define PI                  3.14159265359
#define TWO_PI              6.28318530718
// Lifetime of one generation of disk structure, in orbits of the disk's inner
// edge (Omega from spin, inner radius and Kepler factor). MUST match the value in
// BlackHoleGlobalManager.cs, which integrates the generation clock over time.
// Larger = longer, more wound spiral arcs; smaller = faster regeneration.
#define DISK_NOISE_LIFE_ORBITS 2.0
// The disk's inner edge is ALWAYS the ISCO of its orbit direction (stable circular
// orbits end there; standard thin-disk edge). Spin and orbit direction set it:
// 3 Rs at a = 0, 0.62 Rs prograde / 4.5 Rs retrograde at |a| -> M.
// 1: gas inside the ISCO keeps glowing while it plunges into the hole, as streams
//    that continue the disk's own pattern (see KerrPlungeStream).
#define KERR_DISK_PLUNGING 1
// Opacity of the plunging gas. Mass conservation: the same flux moving faster is
// thinner, tau ~ 1/(r |u^r|). In a real thin disk the gas enters the plunge with a
// tiny inflow speed (~1e-3 c), so the plunging region turns optically thin almost
// immediately -- which is why the lensed sky and the higher-order disk images next
// to the shadow stay visible through it.
//   KERR_ISCO_INFLOW : |u^r| (c) the opacity is normalised to at the ISCO. Kept
//                      separate from KERR_DISK_DRIFT, which is exaggerated so the
//                      disk's inward creep is visible; using it here made the
//                      plunging gas nearly opaque and hid everything behind it.
//   KERR_PLUNGE_DENSITY : extra thinning toward the horizon (1 = pure mass
//                      conservation, smaller = more transparent). Always 1 at the
//                      ISCO, so the edge stays seamless.
#define KERR_ISCO_INFLOW 0.002
#define KERR_PLUNGE_DENSITY 0.25
// Radial inflow speed |u^r| (in c) of the disk gas at its inner edge. The whole disk
// is always falling in: outside the ISCO slowly (viscous drift, scaled like the
// orbital speed, ~ r^-1/2), inside it as a free-fall plunge on top of that drift.
// The inflow moves the disk texture inward, enters the Doppler shift, and (mass
// conservation, Sigma ~ 1/(r |u^r|)) thins the gas where it speeds up.
// Larger = visibly faster creep of the outer disk and denser plunging gas.
#define KERR_DISK_DRIFT 0.02

// Hot spots: bright, hot clumps orbiting just outside the ISCO. Their positions and
// brightness are computed once per frame on the CPU (BlackHoleGlobalManager: count,
// strength, radius range); the shader only draws them.
#define DISK_HOTSPOTS_MAX 6                 // per hole, must match HotSpotsMax in C#
float4 _BHHotSpots[9 * DISK_HOTSPOTS_MAX]; // per spot: (orbit radius, angle, brightness, radial size)
float4 _BHHotSpotBounds[9]; // per hole: (rMin, rMax) of the ring where spots are

// set around one shading call: compute only the opacity (its colour is thrown away)
static bool g_bhOpacityOnly = false;
// Disk noise mip for the traces (sheet and volume). The reduced-res pass sets log2(downsample): what it
// samples is never re-shaded at full resolution (the volume, the sheet's higher-order images), so it must
// not alias; the full-res trace uses the same, as it fills the borders of what is rebuilt from it.
static float g_bhDiskLod = 0.0;
// > 0: the trace gathers its disk (gas, sheet) only, up to this Mino time -- a point on a straight piece
// through its region (the caller's estimate, smooth across the ray's closest approach) -- and keeps no end state
static float g_lamStop = 0.0;

// ------------------------------------------------------------------
// LUTs (baked by KerrEllipticLUTGenerator.cs -- keep sizes in sync)
// ------------------------------------------------------------------
#define KERR_LUT_X   1024.0            // modulus axis, x = log2(mc) / log2(1e-8)
#define KERR_LUT_S   1024.0            // quarter-period phase axis, s in [0, 0.25]
#define KERR_LUT_FX  512.0             // F LUT modulus axis
#define KERR_LUT_FT  256.0             // F LUT angle axis
#define KERR_MC_MIN_LOG2 (-26.5754247591) // log2(1e-8)

Texture2D<float2> _KerrSnCnLUT; // (sn, cn) at u = 4K s
Texture2D<float> _KerrKLUT; // K(m)
Texture2D<float> _KerrFLUT; // F(psi|m) / 4K, psi <= psi*
SamplerState sampler_linear_clamp;

// Texel i sits exactly at coordinate i/(N-1).
float LutCoord(float c, float n)
{
    return (saturate(c) * (n - 1.0) + 0.5) / n;
}

// Everything the LUTs need about one modulus, computed once per motion.
struct EllipticParams
{
    float mc; // 1 - m
    float m;
    float x; // LUT modulus coordinate
    float sqrtMc;
    float qInv; // mc^(-1/4)
    float ysInv; // 1 / asinh(mc^(-1/4))
};

EllipticParams MakeEllipticParams(float mc)
{
    EllipticParams ep;
    ep.mc = clamp(mc, 1e-8, 1.0);
    ep.m = 1.0 - ep.mc;
    float l2 = log2(ep.mc);
    ep.x = l2 / KERR_MC_MIN_LOG2;
    ep.sqrtMc = sqrt(ep.mc);
    ep.qInv = exp2(-0.25 * l2);
    float ys = log(ep.qInv + sqrt(1.0 + ep.qInv * ep.qInv));
    ep.ysInv = 1.0 / max(ys, 1e-6);
    return ep;
}

// sn, cn at phase s in [0, 0.25]
float2 LutSnCnQuarter(EllipticParams ep, float s)
{
    float2 uv = float2(LutCoord(ep.x, KERR_LUT_X), LutCoord(s * 4.0, KERR_LUT_S));
    return _KerrSnCnLUT.SampleLevel(sampler_linear_clamp, uv, 0);
}

// sn, cn at phase s in [0, 0.5]:  sn(2K-u) = sn(u), cn(2K-u) = -cn(u)
float2 LutSnCnHalf(EllipticParams ep, float s)
{
    bool upper = (s > 0.25);
    float2 v = LutSnCnQuarter(ep, upper ? 0.5 - s : s);
    return upper ? float2(v.x, -v.y) : v;
}

float LutK(EllipticParams ep)
{
    return _KerrKLUT.SampleLevel(sampler_linear_clamp, float2(LutCoord(ep.x, KERR_LUT_X), 0.5), 0);
}

// F(psi|m) / (4K) for psi in [0, pi], given cos(psi) (signed) and sin(psi) >= 0.
// psi > pi/2 folds through F(pi - psi) = 2K - F(psi); psi > psi* folds through
// F(psi) + F(chi) = K with sqrt(mc) tan(psi) tan(chi) = 1.
float ForwardPhaseCS(EllipticParams ep, float cosPsi, float sinPsi)
{
    bool fold = (cosPsi < 0.0);
    float c = abs(cosPsi);
    float s = max(sinPsi, 0.0);
    bool comp = (s > c * ep.qInv);
    if (comp)
    {
        float ca = c;
        float cb = s * ep.sqrtMc;
        float n = max(sqrt(ca * ca + cb * cb), 1e-30);
        s = ca / n;
        c = cb / n;
    }
    float t = s / max(c, 1e-30);
    float y = log(t + sqrt(1.0 + t * t)); // asinh(tan psi)
    float v = _KerrFLUT.SampleLevel(sampler_linear_clamp,
        float2(LutCoord(ep.x, KERR_LUT_FX), LutCoord(y * ep.ysInv, KERR_LUT_FT)), 0);
    float ph = comp ? 0.25 - v : v;
    return fold ? 0.5 - ph : ph;
}

// Convenience wrappers (debug modes)
float SampleCompleteK(float m)
{
    return LutK(MakeEllipticParams(1.0 - saturate(m)));
}
float SampleForwardPhase(float m, float psi)
{
    float s, c;
    sincos(clamp(psi, 0.0, PI), s, c);
    return ForwardPhaseCS(MakeEllipticParams(1.0 - saturate(m)), c, s);
}

// ------------------------------------------------------------------
// Quadrature tables
// ------------------------------------------------------------------
static const float GL6_NODES[6] =
{
    -0.93246951420315203f, -0.66120938646626451f, -0.23861918608319691f,
     0.23861918608319691f, 0.66120938646626451f, 0.93246951420315203f
};
static const float GL6_WEIGHTS[6] =
{
    0.17132449237917034f, 0.36076157304813861f, 0.46791393457269104f,
    0.46791393457269104f, 0.36076157304813861f, 0.17132449237917034f
};
static const float GL8_NODES[8] =
{
    -0.96028985649753623f, -0.79666647741362674f, -0.52553240991632899f, -0.18343464249564980f,
     0.18343464249564980f, 0.52553240991632899f, 0.79666647741362674f, 0.96028985649753623f
};
static const float GL8_WEIGHTS[8] =
{
    0.10122853629037626f, 0.22238103445337447f, 0.31370664587788729f, 0.36268378337836198f,
    0.36268378337836198f, 0.31370664587788729f, 0.22238103445337447f, 0.10122853629037626f
};

#define GL32_N 32
static const float GL32_NODES[32] =
{
    -0.99726386184948157f, -0.98561151154526838f, -0.96476225558750639f, -0.93490607593773967f,
    -0.89632115576605209f, -0.84936761373256997f, -0.79448379596794239f, -0.73218211874028971f,
    -0.66304426693021523f, -0.58771575724076230f, -0.50689990893222936f, -0.42135127613063533f,
    -0.33186860228212767f, -0.23928736225213706f, -0.14447196158279649f, -0.04830766568773832f,
     0.04830766568773832f, 0.14447196158279649f, 0.23928736225213706f, 0.33186860228212767f,
     0.42135127613063533f, 0.50689990893222936f, 0.58771575724076230f, 0.66304426693021523f,
     0.73218211874028971f, 0.79448379596794239f, 0.84936761373256997f, 0.89632115576605209f,
     0.93490607593773967f, 0.96476225558750639f, 0.98561151154526838f, 0.99726386184948157f
};
static const float GL32_WEIGHTS[32] =
{
    0.00701861000947051f, 0.01627439473090574f, 0.02539206530926202f, 0.03427386291302176f,
    0.04283589802222684f, 0.05099805926237609f, 0.05868409347853557f, 0.06582222277636168f,
    0.07234579410884834f, 0.07819389578707023f, 0.08331192422694671f, 0.08765209300440378f,
    0.09117387869576378f, 0.09384439908080451f, 0.09563872007927471f, 0.09654008851472766f,
    0.09654008851472766f, 0.09563872007927471f, 0.09384439908080451f, 0.09117387869576378f,
    0.08765209300440378f, 0.08331192422694671f, 0.07819389578707023f, 0.07234579410884834f,
    0.06582222277636168f, 0.05868409347853557f, 0.05099805926237609f, 0.04283589802222684f,
    0.03427386291302176f, 0.02539206530926202f, 0.01627439473090574f, 0.00701861000947051f
};

// ------------------------------------------------------------------
// Metric helpers
// ------------------------------------------------------------------
float KerrDelta(float r, float a)
{
    return r * r - 2.0 * KERR_M * r + a * a;
}

float KerrAkerr(float r, float sinTheta, float a)
{
    float rr_aa = r * r + a * a;
    return rr_aa * rr_aa - a * a * KerrDelta(r, a) * sinTheta * sinTheta;
}

void BuildSpinBasis(float3 spinAxis, out float3 basisX, out float3 basisY, out float3 basisZ)
{
    basisZ = normalize(spinAxis);
    
    float sign = (basisZ.z < 0.0) ? -1.0 : 1.0;
    float a = -1.0 / (sign + basisZ.z);
    float b = basisZ.x * basisZ.y * a;

    basisX = float3(1.0 + sign * basisZ.x * basisZ.x * a, sign * b, -sign * basisZ.x);
    basisY = float3(b, sign + basisZ.y * basisZ.y * a, -basisZ.y);
}

// ------------------------------------------------------------------
// Stable Cartesian to Boyer-Lindquist / Oblate Spheroidal
// ------------------------------------------------------------------
void CartesianToOblate(float3 pos, float a,
                        out float r, out float cosTheta, out float sinTheta, out float phi)
{
    float x = pos.x, y = pos.y, z = pos.z;
    float rho2 = x * x + y * y;
    float p = rho2 + z * z - a * a;
    
    float disc = sqrt(max(p * p + 4.0 * a * a * z * z, 0.0));
    float rr = (p >= 0.0) ? 0.5 * (p + disc) : (2.0 * a * a * z * z) / max(disc - p, 1e-12);
    
    r = sqrt(max(rr, 1e-12));
    cosTheta = clamp(z / max(r, 1e-12), -0.999999f, 0.999999f);
    sinTheta = sqrt(max(1.0 - cosTheta * cosTheta, 0.0));
    phi = (rho2 > 1e-20) ? atan2(y, x) : 0.0;
}

void OblateTetradRegular(float3 pos, float a,
                         out float3 e_r, out float3 e_theta, out float3 e_phi,
                         out float r, out float cosTheta, out float sinTheta, out float phi)
{
    CartesianToOblate(pos, a, r, cosTheta, sinTheta, phi);

    float rho = sqrt(r * r + a * a);
    float sigma = sqrt(max(r * r + a * a * cosTheta * cosTheta, 1e-12));

    float rho2 = pos.x * pos.x + pos.y * pos.y;
    float inv_xy = (rho2 > 1e-20) ? rsqrt(rho2) : 0.0;
    float cp = (rho2 > 1e-20) ? pos.x * inv_xy : 1.0;
    float sp = (rho2 > 1e-20) ? pos.y * inv_xy : 0.0;

    e_r = float3(r * sinTheta * cp, r * sinTheta * sp, rho * cosTheta) / sigma;
    e_theta = float3(rho * cosTheta * cp, rho * cosTheta * sp, -r * sinTheta) / sigma;
    e_phi = float3(-sp, cp, 0.0);
}

// ------------------------------------------------------------------
// Conserved quantities
// ------------------------------------------------------------------
void LocalDirToConstantsRegular(
    float3 pos, float3 rayDir, float a, bool physicalDir,
    out float xi, out float eta, out float E,
    out float r, out float cosTheta, out float sinTheta, out float phi,
    out float n_r, out float n_theta, out float n_phi)
{
    float3 e_r, e_theta, e_phi;
    OblateTetradRegular(pos, a, e_r, e_theta, e_phi, r, cosTheta, sinTheta, phi);

    n_r = dot(rayDir, e_r);
    n_theta = dot(rayDir, e_theta);
    n_phi = dot(rayDir, e_phi);
    float nLen = max(length(float3(n_r, n_theta, n_phi)), 1e-12);
    n_r /= nLen;
    n_theta /= nLen;
    n_phi /= nLen;

    float Sigma = r * r + a * a * cosTheta * cosTheta;
    float Akerr = KerrAkerr(r, sinTheta, a);
    float Delta = KerrDelta(r, a);

    // Camera pixels: the pixel direction is mapped to the local (ZAMO) direction with
    // these weights. physicalDir: the direction already IS the local direction (a ray
    // handed over from another hole's trace, see KerrExitState) -- no weighting.
    [branch]
    if (!physicalDir)
    {
        float rho_emb = sqrt(r * r + a * a);
        float pr_w = n_r * rho_emb / sqrt(max(Delta, 1e-8));
        float pth_w = n_theta;
        float pph_w = n_phi * sqrt(max(Akerr, 0.0)) / (rho_emb * sqrt(max(Sigma, 1e-8)));
        float wLen = max(length(float3(pr_w, pth_w, pph_w)), 1e-12);
        n_r = pr_w / wLen;
        n_theta = pth_w / wLen;
        n_phi = pph_w / wLen;
    }

    float akerrOverSigma = Akerr / max(Sigma, 1e-12);
    float sqrtAkerrOverSigma = sqrt(max(akerrOverSigma, 0.0));

    float alpha = sqrt(max(Delta * Sigma / max(Akerr, 1e-12), 0.0));
    float omega = 2.0 * KERR_M * a * r / max(Akerr, 1e-12);

    float L = n_phi * sinTheta * sqrtAkerrOverSigma;
    float E_raw = alpha + omega * L;

    E = (abs(E_raw) < 1e-6) ? ((E_raw >= 0.0) ? 1e-6 : -1e-6) : E_raw;
    xi = L / E;
    
        // A ray passing exactly over the spin axis (xi = 0) has no polar azimuth term, so it
    // reflects at the pole instead of crossing it (tested: 261 px error; a 1-pixel line
    // along the projected spin axis). Any tiny xi gives the correct limit.
    if (abs(xi) < 1e-6)
        xi = (xi >= 0.0) ? 1e-6 : -1e-6;

    float L2_over_sin2 = n_phi * n_phi * akerrOverSigma;
    float p_theta = n_theta * sqrt(Sigma);
    float Q = p_theta * p_theta + cosTheta * cosTheta * (L2_over_sin2 - a * a * E * E);

    eta = Q / (E * E);
}

// ------------------------------------------------------------------
// Root solvers
// ------------------------------------------------------------------
// Largest root of the Ferrari resolvent  m^3 + p m^2 + (p^2/4 - s) m - q^2/8 = 0
// for the depressed quartic r^4 + p r^2 + q r + s.
// Closed form, but numerically stable in float32:
//  * depressed-cubic coefficients built directly from (p, q, s):
//      P = -p^2/12 - s,   Q = -p^3/108 + p s/3 - q^2/8
//    (going through the monic cubic's b, c, d first cancels badly)
//  * Cardano in the Numerical-Recipes form: one cube root of |Q|/2 + sqrt(D),
//    the second term as -P/(3A) -- no  -Q/2 + sqrt(D)  subtraction.
// That subtraction was what failed for xi ~ a, small eta at high spin.
// Verified in float32 vs float64 roots: 0 wrong real-root counts over 20000
// random (a, xi, eta), root error p99 2e-7, and no Newton polish needed.
float SolveFerrariResolvent(float p, float q, float s)
{
    float P = -p * p / 12.0 - s;
    float Q = -p * p * p / 108.0 + p * s / 3.0 - 0.125 * q * q;
    float disc = (P * P * P) / 27.0 + 0.25 * Q * Q;

    float t;
    [branch]
    if (disc > 0.0)
    {
        float A = ((Q >= 0.0) ? -1.0 : 1.0) * pow(0.5 * abs(Q) + sqrt(disc), 1.0 / 3.0);
        t = A - P / (3.0 * A);
    }
    else
    {
        float rr = sqrt(max(-P / 3.0, 0.0));
        float cosTh = clamp(-0.5 * Q / max(rr * rr * rr, 1e-30), -1.0, 1.0);
        t = 2.0 * rr * cos(acos(cosTh) / 3.0);
    }
    return t - p / 3.0;
}

// Real roots of r^4 + p r^2 + q r + s, sorted descending into a float4.
// Missing roots are -1e30 (they sort to the end). No arrays, no dynamic
// indexing (the old bubble sort on float[4] forces register-indexed/scratch
// access on most GPUs): a 5-comparator min/max network instead.
void SolveDepressedQuarticReal(float p, float q, float s, out float4 roots, out int numReal)
{
    const float NONE = -1e30;
    roots = float4(NONE, NONE, NONE, NONE);
    numReal = 0;

    [branch]
    if (abs(q) < 1e-10)
    {
        // biquadratic
        float disc = p * p - 4.0 * s;
        if (disc < 0.0)
            return;
        float sq = sqrt(disc);
        float y1 = 0.5 * (-p + sq); // y1 >= y2
        float y2 = 0.5 * (-p - sq);
        if (y1 < 0.0)
            return;
        float a1 = sqrt(y1);
        if (y2 >= 0.0)
        {
            float a2 = sqrt(y2);
            roots = float4(a1, a2, -a2, -a1);
            numReal = 4;
        }
        else
        {
            roots = float4(a1, -a1, NONE, NONE);
            numReal = 2;
        }
        return;
    }

    float m0 = SolveFerrariResolvent(p, q, s);
    if (m0 <= 1e-12)
        return;

    float s2 = sqrt(2.0 * m0);
    float hq = q / (2.0 * s2);
    float discA = s2 * s2 - 4.0 * (0.5 * p + m0 - hq);
    float discB = s2 * s2 - 4.0 * (0.5 * p + m0 + hq);

    float4 v = float4(NONE, NONE, NONE, NONE);
    if (discA >= 0.0)
    {
        float d = sqrt(discA);
        v.x = 0.5 * (-s2 + d);
        v.y = 0.5 * (-s2 - d);
        numReal += 2;
    }
    if (discB >= 0.0)
    {
        float d = sqrt(discB);
        v.z = 0.5 * (s2 + d);
        v.w = 0.5 * (s2 - d);
        numReal += 2;
    }

    // v.x >= v.y and v.z >= v.w already; merge the two sorted pairs
    float top = max(v.x, v.z);
    float bot = min(v.y, v.w);
    float m1 = min(v.x, v.z);
    float m2 = max(v.y, v.w);
    roots = float4(top, max(m1, m2), min(m1, m2), bot);
}

float SolveMuMaxSq(float a, float xi, float eta, out bool ordinary)
{
    ordinary = (eta >= 0.0);
    float coefB = a * a - xi * xi - eta;
    float disc = max(coefB * coefB + 4.0 * a * a * eta, 0.0);
    float denom = coefB - sqrt(disc);
    float uPlus = (abs(denom) > 1e-12) ? (-2.0 * eta / denom) : 0.0;
    return saturate(uPlus);
}

// ------------------------------------------------------------------
// Theta motion
// ------------------------------------------------------------------
struct ThetaMotion
{
    float uPlus;
    float uMinus;
    float mTheta;
    float K_theta;
    float s0;
    float kTheta;
    float hemiSign;
    bool isVortical;
    bool isNearEquatorial;
    float cPol; // sqrt(1 - u+), computed without cancellation
    float chiC0; // pi/2 - (polar-phi substitution angle) at the camera, computed directly (see PolarG)
    EllipticParams ep;
};

ThetaMotion SolveThetaMotion(float a, float xi, float eta,
                           float cosThetaStart, float n_theta, float r, float E)
{
    ThetaMotion tm;
    bool ordinary;
    float uPlusRaw = SolveMuMaxSq(a, xi, eta, ordinary);

    const float EQUATORIAL_UPLUS_THRESHOLD = 1e-4;
    tm.isNearEquatorial = ordinary && (uPlusRaw < EQUATORIAL_UPLUS_THRESHOLD);

    float uPlus = max(uPlusRaw, 1e-8f);
    tm.uPlus = uPlus;
    tm.isVortical = !ordinary;
    tm.hemiSign = (cosThetaStart >= 0.0f) ? 1.0f : -1.0f;
    float a2 = max(a * a, 1e-8f);

    float cosPsi, sinPsi;
    tm.cPol = 1.0;
    tm.chiC0 = 0.5 * PI;
    if (ordinary)
    {
        // 1 - u+ : X^2 - Y^2 = -4 eta xi^2  ->  X - Y = -4 eta xi^2 / (X + Y)
        float cB = a * a - xi * xi - eta;
        float Yq = sqrt(max(cB * cB + 4.0 * a * a * eta, 0.0));
        float Xq = a * a - xi * xi + eta;
        float omu = (uPlusRaw < 0.5) ? 1.0 - uPlusRaw
                  : ((Xq >= 0.0) ? 4.0 * eta * xi * xi / max((Xq + Yq) * (Yq - cB), 1e-30)
                                 : (Yq - Xq) / max(Yq - cB, 1e-30));
        tm.cPol = sqrt(max(omu, 1e-20));

        tm.uMinus = -eta / (a2 * uPlus);
        // k^2 = (a^2 u+^2 + eta) / u+ = a^2 u+ + eta / u+, with eta / u+ = (Yq - cB) / 2 taken as it is -- not through
        // u+, which a ray in the hole's equatorial plane drives below the 1e-8 floor: k came out ~0, its theta phase
        // stood still and its polar azimuth was off by ~1e-3 rad (a camera in a disk's plane -- seeing it edge-on --
        // got a line of wrong pixels along the disk plane through the hole)
        float etaOverU = (cB <= 0.0) ? 0.5 * (Yq - cB) : 2.0 * a * a * eta / max(Yq + cB, 1e-30);
        float a2u = a2 * uPlusRaw;
        float k2 = max(a2u + etaOverU, 1e-10f);
        float mT = saturate(a2u / k2);
        float mcT = saturate(etaOverU / k2);   // (1 - m directly: no cancellation)
        tm.ep = MakeEllipticParams(mcT);
        tm.mTheta = tm.ep.m;
        tm.kTheta = sqrt(k2);
        tm.K_theta = max(LutK(tm.ep), 1e-5f);

        // cos(psi) = cos(theta) / sqrt(u+) is exact near the equator; sin(psi) from Theta (no cancellation) near
        // the turning point. Each taken where it is the smaller one, the other from it: sqrt(1 - sin^2) right at
        // the equator is sqrt of a float rounding (0 or 2.4e-4 instead of ~0) -- a camera in a hole's disk plane
        // (orbits in its plane: common) got ~2e-4 rad of noise in every exit direction, i.e. sparkle / flicker.
        float cosD = clamp(cosThetaStart * rsqrt(uPlus), -1.0f, 1.0f);
        if (uPlusRaw > 1e-6f && cosD * cosD > 0.5f)
        {
            float u = cosThetaStart * cosThetaStart;
            float Sigma = r * r + a2 * u;
            float Theta_direct = pow(n_theta * sqrt(max(Sigma, 1e-12f)) / max(abs(E), 1e-9f), 2.0f);
            float gap = Theta_direct * (1.0f - u) / (a2 * max(u - tm.uMinus, 1e-12f));
            float sin2_psi = saturate(gap / uPlus);
            sinPsi = sqrt(sin2_psi);
            cosPsi = sqrt(1.0 - sin2_psi) * ((cosThetaStart >= 0.0f) ? 1.0 : -1.0);
        }
        else
        {
            cosPsi = cosD;
            sinPsi = sqrt(max(1.0f - cosPsi * cosPsi, 0.0f));
        }

        float sMag = ForwardPhaseCS(tm.ep, cosPsi, sinPsi);
        tm.s0 = (n_theta >= 0.0f) ? sMag : -sMag;
        // folded (sn, cn) at s0 are (sin psi, |cos psi|); pi/2 - chi = atan2(c |cos psi|, sin psi), not pi/2 minus
        // chi (see PolarG)
        tm.chiC0 = atan2(tm.cPol * abs(cosPsi), sinPsi);
    }
    else
    {
        float beta = a2 - eta - xi * xi;
        float delta = max(beta * beta + 4.0f * a2 * eta, 0.0f);
        float uMinus = max((beta - sqrt(delta)) / (2.0f * a2), 1e-8f);
        tm.uMinus = uMinus;
        tm.ep = MakeEllipticParams(saturate(uMinus / max(uPlus, 1e-8f))); // mc = u-/u+
        tm.mTheta = tm.ep.m;
        tm.kTheta = sqrt(max(a2 * uPlus, 1e-10f));
        tm.K_theta = max(LutK(tm.ep), 1e-5f);
        float cos2 = max(cosThetaStart * cosThetaStart, 1e-8f);
        float sin2_psi = saturate((1.0f - uMinus / cos2) / max(tm.mTheta, 1e-8f));
        sinPsi = sqrt(sin2_psi);
        cosPsi = sqrt(1.0 - sin2_psi);
        float sMag = ForwardPhaseCS(tm.ep, cosPsi, sinPsi);
        tm.s0 = (cosThetaStart * n_theta <= 0.0f) ? sMag : -sMag;
    }
    return tm;
}

float EvaluateCosTheta(ThetaMotion tm, float s)
{
    float K_safe = max(tm.K_theta, 1e-5f);
    float u = tm.s0 + s * tm.kTheta / (4.0f * K_safe);

    float u01 = frac(u);
    float u_half = frac(2.0f * u01);
    float fold = 1.0f - abs(2.0f * u_half - 1.0f);
    float arg = fold * 0.25f;

    float2 sncn = LutSnCnQuarter(tm.ep, arg);

    if (!tm.isVortical)
    {
        float signRestore = (u01 >= 0.25f && u01 < 0.75f) ? -1.0f : 1.0f;
        float maxCos = sqrt(saturate(tm.uPlus));
        return clamp(maxCos * sncn.y * signRestore, -maxCos, maxCos);
    }
    else
    {
        float dn2 = max(1.0f - tm.mTheta * sncn.x * sncn.x, 1e-8f);
        float cos2 = tm.uMinus / dn2;
        float cosVal = sqrt(clamp(cos2, 0.0f, tm.uPlus));
        return clamp(cosVal * tm.hemiSign, -0.999999f, 0.999999f);
    }
}

// ------------------------------------------------------------------
// Radial motion in Kerr Spacetime
// ------------------------------------------------------------------
struct RadialMotion
{
    float r1, r2, r3, r4;
    float m_r;
    float K_r;
    float radialNorm;
    float lambdaScale;
    float rEndUsed;
    float sStart;
    float sTurn;
    float sInf;
    float deltaS_total;
    float nearRadialDPhiTotal;
    bool hasTurning;
    bool isInward;
    bool isCaptured; // ends at the horizon (no sky direction)
    bool innerRegion; // 4 real roots, camera in the pocket r3 <= r <= r2 below the photon orbit
    int numRealRoots;
    EllipticParams ep;
};

// Escaping near-radial rays: integrate all the way to r = infinity with u = 1/r.
//   dr / sqrt(R(r)) = du / sqrt(Ru(u)),  Ru(u) = u^4 R(1/u)
//   Ru = (1 + (a^2 - a xi) u^2)^2 - u^2 (1 - 2Mu + a^2 u^2)(eta + (xi - a)^2)
// Everything is smooth on u in [0, 1/rStart].
float RadialRu(float u, float a, float M, float xi, float eta)
{
    float X = 1.0 + (a * a - a * xi) * u * u;
    return max(X * X - u * u * (1.0 - 2.0 * M * u + a * a * u * u) * (eta + (xi - a) * (xi - a)), 1e-12);
}

// GL8 in u over [uLo, 1/rStart] (uLo = 0: to infinity, uLo = 1/rEnd: to rEnd).
// Verified < 1e-7 relative vs GL128 for cameras at r >= 1.3 r+.
float RadialMinoToInf(float a, float M, float xi, float eta, float rStart, float uLo)
{
    float h = 0.5 * (1.0 / rStart - uLo);
    float total = 0.0;
    [unroll]
    for (int i = 0; i < 8; i++)
    {
        float u = uLo + h + h * GL8_NODES[i];
        total += GL8_WEIGHTS[i] * rsqrt(RadialRu(u, a, M, xi, eta));
    }
    return total * h;
}

float RadialDPhiToInf(float a, float M, float xi, float eta, float rStart, float uLo)
{
    float h = 0.5 * (1.0 / rStart - uLo);
    float total = 0.0;
    [unroll]
    for (int i = 0; i < 8; i++)
    {
        float u = uLo + h + h * GL8_NODES[i];
        float num = 1.0 + (a * a - a * xi) * u * u;
        float den = 1.0 - 2.0 * M * u + a * a * u * u;
        total += GL8_WEIGHTS[i] * a * num / den * rsqrt(RadialRu(u, a, M, xi, eta));
    }
    return total * h;
}

void SetQuadratureRadialMotion(inout RadialMotion rm, float a, float M, float xi, float eta,
                               float rStart, float rTarget, bool escaping, float rOuter)
{
    rm.r1 = rStart;
    rm.r2 = rTarget;
    rm.r3 = 0.0;
    rm.r4 = 0.0;
    rm.ep = MakeEllipticParams(1.0);
    rm.m_r = 0.0;
    rm.K_r = 1.0;
    rm.radialNorm = 1.0;
    rm.lambdaScale = 1.0; // deltaS_total is already physical Mino time here
    rm.hasTurning = false;
    rm.isCaptured = !escaping;
    rm.innerRegion = false;
    rm.rEndUsed = rTarget;
    // Plunging quadrature rays have no disk crossings and no exit direction:
    // nothing downstream reads these, so skip the 64-point integrals.
    rm.deltaS_total = 0.0;
    rm.nearRadialDPhiTotal = 0.0;
    [branch]
    if (escaping)
    {
        // outgoing: camera -> r = rOuter (infinity if rOuter >= 1e29)
        float uLo = (rOuter < 1e29) ? 1.0 / max(rOuter, rStart) : 0.0;
        rm.deltaS_total = RadialMinoToInf(a, M, xi, eta, rStart, uLo);
        if (a != 0.0)
            rm.nearRadialDPhiTotal = RadialDPhiToInf(a, M, xi, eta, rStart, uLo);
    }
    rm.sStart = 0.0;
    rm.sInf = rm.deltaS_total;
    rm.numRealRoots = 1;
}

// rOuter: where escaping rays stop -- >= 1e29 means r = infinity (exact asymptote),
// finite means the ray is followed out to r = rOuter only (hand-off to another hole).
// sqrtR0: sqrt(R(rStart)) from the start's local radial direction (< 0: not known). Near a radial
// turning point (a ray leaving the camera square to the hole) the start's phase went as
// sqrt(rStart - r1), r1 from the float quartic (~1e-5 relative): its error ~5e-3, the exit up to
// 3e-4 rad off (1 arcmin) -- a few satellite radii at a hand-off, the satellite's lensing jumping
// along the plane square to the hole through the camera. R(rStart) from the direction is exact.
bool SetupRadialMotion(float a, float M, float xi, float eta,
                       float rStart, float rEscape, float n_r, float rOuter, float sqrtR0,
                       out RadialMotion rm)
{
    bool finiteEnd = (rOuter < 1e29);
    rm.r1 = rm.r2 = rm.r3 = rm.r4 = 0.0;
    rm.ep = MakeEllipticParams(1.0);
    rm.m_r = 0.0;
    rm.K_r = 1.0;
    rm.radialNorm = 1.0;
    rm.lambdaScale = 4.0;
    rm.rEndUsed = rEscape;
    rm.sStart = rm.sTurn = rm.sInf = 0.0;
    rm.deltaS_total = 0.0;
    rm.hasTurning = false;
    rm.isInward = (n_r < 0.0);
    rm.isCaptured = false;
    rm.innerRegion = false;
    rm.nearRadialDPhiTotal = 0.0;
    rm.numRealRoots = 0;

    float rHorizon = M + sqrt(max(M * M - a * a, 0.0));
    float rTargetPlunge = (n_r < 0.0f) ? (rHorizon + 0.02f) : rEscape;

    const float NEAR_RADIAL_B2 = 1e-3;
    if (eta + xi * xi < NEAR_RADIAL_B2)
    {
        SetQuadratureRadialMotion(rm, a, M, xi, eta, rStart, rTargetPlunge, n_r >= 0.0, rOuter);
        return true;
    }

    float p = a * a - eta - xi * xi;
    float q = 2.0 * M * (eta + (xi - a) * (xi - a));
    float s = -a * a * eta;

    float4 roots;
    int numReal;
    SolveDepressedQuarticReal(p, q, s, roots, numReal);
    // No Newton polish: with the stable resolvent the roots are already at
    // float precision, and near the shadow edge (near-double roots) Newton
    // steps on f/f' could throw a root far off.

    rm.numRealRoots = numReal;

    // Four real roots all inside the horizon (r1 < r+) is a genuine Kerr case
    // (high spin, prograde, near-equatorial). It is NOT a complex pair: the
    // 4-root map r(sn^2) is valid for every r >= r1, so plunging rays use it
    // directly with the horizon as the endpoint.
    bool useTwoRoot = (numReal < 4);
    if (!useTwoRoot)
        useTwoRoot = (roots.x > 500.0) || (roots.x < roots.y);

    if (useTwoRoot)
    {
        float v2Test = (numReal >= 2)
            ? p + roots.x * roots.x + roots.x * roots.y + roots.y * roots.y
              - 0.25 * (roots.x + roots.y) * (roots.x + roots.y)
            : -1.0;

        if (numReal < 2 || v2Test < 0.0)
        {
            SetQuadratureRadialMotion(rm, a, M, xi, eta, rStart, rTargetPlunge, n_r >= 0.0, rOuter);
            return true;
        }

        // --------------------------------------------------------------
        // 2 real roots + complex pair (Case 3)
        // --------------------------------------------------------------
        float r1 = roots.x;
        float r2 = roots.y;
        float u = -0.5f * (r1 + r2);
        float v2 = max(v2Test, 1e-8f);

        float A = sqrt(max((r1 - u) * (r1 - u) + v2, 1e-12f));
        float B = sqrt(max((r2 - u) * (r2 - u) + v2, 1e-12f));

        rm.r1 = r1;
        rm.r2 = r2;
        rm.r3 = u;
        rm.r4 = sqrt(v2);
        rm.ep = MakeEllipticParams(saturate(((r1 - r2) * (r1 - r2) - (A - B) * (A - B)) / max(4.0f * A * B, 1e-12f)));
        rm.m_r = rm.ep.m;
        rm.K_r = LutK(rm.ep);
        rm.radialNorm = 1.0f / sqrt(max(A * B, 1e-12f));
        rm.lambdaScale = 4.0f;
        rm.numRealRoots = 2;

        const float TURN_EPS = 1e-4;
        bool canTurn = (r1 > rHorizon + TURN_EPS) && (n_r < 0.0f) && (rStart > r1 - TURN_EPS);

        // Escaping rays end at r = infinity (exact sky direction), plunging at the horizon.
        bool plunge2 = (n_r < 0.0f) && !canTurn;
        rm.isCaptured = plunge2;
        // finite end on the OUTGOING leg: after the turning point if there is one (the
        // trace may start outside rOuter, e.g. a far camera), else beyond the start
        float rOut2 = canTurn ? max(rOuter, r1) : max(rOuter, rStart);
        rm.rEndUsed = plunge2 ? rTargetPlunge : (finiteEnd ? rOut2 : 1e30);

        float cosPsiStart = clamp(((A - B) * rStart + (r1 * B - r2 * A)) /
            max((A + B) * rStart - (r1 * B + r2 * A), 1e-12f), -1.0f, 1.0f);
        float cosPsiEsc = plunge2
            ? clamp(((A - B) * rTargetPlunge + (r1 * B - r2 * A)) /
                    max((A + B) * rTargetPlunge - (r1 * B + r2 * A), 1e-12f), -1.0f, 1.0f)
            : (finiteEnd
                ? clamp(((A - B) * rOut2 + (r1 * B - r2 * A)) /
                        max((A + B) * rOut2 - (r1 * B + r2 * A), 1e-12f), -1.0f, 1.0f)
                : clamp((A - B) / (A + B), -1.0f, 1.0f)); // R -> infinity

        // sin psi near the turning point (cos -> 1): sin^2 = 4AB (r - r1)(r - r2) / den^2 with
        // (r - r1)(r - r2) = R(r) / ((r - u)^2 + v^2) -- no r - r1
        float sinPsiStart = sqrt(saturate(1.0 - cosPsiStart * cosPsiStart));
        [branch]
        if (sqrtR0 >= 0.0 && cosPsiStart > 0.0)
        {
            float uS = rStart - u;
            float denS2 = max((A + B) * rStart - (r1 * B + r2 * A), 1e-12f);
            sinPsiStart = saturate(2.0 * sqrtR0 * sqrt(A * B / max(uS * uS + v2, 1e-12f)) / denS2);
        }
        rm.sStart = ForwardPhaseCS(rm.ep, cosPsiStart, sinPsiStart);
        rm.sInf = ForwardPhaseCS(rm.ep, cosPsiEsc, sqrt(saturate(1.0 - cosPsiEsc * cosPsiEsc)));
        rm.sTurn = 0.0f;

        if (canTurn)
        {
            rm.hasTurning = true;
            rm.sStart = max(rm.sStart, rm.sTurn + 1e-7f);
            rm.deltaS_total = abs(rm.sTurn - rm.sStart) + abs(rm.sInf - rm.sTurn);
        }
        else
        {
            rm.hasTurning = false;
            if (abs(rm.sInf - rm.sStart) < 1e-5)
                rm.sInf = rm.sStart + 1e-5;
            rm.deltaS_total = abs(rm.sInf - rm.sStart);
        }
        return true;
    }

    // ------------------------------------------------------------------
    // 4 real roots
    // ------------------------------------------------------------------
    rm.r1 = roots.x;
    rm.r2 = roots.y;
    rm.r3 = roots.z;
    rm.r4 = roots.w;

    float den4 = max(abs((rm.r1 - rm.r3) * (rm.r2 - rm.r4)), 1e-12);
    rm.ep = MakeEllipticParams(saturate(abs((rm.r1 - rm.r2) * (rm.r3 - rm.r4)) / den4)); // exact 1 - m
    rm.m_r = rm.ep.m;
    rm.K_r = LutK(rm.ep);
    rm.radialNorm = 1.0 / max(sqrt(max((rm.r1 - rm.r3) * (rm.r2 - rm.r4), 1e-12)), 1e-8);
    rm.lambdaScale = 8.0f;

    // ------------------------------------------------------------------
    // Camera inside the photon-orbit pocket (r3 <= rStart <= r2, below the
    // unstable photon orbit; happens when the camera is close to the hole):
    // R(r) < 0 for r2 < r < r1, so NO ray from here can escape. Inward rays
    // plunge; outward rays climb to r2, turn, and plunge. Treating these as
    // escaping (the old behaviour) mapped them to garbage sky/disk directions
    // -- the "mirrored" image close to the horizon.
    // Parametrisation on [r3, r2] (same modulus, same Mino scale as r >= r1):
    //   sn^2 = (r2-r4)(r-r3) / ((r2-r3)(r-r4)),  cn^2 = (r3-r4)(r2-r) / ((r2-r3)(r-r4))
    //   r(sn^2) = (r3(r2-r4) - r4(r2-r3) sn^2) / ((r2-r4) - (r2-r3) sn^2)
    // r2 is phase 1/4 (sn = 1). R(rStart) >= 0 guarantees the camera is in one of
    // the two regions; pick the nearer one (robust to float rounding at r1).
    // ------------------------------------------------------------------
    if (rStart - rm.r2 < rm.r1 - rStart)
    {
        float rH2 = rHorizon + 0.02;
        float rS = min(rStart, rm.r2);
        float denIS = max((rm.r2 - rm.r3) * (rS - rm.r4), 1e-12);
        float sinS = sqrt(saturate((rm.r2 - rm.r4) * (rS - rm.r3) / denIS));
        float cosS = sqrt(saturate((rm.r3 - rm.r4) * (rm.r2 - rS) / denIS));
        float denIH = max((rm.r2 - rm.r3) * (rH2 - rm.r4), 1e-12);
        float sinH = sqrt(saturate((rm.r2 - rm.r4) * (rH2 - rm.r3) / denIH));
        float cosH = sqrt(saturate((rm.r3 - rm.r4) * (rm.r2 - rH2) / denIH));

        rm.innerRegion = true;
        rm.isCaptured = true;
        rm.rEndUsed = rH2;
        rm.sTurn = 0.25;
        rm.sStart = min(ForwardPhaseCS(rm.ep, cosS, sinS), rm.sTurn - 1e-5);
        rm.sInf = ForwardPhaseCS(rm.ep, cosH, sinH); // phase at the horizon endpoint
        rm.hasTurning = !rm.isInward; // outward: up to r2, then down
        rm.deltaS_total = rm.hasTurning
            ? (rm.sTurn - rm.sStart) + (rm.sTurn - rm.sInf)
            : max(rm.sStart - rm.sInf, 1e-5);
        return true;
    }

    const float TURN_EPS = 1e-4;
    bool canTurn = (rm.r1 > rHorizon + TURN_EPS) && rm.isInward &&
                   (rStart > rm.r1 - TURN_EPS) && (rm.r2 > rm.r3 + TURN_EPS);

    // Plunging (inward, no turning point) ends at the horizon; everything else
    // escapes to r = infinity, where sn^2 -> (r1-r3)/(r1-r4), cn^2 -> (r3-r4)/(r1-r4).
    bool plunge4 = rm.isInward && !canTurn;
    rm.isCaptured = plunge4;
    float rOut4 = canTurn ? max(rOuter, rm.r1) : max(rOuter, rStart);
    rm.rEndUsed = plunge4 ? rTargetPlunge : (finiteEnd ? rOut4 : 1e30);

    // sn^2 = (r1-r3)(R-r4) / ((r1-r4)(R-r3)),  cn^2 = (r3-r4)(R-r1) / ((r1-r4)(R-r3))  (no 1-sn^2 cancellation)
    float denS = max((rm.r1 - rm.r4) * (rStart - rm.r3), 1e-12);
    float sinPsiStart = sqrt(saturate((rm.r1 - rm.r3) * (rStart - rm.r4) / denS));
    float cosPsiStart = sqrt(saturate((rm.r3 - rm.r4) * (rStart - rm.r1) / denS));
    // near the turning point: rStart - r1 = R(rStart) / ((rStart - r2)(rStart - r3)(rStart - r4)) (see sqrtR0)
    [branch]
    if (sqrtR0 >= 0.0 && sinPsiStart > cosPsiStart)
    {
        float dr1 = sqrtR0 * sqrtR0 / max((rStart - rm.r2) * (rStart - rm.r3) * (rStart - rm.r4), 1e-12);
        cosPsiStart = sqrt(saturate((rm.r3 - rm.r4) * dr1 / denS));
    }
    float sinPsiEsc, cosPsiEsc;
    if (plunge4)
    {
        float denE = max((rm.r1 - rm.r4) * (rTargetPlunge - rm.r3), 1e-12);
        sinPsiEsc = sqrt(saturate((rm.r1 - rm.r3) * (rTargetPlunge - rm.r4) / denE));
        cosPsiEsc = sqrt(saturate((rm.r3 - rm.r4) * (rTargetPlunge - rm.r1) / denE));
    }
    else if (finiteEnd)
    {
        float denO = max((rm.r1 - rm.r4) * (rOut4 - rm.r3), 1e-12);
        sinPsiEsc = sqrt(saturate((rm.r1 - rm.r3) * (rOut4 - rm.r4) / denO));
        cosPsiEsc = sqrt(saturate((rm.r3 - rm.r4) * (rOut4 - rm.r1) / denO));
    }
    else
    {
        float denInf = max(rm.r1 - rm.r4, 1e-12);
        sinPsiEsc = sqrt(saturate((rm.r1 - rm.r3) / denInf));
        cosPsiEsc = sqrt(saturate((rm.r3 - rm.r4) / denInf));
    }

    rm.sStart = ForwardPhaseCS(rm.ep, cosPsiStart, sinPsiStart);
    rm.sInf = ForwardPhaseCS(rm.ep, cosPsiEsc, sinPsiEsc);
    rm.sTurn = 0.25f;
    rm.sStart = min(rm.sStart, rm.sTurn - 1e-7f);

    if (canTurn)
    {
        rm.hasTurning = true;
        rm.deltaS_total = max(rm.sTurn - rm.sStart, 0.0) + max(rm.sTurn - rm.sInf, 0.0);
    }
    else
    {
        rm.hasTurning = false;
        rm.deltaS_total = abs(rm.sInf - rm.sStart);
    }

    return true;
}

float ProgressiveToNativePhase(RadialMotion rm, float sProgVal)
{
    if (!rm.hasTurning)
    {
        float span = rm.sInf - rm.sStart;
        return rm.sStart + sProgVal * sign(span + 1e-12);
    }

    float toTurn = abs(rm.sTurn - rm.sStart);
    float dirIn = sign(rm.sTurn - rm.sStart);
    if (sProgVal <= toTurn)
        return rm.sStart + dirIn * sProgVal;

    float dirOut = sign(rm.sInf - rm.sTurn);
    return rm.sTurn + dirOut * (sProgVal - toTurn);
}

float EvaluateR(RadialMotion rm, float nativePhase)
{
    float ph = clamp(nativePhase, 0.0, 0.5);

    if (rm.numRealRoots >= 4)
    {
        // 4-root phases never leave [0, 0.25]
        float2 snCn = LutSnCnQuarter(rm.ep, min(ph, 0.25));
        float sn2 = snCn.x * snCn.x;
        [branch]
        if (rm.innerRegion)
        {
            float d24 = rm.r2 - rm.r4;
            float d23 = rm.r2 - rm.r3;
            return (rm.r3 * d24 - rm.r4 * d23 * sn2) / max(d24 - d23 * sn2, 1e-12);
        }
        float A = rm.r1 - rm.r3;
        float B = rm.r1 - rm.r4;
        float den = A - B * sn2;
        if (abs(den) < 1e-8)
            return rm.r1;
        return max((rm.r4 * A - rm.r3 * B * sn2) / den, rm.r1);
    }

    float2 snCn2 = LutSnCnHalf(rm.ep, ph);
    float A2 = sqrt((rm.r1 - rm.r3) * (rm.r1 - rm.r3) + rm.r4 * rm.r4);
    float B2 = sqrt((rm.r2 - rm.r3) * (rm.r2 - rm.r3) + rm.r4 * rm.r4);
    float cn = snCn2.y;
    float denom = cn * (A2 + B2) - (A2 - B2);
    if (abs(denom) < 1e-10)
        return 0.5 * (rm.r1 + rm.r2);
    return (rm.r1 * B2 * (1.0 + cn) - rm.r2 * A2 * (1.0 - cn)) / denom;
}

// Radial (frame-dragging) azimuth from the camera up to progressive phase sTarget.
// GL8 in radial phase: r(s) is analytic through the turning point in this variable.
// Verified vs GL64: <1e-6 rad for totals and for crossings at r >= rH + 0.3
// (~2e-3 rad worst for crossings within 0.1 of the horizon).
float RadialPhiPartial(RadialMotion rm, float a, float xi, float sTarget)
{
    float h = 0.5 * sTarget;
    float total = 0.0;
    [unroll]
    for (int i = 0; i < 8; i++)
    {
        float r = EvaluateR(rm, ProgressiveToNativePhase(rm, h + h * GL8_NODES[i]));
        float Delta_r = r * r - 2.0 * KERR_M * r + a * a;
        total += GL8_WEIGHTS[i] * a * ((r * r + a * a) - a * xi) / Delta_r;
    }
    return total * h * rm.lambdaScale * rm.K_r * rm.radialNorm;
}

// Total radial azimuth. Proportional to a: caller skips at a = 0.
void EvaluatePhiT(RadialMotion rm, float a, float M, float xi,
                  out float dPhi, out float dT, float rStartRadius)
{
    dT = 0.0;
    if (rm.numRealRoots == 1)
        dPhi = rm.nearRadialDPhiTotal; // lambda increases along the ray in both directions
    else
        dPhi = RadialPhiPartial(rm, a, xi, rm.deltaS_total);
}

// ------------------------------------------------------------------
// Polar azimuth
// ------------------------------------------------------------------
// Ordinary motion, cos(theta) = sqrt(u+) cn(v), v = 4K t, psi = am(v):
//   integral xi dlambda / sin^2(theta) = xi/kTheta * integral dpsi / ((1 - u+ cos^2 psi) dn(psi))
// With tan(chi) = tan(psi) / c, c = sqrt(1 - u+), the pole peak cancels exactly:
//   = xi/(kTheta c) * integral_0^chi sqrt((cos^2 + c^2 sin^2) / (cos^2 + eps^2 sin^2)) dchi,
//   eps = c sqrt(1 - m)
// and with pi/2 - chi = eps sinh(y) the remaining dn peak (m -> 1) cancels too.
// GL6 in y: < 1.1e-4 rad against adaptive quadrature over all spins and m in [0, 1 - 1e-8].
// Closed form at a = 0 (m = 0, eps = c): the integral is just chi.

// Vortical motion only (rare, never crosses the equator): plain quadrature.
float EvaluatePolarPhiSegmentU(ThetaMotion tm, float xi, float a, float u_a, float u_b)
{
    float K_safe = max(tm.K_theta, 1e-5);
    float lam_a = (u_a - tm.s0) * 4.0 * K_safe / tm.kTheta;
    float lam_b = (u_b - tm.s0) * 4.0 * K_safe / tm.kTheta;
    float mid = 0.5 * (lam_a + lam_b);
    float halff = 0.5 * (lam_b - lam_a);

    float seg = 0.0;
    [loop]
    for (int j = 0; j < GL32_N; j++)
    {
        float t = (PI * 0.5) * GL32_NODES[j];
        float sinT_, cosT_;
        sincos(t, sinT_, cosT_);
        float lam = mid + halff * sinT_;
        float dlam_dt = halff * cosT_;
        float w = GL32_WEIGHTS[j] * (PI * 0.5);
        float cosTheta_ = EvaluateCosTheta(tm, lam);
        float sin2 = max(1.0 - cosTheta_ * cosTheta_, 1e-12);
        seg += w * (xi / sin2 - a) * dlam_dt;
    }
    return seg;
}

// sin^2(theta) has period 0.5 in theta-phase and is mirror symmetric about 0.25, so
//   I(u) = floor(2u) P + J(t),  J(t) = Q(t) (t <= 1/4),  J(t) = P - Q(1/2 - t) (t > 1/4)
// where Q(t) is the integral from the turning point (t = 0) and P = 2 Q(1/4).
struct PolarPhiCum
{
    float P; // one half-period
    float J25; // J(0.25)
    float lamPerU; // dlambda / du
    float I0; // I(s0)
    float coefXi; // xi / (kTheta c)
    float c2; // c^2
    float c;
    float eps;
    float e2; // eps^2
    float invEps;
    float y1; // asinh(pi/2 / eps)
    float yc; // asinh(c / eps): where the remainder f - 1 of G's integrand lives (see PolarG)
    bool exactChi; // a == 0: m = 0 and G(chi) = chi exactly
    bool vortical;
};

// One GL6 node pair: integrand of G at e^y = ey (iey = 1/ey)
// (f - 1) dchi / dy, f = G's integrand: f^2 - 1 = (c^2 - eps^2) cos^2 / (sin^2 + eps^2 cos^2) in pi/2 - chi,
// without forming f ~ 1 and subtracting
float PolarGTerm(PolarPhiCum pc, float ey, float iey)
{
    float sd = sin(pc.eps * 0.5 * (ey - iey));
    float sd2 = sd * sd;
    float den = pc.e2 + (1.0 - pc.e2) * sd2;
    float r1 = (pc.c2 - pc.e2) * (1.0 - sd2) / den; // f^2 - 1
    return r1 / (sqrt(1.0 + r1) + 1.0) * (ey + iey);
}

// G(chi) = integral_0^chi sqrt((cos^2 + c^2 sin^2)/(cos^2 + eps^2 sin^2)), via
// pi/2 - chi = eps sinh(y), GL6 (max 1.1e-4 rad over all spins / m).
// The GL6 nodes are symmetric, so e^(mid +- h x_i) = e^mid * e^(+-h x_i):
// 4 exp + 2 rcp instead of 6 exp + 6 rcp, same result.
// Takes chiC = pi/2 - chi, computed by the callers directly (atan2 with the arguments swapped): for a ray
// passing close over the pole c, eps ~ 1e-6, chi is within ~1e-6 of pi/2, and pi/2 - chi in float lost
// ~10 % to rounding -- times xi / (kTheta c) ~ O(1), a polar azimuth off by up to ~0.01 rad: the exit
// direction jumped (13 arcmin) for every ray within ~1e-5 rad of the spin axis (a seam along the projected
// axis, and failed reconstructions there).
//
// G = chi + integral of (f - 1): the "1" exactly, GL6 only for the remainder. The whole integrand in y is
// ~ c flat up to y ~ yc and then eps cosh y, an exponential; for a ray passing close by the pole (c, eps ~ 1e-6)
// that ran over ~15 e-folds of y, which GL6 cannot integrate: errors to 5e-3 rad in the azimuth (the exit
// direction jumped by ~0.3 deg across the projected spin axis). The remainder falls off as e^-y past yc, so it
// is integrated to yc + 6 only. Tested vs mpmath on 600 escaping rays (spins 0.6, ~1): max error 2.5e-8 rad
// (was 2.3e-3); same 6 nodes, same cost.
float PolarG(PolarPhiCum pc, float chiC)
{
    [branch]
    if (pc.exactChi)
        return 0.5 * PI - chiC;
    float x0 = chiC * pc.invEps;
    float y0 = log(x0 + sqrt(x0 * x0 + 1.0));
    float yb = min(pc.y1, max(pc.yc, y0) + 6.0);
    float h = 0.5 * (yb - y0);
    float mid = 0.5 * (yb + y0);
    float em = exp(mid);
    float iem = 1.0 / em;
    float sum = 0.0;
    [unroll]
    for (int i = 3; i < 6; i++) // positive nodes; weights are symmetric
    {
        float en = exp(h * GL6_NODES[i]);
        float ien = 1.0 / en;
        sum += GL6_WEIGHTS[i] * (PolarGTerm(pc, em * en, iem * ien) + PolarGTerm(pc, em * ien, iem * en));
    }
    return (0.5 * PI - chiC) + sum * h * pc.eps * 0.5;
}

// Q at folded phase t in [0, 1/4], given its chiC = pi/2 - chi
float PolarPhiQ(PolarPhiCum pc, float a, float chiC, float t)
{
    return pc.coefXi * PolarG(pc, chiC) - a * pc.lamPerU * t;
}

// J(t), t in [0, 1/2), for ordinary motion, given chiC = pi/2 - chi at the folded phase
float PolarPhiJChi(PolarPhiCum pc, float a, float chiC, float t)
{
    return (t <= 0.25) ? PolarPhiQ(pc, a, chiC, t) : pc.P - PolarPhiQ(pc, a, chiC, 0.5 - t);
}

// Vortical fallback (quadrature)
float PolarPhiJVortical(ThetaMotion tm, PolarPhiCum pc, float xi, float a, float t)
{
    return (t <= 0.25) ? EvaluatePolarPhiSegmentU(tm, xi, a, 0.0, t)
                       : pc.P - EvaluatePolarPhiSegmentU(tm, xi, a, 0.0, 0.5 - t);
}

PolarPhiCum MakePolarPhiCum(ThetaMotion tm, float xi, float a)
{
    PolarPhiCum pc;
    pc.lamPerU = 4.0 * max(tm.K_theta, 1e-5) / tm.kTheta;
    pc.vortical = tm.isVortical;
    pc.exactChi = (a == 0.0);
    pc.c = tm.cPol;
    pc.c2 = tm.cPol * tm.cPol;
    pc.eps = max(tm.cPol * tm.ep.sqrtMc, 1e-30);
    pc.e2 = pc.eps * pc.eps;
    pc.invEps = 1.0 / pc.eps;
    float x1 = 0.5 * PI * pc.invEps;
    pc.y1 = log(x1 + sqrt(x1 * x1 + 1.0));
    float xc = pc.c * pc.invEps; // = 1 / sqrt(1 - m)
    pc.yc = log(xc + sqrt(xc * xc + 1.0));
    pc.coefXi = xi / (tm.kTheta * tm.cPol);

    float k0 = floor(2.0 * tm.s0);
    float t0 = tm.s0 - 0.5 * k0;
    [branch]
    if (pc.vortical)
    {
        pc.P = EvaluatePolarPhiSegmentU(tm, xi, a, 0.0, 0.5);
        pc.J25 = 0.5 * pc.P;
        pc.I0 = k0 * pc.P + PolarPhiJVortical(tm, pc, xi, a, t0);
    }
    else
    {
        pc.P = 2.0 * (pc.coefXi * PolarG(pc, 0.0) - a * pc.lamPerU * 0.25);
        pc.J25 = 0.5 * pc.P;
        pc.I0 = k0 * pc.P + PolarPhiJChi(pc, a, tm.chiC0, t0); // no LUT fetch
    }
    return pc;
}

// Polar azimuth from lambda = 0 to the k-th equator phase u = 0.25 + 0.5k. O(1).
float PolarPhiToEquator(PolarPhiCum pc, int k)
{
    return (float) k * pc.P + pc.J25 - pc.I0;
}

// End of an escaping ray: cos(theta) and the total polar azimuth from ONE fetch
// (EvaluateCosTheta and Q(t) need sn/cn at the same folded phase).
void EvaluateThetaEnd(ThetaMotion tm, PolarPhiCum pc, float xi, float a, float lambda,
                      out float cosThetaEnd, out float polarDPhi, out float thetaDirSign, out float thetaSn2)
{
    float u = tm.s0 + lambda * tm.kTheta / (4.0 * max(tm.K_theta, 1e-5));
    float k = floor(2.0 * u);
    float t = u - 0.5 * k;
    float arg = (t <= 0.25) ? t : 0.5 - t;
    float2 sncn = LutSnCnQuarter(tm.ep, arg);

    float u01 = frac(u);
    if (!tm.isVortical)
    {
        float maxCos = sqrt(saturate(tm.uPlus));
        float signRestore = (u01 >= 0.25 && u01 < 0.75) ? -1.0 : 1.0;
        cosThetaEnd = clamp(maxCos * sncn.y * signRestore, -maxCos, maxCos);
        float chiC = atan2(pc.c * sncn.y, sncn.x); // pi/2 - chi (see PolarG)
        polarDPhi = k * pc.P + PolarPhiJChi(pc, a, chiC, t) - pc.I0;
        thetaDirSign = (u01 < 0.5) ? 1.0 : -1.0; // cos(theta) falls for u01 < 1/2
        // 1 - mu^2 / u+ without the cancellation, from the same cn as mu (not the table's sn: its own
        // interpolation error would put the velocity off the position)
        float cnA = saturate(abs(sncn.y));
        thetaSn2 = (1.0 - cnA) * (1.0 + cnA);

    }
    else
    {
        float dn2 = max(1.0 - tm.mTheta * sncn.x * sncn.x, 1e-8);
        cosThetaEnd = clamp(sqrt(clamp(tm.uMinus / dn2, 0.0, tm.uPlus)) * tm.hemiSign, -0.999999, 0.999999);
        polarDPhi = k * pc.P + PolarPhiJVortical(tm, pc, xi, a, t) - pc.I0;
        float phaseSign = (u01 < 0.25 || (u01 >= 0.5 && u01 < 0.75)) ? 1.0 : -1.0;
        thetaDirSign = -tm.hemiSign * phaseSign;
        thetaSn2 = -1.0;
    }
}

// ------------------------------------------------------------------
// Disk shading
// ------------------------------------------------------------------
// Per-hole constants (BlackHoleGlobalManager; recomputed only when spin, orbit
// direction or ISCO stress change -- none of them depend on the pixel):
//   _BHDiskConst0[h] = (rIsco, structure life (Rs/c), Omega at the ISCO, dr/dt of the gas at the ISCO)
//   _BHDiskConst1[h] = (E_isco, L_isco (orbit-sense frame), 1 / peak flux, rLow = r+ + 0.05)
//   _BHPlungeLUT row h = plunge stream (Phi, T) at s = cbrt((rIsco - r) / (rIsco - rLow))
float4 _BHDiskConst0[9];
float4 _BHDiskConst1[9];
float4 _BHDiskTint[9]; // rgb: tint colour (linear, luminance 1), w: strength 0..1
float4 _BHDiskLight[9]; // x: brightness by the visible light (1) instead of the total power (0), 0..1 (DiskVisible)
float4 _BHDiskCut[9]; // x = 1 - visibility of the hole's own disk (tides; C#), 0 = full
float _BHNoiseMean; // mean of the noise texture's red channel (CPU)
Texture2D<float2> _BHPlungeLUT;
#define BH_PLUNGE_LUT_W 64.0    // must match PlungeLutWidth in C#
#define BH_PLUNGE_LUT_H 9.0     // = MaxBlackHoles

// 1: radial frame-dragging azimuth integrated segment by segment between crossings
//    (constant part analytic, GL4 per segment, GL6 for the last leg).
// 0: old path (GL8 from the camera for every crossing, GL8 for the total). A/B switch.
#define KERR_RADPHI_FAST 1

static const float GL4_NODES[4] =
{
    -0.86113631159405258f, -0.33998104358485626f, 0.33998104358485626f, 0.86113631159405258f
};
static const float GL4_WEIGHTS[4] =
{
    0.34785484513745386f, 0.65214515486254614f, 0.65214515486254614f, 0.34785484513745386f
};

struct HoleDisk
{
    float rIsco;
    float life;
    float OmegaIn;
    float drdtIsco;
    float E;
    float L;
    float fluxNorm;
    float rLow;
    int h;
};

HoleDisk LoadHoleDisk(int h)
{
    HoleDisk hd;
    float4 c0 = _BHDiskConst0[h];
    float4 c1 = _BHDiskConst1[h];
    hd.rIsco = c0.x;
    hd.life = c0.y;
    hd.OmegaIn = c0.z;
    hd.drdtIsco = c0.w;
    hd.E = c1.x;
    hd.L = c1.y;
    hd.fluxNorm = c1.z;
    hd.rLow = c1.w;
    hd.h = h;
    return hd;
}

// Read-only inputs of the crossing loop (one hole, one ray).
struct DiskCtx
{
    float aPhys;
    float keplerFactor;
    bool plungeOn;
    float diskOutNorm;
    float rHorizon;
    float rotOffset; // static texture angle
    float2 genPhase; // generation clock (fraction, integer part)
    float4 ap1; // peak T, Doppler colour, Doppler beaming, radial contrast
    float4 ap2; // brightness, ISCO stress, turbulence, higher-order fade
    HoleDisk hd;
};

// Everything the crossing loop accumulates / hands back to KerrTraceCore.
struct CrossingAcc
{
    float3 color;
    float transmittance;
    float firstDopplerG;
    int crossingCount;
    float firstRCross;
    float radS; // progressive phase the radial azimuth is integrated up to
    float radPhiRem; // remainder part of the radial azimuth up to radS
    bool hasFirst;
    float4 firstCross; // (r, phi, xiPhys, photonPr) of the first visible crossing
    float firstIdx;
    float3 emFirst;
};

CrossingAcc MakeCrossingAcc()
{
    CrossingAcc acc;
    acc.color = float3(0, 0, 0);
    acc.transmittance = 1.0;
    acc.firstDopplerG = 0.0;
    acc.crossingCount = 0;
    acc.firstRCross = -1.0;
    acc.radS = 0.0;
    acc.radPhiRem = 0.0;
    acc.hasFirst = false;
    acc.firstCross = float4(0, 0, 0, 0);
    acc.firstIdx = 0.0;
    acc.emFirst = float3(0, 0, 0);
    return acc;
}

// Disk kinematics. Units: r in Rs, time in Rs/c (KERR_M = 0.5), so Omega is in
// radians per (Rs/c). keplerFactor k:
//   k = +1 : physical Keplerian orbit in +phi (counter-clockwise seen from +diskNormal)
//   k = -1 : physical Keplerian orbit in -phi
//   |k| < 1: sub-Keplerian, k = 0: static disk, |k| > 1: super-Keplerian (clamped)
// ISCO radius in Rs (Bardeen, Press & Teukolsky 1972). Reference only: the CPU
// computes it (BlackHoleGlobalManager.DiskIscoRadius) and passes it in _BHDiskConst0.
float DiskIscoRadius(float aPhys, float keplerFactor)
{
    float sgn = (keplerFactor < 0.0) ? -1.0 : 1.0;
    float chi = clamp(sgn * aPhys / KERR_M, -1.0, 1.0);
    float ac = abs(chi);
    float z1 = 1.0 + pow(1.0 - ac * ac, 1.0 / 3.0) * (pow(1.0 + ac, 1.0 / 3.0) + pow(1.0 - ac, 1.0 / 3.0));
    float z2 = sqrt(3.0 * ac * ac + z1 * z1);
    float root = sqrt(max((3.0 - z1) * (3.0 + z1 + 2.0 * z2), 0.0));
    float rIscoM = 3.0 + z2 + ((chi >= 0.0) ? -root : root);
    return rIscoM * KERR_M;
}

float DiskOmega(float r, float aPhys, float keplerFactor)
{
    float sgn = (keplerFactor < 0.0) ? -1.0 : 1.0;
    float sqrtM = sqrt(KERR_M);
    float kepler = sgn * sqrtM / (r * sqrt(r) + sgn * aPhys * sqrtM);
    return abs(keplerFactor) * kepler;
}

// Random texture offset for disk-structure generation n (periodic in 1024).
float2 DiskGenOffset(float n)
{
    uint h = (uint) ((int) n & 1023) * 747796405u + 2891336453u;
    h = ((h >> ((h >> 28u) + 4u)) ^ h) * 277803737u;
    h = (h >> 22u) ^ h;
    uint h2 = h * 747796405u + 2891336453u;
    h2 = ((h2 >> ((h2 >> 28u) + 4u)) ^ h2) * 277803737u;
    h2 = (h2 >> 22u) ^ h2;
    return float2(h & 0xFFFFu, h2 & 0xFFFFu) * (1.0 / 65536.0);
}

// u^t, u^phi and |u^r| (fall + drift) of plunging gas at r, orbit-sense frame
// (Cunningham 1975: E, L of the ISCO circular orbit, equatorial geodesic).
void KerrPlungeVel(float r, float ae, float E, float L, out float ut, out float uph, out float urMag)
{
    float M = KERR_M;
    float D = r * r - 2.0 * M * r + ae * ae;
    ut = ((r * r + ae * ae + 2.0 * M * ae * ae / r) * E - (2.0 * M * ae / r) * L) / D;
    uph = ((2.0 * M * ae / r) * E + (1.0 - 2.0 * M / r) * L) / D;
    float X = E * (r * r + ae * ae) - ae * L;
    float Rm = X * X - D * (r * r + (L - ae * E) * (L - ae * E));
    float uFall = sqrt(max(Rm, 0.0)) / (r * r);
    urMag = sqrt(KERR_DISK_DRIFT * KERR_DISK_DRIFT + uFall * uFall);
}

// Plunge stream: the gas now at (r, phi, t) left the ISCO at phi - Phi(r), t - T(r),
//   Phi(r) = integral_r^rIsco u^phi / |u^r| dr',  T(r) = integral_r^rIsco u^t / |u^r| dr'.
// Phi and T come from a per-hole LUT baked on the CPU (Simpson, double precision);
// only the local velocity is evaluated here. Omega and Phi in the physical sense.
void KerrPlungeStreamLUT(HoleDisk hd, float r, float aPhys, float orbitSign,
                         out float OmegaPlunge, out float urMag, out float PhiStream, out float TStream)
{
    float ut, uph;
    KerrPlungeVel(r, orbitSign * aPhys, hd.E, hd.L, ut, uph, urMag);
    OmegaPlunge = orbitSign * uph / ut;

    float W = max(hd.rIsco - hd.rLow, 1e-4);
    float s = pow(saturate((hd.rIsco - r) / W), 1.0 / 3.0);
    float2 uv = float2((s * (BH_PLUNGE_LUT_W - 1.0) + 0.5) / BH_PLUNGE_LUT_W,
                       ((float) hd.h + 0.5) / BH_PLUNGE_LUT_H);
    float2 v = _BHPlungeLUT.SampleLevel(sampler_linear_clamp, uv, 0);
    PhiStream = v.x;
    TStream = v.y;
}

// Redshift of equatorial gas moving with angular velocity Omega and inflow |u^r|:
//   (u^t)^2 = (1 + g_rr (u^r)^2) / -(g_tt + 2 g_tphi Omega + g_phiphi Omega^2)
//   g = 1 / (u^t (1 - Omega xi) + p_r |u^r|)
// Returns -1 where no such timelike flow exists (transparent). Also returns u^t.
float KerrFlowRedshift(float r, float xiPhys, float aPhys, float Omega, float urMag,
                       float photonPr, out float ut)
{
    float Delta = r * r - 2.0 * KERR_M * r + aPhys * aPhys;
    float gtt = -(1.0 - 2.0 * KERR_M / r);
    float gtp = -2.0 * KERR_M * aPhys / r;
    float gpp = r * r + aPhys * aPhys + 2.0 * KERR_M * aPhys * aPhys / r;
    float norm = -(gtt + 2.0 * gtp * Omega + gpp * Omega * Omega);
    ut = 1.0;
    if (norm <= 0.0 || Delta <= 0.0)
        return -1.0;
    ut = sqrt((1.0 + (r * r / Delta) * urMag * urMag) / norm);
    float pu = ut * (1.0 - Omega * xiPhys) + photonPr * urMag;
    return (pu > 0.0) ? clamp(1.0 / pu, 0.0, 3.0) : -1.0;
}

// Disk structure clock. The structure is noise carried with the gas (round at its Omega, creeping in with its
// drift), renewed in two staggered generations so the differential rotation never winds it up. A generation lives
// DISK_NOISE_LIFE_ORBITS orbits at the inner edge, and in the rest a fixed share of the local orbit (below): in life
// zones k = 0, 1, 2 ... (k = log2 Omega_in / Omega: zone 0 is the inner edge, Omega halving from each to the next), neighbouring
// zones blended across the outer fifth of each (there the noise is read twice). (One life everywhere -- the inner edge's -- renewed the outer
// disk long before it got anywhere: at 10 Rs it turned ~30 deg per generation, at 30 Rs ~5 deg. It faded in and out
// in place and drifted with the generations' radial phase, toward the hole: seen from the side, not going round.)
// genPhase = (fraction, integer 0..1023) of the hole's clock G in inner-edge lives (C#).
// Nothing in the outer zones moves inward faster than the inner edge's structure did: the generations there turn over
// all at once (zone 0 keeps its phase falling with radius -- fronts sweeping in at 6.7 Rs per inner-edge life, gone
// across its narrow band in a fifth of one), and the drift creeps the texture no further in a generation than at the
// inner edge. Seen from the side, the outer disk is where the line of sight runs along the gas -- its turning
// invisible, only motion across it shows: long-lived generations' fronts and creep (6.6 Rs per generation at 20 Rs)
// drew the clouds toward the middle.
#define DISK_LIFE_ZONES_MAX 9.0
float DiskLifeZone(float OmegaT, float OmegaIn)
{
    return clamp(log2(max(abs(OmegaIn), 1e-12) / max(abs(OmegaT), 1e-12)), 0.0, DISK_LIFE_ZONES_MAX - 0.001);
}
// The noise is read in log-polar coordinates (u round the disk, v ~ ln r, the same scale: a feature keeps its shape
// at every radius, its size ~ r, as the disk's own thickness and eddies), the textures' streaks (both ~3.6x longer
// in u than in v) lying along the orbits -- sheared gas, as turbulence in a disk is. A whole number of tiles round
// the disk; within ~DISK_TEX_RCORE inner-edge radii of the edge the features stop thinning radially (they would be
// a few texels across, a shimmer). Each generation's tiles are warped once round the disk by its own random phases
// (DISK_TEX_WARP_*: shifted along and across the orbits, the tile's copies show different parts of the texture; the
// two shifts a quarter turn apart, so where neighbouring copies differ little along the orbits, they differ most
// across them -- each of the two shifts is half a feature or more between neighbours, everywhere).
// (Before, the tiles went up with radius in steps -- 4 at the inner edge, ~88 at 22 inner-edge radii -- with the
// radial scale fixed in Rs: round a fast-spinning hole, whose inner edge is close in, the outer disk was radial
// spokes, the same few repeated ~90 times round it, and every life zone's edge a ring.)
#define DISK_TEX_TILES3 16.0   // 3D noise (DiskNoise3D, 1.8 x 6.4 cycles per tile): tiles round the disk
#define DISK_TEX_TILES2 3.0    // 2D noise (9.6 x 34 cycles per tile): about the same feature sizes
#define DISK_TEX_RCORE  2.0    // (inner-edge radii)
#define DISK_TEX_WARP_U1 0.25  // stretch along the orbit, fundamental (share of the tile pitch: features 0.75 - 1.25x)
#define DISK_TEX_WARP_U2 0.12  // ... second harmonic
#define DISK_TEX_WARP_V  0.12  // shift across the orbits (lean: +-7 deg; neighbours ~1.5 features apart)
float DiskTexLnR(float r, float rIsco)
{
    return log(max(r, 1e-4) + DISK_TEX_RCORE * rIsco);
}
// x: the gas's azimuth in its generation's frame (0..1 round the disk); off: the generation's random offset
float2 DiskTexUV(float x, float lnR, float tiles, float2 off)
{
    float a = TWO_PI * x;
    float u = x + (DISK_TEX_WARP_U1 / TWO_PI) * sin(a + TWO_PI * off.x)
                + (DISK_TEX_WARP_U2 / (2.0 * TWO_PI)) * sin(2.0 * a + TWO_PI * off.y);
    float v = (1.0 / TWO_PI) * lnR + (DISK_TEX_WARP_V / TWO_PI) * cos(a + TWO_PI * off.x);
    return float2(u, v) * tiles + off;
}
// The outer zones' generations live 2^-DISK_OUTER_LIFE_SHIFT of 2^k inner-edge lives, 1 - 1/2 of a local orbit --
// about the time an eddy holds together (an orbital time). Long-lived ones were wound up by the shear: a
// wound pattern's crossings at a fixed azimuth move OUTWARD, whichever way it turns -- seen from the side, where the
// line of sight runs along the gas at the disk's two ends, the clouds there spread away from the middle (left and
// right "turning opposite ways"). Short-lived, the structure goes round with the gas before the shear can wind it.
// And it is sheared about the middle of its life, when it is seen most (DiskZoneShearMid: turned by Omega (f - 1/2)
// life, not Omega f life): leaning ahead as it fades in, back as it fades out, the two equally weighted -- the
// crossings' drift in as much as out. (Sheared from its start, every generation leant back all through its life:
// seen from the side, the clouds toward the disk's two ends still drifted outward.) What is left at the very ends
// is real: there the line of sight grazes the thin outer edge, and the gas behind the grazing point (coming toward
// the end of the disk) shows through the gas in front of it (going away from it). (With the streaks along the orbits
// -- DiskTexUV -- it hardly shows: they slide along themselves. Lives of 1/4 - 1/2 orbit, needed while the features
// were round, made the outer disk fade in and out faster than it went round.)
// Zone 0 keeps its shear from the start: its generations live 1 - 2 orbits, and the inner disk seen from above
// would show leading spirals; from the side it is in front of the hole, where the gas crosses the view.
#define DISK_OUTER_LIFE_SHIFT 1.0
float DiskZoneLifeExp(float k)   // zone k's life = 2^e inner-edge lives
{
    return (k < 0.5) ? 0.0 : k - DISK_OUTER_LIFE_SHIFT;
}
float DiskZoneShearMid(float k)  // the generation phase at which zone k's structure is unsheared
{
    return (k < 0.5) ? 0.0 : 0.5;
}
// zone k's generations at radius rT (its clock G x 2^-e): (phase of A, phase of B = A + 1/2), their weights, their
// texture offsets (generation ids periodic in the clock's wrap, 1024 x 2^-e)
void DiskGenZone(float2 genPhase, float k, float e, float dGen, float rT,
                 out float fA, out float fB, out float wA, out float wB, out float2 offA, out float2 offB)
{
    float m = exp2(-e);   // (a power of 2: exact)
    float gInt, gA;
    if (m <= 1.0)
    {
        gInt = floor(genPhase.y * m);
        gA = (genPhase.y - gInt / m + genPhase.x + dGen) * m;
    }
    else
    {
        gInt = genPhase.y * m;
        gA = (genPhase.x + dGen) * m;
    }
    gA += (k < 0.5) ? 0.15 * rT : 0.0;
    float gB = gA + 0.5;
    fA = frac(gA);
    fB = frac(gB);
    wA = 1.0 - abs(2.0 * fA - 1.0);
    wB = 1.0 - wA;
    int idMask = (int) (1024.0 * m + 0.5) - 1;
    float salt = 97.0 * k;
    offA = DiskGenOffset((float) ((int) (gInt + floor(gA)) & idMask) + salt);
    offB = DiskGenOffset((float) ((int) (gInt + floor(gB)) & idMask) + 512.0 + salt);
}

// Disk turbulence pattern (the generations above). lod: noise mip (0 = finest, as before). The texture mean comes
// from the CPU.
float DiskTurbulence(float rT, float phiT, float OmegaT, float OmegaIn, float rIsco, float drdtT, float life,
                     float2 genPhase, float dGen, float DiskRotationOffset, float lod,
                     Texture2D _Noise, SamplerState sampler_Noise)
{
    float nMean = _BHNoiseMean;
    float zc = DiskLifeZone(OmegaT, OmegaIn);
    float k0 = floor(zc);
    float wHi = smoothstep(0.8, 1.0, zc - k0);
    float baseU = (phiT - DiskRotationOffset) * (1.0 / TWO_PI) + 0.5;
    float acc = 0.0, wz2 = 0.0;
    [loop]
    for (int j = 0; j < 2; j++)
    {
        float wz = (j == 0) ? 1.0 - wHi : wHi;
        [branch]
        if (wz <= 0.0)
            continue;
        float k = k0 + (float) j;
        float e = DiskZoneLifeExp(k);
        float lifeK = life * exp2(e);
        float creepK = min(lifeK, life);   // (the drift: no further per generation than at the inner edge)
        float fA, fB, wA, wB;
        float2 offA, offB;
        DiskGenZone(genPhase, k, e, dGen, rT, fA, fB, wA, wB, offA, offB);
        float fMid = DiskZoneShearMid(k);
        float turnsA = OmegaT * (fA - fMid) * lifeK * (1.0 / TWO_PI);
        float turnsB = OmegaT * (fB - fMid) * lifeK * (1.0 / TWO_PI);
        float2 uvA = DiskTexUV(frac(baseU - turnsA), DiskTexLnR(rT + drdtT * fA * creepK, rIsco), DISK_TEX_TILES2, offA);
        float2 uvB = DiskTexUV(frac(baseU - turnsB), DiskTexLnR(rT + drdtT * fB * creepK, rIsco), DISK_TEX_TILES2, offB);
        float nA = _Noise.SampleLevel(sampler_Noise, uvA, lod).r;
        float nB = _Noise.SampleLevel(sampler_Noise, uvB, lod).r;
        acc += wz * ((wA * (nA - nMean) + wB * (nB - nMean)) * rsqrt(wA * wA + wB * wB));
        wz2 += wz * wz;
    }
    return saturate(nMean + acc * rsqrt(wz2));
}

// Blackbody colour, linear sRGB, unit luminance, for T in 1000 - 40000 K.
float3 BlackbodyColor(float T)
{
    float t = (log10(clamp(T, 1000.0, 40000.0)) - 3.8) * 1.25;
    float r = ((((((-0.318020 * t + 0.307137) * t + 0.483045) * t - 1.126463) * t + 1.291352) * t - 0.997409) * t + 1.059097);
    float g = ((((((0.126835 * t - 0.133110) * t - 0.214311) * t + 0.457298) * t - 0.342339) * t + 0.107313) * t + 0.982236);
    float b = ((((((-0.319972 * t + 0.414174) * t + 0.700556) * t - 1.212946) * t - 0.411255) * t + 1.873784) * t + 1.002553);
    return max(float3(r, g, b), 0.0);
}

// What the eye sees of a blackbody at T, per unit of its total (bolometric) emission, relative to one at Tref:
// [Lv(T) / T^4] / [Lv(Tref) / Tref^4], Lv = its photopic radiance (CIE 1931 y-bar). The disk's brightness follows
// the bolometric flux and BlackbodyColor has unit luminance: times this it is the visible light. Lv(T) ~
// exp(c1 / T + c2 / T^2) / (exp(theta / T) - 1), theta = 25893 K (~555 nm): within 1.7 % from 1000 K to 2e6 K. The
// visible share peaks at 6700 K (95 lm/W); much hotter gas (Rayleigh-Jeans) emits mostly UV, its visible light
// only ~ T; much cooler gas mostly infrared (Wien). By total power a hot inner disk outshines the rest by r^-3: a
// 26000 K disk (a 1e11 solar-mass hole) is near black beyond ~15 Rs, its 5000 - 9000 K parts 7 - 8x too dark for
// the eye. Normalised at the disk's peak temperature (its brightness there unchanged); w = the hole's weighting
// (_BHDiskLight.x: 0 = total power, 1 = visible light).
float BlackbodyVisibleShare(float T)
{
    float iT = 1.0 / clamp(T, 500.0, 1.0e7);
    float iT2 = iT * iT;
    return exp(iT * (-478.07 + 1.53632e6 * iT)) * (iT2 * iT2) / max(exp(25893.4 * iT) - 1.0, 1e-30);
}
float DiskVisible(float T, float Tref, int h)
{
    float w = _BHDiskLight[h].x;
    [branch]
    if (w <= 0.0)
        return 1.0;
    return exp2(w * log2(max(BlackbodyVisibleShare(T) / max(BlackbodyVisibleShare(Tref), 1e-38), 1e-30)));
}

// Thin-disk flux relative to its peak: F ~ r^-3 (1 - c sqrt(rIsco / r)), c = 1 - stress.
// fluxNorm = 1 / F(peak), from the CPU.
float DiskFluxRel(float r, float rIsco, float diskISCOStress, float fluxNorm)
{
    float c = 1.0 - diskISCOStress;
    float x = max(r / rIsco, 1.0);
    float f = (1.0 - c * rsqrt(x)) / (x * x * x);
    float inside = saturate(r / rIsco);
    return f * fluxNorm * inside * inside;
}

float DiskWrapPi(float x)
{
    return x - TWO_PI * round(x * (1.0 / TWO_PI));
}

// x^k for the disk's appearance exponents (uniform per hole): the default k = 1 without the exp2 / log2 pair
// (two of them per sample in the volume, per crossing in the sheet)
float DiskPowU(float x, float k)
{
    [branch]
    if (k == 1.0)
        return x;
    return exp2(k * log2(x));
}

float DiskHotSpots(int h, float r, float phi)
{
    float2 bnd = _BHHotSpotBounds[h].xy;
    if (r < bnd.x || r > bnd.y)
        return 0.0;
    float spot = 0.0;
    [unroll]
    for (int i = 0; i < DISK_HOTSPOTS_MAX; i++)
    {
        float4 s = _BHHotSpots[h * DISK_HOTSPOTS_MAX + i];
        [branch]
        if (s.z <= 0.0)
            continue; // (unused / faded-out slot: C# writes amplitude 0)
        float dr = (r - s.x) / s.w;
        float dph = DiskWrapPi(phi - s.y) * r / (2.5 * s.w);
        spot += s.z * exp(-(dr * dr + dph * dph));
    }
    return spot;
}
// A hole's own disk at one crossing: emission (already times opacity), opacity, g.
void ShadeOwnDiskCrossing(
    float r, float phi, float xiPhys, int crossingIndex, float aPhys, float keplerFactor,
    float diskOuterRadius, HoleDisk hd,
    Texture2D _Noise, SamplerState sampler_Noise,
    float DiskRotationOffset, float2 genPhase,
    bool plungeOn, float photonPr,
    float4 ap1, float4 ap2, float noiseLod,
    out float3 em, out float op, out float gOut)
{
    em = float3(0, 0, 0);
    op = 0.0;
    gOut = -1.0;
    float rIsco = hd.rIsco;

    float diskSpan = diskOuterRadius - rIsco;
    float outerFade = 1.0 - smoothstep(diskOuterRadius - diskSpan * 0.4, diskOuterRadius, r);
    float orderFade = outerFade * exp(-abs(crossingIndex) * ap2.w) * (1.0 - _BHDiskCut[hd.h].x);
    float cutR = _BHDiskCut[hd.h].y;
    if (cutR > 0.0)
        orderFade *= smoothstep(0.5 * cutR, 1.1 * cutR, r);
    if (orderFade < 0.001)
        return;

    float orbitSign = (keplerFactor < 0.0) ? -1.0 : 1.0;
    bool plunging = plungeOn && (r < rIsco);

    float Omega, urMag, rTex, phiTex, OmegaTex, drdtTex, dGen;
    [branch]
    if (plunging)
    {
        float PhiS, TS;
        KerrPlungeStreamLUT(hd, r, aPhys, orbitSign, Omega, urMag, PhiS, TS);
        rTex = rIsco;
        phiTex = phi - PhiS;
        OmegaTex = hd.OmegaIn;
        drdtTex = hd.drdtIsco;
        dGen = -TS / hd.life;
    }
    else
    {
        Omega = DiskOmega(r, aPhys, keplerFactor);
        urMag = KERR_DISK_DRIFT * sqrt(rIsco / max(r, rIsco));
        rTex = r;
        phiTex = phi;
        OmegaTex = Omega;
        drdtTex = 0.0;
        dGen = 0.0;
    }

    float ut;
    float g = KerrFlowRedshift(r, xiPhys, aPhys, Omega, urMag, photonPr, ut);
    if (g < 0.0)
        return;
    gOut = g;
    if (!plunging)
        drdtTex = urMag / max(ut, 1e-4);

    const float TAU_DISK = 4.60517;
    float tau = TAU_DISK;
    [branch]
    if (plunging)
    {
        float uFall = sqrt(max(urMag * urMag - KERR_DISK_DRIFT * KERR_DISK_DRIFT, 0.0));
        float f = KERR_ISCO_INFLOW * rsqrt(KERR_ISCO_INFLOW * KERR_ISCO_INFLOW + uFall * uFall);
        tau *= (rIsco / r) * f * lerp(KERR_PLUNGE_DENSITY, 1.0, f);
    }
    else if (!plungeOn)
        tau *= smoothstep(rIsco, rIsco + diskSpan * 0.02, r);
    float diskOpacity = (1.0 - exp(-tau)) * orderFade;
    if (diskOpacity < 0.001)
        return;
    [branch]
    if (g_bhOpacityOnly)
    {
        op = diskOpacity; // em stays 0; opacity doesn't depend on the colour terms below
        return;
    }

    float turbulence = (crossingIndex < 2)
        ? DiskTurbulence(rTex, phiTex, OmegaTex, hd.OmegaIn, rIsco, drdtTex, hd.life,
                         genPhase, dGen, DiskRotationOffset, noiseLod,
                         _Noise, sampler_Noise)
        : _BHNoiseMean;
    float innerMask = plungeOn ? 1.0 : smoothstep(rIsco, rIsco + 0.5, r);
    float edgeMask = innerMask * smoothstep(diskOuterRadius, diskOuterRadius - 0.5, r);

    float flux = DiskFluxRel(r, rIsco, ap2.y, hd.fluxNorm);
    float clump = (turbulence - 0.5) * 2.0 * ap2.z;
    float density = max(1.0 + clump, 0.05) * edgeMask;
    float Temit = ap1.x * sqrt(sqrt(flux)) * (1.0 + 0.12 * clump);

    float spot = DiskHotSpots(hd.h, r, phi);
    density += 1.5 * spot * edgeMask;
    Temit *= 1.0 + 0.35 * spot;

    float Tobs = Temit * lerp(1.0, g, ap1.y);
    float g2 = max(g, 1e-4) * max(g, 1e-4);
    float brightness = ap2.x * DiskPowU(max(flux, 1e-6), ap1.w) * DiskPowU(g2 * g2, ap1.z) * (1.0 + spot);

    float Dl = r * r - 2.0 * KERR_M * r + aPhys * aPhys;
    float Xl = r * r + aPhys * aPhys - aPhys * xiPhys;
    float etaL = (Xl * Xl - photonPr * photonPr * Dl * Dl) / max(Dl, 1e-6)
               - (xiPhys - aPhys) * (xiPhys - aPhys);
    float mu = saturate(g * sqrt(max(etaL, 0.0)) / r);
    float limb = (1.0 + 2.06 * mu) / 2.03;

    float3 col = BlackbodyColor(Tobs) * (brightness * density * limb * DiskVisible(Tobs, ap1.x, hd.h));
    float4 tint = _BHDiskTint[hd.h];
    col = lerp(col, dot(col, float3(0.2126, 0.7152, 0.0722)) * tint.rgb, tint.w);

    em = col * diskOpacity;
    op = diskOpacity;
}

// One disk crossing of an exact trace (20 arguments, unchanged; no hand-over layer any more).
void ShadeKerrDiskCrossing(
    float r, float phi, float xiPhys, int crossingIndex, float aPhys, float keplerFactor,
    float diskOuterRadius, HoleDisk hd,
    Texture2D _Noise, SamplerState sampler_Noise,
    float DiskRotationOffset, float2 genPhase,
    bool plungeOn, float photonPr,
    float4 ap1, float4 ap2, float noiseLod,
    inout float3 accumulatedColor, inout float transmittance,
    inout float outFirstDopplerG)
{
    float3 em;
    float op, g;
    ShadeOwnDiskCrossing(r, phi, xiPhys, crossingIndex, aPhys, keplerFactor, diskOuterRadius, hd,
                         _Noise, sampler_Noise, DiskRotationOffset, genPhase, plungeOn, photonPr,
                         ap1, ap2, noiseLod, em, op, g);
    if (op < 0.001)
        return;
    if (outFirstDopplerG <= 0.0)
        outFirstDopplerG = g;
    accumulatedColor += em * transmittance;
    transmittance *= 1.0 - op;
}

// ------------------------------------------------------------------
// Radial (frame-dragging) azimuth, segment form.
//   a (r^2 + a^2 - a xi) / Delta = a + a (2 M r - a xi) / Delta
// The constant part integrates exactly to a * lambda. Only the remainder (which
// falls off like 1/r) goes through quadrature, so short segments between
// crossings need few nodes. Returns the remainder integral over [s0, s1]
// (progressive phase), in radians.
// ------------------------------------------------------------------
float RadialPhiRemGL4(RadialMotion rm, float a, float xi, float s0, float s1)
{
    float h = 0.5 * (s1 - s0);
    float mid = 0.5 * (s1 + s0);
    float total = 0.0;
    [unroll]
    for (int i = 0; i < 4; i++)
    {
        float r = EvaluateR(rm, ProgressiveToNativePhase(rm, mid + h * GL4_NODES[i]));
        float Delta = r * r - 2.0 * KERR_M * r + a * a;
        total += GL4_WEIGHTS[i] * a * (2.0 * KERR_M * r - a * xi) / Delta;
    }
    return total * h * rm.lambdaScale * rm.K_r * rm.radialNorm;
}

float RadialPhiRemGL6(RadialMotion rm, float a, float xi, float s0, float s1)
{
    float h = 0.5 * (s1 - s0);
    float mid = 0.5 * (s1 + s0);
    float total = 0.0;
    [unroll]
    for (int i = 0; i < 6; i++)
    {
        float r = EvaluateR(rm, ProgressiveToNativePhase(rm, mid + h * GL6_NODES[i]));
        float Delta = r * r - 2.0 * KERR_M * r + a * a;
        total += GL6_WEIGHTS[i] * a * (2.0 * KERR_M * r - a * xi) / Delta;
    }
    return total * h * rm.lambdaScale * rm.K_r * rm.radialNorm;
}

// One crossing loop for plunging and escaping rays (13 arguments).
// dc: read-only disk inputs; acc: everything accumulated / handed back.
void AccumulateDiskCrossings(
    ThetaMotion tm, RadialMotion rad, inout PolarPhiCum pc, inout bool pcReady, float lambdaTotal,
    float phi, float xi, float eta, float a, DiskCtx dc,
    Texture2D _Noise, SamplerState sampler_Noise,
    inout CrossingAcc acc)
{
    // The trace runs in Kerr(-a); the physical photon's xi is -xi.
    float xiPhys = -xi;

    if (tm.isVortical || rad.numRealRoots == 1)
        return;

    float uPerLambda = tm.kTheta / (4.0 * max(tm.K_theta, 1e-5));
    float CR = rad.lambdaScale * rad.K_r * rad.radialNorm; // dlambda / ds
    float C = max(CR, 1e-8);
    float emitLower = dc.plungeOn ? dc.rHorizon + 0.05 : dc.hd.rIsco;
    float uEnd = tm.s0 + lambdaTotal * uPerLambda;

    int k = (int) ceil((tm.s0 - 0.25) / 0.5);

    const int MAX_CROSSING_ITER = 16;
    [loop]
    for (int iter = 0; iter < MAX_CROSSING_ITER; iter++)
    {
        float uCross = 0.25 + 0.5 * (float) k;
        if (uCross > uEnd)
            break;

        float lambdaCross = (uCross - tm.s0) / uPerLambda;
        float sCross = lambdaCross / C;
        if (sCross < 0.0 || sCross > rad.deltaS_total)
            break;

        float rCross = EvaluateR(rad, ProgressiveToNativePhase(rad, sCross));
        rCross = clamp(rCross, dc.rHorizon + 0.05, 1e6);

        if (acc.firstRCross < 0.0)
            acc.firstRCross = rCross;

        bool afterTurn = rad.hasTurning && (sCross > abs(rad.sTurn - rad.sStart));
        bool finalLeg = !rad.hasTurning || afterTurn;
        bool rIncreasing = rad.innerRegion ? (rad.hasTurning && !afterTurn)
                                           : (rad.hasTurning ? afterTurn : !rad.isInward);
        if (finalLeg && rIncreasing && rCross > dc.diskOutNorm)
            break;
        if (finalLeg && !rIncreasing && rCross < emitLower)
            break;

        if (rCross >= emitLower && rCross <= dc.diskOutNorm &&
            KerrDelta(rCross, a) > 0.02 && acc.crossingCount < KERR_MAX_CROSSINGS)
        {
            [branch]
            if (!pcReady)
            {
                pc = MakePolarPhiCum(tm, xi, a);
                pcReady = true;
            }
            float phiCross = phi + PolarPhiToEquator(pc, k);
            [branch]
            if (a != 0.0)
            {
#if KERR_RADPHI_FAST
                acc.radPhiRem += RadialPhiRemGL4(rad, a, xi, acc.radS, sCross);
                acc.radS = sCross;
                phiCross += acc.radPhiRem + a * sCross * CR;
#else
                phiCross += RadialPhiPartial(rad, a, xi, sCross);
#endif
            }

            // Physical photon radial momentum at the crossing
            float Dc = KerrDelta(rCross, a);
            float Xc = rCross * rCross + a * a - a * xi;
            float Rc = max(Xc * Xc - Dc * (eta + (xi - a) * (xi - a)), 0.0);
            float photonPr = (rIncreasing ? -1.0 : 1.0) * sqrt(Rc) / Dc;

            float3 accBefore = acc.color;
            float tBefore = acc.transmittance;

            // (noise at the trace's detail, as the volume: mip 0 aliased at reduced res -- the higher-order
            // images, which stay at that res, sparkled)
            ShadeKerrDiskCrossing(
                rCross, phiCross, xiPhys, acc.crossingCount, dc.aPhys, dc.keplerFactor,
                dc.diskOutNorm, dc.hd,
                _Noise, sampler_Noise, dc.rotOffset, dc.genPhase,
                dc.plungeOn, photonPr, dc.ap1, dc.ap2, g_bhDiskLod,
                acc.color, acc.transmittance, acc.firstDopplerG);

            // First crossing that actually shows (re-shaded at full res later)
            if (!acc.hasFirst && acc.transmittance < tBefore)
            {
                acc.hasFirst = true;
                acc.firstCross = float4(rCross, phiCross, xiPhys, photonPr);
                acc.firstIdx = (float) acc.crossingCount;
                acc.emFirst = acc.color - accBefore;
            }
        }

        acc.crossingCount++;
        if (acc.transmittance < 0.01 || acc.crossingCount >= KERR_MAX_CROSSINGS)
            break;
        k++;
    }
}

#include "Assets/Shaders/Includes/BlackHoleKerrVolume.hlsl"

// ------------------------------------------------------------------
// Weak-field sky direction for rays that never enter the analytic sphere.
// ------------------------------------------------------------------
float3 BlackHoleWeakFieldDirection(float3 rayDirWorld, float3 bhPositionCamRelative, float schwarzschildRadius)
{
    float3 c = -bhPositionCamRelative / max(schwarzschildRadius, 1e-4);
    float rc = length(c);
    float3 er = c / rc;
    float3 d = rayDirWorld;
    d += er * dot(d, er) * (rsqrt(max(1.0 - 1.0 / rc, 1e-6)) - 1.0);
    d = normalize(d);

    float tc = dot(c, d);
    float3 p = c - tc * d;
    float b = length(p);
    float f = 0.5 * (1.0 - tc / rc);
    float alpha = f * (2.0 / b + (15.0 * PI / 16.0) / (b * b));
    float sA, cA;
    sincos(alpha, sA, cA);
    return normalize(cA * d - sA * (p / b));
}

// ------------------------------------------------------------------
// Core trace (28 arguments, signature unchanged). holeIndex must be a valid
// hole (0..7): the per-hole disk constants are read from it.
// ------------------------------------------------------------------
// ==================================================================
// Relativistic jets (shared with BlackHoleLensingPassthrough.shader, which draws them on straight pieces of
// a ray). Inside a hole's exact trace they are sampled along the geodesic itself: r(lambda), cos theta(lambda)
// are closed form, so a jet near the hole is lensed exactly. Only the Mino-time windows where the ray
// comes near the spin axis (around its theta turning points) are sampled.
// Shape (M87-like): width ~ tan(opening) sqrt(z (z + z_c)) (parabolic, then conical); a narrow white-hot
// spine in a blue limb-brightened sheath; light per length ~ z^-0.9 (brightest where it is launched) with a
// soft base / end; knots (shocks) riding out along the spine. Doppler beamed per lobe: (delta / delta_side)^2.7.
// ==================================================================
float4 _BHJet[8];      // x strength (0 = none), y near-camera scale (passthrough), z knot clock, w knot visibility
float4 _BHJetParams;   // x tan(half opening angle), y Lorentz factor, z length (Rs), w base height (Rs)
float4 _BHJetColor;    // rgb (linear)
#define JET_ZC      40.0   // parabolic -> conical (Rs)
#define JET_SPINE   0.15   // spine width / beam width
#define JET_HOLLOW  0.6    // sheath: inner Gaussian / outer
#define JET_SPINE_W 0.65   // share of the light in the spine
#define JET_GAIN    6.0    // overall (the spine's base into HDR at Eddington ~0.1, the rest below)
#define JET_RGEO    60.0   // a separate hole: sampled along the geodesic inside this radius (Rs, at most its region)

float JetWidthRs(float zRs)
{
    return _BHJetParams.x * sqrt(zRs * (zRs + JET_ZC)) + 0.3;
}
float JetHash(float n) { return frac(sin(n * 127.1) * 43758.5453); }
float JetHashP(float n, float period) { return JetHash(n - period * floor(n / period)); }   // (n whole)
// Knots: shocks riding out with the flow, which speeds up over its first ~JET_ZACC Rs (as M87's does, from ~0.1 c
// in its collimation zone) toward the bulk speed beta c: v(z) = beta c z / (z + JET_ZACC). Launched one every
// P = JET_ZACC / (JET_KNOT_K beta) Rs/c (~55 at Lorentz 3 -- about an inner-edge orbit), they are spaced v P: ~ z
// near the base (log z, as before), ~ 1/3 JET_ZACC far out. The pattern is stationary in the flow's travel time
// s(z) = int dz / v -- in knots: u = JET_KNOT_K (ln(z / zMin) + (z - zMin) / JET_ZACC) - clock, the clock (C#, per
// hole) counting knots launched in the hole's own time, the simulation's time scaled (as the disk's clock), so the
// jets go at the speed the rest of the simulation does and stop with it. Two octaves of value noise, periodic in
// the clock's wrap (1000 knots).
#define JET_ZACC   150.0     // the flow's acceleration length (Rs; C#: BlackHoleManager.JetAccelRs)
#define JET_KNOT_K 2.8853901 // knots per e-fold of z near the base (2 / ln 2: two per doubling)
float JetKnots(float zr, float zMin, float sgn, float clock)
{
    float u = JET_KNOT_K * (log(zr) + (zr - 1.0) * zMin * (1.0 / JET_ZACC)) - clock + ((sgn > 0.0) ? 0.0 : 57.0);
    float i = floor(u);
    float f = u - i;
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float v = lerp(JetHashP(i, 1000.0), JetHashP(i + 1.0, 1000.0), f);
    float u2 = 2.7 * u + 0.37;
    float i2 = floor(u2);
    float f2 = u2 - i2;
    f2 = f2 * f2 * (3.0 - 2.0 * f2);
    v = 0.7 * v + 0.3 * lerp(JetHashP(i2 + 17.0, 2700.0), JetHashP(i2 + 18.0, 2700.0), f2);
    return 0.25 + 1.75 * v * v;
}
// light per length along the beam at height z (Rs, as zMin / zMax): (spine, sheath).
// knotAmt: 1 = the knots, 0 = their mean (looking along the beam they blur into it; and while they move too fast
// to follow, _BHJet.w); clock: the hole's knot clock (_BHJet.z)
float2 JetLine1(float z, float zMin, float zMax, float sgn, float knotAmt, float clock)
{
    float zr = max(z / zMin, 1.0);
    float shape = smoothstep(zMin, 2.5 * zMin, z) * (1.0 - smoothstep(0.5 * zMax, zMax, z));
    float kn = 0.75;
    [branch]
    if (knotAmt > 0.0)
        kn = lerp(0.75, JetKnots(zr, zMin, sgn, clock), knotAmt);
    float L = JET_GAIN * pow(zr, -0.9) * shape;
    return L * float2(JET_SPINE_W * kn, (1.0 - JET_SPINE_W) * (0.75 + 0.25 * kn));
}
// the pixel's footprint at the traced hole (Rs; set by the caller): the beam is blurred to it, so a thin
// spine stays a smooth line instead of crawling between pixels (the light is kept: Gaussians just widen)
static float g_jetFootRs = 0.0;
// A ray aimed (nearly) at a hole's centre, b < JET_LINE_B x max(a, 0.05) Rs on the straight line: its jets are
// drawn on that straight line by the caller, unfaded, and its trace samples none (jetRGeo = 0). These rays
// are near-radial (no r(lambda) map) or vortical (b < a) -- the window logic has no turning point to cross the
// beam at for them: they got no jet (a black dot in the jet's glow in front of the shadow) or a sparse sum.
// They plunge almost radially, so the straight line is their path to within a fraction of the beam.
#define JET_LINE_B 1.25
bool JetOnLine(float b, float rs, float spin)
{
    return b < JET_LINE_B * max(abs(clamp(spin, -0.998, 0.998)) * KERR_M, 0.05) * rs;
}
// the cross-sections (unit area each) at distance^2 rho2 from the axis, beam width sg, blurred by a
// footprint of variance f2: (spine, sheath)
float2 JetProfile(float rho2, float sg, float f2)
{
    float sg2 = sg * sg;
    float s22 = JET_HOLLOW * JET_HOLLOW * sg2;
    float ss2 = JET_SPINE * JET_SPINE * sg2 + f2;
    float ge = sg2 + f2, he = s22 + f2;
    float spine = exp(-0.5 * rho2 / ss2) / ss2;
    float sheath = (sg2 * exp(-0.5 * rho2 / ge) / ge - s22 * exp(-0.5 * rho2 / he) / he) / (sg2 - s22);
    return 0.159155 * float2(spine, sheath);
}
// their integrals along a straight line passing at distance^2 b2 (per unit length across)
float2 JetProfileProj(float b2, float sg, float f2)
{
    float sg2 = sg * sg;
    float s22 = JET_HOLLOW * JET_HOLLOW * sg2;
    float ss2 = JET_SPINE * JET_SPINE * sg2 + f2;
    float ge = sg2 + f2, he = s22 + f2;
    float spine = exp(-0.5 * b2 / ss2) * rsqrt(ss2);
    float sheath = (sg2 * exp(-0.5 * b2 / ge) * rsqrt(ge) - s22 * exp(-0.5 * b2 / he) * rsqrt(he)) / (sg2 - s22);
    return 0.398942 * float2(spine, sheath);
}
// (spine, sheath) light -> colour: the spine white-hot, the sheath the jet colour
float3 JetRGB(float2 v)
{
    float3 col = _BHJetColor.rgb;
    float3 core = col + (max(col.x, max(col.y, col.z)) - col) * 0.75;
    return v.x * core + v.y * col;
}
// Doppler beaming of a lobe, cosLobe = traced direction . lobe direction (the light goes the other way)
float JetBoost(float cosLobe)
{
    float G = max(_BHJetParams.y, 1.0001);
    float beta = sqrt(1.0 - 1.0 / (G * G));
    return pow(1.0 / max(1.0 + beta * cosLobe, 1e-4), 2.7);
}

// The traced direction at Mino time lam (r, mu given) in the flat embedding (z = r mu,
// rho = sqrt(r^2 + a^2) sin theta, azimuth from dphi / dlambda): (d rho, rho d phi, dz) per dlambda.
float3 JetGeoVel(ThetaMotion tm, RadialMotion rad, float lam, float sProg, float uPL, float xi, float eta, float a,
                 float r, float mu)
{
    float D = KerrDelta(r, a);
    float X = r * r + a * a - a * xi;
    float Rr = max(X * X - D * (eta + (xi - a) * (xi - a)), 0.0);
    bool afterTurn = rad.hasTurning && (sProg > abs(rad.sTurn - rad.sStart));
    bool rUp = rad.innerRegion ? (rad.hasTurning && !afterTurn)
                               : (rad.hasTurning ? afterTurn : !rad.isInward);
    float dr = (rUp ? 1.0 : -1.0) * sqrt(Rr);
    float u01 = frac(tm.s0 + lam * uPL);
    float s2 = max(1.0 - mu * mu, 1e-6);
    float sinT = sqrt(s2);
    float rq = sqrt(r * r + a * a);
    // dtheta / dlambda = +-sqrt(Theta), Theta = eta + a^2 mu^2 - xi^2 mu^2 / sin^2 theta -- not dmu / sin theta: at the
    // pole both vanish (the expanded M(mu) = sin^2 Theta cancels to 0 in float), and a ray crossing right over the
    // axis lost its sideways motion: its tangent pointed ALONG the axis, so the spine was summed along the ray
    // instead of across it (x100 brighter: a hot line over each jet wherever a pixel looks exactly over the axis).
    float Th = max(eta + a * a * mu * mu - xi * xi * mu * mu / s2, 0.0);
    float dth = ((u01 < 0.5) ? 1.0 : -1.0) * sqrt(Th); // theta grows (cos theta falls) for u01 < 1/2
    return float3(sinT * (r / rq) * dr + rq * mu * dth,
                  rq * sinT * (a * X / max(D, 1e-4) + xi / s2 - a),
                  mu * dr - r * sinT * dth);
}

// The jets of hole h along the exact geodesic, where r < rGeo (Rs), faded out over rGeo / 2 .. rGeo where the
// straight pieces outside fade in (no seam where the two meet). front: before the first equator
// crossing (in front of the disk), back: after it (the caller dims it by the disk's transmittance).
// Each pass near the axis (a theta turning point): crossing the beam, the tangent line there integrated
// across it in closed form (like a straight piece: the thin spine needs no samples); running along it,
// 16 samples of the geodesic itself (blended by the angle to the axis, as the straight pieces do).
// backOn false: the disk is opaque (its transmittance exactly 0) -- what is behind it is not seen: the samples stop at
// the second crossing, where the share in front ends (the rest only went to back, times 0).
void AccumulateJetGeodesic(ThetaMotion tm, RadialMotion rad, float lambdaTotal, float xi, float eta, float a,
                           int h, float rGeo, float rHorizon, float rStart, bool backOn, out float3 front, out float3 back)
{
    front = float3(0, 0, 0);
    back = float3(0, 0, 0);
    float2 fr2 = float2(0, 0), bk2 = float2(0, 0);
    if (rad.numRealRoots == 1 || tm.isVortical)
        return; // (aimed at the centre: no r(lambda) map / no turning point; the caller's -- JetOnLine)
    float zMin = _BHJetParams.w, zMax = _BHJetParams.z;
    float knotVis = _BHJet[h].w, knotClock = _BHJet[h].z;
    // the jets' widest angle (at the base), out to where they are dark (6 widths, as the straight pieces'
    // window, and the pixel's blur): a narrower cut left a hard edge where its light was still visible
    float muWin = cos(atan(6.0 * (JetWidthRs(zMin) + g_jetFootRs) / zMin));
    float maxCos = sqrt(saturate(tm.uPlus));
    if (maxCos < muWin)
        return; // never near the axis: most rays
    // ...and for a ray that stays out at r >= rLeast (its periapsis, or where it starts), that cone is too wide:
    // inside it z = r |cos theta| >= rLeast muWin, where the beam's angle from the hole, (6 widths + blur) / z,
    // is smaller still -- a cone through that height bounds every sample the beam test below keeps. (The base's
    // 45 deg took nearly every ray of a view from above the disk, 14 samples each, ~80 % thrown away.)
    [branch]
    if (!rad.innerRegion && !rad.isCaptured)
    {
        float rLeast = rad.hasTurning ? rad.r1 : rStart;
        float zL = max(0.5 * zMin, rLeast * muWin);
        muWin = max(muWin, cos(atan(6.0 * (JetWidthRs(zL) + g_jetFootRs) / zL)));
        if (maxCos < muWin)
            return;
    }
    float Cl = max(rad.lambdaScale * rad.K_r * rad.radialNorm, 1e-8); // dlambda / ds
    float uPL = tm.kTheta / (4.0 * max(tm.K_theta, 1e-5));
    // the samples only where the ray is inside rGeo (the exact inverse r -> radial phase): beyond it every
    // sample was thrown away after its fetches, often most of them (a camera outside rGeo, rays near the axis)
    float2 rWin = RadialWindowBelow(rad, rGeo, rStart) * Cl;
    rWin.y = min(rWin.y, lambdaTotal);
    float lamX = tm.isVortical ? 3.0e38 : (0.25 + 0.5 * ceil((tm.s0 - 0.25) / 0.5) - tm.s0) / uPL;
    // (a crossing at the camera itself -- camera in the disk's plane -- hides nothing; one clear of it does,
    // weighted in as the disk's image order is: no flip on a float rounding)
    float lamX2 = lamX + 0.5 / uPL;
    float wX = tm.isVortical ? 1.0 : saturate(lamX * uPL / max(ThetaPhaseHalfWidth(tm, KERR_ORDER_MU_EPS), 1e-6));
    float cT = muWin / maxCos;
    float hwU = tm.isVortical ? 1e9 : ForwardPhaseCS(tm.ep, cT, sqrt(saturate(1.0 - cT * cT)));
    float k0 = ceil((tm.s0 - hwU) / 0.5);
    float rEmit = rHorizon * 1.05;
    // (the samples run forward in Mino time: windows in order, legs in order, each leg's samples in order)
    float lamEnd = backOn ? 3.0e38 : lamX2;
    bool past = false;

    [loop]
    for (int w = 0; w < 4; w++)
    {
        float la = 0.0, lb = lambdaTotal;
        float wAlong = 1.0;
        float knotA = 0.0;
        if (!tm.isVortical)
        {
            float lamT = (0.5 * (k0 + (float) w) - tm.s0) / uPL; // a theta turning point
            float hl = hwU / uPL;
            if (lamT - hl >= lambdaTotal || max(lamT - hl, 0.0) >= lamEnd)
                break;
            la = max(lamT - hl, 0.0);
            lb = min(lamT + hl, lambdaTotal);
            // the closed-form crossing needs it inside the trace: weighted in as it gets there (no switch)
            float inside = smoothstep(0.0, hl, lamT) * smoothstep(0.0, hl, lambdaTotal - lamT);
            [branch]
            if (inside > 0.0)
            {
                // across: the tangent line at the turning point, closest approach to the axis in closed form
                float sT = lamT / Cl;
                float rT = EvaluateR(rad, ProgressiveToNativePhase(rad, sT));
                float muT = EvaluateCosTheta(tm, lamT);
                float3 v = JetGeoVel(tm, rad, lamT, sT, uPL, xi, eta, a, rT, muT);
                float iv = rsqrt(max(dot(v, v), 1e-20));
                float sgn = (muT >= 0.0) ? 1.0 : -1.0;
                float dz = sgn * v.z * iv;
                float A = max(1.0 - dz * dz, 1e-6);
                float sinP = sqrt(A);
                float wA = 1.0 - smoothstep(0.15, 0.35, sinP);
                wAlong = 1.0 - (1.0 - wA) * inside;   // (the samples' share)
                knotA = 1.0 - wA;
                float rhoT = sqrt((rT * rT + a * a) * max(1.0 - muT * muT, 0.0));
                float B = rhoT * v.x * iv;
                float tS = -B / A;
                float b2 = max(rhoT * rhoT - B * B / A, 0.0);
                float zC = sgn * rT * muT + tS * dz;
                float rC = sqrt(b2 + zC * zC);
                [branch]
                if (wAlong < 1.0 && rC < rGeo && rC > rEmit && zC > 0.5 * zMin && lamT < lamEnd)
                {
                    float2 c = JetLine1(zC, zMin, zMax, sgn, knotA * knotVis, knotClock) * JetProfileProj(b2, JetWidthRs(zC), g_jetFootRs * g_jetFootRs)
                             * ((1.0 - wAlong) / sinP * JetBoost(dz) * (1.0 - smoothstep(0.5 * rGeo, rGeo, rC)));
                    float fs = (lamT < lamX) ? 1.0 : ((lamT < lamX2) ? 1.0 - wX : 0.0);
                    fr2 += c * fs;
                    bk2 += c * (1.0 - fs);
                }
            }
        }
        else if (w > 0)
            break;
        la = max(la, rWin.x);
        lb = min(lb, rWin.y);
        if (lb <= la || wAlong <= 0.0)
            continue;
        // along: samples of the geodesic, evenly in r on each leg where r is monotonic (r = r1 + t^2, t even, on
        // a leg that touches the periapsis, where ds / dr is singular) -- not evenly in Mino time, whose steps
        // are Sigma ~ r^2 long in path (18 Rs at r = 17 vs 1 Rs at r = 4): the far part of a window had a sample
        // per beam width or less, and the light of rays crossing the jet's base without a turning point in the
        // trace (plunging into the shadow from above it) jumped 2x between neighbouring pixels. Same fetches per
        // sample: the radius -> phase inverse instead of phase -> radius. (The photon-orbit pocket has no such
        // inverse: evenly in Mino time there.)
        bool rSpace = !rad.innerRegion;
        float sa = la / Cl, sb = lb / Cl;
        float sTp = abs(rad.sTurn - rad.sStart); // progressive phase of the periapsis (hasTurning)
        bool twoLegs = rSpace && rad.hasTurning;
        [loop]
        for (int leg = 0; leg < 2; leg++)
        {
            if ((leg > 0 && !twoLegs) || past)
                break;
            float s0L = sa, s1L = sb;
            bool atTurn = false;
            if (twoLegs)
            {
                if (leg == 0) { s1L = min(sb, sTp); atTurn = (sTp <= sb); }
                else { s0L = max(sa, sTp); atTurn = true; }
            }
            if (s1L <= s0L)
                continue;
            int nS = twoLegs ? 8 : 16;
            float tA, tB;
            if (rSpace)
            {
                float rA = EvaluateR(rad, ProgressiveToNativePhase(rad, s0L));
                float rB = EvaluateR(rad, ProgressiveToNativePhase(rad, s1L));
                tA = atTurn ? sqrt(max(rA - rad.r1, 0.0)) : rA;
                tB = atTurn ? sqrt(max(rB - rad.r1, 0.0)) : rB;
            }
            else
            {
                tA = s0L * Cl;
                tB = s1L * Cl;
            }
            float dt = (tB - tA) / (float) nS;
            [loop]
            for (int i = 0; i < nS; i++)
            {
                float t = tA + ((float) i + 0.5) * dt;
                float r, lam, sProg, dl;
                [branch]
                if (rSpace)
                {
                    r = atTurn ? rad.r1 + t * t : t;
                    if (r > rGeo || r < rEmit)
                        continue;
                    float nat = RadialNativePhaseAt(rad, r);
                    sProg = (twoLegs && leg > 0) ? sTp + abs(nat - rad.sTurn) : abs(nat - rad.sStart);
                    lam = sProg * Cl;
                    if (lam >= lamEnd)
                    {
                        past = true;
                        break;
                    }
                    // dlambda / dt: 1 / sqrt(R) (t = r), 2 / sqrt(R / (r - r1)) (r = r1 + t^2); R from the roots
                    float Q = (rad.numRealRoots >= 4)
                        ? (r - rad.r2) * (r - rad.r3) * (r - rad.r4)
                        : (r - rad.r2) * ((r - rad.r3) * (r - rad.r3) + rad.r4 * rad.r4);
                    dl = abs(dt) * (atTurn ? 2.0 * rsqrt(max(Q, 1e-20)) : rsqrt(max(Q * (r - rad.r1), 1e-20)));
                }
                else
                {
                    lam = t;
                    if (lam >= lamEnd)
                    {
                        past = true;
                        break;
                    }
                    sProg = lam / Cl;
                    dl = abs(dt);
                    r = EvaluateR(rad, ProgressiveToNativePhase(rad, sProg));
                    if (r > rGeo || r < rEmit)
                        continue;
                }
                float mu = EvaluateCosTheta(tm, lam);
                float sgn = (mu >= 0.0) ? 1.0 : -1.0;
                float z = r * abs(mu);
                if (z < 0.5 * zMin || z > zMax)
                    continue;
                float rho2 = (r * r + a * a) * max(1.0 - mu * mu, 1e-6);
                // outside the beam (6 of its widest Gaussians, blurred by the largest step it can have): no light
                // (~90 % of the samples in views near the axis cone) -- skip the velocity and the profile
                float sgB = JetWidthRs(z);
                float dPath = (r * r + a * a * mu * mu) * dl;   // dl_path = Sigma dlambda
                if (rho2 > 36.0 * (sgB * sgB + g_jetFootRs * g_jetFootRs + 0.25 * dPath * dPath))
                    continue;
                float3 v = JetGeoVel(tm, rad, lam, sProg, uPL, xi, eta, a, r, mu);
                float iv = rsqrt(max(dot(v, v), 1e-20));
                float nz = v.z * iv;
                // the beam blurred across by the step between samples (the ray moves that much sideways
                // through it): a thin spine between two samples is then not lost, and does not flicker
                float sideStep = 0.5 * dPath * sqrt(saturate(1.0 - nz * nz));
                float2 j = JetLine1(z, zMin, zMax, sgn, knotA * knotVis, knotClock)
                         * JetProfile(rho2, sgB, g_jetFootRs * g_jetFootRs + sideStep * sideStep);
                [branch]
                if (j.x + j.y < 1e-7)
                    continue;
                float2 c = j * (wAlong * JetBoost(nz * sgn) * dPath * (1.0 - smoothstep(0.5 * rGeo, rGeo, r)));
                float fs = (lam < lamX) ? 1.0 : ((lam < lamX2) ? 1.0 - wX : 0.0);
                fr2 += c * fs;
                bk2 += c * (1.0 - fs);
            }
        }
        if (past)
            break;
    }
    front = JetRGB(fr2) * _BHJet[h].x;
    back = JetRGB(bk2) * _BHJet[h].x;
}

void KerrTraceCore(
    float3 rayDirWorld, float3 bhPositionCamRelative, float schwarzschildRadius,
    float diskInnerRadius, float diskOuterRadius, float escapeRadiusParam, float3 diskNormal,
    float spinRatio, float diskKeplerFactor,
    Texture2D _Noise, SamplerState sampler_Noise,
    float DiskRotationOffset, float2 diskGenPhase,
    float rOuter, bool physicalDir, bool diskOn,
    out float3 color, out float3 exitPos, out float3 exitDir,
    float4 diskAppearanceParamaters1, float4 diskAppearanceParamaters2,
    out bool didEscape, out float outTransmittance,
    out bool hasFirst, out float4 firstCross, out float firstIdx, out float3 emFirst, int holeIndex,
    float jetRGeo)
{
    hasFirst = false;
    firstCross = float4(0, 0, 0, 0);
    firstIdx = 0.0;
    emFirst = float3(0, 0, 0);
    // Time reversal: tracing backward in Kerr(a) == forward along +rayDir in Kerr(-a).
    float aPhys = clamp(spinRatio, -0.998, 0.998) * KERR_M;
    float a = -aPhys;
    float rHorizon = KERR_M + sqrt(max(KERR_M * KERR_M - a * a, 0.0));
    float3 basisX, basisY, basisZ;
    BuildSpinBasis(diskNormal, basisX, basisY, basisZ);
    float invRS = 1.0 / max(schwarzschildRadius, 1e-4);
    float3 camPosWorldRel = -bhPositionCamRelative * invRS;
    float3 camLocal = float3(
        dot(camPosWorldRel, basisX),
        dot(camPosWorldRel, basisY),
        dot(camPosWorldRel, basisZ));
    float3 rayLocal = float3(
        dot(rayDirWorld, basisX),
        dot(rayDirWorld, basisY),
        dot(rayDirWorld, basisZ));

    float3 camDir = normalize(camPosWorldRel);
    if (dot(camDir, rayDirWorld) > 0.9999 && length(camLocal) > escapeRadiusParam * 2.0)
    {
        exitPos = float3(0, 0, 0);
        exitDir = rayDirWorld;
        didEscape = true;
        outTransmittance = 1.0;
        color = float3(0, 0, 0);
        return;
    }

    float xi, eta, E, r, cosTheta, sinTheta, phi;
    float n_r, n_theta, n_phi;
    LocalDirToConstantsRegular(camLocal, rayLocal, a, physicalDir,
                               xi, eta, E, r, cosTheta, sinTheta, phi,
                               n_r, n_theta, n_phi);

    ThetaMotion thetaMotion = SolveThetaMotion(a, xi, eta, cosTheta, n_theta, r, E);

    RadialMotion rad;
    // sqrt(R(r)) at the start from its local radial direction: dr/dlambda = Delta p_r, p_r = n_r sqrt(Sigma / Delta) / E
    float sqrtR0 = abs(n_r) * sqrt(max((r * r + a * a * cosTheta * cosTheta) * KerrDelta(r, a), 0.0)) / max(abs(E), 1e-6);
    bool setupSuccess = SetupRadialMotion(a, KERR_M, xi, eta, r, escapeRadiusParam, n_r, rOuter, sqrtR0, rad);
    if (!setupSuccess)
    {
        didEscape = false;
        outTransmittance = 1.0;
        exitPos = camPosWorldRel;
        exitDir = float3(0, 0, 0);
        color = float3(0, 0, 0);
        return;
    }

    bool willPlunge = rad.isCaptured;
    float lambdaPhysicalTotal = rad.deltaS_total * rad.lambdaScale * rad.K_r * rad.radialNorm;
    [branch]
    if (g_lamStop > 0.0 && g_lamStop < lambdaPhysicalTotal)
    {
        rad.deltaS_total *= g_lamStop / lambdaPhysicalTotal;
        lambdaPhysicalTotal = g_lamStop;
    }

    // Disk inputs: inner edge = ISCO of the orbit direction, from the CPU.
    DiskCtx dc;
    dc.hd = LoadHoleDisk(holeIndex);
    dc.aPhys = aPhys;
    dc.keplerFactor = diskKeplerFactor;
    dc.plungeOn = (KERR_DISK_PLUNGING != 0);
    dc.diskOutNorm = max(diskOuterRadius, dc.hd.rIsco * 1.05);
    dc.rHorizon = rHorizon;
    dc.rotOffset = DiskRotationOffset;
    dc.genPhase = diskGenPhase;
    dc.ap1 = diskAppearanceParamaters1;
    dc.ap2 = diskAppearanceParamaters2;

    CrossingAcc acc = MakeCrossingAcc();
    acc.crossingCount = (int) (g_orderBase + 0.5);
    // the order the next equator crossing after this trace would have (a continued trace starts there)
    g_orderOut = g_orderBase;
    [branch]
    if (!thetaMotion.isVortical)
    {
        float uPLo = thetaMotion.kTheta / (4.0 * max(thetaMotion.K_theta, 1e-5));
        int kF = (int) ceil((thetaMotion.s0 - 0.25) / 0.5);
        int kL = (int) floor((thetaMotion.s0 + lambdaPhysicalTotal * uPLo - 0.25) / 0.5);
        float wF = saturate((0.25 + 0.5 * (float) kF - thetaMotion.s0)
                            / max(ThetaPhaseHalfWidth(thetaMotion, KERR_ORDER_MU_EPS), 1e-6));
        if (kL >= kF)
            g_orderOut += (float) (kL - kF) + wF;
    }

    PolarPhiCum polarCum = (PolarPhiCum) 0;
    bool polarReady = false;

    [branch]
    if (diskOn)
    {
#if defined(BH_VOLUMETRIC_DISK)
        AccumulateDiskVolume(thetaMotion, rad, polarCum, polarReady, lambdaPhysicalTotal,
                             phi, r, xi, eta, a, dc, _Noise, sampler_Noise, acc);
#else
        AccumulateDiskCrossings(thetaMotion, rad, polarCum, polarReady, lambdaPhysicalTotal,
                                phi, xi, eta, a, dc, _Noise, sampler_Noise, acc);
#endif
    }

    hasFirst = acc.hasFirst;
    firstCross = acc.firstCross;
    firstIdx = acc.firstIdx;
    emFirst = acc.emFirst;

    // jets along the geodesic (jetRGeo > 0): in front of the disk, or behind it. (Rays aimed at the centre:
    // the caller's, on the straight line -- JetOnLine; the near-radial / vortical rest is skipped inside.)
    float3 jetF = float3(0, 0, 0), jetB = float3(0, 0, 0);
#if defined(BH_JETS) // (keyword: on only while some hole has jets -- the code is large)
    [branch]
    if (jetRGeo > 0.0 && _BHJet[holeIndex].x > 0.0)
        AccumulateJetGeodesic(thetaMotion, rad, lambdaPhysicalTotal, xi, eta, a, holeIndex,
                              jetRGeo, rHorizon, r, acc.transmittance > 0.0, jetF, jetB);
#endif
    acc.color += jetF + jetB * acc.transmittance;

    [branch]
    if (g_lamStop > 0.0)
    {
        didEscape = true;          // (no end state: the caller goes on along its own straight piece)
        exitPos = camPosWorldRel;
        exitDir = rayDirWorld;
        outTransmittance = acc.transmittance;
        color = acc.color;
        return;
    }

    // Disk already opaque: nothing behind it can show.
    [branch]
    if (acc.transmittance < 0.01)
    {
        didEscape = false;
        exitPos = camPosWorldRel;
        exitDir = float3(0, 0, 0);
        outTransmittance = acc.transmittance;
        color = acc.color;
        return;
    }

    if (willPlunge)
    {
        didEscape = false;
        exitPos = camPosWorldRel;
        exitDir = float3(0, 0, 0);
        outTransmittance = acc.transmittance;
        color = acc.color;
        return;
    }

    // ------------------------------------------------------------------
    // Escaping rays only from here: total azimuth + exit direction
    // ------------------------------------------------------------------
    float totalDPhi = 0.0;
    [branch]
    if (a != 0.0)
    {
        if (rad.numRealRoots == 1)
            totalDPhi = rad.nearRadialDPhiTotal;
        else
        {
#if KERR_RADPHI_FAST
            // continue from the last crossing: only the remaining leg is integrated
            float rem = acc.radPhiRem + RadialPhiRemGL6(rad, a, xi, acc.radS, rad.deltaS_total);
            totalDPhi = rem + a * lambdaPhysicalTotal;
#else
            totalDPhi = RadialPhiPartial(rad, a, xi, rad.deltaS_total);
#endif
        }
    }

    [branch]
    if (!polarReady)
        polarCum = MakePolarPhiCum(thetaMotion, xi, a);
    float finalCosTheta, polarDPhi, thetaDirSign, thetaSn2;
    EvaluateThetaEnd(thetaMotion, polarCum, xi, a, lambdaPhysicalTotal, finalCosTheta, polarDPhi, thetaDirSign, thetaSn2);
    // Theta(mu) at the end from the theta phase, = sn^2 (a^2 u+ mu^2 + eta) / (1 - mu^2): the expanded
    // eta + a^2 mu^2 - xi^2 mu^2 / (1 - mu^2) cancels to float noise near a theta turning point (sqrt of it:
    // ~3e-4 rad of direction that jittered as the scene moved, at a hand-off stop)
    float ThetaEndStable = (thetaSn2 >= 0.0)
        ? thetaSn2 * (a * a * thetaMotion.uPlus * finalCosTheta * finalCosTheta + eta)
          / max(1.0 - finalCosTheta * finalCosTheta, 1e-8)
        : -1.0;

    totalDPhi += polarDPhi;
    totalDPhi = fmod(totalDPhi, TWO_PI);

    float sinFinalTheta = sqrt(saturate(1.0 - finalCosTheta * finalCosTheta));
    float finalPhiAccum = phi + totalDPhi;
    float sinP, cosP;
    sincos(finalPhiAccum, sinP, cosP);

    float3 dirLocalOut = float3(sinFinalTheta * cosP, sinFinalTheta * sinP, finalCosTheta);
    float3 exitDirWorld = normalize(
        dirLocalOut.x * basisX +
        dirLocalOut.y * basisY +
        dirLocalOut.z * basisZ);

    [branch]
    if (rOuter < 1e29)
    {
        // Hand-off state at r = rOuter (exact inverse of LocalDirToConstantsRegular
        // with physicalDir).
        float rE = rad.rEndUsed;
        float ctE = finalCosTheta;
        float stE = max(sinFinalTheta, 1e-4);
        float rhoE = sqrt(rE * rE + a * a);
        float3 posL = float3(rhoE * sinFinalTheta * cosP, rhoE * sinFinalTheta * sinP, rE * ctE);

        float SigmaE = rE * rE + a * a * ctE * ctE;
        float DeltaE = max(KerrDelta(rE, a), 1e-6);
        float AE = KerrAkerr(rE, stE, a);
        float Xr = rE * rE + a * a - a * xi;
        // R from the trace's own roots: near the turning point (a stop just past it) the expanded quartic
        // cancels to float noise -- a radial direction off by ~3e-4 rad that jittered as the scene moved
        float RE = max(Xr * Xr - DeltaE * (eta + (xi - a) * (xi - a)), 0.0);
        if (rad.numRealRoots == 4)
            RE = max((rE - rad.r1) * (rE - rad.r2) * (rE - rad.r3) * (rE - rad.r4), 0.0);
        else if (rad.numRealRoots == 2)
            RE = max((rE - rad.r1) * (rE - rad.r2) * ((rE - rad.r3) * (rE - rad.r3) + rad.r4 * rad.r4), 0.0);
        float ThE = (ThetaEndStable >= 0.0) ? ThetaEndStable
                  : max(eta + a * a * ctE * ctE - xi * xi * ctE * ctE / (stE * stE), 0.0);
        float alphaE = sqrt(DeltaE * SigmaE / AE);
        float omegaE = 2.0 * KERR_M * a * rE / AE;
        float Ezamo = (1.0 - omegaE * xi) / alphaE;
        float3 nZ = float3(sqrt(RE / (DeltaE * SigmaE)),
                           thetaDirSign * sqrt(ThE / SigmaE),
                           xi / (stE * sqrt(AE / SigmaE))) / Ezamo;
        nZ = normalize(nZ);

        float3 eR, eTh, ePh;
        float rr_, ct_, st_, ph_;
        OblateTetradRegular(posL, a, eR, eTh, ePh, rr_, ct_, st_, ph_);
        float3 dL = nZ.x * eR + nZ.y * eTh + nZ.z * ePh;

        color = acc.color;
        exitPos = bhPositionCamRelative + (posL.x * basisX + posL.y * basisY + posL.z * basisZ) * schwarzschildRadius;
        exitDir = normalize(dL.x * basisX + dL.y * basisY + dL.z * basisZ);
        didEscape = true;
        outTransmittance = acc.transmittance;
        return;
    }

    // Outgoing asymptote (for chaining through further black holes).
    float sinTs = max(sinFinalTheta, 1e-4);
    float ThetaInf = (ThetaEndStable >= 0.0) ? ThetaEndStable
                   : max(eta + a * a * finalCosTheta * finalCosTheta
                         - xi * xi * finalCosTheta * finalCosTheta / (sinTs * sinTs), 0.0);
    float3 eThetaL = float3(finalCosTheta * cosP, finalCosTheta * sinP, -sinFinalTheta);
    float3 ePhiL = float3(-sinP, cosP, 0.0);
    float3 bLocal = -(thetaDirSign * sqrt(ThetaInf)) * eThetaL - (xi / sinTs) * ePhiL;
    float3 bWorld = bLocal.x * basisX + bLocal.y * basisY + bLocal.z * basisZ;

    color = acc.color;
    exitPos = bhPositionCamRelative + bWorld * schwarzschildRadius;
    exitDir = exitDirWorld;
    didEscape = true;
    outTransmittance = acc.transmittance;
}
#endif