// BlackHoleKerrVolume.hlsl
// Volumetric accretion disk for the analytic Kerr trace (keyword BH_VOLUMETRIC_DISK).
// Included by BlackHoleKerrAnalytic.hlsl right before KerrTraceCore: it uses that file's
// motion solvers, LUT helpers and disk-shading pieces, and stands in for
// AccumulateDiskCrossings when the keyword is on.
//
// Thickness (realistic thin disk, Shakura & Sunyaev inner region):
//   The inner disk is held up by radiation pressure, which gives a half-thickness
//     H = H0 J(r) G(r),   H0 = (3/4) mdot / eta  [Rs],   J = 1 - c sqrt(rIsco / r)
//   mdot = Eddington ratio, eta = 1 - E_isco (radiative efficiency), c = 1 - ISCO stress.
//   H is nearly constant with radius, so H/r peaks at ~2.3-2.7 rIsco and falls like 1/r
//   outward; it pinches at the ISCO (zero torque). G = M r / (L^2 + a^2 (1 - E^2)) is the
//   relativistic vertical gravity of a circular Kerr orbit (Abramowicz, Lanza & Percival 1997).
//   A gas-pressure floor (H/r ~ 1e-3 to 1e-2) takes over far out; H/r is capped at VOL_H_CAP.
//   Inside the ISCO the plunging stream keeps the aspect ratio it has at the ISCO.
//   Outer edge: a rounded rim (full thickness, then an elliptical end VOL_RIM_LENGTH
//   half-thicknesses long). The gas darkens over the outer fade zone (outer 40 %, as the
//   sheet's fade); over its last part (VOL_THIN_ZONE) the gas also runs out (optical depth
//   falls log-linearly to VOL_TAU_EDGE), so the rim is a dark curve, not a dark wall.
//   The rest of the disk is fully opaque.
//   The same formula (without the thinning) runs on the CPU (BlackHoleGlobalManager.DiskAspect)
//   for the envelope.
//
// Vertical profile in q = z / H: flat core with Gaussian edges of width VOL_EDGE, normalised to
// the column of a top hat of half-width 1. The vertical integrals are closed form.
//
// Where to sample:
//  * Slab intervals: the envelope cone |cos theta| < VOL_Q_ZONE x max(H/r) contains the whole
//    disk; ONE F-LUT lookup gives the interval around every equator crossing.
//  * Radial window: the exact inverse r -> radial phase clips the rays to r <= outer edge.
//  * Steps: nSteps per slab thickness, measured by how fast the RAY crosses it (dmu/dlambda),
//    at most ~10 % of r of path. Grazing rays take long steps instead of tiny ones (fewer
//    samples than a fixed path step, and no budget overruns where the disk thins).
//    Above the gas the next step jumps, with the exact theta phase, to the top of the gas.
//
// Transfer, front to back (E = 1), per step between two sample points:
//   tau  = <kappa rho_core Sigma / g> dlambda <f(q)>,  <f(q)> integrated EXACTLY in q
//   I    = S0 (1 - e^-tau) + (S1 - S0) [(1 - e^-tau) / tau - e^-tau]   (source linear in tau)
// Source function: Eddington grey atmosphere, T^4 = 3/4 T_sheet^4 (tauV + 2/3), tauV = optical
// depth to the surface. tauV is the column above the point, but never more than the optical depth
// the ray itself has crossed since it entered the gas: a ray entering the thin rim from the side
// must not see the deep, hot interior (that was a bright dotted line along the rim). Recolouring
// only -- no extra samples.
// Azimuth: exact at each interval start, then trapezoid of dphi/dlambda between sample points.
// Structure: 3D turbulence (DiskNoise3D, a tileable noise volume sheared by the sheet's
// two-generation clock) read per sample -> lognormal density, clumpiness _BHVolume.x. Dense gas is
// more opaque and brighter (as the sheet's clumps). Above the core sits a turbulent atmosphere
// (real disks have magnetically supported coronae): exponential in height with scale _BHVolume2.y H,
// its own optical depth _BHVolume2.x, clumpier than the core, glowing like the disk surface. It gives
// the soft, wispy, see-through outer layers; at 0 the disk is just the core.

#ifndef BLACKHOLE_KERR_VOLUME_INCLUDED
#define BLACKHOLE_KERR_VOLUME_INCLUDED

// x: unused, y: vertical optical depth through the disk outside the ISCO,
// z: samples through the local disk thickness, w: sample budget per disk image  (C#)
float4 _BHVolume;
// per disk slot (C#): x = H0 (Rs, x thickness scale), y = gas-pressure floor of H/r,
//                     z = envelope: max H/r over the disk (x 1.03), w = unused
float4 _BHDiskThick[9];

// (g_bhDiskLod, the noise mip: BlackHoleKerrAnalytic.hlsl)
// a trace continued from where an earlier one of the same ray stopped (a hand-off): the image order its first
// equator crossing has (g_orderBase, in), and the order the next crossing after this trace would have (out)
static float g_orderBase = 0.0;
static float g_orderOut = 0.0;

#define VOL_EDGE   0.2         // edge width / H
#define VOL_CORE   0.7493372   // 1 - VOL_EDGE sqrt(pi/2): flat core half-height
#define VOL_TAIL   0.2506628   // VOL_EDGE sqrt(pi/2): column of one Gaussian edge
#define VOL_Q_CUT  1.5493372   // VOL_CORE + 4 VOL_EDGE: top of the gas (f < 4e-4)
#define VOL_Q_ZONE 1.70        // just above the gas: where the skip lands
#define VOL_H_CAP  0.3         // max H/r, = VolumeHCap in C#
#define VOL_TAU_EDGE 0.01      // optical depth the gas thins to at the outer edge
#define VOL_THIN_ZONE 0.3      // the gas thins out where the outer darkening is below this
#define VOL_RIM_LENGTH 10.0     // length of the rounded rim, in half-thicknesses
// Polar funnel: no gas within ~18-28 degrees of the spin axis (|cos theta| 0.88 -> 0.95). Thick,
// radiation-supported disks leave the axis empty (that is where jets run); without it a thick
// disk's atmosphere (H/r 0.3, ~4 H tall) reached the pole, where only some rays found it.
#define VOL_FUNNEL_MU0 0.88
#define VOL_FUNNEL_MU1 0.95

// erfc(x) for x >= 0, given e^(-x^2) (Abramowitz & Stegun 7.1.26, |error| < 1.5e-7)
float ErfcFromExp(float x, float expMinusX2)
{
    float t = 1.0 / (1.0 + 0.3275911 * x);
    float p = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    return p * expMinusX2;
}

// ------------------------------------------------------------------
// Vertical structure
// ------------------------------------------------------------------
// density / core density at q = z / H
float VolProfile(float q)
{
    float d = max(abs(q) - VOL_CORE, 0.0) * (1.0 / VOL_EDGE);
    return exp(-0.5 * d * d);
}

// column above |q| (integral of f from |q| to infinity), given f(q)
float VolColumnAbove(float q, float fq)
{
    float aq = abs(q);
    return (aq <= VOL_CORE) ? (VOL_CORE - aq) + VOL_TAIL
                            : VOL_TAIL * ErfcFromExp((aq - VOL_CORE) * (0.70710678 / VOL_EDGE), fq);
}

// outer fade of the sheet (1 inside, 0 at the outer edge)
float DiskOuterFade(float r, float rIsco, float rOut)
{
    float span = rOut - rIsco;
    return 1.0 - smoothstep(rOut - span * 0.4, rOut, r);
}

float VolFunnel(float mu)
{
    return 1.0 - smoothstep(VOL_FUNNEL_MU0, VOL_FUNNEL_MU1, abs(mu));
}

// H / r at r (Rs units) without the outer rim. Same formula as BlackHoleGlobalManager.DiskAspect.
float DiskAspectCore(float r, DiskCtx dc)
{
    float4 th = _BHDiskThick[dc.hd.h];
    float rIsco = dc.hd.rIsco;
    float rr = max(r, rIsco); // inside the ISCO: the aspect ratio at the ISCO
    float ae = ((dc.keplerFactor < 0.0) ? -1.0 : 1.0) * dc.aPhys; // spin in the orbit's sense
    float J = max(1.0 - (1.0 - dc.ap2.y) * sqrt(rIsco / rr), 0.0);

    // circular-orbit E, L at rr (Bardeen, Press & Teukolsky 1972) -> vertical gravity factor
    float sM = sqrt(KERR_M);
    float sr = sqrt(rr);
    float r15 = rr * sr;
    float den = sr * sqrt(sr) * sqrt(max(r15 - 3.0 * KERR_M * sr + 2.0 * ae * sM, 1e-8));
    float E = (r15 - 2.0 * KERR_M * sr + ae * sM) / den;
    float L = sM * (rr * rr - 2.0 * ae * sqrt(KERR_M * rr) + ae * ae) / den;
    float G = KERR_M * rr / max(L * L + ae * ae * (1.0 - E * E), 1e-6);

    float hRad = th.x * J * G / rr;
    return min(sqrt(hRad * hRad + th.y * th.y), VOL_H_CAP);
}

// outer edge: a rounded rim. Full thickness until VOL_RIM_LENGTH half-thicknesses before the
// edge rOut, then an elliptical end, sqrt(1 - x^2) (a curve, not a needle)
float DiskRim(float h, float r, float rOut)
{
    float rimLen = VOL_RIM_LENGTH * h * r;
    float x = (r - (rOut - rimLen)) / max(rimLen, 1e-6);
    if (x > 0.0)
        h *= sqrt(saturate(1.0 - x * x));
    return max(h, 1e-4); // > 0: q = mu / h stays finite
}

// H / r at r, with the outer rim
float DiskAspectAt(float r, DiskCtx dc)
{
    return DiskRim(DiskAspectCore(r, dc), r, dc.diskOutNorm);
}

// ------------------------------------------------------------------
// Kinematics / geometry helpers
// ------------------------------------------------------------------
// KerrFlowRedshift off the equator (full Kerr metric at cos theta = mu). Gas at height
// keeps the midplane's Omega(r) and |u^r| (shellular flow). Returns -1 where no such
// timelike flow exists (transparent).
float KerrFlowRedshiftTheta(float r, float mu, float xiPhys, float aPhys, float Omega, float urMag,
                            float photonPr, out float ut)
{
    float a2 = aPhys * aPhys;
    float s2 = 1.0 - mu * mu;
    float Sigma = r * r + a2 * mu * mu;
    float Delta = r * r - 2.0 * KERR_M * r + a2;
    float f2m = 2.0 * KERR_M * r / Sigma;
    float gtt = -(1.0 - f2m);
    float gtp = -f2m * aPhys * s2;
    float gpp = (r * r + a2 + f2m * a2 * s2) * s2;
    float norm = -(gtt + 2.0 * gtp * Omega + gpp * Omega * Omega);
    ut = 1.0;
    if (norm <= 0.0 || Delta <= 0.0)
        return -1.0;
    ut = sqrt((1.0 + (Sigma / Delta) * urMag * urMag) / norm);
    float pu = ut * (1.0 - Omega * xiPhys) + photonPr * urMag;
    return (pu > 0.0) ? clamp(1.0 / pu, 0.0, 3.0) : -1.0;
}

// Polar azimuth from lambda = 0 to lambda (ordinary motion). Same pieces as EvaluateThetaEnd.
float PolarPhiAtLambda(ThetaMotion tm, PolarPhiCum pc, float a, float lambda, float uPerLambda)
{
    float u = tm.s0 + lambda * uPerLambda;
    float k = floor(2.0 * u);
    float t = u - 0.5 * k;
    float arg = (t <= 0.25) ? t : 0.5 - t;
    float2 sncn = LutSnCnQuarter(tm.ep, arg);
    float chiC = atan2(pc.c * sncn.y, sncn.x); // pi/2 - chi (see PolarG)
    return k * pc.P + PolarPhiJChi(pc, a, chiC, t) - pc.I0;
}

// an equator crossing within |cos theta| < this of the camera is (partly) the camera's own: it does not make the
// images after it higher-order ones (the disk's image order, the jets in front of / behind the disk)
#define KERR_ORDER_MU_EPS 0.01

// Theta-phase distance from an equator crossing at which |cos theta| = muT (0: never below)
float ThetaPhaseHalfWidth(ThetaMotion tm, float muT)
{
    float cT = muT / max(sqrt(saturate(tm.uPlus)), 1e-8);
    if (cT >= 1.0)
        return 0.25;
    return 0.25 - ForwardPhaseCS(tm.ep, cT, sqrt(saturate(1.0 - cT * cT)));
}

// Native radial phase at r = R >= r1 (outside region): the maps SetupRadialMotion uses.
// 4 roots: phase falls from 0.25 (r1) as R grows. 2 roots: phase rises from 0 (r1) as R grows.
float RadialNativePhaseAt(RadialMotion rm, float R)
{
    [branch]
    if (rm.numRealRoots >= 4)
    {
        float den = max((rm.r1 - rm.r4) * (R - rm.r3), 1e-12);
        float sn = sqrt(saturate((rm.r1 - rm.r3) * (R - rm.r4) / den));
        float cn = sqrt(saturate((rm.r3 - rm.r4) * (R - rm.r1) / den));
        return ForwardPhaseCS(rm.ep, cn, sn);
    }
    float A = sqrt((rm.r1 - rm.r3) * (rm.r1 - rm.r3) + rm.r4 * rm.r4);
    float B = sqrt((rm.r2 - rm.r3) * (rm.r2 - rm.r3) + rm.r4 * rm.r4);
    float c = clamp(((A - B) * R + (rm.r1 * B - rm.r2 * A)) /
                    max((A + B) * R - (rm.r1 * B + rm.r2 * A), 1e-12), -1.0, 1.0);
    return ForwardPhaseCS(rm.ep, c, sqrt(saturate(1.0 - c * c)));
}

// Progressive radial phase window [x, y] in which r <= R (x >= y: never).
float2 RadialWindowBelow(RadialMotion rm, float R, float rStart)
{
    if (rm.innerRegion)
        return float2(0.0, rm.deltaS_total); // inside the photon-orbit pocket throughout
    bool startIn = (rStart <= R);

    [branch]
    if (!rm.hasTurning)
    {
        if (rm.isInward) // plunging, r falls monotonically
        {
            float sIn = startIn ? 0.0 : abs(RadialNativePhaseAt(rm, R) - rm.sStart);
            return float2(sIn, rm.deltaS_total);
        }
        if (!startIn) // outgoing and already beyond R
            return float2(1.0, 0.0);
        return float2(0.0, min(abs(RadialNativePhaseAt(rm, R) - rm.sStart), rm.deltaS_total));
    }

    // turning: in to r1, then out
    if (R <= rm.r1)
        return float2(1.0, 0.0);
    float ns = RadialNativePhaseAt(rm, R);
    float sIn = startIn ? 0.0 : abs(ns - rm.sStart);
    float sOut = abs(rm.sTurn - rm.sStart) + abs(ns - rm.sTurn);
    return float2(sIn, min(sOut, rm.deltaS_total));
}

// ------------------------------------------------------------------
// 3D turbulence and the atmosphere
// ------------------------------------------------------------------
// Tileable 3D noise (u = azimuth, v = radius, w = height), built by BlackHoleGlobalManager from
// DiskNoise3D.bytes. Mean 0.5, standard deviation 0.18.
Texture3D<float> _BHDiskNoise3D;
SamplerState sampler_BHDiskNoise3D;
// x = atmosphere optical depth (vertical, through both sides), y = its scale height (in H),
// z = noise height scale (texture tiles per H), w = atmosphere brightness  (C#)
float4 _BHVolume2;

// atmosphere scale height in H (guarded: unset globals must not divide by zero)
float VolAtmS()
{
    return max(_BHVolume2.y, 0.05);
}

// height in H above which there is no gas
float VolQTop()
{
    return (_BHVolume2.x > 0.0) ? max(VOL_Q_ZONE, 1.0 + 5.0 * VolAtmS()) : VOL_Q_ZONE;
}

// atmosphere profile in q: 1 inside the core, exp(-(|q| - 1) / s) above it
float VolAtmProfile(float q)
{
    return exp(-max(abs(q) - 1.0, 0.0) / VolAtmS());
}
// its integral from signed q to +infinity (total 2 (1 + s))
float VolAtmCumSigned(float q)
{
    float s = VolAtmS();
    float aq = abs(q);
    float c = (aq < 1.0) ? (1.0 - aq) + s : s * exp(-(aq - 1.0) / s);
    return (q >= 0.0) ? c : 2.0 * (1.0 + s) - c;
}

// Turbulence at a point of the gas: the sheet's generation clock and life zones (DiskTurbulence), with the
// 3D noise read at height w. Returns the noise in standard deviations (0 = mean).
float DiskNoise3D(float rT, float phiT, float OmegaT, float drdtT, float w, DiskCtx dc, float dGen)
{
    float life = dc.hd.life;
    float zc = DiskLifeZone(OmegaT, dc.hd.OmegaIn);
    float k0 = floor(zc);
    float wHi = smoothstep(0.8, 1.0, zc - k0);
    float baseU = (phiT - dc.rotOffset) * (1.0 / TWO_PI) + 0.5;
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
        DiskGenZone(dc.genPhase, k, e, dGen, rT, fA, fB, wA, wB, offA, offB);
        float fMid = DiskZoneShearMid(k);
        float turnsA = OmegaT * (fA - fMid) * lifeK * (1.0 / TWO_PI);
        float turnsB = OmegaT * (fB - fMid) * lifeK * (1.0 / TWO_PI);
        float3 uvwA = float3(DiskTexUV(frac(baseU - turnsA), DiskTexLnR(rT + drdtT * fA * creepK, dc.hd.rIsco), DISK_TEX_TILES3, offA),
                             w + offA.x + offA.y);
        float3 uvwB = float3(DiskTexUV(frac(baseU - turnsB), DiskTexLnR(rT + drdtT * fB * creepK, dc.hd.rIsco), DISK_TEX_TILES3, offB),
                             w + offB.x - offB.y);
        float nA = _BHDiskNoise3D.SampleLevel(sampler_BHDiskNoise3D, uvwA, g_bhDiskLod) - 0.5;
        float nB = _BHDiskNoise3D.SampleLevel(sampler_BHDiskNoise3D, uvwB, g_bhDiskLod) - 0.5;
        acc += wz * ((wA * nA + wB * nB) * rsqrt(wA * wA + wB * wB));
        wz2 += wz * wz;
    }
    return acc * rsqrt(wz2) * (1.0 / 0.18);
}

// ------------------------------------------------------------------
// Sample points
// ------------------------------------------------------------------
struct VolPoint
{
    float lam;      // Mino time along the trace
    float r, mu;    // radius, cos theta
    float h;        // H / r here
    float hJ;       // H / r without the rim: bounds the gas ahead (jumps above the gas)
    float q;        // height in half-thicknesses (signed)
    float colC;     // the core's profile integrated above |q| (VolColumnAbove), and the atmosphere's from q up
    float cumA;     // (VolAtmCumSigned): once per point -- each step's optical depth takes both ends' (they were
                    // computed twice, and the column once more for tauV)
    float dphi;     // dphi / dlambda
    float muDot;    // dmu / dlambda
    float phi;
    float photonPr; // physical photon p_r
    bool inDisk;    // radially inside the emitting disk
    bool sValid;    // there is gas here (core or atmosphere)
    float kA;       // core opacity per unit Mino time, kappa rho_core Sigma / g
    float kAa;      // atmosphere opacity per unit Mino time (times its profile)
    float tauV;     // vertical optical depth to the core's surface (column above)
    float3 S;       // core source function at tauV
    float3 SA;      // atmosphere source function
    float3 S1c;     // surface colour (edd = 1) for a unit amount of gas: S, SA and the depth
                    // recolouring are scalar multiples of it (one blackbody per sample)
    // what the surface colour needs
    float g, clump, spot, kS, flux0;
};

// Everything one ray's march needs, so the helpers stay short.
struct VolRay
{
    ThetaMotion tm;
    RadialMotion rad;
    float xi, eta, a, xiPhys;
    float C;          // dlambda / ds (radial)
    float uPerLambda; // dtheta-phase / dlambda
    float emitLower;
};

VolPoint VolGeometry(VolRay vr, float lam, DiskCtx dc)
{
    VolPoint p = (VolPoint) 0;
    p.lam = lam;
    float s = lam / vr.C;
    float a = vr.a, xi = vr.xi;
    p.r = EvaluateR(vr.rad, ProgressiveToNativePhase(vr.rad, s));
    p.mu = EvaluateCosTheta(vr.tm, lam);
    p.hJ = max(DiskAspectCore(p.r, dc), 1e-4);
    p.h = DiskRim(p.hJ, p.r, dc.diskOutNorm);
    p.q = p.mu / p.h;
    p.colC = VolColumnAbove(p.q, VolProfile(p.q));
    p.cumA = VolAtmCumSigned(p.q);

    float D = KerrDelta(p.r, a);
    float s2 = max(1.0 - p.mu * p.mu, 1e-4);
    p.dphi = a * (p.r * p.r + a * a - a * xi) / max(D, 1e-4) + xi / s2 - a;

    // |dmu/dlambda| = sqrt(eta - (xi^2 + eta - a^2) mu^2 - a^2 mu^4); falls for theta phase < 1/2
    float mu2 = p.mu * p.mu;
    float Mmu = vr.eta - (xi * xi + vr.eta - a * a) * mu2 - a * a * mu2 * mu2;
    float u01 = frac(vr.tm.s0 + lam * vr.uPerLambda);
    p.muDot = ((u01 < 0.5) ? -1.0 : 1.0) * sqrt(max(Mmu, 0.0));

    // physical photon radial momentum (same convention as the sheet)
    bool afterTurn = vr.rad.hasTurning && (s > abs(vr.rad.sTurn - vr.rad.sStart));
    bool rIncreasing = vr.rad.innerRegion ? (vr.rad.hasTurning && !afterTurn)
                                          : (vr.rad.hasTurning ? afterTurn : !vr.rad.isInward);
    float X = p.r * p.r + a * a - a * xi;
    float Rr = max(X * X - D * (vr.eta + (xi - a) * (xi - a)), 0.0);
    p.photonPr = (rIncreasing ? -1.0 : 1.0) * sqrt(Rr) / max(D, 1e-4);

    // (tolerance: intervals start ON the outer edge; a start rounded to just outside it would
    // take a long step straight through the rim -- speckle along the rim edge-on)
    p.inDisk = (p.r >= vr.emitLower) && (p.r <= dc.diskOutNorm * 1.001) && (D >= 0.02);
    return p;
}

// Observed source function at optical depth tauV below the surface (edd = 1: the surface value).
float3 VolSourceEdd(VolPoint p, float edd, float kS, DiskCtx dc)
{
    float flux = p.flux0 * edd;
    float Temit = dc.ap1.x * sqrt(sqrt(flux)) * (1.0 + 0.12 * p.clump) * (1.0 + 0.35 * p.spot);
    float Tobs = Temit * lerp(1.0, p.g, dc.ap1.y);
    float g2 = max(p.g, 1e-4) * max(p.g, 1e-4);
    float brightness = dc.ap2.x * DiskPowU(max(flux, 1e-6), dc.ap1.w) * DiskPowU(g2 * g2, dc.ap1.z)
                     * (1.0 + p.spot);
    float3 col = BlackbodyColor(Tobs) * (brightness * kS * DiskVisible(Tobs, dc.ap1.x, dc.hd.h));
    float4 tint = _BHDiskTint[dc.hd.h];
    return lerp(col, dot(col, float3(0.2126, 0.7152, 0.0722)) * tint.rgb, tint.w);
}
float3 VolSource(VolPoint p, float tauV, DiskCtx dc)
{
    float edd = 0.75 * (tauV + 2.0 / 3.0); // grey atmosphere: T^4 / T_sheet^4
    return p.S1c * (p.kS * DiskPowU(max(edd, 1e-6), dc.ap1.w));
}

// Gas at a sample point (p.phi must be set).
void VolShade(inout VolPoint p, VolRay vr, float order, DiskCtx dc,
              Texture2D _Noise, SamplerState sampler_Noise)
{
    p.kA = 0.0;
    p.kAa = 0.0;
    p.tauV = 0.0;
    p.sValid = false;
    p.S = float3(0, 0, 0);
    p.SA = float3(0, 0, 0);
    if (!p.inDisk)
        return;

    HoleDisk hd = dc.hd;
    float r = p.r;
    float rIsco = hd.rIsco;
    float rOut = dc.diskOutNorm;
    float diskSpan = rOut - rIsco;

    // fades, as the sheet: higher-order images, tides (the outer edge is handled below)
    float fade = exp(-order * dc.ap2.w) * (1.0 - _BHDiskCut[hd.h].x) * VolFunnel(p.mu);
    float cutR = _BHDiskCut[hd.h].y;
    if (cutR > 0.0)
        fade *= smoothstep(0.5 * cutR, 1.1 * cutR, r);
    if (fade < 0.001)
        return;
    float outer = DiskOuterFade(r, rIsco, rOut);

    float orbitSign = (dc.keplerFactor < 0.0) ? -1.0 : 1.0;
    bool plunging = dc.plungeOn && (r < rIsco);

    float Omega, urMag, rTex, phiTex, OmegaTex, drdtTex, dGen;
    [branch]
    if (plunging)
    {
        float PhiS, TS;
        KerrPlungeStreamLUT(hd, r, dc.aPhys, orbitSign, Omega, urMag, PhiS, TS);
        rTex = rIsco;
        phiTex = p.phi - PhiS;
        OmegaTex = hd.OmegaIn;
        drdtTex = hd.drdtIsco;
        dGen = -TS / hd.life;
    }
    else
    {
        Omega = DiskOmega(r, dc.aPhys, dc.keplerFactor);
        urMag = KERR_DISK_DRIFT * sqrt(rIsco / max(r, rIsco));
        rTex = r;
        phiTex = p.phi;
        OmegaTex = Omega;
        drdtTex = 0.0;
        dGen = 0.0;
    }

    float ut;
    float g = KerrFlowRedshiftTheta(r, p.mu, vr.xiPhys, dc.aPhys, Omega, urMag, p.photonPr, ut);
    if (g <= 0.0)
        return;
    if (!plunging)
        drdtTex = urMag / max(ut, 1e-4);

    // column optical depth (gas frame), thinning in the plunge like the sheet
    float thinPlunge = 1.0;
    [branch]
    if (plunging)
    {
        float uFall = sqrt(max(urMag * urMag - KERR_DISK_DRIFT * KERR_DISK_DRIFT, 0.0));
        float f = KERR_ISCO_INFLOW * rsqrt(KERR_ISCO_INFLOW * KERR_ISCO_INFLOW + uFall * uFall);
        thinPlunge = (rIsco / r) * f * lerp(KERR_PLUNGE_DENSITY, 1.0, f);
    }
    else if (!dc.plungeOn)
        thinPlunge = smoothstep(rIsco, rIsco + diskSpan * 0.02, r);
    float tauCol = _BHVolume.y * thinPlunge;

    // 3D turbulence: lognormal density, clumpier in the atmosphere (first two images only)
    float C = _BHVolume.x;
    float sN = 0.0;
    [branch]
    if (order < 1.5 && C > 0.0) // (a ternary would sample the texture either way)
        sN = DiskNoise3D(rTex, phiTex, OmegaTex, drdtTex, p.q * _BHVolume2.z, dc, dGen);
    float dens = exp(C * sN - 0.5 * C * C);
    float CA = 1.75 * C;
    float densA = exp(CA * sN - 0.5 * CA * CA);
    float spot = DiskHotSpots(hd.h, r, p.phi);
    float innerMask = dc.plungeOn ? 1.0 : smoothstep(rIsco, rIsco + 0.5, r);
    float edgeMask = innerMask * smoothstep(rOut, rOut - 0.5, r);
    dens = (dens + 1.5 * spot) * edgeMask;
    densA *= edgeMask;

    // outer edge: darkens over the fade zone (as the sheet), and over its last VOL_THIN_ZONE the
    // gas runs out, optical depth falling log-linearly to VOL_TAU_EDGE, so the rim ends in a
    // dark, see-through curve instead of a dark wall (side-on rays made speckle there)
    float thin = saturate(outer / VOL_THIN_ZONE);
    float edge = 1.0;
    [branch]
    if (thin < 1.0) // (only the last part of the fade zone: most samples skip the exp / log)
        edge = exp((1.0 - thin) * log(max(min(VOL_TAU_EDGE, tauCol), 1e-12) / max(tauCol, 1e-12)));

    // core: kappa rho = tau / (2 H); path length in the gas frame = Sigma dlambda / g
    float Sigma = r * r + dc.aPhys * dc.aPhys * p.mu * p.mu;
    float base = Sigma / (2.0 * p.h * r * max(g, 0.05));
    float tauSmooth = tauCol * edge * fade;
    p.kA = tauSmooth * dens * base;
    p.tauV = 0.5 * tauSmooth * p.colC; // smooth column above
    // atmosphere: its own optical depth, thins with the disk, fades with it at the edge
    float tauAtm = _BHVolume2.x * thinPlunge * edge * fade * outer;
    p.kAa = tauAtm * densA * base / (1.0 + VolAtmS());
    if (p.kA + p.kAa < 1e-6)
        return;

    p.g = g;
    p.clump = 0.3 * sN;
    p.spot = spot;
    p.kS = dens * fade * outer; // brighter clumps, darkening toward the outer edge
    p.flux0 = DiskFluxRel(r, rIsco, dc.ap2.y, hd.fluxNorm);
    p.S1c = VolSourceEdd(p, 1.0, 1.0, dc);
    p.S = VolSource(p, p.tauV, dc);
    // the atmosphere glows like the disk surface (edd = 1)
    p.SA = p.S1c * (densA * fade * outer * _BHVolume2.w);
    p.sValid = true;
}

// Mino-time step from p.
float VolStep(VolPoint p, VolRay vr, float nSteps, float qTop)
{
    float rr = max(p.r, 0.5);
    float stepFar = 0.1 / rr; // ~10 % of r of path
    if (!p.inDisk)
        return stepFar;
    // nSteps per core thickness, by how fast this ray crosses it; one scale height per step
    // in the (thin, soft) atmosphere
    float dq = (abs(p.q) <= VOL_Q_CUT) ? 2.0 * VOL_Q_CUT / nSteps : VolAtmS();
    float stepIn = min(dq * p.h / max(abs(p.muDot), 1e-6), stepFar);
    if (abs(p.q) <= qTop && abs(p.mu) < VOL_FUNNEL_MU1)
        return stepIn;

    // above the gas
    if (p.mu * p.muDot >= 0.0) // moving away from the midplane
        return stepFar;
    // approaching: jump to where |cos theta| reaches the top of the gas (exact theta phase),
    // at most ~10 % of r: H changes along a grazing ray. Measured with the thickness WITHOUT the
    // rim: sized by the thin rim, the jump went straight through the thicker gas just inside it
    // (rims seen edge-on came out undersampled and speckled).
    float dT = ThetaPhaseHalfWidth(vr.tm, min(qTop * p.hJ, VOL_FUNNEL_MU1));
    float u = vr.tm.s0 + p.lam * vr.uPerLambda;
    float uC = 0.25 + 0.5 * ceil((u - 0.25) / 0.5); // the crossing being approached
    float skip = min((uC - dT - u) / vr.uPerLambda, stepFar);
    return max(skip, min(VolAtmS() * p.hJ / max(abs(p.muDot), 1e-6), stepFar));
}

// optical depth between two points, (core, atmosphere): endpoints' opacity, exact mean of the
// vertical profiles over the step
float2 VolTau(VolPoint p0, VolPoint p1, float dl)
{
    float dq = p1.q - p0.q;
    float fc, fa;
    [branch]
    if (abs(dq) > 1e-3)
    {
        fc = max((((p0.q >= 0.0) ? p0.colC : 2.0 - p0.colC) - ((p1.q >= 0.0) ? p1.colC : 2.0 - p1.colC)) / dq, 0.0);
        fa = max((p0.cumA - p1.cumA) / dq, 0.0);
    }
    else
    {
        fc = 0.5 * (VolProfile(p0.q) + VolProfile(p1.q));
        fa = 0.5 * (VolAtmProfile(p0.q) + VolAtmProfile(p1.q));
    }
    return float2(0.5 * (p0.kA + p1.kA) * dl * fc, 0.5 * (p0.kAa + p1.kAa) * dl * fa);
}

// opacity-weighted source of core and atmosphere at a point
float3 VolMix(VolPoint p, float2 t)
{
    return (t.x * p.S + t.y * p.SA) / max(t.x + t.y, 1e-12);
}

// ------------------------------------------------------------------
// Rays the slab intervals can't handle:
//  * vortical theta motion (eta < 0): never crosses the equator, so it has no crossings to build
//    intervals around -- but it can still pass through a thick disk's gas. On screen this set is
//    x^2 + y^2 / cos^2(theta_cam) < a^2: a line at the equator, a circle of radius a at the pole.
//  * near-radial rays (b^2 < NEAR_RADIAL_B2): radial quadrature, no r(lambda) map. A small circle
//    around the hole, at any spin.
// The sheet disk is right to skip them; the volume showed them as black holes in the gas.
// Neither has a radial turning point (b < a resp. b ~ 0), so r runs monotonically: march in Mino time
// with r advanced by dr/dlambda = +-sqrt(R(r)) (midpoint), theta exact (EvaluateCosTheta handles both
// motions), azimuth by the trapezoid rule from the camera. Matches the interval march on ordinary
// monotonic rays to ~0.5 % (median; tested on the Python port).
// ------------------------------------------------------------------
float VolRadialR(float r, float a, float xi, float eta)
{
    float X = r * r + a * a - a * xi;
    float D = r * r - 2.0 * KERR_M * r + a * a;
    return max(X * X - D * (eta + (xi - a) * (xi - a)), 0.0);
}

// geometry at radius r, Mino time lam; muDot is a magnitude only (VolStepBound needs no sign)
VolPoint VolGeometryAtR(VolRay vr, float r, float lam, bool rIncreasing, DiskCtx dc)
{
    VolPoint p = (VolPoint) 0;
    p.lam = lam;
    float a = vr.a, xi = vr.xi;
    p.r = r;
    p.mu = EvaluateCosTheta(vr.tm, lam);
    p.hJ = max(DiskAspectCore(r, dc), 1e-4);
    p.h = DiskRim(p.hJ, r, dc.diskOutNorm);
    p.q = p.mu / p.h;
    p.colC = VolColumnAbove(p.q, VolProfile(p.q));
    p.cumA = VolAtmCumSigned(p.q);
    float D = KerrDelta(r, a);
    float s2 = max(1.0 - p.mu * p.mu, 1e-4);
    p.dphi = a * (r * r + a * a - a * xi) / max(D, 1e-4) + xi / s2 - a;
    float mu2 = p.mu * p.mu;
    p.muDot = sqrt(max(vr.eta - (xi * xi + vr.eta - a * a) * mu2 - a * a * mu2 * mu2, 0.0));
    p.photonPr = (rIncreasing ? -1.0 : 1.0) * sqrt(VolRadialR(r, a, xi, vr.eta)) / max(D, 1e-4);
    p.inDisk = (r >= vr.emitLower) && (r <= dc.diskOutNorm * 1.001) && (D >= 0.02);
    return p;
}

// VolStep without crossing phases: above the gas, the time to reach its top (unrimmed thickness)
// at the current |dmu/dlambda|
float VolStepBound(VolPoint p, float nSteps, float qTop)
{
    float stepFar = 0.1 / max(p.r, 0.5);
    if (!p.inDisk)
        return stepFar;
    float md = max(abs(p.muDot), 1e-6);
    float dq = (abs(p.q) <= VOL_Q_CUT) ? 2.0 * VOL_Q_CUT / nSteps : VolAtmS();
    float stepIn = min(dq * p.h / md, stepFar);
    if (abs(p.q) <= qTop && abs(p.mu) < VOL_FUNNEL_MU1)
        return stepIn;
    float muTop = min(qTop * p.hJ, VOL_FUNNEL_MU1);
    return max(min((abs(p.mu) - muTop) / md, stepFar), min(VolAtmS() * p.hJ / md, stepFar));
}

void AccumulateDiskVolumeRadial(
    ThetaMotion tm, RadialMotion rad, float phi, float rStart, float xi, float eta, float a, DiskCtx dc,
    Texture2D _Noise, SamplerState sampler_Noise, inout CrossingAcc acc)
{
    if (rad.hasTurning || rad.innerRegion)
        return;
    // vortical: |cos theta| >= sqrt(u-) throughout; inside the polar funnel it never meets gas
    if (tm.isVortical && tm.uMinus >= VOL_FUNNEL_MU1 * VOL_FUNNEL_MU1)
        return;
    VolRay vr;
    vr.tm = tm;
    vr.rad = rad;
    vr.xi = xi;
    vr.eta = eta;
    vr.a = a;
    vr.xiPhys = -xi; // the trace runs in Kerr(-a)
    vr.C = 1.0;
    vr.uPerLambda = 0.0;
    vr.emitLower = dc.plungeOn ? dc.rHorizon + 0.05 : dc.hd.rIsco;

    bool inward = rad.isInward;
    float rEnd = inward ? vr.emitLower : min(dc.diskOutNorm, rad.rEndUsed);
    float r0 = rStart;
    float lam = 0.0;
    float ph = phi;
    [branch]
    if (inward && r0 > dc.diskOutNorm)
    {
        // camera outside the disk: Mino time and azimuth at its outer edge (GL8 in u = 1/r)
        float rE = dc.diskOutNorm;
        lam = RadialMinoToInf(a, KERR_M, xi, eta, rE, 1.0 / r0);
        float muM = EvaluateCosTheta(tm, 0.5 * lam);
        ph += ((a != 0.0) ? RadialDPhiToInf(a, KERR_M, xi, eta, rE, 1.0 / r0) : 0.0)
            + (xi / max(1.0 - muM * muM, 1e-4) - a) * lam;
        r0 = rE;
    }
    if (inward ? (r0 <= rEnd) : (r0 >= rEnd))
        return;

    float sgn = inward ? -1.0 : 1.0;
    float nSteps = max(_BHVolume.z, 1.0);
    int maxSteps = max((int) _BHVolume.w, 1);
    float qTop = VolQTop();
    VolPoint p0 = (VolPoint) 0;
    float depth = 0.0;
    float r = r0;
    float dl = 0.0;
    [loop]
    for (int j = 0; j <= maxSteps; j++)
    {
        bool last = false;
        [branch]
        if (j > 0)
        {
            dl = VolStepBound(p0, nSteps, qTop);
            float rm = p0.r + sgn * 0.5 * dl * sqrt(VolRadialR(p0.r, a, xi, eta));
            r = p0.r + sgn * dl * sqrt(VolRadialR(rm, a, xi, eta));
            last = (j == maxSteps) || (inward ? (r <= rEnd) : (r >= rEnd));
            if (last) // the budget's last step / the end: exactly to rEnd
            {
                r = rEnd;
                dl = abs(rEnd - p0.r) / max(sqrt(VolRadialR(0.5 * (p0.r + rEnd), a, xi, eta)), 1e-6);
            }
            lam = p0.lam + dl;
        }
        VolPoint p1 = VolGeometryAtR(vr, r, lam, !inward, dc);
        p1.phi = (j == 0) ? ph : p0.phi + 0.5 * dl * (p0.dphi + p1.dphi);
        VolShade(p1, vr, 0, dc, _Noise, sampler_Noise);
        [branch]
        if (j == 0)
        {
            depth = p1.tauV;
        }
        else
        {
            float2 t = VolTau(p0, p1, dl);
            float dTau = t.x + t.y;
            float d1 = min(p1.tauV, depth + t.x);
            [branch]
            if (p1.sValid && d1 < p1.tauV)
                p1.S = VolSource(p1, d1, dc);
            depth = d1;
            [branch]
            if (dTau > 1e-5 && (p0.sValid || p1.sValid))
            {
                float3 S0 = p0.sValid ? VolMix(p0, t) : VolMix(p1, t);
                float3 S1 = p1.sValid ? VolMix(p1, t) : S0;
                float e = exp(-dTau);
                float one = 1.0 - e;
                float lin = (dTau > 1e-3) ? (one - dTau * e) / dTau : dTau * (0.5 - dTau / 3.0);
                acc.color += (S0 * one + (S1 - S0) * lin) * acc.transmittance;
                acc.transmittance *= e;
                // opaque from here: the rest as the gas here (deep in it the light tends to the local source):
                // dropping it left a step of up to 1% along the contour where a step first got below 0.01
                if (acc.transmittance < 0.01)
                {
                    acc.color += S1 * acc.transmittance;
                    acc.transmittance = 0.0;
                    break;
                }
            }
        }
        p0 = p1;
        if (last)
            break;
    }
    acc.crossingCount = max(acc.crossingCount, 1);
}

// ------------------------------------------------------------------
// Volumetric replacement of AccumulateDiskCrossings (one extra argument: rStart, the radius
// the trace starts at). Fills acc.color / transmittance; no first crossing (hasFirst, emFirst):
// a volume has no single point that the reconstruction could re-shade at full resolution.
// ------------------------------------------------------------------
void AccumulateDiskVolume(
    ThetaMotion tm, RadialMotion rad, inout PolarPhiCum pc, inout bool pcReady, float lambdaTotal,
    float phi, float rStart, float xi, float eta, float a, DiskCtx dc,
    Texture2D _Noise, SamplerState sampler_Noise,
    inout CrossingAcc acc)
{
    // vortical rays never cross the equator and near-radial rays have no r(lambda) map, but both
    // can pass through the gas: their own march
    [branch]
    if (tm.isVortical || rad.numRealRoots == 1)
    {
        AccumulateDiskVolumeRadial(tm, rad, phi, rStart, xi, eta, a, dc, _Noise, sampler_Noise, acc);
        return;
    }

    VolRay vr;
    vr.tm = tm;
    vr.rad = rad;
    vr.xi = xi;
    vr.eta = eta;
    vr.a = a;
    vr.xiPhys = -xi; // the trace runs in Kerr(-a); the physical photon's xi is -xi
    vr.C = max(rad.lambdaScale * rad.K_r * rad.radialNorm, 1e-8);
    vr.uPerLambda = tm.kTheta / (4.0 * max(tm.K_theta, 1e-5));
    vr.emitLower = dc.plungeOn ? dc.rHorizon + 0.05 : dc.hd.rIsco;

    float nSteps = max(_BHVolume.z, 1.0);
    int maxSteps = max((int) _BHVolume.w, 1);
    float qTop = VolQTop();

    // slab intervals: the envelope cone (core + atmosphere) around every equator crossing
    float4 th = _BHDiskThick[dc.hd.h];
    float hEnv = max(max(th.z, th.y), 0.002);
    float halfU = ThetaPhaseHalfWidth(tm, min(qTop * hEnv, VOL_FUNNEL_MU1)); // (no gas above: funnel)
    float halfLam = halfU / vr.uPerLambda;

    // the part of the ray inside the disk's outer radius, in Mino time
    float2 win = RadialWindowBelow(rad, dc.diskOutNorm, rStart) * vr.C;
    win.y = min(win.y, lambdaTotal);
    if (win.x >= win.y || halfU <= 0.0)
        return;

    int k0 = (int) ceil((tm.s0 - 0.25) / 0.5);              // first crossing ahead: image order 0
    int kStart = (int) ceil((tm.s0 - 0.25 - halfU) / 0.5);  // first slab reaching the start
                                                            // (camera inside the disk)
    // ...which makes the later ones higher-order images only once it is clear of the camera: in full from
    // ~0.6 deg (|cos theta| = 0.01) on, not at all right at it. A camera in the disk's plane (orbits in it:
    // common) has that crossing at itself, ahead or behind by a float rounding: the far side's image flipped
    // between order 0 and 1 (faded, thinner, fewer steps) -- flickering patches.
    float wK0 = saturate((0.25 + 0.5 * (float) k0 - tm.s0) / max(ThetaPhaseHalfWidth(tm, KERR_ORDER_MU_EPS), 1e-6));
    [loop]
    for (int it = 0; it < KERR_MAX_CROSSINGS + 2; it++)
    {
        int k = kStart + it;
        float order = g_orderBase + ((k <= k0) ? 0.0 : (float) (k - k0 - 1) + wK0);
        if (order >= (float) KERR_MAX_CROSSINGS)
            break;
        // higher-order images: thin rings squeezed toward the photon ring, faded by Higher Order Fade --
        // a fraction of the samples (the ring around the hole is where the marches pile up), none once
        // they have faded away
        float fadeO = exp(-order * dc.ap2.w);
        if (order > 0.0 && fadeO < 0.01)
            break;
        float ordScale = exp2(-min(order, 2.0));   // 1, 0.5, 0.25
        int maxStepsO = max((int) ((float) maxSteps * ordScale), 4);
        float nStepsO = max(nSteps * ordScale, 2.0);
        float lamC = (0.25 + 0.5 * (float) k - tm.s0) / vr.uPerLambda;
        if (lamC - halfLam >= win.y)
            break;
        float la = max(max(lamC - halfLam, win.x), 0.0);
        float lb = min(lamC + halfLam, win.y);
        if (la >= lb)
            continue;

        // exact azimuth at the interval start
        [branch]
        if (!pcReady)
        {
            pc = MakePolarPhiCum(tm, xi, a);
            pcReady = true;
        }
        float phiStart = phi + PolarPhiAtLambda(tm, pc, a, la, vr.uPerLambda);
        [branch]
        if (a != 0.0)
        {
            float sA = la / vr.C;
            acc.radPhiRem += RadialPhiRemGL4(rad, a, xi, acc.radS, sA);
            acc.radS = sA;
            phiStart += acc.radPhiRem + a * la;
        }

        // (the interval's first point is step -1 of the same loop: one copy of the sample code -- it is large)
        VolPoint p0 = (VolPoint) 0;
        float depth = 0.0; // optical depth below the core's surface the ray can have reached
        [loop]
        for (int j = -1; j < maxStepsO; j++)
        {
            float dl = 0.0;
            float lamN = la;
            [branch]
            if (j >= 0)
            {
                if (p0.lam >= lb)
                    break;
                // the last step of the budget takes whatever is left of the interval
                dl = (j == maxStepsO - 1) ? (lb - p0.lam) : min(VolStep(p0, vr, nStepsO, qTop), lb - p0.lam);
                lamN = p0.lam + dl;
            }
            VolPoint p1 = VolGeometry(vr, lamN, dc);
            // azimuth: exact at the start, then the trapezoid rule from the point before
            p1.phi = (j < 0) ? phiStart : p0.phi + 0.5 * (lamN - p0.lam) * (p0.dphi + p1.dphi);
            VolShade(p1, vr, order, dc, _Noise, sampler_Noise);
            [branch]
            if (j < 0)
            {
                depth = p1.tauV;
                p0 = p1;
                continue;
            }
            float2 t = VolTau(p0, p1, dl);
            float dTau = t.x + t.y;

            // depth cap: not deeper than the column above, nor than what the ray has crossed
            // (0 again once the ray is out of the gas)
            float d1 = min(p1.tauV, depth + t.x);
            [branch]
            if (p1.sValid && d1 < p1.tauV)
                p1.S = VolSource(p1, d1, dc);
            depth = d1;

            [branch]
            if (dTau > 1e-5 && (p0.sValid || p1.sValid))
            {
                float3 S0 = p0.sValid ? VolMix(p0, t) : VolMix(p1, t);
                float3 S1 = p1.sValid ? VolMix(p1, t) : S0;
                float e = exp(-dTau);
                float one = 1.0 - e;
                float lin = (dTau > 1e-3) ? (one - dTau * e) / dTau : dTau * (0.5 - dTau / 3.0);
                acc.color += (S0 * one + (S1 - S0) * lin) * acc.transmittance;
                acc.transmittance *= e;

                // no first-crossing report (acc.hasFirst stays false): a volume has no point to
                // re-shade at full res, and the first-crossing class bits and radius / angle
                // checks made the reconstruction reject nearly every wispy cell (full-res traces)
                if (acc.transmittance < 0.01)
                {
                    acc.color += S1 * acc.transmittance;   // (the rest as the gas here: see AccumulateDiskVolumeRadial)
                    acc.transmittance = 0.0;
                    acc.crossingCount = (int) order + 1;
                    return;
                }
            }
            p0 = p1;
        }
        acc.crossingCount = (int) order + 1;
    }
}

#endif
