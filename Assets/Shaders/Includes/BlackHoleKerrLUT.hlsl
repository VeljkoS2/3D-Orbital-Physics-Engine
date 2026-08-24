#ifndef BLACKHOLE_KERR_LUT_INCLUDED
#define BLACKHOLE_KERR_LUT_INCLUDED

#define KERR_M 0.5
#define KERR_MAX_CROSSINGS 6
#define KERR_QUAD_STEPS 40

// ---------------- Metric & Coordinates ----------------
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
    float3 arbitrary = (abs(basisZ.y) < 0.99) ? float3(0, 1, 0) : float3(1, 0, 0);
    basisX = normalize(cross(arbitrary, basisZ));
    basisY = cross(basisZ, basisX);
}

void CartesianToOblate(float3 posLocal, float a, out float r, out float cosTheta, out float sinTheta, out float phi)
{
    float x = posLocal.x, y = posLocal.y, z = posLocal.z;
    float p = x * x + y * y + z * z - a * a;
    float rr = 0.5 * (p + sqrt(max(p * p + 4.0 * a * a * z * z, 0.0)));
    r = sqrt(max(rr, 1e-8));
    cosTheta = clamp(z / r, -1.0, 1.0);
    sinTheta = max(sqrt(1.0 - cosTheta * cosTheta), 1e-6);
    phi = atan2(y, x);
}

void OblateTetrad(float r, float cosTheta, float sinTheta, float phi, float a,
                  out float3 e_r, out float3 e_theta, out float3 e_phi)
{
    float rho = sqrt(r * r + a * a);
    float sigma = sqrt(max(r * r + a * a * cosTheta * cosTheta, 1e-6));
    
    float cp = cos(phi);
    float sp = sin(phi);

    e_r = float3(r * sinTheta * cp, r * sinTheta * sp, rho * cosTheta) / sigma;
    e_theta = float3(rho * cosTheta * cp, rho * cosTheta * sp, -r * sinTheta) / sigma;
    
    float lenPhi = max(sqrt(cp * cp + sp * sp), 1e-6);
    e_phi = float3(-sp, cp, 0.0) / lenPhi;
}

void LocalDirToConstants(float n_r, float n_theta, float n_phi,
                         float r, float cosTheta, float sinTheta, float a,
                         out float xi, out float eta, out float E)
{
    float Sigma = r * r + a * a * cosTheta * cosTheta;
    float Akerr = KerrAkerr(r, sinTheta, a);
    float Delta = KerrDelta(r, a);
    float alpha = sqrt(max(Delta * Sigma / max(Akerr, 1e-8), 0.0));
    float omega = 2.0 * KERR_M * a * r / max(Akerr, 1e-8);

    float sqrtGphiphi = sinTheta * sqrt(Akerr / Sigma);
    float L = n_phi * sqrtGphiphi;

    E = max(alpha + omega * L, 1e-4);
    xi = L / E;

    float p_theta = n_theta * sqrt(Sigma);
    float Q = p_theta * p_theta + cosTheta * cosTheta * (L * L / max(sinTheta * sinTheta, 1e-8) - a * a * E * E);
    eta = max(Q / (E * E), 0.0);
}

// ---------------- Axis Mapping ----------------
float KerrXAxisToXi(float u, float xiCritPro, float xiCritRetro, float xiFarMax, float logK)
{
    float centeredU = u - 0.5;
    float xiCrit = (centeredU >= 0.0) ? xiCritPro : abs(xiCritRetro);
    float v = abs(centeredU) * 2.0;
    
    float mag;
    if (v < 0.5)
    {
        float w = v * 2.0;
        mag = xiCrit * (w * w * w);
    }
    else
    {
        float w = (v - 0.5) * 2.0;
        float t = (exp(w * logK) - 1.0) / (exp(logK) - 1.0);
        mag = xiCrit + xiFarMax * t;
    }
    
    return (centeredU >= 0.0) ? mag : -mag;
}

float XiToU(float xi, float xiCritPro, float xiCritRetro, float xiFarMax, float logK)
{
    bool prograde = xi >= 0.0;
    float lo = prograde ? 0.5 : 0.0;
    float hi = prograde ? 1.0 : 0.5;
    [unroll]
    for (int i = 0; i < 20; i++)
    {
        float mid = 0.5 * (lo + hi);
        float xiAtMid = KerrXAxisToXi(mid, xiCritPro, xiCritRetro, xiFarMax, logK);
        if (prograde)
        {
            if (xiAtMid < xi)
                lo = mid;
            else
                hi = mid;
        }
        else
        {
            if (xiAtMid > xi)
                hi = mid;
            else
                lo = mid;
        }
    }
    return 0.5 * (lo + hi);
}

// ---------------- Radial LUT Functions ----------------
float FindCameraRadialTau(Texture3D lut, SamplerState samp, float u, float v, float rCam,
                            out float totalTauInbound, out bool captured)
{
    float4 header = lut.SampleLevel(samp, float3(u, v, 0.0), 0);
    totalTauInbound = max(header.g, 1e-4);
    captured = header.b > 0.5;

    float lo = 0.0, hi = 1.0;
    [unroll]
    for (int i = 0; i < 20; i++)
    {
        float mid = 0.5 * (lo + hi);
        float rAtMid = lut.SampleLevel(samp, float3(u, v, mid), 0).r;
        if (rAtMid > rCam)
            lo = mid;
        else
            hi = mid;
    }
    float wBisect = 0.5 * (lo + hi);
    float oneMinusW = 1.0 - wBisect;
    return (1.0 - oneMinusW * oneMinusW) * totalTauInbound;
}

float SampleRadialAtPhysicalTau(Texture3D lut, SamplerState samp, float u, float v, float physicalTau, float totalTauInbound)
{
    float tableTau = totalTauInbound - abs(physicalTau - totalTauInbound);
    float wRow = 1.0 - sqrt(saturate(1.0 - saturate(tableTau / max(totalTauInbound, 1e-4))));
    return lut.SampleLevel(samp, float3(u, v, saturate(wRow)), 0).r;
}

// ---------------- Polar LUT Functions ----------------
float BisectQuarterPeriod(Texture3D lut, SamplerState samp, float u, float v, float target, out float quarterPeriod)
{
    float4 header = lut.SampleLevel(samp, float3(u, v, 0.0), 0);
    quarterPeriod = max(header.g, 1e-4);

    float lo = 0.0, hi = 1.0;
    [unroll]
    for (int i = 0; i < 16; i++)
    {
        float mid = 0.5 * (lo + hi);
        float muAtMid = lut.SampleLevel(samp, float3(u, v, mid), 0).r;
        if (muAtMid < target)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi) * quarterPeriod;
}

float SamplePolarMu(Texture3D lut, SamplerState samp, float u, float v, float tau, float sQuarter)
{
    sQuarter = max(sQuarter, 1e-4);
    float period = 4.0 * sQuarter;
    float tmod = fmod(tau, period);
    if (tmod < 0.0)
        tmod += period;

    float w;
    float signMu = 1.0;
    if (tmod < sQuarter)
        w = tmod / sQuarter;
    else if (tmod < 2.0 * sQuarter)
        w = (2.0 * sQuarter - tmod) / sQuarter;
    else if (tmod < 3.0 * sQuarter)
    {
        w = (tmod - 2.0 * sQuarter) / sQuarter;
        signMu = -1.0;
    }
    else
    {
        w = (4.0 * sQuarter - tmod) / sQuarter;
        signMu = -1.0;
    }
    return signMu * lut.SampleLevel(samp, float3(u, v, saturate(w)), 0).r;
}

float FindCameraPolarTau(Texture3D lut, SamplerState samp, float u, float v, float mu0, float dMuSign, out float sQuarter)
{
    float s0 = BisectQuarterPeriod(lut, samp, u, v, abs(mu0), sQuarter);
    return (mu0 >= 0.0) ? ((dMuSign >= 0.0) ? s0 : (2.0 * sQuarter - s0))
                        : ((dMuSign < 0.0) ? (2.0 * sQuarter + s0) : (4.0 * sQuarter - s0));
}

#ifndef KERR_ORDER_DAMPING_RATE
#define KERR_ORDER_DAMPING_RATE 0.5
#endif

float KerrLapse(float Sigma, float Akerr, float Delta)
{
    return sqrt(max(Delta * Sigma / max(Akerr, 1e-8), 0.0));
}

float KerrOmegaFrameDrag(float r, float Akerr, float a)
{
    return 2.0 * KERR_M * a * r / max(Akerr, 1e-8);
}

float KerrOmegaOrbit(float r, float a)
{
    float sqrtM = sqrt(KERR_M);
    return sqrtM / (pow(max(r, 1e-4), 1.5) + a * sqrtM);
}

// ---------------- Disk Shading ----------------
void ShadeKerrDiskCrossing(
    float r, float phi, float n_r_local, float n_phi_local, int crossingIndex, float a,
    float diskInnerRadius, float diskOuterRadius, float3 diskNormal,
    Texture2D _Noise, SamplerState sampler_Noise, float DiskRotationOffset, float TimeScale,
    inout float3 accumulatedColor, inout float transmittance)
{
    float diskSpan = diskOuterRadius - diskInnerRadius;
    float innerFade = smoothstep(diskInnerRadius, diskInnerRadius + diskSpan * 0.02, r);
    float outerFade = 1.0 - smoothstep(diskOuterRadius - diskSpan * 0.4, diskOuterRadius, r);
    float diskOpacity = 0.90 * innerFade * outerFade * exp(-abs(crossingIndex) * KERR_ORDER_DAMPING_RATE);

    if (diskOpacity < 0.001)
        return;

    float radiusT = saturate((r - diskInnerRadius) / max(diskSpan, 1e-4));
    float3 innerColor = float3(2.5, 1.5, 1.0) * 2.0;
    float3 midColor = float3(1.3, 0.6, 0.4) * 1.2;
    float3 outerColor = float3(1.5, 0.5, 0.4) * 0.5;
    float tt = pow(radiusT, 0.5);
    float3 tempColor = (tt < 0.5) ? lerp(innerColor, midColor, tt * 2.0) : lerp(midColor, outerColor, (tt - 0.5) * 2.5);
    tempColor *= lerp(1.0, 0.1, radiusT);

    float sN, cN;
    sincos(DiskRotationOffset, sN, cN);
    float2 dir = float2(cos(phi), sin(phi));
    float2 rotatedDir = float2(dir.x * cN - dir.y * sN, dir.x * sN + dir.y * cN);
    float angle = atan2(rotatedDir.y, rotatedDir.x);
    float wrappedTime = fmod(TimeScale, 100.0);
    float angleNormalized = (angle + 3.14159) / 6.28318;
    float2 polarUV = float2(angleNormalized + wrappedTime * 0.1, r * 0.2);
    float2 periodicUV = float2(frac(polarUV.x), polarUV.y);
    float2 stretchedUV = periodicUV * float2(2.0, 1.0);
    float noise1 = _Noise.SampleLevel(sampler_Noise, stretchedUV, 0).r;
    float noise2 = _Noise.SampleLevel(sampler_Noise, stretchedUV + float2(0.53, 0.77), 0).r;
    float turbulence = (noise1 + noise2) * 0.5;
    float mask = smoothstep(diskInnerRadius, diskInnerRadius + 0.5, r) * smoothstep(diskOuterRadius, diskOuterRadius - 0.5, r);
    tempColor *= lerp(0.2, 1.0, turbulence * mask);

    float Akerr = KerrAkerr(r, 1.0, a);
    float Delta = KerrDelta(r, a);
    float Sigma = r * r;
    float alpha = KerrLapse(Sigma, Akerr, Delta);
    float omega = KerrOmegaFrameDrag(r, Akerr, a);
    float sqrtGphiphi = sqrt(Akerr / Sigma);
    float Omega = KerrOmegaOrbit(r, a);
    
    float beta = clamp(abs(Omega - omega) * sqrtGphiphi / max(alpha, 1e-3), 0.0, 0.99);
    float gamma = 1.0 / sqrt(max(1e-4, 1.0 - beta * beta));

    float3 localRayDir = n_r_local * float3(cos(phi), sin(phi), 0.0)
                       + n_phi_local * float3(-sin(phi), cos(phi), 0.0);
    localRayDir = normalize(localRayDir);

    float3 orbitalVelocityDir = float3(-sin(phi), cos(phi), 0.0);
    float cosThetaDop = dot(orbitalVelocityDir, -localRayDir);
    float dopplerFactor = 1.0 / (gamma * (1.0 - beta * cosThetaDop));
    
    float g = clamp(dopplerFactor * alpha, 0.0, 3.0);
    float dopplerBeaming = pow(max(g, 1e-4), 2.0);
    float3 cyanTint = float3(0.7, 1.1, 1.6);
    float3 deepRedTint = float3(1.4, 0.4, 0.1);
    float3 dopplerTint = (g >= 1.0) ? lerp(float3(1, 1, 1), cyanTint, saturate((g - 1.0) * 0.8))
                                   : lerp(deepRedTint, float3(1, 1, 1), saturate(g));

    tempColor *= g * dopplerBeaming * dopplerTint;
    accumulatedColor += tempColor * diskOpacity * transmittance;
    transmittance *= (1.0 - diskOpacity);
}

// ---------------- Raymarch Entry Point ----------------
void BlackHoleRaymarchKerrLUT(
    float3 rayDirWorld, float3 bhPositionCamRelative, float schwarzschildRadius,
    float diskInnerRadius, float diskOuterRadius, float escapeRadiusParam, float3 diskNormal,
    float spinRatio,
    TextureCube _SkyBox, SamplerState sampler_SkyBox, Texture2D _Noise, SamplerState sampler_Noise,
    Texture3D _KerrRadialLUT, SamplerState sampler_KerrRadialLUT,
    Texture3D _KerrPolarLUT, SamplerState sampler_KerrPolarLUT,
    Texture2D _KerrCritHeader, SamplerState sampler_KerrCritHeader,
    float etaMax, float xiFarMax, float xiLogK,
    float DiskRotationOffset, float TimeScale,
    out float3 color, out float3 exitPos, out float3 exitDir, out bool didEscape, out float outTransmittance)
{
    float a = clamp(spinRatio, 0.0, 0.999) * KERR_M;
    float rHorizon = KERR_M + sqrt(max(KERR_M * KERR_M - a * a, 0.0));

    float3 basisX, basisY, basisZ;
    BuildSpinBasis(diskNormal, basisX, basisY, basisZ);

    float3 camPosWorldRel = -bhPositionCamRelative / max(schwarzschildRadius, 1e-4);
    float3 camLocal = float3(dot(camPosWorldRel, basisX), dot(camPosWorldRel, basisY), dot(camPosWorldRel, basisZ));
    float3 rayLocal = float3(dot(rayDirWorld, basisX), dot(rayDirWorld, basisY), dot(rayDirWorld, basisZ));

    float r, cosTheta, sinTheta, phi;
    CartesianToOblate(camLocal, a, r, cosTheta, sinTheta, phi);

    float3 straightColor = _SkyBox.SampleLevel(sampler_SkyBox, rayDirWorld, 0).rgb;
    if (dot(normalize(camPosWorldRel), rayDirWorld) > 0.0 && r > escapeRadiusParam)
    {
        exitPos = camPosWorldRel;
        exitDir = rayDirWorld;
        didEscape = true;
        outTransmittance = 1.0;
        color = straightColor;
        return;
    }

    float3 e_r, e_theta, e_phi;
    OblateTetrad(r, cosTheta, sinTheta, phi, a, e_r, e_theta, e_phi);
    float n_r = dot(rayLocal, e_r), n_theta = dot(rayLocal, e_theta), n_phi = dot(rayLocal, e_phi);

    float xi, eta, E;
    LocalDirToConstants(n_r, n_theta, n_phi, r, cosTheta, sinTheta, a, xi, eta, E);

    float vEta = pow(saturate(eta / max(etaMax, 1e-4)), 1.0 / 3.0);
    float2 crit = _KerrCritHeader.SampleLevel(sampler_KerrCritHeader, float2(0.5, vEta), 0).rg;
    float uXi = XiToU(xi, crit.r, crit.g, xiFarMax, xiLogK);

    float totalTauInbound;
    bool captured;
    float tauRCam = FindCameraRadialTau(_KerrRadialLUT, sampler_KerrRadialLUT, uXi, vEta, r, totalTauInbound, captured);

    float sQuarter;
    float tauMuCam = FindCameraPolarTau(_KerrPolarLUT, sampler_KerrPolarLUT, uXi, vEta, cosTheta, sign(n_theta), sQuarter);

    bool inbound = n_r < 0.0;
    float physicalTauCam, remainingTau;
    bool willEscape;

    if (captured)
    {
        physicalTauCam = tauRCam;
        remainingTau = max(totalTauInbound - tauRCam, 0.0);
        willEscape = false;
    }
    else
    {
        physicalTauCam = inbound ? tauRCam : (2.0 * totalTauInbound - tauRCam);
        remainingTau = max(2.0 * totalTauInbound - physicalTauCam, 0.0);
        willEscape = true;
    }

    float3 accumulatedColor = float3(0, 0, 0);
    float transmittance = 1.0;
    float phiAccum = phi;
    float prevMu = cosTheta;
    int crossingCount = 0;
    float rNow = r, muNow = cosTheta, sinThNow = sinTheta;

    [loop]
    for (int i = 1; i <= KERR_QUAD_STEPS; i++)
    {
        float dTauStep = remainingTau / KERR_QUAD_STEPS;
        float dTau = (float) i * dTauStep;

        rNow = SampleRadialAtPhysicalTau(_KerrRadialLUT, sampler_KerrRadialLUT, uXi, vEta, physicalTauCam + dTau, totalTauInbound);
        muNow = SamplePolarMu(_KerrPolarLUT, sampler_KerrPolarLUT, uXi, vEta, tauMuCam + dTau, sQuarter);
        sinThNow = max(sqrt(1.0 - muNow * muNow), 0.02);

        float Delta = KerrDelta(rNow, a);
        float dPhiDTau = clamp(xi / (sinThNow * sinThNow) + a * (2.0 * KERR_M * rNow - a * xi) / max(Delta, 1e-5), -50.0, 50.0);
        phiAccum += dPhiDTau * dTauStep;

        if (i > 1 && crossingCount < KERR_MAX_CROSSINGS && (prevMu * muNow <= 0.0))
        {
            float tCross = saturate(abs(prevMu) / max(abs(prevMu) + abs(muNow), 1e-5));
            float tauCross = (physicalTauCam + dTau - dTauStep) + tCross * dTauStep;
            float rCross = SampleRadialAtPhysicalTau(_KerrRadialLUT, sampler_KerrRadialLUT, uXi, vEta, tauCross, totalTauInbound);

            if (rCross > diskInnerRadius && rCross < diskOuterRadius && rCross > (rHorizon + 0.01))
            {
                float sigmaRNow = (tauCross < totalTauInbound) ? -1.0 : 1.0;
                float Xi = rCross * rCross + a * a - a * xi;
                float DeltaCross = KerrDelta(rCross, a);
                float RCross = max(Xi * Xi - DeltaCross * (eta + (xi - a) * (xi - a)), 0.0);
                float drDTau = sigmaRNow * sqrt(RCross);
                float SigmaCross = rCross * rCross;
                float n_r_local = drDTau / max(sqrt(DeltaCross * SigmaCross), 1e-6);
                float AkerrCross = KerrAkerr(rCross, 1.0, a);
                float n_phi_local = xi * sqrt(SigmaCross / max(AkerrCross, 1e-8));

                ShadeKerrDiskCrossing(rCross, phiAccum, n_r_local, n_phi_local, crossingCount, a,
                    diskInnerRadius, diskOuterRadius, diskNormal,
                    _Noise, sampler_Noise, DiskRotationOffset, TimeScale,
                    accumulatedColor, transmittance);
            }
            crossingCount++;
            if (transmittance < 0.01)
                break;
        }
        prevMu = muNow;
    }

    if (!willEscape)
    {
        color = accumulatedColor;
        exitPos = camPosWorldRel;
        exitDir = float3(0, 0, 0);
        didEscape = false;
        outTransmittance = transmittance;
        return;
    }

    float3 e_r_out, e_theta_out, e_phi_out;
    OblateTetrad(rNow, muNow, sinThNow, phiAccum, a, e_r_out, e_theta_out, e_phi_out);

    float SigmaOut = rNow * rNow + a * a * muNow * muNow;
    float AkerrOut = KerrAkerr(rNow, sinThNow, a);
    float DeltaOut = KerrDelta(rNow, a);
    float XiOut = rNow * rNow + a * a - a * xi;
    float ROut = max(XiOut * XiOut - DeltaOut * (eta + (xi - a) * (xi - a)), 0.0);
    float n_r_out = sqrt(ROut) / max(sqrt(DeltaOut * SigmaOut), 1e-6);
    float ThetaOut = max(eta + a * a * muNow * muNow - xi * xi * muNow * muNow / max(sinThNow * sinThNow, 1e-6), 0.0);
    float n_theta_out = sign(n_theta) * sqrt(ThetaOut) / max(sqrt(SigmaOut), 1e-6);
    float n_phi_out = xi / max(sinThNow * sqrt(AkerrOut / SigmaOut), 1e-6);

    float3 dirLocalOut = normalize(n_r_out * e_r_out + n_theta_out * e_theta_out + n_phi_out * e_phi_out);
    float3 exitDirWorld = dirLocalOut.x * basisX + dirLocalOut.y * basisY + dirLocalOut.z * basisZ;

    float3 skyColor = _SkyBox.SampleLevel(sampler_SkyBox, exitDirWorld, 0).rgb;

    color = accumulatedColor + skyColor * transmittance;
    exitPos = camPosWorldRel + exitDirWorld * escapeRadiusParam;
    exitDir = exitDirWorld;
    didEscape = true;
    outTransmittance = transmittance;
}

#endif