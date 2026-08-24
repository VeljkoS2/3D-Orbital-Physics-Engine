#ifndef BLACKHOLE_KERR_RAYMARCH_INCLUDED
#define BLACKHOLE_KERR_RAYMARCH_INCLUDED

#define KERR_MAX_STEPS 260
#define KERR_MAX_CROSSINGS 6
#define KERR_ORDER_DAMPING_RATE 0.35
#define KERR_M 0.5   // mass in rs=1 normalized units

// ---------------- metric ----------------
float KerrSigma(float r, float cosTheta, float a)
{
    return r * r + a * a * cosTheta * cosTheta;
}
float KerrDelta(float r, float a)
{
    return r * r - 2.0 * KERR_M * r + a * a;
}
float KerrAkerr(float r, float sinTheta, float a)
{
    float rr_aa = r * r + a * a;
    return rr_aa * rr_aa - a * a * KerrDelta(r, a) * sinTheta * sinTheta;
}
float KerrLapse(float Sigma, float Akerr, float Delta)
{
    return sqrt(max(Delta * Sigma / max(Akerr, 1e-8), 0.0));
}
float KerrOmegaFrameDrag(float r, float Akerr, float a)
{
    return 2.0 * KERR_M * a * r / max(Akerr, 1e-8);
}

float KerrOuterHorizon(float a)
{
    return KERR_M + sqrt(max(KERR_M * KERR_M - a * a, 0.0));
}
float KerrErgosphere(float cosTheta, float a)
{
    return KERR_M + sqrt(max(KERR_M * KERR_M - a * a * cosTheta * cosTheta, 0.0));
}

float KerrISCO(float a, bool prograde)
{
    float aStar = clamp(a / KERR_M, 0.0, 0.9999);
    float t1 = pow(1.0 - aStar * aStar, 1.0 / 3.0);
    float Z1 = 1.0 + t1 * (pow(1.0 + aStar, 1.0 / 3.0) + pow(1.0 - aStar, 1.0 / 3.0));
    float Z2 = sqrt(3.0 * aStar * aStar + Z1 * Z1);
    float root = sqrt(max((3.0 - Z1) * (3.0 + Z1 + 2.0 * Z2), 0.0));
    return KERR_M * (3.0 + Z2 - (prograde ? root : -root));
}

float KerrOmegaOrbit(float r, float a)
{
    return sqrt(KERR_M) / (pow(r, 1.5) + a * sqrt(KERR_M));
}

// ---------------- Cartesian <-> oblate spheroidal, spin-aligned frame ----------------
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
    float sigma = sqrt(max(r * r + a * a * cosTheta * cosTheta, 1e-8));
    float cp = cos(phi), sp = sin(phi);
    e_r = float3(r * sinTheta * cp, r * sinTheta * sp, rho * cosTheta) / sigma;
    e_theta = float3(rho * cosTheta * cp, rho * cosTheta * sp, -r * sinTheta) / sigma;
    e_phi = float3(-sp, cp, 0.0);
}

void LocalDirToConstants(float n_r, float n_theta, float n_phi,
                         float r, float cosTheta, float sinTheta, float a,
                         out float xi, out float eta, out float E)
{
    float Sigma = KerrSigma(r, cosTheta, a);
    float Akerr = KerrAkerr(r, sinTheta, a);
    float Delta = KerrDelta(r, a);
    float alpha = KerrLapse(Sigma, Akerr, Delta);
    float omega = KerrOmegaFrameDrag(r, Akerr, a);

    float sqrtGphiphi = sinTheta * sqrt(Akerr / Sigma);
    float L = n_phi * sqrtGphiphi;
    E = alpha + omega * L;
    xi = L / max(E, 1e-6);

    float p_theta = n_theta * sqrt(Sigma);
    float Q = p_theta * p_theta + cosTheta * cosTheta * (L * L / max(sinTheta * sinTheta, 1e-8) - a * a * E * E);
    eta = Q / max(E * E, 1e-6);
}

// ---------------- disk shading at one equatorial crossing ----------------
void ShadeKerrDiskCrossing(
    float r, float phi, float n_r_local, float n_phi_local, int crossingIndex, float a,
    float diskInnerRadius, float diskOuterRadius, float3 diskNormal,
    Texture2D _Noise, SamplerState sampler_Noise, float DiskRotationOffset, float TimeScale,
    inout float3 accumulatedColor, inout float transmittance)
{
    float diskSpan = diskOuterRadius - diskInnerRadius;
    float innerFade = smoothstep(diskInnerRadius, diskInnerRadius + diskSpan * 0.02, r);
    float outerFade = 1.0 - smoothstep(diskOuterRadius - diskSpan * 0.4, diskOuterRadius, r);
    float diskOpacity = 0.99 * innerFade * outerFade * exp(-abs(crossingIndex) * KERR_ORDER_DAMPING_RATE);

    float radiusT = saturate((r - diskInnerRadius) / max(diskSpan, 1e-4));
    float3 innerColor = float3(2.5, 1.5, 1.0) * 8.0;
    float3 midColor = float3(1.3, 0.6, 0.4) * 4.0;
    float3 outerColor = float3(1.5, 0.5, 0.4) * 1.0;
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
    tempColor *= lerp(0.1, 1.0, turbulence * mask);

    float Akerr = KerrAkerr(r, 1.0, a);
    float Delta = KerrDelta(r, a);
    float Sigma = r * r;
    float alpha = KerrLapse(Sigma, Akerr, Delta);
    float omega = KerrOmegaFrameDrag(r, Akerr, a);
    float sqrtGphiphi = sqrt(Akerr / Sigma);
    float Omega = KerrOmegaOrbit(r, a);
    float beta = saturate(abs(Omega - omega) * sqrtGphiphi / max(alpha, 1e-4));
    float gamma = 1.0 / sqrt(max(1e-4, 1.0 - beta * beta));

    float3 localRayDir = n_r_local * float3(cos(phi), sin(phi), 0.0)
                       + n_phi_local * float3(-sin(phi), cos(phi), 0.0);
    localRayDir = normalize(localRayDir);

    float3 orbitalVelocityDir = float3(-sin(phi), cos(phi), 0.0);
    float cosThetaDop = dot(orbitalVelocityDir, -localRayDir);
    float dopplerFactor = 1.0 / (gamma * (1.0 - beta * cosThetaDop));
    float g = dopplerFactor * alpha;

    float beamingPower = lerp(1.0, 3.0, pow(radiusT, 0.35));
    float dopplerBeaming = pow(max(g, 1e-4), beamingPower);
    float3 cyanTint = float3(0.7, 1.1, 1.6);
    float3 deepRedTint = float3(1.4, 0.4, 0.1);
    float3 dopplerTint = (g >= 1.0) ? lerp(float3(1, 1, 1), cyanTint, saturate((g - 1.0) * 0.8))
                                    : lerp(deepRedTint, float3(1, 1, 1), saturate(g));

    tempColor *= g * dopplerBeaming * dopplerTint;
    accumulatedColor += tempColor * diskOpacity * transmittance;
    transmittance *= (1.0 - diskOpacity);
}

// ---------------- the march ----------------
void KerrGeodesicMarch(
    float r0, float cosTheta0, float sinTheta0, float phi0,
    float xi, float eta, float sigmaR0, float sigmaTheta0,
    float a, float rHorizon, float escapeRadius,
    float diskInnerRadius, float diskOuterRadius, float3 diskNormal,
    Texture2D _Noise, SamplerState sampler_Noise, float DiskRotationOffset, float TimeScale,
    out float rOut, out float cosThetaOut, out float phiOut, out float sigmaROut, out float sigmaThetaOut,
    out bool didEscape, inout float3 accumulatedColor, inout float transmittance)
{
    float r = r0, cosTheta = cosTheta0, sinTheta = max(sinTheta0, 1e-6), phi = phi0;
    float sigmaR = sigmaR0, sigmaTheta = sigmaTheta0;
    int crossingCount = 0;
    float stepSize = 0.01;

    [loop]
    for (int i = 0; i < KERR_MAX_STEPS; i++)
    {
        float Delta = KerrDelta(r, a);
        float Xi = r * r + a * a - a * xi;
        float R = Xi * Xi - Delta * (eta + (xi - a) * (xi - a));
        float Theta = eta + cosTheta * cosTheta * (a * a - xi * xi / max(sinTheta * sinTheta, 1e-8));

        float prevR = r, prevCosTheta = cosTheta, prevPhi = phi;

        if (R < 0.0)
        {
            r = prevR;
            sigmaR = -sigmaR;
            Delta = KerrDelta(r, a);
            Xi = r * r + a * a - a * xi;
            R = max(Xi * Xi - Delta * (eta + (xi - a) * (xi - a)), 0.0);
        }
        if (Theta < 0.0)
        {
            cosTheta = prevCosTheta;
            sinTheta = max(sqrt(1.0 - cosTheta * cosTheta), 1e-6);
            sigmaTheta = -sigmaTheta;
            Theta = max(eta + cosTheta * cosTheta * (a * a - xi * xi / max(sinTheta * sinTheta, 1e-8)), 0.0);
        }

        float dr = sigmaR * sqrt(R);
        float dTheta = sigmaTheta * sqrt(Theta);
        float dPhi = xi / max(sinTheta * sinTheta, 1e-8) + a * (2.0 * KERR_M * r - a * xi) / max(Delta, 1e-6);

        float speed = max(max(abs(dr), abs(dTheta) * r), 1e-4);
        float h = clamp(stepSize / speed, stepSize * 0.01, stepSize * 4.0);

        r += dr * h;
        cosTheta = clamp(cosTheta - sinTheta * dTheta * h, -1.0, 1.0);
        sinTheta = max(sqrt(1.0 - cosTheta * cosTheta), 1e-6);
        phi += dPhi * h;

        if (crossingCount < KERR_MAX_CROSSINGS && prevCosTheta * cosTheta <= 0.0)
        {
            float w = abs(prevCosTheta) / max(abs(prevCosTheta) + abs(cosTheta), 1e-6);
            float crossR = lerp(prevR, r, w);
            float crossPhi = lerp(prevPhi, phi, w);
            if (crossR > diskInnerRadius && crossR < diskOuterRadius)
            {
                float AkerrCross = KerrAkerr(crossR, 1.0, a);
                float SigmaCross = crossR * crossR;
                float DeltaCross = KerrDelta(crossR, a);
                float n_r_local = dr / max(sqrt(DeltaCross * SigmaCross), 1e-6);
                float n_phi_local = xi * sqrt(SigmaCross / max(AkerrCross, 1e-8));
                float2 rdir2D = float2(n_r_local, n_phi_local);
                rdir2D = normalize(rdir2D);

                ShadeKerrDiskCrossing(crossR, crossPhi, rdir2D.x, rdir2D.y, crossingCount, a,
                    diskInnerRadius, diskOuterRadius, diskNormal,
                    _Noise, sampler_Noise, DiskRotationOffset, TimeScale,
                    accumulatedColor, transmittance);
            }
            crossingCount++;
            if (transmittance < 0.01)
            {
                rOut = r;
                cosThetaOut = cosTheta;
                phiOut = phi;
                sigmaROut = sigmaR;
                sigmaThetaOut = sigmaTheta;
                didEscape = false;
                return;
            }
        }

        if (r <= rHorizon + 0.005)
        {
            rOut = r;
            cosThetaOut = cosTheta;
            phiOut = phi;
            sigmaROut = sigmaR;
            sigmaThetaOut = sigmaTheta;
            didEscape = false;
            return;
        }
        if (r >= escapeRadius)
        {
            rOut = r;
            cosThetaOut = cosTheta;
            phiOut = phi;
            sigmaROut = sigmaR;
            sigmaThetaOut = sigmaTheta;
            didEscape = true;
            return;
        }
    }
    rOut = r;
    cosThetaOut = cosTheta;
    phiOut = phi;
    sigmaROut = sigmaR;
    sigmaThetaOut = sigmaTheta;
    didEscape = false;
}

// ---------------- entry point ----------------
void BlackHoleRaymarchKerr(
    float3 rayDirWorld, float3 bhPositionCamRelative, float schwarzschildRadius,
    float diskInnerRadius, float diskOuterRadius, float escapeRadiusParam, float3 diskNormal,
    float spinRatio,
    TextureCube _SkyBox, SamplerState sampler_SkyBox, Texture2D _Noise, SamplerState sampler_Noise,
    float DiskRotationOffset, float TimeScale,
    out float3 color, out float3 exitPos, out float3 exitDir, out bool didEscape, out float outTransmittance)
{
    float a = clamp(spinRatio, 0.0, 0.999) * KERR_M;

    float3 basisX, basisY, basisZ;
    BuildSpinBasis(diskNormal, basisX, basisY, basisZ);

    float3 camPosWorldRel = -bhPositionCamRelative / schwarzschildRadius;
    float3 camLocal = float3(dot(camPosWorldRel, basisX), dot(camPosWorldRel, basisY), dot(camPosWorldRel, basisZ));
    float3 rayLocal = float3(dot(rayDirWorld, basisX), dot(rayDirWorld, basisY), dot(rayDirWorld, basisZ));

    float r, cosTheta, sinTheta, phi;
    CartesianToOblate(camLocal, a, r, cosTheta, sinTheta, phi);
    float rHorizon = KerrOuterHorizon(a);

    float3 e_r, e_theta, e_phi;
    OblateTetrad(r, cosTheta, sinTheta, phi, a, e_r, e_theta, e_phi);
    float n_r = dot(rayLocal, e_r), n_theta = dot(rayLocal, e_theta), n_phi = dot(rayLocal, e_phi);

    float xi, eta, E;
    LocalDirToConstants(n_r, n_theta, n_phi, r, cosTheta, sinTheta, a, xi, eta, E);

    float3 accumulatedColor = float3(0, 0, 0);
    float transmittance = 1.0;
    float rOut, cosThetaOut, phiOut, sigmaROut, sigmaThetaOut;
    bool escaped;

    KerrGeodesicMarch(r, cosTheta, sinTheta, phi, xi, eta, sign(n_r), sign(n_theta),
        a, rHorizon, escapeRadiusParam, diskInnerRadius, diskOuterRadius, diskNormal,
        _Noise, sampler_Noise, DiskRotationOffset, TimeScale,
        rOut, cosThetaOut, phiOut, sigmaROut, sigmaThetaOut, escaped, accumulatedColor, transmittance);

    if (!escaped)
    {
        exitPos = camPosWorldRel;
        exitDir = float3(0, 0, 0);
        didEscape = false;
        outTransmittance = transmittance;
        color = accumulatedColor;
        return;
    }

    float sinThetaOut = sqrt(max(1.0 - cosThetaOut * cosThetaOut, 1e-8));
    float DeltaOut = KerrDelta(rOut, a);
    float SigmaOut = KerrSigma(rOut, cosThetaOut, a);
    float AkerrOut = KerrAkerr(rOut, sinThetaOut, a);

    float XiOut = rOut * rOut + a * a - a * xi;
    float ROut = max(XiOut * XiOut - DeltaOut * (eta + (xi - a) * (xi - a)), 0.0);
    float ThetaOut = max(eta + cosThetaOut * cosThetaOut * (a * a - xi * xi / max(sinThetaOut * sinThetaOut, 1e-8)), 0.0);

    float n_r_out = sigmaROut * sqrt(ROut) / max(sqrt(DeltaOut * SigmaOut), 1e-6);
    float n_theta_out = sigmaThetaOut * sqrt(ThetaOut) / max(sqrt(SigmaOut), 1e-6);
    float n_phi_out = xi / max(sinThetaOut * sqrt(AkerrOut / SigmaOut), 1e-6);

    float3 localExitDir = normalize(float3(n_r_out, n_theta_out, n_phi_out));

    float3 e_r_out, e_theta_out, e_phi_out;
    OblateTetrad(rOut, cosThetaOut, sinThetaOut, phiOut, a, e_r_out, e_theta_out, e_phi_out);
    float3 exitDirWorld = localExitDir.x * e_r_out + localExitDir.y * e_theta_out + localExitDir.z * e_phi_out;
    exitDirWorld = normalize(exitDirWorld);

    float3 skyColor = _SkyBox.Sample(sampler_SkyBox, exitDirWorld).rgb;
    color = accumulatedColor + skyColor * transmittance;
    exitPos = camPosWorldRel + exitDirWorld * escapeRadiusParam;
    exitDir = exitDirWorld;
    didEscape = true;
    outTransmittance = transmittance;
}
#endif