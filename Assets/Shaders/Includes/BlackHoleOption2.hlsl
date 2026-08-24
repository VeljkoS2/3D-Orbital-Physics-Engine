#ifndef BLACKHOLE_LUT_RAYMARCH_INCLUDED
#define BLACKHOLE_LUT_RAYMARCH_INCLUDED

// ---- DEBUG TOGGLE ----
#define BH_DEBUG_SKYONLY 0
#define B_CRITICAL 2.598076211
#define B_MAX 200.0
#define B_SPLIT 20.0        // must exactly match bSplit in the LUT generator
#define B_SPLIT_CAP 1.5     // must exactly match bSplitCap in the LUT generator
#define LOG_K 8.0           // must exactly match k in the LUT generator
#define LOG_K_CAP 8.0       // must exactly match logKCap in the LUT generator
#define WEAK_LOG_K 50.0     // must exactly match weakLogK in the LUT generator
#define PI_VAL 3.14159265359


#define ORDER_DAMPING_RATE 0.35 // tune: higher = higher-order images fade faster
// b-axis anchor points in texture-U space. U_MID sits exactly at b=B_CRITICAL;
// U_LOW/U_HIGH bound the two log-packed regions on either side of it. Must
// match the same four-segment layout used in the generator.
#define U_LOW 0.15
#define U_MID 0.5
#define U_HIGH 0.85

// Exact-model constants: right at the "ring" (camera sitting at a ray's own
// turning point), even a dense LUT is sensitive to interpolation error. This
// closed-form quartic Taylor expansion around the turning point (no texture,
// no integration loop) answers r(phi) there directly.
#define RING_EXACT_R_WINDOW 0.5     // how close (in r) the camera's own ray must be
                                     // to its turning point to trust the exact model
#define RING_EXACT_PHI_WINDOW 0.5  // how far (radians from turning point) the quartic
                                     // model itself stays valid for any individual crossing

// ---- Unified b-axis packing (four segments, continuous through B_CRITICAL) ----
float BtoLutU_Unified(float b)
{
    if (b <= B_SPLIT_CAP)
    {
        return U_LOW * saturate(b / B_SPLIT_CAP);
    }
    else if (b < B_CRITICAL)
    {
        float frac = saturate((B_CRITICAL - b) / (B_CRITICAL - B_SPLIT_CAP));
        float t = saturate(-log10(max(frac, 1e-8)) / LOG_K_CAP);
        return U_LOW + (U_MID - U_LOW) * t;
    }
    else if (b <= B_SPLIT)
    {
        float frac = saturate((b - B_CRITICAL) / (B_SPLIT - B_CRITICAL));
        float t = saturate(1.0 + log10(max(frac, 1e-8)) / LOG_K);
        return U_MID + (U_HIGH - U_MID) * t;
    }
    else
    {
        float s = saturate((b - B_SPLIT) / (B_MAX - B_SPLIT));
        float tWeak = log(1.0 + s * (WEAK_LOG_K - 1.0)) / log(WEAK_LOG_K);
        return U_HIGH + (1.0 - U_HIGH) * tWeak;
    }
}

float4 SampleUnifiedTableRawV(Texture2D lut, SamplerState samp, float b, float v)
{
    float u = BtoLutU_Unified(b);
    return lut.SampleLevel(samp, float2(u, saturate(v)), 0);
}

// phiFraction: 0 = r_start (far away, the calm end of the curve),
// 1 = turning point or horizon (the deep/sensitive end). Row density in
// the texture is concentrated near v=1 to match — the mirror image of
// the old table's convention.
float4 SampleUnifiedTable(Texture2D lut, SamplerState samp, float b, float phiFraction)
{
    float oneMinusPhi = saturate(1.0 - phiFraction);
    float v = 1.0 - sqrt(oneMinusPhi);
    return SampleUnifiedTableRawV(lut, samp, b, v);
}

// Bisects directly in raw-v space, so each halving buys quadratically more
// precision near v=1 (the deep end) — same trick as before, just aimed at
// the other end of the table now. r decreases monotonically as v increases
// (v=0 -> r_start, v=1 -> deep endpoint). Returns the TRUE phiFraction.
float FindCameraTablePhiFraction(Texture2D lut, SamplerState samp, float b, float rCam)
{
    float lo = 0.0, hi = 1.0;
    [unroll]
    for (int i = 0; i < 20; i++)
    {
        float mid = 0.5 * (lo + hi);
        float rAtMid = SampleUnifiedTableRawV(lut, samp, b, mid).r;
        if (rAtMid > rCam)
            lo = mid; // r still too big, go deeper into the table
        else
            hi = mid;
    }
    float vBisection = 0.5 * (lo + hi);
    float oneMinusV = 1.0 - vBisection;
    return 1.0 - oneMinusV * oneMinusV; // v -> true phiFraction
}

// Exact turning point via bisection on the same cubic the generator's
// stop condition implicitly solves. Only valid for b > B_CRITICAL (a real
// root exists in (0, 2/3) only there) — callers must guard this themselves.
float FindTurningPointExact(float b)
{
    float lo = 0.0001, hi = 0.6666;
    [unroll]
    for (int i = 0; i < 24; i++)
    {
        float mid = 0.5 * (lo + hi);
        float f = mid * mid * mid - mid * mid + 1.0 / (b * b);
        if (f > 0.0)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi); // uMin
}

// Forward-evaluates the closed-form quartic Taylor model: u(phi) = uMin +
// C2*phi^2 + C4*phi^4. Both odd-order terms vanish because the trajectory
// is symmetric about the turning point.
float QuarticRingU(float uMin, float C2, float C4, float phi)
{
    float x = phi * phi;
    return uMin + C2 * x + C4 * x * x;
}

// Single unified crossing search — no isCaptured branch needed. The same
// "distance from the deep endpoint" formula (camPhi) covers both regimes:
// for escaping rays it's distance from the turning point, for captured
// rays it's distance from the horizon. The crossing search doesn't need
// to know which one it is.

float3 DebugHsvToRgb(float3 hsv)
{
    float h = hsv.x * 6.0;
    float s = hsv.y;
    float v = hsv.z;
    float c = v * s;
    float x = c * (1.0 - abs(fmod(h, 2.0) - 1.0));
    float m = v - c;

    float3 rgb;
    if (h < 1.0)
        rgb = float3(c, x, 0);
    else if (h < 2.0)
        rgb = float3(x, c, 0);
    else if (h < 3.0)
        rgb = float3(0, c, x);
    else if (h < 4.0)
        rgb = float3(0, x, c);
    else if (h < 5.0)
        rgb = float3(x, 0, c);
    else
        rgb = float3(c, 0, x);

    return rgb + m;
}

void TraceCurvedDiskCrossings(
    Texture2D _LUT, SamplerState sampler_LUT,
    float impactParameter, float3 r_hat_cam, float3 tangent, float sideSign,
    float camPhi, float totalPhi, float remainingPhi, bool inbound,
    float diskInnerRadius, float diskOuterRadius, float3 diskNormal,
    Texture2D _Noise, SamplerState sampler_Noise, float DiskRotationOffset, float TimeScale,
    float uMinExact, float C2Exact, float C4Exact, float quarticConfidence, float bCriticalSuppress,
    inout float3 accumulatedColor, inout float transmittance)
{
    float3 diskTangentA = normalize(cross(diskNormal, abs(diskNormal.z) > 0.99999 ? float3(0, 1, 0) : float3(0, 0, 1)));
    float3 diskTangentB = cross(diskNormal, diskTangentA);

    float A = dot(r_hat_cam, diskNormal);
    float B = sideSign * dot(tangent, diskNormal);
    float baseS = fmod(atan2(-A, B) + 2.0 * PI_VAL, PI_VAL);

    int halfSpan = (int) ceil(remainingPhi / PI_VAL) + 1;
    halfSpan = min(halfSpan, 64); // safety cap, tune for perf

    [loop]
    for (int cc = -halfSpan; cc <= halfSpan; cc++)
    {
        float s_angle = baseS + cc * PI_VAL;
        if (s_angle < 0.0 || s_angle > remainingPhi)
            continue;

        float currentPhiFromCam = inbound ? (camPhi - s_angle) : (camPhi + s_angle);
        float phiFrac = saturate(abs(currentPhiFromCam) / max(totalPhi, 0.0001));

        float radiusAtCrossing;
        float phiBlend = 1.0 - smoothstep(RING_EXACT_PHI_WINDOW * 0.1, RING_EXACT_PHI_WINDOW, abs(currentPhiFromCam));
        float crossingBlend = phiBlend * quarticConfidence;

        if (crossingBlend > 0.0)
        {
            float uQ = QuarticRingU(uMinExact, C2Exact, C4Exact, currentPhiFromCam);
            float rQuartic = 1.0 / max(uQ, 1e-6);
            if (crossingBlend < 1.0)
            {
                float rTable = SampleUnifiedTable(_LUT, sampler_LUT, impactParameter, 1.0 - phiFrac).r;
                radiusAtCrossing = lerp(rTable, rQuartic, crossingBlend);
            }
            else
            {
                radiusAtCrossing = rQuartic;
            }
        }
        else
        {
            radiusAtCrossing = SampleUnifiedTable(_LUT, sampler_LUT, impactParameter, 1.0 - phiFrac).r;
        }
#if 0 // TEMP: confirm whether the quartic path is involved in the streak
        if (crossingBlend > 0.001)
        {
            accumulatedColor += float3(5, 0, 0) * transmittance; // hard red tag
            float3 orderColors[4] = { float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1), float3(1, 1, 0) };
            accumulatedColor += orderColors[abs(cc) % 4] * transmittance;
        }
#endif

        if (radiusAtCrossing > diskInnerRadius && radiusAtCrossing < diskOuterRadius)
        {
            float3 crossingPos = radiusAtCrossing * (cos(s_angle) * r_hat_cam + sin(s_angle) * tangent * sideSign);

            float diskSpan = diskOuterRadius - diskInnerRadius;
            float innerFade = smoothstep(diskInnerRadius, diskInnerRadius + diskSpan * 0.02, radiusAtCrossing);
            float outerFade = 1.0 - smoothstep(diskOuterRadius - diskSpan * 0.4, diskOuterRadius, radiusAtCrossing);
            float diskOpacity = 0.99 * innerFade * outerFade * bCriticalSuppress;
            float orderDamping = exp(-abs(cc) * ORDER_DAMPING_RATE);
            diskOpacity *= orderDamping;

            float radiusT = saturate((radiusAtCrossing - diskInnerRadius) / (diskOuterRadius - diskInnerRadius));
            float3 innerColor = float3(2.5, 1.5, 1.0) * 8.0;
            float3 midColor = float3(1.3, 0.6, 0.4) * 4.0;
            float3 outerColor = float3(1.5, 0.5, 0.4) * 1.0;
            float tt = pow(radiusT, 0.5);
            float3 tempColor = ((tt < 0.5) ? lerp(innerColor, midColor, tt * 2.0) : lerp(midColor, outerColor, (tt - 0.5) * 2.5));
            tempColor *= lerp(1.0, 0.1, radiusT);

            float2 localPos = float2(dot(crossingPos, diskTangentA), dot(crossingPos, diskTangentB));
            float r = length(localPos);
            float2 dir = localPos / max(r, 0.0001);
            float sN, cN;
            sincos(DiskRotationOffset, sN, cN);
            float2 rotatedDir = float2(dir.x * cN - dir.y * sN, dir.x * sN + dir.y * cN);
            float angle = atan2(rotatedDir.y, rotatedDir.x);
            float wrappedTime = fmod(TimeScale, 100);

            float angleNormalized = (angle + 3.14159) / 6.28318;
            float2 polarUV = float2(angleNormalized + (wrappedTime * 0.1), r * 0.2);
            float2 periodicUV = float2(frac(polarUV.x), polarUV.y);
            float2 shift = float2(0.53, 0.77);
            float2 stretchFactor = float2(2.0, 1.0);
            float2 stretchedUV = periodicUV * stretchFactor;

            float noise1 = _Noise.SampleLevel(sampler_Noise, stretchedUV, 0).r;
            float noise2 = _Noise.SampleLevel(sampler_Noise, stretchedUV + shift, 0).r;

            float turbulence = (noise1 + noise2) * 0.5;
            float mask = smoothstep(diskInnerRadius, diskInnerRadius + 0.5, r) * smoothstep(diskOuterRadius, diskOuterRadius - 0.5, r);
            turbulence *= mask;
            tempColor *= lerp(0.1, 1.0, turbulence);

            float3 orbitalVelocityDir = normalize(cross(crossingPos, diskNormal));

            // Per-crossing local propagation direction, from the conserved
            // impact parameter. A crossing is "outgoing" (moving away from
            // the black hole) if the ray is outbound entirely, or if it's
            // inbound but this crossing lies past the deep endpoint. Since
            // captured rays never have s_angle exceed camPhi (bounded by
            // remainingPhi = camPhi in that regime), this single check
            // naturally covers both regimes without needing to know which.
            bool crossingIsOutgoing = inbound ? (s_angle > camPhi) : true;
            float localLapse = sqrt(max(1e-6, 1.0 - 1.0 / radiusAtCrossing));
            float localSinPsi = saturate(impactParameter * localLapse / radiusAtCrossing);
            float localCosPsi = sqrt(saturate(1.0 - localSinPsi * localSinPsi)) * (crossingIsOutgoing ? 1.0 : -1.0);
            float3 tangentAtCrossing = -sin(s_angle) * r_hat_cam + cos(s_angle) * tangent * sideSign;
            float3 localRayDir = localCosPsi * normalize(crossingPos) + localSinPsi * tangentAtCrossing;

            float beta = sqrt(saturate(0.5 / radiusAtCrossing));
            float gamma = 1.0 / sqrt(max(0.0001, 1.0 - beta * beta));

            float cosTheta = dot(orbitalVelocityDir, -localRayDir);
            float dopplerFactor = 1.0 / (gamma * (1.0 - beta * cosTheta));

            float gravRedshift = sqrt(saturate(1.0 - 1.0 / radiusAtCrossing));
            float g = dopplerFactor * gravRedshift;

            float beamingPower = lerp(1.0, 3.0, pow(radiusT, 0.35));
            float dopplerBeaming = pow(g, beamingPower);

            float3 cyanTint = float3(0.7, 1.1, 1.6);
            float3 deepRedTint = float3(1.4, 0.4, 0.1);
            float3 dopplerTint = (g >= 1.0) ? lerp(float3(1, 1, 1), cyanTint, saturate((g - 1.0) * 0.8)) : lerp(deepRedTint, float3(1, 1, 1), saturate(g));

            tempColor *= g * dopplerBeaming * dopplerTint;
            accumulatedColor += tempColor * diskOpacity * transmittance;
            //accumulatedColor = DebugHsvToRgb(float3(, 1, 1));
            transmittance *= (1.0 - diskOpacity);
        }
    }
}

void BlackHoleRaymarch(
    float3 rayDirWorld, float3 bhPositionCamRelative, float schwarzschildRadius,
    float diskInnerRadius, float diskOuterRadius, float escapeRadius, float3 diskNormal,
    TextureCube _SkyBox, SamplerState sampler_SkyBox, Texture2D _Noise, SamplerState sampler_Noise,
    Texture2D _BlackHoleLUT, SamplerState sampler_BlackHoleLUT,
    float DiskRotationOffset, float TimeScale,
    out float3 color, out float3 exitPos, out float3 exitDir, out bool didEscape, out float outTransmittance)
{
    float3 camPos = -bhPositionCamRelative / schwarzschildRadius;
    float rCam = length(camPos);
    float3 r_hat_cam = normalize(camPos);
    float cosPsiCam = dot(r_hat_cam, rayDirWorld);
    float sinPsiCam = sqrt(max(0.0, 1.0 - cosPsiCam * cosPsiCam));
    float lapseFactor = sqrt(max(1e-6, 1.0 - 1.0 / rCam));
    float impactParameter = rCam * sinPsiCam / lapseFactor;

    if (cosPsiCam > 0.0 && rCam > escapeRadius)
    {
        exitPos = camPos;
        exitDir = rayDirWorld;
        didEscape = true;
        outTransmittance = 1.0;
        color = _SkyBox.Sample(sampler_SkyBox, rayDirWorld).rgb;
        return;
    }

    // --- Unbent pass-through, smooth fade instead of a hard cutoff ---
    float3 straightColor = _SkyBox.Sample(sampler_SkyBox, rayDirWorld).rgb;
    float unbendFade = (1.0 - smoothstep(escapeRadius * 0.9, escapeRadius, impactParameter))
                     * (1.0 - smoothstep(B_MAX * 0.9, B_MAX, impactParameter));

    if (unbendFade <= 0.0)
    {
        exitPos = camPos;
        exitDir = rayDirWorld;
        didEscape = true;
        outTransmittance = 1.0;
        color = straightColor;
        return;
    }

    float3 crossResult = cross(r_hat_cam, rayDirWorld);
    float planeLen = length(crossResult);
    float3 planeNormal = planeLen > 0.0001 ? (crossResult / planeLen) : float3(0, 1, 0);
    float3 tangent = cross(planeNormal, r_hat_cam);
    float sideSign = sign(dot(cross(r_hat_cam, rayDirWorld), planeNormal));
    if (sideSign == 0.0)
        sideSign = 1.0;

    float u0 = 1.0 / rCam;
    float u0prime = -u0 * cosPsiCam / max(abs(sinPsiCam), 0.000001);
    bool inbound = u0prime > 0.0;

    float4 v0Sample = SampleUnifiedTableRawV(_BlackHoleLUT, sampler_BlackHoleLUT, impactParameter, 0.0);
    float totalPhi = v0Sample.g;
// Trust what the LUT actually baked (its own stop condition), not just the
// analytic b-vs-B_CRITICAL comparison. Near b_critical those two can
// disagree, and when they do, the shader ends up reading a captured
// trajectory's data as if it were a mirrored escaping one.
    bool isEscaping = v0Sample.b < 0.5;
    float bCriticalSuppress = 1.0;
    float uMinExact = 0.0, C2Exact = 0.0, C4Exact = 0.0;
    float ringWeight = 0.0; // 0 = pure LUT, 1 = pure exact quartic, blended in between
    float c2Confidence = 0.0; // 0 = quartic coefficients unusable (near b_critical), ramps to 1

    if (isEscaping)
    {
        uMinExact = FindTurningPointExact(impactParameter);
        float rTurnExact = 1.0 / max(uMinExact, 1e-6);
        float deltaRExact = rCam - rTurnExact;
        float AExact = 1.5 * uMinExact * uMinExact - uMinExact;
        C2Exact = AExact * 0.5;
        C4Exact = (3.0 * uMinExact - 1.0) * AExact / 24.0;

        float rBlend = 1.0 - smoothstep(RING_EXACT_R_WINDOW * 0.1, RING_EXACT_R_WINDOW, abs(deltaRExact));
        c2Confidence = smoothstep(0.0, 1e-6, abs(C2Exact)); // was 1e-4

        ringWeight = rBlend * c2Confidence;
    }
    
    float camPhi;
    float camPhiExact = 0.0;
    bool needExact = ringWeight > 0.0;
    bool needTable = ringWeight < 1.0;

    if (needExact)
    {
        float u0cam = 1.0 / rCam;
        float safeC2 = abs(C2Exact) > 1e-7 ? C2Exact : (C2Exact >= 0.0 ? 1e-7 : -1e-7);
        float x0 = max((u0cam - uMinExact) / safeC2, 0.0);
    [unroll]
        for (int nIter = 0; nIter < 2; nIter++)
        {
            float g = C4Exact * x0 * x0 + C2Exact * x0 + (uMinExact - u0cam);
            float gp = 2.0 * C4Exact * x0 + C2Exact;
            x0 = max(x0 - g / (abs(gp) > 1e-8 ? gp : 1e-8), 0.0);
        }
        camPhiExact = sqrt(x0);
    }

    if (needTable)
    {
        float newPhiFraction = FindCameraTablePhiFraction(_BlackHoleLUT, sampler_BlackHoleLUT, impactParameter, rCam);
        float camPhiTable = totalPhi - newPhiFraction * totalPhi;
        camPhi = needExact ? lerp(camPhiTable, camPhiExact, ringWeight) : camPhiTable;
    }
    else
    {
        camPhi = camPhiExact;
    }

    float remainingPhi;
    bool willEscape;
    if (inbound)
    {
        if (isEscaping)
        {
            remainingPhi = totalPhi + camPhi; // reach turning point, then mirror back out
            willEscape = true;
        }
        else
        {
            remainingPhi = camPhi; // straight to the horizon, nothing beyond
            willEscape = false;
        }
    }
    else
    {
        if (isEscaping)
        {
            remainingPhi = totalPhi - camPhi; // already past the deep endpoint, heading out
            willEscape = true;
        }
        else
        {
            // b < B_CRITICAL has no turning point. An "outbound" ray with a
            // sub-critical b did not come from the sky -- traced backward it
            // originates at the horizon, not from past a turning point.
            remainingPhi = 0.0;
            willEscape = false;
        }
    }

#if BH_DEBUG_SKYONLY
    {
        float3 exitDirFromBH = cos(remainingPhi) * r_hat_cam + sin(remainingPhi) * tangent * sideSign;
        color = _SkyBox.Sample(sampler_SkyBox, exitDirFromBH).rgb;
        exitPos = escapeRadius * exitDirFromBH;
        exitDir = exitDirFromBH;
        didEscape = true;
        outTransmittance = 1.0;
        return;
    }
#endif

    float3 accumulatedColor = float3(0, 0, 0);
    float transmittance = 1.0;

    TraceCurvedDiskCrossings(
    _BlackHoleLUT, sampler_BlackHoleLUT,
    impactParameter, r_hat_cam, tangent, sideSign,
    camPhi, totalPhi, remainingPhi, inbound,
    diskInnerRadius, diskOuterRadius, diskNormal,
    _Noise, sampler_Noise, DiskRotationOffset, TimeScale,
    uMinExact, C2Exact, C4Exact, c2Confidence, bCriticalSuppress,
    accumulatedColor, transmittance);

    if (!willEscape)
    {
        // Absorbed into the horizon — no exit direction, chain stops here.
        exitPos = camPos;
        exitDir = float3(0, 0, 0);
        didEscape = false;
        outTransmittance = transmittance;
        color = accumulatedColor;
        return;
    }

    float3 exitDirFromBH = cos(remainingPhi) * r_hat_cam + sin(remainingPhi) * tangent * sideSign;
    float3 skyColor = _SkyBox.Sample(sampler_SkyBox, exitDirFromBH).rgb;
    float3 curvedColor = accumulatedColor + skyColor * transmittance;

    exitPos = escapeRadius * lerp(rayDirWorld, exitDirFromBH, unbendFade);
    exitDir = normalize(lerp(rayDirWorld, exitDirFromBH, unbendFade));
    didEscape = true;
    outTransmittance = lerp(1.0, transmittance, unbendFade);
    color = lerp(straightColor, curvedColor, unbendFade);
}
#endif