#ifndef BLACKHOLE_RAYMARCH_INCLUDED
#define BLACKHOLE_RAYMARCH_INCLUDED
void BlackHoleRaymarch(float3 rayDirWorld, float3 bhPositionCamRelative, float schwarzschildRadius, float diskInnerRadius, float diskOuterRadius, float escapeRadius, float3 diskNormal, TextureCube _SkyBox, SamplerState sampler_SkyBox, Texture2D _Noise, SamplerState sampler_Noise, float DiskRotationOffset, float TimeScale, out float3 color, out float3 exitPos, out float3 exitDir, out bool didEscape, out float outTransmittance)
{
    float distanceToCamera = length(bhPositionCamRelative);
    
    float3 camPos = -bhPositionCamRelative / schwarzschildRadius;
    float3 r_hat_cam = normalize(camPos);
    float rCam = length(camPos);
    float cosPsiCam = dot(r_hat_cam, rayDirWorld);
    float sinPsiCam = sqrt(max(0.0, 1.0 - cosPsiCam * cosPsiCam));
    float impactParameter = rCam * sinPsiCam;
    if (impactParameter > escapeRadius)
    {
        exitPos = camPos; // or camPos, wherever it bailed from
        exitDir = rayDirWorld; // unbent, since it never entered lensing
        didEscape = true;
        outTransmittance = 1.0;
        color = float3(0, 0, 0); // unused when didEscape
        return;
    }
    float3 pos = camPos;
    float distToBH = length(pos);
    if (distToBH > escapeRadius * 2.0 && dot(rayDirWorld, normalize(pos)) > 0.0)
    {
        exitPos = pos; // or camPos, wherever it bailed from
        exitDir = rayDirWorld; // unbent, since it never entered lensing
        didEscape = true;
        outTransmittance = 1.0;
        color = float3(0, 0, 0); // unused when didEscape
        return;
    }
    if (rCam > escapeRadius)
    {
        float b = dot(camPos, rayDirWorld);
        float c = dot(camPos, camPos) - (escapeRadius * escapeRadius);
        float discriminant = b * b - c;
        if (discriminant >= 0.0)
        {
            float t = -b - sqrt(discriminant);
            if (t > 0.0)
            {
                pos = camPos + rayDirWorld * t;
            }
        }
    }
    float3 r_hat = normalize(pos);
    float3 crossResult = cross(r_hat, rayDirWorld);
    float crossLength = length(crossResult);

    float3 planeNormal = crossResult / max(crossLength, 0.0001);
    float3 tangent = normalize(cross(planeNormal, r_hat));

    float r0 = length(pos);
    float u0 = 1.0 / r0;
    float cosPsi = clamp(dot(r_hat, rayDirWorld), -1.0, 1.0);
    float sinPsi = sqrt(1.0 - cosPsi * cosPsi);
    if (dot(rayDirWorld, tangent) < 0.0)
    {
        sinPsi = -sinPsi;
    }
    float u0prime = -u0 * cosPsi / max(abs(sinPsi), 0.0000001);
    float lensingFade = smoothstep(escapeRadius, escapeRadius * 0.99999, impactParameter);
    u0prime *= lensingFade;
    float2 state = float2(u0, u0prime);
    float phi = 0.0;
    float3 currentPos = pos;
    float3 previousPos = pos;
    
    static const float bCritical = 2.598076211;

    float bProximity = saturate(1.0 - abs(impactParameter - bCritical) / (bCritical * 0.15));
    bProximity *= bProximity;
    float effectiveStepCount = lerp(256, 1024, bProximity);
    if (effectiveStepCount < 1)
    {
        exitPos = pos; // or camPos, wherever it bailed from
        exitDir = rayDirWorld; // unbent, since it never entered lensing
        didEscape = true;
        outTransmittance = 1.0;
        color = float3(0, 0, 0); // unused when didEscape
        return;
    }

    float closenessFactor = saturate((3.0 - rCam) / 5.0);
    closenessFactor *= closenessFactor;

    float ringBoost = max(bProximity, closenessFactor);

    
    float maxUsefulWindings = lerp(3.0, 96.0, bProximity);
    float totalWindings = min(3.0 + ringBoost * 9.0, maxUsefulWindings);
    int steps = (int) (effectiveStepCount * (1.0 + ringBoost * 1.5));
    
    float dPhi = (totalWindings * 2.0 * 3.14159265) / (float) steps;
    float3 accumulatedColor = float3(0, 0, 0);
    float transmittance = 1.0;
    float3 diskTangentA = normalize(cross(diskNormal, abs(diskNormal.z) > 0.99999 ? float3(0, 1, 0) : float3(0, 0, 1)));
    float3 diskTangentB = cross(diskNormal, diskTangentA);
    float distanceFloorScale = saturate(rCam / 10.0); // tune 50.0 — distance (in rs) where floor reaches max
    float adaptiveMultiplierFloor = lerp(0.1, 1.0, distanceFloorScale); // tune 10.0 — max floor far away
   [loop]
    for (int i = 0; i < steps; i++)
    {

        float adaptiveDPhi = dPhi * max(state.x * 10.0, adaptiveMultiplierFloor);
        if (state.x >= 0.9999)
        {
            color = accumulatedColor;
            exitPos = currentPos;
            exitDir = float3(0, 0, 0);
            didEscape = false;
            outTransmittance = 0.0;
            return;
        }
        if (state.x <= 0.0001)
        {
            break;
        }

        float du_ds_start = state.y;
        float du2_ds2_start = (-state.x + 1.5 * state.x * state.x);

        float2 predictedState = state + float2(du_ds_start, du2_ds2_start) * adaptiveDPhi;

        float du_ds_end = predictedState.y;
        float du2_ds2_end = (-predictedState.x + 1.5 * predictedState.x * predictedState.x);

        state.x += 0.5 * (du_ds_start + du_ds_end) * adaptiveDPhi;
        state.y += 0.5 * (du2_ds2_start + du2_ds2_end) * adaptiveDPhi;

        phi += adaptiveDPhi;

        if (state.x <= 0.001 || (state.y < 0.0 && state.x < (1.0 / (escapeRadius * 1.05))))
        {
            break;
        }
        float r = 1.0 / state.x;
        currentPos = r * (cos(phi) * r_hat + sin(phi) * tangent);
        float heightPrev = dot(previousPos, diskNormal);
        float heightCurr = dot(currentPos, diskNormal);
        [branch]
        if (sign(heightPrev) != sign(heightCurr))
        {
            float t = heightPrev / (heightPrev - heightCurr);
            float3 crossingPos = lerp(previousPos, currentPos, t);
            float radiusAtCrossing = length(crossingPos);
            if (radiusAtCrossing > diskInnerRadius && radiusAtCrossing < diskOuterRadius)
            {
                float diskSpan = diskOuterRadius - diskInnerRadius;
                float innerFade = smoothstep(diskInnerRadius, diskInnerRadius + diskSpan * 0.02, radiusAtCrossing);
                float outerFade = smoothstep(diskOuterRadius, diskOuterRadius - diskSpan * 0.4, radiusAtCrossing);
                float diskOpacity = 0.99 * innerFade * outerFade;

                float radiusT = saturate((radiusAtCrossing - diskInnerRadius) / (diskOuterRadius - diskInnerRadius));
                float3 innerColor = float3(2.0, 1.5, 1.0) * 5.0;
                float3 midColor = float3(1.271, 0.631, 0.392) * 2.5;
                float3 outerColor = float3(1.522, 0.525, 0.429) * 1.0;
                float t = pow(radiusT, 0.5);
                float3 tempColor = ((t < 0.5) ? lerp(innerColor, midColor, t * 2.0) : lerp(midColor, outerColor, (t - 0.5) * 2.0));
                tempColor *= lerp(1.0, 0.1, radiusT);

                float2 localPos = float2(dot(crossingPos, diskTangentA), dot(crossingPos, diskTangentB));
                float r = length(localPos);
                float2 dir = localPos / max(r, 0.0001);
                float s, c;
                sincos(DiskRotationOffset, s, c);
                float2 rotatedDir = float2(dir.x * c - dir.y * s, dir.x * s + dir.y * c);
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
                float3 localRayDir = normalize(currentPos - previousPos);

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
                transmittance *= (1.0 - diskOpacity);

                if (transmittance < 0.01)
                {
                    color = accumulatedColor;
                    exitPos = currentPos;
                    exitDir = float3(0, 0, 0);
                    didEscape = false;
                    outTransmittance = 0.0;
                    return;
                }
            }
        }
        previousPos = currentPos;
    }
    float rFinal = 1.0 / max(state.x, 0.00001);

    float drdphi = -rFinal * rFinal * state.y;

    float3 rayVelocity = drdphi * (cos(phi) * r_hat + sin(phi) * tangent) + rFinal * (-sin(phi) * r_hat + cos(phi) * tangent);



    exitPos = currentPos;

    exitDir = normalize(rayVelocity);

    didEscape = true;

    outTransmittance = transmittance;

    color = accumulatedColor;
}
#endif