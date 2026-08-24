#ifndef BLACKHOLE_RAYMARCH_MULTI_INCLUDED
#define BLACKHOLE_RAYMARCH_MULTI_INCLUDED

// Relies on these globals already declared in the calling shader:
 //(x=rs,y=diskInner(rs units),z=diskOuter(rs units),w=escapeRadius(rs units))
 //(x=rotOffset, y=timeScale)
void BlackHoleRaymarchMulti(float3 rayOrigin, float3 rayDirWorld,
    TextureCube _SkyBox, SamplerState sampler_SkyBox,
    Texture2D _Noise, SamplerState sampler_Noise, int _BHCount, float4 _BHPositionsCamRelative[8], float4 _BHParams[8], float4 _BHDiskNormals[8], float4 _BHRotationOffsets[8],
    out float3 color)
{
    float3 pos = rayOrigin;
    float3 dir = normalize(rayDirWorld);

    float3 accumulatedColor = float3(0, 0, 0);
    float transmittance = 1.0;
    float3 prevPos = pos;

    const int MAX_STEPS = 400;

    [loop]
    for (int step = 0; step < MAX_STEPS; step++)
    {
        float minDist = 1e30;
        bool insideAnyHorizon = false;
        bool insideAnyInfluence = false;
        float3 accel = float3(0, 0, 0);

        for (int i = 0; i < _BHCount; i++)
        {
            float3 toBH = _BHPositionsCamRelative[i].xyz - pos;
            float r2 = max(dot(toBH, toBH), 1e-8);
            float r = sqrt(r2);
            float rs = _BHParams[i].x;
            float escapeRadiusWorld = _BHParams[i].w * rs;

            if (r < minDist)
                minDist = r;
            if (r < rs * 1.05)
                insideAnyHorizon = true;
            if (r < escapeRadiusWorld)
                insideAnyInfluence = true;

            if (r > escapeRadiusWorld * 1.5)
                continue; // skip distant BHs, saves cost

            float3 n = toBH / r;
            float3 vPerp = dir - n * dot(dir, n);
            // approximate light-bending acceleration, pulls direction toward this BH
            accel += -1.5 * rs / r2 * vPerp;
        }

        if (insideAnyHorizon)
        {
            color = accumulatedColor; // fell in, disk contribution only
            return;
        }

        // adaptive step size: small near masses, large far away
        float ds = clamp(minDist * 0.15, 0.001, 0.5);

        dir = normalize(dir + accel * ds);
        prevPos = pos;
        pos += dir * ds;

        // --- disk crossing check, per nearby BH ---
        for (int i = 0; i < _BHCount; i++)
        {
            float rs = _BHParams[i].x;
            float3 bhPos = _BHPositionsCamRelative[i].xyz;
            float distToBH = distance(pos, bhPos);
            float escapeRadiusWorld = _BHParams[i].w * rs;
            if (distToBH > escapeRadiusWorld * 1.5)
                continue;

            float3 diskNormal = _BHDiskNormals[i].xyz;
            float3 relPrev = prevPos - bhPos;
            float3 relCurr = pos - bhPos;
            float heightPrev = dot(relPrev, diskNormal);
            float heightCurr = dot(relCurr, diskNormal);

            if (sign(heightPrev) != sign(heightCurr))
            {
                float t = heightPrev / (heightPrev - heightCurr);
                float3 crossingPos = lerp(relPrev, relCurr, t); // BH-relative, in world units
                float radiusAtCrossingWorld = length(crossingPos);
                float diskInnerWorld = _BHParams[i].y * rs;
                float diskOuterWorld = _BHParams[i].z * rs;

                if (radiusAtCrossingWorld > diskInnerWorld && radiusAtCrossingWorld < diskOuterWorld)
                {
                    float diskSpan = diskOuterWorld - diskInnerWorld;
                    float innerFade = smoothstep(diskInnerWorld, diskInnerWorld + diskSpan * 0.02, radiusAtCrossingWorld);
                    float outerFade = smoothstep(diskOuterWorld, diskOuterWorld - diskSpan * 0.3, radiusAtCrossingWorld);
                    float diskOpacity = 0.99 * innerFade * outerFade;

                    float radiusT = saturate((radiusAtCrossingWorld - diskInnerWorld) / diskSpan);
                    float3 innerColor = float3(2.0, 1.8, 2.0);
                    float3 midColor = float3(1.5, 1.0, 0.2);
                    float3 outerColor = float3(1.0, 0.04, 0.01);
                    float tCol = pow(radiusT, 0.5);
                    float3 tempColor = (tCol < 0.5) ? lerp(innerColor, midColor, tCol * 2.0) : lerp(midColor, outerColor, (tCol - 0.5) * 2.0);
                    tempColor *= lerp(2.0, 0.5, radiusT);

                    float3 diskTangentA = normalize(cross(diskNormal, abs(diskNormal.z) > 0.99999 ? float3(0, 1, 0) : float3(0, 0, 1)));
                    float3 diskTangentB = cross(diskNormal, diskTangentA);
                    float2 localPos = float2(dot(crossingPos, diskTangentA), dot(crossingPos, diskTangentB)) / rs; // back to rs units for UV scale
                    float rLocal = length(localPos);
                    float2 dirUV = localPos / max(rLocal, 0.0001);
                    float s, c;
                    sincos(_BHRotationOffsets[i].x, s, c);
                    float2 rotatedDir = float2(dirUV.x * c - dirUV.y * s, dirUV.x * s + dirUV.y * c);
                    float angle = atan2(rotatedDir.y, rotatedDir.x);
                    float wrappedTime = fmod(_BHRotationOffsets[i].y, 100);
                    float angleNormalized = (angle + 3.14159) / 6.28318;
                    float2 polarUV = float2(angleNormalized + (wrappedTime * 0.1), rLocal * 0.2);
                    float2 periodicUV = float2(frac(polarUV.x), polarUV.y);
                    float2 stretchedUV = periodicUV * float2(1.0, 0.2);

                    float noise1 = _Noise.SampleLevel(sampler_Noise, stretchedUV, 0).r;
                    float noise2 = _Noise.SampleLevel(sampler_Noise, stretchedUV + float2(0.53, 0.77), 0).r;
                    float turbulence = (noise1 + noise2) * 0.5;
                    tempColor *= lerp(0.3, 2.0, turbulence);

                    float3 orbitalVelocityDir = normalize(cross(crossingPos, diskNormal));
                    float3 localRayDir = normalize(pos - prevPos);
                    float dopplerFactor = dot(orbitalVelocityDir, -localRayDir);
                    float dopplerBrightness = lerp(0.4, 3.0, saturate(dopplerFactor * 0.5 + 0.5));
                    float3 approachColor = float3(0.5, 0.7, 2.0);
                    float3 recedeColor = float3(2.0, 0.4, 0.2);
                    float3 dopplerTint = dopplerFactor > 0.0 ? lerp(float3(1, 1, 1), approachColor, dopplerFactor) : lerp(float3(1, 1, 1), recedeColor, -dopplerFactor);
                    tempColor *= dopplerBrightness * dopplerTint;

                    float redshiftFactor = sqrt(saturate(1.0 - rs / max(radiusAtCrossingWorld, rs * 1.01)));
                    tempColor *= lerp(0.1, 0.5, redshiftFactor);

                    accumulatedColor += tempColor * diskOpacity * transmittance;
                    transmittance *= (1.0 - diskOpacity);
                    if (transmittance < 0.01)
                    {
                        color = accumulatedColor;
                        return;
                    }
                }
            }
        }

        if (!insideAnyInfluence && step > 5)
        {
            color = accumulatedColor + _SkyBox.Sample(sampler_SkyBox, dir).rgb * transmittance;
            return;
        }
    }

    color = accumulatedColor + _SkyBox.Sample(sampler_SkyBox, dir).rgb * transmittance;
}
#endif