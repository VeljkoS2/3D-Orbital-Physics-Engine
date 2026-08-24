Shader "Custom/BlackHoleLensingPassthrough"
{   
    SubShader
    {
        HLSLINCLUDE
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
        #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareDepthTexture.hlsl"
        //#include "Assets/Shaders/Includes/BlackHoleRaymarch.hlsl"
        //#include "Assets/Shaders/Includes/BlackHoleOption2.hlsl"
        //#include "Assets/Shaders/Includes/BlackHoleOptionKerr.hlsl"
        #include "Assets/Shaders/Includes/BlackHoleKerrLUT.hlsl"
        ENDHLSL
        Tags { "RenderType"="Opaque" }
        LOD 100
        ZWrite Off Cull Off
        Pass
        {
            Name "BlackHoleLensingPassthrough"
            HLSLPROGRAM
            
            #pragma vertex Vert
            #pragma fragment Frag
            int _BHCount;
            float4 _BHPositionsCamRelative[8];
            float4 _BHParams[8];
            float4 _BHDiskNormals[8];
            float4 _BHRotationOffsets[8];
            TEXTURECUBE(_BHSkybox); SAMPLER(sampler_BHSkybox);
            TEXTURE2D(_BHNoise); SAMPLER(sampler_BHNoise);
            TEXTURE2D(_BHLUT); SAMPLER(sampler_BHLUT);
            TEXTURE2D(_BHCapturedLUT); SAMPLER(sampler_BHCapturedLUT);
            TEXTURE3D(_KerrRadialLUT); SAMPLER(sampler_KerrRadialLUT);
            TEXTURE3D(_KerrPolarLUT); SAMPLER(sampler_KerrPolarLUT);
            TEXTURE2D(_KerrCritHeader); SAMPLER(sampler_KerrCritHeader);

            float4 Frag (Varyings input) : SV_Target
            {
                float3 sceneColor = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_LinearClamp, input.texcoord).rgb;
                if (_BHCount == 0) return float4(sceneColor, 1);

                float3 worldPosFar = ComputeWorldSpacePosition(input.texcoord, 1.0, UNITY_MATRIX_I_VP);
                float3 rayDir = normalize(worldPosFar - _WorldSpaceCameraPos);

                int hitIndices[8];
                float hitDist[8];
                int hitCount = 0;

                for (int i = 0; i < _BHCount; i++)
                {
                    float3 toBH = _BHPositionsCamRelative[i].xyz;
                    float rs = _BHParams[i].x;
                    float lensingRadius = rs * _BHParams[i].w*5.0;

                    float distSqToCenter = dot(toBH, toBH);
                    bool cameraInside = distSqToCenter < lensingRadius * lensingRadius;

                    float tHit;
                    if (cameraInside)
                    {
                        tHit = sqrt(distSqToCenter) - lensingRadius;
                    }
                    else
                    {
                        float tca = dot(toBH, rayDir);
                        if (tca < 0) continue;
                        float d2 = distSqToCenter - tca * tca;
                        if (d2 > lensingRadius * lensingRadius) continue;
                        float thc = sqrt(lensingRadius * lensingRadius - d2);
                        tHit = tca - thc;
                        if (tHit < 0) continue;
                    }

                    hitIndices[hitCount] = i;
                    hitDist[hitCount] = tHit;
                    hitCount++;
                }

                if (hitCount == 0)
                    return float4(sceneColor, 1);

                for (int a = 1; a < hitCount; a++)
                {
                    int idxVal = hitIndices[a];
                    float distVal = hitDist[a];
                    int b = a - 1;
                    while (b >= 0 && hitDist[b] > distVal)
                    {
                        hitDist[b+1] = hitDist[b];
                        hitIndices[b+1] = hitIndices[b];
                        b--;
                    }
                    hitDist[b+1] = distVal;
                    hitIndices[b+1] = idxVal;
                }

                float3 finalColor = float3(0,0,0);
                float totalTransmittance = 1.0;
                float3 currentDir = rayDir;
                bool anyEscape = true;

                int chainLimit = min(hitCount, 8);
                [loop]
                for (int c = 0; c < chainLimit; c++)
                {
                    int idx = hitIndices[c];
                    float3 color, exitPos, exitDir; bool escaped; float trans;
                    
                   /* BlackHoleRaymarch(currentDir, _BHPositionsCamRelative[idx].xyz, _BHParams[idx].x,
                        _BHParams[idx].y, _BHParams[idx].z, _BHParams[idx].w, _BHDiskNormals[idx].xyz,
                        _BHSkybox, sampler_BHSkybox, _BHNoise, sampler_BHNoise,
                        _BHRotationOffsets[idx].x, _BHRotationOffsets[idx].y,
                        color, exitPos, exitDir, escaped, trans);*/
                        

                    /*BlackHoleRaymarch(currentDir, _BHPositionsCamRelative[idx].xyz, _BHParams[idx].x,
                        _BHParams[idx].y, _BHParams[idx].z, _BHParams[idx].w*5.0, _BHDiskNormals[idx].xyz,
                        _BHSkybox, sampler_BHSkybox, _BHNoise, sampler_BHNoise, _BHLUT, sampler_BHLUT, //_BHCapturedLUT, sampler_BHCapturedLUT,
                        _BHRotationOffsets[idx].x, _BHRotationOffsets[idx].y,
                        color, exitPos, exitDir, escaped, trans);*/

                    /*BlackHoleRaymarchKerr(currentDir, _BHPositionsCamRelative[idx].xyz, _BHParams[idx].x,
                        _BHParams[idx].y, _BHParams[idx].z, _BHParams[idx].w*5.0, _BHDiskNormals[idx].xyz, 0.99,
                        _BHSkybox, sampler_BHSkybox, _BHNoise, sampler_BHNoise,
                        _BHRotationOffsets[idx].x, _BHRotationOffsets[idx].y,
                        color, exitPos, exitDir, escaped, trans);*/

                    BlackHoleRaymarchKerrLUT(
                        currentDir,
                        _BHPositionsCamRelative[idx].xyz,
                        _BHParams[idx].x,
                        _BHParams[idx].y,
                        _BHParams[idx].z,
                        _BHParams[idx].w * 5.0,
                        _BHDiskNormals[idx].xyz,
                        0.9, // Spin ratio (a / M)
                        _BHSkybox, sampler_BHSkybox,
                        _BHNoise, sampler_BHNoise,
                        _KerrRadialLUT, sampler_KerrRadialLUT,
                        _KerrPolarLUT, sampler_KerrPolarLUT,
                        _KerrCritHeader, sampler_KerrCritHeader,
                        60.0,
                        40.0,
                        8.0,
                        _BHRotationOffsets[idx].x,
                        _BHRotationOffsets[idx].y,
                        color, exitPos, exitDir, escaped, trans
                    );  

                    finalColor += color * totalTransmittance;
                    totalTransmittance *= trans;

                    if (!escaped || totalTransmittance < 0.01)
                    {
                        anyEscape = false;
                        break;
                    }

                    currentDir = exitDir;
                }

                if (anyEscape)
                {
                    finalColor += SAMPLE_TEXTURECUBE(_BHSkybox, sampler_BHSkybox, currentDir).rgb * totalTransmittance;
                }

                return float4(finalColor, 1);
            }
            
            ENDHLSL
        }
    }
}