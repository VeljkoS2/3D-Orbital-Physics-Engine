#define MAX_STARS 100
#define AU_IN_METERS 149597870691.0

float4 _StarPositions[MAX_STARS];
float4 _StarColors[MAX_STARS];
float _StarLuminosities[MAX_STARS];
float _MetersToUnitsScale;
int _StarCount;
bool _RealLight;

void Lighting_float(float3 WorldPos, float3 Normal, float IsRingAndIsFrontFace, out float3 Lighting)
{
#if defined(SHADERGRAPH_PREVIEW)
    Lighting = saturate(dot(Normal, float3(0, 1, 0))) * float3(1, 1, 1);
#else
    float3 totalLight = float3(0, 0, 0);
    float auInUnityUnits = AU_IN_METERS / _MetersToUnitsScale;

    Light mainLight = GetMainLight(TransformWorldToShadowCoord(WorldPos));
    int additionalCount = min(GetAdditionalLightsCount(), 8);

    for (int i = 0; i < _StarCount; i++)
    {
        float3 toStar = _StarPositions[i].xyz - WorldPos;
        float dist = length(toStar);
        float distAU = max(dist / auInUnityUnits, 1e-15);
        float3 dir = toStar / max(dist, 1e-15);
        float irradiance = _StarLuminosities[i] / (distAU * distAU);
        if (!_RealLight)
            irradiance = _StarLuminosities[i];
        float softNdotl = 0;
        if (IsRingAndIsFrontFace != -1)
        {
            float3 correctedNormal = IsRingAndIsFrontFace == 1 ? Normal : -Normal;
            float ndotl = dot(correctedNormal, dir);
            softNdotl = smoothstep(-0.1, 1, ndotl);

            float backNdotl = dot(-correctedNormal, dir);
            float backSoft = smoothstep(-0.1, 1, backNdotl) * 0.6;
            softNdotl = max(softNdotl, backSoft);
        }
        else
        {
            float ndotl = dot(Normal, dir);
            softNdotl = smoothstep(-0.1, 1, ndotl);
            softNdotl = max(softNdotl, ndotl);
        }

        float shadow = 1.0;

        if (dot(dir, mainLight.direction) > 0.999)
        {
            shadow = mainLight.shadowAttenuation;
        }
        else
        {
            for (int a = 0; a < additionalCount; a++)
            {
                Light addLight = GetAdditionalLight(a, WorldPos);
                if (dot(dir, addLight.direction) > 0.999)
                {
                    shadow = addLight.shadowAttenuation;
                    break;
                }
            }
        }

        totalLight += irradiance * _StarColors[i].rgb * softNdotl * shadow;
    }
    Lighting = totalLight;
#endif
}