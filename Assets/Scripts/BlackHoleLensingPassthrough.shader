Shader "Custom/BlackHoleLensingPassthrough"
{
    SubShader
    {
        HLSLINCLUDE
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
        #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/DeclareDepthTexture.hlsl"
        #include "Assets/Shaders/Includes/BlackHoleKerrAnalytic.hlsl"

        // 1: where the GPU time goes (red = full-res exact trace, green = rebuilt from the
        //    reduced-res traces, blue = weak field only, black = untouched)
        #define BH_DEBUG_COST 0
        // 1: why traced pixels are black (set both trace downsamples to 1 to see all):
        //    dark grey = fell into a horizon, blue = photon sphere (merger mode),
        //    red = medium steps ran out, yellow = too many exact traces, magenta = too many segments
        #define BH_DEBUG_CAPTURE 0
                // 1: tint each pixel by the path its ray took (both trace downsamples = 1):
        //    blue = weak only, green = combined field, yellow = combined field + exact
        //    Kerr, red = isolated exact Kerr, magenta = other
        #define BH_DEBUG_PATH 0
        static float g_dbgTraces = 0.0;
        static float g_jetPix = 0.0;   // the low-res cell's angle (rad), in every pass: the jets are blurred to it
        static bool g_bhLowRes = false;   // set in FragLow

        #define BH_DEBUG_CAMREGION 0
        #define BH_GROUP_WEAK 1   // exact 1st-3rd order weak bend for close groups (0 = old)

        // 1: show why reconstruction failed (only meaningful with post-processing off):
        //    red = class differs, grey = neighbour not traced, yellow = transmittance,
        //    magenta = direction spread, cyan = twist, blue = disk radius, white = disk angle
        #define BH_DEBUG_RECON 0
        static int g_reconFail = 0;

        int _BHCount;
        float4 _BHSpins[8];               // x = spin a/M, y = Kepler factor, z = 1 if member of a close group, w = 1 if disk on
        float4 _BHPositionsCamRelative[8];
        float4 _BHParams[8];              // x = rs (world), y = exact Kerr sphere (Rs, 0 = merger mode), z = disk outer (Rs), w = group/accuracy sphere (Rs)
        float4 _BHDiskNormals[8];
        float4 _BHRotationOffsets[8];     // x = static texture angle, y = clock fraction, z = clock integer part
        float4 _BHAppearanceProperties1[8];
        float4 _BHAppearanceProperties2[8];
        float4 _BHSpinJ[8];               // xyz = J = a M s^ (world^2), w = m = Rs / 2 (world)  (C#)
        float4 _BHNoiseSize;
        TEXTURECUBE(_BHSkybox); SAMPLER(sampler_BHSkybox);
        TEXTURE2D(_BHNoise); SAMPLER(sampler_BHNoise);

        float DomainRadius(int i) { return _BHParams[i].x * _BHParams[i].w; }

        // Isotropic <-> Boyer-Lindquist radius (Rs units, M = 1/2).
        float IsoToBL(float rho) { return rho * (1.0 + 0.25 / rho) * (1.0 + 0.25 / rho); }
        float BLToIso(float r)  { return 0.5 * (r - 0.5 + sqrt(max(r * (r - 1.0), 0.0))); }

        float3 SpinJ(int i) { return _BHSpinJ[i].xyz; }

                // Turn d by the kick vector (its length is the bend ANGLE). normalize(d + kick)
        // turns by atan(angle): tested, 1.1 px error at b = 15 Rs, 4.6 px at 10 Rs.
        float3 ApplyKick(float3 d, float3 k)
        {
            float th = length(k);
            if (th < 1e-12) return d;
            float3 kh = k - dot(k, d) * d;
            float kl = length(kh);
            if (kl < 1e-12) return d;
            kh /= kl;
            float s, c;
            sincos(th, s, c);
            return d * c + kh * s;
        }

        // h1 - h0 - t1 g0 = integral over [0, t1] of g(t) - g(0), g = (t - tca) / h, h = sqrt((t - tca)^2 + b^2):
        // the sideways drift of a segment (times the bend's strength). Written without subtracting h1 and h0:
        // for a hole 1e9 units away they are ~1e9 with 128-unit float steps, and the short leg from the camera
        // to a small hole near it got a drift of tens of units instead of ~1e-6 -- that hole's exact trace then
        // started up to a few Rs off, differently per pixel and per frame (its image doubled, speckled, flickered).
        // = (t1 / (h1 + h0))^2 N / h0, N = b^2 + h0 h1 + tca s1 >= b^2 (when tca s1 < 0 the last two cancel:
        // h0 h1 + tca s1 = b^2 (tca^2 + s1^2 + b^2) / (h0 h1 - tca s1)).
        float WeakDrift(float t1, float tca, float b, float h0, float s1, float h1)
        {
            float N = (tca * s1 < 0.0) ? b * b * (1.0 + (tca * tca + s1 * s1 + b * b) / (h0 * h1 - tca * s1))
                                       : b * b + h0 * h1 + tca * s1;
            float q = t1 / (h1 + h0);
            return q * q * N / h0;
        }

        // Weak-field bend of a hole on the straight segment origin + t d, t in [0, t1]
        // (t1 may be infinite): direction kick and sideways displacement at t1.
        //  Mass: leading term exact along the segment, alpha(t) = (Rs/b)[g(t) - g(0)].
        //    Higher-order terms (15pi/16 (Rs/b)^2 ...) build up near the closest
        //    approach. hoR: if the line passes within hoR of the hole (b < hoR), that
        //    closest approach lies inside the combined-field region, which already
        //    produces them -> leading order only (tested: 0.75 px boundary jump instead
        //    of 3.3 px). Otherwise they use y = Rs / (distance to THIS segment).
        //  Spin: kick = 2 d x integral B dt, scale-free closed form.
        void WeakKick(float3 d, float3 toBH, float rs, float3 J, float t1, float hoR,
                      out float3 kick, out float3 disp)
        {
            float tca = dot(toBH, d);
            float3 pOff = tca * d - toBH;
            float b = max(length(pOff), 1e-6);
            float3 bh = pOff / b;
            float3 toward = -bh;
            bool toInf = (t1 > 1e30);
            float tcl = toInf ? max(tca, 0.0) : clamp(tca, 0.0, t1);
            float rSeg = max(length(toBH - d * tcl), b);
            float x = rs / b;
            float y = (b < hoR) ? 0.0 : rs / rSeg;
            float C = x * (1.0 + y * ((15.0 * PI / 32.0) + y * ((8.0 / 3.0) + y * (3465.0 * PI / 2048.0))));
            float h0 = sqrt(tca * tca + b * b);
            float g0 = -tca / h0;
            float s1 = t1 - tca;
            float h1 = toInf ? 1.0 : sqrt(s1 * s1 + b * b);
            float g1 = toInf ? 1.0 : s1 / h1;
            kick = C * (g1 - g0) * toward;
            disp = float3(0, 0, 0);
            if (!toInf)
                disp = C * WeakDrift(t1, tca, b, h0, s1, h1) * toward;

            float u0 = -tca / b;
            float q0 = h0 / b;
            float iq03 = 1.0 / (q0 * q0 * q0);
            float A0 = u0 * (2.0 * u0 * u0 + 3.0) * iq03;
            float C0 = u0 / q0;
            float A1 = 2.0, Q1 = 0.0, C1 = 1.0;
            if (!toInf)
            {
                float u1 = s1 / b;
                float q1 = h1 / b;
                float iq13 = 1.0 / (q1 * q1 * q1);
                A1 = u1 * (2.0 * u1 * u1 + 3.0) * iq13;
                Q1 = iq13;
                C1 = u1 / q1;
            }
            float3 I = (dot(J, bh) * (A1 - A0) - dot(J, d) * (Q1 - iq03)) * bh - (C1 - C0) * J;
            // second-order spin-mass coupling (10 pi a M^2 / b^3): tested vs exact Kerr
            kick += (2.0 / (b * b)) * (1.0 + (5.0 * PI / 4.0) * x) * cross(d, I);
        }
                // The weak formula assumes the ray comes from infinity; from a camera at distance
        // r0 it over-bends by Rs^2 / (b r0) (fitted vs exact Kerr, r0 = 40-200 Rs; tested:
        // camera at 40 Rs, b = 12 Rs: 2.36 -> 0.17 px). Only for full lines starting at the
        // camera. Returns an extra kick (away from the hole).
        // t1: the line ends there (3e38: infinity) -- only the part of the bend made before it (g1 - g0 of 1 - g0)
        // (scaled per hole by C# so that, where the straight view meets the exact trace, both bend alike)
        float3 NearCameraTermTo(float3 d, float3 toBH, float rs, float t1, int h)
        {
            float tca = dot(toBH, d);
            float3 pOff = tca * d - toBH;
            float b = max(length(pOff), 1e-6);
            float r0 = max(length(toBH), 1e-6);
            float g0 = -tca / sqrt(tca * tca + b * b);
            float s1 = t1 - tca;
            float g1 = (t1 > 1e30) ? 1.0 : s1 * rsqrt(s1 * s1 + b * b);
            float sc = (_BHJet[h].y > 0.0) ? _BHJet[h].y : 1.0;
            return (sc * rs * rs / (b * r0)) * 0.5 * (g1 - g0) * (pOff / b);
        }
        float3 NearCameraTerm(float3 d, float3 toBH, float rs, int h) { return NearCameraTermTo(d, toBH, rs, 3.0e38, h); }

                // bent by every other hole up to i's closest approach (the ray reaches i displaced
        // by the holes it passed first). Bending each hole along the unbent line gave a
        // seam at every separate hole's region edge: tested 12-70 px -> <= 1.6 px.
        float3 CoupledWeakKick(int i, float3 p, float3 d, int count, uint mask, bool fromCam, bool touchMed)
        {
            float3 toI = _BHPositionsCamRelative[i].xyz - p;
            float ti = dot(toI, d);
            float3 d2 = d;
            float3 p2 = p;
            if (ti > 0.0)
            {
                float3 k = float3(0, 0, 0);
                float3 disp = float3(0, 0, 0);
                [loop]
                for (int j = 0; j < count; j++)
                {
                    if (j == i || ((mask >> (uint)j) & 1u) == 0u) continue;
                    float hoRj = (touchMed && _BHSpins[j].z > 0.5) ? DomainRadius(j) : 0.0;
                    float3 kj, dj;
                    WeakKick(d, _BHPositionsCamRelative[j].xyz - p, _BHParams[j].x, SpinJ(j), ti, hoRj, kj, dj);
                    k += kj;
                    disp += dj;
                }
                d2 = ApplyKick(d, k);
                // the bent line, re-anchored at its point nearest the start (only the parts across it: a point
                // ~alpha^2 / 2 * ti further along it put the far hole's bend ~1e-4 rad off)
                p2 = p + (d - d2 * dot(d, d2)) * ti + (disp - d2 * dot(disp, d2));
            }
            float hoRi = (touchMed && _BHSpins[i].z > 0.5) ? DomainRadius(i) : 0.0;
            float3 toI2 = _BHPositionsCamRelative[i].xyz - p2;
            float3 ki, di;
            WeakKick(d2, toI2, _BHParams[i].x, SpinJ(i), 3.0e38, hoRi, ki, di);
            if (fromCam)
                ki += NearCameraTerm(d2, toI2, _BHParams[i].x, i);
            return ki;
        }

                // 2nd-order bend (+ end displacement) of a straight segment from its own start, for a
        // mass term of weight w (rs^2, or 2 rs_i rs_j for a pair cross term) centred at c
        void SecondOrderKick(float3 p, float3 d, float t1, float3 c, float w, inout float3 kick, inout float3 disp)
        {
            float3 cc = c - p;
            float tca = dot(cc, d);
            float3 pOff = tca * d - cc;
            float b = max(length(pOff), 1e-6);
            float3 away = pOff / b;
            float ib2 = 1.0 / (b * b);
            float s0 = -tca;
            float r0 = sqrt(tca * tca + b * b);
            float G0 = -15.0 / 16.0 * atan(s0 / b) * ib2 + s0 / (16.0 * b * r0 * r0);
            float G1;
            if (t1 > 1e30)
                G1 = -15.0 * PI / 32.0 * ib2 + 1.0 / (b * r0);
            else
            {
                float s1 = t1 - tca;
                float r1 = sqrt(s1 * s1 + b * b);
                float ir13 = 1.0 / (r1 * r1 * r1);
                G1 = -15.0 / 16.0 * atan(s1 / b) * ib2 - 15.0 * s1 / (16.0 * b * r1 * r1)
                   + s1 * (2.0 * b * b + s1 * s1) * ir13 / (b * r0) - b * s0 * ir13 / r0;
                float K0 = -15.0 / 16.0 * s0 * atan(s0 / b) * ib2;
                float K1 = -15.0 / 16.0 * s1 * atan(s1 / b) * ib2 + s1 * (s1 - s0) / (b * r0 * r1);
                disp += w * (K1 - K0 - G0 * (s1 - s0)) * away;
            }
            kick += w * (G1 - G0) * away;
        }

        // antiderivative of the exact 3rd-order bend, b = 1, fresh start at u0
        float F3(float u, float u0)
        {
            float iR = rsqrt(1.0 + u * u), iR0 = rsqrt(1.0 + u0 * u0);
            float iR2 = iR * iR, iR3 = iR2 * iR, iR4 = iR2 * iR2, iR5 = iR4 * iR, iR02 = iR0 * iR0;
            float n = -64.0 * u * iR + 7.0 * u * iR * iR02 + 30.0 * u * iR2 * iR0 - 32.0 * u * iR3
                    + 7.0 * u * iR3 * iR02 - 31.0 * u0 * iR3 * iR02 + 30.0 * u * iR4 * iR0
                    - 30.0 * u0 * iR4 * iR0 + 24.0 * u * iR5 - 48.0 * u * iR5 * iR02 + 48.0 * u0 * iR5 * iR02;
            return atan(u) * (15.0 / 8.0 * iR0 + 15.0 / 16.0 * iR3) - 15.0 / 16.0 * atan(u0) * iR3 + n / 16.0;
        }

        // 3rd order (exact from the segment start) + 4th order of a group's total mass rsT at c
        void GroupHighOrder(float3 p, float3 d, float t1, float3 c, float rsT, inout float3 kick)
        {
            float3 cc = c - p;
            float tca = dot(cc, d);
            float3 pOff = tca * d - cc;
            float b = max(length(pOff), 1e-6);
            float3 away = pOff / b;
            float x = rsT / b;
            float u0 = -tca / b;
            float R0 = sqrt(1.0 + u0 * u0);
            bool toInf = (t1 > 1e30);
            float u1 = toInf ? 0.0 : (t1 - tca) / b;
            float F1 = toInf ? (15.0 * PI / (16.0 * R0) - 4.0 + 7.0 / (16.0 * R0 * R0)) : F3(u1, u0);
            float y1p = toInf ? -(1.0 - u0 / R0) : -(u1 * rsqrt(1.0 + u1 * u1) - u0 / R0);
            float a3 = F1 - F3(u0, u0) - y1p * y1p * y1p / 3.0;   // tan -> angle at 3rd order
            float P0 = u0 * (3.0 + 2.0 * u0 * u0) / (4.0 * R0 * R0 * R0);
            float R1 = sqrt(1.0 + u1 * u1);
            float P1 = toInf ? 0.5 : u1 * (3.0 + 2.0 * u1 * u1) / (4.0 * R1 * R1 * R1);
            kick += (x * x * x * a3 - (3465.0 * PI / 1024.0) * x * x * x * x * (P1 - P0)) * away;
        }

        // weak bend of the group members in gMask on the segment p + t d, t in [0, t1]
        void GroupWeakKick(float3 p, float3 d, float t1, int count, uint gMask, out float3 kick, out float3 disp)
        {
            kick = float3(0, 0, 0);
            disp = float3(0, 0, 0);
            [loop]
            for (int i = 0; i < count; i++)
            {
                if (((gMask >> (uint)i) & 1u) == 0u) continue;
                float rsI = _BHParams[i].x;
                float3 Pi = _BHPositionsCamRelative[i].xyz;
                float3 kw, dw;
                WeakKick(d, Pi - p, rsI, SpinJ(i), t1, 3.0e38, kw, dw);   // 1st order + spin only
                kick += kw;
                disp += dw;
                SecondOrderKick(p, d, t1, Pi, rsI * rsI, kick, disp);
                float rsT = rsI;
                float3 cSum = rsI * Pi;
                bool firstOfGroup = true;
                [loop]
                for (int j = 0; j < count; j++)
                {
                    if (j == i || ((gMask >> (uint)j) & 1u) == 0u || _BHSpins[j].z != _BHSpins[i].z) continue;
                    float rsJ = _BHParams[j].x;
                    float3 Pj = _BHPositionsCamRelative[j].xyz;
                    if (j > i) SecondOrderKick(p, d, t1, 0.5 * (Pi + Pj), 2.0 * rsI * rsJ, kick, disp);
                    else firstOfGroup = false;
                    rsT += rsJ;
                    cSum += rsJ * Pj;
                }
                if (firstOfGroup)
                    GroupHighOrder(p, d, t1, cSum / rsT, rsT, kick);
            }
        }

        float SceneEyeDepth(float2 uv)
        {
            float raw = SampleSceneDepth(uv);
        #if UNITY_REVERSED_Z
            if (raw <= 1e-7) return 3.0e38;
        #else
            if (raw >= 1.0 - 1e-7) return 3.0e38;
        #endif
            return LinearEyeDepth(raw, _ZBufferParams);
        }

        #define BH_UPS_MAX_SPREAD 8.0   // max direction spread, in low-res texel angles
        #define BH_UPS_MAX_TWIST  1.0   // max interpolation error, in units of 1/4 screen pixel
    #if defined(BH_VOLUMETRIC_DISK)
        #define BH_UPS_MAX_DT     0.8   // soft volumetric gas interpolates fine
    #else
        #define BH_UPS_MAX_DT     0.4
    #endif
        #define BH_UPS_MAX_DPHI   1.0
        #define BH_UPS_MAX_DR     0.3

        float4 _BHObserverTint;
        float3 ObserverShift(float3 c) { return c * _BHObserverTint.rgb; }

                float4 _BHSkyboxSize; // x = cubemap face size in texels (C#)

        // Mip level for a pixel that sees `footprint` radians of sky: lensing can squeeze
        // a large patch of sky into one pixel, which at mip 0 sparkles and thrashes the
        // texture cache. (Needs mipmaps + trilinear filtering on the skybox.)
        float SkyLod(float footprint)
        {
            float texelAngle = (0.5 * PI) / max(_BHSkyboxSize.x, 1.0);
            return clamp(log2(max(footprint, 1e-9) / texelAngle), 0.0, 8.0);
        }

        float3 SampleSky(float3 dir, float footprint)
        {
            return SAMPLE_TEXTURECUBE_LOD(_BHSkybox, sampler_BHSkybox, dir, SkyLod(footprint)).rgb;
        }

        Texture2D<float4> _BHLow0;
        Texture2D<float4> _BHLow1;
        Texture2D<float4> _BHLow2;
        float4 _BHLowSize;
        float _BHDownsample;

        #define BH_WHY_NONE      0
        #define BH_WHY_HORIZON   1
        #define BH_WHY_PHOTONSPH 2
        #define BH_WHY_STEPS     3
        #define BH_WHY_KERR      4
        #define BH_WHY_EVENTS    5

        struct ChainResult
        {
            float3 emAll;
            float3 emFirst;
            float  T;
            float3 dir;
            bool   captured;
            bool   anyStrong;
            int    traces;
            int    firstHole;
            bool   hasFirst;
            float4 firstCross;
            float  firstIdx;
            int    why;
            int    path;      // bits: 1 combined field, 2 exact Kerr inside a group, 4 isolated exact Kerr
        };

        ChainResult InitChain(float3 dir)
        {
            ChainResult cr;
            cr.emAll = 0; cr.emFirst = 0; cr.T = 1.0; cr.dir = dir;
            cr.captured = false; cr.anyStrong = false; cr.traces = 0; cr.firstHole = -1;
            cr.hasFirst = false; cr.firstCross = 0; cr.firstIdx = 0; cr.why = BH_WHY_NONE;
            cr.path = 0;
            return cr;
        }

        float3 DebugPathTint(int path, float3 img)
        {
            float3 t = (path == 0)       ? float3(0.2, 0.4, 1.0)
                     : (path == 1)       ? float3(0.2, 1.0, 0.3)
                     : ((path & 2) != 0) ? float3(1.0, 0.9, 0.2)
                     : ((path & 4) != 0) ? float3(1.0, 0.3, 0.2)
                     :                     float3(1.0, 0.0, 1.0);
            return t * (0.25 + saturate(dot(img, float3(0.2126, 0.7152, 0.0722))));
        }

        float3 PixelRayDir(float2 uv)
        {
            float3 worldPosFar = ComputeWorldSpacePosition(uv, 1.0, UNITY_MATRIX_I_VP);
            return normalize(worldPosFar - _WorldSpaceCameraPos);
        }

        int CameraHole(int count)
        {
            int camHole = -1;
            [loop]
            for (int c = 0; c < count; c++)
            {
                float3 toC = _BHPositionsCamRelative[c].xyz;
                float Rc = DomainRadius(c);
                if (dot(toC, toC) < Rc * Rc) camHole = c;
            }
            return camHole;
        }

        float FirstDomainEntry(float3 dir, int count, int camHole)
        {
            float tFirst = (camHole >= 0) ? 0.0 : 3.0e38;
            [loop]
            for (int j = 0; j < count; j++)
            {
                float3 toBHc = _BHPositionsCamRelative[j].xyz;
                float tcaC = dot(toBHc, dir);
                float m2c = max(dot(toBHc, toBHc) - tcaC * tcaC, 0.0);
                float Rj = DomainRadius(j);
                if (tcaC > 0.0 && m2c < Rj * Rj)
                    tFirst = min(tFirst, tcaC - sqrt(Rj * Rj - m2c));
            }
            return tFirst;
        }

        float3 WeakOnlyDir(float3 dir, int count)
        {
            uint allMask = (1u << (uint)count) - 1u;
            float3 kick = float3(0, 0, 0);
        #if BH_GROUP_WEAK
            uint gMask = 0u;
            [loop]
            for (int g = 0; g < count; g++)
                if (_BHSpins[g].z > 0.5) gMask |= 1u << (uint)g;
            float3 disp;
            GroupWeakKick(float3(0, 0, 0), dir, 3.0e38, count, gMask, kick, disp);
            [loop]
            for (int w = 0; w < count; w++)
                if (_BHSpins[w].z <= 0.5)
                    kick += CoupledWeakKick(w, float3(0, 0, 0), dir, count, allMask, true, false);
        #else
            [loop]
            for (int w = 0; w < count; w++)
                kick += CoupledWeakKick(w, float3(0, 0, 0), dir, count, allMask, true, false);
        #endif
            return ApplyKick(dir, kick);
        }

        // ==================================================================
        // Relativistic jets on straight pieces of a ray (shape, light, beaming: BlackHoleKerrAnalytic.hlsl;
        // near a hole that is traced exactly they are sampled along its geodesic). On a straight piece the beam
        // is integrated in closed form: only where the line is within JET_WIN beam widths (a quadratic in t),
        // cut into pieces evenly in log z (the light ~ z^-0.9 / width^2 varies little across one), each with
        // its width and light fixed and the Gaussians across it exact (erf). Crossing the beam: one piece;
        // running along it: up to 16. Tested vs fine sampling (400 random rays): median 1.000, 5-95 %
        // 0.94-1.07; split anywhere into two pieces: 0.96-1.06 (8-point Gauss-Legendre: 0.0-3.5).
        // ==================================================================
        #define JET_WIN 6.0
        float JetErf(float x)
        {
            float ax = min(abs(x), 9.0);
            float t = 1.0 / (1.0 + 0.3275911 * ax);
            float y = 1.0 - t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))))
                          * exp(-ax * ax);
            return (x >= 0.0) ? y : -y;
        }
        // a Gaussian beam exp(-rho^2 / 2 s^2) along the line over [ta, tb], rho^2 = A (t - tS)^2 + b2
        float JetGauss(float s, float A, float tS, float b2, float ta, float tb)
        {
            float k = sqrt(A) * 0.70710678 / s;
            float x1 = (ta - tS) * k, x2 = (tb - tS) * k;
            float d = (abs(x2 - x1) < 1e-2) ? 1.1283792 * exp(-0.25 * (x1 + x2) * (x1 + x2)) * (x2 - x1)
                                             : JetErf(x2) - JetErf(x1);
            return exp(-0.5 * b2 / (s * s)) * (0.88622693 * d / k);
        }

        // the lobe sgn (+1 / -1 along the axis) of hole h's jet on o + t d, t in [t0, t1]; fadeR > 0: faded in
        // over fadeR / 2 .. fadeR from the hole (inside, its exact trace sampled it along the geodesic)
        float3 JetSeg(int h, float3 o, float3 d, float t0, float t1, float sgn, float fadeR)
        {
            // in Rs from here
            float irs = 1.0 / _BHParams[h].x;
            float3 n = _BHDiskNormals[h].xyz * sgn;
            float3 rel = (o - _BHPositionsCamRelative[h].xyz) * irs;
            float z0 = dot(rel, n), dz = dot(d, n);
            float zMin = _BHJetParams.w, zMax = _BHJetParams.z;
            float lo = t0 * irs, hi = t1 * irs;
            if (abs(dz) < 1e-6)
            {
                if (z0 < zMin || z0 > zMax) return float3(0, 0, 0);
            }
            else
            {
                float ta = (zMin - z0) / dz, tb = (zMax - z0) / dz;
                lo = max(lo, min(ta, tb));
                hi = min(hi, max(ta, tb));
            }
            if (hi <= lo) return float3(0, 0, 0);
            // distance from the axis: rho^2 = A (t - tS)^2 + b2 (perpendicular parts: no cancellation)
            float3 rP = rel - z0 * n;
            float3 dP = d - dz * n;
            float A = max(dot(dP, dP), 1e-6);
            float tS = -dot(rP, dP) / A;
            float3 cx = cross(rP, dP);
            float b2 = dot(cx, cx) / A;
            // where it is within JET_WIN widths (width <= tan(open) (z + 20) + 0.3): a t^2 + 2 b t + c <= 0
            float tanO = _BHJetParams.x;
            float dist0 = length(o) * irs;   // (camera-relative: the footprint grows <= g_jetPix (dist0 + t))
            float w0 = JET_WIN * (tanO * (z0 + 20.0) + 0.3 + g_jetPix * dist0);
            float w1 = JET_WIN * (tanO * dz + g_jetPix);
            float qa = A - w1 * w1;
            float qb = -tS * A - w0 * w1;
            float qc = b2 + A * tS * tS - w0 * w0;
            float disc = qb * qb - qa * qc;
            // up to two parts (outside the cone's waist the line can be inside it on both sides)
            float lo2 = 1.0, hi2 = 0.0;
            if (qa > 1e-7)
            {
                if (disc <= 0.0) return float3(0, 0, 0);
                float sq = sqrt(disc);
                lo = max(lo, (-qb - sq) / qa);
                hi = min(hi, (-qb + sq) / qa);
            }
            else if (qa < -1e-7 && disc > 0.0)
            {
                float sq = sqrt(disc);
                float r1 = (-qb + sq) / qa, r2 = (-qb - sq) / qa;   // inside: t <= r1 or t >= r2
                lo2 = max(lo, r2);
                hi2 = hi;
                hi = min(hi, r1);
            }

            float sinP = sqrt(A);
            float knotAmt = smoothstep(0.15, 0.35, sinP);
            float fR = fadeR * irs;
            // the crossing's length scale (sheath)
            float lCross = JetWidthRs(clamp(z0 + tS * dz, zMin, zMax)) / sinP;
            float2 acc = float2(0, 0);
            [loop]
            for (int part = 0; part < 2; part++)
            {
                float pa = (part == 0) ? lo : lo2, pb = (part == 0) ? hi : hi2;
                if (pb <= pa) continue;
                float za = z0 + pa * dz, zb = z0 + pb * dz;
                bool useLog = abs(dz) > 0.05;
                float lr = useLog ? log(zb / za) : 0.0;
                // pieces of 1/3 in log z: n whole ones and a remainder that grows from 0 (continuous)
                float Kf = useLog ? clamp(abs(lr) * 2.5, 1.0, 10.0) : 1.0;
                int K = (int)ceil(Kf - 1e-4);
                float wP = 1.0 / Kf;                  // (a whole piece, as a fraction of the part)
                [loop]
                for (int k = 0; k < K; k++)
                {
                    float u0 = (float)k * wP, u1 = min((float)(k + 1) * wP, 1.0);
                    float ta, tb;
                    if (useLog)
                    {
                        float z1 = za * exp(lr * u0), z2 = za * exp(lr * u1);
                        ta = (z1 - z0) / dz;
                        tb = (z2 - z0) / dz;
                        if (ta > tb) { float sw = ta; ta = tb; tb = sw; }
                    }
                    else
                    {
                        ta = pa;
                        tb = pb;
                    }
                    // crossing the beam within the piece: the Gaussians across it exact (erf), width and light
                    // frozen at the crossing; running along it (the crossing longer than the piece): 3-point
                    // Gauss-Legendre of the beam itself (smooth there, tails too). Blended by how resolved the
                    // crossing is, so nothing depends on the ill-conditioned closest approach of a near-parallel
                    // line.
                    float wc = smoothstep(0.25, 1.0, (tb - ta) / lCross);
                    [branch]
                    if (wc > 0.0)
                    {
                        float ts = clamp(tS, ta, tb);
                        float zm = z0 + ts * dz;
                        float sg = JetWidthRs(zm);
                        // blurred to the pixel's footprint there (variances add; the light is kept)
                        float fp = g_jetPix * length(o * irs + d * ts);
                        float f2 = fp * fp;
                        float sg2 = sg * sg, s22 = JET_HOLLOW * JET_HOLLOW * sg2;
                        float ss2 = JET_SPINE * JET_SPINE * sg2 + f2, ge = sg2 + f2, he = s22 + f2;
                        float2 g = float2(JetGauss(sqrt(ss2), A, tS, b2, ta, tb) / ss2,
                                          (sg2 * JetGauss(sqrt(ge), A, tS, b2, ta, tb) / ge
                                           - s22 * JetGauss(sqrt(he), A, tS, b2, ta, tb) / he) / (sg2 - s22));
                        float fadeK = 1.0;
                        if (fR > 0.0)
                            fadeK = smoothstep(0.5 * fR, fR, sqrt(b2 + zm * zm));
                        acc += JetLine1(zm, zMin, zMax, sgn, knotAmt) * g * (0.159155 * fadeK * wc);
                    }
                    [branch]
                    if (wc < 1.0)
                    {
                        [unroll]
                        for (int q = 0; q < 3; q++)
                        {
                            float xq = (q == 0) ? 0.1127017 : (q == 1) ? 0.5 : 0.8872983;
                            float wq = ((q == 1) ? 0.4444444 : 0.2777778) * (tb - ta);
                            float tq = lerp(ta, tb, xq);
                            if (useLog)
                            {
                                // (nodes evenly in log z: tq = (z - z0) / dz, dt = z dlog z / dz)
                                float zA = z0 + ta * dz, zB = z0 + tb * dz;
                                float lz = log(zB / zA);
                                float zq = zA * exp(lz * xq);
                                tq = (zq - z0) / dz;
                                wq = ((q == 1) ? 0.4444444 : 0.2777778) * abs(zq * lz / dz);
                            }
                            float zq2 = z0 + tq * dz;
                            float rho2 = b2 + A * (tq - tS) * (tq - tS);
                            float fp = g_jetPix * length(o * irs + d * tq);
                            float fadeK = 1.0;
                            if (fR > 0.0)
                                fadeK = smoothstep(0.5 * fR, fR, sqrt(rho2 + zq2 * zq2));
                            acc += JetLine1(zq2, zMin, zMax, sgn, knotAmt) * JetProfile(rho2, JetWidthRs(zq2), fp * fp)
                                 * (wq * fadeK * (1.0 - wc));
                        }
                    }
                }
            }
            return JetRGB(acc) * (_BHJet[h].x * JetBoost(dz));
        }

        // first-order bend of a line by a mass from its start up to t1 (3e38: to infinity), with the weak-field
        // series of WeakKick (jets only: no spin terms)
        void JetBend(float3 d, float3 toJ, float rs, float t1, inout float3 k, inout float3 disp)
        {
            float tca = dot(toJ, d);
            float3 pOff = tca * d - toJ;
            float bl = length(pOff);
            float b = max(bl, 3.0 * rs);    // (weak field only: rays closer than this are traced, or fall in)
            float3 toward = -pOff / max(bl, 1e-6);
            bool toInf = (t1 > 1e30);
            float tcl = toInf ? max(tca, 0.0) : clamp(tca, 0.0, t1);
            float y = rs / max(length(toJ - d * tcl), b);
            float Cb = (rs / b) * (1.0 + y * ((15.0 * PI / 32.0) + y * ((8.0 / 3.0) + y * (3465.0 * PI / 2048.0))));
            float h0 = sqrt(tca * tca + b * b);
            float g0 = -tca / h0;
            float s1 = t1 - tca;
            float h1 = toInf ? 1.0 : sqrt(s1 * s1 + b * b);
            float g1 = toInf ? 1.0 : s1 / h1;
            k += Cb * (g1 - g0) * toward;
            if (!toInf)
                disp += Cb * WeakDrift(t1, tca, b, h0, s1, h1) * toward;
        }

        // Cheap cull: does the line o + t d (t in [t0, t1]) come within reach of hole h's jets (either lobe:
        // rho <= JET_WIN (tan(open) (|z| + 20) + 0.3) + its bending: 3 Rs + bendA t)? Most pixels: no.
        bool JetNear(int h, float3 o, float3 d, float t0, float t1, float bendA)
        {
            float irs = 1.0 / _BHParams[h].x;
            float3 n = _BHDiskNormals[h].xyz;
            float3 rel = (o - _BHPositionsCamRelative[h].xyz) * irs;
            float z0 = dot(rel, n), dz = dot(d, n);
            float3 rP = rel - z0 * n;
            float3 dP = d - dz * n;
            float A = max(dot(dP, dP), 1e-6);
            float B = dot(rP, dP);
            float C = dot(rP, rP);
            float tanO = _BHJetParams.x;
            float dist0 = length(o) * irs;
            float a0 = JET_WIN * (tanO * 20.0 + 0.3 + g_jetPix * dist0) + 3.0;
            float aw = JET_WIN * (tanO + g_jetPix);
            float ab = bendA + JET_WIN * g_jetPix;
            float lo = t0 * irs, hi = min(t1 * irs, 1e7);
            [unroll]
            for (int s = 0; s < 2; s++)
            {
                float sg = (s == 0) ? 1.0 : -1.0;
                // rho^2 <= (a0 + aw z + ab t)^2 with z = sg (z0 + t dz) >= 0
                float w0 = a0 + aw * sg * z0, w1 = aw * sg * dz + ab;
                float qa = A - w1 * w1, qb = B - w0 * w1, qc = C - w0 * w0;
                float l = lo, u = hi;
                if (abs(dz) > 1e-6)
                {
                    float tz = -z0 / dz;                      // z = 0: this lobe on one side of it
                    if (sg * dz > 0.0) l = max(l, tz); else u = min(u, tz);
                }
                else if (sg * z0 < 0.0)
                    continue;
                if (u <= l) continue;
                // the quadratic at the ends or its minimum inside: <= 0 anywhere on [l, u]?
                float fl = (qa * l + 2.0 * qb) * l + qc, fu = (qa * u + 2.0 * qb) * u + qc;
                if (fl <= 0.0 || fu <= 0.0) return true;
                if (qa > 0.0)
                {
                    float tm = -qb / qa;
                    if (tm > l && tm < u && qc - qb * qb / qa <= 0.0) return true;
                }
            }
            return false;
        }

        // The ray's first-order path through the fields of the holes in kinkMask, at s along the unbent line:
        // o + d s + the displacements of each (as a group's legs have it: chord to the displaced entry)
        float3 JetPathAt(float3 o, float3 d, float s, int count, uint kinkMask)
        {
            float3 k = float3(0, 0, 0), disp = float3(0, 0, 0);
            [loop]
            for (int j = 0; j < count; j++)
                if (((kinkMask >> (uint)j) & 1u) != 0u)
                    JetBend(d, _BHPositionsCamRelative[j].xyz - o, _BHParams[j].x, s, k, disp);
            return o + d * s + disp;
        }

        // All jets on the ray o + t d, t in [t0, t1]; far = the lobes on the other side of the hole from the camera.
        // Bent by the holes in kinkMask: each hole's jet is integrated along a polyline through the ray's
        // first-order path at s = its closest approach + b {-2, -0.7, 0, 0.7, 2} (b = the impact parameter:
        // where the bending happens; chords there are within ~0.05 Rs of the curve), then on the asymptote --
        // the medium a group's legs integrate, so the straight and the traced parts of an image meet. (One
        // kink at the closest approach put the crossing ~1 Rs off at 35 Rs: 2x at the spine's edge.)
        // Only the jets in jetMask; fadeHole's is faded in over fadeR / 2 .. fadeR (inside, its exact trace
        // sampled it along the geodesic).
        void JetLine(float3 o, float3 d, float t0, float t1, int count, uint kinkMask, uint jetMask, int fadeHole,
                     float fadeR, inout float3 nearEm, inout float3 farEm)
        {
            float3 dEnd = d;
            [branch]
            if (kinkMask != 0u)
            {
                float3 kA = float3(0, 0, 0), dA = float3(0, 0, 0);
                [loop]
                for (int j = 0; j < count; j++)
                    if (((kinkMask >> (uint)j) & 1u) != 0u)
                        JetBend(d, _BHPositionsCamRelative[j].xyz - o, _BHParams[j].x, 3.0e38, kA, dA);
                dEnd = ApplyKick(d, kA);
            }
            float bendA = length(dEnd - d);       // how far the bent path can stray from the line (angle)
            [loop]
            for (int h = 0; h < count; h++)
            {
                [branch]
                if (_BHJet[h].x <= 0.0 || ((jetMask >> (uint)h) & 1u) == 0u || !JetNear(h, o, d, t0, t1, bendA))
                    continue;
                float3 c = _BHPositionsCamRelative[h].xyz;
                float camSide = (dot(-c, _BHDiskNormals[h].xyz) >= 0.0) ? 1.0 : -1.0;
                float fr = (h == fadeHole) ? fadeR : 0.0;
                float th = dot(c - o, d);
                float bh = max(length(o + d * th - c), _BHParams[h].x);
                bool bent = (kinkMask != 0u);
                // pieces: [sA, sB] on the unbent line -> chord between the path's points (the last: asymptote)
                float sA = 0.0;
                float3 pA = o;
                [loop]
                for (int v = 0; v < 6; v++)
                {
                    float sB = 3.0e38;
                    if (bent && v < 5)
                    {
                        float f = (v == 0) ? -2.0 : (v == 1) ? -0.7 : (v == 2) ? 0.0 : (v == 3) ? 0.7 : 2.0;
                        sB = th + f * bh;
                        if (sB <= sA) continue;          // (behind the start)
                    }
                    float3 pB = pA, pd = dEnd;
                    float L = 3.0e38;
                    if (sB < 1e37)
                    {
                        pB = JetPathAt(o, d, sB, count, kinkMask);
                        float3 ch = pB - pA;
                        L = length(ch);
                        pd = ch / max(L, 1e-30);
                    }
                    // the part of [t0, t1] on this piece (lengths along the chord ~ along the unbent line)
                    float sc = (sB < 1e37) ? L / max(sB - sA, 1e-30) : 1.0;
                    float a0 = max(t0, sA), a1 = min(t1, sB);
                    [branch]
                    if (a1 > a0)
                    {
                        float l0 = (a0 - sA) * sc, l1 = (a1 >= 1e37) ? 3.0e30 : (a1 - sA) * sc;
                        nearEm += JetSeg(h, pA, pd, l0, l1, camSide, fr);
                        farEm += JetSeg(h, pA, pd, l0, l1, -camSide, fr);
                    }
                    if (sB >= t1 || sB >= 1e37)
                        break;
                    sA = sB;
                    pA = pB;
                }
            }
        }


    #if defined(BH_SINGLE)
        // Exactly one hole: exact trace to infinity, nothing else.
        void TraceChain(float3 dir, int count, int camHole, out ChainResult cr)
        {
            cr = InitChain(dir);

            float3 toBH = _BHPositionsCamRelative[0].xyz;
            float rs = _BHParams[0].x;
            float R = DomainRadius(0);
            float tca = dot(toBH, dir);
            float m2 = dot(toBH, toBH) - tca * tca;
            bool enters = (camHole == 0) || (tca > 0.0 && m2 < R * R);

            [branch]
            if (!enters)
            {
                float3 kick, disp;
                WeakKick(dir, toBH, rs, SpinJ(0), 3.0e38, 0.0, kick, disp);
                kick += NearCameraTerm(dir, toBH, rs, 0);
                cr.dir = ApplyKick(dir, kick);
                float3 jn0 = float3(0, 0, 0), jf0 = float3(0, 0, 0);
                JetLine(float3(0, 0, 0), dir, 0.0, 3.0e30, count, 1u, 1u, -1, 0.0, jn0, jf0);
                cr.emAll += jn0 + jf0;
                return;
            }

            float3 color, exitPos, exitDir; bool escaped; float trans;
            bool hf; float4 fc; float fi; float3 ef;
            g_dbgTraces += 1.0;
            g_jetFootRs = g_jetPix * length(toBH) / rs;
            KerrTraceCore(
                dir, toBH, rs,
                _BHParams[0].y, _BHParams[0].z, _BHParams[0].w,
                _BHDiskNormals[0].xyz,
                _BHSpins[0].x, _BHSpins[0].y,
                _BHNoise, sampler_BHNoise,
                _BHRotationOffsets[0].x, _BHRotationOffsets[0].yz,
                1e30, true, _BHSpins[0].w > 0.5,
                color, exitPos, exitDir, _BHAppearanceProperties1[0], _BHAppearanceProperties2[0],
                escaped, trans, hf, fc, fi, ef, 0, min(JET_RGEO, _BHParams[0].w));

            cr.firstHole = 0;
            cr.hasFirst = hf;
            cr.firstCross = fc;
            cr.firstIdx = fi;
            cr.emFirst = ef;
            cr.traces = 1;
            cr.emAll = color;
            cr.T = trans;
            cr.anyStrong = true;
            cr.path = 4;
            cr.captured = !escaped;
            if (!escaped)
                cr.why = BH_WHY_HORIZON;
            else
                cr.dir = exitDir;

            // jets on straight lines outside where the trace sampled them (min(JET_RGEO, its region), faded
            // into each other): the camera's line up to the closest approach (in front of the disk), and the
            // outgoing asymptote from there (behind it)
            [branch]
            if (_BHJet[0].x > 0.0)
            {
                float rG = min(JET_RGEO, _BHParams[0].w) * rs;
                float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                // (the same path the straight view outside its region takes: through its displaced points)
                JetLine(float3(0, 0, 0), dir, 0.0, max(tca, 0.0), count, 1u, 1u, 0, rG, jn, jf);
                cr.emAll += jn + jf;
                [branch]
                if (escaped)
                {
                    float3 on = float3(0, 0, 0), of = float3(0, 0, 0);
                    JetLine(float3(0, 0, 0), dir, max(tca, 0.0), 3.0e30, count, 1u, 1u, 0, rG, on, of);
                    cr.emAll += (on + of) * cr.T;
                }
            }
        }
    #else
        // ==================================================================
        // Multi-hole lensing: exact Kerr inside each hole's inner sphere, the combined
        // field of all holes as a refractive medium in between, weak kicks far away.
        //   n = psi^3 / chi, psi = 1 + sum m_i / 2r_i, chi = 1 - sum m_i / 2r_i
        //   dd/ds = grad_perp ln n + (2/n) d x B,  B = sum (3 (J.x^) x^ - J) / r^3
        // ==================================================================
        #define BH_MAX_EVENTS      24
        #define BH_MAX_KERR        8
        // Tested (single hole, 40 Rs region): 0.26 px error where the image is not
        // strongly lensed, 0.05 px between neighbouring rays (no banding), 7 steps per
        // ray (was 12). Merger-mode rays skimming a photon sphere: ~1 px on screen.
        #define BH_MED_STEP_NEAR   0.12
        #define BH_MED_STEP_FAR    0.48
        #define BH_MED_STEP_GROW   0.12
        #define BH_MED_MAX_STEPS   360
        #define BH_GRAVITOMAGNETIC 1
        #define BH_QUADRUPOLE 1

        bool IsMember(int i) { return _BHSpins[i].z > 0.5; }
        float InnerRadius(int i) { return _BHParams[i].x * _BHParams[i].y; }

        // dd/ds = grad_perp ln n + (2/n) d x B (traced ray), with
        //  - frame dragging w = (1 + m/r) 2 J x X / r^3 (the 1 + m/r is the lapse and
        //    radius corrections; curl includes grad f x w0),
        //  - Kerr's quadrupole: ln n += -2 M a^2 P2(cos th) / r^3.
        // Tested vs exact Kerr, spin 0.998, b = 8 Rs: 0.2-0.5 px (was 1.2-5.2 px).
        float3 MediumAccel(float3 x, float3 d, int count, uint mask)
        {
            float S = 0.0;
            float3 G = float3(0, 0, 0);
            float3 B = float3(0, 0, 0);
            float3 Q = float3(0, 0, 0);
            [loop]
            for (int i = 0; i < count; i++)
            {
                if (((mask >> (uint)i) & 1u) == 0u) continue;
                float4 sj = _BHSpinJ[i];
                float m = sj.w;
                float3 J = sj.xyz;
                float3 dx = x - _BHPositionsCamRelative[i].xyz;
                float r2 = max(dot(dx, dx), 1e-30);
                float ir = rsqrt(r2);
                float ir2 = ir * ir;
                float ir3 = ir2 * ir;
                S += m * ir;
                G += (m * ir3) * dx;
            #if BH_GRAVITOMAGNETIC
                float3 xh = dx * ir;
                float fr = 1.0 + m * ir;
                float3 gradf = -(m * ir3) * dx;
                B += (fr * (3.0 * dot(J, xh) * xh - J) + cross(gradf, cross(J, dx))) * ir3;
            #endif
            #if BH_QUADRUPOLE
                float J2 = dot(J, J);
                if (J2 > 0.0)
                {
                    float Ma2 = J2 / max(m, 1e-30);          // J = a M  ->  M a^2 = J^2 / M
                    float3 sh = J * rsqrt(J2);
                    float sx = dot(sh, dx);
                    float ir5 = ir3 * ir2;
                    float ir7 = ir5 * ir2;
                    Q -= Ma2 * ((6.0 * sx * ir5) * sh - (2.0 * ir5) * dx - (5.0 * (3.0 * sx * sx - r2) * ir7) * dx);
                }
            #endif
            }
            float psi = 1.0 + 0.5 * S;
            float chi = max(1.0 - 0.5 * S, 1e-3);
            float3 gl = -(3.0 / psi + 1.0 / chi) * 0.5 * G + Q;
            float3 acc = gl - dot(gl, d) * d;
        #if BH_GRAVITOMAGNETIC
            acc += (2.0 * chi / (psi * psi * psi)) * cross(d, B);
        #endif
            return acc;
        }

        float3 ExternalGrad(int h, int count, uint mask)
        {
            float3 x = _BHPositionsCamRelative[h].xyz;
            float S = 0.0;
            float3 G = float3(0, 0, 0);
            [loop]
            for (int i = 0; i < count; i++)
            {
                if (i == h || ((mask >> (uint)i) & 1u) == 0u) continue;
                float3 dx = x - _BHPositionsCamRelative[i].xyz;
                float ir = rsqrt(max(dot(dx, dx), 1e-30));
                float m = _BHSpinJ[i].w;
                S += m * ir;
                G += (m * ir * ir * ir) * dx;
            }
            float psi = 1.0 + 0.5 * S;
            float chi = max(1.0 - 0.5 * S, 1e-3);
            return -(3.0 / psi + 1.0 / chi) * 0.5 * G;
        }

        float3 ZamoVelocity(int h, float3 x)
        {
            float3 dx = x - _BHPositionsCamRelative[h].xyz;
            float ir = rsqrt(max(dot(dx, dx), 1e-30));
            return -2.0 * cross(SpinJ(h), dx) * (ir * ir * ir);
        }
        float3 StaticToZamo(float3 n, float3 v) { return normalize(n - v + dot(n, v) * n); }
        float3 ZamoToStatic(float3 n, float3 v) { return normalize(n + v - dot(n, v) * n); }

        // RK4 through the medium. Returns
        // 0: left every group sphere, 1: reached the exact sphere of hitHole,
        // 2: inside a photon sphere and falling in (merger mode), 3: step budget used up.
        // Jets along the way: on secants of up to 8 RK4 steps, cut where the path turns by ~0.01 rad.
        int IntegrateMedium(inout float3 p, inout float3 d, int count, uint mask, out int hitHole,
                            inout ChainResult cr, inout bool anyEm)
        {
            hitHole = -1;
            int ret = 3;
            float3 jA = p;
            float3 jDA = d;
            int jK = 0;
            bool jetsOn = false;
            [loop]
            for (int jh = 0; jh < count; jh++)
                jetsOn = jetsOn || (_BHJet[jh].x > 0.0);
            [loop]
            for (int s = 0; s < BH_MED_MAX_STEPS; s++)
            {
                float rMin = 3.0e38;
                float rsNear = 1.0;
                float dIn = 3.0e38;
                bool inside = false;
                bool falling = false;
                [loop]
                for (int i = 0; i < count; i++)
                {
                    if (((mask >> (uint)i) & 1u) == 0u) continue;
                    float3 dx = p - _BHPositionsCamRelative[i].xyz;
                    float r = length(dx);
                    float rsI = _BHParams[i].x;
                    if (r < rMin) { rMin = r; rsNear = rsI; }
                    if (IsMember(i))
                    {
                        inside = inside || (r < DomainRadius(i));
                        float Rin = InnerRadius(i);
                        if (Rin > 0.0)
                            dIn = min(dIn, r - Rin);
                        else if (r < 0.3 * rsI || (r < 0.6 * rsI && dot(d, dx) < 0.0))
                        {
                            falling = true;
                            break;
                        }
                    }
                }
                bool stop = false;
                if (falling) { ret = 2; stop = true; }
                else if (!inside) { ret = 0; stop = true; }
                else
                {
                    float k = clamp(BH_MED_STEP_GROW * sqrt(rMin / rsNear), BH_MED_STEP_NEAR, BH_MED_STEP_FAR);
                    float h = min(k * rMin, max(dIn, 0.1 * rMin));

                    float3 p0 = p;
                    float3 d0 = d;
                    float3 a1 = MediumAccel(p0, d0, count, mask);
                    float3 d2 = normalize(d0 + 0.5 * h * a1);
                    float3 a2 = MediumAccel(p0 + 0.5 * h * d0, d2, count, mask);
                    float3 d3 = normalize(d0 + 0.5 * h * a2);
                    float3 a3 = MediumAccel(p0 + 0.5 * h * d2, d3, count, mask);
                    float3 d4 = normalize(d0 + h * a3);
                    float3 a4 = MediumAccel(p0 + h * d3, d4, count, mask);
                    p = p0 + (h / 6.0) * (d0 + 2.0 * d2 + 2.0 * d3 + d4);
                    d = normalize(d0 + (h / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4));

                    // entered an exact sphere: shorten the step to its surface
                    [loop]
                    for (int k2 = 0; k2 < count; k2++)
                    {
                        if (!IsMember(k2)) continue;
                        float R = InnerRadius(k2);
                        if (R <= 0.0) continue;
                        float3 c = _BHPositionsCamRelative[k2].xyz;
                        float R2 = R * R * (1.0 - 2e-4);
                        float3 f0 = p0 - c;
                        float3 f1 = p - c;
                        if (dot(f1, f1) < R2 && dot(f0, f0) >= R2)
                        {
                            float3 e = p - p0;
                            float qa = dot(e, e);
                            float qb = 2.0 * dot(f0, e);
                            float qc = dot(f0, f0) - R2;
                            float t = saturate((-qb - sqrt(max(qb * qb - 4.0 * qa * qc, 0.0))) / max(2.0 * qa, 1e-30));
                            p = p0 + t * e;
                            d = normalize(lerp(d0, d, t));
                            hitHole = k2;
                            ret = 1;
                            stop = true;
                            break;
                        }
                    }
                    jK++;
                }
                // jets on the secant since jA (always before leaving)
                [branch]
                if (jetsOn && (stop || jK >= 16 || dot(d, jDA) < 0.9999 || s == BH_MED_MAX_STEPS - 1))
                {
                    float3 chord = p - jA;
                    float cl = length(chord);
                    [branch]
                    if (cl > 0.0)
                    {
                        float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                        JetLine(jA, chord / cl, 0.0, cl, count, 0u, mask, -1, 0.0, jn, jf);
                        cr.emAll += (jn + jf) * cr.T;
                    }
                    jA = p;
                    jDA = d;
                    jK = 0;
                }
                if (stop)
                    break;
            }
            return ret;
        }

                // Holes whose regions don't overlap: one exact trace each (visit order is
        // unambiguous, so no seam). Overlapping regions (a group): the combined-field
        // integration, entered and left in plain local directions, with leading-order
        // weak segments on both sides (both tested against exact integration).
        void TraceChain(float3 dir, int count, int camHole, out ChainResult cr)
        {
            cr = InitChain(dir);
            bool anyEm = false;

            float3 p = float3(0, 0, 0);
            float3 d = dir;
            uint allMask = (1u << (uint)count) - 1u;
            uint done = 0u;
            uint handed = 0u;             // holes whose trace was stopped once at a nested satellite's distance
            uint noKick = 0u;             // ... and not yet traced on: their bend is the traces' (no weak kicks)
            int hoHole = -1;              // ... and the image order its continuation starts at
            float hoBase = 0.0;
            float3 hoP = float3(0, 0, 0);  // ... and where it stopped (its trace goes on from there)
            int mode = 0;                 // 0 straight, 1 combined field, 2 exact trace
            int kHole = -1;
            float3 kEntry = float3(0, 0, 0);
            bool kFromCam = false;
            bool kFinite = false;
            bool firstSeg = true;
            bool fromMedium = false;      // this straight segment starts at a group exit
            bool reachedInfinity = false;
            bool stopped = false;
            int kerrCount = 0;
            float jetSkip = 0.0;          // the next straight piece's jets start here (behind it: before the ray)
            float3 kP0 = float3(0, 0, 0), kD0 = float3(0, 0, 1);   // a separate hole's leg, unbent (for its jets)
            float kT0 = 0.0;

            int camMember = -1;
            [loop]
            for (int cm = 0; cm < count; cm++)
            {
                if (!IsMember(cm)) continue;
                float3 toC = _BHPositionsCamRelative[cm].xyz;
                float Rc = DomainRadius(cm);
                if (dot(toC, toC) < Rc * Rc) camMember = cm;
            }
            if (camMember >= 0)
            {
                firstSeg = false;
                mode = 1;
                [loop]
                for (int c = 0; c < count; c++)
                {
                    if (!IsMember(c)) continue;
                    float3 toC = _BHPositionsCamRelative[c].xyz;
                    float Rc = InnerRadius(c);
                    if (dot(toC, toC) < Rc * Rc)
                    {
                        mode = 2; kHole = c; kFromCam = true; kFinite = true;
                    }
                }
            }

            [loop]
            for (int ev = 0; ev < BH_MAX_EVENTS; ev++)
            {
                uint mask = allMask & ~done;

                [branch]
                if (mode == 0)
                {
                    // ---- next event: a group's sphere entry, or a separate hole's
                    //      closest approach (its exact trace covers the whole line) ------
                    int target = -1;
                    float tKey = 3.0e38;
                    // a hole stopped at a satellite's distance: first its satellites (the ones this line hits traced,
                    // the others' bends), then it is traced on from where it stopped
                    bool hoPend = hoHole >= 0 && ((noKick >> (uint)hoHole) & 1u) != 0u;
                    float3 cHo = hoPend ? _BHPositionsCamRelative[hoHole].xyz : float3(0, 0, 0);
                    float RHo = hoPend ? DomainRadius(hoHole) : 0.0;
                    [loop]
                    for (int i = 0; i < count; i++)
                    {
                        if ((((done | noKick) >> (uint)i) & 1u) != 0u) continue;
                        float3 toBHi = _BHPositionsCamRelative[i].xyz - p;
                        float Ri = DomainRadius(i);
                        if (hoPend && (IsMember(i) || length(_BHPositionsCamRelative[i].xyz - cHo) + Ri >= RHo)) continue;
                        float tcaI = dot(toBHi, d);
                        float dd = dot(toBHi, toBHi);
                        float m2i = dd - tcaI * tcaI;
                        if (m2i >= Ri * Ri) continue;
                        bool insideI = dd < Ri * Ri;
                        if (!insideI && tcaI <= 0.0) continue;
                        float key = IsMember(i)
                            ? max(tcaI - sqrt(Ri * Ri - m2i), 0.0)
                            : max(tcaI, 0.0);
                        if (key < tKey) { tKey = key; target = i; }
                    }
                    // a hole nested in the target's region (a light companion: C# keeps the region whole around it)
                    // and on this line goes first -- the target's exact trace would pass straight through it; the
                    // target is then traced from where the ray leaves it
                    [branch]
                    if (target >= 0)
                    {
                        float3 cT = _BHPositionsCamRelative[target].xyz;
                        float RT = DomainRadius(target);
                        int nest = -1;
                        float nKey = 3.0e38;
                        [loop]
                        for (int n2 = 0; n2 < count; n2++)
                        {
                            if (n2 == target || ((done >> (uint)n2) & 1u) != 0u || IsMember(n2)) continue;
                            float3 cN = _BHPositionsCamRelative[n2].xyz;
                            float RN = DomainRadius(n2);
                            if (length(cN - cT) + RN >= RT) continue;
                            float3 toN = cN - p;
                            float tcaN = dot(toN, d);
                            float ddN = dot(toN, toN);
                            if (ddN - tcaN * tcaN >= RN * RN) continue;
                            if (ddN >= RN * RN && tcaN <= 0.0) continue;
                            float kN = max(tcaN, 0.0);
                            // one the ray meets after the target's closest approach is met on the target's bent path:
                            // the target's trace stops at its distance and hands over (below) -- unless float can't
                            // place it in the target's frame
                            if (kN >= tKey && !IsMember(target) && RN >= 1e-4 * length(cN - cT)) continue;
                            if (kN < nKey) { nKey = kN; nest = n2; }
                        }
                        if (nest >= 0) { target = nest; tKey = nKey; }
                    }
                    bool tMember = (target >= 0) && IsMember(target);

                    float tEnd = 3.0e38;
                    if (target >= 0)
                        tEnd = tMember ? tKey : max(dot(_BHPositionsCamRelative[target].xyz - p, d), 0.0);

                    // nested holes the line passes outside their regions while the target's trace starts right here
                    // (it would ignore them): a straight piece to the nearest one's closest approach with its whole
                    // weak bend (as on to infinity: its pull is all near it) and the others' up to there, its jets
                    // on that bent line; then the target's trace from there
                    int pass = -1;
                    float tPass = 3.0e38;
                    int pHost = hoPend ? hoHole : target;
                    [branch]
                    if (hoPend ? (target < 0) : (target >= 0 && !tMember && tEnd <= 0.0))
                    {
                        float3 cT = _BHPositionsCamRelative[pHost].xyz;
                        float RT = DomainRadius(pHost);
                        [loop]
                        for (int n3 = 0; n3 < count; n3++)
                        {
                            if (n3 == pHost || ((done >> (uint)n3) & 1u) != 0u || IsMember(n3)) continue;
                            float3 cN = _BHPositionsCamRelative[n3].xyz;
                            if (length(cN - cT) + DomainRadius(n3) >= RT) continue;
                            float tN = max(dot(cN - p, d), 0.0);
                            if (tN < tPass) { tPass = tN; pass = n3; }
                        }
                    }
                    [branch]
                    if (pass >= 0)
                    {
                        float3 kP = float3(0, 0, 0), dP = float3(0, 0, 0);
                        uint gMaskP = 0u;
                        [loop]
                        for (int w2 = 0; w2 < count; w2++)
                        {
                            if ((((done | noKick) >> (uint)w2) & 1u) != 0u) continue;
                            if (IsMember(w2)) { gMaskP |= 1u << (uint)w2; continue; }
                            float3 toW = _BHPositionsCamRelative[w2].xyz - p;
                            float tW = (w2 == pass) ? 3.0e38 : tPass;
                            float3 kw, dw;
                            WeakKick(d, toW, _BHParams[w2].x, SpinJ(w2), tW, 0.0, kw, dw);
                            if (firstSeg)
                                kw += NearCameraTermTo(d, toW, _BHParams[w2].x, tW, w2);
                            kP += kw;
                            dP += dw;
                        }
                        [branch]
                        if (gMaskP != 0u)
                        {
                            float3 kg, dg;
                            GroupWeakKick(p, d, tPass, count, gMaskP, kg, dg);
                            kP += kg;
                            dP += dg;
                        }
                        float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0), on = float3(0, 0, 0), of = float3(0, 0, 0);
                        JetLine(p, d, jetSkip, tPass, count, 0u, mask & ~noKick & ~(1u << (uint)pass), -1, 0.0, jn, jf);
                        JetLine(p, d, jetSkip, 3.0e30, count, 1u << (uint)pass, 1u << (uint)pass, -1, 0.0, on, of);
                        cr.emAll += (jn + jf + on + of) * cr.T;
                        p = p + d * tPass + dP;
                        d = ApplyKick(d, kP);
                        done |= 1u << (uint)pass;
                        jetSkip = 0.0;
                        continue;
                    }
                    [branch]
                    if (hoPend && target < 0)
                    {
                        // its satellites done: on from its stop point -- on this (bent) line where it passes nearest it,
                        // or the stop point itself if the line got there already (a bend after the stop acts there)
                        float tC = dot(hoP - p, d);
                        p = (tC > 0.0) ? p + d * tC : hoP;
                        kP0 = p;
                        kD0 = d;
                        kT0 = 0.0;
                        jetSkip = 0.0;
                        mode = 2; kHole = hoHole; kFromCam = false; kFinite = false;
                        firstSeg = false;
                        continue;
                    }

                    bool touchMed = tMember || fromMedium;
                    float3 kick = float3(0, 0, 0);
                    float3 disp = float3(0, 0, 0);
                #if BH_GROUP_WEAK
                    uint gMask = 0u;
                    [loop]
                    for (int g = 0; g < count; g++)
                        if (((mask >> (uint)g) & 1u) != 0u && IsMember(g)) gMask |= 1u << (uint)g;
                    GroupWeakKick(p, d, tEnd, count, gMask, kick, disp);
                    [loop]
                    for (int w = 0; w < count; w++)
                    {
                        if ((((done | noKick) >> (uint)w) & 1u) != 0u || IsMember(w) || w == target) continue;
                        float3 kw, dw;
                        if (target < 0)
                        {
                            kw = CoupledWeakKick(w, p, d, count, mask, firstSeg, touchMed);
                            dw = float3(0, 0, 0);
                        }
                        else
                        {
                            WeakKick(d, _BHPositionsCamRelative[w].xyz - p, _BHParams[w].x, SpinJ(w), tEnd, 0.0, kw, dw);
                            // from the camera: its finite distance, as the straight view past it has it (the rays just
                            // inside its region, traced exactly from the camera, have it too: a ~4 px seam at its edge)
                            if (firstSeg)
                                kw += NearCameraTermTo(d, _BHPositionsCamRelative[w].xyz - p, _BHParams[w].x, tEnd, w);
                        }
                        kick += kw;
                        disp += dw;
                    }
                #else
                    [loop]
                    for (int w = 0; w < count; w++)
                    {
                        if (((done >> (uint)w) & 1u) != 0u || (w == target && !tMember)) continue;
                        float3 kw, dw;
                        if (target < 0)
                        {
                            kw = CoupledWeakKick(w, p, d, count, mask, firstSeg, touchMed);
                            dw = float3(0, 0, 0);
                        }
                        else
                        {
                            float hoR = (touchMed && IsMember(w)) ? DomainRadius(w) : 0.0;
                            WeakKick(d, _BHPositionsCamRelative[w].xyz - p, _BHParams[w].x, SpinJ(w), tEnd, hoR, kw, dw);
                        }
                        kick += kw;
                        disp += dw;
                    }
                #endif
                    fromMedium = false;
                    // jets on this straight piece (into a group: along the chord to where the combined-field
                    // leg starts). A separate hole's exact trace re-traces its piece: it does them there.
                    [branch]
                    if (target < 0 || tMember)
                    {
                        float3 jDir = d;
                        float jLen = 3.0e30;
                        if (tMember)
                        {
                            float3 chordM = d * tEnd + disp;
                            jLen = length(chordM);
                            jDir = chordM / max(jLen, 1e-30);
                        }
                        float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                        JetLine(p, jDir, jetSkip, jLen, count, tMember ? 0u : mask, mask, -1, 0.0, jn, jf);
                        cr.emAll += (jn + jf) * cr.T;
                        jetSkip = 0.0;
                    }
                    float3 newDir = ApplyKick(d, kick);
                    if (target < 0)
                    {
                        d = newDir;
                        reachedInfinity = true;
                        break;
                    }
                    float3 pEnd = p + d * tEnd + disp;
                    if (tMember)
                    {
                        // The kick displacement can leave the entry point just OUTSIDE the
                        // sphere: the combined-field leg would then end at once, and the
                        // ray would skip the group (wrong lensing, the cutoffs) or bounce
                        // between the modes until the event limit (black). Tested: worst
                        // case 133 px -> 3 px. Keep it just inside.
                        float3 cT = _BHPositionsCamRelative[target].xyz;
                        float RT = DomainRadius(target);
                        float3 rvT = pEnd - cT;
                        float rrT = length(rvT);
                        if (rrT >= RT)
                            pEnd = cT + rvT * (RT * (1.0 - 1e-4) / rrT);
                        p = pEnd;
                        d = newDir;
                        mode = 1;
                    }
                    else
                    {
                        kP0 = p;
                        kD0 = d;
                        kT0 = tEnd;
                        // the kicked line's point nearest the start: the trace starts where the camera is. (It was
                        // pEnd - newDir tEnd: ~alpha^2 / 2 * tEnd further along the line -- 0.1 Rs of a giant 20 Rs
                        // away after a small hole's 0.11 rad bend, a different observer: ~0.4 px seam at the small
                        // hole's region edge. Only the parts across the line: no subtraction of two far points.)
                        p = p + (d - newDir * dot(d, newDir)) * tEnd + (disp - newDir * dot(disp, newDir));
                        d = newDir;
                        mode = 2; kHole = target; kFromCam = firstSeg; kFinite = false;
                    }
                    firstSeg = false;
                }
                else if (mode == 1)
                {
                    cr.anyStrong = true;
                    cr.traces++;
                    cr.path |= 1;
                    g_dbgTraces += 1.0;
                    int hit;
                    int res = IntegrateMedium(p, d, count, mask, hit, cr, anyEm);
                    if (res == 2) { cr.captured = true; cr.why = BH_WHY_PHOTONSPH; break; }
                    if (res == 3) { cr.captured = true; cr.why = BH_WHY_STEPS; break; }
                    if (res == 4) { stopped = true; break; }
                    if (res == 0)
                    {
                        mode = 0;
                        fromMedium = true;
                    }
                    else
                    {
                        float3 c = _BHPositionsCamRelative[hit].xyz;
                        float3 rv = p - c;
                        if (dot(d, rv) >= 0.0)
                            p = c + rv * (1.0 + 1e-3);
                        else
                        {
                            mode = 2; kHole = hit; kEntry = p; kFromCam = false; kFinite = true;
                        }
                    }
                }
                else
                {
                    if (kerrCount >= BH_MAX_KERR) { cr.captured = true; cr.why = BH_WHY_KERR; break; }
                    int h = kHole;
                    float rs = _BHParams[h].x;
                    float3 bhPos = _BHPositionsCamRelative[h].xyz;
                    float3 toBH = bhPos - p;
                    float3 dT = d;
                    // Group traces (kFinite) live in isotropic coordinates with static-frame
                    // directions: convert to BL / ZAMO for the exact trace, also when the trace
                    // starts at the camera. Only the first straight segment to a separate hole
                    // (kFromCam && !kFinite) keeps the single-hole convention.
                    if (kFinite)
                    {
                        float rho = length(toBH) / rs;
                        toBH *= IsoToBL(rho) / rho;
                    }
                    if (kFinite)
                        dT = StaticToZamo(d, ZamoVelocity(h, p));
                    float rOuter = kFinite ? IsoToBL(_BHParams[h].y) : 1e30;
                    float escR = kFinite ? rOuter : _BHParams[h].w;

                    // A light satellite nested in this hole's region that the ray can pass after this trace starts
                    // (its leg reaches inside the satellite's distance from the hole): the trace stops where the leg
                    // comes back out to that distance (the exact state there), the satellite acts on the ray (traced
                    // if it is hit, else its bend), and this hole is traced on from there. Ignoring it, a satellite
                    // heavier than ~1e-4 of the mass had its Einstein ring cut where rays switched traces. Once per
                    // hole and ray; only satellites float can place in this hole's frame (region > 1e-4 distance).
                    bool tryHand = false;
                    [branch]
                    if (!kFinite && ((handed >> (uint)h) & 1u) == 0u)
                    {
                        float RH = DomainRadius(h);
                        float tcaH = dot(toBH, d);
                        float dd2 = dot(toBH, toBH);
                        float b2H = dd2 - tcaH * tcaH;
                        [loop]
                        for (int s3 = 0; s3 < count; s3++)
                        {
                            if (s3 == h || ((done >> (uint)s3) & 1u) != 0u || IsMember(s3)) continue;
                            float dSH = length(_BHPositionsCamRelative[s3].xyz - bhPos);
                            float RS = DomainRadius(s3);
                            if (dSH + RS >= RH || RS < 1e-4 * dSH) continue;
                            if (dd2 < dSH * dSH || (tcaH > 0.0 && b2H < dSH * dSH))
                            {
                                rOuter = min(rOuter, dSH / rs);
                                tryHand = true;
                            }
                        }
                        if (tryHand)
                            escR = rOuter;
                    }
                    g_orderBase = (h == hoHole) ? hoBase : 0.0;
                    noKick &= ~(1u << (uint)h);

                    // jets: a group's trace samples its hole's along the whole geodesic; a separate hole's
                    // trace inside min(JET_RGEO, its region), faded into the straight lines here: this one up to
                    // the closest approach (all holes' jets), the outgoing asymptote after it (next pieces)
                    float rGw = min(JET_RGEO, _BHParams[h].w) * rs;
                    float3 pBack = p;
                    float tPre = cr.T;
                    g_jetFootRs = g_jetPix * length(bhPos) / rs;
                    [branch]
                    if (!kFinite)
                    {
                        // the jets up to the closest approach, on the leg's path bent through every hole left
                        // (displaced points: exactly what the straight view outside the region draws, from the
                        // same start); its own jet faded in where its trace samples it
                        float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                        JetLine(kP0, kD0, jetSkip, kT0, count, mask, mask & ~noKick, h, rGw, jn, jf);
                        cr.emAll += (jn + jf) * cr.T;
                    }

                    float3 color, exitPos, exitDir; bool escaped; float trans;
                    bool hf; float4 fc; float fi; float3 ef;
                    g_dbgTraces += 1.0;
                    KerrTraceCore(
                        dT, toBH, rs,
                        _BHParams[h].y, _BHParams[h].z, escR,
                        _BHDiskNormals[h].xyz,
                        _BHSpins[h].x, _BHSpins[h].y,
                        _BHNoise, sampler_BHNoise,
                        _BHRotationOffsets[h].x, _BHRotationOffsets[h].yz,
                        rOuter, true, _BHSpins[h].w > 0.5,
                        color, exitPos, exitDir, _BHAppearanceProperties1[h], _BHAppearanceProperties2[h],
                        escaped, trans, hf, fc, fi, ef, h, kFinite ? rOuter : min(JET_RGEO, _BHParams[h].w));

                    cr.path |= kFinite ? 2 : 4;
                    if (cr.firstHole < 0)
                        cr.firstHole = h;
                    if (!anyEm && hf)
                    {
                        cr.hasFirst = true;
                        cr.firstHole = h;
                        cr.firstCross = fc;
                        cr.firstIdx = fi;
                        cr.emFirst = ef;
                    }
                    // a volumetric disk reports no first crossing: any absorbing trace counts
                    anyEm = anyEm || hf || (trans < 0.999);
                    kerrCount++;
                    cr.traces++;
                    cr.emAll += color * cr.T;
                    cr.T *= trans;
                    // a group's trace: the jets inside its sphere on the chord through it -- the other holes',
                    // and its own faded in where its trace fades out (rOuter / 2 .. rOuter)
                    [branch]
                    if (kFinite)
                    {
                        float3 pX = escaped ? bhPos + (exitPos - toBH) * (BLToIso(length(exitPos - toBH) / rs)
                                                                      / max(length(exitPos - toBH) / rs, 1e-6))
                                            : kEntry + d * max(dot(bhPos - kEntry, d), 0.0);
                        float3 ch = pX - kEntry;
                        float chl = length(ch);
                        [branch]
                        if (chl > 0.0)
                        {
                            float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                            JetLine(kEntry, ch / chl, 0.0, chl, count, 0u, allMask & ~done, h, InnerRadius(h), jn, jf);
                            cr.emAll += (jn + jf) * tPre;
                        }
                    }
                    cr.anyStrong = true;
                    if (!escaped) { cr.captured = true; cr.why = BH_WHY_HORIZON; break; }
                    if (cr.T < 0.01) { stopped = true; break; }

                    float3 fromHole = exitPos - toBH;
                    if (kFinite)
                    {
                        float rE = length(fromHole) / rs;
                        float3 pOut = bhPos + fromHole * (BLToIso(rE) / max(rE, 1e-6));
                        float3 dOut = ZamoToStatic(exitDir, ZamoVelocity(h, pOut));
                        float L = length(pOut - kEntry);
                        float3 f = ExternalGrad(h, count, mask);
                        f -= dot(f, dOut) * dOut;
                        pOut += (0.5 * L * L) * f;
                        dOut = ApplyKick(dOut, L * f);
                        float R = InnerRadius(h);
                        float3 rv = pOut - bhPos;
                        float rr = max(length(rv), 1e-6);
                        float3 rhat = rv / rr;
                        pOut = bhPos + rhat * max(rr, R * (1.0 + 1e-3));
                        float outward = dot(dOut, rhat);
                        if (outward < 1e-3)
                            dOut = normalize(dOut + (1e-3 - outward) * rhat);
                        p = pOut;
                        d = dOut;
                        mode = 1;
                    }
                    else if (tryHand)
                    {
                        // stopped at the satellite's distance: its own jet's straight part up to there, then on from
                        // there (the satellite next, then this hole again, from that point)
                        float3 pH = bhPos + fromHole;
                        float tH = max(dot(pH - kP0, kD0), kT0);
                        float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                        JetLine(kP0, kD0, max(kT0, jetSkip), tH, count, mask, 1u << (uint)h, h, rGw, jn, jf);
                        cr.emAll += (jn + jf) * cr.T;
                        // the satellite's encounter is on both sides of its distance: the next pieces start on this
                        // outgoing line back at its point nearest the hole (the satellite's bend from there, or its
                        // trace from there), their jets from the stop on, and no weak bend of this hole on them
                        float tB = min(dot(bhPos - pH, exitDir), 0.0);
                        if (dot(toBH, dT) <= 0.0)
                            tB = max(tB, dot(pBack - pH, exitDir));   // started outgoing (a camera inside): from there
                        p = pH + exitDir * tB;
                        hoP = pH;
                        d = exitDir;
                        handed |= 1u << (uint)h;
                        noKick |= 1u << (uint)h;
                        hoHole = h;
                        hoBase = g_orderOut;
                        mode = 0;
                        jetSkip = -tB;
                    }
                    else
                    {
                        // its own jet beyond the closest approach on the same path, behind the disk (later pieces
                        // leave it out: done); the others' from the asymptote on
                        float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                        JetLine(kP0, kD0, max(kT0, jetSkip), 3.0e30, count, mask, 1u << (uint)h, h, rGw, jn, jf);
                        cr.emAll += (jn + jf) * cr.T;
                        p = bhPos + fromHole;
                        d = exitDir;
                        done |= (1u << (uint)h);
                        mode = 0;
                        jetSkip = max(dot(pBack - p, d), 0.0);
                    }
                }
            }
            if (!reachedInfinity && !stopped && !cr.captured)
            {
                cr.captured = true;
                cr.why = BH_WHY_EVENTS;
            }
            cr.dir = d;
        }
    #endif

        float ChainClass(ChainResult cr)
        {
            return (cr.captured ? 1.0 : 0.0)
                 + (cr.hasFirst ? 2.0 : 0.0)
                 + 4.0 * min(cr.firstIdx, 7.0)
                 + 32.0 * (float) min(cr.traces, 15)
                 + 512.0 * (float) (cr.firstHole + 1);
        }

        // first visible emission, re-shaded at this pixel
        float3 ShadeFirstCrossing(int h, float4 fc, float idx, float lod, out float opacity)
        {
            HoleDisk hd = LoadHoleDisk(h);
            float aPhys = clamp(_BHSpins[h].x, -0.998, 0.998) * KERR_M;
            float diskOut = max(_BHParams[h].z, hd.rIsco * 1.05);
            float3 acc = float3(0, 0, 0);
            float trans = 1.0;
            float gDummy = 0.0;
            ShadeKerrDiskCrossing(fc.x, fc.y, fc.z, (int) idx, aPhys, _BHSpins[h].y, diskOut, hd,
                _BHNoise, sampler_BHNoise, _BHRotationOffsets[h].x, _BHRotationOffsets[h].yz,
                KERR_DISK_PLUNGING != 0, fc.w, _BHAppearanceProperties1[h], _BHAppearanceProperties2[h], lod,
                acc, trans, gDummy);
            opacity = 1.0 - trans;
            return acc;
        }

        float3 ComposeChain(ChainResult cr, float skyFootprint)
        {
            float3 c = cr.emAll;
            if (!cr.captured && cr.T >= 0.01)
                c += SampleSky(cr.dir, skyFootprint) * cr.T;
            return c;
        }

        float WrapPi(float x) { return x - TWO_PI * round(x * (1.0 / TWO_PI)); }


        bool Reconstruct(float2 uv, out float3 color)
        {
            color = 0;
            g_reconFail = 0;
            float2 p = uv * _BHLowSize.xy - 0.5;
            int2 i0 = (int2) floor(p);
            float2 f = p - (float2) i0;
            int2 mx = (int2) _BHLowSize.xy - 1;
            int2 c00 = clamp(i0, 0, mx);
            int2 c10 = clamp(i0 + int2(1, 0), 0, mx);
            int2 c01 = clamp(i0 + int2(0, 1), 0, mx);
            int2 c11 = clamp(i0 + int2(1, 1), 0, mx);

            float4 b00 = _BHLow1.Load(int3(c00, 0));
            float4 b10 = _BHLow1.Load(int3(c10, 0));
            float4 b01 = _BHLow1.Load(int3(c01, 0));
            float4 b11 = _BHLow1.Load(int3(c11, 0));

            if (b00.w != b10.w || b00.w != b01.w || b00.w != b11.w) { g_reconFail = 1; return false; }
            float cls = b00.w;
            if (cls < 32.0) { g_reconFail = 2; return false; }
            bool captured = (fmod(cls, 2.0) >= 1.0);
            bool hasFirst = (fmod(floor(cls / 2.0), 2.0) >= 1.0);
            float firstIdx = fmod(floor(cls / 4.0), 8.0);
            int firstHole = (int) floor(cls / 512.0) - 1;

            float4 a00 = _BHLow0.Load(int3(c00, 0));
            float4 a10 = _BHLow0.Load(int3(c10, 0));
            float4 a01 = _BHLow0.Load(int3(c01, 0));
            float4 a11 = _BHLow0.Load(int3(c11, 0));
            float tMin = min(min(a00.a, a10.a), min(a01.a, a11.a));
            float tMax = max(max(a00.a, a10.a), max(a01.a, a11.a));
            if (tMax - tMin > BH_UPS_MAX_DT) { g_reconFail = 3; return false; }

            // Sky hidden behind opaque emission in all four texels: its direction never
            // reaches the screen, so it must not decide whether this cell is rebuilt.
            bool skyVisible = !captured && tMax >= 0.01;

            float3 dir = 0;
            float skyFoot = 0.0;
            if (skyVisible)
            {
                float texelAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _BHLowSize.y);
                float cosMax = cos(BH_UPS_MAX_SPREAD * texelAngle);
                float dMin = min(min(dot(b00.xyz, b10.xyz), dot(b00.xyz, b01.xyz)),
                                 min(dot(b11.xyz, b10.xyz), dot(b11.xyz, b01.xyz)));
                if (dMin < cosMax) { g_reconFail = 4; return false; }

                float pixAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                float l1 = length(b10.xyz - b00.xyz);
                float l2 = length(b01.xyz - b00.xyz);
                float perPixMin = min(l1, l2) / _BHDownsample;
                float perPixMax = max(l1, l2) / _BHDownsample;
                float3 twist = (b00.xyz + b11.xyz) - (b10.xyz + b01.xyz);
                float twistMax = BH_UPS_MAX_TWIST * perPixMin;
                if (dot(twist, twist) > twistMax * twistMax) { g_reconFail = 5; return false; }

                dir = normalize(lerp(lerp(b00.xyz, b10.xyz, f.x), lerp(b01.xyz, b11.xyz, f.x), f.y));
                skyFoot = max(perPixMax, pixAngle);
            }

            float3 emFirst = 0;
            if (hasFirst)
            {
                float4 d00 = _BHLow2.Load(int3(c00, 0));
                float4 d10 = _BHLow2.Load(int3(c10, 0));
                float4 d01 = _BHLow2.Load(int3(c01, 0));
                float4 d11 = _BHLow2.Load(int3(c11, 0));
                float rMin = min(min(d00.x, d10.x), min(d01.x, d11.x));
                float rMax = max(max(d00.x, d10.x), max(d01.x, d11.x));
                if (rMax - rMin > BH_UPS_MAX_DR * rMin) { g_reconFail = 6; return false; }
                d10.y = d00.y + WrapPi(d10.y - d00.y);
                d01.y = d00.y + WrapPi(d01.y - d00.y);
                d11.y = d00.y + WrapPi(d11.y - d00.y);
                float phMin = min(min(d00.y, d10.y), min(d01.y, d11.y));
                float phMax = max(max(d00.y, d10.y), max(d01.y, d11.y));
                if (phMax - phMin > BH_UPS_MAX_DPHI) { g_reconFail = 7; return false; }
                float4 fc = lerp(lerp(d00, d10, f.x), lerp(d01, d11, f.x), f.y);

                float2 gR = float2(0.5 * ((d10.x - d00.x) + (d11.x - d01.x)),
                                   0.5 * ((d01.x - d00.x) + (d11.x - d10.x)));
                float2 gP = float2(0.5 * ((d10.y - d00.y) + (d11.y - d01.y)),
                                   0.5 * ((d01.y - d00.y) + (d11.y - d10.y)));
                float su = (4.0 / TWO_PI) * _BHNoiseSize.x / _BHDownsample;
                float sv = 0.2 * _BHNoiseSize.y / _BHDownsample;
                float2 tx = float2(gP.x * su, gR.x * sv);
                float2 ty = float2(gP.y * su, gR.y * sv);
                float lod = max(0.5 * log2(max(max(dot(tx, tx), dot(ty, ty)), 1e-12)), 0.0);

                float op;
            #if !defined(BH_VOLUMETRIC_DISK)
                // (volumes are not a point: they report no first crossing, all their emission is in a.rgb)
                emFirst = ShadeFirstCrossing(firstHole, fc, firstIdx, lod, op);
            #endif
            }

            float4 a = lerp(lerp(a00, a10, f.x), lerp(a01, a11, f.x), f.y);
            color = emFirst + a.rgb;
            if (skyVisible && a.a >= 0.01)
                color += SampleSky(dir, skyFoot) * a.a;
            return true;
        }
        ENDHLSL

        Tags { "RenderType"="Opaque" }
        LOD 100
        ZWrite Off ZTest Always Cull Off

        // ------------------------------------------------------------------
        // Pass 0: light composite (early-outs, weak-only, reconstruction); pixels that
        // need an exact trace are discarded -> stencil 0 -> pass 2
        // ------------------------------------------------------------------
        Pass
        {
            Name "BlackHoleLensingComposite"
            Stencil
            {
                Ref 1
                Comp Always
                Pass Replace
            }
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment FragComposite
            #pragma use_dxc
            #pragma multi_compile_fragment _ BH_VOLUMETRIC_DISK

            float4 FragComposite (Varyings input) : SV_Target
            {
                g_jetPix = 2.0 * max(_BHDownsample, 1.0) / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                float3 sceneColor = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_LinearClamp, input.texcoord).rgb;
                if (_BHCount == 0) return float4(sceneColor, 1);

                float3 dir = PixelRayDir(input.texcoord);
                int count = min(_BHCount, 8);
                int camHole = CameraHole(count);
                #if BH_DEBUG_CAMREGION
                // corner square: blue = camera outside every region, green = inside a
                // region (combined field starts at the camera), red = inside an exact sphere
                if (input.texcoord.x < 0.04 && input.texcoord.y < 0.04)
                {
                    bool inRegion = false, inExact = false;
                    [loop]
                    for (int c = 0; c < count; c++)
                    {
                        float3 toC = _BHPositionsCamRelative[c].xyz;
                        float d2 = dot(toC, toC);
                        float Rr = DomainRadius(c);
                        float Re = _BHParams[c].x * _BHParams[c].y;
                        inRegion = inRegion || (d2 < Rr * Rr);
                        inExact = inExact || (d2 < Re * Re);
                    }
                    float3 tint = inExact ? float3(1, 0.3, 0.2) : (inRegion ? float3(0.2, 1, 0.3) : float3(0.2, 0.4, 1));
                    return float4(tint, 1);
                }
            #endif

                float3 camFwd0 = -UNITY_MATRIX_V[2].xyz;
                float sceneEye = SceneEyeDepth(input.texcoord);
                bool isSkyPixel = sceneEye > 1e30;
                float pixAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                float bendTol = 0.25 * pixAngle;
                bool visibleBend = false;
                [loop]
                for (int j = 0; j < count; j++)
                {
                    float3 toBHc = _BHPositionsCamRelative[j].xyz;
                    float tcaC = dot(toBHc, dir);
                    float m2c = max(dot(toBHc, toBHc) - tcaC * tcaC, 0.0);
                    if (2.0 * _BHParams[j].x * rsqrt(max(m2c, 1e-12)) > bendTol)
                        visibleBend = true;
                }
                float tFirst = FirstDomainEntry(dir, count, camHole);

                if (!isSkyPixel && sceneEye / max(dot(dir, camFwd0), 1e-4) < tFirst)
                {
                #if BH_DEBUG_COST
                    return float4(0, 0, 0, 1);
                #endif
                    return float4(sceneColor, 1);
                }

                // sky with no visible bending (the scene as it is), or weak field only
                bool skyOnly = isSkyPixel && !visibleBend;
                if (skyOnly || tFirst > 1e30)
                {
                #if BH_DEBUG_COST
                    return skyOnly ? float4(0, 0, 0, 1) : float4(0, 0, 0.6, 1);
                #endif
                    float3 bg = sceneColor;
                    [branch]
                    if (!skyOnly)
                        bg = SampleSky(WeakOnlyDir(dir, count), pixAngle);
                    // jets on the straight line (in front of the scene up to its depth)
                    float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                    JetLine(float3(0, 0, 0), dir, 0.0, isSkyPixel ? 3.0e30 : sceneEye / max(dot(dir, camFwd0), 1e-4),
                            count, (1u << (uint)count) - 1u, (1u << (uint)count) - 1u, -1, 0.0, jn, jf);
                    bg += jn + jf;
                    float3 outC = ObserverShift(bg);
                #if BH_DEBUG_PATH
                    outC = DebugPathTint(0, outC);
                #endif
                    return float4(outC, 1);
                }

                if (_BHDownsample > 1.5)
                {
                    float3 rec;
                    if (Reconstruct(input.texcoord, rec))
                    {
                    #if BH_DEBUG_COST
                        return float4(0, 1, 0, 1);
                    #endif
                        return float4(ObserverShift(rec), 1);
                    }
                #if BH_DEBUG_RECON
                    float3 rc = (g_reconFail == 1) ? float3(1, 0, 0)
                              : (g_reconFail == 2) ? float3(0.4, 0.4, 0.4)
                              : (g_reconFail == 3) ? float3(1, 1, 0)
                              : (g_reconFail == 4) ? float3(1, 0, 1)
                              : (g_reconFail == 5) ? float3(0, 1, 1)
                              : (g_reconFail == 6) ? float3(0, 0, 1)
                              :                      float3(1, 1, 1);
                    return float4(rc, 1);
                #endif
                }

                discard;
                return float4(0, 0, 0, 1);
            }
            ENDHLSL
        }

        // ------------------------------------------------------------------
        // Pass 1: reduced-resolution trace (renders into the three _BHLow targets)
        // ------------------------------------------------------------------
        Pass
        {
            Name "BlackHoleLensingTraceLowRes"
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment FragLow
            #pragma use_dxc
            #pragma multi_compile_fragment _ BH_SINGLE
            #pragma multi_compile_fragment _ BH_VOLUMETRIC_DISK

            struct LowOut
            {
                float4 t0 : SV_Target0;
                float4 t1 : SV_Target1;
                float4 t2 : SV_Target2;
            };

            LowOut FragLow (Varyings input)
            {
                g_jetPix = 2.0 * _BHDownsample / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                LowOut o;
                o.t0 = float4(0, 0, 0, 1);
                o.t1 = float4(0, 0, 1, 0);
                o.t2 = float4(0, 0, 0, 0);
                if (_BHCount == 0) return o;

                float3 dir = PixelRayDir(input.texcoord);
                int count = min(_BHCount, 8);
                int camHole = CameraHole(count);
                bool weakOnly = FirstDomainEntry(dir, count, camHole) > 1e30;
                if (weakOnly)
                {
                #if defined(BH_SINGLE)
                    // Outside the region: write the weak-field direction with the class of an escaping
                    // single-hole trace that crossed no disk (traces = 1 -> 32, firstHole = 0 -> 512).
                    // Cells straddling the region edge then match and reconstruct, instead of all
                    // failing the class test and going to the full-res trace.
                    o.t1 = float4(WeakOnlyDir(dir, count), 544.0);
                #endif
                    return o;
                }

                g_bhLowRes = true;
                g_bhDiskLod = log2(max(_BHDownsample, 1.0));
                ChainResult cr;
                TraceChain(dir, count, camHole, cr);
                o.t0 = float4(cr.emAll - cr.emFirst, cr.T);
                o.t1 = float4(cr.dir, ChainClass(cr));
                o.t2 = cr.firstCross;
                return o;
            }
            ENDHLSL
        }

        // ------------------------------------------------------------------
        // Pass 2: full-resolution exact trace, only where pass 0 discarded
        // ------------------------------------------------------------------
        Pass
        {
            Name "BlackHoleLensingFullTrace"
            Stencil
            {
                Ref 0
                Comp Equal
                Pass Keep
            }
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment FragTrace
            #pragma use_dxc
            #pragma multi_compile_fragment _ BH_SINGLE
            #pragma multi_compile_fragment _ BH_VOLUMETRIC_DISK

            float4 FragTrace (Varyings input) : SV_Target
            {
                g_jetPix = 2.0 * max(_BHDownsample, 1.0) / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                // the disk's turbulence at the same detail as the reduced-res trace these pixels sit among (they
                // are the borders of what is rebuilt from it): sharper noise has more variance, and the gas's
                // lognormal density and transmittance average brighter with it -- the borders glowed
                g_bhDiskLod = log2(max(_BHDownsample, 1.0));
                float3 dir = PixelRayDir(input.texcoord);
                int count = min(_BHCount, 8);
                int camHole = CameraHole(count);

                ChainResult cr;
                TraceChain(dir, count, camHole, cr);
                // how much sky this pixel sees: screen-space derivatives of the final
                // direction (taken at top level, after the trace, so they are valid)
                float3 dDx = ddx(cr.dir);
                float3 dDy = ddy(cr.dir);
                float skyFoot = max(length(dDx), length(dDy));
            #if BH_DEBUG_COST
                return float4(saturate(g_dbgTraces), 0.0, 0.0, 1);
            #endif
            #if BH_DEBUG_CAPTURE
                if (cr.captured)
                {
                    float3 dc = (cr.why == BH_WHY_HORIZON)   ? float3(0.06, 0.06, 0.06)
                              : (cr.why == BH_WHY_PHOTONSPH) ? float3(0, 0, 1)
                              : (cr.why == BH_WHY_STEPS)     ? float3(1, 0, 0)
                              : (cr.why == BH_WHY_KERR)      ? float3(1, 1, 0)
                              :                                float3(1, 0, 1);
                    return float4(dc, 1);
                }
            #endif
                float3 outC = ObserverShift(ComposeChain(cr, skyFoot));
            #if BH_DEBUG_PATH
                outC = DebugPathTint(cr.path, outC);
            #endif
                return float4(outC, 1);
            }
            ENDHLSL
        }
    }
}