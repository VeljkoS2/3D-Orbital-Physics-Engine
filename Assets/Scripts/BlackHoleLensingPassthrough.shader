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
        // Inline sampler states (Unity reads the filter / wrap from the name): both textures are sampled at a
        // computed, fractional mip (SkyLod, the disk's turbulence LOD). Their own samplers follow the import
        // settings -- Bilinear there means NO blending between mips: the level jumped where the footprint crossed
        // half a level, rings of sharper / blurrier sky around the hole and contour lines across the disk.
        SAMPLER(sampler_trilinear_clamp);
        SAMPLER(sampler_trilinear_repeat);

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
        // g(t1) - g(0), g = (t - tca) / h (s = t - tca, h = sqrt(s^2 + b^2)), without subtracting two values ~1: on a
        // line pointing at (or away from) a hole far off it was float noise / b -- a straight piece toward a big hole
        // 1e9 units away got a 0.1 rad bend, one wrong pixel (a block after the rebuild) where the view faced it
        float WeakDG(float s0, float s1, float h0, float h1, float b2, float t1, bool toInf)
        {
            if (toInf) return (s0 > 0.0) ? b2 / ((h0 + s0) * h0) : 1.0 - s0 / h0;
            return (s0 * s1 > 0.0) ? b2 * t1 * (s1 + s0) / ((s1 * h0 + s0 * h1) * h0 * h1) : s1 / h1 - s0 / h0;
        }
        // 2 - |A|, A = u (2u^2 + 3) / (1 + u^2)^1.5 (the spin integral's), from w = b^2 / h^2: ~0.75 w^2 far out
        float SpinE(float w) { return w * w * (3.0 + w) / (2.0 + sqrt(max(1.0 - w, 0.0)) * (2.0 + w)); }

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
            float b2 = b * b;
            float s0 = -tca;
            float h0 = sqrt(tca * tca + b2);
            float s1 = t1 - tca;
            float h1 = toInf ? 1.0 : sqrt(s1 * s1 + b2);
            float dg = WeakDG(s0, s1, h0, h1, b2, t1, toInf);
            kick = C * dg * toward;
            disp = float3(0, 0, 0);
            if (!toInf)
                disp = C * WeakDrift(t1, tca, b, h0, s1, h1) * toward;

            // spin: A, Q = (b / h)^3, C (= g) at both ends; A as sign (2 - E), differences of same-signed ends from E
            float w0 = b2 / (h0 * h0);
            float E0 = SpinE(w0);
            float iq03 = w0 * sqrt(w0);
            float sg0 = (s0 >= 0.0) ? 1.0 : -1.0;
            float dA, Q1 = 0.0;
            if (toInf)
                dA = (sg0 > 0.0) ? E0 : 4.0 - E0;
            else
            {
                float w1 = b2 / (h1 * h1);
                float E1 = SpinE(w1);
                Q1 = w1 * sqrt(w1);
                float sg1 = (s1 >= 0.0) ? 1.0 : -1.0;
                dA = (sg0 == sg1) ? sg0 * (E0 - E1) : sg1 * (2.0 - E1) - sg0 * (2.0 - E0);
            }
            float3 I = (dot(J, bh) * dA - dot(J, d) * (Q1 - iq03)) * bh - dg * J;
            // second-order spin-mass coupling (10 pi a M^2 / b^3): tested vs exact Kerr (y: the distance to this
            // segment -- the closest approach's b where it is on it; x = Rs / b blew up on lines aimed at the hole)
            kick += (2.0 / b2) * (1.0 + (5.0 * PI / 4.0) * min(x, rs / rSeg)) * cross(d, I);
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
            float b2 = b * b;
            float s1 = t1 - tca;
            bool toInf = (t1 > 1e30);
            float dg = WeakDG(-tca, s1, sqrt(tca * tca + b2), toInf ? 1.0 : sqrt(s1 * s1 + b2), b2, t1, toInf);
            float sc = (_BHJet[h].y > 0.0) ? _BHJet[h].y : 1.0;
            return (sc * rs * rs / (b * r0)) * 0.5 * dg * (pOff / b);
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

        // the displacement at t1 of the leading-order mass bend (WeakKick's, alone)
        float3 WeakDisp(float3 d, float3 toBH, float rs, float t1)
        {
            float tca = dot(toBH, d);
            float3 pOff = tca * d - toBH;
            float b = max(length(pOff), 1e-6);
            float rSeg = max(length(toBH - d * clamp(tca, 0.0, t1)), b);
            float x = rs / b;
            float y = rs / rSeg;
            float C = x * (1.0 + y * ((15.0 * PI / 32.0) + y * ((8.0 / 3.0) + y * (3465.0 * PI / 2048.0))));
            float h0 = sqrt(tca * tca + b * b);
            float s1 = t1 - tca;
            float h1 = sqrt(s1 * s1 + b * b);
            return -C * WeakDrift(t1, tca, b, h0, s1, h1) * (pOff / b);
        }

        // weak bend of the group members in gMask on the segment p + t d, t in [0, t1]; for those in gInf (whole
        // groups) the bend runs on to infinity -- the displacement is still the one at t1
        void GroupWeakKickInf(float3 p, float3 d, float t1, int count, uint gMask, uint gInf, out float3 kick, out float3 disp)
        {
            kick = float3(0, 0, 0);
            disp = float3(0, 0, 0);
            [loop]
            for (int i = 0; i < count; i++)
            {
                if (((gMask >> (uint)i) & 1u) == 0u) continue;
                float rsI = _BHParams[i].x;
                float3 Pi = _BHPositionsCamRelative[i].xyz;
                bool inf = ((gInf >> (uint)i) & 1u) != 0u;
                float t1i = inf ? 3.0e38 : t1;
                float3 kw, dw;
                WeakKick(d, Pi - p, rsI, SpinJ(i), t1i, 3.0e38, kw, dw);   // 1st order + spin only
                kick += kw;
                disp += inf ? WeakDisp(d, Pi - p, rsI, t1) : dw;
                SecondOrderKick(p, d, t1i, Pi, rsI * rsI, kick, disp);
                float rsT = rsI;
                float3 cSum = rsI * Pi;
                bool firstOfGroup = true;
                [loop]
                for (int j = 0; j < count; j++)
                {
                    if (j == i || ((gMask >> (uint)j) & 1u) == 0u || _BHSpins[j].z != _BHSpins[i].z) continue;
                    float rsJ = _BHParams[j].x;
                    float3 Pj = _BHPositionsCamRelative[j].xyz;
                    if (j > i) SecondOrderKick(p, d, t1i, 0.5 * (Pi + Pj), 2.0 * rsI * rsJ, kick, disp);
                    else firstOfGroup = false;
                    rsT += rsJ;
                    cSum += rsJ * Pj;
                }
                if (firstOfGroup)
                    GroupHighOrder(p, d, t1i, cSum / rsT, rsT, kick);
            }
        }
        void GroupWeakKick(float3 p, float3 d, float t1, int count, uint gMask, out float3 kick, out float3 disp)
        {
            GroupWeakKickInf(p, d, t1, count, gMask, 0u, kick, disp);
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
        // ...tested only where the sky shows through at least this much (T, the most of the cell's four): behind gas
        // that lets less through, the sky's share is small, and a cell whose directions spread (the sky lensed into a
        // ring close to a hole) is rebuilt with its sky blurred over the spread instead of traced pixel by pixel. With
        // the camera inside the gas a few Rs from a hole, that ring was ~3/4 of the full-res traces (9 % of the screen,
        // the sky behind it at T ~0.1): now ~1/4 as many, the image within 2/255 but along the ring's lensed stars.
        #define BH_UPS_SKY_T      0.3
        // max interpolation error, in units of 1/4 screen pixel: 2 = the sky at most half a pixel off where the cell's
        // directions are farthest from bilinear (1 sent twice the cells to the full-res trace -- up to half of pass 2
        // -- for differences < 2/255 after tonemapping nearly everywhere; none along an edge)
        #define BH_UPS_MAX_TWIST  2.0
    #if defined(BH_VOLUMETRIC_DISK)
        #define BH_UPS_MAX_DT     0.8   // soft volumetric gas interpolates fine
    #else
        #define BH_UPS_MAX_DT     0.4
    #endif
        #define BH_UPS_MAX_DPHI   1.0
        #ifndef BH_UPS_MAX_DEM
        #define BH_UPS_MAX_DEM    0.25   // max emission spread in a cell, on the scale L / (L + knee)
        #endif
        #ifndef BH_UPS_EM_KNEE
        #define BH_UPS_EM_KNEE    0.25   // HDR luminance that counts as mid-grey for that test
        #endif
        #define BH_LUMA float3(0.2126, 0.7152, 0.0722)
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
            return SAMPLE_TEXTURECUBE_LOD(_BHSkybox, sampler_trilinear_clamp, dir, SkyLod(footprint)).rgb;
        }

        Texture2D<float4> _BHLow0;
        Texture2D<float4> _BHLow1;
        Texture2D<float4> _BHLow2;
        float4 _BHLowSize;
        float _BHDownsample;
        // The reduced-res traces sit on the CORNERS of downsample x downsample pixel blocks (texel i at full-res
        // pixel edge i * ds): x = ds / full width, y = ds / full height, z = full width / ds, w = full height / ds
        // (C#). Every reconstruction cell is then one block -- whole 2x2 quads at ds = 2 and 4. With the traces
        // at the blocks' centres the cells straddled the quads at ds = 2, and every full-res trace of a failed
        // cell came with a helper lane tracing the same (the trace ends in ddx / ddy): ~1.8x the pass-2 work.
        float4 _BHTraceGrid;
        float2 LowTexelUV(float2 texcoord) { return (texcoord * _BHLowSize.xy - 0.5) * _BHTraceGrid.xy; }
        // Weak-field texels of the reduced-res trace carry this in their class's fraction (below 0.25: no first
        // crossing for Reconstruct), so the composite can rebuild the jets on weak-field pixels from them
        #define BH_WEAK_TAG 0.125

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
            float d = 2.0;   // the whole crossing in the piece (|x| > 4.5 on both sides: erf saturated to 1e-10)
            [branch]
            if (x1 > -4.5 || x2 < 4.5)
                d = (abs(x2 - x1) < 1e-2) ? 1.1283792 * exp(-0.25 * (x1 + x2) * (x1 + x2)) * (x2 - x1)
                                          : JetErf(x2) - JetErf(x1);
            return exp(-0.5 * b2 / (s * s)) * (0.88622693 * d / k);
        }

        // fade of a straight piece's jet near its hole (Rs): fR > 0 faded in over fR / 2 .. fR (inside, the exact
        // trace samples it along the geodesic); fR < 0 the complement (only inside: for a ray the trace sampled none)
        float JetFade(float fR, float dist)
        {
            float k = smoothstep(0.5 * abs(fR), abs(fR), dist);
            return (fR > 0.0) ? k : 1.0 - k;
        }

        // the lobe sgn (+1 / -1 along the axis) of hole h's jet on o + t d, t in [t0, t1]; fadeR > 0: faded in
        // over fadeR / 2 .. fadeR from the hole (inside, its exact trace sampled it along the geodesic); < 0: the
        // complement (JetFade)
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
            float knotAmt = smoothstep(0.15, 0.35, sinP) * _BHJet[h].w;
            float knotClock = _BHJet[h].z;
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
                        if (fR != 0.0)
                            fadeK = JetFade(fR, sqrt(b2 + zm * zm));
                        acc += JetLine1(zm, zMin, zMax, sgn, knotAmt, knotClock) * g * (0.159155 * fadeK * wc);
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
                            if (fR != 0.0)
                                fadeK = JetFade(fR, sqrt(rho2 + zq2 * zq2));
                            acc += JetLine1(zq2, zMin, zMax, sgn, knotAmt, knotClock) * JetProfile(rho2, JetWidthRs(zq2), fp * fp)
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

        // Does any jet in jetMask come within reach of the line (JetLine's own cull, without integrating anything)?
        bool JetNearAny(float3 o, float3 d, float t0, float t1, int count, uint kinkMask, uint jetMask)
        {
        #if !defined(BH_JETS)
            return false;
        #else
            float3 kA = float3(0, 0, 0), dA = float3(0, 0, 0);
            [loop]
            for (int j = 0; j < count; j++)
                if (((kinkMask >> (uint)j) & 1u) != 0u)
                    JetBend(d, _BHPositionsCamRelative[j].xyz - o, _BHParams[j].x, 3.0e38, kA, dA);
            float bendA = (kinkMask != 0u) ? length(ApplyKick(d, kA) - d) : 0.0;
            bool nearAny = false;
            [loop]
            for (int h = 0; h < count; h++)
                nearAny = nearAny || (_BHJet[h].x > 0.0 && ((jetMask >> (uint)h) & 1u) != 0u && JetNear(h, o, d, t0, t1, bendA));
            return nearAny;
        #endif
        }

        // Does the ray from the camera along d pass the jet axis nearest to it before distance tMax? (Of the holes
        // with jets: the one whose axis comes closest to the ray, and where along the ray that is.)
        bool JetCrossingBefore(float3 d, float tMax, int count)
        {
        #if !defined(BH_JETS)
            return false;
        #else
            float best = 3.0e38, tBest = 3.0e38;
            [loop]
            for (int h = 0; h < count; h++)
            {
                if (_BHJet[h].x <= 0.0)
                    continue;
                float3 c = _BHPositionsCamRelative[h].xyz;
                float3 n = _BHDiskNormals[h].xyz;
                float b = dot(d, n);
                float den = max(1.0 - b * b, 1e-6);
                float t = (dot(d, c) - b * dot(n, c)) / den;   // the ray's point nearest the axis line
                float s = (b * dot(d, c) - dot(n, c)) / den;   // ...and the axis's
                float dist2 = dot(d * t - c - n * s, d * t - c - n * s) / (_BHParams[h].x * _BHParams[h].x);
                if (dist2 < best)
                {
                    best = dist2;
                    tBest = t;
                }
            }
            return tBest < tMax;
        #endif
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
        void JetLineSplit(float3 o, float3 d, float t0, float t1, int count, uint kinkMask, uint jetMask, int fadeHole,
                          float fadeR, float tSplit, float wAfter, inout float3 nearEm, inout float3 farEm)
        {
        #if defined(BH_JETS) // (off while no hole has jets: C# turns the keyword off, and all the jet code compiles out)
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
                    // (before / after tSplit: weighted 1 / wAfter -- a piece across it in two parts)
                    [loop]
                    for (int q = 0; q < 2; q++)
                    {
                        float a0 = (q == 0) ? max(t0, sA) : max(max(t0, sA), tSplit);
                        float a1 = (q == 0) ? min(min(t1, sB), tSplit) : min(t1, sB);
                        float wq = (q == 0) ? 1.0 : wAfter;
                        [branch]
                        if (a1 > a0 && wq > 0.0)
                        {
                            float l0 = (a0 - sA) * sc, l1 = (a1 >= 1e37) ? 3.0e30 : (a1 - sA) * sc;
                            // both lobes through one JetSeg call site (it is large, and every call is inlined)
                            [loop]
                            for (int lobe = 0; lobe < 2; lobe++)
                            {
                                float3 e = JetSeg(h, pA, pd, l0, l1, (lobe == 0) ? camSide : -camSide, fr) * wq;
                                if (lobe == 0) nearEm += e; else farEm += e;
                            }
                        }
                    }
                    if (sB >= t1 || sB >= 1e37)
                        break;
                    sA = sB;
                    pA = pB;
                }
            }
        #endif
        }
        void JetLine(float3 o, float3 d, float t0, float t1, int count, uint kinkMask, uint jetMask, int fadeHole,
                     float fadeR, inout float3 nearEm, inout float3 farEm)
        {
            JetLineSplit(o, d, t0, t1, count, kinkMask, jetMask, fadeHole, fadeR, 3.0e38, 1.0, nearEm, farEm);
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

            // jets on straight lines (one call site: JetLine is large, every call is inlined): outside the region,
            // the whole bent line; inside, outside where the trace sampled them (min(JET_RGEO, its region), faded
            // into each other): the camera's line up to the closest approach (in front of the disk), and the
            // outgoing asymptote from there (behind it, times the disk's transmittance)
            float jT1 = 3.0e30;
            int jFade = -1;
            float jFadeR = 0.0;
            int nJet = 1;
            [branch]
            if (!enters)
            {
                float3 kick, disp;
                WeakKick(dir, toBH, rs, SpinJ(0), 3.0e38, 0.0, kick, disp);
                kick += NearCameraTerm(dir, toBH, rs, 0);
                cr.dir = ApplyKick(dir, kick);
            }
            else
            {
                float3 color, exitPos, exitDir; bool escaped; float trans;
                bool hf; float4 fc; float fi; float3 ef;
                g_dbgTraces += 1.0;
                g_jetFootRs = g_jetPix * length(toBH) / rs;
                // aimed at the centre: its jets on the straight line, not along the geodesic (JetOnLine)
                bool onLine = (tca > 0.0) && JetOnLine(sqrt(max(m2, 0.0)), rs, _BHSpins[0].x);
                KerrTraceCore(
                    dir, toBH, rs,
                    _BHParams[0].y, _BHParams[0].z, _BHParams[0].w,
                    _BHDiskNormals[0].xyz,
                    _BHSpins[0].x, _BHSpins[0].y,
                    _BHNoise, sampler_trilinear_repeat,
                    _BHRotationOffsets[0].x, _BHRotationOffsets[0].yz,
                    1e30, true, _BHSpins[0].w > 0.5,
                    color, exitPos, exitDir, _BHAppearanceProperties1[0], _BHAppearanceProperties2[0],
                    escaped, trans, hf, fc, fi, ef, 0, onLine ? 0.0 : min(JET_RGEO, _BHParams[0].w));

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
                // An opaque disk (trans < 0.01: the trace stopped there) is NOT a capture: nothing behind it shows
                // either way, but as "captured" every opacity contour of the gas split the reconstruction classes
                // and went to the full-res trace (most of pass 2 with the volumetric disk). Only the horizon is.
                cr.captured = !escaped && trans >= 0.01;
                if (cr.captured)
                    cr.why = BH_WHY_HORIZON;
                else if (escaped)
                    cr.dir = exitDir;

                jT1 = max(tca, 0.0);
                jFade = 0;
                jFadeR = min(JET_RGEO, _BHParams[0].w) * rs;
                nJet = (_BHJet[0].x > 0.0) ? (escaped ? 2 : 1) : 0;
                [branch]
                if (onLine)
                {
                    // aimed at the centre (its trace sampled no jets): the straight line, unfaded, up to the
                    // horizon if it falls in
                    float rH = (0.5 + sqrt(max(0.25 - 0.25 * _BHSpins[0].x * _BHSpins[0].x, 0.0))) * rs;
                    jT1 = escaped ? 3.0e30 : max(tca - sqrt(max(rH * rH - m2, 0.0)), 0.0);
                    jFade = -1;
                    jFadeR = 0.0;
                    nJet = (_BHJet[0].x > 0.0) ? 1 : 0;
                }
            }
            [branch]
            if (nJet > 0)
            {
                float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                // (the same path the straight view outside its region takes: through its displaced points; one
                // pass over its pieces for both sides of jT1, the far one behind the disk: times its transmittance)
                JetLineSplit(float3(0, 0, 0), dir, 0.0, (nJet > 1) ? 3.0e30 : jT1, count, 1u, 1u,
                             jFade, jFadeR, jT1, cr.T, jn, jf);
                cr.emAll += jn + jf;
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
        // 2: inside a photon sphere and falling in (merger mode), 3: step budget used up,
        // 5: a secant's jets due (the caller makes them, then calls again: on from here).
        // Jets along the way: on secants of up to 16 RK4 steps, cut where the path turns by ~0.01 rad -- the secant
        // from jA to p (jA = p: none). The caller makes them (TraceChain's one JetLine call site: it is large).
        // s: the steps taken (the budget: across the calls of one leg).
        int IntegrateMedium(inout float3 p, inout float3 d, int count, uint mask, out int hitHole, inout int s,
                            out float3 jA)
        {
            hitHole = -1;
            int ret = 3;
            jA = p;
            float3 jDA = d;
            int jK = 0;
            bool jetsOn = false;
        #if defined(BH_JETS)
            [loop]
            for (int jh = 0; jh < count; jh++)
                jetsOn = jetsOn || (_BHJet[jh].x > 0.0);
        #endif
            bool due = false;
            [loop]
            for (; s < BH_MED_MAX_STEPS; s++)
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
                // jets on the secant since jA (always before leaving): back to the caller with it
                if (jetsOn && (stop || jK >= 16 || dot(d, jDA) < 0.9999 || s == BH_MED_MAX_STEPS - 1))
                {
                    due = true;
                    if (!stop)
                    {
                        s++;
                        ret = 5;
                    }
                    break;
                }
                if (stop)
                    break;
            }
            if (!due)
                jA = p;
            return ret;
        }

                // Holes whose regions don't overlap: one exact trace each (visit order is
        // unambiguous, so no seam). Overlapping regions (a group): the combined-field
        // integration, entered and left in plain local directions, with leading-order
        // weak segments on both sides (both tested against exact integration).
        // A separate hole's region reaching into another's (a light companion: C# keeps the big one's region whole
        // around it): traced first where a ray runs through it, then the big one (or handed over by its trace).
        // Not only the ones whose region is all inside -- one straddling the big region's edge was traced only
        // where it came first along the line: past a plane through the camera it vanished (with its lensing).
        // (the smaller region is the companion's: C# limits it to its tiny share of the distance)
        bool SatIn(float3 cS, float RS, float3 cH, float RH) { return RS < RH && length(cS - cH) - RS < RH; }
        // A group nested like that (light companions close to each other: their combined-field sphere inside the
        // big one's region) is one too: its sphere goes first where a ray runs into it -- the big one's trace from the
        // camera passed straight through it (half the screen, past the plane through the camera at right angles
        // to the big one, lost it). This hole's group (its members), or the hole alone.
        uint GroupBits(int i, int count)
        {
            if (!IsMember(i)) return 1u << (uint)i;
            uint m = 0u;
            [loop]
            for (int j = 0; j < count; j++)
                if (_BHSpins[j].z == _BHSpins[i].z) m |= 1u << (uint)j;
            return m;
        }
        // Is the big hole's field, along this line up to t (a nested one's closest approach), too strong for its weak
        // bend there -- the ray passing it close first? Only then does its trace hand over at the companion's distance
        // (the companion on the bent path past it); else the line goes straight on to the companion, the big one's bend
        // up to there weak (with its displacement). The series' first order left out: ~10 (Rs/b)^4 of the bend.
        // (A camera near a companion at about its distance from the big one -- the big one's closest approach a
        // shallow dip just ahead -- went through the hand-off's restart line: lines and bands of wrong pixels.)
        bool StrongBefore(float3 toT, float3 d, float rsT, float t)
        {
            float tc = dot(toT, d);
            if (tc >= t) return false;
            float b2 = max(dot(toT, toT) - tc * tc, 1e-30);
            float y2 = rsT * rsT / b2;
            float s1 = t - tc;
            float dg = s1 * rsqrt(s1 * s1 + b2) + tc * rsqrt(tc * tc + b2);   // g(t) - g(0)
            return 10.0 * y2 * y2 * sqrt(y2) * dg > 1e-4;
        }

        // is this group (bits) nested in a separate hole's region?
        bool GroupNested(uint g, int count)
        {
            [loop]
            for (int m = 0; m < count; m++)
            {
                if (((g >> (uint)m) & 1u) == 0u) continue;
                float3 cM = _BHPositionsCamRelative[m].xyz;
                float RM = DomainRadius(m);
                [loop]
                for (int h = 0; h < count; h++)
                    if (!IsMember(h) && SatIn(cM, RM, _BHPositionsCamRelative[h].xyz, DomainRadius(h)))
                        return true;
            }
            return false;
        }

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
            uint seen = 0u;               // groups this ray has been through (their spheres): never again as nested
            uint medGroup = 0u;           // the group whose sphere the ray is in (or last entered)
            uint bendDone = 0u;           // whole bend already applied (passed nested ones): no more bends or traces --
                                          // their jets still drawn on the pieces as before (not done)
                                          // ones (a hand-off's restart line, back from the big one, crossed the
                                          // sphere the ray had come through -- integrated twice: a band of garbage)
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
            // A straight piece to a satellite nested in a separate hole's region (to its closest approach, with its
            // bend there; or to its region) left out the hole's gas on it: a camera in the hole's disk saw it cut off
            // along the plane square to the hole, and a window through it around the satellite. The hole's trace
            // along the piece first, for its gas (and sheet disk) only -- the ray's path stays the piece's:
            // gasSt bits 0-7: holes whose gas on the coming piece is taken (cleared when it is run); 8: this exact
            // trace is that one (up to kT0 along the piece, nothing else kept from it); 12-15: 1 + the satellite it
            // went up to the middle of (its host's trace on from there). (One register: TraceChain spills.)
            uint gasSt = 0u;

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
                seen = GroupBits(camMember, count);
                medGroup = seen;
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

        #if defined(BH_JETS)
            // The jets on the straight pieces through ONE JetLine call site: it is large, and every call is inlined
            // -- seven of them in here made this function 16k instructions longer and spill 75 registers. Each piece
            // leaves its call (two at most) and the top of the next event makes it; a separate hole's leg leaves its
            // jets and comes back for the trace (nothing of them stays live across it).
            float3 jbO = float3(0, 0, 0), jbD = float3(0, 0, 1);
            float4 jbT = float4(0, 0, 0, 0);   // t0, t1, fadeR, weight
            uint4 jbM = uint4(0, 0, 0, 0);     // kinkMask, jetMask, fadeHole + 1, pending
            // a second one on the same line (a pass: its piece, then the passed hole's jets beyond it -- the same start,
            // weight and no fade): only what differs
            float jcT1 = 0.0;
            uint jcK = 0u, jcJ = 0u;
            bool jcOn = false;
            bool jPre = false;            // the leg's jets left: on to the trace
            #define JET_JOB(o_, d_, t0_, t1_, km_, jm_, fh_, fr_, w_) \
                { jbO = (o_); jbD = (d_); jbT = float4((t0_), (t1_), (fr_), (w_)); \
                  jbM = uint4((km_), (jm_), (uint)((fh_) + 1), 1u); }
            #define JET_JOB2(t1_, km_, jm_) { jcT1 = (t1_); jcK = (km_); jcJ = (jm_); jcOn = true; }
        #else
            #define JET_JOB(o_, d_, t0_, t1_, km_, jm_, fh_, fr_, w_)
            #define JET_JOB2(t1_, km_, jm_)
        #endif
            bool fin = false;             // ended: its jets left (made at the next top), then out
            bool medOn = false;           // a combined-field leg on after its secant's jets
            int medS = 0;                 // ...its steps so far

            [loop]
            for (int ev = 0; ev <= BH_MAX_EVENTS; ev++)
            {
            #if defined(BH_JETS)
                [loop]
                for (int jq = 0; jq < 2; jq++)
                {
                    if (jbM.w == 0u)
                        break;
                    float3 jn = float3(0, 0, 0), jf = float3(0, 0, 0);
                    JetLine(jbO, jbD, jbT.x, jbT.y, count, jbM.x, jbM.y, (int)jbM.z - 1, jbT.z, jn, jf);
                    cr.emAll += (jn + jf) * jbT.w;
                    jbT.y = jcT1; jbM.xy = uint2(jcK, jcJ);
                    jbM.w = jcOn ? 1u : 0u;
                    jcOn = false;
                }
                // empty: constants again (else they stayed live through the traces below, where the volume march
                // needs the registers most)
                jbO = float3(0, 0, 0); jbD = float3(0, 0, 1); jbT = float4(0, 0, 0, 0); jbM = uint4(0, 0, 0, 0);
                jcT1 = 0.0; jcK = 0u; jcJ = 0u;
            #endif
                if (fin || ev == BH_MAX_EVENTS)
                    break;
                uint mask = allMask & ~done;
                uint kmask = mask & ~bendDone;

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
                        if ((((done | noKick | bendDone) >> (uint)i) & 1u) != 0u) continue;
                        float3 toBHi = _BHPositionsCamRelative[i].xyz - p;
                        float Ri = DomainRadius(i);
                        if (hoPend && (((seen >> (uint)i) & 1u) != 0u || !SatIn(_BHPositionsCamRelative[i].xyz, Ri, cHo, RHo))) continue;
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
                    int host0 = -1;               // the target a nested one went before
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
                            if (n2 == target || (((done | seen | bendDone) >> (uint)n2) & 1u) != 0u) continue;
                            bool nMem = IsMember(n2);
                            if (nMem && IsMember(target)) continue;   // (groups: by their spheres' entries, above)
                            float3 cN = _BHPositionsCamRelative[n2].xyz;
                            float RN = DomainRadius(n2);
                            if (!SatIn(cN, RN, cT, RT)) continue;
                            float3 toN = cN - p;
                            float tcaN = dot(toN, d);
                            float ddN = dot(toN, toN);
                            if (ddN - tcaN * tcaN >= RN * RN) continue;
                            if (ddN >= RN * RN && tcaN <= 0.0) continue;
                            // (a group's member: where the line enters its sphere, as for any group)
                            float kN = nMem ? max(tcaN - sqrt(RN * RN - (ddN - tcaN * tcaN)), 0.0) : max(tcaN, 0.0);
                            // one the ray meets after the target's closest approach is met on the target's bent path:
                            // the target's trace stops at its distance and hands over (below) -- unless float can't
                            // place it in the target's frame. With that closest approach behind (moving away from the
                            // target: key 0) it goes first: the "passed" pieces below took it for one the line misses
                            // -- a weak bend, never traced (its disk gone from every ray leaving the target past it)
                            if (tKey > 0.0 && kN >= tKey && !IsMember(target) && RN >= 1e-4 * length(cN - cT)
                                && StrongBefore(cT - p, d, _BHParams[target].x, kN)) continue;
                            if (kN < nKey) { nKey = kN; nest = n2; }
                        }
                        if (nest >= 0) { host0 = target; target = nest; tKey = nKey; }
                    }
                    bool tMember = (target >= 0) && IsMember(target);
                    float tEnd = 3.0e38;
                    if (target >= 0)
                        tEnd = tMember ? tKey : max(dot(_BHPositionsCamRelative[target].xyz - p, d), 0.0);

                    // the nested one first: its host's gas on the way to it (to its middle -- its host traced on from
                    // there after it). Also when it came first by itself (met before its host's closest approach,
                    // nearer than that): a window in the gas around it, the camera in the disk or outside it
                    [branch]
                    if (host0 < 0 && target >= 0 && !hoPend)
                    {
                        float3 cTg = _BHPositionsCamRelative[target].xyz;
                        float RTg = DomainRadius(target);
                        [loop]
                        for (int g2 = 0; g2 < count; g2++)
                        {
                            if (g2 == target || IsMember(g2) || _BHSpins[g2].w <= 0.5) continue;
                            if (SatIn(cTg, RTg, _BHPositionsCamRelative[g2].xyz, DomainRadius(g2)))
                                host0 = g2;
                        }
                    }
                    [branch]
                    if (host0 >= 0 && !hoPend && _BHSpins[host0].w > 0.5 && ((gasSt >> (uint)host0) & 1u) == 0u
                        && (tMember ? tKey : tEnd) > 0.0)
                    {
                        kT0 = tMember ? tKey : tEnd;
                        gasSt = (gasSt & ~0xF000u) | (1u << (uint)host0) | 0x100u | (tMember ? 0u : (uint)(target + 1) << 12);
                        kHole = host0; kFromCam = firstSeg; kFinite = false;
                        mode = 2;
                        continue;
                    }
                    // (entering the group's sphere: after its host's gas on the way -- marked before, the group was
                    // taken as passed when the ray came back from that trace, half its image cut away)
                    if (tMember) { medGroup = GroupBits(target, count); seen |= medGroup; }

                    // nested holes the line passes outside their regions while the target's trace starts right here
                    // (it would ignore them; or after a hand-off, back on its outgoing line): a straight piece to the
                    // nearest one's closest approach with its whole weak bend (as on to infinity: its pull is all near
                    // it) and the others' up to there, its jets on that bent line; then the target's trace from there
                    // (its gas on the piece first, below)
                    int passIndex = -1;
                    float tPass = 3.0e38;
                    int pHost = hoPend ? hoHole : target;
                    // ...and from inside the target's region on its way in too: the ones it passed got their bend at
                    // the camera there, the gas in front of them bent by them (and a seam along the plane square to
                    // the target, where that switched to the pieces). Not when one will be handed over (strong field
                    // before it) or traced on this line: as before then.
                    // (Not on its way in -- a piece from inside its region toward it, the target's weak bend over it
                    // up to the satellite: sky directions 5-15 px off, its shadow thrown aside where the target's
                    // closest approach and the satellite's met. Those keep the satellite's bend at the start.)
                    bool passIn = false;
                    bool passNo = false;
                    [branch]
                    if (hoPend ? (target < 0) : (target >= 0 && !tMember && (tEnd <= 0.0 || passIn)))
                    {
                        float3 cT = _BHPositionsCamRelative[pHost].xyz;
                        float RT = DomainRadius(pHost);
                        float3 toTp = cT - p;
                        float tcTp = dot(toTp, d), ddTp = dot(toTp, toTp);
                        bool mayHandP = ((handed >> (uint)pHost) & 1u) == 0u;
                        [loop]
                        for (int n3 = 0; n3 < count; n3++)
                        {
                            if (n3 == pHost || (((done | bendDone) >> (uint)n3) & 1u) != 0u) continue;
                            float3 cN = _BHPositionsCamRelative[n3].xyz;
                            float RN3 = DomainRadius(n3);
                            if (!SatIn(cN, RN3, cT, RT)) continue;
                            // (only ones the line passes outside: one whose region it enters is traced)
                            float3 toN3 = cN - p;
                            float tcaN3 = dot(toN3, d), ddN3 = dot(toN3, toN3);
                            if (ddN3 < RN3 * RN3 || (tcaN3 > 0.0 && ddN3 - tcaN3 * tcaN3 < RN3 * RN3))
                            {
                                passNo = passNo || passIn;
                                continue;
                            }
                            [branch]
                            if (passIn)
                            {
                                float dNT3 = length(cN - cT);
                                // the target's field on the piece too strong for its weak bend there (as StrongBefore,
                                // from the start): a ray going on past the target close by was thrown off by it -- its
                                // shadow shifted and swelled on that side of the plane where the piece began
                                float b2P = max(ddTp - tcTp * tcTp, 1e-30);
                                float y2P = _BHParams[pHost].x * _BHParams[pHost].x / b2P;
                                float sP = max(tcaN3, 0.0) - tcTp;
                                float dgP = sP * rsqrt(sP * sP + b2P) + tcTp * rsqrt(tcTp * tcTp + b2P);
                                passNo = passNo || IsMember(n3) || 10.0 * y2P * y2P * sqrt(y2P) * dgP > 1e-4
                                      || (tcaN3 > tEnd && mayHandP && RN3 >= 1e-4 * dNT3
                                          && (ddTp < dNT3 * dNT3 || (tcTp > 0.0 && ddTp - tcTp * tcTp < dNT3 * dNT3))
                                          && StrongBefore(toTp, d, _BHParams[pHost].x, tcaN3));
                            }
                            // (a group behind -- just left: the rest of its bend, from here on, as this trace leaves it out)
                            float tN = max(tcaN3, 0.0);
                            if (tN < tPass) { tPass = tN; passIndex = n3; }
                        }
                    }
                    if (passNo)
                        passIndex = -1;
                    // (the target's gas on the piece -- on its way in if the piece ends before its closest approach;
                    // after a hand-off its first trace had it)
                    [branch]
                    if (passIndex >= 0 && !hoPend && tPass > 0.0 && _BHSpins[pHost].w > 0.5 && ((gasSt >> (uint)pHost) & 1u) == 0u)
                    {
                        kT0 = tPass;
                        gasSt |= (1u << (uint)pHost) | 0x100u;
                        kHole = pHost; kFromCam = firstSeg; kFinite = false;
                        mode = 2;
                        continue;
                    }
                    [branch]
                    if (passIndex >= 0)
                    {
                        float3 kP = float3(0, 0, 0), dP = float3(0, 0, 0);
                        uint passBits = GroupBits(passIndex, count) & ~(done | bendDone);
                        uint gMaskP = 0u;
                        [loop]
                        for (int w2 = 0; w2 < count; w2++)
                        {
                            if ((((done | noKick | passBits | bendDone) >> (uint)w2) & 1u) != 0u && w2 != passIndex) continue;
                            if (IsMember(w2)) { if (w2 != passIndex) gMaskP |= 1u << (uint)w2; continue; }
                            float3 toW = _BHPositionsCamRelative[w2].xyz - p;
                            float tW = (w2 == passIndex) ? 3.0e38 : tPass;
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
                        [branch]
                        if (IsMember(passIndex))
                        {
                            // a nested group passed: its whole bend (displacement left out, as for a hole's)
                            float3 kg, dg;
                            GroupWeakKick(p, d, 3.0e38, count, passBits, kg, dg);
                            kP += kg;
                        }
                        JET_JOB(p, d, jetSkip, tPass, 0u, mask & ~noKick & ~passBits, -1, 0.0, cr.T);
                        JET_JOB2(3.0e30, passBits, passBits);
                        p = p + d * tPass + dP;
                        d = ApplyKick(d, kP);
                        done |= passBits;
                        jetSkip = 0.0;
                        gasSt &= ~0xFFu;
                        continue;
                    }
                    [branch]
                    if (hoPend && target < 0)
                    {
                        // its satellites done: on from its stop point -- on this (bent) line where it passes nearest it,
                        // or the stop point itself if the line got there already (a bend after the stop acts there)
                        float tC = dot(hoP - p, d);
                        p = (tC > 0.0) ? p + d * tC : hoP;
                        gasSt &= ~0xFFu;
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
                    // satellites nested in the (separate) target's region whose regions this line enters: traced after
                    // it (its trace hands over), so their bend is their traces' -- not also a weak kick here, where the
                    // line runs into their strong field (it threw rays grazing them far off)
                    uint traceLater = 0u;
                    // ...and those the line passes outside before the target's closest approach: all of their bend
                    // here (on to infinity), then no more of it -- left to the hand-off, the restart line (back from
                    // the stop, near the camera when it sits at their distance from the target) met them again: bent
                    // twice (a ghost image beside a satellite, garbage bands along the plane square to the big one)
                    uint wholeNow = 0u, wholeNot = 0u;
                    [branch]
                    if (target >= 0 && !tMember)
                    {
                        float3 cT2 = _BHPositionsCamRelative[target].xyz;
                        float RT2 = DomainRadius(target);
                        // (will the target's trace stop at their distance? as it decides below)
                        float3 toT2 = cT2 - p;
                        float tcT2 = dot(toT2, d), ddT2 = dot(toT2, toT2);
                        bool mayHand = ((handed >> (uint)target) & 1u) == 0u;
                        [loop]
                        for (int w3 = 0; w3 < count; w3++)
                        {
                            if (w3 == target || (((done | seen | noKick | bendDone) >> (uint)w3) & 1u) != 0u) continue;
                            float3 cW = _BHPositionsCamRelative[w3].xyz;
                            float RW = DomainRadius(w3);
                            if (!SatIn(cW, RW, cT2, RT2)) continue;
                            float3 toW2 = cW - p;
                            float tcW = dot(toW2, d), ddW = dot(toW2, toW2);
                            float dWT = length(cW - cT2);
                            // ...also those met after it when no hand-off will come (float can't place them in the
                            // target's frame, or its leg stays outside their distance): their bend past the target's
                            // closest approach was dropped -- a dotted line of wrong pixels along the plane square to
                            // the target through the camera (closest approach just ahead), blocks after the rebuild
                            bool hand = mayHand && RW >= 1e-4 * dWT
                                     && (ddT2 < dWT * dWT || (tcT2 > 0.0 && ddT2 - tcT2 * tcT2 < dWT * dWT))
                                     && StrongBefore(toT2, d, _BHParams[target].x, tcW);
                            if (ddW < RW * RW || (tcW > 0.0 && ddW - tcW * tcW < RW * RW))
                                traceLater |= GroupBits(w3, count);
                            else if (tcW <= tEnd || !hand)
                                wholeNow |= 1u << (uint)w3;
                            else
                                wholeNot |= GroupBits(w3, count);
                        }
                        wholeNow &= ~(traceLater | wholeNot);   // (a group: all of it, or none)
                    }
                    uint gMask = 0u;
                    [loop]
                    for (int g = 0; g < count; g++)
                        if (((kmask >> (uint)g) & 1u) != 0u && IsMember(g)) gMask |= 1u << (uint)g;
                    gMask &= ~traceLater;
                    GroupWeakKickInf(p, d, tEnd, count, gMask, wholeNow, kick, disp);
                    [loop]
                    for (int w = 0; w < count; w++)
                    {
                        if ((((done | noKick | bendDone) >> (uint)w) & 1u) != 0u || IsMember(w) || w == target) continue;
                        float3 kw, dw;
                        if (target < 0)
                        {
                            kw = CoupledWeakKick(w, p, d, count, kmask, firstSeg, touchMed);
                            dw = float3(0, 0, 0);
                        }
                        else
                        {
                            // (no mass for the ones traced after: their bend is their trace's)
                            float rsW = (((traceLater >> (uint)w) & 1u) != 0u) ? 0.0 : _BHParams[w].x;
                            bool whole = ((wholeNow >> (uint)w) & 1u) != 0u;
                            float tW = whole ? 3.0e38 : tEnd;
                            float3 toWk = _BHPositionsCamRelative[w].xyz - p;
                            WeakKick(d, toWk, rsW, SpinJ(w) * (rsW > 0.0 ? 1.0 : 0.0), tW, 0.0, kw, dw);
                            if (whole) dw = WeakDisp(d, toWk, rsW, tEnd);
                            // from the camera: its finite distance, as the straight view past it has it (the rays just
                            // inside its region, traced exactly from the camera, have it too: a ~4 px seam at its edge)
                            if (firstSeg)
                                kw += NearCameraTermTo(d, toWk, rsW, tW, w);
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
                    gasSt &= ~0xFFu;
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
                        JET_JOB(p, jDir, jetSkip, jLen, tMember ? 0u : mask, mask, -1, 0.0, cr.T);
                        jetSkip = 0.0;
                    }
                    float3 newDir = ApplyKick(d, kick);
                    if (target < 0)
                    {
                        d = newDir;
                        reachedInfinity = true;
                        fin = true;
                        continue;
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
                    #if BH_GROUP_WEAK
                        bendDone |= wholeNow;
                    #endif
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
                    if (!medOn)
                    {
                        cr.traces++;
                        cr.path |= 1;
                        g_dbgTraces += 1.0;
                        medS = 0;
                    }
                    medOn = false;
                    int hit;
                    float3 jA;
                    int res = IntegrateMedium(p, d, count, mask, hit, medS, jA);
                #if defined(BH_JETS)
                    // its secant's jets: at the next top (then on along it, the same leg: res 5)
                    float3 chJ = p - jA;
                    float clJ = length(chJ);
                    [branch]
                    if (clJ > 0.0)
                        JET_JOB(jA, chJ / clJ, 0.0, clJ, 0u, mask, -1, 0.0, cr.T);
                    if (res == 5)
                    {
                        medOn = true;
                        ev--;     // (not an event)
                        continue;
                    }
                #endif
                    if (res == 2) { cr.captured = true; cr.why = BH_WHY_PHOTONSPH; fin = true; continue; }
                    if (res == 3) { cr.captured = true; cr.why = BH_WHY_STEPS; fin = true; continue; }
                    if (res == 0)
                    {
                        mode = 0;
                        fromMedium = true;
                        // out of a nested group: the rest of its bend at once, then no more of it -- the big
                        // one's trace, often starting right here, leaves it out: cut where that trace's closest
                        // approach went from ahead (bend up to there only) to behind (all of it), a line ~1 px off
                        uint gOut = medGroup & ~(done | bendDone);
                        [branch]
                        if (gOut != 0u && GroupNested(gOut, count))
                        {
                            float3 kg, dg;
                            GroupWeakKick(p, d, 3.0e38, count, gOut, kg, dg);
                            d = ApplyKick(d, kg);
                            bendDone |= gOut;
                        }
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
                    bool kGas = (gasSt & 0x100u) != 0u;
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
                    if (!kFinite && !kGas && ((handed >> (uint)h) & 1u) == 0u)
                    {
                        float RH = DomainRadius(h);
                        float tcaH = dot(toBH, d);
                        float dd2 = dot(toBH, toBH);
                        float b2H = dd2 - tcaH * tcaH;
                        [loop]
                        for (int s3 = 0; s3 < count; s3++)
                        {
                            if (s3 == h || (((done | seen | bendDone) >> (uint)s3) & 1u) != 0u) continue;
                            float3 cS = _BHPositionsCamRelative[s3].xyz;
                            float dSH = length(cS - bhPos);
                            float RS = DomainRadius(s3);
                            if (!SatIn(cS, RS, bhPos, RH) || RS < 1e-4 * dSH) continue;
                            // the leg dips inside the satellite's distance (stops where it comes back out to it), or
                            // its line runs through the satellite's region without doing so: starting out beyond that
                            // distance and moving away from the hole, or passing the hole farther out (the satellite's
                            // far side). Those stop at once (rOuter below the start) and the satellite goes first --
                            // they skipped it: half its disk (all of it, and its lensing, with the camera beside it
                            // on the far side) was missing, cut along the sphere through it around the hole.
                            float3 toS = cS - p;
                            float tcaS = dot(toS, d);
                            float ddS = dot(toS, toS);
                            bool throughS = (ddS < RS * RS) || (tcaS > 0.0 && ddS - tcaS * tcaS < RS * RS);
                            if (dd2 < dSH * dSH || (tcaH > 0.0 && b2H < dSH * dSH) || throughS)
                            {
                                rOuter = min(rOuter, dSH / rs);
                                tryHand = true;
                            }
                        }
                        if (tryHand)
                            escR = rOuter;
                    }
                    g_orderBase = (h == hoHole && !kGas) ? hoBase : 0.0;
                    if (!kGas)
                        noKick &= ~(1u << (uint)h);

                    // jets: a group's trace samples its hole's along the whole geodesic; a separate hole's
                    // trace inside min(JET_RGEO, its region), faded into the straight lines here: this one up to
                    // the closest approach (all holes' jets), the outgoing asymptote after it (next pieces)
                    float rGw = min(JET_RGEO, _BHParams[h].w) * rs;
                    float3 pBack = p;
                    float tPre = cr.T;
                    g_jetFootRs = g_jetPix * length(bhPos) / rs;
                    // aimed at the centre (JetOnLine): its own jet on the straight line, unfaded, up to the
                    // horizon (it falls in), and none along its trace
                    float3 jOff = bhPos - (kFinite ? kEntry : kP0);
                    float3 jDir = kFinite ? d : kD0;
                    float tcaJ = dot(jOff, jDir);
                    float m2J = max(dot(jOff, jOff) - tcaJ * tcaJ, 0.0);
                    bool onLine = (tcaJ > 0.0) && JetOnLine(sqrt(m2J), rs, _BHSpins[h].x);
                    float rHJ = (0.5 + sqrt(max(0.25 - 0.25 * _BHSpins[h].x * _BHSpins[h].x, 0.0))) * rs;
                    float tHJ = max(tcaJ - sqrt(max(rHJ * rHJ - m2J, 0.0)), 0.0);
                #if defined(BH_JETS)
                    [branch]
                    if (!kFinite && !kGas && !jPre)
                    {
                        // the jets up to the closest approach, on the leg's path bent through every hole left
                        // (displaced points: exactly what the straight view outside the region draws, from the
                        // same start); its own jet faded in where its trace samples it -- made at the next top
                        JET_JOB(kP0, kD0, jetSkip, kT0, mask, mask & ~noKick, onLine ? -1 : h, onLine ? 0.0 : rGw, cr.T);
                        jPre = true;
                        ev--;     // (not an event)
                        continue;
                    }
                    jPre = false;
                #endif

                    float3 color, exitPos, exitDir; bool escaped; float trans;
                    bool hf; float4 fc; float fi; float3 ef;
                    g_dbgTraces += 1.0;
                    // a gas-only trace: up to the Mino time of the piece's end on the straight line (dlambda = ds / r^2)
                    // -- smooth across its closest approach, where a stop radius switched legs (the bent path dips)
                    [branch]
                    if (kGas)
                    {
                        float tcS = dot(toBH, d) / rs;
                        float bS = max(sqrt(max(dot(toBH, toBH) / (rs * rs) - tcS * tcS, 0.0)), 1e-3);
                        g_lamStop = max((atan((kT0 / rs - tcS) / bS) + atan(tcS / bS)) / bS, 1e-9);
                    }
                    KerrTraceCore(
                        dT, toBH, rs,
                        _BHParams[h].y, _BHParams[h].z, escR,
                        _BHDiskNormals[h].xyz,
                        _BHSpins[h].x, _BHSpins[h].y,
                        _BHNoise, sampler_trilinear_repeat,
                        _BHRotationOffsets[h].x, _BHRotationOffsets[h].yz,
                        rOuter, true, _BHSpins[h].w > 0.5,
                        color, exitPos, exitDir, _BHAppearanceProperties1[h], _BHAppearanceProperties2[h],
                        escaped, trans, hf, fc, fi, ef, h, (onLine || kGas) ? 0.0 : (kFinite ? rOuter : min(JET_RGEO, _BHParams[h].w)));
                    g_lamStop = 0.0;

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
                    cr.emAll += color * cr.T;
                    cr.T *= trans;
                    // (a gas-only trace counts only where the ray ends in it: the same class as the hole's trace it
                    // stands for -- where the gas went opaque before or after the piece's end, the class changed, a
                    // line of full-res pixels along that contour, sharper than the rebuilt ones around it)
                    if (!kGas || cr.T < 0.01)
                        cr.traces++;

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
                            JET_JOB(kEntry, ch / chl, 0.0, onLine ? min(chl, tHJ) : chl, 0u, allMask & ~done,
                                    onLine ? -1 : h, onLine ? 0.0 : InnerRadius(h), tPre);
                    }
                    cr.anyStrong = true;
                    // (an opaque disk stops the ray without a capture: see BH_SINGLE)
                    // (a gas-only trace's end is not the ray's: only its gas can stop it)
                    // (ending here: the group trace's chord jets above, at the next top)
                    if (!escaped && trans >= 0.01 && !kGas) { cr.captured = true; cr.why = BH_WHY_HORIZON; fin = true; continue; }
                    if ((!escaped && !kGas) || cr.T < 0.01) { stopped = true; fin = true; continue; }

                    float3 fromHole = exitPos - toBH;
                    if (kFinite)
                    {
                        float rE = length(fromHole) / rs;
                        float3 pOut = bhPos + fromHole * (BLToIso(rE) / max(rE, 1e-6));
                        float3 dOut = ZamoToStatic(exitDir, ZamoVelocity(h, pOut));
                        float L = length(pOut - kEntry);
                        float3 f = ExternalGrad(h, count, kmask);
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
                        JET_JOB(kP0, kD0, max(kT0, jetSkip), tH, mask, 1u << (uint)h, h, rGw, cr.T);
                        // the satellite's encounter is on both sides of its distance: the next pieces start on this
                        // outgoing line back at its point nearest the hole (the satellite's bend from there, or its
                        // trace from there), their jets from the stop on, and no weak bend of this hole on them
                        float tB = min(dot(bhPos - pH, exitDir), 0.0);
                        float tB0 = tB;
                        // ...and back before the regions of the satellites nested in it that this line runs through (a
                        // leg grazing their distance stops at its closest approach -- inside the satellite's region when
                        // that is where it passes: its trace from there lost the near half of it, a dark line across it)
                        // -- there, around that point: not where the line, run back past it, crosses the incoming side
                        // (a group the ray had passed near the camera, integrated again from there: a band of garbage)
                        float RH4 = DomainRadius(h);
                        [loop]
                        for (int s4 = 0; s4 < count; s4++)
                        {
                            if (s4 == h || (((done | seen | bendDone) >> (uint)s4) & 1u) != 0u) continue;
                            float3 toS4 = _BHPositionsCamRelative[s4].xyz - pH;
                            float RS4 = DomainRadius(s4);
                            if (!SatIn(_BHPositionsCamRelative[s4].xyz, RS4, bhPos, RH4)) continue;
                            float tc4 = dot(toS4, exitDir);
                            float b24 = dot(toS4, toS4) - tc4 * tc4;
                            float tE4 = tc4 - sqrt(max(RS4 * RS4 - b24, 0.0));
                            if (b24 < RS4 * RS4 && tE4 >= tB0 - 2.0 * RS4)
                                tB = min(tB, tE4 - 1e-3 * RS4);
                        }
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
                    else if (kGas)
                    {
                        // gas only: the piece it went along comes next, from where it was
                        gasSt &= ~0x100u;
                        mode = 0;
                    }
                    else
                    {
                        // its own jet beyond the closest approach on the same path, behind the disk (later pieces
                        // leave it out: done); the others' from the asymptote on
                        JET_JOB(kP0, kD0, max(kT0, jetSkip), 3.0e30, mask, 1u << (uint)h, h, rGw, cr.T);
                        p = bhPos + fromHole;
                        d = exitDir;
                        done |= (1u << (uint)h);
                        mode = 0;
                        jetSkip = max(dot(pBack - p, d), 0.0);
                        // its host's gas taken up to its middle: the host's trace on from there (on its way out)
                        [branch]
                        if ((int)((gasSt >> 12) & 0xFu) == h + 1)
                        {
                            float tBk = min(dot(bhPos - p, d), 0.0);
                            p += d * tBk;
                            jetSkip -= tBk;
                            gasSt &= ~0xF000u;
                        }
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

        // The reconstruction class (compared for equality) + 0.5 if the texel has a first visible crossing to
        // re-shade. That flag is not part of the class: a cell can mix texels with and without one -- the disk's
        // faded outer edge, where it ends in a smooth fade-out, split the classes all around the disk (most of
        // pass 2 for the sheet disk). Abrupt edges still fail the transmittance test.
        float ChainClass(ChainResult cr)
        {
            return (cr.captured ? 1.0 : 0.0)
                 + 4.0 * min(cr.firstIdx, 7.0)
                 + 32.0 * (float) min(cr.traces, 15)
                 + 512.0 * (float) (cr.firstHole + 1)
                 + (cr.hasFirst ? 0.5 : 0.0);
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
                _BHNoise, sampler_trilinear_repeat, _BHRotationOffsets[h].x, _BHRotationOffsets[h].yz,
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


        // Is the scene nearer than every region (the near side of its sphere) at this reduced-res texel and a block
        // around it -- 3 x 3 depth taps over the cells that read it? (Distances along the texel's own ray: the taps'
        // differ by a block's angle, covered by the 2 % margin.)
        bool LowTexelHidden(float2 texcoord, float3 dir, int count)
        {
            float tNear = 3.0e38;
            [loop]
            for (int j = 0; j < count; j++)
                tNear = min(tNear, length(_BHPositionsCamRelative[j].xyz) - DomainRadius(j));
            float eMax = 0.98 * tNear * max(dot(dir, -UNITY_MATRIX_V[2].xyz), 0.0);   // eye depth bound
            [branch]
            if (eMax <= 0.0)
                return false;
            float2 uv0 = LowTexelUV(texcoord);
            [unroll]
            for (int y = -1; y <= 1; y++)
            {
                [unroll]
                for (int x = -1; x <= 1; x++)
                {
                    if (SceneEyeDepth(saturate(uv0 + float2(x, y) * _BHTraceGrid.xy)) >= eMax)
                        return false;
                }
            }
            return true;
        }

        // The jets on a weak-field pixel, rebuilt from the reduced-res trace: its weak-field texels integrated them
        // along the same straight line, blurred to the texel's footprint (g_jetPix), so bilinear interpolation
        // rebuilds them -- 1 / ds^2 of the work of integrating them per pixel. A cell across a region's edge has
        // texels inside it too: traces that escaped with no disk on the way (T = 1, no capture, no first crossing:
        // light = the jets, the region's edge is far out of the gas) -- what the pixels inside take as well. False
        // when a texel is anything else: the caller leaves those to the full-res trace. (Before, the whole region's
        // edge went to it wherever a jet's glow was near: a ring of full-res traces around the region.)
        bool TexelJetsOnly(float c, float T)
        {
            float k = floor(c);
            bool weak = abs(c - k - BH_WEAK_TAG) < 0.01;
            bool clear = (k >= 32.0) && (fmod(k, 2.0) < 1.0) && (c - k < 0.25) && (T >= 0.999);
            return weak || clear;
        }
        bool WeakJets(float2 uv, out float3 em)
        {
            em = 0;
            float2 p = uv * _BHTraceGrid.zw;
            int2 i0 = (int2) floor(p);
            float2 f = p - (float2) i0;
            int2 mx = (int2) _BHLowSize.xy - 1;
            int2 c00 = clamp(i0, 0, mx), c10 = clamp(i0 + int2(1, 0), 0, mx);
            int2 c01 = clamp(i0 + int2(0, 1), 0, mx), c11 = clamp(i0 + int2(1, 1), 0, mx);
            float4 cw = float4(_BHLow1.Load(int3(c00, 0)).w, _BHLow1.Load(int3(c10, 0)).w,
                               _BHLow1.Load(int3(c01, 0)).w, _BHLow1.Load(int3(c11, 0)).w);
            float4 a00 = _BHLow0.Load(int3(c00, 0)), a10 = _BHLow0.Load(int3(c10, 0));
            float4 a01 = _BHLow0.Load(int3(c01, 0)), a11 = _BHLow0.Load(int3(c11, 0));
            if (!(TexelJetsOnly(cw.x, a00.a) && TexelJetsOnly(cw.y, a10.a) && TexelJetsOnly(cw.z, a01.a) && TexelJetsOnly(cw.w, a11.a)))
                return false;
            em = lerp(lerp(a00.rgb, a10.rgb, f.x), lerp(a01.rgb, a11.rgb, f.x), f.y);
            return true;
        }

        bool Reconstruct(float2 uv, out float3 color)
        {
            color = 0;
            g_reconFail = 0;
            float2 p = uv * _BHTraceGrid.zw;   // (texel i at block corner i: see _BHTraceGrid)
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

            float4 cw = float4(b00.w, b10.w, b01.w, b11.w);
            float4 ck = floor(cw);
            if (ck.x != ck.y || ck.x != ck.z || ck.x != ck.w) { g_reconFail = 1; return false; }
            float cls = ck.x;
            if (cls < 32.0) { g_reconFail = 2; return false; }
            float4 hv = step(0.25, cw - ck);   // per texel: has a first crossing to re-shade (ChainClass)
            bool captured = (fmod(cls, 2.0) >= 1.0);
            bool hasFirst = any(hv > 0.0);
            float firstIdx = fmod(floor(cls / 4.0), 8.0);
            int firstHole = (int) floor(cls / 512.0) - 1;

            float4 a00 = _BHLow0.Load(int3(c00, 0));
            float4 a10 = _BHLow0.Load(int3(c10, 0));
            float4 a01 = _BHLow0.Load(int3(c01, 0));
            float4 a11 = _BHLow0.Load(int3(c11, 0));
            float tMin = min(min(a00.a, a10.a), min(a01.a, a11.a));
            float tMax = max(max(a00.a, a10.a), max(a01.a, a11.a));
            if (tMax - tMin > BH_UPS_MAX_DT) { g_reconFail = 3; return false; }

            // Emission bilinear interpolation cannot rebuild: a feature thinner than the cell (the photon ring's
            // higher-order images, a hot spot's core) caught by one texel and not its neighbours was spread into
            // a bright ds x ds block. Compared on the compressed scale the tonemapper shows, L / (L + knee).
            float4 eL = float4(dot(a00.rgb, BH_LUMA), dot(a10.rgb, BH_LUMA), dot(a01.rgb, BH_LUMA), dot(a11.rgb, BH_LUMA));
            float4 eQ = eL / (eL + BH_UPS_EM_KNEE);
            if (max(max(eQ.x, eQ.y), max(eQ.z, eQ.w)) - min(min(eQ.x, eQ.y), min(eQ.z, eQ.w)) > BH_UPS_MAX_DEM)
            { g_reconFail = 8; return false; }

            // Sky hidden behind opaque emission in all four texels: its direction never
            // reaches the screen, so it must not decide whether this cell is rebuilt.
            bool skyVisible = !captured && tMax >= 0.01;

            float3 dir = 0;
            float skyFoot = 0.0;
            if (skyVisible)
            {
                // Only texels that see sky (T >= 0.01) have a direction that matters: behind an opaque disk
                // the trace stops (no exit direction). Their sky is weighted by T, so the direction is
                // interpolated T-weighted and the spread / twist tests only compare texels that see it.
                float4 v = step(0.01, float4(a00.a, a10.a, a01.a, a11.a));
                float texelAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _BHLowSize.y);
                float cosMax = (tMax < BH_UPS_SKY_T) ? -2.0 : cos(BH_UPS_MAX_SPREAD * texelAngle);
                float dMin = 1.0;
                if (v.x * v.y > 0.0) dMin = min(dMin, dot(b00.xyz, b10.xyz));
                if (v.x * v.z > 0.0) dMin = min(dMin, dot(b00.xyz, b01.xyz));
                if (v.w * v.y > 0.0) dMin = min(dMin, dot(b11.xyz, b10.xyz));
                if (v.w * v.z > 0.0) dMin = min(dMin, dot(b11.xyz, b01.xyz));
                if (dMin < cosMax) { g_reconFail = 4; return false; }

                float pixAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                // per-pixel footprint from the edges of the cell that see sky (x: 00-10 / 01-11, y: 00-01 / 10-11)
                float lx = max(v.x * v.y * length(b10.xyz - b00.xyz), v.z * v.w * length(b11.xyz - b01.xyz));
                float ly = max(v.x * v.z * length(b01.xyz - b00.xyz), v.y * v.w * length(b11.xyz - b10.xyz));
                bool allSky = all(v > 0.0);
                if (allSky && tMax >= BH_UPS_SKY_T)
                {
                    float perPixMin = min(length(b10.xyz - b00.xyz), length(b01.xyz - b00.xyz)) / _BHDownsample;
                    float3 twist = (b00.xyz + b11.xyz) - (b10.xyz + b01.xyz);
                    float twistMax = BH_UPS_MAX_TWIST * perPixMin;
                    if (dot(twist, twist) > twistMax * twistMax) { g_reconFail = 5; return false; }
                }

                float4 wT = float4((1.0 - f.x) * (1.0 - f.y), f.x * (1.0 - f.y), (1.0 - f.x) * f.y, f.x * f.y)
                          * float4(a00.a, a10.a, a01.a, a11.a) * v;
                dir = normalize(wT.x * b00.xyz + wT.y * b10.xyz + wT.z * b01.xyz + wT.w * b11.xyz + 1e-20);
                skyFoot = max(max(lx, ly) / _BHDownsample, pixAngle);
            }

            float3 emFirst = 0;
            if (hasFirst)
            {
                float4 d00 = _BHLow2.Load(int3(c00, 0));
                float4 d10 = _BHLow2.Load(int3(c10, 0));
                float4 d01 = _BHLow2.Load(int3(c01, 0));
                float4 d11 = _BHLow2.Load(int3(c11, 0));
                // texels without a crossing (their faint emission is in a.rgb) take the crossing of one that
                // has it for the tests and the gradients, and get no weight in the interpolation
                float4 dRef = (hv.x > 0.0) ? d00 : ((hv.y > 0.0) ? d10 : ((hv.z > 0.0) ? d01 : d11));
                d00 = (hv.x > 0.0) ? d00 : dRef;
                d10 = (hv.y > 0.0) ? d10 : dRef;
                d01 = (hv.z > 0.0) ? d01 : dRef;
                d11 = (hv.w > 0.0) ? d11 : dRef;
                float rMin = min(min(d00.x, d10.x), min(d01.x, d11.x));
                float rMax = max(max(d00.x, d10.x), max(d01.x, d11.x));
                if (rMax - rMin > BH_UPS_MAX_DR * rMin) { g_reconFail = 6; return false; }
                d10.y = d00.y + WrapPi(d10.y - d00.y);
                d01.y = d00.y + WrapPi(d01.y - d00.y);
                d11.y = d00.y + WrapPi(d11.y - d00.y);
                float phMin = min(min(d00.y, d10.y), min(d01.y, d11.y));
                float phMax = max(max(d00.y, d10.y), max(d01.y, d11.y));
                if (phMax - phMin > BH_UPS_MAX_DPHI) { g_reconFail = 7; return false; }
                float4 wF = float4((1.0 - f.x) * (1.0 - f.y), f.x * (1.0 - f.y), (1.0 - f.x) * f.y, f.x * f.y) * hv;
                float wSum = wF.x + wF.y + wF.z + wF.w;   // > 0: hasFirst
                float4 fc = (wF.x * d00 + wF.y * d10 + wF.z * d01 + wF.w * d11) / max(wSum, 1e-6);

                float2 gR = float2(0.5 * ((d10.x - d00.x) + (d11.x - d01.x)),
                                   0.5 * ((d01.x - d00.x) + (d11.x - d10.x)));
                float2 gP = float2(0.5 * ((d10.y - d00.y) + (d11.y - d01.y)),
                                   0.5 * ((d01.y - d00.y) + (d11.y - d10.y)));
                // (texture units per radian round the disk, and per unit radius: DiskTexUV, log-polar)
                float su = (DISK_TEX_TILES2 / TWO_PI) * _BHNoiseSize.x / _BHDownsample;
                float sv = (DISK_TEX_TILES2 / TWO_PI) / (fc.x + DISK_TEX_RCORE * LoadHoleDisk(firstHole).rIsco) * _BHNoiseSize.y / _BHDownsample;
                float2 tx = float2(gP.x * su, gR.x * sv);
                float2 ty = float2(gP.y * su, gR.y * sv);
                float lod = max(0.5 * log2(max(max(dot(tx, tx), dot(ty, ty)), 1e-12)), 0.0);

                float op;
            #if !defined(BH_VOLUMETRIC_DISK)
                // (volumes are not a point: they report no first crossing, all their emission is in a.rgb)
                emFirst = ShadeFirstCrossing(firstHole, fc, firstIdx, lod, op) * wSum;
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
            #pragma multi_compile_fragment _ BH_JETS

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

                float sceneDist = sceneEye / max(dot(dir, camFwd0), 1e-4);
                if (!isSkyPixel && sceneDist < tFirst)
                {
                #if BH_DEBUG_COST
                    return float4(0, 0, 0, 1);
                #endif
                    float3 sc = sceneColor;
                #if defined(BH_JETS)
                    // outside every region: the jets in front of it (a planet far behind a jet cut it out). Their
                    // light from the reduced-res weak-field texels, where the jet nearest the ray crosses it before
                    // the scene does; where those are missing (a cell across a region's edge) nothing is added.
                    float3 jetEm;
                    [branch]
                    if (tFirst > 1e30 && _BHDownsample > 1.5 && JetCrossingBefore(dir, sceneDist, count)
                        && WeakJets(input.texcoord, jetEm))
                        sc += ObserverShift(jetEm);
                #endif
                    return float4(sc, 1);
                }

                // sky with no visible bending (the scene as it is), or weak field only
                bool skyOnly = isSkyPixel && !visibleBend;
                if (skyOnly || tFirst > 1e30)
                {
                #if BH_DEBUG_COST
                    return skyOnly ? float4(0, 0, 0, 1) : float4(0, 0, 0.6, 1);
                #endif
                    // (only sky pixels get here: in front of the scene the early-out above took them)
                    float3 bg = sceneColor;
                    [branch]
                    if (!skyOnly)
                        bg = SampleSky(WeakOnlyDir(dir, count), pixAngle);
                #if defined(BH_JETS)
                    // The jets: from the reduced-res trace's weak-field texels (WeakJets). Where it has none (no
                    // reduced-res pass, or a cell across a region's edge) and a jet is near: left to pass 2, whose
                    // trace takes this same straight-line path with the jets on it. (Integrating them here made
                    // this pass -- every pixel of the screen -- carry the jet code's registers.)
                    float3 jetEm = float3(0, 0, 0);
                    bool haveJets = (_BHDownsample > 1.5) && WeakJets(input.texcoord, jetEm);
                    uint allM = (1u << (uint)count) - 1u;
                    [branch]
                    if (!haveJets && JetNearAny(float3(0, 0, 0), dir, 0.0, 3.0e30, count, allM, allM))
                    {
                        discard;
                        return float4(0, 0, 0, 1);
                    }
                    bg += jetEm;
                #endif
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
            #pragma multi_compile_fragment _ BH_JETS

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

                float3 dir = PixelRayDir(LowTexelUV(input.texcoord));
                int count = min(_BHCount, 8);
                int camHole = CameraHole(count);
                bool weakOnly = FirstDomainEntry(dir, count, camHole) > 1e30;
                // behind the scene here and all around (a planet in front of the hole): nothing of it is seen. Left
                // untraced (class 0): a cell with it rebuilds no pixel, the visible ones of it get the full-res trace.
                [branch]
                if (!weakOnly && camHole < 0 && LowTexelHidden(input.texcoord, dir, count))
                    return o;
                [branch]
                if (weakOnly)
                {
                    // Weak-field texels are read by the cells of pixels just inside a region (BH_SINGLE: those
                    // next to its silhouette), and by the composite for the jets on weak-field pixels (WeakJets):
                    // traced (its straight-line path: the weak bend and the jets) only where one of those needs it
                    o.t1.w = BH_WEAK_TAG;
                    bool need = false;
                #if defined(BH_SINGLE)
                    float3 toB = _BHPositionsCamRelative[0].xyz;
                    float dB = length(toB);
                    float sinR = saturate(DomainRadius(0) / max(dB, 1e-30));
                    float marg = 2.0 * g_jetPix;   // two texels (g_jetPix = the texel's angle here)
                    need = (dot(toB, dir) >= dB * cos(min(asin(sinR) + marg, PI)));
                #endif
                #if defined(BH_JETS)
                    uint allM = (1u << (uint)count) - 1u;
                    need = need || JetNearAny(float3(0, 0, 0), dir, 0.0, 3.0e30, count, allM, allM);
                #endif
                    if (!need)
                        return o;
                }

                g_bhLowRes = true;
                g_bhDiskLod = log2(max(_BHDownsample, 1.0));
                ChainResult cr;
                TraceChain(dir, count, camHole, cr);
                o.t0 = float4(cr.emAll - cr.emFirst, cr.T);
                o.t1 = float4(cr.dir, ChainClass(cr));
                // Outside the regions (TraceChain took its straight-line path: the weak bend and the jets on it).
            #if defined(BH_SINGLE)
                // The class of an escaping single-hole trace that crossed no disk (traces = 1 -> 32, firstHole = 0
                // -> 512), so cells straddling the region edge match and reconstruct instead of all failing the
                // class test and going to the full-res trace. With the jets: without them, those cells faded the
                // jets toward 0 (a dark line across each jet where it leaves the region).
                if (weakOnly)
                    o.t1.w = 544.0 + BH_WEAK_TAG;
            #else
                // (a class Reconstruct never takes, < 32, as before: only the jets are read from these)
                if (weakOnly)
                    o.t1.w = 16.0 + BH_WEAK_TAG;
            #endif
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
            #pragma multi_compile_fragment _ BH_JETS

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
                // direction (taken at top level, after the trace, so they are valid). A neighbour behind an
                // opaque disk has no exit direction (its ray direction is left in cr.dir): across it the
                // difference means nothing -- it blurred the sky along every disk edge to the top mip.
                float opq = (!cr.captured && cr.T < 0.01) ? 1.0 : 0.0;
                float3 dDx = ddx(cr.dir);
                float3 dDy = ddy(cr.dir);
                bool okX = (ddx(opq) == 0.0), okY = (ddy(opq) == 0.0);
                float skyFoot = max(okX ? length(dDx) : 0.0, okY ? length(dDy) : 0.0);
                if (!okX && !okY)
                    skyFoot = 2.0 / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
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
