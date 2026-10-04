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

        int _BHCount;
        float4 _BHSpins[8];               // x = spin a/M, y = Kepler factor, z = 1 if member of a close group, w = 1 if disk on
        float4 _BHSpinAxis[8];   // xyz unit spin axis, w = M a^2 (world^3)  (C#)
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
            disp = toInf ? float3(0, 0, 0) : C * (h1 - h0 - t1 * g0) * toward;

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
        float3 NearCameraTerm(float3 d, float3 toBH, float rs)
        {
            float tca = dot(toBH, d);
            float3 pOff = tca * d - toBH;
            float b = max(length(pOff), 1e-6);
            float r0 = max(length(toBH), 1e-6);
            float g0 = -tca / sqrt(tca * tca + b * b);
            return (rs * rs / (b * r0)) * 0.5 * (1.0 - g0) * (pOff / b);
        }

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
                p2 = p + d * ti + disp - d2 * ti;   // the bent line, re-anchored
            }
            float hoRi = (touchMed && _BHSpins[i].z > 0.5) ? DomainRadius(i) : 0.0;
            float3 toI2 = _BHPositionsCamRelative[i].xyz - p2;
            float3 ki, di;
            WeakKick(d2, toI2, _BHParams[i].x, SpinJ(i), 3.0e38, hoRi, ki, di);
            if (fromCam)
                ki += NearCameraTerm(d2, toI2, _BHParams[i].x);
            return ki;
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
        #define BH_UPS_MAX_DT     0.4
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
        float4 _BHFullSize;              // full-res size: xy pixels, zw 1/pixels (C#)
        #define BH_CLASS_WEAK 16384.0    // low-res texel outside every region (weak bend only, 3+ holes)

        // Low-res texel i is traced exactly at full-res pixel ds*i: every reconstruction cell is an
        // aligned ds x ds pixel block, so whole 2x2 quads pass or fail together.
        float2 LowTexelUV(float2 lowUV)
        {
            float2 i = floor(lowUV * _BHLowSize.xy);
            return (i * _BHDownsample + 0.5) * _BHFullSize.zw;
        }

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
            int cbdN;
        };

        ChainResult InitChain(float3 dir)
        {
            ChainResult cr;
            cr.emAll = 0; cr.emFirst = 0; cr.T = 1.0; cr.dir = dir;
            cr.captured = false; cr.anyStrong = false; cr.traces = 0; cr.firstHole = -1;
            cr.hasFirst = false; cr.firstCross = 0; cr.firstIdx = 0; cr.why = BH_WHY_NONE;
            cr.path = 0;
            cr.cbdN = 0;
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

        // Ray that never enters a region: every hole's weak bend to infinity, each on
        // the line as bent by the others (see CoupledWeakKick).
        float3 WeakOnlyDir(float3 dir, int count)
        {
            uint allMask = (1u << (uint)count) - 1u;
            float3 kick = float3(0, 0, 0);
            [loop]
            for (int w = 0; w < count; w++)
                kick += CoupledWeakKick(w, float3(0, 0, 0), dir, count, allMask, true, false);
            return ApplyKick(dir, kick);
        }

        // ==================================================================
        // Circumbinary disk (one, around the closest close pair; C# fades it in)
        // ==================================================================
        #define BH_CBD_ID 8

        // lapse of the superposed static field at x: redshift of a resting emitter
        float StaticLapse(float3 x, int count)
        {
            float S = 0.0;
            [loop]
            for (int i = 0; i < count; i++)
            {
                float3 dx = x - _BHPositionsCamRelative[i].xyz;
                S += _BHSpinJ[i].w * rsqrt(max(dot(dx, dx), 1e-30));
            }
            return max(1.0 - 0.5 * S, 1e-3) / (1.0 + 0.5 * S);
        }
        // (rW, phi, nR, nPhi) of a ray crossing the shared disk's plane at x, trace direction d
        float4 CBDCrossingState(float3 x, float3 d, int count)
        {
            float3 nrm = _BHCBD1.xyz;
            float3 rel = x - _BHCBD0.xyz;
            float3 relP = rel - dot(rel, nrm) * nrm;
            float r = max(length(relP), 1e-6);
            float3 bx, by, bz;
            BuildSpinBasis(nrm, bx, by, bz);
            float phi = atan2(dot(relP, by), dot(relP, bx));
            float sp, cp;
            sincos(phi, sp, cp);
            float3 eR = cp * bx + sp * by;      // increasing r
            float3 ePhi = -sp * bx + cp * by;   // increasing phi (same convention as the trace)
            return float4(r, phi, dot(d, eR), dot(d, ePhi));
        }

        void CBDCross(float3 x, float3 d, int count, inout ChainResult cr, inout bool anyEm)
        {
            float4 s = CBDCrossingState(x, d, count);
            int idx = cr.cbdN;
            cr.cbdN++;
            float op;
            float3 em = ShadeCBDAt(s.x, s.y, s.z, s.w, idx, 0.0, _BHCBD2.w, _BHNoise, sampler_BHNoise, op);
            if (op <= 0.0)
                return;
            if (!anyEm)
            {
                cr.hasFirst = true;
                cr.firstHole = BH_CBD_ID;
                cr.firstCross = s;
                cr.firstIdx = (float)idx;
                cr.emFirst = em;
                anyEm = true;
                g_bhSkipFirstColor = false;
            }
            cr.emAll += em * cr.T;
            cr.T *= 1.0 - op;
        }

        float3 CBDLineEmission(float3 d, int count, out float T)
        {
            T = 1.0;
            if (_BHCBD2.w <= 0.0) return float3(0, 0, 0);
            float dn = dot(d, _BHCBD1.xyz);
            if (abs(dn) < 1e-8) return float3(0, 0, 0);
            float t = dot(_BHCBD0.xyz, _BHCBD1.xyz) / dn;
            if (t <= 0.0) return float3(0, 0, 0);
            float4 s = CBDCrossingState(d * t, d, count);
            float op;
            float3 em = ShadeCBDAt(s.x, s.y, s.z, s.w, 0, 0.0, _BHCBD2.w, _BHNoise, sampler_BHNoise, op);
            T = 1.0 - op;
            return em;
        }

        // shared disk on a straight piece p + t d, t in (0, tMax]
        void CBDRay(float3 p, float3 d, float tMax, int count, inout ChainResult cr, inout bool anyEm)
        {
            if (_BHCBD2.w <= 0.0) return;
            float dn = dot(d, _BHCBD1.xyz);
            if (abs(dn) < 1e-8) return;
            float t = -dot(p - _BHCBD0.xyz, _BHCBD1.xyz) / dn;
            if (t <= 0.0 || t > tMax) return;
            CBDCross(p + d * t, d, count, cr, anyEm);
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
                kick += NearCameraTerm(dir, toBH, rs);
                cr.dir = ApplyKick(dir, kick);
                return;
            }

            float3 color, exitPos, exitDir; bool escaped; float trans;
            bool hf; float4 fc; float fi; float3 ef;
            g_dbgTraces += 1.0;
            KerrTraceCore(
                dir, toBH, rs,
                _BHParams[0].y, _BHParams[0].z, _BHParams[0].w,
                _BHDiskNormals[0].xyz,
                _BHSpins[0].x, _BHSpins[0].y,
                _BHNoise, sampler_BHNoise,
                _BHRotationOffsets[0].x, _BHRotationOffsets[0].yz,
                1e30, true, _BHSpins[0].w > 0.5,
                color, exitPos, exitDir, _BHAppearanceProperties1[0], _BHAppearanceProperties2[0],
                escaped, trans, hf, fc, fi, ef, 0);

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
        #define BH_MED_STEP_FAR    0.35
        #define BH_MED_STEP_GROW   0.12
        #define BH_MED_MAX_STEPS   320
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
                float Ma2 = _BHSpinAxis[i].w;
                if (Ma2 > 0.0)
                {
                    float3 sh = _BHSpinAxis[i].xyz;
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

        // RK4 through the medium (shades the shared disk on the way). Returns
        // 0: left every group sphere, 1: reached the exact sphere of hitHole,
        // 2: inside a photon sphere and falling in (merger mode), 3: step budget used up,
        // 4: became opaque (shared disk).
        int IntegrateMedium(inout float3 p, inout float3 d, int count, uint mask, out int hitHole,
                            inout ChainResult cr, inout bool anyEm)
        {
            hitHole = -1;
            bool cbdOn = (_BHCBD2.w > 0.0);
            [loop]
            for (int s = 0; s < BH_MED_MAX_STEPS; s++)
            {
                float rMin = 3.0e38;
                float rsNear = 1.0;
                float dIn = 3.0e38;
                bool inside = false;
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
                            return 2;
                    }
                }
                if (!inside)
                    return 0;

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
                bool hitInner = false;
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
                        hitInner = true;
                        break;
                    }
                }

                // shared disk crossed on this (possibly shortened) step
                if (cbdOn)
                {
                    float sa = dot(p0 - _BHCBD0.xyz, _BHCBD1.xyz);
                    float sb = dot(p - _BHCBD0.xyz, _BHCBD1.xyz);
                    if (sa * sb < 0.0)
                    {
                        float t = sa / (sa - sb);
                        CBDCross(lerp(p0, p, t), normalize(lerp(d0, d, t)), count, cr, anyEm);
                        if (cr.T < 0.01)
                            return 4;
                    }
                }
                if (hitInner)
                    return 1;
            }
            return 3;
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
                    [loop]
                    for (int i = 0; i < count; i++)
                    {
                        if (((done >> (uint)i) & 1u) != 0u) continue;
                        float3 toBHi = _BHPositionsCamRelative[i].xyz - p;
                        float Ri = DomainRadius(i);
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
                    bool tMember = (target >= 0) && IsMember(target);

                    float tEnd = 3.0e38;
                    if (target >= 0)
                        tEnd = tMember ? tKey : max(dot(_BHPositionsCamRelative[target].xyz - p, d), 0.0);

                    CBDRay(p, d, tEnd, count, cr, anyEm);
                    if (cr.T < 0.01) { stopped = true; break; }

                    // segments touching a group: its members' higher-order bending
                    // happens inside the group region, not here
                    bool touchMed = tMember || fromMedium;
                    float3 kick = float3(0, 0, 0);
                    float3 disp = float3(0, 0, 0);
                    [loop]
                    for (int w = 0; w < count; w++)
                    {
                        if (((done >> (uint)w) & 1u) != 0u || (w == target && !tMember)) continue;
                        float3 kw, dw;
                        if (target < 0)
                        {
                            // last segment, out to infinity: coupled bends (no seam at the
                            // region edges of the holes this ray just missed)
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
                    fromMedium = false;
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
                        p = pEnd - newDir * tEnd;
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
                    if (!kFromCam)
                    {
                        float rho = length(toBH) / rs;
                        toBH *= IsoToBL(rho) / rho;
                        if (kFinite)
                            dT = StaticToZamo(d, ZamoVelocity(h, p));
                    }
                    float rOuter = kFinite ? IsoToBL(_BHParams[h].y) : 1e30;
                    float escR = kFinite ? rOuter : _BHParams[h].w;

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
                        escaped, trans, hf, fc, fi, ef, h);

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
                    anyEm = anyEm || hf;
                    kerrCount++;
                    cr.traces++;
                    cr.emAll += color * cr.T;
                    cr.T *= trans;
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
                    else
                    {
                        p = bhPos + fromHole;
                        d = exitDir;
                        done |= (1u << (uint)h);
                        mode = 0;
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

        // first visible emission, re-shaded at this pixel (a hole disk or the shared disk)
        float3 ShadeFirstCrossing(int h, float4 fc, float idx, float lod, out float opacity)
        {
            [branch]
            if (h == BH_CBD_ID)
                return ShadeCBDAt(fc.x, fc.y, fc.z, fc.w, (int) idx, lod, _BHCBD2.w, _BHNoise, sampler_BHNoise, opacity);

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
            float2 p = (uv * _BHFullSize.xy - 0.5) / _BHDownsample;
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

            if (b00.w != b10.w || b00.w != b01.w || b00.w != b11.w) return false;
            float cls = b00.w;
            if (cls < 32.0) return false;
            if (cls >= BH_CLASS_WEAK) return false;
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
            if (tMax - tMin > BH_UPS_MAX_DT) return false;

            float3 dir = 0;
            float skyFoot = 0.0;
            if (!captured)
            {
                float texelAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _BHLowSize.y);
                float cosMax = cos(BH_UPS_MAX_SPREAD * texelAngle);
                float dMin = min(min(dot(b00.xyz, b10.xyz), dot(b00.xyz, b01.xyz)),
                                 min(dot(b11.xyz, b10.xyz), dot(b11.xyz, b01.xyz)));
                if (dMin < cosMax) return false;

                // Bilinear interpolation error is about |twist| / 4 in sky angle, which
                // is |twist| / (4 x sky angle per pixel) in SCREEN pixels. Accept the cell
                // if that stays under a quarter pixel: where lensing squeezes the sky, a
                // large sky error is still a tiny shift on screen; where it magnifies,
                // the test gets stricter.
                float pixAngle = 2.0 / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                float l1 = length(b10.xyz - b00.xyz);
                float l2 = length(b01.xyz - b00.xyz);
                float perPixMin = min(l1, l2) / _BHDownsample;
                float perPixMax = max(l1, l2) / _BHDownsample;
                float3 twist = (b00.xyz + b11.xyz) - (b10.xyz + b01.xyz);
                float twistMax = BH_UPS_MAX_TWIST * perPixMin;
                if (dot(twist, twist) > twistMax * twistMax) return false;

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
                if (rMax - rMin > BH_UPS_MAX_DR * rMin) return false;
                d10.y = d00.y + WrapPi(d10.y - d00.y);
                d01.y = d00.y + WrapPi(d01.y - d00.y);
                d11.y = d00.y + WrapPi(d11.y - d00.y);
                float phMin = min(min(d00.y, d10.y), min(d01.y, d11.y));
                float phMax = max(max(d00.y, d10.y), max(d01.y, d11.y));
                if (phMax - phMin > BH_UPS_MAX_DPHI) return false;
                float4 fc = lerp(lerp(d00, d10, f.x), lerp(d01, d11, f.x), f.y);

                float2 gR = float2(0.5 * ((d10.x - d00.x) + (d11.x - d01.x)),
                                   0.5 * ((d01.x - d00.x) + (d11.x - d10.x)));
                float2 gP = float2(0.5 * ((d10.y - d00.y) + (d11.y - d01.y)),
                                   0.5 * ((d01.y - d00.y) + (d11.y - d10.y)));
                if (firstHole == BH_CBD_ID)
                    gR *= lerp(6.0 / max(_BHCBD0.w, 1e-6), 1.0 / max(_BHCBD7.x, 1e-6), _BHCBD7.w);
                float su = (4.0 / TWO_PI) * _BHNoiseSize.x / _BHDownsample;
                float sv = 0.2 * _BHNoiseSize.y / _BHDownsample;
                float2 tx = float2(gP.x * su, gR.x * sv);
                float2 ty = float2(gP.y * su, gR.y * sv);
                float lod = max(0.5 * log2(max(max(dot(tx, tx), dot(ty, ty)), 1e-12)), 0.0);

                float op;
                emFirst = ShadeFirstCrossing(firstHole, fc, firstIdx, lod, op);
            }

            float4 a = lerp(lerp(a00, a10, f.x), lerp(a01, a11, f.x), f.y);
            color = emFirst + a.rgb;
            if (!captured && a.a >= 0.01)
                color += SampleSky(dir, skyFoot) * a.a;
            return true;
        }

        bool ReconstructWeakDir(float2 uv, out float3 dir)
        {
            dir = 0;
            float2 p = (uv * _BHFullSize.xy - 0.5) / _BHDownsample;
            int2 i0 = (int2) floor(p);
            float2 f = p - (float2) i0;
            int2 mx = (int2) _BHLowSize.xy - 1;
            float4 b00 = _BHLow1.Load(int3(clamp(i0, 0, mx), 0));
            float4 b10 = _BHLow1.Load(int3(clamp(i0 + int2(1, 0), 0, mx), 0));
            float4 b01 = _BHLow1.Load(int3(clamp(i0 + int2(0, 1), 0, mx), 0));
            float4 b11 = _BHLow1.Load(int3(clamp(i0 + int2(1, 1), 0, mx), 0));
            if (b00.w != BH_CLASS_WEAK || b10.w != BH_CLASS_WEAK ||
                b01.w != BH_CLASS_WEAK || b11.w != BH_CLASS_WEAK)
                return false;
            float l1 = length(b10.xyz - b00.xyz);
            float l2 = length(b01.xyz - b00.xyz);
            float perPixMin = min(l1, l2) / _BHDownsample;
            float3 twist = (b00.xyz + b11.xyz) - (b10.xyz + b01.xyz);
            float twistMax = BH_UPS_MAX_TWIST * perPixMin;
            if (dot(twist, twist) > twistMax * twistMax) return false;
            dir = normalize(lerp(lerp(b00.xyz, b10.xyz, f.x), lerp(b01.xyz, b11.xyz, f.x), f.y));
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

            float4 FragComposite (Varyings input) : SV_Target
            {
                float3 sceneColor = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_LinearClamp, input.texcoord).rgb;
                if (_BHCount == 0) return float4(sceneColor, 1);

                float3 dir = PixelRayDir(input.texcoord);
                int count = min(_BHCount, 8);
                int camHole = CameraHole(count);

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

                if (isSkyPixel && !visibleBend)
                {
                #if BH_DEBUG_COST
                    return float4(0, 0, 0, 1);
                #endif
                    float Tc;
                    float3 ec = CBDLineEmission(dir, count, Tc);
                    float3 outC = ObserverShift(ec + sceneColor * Tc);
                #if BH_DEBUG_PATH
                    outC = DebugPathTint(0, outC);
                #endif
                    return float4(outC, 1);
                }

                if (!isSkyPixel && sceneEye / max(dot(dir, camFwd0), 1e-4) < tFirst)
                {
                #if BH_DEBUG_COST
                    return float4(0, 0, 0, 1);
                #endif
                    return float4(sceneColor, 1);
                }

                if (tFirst > 1e30)
                {
                #if BH_DEBUG_COST
                    return float4(0, 0, 0.6, 1);
                #endif
                    float Tc;
                    float3 ec = CBDLineEmission(dir, count, Tc);
                    float3 d = 0;
                    bool rec = false;
                    [branch]
                    if (count >= 3 && _BHDownsample > 1.5)
                        rec = ReconstructWeakDir(input.texcoord, d);
                    if (!rec)
                        d = WeakOnlyDir(dir, count);
                    float pixAngleW = 2.0 / (UNITY_MATRIX_P[1][1] * _ScreenParams.y);
                    float3 sky = SampleSky(d, pixAngleW);
                    float3 outC = ObserverShift(ec + sky * Tc);
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
            #pragma multi_compile_fragment _ BH_SINGLE

            struct LowOut
            {
                float4 t0 : SV_Target0;
                float4 t1 : SV_Target1;
                float4 t2 : SV_Target2;
            };

            LowOut FragLow (Varyings input)
            {
                LowOut o;
                o.t0 = float4(0, 0, 0, 1);
                o.t1 = float4(0, 0, 1, 0);
                o.t2 = float4(0, 0, 0, 0);
                if (_BHCount == 0) return o;

                float2 uv = LowTexelUV(input.texcoord);
                float3 dir = PixelRayDir(uv);
                int count = min(_BHCount, 8);
                int camHole = CameraHole(count);
                float tFirst = FirstDomainEntry(dir, count, camHole);
                if (tFirst > 1e30)
                {
                #if defined(BH_SINGLE)
                    // same class as an escaping single-hole trace without disk (traces 1 -> 32, hole 0 -> 512):
                    // cells across the region edge reconstruct instead of going to the full trace
                    o.t1 = float4(WeakOnlyDir(dir, count), 544.0);
                #else
                    if (count >= 3)
                        o.t1 = float4(WeakOnlyDir(dir, count), BH_CLASS_WEAK);
                #endif
                    return o;
                }

                // hidden behind scene geometry before the ray reaches any region: pass 0 shows the scene
                float sceneEye = SceneEyeDepth(uv);
                if (sceneEye < 1e30 && sceneEye / max(dot(dir, -UNITY_MATRIX_V[2].xyz), 1e-4) < tFirst)
                    return o;

                g_bhSkipFirstColor = true;   // first visible crossing is re-shaded at full res
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
            #pragma multi_compile_fragment _ BH_SINGLE

            float4 FragTrace (Varyings input) : SV_Target
            {
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