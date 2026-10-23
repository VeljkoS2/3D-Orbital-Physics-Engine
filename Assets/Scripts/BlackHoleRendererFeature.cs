using System;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.Rendering.RenderGraphModule;

public class BlackHoleRendererFeature : ScriptableRendererFeature
{
    [SerializeField] BlackHoleRendererFeatureSettings settings;
    BlackHoleRendererFeaturePass m_ScriptablePass;

    public override void Create()
    {
        m_ScriptablePass = new BlackHoleRendererFeaturePass(settings);
        m_ScriptablePass.renderPassEvent = RenderPassEvent.AfterRenderingSkybox;
        m_ScriptablePass.ConfigureInput(ScriptableRenderPassInput.Color | ScriptableRenderPassInput.Depth);
        m_ScriptablePass.requiresIntermediateTexture = true;
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        renderer.EnqueuePass(m_ScriptablePass);
    }

    [Serializable]
    public class BlackHoleRendererFeatureSettings
    {
        public Material lensingMaterial;

        // Exact geodesics are traced at 1/downsample resolution and the full-resolution
        // image is rebuilt from them; pixels where neighbouring traces disagree (shadow
        // edge, disk edges, photon ring) are still traced at full resolution.
        // 1 = off (every pixel traced), 2 = half resolution, 4 = quarter resolution.
        [Range(1, 4)] public int traceDownsample = 2;

        // Used instead when the lensed region covers a large part of the screen (close
        // to a hole). Edges are still traced at full resolution.
        [Range(1, 4)] public int traceDownsampleClose = 4;
        // Fraction of the screen (rough estimate from the manager) above which the
        // close setting is used.
        [Range(0f, 1f)] public float closeCoverageThreshold = 0.35f;

        // The reduced-resolution traces are never taller than this (pixels): bigger windows
        // automatically use a stronger downsample, so the cost stops growing with window size.
        // 0 = off (the settings above only).
        [Min(0)] public int maxTraceHeight = 600;

        // The Scene view is a second camera: with it open, the whole effect runs twice.
        public bool renderInSceneView = false;
    }

    class BlackHoleRendererFeaturePass : ScriptableRenderPass
    {
        readonly BlackHoleRendererFeatureSettings settings;
        // per camera: is it on the close setting? (the switch has a band: hovering at the threshold, every crossing
        // swapped the downsample -- and with it the disk turbulence's detail and the jets' blur, a visible pop)
        readonly System.Collections.Generic.Dictionary<int, bool> closeState = new System.Collections.Generic.Dictionary<int, bool>();

        static readonly int Low0Id = Shader.PropertyToID("_BHLow0");
        static readonly int Low1Id = Shader.PropertyToID("_BHLow1");
        static readonly int Low2Id = Shader.PropertyToID("_BHLow2");
        static readonly int LowSizeId = Shader.PropertyToID("_BHLowSize");
        static readonly int DownsampleId = Shader.PropertyToID("_BHDownsample");
        static readonly int LensCoverageId = Shader.PropertyToID("_BHLensCoverage");
        static readonly int TraceGridId = Shader.PropertyToID("_BHTraceGrid");
        static readonly int CountId = Shader.PropertyToID("_BHCount");

        // Shader pass indices
        const int CompositePass = 0;   // light: early-outs, weak-only, reconstruction; marks stencil
        const int TraceLowResPass = 1; // reduced-resolution exact traces
        const int FullTracePass = 2;   // exact traces, only where the stencil is still 0

        public BlackHoleRendererFeaturePass(BlackHoleRendererFeatureSettings settings)
        {
            this.settings = settings;
        }

        class TracePassData
        {
            public TextureHandle source;
            public Material material;
            public float downsample;
            public Vector4 lowSize, traceGrid;
        }

        class CompositePassData
        {
            public TextureHandle source;
            public TextureHandle low0, low1, low2;
            public Vector4 lowSize, traceGrid;
            public float downsample;
            public Material material;
        }

        class FullTracePassData
        {
            public TextureHandle source;
            public float downsample;
            public Material material;
        }

        static TextureHandle CreateLowTarget(RenderGraph renderGraph, int w, int h, GraphicsFormat format, string name)
        {
            var desc = new TextureDesc(w, h)
            {
                colorFormat = format,
                depthBufferBits = DepthBits.None,
                msaaSamples = MSAASamples.None,
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp,
                clearBuffer = false,
                name = name
            };
            return renderGraph.CreateTexture(desc);
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            if (settings.lensingMaterial == null)
                return;

            // no black holes this frame: nothing to lens -- skip all three fullscreen passes (and the copies)
            if (Shader.GetGlobalInt(CountId) <= 0)
                return;

            UniversalResourceData resourceData = frameData.Get<UniversalResourceData>();
            UniversalCameraData cameraData = frameData.Get<UniversalCameraData>();

            // reflection probes and inspector previews never need the effect; the
            // Scene view only when asked for
            if (cameraData.cameraType == CameraType.Preview || cameraData.cameraType == CameraType.Reflection)
                return;
            if (cameraData.cameraType == CameraType.SceneView && !settings.renderInSceneView)
                return;

            var srcColor = resourceData.activeColorTexture;

            var destDesc = renderGraph.GetTextureDesc(srcColor);
            destDesc.name = "_BlackHoleLensingTarget";
            destDesc.clearBuffer = false;
            TextureHandle dest = renderGraph.CreateTexture(destDesc);

            var stencilDesc = destDesc;
            stencilDesc.name = "_BlackHoleRouteStencil";
            stencilDesc.colorFormat = GraphicsFormat.None;
            stencilDesc.depthBufferBits = DepthBits.Depth24;
            stencilDesc.clearBuffer = true;
            TextureHandle routeStencil = renderGraph.CreateTexture(stencilDesc);

            // coverage of this camera's screen by lensing, set by BlackHoleGlobalManager
            // in beginCameraRendering (which runs before this, for the same camera)
            float coverage = Shader.GetGlobalFloat(LensCoverageId);
            int camId = cameraData.camera != null ? cameraData.camera.GetInstanceID() : 0;
            closeState.TryGetValue(camId, out bool wasClose);
            bool close = wasClose
                ? coverage > settings.closeCoverageThreshold * 0.85f
                : coverage > settings.closeCoverageThreshold;
            closeState[camId] = close;
            int dsWanted = close ? settings.traceDownsampleClose : settings.traceDownsample;
            int fullW = cameraData.cameraTargetDescriptor.width;
            int fullH = cameraData.cameraTargetDescriptor.height;
            if (settings.maxTraceHeight > 0)
                dsWanted = Mathf.Max(dsWanted, Mathf.CeilToInt(fullH / (float)settings.maxTraceHeight));
            int ds = Mathf.Clamp(dsWanted, 1, 4);
            // the traces sit on the corners of ds x ds pixel blocks (see _BHTraceGrid in the shader): one more
            // than the blocks in each direction, so the last block has its far corner
            int lowW = Mathf.Max(1, (fullW + ds - 1) / ds + 1);
            int lowH = Mathf.Max(1, (fullH + ds - 1) / ds + 1);
            var lowSize = new Vector4(lowW, lowH, 1f / lowW, 1f / lowH);
            var traceGrid = new Vector4(ds / (float)fullW, ds / (float)fullH, fullW / (float)ds, fullH / (float)ds);

            // ---- pass 1: exact traces at reduced resolution (3 targets) -------------
            TextureHandle low0 = TextureHandle.nullHandle, low1 = TextureHandle.nullHandle, low2 = TextureHandle.nullHandle;
            if (ds > 1)
            {
                low0 = CreateLowTarget(renderGraph, lowW, lowH, GraphicsFormat.R16G16B16A16_SFloat, "_BHLow0");
                low1 = CreateLowTarget(renderGraph, lowW, lowH, GraphicsFormat.R32G32B32A32_SFloat, "_BHLow1");
                low2 = CreateLowTarget(renderGraph, lowW, lowH, GraphicsFormat.R32G32B32A32_SFloat, "_BHLow2");

                using (var builder = renderGraph.AddRasterRenderPass<TracePassData>("Black Hole Trace (reduced res)", out var passData))
                {
                    passData.source = srcColor;
                    passData.material = settings.lensingMaterial;
                    passData.downsample = ds;
                    passData.lowSize = lowSize;
                    passData.traceGrid = traceGrid;

                    builder.UseTexture(srcColor);
                    builder.UseTexture(resourceData.cameraDepthTexture);   // (texels hidden behind the scene: skipped)
                    builder.SetRenderAttachment(low0, 0);
                    builder.SetRenderAttachment(low1, 1);
                    builder.SetRenderAttachment(low2, 2);

                    builder.SetRenderFunc((TracePassData data, RasterGraphContext context) =>
                    {
                        data.material.SetFloat(DownsampleId, data.downsample);
                        data.material.SetVector(LowSizeId, data.lowSize);
                        data.material.SetVector(TraceGridId, data.traceGrid);
                        Blitter.BlitTexture(context.cmd, data.source, new Vector4(1, 1, 0, 0), data.material, TraceLowResPass);
                    });
                }
            }

            // ---- pass 0: light composite (writes stencil 1 wherever it finished) -----
            using (var builder = renderGraph.AddRasterRenderPass<CompositePassData>("Black Hole Lensing (composite)", out var passData))
            {
                passData.source = srcColor;
                passData.material = settings.lensingMaterial;
                passData.low0 = low0;
                passData.low1 = low1;
                passData.low2 = low2;
                passData.lowSize = lowSize;
                passData.traceGrid = traceGrid;
                passData.downsample = ds;

                builder.UseTexture(srcColor);
                builder.UseTexture(resourceData.cameraDepthTexture);
                if (ds > 1)
                {
                    builder.UseTexture(low0);
                    builder.UseTexture(low1);
                    builder.UseTexture(low2);
                }
                builder.SetRenderAttachment(dest, 0, AccessFlags.Write);
                builder.SetRenderAttachmentDepth(routeStencil, AccessFlags.Write);

                builder.SetRenderFunc((CompositePassData data, RasterGraphContext context) =>
                {
                    if (data.downsample > 1.5f)
                    {
                        data.material.SetTexture(Low0Id, (RTHandle)data.low0);
                        data.material.SetTexture(Low1Id, (RTHandle)data.low1);
                        data.material.SetTexture(Low2Id, (RTHandle)data.low2);
                        data.material.SetVector(LowSizeId, data.lowSize);
                        data.material.SetVector(TraceGridId, data.traceGrid);
                    }
                    data.material.SetFloat(DownsampleId, data.downsample);
                    Blitter.BlitTexture(context.cmd, data.source, new Vector4(1, 1, 0, 0), data.material, CompositePass);
                });
            }

            // ---- pass 2: full exact trace, only where pass 0 discarded (stencil 0) ---
            using (var builder = renderGraph.AddRasterRenderPass<FullTracePassData>("Black Hole Lensing (full trace)", out var passData))
            {
                passData.source = srcColor;
                passData.material = settings.lensingMaterial;
                passData.downsample = ds;

                builder.UseTexture(srcColor);
                // ReadWrite: keep what pass 0 wrote (Write alone may use a don't-care load)
                builder.SetRenderAttachment(dest, 0, AccessFlags.ReadWrite);
                builder.SetRenderAttachmentDepth(routeStencil, AccessFlags.Read);

                builder.SetRenderFunc((FullTracePassData data, RasterGraphContext context) =>
                {
                    data.material.SetFloat(DownsampleId, data.downsample);
                    Blitter.BlitTexture(context.cmd, data.source, new Vector4(1, 1, 0, 0), data.material, FullTracePass);
                });
            }

            resourceData.cameraColor = dest;
        }
    }
}