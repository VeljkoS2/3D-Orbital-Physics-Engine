using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.RenderGraphModule.Util;
using UnityEngine.Rendering.Universal;

class TransparentDepthCapturePass : ScriptableRenderPass
{
    LayerMask layerMask;
    Material depthOnlyMaterial;

    public TransparentDepthCapturePass(LayerMask layerMask, Material depthOnlyMaterial)
    {
        this.layerMask = layerMask;
        this.depthOnlyMaterial = depthOnlyMaterial;
    }

    class PassData { public RendererListHandle rendererList; }

    public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
    {
        if (depthOnlyMaterial == null) return;

        UniversalResourceData resourceData = frameData.Get<UniversalResourceData>();
        UniversalCameraData cameraData = frameData.Get<UniversalCameraData>();
        UniversalRenderingData renderingData = frameData.Get<UniversalRenderingData>();
        UniversalLightData lightData = frameData.Get<UniversalLightData>();

        var desc = cameraData.cameraTargetDescriptor;
        desc.graphicsFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.None;
        desc.depthStencilFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.D32_SFloat;
        desc.msaaSamples = 1;
        TextureHandle transparentDepth = renderGraph.CreateTexture(
            new TextureDesc(desc) { name = "_TransparentLensDepth", clearBuffer = true, depthBufferBits = DepthBits.Depth32 });

        using (var builder = renderGraph.AddRasterRenderPass<PassData>("Transparent Depth Capture", out var passData))
        {
            var drawSettings = RenderingUtils.CreateDrawingSettings(
                new ShaderTagId("UniversalForward"), renderingData, cameraData, lightData, SortingCriteria.CommonTransparent);
            drawSettings.overrideMaterial = depthOnlyMaterial;

            var filterSettings = new FilteringSettings(RenderQueueRange.transparent, layerMask);
            var param = new RendererListParams(renderingData.cullResults, drawSettings, filterSettings);
            passData.rendererList = renderGraph.CreateRendererList(param);

            builder.UseRendererList(passData.rendererList);
            builder.SetRenderAttachmentDepth(transparentDepth, AccessFlags.Write);
            builder.AllowPassCulling(false);

            builder.SetRenderFunc((PassData data, RasterGraphContext context) =>
            {
                context.cmd.DrawRendererList(data.rendererList);
            });
        }
    }
}