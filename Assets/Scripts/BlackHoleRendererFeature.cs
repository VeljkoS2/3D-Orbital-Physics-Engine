using System;
using UnityEngine;
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
    }

    class BlackHoleRendererFeaturePass : ScriptableRenderPass
    {
        readonly BlackHoleRendererFeatureSettings settings;

        public BlackHoleRendererFeaturePass(BlackHoleRendererFeatureSettings settings)
        {
            this.settings = settings;
        }

        private class PassData
        {
            public TextureHandle source;
            public Material material;
        }

        static void ExecutePass(PassData data, RasterGraphContext context)
        {
            Blitter.BlitTexture(context.cmd, data.source, new Vector4(1, 1, 0, 0), data.material, 0);
        }

        public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
        {
            if (settings.lensingMaterial == null)
                return;

            const string passName = "Black Hole Lensing";

            UniversalResourceData resourceData = frameData.Get<UniversalResourceData>();

            var srcColor = resourceData.activeColorTexture;

            var destDesc = renderGraph.GetTextureDesc(srcColor);
            destDesc.name = "_BlackHoleLensingTarget";
            destDesc.clearBuffer = false;
            TextureHandle dest = renderGraph.CreateTexture(destDesc);

            using (var builder = renderGraph.AddRasterRenderPass<PassData>(passName, out var passData))
            {
                passData.source = srcColor;
                passData.material = settings.lensingMaterial;

                builder.UseTexture(srcColor);
                builder.UseTexture(resourceData.cameraDepthTexture);
                builder.SetRenderAttachment(dest, 0);

                builder.SetRenderFunc((PassData data, RasterGraphContext context) => ExecutePass(data, context));
            }

            resourceData.cameraColor = dest;
        }
    }
}