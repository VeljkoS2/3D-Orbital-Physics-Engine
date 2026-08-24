using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class LightManager : MonoBehaviour
{
    List<Vector4> positions = new List<Vector4>();
    List<Vector4> colors = new List<Vector4>();
    List<float> luminosities = new List<float>();
    public bool realLight = true;

    public int maxShadowedAdditionalStars = 8;

    void Update()
    {
        positions.Clear();
        colors.Clear();
        luminosities.Clear();

        if (GlobalProperties.closestStar != null)
        {

            GameObject dominant = GlobalProperties.closestStar;
            RenderSettings.sun = dominant.GetComponent<Light>();

            Vector3 focusPos = GlobalProperties.closestBody.transform.position;

            // Pick the next-most-relevant stars (by apparent brightness from the focused body)
            // to also get real shadow-casting lights, capped to the shadow budget.
            var shadowCandidates = new HashSet<GameObject>(
                GlobalProperties.stars
                    .Select(s => s.gameObject)
                    .Where(g => g != dominant)
                    .OrderByDescending(g =>
                    {
                        Properties p = g.GetComponent<Properties>();
                        float sqrDist = Mathf.Max(1f, (g.transform.position - focusPos).sqrMagnitude);
                        return (float)p.luminosity / sqrDist;
                    })
                    .Take(maxShadowedAdditionalStars)
            );

            foreach (var s in GlobalProperties.stars)
            {
                GameObject starObj = s.gameObject;
                Light starLight = starObj.GetComponent<Light>();
                Properties starProperties = starObj.GetComponent<Properties>();
                starProperties.CalculateLight();

                positions.Add(s.position);
                colors.Add(starProperties.color);
                luminosities.Add((float)starProperties.luminosity);

                bool isDominant = starObj == dominant;
                bool isShadowedAdditional = shadowCandidates.Contains(starObj);

                starLight.shadows = (isDominant || isShadowedAdditional) ? LightShadows.Soft : LightShadows.None;
                s.LookAt(focusPos);
            }

            Shader.SetGlobalVectorArray("_StarPositions", positions.ToArray());
            Shader.SetGlobalVectorArray("_StarColors", colors.ToArray());
            Shader.SetGlobalFloatArray("_StarLuminosities", luminosities.ToArray());
            Shader.SetGlobalFloat("_MetersToUnitsScale", (float)GlobalProperties.scale);
            Shader.SetGlobalInt("_StarCount", GlobalProperties.stars.Count);
            Shader.SetGlobalInt("_RealLight", realLight ? 1 : 0);
        }
    }
}