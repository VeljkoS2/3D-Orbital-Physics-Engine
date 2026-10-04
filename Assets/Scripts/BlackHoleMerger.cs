using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

// Black hole mergers: when two black holes touch, the physics replaces them by one remnant.
// The removed hole stays alive as a render-only "ghost" while BlackHoleGlobalManager plays
// the plunge (the two spiral together on screen), then it is destroyed.
// Fits (checked against numerical-relativity values):
//   radiated mass  E/M = 0.0560 eta + 0.5810 eta^2 - 0.9607 eta^3 + 3.3524 eta^4
//   final spin     Rezzolla et al. 2008, vector form, any spins
public static class BlackHoleMerger
{
    const double S4 = -0.1229, S5 = 0.4537, T0 = -2.8904, T2 = -3.5171, T3 = 2.5763;

    public static double RadiatedFraction(double eta)
    {
        return 0.0559745 * eta + 0.580951 * eta * eta - 0.960673 * eta * eta * eta + 3.35241 * eta * eta * eta * eta;
    }

    public static double3 FinalSpin(double m1, double m2, double3 a1, double3 a2, double3 lHat)
    {
        if (m2 > m1) { (m1, m2) = (m2, m1); (a1, a2) = (a2, a1); }
        double q = m2 / m1;
        double q2 = q * q;
        double eta = q / ((1 + q) * (1 + q));
        double A1 = math.length(a1), A2 = math.length(a2);
        double cosA = (A1 * A2 > 0) ? math.dot(a1, a2) / (A1 * A2) : 1.0;
        double cosB = (A1 > 0) ? math.dot(a1, lHat) / A1 : 1.0;
        double cosG = (A2 > 0) ? math.dot(a2, lHat) / A2 : 1.0;
        double l = S4 / ((1 + q2) * (1 + q2)) * (A1 * A1 + A2 * A2 * q2 * q2 + 2 * A1 * A2 * q2 * cosA)
                 + (S5 * eta + T0 + 2) / (1 + q2) * (A1 * cosB + A2 * q2 * cosG)
                 + 2 * math.sqrt(3.0) + T2 * eta + T3 * eta * eta;
        return (a1 + a2 * q2 + l * q * lHat) / ((1 + q) * (1 + q));
    }

    public static bool TryMerge(List<GameObject> bodies, BlackHoleGlobalManager manager, bool recoil)
    {
        for (int i = 0; i < bodies.Count - 1; i++)
        {
            Properties pi = bodies[i].GetComponent<Properties>();
            if (pi.type != 3) continue;
            for (int j = i + 1; j < bodies.Count; j++)
            {
                Properties pj = bodies[j].GetComponent<Properties>();
                if (pj.type != 3) continue;
                double r = math.length(pj.worldPosition - pi.worldPosition);
                if (r > pi.radius + pj.radius) continue;

                bool iKeeps = pi.mass >= pj.mass;
                Merge(iKeeps ? bodies[i] : bodies[j], iKeeps ? bodies[j] : bodies[i], bodies, manager, recoil);
                return true;
            }
        }
        return false;
    }

    static double3 D3(Vector3 v) => new double3(v.x, v.y, v.z);
    static Vector3 V3(double3 v) => new Vector3((float)v.x, (float)v.y, (float)v.z);

    static void Merge(GameObject keep, GameObject gone, List<GameObject> bodies,
                      BlackHoleGlobalManager manager, bool recoil)
    {
        Properties A = keep.GetComponent<Properties>();
        Properties B = gone.GetComponent<Properties>();
        BlackHoleParamaters bhA = A.blackHoleParamaters;
        BlackHoleParamaters bhB = B.blackHoleParamaters;

        double m1 = A.mass, m2 = B.mass, M = m1 + m2;
        double eta = m1 * m2 / (M * M);

        double3 rel = B.worldPosition - A.worldPosition;
        double3 vrel = B.velocity - A.velocity;
        double3 L = math.cross(rel, vrel);
        Vector3 upA = keep.transform.up, upB = gone.transform.up;
        double3 lHat = math.lengthsq(L) > 0 ? math.normalize(L)
                     : math.normalize(D3(upA) * m1 + D3(upB) * m2);

        double3 af = FinalSpin(m1, m2, D3(upA) * bhA.spin, D3(upB) * bhB.spin, lHat);
        double afMag = math.length(af);
        double3 axis = afMag > 1e-3 ? af / afMag : lHat;

        double frac = RadiatedFraction(eta);
        double Mf = M * (1.0 - frac);

        double3 xcom = (A.worldPosition * m1 + B.worldPosition * m2) / M;
        double3 vcom = (A.velocity * m1 + B.velocity * m2) / M;
        if (recoil)
        {
            double vk = 1.2e7 * eta * eta * math.sqrt(math.max(1 - 4 * eta, 0)) * (1 - 0.93 * eta);
            double3 dir = math.cross(lHat, rel);
            if (math.lengthsq(dir) > 0) vcom += math.normalize(dir) * vk;
        }

        // where the two were, relative to the centre of mass (Unity units): the plunge starts here
        Vector3 offKeep = V3((A.worldPosition - xcom) / GlobalProperties.scale);
        Vector3 offGone = V3((B.worldPosition - xcom) / GlobalProperties.scale);

        // ---- the remnant (physics) ---------------------------------------------------
        // The spin axis on the orbit's side (at most 90 deg from it): when the spin points
        // the other way it is stored as a negative spin along that axis. Same physical spin
        // vector, but the disk then always orbits the same way the shared disk did, and
        // tilting from the orbit's plane to the spin's plane never passes through edge-on.
        bool spinWithOrbit = math.dot(axis, lHat) >= 0;
        double3 axisDisk = spinWithOrbit ? axis : -axis;
        float spinSigned = (float)(math.min(afMag, 0.998) * (spinWithOrbit ? 1.0 : -1.0));

        BlackHoleParamaters bp = bhA;
        bp.mass = Mf;
        bp.schwarzschildRadiusWorld = 2.0 * Constants.G * Mf / (Constants.C * Constants.C);
        bp.schwarzschildRadiusUnits = (float)(bp.schwarzschildRadiusWorld / GlobalProperties.scale);
        bp.spin = spinSigned;
        float rotMag = Mathf.Abs(bhA.diskRotation) > 0f ? Mathf.Abs(bhA.diskRotation) : 1f;
        bp.diskRotation = rotMag;   // prograde about axisDisk = the shared disk's orbit sense
        bp.accretionDisk = bhA.accretionDisk || bhB.accretionDisk;

        A.mass = Mf;
        A.blackHoleParamaters = bp;
        A.blackHoleAppearanceProperties = Blend(A.blackHoleAppearanceProperties, B.blackHoleAppearanceProperties, (float)(m1 / M));
        A.radius = bp.schwarzschildRadiusWorld * 1.1;
        A.worldPosition = xcom;
        A.velocity = vcom;
        keep.transform.rotation = Quaternion.FromToRotation(Vector3.up, V3(axisDisk));

        // ---- hand references to the remnant; the removed hole leaves the physics -------
        foreach (GameObject go in bodies)
        {
            Properties p = go.GetComponent<Properties>();
            if (p.dominantBody == gone) p.dominantBody = (go == keep) ? null : keep;
        }
        if (GlobalProperties.focus == gone) GlobalProperties.focus = keep;
        if (GlobalProperties.closestBody == gone) GlobalProperties.closestBody = keep;

        bodies.Remove(gone);
        if (GlobalProperties.crosshairs.TryGetValue(gone, out var img))
        {
            if (img != null) Object.Destroy(img.gameObject);
            GlobalProperties.crosshairs.Remove(gone);
        }
        GlobalProperties.orbitRenderers.Remove(gone);
        gone.tag = "Untagged";   // GlobalProperties must not pick it up again

        if (manager != null)
        {
            // render-only ghost: stays in GlobalProperties.blackHoles until the plunge ends
            manager.OnMerger(keep, gone, offKeep, offGone, V3(lHat),
                             bhA.schwarzschildRadiusUnits, bhB.schwarzschildRadiusUnits,
                             bhA.spin, bhB.spin, upA, upB,
                             (float)(1.0 - frac), spinSigned, V3(axisDisk));
        }
        else
        {
            GlobalProperties.blackHoles.Remove(gone);
            gone.SetActive(false);
            Object.Destroy(gone);
        }
    }

    static BlackHoleAppearanceProperties Blend(BlackHoleAppearanceProperties a, BlackHoleAppearanceProperties b, float wa)
    {
        float wb = 1f - wa;
        BlackHoleAppearanceProperties o = a;
        o.diskPeakTemperature = wa * a.diskPeakTemperature + wb * b.diskPeakTemperature;
        o.diskDopplerColor = wa * a.diskDopplerColor + wb * b.diskDopplerColor;
        o.diskDopplerBeaming = wa * a.diskDopplerBeaming + wb * b.diskDopplerBeaming;
        o.diskRadialContrast = wa * a.diskRadialContrast + wb * b.diskRadialContrast;
        o.diskBrightness = wa * a.diskBrightness + wb * b.diskBrightness;
        o.diskISCOStress = wa * a.diskISCOStress + wb * b.diskISCOStress;
        o.diskTurbulance = wa * a.diskTurbulance + wb * b.diskTurbulance;
        o.diskHigherOrderFade = wa * a.diskHigherOrderFade + wb * b.diskHigherOrderFade;
        o.hotSpotCount = Mathf.RoundToInt(wa * a.hotSpotCount + wb * b.hotSpotCount);
        o.hotSpotStrength = wa * a.hotSpotStrength + wb * b.hotSpotStrength;
        o.hotSpotRMin = wa * a.hotSpotRMin + wb * b.hotSpotRMin;
        o.hotSpotRMax = wa * a.hotSpotRMax + wb * b.hotSpotRMax;
        o.eddingtonRatio = wa * a.eddingtonRatio + wb * b.eddingtonRatio;
        o.diskTint = Color.Lerp(b.diskTint, a.diskTint, wa);
        o.diskTintStrength = wa * a.diskTintStrength + wb * b.diskTintStrength;
        return o;
    }
}