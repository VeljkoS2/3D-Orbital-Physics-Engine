using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

public class Yoshida : MonoBehaviour
{
    List<GameObject> bodies;
    GameObject oldDom;
    double[] yoshidaC;
    double[] yoshidaD;
    double cbrt2 = math.pow(2.0, 1.0 / 3.0);
    public double dt;
    double maxTimeSimulatedPerFrame;
    public double simulatedTimePerFrameFactor = 500;
    public int maxSubsteps = 1000;
    double smallestOrbitalPeriod = double.MaxValue;

    [Header("Gravitational waves")]
    public bool gravitationalWaves = true;
    [Tooltip("1 = physical. Larger exaggerates the inspiral (sandbox use).")]
    public double gwStrength = 1.0;
    [Tooltip("If the needed substeps exceed maxSubsteps, simulate less time this frame instead of taking steps that are too large (keeps close binaries stable near merger).")]
    public bool keepAccuracy = true;
    [Header("Black hole mergers")]
    public bool blackHoleMergers = true;
    [Tooltip("Gravitational-wave recoil of the remnant (0 for equal masses, up to ~175 km/s near 1:3).")]
    public bool mergerRecoil = false;
    BlackHoleGlobalManager bhManager;

    // 2.5PN gravitational-wave radiation reaction for every pair of black holes (type 3).
    // For stars and planets it is many orders of magnitude below anything visible, so they
    // are skipped. Applied as a velocity kick of length h; split by mass so the total
    // momentum is conserved exactly. Tested (Yoshida-4 + half kicks): inspiral time matches
    // the Peters formula to 0.01%.
    void ApplyRadiationReaction(double h)
    {
        if (!gravitationalWaves || bodies.Count < 2) return;
        double G = Constants.G, c = Constants.C;
        double c5 = c * c * c * c * c;
        for (int i = 0; i < bodies.Count - 1; i++)
        {
            Properties pi = bodies[i].GetComponent<Properties>();
            if (pi.type != 3) continue;
            for (int j = i + 1; j < bodies.Count; j++)
            {
                Properties pj = bodies[j].GetComponent<Properties>();
                if (pj.type != 3) continue;

                double M = pi.mass + pj.mass;
                double eta = pi.mass * pj.mass / (M * M);
                double3 xr = pi.worldPosition - pj.worldPosition;
                double3 vr = pi.velocity - pj.velocity;
                double r = math.length(xr);
                if (r <= 0) continue;
                double3 n = xr / r;
                // same softening as ComputeAllAcc: below the touching distance the 2.5PN term
                // (~1/r^4) would give unphysical kicks
                r = math.max(r, pi.radius + pj.radius);
                double rdot = math.dot(n, vr);
                double v2 = math.lengthsq(vr);
                double GM = G * M;
                double GMr = GM / r;

                double k = gwStrength * 1.6 * eta * GM * GM / (c5 * r * r * r);
                double3 arel = k * ((3.0 * v2 + (17.0 / 3.0) * GMr) * rdot * n - (v2 + 3.0 * GMr) * vr);

                pi.velocity += arel * (pj.mass / M * h);
                pj.velocity -= arel * (pi.mass / M * h);
            }
        }
    }

    // Orbital period of the tightest black-hole pair (their own orbit, independent of
    // dominant bodies: equal masses are never each other's dominant body).
    double BlackHolePairPeriod()
    {
        double best = double.MaxValue;
        for (int i = 0; i < bodies.Count - 1; i++)
        {
            Properties pi = bodies[i].GetComponent<Properties>();
            if (pi.type != 3) continue;
            for (int j = i + 1; j < bodies.Count; j++)
            {
                Properties pj = bodies[j].GetComponent<Properties>();
                if (pj.type != 3) continue;
                double r = math.length(pi.worldPosition - pj.worldPosition);
                double P = 2.0 * math.PI_DBL * math.sqrt(r * r * r / (Constants.G * (pi.mass + pj.mass)));
                if (P < best) best = P;
            }
        }
        return best;
    }
    double CalcGForce(double mass1, double mass2, double distSq)
    {
        return (Constants.G * mass1 * mass2) / distSq;
    }

    double SOI(GameObject body, GameObject parent)
    {
        if (body == null || parent == null) return 0;
        double dist = math.length(body.GetComponent<Properties>().worldPosition - parent.GetComponent<Properties>().worldPosition);
        if (dist == 0) return 0;
        return dist * math.pow(body.GetComponent<Properties>().mass / parent.GetComponent<Properties>().mass, 2.0 / 5.0);
    }

    void DecideDominantBody(GameObject body)
    {
        oldDom = body.GetComponent<Properties>().dominantBody;
        body.GetComponent<Properties>().dominantBody = null;
        double minSOI = double.MaxValue;
        for (int i = 0; i < bodies.Count; i++)
        {
            if (body == bodies[i]) continue;
            double rSOI = 0;
            if (bodies[i].GetComponent<Properties>().dominantBody != null) rSOI = SOI(bodies[i], bodies[i].GetComponent<Properties>().dominantBody);
            if (rSOI <= 0) continue;
            double dist = math.length(body.GetComponent<Properties>().worldPosition - bodies[i].GetComponent<Properties>().worldPosition);
            if (dist < rSOI && rSOI < minSOI && bodies[i].GetComponent<Properties>().mass >= body.GetComponent<Properties>().mass)
            {
                minSOI = rSOI;
                body.GetComponent<Properties>().dominantBody = bodies[i];
                //if (oldDom != body.DominantBody) body.HasIntersection = false;
            }
        }
        if (body.GetComponent<Properties>().dominantBody == null)
        {
            double maxGForce = 0;
            for (int i = 0; i < bodies.Count; i++)
            {
                if (body == bodies[i]) continue;
                double3 r = bodies[i].GetComponent<Properties>().worldPosition - body.GetComponent<Properties>().worldPosition;
                double distSq = math.lengthsq(r);
                double gForce = CalcGForce(body.GetComponent<Properties>().mass, bodies[i].GetComponent<Properties>().mass, distSq);
                if (gForce > maxGForce && bodies[i].GetComponent<Properties>().mass >= body.GetComponent<Properties>().mass)
                {
                    maxGForce = gForce;
                    body.GetComponent<Properties>().dominantBody = bodies[i];
                    //if (oldDom != body.DominantBody) body.HasIntersection = false;
                }
            }
        }
    }

    double3 ComputeAcc(double mass, double distance, double3 position)
    {
        return position * ((Constants.G * mass) / (distance * distance * distance));
    }
    void ComputeAllAcc()
    {
        foreach(GameObject b in bodies)
        {
            Properties p = b.GetComponent<Properties>();
            p.acceleration = double3.zero;
        }

        if (bodies.Count > 1)
        {
            for (int i = 0; i < bodies.Count - 1; i++)
            {
                Properties pi = bodies[i].GetComponent<Properties>();
                for (int j = i + 1; j < bodies.Count; j++)
                {
                    Properties pj = bodies[j].GetComponent<Properties>();
                    double3 r = pj.worldPosition - pi.worldPosition;
                    double dist = math.length(r);
                    if (dist < pi.radius + pj.radius) dist = pi.radius + pj.radius;
                    pi.acceleration += ComputeAcc(pj.mass, dist, r);
                    pj.acceleration -= ComputeAcc(pi.mass, dist, r);
                }
            }
        }
    }
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        bodies = GlobalProperties.bodies;
        double w1 = 1.0 / (2.0 - cbrt2);
        double w0 = -cbrt2 * w1;
        yoshidaC = new double[] { w1 / 2.0, (w0 + w1) / 2.0, (w0 + w1) / 2.0, w1 / 2.0 };
        yoshidaD = new double[] { w1, w0, w1 };
        bhManager = FindAnyObjectByType<BlackHoleGlobalManager>();
    }

    // Update is called once per frame
    void Update()
    {
        smallestOrbitalPeriod = double.MaxValue;
        foreach(var b in bodies)
        {
            Properties p = b.GetComponent<Properties>();
            if(p.dominantBody != null)
                if(p.orbitalParamaters.orbitalPeriod < smallestOrbitalPeriod)
                {
                    smallestOrbitalPeriod = p.orbitalParamaters.orbitalPeriod;
                }
        }
        // black-hole pairs set the step too (300+ steps per orbit as they spiral in)
        smallestOrbitalPeriod = math.min(smallestOrbitalPeriod, BlackHolePairPeriod());
        maxTimeSimulatedPerFrame = smallestOrbitalPeriod / simulatedTimePerFrameFactor;
        dt = Time.deltaTime * GlobalProperties.timeScale;
        int needed = (int)math.ceil(math.abs(dt) / maxTimeSimulatedPerFrame);
        if (keepAccuracy && needed > maxSubsteps)
        {
            // too fast to resolve this frame: simulate less time instead of taking steps that
            // are too large (time slows down near merger instead of the orbit breaking)
            dt = math.sign(dt) * maxSubsteps * maxTimeSimulatedPerFrame;
            needed = maxSubsteps;
        }
        int substeps = math.clamp(needed, 1, maxSubsteps);

        double subDt = dt / substeps;

        for (int i = 0; i < substeps; i++)
        {
            DoYoshida(subDt);
            if (blackHoleMergers)
                BlackHoleMerger.TryMerge(bodies, bhManager, mergerRecoil);
        }
        for (int i = 0; i < bodies.Count; i++)
        {
            DecideDominantBody(bodies[i]);
        }
        for(int i = 0; i < bodies.Count; i++)
        {
            if(bodies[i].GetComponent<Properties>().dominantBody != null)
                bodies[i].GetComponent<Properties>().CalculateOrbitalParamaters(bodies[i].GetComponent<Properties>(), bodies[i].GetComponent<Properties>().dominantBody.GetComponent<Properties>());
        }
    }

    void DoYoshida(double dt)
    {
        ApplyRadiationReaction(0.5 * dt);   // dissipative half kick

        for (int stage = 0; stage < 3; stage++)
        {
            double c = yoshidaC[stage];
            double d = yoshidaD[stage];

            foreach (GameObject b in bodies)
            {
                Properties p = b.GetComponent<Properties>();
                p.worldPosition += p.velocity * (c * dt);
            }

            ComputeAllAcc();

            foreach (GameObject b in bodies)
            {
                Properties p = b.GetComponent<Properties>();
                p.velocity += p.acceleration * (d * dt);
            }
        }
        foreach (GameObject b in bodies)
        {
            Properties p = b.GetComponent<Properties>();
            p.worldPosition += p.velocity * (yoshidaC[3] * dt);
        }

        ApplyRadiationReaction(0.5 * dt);   // dissipative half kick
    }
}
