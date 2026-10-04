using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public class Yoshida : MonoBehaviour
{
    List<GameObject> bodies;
    double[] yoshidaC;
    double[] yoshidaD;
    readonly double cbrt2 = math.pow(2.0, 1.0 / 3.0);
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

    // ---- working copy of the bodies (struct of arrays), filled once per frame --------
    int n;
    int capacity;
    Properties[] props = new Properties[0];
    double[] soi = new double[0];
    NativeArray<double3> pos, vel, acc;
    NativeArray<double> mass, rad;
    NativeArray<byte> isBH;
    NativeArray<int> result;   // [0] substeps done, [1] 1 if two black holes touch

    void Start()
    {
        bodies = GlobalProperties.bodies;
        double w1 = 1.0 / (2.0 - cbrt2);
        double w0 = -cbrt2 * w1;
        yoshidaC = new double[] { w1 / 2.0, (w0 + w1) / 2.0, (w0 + w1) / 2.0, w1 / 2.0 };
        yoshidaD = new double[] { w1, w0, w1 };
        bhManager = FindAnyObjectByType<BlackHoleGlobalManager>();
        result = new NativeArray<int>(2, Allocator.Persistent);
    }

    void OnDestroy()
    {
        DisposeArrays();
        if (result.IsCreated) result.Dispose();
    }

    void DisposeArrays()
    {
        if (pos.IsCreated) pos.Dispose();
        if (vel.IsCreated) vel.Dispose();
        if (acc.IsCreated) acc.Dispose();
        if (mass.IsCreated) mass.Dispose();
        if (rad.IsCreated) rad.Dispose();
        if (isBH.IsCreated) isBH.Dispose();
    }

    void EnsureCapacity(int count)
    {
        if (count <= capacity && pos.IsCreated) return;
        DisposeArrays();
        capacity = math.max(count, math.max(8, capacity * 2));
        pos = new NativeArray<double3>(capacity, Allocator.Persistent);
        vel = new NativeArray<double3>(capacity, Allocator.Persistent);
        acc = new NativeArray<double3>(capacity, Allocator.Persistent);
        mass = new NativeArray<double>(capacity, Allocator.Persistent);
        rad = new NativeArray<double>(capacity, Allocator.Persistent);
        isBH = new NativeArray<byte>(capacity, Allocator.Persistent);
        props = new Properties[capacity];
        soi = new double[capacity];
    }

    void Gather()
    {
        n = bodies.Count;
        EnsureCapacity(n);
        for (int i = 0; i < n; i++)
        {
            Properties p = bodies[i].GetComponent<Properties>();
            props[i] = p;
            pos[i] = p.worldPosition;
            vel[i] = p.velocity;
            acc[i] = p.acceleration;
            mass[i] = p.mass;
            rad[i] = p.radius;
            isBH[i] = (byte)(p.type == 3 ? 1 : 0);
        }
    }

    void Scatter()
    {
        for (int i = 0; i < n; i++)
        {
            props[i].worldPosition = pos[i];
            props[i].velocity = vel[i];
            props[i].acceleration = acc[i];
        }
    }

    // Orbital period of the tightest black-hole pair (their own orbit, independent of
    // dominant bodies: equal masses are never each other's dominant body).
    double BlackHolePairPeriod()
    {
        double best = double.MaxValue;
        for (int i = 0; i < n - 1; i++)
        {
            if (isBH[i] == 0) continue;
            for (int j = i + 1; j < n; j++)
            {
                if (isBH[j] == 0) continue;
                double r = math.length(pos[i] - pos[j]);
                double P = 2.0 * math.PI_DBL * math.sqrt(r * r * r / (Constants.G * (mass[i] + mass[j])));
                if (P < best) best = P;
            }
        }
        return best;
    }

    YoshidaSubstepJob MakeJob(int steps, double h) => new YoshidaSubstepJob
    {
        pos = pos,
        vel = vel,
        acc = acc,
        mass = mass,
        rad = rad,
        isBH = isBH,
        result = result,
        n = n,
        steps = steps,
        h = h,
        G = Constants.G,
        c = Constants.C,
        gwStrength = gwStrength,
        gw = gravitationalWaves,
        checkMergers = blackHoleMergers,
        c0 = yoshidaC[0],
        c1 = yoshidaC[1],
        c2 = yoshidaC[2],
        c3 = yoshidaC[3],
        d0 = yoshidaD[0],
        d1 = yoshidaD[1],
        d2 = yoshidaD[2]
    };

    void Update()
    {
        Gather();

        smallestOrbitalPeriod = double.MaxValue;
        for (int i = 0; i < n; i++)
        {
            Properties p = props[i];
            if (p.dominantBody != null && p.orbitalParamaters.orbitalPeriod < smallestOrbitalPeriod)
                smallestOrbitalPeriod = p.orbitalParamaters.orbitalPeriod;
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

        // all substeps in Burst; it stops early only when two black holes touch
        int done = 0;
        while (done < substeps && n > 0)
        {
            MakeJob(substeps - done, subDt).Run();
            done += math.max(result[0], 1);
            if (result[1] != 0)
            {
                Scatter();
                BlackHoleMerger.TryMerge(bodies, bhManager, mergerRecoil);
                Gather();
            }
        }
        Scatter();

        DecideDominantBodies();
        for (int i = 0; i < n; i++)
        {
            GameObject dom = props[i].dominantBody;
            if (dom != null)
                props[i].CalculateOrbitalParamaters(props[i], dom.GetComponent<Properties>());
        }
    }

    // ---- dominant bodies --------------------------------------------------------------
    // soi[i] = sphere of influence of body i about its current dominant body. It only changes
    // when body i's dominant body changes, so it is updated right after each decision:
    // identical to recomputing it inside the loop (same sequential order as before).
    double SOI(int i)
    {
        GameObject parent = props[i].dominantBody;
        if (parent == null) return 0;
        Properties pp = parent.GetComponent<Properties>();
        double dist = math.length(pos[i] - pp.worldPosition);
        if (dist == 0) return 0;
        return dist * math.pow(mass[i] / pp.mass, 2.0 / 5.0);
    }

    void DecideDominantBodies()
    {
        for (int i = 0; i < n; i++) soi[i] = SOI(i);
        for (int b = 0; b < n; b++)
        {
            DecideDominantBody(b);
            soi[b] = SOI(b);
        }
    }

    void DecideDominantBody(int b)
    {
        Properties pb = props[b];
        pb.dominantBody = null;
        double minSOI = double.MaxValue;
        for (int i = 0; i < n; i++)
        {
            if (i == b) continue;
            double rSOI = soi[i];
            if (rSOI <= 0) continue;
            double dist = math.length(pos[b] - pos[i]);
            if (dist < rSOI && rSOI < minSOI && mass[i] >= mass[b])
            {
                minSOI = rSOI;
                pb.dominantBody = bodies[i];
            }
        }
        if (pb.dominantBody == null)
        {
            double maxGForce = 0;
            for (int i = 0; i < n; i++)
            {
                if (i == b) continue;
                double distSq = math.lengthsq(pos[i] - pos[b]);
                double gForce = (Constants.G * mass[b] * mass[i]) / distSq;
                if (gForce > maxGForce && mass[i] >= mass[b])
                {
                    maxGForce = gForce;
                    pb.dominantBody = bodies[i];
                }
            }
        }
    }
}

// Yoshida-4 substeps with 2.5PN radiation-reaction half kicks, Burst-compiled.
[BurstCompile]
struct YoshidaSubstepJob : IJob
{
    public NativeArray<double3> pos, vel, acc;
    [ReadOnly] public NativeArray<double> mass, rad;
    [ReadOnly] public NativeArray<byte> isBH;
    public NativeArray<int> result;
    public int n, steps;
    public double h, G, c, gwStrength;
    public bool gw, checkMergers;
    public double c0, c1, c2, c3, d0, d1, d2;

    public void Execute()
    {
        result[0] = 0;
        result[1] = 0;
        for (int s = 0; s < steps; s++)
        {
            RadiationReaction(0.5 * h);   // dissipative half kick
            Drift(c0 * h); Accelerations(); Kick(d0 * h);
            Drift(c1 * h); Accelerations(); Kick(d1 * h);
            Drift(c2 * h); Accelerations(); Kick(d2 * h);
            Drift(c3 * h);
            RadiationReaction(0.5 * h);   // dissipative half kick
            result[0] = s + 1;
            if (checkMergers && BlackHolesTouch()) { result[1] = 1; return; }
        }
    }

    void Drift(double k) { for (int i = 0; i < n; i++) pos[i] += vel[i] * k; }
    void Kick(double k) { for (int i = 0; i < n; i++) vel[i] += acc[i] * k; }

    void Accelerations()
    {
        for (int i = 0; i < n; i++) acc[i] = double3.zero;
        for (int i = 0; i < n - 1; i++)
        {
            double3 pi = pos[i];
            double ri = rad[i], mi = mass[i];
            double3 ai = acc[i];
            for (int j = i + 1; j < n; j++)
            {
                double3 r = pos[j] - pi;
                double dist = math.length(r);
                double rr = ri + rad[j];
                if (dist < rr) dist = rr;
                double d3 = dist * dist * dist;
                ai += r * ((G * mass[j]) / d3);
                acc[j] -= r * ((G * mi) / d3);
            }
            acc[i] = ai;
        }
    }

    void RadiationReaction(double hh)
    {
        if (!gw || n < 2) return;
        double c5 = c * c * c * c * c;
        for (int i = 0; i < n - 1; i++)
        {
            if (isBH[i] == 0) continue;
            for (int j = i + 1; j < n; j++)
            {
                if (isBH[j] == 0) continue;
                double M = mass[i] + mass[j];
                double eta = mass[i] * mass[j] / (M * M);
                double3 xr = pos[i] - pos[j];
                double3 vr = vel[i] - vel[j];
                double r = math.length(xr);
                if (r <= 0) continue;
                double3 nn = xr / r;
                r = math.max(r, rad[i] + rad[j]);
                double rdot = math.dot(nn, vr);
                double v2 = math.lengthsq(vr);
                double GM = G * M;
                double GMr = GM / r;
                double k = gwStrength * 1.6 * eta * GM * GM / (c5 * r * r * r);
                double3 arel = k * ((3.0 * v2 + (17.0 / 3.0) * GMr) * rdot * nn - (v2 + 3.0 * GMr) * vr);
                vel[i] += arel * (mass[j] / M * hh);
                vel[j] -= arel * (mass[i] / M * hh);
            }
        }
    }

    bool BlackHolesTouch()   // same condition as BlackHoleMerger.TryMerge
    {
        for (int i = 0; i < n - 1; i++)
        {
            if (isBH[i] == 0) continue;
            for (int j = i + 1; j < n; j++)
            {
                if (isBH[j] == 0) continue;
                if (!(math.length(pos[j] - pos[i]) > rad[i] + rad[j])) return true;
            }
        }
        return false;
    }
}