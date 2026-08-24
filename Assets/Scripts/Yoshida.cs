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
            if (dist < rSOI && rSOI < minSOI && bodies[i].GetComponent<Properties>().mass > body.GetComponent<Properties>().mass)
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
                if (gForce > maxGForce && bodies[i].GetComponent<Properties>().mass > body.GetComponent<Properties>().mass)
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
        maxTimeSimulatedPerFrame = smallestOrbitalPeriod / simulatedTimePerFrameFactor;
        dt = Time.deltaTime * GlobalProperties.timeScale;
        int substeps = (int)math.ceil(math.abs(dt) / maxTimeSimulatedPerFrame);
        substeps = math.clamp(substeps,1,maxSubsteps);

        double subDt = dt / substeps;

        for(int i = 0; i < substeps; i++)
        { 
            DoYoshida(subDt);
        }
        for(int i = 0; i < bodies.Count; i++)
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
    }
}
