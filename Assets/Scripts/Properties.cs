using System;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class Properties : MonoBehaviour
{
    public double mass = 1;
    public bool hasAtmosphere = false;
    public bool hasRings = false;
    public double3 worldPosition = double3.zero;
    public double3 acceleration = double3.zero;
    public double3 velocity = double3.zero;
    public double radius = 0;
    double density = 5000;
    public int type { get; set; } = 0;
    public double rotationalPeriod = 86400;
    public double rotationalSpeedDeg { get; set; } = 360 / 86400.0;
    public Vector3 startingRotation = Vector3.up;
    public GameObject dominantBody = null;
    public OrbitalParamaters orbitalParamaters;
    // Star Properties
    public Color color { get; set; }
    public double luminosity { get; set; } = 0;
    public double2 ring = new double2();

    public BlackHoleParamaters blackHoleParamaters;
    public BlackHoleAppearanceProperties blackHoleAppearanceProperties;
    GameObject gameManager;
    public Material starMaterial;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        gameManager = GameObject.FindGameObjectWithTag("GameManager");
        CalculateRadius();
        if (type == 2 /*|| type == 3*/)
        {
            gameObject.GetComponent<Renderer>().material = starMaterial;
            CalculateLight();
            if (!GlobalProperties.stars.Contains(transform))
            {
                GlobalProperties.stars.Add(gameObject.GetComponentInChildren<Transform>());
                if (GlobalProperties.closestStar == null) GlobalProperties.closestStar = gameObject.GetComponentInChildren<Light>().gameObject;
                Light starLight = gameObject.GetComponentInChildren<Light>();
                starLight.type = LightType.Directional;
                starLight.intensity = 0.001f;
                starLight.shadows = LightShadows.Soft;
            }
        }
        if (type == 3)
        {
            gameObject.GetComponent<MeshRenderer>().enabled = false;
            if(!GlobalProperties.blackHoles.Contains(gameObject) && !gameManager.GetComponent<SphericalCoordinateSystem>().placing)
            {
                GlobalProperties.blackHoles.Add(gameObject);
            }
        }
        transform.rotation = Quaternion.Euler(startingRotation);
        rotationalSpeedDeg = 360 / rotationalPeriod;
        if(GlobalProperties.bodies.Count == 1 && !GlobalProperties.setFirstFocus)
        {
            GlobalProperties.focus = GlobalProperties.bodies[0];
            GlobalProperties.focused = true;
            gameManager.GetComponent<ApplyOffset>().distanceFromFocus = GlobalProperties.focus.GetComponent<Properties>().radius * 10;
            GlobalProperties.offset = Camera.main.GetComponent<Move>().direction * gameManager.GetComponent<ApplyOffset>().distanceFromFocus - GlobalProperties.focus.GetComponent<Properties>().worldPosition;
            GlobalProperties.setFirstFocus = true;
        }
    }

    // Update is called once per frame
    void Update()
    {
        Rotate(transform, rotationalSpeedDeg);
    }

    public void Rotate(Transform transform, double rotationalSpeedDeg)
    {
        if(rotationalPeriod != 0)
            transform.Rotate(0, -(float)(rotationalSpeedDeg * Time.deltaTime * GlobalProperties.timeScale), 0);
    }

    public void CalculateRadius()
    {
        if (mass == 0) return;
        if (mass > 1e31)
        {
            CalculateBlackHoleParamaters(this);
            AssignBlackHoleAppearanceProperties(this);
            radius = blackHoleParamaters.schwarzschildRadiusWorld;
            blackHoleParamaters.diskInnerRadiusNormalized = 2.0f;
            var diskRnd = new Unity.Mathematics.Random((uint)System.DateTime.Now.Ticks * 347796405u | 1u);
            blackHoleParamaters.diskOuterRadiusNormalized = math.abs(diskRnd.NextFloat(-12.0f, 50.0f));//rnd.NextFloat(clamped * 0.8f, clamped * 1.2f);
            blackHoleParamaters.escapeRadiusNormalized = blackHoleParamaters.diskOuterRadiusNormalized * 2.0f;
            var spinRnd = new Unity.Mathematics.Random((uint)System.DateTime.Now.Ticks * 747796405u | 1u); // seed must be non-zero
            blackHoleParamaters.spin = spinRnd.NextFloat(-0.998f, 0.998f);
            blackHoleParamaters.diskRotation = math.sign(blackHoleParamaters.spin) * 1.0f;
            blackHoleParamaters.accretionDisk = true;
            //SetShaderPropperties(blackHoleParamaters);
            //gameObject.AddComponent<BlackHoleLod>();
            type = 3;
            //double visualInfluenceRadius = blackHoleParamaters.schwarzschildRadiusUnits*2.6;
            transform.localScale = new Vector3((float)(radius/GlobalProperties.scale) * 2f, (float)(radius / GlobalProperties.scale) * 2f, (float)(radius / GlobalProperties.scale) * 2f);
        }
        else if (mass > 2e29)
        {
            radius = Constants.SolarRadius * math.pow(mass / Constants.SolarMass, 0.8);
            type = 2;
            transform.localScale = new Vector3((float)(radius / GlobalProperties.scale) * 2f, (float)(radius / GlobalProperties.scale) * 2f, (float)(radius / GlobalProperties.scale) * 2f);
        }
        else
        {
            density = 5000;
            if (mass > 5e25) density = 1300;
            radius = math.pow((3 * mass) / (4 * math.PI * density), 1.0 / 3.0);
            type = 1;
            transform.localScale = new Vector3((float)(radius / GlobalProperties.scale) * 2f, (float)(radius / GlobalProperties.scale) * 2f, (float)(radius / GlobalProperties.scale) * 2f);
        }
    }

    public void CalculateBlackHoleISCO(Properties blackHole)
    {
        double a = blackHole.blackHoleParamaters.spin;
        double mass = blackHoleParamaters.mass;

        double Z1 = 1 + Math.Pow((1 - a * a), 1 / 3) * (Math.Pow((1 + a), 1 / 3) + Math.Pow((1 - a), 1 / 3));
        double Z2 = Math.Sqrt(3 * a * a + Z1 * Z1);

        blackHole.blackHoleParamaters.diskInnerRadiusNormalized = (Math.Sign(blackHole.blackHoleParamaters.diskRotation) == Math.Sign(a)) ? (float)(3 + Z2 - Math.Sqrt((3 - Z1) * (3 + Z1 + 2 * Z2)) * mass) : (float)(3 + Z2 + Math.Sqrt((3 - Z1) * (3 + Z1 + 2 * Z2)) * mass);
    }

    public void CalculateLight()
    {
        Renderer renderer = gameObject.GetComponent<Renderer>();
        Material material = renderer.material;

        int starColor = Shader.PropertyToID("_MainColor");

        double massSolar = mass / Constants.SolarMass;
        luminosity = math.pow(massSolar, 3.5);
        double temperature = 5778 * math.pow(massSolar, 0.505);

        if (temperature < 3700) color = new Color(1.000f, 0.745f, 0.604f);      // M - Reddish-orange
        else if (temperature < 5200) color = new Color(1.000f, 0.843f, 0.725f); // K - Pale orange
        else if (temperature < 6000) color = new Color(1.000f, 0.961f, 0.941f); // G - Yellowish white (Sun)
        else if (temperature < 7500) color = new Color(0.953f, 0.969f, 1.000f); // F - Pure white
        else if (temperature < 10000) color = new Color(0.792f, 0.859f, 1.000f);// A - Light blue-white
        else if (temperature < 30000) color = new Color(0.635f, 0.749f, 1.000f);// B - Soft deep blue
        else color = new Color(0.573f, 0.690f, 1.000f);

        material.SetColor(starColor, color);
        gameObject.GetComponent<Renderer>().material = material;
    }
    public void CalculateBlackHoleParamaters(Properties blackHole)
    {
        BlackHoleParamaters bp = new BlackHoleParamaters();

        bp.mass = blackHole.mass;
        bp.schwarzschildRadiusWorld = (2 * Constants.G * blackHole.mass) / (Constants.C * Constants.C);
        bp.schwarzschildRadiusUnits = (float)(bp.schwarzschildRadiusWorld / GlobalProperties.scale);
        bp.blackHolePositionUnits = blackHole.gameObject.transform.position;

        blackHole.blackHoleParamaters = bp;
    }

    public void AssignBlackHoleAppearanceProperties(Properties blackHole)
    {
        BlackHoleAppearanceProperties bp = new BlackHoleAppearanceProperties();

        bp.diskPeakTemperature = 6000;
        bp.diskBrightness = 1.0f;
        bp.diskDopplerBeaming = 1.0f;
        bp.diskDopplerColor = 1.0f;
        bp.diskHigherOrderFade = 0.0f;
        bp.diskISCOStress = 0.03f;
        bp.diskRadialContrast = 1.0f;
        bp.diskVisibleLight = 1.0f;   // 1 = brightness as the eye sees it (visible light) instead of total power
        bp.diskTurbulance = 0.75f;
        bp.hotSpotCount = 3;
        bp.hotSpotStrength = 1.0f;
        bp.hotSpotRMin = 1.08f;  
        bp.hotSpotRMax = 2.7f;
        bp.physicalDiskTemperature = true;
        bp.eddingtonRatio = 0.1f;
        bp.diskTint = new Color(1.0f, 1.0f, 1.0f);
        bp.diskTintStrength = 0.0f;

        blackHole.blackHoleAppearanceProperties = bp;
    }

    public void CalculateOrbitalParamaters(Properties body, Properties dominantBody)
    {
        OrbitalParamaters orbPar = new OrbitalParamaters();

        double standardGravitationalParamater = Constants.G * (body.mass + dominantBody.mass);

        double3 relativeVelocity = body.velocity - dominantBody.velocity;
        double relativeSpeed = math.length(relativeVelocity);

        double3 distanceVector = body.worldPosition - dominantBody.worldPosition;
        double distance = math.length(distanceVector);

        orbPar.specificOrbitalEnergy = (relativeSpeed * relativeSpeed / 2) - (standardGravitationalParamater / distance);
        orbPar.semiMajorAxis = -standardGravitationalParamater / (2 * orbPar.specificOrbitalEnergy);
        orbPar.specificAngularMomentumVector = math.cross(distanceVector, relativeVelocity);
        orbPar.eccentricityVector = (math.cross(relativeVelocity, orbPar.specificAngularMomentumVector) / standardGravitationalParamater) - (distanceVector / distance);
        orbPar.eccentricityScalar = math.length(orbPar.eccentricityVector);
        orbPar.semiMinorAxis = orbPar.semiMajorAxis * math.sqrt(1 - orbPar.eccentricityScalar * orbPar.eccentricityScalar);
        orbPar.apoapsis = orbPar.semiMajorAxis * (1 + orbPar.eccentricityScalar);
        orbPar.periapsis = orbPar.semiMajorAxis * (1 - orbPar.eccentricityScalar);
        orbPar.semiParamater = orbPar.semiMajorAxis * (1 - orbPar.eccentricityScalar * orbPar.eccentricityScalar);
        orbPar.inclination = math.acos((float)(orbPar.specificAngularMomentumVector.y / math.length(orbPar.specificAngularMomentumVector)));
        orbPar.ascendingNodeVector = math.cross(orbPar.specificAngularMomentumVector, new double3(0, 1, 0));
        if (math.length(orbPar.ascendingNodeVector) < 1e-10)
        {
            orbPar.longitudeOfTheAscendingNode = 0;
            if(orbPar.inclination > math.PI/2) 
                orbPar.argumentOfPeriapsis = math.atan2(-(float)orbPar.eccentricityVector.z, (float)orbPar.eccentricityVector.x);
            else
                orbPar.argumentOfPeriapsis = math.atan2((float)orbPar.eccentricityVector.z, (float)orbPar.eccentricityVector.x);
        }
        else
        {
            orbPar.longitudeOfTheAscendingNode = math.atan2((float)orbPar.ascendingNodeVector.z, (float)orbPar.ascendingNodeVector.x);
            orbPar.argumentOfPeriapsis = math.acos((float)(math.dot(orbPar.ascendingNodeVector, orbPar.eccentricityVector) / (math.length(orbPar.ascendingNodeVector) * math.length(orbPar.eccentricityVector))));
            if (orbPar.eccentricityVector.y < 0)
            {
                orbPar.argumentOfPeriapsis = 2.0 * math.PI - orbPar.argumentOfPeriapsis;
            }
        }
        orbPar.orbitalPeriod = 2.0 * math.PI * math.sqrt(math.pow(orbPar.semiMajorAxis, 3) / standardGravitationalParamater);
        orbPar.meanMotion = 2.0 * math.PI / orbPar.orbitalPeriod;
        orbPar.trueAnomaly = math.acos((float)(math.dot(distanceVector, orbPar.eccentricityVector) / (orbPar.eccentricityScalar * distance)));
        if (math.dot(distanceVector, relativeVelocity) < 0)
        {
            orbPar.trueAnomaly = 2.0 * math.PI - orbPar.trueAnomaly;
        }
        double cosE = (math.cos(orbPar.trueAnomaly) + orbPar.eccentricityScalar) / (1 + orbPar.eccentricityScalar * math.cos(orbPar.trueAnomaly));
        cosE = math.clamp(cosE, -1.0, 1.0);
        orbPar.eccentricAnomaly = math.acos(cosE);
        if (orbPar.trueAnomaly > math.PI)
        {
            orbPar.eccentricAnomaly = 2.0 * math.PI - orbPar.eccentricAnomaly;
        }
        orbPar.meanAnomaly = orbPar.eccentricAnomaly - orbPar.eccentricityScalar * math.sin(orbPar.eccentricAnomaly);

        body.orbitalParamaters = orbPar;
    }

    // the orbit line was last drawn open (one branch of a hyperbola): its colours and widths need setting back
    bool orbitDrawnOpen;

    public Vector3[] GetOrbitPoint()
    {
        LineRenderer lr = GlobalProperties.orbitRenderers[gameObject];
        int n = lr.positionCount;
        Vector3[] orbitPoints = new Vector3[n];
        OrbitalParamaters op = orbitalParamaters;
        double e = op.eccentricityScalar;

        // An open orbit (e >= 1: a hyperbola, a parabola at e = 1) has only the part where 1 + e cos(nu) > 0, i.e.
        // |nu| < acos(-1/e). Sweeping the whole circle also drew the other branch (r < 0 past the asymptotes) and the
        // jump through infinity between them. Open orbits are drawn over that branch only, as a line that does not
        // close, out to rMax from the dominant body.
        // (The angle the points are drawn at, a below, is minus the true anomaly: the body sits at a = -nu, and a
        // growing a runs back along the path, against the motion. A closed orbit starts at the body and runs back
        // round, so its bright start is the body and it fades behind it.)
        bool open = e >= 1.0;
        double aStart = -op.trueAnomaly;
        double aStep = 2.0 * math.PI / n;
        float bodyFrac = 0f;
        int kBody = 0;
        double a0 = 0.0, aLimOpen = 0.0;
        if (open)
        {
            double nu0 = op.trueAnomaly > math.PI ? op.trueAnomaly - 2.0 * math.PI : op.trueAnomaly;
            double rNow = op.semiParamater / (1.0 + e * math.cos(nu0));
            double rPeri = op.semiParamater / (1.0 + e);
            double rMax = math.max(2.0 * rNow, 10.0 * rPeri);
            double aAsym = math.acos(-1.0 / e);
            double aLim = math.acos(math.clamp((op.semiParamater / rMax - 1.0) / e, -1.0, 1.0));
            aLim = math.min(math.max(aLim, math.abs(nu0) + 1e-6), aAsym - 1e-6);

            // Points sweep strictly from the incoming end (true anomaly -aLim, a = +aLim) through the body (a = -nu0)
            // to the outgoing end (true anomaly +aLim, a = -aLim). The body is one of the points, and the points are
            // shared out between its two sides by their lengths: a line renderer lays its colours out along the
            // line's length, and points even in angle are far apart out toward the asymptotes -- the bright point
            // landed well off the body.
            a0 = -nu0;
            aLimOpen = aLim;
            double lenIn = ConicArcLength(op.semiParamater, e, aLim, a0);
            double lenOut = ConicArcLength(op.semiParamater, e, a0, -aLim);
            kBody = (int)math.round((n - 1) * lenIn / math.max(lenIn + lenOut, 1e-300));
            kBody = math.clamp(kBody, 1, n - 2);
        }

        for (int i = 0; i < n; i++)
        {
            double trueAnomaly;
            if (open)
                trueAnomaly = (i <= kBody) ? aLimOpen + (a0 - aLimOpen) * i / kBody
                                           : a0 + (-aLimOpen - a0) * (i - kBody) / (n - 1 - kBody);
            else
            {
                trueAnomaly = aStart + i * aStep;
                if (trueAnomaly > 2.0 * math.PI) trueAnomaly -= 2.0 * math.PI;
            }
            double3 p = new double3();
            double r = op.semiParamater / (1 + op.eccentricityScalar * math.cos(trueAnomaly));
            p.x = r * math.cos(trueAnomaly);
            p.z = r * math.sin(trueAnomaly);

            double x = p.x * math.cos(op.argumentOfPeriapsis) - p.z * math.sin(op.argumentOfPeriapsis);
            double z = p.x * math.sin(op.argumentOfPeriapsis) + p.z * math.cos(op.argumentOfPeriapsis);

            p.x = x;
            p.z = z;

            z = p.z * math.cos(op.inclination);
            double y = p.z * math.sin(op.inclination);

            p.z = z;
            p.y = y;

            x = p.x * math.cos(op.longitudeOfTheAscendingNode) - p.z * math.sin(op.longitudeOfTheAscendingNode);
            z = p.x * math.sin(op.longitudeOfTheAscendingNode) + p.z * math.cos(op.longitudeOfTheAscendingNode);

            p.x = x;
            p.z = z;

            p += dominantBody.GetComponent<Properties>().worldPosition + GlobalProperties.offset;
            Vector3 pf = new Vector3((float)(p.x / GlobalProperties.scale), (float)(p.y / GlobalProperties.scale), (float)(p.z / GlobalProperties.scale));
            orbitPoints[i] = pf;
        }

        lr.loop = !open;
        if (open)
        {
            // where the body is along the line, as a share of its length (the body is point kBody)
            double total = 0.0, atBody = 0.0;
            for (int i = 1; i < n; i++)
            {
                total += (orbitPoints[i] - orbitPoints[i - 1]).magnitude;
                if (i == kBody) atBody = total;
            }
            bodyFrac = total > 0.0 ? (float)(atBody / total) : 0.5f;

            // brightening from the incoming end to the body, faint from the body on
            float t1 = Mathf.Clamp(bodyFrac, 0.0005f, 0.999f);
            Gradient g = new Gradient();
            g.SetKeys(new[] { new GradientColorKey(Color.white, 0f), new GradientColorKey(Color.white, 1f) },
                      new[] { new GradientAlphaKey(0.05f, 0f), new GradientAlphaKey(1f, t1), new GradientAlphaKey(0.05f, t1), new GradientAlphaKey(0.05f, 1f) });
            lr.colorGradient = g;
            lr.widthMultiplier = 1f;
            orbitDrawnOpen = true;
        }
        else if (orbitDrawnOpen)
        {
            Gradient g = new Gradient();
            g.SetKeys(new[] { new GradientColorKey(Color.white, 0f), new GradientColorKey(Color.white, 1f) },
                      new[] { new GradientAlphaKey(1f, 0f), new GradientAlphaKey(0.05f, 1f) });
            lr.colorGradient = g;
            orbitDrawnOpen = false;
        }
        return orbitPoints;
    }

    // length of the conic r = p / (1 + e cos a) from angle a1 to a2 (in its plane: the rotations keep lengths)
    static double ConicArcLength(double p, double e, double a1, double a2)
    {
        const int m = 64;
        double len = 0.0, px = 0.0, pz = 0.0;
        for (int j = 0; j <= m; j++)
        {
            double a = a1 + (a2 - a1) * j / m;
            double r = p / (1.0 + e * math.cos(a));
            double x = r * math.cos(a), z = r * math.sin(a);
            if (j > 0) len += math.sqrt((x - px) * (x - px) + (z - pz) * (z - pz));
            px = x;
            pz = z;
        }
        return len;
    }
}
