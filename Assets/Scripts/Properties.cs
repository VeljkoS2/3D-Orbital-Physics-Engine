using Unity.Mathematics;
using UnityEngine;

public class Properties : MonoBehaviour
{
    public double mass = 1;
    public bool hasAtmosphere = false;
    public bool hasRings = false;
    public double3 worldPosition = double3.zero;
    public double3 acceleration = double3.zero;
    public double3 velocity = double3.zero;
    public double radius { get; set; } = 0;
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
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        CalculateRadius();
        if (type == 2 /*|| type == 3*/)
        {
            if (!GlobalProperties.stars.Contains(transform))
            {
                GlobalProperties.stars.Add(transform);
                if (GlobalProperties.closestStar == null) GlobalProperties.closestStar = gameObject;
            }
        }
        if (type == 3)
        {
            if(!GlobalProperties.blackHoles.Contains(gameObject))
            {
                GlobalProperties.blackHoles.Add(gameObject);
                Debug.Log("yay");
            }
        }
        transform.rotation = Quaternion.Euler(startingRotation);
        rotationalSpeedDeg = 360 / rotationalPeriod;
    }

    // Update is called once per frame
    void Update()
    {
        Rotate(transform, rotationalSpeedDeg);
    }

    public void Rotate(Transform transform, double rotationalSpeedDeg)
    {
        transform.Rotate(0, -(float)(rotationalSpeedDeg * Time.deltaTime * GlobalProperties.timeScale), 0);
    }

    public void CalculateRadius()
    {
        if (mass > 6e32)
        {
            CalculateBlackHoleParamaters(this);
            radius = blackHoleParamaters.schwarzschildRadiusWorld*1.5;
            blackHoleParamaters.diskInnerRadiusNormalized = 3.0f;
            blackHoleParamaters.diskOuterRadiusNormalized = 12.0f;//rnd.NextFloat(clamped * 0.8f, clamped * 1.2f);
            blackHoleParamaters.escapeRadiusNormalized = blackHoleParamaters.diskOuterRadiusNormalized * 2.0f;
            //SetShaderPropperties(blackHoleParamaters);
            //gameObject.AddComponent<BlackHoleLod>();
            type = 3;
            //double visualInfluenceRadius = blackHoleParamaters.schwarzschildRadiusUnits*2.6;
            //transform.localScale = new Vector3((float)(visualInfluenceRadius) * 2f, (float)(visualInfluenceRadius) * 2f, (float)(visualInfluenceRadius) * 2f);
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
    public void CalculateLight()
    {
        Renderer renderer = GetComponent<Renderer>();
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

    public Vector3[] GetOrbitPoint()
    {
        Vector3[] orbitPoints = new Vector3[GlobalProperties.orbitRenderers[gameObject].positionCount];
        OrbitalParamaters op = orbitalParamaters;
        for (int i = 0; i < GlobalProperties.orbitRenderers[gameObject].positionCount; i++)
        {
            double trueAnomaly = -op.trueAnomaly + i * (2.0 * math.PI / GlobalProperties.orbitRenderers[gameObject].positionCount);
            if (trueAnomaly > 2.0 * math.PI) trueAnomaly -= 2.0 * math.PI;
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
        return orbitPoints;
    }
}
