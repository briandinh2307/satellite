#include "constellation.h"
#include "ns3/geographic-positions.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include <vector>

#define teledesic_alt 1375 // Polar satellite altitude
#define teledesic_inc 84.7 // Orbit inclination
#define teledesic_nPlane 12 // Number of planes
#define teledesic_nSat 24   // Number of satellites on each plane

using namespace ns3;

struct SatCoord
{
    double satLong;
    double satLati;
    double satAlti;
};

class Constellation
{
    public:
        Constellation(double c_alt, double c_inc, uint c_nPlane, uint c_nSat):
                      alt(c_alt), inc(c_inc), nPlane(c_nPlane), nSat(c_nSat) {}

        ~ Constellation()
        {
            delete[] polarSat;
        }

        void SetSatellite();
        void Set(uint satNode);
        Vector SphericalToCartesianCoordinates(double, double, double);
        SatCoord SatPos(uint satNode);
        Coordinate Coord();
        
    protected:
        double alt;
        double inc;
        uint nPlane;
        uint nSat;
        double period;
        Coordinate init;
        SatCoord satellite;
        OrbitalSat* polarSat = new OrbitalSat [nPlane * nSat];
        SatGeometry geometry;
};


void Constellation::SetSatellite()
{
    double satAngle = 360.0/nSat;
    double planeAngle = 360.0/nPlane;
    double offsetMulti = 5.0;
    double phaseOffset = offsetMulti/nPlane;
    for(uint a = 0; a < nPlane; a++)
        for(uint i = 0; i < nSat; i++)
        {   
            polarSat[a*nSat+i].altitude = alt;
            polarSat[a*nSat+i].inclination =  inc;
            polarSat[a*nSat+i].longitude = planeAngle * a;
            polarSat[a*nSat+i].alpha = i*satAngle + a*phaseOffset*satAngle;
            polarSat[a*nSat+i].plane = a;
        }

    std::cout << polarSat[25].alpha << std::endl;
}

void Constellation::Set(uint satNode)
{
    double satAlt = polarSat[satNode].altitude; 
    double satLon = polarSat[satNode].longitude;
    double satAlpha = polarSat[satNode].alpha;
    double satIncl = polarSat[satNode].inclination;

    // Check parameter
    // std::cout << "Node: " << satNode << std::endl;
    // std::cout << "alpha: " << satAlpha << std::endl;
    // std::cout << "longitude: " << satLon << std::endl;
    // std::cout << "inclination: " << satIncl << std::endl;
    //
	init.r = satAlt + EARTH_RADIUS; // Altitude in km above the earth
	if (satAlpha < 0)
        exit(1);
    
    if (satAlpha >= 360)
        init.theta = DEG_TO_RAD(satAlpha - 360);
    else
    {
        init.theta = DEG_TO_RAD(satAlpha);
    }
    // if (satLon < -180 || satLon > 180)
	// 	exit(1);
	if(satLon < 0)
		init.phi = DEG_TO_RAD(360 + satLon);
	else
		init.phi = DEG_TO_RAD(satLon);

    // Check parameter
    // if(satNode == 25)
    // {
    //     std::cout << "theta: " << init.theta << std::endl;
    // }
    if(satIncl < 0 || satIncl > 180)
		exit(1);
	inc = DEG_TO_RAD(satIncl);

	double num = init.r * init.r * init.r;
	period = 2 * PI * sqrt(num/MU); // seconds
}

Vector Constellation::SphericalToCartesianCoordinates(double r, double theta, double phi)
{
    double x = r * sin(theta) * cos (phi);
	double y = r * sin(theta) * sin (phi);
	double z = r * cos(theta);
    Vector cartesian(x, y, z);
    return cartesian;
}

Coordinate Constellation::Coord()
{
	Coordinate current;
	double partial;  // fraction of orbit period completed
	partial = (fmod(Simulator::Now().GetSeconds(), period)/period) * 2*PI; //rad
	double theta_cur, phi_cur, theta_new, phi_new;

	// Compute current orbit-centric coordinates:
	// theta_cur adds effects of time (orbital rotation) to init.theta
	theta_cur = fmod(init.theta + partial, 2*PI);
	phi_cur = init.phi;
	// Reminder:  theta_cur and phi_cur are temporal translations of 
	// initial parameters and are NOT true spherical coordinates.
	//
	// Now generate actual spherical coordinates,
	// with 0 < theta_new < PI and 0 < phi_new < 360

	//assert (inc < PI);

	// asin returns value between -PI/2 and PI/2, so 
	// theta_new guaranteed to be between 0 and PI
	theta_new = PI/2 - asin(sin(inc) * sin(theta_cur));
	// if theta_new is between PI/2 and 3*PI/2, must correct
	// for return value of atan()
	if ((theta_cur > PI/2 && theta_cur < 3*PI/2) || theta_cur == DEG_TO_RAD(270.0))
		phi_new = atan(cos(inc) * tan(theta_cur)) + phi_cur + PI;
	else
		phi_new = atan(cos(inc) * tan(theta_cur)) + phi_cur;
	phi_new = fmod(phi_new + 2*PI, 2*PI);
	
	current.r = init.r;
	current.theta = theta_new;
	current.phi = phi_new;
	return current;
}

SatCoord Constellation::SatPos(uint sat)
{
    Set(sat);
    satellite.satLati = RAD_TO_DEG(geometry.get_latitude(Coord()));
    satellite.satLong = RAD_TO_DEG(geometry.get_longitude(Coord()));
    satellite.satAlti = geometry.get_altitude(Coord());

    return satellite;
    //return SphericalToCartesianCoordinates(init.r, init.theta, init.phi);
}

static void 
PositionCall (Ptr<const Node> node, uint tmp_sat)
{
    Ptr<MobilityModel> tmp_pos = node->GetObject<MobilityModel>();
    uint sat_th = node->GetId();
    uint plane_th = sat_th / tmp_sat;

    Vector pos = tmp_pos->GetPosition();
    std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", " << sat_th + 1 << ", "
              << plane_th + 1 << ", " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
}

void Initialize(NodeContainer* iSat, Constellation* iTeledesic)
{
    SatCoord geoPos;
    for (NodeContainer::Iterator iter = iSat->Begin(); iter != iSat->End(); ++iter)
    {
        Ptr<Node> tmp_node = (*iter);
        uint sat = tmp_node->GetId();
        geoPos = iTeledesic->SatPos(sat);

        double latitude = geoPos.satLati;
        double longitude = geoPos.satLong;
        double altitude = geoPos.satAlti;

        iSat->Get(sat)->GetObject<MobilityModel>()->SetPosition(Vector(longitude, latitude, altitude));
    }
    Simulator::Schedule (Seconds (0.2), &Initialize, iSat, iTeledesic);
}

double distance(Vector a, Vector b)
{
    double dist_x = (a.x - b.x)*(a.x - b.x);
    double dist_y = (a.x - b.x)*(a.x - b.x);
    return sqrt(dist_x + dist_y);
}

int main (int argc, char *argv[])
{

    double alt = teledesic_alt;
    double inc = teledesic_inc;
    uint nPlane = teledesic_nPlane;
    uint nSat = teledesic_nSat;
    std::string animFile = "constellation.xml";
    double watchTime = 0.0;
    double simTime = 100.0;

    CommandLine cmd;
    cmd.AddValue ("alt", "Altitude of the satellites", alt);
    cmd.AddValue ("inc", "Inclination of the orbits", inc);
    cmd.AddValue ("nPlane", "Number of planes", nPlane);
    cmd.AddValue ("nSat", "Number of satellite on each plane", nSat);
    cmd.AddValue ("animFile", "File Name for Animation Output", animFile);
    cmd.AddValue ("watchTime", "Observe satellites position at 'watchTime'", watchTime);
    cmd.AddValue ("simTime", "Time of the simulation", simTime);
    cmd.Parse (argc, argv);

    alt = alt == 0.0 ? teledesic_alt : alt;
    inc = inc == 0.0 ? teledesic_inc : inc;
    nPlane = nPlane == 0 ? teledesic_nPlane : nPlane;
    nSat = nSat == 0 ? teledesic_nSat : nSat;
    simTime = simTime == 0 ? 100.0 : simTime;

    NodeContainer satellite;
    satellite.Create (nPlane * nSat);

    MobilityHelper mobility;

    Constellation satNetWork(alt, inc, nPlane, nSat);
    satNetWork.SetSatellite();
    
    //mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator");
    mobility.Install (satellite);

    Initialize(&satellite, &satNetWork);

    // std::vector<SatCoord> link;
    // for(uint a = 0; a < nPlane; a++)
    //     for(uint i = 0; i < nSat; i++)
    //     {
    //         uint cur = a*nSat+i;
    //         uint next = (a+1)*nSat+i;
    //         Ptr<MobilityModel> cur_sat = satellite.Get(cur)->GetObject<MobilityModel>();
    //         Vector cur_pos = cur_sat->GetPosition();
    //         Ptr<MobilityModel> next_sat = satellite.Get(next)->GetObject<MobilityModel>();
    //         Vector next_pos = next_sat->GetPosition();
    //     }
            

    PointToPointHelper pointToPoint;
    for(uint a = 0; a < nPlane; a++)
        for(uint i = 0; i < nSat; i++)
        {
            uint link = a*nSat + i;
            // Intraplane link
            if(i == nSat - 1)
                pointToPoint.Install(satellite.Get(link), satellite.Get(a*nSat));
            else
                pointToPoint.Install(satellite.Get(link), satellite.Get(link+1));
                
            // Interplane link
            // if(a == nPlane - 1)
            //     if(i > nSat - 5)
            //         pointToPoint.Install(satellite.Get(link), satellite.Get(i-nSat+5));
            //     else
            //         pointToPoint.Install(satellite.Get(link), satellite.Get(i+5));
            // else
            //     pointToPoint.Install(satellite.Get(link), satellite.Get(link + nSat));
        }
    

    
    //Check
    satellite.Get(0)->GetObject<PointToPointChannel>();
    int x = satellite.Get(0)->GetNDevices();
    std::cout << "Device: " << x << std::endl;
    //

    for (NodeContainer::Iterator iter = satellite.Begin(); iter != satellite.End(); ++iter)
    {
        Ptr<Node> tmp_node = (*iter);
        Simulator::Schedule (Seconds(watchTime), &PositionCall, tmp_node, nSat);
    }

    AnimationInterface anim(animFile);
    anim.SetBackgroundImage("1.gif", -180.0, -90.0, 0.75, 0.75, 0.5);

    for(uint a = 0; a < nPlane; a++)
            for(uint i = 0; i < nSat; i++)
            {
                //Plane 1
                if(a == 0)
                {
                    anim.UpdateNodeColor(a*nSat+i, 0, 0, 255);
                    anim.UpdateNodeSize(a*nSat+i, 4, 4);
                }
                
                //Plane 2
                if(a == 1)
                {
                    anim.UpdateNodeColor(a*nSat+i, 0, 255, 0);
                    anim.UpdateNodeSize(a*nSat+i, 4, 4);
                }

                std::string planeNum = std::to_string(a+1);
                std::string satNum = std::to_string(i+1);
                std::string label = planeNum + "/" + satNum;
                anim.UpdateNodeDescription(a*nSat+i, label);
            }
    
    Simulator::Stop (Seconds (simTime));
    
    std::cout << "Event: " << Simulator::GetEventCount() << std::endl;

    Simulator::Run ();

    Simulator::Destroy ();
    return 0;
}