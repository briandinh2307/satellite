#include "constellation.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include <vector>

#define teledesic_alt 1375 // Polar satellite altitude
#define teledesic_inc 84.7 // Orbit inclination
#define teledesic_nPlane 12 // Number of planes
#define teledesic_nSat 24   // Number of satellites on each plane

using namespace ns3;

static void 
PositionCall (Ptr<const Node> node, uint32_t tmp_sat)
{
    Ptr<MobilityModel> tmp_pos = node->GetObject<MobilityModel>();
    uint32_t sat_th = node->GetId();
    uint32_t plane_th = sat_th / tmp_sat;

    Vector pos = tmp_pos->GetPosition();
    std::cout << "t = " << Simulator::Now().GetSeconds() << "s" << ", " << sat_th + 1 << ", "
              << plane_th + 1 << ", " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
}

void SetSatPos (NodeContainer* satPos, Constellation* wDelta)
{
    Vector geoPos;
    for (NodeContainer::Iterator iter = satPos->Begin(); iter != satPos->End(); ++iter)
    {
        Ptr<Node> tmp_node = (*iter);
        uint32_t sat = tmp_node->GetId();
        geoPos = wDelta->SatPos(sat);

        double longitude = geoPos.x;
        double latitude = geoPos.y;
        double altitude = geoPos.z;

        satPos->Get(sat)->GetObject<MobilityModel>()->SetPosition(Vector(longitude, latitude, altitude));
    }
    Simulator::Schedule (Seconds (0.2), &SetSatPos, satPos, wDelta);
}

// Check for position
static void RetrievePos(Constellation* netWork)
{
    Vector* a = netWork->GetPos();
    std::cout << "x: " << a[1].x << " y: " << a[1].y << std::endl;
}

int main (int argc, char *argv[])
{

    double alt = teledesic_alt;
    double inc = teledesic_inc;
    uint32_t nPlane = teledesic_nPlane;
    uint32_t nSat = teledesic_nSat;
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
    mobility.Install (satellite);

    Constellation satNetWork(alt, inc, nPlane, nSat);
    SetSatPos(&satellite, &satNetWork);

    // Check for position
    Simulator::Schedule (Seconds(watchTime), &RetrievePos, &satNetWork);
    // std::vector<SatCoord> link;
    // for(uint32_t a = 0; a < nPlane; a++)
    //     for(uint32_t i = 0; i < nSat; i++)
    //     {
    //         uint32_t cur = a*nSat+i;
    //         uint32_t next = (a+1)*nSat+i;
    //         Ptr<MobilityModel> cur_sat = satellite.Get(cur)->GetObject<MobilityModel>();
    //         Vector cur_pos = cur_sat->GetPosition();
    //         Ptr<MobilityModel> next_sat = satellite.Get(next)->GetObject<MobilityModel>();
    //         Vector next_pos = next_sat->GetPosition();
    //     }
            

    PointToPointHelper pointToPoint;
    for(uint32_t a = 0; a < nPlane; a++)
        for(uint32_t i = 0; i < nSat; i++)
        {
            uint32_t link = a*nSat + i;
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

    for(uint32_t a = 0; a < nPlane; a++)
            for(uint32_t i = 0; i < nSat; i++)
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