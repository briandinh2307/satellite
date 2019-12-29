#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include <math.h>

#define PI 3.1415926535897
#define MU 398601.2 // Greek Mu (km^3/s^2)
#define LIGHT 299793 // km/s
#define EARTH_PERIOD 86164 // seconds
#define EARTH_RADIUS 6378  // km
#define f_Delta 5

#define DEG_TO_RAD(x) ((x) * PI/180)
#define RAD_TO_DEG(x) ((x) * 180/PI)
#define DISTANCE(s_x, s_y, s_z, e_x, e_y, e_z) (sqrt((s_x - e_x) * (s_x - e_x) \
                + (s_y - e_y) * (s_y - e_y) + (s_z - e_z) * (s_z - e_z)))

using namespace ns3;

struct Coordinate
{
        double r;        // km
        double theta;    // radians
        double phi;      // radians
};

// Library of routines involving satellite geometry
class SatGeometry
{
public:
	SatGeometry () {}
        Vector SpheToCartCoord (Coordinate s);
        double distance_geo (Vector a, Vector b);            
	// static double propdelay(coordinate, coordinate);
	double get_latitude (Coordinate);
	double get_longitude (Coordinate);
	double get_radius (Coordinate a) { return a.r; }
	double get_altitude (Coordinate);
	// static double check_elevation(Coordinate, Coordinate, double);
	// static int are_satellites_mutually_visible(Coordinate, Coordinate);

protected: 
	// Define "command" appropriately if you want OTcl access to this class
        int command(/*int argc, const char*const* argv */) { return 0; }
};

Vector SatGeometry::SpheToCartCoord (Coordinate s)
{
        double x = s.r * sin(s.theta) * cos(s.phi);
        double y = s.r * sin(s.theta) * sin(s.phi);
        double z = s.r * cos(s.theta);
        return {x, y, z};
}

double SatGeometry::distance_geo (Vector a, Vector b)
{
        double dist_x = (a.x - b.x);
        dist_x = dist_x > 180.0 ? (360.0 - dist_x) : dist_x;
        double dist_y = (a.y - b.y);
        return sqrt(dist_x*dist_x + dist_y*dist_y);
}

double SatGeometry::get_altitude (Coordinate a)
{
        return (a.r - EARTH_RADIUS);
}

// Returns latitude in radians, in the range from -PI/2 to PI/2
double SatGeometry::get_latitude (Coordinate a)
{
        return (PI/2 - a.theta);
}

// Returns (earth-centric) longitude corresponding to the position of the node 
// (the input coordinate corresponds to fixed coordinate system, through
// which the Earth rotates, so we have to scale back the effects of rotation).
// The return value ranges from -PI to PI.
double SatGeometry::get_longitude (Coordinate coord_)
{
        double period = EARTH_PERIOD; // period of earth in seconds
        // adjust longitude so that it is earth-centric (i.e., account
        // for earth rotating beneath).   
        double earth_longitude = std::fmod((coord_.phi - (std::fmod(Simulator::Now().GetSeconds(), period)/period) * 2*PI), 2*PI);
	// Bring earth_longitude to be within (-PI, PI)
        if (earth_longitude < (-1*PI))
		earth_longitude = 2*PI + earth_longitude;
        if (earth_longitude > PI)
		earth_longitude = (-(2*PI - earth_longitude));
	if (std::fabs(earth_longitude) < 0.0001)
		return 0;   // To avoid trace output of "-0.00"
	else
		return (earth_longitude);
}       


//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------


struct OrbitalSat
{
        double altitude;
        double inclination;
        double longitude;
        double alpha;
        uint32_t plane;
};  

class Constellation
{
        public:
                Constellation (double c_alt, double c_inc, uint32_t c_nPlane, uint32_t c_nSat):
                               alt (c_alt), inc (c_inc), nPlane (c_nPlane), nSat (c_nSat) { SetSatellite(); }
                ~Constellation()
                {
                        delete[] satellite;
                }
                void SetInit ();
                Vector SatPos (uint32_t satNode);
                Coordinate Coord ();
                Vector* get_pos () { return satellite; }

                void CreateLink ();
                std::vector<Vector> get_link () { return link; }
                std::vector<uint32_t> get_linkdex () { return linkDex; }
                
        protected:
                double alt;
                double inc;
                uint32_t nPlane;
                uint32_t nSat;
                uint32_t curSat;
                double period;
                Vector* satellite = new Vector [nPlane * nSat];
                std::vector<OrbitalSat> polarSat;
                SatGeometry geometry;
                std::vector<Vector> link;       // Contains link to fifth satellite position
                std::vector<uint32_t> linkDex;   // Contain link to fifth satellite number

        private:
                void SetSatellite ();
                Coordinate init;
                Coordinate spheCoord;
                bool link_init = 0;
};

void Constellation::SetSatellite ()
{
        double satAngle = 360.0/nSat;
        double planeAngle = 360.0/nPlane;
        double offsetMulti = f_Delta;
        double phaseOffset = offsetMulti/nPlane;
        for (uint32_t a = 0; a < nPlane; a++)
                for (uint32_t i = 0; i < nSat; i++)
                        polarSat.push_back ({alt, inc, planeAngle*a, i*satAngle + a*phaseOffset*satAngle, a});
}

void Constellation::SetInit ()
{
        double satAlt = polarSat[curSat].altitude; 
        double satLon = polarSat[curSat].longitude;
        double satAlpha = polarSat[curSat].alpha;
        double satIncl = polarSat[curSat].inclination;

        init.r = satAlt + EARTH_RADIUS; // Altitude in km above the earth
        if (satAlpha < 0)
                exit(1);
        
        if (satAlpha >= 360)
                init.theta = DEG_TO_RAD(satAlpha - 360);
        else
                init.theta = DEG_TO_RAD(satAlpha);

        if (satLon < -360 || satLon > 360)
                exit(1);

        if (satLon < 0)
                init.phi = DEG_TO_RAD(360 + satLon);
        else
                init.phi = DEG_TO_RAD(satLon);

        if (satIncl < 0 || satIncl > 180)
                exit(1);
        inc = DEG_TO_RAD (satIncl);

        double num = init.r * init.r * init.r;
        period = 2 * PI * sqrt(num/MU); // seconds
}

Coordinate Constellation::Coord ()
{
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

	// asin returns value between -PI/2 and PI/2, so    //     
	// theta_new guaranteed to be between 0 and PI
	theta_new = PI/2 - asin(sin(inc) * sin(theta_cur));
	// if theta_new is between PI/2 and 3*PI/2, must correct
	// for return value of atan()
	if ((theta_cur > PI/2 && theta_cur < 3*PI/2) || theta_cur == DEG_TO_RAD(270.0))
		phi_new = atan(cos(inc) * tan(theta_cur)) + phi_cur + PI;
	else
		phi_new = atan(cos(inc) * tan(theta_cur)) + phi_cur;
	phi_new = fmod(phi_new + 2*PI, 2*PI);
	
	spheCoord.r = init.r;
	spheCoord.theta = theta_new;
	spheCoord.phi = phi_new;
	return spheCoord;
}

Vector Constellation::SatPos (uint32_t sat)
{
        curSat = sat;// Contains link to fifth satellite position
        SetInit();
        satellite[sat].x = RAD_TO_DEG(geometry.get_longitude(Coord()));
        satellite[sat].y = RAD_TO_DEG(geometry.get_latitude(Coord()));
        satellite[sat].z = geometry.get_altitude(Coord());
        return satellite[sat];
}

void Constellation::CreateLink ()
{
        // std::vector<uint32_t> secLinkIndex;
        for (uint32_t t = 0; t < nPlane*nSat; t++)
        {
                bool flag = 0;
                double minDist;
                // double secMinDist;
                double tempDist;
                uint32_t satIndex;
                // uint32_t secSatIndex;
                uint32_t curPlane = t/nSat;
                for (uint32_t a = 0; a < nPlane; a++)
                {
                        if (a == curPlane)
                                continue;
                        for (uint32_t i = 0; i < nSat; i++)
                        {       
                                // The fifth link won't connect to satellite with same index in adjacent planes
                                if (a*nSat + i == t + nSat || a*nSat + i == t - nSat)
                                        continue;
                                if (a == nPlane - 1)
                                {
                                        if (i >= nSat - f_Delta && t == i - nSat + f_Delta)
                                                continue;
                                        else if (t == i + 5)
                                                continue;
                                }     
                                
                                if (flag == 0)
                                {
                                        flag = 1;
                                        satIndex = a*nSat + i;
                                        minDist = geometry.distance_geo(satellite[t], satellite[satIndex]);
                                        // secSatIndex = a*nSat + i;
                                        // secMinDist = geometry.distance_geo(satellite[t], satellite[secSatIndex]);
                                        continue;
                                }
                                tempDist = geometry.distance_geo(satellite[t], satellite[a*nSat + i]);
                                if (minDist > tempDist)
                                {
                                        satIndex = a*nSat + i;
                                        minDist = tempDist;
                                }
                                // else if (secMinDist > tempDist)
                                // {
                                //         secSatIndex = a*nSat + i;
                                //         secMinDist = tempDist;
                                // }
                        }
                }
                if (link_init == 0)
                {
                        link.push_back(satellite[satIndex]);
                        linkDex.push_back(satIndex);
                        // secLinkIndex.push_back(secSatIndex);
                }
                else
                {
                        link[t] = satellite[satIndex];
                        linkDex[t] = satIndex;
                        // secLinkIndex[t] = secSatIndex;
                }                
        }

        for (uint32_t t = 0; t < nPlane * nSat; t++)
        {
                uint32_t tmp = linkDex[t];
                if (t != linkDex[tmp])
                {
                        linkDex[t] = t;
                        link[t] = satellite[t];
                }
        }

        // for (uint32_t t = 0; t < nPlane * nSat; t++)
        // {
        //         uint32_t tmp;
        //         if (t == linkDex[t])
        //         {
        //                 tmp = secLinkIndex[t];
        //                 if (tmp == secLinkIndex[tmp])
        //                 {
        //                         link[t] = satellite[tmp];
        //                         linkDex[t] = tmp;
        //                         link[tmp] = satellite[t];
        //                         linkDex[tmp] = t;
        //                 }
        //         }
        // }
        link_init = 1;
}