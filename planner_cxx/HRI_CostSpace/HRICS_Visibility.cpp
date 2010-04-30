/*
 *  HRICS_Visbility.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "../../API/planningAPI.hpp"
#include "HRICS_Visibility.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

const int HUMANj_NECK_PAN=  4;
const int HUMANj_NECK_TILT= 7; // 5

const double HRI_EYE_TOLERANCE_TILT=0.3;
const double HRI_EYE_TOLERANCE_PAN=0.3;

double Visibility::getVisibilityCost(Vector3d WSPoint)
{
    double phi,theta;
    double Dphi, Dtheta;
	//    double Ccoord[6];
    p3d_vector4 realcoord,newcoord;
    p3d_matrix4 inv;
	
    realcoord[0] = WSPoint[0];
    realcoord[1] = WSPoint[1];
    realcoord[2] = WSPoint[2];
    realcoord[3] = 1;
	
    // get the right frame
    p3d_matrix4 rotation = {
        {-1,0,0,0},
        {0,-1,0,0},
        {0,0,1,0},
        {0,0,0,1}};
    p3d_matrix4 newABS;
    
	//Warning
	//p3d_mat4Mult(mHuman->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos,rotation,newABS);
	
    // Invert frame and get the point in this frame
    p3d_matInvertXform(
					   newABS, inv);
    p3d_matvec4Mult(inv, realcoord, newcoord);
	
	
    // Compute the angle the point make with the
    Vector3d newCoordVect;
    newCoordVect[0] = newcoord[0];
    newCoordVect[1] = newcoord[1];
    newCoordVect[2] = newcoord[2];
	
    phi = ABS(atan2( newCoordVect[0],newCoordVect[1]));
    theta = ABS(acos( newCoordVect[2]/newCoordVect.norm() )- M_PI_2);
	
    if(phi < HRI_EYE_TOLERANCE_PAN/2.)
        Dphi = 0;
    else
        Dphi = phi - HRI_EYE_TOLERANCE_PAN/2.;
	
    if(theta < HRI_EYE_TOLERANCE_TILT/2.)
        Dtheta = 0;
    else
        Dtheta = theta - HRI_EYE_TOLERANCE_TILT/2.;
	
    double cost = (1/0.65)*((Dtheta+Dphi)/(M_2PI-(HRI_EYE_TOLERANCE_TILT/2.)-(HRI_EYE_TOLERANCE_PAN/2.)));
	
	//    cout << "Visib =  "  << cost << endl;
    return cost;
}
