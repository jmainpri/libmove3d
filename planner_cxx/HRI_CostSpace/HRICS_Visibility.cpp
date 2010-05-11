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

#include "HRICS_costspace.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

#define DISTANCE3D(x1,y1,z1,x2,y2,z2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))+((z2)-(z1))*((z2)-(z1))))



Visibility::Visibility(Robot* H)
{
	if (H->getName().find("HUMAN") == string::npos) 
	{
		cout << "HRICS::Visibility::Warning Human's name does not contain HUMAN" << endl;
		return;
	}
	
	m_Human = H;
}

/* The coordinates change from one human model to the other */
/* Change these numbers when you change the human model */
/* The rest will follow these changes */
//#define HUMANq_X 6
//#define HUMANq_Y 7
//#define HUMANq_Z 8
//#define HUMANq_RZ 11
//#define HUMANq_NECKZ 63
//#define HUMANq_PAN 64
//#define HUMANq_TILT 65

const int __HUMANj_NECK_PAN=  5; // 7
const int __HUMANj_NECK_TILT= 6; // 6

const double __HRI_EYE_TOLERANCE_TILT=0.3;
const double __HRI_EYE_TOLERANCE_PAN=0.3;

const bool Old_Achile =true;

double Visibility::getVisibilityCost(const Vector3d& WSPoint)
{
	
	//return akinVisibilityCost(WSPoint);
	
    double phi,theta;
    double Dphi, Dtheta;
    p3d_vector4 realcoord,newcoord;
    p3d_matrix4 inv;
	
	realcoord[0] = WSPoint[0];
    realcoord[1] = WSPoint[1];
    realcoord[2] = WSPoint[2];
    realcoord[3] = 1;
	
	if (Old_Achile) 
	{
		// get the right frame
		p3d_matrix4 newABS;
		p3d_matrix4 rotation ={	{1,0,0,0},
								{0,1,0,0},
								{0,0,1,0},
								{0,0,0,1}};
		//Matrix4d newAbsPos; = m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos
		
		p3d_mat4Mult(m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos,rotation,newABS);
		
		// Invert frame and get the point in this frame
		p3d_matInvertXform(newABS, inv);
		p3d_matvec4Mult(inv, realcoord, newcoord);
	}
	else 
	{
		double Ccoord[6];
		p3d_mat4ExtractPosReverseOrder(m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos,
									   Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);
		
		p3d_matInvertXform(m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos, inv);
		p3d_matvec4Mult(inv, realcoord, newcoord);
	}
	
    // Compute the angle the point make with the
	// Angular coord of the vector
    Vector3d newCoordVect;
    newCoordVect[0] = newcoord[0];
    newCoordVect[1] = newcoord[1];
    newCoordVect[2] = newcoord[2];
	
    phi = ABS(atan2( newCoordVect[1],newCoordVect[0]));
    theta = ABS(acos( newCoordVect[2]/newCoordVect.norm() )- M_PI_2);
	
	// Compute delta cost for PAN
    if(phi < __HRI_EYE_TOLERANCE_PAN/2.)
        Dphi = 0;
    else
        Dphi = phi - __HRI_EYE_TOLERANCE_PAN/2.;
	
	// Compute delta cost for TILT
    if(theta < __HRI_EYE_TOLERANCE_TILT/2.)
        Dtheta = 0;
    else
        Dtheta = theta - __HRI_EYE_TOLERANCE_TILT/2.;
	
    m_Cost = (1/0.65)*((Dtheta+Dphi)/(M_2PI-(__HRI_EYE_TOLERANCE_TILT/2.)-(__HRI_EYE_TOLERANCE_PAN/2.)));
	
	//    cout << "Visib =  "  << cost << endl;
    return m_Cost;
}

double Visibility::akinVisibilityCost(const Vector3d& WSPoint)
{

	double phi,theta;
	double Dphi, Dtheta;
	double Ccoord[6];
	p3d_vector4 realcoord,newcoord;
	p3d_matrix4 inv;
	
	realcoord[0] = WSPoint[0];
	realcoord[1] = WSPoint[1];
	realcoord[2] = WSPoint[2];
	realcoord[3] = 1;
	
	p3d_mat4ExtractPosReverseOrder(m_Human->getRobotStruct()->joints[__HUMANj_NECK_TILT]->abs_pos,
								   Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);
	
	p3d_matInvertXform(m_Human->getRobotStruct()->joints[__HUMANj_NECK_TILT]->abs_pos, inv);
	
	p3d_matvec4Mult(inv, realcoord, newcoord);
	
	//p3d_psp_cartesian2spherical(newcoord[0],newcoord[1],newcoord[2],
//								0,0,0,&phi,&theta);

	{
		double x =newcoord[0];
		double y =newcoord[1];
		double z =newcoord[2];
		
		double originx=0;
		double originy=0;
		double originz=0;
		
		double distance = DISTANCE3D(x,y,z,originx,originy,originz);
			
		phi = atan2( (y-originy),(x-originx) );
		theta = acos( (z-originz)/distance );
	}
	
	phi = ABS(phi);
	theta = ABS(theta - M_PI_2);
	
	if(phi < HRI_EYE_TOLERANCE_PAN/2.)
		Dphi = 0;
	else
		Dphi = phi - HRI_EYE_TOLERANCE_PAN/2.;
	
	if(theta < HRI_EYE_TOLERANCE_TILT/2.)
		Dtheta = 0;
	else
		Dtheta = theta - HRI_EYE_TOLERANCE_TILT/2.;
	
	return (Dtheta+Dphi)/(M_2PI-(HRI_EYE_TOLERANCE_TILT/2.)-(HRI_EYE_TOLERANCE_PAN/2.))/0.65;
	
}
std::vector<double> Visibility::getGaze()
{

	Vector3d point;
	p3d_vector3 gazeOrigin;
	p3d_vector3 xAxis;
	const double length = 1.5;
	//p3d_vector3 gazeDestin;
	p3d_jnt_get_cur_vect_point(m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT],gazeOrigin);
	//p3d_mat4Print(m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos, "HUMANj_NECK_TILT");
	
	m_VectGaze.clear();
	m_VectGaze.push_back(gazeOrigin[0]);
	m_VectGaze.push_back(gazeOrigin[1]);
	m_VectGaze.push_back(gazeOrigin[2]);
	
	
	xAxis[0]=  gazeOrigin[0] + length*m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos[0][0];
	xAxis[1]=  gazeOrigin[1] + length*m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos[1][0];
	xAxis[2]=  gazeOrigin[2] + length*m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos[2][0];
	
	m_VectGaze.push_back(xAxis[0]);
	m_VectGaze.push_back(xAxis[1]);
	m_VectGaze.push_back(xAxis[2]);  

	return m_VectGaze;
}


