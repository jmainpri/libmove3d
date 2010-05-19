/*
 *  joint.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/05/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "../planningAPI.hpp"
#include <string>

using namespace std;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

Joint::Joint(p3d_jnt* jntPt, bool copy )
{
	m_Joint = jntPt;
	
	string name(jntPt->name);
    m_Name = name;
	
}

Vector3d Joint::getVectorPos()
{
	Vector3d v;
	
	v(0) = m_Joint->abs_pos[0][3];
	v(1) = m_Joint->abs_pos[1][3];
	v(2) = m_Joint->abs_pos[2][3];
	
	return v;
}

Transform3d Joint::getMatrixPos()
{
	Transform3d t;
	
	t(0,0) = m_Joint->abs_pos[0][0];
	t(1,0) = m_Joint->abs_pos[1][0];
	t(2,0) = m_Joint->abs_pos[2][0];
	t(3,0) = 0.0;
	
	t(0,1) = m_Joint->abs_pos[0][1];
	t(1,1) = m_Joint->abs_pos[1][1];
	t(2,1) = m_Joint->abs_pos[2][1];
	t(3,1) = 0.0;
	
	t(0,2) = m_Joint->abs_pos[0][2];
	t(1,2) = m_Joint->abs_pos[1][2];
	t(2,2) = m_Joint->abs_pos[2][2];
	t(3,2) = 0.0;
	
	t(0,3) = m_Joint->abs_pos[0][3];
	t(1,3) = m_Joint->abs_pos[1][3];
	t(2,3) = m_Joint->abs_pos[2][3];
	t(3,3) = 1.0;
	
//	cout << "t of joint " << m_Joint->num << " = " << endl << t.matrix() << endl;
//	
//	p3d_mat4Print(m_Joint->abs_pos, "Abs Pos");
	
	//cout << "Warning: Joint::getAbsPos() undefined" << endl;
	return t;
}
