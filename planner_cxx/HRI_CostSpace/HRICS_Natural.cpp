/*
 *  HRICS_Natural.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "HRICS_Natural.h"

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

Natural::Natural(Robot* R) :
	m_Robot(R)
{
	map<string,Kinematic> kinos;
	
	kinos["Error"] = Default;
	kinos["ROBOT"] = Justin;
	kinos["HUMAN"] = Achile;
	
	m_KinType = kinos[m_Robot->getName()];
	
	switch (m_KinType) 
	{
		case Justin:
			cout << "KinType of HRICS::Natural is Justin ( " << m_Robot->getName() << " ) "<< endl;
			initNaturalJustin();
			break;
		
		case Achile:
			cout << "KinType of HRICS::Natural is Achile" << endl;
			break;

		default:
			cout << "No proper robot has been selected in Natural cost function" << endl;
			break;
	}
	
	m_IndexObjectDof = m_Robot->getObjectDof();
	
	cout << "Object Dof is " << m_IndexObjectDof << endl;
	
	vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;
	
	m_Grid = new NaturalGrid(ENV.getDouble(Env::CellSize),envSize);
	m_Grid->setNaturalCostSpace(this);
}

Natural::~Natural()
{
	delete m_Grid;
}

void Natural::initNaturalJustin()
{
	configPt q;
	q = p3d_alloc_config(m_Robot->getRobotStruct());
	
	q[0] = 0.000000;
	q[1] = 0.000000;
	q[2] = 0.000000;
	q[3] = 0.000000;
	q[4] = 0.000000;
	q[5] = 0.000000;
	q[6] = 1.573257;
	q[7] = -22.123896;
	q[8] = 31.659294;
	q[9] = -9.535398;
	q[10] = 0.000000;
	q[11] = 0.000000;
	q[12] = 5.014758;
	q[13] = -66.076698;
	q[14] = -15.044244;
	q[15] = 115.634224;
	q[16] = 93.608658;
	q[17] = -9.540314;
	q[18] = -3.672564;
	q[19] = -15.000000;
	q[20] = -46.000000;
	q[21] = -8.000000;
	q[22] = 119.000000;
	q[23] = 138.000000;
	q[24] = 62.000000;
	q[25] = 29.000000;
	
	m_q_Confort = shared_ptr<Configuration>(new Configuration(
											m_Robot,p3d_copy_config_deg_to_rad(
											m_Robot->getRobotStruct(),q)));
	
}

/*!
 * Compute the Natural cost for a configuration
 * with weight
 *
 * Input:  The robot,
 *         the two configurations
 *
 * See : p3d_dist_config
 */
double Natural::getCost()
{
	double max_taskdist;
	double max_jlimits;
	
	double c_taskdist;
	double c_natural;
	double c_jlimits;
	
	shared_ptr<Configuration> q_Actual = m_Robot->getCurrentPos();
	
	
	//---------------------------------------------------
	// Joints limits
	//---------------------------------------------------
	c_jlimits = getCustomDistConfig(q_Actual);
	
	//	if(c_jlimits > max_jlimits)
	//		c_jlimits = max_jlimits;
	c_jlimits /= max_jlimits;
	
	if(m_debug)
	{
		cout << "c_jlimits =" << c_jlimits << endl;
	}
	
	//---------------------------------------------------
	// Distance in task space
	//---------------------------------------------------
//	m_Robot->setAndUpdate(*m_q_Goal);
//	Vector3d posGoal = m_Robot->getJointPos(m_IndexObjectDof);
//	
//	m_Robot->setAndUpdate(*q_Actual);
//	Vector3d posActual = m_Robot->getJointPos(m_IndexObjectDof);
//	
//	if(m_debug)
//	{
//		cout << "position actuelle : ";
//		cout << posActual << endl;
//		cout << "position du but : ";
//		cout << posGoal << endl;
//	}
//	
//	
//	double dist = ( posActual - posGoal ).norm();
//	
//	if(m_debug)
//	{
//			cout << dist << endl;
//	}
//		
//	
//	c_taskdist += ENV.getDouble(Env::coeffHei)*dist;
//	
//	if(m_debug)
//	{
//		cout << "c_taskdist =" << c_taskdist << endl;
//		cout << endl;
//	}
	
	//	if(c_taskdist > max_taskdist)
	//		c_taskdist = max_taskdist;
//	c_taskdist /= max_taskdist;
//	
//	
//	if(m_debug)
//	{
//		cout << "c_taskdist =" << c_taskdist << endl;
//		cout << endl;
//	}
	
	double coeff_task = 0; //ENV.getDouble(Env::coeffTas)/100;
	double coeff_limi = ENV.getDouble(Env::coeffLim)/100;
	
	c_natural = coeff_task*c_taskdist + coeff_limi*c_jlimits;
	
	return c_natural;
}

/*!
 * Compute the classic square distance between two configurations
 * with weight
 *
 * Input:  The robot,
 *         the two configurations
 *
 * See : p3d_dist_config
 */
double Natural::getCustomDistConfig(shared_ptr<Configuration> q) 
{
	double l = 0., ljnt = 0.;
	int i, j, njnt = m_Robot->getRobotStruct()->njoints;
	p3d_jnt * jntPt;
	
	for (i=0; i<=njnt; i++) {
		
		jntPt = m_Robot->getRobotStruct()->joints[i];
		
		for (j=0; j<jntPt->dof_equiv_nbr; j++) {
			
			if (m_Robot->getRobotStruct()->cntrt_manager->in_cntrt[jntPt->index_dof + j]
				!= DOF_PASSIF) {
				//Jim Hri Modif
				
				double W;
				
				switch (jntPt->index_dof + j) {
					case 6:
						W = 2;
						break;
					case 7:
						W = 2;
						break;
					case 8:
						W = 2;
						break;
						
					case 12:
						W = 3;
						break;
					case 13:
						W = 3;
						break;
					case 14:
						W = 3;
						break;
					case 15:
						W = 3;
						break;
						
					case 16:
						W = 1;
						break;
					case 17:
						W = 1;
						break;
						
					default:
						W = 1;
				}
				double dof_dist = p3d_jnt_calc_dof_dist(jntPt, j, 
														m_q_Confort->getConfigStruct(), 
														q->getConfigStruct());
				//printf("dof_dist[%d] = %f\n",jntPt->index_dof + j,dof_dist);
				ljnt += W*SQR(dof_dist);
			}
		}
	}
	l = sqrt(ljnt);
	
	return l;
}


Vector3d Natural::sampleSphere()
{
	Vector3d nullVector;
	return nullVector;
}

void Natural::makeNaturalGrid()
{
	
}