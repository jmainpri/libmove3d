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
	kinos["HUMAN_ACHILE"] = Achile;
	
	m_KinType = kinos[m_Robot->getName()];
	
#ifdef HRI_PLANNER
	m_Agents = hri_create_agents();
#endif
	
	switch (m_KinType) 
	{
		case Justin:
			cout << "KinType of HRICS::Natural is Justin ( " << m_Robot->getName() << " ) "<< endl;
			initNaturalJustin();
			m_IndexObjectDof = m_Robot->getObjectDof();
			m_computeNbOfIK = true;
			break;
		
		case Achile:
			cout << "KinType of HRICS::Natural is Achile ( " << m_Robot->getName() << " ) "<< endl;
			initNaturalAchile();
			m_IndexObjectDof = NULL;
			m_computeNbOfIK = false;
			break;

		default:
			cout << "No proper robot has been selected in Natural cost function" << endl;
			break;
	}
	
	m_Grid = NULL;
	cout << "Object Dof is " << m_IndexObjectDof << endl;
}

Natural::~Natural()
{
	delete m_Grid;
}

void Natural::computeNaturalGrid()
{
	vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;
	
	m_Grid = new NaturalGrid(ENV.getDouble(Env::CellSize),envSize);
	m_Grid->setNaturalCostSpace(this);
	
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

const int ACHILE_JOINT_SPINE = 2;					//  2,  3,  4

const int ACHILE_JOINT_ARM_RIGTH_SHOULDER = 8;		//  8,  9, 10
const int ACHILE_JOINT_ARM_RIGTH_ELBOW = 11;		// 11
const int ACHILE_JOINT_ARM_RIGTH_WRIST = 12;		// 12, 13, 14

const int ACHILE_JOINT_ARM_LEFT_SHOULDER = 15;		// 15, 16, 17
const int ACHILE_JOINT_ARM_LEFT_ELBOW = 18;			// 18
const int ACHILE_JOINT_ARM_LEFT_WRIST = 19;			// 19, 20, 21 

void Natural::initNaturalAchile()
{
	configPt q;
	q = p3d_alloc_config(m_Robot->getRobotStruct());
	
	q[0] = 0;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
	q[4] = 0;
	q[5] = 0;
	
	q[6] = 0;
	q[7] = 0;
	q[8] = 0;
	q[9] = 0;
	q[10] = 0;
	q[11] = 0;
	
	q[12] = 0;
	q[13] = 0;
	q[14] = 0;
	q[15] = 0;
	q[16] = 0;
	q[17] = 0;
	q[18] = 1.33012;
	q[19] = 0.365646;
	q[20] = -0.12706;
	q[21] = 0.525519;
	q[22] = 0.17558;
	q[23] = -0.342085;
	q[24] = 0.0233874;
	q[25] = -1.22784;
	q[26] = 0.482584;
	q[27] = 0.00436332;
	q[28] = -0.368439;
	q[29] = -0.210487;
	q[30] = 0;
	q[31] = -0.0935496;
	q[32] = 0;
	q[33] = 0;
	q[34] = 0;
	q[35] = 0;
	q[36] = 0;
	q[37] = 0;
	q[38] = 0;
	q[39] = 0;
	q[40] = 0;
	q[41] = 0;
	q[42] = 0;
	q[43] = 0;
	q[44] = 0;
	q[45] = 0;
	
	
	m_q_Confort = shared_ptr<Configuration>(
	new Configuration(m_Robot,p3d_copy_config_deg_to_rad(m_Robot->getRobotStruct(),q)));
	
	
	
	
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
	//double max_taskdist;
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
 * Computes the cost for a 
 * Workspace point
 */
double Natural::getCost(const Vector3d& WSPoint)
{
	if(m_computeNbOfIK)
	{
		return getNumberOfIKCost(WSPoint);
	}
	
	bool leftArm = false;
	bool IKSucceded;
	double Cost = 0.0;
	
	// 2 - Select Task
	HRI_GIK_TASK_TYPE task;
	
	if (leftArm == true) 
	{
		task = GIK_LAREACH; // Left Arm GIK
	}
	else 
	{
		task = GIK_RAREACH; // Left Arm GIK
	}
	
	configPt q;
	
	p3d_vector3 Tcoord[3];
	Tcoord[0][0] = Tcoord[1][0] = Tcoord[2][0] = WSPoint[0];
	Tcoord[0][1] = Tcoord[1][1] = Tcoord[2][1] = WSPoint[1];
	Tcoord[0][2] = Tcoord[1][2] = Tcoord[2][2] = WSPoint[2];
	
	if(	m_Agents->humans_no > 0 ) // Humans
	{
		q = p3d_get_robot_config(m_Agents->humans[0]->robotPt);
		
		IKSucceded =  hri_agent_single_task_manip_move(m_Agents->humans[0], task, Tcoord, &q);
		
		if ( IKSucceded ) 
		{
			shared_ptr<Configuration> ptrQ(new Configuration(m_Robot,q));
			if( ptrQ->IsInCollision() )
			{
				IKSucceded = false;
			}
		}
	}
	else 
	if ( m_Agents->robots_no > 0) // Robots
	{
		q = p3d_get_robot_config(m_Agents->robots[0]->robotPt);
		IKSucceded = hri_agent_single_task_manip_move(m_Agents->robots[0], task, Tcoord, &q);
		//p3d_set_and_update_this_robot_conf(m_Agents->robots[0]->robotPt,q);
	}
	else 
	{
		cout << "Warning: No Agent for GIK" << endl;
	}
	
	if ( IKSucceded ) 
	{
		Cost = 100;
		//cout << "IK Succeded" << endl;
	}
	else 
	{
		Cost = 0.0;
		//cout << "IK Failed" << endl;
	}

	return Cost;
}

/*!
 * Computes the number of IK
 */
double Natural::getNumberOfIKCost(const Vector3d& WSPoint)
{
	shared_ptr<Configuration> q;
	
	double Cost = 0.0;
	const unsigned int NbDirections = 360;
	
	for (unsigned int i=0; i<NbDirections; i++) 
	{
		q = m_Robot->shoot();
		
		(*q)[m_IndexObjectDof+0] = WSPoint[0];
		(*q)[m_IndexObjectDof+1] = WSPoint[1];
		(*q)[m_IndexObjectDof+2] = WSPoint[2];
		
		//			q->getConfigStruct()[32] = 0.000000;
		//			q->getConfigStruct()[33] = 0.000000;
		//			q->getConfigStruct()[34] = -0.785398;
		
		if( q->setConstraintsWithSideEffect() && !q->IsInCollision() )
		{
			//m_Cost += grid->getNaturalCostSpace()->getCost();
			//cout << "Center :" << endl << center << endl;
			//cout << rob->getName() << endl;
			//m_QStored = q;
			//m_CostIsComputed = true;
			Cost += 1.0;
			//m_QStored->print(true);
			//return 1.0;
		}
	}
	
	return Cost;
}

/*!
 * Joint-displacement : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getJointDisplacement()
{
	shared_ptr<Configuration> q = m_Robot->getCurrentPos();
	return getCustomDistConfig(q);
}

/*!
 * Energy : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getEnergy()
{
	double Energy = 0.0;
	
	vector<double> DeltaHeigth = getHeigthFromConfort();
	
	for (unsigned int i=1; i<m_Robot->getNumberOfJoints(); i++) 
	{
		if( m_mg[i] > 0)
		{
			Energy += pow(m_mg[i],2)*pow(DeltaHeigth[i],2);
		}
	}
	
	return Energy;
}

/*!
 * Discomfort : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getDiscomfort()
{
	double Discomfort = 0.0;
	return Discomfort;
}

/*!
 * Discomfort : This function evaluates the heigth 
 * that each body makes with the confort position
 * from the VRS project at Iwoa
 */
vector<double> Natural::getHeigthFromConfort()
{
	vector<double> heigth;
	return heigth;
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
		
		for (j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			double W = (*m_q_ConfortWeigths)[jntPt->index_dof+j];
			
			double dof_dist = p3d_jnt_calc_dof_dist(jntPt, j, 
														m_q_Confort->getConfigStruct(), 
														q->getConfigStruct());
				//printf("dof_dist[%d] = %f\n",jntPt->index_dof + j,dof_dist);
				ljnt += W*SQR(dof_dist);
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