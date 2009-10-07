//
// C++ Implementation: configuration
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "../planningAPI.hpp"

using namespace std;
using namespace tr1;

//constructor and destructor
Configuration::Configuration(Robot* R) :
	_CollisionTested(false),
	_InCollision(true),
	_CostTested(false),
	_Cost(0.0)
{
	_Robot = R;
	flagInitQuaternions = false;

	if(_Robot==NULL)
	{
		_Configuration = NULL;
	}
	else
	{
		_Configuration = p3d_alloc_config(_Robot->getRobotStruct());
	}
}

Configuration::Configuration(Robot* R, configPt C) :
	_Robot(R),
	flagInitQuaternions(false),
	_CollisionTested(false),
	_InCollision(true),
	_CostTested(false),
	_Cost(0.0)
{
	if(_Robot==NULL)
	{
		_Configuration = NULL;
	}
	else
	{
		if(C==NULL)
		{
			_Configuration = C;
		}
		else
		{
			_Configuration = p3d_copy_config(_Robot->getRobotStruct(), C);
			this->initQuaternions();
		}
	}

}

Configuration::~Configuration()
{
	//  _VectQuaternions.clear();
	this->Clear();
}

void Configuration::Clear()
{
	if(_Configuration != NULL)
	{
		p3d_destroy_config(_Robot->getRobotStruct(), _Configuration);
	}
}

//Accessors
Robot* Configuration::getRobot()
{
	return _Robot;
}

//vector<Gb_quat*> Configuration::getQuat()
//{
//  return _VectQuaternions;
//}

configPt Configuration::getConfigStruct()
{
	return _Configuration;
}

void Configuration::setConfiguration(configPt C)
{
	_Configuration = C;
}

void Configuration::setConfiguration(Configuration& C)
{
	_Configuration = C.getConfigStruct();
}

bool Configuration::isInint()
{
	return flagInitQuaternions;
}

void Configuration::initQuaternions()
{
	//  for(int i = 0; i <= (_Robot->getRobotStruct())->njoints; i++) {
	//    if((_Robot->getRobotStruct())->joints[i]->type == P3D_FREEFLYER) {
	//      Gb_quat quat;
	//      //this->initQuaternions((_Robot->get_robot())->joints[i], _Configuration, &quat);//init qt a revoir
	//      _VectQuaternions.push_back(&quat);
	//    }
	//  }
	flagInitQuaternions = true;
}

double Configuration::dist(Configuration& Conf)
{
	return p3d_dist_config(_Robot->getRobotStruct(), _Configuration,
			Conf.getConfigStruct());
}

double Configuration::dist(Configuration& q, int distChoice)
{
	switch (distChoice)
	{
	case ACTIVE_CONFIG_DIST:
		return (p3d_ActiveDistConfig(_Robot->getRobotStruct(), _Configuration,
				q.getConfigStruct()));
		//  case LIGAND_PROTEIN_DIST:
		//    return(bio_compute_ligand_dist(_Robot->getRobotStruct(), _Configuration, q.getConfigurationStruct()));
		//    break;
	case MOBILE_FRAME_DIST:
		cout
				<< "Warning: the MOBILE_FRAME_DIST can't be directly returned from the configurations"
				<< endl;
		// hrm_mob_frame_dist(robotPt, mob_frame_ref,ListNode->N->rel_mob_frame);
	case GENERAL_CSPACE_DIST:
	default:
		return (this->dist(q));
	}
}

bool Configuration::IsInCollision()
{
	if(!_CollisionTested)
	{
		this->getRobot()->setAndUpdate(*this);
		_CollisionTested = true;
		_InCollision = p3d_col_test();
	//	return p3d_col_test_robot(_Robot->getRobotStruct(), JUST_BOOL);
		return _InCollision;
	}
	else
	{
		return _InCollision;
	}
}

double Configuration::distEnv()
{
	this->getRobot()->setAndUpdate(*this);
	int settings = get_kcd_which_test();
	set_kcd_which_test((p3d_type_col_choice) (40 + 3));
	// 40 = KCD_ROB_ENV
	// 3 = DISTANCE_EXACT
	p3d_col_test_choice();
	// Collision detection with other robots only

	int nof_bodies = _Robot->getRobotStruct()->no;

	double* distances = new double[nof_bodies];

	p3d_vector3 *body = new p3d_vector3[nof_bodies];
	p3d_vector3 *other = new p3d_vector3[nof_bodies];

	 p3d_kcd_closest_points_robot_environment(_Robot->getRobotStruct(),
			 body,other,distances);
	// Get robot closest points to human for each body

	set_kcd_which_test((p3d_type_col_choice) settings);

	int i = (int)(std::min_element(distances,distances+nof_bodies-1 )-distances);

	return distances[i];
}

bool Configuration::equal(Configuration& Conf)
{
	if(_Configuration==Conf.getConfigStruct())
	{
		if(_Configuration==NULL)
		{
			return true;
		}
	}
	else
	{
		if(_Configuration==NULL || Conf.getConfigStruct()==NULL)
		{
			return false;
		}
	}
	return (p3d_equal_config(_Robot->getRobotStruct(), _Configuration,
			Conf.getConfigStruct()));
}

//copie la Configuration courante dans une nouvelle Configuration
shared_ptr<Configuration> Configuration::copy()
{
	return (shared_ptr<Configuration> (new Configuration(_Robot,
			p3d_copy_config(_Robot->getRobotStruct(), _Configuration))));
}

void Configuration::copyPassive(Configuration& C)
{
	for (int i(0); i <= _Robot->getRobotStruct()->njoints; i++)
	{
		p3d_jnt* joint(_Robot->getRobotStruct()->joints[i]);
		for (int j(0); j < joint->dof_equiv_nbr; j++)
		{
			int k = joint->index_dof + j;
			if ((!p3d_jnt_get_dof_is_user(joint, j))
					|| (!p3d_jnt_get_dof_is_active_for_planner(joint, j)))
				C.getConfigStruct()[k] = this->getConfigStruct()[k];
		}
	}
}

shared_ptr<Configuration> Configuration::add(Configuration& C)
{
	configPt q;
	p3d_addConfig(_Robot->getRobotStruct(), _Configuration,
			C.getConfigStruct(), q);
	return (shared_ptr<Configuration> (new Configuration(_Robot, q)));
}

bool Configuration::setConstraints()
{
	Configuration q(_Robot,p3d_get_robot_config(_Robot->getRobotStruct()));

	this->Clear();
	bool respect = _Robot->setAndUpdate(*this);
	_Configuration = p3d_get_robot_config(_Robot->getRobotStruct());

	_Robot->setAndUpdate(q);

	return respect;
}

double Configuration::cost()
{
	if(!_CostTested)
	{
		_Cost = p3d_GetConfigCost(_Robot->getRobotStruct(), _Configuration);
		_CostTested = true;
		return _Cost;
	}
	else
	{
		return _Cost;
	}
}

void Configuration::print()
{

	cout << "Print Configuration; Robot: " << _Robot->getRobotStruct() << endl;

	//	print_config(_Robot->getRobotStruct(),_Configuration);

	configPt degConf = p3d_alloc_config(_Robot->getRobotStruct());

	p3d_convert_config_rad_to_deg(_Robot->getRobotStruct(), _Configuration,
			&degConf);

	for (int i = 0; i < _Robot->getRobotStruct()->nb_dof; i++)
	{
		//	    cout << "q["<<i<<"]"<<" = "<< _Configuration[i] << endl;
		cout << degConf[i] << " ";
	}

	//	int nb_dof;
	//
	//	if(robotPt != NULL){
	//		nb_dof = mR->getP3dRob()->nb_user_dof;
	//	}

	//	for(int i=0; i<nb_dof;i++){
	//		PrintInfo(("q[%d] = %f\n", i, q[i]));
	//	}


	//	int njnt = mR->getP3dRob()->njoints, k;
	//
	//	p3d_jnt * jntPt;
	//
	//	for(int i=0; i<=njnt; i++) {
	//
	//		jntPt = mR->getP3dRob()->joints[i];
	//
	//		for(int j=0; j<jntPt->dof_equiv_nbr; j++) {
	//
	//			k = jntPt->index_dof + j;
	//
	//			if (p3d_jnt_get_dof_is_user(jntPt, j) /*&&
	//					 (p3d_jnt_get_dof_is_active_for_planner(jntPt,j) */) {
	//				cout << "q["<<k<<"] = "<<mQ[k]<<endl;
	//
	//			}
	//		}
	//	}
	cout << "\n--------------------------------" << endl;
}
