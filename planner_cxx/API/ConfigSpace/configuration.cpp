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
  _flagInitQuaternions(false),
  _CollisionTested(false),
  _InCollision(true),
  _CostTested(false),
  _Cost(0.0),
  _Robot(R)
{

    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        _Configuration = p3d_alloc_config(_Robot->getRobotStruct());
    }
}

Configuration::Configuration(Robot* R, configPt C, bool noCopy) :
  _flagInitQuaternions(false),
  _CollisionTested(false),
  _InCollision(true),
  _CostTested(false),
  _Cost(0.0),
  _Robot(R)
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
	  _Configuration = noCopy ? C : p3d_copy_config(_Robot->getRobotStruct(), C);
	  //            this->initQuaternions();
        }
    }
}

Configuration::Configuration(const Configuration& conf) :
  _flagInitQuaternions(conf._flagInitQuaternions),
  _CollisionTested(conf._CollisionTested),
  _InCollision(conf._InCollision),
  _CostTested(conf._CostTested),
  _Cost(conf._Cost),
  _Robot(conf._Robot)
{
    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        _Configuration = p3d_copy_config(_Robot->getRobotStruct(), conf._Configuration);
        //            this->initQuaternions();
    }
}

Configuration::~Configuration()
{
    this->Clear();
}

void Configuration::Clear()
{
    if(_Configuration != NULL)
    {
        p3d_destroy_config(_Robot->getRobotStruct(), _Configuration);
    }
}


void Configuration::convertToRadian()
{
    configPt q = p3d_alloc_config(_Robot->getRobotStruct());
    p3d_convert_config_deg_to_rad(_Robot->getRobotStruct(),_Configuration,&q);
    p3d_destroy_config(_Robot->getRobotStruct(),_Configuration);
    _Configuration = q;
}

shared_ptr<Configuration> Configuration::getConfigInDegree()
{
    configPt q = p3d_alloc_config(_Robot->getRobotStruct());
    p3d_convert_config_rad_to_deg(_Robot->getRobotStruct(),_Configuration,&q);
    return (shared_ptr<Configuration> (new Configuration(_Robot,q,true)));
}


//Accessors
Robot* Configuration::getRobot()
{
    return _Robot;
}

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

/*bool Configuration::isQuatInit()
{
    return _flagInitQuaternions;
}


Eigen::Quaterniond Configuration::getQuaternion()
{
    return _Quaternions;
}*/


/**
  * InitsQuaternion from eulers Angle
  */
/*void Configuration::initQuaternions()
{
    for(int i = 0; i <= (_Robot->getRobotStruct())->njoints; i++)
    {
        if( _Robot->getRobotStruct()->joints[i]->type == P3D_FREEFLYER)
        {
            _QuatDof = _Robot->getRobotStruct()->joints[i]->index_dof+3;

            Matrix3d m;
            m =     Eigen::AngleAxisd(_Configuration[_QuatDof+0], Vector3d::UnitX())
                    *   Eigen::AngleAxisd(_Configuration[_QuatDof+1], Vector3d::UnitY())
                    *   Eigen::AngleAxisd(_Configuration[_QuatDof+2], Vector3d::UnitZ());

            _Quaternions = Eigen::AngleAxisd(m);
        }
    }
    _flagInitQuaternions = true;
}*/

/**
  * InitsQuaternion from ExternQuaternion and indexDof
  */
/*void Configuration::initQuaternions(int quatDof,Eigen::Quaternion<double> quat)
{
    _QuatDof = quatDof;
    _Quaternions = quat;
    _flagInitQuaternions = true;
}*/


/**
* this conversion uses conventions as described on page:
*   http://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
*   Coordinate System: right hand
*   Positive angle: right hand
*   Order of euler angles: heading first, then attitude, then bank
*   matrix row column ordering:
*   [m00 m01 m02]
*   [m10 m11 m12]
*   [m20 m21 m22]*/
/*void Configuration::setQuaternionsToEuler()
{
    Matrix3d m = _Quaternions.toRotationMatrix();

    double* heading =   _Configuration+_QuatDof+0;
    double* attitude =  _Configuration+_QuatDof+1;
    double* bank =      _Configuration+_QuatDof+2;

    // Assuming the angles are in radians.
    if (m(1,0) > 0.998) // singularity at north pole
    {
        *heading = atan2(m(0,2),m(2,2));
        *attitude = M_PI_2;
        *bank = 0;
    }
    else if (m(1,0) < -0.998) // singularity at south pole
    {
        *heading = atan2(m(0,2),m(2,2));
        *attitude = -M_PI_2;
        *bank = 0;
    }
    else
    {
        *heading = atan2(-m(2,0),m(0,0));
        *bank = atan2(-m(1,2),m(1,1));
        *attitude = asin(m(1,0));
    }
}*/

/**
  * Computes the distance
  *  between configurations
  */
double Configuration::dist(Configuration& Conf)
{
    double ljnt = 0.;
    int njnt = _Robot->getRobotStruct()->njoints;
    p3d_jnt * jntPt;
    int* IsConstraintedDof = _Robot->getRobotStruct()->cntrt_manager->in_cntrt;

    bool ActivQuaterion = _flagInitQuaternions && Conf._flagInitQuaternions;

    for (int i = 0; i <= njnt; i++) {
        jntPt = _Robot->getRobotStruct()->joints[i];
        for (int j = 0; j < jntPt->dof_equiv_nbr; j++)
        {
            if (IsConstraintedDof[jntPt->index_dof + j] != DOF_PASSIF)
            {
                if ( ActivQuaterion && ( (jntPt->index_dof + j) >= _QuatDof ) && ( (jntPt->index_dof + j) < (_QuatDof+3) ))
                {

                }
                else
                {
                    ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, _Configuration, Conf.getConfigStruct()));
                }
            }
        }
    }


//    if(ActivQuaterion)
//    {
//        ljnt += SQR(_Quaternions.angularDistance(Conf._Quaternions));
//    }

    return sqrt(ljnt);;
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

bool Configuration::isOutOfBands()
{
  return(p3d_isOutOfBands(_Robot->getRobotStruct(), _Configuration, false));
}

shared_ptr<Configuration> Configuration::add(Configuration& C)
{
    shared_ptr<Configuration> ptrQ(new Configuration(_Robot));

    p3d_addConfig(_Robot->getRobotStruct(), _Configuration,
                  C.getConfigStruct(), ptrQ->getConfigStruct() );

    return ptrQ;
}

bool Configuration::setConstraints()
{
  Configuration q(_Robot,p3d_get_robot_config(_Robot->getRobotStruct()), true);

    bool respect = _Robot->setAndUpdate(*this);

    if(respect)
    {
        this->Clear();
        _Configuration = p3d_get_robot_config(_Robot->getRobotStruct());
    }

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

    configPt degConf = getConfigInDegree()->getConfigStruct();

    for (int i = 0; i < _Robot->getRobotStruct()->nb_dof; i++)
    {
        //	    cout << "q["<<i<<"]"<<" = "<< _Configuration[i] << endl;
        cout << "degConf["<< i <<"] = " << degConf[i] << endl;
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
