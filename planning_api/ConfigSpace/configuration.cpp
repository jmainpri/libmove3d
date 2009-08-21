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
Configuration::Configuration(Robot* R)
{
  _Robot = R;
  flagInitQuaternions = false;
  _Configuration = p3d_alloc_config(_Robot->getRobotStruct());
}

Configuration::Configuration(Robot* R, configPt C)
{
  _Robot = R;
  flagInitQuaternions = false;
  _Configuration = p3d_copy_config(_Robot->getRobotStruct() , C);

  this->initQuaternions();
}

Configuration::~Configuration()
{
//  _VectQuaternions.clear();
  this->Clear();
}

void Configuration::Clear()
{
  p3d_destroy_config(_Robot->getRobotStruct(),_Configuration);
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
  return p3d_dist_config(_Robot->getRobotStruct(), _Configuration, Conf.getConfigStruct());
}

double Configuration::dist(Configuration& q, int distChoice)
{
  switch(distChoice) {
  case ACTIVE_CONFIG_DIST:
    return(p3d_ActiveDistConfig(_Robot->getRobotStruct(), _Configuration, q.getConfigStruct()));
//  case LIGAND_PROTEIN_DIST:
//    return(bio_compute_ligand_dist(_Robot->getRobotStruct(), _Configuration, q.getConfigurationStruct()));
//    break;
  case MOBILE_FRAME_DIST:
    cout << "Warning: the MOBILE_FRAME_DIST can't be directly returned from the configurations" << endl;
    // hrm_mob_frame_dist(robotPt, mob_frame_ref,ListNode->N->rel_mob_frame);
  case GENERAL_CSPACE_DIST:
  default:
    return(this->dist(q));
  }
}

bool Configuration::IsInCollision()
{
  this->getRobot()->setAndUpdate(*this);
  return p3d_col_test();
}

bool Configuration::equal(Configuration& Conf)
{
  return(p3d_equal_config(_Robot->getRobotStruct(), _Configuration, Conf.getConfigStruct()));
}

//copie la Configuration courante dans une nouvelle Configuration
shared_ptr<Configuration> Configuration::copy()
{
  return(shared_ptr<Configuration>(new Configuration(_Robot, p3d_copy_config(_Robot->getRobotStruct(), _Configuration))));
}

void Configuration::copyPassive(Configuration& C)
{
  for(int i(0); i <= _Robot->getRobotStruct()->njoints; i++)
  {
    p3d_jnt* joint(_Robot->getRobotStruct()->joints[i]);
    for(int j(0); j < joint->dof_equiv_nbr; j++)
    {
      int k = joint->index_dof + j;
      if((!p3d_jnt_get_dof_is_user(joint, j)) ||
	 (!p3d_jnt_get_dof_is_active_for_planner(joint, j)))
	C.getConfigStruct()[k] = this->getConfigStruct()[k];
    }
  }
}

shared_ptr<Configuration> Configuration::add(Configuration& C)
{
  configPt q;
  p3d_addConfig(_Robot->getRobotStruct(), _Configuration, C.getConfigStruct(), q);
  return (shared_ptr<Configuration>(new Configuration(_Robot, q)));
}

double Configuration::cost()
{
  return(p3d_GetConfigCost(_Robot->getRobotStruct(), _Configuration));
}

bool Configuration::costTestSucceeded(Node* previousNode, double Step) {

  double currentCost = this->cost();
  p3d_node* previousNodePt(previousNode->getNodeStruct());
  double ThresholdVal;
  double dist;
  bool success(false);
  configPt previousConfig = previousNodePt->q;
  double temperature, cVertex,cOpt,cMax ;
  double minThreshold = 0.05;

  Node* Start = previousNode->getGraph()->getNode(previousNode->getGraph()->getGraphStruct()->search_start);
  Node* Goal = previousNode->getGraph()->getNode(previousNode->getGraph()->getGraphStruct()->search_goal);

  switch(p3d_GetCostMethodChoice()) {
  case MAXIMAL_THRESHOLD:
    // Literature method:
    // the condition test is only based on
    // an absolute cost which increase continuously.
    success = currentCost < p3d_GetCostThreshold();
    break;
  case URMSON_TRANSITION:
    //TODO !
    cVertex = p3d_ComputeUrmsonNodeCost(previousNode->getGraph()->getGraphStruct(), previousNodePt);
    cOpt = Start->getNodeStruct()->comp->minCost * (Start->getConfiguration()->dist(*Goal->getConfiguration(), p3d_GetDistConfigChoice())+1) / (2. * Step);
    cMax = Start->getNodeStruct()->comp->maxUrmsonCost;
    ThresholdVal = 1-(cVertex - cOpt)/(cMax - cOpt);
    ThresholdVal = MAX(ThresholdVal, minThreshold);

    success = p3d_random(0.,1.) < ThresholdVal;

    break;
    //the same part is used for TRANSITION_RRT and
    //MONTE_CARLO_SEARCH
  case TRANSITION_RRT:
  case TRANSITION_RRT_CYCLE:
  case MONTE_CARLO_SEARCH:

    // IsRaisingCost is FALSE when we are expanding the InitComp:
    // Downhill slopes have better chances to be accepted. If
    // we are expanding the GoalComp tests are inversed: Uphill
    // slopes have better chances to be accepted
    // update: this doesn't work. Inverting the acceptance test means that
    // the tree will grow towards the maxima, whereas it should follow the saddle points


    //new simplified test for down hill slopes
    if(currentCost <= previousNodePt->cost) {
      return true;
    }


    // In principle, the distance are not necessarly
    // reversible for non-holonomic robots
    dist = p3d_dist_q1_q2(_Robot->getRobotStruct(), _Configuration, previousConfig);

    // get the value of the auto adaptive temperature.
    temperature = p3d_GetIsLocalCostAdapt() ?
      previousNodePt->temp :
      previousNodePt->comp->temperature;

  //  if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      temperature = 0.001*ENV.getDouble(Env::alpha)*ENV.getInt(Env::maxCostOptimFailures);
   // }
    /*Main function to test if the next config
      will be selected as a new node.
      The TemperatureParam adjust itself automatically
      in function of the fails and successes of extensions*/


    //Metropolis criterion (ie Boltzman probability)
    //    ThresholdVal = exp((PreviousCost-CurrentCost)/(temperature*dist));
    ThresholdVal = exp((previousNodePt->cost-currentCost)/temperature);

    // success = ThresholdVal > 0.5;
    success = p3d_random(0.,1.) < ThresholdVal;
    if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {
      break;
    }

  }
  return success;
}

