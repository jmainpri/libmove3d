// author: Romain Iehl <riehl@laas.fr>
#include "MLTRRT.hpp"
#include <tr1/memory>
#include <limits>
#include <boost/bind.hpp>
#include "costFunctions.hpp"
#include "passive_joints_zone.hpp"
#include "passive_sidechains_zone.hpp"
#include "passive_zone.hpp"
#include "ml_mechanical_functions.hpp"
#include "passive_shoot_functions.hpp"
#include "vector_operations.hpp"

#ifdef BIO_BALL
#include "ball_energy.hpp"
#endif

//bool TRRT(false);
bool TRRT(true);
bool lipaseOrtoTest(1);
bool exponentialCost(0);

using std::tr1::shared_ptr;

double changeTemperature(double temp, double fromCost, double toCost, bool accept)
{
  if(accept && toCost > fromCost)
  {
    return(temp / (1.0 + std::min(0.5, (toCost - fromCost) / 2.0)));
  }

  if(!accept && toCost > fromCost)
  {
    return(temp * exp(log(2.) / ENV.getDouble(Env::temperatureRate)));
  }

  return(temp);
}

double changeTemperatureLinear(double temp, double fromCost, double toCost, bool accept)
{
  const static double step(0.01);
  double tempStep(- step / log(0.5));
  if(accept)
  {
    return(temp - tempStep * ENV.getDouble(Env::temperatureRate));
  }
  else
  {
    return(temp + tempStep);
  }
}

double changeTemperatureExponential(double temp, double fromCost, double toCost, bool accept)
{
  double delta(exp(toCost - fromCost));
  double offset(1.0);
  double speedupFactor(0.1);
  if(accept)
  {
    return(temp / (offset + (speedupFactor * delta)));
  }
  else
  {
    return(temp * exp(log(2.) / ENV.getDouble(Env::temperatureRate)));
  }
}

bool costTestLinear(Node* N,
		    std::tr1::shared_ptr<Configuration> q,
		    double cost)
{
  double temperature(N->getNodeStruct()->comp->temperature);
  double c1(N->getConfiguration()->cost());
  double c2(cost);
  double delta(c1 - c2);
  double threshold(exp(delta / temperature));
  bool success(0.5 < threshold);
  return(success);
}

bool costTestExponential(Node* N,
			 std::tr1::shared_ptr<Configuration> q,
			 double cost)
{
  double temperature(N->getNodeStruct()->comp->temperature);
  double c1(N->getConfiguration()->cost());
  double c2(cost);
  double delta(exp(c1 - c2));
  double threshold(delta * temperature);
  //  std::cout << "from: " << c1 << " to: " << c2 << " p: " << threshold << std::endl;
  bool success(0.5 < threshold);
  return(success);
}

MLTRRT::MLTRRT(Robot* R, Graph* G) :
  Planner(R,G),
  fPassiveShoot(perturbCircularJoints_ShootOthers),
  fChangeTemperature(exponentialCost ?
		     changeTemperatureExponential :
		     //changeTemperatureLinear :
		     changeTemperature),
  mFilterMinDistance(10e6),
  ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
  MaxExpandNodeFailure(10),
  kNearestPercent(10),
  ExpansionDirectionMethod(GLOBAL_CS_EXP),
  IsDirSampleWithRlg(false),
  mPassiveZone(NULL),
  mTransitionExpansion(G)
{
}

MLTRRT::~MLTRRT()
{
  if(mPassiveZone)
  {
    delete(mPassiveZone);
  }
}

#include "Move3d-pkg.h"
int MLTRRT::init()
{
  if(lipaseOrtoTest)
  {
    //    init_ligand_autocol(13);
  }

  int ADDED = 0;
  Planner::init();

  ADDED += Planner::setStart(_Robot->getInitialPosition());
  ADDED += Planner::setGoal(_Robot->getGoTo());

  _Graph->setStart(_Start);
  _Graph->setGoal(_Goal);

  p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
			 this->getStart()->getNodeStruct(),
			 this->getGoal()->getNodeStruct());

  if(exponentialCost)
  {
    fCostTest = costTestExponential;
    //costTestLinear;
  }
  else
  {
    fCostTest = boost::bind(&TransitionExpansion::costTestSucceeded, &mTransitionExpansion, _1, _2, _3);
  }

  std::vector<int> mActiveObjIDs;
  
  if(ENV.getBool(Env::isCostSpace))
  {
    setCostFunction(costType::costMap2D);
  }
  //else if(TRRT)
  //{
    //setCostFunction(costType::passiveAndObstaclesAvoidance);
  //setCostFunction(costType::passiveAndObstaclesProximity);
  //}
  
  if(p3d_col_get_mode() == p3d_col_mode_bio)
  {
    mDistanceFilter = std::tr1::shared_ptr<LigandAtoms>(new LigandAtoms(_Robot));
    mDistanceFilterStop = std::tr1::shared_ptr<LigandAtoms>(new LigandAtoms(_Robot));
    _Robot->setAndUpdate(*this->getGoal()->getConfiguration());
    mDistanceFilter->setTarget(mDistanceFilter->geometricalCenter());
    _Robot->setAndUpdate(*this->getStart()->getConfiguration());
    mDistanceFilterStop->setTarget(mDistanceFilterStop->geometricalCenter());
    mPassiveZone = new PassiveSidechainsZone(_Robot->getRobotStruct());
    fGetCollidingJoints = bioGetCollidingPassiveJoints;
    fGetCurrentInvalidConf = bioGetCurrentInvalidConf;
    fResetCurrentInvalidConf = bioResetCurrentInvalidConf;
  }
  else
  {
    mPassiveZone = new PassiveJointsZone(_Robot, mActiveObjIDs, step());
    fGetCollidingJoints = getCollidingPassiveJoints;
    fGetCurrentInvalidConf = getCurrentInvalidConf;
    fResetCurrentInvalidConf = resetCurrentInvalidConf;
  }
  
  //  if(!ENV.getBool(Env::isCostSpace))
  if(TRRT)
  {
    this->getStart()->getConfiguration()->cost();
    this->getGoal()->getConfiguration()->cost();
  }

  return ADDED;
}

/**
 * Checks out that the plannification
 * problem fits such requirement
 */
bool MLTRRT::preConditions()
{
  std::cout << "Entering preCondition" << std::endl;

  if (ENV.getBool(Env::isCostSpace) && (ENV.getExpansionMethod()
					== Env::Connect))
  {
    std::cout
      << "Warning: Connect expansion strategy \
                is usually unadapted for cost spaces\n"
      << std::endl;
  }

  if ((ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal))
      && (*_Start->getConfiguration() == *_Goal->getConfiguration()) )
  {
    std::cout << "Tree Expansion failed: root nodes are the same" << std::endl;
    return false;
  }

  if (_Start->getConfiguration()->IsInCollision())
  {
    std::cout << "Start in collision" << std::endl;
    return false;
  }

  if (ENV.getBool(Env::expandToGoal)
      && _Goal->getConfiguration()->IsInCollision())
  {
    std::cout << "Goal in collision" << std::endl;
    return false;
  }

  if (ENV.getBool(Env::expandToGoal))
  {
    if(trajFound())
    {
      std::cout << "Start And Goal in same component" << std::endl;
      return true;
    }

    /*
    if(!ENV.getBool(Env::isCostSpace))
    {
      LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
      if (direct.getValid())
      {
	costConnectNodeToCompco(_Start,_Goal);
	std::cout << "Direct connection" << std::endl;
	return true;
      }
    }
    */
  }

  std::cout << "Tree Planner precondition: OK" << std::endl;
  return true;
}

/**
 * Checks out if the plannification
 * problem has reach its goals
 */
bool MLTRRT::checkStopConditions()
{
  if (ENV.getBool(Env::expandToGoal) && this->getStart()->getCompcoStruct()->num == //trajFound())
      this->getGoal()->getCompcoStruct()->num)
  {
    std::cout << "Success: the start and goal components are connected." << std::endl;
    return (true);
  }
  if (/*ENV.getBool(Env::ligandExitTrajectory)*/false)
  {
    double d(_Start->getConfiguration()->dist(
	       *_Graph->getLastnode()->getConfiguration()));
    if (d > 12.0)
    {
      ENV.setBool(Env::expandToGoal, true);
      _Goal = _Graph->getLastnode();
      _Graph->getGraphStruct()->search_goal = _Goal->getNodeStruct();
      _Goal->getNodeStruct()->rankFromRoot = 1;
      _Goal->getNodeStruct()->type = ISOLATED;
      _Robot->getGoTo() = _Goal->getConfiguration()->copy();
      std::cout << "Success: distance from start is " << d << std::endl;
      return (true);
    }
  }

  if (_Start->maximumNumberNodes())
  {
    std::cout
      << "Failure: the maximum number of nodes in the start component is reached."
      << std::endl;
    return (true);
  }

  if (ENV.getBool(Env::biDir))
  {
    if (_Goal->maximumNumberNodes())
    {
      std::cout
	<< "Failure: the maximum number of nodes in the goal component is reached."
	<< std::endl;
      return (true);
    }
  }

  if ((int) _Graph->getNodes().size() >= ENV.getInt(Env::maxNodeCompco))
  {
    std::cout << "Failure: the maximum number of nodes is reached." << std::endl;
    return (true);
  }

  if (!(*_stop_func)())
  {
    p3d_SetStopValue(true);
  }

  if (p3d_GetStopValue())
  {
    std::cout << "Tree expansion cancelled." << std::endl;
    return (true);
  }
  return (false);
}

bool MLTRRT::distanceStopCheck(Node* node)
{
  return(this->distanceStopCheck(node->getConfiguration()));
}

bool MLTRRT::distanceStopCheck(ConfigurationPtr conf)
{
  if(mDistanceFilterStop.get())
  {
    _Robot->setAndUpdate(*conf);
    double dist(mDistanceFilterStop->distanceToTarget());
    return(dist > 15.0);
  }
  return(false);
}

//------------------------------------------------------------------------------
// Try to connect node N to compco C in graph G
bool MLTRRT::costConnectNodeToComp(Graph& G,
				   Node* N,
				   Node* compNode)
{
  if(N->getCompcoStruct()->num == compNode->getCompcoStruct()->num)
  {
    PrintInfo(("Warning: trying to connect a node to its own componant\n"));
    return true;
  }
  
  Node* targetNode = G.nearestWeightNeighbour(compNode,
					      N->getConfiguration(),
					      false,
					      ACTIVE_CONFIG_DIST);
  
  if (targetNode == NULL) {
    PrintInfo(("Warning: failed to find a nearest node in the componant to connect to.\n"));
    return false;
  }

  LocalPath path(N->getConfiguration(), targetNode->getConfiguration());
  // check that the localpath is collision-free and cost acceptable,
  // then merge the comps.
  // Otherwise, returns false.
  return(
    //    path.length() < 10 &&
    path.getValid() &&
    this->isPathAcceptable(path, N->getCompcoStruct()->temperature) &&
    G.MergeComp(N, targetNode, path.length()));
}

/**
 * Main Function of the Tree Planner,
 * Bi-Directionality is handled here
 */
uint MLTRRT::run()
{
  //	std::cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << std::endl;
  if(!preConditions())
  {
    return 0;
  }

  int NbCurCreatedNodes = 0;
  int NbTotCreatedNodes = 0;

  Node* fromNode = _Start;
  Node* toNode = _Goal;

  while (!checkStopConditions())
  {
    ENV.setInt(Env::progress,(int)(_Graph->getNodes().size()/ENV.getInt(Env::maxNodeCompco)));
    //                std::cout << "progress = " << ENV.getInt(Env::progress) << std::endl;
    //                std::cout << (int)(_Graph->getNodes().size()/ENV.getInt(Env::maxNodeCompco)) << std::endl;
    //		std::cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << std::endl;
    // Do not expand in the case of a balanced bidirectional expansion,
    // if the components are unbalanced.
    if (!(ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced)
	  && (fromNode->getCompcoStruct()->nnode
	      > toNode->getCompcoStruct()->nnode + 2)))
    {
      // expand one way
      // one time (Main function of Tree like planners
      NbCurCreatedNodes = this->expandOneStep(fromNode,toNode);

      if (NbCurCreatedNodes > 0)
      {
	if(ENV.getBool(Env::drawGraph))
	{
	  (*_draw_func)();
	}

	if(ENV.getBool(Env::MLTRRTDistanceStop) &&
	   this->distanceStopCheck(_Graph->getLastnode()))
	{
	  _Goal = _Graph->getLastnode();
	  _Robot->setGoTo(*_Goal->getConfiguration());
	  std::cout << "Success: distance from start reached." << std::endl;
	  //	  ENV.setBool(Env::isRunning,false);
	  return (NbTotCreatedNodes);
	}

	NbTotCreatedNodes += NbCurCreatedNodes;

	if (ENV.getBool(Env::expandToGoal))
	{
	  // If it expands towards a goal
	  // Tries to link with local method
	  //                    std::cout << "Tries to connect a node to connected component" << std::endl;
	  //if(false)
	  if (!ENV.getBool(Env::biDir))
	  {
	    if(toNode == this->getGoal())
	    {
	      _Graph->getLastnode()->getConfiguration()->copyPassive(*this->getGoal()->getConfiguration());
	    }
	  }
	  //LocalPath p(_Graph->getLastnode()->getConfiguration(), toNode->getConfiguration());
	  //if(p.getValid())
	  static long unsigned count(0);
	  count++;
	  if(count % 10 && this->costConnectNodeToComp(*_Graph, _Graph->getLastnode(), toNode))
	  {
	    //_Graph->MergeComp(_Graph->getLastnode(), toNode, p.length());
	    std::cout << "Connected two components." << std::endl;
	  }
	}
      }
    }
    if (ENV.getBool(Env::biDir))
    {
      std::swap(fromNode, toNode);
    }
  }
  if (ENV.getBool(Env::drawGraph))
  {
    (*_draw_func)();
  }
  //  ENV.setBool(Env::isRunning,false);
  return (NbTotCreatedNodes);
}

bool MLTRRT::isPathAcceptable(LocalPath& path, double temperature)
{
  if(!TRRT) { return(true); }
  double param(0.);
  double lastCost = std::numeric_limits<double>::max();
  for(;;)
  {
    double cost = path.configAtParam(param)->cost();
    if(cost > lastCost)
    {
      if(p3d_random(0., 1.) > exp((lastCost - cost) / temperature))
      {
	return(false);
      }
      else
      {
	temperature = fChangeTemperature(temperature, lastCost, cost, false);
      }
    }
    lastCost = cost;
    if(param >= path.getParamMax()) {
      break;
    }
    param += step(); // / 3;
  }
  return(true);
}

std::vector<std::pair<double,double> > getPathMaxima(LocalPath& path)
{
  PathSegments segments(path.getParamMax(), std::max(10e-6, step()));
  assert(segments.nbSegments() > 0);
  std::vector<std::pair<double,double> > maxima;
  std::vector<double> costs;
  for(unsigned i(0); i <= segments.nbSegments(); i++)
  {
    if(i == 0) { costs.push_back(path.getBegin()->cost()); }
    else if(i == segments.nbSegments()) { costs.push_back(path.getEnd()->cost()); }
    else { costs.push_back(path.configAtParam(segments[i])->cost()); }
  }
  for(unsigned i(1); i < segments.nbSegments(); i++)
  {
    if(costs[i-1] < costs[i] && costs[i] > costs[i+1])
    {
      maxima.push_back(std::pair<double, double>(segments[i], costs[i]));
    }
  }
  return(maxima);
}

bool MLTRRT::isPathDescending(LocalPath& path)
{
  PathSegments segments(path.getParamMax(), std::max(10e-6, step()));
  assert(segments.nbSegments() > 0);
  double lastCost = std::numeric_limits<double>::max();
  for(unsigned i(0); i <= segments.nbSegments(); i++)
  {
    double cost(0);
    if(i == 0) { cost = path.getBegin()->cost(); }
    else if(i == segments.nbSegments()) { cost = path.getEnd()->cost(); }
    else { cost = path.configAtParam(segments[i])->cost(); }
    if(cost > lastCost + EPS6)
    {
      return(false);
    }
    lastCost = cost;
  }
  return(true);
}

std::vector<Node*> MLTRRT::getVisibleNodes(shared_ptr<Configuration> q, p3d_compco* comp)
{
  std::vector<Node*> visible;
  const std::vector<Node*>& nodes = _Graph->getNodes();
  for(unsigned i(0); i < nodes.size(); i++)
  {
    if(nodes[i]->getCompcoStruct()->num == comp->num)
    {
      LocalPath path(nodes[i]->getConfiguration(), q);
      if(path.getValid() && this->isPathDescending(path))
      {
	visible.push_back(nodes[i]);
	return visible;
      }
    }
  }
  return visible;
}

Node* MLTRRT::getBestVisibleNode(shared_ptr<Configuration> q, p3d_compco* comp)
{
  /*
  int bestIndex(-1);
  double bestLength(std::numeric_limits<double>::max());
  std::vector<Node*>& nodes = _Graph->getNodes();
  for(unsigned i(0); i < nodes.size(); i++)
  {
    if(nodes[i]->getCompcoStruct()->num == comp->num)
    {
      LocalPath path(nodes[i]->getConfiguration(), q);
      if(path.getValid() && this->isPathDescending(path))
      {
	if(path.length() < bestLength)
	{
	  bestLength = path.length();
	  bestIndex = i;
	}
	//return(nodes[i]);
      }
    }
  }
  //return NULL;
  return(bestIndex == -1 ? NULL : nodes[bestIndex]);
  */
  // set version :
  double cost = q->cost();
  std::set<std::pair<double, Node*>, compareNodes>::iterator it = mSortedNodes.upper_bound(
    std::pair<double, Node*>(cost, NULL));
  unsigned howMany(0);
  while(it != mSortedNodes.end())
  {
    howMany++;
    Node* node = it->second;
    if(node->getCompcoStruct()->num == comp->num)
    {
      LocalPath path(node->getConfiguration(), q);
      if(path.length() < 5.0 && path.getValid() && this->isPathDescending(path))
      {
	//std::cout << "tried " << howMany << " on " << mSortedNodes.size() << std::endl;
	return(node);
      }
    }
    it++;
  }
  //std::cout << "tried " << howMany << " on " << mSortedNodes.size() << std::endl;
  return NULL;
}

// expansionConf : start configuration
// joints : which joints to expand
// return : success
bool MLTRRT::expandJointsByPerturbation(Nameless& storage,
					shared_ptr<Configuration> expansionConf,
					std::vector<joint_t*> joints,
					double step,
					bool checkCost)
{
  bool success(false);
  int test(0);
  shared_ptr<Configuration> directionConf = expansionConf->copy();
  for(int i(0); i < ENV.getInt(Env::MaxPassiveExpand) && !success; i++)
  {
    test++;
    //shared_ptr<Configuration> directionConf = expansionConf->copy();
  
    // Modify the passive dofs.
    fPassiveShoot(*directionConf, joints);
    // and expand.
    success = this->expandProcessSimple(storage,
					expansionConf,
					directionConf,
					step,
					checkCost);
  }

  if(success)
  {
    //std::cout << "succeeded on try " << test << std::endl;
  }
  return(success);
}

bool MLTRRT::expandPassiveIteratively(Nameless& storage,
				      shared_ptr<Configuration> expansionConf,
				      double step,
				      std::vector<joint_t*> joints,
				      bool checkCost)
{
  bool success(true);
  unsigned nbPassiveExpansions(0);
  Configuration invalConf(_Robot);
  std::vector<joint_t*> expandedJoints;
  while(true)
  {
    // Check preconditions
    if(fGetCurrentInvalidConf(_Robot, invalConf))
    {
      // Get the colliding passive dofs and select those that have not been expanded yet.
      std::vector<joint_t*> collidingJoints = fGetCollidingJoints(_Robot, invalConf);
      joints.insert(joints.end(), collidingJoints.begin(), collidingJoints.end());
    }
    std::vector<joint_t*> newJoints = selectNewElements(joints, expandedJoints);
    // return if there is no new joint to expand, or the last iteration failed.
    if(!success || newJoints.size() == 0) {
      break;
    }
  
    success = this->expandJointsByPerturbation(storage,
					       storage.getExpansionConf(),
					       newJoints,
					       step,
					       checkCost);
    
    nbPassiveExpansions += success;
    joints.clear();
  }
  return(nbPassiveExpansions > 0);
}

void MLTRRT::adjustTemperature(Node* node, double fromCost, double toCost, bool accept)
{
  double temp = fChangeTemperature(node->getCompcoStruct()->temperature, fromCost, toCost, accept);
  node->getCompcoStruct()->temperature = temp;
  //  std::cout << "temp : " << temp << std::endl;
}

// The expansion is a success, if at least one (active or passive) expansion succeeded.
bool MLTRRT::expandProcess(Nameless& storage,
			   shared_ptr<Configuration> expansionConfig,
			   shared_ptr<Configuration> directionConfig)
{
  static const unsigned modeActiveOnly(0);
  static const unsigned modeZoneBefore(1);
  static const unsigned modeZoneAfter(2);
  //  const unsigned mode = modeZoneBefore;
  unsigned mode = this->manhattanSamplePassive() ? modeZoneBefore : modeActiveOnly;
  assert(mode <= modeZoneAfter);

  bool zoneExpSuccess(false);
  bool activeExpSuccess(false);
  bool passiveExpSuccess(false);

  switch(mode)
  {
  case modeActiveOnly:
    {
      // Copy the passive dofs.
      storage.getExpansionConf()->copyPassive(*directionConfig);
      
      // Perform expansion of active degrees from expansionConfig to directionConfig.
      activeExpSuccess = this->expandProcessSimple(storage,
						   storage.getExpansionConf(),
						   directionConfig,
						   step());
      
      // Env::isPasExtWhenAct : extend only when active expansion succeeded.
      // The performance seems better when Env::isPasExtWhenAct is false.
      if (ENV.getBool(Env::isPasExtWhenAct) && !activeExpSuccess) { goto end; }
      break;
    }
  case modeZoneBefore:
    {
      // get the joints in the expansion zone.
      std::vector<joint_t*> zoneJoints(mPassiveZone->joints(*storage.getExpansionConf()));
      // Perform zone expansion if some joints were found.
      if(zoneJoints.size() > 0)
      {
	//    zoneExpSuccess = this->expandJointsByPerturbation(storage,
	//						      storage.getExpansionConf(),
	//						      zoneJoints);
	// Avoid a nasty side effect from preceding localpath tests.
	fResetCurrentInvalidConf(_Robot);
	// Propagate the expansion iteratively, in case of collisions during the first expansion step.
	zoneExpSuccess = this->expandPassiveIteratively(storage,
							storage.getExpansionConf(),
							100.0,
							zoneJoints,
							true);
      }
      // Copy the passive dofs.
      storage.getExpansionConf()->copyPassive(*directionConfig);
      // Perform expansion of active degrees from expansionConfig to directionConfig.
      activeExpSuccess = this->expandProcessSimple(storage,
						   storage.getExpansionConf(),
						   directionConfig,
						   step());
      // Env::isPasExtWhenAct : extend only when active expansion succeeded.
      // The performance seems better when Env::isPasExtWhenAct is false.
      if (ENV.getBool(Env::isPasExtWhenAct) && !activeExpSuccess) { goto end; }
      // Now expand the passive joints.
      passiveExpSuccess = expandPassiveIteratively(storage,
						   storage.getExpansionConf(),
						   100); 
      break;
    }
  case modeZoneAfter:
    {
      // Copy the passive dofs.
      storage.getExpansionConf()->copyPassive(*directionConfig);
      // Perform expansion of active degrees from expansionConfig to directionConfig.
      activeExpSuccess = this->expandProcessSimple(storage,
						   storage.getExpansionConf(),
						   directionConfig,
						   step());
      // Env::isPasExtWhenAct : extend only when active expansion succeeded.
      // The performance seems better when Env::isPasExtWhenAct is false.
      if (ENV.getBool(Env::isPasExtWhenAct) && !activeExpSuccess) { goto end; }
      // get the joints in the expansion zone.
      std::vector<joint_t*> zoneJoints(mGetPassiveZoneJoints(*storage.getExpansionConf()));
      
      passiveExpSuccess = this->expandPassiveIteratively(storage,
							 storage.getExpansionConf(),
							 100,
							 zoneJoints);
      break;
    }
  default:
    assert(false);
  }
  
 end:
  // Return true if any of these steps succeeded.
  return (activeExpSuccess || passiveExpSuccess || zoneExpSuccess);
  //return (activeExpSuccess || passiveExpSuccess);
  // Works pretty bad.
  //return (activeExpSuccess);
}

bool MLTRRT::manhattanSamplePassive()
{
  return(ENV.getDouble(Env::manhatRatio) > p3d_random(0., 1.));
}

int MLTRRT::expandOneStep(Node* fromComp,Node* toComp)
{
  Node* directionNode(NULL);
  Node* expansionNode(NULL);
  shared_ptr<Configuration> directionConfig;
  // Store all the results in this thing.

  // Get the direction configuration.
  directionConfig = this->getExpansionDirection(fromComp,toComp,
						//false, directionNode);
						// sample the passive dofs
						true, directionNode);
  // Get the node from which the expansion is performed :
  // the closest neighbour of the direction configuration.
  expansionNode = this->getExpansionNode(fromComp,directionConfig,
					 ACTIVE_CONFIG_DIST);

  if(mDistanceFilter.get())
  {
    _Robot->setAndUpdate(*expansionNode->getConfiguration());
    double dist(mDistanceFilter->distanceToTarget());
    if(dist > mFilterMinDistance + 2.0)
    {
      return(0);
    }
    if(dist < mFilterMinDistance)
    {
      std::cout << "Minimal distance to the geometrical center of the ligand's goal position : " << std::endl;
      std::cout << "    " << dist << std::endl;
    }
    mFilterMinDistance = std::min(mFilterMinDistance, dist);
  }

  // Expand.
  Nameless storage(expansionNode, directionNode);
  if(this->expandProcess(storage,
			 expansionNode->getConfiguration(),
			 directionConfig))
  {
    //this->commitToGraph(storage);
    //this->commitToGraphDirectConnection(storage);
    if(TRRT)
    {
      this->commitToGraphCostConnection(storage);
    }
    else
    {
      this->commitToGraphDirectConnection2(storage);
    }
  }

  return(storage.nbCreatedNodes);
}

void MLTRRT::commitToGraph(Nameless& storage)
{
  Node* lastCreatedNode(NULL);
  for(unsigned i(0); i < storage.paths.size(); i++)
  {
    lastCreatedNode = this->addNode(i == 0 ? storage.expansionNode : lastCreatedNode,
				    *storage.paths[i],
				    i == storage.paths.size() - 1 &&
				    storage.directionConfigReached ?
				    storage.directionNode : NULL,
				    storage.nbCreatedNodes);
  }
}

void MLTRRT::commitToGraphDirectConnection(Nameless& storage)
{
  Node* fromNode(storage.expansionNode);
  for(unsigned i(0); i < storage.paths.size(); i++)
  {
    if(storage.paths.size() > 2)
    {
      shared_ptr<LocalPath> directPath(new LocalPath(storage.paths[i]->getBegin(), storage.paths[storage.paths.size()-1]->getEnd()));
      if(directPath->getValid())
      {
	//std::cout << "Succeeded direct path." << std::endl;
	this->addNode(fromNode,
		      *directPath,
		      storage.directionConfigReached ? storage.directionNode : NULL,
		      storage.nbCreatedNodes);
	break;
      }
      else
      {
	//std::cout << "Failed direct path." << std::endl;
      }
    }

    fromNode = this->addNode(fromNode,
			     *storage.paths[i],
			     i == storage.paths.size() - 1 &&
			     storage.directionConfigReached ?
			     storage.directionNode : NULL,
			     storage.nbCreatedNodes);
  }
}

void MLTRRT::commitToGraphDirectConnection2(Nameless& storage)
{
  Node* fromNode(storage.expansionNode);
  shared_ptr<Configuration> lastConfig = storage.paths[storage.paths.size()-1]->getEnd();
  shared_ptr<LocalPath> path(new LocalPath(fromNode->getConfiguration(), lastConfig));

  if(path->getValid())
  {
    //Node* newNode = 
    this->addNode(fromNode,
		  *path,
		  storage.directionConfigReached ? storage.directionNode : NULL,
		  storage.nbCreatedNodes);
  }
}

void MLTRRT::commitToGraphCostConnection(Nameless& storage)
{
  bool costVisible(false);
  Node* fromNode(storage.expansionNode);
  shared_ptr<Configuration> lastConfig = storage.paths[storage.paths.size()-1]->getEnd();
  double cost = lastConfig->cost();
  Node* seeingNode = this->getBestVisibleNode(lastConfig, fromNode->getCompcoStruct());
  if(seeingNode)
  {
    costVisible = true;
  }
  shared_ptr<LocalPath> path(new LocalPath(fromNode->getConfiguration(), lastConfig));
  bool costAccepted(false);

  if(!seeingNode)
  {
    if(path->getValid())
    {
      costAccepted = fCostTest(fromNode, path->getEnd(), cost);
      // std::cout << " adjusting : " << (costAccepted ? "accepted" : "rejected") << std::endl;
      this->adjustTemperature(fromNode, fromNode->getCost(), cost, costAccepted);
      costVisible = costAccepted;
      //costVisible = mTransitionExpansion.transitionTest(*fromNode, *path, cost);
      //if(!costVisible) { std::cout << "Transition test failed "  << fromNode->getCost() << " to " << cost << std::endl; }
    }
    else
    {
      // std::cout << "Invalid path" << std::endl;
    }
  }
  if(seeingNode || costVisible)
  {
    // Expand Control
    // LocalPath expandControlPath(storage.expansionNode->getConfiguration(),
    // 				storage.paths[storage.paths.size()-1]->getEnd());
    
    // if (ENV.getBool(Env::expandControl) && !mTransitionExpansion.expandControl(
    // 	  expandControlPath, 1.0, *storage.expansionNode))
    // {
    //   std::cout << "Rejected by expand control" << std::endl;
    //   return;
    // }

    std::vector<shared_ptr<Configuration> > configurations;
    configurations.push_back(fromNode->getConfiguration());

    std::vector<std::pair<double, double> > maxima = getPathMaxima(*path);

    if(maxima.size() > 0)
    {
      //      std::cout << "maxima : ";
      for(unsigned i(0); i < maxima.size(); i++)
      {
       	configurations.push_back(path->configAtParam(maxima[i].first));
      // 	std::cout << maxima[i].second;
      }
      // std::cout << std::endl;
    }

    configurations.push_back(path->getEnd());

    this->addNodes(storage.nbCreatedNodes, fromNode, configurations, storage.directionConfigReached ? storage.directionNode : NULL);

    // if(seeingNode) { std::cout << "Descending: " << fromNode->getCost() << " to " << cost << std::endl; }
    // if(!seeingNode)
    // {
    //   std::cout << "Increased : " << fromNode->getCost() << " to " << cost << std::endl;
    // }
  }
}

void MLTRRT::addNodes(unsigned& nbCreatedNode, Node* fromNode, std::vector<shared_ptr<Configuration> >& configurations, Node* lastNode)
{
  Node* previousNode(fromNode);

  for(unsigned i(0); i < configurations.size() - 1; i++)
  {
    LocalPath path(configurations[i], configurations[i+1]);
    Node* newNode = this->addNode(previousNode, path, i == configurations.size() - 2 ? lastNode : NULL, nbCreatedNode);
    double cost(configurations[i+1]->cost());
    //    newNode->setCost(cost);
    mSortedNodes.insert(std::pair<double, Node*>(cost, newNode));
    previousNode = newNode;
  }
}

void MLTRRT::expansionFailed(Node& node) {

  if(ExpansionNodeMethod == RANDOM_IN_SHELL_METH)  {
    p3d_SetNGood(0);
  }

  node.getNodeStruct()->n_fail_extend++;

  if((ENV.getBool(Env::discardNodes)) &&
     (node.getNodeStruct() != _Graph->getGraphStruct()->search_start) &&
     (node.getNodeStruct() != _Graph->getGraphStruct()->search_goal) &&
     (node.getNodeStruct()->n_fail_extend > MaxExpandNodeFailure)) {

    node.getNodeStruct()->IsDiscarded = true;
    update_parent_nfails(node.getNodeStruct());

    _Graph->getGraphStruct()->n_consec_pb_level ++;
    _Graph->getGraphStruct()->n_consec_fail_pb_level ++;
  }
}

Node* MLTRRT::addNode(Node* startNode,
		      LocalPath& path,
		      Node* endNode,
		      unsigned& nbCreatedNodes)
{
  if (endNode)
  {
    std::cout << "MergeComp" << std::endl;
    _Graph->MergeComp(startNode, endNode, path.length());
    return(endNode);
  }
  else
  {
    //    std::cout << "start edges before " << this->getStart()->getNodeStruct()->cost << " : " << this->getStart()->getNodeStruct()->nedge << std::endl;
    Node* newNode = _Graph->insertNode(startNode, path);
    //    std::cout << "start edges after " << this->getStart()->getNodeStruct()->cost << " : " << this->getStart()->getNodeStruct()->nedge << std::endl;
    nbCreatedNodes++;
    return(newNode);
  }
}

double step()
{
  return(p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep));
}

bool extend(ExtensionStepData& data, Env::expansionMethod method)
{
  switch(method)
  {
  case Env::Connect:
    return(extendConnect(data));
    break;
  default:
    return(extendFixedStep(data));
  }
}

bool extendFixedStep(ExtensionStepData& data)
{
  // Compute the path parameter : step/(path length)
  data.interpolation = data.directionPath->length() == 0. ? 1. : MIN(1., step() / data.directionPath->length());
  // Create the new extension configuration and validate its constraints.
  shared_ptr<Configuration> extensionConf =
    data.directionPath->configAtParam(data.interpolation * data.directionPath->getLocalpathStruct()->range_param);
  // TODO : verify this. And handle the possible failure to validate the constraints ?
  extensionConf->setConstraints();
  data.extensionPath = shared_ptr<LocalPath>(new LocalPath(data.directionPath->getBegin(), extensionConf));
  return(data.extensionPath->getValid());
}

bool extendConnect(ExtensionStepData& data)
{
  data.extensionPath = shared_ptr<LocalPath>(new LocalPath(*data.directionPath, data.interpolation));
  return(data.interpolation > 0.);
}

bool extendBoundedConnect(ExtensionStepData& data, double length)
{
  shared_ptr<LocalPath> boundedDirectionPath;
  if(data.directionPath->length() > length)
  {
    double param = data.directionPath->getParamMax() * length / data.directionPath->length();
    boundedDirectionPath = shared_ptr<LocalPath>(
      new LocalPath(data.directionPath->getBegin(),
		    data.directionPath->configAtParam(param)));
  }
  else
  {
    boundedDirectionPath = data.directionPath;
  }
  data.extensionPath = shared_ptr<LocalPath>(new LocalPath(*boundedDirectionPath, data.interpolation));
  // Fix this for the goal case, add as a parameter the minimum length
  //return(data.interpolation > 0.2);
  return(data.interpolation > 0);
}

shared_ptr<Configuration> MLTRRT::getExpansionDirection(
  Node* expandComp, Node* goalComp, bool samplePassive,
  Node*& directionNode)
{

  if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
  {

    shared_ptr<Configuration> q = _Graph->getRobot()->shootDir(
      samplePassive);

    p3d_addConfig(_Graph->getRobot()->getRobotStruct(),
		  q->getConfigStruct(),
		  expandComp->getCompcoStruct()->dist_nodes->N->q,
		  q->getConfigStruct());

    return (q);

  }

  shared_ptr<Configuration> q;
  int savedRlg(0);

  if (IsDirSampleWithRlg)
  {
    // Save the previous Rlg setting to shoot without Rlg
    savedRlg = p3d_get_RLG();
    p3d_set_RLG(false);
  }

  // Selection in the entire CSpace and
  // biased to the Comp of the goal configuration
  if (ENV.getBool(Env::isGoalBiased) && p3d_random(0., 1.) <= ENV.getDouble(Env::Bias))
  {
    // select randomly a node in the goal component as direction of expansion
    directionNode = _Graph->randomNodeFromComp(goalComp);
    q = directionNode->getConfiguration();
  }
  else
  {
    switch (ExpansionDirectionMethod)
    {
    case SUBREGION_CS_EXP:
      // Selection in a subregion of the CSpace
      // (typically close to the current tree)
      // and  biased to the goal configuration
      q = shared_ptr<Configuration> (
	new Configuration(_Graph->getRobot()));

      p3d_shoot_inside_box(_Graph->getRobot()->getRobotStruct(),
			   /*expandComp->getConfiguration()->getConfigStruct(),*/
			   q->getConfigStruct(), expandComp->getCompcoStruct()->box_env_small,
			   (int) samplePassive);
      break;

    case GLOBAL_CS_EXP:
    default:
      // Selection in the entire CSpace
      q = _Graph->getRobot()->shoot(samplePassive);
    }
  }
  if (IsDirSampleWithRlg)
  {
    //Restore the previous Rlg setting
    p3d_set_RLG(savedRlg);
  }
  return (q);
}

Node* MLTRRT::getExpansionNode(Node* compNode, shared_ptr<Configuration> direction, int distance)
{
  //    std::cout << "Distance == " << distance << std::endl;

  if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
  {
    return _Graph->getNode(compNode->getCompcoStruct()->dist_nodes->N);
  }

  int KNearest = -1;
  int NearestPercent;

  switch (distance)
  {

  case NEAREST_EXP_NODE_METH:
    /* Choose the nearest node of the componant*/
    return (_Graph->nearestWeightNeighbour(compNode, direction,
					   p3d_GetIsWeightedChoice(), distance));

  case K_NEAREST_EXP_NODE_METH:
    /* Select randomly among the K nearest nodes of a componant */
    NearestPercent = kNearestPercent;
    KNearest
      = MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
    // TODO : fix
    //   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
    // KNearest);

    return (_Graph->nearestWeightNeighbour(compNode, direction,
					   p3d_GetIsWeightedChoice(), distance));

  case BEST_SCORE_EXP_METH:
    /* Select the node which has the best score: weight*dist */
    return (_Graph->nearestWeightNeighbour(compNode, direction,
					   p3d_GetIsWeightedChoice(), distance));

  case K_BEST_SCORE_EXP_METH:
    NearestPercent = kNearestPercent;
    KNearest
      = MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
    // TODO : fix
    // ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
    return (_Graph->nearestWeightNeighbour(compNode, direction,
					   p3d_GetIsWeightedChoice(), distance));

  case RANDOM_IN_SHELL_METH:
    /* Select randomly among all the nodes inside a given portion of shell */
    return (_Graph->getNode(hrm_selected_pb_node(_Graph->getGraphStruct(),
						 direction->getConfigStruct(), compNode->getNodeStruct()->comp)));

  case RANDOM_NODE_METH:
    return (_Graph->getNode(p3d_RandomNodeFromComp(
			      compNode->getCompcoStruct())));

  default:
    /* By default return the nearest node of the componant */
    return (_Graph->nearestWeightNeighbour(compNode, direction,
					   p3d_GetIsWeightedChoice(), distance));
  }
}

bool MLTRRT::expandProcessSimple(Nameless& storage,
				 shared_ptr<Configuration> expansionConfig,
				 shared_ptr<Configuration> directionConfig,
				 double step,
				 bool checkCost)
{
  bool extensionSuccess(false);
  //  bool expansionControlSuccess(true);
  bool directionReached(false);
  shared_ptr<Configuration> fromConfig(expansionConfig);
  unsigned nbIteration(0);
  bool result(false);

  // Perform extension toward directionConfig
  // Additional nodes creation in the nExtend case, but without checking for expansion control
  do
  {
    nbIteration++;

    ExtensionStepData data(shared_ptr<LocalPath>(new LocalPath(fromConfig, directionConfig)));

    // Extend
    //    extensionSuccess = extendConnect(data);
    extensionSuccess = extendBoundedConnect(data, step);
    directionReached = data.interpolation == 1.;

    // Expansion Control
    //    if (nbIteration == 1 && extensionSuccess && ENV.getBool(Env::expandControl))
    //    {
    //      expansionControlSuccess = this->expandControl(data, expansionNode->getCompcoStruct());
    //    }

    // success
    if(extensionSuccess)
    {
      if(TRRT && checkCost)
      {
	_Robot->setAndUpdate(*data.extensionPath->getBegin());
	//double fromCost = computeCost();
	double toCost = data.extensionPath->getEnd()->cost();
	//	if(toCost > fromCost) {
	if(!fCostTest(storage.expansionNode, data.extensionPath->getEnd(), toCost)) {
	  return(false);
	}
      }

      storage.paths.push_back(data.extensionPath);
      storage.directionConfigReached = directionReached;
      
      fromConfig = data.extensionPath->getEnd();
      result = true;
    }
    // failure
    else
    {
      if (nbIteration == 1)
      {
	// this->expansionFailed(*expansionNode);
	result = false;
      }
    }
    // keep iterating if the extention method is n extend, and the target is not reached
    // but the current extension step succeeded.
  } while (false //|| extensionSuccess);// && expansionControlSuccess &&
	   //method == Env::nExtend && !directionReached
  );

  return(result);
}

bool MLTRRT::expandToGoal(Node* expansionNode,
			  std::tr1::shared_ptr<Configuration> directionConfig)
{
  return false;
}
