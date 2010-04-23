#include "costFunctions.hpp"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <map>
#include "Collision-pkg.h"
#include "robot.hpp"
#include "groundWrapper.h"
#include <algorithm>
#include "configuration.hpp"
#ifdef BIO
#include "MLTRRT/passive_sidechains_zone.hpp"
#include "ball_energy.hpp"
#endif

extern void* GroundCostObj;

boost::function<double(Configuration&)> costFunction;
std::map<costType::t, boost::function<double(Configuration&)> > costFunctionMap;

double computeIntersectionWithGround(Configuration& conf)
{
  double cost(0);
  if(GroundCostObj)
  {
    GHintersectionVerticalLineWithGround(GroundCostObj, conf.getConfigStruct()[6],
					   conf.getConfigStruct()[7], &cost);
  }
  return(cost);
}

class ActivePassivePartition
{
public:
  ActivePassivePartition();
  void init(Robot* R);
  
  std::vector<p3d_jnt*>& getActive()
  {
    return(mActiveSet);
  }
  std::vector<p3d_jnt*>& getPassive()
  {
    return(mPassiveSet);
  }

protected:
  std::vector<p3d_jnt*> mActiveSet;
  std::vector<p3d_jnt*> mPassiveSet;
};

ActivePassivePartition::ActivePassivePartition()
{
}

template<class Container>
void pushBackIfValid(Container& l, p3d_jnt* j)
{
  if (j->o == NULL) {
    std::cout << "Joint " << j->name << " has no associated object." << std::endl;
  }
  else {
    l.push_back(j);
  }
}

void ActivePassivePartition::init(Robot* R)
{
  mActiveSet.clear();
  mPassiveSet.clear();
  // Separate the joints/objects into the active ones and passive ones.
  // <= seems necessary, due to crappy old unused joint[0]
  for(int i(0); i <= R->getRobotStruct()->njoints; i++)
  {
    p3d_jnt* jntPt = R->getRobotStruct()->joints[i];
    for(int j=0; j<jntPt->dof_equiv_nbr; j++)
    {
      if(p3d_jnt_get_dof_is_user(jntPt, j))
      {
	if(p3d_jnt_get_dof_is_active_for_planner(jntPt,j))
	{
	  if(std::find(mActiveSet.begin(), mActiveSet.end(), jntPt) == mActiveSet.end())
	  {
	    std::list<p3d_jnt*> todo;
	    pushBackIfValid(todo, jntPt);
	    while(todo.size() > 0)
	    {
	      for(int k(0); k < todo.front()->n_next_jnt; k++)
	      {
		pushBackIfValid(todo, todo.front()->next_jnt[k]);
	      }
	      mActiveSet.push_back(todo.front());
	      todo.pop_front();
	    }
	  }
	}
	else if(std::find(mActiveSet.begin(), mActiveSet.end(), jntPt) == mActiveSet.end() &&
		std::find(mPassiveSet.begin(), mPassiveSet.end(), jntPt) == mPassiveSet.end())
	{
	  pushBackIfValid(mPassiveSet, jntPt);
	}
      }
    }
  }
}

ActivePassivePartition partition;

double passiveAndObstaclesMinDist(Configuration& q)
{
  std::vector<p3d_jnt*>& activeSet = partition.getActive();
  std::vector<p3d_jnt*>& passiveSet = partition.getPassive();
  q.getRobot()->setAndUpdate(q);
  double minDist(std::numeric_limits<double>::max());
  for(unsigned i(0); i < activeSet.size(); i++) {
    for(unsigned j(0); j < passiveSet.size(); j++) {
      minDist = std::min(minDist, p3d_distanceObjToObj(activeSet[i]->o, passiveSet[j]->o));
    }
    minDist = std::min(minDist, p3d_distanceObjToEnv(activeSet[i]->o));
  }
  return(minDist);
}

double passiveAndObstaclesAvoidance(Configuration& q)
{
  return(5 / (passiveAndObstaclesMinDist(q) / ENV.getDouble(Env::extensionStep) * p3d_get_env_dmax()));
}

double passiveAndObstaclesProximity(Configuration& q)
{
  return(5 * (passiveAndObstaclesMinDist(q) / ENV.getDouble(Env::extensionStep) * p3d_get_env_dmax()));
}

#ifdef BIO
#include "Bio-pkg.h"
PassiveSidechainsZone* passiveSidechainsZone(0);

/*!
  Compute the distance between the ligand and all the flexible sidechains.
  @param robotPt
 */
void p3d_ComputeDistLigandSidechains(p3d_rob* robotPt, std::vector<double>& distLigandSidechains)
{
  distLigandSidechains.clear();
  // fairly safely cast int to unsigned
  unsigned nbFlexSc = passiveSidechainsZone->count();
  const std::vector<p3d_jnt*>& firstJoints = passiveSidechainsZone->firstJoints();
  update_ligand_bounding_box(robotPt);
  for(unsigned i(0); i < firstJoints.size(); i++)
  {
    if(!update_sidechain_BBox_from_firstjnt(robotPt, firstJoints[i]))
    {
      std::cout << __PRETTY_FUNCTION__ << "warning... something bad happened in update_sidechain_BBox_from_firstjnt(" << robotPt << ", " << firstJoints[i] << std::endl;
    }
  }
  for(unsigned i = 0; i < nbFlexSc; i++)
  {
    p3d_jnt* jntPt = firstJoints[i];
    int AAnumber = get_AAnumber_from_jnt(jntPt);
    double distVector[3];
    dist_ligand_sidechainBBoxes(robotPt, AAnumber, distVector);
    double dist = distVector[0]*distVector[0] + distVector[1]*distVector[1] + distVector[2]*distVector[2];
    distLigandSidechains.push_back(sqrt(dist));
  }
}

double flexibleSidechainsAvoidance(Configuration& q)
{
  q.getRobot()->setAndUpdate(q);
  // std::vector<double> distances;
  // p3d_ComputeDistLigandSidechains(q.getRobot()->getRobotStruct(), distances);
  // double min(0);
  // if(distances.size() > 0)
  // {
  //   min = *std::min_element(distances.begin(), distances.end());
  // }
  // std::cout << "distances: ";
  // for(unsigned i(0); i < distances.size(); i++)
  // {
  //   std::cout << distances[i] << " ";
  // }
  // std::cout << std::endl;
  //return(min);
  return(0.01 + 1 / passiveSidechainsZone->minDist());
}

#endif

void initCostFunctions(p3d_rob* robot)
{
  Robot R(robot);
  costFunctionMap[costType::costMap2D] = boost::bind(computeIntersectionWithGround, _1);
  partition.init(&R);
  costFunctionMap[costType::passiveAndObstaclesAvoidance] = boost::bind(passiveAndObstaclesAvoidance, _1);
  costFunctionMap[costType::passiveAndObstaclesProximity] = boost::bind(passiveAndObstaclesProximity, _1);
#ifdef BIO
  costFunctionMap[costType::flexibleSidechainsAvoidance] = boost::bind(flexibleSidechainsAvoidance, _1);
  costFunctionMap[costType::BALLmmff94] = boost::bind(&BallEnergy::computeEnergy, XYZ_ENV->energyComputer, _1);
  if(p3d_col_get_mode() == p3d_col_mode_bio)
  {
    if(passiveSidechainsZone != 0)
    {
      delete(passiveSidechainsZone);
    }
    passiveSidechainsZone = new PassiveSidechainsZone(robot);
    costFunction = costFunctionMap[costType::flexibleSidechainsAvoidance];
  }
  else
#endif
  {
    costFunction = costFunctionMap[costType::costMap2D];
  }
}

void setCostFunction(costType::t type)
{
  std::map<costType::t, boost::function<double(Configuration&)> >::iterator i = costFunctionMap.find(type);
  if(i == costFunctionMap.end()) {
    std::cout << "Warning: cost function of type " << type << " not initialized." << std::endl;
  }
  else {
    costFunction = i->second;
  }
}

double computeCost(Configuration& conf)
{
  static unsigned count(0);
  count++;
  if(!(count % 100)) {
    std::cout << count << std::endl; }
  
  if(costFunction.empty())
  {
    std::cout << "Warning: There is no cost function selected." << std::endl;
    return(0);
  }
  else
  {
    return(costFunction(conf));
  }
}
