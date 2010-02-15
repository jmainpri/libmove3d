#ifndef __MANIPULATION_H__
#define __MANIPULATION_H__
#include <vector>
#include <map>
#include "GraspPlanning-pkg.h"

class Manipulation{
  public :
    Manipulation(p3d_rob *robot);
    ~Manipulation();
    int findAllArmsGraspsConfigs();
    int findAllSpecificArmGraspsConfigs(int armId);
    int findGraspConfig(int armId, gpGrasp grasp, bool activateCntrt);
  private :
  std::vector< std::map<gpGrasp, configPt, std::less<gpGrasp> > > _handsGraspsConfig;
    p3d_rob * _robot;
};

#endif