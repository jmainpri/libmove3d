#ifndef __MANIPULATION_H__
#define __MANIPULATION_H__
#include <vector>
#include <map>
#include "GraspPlanning-pkg.h"

class Manipulation{
  public :
    Manipulation(p3d_rob *robot);
    ~Manipulation();
  private :
    std::vector< std::map<gpGrasp, configPt, less<gpGrasp> > > _handsGraspsConfig;
    p3d_rob * _robot;
}

#endif