#include "Manipulation.h"
#include <p3d_config_proto.h>

using namespace std;

Manipulation::Manipulation(p3d_rob * robot){
  _robot = robot;
}
Manipulation::~Manipulation(){
  for(unsigned int i; i < _handsGraspsConfig; i++){
    for(map::<gpGrasp, confgiPt>::iterator it = _handsGraspsConfig[i].begin(); it != _handsGraspsConfig[i].end(); it++){
      p3d_destroy_config(_robot, it->second);
    }
  }
}
