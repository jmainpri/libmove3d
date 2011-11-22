//
//  ManipulationArmData.cpp
//  libmove3d
//
//  Created by Jim Mainprice on 22/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "ManipulationArmData.hpp"

#include "lightPlannerApi.h"

void ArmManipulationData::fixHand(p3d_rob* robot, bool rest) 
{
  if (rest) {
    gpSet_hand_rest_configuration(robot, _handProp, this->getId());
  }
  gpFix_hand_configuration(robot, _handProp, this->getId());
  gpDeactivate_hand_selfcollisions(robot, this->getId());
}

void ArmManipulationData::unFixHand(p3d_rob* robot) 
{
  gpUnFix_hand_configuration(robot, _handProp, this->getId());
  gpActivate_hand_selfcollisions(robot, this->getId());
}

void ArmManipulationData::deactivateManipulationCntrts(p3d_rob* robot)
{
  deactivateCcCntrts(robot, _id);
  desactivateTwoJointsFixCntrt(robot,getManipulationJnt(), getCcCntrt()->pasjnts[ getCcCntrt()->npasjnts-1 ]);
}