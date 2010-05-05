#ifndef HRP2_GIK_H
#define HRP2_GIK_H

#include "genom-openhrp/genom-hrp2.h"

#include "robot/hppGikStandingRobot.h"
//#include "hppGikTest.h"
#include "hppGikTools.h"
#include "hrp2Dynamics/hrp2OptHumanoidDynamicRobot.h"

#include "constraints/hppGikConfigurationConstraint.h"
#include "tasks/hppGikGenericTask.h"
#include "motionplanners/elements/hppGikStepElement.h"
#include "motionplanners/elements/hppGikInterpolatedElement.h"
#include "tasks/hppGikReachTask.h"
#include "tasks/hppGikStepTask.h"

#include "dynamicsJRLJapan/Joint.h"
#include "dynamicsJRLJapan/HumanoidDynamicMultiBody.h"
#include "robotDynamics/jrlRobotDynamicsObjectConstructor.h"
#include "dynamicsJRLJapan/robotDynamicsImpl.h"

#include "constraints/hppGikPlaneConstraint.h"
#include "constraints/hppGikPositionConstraint.h"
#include "constraints/hppGikRotationConstraint.h"
#include "constraints/hppGikTransformationConstraint.h"
#include "constraints/hppGikParallelConstraint.h"
#include "constraints/hppGikGazeConstraint.h"
#include "constraints/hppGikComConstraint.h"

typedef struct gik_solution
{
 configPt configs[5000];
 int no_configs;
 gik_solution()
 {
  no_configs=0;
 }
}gik_solution;


typedef enum STRUCT_MHP_Q_INDEXES{
  MHP_Q_X = 6,
  MHP_Q_Y,
  MHP_Q_Z,
  MHP_Q_RX,
  MHP_Q_RY,
  MHP_Q_RZ,

  MHP_Q_RLEG0,
  MHP_Q_RLEG1,
  MHP_Q_RLEG2,
  MHP_Q_RLEG3,
  MHP_Q_RLEG4,
  MHP_Q_RLEG5,

  MHP_Q_LLEG0,
  MHP_Q_LLEG1,
  MHP_Q_LLEG2,
  MHP_Q_LLEG3,
  MHP_Q_LLEG4,
  MHP_Q_LLEG5,

  MHP_Q_CHEST0,
  MHP_Q_CHEST1,

  MHP_Q_HEAD0,
  MHP_Q_HEAD1,

  MHP_Q_CONE,

  MHP_Q_RARM0,
  MHP_Q_RARM1,
  MHP_Q_RARM2,
  MHP_Q_RARM3,
  MHP_Q_RARM4,
  MHP_Q_RARM5,
  MHP_Q_RARM6,

  MHP_Q_RHAND0,
  MHP_Q_RHAND1,
  MHP_Q_RHAND2,
  MHP_Q_RHAND3,
  MHP_Q_RHAND4,
  MHP_Q_RHAND5,

  MHP_Q_LARM0,
  MHP_Q_LARM1,
  MHP_Q_LARM2,
  MHP_Q_LARM3,
  MHP_Q_LARM4,
  MHP_Q_LARM5,
  MHP_Q_LARM6,

  MHP_Q_LHAND0,
  MHP_Q_LHAND1,
  MHP_Q_LHAND2,
  MHP_Q_LHAND3,
  MHP_Q_LHAND4,
  MHP_Q_LHAND5

} MHP_Q_INDEXES;

//AKP : Structure to hold all the configurations of current solution for a task which might have a collision free path along with gik
typedef struct SOLUTION_CONFIGS_FOR_HRP2
{
 ghrp2_config_t gik_sol[5000];
 int no_configs;
} SOLUTION_CONFIGS_FOR_HRP2;



#endif
