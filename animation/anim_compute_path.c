#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"

static void anim_compute_path_initialize(AnimProb * ZANIM) { 
  p3d_rob * Robot;
  
  Robot = ZANIM->Robot;
  
  if (!Robot) {
    PrintError (("anim_compute_path.c -- NO ROBOT SPECIFIED FOR ANIMATION\n"));
    return;
  }
  
  if (ZANIM->Qi) p3d_destroy_config(Robot, ZANIM->Qi);
  if (ZANIM->Qf) p3d_destroy_config(Robot, ZANIM->Qf);
  
  ZANIM->Qi = p3d_copy_config (Robot, Robot->ROBOT_POS);
  ZANIM->Qf = p3d_copy_config (Robot, Robot->ROBOT_GOTO);
  
  ZANIM->FindTrajResult  = NULL;
  ZANIM->OptimizedResult = NULL;
  
  anim_utils_destroy_buffer(Robot, &(ZANIM->SamplingRes));
  anim_utils_destroy_buffer(Robot, &(ZANIM->WalkContRes));
  anim_utils_destroy_buffer(Robot, &(ZANIM->ColAvoidRes));
  anim_utils_destroy_buffer(Robot, &(ZANIM->ColOpt1Res));
  anim_utils_destroy_buffer(Robot, &(ZANIM->ColOpt2Res));
  anim_utils_destroy_buffer(Robot, &(ZANIM->PosPlanRes));
  
  if (ZANIM->InitRRTGraph)
    p3d_del_graph(ZANIM->InitRRTGraph);

  if (ZANIM->FinalRRTGraph)
    p3d_del_graph(ZANIM->FinalRRTGraph);
  
  ZANIM->TrajCharacterized = 0;
}

void anim_traj_fct_draw (void) {
  g3d_draw_allwin_active();
}

void anim_compute_path(AnimProb * ZANIM) {
  int LDof;
  configPt qi,qf;
  int res;
  configPt Qparking;
  configPt QparkingMin;
  p3d_rob * RobotPt;

/* remember which robot wa selected */
  RobotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

/* select the mini robot of the animation problem */
  p3d_sel_desc_id(P3D_ROBOT, ZANIM->RobotMin);

/* delete dependant animation buffers */
  anim_compute_path_initialize(ZANIM);
  
/* initialize the planning problem and store the animated rob where the mini rob is */
  qi = p3d_alloc_config(ZANIM->RobotMin);
  qf = p3d_alloc_config(ZANIM->RobotMin);
  Qparking    = p3d_alloc_config(ZANIM->Robot);
  QparkingMin = p3d_get_robot_config(ZANIM->RobotMin);

  for (LDof = 0; LDof < ZANIM->RobotMin->nb_dof; LDof ++) {
    qi[LDof] = ZANIM->Robot->ROBOT_POS[LDof];
    qf[LDof] = ZANIM->Robot->ROBOT_GOTO[LDof];
    Qparking[LDof] = QparkingMin[LDof];
    
    ZANIM->RobotMin->ROBOT_POS[LDof]  = ZANIM->Robot->ROBOT_POS[LDof];
    ZANIM->RobotMin->ROBOT_GOTO[LDof] = ZANIM->Robot->ROBOT_GOTO[LDof];
  }

  for (LDof = ZANIM->RobotMin->nb_dof; LDof < ZANIM->Robot->nb_dof; LDof ++) {
    Qparking[LDof] = 0.;
  }

  p3d_set_and_update_this_robot_conf(ZANIM->Robot, Qparking);

/* solve the planning problem */
  res = p3d_specific_learn(qi,qf,NULL,NULL,anim_compute_path_stop,anim_traj_fct_draw);

  if(!res){
    PrintInfo(("anim_compute_path.c -- no solution found\n"));
  }

  else{
    ZANIM->FindTrajResult = p3d_graph_to_traj(ZANIM->RobotMin);
    if(ZANIM->FindTrajResult) {
      g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    }
    else {
      printf("anim_compute_path.c -- unexpected problem : path exists, but A* can't find it\n");
    }
  }

/* cleanup */
  p3d_set_and_update_this_robot_conf(ZANIM->RobotMin, QparkingMin);

  for (LDof = 0; LDof < ZANIM->RobotMin->nb_dof; LDof ++) {
    ZANIM->RobotMin->ROBOT_POS[LDof]  = QparkingMin[LDof];
    ZANIM->RobotMin->ROBOT_GOTO[LDof] = QparkingMin[LDof];
  }
  
  p3d_destroy_config(ZANIM->Robot,Qparking);
  p3d_destroy_config(ZANIM->RobotMin,QparkingMin);
  p3d_destroy_config(ZANIM->RobotMin,qi);
  p3d_destroy_config(ZANIM->RobotMin,qf);

  p3d_sel_desc_id(P3D_ROBOT, RobotPt);  
}

