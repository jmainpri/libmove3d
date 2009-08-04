#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

static p3d_traj * anim_clean_traj(p3d_traj * OldTraj)
{
  p3d_traj * CleanedTraj;
  p3d_localpath 
    * LastLPInTraj, * ScanEndLP, 
    * InitLP,       * LastValidSeg, 
    * LastValidLP,  * ProvLP;
  int ntest, col;
  configPt qi,qf;
  p3d_rob * robot;
  p3d_rob * RobotPt;

  RobotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  
  robot       = OldTraj->rob;
  CleanedTraj = p3d_create_empty_trajectory(robot);
  InitLP      = OldTraj->courbePt;

  p3d_sel_desc_id(P3D_ROBOT, robot);
  
  while (InitLP) {
    qi = InitLP->config_at_param(robot,InitLP,0.);
    LastValidSeg = InitLP;
    LastValidLP = NULL;
    ScanEndLP = InitLP->next_lp;
    while (ScanEndLP) {
      qf = ScanEndLP->config_at_param(robot, ScanEndLP, ScanEndLP->range_param);
      ProvLP = p3d_local_planner(robot, qi, qf);
      p3d_destroy_config(robot,qf);
      col = p3d_col_test_localpath(robot, ProvLP, &ntest);
      if (col == FALSE) { /* valid */
  if (LastValidLP) LastValidLP->destroy(robot, LastValidLP);
  LastValidLP = ProvLP;
  LastValidSeg = ScanEndLP;
      }
      else {
  ProvLP->destroy(robot, ProvLP);
      }
      ScanEndLP = ScanEndLP->next_lp;
    }
    p3d_destroy_config(robot, qi);
    if (LastValidLP == NULL) {
      LastValidLP = InitLP->copy(robot, InitLP);
    }
    /* add last valid path to CleanedTraj */
    if (CleanedTraj->courbePt == NULL) {
      CleanedTraj->courbePt = LastValidLP;
    }
    else {
      LastLPInTraj = CleanedTraj->courbePt;
      while (LastLPInTraj->next_lp) {
  LastLPInTraj = LastLPInTraj->next_lp;
      }
      LastLPInTraj->next_lp = LastValidLP;
      LastValidLP->prev_lp = LastLPInTraj;
    }
    CleanedTraj->nlp ++;
    /* ok */
    /* change InitLP */
    InitLP = LastValidSeg->next_lp;
  }
  CleanedTraj->range_param = p3d_compute_traj_rangeparam(CleanedTraj);
  robot->tcur = CleanedTraj;

  g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
  p3d_sel_desc_id(P3D_ROBOT, RobotPt);

  return CleanedTraj;
}

void anim_optim_traj_start(AnimProb * ZANIM, int NofOptims, int EnableCleanTraj) {
  int LOpt, ntest; 
  double gain;
  if (ZANIM->OptimizedResult) {
    for (LOpt = 0; LOpt < NofOptims; LOpt ++) {
      p3d_optim_traj(ZANIM->OptimizedResult,&gain, &ntest);
      if (G3D_DRAW_TRAJ) g3d_draw_allwin_active();
      if (!anim_optim_path_stop) break;
    }
    if (EnableCleanTraj) {
      ZANIM->OptimizedResult = anim_clean_traj(ZANIM->OptimizedResult);
      if (G3D_DRAW_TRAJ) g3d_draw_allwin_active();
    }
    ZANIM->RobotMin->tcur = ZANIM->OptimizedResult;
  }
  else {
    PrintWarning (("anim_optim_path.c -- No trajectory found to optimize \n"));
  } 
}

void anim_optim_traj_initialize(AnimProb * ZANIM) {
  p3d_rob * RobotPt;
  p3d_traj * TrajPt;

  RobotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  TrajPt =  p3d_get_desc_curid(P3D_TRAJ);

  p3d_sel_desc_id(P3D_ROBOT, ZANIM->RobotMin);
  p3d_sel_desc_id(P3D_TRAJ, ZANIM->FindTrajResult);

  if (ZANIM->OptimizedResult)
    p3d_del_traj(ZANIM->OptimizedResult);
  if (ZANIM->FindTrajResult) {
    ZANIM->OptimizedResult = p3d_create_traj_by_copy(ZANIM->FindTrajResult);
    g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
  }

  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->SamplingRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->WalkContRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColAvoidRes));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt1Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->ColOpt2Res));
  anim_utils_destroy_buffer(ZANIM->Robot, &(ZANIM->PosPlanRes));
  ZANIM->TrajCharacterized = 0;

  p3d_sel_desc_id(P3D_ROBOT, RobotPt);
  p3d_sel_desc_id(P3D_TRAJ, TrajPt);
}
