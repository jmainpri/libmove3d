#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

static FL_FORM * AOPI = NULL;

static FL_OBJECT * AOPI_NOF_OPTIM;
static FL_OBJECT * AOPI_ENABLE_CLEAN;
static FL_OBJECT * AOPI_START;
static FL_OBJECT * AOPI_DRAW;
static FL_OBJECT * AOPI_SHOW;
static FL_OBJECT * AOPI_STOP;
static FL_OBJECT * AOPI_CLOSE;

static AnimProb * ZANIM;

static int AOPI_CONTINUE = TRUE;

static void anim_aopi_cb (FL_OBJECT * obj, long arg) {
  int NofOpt, EnableCleanTraj;
  p3d_rob * RobotPt;

  switch (arg) {
  case 9 :
	  ENV.setBool(Env::drawTraj,ENV.getBool(Env::drawTraj));
    g3d_draw_allwin_active();
    break;
  case 10 : 
    anim_optim_traj_initialize(ZANIM);
    NofOpt = fl_get_slider_value(AOPI_NOF_OPTIM);
    EnableCleanTraj = fl_get_button(AOPI_ENABLE_CLEAN);
    anim_optim_traj_start(ZANIM, NofOpt, EnableCleanTraj);
    anim_interface_update();
    g3d_draw_allwin_active();
    break;
  case 11 : 
    p3d_sel_desc_id(P3D_ROBOT, ZANIM->RobotMin);
    NofOpt = fl_get_slider_value(AOPI_NOF_OPTIM);
    EnableCleanTraj = fl_get_button(AOPI_ENABLE_CLEAN);
    anim_optim_traj_start(ZANIM, NofOpt, EnableCleanTraj);
    anim_interface_update();
    g3d_draw_allwin_active();
    break;
  case 12 : 
    if (ZANIM->OptimizedResult) {
      RobotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

      ZANIM->OptimizedResult->rob->tcur = ZANIM->OptimizedResult;
      p3d_sel_desc_id(P3D_ROBOT, ZANIM->OptimizedResult->rob);
      g3d_show_tcur_rob(ZANIM->OptimizedResult->rob,anim_optim_path_stop);

      p3d_sel_desc_id(P3D_ROBOT, RobotPt);
    }
    break;
  case 13 :
    fl_hide_form(AOPI);
    fl_free_form(AOPI);
    AOPI = NULL;
    anim_interface_update();
    break;
  case 14 : 
    AOPI_CONTINUE = FALSE;
    break;
  default : 
    PrintWarning (("anim_optim_path_interface.c -- unknown command"));
    break;
  }
}

int anim_optim_path_stop(void) {
  int Result;
  fl_check_forms();
  Result = AOPI_CONTINUE;
  AOPI_CONTINUE = TRUE;
  return Result;
}

void anim_optim_path_interface (AnimProb * AnimProbPt) {
  p3d_rob * RobotPt;
  int LTraj;

  ZANIM = AnimProbPt;

  if (AOPI != NULL) {
    fl_hide_form(AOPI);
    fl_free_form(AOPI);
    AOPI = NULL;
    anim_interface_update();
  }

  AOPI = fl_bgn_form(FL_UP_BOX, 200, 260);
  
  RobotPt = ZANIM->RobotMin;
    
  AOPI_ENABLE_CLEAN = fl_add_button (FL_PUSH_BUTTON, 10, 10, 180, 20, "Enable Trajectory Cleanup");
  fl_set_button(AOPI_ENABLE_CLEAN, 1);
  
  AOPI_NOF_OPTIM = fl_add_valslider(FL_HOR_SLIDER, 10, 40, 180, 20, "NofOpt");
  fl_set_slider_bounds(AOPI_NOF_OPTIM, 0, 50);
  fl_set_slider_value(AOPI_NOF_OPTIM, 1);
  fl_set_slider_step(AOPI_NOF_OPTIM, 1);

  AOPI_START = fl_add_button (FL_NORMAL_BUTTON, 10, 80, 180, 20, "Start Optimization");
  fl_set_object_callback(AOPI_START, anim_aopi_cb, 10);
  
  AOPI_CONTINUE = fl_add_button (FL_NORMAL_BUTTON, 10, 110, 180, 20, "Continue Optimization");
  fl_set_object_callback(AOPI_CONTINUE, anim_aopi_cb, 11);

  AOPI_DRAW = fl_add_button (FL_NORMAL_BUTTON, 10, 140, 180, 20, "Draw Optimized Path");
  fl_set_object_callback(AOPI_DRAW, anim_aopi_cb, 9);

  AOPI_SHOW = fl_add_button (FL_NORMAL_BUTTON, 10, 170, 180, 20, "Show Optimized Path");
  fl_set_object_callback(AOPI_SHOW, anim_aopi_cb, 12);

  AOPI_STOP = fl_add_button (FL_NORMAL_BUTTON, 10, 200, 180, 20, "Stop Show/Optimization");
  fl_set_object_callback(AOPI_STOP, anim_aopi_cb, 14);

  AOPI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 230, 180, 20, "Close ");
  fl_set_object_callback(AOPI_CLOSE, anim_aopi_cb, 13);
  
  fl_end_form();
  fl_show_form (AOPI, FL_PLACE_SIZE, FL_FULLBORDER, "Path Optimizer");
}
