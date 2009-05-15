#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

static FL_FORM * ACPI = NULL;

static FL_OBJECT * ACPI_CHOOSE_ROB;
static FL_OBJECT * ACPI_CHOOSE_MINIROB;
static FL_OBJECT * ACPI_COMPUTE_TRAJ;
static FL_OBJECT * ACPI_CLOSE;
static FL_OBJECT * ACPI_SHOW_INIT;
static FL_OBJECT * ACPI_SHOW_GOAL;
static FL_OBJECT * ACPI_SHOW_TRAJ;
static FL_OBJECT * ACPI_DRAW_TRAJ;
static FL_OBJECT * ACPI_DRAW_GRAPH;
static FL_OBJECT * ACPI_VERIF_LPTYPE;
static FL_OBJECT * ACPI_STOP;


static AnimProb * ZANIM;

static int ACPI_CONTINUE = TRUE;

static void anim_update_verif_lptype(void) {
  int RobotNum;
  p3d_rob * RobotPt;
  p3d_rob * MiniRobotPt;
  char text[255];

  RobotNum = fl_get_choice(ACPI_CHOOSE_ROB) -1;
  RobotPt  = XYZ_ENV->robot[RobotNum];
  RobotNum    = fl_get_choice(ACPI_CHOOSE_MINIROB) -1;
  MiniRobotPt = XYZ_ENV->robot[RobotNum];

  if (RobotPt->lpl_type != MiniRobotPt->lpl_type) {
    sprintf(text,"STEERING METHODS DIFFER");
    fl_set_object_label(ACPI_VERIF_LPTYPE, text);
    fl_set_object_color(ACPI_VERIF_LPTYPE, FL_RED, FL_BLACK);
  }
  else {
    sprintf(text,"STEERING METHODS ARE EQUAL");
    fl_set_object_label(ACPI_VERIF_LPTYPE, text);
    fl_set_object_color(ACPI_VERIF_LPTYPE, FL_GREEN, FL_BLACK);
  }
}

static void anim_acpi_cb (FL_OBJECT * obj, long arg) {
  int RobotNum;
  p3d_rob * RobotPt;
  p3d_rob * MiniRobotPt;

  switch (arg) {
  case 5 : 
    fl_set_object_color(ACPI_CHOOSE_ROB, FL_GREEN, FL_BLACK);
    anim_update_verif_lptype();
    break;
  case 6 : 
    fl_set_object_color(ACPI_CHOOSE_MINIROB, FL_GREEN, FL_BLACK);
    anim_update_verif_lptype();
    break;
  case 10 : 

    RobotNum = fl_get_choice(ACPI_CHOOSE_ROB) -1;
    RobotPt  = XYZ_ENV->robot[RobotNum];
    RobotNum    = fl_get_choice(ACPI_CHOOSE_MINIROB) -1;
    MiniRobotPt = XYZ_ENV->robot[RobotNum];

    ZANIM->Robot = RobotPt;
    ZANIM->RobotMin = MiniRobotPt;

    anim_compute_path(ZANIM);

    anim_interface_update();
    break;
  case 101 :
    ACPI_CONTINUE = FALSE;
  case 11 :
    RobotNum = fl_get_choice(ACPI_CHOOSE_ROB) -1;
    RobotPt  = XYZ_ENV->robot[RobotNum];
    p3d_set_and_update_this_robot_conf(RobotPt, RobotPt->ROBOT_POS);
    g3d_draw_allwin_active();
    break;
  case 12 :
    RobotNum = fl_get_choice(ACPI_CHOOSE_ROB) -1;
    RobotPt  = XYZ_ENV->robot[RobotNum];
    p3d_set_and_update_this_robot_conf(RobotPt, RobotPt->ROBOT_GOTO);
    g3d_draw_allwin_active();
    break;
  case 13 :
    G3D_DRAW_TRAJ = !G3D_DRAW_TRAJ;
    g3d_draw_allwin_active();
    break;
  case 14 :
    G3D_DRAW_GRAPH = !G3D_DRAW_GRAPH;
    g3d_draw_allwin_active();
    break;
  case 15 :
    if (ZANIM->FindTrajResult) {
      RobotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

      ZANIM->FindTrajResult->rob->tcur = ZANIM->FindTrajResult;
      p3d_sel_desc_id(P3D_ROBOT, ZANIM->FindTrajResult->rob);
      g3d_show_tcur_rob(ZANIM->FindTrajResult->rob,anim_compute_path_stop);

      p3d_sel_desc_id(P3D_ROBOT, RobotPt);
    }
    else PrintInfo(("No path computed yet\n"));
    break;
  case 16 :
    fl_hide_form(ACPI);
    fl_free_form(ACPI);
    ACPI = NULL;
    anim_interface_update();
    break;
  default : 
    PrintWarning (("anim_compute_path_interface.c -- unknown command"));
    break;
  }
}

int anim_compute_path_stop(void) {
  int Result;
  fl_check_forms();
  Result = ACPI_CONTINUE;
  ACPI_CONTINUE = TRUE;
  return Result;
}

void anim_compute_path_interface (AnimProb * AnimProbPt) {
  int LRob;

  ZANIM = AnimProbPt;

  if (ACPI != NULL) {
    fl_hide_form(ACPI);
    fl_free_form(ACPI);
    ACPI = NULL;
    anim_interface_update();
  }

  ACPI = fl_bgn_form(FL_UP_BOX, 200, 250);

  ACPI_CHOOSE_ROB  = fl_add_choice(FL_NORMAL_CHOICE,100, 10, 80, 20, "Animated robot : ");
  for(LRob = 0; LRob < p3d_get_desc_number(P3D_ROBOT); LRob++) {
    fl_addto_choice(ACPI_CHOOSE_ROB, XYZ_ENV->robot[LRob]->name);    
  }
  fl_set_object_color(ACPI_CHOOSE_ROB, FL_RED, FL_BLACK);
  fl_set_object_callback(ACPI_CHOOSE_ROB, anim_acpi_cb, 5);


  ACPI_CHOOSE_MINIROB  = fl_add_choice(FL_NORMAL_CHOICE,100, 40, 80, 20, "Semi-Bounding robot : ");
  for(LRob = 0; LRob < p3d_get_desc_number(P3D_ROBOT); LRob++) {
    fl_addto_choice(ACPI_CHOOSE_MINIROB, XYZ_ENV->robot[LRob]->name);
  }
  fl_set_object_color(ACPI_CHOOSE_MINIROB, FL_RED, FL_BLACK);
  fl_set_object_callback(ACPI_CHOOSE_MINIROB, anim_acpi_cb, 6);

  ACPI_VERIF_LPTYPE = fl_add_text(FL_NORMAL_TEXT, 10, 70, 180, 20, "");

  ACPI_COMPUTE_TRAJ = fl_add_button (FL_NORMAL_BUTTON, 10, 100, 80, 20, "Compute Path");
  fl_set_object_callback(ACPI_COMPUTE_TRAJ, anim_acpi_cb, 10);

  ACPI_STOP = fl_add_button (FL_NORMAL_BUTTON, 110, 100, 80, 20, "Stop");
  fl_set_object_callback(ACPI_STOP, anim_acpi_cb, 101);

  ACPI_SHOW_INIT = fl_add_button (FL_NORMAL_BUTTON, 10, 130, 80, 20, "Show Init");
  fl_set_object_callback(ACPI_SHOW_INIT, anim_acpi_cb, 11);
  
  ACPI_SHOW_GOAL = fl_add_button (FL_NORMAL_BUTTON, 110, 130, 80, 20, "Show Goal");
  fl_set_object_callback(ACPI_SHOW_GOAL, anim_acpi_cb, 12);

  ACPI_DRAW_TRAJ = fl_add_button (FL_NORMAL_BUTTON, 10, 160, 80, 20, "Draw Path");
  fl_set_object_callback(ACPI_DRAW_TRAJ, anim_acpi_cb, 13);

  ACPI_DRAW_GRAPH = fl_add_button (FL_NORMAL_BUTTON, 110, 160, 80, 20, "Draw Graph");
  fl_set_object_callback(ACPI_DRAW_GRAPH, anim_acpi_cb, 14);
  
  ACPI_SHOW_TRAJ = fl_add_button (FL_NORMAL_BUTTON, 10, 190, 180, 20, "Show Path");
  fl_set_object_callback(ACPI_SHOW_TRAJ, anim_acpi_cb, 15);
  
  ACPI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 220, 180, 20, "Close");
  fl_set_object_callback(ACPI_CLOSE, anim_acpi_cb, 16);
  
  anim_update_verif_lptype();

  fl_end_form();
  fl_show_form (ACPI, FL_PLACE_SIZE, FL_FULLBORDER, "Path Computing");
}
