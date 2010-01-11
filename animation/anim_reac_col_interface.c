#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

static FL_FORM * ARCI = NULL;

static FL_OBJECT * ARCI_START;
static FL_OBJECT * ARCI_CLOSE;
static FL_OBJECT * ARCI_REAC_TIME;
static FL_OBJECT * ARCI_CONST_BLOCKS;
static FL_OBJECT * ARCI_NOF_BLOCK;
static FL_OBJECT * ARCI_NOF_ATTEMPTS;
static FL_OBJECT * ARCI_SOLVE_COL;
static FL_OBJECT * ARCI_SOLVED;
static FL_OBJECT * ARCI_SHOW_SOL;
static FL_OBJECT * ARCI_OPTIM;
static FL_OBJECT * ARCI_OPTIMIZED;
static FL_OBJECT * ARCI_SHOW_OPTIM;


static AnimProb * ZANIM;

static void anim_udpate_nof_blocks(void) {
  char text[255];
  if (ZANIM->ColOptions.NofBlocks > 0) {
    sprintf(text, "%d BLOCKS CONSTITUTED",ZANIM->ColOptions.NofBlocks);
    fl_set_object_label(ARCI_NOF_BLOCK, text);
    fl_set_object_color (ARCI_NOF_BLOCK, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "BLOCKS NOT CONSTITUTED");
    fl_set_object_label(ARCI_NOF_BLOCK, text);
    fl_set_object_color (ARCI_NOF_BLOCK, FL_RED, FL_BLACK);
  }
}

static void anim_udpate_solved(void) {
  char text[255];
  if (ZANIM->ColAvoidRes.NofFrames > 0) {
    sprintf(text, "COLLISIONS SOLVED");
    fl_set_object_label(ARCI_SOLVED, text);
    fl_set_object_color (ARCI_SOLVED, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "COLLISIONS UNSOLVED");
    fl_set_object_label(ARCI_SOLVED, text);
    fl_set_object_color (ARCI_SOLVED, FL_RED, FL_BLACK);
  }
}

static void anim_udpate_optimized(void) {
  char text[255];
  if (ZANIM->ColOpt2Res.NofFrames > 0) {
    sprintf(text, "AVOIDANCE OPTIMIZED");
    fl_set_object_label(ARCI_OPTIMIZED, text);
    fl_set_object_color (ARCI_OPTIMIZED, FL_GREEN, FL_BLACK);
  }
  else {
    sprintf(text, "AVOIDANCE NOT OPTIMIZED");
    fl_set_object_label(ARCI_OPTIMIZED, text);
    fl_set_object_color (ARCI_OPTIMIZED, FL_RED, FL_BLACK);
  }
}

static void anim_arci_cb (FL_OBJECT * obj, long arg) {
  switch (arg) {
  case 10 :
    ZANIM->ColOptions.ColBlockExtTime = fl_get_slider_value(ARCI_REAC_TIME);
    anim_col_avoidance_step1(ZANIM);
    anim_udpate_nof_blocks();
    anim_udpate_solved();
    anim_interface_update();
    break;
  case 11 : 
    ZANIM->ColOptions.NofAttempts = fl_get_slider_value(ARCI_NOF_ATTEMPTS);
    anim_col_avoidance_step2(ZANIM);
    anim_udpate_nof_blocks();
    anim_udpate_solved();
    anim_udpate_optimized();
    break;
  case 12 :
    anim_show_form(ZANIM->ColAvoidRes, ZANIM->Robot);
    break;
  case 13 :
    anim_col_avoidance_step3(ZANIM);
    anim_udpate_optimized();    
    break;
  case 14 :
    anim_show_form(ZANIM->ColOpt2Res, ZANIM->Robot);
    break;
  case 15 :
    fl_hide_form(ARCI);
    fl_free_form(ARCI);
    ARCI = NULL;
    anim_interface_update();
    break;
  default :
    PrintWarning (("anim_reac_col_interface.c -- unknown command"));
    break;
  }
}

void anim_reactive_collision_interface(AnimProb * AnimProbPt) {

  int NofCoef;

  NofCoef = XYZ_ANIM.animation[0]->nof_coef;
  ZANIM = AnimProbPt;
  
  if (ARCI != NULL) {
    fl_hide_form(ARCI);
    fl_free_form(ARCI);
    ARCI = NULL;
  }
  
  ARCI = fl_bgn_form(FL_UP_BOX, 200, 370);

  ARCI_REAC_TIME = fl_add_valslider (FL_HOR_SLIDER, 10, 10, 180, 20, "Reaction Time");
  fl_set_slider_bounds (ARCI_REAC_TIME, 0, 3);
  fl_set_slider_value  (ARCI_REAC_TIME, 1);
  fl_set_slider_step   (ARCI_REAC_TIME, 0.1);

  ARCI_CONST_BLOCKS =  fl_add_button(FL_NORMAL_BUTTON, 10, 50, 180, 20, "Constitute Blocks");
  fl_set_object_callback(ARCI_CONST_BLOCKS, anim_arci_cb, 10);
  
  ARCI_NOF_BLOCK = fl_add_text(FL_NORMAL_TEXT, 10, 80, 180, 20, "");
  anim_udpate_nof_blocks();

  ARCI_NOF_ATTEMPTS = fl_add_valslider (FL_HOR_SLIDER, 10, 110, 180, 20, "Number of Attempt for Solving Collisions");
  fl_set_slider_bounds (ARCI_NOF_ATTEMPTS, 0, 1000);
  fl_set_slider_value  (ARCI_NOF_ATTEMPTS, 100);
  fl_set_slider_step   (ARCI_NOF_ATTEMPTS, 10);

  ARCI_SOLVE_COL = fl_add_button(FL_NORMAL_BUTTON, 10, 150, 180, 20, "Solve Blocks");
  fl_set_object_callback(ARCI_SOLVE_COL, anim_arci_cb, 11);

  ARCI_SOLVED = fl_add_text(FL_NORMAL_TEXT, 10, 180, 180, 20, "");
  anim_udpate_solved();

  ARCI_SHOW_SOL = fl_add_button(FL_NORMAL_BUTTON, 10, 210, 180, 20, "Show Solution(s)");
  fl_set_object_callback(ARCI_SHOW_SOL, anim_arci_cb, 12);
      
  ARCI_OPTIM = fl_add_button(FL_NORMAL_BUTTON, 10, 240, 180, 20, "Optimize Solution(s)");
  fl_set_object_callback(ARCI_OPTIM, anim_arci_cb, 13);

  ARCI_OPTIMIZED = fl_add_text(FL_NORMAL_TEXT, 10, 270, 180, 20, "");
  anim_udpate_optimized();

  ARCI_SHOW_OPTIM = fl_add_button(FL_NORMAL_BUTTON, 10, 300, 180, 20, "Show Optimized Solution(s)");
  fl_set_object_callback(ARCI_SHOW_OPTIM, anim_arci_cb, 14);

  ARCI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 330, 180, 20, "Close");
  fl_set_object_callback(ARCI_CLOSE, anim_arci_cb, 15);
  
  fl_end_form();
  fl_show_form (ARCI, FL_PLACE_SIZE, FL_FULLBORDER, "Reactive collisions");
}
