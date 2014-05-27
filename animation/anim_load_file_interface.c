/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

static FL_FORM * ALFI = NULL;

static FL_OBJECT * ALFI_CHOOSE_FILE;
static FL_OBJECT * ALFI_CHOOSE_MLDEF;
static FL_OBJECT * ALFI_CHOOSE_ROB;
static FL_OBJECT * ALFI_XY_TRAJ;
static FL_OBJECT * ALFI_FILE_SPEC;
static FL_OBJECT * ALFI_FILE_COUNT;
static FL_OBJECT * ALFI_FILE_SEL;
static FL_OBJECT * ALFI_FILE_UNLOAD;
static FL_OBJECT * ALFI_CLOSE;

static void anim_update_xy_traj(int AnimId) {
  float * xdata;
  float * ydata;
  int NofData,LData;
  
  NofData = XYZ_ANIM.animation[AnimId]->nof_frames;
  xdata = MY_ALLOC(float, NofData);
  ydata = MY_ALLOC(float, NofData);
  for (LData = 0; LData < NofData; LData ++) {
    xdata[LData] = XYZ_ANIM.animation[AnimId]->data[LData][6];
    ydata[LData] = XYZ_ANIM.animation[AnimId]->data[LData][7];
  }
  fl_set_xyplot_data(ALFI_XY_TRAJ, xdata, ydata, NofData, "", "","");
  MY_FREE(xdata,float,NofData);
  MY_FREE(ydata,float,NofData);
}

static void anim_update_file_spec(int AnimId) {
  char text[1024] = "";
  sprintf (text, "Associated Robot : %s\nLocomotion Type : %s\nLocomotion Id : %d\nNumber of Frames : %d\nStraight trajectory ? %d\nTheta dof is number : %d\nFramerate : %d\n",
	   XYZ_ANIM.animation[AnimId]->aim_robot->name, 
	   XYZ_ANIM.animation[AnimId]->type,
	   XYZ_ANIM.animation[AnimId]->id,
	   XYZ_ANIM.animation[AnimId]->nof_frames, 
	   XYZ_ANIM.animation[AnimId]->straight,
	   XYZ_ANIM.animation[AnimId]->index_theta,
	   XYZ_ANIM.animation[AnimId]->framerate
	   );
  fl_set_object_label(ALFI_FILE_SPEC, text);  
}

static void anim_update_file_count(void) {
  char text[1024] = "";
  sprintf (text, "%d File(s) Loaded",XYZ_ANIM.nof_animation);
  fl_set_object_label(ALFI_FILE_COUNT, text);
}

static void anim_update_file_sel(int Init, int End) {
  int LAnim;
  for (LAnim = Init; LAnim < End; LAnim ++) {
    fl_addto_choice(ALFI_FILE_SEL, XYZ_ANIM.animation[LAnim]->name);
    fl_addto_choice(ALFI_FILE_UNLOAD, XYZ_ANIM.animation[LAnim]->name);
  }
}

static void anim_alfi_cb (FL_OBJECT * obj, long arg) {
  int RobotNum, FileCount, AnimId;
  p3d_rob * RobotPt;
  char * File;
  switch (arg) {
  case 5 :
    fl_set_object_color(obj, FL_GREEN, FL_BLACK);
    break;
  case 9 :
    RobotNum = fl_get_choice(ALFI_CHOOSE_ROB) -1;
    RobotPt = p3d_sel_desc_num(P3D_ROBOT,RobotNum);
    File = fl_show_fselector("Motion Library Def. Filename", p3d_rw_scenario_get_path(),"*.mldef"," ");
    if (File) {
      FileCount = anim_load_mldef(File, RobotPt->name);
      anim_update_file_sel(XYZ_ANIM.nof_animation -FileCount, XYZ_ANIM.nof_animation);
      anim_update_file_count();
      anim_interface_update();
    }
    break;
  case 10 : 
    File = fl_show_fselector("Motion Capture Filename", p3d_rw_scenario_get_path(),"*.anim2"," ");
    if (File) {
      RobotNum = fl_get_choice(ALFI_CHOOSE_ROB) -1;
      RobotPt = p3d_sel_desc_num(P3D_ROBOT,RobotNum);
      anim_load_mcap_file (File, RobotPt->name);
      anim_update_xy_traj(XYZ_ANIM.nof_animation -1);
      anim_update_file_spec(XYZ_ANIM.nof_animation -1);
      anim_update_file_count();
      anim_update_file_sel(XYZ_ANIM.nof_animation -1, XYZ_ANIM.nof_animation);
      anim_interface_update();
    }
    break;
  case 11 :
    fl_hide_form(ALFI);
    fl_free_form(ALFI);
    ALFI = NULL;
    anim_interface_update();
    break;
  case 12 :
    AnimId = fl_get_choice(ALFI_FILE_SEL) -1;
    anim_update_xy_traj(AnimId);
    anim_update_file_spec(AnimId);
    break;
  case 13 :
    AnimId = fl_get_choice(ALFI_FILE_UNLOAD) -1;
    anim_unload_file(AnimId);
    anim_load_file_interface();
    break;
  default : 
    PrintWarning (("anim_load_file_interface.c -- unknown command"));
    break;
  }
}

void anim_load_file_interface (void) {
  int LRob;
  int LAnim;

  if (ALFI != NULL) {
    fl_hide_form(ALFI);
    fl_free_form(ALFI);
    ALFI = NULL;
  }
  
  ALFI = fl_bgn_form(FL_UP_BOX, 200, 520);
  
  ALFI_CHOOSE_ROB  = fl_add_choice(FL_NORMAL_CHOICE,110, 10, 80, 20, "Associate to robot : ");
  for(LRob = 0; LRob < p3d_get_desc_number(P3D_ROBOT); LRob++) {
    p3d_sel_desc_num(P3D_ROBOT,LRob);
    fl_addto_choice(ALFI_CHOOSE_ROB, p3d_get_desc_curname(P3D_ROBOT));
  }

  fl_set_object_color(ALFI_CHOOSE_ROB, FL_RED, FL_BLACK);
  fl_set_object_callback(ALFI_CHOOSE_ROB, anim_alfi_cb, 5);

  ALFI_CHOOSE_MLDEF = fl_add_button(FL_NORMAL_BUTTON, 10, 40, 180, 20, "Load Motion Library Def.");
  fl_set_object_callback(ALFI_CHOOSE_MLDEF, anim_alfi_cb, 9);

  ALFI_CHOOSE_FILE  = fl_add_button  (FL_NORMAL_BUTTON, 10, 70, 180, 20, "Single File to Load");
  fl_set_object_callback(ALFI_CHOOSE_FILE, anim_alfi_cb, 10);
  
  ALFI_XY_TRAJ = fl_add_xyplot(FL_NORMAL_BUTTON, 10, 100, 180, 180, "");
  fl_set_object_color(ALFI_XY_TRAJ, FL_SLATEBLUE, FL_WHITE);
  
  ALFI_FILE_SPEC = fl_add_text(FL_NORMAL_TEXT, 10, 290, 180, 100, "");

  ALFI_FILE_COUNT = fl_add_text(FL_NORMAL_TEXT, 10, 400, 180, 20, "");
  anim_update_file_count();

  ALFI_FILE_SEL = fl_add_choice(FL_NORMAL_CHOICE, 110, 430, 80, 20, "Display File Info : ");
  for(LAnim = 0; LAnim < XYZ_ANIM.nof_animation; LAnim++) {
    fl_addto_choice(ALFI_FILE_SEL, XYZ_ANIM.animation[LAnim]->name);
  }
  fl_set_object_callback(ALFI_FILE_SEL, anim_alfi_cb, 12);

  ALFI_FILE_UNLOAD = fl_add_choice(FL_NORMAL_CHOICE, 110, 460, 80, 20, "Unload File : ");
  for(LAnim = 0; LAnim < XYZ_ANIM.nof_animation; LAnim++) {
    fl_addto_choice(ALFI_FILE_UNLOAD, XYZ_ANIM.animation[LAnim]->name);
  }
  fl_set_object_color(ALFI_FILE_UNLOAD, FL_RED, FL_BLACK);
  fl_set_object_callback(ALFI_FILE_UNLOAD, anim_alfi_cb, 13);

  ALFI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 490, 180, 20, "Close");
  fl_set_object_callback(ALFI_CLOSE, anim_alfi_cb, 11);
  
  fl_end_form();
  fl_show_form (ALFI, FL_PLACE_SIZE, FL_FULLBORDER, "Motion Capture File Loader");
}
