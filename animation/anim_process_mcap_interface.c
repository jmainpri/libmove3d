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
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Animation-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

static FL_FORM * APMI = NULL;

static FL_OBJECT * APMI_ALL_MCAP;
static FL_OBJECT * APMI_CHOOSE_MCAP;
static FL_OBJECT * APMI_PROCESS_MCAP;
static FL_OBJECT * APMI_XY_TRAJ;
static FL_OBJECT * APMI_MCAP_SPEC;
static FL_OBJECT * APMI_CLOSE;
static FL_OBJECT * APMI_ART_TRAJ;
static FL_OBJECT * APMI_ART_COEF;
static FL_OBJECT * APMI_ART_SEL;
static FL_OBJECT * APMI_BOX;


static void anim_update_art_coef(int AnimId, int ArtId) {
  float * xdata;
  float * ydata;
  int NofData,LData;
  p3d_animation * AnimPt = XYZ_ANIM.animation[AnimId];

  if (AnimPt->processed == 0) return;

  NofData = AnimPt->nof_coef/2;
  xdata = MY_ALLOC(float, NofData);
  ydata = MY_ALLOC(float, NofData);

  for (LData = 0; LData < NofData; LData ++) {
    xdata[LData] = LData;
    ydata[LData] = fabs(AnimPt->fourier_real[LData][ArtId])
      +fabs (AnimPt->fourier_real[LData][ArtId]);
  }
  fl_set_xyplot_data(APMI_ART_COEF, xdata, ydata, NofData, "", "","");
  MY_FREE(xdata,float,NofData);
  MY_FREE(ydata,float,NofData);
}

static void anim_update_art_traj(int AnimId, int ArtId) {
  float * xdata;
  float * ydata;
  int NofData,LData;
  p3d_animation * AnimPt = XYZ_ANIM.animation[AnimId];

  NofData = AnimPt->nof_frames;
  xdata = MY_ALLOC(float, NofData);
  ydata = MY_ALLOC(float, NofData);

  for (LData = 0; LData < NofData; LData ++) {
    xdata[LData] = LData;
    ydata[LData] = AnimPt->data[LData][ArtId];
  }

  fl_set_xyplot_data(APMI_ART_TRAJ, xdata, ydata, NofData, "", "","");
  MY_FREE(xdata,float,NofData);
  MY_FREE(ydata,float,NofData);
}

static void anim_update_art_sel(int AnimId) {
  p3d_rob * RobotPt;

  RobotPt = XYZ_ANIM.animation[AnimId]->aim_robot;

  fl_set_slider_bounds (APMI_ART_SEL, 0,RobotPt->nb_dof-1);
  fl_set_slider_value  (APMI_ART_SEL, 0);
  fl_set_slider_step   (APMI_ART_SEL, 1);
  
}

static void anim_update_xy_traj(int AnimId) {
  float * xdata;
  float * ydata;
  int NofData,LData;
  double Param,EstAngle;
  p3d_animation * AnimPt = XYZ_ANIM.animation[AnimId];
  double AX,BX,AY,BY,AngleDepart,AngleArrivee,A,B,R;

  if (AnimPt->processed == 0) return;

  NofData = 2*AnimPt->nof_frames;
  xdata = MY_ALLOC(float, NofData);
  ydata = MY_ALLOC(float, NofData);

  AX = AnimPt->tcharac.AX;
  BX = AnimPt->tcharac.BX;
  AY = AnimPt->tcharac.AY;
  BY = AnimPt->tcharac.BY;
  AngleDepart = AnimPt->tcharac.AngleDepart;
  AngleArrivee = AnimPt->tcharac.AngleArrivee;
  A = AnimPt->tcharac.A;
  B = AnimPt->tcharac.B;
  R = AnimPt->tcharac.R;

  for (LData = 0; LData < NofData/2; LData ++) {
    xdata[LData] = AnimPt->data[LData][6];
    ydata[LData] = AnimPt->data[LData][7];
  }

  if (AnimPt->straight == 1) {
    for (LData = 0; LData < NofData/2; LData ++) {
      Param = NofData/2 - LData -1;
      xdata[LData+NofData/2] = BX + AX * Param;
      ydata[LData+NofData/2] = BY + AY * Param;
    }
  }
  else {
    for (LData = 0; LData < NofData/2; LData ++) {
      Param = 1. - (double)(LData)/((double)(NofData/2 -1));
      EstAngle = AngleDepart + diff_angle (AngleDepart, AngleArrivee) * Param;
      xdata[LData+NofData/2] = A + R * cos (EstAngle);
      ydata[LData+NofData/2] = B + R * sin (EstAngle);
    }
  }
  fl_set_xyplot_data(APMI_XY_TRAJ, xdata, ydata, NofData, "", "","");
  MY_FREE(xdata,float,NofData);
  MY_FREE(ydata,float,NofData);
}

static void anim_update_mcap_spec(int AnimId) {
  p3d_animation * AnimPt = XYZ_ANIM.animation[AnimId];
  double AX,BX,AY,BY,AngleDepart,AngleArrivee,A,B,R;
  char text[1024] = "";

  if (AnimPt->processed == 0) return;
  
  AX = AnimPt->tcharac.AX;
  BX = AnimPt->tcharac.BX;
  AY = AnimPt->tcharac.AY;
  BY = AnimPt->tcharac.BY;
  AngleDepart = AnimPt->tcharac.AngleDepart;
  AngleArrivee = AnimPt->tcharac.AngleArrivee;
  A = AnimPt->tcharac.A;
  B = AnimPt->tcharac.B;
  R = AnimPt->tcharac.R;
  
  if (AnimPt->straight == 1) {
    sprintf (text, "AX, BX : %f %f\nAY, BY : %f %f\nAZ, BZ : %f %f\nlength : %f\nlinear speed : %f\nangular speed : %f",
	     AnimPt->tcharac.AX,
	     AnimPt->tcharac.BX,
	     AnimPt->tcharac.AY,
	     AnimPt->tcharac.BY,
	     AnimPt->tcharac.AZ,
	     AnimPt->tcharac.BZ,
	     AnimPt->length,
	     AnimPt->linear_speed,
	     AnimPt->rotation_speed);
  }
  else {
    sprintf (text, "A, B : %f %f\nR : %f\nAngleD, AngleA : %f %f\nlength : %f\nlinear speed : %f\nangular speed : %f",
	     AnimPt->tcharac.A,
	     AnimPt->tcharac.B,
	     AnimPt->tcharac.R,
	     AnimPt->tcharac.AngleDepart,
	     AnimPt->tcharac.AngleArrivee,
	     AnimPt->length,
	     AnimPt->linear_speed,
	     AnimPt->rotation_speed);
  }
  fl_set_object_label(APMI_MCAP_SPEC, text);
}

static void anim_apmi_cb (FL_OBJECT * obj, long arg) {
  int AnimNum, LAnim, ArtNum;
  switch (arg) {
  case 5 :
    for (LAnim = 0; LAnim < XYZ_ANIM.nof_animation; LAnim ++) {
      if (XYZ_ANIM.animation[LAnim]->processed == 0) {
	anim_process_mcap(XYZ_ANIM.animation[LAnim]);
	anim_update_xy_traj(LAnim);
	anim_update_mcap_spec(LAnim);
      }
    }
    break;
  case 7 :
    AnimNum = fl_get_choice(APMI_CHOOSE_MCAP) -1;
    anim_update_xy_traj(AnimNum);
    anim_update_mcap_spec(AnimNum);
    anim_update_art_sel(AnimNum);
    break;
  case 10 :
    AnimNum = fl_get_choice(APMI_CHOOSE_MCAP) -1;
    anim_process_mcap(XYZ_ANIM.animation[AnimNum]);
    anim_update_xy_traj(AnimNum);
    anim_update_mcap_spec(AnimNum);
    break;
  case 11 :
    fl_hide_form(APMI);
    fl_free_form(APMI);
    APMI = NULL;
    anim_interface_update();
    break;
  case 12 :
    AnimNum = fl_get_choice(APMI_CHOOSE_MCAP) -1;
    ArtNum = fl_get_slider_value(APMI_ART_SEL);
    anim_update_art_traj(AnimNum,ArtNum);
    anim_update_art_coef(AnimNum,ArtNum);
    break;
  default : 
    PrintWarning (("anim_process_mcap_interface.c -- unknown command\n"));
    break;
  }
}

void anim_process_mcap_interface (void) {
  int LAnim;
  
  if (APMI != NULL) {
    fl_hide_form(APMI);
    fl_free_form(APMI);
    APMI = NULL;
  }
  
  APMI = fl_bgn_form(FL_UP_BOX, 400, 430);
  
  APMI_ALL_MCAP = fl_add_button(FL_NORMAL_BUTTON, 10, 10, 180, 20, "Proceed with ALL");
  fl_set_object_callback(APMI_ALL_MCAP, anim_apmi_cb, 5);

  APMI_CHOOSE_MCAP = fl_add_choice(FL_NORMAL_CHOICE,110, 40, 80, 20, "MCap Id to proceed : ");
  for(LAnim = 0; LAnim < XYZ_ANIM.nof_animation; LAnim++) {
    fl_addto_choice(APMI_CHOOSE_MCAP, XYZ_ANIM.animation[LAnim]->name);
  }
  fl_set_object_callback(APMI_CHOOSE_MCAP, anim_apmi_cb, 7);
  
  APMI_PROCESS_MCAP = fl_add_button (FL_NORMAL_BUTTON, 10, 70, 180, 20, "Proceed");
  fl_set_object_callback(APMI_PROCESS_MCAP, anim_apmi_cb, 10);

  APMI_XY_TRAJ = fl_add_xyplot(FL_NORMAL_XYPLOT, 10, 100, 180, 180, "");
  fl_set_object_color(APMI_XY_TRAJ, FL_SLATEBLUE, FL_WHITE);

  APMI_MCAP_SPEC = fl_add_text(FL_NORMAL_TEXT, 10, 290, 180, 100, "");

  APMI_CLOSE =  fl_add_button(FL_NORMAL_BUTTON, 10, 400, 180, 20, "Close");
  fl_set_object_callback(APMI_CLOSE, anim_apmi_cb, 11);

  APMI_BOX = fl_add_box(FL_FRAME_BOX, 198,5,4,420,"");

  APMI_ART_TRAJ = fl_add_xyplot(FL_NORMAL_XYPLOT, 210, 10, 180, 180, "");
  fl_set_object_color(APMI_ART_TRAJ, FL_SLATEBLUE, FL_WHITE);

  APMI_ART_SEL = fl_add_valslider(FL_HOR_SLIDER, 210, 200, 180, 20, "Articulation Selection");
  fl_set_slider_bounds (APMI_ART_SEL, -1,-1);
  fl_set_slider_value  (APMI_ART_SEL, -1);
  fl_set_slider_step   (APMI_ART_SEL, 1);
  fl_set_object_callback(APMI_ART_SEL, anim_apmi_cb, 12);

  APMI_ART_COEF = fl_add_xyplot(FL_NORMAL_XYPLOT, 210, 240, 180, 180, "");
  fl_set_object_color(APMI_ART_COEF, FL_SLATEBLUE, FL_WHITE);

  fl_end_form();
  fl_show_form (APMI, FL_PLACE_SIZE, FL_FULLBORDER, "Motion Capture Processor");
}
