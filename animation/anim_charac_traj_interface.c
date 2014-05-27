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

static FL_FORM * ACTI = NULL;

static FL_OBJECT * ACTI_START;
static FL_OBJECT * ACTI_CLOSE;
static FL_OBJECT * ACTI_LIN_SPEED;
static FL_OBJECT * ACTI_ANG_SPEED;

static AnimProb * ZANIM;

static void anim_acti_update_plots (void) {
  float * xdata;
  float * ydata;
  int NofData,LData;

  NofData = ZANIM->SamplingRes.NofFrames;
  
  xdata = MY_ALLOC(float, NofData);
  ydata = MY_ALLOC(float, NofData);

  for (LData = 0; LData < NofData; LData ++) {
    xdata[LData] = LData;
    ydata[LData] = ZANIM->CharacRes.LinSpeed[LData];
  }
  fl_set_xyplot_data(ACTI_LIN_SPEED, xdata, ydata, NofData, "", "","");

  for (LData = 0; LData < NofData; LData ++) {
    xdata[LData] = LData;
    ydata[LData] = ZANIM->CharacRes.RotSpeed[LData];
  }
  fl_set_xyplot_data(ACTI_ANG_SPEED, xdata, ydata, NofData, "", "","");

  MY_FREE(xdata,float,NofData);
  MY_FREE(ydata,float,NofData);
}

static void anim_acti_cb (FL_OBJECT * obj, long arg) {
  switch (arg) {
  case 10 :
    anim_charac_traj(ZANIM);
    anim_acti_update_plots();
    anim_interface_update();
    break;
  case 11 :
    fl_hide_form(ACTI);
    fl_free_form(ACTI);
    ACTI = NULL;
    anim_interface_update();
    break;
  default :
    PrintWarning (("anim_charac_traj_interface.c -- unknown command"));
    break;
  }
}

void anim_charac_traj_interface (AnimProb * AnimProbPt) {
  
  ZANIM = AnimProbPt;
  
  if (ACTI != NULL) {
    fl_hide_form(ACTI);
    fl_free_form(ACTI);
    ACTI = NULL;
  }
  
  ACTI = fl_bgn_form(FL_UP_BOX, 200, 450);  

  ACTI_START = fl_add_button(FL_NORMAL_BUTTON, 10, 10, 180, 20, "Start");
  fl_set_object_callback(ACTI_START, anim_acti_cb, 10);

  ACTI_LIN_SPEED = fl_add_xyplot(FL_NORMAL_XYPLOT, 10, 40, 180, 180, "");
  fl_set_object_color(ACTI_LIN_SPEED, FL_SLATEBLUE, FL_WHITE);

  ACTI_ANG_SPEED = fl_add_xyplot(FL_NORMAL_XYPLOT, 10, 230, 180, 180, "");
  fl_set_object_color(ACTI_ANG_SPEED, FL_SLATEBLUE, FL_WHITE);
  
  ACTI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 420, 180, 20, "Close");
  fl_set_object_callback(ACTI_CLOSE, anim_acti_cb, 11);
  
  fl_end_form();
  fl_show_form (ACTI, FL_PLACE_SIZE, FL_FULLBORDER, "Traj. Characterizer");
}
