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

static FL_FORM * AWCI = NULL;

static FL_OBJECT * AWCI_START;
static FL_OBJECT * AWCI_CLOSE;
static FL_OBJECT * AWCI_RESP_INIT;
static FL_OBJECT * AWCI_RESP_END;
static FL_OBJECT * AWCI_RESP_TIME;
static FL_OBJECT * AWCI_FILTER;
static FL_OBJECT * AWCI_SHOW;

static AnimProb * ZANIM;

static void anim_awci_cb (FL_OBJECT * obj, long arg) {
  switch (arg) {
  case 10 :
    anim_walk_initialize(ZANIM);
    ZANIM->WalkOptions.RespectTime = fl_get_slider_value(AWCI_RESP_TIME);
    ZANIM->WalkOptions.RespectInit = fl_get_button(AWCI_RESP_INIT);
    ZANIM->WalkOptions.RespectEnd  = fl_get_button(AWCI_RESP_END);
    ZANIM->WalkOptions.Filter      = fl_get_slider_value(AWCI_FILTER);
    anim_walk(ZANIM);
    anim_interface_update();
    break;
  case 11 :
    fl_hide_form(AWCI);
    fl_free_form(AWCI);
    AWCI = NULL;
    anim_interface_update();
    break;
  case 12 :
    anim_show_form(ZANIM->WalkContRes, ZANIM->Robot);
    break;
  default :
    PrintWarning (("anim_charac_traj_interface.c -- unknown command"));
    break;
  }
}

void anim_walk_controller_interface(AnimProb * AnimProbPt) {
  int NofCoef;

  NofCoef = XYZ_ANIM.animation[0]->nof_coef;
  ZANIM = AnimProbPt;
  
  if (AWCI != NULL) {
    fl_hide_form(AWCI);
    fl_free_form(AWCI);
    AWCI = NULL;
  }
  
  AWCI = fl_bgn_form(FL_UP_BOX, 200, 250);

  AWCI_RESP_INIT = fl_add_button(FL_PUSH_BUTTON, 10, 10, 80, 20, "Resp Init Pos");
  
  AWCI_RESP_END = fl_add_button(FL_PUSH_BUTTON, 110, 10, 80, 20, "Resp End Pos");

  AWCI_RESP_TIME = fl_add_valslider(FL_HOR_SLIDER, 10, 40, 180, 20, "Respect Time");
  fl_set_slider_bounds (AWCI_RESP_TIME, 0, 2);
  fl_set_slider_value  (AWCI_RESP_TIME, 1);
  fl_set_slider_step   (AWCI_RESP_TIME, 0.1);

  AWCI_FILTER = fl_add_valslider(FL_HOR_SLIDER, 10, 100, 180, 20, "Filter");
  fl_set_slider_bounds (AWCI_FILTER, 0, NofCoef / 2);
  fl_set_slider_value  (AWCI_FILTER, NofCoef / 2);
  fl_set_slider_step   (AWCI_FILTER, 1);

  AWCI_START = fl_add_button(FL_NORMAL_BUTTON, 10, 160, 180, 20, "Start");
  fl_set_object_callback(AWCI_START, anim_awci_cb, 10);

  AWCI_SHOW = fl_add_button(FL_NORMAL_BUTTON, 10, 190, 180, 20, "Show");
  fl_set_object_callback(AWCI_SHOW, anim_awci_cb, 12);
  
  AWCI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 220, 180, 20, "Close");
  fl_set_object_callback(AWCI_CLOSE, anim_awci_cb, 11);
  
  fl_end_form();
  fl_show_form (AWCI, FL_PLACE_SIZE, FL_FULLBORDER, "Walk Controller");
}
