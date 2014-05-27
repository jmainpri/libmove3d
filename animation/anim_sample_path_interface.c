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

static FL_FORM * ASPI = NULL;

static FL_OBJECT * ASPI_MAX_LINEAR_SPEED;
static FL_OBJECT * ASPI_ENABLE_MAX_ANGULAR_SPEED;
static FL_OBJECT * ASPI_MAX_ANGULAR_SPEED;
static FL_OBJECT * ASPI_ENABLE_MAX_ACCELERATION;
static FL_OBJECT * ASPI_MAX_ACCELERATION;
static FL_OBJECT * ASPI_FRAME_PER_SECOND;
static FL_OBJECT * ASPI_START;
static FL_OBJECT * ASPI_CLOSE;
static FL_OBJECT * ASPI_SHOW;

static AnimProb * ZANIM;

static void anim_aspi_cb (FL_OBJECT * obj, long arg) {
  switch (arg) {
  case 10 :
    anim_sample_path_initialize(ZANIM);
    ZANIM->SampleOptions.MaxLinearSpeed  = fl_get_slider_value(ASPI_MAX_LINEAR_SPEED);
    ZANIM->SampleOptions.MaxAngularSpeed = fl_get_slider_value(ASPI_MAX_ANGULAR_SPEED);
    ZANIM->SampleOptions.MaxAcceleration = fl_get_slider_value(ASPI_MAX_ACCELERATION);
    ZANIM->SampleOptions.FramePerSecond  = fl_get_slider_value(ASPI_FRAME_PER_SECOND);
    ZANIM->SampleOptions.EnableMaxAngularSpeed = fl_get_button(ASPI_ENABLE_MAX_ANGULAR_SPEED);
    ZANIM->SampleOptions.EnableMaxAcceleration = fl_get_button(ASPI_ENABLE_MAX_ACCELERATION);    
    anim_sample_path(ZANIM);
    anim_interface_update();
    break;
  case 11 :
    anim_show_form(ZANIM->SamplingRes, ZANIM->Robot);
    break;
  case 12 :
    fl_hide_form(ASPI);
    fl_free_form(ASPI);
    ASPI = NULL;
    anim_interface_update();
    break;
  default :
    PrintWarning (("anim_sample_path_interface.c -- unknown command"));
    break;
  }
}

void anim_sample_path_interface (AnimProb * AnimProbPt) {
  p3d_rob * RobotPt;
  
  ZANIM = AnimProbPt;
  
  if (ASPI != NULL) {
    fl_hide_form(ASPI);
    fl_free_form(ASPI);
    ASPI = NULL;
    anim_interface_update();
  }
  
  ASPI = fl_bgn_form(FL_UP_BOX, 200, 360);
  
  RobotPt = ZANIM->RobotMin;

  ASPI_MAX_LINEAR_SPEED = fl_add_valslider(FL_HOR_SLIDER, 10, 10, 180, 20, "Maximum Linear Speed");
  fl_set_slider_bounds (ASPI_MAX_LINEAR_SPEED, 0, 3000);
  fl_set_slider_value  (ASPI_MAX_LINEAR_SPEED, 1000);
  fl_set_slider_step   (ASPI_MAX_LINEAR_SPEED, 100);

  ASPI_ENABLE_MAX_ANGULAR_SPEED = fl_add_button (FL_PUSH_BUTTON, 10, 60, 180, 20, "Enable Max Angular Speed");

  ASPI_MAX_ANGULAR_SPEED = fl_add_valslider(FL_HOR_SLIDER, 10, 90, 180, 20, "Maximum Angular Speed");
  fl_set_slider_bounds (ASPI_MAX_ANGULAR_SPEED, 0, 5);
  fl_set_slider_value  (ASPI_MAX_ANGULAR_SPEED, 1);
  fl_set_slider_step   (ASPI_MAX_ANGULAR_SPEED, 0.1);

  ASPI_ENABLE_MAX_ACCELERATION = fl_add_button (FL_PUSH_BUTTON, 10, 140, 180, 20, "Enable Max Acceleration");

  ASPI_MAX_ACCELERATION = fl_add_valslider(FL_HOR_SLIDER, 10, 170, 180, 20, "Maximum Acceleration");
  fl_set_slider_bounds (ASPI_MAX_ACCELERATION, 0, 5000);
  fl_set_slider_value  (ASPI_MAX_ACCELERATION, 3000);
  fl_set_slider_step   (ASPI_MAX_ACCELERATION, 100);
  
  ASPI_FRAME_PER_SECOND = fl_add_valslider(FL_HOR_SLIDER, 10, 220, 180, 20, "Sample Rate (frames / s)");
  fl_set_slider_bounds (ASPI_FRAME_PER_SECOND, 1, 100);
  fl_set_slider_value  (ASPI_FRAME_PER_SECOND, 25);
  fl_set_slider_step   (ASPI_FRAME_PER_SECOND, 1);
  
  ASPI_START = fl_add_button(FL_NORMAL_BUTTON, 10, 270, 180, 20, "Start");
  fl_set_object_callback(ASPI_START, anim_aspi_cb, 10);
  
  ASPI_SHOW = fl_add_button(FL_NORMAL_BUTTON, 10, 300, 180, 20, "Show");
  fl_set_object_callback(ASPI_SHOW, anim_aspi_cb, 11);

  ASPI_CLOSE = fl_add_button(FL_NORMAL_BUTTON, 10, 330, 180, 20, "Close");
  fl_set_object_callback(ASPI_CLOSE, anim_aspi_cb, 12);
  
  fl_end_form();
  fl_show_form (ASPI, FL_PLACE_SIZE, FL_FULLBORDER, "Path Sampler");
}
