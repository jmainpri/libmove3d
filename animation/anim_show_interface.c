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

static FL_FORM * ANIM_SHOW_FORM = NULL;

static FL_OBJECT * SHOW_INIT_FRAME;
static FL_OBJECT * SHOW_END_FRAME;
static FL_OBJECT * SHOW_PLAY;
static FL_OBJECT * SHOW_REVERSE;
static FL_OBJECT * SHOW_PAUSE;
static FL_OBJECT * SHOW_FORWARD;
static FL_OBJECT * SHOW_BACKWARD;
static FL_OBJECT * SHOW_FILM;
static FL_OBJECT * SHOW_RATIO;
static FL_OBJECT * SHOW_INTERPOLATE;
static FL_OBJECT * SHOW_CLOSE;
static FL_OBJECT * SHOW_TRACE;
static FL_OBJECT * SHOW_SAVE;
static FL_OBJECT * SHOW_ALLMCAP;
static double CurrentFrame = 0;
static int Film;
static double Ratio;
static anim_buffer Buffer;
int ANIM_TRACE = 0;
extern G3D_Window *G3D_WINDOW_LST;

static int anim_movie_fct(int movie_count)
{
  char str[512];
  char file[64];

  fl_check_forms();
  if(++movie_count < 10) sprintf(file,"0000%d.miff",movie_count);
  else if(movie_count < 100) sprintf(file,"000%d.miff",movie_count);
  else sprintf(file,"00%d.miff",movie_count);
  sprintf(str,"/usr/local/imagetools/sparc-solaris/bin/import -silent -window %d  %s",g3d_win_id(G3D_WIN),file); 
  system(str);
  return;
}

static double my_diff_angle(double theta1, double theta2) 
{
  double Result;
  Result = theta2 - theta1;
  while (Result > M_PI) {
    Result -= 2* M_PI;
  }
  while (Result < -M_PI) {
    Result += 2* M_PI;
  }
  return Result;
}

static void anim_show_one_frame(void) 
{
  int 
    IFrame, LDof,
    Interp = fl_get_button(SHOW_INTERPOLATE);  
  double
    Step  = fl_get_slider_value (SHOW_RATIO),
    Curseur, 
    DFrame,
    DiffDof;
  
  configPt Q, Qnext;
  p3d_rob * Robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  IFrame = (int)CurrentFrame;
  if (Interp == 0) {
    p3d_set_and_update_robot_conf(Buffer.AnimBuffer[IFrame]);
  }
  else {
    DFrame = CurrentFrame;
    if (IFrame == (Buffer.NofFrames -1)) {
      IFrame --;
      DFrame -=1.;
    }
    
    Curseur = (DFrame - IFrame);

    Q     = p3d_copy_config(Robot, Buffer.AnimBuffer[IFrame]);
    Qnext = p3d_copy_config(Robot, Buffer.AnimBuffer[IFrame +1]);
    for (LDof = 0; LDof < 9; LDof ++) {
      Q[LDof] = Q[LDof] * (1.-Curseur) + Qnext[LDof] * Curseur;
    }
    for (LDof = 10; LDof < Robot->nb_dof; LDof ++) {
      DiffDof = my_diff_angle(Q[LDof], Qnext[LDof]);
      Q[LDof] = Q[LDof] + Curseur * DiffDof;
    }
    p3d_set_and_update_robot_conf(Q);
    p3d_destroy_config (Robot, Q);
    p3d_destroy_config (Robot, Qnext);
  }
  p3d_col_test_all();
  g3d_draw_allwin_active();
  fl_check_forms();
}

static void anim_show_save_anim (void) {
  configPt Q, Qnext;
  p3d_rob * Robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

  const char * file;
  FILE * fdc;
  int LFrame, LDof;

  file = fl_show_fselector("Animation filename", p3d_rw_scenario_get_path(), 
			   "*.atr","animation.atr");
  if (file) {
    if(!(fdc = fopen(file,"w"))) {
      PrintError(("p3d_rw_scenario_read: can't open %s\n",file));
    }
    else {
      fprintf (fdc, "%d \n", Buffer.NofFrames);
      for (LFrame = 0; LFrame < Buffer.NofFrames; LFrame ++) {
	for (LDof = 0; LDof < Robot->nb_dof; LDof ++) {
	  fprintf (fdc, "%f ", Buffer.AnimBuffer[LFrame][LDof]);
	}
	fprintf(fdc,"\n");
      }
    }
    fclose(fdc);
  }
}

static void anim_show_CB (FL_OBJECT * obj, long arg) 
{
  int 
    InitFrame = fl_get_slider_value(SHOW_INIT_FRAME),
    EndFrame = fl_get_slider_value(SHOW_END_FRAME);  
  char str[512];
  double RememberCurrentFrame;
  G3D_Window * win = G3D_WINDOW_LST;
  FL_OBJECT *ob = ((FL_OBJECT *)win->canvas); 

  switch (arg) {
  case 10 :			/* Init frame */
    InitFrame = fl_get_slider_value(obj);
    break;
    
  case 11 :			/* End frame */
    EndFrame = fl_get_slider_value(obj);
    break;

  case 20 :			/* Play */
    fl_set_button (SHOW_REVERSE, 0);
    if (fl_get_button(SHOW_PLAY) == 1) {
      for (CurrentFrame = InitFrame; CurrentFrame < EndFrame; CurrentFrame += Ratio) {
	anim_show_one_frame();
	if (fl_get_button(SHOW_FILM) == 1) {
	  anim_movie_fct(CurrentFrame - InitFrame);
	}
	if (fl_get_button(SHOW_PLAY) == 0) {
	  break;
	}
      }
      if (fl_get_button(SHOW_FILM) == 1) {
	sprintf(str,"%s/bin/script/mpeg_make *.miff m3d.mpg",getenv("HOME_MOVE3D"));
	system(str);
	sprintf(str,"/bin/rm *.miff");
	system(str);
      }
      if (ANIM_TRACE == 1) {
	
      }
      fl_set_button (SHOW_PLAY, 0);
    }
    break;
  case 21 :			/* Reverse */
    fl_set_button (SHOW_PLAY, 0);
    if (fl_get_button(SHOW_REVERSE) == 1) {
      if (ANIM_TRACE == 1) {
	g3d_draw_env_box();
        g3d_draw_obstacles(g3d_get_cur_win(),opengl_context); // Modif NIC pour Florent
      }
      for (CurrentFrame = EndFrame; CurrentFrame >= InitFrame; CurrentFrame -= Ratio) {
	anim_show_one_frame();
	if (fl_get_button(SHOW_REVERSE) == 0) {
	  break;
	}
      }
      if (fl_get_button(SHOW_FILM) == 1) {
	sprintf(str,"%s/bin/script/mpeg_make *.miff m3d.mpg",getenv("HOME_MOVE3D"));
	system(str);
	sprintf(str,"/bin/rm *.miff");
	system(str);
      }
      fl_set_button (SHOW_REVERSE, 0);
    }
    break;
  case 22 :			/* Forward */
    CurrentFrame += Ratio;
    if (CurrentFrame > EndFrame) CurrentFrame = InitFrame;
    anim_show_one_frame();
    break;
  case 23 :			/* Backward */
    CurrentFrame -= Ratio;
    if (CurrentFrame < InitFrame) CurrentFrame = EndFrame;
    anim_show_one_frame();
    break;
  case 30 :			/* Film */
    break;
  case 31 :			/* Interp */
    break;
  case 32 :			/* Close */
    fl_set_button (SHOW_PLAY, 0);
    fl_set_button (SHOW_REVERSE, 0);
    CurrentFrame = 0;
    fl_hide_form(ANIM_SHOW_FORM);
    fl_free_form(ANIM_SHOW_FORM);
    ANIM_SHOW_FORM = NULL;
    break;
  case 33 : ANIM_TRACE = fl_get_button(obj);
    break;
  case 40 :			/* Ratio */
    Ratio = fl_get_slider_value(SHOW_RATIO);
    break;
  case 50 :
    anim_show_save_anim();
    break;
  default :
    break;
  }
}


void anim_show_form (anim_buffer BufferToShow, p3d_rob * RobotPt)
{
  Buffer = BufferToShow;

  if (ANIM_SHOW_FORM != NULL) {
    fl_set_button (SHOW_PLAY, 0);
    fl_set_button (SHOW_REVERSE, 0);
    fl_hide_form(ANIM_SHOW_FORM);
    fl_free_form(ANIM_SHOW_FORM);
    ANIM_SHOW_FORM = NULL;
    ANIM_TRACE = 0;
    Film = 0;
    CurrentFrame = 0;
  }
  
  p3d_sel_desc_id(P3D_ROBOT, RobotPt);
  
  ANIM_SHOW_FORM = fl_bgn_form(FL_UP_BOX, 200., 200.);
  
  SHOW_INIT_FRAME = fl_add_valslider(FL_HOR_SLIDER, 10, 10, 180, 20, "");
  fl_set_slider_return(SHOW_INIT_FRAME, FL_RETURN_END_CHANGED);
  fl_set_slider_bounds(SHOW_INIT_FRAME, 0, Buffer.NofFrames -1);
  fl_set_slider_value(SHOW_INIT_FRAME, 0);
  fl_set_slider_step(SHOW_INIT_FRAME, 1);
  fl_set_object_callback(SHOW_INIT_FRAME, anim_show_CB, 10);

  SHOW_END_FRAME = fl_add_valslider(FL_HOR_SLIDER, 10, 40, 180, 20, "");
  fl_set_slider_return(SHOW_END_FRAME, FL_RETURN_END_CHANGED);
  fl_set_slider_bounds(SHOW_END_FRAME, 0, Buffer.NofFrames -1);
  fl_set_slider_value(SHOW_END_FRAME, Buffer.NofFrames -1);
  fl_set_slider_step(SHOW_END_FRAME, 1);
  fl_set_object_callback(SHOW_END_FRAME, anim_show_CB, 11);

  SHOW_PLAY = fl_add_button (FL_PUSH_BUTTON, 105, 70, 40, 20, "PLAY");
  fl_set_object_callback (SHOW_PLAY, anim_show_CB, 20);

  SHOW_REVERSE = fl_add_button (FL_PUSH_BUTTON, 55, 70, 40, 20, "REV");
  fl_set_object_callback (SHOW_REVERSE, anim_show_CB, 21);

  SHOW_FORWARD = fl_add_button (FL_NORMAL_BUTTON, 155, 70, 40, 20, "FOR");
  fl_set_object_callback (SHOW_FORWARD, anim_show_CB, 22);
  
  SHOW_BACKWARD = fl_add_button (FL_NORMAL_BUTTON, 5, 70, 40, 20, "BAC");
  fl_set_object_callback (SHOW_BACKWARD, anim_show_CB, 23);
  
  SHOW_FILM = fl_add_button (FL_PUSH_BUTTON, 5, 100, 40, 20, "FILM");
  fl_set_object_callback (SHOW_FILM, anim_show_CB, 30);

  SHOW_INTERPOLATE = fl_add_button (FL_PUSH_BUTTON, 55, 100, 40, 20, "INTERP.");
  fl_set_object_callback (SHOW_INTERPOLATE, anim_show_CB, 31);

  SHOW_TRACE = fl_add_button (FL_PUSH_BUTTON, 105, 100, 40, 20, "TRACE");
  fl_set_object_callback (SHOW_TRACE, anim_show_CB, 33);

  SHOW_CLOSE = fl_add_button (FL_NORMAL_BUTTON, 155, 100, 40, 20, "CLOSE");
  fl_set_object_callback (SHOW_CLOSE, anim_show_CB, 32);  
  
  SHOW_RATIO = fl_add_valslider(FL_HOR_SLIDER, 10, 130, 180, 20, "Ratio");
  fl_set_slider_return(SHOW_RATIO, FL_RETURN_END_CHANGED);
  fl_set_slider_bounds(SHOW_RATIO, 0.1, 4.0);
  fl_set_slider_value(SHOW_RATIO, 1); Ratio = 1.;
  fl_set_slider_step(SHOW_RATIO, 0.1);
  fl_set_object_callback(SHOW_RATIO, anim_show_CB, 40);

  SHOW_SAVE = fl_add_button (FL_NORMAL_BUTTON, 10, 160, 40, 20, "SAVE");
  fl_set_object_callback (SHOW_SAVE, anim_show_CB, 50);

  fl_end_form();
  fl_show_form (ANIM_SHOW_FORM, FL_PLACE_SIZE, FL_FULLBORDER, "Animation player");
}
