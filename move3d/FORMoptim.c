#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"

#define MAX_NB_TRY_OPTIM  20

FL_FORM *OPTIM_FORM = NULL;

extern FL_OBJECT  *OPTIM_OBJ;
extern FL_OBJECT  *SEARCH_DRAW_OPTIM_OBJ;

static int STOP = FALSE;

static int QUICK_DESCENT = TRUE;

static double D0 = 0;

static FL_OBJECT *OPTIM_FRAME_OBJ;
static FL_OBJECT *START_OPTIM_OBJ;
static FL_OBJECT *STOP_OPTIM_OBJ;
static FL_OBJECT *DRAW_OPTIM_OBJ;
static FL_OBJECT *RAND_FRAME_OBJ;
static FL_OBJECT *RAND_ACTIVATE_OBJ;
static FL_OBJECT *START_RAND_OBJ;
static FL_OBJECT *TIME_ACTIVATE_OBJ;
static FL_OBJECT *TIME_INPUT_OBJ;
static FL_OBJECT *NB_RAND_OPTIM_OBJ;
static FL_OBJECT *ELASTIC_FRAME_OBJ;
static FL_OBJECT *ELASTIC_ACTIVATE_OBJ;
static FL_OBJECT *START_ELASTIC_OBJ;
static FL_OBJECT *SLIPPERY_BAND_OBJ;
static FL_OBJECT *STICKY_BAND_OBJ;
static FL_OBJECT *EPS_OBJ;
static FL_OBJECT *D0_OBJ;
static FL_OBJECT *CLEAR_FRAME_OBJ;
static FL_OBJECT *CLEAR_ACTIVATE_OBJ;
static FL_OBJECT *START_CLEAR_OBJ;

static FL_OBJECT *SPACE1;
static FL_OBJECT *SPACE2;
static FL_OBJECT *SPACE3;
static FL_OBJECT *SPACE4;
static FL_OBJECT *SPACE5;
static FL_OBJECT *SPACE6;

void CB_start_optim_obj(FL_OBJECT *ob, long arg);
void CB_stop_optim_obj(FL_OBJECT *ob, long arg);
static void CB_start_elastic_obj(FL_OBJECT *ob, long arg);
static void CB_start_clear_obj(FL_OBJECT *ob, long arg);
static int fct_stop_optim(void);
static void CB_set_nb_optim(FL_OBJECT *ob, long arg);
static void CB_set_d0(FL_OBJECT *ob, long arg);
static void CB_set_band_obj(FL_OBJECT *ob, long arg);
static int CB_optimForm_OnClose(FL_FORM *form, void *arg);
static void CB_time_input(FL_OBJECT *ob, long arg);
static void CB_time_activate(FL_OBJECT *ob, long arg);


static int init_draw_optim(void (**fct_draw)(void));
// static void compute_rand_optim(void (*fct_draw)(void), p3d_traj **trajectory);
static void compute_elastic_optim(p3d_traj **trajectory, void (*fct_draw)(void));
static void compute_clear_optim(p3d_traj *traj);
static void finish_optim(void);
#define max(a,b) (a>b)?a:b


void g3d_create_optim_form(void) {
  double dmax;
  p3d_col_get_dmax(&dmax);
  char buffer [10];
  g3d_create_form(&OPTIM_FORM,250,400,FL_UP_BOX);//370
  g3d_create_frame(&OPTIM_FRAME_OBJ,FL_NO_FRAME,-1,-1,"",(void**)&OPTIM_FORM, 1);

  g3d_create_button(&START_OPTIM_OBJ,FL_PUSH_BUTTON,100.0,30.0,"Start",(void**)&OPTIM_FRAME_OBJ,0);
  fl_set_call_back(START_OPTIM_OBJ,CB_start_optim_obj,0);

  g3d_create_frame(&SPACE1,FL_NO_FRAME,19,30,"",(void**)&OPTIM_FRAME_OBJ, 0);

  g3d_create_button(&STOP_OPTIM_OBJ,FL_PUSH_BUTTON,100.0,30.0,"Stop",(void**)&OPTIM_FRAME_OBJ,0);
  fl_set_object_callback(STOP_OPTIM_OBJ,CB_stop_optim_obj,0);

  g3d_create_checkbutton(&DRAW_OPTIM_OBJ,FL_PUSH_BUTTON,-1,-1,"Show Optimization",(void**)&OPTIM_FRAME_OBJ,0);
  fl_set_object_color(DRAW_OPTIM_OBJ,FL_MCOL,FL_GREEN);


  /**************************************************/
  /*            random optimization                 */
  /**************************************************/

  g3d_create_frame(&RAND_FRAME_OBJ,FL_ENGRAVED_FRAME,-1,-1,"",(void**)&OPTIM_FORM, 1);

  g3d_create_checkbutton(&RAND_ACTIVATE_OBJ,FL_PUSH_BUTTON,-1,-1,"Random Optimization",(void**)&RAND_FRAME_OBJ,0);
  fl_set_button(RAND_ACTIVATE_OBJ, 1);

  g3d_create_frame(&SPACE2,FL_NO_FRAME,35,-1,"",(void**)&RAND_FRAME_OBJ, 0);

  g3d_create_button(&START_RAND_OBJ,FL_PUSH_BUTTON,50.0,20.0,"Start",(void**)&RAND_FRAME_OBJ,0);
  fl_set_call_back(START_RAND_OBJ,CB_start_rand_obj,0);

  g3d_create_checkbutton(&TIME_ACTIVATE_OBJ,FL_PUSH_BUTTON,-1,-1,"Time Limit",(void**)&RAND_FRAME_OBJ,0);
  fl_set_button(TIME_ACTIVATE_OBJ, p3d_get_use_optimization_time());
  fl_set_call_back(TIME_ACTIVATE_OBJ,CB_time_activate,0);
  g3d_create_input(&TIME_INPUT_OBJ,FL_NORMAL_INPUT,20,20,"",(void**)&RAND_FRAME_OBJ,0);
  sprintf(buffer, "%f", p3d_get_optimization_time());
  fl_set_input(TIME_INPUT_OBJ, buffer);
  fl_set_call_back(TIME_INPUT_OBJ,CB_time_input,0);

  g3d_create_valslider(&NB_RAND_OPTIM_OBJ,FL_HOR_SLIDER,224,30.0,"Nb Random Optimization",(void**)&RAND_FRAME_OBJ,0);
  fl_set_slider_bounds(NB_RAND_OPTIM_OBJ, 1, 300);
  fl_set_slider_step(NB_RAND_OPTIM_OBJ, 1);
  fl_set_slider_value(NB_RAND_OPTIM_OBJ, p3d_get_NB_OPTIM());
  fl_set_call_back(NB_RAND_OPTIM_OBJ, CB_set_nb_optim, 0);

  /**************************************************/
  /*            elastic optimization                 */
  /**************************************************/

  g3d_create_frame(&ELASTIC_FRAME_OBJ,FL_ENGRAVED_FRAME,-1,-1,"",(void**)&OPTIM_FORM, 1);

  g3d_create_checkbutton(&ELASTIC_ACTIVATE_OBJ,FL_PUSH_BUTTON,-1,-1,"Elastic Optimization",(void**)&ELASTIC_FRAME_OBJ,0);
  fl_set_button(ELASTIC_ACTIVATE_OBJ, 0);

  g3d_create_frame(&SPACE3,FL_NO_FRAME,42,-1,"",(void**)&ELASTIC_FRAME_OBJ, 0);

  g3d_create_button(&START_ELASTIC_OBJ,FL_PUSH_BUTTON,50.0,20.0,"Start",(void**)&ELASTIC_FRAME_OBJ,0);
  fl_set_call_back(START_ELASTIC_OBJ,CB_start_elastic_obj,0);

  fl_bgn_group();
  g3d_create_checkbutton(&SLIPPERY_BAND_OBJ,FL_RADIO_BUTTON,-1,-1,"Slippery Band",(void**)&ELASTIC_FRAME_OBJ,0);
  fl_set_object_color(SLIPPERY_BAND_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(SLIPPERY_BAND_OBJ,CB_set_band_obj,0);

  g3d_create_frame(&SPACE4,FL_NO_FRAME,10,-1,"",(void**)&ELASTIC_FRAME_OBJ, 0);

  g3d_create_checkbutton(&STICKY_BAND_OBJ,FL_RADIO_BUTTON,-1,-1,"Sticky Band",(void**)&ELASTIC_FRAME_OBJ,0);
  fl_set_object_color(STICKY_BAND_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(STICKY_BAND_OBJ,CB_set_band_obj,1);
  fl_end_group();
  fl_set_button(STICKY_BAND_OBJ,1);

  g3d_create_valslider(&EPS_OBJ,FL_HOR_SLIDER,224,30.0,"Optimization Precision",(void**)&ELASTIC_FRAME_OBJ,0);
  fl_set_slider_bounds(EPS_OBJ, 0,5);
  fl_set_slider_step(EPS_OBJ, 0.01);
  fl_set_slider_value(EPS_OBJ, 1);

  g3d_create_valslider(&D0_OBJ,FL_HOR_SLIDER,224,30.0,"Contact Distance",(void**)&ELASTIC_FRAME_OBJ,0);
  fl_set_slider_bounds(D0_OBJ, dmax/20, 5*dmax);
  fl_set_slider_step(D0_OBJ, dmax/20);
  D0 = dmax;
  fl_set_slider_value(D0_OBJ, D0);
  fl_set_call_back(D0_OBJ, CB_set_d0, 0);


  /**************************************************/
  /*            clear                               */
  /**************************************************/

  g3d_create_frame(&CLEAR_FRAME_OBJ,FL_ENGRAVED_FRAME,-1,-1,"",(void**)&OPTIM_FORM, 1);

  g3d_create_checkbutton(&CLEAR_ACTIVATE_OBJ,FL_PUSH_BUTTON,-1,-1,"Clear Trajectory",(void**)&CLEAR_FRAME_OBJ,0);
  fl_set_button(CLEAR_ACTIVATE_OBJ,1);

  g3d_create_frame(&SPACE5,FL_NO_FRAME,60,-1,"",(void**)&CLEAR_FRAME_OBJ, 0);

  g3d_create_button(&START_CLEAR_OBJ,FL_PUSH_BUTTON,50.0,20.0,"Start",(void**)&CLEAR_FRAME_OBJ,0);
  fl_set_call_back(START_CLEAR_OBJ,CB_start_clear_obj,0);
  g3d_create_frame(&SPACE6,FL_NO_FRAME,5,-1,"",(void**)&CLEAR_FRAME_OBJ, 0);

  fl_end_form();
  fl_set_form_atclose(OPTIM_FORM, CB_optimForm_OnClose, 0);
}

void g3d_delete_optim_form(void) {
  if(fl_get_button(OPTIM_OBJ))
    fl_hide_form(OPTIM_FORM);
  g3d_fl_free_object(START_OPTIM_OBJ);
  g3d_fl_free_object(SPACE1);
  g3d_fl_free_object(STOP_OPTIM_OBJ);
  g3d_fl_free_object(DRAW_OPTIM_OBJ);
  g3d_fl_free_object(OPTIM_FRAME_OBJ);

  g3d_fl_free_object(RAND_ACTIVATE_OBJ);
  g3d_fl_free_object(SPACE2);
  g3d_fl_free_object(START_RAND_OBJ);
  g3d_fl_free_object(NB_RAND_OPTIM_OBJ);
  g3d_fl_free_object(RAND_FRAME_OBJ);

  g3d_fl_free_object(ELASTIC_ACTIVATE_OBJ);
  g3d_fl_free_object(SPACE3);
  g3d_fl_free_object(START_ELASTIC_OBJ);
  g3d_fl_free_object(SLIPPERY_BAND_OBJ);
  g3d_fl_free_object(SPACE4);
  g3d_fl_free_object(STICKY_BAND_OBJ);
  g3d_fl_free_object(EPS_OBJ);
  g3d_fl_free_object(D0_OBJ);
  g3d_fl_free_object(ELASTIC_FRAME_OBJ);

  g3d_fl_free_object(CLEAR_ACTIVATE_OBJ);
  g3d_fl_free_object(SPACE5);
  g3d_fl_free_object(START_CLEAR_OBJ);
  g3d_fl_free_object(SPACE6);
  g3d_fl_free_object(CLEAR_FRAME_OBJ);

  g3d_fl_free_form(OPTIM_FORM);
}

static int CB_optimForm_OnClose(FL_FORM *form, void *arg) {
  //Call the fonction closing the form.
  fl_hide_form(OPTIM_FORM);
  fl_set_button(OPTIM_OBJ,0);//release the button optimisation on planer FORM
  //If we return FL_OK, the application will continue to try to shut down itself
  //   if however we return FL_IGNORE, the application will not continue this event
  return FL_IGNORE;
}

// lancement de la phase d'optimisation

void CB_start_optim_obj(FL_OBJECT *ob, long arg) {
  void (*fct_draw)(void);
  p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  int i, ntest=0, nb_optim=0;
  double gain,gaintot=1., epsilon = 0.0;

  STOP = FALSE;
#ifdef DPG
  if(traj){
    traj->isOptimized = true;
  }
#endif
  if (ob && !init_draw_optim(&fct_draw)) {
    fl_set_button(ob,0);
    return;
  }
  if(!traj || traj->nlp <= 1) {
    if(ob){//if is not a automatic call
      printf("Optimization not possible: current trajectory\
    contains one or zero local path\n");
    }
   fl_set_button(START_OPTIM_OBJ,0);
   return;
  }
  if(ob){fl_set_cursor(FL_ObjWin(ob), XC_watch);}
  if (fl_get_button(DRAW_OPTIM_OBJ))
    fct_draw = g3d_draw_allwin_active;
  else
    fct_draw = NULL;
  if (fl_get_button(RAND_ACTIVATE_OBJ))
    {
      ChronoOn();
      while (fabs(gaintot-1.0) < EPS6 && nb_optim < MAX_NB_TRY_OPTIM){
	for(i=1;i<=p3d_get_NB_OPTIM();i++){
	  if(p3d_optim_traj(traj,&gain, &ntest)){
	    gaintot = gaintot*(1.- gain);
	    /* position the robot at the beginning of the optimized trajectory */
	    position_robot_at_beginning(ir, traj);
	  }
	  if (fct_draw){(*fct_draw)();}
	  if (!fct_stop_optim())
	    {
	      g3d_draw_allwin_active();
	      ChronoPrint("");
	      /* When retrieving statistics;Commit Jim; date: 01/10/2008 */
	      double tu = 0.0, ts = 0.0;
	      ChronoTimes(&tu, &ts);
	      if(getStatStatus()){
	        XYZ_GRAPH->stat->postTime += tu;
	      }
	      ChronoOff();
        if(ob){
          fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);
          fl_set_button(START_OPTIM_OBJ,0);
        }
	      return;
	    }
	}
	nb_optim++;
      }
      if (fabs(gaintot-1.0) > EPS6){
	/* the curve has been optimized */
	p3d_simplify_traj(traj);
      }
      gaintot = (1.-gaintot)*100.;
      printf("La courbe a ete optimisee de %f%%\n",gaintot);
      printf("nb collision test : %d\n", ntest);
      ChronoPrint("");
      /* When retrieving statistics;Commit Jim; date: 01/10/2008 */
      double tu = 0.0, ts = 0.0;
      ChronoTimes(&tu, &ts);
      if(getStatStatus()){
        XYZ_GRAPH->stat->postTime += tu;
      }
      ChronoOff();
    }
  if (fl_get_button(ELASTIC_ACTIVATE_OBJ) && p3d_get_ik_choice() == IK_NORMAL)
    {
      epsilon = 0.001*fl_get_slider_value(EPS_OBJ);
      printf("\ngradient descent optimization \n");

      p3d_gradientDescentOptimize(traj, 0, epsilon, D0, QUICK_DESCENT, fct_stop_optim, fct_draw);
      position_robot_at_beginning(ir, traj);
    }
  else
    if (fl_get_button(CLEAR_ACTIVATE_OBJ) && p3d_get_ik_choice() == IK_NORMAL)
      {
	printf("\nclearing trajectory \n");
	p3d_clearTraj(traj, 0, fct_stop_optim);
	position_robot_at_beginning(ir, traj);
      }
  g3d_draw_allwin_active();
  if(ob){fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);}
  if(ob){fl_set_button(ob,0);}
}

void CB_start_rand_obj(FL_OBJECT *ob, long arg) {
  void (*fct_draw)(void);
  STOP = FALSE;

  if (!init_draw_optim(&fct_draw)) {
    fl_set_button(ob,0);
    return;
  }
  fl_set_cursor(FL_ObjWin(ob), XC_watch);
  compute_rand_optim(fct_draw, NULL);

  finish_optim();
  fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);
  fl_set_button(ob,0);
}

static void CB_start_elastic_obj(FL_OBJECT *ob, long arg) {
  void (*fct_draw)(void);
  STOP = FALSE;

  if (!init_draw_optim(&fct_draw)) {
    fl_set_button(ob,0);
    return;
  }
  fl_set_cursor(FL_ObjWin(ob), XC_watch);

  compute_elastic_optim(NULL, fct_draw);

  finish_optim();
  fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);
  fl_set_button(ob,0);
}

static void CB_start_clear_obj(FL_OBJECT *ob, long arg) {
  p3d_traj *traj= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  STOP = FALSE;

  if(!traj) {
    printf("Optimize : ERREUR : no current traj\n");
    fl_set_button(START_CLEAR_OBJ,0);
    return;
  }
  fl_set_cursor(FL_ObjWin(ob), XC_watch);

  compute_clear_optim(NULL);

  finish_optim();
  fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);
}

void CB_stop_optim_obj(FL_OBJECT *ob, long arg) {
  STOP = TRUE;
  fl_set_button(STOP_OPTIM_OBJ,0);
}

static int fct_stop_optim(void) {
  double ts, tu, tmax = p3d_get_optimization_time();
  fl_check_forms();
  ChronoMicroTimes(&tu, &ts);
  if (p3d_get_use_optimization_time() && tu >= tmax){
    printf("Optimization stoped by counter.\n");
    return FALSE;
  }
  if(STOP) {
    STOP = FALSE;
    printf("Optimization stoped by user.\n");
    return(FALSE);
  }else{
    return(TRUE);
  }
}

double p3d_get_d0() {
  return D0;
}

int p3d_get_QUICK_DESCENT() {
  return QUICK_DESCENT;
}

static void CB_set_d0(FL_OBJECT *ob, long arg) {
  D0 = fl_get_slider_value(ob);
}

static void CB_set_nb_optim(FL_OBJECT *ob, long arg) {
  p3d_set_NB_OPTIM(fl_get_slider_value(ob));
}

static void CB_set_band_obj(FL_OBJECT *ob, long arg) {
  if(arg == 0) {QUICK_DESCENT = FALSE;}
  if(arg == 1) {QUICK_DESCENT = TRUE;}
}

static void CB_time_input(FL_OBJECT *ob, long arg){
  p3d_set_optimization_time(atof(fl_get_input(TIME_INPUT_OBJ)) < 0?p3d_get_optimization_time():atof(fl_get_input(TIME_INPUT_OBJ)));
}

static void CB_time_activate(FL_OBJECT *ob, long arg){
  p3d_set_optimization_time(fl_get_button(TIME_ACTIVATE_OBJ));
}

static int init_draw_optim(void (**fct_draw)(void)) {
  p3d_traj *traj= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if(!traj) {
    printf("Optimize : ERREUR : no current traj\n");
    return 0;
  }
  if (fl_get_button(DRAW_OPTIM_OBJ))
    *fct_draw = g3d_draw_allwin_active;
  else
    *fct_draw = NULL;
  return 1;
}

void compute_rand_optim(void (*fct_draw)(void), p3d_traj **trajectory) {
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  int i, ntest=0, nb_optim=0;
  double gain,gaintot=1.0;
  p3d_traj *traj;

  if (trajectory == NULL) {
    traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  } else {
    traj = *trajectory;
  }
  ChronoOn();
  while (fabs(gaintot-1.0) < EPS6 && nb_optim < MAX_NB_TRY_OPTIM) {
    for(i=1;i<=p3d_get_NB_OPTIM();i++) {
      if(p3d_optim_traj(traj,&gain, &ntest)) {
        gaintot = gaintot*(1.- gain);
        /* position the robot at the beginning of the optimized trajectory */
        position_robot_at_beginning(ir, traj);
      }
      if (fct_draw) {(*fct_draw)();}
      if (!fct_stop_optim()) {
        g3d_draw_allwin_active();
        gaintot = (1.-gaintot)*100.;
        printf("La courbe a ete optimisee de %f%%\n",gaintot);
        printf("nb collision test : %d\n", ntest);
        ChronoPrint("");
        /* When retrieving statistics;Commit Jim; date: 01/10/2008 */
        double tu = 0.0, ts = 0.0;
        ChronoTimes(&tu, &ts);
        if(getStatStatus()){
          XYZ_GRAPH->stat->postTime += tu;
        }

        ChronoOff();
        return;
      }
    }
    nb_optim++;
  }
  if (fabs(gaintot-1.0) > EPS6) {
    /* the curve has been optimized */
    p3d_simplify_traj(traj);
  }
  gaintot = (1.-gaintot)*100.;
  printf("La courbe a ete optimisee de %f%%\n",gaintot);
  printf("nb collision test : %d\n", ntest);

  ChronoPrint("");
  /* When retrieving statistics;Commit Jim; date: 01/10/2008 */
  double tu = 0.0, ts = 0.0;
  ChronoTimes(&tu, &ts);
  if(getStatStatus()){
    XYZ_GRAPH->stat->postTime += tu;
  }
  ChronoOff();
}

static void compute_elastic_optim(p3d_traj **trajectory, void (*fct_draw)(void)) {
  double epsilon;
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  p3d_traj *traj;

  if (trajectory == NULL) {
    traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  } else {
    traj = *trajectory;
  }

  epsilon = 0.001*fl_get_slider_value(EPS_OBJ);
  printf("\ngradient descent optimization \n");

  p3d_gradientDescentOptimize(traj, 0, epsilon, D0, QUICK_DESCENT, fct_stop_optim, fct_draw);
  position_robot_at_beginning(ir, traj);
}

static void compute_clear_optim(p3d_traj *traj) {
  int ir = p3d_get_desc_curnum(P3D_ROBOT);

  if (traj == NULL) {
    traj= (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  }

  printf("\nclearing trajectory \n");
  p3d_clearTraj(traj, 0, fct_stop_optim);
  position_robot_at_beginning(ir, traj);
}

static void finish_optim(void) {
  g3d_draw_allwin_active();
}

void p3d_enable_optim_time_limit(void){
  fl_set_button(TIME_ACTIVATE_OBJ, 1);
}
void p3d_disable_optim_time_limit(void){
  fl_set_button(TIME_ACTIVATE_OBJ, 0);
}
