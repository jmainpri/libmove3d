#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#define LOCAL_COMPUTATION_EPSILON (1e-9)

/* ------- FUNCTION VARIABLES ------- */

hri_bitmapset * ACBTSET = NULL;

int PLACEMENT;
int PLCMT_TYPE;

static int HUMAN_FORM_CREATED = FALSE;
static int SELECTED_BTSET = 1;
static gnuplot_ctrl* gnuplots[] = {NULL,NULL,NULL,NULL,NULL};
static int GNUPLOT_ACTIVE = FALSE;

/* external psp variable */
extern int PSP_MA_SEGMENTS;
extern int PSP_MA_LAYERS;
extern int PSP_SRCH_MTD;
extern double PSP_PS_TRSHLD;

/* external gik variables */
extern int GIK_VIS;
extern double GIK_PRECISION;
extern int GIK_STEP;
extern double GIK_FORCE;
extern p3d_rob * GIK_target_robot;

/* --------- FORM VARIABLES ------- */
FL_FORM  * HRI_PLANNER_FORM = NULL;

static G3D_Window *persp_win;
static FL_OBJECT  *BT_FIND_MODEL_Q_POINT;
static FL_OBJECT  *BT_WATCH_OBJECT;
static FL_OBJECT  *BT_PSP_PARAMETERS_OBJECT;

static FL_OBJECT * MOTIONGROUP;
static FL_OBJECT * BT_MOTION_NAV_OBJ;
static FL_OBJECT * BT_MOTION_MANIP_OBJ;
static FL_OBJECT * BT_MOTION_OBJR_OBJ;
static FL_OBJECT * BT_MOTION_INIT_OBJ;

static FL_OBJECT * PATHGROUP;
static FL_OBJECT * BT_PATH_FIND_OBJ;

static FL_OBJECT * HUMANGROUP;
static FL_OBJECT * HUMANGROUPFR;
static FL_OBJECT * BT_HUMAN_ACTUAL_OBJ;
static FL_OBJECT * BT_HUMAN_EXISTS_OBJ;
static FL_OBJECT * BT_HUMAN_STATE_OBJ;

static FL_OBJECT * SHOWBTGROUP;
static FL_OBJECT * SHOWBTGROUPFR;
static FL_OBJECT * BT_SHOWBT_GNUPLOT_OBJ;
static FL_OBJECT * BT_SHOWBT_DIST_OBJ;
static FL_OBJECT * BT_SHOWBT_VIS_OBJ;
static FL_OBJECT * BT_SHOWBT_HZAC_OBJ;
static FL_OBJECT * BT_SHOWBT_OBS_OBJ;
static FL_OBJECT * BT_SHOWBT_COMB_OBJ;
static FL_OBJECT * BT_SHOWBT_PATH_OBJ;

static FL_OBJECT * BT_SAVE_BITMAPS_OBJ;

static FL_OBJECT * NAVGROUP;
static FL_OBJECT * NAVGROUPFR;
static FL_OBJECT * BT_NAV_DIST_OBJ;
static FL_OBJECT * BT_NAV_VIS_OBJ;
static FL_OBJECT * BT_NAV_HZ_OBJ;
static FL_OBJECT * BT_NAV_LEN_OBJ;
static FL_OBJECT  *BT_NAV_PARAM1_OBJ;
static FL_OBJECT  *BT_NAV_PARAM2_OBJ;
static FL_OBJECT  *BT_NAV_PARAM3_OBJ;
static FL_OBJECT  *BT_NAV_PARAM4_OBJ;

static FL_OBJECT * MANIPGROUP;
static FL_OBJECT * MANIPGROUPFR;
static FL_OBJECT * BT_MANIP_EXP_NO_OBJ;
static FL_OBJECT * BT_MANIP_EXP_FIND_OBJ;
static FL_OBJECT * BT_MANIP_EXP_SHOW_OBJ;

static FL_OBJECT * GIKGROUP;
FL_OBJECT * GIK_RUN_OBJ;  /* These variables are also accessed by the big gik interface */
FL_OBJECT * GIK_TARGET_ROBOT_OBJ;
FL_OBJECT * GIK_VIS_OBJ;
FL_OBJECT * GIK_PRECISION_OBJ;
FL_OBJECT * GIK_STEP_OBJ;

static FL_OBJECT * GIK_JOINTSEL_OBJ;

static FL_OBJECT * TESTGROUP;
static FL_OBJECT * TEST_BUTTON1_OBJ;
static FL_OBJECT * TEST_BUTTON2_OBJ;
static FL_OBJECT * TEST_BUTTON3_OBJ;
static FL_OBJECT * TEST_BUTTON4_OBJ;

/* ---------------------------------- */

/* ---------- FUNCTION DECLARATIONS --------- */
/* ------------------------------------------ */
static void g3d_create_find_model_q(void); /* Luis */
static void CB_find_model_q(FL_OBJECT *ob, long arg);
static void CB_watch_object(FL_OBJECT *ob, long arg);
static void CB_show_psp_parameters(FL_OBJECT *ob, long arg);

static void g3d_create_motion_group(void);
static void CB_motion_obj(FL_OBJECT *obj, long arg);
static void CB_motion_init_obj(FL_OBJECT *obj, long arg);

static void g3d_create_path_group(void);
static void CB_path_find_obj(FL_OBJECT *obj, long arg);

static void g3d_create_human_group(void);
static void CB_human_actual_obj(FL_OBJECT *obj, long arg);
static void CB_human_exists_obj(FL_OBJECT *obj, long arg);
static void CB_human_state_obj(FL_OBJECT *obj, long arg);

static void g3d_create_showbt_group(void);
static void CB_showbt_gnuplot_obj(FL_OBJECT *obj, long arg);
static void CB_showbt_obj(FL_OBJECT *obj, long arg);

static void g3d_create_save_bitmaps_obj(void);
static void CB_save_bitmaps_obj(FL_OBJECT *obj, long arg);

static void g3d_create_nav_group(void);
static void CB_nav_btchoice_obj(FL_OBJECT *obj, long arg);
static void CB_nav_param_obj(FL_OBJECT *obj, long arg);

static void g3d_create_manip_group(void);
static void CB_manip_exp_no_obj(FL_OBJECT *obj, long arg);
static void CB_manip_exp_find_obj(FL_OBJECT *obj, long arg);
static void CB_manip_exp_show_obj(FL_OBJECT *obj, long arg);

static void g3d_create_GIK_group(void);
static void CB_gik_jointsel_obj(FL_OBJECT *obj, long arg);

static void g3d_create_TEST_group(void);
static void CB_test_button1_obj(FL_OBJECT *obj, long arg);
static void CB_test_button2_obj(FL_OBJECT *obj, long arg);
static void CB_test_button3_obj(FL_OBJECT *obj, long arg);
static void CB_test_button4_obj(FL_OBJECT *obj, long arg);

static void g3d_delete_find_model_q(void);
void g3d_delete_psp_parameters_form(void);
/* ------------------------------------------ */


/* -------------- FUNCTION DEFINITIONS --------------- */

/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_hri_planner_form(void)
{
  HRI_PLANNER_FORM = fl_bgn_form(FL_UP_BOX,400.0,610.0);

  g3d_create_find_model_q();

  g3d_create_motion_group();
  g3d_create_path_group();
  g3d_create_human_group();
  g3d_create_showbt_group();
  g3d_create_save_bitmaps_obj();
  g3d_create_nav_group();
  g3d_create_manip_group();
  g3d_create_GIK_group();
	g3d_create_TEST_group();

  fl_end_form();

  g3d_create_gik_jointsel_form();
  g3d_create_psp_parameters_form();

}

void g3d_show_hri_planner_form(void)
{
  fl_show_form(HRI_PLANNER_FORM,FL_PLACE_SIZE,TRUE, "HRI Planner");
}

void g3d_hide_hri_planner_form(void)
{
  fl_hide_form(HRI_PLANNER_FORM);
}

void g3d_delete_hri_planner_form(void)
{
  g3d_delete_find_model_q();
  g3d_delete_psp_parameters_form();

  fl_free_form(HRI_PLANNER_FORM);
}

static int my_drawtraj_fct(void)
{
  fl_check_forms();
  return TRUE;
}

/* -------------------- PSP GROUP --------------------- */
static void g3d_create_find_model_q(void)
{
  FL_OBJECT *obj;

  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,10,350,40,"");
  obj = fl_add_box(FL_FLAT_BOX,20,5,80,10,"PSP");

  BT_WATCH_OBJECT = fl_add_button(FL_PUSH_BUTTON,15,15,80,30,"Watch");
  fl_set_call_back(BT_WATCH_OBJECT,CB_watch_object,0);

  BT_FIND_MODEL_Q_POINT = fl_add_button(FL_PUSH_BUTTON,100,15,80,30,"Modeling");
  fl_set_call_back(BT_FIND_MODEL_Q_POINT,CB_find_model_q,0);

  BT_PSP_PARAMETERS_OBJECT = fl_add_button(FL_PUSH_BUTTON,185,15,80,30,"PSP Params");
  fl_set_call_back(BT_PSP_PARAMETERS_OBJECT,CB_show_psp_parameters,0);
}

static void CB_find_model_q(FL_OBJECT *ob, long arg)
{
	//funcion par empezar el random
	//p3d_search_best_point();
	int i,j;
	if (!BTSET)
	{	
		printf("BTSET not initialized");
		return;
	}
	p3d_rob *r = (BTSET->robot);
	p3d_rob *human = (BTSET->human[BTSET->actual_human]->HumanPt);
	//int *iksols=NULL, *iksolg=NULL;
	//persp_win->win_perspective=1;

	fl_set_button(ob,0);
	g3d_draw_allwin_active();
	printf("%s\n",(human->name));
	for(i=0 ; i<=3 ; i++){
		for(j=0 ; j<=3 ; j++){
			printf("%f ",human->joints[1]->abs_pos[i][j]);
		}
		printf("\n");
	}
	//PSP_DEACTIVATE_AUTOHIDE=1;
	printf("robot name %s\n",r->name);
	p3d_select_robot_to_view(human);
	//p3d_select_robot_to_view(bottle);
	//psp_select_object_to_view_by_name("table1");
	g3d_draw_allwin_active();
	printf("Selected\n");
	// no es el BTSET
	//p3d_psp_srch_model_pt(r,human,50,iksols,iksolg,fct_stop,fct_draw);
	if (persp_win){
		//psp_srch_for_target_obj(r,50);
		//p3d_deselect_all_objects();

		PSP_DEACTIVATE_AUTOHIDE=1;
		psp_srch_model_pt(r,human, PSP_MA_SEGMENTS, PSP_MA_LAYERS, &PSP_SRCH_MTD, PSP_PS_TRSHLD,BTSET);
		printf("Searching\n");
		//psp_srch_model_pt(r,bottle,50,1,BTSET);

		g3d_draw_allwin_active();
	}
	p3d_deselect_robot_to_view(human);
	//p3d_deselect_robot_to_view(bottle);
	PSP_DEACTIVATE_AUTOHIDE=0;
}

static void CB_watch_object(FL_OBJECT *ob, long arg)
{
	p3d_init_robot_parameters();
	persp_win = g3d_show_persp_win();

	g3d_draw_allwin_active();
	fl_set_button(ob,0);
	// p3d_watch2_obj();
	//p3d_watch_obj();
}


static void CB_show_psp_parameters(FL_OBJECT *ob, long arg)
{
	//p3d_init_robot_parameters();
	//g3d_set_movie_flag(TRUE);
	//g3d_start_movie();
	if (fl_get_button(ob))
	{

		g3d_show_psp_parameters_form();
	}
	else
		g3d_hide_psp_parameters_form();
}

/* -------------------- MOTION GROUP --------------------- */
static void g3d_create_motion_group(void)
{
  FL_OBJECT *obj;

  obj = fl_add_labelframe(FL_ENGRAVED_FRAME,10,60,290,70,"Motion");

  MOTIONGROUP = fl_bgn_group();

  BT_MOTION_NAV_OBJ = fl_add_button(FL_NORMAL_BUTTON,20,70,70,25,"Navigation");
  BT_MOTION_MANIP_OBJ = fl_add_button(FL_NORMAL_BUTTON,100,70,90,25,"Object Handling");
  BT_MOTION_OBJR_OBJ = fl_add_button(FL_NORMAL_BUTTON,200,70,90,25,"Object Reach");
  BT_MOTION_INIT_OBJ = fl_add_button(FL_NORMAL_BUTTON,20,100,270,25,"Initialize");

  fl_set_call_back(BT_MOTION_NAV_OBJ,CB_motion_obj,1);
  fl_set_call_back(BT_MOTION_MANIP_OBJ,CB_motion_obj,2);
  fl_set_call_back(BT_MOTION_OBJR_OBJ,CB_motion_obj,3);
  fl_set_call_back(BT_MOTION_INIT_OBJ,CB_motion_init_obj,1);

  fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);

  fl_end_group();

}

/* --------- Bitmap Choice ----------- */
static void CB_motion_obj(FL_OBJECT *obj, long arg)
{
	/* NAVIGATION */
  if(arg == 1){
    ACBTSET = BTSET;
    fl_set_button(BT_MOTION_NAV_OBJ,1);
    fl_set_button(BT_MOTION_MANIP_OBJ,0);
    fl_set_button(BT_MOTION_OBJR_OBJ,0);
    if(BTSET==NULL){
      fl_deactivate_object(PATHGROUP);
      fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
      fl_deactivate_object(HUMANGROUP);
      fl_set_object_color(HUMANGROUPFR,FL_RED,FL_COL1);
      fl_deactivate_object(SHOWBTGROUP);
      fl_set_object_color(SHOWBTGROUPFR,FL_RED,FL_COL1);
      fl_deactivate_object(NAVGROUP);
      fl_set_object_color(NAVGROUPFR,FL_RED,FL_COL1);
    }
    else{
      fl_activate_object(PATHGROUP);
      fl_set_object_color(BT_PATH_FIND_OBJ,FL_GREEN,FL_COL1);
      fl_activate_object(HUMANGROUP);
      fl_set_object_color(HUMANGROUPFR,FL_GREEN,FL_COL1);
      fl_activate_object(SHOWBTGROUP);
      fl_set_object_color(SHOWBTGROUPFR,FL_GREEN,FL_COL1);
      fl_activate_object(NAVGROUP);
      fl_set_object_color(NAVGROUPFR,FL_GREEN,FL_COL1);
    }
    fl_deactivate_object(MANIPGROUP);
    fl_deactivate_object(BT_SHOWBT_GNUPLOT_OBJ);

  }
  /* OBJECT HANDLING */
  if(arg == 2){
    ACBTSET = INTERPOINT;
    fl_set_button(BT_MOTION_NAV_OBJ,0);
    fl_set_button(BT_MOTION_MANIP_OBJ,1);
    fl_set_button(BT_MOTION_OBJR_OBJ,0);
    if(INTERPOINT==NULL){
      fl_deactivate_object(PATHGROUP);
      fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
      fl_deactivate_object(HUMANGROUP);
      fl_set_object_color(HUMANGROUPFR,FL_RED,FL_COL1);
      fl_deactivate_object(SHOWBTGROUP);
      fl_set_object_color(SHOWBTGROUPFR,FL_RED,FL_COL1);
      fl_deactivate_object(NAVGROUP);
      fl_set_object_color(NAVGROUPFR,FL_RED,FL_COL1);
    }
    else{
      fl_activate_object(PATHGROUP);
      fl_set_object_color(BT_PATH_FIND_OBJ,FL_GREEN,FL_COL1);
      fl_activate_object(HUMANGROUP);
      fl_set_object_color(HUMANGROUPFR,FL_GREEN,FL_COL1);
      fl_activate_object(SHOWBTGROUP);
      fl_set_object_color(SHOWBTGROUPFR,FL_GREEN,FL_COL1);
      fl_activate_object(NAVGROUP);
      fl_set_object_color(NAVGROUPFR,FL_GREEN,FL_COL1);
    }
    fl_deactivate_object(NAVGROUP);
    fl_activate_object(BT_SHOWBT_GNUPLOT_OBJ);
  }
  /* OBJECT REACH */
  if(arg == 3){
    ACBTSET = OBJSET;
    fl_set_button(BT_MOTION_NAV_OBJ,0);
    fl_set_button(BT_MOTION_MANIP_OBJ,0);
    fl_set_button(BT_MOTION_OBJR_OBJ,1);
    if(OBJSET==NULL){
      fl_deactivate_object(PATHGROUP);
      fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
      fl_deactivate_object(HUMANGROUP);
      fl_set_object_color(HUMANGROUPFR,FL_RED,FL_COL1);
      fl_deactivate_object(SHOWBTGROUP);
      fl_set_object_color(SHOWBTGROUPFR,FL_RED,FL_COL1);
      fl_deactivate_object(NAVGROUP);
      fl_set_object_color(NAVGROUPFR,FL_RED,FL_COL1);
    }
    else{
      fl_activate_object(PATHGROUP);
      fl_set_object_color(BT_PATH_FIND_OBJ,FL_GREEN,FL_COL1);
      fl_activate_object(HUMANGROUP);
      fl_set_object_color(HUMANGROUPFR,FL_GREEN,FL_COL1);
      fl_activate_object(SHOWBTGROUP);
      fl_set_object_color(SHOWBTGROUPFR,FL_GREEN,FL_COL1);
      fl_activate_object(NAVGROUP);
      fl_set_object_color(NAVGROUPFR,FL_GREEN,FL_COL1);
    }
    fl_deactivate_object(MANIPGROUP);
    fl_deactivate_object(NAVGROUP);
  }

  SELECTED_BTSET = arg;

}

/* -------- Initialization of Bitmaps -------- */
static void CB_motion_init_obj(FL_OBJECT *obj, long arg)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int dimx,dimy,dimz;
  int i;
  double objx, objy, objz;

  /* NAVIGATION */
  if(SELECTED_BTSET==1){
    if(BTSET != NULL)
      hri_bt_destroy_bitmapset(BTSET);

    dimx = (int)((env->box.x2 - env->box.x1)/BT_SAMPLING);
    dimy = (int)((env->box.y2 - env->box.y1)/BT_SAMPLING);
    dimz = 1;
    BTSET = hri_bt_create_bitmaps();
    hri_bt_init_bitmaps(BTSET,dimx,dimy,dimz,BT_SAMPLING);
    hri_bt_change_bitmap_position(BTSET,env->box.x1,env->box.y1,
																	BTSET->robot->joints[ROBOTj_BASE]->dof_data[2].v);

    ACBTSET = BTSET;
    fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
  }
  /* MANIPULATION */
  if(SELECTED_BTSET==2){
    if(INTERPOINT != NULL)
      hri_bt_destroy_bitmapset(INTERPOINT);

    INTERPOINT = hri_exp_init();
    ACBTSET = INTERPOINT;
    fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
  }
  /* OBJECT REACH */
  if(SELECTED_BTSET==3){
    if(OBJSET != NULL)
      hri_bt_destroy_bitmapset(OBJSET);

    for(i=0; i<env->nr; i++){
      if( !strcmp("bottle",env->robot[i]->name) )
				break;
    }
    if(i==env->nr){
      printf("No bottle in the environment\n");
      return;
    }
    objx =  env->robot[i]->joints[1]->abs_pos[0][3];
    objy =  env->robot[i]->joints[1]->abs_pos[1][3];
    objz =  env->robot[i]->joints[1]->abs_pos[2][3];

    OBJSET = hri_object_reach_init(objx,objy,objz);
    ACBTSET = OBJSET;
    fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
  }

  if(!HUMAN_FORM_CREATED){
    for(i=0; i<ACBTSET->human_no; i++)
      fl_addto_choice(BT_HUMAN_ACTUAL_OBJ, ACBTSET->human[i]->HumanPt->name);
    if(ACBTSET->human_no > 0){
      for(i=0; i<ACBTSET->human[ACBTSET->actual_human]->states_no; i++)
				fl_addto_choice(BT_HUMAN_STATE_OBJ, ACBTSET->human[ACBTSET->actual_human]->state[i].name);
      fl_addto_choice(BT_HUMAN_EXISTS_OBJ,"not exist");
      fl_addto_choice(BT_HUMAN_EXISTS_OBJ,"exist");
    }
    HUMAN_FORM_CREATED = TRUE;
  } else {
    fl_set_choice(BT_HUMAN_EXISTS_OBJ, 1); // 0 =nothing, 1 =not exist
  }
  fl_set_button(BT_SHOWBT_GNUPLOT_OBJ,0);
  fl_set_button(BT_SHOWBT_DIST_OBJ,0);
  fl_set_button(BT_SHOWBT_VIS_OBJ,0);
  fl_set_button(BT_SHOWBT_HZAC_OBJ,0);
  fl_set_button(BT_SHOWBT_OBS_OBJ,0);
  fl_set_button(BT_SHOWBT_COMB_OBJ,0);
  fl_set_button(BT_SHOWBT_PATH_OBJ,0);

  CB_motion_obj(BT_MOTION_INIT_OBJ, SELECTED_BTSET);


}

/* ------------------ PATH GROUP ----------------------- */
static void g3d_create_path_group(void)
{
  FL_OBJECT *obj;

  obj = fl_add_labelframe(FL_ENGRAVED_FRAME,300,60,90,70,"Path");

  PATHGROUP = fl_bgn_group();

  BT_PATH_FIND_OBJ = fl_add_button(FL_NORMAL_BUTTON,310,90,70,35,"Find Path");
  fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
  fl_set_call_back(BT_PATH_FIND_OBJ,CB_path_find_obj,1);

  fl_end_group();

  fl_deactivate_object(PATHGROUP);

}

/* Calculate the path for the chosen bitmap */
static void CB_path_find_obj(FL_OBJECT *obj, long arg)
{
  configPt qs, qg, qresult;
  int res;
  int nb;
  p3d_rob* robotPt;

  if(SELECTED_BTSET == 1){

//    if(ACBTSET!=NULL){
//      hri_bt_reset_path(ACBTSET);
//    }
    if(BTGRAPH!=NULL){
      p3d_del_graph(BTGRAPH);
      BTGRAPH = NULL;
    }

    qs = p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    qg = p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_GOTO);

    MY_ALLOC_INFO("Avant la creation du graphe");

    res = hri_bt_calculate_bitmap_path(ACBTSET,ACBTSET->robot,qs,qg,FALSE);
    p3d_destroy_config(ACBTSET->robot, qs);

    if(!res){
      printf("p3d_hri_planner : FAIL : cannot find a path\n");
    }
    else{
      robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
      p3d_sel_desc_name(P3D_ROBOT,ACBTSET->robot->name);

      p3d_graph_to_traj(BTSET->robot);
      g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);

      ENV.setBool(Env::drawTraj,true);

      while( (qs=g3d_bt_dynamic_tshow(ACBTSET->robot,my_drawtraj_fct,&nb)) ){
        qresult = hri_bt_set_TARGET();
        if(qresult != NULL)
          qg = qresult;
        p3d_del_graph(BTGRAPH);
        p3d_del_graph(ACBTSET->robot->GRAPH);
        ACBTSET->robot->GRAPH = NULL;
        BTGRAPH = NULL;
        //	hri_bt_reset_path(BTSET);
        if (BTSET->robot->tcur != NULL) {
          p3d_del_traj(BTSET->robot->tcur);
        }
        ACBTSET->robot->tcur = NULL;
        hri_bt_calculate_bitmap_path(ACBTSET,ACBTSET->robot,qs,qg,FALSE);
        robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
        p3d_sel_desc_name(P3D_ROBOT,ACBTSET->robot->name);
        p3d_graph_to_traj(ACBTSET->robot);
        p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
        /* g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ)); */
        //printf("image\n");
        //g3d_save_movie_image();
      }
    }
  }
  if(SELECTED_BTSET == 2){

    if(ACBTSET!=NULL){
      hri_bt_reset_path(ACBTSET);
    }
    if(BTGRAPH!=NULL){
      p3d_del_graph(BTGRAPH);
      BTGRAPH = NULL;
    }

    if (hri_exp_find_manip_path(ACBTSET)) {
      robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
      ENV.setBool(Env::drawGraph,true);
      p3d_graph_to_traj(ACBTSET->robot);
    g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);

      ENV.setBool(Env::drawTraj,true);
      hri_exp_find_exchange_point();
    }
  }
  if(SELECTED_BTSET == 3){
    if(ACBTSET!=NULL){
      hri_bt_reset_path(ACBTSET);
    }
    if(BTGRAPH!=NULL){
      p3d_del_graph(BTGRAPH);
      BTGRAPH = NULL;
    }

    hri_exp_find_obj_reach_path(ACBTSET);

    robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
    p3d_graph_to_traj(ACBTSET->robot);
    g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);

    ENV.setBool(Env::drawTraj,true);

  }
  fl_set_button(BT_PATH_FIND_OBJ,0);
  g3d_draw_allwin_active();


}

/* ------------------- HUMAN GROUP ------------------ */
static void g3d_create_human_group(void)
{
  HUMANGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,10,140,380,40,"Humans");

  HUMANGROUP = fl_bgn_group();

  BT_HUMAN_ACTUAL_OBJ = fl_add_choice(FL_NORMAL_CHOICE,20,150,70,20,"");
  BT_HUMAN_EXISTS_OBJ = fl_add_choice(FL_NORMAL_CHOICE,110,150,70,20,"");
  BT_HUMAN_STATE_OBJ  = fl_add_choice(FL_NORMAL_CHOICE,220,150,70,20,"");

  fl_add_text(FL_NORMAL_TEXT,90,150,20,20,"do");
  fl_add_text(FL_NORMAL_TEXT,180,150,40,20,"and is");
  fl_set_call_back(BT_HUMAN_ACTUAL_OBJ,CB_human_actual_obj,1);
  fl_set_call_back(BT_HUMAN_EXISTS_OBJ,CB_human_exists_obj,1);
  fl_set_call_back(BT_HUMAN_STATE_OBJ,CB_human_state_obj,1);

  fl_end_group();

  fl_set_object_color(HUMANGROUPFR,FL_RED,FL_COL1);
  fl_deactivate_object(HUMANGROUP);
}


static void CB_human_actual_obj(FL_OBJECT *obj, long arg)
{
  int val = fl_get_choice(obj);

  ACBTSET->actual_human = val-1;

  fl_set_choice(BT_HUMAN_STATE_OBJ,ACBTSET->human[ACBTSET->actual_human]->actual_state+1);
  fl_set_choice(BT_HUMAN_EXISTS_OBJ,ACBTSET->human[ACBTSET->actual_human]->exists+1);

  if(fl_get_button(BT_NAV_DIST_OBJ)){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dheight);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dradius);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

    fl_activate_object(BT_NAV_PARAM1_OBJ);
    fl_activate_object(BT_NAV_PARAM2_OBJ);
    fl_deactivate_object(BT_NAV_PARAM3_OBJ);
    fl_deactivate_object(BT_NAV_PARAM4_OBJ);
  }
  if(fl_get_button(BT_NAV_VIS_OBJ)){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vheight);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vback);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vradius);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

    fl_activate_object(BT_NAV_PARAM1_OBJ);
    fl_activate_object(BT_NAV_PARAM2_OBJ);
    fl_activate_object(BT_NAV_PARAM3_OBJ);
    fl_deactivate_object(BT_NAV_PARAM4_OBJ);

  }
  if(fl_get_button(BT_NAV_HZ_OBJ)){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].hradius);

    fl_deactivate_object(BT_NAV_PARAM1_OBJ);
    fl_deactivate_object(BT_NAV_PARAM2_OBJ);
    fl_deactivate_object(BT_NAV_PARAM3_OBJ);
    fl_activate_object(BT_NAV_PARAM4_OBJ);
  }

  if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
  if(INTERPOINT!=NULL){
    hri_bt_3drefresh_all(INTERPOINT);
  }

	g3d_draw_allwin_active();
}

static void CB_human_exists_obj(FL_OBJECT *obj, long arg)
{
  int val = fl_get_choice(obj);

  ACBTSET->human[ACBTSET->actual_human]->exists = val-1;
  ACBTSET->changed = TRUE;
  if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
  if(INTERPOINT!=NULL)
    hri_bt_3drefresh_all(INTERPOINT);

  g3d_draw_allwin_active();
}

static void CB_human_state_obj(FL_OBJECT *obj, long arg)
{
  int new_state = fl_get_choice(obj);
  configPt q;

  // prevent segfault when list not initialized
  if (new_state < 1 )
    return;
  new_state--; // states being with 0

//  ACBTSET->human[ACBTSET->actual_human]->actual_state = new_state;// changed below

  if(fl_get_button(BT_NAV_DIST_OBJ)){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[new_state].dheight);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[new_state].dradius);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

    fl_activate_object(BT_NAV_PARAM1_OBJ);
    fl_activate_object(BT_NAV_PARAM2_OBJ);
    fl_deactivate_object(BT_NAV_PARAM3_OBJ);
    fl_deactivate_object(BT_NAV_PARAM4_OBJ);
  }
  if(fl_get_button(BT_NAV_VIS_OBJ)){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[new_state].vheight);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[new_state].vback);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[new_state].vradius);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

    fl_activate_object(BT_NAV_PARAM1_OBJ);
    fl_activate_object(BT_NAV_PARAM2_OBJ);
    fl_activate_object(BT_NAV_PARAM3_OBJ);
    fl_deactivate_object(BT_NAV_PARAM4_OBJ);

  }
  if(fl_get_button(BT_NAV_HZ_OBJ)){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[new_state].hradius);

    fl_deactivate_object(BT_NAV_PARAM1_OBJ);
    fl_deactivate_object(BT_NAV_PARAM2_OBJ);
    fl_deactivate_object(BT_NAV_PARAM3_OBJ);
    fl_activate_object(BT_NAV_PARAM4_OBJ);
  }

  q = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
//  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c7;
//  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c1;
//  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c2;
//  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c3;
//  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c4;
//  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c5;
//  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c6;
  hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], new_state, q, FALSE);

  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,q);

  p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,q);

  ACBTSET->changed = TRUE;
  if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
  if(INTERPOINT!=NULL){
    hri_bt_3drefresh_all(INTERPOINT);
  }

  g3d_draw_allwin_active();
}

/* ------------------------------------------------------- */
static void g3d_create_showbt_group(void)
{
  int i;

  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	
  SHOWBTGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,10,190,305,50,"Show Bitmaps");

  SHOWBTGROUP = fl_bgn_group();

  BT_SHOWBT_GNUPLOT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,20,200,50,30,"GnuPlot");
  BT_SHOWBT_DIST_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,80,195,50,20,"Distance");
  BT_SHOWBT_VIS_OBJ  = fl_add_checkbutton(FL_PUSH_BUTTON,80,215,50,20,"Visibility");
  BT_SHOWBT_HZAC_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,140,195,50,20,"HZ/AC");
  BT_SHOWBT_OBS_OBJ  = fl_add_checkbutton(FL_PUSH_BUTTON,140,215,50,20,"Obstacles");
  BT_SHOWBT_COMB_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,210,195,50,20,"Combined");
  BT_SHOWBT_PATH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,210,215,50,20,"Path");

  fl_set_call_back(BT_SHOWBT_GNUPLOT_OBJ,CB_showbt_gnuplot_obj,1);
  fl_set_call_back(BT_SHOWBT_DIST_OBJ,CB_showbt_obj,1);
  fl_set_call_back(BT_SHOWBT_VIS_OBJ,CB_showbt_obj,2);
  fl_set_call_back(BT_SHOWBT_HZAC_OBJ,CB_showbt_obj,3);
  fl_set_call_back(BT_SHOWBT_OBS_OBJ,CB_showbt_obj,4);
  fl_set_call_back(BT_SHOWBT_COMB_OBJ,CB_showbt_obj,5);
  fl_set_call_back(BT_SHOWBT_PATH_OBJ,CB_showbt_obj,6);

  fl_end_group();

  fl_set_object_color(SHOWBTGROUPFR,FL_RED,FL_COL1);
  fl_deactivate_object(SHOWBTGROUP);

  for(i=0; i<5; i++)
    gnuplots[i] = hri_bt_init_gnuplot(env->box.x1,env->box.x2,env->box.y1,env->box.y2,env->box.z1,env->box.z2); 
}

static void CB_showbt_gnuplot_obj(FL_OBJECT *obj, long arg)
{
  int i;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	
  if(GNUPLOT_ACTIVE){
    GNUPLOT_ACTIVE = FALSE;
    for(i=0; i<5; i++){
      gnuplot_close(gnuplots[i]);
      gnuplots[i] = hri_bt_init_gnuplot(env->box.x1,env->box.x2,env->box.y1,env->box.y2,env->box.z1,env->box.z2);
    }
  }
  else{
    GNUPLOT_ACTIVE = TRUE;
  }
}

static void CB_showbt_obj(FL_OBJECT *obj, long arg)
{
  if(SELECTED_BTSET == 1){
    if(arg == 1){
      if(!hri_bt_is_active(BT_DISTANCE,BTSET)) hri_bt_activate(BT_DISTANCE,BTSET);
      else                                     hri_bt_desactivate(BT_DISTANCE,BTSET);
    }
    if(arg == 2){
      if(!hri_bt_is_active(BT_VISIBILITY,BTSET)) hri_bt_activate(BT_VISIBILITY,BTSET);
      else                                       hri_bt_desactivate(BT_VISIBILITY,BTSET);
    }
    if(arg == 3){
      if(!hri_bt_is_active(BT_HIDZONES,BTSET)) hri_bt_activate(BT_HIDZONES,BTSET);
      else                                     hri_bt_desactivate(BT_HIDZONES,BTSET);
    }
    if(arg == 4){
      if(!hri_bt_is_active(BT_OBSTACLES,BTSET)) hri_bt_activate(BT_OBSTACLES,BTSET);
      else                                      hri_bt_desactivate(BT_OBSTACLES,BTSET);
    }
    if(arg == 5){
      if(!hri_bt_is_active(BT_COMBINED,BTSET)) hri_bt_activate(BT_COMBINED,BTSET);
      else                                     hri_bt_desactivate(BT_COMBINED,BTSET);
    }
    if(arg == 6){
      if(!hri_bt_is_active(BT_PATH,BTSET)) hri_bt_activate(BT_PATH,BTSET);
      else                                 hri_bt_desactivate(BT_PATH,BTSET);
    }
  }
  if(SELECTED_BTSET == 2){
    if(GNUPLOT_ACTIVE){
      if(arg == 1){
				if(fl_get_button(BT_SHOWBT_DIST_OBJ)){
					hri_bt_fill_bitmap(INTERPOINT,BT_3D_DISTANCE);
					hri_bt_gnuplot_bitmap(gnuplots[arg-1],INTERPOINT,BT_3D_DISTANCE,10);
				}
				else{
					gnuplot_close(gnuplots[arg-1]);
					gnuplots[arg-1] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
				}
      }
      if(arg == 2){
				if(fl_get_button(BT_SHOWBT_VIS_OBJ)){
					hri_bt_fill_bitmap(INTERPOINT,BT_3D_VISIBILITY);
					hri_bt_gnuplot_bitmap(gnuplots[arg-1],INTERPOINT,BT_3D_VISIBILITY,10);
				}
				else{
					gnuplot_close(gnuplots[arg-1]);
					gnuplots[arg-1] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
				}
      }
      if(arg == 3){
				if(fl_get_button(BT_SHOWBT_HZAC_OBJ)){
					hri_bt_fill_bitmap(INTERPOINT,BT_3D_HCOMFORT);
					hri_bt_gnuplot_bitmap(gnuplots[arg-1],INTERPOINT,BT_3D_HCOMFORT,10);
				}
				else{
					gnuplot_close(gnuplots[arg-1]);
					gnuplots[arg-1] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
				}
      }
			if(arg == 5){
				if(fl_get_button(BT_SHOWBT_COMB_OBJ)){
					hri_bt_fill_bitmap(INTERPOINT,BT_3D_COMBINED);
					hri_bt_gnuplot_bitmap(gnuplots[arg-1],INTERPOINT,BT_3D_COMBINED,10);
				}
				else{
					gnuplot_close(gnuplots[arg-1]);
					gnuplots[arg-1] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
				}
      }
    }
    else
    {
        if(arg == 1){
          if(!hri_bt_is_active(BT_DISTANCE,INTERPOINT)) hri_bt_activate(BT_DISTANCE,INTERPOINT);
          else                                     hri_bt_desactivate(BT_DISTANCE,INTERPOINT);
        }
        if(arg == 2){
          if(!hri_bt_is_active(BT_VISIBILITY,INTERPOINT)) hri_bt_activate(BT_VISIBILITY,INTERPOINT);
          else                                       hri_bt_desactivate(BT_VISIBILITY,INTERPOINT);
        }
        if(arg == 3){
          if(!hri_bt_is_active(BT_HIDZONES,INTERPOINT)) hri_bt_activate(BT_HIDZONES,INTERPOINT);
          else                                     hri_bt_desactivate(BT_HIDZONES,INTERPOINT);
        }
        if(arg == 4){
          if(!hri_bt_is_active(BT_OBSTACLES,INTERPOINT)) hri_bt_activate(BT_OBSTACLES,INTERPOINT);
          else                                      hri_bt_desactivate(BT_OBSTACLES,INTERPOINT);
        }
        if(arg == 5){
          if(!hri_bt_is_active(BT_COMBINED,INTERPOINT)) hri_bt_activate(BT_COMBINED,INTERPOINT);
          else                                     hri_bt_desactivate(BT_COMBINED,INTERPOINT);
        }
        if(arg == 6){
          if(!hri_bt_is_active(BT_PATH,INTERPOINT)) hri_bt_activate(BT_PATH,INTERPOINT);
          else                                 hri_bt_desactivate(BT_PATH,INTERPOINT);
        }
    }
  }
  g3d_draw_allwin_active();
}

/* ------------------------------------------------------- */
static void g3d_create_save_bitmaps_obj(void)
{

  BT_SAVE_BITMAPS_OBJ = fl_add_button(FL_NORMAL_BUTTON,321,189,69,51,"Save\nBitmaps");
  fl_set_call_back(BT_SAVE_BITMAPS_OBJ,CB_save_bitmaps_obj,1);

  fl_deactivate_object(BT_SAVE_BITMAPS_OBJ);
}

static void CB_save_bitmaps_obj(FL_OBJECT *obj, long arg)
{
  printf("Saving grids is not yet implemented.\n");
}

/* ------------------------------------------------------- */
static void g3d_create_nav_group(void)
{
  NAVGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,10,250,380,80,"Navigation Parameters");

  NAVGROUP = fl_bgn_group();

  BT_NAV_DIST_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,260,50,20,"Distance");
  fl_set_call_back(BT_NAV_DIST_OBJ,CB_nav_btchoice_obj,1);
  BT_NAV_VIS_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,275,50,20,"Visibility");
  fl_set_call_back(BT_NAV_VIS_OBJ,CB_nav_btchoice_obj,2);
  BT_NAV_HZ_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,290,50,20,"Hiddens");
  fl_set_call_back(BT_NAV_HZ_OBJ,CB_nav_btchoice_obj,3);
  BT_NAV_LEN_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,305,50,20,"Length");
  fl_set_call_back(BT_NAV_LEN_OBJ,CB_nav_btchoice_obj,4);

  BT_NAV_PARAM1_OBJ = fl_add_valslider(FL_HOR_SLIDER,80,260,245,15,"");
  fl_set_slider_step(BT_NAV_PARAM1_OBJ,10);
  fl_set_slider_bounds(BT_NAV_PARAM1_OBJ,0,500);
  fl_set_slider_value(BT_NAV_PARAM1_OBJ,500);
  fl_set_object_callback(BT_NAV_PARAM1_OBJ,CB_nav_param_obj,1);
  BT_NAV_PARAM2_OBJ = fl_add_valslider(FL_HOR_SLIDER,80,275,245,15,"");
  fl_set_slider_step(BT_NAV_PARAM2_OBJ,0.05);
  fl_set_slider_bounds(BT_NAV_PARAM2_OBJ,0,20);
  fl_set_slider_value(BT_NAV_PARAM2_OBJ,0);
  fl_set_object_callback(BT_NAV_PARAM2_OBJ,CB_nav_param_obj,2);
  BT_NAV_PARAM3_OBJ = fl_add_valslider(FL_HOR_SLIDER,80,290,245,15,"");
  fl_set_slider_step(BT_NAV_PARAM3_OBJ,0.2);
  fl_set_slider_bounds(BT_NAV_PARAM3_OBJ,0,5);
  fl_set_slider_value(BT_NAV_PARAM3_OBJ,2);
  fl_set_object_callback(BT_NAV_PARAM3_OBJ,CB_nav_param_obj,3);
  BT_NAV_PARAM4_OBJ = fl_add_valslider(FL_HOR_SLIDER,80,305,245,15,"");
  fl_set_slider_step(BT_NAV_PARAM4_OBJ,0.2);
  fl_set_slider_bounds(BT_NAV_PARAM4_OBJ,0,5);
  fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);
  fl_set_object_callback(BT_NAV_PARAM4_OBJ,CB_nav_param_obj,4);

  fl_end_group();
  fl_set_object_color(NAVGROUPFR,FL_RED,FL_COL1);
  fl_deactivate_object(NAVGROUP);
}

static void CB_nav_btchoice_obj(FL_OBJECT *obj, long arg)
{
  if(ACBTSET==NULL){
    return;
  }

  if(arg==1){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dheight);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dradius);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

    fl_activate_object(BT_NAV_PARAM1_OBJ);
    fl_activate_object(BT_NAV_PARAM2_OBJ);
    fl_deactivate_object(BT_NAV_PARAM3_OBJ);
    fl_deactivate_object(BT_NAV_PARAM4_OBJ);
  }
  if(arg==2){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vheight);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vback);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vradius);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

    fl_activate_object(BT_NAV_PARAM1_OBJ);
    fl_activate_object(BT_NAV_PARAM2_OBJ);
    fl_activate_object(BT_NAV_PARAM3_OBJ);
    fl_deactivate_object(BT_NAV_PARAM4_OBJ);

  }
  if(arg==3){
    fl_set_slider_value(BT_NAV_PARAM1_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM2_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
    fl_set_slider_value(BT_NAV_PARAM4_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].hradius);

    fl_deactivate_object(BT_NAV_PARAM1_OBJ);
    fl_deactivate_object(BT_NAV_PARAM2_OBJ);
    fl_deactivate_object(BT_NAV_PARAM3_OBJ);
    fl_activate_object(BT_NAV_PARAM4_OBJ);
  }
  if(arg==4){
      fl_set_slider_value(BT_NAV_PARAM1_OBJ, ACBTSET->parameters->path_length_weight);
      fl_set_slider_value(BT_NAV_PARAM2_OBJ,0);
      fl_set_slider_value(BT_NAV_PARAM3_OBJ,0);
      fl_set_slider_value(BT_NAV_PARAM4_OBJ,0);

      fl_activate_object(BT_NAV_PARAM1_OBJ);
      fl_deactivate_object(BT_NAV_PARAM2_OBJ);
      fl_deactivate_object(BT_NAV_PARAM3_OBJ);
      fl_deactivate_object(BT_NAV_PARAM4_OBJ);
    }
}

/** updates the visuals after changing the slider values */
static void CB_nav_param_obj(FL_OBJECT *obj, long arg)
{

  if(fl_get_button(BT_NAV_DIST_OBJ)){
    hri_bt_update_distance(BTSET,
													 fl_get_slider_value(BT_NAV_PARAM1_OBJ),
													 fl_get_slider_value(BT_NAV_PARAM2_OBJ));
  } else if(fl_get_button(BT_NAV_VIS_OBJ)){
    hri_bt_update_visibility(BTSET,
														 fl_get_slider_value(BT_NAV_PARAM1_OBJ),
														 fl_get_slider_value(BT_NAV_PARAM2_OBJ),
														 fl_get_slider_value(BT_NAV_PARAM3_OBJ));
  } else if(fl_get_button(BT_NAV_HZ_OBJ)){
    hri_bt_update_hidzones(BTSET,
													 fl_get_slider_value(BT_NAV_PARAM4_OBJ));
  } else if(fl_get_button(BT_NAV_LEN_OBJ)){
    BTSET->parameters->path_length_weight = fl_get_slider_value(BT_NAV_PARAM1_OBJ);
  }

  g3d_draw_allwin_active();
}


/* ------------------------------------------------------- */
static void g3d_create_manip_group(void)
{
  MANIPGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,10,340,275,80,"Object Handling Parameters");

  MANIPGROUP = fl_bgn_group();

  fl_add_labelframe(FL_ENGRAVED_FRAME,290,340,100,80,"Exchange Point");
  BT_MANIP_EXP_NO_OBJ = fl_add_input(FL_NORMAL_INPUT,340,350,40,20,"Number");
  BT_MANIP_EXP_FIND_OBJ = fl_add_button(FL_NORMAL_BUTTON,300,380,40,30,"Find");
  BT_MANIP_EXP_SHOW_OBJ = fl_add_button(FL_NORMAL_BUTTON,340,380,40,30,"Show");

  fl_set_call_back(BT_MANIP_EXP_NO_OBJ,CB_manip_exp_no_obj,1);
  fl_set_call_back(BT_MANIP_EXP_FIND_OBJ,CB_manip_exp_find_obj,1);
  fl_set_call_back(BT_MANIP_EXP_SHOW_OBJ,CB_manip_exp_show_obj,1);

  fl_end_group();
  fl_set_object_color(MANIPGROUPFR,FL_RED,FL_COL1);
  fl_deactivate_object(MANIPGROUP);

}

static void CB_manip_exp_no_obj(FL_OBJECT *obj, long arg)
{

}

static void CB_manip_exp_find_obj(FL_OBJECT *obj, long arg)
{

}

static void CB_manip_exp_show_obj(FL_OBJECT *obj, long arg)
{

}

/* ------------------------------------------------------- */
static void g3d_create_GIK_group(void)
{
  FL_OBJECT *obj;
	p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	int i;
	int framex = 10, framey = 430;

	obj = fl_add_labelframe(FL_BORDER_FRAME,framex,framey,380,60,"GIK Parameters");

  GIKGROUP = fl_bgn_group();
	GIK_RUN_OBJ = fl_add_button(FL_NORMAL_BUTTON,framex+10,framey+30,70,25,"Run GIK");
  GIK_TARGET_ROBOT_OBJ = fl_add_choice(FL_NORMAL_CHOICE,framex+10,framey+7,70,20,"");
	GIK_VIS_OBJ = fl_add_valslider(FL_HOR_SLIDER,framex+150,framey+7,100,15,"");
	GIK_PRECISION_OBJ = fl_add_valslider(FL_HOR_SLIDER,framex+150,framey+23,100,15,"");
  GIK_STEP_OBJ = fl_add_valslider(FL_HOR_SLIDER,framex+150,framey+39,100,15,"");
	fl_add_text(FL_NORMAL_TEXT,framex+90,framey+5,60,20,"Vis. Step");
	fl_add_text(FL_NORMAL_TEXT,framex+90,framey+21,60,20,"Precision");
	fl_add_text(FL_NORMAL_TEXT,framex+90,framey+37,60,20,"Iterations");
	GIK_JOINTSEL_OBJ = fl_add_button(FL_PUSH_BUTTON,framex+255,framey+7,80,30,"Joint Selection");

	for(i=0; i<env->nr; i++){
		fl_addto_choice(GIK_TARGET_ROBOT_OBJ,env->robot[i]->name);
	}
	GIK_target_robot = env->robot[0];

	fl_set_call_back(GIK_RUN_OBJ,CB_gik_run_obj,1);
	fl_set_call_back(GIK_TARGET_ROBOT_OBJ,CB_gik_target_robot_obj,1);

  fl_set_object_callback(GIK_VIS_OBJ,CB_gik_vis_obj,1);
	fl_set_slider_step(GIK_VIS_OBJ,10);
  fl_set_slider_bounds(GIK_VIS_OBJ,1,500);
  fl_set_slider_value(GIK_VIS_OBJ,GIK_VIS);

	fl_set_object_callback(GIK_PRECISION_OBJ,CB_gik_precision_obj,1);
	fl_set_slider_step(GIK_PRECISION_OBJ,0.01);
  fl_set_slider_bounds(GIK_PRECISION_OBJ,0,1);
  fl_set_slider_value(GIK_PRECISION_OBJ,GIK_PRECISION);

	fl_set_object_callback(GIK_STEP_OBJ,CB_gik_step_obj,1);
	fl_set_slider_step(GIK_STEP_OBJ,10);
  fl_set_slider_bounds(GIK_STEP_OBJ,1,500);
  fl_set_slider_value(GIK_STEP_OBJ,GIK_STEP);

	fl_set_object_callback(GIK_JOINTSEL_OBJ,CB_gik_jointsel_obj,0);

  fl_end_group();
	fl_set_object_color(obj,FL_GREEN,FL_COL1);
}

void CB_gik_jointsel_obj(FL_OBJECT *obj, long arg)
{
	if(fl_get_button(obj)) g3d_show_gik_jointsel_form();
  else                   g3d_hide_gik_jointsel_form();
}

/* ------------------------------------------------------- */
static void g3d_create_TEST_group(void)
{
	FL_OBJECT *obj;
	int framex = 10, framey = 500;
	
	obj = fl_add_labelframe(FL_BORDER_FRAME,framex,framey,380,70,"TEST");
	
	TESTGROUP = fl_bgn_group();
	TEST_BUTTON1_OBJ = fl_add_button(FL_NORMAL_BUTTON,framex+10,framey+10,50,50,"TEST1");
	TEST_BUTTON2_OBJ = fl_add_button(FL_NORMAL_BUTTON,framex+70,framey+10,50,50,"TEST2");
	TEST_BUTTON3_OBJ = fl_add_button(FL_NORMAL_BUTTON,framex+130,framey+10,50,50,"TEST3");
	TEST_BUTTON4_OBJ = fl_add_button(FL_NORMAL_BUTTON,framex+190,framey+10,50,50,"TEST4");
	
	fl_set_call_back(TEST_BUTTON1_OBJ,CB_test_button1_obj,1);
	fl_set_call_back(TEST_BUTTON2_OBJ,CB_test_button2_obj,1);
	fl_set_call_back(TEST_BUTTON3_OBJ,CB_test_button3_obj,1);
	fl_set_call_back(TEST_BUTTON4_OBJ,CB_test_button4_obj,1);
	
	fl_end_group();
}

void CB_test_button1_obj(FL_OBJECT *obj, long arg)
{
	p3d_env * env;
  int i;
	p3d_rob * temprobot1 = NULL;
	p3d_rob * temprobot2 = NULL;
	p3d_vector4 targetRealCoord;
  p3d_vector4 targetRelativeCoord;
  p3d_matrix4 inv;
  double phi, theta;
  int is_far;
	
  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  for(i=0; i<env->nr; i++){
    if( !strcmp("human1",env->robot[i]->name) ){
      temprobot1 = env->robot[i];
      continue;
    }
    if( !strcmp("TRASHBIN",env->robot[i]->name) ){
      temprobot2 = env->robot[i];
      continue;
    }
  }
	targetRealCoord[0] = temprobot2->joints[1]->abs_pos[0][3];
  targetRealCoord[1] = temprobot2->joints[1]->abs_pos[1][3];
  targetRealCoord[2] = temprobot2->joints[1]->abs_pos[2][3];
  targetRealCoord[3] = 1;
  
  p3d_matInvertXform(temprobot1->joints[1]->abs_pos, inv);
  
  p3d_matvec4Mult(inv, targetRealCoord, targetRelativeCoord);
  
  p3d_psp_cartesian2spherical(targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2],
															0,0,0,&phi,&theta);
  
	
  is_far = (2 > DISTANCE2D(targetRelativeCoord[0],targetRelativeCoord[1],0,0));
	
	
  printf("Phi is %f, is_far is %d\n",phi,is_far);
	
  /* Phi is the horizontal one */
  
  if(ABS(phi) < M_PI/8){ /* In front */
    if(is_far) printf("S_mhp_F_F\n");
    else   printf("S_mhp_N_F\n");
  }
  if(M_PI/8 <= phi && phi < 3*M_PI/8){ /* Front left */
    if(is_far)  printf("S_mhp_F_FL\n");
    else   printf("S_mhp_N_FL\n");
  }
  if(3*M_PI/8 <= phi && phi < 5*M_PI/8){ /* left */ 
    if(is_far)   printf("S_mhp_F_L\n");
    else    printf("S_mhp_N_L\n");
  }
  if(5*M_PI/8 <= phi && phi < 7*M_PI/8){ /* Back left */
    if(is_far)  printf("S_mhp_F_BL\n");
    else   printf("S_mhp_N_BL\n");
  }
  if(7*M_PI/8 <= ABS(phi)){ /* Back */
    if(is_far)  printf("S_mhp_F_B\n");
    else   printf("S_mhp_N_B\n");
  }
  if(-7*M_PI/8 <= phi && phi < -5*M_PI/8){ /* Back right */
    if(is_far)  printf("S_mhp_F_BR\n");
    else  printf("S_mhp_N_FR\n");
  }
  if(-5*M_PI/8 <= phi && phi < -3*M_PI/8){ /* right */
    if(is_far) printf("S_mhp_F_R\n");
    else   printf("S_mhp_N_R\n");
  }
  if(-3*M_PI/8 <= phi && phi < -1*M_PI/8){ /* Front right */
    if(is_far)  printf("S_mhp_F_FR\n");
    else   printf("S_mhp_N_FR\n");
  }
	
  printf("Bad angle value, This shouldn't happen.\n");
	
}

void CB_test_button2_obj(FL_OBJECT *obj, long arg)
{
	
	psp_is_object_visible(BTSET->robot, BTSET->visball, 50);
	
	
	
}

void CB_test_button3_obj(FL_OBJECT *obj, long arg)
{
	p3d_init_robot_parameters();
	//persp_win = g3d_show_new_persp_win();
	
	g3d_draw_allwin_active();
	
	
}

void CB_test_button4_obj(FL_OBJECT *obj, long arg)
{
	double qs[3], qf[3];
	hri_bitmapset * bitmapset = NULL;
	hri_bitmap * bitmap;
	
	
	qs[0] = ACBTSET->robot->joints[25]->abs_pos[0][3];
	qs[1] = ACBTSET->robot->joints[25]->abs_pos[1][3];
	qs[2] = ACBTSET->robot->joints[25]->abs_pos[2][3];
	
	p3d_set_and_update_this_robot_conf(ACBTSET->robot, ACBTSET->robot->ROBOT_GOTO);
	
	qf[0] = ACBTSET->robot->joints[25]->abs_pos[0][3];
	qf[1] = ACBTSET->robot->joints[25]->abs_pos[1][3];
	qf[2] = ACBTSET->robot->joints[25]->abs_pos[2][3];
	
	bitmapset = ACBTSET;
	
	if(bitmapset==NULL || bitmapset->bitmap[BT_PATH]==NULL){
    PrintError(("Trying to find a path in a non existing bitmap or bitmapset\n"));    
    return ;
  }
  bitmap = bitmapset->bitmap[BT_PATH]; 
  
  bitmap->search_start = hri_bt_get_cell(bitmap,(int)((qs[0]-bitmapset->realx)/bitmapset->pace),
																				 (int)((qs[1]-bitmapset->realy)/bitmapset->pace),
																				 (int)((qs[2]-bitmapset->realz)/bitmapset->pace));  
  bitmap->search_goal  = hri_bt_get_cell(bitmap,(int)((qf[0]-bitmapset->realx)/bitmapset->pace),
																				 (int)((qf[1]-bitmapset->realy)/bitmapset->pace),
																				 (int)((qf[2]-bitmapset->realz)/bitmapset->pace)); 
  
  if(bitmap->search_start == NULL) {
    PrintWarning(("Search start cell does not exist\n"));
    bitmapset->pathexist = FALSE;
    return ;
  }
  if(bitmap->search_goal == NULL ){
    PrintWarning(("Search goal cell does not exist\n"));
    bitmapset->pathexist = FALSE;
    return ;
  }
	
	if(bitmapset->pathexist)
    hri_bt_reset_path(bitmapset);
	
  if( hri_bt_astar_bh(bitmapset,bitmap) > -1){
    bitmapset->pathexist = TRUE;
		printf("Success.\n");
    return ;
  }
  else{
    bitmapset->pathexist = FALSE;
		printf("Fail.\n");
    return ;
  }   
	
	
}

/*************************************** END NEW FUNCTIONS ****************************************/

/*******************DELETES**********************/

static void g3d_delete_find_model_q(void)
{
  if(persp_win != NULL)
    g3d_del_win(persp_win);
  fl_free_object(BT_FIND_MODEL_Q_POINT);
  fl_free_object(BT_WATCH_OBJECT);
  fl_free_object(BT_PSP_PARAMETERS_OBJECT);
}
