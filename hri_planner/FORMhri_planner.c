#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"


/* ------- FUNCTION VARIABLES ------- */

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
extern double GIK_PRECISION;
extern int GIK_STEP;
extern double GIK_FORCE;
extern p3d_rob * GIK_target_robot;

extern hri_shared_zone zone[];
extern int shared_zone_l;
extern int SWITCH_TO_GREEN;
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
static FL_OBJECT * TEST_BUTTON5_OBJ;

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
static void CB_test_button5_obj(FL_OBJECT *obj, long arg);

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

  hri_initialize_visibility();

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

  GLOBAL_AGENTS = hri_create_agents();
  hri_assign_source_agent("JIDOKUKA", GLOBAL_AGENTS);

  GLOBAL_ENTITIES = hri_create_entities();
  hri_refine_entity_types(GLOBAL_ENTITIES, GLOBAL_AGENTS);
  /* NAVIGATION */
  if(SELECTED_BTSET==1){
    if(BTSET != NULL)
      hri_bt_destroy_bitmapset(BTSET);

    dimx = (int)((env->box.x2 - env->box.x1)/BT_SAMPLING);
    dimy = (int)((env->box.y2 - env->box.y1)/BT_SAMPLING);
    dimz = 1;
    BTSET = hri_bt_create_bitmaps();
    hri_bt_init_bitmaps(BTSET,dimx,dimy,dimz,BT_SAMPLING);
    hri_bt_change_bitmap_position(BTSET,env->box.x1,env->box.y1, BTSET->robot->joints[ROBOTj_BASE]->dof_data[2].v);

    ACBTSET = BTSET;
    fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
  }
  /* MANIPULATION */
  if(SELECTED_BTSET==2){
    if(INTERPOINT != NULL)
      hri_bt_destroy_bitmapset(INTERPOINT);

    INTERPOINT = hri_exp_init();
    HRI_GIK = hri_gik_create_gik();
    ACBTSET = INTERPOINT;
    fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
  }
  /* OBJECT REACH */
  if(SELECTED_BTSET==3){
    if(OBJSET != NULL)
      hri_bt_destroy_bitmapset(OBJSET);

    for(i=0; i<env->nr; i++){
      if( strcasestr(env->robot[i]->name,"BOTTLE") )
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
    hri_exp_find_exchange_point();
    if (hri_exp_find_manip_path(ACBTSET)) {
      robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
      ENV.setBool(Env::drawGraph,true);
      p3d_graph_to_traj(ACBTSET->robot);
      g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);

      ENV.setBool(Env::drawTraj,true);

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
  // p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  if(GNUPLOT_ACTIVE){
    GNUPLOT_ACTIVE = FALSE;
    for(i=0; i<5; i++){
      gnuplot_close(gnuplots[i]);
      //	gnuplots[i] = hri_bt_init_gnuplot(env->box.x1,env->box.x2,env->box.y1,env->box.y2,env->box.z1,env->box.z2);
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
          gnuplots[arg-1] = hri_bt_init_gnuplot_bitmap(INTERPOINT,BT_3D_DISTANCE);
          hri_bt_gnuplot_bitmap(gnuplots[arg-1],INTERPOINT,BT_3D_DISTANCE,10);
        }
        else{
          gnuplot_close(gnuplots[arg-1]);
          //			gnuplots[arg-1] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
        }
      }
      if(arg == 2){
        if(fl_get_button(BT_SHOWBT_VIS_OBJ)){
          hri_bt_fill_bitmap(INTERPOINT,BT_3D_VISIBILITY);
          //		hri_bt_gnuplot_bitmap(gnuplots[arg-1],INTERPOINT,BT_3D_VISIBILITY,10);
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
  fl_set_slider_step(BT_NAV_PARAM1_OBJ,2);
  fl_set_slider_bounds(BT_NAV_PARAM1_OBJ,0,100);
  fl_set_slider_value(BT_NAV_PARAM1_OBJ,50);
  fl_set_object_callback(BT_NAV_PARAM1_OBJ,CB_nav_param_obj,1);
  BT_NAV_PARAM2_OBJ = fl_add_valslider(FL_HOR_SLIDER,80,275,245,15,"");
  fl_set_slider_step(BT_NAV_PARAM2_OBJ,0.05);
  fl_set_slider_bounds(BT_NAV_PARAM2_OBJ,0,50);
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
  TEST_BUTTON5_OBJ = fl_add_button(FL_NORMAL_BUTTON,framex+250,framey+10,50,50,"TEST5");

  fl_set_call_back(TEST_BUTTON1_OBJ,CB_test_button1_obj,1);
  fl_set_call_back(TEST_BUTTON2_OBJ,CB_test_button2_obj,1);
  fl_set_call_back(TEST_BUTTON3_OBJ,CB_test_button3_obj,1);
  fl_set_call_back(TEST_BUTTON4_OBJ,CB_test_button4_obj,1);
  fl_set_call_back(TEST_BUTTON5_OBJ,CB_test_button5_obj,1);

  fl_end_group();
}

void CB_test_button1_obj(FL_OBJECT *obj, long arg)
{
  HRI_AGENTS * agents;
  p3d_vector3 Tcoord[3];
  configPt q_r, q_h, q_hs, q_r_saved, q_h_saved, q_hs_saved;
  int i,j=0;
  int rreached = FALSE, hreached = FALSE;
  int robot_in_collision = FALSE;

  SWITCH_TO_GREEN = FALSE;

  /* CREATION OF AGENTS */
  agents = hri_create_agents();

  q_r = p3d_get_robot_config(agents->robots[0]->robotPt);
  q_h = p3d_get_robot_config(agents->humans[0]->robotPt);
  //q_hs = p3d_get_robot_config(agents->humans[1]->robotPt);
  //q_hs_saved = p3d_copy_config(agents->humans[1]->robotPt, agents->humans[1]->robotPt->ROBOT_POS);

  q_r_saved = p3d_get_robot_config(agents->robots[0]->robotPt);
  q_h_saved = p3d_get_robot_config(agents->humans[0]->robotPt);

  for(i=0; i<500; i++){

    // Shoot random position
    Tcoord[0][0] = Tcoord[1][0] = Tcoord[2][0] = p3d_random(agents->robots[0]->robotPt->joints[1]->abs_pos[0][3],
                                                            agents->humans[0]->robotPt->joints[1]->abs_pos[0][3]);
    Tcoord[0][1] = Tcoord[1][1] = Tcoord[2][1] = p3d_random(agents->robots[0]->robotPt->joints[1]->abs_pos[1][3]-0.5,
                                                            agents->humans[0]->robotPt->joints[1]->abs_pos[1][3]+0.5);
    Tcoord[0][2] = Tcoord[1][2] = Tcoord[2][2] = p3d_random(0.8, 1.5);

    // Test if Human can reach that position

    p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r_saved);
    rreached = hri_agent_single_task_manip_move(agents->robots[0], GIK_RATREACH, Tcoord, 0.05, &q_r);
    p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r);
    if(!rreached){
      p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r_saved);
      rreached = hri_agent_single_task_manip_move(agents->robots[0], GIK_LATREACH, Tcoord, 0.05, &q_r);
      p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r);
    }
    if(!rreached){
      zone[j].x = Tcoord[0][0]; zone[j].y = Tcoord[0][1]; zone[j].z = Tcoord[0][2];
      zone[j].value = 0; // 0 means robot not reached
      shared_zone_l++;
      j++;
      continue;
    }
    robot_in_collision = p3d_col_test_robot(agents->robots[0]->robotPt,FALSE);
//    if(p3d_col_test_robot(agents->robots[0]->robotPt,FALSE)){
//      zone[j].x = Tcoord[0][0]; zone[j].y = Tcoord[0][1]; zone[j].z = Tcoord[0][2];
//      zone[j].value = -1; // -1 means robot reached but in collision
//      shared_zone_l++;
//      j++;
//      continue;
//    }

    p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r_saved);

    // Robot Can reach
    // Test if Human can reach that position

    p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h_saved);
    hreached = hri_agent_single_task_manip_move(agents->humans[0], GIK_RATREACH, Tcoord, 0.02, &q_h);
    p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h);
    if(!hreached){
      p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h_saved);
      hreached = hri_agent_single_task_manip_move(agents->humans[0], GIK_LATREACH, Tcoord, 0.02, &q_h);
      p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h);
    }
    if(!hreached){
      zone[j].x = Tcoord[0][0]; zone[j].y = Tcoord[0][1]; zone[j].z = Tcoord[0][2];
      zone[j].value = 0; // human cannot reach
      shared_zone_l++;
      j++;
      continue;
    }
    if(robot_in_collision){
      zone[j].x = Tcoord[0][0]; zone[j].y = Tcoord[0][1]; zone[j].z = Tcoord[0][2];
      zone[j].value = -1; //robot has reached but in collision
      shared_zone_l++;
      j++;
      robot_in_collision = FALSE;
      continue;
    }

    if(p3d_col_test_robot(agents->humans[0]->robotPt,FALSE)){
      zone[j].x = Tcoord[0][0]; zone[j].y = Tcoord[0][1]; zone[j].z = Tcoord[0][2];
      zone[j].value = -1; //human has reached but in collision
      shared_zone_l++;
      j++;
      continue;
    }
    p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h_saved);
//    hreached = hri_agent_single_task_manip_move(agents->humans[1], GIK_LATREACH, Tcoord, &q_hs);
//    p3d_set_and_update_this_robot_conf(agents->humans[1]->robotPt,q_hs);

    zone[j].x = Tcoord[0][0]; zone[j].y = Tcoord[0][1]; zone[j].z = Tcoord[0][2];
    zone[j].value = 1; //they both reached without any collision
    shared_zone_l++;
    j++;

    g3d_draw_allwin_active();
  }
  p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h_saved);
  p3d_destroy_config(agents->robots[0]->robotPt,q_r);
  p3d_destroy_config(agents->humans[0]->robotPt,q_h);
  p3d_destroy_config(agents->robots[0]->robotPt,q_r_saved);
  p3d_destroy_config(agents->humans[0]->robotPt,q_h_saved);

  SWITCH_TO_GREEN = TRUE;
}

void CB_test_button2_obj(FL_OBJECT *obj, long arg)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;
  configPt q_h, q_r;
  HRI_AGENTS * agents;

  for(i=0; i<env->nr; i++){
    if( strcasestr(env->robot[i]->name,"TAPE") )
      break;
  }
  if(i==env->nr){
    printf("No human in the environment\n");
    return;
  }

  agents = hri_create_agents();

  q_h = p3d_get_robot_config(agents->humans[0]->robotPt);
  q_r = p3d_get_robot_config(agents->robots[0]->robotPt);

  hri_agent_single_task_manip_move(agents->humans[0], GIK_RAREACH, env->robot[i], &q_h);
  p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q_h);
  hri_agent_single_task_manip_move(agents->robots[0], GIK_LAREACH, env->robot[i], &q_r);
  p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r);
  hri_agent_single_task_manip_move(agents->robots[0], GIK_LOOK, env->robot[i], &q_r);
  p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q_r);
  printf("Agent: %s  Type: %d\n",agents->robots[0]->robotPt->name, agents->robots[0]->type);
  printf("Agent: %s  Type: %d\n",agents->humans[0]->robotPt->name, agents->humans[0]->type);
  g3d_draw_allwin_active();

}

void CB_test_button3_obj(FL_OBJECT *obj, long arg)
{
  int i;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  //int jointindexes[2][10]={ {0,0,0,15,16,17,18,19,20,21},{2,3,4,0,0,0,0,0,0,0} };
  int jointindexes1[10] = {2,3,4,15,16,17,18,19,20,21};
  p3d_vector3 Tcoord;
  p3d_rob * rob;
  configPt q_s;

  for(i=0; i<env->nr; i++){
    if( strcasestr(env->robot[i]->name,"ACHILE") )
      break;
  }
  if(i==env->nr){
    printf("No human in the environment\n");
    return;
  }
  rob = env->robot[i];
  if(!HRI_GIK->GIKInitialized){
    hri_gik_initialize_gik(HRI_GIK,env->robot[i],10);
    hri_gik_add_task(HRI_GIK, 3, 10, 1, jointindexes1,37);  /* Larm */
    // hri_gik_add_task(HRI_GIK, 3, 10, 2, jointindexes[1],37);  /* torso */
  }

  for(i=0; i<env->nr; i++){
    if( strcasestr(env->robot[i]->name,"VISBALL") )
      break;
  }
  if(i==env->nr){
    printf("No human in the environment\n");
    return;
  }

  Tcoord[0] = env->robot[i]->joints[1]->abs_pos[0][3];
  Tcoord[1] = env->robot[i]->joints[1]->abs_pos[1][3];
  Tcoord[2] = env->robot[i]->joints[1]->abs_pos[2][3];

  q_s = p3d_get_robot_config(rob);
  hri_gik_sdls(rob, HRI_GIK, 500, 0.01, &Tcoord, &q_s, NULL);

  p3d_set_and_update_this_robot_conf(rob, q_s);

  g3d_draw_allwin_active();
}

void CB_test_button4_obj(FL_OBJECT *obj, long arg)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i,j;
  p3d_rob * rob1, * rob2;
  double rob1_cx, rob1_cy, rob1_cz, rob2_cx, rob2_cy, rob2_cz;
  configPt q;

  q = p3d_get_robot_config(GLOBAL_AGENTS->humans[0]->robotPt);
  hri_agent_load_default_arm_posture(GLOBAL_AGENTS->humans[0], q);
  hri_agent_compute_state_posture(GLOBAL_AGENTS->humans[0], 0, q);
  p3d_copy_config_into(GLOBAL_AGENTS->humans[0]->robotPt, q, &GLOBAL_AGENTS->humans[0]->robotPt->ROBOT_POS);

  q = p3d_get_robot_config(GLOBAL_AGENTS->humans[1]->robotPt);
  hri_agent_load_default_arm_posture(GLOBAL_AGENTS->humans[1], q);
  hri_agent_compute_state_posture(GLOBAL_AGENTS->humans[1], 0, q);
  p3d_copy_config_into(GLOBAL_AGENTS->humans[1]->robotPt, q, &GLOBAL_AGENTS->humans[1]->robotPt->ROBOT_POS);

  //GLOBAL_AGENTS->robots[0]->perspective->enable_vision_draw = TRUE;
  // GLOBAL_AGENTS->humans[0]->perspective->enable_pointing_draw = TRUE;
  GLOBAL_AGENTS->humans[1]->perspective->enable_vision_draw = TRUE;
  GLOBAL_AGENTS->humans[1]->perspective->enable_pointing_draw = TRUE;


  return ;
  // Visibility performance test
  HRI_AGENT *selected_agent;
  p3d_rob *selected_target;
  float clock_val = 0;
  float elapsed_time = 0;
  int test_no = 10;

  for (i=0; i<test_no; i++) {
    selected_agent = GLOBAL_AGENTS->all_agents[random()%GLOBAL_AGENTS->all_agents_no];
    selected_target = env->robot[random()%env->nr];

    clock_val = clock();
    hri_is_object_visible(selected_agent, selected_target, 50, FALSE, TRUE);

    elapsed_time = (clock() - clock_val)/CLOCKS_PER_SEC + elapsed_time;
  }
  printf("TIME passed for %d visibility tests: %f EACH VIS: %f\n",test_no, elapsed_time,elapsed_time/test_no);
  // End test




}

void CB_test_button5_obj(FL_OBJECT *obj, long arg)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;
  int fov = FALSE, foa = FALSE;
  p3d_rob * robot;
  p3d_rob * object;
  int foa_angle = 30;
  int fov_angle = 150;
  configPt q_source, q_object;
  double phi, theta;
  int tilt_joint, pan_joint;
  int visible = FALSE;
  configPt q;

  for(i=0; i<env->nr; i++){
    if( strcasestr(env->robot[i]->name,"ROBOT") ){
      robot = env->robot[i];
      continue;
    }
    if( strcasestr(env->robot[i]->name,"SPACENAV") ){
      object = env->robot[i];
      continue;
    }
  }
  double result;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
//  g3d_is_object_visible_from_current_viewpoint2(win, object, &result, FALSE, NULL);
//  return;

  // hri_object_visibility_placement(GLOBAL_AGENTS->robots[0], object, &visibil);
  //g3d_is_object_visible_from_viewpoint(GLOBAL_AGENTS->robots[0]->perspective->camjoint->abs_pos, 50, object, &phi);
  //g3d_object_visibility_placement(GLOBAL_AGENTS->robots[0]->perspective->camjoint->abs_pos, object, DTOR(90), DTOR(90*0.75), DTOR(50), DTOR(50*0.75), &visibil);
  GLOBAL_AGENTS->humans[0]->perspective->enable_vision_draw = TRUE;
 // GLOBAL_AGENTS->humans[0]->perspective->enable_pointing_draw = TRUE;
  GLOBAL_AGENTS->humans[1]->perspective->enable_vision_draw = TRUE;
 // GLOBAL_AGENTS->humans[1]->perspective->enable_pointing_draw = TRUE;

  GLOBAL_AGENTS->robots[0]->perspective->enable_vision_draw = TRUE;
  //GLOBAL_AGENTS->robots[0]->perspective->enable_pointing_draw = TRUE;
  GLOBAL_AGENTS->robots[1]->perspective->enable_vision_draw = TRUE;
  //GLOBAL_AGENTS->robots[1]->perspective->enable_pointing_draw = TRUE;

  // GLOBAL_AGENTS->humans[1]->perspective->enable_vision_draw = TRUE;
  //GLOBAL_AGENTS->humans[2]->perspective->enable_vision_draw = TRUE;
  //GLOBAL_AGENTS->humans[3]->perspective->enable_vision_draw = TRUE;
  //GLOBAL_AGENTS->humans[3]->perspective->enable_pointing_draw = TRUE;

  q = p3d_get_robot_config(GLOBAL_AGENTS->humans[0]->robotPt);
  hri_agent_load_default_arm_posture(GLOBAL_AGENTS->humans[0], q);
  hri_agent_compute_state_posture(GLOBAL_AGENTS->humans[0], 0, q);
  p3d_set_and_update_this_robot_conf(GLOBAL_AGENTS->humans[0]->robotPt, q);

  q = p3d_get_robot_config(GLOBAL_AGENTS->humans[1]->robotPt);
  hri_agent_load_default_arm_posture(GLOBAL_AGENTS->humans[1], q);
  hri_agent_compute_state_posture(GLOBAL_AGENTS->humans[1], 0, q);
  p3d_set_and_update_this_robot_conf(GLOBAL_AGENTS->humans[1]->robotPt, q);

  return ;
  //printf("VISIBILITY RESULT ROBOT: %d PLACEMENT: %d\n ",result,visibil);
  result = hri_is_object_pointed(GLOBAL_AGENTS->humans[0],object, 50, TRUE);
  //g3d_is_object_visible_from_viewpoint(GLOBAL_AGENTS->humans[0]->perspective->camjoint->abs_pos, 150, object, &phi);

  printf("VISIBILITY RESULT HUMAN: %d\n",result);

  q_source = p3d_get_robot_config(GLOBAL_AGENTS->humans[0]->robotPt);
  hri_agent_load_default_arm_posture(GLOBAL_AGENTS->humans[0], q_source);
  hri_agent_compute_posture(GLOBAL_AGENTS->humans[0],1.5 , 2, q_source);
  p3d_set_and_update_this_robot_conf(GLOBAL_AGENTS->humans[0]->robotPt, q_source);

  return;

  foa = psp_is_object_in_fov(robot, object, DTOR(foa_angle) , DTOR(foa_angle)*3/4);

  if(foa){
    printf("IN FOA");
  }
  else {
    fov = psp_is_object_in_fov(robot, object, DTOR(fov_angle) , DTOR(fov_angle)*3/4);
    if (fov) {
      printf("IN FOV");
    }
    else {
      printf("IN OOF");
    }

  }

  pan_joint = HUMANj_NECK_PAN;
  tilt_joint = HUMANj_NECK_TILT;

  q_source = p3d_get_robot_config(robot);
  q_object = p3d_get_robot_config(object);

  p3d_psp_cartesian2spherical(q_object[6],q_object[7],q_object[8],
                              robot->joints[HUMANj_NECK_TILT]->abs_pos[0][3],
                              robot->joints[HUMANj_NECK_TILT]->abs_pos[1][3],
                              robot->joints[HUMANj_NECK_TILT]->abs_pos[2][3],
                              &phi,&theta);

  theta = theta - M_PI_2;
  printf("\nPhi: %f, theta: %f\n",RTOD(phi),RTOD(theta));
  printf("Robot Orient: %f\n",RTOD(q_source[11]));

  //temp_orient = q_source[11] - M_PI_2;

  //if(temp_orient < -M_PI) temp_orient = temp_orient + M_2PI;
  //if(temp_orient >  M_PI) temp_orient = temp_orient - M_2PI;

  //phi = phi - temp_orient;

  phi = phi - q_source[11];

  if(phi < -M_PI) phi = phi + M_2PI;
  if(phi >  M_PI) phi = phi - M_2PI;
//

//
//
//  phi = M_2PI - (temp_orient- phi);
//
//  if(phi < -M_PI) phi = phi + M_2PI;
//  if(phi >  M_PI) phi = phi - M_2PI;

  /* TURN PAN  */
  if(robot->joints[pan_joint]->dof_data[0].vmin > phi){
    q_source[robot->joints[pan_joint]->index_dof] = robot->joints[pan_joint]->dof_data[0].vmin;
  }
  else{
    if(robot->joints[pan_joint]->dof_data[0].vmax < phi){
      q_source[robot->joints[pan_joint]->index_dof] = robot->joints[pan_joint]->dof_data[0].vmax;
    }
    else{
      q_source[robot->joints[pan_joint]->index_dof] = phi;
    }
  }

  /* TURN TILT */
  if(robot->joints[tilt_joint]->dof_data[0].vmin > theta){
    q_source[robot->joints[tilt_joint]->index_dof] = robot->joints[tilt_joint]->dof_data[0].vmin;
  }
  else{
    if(robot->joints[tilt_joint]->dof_data[0].vmax < theta){
      q_source[robot->joints[tilt_joint]->index_dof] = robot->joints[tilt_joint]->dof_data[0].vmax;
    }
    else{
      q_source[robot->joints[tilt_joint]->index_dof] = theta;
    }
  }


  p3d_set_and_update_this_robot_conf(robot, q_source);

  //  visible = hri_is_object_visible(robot, object, 10,TRUE);

  if(visible)
    printf("OBJECT VISIBLE\n");
  else
    printf("OBJECT INVISIBLE\n");

 return ;

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
