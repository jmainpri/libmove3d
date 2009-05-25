#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#define LOCAL_COMPUTATION_EPSILON (1e-9)

FL_FORM  *HRI_PLANNER_FORM = NULL;
extern FL_FORM  *PSP_PARAMETERS_FORM;

hri_bitmapset * ACBTSET = NULL;

/* ---------- FUNCTION DECLARATIONS --------- */

static void g3d_create_find_model_q(void); //Luis

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

/***********************************/
static void g3d_create_bitmap_init_obj(void);     
static void g3d_create_distance_active_obj(void);
static void g3d_create_visibility_active_obj(void);
static void g3d_create_hidzones_active_obj(void);
static void g3d_create_btcombined_active_obj(void);
static void g3d_create_btpath_active_obj(void);
static void g3d_create_btobstacles_active_obj(void);
static void g3d_create_calculate_path_obj(void);
static void g3d_create_update_choice_obj(void);
static void g3d_create_save_choice_obj(void);
static void g3d_create_combine_choice_obj(void);
static void g3d_create_param1_obj(void);
static void g3d_create_param2_obj(void);
static void g3d_create_param3_obj(void);
static void g3d_create_param4_obj(void);
static void g3d_create_show_bitmaps_obj(void);
static void g3d_create_save_3dbitmaps_obj(void);
static void g3d_create_run_GIK_obj(void);
static void g3d_create_run_GIKstep_obj(void);
static void g3d_create_run_GIKstepper_obj(void);
static void g3d_create_interpoint_weight1_obj(void);
static void g3d_create_interpoint_distance_active_obj(void);
static void g3d_create_interpoint_weight2_obj(void);
static void g3d_create_interpoint_visibility_active_obj(void);
static void g3d_create_interpoint_weight3_obj(void);
static void g3d_create_interpoint_humanreach_active_obj(void);
static void g3d_create_drawno_obj(void);
/* static void g3d_create_init_interpoint_obj(void); */

static void g3d_create_rrttest_obj(void);
static void g3d_create_stop_obj(void);

static void g3d_create_GIKstep_obj(void);
static void g3d_create_GIKforce_obj(void);
static void g3d_create_GIKvision_obj(void);
static void g3d_create_gik_direct_obj(void);
static void g3d_create_placement_obj(void);
static void g3d_create_plcmt_type_obj(void);
static void g3d_create_select_human_obj(void);
static void g3d_create_select_state_obj(void);
static void g3d_create_select_exists_obj(void);     
static void g3d_create_select_bitmapset_obj(void);  
static void g3d_delete_find_model_q(void); //Luis

static void g3d_delete_bitmap_init_obj(void);
static void g3d_delete_distance_active_obj(void);
static void g3d_delete_visibility_active_obj(void);
static void g3d_delete_hidzones_active_obj(void);
static void g3d_delete_btcombined_active_obj(void);
static void g3d_delete_btpath_active_obj(void);
static void g3d_delete_btobstacles_active_obj(void);
static void g3d_delete_calculate_path_obj(void);
static void g3d_delete_update_choice_obj(void);
static void g3d_delete_combine_choice_obj(void);
static void g3d_delete_param1_obj(void);
static void g3d_delete_param2_obj(void);
static void g3d_delete_param3_obj(void);
static void g3d_delete_param4_obj(void);
static void g3d_delete_show_bitmaps_obj(void);
static void g3d_delete_save_3dbitmaps_obj(void);
static void g3d_delete_placement_obj(void);
static void g3d_delete_plcmt_type_obj(void);
static void g3d_delete_select_human_obj(void);
static void g3d_delete_select_state_obj(void);
static void g3d_delete_select_exists_obj(void);
static void g3d_delete_run_GIKstep_obj(void);
static void g3d_delete_run_GIKstepper_obj(void);
static void g3d_delete_interpoint_weight1_obj(void);
static void g3d_delete_interpoint_distance_active_obj(void);
static void g3d_delete_interpoint_weight2_obj(void);
static void g3d_delete_interpoint_visibility_active_obj(void);
static void g3d_delete_interpoint_weight3_obj(void);
static void g3d_delete_interpoint_humanreach_active_obj(void);
static void g3d_delete_select_bitmapset_obj(void);
static void g3d_delete_drawn_obj(void);
static void g3d_delete_run_GIK_obj(void);
static void g3d_delete_gik_direct_obj(void);
static void g3d_delete_GIKstep_obj(void);
static void g3d_delete_GIKforce_obj(void);
static void g3d_delete_GIKvision_obj(void);

static void g3d_delete_stop_obj(void);

static void g3d_delete_find_model_q(void);
void g3d_delete_psp_parameters_form(void);


/* ------- VARIABLES ------- */

int GIK_VIS;
int PLACEMENT;
int PLCMT_TYPE;

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

/* ---------------------------------- */



FL_OBJECT  *BT_CHOICE1_OBJ;
FL_OBJECT  *BT_CHOICE2_OBJ;
FL_OBJECT  *BT_CHOICE3_OBJ;
static FL_OBJECT  *STRAT1_OBJ; 
static FL_OBJECT  *STRAT2_OBJ;
//FL_OBJECT  *STRAT3_OBJ;
//FL_OBJECT  *STRAT4_OBJ;
//FL_OBJECT  *STRAT5_OBJ;
static FL_OBJECT  *SEARCH_DRAW_OPTIM_OBJ;
static FL_OBJECT  *UPDATEGROUP; 
static FL_OBJECT  *COMBINEGROUP;
static FL_OBJECT  *PLACEMENTGROUP;
static FL_OBJECT  *PLCMTTYPEGROUP;

static FL_OBJECT  *BT_INIT_OBJ;
static FL_OBJECT  *BT_DISTANCE_OBJ = NULL;
static FL_OBJECT  *BT_VISIBILITY_OBJ = NULL;
static FL_OBJECT  *BT_HIDZONES_OBJ = NULL;
static FL_OBJECT  *BT_COMBI_OBJ = NULL;
static FL_OBJECT  *BT_PATH_OBJ = NULL;
static FL_OBJECT  *BT_OBSTACLES_OBJ = NULL;

static FL_OBJECT  *BT_PARAM1_OBJ;
static FL_OBJECT  *BT_PARAM2_OBJ;
static FL_OBJECT  *BT_PARAM3_OBJ;
static FL_OBJECT  *BT_PARAM4_OBJ;

static FL_OBJECT  *BT_COMBINE1_OBJ;
static FL_OBJECT  *BT_COMBINE2_OBJ;

static FL_OBJECT  *BT_CALPATH_OBJ; 

static FL_OBJECT  *BT_SHOW_BITMAPS_OBJ;
static FL_OBJECT  *BT_SAVE_3DBITMAPS_OBJ;

FL_OBJECT  *BT_SAVE_CHOICE1_OBJ;
FL_OBJECT  *BT_SAVE_CHOICE2_OBJ;
FL_OBJECT  *BT_SAVE_CHOICE3_OBJ;
FL_OBJECT  *BT_SAVE_CHOICE4_OBJ;
static FL_OBJECT  *SAVEGROUP; 

static FL_OBJECT  *BT_RRTTEST_OBJ;
static FL_OBJECT  *STOP_OBJ;

static FL_OBJECT  *BT_RUN_GIK_OBJ;
static FL_OBJECT  *BT_RUN_GIKstep_OBJ;
static FL_OBJECT  *BT_RUN_GIKstepper_OBJ;
static FL_OBJECT  *BT_GIKSTEP_OBJ;
static FL_OBJECT  *BT_GIKFORCE_OBJ;
static FL_OBJECT  *BT_GIKVISION_OBJ;
static FL_OBJECT  *BT_IPW1_OBJ;
static FL_OBJECT  *BT_INTDISTANCE_OBJ;
static FL_OBJECT  *BT_IPW2_OBJ;
static FL_OBJECT  *BT_INTVISIBILITY_OBJ;
static FL_OBJECT  *BT_IPW3_OBJ;
static FL_OBJECT  *BT_INTHUMANR_OBJ;
static FL_OBJECT  *BT_PLCMT_OBJ1;
static FL_OBJECT  *BT_PLCMT_OBJ2;
static FL_OBJECT  *BT_PLCMT_OBJ3;
static FL_OBJECT  *BT_PLCMT_OBJ4;
static FL_OBJECT  *BT_PLCMT_OBJ5;
static FL_OBJECT  *BT_PLCMT_OBJ6;
static FL_OBJECT  *BT_PLCMT_OBJ7;
static FL_OBJECT  *BT_PLCMT_OBJ8;
static FL_OBJECT  *BT_DRAWNO_OBJ;

static FL_OBJECT  *ACTUAL_HUMAN;
static FL_OBJECT  *HUMAN_STATE;
static FL_OBJECT  *HUMAN_EXISTS;

static FL_OBJECT  *SELECT_BTSET;

static FL_OBJECT  *BT_PLCMT_TYPE_OBJ1;
static FL_OBJECT  *BT_PLCMT_TYPE_OBJ2;
static FL_OBJECT  *BT_PLCMT_TYPE_OBJ3;

static FL_OBJECT  *SEARCH_DRAW_OBJ; 

static FL_OBJECT  *GIK_DIRECT_OBJ;


/* ---------------- FUNCTIONS ---------------- */
static int HUMAN_FORM_CREATED = FALSE;
static int SELECTED_BTSET = 1;
static gnuplot_ctrl* gnuplots[] = {NULL,NULL,NULL,NULL,NULL};
static int GNUPLOT_ACTIVE = FALSE;

extern int PSP_MA_SEGMENTS;
extern int PSP_MA_LAYERS;
extern int PSP_SRCH_MTD;
extern double PSP_PS_TRSHLD;


/* -------------- CREATION --------------- */
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
	
  /* g3d_create_bitmap_init_obj(); */
/*   g3d_create_distance_active_obj(); */
/*   g3d_create_visibility_active_obj(); */
/*   g3d_create_hidzones_active_obj(); */
/*   g3d_create_calculate_path_obj(); */
/*   g3d_create_btcombined_active_obj(); */
/*   g3d_create_btpath_active_obj(); */
/*   g3d_create_btobstacles_active_obj(); */
/*   g3d_create_update_choice_obj(); */
/*   g3d_create_save_choice_obj(); */
/*   g3d_create_combine_choice_obj(); */
/*   g3d_create_param1_obj(); */
/*   g3d_create_param2_obj(); */
/*   g3d_create_param3_obj(); */
/*   g3d_create_param4_obj(); */
/*   g3d_create_show_bitmaps_obj(); */
/*   g3d_create_save_3dbitmaps_obj(); */
/*   g3d_create_run_GIK_obj(); */
/*   g3d_create_run_GIKstep_obj(); */
/*   g3d_create_run_GIKstepper_obj(); */
/*   g3d_create_GIKstep_obj(); */
/*   g3d_create_GIKforce_obj(); */
/*   g3d_create_GIKvision_obj(); */
/*   g3d_create_interpoint_weight1_obj(); */
/*   g3d_create_interpoint_distance_active_obj(); */
/*   g3d_create_interpoint_weight2_obj(); */
/*   g3d_create_interpoint_visibility_active_obj(); */
/*   g3d_create_interpoint_weight3_obj(); */
/*   g3d_create_interpoint_humanreach_active_obj(); */
/*   g3d_create_drawno_obj(); */
	
/*   g3d_create_rrttest_obj(); */
/*   g3d_create_stop_obj(); */
	
/*   g3d_create_gik_direct_obj(); */
/*   g3d_create_placement_obj(); */
/*   g3d_create_plcmt_type_obj(); */
/*   g3d_create_select_human_obj(); */
/*   g3d_create_select_state_obj(); */
/*   g3d_create_select_exists_obj(); */
/*   g3d_create_select_bitmapset_obj(); */

  fl_end_form();

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
 /*  g3d_delete_bitmap_init_obj(); */
/*   g3d_delete_distance_active_obj(); */
/*   g3d_delete_visibility_active_obj(); */
/*   g3d_delete_hidzones_active_obj(); */
/*   g3d_delete_btcombined_active_obj(); */
/*   g3d_delete_btpath_active_obj(); */
/*   g3d_delete_btobstacles_active_obj(); */
/*   g3d_delete_calculate_path_obj(); */
/*   g3d_delete_update_choice_obj(); */
/*   g3d_delete_combine_choice_obj(); */
/*   g3d_delete_param1_obj(); */
/*   g3d_delete_param2_obj(); */
/*   g3d_delete_param3_obj(); */
/*   g3d_delete_param4_obj(); */
/*   g3d_delete_show_bitmaps_obj(); */
/*   g3d_delete_save_3dbitmaps_obj(); */
/*   g3d_delete_placement_obj(); */
/*   g3d_delete_plcmt_type_obj(); */
/*   g3d_delete_select_human_obj(); */
/*   g3d_delete_select_state_obj(); */
/*   g3d_delete_select_exists_obj(); */
/*   g3d_delete_run_GIKstep_obj(); */
/*   g3d_delete_run_GIKstepper_obj(); */
/*   g3d_delete_interpoint_weight1_obj(); */
/*   g3d_delete_interpoint_distance_active_obj(); */
/*   g3d_delete_interpoint_weight2_obj(); */
/*   g3d_delete_interpoint_visibility_active_obj(); */
/*   g3d_delete_interpoint_weight3_obj(); */
/*   g3d_delete_interpoint_humanreach_active_obj(); */
/*   g3d_delete_select_bitmapset_obj(); */
/*   g3d_delete_drawn_obj(); */
/*   g3d_delete_run_GIK_obj(); */
/*   g3d_delete_gik_direct_obj(); */
/*   g3d_delete_GIKstep_obj(); */
/*   g3d_delete_GIKforce_obj(); */
/*   g3d_delete_GIKvision_obj(); */
	
/*   g3d_delete_stop_obj(); */
	
  g3d_delete_find_model_q();
  g3d_delete_psp_parameters_form();
	
  fl_free_form(HRI_PLANNER_FORM);
}

static int my_drawtraj_fct(void)
{
  fl_check_forms();
  return TRUE;
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
    for(i=0; i<ACBTSET->human[ACBTSET->actual_human]->states_no; i++) 
      fl_addto_choice(BT_HUMAN_STATE_OBJ, ACBTSET->human[ACBTSET->actual_human]->state[i].name); 
    fl_addto_choice(BT_HUMAN_EXISTS_OBJ,"not exist");
    fl_addto_choice(BT_HUMAN_EXISTS_OBJ,"exist");
    
    HUMAN_FORM_CREATED = TRUE;
  }
 
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
		
    if(ACBTSET!=NULL){
      hri_bt_reset_path(ACBTSET); 
    }
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
      g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
      
      G3D_DRAW_TRAJ = 1;
      
      while( (qs=g3d_bt_dynamic_tshow(ACBTSET->robot,my_drawtraj_fct,&nb)) ){        
	qresult = hri_bt_set_TARGET();  
	if(qresult != NULL)  
	  qg = qresult;  
	p3d_del_graph(BTGRAPH);  
	p3d_del_graph(ACBTSET->robot->GRAPH);  
	ACBTSET->robot->GRAPH = NULL;  
	BTGRAPH = NULL;  
	hri_bt_reset_path(BTSET);        
	p3d_del_traj(BTSET->robot->tcur);  
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
		
    hri_exp_find_manip_path(ACBTSET);
		
    robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
    G3D_DRAW_GRAPH = 1;
    p3d_graph_to_traj(ACBTSET->robot);
    g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
		
    G3D_DRAW_TRAJ = 1;
    hri_exp_find_exchange_point();
		
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
    g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
		
    G3D_DRAW_TRAJ = 1;    
    
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
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vsides); 
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
  int val = fl_get_choice(obj);
  configPt q;
  p3d_rob* robotPt;
  
  ACBTSET->human[ACBTSET->actual_human]->actual_state = val-1;
  
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
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vsides); 
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
 
  q = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);  
  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c7;
  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c1;
  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c2;
  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c3;
  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c4;
  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c5;
  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c6;
  
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
  
  SHOWBTGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,10,190,310,50,"Show Bitmaps"); 
  
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
    gnuplots[i] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);

}

static void CB_showbt_gnuplot_obj(FL_OBJECT *obj, long arg)
{
  int i;
  
  if(GNUPLOT_ACTIVE){
    GNUPLOT_ACTIVE = FALSE;
    for(i=0; i<5; i++){
      gnuplot_close(gnuplots[i]);
      gnuplots[i] = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
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
  BT_NAV_VIS_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,280,50,20,"Visibility");
  fl_set_call_back(BT_NAV_VIS_OBJ,CB_nav_btchoice_obj,2);
  BT_NAV_HZ_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,300,50,20,"Hiddens");
  fl_set_call_back(BT_NAV_HZ_OBJ,CB_nav_btchoice_obj,3);
  
  BT_NAV_PARAM1_OBJ = fl_add_valslider(FL_HOR_SLIDER,80,260,245,15,"");
  fl_set_slider_step(BT_NAV_PARAM1_OBJ,10);
  fl_set_slider_bounds(BT_NAV_PARAM1_OBJ,0,1000);
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
    fl_set_slider_value(BT_NAV_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vsides); 
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
}

static void CB_nav_param_obj(FL_OBJECT *obj, long arg)
{
  
  if(fl_get_button(BT_NAV_DIST_OBJ)){
    hri_bt_update_distance(BTSET,
			   fl_get_slider_value(BT_NAV_PARAM1_OBJ),
			   fl_get_slider_value(BT_NAV_PARAM2_OBJ));
  }
  
  
  if(fl_get_button(BT_NAV_VIS_OBJ)){
    hri_bt_update_visibility(BTSET,
			     fl_get_slider_value(BT_NAV_PARAM1_OBJ),
			     fl_get_slider_value(BT_NAV_PARAM2_OBJ),
			     fl_get_slider_value(BT_NAV_PARAM3_OBJ));
  }
  
  
  if(fl_get_button(BT_NAV_HZ_OBJ)){
    hri_bt_update_hidzones(BTSET,
			   fl_get_slider_value(BT_NAV_PARAM4_OBJ));
  }  
  
  g3d_draw_allwin_active();
}


/* ------------------------------------------------------- */
static void g3d_create_manip_group(void)
{
  FL_OBJECT *obj;
  
  MANIPGROUPFR = fl_add_labelframe(FL_BORDER_FRAME,10,340,280,80,"Object Handling Parameters"); 
  
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
  
  obj = fl_add_labelframe(FL_ENGRAVED_FRAME,10,430,380,60,"GIK Parameters"); 
  
  GIKGROUP = fl_bgn_group();

  fl_end_group();
  
} 




/************************************************************************************************************/  
/****************************************** BITMAP INTERFACE ************************************************/
/************************************************************************************************************/
int GIK_STEP;
double GIK_FORCE;
//static int HUMAN_FORM_CREATED = FALSE;

static void CB_bitmap_init_obj(FL_OBJECT *ob, long arg)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int dimx,dimy,dimz;
  int val = fl_get_choice(SELECT_BTSET);
  int i;
  double objx, objy, objz;
	
  if(val == 1){    
    if(BTSET != NULL)
      hri_bt_destroy_bitmapset(BTSET);
		
    dimx = (int)((env->box.x2 - env->box.x1)/BT_SAMPLING);
    dimy = (int)((env->box.y2 - env->box.y1)/BT_SAMPLING);
    dimz = 1;
    BTSET = hri_bt_create_bitmaps();
    hri_bt_init_bitmaps(BTSET,dimx,dimy,dimz,BT_SAMPLING);
    hri_bt_change_bitmap_position(BTSET,env->box.x1,env->box.y1,
				  BTSET->robot->joints[ROBOTj_BASE]->dof_data[2].v);
		
    if(BT_DISTANCE_OBJ && BT_VISIBILITY_OBJ && BT_HIDZONES_OBJ && BT_COMBI_OBJ){
      fl_set_button(BT_DISTANCE_OBJ,0);
      fl_set_button(BT_VISIBILITY_OBJ,0);
      fl_set_button(BT_HIDZONES_OBJ,0);
      fl_set_button(BT_COMBI_OBJ,0);
    }
    ACBTSET = BTSET;
  }
  if(val == 2){
    if(INTERPOINT != NULL)
      hri_bt_destroy_bitmapset(INTERPOINT);
		
    INTERPOINT = hri_exp_init();
    ACBTSET = INTERPOINT;
  }
  if(val == 3){
    if(OBJSET != NULL)
      hri_bt_destroy_bitmapset(OBJSET);
     
    for(i=0; i<env->nr; i++){
      if( !strcmp("bottle",env->robot[i]->name) )
	break;
    }
    objx =  env->robot[i]->joints[1]->abs_pos[0][3];
    objy =  env->robot[i]->joints[1]->abs_pos[1][3];
    objz =  env->robot[i]->joints[1]->abs_pos[2][3];

    OBJSET = hri_object_reach_init(objx,objy,objz);
    ACBTSET = OBJSET;
  }

   if(!HUMAN_FORM_CREATED){
    for(i=0; i<ACBTSET->human_no; i++)
      fl_addto_choice(ACTUAL_HUMAN, ACBTSET->human[i]->HumanPt->name);
    for(i=0; i<ACBTSET->human[ACBTSET->actual_human]->states_no; i++) 
      fl_addto_choice(HUMAN_STATE, ACBTSET->human[ACBTSET->actual_human]->state[i].name); 
    HUMAN_FORM_CREATED = TRUE;
  }
	
  fl_set_button(BT_INIT_OBJ,0); 
	
  g3d_draw_allwin_active();
	
} 

static void g3d_create_bitmap_init_obj(void)
{  
  BT_INIT_OBJ = fl_add_button(FL_PUSH_BUTTON,10,105,80,30,"Init Bitmaps");
  fl_set_call_back(BT_INIT_OBJ,CB_bitmap_init_obj,0);     
}


///////////////////////////Luis

/* static void CB_find_model_q(FL_OBJECT *ob, long arg) */
/* { */
/*   //funcion par empezar el random   */
/*   //p3d_search_best_point(); */
/*   int i,j; */
/*   p3d_rob *r; */
/*   p3d_rob *human = (BTSET->human[BTSET->actual_human]->HumanPt); */
/*   //int *iksols=NULL, *iksolg=NULL;  */
/*   //persp_win->win_perspective=1; */
  
/*   if(BTSET == NULL) return; */

/*   r = (BTSET->robot); */

/*   fl_set_button(ob,0); */
/*   g3d_draw_allwin_active(); */
/*   printf("%s\n",(human->name)); */
/*   for(i=0 ; i<=3 ; i++){ */
/*     for(j=0 ; j<=3 ; j++){ */
/*       printf("%f ",human->joints[1]->abs_pos[i][j]); */
/*     } */
/*     printf("\n"); */
/*   } */
/*   p3d_select_robot_to_view(human); */
/*   //psp_select_object_to_view_by_name("table1"); */
/*   g3d_draw_allwin_active(); */
  
/*   // no es el BTSET */
/*   //p3d_psp_srch_model_pt(r,human,50,iksols,iksolg,fct_stop,fct_draw);  */
/*   if (persp_win) */
/*     { */
/*       //psp_srch_for_target_obj(r,50); */
/*       //p3d_deselect_all_objects(); */
/*       psp_srch_model_pt(r,human,50,BTSET);  */
/*       p3d_deselect_robot_to_view(human); */
/*       g3d_draw_allwin_active(); */
/*     } */
  
/* } */

static void CB_find_model_q(FL_OBJECT *ob, long arg)
{
  //funcion par empezar el random  
  //p3d_search_best_point();
  int i,j;
  p3d_rob *r = (BTSET->robot);//(p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_rob *human = (BTSET->human[BTSET->actual_human]->HumanPt);
  p3d_rob *bottle = BTSET->object;
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
  if (persp_win)
    {
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





static void g3d_create_find_model_q(void)
{  
  //p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  //int dimx,dimy;
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

///////////////////////////


static void CB_distance_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_DISTANCE,BTSET)) hri_bt_activate(BT_DISTANCE,BTSET);
  else                                     hri_bt_desactivate(BT_DISTANCE,BTSET);
	
  g3d_draw_allwin_active();
} 

static void g3d_create_distance_active_obj(void)
{
  FL_OBJECT *obj;
	
  obj = fl_add_frame(FL_ENGRAVED_FRAME,5,70,360,30,""); 
  obj = fl_add_box(FL_FLAT_BOX,145,65,80,10," Active Bitmaps");
	
  BT_DISTANCE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,7,68,50,20,"Distance");  
  fl_set_call_back(BT_DISTANCE_OBJ,CB_distance_active_obj,0);
	
} 

static void CB_visibility_active_obj(FL_OBJECT *ob, long arg)
{  
  if(!hri_bt_is_active(BT_VISIBILITY,BTSET))  hri_bt_activate(BT_VISIBILITY,BTSET);    
  else                                            hri_bt_desactivate(BT_VISIBILITY,BTSET);  
	
  g3d_draw_allwin_active();
}

static void g3d_create_visibility_active_obj(void)
{ 
  BT_VISIBILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,7,81,50,20,"Visibility");
	
  fl_set_call_back(BT_VISIBILITY_OBJ,CB_visibility_active_obj,0);
	
}

static void CB_hidzones_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_HIDZONES,BTSET))    hri_bt_activate(BT_HIDZONES,BTSET);
  else                                        hri_bt_desactivate(BT_HIDZONES,BTSET);  
	
  g3d_draw_allwin_active(); 
	
} 

static void g3d_create_hidzones_active_obj(void)
{ 
  BT_HIDZONES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,80,68,50,20,"Hidden Zones");
	
  fl_set_call_back(BT_HIDZONES_OBJ,CB_hidzones_active_obj,0);
	
} 

static void CB_btcombined_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_COMBINED,BTSET))  hri_bt_activate(BT_COMBINED,BTSET);    
  else                                      hri_bt_desactivate(BT_COMBINED,BTSET);  
	
  g3d_draw_allwin_active();      
}

static void g3d_create_btcombined_active_obj(void)
{ 
  BT_COMBI_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,80,81,50,20,"Combined");
	
  fl_set_call_back(BT_COMBI_OBJ,CB_btcombined_active_obj,0);  
}

static void CB_btpath_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_PATH,BTSET))  hri_bt_activate(BT_PATH,BTSET);    
  else                                  hri_bt_desactivate(BT_PATH,BTSET);  
	
  g3d_draw_allwin_active();      
}

static void g3d_create_btpath_active_obj(void)
{ 
  BT_PATH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,165,75,50,20,"A* Path");
	
  fl_set_call_back(BT_PATH_OBJ,CB_btpath_active_obj,0);  
}

static void CB_btobstacles_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_OBSTACLES,BTSET))  hri_bt_activate(BT_OBSTACLES,BTSET);    
  else                                       hri_bt_desactivate(BT_OBSTACLES,BTSET);  
	
  g3d_draw_rob_pos_area();
	
  g3d_draw_allwin_active();      
}

static void g3d_create_btobstacles_active_obj(void)
{ 
  BT_OBSTACLES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,249,75,50,20,"Obstacles");
	
  fl_set_call_back(BT_OBSTACLES_OBJ,CB_btobstacles_active_obj,0);  
}


static int traj_play = TRUE;

/* static int my_drawtraj_fct(void) */
/* { */
/*   fl_check_forms(); */
/*   return(traj_play); */
/* } */

int place;
int place_type;

static void CB_calculate_path_obj(FL_OBJECT *ob, long arg)
{
  configPt qs, qg, qresult;
  int res;
  int nb;
  p3d_rob* robotPt;
  int val = fl_get_choice(SELECT_BTSET);
	
  if(val == 1){
		
    if(ACBTSET!=NULL){
      hri_bt_reset_path(ACBTSET); 
    }
    if(BTGRAPH!=NULL){
      p3d_del_graph(BTGRAPH);
      BTGRAPH = NULL;
    }
		
    qs = p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS); 
    if(place==0 || place_type==0)
      qg = p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_GOTO);    
    else
      qg = hri_bt_set_TARGET();
		
    MY_ALLOC_INFO("Avant la creation du graphe");
		
    res = hri_bt_calculate_bitmap_path(ACBTSET,ACBTSET->robot,qs,qg,FALSE);  
    p3d_destroy_config(ACBTSET->robot, qs);
    p3d_destroy_config(ACBTSET->robot, qg);		
    if(!res){
      printf("p3d_hri_planner : FAIL : cannot find a path\n");
    }
    else{ 
      robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
      p3d_sel_desc_name(P3D_ROBOT,ACBTSET->robot->name);
			
      p3d_graph_to_traj(BTSET->robot);
      g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
      p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
			
      G3D_DRAW_TRAJ = 1;
      fl_set_button(SEARCH_DRAW_OBJ,G3D_DRAW_GRAPH);
      fl_set_button(SEARCH_DRAW_OPTIM_OBJ,G3D_DRAW_TRAJ);
      /* while( (qs=g3d_bt_dynamic_tshow(ACBTSET->robot,my_drawtraj_fct,&nb)) ){          */
      /*         qresult = hri_bt_set_TARGET();    */
      /*         if(qresult != NULL)    */
      /*           qg = qresult;    */
      /*         p3d_del_graph(BTGRAPH);    */
      /*         p3d_del_graph(ACBTSET->robot->GRAPH);    */
      /*         ACBTSET->robot->GRAPH = NULL;    */
      /*         BTGRAPH = NULL;    */
      /*         hri_bt_reset_path(ACBTSET);          */
      /*         p3d_del_traj(BTSET->robot->tcur);    */
      /*         ACBTSET->robot->tcur = NULL;    */
      /*         hri_bt_calculate_bitmap_path(ACBTSET,ACBTSET->robot,qs,qg,FALSE);   */
      /*         robotPt = p3d_get_desc_curid(P3D_ROBOT);    */
      /*         p3d_sel_desc_name(P3D_ROBOT,ACBTSET->robot->name);    */
      /*         p3d_graph_to_traj(ACBTSET->robot);    */
      /*         p3d_sel_desc_name(P3D_ROBOT,robotPt->name);   */
      /*******/
      /* g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ)); */
      //printf("image\n");
      //g3d_save_movie_image();
      /*    }     */
    }
  }
  if(val == 2){
		
    if(ACBTSET!=NULL){
      hri_bt_reset_path(ACBTSET); 
    }
    if(BTGRAPH!=NULL){
      p3d_del_graph(BTGRAPH);
      BTGRAPH = NULL;
    }
		
    hri_exp_find_manip_path(ACBTSET);
		
    robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
    G3D_DRAW_GRAPH = 1;
    p3d_graph_to_traj(ACBTSET->robot);
    g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
		
    G3D_DRAW_TRAJ = 1;
    hri_exp_find_exchange_point();
		
  }
  if(val == 3){
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
    G3D_DRAW_GRAPH = 1;
    p3d_graph_to_traj(ACBTSET->robot);
    g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
    p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
		
    G3D_DRAW_TRAJ = 1;
    
    
  }
  fl_set_button(BT_CALPATH_OBJ,0);
  g3d_draw_allwin_active();
}

static void g3d_create_calculate_path_obj(void)
{ 
  FL_OBJECT *obj;
	
  obj = fl_add_frame(FL_ENGRAVED_FRAME,95,100,150,40,""); 
	
  BT_CALPATH_OBJ = fl_add_button(FL_PUSH_BUTTON,100,105,70,30,"Find Path");
  fl_deactivate_object(BT_CALPATH_OBJ);
  fl_set_call_back(BT_CALPATH_OBJ,CB_calculate_path_obj,0);  
}

static void CB_combine_choice_obj(FL_OBJECT *ob, long arg)
{
  if(ACBTSET == NULL)
    return;
  if(arg==0)
    ACBTSET->combine_type = BT_COMBINE_MAX;
  if(arg==1)
    ACBTSET->combine_type = BT_COMBINE_SUM;
	
  fl_activate_object(BT_CALPATH_OBJ);
}

static void g3d_create_combine_choice_obj(void)
{
  COMBINEGROUP = fl_bgn_group();
  BT_COMBINE1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,180,100,20,20,"MAX");
  fl_set_object_color(BT_COMBINE1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_COMBINE1_OBJ,CB_combine_choice_obj,0);
  BT_COMBINE2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,180,120,20,20,"SUM");
  fl_set_object_color(BT_COMBINE2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_COMBINE2_OBJ,CB_combine_choice_obj,1);
  COMBINEGROUP = fl_end_group();
} 

static void CB_update_choice_obj(FL_OBJECT *ob, long arg)
{
  if(ACBTSET==NULL){
    return;
  }
  
  if(arg==0){   
    fl_set_slider_value(BT_PARAM1_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].dheight);
    fl_set_slider_value(BT_PARAM2_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].dradius);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,0);
    
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
  }
  if(arg==1){  
    fl_set_slider_value(BT_PARAM1_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].vheight);
    fl_set_slider_value(BT_PARAM2_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].vback);
    fl_set_slider_value(BT_PARAM3_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].vsides); 
    fl_set_slider_value(BT_PARAM4_OBJ,0);
		
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_activate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
		
  }  
  if(arg==2){ 
    fl_set_slider_value(BT_PARAM1_OBJ,0);
    fl_set_slider_value(BT_PARAM2_OBJ,0);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].hradius);
		
    fl_deactivate_object(BT_PARAM1_OBJ);
    fl_deactivate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_activate_object(BT_PARAM4_OBJ);
  }
}


static void g3d_create_update_choice_obj(void)
{
  FL_OBJECT *obj;
	
  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,150,70,80,""); 
  obj = fl_add_box(FL_FLAT_BOX,25,145,40,10,"Bitmaps"); 
	
  UPDATEGROUP = fl_bgn_group();
  BT_CHOICE1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,158,50,20,"Distance");
  fl_set_object_color(BT_CHOICE1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_CHOICE1_OBJ,CB_update_choice_obj,0);
  BT_CHOICE2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,178,50,20,"Visibility");
  fl_set_object_color(BT_CHOICE2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_CHOICE2_OBJ,CB_update_choice_obj,1);
  BT_CHOICE3_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,10,198,50,20,"Hiddens");
  fl_set_object_color(BT_CHOICE3_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_CHOICE3_OBJ,CB_update_choice_obj,2);
  UPDATEGROUP = fl_end_group();
  fl_set_button(STRAT2_OBJ,1);
	
} 

/****  THE HEIGHT OF BITMAPS ****/
static void CB_update_param1_obj(FL_OBJECT *ob, long arg)
{

  if(fl_get_button(BT_CHOICE1_OBJ)){
    hri_bt_update_distance(BTSET,floor(fl_get_slider_value(BT_PARAM1_OBJ)),floor(fl_get_slider_value(BT_PARAM2_OBJ)));
  }
  if(fl_get_button(BT_CHOICE2_OBJ)){
    hri_bt_update_visibility(BTSET,floor(fl_get_slider_value(BT_PARAM1_OBJ)),floor(fl_get_slider_value(BT_PARAM2_OBJ)),
			     floor(fl_get_slider_value(BT_PARAM3_OBJ)));
  }
	
  //g3d_draw_and_col_allwin_active();  
} 

static void g3d_create_param1_obj(void)
{
  BT_PARAM1_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,155.0,245.0,20.0,"");
  fl_set_slider_step(BT_PARAM1_OBJ,10);
  fl_set_slider_bounds(BT_PARAM1_OBJ,0,1000);
  fl_set_slider_value(BT_PARAM1_OBJ,500);
  fl_set_object_callback(BT_PARAM1_OBJ,CB_update_param1_obj,0);
	
}

/*** RADIUS ***/ 
static void CB_update_param2_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_CHOICE1_OBJ)){
    hri_bt_update_distance(BTSET,floor(fl_get_slider_value(BT_PARAM1_OBJ)),floor(fl_get_slider_value(BT_PARAM2_OBJ)));
  }
  if(fl_get_button(BT_CHOICE2_OBJ)){
    hri_bt_update_visibility(BTSET,floor(fl_get_slider_value(BT_PARAM1_OBJ)),floor(fl_get_slider_value(BT_PARAM2_OBJ)),
			     floor(fl_get_slider_value(BT_PARAM3_OBJ)));
  } 
  //g3d_draw_and_col_allwin_active();
}

static void g3d_create_param2_obj(void)
{
  BT_PARAM2_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,175.0,245.0,20.0,"");
  fl_set_slider_step(BT_PARAM2_OBJ,0.05); 
  fl_set_slider_bounds(BT_PARAM2_OBJ,0,20); 
  fl_set_slider_value(BT_PARAM2_OBJ,0); 
  fl_set_object_callback(BT_PARAM2_OBJ,CB_update_param2_obj,0); 
	
}

/*** ONLY FOR VISIBILITY***/
static void CB_update_param3_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_CHOICE2_OBJ)){
    hri_bt_update_visibility(BTSET,floor(fl_get_slider_value(BT_PARAM1_OBJ)),floor(fl_get_slider_value(BT_PARAM2_OBJ)),
			     floor(fl_get_slider_value(BT_PARAM3_OBJ)));
  }
  //g3d_draw_and_col_allwin_active();
} 

static void g3d_create_param3_obj(void)
{
  BT_PARAM3_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,195.0,245.0,20.0,"");
  fl_set_slider_step(BT_PARAM3_OBJ,0.2); 
  fl_set_slider_bounds(BT_PARAM3_OBJ,0,5); 
  fl_set_slider_value(BT_PARAM3_OBJ,2); 
  fl_set_object_callback(BT_PARAM3_OBJ,CB_update_param3_obj,0); 
	
}

/*** ONLY FOR VISIBILITY***/
static void CB_update_param4_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_CHOICE3_OBJ)){
    hri_bt_update_hidzones(BTSET,(double)fl_get_slider_value(BT_PARAM4_OBJ));
  }  
  //g3d_draw_and_col_allwin_active();
} 

static void g3d_create_param4_obj(void)
{
  BT_PARAM4_OBJ = fl_add_valslider(FL_HOR_SLIDER,115.0,215.0,245.0,20.0,"");
  fl_set_slider_step(BT_PARAM4_OBJ,0.2); 
  fl_set_slider_bounds(BT_PARAM4_OBJ,0,5); 
  fl_set_slider_value(BT_PARAM4_OBJ,0); 
  fl_set_object_callback(BT_PARAM4_OBJ,CB_update_param4_obj,0); 
	
}

static void CB_show_bitmaps_obj(FL_OBJECT *ob, long arg)
{  
  double * costtable = NULL;
  int i;
  configPt q;
  p3d_vector3 coord[3];
  double Ccoord[6];
	
  
  p3d_mat4ExtractPosReverseOrder(ACBTSET->robot->joints[ROBOTj_OBJECT]->abs_pos, Ccoord, Ccoord+1, Ccoord+2,
                     Ccoord+3, Ccoord+4, Ccoord+5); 
  
  printf(" endeffector orient : %f %f %f\n",Ccoord[3],Ccoord[4],Ccoord[5]);
  
  if(INTERPOINT == NULL){
    PrintWarning(("Bitmap not initialized"));
    return ;
  }
	
 /*  costtable = MY_ALLOC(double,1500); */
	
/*   if(orderedpointsx == NULL) */
/*     orderedpointsx =  MY_ALLOC(int,1500); */
/*   if(orderedpointsy == NULL) */
/*     orderedpointsy =  MY_ALLOC(int,1500); */
/*   if(orderedpointsz == NULL) */
/*     orderedpointsz =  MY_ALLOC(int,1500); */
/*   orderedlength = 1500; */
	
/*   hri_bt_min_cell_n(INTERPOINT,INTERPOINT->bitmap[BT_3D_COMBINED], */
/* 		    orderedpointsx,orderedpointsy,orderedpointsz,costtable,orderedlength);  */
/*   hri_exp_save_4tables("mincosts10.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 10);  */
/*   hri_exp_save_4tables("mincosts50.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 50); */
/*   hri_exp_save_4tables("mincosts100.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 100); */
/*   hri_exp_save_4tables("mincosts250.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 250); */
/*   hri_exp_save_4tables("mincosts500.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 500); */
/*   hri_exp_save_4tables("mincosts1000.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 1000); */
/*   hri_exp_save_4tables("mincosts1500.dat",orderedpointsx,orderedpointsy,orderedpointsz, costtable, 1500); */
	
  /*  for(i=0; i<50; i++){     */
  /*      hri_gik_compute(INTERPOINT->robot, HRI_GIK, GIK_STEP, 0.01, 0, GIK_FORCE, coord,&q, NULL); */
  /*    } */
	
  /* hri_exp_save_table("mincosts.dat", costtable, 1500); */
	
  /*   for(i=1; i<1500; i++){ */
  /*     costtable[i] = DISTANCE3D( orderedpointsx[i], orderedpointsy[i], orderedpointsz[i], */
  /* 			       orderedpointsx[0], orderedpointsy[0], orderedpointsz[0]) */
  /*       * INTERPOINT->pace;; */
	
  /*   }     */
	
  /*   hri_exp_save_table("mincostsdistance.dat", costtable, 1500); */
	
  /*   for(i=1; i<1500; i++){ */
  /*     costtable[i] =  */
  /*       DISTANCE3D( orderedpointsx[i], orderedpointsy[i], orderedpointsz[i], */
  /* 		  INTERPOINT->human[INTERPOINT->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[0][3], */
  /* 		  INTERPOINT->human[INTERPOINT->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[1][3], */
  /* 		  INTERPOINT->human[INTERPOINT->actual_human]->HumanPt->joints[HUMANj_BODY]->abs_pos[2][3])*INTERPOINT->pace; */
	
  /*   }  */
	
  /*   hri_exp_save_table("mincostsHdistance.dat", costtable, 1500); */
	
/*   MY_FREE(costtable,double,1500); */
	
  hri_exp_find_exchange_point();
	
  fl_set_button(BT_SHOW_BITMAPS_OBJ,0);   
  //g3d_draw_and_col_allwin_active();
	
} 

static void g3d_create_show_bitmaps_obj(void)
{   
  BT_SHOW_BITMAPS_OBJ = fl_add_button(FL_PUSH_BUTTON,10,235,40,20,"TEST");  
  fl_set_call_back(BT_SHOW_BITMAPS_OBJ,CB_show_bitmaps_obj,0);     
}

static gnuplot_ctrl * plot = NULL;

static void CB_save_3dbitmaps_obj(FL_OBJECT *ob, long arg)
{  
  int x,y,z;
  
  fl_set_button(BT_SAVE_3DBITMAPS_OBJ,0); 
	
  //hri_bt_fill_bitmap(INTERPOINT,BT_3D_VISIBILITY);
  hri_bt_fill_bitmap(INTERPOINT,BT_3D_DISTANCE);
  //hri_bt_fill_bitmap(INTERPOINT,BT_3D_HCOMFORT);
  // hri_bt_fill_bitmap(INTERPOINT,BT_3D_RREACH);
  //hri_bt_fill_bitmap(INTERPOINT,BT_3D_COMBINED);
	
  /*   printf("Visibility min: %f, max: %f\n", */
  /*    	 hri_bt_min_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_VISIBILITY], &x,&y,&z), */
  /*      	 hri_bt_max_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_VISIBILITY], &x,&y,&z)	 ); */
  /*   printf("Distance min: %f, max: %f\n", */
  /*      	 hri_bt_min_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_DISTANCE], &x,&y,&z), */
  /*      	 hri_bt_max_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_DISTANCE], &x,&y,&z)       ); */
  /*   printf("HumanReach min: %f, max: %f\n", */
  /*      	 hri_bt_min_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_HCOMFORT], &x,&y,&z), */
  /*      	 hri_bt_max_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_HCOMFORT], &x,&y,&z)        ); */
  /*   printf("RobotReach min: %f, max: %f\n", */
  /* 	 hri_bt_min_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_RREACH], &x,&y,&z), */
  /* 	 hri_bt_max_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_RREACH], &x,&y,&z)        ); */
  /*   printf("Combined min: %f, max: %f\n", */
  /* 	 hri_bt_min_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_COMBINED], &x,&y,&z), */
  /* 	 hri_bt_max_cell(INTERPOINT, INTERPOINT->bitmap[BT_3D_COMBINED], &x,&y,&z)       ); */
  
  
  //hri_exp_save(INTERPOINT,INTERPOINT->bitmap[BT_3D_VISIBILITY],"costsV.dat",-1);
  //hri_exp_save(INTERPOINT,INTERPOINT->bitmap[BT_3D_DISTANCE],"costsD.dat",10);
  //hri_exp_save(INTERPOINT,INTERPOINT->bitmap[BT_3D_HCOMFORT],"costsHR.dat",10);
  // hri_exp_save(INTERPOINT,INTERPOINT->bitmap[BT_3D_RREACH],"costsRR.dat",0);
  //hri_exp_save(INTERPOINT,INTERPOINT->bitmap[BT_3D_COMBINED],"costsC.dat",0);
  //hri_exp_save(INTERPOINT,INTERPOINT->bitmap[BT_3D_OBSTACLES],"costsO.dat",0);
  //p3d_hri_save_npoint(INTERPOINT,INTERPOINT->bitmap[BT_3D_COMBINED],"costsCn.dat",1,40);
  if(plot == NULL)
    plot = hri_bt_init_gnuplot(2,6,-1,-5,0,3);
  
  hri_bt_gnuplot_bitmap(plot,INTERPOINT,BT_3D_DISTANCE,10);
  
  return;
  
  configPt qrobotwb;
  configPt qrobotwob;
  p3d_rob * robotwb, * robotwob;
  int i;
  
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
  for(i=0; i<env->nr; i++){
    if( !strcmp("robot",env->robot[i]->name) ){
      robotwob = env->robot[i];
    }
    if( !strcmp("robot1",env->robot[i]->name) ){ /* with bottle */
      robotwb = env->robot[i];
    }
  }

  if(BTSET!=NULL && BTSET->robot!=NULL){
    if(BTSET->robot == robotwob){
      qrobotwob = p3d_get_robot_config(robotwob);
      p3d_set_and_update_this_robot_conf(robotwb,qrobotwob);
      p3d_copy_config_into(robotwob,qrobotwob,&robotwb->ROBOT_POS);
      p3d_copy_config_into(robotwob,qrobotwob,&robotwb->ROBOT_GOTO);
      qrobotwob[6] = -9;
      qrobotwob[7] = -12;
      p3d_set_and_update_this_robot_conf(robotwob,qrobotwob);
      BTSET->robot = robotwb;
      if(INTERPOINT!=NULL && INTERPOINT->robot!=NULL)
	INTERPOINT->robot = robotwb;
      if(OBJSET!=NULL && OBJSET->robot!=NULL)
	OBJSET->robot = robotwb;
      PSP_ROBOT = robotwb;
    }
    else{
      if(BTSET->robot == robotwb){
	qrobotwb = p3d_get_robot_config(robotwb);
	p3d_set_and_update_this_robot_conf(robotwob,qrobotwb);
	p3d_copy_config_into(robotwob,qrobotwb,&robotwob->ROBOT_POS);
	p3d_copy_config_into(robotwob,qrobotwb,&robotwob->ROBOT_GOTO);
	qrobotwb[6] = -9;
	qrobotwb[7] = -12;
	p3d_set_and_update_this_robot_conf(robotwb,qrobotwb);
	BTSET->robot = robotwob;
	if(INTERPOINT!=NULL && INTERPOINT->robot!=NULL)
	  INTERPOINT->robot = robotwob;
	if(OBJSET!=NULL && OBJSET->robot!=NULL)
	  OBJSET->robot = robotwob;
	   PSP_ROBOT = robotwob;
      }
    }
  }
  else{
     if(INTERPOINT!=NULL && INTERPOINT->robot!=NULL){
       if(INTERPOINT->robot == robotwob){
	 qrobotwob = p3d_get_robot_config(robotwob);
	 p3d_set_and_update_this_robot_conf(robotwb,qrobotwob);
	 p3d_copy_config_into(robotwob,qrobotwob,&robotwb->ROBOT_POS);
	 p3d_copy_config_into(robotwob,qrobotwob,&robotwb->ROBOT_GOTO);
	 qrobotwob[6] = -9;
	 qrobotwob[7] = -12;
	 p3d_set_and_update_this_robot_conf(robotwob,qrobotwob);
	 INTERPOINT->robot = robotwb;
	 if(OBJSET!=NULL && OBJSET->robot!=NULL)
	   OBJSET->robot = robotwb;
	 PSP_ROBOT = robotwb;
       }
       else{
	 if(INTERPOINT->robot == robotwb){
	   qrobotwb = p3d_get_robot_config(robotwb);
	   p3d_set_and_update_this_robot_conf(robotwob,qrobotwb);
	   p3d_copy_config_into(robotwob,qrobotwb,&robotwob->ROBOT_POS);
	   p3d_copy_config_into(robotwob,qrobotwb,&robotwob->ROBOT_GOTO);
	   qrobotwb[6] = -9;
	   qrobotwb[7] = -12;
	   p3d_set_and_update_this_robot_conf(robotwb,qrobotwb);
	   INTERPOINT->robot = robotwob;
	   if(OBJSET!=NULL && OBJSET->robot!=NULL)
	     OBJSET->robot = robotwob;
	      PSP_ROBOT = robotwob;
	 }
       }
     }
     else{
         if(OBJSET!=NULL && OBJSET->robot!=NULL){
	   if(OBJSET->robot == robotwob){
	     qrobotwob = p3d_get_robot_config(robotwob);
	     p3d_set_and_update_this_robot_conf(robotwb,qrobotwob);
	     p3d_copy_config_into(robotwob,qrobotwob,&robotwb->ROBOT_POS);
	     p3d_copy_config_into(robotwob,qrobotwob,&robotwb->ROBOT_GOTO);
	     qrobotwob[6] = -9;
	     qrobotwob[7] = -12;
	     p3d_set_and_update_this_robot_conf(robotwob,qrobotwob);
	     OBJSET->robot = robotwb;
	     PSP_ROBOT = robotwb;
	   }
	   else{
	     if(OBJSET->robot == robotwb){
	       qrobotwb = p3d_get_robot_config(robotwb);
	       p3d_set_and_update_this_robot_conf(robotwob,qrobotwb);
	       p3d_copy_config_into(robotwob,qrobotwb,&robotwob->ROBOT_POS);
	       p3d_copy_config_into(robotwob,qrobotwb,&robotwob->ROBOT_GOTO);
	       qrobotwb[6] = -9;
	       qrobotwb[7] = -12;
	       p3d_set_and_update_this_robot_conf(robotwb,qrobotwb);
	       OBJSET->robot = robotwob;
	       PSP_ROBOT = robotwob;
	     }
	   }
	 }
     }
  }
  
  
  



	
  //g3d_draw_and_col_allwin_active();
	
} 

static void g3d_create_save_3dbitmaps_obj(void)
{ 
  BT_SAVE_3DBITMAPS_OBJ = fl_add_button(FL_PUSH_BUTTON,10,255,40,20,"SAVE");  
  fl_set_call_back(BT_SAVE_3DBITMAPS_OBJ,CB_save_3dbitmaps_obj,0);     
}

static void CB_save_choice_obj(FL_OBJECT *ob, long arg)
{
  if(ACBTSET==NULL){
    return;
  }
  
  if(arg==0){   
    fl_set_slider_value(BT_PARAM1_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].dheight);
    fl_set_slider_value(BT_PARAM2_OBJ,BTSET->human[BTSET->actual_human]->state[BTSET->human[BTSET->actual_human]->actual_state].dradius);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,0);
    
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
  }

}

static void g3d_create_save_choice_obj(void)
{
  FL_OBJECT *obj;
  
  obj = fl_add_frame(FL_ENGRAVED_FRAME,60,255,60,20,""); 
  //obj = fl_add_box(FL_FLAT_BOX,60,255,100,10,"3D Bitmaps"); 
  
  SAVEGROUP = fl_bgn_group();
  BT_SAVE_CHOICE1_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,60,255,50,20,"D");
  fl_set_object_color(BT_SAVE_CHOICE1_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_SAVE_CHOICE1_OBJ,CB_save_choice_obj,0);
  BT_SAVE_CHOICE2_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,75,255,50,20,"V");
  fl_set_object_color(BT_SAVE_CHOICE2_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_SAVE_CHOICE2_OBJ,CB_save_choice_obj,1);
  BT_SAVE_CHOICE3_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,90,255,50,20,"A");
  fl_set_object_color(BT_SAVE_CHOICE3_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_SAVE_CHOICE3_OBJ,CB_save_choice_obj,2);
  BT_SAVE_CHOICE4_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,105,255,50,20,"C");
  fl_set_object_color(BT_SAVE_CHOICE4_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(BT_SAVE_CHOICE4_OBJ,CB_save_choice_obj,3);
  SAVEGROUP = fl_end_group();
  
} 




static void CB_placement_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_PLCMT_OBJ1))
    place= 1;
  if(fl_get_button(BT_PLCMT_OBJ2))
    place= 2;
  if(fl_get_button(BT_PLCMT_OBJ3))
    place= 3;
  if(fl_get_button(BT_PLCMT_OBJ4))
    place= 4;
  if(fl_get_button(BT_PLCMT_OBJ5))
    place= 5;
  if(fl_get_button(BT_PLCMT_OBJ6))
    place= 6;
  if(fl_get_button(BT_PLCMT_OBJ7))
    place= 7;
  if(fl_get_button(BT_PLCMT_OBJ8))
    place= 8;
	
  hri_bt_init_TARGET(place, place_type);
  g3d_draw_allwin_active();
}

static void g3d_create_placement_obj(void)
{
  place = 0;
	
  PLACEMENTGROUP = fl_bgn_group();
	
  BT_PLCMT_OBJ1 = fl_add_checkbutton(FL_RADIO_BUTTON,270,100,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ1,CB_placement_obj,0);
  BT_PLCMT_OBJ2 = fl_add_checkbutton(FL_RADIO_BUTTON,285,105,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ2,CB_placement_obj,0);
  BT_PLCMT_OBJ3 = fl_add_checkbutton(FL_RADIO_BUTTON,290,120,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ3,CB_placement_obj,0);
  BT_PLCMT_OBJ4 = fl_add_checkbutton(FL_RADIO_BUTTON,285,135,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ4,CB_placement_obj,0);
  BT_PLCMT_OBJ5 = fl_add_checkbutton(FL_RADIO_BUTTON,270,140,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ5,CB_placement_obj,0);
  BT_PLCMT_OBJ6 = fl_add_checkbutton(FL_RADIO_BUTTON,255,135,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ6,CB_placement_obj,0);
  BT_PLCMT_OBJ7 = fl_add_checkbutton(FL_RADIO_BUTTON,250,120,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ7,CB_placement_obj,0);
  BT_PLCMT_OBJ8 = fl_add_checkbutton(FL_RADIO_BUTTON,255,105,20,20,"");
  fl_set_call_back(BT_PLCMT_OBJ8,CB_placement_obj,0);
	
  PLACEMENTGROUP = fl_end_group();  
}

static void CB_plcmt_type_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_PLCMT_TYPE_OBJ1)){
    place_type = BT_TRG_LOOK;
    hri_bt_init_TARGET(place,place_type);
  }
  if(fl_get_button(BT_PLCMT_TYPE_OBJ2)){
    place_type = BT_TRG_BODY;
    hri_bt_init_TARGET(place,place_type);
  }
  if(fl_get_button(BT_PLCMT_TYPE_OBJ3)){
    place_type = BT_TRG_APPROACH;
    hri_bt_init_TARGET(place,place_type);
  }
	
	
  g3d_draw_allwin_active();
}

static void g3d_create_plcmt_type_obj(void)
{
  place_type = 0;
	
  PLCMTTYPEGROUP = fl_bgn_group();
	
  BT_PLCMT_TYPE_OBJ1 = fl_add_checkbutton(FL_RADIO_BUTTON,310,110,20,20,"Look");
  fl_set_call_back(BT_PLCMT_TYPE_OBJ1,CB_plcmt_type_obj,0);
  BT_PLCMT_TYPE_OBJ2 = fl_add_checkbutton(FL_RADIO_BUTTON,310,125,20,20,"Body");
  fl_set_call_back(BT_PLCMT_TYPE_OBJ2,CB_plcmt_type_obj,0);
  BT_PLCMT_TYPE_OBJ3 = fl_add_checkbutton(FL_RADIO_BUTTON,310,140,20,20,"Go!");
  fl_set_call_back(BT_PLCMT_TYPE_OBJ3,CB_plcmt_type_obj,0);
	
  PLCMTTYPEGROUP = fl_end_group();
	
}

static void CB_select_human_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_choice(ob);
	
  ACBTSET->actual_human = val-1; 
	
  fl_set_choice(HUMAN_STATE,ACBTSET->human[ACBTSET->actual_human]->actual_state+1);
  fl_set_choice(HUMAN_EXISTS,ACBTSET->human[ACBTSET->actual_human]->exists+1);
  if(fl_get_button(BT_CHOICE1_OBJ)) {
    fl_set_slider_value(BT_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dheight);
    fl_set_slider_value(BT_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dradius);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,0);
		
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
  }
  if( fl_get_button(BT_CHOICE2_OBJ)) {
    fl_set_slider_value(BT_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vheight);
    fl_set_slider_value(BT_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vback);
    fl_set_slider_value(BT_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vsides); 
    fl_set_slider_value(BT_PARAM4_OBJ,0);
		
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_activate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
		
  }  
  if(fl_get_button(BT_CHOICE3_OBJ)) {
    fl_set_slider_value(BT_PARAM1_OBJ,0);
    fl_set_slider_value(BT_PARAM2_OBJ,0);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].hradius);
		
    fl_deactivate_object(BT_PARAM1_OBJ);
    fl_deactivate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_activate_object(BT_PARAM4_OBJ);
  }
  if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
  if(INTERPOINT!=NULL){
    hri_bt_3drefresh_all(INTERPOINT);
  }
  //g3d_draw_and_col_allwin_active();
	
}

static void g3d_create_select_human_obj(void)
{
//	int i;
	ACTUAL_HUMAN = fl_add_choice(FL_NORMAL_CHOICE,130.0,250.,70.0,20.0,"");
	
  /* for(i=0; i<BTSET->human_no; i++) */
  /*     fl_addto_choice(ACTUAL_HUMAN, BTSET->human[i]->HumanPt->name); */
	
  fl_set_choice(ACTUAL_HUMAN,1);
  fl_set_call_back(ACTUAL_HUMAN,CB_select_human_obj,0);
}

static void CB_select_state_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_choice(ob);
  configPt q;
  p3d_rob* robotPt;
	
  if(ACBTSET==NULL)
    return;
  ACBTSET->human[ACBTSET->actual_human]->actual_state = val-1;
  if(fl_get_button(BT_CHOICE1_OBJ)) {
    fl_set_slider_value(BT_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dheight);
    fl_set_slider_value(BT_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].dradius);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,0);
		
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
  }
  if( fl_get_button(BT_CHOICE2_OBJ)) {
    fl_set_slider_value(BT_PARAM1_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vheight);
    fl_set_slider_value(BT_PARAM2_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vback);
    fl_set_slider_value(BT_PARAM3_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].vsides); 
    fl_set_slider_value(BT_PARAM4_OBJ,0);
		
    fl_activate_object(BT_PARAM1_OBJ);
    fl_activate_object(BT_PARAM2_OBJ);
    fl_activate_object(BT_PARAM3_OBJ);
    fl_deactivate_object(BT_PARAM4_OBJ);
		
  }  
  if(fl_get_button(BT_CHOICE3_OBJ)) {
    fl_set_slider_value(BT_PARAM1_OBJ,0);
    fl_set_slider_value(BT_PARAM2_OBJ,0);
    fl_set_slider_value(BT_PARAM3_OBJ,0);
    fl_set_slider_value(BT_PARAM4_OBJ,ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].hradius);
		
    fl_deactivate_object(BT_PARAM1_OBJ);
    fl_deactivate_object(BT_PARAM2_OBJ);
    fl_deactivate_object(BT_PARAM3_OBJ);
    fl_activate_object(BT_PARAM4_OBJ);
  }
	
  q = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);  
  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c7;
  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c1;
  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c2;
  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c3;
  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c4;
  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c5;
  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[ACBTSET->human[ACBTSET->actual_human]->actual_state].c6;
	
  robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
  p3d_sel_desc_name(P3D_ROBOT,ACBTSET->human[ACBTSET->actual_human]->HumanPt->name);
	
  p3d_set_and_update_robot_conf(q); 
  p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
	
	
  ACBTSET->changed = TRUE;
  if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
  if(INTERPOINT!=NULL){
    hri_bt_3drefresh_all(INTERPOINT);
  }
	
  //g3d_draw_and_col_allwin_active();
	
}

static void g3d_create_select_state_obj(void)
{

//	int i;
	HUMAN_STATE = fl_add_choice(FL_NORMAL_CHOICE,280.0,250.,60.0,20.0,"");

	
  /* for(i=0; i<BTSET->human[BTSET->actual_human]->states_no; i++) */
  /*     fl_addto_choice(HUMAN_STATE, BTSET->human[BTSET->actual_human]->state[i].name); */
	
  // fl_set_choice(HUMAN_STATE,BTSET->human[BTSET->actual_human]->actual_state+1);
  fl_set_call_back(HUMAN_STATE,CB_select_state_obj,0);
}

static void CB_select_exists_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_choice(ob);
	
  if(ACBTSET==NULL)
    return;
	
  ACBTSET->human[ACBTSET->actual_human]->exists = val-1;
  ACBTSET->changed = TRUE;
  if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
  if(INTERPOINT!=NULL)
    hri_bt_3drefresh_all(INTERPOINT);
  //g3d_draw_and_col_allwin_active();
	
}


static void g3d_create_select_exists_obj(void)
{
  HUMAN_EXISTS = fl_add_choice(FL_NORMAL_CHOICE,210.0,250.,60.0,20.0,"");
	
  fl_addto_choice(HUMAN_EXISTS, "not exists");
  fl_addto_choice(HUMAN_EXISTS, "exists");
	
  fl_set_choice(HUMAN_EXISTS,1);
  fl_set_call_back(HUMAN_EXISTS,CB_select_exists_obj,0);
} 

/***********************************************************************/
/***********************************************************************/
/*                              GIK MENU                               */
/***********************************************************************/ 
/***********************************************************************/
static int DIRECT = FALSE;


static void CB_run_GIKstep_obj(FL_OBJECT *ob, long arg)
{  
  fl_set_button(BT_RUN_GIKstep_OBJ,0); 
  hri_gik_computestepWoP(BTSET->robot, HRI_GIK, GIK_STEP, DIRECT, GIK_FORCE);
	
  //g3d_draw_and_col_allwin_active();
	
} 

static void g3d_create_run_GIKstep_obj(void)
{ 
  HRI_GIK = hri_gik_create_gik();
  BT_RUN_GIKstep_OBJ = fl_add_button(FL_PUSH_BUTTON,80,275,80,40,"Run n step GIK");  
  fl_set_call_back(BT_RUN_GIKstep_OBJ,CB_run_GIKstep_obj,0);     
}

static void CB_run_GIKstepper_obj(FL_OBJECT *ob, long arg)
{  
  fl_set_button(BT_RUN_GIKstepper_OBJ,0); 
  hri_gik_computestepPer(BTSET->robot, HRI_GIK, GIK_STEP, GIK_FORCE);
	
  //g3d_draw_and_col_allwin_active();
	
} 

static void g3d_create_run_GIKstepper_obj(void)
{   
  BT_RUN_GIKstepper_OBJ = fl_add_button(FL_PUSH_BUTTON,170,275,80,40,"Run Perturbation");  
  fl_set_call_back(BT_RUN_GIKstepper_OBJ,CB_run_GIKstepper_obj,0);     
}

static void CB_update_interpoint_weight1_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_IPW1_OBJ)){
    HRI_WEIGHTS[0] = atof(fl_get_input(BT_IPW1_OBJ));
    HRI_WEIGHTS[1] = atof(fl_get_input(BT_IPW2_OBJ)); 
    HRI_WEIGHTS[2] = atof(fl_get_input(BT_IPW3_OBJ));
    printf("values: %f, %f, %f\n",HRI_WEIGHTS[0],HRI_WEIGHTS[1],HRI_WEIGHTS[2]); 
  }
  //hri_exp_find_10_exchange_point(INTERPOINT);
  //g3d_draw_and_col_allwin_active();
} 

static void g3d_create_interpoint_weight1_obj(void)
{
  BT_IPW1_OBJ = fl_add_input(FL_FLOAT_INPUT,290,275,40,30.0,
			     "DisVal");
  
  /* BT_IPW1_OBJ = fl_add_valslider(FL_VERT_SLIDER,270,275,20,160,""); */
  /*   fl_set_slider_step(BT_IPW1_OBJ,0.001);  */
  /*   fl_set_slider_bounds(BT_IPW1_OBJ,0,2);  */
  /*   fl_set_slider_value(BT_IPW1_OBJ,1);  */
  fl_set_input(BT_IPW1_OBJ,"1"); 
  fl_set_object_callback(BT_IPW1_OBJ,CB_update_interpoint_weight1_obj,0); 
	
}

static void CB_interpoint_distance_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_3D_DISTANCE,INTERPOINT)) hri_bt_activate(BT_3D_DISTANCE,INTERPOINT);
  else                                             hri_bt_desactivate(BT_3D_DISTANCE,INTERPOINT);
	
  g3d_draw_allwin_active();
} 

static void g3d_create_interpoint_distance_active_obj(void)
{
  BT_INTDISTANCE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,270,440,50,20,"");  
  fl_set_call_back(BT_INTDISTANCE_OBJ,CB_interpoint_distance_active_obj,0);
	
} 

static void CB_update_interpoint_weight2_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_IPW2_OBJ)){
    HRI_WEIGHTS[0] = atof(fl_get_input(BT_IPW1_OBJ));
    HRI_WEIGHTS[1] = atof(fl_get_input(BT_IPW2_OBJ)); 
    HRI_WEIGHTS[2] = atof(fl_get_input(BT_IPW3_OBJ));
    printf("values: %f, %f, %f\n",HRI_WEIGHTS[0],HRI_WEIGHTS[1],HRI_WEIGHTS[2]); 
  }
  //hri_exp_find_10_exchange_point(INTERPOINT);
  //g3d_draw_and_col_allwin_active();
} 

static void g3d_create_interpoint_weight2_obj(void)
{

  BT_IPW2_OBJ = fl_add_input(FL_FLOAT_INPUT,290,305,40,30.0,
			     "Visibil");

/*   BT_IPW2_OBJ = fl_add_valslider(FL_VERT_SLIDER,300,275,20,160,""); */
/*   fl_set_slider_step(BT_IPW2_OBJ,0.01);  */
/*   fl_set_slider_bounds(BT_IPW2_OBJ,0,5);  */
  fl_set_input(BT_IPW2_OBJ,"1"); 
  fl_set_object_callback(BT_IPW2_OBJ,CB_update_interpoint_weight2_obj,0); 
	
}

static void CB_interpoint_visibility_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_3D_VISIBILITY,INTERPOINT)) hri_bt_activate(BT_3D_VISIBILITY,INTERPOINT);
  else                                     hri_bt_desactivate(BT_3D_VISIBILITY,INTERPOINT);
	
  g3d_draw_allwin_active();
} 

static void g3d_create_interpoint_visibility_active_obj(void)
{
  BT_INTVISIBILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,300,440,50,20,"");  
  fl_set_call_back(BT_INTVISIBILITY_OBJ,CB_interpoint_visibility_active_obj,0);
	
} 

static void CB_update_interpoint_weight3_obj(FL_OBJECT *ob, long arg)
{
  if(fl_get_button(BT_IPW3_OBJ)){
    HRI_WEIGHTS[0] = atof(fl_get_input(BT_IPW1_OBJ));
    HRI_WEIGHTS[1] = atof(fl_get_input(BT_IPW2_OBJ)); 
    HRI_WEIGHTS[2] = atof(fl_get_input(BT_IPW3_OBJ));
    printf("values: %f, %f, %f\n",HRI_WEIGHTS[0],HRI_WEIGHTS[1],HRI_WEIGHTS[2]); 
  }
  //hri_exp_find_10_exchange_point(INTERPOINT);
  //g3d_draw_and_col_allwin_active();
} 

static void g3d_create_interpoint_weight3_obj(void)
{
  BT_IPW3_OBJ = fl_add_input(FL_FLOAT_INPUT,290,335,40,30.0,
			     "HArm");
  
  /* BT_IPW3_OBJ = fl_add_valslider(FL_VERT_SLIDER,330,275,20,160,""); */
  /*   fl_set_slider_step(BT_IPW3_OBJ,0.01);  */
  /*   fl_set_slider_bounds(BT_IPW3_OBJ,0,5);  */
  /*   fl_set_slider_value(BT_IPW3_OBJ,1);  */
  fl_set_input(BT_IPW3_OBJ,"1"); 
  fl_set_object_callback(BT_IPW3_OBJ,CB_update_interpoint_weight3_obj,0); 
	
}

static void CB_interpoint_humanreach_active_obj(FL_OBJECT *ob, long arg)
{
  if(!hri_bt_is_active(BT_3D_OBSTACLES,ACBTSET)) hri_bt_activate(BT_3D_OBSTACLES,ACBTSET);
  else                                           hri_bt_desactivate(BT_3D_OBSTACLES,ACBTSET);
  if(!hri_bt_is_active(BT_3D_PATH,ACBTSET)) hri_bt_activate(BT_3D_PATH,ACBTSET);
  else                                           hri_bt_desactivate(BT_3D_PATH,ACBTSET);
  
  g3d_draw_allwin_active();
} 

static void g3d_create_interpoint_humanreach_active_obj(void)
{
  BT_INTHUMANR_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,330,440,50,20,"");  
  fl_set_call_back(BT_INTHUMANR_OBJ,CB_interpoint_humanreach_active_obj,0);
	
} 

static void CB_select_bitmapset_obj(FL_OBJECT *ob, long arg)
{
  int val = fl_get_choice(ob);
	
  if(val == 1)
    ACBTSET = BTSET;
  if(val == 2)
    ACBTSET = INTERPOINT;
  if(val == 3)
    ACBTSET = OBJSET;
	
}


static void g3d_create_select_bitmapset_obj(void)
{
  SELECT_BTSET = fl_add_choice(FL_NORMAL_CHOICE,270,460.,80.0,20.0,"");
	
  fl_addto_choice(SELECT_BTSET, "Navigation");
  fl_addto_choice(SELECT_BTSET, "Manipulation");
  fl_addto_choice(SELECT_BTSET, "Object Reach");
	
  fl_set_choice(HUMAN_EXISTS,1);
  fl_set_call_back(SELECT_BTSET,CB_select_bitmapset_obj,0);
	
  BTSET = NULL;
  INTERPOINT = NULL;
	
  ACBTSET = BTSET;
} 

static void CB_update_drawno_obj(FL_OBJECT *ob, long arg)
{
  ordereddrawno =(int) floor(fl_get_slider_value(BT_DRAWNO_OBJ));
	
  //g3d_draw_and_col_allwin_active();
} 

static void g3d_create_drawno_obj(void)
{
  BT_DRAWNO_OBJ = fl_add_valslider(FL_HOR_SLIDER,10,460,160,20,"");
  fl_set_slider_step(BT_DRAWNO_OBJ,1); 
  fl_set_slider_bounds(BT_DRAWNO_OBJ,0,500); 
  fl_set_slider_value(BT_DRAWNO_OBJ,0); 
  fl_set_object_callback(BT_DRAWNO_OBJ,CB_update_drawno_obj,0); 
	
}


/*******************************************/

static void CB_run_GIK_obj(FL_OBJECT *ob, long arg)
{ 
  p3d_vector3 orient[3]; 
  p3d_vector3 coord[3];
  p3d_rob * visball = NULL, *bottle=NULL;
  int i;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  double qarm[6];
  
  p3d_rob * robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
  configPt q;
  p3d_vector3 axe1={1,0,0}, axe2={0,1,0}, axe3={0,0,1};
  double value1[6], value2[6],value3[6];
  p3d_matrix4 object_mat,inv;
  
  q = p3d_get_robot_config(robotPt);
	
  fl_set_button(BT_RUN_GIK_OBJ,0); 
	
  for(i=0; i<env->nr; i++){
    if( !strcmp("visball",env->robot[i]->name) ){
      visball = env->robot[i];
    }
    if( !strcmp("bottle",env->robot[i]->name) ){
      bottle = env->robot[i];
    }
  }
  /* CORRECT THESE AKIN */
/* #ifdef BH */
/*   p3d_mat4ExtractPosReverseOrder(bottle->joints[1]->abs_pos, */
/* 		     &coord[0][0], &coord[0][1], &coord[0][2], orient, orient+1, orient+2); */
/*   p3d_mat4ExtractPosReverseOrder(bottle->joints[1]->abs_pos, */
/* 		     &coord[1][0], &coord[1][1], &coord[1][2], orient, orient+1, orient+2); */
/*   p3d_mat4ExtractPosReverseOrder(bottle->joints[1]->abs_pos, */
/* 		     &coord[2][0], &coord[2][1], &coord[2][2], orient, orient+1, orient+2); */
/* #endif */
/*   p3d_matInvertXform( robotPt->joints[ROBOTj_OBJECT]->abs_pos, inv); */
/*   p3d_mat4Mult(inv, bottle->joints[1]->abs_pos, object_mat); */
  
/*   p3d_mat4ExtractPosReverseOrder(object_mat, */
/*                      &coord[0][0], &coord[0][1], &coord[0][2], &orient[0][0], &orient[0][1], &orient[0][2]); */
/*   printf("jorient1:%f , jorient2:%f , jorient3:%f \n",orient[0][0],orient[0][1], orient[0][2]); */
  
  
/*   p3d_mat4ExtractPosReverseOrder(robotPt->joints[ROBOTj_OBJECT]->abs_pos, */
/*                      &coord[0][0], &coord[0][1], &coord[0][2], &orient[0][0], &orient[0][1], &orient[0][2]); */
/*   printf("jorient1:%f , jorient2:%f , jorient3:%f \n",orient[0][0],orient[0][1], orient[0][2]); */
  
  
/* #ifdef JIDO */
/*   p3d_mat4ExtractPosReverseOrder(bottle->joints[1]->abs_pos, */
/* 		     &coord[0][0], &coord[0][1], &coord[0][2], &orient[0][0], &orient[0][1], &orient[0][2]); */
/*   printf("orient1:%f , orient2:%f , orient3:%f \n",orient[0][0],orient[0][1], orient[0][2]); */
/* #endif */
  
  configPt qr,qbot;

  qr = p3d_get_robot_config(robotPt);
  qbot = p3d_get_robot_config(bottle);
  qbot[10] = -(M_PI/2) -  atan2(qr[7]-qbot[7],qr[6]-qbot[6]);
  
  p3d_set_and_update_this_robot_conf(bottle,qbot);
			   
  p3d_matrix4 invers,mat,ref;
  configPt qPt;
  Gb_6rParameters * bras;
  Gb_th * eth;
  Gb_q6 * old_q, * qarm2;
  Gb_dataMGD * d;
  int res;
  double r7 = 0.25;
  int eee[8][3] = {
    {  1,  1,  1 },
    {  1,  1, -1 },
    {  1, -1,  1 },
    {  1, -1, -1 },
    { -1,  1,  1 },
    { -1,  1, -1 },
    { -1, -1,  1 },
    { -1, -1, -1 }
  };
  
  bras = MY_ALLOC(Gb_6rParameters,1);
  eth = MY_ALLOC(Gb_th,1);
  old_q = MY_ALLOC(Gb_q6,1);
  qarm2 = MY_ALLOC(Gb_q6,1);
  d = MY_ALLOC(Gb_dataMGD,1);
  
  p3d_mat4Copy(robotPt->joints[1]->abs_pos,ref);

  ref[0][3] = robotPt->joints[6]->abs_pos[0][3];
  ref[1][3] = robotPt->joints[6]->abs_pos[1][3];
  ref[2][3] = robotPt->joints[6]->abs_pos[2][3];

  p3d_matInvertXform(ref, invers);
  p3d_mat4Mult(invers,bottle->joints[1]->abs_pos,mat);

  
  eth->vx.x = mat[0][0];
  eth->vx.y = mat[1][0];
  eth->vx.z = mat[2][0];

  eth->vy.x = mat[0][1];
  eth->vy.y = mat[1][1];
  eth->vy.z = mat[2][1];

  eth->vz.x = mat[0][2];
  eth->vz.y = mat[1][2];
  eth->vz.z = mat[2][2];

  eth->vp.x = mat[0][3] - r7 * mat[0][2];
  eth->vp.y = mat[1][3] - r7 * mat[1][2];
  eth->vp.z = mat[2][3]  - r7 * mat[2][2];

  bras->a2 = 0.45;
  bras->r4 = 0.48;
  bras->epsilon = 0.01;
  bras->of1 = M_PI;
  bras->of2 = -M_PI/2;
  bras->of3 = -M_PI/2;
  bras->of4 = 0;
  bras->of5 = 0;
  bras->of6 = 0;

  qPt = p3d_get_robot_config(robotPt);
  old_q->q1 = qPt[5];
  old_q->q2 = qPt[6];
  old_q->q3 = qPt[7];
  old_q->q4 = qPt[8];
  old_q->q5 = qPt[9];
  old_q->q6 = qPt[10];
  
  // res =  Gb_MGI6rTh_O(bras, eth, old_q, d, qarm2); 
  

    res=Gb_MGI6rTh(bras, eth, -1,  -1, -1, old_q, d, qarm2);
    
    if(res == MGI_OK){
      printf("DONE\n");
    }
    q[12] = qarm2->q1;
    q[13] = qarm2->q2;
    q[14] = qarm2->q3;
    q[15] = qarm2->q4;
    q[16] = qarm2->q5;
    q[17] = qarm2->q6;
    p3d_set_and_update_this_robot_conf(robotPt,q);
  
  
    
  //hri_gik_compute(robotPt, HRI_GIK, GIK_STEP, 0.01, DIRECT, GIK_FORCE, coord, orient, &q, NULL);

	
  p3d_set_and_update_this_robot_conf(robotPt,q);
  p3d_mat4ExtractPosReverseOrder(robotPt->joints[ROBOTj_OBJECT]->abs_pos,value1,value1+1,value1+2,
		     value1+3,value1+4,value1+5);


  printf("ANGLES ARE: %f %f %f\n",value1[3],value1[4],value1[5]);


  p3d_destroy_config(robotPt,q);
	
  //g3d_draw_and_col_allwin_active();
	
} 



static void g3d_create_run_GIK_obj(void)
{   
  BT_RUN_GIK_OBJ = fl_add_button(FL_PUSH_BUTTON,10,275,60,40,"Run GIK");  
  fl_set_call_back(BT_RUN_GIK_OBJ,CB_run_GIK_obj,0);     
}

static void CB_gik_direct_obj(FL_OBJECT *ob, long arg)
{
  if(HRI_GIK_CONTINUOUS == TRUE)  HRI_GIK_CONTINUOUS = FALSE;    
  else                            HRI_GIK_CONTINUOUS = TRUE;  
	
  g3d_draw_allwin_active();      
}

static void g3d_create_gik_direct_obj(void)
{ 
  GIK_DIRECT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,10,310,30,30,"Continuous");
  fl_set_button(GIK_DIRECT_OBJ,1);
  fl_set_call_back(GIK_DIRECT_OBJ,CB_gik_direct_obj,0);  
}

static void CB_update_GIKstep_obj(FL_OBJECT *ob, long arg)
{
	
  GIK_STEP = floor(fl_get_slider_value(BT_GIKSTEP_OBJ));
	
  //g3d_draw_and_col_allwin_active();  
} 

static void g3d_create_GIKstep_obj(void)
{
  BT_GIKSTEP_OBJ = fl_add_valslider(FL_HOR_SLIDER,10,340,250,20.0,"");
  fl_set_slider_step(BT_GIKSTEP_OBJ,1);
  fl_set_slider_bounds(BT_GIKSTEP_OBJ,1,200);
  fl_set_slider_value(BT_GIKSTEP_OBJ,10);
  GIK_STEP = 10;
  fl_set_object_callback(BT_GIKSTEP_OBJ,CB_update_GIKstep_obj,0);
	
}

static void CB_update_GIKforce_obj(FL_OBJECT *ob, long arg)
{
	
  GIK_FORCE = fl_get_slider_value(BT_GIKFORCE_OBJ);
	
  //g3d_draw_and_col_allwin_active();  
} 

static void g3d_create_GIKforce_obj(void)
{
  BT_GIKFORCE_OBJ = fl_add_valslider(FL_HOR_SLIDER,10,360,250,20.0,"");
  fl_set_slider_step(BT_GIKFORCE_OBJ,0.1);
  fl_set_slider_bounds(BT_GIKFORCE_OBJ,0,20);
  fl_set_slider_value(BT_GIKFORCE_OBJ,1);
  GIK_FORCE = 1;
  fl_set_object_callback(BT_GIKFORCE_OBJ,CB_update_GIKforce_obj,0);
	
}

static void CB_update_GIKvision_obj(FL_OBJECT *ob, long arg)
{
	
  GIK_VIS = floor(fl_get_slider_value(BT_GIKVISION_OBJ));
	
  //g3d_draw_and_col_allwin_active();  
} 

static void g3d_create_GIKvision_obj(void)
{
  BT_GIKVISION_OBJ = fl_add_valslider(FL_HOR_SLIDER,10,380,250,20.0,"");
  fl_set_slider_step(BT_GIKVISION_OBJ,1);
  fl_set_slider_bounds(BT_GIKVISION_OBJ,0,500);
  fl_set_slider_value(BT_GIKVISION_OBJ,500);
  GIK_VIS = 500;
  fl_set_object_callback(BT_GIKVISION_OBJ,CB_update_GIKvision_obj,0);
	
}





/*******************DELETES**********************/

static void g3d_delete_find_model_q(void)
{
  if(persp_win != NULL)
    g3d_del_win(persp_win);
  fl_free_object(BT_FIND_MODEL_Q_POINT); 
  fl_free_object(BT_WATCH_OBJECT); 
  fl_free_object(BT_PSP_PARAMETERS_OBJECT);
}

static void g3d_delete_bitmap_init_obj(void)
{
  if(ACBTSET!=NULL)
    hri_bt_destroy_bitmapset(ACBTSET);
	
  fl_free_object(BT_INIT_OBJ); 
}

static void g3d_delete_distance_active_obj(void)
{
  fl_free_object(BT_DISTANCE_OBJ); 
}

static void g3d_delete_visibility_active_obj(void)
{
  fl_free_object(BT_VISIBILITY_OBJ); 
}

static void g3d_delete_hidzones_active_obj(void)
{
  fl_free_object(BT_HIDZONES_OBJ); 
}

static void g3d_delete_btcombined_active_obj(void)
{
  fl_free_object(BT_COMBI_OBJ); 
}

static void g3d_delete_btpath_active_obj(void)
{
  fl_free_object(BT_PATH_OBJ); 
}

static void g3d_delete_btobstacles_active_obj(void)
{
  fl_free_object(BT_OBSTACLES_OBJ); 
}

static void g3d_delete_calculate_path_obj(void)
{
  fl_free_object(BT_CALPATH_OBJ); 
}

static void g3d_delete_update_choice_obj(void)
{
  fl_free_object(UPDATEGROUP);
  fl_free_object(BT_CHOICE1_OBJ);
  fl_free_object(BT_CHOICE2_OBJ);
  fl_free_object(BT_CHOICE3_OBJ);
}

static void g3d_delete_combine_choice_obj(void)
{
  fl_free_object(COMBINEGROUP);
  fl_free_object(BT_COMBINE1_OBJ);
  fl_free_object(BT_COMBINE2_OBJ);
}

static void g3d_delete_param1_obj(void)
{
  fl_free_object(BT_PARAM1_OBJ); 
}

static void g3d_delete_param2_obj(void)
{
  fl_free_object(BT_PARAM2_OBJ); 
}

static void g3d_delete_param3_obj(void)
{
  fl_free_object(BT_PARAM3_OBJ); 
}

static void g3d_delete_param4_obj(void)
{
  fl_free_object(BT_PARAM4_OBJ); 
}

static void g3d_delete_show_bitmaps_obj(void)
{
  fl_free_object(BT_SHOW_BITMAPS_OBJ); 
}

static void g3d_delete_save_3dbitmaps_obj(void)
{
  gnuplot_close(plot);
  fl_free_object(BT_SAVE_3DBITMAPS_OBJ); 
}

static void g3d_delete_placement_obj(void)
{
  fl_free_object(PLACEMENTGROUP); 
  fl_free_object(BT_PLCMT_OBJ1); 
  fl_free_object(BT_PLCMT_OBJ2);
  fl_free_object(BT_PLCMT_OBJ3); 
  fl_free_object(BT_PLCMT_OBJ4);
  fl_free_object(BT_PLCMT_OBJ5);
  fl_free_object(BT_PLCMT_OBJ6); 
  fl_free_object(BT_PLCMT_OBJ7);
  fl_free_object(BT_PLCMT_OBJ8);
}

static void g3d_delete_plcmt_type_obj(void)
{
  fl_free_object(PLCMTTYPEGROUP); 
  fl_free_object(BT_PLCMT_TYPE_OBJ1); 
  fl_free_object(BT_PLCMT_TYPE_OBJ2); 
  fl_free_object(BT_PLCMT_TYPE_OBJ3); 
}

static void g3d_delete_select_human_obj(void)
{
  fl_free_object(ACTUAL_HUMAN); 
}

static void g3d_delete_select_state_obj(void)
{
  fl_free_object(HUMAN_STATE); 
}

static void g3d_delete_select_exists_obj(void)
{
  fl_free_object(HUMAN_EXISTS); 
}

/********** GIK ************/

static void g3d_delete_run_GIKstep_obj(void)
{
  fl_free_object(BT_RUN_GIKstep_OBJ); 
}

static void g3d_delete_run_GIKstepper_obj(void)
{
  fl_free_object(BT_RUN_GIKstepper_OBJ); 
}

static void g3d_delete_interpoint_weight1_obj(void)
{
  fl_free_object(BT_IPW1_OBJ); 
}

static void g3d_delete_interpoint_distance_active_obj(void)
{
  fl_free_object(BT_INTDISTANCE_OBJ); 
}

static void g3d_delete_interpoint_weight2_obj(void)
{
  fl_free_object(BT_IPW2_OBJ); 
}

static void g3d_delete_interpoint_visibility_active_obj(void)
{
  fl_free_object(BT_INTVISIBILITY_OBJ); 
}

static void g3d_delete_interpoint_weight3_obj(void)
{
  fl_free_object(BT_IPW3_OBJ); 
}

static void g3d_delete_interpoint_humanreach_active_obj(void)
{
  fl_free_object(BT_INTHUMANR_OBJ); 
}

static void g3d_delete_select_bitmapset_obj(void)
{
  fl_free_object(SELECT_BTSET); 
}

static void g3d_delete_drawn_obj(void)
{
  fl_free_object(BT_DRAWNO_OBJ); 
}

static void g3d_delete_run_GIK_obj(void)
{
  fl_free_object(BT_RUN_GIK_OBJ); 
}

static void g3d_delete_gik_direct_obj(void)
{
  fl_free_object(GIK_DIRECT_OBJ); 
}

static void g3d_delete_GIKstep_obj(void)
{
  fl_free_object(BT_GIKSTEP_OBJ); 
}

static void g3d_delete_GIKforce_obj(void)
{
  fl_free_object(BT_GIKFORCE_OBJ); 
}

static void g3d_delete_GIKvision_obj(void)
{
  fl_free_object(BT_GIKVISION_OBJ); 
}

static void g3d_delete_stop_obj(void)
{
  fl_free_object(STOP_OBJ);
}


/************ RRT BUTTONS *************/

static void CB_update_rrttest_obj(FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt qs, qg;
  int res;
  int *iksols=NULL, *iksolg=NULL;
	
  G3D_DRAW_GRAPH = 1;

  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
	
  /* on construit un graph ou qs et qg seront dans la meme composante connexe */
  MY_ALLOC_INFO("Avant la creation du graphe");
  res = hri_exp_rrt_path(qs,iksols,iksolg,fct_stop,fct_draw);
	
  p3d_destroy_config(robotPt, qs);
	
  g3d_draw_allwin_active();
	
  if(!res){
    printf("p3d_specific_planner : ECHEC : il n'existe pas de chemin\n");
  }
  else{
    /* on construit la trajectoire entre les points etapes */
		
    if(p3d_graph_to_traj(robotPt)) {
      g3d_add_traj("Globalsearch",p3d_get_desc_number(P3D_TRAJ));
			
      G3D_DRAW_TRAJ = 1; 
      fl_set_button(SEARCH_DRAW_OBJ,G3D_DRAW_GRAPH);
      fl_set_button(SEARCH_DRAW_OPTIM_OBJ,G3D_DRAW_TRAJ);
			
      g3d_draw_allwin_active();
    }
    else{printf("Y A UN PROBLEME....\n");g3d_draw_allwin_active();return;}
  }
	
  fl_ringbell(0);
	
  fl_set_button(BT_RRTTEST_OBJ,0);
	
}

static void g3d_create_rrttest_obj(void)
{
  BT_RRTTEST_OBJ = fl_add_button(FL_PUSH_BUTTON,10,400,80,40,"RRT TEST");
  fl_set_object_callback(BT_RRTTEST_OBJ,CB_update_rrttest_obj,0); 
}

static void CB_update_stop_obj(FL_OBJECT *ob, long arg)
{
  fl_set_button(STOP_OBJ,0);  
}

static void g3d_create_stop_obj(void)
{
  STOP_OBJ = fl_add_button(FL_PUSH_BUTTON,90,400.0,50.0,30.0,"Stop");
  fl_set_object_callback(STOP_OBJ,CB_update_stop_obj,0);
}

/*******************************************/
