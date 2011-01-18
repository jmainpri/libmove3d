#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#ifdef USE_SYM_GEO_PLAN
#include "include/Geo_Sym_Sys.h"
#endif
#define LOCAL_COMPUTATION_EPSILON (1e-9)
/* #define BH  Change this definition to JIDO, HRP2 or BH if you use these robots */


//extern int CALCULATE_AFFORDANCE;
FL_FORM  *HRI_AFFORDANCE_FORM = NULL;
extern FL_FORM  *PSP_PARAMETERS_FORM;
extern double PSP_PS_TRSHLD;

static FL_OBJECT  *BT_SHOW_OBJECT_REACHABILITY_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_OBJECT_VISIBILITY_OBJ; //AKP

extern int SHOW_MM_BASED_OBJECT_REACHABLE;
extern int SHOW_MM_BASED_OBJECT_VISIBLE;


static FL_OBJECT  *BT_SHOW_DIRECT_REACHABILITY_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_BENDING_REACHABILITY_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_TURNING_AROUND_REACHABLE_OBJ; //AKP
static FL_OBJECT  *BT_CALCULATE_AFFORDANCE_OBJ;//AKP
static FL_OBJECT  *BT_HRP2_REACH_TARGET_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_HRP2_GIK_SOL_OBJ; //AKP
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_TURN_AROUND_REACH_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_DIRECT_REACH_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_2D_DIRECT_REACH_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_VISIBILE_PLACE_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_HRP2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_HRP2_HUM_COMMON_VISIBLE_OBJ;
static FL_OBJECT  *BT_SHOW_3D_HRP2_HUM_COMMON_REACH_OBJ;
static FL_OBJECT  *BT_SHOW_2D_HRP2_HUM_COMMON_VISIBLE_OBJ;
static FL_OBJECT  *BT_SHOW_2D_HRP2_HUM_COMMON_REACH_OBJ;
static FL_OBJECT  *BT_HRP2_PUT_OBJECT_OBJ;

static FL_OBJECT  *BT_SHOW_3D_DIRECT_REACH_HUM_OBJ;
static FL_OBJECT  *BT_SHOW_3D_BENDING_REACH_HUM_OBJ;

static FL_OBJECT  *BT_SHOW_OBSTACLE_CELLS_OBJ;
static FL_OBJECT  *BT_CREATE_HRP2_ROBOT_OBJ;
static FL_OBJECT  *BT_UPDATE_HRP2_STATE_OBJ;
static FL_OBJECT  *BT_RECORD_WINDOW_MOVEMNT_OBJ;
  

static FL_OBJECT  *BT_SHOW_CURRENT_TASK_CANDIDATES_OBJ;
static FL_OBJECT  *BT_SHOW_SHOW_OBJ_CANDIDATES_OBJ;
static FL_OBJECT  *BT_SHOW_HIDE_OBJ_CANDIDATES_OBJ;
static FL_OBJECT  *BT_FIND_CURRENT_TASK_CANDIDATES_OBJ;
static FL_OBJECT  *BT_FIND_CURRENT_TASK_SOLUTION_OBJ;
static FL_OBJECT  *BT_EXECUTE_CURRENT_TASK_SOLUTION_OBJ;
static FL_OBJECT  *BT_SHOW_WEIGHT_FOR_CANDIDATES_OBJ;
static FL_OBJECT  *BT_TEST_GEOMETRIC_PLAN_OBJ;
static FL_OBJECT  *BT_SHOW_HUMAN_PERSPECTIVE_OBJ;
static FL_OBJECT  *BT_MAKE_OBJECT_ACCESSIBLE_OBJ;
static FL_OBJECT  *BT_SHOW_OBJECT_OBJ;
static FL_OBJECT  *BT_GIVE_OBJECT_OBJ;
static FL_OBJECT  *BT_HIDE_OBJECT_OBJ;
static FL_OBJECT  *BT_PUT_AWAY_OBJECT_OBJ;
static FL_OBJECT  *BT_HIDE_AWAY_OBJECT_OBJ;
static FL_OBJECT  *BT_MAKE_SPACE_FREE_OF_OBJECT_OBJ;
static FL_OBJECT  *BT_PUT_INTO_OBJECT_OBJ;
static FL_OBJECT  *HRI_MANIP_TASK_FRAME_OBJ;
static FL_OBJECT  *HRI_MANIP_TASK_GROUP_OBJ;
static FL_OBJECT  *MIGHTABILITY_SET_FRAME_OBJ;
static FL_OBJECT  *MIGHTABILITY_SET_OPERATIONS_GROUP_OBJ;


 #ifdef SECOND_HUMAN_EXISTS
static FL_OBJECT  *BT_SHOW_DIRECT_REACHABILITY_HUM2_OBJ; 
static FL_OBJECT  *BT_SHOW_BENDING_REACHABILITY_HUM2_OBJ;
static FL_OBJECT  *BT_SHOW_TURNING_AROUND_REACHABLE_HUM2_OBJ; 
static FL_OBJECT  *BT_SHOW_3D_TURN_AROUND_REACH_HUM2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_DIRECT_REACH_HUM2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_BENDING_REACH_HUM2_OBJ;

static FL_OBJECT  *BT_SHOW_2D_VISIBILE_PLACE_HUM2_OBJ; 
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_HUM2_OBJ;
static FL_OBJECT  *BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM2_OBJ;
static FL_OBJECT  *BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM2_OBJ;
 #endif

static FL_OBJECT  *BT_MIGHTABILITY_SET_AND_OPERATOR_OBJ; 
static FL_OBJECT  *BT_MIGHTABILITY_SET_OR_OPERATOR_OBJ; 
static FL_OBJECT  *BT_USE_RESULTANT_MIGHTABILITY_SET_OBJ; 
static FL_OBJECT  *BT_SHOW_CURRENT_HOW_TO_PLACEMENTS_CANDIDATES_OBJ;
static FL_OBJECT  *BT_SHOW_ALL_HOW_TO_PLACEMENTS_OBJ;
static FL_OBJECT  *BT_SHOW_CURRENT_HAND_ONLY_GRASPS_OBJ;
static FL_OBJECT  *BT_SHOW_CURRENT_WHOLE_BODY_GRASPS_OBJ;
static FL_OBJECT  *BT_SHOW_CURRENT_WHOLE_BODY_COLLISION_FREE_GRASPS_OBJ;
static FL_OBJECT  *BT_SHOW_WHOLE_BODY_FINAL_PLACE_GRASPS_OBJ;

extern int CALCULATE_AFFORDANCE;
int SHOW_HRP2_GIK_SOL=0;
extern int PERSPECTIVE_WIN_ENABLED;


extern int Affordances_Found; 
extern int SHOW_2D_COMMON_REACH_HRP2_HUMAN;
extern int SHOW_3D_COMMON_REACH_HRP2_HUMAN;
extern int SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN;
extern int SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN;
extern int SHOW_HRP2_HUMAN_COMMON_REACHABLE_VISIBLE;

extern int SHOW_2D_BENDING_REACHABLE_HUM;
extern int SHOW_3D_BENDING_REACHABLE_HUM;
extern int SHOW_2D_DIRECT_REACHABLE_HUM;
extern int SHOW_3D_DIRECT_REACHABLE_HUM;
extern int SHOW_2D_DIRECT_REACHABLE_HRP2;
extern int SHOW_3D_DIRECT_REACHABLE_HRP2;
extern int SHOW_2D_VISIBLE_PLACES_FOR_HRP2;
extern int SHOW_3D_VISIBLE_PLACES_FOR_HRP2;
extern int SHOW_2D_VISIBLE_PLACE_HUM;
extern int SHOW_3D_VISIBLE_PLACE_HUM;
extern int SHOW_2D_TURNING_AROUND_REACHABLE_HUM;
extern int SHOW_3D_TURNING_AROUND_REACHABLE_HUM;
extern int SHOW_OBSTACLE_CELLS;
extern int SHOW_2D_VISIBLE_PLACE_STANDING_HUM;
extern int SHOW_3D_VISIBLE_PLACE_STANDING_HUM;
extern int SHOW_PUT_OBJ_CANDIDATES;
extern int SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS;
extern int SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS;

 #ifdef SECOND_HUMAN_EXISTS
extern int SHOW_3D_VISIBLE_PLACE_HUM2;
extern int SHOW_2D_VISIBLE_PLACE_HUM2;
extern int SHOW_2D_BENDING_REACHABLE_HUM2;
extern int SHOW_3D_BENDING_REACHABLE_HUM2;
extern int SHOW_2D_DIRECT_REACHABLE_HUM2;
extern int SHOW_3D_DIRECT_REACHABLE_HUM2;
extern int SHOW_2D_TURNING_AROUND_REACHABLE_HUM2;
extern int SHOW_3D_TURNING_AROUND_REACHABLE_HUM2;
extern int SHOW_2D_VISIBLE_PLACE_STANDING_HUM2;
extern int SHOW_3D_VISIBLE_PLACE_STANDING_HUM2;
 #endif

extern int CURRENT_SET_OPERATOR_ON_MM;
extern int USE_RESULTANT_MIGHTABILITY_SET; 
  
extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting
extern int CANDIDATE_POINTS_FOR_TASK_FOUND;
extern int HRP2_CURRENT_TASK;//1 for take object, 2 for put object, 3 for return to rest position

int SHOW_HUMAN_PERSPECTIVE=0;

extern p3d_env *envPt_MM;

extern int CURRENT_HRI_MANIPULATION_TASK;
extern int SHOW_CURRENT_TASK_CANDIDATE_POINTS;
extern char CURRENT_OBJECT_TO_MANIPULATE[50];


extern int UPDATE_MIGHTABILITY_MAP_INFO;
extern int SHOW_MIGHTABILITY_MAP_INFO;  

extern int AKP_RECORD_WINDOW_MOVEMENT;
/*static void fct_draw(void)
{
  if(G3D_DRAW_GRAPH) 
    g3d_draw_allwin_active();
} 
*/

#ifdef USE_HRP2_GIK
static void CB_create_HRP2_robot_obj(FL_OBJECT *ob, long arg)
{
 
 //////////create_HRP2_robot(HRP2_CURRENT_STATE);
 ////////////////////create_HRP2_robot_for_GIK_in_Move3d(HRP2_CURRENT_STATE);

 create_HRP2_robot_for_GIK_in_Move3d_new();
 printf(" Created HRP2 Robot for GIK in Move3D \n");
 fl_check_forms();
 g3d_draw_allwin_active();

}
#endif

static void CB_update_HRP2_state_obj(FL_OBJECT *ob, long arg)
{
 if(HRP2_CURRENT_STATE==1)//Sitting
 HRP2_CURRENT_STATE=2;//Half sitting
 else
 HRP2_CURRENT_STATE=1;


 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_obstacle_cells_obj(FL_OBJECT *ob, long arg)
{
 if(SHOW_OBSTACLE_CELLS==0)
 SHOW_OBSTACLE_CELLS=1;
 else
 SHOW_OBSTACLE_CELLS=0;

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_record_window_movement_obj(FL_OBJECT *ob, long arg)
{
 if(AKP_RECORD_WINDOW_MOVEMENT==0)
 AKP_RECORD_WINDOW_MOVEMENT=1;
 else
 AKP_RECORD_WINDOW_MOVEMENT=0;

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_show_human_perspective_obj(FL_OBJECT *ob, long arg)
{
 if(SHOW_HUMAN_PERSPECTIVE==0)
 {
  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int cur_hum_index=get_index_of_robot_by_name ( "ACHILE_HUMAN1" );
   HRI_AGENT * target_human;
   target_human = hri_create_agent(envPt_MM->robot[cur_hum_index]);
   printf(" Inside JIDO_hide_obj_from_human(), HRI_AGENT for human is created, target_human name = %s\n",target_human->robotPt->name);
   show_humans_perspective(target_human, FALSE);
 SHOW_HUMAN_PERSPECTIVE=1;
 }
 else
 {
 restore_previous_win_state();
 SHOW_HUMAN_PERSPECTIVE=0;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_bending_reach_hum_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_BENDING_REACHABLE_HUM==1)
  SHOW_2D_BENDING_REACHABLE_HUM=0;
  else
  SHOW_2D_BENDING_REACHABLE_HUM=1;
 break;

 case 1:
  if(SHOW_3D_BENDING_REACHABLE_HUM==1)
  SHOW_3D_BENDING_REACHABLE_HUM=0;
  else
  SHOW_3D_BENDING_REACHABLE_HUM=1;
 break;
  #ifdef SECOND_HUMAN_EXISTS
 case 2:
  if(SHOW_2D_BENDING_REACHABLE_HUM2==1)
  SHOW_2D_BENDING_REACHABLE_HUM2=0;
  else
  SHOW_2D_BENDING_REACHABLE_HUM2=1;
 break;

 case 3:
  if(SHOW_3D_BENDING_REACHABLE_HUM2==1)
  SHOW_3D_BENDING_REACHABLE_HUM2=0;
  else
  SHOW_3D_BENDING_REACHABLE_HUM2=1;
 break;
 #endif

 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_dir_reach_active_obj(FL_OBJECT *ob, long arg)
{
 switch (arg)
 {
 case 0:
  if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
  SHOW_2D_DIRECT_REACHABLE_HUM=0;
  else
  SHOW_2D_DIRECT_REACHABLE_HUM=1;
 break;
   
 case 1:
  if(SHOW_3D_DIRECT_REACHABLE_HUM==1)
  SHOW_3D_DIRECT_REACHABLE_HUM=0;
  else
  SHOW_3D_DIRECT_REACHABLE_HUM=1;
 break;
  #ifdef SECOND_HUMAN_EXISTS
 case 2:
  if(SHOW_2D_DIRECT_REACHABLE_HUM2==1)
  SHOW_2D_DIRECT_REACHABLE_HUM2=0;
  else
  SHOW_2D_DIRECT_REACHABLE_HUM2=1;
 break;
   
 case 3:
  if(SHOW_3D_DIRECT_REACHABLE_HUM2==1)
  SHOW_3D_DIRECT_REACHABLE_HUM2=0;
  else
  SHOW_3D_DIRECT_REACHABLE_HUM2=1;
 break;
  #endif
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_dir_reach_HRP2_active_obj(FL_OBJECT *ob, long arg)
{
 switch (arg)
 {
 case 0:
  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
  SHOW_2D_DIRECT_REACHABLE_HRP2=0;
  else
  SHOW_2D_DIRECT_REACHABLE_HRP2=1;
 break;
   
 case 1:
  if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
  SHOW_3D_DIRECT_REACHABLE_HRP2=0;
  else
  SHOW_3D_DIRECT_REACHABLE_HRP2=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_show_visible_place_HRP2_active_obj(FL_OBJECT *ob, long arg)
{
 switch (arg)
 {
 case 0:
  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
  SHOW_2D_VISIBLE_PLACES_FOR_HRP2=0;
  else
  SHOW_2D_VISIBLE_PLACES_FOR_HRP2=1;
 break;
   
 case 1:
  if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)
  SHOW_3D_VISIBLE_PLACES_FOR_HRP2=0;
  else
  SHOW_3D_VISIBLE_PLACES_FOR_HRP2=1;
 break;
 }
 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_visible_place_hum_active_obj(FL_OBJECT *ob, long arg)
{

 switch(arg)
 {
 case 0:
  if(SHOW_2D_VISIBLE_PLACE_HUM==1)
  SHOW_2D_VISIBLE_PLACE_HUM=0;
  else
  SHOW_2D_VISIBLE_PLACE_HUM=1;
 break;
 
 case 1:
  if(SHOW_3D_VISIBLE_PLACE_HUM==1)
  SHOW_3D_VISIBLE_PLACE_HUM=0;
  else
  SHOW_3D_VISIBLE_PLACE_HUM=1;
 break;

 case 2:
  if(SHOW_2D_VISIBLE_PLACE_STANDING_HUM==1)
  SHOW_2D_VISIBLE_PLACE_STANDING_HUM=0;
  else
  SHOW_2D_VISIBLE_PLACE_STANDING_HUM=1;
 break;

 case 3:
  if(SHOW_3D_VISIBLE_PLACE_STANDING_HUM==1)
  SHOW_3D_VISIBLE_PLACE_STANDING_HUM=0;
  else
  SHOW_3D_VISIBLE_PLACE_STANDING_HUM=1;
 break;
  #ifdef SECOND_HUMAN_EXISTS
 case 4:
  if(SHOW_2D_VISIBLE_PLACE_HUM2==1)
  SHOW_2D_VISIBLE_PLACE_HUM2=0;
  else
  SHOW_2D_VISIBLE_PLACE_HUM2=1;
 break;
 
 case 5:
  if(SHOW_3D_VISIBLE_PLACE_HUM2==1)
  SHOW_3D_VISIBLE_PLACE_HUM2=0;
  else
  SHOW_3D_VISIBLE_PLACE_HUM2=1;
 break;

 case 6:
  if(SHOW_2D_VISIBLE_PLACE_STANDING_HUM2==1)
  SHOW_2D_VISIBLE_PLACE_STANDING_HUM2=0;
  else
  SHOW_2D_VISIBLE_PLACE_STANDING_HUM2=1;
 break;

 case 7:
  if(SHOW_3D_VISIBLE_PLACE_STANDING_HUM2==1)
  SHOW_3D_VISIBLE_PLACE_STANDING_HUM2=0;
  else
  SHOW_3D_VISIBLE_PLACE_STANDING_HUM2=1;
 break;
 #endif

 }
 
 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_turn_around_reach_place_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
  SHOW_2D_TURNING_AROUND_REACHABLE_HUM=0;
  else
  SHOW_2D_TURNING_AROUND_REACHABLE_HUM=1;
 break;
 case 1:
  if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM==1)
  SHOW_3D_TURNING_AROUND_REACHABLE_HUM=0;
  else
  SHOW_3D_TURNING_AROUND_REACHABLE_HUM=1;
 break;
  #ifdef SECOND_HUMAN_EXISTS
 case 2:
  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM2==1)
  SHOW_2D_TURNING_AROUND_REACHABLE_HUM2=0;
  else
  SHOW_2D_TURNING_AROUND_REACHABLE_HUM2=1;
 break;
 case 3:
  if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM2==1)
  SHOW_3D_TURNING_AROUND_REACHABLE_HUM2=0;
  else
  SHOW_3D_TURNING_AROUND_REACHABLE_HUM2=1;
 break;
  #endif
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_common_reach_HRP2_hum_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_COMMON_REACH_HRP2_HUMAN==1)
  SHOW_2D_COMMON_REACH_HRP2_HUMAN=0;
  else
  SHOW_2D_COMMON_REACH_HRP2_HUMAN=1;
 break;
 case 1:
  if(SHOW_3D_COMMON_REACH_HRP2_HUMAN==1)
  SHOW_3D_COMMON_REACH_HRP2_HUMAN=0;
  else
  SHOW_3D_COMMON_REACH_HRP2_HUMAN=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_show_common_visible_HRP2_hum_active_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
  if(SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN==1)
  SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=0;
  else
  SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=1;
 break;
 case 1:
  if(SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN==1)
  SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=0;
  else
  SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=1;
 break;
 }

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_test_geometric_plan_obj(FL_OBJECT *ob, long arg)
{
 ////creatTask();
 ////return;
 //////////test_geometric_plan_creation();
  //////////////test_geometric_plan_creation_new();
/////find_backtrack_solution_for_geo_plan(curr_geo_plan, 0, 0);

////TODO : Following declearation and function call is giving Segementation Fault, debug it
////geometric_plan curr_geo_plan2;
////find_backtrack_solution_for_geo_plan(curr_geo_plan2, 0, 0);
//////test_geometric_plan_creation_new2();
////get_list_of_occluding_objects("RED_BOTTLE");
////get_list_of_occluding_objects("ORANGE_BOTTLE");
////get_list_of_occluding_objects("ACCESSKIT");
////return;

/////////////test_geometric_plan_creation_new();
  int exec_path_configs=1;//To show the execution of entire path
  /////show_world_state_of_entire_plan(exec_path_configs);

}

static void CB_show_object_reach_active_obj(FL_OBJECT *ob, long arg)
{
  if(SHOW_MM_BASED_OBJECT_REACHABLE==1)
  SHOW_MM_BASED_OBJECT_REACHABLE=0;
  else
  SHOW_MM_BASED_OBJECT_REACHABLE=1;


 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_object_visibility_active_obj(FL_OBJECT *ob, long arg)
{
  if(SHOW_MM_BASED_OBJECT_VISIBLE==1)
  SHOW_MM_BASED_OBJECT_VISIBLE=0;
  else
  SHOW_MM_BASED_OBJECT_VISIBLE=1;


 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_select_current_task_obj(FL_OBJECT *ob, long arg)
{
 set_current_HRI_manipulation_task((int) arg);
}


static void CB_calculate_affordance_active_obj_old(FL_OBJECT *ob, long arg)
{


//// if(CALCULATE_AFFORDANCE==1)
//// {
//// CALCULATE_AFFORDANCE=0;
//// Affordances_Found=0; 
//// }
//// else
//// {
 //////////update_human_state(1);
 //////////virtually_update_human_state_new(1);
 //////find_affordance();
 //////////find_affordance_new();
 find_Mightability_Maps();
 CALCULATE_AFFORDANCE=1;
 //g3d_draw_env();
 
 ////find_affordance();
 ////find_human_affordance();
 
 g3d_draw_env();
 Affordances_Found=1; 
 ////fl_check_forms();
 ////g3d_draw_allwin_active();

   ////show_affordance();
    
//// }

 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_calculate_affordance_active_obj(FL_OBJECT *ob, long arg)
{

 Create_and_init_Mightability_Maps();

 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
}

#ifdef USE_HRP2_GIK
static void CB_show_hrp2_gik_sol_obj(FL_OBJECT *ob, long arg)
{
 if(SHOW_HRP2_GIK_SOL==0)
 {
  SHOW_HRP2_GIK_SOL=1;
  show_gik_sol();
 }
 else
 SHOW_HRP2_GIK_SOL=0;
}
#endif

#ifdef USE_HRP2_GIK
static void CB_hrp2_reach_target_obj(FL_OBJECT *ob, long arg)
{
 int hand=2; //1 for left, 2 for right
 M3D_GIK_TEST(hand);
 return;
}
#endif

#ifdef USE_SYM_GEO_PLAN
void g3d_create_test_geometric_plan_obj(void)
{
BT_TEST_GEOMETRIC_PLAN_OBJ = fl_add_button(FL_NORMAL_BUTTON,50,560,150,35,"Test Geometric Plan");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_TEST_GEOMETRIC_PLAN_OBJ,CB_test_geometric_plan_obj,0);
       
}
#endif

#ifdef USE_HRP2_GIK
static void CB_hrp2_put_object_obj(FL_OBJECT *ob, long arg)
{

/*
////********tmp testing for look at loop
p3d_vector3 point_to_look, prev_point_to_look;
int ctr=0;
 //////create_HRP2_look_at_constraint();
while(1)
 {
    point_to_look[0] = ACBTSET->object->joints[1]->abs_pos[0][3];
    point_to_look[1] = ACBTSET->object->joints[1]->abs_pos[1][3];
    point_to_look[2] = ACBTSET->object->joints[1]->abs_pos[2][3];

int use_body_part=1;//0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet
  if(ctr==0)
  {
 
int look_res=HRP2_look_at_point(point_to_look, use_body_part);
prev_point_to_look[0]=point_to_look[0];
prev_point_to_look[1]=point_to_look[1];
prev_point_to_look[2]=point_to_look[2];
execute_current_HRP2_GIK_solution(0);
ctr++;
  }
  else
  {
   if(fabs(prev_point_to_look[0]-point_to_look[0])<0.05&&fabs(prev_point_to_look[1]-point_to_look[1])<0.05&&fabs(prev_point_to_look[2]-point_to_look[2])<0.05)
   {
    
   }
   else
   {
   int look_res=HRP2_look_at_point(point_to_look, use_body_part);
   prev_point_to_look[0]=point_to_look[0];
   prev_point_to_look[1]=point_to_look[1];
   prev_point_to_look[2]=point_to_look[2];
   execute_current_HRP2_GIK_solution(0);
   
   } 
   ////ctr++;
  }
fl_check_forms();
 }

 /////delete_HRP2_look_at_constraint();
return ;
////*******END tmp testing for look at loop
*/

 ////put_object_for_human_to_take();

//////////int hand_by_reach=2;

int for_hand=2;//1 for left hand, 2 for right hand

double *hand_pos;


hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

printf(" Before calling HRP2_look_at_bottle(), hand_pos=(%lf, %lf, %lf)\n",hand_pos[0],hand_pos[1], hand_pos[2]);
HRP2_look_at_bottle();

hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
printf(" After calling HRP2_look_at_bottle(), hand_pos=(%lf, %lf, %lf)\n",hand_pos[0],hand_pos[1], hand_pos[2]);

execute_current_HRP2_GIK_solution(0);


hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
printf(" After execute_current_HRP2_GIK_solution, hand_pos=(%lf, %lf, %lf)\n",hand_pos[0],hand_pos[1], hand_pos[2]);



////return;
for_hand=2;//1 for left hand, 2 for right hand
int res=HRP2_find_collision_free_path_to_take_object_new();
////////int res=HRP2_find_collision_free_path_to_take_object();
if(res==0)
return; 
execute_current_HRP2_GIK_solution(0);


double hand_clench_val=0.4;
printf(" Before calling HRP2_grasp_object\n");
HRP2_grasp_object(for_hand,hand_clench_val);
printf(" After calling HRP2_grasp_object, now calling execute_current_HRP2_GIK_solution\n");
execute_current_HRP2_GIK_solution(0);


////////HRP2_put_object_for_human_to_take();
/////////HRP2_show_object_to_human();
/////////HRP2_hide_object_from_human();

int SHOW_TASK=0;
int HIDE_TASK=0;
int PUT_TASK=0;
//////////res=HRP2_put_object_for_human_to_take_new();
//////////res=HRP2_hide_object_from_human_new();
res=HRP2_show_object_to_human_new();
SHOW_TASK=1;

if(res==0)
return;

CANDIDATE_POINTS_FOR_TASK_FOUND=1;
execute_current_HRP2_GIK_solution(1);//1 because need to move the bottle also


if(SHOW_TASK==0)
{
hand_clench_val=0.3;
HRP2_release_object(for_hand,hand_clench_val);
execute_current_HRP2_GIK_solution(0);
}
else
{
 
}



/*
HRP2_return_hand_to_rest_position();
execute_current_HRP2_GIK_solution(0);
*/
}
#endif


static void CB_show_current_task_candidates_obj(FL_OBJECT *ob, long arg)
{
 if(SHOW_CURRENT_TASK_CANDIDATE_POINTS==0)
 SHOW_CURRENT_TASK_CANDIDATE_POINTS=1;
 else
 SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
 

 fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_show_weight_for_candidates_obj(FL_OBJECT *ob, long arg)
{
 switch(arg)
 {
 case 0:
 if(SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS==1)
  {
  SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS=0;
  }
 else
  {
 SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS=1;
  }
 break;
 case 1:
 if(SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS==1)
  {
  SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS=0;
  }
 else
  {
 SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS=1;
  }
 break;
 }
/////show_world_state_of_entire_plan(1);
 fl_check_forms();
 g3d_draw_allwin_active();
}


static void CB_find_current_task_candidates_obj(FL_OBJECT *ob, long arg)
{

 ////find_candidate_points_on_plane_to_put_obj_new();
 ////assign_weights_on_candidte_points_to_put_obj(); 
 ////CANDIDATE_POINTS_FOR_TASK_FOUND=1;

//tmp for testing
////move_object_on_a_path();
////return;

//Tmp for testing
////test_jido_grasp_traj();
////return;

/////JIDO_make_obj_accessible_to_human ( "RED_BOTTLE" );
/////JIDO_make_obj_accessible_to_human ( "HORSE" );
/////show_world_state_of_entire_plan ( 1 );
/////return;
// p3d_rob *to_place_obj=p3d_get_robot_by_name("HORSE");
// point_co_ordi at_place;
// at_place.x=5;
// at_place.y=-3.5;
// at_place.z=1.5;
//         show_all_placements_in_3D(to_place_obj, at_place);
// 
// fl_check_forms();
//  g3d_draw_allwin_active();
// return;

////JIDO_show_obj_to_human ( "YELLOW_BOTTLE" );
////JIDO_show_obj_to_human ( "HORSE" );
/////JIDO_show_obj_to_human ( "CUPHANDLE" );
////show_world_state_of_entire_plan ( 1 );
////return;

// JIDO_give_obj_to_human ( "HORSE" );
////JIDO_give_obj_to_human ( "SMALL_YELLOW_BOTTLE" );
// JIDO_give_obj_to_human ( "YELLOW_BOTTLE" );
////show_world_state_of_entire_plan ( 1 );
////return;

//////JIDO_hide_obj_from_human ( "YELLOW_BOTTLE" );
//////JIDO_hide_obj_from_human ( "HORSE" );
//////show_world_state_of_entire_plan ( 1 );

//////return;

//////JIDO_hide_away_obj_from_human ( "HORSE" );
//////show_world_state_of_entire_plan ( 1 );
//////return;

//////test_geometric_plan_creation_for_JIDO();

/////show_world_state_of_entire_plan ( 1 );

 //////return;

 //////get_set_of_points_to_put_object();

}


static void CB_find_current_task_solution_obj(FL_OBJECT *ob, long arg)
{
 ////char CURRENT_OBJECT_TO_MANIPULATE[50]="HORSE";
 /////strcpy(CURRENT_OBJECT_TO_MANIPULATE,XYZ_ENV->cur_robot->name);

 find_current_HRI_manip_task_solution();

//  printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE);
// 
//  switch(CURRENT_HRI_MANIPULATION_TASK)
//  {
//  #ifdef HRI_JIDO 
//  case MAKE_OBJECT_ACCESSIBLE:
//  JIDO_make_obj_accessible_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  case SHOW_OBJECT:
//  printf(">>> CURRENT_HRI_MANIPULATION_TASK=%d\n",CURRENT_HRI_MANIPULATION_TASK);
//  JIDO_show_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  case GIVE_OBJECT:
//  JIDO_give_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  case HIDE_OBJECT:
//  JIDO_hide_obj_from_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  #endif
//   #ifdef HRI_HRP2 
//  case MAKE_OBJECT_ACCESSIBLE:
//  HRP2_make_obj_accessible_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  case SHOW_OBJECT:
//  printf(">>> CURRENT_HRI_MANIPULATION_TASK=%d\n",CURRENT_HRI_MANIPULATION_TASK);
//  HRP2_show_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  case GIVE_OBJECT:
//  HRP2_give_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  case HIDE_OBJECT:
//  HRP2_hide_obj_from_human ( CURRENT_OBJECT_TO_MANIPULATE );
//  break;
//  #endif
//  }
// 
// UPDATE_MIGHTABILITY_MAP_INFO=1;
//   SHOW_MIGHTABILITY_MAP_INFO=1;   
////show_world_state_of_entire_plan ( 1 );

////// To disable the display of soft motion traj
  g3d_win *win= NULL;

  win= g3d_get_cur_win();
  win->fct_draw2= NULL;

return;



}



static void CB_execute_current_task_solution_obj(FL_OBJECT *ob, long arg)
{
 ////char CURRENT_OBJECT_TO_MANIPULATE[50]="HORSE";
 /////strcpy(CURRENT_OBJECT_TO_MANIPULATE,XYZ_ENV->cur_robot->name);

 printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE);

////////show_world_state_of_entire_plan ( 1 );
return;



}

static void CB_select_MM_set_operation_obj(FL_OBJECT *ob, long arg)
{

 switch(arg)
 {
 case 0:
  CURRENT_SET_OPERATOR_ON_MM=MM_SET_OPR_NONE;
 break;
 case 1:
  CURRENT_SET_OPERATOR_ON_MM=MM_SET_OPR_OR;
 break;
 case 2:
  CURRENT_SET_OPERATOR_ON_MM=MM_SET_OPR_AND;
 break;
 
  
 }
fl_check_forms();
 g3d_draw_allwin_active();
}

static void CB_use_resultant_MM_set(FL_OBJECT *ob, long arg)
{
 if(USE_RESULTANT_MIGHTABILITY_SET==0)
 USE_RESULTANT_MIGHTABILITY_SET=1;
 else
 USE_RESULTANT_MIGHTABILITY_SET=0; 

 
}






static void g3d_create_show_obstacle_cells_obj(void)
{
 BT_SHOW_OBSTACLE_CELLS_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,400,640,50,20,"Show Obstacle cells");
	
  fl_set_call_back(BT_SHOW_OBSTACLE_CELLS_OBJ,CB_show_obstacle_cells_obj,0);

}

static void g3d_record_window_movement_obj(void)
{
 BT_RECORD_WINDOW_MOVEMNT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,400,660,50,20,"Record Window Movement");
	
  fl_set_call_back(BT_RECORD_WINDOW_MOVEMNT_OBJ,CB_record_window_movement_obj,0);

}


static void g3d_create_show_2D_visible_place_hum_obj(void)
{
 BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,50,50,20,"Show Visible Places for Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ,CB_show_visible_place_hum_active_obj,0);

}

static void g3d_create_show_3D_visible_place_hum_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,70,50,20,"Show Visible Places for Human in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_HUM_OBJ,CB_show_visible_place_hum_active_obj,1);

}

static void g3d_create_show_2D_visible_place_standing_hum_obj(void)
{
 BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,260,50,50,20,"Show Visible Places for Standing Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM_OBJ,CB_show_visible_place_hum_active_obj,2);

}


static void g3d_create_show_3D_visible_place_standing_hum_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,260,70,50,20,"Show Visible Places for Standing Human in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM_OBJ,CB_show_visible_place_hum_active_obj,3);

}


static void g3d_create_show_dir_reach_obj(void)
{
 BT_SHOW_DIRECT_REACHABILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,90,50,20,"Show Human Direct Reachability on Plane");
	
  fl_set_call_back(BT_SHOW_DIRECT_REACHABILITY_OBJ,CB_show_dir_reach_active_obj,0);

}

static void g3d_create_show_3D_dir_reach_hum_obj(void)
{
 BT_SHOW_3D_DIRECT_REACH_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,110,50,20,"Show Human Direct Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_DIRECT_REACH_HUM_OBJ,CB_show_dir_reach_active_obj,1);

}


static void g3d_create_show_bending_reach_obj(void)
{
 BT_SHOW_BENDING_REACHABILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,130,50,20,"Show Human Bending Reachability on Plane");
	
  fl_set_call_back(BT_SHOW_BENDING_REACHABILITY_OBJ,CB_show_bending_reach_hum_active_obj,0);

}



static void g3d_create_show_3D_bending_reach_hum_obj(void)
{
  BT_SHOW_3D_BENDING_REACH_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,150,50,20,"Show Human Bending Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_BENDING_REACH_HUM_OBJ,CB_show_bending_reach_hum_active_obj,1);

}

static void g3d_create_show_turn_around_reach_place_obj(void)
{
 BT_SHOW_TURNING_AROUND_REACHABLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,170,50,20,"Show Human Turn Around Reachability on Plane ");
	
  fl_set_call_back(BT_SHOW_TURNING_AROUND_REACHABLE_OBJ,CB_show_turn_around_reach_place_active_obj,0);

}


static void g3d_create_show_3D_turn_around_reach_hum_obj(void)
{
  BT_SHOW_3D_TURN_AROUND_REACH_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,190,50,20,"Show Human Turn Around Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_TURN_AROUND_REACH_HUM_OBJ,CB_show_turn_around_reach_place_active_obj,1);

}


static void g3d_create_show_2D_visible_place_HRP2_obj(void)
{
 BT_SHOW_VISIBILE_PLACE_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,210,50,20,"Show Visible Places for HRP2 on planes");
	
  fl_set_call_back(BT_SHOW_VISIBILE_PLACE_HRP2_OBJ,CB_show_visible_place_HRP2_active_obj,0);

}

static void g3d_create_show_3D_visible_place_HRP2_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,230,50,20,"Show Visible Places for HRP2 in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_HRP2_OBJ,CB_show_visible_place_HRP2_active_obj,1);

} 


static void g3d_create_show_dir_reach_HRP2_obj(void)
{
 BT_SHOW_2D_DIRECT_REACH_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,250,50,20,"Show Direct Reachability of HRP2 on planes");
	
  fl_set_call_back(BT_SHOW_2D_DIRECT_REACH_HRP2_OBJ,CB_show_dir_reach_HRP2_active_obj,0);

}

static void g3d_create_show_3D_dir_reach_HRP2_obj(void)
{
 BT_SHOW_3D_DIRECT_REACH_HRP2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,270,50,20,"Show Direct Reachability of HRP2 in 3D");
	
  fl_set_call_back(BT_SHOW_3D_DIRECT_REACH_HRP2_OBJ,CB_show_dir_reach_HRP2_active_obj,1);

}

static void g3d_show_2D_HRP2_hum_common_visible_obj(void)
{
 BT_SHOW_2D_HRP2_HUM_COMMON_VISIBLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,290,50,20,"Show Common Visibility for HRP2 and Human on Plane");
	
 fl_set_call_back(BT_SHOW_2D_HRP2_HUM_COMMON_VISIBLE_OBJ,CB_show_common_visible_HRP2_hum_active_obj,0);

}

static void g3d_show_3D_HRP2_hum_common_visible_obj(void)
{
 BT_SHOW_3D_HRP2_HUM_COMMON_VISIBLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,310,50,20,"Show Common Visibility for HRP2 and Human in 3D");
	
 fl_set_call_back(BT_SHOW_3D_HRP2_HUM_COMMON_VISIBLE_OBJ,CB_show_common_visible_HRP2_hum_active_obj,1);

}

static void g3d_show_2D_HRP2_hum_common_reachable_obj(void)
{
 BT_SHOW_2D_HRP2_HUM_COMMON_REACH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,330,50,20,"Show Common Reachability for HRP2 and Human on Plane");
	
 fl_set_call_back(BT_SHOW_2D_HRP2_HUM_COMMON_REACH_OBJ,CB_show_common_reach_HRP2_hum_active_obj,0);

}

static void g3d_show_3D_HRP2_hum_common_reachable_obj(void)
{
 BT_SHOW_3D_HRP2_HUM_COMMON_REACH_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,350,50,20,"Show Common Reachability for HRP2 and Human in 3D");
	
 fl_set_call_back(BT_SHOW_3D_HRP2_HUM_COMMON_REACH_OBJ,CB_show_common_reach_HRP2_hum_active_obj,1);

}

#ifdef USE_HRP2_GIK
static void g3d_create_HRP2_robot_obj(void)
{
 BT_CREATE_HRP2_ROBOT_OBJ=fl_add_button(FL_NORMAL_BUTTON,50,380,100,30,"Create HRP2 robot");
	
 fl_set_call_back(BT_CREATE_HRP2_ROBOT_OBJ,CB_create_HRP2_robot_obj,0);
 
}
#endif
  
#ifdef USE_HRP2_GIK
static void g3d_update_HRP2_state_obj(void)
{
 BT_UPDATE_HRP2_STATE_OBJ=fl_add_checkbutton(FL_PUSH_BUTTON,200,380,50,20,"Change HRP2 state as Half Sitting");
	
 fl_set_call_back(BT_UPDATE_HRP2_STATE_OBJ,CB_update_HRP2_state_obj,0);
 
}
#endif

static void g3d_create_calculate_affordance_obj(void)
{
 BT_CALCULATE_AFFORDANCE_OBJ = fl_add_button(FL_NORMAL_BUTTON,10,10,150,30,"Calculate Mightabilities");
	
  fl_set_call_back(BT_CALCULATE_AFFORDANCE_OBJ,CB_calculate_affordance_active_obj,0);

}

#ifdef USE_HRP2_GIK
void g3d_create_hrp2_reach_target_obj(void)
{
BT_HRP2_REACH_TARGET_OBJ = fl_add_button(FL_NORMAL_BUTTON,50,450,100,35,"RUN HRP2 GIK");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HRP2_REACH_TARGET_OBJ,CB_hrp2_reach_target_obj,0);
}
#endif

#ifdef USE_HRP2_GIK
void g3d_create_show_hrp2_gik_sol_obj(void)
{
BT_SHOW_HRP2_GIK_SOL_OBJ=fl_add_button(FL_NORMAL_BUTTON,200,450,150,35,"Show HRP2 gik solution");
fl_set_call_back(BT_SHOW_HRP2_GIK_SOL_OBJ,CB_show_hrp2_gik_sol_obj,0);
}
#endif

#ifdef USE_HRP2_GIK
void g3d_create_put_object_obj(void)
{
BT_HRP2_PUT_OBJECT_OBJ = fl_add_button(FL_NORMAL_BUTTON,50,490,100,35,"Put Object");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HRP2_PUT_OBJECT_OBJ,CB_hrp2_put_object_obj,0);
       
}
#endif



static void g3d_create_show_object_reach_obj(void)
{
 BT_SHOW_OBJECT_REACHABILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,160,10,150,30,"Show Object Reachability");
	
  fl_set_call_back(BT_SHOW_OBJECT_REACHABILITY_OBJ,CB_show_object_reach_active_obj,0);

}

static void g3d_create_show_object_visibility_obj(void)
{
 BT_SHOW_OBJECT_VISIBILITY_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,310,10,150,30,"Show Object Visibility");
	
  fl_set_call_back(BT_SHOW_OBJECT_VISIBILITY_OBJ,CB_show_object_visibility_active_obj,0);

}



/*
void g3d_create_make_object_accessible_obj(void)
{
BT_MAKE_OBJECT_ACCESSIBLE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,200,440,50,20,"Make Accessible");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_MAKE_OBJECT_ACCESSIBLE_OBJ,CB_select_current_task_obj,0);
       
}

void g3d_create_show_object_obj(void)
{
BT_SHOW_OBJECT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,200,460,50,20,"Show");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_OBJECT_OBJ,CB_select_current_task_obj,1);
       
}

void g3d_create_give_object_obj(void)
{
BT_GIVE_OBJECT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,200,480,50,20,"Give");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_GIVE_OBJECT_OBJ,CB_select_current_task_obj,2);
       
}

void g3d_create_hide_object_obj(void)
{
BT_HIDE_OBJECT_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,200,500,50,20,"Hide");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HIDE_OBJECT_OBJ,CB_select_current_task_obj,3);
       
}
*/

void g3d_create_find_current_task_candidates_obj(void)
{
BT_FIND_CURRENT_TASK_CANDIDATES_OBJ = fl_add_button(FL_NORMAL_BUTTON,200,400,150,20,"Find Current Task Candidates");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_FIND_CURRENT_TASK_CANDIDATES_OBJ,CB_find_current_task_candidates_obj,0);
       
}


void g3d_create_find_current_task_solution_obj(void)
{
BT_FIND_CURRENT_TASK_SOLUTION_OBJ = fl_add_button(FL_NORMAL_BUTTON,360,385,150,20,"Find Current Task Solution");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_FIND_CURRENT_TASK_SOLUTION_OBJ,CB_find_current_task_solution_obj,0);
       
}


void g3d_create_execute_current_task_solution_obj(void)
{
BT_EXECUTE_CURRENT_TASK_SOLUTION_OBJ = fl_add_button(FL_NORMAL_BUTTON,360,415,150,20,"Execute Current Task Solution");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_EXECUTE_CURRENT_TASK_SOLUTION_OBJ,CB_execute_current_task_solution_obj,0);
       
}


static void g3d_create_hri_manipulation_task_group(void)
{
  HRI_MANIP_TASK_FRAME_OBJ = fl_add_labelframe(FL_BORDER_FRAME,190,450,150,180,"Current HRI Manipulation Task");

  HRI_MANIP_TASK_GROUP_OBJ = fl_bgn_group();

  BT_MAKE_OBJECT_ACCESSIBLE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,460,50,20,"Make Accessible");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_MAKE_OBJECT_ACCESSIBLE_OBJ,CB_select_current_task_obj,0);

  BT_SHOW_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,480,50,20,"Show");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_OBJECT_OBJ,CB_select_current_task_obj,1);

  BT_GIVE_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,500,50,20,"Give");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_GIVE_OBJECT_OBJ,CB_select_current_task_obj,2);

  BT_HIDE_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,520,50,20,"Hide");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HIDE_OBJECT_OBJ,CB_select_current_task_obj,3);
/*
   BT_PUT_AWAY_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,540,50,20,"Put Away");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_PUT_AWAY_OBJECT_OBJ,CB_select_current_task_obj,4);
       
   BT_HIDE_AWAY_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,560,50,20,"Hide Away");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_HIDE_AWAY_OBJECT_OBJ,CB_select_current_task_obj,5);
   
   BT_MAKE_SPACE_FREE_OF_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,580,50,20,"Make Space Free");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_MAKE_SPACE_FREE_OF_OBJECT_OBJ,CB_select_current_task_obj,6);

   BT_PUT_INTO_OBJECT_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,200,600,50,20,"Put Into/ Dump");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_PUT_INTO_OBJECT_OBJ,CB_select_current_task_obj,7);
*/
   
  fl_end_group();
 
}


static void g3d_create_Mightability_Maps_Set_operations_group(void)
{
  MIGHTABILITY_SET_FRAME_OBJ = fl_add_labelframe(FL_BORDER_FRAME,10,450,150,180,"Mightability Map Set Operations");

  MIGHTABILITY_SET_OPERATIONS_GROUP_OBJ = fl_bgn_group();


  BT_MIGHTABILITY_SET_AND_OPERATOR_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,15,460,50,20,"OR");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_MIGHTABILITY_SET_AND_OPERATOR_OBJ,CB_select_MM_set_operation_obj,1);

  BT_MIGHTABILITY_SET_OR_OPERATOR_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,15,480,50,20,"AND");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_MIGHTABILITY_SET_OR_OPERATOR_OBJ,CB_select_MM_set_operation_obj,2);

  BT_MIGHTABILITY_SET_OR_OPERATOR_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,15,500,50,20,"NONE");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_MIGHTABILITY_SET_OR_OPERATOR_OBJ,CB_select_MM_set_operation_obj,0);


  BT_USE_RESULTANT_MIGHTABILITY_SET_OBJ= fl_add_checkbutton(FL_PUSH_BUTTON,25,520,50,20,"Use Resultant Set");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_USE_RESULTANT_MIGHTABILITY_SET_OBJ,CB_use_resultant_MM_set,0);

  fl_end_group();
 
}

void g3d_create_show_current_task_candidates_obj(void)
{
BT_SHOW_CURRENT_TASK_CANDIDATES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,370,440,50,20,"Show Current Task Candidates");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_CURRENT_TASK_CANDIDATES_OBJ,CB_show_current_task_candidates_obj,0);
       
}

void g3d_create_show_weight_for_candidates_obj(void)
{
BT_SHOW_WEIGHT_FOR_CANDIDATES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,370,460,150,20,"Show Weights for Candidates");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_WEIGHT_FOR_CANDIDATES_OBJ,CB_show_weight_for_candidates_obj,0);
       
}







#ifdef SECOND_HUMAN_EXISTS
////////////Start For human2
int x_shift=500;
static void g3d_create_show_2D_visible_place_hum2_obj(void)
{
 BT_SHOW_2D_VISIBILE_PLACE_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,50,50,20,"Show Visible Places for Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_HUM2_OBJ,CB_show_visible_place_hum_active_obj,4);

}

static void g3d_create_show_3D_visible_place_hum2_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,70,50,20,"Show Visible Places for Human in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_HUM2_OBJ,CB_show_visible_place_hum_active_obj,5);

}

static void g3d_create_show_2D_visible_place_standing_hum2_obj(void)
{
 BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+260,50,50,20,"Show Visible Places for Standing Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_STANDING_HUM2_OBJ,CB_show_visible_place_hum_active_obj,6);

}


static void g3d_create_show_3D_visible_place_standing_hum2_obj(void)
{
 BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+260,70,50,20,"Show Visible Places for Standing Human in 3D");
	
  fl_set_call_back(BT_SHOW_3D_VISIBILE_PLACE_STANDING_HUM2_OBJ,CB_show_visible_place_hum_active_obj,7);

}


static void g3d_create_show_dir_reach_hum2_obj(void)
{
 BT_SHOW_DIRECT_REACHABILITY_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,90,50,20,"Show Human Direct Reachability on Plane");
	
  fl_set_call_back(BT_SHOW_DIRECT_REACHABILITY_HUM2_OBJ,CB_show_dir_reach_active_obj,2);

}

static void g3d_create_show_3D_dir_reach_hum2_obj(void)
{
 BT_SHOW_3D_DIRECT_REACH_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,110,50,20,"Show Human Direct Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_DIRECT_REACH_HUM2_OBJ,CB_show_dir_reach_active_obj,3);

}


static void g3d_create_show_bending_reach_hum2_obj(void)
{
 BT_SHOW_BENDING_REACHABILITY_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,130,50,20,"Show Human Bending Reachability on Plane");
	
  fl_set_call_back(BT_SHOW_BENDING_REACHABILITY_HUM2_OBJ,CB_show_bending_reach_hum_active_obj,2);

}



static void g3d_create_show_3D_bending_reach_hum2_obj(void)
{
  BT_SHOW_3D_BENDING_REACH_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,150,50,20,"Show Human Bending Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_BENDING_REACH_HUM2_OBJ,CB_show_bending_reach_hum_active_obj,3);

}

static void g3d_create_show_turn_around_reach_place_hum2_obj(void)
{
 BT_SHOW_TURNING_AROUND_REACHABLE_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,170,50,20,"Show Human Turn Around Reachability on Plane ");
	
  fl_set_call_back(BT_SHOW_TURNING_AROUND_REACHABLE_HUM2_OBJ,CB_show_turn_around_reach_place_active_obj,2);

}


static void g3d_create_show_3D_turn_around_reach_hum2_obj(void)
{
  BT_SHOW_3D_TURN_AROUND_REACH_HUM2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,x_shift+50,190,50,20,"Show Human Turn Around Reachability in 3D");
	
  fl_set_call_back(BT_SHOW_3D_TURN_AROUND_REACH_HUM2_OBJ,CB_show_turn_around_reach_place_active_obj,3);

}
/////////////End for Human2
#endif

static void g3d_create_show_human_perspective_obj(void)
{
 BT_SHOW_HUMAN_PERSPECTIVE_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,400,620,50,20,"Show Human's Perspective");
	
  fl_set_call_back(BT_SHOW_HUMAN_PERSPECTIVE_OBJ,CB_show_human_perspective_obj,0);

}

void g3d_show_HRI_affordance_form(void)
{ 
  fl_show_form(HRI_AFFORDANCE_FORM,
	       FL_PLACE_SIZE,TRUE, "HRI Affordance");
}

void g3d_hide_HRI_affordance_form(void)
{ 
  fl_hide_form(HRI_AFFORDANCE_FORM);
}

void g3d_delete_HRI_affordance_form(void)
{

  ////g3d_delete_bitmap_init_obj();
  
  ////g3d_delete_motion_capture_obj();
  
  //////g3d_delete_psp_parameters_form();
	
  fl_free_form(HRI_AFFORDANCE_FORM);
}



////*** End Creating form for Mocap data run 

/************************/
/* Option form creation */
/************************/
void g3d_create_HRI_affordance_form(void)
{
  #ifdef MM_SHOW_DEBUG_MODE_BUTTONS
   #ifdef SECOND_HUMAN_EXISTS
   HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,1000.0,700.0);
   #else
  HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,600.0,700.0);
   #endif
  #else
  HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,600.0,100.0);
  #endif

   g3d_create_calculate_affordance_obj();
   g3d_create_show_object_reach_obj();
   g3d_create_show_object_visibility_obj();



  //g3d_create_find_model_q(); 
	
  //g3d_create_bitmap_init_obj();

  //////////g3d_create_hrp2_reach_target_obj();
  //////////g3d_create_show_hrp2_gik_sol_obj();
  
  #ifdef MM_SHOW_DEBUG_MODE_BUTTONS
//For 2D Affordance analysis
  g3d_create_show_dir_reach_obj();
  g3d_create_show_bending_reach_obj();
  g3d_create_show_2D_visible_place_hum_obj();
  g3d_create_show_turn_around_reach_place_obj();
  
  g3d_create_show_dir_reach_HRP2_obj();
  g3d_create_show_2D_visible_place_HRP2_obj();

//FOR 3D Affordance analysis
  g3d_create_show_3D_dir_reach_hum_obj();
  g3d_create_show_3D_bending_reach_hum_obj();
  g3d_create_show_3D_visible_place_hum_obj();
   #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
  g3d_create_show_2D_visible_place_standing_hum_obj();
  g3d_create_show_3D_visible_place_standing_hum_obj();
   #endif
  g3d_create_show_3D_turn_around_reach_hum_obj();

  g3d_create_show_3D_dir_reach_HRP2_obj();
  //g3d_create_show_3d_bending_reach_HRP2_obj();
  g3d_create_show_3D_visible_place_HRP2_obj();
  //g3d_create_show_3d_turn_around_reach_HRP2_obj();
     

//FOR 2D decision making;
   g3d_show_2D_HRP2_hum_common_reachable_obj();
   g3d_show_2D_HRP2_hum_common_visible_obj();
   
//FOR 3D decision making;
   g3d_show_3D_HRP2_hum_common_reachable_obj();
   g3d_show_3D_HRP2_hum_common_visible_obj();

//For candidate points for a task
   g3d_create_show_current_task_candidates_obj();   
   g3d_create_find_current_task_candidates_obj(); 

   g3d_create_find_current_task_solution_obj(); 

   g3d_create_execute_current_task_solution_obj();
   g3d_create_show_weight_for_candidates_obj();
   g3d_create_show_obstacle_cells_obj();
   g3d_record_window_movement_obj();
   g3d_create_show_human_perspective_obj();

   g3d_create_hri_manipulation_task_group();

   g3d_create_Mightability_Maps_Set_operations_group();
// // // //    g3d_create_show_current_how_to_placements_candidates_obj();
// // // //    g3d_create_show_all_how_to_placements_obj();
// // // //    g3d_create_show_current_hand_only_grasps_obj();
// // // //    g3d_create_show_current_whole_body_grasps_obj();
// // // //    g3d_create_show_current_whole_body_collision_free_grasps_obj();
// // // //    g3d_create_show_whole_body_final_place_grasps_obj();
   /*g3d_create_make_object_accessible_obj(); 
   g3d_create_show_object_obj();
   g3d_create_give_object_obj();
   g3d_create_hide_object_obj();
   */
    #ifdef USE_HRP2_GIK
   g3d_create_HRP2_robot_obj();
   g3d_update_HRP2_state_obj();
   ////g3d_create_put_object_obj();
    #endif
  
    #ifdef SECOND_HUMAN_EXISTS
    //For 2D Affordance analysis
  g3d_create_show_dir_reach_hum2_obj();
  g3d_create_show_bending_reach_hum2_obj();
  g3d_create_show_2D_visible_place_hum2_obj();
  g3d_create_show_turn_around_reach_place_hum2_obj();

//FOR 3D Affordance analysis
  g3d_create_show_3D_dir_reach_hum2_obj();
  g3d_create_show_3D_bending_reach_hum2_obj();
  g3d_create_show_3D_visible_place_hum2_obj();
   #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
  g3d_create_show_2D_visible_place_standing_hum2_obj();
  g3d_create_show_3D_visible_place_standing_hum2_obj();
   #endif
  g3d_create_show_3D_turn_around_reach_hum2_obj();

    #endif
   #endif
  
    #ifdef USE_SYM_GEO_PLAN
    g3d_create_test_geometric_plan_obj();
    #endif

   
  fl_end_form();

 
  
 

   
  //g3d_create_psp_parameters_form();
       
}





