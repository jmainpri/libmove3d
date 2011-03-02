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

//#define COMPILE_WITH_HTL
#ifdef COMPILE_WITH_HTL
#include "proto/FORM_Mocap_data_run_proto.h"
#endif
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
static FL_OBJECT  *HRI_TASK_PLAN_LIST_FRAME_OBJ;
static FL_OBJECT  *HRI_TASK_PLAN_LIST_GROUP_OBJ;

static FL_OBJECT  *BT_SHOW_VISIBILE_PLACE_AGENTS_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][12];
static FL_OBJECT  *BT_SHOW_REACHABLE_PLACE_AGENTS_HAND_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][5];
static FL_OBJECT  *BT_SHOW_REACHABLE_PLACE_AGENTS_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][12];
static FL_OBJECT  *BT_SHOW_MIGHTABILITY_FOR_AGENTS_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
static FL_OBJECT  *BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
static FL_OBJECT  *BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
static FL_OBJECT  *BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
static FL_OBJECT  *BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
static FL_OBJECT  *BT_SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS_OBJ[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
static FL_OBJECT  *BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ;
static FL_OBJECT  *BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ;
static FL_OBJECT  *BT_HRI_TASK_PERFORMED_FOR_OBJECT_OBJ;
static FL_OBJECT  *BT_USE_OBJECT_DIMENSION_FOR_CANDIDATE_PTS_OBJ;

 #ifdef HUMAN2_EXISTS_FOR_MA
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

static FL_OBJECT  *BT_SELECT_HRI_TASK_PLAN_ID;
static FL_OBJECT  *BT_SELECT_HRI_TASK_SUB_PLAN_ID;
static FL_OBJECT  *BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ;
static FL_OBJECT  *BT_SHOW_HRI_PLAN_TYPE_OBJ;

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

 #ifdef HUMAN2_EXISTS_FOR_MA
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

extern HRI_TASK_TYPE CURRENT_HRI_MANIPULATION_TASK;
extern int SHOW_CURRENT_TASK_CANDIDATE_POINTS;
extern char CURRENT_OBJECT_TO_MANIPULATE[50];


extern int UPDATE_MIGHTABILITY_MAP_INFO;
extern int SHOW_MIGHTABILITY_MAP_INFO;  

extern int AKP_RECORD_WINDOW_MOVEMENT;

extern candidate_poins_for_task resultant_current_candidate_point;
/*static void fct_draw(void)
{
  if(G3D_DRAW_GRAPH) 
    g3d_draw_allwin_active();
} 
*/
extern flags_show_Mightability_Maps curr_flags_show_Mightability_Maps;

extern int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

extern int CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS;

extern int NEED_TO_SHOW_MIGHTABILITY_MAPS;
extern int NEED_TO_SHOW_OBJECT_MIGHTABILITY;

extern HRI_TASK_AGENT CURRENT_TASK_PERFORMED_BY;
extern HRI_TASK_AGENT CURRENT_TASK_PERFORMED_FOR;

extern int indices_of_MA_agents[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

extern std::vector<HRI_task_node> HRI_task_list;

extern std::map<std::string, int > HRI_task_plan_DESC_ID_map;
extern std::map<int,std::string > HRI_sub_task_NAME_ID_map;
extern int CURRENT_HRI_TASK_PLAN_ID_TO_SHOW;
extern int INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW;
extern int SHOW_HRI_TASK_TRAJ_TYPE;
extern int SHOW_HRI_PLAN_TYPE;

int update_HRI_task_plan_list();

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
////   printf(" Inside JIDO_hide_obj_from_human(), HRI_AGENT for human is created, target_human name = %s\n",target_human->robotPt->name);
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
  #ifdef HUMAN2_EXISTS_FOR_MA
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
  #ifdef HUMAN2_EXISTS_FOR_MA
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
  #ifdef HUMAN2_EXISTS_FOR_MA
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
  #ifdef HUMAN2_EXISTS_FOR_MA
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

int add_agents_for_HRI_task()
{
  for(int i=0; i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK; i++)
  {
    fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,envPt_MM->robot[indices_of_MA_agents[i]]->name);
    fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ,envPt_MM->robot[indices_of_MA_agents[i]]->name);
  }
  return 1;
}

int add_objects_for_HRI_task()
{
  for(int i=0; i<envPt_MM->nr; i++)
  {
    fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_OBJECT_OBJ,envPt_MM->robot[i]->name);
  }
  return 1;
}
 
static void CB_calculate_affordance_active_obj(FL_OBJECT *ob, long arg)
{

 Create_and_init_Mightability_Maps();
 
 add_agents_for_HRI_task();
 add_objects_for_HRI_task();

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

 //// find_candidate_points_for_current_HRI_task(CURRENT_HRI_MANIPULATION_TASK, JIDO_MA, HUMAN1_MA, &resultant_current_candidate_point);
 candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
 
 find_HRI_task_candidate_points(CURRENT_HRI_MANIPULATION_TASK,CURRENT_OBJECT_TO_MANIPULATE,CURRENT_TASK_PERFORMED_BY,CURRENT_TASK_PERFORMED_FOR,curr_resultant_candidate_points);
 
  MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);

  CANDIDATE_POINTS_FOR_TASK_FOUND=1;
  
  fl_check_forms();
  g3d_draw_allwin_active();
 return;
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

 ////////////////traj_for_HRI_task res_trajs;
 ////////////////find_current_HRI_manip_task_solution(res_trajs);
 HRI_task_desc task;
 task.task_type=CURRENT_HRI_MANIPULATION_TASK;
 task.by_agent=CURRENT_TASK_PERFORMED_BY;
 task.for_agent=CURRENT_TASK_PERFORMED_FOR;
 task.for_object=CURRENT_OBJECT_TO_MANIPULATE;
 printf(" inside CB_find_current_task_solution_obj() CURRENT_OBJECT_TO_MANIPULATE= %s,  task.for_object =%s\n",CURRENT_OBJECT_TO_MANIPULATE,task.for_object.c_str());
 static int task_plan_id=0;
 validate_HRI_task(task, task_plan_id);
 printf(" After validate_HRI_task\n");
 task_plan_id++;
 update_HRI_task_plan_list();
 printf(" After update_HRI_task_plan_list \n");

 fl_check_forms();
 g3d_draw_allwin_active(); 

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
//   g3d_win *win= NULL;
// 
//   win= g3d_get_cur_win();
//   win->fct_draw2= NULL;

printf(" returning from CB_find_current_task_solution_obj()\n");
return;



}



static void CB_execute_current_task_solution_obj(FL_OBJECT *ob, long arg)
{
 ////char CURRENT_OBJECT_TO_MANIPULATE[50]="HORSE";
 /////strcpy(CURRENT_OBJECT_TO_MANIPULATE,XYZ_ENV->cur_robot->name);

 ////printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE);
////////int exec_path_configs=0;
////////show_world_state_of_entire_plan(HRI_task_list, exec_path_configs);
 show_desired_HRI_task_plan();
////show_world_state_of_entire_plan ( 1 );
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


static void CB_update_show_Mightabilities_for_agent_obj(FL_OBJECT *ob, long arg)
{
  ////printf(" %d\n",fl_get_button(ob));
  NEED_TO_SHOW_MIGHTABILITY_MAPS=0;
  NEED_TO_SHOW_OBJECT_MIGHTABILITY=0;
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    if(fl_get_button(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_OBJ[i])==TRUE)
    {
      SHOW_MIGHTABILITY_MAPS_FOR_AGENTS[i]=1;
      
      ////if(fl_get_button(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D_OBJ[i])==TRUE)
	SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D[i]=fl_get_button(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D_OBJ[i]);
      
      ////if(fl_get_button(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE_OBJ[i])==TRUE)
	SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE[i]=fl_get_button(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE_OBJ[i]);
	
	SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE[i]=fl_get_button(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE_OBJ[i]);
      
      NEED_TO_SHOW_MIGHTABILITY_MAPS=1;
    }
    else
    {
      SHOW_MIGHTABILITY_MAPS_FOR_AGENTS[i]=0;
    }
    if(fl_get_button(BT_SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS_OBJ[i])==TRUE)
    {
      SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS[i]=1;
      NEED_TO_SHOW_OBJECT_MIGHTABILITY=1;
    }
    else
    {
      SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS[i]=0;
    }
    
    if(fl_get_button(BT_SHOW_MIGHTABILITY_FOR_AGENTS_OBJ[i])==TRUE)
    {
      for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
     {
      if(fl_get_button(BT_SHOW_VISIBILE_PLACE_AGENTS_OBJ[i][j])==TRUE)
      {
	curr_flags_show_Mightability_Maps.show_visibility[i][j]=1+j%4;
      }
      else
      {
	curr_flags_show_Mightability_Maps.show_visibility[i][j]=0;
      }
     }
     
      for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
       {
	
       if(fl_get_button(BT_SHOW_REACHABLE_PLACE_AGENTS_HAND_OBJ[i][k])==TRUE)
        {
	  for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
         {
	  if(fl_get_button(BT_SHOW_REACHABLE_PLACE_AGENTS_OBJ[i][j])==TRUE)
          {
	    	curr_flags_show_Mightability_Maps.show_reachability[i][j][k]=1+j%4;

	  }
	  else
	  {
	    curr_flags_show_Mightability_Maps.show_reachability[i][j][k]=0;
	  }
	 }
        }
       }
       
    }
    else
    {
      for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
     {
     
	curr_flags_show_Mightability_Maps.show_visibility[i][j]=0;
      
     }
     
      for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
       {
	
       
	  for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
         {
	  
	    curr_flags_show_Mightability_Maps.show_reachability[i][j][k]=0;
	  
	 }
        }
       }
    }
 
  
  fl_check_forms();
 g3d_draw_allwin_active();
     
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

static void g3d_create_show_visible_place_agents_obj(void)
{
  int x_st=50;
  int y_st=50;
  int width=50;
  int height=20;
  int x_shift=50;
  int y_shift=20;
  char type_name[20];
  int label_fr_width=500;
  int label_fr_height=90;
  int x_init=50;
  
  
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     int y_init=y_st;
     x_st=x_init;
     //y_st=50;
     ////x_end=50;
     //y_end=20;
      sprintf(type_name,"%d",i);

      HRI_MANIP_TASK_FRAME_OBJ = fl_add_labelframe(FL_BORDER_FRAME,x_st-5,y_st-5,label_fr_width,label_fr_height,type_name);
      HRI_MANIP_TASK_GROUP_OBJ = fl_bgn_group();
      x_st=x_init+120;
      y_st-=15;
      BT_SHOW_MIGHTABILITY_FOR_AGENTS_OBJ[i] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width+5,height+5,"ACTIVATE");
       fl_set_call_back(BT_SHOW_MIGHTABILITY_FOR_AGENTS_OBJ[i],CB_update_show_Mightabilities_for_agent_obj,0);
       x_st+=x_shift+50;
       y_st+=5;
       BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_OBJ[i] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width+5,height+5,"MM");
       fl_set_call_back(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_OBJ[i],CB_update_show_Mightabilities_for_agent_obj,0);
       
       x_st+=x_shift;
       
       BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D_OBJ[i] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width,height,"3D");
       fl_set_call_back(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D_OBJ[i],CB_update_show_Mightabilities_for_agent_obj,0);

       x_st+=x_shift-10;
       
       BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE_OBJ[i] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width,height,"PLANES");
       fl_set_call_back(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE_OBJ[i],CB_update_show_Mightabilities_for_agent_obj,0);

       x_st+=x_shift;
       
       BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE_OBJ[i] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width,height,"TABLES");
       fl_set_call_back(BT_SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE_OBJ[i],CB_update_show_Mightabilities_for_agent_obj,0);

       x_st+=x_shift+15;
       BT_SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS_OBJ[i] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width+5,height+5,"OOM");
       fl_set_call_back(BT_SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS_OBJ[i],CB_update_show_Mightabilities_for_agent_obj,0);
       
       x_st=x_init;
       y_st=y_init;
       ////y_st+=y_shift;
      fl_add_text(FL_NORMAL_TEXT,x_st,y_st,50,20,"Visible");
      x_st=x_init;
      y_st+=y_shift;
     for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
     {
       ////itoa(j,type_name,10);
       sprintf(type_name,"%d",j);
       ////printf(" %s ",MM_CURRENT_STATE_HUM_VIS);
       BT_SHOW_VISIBILE_PLACE_AGENTS_OBJ[i][j] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width,height,type_name);
       fl_set_call_back(BT_SHOW_VISIBILE_PLACE_AGENTS_OBJ[i][j],CB_update_show_Mightabilities_for_agent_obj,0);
       
       x_st+=x_shift;
       ////x_end+=x_shift;
     }
      y_st+=y_shift;
      x_st=x_init;
      fl_add_text(FL_NORMAL_TEXT,x_st-5,y_st,150,20,"Reachability By hand -->");
      x_st=x_init+120;
      for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
       {
	 ////y_st+=y_shift;
      
	 sprintf(type_name,"%d",k);
      ////printf(" %s ",MM_CURRENT_STATE_HUM_VIS);
      ////fl_add_labelframe(FL_OBJECT
      ////fl_add_labelframe(FL_BORDER_FRAME,x_st-5,y_st-5,500,80,"By hand");
	 
       BT_SHOW_REACHABLE_PLACE_AGENTS_HAND_OBJ[i][k] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width,height,type_name);
       fl_set_call_back(BT_SHOW_REACHABLE_PLACE_AGENTS_HAND_OBJ[i][k],CB_update_show_Mightabilities_for_agent_obj,0);
       
       x_st+=x_shift;
       }
       
       y_st+=y_shift;
       x_st=x_init;
     for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
     {
      sprintf(type_name,"%d",j);
      ////printf(" %s ",MM_CURRENT_STATE_HUM_VIS);
      ////fl_add_labelframe(FL_OBJECT
      ////fl_add_labelframe(FL_BORDER_FRAME,x_st-5,y_st-5,500,80,"By hand");
	 
       BT_SHOW_REACHABLE_PLACE_AGENTS_OBJ[i][j] = fl_add_checkbutton(FL_PUSH_BUTTON,x_st,y_st,width,height,type_name);
       fl_set_call_back(BT_SHOW_REACHABLE_PLACE_AGENTS_OBJ[i][j],CB_update_show_Mightabilities_for_agent_obj,0);
       
       x_st+=x_shift;
      
     }
     ////y_end+=y_shift;
        fl_end_group();
	y_st=y_init+label_fr_height+10;

   }
   
/*
     curr_flags_show_Mightability_Maps.show_visibility[i][j]=0;
     }
     for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
     {
       for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
       {
       curr_flags_show_Mightability_Maps.show_reachability[i][j][k]=0;
       }
     }
   }
   
 BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,50,50,50,20,"Show Visible Places for Human on Plane");
	
  fl_set_call_back(BT_SHOW_2D_VISIBILE_PLACE_HUM_OBJ,CB_show_visible_place_hum_active_obj,0);
*/
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
BT_FIND_CURRENT_TASK_CANDIDATES_OBJ = fl_add_button(FL_NORMAL_BUTTON,360,450,150,20,"Find Current Task Candidates");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_FIND_CURRENT_TASK_CANDIDATES_OBJ,CB_find_current_task_candidates_obj,0);
       
}


void g3d_create_find_current_task_solution_obj(void)
{
BT_FIND_CURRENT_TASK_SOLUTION_OBJ = fl_add_button(FL_NORMAL_BUTTON,360,510,150,20,"Find Current Task Solution");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_FIND_CURRENT_TASK_SOLUTION_OBJ,CB_find_current_task_solution_obj,0);
       
}


void g3d_create_execute_current_task_solution_obj(void)
{
BT_EXECUTE_CURRENT_TASK_SOLUTION_OBJ = fl_add_button(FL_NORMAL_BUTTON,360,540,150,20,"Execute Solution");
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
BT_SHOW_CURRENT_TASK_CANDIDATES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,370,470,50,20,"Show Current Task Candidates");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_CURRENT_TASK_CANDIDATES_OBJ,CB_show_current_task_candidates_obj,0);
       
}

void g3d_create_show_weight_for_candidates_obj(void)
{
BT_SHOW_WEIGHT_FOR_CANDIDATES_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,370,480,150,20,"Show Weights for Candidates");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_WEIGHT_FOR_CANDIDATES_OBJ,CB_show_weight_for_candidates_obj,0);
       
}







#ifdef HUMAN2_EXISTS_FOR_MA
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

void get_hri_task_performed_by_agent(FL_OBJECT *obj, long arg)
{

  int val = fl_get_choice(obj);
  ////printf(" Agent by =%d\n",val);
  CURRENT_TASK_PERFORMED_BY=HRI_TASK_AGENT( val-1);
}

void g3d_create_select_agent_by_task_obj(void)
{
  BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ=fl_add_choice(FL_NORMAL_CHOICE,200,640,80,20,"Performed by");
  //for(int i=0; i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK; i++)
  //{
    ////fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,envPt_MM->robot[indices_of_MA_agents[i]]->name);
    /*
    switch(i)
    {
      case HUMAN1_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,"HUMAN1");
     break;
     
     case HUMAN2_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,"HUMAN2");
     break;
     
     case JIDO_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,"JIDO");
     break;
     
     case HRP2_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,"HRP2");
     break;
    }
    */
  //}
  fl_set_call_back(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,get_hri_task_performed_by_agent,0);
}

void get_hri_task_performed_for_agent(FL_OBJECT *obj, long arg)
{

  int val = fl_get_choice(obj);
  printf(" Agent for =%d\n",val);
  CURRENT_TASK_PERFORMED_FOR=HRI_TASK_AGENT( val-1);
}

void g3d_create_select_agent_for_task_obj(void)
{
  BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ=fl_add_choice(FL_NORMAL_CHOICE,300,640,80,20," for");
  //for(int i=0; i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK; i++)
  //{
    /////fl_addto_choice(BT_HRI_TASK_PERFORMED_BY_AGENT_OBJ,envPt_MM->robot[indices_of_MA_agents[i]]->name);
    /*
    switch(i)
    {
      case HUMAN1_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ,"HUMAN1");
     break;
     
     case HUMAN2_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ,"HUMAN2");
     break;
     
     case JIDO_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ,"JIDO");
     break;
     
     case HRP2_MA:
    fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ,"HRP2");
     break;
    }
    */
 // }
  fl_set_call_back(BT_HRI_TASK_PERFORMED_FOR_AGENT_OBJ,get_hri_task_performed_for_agent,0);
}

void get_hri_task_performed_for_object(FL_OBJECT *obj, long arg)
{

  int val = fl_get_choice(obj);
  printf(" For object =%d\n",val);
  strcpy(CURRENT_OBJECT_TO_MANIPULATE,envPt_MM->robot[val-1]->name);
  printf(" CURRENT_OBJECT_TO_MANIPULATE=%s\n",CURRENT_OBJECT_TO_MANIPULATE);
}

void g3d_create_select_object_for_task_obj(void)
{
  BT_HRI_TASK_PERFORMED_FOR_OBJECT_OBJ=fl_add_choice(FL_NORMAL_CHOICE,50,640,80,20," object");
//   for(int i=0; i<envPt_MM->nr; i++)
//   {
//     fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_OBJECT_OBJ,envPt_MM->robot[i]->name);
//   }
  
  fl_set_call_back(BT_HRI_TASK_PERFORMED_FOR_OBJECT_OBJ,get_hri_task_performed_for_object,0);
}

void get_choice_use_object_dimension_for_candidate_pts(FL_OBJECT *obj, long arg)
{
  CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS=(int)fl_get_button(BT_USE_OBJECT_DIMENSION_FOR_CANDIDATE_PTS_OBJ);

}
void g3d_create_use_object_dimension_for_candidate_pts()
{
  BT_USE_OBJECT_DIMENSION_FOR_CANDIDATE_PTS_OBJ=fl_add_checkbutton(FL_PUSH_BUTTON,250,660,80,20," Use Object Dimension");
//   for(int i=0; i<envPt_MM->nr; i++)
//   {
//     fl_addto_choice(BT_HRI_TASK_PERFORMED_FOR_OBJECT_OBJ,envPt_MM->robot[i]->name);
//   }
  
  fl_set_call_back(BT_USE_OBJECT_DIMENSION_FOR_CANDIDATE_PTS_OBJ,get_choice_use_object_dimension_for_candidate_pts,0);
}


int update_HRI_task_plan_list()
{
  
  fl_clear_choice(BT_SELECT_HRI_TASK_PLAN_ID);
  std::map<std::string,int>::iterator it;

  for ( it=HRI_task_plan_DESC_ID_map.begin() ; it != HRI_task_plan_DESC_ID_map.end(); it++ )
  {
    fl_addto_choice(BT_SELECT_HRI_TASK_PLAN_ID,it->first.c_str());
    
  }

  
  return 1;
}

int update_HRI_task_sub_plan_list(int plan_id)
{
    fl_clear_choice(BT_SELECT_HRI_TASK_SUB_PLAN_ID);
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==plan_id)
   {
    for(int j=0;j<HRI_task_list[i].traj.sub_task_traj.size();j++)
    {
    INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW=j;
    fl_addto_choice(BT_SELECT_HRI_TASK_SUB_PLAN_ID,HRI_sub_task_NAME_ID_map.find(HRI_task_list[i].traj.sub_task_traj[j].sub_task_type)->second.c_str());
    }
    
    break;
   }
  }
  

  
  return 1;
}

void get_hri_task_plan(FL_OBJECT *obj, long arg)
{

  int val = fl_get_choice(obj);
  printf(" For task plan =%d\n",val);

 const char *task_plan_desc=fl_get_choice_text(obj);
 CURRENT_HRI_TASK_PLAN_ID_TO_SHOW=HRI_task_plan_DESC_ID_map.find(task_plan_desc)->second;

  update_HRI_task_sub_plan_list(CURRENT_HRI_TASK_PLAN_ID_TO_SHOW);
  
  ////CURRENT_TASK_PERFORMED_FOR=HRI_TASK_AGENT( val-1);
}

void get_hri_task_sub_plan(FL_OBJECT *obj, long arg)
{

  int val = fl_get_choice(obj);
  INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW=val-1;
  printf(" For task sub plan =%d\n",val-1);
  ////CURRENT_TASK_PERFORMED_FOR=HRI_TASK_AGENT( val-1);
}


void CB_select_show_hri_task_traj_type_obj(FL_OBJECT *obj, long arg)
{
 SHOW_HRI_TASK_TRAJ_TYPE=(int) arg;
}

void CB_select_show_hri_plan_type_obj(FL_OBJECT *obj, long arg)
{
 ////int val = fl_get_choice(obj);
 
 SHOW_HRI_PLAN_TYPE=(int) arg;
}

static void g3d_create_hri_task_plan_list_group(void)
{
  int x=360, y=570;
  HRI_TASK_PLAN_LIST_FRAME_OBJ = fl_add_labelframe(FL_BORDER_FRAME,x,y,400,50,"Current HRI Manipulation Task");

  int x_shift=50, y_shift=10;

  ////HRI_TASK_PLAN_LIST_GROUP_OBJ = fl_bgn_group();

  BT_SELECT_HRI_TASK_PLAN_ID=fl_add_choice(FL_NORMAL_CHOICE,x+x_shift,y+y_shift,100,20,"Task Plan");
  fl_set_call_back(BT_SELECT_HRI_TASK_PLAN_ID,get_hri_task_plan,0);

  x_shift+=160;
  BT_SELECT_HRI_TASK_SUB_PLAN_ID=fl_add_choice(FL_NORMAL_CHOICE,x+x_shift,y+y_shift,100,20,"Sub Plan");
  fl_set_call_back(BT_SELECT_HRI_TASK_SUB_PLAN_ID,get_hri_task_sub_plan,0);

   fl_bgn_group();
  x_shift+=100;
  y_shift=2;
  fl_add_text(FL_NORMAL_TEXT,x+x_shift,y+y_shift,50,20," Show ");
  y_shift+=12;
  BT_SHOW_HRI_PLAN_TYPE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,x+x_shift,y+y_shift,50,20,"Final Configs");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_HRI_PLAN_TYPE_OBJ,CB_select_show_hri_plan_type_obj,0);

  
   y_shift+=12;
  BT_SHOW_HRI_PLAN_TYPE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,x+x_shift,y+y_shift,50,20,"Trajectory");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_HRI_PLAN_TYPE_OBJ,CB_select_show_hri_plan_type_obj,1);
  
  fl_end_group();
 
  fl_bgn_group();
  x_shift=1, y_shift=30;
  fl_add_text(FL_NORMAL_TEXT,x+x_shift,y+y_shift,20,20," for ");
  x_shift+=30;
  BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,x+x_shift,y+y_shift,50,20,"Complete Plan");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ,CB_select_show_hri_task_traj_type_obj,0);

  
   x_shift+=90;
  BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,x+x_shift,y+y_shift,50,20,"Selected Task");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ,CB_select_show_hri_task_traj_type_obj,1);

  
   x_shift+=90;
  BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,x+x_shift,y+y_shift,50,20,"Selected Sub Task");
	//fl_set_object_color(BT_PATH_FIND_OBJ,FL_RED,FL_COL1);
	fl_set_call_back(BT_SHOW_HRI_TASK_TRAJ_TYPE_OBJ,CB_select_show_hri_task_traj_type_obj,2);

  fl_end_group();
 
}

/************************/
/* Option form creation */
/************************/
void g3d_create_HRI_affordance_form(void)
{
  #ifdef MM_SHOW_DEBUG_MODE_BUTTONS
   ////#ifdef HUMAN2_EXISTS_FOR_MA
   /////HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,1000.0,700.0);
   /////#else
  HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,800.0,700.0);
  ///// #endif
  #else
  HRI_AFFORDANCE_FORM = fl_bgn_form(FL_UP_BOX,800.0,100.0);
  #endif

  g3d_create_show_visible_place_agents_obj();
  
   g3d_create_calculate_affordance_obj();
   g3d_create_show_object_reach_obj();
   g3d_create_show_object_visibility_obj();



  //g3d_create_find_model_q(); 
	
  //g3d_create_bitmap_init_obj();

  //////////g3d_create_hrp2_reach_target_obj();
  //////////g3d_create_show_hrp2_gik_sol_obj();
  
  #ifdef MM_SHOW_DEBUG_MODE_BUTTONS
//For 2D Affordance analysis
//  /* g3d_create_show_dir_reach_obj();
//   g3d_create_show_bending_reach_obj();
//   g3d_create_show_2D_visible_place_hum_obj();
//   g3d_create_show_turn_around_reach_place_obj();
//   
//   g3d_create_show_dir_reach_HRP2_obj();
//   g3d_create_show_2D_visible_place_HRP2_obj();
// 
// //FOR 3D Affordance analysis
//   g3d_create_show_3D_dir_reach_hum_obj();
//   g3d_create_show_3D_bending_reach_hum_obj();
//   g3d_create_show_3D_visible_place_hum_obj();
//    #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
//   g3d_create_show_2D_visible_place_standing_hum_obj();
//   g3d_create_show_3D_visible_place_standing_hum_obj();
//    #endif
//   g3d_create_show_3D_turn_around_reach_hum_obj();
// 
//   g3d_create_show_3D_dir_reach_HRP2_obj();
//   //g3d_create_show_3d_bending_reach_HRP2_obj();
//   g3d_create_show_3D_visible_place_HRP2_obj();*/
  //g3d_create_show_3d_turn_around_reach_HRP2_obj();
     
/*
//FOR 2D decision making;
   g3d_show_2D_HRP2_hum_common_reachable_obj();
   g3d_show_2D_HRP2_hum_common_visible_obj();
   
//FOR 3D decision making;
   g3d_show_3D_HRP2_hum_common_reachable_obj();
   g3d_show_3D_HRP2_hum_common_visible_obj();
*/
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
   
   g3d_create_select_agent_by_task_obj();
   g3d_create_select_agent_for_task_obj();
   g3d_create_select_object_for_task_obj();
   
   g3d_create_use_object_dimension_for_candidate_pts();

   g3d_create_Mightability_Maps_Set_operations_group();

   g3d_create_hri_task_plan_list_group();
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
  /*
    #ifdef HUMAN2_EXISTS_FOR_MA
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
    */
   #endif
  
    #ifdef USE_SYM_GEO_PLAN
    g3d_create_test_geometric_plan_obj();
    #endif

    #ifdef COMPILE_WITH_HTL
    g3d_create_show_mocap_data_run_form();
    #endif
   
  fl_end_form();

 #ifdef COMPILE_WITH_HTL
   g3d_create_mocap_data_run_form();
    #endif
  //g3d_create_psp_parameters_form();
       
}





