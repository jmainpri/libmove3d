//
// C++ Implementation: Mightability
//
// Description: 
//
//
// Author: Amit Kumar Pandey <akpandey@laas.fr>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Rrt-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"
#include "include/hri_bitmap_util.h"
#include "include/hri_bitmap_draw.h"
#include "include/hri_bitmap_cost.h"
#include "include/hri_bitmap_bin_heap.h"
#include "include/HRI_tasks.h"
#include "include/Mightability_Analysis.h"
#include "../graspPlanning/include/gpPlacement.h"
#include "./graspPlanning/proto/gp_geometry_proto.h"

#include <list>
#include <string>
#include <iostream>


#define COMMENT_TMP

//// This will be used for exporting the object level mightability information to the external modules/functions
object_mightabilities_info_set Mightabilities_for_obj;


extern hri_bitmapset * ACBTSET;


//AKP
int PERSPECTIVE_WIN_ENABLED=0;
int ONLINE_TRACKING_FLAG=0;
// AKP for storing the information about surfaces in the environment
struct env_surfaces curr_surfaces_in_env; 
//struct surface_grid_cell surf_grid[100][100]; 
//int grid_i_max=0;// max 1st index of surf_grid
//int grid_j_max=0;// max 2nd index of surf_grid
double surf_grid_samp_rate=0.05;//0.05;//0.1;// Sampling rate of the surface for constructing the grid
//int current_surface_index; // index i of the curr_surfaces_in_env.flat_surf[i] of env_surfaces

int CALCULATE_AFFORDANCE=0;

int SHOW_2D_COMMON_REACH_HRP2_HUMAN=0;
int SHOW_3D_COMMON_REACH_HRP2_HUMAN=0;
int SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=0;
int SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=0;
int SHOW_HRP2_HUMAN_COMMON_REACHABLE_VISIBLE=0;


int SHOW_2D_BENDING_REACHABLE_HUM=0;
int SHOW_3D_BENDING_REACHABLE_HUM=0;
int SHOW_2D_DIRECT_REACHABLE_HUM=0;
int SHOW_3D_DIRECT_REACHABLE_HUM=0;
int SHOW_2D_DIRECT_REACHABLE_HRP2=0;
int SHOW_3D_DIRECT_REACHABLE_HRP2=0;
int SHOW_2D_VISIBLE_PLACES_FOR_HRP2=0;
int SHOW_3D_VISIBLE_PLACES_FOR_HRP2=0;
int SHOW_2D_VISIBLE_PLACE_HUM=0;
int SHOW_2D_VISIBLE_PLACE_STANDING_HUM=0;
int SHOW_3D_VISIBLE_PLACE_HUM=0;
int SHOW_3D_VISIBLE_PLACE_STANDING_HUM=0;
int SHOW_2D_TURNING_AROUND_REACHABLE_HUM=0;
int SHOW_3D_TURNING_AROUND_REACHABLE_HUM=0;
int SHOW_PUT_OBJ_CANDIDATES=0;
int SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS=0;
int SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS=0;
int SHOW_MM_BASED_OBJECT_REACHABLE=0;
int SHOW_MM_BASED_OBJECT_VISIBLE=0;
#ifdef HUMAN2_EXISTS_FOR_MA
int SHOW_3D_VISIBLE_PLACE_HUM2=0;
int SHOW_2D_VISIBLE_PLACE_HUM2=0;
int SHOW_2D_BENDING_REACHABLE_HUM2=0;
int SHOW_3D_BENDING_REACHABLE_HUM2=0;
int SHOW_2D_DIRECT_REACHABLE_HUM2=0;
int SHOW_3D_DIRECT_REACHABLE_HUM2=0;
int SHOW_2D_TURNING_AROUND_REACHABLE_HUM2=0;
int SHOW_3D_TURNING_AROUND_REACHABLE_HUM2=0;
int SHOW_2D_VISIBLE_PLACE_STANDING_HUM2=0;
int SHOW_3D_VISIBLE_PLACE_STANDING_HUM2=0;
#endif
//extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting
int HRP2_CURRENT_STATE=HRP2_SITTING;//1 for sitting, 2 for half sitting
int HUMAN1_CURRENT_STATE_MM=HRI_SITTING;
#ifdef HUMAN2_EXISTS_FOR_MA
int HUMAN2_CURRENT_STATE_MM=HRI_SITTING;//HRI_STANDING;
#endif
#ifdef PR2_EXISTS_FOR_MA
int PR2_CURRENT_POSTURE=PR2_ARBITRARY_MA;
#endif

struct grid_3D grid_around_HRP2;
int HRP2_GIK_MANIP=0;// Just to set the type of the bitmap
int BT_AFFORDANCE_VISIBILITY=1;//For the bitmap which is used for calculating visibility on 3d grid

extern HRI_TASK_TYPE CURRENT_HRI_MANIPULATION_TASK;
extern candidate_poins_for_task *CANDIDATE_POINTS_FOR_CURRENT_TASK;
// extern candidate_poins_for_task candidate_points_to_put;
// extern candidate_poins_for_task candidate_points_to_show;
// extern candidate_poins_for_task candidate_points_to_hide;
// 
// extern candidate_poins_for_task candidate_points_to_give;
// extern candidate_poins_for_task candidate_points_to_putinto_by_jido;
// extern candidate_poins_for_task candidate_points_to_putinto_blue_trashbin_by_jido;
// extern candidate_poins_for_task candidate_points_to_putinto_pink_trashbin_by_jido;
// extern candidate_poins_for_task current_candidate_points_to_putinto;
// 
// extern candidate_poins_for_task candidate_points_to_displace_obj;

extern candidate_poins_for_task resultant_current_candidate_point;

int SHOW_OBSTACLE_CELLS=0;
int Affordances_Found=0;
int CANDIDATE_POINTS_FOR_TASK_FOUND=0;

point_co_ordi CURRENT_POINT_FOR_SHOWING_HOW_TO_PLACEMENT;


int no_candidate_points_to_put=0;
int grid_3d_affordance_calculated=0;

p3d_env *envPt_MM;

////////point_co_ordi FOV_end_point_vertices[1000][8];//For every set there will be 8 vertices in the order mentioned in the function gpsp_computeFrustumVertices() in the file g3d_draw_camera.c
////////int no_FOV_end_point_vertices=0;
p3d_matrix4 frustum_transformation_mat;

point_co_ordi sphere_surface_pts[25000];
int no_sphere_surface_pts=0;

/*
  int ROBOTj_RSHOULDER=19;//29;
  int ROBOTj_LSHOULDER=32;//42;
  int ROBOTj_SHOULDER=4;//For Jido
*/



extern int CANDIDATE_POINTS_FOR_TASK_FOUND;


point_co_ordi agent_eye_pos;//To store the eye position for calculating visibility, for debuging only

////int HUMAN1_HAS_MOVED=0;//For updating the mightability maps
////int HUMAN2_HAS_MOVED=0;//For updating the mightability maps
////int JIDO_HAS_MOVED=0;//For updating the mightability maps
////int HRP2_HAS_MOVED=0;//For updating the mightability maps
////int NEED_HUMAN1_VISIBILITY_UPDATE=0; //For updating the mightability maps
////int NEED_HUMAN2_VISIBILITY_UPDATE=0; //For updating the mightability maps
////int NEED_HRP2_VISIBILITY_UPDATE=0; //For updating the mightability maps
////int NEED_JIDO_VISIBILITY_UPDATE=0; //For updating the mightability maps
////int NEED_HUMAN_CURRENT_VISIBILITY_UPDATE=0;// For updating the visibility from the current head orientation
////int NEED_HUMAN1_CURRENT_VISIBILITY_UPDATE=0;
////int NEED_HUMAN2_CURRENT_VISIBILITY_UPDATE=0;
////int NEED_HRP2_CURRENT_VISIBILITY_UPDATE=0;
////int NEED_JIDO_CURRENT_VISIBILITY_UPDATE=0;
////int NEED_HUMAN1_ALL_VISIBILITY_UPDATE=0;
////int NEED_HUMAN2_ALL_VISIBILITY_UPDATE=0;
////int NEED_HRP2_ALL_VISIBILITY_UPDATE=0;
////int NEED_JIDO_ALL_VISIBILITY_UPDATE=0;
////int NEED_HUMAN1_REACHABILITY_UPDATE=0;
////int NEED_HUMAN2_REACHABILITY_UPDATE=0;
////int NEED_HRP2_REACHABILITY_UPDATE=0;
////int NEED_JIDO_REACHABILITY_UPDATE=0;
int NEED_CURRENT_VISIBILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int NEED_ALL_VISIBILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int NEED_ALL_REACHABILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int NEED_CURRENT_REACHABILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//if condition has been implemented but this flag is not bein set now, TODO: Set the flag at appropriate place

struct object_Symbolic_Mightability_Maps_Relation object_MM;

struct robots_indices rob_indx; 

int UPDATE_MIGHTABILITY_MAP_INFO=1;
int SHOW_MIGHTABILITY_MAP_INFO=1;

int show_obstacle_cells_belonging_to(int object_index);

point_co_ordi point_to_put;
int MM_RECORD_MOVIE_FRAMES=0;
int AKP_RECORD_WINDOW_MOVEMENT=0;
#if defined(WITH_XFORM)
int AKP_record_movie_frames();
#endif
point_co_ordi human1_curr_eye_pos; //To store the eye pos used in calculating the 3D grid visibility from the current head orientation

////Tmp for testing

robots_status robots_status_for_Mightability_Maps[100];

////Tmp for displaying the axis of view
point_co_ordi forehead_pos_from_eye_glass;
point_co_ordi brain_pos_from_eye_glass;


//tmp for debug
point_co_ordi hum_cam_joint_pos, hum_agent_pos;
point_co_ordi standing_human_eye_pos;

////int CURR_HUMAN_INDEX=0; 
p3d_jnt *human_head_Joint=NULL;

int SHOW_CONE=0;


int CURRENT_SET_OPERATOR_ON_MM;
int USE_RESULTANT_MIGHTABILITY_SET=0; 
int resultant_MM_after_set_operation[100][100][100];//To store 1 as a valid resultant cell after the set operation. The indices should be synchronized with the indices of the corresponding bitmap set

int SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
char CURRENT_OBJECT_TO_MANIPULATE[50]="PAPERDOG";//"CUPHANDLE";//"WOODEN_OBJECT";//"GREY_TAPE";//"YELLOW_BOTTLE";//"HORSE";//"SMALL_YELLOW_BOTTLE";//"HORSE";

//HRI_AGENT * primary_human_MM;
//HRI_AGENT * human2_MM;
//HRI_AGENT * jido_robot_MM;
//HRI_AGENT * hrp2_robot_MM;
HRI_AGENT *HRI_AGENTS_FOR_MA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
configPt HRI_AGENTS_FOR_MA_running_pos[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
configPt HRI_AGENTS_FOR_MA_actual_pos[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

//configPt JIDO_running_pos_MM;
//configPt HRP2_running_pos_MM; 
//configPt HUMAN1_running_pos_MM;
//configPt HUMAN1_actual_pos_MM;
//configPt HUMAN2_running_pos_MM;
//configPt HUMAN2_actual_pos_MM;
configPt HUMAN_curr_pos_for_state_change;

hri_human * hri_human_MM;//should only be used to switch the human between standing and sitting configs 

//tmp for testing
int show_point_of_screen();
p3d_vector4 FOV_left_up_abs, FOV_left_down_abs, FOV_right_up_abs, FOV_right_down_abs;
p3d_vector3 points_on_FOV_screen[4000]; //To store all the points on the screen
int no_points_on_FOV_screen=0;

std::list<gpTriangle> global_htris;
////std::vector<std::pair<int,std::string> > HRI_task_NAME_ID_pair;
////std::vector<std::pair<int,std::string> > HRI_sub_task_NAME_ID_pair;
extern std::map<int,std::string > HRI_task_NAME_ID_map;
extern std::map<int,std::string > HRI_sub_task_NAME_ID_map;


//TODO Put into proto file
int get_current_FOV_vertices(HRI_AGENT *agent);
int draw_current_FOV_vertices(); 
int find_Mightability_Maps();
int draw_all_current_points_on_FOV_screen();
int update_Mightability_Maps_new();
int get_horizontal_triangles(std::list<gpTriangle> &htris);
int display_horizontal_triangles(std::list<gpTriangle> htris);
int display_horizontal_triangles_samples(std::list<gpTriangle> htris);
int get_horizontal_surfaces();
int update_horizontal_surfaces();
int show_Mightability_Maps();
int show_Object_Oriented_Mightabilities();
int test_inside(p3d_rob *container, p3d_rob *object);
int voronoi(p3d_rob *container, p3d_rob *object);

// agent_state_task_constraint accepted_states_for_HRP2[MAXI_NUM_OF_HRI_TASKS]; 
// agent_state_task_constraint accepted_states_for_HUMAN1[MAXI_NUM_OF_HRI_TASKS]; 
// agent_state_task_constraint accepted_states_for_JIDO[MAXI_NUM_OF_HRI_TASKS]; 
agent_state_task_constraint accepted_states_for_HRI_task[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_OF_HRI_TASKS]; //First index will be the performer agent and the second will be the target agent, third index will store the agent for whom the current constraint is, 4th index will store the task 

agent_state_task_constraint accepted_states_for_agent_obj_MA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//It will store the states which will be used for returning object is reachable and visible for requests from external modules 

////MA_state_tansition_cost agents_state_transition_cost;
flags_show_Mightability_Maps curr_flags_show_Mightability_Maps;

////static agents_for_MA agents_for_MA_obj2;

int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int NEED_TO_SHOW_MIGHTABILITY_MAPS=0;
int NEED_TO_SHOW_OBJECT_MIGHTABILITY=0;

////int MA_agent_has_turned_head[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int indices_of_MA_agents[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
////MA_agent_head_info agents_head_info[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
HRI_TASK_AGENT CURRENT_TASK_PERFORMED_BY;
HRI_TASK_AGENT CURRENT_TASK_PERFORMED_FOR;

int CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS=0;

int CURR_VIS_STATE_INDEX_MA_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int CURR_REACH_STATE_INDEX_MA_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int PR2_Q_TORSO=12;
double PR2_Q_TORSO_HIGH_VAL=M_PI/6.0;
double PR2_Q_TORSO_LOW_VAL=0.0;

int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
int SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

double mini_visibility_threshold_for_task[MAXI_NUM_OF_HRI_TASKS];
double maxi_visibility_threshold_for_task[MAXI_NUM_OF_HRI_TASKS];


//================================

int execute_Mightability_Map_functions()
{
   
////test_inside(( p3d_rob* ) p3d_get_robot_by_name ( "TRASHBIN" ), ( p3d_rob* ) p3d_get_robot_by_name ( "GREY_TAPE" ));

  if(Affordances_Found==1)
    {
  
      // // //Tmp for testing
      // // //   show_point_of_screen();
      // // //   HRI_AGENT * target_robot;
      // // //   printf("rob_indx.JIDO_ROBOT=%d\n",rob_indx.JIDO_ROBOT);
      // // //   target_robot = hri_create_agent(envPt_MM->robot[rob_indx.JIDO_ROBOT]);
      // // //   target_robot->perspective->enable_vision_draw=TRUE;
      // // //    ////g3d_draw_rob_cone(ACBTSET->robot);
      // // //   g3d_draw_agent_fov(target_robot);
      // // //   get_current_FOV_vertices(target_robot);
      // // //   draw_current_FOV_vertices(); 
   
      //tmp for testing
      /////find_Mightability_Maps();
      ////draw_all_current_points_on_FOV_screen();
      g3d_drawDisc(standing_human_eye_pos.x,standing_human_eye_pos.y,standing_human_eye_pos.z,.1,Green,NULL); 
      ////return 1;

      ////get_horizontal_triangles(global_htris);
      ////display_horizontal_triangles(global_htris);
      ////display_horizontal_triangles_samples(global_htris);
      ////return 1;
      
      ////store_OOM_before_task();
      ////return 1;
      
      MM_RECORD_MOVIE_FRAMES=0;
      // printf(" Inside Affordances_Found==1\n");
      if(UPDATE_MIGHTABILITY_MAP_INFO==1)
	{
	  ////printf(" Inside UPDATE_MIGHTABILITY_MAP_INFO \n");
 
	  update_robots_and_objects_status();
	  update_horizontal_surfaces();
	  //////////update_Mightability_Maps();
          ////printf(" **** Calling update_Mightability_Maps_new()\n");
	  update_Mightability_Maps_new();
          /////printf(" **** After Calling update_Mightability_Maps_new()\n");

	  //printf(" **** Calling find_symbolic_Mightability_Map_new()\n");
	  ////find_symbolic_Mightability_Map();
	  find_symbolic_Mightability_Map_new();
  
	  ////printf(" **** Calling get_object_mightabilities() \n");

	  get_object_mightabilities();
	  ////printf(" **** After Calling get_object_mightabilities() \n");
          //////////show_obstacle_cells_belonging_to(get_index_of_robot_by_name( CURRENT_OBJECT_TO_MANIPULATE ));
          
	}

      if(SHOW_MIGHTABILITY_MAP_INFO==1)
	{
          //printf(" **** Calling show_object_Mightabilities() \n");

	  show_object_Mightabilities();

          //printf(" **** After show_object_Mightabilities() \n");
	  ////////////////show_symbolic_Mightability_Map_Relations();
	  if(NEED_TO_SHOW_OBJECT_MIGHTABILITY==1)
	  {
	    show_Object_Oriented_Mightabilities();
	  }
	  //printf(" **** after show_symbolic_Mightability_Map_Relations() \n");
	  ////////////show_3d_grid_Bounding_box_for_HRP2_GIK();
   
	  /////****AKP: Below is commented because it is creating Red color graphics lines which is not taken care into new hri visibility function and is increasing the %visibility calculation of the object
	  ////////////show_3D_workspace_Bounding_Box();

	  if(grid_3d_affordance_calculated==1)
	    {
      
	      if(CURRENT_SET_OPERATOR_ON_MM!=MM_SET_OPR_NONE)
		{
		  caculate_and_show_resultant_MM();
		}
	      else
		{
		  ////printf(" >>> Calling show_Mightability_Maps()\n");
		  if(NEED_TO_SHOW_MIGHTABILITY_MAPS==1)
		  {
		  show_Mightability_Maps();
		  }
		 
		  ////printf(" >>> After Calling show_Mightability_Maps()\n");
		  
		  ////show_3d_grid_affordances_new();  
		}
	    } 
    
	  /* 
	////To show the obstacle cell belonging to an object, For debuging 
	int index=get_index_of_robot_by_name("RED_BOTTLE");//("SPACENAVBOX");//("ACCESSKIT");//("YELLOW_BOTTLE");
	show_obstacle_cells_belonging_to(index);
	show_first_non_visible_cells(index);
	  */
	}

      if(SHOW_OBSTACLE_CELLS==1)
	show_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP);
      ////show_3d_grid_for_HRP2_GIK();
      //show_3d_grid_affordances();  
      ////Affordances_Found=0;
      /*
	int ROBOTj_RSHOULDER=19;
	double hum_R_shoulder_pos[3];
	hum_R_shoulder_pos[0]= ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
	hum_R_shoulder_pos[1] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
	hum_R_shoulder_pos[2] = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
	g3d_drawDisc(hum_R_shoulder_pos[0], hum_R_shoulder_pos[1], hum_R_shoulder_pos[2], 0.05, 4, NULL);
      */ 
      ////g3d_drawDisc(point_to_look[0], point_to_look[1], point_to_look[2], 0.1, 4, NULL);
#if defined(WITH_XFORM)
      if(MM_RECORD_MOVIE_FRAMES==1)
	{
	  AKP_record_movie_frames();
	}
      if(AKP_RECORD_WINDOW_MOVEMENT==1)
	{
	  AKP_record_movie_frames();
	}
#endif
    }
   
    
 

  if(CANDIDATE_POINTS_FOR_TASK_FOUND==1)
    {
      if(SHOW_CURRENT_TASK_CANDIDATE_POINTS==1)
	{
	  ////show_current_task_candidate_points(SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS, SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS);
	  show_candidate_points_for_current_task(SHOW_WEIGHT_BY_COLOR_FOR_CANDIDATE_POINTS, SHOW_WEIGHT_BY_LENGTH_FOR_CANDIDATE_POINTS);
	}

      //Tmp for test
  

    } 


  /////Tmp for testing
   /////////show_axis_of_FOV_from_mocap_eye_glass_data();

    ////////pqp_print_colliding_pair();
  
    return 1;
}

//! @ingroup graphic
//! Computes an RGB color from a hue value.
//! If the hue parameter varies from 0 to 1, the color will vary from red -> green -> cyan -> blue
//! with all intermediate hues.
//! \param x hue value (must be between 0 and 1)
//! \param color an array that will be filled with the RGB values corresponding to the given hue. The fourth element is set to 1
void AKP_rgb_from_hue2(double x, double color[4])
{
  double x1, x2, x3;

  if(x < 0.0)
    { x= 0.0; }

  if(x > 1.0)
    { x= 1.0; }

  x1= 1.0/4.0;
  x2= 2.0/4.0;
  x3= 3.0/4.0;

  color[3]= 1.0;

  if(x < x1)
    {
      color[0]= 1.0;
      color[1]= x/x1;
      color[2]= 0.0;
    }
  else if(x < x2)
    {
      color[0]= (x2-x)/(x2-x1);
      color[1]= 1.0;
      color[2]= 0.0;
    }
  else if(x < x3)
    {
      color[0]= 0.0;
      color[1]= 1.0;
      color[2]= (x-x2)/(x3-x2);
    }
  else
    {
      color[0]= 0.0;
      color[1]= (1.0-x)/(1.0-x3);
      color[2]= 1.0;
    }
}


int show_axis_of_FOV_from_mocap_eye_glass_data()
{
  g3d_drawDisc(forehead_pos_from_eye_glass.x, forehead_pos_from_eye_glass.y, forehead_pos_from_eye_glass.z+0.2, 0.1, Green, NULL);
  g3d_drawDisc(brain_pos_from_eye_glass.x, brain_pos_from_eye_glass.y, brain_pos_from_eye_glass.z+0.2, 0.1, Red, NULL);
  return 0;
}

g3d_states prev_state;
g3d_win *curr_win_to_restore;
GLint viewport[4];

int show_humans_perspective(HRI_AGENT * agent, int save)//AKP WARNING: FOV is not taken from agent->perspective->fov; It is set  in this function itself
{
  
  ////////g3d_states st;
  curr_win_to_restore= g3d_get_win_by_name((char*) "Move3D");
  double result;
  
  if( agent==NULL){
    printf("AKP WARNING: show_humans_perspective() agent is NULL.\n");
    return FALSE;
  }  
  //Change the size of the viewport if you want speed
  /*if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/3),(GLint)(viewport[3]/3));
    }*/
  
  g3d_save_win_camera(curr_win_to_restore->vs);
  g3d_save_state(curr_win_to_restore, &prev_state);
  
  // only keep what is necessary:
  curr_win_to_restore->vs.fov            = 120;//80;//agent->perspective->fov;
  curr_win_to_restore->vs.displayFrame   = FALSE;
  curr_win_to_restore->vs.displayJoints  = FALSE;
  curr_win_to_restore->vs.displayShadows = FALSE;
  curr_win_to_restore->vs.displayWalls   = FALSE;
  curr_win_to_restore->vs.displayFloor   = FALSE;
  curr_win_to_restore->vs.displayTiles   = FALSE;
  curr_win_to_restore->vs.cullingEnabled =  1;
  //do not forget to set the backgroung to black:
  ////////g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(agent->perspective->camjoint->abs_pos,curr_win_to_restore->vs);

  /////**** AKP: Uncomment below to set the projection mode as perspective projection
 g3d_set_projection_matrix(curr_win_to_restore->vs.projection_mode);
////g3d_set_projection_matrix(G3D_ORTHOGRAPHIC);

  //AKP
  /////g3d_draw_win(win);
    return 0;

    //everything is ready now.
    ////g3d_is_object_visible_from_current_viewpoint(win, object,&result,save,(char*)"");
  
    //restore viewport
    //if(!save){
    //glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
    //}
    ////g3d_load_state(win, &st);
  
    ////g3d_restore_win_camera(win->vs);
    ////g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
 
  
}




int restore_previous_win_state()
{
  ////g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  //restore viewport
  
  //// glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
  
  g3d_load_state(curr_win_to_restore, &prev_state);
  
  g3d_restore_win_camera(curr_win_to_restore->vs);
  g3d_set_projection_matrix(curr_win_to_restore->vs.projection_mode); // do this after restoring the camera fov
}

int draw_current_FOV_vertices()
{
  ////printf(" FOV_left_up_abs = (%lf, %lf, %lf)\n",FOV_left_up_abs[0],FOV_left_up_abs[1],FOV_left_up_abs[2]);
  g3d_drawDisc(FOV_left_up_abs[0], FOV_left_up_abs[1], FOV_left_up_abs[2],0.1, Red, NULL);
  g3d_drawDisc(FOV_left_down_abs[0], FOV_left_down_abs[1], FOV_left_down_abs[2],0.1, Red, NULL);
  g3d_drawDisc(FOV_right_up_abs[0], FOV_right_up_abs[1], FOV_right_up_abs[2],0.1, Red, NULL);
  g3d_drawDisc(FOV_right_down_abs[0], FOV_right_down_abs[1], FOV_right_down_abs[2],0.1, Red, NULL);

}

int draw_all_current_points_on_FOV_screen()
{
  for(int i=0;i<no_points_on_FOV_screen;i++)
    {
      g3d_drawDisc(points_on_FOV_screen[i][0],points_on_FOV_screen[i][1],points_on_FOV_screen[i][2],0.05,Red,NULL);

    }
}

int get_all_points_on_FOV_screen(HRI_AGENT *agent)
{
  
  get_current_FOV_vertices(agent); //It will store the result in FOV_left_up_abs,FOV_left_down_abs,FOV_right_up_abs,FOV_right_down_abs
  p3d_vector3 left_boundary_points[1000];
  p3d_vector3 right_boundary_points[1000];
  p3d_vector3 top_boundary_points[1000];
  p3d_vector3 bottom_boundary_points[1000];
  int no_right_boundary_points=0;
  
  no_points_on_FOV_screen=0;

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
 
  double t=0;
  double x1;
  double y1;
  double z1;
  double x2;
  double y2;
  double z2;

     
  for(;t<1;t+=interval)
    {
      
      x1=(1-t)*FOV_right_down_abs[0]+t*FOV_right_up_abs[0];
      y1=(1-t)*FOV_right_down_abs[1]+t*FOV_right_up_abs[1];
      z1=(1-t)*FOV_right_down_abs[2]+t*FOV_right_up_abs[2];

      x2=(1-t)*FOV_left_down_abs[0]+t*FOV_left_up_abs[0];
      y2=(1-t)*FOV_left_down_abs[1]+t*FOV_left_up_abs[1];
      z2=(1-t)*FOV_left_down_abs[2]+t*FOV_left_up_abs[2];

      for(double t2=0;t2<1;t2+=interval)
	{
	  points_on_FOV_screen[no_points_on_FOV_screen][0]=(1-t2)*x1+t2*x2;
	  points_on_FOV_screen[no_points_on_FOV_screen][1]=(1-t2)*y1+t2*y2;
	  points_on_FOV_screen[no_points_on_FOV_screen][2]=(1-t2)*z1+t2*z2;
	  no_points_on_FOV_screen++;
	}
   
    }

 
}

int get_current_FOV_vertices(HRI_AGENT *agent)
{

  p3d_matrix4 camera_frame;
  for(int i=0 ; i<=3 ; i++){
    for(int j=0 ; j<=3 ; j++){
      camera_frame[i][j]=agent->perspective->camjoint->abs_pos[i][j];
    }
  }

  double Hfov=DTOR(agent->perspective->fov);
  double Vfov=DTOR(agent->perspective->fov*0.75);
  double max_dist=2;



  double x_source, y_source, z_source;
  p3d_vector4 left_up, left_down, right_up, right_down;
  p3d_vector4 left_up_abs, left_down_abs, right_up_abs, right_down_abs;
  
  x_source = camera_frame[0][3];
  y_source = camera_frame[1][3];
  z_source = camera_frame[2][3];
  
  left_up[0] = max_dist;
  left_up[1] = atan(Hfov/2)*max_dist;
  left_up[2] = atan(Vfov/2)*max_dist;
  left_up[3] = 1;
  
  left_down[0] = max_dist;
  left_down[1] = atan(Hfov/2)*max_dist;
  left_down[2] = -atan(Vfov/2)*max_dist;
  left_down[3] = 1;
  
  right_up[0] = max_dist;
  right_up[1] = -atan(Hfov/2)*max_dist;
  right_up[2] = atan(Vfov/2)*max_dist;
  right_up[3] = 1;
  
  right_down[0] = max_dist;
  right_down[1] = -atan(Hfov/2)*max_dist;
  right_down[2] = -atan(Vfov/2)*max_dist;
  right_down[3] = 1;
  
  p3d_matvec4Mult(camera_frame, left_up, left_up_abs);
  p3d_matvec4Mult(camera_frame, left_down, left_down_abs);
  p3d_matvec4Mult(camera_frame, right_up, right_up_abs);
  p3d_matvec4Mult(camera_frame, right_down, right_down_abs);

  ////printf(" left_up_abs = (%lf, %lf, %lf)\n",left_up_abs[0],left_up_abs[1],left_up_abs[2]);
  for(int i=0;i<4;i++)
    {
      FOV_left_up_abs[i]=left_up_abs[i];
      FOV_left_down_abs[i]=left_down_abs[i];
      FOV_right_up_abs[i]=right_up_abs[i];
      FOV_right_down_abs[i]=right_down_abs[i];

  
    }
}

static int movie_count = 0;
/* static int movie_count_real = 0; */
static int image_rate = 1;
static int image_compress = 100;

#if defined(WITH_XFORM)
int AKP_record_movie_frames()
{
  char str[512];
  char file[64];
  int count;

  if((++movie_count)%image_rate == 0) {
    count = movie_count/image_rate;
    if(count < 10) sprintf(file,"0000%d.jpg",count);
    else if(count < 100) sprintf(file,"000%d.jpg",count);
    else sprintf(file,"00%d.jpg",count);
    sprintf(str,"/usr/bin/import -silent -window %d -quality %d %s",g3d_win_id(G3D_WIN),image_compress,file);
    /*     sprintf(str,"/usr/local/imagetools/sparc-solaris/bin/import -silent -window %d -quality %d %s",g3d_win_id(G3D_WIN),image_compress,file); */
    system(str);
    printf("**** AKP >>>> Recorded Frame %s \n",file);
  }

  return 1;
}
#endif

int move_object_on_a_path()
{
  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int obj_index=get_index_of_robot_by_name ( "PINK_TRASHBIN" );
  configPt rob_cur_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */

  p3d_get_robot_config_into(envPt_MM->robot[obj_index],&rob_cur_pos);
 
  double x=rob_cur_pos[6];
  double y=rob_cur_pos[7];
  double z=rob_cur_pos[8];
  double end_x=x+0.75;
  double end_y=y+0.75;
  double end_z=z+0.75;

  int obj_index_2=get_index_of_robot_by_name ( "BLUE_TRASHBIN" );
  configPt rob_cur_pos_2 = MY_ALLOC(double,envPt_MM->robot[obj_index_2]->nb_dof); /* Allocation of temporary robot configuration */

  p3d_get_robot_config_into(envPt_MM->robot[obj_index_2],&rob_cur_pos_2);
 
  double x2=rob_cur_pos_2[6];
  double y2=rob_cur_pos_2[7];
  double z2=rob_cur_pos_2[8];
  double end_x2=x2+0.75;
  double end_y2=y2+0.75;
  double end_z2=z2+0.75;
 
  for(;x<end_x;x+=0.01)
    {
      y+=0.01;
      rob_cur_pos[6]=x;
      rob_cur_pos[7]=y;
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[obj_index], rob_cur_pos); 
      robots_status_for_Mightability_Maps[obj_index].has_moved=1;

      y2-=0.01;
      ////x2-=0.005;
      rob_cur_pos_2[7]=y2;
      rob_cur_pos_2[6]=x2;
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[obj_index_2], rob_cur_pos_2); 
      robots_status_for_Mightability_Maps[obj_index_2].has_moved=1;
  
#if defined(WITH_XFORM)  
      fl_check_forms();
#endif 
      g3d_draw_allwin_active();
    }

  /*
    for(;x<end_x;x+=0.02)
    {
    y+=0.02;
    rob_cur_pos[6]=x;
    rob_cur_pos[7]=y;
    p3d_set_and_update_this_robot_conf(envPt_MM->robot[obj_index], rob_cur_pos); 
    fl_check_forms();
    g3d_draw_allwin_active();
    }
  */

}

int init_Mightability_Analyses_data()
{
  //for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  //{
    ///grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.for_agent[HUMAN1].visible[MM_CURRENT_STATE_HUM_VIS]==1)
		   
  //}
}

int init_HRI_task_name_ID_map()
{
 HRI_task_NAME_ID_map[MAKE_OBJECT_ACCESSIBLE]="MAKE_OBJECT_ACCESSIBLE";
 HRI_task_NAME_ID_map[SHOW_OBJECT]="SHOW_OBJECT";
 HRI_task_NAME_ID_map[GIVE_OBJECT]="GIVE_OBJECT";
 HRI_task_NAME_ID_map[HIDE_OBJECT]="HIDE_OBJECT";
 HRI_task_NAME_ID_map[PUT_AWAY_OBJECT]="PUT_AWAY_OBJECT";
 HRI_task_NAME_ID_map[HIDE_AWAY_OBJECT]="HIDE_AWAY_OBJECT";
 HRI_task_NAME_ID_map[MAKE_SPACE_FREE_OF_OBJECT_OBJ]="MAKE_SPACE_FREE_OF_OBJECT_OBJ";
 HRI_task_NAME_ID_map[PUT_INTO_OBJECT]="PUT_INTO_OBJECT";

 HRI_sub_task_NAME_ID_map[REACH_TO_TAKE]="REACH_TO_TAKE";
 HRI_sub_task_NAME_ID_map[REACH_TO_GRASP]="REACH_TO_GRASP";
 HRI_sub_task_NAME_ID_map[GRASP]="GRASP";
 HRI_sub_task_NAME_ID_map[LIFT_OBJECT]="LIFT_OBJECT";
 HRI_sub_task_NAME_ID_map[CARRY_OBJECT]="CARRY_OBJECT";
 HRI_sub_task_NAME_ID_map[PUT_DOWN_OBJECT]="PUT_DOWN_OBJECT";
 HRI_sub_task_NAME_ID_map[RELEASE_OBJECT]="RELEASE_OBJECT";
 HRI_sub_task_NAME_ID_map[RETREAT_HAND]="RETREAT_HAND";

/*
 HRI_task_NAME_ID_pair.push_back(std::make_pair(MAKE_OBJECT_ACCESSIBLE,"MAKE_OBJECT_ACCESSIBLE"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(SHOW_OBJECT,"SHOW_OBJECT"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(GIVE_OBJECT,"GIVE_OBJECT"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(HIDE_OBJECT,"HIDE_OBJECT"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(PUT_AWAY_OBJECT,"PUT_AWAY_OBJECT"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(HIDE_AWAY_OBJECT,"HIDE_AWAY_OBJECT"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(MAKE_SPACE_FREE_OF_OBJECT_OBJ,"MAKE_SPACE_FREE_OF_OBJECT_OBJ"));
 HRI_task_NAME_ID_pair.push_back(std::make_pair(PUT_INTO_OBJECT,"PUT_INTO_OBJECT"));

 HRI_sub_task_NAME_ID_pair.push_back(std::make_pair(REACH_TO_TAKE,"REACH_TO_TAKE"));
 HRI_sub_task_NAME_ID_pair.push_back(std::make_pair(REACH_TO_GRASP,"REACH_TO_GRASP"));
 HRI_sub_task_NAME_ID_pair.push_back(std::make_pair(GRASP,"GRASP"));
 HRI_sub_task_NAME_ID_pair.push_back(std::make_pair(LIFT_OBJECT,"LIFT_OBJECT"));
 HRI_sub_task_NAME_ID_pair.push_back(std::make_pair(CARRY_OBJECT,"CARRY_OBJECT"));
 HRI_sub_task_NAME_ID_pair.push_back(std::make_pair(PUT_DOWN_OBJECT,"PUT_DOWN_OBJECT"));
*/
}


int init_visibility_acceptance_for_tasks()
{
for(int i=0;i<MAXI_NUM_OF_HRI_TASKS;i++)
 {
switch(i)
  {
   case SHOW_OBJECT:
    mini_visibility_threshold_for_task[i]=0.9;
    maxi_visibility_threshold_for_task[i]=1.1;
   break;
  
   case GIVE_OBJECT:
    mini_visibility_threshold_for_task[i]=0.9;
    maxi_visibility_threshold_for_task[i]=1.1;
   break;

   case MAKE_OBJECT_ACCESSIBLE:
    mini_visibility_threshold_for_task[i]=0.9;
    maxi_visibility_threshold_for_task[i]=1.0;
   break;

   case HIDE_OBJECT:
    mini_visibility_threshold_for_task[i]=-0.1;
    maxi_visibility_threshold_for_task[i]=.05;
   break;

  }

 }
}

int init_accepted_states_for_agent_obj_MA()
{
  ////for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  ////{
    HRI_TASK_AGENT for_agent;
    for_agent=HUMAN1_MA;
    
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states=0;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states=0;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
   {
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
     
   }
   if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
   {
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   }

#ifdef HUMAN2_EXISTS_FOR_MA
   for_agent=HUMAN2_MA;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states=0;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states=0;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
   {
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
     
   }
   if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
   {
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   }
#endif

#ifdef JIDO_EXISTS_FOR_MA
for_agent=JIDO_MA;
accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states=0;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states=0;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   
#endif
   
#ifdef HRP2_EXISTS_FOR_MA
   for_agent HRP2_MA;
accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states=0;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states=0;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_reach[accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states]=MM_CURRENT_STATE_HRP2_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_reach_states++;
   
   accepted_states_for_agent_obj_MA[for_agent].accepted_visibility[accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states]=MM_CURRENT_STATE_HRP2_REACH;
   accepted_states_for_agent_obj_MA[for_agent].no_accepted_vis_states++;
   
   
#endif
////}
}

int init_accepted_states_for_tasks_HUMAN1_JIDO()
{
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
   
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
      
    break;
    
     case SHOW_OBJECT:
    
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
         accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
       
       
   break;

  case GIVE_OBJECT:
     
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
    break;

    case HIDE_OBJECT:
    
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].non_accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
     //// accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states++;
       #endif
       
       
    break;

    case HIDE_AWAY_OBJECT:
     
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].non_accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN1_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
       
    break;
    }
  }
}

#ifdef HUMAN2_EXISTS_FOR_MA
int init_accepted_states_for_tasks_HUMAN2_JIDO()
{
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
   
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
      
    break;
    
     case SHOW_OBJECT:
    
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
         accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
       
       
   break;

  case GIVE_OBJECT:
     
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
    break;

    case HIDE_OBJECT:
    
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].non_accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
     //// accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states++;
       #endif
       
       
    break;

    case HIDE_AWAY_OBJECT:
     
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].non_accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[HUMAN2_MA][JIDO_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
       
    break;
    }
  }
}

#endif

int init_accepted_states_for_tasks_JIDO_HUMAN1()
{
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
   
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
      
    break;
    
     case SHOW_OBJECT:
    
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
       
       
   break;

  case GIVE_OBJECT:
     
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
    break;

    case HIDE_OBJECT:
    
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
	
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
       
    break;

    case HIDE_AWAY_OBJECT:
     
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][HUMAN1_MA][i].no_non_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN1_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
       
    break;
    }
  }
}

#ifdef PR2_EXISTS_FOR_MA
int init_accepted_states_for_tasks_PR2_HUMAN1()
{
  int performed_by_agent=PR2_MA;
  int performed_for_agent=HUMAN1_MA;
  int constraint_for_agent=HUMAN1_MA;
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
	
      constraint_for_agent=HUMAN1_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
    
       
      
      
    break;
    
     case SHOW_OBJECT:
    
       constraint_for_agent=HUMAN1_MA;
       
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;

      
       
       
   break;

  case GIVE_OBJECT:
     
    constraint_for_agent=HUMAN1_MA;
    
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
     constraint_for_agent=PR2_MA;
     
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
     
       
      
    break;

    case HIDE_OBJECT:
    constraint_for_agent=HUMAN1_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
 
       
       
    break;

    case HIDE_AWAY_OBJECT:
     constraint_for_agent=HUMAN1_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       }	
      }
       
    constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;

       
       
    break;
    }
  }
}
#endif

#if defined(HUMAN2_EXISTS_FOR_MA) && defined(PR2_EXISTS_FOR_MA)
int init_accepted_states_for_tasks_PR2_HUMAN2()
{
  int performed_by_agent=PR2_MA;
  int performed_for_agent=HUMAN2_MA;
  int constraint_for_agent=HUMAN2_MA;
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
	
      constraint_for_agent=HUMAN2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
    
       
      
      
    break;
    
     case SHOW_OBJECT:
    
       constraint_for_agent=HUMAN2_MA;
       
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;

      
       
       
   break;

  case GIVE_OBJECT:
     
    constraint_for_agent=HUMAN2_MA;
    
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
     constraint_for_agent=PR2_MA;
     
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
     
       
      
    break;

    case HIDE_OBJECT:
    constraint_for_agent=HUMAN2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
 
       
       
    break;

    case HIDE_AWAY_OBJECT:
     constraint_for_agent=HUMAN2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       }	
      }
       
    constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;

       
       
    break;
    }
  }
}
#endif

#if defined(HUMAN1_EXISTS_FOR_MA) && defined(PR2_EXISTS_FOR_MA)

int init_accepted_states_for_tasks_HUMAN1_PR2()
{
  int performed_by_agent=HUMAN1_MA;
  int performed_for_agent=PR2_MA;
  int constraint_for_agent=HUMAN1_MA;
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
	
      constraint_for_agent=HUMAN1_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
    
       
      
      
    break;
    
     case SHOW_OBJECT:
    
       constraint_for_agent=HUMAN1_MA;
       
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
	
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;

      
       
       
   break;

  case GIVE_OBJECT:
     
    constraint_for_agent=HUMAN1_MA;
    
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
     constraint_for_agent=PR2_MA;
     
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
     
       
      
    break;

    case HIDE_OBJECT:
    constraint_for_agent=HUMAN1_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       
    break;

    case HIDE_AWAY_OBJECT:
    
 constraint_for_agent=HUMAN1_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	
	
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       
    break;
    }
  }
}
#endif

#if defined(HUMAN2_EXISTS_FOR_MA) && defined(PR2_EXISTS_FOR_MA)

int init_accepted_states_for_tasks_HUMAN2_PR2()
{
  int performed_by_agent=HUMAN2_MA;
  int performed_for_agent=PR2_MA;
  int constraint_for_agent=HUMAN2_MA;
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
	
      constraint_for_agent=HUMAN2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
    
       
      
      
    break;
    
     case SHOW_OBJECT:
    
       constraint_for_agent=HUMAN2_MA;
       
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
	
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;

      
       
       
   break;

  case GIVE_OBJECT:
     
    constraint_for_agent=HUMAN2_MA;
    
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       }	
      }
       
     constraint_for_agent=PR2_MA;
     
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
     
       
      
    break;

    case HIDE_OBJECT:
    constraint_for_agent=HUMAN2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       
    break;

    case HIDE_AWAY_OBJECT:
    
 constraint_for_agent=HUMAN2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	
	
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
	
	accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states]=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
        accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states++;
       }	
      }
       
      constraint_for_agent=PR2_MA;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_reach[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_PR2_REACH;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_reach_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].non_accepted_visibility[accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states]=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
       accepted_states_for_HRI_task[performed_by_agent][performed_for_agent][constraint_for_agent][i].no_non_accepted_vis_states++;
       
       
    break;
    }
  }
}
#endif

#if defined(HUMAN2_EXISTS_FOR_MA) && defined(JIDO_EXISTS_FOR_MA)
int init_accepted_states_for_tasks_JIDO_HUMAN2()
{
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
   
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
      
    break;
    
     case SHOW_OBJECT:
    
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
       
       
   break;

  case GIVE_OBJECT:
     
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
      
    break;

    case HIDE_OBJECT:
    
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
	
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
       
    break;

    case HIDE_AWAY_OBJECT:
     
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
      
      if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].non_accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][HUMAN2_MA][i].no_non_accepted_vis_states++;
       }	
      }
       
      #ifdef JIDO_EXISTS_FOR_MA
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_reach[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states]=MM_CURRENT_STATE_JIDO_REACH;
      accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_reach_states++;
      
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_CURRENT_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].accepted_visibility[accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states]=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
       accepted_states_for_HRI_task[JIDO_MA][HUMAN2_MA][JIDO_MA][i].no_accepted_vis_states++;
       #endif
       
       
    break;
    }
  }
}
#endif

#ifdef HRP2_EXISTS_FOR_MA
int init_accepted_states_for_tasks_HRP2_HUMAN1()
{
  HRI_TASK_AGENT agent_type=HUMAN1_MA;
  for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
  {
    switch (i)
    {
      case MAKE_OBJECT_ACCESSIBLE:
          agent_type=HUMAN1_MA;

      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }	
      }
       
     
       
      
              agent_type=HRP2;

      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HRP2_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
      
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
       if(HRP2_CURRENT_STATE==HRP2_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
        if(HRP2_CURRENT_STATE==HRP2_HALF_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
   
      
    break;
    
     case SHOW_OBJECT:
           agent_type=HUMAN1_MA;

      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }	
      }
       
      
              agent_type=HRP2;

        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HRP2_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
      
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
       if(HRP2_CURRENT_STATE==HRP2_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
        if(HRP2_CURRENT_STATE==HRP2_HALF_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
     
       
   break;

  case GIVE_OBJECT:
            agent_type=HUMAN1_MA;

      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }	
      }
       
      
       
      
              agent_type=HRP2;

        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HRP2_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
      
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
       if(HRP2_CURRENT_STATE==HRP2_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
        if(HRP2_CURRENT_STATE==HRP2_HALF_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
   
    break;

    case HIDE_OBJECT:
           agent_type=HUMAN1_MA;

      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_stats]=MM_CURRENT_STATE_HUM_REACH;
      ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_no_accepted_reach_stats++;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
	
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 ////accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
       }	
      }
       
      
    
              agent_type=HRP2;

        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HRP2_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
      
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
       if(HRP2_CURRENT_STATE==HRP2_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
        if(HRP2_CURRENT_STATE==HRP2_HALF_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
      
    break;

    case HIDE_AWAY_OBJECT:
            agent_type=HUMAN1_MA;

      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states]=MM_CURRENT_STATE_HUM_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_states++;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_CURRENT_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
      
      if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
      {
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
	accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
      }	
      else
      {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
       {
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states]=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_states++;
	 accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].non_accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states++;
       }	
      }
       
     
     
       agent_type=HRP2;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states=0;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_reach_stats=0;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_non_accepted_vis_states=0;
      
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_reach[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats]=MM_CURRENT_STATE_HRP2_REACH;
      accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_reach_stats++;
      
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_CURRENT_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
       if(HRP2_CURRENT_STATE==HRP2_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
       
        if(HRP2_CURRENT_STATE==HRP2_HALF_SITTING)
       {
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_STRAIGHT_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       
        accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].accepted_visibility[accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states]=MM_STANDING_LOOK_AROUND_HEAD_STATE_HRP2_VIS;
       accepted_states_for_HRI_task[HRP2_MA][HUMAN1_MA][agent_type][i].no_accepted_vis_states++;
       }
    
    break;
    }
  }
}
#endif

int init_accepted_states_of_agents_for_tasks()
{
#ifdef JIDO_EXISTS_FOR_MA
  init_accepted_states_for_tasks_JIDO_HUMAN1();
  init_accepted_states_for_tasks_HUMAN1_JIDO();
 #ifdef HUMAN2_EXISTS_FOR_MA
  init_accepted_states_for_tasks_JIDO_HUMAN2();
  init_accepted_states_for_tasks_HUMAN2_JIDO();
  #endif
  
#endif
  
#ifdef PR2_EXISTS_FOR_MA
  init_accepted_states_for_tasks_PR2_HUMAN1();
  init_accepted_states_for_tasks_HUMAN1_PR2();
 #ifdef HUMAN2_EXISTS_FOR_MA
  init_accepted_states_for_tasks_PR2_HUMAN2();
  init_accepted_states_for_tasks_HUMAN2_PR2();
  #endif
#endif
  
#ifdef HRP2_EXISTS_FOR_MA
  init_accepted_states_for_tasks_HRP2_HUMAN1();

#endif
  
  ////init_accepted_states_for_tasks_HRP2_HUMAN2();
}

int get_indices_for_MA_agents()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case JIDO_MA:
      indices_of_MA_agents[i]=get_index_of_robot_by_name("JIDOKUKA_ROBOT");
      break;
#ifdef HRP2_EXISTS_FOR_MA
      case HRP2_MA:
	 indices_of_MA_agents[i]=get_index_of_robot_by_name("HRP2_ROBOT");
      break;
#endif

      case HUMAN1_MA:
	 indices_of_MA_agents[i]=get_index_of_robot_by_name("ACHILE_HUMAN1");
      break;

#ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
	 indices_of_MA_agents[i]=get_index_of_robot_by_name("ACHILE_HUMAN2");
      break;
#endif
      
#ifdef PR2_EXISTS_FOR_MA
      case PR2_MA:
	 indices_of_MA_agents[i]=get_index_of_robot_by_name("PR2_ROBOT");
      break;
#endif
      
    }
    
  }
  
}

int assign_indices_of_robots()
{

  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 
  int nr = envPt_MM->nr;
  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
      if (strcasestr(envPt_MM->robot[nr_ctr]->name,"HRP2_ROBOT"))
	{
	  rob_indx.HRP2_ROBOT=nr_ctr;
	}
      else
	{
	  if (strcasestr(envPt_MM->robot[nr_ctr]->name,"JIDO_ROBOT")|| strcasestr(envPt_MM->robot[nr_ctr]->name,"JIDOKUKA_ROBOT"))
	    {
	      rob_indx.JIDO_ROBOT=nr_ctr;
	    }
	  else
	    {
	      if (strcasestr(envPt_MM->robot[nr_ctr]->name,"YELLOW"))
		{
		  rob_indx.YELLOW_BOTTLE=nr_ctr;
		}
	      else
		{
		  if (strcasestr(envPt_MM->robot[nr_ctr]->name,"HUMAN1"))
		    {
		      rob_indx.HUMAN=nr_ctr;
		    }
		  else
		    {
		      if (strcasestr(envPt_MM->robot[nr_ctr]->name,"HUMAN2"))
			{
			  rob_indx.HUMAN2=nr_ctr;
			}
		      else
			{
			  if (strcasestr(envPt_MM->robot[nr_ctr]->name,"VISBALL_MIGHTABILITY"))
			    {
			      rob_indx.VISBALL_MIGHTABILITY=nr_ctr;
			    }
			}
		    }
		}
	    }
	} 
    }      
  return 1;
}

int initialize_MM_resultant_set()
{
  int i=0;
  for(i=0;i<100;i++)
    {
      int j=0;
      for(j=0;j<100;j++)
	{
	  int k=0;
	  for(k=0;k<100;k++)
	    {
	      resultant_MM_after_set_operation[i][j][k]=1;//Initially the entire grid is the solution space
	    }
	}

    } 
}

int create_agents_for_Mightabilities()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
     HRI_AGENTS_FOR_MA[i]=NULL;
    printf(" Creating agent %d for HRI tasks \n",i);
    HRI_AGENTS_FOR_MA[i]=hri_create_agent(envPt_MM->robot[indices_of_MA_agents[i]]);
    ////printf(" HRI_AGENTS_FOR_MA[i] = %p \n",HRI_AGENTS_FOR_MA[i]);
    if(HRI_AGENTS_FOR_MA[i]==NULL)
    {
    printf(" >>>> AKP ERROR : Agent %d is defined for HRI task but can not be created, it can lead to segmentation fault for finding task solutions.\n", i);
    
    }
    HRI_AGENTS_FOR_MA[i]->perspective->enable_vision_draw=TRUE;
 
    HRI_AGENTS_FOR_MA_running_pos[i]=MY_ALLOC(double, envPt_MM->robot[indices_of_MA_agents[i]]->nb_dof);
    p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[i]],&HRI_AGENTS_FOR_MA_running_pos[i]);
    
    HRI_AGENTS_FOR_MA_actual_pos[i]=MY_ALLOC(double, envPt_MM->robot[indices_of_MA_agents[i]]->nb_dof);
    p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[i]],&HRI_AGENTS_FOR_MA_actual_pos[i]);

  }
  /*
  primary_human_MM = hri_create_agent(envPt_MM->robot[rob_indx.HUMAN]);
  primary_human_MM->perspective->enable_vision_draw=TRUE;
    
#ifdef HUMAN2_EXISTS_FOR_MA
  human2_MM = hri_create_agent(envPt_MM->robot[rob_indx.HUMAN2]);
  human2_MM->perspective->enable_vision_draw=TRUE;
#endif
  
  jido_robot_MM = hri_create_agent(envPt_MM->robot[rob_indx.JIDO_ROBOT]);
  jido_robot_MM->perspective->enable_vision_draw=TRUE;

  hrp2_robot_MM = hri_create_agent(envPt_MM->robot[rob_indx.JIDO_ROBOT]);
  hrp2_robot_MM->perspective->enable_vision_draw=TRUE;
*/
  
  /*
  JIDO_running_pos_MM = MY_ALLOC(double,envPt_MM->robot[rob_indx.JIDO_ROBOT]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.JIDO_ROBOT],&JIDO_running_pos_MM);

  HRP2_running_pos_MM = MY_ALLOC(double,envPt_MM->robot[rob_indx.HRP2_ROBOT]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HRP2_ROBOT],&HRP2_running_pos_MM);
 
  HUMAN1_running_pos_MM = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&HUMAN1_running_pos_MM);

  HUMAN1_actual_pos_MM = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&HUMAN1_actual_pos_MM);
  
  HUMAN2_running_pos_MM = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN2]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN2],&HUMAN2_running_pos_MM);

  HUMAN2_actual_pos_MM = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN2]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN2],&HUMAN2_actual_pos_MM);
  */
  
  HUMAN_curr_pos_for_state_change = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); 
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&HUMAN_curr_pos_for_state_change);
  
  hri_human_MM=hri_bt_create_human(envPt_MM->robot[rob_indx.HUMAN]);
  

}

int hri_update_agent_perspective_params(HRI_AGENT * agent)
{
  int i;
  HRI_PERSP *persp=agent->perspective;

  switch (agent->type) {
    case HRI_JIDO1:
      persp->camjoint = agent->robotPt->joints[14];
//       persp->fov = 60;
//       persp->foa = 60;
//       persp->tilt_jnt_idx = 3;
//       persp->pan_jnt_idx  = 2;
      persp->pointjoint = agent->robotPt->joints[17];
//       persp->point_tolerance = 20;
      break;
    case HRI_HRP214:
      persp->camjoint = agent->robotPt->joints[49];
//       persp->fov = 40;
//       persp->foa = 40;
//       persp->tilt_jnt_idx = 16;
//       persp->pan_jnt_idx  = 17;
      persp->pointjoint = agent->robotPt->joints[48];
//       persp->point_tolerance = 20;
      break;
    case HRI_JIDOKUKA:
      persp->camjoint = agent->robotPt->joints[30];
//       persp->fov = 63.6;
//       persp->foa = 33.4;
//       persp->tilt_jnt_idx = 3;
//       persp->pan_jnt_idx  = 2;
      persp->pointjoint = agent->robotPt->joints[33];
//       persp->point_tolerance = 20;
      break;
    case HRI_PR2:
      persp->camjoint = agent->robotPt->joints[24];
//       persp->fov = 120; //TODO: put the correct value
//       persp->foa = 60; //TODO: put the correct value
//       persp->tilt_jnt_idx = 4;
//       persp->pan_jnt_idx  = 3;
      persp->pointjoint = agent->robotPt->joints[26];
//       persp->point_tolerance = 20;
      break;
    case HRI_ICUB:
      persp->camjoint = agent->robotPt->joints[34];
//       persp->fov = 120; //TODO: put the correct value
//       persp->foa = 60; //TODO: put the correct value
//       persp->tilt_jnt_idx = 31;
//       persp->pan_jnt_idx  = 33;
      persp->pointjoint = agent->robotPt->joints[35];
//       persp->point_tolerance = 20;
      break;
    case HRI_ACHILE:
      persp->camjoint = agent->robotPt->joints[42];
//       persp->fov = 160;
//       persp->foa = 70; // TODO: By default This should be 30. Change for a particular manip while waiting a fix on vis placements
//       persp->tilt_jnt_idx = 6;
//       persp->pan_jnt_idx  = 5;
      persp->pointjoint = agent->robotPt->joints[36];
      persp->point_tolerance = 20;
    break;
    case HRI_SUPERMAN:
      persp->camjoint = agent->robotPt->joints[1]; //TODO: put the correct value
//       persp->fov = 160;
//       persp->foa = 30;
//       persp->tilt_jnt_idx = 55;
//       persp->pan_jnt_idx  = 54;
      persp->pointjoint = agent->robotPt->joints[1]; //TODO: put the correct value
//       persp->point_tolerance = 20;
      break;
    default:
      persp->fov = 0;
      persp->foa = 0;
      break;
  }
 
}

int init_flags_to_show_MM()
{
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
     {
       ////printf(" %s ",MM_CURRENT_STATE_HUM_VIS);
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
   ////curr_flags_show_Mightability_Maps.show_visibility[HUMAN1_MA][MM_CURRENT_STATE_HUM_VIS]=2;
   ////curr_flags_show_Mightability_Maps.show_reachability[HUMAN1_MA][MM_CURRENT_STATE_HUM_REACH][MA_RIGHT_HAND]=3;
}
/*
int init_agents_head_info_for_MA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case HUMAN1_MA:
      agents_head_info[i].no_joints_neck=3;
      agents_head_info[i].joint_indices[PAN]=HUMAN_J_NECK_PAN;
      agents_head_info[i].joint_indices[TILT]=HUMAN_J_NECK_TILT;
      agents_head_info[i].joint_indices[ROLL]=HUMAN_J_NECK_ROLL;
      
      agents_head_info[i].no_Qs_neck=3;
      agents_head_info[i].Q_indices[PAN]=HUMAN_Q_NECK_PAN;
      agents_head_info[i].Q_indices[TILT]=HUMAN_Q_NECK_TILT;
      agents_head_info[i].Q_indices[ROLL]=HUMAN_Q_NECK_ROLL;
      break;

      case HUMAN2_MA:
      agents_head_info[i].no_joints_neck=3;
      agents_head_info[i].joint_indices[PAN]=HUMAN_J_NECK_PAN;
      agents_head_info[i].joint_indices[TILT]=HUMAN_J_NECK_TILT;
      agents_head_info[i].joint_indices[ROLL]=HUMAN_J_NECK_ROLL;

      agents_head_info[i].no_Qs_neck=3;
      agents_head_info[i].Q_indices[PAN]=HUMAN_Q_NECK_PAN;
      agents_head_info[i].Q_indices[TILT]=HUMAN_Q_NECK_TILT;
      agents_head_info[i].Q_indices[ROLL]=HUMAN_Q_NECK_ROLL;

      break;
      
      case JIDO_MA:
      agents_head_info[i].no_joints_neck=2;
      agents_head_info[i].joint_indices[PAN]=JIDO_J_NECK_PAN;
      agents_head_info[i].joint_indices[TILT]=JIDO_J_NECK_TILT;

      agents_head_info[i].no_joints_neck=2;
      agents_head_info[i].Q_indices[PAN]=JIDO_Q_NECK_PAN;
      agents_head_info[i].Q_indices[TILT]=JIDO_Q_NECK_TILT;

      break;


    }
  }
}
*/
int init_current_vis_state_indices_for_MA_agents()
{

  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case HUMAN1_MA:
   CURR_VIS_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_HUM_VIS;
   break;
#ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
	   CURR_VIS_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_HUM_VIS;
break;
#endif

#ifdef JIDO_EXISTS_FOR_MA
      case JIDO_MA:
	   CURR_VIS_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_JIDO_VIS;
	   break;
#endif
#ifdef HRP2_EXISTS_FOR_MA
      case HRP2_MA:
	   CURR_VIS_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_HRP2_VIS;
break;
#endif

#ifdef PR2_EXISTS_FOR_MA
      case PR2_MA:
	   CURR_VIS_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_PR2_VIS;
break;
#endif


    }
  }
}


int init_current_reach_state_indices_for_MA_agents()
{

  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case HUMAN1_MA:
   CURR_REACH_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_HUM_REACH;
   break;
#ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
	   CURR_REACH_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_HUM_REACH;
break;
#endif

#ifdef JIDO_EXISTS_FOR_MA
      case JIDO_MA:
	   CURR_REACH_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_JIDO_REACH;
	   break;
#endif
#ifdef HRP2_EXISTS_FOR_MA
      case HRP2_MA:
	   CURR_REACH_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_HRP2_REACH;
break;
#endif

#ifdef PR2_EXISTS_FOR_MA
      case PR2_MA:
	   CURR_REACH_STATE_INDEX_MA_AGENT[i]=MM_CURRENT_STATE_PR2_REACH;
break;
#endif


    }
  }
}


  
int Create_and_init_Mightability_Maps()
{
  printf(" Inside Create_and_init_Mightability_Maps()\n");

 
  ////p3d_init_robot_parameters(); //To remove the dependency on Watch button
  ////printf(" After p3d_init_robot_parameters()\n");
   get_indices_for_MA_agents();
  assign_indices_of_robots();
  create_agents_for_Mightabilities();
  init_HRI_task_name_ID_map();
 
  init_visibility_acceptance_for_tasks();
  init_current_vis_state_indices_for_MA_agents();
  init_current_reach_state_indices_for_MA_agents();
  /////for(int it=0;it<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;it++)
  /////printf("indices_of_MA_agents[%d]=%d\n",it,indices_of_MA_agents[it]);
  //////init_agents_head_info_for_MA();
  ////////get_horizontal_triangles(global_htris);
  ////////return 1;

  //init_agents_state_transition_costs();
  
  printf(" Calling find_Mightability_Maps() \n");
  //////////////find_affordance_new();
  find_Mightability_Maps();
  printf(" After find_Mightability_Maps()\n");
  ////find_symbolic_Mightability_Map();
  find_symbolic_Mightability_Map_new();
  printf(" After find_symbolic_Mightability_Map_new()\n");
  ////show_symbolic_Mightability_Map_Relations();
  get_object_mightabilities();
  
  initialize_MM_resultant_set();
  init_flags_to_show_MM();
  init_accepted_states_for_agent_obj_MA();
  init_accepted_states_of_agents_for_tasks();
  
  CALCULATE_AFFORDANCE=1;

  Affordances_Found=1; 

  ////fl_check_forms();
  ////g3d_draw_allwin_active();
  return 1;
}

int get_set_of_points_to_put_object(char *object_name)
{
  if(Affordances_Found==1)
    {
      //////////printf(" Inside get_set_of_points_to_put_object()\n");
      ////update_robots_and_objects_status();

      /*
	envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
	int x,y,z;

	int no = envPt_MM->no;
	int nr = envPt_MM->nr;
	p3d_obj *o;
	p3d_rob *r;
	int r_ctr=0;
	for(r_ctr=0;r_ctr<nr;r_ctr++)
	{
	robots_status_for_Mightability_Maps[r_ctr].has_moved=1
	} 
      */      

      ////HRP2_HAS_MOVED=1;
      ////HUMAN_HAS_MOVED=1;
      ////JIDO_HAS_MOVED=1;
      ////update_Mightability_Maps();

#ifdef JIDO_EXISTS_FOR_MA
      JIDO_find_candidate_points_on_plane_to_put_obj();
      ////assign_weights_on_candidte_points_to_put_obj(object_name); 
#elif defined(HRI_HRP2)
      find_candidate_points_on_plane_to_put_obj_new();
      assign_weights_on_candidte_points_to_put_obj(object_name); 
#endif
   
      CANDIDATE_POINTS_FOR_TASK_FOUND=1;
      ////SHOW_PUT_OBJ_CANDIDATES=1;
      ////SHOW_WEIGHT_FOR_CANDIDATE_POINTS=0;
      ////show_3D_workspace_Bounding_Box();
      ////show_weighted_candidate_points_to_put_obj(0);
      return 1;
    } 

  return 0;


}


int get_set_of_points_to_hide_object(char *object_name)
{
  if(Affordances_Found==1)
    {
      //////////printf(" Inside get_set_of_points_to_put_object()\n");
      ////update_robots_and_objects_status();

      /*
	envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
	int x,y,z;

	int no = envPt_MM->no;
	int nr = envPt_MM->nr;
	p3d_obj *o;
	p3d_rob *r;
	int r_ctr=0;
	for(r_ctr=0;r_ctr<nr;r_ctr++)
	{
	robots_status_for_Mightability_Maps[r_ctr].has_moved=1
	} 
      */      

      ////HRP2_HAS_MOVED=1;
      ////HUMAN_HAS_MOVED=1;
      ////JIDO_HAS_MOVED=1;
      ////update_Mightability_Maps();

#ifdef JIDO_EXISTS_FOR_MA
      JIDO_find_candidate_points_on_plane_to_hide_obj();
      ////assign_weights_on_candidte_points_to_hide_obj(object_name); 
#elif defined(HRI_HRP2)
      find_candidate_points_on_plane_to_hide_obj_new();
      assign_weights_on_candidte_points_to_hide_obj(object_name); 
#endif
   
      CANDIDATE_POINTS_FOR_TASK_FOUND=1;
      ////SHOW_PUT_OBJ_CANDIDATES=1;
      ////SHOW_WEIGHT_FOR_CANDIDATE_POINTS=0;
      ////show_3D_workspace_Bounding_Box();
      ////show_weighted_candidate_points_to_put_obj(0);
      return 1;
    } 

  return 0;


}



int get_set_of_points_to_show_object(char *object_name)
{
  if(Affordances_Found==1)
    {
      //////////printf(" Inside get_set_of_points_to_put_object()\n");
      ////update_robots_and_objects_status();

      /*
	envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
	int x,y,z;

	int no = envPt_MM->no;
	int nr = envPt_MM->nr;
	p3d_obj *o;
	p3d_rob *r;
	int r_ctr=0;
	for(r_ctr=0;r_ctr<nr;r_ctr++)
	{
	robots_status_for_Mightability_Maps[r_ctr].has_moved=1
	} 
      */      

      ////HRP2_HAS_MOVED=1;
      ////HUMAN_HAS_MOVED=1;
      ////JIDO_HAS_MOVED=1;
      ////update_Mightability_Maps();

#ifdef JIDO_EXISTS_FOR_MA
      JIDO_find_candidate_points_to_show_obj();
      ////assign_weights_on_candidte_points_to_show_obj(object_name); 
#elif defined(HRI_HRP2)
      find_candidate_points_to_show_obj_new();
      assign_weights_on_candidte_points_to_show_obj(object_name); 
#endif
   
      CANDIDATE_POINTS_FOR_TASK_FOUND=1;
      ////SHOW_PUT_OBJ_CANDIDATES=1;
      ////SHOW_WEIGHT_FOR_CANDIDATE_POINTS=0;
      ////show_3D_workspace_Bounding_Box();
      ////show_weighted_candidate_points_to_put_obj(0);
      return 1;
    } 

  return 0;


}

int get_set_of_points_to_give_object(char *object_name)
{
  if(Affordances_Found==1)
    {
      //////////printf(" Inside get_set_of_points_to_put_object()\n");
      ////update_robots_and_objects_status();

      /*
	envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
	int x,y,z;

	int no = envPt_MM->no;
	int nr = envPt_MM->nr;
	p3d_obj *o;
	p3d_rob *r;
	int r_ctr=0;
	for(r_ctr=0;r_ctr<nr;r_ctr++)
	{
	robots_status_for_Mightability_Maps[r_ctr].has_moved=1
	} 
      */      

      ////HRP2_HAS_MOVED=1;
      ////HUMAN_HAS_MOVED=1;
      ////JIDO_HAS_MOVED=1;
      ////update_Mightability_Maps();

#ifdef JIDO_EXISTS_FOR_MA
      JIDO_find_candidate_points_to_give_obj();
      ////assign_weights_on_candidte_points_to_give_obj(object_name); 
#elif defined(HRI_HRP2)
      find_candidate_points_to_give_obj();
      assign_weights_on_candidte_points_to_give_obj(object_name); 
#endif
   
      CANDIDATE_POINTS_FOR_TASK_FOUND=1;
      ////SHOW_PUT_OBJ_CANDIDATES=1;
      ////SHOW_WEIGHT_FOR_CANDIDATE_POINTS=0;
      ////show_3D_workspace_Bounding_Box();
      ////show_weighted_candidate_points_to_put_obj(0);
      return 1;
    } 

  return 0;


}


/*
  void get_Frustum_Vertices(float l, float r, float b, float t, float n, float f)
  {
  point_co_ordi frustumVertices[8];
  float ratio;
  float farLeft;
  float farRight;
  float farBottom;
  float farTop;
  int  projectionMode = 0;
  // perspective mode
  if(projectionMode == 0)
  ratio     = f / n;
  // orthographic mode
  else
  ratio = 1;
  farLeft   = l * ratio;
  farRight  = r * ratio;
  farBottom = b * ratio;
  farTop    = t * ratio;
	
  // compute 8 vertices of the frustum
  // near top right
  frustumVertices[0].x = r;
  frustumVertices[0].y = t;
  frustumVertices[0].z = -n;
	
  // near top left
  frustumVertices[1].x = l;
  frustumVertices[1].y = t;
  frustumVertices[1].z = -n;
	
  // near bottom left
  frustumVertices[2].x = l;
  frustumVertices[2].y = b;
  frustumVertices[2].z = -n;
	
  // near bottom right
  frustumVertices[3].x = r;
  frustumVertices[3].y = b;
  frustumVertices[3].z = -n;
	
  // far top right
  frustumVertices[4].x = farRight;
  frustumVertices[4].y = farTop;
  frustumVertices[4].z = -f;
	
  // far top left
  frustumVertices[5].x = farLeft;
  frustumVertices[5].y = farTop;
  frustumVertices[5].z = -f;
	
  // far bottom left
  frustumVertices[6].x = farLeft;
  frustumVertices[6].y = farBottom;
  frustumVertices[6].z = -f;
	
  // far bottom right
  frustumVertices[7].x = farRight;
  frustumVertices[7].y = farBottom;
  frustumVertices[7].z = -f;
	
   
  //AKP: To store the frustumVertices
  int i=0;
  for(i=0;i<8&&no_FOV_end_point_vertices<1000;i++)
  {
  //printf(" get_Frustum_Vertices, i= %d\n",i);
  p3d_vector4 point, point2;
  point2[0] = frustumVertices[i].x;
  point2[1] = frustumVertices[i].y;
  point2[2] = frustumVertices[i].z;
  point2[3] = 1.0;
  //p3d_rob *rob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //p3d_matvec4Mult(rob->o[rob->cam_body_index]->jnt->abs_pos,point2,point);
  p3d_matvec4Mult(frustum_transformation_mat,point2,point);
  FOV_end_point_vertices[no_FOV_end_point_vertices][i].x=point[0];
  FOV_end_point_vertices[no_FOV_end_point_vertices][i].y=point[1];
  FOV_end_point_vertices[no_FOV_end_point_vertices][i].z=point[2];
  ////FOV_end_point_vertices[no_FOV_end_point_vertices][i].x=frustumVertices[i].x;
  ////FOV_end_point_vertices[no_FOV_end_point_vertices][i].y=frustumVertices[i].y;
  ////FOV_end_point_vertices[no_FOV_end_point_vertices][i].z=frustumVertices[i].z;
  }
  no_FOV_end_point_vertices++;
  ////////printf(" no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  }
*/
/*
  #if defined(WITH_XFORM)  
  int get_points_on_FOV_screen(p3d_rob *r)
  {
  printf(">>>> get_points_on_FOV_screen, r->cam_body_index =%d\n",r->cam_body_index);
  p3d_obj *objPt = r->o[r->cam_body_index]; 
  
  p3d_jnt *jntPt = objPt->jnt;
  //p3d_jnt *jntPt = r->joints[1];
  
  p3d_matrix4 mattemp,mattemp2;
  int i,j;
  switch ( r->cam_axe) {
  case 0:
  p3d_mat4Pos(mattemp,r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], M_PI-r->cam_tilt,r->cam_pan,0);
  break;
  case 1:
  p3d_mat4Pos(mattemp,r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], M_PI_2+r->cam_pan,0,-r->cam_tilt);
  p3d_mat4Pos(mattemp2,0,0,0, 0,0,M_PI_2);
  break;
  case 2:
  p3d_mat4Pos(mattemp,r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], 0, -M_PI_2-r->cam_pan, r->cam_tilt);
  break;
  default:
  break;
  }
  

  
  
  for(i=0 ; i<=3 ; i++){
  for(j=0 ; j<=3 ; j++){
      
  frustum_transformation_mat[i][j]=jntPt->abs_pos[i][j];
  ////printf(" %lf ",frustum_transformation_mat[i][j]);  
      
  }
  ////printf("\n");
  }
  //matrix[14]=0.0;
  //float degYang = (r->cam_v_angle * 180.0/M_PI);//good
  float degYang = (((r->cam_h_angle*2.0)/3.0) * 180.0/M_PI);
  
  
  ////g3d_draw_cone(r->cam_pos[0],r->cam_pos[1],r->cam_pos[2], r->cam_min_range, r->cam_max_range, r->cam_v_angle, r->cam_h_angle, r->cam_axe, r->cam_pan, r->cam_tilt);
  
  p3d_matrix4 tmp_res;
  p3d_mat4Mult(frustum_transformation_mat,mattemp,tmp_res);
  p3d_mat4Copy(tmp_res,frustum_transformation_mat);
  
  if ( r->cam_axe==1)
  {
      
  p3d_mat4Mult(frustum_transformation_mat,mattemp2,tmp_res);
  p3d_mat4Copy(tmp_res,frustum_transformation_mat);
  ////p3d_matvec4Mult(frustum_trans_mat,mattemp2,frustum_trans_mat);
  }
  //perspectiveGL(degYang, r->cam_h_angle/r->cam_v_angle ,0.001, 8.0);//original
  
  
  ////perspectiveGL(degYang, 3.0/2.0 ,0.001, 2.0);
  
  GLdouble fovY=degYang; 
  GLdouble aspect=3.0/2.0;
  GLdouble zNear=0.001;
  GLdouble zFar=2.0; 
  GLdouble fW, fH;
  //	Note:	tan( double ) uses radians but OpenGL works in degrees so we convert
  //			degrees to radians by dividing by 360 then multiplying by pi.
  fH = tanf( (fovY / 2.0) / 180.0 * M_PI ) * zNear;	
  // Same as fH = tan( fovY / 360 * pi ) * zNear;
  //	Calculate the distance from 0 of the x clipping plane based on the aspect ratio.
  //fW = tanf( (aspect / 2.0) / 180.0 * M_PI ) * zNear;
  fW = fH * aspect;
  //printf("fh %f   fw %f\n",fH, fW);	
  //glFrustum( -fW, fW, -fH, fH, zNear, zFar );
  
  //glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
  //g3d_set_color_mat(Any,tBluev);
  get_Frustum_Vertices(-fW, fW, -fH, fH, zNear, zFar );	
  
  return 1;

  }

  #endif
*/

int check_inside_polygon(int no_vertices, point_co_ordi *vertices, point_co_ordi point)//the order of vertices should be clockwise or counter clockwise 
{
  //Equation of one edge is (y-y1)-((y2-y1)/(x2-x1))*(x-x1)=0;
  int i=0;
  int ok=0;
  for(i=0;i<no_vertices;i++)
    {
      double sign_of_next_vertex=(vertices[(i+2)%no_vertices].y-vertices[i%no_vertices].y)-((vertices[(i+1)%no_vertices].y-vertices[i%no_vertices].y)/(vertices[(i+1)%no_vertices].x-vertices[i%no_vertices].x))/(vertices[(i+2)%no_vertices].x-vertices[(i)%no_vertices].x);
      double sign_of_point=(point.y-vertices[i%no_vertices].y)-((vertices[(i+1)%no_vertices].y-vertices[i%no_vertices].y)/(vertices[(i+1)%no_vertices].x-vertices[i%no_vertices].x))/(point.x-vertices[(i)%no_vertices].x);
      if((sign_of_next_vertex>0&&sign_of_point>0)||(sign_of_next_vertex<0&&sign_of_point<0))
	{
	  ok=1;
	}
      else
	{
	  ok=0;
	  return 0;
	}
  
    }

  if(ok==1)
    return 1;

}

#if defined(WITH_XFORM)
int is_point_in_fov(p3d_rob* robot, p3d_vector4 p)
{
  
  p3d_rob* rtemp = PSP_ROBOT;
  PSP_ROBOT = robot;
  //p3d_vector4 objectCenter;
  //double tempAngH = robot->cam_h_angle;
  //double tempAngW = robot->cam_h_angle;
  
  //p3d_get_robot_center(object, objectCenter); 
  //robot->cam_h_angle = angleH;
  //robot->cam_v_angle = angleW;
  int plane;
  G3D_Window *win = g3d_get_win_by_name("Perspective");
#if defined(WITH_XFORM)  
  g3d_refresh_win(win);
#endif
  int is_in_FOV=0;
  for(plane = 0; plane < 6; plane++ ) // for all perspective window frustum planes
    {
      // if the point is in the negative side of the frustum plan means that it i maybe inside the frustum box
      if(win->vs.frustum[plane][0] * (p[0]) + win->vs.frustum[plane][1] * (p[1])
	 + win->vs.frustum[plane][2] * (p[2]) + win->vs.frustum[plane][3] > 0 ) 
	{
	  is_in_FOV=1;
	}
      else
	{
	  is_in_FOV= 0;
	  plane=1000;
	}
    }
  PSP_ROBOT = rtemp;
  
  
  return is_in_FOV;
}
#endif

int initialize_surfaces_in_env()
{


  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_vector4 objCenter,robCenter;
  int i,j;
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  p3d_obj *o;
  p3d_rob *r;
  int contObj =0;
  p3d_vector4 pointHead,pointDisc, pointAhead;
 

  curr_surfaces_in_env.total_no_of_surfaces=0;
  int ns=0;
  //Robot body parts
  
  for(i=0;i<nr;i++)
    {
      r = envPt_MM->robot[i];
      if (strcasestr(r->name,"table"))
	{
	  //// printf("envPt_MM->robot[%d]->name =%s\n", i, envPt_MM->robot[i]->name);
	  ////printf(" envPt_MM->robot[%d]->BB.xmin, xmax, ymin, ymax, zmin, zmax : ( %lf, %lf, %lf, %lf, %lf, %lf )\n",i,envPt_MM->robot[i]->BB.xmin,envPt_MM->robot[i]->BB.xmax,envPt_MM->robot[i]->BB.ymin,envPt_MM->robot[i]->BB.ymax, envPt_MM->robot[i]->BB.zmin,envPt_MM->robot[i]->BB.zmax);	

	  curr_surfaces_in_env.flat_surf[ns].no_vertices=4;
	  curr_surfaces_in_env.flat_surf[ns].surface_shape=3;//Rectangle

	  //AKP NOTE : Take care that vertices should be in clockwise or anticlockwise order
	  // TO DO : AKP : WARNING : Now assuming that the rectangle sides are parallel to the x and y axes, so the vertices are directly assigned as the bounding box values. In future make changes to assign the vertices of such rectangles wgich are not parallel to the axes

	  curr_surfaces_in_env.flat_surf[ns].vertices[0].x=envPt_MM->robot[i]->BB.xmin;
	  curr_surfaces_in_env.flat_surf[ns].vertices[0].y=envPt_MM->robot[i]->BB.ymin;
	  curr_surfaces_in_env.flat_surf[ns].vertices[0].z=envPt_MM->robot[i]->BB.zmax;
 
	  curr_surfaces_in_env.flat_surf[ns].vertices[1].x=envPt_MM->robot[i]->BB.xmin;
	  curr_surfaces_in_env.flat_surf[ns].vertices[1].y=envPt_MM->robot[i]->BB.ymax;
	  curr_surfaces_in_env.flat_surf[ns].vertices[1].z=envPt_MM->robot[i]->BB.zmax;
 
	  curr_surfaces_in_env.flat_surf[ns].vertices[2].x=envPt_MM->robot[i]->BB.xmax;
	  curr_surfaces_in_env.flat_surf[ns].vertices[2].y=envPt_MM->robot[i]->BB.ymax;
	  curr_surfaces_in_env.flat_surf[ns].vertices[2].z=envPt_MM->robot[i]->BB.zmax;
 
	  curr_surfaces_in_env.flat_surf[ns].vertices[3].x=envPt_MM->robot[i]->BB.xmax;
	  curr_surfaces_in_env.flat_surf[ns].vertices[3].y=envPt_MM->robot[i]->BB.ymin;
	  curr_surfaces_in_env.flat_surf[ns].vertices[3].z=envPt_MM->robot[i]->BB.zmax;

	  curr_surfaces_in_env.flat_surf[ns].BR_x_min=envPt_MM->robot[i]->BB.xmin;
	  curr_surfaces_in_env.flat_surf[ns].BR_x_max=envPt_MM->robot[i]->BB.xmax;
	  curr_surfaces_in_env.flat_surf[ns].BR_y_min=envPt_MM->robot[i]->BB.ymin;;
	  curr_surfaces_in_env.flat_surf[ns].BR_y_max=envPt_MM->robot[i]->BB.ymax;;

	  curr_surfaces_in_env.flat_surf[ns].BR_z=envPt_MM->robot[i]->BB.zmax;
	  curr_surfaces_in_env.total_no_of_surfaces++;
	  ns++;
	}
      /*
	for(j=0;j<r->no;j++)
	{
	o = r->o[j];
	printf("envPt_MM->robot[%d]->o[%d]->name =%s, envPt_MM->robot[%d]->o[%d]->np =%d\n", i,j,  envPt_MM->robot[i]->o[j]->name,i,j,envPt_MM->robot[i]->o[j]->np);
	if (strcasestr(o->name,"table"))
	{
	//p3d_get_object_center(o,objCenter);
	printf(" envPt_MM->robot[%d]->o[%d]->BB.xmin, xmax, ymin, ymax, zmin, zmax : ( %lf, %lf, %lf, %lf, %lf, %lf )\n",i,j,envPt_MM->robot[i]->o[j]->BB.xmin,envPt_MM->robot[i]->o[j]->BB.xmax,envPt_MM->robot[i]->o[j]->BB.ymin,envPt_MM->robot[i]->o[j]->BB.ymax, envPt_MM->robot[i]->o[j]->BB.zmin,envPt_MM->robot[i]->o[j]->BB.zmax);	

              
	}
	}
      */
      //define if robot is near or not? here or in observation? od we need a different list
      // if ((ContObjTmp/r->no)>.4)
      // 

    }

  // 1st surface
  /*
    curr_surfaces_in_env.flat_surf[0].no_vertices=4;
    curr_surfaces_in_env.flat_surf[0].surface_shape=3;//Rectangle

    curr_surfaces_in_env.flat_surf[0].vertices[0].x=0.5+4.6;
    curr_surfaces_in_env.flat_surf[0].vertices[0].y=0.5-3;
    curr_surfaces_in_env.flat_surf[0].vertices[0].z=0.75;
 
    curr_surfaces_in_env.flat_surf[0].vertices[1].x=0.5+4.6;
    curr_surfaces_in_env.flat_surf[0].vertices[1].y=-0.5-3;
    curr_surfaces_in_env.flat_surf[0].vertices[1].z=0.75;
 
    curr_surfaces_in_env.flat_surf[0].vertices[2].x=-0.5+4.6;
    curr_surfaces_in_env.flat_surf[0].vertices[2].y=-0.5-3;
    curr_surfaces_in_env.flat_surf[0].vertices[2].z=0.75;
 
    curr_surfaces_in_env.flat_surf[0].vertices[3].x=-0.5+4.6;
    curr_surfaces_in_env.flat_surf[0].vertices[3].y=0.5-3;
    curr_surfaces_in_env.flat_surf[0].vertices[3].z=0.75;

    curr_surfaces_in_env.flat_surf[0].BR_x_min=-0.5+4.6;
    curr_surfaces_in_env.flat_surf[0].BR_x_max=0.5+4.6;
    curr_surfaces_in_env.flat_surf[0].BR_y_min=-0.5-3;
    curr_surfaces_in_env.flat_surf[0].BR_y_max=0.5-3;

    curr_surfaces_in_env.flat_surf[0].BR_z=0.75;
    curr_surfaces_in_env.total_no_of_surfaces=1;

    //2nd surface
    curr_surfaces_in_env.flat_surf[1].no_vertices=4;
    curr_surfaces_in_env.flat_surf[1].surface_shape=3;//Rectangle

    curr_surfaces_in_env.flat_surf[1].vertices[0].x=1.2+4.6;
    curr_surfaces_in_env.flat_surf[1].vertices[0].y=1.75-3;
    curr_surfaces_in_env.flat_surf[1].vertices[0].z=0.5;
 
    curr_surfaces_in_env.flat_surf[1].vertices[1].x=1.2+4.6;
    curr_surfaces_in_env.flat_surf[1].vertices[1].y=0.75-3;
    curr_surfaces_in_env.flat_surf[1].vertices[1].z=0.5;
 
    curr_surfaces_in_env.flat_surf[1].vertices[2].x=0.2+4.6;
    curr_surfaces_in_env.flat_surf[1].vertices[2].y=0.75-3;
    curr_surfaces_in_env.flat_surf[1].vertices[2].z=0.5;
 
    curr_surfaces_in_env.flat_surf[1].vertices[3].x=0.2+4.6;
    curr_surfaces_in_env.flat_surf[1].vertices[3].y=1.75-3;
    curr_surfaces_in_env.flat_surf[1].vertices[3].z=0.5;

    curr_surfaces_in_env.flat_surf[1].BR_x_min=0.2+4.6;
    curr_surfaces_in_env.flat_surf[1].BR_x_max=1.2+4.6;
    curr_surfaces_in_env.flat_surf[1].BR_y_min=0.75-3;
    curr_surfaces_in_env.flat_surf[1].BR_y_max=1.75-3;

    curr_surfaces_in_env.flat_surf[1].BR_z=0.5;
    curr_surfaces_in_env.total_no_of_surfaces=2;
  
  */
  return 1;
}

int caculate_and_show_resultant_MM()
{
#ifndef COMMENT_TMP
  double radius=grid_around_HRP2.GRID_SET->pace/3.0;
  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;

	      double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);

  
      
	      int show_showing_points=0;

       
	      if(SHOW_3D_VISIBLE_PLACE_HUM==1)
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
		    {
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			{
			  ////////printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  resultant_MM_after_set_operation[x][y][z]=0;
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			      resultant_MM_after_set_operation[x][y][z]=1;
			    }
			}
		    }
		}
#ifdef HUMAN2_EXISTS_FOR_MA
	      if(SHOW_3D_VISIBLE_PLACE_HUM2==1)
		{        
		  ////  printf(" Showing for second human\n");
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
		    {
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  resultant_MM_after_set_operation[x][y][z]=0;
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 

			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			      resultant_MM_after_set_operation[x][y][z]=1;
			    }

      
			}
      
       
		    } 
		}
#endif

#ifdef HRP2_EXISTS_FOR_MA
	      if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)//CURRENTLY FOR JIDO same flag is being used
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
        
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1)||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  ////  printf(" Drawing disc\n"); 
			  ////resultant_MM_after_set_operation[x][y][z]=1;
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
         
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc AND\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
         
		} 

#endif
       
#ifdef JIDO_EXISTS_FOR_MA 
       
	      if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)//CURRENTLY FOR JIDO same flag is being used
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
        
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation==1)||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  ////  printf(" Drawing disc\n"); 
			  ////resultant_MM_after_set_operation[x][y][z]=1;
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
         
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc AND\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
         
		} 
#endif
      
	      if(SHOW_3D_DIRECT_REACHABLE_HUM==1)
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc Human reach OR\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;   
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      ////printf(" Drawing disc Human reach AND\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
        
		} 

	      if(SHOW_3D_BENDING_REACHABLE_HUM==1) 
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
        
		} 

	      if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM==1)
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
        
		} 

 
#ifdef HUMAN2_EXISTS_FOR_MA

	      if(SHOW_3D_DIRECT_REACHABLE_HUM2==1)
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    } 
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
        
		} 

	      if(SHOW_3D_BENDING_REACHABLE_HUM==1) 
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		    }
        
		} 

	      if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM==1)
		{        
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
         
		    }
		} 


#endif

#ifdef HRP2_EXISTS_FOR_MA

	      if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
		{        
             
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    } 
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
         
		    }
       
		}

      
#endif

#ifdef JIDO_EXISTS_FOR_MA
	      if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
		{        
             
		  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1||resultant_MM_after_set_operation[x][y][z]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  resultant_MM_after_set_operation[x][y][z]=1;
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    } 
		  else
		    {
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
         
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
         
		    }
       
		}
#endif

       
	      //======START for 2D on table surfaces========//
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		{
		  ////printf(" is_horizontal_surface\n");
		  if(SHOW_2D_VISIBLE_PLACE_HUM==1)
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)||resultant_MM_after_set_operation[x][y][z]==1)
				{
				  //// printf(" Drawing disc\n"); 
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				  resultant_MM_after_set_operation[x][y][z]=1;
				}
			    }
			}
		    }
#ifdef HUMAN2_EXISTS_FOR_MA
		  if(SHOW_2D_VISIBLE_PLACE_HUM2==1)
		    {        
		      ////  printf(" Showing for second human\n");
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)&&resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			  else
			    {
			      resultant_MM_after_set_operation[x][y][z]=0;
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)||resultant_MM_after_set_operation[x][y][z]==1)
				{
				  //// printf(" Drawing disc\n"); 

				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				  resultant_MM_after_set_operation[x][y][z]=1;
				}

      
			    }
      
       
			} 
		    }
#endif

#ifdef HRP2_EXISTS_FOR_MA

		  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)//CURRENTLY FOR JIDO same flag is being used
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
        
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1)||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      ////  printf(" Drawing disc\n"); 
			      ////resultant_MM_after_set_operation[x][y][z]=1;
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
         
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
				  //// printf(" Drawing disc AND\n"); 
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
         
		    } 

       
#endif


#ifdef JIDO_EXISTS_FOR_MA
       
		  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)//CURRENTLY FOR JIDO same flag is being used
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
        
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation==1)||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      ////  printf(" Drawing disc\n"); 
			      ////resultant_MM_after_set_operation[x][y][z]=1;
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
         
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
				  //// printf(" Drawing disc AND\n"); 
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
         
		    } 

#endif

		  if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc Human reach OR\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;   
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
				  ////printf(" Drawing disc Human reach AND\n"); 
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
        
		    } 

		  if(SHOW_2D_BENDING_REACHABLE_HUM==1) 
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
        
		    } 

		  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				 g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
        
		    } 

 
#ifdef HUMAN2_EXISTS_FOR_MA

		  if(SHOW_2D_DIRECT_REACHABLE_HUM2==1)
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			} 
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
        
		    } 

		  if(SHOW_2D_BENDING_REACHABLE_HUM==1) 
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
			}
        
		    } 

		  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
		    {        
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			}
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
         
			}
		    } 


#endif

#ifdef HRP2_EXISTS_FOR_MA

		  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
		    {        
             
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			} 
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
         
			}
       
		    }
 
       
#endif

#ifdef JIDO_EXISTS_FOR_MA
		  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
		    {        
             
		      if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_OR)
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1||resultant_MM_after_set_operation[x][y][z]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      resultant_MM_after_set_operation[x][y][z]=1;
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			    }
			} 
		      else
			{
			  if(CURRENT_SET_OPERATOR_ON_MM==MM_SET_OPR_AND)
			    {
			      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1&&resultant_MM_after_set_operation[x][y][z]==1)
				{
         
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
				}
			      else
				{
				  resultant_MM_after_set_operation[x][y][z]=0;
				}
			    }
         
			}
       
		    }
#endif
		}
	      //fl_check_forms();
	      //g3d_draw_allwin_active();
	      ////}
	      ////else
	      ////{
	      ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
	      ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
	      ////} 
	      //  fl_check_forms();
	    }
	} 
    }
  //fl_check_forms();
  //     g3d_draw_allwin_active();
#endif
  return 1;
}

int show_Mightability_Maps()
{
double radius=grid_around_HRP2.GRID_SET->pace/3.0;
double z_world;
double cell_x_world;
double cell_y_world;
double cell_z_world;
 double z_shift_for_horizontal_points=0.02;
	      
  ////printf(" Inside show affordance, radious=%lf\n",radius);
  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	
	      cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);

              for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	      {
		if(SHOW_MIGHTABILITY_MAPS_FOR_AGENTS[i]==0)
		  continue;
		
	      for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
		{
		if(curr_flags_show_Mightability_Maps.show_visibility[i][j]>0)
		 {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[i][j]==1)
		    {
		      //printf(" Drawing disc\n");
		      if(SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D[i]==1)
		      {
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, curr_flags_show_Mightability_Maps.show_visibility[i][j], NULL);
		      }
		      
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1&&SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE[i]==1)
		      {
		       z_world=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.exact_z_val+z_shift_for_horizontal_points;
    
		       g3d_drawDisc(cell_x_world, cell_y_world, z_world, radius, curr_flags_show_Mightability_Maps.show_visibility[i][j], NULL);
		      }
		      
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_table==1&&SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE[i]==1)
		      {
		       z_world=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.exact_z_val+z_shift_for_horizontal_points;
    
		       g3d_drawDisc(cell_x_world, cell_y_world, z_world, radius, curr_flags_show_Mightability_Maps.show_visibility[i][j], NULL);
		      }
		    }
		 } 
		}
	      for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		{
		for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
		  {
		  if(curr_flags_show_Mightability_Maps.show_reachability[i][j][k]>0)
		   {
		     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[i][j][k]==1)
		    {
		      if(SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_IN_3D[i]==1)
		      {
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, curr_flags_show_Mightability_Maps.show_reachability[i][j][k], NULL);
		      }
		      
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1&&SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_PLANE[i]==1)
		      {
		       z_world=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.exact_z_val+z_shift_for_horizontal_points;
    
		      g3d_drawDisc(cell_x_world, cell_y_world, z_world, radius, curr_flags_show_Mightability_Maps.show_reachability[i][j][k], NULL);
		      }
		      
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_table==1&&SHOW_MIGHTABILITY_MAPS_FOR_AGENTS_ON_TABLE[i]==1)
		      {
		       z_world=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.exact_z_val+z_shift_for_horizontal_points;
    
		       g3d_drawDisc(cell_x_world, cell_y_world, z_world, radius, curr_flags_show_Mightability_Maps.show_reachability[i][j][k], NULL);
		      }
		      //printf(" Drawing disc\n"); 
		      //////g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, curr_flags_show_Mightability_Maps.show_reachability[i][j][k], NULL);
		    }
		   }
		  }
		 }
	      }
	      /*
	      if(SHOW_3D_VISIBLE_PLACE_HUM==1)
		{        
        
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[HUMAN1_MA][MM_CURRENT_STATE_HUM_VIS]==1)
		    {
		      //printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  ////else
		  {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[HUMAN1_MA][MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS]==1)
		      {
			//printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
		      }
		  }
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[HUMAN1_MA][MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS]==1)
		    {
		      //printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
		    } 
		}
		*/
	    }
	}
    }
  
}



int show_3d_grid_affordances_new()
{
#ifndef COMMENT_TMP
  /*
  //tmp for showing the points for putting inside 
  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  double increment=3.0/4.0*grid_around_HRP2.GRID_SET->pace;
  int nr_ctr=0;
  int nr = envPt_MM->nr;
  for(nr_ctr=0;nr_ctr<nr;nr_ctr++)
  {
  if(strcasestr(envPt_MM->robot[nr_ctr]->name,"ROBOT")||strcasestr(envPt_MM->robot[nr_ctr]->name,"HUMAN")||strcasestr(envPt_MM->robot[nr_ctr]->name,"TABLE"))
  {
  //No need to calculate PUTABILITY :-)
  }
  else
  {
  double top_z=envPt_MM->robot[nr_ctr]->BB.zmax;
  int cell_top_z=(top_z- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace;
  ////cell_top_z+=1;
  double BB_x_ctr;
  double BB_y_ctr;

  if(cell_top_z>=0&&cell_top_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  { 
  for(BB_x_ctr=envPt_MM->robot[nr_ctr]->BB.xmin;BB_x_ctr<envPt_MM->robot[nr_ctr]->BB.xmax;BB_x_ctr+=increment)
  {
  int cell_x=(BB_x_ctr- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
  for(BB_y_ctr=envPt_MM->robot[nr_ctr]->BB.ymin;BB_y_ctr<envPt_MM->robot[nr_ctr]->BB.ymax;BB_y_ctr+=increment)
  {
      
  int cell_y=(BB_y_ctr- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace;  
 
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {

  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].val>=0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z-1].val>=0)//No obstacle
  {

  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_JIDO_Hand==1)
  {  
  //g3d_drawDisc(BB_x_ctr, BB_y_ctr, top_z, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);
  }
    
  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_human_RHand==1)
  {  
  g3d_drawDisc(BB_x_ctr, BB_y_ctr, top_z, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
  }

  }
  }
  } 
  }
  }
  }
  }
  }
  */
    

  /* point_co_ordi shoulder_pos;
     shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);

     shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Red, NULL);
  */
  /*
    point_co_ordi neck_pos;
    neck_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    neck_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    neck_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(neck_pos.x, neck_pos.y, neck_pos.z, .1, Blue, NULL);
  */
  ////printf("Inside show_3d_grid_affordances()\n");
  /*printf("Inside show_3d_grid_affordances_new()\n");
    point_co_ordi shoulder_pos;
    ////point_co_ordi sphere_pt;
      
    // Reachability Test by right hand
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
 
    no_sphere_surface_pts=0;
    find_reachable_sphere_surface(1,1);
    double interval=grid_around_HRP2.GRID_SET->pace/1.0;
    for(int sp_ctr=0;sp_ctr<no_sphere_surface_pts;sp_ctr++)
    {
    double t=0;
    for(;t<1;t+=interval) 
    { 
      
    //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

    double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
    double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
    double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
    g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
    }
    //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
    }
   
    
    shoulder_pos.x = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Red, NULL);
  
  
    no_sphere_surface_pts=0;
    find_reachable_sphere_surface(2,1);

    for(int sp_ctr=0;sp_ctr<no_sphere_surface_pts;sp_ctr++)
    {
    double t=0;
    for(;t<1;t+=interval) 
    { 
      
    //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

    double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
    double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
    double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
    g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
    }
    //// g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
    }
  */
  /*


  point_co_ordi JIDO_eye_pos;
  JIDO_eye_pos.x = ACBTSET->robot->joints[ACBTSET->robot->cam_body_index]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  JIDO_eye_pos.y = ACBTSET->robot->joints[ACBTSET->robot->cam_body_index]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  JIDO_eye_pos.z = ACBTSET->robot->joints[ACBTSET->robot->cam_body_index]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  g3d_drawDisc(JIDO_eye_pos.x, JIDO_eye_pos.y, JIDO_eye_pos.z, .1, Green, NULL);


  g3d_drawDisc(agent_eye_pos.x, agent_eye_pos.y,agent_eye_pos.z, 0.1, Red, NULL);
  */

  double radius=grid_around_HRP2.GRID_SET->pace/3.0;
  ////printf(" Inside show affordance, radious=%lf\n",radius);
  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;

	      double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);

  
      
	      int show_showing_points=0;

	      ////printf("SHOW_3D_VISIBLE_PLACE_HUM=%d\n",SHOW_3D_VISIBLE_PLACE_HUM);
	      if(SHOW_3D_VISIBLE_PLACE_HUM==1)
		{        
        
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_CURRENT_STATE_HUM_VIS]==1)
		    {
		      //printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  ////else
		  {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS]==1)
		      {
			//printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
		      }
		  }
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS]==1)
		    {
		      //printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
		    } 
		  /*
		    #ifdef HUMAN2_EXISTS_FOR_MA
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1)
		    {
		    //// printf(" Drawing disc\n"); 
		    g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
		    }
		    ////else
		    {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)
		    {
		    //// printf(" Drawing disc\n"); 
		    g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
		    }
		    }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1)
		    {
		    //// printf(" Drawing disc\n"); 
		    g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Blue, NULL);
		    } 
		    #endif
		  */
		}

	      ////int SHOW_3D_VISIBLE_PLACE_HUM2=1;
#ifdef HUMAN2_EXISTS_FOR_MA
	      if(SHOW_3D_VISIBLE_PLACE_HUM2==1)
		{        
		  printf(" Showing for second human\n");
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  ////else
		  {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
		      }
		  }
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
		    } 
       
        
		}
#endif

	      if(SHOW_3D_VISIBLE_PLACE_STANDING_HUM==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS]==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  ////else
		  {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS]==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
		      }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_STANDING_LEAN_FORWARD_STATE_HUM_VIS]==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
		      }
		  } 
		}
       
	      if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2[MM_CURRENT_STATE_HRP2_VIS]==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  ////else
		  {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2[MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS]==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
		      }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2[MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS]==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
		      }
		  }
		} 
      
       
	      if(SHOW_3D_VISIBLE_PLACES_FOR_HRP2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO[MM_CURRENT_STATE_JIDO_VIS]==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  ////else
		  {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO[MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS]==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
		      }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO[MM_STRAIGHT_HEAD_STATE_JIDO_VIS]==1)
		      {
			//// printf(" Drawing disc\n"); 
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
		      }
		  }
		}        

/*
	      if(SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN==1)
		{        
		  //printf("**** SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=%d, SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=%d\n",SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN,SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN);
        
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		}
*/

	      if(SHOW_3D_DIRECT_REACHABLE_HUM==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_CURRENT_STATE_HUM_REACH]==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_CURRENT_STATE_HUM_REACH]==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_CURRENT_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_CURRENT_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}


	      if(SHOW_3D_BENDING_REACHABLE_HUM==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}

	      if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}

#ifdef HUMAN2_EXISTS_FOR_MA
	      if(SHOW_3D_DIRECT_REACHABLE_HUM2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}


	      if(SHOW_3D_BENDING_REACHABLE_HUM2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}

	      if(SHOW_3D_TURNING_AROUND_REACHABLE_HUM2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}
#endif
#ifndef COMMENT_TMP
	      if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			}
   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }
		}

	      if(SHOW_3D_DIRECT_REACHABLE_HRP2==1)
		{        

		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
        
        
		}

	      if(SHOW_3D_COMMON_REACH_HRP2_HUMAN==1)
		{
		  //printf("SHOW_3D_COMMON_REACH_HRP2_HUMAN=%d\n",SHOW_3D_COMMON_REACH_HRP2_HUMAN);
		  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)||(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)))//Points which are reachable by both hands of both, human and HRP2
		    {
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)))//Points which are reachable by both hands of HRP2 and atleast one hand of human
			{
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		      else
			{ 
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1))//Point which is reachable by either hand of both HRP2 and human
			    {
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }
		}

	      if(SHOW_3D_COMMON_REACH_HRP2_HUMAN==1)
		{
		  //printf("SHOW_3D_COMMON_REACH_HRP2_HUMAN=%d\n",SHOW_3D_COMMON_REACH_HRP2_HUMAN);
		  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)&&((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)))//Points which are reachable by both hands of both, human and HRP2
		    {
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
		    }
		  else
		    {
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1))//Points which are reachable by both hands of HRP2 and atleast one hand of human
			{
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
         
		    }
		}

#endif
	      //======START for 2D on table surfaces========//
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		{
		  ////printf(" is_horizontal_surface\n");
		  if(SHOW_2D_VISIBLE_PLACE_HUM==1)
		    {        
        
       
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_CURRENT_STATE_HUM_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      ////else
		      {
			if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS]==1)
			  {
			    //// printf(" Drawing disc\n"); 
			    g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
			  }
		      }
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}  

		    }

       
#ifdef HUMAN2_EXISTS_FOR_MA
		  if(SHOW_2D_VISIBLE_PLACE_HUM2==1)
		    {        
        
       
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      ////else
		      {
			if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_neck_turn==1)
			  {
			    //// printf(" Drawing disc\n"); 
			    g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
			  }
		      }
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human2_straight_head_orientation==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}  

		    }
#endif
       
		  if(SHOW_2D_VISIBLE_PLACE_STANDING_HUM==1)
		    {        
        
		      //if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[]==1)
			//{
			  ////printf(" visible by human\n");
			  //// printf(" Drawing disc\n"); 
			//  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			//}
        
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
			}

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    }

		  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2[MM_CURRENT_STATE_HRP2_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2[MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
			}
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2[MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    } 

		  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1)
		    {

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO[MM_CURRENT_STATE_JIDO_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO[MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Red, NULL);
			}
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO[MM_STRAIGHT_HEAD_STATE_JIDO_VIS]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			}
		    } 
#ifndef COMMENT_TMP      
		  if(SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN==1)
		    {        
		      //printf("**** SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=%d, SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=%d\n",SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN,SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN);
        
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }
#endif       
		  if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_CURRENT_STATE_HUM_REACH]==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_CURRENT_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_CURRENT_STATE_HUM_REACH]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_CURRENT_STATE_HUM_REACH]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }

		  if(SHOW_2D_BENDING_REACHABLE_HUM==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }

		  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1)
			{
			  //// printf(" Drawing disc\n"); 
			  //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL); 
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH]==1)
			    {
			      ////printf(" RHand reach by turn and bend Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }
        
#ifdef HUMAN2_EXISTS_FOR_MA
       
		  if(SHOW_2D_DIRECT_REACHABLE_HUM2==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }

		  if(SHOW_2D_BENDING_REACHABLE_HUM2==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1)
			{
			  //// printf(" Drawing disc\n"); 
			  //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_bending==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL);
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_bending==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }

		  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM2==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending==1)
			{
			  //// printf(" Drawing disc\n"); 
			  //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Yellow, NULL); 
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending==1)
			    {
			      ////printf(" RHand reach by turn and bend Drawing disc\n"); 
			      //////////g3d_drawDisc(cell_x_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_y_world+grid_around_HRP2.GRID_SET->pace/2.0, cell_z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world,radius, Blue, NULL);
			    }
			}
		    }
       
#endif  
#ifndef COMMENT_TMP
		  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world,radius, Yellow, NULL);
			    }
   
			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			    {
			      //// printf(" Drawing disc\n"); 
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			}
		    }
    
		  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
		    {        

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			{
			  //// printf(" Drawing disc\n"); 
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		    }

		  if(SHOW_2D_COMMON_REACH_HRP2_HUMAN==1)
		    {
		      ////////printf("inside SHOW_3D_COMMON_REACH_HRP2_HUMAN=%d\n",SHOW_3D_COMMON_REACH_HRP2_HUMAN);
		      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)||(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)))//Points which are reachable by both hands of both, human and HRP2
			{
			  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
			}
		      else
			{
			  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)))//Points which are reachable by both hands of HRP2 and atleast one hand of human
			    {
			      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
			    }
			  else
			    { 
			      if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1))//Point which is reachable by either hand of both HRP2 and human
				{
				  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Blue, NULL);
				}
			    }
			}
		    } 
#endif
		} 
	      //======END for 2D on table surfaces ========//

	      int show_horizontal_surfaces=0;
	      if(show_horizontal_surfaces==1)
		{  
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		    {
		      //// printf(" Drawing disc\n"); 
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world+0.0, radius, Green, NULL);
		    }
		}
#ifndef COMMENT_TMP
	      if(show_showing_points==1) // To calculate the candidate points to show some object to human
		{
		  if((grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)&&(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1))
		    {
          
		      g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, Green, NULL);
         
		      /*
			p3d_vector3 target_in_global_frame;
			target_in_global_frame[0]=cell_x_world;
			target_in_global_frame[1]=cell_y_world;
			target_in_global_frame[2]=cell_z_world;
			int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
			int hand_by_reach=2; //1 for left, 2 for right hand
			double task_duration=3.0;//in s
			int thumb_up_constraint=1;//Means the x axis of the hand will be made parallel to the z axis og global frame
			vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
			int reachable_by_R_hand=HRP2_hand_reach(target_in_global_frame, hand_by_reach, task_duration, HRP2_state,thumb_up_constraint);
			attStandingRobot->staticState ( backupConfig );
			if(reachable_by_R_hand==1)
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
			else
			g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
		      */
          
           
		    } 
		}
#endif
	      //fl_check_forms();
	      //g3d_draw_allwin_active();
	      ////}
	      ////else
	      ////{
	      ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
	      ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
	      ////} 
	      //  fl_check_forms();

	    }
	} 
    }
  //fl_check_forms();
  //     g3d_draw_allwin_active();

#endif
  return 1;
}

int MA_deactivate_collision_between_non_torso_parts_and_env(p3d_rob * for_human)
{
  
  for(int i=0; i<for_human->no; i++)
  {
   //// printf(" >>>> for_human->o[i]->name = %s \n", for_human->o[i]->name);
    if(strcasestr(for_human->o[i]->name,"Humerus")||strcasestr(for_human->o[i]->name,"Radius")||strcasestr(for_human->o[i]->name,"Hand"))
    {
       p3d_col_deactivate_obj(for_human->o[i]);
    }
  }
}

int MA_activate_collision_between_non_torso_parts_and_env(p3d_rob * for_human)
{
  for(int i=0; i<for_human->no; i++)
  {
    ////printf(" >>>> for_human->o[i]->name = %s \n", for_human->o[i]->name);
    if(strcasestr(for_human->o[i]->name,"Humerus")||strcasestr(for_human->o[i]->name,"Radius")||strcasestr(for_human->o[i]->name,"Hand"))
    {
       p3d_col_activate_obj(for_human->o[i]);
    }
  }
}

int update_current_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT for_agent) 
{

  int hum_index;
  if(for_agent==HUMAN1_MA)
    {
     hum_index=indices_of_MA_agents[for_agent];
     
    }
#ifdef HUMAN2_EXISTS_FOR_MA
  else
    {
      if(for_agent==HUMAN2_MA)
	{
	  ////printf(" Updating reachability of Human2 \n");
	  hum_index=indices_of_MA_agents[for_agent];
	          
	}
      else
	{
	  printf(" ******* AKP Warning: Not the correct type for_agent has been given for update_3d_grid_reachability_for_human_new(), So returning 0. >>>>>>\n");
	  return 0;
	}
    }
#endif 
  //////////printf("Inside update_3d_grid_reachability_for_human_new()\n");

  p3d_col_deactivate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],envPt_MM->robot[hum_index]);
  MA_deactivate_collision_between_non_torso_parts_and_env(envPt_MM->robot[hum_index]); 

  point_co_ordi shoulder_pos;
  ////point_co_ordi sphere_pt;
  
 
  double interval=grid_around_HRP2.GRID_SET->pace-0.005;///1.5;
   int kcd_with_report=0;
 
   int res = p3d_col_test_robot(envPt_MM->robot[hum_index],kcd_with_report);
	    if(res>0)
	      {
// 		kcd_with_report=0;
// 		res = p3d_col_test_self_collision(envPt_MM->robot[hum_index],kcd_with_report);
// 		if(res>0)
// 		  {
// 		    printf(" There is self collision with human, res=%d \n", res);
// 		    pqp_print_colliding_pair();
// 		    //return 0;
// 		  }
// 		else
// 		  {
		    printf(" **** There is collision with human current position res=%d \n",  res);
		    pqp_print_colliding_pair();
		    
		    p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],envPt_MM->robot[hum_index]);
                    MA_activate_collision_between_non_torso_parts_and_env(envPt_MM->robot[hum_index]); 
                    return 0;
//		  }
	      }
	      else
	      {
	     
	      update_3d_grid_reachability_for_agent_MM(for_agent, MA_LEFT_HAND, CURR_REACH_STATE_INDEX_MA_AGENT[for_agent]);
	
	      update_3d_grid_reachability_for_agent_MM(for_agent, MA_RIGHT_HAND, CURR_REACH_STATE_INDEX_MA_AGENT[for_agent]);
	
	      }
	      
	          p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],envPt_MM->robot[hum_index]);
                    MA_activate_collision_between_non_torso_parts_and_env(envPt_MM->robot[hum_index]); 
		    return 1;
}

int update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT for_agent, int for_posture_state) 
{

  int update_curr_state=0;
  int hum_index;
  if(for_agent==HUMAN1_MA)
    {
     hum_index=indices_of_MA_agents[for_agent];
     
    }
#ifdef HUMAN2_EXISTS_FOR_MA
  else
    {
      if(for_agent==HUMAN2_MA)
	{
	  ////printf(" Updating reachability of Human2 \n");
	  hum_index=indices_of_MA_agents[for_agent];
	          
	}
      else
	{
	  printf(" ******* AKP Warning: Not the correct type for_agent has been given for update_3d_grid_reachability_for_human_new(), So returning 0. >>>>>>\n");
	  return 0;
	}
    }
#endif 
  //////////printf("Inside update_3d_grid_reachability_for_human_new()\n");

  p3d_col_deactivate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],envPt_MM->robot[hum_index]);
  MA_deactivate_collision_between_non_torso_parts_and_env(envPt_MM->robot[hum_index]); 

  point_co_ordi shoulder_pos;
  ////point_co_ordi sphere_pt;
  configPt hum_tmp_pos=p3d_get_robot_config(envPt_MM->robot[hum_index]);
  //////////double yaw_ang=hum_tmp_pos[11]; 
   
  double orig_yaw_ang=hum_tmp_pos[HUMANq_TORSO_PAN];
  //////////double pitch_ang=hum_tmp_pos[14]; 
  
  double orig_pitch_ang=hum_tmp_pos[HUMANq_TORSO_TILT]; 
  
  int turn_collision=0;
  //double yaw_
  ////for(;yaw_ang<2.0*M_PI&&turn_collision==0;yaw_ang+=0.5)


  double interval=grid_around_HRP2.GRID_SET->pace-0.005;///1.5;

  
  int straight_lean_type;
  int lean_reach_type;
  int turn_reach_state;
  int turn_lean_reach_type;

  if(for_posture_state==HRI_SITTING)//sitting
  {
   straight_lean_type=MM_SITTING_STRAIGHT_STATE_HUM_REACH;
   lean_reach_type=MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH;
   turn_reach_state=MM_SITTING_TURN_AROUND_STATE_HUM_REACH;
   turn_lean_reach_type=MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH;
  }
  else
  {
   if(for_posture_state==HRI_STANDING)
   {
   straight_lean_type=MM_STANDING_STRAIGHT_STATE_HUM_REACH;
   lean_reach_type=MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH;
   turn_reach_state=MM_STANDING_TURN_AROUND_STATE_HUM_REACH;
   turn_lean_reach_type=MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH;
   }
  }
  
   int kcd_with_report=0;
  double ref_yaw_ang=  0.0;//hum_tmp_pos[HUMANq_TORSO_PAN];
  double ref_pitch_ang=0.0;//hum_tmp_pos[HUMANq_TORSO_TILT]; 
  double maxi_left_turn=M_PI/2.0;
  double mini_right_turn=-M_PI/2.0;
  double maxi_lean_forward=M_PI/3.0;
  
  double curr_pitch_ang= ref_pitch_ang;
  double curr_yaw_ang= ref_yaw_ang;

  hum_tmp_pos[HUMANq_TORSO_PAN]=0.0;
  hum_tmp_pos[HUMANq_TORSO_TILT]=0.0;
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[hum_index], hum_tmp_pos);
      int collision_occured=0;
      int res = p3d_col_test_robot(envPt_MM->robot[hum_index],kcd_with_report);
	    if(res>0)
	      {
// 		kcd_with_report=0;
// 		res = p3d_col_test_self_collision(envPt_MM->robot[hum_index],kcd_with_report);
// 		if(res>0)
// 		  {
// 		    printf(" There is self collision with human, res=%d \n", res);
// 		    pqp_print_colliding_pair();
// 		    //return 0;
// 		  }
// 		else
// 		  {
		    printf(" **** There is collision with human, for pitch_ang = 0, res=%d \n",  res);
		    pqp_print_colliding_pair();

		    collision_occured=1;
//		  }
	      }
	      if(collision_occured==0)
	      {
	      MA_agent_hand_name for_hand=MA_LEFT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, straight_lean_type);
	      for_hand=MA_RIGHT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, straight_lean_type);
	
	      }
	     //// printf(" Straight reach calculated \n");
	      
	      int lean_forward=1;
	      while(lean_forward==1)
              {
		curr_pitch_ang+=0.25;
		if(curr_pitch_ang>maxi_lean_forward)
		{
		  break;
		}
		
      hum_tmp_pos[HUMANq_TORSO_TILT]=curr_pitch_ang; 
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[hum_index], hum_tmp_pos);
     //// g3d_draw_allwin_active();
      
      ////printf(" Lean reach calculating for lean angle %lf\n",curr_pitch_ang);
      int res = p3d_col_test_robot(envPt_MM->robot[hum_index],kcd_with_report);
	    if(res>0)
	      {
		////printf("Collision found for %lf lean angle\n",curr_pitch_ang);
//		pqp_print_colliding_pair();
// 		kcd_with_report=0;
// 		res = p3d_col_test_self_collision(envPt_MM->robot[hum_index],kcd_with_report);
// 		if(res>0)
// 		  {
// 		    printf(" There is self collision with human, res=%d \n", res);
// 		    pqp_print_colliding_pair();
// 		    //return 0;
// 		  }
// 		else
// 		  {
		    printf(" **** There is collision with human, for pitch_ang = %lf res=%d \n", curr_pitch_ang, res);
		    pqp_print_colliding_pair();
                    lean_forward=0;
		    break;
//		  }
	      }
	      MA_agent_hand_name for_hand=MA_LEFT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, lean_reach_type);
	      for_hand=MA_RIGHT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, lean_reach_type);
	  
	       ////printf(" Lean reach calculated for curr_pitch_ang=%lf\n", curr_pitch_ang);
             }
             
             	     
 ////printf("Now calculating turn around and lean reach\n");
  int turn_left=1;
  
  
  
  int curr_state_type;
  curr_yaw_ang=ref_yaw_ang;
  
  while(turn_left==1)
  {
    curr_yaw_ang+= 0.25;
    if(curr_yaw_ang>maxi_left_turn)
    {
    
      break;
    }
    
    curr_pitch_ang= ref_pitch_ang;
    
    hum_tmp_pos[HUMANq_TORSO_PAN]=curr_yaw_ang;
   
    curr_state_type=turn_reach_state;//This will change to turn_lean_reach_type for lean at the end of below while loop;
  
    int lean_forward=1;
    
    while(lean_forward==1)
    {
      hum_tmp_pos[HUMANq_TORSO_TILT]=curr_pitch_ang; 
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[hum_index], hum_tmp_pos);
      
      ////g3d_draw_allwin_active();
      //printf(" **** testing, for yaw_angle= %lf, pitch_ang = %lf \n", curr_yaw_ang, curr_pitch_ang);
      
      int res = p3d_col_test_robot(envPt_MM->robot[hum_index],kcd_with_report);
	    if(res>0)
	      {
// 		kcd_with_report=0;
// 		res = p3d_col_test_self_collision(envPt_MM->robot[hum_index],kcd_with_report);
// 		if(res>0)
// 		  {
// 		    printf(" There is self collision with human, res=%d \n", res);
// 		    pqp_print_colliding_pair();
// 		    //return 0;
// 		  }
// 		else
// 		  {
		    printf(" **** There is collision with human, for yaw_angle= %lf, pitch_ang = %lf res=%d \n", curr_yaw_ang, curr_pitch_ang, res);
		    pqp_print_colliding_pair();
                    lean_forward=0;
		    break;
//		  }
	      }
	      
	      MA_agent_hand_name for_hand=MA_LEFT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, curr_state_type);
	      for_hand=MA_RIGHT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, curr_state_type);
	      
	      curr_pitch_ang+=0.25;
	      if(curr_pitch_ang>=maxi_lean_forward)
		{
		  lean_forward=0;
		  break;
		}
		
		curr_state_type=turn_lean_reach_type;
	  
      }//End while(lean_forward==1)
    
  }
  
  int turn_right=1;
  curr_yaw_ang=ref_yaw_ang;
  
  while(turn_right==1)
  {
    curr_yaw_ang-= 0.25;
    if(curr_yaw_ang<mini_right_turn)
    {
    
      break;
    }
    
    curr_pitch_ang= ref_pitch_ang;
    
    hum_tmp_pos[HUMANq_TORSO_PAN]=curr_yaw_ang;
   
    curr_state_type=turn_reach_state;//This will change to turn_lean_reach_type for lean at the end of below while loop;
  
    int lean_forward=1;
    
    while(lean_forward==1)
    {
      hum_tmp_pos[HUMANq_TORSO_TILT]=curr_pitch_ang; 
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[hum_index], hum_tmp_pos);
      
      ////g3d_draw_allwin_active();
      ////printf(" **** testing, for yaw_angle= %lf, pitch_ang = %lf \n", curr_yaw_ang, curr_pitch_ang);
      
      int res = p3d_col_test_robot(envPt_MM->robot[hum_index],kcd_with_report);
	    if(res>0)
	      {
// 		kcd_with_report=0;
// 		res = p3d_col_test_self_collision(envPt_MM->robot[hum_index],kcd_with_report);
// 		if(res>0)
// 		  {
// 		    printf(" There is self collision with human, res=%d \n", res);
// 		    pqp_print_colliding_pair();
// 		    //return 0;
// 		  }
// 		else
// 		  {
		    printf(" **** There is collision with human, for yaw_angle= %lf, pitch_ang = %lf res=%d \n", curr_yaw_ang, curr_pitch_ang, res);
		    pqp_print_colliding_pair();
                    lean_forward=0;
		    break;
//		  }
	      }
	      
	      MA_agent_hand_name for_hand=MA_LEFT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, curr_state_type);
	      for_hand=MA_RIGHT_HAND;
	      update_3d_grid_reachability_for_agent_MM(for_agent, for_hand, curr_state_type);
	      
	      curr_pitch_ang+=0.25;
	      if(curr_pitch_ang>=maxi_lean_forward)
		{
		  lean_forward=0;
		  break;
		}
		
		curr_state_type=turn_lean_reach_type;
	  
      }//End while(lean_forward==1)
    
  }
    p3d_destroy_config(envPt_MM->robot[hum_index],hum_tmp_pos);
    p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],envPt_MM->robot[hum_index]);
    MA_activate_collision_between_non_torso_parts_and_env(envPt_MM->robot[hum_index]); 
    return 1;

 

}



int find_reachable_sphere_surface(int for_hand, HRI_TASK_AGENT for_agent)
{

  point_co_ordi shoulder_pos;
  ////point_co_ordi sphere_pt;
  no_sphere_surface_pts=0;
   
  ////int for_hand=1;//1 for left, 2 for right
  int shoulder_indx; 
  if(for_hand==MA_RIGHT_HAND)
  {
    shoulder_indx=agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[RSHOULDER];
    
  }
  else
  {
    if(for_hand==MA_LEFT_HAND)
   {
    shoulder_indx=agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[LSHOULDER];
    
   }
  }
  /*
  if(for_agent==HUMAN1_MA||for_agent==HUMAN2_MA)//for primary human i.e. human1
    {
      if(for_hand==1)//1 for left, 2 for right
	{
	  shoulder_indx=HUMANj_LSHOULDER;
	}
      else
	{
	  if(for_hand==2)
	    {
	      shoulder_indx=HUMANj_RSHOULDER;
	    }
	}
    }
  else
    {
      if(for_agent==HRP2_MA)//for HRP2
	{
	  if(for_hand==1)//1 for left, 2 for right
	    {
	      shoulder_indx=ROBOTj_LSHOULDER;
	    }
	  else
	    {
	      if(for_hand==2)
		{
		  shoulder_indx=ROBOTj_RSHOULDER;
		}
	    }
	}
      else
	{
	  if(for_agent==JIDO_MA)//for JIDO
	    {
   
	      shoulder_indx=ROBOTj_RSHOULDER;
   
	    }

	}  
    }
*/
  double r=0;    
  double curr_yaw=0;

  double shoulder_back_limit;// The maxi possible angle away from the front axis of the sgent
  double shoulder_front_limit;// The maxi possible angle crossing the front axis of the agent
  
   shoulder_pos.x = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[shoulder_indx]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
      shoulder_pos.y = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[shoulder_indx]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
      shoulder_pos.z = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[shoulder_indx]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
      
      curr_yaw=envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[1]->dof_data[5].v;

  switch(for_agent)
  {
    
    case HUMAN1_MA:
     
      r=0.7;//Maximum reach boundary for human
      ////curr_yaw=envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_RZ];   
      //[HUMANq_RZ];   
      shoulder_back_limit=M_PI/2.0+M_PI/6.0;// The maxi possible angle away from the front axis of human
      shoulder_front_limit=M_PI/3.0;// The maxi possible angle crossing the front axis of human
      break;
      
    
      #ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
       
      r=0.7;//Maximum reach boundary for human
      ////curr_yaw=envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_RZ];   
      
      shoulder_back_limit=M_PI/2.0+M_PI/6.0;// The maxi possible angle away from the front axis of human
      shoulder_front_limit=M_PI/3.0;// The maxi possible angle crossing the front axis of human
      break;
      #endif
      
      #ifdef JIDO_EXISTS_FOR_MA
      case JIDO_MA:
	
	      r=1.1;//Maximum reach boundary for JIDO
	      ////curr_yaw=envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[11];   
	    
	      shoulder_back_limit=5.0*M_PI/6.0;// The maxi possible angle away from the front axis of JIDO
	      shoulder_front_limit=4.0*M_PI/6.0;// The maxi possible angle crossing the front axis of JIDO

	break;
      
      #endif
	
#ifdef HRP2_EXISTS_FOR_MA
      case HRP2_MA:
	
	  r=0.7;//Maximum reach boundary for HRP2
	  ////curr_yaw=envPt_MM->robot[rob_indx.HRP2_ROBOT]->ROBOT_POS[11];   
	
	  shoulder_back_limit=M_PI/2.0;// The maxi possible angle away from the front axis of HRP2
	  shoulder_front_limit=M_PI/3.0;// The maxi possible angle crossing the front axis of HRP2
	break;
	#endif
	
#ifdef PR2_EXISTS_FOR_MA
      case PR2_MA:
	
	  r=1.0;//Maximum reach boundary for PR2
	  ////curr_yaw=envPt_MM->robot[rob_indx.HRP2_ROBOT]->ROBOT_POS[11];   
	
	  shoulder_back_limit=M_PI/2.0;// The maxi possible angle away from the front axis of HRP2
	  shoulder_front_limit=M_PI/3.0;// The maxi possible angle crossing the front axis of HRP2
	break;
	#endif
	
  }
  

    
  double theta, phi;
    
  ////printf("curr_yaw=%lf\n",curr_yaw);
  double phi_st, phi_end;
  //phi_st=phi;
    
    
  if(for_hand==MA_RIGHT_HAND)//For right hand
    {
      phi_st=curr_yaw-shoulder_back_limit;
      phi_end=curr_yaw+shoulder_front_limit;
    }
  if(for_hand==MA_LEFT_HAND)
    {
      phi_st=curr_yaw-shoulder_front_limit;
      phi_end= (curr_yaw)+(shoulder_back_limit);
    
    }
  /*
    if(phi_st>2.0*M_PI)
    {
    phi_st=phi_st-2.0*M_PI;
    }*/
   
  double increment=grid_around_HRP2.GRID_SET->pace/r;  
  ////////printf("phi_st=%lf, phi_end=%lf, increment=%lf\n",phi_st,phi_end, increment);
  //// phi=hum_yaw;
     
     
  for(phi=phi_st;phi<=phi_end;phi+=increment)
    { 
      for(theta=0.0;theta<=M_PI;theta+=increment)
	{
	  sphere_surface_pts[no_sphere_surface_pts].x=shoulder_pos.x+r*sin(theta)*cos(phi);
	  sphere_surface_pts[no_sphere_surface_pts].y=shoulder_pos.y+r*sin(theta)*sin(phi);
	  sphere_surface_pts[no_sphere_surface_pts].z=shoulder_pos.z+r*cos(theta);
	  //////// printf("no_sphere_surface_pts =%d\n",no_sphere_surface_pts);
	  no_sphere_surface_pts++;
    
	}
    } 
  ////////printf("no_sphere_surface_pts =%d\n",no_sphere_surface_pts);
  return no_sphere_surface_pts;
}

int update_3d_grid_reachability_for_JIDO_MM()
{

  
  HRI_TASK_AGENT for_agent=JIDO_MA; //1 for human, 2 for HRP2, 3 for Jido
    //////////printf("Inside update_3d_grid_reachability_for_JIDO_new()\n");
  point_co_ordi shoulder_pos;
  ////point_co_ordi sphere_pt;
      
  // Reachability Test by left hand
  ////shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  /////shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    /////shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively

    shoulder_pos.x = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[RSHOULDER]]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[RSHOULDER]]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[RSHOULDER]]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
 
    int for_hand=MA_RIGHT_HAND;//Although JIDO has only one arm, but just to comply with find_reachable_sphere_surface() function, we should always pass RIGHT_HAND
    no_sphere_surface_pts=0;
    int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
    double interval=grid_around_HRP2.GRID_SET->pace/1.5;
    for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
       
	double t=0;
	for(;t<1;t+=interval) 
	  { 
      
	    //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

	    double x2;
	    double y2;
	    double z2;
	    int cell_x;
	    int cell_y;
	    int cell_z;
 
	    x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
	    cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  

	    if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	      {
		y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
		cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 

		if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		  { 
		    z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
		    cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 

		    if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
		      {
			////printf(" Cell (%d, %d, %d) is reachabke by JIDO\n", cell_x,cell_y,cell_z);
			grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable[for_agent][MM_CURRENT_STATE_JIDO_REACH][MA_RIGHT_HAND]=1;   
		      }
		  }
		////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
	      }
	  }

	//g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
      }
   
    return 1;

}

int update_3d_grid_reachability_for_agent_MM(HRI_TASK_AGENT for_agent, MA_agent_hand_name for_hand, int for_state)
{

  MA_agents_hand_joints_names for_shoulder;
  if(for_hand==MA_RIGHT_HAND)
  {
    for_shoulder=RSHOULDER;
  }
  else
  {
   if(for_hand==MA_LEFT_HAND)
   {
    for_shoulder=LSHOULDER;
   }
  }
  ////HRI_TASK_AGENT for_agent=JIDO_MA; //1 for human, 2 for HRP2, 3 for Jido
    //////////printf("Inside update_3d_grid_reachability_for_JIDO_new()\n");
  point_co_ordi shoulder_pos;
  ////point_co_ordi sphere_pt;
      
  // Reachability Test by left hand
  ////shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  /////shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    /////shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively

    shoulder_pos.x = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[for_shoulder]]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.y = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[for_shoulder]]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    shoulder_pos.z = envPt_MM->robot[indices_of_MA_agents[for_agent]]->joints[agents_for_MA_obj.for_agent[for_agent].hand_params.joint_indices[for_shoulder]]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
    ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
 
    ////int for_hand=MA_RIGHT_HAND;//Although JIDO has only one arm, but just to comply with find_reachable_sphere_surface() function, we should always pass RIGHT_HAND
    no_sphere_surface_pts=0;
    int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
    double interval=grid_around_HRP2.GRID_SET->pace/1.5;
    for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
      {
       
	double t=0;
	for(;t<1;t+=interval) 
	  { 
      
	    //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

	    double x2;
	    double y2;
	    double z2;
	    int cell_x;
	    int cell_y;
	    int cell_z;
 
	    x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
	    cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  

	    if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	      {
		y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
		cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 

		if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		  { 
		    z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
		    cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 

		    if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
		      {
			////printf(" Cell (%d, %d, %d) is reachabke by JIDO\n", cell_x,cell_y,cell_z);
			grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable[for_agent][for_state][for_hand]=1;   
		      }
		  }
		////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
	      }
	  }

	//g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
      }
   
    return 1;

}

int update_3d_grid_reachability_for_HRP2_new()
{
#ifndef COMMENT_TMP
  
  int for_agent=2; //1 for human, 2 for HRP2

  

  //////////printf("Inside update_3d_grid_reachability_for_HRP2_new()\n");
  point_co_ordi shoulder_pos;
  if(ACBTSET==NULL)
    {
      printf(" AKP WARNING : ACBTSET==NULL");
    }
  ////point_co_ordi sphere_pt;
  //       printf("ROBOTj_LSHOULDER=%d, ACBTSET->robot->name =%s\n",ROBOTj_LSHOULDER, ACBTSET->robot);
  // Reachability Test by left hand
  /* shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
     ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
     */
  printf(" envPt_MM->robot[rob_indx.HRP2]->name = %s \n", envPt_MM->robot[rob_indx.HRP2_ROBOT]->name);
  shoulder_pos.x = envPt_MM->robot[rob_indx.HRP2_ROBOT]->joints[ROBOTj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  shoulder_pos.y = envPt_MM->robot[rob_indx.HRP2_ROBOT]->joints[ROBOTj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  shoulder_pos.z = envPt_MM->robot[rob_indx.HRP2_ROBOT]->joints[ROBOTj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively

  int for_hand=MA_LEFT_HAND;//1 for left, 2 for right
  no_sphere_surface_pts=0;
  int no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);
    
  double interval=grid_around_HRP2.GRID_SET->pace/1.5;
  for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
    {
      double t=0;
      for(;t<1;t+=interval) 
	{ 
      
	  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

	  double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
	  double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
	  double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
	  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
	  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	    {
       
	      int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
	      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		{
		  int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
       
		  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)  
		    {
		      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_LHand=1;   
		    }
		}  ////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
	    }
	}
      //g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
    }
   
    
  // Reachability Test by right hand
  shoulder_pos.x = envPt_MM->robot[rob_indx.HRP2_ROBOT]->joints[ROBOTj_RSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  shoulder_pos.y = envPt_MM->robot[rob_indx.HRP2_ROBOT]->joints[ROBOTj_RSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  shoulder_pos.z = envPt_MM->robot[rob_indx.HRP2_ROBOT]->joints[ROBOTj_RSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
  for_hand=MA_RIGHT_HAND;//1 for left, 2 for right
  no_sphere_surface_pts=0;
  no_of_sph_surf_pts=find_reachable_sphere_surface(for_hand, for_agent);

  for(int sp_ctr=0;sp_ctr<no_of_sph_surf_pts;sp_ctr++)
    {
      double t=0;
      for(;t<1;t+=interval) 
	{ 
      
	  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

	  double x2=(1-t)*shoulder_pos.x+t*sphere_surface_pts[sp_ctr].x;
	  double y2=(1-t)*shoulder_pos.y+t*sphere_surface_pts[sp_ctr].y;
	  double z2=(1-t)*shoulder_pos.z+t*sphere_surface_pts[sp_ctr].z; 
	  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
	  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	    {
       
	      int cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
	      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		{
		  int cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
       
		  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)  
		    {
		      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_RHand=1;   
		    }
		}  ////g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
	    }  
     
	  //g3d_drawDisc(x2, y2, z2, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
	}
      //// g3d_drawDisc(sphere_pts[sp_ctr].x, sphere_pts[sp_ctr].y, sphere_pts[sp_ctr].z, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
    }
 
  return 1;

 #endif

}


int find_3D_grid_visibility_for_MM(HRI_AGENT *agent,HRI_TASK_AGENT agent_type, int visibility_type)
{

  
  ////envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
  int nr = envPt_MM->nr;

  //////////printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=agent->perspective->camjoint->abs_pos[0][3];
  eye_pos.y=agent->perspective->camjoint->abs_pos[1][3];
  eye_pos.z=agent->perspective->camjoint->abs_pos[2][3];


  agent_eye_pos.x=eye_pos.x;       
  agent_eye_pos.y=eye_pos.y;
  agent_eye_pos.z=eye_pos.z;
  

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  ////double interval=grid_around_HRP2.GRID_SET->pace*3.0/4.0;
  //////////printf("interval=%lf, no_FOV_end_point_vertices=%d\n",interval,no_FOV_end_point_vertices);
 
  int visible_ctr=0;  
  int not_visible_ctr=0;
  double x2;
  double y2;
  double z2;
  int cell_x;  
  int cell_y;
  int cell_z;

  for(int i=0;i<no_points_on_FOV_screen;i++)
    {
      int obs_found=0;
      int first_obs_cell=0;
      double t2=0.05;//Just to avoid very checking very near to human
      ////g3d_drawDisc(points_on_FOV_screen[i][0],points_on_FOV_screen[i][1],points_on_FOV_screen[i][2],0.05,Red,NULL);
      for(;t2<1;t2+=interval) 
	{ 
      
	  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

	  x2=(1-t2)*eye_pos.x+t2*points_on_FOV_screen[i][0];
	  y2=(1-t2)*eye_pos.y+t2*points_on_FOV_screen[i][1];
	  z2=(1-t2)*eye_pos.z+t2*points_on_FOV_screen[i][2]; 


	  cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       
	  int cell_valid=0;       

	  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	    {
      
 
	      cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
	      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		{
		  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
		  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
		    {
		      ////curr_cell=
		      ////curr_cell->x=cell_x;
		      ////curr_cell->y=cell_y;
		      ////curr_cell->z=cell_z;
		      cell_valid=1;
		    } 
		}
	    }

	  if(cell_valid==1)
	    {
	      int is_visible=1; 
        
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
		{
		  is_visible=0;
		  obs_found=1;
		  if(first_obs_cell==0)
		    {
		      first_obs_cell=1;
		      ////This is the first obstacle cell found in the ray
		       grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible[agent_type][visibility_type]=1;
		      

		    }

		  //printf(" Not Visible ");
		  ////not_visible_ctr++;

          
		}//end if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells

	      if(obs_found==0&&is_visible==1)
		{
		  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible[agent_type][visibility_type]=1;
		  
		 
		  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
		  ////visible_ctr++;
		  //g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
		} 
	    }//end if(cell_valid==1)
	}//End for(;t2<1;t2+=interval) 
    }

  return 1;
}


/*
  int update_3d_grid_visibility(int type)//1 means human, 2 means HRP2, 3 means JIDO, 4 means second human
  {

  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
  int nr = envPt_MM->nr;

  //////////printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
  configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
  hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
  hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
  hum_head_pos[8] = eye_pos.z;
 
  if(type==3)
  {
  agent_eye_pos.x=eye_pos.x;       
  agent_eye_pos.y=eye_pos.y;
  agent_eye_pos.z=eye_pos.z;
  }

  configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  //////////printf("interval=%lf, no_FOV_end_point_vertices=%d\n",interval,no_FOV_end_point_vertices);
 
  int visible_ctr=0;  
  int not_visible_ctr=0;
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
  double t=0;

  for(;t<1;t+=interval)
  {
      
  double x;
  double y;
  double z;
      
  if(i<7)
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
  } 
  else
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
  }
  ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
  int obs_found=0;
  int first_obs_cell=0;
  double t2=0.1;//Just to avoid very checking very near to human
      
  for(;t2<1;t2+=interval) 
  { 
      
  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

  double x2=(1-t2)*eye_pos.x+t2*x;
  double y2=(1-t2)*eye_pos.y+t2*y;
  double z2=(1-t2)*eye_pos.z+t2*z; 
      
  ////  double x2=(1-t2)*x+t*eye_pos.x;
  ////    double y2=(1-t2)*y+t*eye_pos.y;
  ////    double z2=(1-t2)*z+t*eye_pos.z; 
  
  //////////hri_bitmap_cell* curr_cell=NULL;
  
  ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

  //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
  int cell_y;
  int cell_z;

  int cell_valid=0;       

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
      
 
  cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {
  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  {
  ////curr_cell=
  ////curr_cell->x=cell_x;
  ////curr_cell->y=cell_y;
  ////curr_cell->z=cell_z;
  cell_valid=1;
  } 
  }
  }

  ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

  ////////if(curr_cell!=NULL)
  if(cell_valid==1)
  { 
        
  ////if(type==1)//means for human
  //// {
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=0;
  //// }
  ////else
  //// {
  ////if(type==2)//means for HRP2
  ////  {
  ////  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=0;
  ////  }
  ////else
  ////  {
  ////   if(type==3)//means for JIDO
  ////  {
  ////  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=0;
  ////  } 
  //// }
  //// }
         

  int is_visible=1; 
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human=0;

  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
  {
  ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
  ////////is_visible=0;
 

  //AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
  //           
  //          if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
  //           {
  // 
  //           point_to_look[6] = x2;
  //           point_to_look[7] = y2;
  //           point_to_look[8] = z2;
  //           
  //           //Using perspective taking function
  //           if(type==1) //for human
  //           {
  //           p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //           }
  //           else
  //           {
  //            if(type==2) //for HRP2
  //            {
  //           p3d_rob * currRob=ACBTSET->robot;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //            } 
  //           }
  //           
  //           //Using collison test by local path method  
  //           //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
  //           
  //           if(is_visible==1)
  //           {  
  //           }
  //           else
  //           {
  //           is_visible=0;
  //           ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
//           }
//           }
//           //END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
//           else
////////{
is_visible=0;
obs_found=1;
if(first_obs_cell==0)
{
first_obs_cell=1;
////This is the first obstacle cell found in the ray
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human=1;
} 

if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2=1;
} 

if(type==3)//means for JIDO
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO=1;
} 
  
#ifdef HUMAN2_EXISTS_FOR_MA
if(type==4)//means for second human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2=1;
} 
#endif

}

//printf(" Not Visible ");
not_visible_ctr++;
////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
////////}

           
         
// //          //, so assume that the object corresponding to this obstacle is visible
// //           
// //           int i=0;
// //           for(i=0;i<nr;i++)
// //           {
// //            
// //        if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.belongs_to_objects_indx[i]==1)
// //            {
// //             object_MM.object[i].visible_by_human=1;
// //            }
// //           }
          

}
//else
if(obs_found==0&&is_visible==1)
{
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==3)//means for JIDO
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=1;
}
#ifdef HUMAN2_EXISTS_FOR_MA
else
{
if(type==4)//means for second human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2=1;
}
}  
#endif 
           
         
}
} 
////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
visible_ctr++;
//g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
} 
}
}
}
//}

}
}
  
p3d_destroy_config(ACBTSET->visball, hum_head_pos);
p3d_destroy_config(ACBTSET->visball, point_to_look);   
//////////printf(" Visible_ctr=%d, not_visible_ctr=%d\n",visible_ctr,not_visible_ctr); 
return 1;
}
*/
/*
  int update_3d_grid_straight_visibility(int type)//1 means human, 2 means HRP2, 3 means JIDO, 4 for human2
  {
  //////////printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
  configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
  hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
  hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
  hum_head_pos[8] = eye_pos.z;
 
  if(type==3)
  {
  agent_eye_pos.x=eye_pos.x;       
  agent_eye_pos.y=eye_pos.y;
  agent_eye_pos.z=eye_pos.z;
  }

  configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  //////////printf("interval=%lf, no_FOV_end_point_vertices=%d\n",interval,no_FOV_end_point_vertices);
 
  int visible_ctr=0;  
  int not_visible_ctr=0;
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
  double t=0;

  for(;t<1;t+=interval)
  {
      
  double x;
  double y;
  double z;
      
  if(i<7)
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
  } 
  else
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
  }
  ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
  int obs_found=0;
  int first_obs_cell=0;
  double t2=0.1;//Just to avoid very checking very near to human
      
  for(;t2<1;t2+=interval) 
  { 
      
  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

  double x2=(1-t2)*eye_pos.x+t2*x;
  double y2=(1-t2)*eye_pos.y+t2*y;
  double z2=(1-t2)*eye_pos.z+t2*z; 
      
  //       double x2=(1-t2)*x+t*eye_pos.x;
  //       double y2=(1-t2)*y+t*eye_pos.y;
  //       double z2=(1-t2)*z+t*eye_pos.z;
  
  //////////hri_bitmap_cell* curr_cell=NULL;
  
  ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

  //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
  int cell_y;
  int cell_z;

  int cell_valid=0;       

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
      
 
  cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {
  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  {
  ////curr_cell=
  ////curr_cell->x=cell_x;
  ////curr_cell->y=cell_y;
  ////curr_cell->z=cell_z;
  cell_valid=1;
  } 
  }
  }

  ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

  ////////if(curr_cell!=NULL)
  if(cell_valid==1)
  { 
        
  //         if(type==1)//means for human
  //          {
  //         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=0;
  //          }
  //         else
  //          {
  //         if(type==2)//means for HRP2
  //           {
  //           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;
  //           }
  //         else
  //           {
  //            if(type==3)//means for JIDO
  //            {
  //            grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;
  //            } 
  //           }
  //          }
         

  int is_visible=1; 
  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
  {
  ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
  ////////is_visible=0;
 

  //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          
  //          if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
  //           {
  // 
  //           point_to_look[6] = x2;
  //           point_to_look[7] = y2;
  //           point_to_look[8] = z2;
  //           
  //           Using perspective taking function
  //           if(type==1) //for human
  //           {
  //           p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //           }
  //           else
  //           {
  //            if(type==2) //for HRP2
  //            {
  //           p3d_rob * currRob=ACBTSET->robot;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //            } 
  //           }
  //           
  //           Using collison test by local path method  
  //           is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
  //           
  //           if(is_visible==1)
  //           {  
  //           }
  //           else
  //           {
  //           is_visible=0;
  //           t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
//           }
//           }
//           ****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
//           else

////////{
is_visible=0;
obs_found=1;
//printf(" Not Visible ");
 
if(first_obs_cell==0)
{
first_obs_cell=1;
////This is the first obstacle cell found in the ray
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation=1;
} 

if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_straight_head_orientation=1;
} 

if(type==3)//means for JIDO
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO_straight_head_orientation=1;
} 
#ifdef HUMAN2_EXISTS_FOR_MA
if(type==4)//means for human2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_straight_head_orientation=1;
} 
#endif
}

not_visible_ctr++;
////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
////////}
}
//else
if(obs_found==0&&is_visible==1)
{
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==3)//means for JIDO
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=1;
}
#ifdef HUMAN2_EXISTS_FOR_MA
else
{
if(type==4)//means for human2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_straight_head_orientation=1;
}
}
#endif  
}
} 
////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
visible_ctr++;
//g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
} 
}
}
}
//}

}
}
  
p3d_destroy_config(ACBTSET->visball, hum_head_pos);
p3d_destroy_config(ACBTSET->visball, point_to_look);   
//////////printf(" Visible_ctr=%d, not_visible_ctr=%d\n",visible_ctr,not_visible_ctr); 
return 1;
}
*/
/*
  int update_3d_grid_visibility_standing(int type)//1 means human, 2 means HRP2
  {
  //////////printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
  configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
  hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
  hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
  hum_head_pos[8] = eye_pos.z;
    
  configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  //////////printf("interval=%lf\n",interval);
 
  int visible_ctr=0;  
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
  double t=0;

  for(;t<1;t+=interval)
  {
      
  double x;
  double y;
  double z;
      
  if(i<7)
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
  } 
  else
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
  }
  ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
  int obs_found=0;
  int first_obs_cell=0;

  double t2=0.1;//Just to avoid very checking very near to human
  for(;t2<1;t2+=interval) 
  { 
      
  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

  double x2=(1-t2)*eye_pos.x+t2*x;
  double y2=(1-t2)*eye_pos.y+t2*y;
  double z2=(1-t2)*eye_pos.z+t2*z; 
      
  //     double x2=(1-t2)*x+t*eye_pos.x;
  //       double y2=(1-t2)*y+t*eye_pos.y;
  //       double z2=(1-t2)*z+t*eye_pos.z; 
  //   
  //////////hri_bitmap_cell* curr_cell=NULL;
  
  ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

  //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
  int cell_y;
  int cell_z;

  int cell_valid=0;       

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
      
 
  cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {
  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  {
  ////curr_cell=
  ////curr_cell->x=cell_x;
  ////curr_cell->y=cell_y;
  ////curr_cell->z=cell_z;
  cell_valid=1;
  } 
  }
  }

  ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

  ////////if(curr_cell!=NULL)
  if(cell_valid==1)
  { 
        
  //         if(type==1)//means for human
  //          {
  //         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=0;
  //          }
  //         else
  //          {
  //         if(type==2)//means for HRP2
  //           {
  //           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2=0;
  //           }
  //          }
        
  int is_visible=1; 
  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
  {
  ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
  ////////is_visible=0;
 

  //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          
  //          if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
  //           {
  // 
  //           point_to_look[6] = x2;
  //           point_to_look[7] = y2;
  //           point_to_look[8] = z2;
  //           
  //           //Using perspective taking function
  //           if(type==1) //for human
  //           {
  //           p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //           }
  //           else
  //           {
  //            if(type==2) //for HRP2
  //            {
  //           p3d_rob * currRob=ACBTSET->robot;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //            } 
  //           }
  //           
  //           //Using collison test by local path method  
  //           //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
  //           
  //           if(is_visible==1)
  //           {  
  //           }
  //           else
  //           {
  //           is_visible=0;
  //           ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
//           }
//           }
//           //****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
//           else
////////{
obs_found=1;
is_visible=0;

          
if(first_obs_cell==0)
{
first_obs_cell=1;
////This is the first obstacle cell found in the ray
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human=1;
} 

if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_HRP2=1;
} 

}
////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
////////}
}
//else
if(obs_found==0&&is_visible==1)
{
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
} 
////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
visible_ctr++;
//g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
} 
}
}
}
//}

}
}
  
p3d_destroy_config(ACBTSET->visball, hum_head_pos);
p3d_destroy_config(ACBTSET->visball, point_to_look);   
//////////printf(" Visible_ctr=%d\n",visible_ctr); 
return 1;
}
*/

/*
  int update_3d_grid_visibility_by_neck_turn(int type)//1 means human, 2 means HRP2, 3 means JIDO, 4 for human2
  {
  //////////printf(" **** Inside update_3d_grid_visibility_by_neck_turn() for %d,  no_FOV_end_point_vertices=%d\n",type,no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
  configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
  hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
  hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
  hum_head_pos[8] = eye_pos.z;
    
  configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  //////////printf("interval=%lf\n",interval);
 
  int visible_ctr=0;  
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
  double t=0;

  for(;t<1;t+=interval)
  {
      
  double x;
  double y;
  double z;
      
  if(i<7)
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
  } 
  else
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
  }
  ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
  int obs_found=0;
  int first_obs_cell=0;
  double t2=0.1;//Just to avoid very checking very near to human
  for(;t2<1;t2+=interval) 
  { 
      
  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

  double x2=(1-t2)*eye_pos.x+t2*x;
  double y2=(1-t2)*eye_pos.y+t2*y;
  double z2=(1-t2)*eye_pos.z+t2*z; 
      
  //     double x2=(1-t2)*x+t*eye_pos.x;
  //       double y2=(1-t2)*y+t*eye_pos.y;
  //       double z2=(1-t2)*z+t*eye_pos.z; 
  
  //////////hri_bitmap_cell* curr_cell=NULL;
  
  ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

  //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
  //////////if(curr_cell!=NULL)
  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
  int cell_y;
  int cell_z;

  int cell_valid=0;       

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
      
 
  cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {
  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  {
  ////curr_cell=
  ////curr_cell->x=cell_x;
  ////curr_cell->y=cell_y;
  ////curr_cell->z=cell_z;
  cell_valid=1;
  } 
  }
  }

  ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

  ////////if(curr_cell!=NULL)
  if(cell_valid==1)
  { 
        
  //         if(type==1)//means for human
  //          {
  //         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=0;
  //          }
  //         else
  //          {
  //         if(type==2)//means for HRP2
  //           {
  //           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=0;
  //           }
  //         else
  //           //{
  //            if(type==3)//means for JIDO
  //            {
  //           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=0;
  //            }
  //           //}
  //          }

  int is_visible=1; 
  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
  {
  ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
  //////////is_visible=0;
 

  //AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          
  //          if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
  //           {
  // 
  //           point_to_look[6] = x2;
  //           point_to_look[7] = y2;
  //           point_to_look[8] = z2;
  //           
  //           Using perspective taking function
  //           if(type==1) //for human
  //           {
  //           p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //           }
  //           else
  //           {
  //            if(type==2) //for HRP2
  //            {
  //           p3d_rob * currRob=ACBTSET->robot;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //            } 
  //           }
  //           
  //           Using collison test by local path method  
  //           is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
  //           
  //           if(is_visible==1)
  //           {  
  //           }
  //           else
  //           {
  //           is_visible=0;
  //           t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
//           }
//           }
//           ****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
//           else
////////{
is_visible=0;
obs_found=1;

if(first_obs_cell==0)
{
first_obs_cell=1;
////This is the first obstacle cell found in the ray
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_neck_turn=1;
} 

if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_neck_turn=1;
} 

if(type==3)//means for JIDO
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO_neck_turn=1;
////printf(" ************ first_non_visible_by_JIDO_neck_turn \n");
} 
#ifdef HUMAN2_EXISTS_FOR_MA
if(type==4)//means for human2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_neck_turn=1;
} 
#endif
}
////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
////////}
}
//else
if(obs_found==0&&is_visible==1)
{
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==3)//means for JIDO
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=1;
}
#ifdef HUMAN2_EXISTS_FOR_MA
else
{
if(type==4)//means for human2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_neck_turn=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
}
#endif
}
} 
////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
visible_ctr++;
//g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
} 
}
}

}
//}

}
}
  
p3d_destroy_config(ACBTSET->visball, hum_head_pos);
p3d_destroy_config(ACBTSET->visball, point_to_look);   
return 1;
//////////printf(" Visible_ctr=%d\n",visible_ctr); 
}
*/

/*
  int update_3d_grid_straight_visibility_standing(int type)//1 means human, 2 means HRP2
  {
  //////////printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
  configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
  hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
  hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
  hum_head_pos[8] = eye_pos.z;
    
  configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  //////////printf("interval=%lf\n",interval);
 
  int visible_ctr=0;  
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
  double t=0;

  for(;t<1;t+=interval)
  {
      
  double x;
  double y;
  double z;
      
  if(i<7)
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
  } 
  else
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
  }
  ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
  int obs_found=0;
  int first_obs_cell=0;
  double t2=0.1;//Just to avoid very checking very near to human
  for(;t2<1;t2+=interval) 
  { 
      
  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

  double x2=(1-t2)*eye_pos.x+t2*x;
  double y2=(1-t2)*eye_pos.y+t2*y;
  double z2=(1-t2)*eye_pos.z+t2*z; 
      
  //     double x2=(1-t2)*x+t*eye_pos.x;
  //       double y2=(1-t2)*y+t*eye_pos.y;
  //       double z2=(1-t2)*z+t*eye_pos.z; 
  
  //////////hri_bitmap_cell* curr_cell=NULL;
  
  ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

  //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
  //////////if(curr_cell!=NULL)
  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
  int cell_y;
  int cell_z;

  int cell_valid=0;       

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
      
 
  cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {
  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  {
  ////curr_cell=
  ////curr_cell->x=cell_x;
  ////curr_cell->y=cell_y;
  ////curr_cell->z=cell_z;
  cell_valid=1;
  } 
  }
  }

  ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

  ////////if(curr_cell!=NULL)
  if(cell_valid==1)
  { 
        
  //         if(type==1)//means for human
  //          {
  //         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;
  //          }
  //         else
  //          {
  //         if(type==2)//means for HRP2
  //           {
  //           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation_standing=0;
  //           }
  //          }
        
  int is_visible=1; 
  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
  {
  ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
  //////////is_visible=0;
 

  //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          
  //          if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
  //           {
  // 
  //           point_to_look[6] = x2;
  //           point_to_look[7] = y2;
  //           point_to_look[8] = z2;
  //           
  //           //Using perspective taking function
  //           if(type==1) //for human
  //           {
  //           p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //           }
  //           else
  //           {
  //            if(type==2) //for HRP2
  //            {
  //           p3d_rob * currRob=ACBTSET->robot;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //            } 
  //           }
  //           
  //           //Using collison test by local path method  
  //           //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
  //           
  //           if(is_visible==1)
  //           {  
  //           }
  //           else
  //           {
  //           is_visible=0;
  //           ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
//           }
//           }
//****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
//  else
////////{
obs_found=1;
is_visible=0;

if(first_obs_cell==0)
{
first_obs_cell=1;
////This is the first obstacle cell found in the ray
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation_standing=1;
} 

if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_straight_head_orientation_standing=1;
} 

}
////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
////////}
}
//else
if(obs_found==0&&is_visible==1)
{
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
} 
////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
visible_ctr++;
//g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
} 
}
}
}
//}

}
}
  
p3d_destroy_config(ACBTSET->visball, hum_head_pos);
p3d_destroy_config(ACBTSET->visball, point_to_look);   
//////////printf(" Visible_ctr=%d\n",visible_ctr); 
return 1;
}
*/
/*
  int update_3d_grid_visibility_by_neck_turn_standing(int type)//1 means human, 2 means HRP2
  {
  //////////printf(" Inside update_3d_grid_visibility(),  no_FOV_end_point_vertices=%d\n",no_FOV_end_point_vertices);
  point_co_ordi eye_pos;
   
  eye_pos.x=(FOV_end_point_vertices[0][0].x+FOV_end_point_vertices[0][1].x+FOV_end_point_vertices[0][2].x+FOV_end_point_vertices[0][3].x)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.y=(FOV_end_point_vertices[0][0].y+FOV_end_point_vertices[0][1].y+FOV_end_point_vertices[0][2].y+FOV_end_point_vertices[0][3].y)/4.0;//Average of the 4 vertices of the near screen
  eye_pos.z=(FOV_end_point_vertices[0][0].z+FOV_end_point_vertices[0][1].z+FOV_end_point_vertices[0][2].z+FOV_end_point_vertices[0][3].z)/4.0;//Average of the 4 vertices of the near screen

    
  configPt hum_head_pos  = p3d_get_robot_config(ACBTSET->visball);  // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
  hum_head_pos[6] = eye_pos.x;//hum_cur_pos[6];
  hum_head_pos[7] = eye_pos.y;//hum_cur_pos[7];
  hum_head_pos[8] = eye_pos.z;
    
  configPt point_to_look = p3d_get_robot_config(ACBTSET->visball);  
    

  double interval=grid_around_HRP2.GRID_SET->pace/2.0;
  //////////printf("interval=%lf\n",interval);
 
  int visible_ctr=0;  
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  //// printf(" j=%d, i=%d\n",j,i);
  double t=0;

  for(;t<1;t+=interval)
  {
      
  double x;
  double y;
  double z;
      
  if(i<7)
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][i+1].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][i+1].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][i+1].z;
  } 
  else
  {
  x=(1-t)*FOV_end_point_vertices[j][i].x+t*FOV_end_point_vertices[j][4].x;
  y=(1-t)*FOV_end_point_vertices[j][i].y+t*FOV_end_point_vertices[j][4].y;
  z=(1-t)*FOV_end_point_vertices[j][i].z+t*FOV_end_point_vertices[j][4].z;
  }
  ////g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" x,y,z=(%lf,%lf,%lf)\n",x,y,z);
  int obs_found=0;
  int first_obs_cell=0;
  double t2=0.1;//Just to avoid very checking very near to human
  for(;t2<1;t2+=interval) 
  { 
      
  //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

  double x2=(1-t2)*eye_pos.x+t2*x;
  double y2=(1-t2)*eye_pos.y+t2*y;
  double z2=(1-t2)*eye_pos.z+t2*z; 
      
  //     double x2=(1-t2)*x+t*eye_pos.x;
  //       double y2=(1-t2)*y+t*eye_pos.y;
  //       double z2=(1-t2)*z+t*eye_pos.z; 
  
  //////////hri_bitmap_cell* curr_cell=NULL;
  
  ////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[BT_AFFORDANCE_VISIBILITY], x2, y2, z2);

  //////////curr_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], x2, y2, z2);
  
  //////////if(curr_cell!=NULL)
  int cell_x=(x2- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
  int cell_y;
  int cell_z;

  int cell_valid=0;       

  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
  {
      
 
  cell_y=(y2- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
  {
  cell_z=(z2- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
  {
  ////curr_cell=
  ////curr_cell->x=cell_x;
  ////curr_cell->y=cell_y;
  ////curr_cell->z=cell_z;
  cell_valid=1;
  } 
  }
  }

  ////printf(" curr_cell (%d, %d, %d)\n",cell_x,cell_y,cell_z);

  ////////if(curr_cell!=NULL)
  if(cell_valid==1)
  { 
         
  //         if(type==1)//means for human
  //          {
  //         grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=0;
  //          }
  //         else
  //          {
  //         if(type==2)//means for HRP2
  //           {
  //           grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2_neck_turn=0;
  //           }
  }
         
  int is_visible=1; 
  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].val==-1)//Exact obstacle cells
  {
  ////g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
  ////printf(" point (%lf,%lf,%lf) of which the cell (%d,%d,%d) is not empty and the cell value is %lf \n",x2,y2,z2,curr_cell->x,curr_cell->y,curr_cell->z,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].val);
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1;
  //////////is_visible=0;
 

  //******AKP NOTE :Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
          
  //          if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].is_horizontal_surface==1)
  //           {
  // 
  //           point_to_look[6] = x2;
  //           point_to_look[7] = y2;
  //           point_to_look[8] = z2;
  //           
  //           //Using perspective taking function
  //           if(type==1) //for human
  //           {
  //           p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //           }
  //           else
  //           {
  //            if(type==2) //for HRP2
  //            {
  //           p3d_rob * currRob=ACBTSET->robot;
  //           p3d_set_and_update_this_robot_conf(ACBTSET->visball, point_to_look);
  //           is_visible=psp_is_object_visible(currRob, ACBTSET->visball, 50);
  //            } 
  //           }
  //           
  //           //Using collison test by local path method  
  //           //is_visible=is_point_visible_by_st_line(hum_head_pos, point_to_look, type); 
  //           
  //           if(is_visible==1)
  //           {  
  //           }
  //           else
  //           {
  //           is_visible=0;
  //           ////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
//           }
//           }
//****** END Check for more expensive test, because we are not using directed BB so many cell unnecessary becomes obstacle cell
//         else
////////{
obs_found=1;
is_visible=0;

if(first_obs_cell==0)
{
first_obs_cell=1;
////This is the first obstacle cell found in the ray
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human_neck_turn=1;
} 

if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human_neck_turn=1;
} 

}
////////////t2=1000;// Break the ray beyond this, because the first obstacle has been found, so human can't see beyond this point.
////////}
}
//else
if(obs_found==0&&is_visible==1)
{
if(type==1)//means for human
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=1;//NOTE :In EARLIER version affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
else
{
if(type==2)//means for HRP2
{
grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_HRP2_neck_turn=1;//NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
}
} 
////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[curr_cell->x][curr_cell->y][curr_cell->z].affordances.visible_by_human=1; //NOTE : In EARLIER version  affordance values like visibility, reachability etc will be set in HRP2_GIK_MANIP bitmap, only for checking the obstacle values for visibility,BT_AFFORDANCE_VISIBILITY bitmap will be used.
visible_ctr++;
//g3d_drawDisc(x2,y2,z2,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);
} 
}
}
}
//}

}
}
  
p3d_destroy_config(ACBTSET->visball, hum_head_pos);
p3d_destroy_config(ACBTSET->visball, point_to_look);   
//////////printf(" Visible_ctr=%d\n",visible_ctr); 
return 1;
}

*/



/*
  int update_human_state_new(int state) //1 means sitting 0 means standing
  {
 
  printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
  if(human_state_updated==0)
  { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  configPt config;
  config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
  hri_human * human=ACBTSET->human[ACBTSET->actual_human];
  ////int state=1;
 
  //  q[8] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c7;
  //  q[43] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c1;
  //  q[44] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c2;
  //  q[46] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c3;
  //  q[47] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c4;
  //  q[50] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c5;
  //  q[53] = ACBTSET->human[ACBTSET->actual_human]->state[new_state].c6;
     
  if(strcasestr(human->HumanPt->name,"superman"))
  {
  config[8] = human->state[state].c7;
  config[43] = human->state[state].c1;
  config[44] = human->state[state].c2;
  config[46] = human->state[state].c3;
  config[47] = human->state[state].c4;
  config[50] = human->state[state].c5;
  config[53] = human->state[state].c6;
  // Right Hand 
  config[66] = config[6] + cos(config[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
  config[67] = config[7] + sin(config[11]-0.4)*0.5;
  config[68] = config[68]-0.34+0.1;
  // Left Hand 
  config[72] = config[6] + cos(config[11]+0.4)*0.5;
  config[73] = config[7] + sin(config[11]+0.4)*0.5;
  config[74] = config[74]-0.34+0.1;
  }
  else
  {
  if(strcasestr(human->HumanPt->name,"achile"))
  {
   
  config[8] = human->state[state].c7;
  config[32] = human->state[state].c3;
  config[35] = human->state[state].c4;
  config[39] = human->state[state].c1;
  config[42] = human->state[state].c2;
  }   
  } 
  human->actual_state=state;
  ////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);

  p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
  ACBTSET->changed = TRUE;

	
  ////g3d_draw_env();
  ////fl_check_forms();
  ////g3d_draw_allwin_active();
  ////g3d_draw_allwin_active();
  human_state_updated=0;
  }
 
  printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);

  }
*/



int virtually_update_human_state_new(p3d_rob *for_agent, int state) //1 means sitting 0 means standing
{
 
  //////////printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
  //////////if(human_state_updated==0)
    { //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
      ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
      //////////configPt config;
      //////////config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
      ////ACBTSET->actual_human=0;
      ///////config = p3d_copy_config(human_agent->robotPt, human_agent->robotPt->ROBOT_POS);
      p3d_get_robot_config_into(for_agent,&HUMAN_curr_pos_for_state_change);
      ////////////hri_human * human=hri_bt_create_human(human_agent->robotPt);
      //////human->HumanPt=human_agent->robotPt;
      ////int state=1;
 
      if(strcasestr(hri_human_MM->HumanPt->name,"superman"))
	{
	  HUMAN_curr_pos_for_state_change[8] = hri_human_MM->state[state].c7;
	  HUMAN_curr_pos_for_state_change[43] = hri_human_MM->state[state].c1;
	  HUMAN_curr_pos_for_state_change[44] = hri_human_MM->state[state].c2;
	  HUMAN_curr_pos_for_state_change[46] =hri_human_MM->state[state].c3;
	  HUMAN_curr_pos_for_state_change[47] = hri_human_MM->state[state].c4;
	  HUMAN_curr_pos_for_state_change[50] = hri_human_MM->state[state].c5;
	  HUMAN_curr_pos_for_state_change[53] = hri_human_MM->state[state].c6;
	  // Right Hand 
	  HUMAN_curr_pos_for_state_change[66] = HUMAN_curr_pos_for_state_change[6] + cos(HUMAN_curr_pos_for_state_change[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
	  HUMAN_curr_pos_for_state_change[67] = HUMAN_curr_pos_for_state_change[7] + sin(HUMAN_curr_pos_for_state_change[11]-0.4)*0.5;
	  HUMAN_curr_pos_for_state_change[68] = HUMAN_curr_pos_for_state_change[68]-0.34+0.1;
	  // Left Hand 
	  HUMAN_curr_pos_for_state_change[72] = HUMAN_curr_pos_for_state_change[6] + cos(HUMAN_curr_pos_for_state_change[11]+0.4)*0.5;
	  HUMAN_curr_pos_for_state_change[73] = HUMAN_curr_pos_for_state_change[7] + sin(HUMAN_curr_pos_for_state_change[11]+0.4)*0.5;
	  HUMAN_curr_pos_for_state_change[74] = HUMAN_curr_pos_for_state_change[74]-0.34+0.1;
	}
      else
	{
	  if(strcasestr(hri_human_MM->HumanPt->name,"achile"))
	    {
	      ////printf("Updating achile human state\n");
	      HUMAN_curr_pos_for_state_change[8] = hri_human_MM->state[state].c7+0.05;
	      HUMAN_curr_pos_for_state_change[33] = hri_human_MM->state[state].c3;
	      HUMAN_curr_pos_for_state_change[35] = hri_human_MM->state[state].c4;
	      HUMAN_curr_pos_for_state_change[40] = hri_human_MM->state[state].c1;
	      HUMAN_curr_pos_for_state_change[42] = hri_human_MM->state[state].c2;
	    }   
	} 
      /////////////////hri_human_MM->actual_state=state;
      
      ////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

      //////////p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
      p3d_set_and_update_this_robot_conf(for_agent,HUMAN_curr_pos_for_state_change);
      //////////p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
      //////////p3d_destroy_config(human_agent->robotPt,config);
      ////////////hri_bt_destroy_human(human);
      ////////////ACBTSET->changed = TRUE;

      /*if(BTSET!=NULL)
	hri_bt_refresh_all(BTSET);
	if(INTERPOINT!=NULL){
	hri_bt_3drefresh_all(INTERPOINT);
	}*/
      ////////g3d_draw_env();
      ////////fl_check_forms();
      ////////g3d_draw_allwin_active();
      ////g3d_draw_allwin_active();
      //////////human_state_updated=0;
    }
    //// g3d_draw_allwin_active();

  //////////printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
  return 1;
}

int virtually_update_non_primary_human_state(int state, int hum_index) //1 means sitting 0 means standing, hum_index is the index of robot in environment (envPt)
{
  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  configPt hum_cur_pos = MY_ALLOC(double,envPt_MM->robot[hum_index]->nb_dof); /* Allocation of temporary robot configuration */

  p3d_get_robot_config_into(envPt_MM->robot[hum_index],&hum_cur_pos);

  //////////printf("Before updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
  //ACBTSET->human[ACBTSET->actual_human]->actual_state = 1; //// AKP Note: Comment it if using motion capture button
  ////update_human_state(1);//For sitting //// AKP Note: Comment it if using motion capture button
  //////configPt config;
  //////config = p3d_copy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt, ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS);
  ////ACBTSET->actual_human=0;
  hri_human * human=ACBTSET->human[ACBTSET->actual_human];
  ////int state=1;
 
  if(strcasestr(human->HumanPt->name,"superman"))
    {
      hum_cur_pos[8] = human->state[state].c7;
      hum_cur_pos[43] = human->state[state].c1;
      hum_cur_pos[44] = human->state[state].c2;
      hum_cur_pos[46] = human->state[state].c3;
      hum_cur_pos[47] = human->state[state].c4;
      hum_cur_pos[50] = human->state[state].c5;
      hum_cur_pos[53] = human->state[state].c6;
      // Right Hand 
      hum_cur_pos[66] = hum_cur_pos[6] + cos(hum_cur_pos[11]-0.4)*0.5; // REVIEW 0.4 --> 0.2 
      hum_cur_pos[67] = hum_cur_pos[7] + sin(hum_cur_pos[11]-0.4)*0.5;
      hum_cur_pos[68] = hum_cur_pos[68]-0.34+0.1;
      // Left Hand 
      hum_cur_pos[72] = hum_cur_pos[6] + cos(hum_cur_pos[11]+0.4)*0.5;
      hum_cur_pos[73] = hum_cur_pos[7] + sin(hum_cur_pos[11]+0.4)*0.5;
      hum_cur_pos[74] = hum_cur_pos[74]-0.34+0.1;
    }
  else
    {
      if(strcasestr(human->HumanPt->name,"achile"))
	{
	  hum_cur_pos[8] = human->state[state].c7;
	  hum_cur_pos[33] = human->state[state].c3;
	  hum_cur_pos[35] = human->state[state].c4;
	  hum_cur_pos[40] = human->state[state].c1;
	  hum_cur_pos[42] = human->state[state].c2;
	}   
    } 
  ////human->actual_state=state;
  ////hri_set_human_state_SICK(ACBTSET->human[ACBTSET->actual_human], 1, q, FALSE);

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[hum_index],hum_cur_pos);
  MY_FREE(hum_cur_pos,double,envPt_MM->robot[hum_index]->nb_dof);

  ////p3d_destroy_config(ACBTSET->human[ACBTSET->actual_human]->HumanPt,config);
  ////ACBTSET->changed = TRUE;

  /*if(BTSET!=NULL)
    hri_bt_refresh_all(BTSET);
    if(INTERPOINT!=NULL){
    hri_bt_3drefresh_all(INTERPOINT);
    }*/
  ////////g3d_draw_env();
  ////////fl_check_forms();
  ////////g3d_draw_allwin_active();
  ////g3d_draw_allwin_active();
  ////human_state_updated=0;
 
 
  //////////printf("After updating human state, ACBTSET->human[ACBTSET->actual_human]->actual_state=%d\n",ACBTSET->human[ACBTSET->actual_human]->actual_state);
  return 1;
}

int create_workspace_3D_grid()
{
  int HRP2_table_index;
  HRP2_table_index=get_index_of_robot_by_name("HRP2TABLE");
  ////HRP2_table_index=get_index_of_robot_by_name("IKEA_SHELF");
  configPt HRP2_table_pos = MY_ALLOC(double,envPt_MM->robot[HRP2_table_index]->nb_dof); /* Allocation of temporary robot configuration */

  p3d_get_robot_config_into(envPt_MM->robot[HRP2_table_index],&HRP2_table_pos);
  point_co_ordi grid_center;
  grid_center.x=HRP2_table_pos[6];
  grid_center.y=HRP2_table_pos[7];
  grid_center.z=HRP2_table_pos[8];

  create_3d_grid_for_HRP2_GIK(grid_center);
  MY_FREE(HRP2_table_pos,double,envPt_MM->robot[HRP2_table_index]->nb_dof);
}

int find_human_current_visibility_in_3D(HRI_AGENT *human_agent_MM, HRI_TASK_AGENT agent_type)
{
  
  int visibility_type=MM_CURRENT_STATE_HUM_VIS;
  /*HRI_TASK_AGENT agent_type;
 
  if(strcasestr(human_agent_MM->robotPt->name,"HUMAN1"))
  {
    agent_type=HUMAN1_MA;
  }   
  if(strcasestr(human_agent_MM->robotPt->name,"HUMAN2"))
  {
    agent_type=HUMAN2_MA;
  }   */
  
   get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_visibility(primary_human_MM,1,MM_CURRENT_STATE_HUM_VIS);
  find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);//agent_type: 1 means human, 2 means HRP2, 3 means JIDO, 4 means second human
  ////ChronoPrint(" Time for getting points on HUMAN FOV screen");
}
 
int find_human_all_visibilities_in_3D(HRI_AGENT *human_agent_MM, HRI_TASK_AGENT agent_type)
{
  p3d_get_robot_config_into(human_agent_MM->robotPt,&HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  ////virtually_update_human_state_new(1);// Sitting
  p3d_rob * human_Pt=envPt_MM->robot[indices_of_MA_agents[agent_type]];
  ////ChronoOn();
  
  ////printf("rob_indx.HUMAN=%d\n",rob_indx.HUMAN);
  ////int agent_type=1;//for human 1
  int visibility_type=MM_CURRENT_STATE_HUM_VIS;

  //////////get_all_points_on_FOV_screen(primary_human_MM);
  ////find_3D_grid_visibility(primary_human_MM,1,MM_CURRENT_STATE_HUM_VIS);
  //////////find_3D_grid_visibility_for_MM(primary_human_MM,agent_type, visibility_type);//agent_type: 1 means human, 2 means HRP2, 3 means JIDO, 4 means second human
  ////ChronoPrint(" Time for getting points on HUMAN FOV screen");

  find_human_current_visibility_in_3D(human_agent_MM, agent_type);
  
  //Now making the head straight along of axis of torso
  ////configPt hum_cur_pos = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); /* Allocation of temporary robot configuration */

  /*HRI_TASK_AGENT agent_type;
 
  if(strcasestr(human_agent_MM->robotPt->name,"HUMAN1"))
  {
    agent_type=HUMAN1_MA;
  }   
  if(strcasestr(human_agent_MM->robotPt->name,"HUMAN2"))
  {
    agent_type=HUMAN2_MA;
  }   
  */
  ////p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&HUMAN1_running_pos_MM);
  p3d_get_robot_config_into(human_agent_MM->robotPt,&HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  
  double fixed_pitch=-M_PI/6.0;//M_PI/8.0;
 
  double yaw=0.0;
  double pitch=fixed_pitch;
  double orig_pan=HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]];
  double orig_tilt=HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]];
  //////////printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]]=pitch; // Human pitch angle relative to the human body frame
  ////p3d_set_and_update_this_robot_conf(envPt_MM->robot[rob_indx.HUMAN], HUMAN1_running_pos_MM);
  
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  
  ///////////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_PAN]=HUMAN1_running_pos_MM[HUMANq_PAN];
  ///////////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_TILT]=HUMAN1_running_pos_MM[HUMANq_TILT];
  int agents_curr_state;
  
  switch(agent_type)
  {
    case HUMAN1_MA:
      agents_curr_state=HUMAN1_CURRENT_STATE_MM;
    
   break;
  
#ifdef HUMAN2_EXISTS_FOR_MA
    case HUMAN2_MA:
      agents_curr_state=HUMAN2_CURRENT_STATE_MM;
     
    break;
#endif
  }
 
 if(agents_curr_state==HRI_SITTING)
   {
  visibility_type=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
   }
  else
   {
    if(agents_curr_state==HRI_STANDING)
    {
     visibility_type=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
    }
   }
  ///////get_all_points_on_FOV_screen(primary_human_MM);
  get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_straight_visibility(primary_human_MM,1);
  //////////find_3D_grid_visibility_for_MM(primary_human_MM,agent_type, visibility_type);
  find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);

  yaw=M_PI/3.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  //////////p3d_set_and_update_this_robot_conf(envPt_MM->robot[rob_indx.HUMAN], HUMAN1_running_pos_MM);
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  //////////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_PAN]=HUMAN1_running_pos_MM[HUMANq_PAN];
  //////////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_TILT]=HUMAN1_running_pos_MM[HUMANq_TILT];
  if(agents_curr_state==HRI_SITTING)
   {
  visibility_type=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
   }
  else
   {
    if(agents_curr_state==HRI_STANDING)
    {
     visibility_type=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
    }
   }

  get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_turn_head_visibility(primary_human_MM,1);
   find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);

  yaw=-M_PI/3.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  //////////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_PAN]=HUMAN1_running_pos_MM[HUMANq_PAN];
  //////////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_TILT]=HUMAN1_running_pos_MM[HUMANq_TILT];
  get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_turn_head_visibility(primary_human_MM,1);
  find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);

  yaw=0.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  
  int CURR_HUMAN_TEMP_STATE_MM;

    if(agents_curr_state==HRI_SITTING)
   {
  virtually_update_human_state_new(human_Pt,HRI_STANDING);// Standing
  visibility_type=MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS;
  CURR_HUMAN_TEMP_STATE_MM=HRI_STANDING;
   }
  else
   {
    if(agents_curr_state==HRI_STANDING)
    {
     virtually_update_human_state_new(human_Pt,HRI_SITTING);// Sitting
     visibility_type=MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS;
     CURR_HUMAN_TEMP_STATE_MM=HRI_SITTING;
    }
   }

  p3d_get_robot_config_into(human_agent_MM->robotPt,&HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////g3d_draw_env();
   //// fl_check_forms();
   ////g3d_draw_allwin_active();
  ////////hri_update_agent_perspective_params(human_agent_MM);
  get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_straight_visibility(primary_human_MM,1);
  find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);
  
  

  ////standing_human_eye_pos.x=human_agent_MM->perspective->camjoint->abs_pos[0][3];
  ////standing_human_eye_pos.y=human_agent_MM->perspective->camjoint->abs_pos[1][3];
  ////standing_human_eye_pos.z=human_agent_MM->perspective->camjoint->abs_pos[2][3];
  //// standing_human_eye_pos.x=envPt_MM->robot[rob_indx.HUMAN]->joints[42]->abs_pos[0][3];
  ////standing_human_eye_pos.y=envPt_MM->robot[rob_indx.HUMAN]->joints[42]->abs_pos[1][3];
  ////standing_human_eye_pos.z=envPt_MM->robot[rob_indx.HUMAN]->joints[42]->abs_pos[2][3];
  
//     fl_check_forms();
//    g3d_draw_allwin_active();
  
  yaw=M_PI/3.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_PAN]=HUMAN1_running_pos_MM[HUMANq_PAN];
 //// envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_TILT]=HUMAN1_running_pos_MM[HUMANq_TILT];
  
  if(CURR_HUMAN_TEMP_STATE_MM==HRI_SITTING)
   {
  visibility_type=MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
   }
  else
   {
  if(CURR_HUMAN_TEMP_STATE_MM==HRI_STANDING)
    {
  visibility_type=MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS;
    }
   }
 
  
  get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_turn_head_visibility(primary_human_MM,1);
   find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);
    
    ////fl_check_forms();
   ////g3d_draw_allwin_active();
   
  yaw=-M_PI/3.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_PAN]=HUMAN1_running_pos_MM[HUMANq_PAN];
  ////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_TILT]=HUMAN1_running_pos_MM[HUMANq_TILT];
  get_all_points_on_FOV_screen(human_agent_MM);
  ////find_3D_grid_turn_head_visibility(primary_human_MM,1);
  find_3D_grid_visibility_for_MM(human_agent_MM,agent_type, visibility_type);
  
  
    ////fl_check_forms();
   ////g3d_draw_allwin_active();
  ////virtually_update_human_state_new(HUMAN1_CURRENT_STATE_MM);
  
  ////HUMAN1_running_pos_MM[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  ////HUMAN1_running_pos_MM[HUMANq_TILT]=orig_tilt; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(human_agent_MM->robotPt, HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_PAN]=HUMAN1_running_pos_MM[HUMANq_PAN];
  ////envPt_MM->robot[rob_indx.HUMAN]->ROBOT_POS[HUMANq_TILT]=HUMAN1_running_pos_MM[HUMANq_TILT];
}

int find_JIDO_robot_current_visibility_in_3D()
{
  int visibility_type=MM_CURRENT_STATE_JIDO_VIS;
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[JIDO_MA]);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[JIDO_MA],JIDO_MA,visibility_type);
}

int find_JIDO_robot_all_visibilities_in_3D()
{

  double fixed_pitch=-M_PI/6.0;//M_PI/8.0;
  double yaw=0.0;
  double pitch=fixed_pitch;
  double orig_pan=0;
  double orig_tilt=0;

  ////printf("rob_indx.JIDO_ROBOT=%d\n",rob_indx.JIDO_ROBOT);
 
  ////g3d_draw_rob_cone(ACBTSET->robot);
  ////g3d_draw_agent_fov(jido_robot);
  HRI_TASK_AGENT agent_type=JIDO_MA;
  int visibility_type;
  ////get_all_points_on_FOV_screen(jido_robot_MM);
  ////find_3D_grid_visibility_for_MM(jido_robot_MM,3,visibility_type);
  find_JIDO_robot_current_visibility_in_3D();
  
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_running_pos[agent_type]);

  orig_pan=HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]];
  orig_tilt=HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]];

  yaw=0.0;
  pitch=fixed_pitch;

  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];
  
  visibility_type=MM_STRAIGHT_HEAD_STATE_JIDO_VIS;
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[agent_type]);
  ////find_3D_grid_straight_visibility(jido_robot_MM,3);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[agent_type],agent_type,visibility_type);

  yaw=M_PI/4.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];
  
  visibility_type=MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS;
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[agent_type]);
  ////find_3D_grid_turn_head_visibility(jido_robot_MM,3);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[agent_type],agent_type,visibility_type);

  yaw=-M_PI/4.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  /////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  /////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[agent_type]);
  ////find_3D_grid_turn_head_visibility(jido_robot_MM,3);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[agent_type],agent_type,visibility_type);

  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=orig_pan; // Human Yaw angle relative to the human body frame
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]]=orig_tilt; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];


  //TODO WRITE code for HRP2


 

  //////////printf(" Original pan = %lf \n",orig_pan); 
  ////fixed_pitch=-M_PI/10.0;

}

#ifdef PR2_EXISTS_FOR_MA
int find_PR2_robot_current_visibility_in_3D()
{
  int visibility_type=MM_CURRENT_STATE_PR2_VIS;
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[PR2_MA]);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[PR2_MA],PR2_MA,visibility_type);
}

int find_PR2_robot_all_visibilities_for_posture_in_3D(PR2_POSTURE_MA for_posture)
{
  
  double fixed_pitch=M_PI/6.0;//M_PI/8.0;
  double yaw=0.0;
  double pitch=fixed_pitch;
  double orig_pan=0;
  double orig_tilt=0;

  ////printf("rob_indx.JIDO_ROBOT=%d\n",rob_indx.JIDO_ROBOT);
 
  ////g3d_draw_rob_cone(ACBTSET->robot);
  ////g3d_draw_agent_fov(jido_robot);
  HRI_TASK_AGENT agent_type=PR2_MA;
  int visibility_type;
  ////get_all_points_on_FOV_screen(jido_robot_MM);
  ////find_3D_grid_visibility_for_MM(jido_robot_MM,3,visibility_type);
  //////////if(PR2_CURRENT_POSTURE==for_posture)
  //////////find_PR2_robot_current_visibility_in_3D();
  
  
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_running_pos[agent_type]);

  orig_pan=HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]];
  orig_tilt=HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]];

  yaw=0.0;
  pitch=fixed_pitch;

  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];
  switch (for_posture)
  {
    case PR2_ARBITRARY_MA:
      visibility_type=MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
      break;
      
    case PR2_LOW_MA:
      visibility_type=MM_LOW_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
      break;
      
    case PR2_HIGH_MA:
      visibility_type=MM_HIGH_POST_STRAIGHT_HEAD_STATE_PR2_VIS;
      break;
      
  }

  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[agent_type]);
  ////find_3D_grid_straight_visibility(jido_robot_MM,3);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[agent_type],agent_type,visibility_type);

  yaw=M_PI/4.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  ////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];
  switch (for_posture)
  {
    case PR2_ARBITRARY_MA:
      visibility_type=MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
      break;
      
    case PR2_LOW_MA:
      visibility_type=MM_LOW_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
      break;
      
    case PR2_HIGH_MA:
      visibility_type=MM_HIGH_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS;
      break;
      
  }
  
  ////visibility_type=MM_LOW_LOOK_AROUND_HEAD_STATE_PR2_VIS;
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[agent_type]);
  ////find_3D_grid_turn_head_visibility(jido_robot_MM,3);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[agent_type],agent_type,visibility_type);

  yaw=-M_PI/4.0;
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=yaw; // Human Yaw angle relative to the human body frame
  ////rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  /////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_PAN]=JIDO_running_pos_MM[ROBOTq_PAN];
  /////envPt_MM->robot[rob_indx.JIDO_ROBOT]->ROBOT_POS[ROBOTq_TILT]=JIDO_running_pos_MM[ROBOTq_TILT];
  get_all_points_on_FOV_screen(HRI_AGENTS_FOR_MA[agent_type]);
  ////find_3D_grid_turn_head_visibility(jido_robot_MM,3);
  find_3D_grid_visibility_for_MM(HRI_AGENTS_FOR_MA[agent_type],agent_type,visibility_type);

  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[PAN]]=orig_pan; // Human Yaw angle relative to the human body frame
  HRI_AGENTS_FOR_MA_running_pos[agent_type][agents_for_MA_obj.for_agent[agent_type].head_params.Q_indices[TILT]]=orig_tilt; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);

}


int find_PR2_robot_all_visibilities_in_3D()
{
  int agent_type=PR2_MA;
  
find_PR2_robot_current_visibility_in_3D();

p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_running_pos[agent_type]);

PR2_POSTURE_MA pr2_posture=PR2_ARBITRARY_MA;
  find_PR2_robot_all_visibilities_for_posture_in_3D(pr2_posture);
  
  
  HRI_AGENTS_FOR_MA_running_pos[agent_type][PR2_Q_TORSO]=PR2_Q_TORSO_HIGH_VAL;
    pr2_posture=PR2_HIGH_MA;
    
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
   ////g3d_draw_allwin_active();
   find_PR2_robot_all_visibilities_for_posture_in_3D(pr2_posture);
   
  
     HRI_AGENTS_FOR_MA_running_pos[agent_type][PR2_Q_TORSO]=PR2_Q_TORSO_LOW_VAL;
    pr2_posture=PR2_LOW_MA;
    
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
      ////g3d_draw_allwin_active();

   find_PR2_robot_all_visibilities_for_posture_in_3D(pr2_posture);
  
  
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  
}


#endif
int find_Mightability_Maps()
{

  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr = envPt_MM->nr;
  ChronoOff();
  ChronoOn();
  create_workspace_3D_grid();
  ChronoPrint("Time for creating and Initializing 3D grid");
  printf(" **** 3D grid dimension is (%d x %d x %d), no. of cells =%d \n",grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx*grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny*grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz);

  get_horizontal_surfaces();
  //////////virtually_update_human_state_new(1);// Sitting
  ChronoOff();
  ChronoOn();
  HRI_TASK_AGENT agent_type=HUMAN1_MA;
  
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  find_human_all_visibilities_in_3D(HRI_AGENTS_FOR_MA[agent_type], agent_type);
  ////update_3d_grid_reachability_for_human_new(1);//for human1
  p3d_rob * agent_Pt=envPt_MM->robot[indices_of_MA_agents[agent_type]];
  
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  
  update_current_3d_grid_reachability_for_human_MM(agent_type);
  
  update_3d_grid_reachability_for_human_MM(agent_type,HUMAN1_CURRENT_STATE_MM);//for human1
  
  if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
  {
   virtually_update_human_state_new(agent_Pt,HRI_STANDING);
   update_3d_grid_reachability_for_human_MM(agent_type,HRI_STANDING);//for human1
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
   ////virtually_update_human_state_new(1);
  }
  else
  {
   if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
   {
   virtually_update_human_state_new(agent_Pt,HRI_SITTING);
   update_3d_grid_reachability_for_human_MM(agent_type,HRI_SITTING);//for human1
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
   ////virtually_update_human_state_new(2);
   }
  }
  
#ifdef HUMAN2_EXISTS_FOR_MA
agent_type=HUMAN2_MA;
  agent_Pt=envPt_MM->robot[indices_of_MA_agents[agent_type]];
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  find_human_all_visibilities_in_3D(HRI_AGENTS_FOR_MA[agent_type], agent_type);
  ////update_3d_grid_reachability_for_human_new(1);//for human1
  
  
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  
  update_3d_grid_reachability_for_human_MM(agent_type,HUMAN2_CURRENT_STATE_MM);
  
  if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
  {
   virtually_update_human_state_new(agent_Pt,HRI_STANDING);
   update_3d_grid_reachability_for_human_MM(agent_type,HRI_STANDING);//for human1
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
   ////virtually_update_human_state_new(1);
  }
  else
  {
   if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
   {
   virtually_update_human_state_new(agent_Pt,HRI_SITTING);
   update_3d_grid_reachability_for_human_MM(agent_type,HRI_SITTING);//for human1
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
   ////virtually_update_human_state_new(2);
   }
  }
  
#endif
 

  ////ChronoPrint(" Time for getting points on all agents FOV screen");
  ////ChronoOff();
int for_state;
  ////ChronoOn();
#ifdef JIDO_EXISTS_FOR_MA
agent_type=JIDO_MA;
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  find_JIDO_robot_all_visibilities_in_3D();
  ////update_3d_grid_reachability_for_JIDO_MM();
  for_state=MM_CURRENT_STATE_JIDO_REACH;
  update_3d_grid_reachability_for_agent_MM(JIDO_MA, MA_RIGHT_HAND, for_state);
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
#endif
  
#ifdef PR2_EXISTS_FOR_MA
  agent_type=PR2_MA;
  
  
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
  p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[agent_type]],&HRI_AGENTS_FOR_MA_running_pos[agent_type]);
  
  
  find_PR2_robot_all_visibilities_in_3D();
  
  
  for_state=MM_CURRENT_STATE_PR2_REACH;
  update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_RIGHT_HAND, for_state);
  update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_LEFT_HAND, for_state);
 
  
  HRI_AGENTS_FOR_MA_running_pos[agent_type][PR2_Q_TORSO]=PR2_Q_TORSO_HIGH_VAL;
   
    
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
   
   for_state=MM_HIGH_REACH_STATE_PR2;
  update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_RIGHT_HAND, for_state);
  update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_LEFT_HAND, for_state);
   
     HRI_AGENTS_FOR_MA_running_pos[agent_type][PR2_Q_TORSO]=PR2_Q_TORSO_LOW_VAL;
    
    
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_running_pos[agent_type]);
   
  
  for_state=MM_LOW_REACH_STATE_PR2;
  update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_RIGHT_HAND, for_state);
  update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_LEFT_HAND, for_state);
  
  
  
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[agent_type]], HRI_AGENTS_FOR_MA_actual_pos[agent_type]);
#endif
  
#ifdef HRP2_EXISTS_FOR_MA
  //TODO Write for finding visibility for HRP2
  update_3d_grid_reachability_for_HRP2_new();
#endif
  



  grid_3d_affordance_calculated=1; 

  ChronoPrint(" Time for getting 3D grid Mightability of all agents");
  ChronoOff();

  p3d_set_freeflyer_pose2(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],0,0,0,0,0,0);
  return 1;
}
/*
  #if defined(WITH_XFORM)  
  int find_affordance_new()
  {

  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr = envPt_MM->nr;

  ChronoOn();
  ////M3D_GIK_TEST();
  ////printf(" Inside find_affordance \n");

  int kcd_with_report=0;
  p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

  int res = p3d_col_test_robot(human,kcd_with_report);
  if(res>0)
  {
  printf(" There is collision with human, res=%d \n", res);
  pqp_print_colliding_pair();
  //return 0;
  }
  kcd_with_report=0;
  res = p3d_col_test_self_collision(human,kcd_with_report);
  if(res>0)
  {
  printf(" There is self collision with human, res=%d \n", res);
  pqp_print_colliding_pair();
  //return 0;
  }

  int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  ///////create_HRP2_robot(HRP2_state);

  p3d_rob *cur_rob=ACBTSET->robot;

	
  for(int j=0;j<cur_rob->no;j++)
  {
  p3d_obj *o = cur_rob->o[j];
  //if (strstr(o->name,"head") || strstr(o->name,"HEAD") || strstr(o->name,"hand") || strstr(o->name,"HAND"))
  // {
  ////p3d_get_object_center(o,objCenter);
  printf("%s\n",o->name);
  // }	  
  }
			
  //define if robot is near or not? here or in observation? od we need a different list
  // if ((ContObjTmp/r->no)>.4)
  // 
			
		


  kcd_with_report=0;
  //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
  res = p3d_col_test_self_collision(cur_rob,2);
  ////res = p3d_col_test_robot(cur_rob,2);
  //printf("collision 2:   %i \n",res2);
  set_kcd_which_test(P3D_KCD_ROB_ALL);
  res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
  //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
  if(res>0)
  {
  printf(" There is self collision with robot, res=%d \n", res);
  ////return 0;
  }

  kcd_with_report=0;
  res = p3d_col_test_robot(cur_rob,2);
  //printf("collision 2:   %i \n",res2);
  set_kcd_which_test(P3D_KCD_ROB_ALL);
  res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
  //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
  if(res>0)
  {
  printf(" There is collision with robot, res=%d \n", res);
  ////return 0;
  }
 

  if(ONLINE_TRACKING_FLAG==0)
  initialize_surfaces_in_env(); // AKP Note: Comment it if using motion capture button


 
  ////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 

  //////////printf(" Before create_3d_grid_for_HRP2_GIK() \n");
  ////////////ChronoPrint("***");
  int HRP2_table_index;
  HRP2_table_index=get_index_of_robot_by_name("HRP2TABLE");
  configPt HRP2_table_pos = MY_ALLOC(double,envPt_MM->robot[HRP2_table_index]->nb_dof); 

  p3d_get_robot_config_into(envPt_MM->robot[HRP2_table_index],&HRP2_table_pos);
  point_co_ordi grid_center;
  grid_center.x=HRP2_table_pos[6];
  grid_center.y=HRP2_table_pos[7];
  grid_center.z=HRP2_table_pos[8];

  create_3d_grid_for_HRP2_GIK(grid_center);
  //////////ChronoPrint("Time for create_3d_grid_for_HRP2_GIK()");
  //////////printf(" After create_3d_grid_for_HRP2_GIK() \n");
  ChronoOff();

  printf(" **** 3D grid dimension is (%d x %d x %d), no. of cells =%d \n",grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx*grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny*grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz);

  double cur_h_angle;
  double interval;


  // //////////////////////tmp for jido
  // ChronoOn();
  // grid_3d_affordance_calculated=1; 
  // 
  // 
  // ////p3d_rob *cur_rob=ACBTSET->robot;
  // cur_h_angle=cur_rob->cam_h_angle;
  // interval=grid_around_HRP2.GRID_SET->pace;
  // printf("cur_h_angle=%lf\n",cur_h_angle);
  // no_FOV_end_point_vertices=0;
  // while(cur_rob->cam_h_angle>0.001)
  // { 
  // cur_rob->cam_h_angle-=interval;
  //  
  // get_points_on_FOV_screen(cur_rob);
  // }
  // cur_rob->cam_h_angle=cur_h_angle;
  // ////ChronoOn();
  // update_3d_grid_visibility(3); //2 for JIDO
  // 
  // ChronoOff();
  // 
  // update_3d_grid_reachability_for_JIDO_new();
  // 
  // 
  // return 1;
  // ///////////////////////end tmp for jido



  ChronoOn();

  //*****AKP: Tmp uncomment/comment because of problem in new human model for sitting as well as standing
  virtually_update_human_state_new(1);// Sitting



  cur_h_angle=human->cam_h_angle;
  printf(" cur_h_angle=%lf\n",cur_h_angle);


  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);

  }
  ////return 0;
  human->cam_h_angle=cur_h_angle;
  ////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
  ////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
  ////ChronoOn();	    

  update_3d_grid_visibility(1); //1 for human

  ////////////ChronoPrint("TIME of 3D Visibility calculation for sitting Human from current position for current head orientation");

  //Now making the head straight along of axis of torso
  configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);  

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

  double fixed_pitch=M_PI/8.0;

  double yaw=0.0;
  double pitch=fixed_pitch;
  double orig_pan=hum_cur_pos[HUMANq_PAN];
  double orig_tilt=hum_cur_pos[HUMANq_TILT];
  //////////printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_straight_visibility(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  //Now turning the head only

  hum_cur_pos[HUMANq_PAN]=0.0;
  yaw=M_PI/3.0;
  pitch=fixed_pitch;
  ////orig_pan=hum_cur_pos[HUMANq_PAN];
  ////printf(" Original pan = %lf \n",orig_pan); 
  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_PAN]=0.0;// Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

  hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


  MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  double tu,ts;
  //////////ChronoPrint("TIME of 3D Visibility calculation for sitting Human");
  ChronoTimes(&tu,&ts);
  //////////printf(" %lf, %lf \n",tu,ts);
  ChronoOff();

  #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
  ///////////////3D visibility calculation for making human virtually standing//////////////////////////

  ChronoOn();
  virtually_update_human_state_new(0);// Standing
  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle for standing human=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }

  human->cam_h_angle=cur_h_angle;
  ////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
  ////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
  ////ChronoOn();	    

  update_3d_grid_visibility_standing(1); //1 for human

  /////////////ChronoPrint("TIME of 3D Visibility calculation for standing Human from current position");

  //Now making the head straight
  hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); 

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  yaw=0.0;
  pitch=fixed_pitch;
  orig_pan=hum_cur_pos[HUMANq_PAN];
  orig_tilt=hum_cur_pos[HUMANq_TILT];
  //////////printf(" Original pan = %lf \n",orig_pan); 
  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_straight_visibility_standing(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


  //Now turning the head only
  ////hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); 

  ////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  yaw=M_PI/3.0;
  pitch=fixed_pitch;
  ////orig_pan=hum_cur_pos[HUMANq_PAN];
  ////printf(" Original pan = %lf \n",orig_pan); 

  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

  hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);


  //////////ChronoPrint("TIME of 3D Visibility calculation for standing Human");
  ChronoTimes(&tu,&ts);
  ////printf(" %lf, %lf \n",tu,ts);
  ////////update_human_state(1);// Again make it to Sitting
  virtually_update_human_state_new(1);// // Again make it to Sitting
  ChronoOff();

  ///////////////END of 3D visibility calculation for making human virtually standing//////////////////////////
  #endif


  ChronoOn();
  grid_3d_affordance_calculated=1; 


  ////p3d_rob *cur_rob=ACBTSET->robot;
  cur_h_angle=cur_rob->cam_h_angle;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }
  cur_rob->cam_h_angle=cur_h_angle;
  ////ChronoOn();
  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_visibility(3); //3 for JIDO
  #elif defined(HRI_HRP2)
  update_3d_grid_visibility(2); //2 for HRP2
  #endif


  //Now making the head straight
  configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

  orig_pan=rob_cur_pos[ROBOTq_PAN];
  orig_tilt=rob_cur_pos[ROBOTq_TILT];
  //////////printf(" Original pan = %lf \n",orig_pan); 

  yaw=0.0;
  pitch=fixed_pitch;

  rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  cur_h_angle=cur_rob->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }

  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_straight_visibility(3); //3 for JIOD
  #elif defined(HRI_HRP2)
  update_3d_grid_straight_visibility(2); //2 for HRP2
  #endif
  cur_rob->cam_h_angle=cur_h_angle;

  rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  //Now turning the head only
  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
  yaw=M_PI/4.0;
  pitch=fixed_pitch;
  rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  cur_h_angle=cur_rob->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }

  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
  #elif defined(HRI_HRP2)
  update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
  #endif

  cur_rob->cam_h_angle=cur_h_angle;

  rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

  rob_cur_pos[ROBOTq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);

  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  cur_h_angle=cur_rob->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }
  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
  #elif defined(HRI_HRP2)
  update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
  #endif
  //////////update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
  cur_rob->cam_h_angle=cur_h_angle;

  rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  MY_FREE(rob_cur_pos,double,ACBTSET->robot->nb_dof);


  ////////////ChronoPrint("TIME of 3D Visibility calculation for ROBOT");
  ////ChronoOff();
  ////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  ////create_HRP2_robot(HRP2_state);
  ////g3d_draw_env();
  //// fl_check_forms();
  //// g3d_draw_allwin_active();


  ChronoOff();



  ChronoOn();
  //// create_HRP2_robot(HRP2_state);
  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_reachability_for_JIDO_new();
  #elif defined(HRI_HRP2)
  update_3d_grid_reachability_for_HRP2_new();
  #endif
  //////////update_3d_grid_reachability_for_HRP2_new();
  ////ChronoPrint("****TIME of 3D reachability calculation for ROBOT\n");



  ////return 1;
  ChronoOff();


  ChronoOn();

  update_3d_grid_reachability_for_human_new(1);//for human1

  #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
  //////////update_human_state(0);// Standing
  virtually_update_human_state_new(0);// Standing
  update_3d_grid_reachability_for_human_standing_new();

  ////////update_human_state(1);
  virtually_update_human_state_new(1);// Sitting
  #endif

  ////ChronoPrint("****TIME of 3D reachability calculation for human\n");

  ChronoOff();

  ///// Code for Mightability Calculation for second human
  #ifdef HUMAN2_EXISTS_FOR_MA

  int human_2_index;
  human_2_index=get_index_of_robot_by_name("ACHILE_HUMAN2");
  
  p3d_rob *human2=envPt_MM->robot[human_2_index];

  res = p3d_col_test_robot(human2,kcd_with_report);
  if(res>0)
  {
  printf(" There is collision with human2, res=%d \n", res);
  pqp_print_colliding_pair();
  //return 0;
  }
  kcd_with_report=0;
  res = p3d_col_test_self_collision(human2,kcd_with_report);
  if(res>0)
  {
  printf(" There is self collision with human2, res=%d \n", res);
  pqp_print_colliding_pair();
  //return 0;
  }

  virtually_update_non_primary_human_state(1, human_2_index);// Sitting

  cur_h_angle=human2->cam_h_angle;
  printf(" cur_h_angle=%lf\n",cur_h_angle);


  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);

  }
  ////return 0;
  human2->cam_h_angle=cur_h_angle;
  ////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
  ////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
  ////ChronoOn();	    

  update_3d_grid_visibility(4); //4 for human2


  ////////////ChronoPrint("TIME of 3D Visibility calculation for sitting Human from current position for current head orientation");

  //Now making the head straight along of axis of torso
  configPt hum2_cur_pos = MY_ALLOC(double,envPt_MM->robot[human_2_index]->nb_dof); 

  p3d_get_robot_config_into(envPt_MM->robot[human_2_index],&hum2_cur_pos);



  yaw=0.0;
  pitch=fixed_pitch;
  orig_pan=hum2_cur_pos[HUMANq_PAN];
  orig_tilt=hum2_cur_pos[HUMANq_TILT];
  //////////printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

  hum2_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=pitch; // Human pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }

  update_3d_grid_straight_visibility(4); //4 for human2
  human2->cam_h_angle=cur_h_angle;

  hum2_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  /////return 0;
  //Now turning the head only

  hum2_cur_pos[HUMANq_PAN]=0.0;
  yaw=M_PI/3.0;
  pitch=fixed_pitch;
  ////orig_pan=hum_cur_pos[HUMANq_PAN];
  ////printf(" Original pan = %lf \n",orig_pan); 
  hum2_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }
  update_3d_grid_visibility_by_neck_turn(4); //4 for human2
  human2->cam_h_angle=cur_h_angle;

  hum2_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  ////hum_cur_pos[HUMANq_PAN]=0.0;// Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  p3d_get_robot_config_into(envPt_MM->robot[human_2_index],&hum2_cur_pos);

  hum2_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];


  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }
  update_3d_grid_visibility_by_neck_turn(4); //4 for human2
  human2->cam_h_angle=cur_h_angle;

  hum2_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  update_3d_grid_reachability_for_human_new(4);//for human1

  MY_FREE(hum2_cur_pos,double,envPt_MM->robot[human_2_index]->nb_dof);

  #endif


  return 1;

  //return 1;

  // ChronoPrint("Time before calculating affordance on surfaces ");
  //  //current_surface_index=0;
  //  int i=0;
  //  for(i=0;i<curr_surfaces_in_env.total_no_of_surfaces;i++)
  //  {
  //  printf(" For surface %d\n",i);
  //  ChronoPrint("Time Before update_surface_grid_based_on_curr_pos");
  //  update_surface_grid_based_on_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  // 
  //  ChronoPrint("Time Before update_surface_grid_by_bending_human_at_curr_pos");
  //  update_surface_grid_by_bending_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  // 
  //  ChronoPrint("Time Before update_surface_grid_by_turning_human_at_curr_pos");
  //  update_surface_grid_by_turning_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  //  
  //  
  // 
  //  
  //  
  //  HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  //  //////ChronoPrint(" Time before create_HRP2_robot");
  //  //////create_HRP2_robot(HRP2_state);
  //  //////ChronoPrint(" Time after create_HRP2_robot and before update_surface_grid_for_HRP2_without_GIK");
  //  ////update_surface_grid_for_HRP2_with_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  //  update_surface_grid_for_HRP2_without_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  //  ChronoPrint(" Time after update_surface_grid_for_HRP2_without_GIK");
  // 
  //  }

  ////ChronoPrint("TIME for all affordance calculation");
  ChronoOff();
  ////create_3d_grid_for_HRP2_GIK();
  ////update_3D_grid_based_on_current_human_pos();
  //// grid_3d_affordance_calculated=1; 
 
  //update_surface_grid(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[1]);
  }
  #endif
*/
int update_Mightability_Maps_new()
{
  double total_time=0.0;
  ChronoOn();

  ////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  int expansion=1;
  update_3D_grid_for_Mightability_Maps_new(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP);
  
 

  int cell_x,cell_y,cell_z;
  for(cell_x=0; cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx; cell_x++)
    {
      for(cell_y=0; cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny; cell_y++)
	{
	  for(cell_z=0; cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz; cell_z++)
	    {
	      
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human=0;
	      ////if(HUMAN1_HAS_MOVED==0&&NEED_HUMAN1_VISIBILITY_UPDATE==1)
	      for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	      {
		 if(NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]==1)
		{
		  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible[i][CURR_VIS_STATE_INDEX_MA_AGENT[i]]=0;
		  
		  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible[i][CURR_VIS_STATE_INDEX_MA_AGENT[i]]=0;
		}
		
		if(NEED_ALL_VISIBILITY_UPDATE_AGENT[i]==1)
		{
		  for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
		  {
		    grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible[i][j]=0;
		     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible[i][j]=0;
		  }
		}
	      
	        if(NEED_CURRENT_REACHABILITY_UPDATE_AGENT[i]==1)
		{
		  for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
		    {
		      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable[i][CURR_REACH_STATE_INDEX_MA_AGENT[i]][k]=0;
		    }
		}
	       
	        if(NEED_ALL_REACHABILITY_UPDATE_AGENT[i]==1)
		{  
		 
		  for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		  {
		    for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
		    {
		      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable[i][j][k]=0;
		    }
		  }
		  
		}
	      }


	    }
	}
    }   
  //// }
  //////////ChronoPrint("Time for update_3D_grid_for_Mightability_Maps_new()");
  //////////printf(" After update_3D_grid_for_Mightability_Maps_new() \n");

  ChronoOff();
  ChronoOn();
for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
 {
  p3d_rob *agent_Pt=envPt_MM->robot[indices_of_MA_agents[i]];
    switch(i)
    {
      case HUMAN1_MA:
	
	if(NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_human_current_visibility_in_3D(HRI_AGENTS_FOR_MA[i],HRI_TASK_AGENT(i));
        NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=0;
	}
        
        if(NEED_ALL_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_human_all_visibilities_in_3D(HRI_AGENTS_FOR_MA[i],HRI_TASK_AGENT(i));
        NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_CURRENT_REACHABILITY_UPDATE_AGENT[i]==1)
        {
	////find_human_current_reachability_in_3D(HRI_AGENTS_FOR_MA[i],HRI_TASK_AGENT(i));
	update_current_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i));
        NEED_CURRENT_REACHABILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_ALL_REACHABILITY_UPDATE_AGENT[i]==1)
        {
      ////update_3d_grid_reachability_for_human_new(1);//1 for human1
      p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[i]],&HRI_AGENTS_FOR_MA_actual_pos[i]);
      update_current_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i));
      update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i),HUMAN1_CURRENT_STATE_MM);//for human1
  
	if(HUMAN1_CURRENT_STATE_MM==HRI_SITTING)
	  {
      virtually_update_human_state_new(agent_Pt,HRI_STANDING);
      update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i),HRI_STANDING);//for human1
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[i]],HRI_AGENTS_FOR_MA_actual_pos[i]);
      ////virtually_update_human_state_new(1);
	  }
      else
          {
      if(HUMAN1_CURRENT_STATE_MM==HRI_STANDING)
           {
	virtually_update_human_state_new(agent_Pt,HRI_SITTING);
	update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i),HRI_SITTING);//for human1
	p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[i]], HRI_AGENTS_FOR_MA_actual_pos[i]);
	////virtually_update_human_state_new(2);
            }
           }


      NEED_ALL_REACHABILITY_UPDATE_AGENT[i]=0;
         }
	break;
	
#ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
	
	if(NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_human_current_visibility_in_3D(HRI_AGENTS_FOR_MA[i],HRI_TASK_AGENT(i));
        NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=0;
	}
        
        if(NEED_ALL_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_human_all_visibilities_in_3D(HRI_AGENTS_FOR_MA[i],HRI_TASK_AGENT(i));
        NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_CURRENT_REACHABILITY_UPDATE_AGENT[i]==1)
        {
	////find_human_current_reachability_in_3D(HRI_AGENTS_FOR_MA[i],HRI_TASK_AGENT(i));
	update_current_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i));
        NEED_CURRENT_REACHABILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_ALL_REACHABILITY_UPDATE_AGENT[i]==1)
        {
      ////update_3d_grid_reachability_for_human_new(1);//1 for human1
      p3d_get_robot_config_into(envPt_MM->robot[indices_of_MA_agents[i]],&HRI_AGENTS_FOR_MA_actual_pos[i]);
      update_current_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i));
      update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i),HUMAN1_CURRENT_STATE_MM);//for human1
  
	if(HUMAN2_CURRENT_STATE_MM==HRI_SITTING)
	  {
      virtually_update_human_state_new(agent_Pt,HRI_STANDING);
      update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i),HRI_STANDING);//for human1
      p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[i]],HRI_AGENTS_FOR_MA_actual_pos[i]);
      ////virtually_update_human_state_new(1);
	  }
      else
          {
      if(HUMAN2_CURRENT_STATE_MM==HRI_STANDING)
           {
	virtually_update_human_state_new(agent_Pt,HRI_SITTING);
	update_3d_grid_reachability_for_human_MM(HRI_TASK_AGENT(i),HRI_SITTING);//for human1
	p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[i]], HRI_AGENTS_FOR_MA_actual_pos[i]);
	////virtually_update_human_state_new(2);
            }
           }


      NEED_ALL_REACHABILITY_UPDATE_AGENT[i]=0;
         }
	break;
#endif
	
#ifdef JIDO_EXISTS_FOR_MA
      case JIDO_MA:
	if(NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	 find_JIDO_robot_current_visibility_in_3D();
        NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=0;
	}
        
        if(NEED_ALL_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_JIDO_robot_all_visibilities_in_3D();
        NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_ALL_REACHABILITY_UPDATE_AGENT[i]==1)
        {
	  int for_state=MM_CURRENT_STATE_JIDO_REACH;
          update_3d_grid_reachability_for_agent_MM(JIDO_MA, MA_RIGHT_HAND, for_state);
	  NEED_ALL_REACHABILITY_UPDATE_AGENT[i]=0;
	}
	break;
#endif
	
#ifdef HRP2_EXISTS_FOR_MA
      case HRP2_MA:
	if(NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	 find_HRP2_robot_current_visibility_in_3D();
        NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=0;
	}
        
        if(NEED_ALL_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_HRP2_robot_all_visibilities_in_3D();
        NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_ALL_REACHABILITY_UPDATE_AGENT[i]==1)
        {
	  int for_state=MM_CURRENT_STATE_HRP2_REACH;
          update_3d_grid_reachability_for_agent_MM(HRP2_MA, MA_RIGHT_HAND, for_state);
	  
          update_3d_grid_reachability_for_agent_MM(HRP2_MA, MA_LEFT_HAND, for_state);
	  NEED_ALL_REACHABILITY_UPDATE_AGENT[i]=0;
	}
	break;
#endif
	
#ifdef PR2_EXISTS_FOR_MA
      case PR2_MA:
	if(NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	 find_PR2_robot_current_visibility_in_3D();
        NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=0;
	}
        
        if(NEED_ALL_VISIBILITY_UPDATE_AGENT[i]==1)
        {
	find_PR2_robot_all_visibilities_in_3D();
        NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=0;
	}
	
	if(NEED_ALL_REACHABILITY_UPDATE_AGENT[i]==1)
        {
	  int for_state=MM_CURRENT_STATE_PR2_REACH;
          update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_RIGHT_HAND, for_state);
	  
          update_3d_grid_reachability_for_agent_MM(PR2_MA, MA_LEFT_HAND, for_state);
	  NEED_ALL_REACHABILITY_UPDATE_AGENT[i]=0;
	}
	break;
#endif
    }
  
 }
  




  // ChronoPrint("Time for Update_Mightability_Maps_new()");
  ChronoOff();

  //Setting it to 0 to avoid any collision during planning
  p3d_set_freeflyer_pose2(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],0,0,0,0,0,0);
  return 1;

}
/*
  #if defined(WITH_XFORM)  
  int update_Mightability_Maps()
  {
  double total_time=0.0;
  ChronoOn();
  ////M3D_GIK_TEST();
  ////printf(" Inside find_affordance \n");

  int kcd_with_report=0;
  p3d_rob *human=ACBTSET->human[ACBTSET->actual_human]->HumanPt;


  // int res = p3d_col_test_robot(human,kcd_with_report);
  //  if(res>0)
  //  {
  //   printf(" There is collision with human, res=%d \n", res);
  //   //return 0;
  //  }
  // kcd_with_report=0;
  //  res = p3d_col_test_self_collision(human,kcd_with_report);
  //  if(res>0)
  //  {
  //   printf(" There is self collision with human, res=%d \n", res);
  //   //return 0;
  //  }

  int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  ///////create_HRP2_robot(HRP2_state);

  p3d_rob *cur_rob=ACBTSET->robot;

	
  // // 	for(int j=0;j<cur_rob->no;j++)
  // // 	{
  // // 	p3d_obj *o = cur_rob->o[j];
  // // 	//if (strstr(o->name,"head") || strstr(o->name,"HEAD") || strstr(o->name,"hand") || strstr(o->name,"HAND"))
  // // 	// {
  // // 	////p3d_get_object_center(o,objCenter);
  // //         printf("%s\n",o->name);
  // // 					// }	  
  // // 	}
			
  //define if robot is near or not? here or in observation? od we need a different list
  // if ((ContObjTmp/r->no)>.4)
  // 
			
		


  // //  kcd_with_report=0;
  // //  //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
  // //  res = p3d_col_test_self_collision(cur_rob,2);
  // //  ////res = p3d_col_test_robot(cur_rob,2);
  // // 		//printf("collision 2:   %i \n",res2);
  // //       set_kcd_which_test(P3D_KCD_ROB_ALL);
  // //       res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
  // //  //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
  // //  if(res>0)
  // //  {
  // //   printf(" There is self collision with robot, res=%d \n", res);
  // //   ////return 0;
  // //  }
  // // 
  // // kcd_with_report=0;
  // //   res = p3d_col_test_robot(cur_rob,2);
  // // 		//printf("collision 2:   %i \n",res2);
  // //       set_kcd_which_test(P3D_KCD_ROB_ALL);
  // //       res = p3d_col_does_robot_collide(cur_rob->num, p3d_numcoll);
  // //  //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
  // //  if(res>0)
  // //  {
  // //   printf(" There is collision with robot, res=%d \n", res);
  // //   ////return 0;
  // //  }
 

  // //  if(ONLINE_TRACKING_FLAG==0)
  // //  initialize_surfaces_in_env(); // AKP Note: Comment it if using motion capture button


 
  ////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  ////update_robots_status();

  ////printf(" Before update_3D_grid_for_Mightability_Maps_new() \n");
  ////////////ChronoPrint("***");
  ////update_3d_grid_for_Mightability_Maps();
  int expansion=1;
  update_3D_grid_for_Mightability_Maps_new(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP);

  ////if(HUMAN_HAS_MOVED==1)
  ////{
  int cell_x,cell_y,cell_z;
  for(cell_x=0; cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx; cell_x++)
  {
  for(cell_y=0; cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny; cell_y++)
  {
  for(cell_z=0; cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz; cell_z++)
  {
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human=0;
  if(HUMAN1_HAS_MOVED==0&&NEED_HUMAN1_VISIBILITY_UPDATE==1)
  {
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;
      
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_neck_turn=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation_standing=0;
       
  }
      
  if(HUMAN1_HAS_MOVED==1)
  {  
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human_straight_head_orientation_standing=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_LHand=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_RHand=0;
     
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_LHand_by_bending=0;  
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_RHand_by_bending=0;  

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending=0; 
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending=0;
     
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_neck_turn=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation_standing=0;

  NEED_HUMAN1_VISIBILITY_UPDATE=1; //To update visibility also
  }


  #ifdef HUMAN2_EXISTS_FOR_MA
  if(HUMAN2_HAS_MOVED==0&&NEED_HUMAN2_VISIBILITY_UPDATE==1)
  {
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_straight_head_orientation=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_straight_head_orientation_standing=0;
      
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_neck_turn=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_straight_head_orientation_standing=0;
       
  }
      
  if(HUMAN2_HAS_MOVED==1)
  {  
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_standing_human2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_human2_straight_head_orientation_standing=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human2_LHand=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human2_RHand=0;
     
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human2_LHand_by_bending=0;  
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human2_RHand_by_bending=0;  

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human2_LHand_by_turning_around_bending=0; 
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_human2_RHand_by_turning_around_bending=0;
     
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_neck_turn=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_standing_human2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_human2_straight_head_orientation_standing=0;

  NEED_HUMAN2_VISIBILITY_UPDATE=1; //To update visibility also
  }
  #endif

  if(HRP2_HAS_MOVED==0&&NEED_HRP2_VISIBILITY_UPDATE==1)
  {
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_neck_turn=0;

    
      
  }
 
  if(HRP2_HAS_MOVED==1)
  {  
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_HRP2_straight_head_orientation=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_LHand=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_HRP2_RHand=0;
     

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2_neck_turn=0;

  NEED_HRP2_VISIBILITY_UPDATE=1; //To update visibility also
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=0;  
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=0;  

  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=0; 
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=0;
  }
    

  if(JIDO_HAS_MOVED==0&&NEED_JIDO_VISIBILITY_UPDATE==1)
  {
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO_neck_turn=0;
  }

  if(JIDO_HAS_MOVED==1)
  {  
     
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_neck_turn=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.visible_by_JIDO_straight_head_orientation=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_JIDO_Hand=0;

  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO_straight_head_orientation=0;
  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO_neck_turn=0;
     
  NEED_JIDO_VISIBILITY_UPDATE=1; //To update visibility also
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_bending=0;  
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_bending=0;  

  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_LHand_by_turning_around_bending=0; 
  ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_Map.reachable_by_RHand_by_turning_around_bending=0;
  }
  }
  }
  }   
  //// }
  //////////ChronoPrint("Time for update_3D_grid_for_Mightability_Maps_new()");
  //////////printf(" After update_3D_grid_for_Mightability_Maps_new() \n");
  double tu,ts;
  ChronoTimes(&tu,&ts);
  //////////printf(" tu=%lf, ts=%lf \n",tu,ts);
  total_time+=tu;//In sec
  ChronoOff();

  //////////printf(" **** 3D grid dimension is (%d x %d x %d) cells.  \n",grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny,grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz);

  double cur_h_angle;
  double interval=grid_around_HRP2.GRID_SET->pace;
  double yaw;
  double pitch;
  double orig_pan;
  double orig_tilt;
  double fixed_pitch=M_PI/8.0;

  //////////printf(" NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);

  // //////////////////////tmp for jido
  // ChronoOn();
  // grid_3d_affordance_calculated=1; 
  // 
  // 
  // ////p3d_rob *cur_rob=ACBTSET->robot;
  // cur_h_angle=cur_rob->cam_h_angle;
  // interval=grid_around_HRP2.GRID_SET->pace;
  // printf("cur_h_angle=%lf\n",cur_h_angle);
  // no_FOV_end_point_vertices=0;
  // while(cur_rob->cam_h_angle>0.001)
  // { 
  // cur_rob->cam_h_angle-=interval;
  //  
  // get_points_on_FOV_screen(cur_rob);
  // }
  // cur_rob->cam_h_angle=cur_h_angle;
  // ////ChronoOn();
  // update_3d_grid_visibility(3); //2 for JIDO
  // 
  // ChronoOff();
  // 
  // update_3d_grid_reachability_for_JIDO_new();
  // 
  // 
  // return 1;
  // ///////////////////////end tmp for jido


  if(NEED_HUMAN1_VISIBILITY_UPDATE==1)
  {

  ChronoOn();

  //*****AKP: Tmp uncomment/comment because of problem in new human model for sitting as well as standing
  virtually_update_human_state_new(1);// Sitting


  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);

  //int i_h_a=0;
  ////////interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }

  human->cam_h_angle=cur_h_angle;
  ////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
  ////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
  ////ChronoOn();	    

  update_3d_grid_visibility(1); //1 for human

  //////////ChronoPrint("TIME of 3D Visibility calculation for sitting Human from current position");

  ////return 1;

  //Now making the head straight
  configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); 

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  double yaw=0.0;
  double pitch=fixed_pitch;
  double orig_pan=hum_cur_pos[HUMANq_PAN];
  double orig_tilt=hum_cur_pos[HUMANq_TILT];
  //////////printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw/pan angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_straight_visibility(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];


  //Now turning the head only
  //////configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof); 

  //////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  yaw=M_PI/3.0;
  pitch=fixed_pitch;
  //////orig_pan=hum_cur_pos[HUMANq_PAN];
  //////printf(" Original pan = %lf \n",orig_pan); 
  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

  hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  ////double tu,ts;
  //////////ChronoPrint("TIME for all 3D Visibility calculation for sitting Human");
  ChronoTimes(&tu,&ts);
  //////////printf(" tu=%lf, ts=%lf \n",tu,ts);
  total_time+=tu;//In sec
  ChronoOff();
 
  ////return 1;
  #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
  ///////////////3D visibility calculation for making human virtually standing//////////////////////////

  ChronoOn();
  virtually_update_human_state_new(0);// Standing
  cur_h_angle=human->cam_h_angle;
  //////////printf(" cur_h_angle for standing human=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }

  human->cam_h_angle=cur_h_angle;
  ////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
  ////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
  ////ChronoOn();	    

  update_3d_grid_visibility_standing(1); //1 for human

  ////////////ChronoPrint("TIME of 3D Visibility calculation for standing Human from current position");

  //Now making the head straight
  hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  yaw=0.0;
  pitch=fixed_pitch;

  orig_pan=hum_cur_pos[HUMANq_PAN];
  orig_tilt=hum_cur_pos[HUMANq_TILT];
  //////////printf(" Original pan = %lf \n",orig_pan); 
  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_straight_visibility_standing(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];



  //Now turning the head only
  //////hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  //////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  yaw=M_PI/3.0;
  pitch=fixed_pitch;
  //////orig_pan=hum_cur_pos[HUMANq_PAN];
  //////printf(" Original pan = %lf \n",orig_pan); 
  hum_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_PAN]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);

  hum_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=pitch; // Human Yaw angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  cur_h_angle=human->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human->cam_h_angle>0.001)
  { 
  human->cam_h_angle-=interval;
  get_points_on_FOV_screen(human);
  }
  update_3d_grid_visibility_by_neck_turn_standing(1); //1 for human
  human->cam_h_angle=cur_h_angle;

  hum_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->human[ACBTSET->actual_human]->HumanPt, hum_cur_pos);
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_PAN]=hum_cur_pos[HUMANq_PAN];
  ACBTSET->human[ACBTSET->actual_human]->HumanPt->ROBOT_POS[HUMANq_TILT]=hum_cur_pos[HUMANq_TILT];

  MY_FREE(hum_cur_pos,double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);


  //////////ChronoPrint("TIME for all 3D Visibility calculation for standing Human");
  ChronoTimes(&tu,&ts);
  ////printf(" %lf, %lf \n",tu,ts);
  virtually_update_human_state_new(1);// Again make it to Sitting
  ChronoTimes(&tu,&ts);
  //////////printf(" tu=%lf, ts=%lf \n",tu,ts);
  total_time+=tu;//In sec
  ChronoOff();

  ////NEED_HUMAN1_VISIBILITY_UPDATE=0;
  ///////////////END of 3D visibility calculation for making human virtually standing//////////////////////////
  #endif

  NEED_HUMAN1_VISIBILITY_UPDATE=0;
  }//END if(NEED_HUMAN_VISIBILITY_UPDATE==1)
  ////////return 1;



  if(NEED_HRP2_VISIBILITY_UPDATE==1||NEED_JIDO_VISIBILITY_UPDATE==1)
  {
  //////////printf(" Need visibility update for robot\n");
  ChronoOn();
  grid_3d_affordance_calculated=1; 


  ////p3d_rob *cur_rob=ACBTSET->robot;
  cur_h_angle=cur_rob->cam_h_angle;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }
  cur_rob->cam_h_angle=cur_h_angle;
  ////ChronoOn();
  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_visibility(3); //3 for JIDO
  #elif defined(HRI_HRP2)
  update_3d_grid_visibility(2); //2 for HRP2
  #endif


  //Now making the head straight
  configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

  orig_pan=rob_cur_pos[ROBOTq_PAN];
  orig_tilt=rob_cur_pos[ROBOTq_TILT];
  yaw=0.0;
  pitch=fixed_pitch;
  //////////printf(" Original pan = %lf, tilt = %lf \n",orig_pan, orig_tilt); 

  rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=pitch; // Human Pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  cur_h_angle=cur_rob->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }

  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_straight_visibility(3); //3 for JIOD
  #elif defined(HRI_HRP2)
  update_3d_grid_straight_visibility(2); //2 for HRP2
  #endif
  cur_rob->cam_h_angle=cur_h_angle;

  rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

  //Now turning the head only
  ////configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

  ////p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  yaw=M_PI/4.0;
  pitch=fixed_pitch;
  ////orig_pan=rob_cur_pos[ROBOTq_PAN];
  ////printf(" Original pan = %lf \n",orig_pan); 
  rob_cur_pos[ROBOTq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=pitch; // Human Pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  cur_h_angle=cur_rob->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }

  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
  #elif defined(HRI_HRP2)
  update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
  #endif

  cur_rob->cam_h_angle=cur_h_angle;

  rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

  rob_cur_pos[ROBOTq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=pitch; // Human Pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);

  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  cur_h_angle=cur_rob->cam_h_angle;
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(cur_rob->cam_h_angle>0.001)
  { 
  cur_rob->cam_h_angle-=interval;
  get_points_on_FOV_screen(cur_rob);
  }
  #ifdef JIDO_EXISTS_FOR_MA
  update_3d_grid_visibility_by_neck_turn(3); //3 for JIOD
  #elif defined(HRI_HRP2)
  update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
  #endif
  //////////update_3d_grid_visibility_by_neck_turn(2); //2 for HRP2
  cur_rob->cam_h_angle=cur_h_angle;

  rob_cur_pos[ROBOTq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  rob_cur_pos[ROBOTq_TILT]=orig_tilt; // Human Pitch angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(ACBTSET->robot, rob_cur_pos);
  ACBTSET->robot->ROBOT_POS[ROBOTq_PAN]=rob_cur_pos[ROBOTq_PAN];
  ACBTSET->robot->ROBOT_POS[ROBOTq_TILT]=rob_cur_pos[ROBOTq_TILT];

  MY_FREE(rob_cur_pos,double,ACBTSET->robot->nb_dof);


  //////////ChronoPrint("TIME of 3D Visibility calculation for ROBOT");
  ////ChronoOff();
  ////int HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  ////create_HRP2_robot(HRP2_state);
  ////g3d_draw_env();
  //// fl_check_forms();
  //// g3d_draw_allwin_active();

  ChronoTimes(&tu,&ts);
  //////////printf(" tu=%lf, ts=%lf \n",tu,ts);
  total_time+=tu;//In sec
  ChronoOff();

  NEED_HRP2_VISIBILITY_UPDATE=0;
  NEED_JIDO_VISIBILITY_UPDATE=0;

  }

 
  ChronoOn();
  //// create_HRP2_robot(HRP2_state);

  #ifdef JIDO_EXISTS_FOR_MA
  if(JIDO_HAS_MOVED==1)
  {
  update_3d_grid_reachability_for_JIDO_new();
  JIDO_HAS_MOVED=0;
  }
  #elif defined(HRI_HRP2)
  if(HRP2_HAS_MOVED==1)
  {
  update_3d_grid_reachability_for_HRP2_new();
  HRP2_HAS_MOVED=0;
  }
  #endif
  //////////update_3d_grid_reachability_for_HRP2_new();
  //////////ChronoPrint("****TIME of 3D reachability calculation for ROBOT\n");

  ChronoTimes(&tu,&ts);
  //////////printf(" tu=%lf, ts=%lf \n",tu,ts);
  total_time+=tu;//In sec

  ////return 1;
  ChronoOff();

  ChronoOn();

  if(HUMAN1_HAS_MOVED==1)
  {
  update_3d_grid_reachability_for_human_new(1);//1 for human1
 
  #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
 
  virtually_update_human_state_new(0);// Standing
  update_3d_grid_reachability_for_human_standing_new();
  virtually_update_human_state_new(1);
  #endif

  HUMAN1_HAS_MOVED=0;
  }

  //////////ChronoPrint("****TIME of 3D reachability calculation for human\n");
 
  ChronoTimes(&tu,&ts);
  //////////printf(" tu=%lf, ts=%lf \n",tu,ts);
  total_time+=tu;//In sec

  ChronoOff();

  //////////printf(" <<<<<<<<<< Total Time for updating all the Mightability Maps=%lf s >>>>>>>>>>\n",total_time);

  //   configPt visq;
  //  visq= MY_ALLOC(double,ACBTSET->visball->nb_dof); 
  //  p3d_get_robot_config_into(ACBTSET->visball,&visq);
  //  visq[6]=0.0;
  //  visq[7]=0.0;
  //  visq[8]=0.0;
  //   
  //  p3d_set_and_update_this_robot_conf(ACBTSET->visball, visq);


  #ifdef HUMAN2_EXISTS_FOR_MA
  if(NEED_HUMAN2_VISIBILITY_UPDATE==1)
  {
  int human_2_index;
  human_2_index=get_index_of_robot_by_name("ACHILE_HUMAN2");
  
  p3d_rob *human2=envPt_MM->robot[human_2_index];

  ChronoOn();


  virtually_update_non_primary_human_state(1, human_2_index);// Sitting


  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);

  //int i_h_a=0;
  ////////interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }

  human2->cam_h_angle=cur_h_angle;
  ////int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also
  ////create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
  ////ChronoOn();	    

  update_3d_grid_visibility(4); //4 for human 2

  //////////ChronoPrint("TIME of 3D Visibility calculation for sitting Human from current position");

  ////return 1;

  //Now making the head straight
  configPt hum2_cur_pos = MY_ALLOC(double,envPt_MM->robot[human_2_index]->nb_dof); 

  p3d_get_robot_config_into(envPt_MM->robot[human_2_index],&hum2_cur_pos);


  double yaw=0.0;
  double pitch=fixed_pitch;
  double orig_pan=hum2_cur_pos[HUMANq_PAN];
  double orig_tilt=hum2_cur_pos[HUMANq_TILT];
  //////////printf(" Original pan = %lf, tilt= %lf \n",orig_pan, orig_tilt); 

  hum2_cur_pos[HUMANq_PAN]=yaw; // Human Yaw/pan angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }
  update_3d_grid_straight_visibility(4); //4 for human2
  human2->cam_h_angle=cur_h_angle;

  hum2_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  //Now turning the head only
  //////configPt hum_cur_pos = MY_ALLOC(double,ACBTSET->human[ACBTSET->actual_human]->HumanPt->nb_dof);

  //////p3d_get_robot_config_into(ACBTSET->human[ACBTSET->actual_human]->HumanPt,&hum_cur_pos);


  yaw=M_PI/3.0;
  pitch=fixed_pitch;
  //////orig_pan=hum_cur_pos[HUMANq_PAN];
  //////printf(" Original pan = %lf \n",orig_pan); 
  hum2_cur_pos[HUMANq_PAN]=yaw; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }
  update_3d_grid_visibility_by_neck_turn(4); //4 for human2
  human2->cam_h_angle=cur_h_angle;

  hum2_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  p3d_get_robot_config_into(envPt_MM->robot[human_2_index],&hum2_cur_pos);

  hum2_cur_pos[HUMANq_PAN]=-yaw; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=pitch; // Human pitch/tilt angle relative to the human body frame

  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  cur_h_angle=human2->cam_h_angle;
  //////////printf(" cur_h_angle=%lf\n",cur_h_angle);
  //int i_h_a=0;
  interval=grid_around_HRP2.GRID_SET->pace;
  no_FOV_end_point_vertices=0;
  while(human2->cam_h_angle>0.001)
  { 
  human2->cam_h_angle-=interval;
  get_points_on_FOV_screen(human2);
  }
  update_3d_grid_visibility_by_neck_turn(4); //4 for human2
  human2->cam_h_angle=cur_h_angle;

  hum2_cur_pos[HUMANq_PAN]=orig_pan; // Human Yaw angle relative to the human body frame
  hum2_cur_pos[HUMANq_TILT]=orig_tilt; // Human Yaw angle relative to the human body frame
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[human_2_index], hum2_cur_pos);
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_PAN]=hum2_cur_pos[HUMANq_PAN];
  envPt_MM->robot[human_2_index]->ROBOT_POS[HUMANq_TILT]=hum2_cur_pos[HUMANq_TILT];

  MY_FREE(hum2_cur_pos,double,envPt_MM->robot[human_2_index]->nb_dof);

  NEED_HUMAN2_VISIBILITY_UPDATE=0;
  ChronoOff();
  }

  ////printf(" val of HUMAN2_HAS_MOVED = %d \n", HUMAN2_HAS_MOVED);

  if(HUMAN2_HAS_MOVED==1)
  {
  update_3d_grid_reachability_for_human_new(4);//4 for human2

 
  //  #ifdef MM_FOR_VIRTUALLY_STANDING_HUMAN
  //  
  //  virtually_update_human_state_new(0);// Standing
  // update_3d_grid_reachability_for_human_standing_new();
  //  virtually_update_human_state_new(1);
  //  #endif


  HUMAN2_HAS_MOVED=0;
  }

  #endif

  ACBTSET->visball->joints[1]->abs_pos[0][3]=0;
  ACBTSET->visball->joints[1]->abs_pos[1][3]=0;
  ACBTSET->visball->joints[1]->abs_pos[2][3]=0;


  ////ChronoOff();

  return 1;

  //return 1;

  // ChronoPrint("Time before calculating affordance on surfaces ");
  //  //current_surface_index=0;
  //  int i=0;
  //  for(i=0;i<curr_surfaces_in_env.total_no_of_surfaces;i++)
  //  {
  //  printf(" For surface %d\n",i);
  //  ChronoPrint("Time Before update_surface_grid_based_on_curr_pos");
  //  update_surface_grid_based_on_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  // 
  //  ChronoPrint("Time Before update_surface_grid_by_bending_human_at_curr_pos");
  //  update_surface_grid_by_bending_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  // 
  //  ChronoPrint("Time Before update_surface_grid_by_turning_human_at_curr_pos");
  //  update_surface_grid_by_turning_human_at_curr_pos(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  //  
  //  
  // 
  //  
  //  
  //  HRP2_state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is standing 
  //  //////ChronoPrint(" Time before create_HRP2_robot");
  //  //////create_HRP2_robot(HRP2_state);
  //  //////ChronoPrint(" Time after create_HRP2_robot and before update_surface_grid_for_HRP2_without_GIK");
  //  ////update_surface_grid_for_HRP2_with_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  //  update_surface_grid_for_HRP2_without_GIK(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[i]);
  //  ChronoPrint(" Time after update_surface_grid_for_HRP2_without_GIK");

  // }

  ////ChronoPrint("TIME for all affordance calculation");
  ////ChronoOff();
  ////create_3d_grid_for_HRP2_GIK();
  ////update_3D_grid_based_on_current_human_pos();
  //// grid_3d_affordance_calculated=1; 
 
  //update_surface_grid(surf_grid_samp_rate, &curr_surfaces_in_env.flat_surf[1]);
  }
  #endif


  // AKP: To draw a cylinder
  void g3d_draw_vertical_cylinder(double x, double y, double z, double radius, double height, double slice_distance, int color, double *color_vect)
  {
  double i=z;
  for(;i<z+height; i+=slice_distance)
  {
  g3d_drawDisc(x, y, i, radius, color, color_vect);
  }
  }



  //AKP : To show the different affordance values 
  int show_affordance_new()
  {
  no_candidate_points_to_put=0;

  int current_surface_index=0;
  for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
  {

  int no_vert=0;
  for(;no_vert<curr_surfaces_in_env.flat_surf[current_surface_index].no_vertices-1;no_vert++)
  {
  g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert+1].z,4,NULL);
  }
  g3d_drawOneLine(curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[no_vert].z,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].x,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].y,curr_surfaces_in_env.flat_surf[current_surface_index].vertices[0].z,4,NULL);


  ////printf(" Inside show_affordance grid_i_max =%d , grid_j_max = %d \n", curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max, curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max);
  int i=0;
  for(;i<curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max;i++)
  {
  int j=0;
  int show_cell=0;
  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;
  //printf("\n");
  for(;j<curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max;j++)
  {
  show_cell=0;
  reachable_by_hand=0;
  reachable_by_HRP2_hand=0;
  visible_by_human=0;
  visible_by_HRP2=0;
  reachable_by_turning_around=0;
  reachable_by_bending=0;

  double x=i*surf_grid_samp_rate;
  double y=j*surf_grid_samp_rate;
  x+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_x_min;
  y+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_y_min;
  double z=curr_surfaces_in_env.flat_surf[current_surface_index].BR_z;

  double cur_z_st=z;
  double height=0.1;//0.3;
  double cur_z_end=z;
  //double cur_z_end=;
  //printf("%d ", curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible);
  ////printf("SHOW_VISIBLE_PLACE=%d\n",SHOW_VISIBLE_PLACE);
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible==1)
  {
 
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
  //cur_z_st+=cur_z_end;
  //cur_z_end+=cur_z_st;
  show_cell=1;
  visible_by_human=1;

  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible_by_HRP2==1)
  {
 
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
  //cur_z_st+=cur_z_end;
  //cur_z_end+=cur_z_st;
  show_cell=1;
  visible_by_HRP2=1;
  }

  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_RHand==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
  show_cell=1;
  reachable_by_hand=1;
  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_LHand==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
  show_cell=1;
  if(reachable_by_hand==1)//Already reachable by right hand
  reachable_by_hand=3; //Reachable by both hands
  else
  reachable_by_hand=2; //Reachable by left hand only
  
  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_RHand_by_bending==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
  show_cell=1;
  reachable_by_bending=1;
  //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_LHand_by_bending==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
  show_cell=1;
  if(reachable_by_bending==1)//Already reachable by right hand
  reachable_by_bending=3; //Reachable by both hands
  else
  reachable_by_bending=2; //Reachable by left hand only
  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_RHand_by_turning_around_bending==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
  show_cell=1;
  reachable_by_turning_around=1;
  ////   g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_LHand_by_turning_around_bending==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
  show_cell=1;

  if(reachable_by_turning_around==1)//Already reachable by right hand
  reachable_by_turning_around=3; //Reachable by both hands
  else
  reachable_by_turning_around=2; //Reachable by left hand only
  }

  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_RHand==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
  show_cell=1;
  reachable_by_HRP2_hand=1;
  //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
   
  }
  if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_LHand==1)
  {
  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
  show_cell=1;
  if(reachable_by_HRP2_hand==1)//Already reachable by right hand
  reachable_by_HRP2_hand=3; //Reachable by both hands
  else
  reachable_by_HRP2_hand=2; //Reachable by left hand only
  }
    
  if(SHOW_2D_VISIBLE_PLACE_HUM==1&&visible_by_human==1)
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
    
  }

  if(SHOW_2D_VISIBLE_PLACES_FOR_HRP2==1&&visible_by_HRP2==1)
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
    
  }

  if(SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN==1&&visible_by_human==1&&visible_by_HRP2==1)
  {
  //printf("SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN=%d, SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN=%d\n",SHOW_2D_COMMON_VISIBLE_HRP2_HUMAN,SHOW_3D_COMMON_VISIBLE_HRP2_HUMAN);
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Green, NULL);
  }

  if(SHOW_2D_DIRECT_REACHABLE_HUM==1)
  {
  if(reachable_by_hand==1)
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
  else
  {
  if(reachable_by_hand==2)
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
  }
  else
  {
  if(reachable_by_hand==3) //Reachable by both hands
  { 
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
  }
  //else
  //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
  }
  }
  }
     
  if(SHOW_2D_BENDING_REACHABLE_HUM==1)
  {
  if(reachable_by_bending==1)
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
  else
  {
  if(reachable_by_bending==2)
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
  }
  else
  {
  if(reachable_by_bending==3) //Reachable by both hands
  { 
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
  }
  //else
  //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
  }
  }
  }
   
  if(SHOW_2D_TURNING_AROUND_REACHABLE_HUM==1)
  {
  if(reachable_by_turning_around==1)
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
  else
  {
  if(reachable_by_turning_around==2)
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
  }
  else
  {
  if(reachable_by_turning_around==3) //Reachable by both hands
  { 
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
  }
  //else
  //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
  }
  }
  }
    
  if(SHOW_2D_DIRECT_REACHABLE_HRP2==1)
  {
  if(reachable_by_HRP2_hand==1)
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 1, NULL);
  else
  {
  if(reachable_by_HRP2_hand==2)
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
  }
  else
  {
  if(reachable_by_HRP2_hand==3) //Reachable by both hands
  { 
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
  }
  //else
  //g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 3, NULL);
  }
  }
  }
    
  if(SHOW_2D_COMMON_REACH_HRP2_HUMAN==1)
  {
    
  if(reachable_by_HRP2_hand==3&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3)) //Reachable by both hands of both human and HRP2
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Green, NULL);
  }
  else
  {
  if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Red, NULL);
  }
  else
  {
  if((reachable_by_HRP2_hand==3)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Yellow, NULL);
  }
  else
  {
  if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3))
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, Blue, NULL);
  }
  } 
  }
  }
  } 
   
  if(SHOW_HRP2_HUMAN_COMMON_REACHABLE_VISIBLE==1)
  {
  if(visible_by_HRP2==1&&visible_by_human==1)
  {
  if(reachable_by_HRP2_hand==3&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3)) //Reachable by both hands of both human and HRP2
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
     
  candidate_points_to_put.point[no_candidate_points_to_put].x=x;
  candidate_points_to_put.point[no_candidate_points_to_put].y=y;
  candidate_points_to_put.point[no_candidate_points_to_put].z=z;
  candidate_points_to_put.no_points++;
  no_candidate_points_to_put++;
     
  }
  if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
  {
  g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
     
  candidate_points_to_put.point[no_candidate_points_to_put].x=x;
  candidate_points_to_put.point[no_candidate_points_to_put].y=y;
  candidate_points_to_put.point[no_candidate_points_to_put].z=z;
  candidate_points_to_put.no_points++;
  no_candidate_points_to_put++;
  }
  } 
  }

  
   
   
  }
  }
  }//END for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)

  // int no_path_pts=0;
  // for(no_path_pts=0;no_path_pts<cur_manipulation_path.total_no_pts;no_path_pts++)
  //  {
  //  g3d_drawDisc(cur_manipulation_path.path_points[no_path_pts].x, cur_manipulation_path.path_points[no_path_pts].y, cur_manipulation_path.path_points[no_path_pts].z, 0.01, 4, NULL);
  //  }


  }
*/


double get_cell_value_3D_grid(hri_bitmapset * btset, int x, int y, int z)
{
  if(btset == NULL){
    PrintWarning(("Cant get obstacle value: btset=null"));
    return -1;
  }
  if(btset->bitmap[HRP2_GIK_MANIP] == NULL){
    PrintWarning(("Cant get obstacle value: bitmap=null"));
    return -1;
  }

  /*if(btset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val == -1)
    return -1;
    else
    return 0;
  */
  return btset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val;
}

int hri_bt_initialize_affordance_data(hri_bitmapset* bitmapset,int bt_type)
{
  int hum_vis_states_ctr=0;
  int hum_reach_states_ctr=0;
  int HRP2_vis_states_ctr=0;
  int HRP2_reach_states_ctr=0;
  int JIDO_vis_states_ctr=0;
  int JIDO_reach_states_ctr=0;

  int x,y,z;
  for(x=0; x<bitmapset->bitmap[bt_type]->nx; x++)
    {
      for(y=0; y<bitmapset->bitmap[bt_type]->ny; y++)
	{
	  for(z=0; z<bitmapset->bitmap[bt_type]->nz; z++)
	    {
	      for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	      {
		for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
		{
	          bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible[i][j] = 0;
                  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible[i][j] =0;
		}
		for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		{
		  for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
		  {
		    bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable[i][j][k] = 0;
		  }
		}
	      }
//               for(hum_vis_states_ctr=0;hum_vis_states_ctr<MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN;hum_vis_states_ctr++)
//              {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human[hum_vis_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human[hum_vis_states_ctr] =0;
//              }
// 	      
//               for(hum_reach_states_ctr=0;hum_reach_states_ctr<MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN;hum_reach_states_ctr++)
//              {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[hum_reach_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[hum_reach_states_ctr] = 0;
//              }
// 
// 	      
// 	      #ifdef HUMAN2_EXISTS_FOR_MA
//               for(hum_vis_states_ctr=0;hum_vis_states_ctr<MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN;hum_vis_states_ctr++)
//               {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_human2[hum_vis_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human2[hum_vis_states_ctr] =0;
//               } 
//                
//                for(hum_reach_states_ctr=0;hum_reach_states_ctr<MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN;hum_reach_states_ctr++)
//               {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand[hum_reach_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand[hum_reach_states_ctr] = 0;
//               }
// 
//               #endif
// 
//               #ifdef HRP2_EXISTS_FOR_MA
// 	      //For HRP2
//               for(HRP2_vis_states_ctr=0;HRP2_vis_states_ctr<MAXI_NUM_POSSIBLE_STATES_VIS_HRP2;HRP2_vis_states_ctr++)
//              {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_HRP2[HRP2_vis_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_HRP2[HRP2_vis_states_ctr] =0;
//              }
// 	      
//               for(HRP2_reach_states_ctr=0;HRP2_reach_states_ctr<MAXI_NUM_POSSIBLE_STATES_REACH_HRP2;HRP2_reach_states_ctr++)
//              {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand[HRP2_reach_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand[HRP2_reach_states_ctr] = 0;
//              }
//               #endif
// 
//               #ifdef JIDO_EXISTS_FOR_MA
// 	      //For Jido
//               for(JIDO_vis_states_ctr=0;JIDO_vis_states_ctr<MAXI_NUM_POSSIBLE_STATES_VIS_JIDO;JIDO_vis_states_ctr++)
//              {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible_by_JIDO[JIDO_vis_states_ctr] = 0;
//               bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_JIDO[JIDO_vis_states_ctr] =0;
//              }
// 	      
//               for(JIDO_reach_states_ctr=0;JIDO_reach_states_ctr<MAXI_NUM_POSSIBLE_STATES_REACH_JIDO;JIDO_reach_states_ctr++)
//              {
// 	      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand[JIDO_reach_states_ctr] = 0;
//               
//              }
// 	     
//               #endif 

	    }
	}
    }   
}

int create_Mightability_data_fields(  hri_bitmapset * btset, int bt_type)
{
  
  int x,y,z;
  for(x=0; x<btset->bitmap[bt_type]->nx; x++)
    {
      for(y=0; y<btset->bitmap[bt_type]->ny; y++)
	{
	  for(z=0; z<btset->bitmap[bt_type]->nz; z++)
	    {
	      for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	      {
		
		  btset->bitmap[bt_type]->data[x][y][z].Mightability_Map.visible[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
		  
		  btset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
		  
		  btset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable[i]=MY_ALLOC(int*, agents_for_MA_obj.for_agent[i].maxi_num_reach_states);
		  
		for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		{
		btset->bitmap[bt_type]->data[x][y][z].Mightability_Map.reachable[i][j]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].no_of_arms);
      
		}
		
	      }
	    }
	}
    }
 
}

hri_bitmapset* create_3D_grid(int BB_length,int BB_width,int BB_height, double sampling_rate)
{
  ////////////ChronoPrint("<<<<<<<<Entering create_3D_grid");
  int dimx,dimy, dimz;
  configPt humanConf;
  double hx,hy,hz;
  double Ccoord[6];
  
  double xsize=6,ysize=6,zsize=3;

  hri_bitmapset * btset;
  btset = hri_bt_create_bitmaps();

  btset->pace = sampling_rate;

  dimx = (int)(BB_length/sampling_rate);
  dimy = (int)(BB_width/sampling_rate);
  dimz = (int)(BB_height/sampling_rate);

  btset->n = 1;
  btset->bitmap = MY_ALLOC(hri_bitmap*,btset->n);
  
  
  btset->bitmap[HRP2_GIK_MANIP]=hri_bt_create_empty_bitmap(dimx, dimy, dimz, sampling_rate, HRP2_GIK_MANIP, get_cell_value_3D_grid); // AKP : get_cell_value_3D_grid is the function name passed as the argument and will be assigned to the function calculate_cell_value of the btset.
  ////btset->bitmap[BT_AFFORDANCE_VISIBILITY]=hri_bt_create_empty_bitmap(dimx, dimy, dimz, sampling_rate, BT_AFFORDANCE_VISIBILITY, get_cell_value_3D_grid); // AKP : get_cell_value_3D_grid is the function name passed as the argument and will be assigned to the function calculate_cell_value of the btset.
  int i=0;

  for(;i<btset->n;i++)
    {
      if(btset->bitmap[i] == NULL)
	{
	  printf("AKP WARNING : Could not create the empty bitmap %d inside the function create_3D_grid, so returning with NULL \n",i);
	  return NULL;
	}
    }
  // create all cells
  for(i=0;i<btset->n;i++)
    {
      hri_bt_create_data(btset->bitmap[i]); // AKP : Creates necessary data field for an empty bitmap and populate with the value 0
      create_Mightability_data_fields(btset,i);
      hri_bt_initialize_affordance_data(btset,i);
    }
  
  

  btset->path = NULL;
  btset->pathexist = FALSE;
  btset->combine_type = BT_COMBINE_SUM; /* default value */
  btset->changed = FALSE;

  

  //hri_exp_fill_obstacles(btset);
  ////////////ChronoPrint(">>>>>>>>>>>>Returning create_3D_grid");
  return btset;
}

//AKP : For populating the obstacles in the bitmap 

int create_obstacles_for_HRP2_GIK_manip( hri_bitmapset* btset )
{
  int i;
  p3d_env* env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int expand_rate, minimum_expand_rate;
  configPt robotq;
	
  if(btset == NULL)
    return FALSE;
  
  
  
  if(btset->robot == NULL)
    {
      expand_rate = 0;
      PrintWarning(("WARNING: btset->robot NULL\n"));
    }
  /*else {
    robotq = p3d_get_robot_config(btset->robot);
    //  expand_rate = (bitmapset->robot->BB.xmax-bitmapset->robot->BB.xmin>bitmapset->robot->BB.ymax-bitmapset->robot->BB.ymin)? 
    //       ((bitmapset->robot->BB.xmax-bitmapset->robot->BB.xmin)/(bitmapset->bitmap[BT_OBSTACLES]->pace*2)): 
    //       ((bitmapset->robot->BB.ymax-bitmapset->robot->BB.ymin)/(bitmapset->bitmap[BT_OBSTACLES]->pace*2)); 
    if(DISTANCE2D(btset->robot->BB.xmax,btset->robot->BB.ymax,robotq[ROBOTq_X],robotq[ROBOTq_Y]) >
    DISTANCE2D(btset->robot->BB.xmin,btset->robot->BB.ymin,robotq[ROBOTq_X],robotq[ROBOTq_Y]))
      
    expand_rate = (btset->robot->BB.xmax-robotq[ROBOTq_X] > btset->robot->BB.ymax-robotq[ROBOTq_Y])?
    ((btset->robot->BB.xmax-robotq[ROBOTq_X])/btset->pace):
    ((btset->robot->BB.ymax-robotq[ROBOTq_Y])/btset->pace);
    
    else
    expand_rate = (btset->robot->BB.xmin-robotq[ROBOTq_X] > btset->robot->BB.ymin-robotq[ROBOTq_Y])?
    ((robotq[ROBOTq_X]-btset->robot->BB.xmin)/btset->pace):
    ((robotq[ROBOTq_Y]-btset->robot->BB.ymin)/btset->pace);	 
    
    p3d_destroy_config(btset->robot,robotq);
    //expand_rate--;		
    }


    expand_rate=1;

    hri_bt_reset_bitmap_data(btset->bitmap[BT_OBSTACLES]);
    minimum_expand_rate = (int) (0.40/btset->pace) - 1; 	
  
    printf("expand rate %i min rate %i \n",expand_rate, minimum_expand_rate);

    if (expand_rate <=  minimum_expand_rate )
    expand_rate += minimum_expand_rate;
  */
  
  double expand_rate_for_HRP2_GIK=0.01;
  double minimum_expand_rate_for_HRP2_GIK=0.005;

  for(i=0; i<env->no ; i++){  
    //////////printf(" Inserting OBJECT %s \n",env->o[i]->name); 
    hri_bt_insert_obs(btset,btset->bitmap[HRP2_GIK_MANIP], env->o[i], env, minimum_expand_rate_for_HRP2_GIK, -2, 1);
    // potential 3d collision
    //hri_bt_insert_obs(btset,btset->bitmap[HRP2_GIK_MANIP], env->o[i], env, safe_expand_rate, BT_OBST_POTENTIAL_OBJECT_COLLISION, 0);
    //hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, expand_rate_for_voronoi, -1)  ;
    /* if(hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, expand_rate, -1)) */
    /* printf("Obstacle placed\n"); */
  }  

  /* for(i=0; i<env->no ; i++){                             
  // hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, (int)(expand_rate/2)+1, -2);
  hri_bt_insert_1obs2bitmap(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, minimum_expand_rate_for_voronoi, -2);
    
  }*/

  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  for(i=0; i<env->nr; i++){
    
    int is_robot=0;
    int is_human=0;
    int is_HRP2_chair=0;
    //printf(" Inserting ROBOT %s \n",env->robot[i]->name);
    if(!strncmp("HRP2ROBOT",env->robot[i]->name,9))
      is_robot=1;
    if((!strncmp("human",env->robot[i]->name,5))||(!strncmp("HUMAN",env->robot[i]->name,5)))
      is_human=1;
    if(!strncmp("HRP2CHAIR",env->robot[i]->name,9))
      is_HRP2_chair=1;
    
    if(is_robot==0&&is_human==0&& is_HRP2_chair==0)
      {
	////  if(dynamic_obj[i].is_curr_visible==1)
	////{
        //////////printf(" Inserting ROBOT %s \n",env->robot[i]->name);
	hri_bt_insert_obsrobot(btset, btset->bitmap[HRP2_GIK_MANIP], env->robot[i], env, minimum_expand_rate_for_HRP2_GIK, -2, 1);
	//hri_bt_insert_1obs2bitmaprobot(btset,btset->bitmap[BT_OBSTACLES],env->robot[i] , env, minimum_expand_rate_for_voronoi, -2);
	////printf("Obstacles updated for %s\n",env->robot[i]->name);
	////printf(" BB : %lf, %lf, %lf, %lf, %lf, %lf \n",env->robot[i]->BB.xmin,env->robot[i]->BB.xmax,env->robot[i]->BB.ymin,env->robot[i]->BB.ymax,env->robot[i]->BB.zmin,env->robot[i]->BB.zmax);
	////}
     
      }
  }
  
  return TRUE;  
}




int show_exact_obstacles_for_HRP2_GIK_manip(hri_bitmapset * bitmapset, int bt_type)
{
  if(bt_type==HRP2_GIK_MANIP)
    {
      double shift_for_cell_mid=bitmapset->pace/2.0;
      hri_bitmap * bitmap;
      bitmap = bitmapset->bitmap[bt_type];
      int x,y,z;
      for(x=0; x<bitmap->nx; x++)
	{
	  for(y=0; y<bitmap->ny; y++)
	    {
	      for(z=0; z<bitmap->nz; z++)
		{
		  if(bitmapset->bitmap[bt_type]->data[x][y][z].val < 0)  
		    {
		      double tmp_x  = x*bitmapset->pace+bitmapset->realx+shift_for_cell_mid;
		      double tmp_y  = y*bitmapset->pace+bitmapset->realy+shift_for_cell_mid;
		      double tmp_z  = z*bitmapset->pace+bitmapset->realz+shift_for_cell_mid;
		      if(bitmapset->bitmap[bt_type]->data[x][y][z].val == -3)  
			g3d_drawDisc(tmp_x, tmp_y, tmp_z, 0.01, 3, NULL);
		      else
		      {
			if(bitmapset->bitmap[bt_type]->data[x][y][z].val == -2)
			{  
			g3d_drawDisc(tmp_x, tmp_y, tmp_z, 0.01, 2, NULL);
			}
			else
			  g3d_drawDisc(tmp_x, tmp_y, tmp_z, 0.02, 4, NULL);
		      }	
		    }
		}
	    }
	}  
      //////////g3d_drawDisc(point_of_curr_collision.x, point_of_curr_collision.y,point_of_curr_collision.z, 0.1,4, NULL);
    } 
  return 1;
}


int update_robots_and_objects_status()
{
  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  //double cur_x;
  //double cur_y;
  //double cur_z;
  //////////HUMAN1_HAS_MOVED=0;
 int at_least_one_object_has_moved=0;
 
  for(r_ctr=0;r_ctr<nr;r_ctr++)
    {
      r = envPt_MM->robot[r_ctr];

      //double cur_x=r->ROBOT_POS[6];
      if(strcasestr(r->name,"visball"))
	{
	}
      else
	{
	  if(robots_status_for_Mightability_Maps[r_ctr].has_moved==0)
	    {
  
	      //////////if(fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]-r->ROBOT_POS[6])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]-r->ROBOT_POS[7])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]-r->ROBOT_POS[8])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[9]-r->ROBOT_POS[9])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[10]-r->ROBOT_POS[10])>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[11]-r->ROBOT_POS[11])>=0.01)
	      if(fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]-r->joints[1]->dof_data[0].v)>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]-r->joints[1]->dof_data[1].v)>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]-r->joints[1]->dof_data[2].v)>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[9]-r->joints[1]->dof_data[3].v)>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[10]-r->joints[1]->dof_data[4].v)>=0.01||fabs(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[11]-r->joints[1]->dof_data[5].v)>=0.01)
		{
    
		  robots_status_for_Mightability_Maps[r_ctr].has_moved=1;
                  at_least_one_object_has_moved=1;
		  ////////printf(" >>>> Robot = %s has moved.\n",r->name);
		  /*
		    robots_status_for_Mightability_Maps[r_ctr].prev_x=robots_status_for_Mightability_Maps[r_ctr].curr_x;
		    robots_status_for_Mightability_Maps[r_ctr].prev_y=robots_status_for_Mightability_Maps[r_ctr].curr_y;
		    robots_status_for_Mightability_Maps[r_ctr].prev_z=robots_status_for_Mightability_Maps[r_ctr].curr_z;
    
		    robots_status_for_Mightability_Maps[r_ctr].curr_x=r->ROBOT_POS[6];
		    robots_status_for_Mightability_Maps[r_ctr].curr_y=r->ROBOT_POS[7];
		    robots_status_for_Mightability_Maps[r_ctr].curr_z=r->ROBOT_POS[8];
		  */
		  /*
		  if(strcasestr(r->name,"HUMAN1"))
		    {
		      HUMAN1_HAS_MOVED=1;
		      ////NEED_HUMAN1_REACHABILITY_UPDATE=1;
		      ////NEED_ALL_REACHABILITY_UPDATE_AGENT[HUMAN1_MA]=1;
		    }
		  else
		    {
		      if(strcasestr(r->name,"HRP2_ROBOT"))
			{
			  HRP2_HAS_MOVED=1;
			  ////NEED_HRP2_REACHABILITY_UPDATE=1;
			}
		      else
			{
			  if(strcasestr(r->name,"JIDOKUKA_ROBOT"))
			    {
			      JIDO_HAS_MOVED=1;
			      ////NEED_JIDO_REACHABILITY_UPDATE=1;
			    }
#ifdef HUMAN2_EXISTS_FOR_MA
			  else
			    {
			      if(strcasestr(r->name,"HUMAN2"))
				{
				  ////printf(" <<<< **** setting HUMAN2_HAS_MOVED \n");
				  HUMAN2_HAS_MOVED=1;
				  ////NEED_HUMAN2_REACHABILITY_UPDATE=1;
				}
			    } 
#endif
			} 
		    }
		    */
		}
     
	    }
	}
    }
    
    for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
    {
      
      
      if(robots_status_for_Mightability_Maps[indices_of_MA_agents[i]].has_moved==1)
      {
       ///////////////printf(" >>>> NEED_ALL_REACHABILITY_UPDATE_AGENT %d\n",i);
	NEED_ALL_REACHABILITY_UPDATE_AGENT[i]=1;
	////NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=1;
      }
      
      if(at_least_one_object_has_moved==1)
      {
      NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=1;
      }
      else
      {
      for(int j=0;j<agents_for_MA_obj.for_agent[i].head_params.no_joints_neck;j++)
       {
      if(fabs(robots_status_for_Mightability_Maps[indices_of_MA_agents[i]].rob_prev_config[agents_for_MA_obj.for_agent[i].head_params.joint_indices[j]]-r->joints[agents_for_MA_obj.for_agent[i].head_params.joint_indices[j]]->dof_data[0].v)>=0.01)
        {
	 NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;
	 ////MA_agent_has_turned_head[i]=1;
	
       break;
        }
       }
      }
    }
     
    
  return 1;
}



int update_3D_grid_for_Mightability_Maps(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  configPt visq;
  configPt cur_rob_pos;
  hri_bitmap * bitmap;
  double increment=3.0/4.0*bitmapset->pace;
  visq = p3d_get_robot_config(bitmapset->visball);
  bitmap = bitmapset->bitmap[bt_type];
  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  
  int x,y,z;

  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  for(r_ctr=0;r_ctr<nr;r_ctr++)
    {
      if(robots_status_for_Mightability_Maps[r_ctr].has_moved==1)
	{
  
	  r = envPt_MM->robot[r_ctr];

	  cur_rob_pos=MY_ALLOC(double,r->nb_dof); 
	  p3d_get_robot_config_into(r,&cur_rob_pos);
  
	  //////////printf(" Robot name = %s \n",r->name);
/*
	  NEED_HUMAN1_ALL_VISIBILITY_UPDATE=1;
#ifdef HUMAN2_EXISTS_FOR_MA
	  NEED_HUMAN2_ALL_VISIBILITY_UPDATE=1;
#endif
#ifdef JIDO_EXISTS_FOR_MA
	  NEED_JIDO_ALL_VISIBILITY_UPDATE=1;
#endif
#ifdef HRP2_EXISTS_FOR_MA
	  NEED_HRP2_ALL_VISIBILITY_UPDATE=1;
#endif
  */
	  //////////// printf(" inside update_3D_grid_for_Mightability_Maps, NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);
	  ////**** First making previously occupied cells as free cells  
	    p3d_set_and_update_this_robot_conf(r, robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
	    p3d_col_deactivate_rob_rob(bitmapset->visball,r);//To make a cell occupied only if there is collision of visball with other objects not with robot
	    int r_o_ctr=0; 
	    for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	      {
		o = r->o[r_o_ctr];
		////printf(" o->name = %s \n",o->name);
		double BBx;
		for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		  {
		    visq[6] = BBx;
		    x=(BBx- bitmapset->realx)/bitmapset->pace;  
		    double BBy;
		    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		      {
			visq[7]  = BBy;
			y=(BBy- bitmapset->realy)/bitmapset->pace;  
			double BBz;
			for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			  {
			    visq[8]  = BBz;
			    z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			    if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			      {
				////bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle

				p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
				int kcd_with_report=0;
				int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
				if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				  {
				    //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
				    bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
        
        
        
				    int i=0; 
				    for(i=x-expansion;i<=x+expansion;i++)
				      {
					//printf(" for i = %d\n",i);
					if(i>=0&&i<bitmap->nx)
					  {
					    int j=0;
					    for(j=y-expansion;j<=y+expansion;j++)
					      {
						//printf(" for i, j = %d %d\n",i, j);
						if(j>=0&&j<bitmap->ny)
						  {
						    int k=0;
						    for(k=z-expansion;k<=z+expansion;k++)
						      {
							//printf(" for i, j, k = %d %d %\n",i, j, k);
							if(k>=0&&k<bitmap->nz)
							  {
							    // printf(" Populating %d, %d, %d \n",i,j,k);
							    if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
							      {
							      }
							    else
							      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
							  }
						      } 
						  }
					      }
					  } 
				      } 
				  }
				else
				  {
        
				    /* 
				       bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //No obstacle
        
      
				       int i=0; 
				       for(i=x-expansion;i<=x+expansion;i++)
				       {
				       //printf(" for i = %d\n",i);
				       if(i>=0&&i<bitmap->nx)
				       {
				       int j=0;
				       for(j=y-expansion;j<=y+expansion;j++)
				       {
				       //printf(" for i, j = %d %d\n",i, j);
				       if(j>=0&&j<bitmap->ny)
				       {
				       int k=0;
				       for(k=z-expansion;k<=z+expansion;k++)
				       {
				       //printf(" for i, j, k = %d %d %\n",i, j, k);
				       if(k>=0&&k<bitmap->nz)
				       {
				       // printf(" Populating %d, %d, %d \n",i,j,k);
				       bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //No obstacle
				       }
				       } 
				       }
				       }
				       } 
				       } 
				    */
       
				  }
			      }
			  }
		      }
		  }
	      }
	  p3d_col_activate_rob_rob(bitmapset->visball,r);
	  p3d_set_and_update_this_robot_conf(r, cur_rob_pos);
	  ////*****END making the previously occupied cells as free cells  

	    ////*****Now creating exact obstacle at new place of robot
	    ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	    r_o_ctr=0; 
	    for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	      {
		o = r->o[r_o_ctr];
		////printf(" o->name = %s \n",o->name);
		double BBx;
		for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		  {
		    visq[6] = BBx;
		    x=(BBx- bitmapset->realx)/bitmapset->pace;  
		    double BBy;
		    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		      {
			visq[7]  = BBy;
			y=(BBy- bitmapset->realy)/bitmapset->pace;  
			double BBz;
			for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			  {
			    visq[8]  = BBz;
			    z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			    if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			      {
				p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
				int kcd_with_report=0;
				int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
				if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				  {
				    //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
				    bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
       
      
				    //else
				    //{
				    //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				    // }
          
				    int i=0; 
				    for(i=x-expansion;i<=x+expansion;i++)
				      {
					//printf(" for i = %d\n",i);
					if(i>=0&&i<bitmap->nx)
					  {
					    int j=0;
					    for(j=y-expansion;j<=y+expansion;j++)
					      {
						//printf(" for i, j = %d %d\n",i, j);
						if(j>=0&&j<bitmap->ny)
						  {
						    int k=0;
						    for(k=z-expansion;k<=z+expansion;k++)
						      {
							//printf(" for i, j, k = %d %d %\n",i, j, k);
							if(k>=0&&k<bitmap->nz)
							  {
							    // printf(" Populating %d, %d, %d \n",i,j,k);
							    if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
							      {
							      }
							    else
							      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
							  }
						      } 
						  }
					      }
					  } 
				      } 
				  }
				////else
				////{
				////printf("Inside BB but no collison\n");
				////exit(0);
				////}
			      }
			  }   
		      }
		  }
	      }
 
	    robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
	    ////p3d_set_and_update_this_robot_conf(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config,r->ROBOT_POS);
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]=r->ROBOT_POS[6];
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]=r->ROBOT_POS[7];
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]=r->ROBOT_POS[8];
	    p3d_copy_config_into(r,cur_rob_pos, &(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config));
  
	    ////////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
	}
   
    }
  

  return 1;
}


int update_3D_grid_for_Mightability_Maps_new(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  if(bitmapset==NULL)
  {
   printf(" AKP ERROR : bitmapset=NULL\n");
   return 0;
  }

  int x,y,z;
 hri_bitmap * bitmap;

//Tmp for test commet it
/*
      bitmap = bitmapset->bitmap[HRP2_GIK_MANIP];
int object_index=get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE);
  for(x=0; x<bitmap->nx; x++)
	{
	  for(y=0; y<bitmap->ny; y++)
	    {
	      for(z=0; z<bitmap->nz; z++)
		{
                   if(bitmapset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]==1)
		  ////if(bitmapset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val < 0)  
		    {
                      
		     printf("  cell (x, y, z) = (%d %d %d) belongs to %s\n",x, y, z,envPt_MM->robot[object_index]->name);
                     printf("no_of_objects_belogs to this cell=%d\n", grid_around_HRP2.GRID_SET->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]);
                                      
                                     	      
		    }
		}
	    }
	}  
*/
//End tmp for tets

  configPt visq;
  configPt cur_rob_pos;
 
  double increment=3.0/4.0*bitmapset->pace;
  visq = p3d_get_robot_config(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
  bitmap = bitmapset->bitmap[bt_type];
  ////envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  


  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  ////printf(" ***************************\n");
  for(r_ctr=0;r_ctr<nr;r_ctr++)
    {
      //// printf(" **** [%d] : %s \n",r_ctr,envPt_MM->robot[r_ctr]->name);
      if(robots_status_for_Mightability_Maps[r_ctr].has_moved==1)
	{
	  r = envPt_MM->robot[r_ctr];

	  cur_rob_pos=MY_ALLOC(double,r->nb_dof); 
	  p3d_get_robot_config_into(r,&cur_rob_pos);
  
	  ////////printf(" Updating obstacle cells for Object = %s \n",r->name);
	  if(strcasestr(r->name,"visball"))
	    {
	      robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
	      continue;
	    }
 /*  
	  NEED_HUMAN1_ALL_VISIBILITY_UPDATE=1;
#ifdef HUMAN2_EXISTS_FOR_MA
	  NEED_HUMAN2_ALL_VISIBILITY_UPDATE=1;
#endif

#ifdef JIDO_EXISTS_FOR_MA
	  NEED_JIDO_ALL_VISIBILITY_UPDATE=1;
#elif defined(HRI_HRP2)
	  NEED_HRP2_ALL_VISIBILITY_UPDATE=1;
#endif*/
  
	  ////////// printf(" inside update_3D_grid_for_Mightability_Maps_new(), NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);
    
          p3d_col_deactivate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
          p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], envPt_MM->robot[r_ctr]);//Only to test the collision between the visball and current object

	  ////**** First making previously occupied cells as free cells  
	    p3d_set_and_update_this_robot_conf(r, robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
	    ////////p3d_col_deactivate_rob_rob(bitmapset->visball,r);//To make a cell occupied only if there is collision of visball with other objects not with robot
	    int r_o_ctr=0; 
	    int cell_ctr_tmp=0;
	    for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	      {
		o = r->o[r_o_ctr];
		////printf(" o->name = %s \n",o->name);
		double BBx;
		for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		  {
		    visq[6] = BBx;
		    x=(BBx- bitmapset->realx)/bitmapset->pace;  
		    double BBy;
		    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		      {
			visq[7]  = BBy;
			y=(BBy- bitmapset->realy)/bitmapset->pace;  
			double BBz;
			for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			  {
			    visq[8]  = BBz;
			    z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			    if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			      {
				////bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
 
				/*if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
				  {
				  printf(" Cell belongs to bottle only\n");
				  }*/

				////////cell_ctr_tmp++;
				//////////printf(" cell_ctr=%d: %d, %d\n",cell_ctr_tmp,bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr],bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);
				//////////// if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==0)//NOT the cell just above the  horizontal surface of table
				//{
				  //////////printf(" Not horizontal surface\n");
				  ////printf(" cell_ctr=%d, %d, %d\n",cell_ctr_tmp,bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr],bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);

				  ////////////if((bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==0)||(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==0)||(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1))
                                   /*if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                      {
                                      printf("  cell (x, y, z) = (%d %d %d) ",x, y, z);
                                      
                                      printf("no_of_objects_belogs to this cell=%d\n", grid_around_HRP2.GRID_SET->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]);
                                      } */

				  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1)
				    {  
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
                                      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects--;
				      ////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
                                      ////printf(" Cell belongs to moved object and no of obj belongs to this cell =%d\n",bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);
				      if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==0)
                                      {
                                           bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
				      }
				    }//End if the cell belongs to this object
				 
				//}//End if NOT the cell just above the  horizontal surface of table
        
        
        
			
       
       
			      }
			  }
		      }
		  }
	      }
   
	  ////////p3d_col_activate_rob_rob(bitmapset->visball,r);
	  p3d_set_and_update_this_robot_conf(r, cur_rob_pos);
	  MY_FREE(cur_rob_pos, double,r->nb_dof); 
	  ////*****END making the previously occupied cells as free cells  

	    ////*****Now creating exact obstacle at new place of robot

	    ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	    r_o_ctr=0; 
	    for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	      {
		o = r->o[r_o_ctr];
		////printf(" o->name = %s \n",o->name);
		double BBx;
		for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		  {
		    visq[6] = BBx;
		    x=(BBx- bitmapset->realx)/bitmapset->pace;  
		    double BBy;
		    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		      {
			visq[7]  = BBy;
			y=(BBy- bitmapset->realy)/bitmapset->pace;  
			double BBz;
			for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			  {
			    visq[8]  = BBz;
			    z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			    if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			      {
				p3d_set_and_update_this_robot_conf(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq);
				int kcd_with_report=0;
				int res = p3d_col_test_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report);
				if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				  {
				    //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
				    bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
       
				    if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0)//the cell does not already belong to this robot index
				      { 
					bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;
					bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
				      }
				    /////////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;
				      //else
				      //{
				      //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				      // }
          /*
				      int i=0; 
				      for(i=x-expansion;i<=x+expansion;i++)
					{
					  //printf(" for i = %d\n",i);
					  if(i>=0&&i<bitmap->nx)
					    {
					      int j=0;
					      for(j=y-expansion;j<=y+expansion;j++)
						{
						  //printf(" for i, j = %d %d\n",i, j);
						  if(j>=0&&j<bitmap->ny)
						    {
						      int k=0;
						      for(k=z-expansion;k<=z+expansion;k++)
							{
							  //printf(" for i, j, k = %d %d %\n",i, j, k);
							  if(k>=0&&k<bitmap->nz)
							    {
                 
							      // printf(" Populating %d, %d, %d \n",i,j,k);
							      if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
								{
                   
								  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
								    { 
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								    }
								}
							      else
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
                
								  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
								    { 
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								    }
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
							    }
							} 
						    }
						}
					    } 
					} */
				  }
				////else
				////{
				////printf("Inside BB but no collison\n");
				////exit(0);
				////}
			      }
			  }   
		      }
		  }
	      }
	    ////printf(" Setting has moved = 0\n");
	    robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
	    ////p3d_set_and_update_this_robot_conf(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config,r->ROBOT_POS);
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]=r->ROBOT_POS[6];
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]=r->ROBOT_POS[7];
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]=r->ROBOT_POS[8];
	    ////////////////p3d_copy_config_into(r,r->ROBOT_POS, &(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config));
	    ////////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
             p3d_col_activate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
	    p3d_get_robot_config_into ( r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config ); 
	}
   
    }
  

  return 1;
}

int update_3D_grid_for_Mightability_Maps_new_28_02_2011(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  if(bitmapset==NULL)
  {
  
   printf(" AKP ERROR : bitmapset=NULL\n");
   return 0;
  }

  int x,y,z;
 hri_bitmap * bitmap;

//Tmp for test commet it

      bitmap = bitmapset->bitmap[HRP2_GIK_MANIP];
int object_index=get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE);
  for(x=0; x<bitmap->nx; x++)
	{
	  for(y=0; y<bitmap->ny; y++)
	    {
	      for(z=0; z<bitmap->nz; z++)
		{
                   if(bitmapset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]==1)
		  ////if(bitmapset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val < 0)  
		    {
                      
		     printf("  cell (x, y, z) = (%d %d %d) belongs to %s\n",x, y, z,envPt_MM->robot[object_index]->name);
                     printf("no_of_objects_belogs to this cell=%d\n", grid_around_HRP2.GRID_SET->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]);
                                      
                                     	      
		    }
		}
	    }
	}  

//End tmp for tets

  configPt visq;
  configPt cur_rob_pos;
 
  double increment=3.0/4.0*bitmapset->pace;
  visq = p3d_get_robot_config(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
  bitmap = bitmapset->bitmap[bt_type];
  ////envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  


  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  p3d_obj *o;
  p3d_rob *r;
  int r_ctr=0;
  ////printf(" ***************************\n");
  for(r_ctr=0;r_ctr<nr;r_ctr++)
    {
      //// printf(" **** [%d] : %s \n",r_ctr,envPt_MM->robot[r_ctr]->name);
      if(robots_status_for_Mightability_Maps[r_ctr].has_moved==1)
	{
	  r = envPt_MM->robot[r_ctr];

	  cur_rob_pos=MY_ALLOC(double,r->nb_dof); 
	  p3d_get_robot_config_into(r,&cur_rob_pos);
  
	  printf(" Updating obstacle cells for Object = %s \n",r->name);
	  if(strcasestr(r->name,"visball"))
	    {
	      robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
	      continue;
	    }
 /*  
	  NEED_HUMAN1_ALL_VISIBILITY_UPDATE=1;
#ifdef HUMAN2_EXISTS_FOR_MA
	  NEED_HUMAN2_ALL_VISIBILITY_UPDATE=1;
#endif

#ifdef JIDO_EXISTS_FOR_MA
	  NEED_JIDO_ALL_VISIBILITY_UPDATE=1;
#elif defined(HRI_HRP2)
	  NEED_HRP2_ALL_VISIBILITY_UPDATE=1;
#endif*/
  
	  ////////// printf(" inside update_3D_grid_for_Mightability_Maps_new(), NEED_HUMAN_VISIBILITY_UPDATE=%d\n",NEED_HUMAN_VISIBILITY_UPDATE);
    
          p3d_col_deactivate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
          p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], envPt_MM->robot[r_ctr]);//Only to test the collision between the visball and current object

	  ////**** First making previously occupied cells as free cells  
	    p3d_set_and_update_this_robot_conf(r, robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
	    ////////p3d_col_deactivate_rob_rob(bitmapset->visball,r);//To make a cell occupied only if there is collision of visball with other objects not with robot
	    int r_o_ctr=0; 
	    int cell_ctr_tmp=0;
	    for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	      {
		o = r->o[r_o_ctr];
		////printf(" o->name = %s \n",o->name);
		double BBx;
		for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		  {
		    visq[6] = BBx;
		    x=(BBx- bitmapset->realx)/bitmapset->pace;  
		    double BBy;
		    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		      {
			visq[7]  = BBy;
			y=(BBy- bitmapset->realy)/bitmapset->pace;  
			double BBz;
			for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			  {
			    visq[8]  = BBz;
			    z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			    if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			      {
				////bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
 
				/*if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
				  {
				  printf(" Cell belongs to bottle only\n");
				  }*/

				cell_ctr_tmp++;
				//////////printf(" cell_ctr=%d: %d, %d\n",cell_ctr_tmp,bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr],bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);
				//////////// if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==0)//NOT the cell just above the  horizontal surface of table
				//{
				  //////////printf(" Not horizontal surface\n");
				  ////printf(" cell_ctr=%d, %d, %d\n",cell_ctr_tmp,bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr],bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);

				  ////////////if((bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==0)||(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==0)||(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1))
                                   if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                      {
                                      printf("  cell (x, y, z) = (%d %d %d) ",x, y, z);
                                      
                                      printf("no_of_objects_belogs to this cell=%d\n", grid_around_HRP2.GRID_SET->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]);
                                      } 

				  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1)
				    {  
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
				      ////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
                                      printf(" Cell belongs to moved object and no of obj belongs to this cell =%d\n",bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);
				      if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
					{
                                           bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
					  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0; 

					  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==0)
					    {
					      //////////printf(" Cell belongs to the displaced object only and no. of other close object=0\n");
					      bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
            
					    }
					  else//No of close object is >=1
					    {
					      if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1)//No. of close obj =1 
						{
						  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The close object is this object itself, so remove the object from the close as well as from the belonging object list
						    { 
						      //////////printf(" Cell belongs to the displaced object only\n");
						      bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle

						      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
						    }
						  else//There is one close object but it is not this object, so just remove this object from the close object list
						    {
             
						      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
						    }
						}
					      else//No. of close obj >1, 
						{
						  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//One of the close object is this object itself, so remove the object from the close object list
						    { 
              
						      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
						    }
             
						} 
            
					    } 
					}
				      else
					{
                                          printf(" The cell belongs to some other obstacles also \n");
					  //bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
					  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects--;
					} 
				    }//End if the cell belongs to this object
				  else //The cell does not belong to this object
				    {
                                      if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                      {
                                      ////printf("  cell (x, y, z) = (%d %d %d) ",x, y, z);
                                      
                                      printf(" The cell does not belong to the object\n");
                                      } 
                                      
				      if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The cell close to this object, so remove from the list of close objects
					{
					  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   ////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
            
					  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1)//This object is the only object close to the cell, so make the cell free
					    {
					      bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  //no obstacle
					      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
					    }
					  else
					    {
					      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects--;
					    }
					}
            

				    }// End else of if the cell belongs to this object
				//}//End if NOT the cell just above the  horizontal surface of table
        
        
        
				int i=0; 
				for(i=x-expansion;i<=x+expansion;i++)
				  {
				    //printf(" for i = %d\n",i);
				    if(i>=0&&i<bitmap->nx)
				      {
					int j=0;
					for(j=y-expansion;j<=y+expansion;j++)
					  {
					    //printf(" for i, j = %d %d\n",i, j);
					    if(j>=0&&j<bitmap->ny)
					      {
						int k=0;
						for(k=z-expansion;k<=z+expansion;k++)
						  {
						    //printf(" for i, j, k = %d %d %\n",i, j, k);
						    if(k>=0&&k<bitmap->nz)
						      {
							// printf(" Populating %d, %d, %d \n",i,j,k);
							//if((bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)||(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1&&   bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1))
							////////////if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.is_horizontal_surface==0)//NOT the cell just above the horizontal surface of table
							{
							  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==1)
							    {  
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=0;  
							      ////bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belong_to_objects--;

							      if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belongs_to_objects=0; 

								  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects==0)
								    {
								      //////////printf(" Cell belongs to the displaced object only\n");
								      bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //no obstacle
            
								    }
								  else//No of close object is >=1
								    {
								      if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects==1)//No. of close obj =1 
									{
									  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The close object is this object itself, so remove the object from the close as well as from the belonging object list
									    { 
									      //////////printf(" Cell belongs to the displaced object only\n");
									      bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //no obstacle

									      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects=0;
									    }
									  else//There is one close object but it is not this object, so just remove this object from the close object list
									    {
             
									      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
									    }
									}
								      else//No. of close obj >1, 
									{
									  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//One of the close object is this object itself, so remove the object from the close object list
									    { 
              
									      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
									    }
                    
									} 
            
								    } 
								}
							      else
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_belongs_to_objects--;
								}
							    }//End if the cell belongs to this object
							  else //The cell does not belong to this object
							    {
							      if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==1)//The cell close to this object, so remove from the list of close objects
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=0;   ////bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
            
								  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects==1)//This object is the only object close to the cell, so make the cell free
								    {
								      bitmapset->bitmap[bt_type]->data[i][j][k].val = 0;  //no obstacle
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects=0;
								    }
								  else
								    {
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects--;
								    }
								}
							    }// End else of if the cell belongs to this object 
							}
						      }
						  } 
					      }
					  }
				      } 
				  } 
       
       
			      }
			  }
		      }
		  }
	      }
   
	  ////////p3d_col_activate_rob_rob(bitmapset->visball,r);
	  p3d_set_and_update_this_robot_conf(r, cur_rob_pos);
	  MY_FREE(cur_rob_pos, double,r->nb_dof); 
	  ////*****END making the previously occupied cells as free cells  

	    ////*****Now creating exact obstacle at new place of robot

	    ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	    r_o_ctr=0; 
	    for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	      {
		o = r->o[r_o_ctr];
		////printf(" o->name = %s \n",o->name);
		double BBx;
		for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		  {
		    visq[6] = BBx;
		    x=(BBx- bitmapset->realx)/bitmapset->pace;  
		    double BBy;
		    for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		      {
			visq[7]  = BBy;
			y=(BBy- bitmapset->realy)/bitmapset->pace;  
			double BBz;
			for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			  {
			    visq[8]  = BBz;
			    z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			    if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			      {
				p3d_set_and_update_this_robot_conf(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq);
				int kcd_with_report=0;
				int res = p3d_col_test_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report);
				if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				  {
				    //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
				    bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
       
				    if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0)//the cell does not already belong to this robot index
				      { 
					bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;
					bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
				      }
				    /////////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;
				      //else
				      //{
				      //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				      // }
          
				      int i=0; 
				      for(i=x-expansion;i<=x+expansion;i++)
					{
					  //printf(" for i = %d\n",i);
					  if(i>=0&&i<bitmap->nx)
					    {
					      int j=0;
					      for(j=y-expansion;j<=y+expansion;j++)
						{
						  //printf(" for i, j = %d %d\n",i, j);
						  if(j>=0&&j<bitmap->ny)
						    {
						      int k=0;
						      for(k=z-expansion;k<=z+expansion;k++)
							{
							  //printf(" for i, j, k = %d %d %\n",i, j, k);
							  if(k>=0&&k<bitmap->nz)
							    {
                 
							      // printf(" Populating %d, %d, %d \n",i,j,k);
							      if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
								{
                   
								  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
								    { 
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								    }
								}
							      else
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
                
								  if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
								    { 
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
								      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								    }
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;
							    }
							} 
						    }
						}
					    } 
					} 
				  }
				////else
				////{
				////printf("Inside BB but no collison\n");
				////exit(0);
				////}
			      }
			  }   
		      }
		  }
	      }
	    ////printf(" Setting has moved = 0\n");
	    robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
	    ////p3d_set_and_update_this_robot_conf(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config,r->ROBOT_POS);
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[6]=r->ROBOT_POS[6];
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[7]=r->ROBOT_POS[7];
	    ////robots_status_for_Mightability_Maps[r_ctr].rob_prev_config[8]=r->ROBOT_POS[8];
	    ////////////////p3d_copy_config_into(r,r->ROBOT_POS, &(robots_status_for_Mightability_Maps[r_ctr].rob_prev_config));
	    ////////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
             p3d_col_activate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
	    p3d_get_robot_config_into ( r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config ); 
	}
   
    }
  

  return 1;
}


int create_exact_obstacles_for_HRP2_GIK_manip_fast_new2 ( hri_bitmapset * bitmapset, int expansion, int bt_type )
{
  ////ChronoOff();
  ////ChronoOn();

  if ( bt_type==HRP2_GIK_MANIP )
    {
      //                  configPt visq = MY_ALLOC(double,ACBTSET->visball->nb_dof); /* Allocation of temporary robot configuration */
      //                  p3d_get_robot_config_into(ACBTSET->robot,&visq); 
      // 		
      // 		hri_bitmap * bitmap;
      // 		double increment=3.0/4.0*bitmapset->pace;
      configPt visq;
      hri_bitmap * bitmap;
      double increment=3.0/4.0*bitmapset->pace;
      visq = p3d_get_robot_config(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);

      ////visq = p3d_get_robot_config(bitmapset->visball);

      bitmap = bitmapset->bitmap[bt_type];
      envPt_MM= ( p3d_env * ) p3d_get_desc_curid ( P3D_ENV );
      int no = envPt_MM->no;
      int nr = envPt_MM->nr;

      int x,y,z;
      for ( x=0; x<bitmap->nx; x++ )
	{
	  for ( y=0; y<bitmap->ny; y++ )
	    {
	      for ( z=0; z<bitmap->nz; z++ )
		{
		  bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;
		  int nr_ctr=0;
		  for ( ;nr_ctr<nr;nr_ctr++ )
		    {
		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell

		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
		    }
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of=nr_ctr;//horizontal surface belongs to this robot index
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.near_horizontal_surface=0;


		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;
		}
	    }
	}


      p3d_obj *o;
      p3d_rob *r;
      int r_ctr=0;
      for ( r_ctr=0;r_ctr<nr;r_ctr++ )
	{
			
	  r = envPt_MM->robot[r_ctr];
	  if ( strcasestr ( r->name,"visball" ) )
	    {
	      continue;
	    }
	  robots_status_for_Mightability_Maps[r_ctr].rob_prev_config=MY_ALLOC ( double,r->nb_dof );
	  p3d_get_robot_config_into ( r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config );
	  ////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
	  robots_status_for_Mightability_Maps[r_ctr].has_moved=0;

          p3d_col_deactivate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
          p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], envPt_MM->robot[r_ctr]);//Only to test the collision between the visball and current object
	  //////////printf(" Robot name = %s \n",r->name);
				
	  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	  int r_o_ctr=0;
	  for ( r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++ )
	    {
	      o = r->o[r_o_ctr];
	      ////printf(" o->name = %s \n",o->name);
	      double BBx;
	      for ( BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment )
		{
		  visq[6]  = BBx;
		  x= ( BBx- bitmapset->realx ) /bitmapset->pace;
		  double BBy;
		  for ( BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment )
		    {
		      visq[7]  = BBy;
		      y= ( BBy- bitmapset->realy ) /bitmapset->pace;
		      double BBz;
		      for ( BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment )
			{
			  visq[8]  = BBz;
			  z= ( BBz - bitmapset->realz ) /bitmapset->pace;

			  if ( x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz )
			    {
			      p3d_set_and_update_this_robot_conf ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
			      int kcd_with_report=0;
			      int res = p3d_col_test_robot ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report );
			      if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				{
                                   if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                   {
                                      printf(" Obstacle cell (x, y, z) = (%d %d %d)\n",x, y, z);
                                     //// printf(" Obstacle cell found for object\n");
                                   }
				  //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);

				  bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle

				  if ( bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0 ) //the cell does not already belong to this robot index
				    {
                   
                                      if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                      {
                                     
                                      printf(" Assigned cell belongs to object %s \n",envPt_MM->robot[r_ctr]->name);
                                      } 

				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.objects_belonging_to[bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects]=r_ctr;

				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
				    }

				  ////////////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index


										
										
				  //else
				  //{
				  //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				  // }
                                 /*
				  int i=0;
				  for ( i=x-expansion;i<=x+expansion;i++ )
				    {
				      //printf(" for i = %d\n",i);
				      if ( i>=0&&i<bitmap->nx )
					{
					  int j=0;
					  for ( j=y-expansion;j<=y+expansion;j++ )
					    {
					      //printf(" for i, j = %d %d\n",i, j);
					      if ( j>=0&&j<bitmap->ny )
						{
						  int k=0;
						  for ( k=z-expansion;k<=z+expansion;k++ )
						    {
						      //printf(" for i, j, k = %d %d %\n",i, j, k);
						      if ( k>=0&&k<bitmap->nz )
							{
							  // printf(" Populating %d, %d, %d \n",i,j,k);
							  if ( bitmapset->bitmap[bt_type]->data[i][j][k].val == -1 ) //Already exact obstacle
							    {

							      if ( bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0 ) //the cell does not already close to this robot index
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
							    }
							  else
							    {
							      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle

							      if ( bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0 ) //the cell does not already close to this robot index
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
							    }
							}
						    }
						}
					    }
					}
				    }*/
				}
			      ////else
			      ////{
			      ////printf("Inside BB but no collison\n");
			      ////exit(0);
			      ////}
			    }
			}
		    }
		}
	    }
			
	}//End for ( r_ctr=0;r_ctr<nr;r_ctr++ )

p3d_col_activate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);

      ////Now same thing for objects (non robots)
      int o_ctr=0;
      for ( o_ctr=0;o_ctr<no;o_ctr++ )
	{

	  o = envPt_MM->o[r_ctr];
	  //// printf(" o->name = %s, \n",o->name);
	  double BBx;
	  for ( BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment )
	    {
	      visq[6]  = BBx;
	      x= ( BBx- bitmapset->realx ) /bitmapset->pace;
	      double BBy;
	      for ( BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment )
		{
		  visq[7]  = BBy;
		  y= ( BBy- bitmapset->realy ) /bitmapset->pace;
		  double BBz;
		  for ( BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment )
		    {
		      visq[8]  = BBz;
		      z= ( BBz- bitmapset->realz ) /bitmapset->pace;


		      if ( z>0&&z<bitmap->nz )
			{
			  p3d_set_and_update_this_robot_conf ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
			  int kcd_with_report=0;
			  int res = p3d_col_test_robot ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report );
			  if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
			    {
			      //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);

			      bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;

			      int i=0;
			      for ( i=x-expansion;i<=x+expansion;i++ )
				{
				  //printf(" for i = %d\n",i);
				  if ( i>=0&&i<bitmap->nx )
				    {
				      int j=0;
				      for ( j=y-expansion;j<=y+expansion;j++ )
					{
					  //printf(" for i, j = %d %d\n",i, j);
					  if ( j>=0&&j<bitmap->ny )
					    {
					      int k=0;
					      for ( k=z-expansion;k<=z+expansion;k++ )
						{
						  //printf(" for i, j, k = %d %d %\n",i, j, k);
						  if ( k>=0&&k<bitmap->nz )
						    {
						      // printf(" Populating %d, %d, %d \n",i,j,k);

						      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;
						    }
						}
					    }
					}
				    }
				}
			    }
			}
		    }
		}

	    }
	}
      p3d_destroy_config ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
      //         MY_FREE ( visq,double,bitmapset->visball->nb_dof );
    }

  ////ChronoPrint(" Time for create obstacle cell fast ");
  ////ChronoOff();
	
  return 1;
}

int create_exact_obstacles_for_HRP2_GIK_manip_fast_new2_28_02_2011 ( hri_bitmapset * bitmapset, int expansion, int bt_type )
{
  ////ChronoOff();
  ////ChronoOn();

  if ( bt_type==HRP2_GIK_MANIP )
    {
      //                  configPt visq = MY_ALLOC(double,ACBTSET->visball->nb_dof); /* Allocation of temporary robot configuration */
      //                  p3d_get_robot_config_into(ACBTSET->robot,&visq); 
      // 		
      // 		hri_bitmap * bitmap;
      // 		double increment=3.0/4.0*bitmapset->pace;
      configPt visq;
      hri_bitmap * bitmap;
      double increment=3.0/4.0*bitmapset->pace;
      visq = p3d_get_robot_config(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);

      ////visq = p3d_get_robot_config(bitmapset->visball);

      bitmap = bitmapset->bitmap[bt_type];
      envPt_MM= ( p3d_env * ) p3d_get_desc_curid ( P3D_ENV );
      int no = envPt_MM->no;
      int nr = envPt_MM->nr;

      int x,y,z;
      for ( x=0; x<bitmap->nx; x++ )
	{
	  for ( y=0; y<bitmap->ny; y++ )
	    {
	      for ( z=0; z<bitmap->nz; z++ )
		{
		  bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;
		  int nr_ctr=0;
		  for ( ;nr_ctr<nr;nr_ctr++ )
		    {
		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell

		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
		    }
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of=nr_ctr;//horizontal surface belongs to this robot index
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.near_horizontal_surface=0;


		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;
		}
	    }
	}


      p3d_obj *o;
      p3d_rob *r;
      int r_ctr=0;
      for ( r_ctr=0;r_ctr<nr;r_ctr++ )
	{
			
	  r = envPt_MM->robot[r_ctr];
	  if ( strcasestr ( r->name,"visball" ) )
	    {
	      continue;
	    }
	  robots_status_for_Mightability_Maps[r_ctr].rob_prev_config=MY_ALLOC ( double,r->nb_dof );
	  p3d_get_robot_config_into ( r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config );
	  ////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
	  robots_status_for_Mightability_Maps[r_ctr].has_moved=0;

          p3d_col_deactivate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
          p3d_col_activate_rob_rob(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], envPt_MM->robot[r_ctr]);//Only to test the collision between the visball and current object
	  //////////printf(" Robot name = %s \n",r->name);
				
	  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	  int r_o_ctr=0;
	  for ( r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++ )
	    {
	      o = r->o[r_o_ctr];
	      ////printf(" o->name = %s \n",o->name);
	      double BBx;
	      for ( BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment )
		{
		  visq[6]  = BBx;
		  x= ( BBx- bitmapset->realx ) /bitmapset->pace;
		  double BBy;
		  for ( BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment )
		    {
		      visq[7]  = BBy;
		      y= ( BBy- bitmapset->realy ) /bitmapset->pace;
		      double BBz;
		      for ( BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment )
			{
			  visq[8]  = BBz;
			  z= ( BBz - bitmapset->realz ) /bitmapset->pace;

			  if ( x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz )
			    {
			      p3d_set_and_update_this_robot_conf ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
			      int kcd_with_report=0;
			      int res = p3d_col_test_robot ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report );
			      if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				{
                                   if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                   {
                                      printf(" Obstacle cell (x, y, z) = (%d %d %d)\n",x, y, z);
                                     //// printf(" Obstacle cell found for object\n");
                                   }
				  //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);

				  bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle

				  if ( bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0 ) //the cell does not already belong to this robot index
				    {
                   
                                      if(r_ctr==get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE))
                                      {
                                     
                                      printf(" Assigned cell belongs to object %s \n",envPt_MM->robot[r_ctr]->name);
                                      } 

				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.objects_belonging_to[bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects]=r_ctr;

				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
				    }

				  ////////////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index


										
										
				  //else
				  //{
				  //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				  // }

				  int i=0;
				  for ( i=x-expansion;i<=x+expansion;i++ )
				    {
				      //printf(" for i = %d\n",i);
				      if ( i>=0&&i<bitmap->nx )
					{
					  int j=0;
					  for ( j=y-expansion;j<=y+expansion;j++ )
					    {
					      //printf(" for i, j = %d %d\n",i, j);
					      if ( j>=0&&j<bitmap->ny )
						{
						  int k=0;
						  for ( k=z-expansion;k<=z+expansion;k++ )
						    {
						      //printf(" for i, j, k = %d %d %\n",i, j, k);
						      if ( k>=0&&k<bitmap->nz )
							{
							  // printf(" Populating %d, %d, %d \n",i,j,k);
							  if ( bitmapset->bitmap[bt_type]->data[i][j][k].val == -1 ) //Already exact obstacle
							    {

							      if ( bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0 ) //the cell does not already close to this robot index
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
							    }
							  else
							    {
							      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle

							      if ( bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0 ) //the cell does not already close to this robot index
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
							    }
							}
						    }
						}
					    }
					}
				    }
				}
			      ////else
			      ////{
			      ////printf("Inside BB but no collison\n");
			      ////exit(0);
			      ////}
			    }
			}
		    }
		}
	    }
			
	}//End for ( r_ctr=0;r_ctr<nr;r_ctr++ )

p3d_col_activate_robot(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);
      ////Now same thing for objects (non robots)
      int o_ctr=0;
      for ( o_ctr=0;o_ctr<no;o_ctr++ )
	{

	  o = envPt_MM->o[r_ctr];
	  //// printf(" o->name = %s, \n",o->name);
	  double BBx;
	  for ( BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment )
	    {
	      visq[6]  = BBx;
	      x= ( BBx- bitmapset->realx ) /bitmapset->pace;
	      double BBy;
	      for ( BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment )
		{
		  visq[7]  = BBy;
		  y= ( BBy- bitmapset->realy ) /bitmapset->pace;
		  double BBz;
		  for ( BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment )
		    {
		      visq[8]  = BBz;
		      z= ( BBz- bitmapset->realz ) /bitmapset->pace;


		      if ( z>0&&z<bitmap->nz )
			{
			  p3d_set_and_update_this_robot_conf ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
			  int kcd_with_report=0;
			  int res = p3d_col_test_robot ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report );
			  if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
			    {
			      //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);

			      bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;

			      int i=0;
			      for ( i=x-expansion;i<=x+expansion;i++ )
				{
				  //printf(" for i = %d\n",i);
				  if ( i>=0&&i<bitmap->nx )
				    {
				      int j=0;
				      for ( j=y-expansion;j<=y+expansion;j++ )
					{
					  //printf(" for i, j = %d %d\n",i, j);
					  if ( j>=0&&j<bitmap->ny )
					    {
					      int k=0;
					      for ( k=z-expansion;k<=z+expansion;k++ )
						{
						  //printf(" for i, j, k = %d %d %\n",i, j, k);
						  if ( k>=0&&k<bitmap->nz )
						    {
						      // printf(" Populating %d, %d, %d \n",i,j,k);

						      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;
						    }
						}
					    }
					}
				    }
				}
			    }
			}
		    }
		}

	    }
	}
      p3d_destroy_config ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
      //         MY_FREE ( visq,double,bitmapset->visball->nb_dof );
    }

  ////ChronoPrint(" Time for create obstacle cell fast ");
  ////ChronoOff();
	
  return 1;
}


int create_exact_obstacles_for_HRP2_GIK_manip_fast_new ( hri_bitmapset * bitmapset, int expansion, int bt_type )
{
  ChronoOff();
  ChronoOn();

  if ( bt_type==HRP2_GIK_MANIP )
    {
      //                  configPt visq = MY_ALLOC(double,ACBTSET->visball->nb_dof); /* Allocation of temporary robot configuration */
      //                  p3d_get_robot_config_into(ACBTSET->robot,&visq); 
      // 		
      // 		hri_bitmap * bitmap;
      // 		double increment=3.0/4.0*bitmapset->pace;
      configPt visq;
      hri_bitmap * bitmap;
      double increment=3.0/4.0*bitmapset->pace;
      visq = p3d_get_robot_config(envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY]);

      ////visq = p3d_get_robot_config(bitmapset->visball);

      bitmap = bitmapset->bitmap[bt_type];
      envPt_MM= ( p3d_env * ) p3d_get_desc_curid ( P3D_ENV );
      int no = envPt_MM->no;
      int nr = envPt_MM->nr;

      int x,y,z;
      for ( x=0; x<bitmap->nx; x++ )
	{
	  for ( y=0; y<bitmap->ny; y++ )
	    {
	      for ( z=0; z<bitmap->nz; z++ )
		{
		  bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;
		  int nr_ctr=0;
		  for ( ;nr_ctr<nr;nr_ctr++ )
		    {
		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell

		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
		    }
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of=nr_ctr;//horizontal surface belongs to this robot index
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.near_horizontal_surface=0;


		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;
		}
	    }
	}


      p3d_obj *o;
      p3d_rob *r;
      int r_ctr=0;
      for ( r_ctr=0;r_ctr<nr;r_ctr++ )
	{
			
	  r = envPt_MM->robot[r_ctr];
	  if ( strcasestr ( r->name,"visball" ) )
	    {
	      continue;
	    }
	  robots_status_for_Mightability_Maps[r_ctr].rob_prev_config=MY_ALLOC ( double,r->nb_dof );
	  p3d_get_robot_config_into ( r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config );
	  ////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
	  robots_status_for_Mightability_Maps[r_ctr].has_moved=0;

	  //////////printf(" Robot name = %s \n",r->name);
	  int surf_z;
	  int is_tabel=0;
	  if ( strcasestr ( r->name,"table" ) )
	    {
	      //int surf_z=z;
	      //////////printf(" Cell belongs to table %s \n",r->name);

	      surf_z= ( ( r->BB.zmax - bitmapset->realz ) /bitmapset->pace ) +1;//One cell above the surface
	      is_tabel=1;
	      ////////// printf(" surf_z = %d \n",surf_z);
	    }
	  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	  int r_o_ctr=0;
	  for ( r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++ )
	    {
	      o = r->o[r_o_ctr];
	      ////printf(" o->name = %s \n",o->name);
	      double BBx;
	      for ( BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment )
		{
		  visq[6]  = BBx;
		  x= ( BBx- bitmapset->realx ) /bitmapset->pace;
		  double BBy;
		  for ( BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment )
		    {
		      visq[7]  = BBy;
		      y= ( BBy- bitmapset->realy ) /bitmapset->pace;
		      double BBz;
		      for ( BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment )
			{
			  visq[8]  = BBz;
			  z= ( BBz - bitmapset->realz ) /bitmapset->pace;

			  if ( x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz )
			    {
			      p3d_set_and_update_this_robot_conf ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
			      int kcd_with_report=0;
			      int res = p3d_col_test_robot ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report );
			      if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				{
				  //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);

				  bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle

				  if ( bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0 ) //the cell does not already belong to this robot index
				    {
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.objects_belonging_to[bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects]=r_ctr;

				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
				    }

				  ////////////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index


										
				  if ( is_tabel==1&& ( r->BB.zmax-BBz ) <=bitmapset->pace )
				    {
				      bitmapset->bitmap[bt_type]->data[x][y][surf_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;//Belongs to a horizontal surface of table
				      bitmapset->bitmap[bt_type]->data[x][y][surf_z].Mightability_map_cell_obj_info.horizontal_surface_of=r_ctr;//horizontal surface belongs to this robot index
				      bitmapset->bitmap[bt_type]->data[x][y][surf_z].val = -2;  //To avoid the plan path close to the  table surface
         										
				      /*
					if(surf_z+1<bitmap->nz)
					{
					bitmapset->bitmap[bt_type]->data[x][y][surf_z+1].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x][y][surf_z+1].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x-1>0&&x-1<bitmap->nx&&y+1>0&&y+1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x-1][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x-1][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(y+1>0&&y+1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x+1>0&&x+1<bitmap->nx&&y+1>0&&y+1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x+1][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x+1][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x-1>0&&x-1<bitmap->nx)
					{
					bitmapset->bitmap[bt_type]->data[x-1][y][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x-1][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x+1>0&&x+1<bitmap->nx)
					{
					bitmapset->bitmap[bt_type]->data[x+1][y][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x+1][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x-1>0&&x-1<bitmap->nx&&y-1>0&&y-1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x-1][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x-1][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(y-1>0&&y-1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x+1>0&&x+1<bitmap->nx&&y-1>0&&y-1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x+1][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x+1][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
				      */
				    }

				  //else
				  //{
				  //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				  // }

				  int i=0;
				  for ( i=x-expansion;i<=x+expansion;i++ )
				    {
				      //printf(" for i = %d\n",i);
				      if ( i>=0&&i<bitmap->nx )
					{
					  int j=0;
					  for ( j=y-expansion;j<=y+expansion;j++ )
					    {
					      //printf(" for i, j = %d %d\n",i, j);
					      if ( j>=0&&j<bitmap->ny )
						{
						  int k=0;
						  for ( k=z-expansion;k<=z+expansion;k++ )
						    {
						      //printf(" for i, j, k = %d %d %\n",i, j, k);
						      if ( k>=0&&k<bitmap->nz )
							{
							  // printf(" Populating %d, %d, %d \n",i,j,k);
							  if ( bitmapset->bitmap[bt_type]->data[i][j][k].val == -1 ) //Already exact obstacle
							    {

							      if ( bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0 ) //the cell does not already close to this robot index
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
							    }
							  else
							    {
							      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle

							      if ( bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0 ) //the cell does not already close to this robot index
								{
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index
							    }
							}
						    }
						}
					    }
					}
				    }
				}
			      ////else
			      ////{
			      ////printf("Inside BB but no collison\n");
			      ////exit(0);
			      ////}
			    }
			}
		    }
		}
	    }
			
	}//End for ( r_ctr=0;r_ctr<nr;r_ctr++ )


      ////Now same thing for onjects (non robots)
      int o_ctr=0;
      for ( o_ctr=0;o_ctr<no;o_ctr++ )
	{

	  o = envPt_MM->o[r_ctr];
	  //// printf(" o->name = %s, \n",o->name);
	  double BBx;
	  for ( BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment )
	    {
	      visq[6]  = BBx;
	      x= ( BBx- bitmapset->realx ) /bitmapset->pace;
	      double BBy;
	      for ( BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment )
		{
		  visq[7]  = BBy;
		  y= ( BBy- bitmapset->realy ) /bitmapset->pace;
		  double BBz;
		  for ( BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment )
		    {
		      visq[8]  = BBz;
		      z= ( BBz- bitmapset->realz ) /bitmapset->pace;


		      if ( z>0&&z<bitmap->nz )
			{
			  p3d_set_and_update_this_robot_conf ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
			  int kcd_with_report=0;
			  int res = p3d_col_test_robot ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY],kcd_with_report );
			  if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
			    {
			      //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);

			      bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;

			      int i=0;
			      for ( i=x-expansion;i<=x+expansion;i++ )
				{
				  //printf(" for i = %d\n",i);
				  if ( i>=0&&i<bitmap->nx )
				    {
				      int j=0;
				      for ( j=y-expansion;j<=y+expansion;j++ )
					{
					  //printf(" for i, j = %d %d\n",i, j);
					  if ( j>=0&&j<bitmap->ny )
					    {
					      int k=0;
					      for ( k=z-expansion;k<=z+expansion;k++ )
						{
						  //printf(" for i, j, k = %d %d %\n",i, j, k);
						  if ( k>=0&&k<bitmap->nz )
						    {
						      // printf(" Populating %d, %d, %d \n",i,j,k);

						      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;
						    }
						}
					    }
					}
				    }
				}
			    }
			}
		    }
		}

	    }
	}
      p3d_destroy_config ( envPt_MM->robot[rob_indx.VISBALL_MIGHTABILITY], visq );
      //         MY_FREE ( visq,double,bitmapset->visball->nb_dof );
    }

  ChronoPrint(" Time for create obstacle cell fast ");
  ChronoOff();
	
  return 1;
}



int create_exact_obstacles_for_HRP2_GIK_manip_fast(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  if(bt_type==HRP2_GIK_MANIP)
    {
      configPt visq;
      hri_bitmap * bitmap;
      double increment=3.0/4.0*bitmapset->pace;
      visq = p3d_get_robot_config(bitmapset->visball);

      bitmap = bitmapset->bitmap[bt_type];
      envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
      int no = envPt_MM->no;
      int nr = envPt_MM->nr;

      int x,y,z;
      for(x=0; x<bitmap->nx; x++)
	{
	  for(y=0; y<bitmap->ny; y++)
	    {
	      for(z=0; z<bitmap->nz; z++)
		{
		  bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  
		  int nr_ctr=0;
		  for(;nr_ctr<nr;nr_ctr++)
		    {
		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
     
		      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
		    }
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.near_horizontal_surface=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of=-1;//horizontal surface belongs to this robot index
     
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
		  bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;  
		}
	    }
	}   
  
  
      p3d_obj *o;
      p3d_rob *r;
      int r_ctr=0;
      for(r_ctr=0;r_ctr<nr;r_ctr++)
	{

	  r = envPt_MM->robot[r_ctr];

	  robots_status_for_Mightability_Maps[r_ctr].rob_prev_config=MY_ALLOC(double,r->nb_dof); 
	  p3d_get_robot_config_into(r,&robots_status_for_Mightability_Maps[r_ctr].rob_prev_config);
	  ////robots_status_for_Mightability_Maps[r_ctr].prev_BB=r->BB;
	  robots_status_for_Mightability_Maps[r_ctr].has_moved=0;
 
	  //////////printf(" Robot name = %s \n",r->name);
	  int surf_z;
	  int is_tabel=0;
	  if (strcasestr(r->name,"table"))
	    {
	      //int surf_z=z;
	      //////////printf(" Cell belongs to table %s \n",r->name);

	      surf_z=((r->BB.zmax - bitmapset->realz)/bitmapset->pace)+1;//One cell above the surface
	      is_tabel=1;
	      ////////// printf(" surf_z = %d \n",surf_z);
	    }
	  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
	  int r_o_ctr=0; 
	  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
	    {
	      o = r->o[r_o_ctr];
	      ////printf(" o->name = %s \n",o->name);
	      double BBx;
	      for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
		{
		  visq[6]  = BBx;
		  x=(BBx- bitmapset->realx)/bitmapset->pace;  
		  double BBy;
		  for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		    {
		      visq[7]  = BBy;
		      y=(BBy- bitmapset->realy)/bitmapset->pace;  
		      double BBz;
		      for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
			{
			  visq[8]  = BBz;
			  z=(BBz - bitmapset->realz)/bitmapset->pace;  
      
     

			  if(x>0&&x<bitmap->nx&&y>0&&y<bitmap->ny&&z>0&&z<bitmap->nz)
			    {
			      p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
			      int kcd_with_report=0;
			      int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
			      if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
				{
				  //////// printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
				  bitmapset->bitmap[bt_type]->data[x][y][z].val = -1;  //Exact obstacle
      
				  if(bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]==0)//the cell does not already belong to this robot index
				    { 
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index 
				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.objects_belonging_to[bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects]=r_ctr;

				      bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects++;
				    }

				  ////////////bitmapset->bitmap[bt_type]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[r_ctr]=1;//cell belong to this robot index 

   
      
				  if(is_tabel==1&&(r->BB.zmax-BBz)<=bitmapset->pace)
				    {
				      bitmapset->bitmap[bt_type]->data[x][y][surf_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;//Belongs to a horizontal surface of table
				      bitmapset->bitmap[bt_type]->data[x][y][surf_z].Mightability_map_cell_obj_info.horizontal_surface_of=r_ctr;//horizontal surface belongs to this robot index
				      bitmapset->bitmap[bt_type]->data[x][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
				      /*
					if(surf_z+1<bitmap->nz)
					{
					bitmapset->bitmap[bt_type]->data[x][y][surf_z+1].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x][y][surf_z+1].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x-1>0&&x-1<bitmap->nx&&y+1>0&&y+1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x-1][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x-1][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(y+1>0&&y+1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x+1>0&&x+1<bitmap->nx&&y+1>0&&y+1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x+1][y+1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x+1][y+1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x-1>0&&x-1<bitmap->nx)
					{
					bitmapset->bitmap[bt_type]->data[x-1][y][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x-1][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x+1>0&&x+1<bitmap->nx)
					{
					bitmapset->bitmap[bt_type]->data[x+1][y][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x+1][y][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x-1>0&&x-1<bitmap->nx&&y-1>0&&y-1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x-1][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x-1][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					} 
					if(y-1>0&&y-1<bitmap->ny) 
					{
					bitmapset->bitmap[bt_type]->data[x][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
					if(x+1>0&&x+1<bitmap->nx&&y-1>0&&y-1<bitmap->ny)
					{
					bitmapset->bitmap[bt_type]->data[x+1][y-1][surf_z].Mightability_map_cell_obj_info.near_horizontal_surface=1;//very close to a horizontal surface of table
					bitmapset->bitmap[bt_type]->data[x+1][y-1][surf_z].val = -2;  //To avoid the plan path close to the table surface
					}
				      */
				    }
        
				  //else
				  //{
				  //bitmapset->bitmap[bt_type]->data[x][y][surf_z].is_horizontal_surface=0;//Does not belong to a horizontal surface of table
				  // }
          
				  int i=0; 
				  for(i=x-expansion;i<=x+expansion;i++)
				    {
				      //printf(" for i = %d\n",i);
				      if(i>=0&&i<bitmap->nx)
					{
					  int j=0;
					  for(j=y-expansion;j<=y+expansion;j++)
					    {
					      //printf(" for i, j = %d %d\n",i, j);
					      if(j>=0&&j<bitmap->ny)
						{
						  int k=0;
						  for(k=z-expansion;k<=z+expansion;k++)
						    {
						      //printf(" for i, j, k = %d %d %\n",i, j, k);
						      if(k>=0&&k<bitmap->nz)
							{
							  // printf(" Populating %d, %d, %d \n",i,j,k);
							  if(bitmapset->bitmap[bt_type]->data[i][j][k].val == -1) //Already exact obstacle
							    {
                  
							      if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
								{ 
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index  
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index   
							    }
							  else
							    {
							      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  //Near to the obstacle
                 
							      if(bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]==0)//the cell does not already close to this robot index
								{ 
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index  
								  bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.no_close_to_objects++;
								}
							      bitmapset->bitmap[bt_type]->data[i][j][k].Mightability_map_cell_obj_info.close_to_objects_indx[r_ctr]=1;//cell close to this robot index   
							    }
							}
						    } 
						}
					    }
					} 
				    } 
				}
			      ////else
			      ////{
			      ////printf("Inside BB but no collison\n");
			      ////exit(0);
			      ////}
			    }
			}   
		    }
		}
	    }
	}
  

      int o_ctr=0;
      for(o_ctr=0;o_ctr<no;o_ctr++)
	{
  
	  o = envPt_MM->o[r_ctr];
	  //// printf(" o->name = %s, \n",o->name);
	  double BBx;
	  for(BBx=o->BB.xmin;BBx<o->BB.xmax;BBx+=increment)
	    {
	      visq[6]  = BBx;
	      x=(BBx- bitmapset->realx)/bitmapset->pace;  
	      double BBy;
	      for(BBy=o->BB.ymin;BBy<o->BB.ymax&&x>0&&x<bitmap->nx;BBy+=increment)
		{
		  visq[7]  = BBy;
		  y=(BBy- bitmapset->realy)/bitmapset->pace;  
		  double BBz;
		  for(BBz=o->BB.zmin;BBz<o->BB.zmax&&y>0&&y<bitmap->ny;BBz+=increment)
		    {
		      visq[8]  = BBz;
		      z=(BBz- bitmapset->realz)/bitmapset->pace;  
 
       
		      if(z>0&&z<bitmap->nz)
			{
			  p3d_set_and_update_this_robot_conf(bitmapset->visball, visq); 
			  int kcd_with_report=0;
			  int res = p3d_col_test_robot(bitmapset->visball,kcd_with_report);
			  if(res>0) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
			    {
			      //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
       
			      bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;  
          
			      int i=0; 
			      for(i=x-expansion;i<=x+expansion;i++)
				{
				  //printf(" for i = %d\n",i);
				  if(i>=0&&i<bitmap->nx)
				    {
				      int j=0;
				      for(j=y-expansion;j<=y+expansion;j++)
					{
					  //printf(" for i, j = %d %d\n",i, j);
					  if(j>=0&&j<bitmap->ny)
					    {
					      int k=0;
					      for(k=z-expansion;k<=z+expansion;k++)
						{
						  //printf(" for i, j, k = %d %d %\n",i, j, k);
						  if(k>=0&&k<bitmap->nz)
						    {
						      // printf(" Populating %d, %d, %d \n",i,j,k);
                 
						      bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  
						    }
						}  
					    }
					}
				    } 
				} 
			    }
			}
		    }   
		}
    
	    }
	}
      p3d_destroy_config(bitmapset->visball, visq);
    } 
  return 1;
}

int create_exact_obstacles_for_HRP2_GIK_manip(hri_bitmapset * bitmapset, int expansion, int bt_type)
{
  configPt visq;
  hri_bitmap * bitmap;
  int x,y,z;
  if(bt_type==-1) //Need to create for all bitmap types
    {
  

      visq = p3d_get_robot_config(bitmapset->visball);

      //p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->human[bitmapset->actual_human]->HumanPt);
  

      for(x=0; x<bitmap->nx; x++)
	{
	  for(y=0; y<bitmap->ny; y++)
	    {
	      for(z=0; z<bitmap->nz; z++)
		{
		  int btm_ctr=0;
      
		  for(btm_ctr=0;btm_ctr<bitmapset->n;btm_ctr++)
		    {
		      ////printf(" x,y,z = (%d,%d,%d), btm_ctr=%d\n",x,y,z,btm_ctr);
		      bitmap = bitmapset->bitmap[btm_ctr]; 
		      if(bitmap->type==HRP2_GIK_MANIP)
			p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->robot);
       
		      visq[6]  = x*bitmapset->pace+bitmapset->realx;
		      visq[7]  = y*bitmapset->pace+bitmapset->realy;
		      visq[8]  = z*bitmapset->pace+bitmapset->realz;
		      p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
		      //if(p3d_col_test_robot_statics(bitmapset->visball,FALSE))
		      if(p3d_col_test_robot(bitmapset->visball,FALSE))
			{
			  //printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
			  bitmapset->bitmap[btm_ctr]->data[x][y][z].val = -2;  
          
			  int i=0; 
			  for(i=x-expansion;i<=x+expansion;i++)
			    {
			      //printf(" for i = %d\n",i);
			      if(i>=0&&i<bitmap->nx)
				{
				  int j=0;
				  for(j=y-expansion;j<=y+expansion;j++)
				    {
				      //printf(" for i, j = %d %d\n",i, j);
				      if(j>=0&&j<bitmap->ny)
					{
					  int k=0;
					  for(k=z-expansion;k<=z+expansion;k++)
					    {
					      //printf(" for i, j, k = %d %d %\n",i, j, k);
					      if(k>=0&&k<bitmap->nz)
						{
						  // printf(" Populating %d, %d, %d \n",i,j,k);
						  bitmapset->bitmap[btm_ctr]->data[i][j][k].val = -2;  
						}
					    } 
					}
				    }
				} 
			    } 
			}
		      else
			{
			  bitmapset->bitmap[btm_ctr]->data[x][y][z].val = 0;
			}
		      if(bitmap->type==HRP2_GIK_MANIP)
			p3d_col_activate_rob_rob(bitmapset->visball,bitmapset->robot); 
		    }
		}
	    }
	}
      p3d_destroy_config(bitmapset->visball, visq);


      //p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->human[ACBTSET->actual_human]->HumanPt);

      return TRUE;
    
   
    }
  else //need to create for the specific bitmap type provided
    {
      bitmap = bitmapset->bitmap[bt_type];

      visq = p3d_get_robot_config(bitmapset->visball);

      //p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->human[bitmapset->actual_human]->HumanPt);
 
      ////AKP NOTE : Below is the old version in which the collision test has been disactivated with HRP2 for the purpose of letting the A* planner to find the collision free path from the hand to the goal. But now instead of this now using a function to make the cell values 0 around the hand to be used for manipulation
      ////if(bt_type==HRP2_GIK_MANIP)
      ////p3d_col_deactivate_rob_rob(bitmapset->visball,bitmapset->robot);

      for(x=0; x<bitmap->nx; x++){
	for(y=0; y<bitmap->ny; y++){
	  for(z=0; z<bitmap->nz; z++){
	    visq[6]  = x*bitmapset->pace+bitmapset->realx;
	    visq[7]  = y*bitmapset->pace+bitmapset->realy;
	    visq[8]  = z*bitmapset->pace+bitmapset->realz;
	    p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
	    //if(p3d_col_test_robot_statics(bitmapset->visball,FALSE))
	    if(p3d_col_test_robot(bitmapset->visball,FALSE))
	      {
		//printf(" Populating ACTUAL obstacle for x, y, z = %d %d %d\n",x, y, z);
		bitmapset->bitmap[bt_type]->data[x][y][z].val = -2;  
          
		int i=0; 
		for(i=x-expansion;i<=x+expansion;i++)
		  {
		    //printf(" for i = %d\n",i);
		    if(i>=0&&i<bitmap->nx)
		      {
			int j=0;
			for(j=y-expansion;j<=y+expansion;j++)
			  {
			    //printf(" for i, j = %d %d\n",i, j);
			    if(j>=0&&j<bitmap->ny)
			      {
				int k=0;
				for(k=z-expansion;k<=z+expansion;k++)
				  {
				    //printf(" for i, j, k = %d %d %\n",i, j, k);
				    if(k>=0&&k<bitmap->nz)
				      {
					// printf(" Populating %d, %d, %d \n",i,j,k);
					bitmapset->bitmap[bt_type]->data[i][j][k].val = -2;  
				      }
				  } 
			      }
			  }
		      } 
		  } 
	      }
	    else
	      {
		bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;
	      } 
	  }
	}
      }
      p3d_destroy_config(bitmapset->visball, visq);

      ////if(bt_type==HRP2_GIK_MANIP)
      //// p3d_col_activate_rob_rob(bitmapset->visball,bitmapset->robot);
      //p3d_col_activate_rob_rob(ACBTSET->visball,ACBTSET->human[ACBTSET->actual_human]->HumanPt);

      return TRUE;
    }
}

int create_3d_grid_for_HRP2_GIK(point_co_ordi grid_center)
{
 
  //////////configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
  //////////p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
  double BB_half_length=1.5; // along x axis
  double BB_half_width=1.5; // along y axis
  double BB_half_height=2.5; // along z axis
  /*
    double BB_half_length=0.5; // along x axis
    double BB_half_width=0.5; // along y axis
    double BB_half_height=2; // along z axis
  */
  //Bounding box coordinates are in global frame
  /*grid_around_HRP2.grid_BB.min_x=rob_cur_pos[6]-BB_half_length+0.5;
    grid_around_HRP2.grid_BB.max_x=rob_cur_pos[6]+BB_half_length+0.5;
    grid_around_HRP2.grid_BB.min_y=rob_cur_pos[7]-BB_half_width;
    grid_around_HRP2.grid_BB.max_y=rob_cur_pos[7]+BB_half_width;
    grid_around_HRP2.grid_BB.min_z=rob_cur_pos[8]-0.05;//-BB_length;//No need to go underground :)
    grid_around_HRP2.grid_BB.max_z=rob_cur_pos[8]+BB_half_height;
  */

  /*
    grid_around_HRP2.grid_BB.min_x=grid_center.x-BB_half_length+0.5;
    grid_around_HRP2.grid_BB.max_x=grid_center.x+BB_half_length+0.5;
    grid_around_HRP2.grid_BB.min_y=grid_center.y-BB_half_width;
    grid_around_HRP2.grid_BB.max_y=grid_center.y+BB_half_width;
    grid_around_HRP2.grid_BB.min_z=grid_center.z-0.05;//-BB_length;//No need to go underground :)
    grid_around_HRP2.grid_BB.max_z=grid_center.z+BB_half_height;
  */

  grid_around_HRP2.grid_BB.min_x=grid_center.x-BB_half_length+0.5;
  grid_around_HRP2.grid_BB.max_x=grid_center.x+BB_half_length+0.5;
  grid_around_HRP2.grid_BB.min_y=grid_center.y-BB_half_width;
  grid_around_HRP2.grid_BB.max_y=grid_center.y+BB_half_width;
  grid_around_HRP2.grid_BB.min_z=grid_center.z-0.05;//-BB_length;//No need to go underground :)
  grid_around_HRP2.grid_BB.max_z=grid_center.z+BB_half_height;

  double BB_length=grid_around_HRP2.grid_BB.max_x-grid_around_HRP2.grid_BB.min_x;
  double BB_width=grid_around_HRP2.grid_BB.max_y-grid_around_HRP2.grid_BB.min_y;
  double BB_height=grid_around_HRP2.grid_BB.max_z-grid_around_HRP2.grid_BB.min_z;

  if(grid_around_HRP2.GRID_SET != NULL)
    hri_bt_destroy_bitmapset(grid_around_HRP2.GRID_SET);

  double sampling_rate=0.05;

  grid_around_HRP2.GRID_SET=create_3D_grid(BB_length,BB_width,BB_height,sampling_rate);
 
  hri_bt_change_bitmap_position(grid_around_HRP2.GRID_SET,grid_around_HRP2.grid_BB.min_x,grid_around_HRP2.grid_BB.min_y,grid_around_HRP2.grid_BB.min_z);

  int expansion=1; //To populate the number of cells, around a cell having obstacle, with obstacles also

  //***** AKP : To populate bitmap based on the exact obstacles' position, not based on their bounding Boxes..
    /* 
       ChronoPrint("<<<<<<<<Calling create_exact_obstacles_for_HRP2_GIK_manip for HRP2_GIK_MANIP bitmap");
       create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); 
       ChronoPrint("<<<<<<<<Calling create_exact_obstacles_for_HRP2_GIK_manip for BT_AFFORDANCE_VISIBILITY bitmap");
       create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,BT_AFFORDANCE_VISIBILITY); 
       ChronoPrint("<<<<<<<<Returned from create_exact_obstacles_for_HRP2_GIK_manip ");
    */
    //***** AKP: Uncomment below to populate the bitmap based on the bounding box of the objects' and robots'
    ////create_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET); 

    ////////////ChronoPrint("<<<<<<<<Calling create_exact_obstacles_for_HRP2_GIK_manip");
    //// create_exact_obstacles_for_HRP2_GIK_manip(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); //-1 is for populating every bitmap type 
    ////create_exact_obstacles_for_HRP2_GIK_manip_fast(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); //-1 is for populating every bitmap type 
    create_exact_obstacles_for_HRP2_GIK_manip_fast_new2(grid_around_HRP2.GRID_SET,expansion,HRP2_GIK_MANIP); //-1 is for populating every bitmap type 
  ////////////ChronoPrint("<<<<<<<<Returned from create_exact_obstacles_for_HRP2_GIK_manip");

  //////////MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
  return 1;
}

int show_3D_workspace_Bounding_Box()
{
  //Line id A onwards in alphabetical order. See notebook for fig
  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z,
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z,
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z,
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z,
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.max_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.min_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  g3d_drawOneLine(
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.min_z,
		  grid_around_HRP2.grid_BB.min_x, grid_around_HRP2.grid_BB.max_y, grid_around_HRP2.grid_BB.max_z, Red, NULL);

  return 1;
}


#ifndef COMMENT_TMP
int show_weighted_candidate_points_for_putinto_obj(int show_weight)
{
  ////printf("Inside show_weighted_candidate_points_for_putinto_obj, current_candidate_points_to_putinto.no_points=%d\n",current_candidate_points_to_putinto.no_points);
  int i=0;
  for(i=0;i<current_candidate_points_to_putinto.no_points;i++)
    {
      ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
      g3d_drawDisc(current_candidate_points_to_putinto.point[i].x, current_candidate_points_to_putinto.point[i].y, current_candidate_points_to_putinto.point[i].z, .02, Green, NULL);
      ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
      if(show_weight==1)
	{
	  g3d_drawOneLine(current_candidate_points_to_putinto.point[i].x, current_candidate_points_to_putinto.point[i].y,current_candidate_points_to_putinto.point[i].z, current_candidate_points_to_putinto.point[i].x, current_candidate_points_to_putinto.point[i].y, current_candidate_points_to_putinto.point[i].z+current_candidate_points_to_putinto.weight[i]/2.0, Green, NULL);
	}
    }
  /*
    i=0;
    for(i=0;i<candidate_points_to_putinto_blue_trashbin_by_jido.no_points;i++)
    {
    ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
    g3d_drawDisc(candidate_points_to_putinto_blue_trashbin_by_jido.point[i].x, candidate_points_to_putinto_blue_trashbin_by_jido.point[i].y, candidate_points_to_putinto_blue_trashbin_by_jido.point[i].z, .02, Green, NULL);
    ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET
  
    }
 

    i=0;
    for(i=0;i<candidate_points_to_putinto_by_jido.no_points;i++)
    {
    ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
    g3d_drawDisc(candidate_points_to_putinto_by_jido.point[i].x, candidate_points_to_putinto_by_jido.point[i].y, candidate_points_to_putinto_by_jido.point[i].z, .02, Green, NULL);
    ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET
  
    }
  */
  return 1;
}



int show_weighted_candidate_points_to_put_obj(int show_weight)
{


  int i=0;
  
  //   gpConvexHull3D *chull= new gpConvexHull3D(ACBTSET->object->o[0]->pol[0]->poly->the_points, ACBTSET->object->o[0]->pol[0]->poly->nb_points);
  //   chull->compute(false, 0.0005, false);

  

  for(i=0;i<candidate_points_to_put.no_points;i++)
    {
      ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
      g3d_drawDisc(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, 4, NULL);
      ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);

      //    chull->draw();
  
      ////get_placements_at_position(ACBTSET->object, candidate_points_to_put.point[i], PLACEMENTS, 2, placementListOut);


      ////for(std::list<gpPlacement>::iterator iter=placementListOut.begin(); iter!=placementListOut.end(); ++iter)
      ////{ iter->draw(0.05); }


      if(show_weight==1)
	{
	  g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z, candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z+candidate_points_to_put.weight[i]/2.0, Green, NULL);
	}
    }

  ////set_object_at_placement(ACBTSET->object, placementListOut.front());

  return 1;
}
#endif
int show_candidate_points_for_current_task(int show_weight_by_color, int show_weight_by_length)
{
   
  double radius=grid_around_HRP2.GRID_SET->pace/3.0;
  int i=0;
  double min, max, w;
  double color[4];
  if(show_weight_by_color==1||show_weight_by_length==1)
    {
      for(i=0;i<CANDIDATE_POINTS_FOR_CURRENT_TASK->no_points;i++)
	{
	  if( i==0 || CANDIDATE_POINTS_FOR_CURRENT_TASK->weight[i]<min)
	    min= CANDIDATE_POINTS_FOR_CURRENT_TASK->weight[i];

	  if( i==0 || CANDIDATE_POINTS_FOR_CURRENT_TASK->weight[i]>max)
	    max= CANDIDATE_POINTS_FOR_CURRENT_TASK->weight[i];
	}
    }
  // printf("minmax %f %f\n",min,max);
  for(i=0;i<CANDIDATE_POINTS_FOR_CURRENT_TASK->no_points;i++)
    {
      if(show_weight_by_color==1||show_weight_by_length==1)
	{
	  w= (CANDIDATE_POINTS_FOR_CURRENT_TASK->weight[i] - min )/(max-min);
	  // printf("w0 %f w %f \n",candidate_points_to_give.weight[i], w);
	  ////////////g3d_rgb_from_hue2(w, color);
	  AKP_rgb_from_hue2(w, color);
	  ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
	}

      if(show_weight_by_color==1)
	{
	  g3d_drawDisc(CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].x, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].y, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].z, radius, Any, color);
	}
      else
	{
	  g3d_drawDisc(CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].x, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].y, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].z, radius, 4, NULL);
	}
    
      if(show_weight_by_length==1)
	{
	  g3d_drawOneLine(CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].x, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].y,CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].z, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].x, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].y, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].z+w/10.0, Any, color);
	  /////g3d_drawOneLine(CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].x, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].y,CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].z, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].x, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].y, CANDIDATE_POINTS_FOR_CURRENT_TASK->point[i].z+w/10.0, 3, NULL);
	    }
      ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
      //////////g3d_drawOneLine(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y,candidate_points_to_show.point[i].z, candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z+candidate_points_to_show.weight[i]/2.0, Green, NULL);
   
  
    }

  
  return 1;

  
}


#ifndef COMMENT_TMP
int show_weighted_candidate_points_to_show_obj(int show_weight, int show_length)
{
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_show;
  show_candidate_points_for_current_task(show_weight,show_length);
  /*
    int i=0;
    double min, max, w;
    double color[4];

    for(i=0;i<candidate_points_to_show.no_points;i++)
    {
    if( i==0 || candidate_points_to_show.weight[i]<min)
    min= candidate_points_to_show.weight[i];

    if( i==0 || candidate_points_to_show.weight[i]>max)
    max= candidate_points_to_show.weight[i];
    }

    // printf("minmax %f %f\n",min,max);
    for(i=0;i<candidate_points_to_show.no_points;i++)
    {
    w= (candidate_points_to_show.weight[i] - min )/(max-min);
    // printf("w0 %f w %f \n",candidate_points_to_give.weight[i], w);
    g3d_rgb_from_hue2(w, color);
    ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
    g3d_drawDisc(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z, .02, Any, color);
    ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
    //////////g3d_drawOneLine(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y,candidate_points_to_show.point[i].z, candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z+candidate_points_to_show.weight[i]/2.0, Green, NULL);
   
    if(show_length==1)
    {

    
   
    g3d_drawOneLine(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y,candidate_points_to_show.point[i].z, candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z+w/10.0, Any, color);
    }
    }
  */
  return 1;

  
}

int show_weighted_candidate_points_to_give_obj(int show_weight)
{
  int i=0;
  double min, max, w;
  double color[4];

  for(i=0;i<candidate_points_to_give.no_points;i++)
    {
      if( i==0 || candidate_points_to_give.weight[i]<min)
	min= candidate_points_to_give.weight[i];

      if( i==0 || candidate_points_to_give.weight[i]>max)
	max= candidate_points_to_give.weight[i];
    }

  // printf("minmax %f %f\n",min,max);
  for(i=0;i<candidate_points_to_give.no_points;i++)
    {
      w= (candidate_points_to_give.weight[i] - min )/(max-min);
      // printf("w0 %f w %f \n",candidate_points_to_give.weight[i], w);
      ////////////g3d_rgb_from_hue2(w, color);
      AKP_rgb_from_hue2(w, color);
      ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
      g3d_drawDisc(candidate_points_to_give.point[i].x, candidate_points_to_give.point[i].y, candidate_points_to_give.point[i].z, .02, Any, color);
      ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
      //////////g3d_drawOneLine(candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y,candidate_points_to_show.point[i].z, candidate_points_to_show.point[i].x, candidate_points_to_show.point[i].y, candidate_points_to_show.point[i].z+candidate_points_to_show.weight[i]/2.0, Green, NULL);
   
      if(show_weight==1)
	{

    
   
	  g3d_drawOneLine(candidate_points_to_give.point[i].x, candidate_points_to_give.point[i].y,candidate_points_to_give.point[i].z, candidate_points_to_give.point[i].x, candidate_points_to_give.point[i].y, candidate_points_to_give.point[i].z+w/10.0, Any, color);
	}
    }
  return 1;
}



int show_weighted_candidate_points_to_hide_obj()
{
   
  int i=0;
  ////////printf(" inside show, candidate_points_to_hide.no_points= %d\n",candidate_points_to_hide.no_points);
  for(i=0;i<candidate_points_to_hide.no_points;i++)
    {
      ////////g3d_draw_vertical_cylinder(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y, candidate_points_to_put.point[i].z, .02, candidate_points_to_put.weight[i]/2.0, 0.005, 4, NULL);
      g3d_drawDisc(candidate_points_to_hide.point[i].x, candidate_points_to_hide.point[i].y, candidate_points_to_hide.point[i].z, .02, 4, NULL);
      ////////g3d_drawOneLine(candidate_points_to_put.point[i].x, candidate_points_to_put.point[i].y,candidate_points_to_put.point[i].z,ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3], ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3], Red, NULL);
  
      g3d_drawOneLine(candidate_points_to_hide.point[i].x, candidate_points_to_hide.point[i].y,candidate_points_to_hide.point[i].z, candidate_points_to_hide.point[i].x, candidate_points_to_hide.point[i].y, candidate_points_to_hide.point[i].z+candidate_points_to_hide.weight[i]/2.0, Green, NULL);
    }
  return 1;
}


int reverse_sort_weighted_candidate_points_to_putinto_obj()
{
  ////point_co_ordi sorted_points[1000];
  int i=0;
  for(i=0;i<current_candidate_points_to_putinto.no_points;i++)  

    //////////printf("Inside sort_weighted_candidate_points_to_putinto_obj(), current_candidate_points_to_putinto.no_points=%d\n",current_candidate_points_to_putinto.no_points);

    //////////printf(" Before sorting \n");
    for(int i=0;i<current_candidate_points_to_putinto.no_points;i++)
      {
	//////////printf(" >>>>>>>>> current_candidate_points_to_putinto.weight[%d]=%lf\n",i, current_candidate_points_to_putinto.weight[i]); 
      }  

  for (int i = 0; i < current_candidate_points_to_putinto.no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < current_candidate_points_to_putinto.no_points; j++) 
	{
	  if(current_candidate_points_to_putinto.weight[j]>current_candidate_points_to_putinto.weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=current_candidate_points_to_putinto.weight[i];
	  double tmp_x=current_candidate_points_to_putinto.point[i].x;
	  double tmp_y=current_candidate_points_to_putinto.point[i].y;
	  double tmp_z=current_candidate_points_to_putinto.point[i].z;
	  current_candidate_points_to_putinto.weight[i]=current_candidate_points_to_putinto.weight[curmax];
	  current_candidate_points_to_putinto.point[i].x=current_candidate_points_to_putinto.point[curmax].x;
	  current_candidate_points_to_putinto.point[i].y=current_candidate_points_to_putinto.point[curmax].y;
	  current_candidate_points_to_putinto.point[i].z=current_candidate_points_to_putinto.point[curmax].z;
 
	  current_candidate_points_to_putinto.weight[curmax]=tmp_wt;
	  current_candidate_points_to_putinto.point[curmax].x=tmp_x;
	  current_candidate_points_to_putinto.point[curmax].y=tmp_y;
	  current_candidate_points_to_putinto.point[curmax].z=tmp_z;

	}
    }
   
  //////////printf(" After sorting \n");
  for(int i=0;i<current_candidate_points_to_putinto.no_points;i++)
    {
      //////////printf(" <<<<<<<< current_candidate_points_to_putinto.weight[%d]=%lf\n",i, current_candidate_points_to_putinto.weight[i]); 
    }  
  return 1;
}
#endif
int reverse_sort_HRI_task_weighted_candidate_points(candidate_poins_for_task *candidate_points)
{
  ////point_co_ordi sorted_points[1000];
  
  //////////printf("Inside sort_weighted_candidate_points_to_put_obj(), candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);

  //////////printf(" Before sorting \n");
  ////for(int i=0;i<candidate_points->no_points;i++)
    ////{
      //////////printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
    ////}  

  for (int i = 0; i < candidate_points->no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < candidate_points->no_points; j++) 
	{
	  if(candidate_points->weight[j]>candidate_points->weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=candidate_points->weight[i];
	  double tmp_x=candidate_points->point[i].x;
	  double tmp_y=candidate_points->point[i].y;
	  double tmp_z=candidate_points->point[i].z;
	  int hz_suf_obj_indx=candidate_points->horizontal_surface_of[i];
   
	  candidate_points->horizontal_surface_of[i]=candidate_points->horizontal_surface_of[curmax];
	  candidate_points->weight[i]=candidate_points->weight[curmax];
	  candidate_points->point[i].x=candidate_points->point[curmax].x;
	  candidate_points->point[i].y=candidate_points->point[curmax].y;
	  candidate_points->point[i].z=candidate_points->point[curmax].z;
 
	  candidate_points->horizontal_surface_of[curmax]=hz_suf_obj_indx;
	  candidate_points->weight[curmax]=tmp_wt;
	  candidate_points->point[curmax].x=tmp_x;
	  candidate_points->point[curmax].y=tmp_y;
	  candidate_points->point[curmax].z=tmp_z;

	}
    }
   
  //////////printf(" After sorting \n");
  ////for(int i=0;i<candidate_points_to_put.no_points;i++)
  ////{
  //////////printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
  ////}  
  return 1;
}
#ifndef COMMENT_TMP
int reverse_sort_weighted_candidate_points_to_put_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  //////////printf("Inside sort_weighted_candidate_points_to_put_obj(), candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);

  //////////printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_put.no_points;i++)
    {
      //////////printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
    }  

  for (int i = 0; i < candidate_points_to_put.no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < candidate_points_to_put.no_points; j++) 
	{
	  if(candidate_points_to_put.weight[j]>candidate_points_to_put.weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=candidate_points_to_put.weight[i];
	  double tmp_x=candidate_points_to_put.point[i].x;
	  double tmp_y=candidate_points_to_put.point[i].y;
	  double tmp_z=candidate_points_to_put.point[i].z;
	  int hz_suf_obj_indx=candidate_points_to_put.horizontal_surface_of[i];
   
	  candidate_points_to_put.horizontal_surface_of[i]=candidate_points_to_put.horizontal_surface_of[curmax];
	  candidate_points_to_put.weight[i]=candidate_points_to_put.weight[curmax];
	  candidate_points_to_put.point[i].x=candidate_points_to_put.point[curmax].x;
	  candidate_points_to_put.point[i].y=candidate_points_to_put.point[curmax].y;
	  candidate_points_to_put.point[i].z=candidate_points_to_put.point[curmax].z;
 
	  candidate_points_to_put.horizontal_surface_of[curmax]=hz_suf_obj_indx;
	  candidate_points_to_put.weight[curmax]=tmp_wt;
	  candidate_points_to_put.point[curmax].x=tmp_x;
	  candidate_points_to_put.point[curmax].y=tmp_y;
	  candidate_points_to_put.point[curmax].z=tmp_z;

	}
    }
   
  //////////printf(" After sorting \n");
  ////for(int i=0;i<candidate_points_to_put.no_points;i++)
  ////{
  //////////printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
  ////}  
  return 1;
}

int reverse_sort_weighted_candidate_points_to_show_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  //////////printf("Inside reverse_sort_weighted_candidate_points_to_show_obj(), candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);

  //////////printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_show.no_points;i++)
    {
      //////////printf(" candidate_points_to_show.weight[%d]=%lf\n",i, candidate_points_to_show.weight[i]); 
    }  

  for (int i = 0; i < candidate_points_to_show.no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < candidate_points_to_show.no_points; j++) 
	{
	  if(candidate_points_to_show.weight[j]>candidate_points_to_show.weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=candidate_points_to_show.weight[i];
	  double tmp_x=candidate_points_to_show.point[i].x;
	  double tmp_y=candidate_points_to_show.point[i].y;
	  double tmp_z=candidate_points_to_show.point[i].z;
	  candidate_points_to_show.weight[i]=candidate_points_to_show.weight[curmax];
	  candidate_points_to_show.point[i].x=candidate_points_to_show.point[curmax].x;
	  candidate_points_to_show.point[i].y=candidate_points_to_show.point[curmax].y;
	  candidate_points_to_show.point[i].z=candidate_points_to_show.point[curmax].z;
 
	  candidate_points_to_show.weight[curmax]=tmp_wt;
	  candidate_points_to_show.point[curmax].x=tmp_x;
	  candidate_points_to_show.point[curmax].y=tmp_y;
	  candidate_points_to_show.point[curmax].z=tmp_z;

	}
    }
   
  //////////printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_show.no_points;i++)
    {
      //////////printf(" candidate_points_to_show.weight[%d]=%lf\n",i, candidate_points_to_show.weight[i]); 
    }  
  return 1;
}

int reverse_sort_weighted_candidate_points_to_displace_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  /////////printf("Inside sort_weighted_candidate_points_to_displace_obj(), candidate_points_to_displace_obj.no_points=%d\n",candidate_points_to_displace_obj.no_points);

  /////////printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_displace_obj.no_points;i++)
    {
      //////////printf(" candidate_points_to_put.weight[%d]=%lf\n",i, candidate_points_to_put.weight[i]); 
    }  

  for (int i = 0; i < candidate_points_to_displace_obj.no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < candidate_points_to_displace_obj.no_points; j++) 
	{
	  if(candidate_points_to_displace_obj.weight[j]>candidate_points_to_displace_obj.weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=candidate_points_to_displace_obj.weight[i];
	  double tmp_x=candidate_points_to_displace_obj.point[i].x;
	  double tmp_y=candidate_points_to_displace_obj.point[i].y;
	  double tmp_z=candidate_points_to_displace_obj.point[i].z;
	  candidate_points_to_displace_obj.weight[i]=candidate_points_to_displace_obj.weight[curmax];
	  candidate_points_to_displace_obj.point[i].x=candidate_points_to_displace_obj.point[curmax].x;
	  candidate_points_to_displace_obj.point[i].y=candidate_points_to_displace_obj.point[curmax].y;
	  candidate_points_to_displace_obj.point[i].z=candidate_points_to_displace_obj.point[curmax].z;
 
	  candidate_points_to_displace_obj.weight[curmax]=tmp_wt;
	  candidate_points_to_displace_obj.point[curmax].x=tmp_x;
	  candidate_points_to_displace_obj.point[curmax].y=tmp_y;
	  candidate_points_to_displace_obj.point[curmax].z=tmp_z;

	}
    }
   
  /////////printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_displace_obj.no_points;i++)
    {
      /////////printf(" candidate_points_to_displace_obj.weight[%d]=%lf\n",i, candidate_points_to_displace_obj.weight[i]); 
      candidate_points_to_displace_obj.status[i]=0;//Initially all the candidate points are untested
    }  
 
}

int reverse_sort_weighted_candidate_points_to_give_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  //////////printf("Inside reverse_sort_weighted_candidate_points_to_show_obj(), candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);

  //////////printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_give.no_points;i++)
    {
      //////////printf(" candidate_points_to_show.weight[%d]=%lf\n",i, candidate_points_to_show.weight[i]); 
    }  

  for (int i = 0; i < candidate_points_to_give.no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < candidate_points_to_give.no_points; j++) 
	{
	  if(candidate_points_to_give.weight[j]>candidate_points_to_give.weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=candidate_points_to_give.weight[i];
	  double tmp_x=candidate_points_to_give.point[i].x;
	  double tmp_y=candidate_points_to_give.point[i].y;
	  double tmp_z=candidate_points_to_give.point[i].z;
	  candidate_points_to_give.weight[i]=candidate_points_to_give.weight[curmax];
	  candidate_points_to_give.point[i].x=candidate_points_to_give.point[curmax].x;
	  candidate_points_to_give.point[i].y=candidate_points_to_give.point[curmax].y;
	  candidate_points_to_give.point[i].z=candidate_points_to_give.point[curmax].z;
 
	  candidate_points_to_give.weight[curmax]=tmp_wt;
	  candidate_points_to_give.point[curmax].x=tmp_x;
	  candidate_points_to_give.point[curmax].y=tmp_y;
	  candidate_points_to_give.point[curmax].z=tmp_z;

	}
    }
   
  //////////printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_give.no_points;i++)
    {
      //////////printf(" candidate_points_to_show.weight[%d]=%lf\n",i, candidate_points_to_show.weight[i]); 
    }  
  return 1;
}

int reverse_sort_weighted_candidate_points_to_hide_obj()
{
  ////point_co_ordi sorted_points[1000];
  
  //////////printf("Inside reverse_sort_weighted_candidate_points_to_hide_obj(), candidate_points_to_hide.no_points=%d\n",candidate_points_to_hide.no_points);

  ////////// printf(" Before sorting \n");
  for(int i=0;i<candidate_points_to_hide.no_points;i++)
    {
      //////////printf(" candidate_points_to_hide.weight[%d]=%lf\n",i, candidate_points_to_hide.weight[i]); 
    }  

  for (int i = 0; i < candidate_points_to_hide.no_points; i++) 
    {
      int curmax = i;
      for (int j = i+1; j < candidate_points_to_hide.no_points; j++) 
	{
	  if(candidate_points_to_hide.weight[j]>candidate_points_to_hide.weight[curmax])
	    {
	      curmax=j;
	    } 
	}
      if (curmax != i)
	{
	  double tmp_wt=candidate_points_to_hide.weight[i];
	  double tmp_x=candidate_points_to_hide.point[i].x;
	  double tmp_y=candidate_points_to_hide.point[i].y;
	  double tmp_z=candidate_points_to_hide.point[i].z;
	  candidate_points_to_hide.weight[i]=candidate_points_to_hide.weight[curmax];
	  candidate_points_to_hide.point[i].x=candidate_points_to_hide.point[curmax].x;
	  candidate_points_to_hide.point[i].y=candidate_points_to_hide.point[curmax].y;
	  candidate_points_to_hide.point[i].z=candidate_points_to_hide.point[curmax].z;
 
	  candidate_points_to_hide.weight[curmax]=tmp_wt;
	  candidate_points_to_hide.point[curmax].x=tmp_x;
	  candidate_points_to_hide.point[curmax].y=tmp_y;
	  candidate_points_to_hide.point[curmax].z=tmp_z;

	}
    }
   
  //////////printf(" After sorting \n");
  for(int i=0;i<candidate_points_to_hide.no_points;i++)
    {
      //////////printf(" candidate_points_to_hide.weight[%d]=%lf\n",i, candidate_points_to_hide.weight[i]); 
    }  
  return 1;
}
#endif

void Cartesian_to_Spherical(double x, double y, double z,
			    double originx, double originy, double originz,
			    double *phi, double *theta)
{
  double distance = DISTANCE3D(x,y,z,originx,originy,originz);

  *phi = atan2( (y-originy),(x-originx) );
  *theta = acos( (z-originz)/distance );

}



int assign_weights_on_candidte_points_to_put_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent)
{
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  ////p3d_matInvertXform(ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos, hum_pos_inverse);
  p3d_matInvertXform(envPt_MM->robot[indx_for_agent]->joints[1]->abs_pos, hum_pos_inverse);
   
  int i=0;
  double Amplitude=1;
  double yaw_mean=0;
  double pitch_mean=0;
  double sig_yaw=M_PI;
  int obj_index=get_index_of_robot_by_name ( object_name );

  double human_pos[3];
      double sig_hum_dist=0.7;
      double mean_dist=0.3;
      human_pos[0]=envPt_MM->robot[indx_for_agent]->joints[1]->abs_pos[0][3];
      human_pos[1]=envPt_MM->robot[indx_for_agent]->joints[1]->abs_pos[1][3];
      human_pos[2]=envPt_MM->robot[indx_for_agent]->joints[1]->abs_pos[2][3];
      
      double bottle_pos[3];
      double sig_dist=2.0;
      bottle_pos[0]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[0][3];
      bottle_pos[1]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[1][3];
      bottle_pos[2]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[2][3];
      
  for(i=0;i<candidate_points->no_points;i++)
    {
      point_in_global_frame[0] = candidate_points->point[i].x;
      point_in_global_frame[1] = candidate_points->point[i].y;
      point_in_global_frame[2] = candidate_points->point[i].z;
      point_in_global_frame[3] = 1;
  
  
      ////Assigning weight wrt human axis
      p3d_matvec4Mult(hum_pos_inverse, point_in_global_frame, point_in_human_frame);
  
      //////////p3d_psp_cartesian2spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
      Cartesian_to_Spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);

  
      ////////candidate_points_to_put.weight[i]+=(1.0-(fabs(relative_yaw))/M_PI);

      ////////candidate_points_to_put.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_x*sig_x)+((pitch_mean-relative_yaw)*(pitch_mean-relative_yaw)/2.0*sig_y*sig_y))));

      candidate_points->weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)));

      //////////printf(" relative_yaw=%lf, weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_put.weight[i]);

  
      ////Assigning weight based on the closeness to the point which is at dist 0.3 m from the human 
      
      ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
      double point_to_human_dist=sqrt((human_pos[0]-point_in_global_frame[0])*(human_pos[0]-point_in_global_frame[0])+(human_pos[1]-point_in_global_frame[1])*(human_pos[1]-point_in_global_frame[1])+(human_pos[2]-point_in_global_frame[2])*(human_pos[2]-point_in_global_frame[2]));
      //////////printf(" point to human dist = %lf \n", point_to_human_dist);
      candidate_points->weight[i]+=Amplitude/2.0*exp(-(((mean_dist-point_to_human_dist)*(mean_dist-point_to_human_dist)/2.0*sig_hum_dist*sig_hum_dist)));
  

      ////Assigning weight based on the closeness to the bottle current position
  
     
      ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
      double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
      candidate_points->weight[i]+=Amplitude/1.5*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));

      /*if(fabs(point_to_bottle_dist)<0.00001)
	candidate_points_to_put.weight[i]+=1;
	else
	candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
      */
    } 
  
 
  
  return 1;
  
}

int assign_weights_on_candidte_points_to_show_obj_old(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent)
{
  
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_matInvertXform(envPt_MM->robot[indx_for_agent]->joints[HUMANj_NECK_PAN]->abs_pos, hum_pos_inverse);

  configPt hum_cur_pos = MY_ALLOC(double,envPt_MM->robot[indx_for_agent]->nb_dof); /* Allocation of temporary robot configuration */

  p3d_get_robot_config_into(envPt_MM->robot[indx_for_agent],&hum_cur_pos);


  ////double yaw=M_PI/3.0;

  ////double orig_pan=hum_cur_pos[HUMANq_PAN]; 
  int i=0;
  double Amplitude=1.0;
  double yaw_mean=hum_cur_pos[HUMANq_PAN];
  double pitch_mean=hum_cur_pos[HUMANq_TILT];
  double sig_yaw=M_PI/2.0;
  double sig_pitch=M_PI/2.0;
  double max_weight=0;

  //////////printf("candidate_points_to_show.no_points=%d\n",candidate_points->no_points);

  for(i=0;i<candidate_points->no_points;i++)
    {
      point_in_global_frame[0] = candidate_points->point[i].x;
      point_in_global_frame[1] = candidate_points->point[i].y;
      point_in_global_frame[2] = candidate_points->point[i].z;
      point_in_global_frame[3] = 1;
  
  
      ////Assigning weight wrt human axis
      p3d_matvec4Mult(hum_pos_inverse, point_in_global_frame, point_in_human_frame);
  
      ////p3d_psp_cartesian2spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
      (point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
  
      ////////candidate_points_to_put.weight[i]+=(1.0-(fabs(relative_yaw))/M_PI);

      ////printf(" For point %d \n",i);
      candidate_points->weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)+((pitch_mean-relative_pitch)*(pitch_mean-relative_pitch)/2.0*sig_pitch*sig_pitch)));

      if(max_weight<candidate_points->weight[i])
	max_weight=candidate_points->weight[i];
      ////////candidate_points_to_show.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)));
  
      //////////printf(" relative_yaw=%lf, relative_pitch=%lf, weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,relative_pitch, i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);

      /*
    ////Assigning weight based on the closeness to the point which is at dist 0.3 m from the human 
    double human_pos[3];
    double sig_hum_dist=0.7;
    double mean_dist=0.3;
    human_pos[0]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3];
    human_pos[1]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3];
    human_pos[2]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3];
    ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
    double point_to_human_dist=sqrt((human_pos[0]-point_in_global_frame[0])*(human_pos[0]-point_in_global_frame[0])+(human_pos[1]-point_in_global_frame[1])*(human_pos[1]-point_in_global_frame[1])+(human_pos[2]-point_in_global_frame[2])*(human_pos[2]-point_in_global_frame[2]));
    printf(" point to human dist = %lf \n", point_to_human_dist);
    candidate_points_to_put.weight[i]+=Amplitude/2.0*exp(-(((mean_dist-point_to_human_dist)*(mean_dist-point_to_human_dist)/2.0*sig_hum_dist*sig_hum_dist)));
      */

      /*
    ////Assigning weight based on the closeness to the bottle current position
    double bottle_pos[3];
    double sig_dist=2.0;
    bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
    bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
    bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
    ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
    double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
    candidate_points_to_show.weight[i]+=max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
      */
      /*if(fabs(point_to_bottle_dist)<0.00001)
	candidate_points_to_put.weight[i]+=1;
	else
	candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
      */
    } 
  
  //////////printf(" max_weight=%lf\n",max_weight);
  int obj_index=get_index_of_robot_by_name ( object_name );

  double bottle_pos[3];
      double sig_dist=2.0;
    bottle_pos[0]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[0][3];
      bottle_pos[1]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[1][3];
      bottle_pos[2]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[2][3];
      
  for(i=0;i<candidate_points->no_points;i++)
    {
      point_in_global_frame[0] = candidate_points->point[i].x;
      point_in_global_frame[1] = candidate_points->point[i].y;
      point_in_global_frame[2] = candidate_points->point[i].z;
      point_in_global_frame[3] = 1;
      ////Assigning weight based on the closeness to the bottle current position
      
  
    
      ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
      double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
      //////////printf(" Old weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
      candidate_points->weight[i]+=2.0*max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  
      //////////printf(" New weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
    }
  
  MY_FREE(hum_cur_pos,double,envPt_MM->robot[indx_for_agent]->nb_dof);

  return 1;
}

int assign_weights_on_candidate_points_to_displace_obj(char *object_name)
{
#ifndef COMMENT_TMP
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_matInvertXform(ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos, hum_pos_inverse);

   
  int i=0;
  double Amplitude=1;
  double yaw_mean=0;
  double pitch_mean=0;
  double sig_yaw=M_PI;
  int obj_index=get_index_of_robot_by_name ( object_name );

  for(i=0;i<candidate_points_to_displace_obj.no_points;i++)
    {
      point_in_global_frame[0] = candidate_points_to_displace_obj.point[i].x;
      point_in_global_frame[1] = candidate_points_to_displace_obj.point[i].y;
      point_in_global_frame[2] = candidate_points_to_displace_obj.point[i].z;
      point_in_global_frame[3] = 1;
  
  
 

      ////Assigning weight based on the closeness to the bottle current position
      double bottle_pos[3];
      double sig_dist=2.0;
      bottle_pos[0]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[0][3];
      bottle_pos[1]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[1][3];
      bottle_pos[2]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[2][3];
      ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
      double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
      if(point_to_bottle_dist<=0.25)//Give lower wt to avoid putting very close to the current position
	{
	  candidate_points_to_displace_obj.weight[i]=Amplitude/2.5*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
	}
      else//Assign higher weight to avoid going very far from the current position
	{
	  candidate_points_to_displace_obj.weight[i]=Amplitude/1.0*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
	}

      ////Assign higher weights to the farthest reachable point by robot
      point_co_ordi shoulder_pos;
      ////point_co_ordi sphere_pt;
      
      // Reachability Test by left hand
      shoulder_pos.x = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
      shoulder_pos.y = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
      shoulder_pos.z = ACBTSET->robot->joints[ROBOTj_LSHOULDER]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
      ////g3d_drawDisc(shoulder_pos.x, shoulder_pos.y, shoulder_pos.z, .1, Green, NULL);
      double point_to_shoulder_dist=sqrt((shoulder_pos.x-point_in_global_frame[0])*(shoulder_pos.x-point_in_global_frame[0])+(shoulder_pos.y-point_in_global_frame[1])*(shoulder_pos.y-point_in_global_frame[1])+(shoulder_pos.z-point_in_global_frame[2])*(shoulder_pos.z-point_in_global_frame[2]));
      candidate_points_to_displace_obj.weight[i]+=Amplitude/1.5*exp(-(((point_to_shoulder_dist)*(point_to_shoulder_dist)/2.0*sig_dist*sig_dist)));


      /*if(fabs(point_to_bottle_dist)<0.00001)
	candidate_points_to_put.weight[i]+=1;
	else
	candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
      */
    } 
  
 
  
#endif
  
}

int assign_weights_on_candidte_points_to_show_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent)
{
  int i;
  p3d_vector4 meanPoint;
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos, hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_mat4Copy(envPt_MM->robot[indx_for_agent]->joints[HUMAN_J_NECK_PAN]->abs_pos, hum_pos);
  p3d_matInvertXform(hum_pos, hum_pos_inverse);
  configPt hum_cur_pos = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); /* Allocation of temporary robot configuration */

  for(i=0;i<3;++i)
    meanPoint[i]= hum_pos[i][3] + 0.5*hum_pos[i][0];
  //meanPoint[3]= 1.0;
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&hum_cur_pos);


  ////double yaw=M_PI/3.0;

  ////double orig_pan=hum_cur_pos[HUMANq_PAN]; 
  i=0;
  double Amplitude=1.0;
  ////double yaw_mean=hum_cur_pos[HUMANq_PAN];
  ////double pitch_mean=hum_cur_pos[HUMANq_TILT];
  double yaw_mean=0;
  double pitch_mean=0;
  double sig_yaw=M_PI/2.0;
  double sig_pitch=M_PI/2.0;
  double max_weight=0;

  ///////////printf("candidate_points_to_show_no_points=%d\n",candidate_points->no_points);

  for(i=0;i<candidate_points->no_points;i++)
    {
      point_in_global_frame[0] = candidate_points->point[i].x;
      point_in_global_frame[1] = candidate_points->point[i].y;
      point_in_global_frame[2] = candidate_points->point[i].z;
      point_in_global_frame[3] = 1;
  
  
      ////Assigning weight wrt human axis
      p3d_matvec4Mult(hum_pos_inverse, point_in_global_frame, point_in_human_frame);
     //// printf("%f %f %f\n", point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2]);
      ////  p3d_psp_cartesian2spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
      Cartesian_to_Spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
     //// printf("%f %f\n", relative_yaw,relative_pitch);
  
      ////////candidate_points_to_put.weight[i]+=(1.0-(fabs(relative_yaw))/M_PI);

      ////printf(" For point %d \n",i);
      //     candidate_points_to_give.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)+((pitch_mean-relative_pitch)*(pitch_mean-relative_pitch)/2.0*sig_pitch*sig_pitch)));

      candidate_points->weight[i]= 1.0/( 2.0 + sqrt( pow(point_in_global_frame[0]-meanPoint[0], 2) + pow(point_in_global_frame[1]-meanPoint[1], 2) + pow(point_in_global_frame[2]-meanPoint[2], 2) ) );

      if(max_weight<candidate_points->weight[i])
	max_weight=candidate_points->weight[i];
      ////////candidate_points_to_show.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)));
  
     //// printf(" max_weight=%lf\n",max_weight);
      if(max_weight>100)
	{
	////  printf(" relative_yaw=%lf, relative_pitch=%lf, weight for candidate point %d to give with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,relative_pitch, i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points->weight[i]);

	  exit(0);
	}
     //// printf(" relative_yaw=%lf, relative_pitch=%lf, weight for candidate point %d to give with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,relative_pitch, i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points->weight[i]);

      /*
    ////Assigning weight based on the closeness to the point which is at dist 0.3 m from the human 
    double human_pos[3];
    double sig_hum_dist=0.7;
    double mean_dist=0.3;
    human_pos[0]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3];
    human_pos[1]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3];
    human_pos[2]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3];
    ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
    double point_to_human_dist=sqrt((human_pos[0]-point_in_global_frame[0])*(human_pos[0]-point_in_global_frame[0])+(human_pos[1]-point_in_global_frame[1])*(human_pos[1]-point_in_global_frame[1])+(human_pos[2]-point_in_global_frame[2])*(human_pos[2]-point_in_global_frame[2]));
    printf(" point to human dist = %lf \n", point_to_human_dist);
    candidate_points_to_put.weight[i]+=Amplitude/2.0*exp(-(((mean_dist-point_to_human_dist)*(mean_dist-point_to_human_dist)/2.0*sig_hum_dist*sig_hum_dist)));
      */

      /*
    ////Assigning weight based on the closeness to the bottle current position
    double bottle_pos[3];
    double sig_dist=2.0;
    bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
    bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
    bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
    ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
    double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
    candidate_points_to_show.weight[i]+=max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
      */
      /*if(fabs(point_to_bottle_dist)<0.00001)
	candidate_points_to_put.weight[i]+=1;
	else
	candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
      */
    } 
  
  /*
//////////printf(" max_weight=%lf\n",max_weight);
int obj_index=get_index_of_robot_by_name ( object_name );

for(i=0;i<candidate_points_to_give.no_points;i++)
{
point_in_global_frame[0] = candidate_points_to_give.point[i].x;
point_in_global_frame[1] = candidate_points_to_give.point[i].y;
point_in_global_frame[2] = candidate_points_to_give.point[i].z;
point_in_global_frame[3] = 1;
////Assigning weight based on the closeness to the bottle current position
double bottle_pos[3];
double sig_dist=2.0;
  
bottle_pos[0]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[0][3];
bottle_pos[1]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[1][3];
bottle_pos[2]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[2][3];
////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
//////////printf(" Old weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
candidate_points_to_give.weight[i]+=2.0*max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  
//////////printf(" New weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
}
  */
  
  MY_FREE(hum_cur_pos,double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof);

  return 1;
}


int assign_weights_on_candidte_points_to_give_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent)
{
  int i;
  p3d_vector4 meanPoint;
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos, hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_mat4Copy(envPt_MM->robot[indx_for_agent]->joints[HUMAN_J_NECK_PAN]->abs_pos, hum_pos);
  p3d_matInvertXform(hum_pos, hum_pos_inverse);
  configPt hum_cur_pos = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); /* Allocation of temporary robot configuration */

  for(i=0;i<3;++i)
    meanPoint[i]= hum_pos[i][3] + 0.5*hum_pos[i][0];
  //meanPoint[3]= 1.0;
  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&hum_cur_pos);


  ////double yaw=M_PI/3.0;

  ////double orig_pan=hum_cur_pos[HUMANq_PAN]; 
  i=0;
  double Amplitude=1.0;
  ////double yaw_mean=hum_cur_pos[HUMANq_PAN];
  ////double pitch_mean=hum_cur_pos[HUMANq_TILT];
  double yaw_mean=0;
  double pitch_mean=0;
  double sig_yaw=M_PI/2.0;
  double sig_pitch=M_PI/2.0;
  double max_weight=0;

  ////////////////printf("candidate_points_to_give.no_points=%d\n",candidate_points->no_points);

  for(i=0;i<candidate_points->no_points;i++)
    {
      point_in_global_frame[0] = candidate_points->point[i].x;
      point_in_global_frame[1] = candidate_points->point[i].y;
      point_in_global_frame[2] = candidate_points->point[i].z;
      point_in_global_frame[3] = 1;
  
  
      ////Assigning weight wrt human axis
      p3d_matvec4Mult(hum_pos_inverse, point_in_global_frame, point_in_human_frame);
      /////printf("%f %f %f\n", point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2]);
      ////  p3d_psp_cartesian2spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
      Cartesian_to_Spherical(point_in_human_frame[0],point_in_human_frame[1],point_in_human_frame[2],0,0,0,&relative_yaw,&relative_pitch);
      /////printf("%f %f\n", relative_yaw,relative_pitch);
  
      ////////candidate_points_to_put.weight[i]+=(1.0-(fabs(relative_yaw))/M_PI);

      ////printf(" For point %d \n",i);
      //     candidate_points_to_give.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)+((pitch_mean-relative_pitch)*(pitch_mean-relative_pitch)/2.0*sig_pitch*sig_pitch)));

      candidate_points->weight[i]= 1.0/( 2.0 + sqrt( pow(point_in_global_frame[0]-meanPoint[0], 2) + pow(point_in_global_frame[1]-meanPoint[1], 2) + pow(point_in_global_frame[2]-meanPoint[2], 2) ) );

      if(max_weight<candidate_points->weight[i])
	max_weight=candidate_points->weight[i];
      ////////candidate_points_to_show.weight[i]=Amplitude*exp(-(((yaw_mean-relative_yaw)*(yaw_mean-relative_yaw)/2.0*sig_yaw*sig_yaw)));
  
      ////printf(" max_weight=%lf\n",max_weight);
      if(max_weight>100)
	{
	  printf(" relative_yaw=%lf, relative_pitch=%lf, weight for candidate point %d to give with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,relative_pitch, i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points->weight[i]);

	  exit(0);
	}
      ////printf(" relative_yaw=%lf, relative_pitch=%lf, weight for candidate point %d to give with pos (%lf,%lf,%lf) is %lf\n",relative_yaw,relative_pitch, i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points->weight[i]);

      /*
    ////Assigning weight based on the closeness to the point which is at dist 0.3 m from the human 
    double human_pos[3];
    double sig_hum_dist=0.7;
    double mean_dist=0.3;
    human_pos[0]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[0][3];
    human_pos[1]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[1][3];
    human_pos[2]=ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[1]->abs_pos[2][3];
    ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
    double point_to_human_dist=sqrt((human_pos[0]-point_in_global_frame[0])*(human_pos[0]-point_in_global_frame[0])+(human_pos[1]-point_in_global_frame[1])*(human_pos[1]-point_in_global_frame[1])+(human_pos[2]-point_in_global_frame[2])*(human_pos[2]-point_in_global_frame[2]));
    printf(" point to human dist = %lf \n", point_to_human_dist);
    candidate_points_to_put.weight[i]+=Amplitude/2.0*exp(-(((mean_dist-point_to_human_dist)*(mean_dist-point_to_human_dist)/2.0*sig_hum_dist*sig_hum_dist)));
      */

      /*
    ////Assigning weight based on the closeness to the bottle current position
    double bottle_pos[3];
    double sig_dist=2.0;
    bottle_pos[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
    bottle_pos[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
    bottle_pos[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
    ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
    double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
    candidate_points_to_show.weight[i]+=max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
      */
      /*if(fabs(point_to_bottle_dist)<0.00001)
	candidate_points_to_put.weight[i]+=1;
	else
	candidate_points_to_put.weight[i]+=(1.0/point_to_bottle_dist);
      */
    } 
  
  /*
//////////printf(" max_weight=%lf\n",max_weight);
int obj_index=get_index_of_robot_by_name ( object_name );

for(i=0;i<candidate_points_to_give.no_points;i++)
{
point_in_global_frame[0] = candidate_points_to_give.point[i].x;
point_in_global_frame[1] = candidate_points_to_give.point[i].y;
point_in_global_frame[2] = candidate_points_to_give.point[i].z;
point_in_global_frame[3] = 1;
////Assigning weight based on the closeness to the bottle current position
double bottle_pos[3];
double sig_dist=2.0;
  
bottle_pos[0]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[0][3];
bottle_pos[1]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[1][3];
bottle_pos[2]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[2][3];
////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
//////////printf(" Old weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
candidate_points_to_give.weight[i]+=2.0*max_weight*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  
//////////printf(" New weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
}
  */
  
  MY_FREE(hum_cur_pos,double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof);

  return 1;
}

int assign_weights_on_candidte_points_to_hide_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent)
{
  
  p3d_vector4 point_in_global_frame;
  p3d_vector4 point_in_human_frame;
  p3d_matrix4 hum_pos_inverse;
  double relative_yaw, relative_pitch; 
  
  p3d_matInvertXform(envPt_MM->robot[rob_indx.HUMAN]->joints[HUMAN_J_NECK_PAN]->abs_pos, hum_pos_inverse);

  configPt hum_cur_pos = MY_ALLOC(double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof); /* Allocation of temporary robot configuration */

  p3d_get_robot_config_into(envPt_MM->robot[rob_indx.HUMAN],&hum_cur_pos);


  ////double yaw=M_PI/3.0;

  ////double orig_pan=hum_cur_pos[HUMANq_PAN]; 
  int i=0;
  double Amplitude=1.0;
  double yaw_mean=hum_cur_pos[HUMANq_PAN];
  double pitch_mean=hum_cur_pos[HUMANq_TILT];
  double sig_yaw=M_PI/2.0;
  double sig_pitch=M_PI/2.0;
  double max_weight=0;
  ////int i=0;
  double bottle_pos[3];
  double sig_dist=2.0;
  int obj_index=get_index_of_robot_by_name ( object_name );

  bottle_pos[0]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[0][3];
  bottle_pos[1]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[1][3];
  bottle_pos[2]=envPt_MM->robot[obj_index]->joints[1]->abs_pos[2][3];

  for(i=0;i<candidate_points->no_points;i++)
    {
      point_in_global_frame[0] = candidate_points->point[i].x;
      point_in_global_frame[1] = candidate_points->point[i].y;
      point_in_global_frame[2] = candidate_points->point[i].z;
      point_in_global_frame[3] = 1;
      ////Assigning weight based on the closeness to the bottle current position
  
      ////printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",bottle_pos[0],bottle_pos[1],bottle_pos[2]); 
      double point_to_bottle_dist=sqrt((bottle_pos[0]-point_in_global_frame[0])*(bottle_pos[0]-point_in_global_frame[0])+(bottle_pos[1]-point_in_global_frame[1])*(bottle_pos[1]-point_in_global_frame[1])+(bottle_pos[2]-point_in_global_frame[2])*(bottle_pos[2]-point_in_global_frame[2]));
  
      ////printf(" Old weight for candidate point %d to put with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_show.weight[i]);
      candidate_points->weight[i]=Amplitude*exp(-(((point_to_bottle_dist)*(point_to_bottle_dist)/2.0*sig_dist*sig_dist)));
  
      //////////printf(" New weight for candidate point %d to hide with pos (%lf,%lf,%lf) is %lf\n", i,point_in_global_frame[0],point_in_global_frame[1],point_in_global_frame[2],candidate_points_to_hide.weight[i]);
    }
  
  MY_FREE(hum_cur_pos,double,envPt_MM->robot[rob_indx.HUMAN]->nb_dof);

  return 1;
}


int find_candidate_points_on_plane_to_hide_obj_new()
{
#ifndef COMMENT_TMP
  //////////printf(" Inside find_candidate_points_to_hide_obj_new()\n");
  ////no_candidate_points_to_show=0;
  candidate_points_to_hide.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		{
		  reachable_by_hand=0;
		  reachable_by_HRP2_hand=0;
		  visible_by_human=0;
		  visible_by_HRP2=0;
		  reachable_by_turning_around=0;
		  reachable_by_bending=0;

		  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
		  ////////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==0)
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_CURRENT_STATE_HUM_VIS]==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS]==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS]==0)
		    {
		      ////////// printf("* Not Visible by human\n");
		      ////////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2_neck_turn==1)
		      ////////////{
		      ////////////printf("** Visible by HRP2 neck turn\n");
		      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
		      ////{
		      //// printf("*** Bending reachable by by human \n");

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand[MM_CURRENT_STATE_HRP2_REACH]==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand[MM_CURRENT_STATE_HRP2_REACH]==1)
			{
			  //////////  printf("**** Reachable by HRP2 \n");
			  candidate_points_to_hide.point[candidate_points_to_hide.no_points].x=cell_x_world;
			  candidate_points_to_hide.point[candidate_points_to_hide.no_points].y=cell_y_world;
			  candidate_points_to_hide.point[candidate_points_to_hide.no_points].z=cell_z_world;
			  candidate_points_to_hide.weight[candidate_points_to_hide.no_points]=0.0;
			  candidate_points_to_hide.no_points++;

			} 
		      ////} 
		      ////////////} 
		    } 
		}   
	    }  
	}
    }
      
  //////////printf(" candidate_points_to_hide.no_points=%d\n",candidate_points_to_hide.no_points);
  return candidate_points_to_hide.no_points;
#endif
}

int find_candidate_points_for_current_HRI_task(HRI_TASK_TYPE curr_task, HRI_TASK_AGENT_ENUM performed_by, HRI_TASK_AGENT_ENUM performed_for, candidate_poins_for_task *resultant_candidate_point)
{
  printf(" >>> Inside find_candidate_points_for_current_HRI_task with task %d \n",curr_task);
  HRI_TASK_AGENT state_for_agent=performed_by;
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_vis_states);
   printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_vis_states);
  
  state_for_agent=performed_for;
  
 printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_vis_states);
   printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_vis_states);
  
  resultant_candidate_point->no_points=0;
   int need_placement_on_plane=0;
   if(curr_task==MAKE_OBJECT_ACCESSIBLE||curr_task==HIDE_AWAY_OBJECT||curr_task==HIDE_OBJECT)
   {
   need_placement_on_plane=1;  
   }
   int i,j,k;
   int x=0;
   int y=0;
   int z=0;
   int cell_OK_vis=0;
   int cell_OK_reach=0;
   double cell_x_world;
   double cell_y_world;
   double cell_z_world;
     HRI_TASK_AGENT test_for_agent;
   
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
             if(need_placement_on_plane==1&& grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface!=1)
	     {
	       continue;
	     }
	     
	     test_for_agent=performed_by;
	     
	     cell_OK_reach=0;
	     
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=1;
         		   break;
 
		    }
		   }
		   if(cell_OK_reach==1)
			   break;
		 ////}
		 
	       }
	       
		if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states==0)
		 cell_OK_reach=1;
		
		if(cell_OK_reach==0)
		  continue;
		
		  cell_OK_vis=0;
	       for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=1;
		   break;
		   }
	       }
		 ////}
		
	       
	       if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states==0)
		 cell_OK_vis=1;

	       
	       if(cell_OK_vis==0)
		 continue;
	       
	       		
	       //Now testing for the target agent
		 test_for_agent=performed_for;
		 cell_OK_reach=0;
		 
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=1;
         		   break;
 
		    }
		   }
		   if(cell_OK_reach==1)
			   break;
		 ////}
		 
	       }
	       
		if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states==0)
		 cell_OK_reach=1;
		
		if(cell_OK_reach==0)
		  continue;
				
		
		  cell_OK_vis=0;
	       for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=1;
		   break;
		   }
	       }
	       
		 if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states==0)
		 cell_OK_vis=1;

	       
	       if(cell_OK_vis==0)
		 continue;
	       
	       /////// Now check for non acceptable Mightability
		 test_for_agent=performed_by;
	     
	     
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=0;
         		   break;
 
		    }
		   }
		   if(cell_OK_reach==0)
			   break;
		 ////}
		 
	       }
	       
		if(cell_OK_reach==0)
		  continue;
		
		for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=0;
		     
		   break;
		   }
	       }
		 ////}
		
	       if(cell_OK_vis==0)
		 continue;
	       
	       //Now testing for the target agent
		 test_for_agent=performed_for;
		 
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=0;
         		   break;
 
		    }
		   }
		   if(cell_OK_reach==0)
			   break;
		 ////}
		 
	       }
	       
		
		if(cell_OK_reach==0)
		  continue;
		
		////static int non_accept_vis_filter_ctr=0;
	       for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   ////printf(" testing for cell %d, %d, %d for non visibility\n",x,y,z);
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=0;
		     ////printf(" Test fail\n");
		     ////non_accept_vis_filter_ctr++;
		     break;
		   }
	       }
		////printf(" >>>>> non_accept_vis_filter_ctr=%d\n",non_accept_vis_filter_ctr);
	       if(cell_OK_vis==0)
		 continue;
	       
	       
	       //Now the current cell satisfies all the constraints 
	      cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
	      resultant_candidate_point->point[resultant_candidate_point->no_points].x=cell_x_world;
	      resultant_candidate_point->point[resultant_candidate_point->no_points].y=cell_y_world;	  resultant_candidate_point->point[resultant_candidate_point->no_points].z=cell_z_world;
	      
	      resultant_candidate_point->weight[resultant_candidate_point->no_points]=0;	
	      
	      resultant_candidate_point->no_points++;
	      
	    }
	}
    }
    printf("resultant_candidate_point->no_points=%d\n",resultant_candidate_point->no_points);
    return 1;
}

int JIDO_find_candidate_points_to_show_obj()
{
#ifndef COMMENT_TMP
  //////////printf(" Inside find_candidate_points_to_show_obj_new()\n");
  ////no_candidate_points_to_show=0;
  candidate_points_to_show.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].is_horizontal_surface==1)
	      ////{
	      reachable_by_hand=0;
	      reachable_by_HRP2_hand=0;
	      visible_by_human=0;
	      visible_by_HRP2=0;
	      reachable_by_turning_around=0;
	      reachable_by_bending=0;

	      double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[MM_CURRENT_STATE_HUM_VIS]==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human[]==1)
		{
		  //////////printf("* Visible by human\n");
		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
		  ////{
		  ////////// printf("** Visible by HRP2\n");
		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
		  //////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
		  ////{
		  //// printf("*** Bending reachable by by human \n");
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_JIDO_neck_turn==1)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			{
			  ////////// printf("**** Reachable by HRP2 \n");
			  candidate_points_to_show.point[candidate_points_to_show.no_points].x=cell_x_world;
			  candidate_points_to_show.point[candidate_points_to_show.no_points].y=cell_y_world;
			  candidate_points_to_show.point[candidate_points_to_show.no_points].z=cell_z_world;
			  candidate_points_to_show.weight[candidate_points_to_show.no_points]=0;
			  candidate_points_to_show.no_points++;
			  ////printf(" candidate_points_to_show.no_points = %d \n",candidate_points_to_show.no_points);
			  /*if(candidate_points_to_show.no_points>900)
			    {
			    printf(" candidate_points_to_show.no_points = %d \n");
			    return candidate_points_to_show.no_points;
			    } */

			} 
		      ////} 
		    } 
		} 
	      ////}   
	    }  
	}
    }
      


  //////////printf(" candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);
  return candidate_points_to_show.no_points;
#endif
}

int JIDO_find_candidate_points_to_give_obj()
{
#ifndef COMMENT_TMP
  //////////printf(" Inside find_candidate_points_to_show_obj_new()\n");
  ////no_candidate_points_to_show=0;
  candidate_points_to_give.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].is_horizontal_surface==1)
	      ////{
	      reachable_by_hand=0;
	      reachable_by_HRP2_hand=0;
	      visible_by_human=0;
	      visible_by_HRP2=0;
	      reachable_by_turning_around=0;
	      reachable_by_bending=0;

	      double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1)
		{
		  //////////printf("* Visible by human\n");
		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
		  ////{
		  ////////// printf("** Visible by HRP2\n");
		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
		    {
		      //// printf("*** Bending reachable by by human \n");

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			{
			  ////////// printf("**** Reachable by HRP2 \n");
			  candidate_points_to_give.point[candidate_points_to_give.no_points].x=cell_x_world;
			  candidate_points_to_give.point[candidate_points_to_give.no_points].y=cell_y_world;
			  candidate_points_to_give.point[candidate_points_to_give.no_points].z=cell_z_world;
			  candidate_points_to_give.weight[candidate_points_to_give.no_points]=0;
			  candidate_points_to_give.no_points++;
			  ////printf(" candidate_points_to_show.no_points = %d \n",candidate_points_to_show.no_points);
			  /*if(candidate_points_to_show.no_points>900)
			    {
			    printf(" candidate_points_to_show.no_points = %d \n");
			    return candidate_points_to_show.no_points;
			    } */

			} 
		    } 
		  ////} 
		} 
	      ////}   
	    }  
	}
    }
      


  //////////printf(" candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);
  return candidate_points_to_give.no_points;
#endif
}


int find_candidate_points_to_give_obj()
{
#ifndef COMMENT_TMP
  //////////printf(" Inside find_candidate_points_to_show_obj_new()\n");
  ////no_candidate_points_to_show=0;
  candidate_points_to_give.no_points=0;

 

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].is_horizontal_surface==1)
	      ////{
  
	      double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1)
		{
		  //////////printf("* Visible by human\n");
		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1)
		  ////{
		  ////////// printf("** Visible by HRP2\n");
		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
		    {
		      //// printf("*** Bending reachable by by human \n");

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			{
			  ////////// printf("**** Reachable by HRP2 \n");
			  candidate_points_to_give.point[candidate_points_to_give.no_points].x=cell_x_world;
			  candidate_points_to_give.point[candidate_points_to_give.no_points].y=cell_y_world;
			  candidate_points_to_give.point[candidate_points_to_give.no_points].z=cell_z_world;
			  candidate_points_to_give.weight[candidate_points_to_give.no_points]=0;
			  candidate_points_to_give.no_points++;
			  ////printf(" candidate_points_to_show.no_points = %d \n",candidate_points_to_show.no_points);
			  /*if(candidate_points_to_show.no_points>900)
			    {
			    printf(" candidate_points_to_show.no_points = %d \n");
			    return candidate_points_to_show.no_points;
			    } */

			} 
		    } 
		  ////} 
		} 
	      ////}   
	    }  
	}
    }
      


  //////////printf(" candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);
  return candidate_points_to_give.no_points;
#endif
}


int find_candidate_points_to_show_obj_new()
{
#ifndef COMMENT_TMP
  //////////printf(" Inside find_candidate_points_to_show_obj_new()\n");
  ////no_candidate_points_to_show=0;
  candidate_points_to_show.no_points=0;



  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].is_horizontal_surface==1)
	      {
     
		double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
		if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1)
		  {
		    //////////printf("* Visible by human\n");
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_straight_head_orientation==1||  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_HRP2_neck_turn==1)
		      {
			////////// printf("** Visible by HRP2\n");
			////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.reachable_by_RHand_by_bending==1)
			{
			  //// printf("*** Bending reachable by by human \n");

			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			    {
			      ////////// printf("**** Reachable by HRP2 \n");
			      candidate_points_to_show.point[candidate_points_to_show.no_points].x=cell_x_world;
			      candidate_points_to_show.point[candidate_points_to_show.no_points].y=cell_y_world;
			      candidate_points_to_show.point[candidate_points_to_show.no_points].z=cell_z_world;
			      candidate_points_to_show.weight[candidate_points_to_show.no_points]=0;
			      candidate_points_to_show.no_points++;

			    } 
			} 
		      } 
		  } 
	      } 
	    } 
	}
    }
      
  //////////printf(" candidate_points_to_show.no_points=%d\n",candidate_points_to_show.no_points);
  return candidate_points_to_show.no_points;
#endif
}

int find_candidate_points_on_plane_to_displace_obj()//(int type)//type=1 means only directly reachable places by human, 2 mean also bending reachable places by human
{
#ifndef COMMENT_TMP
  printf(" \n****Inside find_candidate_points_on_plane_to_displace_obj()\n");

  candidate_points_to_displace_obj.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		{
		  reachable_by_hand=0;
		  reachable_by_HRP2_hand=0;
		  visible_by_human=0;
		  visible_by_HRP2=0;
		  reachable_by_turning_around=0;
		  reachable_by_bending=0;

		  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
		  //////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==1)
		  {
		    ////printf("* Visible by human\n");
		    ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1)
		    {
		      ////printf("** Visible by HRP2\n");
		      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==1)
		      ////////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
		      {
			////printf("*** Reachable by by human \n");
#ifdef HRP2_EXISTS_FOR_MA
			if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			  {
			    ////printf("**** Reachable by HRP2 \n");
			    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==0)//&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_LHand_by_bending==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_RHand_by_bending==0)
			      {
				////printf("**** Reachable by HRP2 \n");
				candidate_points_to_displace_obj.point[candidate_points_to_displace_obj.no_points].x=cell_x_world;
				candidate_points_to_displace_obj.point[candidate_points_to_displace_obj.no_points].y=cell_y_world;
				candidate_points_to_displace_obj.point[candidate_points_to_displace_obj.no_points].z=cell_z_world;
				candidate_points_to_displace_obj.weight[candidate_points_to_displace_obj.no_points]=0;
				candidate_points_to_displace_obj.no_points++;
			      }
			  } 
#endif
#ifdef JIDO_EXISTS_FOR_MA
			if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			  {
			    ////printf("**** Reachable by HRP2 \n");
			    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==0)
			      {
				////printf("**** Reachable by HRP2 \n");
				candidate_points_to_displace_obj.point[candidate_points_to_displace_obj.no_points].x=cell_x_world;
				candidate_points_to_displace_obj.point[candidate_points_to_displace_obj.no_points].y=cell_y_world;
				candidate_points_to_displace_obj.point[candidate_points_to_displace_obj.no_points].z=cell_z_world;
				candidate_points_to_displace_obj.weight[candidate_points_to_displace_obj.no_points]=0;
				candidate_points_to_displace_obj.no_points++;
			      }

			  } 
#endif
		      } 
		    } 
		  } 
		}   
	    }  
	}
    }
      
  printf(" candidate_points_to_displace_obj.no_points=%d\n",candidate_points_to_displace_obj.no_points);
  return candidate_points_to_displace_obj.no_points;
#endif
}

int find_candidate_points_on_plane_to_put_obj_new()//(int type)//type=1 means only directly reachable places by human, 2 mean also bending reachable places by human
{
#ifndef COMMENT_TMP
  //////////printf(" \n****Inside find_candidate_points_on_plane_to_put_obj_new()\n");
  no_candidate_points_to_put=0;
  candidate_points_to_put.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_HRP2=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		{
		  reachable_by_hand=0;
		  reachable_by_HRP2_hand=0;
		  visible_by_human=0;
		  visible_by_HRP2=0;
		  reachable_by_turning_around=0;
		  reachable_by_bending=0;

		  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1)
		    {
		      ////printf("* Visible by human\n");
		      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1)
		      {
			////printf("** Visible by HRP2\n");
			if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
			  {
			    ////printf("*** Reachable by by human \n");
#ifdef HRP2_EXISTS_FOR_MA
			    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand==1)
			      {
				////printf("**** Reachable by HRP2 \n");
				candidate_points_to_put.point[candidate_points_to_put.no_points].x=cell_x_world;
				candidate_points_to_put.point[candidate_points_to_put.no_points].y=cell_y_world;
				candidate_points_to_put.point[candidate_points_to_put.no_points].z=cell_z_world;
				candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
				candidate_points_to_put.no_points++;

			      } 
#endif
#ifdef JIDO_EXISTS_FOR_MA
			    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			      {
				////printf("**** Reachable by HRP2 \n");
				candidate_points_to_put.point[candidate_points_to_put.no_points].x=cell_x_world;
				candidate_points_to_put.point[candidate_points_to_put.no_points].y=cell_y_world;
				candidate_points_to_put.point[candidate_points_to_put.no_points].z=cell_z_world;
				candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
				candidate_points_to_put.no_points++;

			      } 
#endif
			  } 
		      } 
		    } 
		}   
	    }  
	}
    }
      
  //////////printf(" candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);
  return candidate_points_to_put.no_points;
#endif
}

int JIDO_find_candidate_points_on_plane_to_put_obj()
{
#ifndef COMMENT_TMP
  ////////// printf(" \n****Inside JIDO_find_candidate_points_on_plane_to_put_obj()\n");
  no_candidate_points_to_put=0;
  candidate_points_to_put.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_JIDO_hand=0; 
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_JIDO=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)////&&(strcasestr(envPt_MM->robot[grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of]->name,"SHELF_TABLE")))
		{
		  reachable_by_hand=0;
		  reachable_by_JIDO_hand=0;
		  visible_by_human=0;
		  visible_by_JIDO=0;
		  reachable_by_turning_around=0;
		  reachable_by_bending=0;

		  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		  ////double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
                  double cell_z_world=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.exact_z_val;
    
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==1)
		    {
		      ////////// printf("* Visible by human\n");
		      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1)
		      {
			////printf("** Visible by HRP2\n");
			////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
			if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
			  {
			    //////////printf("*** Reachable by by human \n");

			    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			      {
				////////// printf("**** Reachable by HRP2 \n");
				candidate_points_to_put.point[candidate_points_to_put.no_points].x=cell_x_world;
				candidate_points_to_put.point[candidate_points_to_put.no_points].y=cell_y_world;
				candidate_points_to_put.point[candidate_points_to_put.no_points].z=cell_z_world;
				candidate_points_to_put.horizontal_surface_of[candidate_points_to_put.no_points]=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of;
				candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
				candidate_points_to_put.no_points++;

			      } 
			  } 
		      } 
		    } 
		}   
	    }  
	}
    }
      
  //////////printf(" candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);
  return candidate_points_to_put.no_points;
#endif
}



int JIDO_find_candidate_points_on_plane_to_hide_obj()
{
#ifndef COMMENT_TMP
  ////////// printf(" \n****Inside JIDO_find_candidate_points_on_plane_to_put_obj()\n");
  //////no_candidate_points_to_hide=0;
  candidate_points_to_hide.no_points=0;

  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
  int reachable_by_JIDO_hand=0; 
  int reachable_by_bending=0;
  int reachable_by_turning_around=0;
  int visible_by_human=0;
  int visible_by_JIDO=0;

  int x=0;
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      int y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  int z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
	      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
	      ////{
	      ////grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_human=0;
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)
		{
		  reachable_by_hand=0;
		  reachable_by_JIDO_hand=0;
		  visible_by_human=0;
		  visible_by_JIDO=0;
		  reachable_by_turning_around=0;
		  reachable_by_bending=0;

		  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
    
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_straight_head_orientation==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible_by_human_neck_turn==0)
		    {
		      ////printf("* Not Visible by human\n");
		      ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].affordances.visible_by_HRP2==1)
		      {
			////printf("** Visible by HRP2\n");
			//////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
			////////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==0)
			////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==0)
			{
			  //////////printf("*** Reachable by by human \n");

			  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand==1)
			    {
			      ////////// printf("**** Reachable by HRP2 \n");
			      candidate_points_to_hide.point[candidate_points_to_hide.no_points].x=cell_x_world;
			      candidate_points_to_hide.point[candidate_points_to_hide.no_points].y=cell_y_world;
			      candidate_points_to_hide.point[candidate_points_to_hide.no_points].z=cell_z_world;
			      candidate_points_to_hide.weight[candidate_points_to_hide.no_points]=0;
			      candidate_points_to_hide.no_points++;

			    } 
			} 
		      } 
		    } 
		}   
	    }  
	}
    }
      
  //////////printf(" candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);
  return candidate_points_to_hide.no_points;
#endif
}


int find_candidate_points_on_plane_to_put_obj()
{
#ifndef COMMENT_TMP
  no_candidate_points_to_put=0;
  candidate_points_to_put.no_points=0;

  int current_surface_index=0;
  for(;current_surface_index<curr_surfaces_in_env.total_no_of_surfaces;current_surface_index++)
    {

      int i=0;
      for(;i<curr_surfaces_in_env.flat_surf[current_surface_index].grid_i_max;i++)
	{
	  int j=0;
	  int show_cell=0;
	  int reachable_by_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
	  int reachable_by_HRP2_hand=0; //0 Means not reachable by any hand, 1 means reachable by right hand only, 2 means reachable by left hand only, 3 means reachable by both hands;
	  int reachable_by_bending=0;
	  int reachable_by_turning_around=0;
	  int visible_by_human=0;
	  int visible_by_HRP2=0;
	  //printf("\n");
	  for(;j<curr_surfaces_in_env.flat_surf[current_surface_index].grid_j_max;j++)
	    {
	      show_cell=0;
	      reachable_by_hand=0;
	      reachable_by_HRP2_hand=0;
	      visible_by_human=0;
	      visible_by_HRP2=0;
	      reachable_by_turning_around=0;
	      reachable_by_bending=0;

	      double x=i*surf_grid_samp_rate;
	      double y=j*surf_grid_samp_rate;
	      x+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_x_min;
	      y+=curr_surfaces_in_env.flat_surf[current_surface_index].BR_y_min;
	      double z=curr_surfaces_in_env.flat_surf[current_surface_index].BR_z;

	      double cur_z_st=z;
	      double height=0.1;//0.3;
	      double cur_z_end=z;
	      //double cur_z_end=;
	      //printf("%d ", curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible);
	      ////printf("SHOW_VISIBLE_PLACE=%d\n",SHOW_VISIBLE_PLACE);
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible==1)
		{
 
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
		  //cur_z_st+=cur_z_end;
		  //cur_z_end+=cur_z_st;
		  show_cell=1;
		  visible_by_human=1;

		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].visible_by_HRP2==1)
		{
 
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.2, 6, 3, NULL);
		  //cur_z_st+=cur_z_end;
		  //cur_z_end+=cur_z_st;
		  show_cell=1;
		  visible_by_HRP2=1;
		}

	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_RHand==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
		  show_cell=1;
		  reachable_by_hand=1;
		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_LHand==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
		  show_cell=1;
		  if(reachable_by_hand==1)//Already reachable by right hand
		    reachable_by_hand=3; //Reachable by both hands
		  else
		    reachable_by_hand=2; //Reachable by left hand only
  
		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_RHand_by_bending==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
		  show_cell=1;
		  reachable_by_bending=1;
		  //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_LHand_by_bending==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
		  show_cell=1;
		  if(reachable_by_bending==1)//Already reachable by right hand
		    reachable_by_bending=3; //Reachable by both hands
		  else
		    reachable_by_bending=2; //Reachable by left hand only
		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_RHand_by_turning_around_bending==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
		  show_cell=1;
		  reachable_by_turning_around=1;
		  ////   g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
    
		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_human_LHand_by_turning_around_bending==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
		  show_cell=1;

		  if(reachable_by_turning_around==1)//Already reachable by right hand
		    reachable_by_turning_around=3; //Reachable by both hands
		  else
		    reachable_by_turning_around=2; //Reachable by left hand only
		}

	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_RHand==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.3, 4, 4, NULL);
		  show_cell=1;
		  reachable_by_HRP2_hand=1;
		  //// g3d_drawDisc(x, y, z+0.02, surf_grid_samp_rate/2.0-0.0075, 4, NULL);
   
		}
	      if(curr_surfaces_in_env.flat_surf[current_surface_index].surf_grid[i][j].reachable_by_HRP2_LHand==1)
		{
		  //g3d_draw_line_with_width(x,y,z,x,y,z+0.4, 2, 6, NULL);
		  show_cell=1;
		  if(reachable_by_HRP2_hand==1)//Already reachable by right hand
		    reachable_by_HRP2_hand=3; //Reachable by both hands
		  else
		    reachable_by_HRP2_hand=2; //Reachable by left hand only
		}
    
  
   
	      ////if(visible_by_HRP2==1&&visible_by_human==1)
	      if(visible_by_human==1) 
		{
		  if(reachable_by_HRP2_hand==3&&(reachable_by_hand==3||reachable_by_bending==3||reachable_by_turning_around==3)) //Reachable by both hands of both human and HRP2
		    {
		      ////g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 4, NULL);
     
		      candidate_points_to_put.point[candidate_points_to_put.no_points].x=x;
		      candidate_points_to_put.point[candidate_points_to_put.no_points].y=y;
		      candidate_points_to_put.point[candidate_points_to_put.no_points].z=z;
		      candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
		      candidate_points_to_put.no_points++;
		      ////candidate_points_to_put[no_candidate_points_to_put].x=x;
		      ////candidate_points_to_put[no_candidate_points_to_put].y=y;
		      ////candidate_points_to_put[no_candidate_points_to_put].z=z;
		      ////no_candidate_points_to_put++;
    
     
		    }
		  if((reachable_by_HRP2_hand==1||reachable_by_HRP2_hand==2)&&(reachable_by_hand==1||reachable_by_bending==1||reachable_by_turning_around==1||reachable_by_hand==2||reachable_by_bending==2||reachable_by_turning_around==2))
		    {
		      ////g3d_drawDisc(x, y, cur_z_end, surf_grid_samp_rate/2.0, 2, NULL);
		      candidate_points_to_put.point[candidate_points_to_put.no_points].x=x;
		      candidate_points_to_put.point[candidate_points_to_put.no_points].y=y;
		      candidate_points_to_put.point[candidate_points_to_put.no_points].z=z;
		      candidate_points_to_put.weight[candidate_points_to_put.no_points]=0;
		      candidate_points_to_put.no_points++;

		      ////candidate_points_to_put.point[no_candidate_points_to_put].x=x;
		      ////candidate_points_to_put.point[no_candidate_points_to_put].y=y;
		      ////candidate_points_to_put.point[no_candidate_points_to_put].z=z;
		      ////no_candidate_points_to_put++;
		    }
		} 
	    }
	}
    }
  return 1;
#endif
}

int get_index_of_robot_by_name(char *rob_name)
{
   if(rob_name==NULL)
   {
     return -1;
   }
  p3d_env *envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr,i;
 
  nr = envPt_MM->nr;
	
  for(i=0;i<nr;i++)
    {
       
		////printf(" comparing %s and %s \n",envPt_MM->robot[i]->name,rob_name);
      if (strcmp(envPt_MM->robot[i]->name,rob_name)==0)
	{
	  return i;
	}
	////else
	////{
	////  printf(" Fail \n");
	////}
    }
  return -1;
}
  
int make_cells_around_point_obstacle_free(double hand_pos[3], int expansion)
{ 
  //////////printf(" Inside  make_cells_around_point_obstacle_free=(%lf,%lf,%lf)\n",hand_pos[0],hand_pos[1],hand_pos[2]);
  hri_bitmap_cell* current_cell=NULL;
  current_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], hand_pos[0], hand_pos[1], hand_pos[2]);

  if(current_cell!=NULL)
    { 
      //////////  printf(" Inside  make_cells_around_point_obstacle_free, cell=(%d,%d,%d)\n",current_cell->x,current_cell->y,current_cell->z);
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[current_cell->x][current_cell->y][current_cell->z].val=0;
        

         
      int i=0; 
      for(i=current_cell->x-expansion;i<=current_cell->x+expansion;i++)
	{
	  //printf(" for i = %d\n",i);
	  if(i>=0&&i<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
            {
	      int j=0;
	      for(j=current_cell->y-expansion;j<=current_cell->y+expansion;j++)
		{
		  //printf(" for i, j = %d %d\n",i, j);
		  if(j>=0&&j<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		    {
		      int k=0;
		      for(k=current_cell->z-expansion;k<=current_cell->z+expansion;k++)
			{
			  //printf(" for i, j, k = %d %d %\n",i, j, k);
			  if(k>=0&&k<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			    {
			      ////printf(" Populating cell (%d, %d, %d) with 0\n",i,j,k);
			      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[i][j][k].val=0;
                 
        
			    }
			} 
		    }
		}
            } 
	}
    } 
  else
    {
      //////////printf(" Can't get cell corresponding to the hand\n");
    } 
  return 1;
}

int make_cells_around_point_as_near_to_obstacle(double hand_pos[3], int expansion)
{ 
  //////////printf(" Inside  make_cells_around_point_obstacle=(%lf,%lf,%lf)\n",hand_pos[0],hand_pos[1],hand_pos[2]);
  hri_bitmap_cell* current_cell=NULL;
  current_cell  =  hri_bt_get_closest_cell(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP], hand_pos[0], hand_pos[1], hand_pos[2]);

  if(current_cell!=NULL)
    { 
      //////////  printf(" Inside  make_cells_around_point_obstacle_free, cell=(%d,%d,%d)\n",current_cell->x,current_cell->y,current_cell->z);
      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[current_cell->x][current_cell->y][current_cell->z].val=0;
        
      int i=0; 
      for(i=current_cell->x-expansion;i<=current_cell->x+expansion;i++)
	{
	  //printf(" for i = %d\n",i);
	  if(i>=0&&i<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
            {
	      int j=0;
	      for(j=current_cell->y-expansion;j<=current_cell->y+expansion;j++)
		{
		  //printf(" for i, j = %d %d\n",i, j);
		  if(j>=0&&j<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		    {
		      int k=0;
		      for(k=current_cell->z-expansion;k<=current_cell->z+expansion;k++)
			{
			  //printf(" for i, j, k = %d %d %\n",i, j, k);
			  if(k>=0&&k<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			    {
			      ////printf(" Populating cell (%d, %d, %d) with 0\n",i,j,k);
			      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[i][j][k].val=-2;
                 
        
			    }
			} 
		    }
		}
            } 
	}
    } 
  else
    {
      //////////   printf(" Can't get cell corresponding to the hand\n");
    } 
  return 1;
}

/*int store_cur_GIK_sol_for_real_HRP2()
  {
  int i=0;
  for(i=0;i<HRP2_GIK_sol.no_configs;i++)
  {
  
  }
 
  }*/





int make_cells_around_point_as_obstacle(hri_bitmapset *btset, int bt_type, point_co_ordi point, int extension)
{
  hri_bitmap* bitmap;

  bitmap = btset->bitmap[bt_type];

  int x,y,z;

  x=(point.x-btset->realx)/btset->pace;  
  y=(point.y-btset->realy)/btset->pace;
  z=(point.z-btset->realz)/btset->pace;
  bitmap->data[x][y][z].val =-3;  //-3 to indicate that it has been marked as obstacle due to collision during path planning using GIK, it is not actual obstacle

  if((x>0&&x<bitmap->nx)&&(y>0&&y<bitmap->ny)&&(z>0&&z<bitmap->nz))
    {
      int x1=x-extension;
  
      for(x1=x-extension; x1<x+extension; x1++)
	{
	  int y1=y-extension; 
	  for(y1=y-extension;y1<y+extension;y1++) 
	    {
	      int z1=z-extension; 
	      for(z1=z-extension;z1<z+extension;z1++) 
   
		////printf(" Making cell (%d,%d,%d) as value 0 \n",x,y,z); 
		bitmap->data[x1][y1][z1].val =-3;  //-3 to indicate that it has been marked as obstacle due to collision during path planning using GIK, it is not actual obstacle
	    } 
	}
    }
  return 1;
}


int find_Object_Oriented_Mightabilities_vis_reach()
{

  hri_bitmap* bitmap=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP];
  ////envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  int i=0, j, k;
  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
      for(i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
      {
	for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
	{
	  object_MM.object[nr_ctr].geo_MM.visible[i][j]=0;
	}
 	for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
	{
	  for(k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
	  {
	     object_MM.object[nr_ctr].geo_MM.reachable[i][j][k]=0;
	  }
	}
      }
    }  

  int x,y,z;
  for(x=0; x<bitmap->nx; x++)
    {
      for(y=0; y<bitmap->ny; y++)
	{
	  for(z=0; z<bitmap->nz; z++)
	    {
  
	      int nr_ctr=0;
	      for(;nr_ctr<nr;nr_ctr++)
		{
    
		  ////if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]==1)
		    ////{
		      for(i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
		      {
		       if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]==1)
		       {
		       for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
			{
			 if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible[i][j]==1)
			 {
			  object_MM.object[nr_ctr].geo_MM.visible[i][j]++;//=1;
			 } 
			}
		       
		        for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		        {
			for(k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
	                 {
			  if(bitmap->data[x][y][z].Mightability_Map.reachable[i][j][k]==1)
			  {
	                  object_MM.object[nr_ctr].geo_MM.reachable[i][j][k]++;
			  }
	                 }
	                }
		       }
		      }	
		    ////}
		    
		    
		}
	    }
	}
    }  

  return 1;     
}

int find_symbolic_MM_visibility()
{

  hri_bitmap* bitmap=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP];
  ////envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  int i=0, j, k;
  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
      for(i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
      {
	for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
	{
	  object_MM.object[nr_ctr].geo_MM.visible[i][j]=0;
	}
// 	for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
// 	{
// 	  for(k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
// 	  {
// 	    
// 	  }
// 	}
      }
    }  

  int x,y,z;
  for(x=0; x<bitmap->nx; x++)
    {
      for(y=0; y<bitmap->ny; y++)
	{
	  for(z=0; z<bitmap->nz; z++)
	    {
  
	      int nr_ctr=0;
	      for(;nr_ctr<nr;nr_ctr++)
		{
    
		  if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]==1)
		    {
		      for(i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
		      {
		      for(j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
			{
			 if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible[i][j]==1)
			 {
			  object_MM.object[nr_ctr].geo_MM.visible[i][j]++;//=1;
			 } 
			}
		      }	
		    }
		}
	    }
	}
    }  

  return 1;     
}

int show_first_non_visible_cells(int obj_index)
{
#ifndef COMMENT_TMP
  hri_bitmap* bitmap=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP];
  int x,y,z;
  for(x=0; x<bitmap->nx; x++)
    {
      for(y=0; y<bitmap->ny; y++)
	{
	  for(z=0; z<bitmap->nz; z++)
	    {
  
	      //int nr_ctr=0;
	      //for(;nr_ctr<nr;nr_ctr++)
	      //{
    
	      if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[obj_index]==1)
		{

		  /////// FOR HUMAN

		  if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human==1)
		    {
		      double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		      double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		      double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		      g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);
     
		    }
     
		  if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation==1)
		    {
		      double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		      double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		      double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		      g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/2.0, Yellow, NULL);
		    }
  
   
		  if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human_neck_turn==1)
		    {
		      double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		      double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		      double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		      g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
		    }

		}
	    }
	}
    }
#endif
}

int get_object_mightabilities()
{
  ////printf(" Inside get_object_mightabilities()\n");
 
  Mightabilities_for_obj.total_no_obj=0;
  envPt_MM = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;

  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
      strcpy( Mightabilities_for_obj.object[Mightabilities_for_obj.total_no_obj].object_name, envPt_MM->robot[nr_ctr]->name);
      Mightabilities_for_obj.object[Mightabilities_for_obj.total_no_obj].object_index=nr_ctr;
      for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
      {
	      int ok_reach=0;

	for(int j=0;j<accepted_states_for_agent_obj_MA[i].no_accepted_reach_states;j++)
	{
	////for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;k++)
	 ////{
	  for(int l=0;l<agents_for_MA_obj.for_agent[i].no_of_arms;l++)
	  {
	   if(object_MM.object[nr_ctr].geo_MM.reachable[i][accepted_states_for_agent_obj_MA[i].accepted_reach[j]][l]>=1)
	   {
	     Mightabilities_for_obj.object[Mightabilities_for_obj.total_no_obj].obj_mightability.is_reachable_by[i]=1;
	     ok_reach=1;

	     break;
	   }
	   if(ok_reach==1)
	     break;
	  }
	 ////}
	} 
	
	int ok_vis=0;

	for(int j=0;j<accepted_states_for_agent_obj_MA[i].no_accepted_vis_states;j++)
	{
	   if(object_MM.object[nr_ctr].geo_MM.visible[i][accepted_states_for_agent_obj_MA[i].accepted_visibility[j]]>=1)
	   {
	     Mightabilities_for_obj.object[Mightabilities_for_obj.total_no_obj].obj_mightability.is_visible_by[i]=1;
	     ok_vis=1;

	     break;
	   }
	   if(ok_vis==1)
	     break;
	} 
      }

      
      Mightabilities_for_obj.total_no_obj++;
    }
    
  return 1;

}

int show_object_Mightabilities()
{
  ////printf(" Inside show_object_Mightabilities()\n");
  int i=0;
  double x,y,z;
  double radius=0.015;
 
  for(i=0;i<Mightabilities_for_obj.total_no_obj;i++)
    {
      int obj_index=Mightabilities_for_obj.object[i].object_index;
   
      if(strcasestr(envPt_MM->robot[obj_index]->name,"visball"))
	{
	  continue;
	}
  
  
      x=(envPt_MM->robot[obj_index]->BB.xmin+envPt_MM->robot[obj_index]->BB.xmax)/2.0;
      y=(envPt_MM->robot[obj_index]->BB.ymin+envPt_MM->robot[obj_index]->BB.ymax)/2.0;
      z=envPt_MM->robot[obj_index]->BB.zmax;

      if(SHOW_MM_BASED_OBJECT_REACHABLE==1)
	{
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_reachable_by[HUMAN1_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmin, envPt_MM->robot[obj_index]->BB.ymin, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Red, NULL);
	      g3d_drawDisc(x-radius, y-radius, z, radius, Red, NULL);
   
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    }  
  
#ifdef HUMAN2_EXISTS_FOR_MA
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_reachable_by[HUMAN2_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmin, envPt_MM->robot[obj_index]->BB.ymin, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Red, NULL);
	      g3d_drawDisc(x-radius, y-radius, z, radius+0.02, Yellow, NULL);
   
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    }  
#endif

#ifdef JIDO_EXISTS_FOR_MA
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_reachable_by[JIDO_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmax, envPt_MM->robot[obj_index]->BB.ymin, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Green, NULL);
	      g3d_drawDisc(x-radius, y+radius, z, radius, Green, NULL);
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    }
	} 
#endif

#ifdef HRP2_EXISTS_FOR_MA
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_reachable_by[HRP2_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmax, envPt_MM->robot[obj_index]->BB.ymin, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Green, NULL);
	      g3d_drawDisc(x-radius, y+radius, z, radius, Green, NULL);
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    }
	} 
#endif

      if(SHOW_MM_BASED_OBJECT_VISIBLE==1)
	{
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_visible_by[HUMAN1_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmin, envPt_MM->robot[obj_index]->BB.ymax, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Blue, NULL);
	      g3d_drawDisc(x+radius, y-radius, z, radius, Blue, NULL);
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    } 

#ifdef HUMAN2_EXISTS_FOR_MA
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_visible_by[HUMAN2_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmin, envPt_MM->robot[obj_index]->BB.ymax, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Blue, NULL);
	      g3d_drawDisc(x+radius, y-radius, z, radius+0.02, Green, NULL);
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    } 
#endif

#ifdef JIDO_EXISTS_FOR_MA
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_visible_by[JIDO_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmax, envPt_MM->robot[obj_index]->BB.ymax, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Yellow, NULL);
	      g3d_drawDisc(x+radius, y+radius, z, radius, Yellow, NULL);
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    } 
#endif	    

#ifdef HRP2_EXISTS_FOR_MA
	  if( Mightabilities_for_obj.object[obj_index].obj_mightability.is_visible_by[HRP2_MA]==1)
	    {
	      ////g3d_drawDisc(envPt_MM->robot[obj_index]->BB.xmax, envPt_MM->robot[obj_index]->BB.ymax, envPt_MM->robot[obj_index]->BB.zmax, 0.01, Yellow, NULL);
	      g3d_drawDisc(x+radius, y+radius, z, radius, Yellow, NULL);
	      ////Draw_Bounding_Box(envPt_MM->robot[obj_index]->BB.xmin,envPt_MM->robot[obj_index]->BB.ymin,envPt_MM->robot[obj_index]->BB.zmin,envPt_MM->robot[obj_index]->BB.xmax,envPt_MM->robot[obj_index]->BB.ymax,envPt_MM->robot[obj_index]->BB.zmax, Green);
   
	    } 
#endif	    
	}
    }

  return 1;

}


int find_symbolic_Mightability_Map_new()
{

  ////find_symbolic_MM_visibility();
  find_Object_Oriented_Mightabilities_vis_reach();
return 1; //Tmp not calculating putintoability

  hri_bitmap* bitmap=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP];
  ////envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  int i=0;
  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
       
	  /*for(i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	  {
	
	  for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
	    {
 	  for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
	      {
 	    object_MM.object[nr_ctr].geo_MM.reachable[i][j][k]=0;
	      }
	    }
	  }*/


      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.is_true=0;
      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.is_true=0;
      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.is_true=0;
 
      
      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.no_points=0;
      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points=0;
      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.no_points=0;
      ////object_MM.object[nr_ctr].visible_by_human=0;
    }  

  int x,y,z;
  double xc=0, yc=0, zc=0;
  
  /*
  nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
     
      xc=(envPt_MM->robot[nr_ctr]->BB.xmax+envPt_MM->robot[nr_ctr]->BB.xmin)/2.0;
      yc=(envPt_MM->robot[nr_ctr]->BB.ymax+envPt_MM->robot[nr_ctr]->BB.ymin)/2.0;
      zc=(envPt_MM->robot[nr_ctr]->BB.zmax+envPt_MM->robot[nr_ctr]->BB.zmin)/2.0;
    
     
      x=(xc- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;
     
  
      if(x>=0&&x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	{
      
	  y=(yc- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace;
      
	  if(y>=0&&y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
	    {
      
	      z=(zc- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace;
 
	      if(z>=0&&z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
		{
		      
    
		 
		  ////curr_cell=
                  for(i=0; i<MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN; i++)
		  {  
		     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand[i]++;
		    }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand[i]++;
		    }
		  }
	

#ifdef HUMAN2_EXISTS_FOR_MA
		  for(i=0; i<MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN; i++)
		  {  
		     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_LHand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_human2_LHand[i]++;
		    }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human2_RHand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_human2_RHand[i]++;
		    }
		  }
#endif

		  for(i=0; i<MAXI_NUM_POSSIBLE_STATES_REACH_HRP2; i++)
		  {  
		     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_LHand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_HRP2_LHand[i]++;
		    }
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_HRP2_RHand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_HRP2_RHand[i]++;
		    }
		  }


		  for(i=0; i<MAXI_NUM_POSSIBLE_STATES_REACH_JIDO; i++)
		  {  
		     if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_JIDO_Hand[i]==1)
		    {
		  object_MM.object[nr_ctr].geo_MM.reachable_by_JIDO_Hand[i]++;
		    }
		    
		  }
*/
		  ////Object based visibility
		  /*if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]==1)
		    {
		    object_MM.object[nr_ctr].visible_by_human=1;
		    }
		  */  
		  /*
		}
	    }
	}
   
*/
/*
      //// For calculating that something can be put inside the object by the agent
      if(strcasestr(envPt_MM->robot[nr_ctr]->name,"ROBOT")||strcasestr(envPt_MM->robot[nr_ctr]->name,"HUMAN1")||strcasestr(envPt_MM->robot[nr_ctr]->name,"TABLE"))
	{
	  //No need to calculate PUTABILITY :-)
	}
      else
	{
    
   
	  double increment=3.0/4.0*grid_around_HRP2.GRID_SET->pace;
	  double top_z=envPt_MM->robot[nr_ctr]->BB.zmax;
	  int cell_top_z=(top_z- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace;
     
	  double BB_x_ctr;
	  double BB_y_ctr;

	  if(cell_top_z>=0&&cell_top_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
	    { 
	      for(BB_x_ctr=envPt_MM->robot[nr_ctr]->BB.xmin;BB_x_ctr<envPt_MM->robot[nr_ctr]->BB.xmax;BB_x_ctr+=increment)
		{
		  int cell_x=(BB_x_ctr- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  

		  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
		    {
		      for(BB_y_ctr=envPt_MM->robot[nr_ctr]->BB.ymin;BB_y_ctr<envPt_MM->robot[nr_ctr]->BB.ymax;BB_y_ctr+=increment)
			{
      
			  int cell_y=(BB_y_ctr- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace;  
 
			  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
			    {

			      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].val>=0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z-1].val>=0)//No obstacle
				{

				  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_JIDO_Hand[MM_CURRENT_STATE_JIDO_REACH]==1)
				    {  
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.is_true=1;
        
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[
														 object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.no_points].x=BB_x_ctr; 
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[
														 object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.no_points].y=BB_y_ctr;
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[
														 object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.no_points].z=top_z;
 
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.no_points++;
				    }
       
				  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_human_LHand[MM_CURRENT_STATE_HUM_REACH]==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_human_RHand[MM_CURRENT_STATE_HUM_REACH]==1)
				    {  
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.is_true=1;
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.point[
														  object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points].x=BB_x_ctr; 
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.point[
														  object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points].y=BB_y_ctr;
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.point[
														  object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points].z=top_z;
 
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points++;
				    }  
  
				  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_HRP2_LHand[MM_CURRENT_STATE_HRP2_REACH]==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_top_z].Mightability_Map.reachable_by_HRP2_RHand[MM_CURRENT_STATE_HRP2_REACH]==1)
				    {  
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.is_true=1;

				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.point[
														 object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.no_points].x=BB_x_ctr; 

				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.point[
														 object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.no_points].y=BB_y_ctr;

				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.point[
														 object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.no_points].z=top_z;
 
				      object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_HRP2.no_points++;
				    }

				}
			    }
			} 
		    }
		}
	    }
	}
	*/
      /*
      //, so assume that the object corresponding to this obstacle is visible
          
      int i=0;
      for(i=0;i<nr;i++)
      {
           
      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.belongs_to_objects_indx[i]==1)
   
      /*int is_visible=psp_is_object_visible(ACBTSET->human[ACBTSET->actual_human]->HumanPt, envPt_MM->robot[nr_ctr], 50, FALSE); 
      if(is_visible==1)
      {
      object_MM.object[nr_ctr].visible_by_human=1;
      }*/
    ////}   
  return 1;

}

 
int find_symbolic_Mightability_Map()
{
#ifndef COMMENT_TMP
  hri_bitmap* bitmap=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP];
  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;

  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
      object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand_by_bending=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand_by_bending=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand_by_turning_around_bending=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand_by_turning_around_bending=0;
#ifdef HUMAN2_EXISTS_FOR_MA
      object_MM.object[nr_ctr].geo_MM.reachable_by_human2_LHand=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human2_RHand=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human2_LHand_by_bending=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human2_RHand_by_bending=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human2_LHand_by_turning_around_bending=0;
      object_MM.object[nr_ctr].geo_MM.reachable_by_human2_RHand_by_turning_around_bending=0;
#endif
    }  

  int x,y,z;
  for(x=0; x<bitmap->nx; x++)
    {
      for(y=0; y<bitmap->ny; y++)
	{
	  for(z=0; z<bitmap->nz; z++)
	    {
  
	      int nr_ctr=0;
	      for(;nr_ctr<nr;nr_ctr++)
		{
		  if(bitmap->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[nr_ctr]==1)
		    {
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand==1)
			{
			  object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand=1;
			}
       
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand==1)
			{
			  object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand=1;
			}

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_bending==1)
			{
			  object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand_by_bending=1;
			}  

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_bending==1)
			{
			  object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand_by_bending=1;
			}  

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_LHand_by_turning_around_bending==1)
			{
			  object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand_by_turning_around_bending=1;
			} 

		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable_by_human_RHand_by_turning_around_bending==1)
			{
			  object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand_by_turning_around_bending=1;
			}
 
      
		    }  
		  //bitmap->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[nr_ctr]=0;//Initially does not belong to any cell
		  //bitmap->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
		  //bitmap->data[x][y][z].Mightability_map_cell_obj_info.near_horizontal_surface=0;
		}
	      //bitmap->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
	      //bitmap->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;  
	    }
	}
    }   
  return 1;
#endif
}

int Draw_Bounding_Box(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z, int color)
{
  g3d_drawOneLine(
		  min_x, min_y, min_z,
		  max_x, min_y, min_z, color, NULL);

  g3d_drawOneLine(
		  max_x, min_y, min_z,
		  max_x, max_y, min_z, color, NULL);

  g3d_drawOneLine(
		  max_x, max_y, min_z,
		  min_x, max_y, min_z, color, NULL);

  g3d_drawOneLine(
		  min_x, max_y, min_z,
		  min_x, min_y, min_z, color, NULL);

  g3d_drawOneLine(
		  min_x, min_y, max_z,
		  max_x, min_y, max_z, color, NULL);

  g3d_drawOneLine(
		  max_x, min_y, max_z,
		  max_x, max_y, max_z, color, NULL);

  g3d_drawOneLine(
		  max_x, max_y, max_z,
		  min_x, max_y, max_z, color, NULL);

  g3d_drawOneLine(
		  min_x, max_y, max_z,
		  min_x, min_y, max_z, color, NULL);

  g3d_drawOneLine(
		  max_x, min_y, min_z,
		  max_x, min_y, max_z, color, NULL);

  g3d_drawOneLine(
		  max_x, max_y, min_z,
		  max_x, max_y, max_z, color, NULL);

  g3d_drawOneLine(
		  min_x, min_y, min_z,
		  min_x, min_y, max_z, color, NULL);

  g3d_drawOneLine(
		  min_x, max_y, min_z,
		  min_x, max_y, max_z, color, NULL);

  return 1;
}

int show_Object_Oriented_Mightabilities()
{
    int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  int i=0, j, k;
  double radius=grid_around_HRP2.GRID_SET->pace/2.0;
  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
      for(i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
      {
	if(SHOW_OBJECT_MIGHTABILITY_FOR_AGENTS[i]==0)
	  continue;
	
	for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
		{
		if(curr_flags_show_Mightability_Maps.show_visibility[i][j]>0)
		 {
		    if(object_MM.object[nr_ctr].geo_MM.visible[i][j]>0)
		    {
		      //printf(" Drawing disc\n"); 
		      
		      g3d_drawDisc((envPt_MM->robot[nr_ctr]->BB.xmin+envPt_MM->robot[nr_ctr]->BB.xmax)/2.0,(envPt_MM->robot[nr_ctr]->BB.ymin+envPt_MM->robot[nr_ctr]->BB.ymax)/2.0, envPt_MM->robot[nr_ctr]->BB.zmax, radius, curr_flags_show_Mightability_Maps.show_visibility[i][j], NULL);
		    }
		 } 
		}
	      for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		{
		for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
		  {
		  if(curr_flags_show_Mightability_Maps.show_reachability[i][j][k]>0)
		   {
		     if(object_MM.object[nr_ctr].geo_MM.reachable[i][j][k]>0)
		    {
		      //printf(" Drawing disc\n"); 
		      ////g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, curr_flags_show_Mightability_Maps.show_reachability[i][j][k], NULL);
		      g3d_drawDisc((envPt_MM->robot[nr_ctr]->BB.xmin+envPt_MM->robot[nr_ctr]->BB.xmax)/2.0,(envPt_MM->robot[nr_ctr]->BB.ymin+envPt_MM->robot[nr_ctr]->BB.ymax)/2.0, envPt_MM->robot[nr_ctr]->BB.zmax, radius, curr_flags_show_Mightability_Maps.show_visibility[i][j], NULL);
		    }
		   }
		  }
		}
		
	
      }
    }  

		    
}

int show_symbolic_Mightability_Map_Relations()
{
 #ifndef COMMENT_TMP
  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int no = envPt_MM->no;
  int nr = envPt_MM->nr;
  int x,y,z;
  
  int show_visibility=0;
  int show_reachability=0;
  int show_reachability_to_put_inside=0;
  
  int nr_ctr=0;
  for(;nr_ctr<nr;nr_ctr++)
    {
    
      if(show_reachability==1)
	{
	  int for_jido=1;
	  int for_human=0;

	  if(for_human==1)
	    {
	      if(object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand==1||object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand==1)
		{
		  Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Green);
		  ////envPt_MM->robot[i]->BB.xmin
		}
       
      
	      if(object_MM.object[nr_ctr].geo_MM.reachable_by_human_LHand_by_bending==1||object_MM.object[nr_ctr].geo_MM.reachable_by_human_RHand_by_bending==1)
		{
		  Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Blue);
		}  
	      /*
		if(object_MM.object[nr_ctr].geo_MM.reachable_by_LHand_by_turning_around_bending==1||object_MM.object[nr_ctr].geo_MM.reachable_by_RHand_by_turning_around_bending==1)
		{
		Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Red);
		} 
	      */
	    }
  
	  if(for_jido==1)
	    {  
	      if(object_MM.object[nr_ctr].geo_MM.reachable_by_JIDO_Hand==1)
		{
		  Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
		}  
	    }
	}

      if(show_visibility==1)
	{ 
 
	  if(object_MM.object[nr_ctr].geo_MM.visible_by_human==1)
	    {
	      Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Green);
       
	    }
	  /*       
		   if(object_MM.object[nr_ctr].geo_MM.visible_by_human_neck_turn==1)
		   {
		   Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Red);
       
		   }

		   if(object_MM.object[nr_ctr].geo_MM.visible_by_human_straight_head_orientation==1)
		   {
		   Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Blue);
       
		   }
	  */
	  /*
	    if(object_MM.object[nr_ctr].visible_by_HRP2==1)
	    {
	    Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
	    }
       
	    if(object_MM.object[nr_ctr].visible_by_HRP2_straight_head_orientation==1)
	    {
	    Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
	    }

         
	    if(object_MM.object[nr_ctr].visible_by_HRP2_neck_turn==1)
	    {
	    Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
	    }

	  */
	  if(object_MM.object[nr_ctr].geo_MM.visible_by_JIDO==1)
	    {
	      Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
	    }
	  /*       
		   if(object_MM.object[nr_ctr].visible_by_JIDO_straight_head_orientation==1)
		   {
		   Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
		   }

         
		   if(object_MM.object[nr_ctr].visible_by_JIDO_neck_turn==1)
		   {
		   Draw_Bounding_Box(envPt_MM->robot[nr_ctr]->BB.xmin,envPt_MM->robot[nr_ctr]->BB.ymin,envPt_MM->robot[nr_ctr]->BB.zmin,envPt_MM->robot[nr_ctr]->BB.xmax,envPt_MM->robot[nr_ctr]->BB.ymax,envPt_MM->robot[nr_ctr]->BB.zmax, Yellow);
		   }
	  */

	} 

      if(show_reachability_to_put_inside==1)
	{
	  int i=0;
       
	  for(i=0;i<object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.no_points;i++)
	    {
	      g3d_drawDisc(object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[i].x, object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[i].y, object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[i].z, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);

	    }
	  /*
	    for(i=0;i<object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points;i++)
	    {
	    g3d_drawDisc(object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.point[i].x, object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.point[i].y, object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.point[i].z, grid_around_HRP2.GRID_SET->pace/2.0, Green, NULL);

      
	    } */
	}
      /* if(object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_human.no_points>0)
	 {
	 g3d_drawDisc(object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[0].x, object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[0].y, object_MM.object[nr_ctr].sym_MM.reachable_for_putting_inside_by_JIDO.point[0].z, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);

	 } 
      */
    }

#endif
  return 1;
}

int show_obstacle_cells_belonging_to(int object_index)
{
hri_bitmapset * bitmapset=grid_around_HRP2.GRID_SET;


      double shift_for_cell_mid=bitmapset->pace/2.0;
      hri_bitmap * bitmap;
      bitmap = bitmapset->bitmap[HRP2_GIK_MANIP];
      int x,y,z;
      for(x=0; x<bitmap->nx; x++)
	{
	  for(y=0; y<bitmap->ny; y++)
	    {
	      for(z=0; z<bitmap->nz; z++)
		{
                   if(bitmapset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]==1)
		  ////if(bitmapset->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val < 0)  
		    {
		      double tmp_x  = x*bitmapset->pace+bitmapset->realx+shift_for_cell_mid;
		      double tmp_y  = y*bitmapset->pace+bitmapset->realy+shift_for_cell_mid;
		      double tmp_z  = z*bitmapset->pace+bitmapset->realz+shift_for_cell_mid;
		      
			g3d_drawDisc(tmp_x, tmp_y, tmp_z, 0.01, 3, NULL);
		      
		    }
		}
	    }
	}  
      //////////g3d_drawDisc(point_of_curr_collision.x, point_of_curr_collision.y,point_of_curr_collision.z, 0.1,4, NULL);
  
  return 1;
}

int show_obstacle_cells_belonging_to_old(int object_index)
{
#ifndef COMMENT_TMP
  //////////printf(" Showing obstacle cells belonging to index %d\n",object_index);
  int x,y,z;
  for(x=0; x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx; x++)
    {
      for(y=0; y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny; y++)
	{
	  for(z=0; z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz; z++)
	    {
  
  
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]==1)
		{
       
       
		  ////////// printf("cell %d,%d, %d,\n",x,y,z);
 
       
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human==1)
		    {
		      //////////printf("This is first non visible cell for human also \n");          

		      double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		      double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		      double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		      g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/2.0, Blue, NULL);
		    }
		  /*
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation==1)
		    {
		    ////////// printf("This is first_non_visible_by_human_straight_head_orientation also \n");          

		    double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		    double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		    double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		    g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);
		    }
		  */

		  double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		  double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		  double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		  g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/4.0, Green, NULL);
    
		  envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 
		  int nr = envPt_MM->nr;
		  int i=0; 
		  for(i=0;i<nr;i++)
		    {
		      if(i!=object_index&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[i]==1)
			{
			  ////////// printf("This cell also belongs to %s\n",envPt_MM->robot[i]->name);
			}
        
		    }
 
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1)//the cell just above the  horizontal surface of table
		    {
		      //////////printf(" This cell also belongs to horizontal surface\n");
		    }
		}
 
	    }
	}
    }
#endif
  return 1;
}

int make_cells_corresponding_to_object_obstacle_free(char *object_name)
{
  int object_index=get_index_of_robot_by_name ( object_name );
  //////////printf(" Showing obstacle cells belonging to index %d\n",object_index);
  int x,y,z;
  for(x=0; x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx; x++)
    {
      for(y=0; y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny; y++)
	{
	  for(z=0; z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz; z++)
	    {
  
  
	      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[object_index]==1||grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.close_to_objects_indx[object_index]==1)
		{
       
       
		  ////////// printf("cell %d,%d, %d,\n",x,y,z);
 
       
      
		  /*
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.first_non_visible_by_human_straight_head_orientation==1)
		    {
		    ////////// printf("This is first_non_visible_by_human_straight_head_orientation also \n");          

		    double x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
		    double y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
		    double z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
 
    
		    g3d_drawDisc(x_world, y_world, z_world, grid_around_HRP2.GRID_SET->pace/2.0, Red, NULL);
		    }
		  */

		  ////printf(" >>>> no_belongs_to_objects = %d \n", grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects);

		  ////if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface!=1)
		  if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface!=1)
		    {   
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects==1)
			{
			  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val=0;
			  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.no_belongs_to_objects=0;
			}
		      if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects==1)
			{
			  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val=0;
			  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.no_close_to_objects=0;
			}
		    } 
		  /* int nr = envPt_MM->nr;
		     int i=0; 
		     for(i=0;i<nr;i++)
		     {
		     if(i!=object_index&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.belongs_to_objects_indx[i]!=1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface!=1)
		     {
		     ////////// printf("This cell also belongs to %s\n",envPt_MM->robot[i]->name);
		     grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val=0;
		     }
       
        
		     }
		  */
 
		}
 
	    }
	}
    }
  return 1;
}

//////////// For mhp requests
int MM_is_object_visible(HRI_AGENT *agent, p3d_rob *object)
{
  int i=0;
  for(i=0;i<Mightabilities_for_obj.total_no_obj;i++)
    {
      if(strcmp(object->name,Mightabilities_for_obj.object[i].object_name)==0)
	{
#ifdef JIDO_EXISTS_FOR_MA
	  if(agent->type==HRI_JIDO1)
	    {
	      if( Mightabilities_for_obj.object[i].obj_mightability.is_visible_by[JIDO_MA]==1)
		{
		  return 1;
		}
	    }	
#endif
#ifdef HRP2_EXISTS_FOR_MA
	  if(agent->type==HRI_HRP214)
	    {
	      if( Mightabilities_for_obj.object[i].obj_mightability.is_visible_by[HRP2_MA]==1)
		{
		  return 1;
		}
	    }	
#endif
	  if(agent->type==HRI_ACHILE)
	    {
	      if( Mightabilities_for_obj.object[i].obj_mightability.is_visible_by[HUMAN1_MA]==1)
		{
		  return 1;
		}
	    }	
	}
    }
printf(" >>>>> MA ERROR : THE CORRECT TYPE OF THE AGENT NOT SUPPORTED BY MIGHTABILITY ANALYSIS (MA). \n ");
return 0;
}

int MM_is_object_reachable(HRI_AGENT *agent, p3d_rob *object)
{
  int i=0;
  for(i=0;i<Mightabilities_for_obj.total_no_obj;i++)
    {
      if(strcmp(object->name,Mightabilities_for_obj.object[i].object_name)==0)
	{
	  #ifdef JIDO_EXISTS_FOR_MA
	  if(agent->type==HRI_JIDO1)
	    {
	      if( Mightabilities_for_obj.object[i].obj_mightability.is_reachable_by[JIDO_MA]==1)
		{
		  return 1;
		}
	    }	
	  #endif
	  
	  #ifdef HRP2_EXISTS_FOR_MA
	  if(agent->type==HRI_HRP214)
	    {
	      if( Mightabilities_for_obj.object[i].obj_mightability.is_reachable_by[HRP2_MA]==1)
		{
		  return 1;
		}
	    }	
	  #endif
	  
	  if(agent->type==HRI_ACHILE)
	    {
	      if( Mightabilities_for_obj.object[i].obj_mightability.is_reachable_by[HUMAN1_MA]==1)
		{
		  return 1;
		}
	    }	
	}
    }
    
    printf(" >>>>> MA ERROR : THE CORRECT TYPE OF THE AGENT NOT SUPPORTED BY MIGHTABILITY ANALYSIS (MA). \n ");
return 0;
    
}

int find_candidate_points_for_putinto(int by_agent, int container_index)
{
#ifndef COMMENT_TMP
  if(by_agent==3)//For JIDO
    { 
      envPt_MM= (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 
      point_co_ordi center_of_opening;
      center_of_opening.x=(envPt_MM->robot[container_index]->BB.xmin+envPt_MM->robot[container_index]->BB.xmax)/2.0;
      center_of_opening.y=(envPt_MM->robot[container_index]->BB.ymin+envPt_MM->robot[container_index]->BB.ymax)/2.0;
      center_of_opening.z=(envPt_MM->robot[container_index]->BB.zmin+envPt_MM->robot[container_index]->BB.zmax)/2.0;

      double Amplitude=1.0;
      double sig_dist=4.0;

      candidate_points_to_putinto_by_jido.no_points=0;
      int i=0;
      for ( i=0;i<object_MM.object[container_index].sym_MM.reachable_for_putting_inside_by_JIDO.no_points;i++ )
	{
	  candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].x=object_MM.object[container_index].sym_MM.reachable_for_putting_inside_by_JIDO.point[i].x;
	  candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].y=object_MM.object[container_index].sym_MM.reachable_for_putting_inside_by_JIDO.point[i].y;
	  candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].z=object_MM.object[container_index].sym_MM.reachable_for_putting_inside_by_JIDO.point[i].z;

	  //Assigning initial weights based on the distance from the centre of the opening
	  double point_to_cnter_dist=sqrt((candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].x-center_of_opening.x)*(candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].x-center_of_opening.x)+(candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].y-center_of_opening.y)*(candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].y-center_of_opening.y)+(candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].z-center_of_opening.z)*(candidate_points_to_putinto_by_jido.point[candidate_points_to_putinto_by_jido.no_points].z-center_of_opening.z));
  
	  candidate_points_to_putinto_by_jido.weight[candidate_points_to_putinto_by_jido.no_points]=Amplitude*exp(-(((point_to_cnter_dist)*(point_to_cnter_dist)/2.0*sig_dist*sig_dist)));

  
	  candidate_points_to_putinto_by_jido.no_points++;
	}  
    }
#endif
}




int is_object_visible_for_agent(HRI_AGENT * agent, p3d_rob *object, double threshold, int save, int draw_at_end)
{
  GLint viewport[4];
  g3d_states st;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");
  double result;
  
  if(object==NULL || agent==NULL){
    printf("%s: %d: g3d_is_object_visible_from_viewpoint(): input object is NULL.\n",__FILE__,__LINE__);
    return FALSE;
  }  
  //Change the size of the viewport if you want speed
  if(!save){
    glGetIntegerv(GL_VIEWPORT, viewport);
    glViewport(0,0,(GLint)(viewport[2]/2),(GLint)(viewport[3]/2));
  }
  
  g3d_save_win_camera(win->vs);
  g3d_save_state(win, &st);
  
  // only keep what is necessary:
  win->vs.fov            = agent->perspective->fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled =  1;
  win->vs.enableLogo=0;//AKP:Logo red parts are (mis)considered as object parts in the core of visibility calculation
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);
  
  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(agent->perspective->camjoint->abs_pos, win->vs);

  //AKP: Not setting the projection mode for the time being, because then not getting the desired % of visibility in some cases
  g3d_set_projection_matrix(win->vs.projection_mode);

  //AKP
  /////g3d_draw_win(win);
    ////return 0;
    g3d_draw_allwin_active();
#if defined(WITH_XFORM)
    fl_check_forms();
#endif
    //everything is ready now.
    g3d_is_object_visible_from_current_viewpoint(win, object,&result,save,(char*)"");
  
    //restore viewport
    if(!save){
      glViewport(0,0,(GLint)viewport[2],(GLint)viewport[3]);
    }
    g3d_load_state(win, &st);
  
    g3d_restore_win_camera(win->vs);

  
    g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov
  
    if(draw_at_end)
      g3d_draw_win(win);
  
    //AKP
    ////////printf("  >>>>>>> result = %lf, visibility = %lf \n",result, 100.0*result);
    g3d_draw_allwin_active();
#if defined(WITH_XFORM)
    fl_check_forms();
#endif
    if(100.0*result>=threshold)
      return TRUE;
    else
      return FALSE;
  
}
/*
  int show_point_of_screen()
  {
  int j=0;
  for(j=0;j<no_FOV_end_point_vertices;j++)
  {

  int i=4;
  for(i=4;i<8;i++)
  {
  //// if(i<4)
  //// {
  ////g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 3, NULL);
  //// }  
  ////else
  // {
  g3d_drawDisc(FOV_end_point_vertices[j][i].x,FOV_end_point_vertices[j][i].y, FOV_end_point_vertices[j][i].z, grid_around_HRP2.GRID_SET->pace/4.0, 2, NULL);
 
  }
  }
  }
*/
int get_horizontal_triangles(std::list<gpTriangle> &htris)
{
  ChronoOff();
  ChronoOn();

  htris.clear();
  unsigned int i, j, k;
  p3d_index *face_indices= NULL;
  double angle;
  p3d_vector3 normal;
  p3d_vector3 *points= NULL;
  p3d_matrix4 Tsupport;
  p3d_polyhedre *polyh= NULL;
  gpTriangle triangle;
  gpPlacement placement;
  std::list<gpTriangle>::iterator iterT1, iterT2;
  double translationStep=grid_around_HRP2.GRID_SET->pace-0.005;
  ////std::list<gpTriangle>::iterator iter;
  p3d_vector3 *trisamples= NULL;
  unsigned int nb_samples;

  int cell_x;
  int cell_y;
  int cell_z;
  unsigned int tmp_ctr=0;

  int nr = envPt_MM->nr;

  for(k=0;k<nr;k++)
    {
      int is_table=0;
      if (strcasestr(envPt_MM->robot[k]->name,"HUMAN")||strcasestr(envPt_MM->robot[k]->name,"ROBOT")||strcasestr(envPt_MM->robot[k]->name,"VISBALL")||strcasestr(envPt_MM->robot[k]->name,"GHOST"))
	continue;
      if(strcasestr(envPt_MM->robot[k]->name,"TABLE"))
	is_table=1;
      
      for(i=0; i<(unsigned int) envPt_MM->robot[k]->o[0]->np; ++i)
	{
	  p3d_matMultXform(envPt_MM->robot[k]->o[0]->jnt->abs_pos, envPt_MM->robot[k]->o[0]->pol[i]->pos_rel_jnt, Tsupport);

	  polyh= envPt_MM->robot[k]->o[0]->pol[i]->poly;
	  poly_build_planes(polyh);
	  points= polyh->the_points;
	  for(j=0; j<polyh->nb_faces; j++)
	    {
	      p3d_xformVect(Tsupport, polyh->the_faces[j].plane->normale, normal);
	      p3d_vectNormalize(normal, normal);
	      face_indices= polyh->the_faces[j].the_indexs_points;

	      angle= fabs( (180.0/M_PI)*acos(normal[2]) );
	      if( (normal[2]<0) || angle>5 )
		{  continue;  }
	      if(polyh->the_faces[j].nb_points==3)
		{
		  p3d_xformPoint(Tsupport, points[face_indices[0] - 1], triangle.p1);
		  p3d_xformPoint(Tsupport, points[face_indices[1] - 1], triangle.p2);
		  p3d_xformPoint(Tsupport, points[face_indices[2] - 1], triangle.p3);
		  triangle.description= GP_DESCRIPTION_POINTS;
		  htris.push_back(triangle);

		  trisamples= gpSample_triangle_surface(triangle.p1,triangle.p2,triangle.p3, translationStep, &nb_samples);

		  if(trisamples==NULL)
		    {   continue;   
		    }

		  for(int i=0; i<nb_samples; i++)
		    {
		      cell_x=(trisamples[i][0]- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
     
		      if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			{
			  cell_y=(trisamples[i][1]- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
			  if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
			    {
			      cell_z=(trisamples[i][2]+0.05- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
			      if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
				{
				  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;
				  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.horizontal_surface_of=k;//horizontal surface belongs to this robot index
                                  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;
				  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.exact_z_val=trisamples[i][2];
				  
				  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_table=is_table;
				  ////g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
				  ////tmp_ctr++;
				} 
			    }
			}
		      ////g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
		    }

		  free(trisamples);
		  trisamples= NULL; 
		  nb_samples=0;
		}
	      else
		{
		  ////printf("%s: %d: gpFind_placements_on_object(): the faces of \"%s\" should all be triangles.\n", __FILE__,__LINE__,support->name);
		}
	    }
	}
    }
  //printf(" tmp_ctr=%d\n",tmp_ctr);
  //ChronoPrint(" Time for finding triangles");
  return 0;
}

int update_horizontal_triangles(std::list<gpTriangle> &htris)
{
  ChronoOff();
  ChronoOn();

  htris.clear();
  unsigned int i, j, k;
  p3d_index *face_indices= NULL;
  double angle;
  p3d_vector3 normal;
  p3d_vector3 *points= NULL;
  p3d_matrix4 Tsupport;
  p3d_polyhedre *polyh= NULL;
  gpTriangle triangle;
  gpPlacement placement;
  std::list<gpTriangle>::iterator iterT1, iterT2;
  double translationStep=grid_around_HRP2.GRID_SET->pace-0.005;
  ////std::list<gpTriangle>::iterator iter;
  p3d_vector3 *trisamples= NULL;
  unsigned int nb_samples;

  int cell_x;
  int cell_y;
  int cell_z;
  unsigned int tmp_ctr=0;

  int nr = envPt_MM->nr;
  configPt cur_rob_pos;

  for(k=0;k<nr;k++)
    {
      int is_table=0;
      if (strcasestr(envPt_MM->robot[k]->name,"HUMAN")||strcasestr(envPt_MM->robot[k]->name,"ROBOT")||strcasestr(envPt_MM->robot[k]->name,"VISBALL")||strcasestr(envPt_MM->robot[k]->name,"GHOST"))
	{
	  continue;
	}

      if(robots_status_for_Mightability_Maps[k].has_moved==1)
	{
          if (strcasestr(envPt_MM->robot[k]->name,"TABLE"))
	    is_table=1;
	  
	  cur_rob_pos=MY_ALLOC(double,envPt_MM->robot[k]->nb_dof); 
	  p3d_get_robot_config_into(envPt_MM->robot[k],&cur_rob_pos);
	  p3d_set_and_update_this_robot_conf(envPt_MM->robot[k], robots_status_for_Mightability_Maps[k].rob_prev_config);

	  for(i=0; i<(unsigned int) envPt_MM->robot[k]->o[0]->np; ++i)
	    {
	      p3d_matMultXform(envPt_MM->robot[k]->o[0]->jnt->abs_pos, envPt_MM->robot[k]->o[0]->pol[i]->pos_rel_jnt, Tsupport);

	      polyh= envPt_MM->robot[k]->o[0]->pol[i]->poly;
	      poly_build_planes(polyh);
	      points= polyh->the_points;
	      for(j=0; j<polyh->nb_faces; j++)
		{
		  p3d_xformVect(Tsupport, polyh->the_faces[j].plane->normale, normal);
		  p3d_vectNormalize(normal, normal);
		  face_indices= polyh->the_faces[j].the_indexs_points;

		  angle= fabs( (180.0/M_PI)*acos(normal[2]) );
		  if( (normal[2]<0) || angle>5 )
		    {  continue;  }
		  if(polyh->the_faces[j].nb_points==3)
		    {
		      p3d_xformPoint(Tsupport, points[face_indices[0] - 1], triangle.p1);
		      p3d_xformPoint(Tsupport, points[face_indices[1] - 1], triangle.p2);
		      p3d_xformPoint(Tsupport, points[face_indices[2] - 1], triangle.p3);
		      triangle.description= GP_DESCRIPTION_POINTS;
		      htris.push_back(triangle);

		      trisamples= gpSample_triangle_surface(triangle.p1,triangle.p2,triangle.p3, translationStep, &nb_samples);

		      if(trisamples==NULL)
			{   continue;   
			}

		      for(int i=0; i<nb_samples; i++)
			{
			  cell_x=(trisamples[i][0]- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
     
			  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			    {
			      cell_y=(trisamples[i][1]- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
			      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
				{
				  cell_z=(trisamples[i][2]+0.05- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
				  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
				    {
				      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_horizontal_surface=0;
				      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.horizontal_surface_of=-1;//horizontal surface belongs to this robot index
                                      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_table=0;
				      ////g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
				      ////tmp_ctr++;
				    } 
				}
			    }
			  ////g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
			}

		      free(trisamples);
		      trisamples= NULL; 
		      nb_samples=0;
		    }
		  else
		    {
		      ////printf("%s: %d: gpFind_placements_on_object(): the faces of \"%s\" should all be triangles.\n", __FILE__,__LINE__,support->name);
		    }
		}
	    }
	  p3d_set_and_update_this_robot_conf(envPt_MM->robot[k],cur_rob_pos);

 
	  for(i=0; i<(unsigned int) envPt_MM->robot[k]->o[0]->np; ++i)
	    {
	      p3d_matMultXform(envPt_MM->robot[k]->o[0]->jnt->abs_pos, envPt_MM->robot[k]->o[0]->pol[i]->pos_rel_jnt, Tsupport);

	      polyh= envPt_MM->robot[k]->o[0]->pol[i]->poly;
	      poly_build_planes(polyh);
	      points= polyh->the_points;
	      for(j=0; j<polyh->nb_faces; j++)
		{
		  p3d_xformVect(Tsupport, polyh->the_faces[j].plane->normale, normal);
		  p3d_vectNormalize(normal, normal);
		  face_indices= polyh->the_faces[j].the_indexs_points;

		  angle= fabs( (180.0/M_PI)*acos(normal[2]) );
		  if( (normal[2]<0) || angle>5 )
		    {  continue;  }
		  if(polyh->the_faces[j].nb_points==3)
		    {
		      p3d_xformPoint(Tsupport, points[face_indices[0] - 1], triangle.p1);
		      p3d_xformPoint(Tsupport, points[face_indices[1] - 1], triangle.p2);
		      p3d_xformPoint(Tsupport, points[face_indices[2] - 1], triangle.p3);
		      triangle.description= GP_DESCRIPTION_POINTS;
		      htris.push_back(triangle);

		      trisamples= gpSample_triangle_surface(triangle.p1,triangle.p2,triangle.p3, translationStep, &nb_samples);

		      if(trisamples==NULL)
			{   continue;   
			}

		      for(int i=0; i<nb_samples; i++)
			{
			  cell_x=(trisamples[i][0]- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
     
			  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			    {
			      cell_y=(trisamples[i][1]- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
			      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
				{
				  cell_z=(trisamples[i][2]+0.05- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
				  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
				    {
				      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;
				      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.horizontal_surface_of=k;//horizontal surface belongs to this robot index
                                      grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_horizontal_surface=1;
				  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.exact_z_val=trisamples[i][2];
				  
				  grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[cell_x][cell_y][cell_z].Mightability_map_cell_obj_info.is_table=is_table;
				      ////g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
				      ////tmp_ctr++;
				    } 
				}
			    }
			  ////g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
			}

		      free(trisamples);
		      trisamples= NULL; 
		      nb_samples=0;
		    }
		  else
		    {
		      ////printf("%s: %d: gpFind_placements_on_object(): the faces of \"%s\" should all be triangles.\n", __FILE__,__LINE__,support->name);
		    }
		}
	    }
 
	}
    }
  ///printf(" tmp_ctr=%d\n",tmp_ctr);
  //ChronoPrint(" Time for finding triangles");
  return 0;
}


int display_horizontal_triangles(std::list<gpTriangle> htris)
{
  std::list<gpTriangle>::iterator iter;

  for(iter=htris.begin(); iter!=htris.end(); ++iter)
    {
      glBegin(GL_TRIANGLES);
      glVertex3dv(iter->p1);
      glVertex3dv(iter->p2);
      glVertex3dv(iter->p3);
      glEnd();

   
    }


}

int display_horizontal_triangles_samples(std::list<gpTriangle> htris)
{
  double translationStep=grid_around_HRP2.GRID_SET->pace-0.005;
  std::list<gpTriangle>::iterator iter;
  p3d_vector3 *trisamples= NULL;
  unsigned int nb_samples;

  for(iter=htris.begin(); iter!=htris.end(); ++iter)
    {
    

      trisamples= gpSample_triangle_surface_with_edges((*iter).p1,(*iter).p2,(*iter).p3, translationStep, &nb_samples);

      if(trisamples==NULL)
	{   continue;   
	}

      for(int i=0; i<nb_samples; i++)
	{
	  g3d_drawDisc(trisamples[i][0], trisamples[i][1], trisamples[i][2]+0.01, 0.02, 4, NULL);
	}

      free(trisamples);
      trisamples= NULL; 
      nb_samples=0;
    }
}

int get_horizontal_surfaces()
{
  get_horizontal_triangles(global_htris);
}

int update_horizontal_surfaces()
{
  update_horizontal_triangles(global_htris);
}

object_Symbolic_Mightability_Maps_Relation* create_object_oriented_Mightability_obj()
{
  object_Symbolic_Mightability_Maps_Relation *OOM=MY_ALLOC(object_Symbolic_Mightability_Maps_Relation,1);
  ////return OOM;
  ////object_Symbolic_Mightability_Maps_Relation *OOM;
  int nr=envPt_MM->nr;
  
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     for(int j=0;j<nr;j++)
     {
     ////printf(" creating for object %s, for agent %d \n",envPt_MM->robot[j]->name,i);
     OOM->object[j].geo_MM.visible[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     for(int m=0;m<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;m++)
     {
     OOM->object[j].geo_MM.visible[i][m]=-1;
     }
    
     OOM->object[j].geo_MM.reachable[i]=MY_ALLOC(int*, agents_for_MA_obj.for_agent[i].maxi_num_reach_states);
     
     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;k++)
      {
     OOM->object[j].geo_MM.reachable[i][k]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].no_of_arms);
     
       for(int m=0;m<agents_for_MA_obj.for_agent[i].no_of_arms;m++)
       {
       OOM->object[j].geo_MM.reachable[i][k][m]=-1;
       }
     
      }
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     }
   }
   
   return OOM;
}

int copy_current_object_oriented_Mightability_into(object_Symbolic_Mightability_Maps_Relation* target)
{
  
  int nr=envPt_MM->nr;
  
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     for(int j=0;j<nr;j++)
     {
     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;k++)
      {
	
     target->object[j].geo_MM.visible[i][k]=object_MM.object[j].geo_MM.visible[i][k];
      }
     
     
     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;k++)
      {
	for(int l=0;l<agents_for_MA_obj.for_agent[i].no_of_arms;l++)
	{
     target->object[j].geo_MM.reachable[i][k][l]=object_MM.object[j].geo_MM.reachable[i][k][l];
	}
      }
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     }
   }
   
   return 1;
}

int print_object_oriented_Mightability(object_Symbolic_Mightability_Maps_Relation* OOM)
{
  
  int nr=envPt_MM->nr;
  
  
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     printf(" For Agent %d >>>>\n",i);
     for(int j=0;j<nr;j++)
     {
            printf(" the Object %s >>\n",envPt_MM->robot[j]->name);

     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;k++)
      {
	
     if(OOM->object[j].geo_MM.visible[i][k]>0)
       {
	 printf(" is visible from state %d \n",k);
       }
      }
     
     
     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;k++)
      {
	for(int l=0;l<agents_for_MA_obj.for_agent[i].no_of_arms;l++)
	{
     if(OOM->object[j].geo_MM.reachable[i][k][l]>0)
         {
	 printf(" is reachable from state %d by %d hand\n",k,l);
         }
	}
      }
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     }
   }
   
   
   return 1;
}

int delete_object_oriented_Mightability_obj(object_Symbolic_Mightability_Maps_Relation *OOM)
{
  int nr=envPt_MM->nr;
  
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     for(int j=0;j<nr;j++)
     {
     
     MY_FREE(OOM->object[j].geo_MM.visible[i], int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
    
     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;k++)
      {
       MY_FREE(OOM->object[j].geo_MM.reachable[i][k], int, agents_for_MA_obj.for_agent[i].no_of_arms);
      
      }
      
     MY_FREE(OOM->object[j].geo_MM.reachable[i], int*, agents_for_MA_obj.for_agent[i].maxi_num_reach_states);
     
     //////////MY_FREE(OOM,object_Symbolic_Mightability_Maps_Relation,1);
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     }
   }
   return 1;
}

int print_object_oriented_Mightability_for_object(object_Symbolic_Mightability_Maps_Relation* OOM, int obj_index)
{
  
  int nr=envPt_MM->nr;
  
  
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     printf(" For Agent %d >>>>\n",i);
     ////for(int j=0;j<nr;j++)
     //{
            printf(" the Object %s >>\n",envPt_MM->robot[obj_index]->name);

     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;k++)
      {
	
     if(OOM->object[obj_index].geo_MM.visible[i][k]>0)
       {
	 printf(" is visible from state %d \n",k);
       }
      }
     
     
     for(int k=0;k<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;k++)
      {
	for(int l=0;l<agents_for_MA_obj.for_agent[i].no_of_arms;l++)
	{
     if(OOM->object[obj_index].geo_MM.reachable[i][k][l]>0)
         {
	 printf(" is reachable from state %d by %d hand\n",k,l);
         }
	}
      }
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     //}
   }
   
   
   return 1;
}

int print_object_oriented_Mightability_for_object_by_agent(object_Symbolic_Mightability_Maps_Relation* OOM, int obj_index, HRI_TASK_AGENT agent)
{
  
  int nr=envPt_MM->nr;
  
  
   ///for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   ///{
     printf(" For Agent %d >>>>\n",agent);
     ////for(int j=0;j<nr;j++)
     //{
            printf(" the Object %s >>\n",envPt_MM->robot[obj_index]->name);

     for(int k=0;k<agents_for_MA_obj.for_agent[agent].maxi_num_vis_states;k++)
      {
	
     if(OOM->object[obj_index].geo_MM.visible[agent][k]>0)
       {
	 printf(" is visible from state %d and visibility= %d\n",k,OOM->object[obj_index].geo_MM.visible[agent][k]);
       }
      }
     
     
     for(int k=0;k<agents_for_MA_obj.for_agent[agent].maxi_num_reach_states;k++)
      {
	for(int l=0;l<agents_for_MA_obj.for_agent[agent].no_of_arms;l++)
	{
     if(OOM->object[obj_index].geo_MM.reachable[agent][k][l]>0)
         {
	 printf(" is reachable from state %d by %d hand\n",k,l);
         }
	}
      }
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     //}
   ////}
   
   
   return 1;
}

int find_candidate_points_for_current_HRI_task_for_object(HRI_TASK_TYPE curr_task, HRI_TASK_AGENT_ENUM performed_by, HRI_TASK_AGENT_ENUM performed_for, candidate_poins_for_task *resultant_candidate_point, char *object)
{
  int no_expansion_cells=0;
  double ox,oy,oz,orx,ory,orz;
  int obj_index=get_index_of_robot_by_name(object);

  if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
  {
  
  
  p3d_get_freeflyer_pose2(envPt_MM->robot[obj_index], &ox,&oy,&oz,&orx,&ory,&orz);
  p3d_set_freeflyer_pose2(envPt_MM->robot[obj_index], 0,0,10,0,0,0);
  
  //To update the MM assuming the object is not there as a constraint for visibility and reachability
  update_robots_and_objects_status();
  update_horizontal_surfaces();
  update_Mightability_Maps_new();

  double obj_BB_height=envPt_MM->robot[obj_index]->BB.zmax-envPt_MM->robot[obj_index]->BB.zmin;
  double obj_BB_length=envPt_MM->robot[obj_index]->BB.xmax-envPt_MM->robot[obj_index]->BB.xmin;
  double obj_BB_width=envPt_MM->robot[obj_index]->BB.ymax-envPt_MM->robot[obj_index]->BB.ymin;
  p3d_set_freeflyer_pose2(envPt_MM->robot[obj_index], ox,oy,oz,orx,ory,orz);
  double max_dim=obj_BB_height;
  
  if(max_dim<obj_BB_length)
    max_dim=obj_BB_length;
  
  if(max_dim<obj_BB_width)
    max_dim=obj_BB_width;
  
   printf(" max_dim=%lf\n",max_dim);
   no_expansion_cells=(int) (max_dim/grid_around_HRP2.GRID_SET->pace);
  
  
  printf(" no_expansion_cells=%d\n",no_expansion_cells);
  }
  
  printf(" >>> Inside find_candidate_points_for_current_HRI_task_for_object() with task %d \n",curr_task);
  HRI_TASK_AGENT state_for_agent=performed_by;
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_vis_states);
   printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_vis_states);
  
  state_for_agent=performed_for;
  
 printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_accepted_vis_states);
   printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_reach_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_reach_states);
  printf(" accepted_states_for_HRI_task[%d][%d][%d][%d].no_non_accepted_vis_states=%d\n",performed_by, performed_for,state_for_agent,curr_task,accepted_states_for_HRI_task[performed_by][performed_for][state_for_agent][curr_task].no_non_accepted_vis_states);
  
  resultant_candidate_point->no_points=0;
   int need_placement_on_plane=0;
   if(curr_task==MAKE_OBJECT_ACCESSIBLE||curr_task==HIDE_AWAY_OBJECT||curr_task==HIDE_OBJECT)
   {
   need_placement_on_plane=1;  
   }
   
   
   int i,j,k;
   int x=0;
   int y=0;
   int z=0;
   int cell_OK_vis=0;
   int cell_OK_reach=0;
   double cell_x_world;
   double cell_y_world;
   double cell_z_world;
     HRI_TASK_AGENT test_for_agent;
   
  for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
    {
      y=0;
      for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
	{
	  z=0;
	  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
	    {
             if(need_placement_on_plane==1&& grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface!=1)
	     {
	      
	       continue;
	     }
	     
	     test_for_agent=performed_by;
	     
	     cell_OK_reach=0;
	     
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    /////if((need_placement_on_plane==1&& grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)||(need_placement_on_plane==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1))
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=1;
         		   break;
 
		    }
		   }
		    if(cell_OK_reach==1)
			   break;
	       }
		   
		 
	       
		if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states==0)
		 cell_OK_reach=1;
		
		if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
		{
		//In case of placement on plane test the reachability of the performing agent above the plane
		if(cell_OK_reach==0&&need_placement_on_plane==1)
		{
		  for(int z_ctr=z;z_ctr<z+no_expansion_cells;z_ctr++)
		        {
			  if(z_ctr<0||z_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			  continue;
			   
			 for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		         {
		         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z_ctr].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)
		          {
		           cell_OK_reach=1;
			   
         		   break;
			  }
		         }
		         if(cell_OK_reach==1)
			   break;
		  
		       }
		}
		}
		/*
		 if(cell_OK_reach==0)
		    {
		      for(int x_ctr=x-no_expansion_cells;x_ctr<x+no_expansion_cells;x_ctr++)
		      {
			  if(x_ctr<0||x_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			  continue;
			
		       for(int y_ctr=y-no_expansion_cells;y_ctr<y+no_expansion_cells;y_ctr++)
		       {
			  if(y_ctr<0||y_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
			  continue;
			
			for(int z_ctr=z-no_expansion_cells;z_ctr<z+no_expansion_cells;z_ctr++)
		        {
			  if(z_ctr<0||z_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			  continue;
			   
			 for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		         {
		         if((need_placement_on_plane==1&& grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x_ctr][y_ctr][z_ctr].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)||(need_placement_on_plane==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x_ctr][y_ctr][z_ctr].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1))
		          {
		           cell_OK_reach=1;
			   
         		   break;
			  }
		         }
		         if(cell_OK_reach==1)
			   break;
		        }
		        if(cell_OK_reach==1)
			   break;
		       }
		       if(cell_OK_reach==1)
			   break;
		      }
		    }
		    */
		
		if(cell_OK_reach==0)
		  continue;
		
		//The place where the agent will put the object should be visible in any case so there will be no expansion based on visibility of the performing agent
		  cell_OK_vis=0;
	       for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   /////if((need_placement_on_plane==1&& grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)||(need_placement_on_plane==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1))
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=1;
		   break;
		   }
	       }
		    
	       
		 ////}
		
	       /*
	       if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states==0)
		 cell_OK_vis=1;

	          if(cell_OK_vis==0)
		    {
		      for(int x_ctr=x-no_expansion_cells;x_ctr<x+no_expansion_cells;x_ctr++)
		      {
			  if(x_ctr<0||x_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			  continue;
			
		       for(int y_ctr=y-no_expansion_cells;y_ctr<y+no_expansion_cells;y_ctr++)
		       {
			  if(y_ctr<0||y_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
			  continue;
			
			for(int z_ctr=z-no_expansion_cells;z_ctr<z+no_expansion_cells;z_ctr++)
		        {
			  if(z_ctr<0||z_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			  continue;
			   
			    if((need_placement_on_plane==1&& grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.is_horizontal_surface==1&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x_ctr][y_ctr][z_ctr].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)||(need_placement_on_plane==0&&grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x_ctr][y_ctr][z_ctr].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1))
			   {
			      cell_OK_vis=1;
		              break;
			   }
		        
		        }
		        if(cell_OK_vis==1)
			   break;
		       }
		       if(cell_OK_vis==1)
			   break;
		      }
		    }
		    */
	       
	       if(cell_OK_vis==0)
		 continue;
	       
	       		
	       //Now testing for the target agent
		 test_for_agent=performed_for;
		 cell_OK_reach=0;
		 
	    
	      for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=1;
         		   break;
 
		    }
		   }
		   if(cell_OK_reach==1)
         		   break;
	       }
		   
		 
	       
		if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_reach_states==0)
		 cell_OK_reach=1;
		
		if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
		{
		 if(cell_OK_reach==0)
		    {
		      for(int x_ctr=x-no_expansion_cells;x_ctr<x+no_expansion_cells;x_ctr++)
		      {
			  if(x_ctr<0||x_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			  continue;
			
		       for(int y_ctr=y-no_expansion_cells;y_ctr<y+no_expansion_cells;y_ctr++)
		       {
			  if(y_ctr<0||y_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
			  continue;
			
			for(int z_ctr=z-no_expansion_cells;z_ctr<z+no_expansion_cells;z_ctr++)
		        {
			  if(z_ctr<0||z_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			  continue;
			   
			 for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		         {
		         if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x_ctr][y_ctr][z_ctr].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_reach[i]][j1]==1)
		          {
		           cell_OK_reach=1;
			   
         		   break;
			  }
		         }
		         if(cell_OK_reach==1)
			   break;
		        }
		        if(cell_OK_reach==1)
			   break;
		       }
		       if(cell_OK_reach==1)
			   break;
		      }
		    }
		}
		
		if(cell_OK_reach==0)
		  continue;
		
		  cell_OK_vis=0;
	        for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=1;
		   break;
		   }
	       }
		    
	       
		 ////}
		
	       
	       if(accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_accepted_vis_states==0)
		 cell_OK_vis=1;

	       if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
	       {
	          if(cell_OK_vis==0)
		    {
		      for(int x_ctr=x-no_expansion_cells;x_ctr<x+no_expansion_cells;x_ctr++)
		      {
			  if(x_ctr<0||x_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
			  continue;
			
		       for(int y_ctr=y-no_expansion_cells;y_ctr<y+no_expansion_cells;y_ctr++)
		       {
			  if(y_ctr<0||y_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
			  continue;
			
			for(int z_ctr=z-no_expansion_cells;z_ctr<z+no_expansion_cells;z_ctr++)
		        {
			  if(z_ctr<0||z_ctr>=grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
			  continue;
			   
			    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x_ctr][y_ctr][z_ctr].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].accepted_visibility[i]]==1)
			   {
			      cell_OK_vis=1;
		              break;
			   }
		        
		        }
		        if(cell_OK_vis==1)
			   break;
		       }
		       if(cell_OK_vis==1)
			   break;
		      }
		    }
	       }
	       if(cell_OK_vis==0)
		 continue;
	       
	       /////// Now check for non acceptable Mightability
		 test_for_agent=performed_by;
	     
	     
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=0;
         		   break;
 
		    }
		   }
		 ////}
		 
	       }
	       
		if(cell_OK_reach==0)
		  continue;
		
		for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=0;
		     
		   break;
		   }
	       }
		 ////}
		
	       if(cell_OK_vis==0)
		 continue;
	       
	       //Now testing for the target agent
		 test_for_agent=performed_for;
		 
	     for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_reach_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   for(int j1=0;j1<agents_for_MA_obj.for_agent[test_for_agent].no_of_arms;j1++)
		   {
		    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.reachable[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_reach[i]][j1]==1)
		    {
		     cell_OK_reach=0;
         		   break;
 
		    }
		   }
		 ////}
		 
	       }
	       
		
		if(cell_OK_reach==0)
		  continue;
		
		////static int non_accept_vis_filter_ctr=0;
	       for( i=0; i<accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].no_non_accepted_vis_states;i++)
	       {
		 ////for(int i1=0;i1<agents_for_MA_obj.for_agent[test_for_agent].maxi_num_reach_states;i1++)
		 ////{
		   ////printf(" testing for cell %d, %d, %d for non visibility\n",x,y,z);
		   if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_Map.visible[test_for_agent][accepted_states_for_HRI_task[performed_by][performed_for][test_for_agent][curr_task].non_accepted_visibility[i]]==1)
		   {
		     cell_OK_vis=0;
		     ////printf(" Test fail\n");
		     ////non_accept_vis_filter_ctr++;
		     break;
		   }
	       }
		////printf(" >>>>> non_accept_vis_filter_ctr=%d\n",non_accept_vis_filter_ctr);
	       if(cell_OK_vis==0)
		 continue;
	       
	       
	       //Now the current cell satisfies all the constraints 
	      cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
	      cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
	      cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
     
	      resultant_candidate_point->point[resultant_candidate_point->no_points].x=cell_x_world;
	      resultant_candidate_point->point[resultant_candidate_point->no_points].y=cell_y_world;	  
	      resultant_candidate_point->point[resultant_candidate_point->no_points].z=cell_z_world;
	      
	      resultant_candidate_point->weight[resultant_candidate_point->no_points]=0;
              resultant_candidate_point->horizontal_surface_of[resultant_candidate_point->no_points]=	grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].Mightability_map_cell_obj_info.horizontal_surface_of;
	      
	      resultant_candidate_point->no_points++;
	      
	      ////if(resultant_candidate_point->no_points>1000)
		////return 1;
	      
	    }
	}
    }


  if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
  {
  ////int obj_index=get_index_of_robot_by_name(object);
  
  p3d_set_freeflyer_pose2(envPt_MM->robot[obj_index], ox,oy,oz,orx,ory,orz);
  
  //To reupdate the MM with object at original position 
  update_robots_and_objects_status();
  update_horizontal_surfaces();
  update_Mightability_Maps_new();

  
  }

    printf("resultant_candidate_point->no_points=%d\n",resultant_candidate_point->no_points);
    return 1;
}

int test_inside(p3d_rob *container, p3d_rob *object)
{
  if(container==NULL||object==NULL)
  {
   printf(" NULL argument\n");
   return 0;
  }
  bool inside;
  double distance;
  int i, j, k, nbPoints, result;
  p3d_vector3 p;
  p3d_vector3 *points= NULL;
  p3d_matrix4 T;
  p3d_polyhedre *polyContainer= NULL, *poly= NULL;
  std::vector<double> point(3);
  gpConvexHull3D chull;

  polyContainer= container->o[0]->pol[0]->poly;
  poly= object->o[0]->pol[0]->poly;

  nbPoints= 0;
  for(i=0; i <container->o[0]->np; ++i)
  {  nbPoints+= container->o[0]->pol[i]->poly->nb_points; }
  points= (p3d_vector3 *) malloc(nbPoints*sizeof(p3d_vector3));

  k= 0;
  for(i=0; i <container->o[0]->np; ++i)
  {  
    p3d_matMultXform(container->o[0]->jnt->abs_pos, container->o[0]->pol[i]->pos_rel_jnt, T);

    for(j=0; j<container->o[0]->pol[i]->poly->nb_points; ++j)
    {
      p3d_vectCopy(container->o[0]->pol[i]->poly->the_points[j], p);
      p3d_xformPoint(T, p, points[k]);
      k++;
    }
  }


  chull.setPoints(points, nbPoints);
  chull.compute(true, 0.0, false);
//   chull.draw();

  p3d_get_freeflyer_pose(object, T);
  result= 1;

  for(i=0; i <object->o[0]->np; ++i)
  {  
    p3d_matMultXform(object->o[0]->jnt->abs_pos, object->o[0]->pol[i]->pos_rel_jnt, T);

    for(j=0; j<object->o[0]->pol[i]->poly->nb_points; ++j)
    {
      p3d_xformPoint(T, object->o[0]->pol[i]->poly->the_points[j], p);

      point[0]= p[0];
      point[1]= p[1];
      point[2]= p[2];
      chull.isPointInside(point, inside, distance);
      if(!inside)
      {
        printf(" NOT \n"); 
        result= 0;
        i= object->o[0]->np;
        break;  
      }
    }
  }


  free(points);
  if(result==1) printf(" YES \n");
  return result;
}



int voronoi(p3d_rob *container, p3d_rob *object)
{
  if(container==NULL||object==NULL)
  {
   printf(" NULL argument\n");
   return 0;
  }
  bool inside;
  double distance;
  int i, j, k, nbPoints, result;
  p3d_vector3 p;
  p3d_vector3 *points= NULL;
  p3d_matrix4 T;
  p3d_polyhedre *polyContainer= NULL, *poly= NULL;
  std::vector<double> point(3);
  gpConvexHull3D chull;

  polyContainer= container->o[0]->pol[0]->poly;
  poly= object->o[0]->pol[0]->poly;

  nbPoints= 0;
  for(i=0; i <container->o[0]->np; ++i)
  {  nbPoints+= container->o[0]->pol[i]->poly->nb_points; }
  points= (p3d_vector3 *) malloc(nbPoints*sizeof(p3d_vector3));

  k= 0;
  for(i=0; i <container->o[0]->np; ++i)
  {  
    p3d_matMultXform(container->o[0]->jnt->abs_pos, container->o[0]->pol[i]->pos_rel_jnt, T);

    for(j=0; j<container->o[0]->pol[i]->poly->nb_points; ++j)
    {
      p3d_vectCopy(container->o[0]->pol[i]->poly->the_points[j], p);
      p3d_xformPoint(T, p, points[k]);
      k++;
    }
  }


  chull.setPoints(points, nbPoints);
//   chull.compute(true, 0.0, false);
 chull.voronoi(false);
 // chull.draw();



  free(points);
  if(result==1) printf(" YES \n");
  return result;
}

