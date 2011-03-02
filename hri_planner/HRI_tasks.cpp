//
// C++ Implementation: HRI_tasks
//
// Description: 
//
//
// Author: Amit Kumar Pandey <akpandey@verdier.laas.fr>, (C) 2010
//
// Copyright: See COPYING file that comes with this distribution
//
/*
 cd /home/akpandey/AKP_modules/BioMove3D_New3/build/;
 /home/akpandey/AKP_modules/BioMove3D_New3/build/Debug/bin/i386-linux/move3d-studio -f /home/akpandey/AKP_modules/mhp_new/BioMove3DDemos_new3/GS/gsJidoKukaSAHandSM_MA_new.p3d -sc /home/akpandey/AKP_modules/mhp_new/BioMove3DDemos_new3/GS/SCENARIO/ManipulationTestSAHand_MA_new4.sce -dmax 0.01
*/

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
#include "lightPlanner/proto/lightPlannerApi.h"

#include "ManipulationPlanner.hpp"
#include "ManipulationUtils.hpp"
#include "robotPos.h"
#include <list>
#include <string>
#include <iostream>

HRI_TASK_TYPE CURRENT_HRI_MANIPULATION_TASK;
//candidate_poins_for_task candidate_points_to_put;
//candidate_poins_for_task candidate_points_to_show;
//candidate_poins_for_task candidate_points_to_hide;
//candidate_poins_for_task candidate_points_to_hide_away;
//candidate_poins_for_task candidate_points_to_give;
//candidate_poins_for_task candidate_points_to_putinto_by_jido;
//candidate_poins_for_task candidate_points_to_putinto_blue_trashbin_by_jido;
//candidate_poins_for_task candidate_points_to_putinto_pink_trashbin_by_jido;
//candidate_poins_for_task current_candidate_points_to_putinto;
//candidate_poins_for_task candidate_points_to_put_away;
//candidate_poins_for_task candidate_points_to_displace_obj;

candidate_poins_for_task *CANDIDATE_POINTS_FOR_CURRENT_TASK;
candidate_poins_for_task resultant_current_candidate_point;

extern int UPDATE_MIGHTABILITY_MAP_INFO;
extern int SHOW_MIGHTABILITY_MAP_INFO;
extern char CURRENT_OBJECT_TO_MANIPULATE[50];

extern p3d_env *envPt_MM;
////extern HRI_AGENT * primary_human_MM;
////extern HRI_AGENT * jido_robot_MM;
////extern HRI_AGENT * hrp2_robot_MM;
extern struct robots_indices rob_indx;
static ManipulationPlanner *manipulation= NULL;

extern candidate_poins_for_task resultant_current_candidate_point;
extern int indices_of_MA_agents[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern HRI_TASK_AGENT CURRENT_TASK_PERFORMED_BY;
extern HRI_TASK_AGENT CURRENT_TASK_PERFORMED_FOR;
extern int CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS;
extern HRI_AGENT *HRI_AGENTS_FOR_MA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern double mini_visibility_threshold_for_task[MAXI_NUM_OF_HRI_TASKS];
extern double maxi_visibility_threshold_for_task[MAXI_NUM_OF_HRI_TASKS];

////action_performance_node curret_task;
std::vector<HRI_task_node> HRI_task_list;
std::map<int,std::string > HRI_task_NAME_ID_map;
std::map<int,std::string > HRI_sub_task_NAME_ID_map;
std::map<std::string, int > HRI_task_plan_DESC_ID_map;
world_state_configs final_configs_after_a_task;

int CURRENT_HRI_TASK_PLAN_ID_TO_SHOW;
int INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW;
int SHOW_HRI_TASK_TRAJ_TYPE;
int SHOW_HRI_PLAN_TYPE;

//TODO : Put in HRI_tasks_Proto.h
int get_placements_in_3D(char *obj_to_manipulate,  std::list<gpPlacement> &placementListOut);
int get_ranking_based_on_view_point(p3d_matrix4 view_frame,point_co_ordi point,p3d_rob *object, p3d_rob *human, std::list<gpPlacement> &placementList);
int get_placements_at_position(p3d_rob *object, point_co_ordi point, std::list<gpPlacement> placementList, int no_rot, std::list<gpPlacement> &placementListOut);
int copy_HRI_task_candidate_points(candidate_poins_for_task *from_candidate_points, candidate_poins_for_task *to_candidate_points);

void fct_draw_loop();

int set_current_HRI_manipulation_task(int arg)
{
 switch(arg)
 {
 case 0:
 CURRENT_HRI_MANIPULATION_TASK=MAKE_OBJECT_ACCESSIBLE;
 break;
 case 1:
 CURRENT_HRI_MANIPULATION_TASK=SHOW_OBJECT;
 break;
 case 2:
 CURRENT_HRI_MANIPULATION_TASK=GIVE_OBJECT;
 break;
 case 3:
 CURRENT_HRI_MANIPULATION_TASK=HIDE_OBJECT;
 break;
 case 4:
 CURRENT_HRI_MANIPULATION_TASK=PUT_AWAY_OBJECT;
 break;
 case 5:
 CURRENT_HRI_MANIPULATION_TASK=HIDE_AWAY_OBJECT;
 break;
 case 6:
 CURRENT_HRI_MANIPULATION_TASK=MAKE_SPACE_FREE_OF_OBJECT_OBJ;
 break;
 case 7:
 CURRENT_HRI_MANIPULATION_TASK=PUT_INTO_OBJECT;
 break;
 }
}

p3d_matrix4 WRIST_FRAME;

static void initManipulation() {
  if (manipulation == NULL) {
	p3d_rob * robotPt= p3d_get_robot_by_name("JIDOKUKA_ROBOT");//justin//JIDOKUKA_ROBOT
	manipulation= new ManipulationPlanner(robotPt);
//         manipulation->setArmType(GP_LWR); // set the arm type
  }
  return;
}

int init_manipulation_planner()
{
if ( manipulation== NULL )
   {
      p3d_rob * robotPt= p3d_get_robot_by_name("JIDOKUKA_ROBOT");//justin//JIDOKUKA_ROBOT
      manipulation= new ManipulationPlanner(robotPt);
//         manipulation->setArmType(GP_LWR); // set the arm type
   }
return 1;
}



int get_grasp_list_for_object(char *obj_to_manipulate, std::list<gpGrasp> &graspList)
{
gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
 gpGet_grasp_list ( obj_to_manipulate, armHandProp.type, graspList );
 
 if(graspList.size()>=1)
return 1;
 else
return 0;
}

int reduce_grasp_list_for_hand_over_task(std::list<gpGrasp> &orig_graspList, char* obj_to_manipulate, char* from_hand, char* to_hand, std::list<gpGrasp> &res_graspList )
{
 p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
// std::list<gpGrasp> graspList2;
// graspList2= graspList;
 gpGrasp_handover_filter(p3d_get_robot_by_name ( from_hand ), p3d_get_robot_by_name ( to_hand ), object, res_graspList, orig_graspList);
  printf(" After gpGrasp_handover_filter()\n");
  printf(" orig_graspList.size()=%d,res_graspList.size()=%d\n",orig_graspList.size(),res_graspList.size());
  
  return 1;
}

int show_current_task_candidate_points(int show_weight_by_color, int show_weight_by_length, candidate_poins_for_task *candidate_points)
{
////////printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE);
/*
 switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 case MAKE_OBJECT_ACCESSIBLE:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_put;
  
  ////show_weighted_candidate_points_to_put_obj(show_weight_by_color);
 break;
 case SHOW_OBJECT:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_show;
  ////show_weighted_candidate_points_to_show_obj(show_weight_by_color);
 break;
 case GIVE_OBJECT:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_give;
  ////show_weighted_candidate_points_to_give_obj(show_weight_by_color);
 break;
 case HIDE_OBJECT:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_hide;
  ////show_weighted_candidate_points_to_hide_obj();
 break;
 }
*/
printf("candidate_points->no_points=%d\n",candidate_points->no_points);
CANDIDATE_POINTS_FOR_CURRENT_TASK=candidate_points;
show_candidate_points_for_current_task(show_weight_by_color,show_weight_by_length);
 
}

int find_HRI_task_candidate_points(HRI_TASK_TYPE CURR_TASK, char *obj_to_manipulate, HRI_TASK_AGENT performed_by,  HRI_TASK_AGENT performed_for, candidate_poins_for_task *curr_resultant_candidate_points)
{
  int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
    
      ////find_candidate_points_for_current_HRI_task(CURR_TASK,  for_agent, JIDO_MA,curr_resultant_candidate_points);
   ////if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==0)
   /////find_candidate_points_for_current_HRI_task(CURR_TASK,  performed_by, performed_for,curr_resultant_candidate_points);
   
   /////if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
   init_visibility_acceptance_for_tasks();
 
   find_candidate_points_for_current_HRI_task_for_object(CURR_TASK,  performed_by, performed_for,curr_resultant_candidate_points, CURRENT_OBJECT_TO_MANIPULATE);
   
   switch (CURR_TASK)
   {
     case MAKE_OBJECT_ACCESSIBLE:
        assign_weights_on_candidte_points_to_put_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
     break;
     
     case SHOW_OBJECT:
	assign_weights_on_candidte_points_to_show_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
      break;
      
     case GIVE_OBJECT:
       assign_weights_on_candidte_points_to_give_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
      break;
      
      case HIDE_OBJECT:
       assign_weights_on_candidte_points_to_hide_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
      break;
      
   }
  
   
   reverse_sort_HRI_task_weighted_candidate_points(curr_resultant_candidate_points);
   CANDIDATE_POINTS_FOR_TASK_FOUND=1;
   
   ////CANDIDATE_POINTS_FOR_CURRENT_TASK=curr_resultant_candidate_points;
   copy_HRI_task_candidate_points(curr_resultant_candidate_points,&resultant_current_candidate_point);
   CANDIDATE_POINTS_FOR_CURRENT_TASK=&resultant_current_candidate_point;
   ////MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);
}
/*
int store_initial_world_state_for_action(action_performance_node curret_task, std::list<action_performance_node> &task_list)
{
    action_performance_node tmp_node;
 printf(" Inside store_current_world_state()\n");

 strcpy(tmp_node.action.action_name,curret_task.action_name);
 strcpy(tmp_node.action.active_agent_name,curret_task.active_agent_name);
 strcpy(tmp_node.action.object_name,curret_task.object_name);
 strcpy(tmp_node.action.target_agent_name,curret_task.target_agent_name);

    int nr = envPt_MM->nr;
    int nr_ctr=0;
    for(;nr_ctr<nr;nr_ctr++)
     {
       tmp_node.WS_before_exec.robot_config[nr_ctr]=MY_ALLOC(double,envPt_MM->robot[nr_ctr]->nb_dof); 
       p3d_get_robot_config_into(envPt_MM->robot[nr_ctr],&tmp_node.WS_before_exec.robot_config[nr_ctr]);
     }   
   
   
}
*/

int find_current_HRI_manip_task_solution(HRI_task_desc curr_task, traj_for_HRI_task &res_traj)
{

CURRENT_HRI_MANIPULATION_TASK=curr_task.task_type;
CURRENT_TASK_PERFORMED_BY=curr_task.by_agent;
CURRENT_TASK_PERFORMED_FOR=curr_task.for_agent;
strcpy(CURRENT_OBJECT_TO_MANIPULATE,curr_task.for_object.c_str());

printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s, TASK_PERFORMED_BY=%d, TASK_PERFORMED_FOR=%d\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE, CURRENT_TASK_PERFORMED_BY, CURRENT_TASK_PERFORMED_FOR);


switch(CURRENT_TASK_PERFORMED_BY)
 {
 case JIDO_MA:
  init_manipulation_planner();
 break;

 }

candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
std::list<gpGrasp> grasps_for_object;
std::list<gpGrasp> candidate_grasps_for_task;
std::list<gpPlacement> curr_placementList;


find_HRI_task_candidate_points(CURRENT_HRI_MANIPULATION_TASK,CURRENT_OBJECT_TO_MANIPULATE,CURRENT_TASK_PERFORMED_BY,CURRENT_TASK_PERFORMED_FOR,curr_resultant_candidate_points);

get_grasp_list_for_object(CURRENT_OBJECT_TO_MANIPULATE, grasps_for_object);
candidate_grasps_for_task=grasps_for_object;



switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 case GIVE_OBJECT:
  {
 
  char* from_hand="SAHandRight";
  char* to_hand="SAHandRight2";

  reduce_grasp_list_for_hand_over_task(grasps_for_object, CURRENT_OBJECT_TO_MANIPULATE, from_hand, to_hand, candidate_grasps_for_task );

   get_placements_in_3D ( CURRENT_OBJECT_TO_MANIPULATE,  curr_placementList );
  }
 break;
 
 case SHOW_OBJECT:
 {
  get_placements_in_3D ( CURRENT_OBJECT_TO_MANIPULATE,  curr_placementList );
 } 
 break;

 case MAKE_OBJECT_ACCESSIBLE:
  {
  p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) CURRENT_OBJECT_TO_MANIPULATE );
  gpCompute_stable_placements (object, curr_placementList ); //this will give just based on the object and infinite plane the different ways an object can be put onto the place with different faces touching the plane. It will not give different orientation of the object along the vertical axis

  }
 break;
 
case HIDE_OBJECT:
  {
  p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) CURRENT_OBJECT_TO_MANIPULATE );
  gpCompute_stable_placements (object, curr_placementList ); //this will give just based on the object and infinite plane the different ways an object can be put onto the place with different faces touching the plane. It will not give different orientation of the object along the vertical axis

  ////remove_hide_places_based_due_to_object(( char* ) CURRENT_OBJECT_TO_MANIPULATE);
  }
 break;
 
 }

 res_traj.task_type=CURRENT_HRI_MANIPULATION_TASK;

 int validate_task_result=JIDO_perform_task ( CURRENT_OBJECT_TO_MANIPULATE, CURRENT_HRI_MANIPULATION_TASK, CURRENT_TASK_PERFORMED_BY, CURRENT_TASK_PERFORMED_FOR, curr_resultant_candidate_points, candidate_grasps_for_task, curr_placementList,  res_traj);



////JIDO_find_HRI_task_solution(CURRENT_HRI_MANIPULATION_TASK, HUMAN1_MA, CURRENT_OBJECT_TO_MANIPULATE);

/*
 switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 #ifdef HRI_JIDO 
 case MAKE_OBJECT_ACCESSIBLE:
 JIDO_make_obj_accessible_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case SHOW_OBJECT:
 JIDO_show_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case GIVE_OBJECT:
 JIDO_give_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case HIDE_OBJECT:
 JIDO_hide_obj_from_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 #endif
  #ifdef HRI_HRP2 
 case MAKE_OBJECT_ACCESSIBLE:
 HRP2_make_obj_accessible_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case SHOW_OBJECT:
 ////printf(">>> CURRENT_HRI_MANIPULATION_TASK=%d\n",CURRENT_HRI_MANIPULATION_TASK);
 HRP2_show_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case GIVE_OBJECT:
 HRP2_give_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case HIDE_OBJECT:
 HRP2_hide_obj_from_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 #endif
 }
*/

   MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);

  UPDATE_MIGHTABILITY_MAP_INFO=1;
  SHOW_MIGHTABILITY_MAP_INFO=1;   
  
  
 return validate_task_result;
}


#ifndef COMMENT_TMP
int JIDO_make_obj_accessible_to_humanOld ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_put_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_put_obj();

 printf ( " <<<<<< candidate_points_to_put.no_points = %d >>>>>>>>\n", candidate_points_to_put.no_points );
 if ( candidate_points_to_put.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
std::vector <p3d_traj*> trajs;
ManipulationData configs(manipulation->robot());
p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
int armID= 0;
p3d_matrix4 T0, T, tAtt;
gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
std::vector <double>  m_objStart, m_objGoto;
status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
			
  if(status==MANIPULATION_TASK_OK)
    //grasp= manipulation->getCurrentGrasp();
    grasp= *( manipulation->getManipulationData().getGrasp() );

  else { 
    printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
    return 1;
  }

  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];

  p3d_mat4Mult(grasp.frame, armHandProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

p3d_get_freeflyer_pose(object, T0);
point_co_ordi goal_pos, point_to_put;

int i1=0;
result= 0;
for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
 {
printf("checking for place %d \n",i1);

	goal_pos.x=candidate_points_to_put.point[i1].x;
	goal_pos.y=candidate_points_to_put.point[i1].y;
	goal_pos.z=candidate_points_to_put.point[i1].z+0.02;

	point_to_put.x=goal_pos.x;
	point_to_put.y=goal_pos.y;
	point_to_put.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_put.x;
	obj_tmp_pos[7]=point_to_put.y;
	obj_tmp_pos[8]=point_to_put.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_put.x;
	T[1][3]=point_to_put.y;
	T[2][3]=point_to_put.z;

        p3d_set_freeflyer_pose(object, T);
        g3d_draw_allwin_active();
        for(int i=0; i<50; ++i)
        {
          q = setRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
          if(q!=NULL) break;
        }
	if(q==NULL) 
        {
        printf("q==NULL after setRobotGraspPosWithoutBase()\n"); 
        continue;
        }
        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,manipulation->robotStart(), q,  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", confs, smTrajs);

        printf(" After armPlanTask, result = %d \n",status);
	p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );

        if(status==MANIPULATION_TASK_OK)
        { 
           result= 1;
           break;
        }
//         else
//         { manipulation->cleanRoadmap(); }
 }//end for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
 
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}



int JIDO_make_obj_accessible_to_human_old_new ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_put_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_put_obj();

 printf ( " <<<<<< candidate_points_to_put.no_points = %d >>>>>>>>\n", candidate_points_to_put.no_points );
 if ( candidate_points_to_put.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_put;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     p3d_set_freeflyer_pose(object, T0);
    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
//     status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);



    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);

    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_put.point[i1].x;
	goal_pos.y=candidate_points_to_put.point[i1].y;
	goal_pos.z=candidate_points_to_put.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_put.x=goal_pos.x;
	point_to_put.y=goal_pos.y;
	point_to_put.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_put.x;
	obj_tmp_pos[7]=point_to_put.y;
	obj_tmp_pos[8]=point_to_put.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_put.x;
	T[1][3]=point_to_put.y;
	T[2][3]=point_to_put.z;

        p3d_set_freeflyer_pose2(object, point_to_put.x, point_to_put.y, point_to_put.z, rx, ry, rz);
        
       double visibility_threshold=95.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==0)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);



        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);
        for(int i=0; i<1; ++i)
        {
          //q = setRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
         // message= manipulation->findArmGraspsConfigs(armID, object, grasp, configs);
          //if(q!=NULL) break;
        }
	if(message!=MANIPULATION_TASK_OK) 
        {
//         printf("findArmGraspsConfigs != MANIPULATION_TASK_OK\n"); 
//         return 1;
//         continue;
        }
// 	if(q==NULL) 
//         {
//         printf("q==NULL after setRobotGraspPosWithoutBase()\n"); 
//         continue;
//         }
        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_put.x;
        m_objGoto[1]= point_to_put.y;
        m_objGoto[2]= point_to_put.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;
//         m_objGoto[3]= P3D_HUGE;
//         m_objGoto[4]= P3D_HUGE;
//         m_objGoto[5]= P3D_HUGE;

       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);
        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);


         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d and grasp id= %d\n",i1, iter->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         printf(">>>>> Found for place %d and grasp id= %d\n",i1, iter->ID);
           result= 1;
           quit= true;
           break;
        }
     }
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}

int JIDO_show_obj_to_human ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_show_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_show_obj();

 printf ( " <<<<<< candidate_points_to_show.no_points = %d >>>>>>>>\n", candidate_points_to_show.no_points );
 if ( candidate_points_to_show.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_show;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     p3d_set_freeflyer_pose(object, T0);
    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
//     status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);



    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);

    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_show.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_show.point[i1].x;
	goal_pos.y=candidate_points_to_show.point[i1].y;
	goal_pos.z=candidate_points_to_show.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_show.x=goal_pos.x;
	point_to_show.y=goal_pos.y;
	point_to_show.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_show.x;
	obj_tmp_pos[7]=point_to_show.y;
	obj_tmp_pos[8]=point_to_show.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_show.x;
	T[1][3]=point_to_show.y;
	T[2][3]=point_to_show.z;

        p3d_set_freeflyer_pose2(object, point_to_show.x, point_to_show.y, point_to_show.z, rx, ry, rz);
       double visibility_threshold=95.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==0)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);



        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);
        for(int i=0; i<1; ++i)
        {
          //q = setRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
         // message= manipulation->findArmGraspsConfigs(armID, object, grasp, configs);
          //if(q!=NULL) break;
        }
	if(message!=MANIPULATION_TASK_OK) 
        {
//         printf("findArmGraspsConfigs != MANIPULATION_TASK_OK\n"); 
//         return 1;
//         continue;
        }
// 	if(q==NULL) 
//         {
//         printf("q==NULL after setRobotGraspPosWithoutBase()\n"); 
//         continue;
//         }
        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_show.x;
        m_objGoto[1]= point_to_show.y;
        m_objGoto[2]= point_to_show.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;
//         m_objGoto[3]= P3D_HUGE;
//         m_objGoto[4]= P3D_HUGE;
//         m_objGoto[5]= P3D_HUGE;

       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);

        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);


         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d and grasp id= %d\n",i1, iter->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         printf(">>>>> Found for place %d and grasp id= %d\n",i1, iter->ID);
           result= 1;
           quit= true;
           break;
        }
     }
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}
#endif
void fct_draw_loop()
{
#if !defined(COMPILE_FOR_GENOM)
  g3d_screenshot((char *)"Move3D");
#endif
}

double get_wrist_head_alignment_angle(p3d_matrix4 wristFrame, p3d_matrix4 headFrame)
{
  double angle;
  p3d_vector3 wristPos, headPos, wristDir, dir;

  p3d_mat4ExtractTrans(wristFrame, wristPos);
  p3d_mat4ExtractColumnZ(wristFrame, wristDir);
  p3d_mat4ExtractTrans(headFrame, headPos);

  p3d_vectSub(headPos, wristPos, dir);
  p3d_vectNormalize(dir, dir);

  angle= acos( p3d_vectDotProd(wristDir, dir) );
  return angle;
}

static int traj_play = TRUE;

static int 
default_drawtraj_fct(p3d_rob* robot, p3d_localpath* curLp)
{
  g3d_draw_allwin_active();
  fl_check_forms();
  return(traj_play);
}


int JIDO_perform_task ( char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs)
{
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
   std::list<gpPlacement> curr_placementList=placementList;

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=0;
if(PLAN_IN_CARTESIAN == 1) 
    {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     manipulation->setArmCartesian(i,true);
     }
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   int armID= 0;
   p3d_matrix4 Tplacement0, T;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   obj_refConf= p3d_get_robot_config ( object );


   ( *manipulation->robot()->armManipulationData ) [armID].setManipState ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );

   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( "SAHandRight" ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( "SAHandRight" ),5,5,5,0,0,0);

   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      p3d_set_freeflyer_pose ( object, Tplacement0 );


      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      for ( int i=0; i<5; ++i )
      {
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );

         graspConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {
          printf(" No IK Found to grasp\n");
          continue;
         }
         printf ( "graspConf= %p\n", graspConf );
         if ( graspConf!=NULL )
         {
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );

            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               g3d_draw_allwin_active();
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );

                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );

//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " Found for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );

                    

/* Following is commented because no need to concatanate trajs segments and compute soft motion at this stage
                     manipulation->concatTrajectories ( take_trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );
*/
                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

                        if(task==MAKE_OBJECT_ACCESSIBLE||task==HIDE_OBJECT)
                        {

                        std::list<gpPlacement> general_stable_configuration_list; 
                        std::list<gpPlacement> tmp_placement_list; 
                        curr_placementList.clear();
                        get_placements_at_position(  object, goal_pos, placementList, 10, curr_placementList );
        //printf("stable_placements_list.size()=%d, curr_placementListOut.size = %d \n", stable_placements_list.size(), curr_placementListOut.size());
                        printf("curr_placementList.size = %d \n",  curr_placementList.size());
                        if(curr_placementList.size()==0)
                          {
                         continue;
        ////return 0;
                          }
                         tmp_placement_list.clear();
                         tmp_placement_list= curr_placementList;
	                 gpPlacement_on_support_filter ( object, envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]],tmp_placement_list,curr_placementList);                          
                          printf(" Support name for placement =%s\n",envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
                          printf("After gpPlacement_on_support_filter(), curr_placementList.size = %d \n",  curr_placementList.size());
                          if(curr_placementList.size()==0)
                          {
                          
                         continue;
        ////return 0;
                          }
                         }

                        if(task!=HIDE_OBJECT)
                        {
                        get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[indices_of_MA_agents[for_agent]], curr_placementList );
                        }

                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           if(task!=HIDE_OBJECT)
                           {
                           if ( iplacement->stability<=0 ) //As the placement list is sorted based on the visibility range. NOTE: This stability is based on the visibility range of the front
                            {
                              break;
                            }
                           }

                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;

                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
                           g3d_draw_allwin_active();
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

                           //// p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );

                           if(task==GIVE_OBJECT||task==SHOW_OBJECT)
                           {
                           p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
                           if( get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos) > 30*DEGTORAD)
                            { 
                            printf(" Wrist Alignment is not good \n");
                            continue; 
                            }
                           }

                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);

                           for ( int j=0; j<5; ++j )
                           {
                              deactivateCcCntrts(manipulation->robot(), armID);
                              p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              ////p3d_matrix4 testFrame;
                              ////p3d_mat4Copy(Tplacement,testFrame);
                              
                              placeConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {
                               printf(" No IK Found to place\n");
                               continue;
                              }

                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                               printf(" Place config IK found\n");
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                if(task==SHOW_OBJECT||task==GIVE_OBJECT) //Need to set the robot at placement config to test the visibility of object while in robot's hand
                                {
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                }
                                else
                                {
                                if(task==HIDE_OBJECT||task==GIVE_OBJECT) //Need to set the robot at placement config to test the visibility of object while in robot's hand
                                 {
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                 }
                                }
                                g3d_draw_allwin_active();

                                
                                g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[for_agent]->perspective->fov, object, &visibility, 0 );

                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                continue;
                                } 

                                
                                 printf(" IK found to place and the object's visibility %lf is good\n",visibility); 
                                
                                
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );

                               p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "SHELF" );
                               p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);

                               p3d_mat4Copy(Tplacement0, Tplacement);
                               Tplacement[2][3]+= 0.03;
                               p3d_set_freeflyer_pose ( object, Tplacement );
                               
                               liftConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
//                                liftConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, p3d_mat4IDENTITY,  0, 0, armID, false );
                                 g3d_draw_allwin_active();
                               if ( liftConf==NULL )
                               {
                                printf(" Fail to find liftConf\n");
                                continue;
                               }
                               printf(" LiftConf found\n");
                              
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 deactivateCcCntrts(manipulation->robot(), armID);

                                 //This test is to avoid the situations where robot should not make a big loop but it is doing so. Like between grasp and lift configs TODO: Do such tests between approach and open hand grasp configs also
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1)
                                 {
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   liftConf = NULL;
                                   printf(" Fail optimizeRedundentJointConfigDist()\n");
                                   continue;
                                  }
                                
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               ////if(liftConf!=NULL)
                               ////{
                                  printf(" %d th IK Found to lift \n",j);
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);

                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
/*
                                  manipulation->robot()->tcur=place_trajs[0];

                                  g3d_show_tcur_rob(manipulation->robot(),default_drawtraj_fct);

                                  manipulation->robot()->tcur=place_trajs[1];

                                  g3d_show_tcur_rob(manipulation->robot(),default_drawtraj_fct);
*/
                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_GRASP;
                                  traj_sub_task.traj=take_trajs[1];
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=GRASP;
                                  traj_sub_task.traj=take_trajs[2];
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=LIFT_OBJECT;
                                  traj_sub_task.traj=place_trajs[0];
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=CARRY_OBJECT;
                                  traj_sub_task.traj=place_trajs[1];
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);

/* Below is commented because no need to concatinate traj now and no need to generate soft motion
 
                                  manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
*/
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config ( manipulation->robot(), obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
                                   return 1;
                                  }////else {printf ( " no path found for place \n"); return 1;}
                               ////}

                              }
                           }
                        }
                     }

                     ////return 1;
                  }
                  else
                  {
                     ManipulationUtils::printManipulationMessage ( status );
                  }
               ////}
            ////}
         }
      }
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      continue;
/*

      printf ( " >>>>> before armPlanTask(ARM_PICK_GOTO \n" );
      status = manipulation->armPlanTask ( ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "", *igrasp, confs, smTrajs );
      printf ( " >>>>>**** after armPlanTask(ARM_PICK_GOTO \n" );

      if ( qcur!=NULL )
         p3d_destroy_config ( manipulation->robot(), qcur );

      qcur= p3d_get_robot_config ( manipulation->robot() );
      p3d_copy_config_into ( manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur );

      if ( status==MANIPULATION_TASK_OK )
      {
         //grasp= manipulation->getCurrentGrasp();
         // grasp= *( manipulation->getManipulationData().getGrasp() );
         //return 0;
         remove ( "softMotion_Smoothed_Q_goto.traj" );
         remove ( "softMotion_Smoothed_Seg_goto.traj" );
         rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj" );
         rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj" );
      }
      else
      {
         printf ( "%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__ );
         continue;
      }
      //grasp.print();


      p3d_mat4Mult ( igrasp->frame, armHandProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, mData.getCcCntrt()->Tatt2, tAtt );
      //Check if there is a valid configuration of the robot using this graspFrame

      // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
      // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

      for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      {
         printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
//          if(i1==0)
//     goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
         goal_pos.x=curr_candidate_points->point[i1].x;
         goal_pos.y=curr_candidate_points->point[i1].y;
         goal_pos.z=curr_candidate_points->point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
//    goal_pos.x=T0[0][3];
//    goal_pos.y=T0[1][3];
//    goal_pos.z=T0[2][3]+0.04;


         point_to_give.x=goal_pos.x;
         point_to_give.y=goal_pos.y;
         point_to_give.z=goal_pos.z;

         obj_tmp_pos[6]=point_to_give.x;
         obj_tmp_pos[7]=point_to_give.y;
         obj_tmp_pos[8]=point_to_give.z;

         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );

         for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
         {
            ////iter->draw(0.05);
            if ( iplacement->stability<=0 ) //As the placement list is sorted
               break;
            p3d_mat4Copy ( Tplacement0, T );

            iplacement->position[0]= point_to_give.x;
            iplacement->position[1]= point_to_give.y;
            iplacement->position[2]= point_to_give.z;
            p3d_matrix4 Tplacement;
            iplacement->computePoseMatrix ( Tplacement );
            p3d_set_freeflyer_pose ( object, Tplacement );
            p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
// // //        T[0][3]=point_to_give.x;
// // //    T[1][3]=point_to_give.y;
// // //    T[2][3]=point_to_give.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);

            double visibility_threshold=95.0;
            int is_visible=is_object_visible_for_agent ( HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1 );
            printf ( " is_visible for place %d= %d \n",i1,is_visible );
            if ( is_visible==0 )
               continue;

            configPt qobject= p3d_get_robot_config ( object );
            p3d_copy_config_into ( object, qobject, &object->ROBOT_GOTO );
            p3d_destroy_config ( object, qobject );


            g3d_draw_allwin_active();
            p3d_set_freeflyer_pose ( object, T0 );

            m_objStart.resize ( 6 );
            m_objGoto.resize ( 6 );
            m_objStart[0]= P3D_HUGE;
            m_objStart[1]= P3D_HUGE;
            m_objStart[2]= P3D_HUGE;
            m_objStart[3]= P3D_HUGE;
            m_objStart[4]= P3D_HUGE;
            m_objStart[5]= P3D_HUGE;

            m_objGoto[0]= point_to_give.x;
            m_objGoto[1]= point_to_give.y;
            m_objGoto[2]= point_to_give.z;
            m_objGoto[3]= rx;
            m_objGoto[4]= ry;
            m_objGoto[5]= rz;


            p3d_copy_config_into ( manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS );
            //switch to cartesian for the giving motion:
            // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

            status= manipulation->armPlanTask ( ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "HRP2TABLE", confs, smTrajs );
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

            printf ( " After armPlanTask, result = %d \n",status );
            //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
            printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );

            if ( status==MANIPULATION_TASK_OK )
            {
               remove ( "softMotion_Smoothed_Q_place.traj" );
               remove ( "softMotion_Smoothed_Seg_place.traj" );
               rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj" );
               rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj" );
               printf ( ">>>>> Found for place %d and grasp id= %d\n",i1, igrasp->ID );
               result= 1;
               quit= true;

//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   // COMPUTE THE SOFTMOTION TRAJECTORY 
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
               break;
            }

         }//End of placement itr
         if ( quit==true )
            break;
      }//End for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      if ( quit==true )
      {
         break;
      }



      result= 0;*/
   }
   result= 0;
   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( manipulation->robot(), obj_refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   //////////p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}

int JIDO_give_obj_to_human ( char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> &graspList, std::list<gpPlacement> &curr_placementList)
{
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=1;
if(PLAN_IN_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   int armID= 0;
   p3d_matrix4 Tplacement0, T;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> place_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );


   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );


   ( *manipulation->robot()->armManipulationData ) [armID].setManipState ( handFree ) ;
   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );

   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      p3d_set_freeflyer_pose ( object, Tplacement0 );


      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      for ( int i=0; i<5; ++i )
      {
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );

         graspConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {/*printf(" No IK Found to grasp\n");*/
          continue;
         }
         printf ( "graspConf= %p\n", graspConf );
         if ( graspConf!=NULL )
         {
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );

            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               g3d_draw_allwin_active();
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );

                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, trajs );

//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " Found for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
                     manipulation->concatTrajectories ( trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );

                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

                        get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[indices_of_MA_agents[for_agent]], curr_placementList );

                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           if ( iplacement->stability<=0 ) //As the placement list is sorted
                           {
                              break;
                           }

                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;

                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

                           //// p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );

                           if(task==GIVE_OBJECT||task==SHOW_OBJECT)
                           {
                           p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
                           if( get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos) > 30*DEGTORAD)
                            { 
                            continue; 
                            }
                           }

                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);

                           for ( int j=0; j<5; ++j )
                           {
                              deactivateCcCntrts(manipulation->robot(), armID);
                              p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              p3d_matrix4 testFrame;
                              p3d_mat4Copy(Tplacement,testFrame);
                              
                              placeConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {/*printf(" No IK Found to place\n");*/
                               continue;
                              }

                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                g3d_draw_allwin_active();

                                
                                g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[for_agent]->perspective->fov, object, &visibility, 0 );

                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                continue;
                                } 

                                
                                 printf(" IK found to place and the object's visibility %lf is good\n",visibility); 
                                
                                
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );

                               p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "SHELF" );
                               //////////p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);

                               p3d_mat4Copy(Tplacement0, Tplacement);
                               Tplacement[2][3]+= 0.1;
                               p3d_set_freeflyer_pose ( object, Tplacement );
                               liftConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                               
                               if ( liftConf==NULL )
                               {
                                continue;
                               }

                              
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 deactivateCcCntrts(manipulation->robot(), armID);

                                 //This test is to avoid the situations where robot should not make a big loop but it is doing so. Like between grasp and lift configs TODO: Do such tests between approach and open hand grasp configs also
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1)
                                 {
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   liftConf = NULL;
                                   continue;
                                  }
                                
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               ////if(liftConf!=NULL)
                               ////{
                                  printf(" %d th IK Found to lift \n",j);
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);

                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
                                 manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                    
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );
                                  return 1;
                                  }////else {printf ( " no path found for place \n"); return 1;}
                               ////}

                              }
                           }
                        }
                     }

                     ////return 1;
                  }
                  else
                  {
                     ManipulationUtils::printManipulationMessage ( status );
                  }
               ////}
            ////}
         }
      }
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      continue;
/*

      printf ( " >>>>> before armPlanTask(ARM_PICK_GOTO \n" );
      status = manipulation->armPlanTask ( ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "", *igrasp, confs, smTrajs );
      printf ( " >>>>>**** after armPlanTask(ARM_PICK_GOTO \n" );

      if ( qcur!=NULL )
         p3d_destroy_config ( manipulation->robot(), qcur );

      qcur= p3d_get_robot_config ( manipulation->robot() );
      p3d_copy_config_into ( manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur );

      if ( status==MANIPULATION_TASK_OK )
      {
         //grasp= manipulation->getCurrentGrasp();
         // grasp= *( manipulation->getManipulationData().getGrasp() );
         //return 0;
         remove ( "softMotion_Smoothed_Q_goto.traj" );
         remove ( "softMotion_Smoothed_Seg_goto.traj" );
         rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj" );
         rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj" );
      }
      else
      {
         printf ( "%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__ );
         continue;
      }
      //grasp.print();


      p3d_mat4Mult ( igrasp->frame, armHandProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, mData.getCcCntrt()->Tatt2, tAtt );
      //Check if there is a valid configuration of the robot using this graspFrame

      // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
      // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

      for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      {
         printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
//          if(i1==0)
//     goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
         goal_pos.x=curr_candidate_points->point[i1].x;
         goal_pos.y=curr_candidate_points->point[i1].y;
         goal_pos.z=curr_candidate_points->point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
//    goal_pos.x=T0[0][3];
//    goal_pos.y=T0[1][3];
//    goal_pos.z=T0[2][3]+0.04;


         point_to_give.x=goal_pos.x;
         point_to_give.y=goal_pos.y;
         point_to_give.z=goal_pos.z;

         obj_tmp_pos[6]=point_to_give.x;
         obj_tmp_pos[7]=point_to_give.y;
         obj_tmp_pos[8]=point_to_give.z;

         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );

         for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
         {
            ////iter->draw(0.05);
            if ( iplacement->stability<=0 ) //As the placement list is sorted
               break;
            p3d_mat4Copy ( Tplacement0, T );

            iplacement->position[0]= point_to_give.x;
            iplacement->position[1]= point_to_give.y;
            iplacement->position[2]= point_to_give.z;
            p3d_matrix4 Tplacement;
            iplacement->computePoseMatrix ( Tplacement );
            p3d_set_freeflyer_pose ( object, Tplacement );
            p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
// // //        T[0][3]=point_to_give.x;
// // //    T[1][3]=point_to_give.y;
// // //    T[2][3]=point_to_give.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);

            double visibility_threshold=95.0;
            int is_visible=is_object_visible_for_agent ( HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1 );
            printf ( " is_visible for place %d= %d \n",i1,is_visible );
            if ( is_visible==0 )
               continue;

            configPt qobject= p3d_get_robot_config ( object );
            p3d_copy_config_into ( object, qobject, &object->ROBOT_GOTO );
            p3d_destroy_config ( object, qobject );


            g3d_draw_allwin_active();
            p3d_set_freeflyer_pose ( object, T0 );

            m_objStart.resize ( 6 );
            m_objGoto.resize ( 6 );
            m_objStart[0]= P3D_HUGE;
            m_objStart[1]= P3D_HUGE;
            m_objStart[2]= P3D_HUGE;
            m_objStart[3]= P3D_HUGE;
            m_objStart[4]= P3D_HUGE;
            m_objStart[5]= P3D_HUGE;

            m_objGoto[0]= point_to_give.x;
            m_objGoto[1]= point_to_give.y;
            m_objGoto[2]= point_to_give.z;
            m_objGoto[3]= rx;
            m_objGoto[4]= ry;
            m_objGoto[5]= rz;


            p3d_copy_config_into ( manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS );
            //switch to cartesian for the giving motion:
            // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

            status= manipulation->armPlanTask ( ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "HRP2TABLE", confs, smTrajs );
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

            printf ( " After armPlanTask, result = %d \n",status );
            //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
            printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );

            if ( status==MANIPULATION_TASK_OK )
            {
               remove ( "softMotion_Smoothed_Q_place.traj" );
               remove ( "softMotion_Smoothed_Seg_place.traj" );
               rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj" );
               rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj" );
               printf ( ">>>>> Found for place %d and grasp id= %d\n",i1, igrasp->ID );
               result= 1;
               quit= true;

//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   // COMPUTE THE SOFTMOTION TRAJECTORY 
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
               break;
            }

         }//End of placement itr
         if ( quit==true )
            break;
      }//End for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      if ( quit==true )
      {
         break;
      }



      result= 0;*/
   }

   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}

int JIDO_give_obj_to_human_working_version_24_2_2011 ( char *obj_to_manipulate,  candidate_poins_for_task *curr_candidate_points)
{
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

   if ( manipulation== NULL )
   {
      initManipulation();
   }

int PLAN_IN_CARTESIAN=1;
if(PLAN_IN_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   std::list<gpGrasp> graspList;
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   int armID= 0;
   p3d_matrix4 Tplacement0, T;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> place_trajs;
   double visibility, confCost;


   gpGet_grasp_list ( object->name, armHandProp.type, graspList );
   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

//    ////gpReduce_grasp_list_size ( graspList, graspList, 30 );
    std::list<gpGrasp> graspList2;
    graspList2= graspList;
    gpGrasp_handover_filter(p3d_get_robot_by_name ( "SAHandRight" ), p3d_get_robot_by_name ( "SAHandRight2" ), object, graspList, graspList2);
    printf(" After gpGrasp_handover_filter()\n");
    printf(" graspList.size()=%d,graspList2.size()=%d\n",graspList.size(),graspList2.size());
   ////return 0;
//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL;
   std::list<gpPlacement> curr_placementListOut;
   ////get_placements_in_3D ( object,  curr_placementListOut );
   get_placements_in_3D ( obj_to_manipulate,  curr_placementListOut );
   refConf= p3d_get_robot_config ( manipulation->robot() );


// while(graspList.size()>5)
// graspList.pop_back();

   ( *manipulation->robot()->armManipulationData ) [armID].setManipState ( handFree ) ;
   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );
   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      p3d_set_freeflyer_pose ( object, Tplacement0 );


      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      for ( int i=0; i<5; ++i )
      {
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );

         graspConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {/*printf(" No IK Found to grasp\n");*/}
         printf ( "graspConf= %p\n", graspConf );
         if ( graspConf!=NULL )
         {
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );
            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               g3d_draw_allwin_active();
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );

                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, trajs );

//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " Found for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
                     manipulation->concatTrajectories ( trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );

                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

                        get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );
                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           if ( iplacement->stability<=0 ) //As the placement list is sorted
                              break;
                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;
                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                            p3d_set_freeflyer_pose ( object, Tplacement );
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );
                            p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
//                             g3d_win *win= NULL;
//                             win= g3d_get_cur_win();
//                             win->fct_draw2= &(execute_Mightability_Map_functions);

//                             p3d_mat4ExtractTrans(FRAME, startPoint);
//                             p3d_mat4ExtractColumnZ(FRAME, pointingDirection);
//                             for(int ki=0; ki<3; ++ki)
//                             { endPoint[ki]= startPoint[ki] + 10*pointingDirection[ki]; }
//                             p3d_mat4ExtractTrans(HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, headCenter);
//                             headRadius= 0.5;
// p3d_vectCopy(headCenter, SPHERE_CENTER);
// p3d_vectCopy(startPoint, STARTPOINT);
// p3d_vectCopy(endPoint, ENDPOINT);
//                             p3d_vectSub(headCenter, wristPosition, )
// 
// 
//                           //  g3d_draw_allwin_active();
//                             if( gpLine_segment_sphere_intersection(startPoint, endPoint, headCenter, headRadius, intersection1, intersection2)==0 )
//                             { continue; }
                           if( get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos) > 30*DEGTORAD)
                            { continue; }

                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                           for ( int j=0; j<5; ++j )
                           {
                              deactivateCcCntrts(manipulation->robot(), armID);
                              p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              p3d_matrix4 testFrame;
                              p3d_mat4Copy(Tplacement,testFrame);
                              
                              placeConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {/*printf(" No IK Found to place\n");*/}

                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                g3d_draw_allwin_active();
                                g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 0 );
                                printf ( " visibility=%lf\n",visibility );
                                if(visibility<=0.1)
                                {
                                 printf(" IK found to place but the object's visibility %lf is not good\n",visibility);
                                 p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                 continue;
                                }

                                 printf(" IK found to place and the object's visibility %lf is good\n",visibility); 
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );

                               p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "SHELF" );
                               p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);

                               p3d_mat4Copy(Tplacement0, Tplacement);
                               Tplacement[2][3]+= 0.1;
                               p3d_set_freeflyer_pose ( object, Tplacement );
                               liftConf= setRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                               
                               if ( liftConf ){
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 deactivateCcCntrts(manipulation->robot(), armID);
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1){
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   liftConf = NULL;
                                   continue;
                                  }
                                }
                                else
                                {
                                 
                                } 
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               if(liftConf!=NULL)
                               {printf(" %d th IK Found to lift \n",j);
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);

                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
                                 manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                  return 1;
                                  }else {printf ( " no path found for place \n"); return 1;}
                               }

                              }
                           }
                        }
                     }

                     ////return 1;
                  }
                  else
                  {
                     ManipulationUtils::printManipulationMessage ( status );
                  }
               ////}
            ////}
         }
      }
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      continue;
/*

      printf ( " >>>>> before armPlanTask(ARM_PICK_GOTO \n" );
      status = manipulation->armPlanTask ( ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "", *igrasp, confs, smTrajs );
      printf ( " >>>>>**** after armPlanTask(ARM_PICK_GOTO \n" );

      if ( qcur!=NULL )
         p3d_destroy_config ( manipulation->robot(), qcur );

      qcur= p3d_get_robot_config ( manipulation->robot() );
      p3d_copy_config_into ( manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur );

      if ( status==MANIPULATION_TASK_OK )
      {
         //grasp= manipulation->getCurrentGrasp();
         // grasp= *( manipulation->getManipulationData().getGrasp() );
         //return 0;
         remove ( "softMotion_Smoothed_Q_goto.traj" );
         remove ( "softMotion_Smoothed_Seg_goto.traj" );
         rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj" );
         rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj" );
      }
      else
      {
         printf ( "%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__ );
         continue;
      }
      //grasp.print();


      p3d_mat4Mult ( igrasp->frame, armHandProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, mData.getCcCntrt()->Tatt2, tAtt );
      //Check if there is a valid configuration of the robot using this graspFrame

      // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
      // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

      for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      {
         printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
//          if(i1==0)
//     goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
         goal_pos.x=curr_candidate_points->point[i1].x;
         goal_pos.y=curr_candidate_points->point[i1].y;
         goal_pos.z=curr_candidate_points->point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
//    goal_pos.x=T0[0][3];
//    goal_pos.y=T0[1][3];
//    goal_pos.z=T0[2][3]+0.04;


         point_to_give.x=goal_pos.x;
         point_to_give.y=goal_pos.y;
         point_to_give.z=goal_pos.z;

         obj_tmp_pos[6]=point_to_give.x;
         obj_tmp_pos[7]=point_to_give.y;
         obj_tmp_pos[8]=point_to_give.z;

         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );

         for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
         {
            ////iter->draw(0.05);
            if ( iplacement->stability<=0 ) //As the placement list is sorted
               break;
            p3d_mat4Copy ( Tplacement0, T );

            iplacement->position[0]= point_to_give.x;
            iplacement->position[1]= point_to_give.y;
            iplacement->position[2]= point_to_give.z;
            p3d_matrix4 Tplacement;
            iplacement->computePoseMatrix ( Tplacement );
            p3d_set_freeflyer_pose ( object, Tplacement );
            p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
// // //        T[0][3]=point_to_give.x;
// // //    T[1][3]=point_to_give.y;
// // //    T[2][3]=point_to_give.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);

            double visibility_threshold=95.0;
            int is_visible=is_object_visible_for_agent ( HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1 );
            printf ( " is_visible for place %d= %d \n",i1,is_visible );
            if ( is_visible==0 )
               continue;

            configPt qobject= p3d_get_robot_config ( object );
            p3d_copy_config_into ( object, qobject, &object->ROBOT_GOTO );
            p3d_destroy_config ( object, qobject );


            g3d_draw_allwin_active();
            p3d_set_freeflyer_pose ( object, T0 );

            m_objStart.resize ( 6 );
            m_objGoto.resize ( 6 );
            m_objStart[0]= P3D_HUGE;
            m_objStart[1]= P3D_HUGE;
            m_objStart[2]= P3D_HUGE;
            m_objStart[3]= P3D_HUGE;
            m_objStart[4]= P3D_HUGE;
            m_objStart[5]= P3D_HUGE;

            m_objGoto[0]= point_to_give.x;
            m_objGoto[1]= point_to_give.y;
            m_objGoto[2]= point_to_give.z;
            m_objGoto[3]= rx;
            m_objGoto[4]= ry;
            m_objGoto[5]= rz;


            p3d_copy_config_into ( manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS );
            //switch to cartesian for the giving motion:
            // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

            status= manipulation->armPlanTask ( ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "HRP2TABLE", confs, smTrajs );
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

            printf ( " After armPlanTask, result = %d \n",status );
            //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
            printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );

            if ( status==MANIPULATION_TASK_OK )
            {
               remove ( "softMotion_Smoothed_Q_place.traj" );
               remove ( "softMotion_Smoothed_Seg_place.traj" );
               rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj" );
               rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj" );
               printf ( ">>>>> Found for place %d and grasp id= %d\n",i1, igrasp->ID );
               result= 1;
               quit= true;

//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   // COMPUTE THE SOFTMOTION TRAJECTORY 
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
               break;
            }

         }//End of placement itr
         if ( quit==true )
            break;
      }//End for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      if ( quit==true )
      {
         break;
      }



      result= 0;*/
   }

   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}



int copy_HRI_task_candidate_points(candidate_poins_for_task *from_candidate_points, candidate_poins_for_task *to_candidate_points)
{
  for(int i=0;i<from_candidate_points->no_points;i++)
  {
    to_candidate_points->point[i].x=from_candidate_points->point[i].x;
    to_candidate_points->point[i].y=from_candidate_points->point[i].y;
    to_candidate_points->point[i].z=from_candidate_points->point[i].z;
    to_candidate_points->weight[i]=from_candidate_points->weight[i];
    to_candidate_points->horizontal_surface_of[i]=from_candidate_points->horizontal_surface_of[i];
    
  }
  
  to_candidate_points->curr_solution_point_index=from_candidate_points->curr_solution_point_index;
  to_candidate_points->no_points=from_candidate_points->no_points;
  return 1;
}

int JIDO_find_HRI_task_solution(HRI_TASK_TYPE CURR_TASK, HRI_TASK_AGENT for_agent, char *obj_to_manipulate)
{
   
   
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
    
   candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
   ////find_candidate_points_for_current_HRI_task(CURR_TASK,  for_agent, JIDO_MA,curr_resultant_candidate_points);
   
   find_candidate_points_for_current_HRI_task(CURR_TASK,  JIDO_MA, for_agent,curr_resultant_candidate_points);
   
   switch (CURR_TASK)
   {
     case MAKE_OBJECT_ACCESSIBLE:
       assign_weights_on_candidte_points_to_put_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT, rob_indx.HUMAN);
     break;
     
     case SHOW_OBJECT:
	assign_weights_on_candidte_points_to_show_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT,rob_indx.HUMAN);
      break;
      
     case GIVE_OBJECT:
       assign_weights_on_candidte_points_to_give_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT, rob_indx.HUMAN);
      break;
      
      case HIDE_OBJECT:
       assign_weights_on_candidte_points_to_hide_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT, rob_indx.HUMAN);
      break;
      
   }
  
   
   reverse_sort_HRI_task_weighted_candidate_points(curr_resultant_candidate_points);
   CANDIDATE_POINTS_FOR_TASK_FOUND=1;
   ////CANDIDATE_POINTS_FOR_CURRENT_TASK=curr_resultant_candidate_points;
   copy_HRI_task_candidate_points(curr_resultant_candidate_points,&resultant_current_candidate_point);
   CANDIDATE_POINTS_FOR_CURRENT_TASK=&resultant_current_candidate_point;
   MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);
}

#ifndef COMMENT_TMP

int JIDO_make_obj_accessible_to_human ( char *obj_to_manipulate )
{
   if(Affordances_Found==0)
    {
      printf(" >>>> MA ERROR : First initialize Mightability Maps. NOT moving ahead. \n");
      return 0;
    }
    
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
 ////candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
 ////find_candidate_points_for_current_HRI_task(CURRENT_HRI_MANIPULATION_TASK, JIDO, HUMAN1, curr_resultant_candidate_points);
 ////CANDIDATE_POINTS_FOR_TASK_FOUND=1;
 get_set_of_points_to_put_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_put_obj();

 printf ( " <<<<<< candidate_points_to_put.no_points = %d >>>>>>>>\n", candidate_points_to_put.no_points );
 ////return 0;
 if ( candidate_points_to_put.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_put;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;
  p3d_traj *traj = NULL;
  std::vector <p3d_traj*> trajs;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//    ////gpReduce_grasp_list_size ( graspList, graspList, 30 );
//    std::list<gpGrasp> graspList2;
//    graspList2= graspList;
//    gpGrasp_handover_filter(p3d_get_robot_by_name ( "SAHandRight" ), p3d_get_robot_by_name ( "SAHandRight2" ), object, graspList, graspList2);
//    printf(" After gpGrasp_handover_filter()\n");
//    printf(" graspList.size()=%d,graspList2.size()=%d\n",graspList.size(),graspList2.size());
   ////return 0;
//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
std::list<gpPlacement> curr_placementListOut;
std::list<gpPlacement> general_stable_configuration_list; 
std::list<gpPlacement> tmp_placement_list; 
  ////////get_placements_in_3D(object,  curr_placementListOut);
  gpCompute_stable_placements (object, general_stable_configuration_list );
  ////curr_placementListOut=general_stable_confi_list;
  
   int valid_grasp_ctr=0;

  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     ////if(iter->ID==11)
     ////continue;
     p3d_set_freeflyer_pose(object, T0);
    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
    printf(" >>>>> before armPlanTask(ARM_PICK_GOTO \n");
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    printf(" >>>>>**** after armPlanTask(ARM_PICK_GOTO \n");

    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);
    printf(" >>>>>>> Status of manipulation->armPlanTask(ARM_PICK_GOTO) = %d\n",status);
    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
     valid_grasp_ctr++;
     remove("softMotion_Smoothed_Q_goto.traj");
     remove("softMotion_Smoothed_Seg_goto.traj");
     rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj");
     rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj");
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_put.point[i1].x;
	goal_pos.y=candidate_points_to_put.point[i1].y;
	goal_pos.z=candidate_points_to_put.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_put.x=goal_pos.x;
	point_to_put.y=goal_pos.y;
	point_to_put.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_put.x;
	obj_tmp_pos[7]=point_to_put.y;
	obj_tmp_pos[8]=point_to_put.z;
        
        curr_placementListOut.clear();
        get_placements_at_position(  object, goal_pos, general_stable_configuration_list, 10, curr_placementListOut );
        //printf("stable_placements_list.size()=%d, curr_placementListOut.size = %d \n", stable_placements_list.size(), curr_placementListOut.size());
        printf("curr_placementListOut.size = %d \n",  curr_placementListOut.size());
//         if(curr_placementListOut.size()==0)
//         {
//          continue;
//         ////return 0;
//         }
        tmp_placement_list.clear();
        tmp_placement_list= curr_placementListOut;
	gpPlacement_on_support_filter ( object, envPt_MM->robot[candidate_points_to_put.horizontal_surface_of[i1]],tmp_placement_list,curr_placementListOut );
        ////get_ranking_based_on_view_point(primary_human_MM->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut);

        for(std::list<gpPlacement>::iterator iter_p=curr_placementListOut.begin(); iter_p!=curr_placementListOut.end(); ++iter_p)
        {
  ////iter->draw(0.05); 
         if(iter_p->stability<=0)//As the placement list is sorted
         break;
                p3d_mat4Copy(T0, T);
	
        iter_p->position[0]= point_to_put.x;
    	iter_p->position[1]= point_to_put.y;
    	iter_p->position[2]= point_to_put.z;
        p3d_matrix4 Tplacement;
        iter_p->computePoseMatrix(Tplacement);
        p3d_set_freeflyer_pose(object, Tplacement);
        p3d_get_freeflyer_pose2(object,  &x, &y, &z, &rx, &ry, &rz);
// // //        T[0][3]=point_to_put.x;
// // // 	T[1][3]=point_to_put.y;
// // // 	T[2][3]=point_to_put.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_put.x, point_to_put.y, point_to_put.z, rx, ry, rz);

       double visibility_threshold=85.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==0)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);


        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);

        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_put.x;
        m_objGoto[1]= point_to_put.y;
        m_objGoto[2]= point_to_put.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;


       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);
       //switch to cartesian for the giving motion:
      // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d, grasp id= %d and placement id = %d\n",i1, iter->ID,iter_p->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         remove("softMotion_Smoothed_Q_place.traj");
         remove("softMotion_Smoothed_Seg_place.traj");
         rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
         rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
         printf(">>>>> Found for place %d and grasp id= %d and placement id = %d\n",i1, iter->ID,iter_p->ID);
           result= 1;
           quit= true;
           
//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   /* COMPUTE THE SOFTMOTION TRAJECTORY */
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
           break;
        }
     
      }//End of placement itr
       if(quit==true)
      break;
     }//End for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);

//WARNING: TMP for Videos, Comment the line below
 p3d_col_deactivate_robot(object);

 // g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
 // win->fct_draw2= & ( fct_draw_loop);
 printf(" valid_grasp_ctr=%d\n",valid_grasp_ctr);
 return result;
 
}
#endif
#ifndef COMMENT_TMP
int JIDO_hide_obj_from_human ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_hide_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_hide_obj();

 printf ( " <<<<<< candidate_points_to_hide.no_points = %d >>>>>>>>\n", candidate_points_to_hide.no_points );
 if ( candidate_points_to_hide.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_hide;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     p3d_set_freeflyer_pose(object, T0);
    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
//     status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);



    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);

    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_hide.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_hide.point[i1].x;
	goal_pos.y=candidate_points_to_hide.point[i1].y;
	goal_pos.z=candidate_points_to_hide.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_hide.x=goal_pos.x;
	point_to_hide.y=goal_pos.y;
	point_to_hide.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_hide.x;
	obj_tmp_pos[7]=point_to_hide.y;
	obj_tmp_pos[8]=point_to_hide.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_hide.x;
	T[1][3]=point_to_hide.y;
	T[2][3]=point_to_hide.z;

        p3d_set_freeflyer_pose2(object, point_to_hide.x, point_to_hide.y, point_to_hide.z, rx, ry, rz);

       double visibility_threshold=5.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 0);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==1)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);



        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);
        for(int i=0; i<1; ++i)
        {
          //q = setRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
         // message= manipulation->findArmGraspsConfigs(armID, object, grasp, configs);
          //if(q!=NULL) break;
        }
	if(message!=MANIPULATION_TASK_OK) 
        {
//         printf("findArmGraspsConfigs != MANIPULATION_TASK_OK\n"); 
//         return 1;
//         continue;
        }
// 	if(q==NULL) 
//         {
//         printf("q==NULL after setRobotGraspPosWithoutBase()\n"); 
//         continue;
//         }
        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_hide.x;
        m_objGoto[1]= point_to_hide.y;
        m_objGoto[2]= point_to_hide.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;
//         m_objGoto[3]= P3D_HUGE;
//         m_objGoto[4]= P3D_HUGE;
//         m_objGoto[5]= P3D_HUGE;

       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);
        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);


         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d and grasp id= %d\n",i1, iter->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         printf(">>>>> Found for place %d and grasp id= %d\n",i1, iter->ID);
           result= 1;
           quit= true;
           break;
        }
     }
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}
#endif

int get_placements_at_position(p3d_rob *object, point_co_ordi point, std::list<gpPlacement> placementList, int no_rot, std::list<gpPlacement> &placementListOut)
{
  int i, count;
  p3d_vector3 zAxis={0.0,0.0,1.0};
  p3d_matrix4 Tcur, T1, T2, Tplacement;
  gpPlacement placement;
  std::vector<double> thetas;
  double verticalOffset= 0.01;

  placementList.sort();
  placementList.reverse();

  placementListOut.clear();


  p3d_get_freeflyer_pose(object, Tcur);

  if(no_rot==0)
  {
    thetas.push_back(0);
  }
  else
  {
    for(i=0; i<no_rot; ++i)
    {
     thetas.push_back( i*2*M_PI/((double) no_rot) );
    }
    
  }

  count= 0;
  for(std::list<gpPlacement>::iterator iterP=placementList.begin(); iterP!=placementList.end(); iterP++)
  { 
     if(count > 10)
     { break; }
     count++;


     for(i=0; i<thetas.size(); ++i)
     {
        placement= (*iterP);
        placement.theta= thetas[i];
	placement.position[0]= point.x;
	placement.position[1]= point.y;
	placement.position[2]= point.z + verticalOffset;

        placement.computePoseMatrix(Tplacement);

        p3d_set_freeflyer_pose(object, Tplacement);
	
        if(p3d_col_test_robot(object, 0))
	{ continue; }

	placementListOut.push_back(placement);
     }
  }

   placementListOut.sort();
   placementListOut.reverse();

  for(std::list<gpPlacement>::iterator iterP=placementListOut.begin(); iterP!=placementListOut.end(); iterP++)
  { 
    ////std::cout << "stability " << iterP->stability << std::endl;
  }

  return 0;
}

int get_placements_in_3D(char *obj_to_manipulate,  std::list<gpPlacement> &placementListOut)
{
p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
  int i, j, k, n;
  double dalpha;
  p3d_matrix4 Tcur, T;
  gpPlacement placement;

  placementListOut.clear();


 
  dalpha= 40*(M_PI/180.0);
  n= 2*M_PI/dalpha;


  for(i=0; i<n; ++i)
  { 
   for(j=0; j<n; ++j)
   { 
	for(k=0; k<n; ++k)
	{ 
		p3d_mat4Pos (T, 0, 0, 0, i*dalpha, j*dalpha, k*dalpha);
		
        placement.stability=i+j+k;//AKP: Tmp for displaying the comfigs with different colors
        placement.polyhedron= object->o[0]->pol[0]->poly;
        p3d_mat4Copy(T, placement.T);
	placementListOut.push_back(placement);
                   
	}
    }
  }


 

  return 0;
}

//! This function is used to know how much a robot (visually) sees an object from a specific viewpoint.
//! \param camera_frame the frame of the viewpoint (the looking direction is X, Y points downward and Z to the left)
//! \param camera_fov the field of view angle of the robot's camera (in degrees)
//! \param robot pointer to the robot
//! \param object pointer to the object
//! \param result return the ratio between the number of object's pixels that are visible
//! and the overall size (in pixels) of the image. So the value is between 0 (invisible object) and 1 (the object
//! occupies all the image).
//! \return 0 in case of success, 1 otherwise
int g3d_is_object_visible_from_robot(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *robot, p3d_rob *object, double *result)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_is_object_visible_from_robot(): input robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
  if(object==NULL)
  {
    printf("%s: %d: g3d_is_object_visible_from_robot(): input object is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
 
  int i, width, height;
  GLint viewport_original[4], viewport[4];
  int displayFrame, displayJoints, displayShadows, displayWalls, displayFloor, displayTiles, cullingEnabled;
  double fov;
  int draw_logo;
  int count;
  unsigned char *image= NULL;
  float red, green, blue;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");

  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_UNLIT_GREEN_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    if(XYZ_ENV->robot[i]==robot) {
      p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_NO_DISPLAY);
    }
    else
     p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_UNLIT_GREEN_DISPLAY);
  }
  // display our robot and object with specific colors:
  p3d_set_robot_display_mode(object, P3D_ROB_UNLIT_RED_DISPLAY);

  glGetIntegerv(GL_VIEWPORT, viewport_original);
  //reduce the display size:
  glViewport(0,0,(GLint)(viewport_original[2]/2),(GLint)(viewport_original[3]/2));
  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];
 
  // save the current display options:
  g3d_save_win_camera(win->vs);
  fov            =  win->vs.fov;
  displayFrame   =  win->vs.displayFrame;
  displayJoints  =  win->vs.displayJoints;
  displayShadows =  win->vs.displayShadows;
  displayWalls   =  win->vs.displayWalls;
  displayFloor   =  win->vs.displayFloor;
  displayTiles   =  win->vs.displayTiles;
  cullingEnabled =  win->vs.cullingEnabled;
  red            =  win->vs.bg[0]; 
  green          =  win->vs.bg[1]; 
  blue           =  win->vs.bg[2]; 
  draw_logo      =  win->vs.enableLogo;

  // only keep what is necessary:
  win->vs.fov            = camera_fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled=  1;
  win->vs.enableLogo = 0;
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);


  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);


  ////g3d_draw_win(win);
  g3d_draw_win_back_buffer(win); //only the object should be drawn in red, everthing else is black

  // restore the display options:
  g3d_restore_win_camera(win->vs);
  win->vs.fov            = fov;
  win->vs.displayFrame   = displayFrame;
  win->vs.displayJoints  = displayJoints;
  win->vs.displayShadows = displayShadows;
  win->vs.displayWalls   = displayWalls;
  win->vs.displayFloor   = displayFloor;
  win->vs.displayTiles   = displayTiles;
  win->vs.cullingEnabled =  cullingEnabled;
  win->vs.enableLogo = draw_logo;
  g3d_set_win_bgcolor(win->vs, red, green, blue);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov

  // reset the display modes of everything
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_DEFAULT_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_DEFAULT_DISPLAY);
  }

//   static int cnt= 0;
//   char name[256];
//   sprintf(name, "/home/jpsaut/BioMove3Dgit/BioMove3D/screenshots/image%i.ppm", cnt++);
//   g3d_export_OpenGL_display(name);


  // get the OpenGL image buffer:
  image = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration

  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  // get the image pixels (from (0,0) position):
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

  // count the pixels corresponding to the object's color:
  count= 0;
  for(i=0; i<width*height; i++)
  {
    if(image[3*i] > 200) {
      count++;
    }
  }

  *result= ((double) count)/((double) width*height);

  free(image);

  glViewport(0,0,(GLint)viewport_original[2],(GLint)viewport_original[3]);

  return 0;
}

int get_ranking_based_on_view_point(p3d_matrix4 view_frame,point_co_ordi point,p3d_rob *object, p3d_rob *human, std::list<gpPlacement> &placementList)
{
  
  double dotX, dotZ, alphaX, alphaZ, visibility;
  p3d_vector3 dir, xAxis, yAxis, zAxis= {0,0,1}, zAxis_view;
  p3d_matrix4 Tcur, T, Topt;
  std::list<gpPlacement>::iterator iplacement;
  GLfloat mat[16];

  dir[0]= view_frame[0][3] - point.x;
  dir[1]= view_frame[1][3] - point.y;
  dir[2]= view_frame[2][3] - point.z;
  p3d_vectNormalize(dir, dir);

  p3d_mat4ExtractColumnZ(view_frame, zAxis_view);

  
  p3d_vectXprod(dir, zAxis, yAxis);
  p3d_vectNormalize(yAxis, yAxis);
  p3d_vectXprod(dir, yAxis, zAxis);

  p3d_mat4Copy(p3d_mat4IDENTITY, Topt);

  Topt[0][0]= dir[0];
  Topt[1][0]= dir[1];
  Topt[2][0]= dir[2];

  Topt[0][1]= -yAxis[0];
  Topt[1][1]= -yAxis[1];
  Topt[2][1]= -yAxis[2];

  Topt[0][2]= -zAxis[0];
  Topt[1][2]= -zAxis[1];
  Topt[2][2]= -zAxis[2];

  p3d_get_freeflyer_pose(object, Tcur);

  for(iplacement=placementList.begin(); iplacement!=placementList.end(); ++iplacement)
  {
    iplacement->position[0]= point.x;
    iplacement->position[1]= point.y;
    iplacement->position[2]= point.z;

    iplacement->polyhedron= object->o[0]->pol[0]->poly;
    iplacement->computePoseMatrix(T);
    p3d_set_freeflyer_pose(object, T);

    if( p3d_col_test_robot(object , 0) )
    {
       iplacement->stability= 0;
    }
    else
    {
       p3d_mat4ExtractColumnX(T, xAxis);
       p3d_mat4ExtractColumnZ(T, zAxis);
//        iplacement->stability= -p3d_vectDotProd(dir, xAxis);
       dotX= p3d_vectDotProd(dir, xAxis);
//        dotZ= zAxis[2];
       dotZ= p3d_vectDotProd(zAxis, zAxis_view);

       alphaX= fabs(acos(dotX));
       alphaZ= fabs(acos(dotZ));

       if( dotX<0  || alphaX > 75*M_PI/180.0 || dotZ<0 || alphaZ > 60*M_PI/180.0 )
       { 
          iplacement->stability= 0;
       }
       else
       {
         ////set_object_at_placement ( object, *iplacement );
         g3d_is_object_visible_from_robot(view_frame, 120, human, object, &visibility);
         
         iplacement->stability= visibility;
         ////////iplacement->stability=0.5;
	 p3d_polyhedre *poly= NULL;
////p3d_rob *horse= p3d_get_robot_by_name("Horse");


// printf(" Drawing object\n");
// 
//  p3d_to_gl_matrix(T, mat);
//  glPushMatrix();
//   glMultMatrixf(mat);
//   g3d_draw_p3d_polyhedre(poly);
//  glPopMatrix();


        }
    }
  }
  ////fl_check_forms();
  ////g3d_draw_allwin_active();
  placementList.sort();
  placementList.reverse();


  p3d_set_freeflyer_pose(object, Tcur);
////////exit(0);
  return 0;
}

////// related to execution of a sequence of tasks //////
int show_desired_HRI_task_plan()
{
 if(SHOW_HRI_TASK_TRAJ_TYPE==0)//Show complete plan
 {
  show_world_state_of_entire_plan(HRI_task_list, SHOW_HRI_PLAN_TYPE);
  return 1;
 }
 
 if(SHOW_HRI_TASK_TRAJ_TYPE==1)//Show plan for selected task
 {
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==CURRENT_HRI_TASK_PLAN_ID_TO_SHOW)
   {
   show_traj_for_this_HRI_task(HRI_task_list[i], SHOW_HRI_PLAN_TYPE);
   return 1;
   }
  }
 }
 
 if(SHOW_HRI_TASK_TRAJ_TYPE==2)//Show plan for selected sub task of selected task
 {
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==CURRENT_HRI_TASK_PLAN_ID_TO_SHOW)
   {
   show_plan_for_this_sub_task(HRI_task_list[i], HRI_task_list[i].traj.sub_task_traj[INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW], INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW, SHOW_HRI_PLAN_TYPE);
   
   return 1;
   }
  }
  
 } 
 
// SHOW_HRI_PLAN_TYPE
// INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW
// CURRENT_HRI_TASK_PLAN_ID_TO_SHOW
//  SHOW_HRI_TASK_TRAJ_TYPE
}

int show_traj_for_this_HRI_task(HRI_task_node &for_task, int show_traj)
{
 for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.before_task.robot_config[j]);
   
   }
g3d_draw_allwin_active();
for(int i=0;i<for_task.traj.sub_task_traj.size();i++)
  {

 

  if(show_traj==1)
   {
  envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->tcur=for_task.traj.sub_task_traj[i].traj;

  printf(" Sub traj type=%d\n",for_task.traj.sub_task_traj[i].sub_task_type);

  if(for_task.traj.sub_task_traj[i].sub_task_type==LIFT_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==CARRY_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==PUT_DOWN_OBJECT)
    {
    envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = TRUE;
    }

  
  g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],default_drawtraj_fct);
 

    if(for_task.traj.sub_task_traj[i].sub_task_type==LIFT_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==CARRY_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==PUT_DOWN_OBJECT)
    {
    envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = FALSE;
    }
   }

   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.traj.sub_task_traj[i].config_after_sub_task.robot_config[j]);
  
   }
  g3d_draw_allwin_active();
  }

 for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.after_task.robot_config[j]);
   
   }
g3d_draw_allwin_active();
return 1;
}

int show_traj_for_this_HRI_sub_task(HRI_task_node &for_task,traj_for_HRI_sub_task &sub_task_traj)
{
  envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->tcur=sub_task_traj.traj;

  if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
   {
     p3d_set_object_to_carry_to_arm(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]], sub_task_traj.armID, for_task.hri_task.for_object.c_str());
     printf(" setting object to carry %s \n",for_task.hri_task.for_object.c_str());
     envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = TRUE;
   }
  
  
  g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],default_drawtraj_fct);

  if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
   {
    envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = FALSE;
   }
 
}

int free_final_configs_stored()
{
 for(int i=envPt_MM->nr-1;i>=0;i--)
  {
   if(final_configs_after_a_task.robot_config[i]!=NULL)
   {
    MY_FREE(final_configs_after_a_task.robot_config[i], double, envPt_MM->robot[i]->nb_dof);

   }
   final_configs_after_a_task.robot_config.pop_back();
  }
  
}

//It will modify the envPt_MM->robots to refect the world state as it would be after executing the sub task 
int get_final_configs_for_this_HRI_sub_task_traj(HRI_task_node &for_task, traj_for_HRI_sub_task &sub_task_traj )
{
////for(int i=0;i<for_task.traj.sub_task_traj.size();i++)
 //// {
 
  envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->tcur=sub_task_traj.traj;//for_task.traj.sub_task_traj[i].traj;

  printf(" Sub traj type=%d\n",sub_task_traj.sub_task_type);

  p3d_localpath* lp = NULL;
  for(lp=sub_task_traj.traj->courbePt;lp->next_lp;lp=lp->next_lp)
  {//Go to the last lp of the trajectory
  }
  

//   if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
//    {}
    if(lp->isCarryingObject)
    {
      p3d_set_object_to_carry_to_arm(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]], sub_task_traj.armID, lp->carriedObject[sub_task_traj.armID]->name);
    }

    final_configs_after_a_task.robot_config.resize(envPt_MM->nr);

    final_configs_after_a_task.robot_config[indices_of_MA_agents[for_task.hri_task.by_agent]]= p3d_config_at_param_along_traj(sub_task_traj.traj,sub_task_traj.traj->range_param);
    
    p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],final_configs_after_a_task.robot_config[indices_of_MA_agents[for_task.hri_task.by_agent]]);
    ////if(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject == TRUE)
    if(lp->isCarryingObject)
    {
      p3d_matrix4 objPos;
     ArmManipulationData& mData = (*envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->armManipulationData)[sub_task_traj.armID];
     p3d_mat4Copy(mData.getManipulationJnt()->abs_pos , objPos);
     p3d_set_freeflyer_pose ( envPt_MM->robot[get_index_of_robot_by_name((char*)for_task.hri_task.for_object.c_str())], objPos );
     envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = false;
     //final_configs_after_a_task.robot_config[get_index_of_robot_by_name(for_task.for_object.c_str())]=
    }
    ////p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],final_configs_after_a_task.robot_config[indices_of_MA_agents[for_task.hri_task.by_agent]]);
    
  /////g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],default_drawtraj_fct);

  //if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
   //{
   // envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = FALSE;
   //}
  
}
 
int validate_HRI_task(HRI_task_desc curr_task, int task_plan_id)
{
 HRI_task_node curr_task_to_validate;
 curr_task_to_validate.task_plan_id=task_plan_id;
 curr_task_to_validate.hri_task.task_type=curr_task.task_type;
 curr_task_to_validate.hri_task.for_object=curr_task.for_object;
 printf(">>> Inside validate_HRI_task(), curr_task_to_validate.hri_task.for_object =%s curr_task.for_object = %s\n",curr_task_to_validate.hri_task.for_object.c_str(), curr_task.for_object.c_str());
 curr_task_to_validate.hri_task.by_agent=curr_task.by_agent;
 curr_task_to_validate.hri_task.for_agent=curr_task.for_agent;

 configPt tmp_config;

 for(int i=0;i<envPt_MM->nr;i++)
  {
  tmp_config=MY_ALLOC(double,envPt_MM->robot[i]->nb_dof);
  p3d_get_robot_config_into(envPt_MM->robot[i],&tmp_config);
  
  curr_task_to_validate.before_task.robot_config.push_back(tmp_config);
  ////MY_FREE(tmp_config,double,envPt_MM->robot[i]->nb_dof);//DONOT FREE it because configs are stored 
  }

  ////traj_for_HRI_task res_trajs;
  
  if(find_current_HRI_manip_task_solution(curr_task, curr_task_to_validate.traj)==0)
  {
   printf(" Fail to validate current task \n");
   return 0;
  }
  
  ////curr_task_to_validate.traj=res_trajs;

  printf(" Num of sub task traj for the current task = %d \n", curr_task_to_validate.traj.sub_task_traj.size());

  //To get the world state if the task will be executed
  // p3d_config_at_param_along_traj(..,..);
  int show_traj=0;
  if(show_traj==1)
  {
   show_traj_for_this_HRI_task(curr_task_to_validate, show_traj);
  }
  
  ////p3d_config_at_param_along_traj(curr_task_to_validate.traj.sub_task_traj.[curr_task_to_validate.traj.sub_task_traj.size()-1]
  for(int i=0;i<curr_task_to_validate.traj.sub_task_traj.size();i++)
  {
  ////get_final_configs_for_this_HRI_sub_task_traj(curr_task_to_validate, curr_task_to_validate.traj.sub_task_traj[curr_task_to_validate.traj.sub_task_traj.size()-1]);
  get_final_configs_for_this_HRI_sub_task_traj(curr_task_to_validate, curr_task_to_validate.traj.sub_task_traj[i]);
  for(int j=0;j<envPt_MM->nr;j++)
   {
  
  tmp_config=MY_ALLOC(double,envPt_MM->robot[j]->nb_dof);
  p3d_get_robot_config_into(envPt_MM->robot[j],&tmp_config);
  
  curr_task_to_validate.traj.sub_task_traj[i].config_after_sub_task.robot_config.push_back(tmp_config);
  ////MY_FREE(tmp_config,double,envPt_MM->robot[i]->nb_dof);//DONOT FREE it because configs are stored 
   }
  free_final_configs_stored();
  }
                                  
  //Store the last sub task final traj as the final traj of the task itself
  for(int i=0;i<envPt_MM->nr;i++)
  {
  
  tmp_config=MY_ALLOC(double,envPt_MM->robot[i]->nb_dof);
  p3d_get_robot_config_into(envPt_MM->robot[i],&tmp_config);
  
  curr_task_to_validate.after_task.robot_config.push_back(tmp_config);
  ////MY_FREE(tmp_config,double,envPt_MM->robot[i]->nb_dof);//DONOT FREE it because configs are stored 
  }

  HRI_task_list.push_back(curr_task_to_validate);

  char task_plan_id_str[20];
  std::string task_plan_desc;
  sprintf (task_plan_id_str, "%d_",curr_task_to_validate.task_plan_id);
  task_plan_desc=task_plan_id_str;
  task_plan_desc+=envPt_MM->robot[indices_of_MA_agents[curr_task_to_validate.hri_task.by_agent]]->name;
  task_plan_desc+='_';
  task_plan_desc+=HRI_task_NAME_ID_map.find(curr_task_to_validate.hri_task.task_type)->second;
  task_plan_desc+='_';
  ////printf("task_plan_desc before adding obj name = %s \n",task_plan_desc.c_str());
 //// printf(" curr_task.for_object.c_str() = %s\n", curr_task.for_object.c_str());
  task_plan_desc+=curr_task_to_validate.hri_task.for_object.c_str();
 ////  printf("task_plan_desc after adding obj name = %s \n",task_plan_desc.c_str());
  task_plan_desc+='_';
  task_plan_desc+=envPt_MM->robot[indices_of_MA_agents[curr_task_to_validate.hri_task.for_agent]]->name;
  
  HRI_task_plan_DESC_ID_map[task_plan_desc]=curr_task_to_validate.task_plan_id;
  printf(" Inserted into HRI_task_plan_DESC_ID_map key= %s and task_plan_id = %d\n",task_plan_desc.c_str(), curr_task_to_validate.task_plan_id);
  ////show_traj_for_this_HRI_task(curr_task_to_validate[0]);
  printf(" Restoring the actual environment before this task. \n");
  for(int i=0;i<envPt_MM->nr;i++)
  {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[i],HRI_task_list[HRI_task_list.size()-1].before_task.robot_config[i]);
   
  }

 ////free_final_configs_stored();
 g3d_draw_allwin_active();
 printf(" returning form validate_HRI_task()\n");
  ////robots_status_for_Mightability_Maps[r_ctr].has_moved
  
}

int show_world_state_of_entire_plan(std::vector<HRI_task_node> &hri_task_list, int exec_path_configs)
{
 for(int i=0;i<hri_task_list.size();i++)
 {
   for(int j=0;j<envPt_MM->nr;j++)
  {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],hri_task_list[i].before_task.robot_config[j]);
   
  }
  g3d_draw_allwin_active();
  fl_check_forms();
  
  for(int k=0;k<hri_task_list[i].traj.sub_task_traj.size();k++)
  {
 
  
   if(exec_path_configs==1)
    {
    show_traj_for_this_HRI_sub_task(hri_task_list[i],hri_task_list[i].traj.sub_task_traj[k]);
    }

   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],hri_task_list[i].traj.sub_task_traj[k].config_after_sub_task.robot_config[j]);

   }

  g3d_draw_allwin_active();
  fl_check_forms();
  
//  printf(" Sub traj type=%d\n",for_task.traj.sub_task_traj[i].sub_task_type);
  }

 
  
  for(int j=0;j<envPt_MM->nr;j++)
  {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],hri_task_list[i].after_task.robot_config[j]);
   
  }
g3d_draw_allwin_active();
  fl_check_forms();
 }

}

int show_plan_for_this_sub_task(HRI_task_node &for_task, traj_for_HRI_sub_task &sub_task_traj, int sub_task_index, int show_traj)
{
  if(sub_task_index==0)
  {
  
  for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.before_task.robot_config[j]);
   
   }
  }
  if(sub_task_index>0)
  {
   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.traj.sub_task_traj[sub_task_index-1].config_after_sub_task.robot_config[j]);

   }
  }

  g3d_draw_allwin_active();
  fl_check_forms();
  
   if(show_traj==1)
    {
    show_traj_for_this_HRI_sub_task(for_task,sub_task_traj);
    }

   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],sub_task_traj.config_after_sub_task.robot_config[j]);

   }

  g3d_draw_allwin_active();
  fl_check_forms();
  
//  printf(" Sub traj type=%d\n",for_task.traj.sub_task_traj[i].sub_task_type);
  

 
 

}
