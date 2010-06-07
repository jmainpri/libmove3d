#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#include "proto/hri_agent_proto.h"
#include "proto/hri_gik_proto.h"

HRI_AGENTS * hri_create_agents()
{
  int i, i_r=0, i_h=0;
  HRI_AGENTS * agents;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  agents = MY_ALLOC(HRI_AGENTS,1);
  agents->robots_no = 0;
  agents->humans_no = 0;

  for(i=0; i<env->nr; i++){
    if(strcasestr(env->robot[i]->name,"ROBOT"))
      agents->robots_no++;
    else
      if(strcasestr(env->robot[i]->name,"HUMAN"))
        agents->humans_no++;
  }

  agents->robots = MY_ALLOC(HRI_AGENT *,agents->robots_no);
  agents->humans = MY_ALLOC(HRI_AGENT *,agents->humans_no);

  for(i=0; i<env->nr; i++){
    if(strcasestr(env->robot[i]->name,"ROBOT")){
      agents->robots[i_r] = hri_create_agent(env->robot[i]);
      i_r++;
    }
    else {
      if(strcasestr(env->robot[i]->name,"HUMAN")) {
        agents->humans[i_h] = hri_create_agent(env->robot[i]);
        i_h++;
      }
    }
  }

  return agents;
}


HRI_AGENT * hri_create_agent(p3d_rob * robot)
{
  HRI_AGENT * hri_agent;

  hri_agent = MY_ALLOC(HRI_AGENT,1);

  if(strcasestr(robot->name,"SUPERMAN")){
    hri_agent->type = HRI_SUPERMAN;
  }
  else {
    if(strcasestr(robot->name,"ACHILE")){
      hri_agent->type = HRI_ACHILE;
    }
    else {
      if(strcasestr(robot->name,"TINMAN")){
        hri_agent->type = HRI_TINMAN;
      }
      else {
        if(strcasestr(robot->name,"JIDO")){
          hri_agent->type = HRI_JIDO1;
        }
        else {
          if(strcasestr(robot->name,"HRP2")){
            hri_agent->type = HRI_HRP214;
          }
          else {
            if(strcasestr(robot->name,"B21")){
              hri_agent->type = HRI_B21;
            }
            else {
              if(strcasestr(robot->name,"JUSTIN")){
                hri_agent->type = HRI_MOBILE_JUSTIN;
              }
              else {
                if(strcasestr(robot->name,"BH")){
                  hri_agent->type = HRI_BH;
                }
                else {
                  if(strcasestr(robot->name,"ICUB")){
                    hri_agent->type = HRI_ICUB;
                  }
                  else {
                    if(strcasestr(robot->name,"BERT")){
                      hri_agent->type = HRI_BERT;
                    }
                    else {
                      PrintWarning(("Robot is unknown! Cannot initialize agents.\n"));
                      return NULL;
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

  hri_agent->robotPt = robot;

  hri_agent->navig  = hri_create_agent_navig(hri_agent);
  hri_agent->manip = hri_create_agent_manip(hri_agent);
  hri_agent->perspective = hri_create_agent_perspective(hri_agent);
  
  hri_agent->exists = FALSE;

  /* TODO: Fix proper state assignment */
  hri_agent->states_no = 0;
  hri_agent->actual_state = 0;
  hri_agent->state = NULL;

  return hri_agent;
}

HRI_MANIP * hri_create_empty_agent_manip()
{
  HRI_MANIP * manip = NULL;

  manip = MY_ALLOC(HRI_MANIP,1);
  manip->gik = NULL;
  manip->tasklist = NULL;
  manip->tasklist_no = 0;

  return manip;
}

HRI_PERSP * hri_create_agent_perspective(HRI_AGENT * agent)
{
  HRI_PERSP * persp = NULL;
  int res;
  
  persp = MY_ALLOC(HRI_PERSP,1);
  
  res = hri_get_default_camera_joint_no(agent->type);
  
  if(res == FALSE){
    PrintError(("can't find camera joints"));
    return NULL;
  }
  
  persp->camjoint = agent->robotPt->joints[res];
  
  switch (agent->type) {
  case HRI_JIDO1:
    persp->fov = 60;
    persp->foa = 60;
    persp->tilt_jnt_idx = 3;
    persp->pan_jnt_idx  = 2;
    break;
  case HRI_ACHILE:
    persp->fov = 160;
    persp->foa = 30;
    persp->tilt_jnt_idx = 6;
    persp->pan_jnt_idx  = 5;
    break;
  case HRI_SUPERMAN:
    persp->fov = 160;
    persp->foa = 30;
    persp->tilt_jnt_idx = 55;
    persp->pan_jnt_idx  = 54;
    break;
  default:
    persp->fov = 0;
    persp->foa = 0;
    break;
  }
    
  return persp;
}

int hri_get_default_camera_joint_no(HRI_AGENT_TYPE type)
{
  switch (type) {
    case HRI_JIDO1:
      return 14;
    case HRI_ACHILE:
      return 40;
    default:
      return 0;
  }
}

int hri_create_assign_default_manipulation(HRI_AGENTS * agents)
{
  int i;

  for(i=0; i<agents->robots_no; i++){
    agents->robots[i]->manip = hri_create_agent_manip(agents->robots[i]);
  }
  for(i=0; i<agents->humans_no; i++){
    agents->humans[i]->manip = hri_create_agent_manip(agents->humans[i]);
  }
  return TRUE;
}

HRI_NAVIG * hri_create_agent_navig(HRI_AGENT * agent)
{
  HRI_NAVIG * navig = NULL;

  navig = MY_ALLOC(HRI_NAVIG,1);

  navig->btset_initialized = FALSE;
#ifdef HRI_PLANNER
  navig->btset = hri_bt_create_bitmaps();
#endif
  return navig;
}


HRI_MANIP * hri_create_agent_manip(HRI_AGENT * agent)
{
  HRI_MANIP * manip = NULL;
  int res;

  manip = MY_ALLOC(HRI_MANIP,1);

  manip->gik = NULL;
  manip->tasklist = NULL;
  manip->tasklist_no = 0;

  res = hri_create_fill_agent_default_manip_tasks(&manip->tasklist, &manip->tasklist_no, agent->type);

  if(res == FALSE){
    PrintError(("Fill default task failed"));
    return NULL;
  }

  manip->gik = hri_gik_create_gik();

  return manip;
}

int hri_create_fill_agent_default_manip_tasks(GIK_TASK ** tasklist, int * tasklist_no, HRI_AGENT_TYPE type)
{

  switch (type) {
    case HRI_HRP214:
      // TODO: Test if alloc works in a case statement
      *tasklist_no = 7;
      *tasklist = MY_ALLOC(GIK_TASK, *tasklist_no);


      // These values are the default joint numbers for gik task
      // It is completely dependent to the robot's macro
      // TODO: add a field to p3d to load them automatically

      (*tasklist)[0].type = GIK_LOOK;
      (*tasklist)[0].default_joints[0] = 16;
      (*tasklist)[0].default_joints[1] = 17;
      (*tasklist)[0].default_joints[2] = 18;
      (*tasklist)[0].active_joint = 18; /* active joint */
      (*tasklist)[0].default_joints_no = 3;

      (*tasklist)[1].type = GIK_RATREACH;
      (*tasklist)[1].default_joints[0] = 14;
      (*tasklist)[1].default_joints[1] = 15;
      (*tasklist)[1].default_joints[2] = 19;
      (*tasklist)[1].default_joints[3] = 20;
      (*tasklist)[1].default_joints[4] = 21;
      (*tasklist)[1].default_joints[5] = 22;
      (*tasklist)[1].default_joints[6] = 23;
      (*tasklist)[1].default_joints[7] = 24;
      (*tasklist)[1].active_joint = 48; /* active joint */
      (*tasklist)[1].default_joints_no = 8;

      (*tasklist)[2].type = GIK_LATREACH;
      (*tasklist)[2].default_joints[0] = 14;
      (*tasklist)[2].default_joints[1] = 15;
      (*tasklist)[2].default_joints[2] = 32;
      (*tasklist)[2].default_joints[3] = 33;
      (*tasklist)[2].default_joints[4] = 34;
      (*tasklist)[2].default_joints[5] = 35;
      (*tasklist)[2].default_joints[6] = 36;
      (*tasklist)[2].default_joints[7] = 37;
      (*tasklist)[2].active_joint = 47; /* active joint */
      (*tasklist)[2].default_joints_no = 8;

      (*tasklist)[3].type = GIK_RAPOINT;
      (*tasklist)[3].default_joints[0] = 19;
      (*tasklist)[3].default_joints[1] = 20;
      (*tasklist)[3].default_joints[2] = 21;
      (*tasklist)[3].default_joints[3] = 22;
      (*tasklist)[3].default_joints[4] = 23;
      (*tasklist)[3].default_joints[5] = 24;
      (*tasklist)[3].active_joint = 46; /* active joint */
      (*tasklist)[3].default_joints_no = 6;

      (*tasklist)[4].type = GIK_LAPOINT;
      (*tasklist)[4].default_joints[0] = 32;
      (*tasklist)[4].default_joints[1] = 33;
      (*tasklist)[4].default_joints[2] = 34;
      (*tasklist)[4].default_joints[3] = 35;
      (*tasklist)[4].default_joints[4] = 36;
      (*tasklist)[4].default_joints[5] = 37;
      (*tasklist)[4].active_joint = 45; /* active joint */
      (*tasklist)[4].default_joints_no = 6;

      (*tasklist)[5].type = GIK_RAREACH;
      (*tasklist)[5].default_joints[0] = 19;
      (*tasklist)[5].default_joints[1] = 20;
      (*tasklist)[5].default_joints[2] = 21;
      (*tasklist)[5].default_joints[3] = 22;
      (*tasklist)[5].default_joints[4] = 23;
      (*tasklist)[5].default_joints[5] = 24;
      (*tasklist)[5].active_joint = 48; /* active joint */
      (*tasklist)[5].default_joints_no = 6;

      (*tasklist)[6].type = GIK_LAREACH;
      (*tasklist)[6].default_joints[0] = 32;
      (*tasklist)[6].default_joints[1] = 33;
      (*tasklist)[6].default_joints[2] = 34;
      (*tasklist)[6].default_joints[3] = 35;
      (*tasklist)[6].default_joints[4] = 36;
      (*tasklist)[6].default_joints[5] = 37;
      (*tasklist)[6].active_joint = 47; /* active joint */
      (*tasklist)[6].default_joints_no = 6;

      return TRUE;

    case HRI_JIDO1:
      *tasklist_no = 3;
      *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);
      
      (*tasklist)[0].type = GIK_LOOK;
      (*tasklist)[0].default_joints[0] = 2;
      (*tasklist)[0].default_joints[1] = 3;
      (*tasklist)[0].active_joint = 14; /* active joint */
      (*tasklist)[0].default_joints_no = 2;

      (*tasklist)[1].type = GIK_LATREACH;
      (*tasklist)[1].default_joints[0] = 5;
      (*tasklist)[1].default_joints[1] = 6;
      (*tasklist)[1].default_joints[2] = 7;
      (*tasklist)[1].default_joints[3] = 8;
      (*tasklist)[1].default_joints[4] = 9;
      (*tasklist)[1].default_joints[5] = 10;
      (*tasklist)[1].active_joint = 16; /* active joint */
      (*tasklist)[1].default_joints_no = 6;
      
      (*tasklist)[2].type = GIK_RATREACH;
      (*tasklist)[2].default_joints[0] = 5;
      (*tasklist)[2].default_joints[1] = 6;
      (*tasklist)[2].default_joints[2] = 7;
      (*tasklist)[2].default_joints[3] = 8;
      (*tasklist)[2].default_joints[4] = 9;
      (*tasklist)[2].default_joints[5] = 10;
      (*tasklist)[2].active_joint = 16; /* active joint */
      (*tasklist)[2].default_joints_no = 6;
      
      return TRUE;

    case HRI_SUPERMAN:
      *tasklist_no = 2;
      *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);

      (*tasklist)[0].type = GIK_LATREACH;
      (*tasklist)[0].default_joints[0] = 5;
      (*tasklist)[0].default_joints[1] = 6;
      (*tasklist)[0].default_joints[2] = 7;
      (*tasklist)[0].default_joints[3] = 8;
      (*tasklist)[0].default_joints[4] = 9;
      (*tasklist)[0].default_joints[5] = 10;
      (*tasklist)[0].default_joints[6] = 14;
      (*tasklist)[0].default_joints[7] = 15;
      (*tasklist)[0].default_joints[8] = 16;
      (*tasklist)[0].default_joints[9] = 20;
      (*tasklist)[0].default_joints[10] = 21;
      (*tasklist)[0].default_joints[11] = 22;
      (*tasklist)[0].default_joints[12] = 26;
      (*tasklist)[0].active_joint = 26; /* active joint */
      (*tasklist)[0].default_joints_no = 13;

      (*tasklist)[1].type = GIK_LAREACH;
      (*tasklist)[1].default_joints[0] = 8;
      (*tasklist)[1].default_joints[1] = 9;
      (*tasklist)[1].default_joints[2] = 10;
      (*tasklist)[1].default_joints[3] = 14;
      (*tasklist)[1].default_joints[4] = 15;
      (*tasklist)[1].default_joints[5] = 16;
      (*tasklist)[1].default_joints[6] = 20;
      (*tasklist)[1].default_joints[7] = 21;
      (*tasklist)[1].default_joints[8] = 22;
      (*tasklist)[1].default_joints[9] = 26;
      (*tasklist)[1].active_joint = 26; /* active joint */
      (*tasklist)[1].default_joints_no = 10;


      return TRUE;

    case HRI_ACHILE:
      *tasklist_no = 4;
      *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);
      
      (*tasklist)[0].type = GIK_RATREACH;
      (*tasklist)[0].default_joints[0] = 3;
      (*tasklist)[0].default_joints[1] = 4;
      (*tasklist)[0].default_joints[2] = 8;
      (*tasklist)[0].default_joints[3] = 9;
      (*tasklist)[0].default_joints[4] = 10;
      (*tasklist)[0].default_joints[5] = 11;
      (*tasklist)[0].default_joints[6] = 12;
      (*tasklist)[0].default_joints[7] = 13;
      (*tasklist)[0].default_joints[8] = 14;
      (*tasklist)[0].active_joint = 36; /* active joint */
      (*tasklist)[0].default_joints_no = 9;

      (*tasklist)[1].type = GIK_LATREACH;
      (*tasklist)[1].default_joints[0] = 3;
      (*tasklist)[1].default_joints[1] = 4;
      (*tasklist)[1].default_joints[2] = 15;
      (*tasklist)[1].default_joints[3] = 16;
      (*tasklist)[1].default_joints[4] = 17;
      (*tasklist)[1].default_joints[5] = 18;
      (*tasklist)[1].default_joints[6] = 19;
      (*tasklist)[1].default_joints[7] = 20;
      (*tasklist)[1].default_joints[8] = 21;
      (*tasklist)[1].active_joint = 37; /* active joint */
      (*tasklist)[1].default_joints_no = 9;

      (*tasklist)[2].type = GIK_RAREACH;
      (*tasklist)[2].default_joints[0] = 8;
      (*tasklist)[2].default_joints[1] = 9;
      (*tasklist)[2].default_joints[2] = 10;
      (*tasklist)[2].default_joints[3] = 11;
      (*tasklist)[2].default_joints[4] = 12;
      (*tasklist)[2].default_joints[5] = 13;
      (*tasklist)[2].default_joints[6] = 14;
      (*tasklist)[2].active_joint = 36; /* active joint */
      (*tasklist)[2].default_joints_no = 7;

      (*tasklist)[3].type = GIK_LAREACH;
      (*tasklist)[3].default_joints[0] = 15;
      (*tasklist)[3].default_joints[1] = 16;
      (*tasklist)[3].default_joints[2] = 17;
      (*tasklist)[3].default_joints[3] = 18;
      (*tasklist)[3].default_joints[4] = 19;
      (*tasklist)[3].default_joints[5] = 20;
      (*tasklist)[3].default_joints[6] = 21;
      (*tasklist)[3].active_joint = 37; /* active joint */
      (*tasklist)[3].default_joints_no = 7;

      return TRUE;

    case HRI_BERT:
      *tasklist_no = 2;
      *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);

      (*tasklist)[0].type = GIK_RATREACH;
      (*tasklist)[0].default_joints[0] = 2;
      (*tasklist)[0].default_joints[1] = 3;
      (*tasklist)[0].default_joints[2] = 6;
      (*tasklist)[0].default_joints[3] = 7;
      (*tasklist)[0].default_joints[4] = 8;
      (*tasklist)[0].default_joints[5] = 9;
      (*tasklist)[0].active_joint = 15; /* active joint */
      (*tasklist)[0].default_joints_no = 6;

      (*tasklist)[1].type = GIK_LATREACH;
      (*tasklist)[1].default_joints[0] = 2;
      (*tasklist)[1].default_joints[1] = 3;
      (*tasklist)[1].default_joints[2] = 10;
      (*tasklist)[1].default_joints[3] = 11;
      (*tasklist)[1].default_joints[4] = 12;
      (*tasklist)[1].default_joints[5] = 13;
      (*tasklist)[1].active_joint = 14; /* active joint */
      (*tasklist)[1].default_joints_no = 6;

      return TRUE;
		  
		  
	  case HRI_MOBILE_JUSTIN:
	  *tasklist_no = 2;
	  *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);
		  
	  (*tasklist)[0].type = GIK_RAREACH;
	  (*tasklist)[0].default_joints[0] = 9;
	  (*tasklist)[0].default_joints[1] = 10;
	  (*tasklist)[0].default_joints[2] = 11;
	  (*tasklist)[0].default_joints[3] = 12;
	  (*tasklist)[0].default_joints[4] = 13;
	  (*tasklist)[0].default_joints[5] = 14;
	  (*tasklist)[0].default_joints[6] = 15;
	  (*tasklist)[0].default_joints[7] = 16;
	  (*tasklist)[0].active_joint = 25; /* active joint */
	  (*tasklist)[0].default_joints_no = 8;
		  
	  (*tasklist)[1].type = GIK_LAREACH;
	  (*tasklist)[1].default_joints[0] = 17;
	  (*tasklist)[1].default_joints[1] = 18;
	  (*tasklist)[1].default_joints[2] = 19;
	  (*tasklist)[1].default_joints[3] = 20;
	  (*tasklist)[1].default_joints[4] = 21;
	  (*tasklist)[1].default_joints[5] = 22;
	  (*tasklist)[1].default_joints[6] = 23;
	  (*tasklist)[1].default_joints[7] = 24;
	  (*tasklist)[1].active_joint = 27; /* active joint */
	  (*tasklist)[1].default_joints_no = 8;
		  
	return TRUE;

    default:
      PrintError(("Agent type unknown\n"));
      return FALSE;

  }

  return FALSE;
}


int hri_agent_single_task_manip_move(HRI_AGENT * agent, HRI_GIK_TASK_TYPE type, p3d_vector3 * goalCoord, configPt *q)
{
  int i;
  //configPt savedq;
  HRI_MANIP * manip = NULL;

  if(agent == NULL){
    PrintError(("Cannot initialize manip. Agent == NULL\n"));
    return FALSE;
  }

  if(agent->manip == NULL){
    PrintError(("Cannot initalize manip. Agent->manip == NULL\n"));
    return FALSE;
  }
  else{
    manip = agent->manip;
  }

  if(manip->gik == NULL){
    manip->gik = hri_gik_create_gik();
  }

  //savedq = p3d_get_robot_config(agent->robotPt);
  //p3d_copy_config_into(agent->robotPt, savedq, q);

  if(manip->activetasks_no == 1){
    // Maybe gik is already well initialized
    if(manip->tasklist[manip->activetasks[0]].type == type){
      // Gik is well initialized - This test is only by task type number and not by joints
      if(manip->gik->GIKInitialized){
        if(!hri_gik_compute(agent->robotPt, manip->gik, 500, 0.04, FALSE, 0, goalCoord, NULL, q, NULL)){
          return FALSE;
        }
        else{
          return TRUE;
        }
      }
    }
  }

  // Initialize gik
  if(manip->gik->GIKInitialized){
    PrintWarning(("Remove existing GIK\n"));
    hri_gik_destroy_gik_data(manip->gik);
    hri_gik_uninitialize_gik(manip->gik);
  }

  for(i=0; i<manip->tasklist_no; i++){
    if(manip->tasklist[i].type == type){
      manip->activetasks[0] = i;
      manip->activetasks_no = 1;
      break;
    }
  }
  if(i == manip->tasklist_no){
    PrintError(("Agent does not have the task capability,\n"));
    return FALSE;
  }

  if(!hri_gik_initialize_gik(manip->gik,agent->robotPt,FALSE,
                             manip->tasklist[manip->activetasks[0]].default_joints_no))
    return FALSE;

  if(!hri_gik_add_task(manip->gik,3,manip->tasklist[manip->activetasks[0]].default_joints_no,1,
                       manip->tasklist[manip->activetasks[0]].default_joints,
                       manip->tasklist[manip->activetasks[0]].active_joint))
    return FALSE;


  if(!hri_gik_compute(agent->robotPt, manip->gik, 500, 0.02, FALSE, 0, goalCoord, NULL, q, NULL))
    return FALSE;

  return TRUE;
}

hri_shared_zone zone[500];
int shared_zone_l = 0;
int SWITCH_TO_GREEN = FALSE;


int g3d_hri_display_shared_zone()
{
  int i;

  for(i=0; i<shared_zone_l; i++){
    if(zone[i].value == 1){
      g3d_draw_a_box(zone[i].x-0.02, zone[i].x+0.02, zone[i].y-0.02, zone[i].y+0.02, zone[i].z-0.02, zone[i].z+0.02, Green, FALSE);
    }
    else{
      if(!SWITCH_TO_GREEN){
        if(zone[i].value == 0){
          g3d_draw_a_box(zone[i].x-0.02, zone[i].x+0.02, zone[i].y-0.02, zone[i].y+0.02, zone[i].z-0.02, zone[i].z+0.02, Red, FALSE);
        }
        else {
          g3d_draw_a_box(zone[i].x-0.02, zone[i].x+0.02, zone[i].y-0.02, zone[i].y+0.02, zone[i].z-0.02, zone[i].z+0.02, Yellow, FALSE);
        }
      }
    }
  }
  return TRUE;
}

static int hri_compute_leg_angles(double dist1, double dist2, double height, double *hip, double *knee, double *ankle)
{
  if( dist1+dist2 <= height ) {
    *hip = 0;
    *ankle = 0;
    *knee = 0;
  }
  else {
    *hip = acos( (SQR(dist1)+SQR(height)-SQR(dist2)) / (2*dist1*height) );
    *ankle = acos( (SQR(dist2)+SQR(height)-SQR(dist1)) / (2*dist2*height) );
    *knee = *hip + *ankle;
  }
  return TRUE;
}

int hri_agent_compute_posture(HRI_AGENT * agent, double head_height, double height_threshold, configPt q)
{
  double hiptoknee_dist, kneetoankle_dist, necktobase_dist, hiptoground_dist; 
  double basetohip_dist, headtoneck_dist, hiptoankle_dist, ankletoground_dist;
  double hip_angle, knee_angle, ankle_angle,hz_hip_angle;
  
  if(agent == NULL) return FALSE;
    
  // Only supports ACHILE human model
  // everything else return FALSE
  
  if(agent->type == HRI_ACHILE) {
         
    headtoneck_dist = (agent->robotPt->joints[6]->o->BB.zmax + agent->robotPt->joints[6]->o->BB.zmin) / 2 - agent->robotPt->joints[5]->abs_pos[2][3];
    
    necktobase_dist = agent->robotPt->joints[5]->abs_pos[2][3]-agent->robotPt->joints[1]->abs_pos[2][3];
    
    hiptoknee_dist = DISTANCE2D(agent->robotPt->joints[23]->abs_pos[0][3],
                                agent->robotPt->joints[23]->abs_pos[2][3],
                                agent->robotPt->joints[25]->abs_pos[0][3],                                 
                                agent->robotPt->joints[25]->abs_pos[2][3]); // Constant -> 0.47
    kneetoankle_dist = DISTANCE2D(agent->robotPt->joints[25]->abs_pos[0][3],                                   
                                  agent->robotPt->joints[25]->abs_pos[2][3],
                                  agent->robotPt->joints[27]->abs_pos[0][3],                                   
                                  agent->robotPt->joints[27]->abs_pos[2][3]); // Constant -> 0.39
    basetohip_dist = agent->robotPt->joints[1]->abs_pos[2][3]-agent->robotPt->joints[23]->abs_pos[2][3];
    
    hiptoground_dist =  head_height - headtoneck_dist - necktobase_dist - basetohip_dist;
    
    ankletoground_dist = 0.08; // Fixed distance in ACHILE model
    
    if(head_height > height_threshold) {
      hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, hiptoground_dist - ankletoground_dist, 
                             &hip_angle, &knee_angle, &ankle_angle);      
      
      q[8] = head_height - headtoneck_dist - necktobase_dist;
      q[agent->robotPt->joints[23]->index_dof] = -hip_angle;
      q[agent->robotPt->joints[25]->index_dof] = knee_angle;
      q[agent->robotPt->joints[27]->index_dof] = -ankle_angle;
      q[agent->robotPt->joints[30]->index_dof] = -hip_angle;
      q[agent->robotPt->joints[32]->index_dof] = knee_angle;
      q[agent->robotPt->joints[34]->index_dof] = -ankle_angle;
      
      agent->actual_state = BT_STANDING;
    } 
    else {
      
      hiptoankle_dist = sqrt(SQR(0.4)+SQR(hiptoground_dist-ankletoground_dist));
      
      hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, 
                             hiptoankle_dist, 
                             &hip_angle, &knee_angle, &ankle_angle);      
      
      hz_hip_angle = acos( (SQR(hiptoknee_dist)+SQR(hiptoground_dist-ankletoground_dist)-SQR(kneetoankle_dist)) / 
                          (2*hiptoknee_dist*(hiptoground_dist-ankletoground_dist)) );
      q[8] = head_height - headtoneck_dist - necktobase_dist;
      
      hip_angle = hip_angle+hz_hip_angle;
      ankle_angle = (M_PI_2-(M_PI - ankle_angle - atan2(hiptoground_dist-ankletoground_dist,0.4)));
      
      
      if (-hip_angle > agent->robotPt->joints[23]->dof_data->vmax ||
          -hip_angle < agent->robotPt->joints[23]->dof_data->vmin ||
          knee_angle > agent->robotPt->joints[25]->dof_data->vmax ||
          knee_angle < agent->robotPt->joints[25]->dof_data->vmin ||
          -ankle_angle > agent->robotPt->joints[27]->dof_data->vmax ||
          -ankle_angle < agent->robotPt->joints[27]->dof_data->vmin ) {
        q[agent->robotPt->joints[23]->index_dof] = -M_PI_2;
        q[agent->robotPt->joints[25]->index_dof] = M_PI_2;
        q[agent->robotPt->joints[27]->index_dof] = 0;
        q[agent->robotPt->joints[30]->index_dof] = -M_PI_2;
        q[agent->robotPt->joints[32]->index_dof] = M_PI_2;
        q[agent->robotPt->joints[34]->index_dof] = 0;
      }
      else {
        q[agent->robotPt->joints[23]->index_dof] = -hip_angle;
        q[agent->robotPt->joints[25]->index_dof] = knee_angle;
        q[agent->robotPt->joints[27]->index_dof] = -ankle_angle;
        q[agent->robotPt->joints[30]->index_dof] = -hip_angle;
        q[agent->robotPt->joints[32]->index_dof] = knee_angle;
        q[agent->robotPt->joints[34]->index_dof] = -ankle_angle;
      }
      agent->actual_state = BT_SITTING;
    }    
    return TRUE;
  }
  else {
    printf("In %s:%d, Trying to compute the posture of an unsupported robot\n",__FILE__,__LINE__);
    return FALSE;
  }
}

int hri_agent_load_default_arm_posture(HRI_AGENT * agent, configPt q)
{
  int first_rshoulder_dof,first_lshoulder_dof;
  
  if(agent == NULL) return FALSE;
  
  if(agent->type == HRI_ACHILE) {
    first_rshoulder_dof = agent->robotPt->joints[8]->index_dof;
    q[first_rshoulder_dof]   = DTOR( 80);
    q[first_rshoulder_dof+1] = DTOR(  0);
    q[first_rshoulder_dof+2] = DTOR(-20);
    q[first_rshoulder_dof+3] = DTOR( 45);
    q[first_rshoulder_dof+4] = DTOR(  0);
    q[first_rshoulder_dof+5] = DTOR(  0);
    q[first_rshoulder_dof+6] = DTOR(  0);
    
    first_lshoulder_dof = agent->robotPt->joints[15]->index_dof;
    q[first_lshoulder_dof]   = DTOR(-80);
    q[first_lshoulder_dof+1] = DTOR(  0);
    q[first_lshoulder_dof+2] = DTOR( 20);
    q[first_lshoulder_dof+3] = DTOR(-45);
    q[first_lshoulder_dof+4] = DTOR(  0);
    q[first_lshoulder_dof+5] = DTOR(  0);
    q[first_lshoulder_dof+6] = DTOR(  0);
    
    return TRUE;
  }
  else {
    printf("In %s:%d, Trying to load default arm posture to a unsupported robot\n",__FILE__,__LINE__);
    return FALSE;
  }
  
}