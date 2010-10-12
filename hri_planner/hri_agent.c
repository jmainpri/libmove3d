#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"

#include "proto/hri_agent_proto.h"
#include "proto/hri_gik_proto.h"

// The global agents are not included 
// when compiling minimal (only hri_agent.c and hri_gik.c)
#if defined( HRI_GENERALIZED_IK ) && !defined( HRI_PLANNER )
HRI_AGENTS * GLOBAL_AGENTS = NULL;
#endif

int hri_assign_global_agents(HRI_AGENTS *agents)
{
  if(GLOBAL_AGENTS != NULL){
    printf("Global agents is not null. Erasing the existing data\n");
    if(!hri_destroy_agents(GLOBAL_AGENTS))  return FALSE;
  }
  GLOBAL_AGENTS = agents;
  return TRUE;
}

HRI_AGENT* hri_assign_source_agent(char *agent_name, HRI_AGENTS *agents)
{
  int i;
  
  if (agents == NULL) {
    printf("%s:%d - Cannot assign source agent\n",__FILE__, __LINE__);
    return NULL;
  }
  else {
    for (i=0; i<agents->all_agents_no; i++) {
      if (strcasestr(agents->all_agents[i]->robotPt->name, agent_name)) {
        agents->source_agent_idx = i;
        return agents->all_agents[i];
      }
    }
  }
  return NULL;  
}

HRI_AGENTS * hri_create_agents()
{
  int i, i_r=0, i_h=0;
  HRI_AGENTS * agents;
  HRI_AGENT * new_agent;
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  agents = MY_ALLOC(HRI_AGENTS,1);
  agents->robots_no = 0;
  agents->humans_no = 0;
  agents->robots = NULL;
  agents->humans = NULL;

  for(i=0; i<env->nr; i++){
    if(strcasestr(env->robot[i]->name,"ROBOT")) {
      new_agent = hri_create_agent(env->robot[i]);
      if( new_agent != NULL) {
        agents->robots = MY_REALLOC(agents->robots, HRI_AGENT *, agents->robots_no, ++agents->robots_no);
        agents->robots[agents->robots_no-1] = new_agent;
      }
    }
    else {
      if(strcasestr(env->robot[i]->name,"HUMAN")) {
        new_agent = hri_create_agent(env->robot[i]);
        if( new_agent != NULL) {
          agents->humans = MY_REALLOC(agents->humans, HRI_AGENT *, agents->humans_no, ++agents->humans_no);
          agents->humans[agents->humans_no-1] = new_agent;
        }
      }
    }
  }
    
  agents->all_agents_no = agents->robots_no + agents->humans_no;
  agents->all_agents = MY_ALLOC(HRI_AGENT *, agents->all_agents_no);
  i = 0;
  for(i_r=0; i_r<agents->robots_no; i_r++, i++)
    agents->all_agents[i] = agents->robots[i_r];
  for(i_h=0; i_h<agents->humans_no; i_h++, i++)
    agents->all_agents[i] = agents->humans[i_h];
  
  return agents;
}

int hri_destroy_agents(HRI_AGENTS *agents)
{
  int i;
  int res = 1;
  
  if(agents == NULL)
    return TRUE;
  
  for (i=0; i<agents->all_agents_no; i++) {
    res = res && hri_destroy_agent(agents->all_agents[i]);
  }
  if (res) {
    MY_FREE(agents->robots, HRI_AGENT *,agents->robots_no);
    MY_FREE(agents->humans, HRI_AGENT *,agents->humans_no);
    MY_FREE(agents->all_agents, HRI_AGENT *,agents->all_agents_no);
    return TRUE;
  }
  else {
    return FALSE;
  }
}

HRI_AGENT * hri_create_agent(p3d_rob * robot)
{
  HRI_AGENT * hri_agent;

  hri_agent = MY_ALLOC(HRI_AGENT,1);
  
  if(strcasestr(robot->name,"SUPERMAN")) {
    hri_agent->type = HRI_SUPERMAN;
    hri_agent->is_human = TRUE;
  }
  else {
    if(strcasestr(robot->name,"ACHILE")) {
      hri_agent->type = HRI_ACHILE;
      hri_agent->is_human = TRUE;
    }
    else {
      if(strcasestr(robot->name,"TINMAN")) {
        hri_agent->type = HRI_TINMAN;
        hri_agent->is_human = TRUE;
      }
      else {
        if(strcasestr(robot->name,"JIDOKUKA")) {
          hri_agent->type = HRI_JIDOKUKA;
          hri_agent->is_human = FALSE;
        }
        else {
          if(strcasestr(robot->name,"PR2")) {
            hri_agent->type = HRI_PR2;
            hri_agent->is_human = FALSE;
          }
          else {
            if(strcasestr(robot->name,"JIDO")) {
              hri_agent->type = HRI_JIDO1;
              hri_agent->is_human = FALSE;
            }
          else {
            if(strcasestr(robot->name,"HRP2")) {
              hri_agent->type = HRI_HRP214;
              hri_agent->is_human = FALSE;
            }
            else {
              if(strcasestr(robot->name,"B21")) {
                hri_agent->type = HRI_B21;
                hri_agent->is_human = FALSE;
              }
              else {
                if(strcasestr(robot->name,"JUSTIN")) {
                  hri_agent->type = HRI_MOBILE_JUSTIN;
                  hri_agent->is_human = FALSE;
                }
                else {
                  if(strcasestr(robot->name,"BH")) {
                    hri_agent->type = HRI_BH;
                    hri_agent->is_human = FALSE;
                  }
                  else {
                    if(strcasestr(robot->name,"ICUB")) {
                      hri_agent->type = HRI_ICUB;
                      hri_agent->is_human = FALSE;
                    }
                    else {
                      if(strcasestr(robot->name,"BERT")) {
                        hri_agent->type = HRI_BERT;
                        hri_agent->is_human = FALSE;
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
    }
  }
  hri_agent->robotPt = robot;
  
  hri_agent->navig  = hri_create_agent_navig(hri_agent);
  hri_agent->manip = hri_create_agent_manip(hri_agent);
  hri_agent->perspective = hri_create_agent_perspective(hri_agent, robot->env);
  
  hri_agent->exists = FALSE;

  /* TODO: Fix proper state assignment */
  hri_agent->states_no = 0;
  hri_agent->actual_state = 0;
  hri_agent->state = NULL;
  
  return hri_agent;
}

int hri_destroy_agent(HRI_AGENT *agent)
{
  int res = 1;
  
  if ( agent == NULL )
    return TRUE;
  
  res = res && hri_destroy_agent_navig(agent->navig);
  res = res && hri_destroy_agent_manip(agent->manip);
  res = res && hri_destroy_agent_perspective(agent->perspective);
  
  return res;
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

HRI_PERSP * hri_create_agent_perspective(HRI_AGENT * agent, p3d_env *env)
{
  int i;
  HRI_PERSP *persp;
  
  persp = MY_ALLOC(HRI_PERSP, 1);
  
  switch (agent->type) {
    case HRI_JIDO1:
      persp->camjoint = agent->robotPt->joints[14];
      persp->fov = 60;
      persp->foa = 60;
      persp->tilt_jnt_idx = 3;
      persp->pan_jnt_idx  = 2;
      persp->pointjoint = agent->robotPt->joints[17];
      persp->point_tolerance = 20;      
      break;
    case HRI_HRP214:
      persp->camjoint = agent->robotPt->joints[49];
      persp->fov = 40;
      persp->foa = 40;
      persp->tilt_jnt_idx = 16;
      persp->pan_jnt_idx  = 17;
      persp->pointjoint = agent->robotPt->joints[48];
      persp->point_tolerance = 20;      
      break;
    case HRI_JIDOKUKA:
      persp->camjoint = agent->robotPt->joints[15];
      persp->fov = 60;
      persp->foa = 60;
      persp->tilt_jnt_idx = 3;
      persp->pan_jnt_idx  = 2;
      persp->pointjoint = agent->robotPt->joints[18];
      persp->point_tolerance = 20;      
      break;
    case HRI_PR2:
      persp->camjoint = agent->robotPt->joints[24]; 
      persp->fov = 120; //TODO: put the correct value
      persp->foa = 60; //TODO: put the correct value
      persp->tilt_jnt_idx = 4;
      persp->pan_jnt_idx  = 3;
      persp->pointjoint = agent->robotPt->joints[26];
      persp->point_tolerance = 20;   
      break;
    case HRI_ICUB:
      persp->camjoint = agent->robotPt->joints[34]; 
      persp->fov = 120; //TODO: put the correct value
      persp->foa = 60; //TODO: put the correct value
      persp->tilt_jnt_idx = 31;
      persp->pan_jnt_idx  = 33;
      persp->pointjoint = agent->robotPt->joints[35];
      persp->point_tolerance = 20;   
      break;
    case HRI_ACHILE:
      persp->camjoint = agent->robotPt->joints[42];
      persp->fov = 160;
      persp->foa = 30;
      persp->tilt_jnt_idx = 6;
      persp->pan_jnt_idx  = 5;
      persp->pointjoint = agent->robotPt->joints[36];
      persp->point_tolerance = 20;      
    break;
    case HRI_SUPERMAN:
      persp->camjoint = agent->robotPt->joints[1]; //TODO: put the correct value
      persp->fov = 160;
      persp->foa = 30;
      persp->tilt_jnt_idx = 55;
      persp->pan_jnt_idx  = 54;
      persp->pointjoint = agent->robotPt->joints[1]; //TODO: put the correct value
      persp->point_tolerance = 20;      
      break;
    default:
      persp->fov = 0;
      persp->foa = 0;
      break;
  }
  persp->currently_sees.vis_nb = env->nr;
  persp->currently_sees.vis = MY_ALLOC(HRI_VISIBILITY, persp->currently_sees.vis_nb);
  persp->currently_sees.vispl = MY_ALLOC(HRI_VISIBILITY_PLACEMENT, persp->currently_sees.vis_nb);
  for (i=0; i<persp->currently_sees.vis_nb; i++) {
    persp->currently_sees.vis[i] = HRI_INVISIBLE;
    persp->currently_sees.vispl[i] = HRI_OOF;
  }
  
  persp->enable_vision_draw = FALSE;
  persp->enable_pointing_draw = FALSE;
  persp->enable_visible_objects_draw = FALSE;
  
  return persp;
}

int hri_destroy_agent_perspective(HRI_PERSP *persp)
{  
  MY_FREE(persp->currently_sees.vis, HRI_VISIBILITY, persp->currently_sees.vis_nb);  
  MY_FREE(persp,HRI_PERSP,1);  
  
  return TRUE;
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
	
#if defined( HRI_GENERALIZED_IK ) && defined( HRI_PLANNER )
  navig->btset = hri_bt_create_bitmaps();
#else
	navig->btset = NULL;
#endif
	
  return navig;
}

int hri_destroy_agent_navig(HRI_NAVIG *navig)
{
#if defined( HRI_GENERALIZED_IK ) && defined( HRI_PLANNER )
  if(hri_bt_destroy_bitmapset(navig->btset)) {  
    MY_FREE(navig,HRI_NAVIG,1);
    return TRUE;
  }
  else {
    return FALSE;
  }
#else
	return TRUE;
#endif
}


HRI_MANIP * hri_create_agent_manip(HRI_AGENT * agent)
{
  HRI_MANIP * manip = NULL;
  int res;

  manip = MY_ALLOC(HRI_MANIP,1);

  manip->gik = NULL;
  manip->tasklist = NULL;
  manip->tasklist_no = 0;

  res = hri_create_fill_agent_default_manip_tasks(manip, &manip->tasklist, &manip->tasklist_no, agent->type);

  if(res == FALSE){
    PrintError(("Fill default task failed"));
    return NULL;
  }

  manip->gik = hri_gik_create_gik();

  return manip;
}

int hri_destroy_agent_manip(HRI_MANIP *manip)
{
  if(hri_gik_destroy_gik(manip->gik)) {  
    MY_FREE(manip,HRI_MANIP,1);  
    return TRUE;
  }
  else {
    return FALSE;
  }
}

int hri_create_fill_agent_default_manip_tasks(HRI_MANIP * manip, GIK_TASK ** tasklist, int * tasklist_no, HRI_AGENT_TYPE type)
{

  switch (type) {
    case HRI_HRP214:
      manip->type = TWO_ARMED;
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
      manip->type = ONE_ARMED;
      *tasklist_no = 5;
      *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);
      
      (*tasklist)[0].type = GIK_LOOK;
      (*tasklist)[0].default_joints[0] = 2;
      (*tasklist)[0].default_joints[1] = 3;
      (*tasklist)[0].default_joints[2] = 15;
      (*tasklist)[0].active_joint = 15; /* active joint */
      (*tasklist)[0].default_joints_no = 3;

      (*tasklist)[1].type = GIK_LATREACH;
      (*tasklist)[1].default_joints[0] = 5;
      (*tasklist)[1].default_joints[1] = 6;
      (*tasklist)[1].default_joints[2] = 7;
      (*tasklist)[1].default_joints[3] = 8;
      (*tasklist)[1].default_joints[4] = 9;
      (*tasklist)[1].default_joints[5] = 10;
      (*tasklist)[1].active_joint = 17; /* active joint */
      (*tasklist)[1].default_joints_no = 6;
      
      (*tasklist)[2].type = GIK_RATREACH;
      (*tasklist)[2].default_joints[0] = 5;
      (*tasklist)[2].default_joints[1] = 6;
      (*tasklist)[2].default_joints[2] = 7;
      (*tasklist)[2].default_joints[3] = 8;
      (*tasklist)[2].default_joints[4] = 9;
      (*tasklist)[2].default_joints[5] = 10;
      (*tasklist)[2].active_joint = 17; /* active joint */
      (*tasklist)[2].default_joints_no = 6;
      
      (*tasklist)[3].type = GIK_RAPOINT;
      (*tasklist)[3].default_joints[0] = 5;
      (*tasklist)[3].default_joints[1] = 6;
      (*tasklist)[3].default_joints[2] = 7;
      (*tasklist)[3].default_joints[3] = 8;
      (*tasklist)[3].default_joints[4] = 9;
      (*tasklist)[3].default_joints[5] = 10;
      (*tasklist)[3].active_joint = 16; /* active joint */
      (*tasklist)[3].default_joints_no = 6;
      
      (*tasklist)[4].type = GIK_LAPOINT;
      (*tasklist)[4].default_joints[0] = 5;
      (*tasklist)[4].default_joints[1] = 6;
      (*tasklist)[4].default_joints[2] = 7;
      (*tasklist)[4].default_joints[3] = 8;
      (*tasklist)[4].default_joints[4] = 9;
      (*tasklist)[4].default_joints[5] = 10;
      (*tasklist)[4].active_joint = 16; /* active joint */
      (*tasklist)[4].default_joints_no = 6;
      
      return TRUE;

    case HRI_JIDOKUKA:
      manip->type = ONE_ARMED;
      *tasklist_no = 5;
      *tasklist = MY_ALLOC(GIK_TASK,*tasklist_no);
      
      (*tasklist)[0].type = GIK_LOOK;
      (*tasklist)[0].default_joints[0] = 2;
      (*tasklist)[0].default_joints[1] = 3;
      (*tasklist)[0].default_joints[2] = 16;
      (*tasklist)[0].active_joint = 16; /* active joint */
      (*tasklist)[0].default_joints_no = 3;
      
      (*tasklist)[1].type = GIK_LATREACH;
      (*tasklist)[1].default_joints[0] = 5;
      (*tasklist)[1].default_joints[1] = 6;
      (*tasklist)[1].default_joints[2] = 7;
      (*tasklist)[1].default_joints[3] = 8;
      (*tasklist)[1].default_joints[4] = 9;
      (*tasklist)[1].default_joints[5] = 10;
      (*tasklist)[1].default_joints[6] = 11;
      (*tasklist)[1].active_joint = 18; /* active joint */
      (*tasklist)[1].default_joints_no = 7;
      
      (*tasklist)[2].type = GIK_RATREACH;
      (*tasklist)[2].default_joints[0] = 5;
      (*tasklist)[2].default_joints[1] = 6;
      (*tasklist)[2].default_joints[2] = 7;
      (*tasklist)[2].default_joints[3] = 8;
      (*tasklist)[2].default_joints[4] = 9;
      (*tasklist)[2].default_joints[5] = 10;
      (*tasklist)[2].default_joints[6] = 11;
      (*tasklist)[2].active_joint = 18; /* active joint */
      (*tasklist)[2].default_joints_no = 7;
      
      (*tasklist)[3].type = GIK_RAPOINT;
      (*tasklist)[3].default_joints[0] = 5;
      (*tasklist)[3].default_joints[1] = 6;
      (*tasklist)[3].default_joints[2] = 7;
      (*tasklist)[3].default_joints[3] = 8;
      (*tasklist)[3].default_joints[4] = 9;
      (*tasklist)[3].default_joints[5] = 10;
      (*tasklist)[3].default_joints[6] = 11;
      (*tasklist)[3].active_joint = 17; /* active joint */
      (*tasklist)[3].default_joints_no = 7;
      
      (*tasklist)[4].type = GIK_LAPOINT;
      (*tasklist)[4].default_joints[0] = 5;
      (*tasklist)[4].default_joints[1] = 6;
      (*tasklist)[4].default_joints[2] = 7;
      (*tasklist)[4].default_joints[3] = 8;
      (*tasklist)[4].default_joints[4] = 9;
      (*tasklist)[4].default_joints[5] = 10;
      (*tasklist)[4].default_joints[6] = 11;
      (*tasklist)[4].active_joint = 17; /* active joint */
      (*tasklist)[4].default_joints_no = 7;
      
      return TRUE;
     
    case HRI_PR2:
      manip->type = TWO_ARMED;
      *tasklist_no = 5;
      *tasklist = MY_ALLOC(GIK_TASK, *tasklist_no);
      
      (*tasklist)[0].type = GIK_LOOK;
      (*tasklist)[0].default_joints[0] = 3;
      (*tasklist)[0].default_joints[1] = 4;
      (*tasklist)[0].default_joints[2] = 27;
      (*tasklist)[0].active_joint = 27; /* active joint */
      (*tasklist)[0].default_joints_no = 3;
      
      (*tasklist)[1].type = GIK_RAPOINT;
      (*tasklist)[1].default_joints[0] = 15;
      (*tasklist)[1].default_joints[1] = 16;
      (*tasklist)[1].default_joints[2] = 17;
      (*tasklist)[1].default_joints[3] = 18;
      (*tasklist)[1].default_joints[4] = 19;
      (*tasklist)[1].default_joints[5] = 20;
      (*tasklist)[1].default_joints[6] = 21;
      (*tasklist)[1].default_joints[7] = 29;
      (*tasklist)[1].active_joint = 29; /* active joint */
      (*tasklist)[1].default_joints_no = 8;
      
      (*tasklist)[2].type = GIK_LAPOINT;
      (*tasklist)[2].default_joints[0] = 5;
      (*tasklist)[2].default_joints[1] = 6;
      (*tasklist)[2].default_joints[2] = 7;
      (*tasklist)[2].default_joints[3] = 8;
      (*tasklist)[2].default_joints[4] = 9;
      (*tasklist)[2].default_joints[5] = 10;
      (*tasklist)[2].default_joints[6] = 11;
      (*tasklist)[2].default_joints[7] = 28;
      (*tasklist)[2].active_joint = 28; /* active joint */
      (*tasklist)[2].default_joints_no = 8;
      
      (*tasklist)[3].type = GIK_RAREACH;
      (*tasklist)[3].default_joints[0] = 15;
      (*tasklist)[3].default_joints[1] = 16;
      (*tasklist)[3].default_joints[2] = 17;
      (*tasklist)[3].default_joints[3] = 18;
      (*tasklist)[3].default_joints[4] = 19;
      (*tasklist)[3].default_joints[5] = 20;
      (*tasklist)[3].default_joints[6] = 21;
      (*tasklist)[3].active_joint = 26; /* active joint */
      (*tasklist)[3].default_joints_no = 7;
      
      (*tasklist)[4].type = GIK_LAREACH;
      (*tasklist)[4].default_joints[0] = 5;
      (*tasklist)[4].default_joints[1] = 6;
      (*tasklist)[4].default_joints[2] = 7;
      (*tasklist)[4].default_joints[3] = 8;
      (*tasklist)[4].default_joints[4] = 9;
      (*tasklist)[4].default_joints[5] = 10;
      (*tasklist)[4].default_joints[6] = 11;
      (*tasklist)[4].active_joint = 25; /* active joint */
      (*tasklist)[4].default_joints_no = 7;
      
      return TRUE;
      
    case HRI_ICUB:
      manip->type = TWO_ARMED;
      *tasklist_no = 4;
      *tasklist = MY_ALLOC(GIK_TASK, *tasklist_no);
      
      (*tasklist)[0].type = GIK_RAREACH;
      (*tasklist)[0].default_joints[0] = 17;
      (*tasklist)[0].default_joints[1] = 18;
      (*tasklist)[0].default_joints[2] = 19;
      (*tasklist)[0].default_joints[3] = 20;
      (*tasklist)[0].default_joints[4] = 21;
      (*tasklist)[0].default_joints[5] = 22;
      (*tasklist)[0].default_joints[6] = 23;
      (*tasklist)[0].active_joint = 36; /* active joint */
      (*tasklist)[0].default_joints_no = 7;
      
      (*tasklist)[1].type = GIK_LAREACH;
      (*tasklist)[1].default_joints[0] = 24;
      (*tasklist)[1].default_joints[1] = 25;
      (*tasklist)[1].default_joints[2] = 26;
      (*tasklist)[1].default_joints[3] = 27;
      (*tasklist)[1].default_joints[4] = 28;
      (*tasklist)[1].default_joints[5] = 29;
      (*tasklist)[1].default_joints[6] = 30;
      (*tasklist)[1].active_joint = 35; /* active joint */
      (*tasklist)[1].default_joints_no = 7;
      
      (*tasklist)[2].type = GIK_RATREACH;
      (*tasklist)[2].default_joints[0] = 14;
      (*tasklist)[2].default_joints[1] = 15;
      (*tasklist)[2].default_joints[2] = 16;
      (*tasklist)[2].default_joints[3] = 17;
      (*tasklist)[2].default_joints[4] = 18;
      (*tasklist)[2].default_joints[5] = 19;
      (*tasklist)[2].default_joints[6] = 20;
      (*tasklist)[2].default_joints[7] = 21;
      (*tasklist)[2].default_joints[8] = 22;
      (*tasklist)[2].default_joints[9] = 23;
      (*tasklist)[2].active_joint = 36; /* active joint */
      (*tasklist)[2].default_joints_no = 10;
      
      (*tasklist)[3].type = GIK_LATREACH;
      (*tasklist)[3].default_joints[0] = 14;
      (*tasklist)[3].default_joints[1] = 15;
      (*tasklist)[3].default_joints[2] = 16;
      (*tasklist)[3].default_joints[3] = 24;
      (*tasklist)[3].default_joints[4] = 25;
      (*tasklist)[3].default_joints[5] = 26;
      (*tasklist)[3].default_joints[6] = 27;
      (*tasklist)[3].default_joints[7] = 28;
      (*tasklist)[3].default_joints[8] = 29;
      (*tasklist)[3].default_joints[9] = 30;
      (*tasklist)[3].active_joint = 35; /* active joint */
      (*tasklist)[3].default_joints_no = 10;
      
      return TRUE;
      
    case HRI_SUPERMAN:
      manip->type = TWO_ARMED;
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
      manip->type = TWO_ARMED;
      *tasklist_no = 9;
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

      (*tasklist)[4].type = GIK_RAWREACH;
      (*tasklist)[4].default_joints[0] = 8;
      (*tasklist)[4].default_joints[1] = 9;
      (*tasklist)[4].default_joints[2] = 10;
      (*tasklist)[4].default_joints[3] = 11;
      (*tasklist)[4].default_joints[4] = 12;
      (*tasklist)[4].default_joints[5] = 13;
      (*tasklist)[4].default_joints[6] = 14;
      (*tasklist)[4].active_joint = 14; /* active joint */
      (*tasklist)[4].default_joints_no = 7;

      (*tasklist)[5].type = GIK_LAWREACH;
      (*tasklist)[5].default_joints[0] = 15;
      (*tasklist)[5].default_joints[1] = 16;
      (*tasklist)[5].default_joints[2] = 17;
      (*tasklist)[5].default_joints[3] = 18;
      (*tasklist)[5].default_joints[4] = 19;
      (*tasklist)[5].default_joints[5] = 20;
      (*tasklist)[5].default_joints[6] = 21;
      (*tasklist)[5].active_joint = 21; /* active joint */
      (*tasklist)[5].default_joints_no = 7;
      
      (*tasklist)[6].type = GIK_LOOK;
      (*tasklist)[6].default_joints[0] = 3;
      (*tasklist)[6].default_joints[1] = 4;
      (*tasklist)[6].default_joints[2] = 5;
      (*tasklist)[6].default_joints[3] = 6;
      (*tasklist)[6].default_joints[4] = 43;
      (*tasklist)[6].active_joint = 43; /* active joint */
      (*tasklist)[6].default_joints_no = 5;
           
      (*tasklist)[7].type = GIK_RAPOINT;
      (*tasklist)[7].default_joints[0] = 8;
      (*tasklist)[7].default_joints[1] = 9;
      (*tasklist)[7].default_joints[2] = 10;
      (*tasklist)[7].default_joints[3] = 11;
      (*tasklist)[7].default_joints[4] = 12;
      (*tasklist)[7].default_joints[5] = 13;
      (*tasklist)[7].default_joints[6] = 14;
      (*tasklist)[7].default_joints[7] = 38;
      (*tasklist)[7].active_joint = 38; /* active joint */
      (*tasklist)[7].default_joints_no = 8;
      
      (*tasklist)[8].type = GIK_LAPOINT;
      (*tasklist)[8].default_joints[0] = 15;
      (*tasklist)[8].default_joints[1] = 16;
      (*tasklist)[8].default_joints[2] = 17;
      (*tasklist)[8].default_joints[3] = 18;
      (*tasklist)[8].default_joints[4] = 19;
      (*tasklist)[8].default_joints[5] = 20;
      (*tasklist)[8].default_joints[6] = 21;
      (*tasklist)[8].default_joints[7] = 39;
      (*tasklist)[8].active_joint = 39; /* active joint */
      (*tasklist)[8].default_joints_no = 8;
      
      return TRUE;

    case HRI_BERT:
      manip->type = TWO_ARMED;
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
      manip->type = TWO_ARMED;
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
      (*tasklist)[0].active_joint = 26; /* active joint */
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
      (*tasklist)[1].active_joint = 28; /* active joint */
      (*tasklist)[1].default_joints_no = 8;
		  
      return TRUE;
      
    default:
      PrintError(("Agent type unknown. Cannot create manipulation structures\n"));
      return FALSE;
  }
  
  return FALSE;
}
/* This function computes the tolerance distance of GIK computation depending on the BB of the object */
/* It then calls for hri_agent_single_task_manip_move */
int hri_agent_single_task_manip_move(HRI_AGENT * agent, HRI_GIK_TASK_TYPE type, p3d_rob * object, configPt *q)
{
  double smallest_edge_dim;
  p3d_vector3 goalCoord;
  int res;
  
  if(agent==NULL || object==NULL || q==NULL)
    return FALSE;
  
  smallest_edge_dim = (object->BB.xmax-object->BB.xmin < object->BB.ymax-object->BB.ymin)?object->BB.xmax-object->BB.xmin:object->BB.ymax-object->BB.ymin;
  
  smallest_edge_dim = (smallest_edge_dim < object->BB.zmax-object->BB.zmin)?smallest_edge_dim:object->BB.zmax-object->BB.zmin;
  
  goalCoord[0] = (object->BB.xmax+object->BB.xmin)/2;
  goalCoord[1] = (object->BB.ymax+object->BB.ymin)/2; 
  goalCoord[2] = (object->BB.zmax+object->BB.zmin)/2; 
  
  res = hri_agent_single_task_manip_move(agent, type, &goalCoord, smallest_edge_dim/2, q);
    
  return res;
}
int hri_agent_single_task_manip_move(HRI_AGENT * agent, HRI_GIK_TASK_TYPE type, p3d_vector3 * goalCoord, double approach_distance, configPt *q)
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
        if(!hri_gik_compute(agent->robotPt, manip->gik, 500, approach_distance, goalCoord, q, NULL)){
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

  if(!hri_gik_initialize_gik(manip->gik,agent->robotPt,
                             manip->tasklist[manip->activetasks[0]].default_joints_no))
    return FALSE;

  if(!hri_gik_add_task(manip->gik,3,manip->tasklist[manip->activetasks[0]].default_joints_no,1,
                       manip->tasklist[manip->activetasks[0]].default_joints,
                       manip->tasklist[manip->activetasks[0]].active_joint))
    return FALSE;


  if(!hri_gik_compute(agent->robotPt, manip->gik, 500, 0.02, goalCoord, q, NULL))
    return FALSE;

  return TRUE;
}

hri_shared_zone zone[500];
int shared_zone_l = 0;
int SWITCH_TO_GREEN = FALSE;


int g3d_hri_display_shared_zone()
{
  int i;

  for(i=0; i<shared_zone_l; i++) {
    if(zone[i].value == 1){
      g3d_draw_a_box(zone[i].x-0.02, zone[i].x+0.02, zone[i].y-0.02, zone[i].y+0.02, zone[i].z-0.02, zone[i].z+0.02, Green, FALSE);
    }
    else {
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


static int hri_compute_leg_angles(double hipknee, double kneeankle, double ankleground, double hipground, double ankledist, double *hip, double *knee, double *ankle)
{
  // legs form 2 triangles: (hipknee,kneeankle,hipankle) and (hipgound,ankledist,hipankle)
  
  double alpha;  // alpha = angle between hipground and hipankle
  double hiptoanklelevel = hipground - ankleground;
  double hipankle = sqrt( SQR(hiptoanklelevel)+SQR(ankledist) );
  
  if (hiptoanklelevel < ankleground) {
    *hip   = 0;
    *ankle = 0;
    *knee  = 0;
    return TRUE;
  }
  
  if(hipankle > hipknee+kneeankle) {
    // Legs cannot reach to ankledist
    if(hipknee+kneeankle < ABS(hiptoanklelevel)) {
      // Flying man
      *hip   = 0;
      *ankle = 0;
      *knee  = 0;
    }
    else {
      *knee = 0;
      *hip = acos(hiptoanklelevel/(hipknee+kneeankle));
      *ankle = -*hip;
    }
  }
  else {
    alpha = atan2(ankledist, hiptoanklelevel);
    
    *hip   = acos( (SQR(hipknee)+SQR(hipankle)-SQR(kneeankle))/(2*hipknee*hipankle) ) + alpha;
    *knee  = M_PI - acos( (SQR(hipknee)+SQR(kneeankle)-SQR(hipankle))/(2*hipknee*kneeankle) );
    *ankle = *knee - *hip; 
  }
  return TRUE;
}

/**
 * sets agents joints and state according to head height and state (STANDING, SITTING, MOVING) .
 */
int hri_agent_compute_posture(HRI_AGENT * agent, double neck_height, int state, configPt q)
{
  double hiptoknee_dist, kneetoankle_dist, necktobase_dist, hiptoground_dist; 
  double basetohip_dist, ankletoground_dist;
  double lhip_angle, lknee_angle, lankle_angle;
  double rhip_angle, rknee_angle, rankle_angle;
  
  if(agent == NULL) return FALSE;
  
  // Only supports ACHILE human model. Everything else returns FALSE.
  
  if(agent->type == HRI_ACHILE) {
    
    necktobase_dist = agent->robotPt->joints[5]->abs_pos[2][3]-agent->robotPt->joints[1]->abs_pos[2][3];
    hiptoknee_dist = DISTANCE3D(agent->robotPt->joints[23]->abs_pos[0][3],
                                agent->robotPt->joints[23]->abs_pos[1][3],
                                agent->robotPt->joints[23]->abs_pos[2][3],
                                agent->robotPt->joints[25]->abs_pos[0][3],
                                agent->robotPt->joints[25]->abs_pos[1][3],
                                agent->robotPt->joints[25]->abs_pos[2][3]); // Constant -> 0.47
    kneetoankle_dist = DISTANCE3D(agent->robotPt->joints[25]->abs_pos[0][3],
                                  agent->robotPt->joints[25]->abs_pos[1][3],
                                  agent->robotPt->joints[25]->abs_pos[2][3],
                                  agent->robotPt->joints[27]->abs_pos[0][3],  
                                  agent->robotPt->joints[27]->abs_pos[1][3],
                                  agent->robotPt->joints[27]->abs_pos[2][3]); // Constant -> 0.39
    basetohip_dist = agent->robotPt->joints[1]->abs_pos[2][3]-agent->robotPt->joints[23]->abs_pos[2][3];
    hiptoground_dist =  neck_height - necktobase_dist - basetohip_dist;
    ankletoground_dist = 0.09; // Fixed distance in ACHILE model
    
    switch (state) {
      case 0: // STANDING 
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, 0, 
                               &lhip_angle, &lknee_angle, &lankle_angle);
        rhip_angle = lhip_angle;
        rknee_angle = lknee_angle;
        rankle_angle = lankle_angle;
        break;
      case 1: // SITTING
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, 0.4, 
                               &lhip_angle, &lknee_angle, &lankle_angle);
        rhip_angle = lhip_angle;
        rknee_angle = lknee_angle;
        rankle_angle = lankle_angle;
        break;
      case 2: // MOVING
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, 0.2, 
                               &lhip_angle, &lknee_angle, &lankle_angle);
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, -0.2, 
                               &rhip_angle, &rknee_angle, &rankle_angle);
        break;
      default:
        rhip_angle = lhip_angle = 0;
        rknee_angle = lknee_angle = 0;
        rankle_angle = lankle_angle = 0;
        break;
    }
    q[8] = neck_height - necktobase_dist;
    q[agent->robotPt->joints[23]->index_dof] = -lhip_angle;
    q[agent->robotPt->joints[25]->index_dof] = lknee_angle;
    q[agent->robotPt->joints[27]->index_dof] = -lankle_angle;
    q[agent->robotPt->joints[30]->index_dof] = -rhip_angle;
    q[agent->robotPt->joints[32]->index_dof] = rknee_angle;
    q[agent->robotPt->joints[34]->index_dof] = -rankle_angle;
    
    agent->actual_state = state;
    
    return TRUE;
  } 
  else {
    //printf("In %s:%d, Trying to compute the posture of an unsupported robot\n",__FILE__,__LINE__);
    return FALSE;
  }
  
}

/**
 * sets agents joints and state according to head height and state (STANDING, SITTING, MOVING) .
 */
int hri_agent_compute_state_posture(HRI_AGENT * agent, int state, configPt q)
{
  double hiptoknee_dist, kneetoankle_dist, necktobase_dist, hiptoground_dist; 
  double basetohip_dist, ankletoground_dist;
  double lhip_angle, lknee_angle, lankle_angle;
  double rhip_angle, rknee_angle, rankle_angle;
  double neck_height;
  
  if(agent == NULL) return FALSE;
  
  // Only supports ACHILE human model. Everything else returns FALSE.
  
  if(agent->type == HRI_ACHILE) {
    
    necktobase_dist = agent->robotPt->joints[5]->abs_pos[2][3]-agent->robotPt->joints[1]->abs_pos[2][3];
    hiptoknee_dist = DISTANCE3D(agent->robotPt->joints[23]->abs_pos[0][3],
                                agent->robotPt->joints[23]->abs_pos[1][3],
                                agent->robotPt->joints[23]->abs_pos[2][3],
                                agent->robotPt->joints[25]->abs_pos[0][3],
                                agent->robotPt->joints[25]->abs_pos[1][3],
                                agent->robotPt->joints[25]->abs_pos[2][3]); // Constant -> 0.47
    kneetoankle_dist = DISTANCE3D(agent->robotPt->joints[25]->abs_pos[0][3],
                                  agent->robotPt->joints[25]->abs_pos[1][3],
                                  agent->robotPt->joints[25]->abs_pos[2][3],
                                  agent->robotPt->joints[27]->abs_pos[0][3],  
                                  agent->robotPt->joints[27]->abs_pos[1][3],
                                  agent->robotPt->joints[27]->abs_pos[2][3]); // Constant -> 0.39
    basetohip_dist = agent->robotPt->joints[1]->abs_pos[2][3]-agent->robotPt->joints[23]->abs_pos[2][3];
    ankletoground_dist = 0.09; // Fixed distance in ACHILE model
    
    switch (state) {
      case 0: // STANDING 
        neck_height = 1.6;
        hiptoground_dist =  neck_height - necktobase_dist - basetohip_dist;
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, 0, 
                               &lhip_angle, &lknee_angle, &lankle_angle);
        rhip_angle = lhip_angle;
        rknee_angle = lknee_angle;
        rankle_angle = lankle_angle;
        break;
      case 1: // SITTING
        neck_height = 1.1;
        hiptoground_dist =  neck_height - necktobase_dist - basetohip_dist;
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, 0.4, 
                               &lhip_angle, &lknee_angle, &lankle_angle);
        rhip_angle = lhip_angle;
        rknee_angle = lknee_angle;
        rankle_angle = lankle_angle;
        break;
      case 2: // MOVING
        neck_height = 1.6;
        hiptoground_dist =  neck_height - necktobase_dist - basetohip_dist;
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, 0.2, 
                               &lhip_angle, &lknee_angle, &lankle_angle);
        hri_compute_leg_angles(hiptoknee_dist, kneetoankle_dist, ankletoground_dist, hiptoground_dist, -0.2, 
                               &rhip_angle, &rknee_angle, &rankle_angle);
        break;
      default:
        neck_height = 1.6;
        rhip_angle = lhip_angle = 0;
        rknee_angle = lknee_angle = 0;
        rankle_angle = lankle_angle = 0;
        break;
    }
    q[8] = neck_height - necktobase_dist;
    q[agent->robotPt->joints[23]->index_dof] = -lhip_angle;
    q[agent->robotPt->joints[25]->index_dof] = lknee_angle;
    q[agent->robotPt->joints[27]->index_dof] = -lankle_angle;
    q[agent->robotPt->joints[30]->index_dof] = -rhip_angle;
    q[agent->robotPt->joints[32]->index_dof] = rknee_angle;
    q[agent->robotPt->joints[34]->index_dof] = -rankle_angle;
    
    agent->actual_state = state;
    
    return TRUE;
  } 
  else {
    //printf("In %s:%d, Trying to compute the posture of an unsupported robot\n",__FILE__,__LINE__);
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
