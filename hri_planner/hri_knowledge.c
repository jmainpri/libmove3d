#include "P3d-pkg.h"
#include "Hri_planner-pkg.h"

HRI_ENTITIES * GLOBAL_ENTITIES = NULL;

HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent)
{
  HRI_KNOWLEDGE * kn;
  
  kn = MY_ALLOC(HRI_KNOWLEDGE, 1);

  kn->sees = NULL;
  kn->sees_nb = 0;
  kn->reaches = NULL;
  kn->reaches_nb = 0;
  kn->points_at = 0;
  kn->points_at_nb = 0;
  kn->looks_at = NULL;
  kn->looks_at_nb = 0;
  kn->entities = NULL;
  kn->entities_nb = 0;
  
  return kn;
}

// Entities is a list of things that have some interest for spatial reasoning. For ex. objects, human body parts, surfaces, etc
// We consider all move3d robots except virtual ones (VISBALL for ex) and all move3d objects containing the words "surface, hand, head, camera"
// TODO: The cool thing would be to give a list of important things in the p3d file
//       and make this function take that list
HRI_ENTITIES * hri_create_entities()
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i, j;
  int ent_i;
  HRI_ENTITIES * entities;
  
  entities = MY_ALLOC(HRI_ENTITIES, 1);
  entities->entities = NULL;
  ent_i = 0;
  
  for(i=0; i<env->nr; i++) {
    if(!strcasestr(env->robot[i]->name,"GRIPPER") && !strcasestr(env->robot[i]->name,"VISBALL")) {
      entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY, ent_i, ent_i+1);
      entities->entities[ent_i].can_disappear = TRUE;
      entities->entities[ent_i].robotPt = env->robot[i];
      entities->entities[ent_i].partPt = NULL;
      entities->entities[ent_i].type = HRI_OBJECT;
      ent_i++;
      
      for(j=0; j<env->robot[i]->no; j++) {
        if(!strcasestr(env->robot[i]->o[j]->name,"GHOST") && 
           (strcasestr(env->robot[i]->o[j]->name,"SURFACE") || strcasestr(env->robot[i]->o[j]->name,"HAND") ||
            strcasestr(env->robot[i]->o[j]->name,"HEAD")    || strcasestr(env->robot[i]->o[j]->name,"CAMERA")) ) {
             entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY, ent_i, ent_i+1);
             entities->entities[ent_i].can_disappear = TRUE;
             entities->entities[ent_i].robotPt = env->robot[i];
             entities->entities[ent_i].partPt = env->robot[i]->o[j];
             entities->entities[ent_i].type = HRI_OBJECT_PART;
             ent_i++;
           }
      }
    }
  }
  
  entities->entities_nb = ent_i;

  return entities;     
}

int hri_refine_entity_types(HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
  int i;
  int is_human;
  int agent_idx;
  
  if(entities == NULL || agents == NULL)
    return FALSE;
  
  for(i=0; i<entities->entities_nb; i++) {
    switch (entities->entities[i].type) {
      case HRI_OBJECT:
        if(hri_is_robot_an_agent(entities->entities[i].robotPt, agents, &is_human, &agent_idx)) {
          entities->entities[i].type = HRI_ISAGENT;
          entities->entities[i].agent_idx = agent_idx;
        }
        break;
      case HRI_OBJECT_PART:
        if(hri_is_robot_an_agent(entities->entities[i].robotPt, agents, &is_human, &agent_idx)) {
          entities->entities[i].type = HRI_AGENT_PART;
          entities->entities[i].agent_idx = agent_idx;
        }
        break;
      default:
        break;
    }
  }
  return TRUE;
}
        
        








