#include "P3d-pkg.h"
#include "Collision-pkg.h"
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

/* By default the entity structure doesn't attached to agents */
/* The user can call following function to link entities to agents */
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


HRI_REACHABILITY hri_is_reachable(HRI_ENTITY * object, HRI_AGENT *agent)
{
  int reached = FALSE;
  configPt qs, q;
  p3d_vector3 Tcoord;
  HRI_REACHABILITY reachability;

  // Target object
  if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART)) {
    Tcoord[0] = (object->partPt->BB.xmax + object->partPt->BB.xmin)/2;
    Tcoord[1] = (object->partPt->BB.ymax + object->partPt->BB.ymin)/2;
    Tcoord[2] = (object->partPt->BB.zmax + object->partPt->BB.zmin)/2;
  }
  else { //TODO: make it go to the center of the object
    Tcoord[0] = (object->robotPt->BB.xmax + object->robotPt->BB.xmin)/2;
    Tcoord[1] = (object->robotPt->BB.ymax + object->robotPt->BB.ymin)/2;
    Tcoord[2] = (object->robotPt->BB.zmax + object->robotPt->BB.zmin)/2;
  }

  if(DISTANCE3D(Tcoord[0],Tcoord[1],Tcoord[2],
                (agent->robotPt->BB.xmax + agent->robotPt->BB.xmin)/2,
                (agent->robotPt->BB.ymax + agent->robotPt->BB.ymin)/2,
                (agent->robotPt->BB.zmax + agent->robotPt->BB.zmin)/2) > 1.5) {

    reachability = HRI_UNREACHABLE;
  }

  qs = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */
  q  = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */

  p3d_get_robot_config_into(agent->robotPt, &qs); /* Saving agent config */

  reached = hri_agent_single_task_manip_move(agent, GIK_LATREACH , &Tcoord, 0.02, &q);

  if(agent->manip->type == TWO_ARMED) {
    p3d_set_and_update_this_robot_conf(agent->robotPt, qs);
    reached = hri_agent_single_task_manip_move(agent, GIK_RATREACH , &Tcoord, 0.02, &q);
  }
  p3d_set_and_update_this_robot_conf(agent->robotPt, q);

  p3d_col_activate_robot(agent->robotPt);

  // Check the final config for collision
  // If it is in collision with the entity, then it is reached
  // If it is in collision with another thing then it is hard to reach
  // Otherwise unreachable

  if(!p3d_col_test_robot(agent->robotPt, 0)) {
    // There is a collision. Test if it's with the target.
    if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART))
      p3d_col_deactivate_obj(object->partPt);
    else
      p3d_col_deactivate_rob(object->robotPt);

    if(!p3d_col_test_robot(agent->robotPt, 0)) {
      // The robot colliding another object
      if(reached)
        reachability = HRI_HARDLY_REACHABLE;
      else
        reachability = HRI_UNREACHABLE;
    }
    else {
      // Robot collides only with the target
      // That means it is reached
      reachability = HRI_REACHABLE;
    }
    if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART))
      p3d_col_activate_obj(object->partPt);
    else
      p3d_col_activate_rob(object->robotPt);
  }
  else {
    if(reached) {
      reachability = HRI_REACHABLE;
    }
    else {
      reachability = HRI_UNREACHABLE;
    }
  }

  p3d_set_and_update_this_robot_conf(agent->robotPt,qs);

  MY_FREE(q, double , agent->robotPt->nb_dof); /* FREE */
  MY_FREE(qs, double, agent->robotPt->nb_dof); /* FREE */

  return reachability;
}

/* Computes Placement relations (Ison, isin, etc) between two objects */
HRI_PLACEMENT_RELATION hri_placement_relation(p3d_rob *sourceObj, p3d_rob *targetObj)
{
  p3d_vector3 sourceObjC, targetObjC;

  if(sourceObj == NULL || targetObj == NULL) {
    printf("%s:%d plrelation input is null",__FILE__,__LINE__);
    return HRI_NOPLR;
  }

  sourceObjC[0] = (sourceObj->BB.xmin + sourceObj->BB.xmax)/2;
  sourceObjC[1] = (sourceObj->BB.ymin + sourceObj->BB.ymax)/2;
  sourceObjC[2] = (sourceObj->BB.zmin + sourceObj->BB.zmax)/2;

  targetObjC[0] = (targetObj->BB.xmin + targetObj->BB.xmax)/2;
  targetObjC[1] = (targetObj->BB.ymin + targetObj->BB.ymax)/2;
  targetObjC[2] = (targetObj->BB.zmin + targetObj->BB.zmax)/2;

  /* Test if source Obj is in targetObj */

  if( hri_is_in(&sourceObj->BB, &targetObj->BB) )
    return HRI_ISIN;

  /* Test if sourceObj is on targetObj */

  if( hri_is_on(sourceObjC, &targetObj->BB) )
    return HRI_ISON;

  /* Test if sourceObj is next to targetObj */

  if ( hri_is_nexto(sourceObjC, &sourceObj->BB, targetObjC, &targetObj->BB) )
    return HRI_ISNEXTTO;

  return HRI_NOPLR;
}

/* Computes Placement relations (Ison, isin, etc) between two entities */
HRI_PLACEMENT_RELATION hri_placement_relation(HRI_ENTITY *sourceObj, HRI_ENTITY *targetObj)
{
  p3d_vector3 sourceObjC, targetObjC;
  p3d_BB * sourceBB, *targetBB;

  if(sourceObj == NULL || targetObj == NULL) {
    printf("%s:%d plrelation input is null",__FILE__,__LINE__);
    return HRI_NOPLR;
  }

  if((sourceObj->type == HRI_OBJECT_PART) || (sourceObj->type == HRI_AGENT_PART) ) {
    sourceObjC[0] = (sourceObj->partPt->BB.xmax + sourceObj->partPt->BB.xmin)/2;
    sourceObjC[1] = (sourceObj->partPt->BB.ymax + sourceObj->partPt->BB.ymin)/2;
    sourceObjC[2] = (sourceObj->partPt->BB.zmax + sourceObj->partPt->BB.zmin)/2;
    sourceBB = &sourceObj->partPt->BB;
  }
  else {
    sourceObjC[0] = (sourceObj->robotPt->BB.xmax + sourceObj->robotPt->BB.xmin)/2;
    sourceObjC[1] = (sourceObj->robotPt->BB.ymax + sourceObj->robotPt->BB.ymin)/2;
    sourceObjC[2] = (sourceObj->robotPt->BB.zmax + sourceObj->robotPt->BB.zmin)/2;
    sourceBB = &sourceObj->robotPt->BB;
  }

  if((targetObj->type == HRI_OBJECT_PART) || (targetObj->type == HRI_AGENT_PART) ) {
    targetObjC[0] = (targetObj->partPt->BB.xmax + targetObj->partPt->BB.xmin)/2;
    targetObjC[1] = (targetObj->partPt->BB.ymax + targetObj->partPt->BB.ymin)/2;
    targetObjC[2] = (targetObj->partPt->BB.zmax + targetObj->partPt->BB.zmin)/2;
    targetBB = &targetObj->partPt->BB;
  }
  else {
    targetObjC[0] = (targetObj->robotPt->BB.xmax + targetObj->robotPt->BB.xmin)/2;
    targetObjC[1] = (targetObj->robotPt->BB.ymax + targetObj->robotPt->BB.ymin)/2;
    targetObjC[2] = (targetObj->robotPt->BB.zmax + targetObj->robotPt->BB.zmin)/2;
    targetBB = &targetObj->robotPt->BB;
  }

  /* TEST of placement relations */
  /* Relations are mutually exclusive */

  /* Test if source Obj is in targetObj */

  if( hri_is_in(sourceBB, targetBB) )
    return HRI_ISIN;

  /* Test if sourceObj is on targetObj */

  if( hri_is_on(sourceObjC, targetBB) )
      return HRI_ISON;

  /* Test if sourceObj is next to targetObj */

  if ( hri_is_nexto(sourceObjC, sourceBB, targetObjC, targetBB) )
      return HRI_ISNEXTTO;

  return HRI_NOPLR;
}

/* Test if sourceObj is on targetObj */
int hri_is_on(p3d_vector3 sourceC, p3d_BB *targetBB)
{
  /* Compute ON */

  /* Test if sourceObj is on targetObj */
  /* Condition 1: The center of SourceObj BB should be in the x,y limits of targetObj BB */
  /* Condition 2: The lower part of SourceObj BB should either intersect with the upper part of targetObj BB */
  /*              or there should be few cm's (1.2 times) */
  // TODO: There is something weird here. is on do not depend on the BB of source object??

  if((sourceC[0] >= targetBB->xmin) && (sourceC[0] <= targetBB->xmax) &&
     (sourceC[1] >= targetBB->ymin) && (sourceC[1] <= targetBB->ymax) &&
     (sourceC[2] >= targetBB->zmax)) {

    if( ABS(sourceC[2]-targetBB->zmin)*1.2 >= ABS(sourceC[2]-targetBB->zmax)) {
      return TRUE;
    }
  }

  return FALSE;
}

/* Test if source Obj is in targetObj */
int hri_is_in(p3d_BB *sourceBB, p3d_BB *targetBB)
{
  /* Compute IN */
  /* Test if source Obj is in targetObj */
  /* Condition: Source Obj BB should be wholly in targetObj BB */

  if((targetBB->xmin <= sourceBB->xmin) && (targetBB->xmax >= sourceBB->xmax) &&
     (targetBB->ymin <= sourceBB->ymin) && (targetBB->ymax >= sourceBB->ymax) &&
     (targetBB->zmin <= sourceBB->zmin) && (targetBB->zmax >= sourceBB->zmax)) {
    return TRUE;
  }

  return FALSE;
}

/* Test if sourceObj is next to targetObj */
int hri_is_nexto(p3d_vector3 sourceC, p3d_BB *sourceBB, p3d_vector3 targetC, p3d_BB *targetBB)
{
  /* Compute NEXT */

  /* Test if sourceObj is next to targetObj */
  /* Condition 1: The Z values of sourceObj and targetObj BB's should intersect = One should not be wholly above the other */
  /* Condition 2: Distance(sourceObj,targetObj) should be less then a constant */

  if ( !(sourceBB->zmax <= targetBB->zmin || sourceBB->zmin >= targetBB->zmax) ) {
    if(DISTANCE2D(sourceC[0], sourceC[1], targetC[0], targetC[1]) <
       MAX(ABS(sourceBB->xmin-sourceBB->xmax), ABS(sourceBB->ymin-sourceBB->ymax)) &&
       DISTANCE2D(sourceC[0], sourceC[1], targetC[0], targetC[1]) >
       MAX(ABS(sourceBB->xmin-sourceBB->xmax)/2, ABS(sourceBB->ymin-sourceBB->ymax)/2)) {

      return TRUE;
    }
  }

  if ( !(targetBB->zmax <= sourceBB->zmin || targetBB->zmin >= sourceBB->zmax) ) {
    if(DISTANCE2D(targetC[0], targetC[1], sourceC[0], sourceC[1]) <
       MAX(ABS(targetBB->xmin-targetBB->xmax), ABS(targetBB->ymin-targetBB->ymax)) &&
       DISTANCE2D(targetC[0], targetC[1], sourceC[0], sourceC[1]) >
       MAX(ABS(targetBB->xmin-targetBB->xmax)/2, ABS(targetBB->ymin-targetBB->ymax)/2)) {

      return TRUE;
    }
  }

  return FALSE;
}

