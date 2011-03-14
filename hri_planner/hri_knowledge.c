#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Hri_planner-pkg.h"

HRI_ENTITIES * GLOBAL_ENTITIES = NULL;

HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent)
{
  HRI_KNOWLEDGE * kn;

  kn = MY_ALLOC(HRI_KNOWLEDGE, 1);

  /* kn->points_at = 0; */
  /* kn->points_at_nb = 0; */
  /* kn->looks_at = NULL; */
  /* kn->looks_at_nb = 0; */
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
  char* objectrealname;

  entities = MY_ALLOC(HRI_ENTITIES, 1);
  entities->entities = NULL;
  ent_i = 0;
  entities->eventsInTheWorld = FALSE;
  entities->lastEventsInTheWorldStep = 4; //TODO use a constant
  entities->isWorldStatic = TRUE;
  entities->needSituationAssessmentUpdate = FALSE;

  for(i=0; i<env->nr; i++) {
    if(!strcasestr(env->robot[i]->name,"GRIPPER") && !strcasestr(env->robot[i]->name,"VISBALL") && !strcasestr(env->robot[i]->name,"SAHandRight")) {

      if(!strcasestr(env->robot[i]->name,"CHAIR")) {
        entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY*, ent_i, ent_i+1);
        entities->entities[ent_i] = MY_ALLOC(HRI_ENTITY,1);
	strcpy(entities->entities[ent_i]->name, env->robot[i]->name);

        entities->entities[ent_i]->is_present = FALSE;
        entities->entities[ent_i]->is_detected = FALSE;
	entities->entities[ent_i]->detection_time = 0;
	entities->entities[ent_i]->last_detection_time = 0;
	entities->entities[ent_i]->undetection_iter = 0;
	entities->entities[ent_i]->undetection_status = HRI_NEVER_DETECTED;
	entities->entities[ent_i]->visibility_percentage = 0.0;
        entities->entities[ent_i]->can_disappear_and_move = FALSE;
        entities->entities[ent_i]->disappeared = FALSE;
	entities->entities[ent_i]->last_ismoving_iter = 0;
	entities->entities[ent_i]->filtered_motion = HRI_UK_MOTION;
	entities->entities[ent_i]->is_pl_state_transition_new = FALSE;
	entities->entities[ent_i]->pl_state_transition = HRI_UK_PL_STATE_TRANSITION;

        entities->entities[ent_i]->robotPt = env->robot[i];
        entities->entities[ent_i]->partPt = NULL;
        entities->entities[ent_i]->type = HRI_OBJECT;
	if(strcasestr(env->robot[i]->name,"TABLE")||strcasestr(env->robot[i]->name,"SHELF"))
	  entities->entities[ent_i]->subtype = HRI_OBJECT_SUPPORT;
	else if(strcasestr(env->robot[i]->name,"TAPE")||strcasestr(env->robot[i]->name,"BOTTLE")||strcasestr(env->robot[i]->name,"BOX")||strcasestr(env->robot[i]->name,"CUBE")){
	  entities->entities[ent_i]->subtype = HRI_MOVABLE_OBJECT;
	  entities->entities[ent_i]->can_disappear_and_move = TRUE;
	}
	else if(strcasestr(env->robot[i]->name,"TRASHBIN"))
	  entities->entities[ent_i]->subtype = HRI_OBJECT_CONTAINER;
	else if(strcasestr(env->robot[i]->name,"PLACEMAT"))
	  entities->entities[ent_i]->subtype = HRI_OBJECT_PLACEMAT;
	else
	  entities->entities[ent_i]->subtype = HRI_UK_ENTITY_SUBTYPE;
        ent_i++;
      }

      for(j=0; j<env->robot[i]->no; j++) {
        objectrealname = strrchr(env->robot[i]->o[j]->name, '.')+1;
        if(!strcasestr(objectrealname,"GHOST") &&
           (strcasestr(objectrealname,"SURFACE") || strcasestr(objectrealname,"HAND") ||
            strcasestr(objectrealname,"HEAD")    || strcasestr(objectrealname,"CAMERA")) ) {
             entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY*, ent_i, ent_i+1);
             entities->entities[ent_i] = MY_ALLOC(HRI_ENTITY,1);
	     strcpy(entities->entities[ent_i]->name, objectrealname);
             entities->entities[ent_i]->can_disappear_and_move = FALSE;
             entities->entities[ent_i]->is_present = FALSE;
             entities->entities[ent_i]->disappeared = FALSE;
             entities->entities[ent_i]->robotPt = env->robot[i];
             entities->entities[ent_i]->partPt = env->robot[i]->o[j];
             entities->entities[ent_i]->type = HRI_OBJECT_PART;
	     if(strcasestr(env->robot[i]->name,"HEAD")||strcasestr(env->robot[i]->name,"CAMERA"))
	       entities->entities[ent_i]->subtype = HRI_AGENT_HEAD;
	     else if(strcasestr(env->robot[i]->name,"HAND"))
	       entities->entities[ent_i]->subtype = HRI_AGENT_HAND;
	     else
	       entities->entities[ent_i]->subtype = HRI_UK_ENTITY_SUBTYPE;
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
int hri_link_agents_with_entities(HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
  int i;
  int is_human;
  int agent_idx;

  if(entities == NULL || agents == NULL)
    return FALSE;

  for(i=0; i<entities->entities_nb; i++) {
    switch (entities->entities[i]->type) {
      case HRI_OBJECT:
        if(hri_is_robot_an_agent(entities->entities[i]->robotPt, agents, &is_human, &agent_idx)) {
          entities->entities[i]->type = HRI_ISAGENT;
          entities->entities[i]->agent_idx = agent_idx;
          agents->all_agents[agent_idx]->entity_idx = i;
        }
        break;
      case HRI_OBJECT_PART:
        if(hri_is_robot_an_agent(entities->entities[i]->robotPt, agents, &is_human, &agent_idx)) {
          entities->entities[i]->type = HRI_AGENT_PART;
          entities->entities[i]->agent_idx = agent_idx;
          if(strcasestr(entities->entities[i]->partPt->name, "head") || strcasestr(entities->entities[i]->partPt->name, "camera")) {
            agents->all_agents[agent_idx]->head = MY_REALLOC(agents->all_agents[agent_idx]->head, HRI_ENTITY*,
							     agents->all_agents[agent_idx]->head_nb,
							     agents->all_agents[agent_idx]->head_nb+1);
            agents->all_agents[agent_idx]->head[agents->all_agents[agent_idx]->head_nb++] = entities->entities[i];
	    
          }
          if(strcasestr(entities->entities[i]->partPt->name, "hand")) {
            agents->all_agents[agent_idx]->hand = MY_REALLOC(agents->all_agents[agent_idx]->hand, HRI_ENTITY*,
							     agents->all_agents[agent_idx]->hand_nb,
							     agents->all_agents[agent_idx]->hand_nb+1);
            agents->all_agents[agent_idx]->hand[agents->all_agents[agent_idx]->hand_nb++] = entities->entities[i];
          }
        }
        break;
      default:
        break;
    }
  }

  return TRUE;
}

int hri_initialize_all_agents_knowledge(HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
  int i;

  for(i=0; i<agents->all_agents_no; i++) {
    if (!hri_initialize_agent_knowledge(agents->all_agents[i]->knowledge, entities, agents)) {
      return FALSE;
    }
  }
  return TRUE;
}

int hri_initialize_agent_knowledge(HRI_KNOWLEDGE * knowledge, HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
  int i, j;

  if((knowledge == NULL) || (entities == NULL) || (agents == NULL))
    return FALSE;

  knowledge->entities = MY_ALLOC(HRI_KNOWLEDGE_ON_ENTITY, entities->entities_nb);
  knowledge->entities_nb = entities->entities_nb;

  for(i=0; i<knowledge->entities_nb; i++) {

    knowledge->entities[i].entPt = entities->entities[i];

    knowledge->entities[i].disappeared_isexported = TRUE;

    knowledge->entities[i].motion = HRI_UK_MOTION;
    knowledge->entities[i].motion_ischanged = FALSE;
    knowledge->entities[i].motion_isexported = FALSE;

    knowledge->entities[i].is_placed_from_visibility = HRI_UK_VIS_PLACE;
    knowledge->entities[i].visibility_placement_ischanged = FALSE;
    knowledge->entities[i].visibility_placement_isexported = FALSE;

    knowledge->entities[i].visibility = HRI_UK_VIS;
    knowledge->entities[i].visibility_ischanged = FALSE;
    knowledge->entities[i].visibility_isexported = FALSE;

    knowledge->entities[i].reachability = HRI_UK_REACHABILITY;
    knowledge->entities[i].reachability_ischanged = FALSE;
    knowledge->entities[i].reachability_isexported = FALSE;

    knowledge->entities[i].is_looked_at = HRI_UK_V;
    knowledge->entities[i].is_looked_at_ischanged = FALSE;
    knowledge->entities[i].is_looked_at_isexported = FALSE;

    knowledge->entities[i].is_pointed_at = HRI_UK_V;
    knowledge->entities[i].is_pointed_at_ischanged = FALSE;
    knowledge->entities[i].is_pointed_at_isexported = FALSE;

    knowledge->entities[i].is_located_from_agent = HRI_UK_RELATION;
    knowledge->entities[i].spatial_relation_ischanged = FALSE;
    knowledge->entities[i].spatial_relation_isexported = FALSE;

    knowledge->entities[i].is_placed = MY_ALLOC(HRI_PLACEMENT_RELATION, entities->entities_nb);
    knowledge->entities[i].is_placed_old = MY_ALLOC(HRI_PLACEMENT_RELATION, entities->entities_nb);
    knowledge->entities[i].placement_relation_ischanged = MY_ALLOC(int, entities->entities_nb);
    knowledge->entities[i].placement_relation_isexported = MY_ALLOC(int, entities->entities_nb);

    knowledge->entities[i].is_placed_nb = entities->entities_nb;

    for(j=0; j<knowledge->entities[i].is_placed_nb; j++) {
      knowledge->entities[i].is_placed[j] = HRI_UK_PLR;
      knowledge->entities[i].is_placed_old[j] = HRI_UK_PLR;
      knowledge->entities[i].placement_relation_ischanged[j] = FALSE;
      knowledge->entities[i].placement_relation_isexported[j] = FALSE;
    }
  }

  return TRUE;
}

HRI_REACHABILITY hri_is_reachable_single_arm(HRI_ENTITY * object, HRI_AGENT *agent,HRI_GIK_TASK_TYPE task_type,p3d_vector3* Tcoord)
{
  int reached = FALSE;
  configPt q;
  HRI_REACHABILITY reachability;

  q  = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */

  reached = hri_agent_single_task_manip_move(agent, task_type , Tcoord, 0.02, &q);

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

  MY_FREE(q, double , agent->robotPt->nb_dof); /* FREE */  
 
  return reachability;
}

// TODO: There is a serious problem on the bounding boxes of robot parts.
// I think the reason is that in a macro the first body is takeninto account ghost <->real switch
HRI_REACHABILITY hri_is_reachable(HRI_ENTITY * object, HRI_AGENT *agent)
{
  configPt qs,q;
  p3d_vector3 Tcoord;
  HRI_REACHABILITY reachability,reachabilitySecondArm;

  // Target object
  if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART)) {
    Tcoord[0] = (object->partPt->BB.xmax + object->partPt->BB.xmin)/2;
    Tcoord[1] = (object->partPt->BB.ymax + object->partPt->BB.ymin)/2;
    Tcoord[2] = (object->partPt->BB.zmax + object->partPt->BB.zmin)/2;
  }
  else {
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
  else {

    qs = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */


    p3d_get_robot_config_into(agent->robotPt, &qs); /* Saving agent config */

    reachability = hri_is_reachable_single_arm(object,agent,GIK_LATREACH,&Tcoord);

    // If agent has two arms and object not reachable with first hand we try with second
    if(agent->manip->type == TWO_ARMED && (reachability != HRI_REACHABLE)) {
      p3d_set_and_update_this_robot_conf(agent->robotPt, qs);
      reachabilitySecondArm = hri_is_reachable_single_arm(object,agent,GIK_RATREACH,&Tcoord);
      if(reachabilitySecondArm ==  HRI_REACHABLE) {
	reachability =  HRI_REACHABLE;
      }
      else if(reachabilitySecondArm ==  HRI_HARDLY_REACHABLE){
	reachability =  HRI_HARDLY_REACHABLE;
      }
    }

    p3d_set_and_update_this_robot_conf(agent->robotPt,qs);

    MY_FREE(qs, double, agent->robotPt->nb_dof); /* FREE */
  }
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

  if( hri_is_in(sourceObjC, &targetObj->BB) )
    return HRI_ISIN;

  /* Test if sourceObj is on targetObj */

  if( hri_is_on(sourceObjC, &sourceObj->BB, &targetObj->BB) )
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

  if( hri_is_in(sourceObjC, targetBB) )
    return HRI_ISIN;

  /* Test if sourceObj is on targetObj */

  if( hri_is_on(sourceObjC, sourceBB, targetBB) )
      return HRI_ISON;

  /* Test if sourceObj is next to targetObj */

  if ( hri_is_nexto(sourceObjC, sourceBB, targetObjC, targetBB) )
      return HRI_ISNEXTTO;

  return HRI_NOPLR;
}

/* Test if topObj is on bottomObj */
int hri_is_on(p3d_vector3 topObjC, p3d_BB *topObjBB, p3d_BB *bottomObjBB)
{
  /* Compute ON */

  /* Test if topObj is on bottomObj */
  /* Condition 1: The center of topObj BB should be in the x,y limits of bottomObj BB and higher than bottomObj maximum z limit */
  /* Condition 2: The lower part of topObj BB souldn not be (higher than 5 cm) and (lower than 5 cm) from the higher part of bottomObj BB */

  if((topObjC[0] >= bottomObjBB->xmin) && (topObjC[0] <= bottomObjBB->xmax) &&
     (topObjC[1] >= bottomObjBB->ymin) && (topObjC[1] <= bottomObjBB->ymax) &&
     (topObjC[2] >= bottomObjBB->zmax))
    if((topObjBB->zmin - bottomObjBB->zmax > -0.05) && (topObjBB->zmin-bottomObjBB->zmax < 0.05))
      return TRUE;

  return FALSE;
}

/* Test if insideObj is in outsideObj */
int hri_is_in(p3d_vector3 insideObjC, p3d_BB *outsideObjBB)
{
  /* Compute IN */
  /* Test if insideObj is in outsideObj */
  /* Condition: insideObj center should be wholly in outsideObj BB */

  if((outsideObjBB->xmin <= insideObjC[0]) && (outsideObjBB->xmax >= insideObjC[0]) &&
     (outsideObjBB->ymin <= insideObjC[1]) && (outsideObjBB->ymax >= insideObjC[1]) &&
     (outsideObjBB->zmin <= insideObjC[2]) && (outsideObjBB->zmax >= insideObjC[2]))
    return TRUE;

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

HRI_SPATIAL_RELATION hri_spatial_relation(p3d_rob * object, p3d_rob * robot)
{
  p3d_vector4 targetRealCoord;
  p3d_vector4 targetRelativeCoord;
  p3d_matrix4 inv;
  double rho, phi, theta;
  int isFar;
  double frontAngle = 0.26; //TODO: Make this changeable
  double farLimit = 5.0; //TODO: Make this changeable

  if( (robot == NULL) || (object == NULL) ) {
    return HRI_NO_RELATION;
  }

  targetRealCoord[0] = object->joints[1]->abs_pos[0][3];
  targetRealCoord[1] = object->joints[1]->abs_pos[1][3];
  targetRealCoord[2] = object->joints[1]->abs_pos[2][3];
  targetRealCoord[3] = 1;

  p3d_matInvertXform(robot->joints[1]->abs_pos, inv);

  p3d_matvec4Mult(inv, targetRealCoord, targetRelativeCoord);

  p3d_cartesian2spherical(targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2],
                          &rho, &theta, &phi);


  isFar = (farLimit < DISTANCE2D(targetRelativeCoord[0],targetRelativeCoord[1],0,0));

  //  printf("real coord %f %f %f\n",targetRealCoord[0],targetRealCoord[1],targetRealCoord[2]);
  //  printf("relative coord %f %f %f\n",targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2]);
  //  printf("Phi is %f, isFar is %d\n",phi,isFar);

  /* Phi is the horizontal one */

  if(ABS(phi) < frontAngle) { /* In front */
    if(isFar) return HRI_FAR_FRONT ;
    else   return  HRI_NEAR_FRONT ;
  }
  if(frontAngle <= phi && phi < 3*M_PI/8) { /* Front left */
    if(isFar)  return  HRI_FAR_FRONT_LEFT ;
    else   return HRI_NEAR_FRONT_LEFT ;
  }
  if(3*M_PI/8 <= phi && phi < 5*M_PI/8) { /* left */
    if(isFar)   return HRI_FAR_LEFT ;
    else    return HRI_NEAR_LEFT ;
  }
  if(5*M_PI/8 <= phi && phi < 7*M_PI/8) { /* Back left */
    if(isFar)  return  HRI_FAR_BACK_LEFT ;
    else   return HRI_NEAR_BACK_LEFT ;
  }
  if(7*M_PI/8 <= ABS(phi)) { /* Back */
    if(isFar)  return  HRI_FAR_BACK ;
    else   return  HRI_NEAR_BACK ;
  }
  if(-7*M_PI/8 <= phi && phi < -5*M_PI/8) { /* Back right */
    if(isFar)  return HRI_FAR_BACK_RIGHT ;
    else  return HRI_NEAR_BACK_RIGHT ;
  }
  if(-5*M_PI/8 <= phi && phi < -3*M_PI/8) { /* right */
    if(isFar) return HRI_FAR_RIGHT ;
    else   return HRI_NEAR_RIGHT ;
  }
  if(-3*M_PI/8 <= phi && phi < -1*frontAngle) { /* Front right */
    if(isFar)  return HRI_FAR_FRONT_RIGHT ;
    else   return HRI_NEAR_FRONT_RIGHT ;
  }
  printf("Bad angle value, This shouldn't happen.\n");

  return HRI_NO_RELATION;
}

HRI_SPATIAL_RELATION hri_spatial_relation(HRI_ENTITY * object, HRI_AGENT * agent)
{
  p3d_vector4 targetRealCoord;
  p3d_vector4 targetRelativeCoord;
  p3d_matrix4 inv;
  p3d_BB *objectBB;
  double rho, phi, theta;
  int isFar;
  double frontAngle = 0.26; //TODO: Make this changeable
  double farLimit = 5.0; //TODO: Make this changeable

  if( (agent == NULL) || (object == NULL) ) {
    return HRI_NO_RELATION;
  }

  if(object->type == HRI_OBJECT_PART || object->type == HRI_AGENT_PART)
    objectBB = &object->partPt->BB;
  else
    objectBB = &object->robotPt->BB;

  targetRealCoord[0] = (objectBB->xmax+objectBB->xmin)/2;
  targetRealCoord[1] = (objectBB->ymax+objectBB->ymin)/2;
  targetRealCoord[2] = (objectBB->zmax+objectBB->zmin)/2;
  targetRealCoord[3] = 1;

  p3d_matInvertXform(agent->robotPt->joints[1]->abs_pos, inv);

  p3d_matvec4Mult(inv, targetRealCoord, targetRelativeCoord);

  p3d_cartesian2spherical(targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2],
                          &rho, &theta, &phi);


  isFar = (farLimit < DISTANCE2D(targetRelativeCoord[0],targetRelativeCoord[1],0,0));

  //  printf("real coord %f %f %f\n",targetRealCoord[0],targetRealCoord[1],targetRealCoord[2]);
  //  printf("relative coord %f %f %f\n",targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2]);
  //  printf("Phi is %f, isFar is %d\n",phi,isFar);

  /* Phi is the horizontal one */

  if(ABS(phi) < frontAngle) { /* In front */
    if(isFar) return HRI_FAR_FRONT ;
    else   return  HRI_NEAR_FRONT ;
  }
  if(frontAngle <= phi && phi < 3*M_PI/8) { /* Front left */
    if(isFar)  return  HRI_FAR_FRONT_LEFT ;
    else   return HRI_NEAR_FRONT_LEFT ;
  }
  if(3*M_PI/8 <= phi && phi < 5*M_PI/8) { /* left */
    if(isFar)   return HRI_FAR_LEFT ;
    else    return HRI_NEAR_LEFT ;
  }
  if(5*M_PI/8 <= phi && phi < 7*M_PI/8) { /* Back left */
    if(isFar)  return  HRI_FAR_BACK_LEFT ;
    else   return HRI_NEAR_BACK_LEFT ;
  }
  if(7*M_PI/8 <= ABS(phi)) { /* Back */
    if(isFar)  return  HRI_FAR_BACK ;
    else   return  HRI_NEAR_BACK ;
  }
  if(-7*M_PI/8 <= phi && phi < -5*M_PI/8) { /* Back right */
    if(isFar)  return HRI_FAR_BACK_RIGHT ;
    else  return HRI_NEAR_BACK_RIGHT ;
  }
  if(-5*M_PI/8 <= phi && phi < -3*M_PI/8) { /* right */
    if(isFar) return HRI_FAR_RIGHT ;
    else   return HRI_NEAR_RIGHT ;
  }
  if(-3*M_PI/8 <= phi && phi < -1*frontAngle) { /* Front right */
    if(isFar)  return HRI_FAR_FRONT_RIGHT ;
    else   return HRI_NEAR_FRONT_RIGHT ;
  }
  printf("Bad angle value, This shouldn't happen.\n");

  return HRI_NO_RELATION;
}


void hri_display_entities(HRI_ENTITIES * ents)
{
  int i;

  if(ents == NULL) {
    printf("ENTITIES not initialized\n");
    return ;
  }

  for(i=0; i<ents->entities_nb; i++) {
      printf("%d - ENTITY name: %s type: %d\n", i, ents->entities[i]->name, ents->entities[i]->type);
  }
}

void hri_display_agent_knowledge(HRI_AGENT * agent)
{
  int i, j;
  HRI_KNOWLEDGE * kn;

  if(agent == NULL || agent->knowledge == NULL) {
    printf("AGENT not initialized\n");
    return ;
  }

  kn = agent->knowledge;

  printf("\nFOR AGENT %s:\n", agent->robotPt->name);

  printf("\nKNOWLEDGE ON ENTITY ");
  for(i=0; i<kn->entities_nb; i++) {
    printf("%s\n", kn->entities[i].entPt->name);

    printf("Motion: %d\n", kn->entities[i].motion);
    printf("Visibility: %d\n",kn->entities[i].visibility);
    printf("Reachability: %d\n",kn->entities[i].reachability);
    printf("IsPlaced(vis): %d\n",kn->entities[i].is_placed_from_visibility);

    printf("IsLocated(byAgent): ");
    printf("%d",kn->entities[i].is_located_from_agent);

    printf("\nIsPlaced(Entities): ");
    for(j=0; j<kn->entities[i].is_placed_nb; j++)
      printf("%d,",kn->entities[i].is_placed[j]);

     printf("\n\n");
  }
}

// Function to init motion history array. Matthieu Warnier 01022011 
// 

/* int hri_init_motion_history(HRI_ENTITIES * ents, int ent_index) */
/* { */
/*   int i; */
/*   if(ent_index > -1 && ent_index < entities_nb) { */
/*     if(!ents->entities[ent_index]->can_move){ */
/*       ents->entities[ent_index]->motion_history = MY_ALLOC(HRI_MOTION, 3); */
/*       for(i=0;i<3;i++) */
/* 	ents->entities[ent_index]->motion_history[i]=HRI_UK_MOTION; */
/*       ents->entities[ent_index]->can_move = TRUE; */
/*     } */
/*     return TRUE; */
/*   }      */
/*   else */
/*     return FALSE; */
/* } */


// Function that decide and wethre an object should be 
// 
void hri_manage_object_disappearance_and_move(HRI_AGENTS * agents, HRI_ENTITIES * ents,int robotMyselfIndex , int hasDisappearFilterLength)
{ 
  int e_i;  
  HRI_AGENT * agent;
  HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
  configPt objectQ;
  double visibility_percentage_threshold = 80;

  //Object moving and disappearance management.

  for(e_i=0; e_i<ents->entities_nb; e_i++) {
    if(ents->entities[e_i]->can_disappear_and_move){
      // Is object detected?
      if(ents->entities[e_i]->is_detected){
	if(ents->entities[e_i]->detection_time != ents->entities[e_i]->last_detection_time){
	  if(!ents->entities[e_i]->is_present)
	    ents->entities[e_i]->is_present = TRUE;
	  /// reinitialize visibility percentage for undetection management
	  if( ents->entities[e_i]->visibility_percentage > 0 )
	    ents->entities[e_i]->visibility_percentage = 0;
	  if(ents->entities[e_i]->undetection_status != HRI_DETECTED){
	    ents->entities[e_i]->undetection_status = HRI_DETECTED;
	    ents->entities[e_i]->undetection_iter = 0;
	  }

	  if(ents->entities[e_i]->disappeared){
	    //APPEAR we put object has static.	      
	    ents->entities[e_i]->disappeared = FALSE;	      
	    ents->entities[e_i]->filtered_motion = HRI_STATIC;
	    ents->entities[e_i]->last_ismoving_iter = 0; // Filter first VIMAN isMoving to avoid HRI_START_MOVING and direct HRI_STOP_MOVING
	    ents->eventsInTheWorld = TRUE;
	    ents->entities[e_i]->is_pl_state_transition_new = TRUE;
	    ents->entities[e_i]->pl_state_transition = HRI_APPEAR;
	    printf("%s APPEARED\n",ents->entities[e_i]->name);
	  }
	  else {
	    if((ents->entities[e_i]->last_ismoving_iter>0) && (ents->entities[e_i]->filtered_motion != HRI_MOVING)){
	      //START MOVING
	      ents->entities[e_i]->filtered_motion = HRI_MOVING;
	      ents->eventsInTheWorld = TRUE;
	      ents->entities[e_i]->is_pl_state_transition_new = TRUE;
	      ents->entities[e_i]->pl_state_transition = HRI_START_MOVING;
	      printf("%s START MOVING\n",ents->entities[e_i]->name); 
	    }
	    else if ((ents->entities[e_i]->last_ismoving_iter == 0 ) && (ents->entities[e_i]->filtered_motion != HRI_STATIC)){
	      //STOP MOVING
	      ents->entities[e_i]->filtered_motion = HRI_STATIC;
	      ents->eventsInTheWorld = TRUE;
	      ents->entities[e_i]->is_pl_state_transition_new = TRUE;
	      ents->entities[e_i]->pl_state_transition = HRI_STOP_MOVING;
	      printf("%s STOP MOVING\n",ents->entities[e_i]->name); 
	    }	    
	    else if ((ents->entities[e_i]->last_ismoving_iter == 0 ) && (ents->entities[e_i]->filtered_motion == HRI_STATIC)){
	      //Static, Nothing to do
	    }
	    else if ((ents->entities[e_i]->last_ismoving_iter > 0 ) && (ents->entities[e_i]->filtered_motion == HRI_MOVING)){
	      printf("%s IS MOVING\n",ents->entities[e_i]->name);  
	    }
	    else
	      printf("Impossible motion state %s for entity %d\n",ents->entities[e_i]->name,ents->entities[e_i]->filtered_motion);  
	  }
	  // increment last is moving seen
	  if(ents->entities[e_i]->last_ismoving_iter>0)
	    ents->entities[e_i]->last_ismoving_iter--;	    
	}
      }	
      else{
	if( ents->entities[e_i]->undetection_status != HRI_NEVER_DETECTED){
	  agent=agents->all_agents[robotMyselfIndex];
	  kn_on_ent = &agent->knowledge->entities[e_i];	  
	  if(!ents->entities[e_i]->disappeared && ((kn_on_ent->is_placed_from_visibility == HRI_FOV) || (kn_on_ent->is_placed_from_visibility == HRI_FOA)) && (kn_on_ent->visibility == HRI_VISIBLE)){
	    if(ents->isWorldStatic){

	      // specific test for this entity to assess percentage visibility 
	      // Todo : when to recompute it. Not all the time but often enough.
	      if( ents->entities[e_i]->visibility_percentage == 0 ){
		g3d_compute_visibility_in_fov_for_suspect_undetected_entity( ents, e_i, agent,agents);	    
		printf("Disappear Management - Visibility percentage : %f for entity %s\n" ,ents->entities[e_i]->visibility_percentage , ents->entities[e_i]->name);
	      }

	      // We will consider that the object should be detected above a certain threshold
	      if( ents->entities[e_i]->visibility_percentage > visibility_percentage_threshold){
	      
		// iter on unexplained detection
		if((ents->entities[e_i]->undetection_status == HRI_UNEXPLAINED_UNDETECTION_ITER) && (ents->entities[e_i]->undetection_iter < hasDisappearFilterLength))
		  ents->entities[e_i]->undetection_iter++;
		//  reach maximum number of unexplained detection
		else if((ents->entities[e_i]->undetection_status == HRI_UNEXPLAINED_UNDETECTION_ITER) && (ents->entities[e_i]->undetection_iter == hasDisappearFilterLength))
		  ents->entities[e_i]->undetection_status = HRI_UNEXPLAINED_UNDETECTION_MAX;
		//  initialize iteration on maximum unexplained detection
		else if((ents->entities[e_i]->undetection_status != HRI_UNEXPLAINED_UNDETECTION_ITER) && (ents->entities[e_i]->undetection_status != HRI_UNEXPLAINED_UNDETECTION_MAX)){
		  ents->entities[e_i]->undetection_status = HRI_UNEXPLAINED_UNDETECTION_ITER;
		  ents->entities[e_i]->undetection_iter = 0;
		}
		// Object has disappeared
		else if((ents->entities[e_i]->undetection_status == HRI_UNEXPLAINED_UNDETECTION_MAX)){
		  ents->entities[e_i]->disappeared = TRUE;
		  ents->eventsInTheWorld = TRUE;
		  ents->entities[e_i]->is_pl_state_transition_new = TRUE;
		  ents->entities[e_i]->pl_state_transition = HRI_DISAPPEAR;
		  printf("%s HAS DISAPPEAR\n",ents->entities[e_i]->name);  
		  // put object in 0,0,0 if disappear. 	
		  objectQ = MY_ALLOC(double, ents->entities[e_i]->robotPt->nb_dof); /* ALLOC */
		  p3d_get_robot_config_into(ents->entities[e_i]->robotPt, &objectQ);
		  objectQ[6] = objectQ[7] = objectQ[8] = 0;      
		  p3d_set_and_update_this_robot_conf(ents->entities[e_i]->robotPt, objectQ);
		  MY_FREE(objectQ, double, ents->entities[e_i]->robotPt->nb_dof); /* FREE */
		}
		else
		  printf("Unmanaged state for undetected objects in  hri_manage_object_disappearance_and_move function\n");
	      }
	      else {
		/** low percentage can explain undetection */	    
		if(!ents->entities[e_i]->disappeared)
		  ents->entities[e_i]->undetection_status = HRI_EXPLAINED_UNDETECTION;
	      }
	    }
	    else {
	      // World is not Static
	      // need to update visibility precentage for this entity for next disappear management.
	      ents->entities[e_i]->visibility_percentage = 0;
	    }
	    
	  }
	  else {
	    if(!ents->entities[e_i]->disappeared)
	      ents->entities[e_i]->undetection_status = HRI_EXPLAINED_UNDETECTION;
	  }
	}
      }
    }
  }
  
  // Computing all Situation assessment after each event Appear, Start Moving, Stop Moving and disappear can be costly and delay reading the state of the world ( object, human, robot ) as these events can be quite often folowed shortly one by another. Heavy computations should be done only once the world is detected as "static". We wait four step without an event. This Four should be replaced by a constant.

  if(ents->eventsInTheWorld){
    ents->lastEventsInTheWorldStep = 0;
    ents->isWorldStatic = FALSE;
    ents->needSituationAssessmentUpdate = TRUE;
  }
  else {
    if(ents->lastEventsInTheWorldStep == 4)
      ents->isWorldStatic = TRUE;
    else
      ents->lastEventsInTheWorldStep ++;
  }
  
  
}


// Function computing geometric facts between agents and objects
// Each agent has its own view of the environment.

int hri_compute_geometric_facts(HRI_AGENTS * agents, HRI_ENTITIES * ents, int robotMyselfIndex)
{
  int a_i, e_i, e_j, ge_i, ge_j;
  double elevation, azimuth;
  HRI_ENTITY * ent, ** present_ents;
  int * present_ents_global_idxs;
  int present_ents_nb;
  HRI_AGENT * agent;
  HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
  int res;
  int counter = 0;
  HRI_VISIBILITY * vis_result;
  HRI_REACHABILITY reachability_result;
  HRI_SPATIAL_RELATION spatial_relation_result; 
  HRI_PLACEMENT_RELATION placement_relation_result;

  if(agents == NULL || ents == NULL) {
    printf("Not Initialized\n");
    return FALSE;
  }

  if(ents->eventsInTheWorld || (ents->needSituationAssessmentUpdate && ents->isWorldStatic) ){
    vis_result = MY_ALLOC(HRI_VISIBILITY, ents->entities_nb); // ALLOC
    present_ents = MY_ALLOC(HRI_ENTITY*, ents->entities_nb); // ALLOC
    present_ents_global_idxs = MY_ALLOC(int, ents->entities_nb); // ALLOC
  
    for(a_i=0; a_i<agents->all_agents_no; a_i++) {
      agent = agents->all_agents[a_i];
    
      if(agent->is_present == FALSE)
	continue;




      // Pick entities that exist 
      present_ents_nb = 0;
      for(e_i=0; e_i<ents->entities_nb; e_i++) {
	// If the entity is a part of the current agent, we skip it since it doesn't make sense to compute it from his own point of view
	// TODO: Or does it?
	if( (ents->entities[e_i]->type == HRI_AGENT_PART) || (ents->entities[e_i]->type == HRI_ISAGENT) ) {
	  if( agent == agents->all_agents[ents->entities[e_i]->agent_idx] )
	    continue;
	}
	if(ents->entities[e_i]->is_present) {
	  present_ents[present_ents_nb] = ents->entities[e_i];
	  present_ents_global_idxs[present_ents_nb] = e_i;
	  present_ents_nb++;
	}
      }

      if(ents->needSituationAssessmentUpdate && ents->isWorldStatic){
	/*** for Agent itself ( the one that perceive ) , we use a specific visibility*/
	if(a_i == agents->source_agent_idx){
	  /** VISIBILITY in fov only and taking into account own agent: we do the actual visibility procesing only for entities in fov and foa ie: that should be perceived if not hidden. For other we remember past values */
	  /// We first remember past visibility values
	  for(e_j=0; e_j<present_ents_nb; e_j++) {
	    ge_j = present_ents_global_idxs[e_j];
	    kn_on_ent = &agent->knowledge->entities[ge_j];
	    vis_result[e_j] = kn_on_ent->visibility;
	  }
	  /// make sure we can add an entity in the the present entities table to take into account agent itself
	  if(present_ents_nb<ents->entities_nb)
	    g3d_compute_visibility_in_fov_for_given_entities(present_ents, ents->entities[agent->entity_idx] , agent, vis_result, present_ents_nb);
	  else
	    printf("Number of present entities equal total number of entities. Impossible to use g3d_compute_visibility_in_fov_for_given_entities functions that must be used without agent itself in present entities\n");
	}
	else {	  
	  /** VISIBLITY we recompute visibility from scratch moving pan and tilt to cover all pan and tilt spectrum with fov delta. */
	  g3d_compute_visibility_for_given_entities(present_ents, agent, vis_result, present_ents_nb); 
	}
	  

	
	for(e_j=0; e_j<present_ents_nb; e_j++) {
	  ge_j = present_ents_global_idxs[e_j];
	  kn_on_ent = &agent->knowledge->entities[ge_j];
	  if ( kn_on_ent->visibility  ==  vis_result[e_j]) {
	    if ( kn_on_ent->visibility_ischanged)
	      kn_on_ent->visibility_ischanged  = FALSE;
	  }
	  else {
	    kn_on_ent->visibility = vis_result[e_j];
	    kn_on_ent->visibility_ischanged = TRUE;
	    kn_on_ent->visibility_isexported = FALSE;
	  }      
	}
      }

    
      for(e_i=0; e_i<present_ents_nb; e_i++) {
	ge_i = present_ents_global_idxs[e_i];
      
	ent = ents->entities[ge_i];
	kn_on_ent = &agent->knowledge->entities[ge_i];
      
	//printf("Testing: %s with %s\n", agent->robotPt->name, ent->robotPt->name);

	// 
	if(ent->is_pl_state_transition_new){
	  if((ent->pl_state_transition == HRI_APPEAR) || (ent->pl_state_transition == HRI_DISAPPEAR)){
	    kn_on_ent->disappeared_isexported = FALSE;
	  }
	  else if(((ent->pl_state_transition == HRI_START_MOVING) || (ent->pl_state_transition == HRI_STOP_MOVING) || (ent->pl_state_transition == HRI_APPEAR)) && (a_i == robotMyselfIndex)){
	    kn_on_ent->motion = ent->filtered_motion;
	    kn_on_ent->motion_ischanged = TRUE;
	    kn_on_ent->motion_isexported = FALSE;
	  }
	    
	}

	if(ents->needSituationAssessmentUpdate && ents->isWorldStatic){

	  // LOOKS AT / VISIBILITY PLACEMENT - FOV,FOA,OOF
	  // TODO: visibility placement for robot parts	
	  if(ent->disappeared)
	    res = HRI_UK_VIS_PLACE;
	  else
	    hri_entity_visibility_placement(agent, ent, &res, &elevation, &azimuth);

	  // need to update is_exported , is_changed?
	  kn_on_ent->is_placed_from_visibility = (HRI_VISIBILITY_PLACEMENT) res;
      
	  if ( (((HRI_VISIBILITY_PLACEMENT) res) == HRI_FOA) && (kn_on_ent->visibility  == HRI_VISIBLE)) {
	    if (kn_on_ent->is_looked_at != HRI_TRUE_V){
	      kn_on_ent->is_looked_at = HRI_TRUE_V;
	      kn_on_ent->is_looked_at_ischanged = TRUE;
	      kn_on_ent->is_looked_at_isexported = FALSE;
	    }	      	      
	  }
	  else {
	    if (kn_on_ent->is_looked_at == HRI_TRUE_V){
	      kn_on_ent->is_looked_at_ischanged = TRUE;
	      kn_on_ent->is_looked_at_isexported = FALSE;
	    }	 
	    if( (res == HRI_UK_VIS_PLACE) || (kn_on_ent->visibility  == HRI_UK_VIS))
	      kn_on_ent->is_looked_at = HRI_UK_V;
	    else
	      kn_on_ent->is_looked_at = HRI_FALSE_V;
	  }

	  // POINTS AT / POINTING PLACEMENT - FOV,FOA,OOF
	  // TODO: visibility placement for robot parts	
	  if(ent->disappeared)
	    res = HRI_UK_VIS_PLACE;
	  else
	    hri_entity_pointing_placement(agent, ent, &res, &elevation, &azimuth);
      
	  if ( (((HRI_VISIBILITY_PLACEMENT) res) == HRI_FOA) && (kn_on_ent->visibility  == HRI_VISIBLE)) {
	    if (kn_on_ent->is_pointed_at != HRI_TRUE_V){
	      kn_on_ent->is_pointed_at = HRI_TRUE_V;
	      kn_on_ent->is_pointed_at_ischanged = TRUE;
	      kn_on_ent->is_pointed_at_isexported = FALSE;
	    }	      	      
	  }
	  else {
	    if (kn_on_ent->is_pointed_at == HRI_TRUE_V){
	      kn_on_ent->is_pointed_at_ischanged = TRUE;
	      kn_on_ent->is_pointed_at_isexported = FALSE;
	    }	 
	    if( (res == HRI_UK_VIS_PLACE) || (kn_on_ent->visibility  == HRI_UK_VIS))
	      kn_on_ent->is_pointed_at = HRI_UK_V;
	    else
	      kn_on_ent->is_pointed_at = HRI_FALSE_V;
	  }

	  // REACHABILITY - REACHABLE, UNREACHABLE, HARDLY REACHABLE
	  // TODO: Fix this global variable use. It's ugly.     
	  // To simplify we do not compute reachability on agent or agent parts
	  if ( (ent->type != HRI_AGENT_PART) && (ent->type != HRI_ISAGENT) && ent->can_disappear_and_move) {
	    GIK_VIS = 500;
	    if(ent->disappeared)
	      reachability_result = HRI_UK_REACHABILITY;
	    else
	      reachability_result = hri_is_reachable(ent, agent);
	    if ( kn_on_ent->reachability ==  reachability_result) {
	      if ( kn_on_ent->reachability_ischanged)
		kn_on_ent->reachability_ischanged = FALSE;
	    }
	    else {
	      kn_on_ent->reachability = reachability_result;
	      kn_on_ent->reachability_ischanged = TRUE;
	      kn_on_ent->reachability_isexported = FALSE;
	    }
	  }
	  // SPATIAL RELATION      
	  if( ent->type != HRI_AGENT_PART) {
	    if(ent->disappeared)
	      spatial_relation_result = HRI_UK_RELATION;
	    else
	      spatial_relation_result = hri_spatial_relation(ent, agent);
	    if ( kn_on_ent->is_located_from_agent ==  spatial_relation_result) {
	      if (kn_on_ent->spatial_relation_ischanged)
		kn_on_ent->spatial_relation_ischanged = FALSE;
	    }
	    else {
	      kn_on_ent->is_located_from_agent  = spatial_relation_result;
	      kn_on_ent->spatial_relation_ischanged = TRUE;
	      kn_on_ent->spatial_relation_isexported = FALSE;
	    }
	  }
       
      
	  // PLACEMENT RELATION
	  for(e_j=0; e_j<present_ents_nb; e_j++) {
	    ge_j = present_ents_global_idxs[e_j];
	    // do not compute placement relations that involve an agent or an agent part
	    /* if( ((ent->type == HRI_AGENT_PART) || (ent->type == HRI_ISAGENT)) || !ent->can_disappear_and_move || ((ents->entities[ge_j]->type == HRI_AGENT_PART) || (ents->entities[ge_j]->type == HRI_ISAGENT)) ) { */
	    /*   continue; */
	    /* } */

	    // We want to know wether objects are on furniture, on placemat or inside a container
	    // Wa also want to know on which furnitures are placemat
	    if( ((ent->type == HRI_MOVABLE_OBJECT) && ((ents->entities[ge_j]->type == HRI_MOVABLE_OBJECT) || (ents->entities[ge_j]->type == HRI_OBJECT_SUPPORT) || (ents->entities[ge_j]->type == HRI_OBJECT_CONTAINER) || (ents->entities[ge_j]->type == HRI_OBJECT_PLACEMAT))) || ((ent->type == HRI_OBJECT_PLACEMAT) && (ents->entities[ge_j]->type == HRI_OBJECT_SUPPORT))) {

	      if( e_j != e_i) {
		if(ent->disappeared || ents->entities[ge_j]->disappeared)
		  placement_relation_result = HRI_UK_PLR;
		else
		  placement_relation_result = hri_placement_relation(ent, ents->entities[ge_j]);
		if (  kn_on_ent->is_placed[ge_j] ==  placement_relation_result) {
		  if ( kn_on_ent->placement_relation_ischanged[ge_j])
		    kn_on_ent->placement_relation_ischanged[ge_j] = FALSE;
		}
		else {
		  kn_on_ent->is_placed_old [ge_j] = kn_on_ent->is_placed[ge_j];
		  kn_on_ent->is_placed[ge_j] = placement_relation_result;
		  kn_on_ent->placement_relation_ischanged[ge_j] = TRUE;
		  kn_on_ent->placement_relation_isexported[ge_j] = FALSE;
		}
	      }
	    }
	  }
	}
      }
      
    }

    // all placement state transition events have been managed
    for(e_i=0; e_i<present_ents_nb; e_i++) {
      ge_i = present_ents_global_idxs[e_i];      
      ent = ents->entities[ge_i];
      if(ent->is_pl_state_transition_new)
	ent->is_pl_state_transition_new = FALSE;
    }

    MY_FREE(vis_result, HRI_VISIBILITY, ents->entities_nb); // FREE
    MY_FREE(present_ents, HRI_ENTITY*, ents->entities_nb); // FREE
    MY_FREE(present_ents_global_idxs, int, ents->entities_nb); // FREE

  }
  // Events in the Wolrd have been managed.
  if(ents->eventsInTheWorld)
    ents->eventsInTheWorld = FALSE;

  if(ents->needSituationAssessmentUpdate && ents->isWorldStatic)
    ents->needSituationAssessmentUpdate = FALSE;
  return counter;
}
