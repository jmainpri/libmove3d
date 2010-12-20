#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Hri_planner-pkg.h"

HRI_ENTITIES * GLOBAL_ENTITIES = NULL;

HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent)
{
  HRI_KNOWLEDGE * kn;

  kn = MY_ALLOC(HRI_KNOWLEDGE, 1);

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
  char* objectrealname;

  entities = MY_ALLOC(HRI_ENTITIES, 1);
  entities->entities = NULL;
  ent_i = 0;

  for(i=0; i<env->nr; i++) {
    if(!strcasestr(env->robot[i]->name,"GRIPPER") && !strcasestr(env->robot[i]->name,"VISBALL") && !strcasestr(env->robot[i]->name,"SAHandRight")) {

      if(!strcasestr(env->robot[i]->name,"CHAIR") && !strcasestr(env->robot[i]->name,"TABLE") ) {
        entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY*, ent_i, ent_i+1);
        entities->entities[ent_i] = MY_ALLOC(HRI_ENTITY,1);
        entities->entities[ent_i]->can_disappear = TRUE;
        entities->entities[ent_i]->is_present = FALSE;
        entities->entities[ent_i]->disappeared = FALSE;
        entities->entities[ent_i]->robotPt = env->robot[i];
        entities->entities[ent_i]->partPt = NULL;
        entities->entities[ent_i]->type = HRI_OBJECT;
        ent_i++;
      }

      for(j=0; j<env->robot[i]->no; j++) {
        objectrealname = strrchr(env->robot[i]->o[j]->name, '.');
        if(!strcasestr(objectrealname,"GHOST") &&
           (strcasestr(objectrealname,"SURFACE") || strcasestr(objectrealname,"HAND") ||
            strcasestr(objectrealname,"HEAD")    || strcasestr(objectrealname,"CAMERA")) ) {
             entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY*, ent_i, ent_i+1);
             entities->entities[ent_i] = MY_ALLOC(HRI_ENTITY,1);
             entities->entities[ent_i]->can_disappear = TRUE;
             entities->entities[ent_i]->is_present = FALSE;
             entities->entities[ent_i]->disappeared = FALSE;
             entities->entities[ent_i]->robotPt = env->robot[i];
             entities->entities[ent_i]->partPt = env->robot[i]->o[j];
             entities->entities[ent_i]->type = HRI_OBJECT_PART;
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
            agents->all_agents[agent_idx]->head_idx = MY_REALLOC(agents->all_agents[agent_idx]->head_idx, int,
                                                                 agents->all_agents[agent_idx]->head_nb,
                                                                 agents->all_agents[agent_idx]->head_nb+1);
            agents->all_agents[agent_idx]->head_idx[agents->all_agents[agent_idx]->head_nb++] = i;

          }
          if(strcasestr(entities->entities[i]->partPt->name, "hand")) {
            agents->all_agents[agent_idx]->hand_idx = MY_REALLOC(agents->all_agents[agent_idx]->hand_idx, int,
                                                                 agents->all_agents[agent_idx]->hand_nb,
                                                                 agents->all_agents[agent_idx]->hand_nb+1);
            agents->all_agents[agent_idx]->hand_idx[agents->all_agents[agent_idx]->hand_nb++] = i;
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
    if(entities->entities[i]->type == HRI_OBJECT_PART || entities->entities[i]->type == HRI_AGENT_PART)
      strcpy(knowledge->entities[i].name, entities->entities[i]->partPt->name);
    else
      strcpy(knowledge->entities[i].name, entities->entities[i]->robotPt->name);

    knowledge->entities[i].motion = HRI_UK_MOTION;
    knowledge->entities[i].is_placed_from_visibility = HRI_UK_VIS_PLACE;

    knowledge->entities[i].visibility = HRI_UK_VIS;
    knowledge->entities[i].reachability = HRI_UK_REACHABILITY;

    knowledge->entities[i].is_located_from_agent = HRI_UK_RELATION;

    knowledge->entities[i].is_placed = MY_ALLOC(HRI_PLACEMENT_RELATION, entities->entities_nb);
    knowledge->entities[i].is_placed_nb = entities->entities_nb;

    for(j=0; j<knowledge->entities[i].is_placed_nb; j++) {
      knowledge->entities[i].is_placed[j] = HRI_UK_PLR;
    }
  }

  return TRUE;
}

// TODO: There is a serious problem on the bounding boxes of robot parts.
// I think the reason is that in a macro the first body is takeninto account ghost <->real switch
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
  /* Condition 2: The lower part of topObj BB souldn not be higher than 5 cm from the higher part of bottomObj BB */

  if((topObjC[0] >= bottomObjBB->xmin) && (topObjC[0] <= bottomObjBB->xmax) &&
     (topObjC[1] >= bottomObjBB->ymin) && (topObjC[1] <= bottomObjBB->ymax) &&
     (topObjC[2] >= bottomObjBB->zmax))
    if((topObjBB->zmin > bottomObjBB->zmax) && (topObjBB->zmin-bottomObjBB->zmax < 0.05))
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
    if(ents->entities[i]->type ==HRI_OBJECT_PART || ents->entities[i]->type ==HRI_AGENT_PART)
      printf("%d - ENTITY name: %s type: %d\n", i, ents->entities[i]->partPt->name, ents->entities[i]->type);
    else
      printf("%d - ENTITY name: %s type: %d\n", i, ents->entities[i]->robotPt->name, ents->entities[i]->type);
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
    printf("%s\n", kn->entities[i].name);

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


// Function computing geometric facts between agents and objects
// Each agent has its own view of the environment.

int hri_compute_geometric_facts(HRI_AGENTS * agents, HRI_ENTITIES * ents)
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


  if(agents == NULL || ents == NULL) {
    printf("Not Initialized\n");
    return FALSE;
  }

  vis_result = MY_ALLOC(HRI_VISIBILITY, ents->entities_nb); // ALLOC
  present_ents = MY_ALLOC(HRI_ENTITY*, ents->entities_nb); // ALLOC
  present_ents_global_idxs = MY_ALLOC(int, ents->entities_nb); // ALLOC

  // Pick entities that exist and not disappeared
  present_ents_nb = 0;
  for(e_i=0; e_i<ents->entities_nb; e_i++) {
    if(ents->entities[e_i]->is_present && !ents->entities[e_i]->disappeared) {
      present_ents[present_ents_nb] = ents->entities[e_i];
      present_ents_global_idxs[present_ents_nb] = e_i;
      present_ents_nb++;
    }
  }
  
  for(a_i=0; a_i<agents->all_agents_no; a_i++) {
    agent = agents->all_agents[a_i];
    
    if(agent->is_present == FALSE)
      continue;
    
    for(e_i=0; e_i<present_ents_nb; e_i++) {
      ge_i = present_ents_global_idxs[e_i];
      
      ent = ents->entities[ge_i];
      kn_on_ent = &agent->knowledge->entities[ge_i];
      
      printf("Testing: %s with %s\n", agent->robotPt->name, ent->robotPt->name);

      // If the entity is a part of the current agent, we skip it since it doesn't make sense to compute it from his own point of view
      // TODO: Or does it?
      if( (ent->type == HRI_AGENT_PART) || (ent->type == HRI_ISAGENT) ) {
        if( agent == agents->all_agents[ent->agent_idx] )
          continue;
      }

      // VISIBILITY PLACEMENT - FOV,FOA,OOF
      // TODO: visibility placement for robot parts
      hri_object_visibility_placement(agent, ent->robotPt, &res, &elevation, &azimuth);
      kn_on_ent->is_placed_from_visibility = (HRI_VISIBILITY_PLACEMENT) res;

      // REACHABILITY - REACHABLE, UNREACHABLE, HARDLY REACHABLE
      // TODO: Fix this global variable use. It's ugly.
      // GIK_VIS = 500;
      // kn_on_ent->reachability = hri_is_reachable(ent, agent);

      // PLACEMENT RELATION
      kn_on_ent->is_located_from_agent = hri_spatial_relation(ent, agent);

      // SPATIAL RELATION
      for(e_j=0; e_j<present_ents_nb; e_j++) {
        ge_j = present_ents_global_idxs[e_j];
        if( e_j != e_i)
          kn_on_ent->is_placed[ge_j] = hri_placement_relation(ent, ents->entities[ge_j]);
      }
    }

    // VISIBLITY
    g3d_compute_visibility_for_given_entities(present_ents, agent, vis_result, present_ents_nb);

    for(e_j=0; e_j<present_ents_nb; e_j++) {
      ge_j = present_ents_global_idxs[e_j];
      kn_on_ent = &agent->knowledge->entities[ge_j];

      kn_on_ent->visibility = vis_result[e_j];
    }
  }


  MY_FREE(vis_result, HRI_VISIBILITY, ents->entities_nb); // FREE
  MY_FREE(present_ents, HRI_ENTITY*, ents->entities_nb); // FREE
  MY_FREE(present_ents_global_idxs, int, ents->entities_nb); // FREE


  return counter;
}
