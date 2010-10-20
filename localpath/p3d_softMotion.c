#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "../lightPlanner/proto/ManipulationStruct.h"
#include <stdio.h>

char * array_group_name[] = {
  (char*)"base",
  (char*)"freeflyer",
  (char*)"joint"
};
int P3D_NB_GROUP = 3;

ptr_to_softMotion_groupplanner array_softMotion_groupplanner[]= {
  (NULL),
  (int (*)(p3d_rob*, int, p3d_group_type, p3d_softMotion_data* , int*))(p3d_softMotion_localplanner_FREEFLYER),
  (int (*)(p3d_rob*, int, p3d_group_type, p3d_softMotion_data* , int*))(p3d_softMotion_localplanner_JOINT)
};

p3d_group_type p3d_group_getid_group(const char * name) {
  int i;
  for(i=0; i<P3D_NB_GROUP; i++) {
    if (strcmp(name, array_group_name[i]) == 0)
      { return (p3d_group_type)i; }
  }
  return (p3d_group_type)P3D_NULL_OBJ;
}

p3d_group_type p3d_group_getType_group(int nblpGp) {
  p3d_rob * robotPt = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);

  return robotPt->mlp->mlpJoints[nblpGp]->gpType;
}

int p3d_group_planner(p3d_rob* robotPt, int nblpGp, p3d_group_type gpType, p3d_softMotion_data* softMotion_data, int *iksol) {
  if(gpType >= P3D_NB_GROUP) {
    return FALSE;
  }
  return array_softMotion_groupplanner[gpType](robotPt, nblpGp, gpType, softMotion_data, iksol);
}

////////////////////////////////////////////////
//             PLANNER FUNCTIONS		 					//
////////////////////////////////////////////////
/*
 * SoftMotion local planner
 *
 * Input:  the robot, the softMotion_data and three configurations
 *
 * Output: a local path.
 *
 * Allocation: the initial, goal and next goal config are copied
 */
/**
 * SoftMotion local planner
 * @param robotPt The robot
 * @param multiLocalpathID The id of the group stored in the robot struct
 * @param softMotion_data The softMotion data struct
 * @param qi initial config
 * @param qf final config
 * @param qfp1
 * @param ikSol
 * @return the localpath pointer
 */
p3d_localpath *p3d_softMotion_localplanner(p3d_rob *robotPt, int multiLocalpathID, p3d_softMotion_data* softMotion_data, configPt qi, configPt qf, configPt qfp1, int* ikSol) {
  p3d_localpath *localpathPt = NULL;
  p3d_group_type gpType;

  /* on verifie que les configurations de depart et d'arrivee existent */
  if(qi == NULL){
    PrintInfo((("MP: p3d_softMotion_localplanner: no start configuration\n")));
    p3d_set_search_status(P3D_ILLEGAL_START);
    return(NULL);
  }
  if(qf == NULL){
    PrintInfo((("MP: p3d_softMotion_localplanner: no goal configuration\n")));
    p3d_set_search_status(P3D_ILLEGAL_GOAL);
    return(NULL);
  }
  if(qfp1 == NULL){
    PrintInfo((("MP: p3d_softMotion_localplanner: no goalp1 configuration\n")));
    p3d_set_search_status(P3D_ILLEGAL_GOAL);
    return(NULL);
  }

  if(p3d_get_search_verbose()){
    PrintInfo(("MP: p3d_softMotion_localplanner : "));
    PrintInfo(("qi=("));
    print_config(robotPt, qi);
    PrintInfo((") ; "));
    PrintInfo(("qf=("));
    print_config(robotPt, qf);
    PrintInfo(("=("));
    PrintInfo((")\n"));
  }

  softMotion_data->q_init  = p3d_copy_config(robotPt, qi);
  softMotion_data->q_end   = p3d_copy_config(robotPt, qf);
  softMotion_data->q_endp1 = p3d_copy_config(robotPt, qfp1);
  softMotion_data->isPlanned = FALSE;

  if (multiLocalpathID>=0 && multiLocalpathID<=robotPt->mlp->nblpGp) {
    gpType = p3d_group_getType_group(multiLocalpathID);
    softMotion_data->isPlanned = TRUE;
    if(p3d_group_planner(robotPt, multiLocalpathID, gpType, softMotion_data, ikSol)!= TRUE) {
      return NULL;
    }

  } else {
    PrintError(("p3d_softMotion_lp: Wrong multiLocalpathID\n"));
    return NULL;
  }

  /* Allocation of the localpath */
  localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_data, 0, TRUE);
  /* Here localpathPt can't be NULL but just in case ... */
  if (localpathPt == NULL) {
    PrintError(("p3d softMotion localpath return NULL at the end\n"));
  }
  localpathPt->mlpID = multiLocalpathID;
  p3d_set_search_status(P3D_SUCCESS);
  localpathPt->ikSol = ikSol;
  localpathPt->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt->nbActiveCntrts));
  return(localpathPt);
}

int p3d_softMotion_localplanner_FREEFLYER(p3d_rob* robotPt, int mlpId, p3d_group_type gpType, p3d_softMotion_data* softMotion_data, int *iksol) {
  int equal = 0;
  int index_dof = robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[0]]->index_dof;
  int nb_dof = 0;
  int nbJoints = robotPt->mlp->mlpJoints[mlpId]->nbJoints;

  // 	SM_POSELIMITS poseLimits;
  int i=0;
  Gb_v3 poseAngInit, poseLinInit, poseAngEnd, poseLinEnd;
  configPt q_init= NULL, q_end = NULL;
  Gb_th thInit, thEnd;

  p3d_matrix4 freeflyerPose_init, freeflyerPose_end;
  Gb_quat quatInit, quatEnd;
  Gb_dep depDelta;

  /* If initconfPt == goalconfPt, free initconfPt and goalconfPt and return NULL */
  for(int v=0; v<nbJoints; v++) {
    nb_dof += robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[v]]->user_dof_equiv_nbr;
  }
  equal = p3d_equal_config_n_offset(nb_dof, index_dof, softMotion_data->q_init, softMotion_data->q_end);

  if(equal && softMotion_data->isPTP == TRUE) {
    PrintInfo((("MP: p3d_softMotion_localplanner FREEFLYER: q_init = q_goal! \n")));
    p3d_set_search_status(P3D_CONFIG_EQUAL);
    return FALSE;
  }

  if(softMotion_data->isPTP == TRUE) {
    // It's a point to point motion
    p3d_mat4PosReverseOrder(freeflyerPose_init, softMotion_data->q_init[index_dof],
			    softMotion_data->q_init[index_dof+1],
			    softMotion_data->q_init[index_dof+2],
			    softMotion_data->q_init[index_dof+3],
			    softMotion_data->q_init[index_dof+4],
			    softMotion_data->q_init[index_dof+5]);

    p3d_mat4PosReverseOrder(freeflyerPose_end, softMotion_data->q_end[index_dof],
			    softMotion_data->q_end[index_dof+1],
			    softMotion_data->q_end[index_dof+2],
			    softMotion_data->q_end[index_dof+3],
			    softMotion_data->q_end[index_dof+4],
			    softMotion_data->q_end[index_dof+5]);

    lm_convert_p3dMatrix_To_GbTh(freeflyerPose_init ,&thInit);
    lm_convert_p3dMatrix_To_GbTh(freeflyerPose_end,&thEnd);
    //Same method as Gb_quat_interpole
    Gb_th_quat(&thInit, &quatInit);
    Gb_th_quat(&thEnd, &quatEnd);

    //set translation to null to compute the rotation displacement
    quatInit.x = 0.0;
    quatInit.y = 0.0;
    quatInit.z = 0.0;
    quatEnd.x = 0.0;
    quatEnd.y = 0.0;
    quatEnd.z = 0.0;
    Gb_quat_compute_relativeDep_to_interpole(&quatInit, &quatEnd, &depDelta);

    if(SOFT_MOTION_PRINT_DATA) {
      PrintInfo(("first point x=%f y=%f z=%f\n",softMotion_data->q_init[index_dof], softMotion_data->q_init[index_dof+1], softMotion_data->q_init[index_dof+2]));
      PrintInfo(("middle point x=%f y=%f z=%f\n",softMotion_data->q_end[index_dof], softMotion_data->q_end[index_dof+1], softMotion_data->q_end[index_dof+2]));
    }
    depDelta.x = thEnd.vp.x - thInit.vp.x;
    depDelta.y = thEnd.vp.y - thInit.vp.y;
    depDelta.z = thEnd.vp.z - thInit.vp.z;
    Gb_v3_set(&poseAngInit, 0.0, 0.0, 0.0);
    Gb_v3_set(&poseLinInit, 0.0, 0.0, 0.0);
    Gb_v3_set(&poseLinEnd, depDelta.x, depDelta.y, depDelta.z);
    Gb_v3_set(&poseAngEnd, depDelta.rx*depDelta.a, depDelta.ry*depDelta.a, depDelta.rz*depDelta.a);

    q_init = p3d_copy_config(robotPt, softMotion_data->q_init);
    q_end = p3d_copy_config(robotPt, softMotion_data->q_end);
    for(int v=index_dof; v<index_dof + softMotion_data->nbDofs; v++ ) {
      q_init[v] = 0.0;
    }

    q_end[index_dof]   = poseLinEnd.x;
    q_end[index_dof+1] = poseLinEnd.y;
    q_end[index_dof+2] = poseLinEnd.z;
    q_end[index_dof+3] = poseAngEnd.x;
    q_end[index_dof+4] = poseAngEnd.y;
    q_end[index_dof+5] = poseAngEnd.z;

    /* Set initial and final conditions (SM_COND IC and SM_COND FC structures needed by the planner) in softMotion_data */
    lm_set_cond_softMotion_data(index_dof, softMotion_data->nbDofs, q_init, q_end, softMotion_data->specific->velInit, softMotion_data->specific->velEnd, softMotion_data->specific->accInit, softMotion_data->specific->accEnd, softMotion_data);

    /* Compute the point to point motion */
    if(sm_ComputeSoftMotionPointToPoint_gen(softMotion_data->nbDofs, softMotion_data->specific->J_max, softMotion_data->specific->A_max, softMotion_data->specific->V_max, softMotion_data->specific->motion)!=0) {
      PrintError(("p3d softMotion localpath CANNOT compute point to point motion on group JOINT\n"));
      return FALSE;
    }
    /* Determine the motion duration */
    softMotion_data->specific->motionTime = 0.0;
    for(i=0;i<softMotion_data->nbDofs;i++) {
      if(softMotion_data->specific->motion[i].MotionDuration > softMotion_data->specific->motionTime) {
	softMotion_data->specific->motionTime = softMotion_data->specific->motion[i].MotionDuration;
      }
    }

    p3d_destroy_config(robotPt, q_init);
    p3d_destroy_config(robotPt, q_end);
    return TRUE;
  }
  else {
    /* PTP motion is already computed */

    p3d_mat4PosReverseOrder(freeflyerPose_init, softMotion_data->q_init[index_dof],
			    softMotion_data->q_init[index_dof+1],
			    softMotion_data->q_init[index_dof+2],
			    softMotion_data->q_init[index_dof+3],
			    softMotion_data->q_init[index_dof+4],
			    softMotion_data->q_init[index_dof+5]);

    p3d_mat4PosReverseOrder(freeflyerPose_end, softMotion_data->q_end[index_dof],
			    softMotion_data->q_end[index_dof+1],
			    softMotion_data->q_end[index_dof+2],
			    softMotion_data->q_end[index_dof+3],
			    softMotion_data->q_end[index_dof+4],
			    softMotion_data->q_end[index_dof+5]);

    lm_convert_p3dMatrix_To_GbTh(freeflyerPose_init ,&thInit);
    lm_convert_p3dMatrix_To_GbTh(freeflyerPose_end,&thEnd);
    //Same method as Gb_quat_interpole
    Gb_th_quat(&thInit, &quatInit);
    Gb_th_quat(&thEnd, &quatEnd);

    //set translation to null to compute the rotation displacement
    quatInit.x = 0.0;
    quatInit.y = 0.0;
    quatInit.z = 0.0;
    quatEnd.x = 0.0;
    quatEnd.y = 0.0;
    quatEnd.z = 0.0;
    Gb_quat_compute_relativeDep_to_interpole(&quatInit, &quatEnd, &depDelta);

    if(SOFT_MOTION_PRINT_DATA) {
      PrintInfo(("first point x=%f y=%f z=%f\n",softMotion_data->q_init[index_dof], softMotion_data->q_init[index_dof+1], softMotion_data->q_init[index_dof+2]));
      PrintInfo(("middle point x=%f y=%f z=%f\n",softMotion_data->q_end[index_dof], softMotion_data->q_end[index_dof+1], softMotion_data->q_end[index_dof+2]));
    }
    depDelta.x = thEnd.vp.x - thInit.vp.x;
    depDelta.y = thEnd.vp.y - thInit.vp.y;
    depDelta.z = thEnd.vp.z - thInit.vp.z;
    Gb_v3_set(&poseAngInit, 0.0, 0.0, 0.0);
    Gb_v3_set(&poseLinInit, 0.0, 0.0, 0.0);
    Gb_v3_set(&poseLinEnd, depDelta.x, depDelta.y, depDelta.z);
    Gb_v3_set(&poseAngEnd, depDelta.rx*depDelta.a, depDelta.ry*depDelta.a, depDelta.rz*depDelta.a);

    q_init = p3d_copy_config(robotPt, softMotion_data->q_init);
    q_end = p3d_copy_config(robotPt, softMotion_data->q_end);
    for(int v=index_dof; v<index_dof + softMotion_data->nbDofs; v++ ) {
      q_init[v] = 0.0;
    }

    q_end[index_dof]   = poseLinEnd.x;
    q_end[index_dof+1] = poseLinEnd.y;
    q_end[index_dof+2] = poseLinEnd.z;
    q_end[index_dof+3] = poseAngEnd.x;
    q_end[index_dof+4] = poseAngEnd.y;
    q_end[index_dof+5] = poseAngEnd.z;

    /* Set initial and final conditions (SM_COND IC and SM_COND FC structures needed by the planner) in softMotion_data */
    lm_set_cond_softMotion_data(index_dof, softMotion_data->nbDofs, q_init, q_end, softMotion_data->specific->velInit, softMotion_data->specific->velEnd, softMotion_data->specific->accInit, softMotion_data->specific->accEnd, softMotion_data);

    if(lm_compute_softMotion(robotPt, mlpId, softMotion_data) == FALSE) {
      return FALSE;
    }
    /* Determine the motion duration */
    softMotion_data->specific->motionTime = 0.0;
    for(i=0;i<softMotion_data->nbDofs;i++) {
      if(softMotion_data->specific->motion[i].MotionDuration > softMotion_data->specific->motionTime) {
	softMotion_data->specific->motionTime = softMotion_data->specific->motion[i].MotionDuration;
      }
    }
    p3d_destroy_config(robotPt, q_init);
    p3d_destroy_config(robotPt, q_end);
    return TRUE;
  }
}

int p3d_softMotion_localplanner_JOINT(p3d_rob* robotPt, int mlpId, p3d_group_type gpType, p3d_softMotion_data* softMotion_data, int *iksol) {

  int equal = 0;
  int index_dof = robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[0]]->index_dof;
  int nbDofs = 0;
  int nbJoints = robotPt->mlp->mlpJoints[mlpId]->nbJoints;
  // 	SM_POSELIMITS poseLimits;
  int i=0;
  configPt q_init, q_end;

  /* If initconfPt == goalconfPt, free initconfPt and goalconfPt and return NULL */
  for(int v=0; v<nbJoints; v++) {
    nbDofs += robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[v]]->user_dof_equiv_nbr;
  }
  equal = p3d_equal_config_n_offset(nbDofs, index_dof, softMotion_data->q_init, softMotion_data->q_end);

  if(equal && softMotion_data->isPTP == TRUE) {
    // 		PrintInfo((("MP: p3d_softMotion_localplanner JOINT: q_init = q_goal! \n")));
    p3d_set_search_status(P3D_CONFIG_EQUAL);
    return FALSE;
  }

  if(softMotion_data->isPTP == TRUE) {
    // It's a point to point motion
    /* Set initial and final conditions (SM_COND IC and SM_COND FC structures needed by the planner) in softMotion_data */
    q_init = p3d_copy_config(robotPt, softMotion_data->q_init);
    q_end = p3d_copy_config(robotPt, softMotion_data->q_end);

    for(int v=0; v<softMotion_data->nbDofs; v++) {
      q_end[index_dof +v] = q_end[index_dof +v] - q_init[index_dof +v];
      q_init[index_dof +v] = 0.0;
    }
    lm_set_cond_softMotion_data(index_dof, nbDofs, q_init, q_end, softMotion_data->specific->velInit, softMotion_data->specific->velEnd, softMotion_data->specific->accInit, softMotion_data->specific->accEnd, softMotion_data);
    p3d_destroy_config(robotPt, q_init);
    p3d_destroy_config(robotPt, q_end);

    if(sm_ComputeSoftMotionPointToPoint_gen(softMotion_data->nbDofs, softMotion_data->specific->J_max, softMotion_data->specific->A_max, softMotion_data->specific->V_max, softMotion_data->specific->motion)!=0) {
      PrintError(("p3d softMotion localpath CANNOT compute point to point motion on group JOINT\n"));
      return FALSE;
    }
    /* Determine the motion duration */
    softMotion_data->specific->motionTime = 0.0;
    for(i=0;i<softMotion_data->nbDofs;i++) {
      if(softMotion_data->specific->motion[i].MotionDuration > softMotion_data->specific->motionTime) {
	softMotion_data->specific->motionTime = softMotion_data->specific->motion[i].MotionDuration;
      }
    }
    return TRUE;
  }
  else {
    /* PTP motion is already computed */
    q_init = p3d_copy_config(robotPt, softMotion_data->q_init);
    q_end = p3d_copy_config(robotPt, softMotion_data->q_end);

    for(int v=0; v<softMotion_data->nbDofs; v++) {
      q_end[index_dof +v] = q_end[index_dof +v] - q_init[index_dof +v];
      q_init[index_dof +v] = 0.0;
    }
    lm_set_cond_softMotion_data(index_dof, nbDofs, q_init, q_end, softMotion_data->specific->velInit, softMotion_data->specific->velEnd, softMotion_data->specific->accInit, softMotion_data->specific->accEnd, softMotion_data);

    if(lm_compute_softMotion(robotPt, mlpId, softMotion_data) == FALSE) {
      return FALSE;
    }
    /* Determine the motion duration */
    softMotion_data->specific->motionTime = 0.0;
    for(i=0;i<softMotion_data->nbDofs;i++) {
      if(softMotion_data->specific->motion[i].MotionDuration > softMotion_data->specific->motionTime) {
	softMotion_data->specific->motionTime = softMotion_data->specific->motion[i].MotionDuration;
      }
    }
    p3d_destroy_config(robotPt, q_init);
    p3d_destroy_config(robotPt, q_end);
    return TRUE;
  }
  return TRUE;
}

/*
 *   compute softMotion for the localpath
 *
 */
int lm_compute_softMotion(p3d_rob* robotPt, int mlpID, p3d_softMotion_data* softMotion_data){
  SM_LIMITS auxLimits;
  SM_COND IC, FC;
  int axisMotionMax = 0;
  double GD = 0.0;
  int i = 0;
  double timeMotionMax = 0.0;
  int adjustTimeError = 0;
  SM_MOTION_MONO *motion3seg;

  if ((motion3seg = MY_ALLOC(SM_MOTION_MONO, softMotion_data->nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return FALSE;
  }

  for (i=0; i < softMotion_data->nbDofs; i++) {
    softMotion_data->specific->motion[i].motionIsAdjusted = 0;
    auxLimits.maxJerk = softMotion_data->specific->J_max[i];
    auxLimits.maxAcc  = softMotion_data->specific->A_max[i];
    auxLimits.maxVel  = softMotion_data->specific->V_max[i];
    softMotion_data->specific->motion[i].jerk.J1 = softMotion_data->specific->J_max[i];
    softMotion_data->specific->motion[i].jerk.sel = 1;
    IC.a =  softMotion_data->specific->motion[i].IC.a;
    IC.v =  softMotion_data->specific->motion[i].IC.v;
    IC.x =  0.0;
    FC.a = softMotion_data->specific->motion[i].FC.a;
    FC.v = softMotion_data->specific->motion[i].FC.v;
    FC.x = (softMotion_data->specific->motion[i].FC.x - softMotion_data->specific->motion[i].IC.x);

    if (sm_ComputeSoftMotion( IC, FC, auxLimits, &(softMotion_data->specific->motion[i].Times), &(softMotion_data->specific->motion[i].Dir))!=0) {
      printf("ERROR Jerk Profile on dim %d\n",i);
      return FALSE;
    }
    /* Get initial conditions for each vectors Acc Vel and Pos */
    GD =  FC.x * softMotion_data->specific->motion[i].Dir;
    if (sm_VerifyTimes( SM_DISTANCE_TOLERANCE, GD, softMotion_data->specific->motion[i].jerk, softMotion_data->specific->motion[i].IC, softMotion_data->specific->motion[i].Dir, softMotion_data->specific->motion[i].Times, &FC, &(softMotion_data->specific->motion[i].Acc), &(softMotion_data->specific->motion[i].Vel), &(softMotion_data->specific->motion[i].Pos), SM_ON)!=0) {
      printf(" Verify Times on dim %d\n",i);

      return FALSE;
    } else {
      if(SOFT_MOTION_PRINT_DATA) {
	printf(" Verify Times on dim %d is OK\n",i);
      }
    }
  }
  lm_set_and_get_motionTimes(softMotion_data, &timeMotionMax, &axisMotionMax);
  softMotion_data->specific->motionTime = timeMotionMax;

  /* Adjust Motion Times */
  adjustTimeError = 0;
  for (i=0; i < softMotion_data->nbDofs; i++) {
    if (i != axisMotionMax) {
      auxLimits.maxJerk = softMotion_data->specific->J_max[i];
      auxLimits.maxAcc  = softMotion_data->specific->A_max[i];
      auxLimits.maxVel  = softMotion_data->specific->V_max[i];
      softMotion_data->specific->motion[i].jerk.J1 = softMotion_data->specific->J_max[i];
      softMotion_data->specific->motion[i].jerk.sel = 1;
      IC.a =  softMotion_data->specific->motion[i].IC.a;
      IC.v =  softMotion_data->specific->motion[i].IC.v;
      IC.x =  0.0;
      FC.a = softMotion_data->specific->motion[i].FC.a;
      FC.v = softMotion_data->specific->motion[i].FC.v;
      FC.x = (softMotion_data->specific->motion[i].FC.x - softMotion_data->specific->motion[i].IC.x);

      if(sm_adjustMotionWith3seg( IC, FC, timeMotionMax, &motion3seg[i])!= 0) {
	printf("sm_AdjustTime ERROR 3seg at axis %d\n",i);
	adjustTimeError ++;
      }
    }
  }


  /* Replace old motion by adjusted motion */
  sm_SM_TIMES_copy_into(&(softMotion_data->specific->motion[axisMotionMax].Times), &motion3seg[axisMotionMax].Times);
  sm_SM_TIMES_copy_into(&(softMotion_data->specific->motion[axisMotionMax].TimesM), &motion3seg[axisMotionMax].TimesM);
  motion3seg[axisMotionMax].jerk.sel = 1;
  motion3seg[axisMotionMax].jerk.J1 = softMotion_data->specific->motion[axisMotionMax].jerk.J1;
  motion3seg[axisMotionMax].jerk.J2 = softMotion_data->specific->motion[axisMotionMax].jerk.J1;
  motion3seg[axisMotionMax].jerk.J3 =softMotion_data->specific->motion[axisMotionMax].jerk.J1;
  motion3seg[axisMotionMax].jerk.J4 = softMotion_data->specific->motion[axisMotionMax].jerk.J1;
  motion3seg[axisMotionMax].Dir = softMotion_data->specific->motion[axisMotionMax].Dir;
  motion3seg[axisMotionMax].Dir_a = softMotion_data->specific->motion[axisMotionMax].Dir;
  motion3seg[axisMotionMax].Dir_b = -softMotion_data->specific->motion[axisMotionMax].Dir;
  motion3seg[axisMotionMax].IC.a = softMotion_data->specific->motion[axisMotionMax].IC.a;
  motion3seg[axisMotionMax].IC.v = softMotion_data->specific->motion[axisMotionMax].IC.v;
  motion3seg[axisMotionMax].IC.x = softMotion_data->specific->motion[axisMotionMax].IC.x;
  motion3seg[axisMotionMax].FC.a = softMotion_data->specific->motion[axisMotionMax].FC.a;
  motion3seg[axisMotionMax].FC.v = softMotion_data->specific->motion[axisMotionMax].FC.v;
  motion3seg[axisMotionMax].FC.x = softMotion_data->specific->motion[axisMotionMax].FC.x;
  motion3seg[axisMotionMax].motionIsAdjusted = 0;
  motion3seg[axisMotionMax].MotionDuration = softMotion_data->specific->motion[axisMotionMax].MotionDuration ;
  motion3seg[axisMotionMax].MotionDurationM = softMotion_data->specific->motion[axisMotionMax].MotionDurationM ;
  motion3seg[axisMotionMax].TimeCumulM[0] = 0;
  motion3seg[axisMotionMax].TimeCumulM[1] = (int)motion3seg[axisMotionMax].TimesM.Tjpa;
  motion3seg[axisMotionMax].TimeCumulM[2] = (int)motion3seg[axisMotionMax].TimeCumulM[1] \
    + (int)motion3seg[axisMotionMax].TimesM.Taca;
  motion3seg[axisMotionMax].TimeCumulM[3] = (int)motion3seg[axisMotionMax].TimeCumulM[2] \
    + (int)motion3seg[axisMotionMax].TimesM.Tjna;
  motion3seg[axisMotionMax].TimeCumulM[4] = (int)motion3seg[axisMotionMax].TimeCumulM[3] \
    + (int)motion3seg[axisMotionMax].TimesM.Tvc;
  motion3seg[axisMotionMax].TimeCumulM[5] = (int)motion3seg[axisMotionMax].TimeCumulM[4] \
    + (int)motion3seg[axisMotionMax].TimesM.Tjnb;
  motion3seg[axisMotionMax].TimeCumulM[6] = (int)motion3seg[axisMotionMax].TimeCumulM[5] \
    + (int)motion3seg[axisMotionMax].TimesM.Tacb;

  motion3seg[axisMotionMax].TimeCumul[0] = 0.0;
  motion3seg[axisMotionMax].TimeCumul[1] = motion3seg[axisMotionMax].Times.Tjpa;
  motion3seg[axisMotionMax].TimeCumul[2] = motion3seg[axisMotionMax].TimeCumul[1] \
    + motion3seg[axisMotionMax].Times.Taca;
  motion3seg[axisMotionMax].TimeCumul[3] = motion3seg[axisMotionMax].TimeCumul[2] \
    + motion3seg[axisMotionMax].Times.Tjna;
  motion3seg[axisMotionMax].TimeCumul[4] = motion3seg[axisMotionMax].TimeCumul[3] \
    + motion3seg[axisMotionMax].Times.Tvc;
  motion3seg[axisMotionMax].TimeCumul[5] = motion3seg[axisMotionMax].TimeCumul[4] \
    + motion3seg[axisMotionMax].Times.Tjnb;
  motion3seg[axisMotionMax].TimeCumul[6] = motion3seg[axisMotionMax].TimeCumul[5] \
    + motion3seg[axisMotionMax].Times.Tacb;

  for(int v=0; v<softMotion_data->nbDofs; v++) {
    sm_copy_SM_MOTION_MONO_into(&motion3seg[v], &softMotion_data->specific->motion[v]);
  }

  if (adjustTimeError > 0) {
    printf("lm_compute_softMotion_for_r6Arm can't adjust time motion \n motion must be stopped at next configuration\n");
    return FALSE;
  }

  for (i=0; i < softMotion_data->nbDofs; i++) {
    IC.a =  softMotion_data->specific->motion[i].IC.a;
    IC.v =  softMotion_data->specific->motion[i].IC.v;
    IC.x =  softMotion_data->specific->motion[i].IC.x;
    FC.a = softMotion_data->specific->motion[i].FC.a;
    FC.v = softMotion_data->specific->motion[i].FC.v;
    FC.x = (softMotion_data->specific->motion[i].FC.x - softMotion_data->specific->motion[i].IC.x);
    /* Verify Times */
    if (sm_VerifyTimes_Dir_ab(SM_DISTANCE_TOLERANCE, FC.x, softMotion_data->specific->motion[i].jerk, IC,
			      softMotion_data->specific->motion[i].Dir_a, softMotion_data->specific->motion[i].Dir_b,
			      softMotion_data->specific->motion[i].Times, &FC, &(softMotion_data->specific->motion[i].Acc),
			      &(softMotion_data->specific->motion[i].Vel), &(softMotion_data->specific->motion[i].Pos)) != 0) {
      printf("lm_compute_softMotion_for_r6Arm ERROR Verify Times on axis [%d] \n",i);
      return FALSE;
    }
    // 				softMotion_data->freeflyer->motion.motionIsAdjusted[i] = 1;
  }
  lm_set_and_get_motionTimes(softMotion_data, &timeMotionMax, &axisMotionMax);
  softMotion_data->specific->motionTime = timeMotionMax;
  if(motion3seg != NULL) {
    MY_FREE(motion3seg, SM_MOTION_MONO, softMotion_data->nbDofs);
  }
  return TRUE;
}

///////////////////////////////////////////////
//             LOCALPATH FUNCTIONS	  	   	 //
///////////////////////////////////////////////
/*
 *  Compute the configuration situated at given param (time) on the local path.
 *
 *  Input:  the robot, the param.
 *
 *  Output: the configuration
 */
configPt p3d_softMotion_config_at_param(p3d_rob *robotPt, p3d_localpath *localpathPt, double param) {
  p3d_softMotion_data *softMotion_specificPt;
  configPt q = NULL;
  int i, j;
  double paramDiff = 0.0;
  double paramLocal = 0.0;
  int segId = 0;
  int index_dof = robotPt->joints[robotPt->mlp->mlpJoints[localpathPt->mlpID]->joints[0]]->index_dof;
  double *q_init = NULL;
  double *q_end = NULL;
  SM_SEGMENT *segment = NULL;
  SM_COND *condEnd = NULL;
  double angle = 0.0;
  Gb_th thMat , thInit, thMats;
  p3d_matrix4 p3dMat, freeflyerPose_init;
  Gb_dep gbDep;
  Gb_v3 gbV3Rot, gbV3Rotn;
  double Tx, Ty, Tz, Rx, Ry, Rz;

  if (localpathPt == NULL) {
    return NULL;
  }
  if (localpathPt->type_lp != SOFT_MOTION){
    PrintError(("p3d_softMotion_config_at_param: local path must be softMotion\n"));
    return NULL;
  }
  softMotion_specificPt = localpathPt->specific.softMotion_data;
  q = p3d_alloc_config(robotPt);

  if ((q_init = MY_ALLOC(double, softMotion_specificPt->nbDofs)) == NULL) {
    printf("  p3d_softMotion_config_at_param: allocation failed\n");
    return NULL;
  }
  if ((q_end = MY_ALLOC(double, softMotion_specificPt->nbDofs)) == NULL) {
    printf("  p3d_softMotion_config_at_param: allocation failed\n");
    return NULL;
  }
  if ((segment = MY_ALLOC(SM_SEGMENT, softMotion_specificPt->nbDofs)) == NULL) {
    printf("  p3d_softMotion_config_at_param: allocation failed\n");
    return NULL;
  }
  if ((condEnd = MY_ALLOC(SM_COND, softMotion_specificPt->nbDofs)) == NULL) {
    printf("  p3d_softMotion_config_at_param: allocation failed\n");
    return NULL;
  }
  j = 0;
  for(i=index_dof; i<(index_dof+softMotion_specificPt->nbDofs); i++) {
    q_init[j] = softMotion_specificPt->q_init[i];
    if(SOFT_MOTION_PRINT_DATA) {
      printf("q_init[%d]= %f\n",i,q_init[i]);
    }
    q_end[j] = softMotion_specificPt->q_end[i];
    j++;
  }
  if (param < 0) { param = 0.0;}
  if (param > localpathPt->range_param){ param = localpathPt->range_param;}
  for (i=0;i<softMotion_specificPt->nbDofs;i++) {
    lm_get_softMotion_segment_params( softMotion_specificPt, param, &segment[i], &segId, i);
    if (param >= softMotion_specificPt->specific->motionTime) {
      paramLocal = softMotion_specificPt->specific->motionTime;
    } else {
      paramLocal = param;
    }
    lm_get_paramDiff_for_param( softMotion_specificPt, &segment[i], segId, i, paramLocal, &paramDiff);
    sm_CalculOfAccVelPosAtTimeSecond(paramDiff, &segment[i], &condEnd[i]);
  }

  /* SWITCH WRT group */
  switch (robotPt->mlp->mlpJoints[localpathPt->mlpID]->gpType) {
  case FREEFLYER:
    p3d_mat4PosReverseOrder(freeflyerPose_init, 0.0, 0.0, 0.0, q_init[3], q_init[4], q_init[5]);
    lm_convert_p3dMatrix_To_GbTh(freeflyerPose_init ,&thInit);
    Gb_v3_set(&gbV3Rot,condEnd[3].x, condEnd[4].x,condEnd[5].x);
    angle = Gb_v3_norme(&gbV3Rot, &gbV3Rotn);
    Gb_dep_set(&gbDep, 0.0, 0.0, 0.0, gbV3Rotn.x, gbV3Rotn.y, gbV3Rotn.z, angle);
    Gb_dep_th(&gbDep, &thMat);
    Gb_th_produit(&thInit, &thMat, &thMats);
    thMats.vp.x = q_init[0] + condEnd[0].x;
    thMats.vp.y = q_init[1] + condEnd[1].x;
    thMats.vp.z = q_init[2] + condEnd[2].x;
    lm_convert_GbTh_To_p3dMatrix(&thMats,p3dMat);
    p3d_mat4ExtractPosReverseOrder(p3dMat,&Tx, &Ty, &Tz, &Rx, &Ry, &Rz);
    p3d_copy_config_into(robotPt, softMotion_specificPt->q_init, &q);
    q[index_dof]   = Tx;
    q[index_dof+1] = Ty;
    q[index_dof+2] = Tz;
    q[index_dof+3] = Rx;
    q[index_dof+4] = Ry;
    q[index_dof+5] = Rz;
    if(isnan(Rx)) { printf("isnan Rx\n");}
    if(isnan(Ry)) {printf("isnan Rx\n");}
    if(isnan(Rz)) {printf("isnan Rx\n");}
    break;
  case JOINT :
    p3d_copy_config_into(robotPt, softMotion_specificPt->q_init, &q);
    for(int v =0; v<softMotion_specificPt->nbDofs; v++) {
      q[index_dof+v]   = softMotion_specificPt->q_init[index_dof+v] + condEnd[v].x;
    }
    break;
  default:
    break;
  }
  MY_FREE(q_init, double, softMotion_specificPt->nbDofs);
  MY_FREE(q_end, double, softMotion_specificPt->nbDofs);
  MY_FREE(segment, SM_SEGMENT, softMotion_specificPt->nbDofs);
  MY_FREE(condEnd, SM_COND, softMotion_specificPt->nbDofs);
  return q;
}

/*  p3d_softMotion_stay_within_dist
 *
 *  Input:  the robot,
 *          the local path,
 *          the parameter along the curve,
 *          the direction of motion
 *          the maximal distance moved by all the points of the
 *          robot
 *
 *  Output: length of the interval of parameter the robot can
 *          stay on without any body moving by more than the input distance
 *
 *  Description:
 *          From a configuration on a local path, this function
 *          computes an interval of parameter on the local path on
 *          which all the points of the robot move by less than the
 *          distance given as input.  The interval is centered on the
 *          configuration given as input. The function returns the
 *          half length of the interval
 */
double p3d_softMotion_stay_within_dist(p3d_rob* robotPt, p3d_localpath* localpathPt,	double parameterIn, whichway dir,	double *distances) {
  p3d_softMotion_data *softMotion_specificPt = localpathPt->specific.softMotion_data;

  p3d_stay_within_dist_data * stay_within_dist_data = NULL;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt *cur_jntPt, *prev_jntPt;
  configPt q_max_param, q_param;
  double max_param, min_param;
  double range_param = 0.0;
  double parameter = 0.0;
  /* update length and range of parameter */
  range_param =  p3d_dist_config(robotPt, softMotion_specificPt->q_init, softMotion_specificPt->q_end);
  double value= 0.0 ;
  int minJnt = 0;

  parameter = (parameterIn/localpathPt->length_lp)*range_param;

  /* SWITCH WRT group */
  switch (robotPt->mlp->mlpJoints[localpathPt->mlpID]->gpType) {

  case FREEFLYER:
  case JOINT:
    /* store the data to compute the maximal velocities at the
       joint for each body of the robot */
    stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2);
    p3d_init_stay_within_dist_data(stay_within_dist_data);

    if (dir == FORWARD)
      {
	q_max_param = softMotion_specificPt->q_end;
	min_param = max_param = range_param - parameter;
      } else
      {
	q_max_param = softMotion_specificPt->q_init;
	min_param = max_param = parameter;
      }
    /* Get the current config to have the modifications of the constraints */
    /* Supose that q_init and q_goal respect cronstraints */
    q_param = p3d_get_robot_config(robotPt);
    //  q_param = localpathPt->config_at_param(robotPt,localpathPt,parameter);

    /* computation of the bounds for the linear and angular
       velocities of each body */
    for(i=0; i<=njnt; i++) {
      cur_jntPt = robotPt->joints[i];
      prev_jntPt = cur_jntPt->prev_jnt;

      /* j = index of the joint to which the current joint is attached */
      if (prev_jntPt==NULL)
	{ j = -1; } /* environment */
      else
	{ j = prev_jntPt->num; }
      double bakMinParam = min_param;

      p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
			       &(stay_within_dist_data[i+1]), &(distances[i]),
			       q_param, q_max_param, max_param, &min_param);
      if (min_param < bakMinParam){
	minJnt = cur_jntPt->num;
      }
      /* Rem: stay_within_dist_data[0] is bound to the environment */
    }

    MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);
    p3d_destroy_config(robotPt, q_param);

    if(max_param < min_param)  {
      return localpathPt->length_lp;
    }

    if(max_param  != 0.0) {
      value = (min_param / max_param)*localpathPt->length_lp;
    } else {
      value = 0.0;
    }
    // 			if(isnan(value)) {
    // 				printf("isnan value\n");
    // 				return localpathPt->length_lp;
    // 			}
    break;
  default:
    break;
  }



  return value;
}

/*
 *  Cost of a local path
 *
 *  Input:  the local path
 *
 *  Output: the cost
 */
double p3d_softMotion_cost(p3d_rob *robotPt, p3d_localpath *localpathPt) {
  return localpathPt->length_lp;
}

/*
 *  Extract from a linear local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */
p3d_localpath *p3d_extract_softMotion(p3d_rob *robotPt, p3d_localpath *localpathPt,	double l1, double l2) {
  configPt q1, q2;
  p3d_localpath *sub_localpathPt;
  p3d_softMotion_data *softMotion_data = NULL;

  softMotion_data = p3d_create_softMotion_data_multilocalpath(robotPt, localpathPt->mlpID);

  // 	if (l1 == l2) {
  // 		printf("p3d_extract_softMotion return NULL l1==l2\n");
  // 		return NULL;
  // 	}

  q1 = p3d_softMotion_config_at_param(robotPt, localpathPt, l1);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(robotPt, q1);
    p3d_get_robot_config_into(robotPt, &q1);
  }
  q2 = p3d_softMotion_config_at_param(robotPt, localpathPt, l2);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(robotPt, q2);
    p3d_get_robot_config_into(robotPt, &q2);
  }

  softMotion_data->gpType = localpathPt->specific.softMotion_data->gpType;
  softMotion_data->nbJoints = localpathPt->specific.softMotion_data->nbJoints;
  softMotion_data->nbDofs = localpathPt->specific.softMotion_data->nbDofs;
  softMotion_data->isPTP = localpathPt->specific.softMotion_data->isPTP ;
  softMotion_data->q_init  = p3d_copy_config(robotPt, q1);
  softMotion_data->q_end   = p3d_copy_config(robotPt, q2);

  for(int v=0; v<softMotion_data->nbDofs; v++) {
    softMotion_data->specific->J_max[v] = localpathPt->specific.softMotion_data->specific->J_max[v];
    softMotion_data->specific->A_max[v] = localpathPt->specific.softMotion_data->specific->A_max[v];
    softMotion_data->specific->V_max[v] = localpathPt->specific.softMotion_data->specific->V_max[v];
  }

  sub_localpathPt = p3d_softMotion_localplanner(robotPt, localpathPt->mlpID, softMotion_data, q1, q2, q2, localpathPt->ikSol);
  if(sub_localpathPt != NULL) {
    sub_localpathPt->mlpID = localpathPt->mlpID;
    p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
    sub_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
    sub_localpathPt->activeCntrts = MY_ALLOC(int, sub_localpathPt->nbActiveCntrts);
    for(int i = 0; i < sub_localpathPt->nbActiveCntrts; i++){
      sub_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
    }
  }
  return sub_localpathPt;
}

/*_with_velocities
 *  Extract from a linear local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */
p3d_localpath *p3d_extract_softMotion_with_velocities(p3d_rob *robotPt, p3d_localpath *localpathPt, double l1, double l2) {
  configPt q1, q2, q_init, q_end;
  p3d_localpath* sub_localpathPt = NULL;
  p3d_softMotion_data* softMotion_data_l1 = NULL;
  p3d_softMotion_data* softMotion_data = NULL;
  p3d_softMotion_data* softMotion_data_In = NULL;
  double ltmp=0.0, sum=0.0;
  int j=0, i=0;
  double paramDiffl1 = 0.0;
  double paramLocal = 0.0;
  int segIdl1 = 0;
  double paramDiffl2 = 0.0;
  int segIdl2 = 0;
  SM_SEGMENT *segmentl1 = NULL;
  SM_COND *condl1 = NULL;
  SM_SEGMENT *segmentl2 = NULL;
  SM_COND *condl2 = NULL;

  int index_dof = robotPt->joints[robotPt->mlp->mlpJoints[localpathPt->mlpID]->joints[0]]->index_dof;


  // 	if (l1 == l2) {
  // 		printf("p3d_extract_softMotion return NULL l1==l2\n");
  // 		return NULL;
  // 	}

  if (l1 > l2) {
    ltmp = l1;
    l1 = l2;
    l2 = l1;
  }

  q1 = p3d_softMotion_config_at_param(robotPt, localpathPt, l1);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(robotPt, q1);
    p3d_get_robot_config_into(robotPt, &q1);
  }
  q2 = p3d_softMotion_config_at_param(robotPt, localpathPt, l2);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(robotPt, q2);
    p3d_get_robot_config_into(robotPt, &q2);
  }
  softMotion_data_In = (localpathPt->specific.softMotion_data);
  if(softMotion_data == NULL) {
    softMotion_data = p3d_create_softMotion_data_multilocalpath(robotPt, localpathPt->mlpID);
  }

  if ((segmentl1 = MY_ALLOC(SM_SEGMENT, softMotion_data_In->nbDofs)) == NULL) {
    printf("  p3d_softMotion_extract_with_velocities: allocation failed\n");
    return NULL;
  }
  if ((condl1 = MY_ALLOC(SM_COND, softMotion_data_In->nbDofs)) == NULL) {
    printf("  p3d_softMotion_extract_with_velocities: allocation failed\n");
    return NULL;
  }
  if ((segmentl2 = MY_ALLOC(SM_SEGMENT, softMotion_data_In->nbDofs)) == NULL) {
    printf("  p3d_softMotion_extract_with_velocities: allocation failed\n");
    return NULL;
  }
  if ((condl2 = MY_ALLOC(SM_COND, softMotion_data_In->nbDofs)) == NULL) {
    printf("  p3d_softMotion_extract_with_velocities: allocation failed\n");
    return NULL;
  }

  for(int v=0; v<softMotion_data_In->nbDofs; v++) {
    softMotion_data->specific->J_max[v] = softMotion_data_In->specific->J_max[v];
    softMotion_data->specific->A_max[v] = softMotion_data_In->specific->A_max[v];
    softMotion_data->specific->V_max[v] = softMotion_data_In->specific->V_max[v];
  }

  for (i=0;i<softMotion_data_In->nbDofs;i++) {
    lm_get_softMotion_segment_params( softMotion_data_In, l1, &segmentl1[i], &segIdl1, i);
    if (l1 >= softMotion_data_In->specific->motion[i].MotionDuration) {
      paramLocal = softMotion_data_In->specific->motion[i].MotionDuration;
    } else {
      paramLocal = l1;
    }
    lm_get_paramDiff_for_param( softMotion_data_In, &segmentl1[i], segIdl1, i, paramLocal, &paramDiffl1);
    sm_CalculOfAccVelPosAtTimeSecond(paramDiffl1, &segmentl1[i], &condl1[i]);
  }

  for (i=0;i<softMotion_data_In->nbDofs;i++) {
    lm_get_softMotion_segment_params( softMotion_data_In, l2, &segmentl2[i], &segIdl2, i);
    if (l2 >= softMotion_data_In->specific->motion[i].MotionDuration) {
      paramLocal = softMotion_data_In->specific->motion[i].MotionDuration;
    } else {
      paramLocal = l2;
    }
    lm_get_paramDiff_for_param( softMotion_data_In, &segmentl2[i], segIdl2, i, paramLocal, &paramDiffl2);
    sm_CalculOfAccVelPosAtTimeSecond(paramDiffl2, &segmentl2[i], &condl2[i]);
  }

  /* Set sub Motion */
  if(softMotion_data_l1 == NULL) {
    softMotion_data_l1 = p3d_create_softMotion_data_multilocalpath(robotPt, localpathPt->mlpID);
  }
  softMotion_data_copy_into(robotPt, softMotion_data_In, softMotion_data_l1);

  q_init = p3d_copy_config(robotPt, softMotion_data_In->q_init);
  q_end = p3d_copy_config(robotPt, softMotion_data_In->q_end);

  for(int v=0; v<softMotion_data_In->nbDofs; v++) {
    q_init[index_dof + v] = 0.0;
    q_end[index_dof + v] = condl2[v].x-condl1[v].x;
    softMotion_data_l1->specific->velInit[v] = condl1[v].v;
    softMotion_data_l1->specific->velEnd[v] = condl2[v].v;
    softMotion_data_l1->specific->accInit[v] = condl1[v].a;
    softMotion_data_l1->specific->accEnd[v] = condl2[v].a;
  }

  lm_set_cond_softMotion_data(index_dof, softMotion_data_In->nbDofs, q_init, q_end, softMotion_data_l1->specific->velInit, softMotion_data_l1->specific->velEnd, softMotion_data_l1->specific->accInit, softMotion_data_l1->specific->accEnd, softMotion_data_l1);

  p3d_destroy_config(robotPt, q_init);
  p3d_destroy_config(robotPt, q_end);

  int paramDiffl1M = (int)(paramDiffl1*100);
  int paramDiffl2M = (int)(paramDiffl2*100);

  for(i=0;i<softMotion_data_In->nbDofs;i++) {
    if(segIdl1==0) {
      softMotion_data_l1->specific->motion[i].Times.Tjpa = softMotion_data_In->specific->motion[i].Times.Tjpa - paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = softMotion_data_In->specific->motion[i].TimesM.Tjpa - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1;
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M;
      for(j=1;j<SM_NB_SEG;j++) {
	softMotion_data_l1->specific->motion[i].TimeCumulM[j] = softMotion_data_In->specific->motion[i].TimeCumulM[j] - paramDiffl1M;
	softMotion_data_l1->specific->motion[i].TimeCumul[j] = softMotion_data_In->specific->motion[i].TimeCumul[j] - paramDiffl1;
      }
      softMotion_data_l1->specific->motion[i].Pos.Tjpa= softMotion_data_In->specific->motion[i].Pos.Tjpa - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Taca= softMotion_data_In->specific->motion[i].Pos.Taca - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= softMotion_data_In->specific->motion[i].Pos.Tjna - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = softMotion_data_In->specific->motion[i].Pos.Tvc - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= softMotion_data_In->specific->motion[i].Pos.Tjnb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= softMotion_data_In->specific->motion[i].Pos.Tacb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjpb - condl1[i].x;
    }

    if(segIdl1==1) {
      softMotion_data_l1->specific->motion[i].Times.Taca = softMotion_data_In->specific->motion[i].Times.Taca - paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Taca = softMotion_data_In->specific->motion[i].TimesM.Taca - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].Times.Tjpa = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = 0;

      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1
	- softMotion_data_In->specific->motion[i].TimeCumul[1];
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M
	- softMotion_data_In->specific->motion[i].TimeCumulM[1];

      softMotion_data_l1->specific->motion[i].TimeCumulM[1] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumul[1] = 0.0;
      for(j=2;j<SM_NB_SEG;j++) {
	softMotion_data_l1->specific->motion[i].TimeCumulM[j] = softMotion_data_In->specific->motion[i].TimeCumulM[j] - paramDiffl1M
	  -softMotion_data_In->specific->motion[i].TimeCumulM[1];
	softMotion_data_l1->specific->motion[i].TimeCumul[j] = softMotion_data_In->specific->motion[i].TimeCumul[j] - paramDiffl1
	  -softMotion_data_In->specific->motion[i].TimeCumul[1];
      }
      softMotion_data_l1->specific->motion[i].Pos.Tjpa= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Taca= softMotion_data_In->specific->motion[i].Pos.Taca - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= softMotion_data_In->specific->motion[i].Pos.Tjna - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = softMotion_data_In->specific->motion[i].Pos.Tvc - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= softMotion_data_In->specific->motion[i].Pos.Tjnb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= softMotion_data_In->specific->motion[i].Pos.Tacb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjpb - condl1[i].x;
    }

    if(segIdl1==2) {
      softMotion_data_l1->specific->motion[i].Times.Tjna = softMotion_data_In->specific->motion[i].Times.Tjna - paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Tjna = softMotion_data_In->specific->motion[i].TimesM.Tjna - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].Times.Tjpa = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = 0;
      softMotion_data_l1->specific->motion[i].Times.Taca = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Taca = 0;
      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1
	- softMotion_data_In->specific->motion[i].TimeCumul[2];
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M
	- softMotion_data_In->specific->motion[i].TimeCumulM[2];;
      softMotion_data_l1->specific->motion[i].TimeCumulM[1] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[2] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumul[1] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[2] = 0.0;
      for(j=3;j<SM_NB_SEG;j++) {
	softMotion_data_l1->specific->motion[i].TimeCumulM[j] = softMotion_data_In->specific->motion[i].TimeCumulM[j] - paramDiffl1M
	  -softMotion_data_In->specific->motion[i].TimeCumulM[2];
	softMotion_data_l1->specific->motion[i].TimeCumul[j] = softMotion_data_In->specific->motion[i].TimeCumul[j] - paramDiffl1
	  -softMotion_data_In->specific->motion[i].TimeCumul[2];
      }
      softMotion_data_l1->specific->motion[i].Pos.Tjpa= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Taca= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= softMotion_data_In->specific->motion[i].Pos.Tjna - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = softMotion_data_In->specific->motion[i].Pos.Tvc - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= softMotion_data_In->specific->motion[i].Pos.Tjnb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= softMotion_data_In->specific->motion[i].Pos.Tacb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjnb - condl1[i].x;
    }

    if(segIdl1==3) {
      softMotion_data_l1->specific->motion[i].Times.Tvc = softMotion_data_In->specific->motion[i].Times.Tvc - paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Tvc = softMotion_data_In->specific->motion[i].TimesM.Tvc - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].Times.Tjpa = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = 0;
      softMotion_data_l1->specific->motion[i].Times.Taca = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Taca = 0;
      softMotion_data_l1->specific->motion[i].Times.Tjna = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjna = 0;

      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1
	- softMotion_data_In->specific->motion[i].TimeCumul[3];
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M
	- softMotion_data_In->specific->motion[i].TimeCumulM[3];
      softMotion_data_l1->specific->motion[i].TimeCumulM[1] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[2] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[3] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumul[1] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[2] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[3] = 0.0;
      for(j=4;j<SM_NB_SEG;j++) {
	softMotion_data_l1->specific->motion[i].TimeCumul[j] = softMotion_data_In->specific->motion[i].TimeCumul[j] - paramDiffl1
	  -softMotion_data_In->specific->motion[i].TimeCumul[3];
	softMotion_data_l1->specific->motion[i].TimeCumulM[j] = softMotion_data_In->specific->motion[i].TimeCumulM[j] - paramDiffl1M
	  -softMotion_data_In->specific->motion[i].TimeCumulM[3];
      }
      softMotion_data_l1->specific->motion[i].Pos.Tjpa= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Taca= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = softMotion_data_In->specific->motion[i].Pos.Tvc - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= softMotion_data_In->specific->motion[i].Pos.Tjnb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= softMotion_data_In->specific->motion[i].Pos.Tacb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjpb - condl1[i].x;
    }

    if(segIdl1==4) {
      softMotion_data_l1->specific->motion[i].Times.Tjnb = softMotion_data_In->specific->motion[i].Times.Tjnb - paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Tjnb = softMotion_data_In->specific->motion[i].TimesM.Tjnb - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].Times.Tjpa = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = 0;
      softMotion_data_l1->specific->motion[i].Times.Taca = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Taca = 0;
      softMotion_data_l1->specific->motion[i].Times.Tjna = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjna = 0;
      softMotion_data_l1->specific->motion[i].Times.Tvc = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tvc = 0;

      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1
	- softMotion_data_In->specific->motion[i].TimeCumul[4];
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M
	- softMotion_data_In->specific->motion[i].TimeCumulM[4];
      softMotion_data_l1->specific->motion[i].TimeCumulM[1] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[2] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[3] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[4] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumul[1] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[2] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[3] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[4] = 0.0;

      for(j=5;j<SM_NB_SEG;j++) {
	softMotion_data_l1->specific->motion[i].TimeCumul[j] = softMotion_data_In->specific->motion[i].TimeCumul[j] - paramDiffl1
	  -softMotion_data_In->specific->motion[i].TimeCumul[4];
	softMotion_data_l1->specific->motion[i].TimeCumulM[j] = softMotion_data_In->specific->motion[i].TimeCumulM[j] - paramDiffl1M
	  -softMotion_data_In->specific->motion[i].TimeCumulM[4];
      }

      softMotion_data_l1->specific->motion[i].Pos.Tjpa= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Taca= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= softMotion_data_In->specific->motion[i].Pos.Tjnb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= softMotion_data_In->specific->motion[i].Pos.Tacb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjpb - condl1[i].x;
    }

    if(segIdl1==5) {
      softMotion_data_l1->specific->motion[i].Times.Tacb = softMotion_data_In->specific->motion[i].Times.Tacb - paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Tacb = softMotion_data_In->specific->motion[i].TimesM.Tacb - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].Times.Tjpa = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = 0;
      softMotion_data_l1->specific->motion[i].Times.Taca = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Taca = 0;
      softMotion_data_l1->specific->motion[i].Times.Tjna = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjna = 0;
      softMotion_data_l1->specific->motion[i].Times.Tvc = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tvc = 0;
      softMotion_data_l1->specific->motion[i].Times.Tjnb = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjnb = 0;

      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1
	- softMotion_data_In->specific->motion[i].TimeCumul[5];
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M
	- softMotion_data_In->specific->motion[i].TimeCumulM[5];
      softMotion_data_l1->specific->motion[i].TimeCumulM[1] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[2] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[3] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[4] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[5] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumul[1] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[2] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[3] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[4] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[5] = 0.0;
      for(j=6;j<SM_NB_SEG;j++) {
	softMotion_data_l1->specific->motion[i].TimeCumul[j] = softMotion_data_In->specific->motion[i].TimeCumul[j] - paramDiffl1
	  -softMotion_data_In->specific->motion[i].TimeCumul[5];
	softMotion_data_l1->specific->motion[i].TimeCumulM[j] = softMotion_data_In->specific->motion[i].TimeCumulM[j] - paramDiffl1M
	  -softMotion_data_In->specific->motion[i].TimeCumulM[5];
      }

      softMotion_data_l1->specific->motion[i].Pos.Tjpa= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Taca= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= softMotion_data_In->specific->motion[i].Pos.Tacb - condl1[i].x;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjpb - condl1[i].x;
    }

    if(segIdl1==6) {
      softMotion_data_l1->specific->motion[i].Times.Tjpb = softMotion_data_In->specific->motion[i].Times.Tjpb- paramDiffl1;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpb = softMotion_data_In->specific->motion[i].TimesM.Tjpb - paramDiffl1M;
      softMotion_data_l1->specific->motion[i].Times.Tjpa = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjpa = 0;
      softMotion_data_l1->specific->motion[i].Times.Taca = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Taca = 0;
      softMotion_data_l1->specific->motion[i].Times.Tjna = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjna = 0;
      softMotion_data_l1->specific->motion[i].Times.Tvc = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tvc = 0;
      softMotion_data_l1->specific->motion[i].Times.Tjnb = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tjnb = 0;
      softMotion_data_l1->specific->motion[i].Times.Tacb = 0.0;
      softMotion_data_l1->specific->motion[i].TimesM.Tacb = 0;

      softMotion_data_l1->specific->motion[i].MotionDuration = softMotion_data_In->specific->motion[i].MotionDuration - paramDiffl1
	- softMotion_data_In->specific->motion[i].TimeCumul[6];
      softMotion_data_l1->specific->motion[i].MotionDurationM = softMotion_data_In->specific->motion[i].MotionDurationM - paramDiffl1M
	- softMotion_data_In->specific->motion[i].TimeCumulM[6];
      softMotion_data_l1->specific->motion[i].TimeCumulM[1] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[2] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[3] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[4] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[5] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumulM[6] = 0;
      softMotion_data_l1->specific->motion[i].TimeCumul[1] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[2] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[3] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[4] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[5] = 0.0;
      softMotion_data_l1->specific->motion[i].TimeCumul[6] = 0.0;

      softMotion_data_l1->specific->motion[i].Pos.Tjpa= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Taca= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjna= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tvc = 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjnb= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tacb= 0.0;
      softMotion_data_l1->specific->motion[i].Pos.Tjpb= softMotion_data_In->specific->motion[i].Pos.Tjpb - condl1[i].x;
    }
  }

  /*
   *
   * Take into account l2
   *
   */

  softMotion_data_copy_into(robotPt, softMotion_data_l1, softMotion_data);

  for(i=0;i<SM_NB_DIM;i++) {

    if(segIdl2==6) {
      if(segIdl1!=6) {
	softMotion_data->specific->motion[i].Times.Tjpb  = paramDiffl2;
	softMotion_data->specific->motion[i].TimesM.Tjpb = paramDiffl2M;

	softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration  - (softMotion_data_l1->specific->motion[i].Times.Tjpb  - paramDiffl2);
	softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM -(softMotion_data_l1->specific->motion[i].TimesM.Tjpb - paramDiffl2M);

      } else {
	softMotion_data->specific->motion[i].Times.Tjpb  = paramDiffl2- paramDiffl1;
	softMotion_data->specific->motion[i].TimesM.Tjpb = paramDiffl2M - paramDiffl1M;

	softMotion_data->specific->motion[i].MotionDuration = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].MotionDurationM = paramDiffl2M - paramDiffl1M;
      }

      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
    }

    if(segIdl2==5) {

      if(segIdl1!=5) {
	softMotion_data->specific->motion[i].Times.Tacb  = paramDiffl2;
	softMotion_data->specific->motion[i].TimesM.Tacb = paramDiffl2M;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration - softMotion_data_l1->specific->motion[i].Times.Tjpb
	  - (softMotion_data_l1->specific->motion[i].Times.Tacb  - paramDiffl2);
	softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM - softMotion_data_l1->specific->motion[i].TimesM.Tjpb
	  -(softMotion_data_l1->specific->motion[i].TimesM.Tjpb - paramDiffl2M);

	softMotion_data->specific->motion[i].TimeCumulM[6] =  softMotion_data_l1->specific->motion[i].TimeCumulM[5] + paramDiffl2M;
	softMotion_data->specific->motion[i].TimeCumul[6] =  softMotion_data_l1->specific->motion[i].TimeCumul[5] + paramDiffl2;

      } else {
	softMotion_data->specific->motion[i].Times.Tacb  = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimesM.Tacb = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	softMotion_data->specific->motion[i].MotionDuration = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].MotionDurationM = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[6] =  paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumul[6] =  paramDiffl2 - paramDiffl1;
      }

      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tacb=  condl2[i].x-condl1[i].x;
    }

    if(segIdl2==4) {
      if(segIdl1!=4) {
	softMotion_data->specific->motion[i].Times.Tjnb  = paramDiffl2;
	softMotion_data->specific->motion[i].TimesM.Tjnb = paramDiffl2M;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration - softMotion_data_l1->specific->motion[i].Times.Tjpb
	  - softMotion_data_l1->specific->motion[i].Times.Tacb
	  - (softMotion_data_l1->specific->motion[i].Times.Tjnb  - paramDiffl2);
	softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM - softMotion_data_l1->specific->motion[i].TimesM.Tjpb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tacb
	  -(softMotion_data_l1->specific->motion[i].TimesM.Tjnb - paramDiffl2M);

	softMotion_data->specific->motion[i].TimeCumulM[5] = softMotion_data->specific->motion[i].TimeCumulM[4] + paramDiffl2M;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;
	softMotion_data->specific->motion[i].TimeCumul[5] = softMotion_data->specific->motion[i].TimeCumul[4] + paramDiffl2;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;
      } else {
	softMotion_data->specific->motion[i].Times.Tjnb  = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimesM.Tjnb = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	softMotion_data->specific->motion[i].MotionDuration = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].MotionDurationM = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[5] =  paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;
	softMotion_data->specific->motion[i].TimeCumul[5] =  paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;
      }
      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tacb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjnb=  condl2[i].x-condl1[i].x;
    }

    if(segIdl2==3) {

      if(segIdl1!=3) {
	softMotion_data->specific->motion[i].Times.Tvc  = paramDiffl2;
	softMotion_data->specific->motion[i].TimesM.Tvc = paramDiffl2M;
	softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	// Acc , Vel and Pos don't change
	softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration - softMotion_data_l1->specific->motion[i].Times.Tjpb
	  - softMotion_data_l1->specific->motion[i].Times.Tacb
	  - softMotion_data_l1->specific->motion[i].Times.Tjnb
	  - (softMotion_data_l1->specific->motion[i].Times.Tvc - paramDiffl2);
	softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM - softMotion_data_l1->specific->motion[i].TimesM.Tjpb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tacb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tjnb
	  -(softMotion_data_l1->specific->motion[i].TimesM.Tvc - paramDiffl2M);

	softMotion_data->specific->motion[i].TimeCumulM[4] = softMotion_data->specific->motion[i].TimeCumulM[3] + paramDiffl2M;
	softMotion_data->specific->motion[i].TimeCumulM[5] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;
	softMotion_data->specific->motion[i].TimeCumul[4] = softMotion_data->specific->motion[i].TimeCumul[3] + paramDiffl2;
	softMotion_data->specific->motion[i].TimeCumul[5] = 999;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;

      } else {
	softMotion_data->specific->motion[i].Times.Tvc  = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimesM.Tvc = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;
	// Acc , Vel and Pos don't change
	softMotion_data->specific->motion[i].MotionDuration = paramDiffl2- paramDiffl1;
	softMotion_data->specific->motion[i].MotionDurationM = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[4] =  paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[5] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;

	softMotion_data->specific->motion[i].TimeCumul[4] =  paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimeCumul[5] = 999;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;
      }
      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tacb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjnb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tvc=  condl2[i].x-condl1[i].x;
    }

    if(segIdl2==2) {
      if(segIdl1!=2) {
	softMotion_data->specific->motion[i].Times.Tjna  = paramDiffl2;
	softMotion_data->specific->motion[i].TimesM.Tjna = paramDiffl2M;
	softMotion_data->specific->motion[i].Times.Tvc  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tvc = 0;
	softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	// Acc , Vel and Pos don't change
	softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration - softMotion_data_l1->specific->motion[i].Times.Tjpb
	  - softMotion_data_l1->specific->motion[i].Times.Tacb
	  - softMotion_data_l1->specific->motion[i].Times.Tjnb
	  - softMotion_data_l1->specific->motion[i].Times.Tvc
	  - (softMotion_data_l1->specific->motion[i].Times.Tjna - paramDiffl2);
	softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM - softMotion_data_l1->specific->motion[i].TimesM.Tjpb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tacb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tjnb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tvc
	  -(softMotion_data_l1->specific->motion[i].TimesM.Tjna - paramDiffl2M);

	softMotion_data->specific->motion[i].TimeCumulM[3] = softMotion_data->specific->motion[i].TimeCumulM[2] + paramDiffl2M;
	softMotion_data->specific->motion[i].TimeCumulM[4] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[5] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;
	softMotion_data->specific->motion[i].TimeCumul[3] = softMotion_data->specific->motion[i].TimeCumul[2] + paramDiffl2;
	softMotion_data->specific->motion[i].TimeCumul[4] = 999;
	softMotion_data->specific->motion[i].TimeCumul[5] = 999;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;

      } else {
	softMotion_data->specific->motion[i].Times.Tjna  = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimesM.Tjna = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].Times.Tvc  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tvc = 0;
	softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	softMotion_data->specific->motion[i].MotionDuration = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].MotionDurationM = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[3] =  paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[4] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[5] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;
	softMotion_data->specific->motion[i].TimeCumul[3] =  paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimeCumul[4] = 999;
	softMotion_data->specific->motion[i].TimeCumul[5] = 999;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;
      }
      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tacb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjnb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tvc=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjna=  condl2[i].x-condl1[i].x;
    }

    if(segIdl2==1) {
      if(segIdl1!=1) {
	softMotion_data->specific->motion[i].Times.Taca  = paramDiffl2;
	softMotion_data->specific->motion[i].TimesM.Taca = paramDiffl2M;
	softMotion_data->specific->motion[i].Times.Tjna  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjna = 0;
	softMotion_data->specific->motion[i].Times.Tvc  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tvc = 0;
	softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	// Acc , Vel and Pos don't change
	softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration - softMotion_data_l1->specific->motion[i].Times.Tjpb
	  - softMotion_data_l1->specific->motion[i].Times.Tacb
	  - softMotion_data_l1->specific->motion[i].Times.Tjnb
	  - softMotion_data_l1->specific->motion[i].Times.Tvc
	  - softMotion_data_l1->specific->motion[i].Times.Tjna
	  - (softMotion_data_l1->specific->motion[i].Times.Taca - paramDiffl2);
	softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM - softMotion_data_l1->specific->motion[i].TimesM.Tjpb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tacb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tjnb
	  - softMotion_data_l1->specific->motion[i].TimesM.Tvc
	  - softMotion_data_l1->specific->motion[i].TimesM.Tjna
	  -(softMotion_data_l1->specific->motion[i].TimesM.Taca - paramDiffl2M);

	softMotion_data->specific->motion[i].TimeCumulM[2] = softMotion_data->specific->motion[i].TimeCumulM[1] + paramDiffl2M;
	softMotion_data->specific->motion[i].TimeCumulM[3] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[4] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[5] = 999;
	softMotion_data->specific->motion[i].TimeCumulM[6] = 999;

	softMotion_data->specific->motion[i].TimeCumul[2] = softMotion_data->specific->motion[i].TimeCumul[1] + paramDiffl2M;
	softMotion_data->specific->motion[i].TimeCumul[3] = 999;
	softMotion_data->specific->motion[i].TimeCumul[4] = 999;
	softMotion_data->specific->motion[i].TimeCumul[5] = 999;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;
      } else {
	softMotion_data->specific->motion[i].Times.Taca  = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimesM.Taca = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].Times.Tjna  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjna = 0;
	softMotion_data->specific->motion[i].Times.Tvc  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tvc = 0;
	softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
	softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tacb = 0;
	softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
	softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

	softMotion_data->specific->motion[i].MotionDuration = paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].MotionDurationM = paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[2] =  paramDiffl2M - paramDiffl1M;
	softMotion_data->specific->motion[i].TimeCumulM[3] = 2*softMotion_data->specific->motion[i].TimeCumulM[2];
	softMotion_data->specific->motion[i].TimeCumulM[4] = 2*softMotion_data->specific->motion[i].TimeCumulM[3];
	softMotion_data->specific->motion[i].TimeCumulM[5] = 2*softMotion_data->specific->motion[i].TimeCumulM[4];
	softMotion_data->specific->motion[i].TimeCumulM[6] = 2*softMotion_data->specific->motion[i].TimeCumulM[5];

	softMotion_data->specific->motion[i].TimeCumul[2] =  paramDiffl2 - paramDiffl1;
	softMotion_data->specific->motion[i].TimeCumul[3] = 999;
	softMotion_data->specific->motion[i].TimeCumul[4] = 999;
	softMotion_data->specific->motion[i].TimeCumul[5] = 999;
	softMotion_data->specific->motion[i].TimeCumul[6] = 999;
      }
      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tacb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjnb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tvc=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjna=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Taca=  condl2[i].x-condl1[i].x;
    }

    if(segIdl2==0) {
      softMotion_data->specific->motion[i].Times.Tjpa  = paramDiffl2 - paramDiffl1;
      softMotion_data->specific->motion[i].TimesM.Tjpa = paramDiffl2M - paramDiffl1M;
      softMotion_data->specific->motion[i].Times.Taca  = 0.0;
      softMotion_data->specific->motion[i].TimesM.Taca = 0;
      softMotion_data->specific->motion[i].Times.Tjna  = 0.0;
      softMotion_data->specific->motion[i].TimesM.Tjna = 0;
      softMotion_data->specific->motion[i].Times.Tvc  = 0.0;
      softMotion_data->specific->motion[i].TimesM.Tvc = 0;
      softMotion_data->specific->motion[i].Times.Tjnb  = 0.0;
      softMotion_data->specific->motion[i].TimesM.Tjnb = 0;
      softMotion_data->specific->motion[i].Times.Tacb  = 0.0;
      softMotion_data->specific->motion[i].TimesM.Tacb = 0;
      softMotion_data->specific->motion[i].Times.Tjpb  = 0.0;
      softMotion_data->specific->motion[i].TimesM.Tjpb = 0;

      // Acc , Vel and Pos don't change
      softMotion_data->specific->motion[i].MotionDuration = softMotion_data_l1->specific->motion[i].MotionDuration - softMotion_data_l1->specific->motion[i].Times.Tjpb
	- softMotion_data_l1->specific->motion[i].Times.Tacb
	- softMotion_data_l1->specific->motion[i].Times.Tjnb
	- softMotion_data_l1->specific->motion[i].Times.Tvc
	- softMotion_data_l1->specific->motion[i].Times.Tjna
	- softMotion_data_l1->specific->motion[i].Times.Taca
	- (softMotion_data_l1->specific->motion[i].Times.Tjpa - paramDiffl2 - paramDiffl1);
      softMotion_data->specific->motion[i].MotionDurationM = softMotion_data_l1->specific->motion[i].MotionDurationM - softMotion_data_l1->specific->motion[i].TimesM.Tjpb
	- softMotion_data_l1->specific->motion[i].TimesM.Tacb
	- softMotion_data_l1->specific->motion[i].TimesM.Tjnb
	- softMotion_data_l1->specific->motion[i].TimesM.Tvc
	- softMotion_data_l1->specific->motion[i].TimesM.Tjna
	- softMotion_data_l1->specific->motion[i].TimesM.Taca
	-(softMotion_data_l1->specific->motion[i].TimesM.Tjpa - paramDiffl2M - paramDiffl1M);


      softMotion_data->specific->motion[i].TimeCumulM[1] =  paramDiffl2M - paramDiffl1M;
      softMotion_data->specific->motion[i].TimeCumulM[2] = 999;
      softMotion_data->specific->motion[i].TimeCumulM[3] = 999;
      softMotion_data->specific->motion[i].TimeCumulM[4] = 999;
      softMotion_data->specific->motion[i].TimeCumulM[5] = 999;
      softMotion_data->specific->motion[i].TimeCumulM[6] = 999;

      softMotion_data->specific->motion[i].TimeCumul[1] =  paramDiffl2 - paramDiffl1;
      softMotion_data->specific->motion[i].TimeCumul[2] = 999;
      softMotion_data->specific->motion[i].TimeCumul[3] = 999;
      softMotion_data->specific->motion[i].TimeCumul[4] = 999;
      softMotion_data->specific->motion[i].TimeCumul[5] = 999;
      softMotion_data->specific->motion[i].TimeCumul[6] = 999;

      softMotion_data->specific->motion[i].Pos.Tjpb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tacb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjnb=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tvc=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjna=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Taca=  condl2[i].x-condl1[i].x;
      softMotion_data->specific->motion[i].Pos.Tjpa=  condl2[i].x-condl1[i].x;
    }
  }

  /* We assume that this a ptp motion thus all axis share the same motion time */
  sm_sum_motionTimes(&softMotion_data->specific->motion[0].Times, &sum);
  softMotion_data->specific->motionTime = sum;

  softMotion_data->q_init  = p3d_copy_config(robotPt, q1);
  softMotion_data->q_end   = p3d_copy_config(robotPt, q2);

  sub_localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_data, 0, TRUE);

  p3d_destroy_softMotion_data(robotPt, softMotion_data_l1);


  if(sub_localpathPt != NULL) {
    sub_localpathPt->mlpID = localpathPt->mlpID;
    sub_localpathPt->length_lp = softMotion_data->specific->motionTime;
    sub_localpathPt->range_param = softMotion_data->specific->motionTime;
    sub_localpathPt->q_init = p3d_copy_config(robotPt, q1);
    p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
    sub_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
    sub_localpathPt->activeCntrts = MY_ALLOC(int, sub_localpathPt->nbActiveCntrts);
    for(int i = 0; i < sub_localpathPt->nbActiveCntrts; i++){
      sub_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
    }
  }
  MY_FREE(segmentl1, SM_SEGMENT, softMotion_data->nbDofs);
  MY_FREE(condl1, SM_COND, softMotion_data->nbDofs);
  MY_FREE(segmentl2, SM_SEGMENT, softMotion_data->nbDofs);
  MY_FREE(condl2, SM_COND, softMotion_data->nbDofs);
  p3d_destroy_config(robotPt, q1);
  p3d_destroy_config(robotPt, q2);
  return sub_localpathPt;
}

/*
 *  does nothing
 */
p3d_localpath *p3d_simplify_softMotion(p3d_rob *robotPt, p3d_localpath *localpathPt, int *need_colcheck) {
  return localpathPt;
}

int p3d_write_softMotion_localpath(FILE *file, p3d_rob* robotPt, p3d_localpath* localpathPt) {
  printf("ERROR TODO p3d_write_softMotion_localpath\n");

  return TRUE;
}



///////////////////////////////////////////////
//             DESTROY FUNCTIONS				   	 //
///////////////////////////////////////////////
/* Destroy a softMotion local path */
void p3d_softMotion_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt) {
  if (localpathPt != NULL){

    /* test whether the type of local path is the expected one */
    if (localpathPt->type_lp != SOFT_MOTION){
      PrintError(("p3d_softMotion_destroy: softMotion local path expected\n"));
    }
    /* destroy the specific part */
    if (localpathPt->specific.softMotion_data != NULL){
      p3d_destroy_softMotion_data(robotPt, localpathPt->specific.softMotion_data);
    }

    localpathPt->next_lp = NULL;
    localpathPt->prev_lp = NULL;
    MY_FREE(localpathPt->activeCntrts, int, localpathPt->nbActiveCntrts);
    MY_FREE(localpathPt, p3d_localpath, 1);
  }
}

/* destroys a structure of type p3d_softMotion_data */
void p3d_destroy_softMotion_data(p3d_rob* robotPt, p3d_softMotion_data* softMotion_dataPt) {
  if (softMotion_dataPt != NULL){
    if (softMotion_dataPt->q_init != NULL){
      p3d_destroy_config(robotPt, softMotion_dataPt->q_init);
      p3d_destroy_config(robotPt, softMotion_dataPt->q_end);
      p3d_destroy_config(robotPt, softMotion_dataPt->q_endp1);
    }

    if(softMotion_dataPt->specific != NULL){
      if(softMotion_dataPt->specific->J_max != NULL) {
	MY_FREE(softMotion_dataPt->specific->J_max, double,  softMotion_dataPt->nbDofs);
      }
      if(softMotion_dataPt->specific->A_max != NULL) {
	MY_FREE(softMotion_dataPt->specific->A_max, double,  softMotion_dataPt->nbDofs);
      }
      if(softMotion_dataPt->specific->V_max != NULL) {
	MY_FREE(softMotion_dataPt->specific->V_max, double, softMotion_dataPt->nbDofs);
      }
      if(softMotion_dataPt->specific->velInit != NULL) {
	MY_FREE(softMotion_dataPt->specific->velInit, double,  softMotion_dataPt->nbDofs);
      }
      if(softMotion_dataPt->specific->velEnd != NULL) {
	MY_FREE(softMotion_dataPt->specific->velEnd, double, softMotion_dataPt->nbDofs);
      }
      if(softMotion_dataPt->specific->accInit != NULL) {
	MY_FREE(softMotion_dataPt->specific->accInit, double,  softMotion_dataPt->nbDofs);
      }
      if(softMotion_dataPt->specific->accEnd != NULL) {
	MY_FREE(softMotion_dataPt->specific->accEnd, double, softMotion_dataPt->nbDofs);
      }

      if(softMotion_dataPt->specific->motion != NULL) {
	MY_FREE(softMotion_dataPt->specific->motion, SM_MOTION_MONO, softMotion_dataPt->nbDofs);
      }
      MY_FREE(softMotion_dataPt->specific, p3d_softMotion_data_specific, 1);
    }

    MY_FREE(softMotion_dataPt, p3d_softMotion_data, 1);
    softMotion_dataPt = NULL;
  }
}

void lm_destroy_softMotion_params(p3d_rob * robotPt, void *local_method_params) {
  if (local_method_params != NULL){
    softMotion_str * paramPt = (softMotion_str *)local_method_params;

    if (paramPt->specific != NULL){
      if(paramPt->specific->J_max != NULL) {
	MY_FREE(paramPt->specific->J_max, double,  paramPt->nbDofs);
      }
      if(paramPt->specific->A_max != NULL) {
	MY_FREE(paramPt->specific->A_max, double,  paramPt->nbDofs);
      }
      if(paramPt->specific->V_max != NULL) {
	MY_FREE(paramPt->specific->V_max, double, paramPt->nbDofs);
      }
      MY_FREE(paramPt->specific, gp_specific_str, 1);
    }
    MY_FREE(paramPt, softMotion_str, 1);
  }
}

///////////////////////////////////////////////
//             ALLOC FUNCTIONS				    	 //
///////////////////////////////////////////////
p3d_softMotion_data * p3d_create_softMotion_data_multilocalpath(p3d_rob* robotPt, int mlpId) {
  p3d_softMotion_data * softMotion_data = NULL;
  psoftMotion_str softMotion_params = NULL;
  int nbDofs = 0;
  int nbJoints = 0;
  p3d_group_type gpType = robotPt->mlp->mlpJoints[mlpId]->gpType;

  if ((softMotion_data = MY_ALLOC(p3d_softMotion_data,1)) == NULL) {
    return NULL;
  }
  softMotion_data->isPlanned = FALSE;
  softMotion_data->isPTP     = TRUE;
  softMotion_data->gpType    = gpType;
  softMotion_data->specific = NULL;
  softMotion_data->q_init  = NULL;
  softMotion_data->q_end   = NULL;
  softMotion_data->q_endp1 = NULL;

  nbJoints = robotPt->mlp->mlpJoints[mlpId]->nbJoints;
  for(int v=0; v<nbJoints; v++) {
    nbDofs += robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[v]]->user_dof_equiv_nbr;
  }

  if ((softMotion_data->specific = MY_ALLOC(p3d_softMotion_data_specific, 1)) == NULL) {
    return NULL;
  }
  if ((softMotion_data->specific->J_max = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->A_max = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->V_max = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->velInit = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed A_max\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->velEnd = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed V_max\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->accInit = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed A_max\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->accEnd = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed V_max\n"));
    return (NULL);
  }
  if ((softMotion_data->specific->motion = MY_ALLOC(SM_MOTION_MONO, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return (NULL);
  }
  softMotion_data->nbJoints  = nbJoints;
  softMotion_data->nbDofs  = nbDofs;
  softMotion_data->specific->motionTime = 0.0;
  softMotion_params = lm_get_softMotion_lm_param_multilocalpath(robotPt, mlpId);
  for(int i = 0; i< nbDofs ; i++) {
    softMotion_data->specific->J_max[i] = softMotion_params->specific->J_max[i];
    softMotion_data->specific->A_max[i] = softMotion_params->specific->A_max[i];
    softMotion_data->specific->V_max[i] = softMotion_params->specific->V_max[i];
    softMotion_data->specific->velInit[i] = 0.0;
    softMotion_data->specific->velEnd[i] = 0.0;
    softMotion_data->specific->accInit[i] = 0.0;
    softMotion_data->specific->accEnd[i] = 0.0;
  }
  return softMotion_data;
}

/* allocation of local path of type softMotion */
p3d_localpath * p3d_alloc_softMotion_localpath(p3d_rob *robotPt, p3d_softMotion_data * sm_data,	int lp_id, int is_valid) {
  p3d_localpath * localpathPt = NULL;
  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

  localpathPt->specific.softMotion_data = sm_data;

  /* Initialization of the generic part */
  /* fields */
  localpathPt->type_lp = SOFT_MOTION;
  localpathPt->valid = is_valid;
  localpathPt->lp_id = lp_id;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;

#ifdef MULTILOCALPATH
  localpathPt->mlpID = -1;
  for(int j=0; j< MAX_MULTILOCALPATH_NB ; j++) {
    localpathPt->mlpLocalpath[j] = NULL;
  }
#endif

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length = NULL;
  //		(double (*)(p3d_rob*, p3d_localpath*))(p3d_lin_dist);
  /* extract from a local path a sub local path starting from length
     l1 and ending at length l2 */
  localpathPt->extract_sub_localpath =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, double, double))(p3d_extract_softMotion);
  /* extract from a local path a sub local path starting from parameter
     u1 and ending at parameter u2 */
  localpathPt->extract_by_param =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*, double, double))(p3d_extract_softMotion);
  /* destroy the localpath */
  localpathPt->destroy =
    (void (*)(p3d_rob*, p3d_localpath*))(p3d_softMotion_destroy);
  /*copy the local path */
  localpathPt->copy =
    (p3d_localpath* (*)(p3d_rob*, p3d_localpath*))(p3d_copy_softMotion_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =
    (configPt (*)(p3d_rob*, p3d_localpath*, double))(p3d_softMotion_config_at_param);
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =
    (configPt (*)(p3d_rob*, p3d_localpath*, double))(p3d_softMotion_config_at_param);
  /* This function return the step in tick for range_param (1 tick here = 10 ms) */
  localpathPt->stay_within_dist =
    (double (*)(p3d_rob*, p3d_localpath*, double, whichway, double*))(p3d_softMotion_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost =
    (double (*)(p3d_rob*, p3d_localpath*))(p3d_softMotion_cost);
  /* function that simplifies the sequence of two local paths: valid
  // 	only for RS curves */
     localpathPt->simplify =		(p3d_localpath* (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_softMotion);
  // 			(p3d_localpath* (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_softMotion);
  /* write the local path in a file */
  localpathPt->write =	(int (*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_softMotion_localpath);

  /* the length of the localpath is the time duration */

  switch (sm_data->gpType) {
  case FREEFLYER:
  case JOINT:
    localpathPt->length_lp = sm_data->specific->motionTime;
    localpathPt->range_param = sm_data->specific->motionTime;
    break;
  default:
    localpathPt->length_lp = 0.0;
    localpathPt->range_param = 0.0;
    break;
  }
  localpathPt->ikSol = NULL;
  localpathPt->nbActiveCntrts = 0;
  localpathPt->activeCntrts = NULL;
  localpathPt->q_init = NULL;
  return localpathPt;
}

/*
 *  lm_create_softMotion
 */
psoftMotion_str lm_create_softMotion(p3d_rob *robotPt, int mlpId) {
  psoftMotion_str softMotion_params = NULL;
  p3d_jnt* jnt = NULL;
  int k=0;
  int nbJoints = robotPt->mlp->mlpJoints[mlpId]->nbJoints;
  int nbDofs = robotPt->mlp->mlpJoints[mlpId]->nbDofs;

  if ((softMotion_params = MY_ALLOC(softMotion_str, 1)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return (NULL);
  }
  if ((softMotion_params->specific = MY_ALLOC(gp_specific_str, 1)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed\n"));
    return (NULL);
  }

  if ((softMotion_params->specific->J_max = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed J_max\n"));
    return (NULL);
  }
  if ((softMotion_params->specific->A_max = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed A_max\n"));
    return (NULL);
  }
  if ((softMotion_params->specific->V_max = MY_ALLOC(double, nbDofs)) == NULL) {
    PrintWarning(("  lm_create_softMotion: allocation failed V_max\n"));
    return (NULL);
  }
  k =0;
  for(int i=0; i<nbJoints; i++) {
    jnt= robotPt->joints[robotPt->mlp->mlpJoints[mlpId]->joints[i]];
    for(int j=0; j< jnt->user_dof_equiv_nbr; j++) {
      if(jnt->dof_data[j].velocity_max != 0.0) {
	softMotion_params->specific->V_max[k] = jnt->dof_data[j].velocity_max;
      } else {
	softMotion_params->specific->V_max[k] = 0.5;
	printf("!!! ERROR (joint: %d dof: %d) velocity max of the dof not given, set to default (0.5)!!!\n", robotPt->mlp->mlpJoints[mlpId]->joints[i],j);
      }
      if(jnt->dof_data[j].acceleration_max != 0.0) {
	softMotion_params->specific->A_max[k] = jnt->dof_data[j].acceleration_max;
      } else {
	softMotion_params->specific->A_max[k] = 1.0;
	printf("!!! ERROR (joint: %d dof: %d) acceleration max of the dof not given, set to default (1.0)!!!\n", robotPt->mlp->mlpJoints[mlpId]->joints[i],j);
      }
      if(jnt->dof_data[j].jerk_max != 0.0) {
	softMotion_params->specific->J_max[k] = jnt->dof_data[j].jerk_max;
      } else {
	softMotion_params->specific->J_max[k] = 3.0;
	printf("!!! ERROR (joint: %d dof: %d) jerk max of the dof not given, set to default (3.0)!!!\n", robotPt->mlp->mlpJoints[mlpId]->joints[i],j);
      }
      k++;
    }
  }

  softMotion_params->nbJoints = nbJoints;
  softMotion_params->nbDofs = nbDofs;
  return(softMotion_params);
}

/*
 *  Copy one local path.
 *  Input:  the robot, the local path.
 *  Output: the copied local path
 */
p3d_localpath *p3d_copy_softMotion_localpath(p3d_rob* robotPt, p3d_localpath* localpathPt) {

  p3d_localpath *softMotion_localpathPt;
  p3d_softMotion_data*  softMotion_dataPt = NULL;
  int lp_id = localpathPt->lp_id;
  int is_valid = localpathPt->valid;

  softMotion_dataPt = p3d_copy_softMotion_data(robotPt, localpathPt->mlpID, localpathPt->specific.softMotion_data);

  softMotion_localpathPt = p3d_alloc_softMotion_localpath(robotPt, softMotion_dataPt, lp_id, is_valid);

  /* update length and range of parameter */
  softMotion_localpathPt->length_lp   = localpathPt->length_lp;
  softMotion_localpathPt->range_param = localpathPt->range_param;
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(softMotion_localpathPt->ikSol));
  softMotion_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  softMotion_localpathPt->activeCntrts = MY_ALLOC(int, softMotion_localpathPt->nbActiveCntrts);
  for(int i = 0; i < softMotion_localpathPt->nbActiveCntrts; i++){
    softMotion_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
  }
  return softMotion_localpathPt;
}

p3d_softMotion_data* p3d_copy_softMotion_data(p3d_rob* robotPt, int mlpID, p3d_softMotion_data *sm_data) {
  p3d_group_type gpType;
  int nbJoints = 0;
  p3d_softMotion_data*  softMotion_dataPt = NULL;

  gpType = robotPt->mlp->mlpJoints[mlpID]->gpType;
  nbJoints = robotPt->mlp->mlpJoints[mlpID]->nbJoints;

  softMotion_dataPt = p3d_create_softMotion_data_multilocalpath(robotPt, mlpID);

  softMotion_dataPt->isPlanned = sm_data->isPlanned;
  softMotion_dataPt->isPTP     = sm_data->isPTP;
  softMotion_dataPt->nbJoints  = sm_data->nbJoints;
  softMotion_dataPt->nbDofs  = sm_data->nbDofs;

  softMotion_dataPt->q_init  = p3d_copy_config(robotPt, sm_data->q_init);
  softMotion_dataPt->q_end   = p3d_copy_config(robotPt, sm_data->q_end);
  softMotion_dataPt->q_endp1 = p3d_copy_config(robotPt, sm_data->q_endp1);

  if(sm_data->specific != NULL) {
    for(int i=0; i<sm_data->nbDofs; i++) {
      softMotion_dataPt->specific->J_max[i] = sm_data->specific->J_max[i];
      softMotion_dataPt->specific->A_max[i] = sm_data->specific->A_max[i];
      softMotion_dataPt->specific->V_max[i] = sm_data->specific->V_max[i];
      softMotion_dataPt->specific->velInit[i] = sm_data->specific->velInit[i];
      softMotion_dataPt->specific->velEnd[i]  = sm_data->specific->velEnd[i];
      softMotion_dataPt->specific->accInit[i] = sm_data->specific->accInit[i];
      softMotion_dataPt->specific->accEnd[i]  = sm_data->specific->accEnd[i];
      sm_copy_SM_MOTION_MONO_into(&(sm_data->specific->motion[i]), &(softMotion_dataPt->specific->motion[i]));
    }
    softMotion_dataPt->specific->motionTime = sm_data->specific->motionTime;
  }
  return softMotion_dataPt;

}

///////////////////////////////////////////////
//             OTHER FUNCTIONS				    	 //
///////////////////////////////////////////////
void lm_set_cond_softMotion_data(int index_dof, int nbDofs, configPt qi, configPt qf, double *velInit, double *velEnd, double *accInit, double *accEnd, p3d_softMotion_data* softMotion_data) {

  for(int i = 0; i< nbDofs; i++) {
    softMotion_data->specific->motion[i].IC.a = accInit[i];
    softMotion_data->specific->motion[i].IC.v = velInit[i];
    softMotion_data->specific->motion[i].IC.x = qi[index_dof +i];

    softMotion_data->specific->motion[i].FC.a = accEnd[i];
    softMotion_data->specific->motion[i].FC.v = velEnd[i];
    softMotion_data->specific->motion[i].FC.x = qf[index_dof +i];
  }

  return;
}

/*
 *  lm_get_softMotion_lm_param_multilocapath --
 *
 *  find the first occurence of softMotion local method parameters.
 */
psoftMotion_str lm_get_softMotion_lm_param_multilocalpath(p3d_rob *robotPt, int nblpGp) {
  lm_list_param_str *list_paramPt = (robotPt->mlp->mlpJoints[nblpGp])->local_method_params;
  psoftMotion_str resultPt=NULL;

  while (list_paramPt) {
    if (list_paramPt->lpl_type != P3D_SOFT_MOTION_PLANNER) {
      list_paramPt = list_paramPt->next;
    }
    else {
      resultPt = (psoftMotion_str)(list_paramPt->lm_param);
      list_paramPt = NULL;
    }
  }
  return resultPt;
}

void lm_get_softMotion_segment_params(p3d_softMotion_data* softMotion_data, double param, SM_SEGMENT * segment, int * segId, int index) {
  if (param>=softMotion_data->specific->motion[index].MotionDuration) {
    param = softMotion_data->specific->motion[index].MotionDuration;
  }

  if(softMotion_data->specific->motion[index].motionIsAdjusted == 0) {
    if(param >= softMotion_data->specific->motion[index].TimeCumul[6]) {
      segment->type = 7;
      *segId = 6;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjpb;
      segment->time = softMotion_data->specific->motion[index].Times.Tjpb;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tacb;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tacb;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tacb;
      segment->dir  = softMotion_data->specific->motion[index].Dir;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[5]) {
      segment->type = 6;
      *segId = 5;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tacb;
      segment->time = softMotion_data->specific->motion[index].Times.Tacb;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tjnb;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tjnb;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tjnb;
      segment->dir  = softMotion_data->specific->motion[index].Dir;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[4]) {
      segment->type = 5;
      *segId = 4;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjnb;
      segment->time = softMotion_data->specific->motion[index].Times.Tjnb;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tvc;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tvc;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tvc;
      segment->dir  = softMotion_data->specific->motion[index].Dir;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[3]) {
      segment->type = 4;
      *segId = 3;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tvc;
      segment->time = softMotion_data->specific->motion[index].Times.Tvc;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tjna;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tjna;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tjna;
      segment->dir  = softMotion_data->specific->motion[index].Dir;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[2]) {
      segment->type = 3;
      *segId = 2;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjna;
      segment->time = softMotion_data->specific->motion[index].Times.Tjna;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Taca;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Taca;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Taca;
      segment->dir  = softMotion_data->specific->motion[index].Dir;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[1]) {
      segment->type = 2;
      *segId = 1;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Taca;
      segment->time = softMotion_data->specific->motion[index].Times.Taca;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tjpa;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tjpa;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tjpa;
      segment->dir  = softMotion_data->specific->motion[index].Dir;

    } else {
      segment->type = 1;
      *segId = 0;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjpa;
      segment->time = softMotion_data->specific->motion[index].Times.Tjpa;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].IC.a;
      segment->V0   = softMotion_data->specific->motion[index].IC.v;
      segment->X0   = softMotion_data->specific->motion[index].IC.x;
      segment->dir  = softMotion_data->specific->motion[index].Dir;
    }

  } else {  // motionIsAdjusted == 1

    if(param >= softMotion_data->specific->motion[index].TimeCumul[6]) {
      segment->type = 3;
      *segId = 6;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjpb;
      segment->time = softMotion_data->specific->motion[index].Times.Tjpb;
      segment->J    = softMotion_data->specific->motion[index].jerk.J4;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tacb;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tacb;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tacb;
      segment->dir  = softMotion_data->specific->motion[index].Dir_b;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[5]) {
      segment->type = 2;
      *segId = 5;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tacb;
      segment->time = softMotion_data->specific->motion[index].Times.Tacb;
      segment->J    = 0.0;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tjnb;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tjnb;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tjnb;
      segment->dir  = softMotion_data->specific->motion[index].Dir_b;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[4]) {
      segment->type = 1;
      *segId = 4;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjnb;
      segment->time = softMotion_data->specific->motion[index].Times.Tjnb;
      segment->J    = softMotion_data->specific->motion[index].jerk.J3;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tvc;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tvc;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tvc;
      segment->dir  = softMotion_data->specific->motion[index].Dir_b;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[3]) {
      segment->type = 4;
      *segId = 3;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tvc;
      segment->time = softMotion_data->specific->motion[index].Times.Tvc;
      segment->J    = 0.0;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tjna;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tjna;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tjna;
      segment->dir  = softMotion_data->specific->motion[index].Dir_a;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[2]) {
      segment->type = 3;
      *segId = 2;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjna;
      segment->time = softMotion_data->specific->motion[index].Times.Tjna;
      segment->J    = softMotion_data->specific->motion[index].jerk.J2;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Taca;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Taca;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Taca;
      segment->dir  = softMotion_data->specific->motion[index].Dir_a;

    } else if (param >= softMotion_data->specific->motion[index].TimeCumul[1]) {
      segment->type = 2;
      *segId = 1;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Taca;
      segment->time = softMotion_data->specific->motion[index].Times.Taca;
      segment->J    = 0.0;
      segment->A0   = softMotion_data->specific->motion[index].Acc.Tjpa;
      segment->V0   = softMotion_data->specific->motion[index].Vel.Tjpa;
      segment->X0   = softMotion_data->specific->motion[index].Pos.Tjpa;
      segment->dir  = softMotion_data->specific->motion[index].Dir_a;

    } else {
      segment->type = 1;
      *segId = 0;
      segment->timeM = (int)softMotion_data->specific->motion[index].TimesM.Tjpa;
      segment->time = softMotion_data->specific->motion[index].Times.Tjpa;
      segment->J    = softMotion_data->specific->motion[index].jerk.J1;
      segment->A0   = softMotion_data->specific->motion[index].IC.a;
      segment->V0   = softMotion_data->specific->motion[index].IC.v;
      segment->X0   = softMotion_data->specific->motion[index].IC.x;
      segment->dir  = softMotion_data->specific->motion[index].Dir_a;
    }
  }
  return;
}

void lm_get_paramDiff_for_param(p3d_softMotion_data* softMotion_data, SM_SEGMENT* seg, int segId, int index, double param, double* paramDiff) {
  if (segId == 0) {
    *paramDiff = param;
    return;
  }

  *paramDiff = param - softMotion_data->specific->motion[index].TimeCumul[segId];

  return;
}

void softMotion_data_copy_into(p3d_rob *robotPt, const p3d_softMotion_data * sm_data, p3d_softMotion_data * softMotion_data) {

  if(softMotion_data->specific != NULL) {
    softMotion_data->q_init = p3d_copy_config(robotPt, sm_data->q_init);
    softMotion_data->q_end  = p3d_copy_config(robotPt, sm_data->q_end);
    softMotion_data->q_endp1 = p3d_copy_config(robotPt, sm_data->q_endp1);

    for(int i=0; i<softMotion_data->nbDofs; i++) {
      softMotion_data->specific->J_max[i] = sm_data->specific->J_max[i];
      softMotion_data->specific->A_max[i] = sm_data->specific->A_max[i];
      softMotion_data->specific->V_max[i] = sm_data->specific->V_max[i];
      softMotion_data->specific->velInit[i] = sm_data->specific->velInit[i];
      softMotion_data->specific->velEnd[i]  = sm_data->specific->velEnd[i];
      softMotion_data->specific->accInit[i] = sm_data->specific->accInit[i];
      softMotion_data->specific->accEnd[i]  = sm_data->specific->accEnd[i];
      sm_copy_SM_MOTION_MONO_into(&(sm_data->specific->motion[i]), &(softMotion_data->specific->motion[i]));
    }
    softMotion_data->specific->motionTime = sm_data->specific->motionTime;
  }
  return;
}

void lm_set_and_get_motionTimes(p3d_softMotion_data* softMotion_data, double* timeMotionMax, int* axisMotionMax) {
  int i=0;
  int NOE;
  *timeMotionMax = 0.0;
  * axisMotionMax = 0;
  SM_TIMES smTimesTmp;

  for(i=0; i<softMotion_data->nbDofs; i++) {
    sm_GetMonotonicTimes(softMotion_data->specific->motion[i].Times, &smTimesTmp, &NOE);
    sm_GetNumberOfElement(&smTimesTmp, &softMotion_data->specific->motion[i].TimesM);

    sm_sum_motionTimes(&(softMotion_data->specific->motion[i].Times), &(softMotion_data->specific->motion[i].MotionDuration));
    sm_sum_motionTimes(&(softMotion_data->specific->motion[i].TimesM), &(softMotion_data->specific->motion[i].MotionDurationM));

    if(SOFT_MOTION_PRINT_DATA) {
      printf("motionDuration[%d] = %f\n",i,softMotion_data->specific->motion[i].MotionDurationM);
    }

    softMotion_data->specific->motion[i].TimeCumulM[0] = 0;
    softMotion_data->specific->motion[i].TimeCumulM[1] = (int)softMotion_data->specific->motion[i].TimesM.Tjpa;
    softMotion_data->specific->motion[i].TimeCumulM[2] = (int)softMotion_data->specific->motion[i].TimeCumulM[1] \
      + (int)softMotion_data->specific->motion[i].TimesM.Taca;
    softMotion_data->specific->motion[i].TimeCumulM[3] = (int)softMotion_data->specific->motion[i].TimeCumulM[2] \
      + (int)softMotion_data->specific->motion[i].TimesM.Tjna;
    softMotion_data->specific->motion[i].TimeCumulM[4] = (int)softMotion_data->specific->motion[i].TimeCumulM[3] \
      + (int)softMotion_data->specific->motion[i].TimesM.Tvc;
    softMotion_data->specific->motion[i].TimeCumulM[5] = (int)softMotion_data->specific->motion[i].TimeCumulM[4] \
      + (int)softMotion_data->specific->motion[i].TimesM.Tjnb;
    softMotion_data->specific->motion[i].TimeCumulM[6] = (int)softMotion_data->specific->motion[i].TimeCumulM[5] \
      + (int)softMotion_data->specific->motion[i].TimesM.Tacb;

    softMotion_data->specific->motion[i].TimeCumul[0] = 0.0;
    softMotion_data->specific->motion[i].TimeCumul[1] = softMotion_data->specific->motion[i].Times.Tjpa;
    softMotion_data->specific->motion[i].TimeCumul[2] = softMotion_data->specific->motion[i].TimeCumul[1] \
      + softMotion_data->specific->motion[i].Times.Taca;
    softMotion_data->specific->motion[i].TimeCumul[3] = softMotion_data->specific->motion[i].TimeCumul[2] \
      + softMotion_data->specific->motion[i].Times.Tjna;
    softMotion_data->specific->motion[i].TimeCumul[4] = softMotion_data->specific->motion[i].TimeCumul[3] \
      + softMotion_data->specific->motion[i].Times.Tvc;
    softMotion_data->specific->motion[i].TimeCumul[5] = softMotion_data->specific->motion[i].TimeCumul[4] \
      + softMotion_data->specific->motion[i].Times.Tjnb;
    softMotion_data->specific->motion[i].TimeCumul[6] = softMotion_data->specific->motion[i].TimeCumul[5] \
      + softMotion_data->specific->motion[i].Times.Tacb;

    //softMotion_data->freeflyer->motion.TimeCumulM[i][7] = softMotion_data->freeflyer->motion.TimeCumulM[i][6] + softMotion_data->freeflyer->motion.TimesM[i].Tjpb;

    if (softMotion_data->specific->motion[i].MotionDuration > *timeMotionMax) {
      *timeMotionMax = softMotion_data->specific->motion[i].MotionDuration;
      *axisMotionMax = i;
    }
  }
  return;
}

void p3d_softMotion_export_traj(p3d_rob* robotPt, p3d_traj* traj, int trajType,  char *fileName, bool flagPlot,
					    std::vector <int> &lp, std::vector < std::vector <double> > &positions,
					    SM_TRAJ &smTraj) {
  double SIMPLING_TIME = 0.01;

  int j=0;
  int nb_dof = 0, nbGpJnt = 0;
  p3d_localpath* localpathPt =  NULL;
  p3d_localpath* localpathSMPt =  NULL;
  configPt q = NULL;
  int index;
  FILE *filepTrajtr = NULL;
  p3d_softMotion_data *specificPt = NULL;
  double dq;
  int end_localpath = 0;
  double u = 0.0;
  double du, umax;

  std::vector <double>  qplot_i;
  std::vector <double>  q_arm;
  std::vector <double>  q_armOld;
  std::vector <double>  vqi;
  std::vector <double>  min, max;
  gnuplot_ctrl * h = NULL;


  SM_SEG seg;

  int I_can = 0;
  int lpId = 0;
  int upBodySm_mlpID = -1;

  
	
  if ((filepTrajtr = fopen(fileName,"w+"))==NULL) {
    printf("cannot open File %s", fileName);
  }

  index = 0;
  for(int iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
    if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
      upBodySm_mlpID = iGraph;
      break;
    }
  }
  if(upBodySm_mlpID == -1) {
    printf("ERROR p3d_softMotion_write_curve_for_bltplot unknow robot type \n");
    return;
  }
  nbGpJnt = robotPt->mlp->mlpJoints[upBodySm_mlpID]->nbJoints;
  std::cout << "there are " << nbGpJnt << std::endl;
  int nb_armDof =0;
  q_armOld.clear();
  vqi.clear();
  min.clear();
  max.clear();
	
  double min_i=0.0, max_i=0.0;
	
  for(int v=0; v<nbGpJnt; v++) {
    nb_dof = robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->dof_equiv_nbr;
    for(int k=0; k<nb_dof; k++) {
      if(trajType == 0) {
        q_armOld.push_back(traj->courbePt->mlpLocalpath[upBodySm_mlpID]->specific.softMotion_data->q_init[robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->index_dof + k]);
      } else {
	q_armOld.push_back(traj->courbePt->specific.softMotion_data->q_init[robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->index_dof + k]);
      }
      vqi.push_back(0.0);
      p3d_jnt_get_dof_bounds(robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]], k, &min_i, &max_i);
      min.push_back(min_i);
      max.push_back(max_i);
      j++;
      nb_armDof ++;
    }
  }
  positions.clear();
  smTraj.resize(nb_armDof);
  localpathPt = traj->courbePt;
  lpId = 0;

  while (localpathPt !=NULL) {
    if(trajType == 0) {
      localpathSMPt = localpathPt->mlpLocalpath[upBodySm_mlpID];
    } else {
      localpathSMPt = localpathPt;
    }
    if(localpathSMPt != NULL) {

      specificPt = localpathSMPt->specific.softMotion_data;	    
      for(int s=1; s<=7;s++) {
	for (int i=0;i<nb_armDof;i++) {
		
	  seg.lpId = lpId;
	  seg.timeOnTraj = 0.0;

	  if(s==1) { 
	    seg.IC.a = specificPt->specific->motion[i].IC.a;
	    seg.IC.v = specificPt->specific->motion[i].IC.v;
	    seg.IC.x = specificPt->specific->motion[i].IC.x;
	    seg.time = specificPt->specific->motion[i].Times.Tjpa;
	    seg.jerk = specificPt->specific->motion[i].jerk.J1*specificPt->specific->motion[i].Dir;
	  }

	  if(s==2) {
	    seg.IC.a = specificPt->specific->motion[i].Acc.Tjpa;
	    seg.IC.v = specificPt->specific->motion[i].Vel.Tjpa;
	    seg.IC.x = specificPt->specific->motion[i].Pos.Tjpa;
	    seg.time = specificPt->specific->motion[i].Times.Taca;
	    seg.jerk = 0.0;
	  }
	  if(s==3) { 
	    seg.IC.a = specificPt->specific->motion[i].Acc.Taca;
	    seg.IC.v = specificPt->specific->motion[i].Vel.Taca;
	    seg.IC.x = specificPt->specific->motion[i].Pos.Taca;		
	    seg.time = specificPt->specific->motion[i].Times.Tjna;
	    seg.jerk = -specificPt->specific->motion[i].jerk.J2*specificPt->specific->motion[i].Dir;
	  }
	  if(s==4) { 
	    seg.IC.a = specificPt->specific->motion[i].Acc.Tjna;
	    seg.IC.v = specificPt->specific->motion[i].Vel.Tjna;
	    seg.IC.x = specificPt->specific->motion[i].Pos.Tjna;	
	    seg.time = specificPt->specific->motion[i].Times.Tvc;
	    seg.jerk = 0.0;
	  }
	  if(s==5) { 
	    seg.IC.a = specificPt->specific->motion[i].Acc.Tvc;
	    seg.IC.v = specificPt->specific->motion[i].Vel.Tvc;
	    seg.IC.x = specificPt->specific->motion[i].Pos.Tvc;			
	    seg.time = specificPt->specific->motion[i].Times.Tjnb;
	    seg.jerk = -specificPt->specific->motion[i].jerk.J3*specificPt->specific->motion[i].Dir;
	  }
	  if(s==6) { 
	    seg.IC.a = specificPt->specific->motion[i].Acc.Tjnb;
	    seg.IC.v = specificPt->specific->motion[i].Vel.Tjnb;
	    seg.IC.x = specificPt->specific->motion[i].Pos.Tjnb;
	    seg.time = specificPt->specific->motion[i].Times.Tacb;
	    seg.jerk = 0.0;
	  }
	  if(s==7) { 
	    seg.IC.a = specificPt->specific->motion[i].Acc.Tacb;
	    seg.IC.v = specificPt->specific->motion[i].Vel.Tacb;
	    seg.IC.x = specificPt->specific->motion[i].Pos.Tacb;
	    seg.time = specificPt->specific->motion[i].Times.Tjpb;
	    seg.jerk = specificPt->specific->motion[i].jerk.J4*specificPt->specific->motion[i].Dir;
	  }
	  smTraj.traj[i].push_back(seg);
	}
      } 
    }
    lpId ++;
    localpathPt = localpathPt->next_lp;
  }

  smTraj.computeTimeOnTraj();
  //smTraj.print();
  localpathPt = traj->courbePt;
  u = 0.0;
  lpId = 0;
  while (localpathPt != NULL) {
    //specificPt = localpathPt->mlpLocalpath[upBodySm_mlpID]->specific.softMotion_data;
    umax = localpathPt->range_param;
    //activate the constraint for the local path
    p3d_desactivateAllCntrts(robotPt);
    for(int i = 0; i < localpathPt->nbActiveCntrts; i++){
      p3d_activateCntrt(robotPt, robotPt->cntrt_manager->cntrts[localpathPt->activeCntrts[i]]);
    }

    while (end_localpath < 1) {
      q_arm.clear();
      qplot_i.clear();
			
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      I_can = p3d_set_and_update_this_robot_conf_multisol(robotPt, q, NULL, 0, localpathPt->ikSol);
      if(I_can == FALSE) {
	printf("error config %d in lp %d\n",index, lpId);
      }
			  
      p3d_get_robot_config_into(robotPt, &q);
      // Check for the bounds for the arm
      j=0;
      for(int v=0; v<nbGpJnt; v++) {
	nb_dof = robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->dof_equiv_nbr;
	for(int k=0; k<nb_dof; k++) {
	  dq = fmod((q[robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->index_dof +k] - q_armOld[j]), 2*M_PI);
	  q_arm.push_back(q_armOld[j] + dq);
	  qplot_i.push_back(q_arm[j]);
	  j++;
	}
      }
      if(filepTrajtr != NULL) {
	for(unsigned int w=0; w<qplot_i.size();w++){
	  fprintf(filepTrajtr,"%f ",qplot_i[w]);
	}
	fprintf(filepTrajtr,"%d ", lpId);
	fprintf(filepTrajtr,"\n");
      }
  
      positions.push_back(qplot_i);
      lp.push_back(lpId);
      index = index + 1;
      q_armOld.clear();
      for(int i=0; i<nb_armDof; i++) {
	q_armOld.push_back(q_arm[i]);
      }
      p3d_destroy_config(robotPt, q);
      q = NULL;
      du = SIMPLING_TIME;
      u += du;
			
      if (u > umax - EPS6) {
	u -= umax;
	end_localpath++;
      }
    }
    localpathPt = localpathPt->next_lp;
    lpId ++;
    u=0;
    end_localpath = 0;
  }

  printf("lpId %d traj->nlp %d\n ",lpId, traj->nlp);

  if(filepTrajtr != NULL) {
    fclose(filepTrajtr);
    printf("File arm.traj created\n");
  }
	
  if(flagPlot == true) {
    FILE * f = NULL;
    f = fopen("temp.dat","w");

    for(unsigned int i=0; i<positions.size(); i++){
      fprintf(f,"%d ",i);
      for(unsigned int v=0; v<positions[i].size(); v++){
	fprintf(f,"%f ",positions[i][v]);
      }
      fprintf(f,"\n ");
    }

		
    fclose(f);
    h = gnuplot_init();
    if(h == NULL){
      printf("Gnuplot Init problem");
    }
    gnuplot_cmd(h,(char*)"set term wxt");
    gnuplot_cmd(h,(char*)"set xrange [%d:%d]",0,index-1);
    gnuplot_cmd(h,(char*)"set yrange [-4.5:4.5]");  // maxi for Jido is 255�
    std::string gnuplotCmd;
    gnuplotCmd.clear();
    char text[255];
    char text2[255];
    if(positions.size()>1){
      for(unsigned int i=0; i<positions.at(0).size(); i++){
	sprintf(text,"%d",i+1);
	sprintf(text2,"%d",i+2);
	if(i==0) {
	  gnuplotCmd.append("plot ");
	}
	gnuplotCmd.append("\"temp.dat\" using 1:");
	gnuplotCmd.append(text2);
	gnuplotCmd.append(" with lines lt ");
	gnuplotCmd.append(text);
	gnuplotCmd.append(" ti \"q");
	gnuplotCmd.append(text);
	if(i<qplot_i.size()-1){
	  gnuplotCmd.append("\", ");
	} else {
	  gnuplotCmd.append("\" ");
	}
      }	
    }
    gnuplot_cmd(h, (char*)gnuplotCmd.c_str());
  }
  return;
}


//void p3d_softMotion_write_single_curve_for_bltplot(p3d_rob* robotPt, p3d_traj* traj, char *fileName, bool flagPlot,
//						   std::vector <int> &lp, std::vector < std::vector <double> > &positions,
//						   SM_TRAJ &smTraj) {
//  	double SIMPLING_TIME = 0.01;
//  	//int index_dof = 0;
//  	int j=0;
//  	int nb_dof = 0, nbGpJnt = 0;
//  	p3d_localpath* localpathPt =  NULL;
//  	configPt q = NULL;
//  	int index;
//  	FILE *filepTrajtr = NULL;
//  	p3d_softMotion_data *specificPt = NULL;
//  	double dq;
//  	int end_localpath = 0;
//  	double u = 0.0;
//  	double du, umax;
//  // 	std::vector <std::vector <double> > segment_q;
//  // 	std::vector <SM_COND>  cond;
//  // 	SM_COND cond_i;
//  // 	SM_SEGMENT segment_i;
//  	std::vector <double>  qplot_i;
//  	std::vector <double>  q_arm;
//  	std::vector <double>  q_armOld;
//  	std::vector <double>  vqi;
//  	std::vector <double>  min, max;
//  	gnuplot_ctrl * h = NULL;
//  	MANIPULATION_SEGMENT_STR manip_seg;
//  	MANIPULATION_SEGMENT_AXIS_DATA_STR manip_seg_data;
//  	int I_can = 0;
//  	int lpId = 0;
//  	int upBodySm_mlpID = -1;
//  
//  	if ((filepTrajtr = fopen("arm2.traj","w+"))==NULL) {
//  		printf("cannot open File arm.traj");
//  	}
//  
//  	index = 0;
//  	for(int iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
//  		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
//  			upBodySm_mlpID = iGraph;
//  			break;
//  		}
//  	}
//  	if(upBodySm_mlpID == -1) {
//  	  printf("ERROR p3d_softMotion_write_curve_for_bltplot unknow robot type \n");
//  	  return;
//  	}
//  
//  
//  
//  
//  
//  	nbGpJnt = robotPt->mlp->mlpJoints[upBodySm_mlpID]->nbJoints;
//  	std::cout << "there are " << nbGpJnt << std::endl;
//  	int nb_armDof =0;
//  	q_armOld.clear();
//  	vqi.clear();
//  	min.clear();
//  	max.clear();
//  	segments.seg.clear();
//  
//  	double min_i=0.0, max_i=0.0;
//  	for(int v=0; v<nbGpJnt; v++) {
//  	  nb_dof = robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->dof_equiv_nbr;
//  	  for(int k=0; k<nb_dof; k++) {
//  		q_armOld.push_back(traj->courbePt->specific.softMotion_data->q_init[robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->index_dof + k]);
//  		vqi.push_back(0.0);
//  		p3d_jnt_get_dof_bounds(robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]], k, &min_i, &max_i);
//  		min.push_back(min_i);
//  		max.push_back(max_i);
//  		j++;
//  		nb_armDof ++;
//  	  }
//  	}
//  
//  	positions.clear();
//  	localpathPt = traj->courbePt;
//  	lpId = 0;
//  	while (localpathPt !=NULL) {
//  	  if(localpathPt != NULL) {
//  	  specificPt = localpathPt->specific.softMotion_data;
//  	  for(int s=1; s<=7;s++) {
//  	    manip_seg.time = 0.0;
//  	    if(s==1) { manip_seg.time = specificPt->specific->motion[0].Times.Tjpa;}
//  	    if(s==2) { manip_seg.time = specificPt->specific->motion[0].Times.Taca;}
//  	    if(s==3) { manip_seg.time = specificPt->specific->motion[0].Times.Tjna;}
//  	    if(s==4) { manip_seg.time = specificPt->specific->motion[0].Times.Tvc;}
//  	    if(s==5) { manip_seg.time = specificPt->specific->motion[0].Times.Tjnb;}
//  	    if(s==6) { manip_seg.time = specificPt->specific->motion[0].Times.Tacb;}
//  	    if(s==7) { manip_seg.time = specificPt->specific->motion[0].Times.Tjpb;}
//  
//  	    if(manip_seg.time > EPS6) {
//  	        manip_seg.lp = lpId;
//  		manip_seg.data.clear();
//  		for (int i=0;i<nb_armDof;i++) {
//  		  manip_seg_data.ic_a = specificPt->specific->motion[i].IC.a;
//  		  manip_seg_data.ic_a = specificPt->specific->motion[i].IC.v;
//  		  manip_seg_data.ic_a = specificPt->specific->motion[i].IC.x;
//  		  if(s==1) { manip_seg_data.jerk = specificPt->specific->motion[i].jerk.J1*specificPt->specific->motion[i].Dir;}
//  		  if(s==2) { manip_seg_data.jerk = 0.0;}
//  		  if(s==3) { manip_seg_data.jerk = -specificPt->specific->motion[i].jerk.J2*specificPt->specific->motion[i].Dir;}
//  		  if(s==4) { manip_seg_data.jerk = 0.0;}
//  		  if(s==5) { manip_seg_data.jerk = -specificPt->specific->motion[i].jerk.J3*specificPt->specific->motion[i].Dir;}
//  		  if(s==6) { manip_seg_data.jerk = 0.0;}
//  		  if(s==7) { manip_seg_data.jerk = specificPt->specific->motion[i].jerk.J4*specificPt->specific->motion[i].Dir;}
//  		  manip_seg.data.push_back(manip_seg_data);
//  		}
//  		segments.seg.push_back(manip_seg);
//  	    }
//  	  }
//  
//  	  }
//  	  lpId ++;
//  
//  	  localpathPt = localpathPt->next_lp;
//  	}
//  
//  	localpathPt = traj->courbePt;
//  	u = 0.0;
//  	lpId = 0;
//  	while (localpathPt != NULL) {
//  		//specificPt = localpathPt->mlpLocalpath[upBodySm_mlpID]->specific.softMotion_data;
//  		umax = localpathPt->range_param;
//  		//activate the constraint for the local path
//  		p3d_desactivateAllCntrts(robotPt);
//  		for(int i = 0; i < localpathPt->nbActiveCntrts; i++){
//  			p3d_activateCntrt(robotPt, robotPt->cntrt_manager->cntrts[localpathPt->activeCntrts[i]]);
//  		}
//  
//  		while (end_localpath < 1) {
//  			q_arm.clear();
//  			qplot_i.clear();
//  
//  			/* position of the robot corresponding to parameter u */
//  			q = localpathPt->config_at_param(robotPt, localpathPt, u);
//  			I_can = p3d_set_and_update_this_robot_conf_multisol(robotPt, q, NULL, 0, localpathPt->ikSol);
//  			if(I_can == FALSE) {
//                                printf("error config %d in lp %d\n",index, lpId);
//  			}
//  
//  			p3d_get_robot_config_into(robotPt, &q);
//  			// Check for the bounds for the arm
//  			j=0;
//  			 for(int v=0; v<nbGpJnt; v++) {
//  			    nb_dof = robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->dof_equiv_nbr;
//  			      for(int k=0; k<nb_dof; k++) {
//   				dq = fmod((q[robotPt->joints[robotPt->mlp->mlpJoints[upBodySm_mlpID]->joints[v]]->index_dof +k] - q_armOld[j]), 2*M_PI);
//   				q_arm.push_back(q_armOld[j] + dq);
//  				qplot_i.push_back(q_arm[j]);
//  				j++;
//  			      }
//  			 }
//  
//  			if(filepTrajtr != NULL) {
//  
//                           for(unsigned int w=0; w<qplot_i.size();w++){
//                            fprintf(filepTrajtr,"%f ",qplot_i[w]);
//  
//                           }
//  			 fprintf(filepTrajtr,"%d ", lpId);
//                           fprintf(filepTrajtr,"\n");
//  			}
//  
//  			positions.push_back(qplot_i);
//  			lp.push_back(lpId);
//  			index = index + 1;
//  			q_armOld.clear();
//  			for(int i=0; i<nb_armDof; i++) {
//  			  q_armOld.push_back(q_arm[i]);
//  			}
//  			p3d_destroy_config(robotPt, q);
//  			q = NULL;
//  			du = SIMPLING_TIME;
//  			u += du;
//  
//  			if (u > umax - EPS6) {
//  				u -= umax;
//  				end_localpath++;
//  			}
//  		}
//  		localpathPt = localpathPt->next_lp;
//  		lpId ++;
//  		u=0;
//  		end_localpath = 0;
//  	}
//  
//          printf("lpId %d traj->nlp %d\n ",lpId, traj->nlp);
//  
//  	if(filepTrajtr != NULL) {
//  	fclose(filepTrajtr);
//  	printf("File arm2.traj created\n");
//  	}
//  
//  	if(flagPlot == true) {
//  		FILE * f = NULL;
//  		f = fopen("temp2.dat","w");
//  
//  	        for(unsigned int i=0; i<positions.size(); i++){
//  		  fprintf(f,"%d ",i);
//  		  for(unsigned int v=0; v<positions[i].size(); v++){
//  			fprintf(f,"%f ",positions[i][v]);
//  		  }
//  		  fprintf(f,"\n ");
//  		}
//  
//  
//  		fclose(f);
//  		h = gnuplot_init();
//  		if(h == NULL){
//  			printf("Gnuplot Init problem");
//  		}
//  		gnuplot_cmd(h,(char*)"set term wxt");
//  		gnuplot_cmd(h,(char*)"set xrange [%d:%d]",0,index-1);
//  		gnuplot_cmd(h,(char*)"set yrange [-4.5:4.5]");  // maxi for Jido is 255�
//  		std::string gnuplotCmd;
//  		gnuplotCmd.clear();
//  		char text[255];
//  		char text2[255];
//  		if(positions.size()>1){
//  		  for(unsigned int i=0; i<positions.at(0).size(); i++){
//  		    sprintf(text,"%d",i+1);
//  		    sprintf(text2,"%d",i+2);
//  		    if(i==0) {
//  		      gnuplotCmd.append("plot ");
//  		    }
//  		    gnuplotCmd.append("\"temp2.dat\" using 1:");
//  		    gnuplotCmd.append(text2);
//  		    gnuplotCmd.append(" with lines lt ");
//  		    gnuplotCmd.append(text);
//  		    gnuplotCmd.append(" ti \"q");
//  		    gnuplotCmd.append(text);
//  		    if(i<qplot_i.size()-1){
//  		      gnuplotCmd.append("\", ");
//  		    } else {
//  			gnuplotCmd.append("\" ");
//  		    }
//  		  }
//  		}
//  		gnuplot_cmd(h, (char*)gnuplotCmd.c_str());
//  	}
//  return;
//}

///////////////////////////////////////////////
//                      UTILS							 	 //
///////////////////////////////////////////////

void lm_convert_p3dMatrix_To_GbTh(const p3d_matrix4 M ,Gb_th* th) {

  th->vx.x = M[0][0];
  th->vx.y = M[1][0];
  th->vx.z = M[2][0];
  th->vy.x = M[0][1];
  th->vy.y = M[1][1];
  th->vy.z = M[2][1];
  th->vz.x = M[0][2];
  th->vz.y = M[1][2];
  th->vz.z = M[2][2];
  th->vp.x = M[0][3];
  th->vp.y = M[1][3];
  th->vp.z = M[2][3];
  return;
}

void lm_convert_GbTh_To_p3dMatrix(const Gb_th* th, p3d_matrix4 M) {

  M[0][0] = th->vx.x;
  M[1][0] = th->vx.y;
  M[2][0] = th->vx.z;
  M[3][0] = 0.0;
  M[0][1] = th->vy.x;
  M[1][1] = th->vy.y;
  M[2][1] = th->vy.z;
  M[3][1] = 0.0;
  M[0][2] = th->vz.x;
  M[1][2] = th->vz.y;
  M[2][2] = th->vz.z;
  M[3][2] = 0.0;
  M[0][3] = th->vp.x;
  M[1][3] = th->vp.y;
  M[2][3] = th->vp.z;
  M[3][3] = 1.0;
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      if(M[i][j] < -1.0) {
	M[i][j] = -1.0;
      }
      if(M[i][j] > 1.0) {
	M[i][j] = 1.0;
      }
    }
  }

  return;
}
