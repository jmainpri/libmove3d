#ifdef MULTILOCALPATH

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Planner-pkg.h"

#define VELOCITY_BASE_JIDO 200

static int groupToPlan[MAX_MULTILOCALPATH_NB];

configPt p3d_mergeMultiLocalPathConfig(p3d_rob *r, int nConfig, configPt *configs, p3d_multiLocalPathJoint ** mlpJoints) {
  configPt q = NULL;
  if (nConfig >= 1 && configs && mlpJoints) {//simple protection
    q = p3d_copy_config(r, r->ROBOT_INTPOS);
    for (int i = 0; i < nConfig; i++) {//pour tous les noeuds de la liste
      if (configs[i] != NULL) {
        for (int j = 0; j < (mlpJoints[i])->nbJoints; j++) {//pour tous les joints correspondants au noeud
          p3d_jnt* joint = r->joints[(mlpJoints[i])->joints[j]];
          for (int k = 0; k < joint->dof_equiv_nbr; k++) {//pour tous les Dof du joint
            q[joint->index_dof+k] = configs[i][joint->index_dof+k];
          }
        }
      }
    }
    return q;
  }
  return NULL;
}

configPt p3d_separateMultiLocalPathConfig(p3d_rob *r, configPt refConfig, configPt config, int mlpID, p3d_multiLocalPathJoint ** mlpJoints) {
  configPt q = NULL;
  if (mlpJoints) {//simple protection
    q = p3d_copy_config(r, refConfig);
    for (int j = 0; j < (mlpJoints[mlpID])->nbJoints; j++) {//pour tous les joints correspondants
      p3d_jnt* joint = r->joints[(mlpJoints[mlpID])->joints[j]];
      for (int k = 0; k < joint->dof_equiv_nbr; k++) {//pour tous les Dof du joint
        q[joint->index_dof+k] = config[joint->index_dof+k];
      }
    }
  }
  return q;
}

/* allocation of Multi local path */
p3d_localpath * p3d_alloc_multiLocalPath_localpath(p3d_rob *robotPt, p3d_localpath** localpathSpecific, int lp_id, int is_valid) {
  p3d_localpath * localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

//  localpathPt->specific.softMotion_data = NULL;

  /* Initialization of the generic part */
  /* fields */
  localpathPt->type_lp = MULTI_LOCALPATH;
  localpathPt->valid = is_valid;
  localpathPt->lp_id = lp_id;
  localpathPt->prev_lp = NULL;
  localpathPt->next_lp = NULL;
  localpathPt->mlpID = -1;

  for (int j = 0; j < MAX_MULTILOCALPATH_NB ; j++) {
    localpathPt->mlpLocalpath[j] = NULL;
  }


  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    localpathPt->mlpLocalpath[i] = localpathSpecific[i];
  }

  /* methods associated to the local path */
  /* compute the length of the local path */
  localpathPt->length =
    (double(*)(p3d_rob*, p3d_localpath*))(p3d_multiLocalPath_length);
  /* extract from a local path a sub local path starting from length
  l1 and ending at length l2 */
  localpathPt->extract_sub_localpath =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*,
                         double, double))(p3d_extract_multiLocalPath);
  /* extract from a local path a sub local path starting from parameter
  u1 and ending at parameter u2 */
  localpathPt->extract_by_param =
    (p3d_localpath * (*)(p3d_rob*, p3d_localpath*,
                         double, double))(p3d_extract_multiLocalPath);
  /* destroy the localpath */
  localpathPt->destroy =
    (void(*)(p3d_rob*, p3d_localpath*))(p3d_multiLocalPath_destroy);
  /*copy the local path */
  localpathPt->copy =
    (p3d_localpath * (*)(p3d_rob*,
                         p3d_localpath*))(p3d_copy_multiLocalPath_localpath);
  /* computes the configuration at given distance along the path */
  localpathPt->config_at_distance =
    (configPt(*)(p3d_rob*, p3d_localpath*, double))(p3d_multiLocalPath_config_at_param);
  /* computes the configuration at given parameter along the path */
  localpathPt->config_at_param =
    (configPt(*)(p3d_rob*, p3d_localpath*, double))(p3d_multiLocalPath_config_at_param);
  /* from a configuration on a local path, this function computes an
  interval of parameter on the local path on which all the points
  of the robot move by less than the distance given as input.
  The interval is centered on the configuration given as input. The
  function returns the half length of the interval */
  localpathPt->stay_within_dist =
    (double(*)(p3d_rob*, p3d_localpath*, double, whichway, double*))(p3d_multiLocalPath_stay_within_dist);
  /* compute the cost of a local path */
  localpathPt->cost = (double(*)(p3d_rob*, p3d_localpath*))(p3d_multiLocalPath_cost);
  /* function that simplifies the sequence of two local paths: valid
  only for RS curves */
  localpathPt->simplify = (p3d_localpath * (*)(p3d_rob*, p3d_localpath*, int*))(p3d_simplify_multiLocalPath);
  /* write the local path in a file */
  localpathPt->write = (int (*)(FILE *, p3d_rob*, p3d_localpath*))(p3d_write_multiLocalPath);

  localpathPt->length_lp = 0.0;

  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (localpathPt->mlpLocalpath[i] != NULL) {
      //if(localpathPt->mlpLocalpath[i]->type_lp != SOFT_MOTION) {
      //localpathPt->length_lp = (localpathPt->mlpLocalpath[i]->range_param / VELOCITY_BASE_JIDO) * SM_S_T;
      //localpathPt->length_lp = localpathPt->mlpLocalpath[i]->length_lp;
      //} else {
      if (localpathPt->mlpLocalpath[i]->length_lp > localpathPt->length_lp) {
        localpathPt->length_lp = localpathPt->mlpLocalpath[i]->length_lp;
      }
      //}
    }
  }

  if (isnan(localpathPt->length_lp)) {
    printf("nan length_lp\n");

  }

//  for(int i=0; i<robotPt->mg->nbGraphs; i++) {
//   if(localpathPt->mlpLocalpath[i] != NULL) {
//    if(localpathPt->mlpLocalpath[i]->length_lp > 0.0) {
//     localpathPt->length_lp = 1;
//    }
//   }
//  }

  localpathPt->range_param = localpathPt->length_lp;
  localpathPt->ikSol = NULL;
  return localpathPt;
}


/*
 * destroys a structures of type p3d_****_data
 */
void p3d_destroy_multiLocalPath_data(p3d_rob* robotPt) {
  return;
}


/*
 * destroy a linear local path
 */
void p3d_multiLocalPath_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt) {

  if (localpathPt != NULL) {

    /* test whether the type of local path is the expected one */
    if (localpathPt->type_lp != MULTI_LOCALPATH) {
      PrintError(("p3d_softMotion_destroy: softMotion local path expected\n"));
    }
    for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
      if (localpathPt->mlpLocalpath[i] != NULL) {
        localpathPt->mlpLocalpath[i]->destroy(robotPt, localpathPt->mlpLocalpath[i]);
        localpathPt->mlpLocalpath[i] = NULL;
      }
    }

    localpathPt->next_lp = NULL;
    localpathPt->prev_lp = NULL;
    if (localpathPt->ikSol){
      p3d_destroy_specific_iksol(robotPt->cntrt_manager, localpathPt->ikSol);
      localpathPt->ikSol = NULL;
    }
    MY_FREE(localpathPt, p3d_localpath, 1);

  }
}

/*
 *  Compute the configuration situated at given distance on the local path.
 *
 *  Input:  the robot, the distance.
 *
 *  Output: the configuration
 */
configPt p3d_multiLocalPath_config_at_param(p3d_rob *robotPt, p3d_localpath *localpathPt, double param) {
  configPt q[robotPt->mlp->nblpGp];
  configPt qRes = NULL;
  double mgParam = 0.0;

  int i = 0;

  for (i = 0; i < robotPt->mlp->nblpGp; i++) {
    q[i] = NULL;
    if (localpathPt->mlpLocalpath[i] != NULL) {

      mgParam = (param / localpathPt->length_lp) * localpathPt->mlpLocalpath[i]->range_param;
      if (mgParam >= localpathPt->mlpLocalpath[i]->range_param) {
        mgParam = localpathPt->mlpLocalpath[i]->range_param;
      }
      q[i] = localpathPt->mlpLocalpath[i]->config_at_param(robotPt, localpathPt->mlpLocalpath[i], mgParam);
    } else {
      q[i] = NULL;
    }
  }

  /* Merge config of each multiLocalPath */
  qRes = p3d_mergeMultiLocalPathConfig(robotPt, robotPt->mlp->nblpGp, q, robotPt->mlp->mlpJoints);

  for (i = 0; i < robotPt->mlp->nblpGp; i++) {
    p3d_destroy_config(robotPt, q[i]);
  }

  /* Update the configuration in the robot structures */
  p3d_copy_config_into(robotPt, qRes, &(robotPt->ROBOT_INTPOS));
  //robotPt->ROBOT_INTPOS = p3d_copy_config(robotPt, qRes);
  return qRes;
}

double p3d_multiLocalPath_length(p3d_rob* robotPt, p3d_localpath* localpathPt) {
  double length_lp = 0.0;
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (localpathPt->mlpLocalpath[i] != NULL) {
      if (localpathPt->mlpLocalpath[i]->length_lp > length_lp) {
        length_lp = localpathPt->mlpLocalpath[i]->length_lp;
      }
    }
  }
  return length_lp;
}


/*
 * destroy a linear local path
 */
void p3d_write_multiLocalPath(FILE *filePtr, p3d_rob* robotPt, p3d_localpath* localpathPt) {
  return;
}

/*  p3d_lin__within_dist
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
double p3d_multiLocalPath_stay_within_dist(p3d_rob* robotPt,
    p3d_localpath* localpathPt,
    double parameter, whichway dir_in,
    double *distances) {
  double dist, distTmp;
  int distIsInit = 0;
  double parameterScaled = 0.0;
  whichway dir;
  double dmax = 0, *mlpDistance = NULL;
  int njnt = robotPt->njoints;
  configPt currentRobotConfig = p3d_get_robot_config(robotPt);

  dmax = p3d_get_env_graphic_dmax();

  mlpDistance = MY_ALLOC(double, njnt + 1);
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {

    if (localpathPt->mlpLocalpath[i] != NULL) {

      parameterScaled  = (parameter / localpathPt->range_param) * localpathPt->mlpLocalpath[i]->range_param;
      if (parameterScaled >= localpathPt->mlpLocalpath[i]->range_param) {
        parameterScaled = localpathPt->mlpLocalpath[i]->range_param;
      }
      for (int j = 0; j <= njnt; j++) {
        mlpDistance[j] = dmax;
      }
      dir = dir_in;

      configPt q0 = localpathPt->config_at_param(robotPt, localpathPt, 0.0);
      // configPt q = p3d_separateMultiLocalPathConfig(robotPt, localpathPt->mlpLocalpath[i]->specific.lin_data->q_end, currentRobotConfig , i, robotPt->mlp->mlpJoints);
      configPt q = p3d_separateMultiLocalPathConfig(robotPt, q0, currentRobotConfig , i, robotPt->mlp->mlpJoints);
      p3d_set_and_update_robot_conf(q);
      distTmp = localpathPt->mlpLocalpath[i]->stay_within_dist(robotPt, localpathPt->mlpLocalpath[i], parameterScaled, dir, mlpDistance);
      p3d_destroy_config(robotPt, q);
      p3d_destroy_config(robotPt, q0);
      //scale le distTmp
      distTmp = localpathPt->range_param / (localpathPt->mlpLocalpath[i]->range_param / distTmp);
      if (distIsInit == 0) {
        distIsInit = 1;
        dist = distTmp;
      } else {
        if (distTmp < dist) {
          dist = distTmp;
        }
      }
    }
  }
  p3d_set_robot_config(robotPt, currentRobotConfig);
  p3d_destroy_config(robotPt, currentRobotConfig);
  MY_FREE(mlpDistance, double, njnt + 1);
  return dist;
}


/*
 *  Copy one local path.
 *
 *  Input:  the robot, the local path.
 *
 *  Output: the copied local path
 */

p3d_localpath *p3d_copy_multiLocalPath_localpath(p3d_rob* robotPt,
    p3d_localpath* localpathPt) {
  p3d_localpath *localpathPtMg = NULL;
  p3d_localpath *localpath[robotPt->mlp->nblpGp];

  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (groupToPlan[i] == 1) {
      if ((localpath[i] = MY_ALLOC(p3d_localpath, 1)) == NULL)
        return NULL;
      localpath[i]->mlpID = i;
    } else {
      localpath[i] = NULL;
    }
  }


  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (localpathPt->mlpLocalpath[i] != NULL) {
      localpath[i] = localpathPt->mlpLocalpath[i]->copy(robotPt, localpathPt->mlpLocalpath[i]);
      localpath[i]->mlpID = i;
    } else {
      localpath[i] = NULL;
    }
  }

  localpathPtMg = p3d_alloc_multiLocalPath_localpath(robotPt, localpath, localpathPt->lp_id, localpathPt->valid);

  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(localpathPtMg->ikSol));

  return localpathPtMg;
}

/*
 *  Extract from a linear local path the sub local path starting
 *  at length l1 and ending at length l2.
 *  The length of the extracted local path is computed
 *
 *  If l2 > length local path, return end of local path
 */
p3d_localpath *p3d_extract_multiLocalPath(p3d_rob *robotPt,
    p3d_localpath *localpathPt,
    double l1, double l2) {
  p3d_localpath *sub_localpathPt;
  int nblpGp = robotPt->mlp->nblpGp;
  p3d_localpath *mlpLocalpath[nblpGp];
  double l1_l, l2_l;
//  configPt qiMg[nbGraphs], qfMg[nbGraphs];
//  configPt qi, qf;
  // TODO

  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (groupToPlan[i] == 1) {
      if ((mlpLocalpath[i] = MY_ALLOC(p3d_localpath, 1)) == NULL)
        return NULL;
      mlpLocalpath[i]->mlpID = i;
    } else {
      mlpLocalpath[i] = NULL;
    }
  }

  for (int i = 0; i < nblpGp; i++) {
    if (localpathPt->mlpLocalpath[i] != NULL) {
      l1_l = (l1 * localpathPt->mlpLocalpath[i]->range_param) / localpathPt->length_lp;
      if (l1_l >= localpathPt->mlpLocalpath[i]->range_param) {
        l1_l = localpathPt->mlpLocalpath[i]->range_param;
      }


      l2_l = (l2 * localpathPt->mlpLocalpath[i]->range_param) / localpathPt->length_lp;
      if (l2_l >= localpathPt->mlpLocalpath[i]->range_param) {
        l2_l = localpathPt->mlpLocalpath[i]->range_param;
      }
//    qiMg[i] = localpathPt->mlpLocalpath[i]->config_at_param(robotPt, localpathPt->mlpLocalpath[i], l1_l);
//   qfMg[i] = localpathPt->mlpLocalpath[i]->config_at_param(robotPt, localpathPt->mlpLocalpath[i], l2_l);

      mlpLocalpath[i] = localpathPt->mlpLocalpath[i]->extract_by_param(robotPt, localpathPt->mlpLocalpath[i], l1_l, l2_l);
    }
  }

// qi = p3d_mergemultiLocalPathConfig(robotPt, nbGraphs, qiMg, robotPt->mg->mgJoints);
// qf = p3d_mergemultiLocalPathConfig(robotPt, nbGraphs, qfMg, robotPt->mg->mgJoints);
//  for(int i=nbGraphs; i>0; i--) {
//   if(localpathPt->mlpLocalpath[i] != NULL) {
//    p3d_destroy_config(robotPt, qiMg[i]);
//    p3d_destroy_config(robotPt, qfMg[i]);
//   }
//  }
  sub_localpathPt = p3d_alloc_multiLocalPath_localpath(robotPt, mlpLocalpath, localpathPt->lp_id, localpathPt->valid);

  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
  return sub_localpathPt;
}

/*
 *  Cost of a local path
 *
 *  Input:  the local path
 *
 *  Output: the cost
 */
double p3d_multiLocalPath_cost(p3d_rob *robotPt, p3d_localpath *localpathPt) {
  double length_lp = 0.0;
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (localpathPt->mlpLocalpath[i] != NULL) {
      if (localpathPt->mlpLocalpath[i]->length_lp > length_lp) {
        length_lp = localpathPt->mlpLocalpath[i]->length_lp;
      }
    }
  }
  return length_lp;
}

/*
 *  does nothing
 */

p3d_localpath *p3d_simplify_multiLocalPath(p3d_rob *robotPt, p3d_localpath *localpathPt,
    int *need_colcheck) {
  return localpathPt;
}


/*
 * Linear local planner
 *
 * Input:  the robot, two configurations
 *
 * Output: a local path.
 *
 * Allocation: the initial and goal config are copied
 */
p3d_localpath *p3d_multiLocalPath_localplanner(p3d_rob *robotPt, int multiLocalPathID, p3d_softMotion_data** softMotion_data,
    configPt qi, configPt qf, configPt qfp1, int* ikSol) {
  p3d_localplanner_type lplType;
  int nblpGp = robotPt->mlp->nblpGp;
  p3d_localpath *localpathPt[nblpGp];
  p3d_localpath *localpathMg = NULL;
  configPt qfTmp[nblpGp];
  configPt qfp1Tmp[nblpGp];

// robotPt->ROBOT_INTPOS = p3d_copy_config(robotPt, qi);
  p3d_copy_config_into(robotPt, qi, &(robotPt->ROBOT_INTPOS));


  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    /* Separate config of each multiLocalPath */
    qfTmp[i] = p3d_separateMultiLocalPathConfig(robotPt, robotPt->ROBOT_INTPOS, qf , i, robotPt->mlp->mlpJoints);
    qfp1Tmp[i] = p3d_separateMultiLocalPathConfig(robotPt, robotPt->ROBOT_INTPOS, qfp1 , i, robotPt->mlp->mlpJoints);


//   if(groupToPlan[i] == 1) {
//    if ((localpathPt[i] = MY_ALLOC(p3d_localpath, 1)) == NULL)
//     return NULL;
//   } else {
//    localpathPt[i] = NULL;
//   }
  }

  for (int i = 0; i < nblpGp; i++) {
    if (groupToPlan[i] == 1) {
      lplType = robotPt->mlp->mlpJoints[i]->lplType;

      if (lplType == P3D_SOFT_MOTION_PLANNER) {

        localpathPt[i] = p3d_softMotion_localplanner(robotPt, i, softMotion_data[i], qi, qfTmp[i] , qfp1Tmp[i]);
        if (localpathPt[i] != NULL) {
          localpathPt[i]->mlpID = i;
          for (int j = 0; j < MAX_MULTILOCALPATH_NB ; j++) {
            localpathPt[i]->mlpLocalpath[j] = NULL;
          }
        }
      } else {
        int * ikSolSub = NULL;
        p3d_copy_iksol(robotPt->cntrt_manager, ikSol, &ikSolSub);
        localpathPt[i] = array_localplanner[lplType](robotPt, qi, qfTmp[i] , ikSolSub);
        if (localpathPt[i] != NULL) {
          localpathPt[i]->mlpID = i;
          for (int j = 0; j < MAX_MULTILOCALPATH_NB ; j++) {
            localpathPt[i]->mlpLocalpath[j] = NULL;
          }
        }
      }
    } else if (groupToPlan[i] == 0) {
      localpathPt[i] = NULL;

    } else {
      printf("ERROR MULTILOCALPATH localpath : wrong number in value of the vector groupToPlan\n");
    }
  }

  for (int v = 0; v < robotPt->mlp->nblpGp; v++) {
    p3d_destroy_config(robotPt, qfTmp[v]);
    p3d_destroy_config(robotPt, qfp1Tmp[v]);
  }

  localpathMg = p3d_alloc_multiLocalPath_localpath(robotPt, localpathPt, 0, TRUE);
  int * ikSolSub = NULL;
  p3d_copy_iksol(robotPt->cntrt_manager, ikSol, &ikSolSub);
  localpathMg->ikSol = ikSolSub;

  return localpathMg;

}

void lm_destroy_multiLocalPath_params(p3d_rob *robotPt, void *paramPt) {
  return;
}

void p3d_multiLocalPath_set_groupToPlan(p3d_rob* robotPt, int mlpID, int value) {
  if ((mlpID < 0) && (mlpID > (robotPt->mlp->nblpGp - 1))) {
    printf("p3d_multiLocalPath_set_groupToPlan : mgID out of nbGroups\n");
    return;
  }
  if (value == TRUE) {
    groupToPlan[mlpID] = 1;
  } else if (value == FALSE) {
    groupToPlan[mlpID] = 0;
  } else {
    printf("p3d_multiLocalPath_set_groupToPlan : value %d is incompatible\n", value);
  }
  return;
}

void p3d_multiLocalPath_init_groupToPlan(p3d_rob* robotPt) {
  for (int i = 0; i < MAX_MULTILOCALPATH_NB; i++) {
    groupToPlan[i] = 0;
  }
}

int p3d_multiLocalPath_get_value_groupToPlan(p3d_rob* robotPt, const int mlpID) {

  if (mlpID >= 0 && mlpID < robotPt->mlp->nblpGp) {
    return groupToPlan[mlpID];
  } else {
    return 0;
  }
}


void p3d_multiLocalPath_disable_all_groupToPlan(p3d_rob* robotPt) {
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    groupToPlan[i] = 0;
  }
}

void p3d_multiLocalPath_enable_all_groupToPlan(p3d_rob* robotPt) {
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    groupToPlan[i] = 1;
  }
}

#endif
