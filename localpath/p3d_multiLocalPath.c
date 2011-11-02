#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Planner-pkg.h"
#ifdef LIGHT_PLANNER
#include "ManipulationUtils.hpp"
#endif

#define VELOCITY_BASE_JIDO 200

static int groupToPlan[MAX_MULTILOCALPATH_NB];

/**
 * Merge the configurations of the sub_localpaths of the multilocalpath
 * @param r The robot
 * @param nConfig The number of configs
 * @param configs The array of configs
 * @param mlpJoints The list of multiLocalpathJoints
 * @return
 */
configPt p3d_mergeMultiLocalPathConfig(p3d_rob *r, configPt qRef, int nConfig, configPt *configs, p3d_multiLocalPathJoint ** mlpJoints) {
  configPt q = NULL;
  int indexJnt = 0;
  if (nConfig >= 1 && configs && mlpJoints) {//simple protection
    q = p3d_copy_config(r, qRef);
    for (int i = 0; i < nConfig; i++) {//pour tous les noeuds de la liste
      if (configs[i] != NULL) {
        for (int j = 0; j < (mlpJoints[i])->nbJoints; j++) {//pour tous les joints correspondants au noeud
	  indexJnt = (mlpJoints[i])->joints[j];
          p3d_jnt* joint = r->joints[indexJnt];
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


/**
 * Extract one configuration of the multilocalpath for a specified group, joints that are not included
 * in the specified group are set to the refConfig values
 * @param r The robot
 * @param refConfig The reference config
 * @param config The config that must be separated
 * @param mlpID The ID of the specified group
 * @param mlpJoints The list of multiLocalpathJoints
 * @return
 */
configPt p3d_separateMultiLocalPathConfig(p3d_rob *r, configPt refConfig, configPt config, int mlpID, p3d_multiLocalPathJoint ** mlpJoints) {
  configPt q = NULL;
  int indexJnt = 0;
  if (mlpJoints) {//simple protection
    q = p3d_copy_config(r, refConfig);
    for (int j = 0; j < (mlpJoints[mlpID])->nbJoints; j++) {
      //pour tous les joints correspondants
      indexJnt = (mlpJoints[mlpID])->joints[j];
      p3d_jnt* joint = r->joints[indexJnt];
      for (int k = 0; k < joint->dof_equiv_nbr; k++) {
	//pour tous les Dof du joint
	q[joint->index_dof+k] = config[joint->index_dof+k];
      }
    }
   }

   // Copy the passive jnt : needed for the stay within dist
   for (int k = 0; k < r->nb_dof; k++) {
     //pour tous les joints correspondant
     if (r->cntrt_manager->in_cntrt[k] == 2) {
      q[k] = config[k];
     }
   }
   return q;
}

/**
 * allocation of Multi local path
 * @param robotPt The robot
 * @param localpathSpecific The array of sub_localpaths
 * @param lp_id
 * @param is_valid
 * @return
 */
p3d_localpath * p3d_alloc_multiLocalPath_localpath(p3d_rob *robotPt, p3d_localpath** localpathSpecific, int lp_id, int is_valid) {
  p3d_localpath * localpathPt = NULL;

  if ((localpathPt = MY_ALLOC(p3d_localpath, 1)) == NULL)
    return NULL;

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
// 		localpathPt->mlpLocalpath[i]->q_init = NULL;
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
      if (localpathPt->mlpLocalpath[i]->length_lp > localpathPt->length_lp) {
        localpathPt->length_lp = localpathPt->mlpLocalpath[i]->length_lp;
      }
    }
  }

  if (isnan(localpathPt->length_lp)) {
    printf("nan length_lp\n");
  }

  localpathPt->range_param = localpathPt->length_lp;
  localpathPt->ikSol = NULL;
  localpathPt->nbActiveCntrts = 0;
  localpathPt->activeCntrts = NULL;

  for(int i=0; i<MAX_MULTILOCALPATH_NB; i++) {
   localpathPt->mlpGpIsActive[i] = groupToPlan[i];
  }
#if defined(LIGHT_PLANNER)
	localpathPt->isCarryingObject = FALSE;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++) {
    localpathPt->carriedObject[i] = NULL;
  }
//	localpathPt->carriedObject = NULL;
	//	p3d_mat4Copy(p3d_mat4IDENTITY, localpathPt->Tgrasp);
#endif

  return localpathPt;
}


/**
 * Destroys a structures of type p3d_multiLocalPath_data
 * @param robotPt The robot
 */
void p3d_destroy_multiLocalPath_data(p3d_rob* robotPt) {
  return;
}

/**
 * Destroy a multilocalpath local path
 * @param robotPt The robot
 * @param localpathPt The localpath
 */
void p3d_multiLocalPath_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt) {
  if (localpathPt != NULL) {
    /* test whether the type of local path is the expected one */
    if (localpathPt->type_lp != MULTI_LOCALPATH) {
			PrintError(("p3d_multiLocalPath_destroy: MULTI_LOCALPATH local path expected\n"));
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
		if (localpathPt->q_init){
			p3d_destroy_config(robotPt, localpathPt->q_init);
			localpathPt->q_init = NULL;
		}


    if(localpathPt->activeCntrts != NULL) {
      MY_FREE(localpathPt->activeCntrts, int, localpathPt->nbActiveCntrts);
    }
    MY_FREE(localpathPt, p3d_localpath, 1);
  }
}

/**
 * Compute the configuration situated at given distance on the local path.
 * @param robotPt  The robot
 * @param localpathPt The localpath
 * @param param The distance
 * @return The configuration
 */
configPt p3d_multiLocalPath_config_at_param(p3d_rob *robotPt, p3d_localpath *localpathPt, double param) {
  configPt q[robotPt->mlp->nblpGp];
  configPt qRes = NULL;
  double mgParam = 0.0;

// Reactive les bons les bons groupes de mlp
//   p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
//   for(int i=0; i<MAX_MULTILOCALPATH_NB; i++) {
//     p3d_multiLocalPath_set_groupToPlan(robotPt, i, localpathPt->mlpGpIsActive[i]);
//   }
  

  int i = 0;
  if(param == 0.0){
	return p3d_copy_config(robotPt, localpathPt->q_init);
  }


  if(fabs(localpathPt->length_lp)< 1e-10) {
	return p3d_copy_config(robotPt, localpathPt->q_init);
  }

  if (param > localpathPt->length_lp)
    param = localpathPt->length_lp;
  
  for (i = 0; i < robotPt->mlp->nblpGp; i++) {
    q[i] = NULL;

    if(localpathPt->mlpGpIsActive[i] == 1) {
      if (localpathPt->mlpLocalpath[i] != NULL) {
	mgParam = (param / localpathPt->length_lp) * localpathPt->mlpLocalpath[i]->range_param;
	if (mgParam >= localpathPt->mlpLocalpath[i]->range_param) {
	  mgParam = localpathPt->mlpLocalpath[i]->range_param;
	}
	q[i] = localpathPt->mlpLocalpath[i]->config_at_param(robotPt, localpathPt->mlpLocalpath[i], mgParam);
      } else {
	q[i] = NULL;
      }
    } else {
      q[i] = NULL;
    }
    
  }
  /* Merge config of each multiLocalPath */
  qRes = p3d_mergeMultiLocalPathConfig(robotPt, localpathPt->q_init, robotPt->mlp->nblpGp, q, robotPt->mlp->mlpJoints);

  for (i = 0; i < robotPt->mlp->nblpGp; i++) {
    p3d_destroy_config(robotPt, q[i]);
  }

  return qRes;
}

/**
 * Determine the length of the multilocalpath local path
 * @param robotPt The length
 * @param localpathPt The localpath
 * @return The length
 */
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

/**
 * Write a multilocalpath local path into a file
 * @param filePtr The file
 * @param robotPt The robot
 * @param localpathPt The localpath
 */
int p3d_write_multiLocalPath(FILE *file, p3d_rob* robotPt, p3d_localpath* localpathPt) {

	if (localpathPt->type_lp != MULTI_LOCALPATH) {
		return FALSE;
	}

	fprintf(file, "\n\np3d_add_localpath MULTI_LOCALPATH\n");
	fprintf(file, "nbGroup %d\n",robotPt->mlp->nblpGp);
	fprintf(file, "active");
	for(int i=0; i<robotPt->mlp->nblpGp; i++) {
		if(localpathPt->mlpLocalpath[i] != NULL) {
		fprintf(file, " 1");
		} else {
		fprintf(file, " 0");
		}
	}
	fprintf(file, "\n");
	fprintf(file, "conf_init");
	fprint_config_one_line(file, robotPt, localpathPt->config_at_distance(robotPt, localpathPt, 0.0));
	fprintf(file, "\n");
	for(int i=0; i<robotPt->mlp->nblpGp; i++) {
		localpathPt->mlpLocalpath[i]->write(file, robotPt, localpathPt->mlpLocalpath[i]);
	}
	fprintf(file, "\np3d_end_local_path\n");

  return TRUE;
}

/**
 * From a configuration on a local path, this function
 * computes an interval of parameter on the local path on
 * which all the points of the robot move by less than the
 * distance given as input.  The interval is centered on the
 * configuration given as input. The function returns the
 * half length of the interval
 * @param robotPt The robot
 * @param localpathPt The localpath
 * @param parameter The parameter along the curve
 * @param dir_in The direction of motion
 * @param distances The maximal distance moved by all the points of the robot
 * @return The length of the interval of parameter the robot can stay on without any body moving by more than the input distance
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

  dmax = p3d_get_env_dmax();

//   p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
//   for(int i=0; i<MAX_MULTILOCALPATH_NB; i++) {
//     p3d_multiLocalPath_set_groupToPlan(robotPt, i, localpathPt->mlpGpIsActive[i]);
//   }
  

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
      p3d_set_robot_config(robotPt, currentRobotConfig);

    }
  }
if(currentRobotConfig!=NULL){
      p3d_destroy_config(robotPt, currentRobotConfig);
      currentRobotConfig = NULL;
      }
  MY_FREE(mlpDistance, double, njnt + 1);
  return dist;
}

/**
 * Copy one local path
 * @param robotPt The robot
 * @param localpathPt The localpath
 * @return The copied local path
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
	localpathPtMg->q_init = p3d_copy_config(robotPt, localpathPt->q_init);
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(localpathPtMg->ikSol));
  localpathPtMg->nbActiveCntrts = localpathPt->nbActiveCntrts;
  if(localpathPt->nbActiveCntrts >0) {
   localpathPtMg->activeCntrts = MY_ALLOC(int, localpathPtMg->nbActiveCntrts);
   for(int i = 0; i < localpathPtMg->nbActiveCntrts; i++){
     localpathPtMg->activeCntrts[i] = localpathPt->activeCntrts[i];
   }
  } else {
   localpathPtMg->activeCntrts = NULL;
  }

#if defined(LIGHT_PLANNER)
	localpathPtMg->isCarryingObject = localpathPt->isCarryingObject;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++) {
    localpathPtMg->carriedObject[i] = localpathPt->carriedObject[i];/*!< pointer to the carried object (obstacle environment or robot body) */
  }
	//	p3d_mat4Copy(localpathPt->Tgrasp, localpathPtMg->Tgrasp);
#endif

 
  for(int i=0; i<MAX_MULTILOCALPATH_NB; i++) {
    localpathPtMg->mlpGpIsActive[i] = localpathPt->mlpGpIsActive[i];
  }
  

  return localpathPtMg;
}

/**
 * Extract from a linear local path the sub local path starting
 * at length l1 and ending at length l2.
 * The length of the extracted local path is computed
 * If l2 > length local path, return end of local path
 * @param robotPt The robot
 * @param localpathPt The localpath
 * @param l1 The length l1
 * @param l2 The length l2
 * @return The extracted localpath
 */
p3d_localpath *p3d_extract_multiLocalPath(p3d_rob *robotPt,
    p3d_localpath *localpathPt,
    double l1, double l2) {
  p3d_localpath *sub_localpathPt;
  int nblpGp = robotPt->mlp->nblpGp;
  p3d_localpath *mlpLocalpath[nblpGp];
  double l1_l, l2_l;
// 	configPt qiMg[nblpGp];//, qfMg[nbGraphs];
  configPt qi = NULL;

   qi = localpathPt->config_at_param(robotPt, localpathPt, l1);
   if (robotPt->cntrt_manager->cntrts != NULL) {
	p3d_set_and_update_this_robot_conf(robotPt, qi);
   }


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
//       qiMg[i] = localpathPt->mlpLocalpath[i]->config_at_param(robotPt, localpathPt->mlpLocalpath[i], l1_l);
//   qfMg[i] = localpathPt->mlpLocalpath[i]->config_at_param(robotPt, localpathPt->mlpLocalpath[i], l2_l);

      mlpLocalpath[i] = localpathPt->mlpLocalpath[i]->extract_by_param(robotPt, localpathPt->mlpLocalpath[i], l1_l, l2_l);
    } else {
      mlpLocalpath[i] = NULL;
// 			qiMg[i] = NULL;
    }
  }

// 	sub_localpathPt->q_init = p3d_mergeMultiLocalPathConfig(robotPt, localpathPt->q_init, nblpGp, qiMg, robotPt->mlp->mlpJoints);


// qf = p3d_mergemultiLocalPathConfig(robotPt, nbGraphs, qfMg, robotPt->mg->mgJoints);
// 	for(int i=nblpGp; i>0; i--) {
// 		   if(localpathPt->mlpLocalpath[i] != NULL) {
//     p3d_destroy_config(robotPt, qiMg[i]);
//    p3d_destroy_config(robotPt, qfMg[i]);
//    }

//   }
  sub_localpathPt = p3d_alloc_multiLocalPath_localpath(robotPt, mlpLocalpath, localpathPt->lp_id, localpathPt->valid);
	sub_localpathPt->q_init = p3d_copy_config(robotPt, qi);
	p3d_destroy_config(robotPt, qi);
  p3d_copy_iksol(robotPt->cntrt_manager, localpathPt->ikSol, &(sub_localpathPt->ikSol));
  sub_localpathPt->nbActiveCntrts = localpathPt->nbActiveCntrts;
  if(localpathPt->nbActiveCntrts> 0) {
     sub_localpathPt->activeCntrts = MY_ALLOC(int, sub_localpathPt->nbActiveCntrts);

    for(int i = 0; i < sub_localpathPt->nbActiveCntrts; i++){
      sub_localpathPt->activeCntrts[i] = localpathPt->activeCntrts[i];
    }

  } else {
   sub_localpathPt->activeCntrts = NULL;
  }

#if defined(LIGHT_PLANNER)
	sub_localpathPt->isCarryingObject = localpathPt->isCarryingObject;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++){
    sub_localpathPt->carriedObject[i] = localpathPt->carriedObject[i]; /*!< pointer to the carried object (obstacle environment or robot body) */
  }
	//	p3d_mat4Copy(localpathPt->Tgrasp, sub_localpathPt->Tgrasp);
#endif

  return sub_localpathPt;
}

/**
 *  Cost of a local path
 * @param robotPt The robot
 * @param localpathPt The localpath
 * @return The cost
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


/**
 * Does nothing
 * @param robotPt
 * @param localpathPt
 * @param need_colcheck
 * @return
 */
p3d_localpath *p3d_simplify_multiLocalPath(p3d_rob *robotPt, p3d_localpath *localpathPt,
    int *need_colcheck) {
  return localpathPt;
}

/**
 * Compute a multilocalpath local path by calling the corresponding sub_localplanner for each groups
 * @param robotPt The robot
 * @param softMotion_data The parameter for softmotion
 * @param qi The initial configuration
 * @param qf The final configuration
 * @param qfp1  The final configuration of the next localpath
 * @param ikSol The Inverse kinematic solution
 * @return
 */
p3d_localpath *p3d_multiLocalPath_localplanner(p3d_rob *robotPt, p3d_softMotion_data** softMotion_data,
    configPt qi, configPt qf, configPt qfp1, int* ikSol) {
  p3d_localpath_type lplType;
  int nblpGp = robotPt->mlp->nblpGp;
  p3d_localpath *localpathPt[nblpGp];
  p3d_localpath *localpathMg = NULL;
  configPt qfTmp[nblpGp];
  configPt qfp1Tmp[nblpGp];

  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    /* Separate config of each multiLocalPath */
    qfTmp[i] = p3d_separateMultiLocalPathConfig(robotPt, qi, qf , i, robotPt->mlp->mlpJoints);
    qfp1Tmp[i] = p3d_separateMultiLocalPathConfig(robotPt, qi, qfp1 , i, robotPt->mlp->mlpJoints);
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

      if (lplType == SOFT_MOTION) {

        localpathPt[i] = p3d_softMotion_localplanner(robotPt, i, softMotion_data[i], qi, qfTmp[i] , qfp1Tmp[i], NULL);
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
			if (localpathPt[i] != NULL) {
				#if defined(LIGHT_PLANNER)
				localpathPt[i]->isCarryingObject = robotPt->isCarryingObject;
        for(unsigned int j = 0; j < robotPt->armManipulationData->size(); j++){
          localpathPt[i]->carriedObject[j] = ((*robotPt->armManipulationData)[j]).getCarriedObject();
        }
//				localpathPt[i]->carriedObject = robotPt->carriedObject; /*!< pointer to the carried object (obstacle environment or robot body) */
				//				p3d_mat4Copy(robotPt->Tgrasp, localpathPt[i]->Tgrasp);

				localpathPt[i]->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathPt[i]->nbActiveCntrts));
				
				#endif
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
  localpathMg->q_init = p3d_copy_config(robotPt, qi);
  localpathMg->ikSol = ikSolSub;
  localpathMg->activeCntrts = p3d_getActiveCntrts(robotPt,&(localpathMg->nbActiveCntrts));

#if defined(LIGHT_PLANNER)
	localpathMg->isCarryingObject = robotPt->isCarryingObject;
  for (int i = 0; i < MAX_CARRIED_OBJECTS; i++){
   if(i < robotPt->armManipulationData->size() ){
    localpathMg->carriedObject[i] = (*robotPt->armManipulationData)[i].getCarriedObject(); /*!< pointer to the carried object (obstacle environment or robot body) */
   }
  }
	//p3d_mat4Copy(robotPt->Tgrasp, localpathMg->Tgrasp);
#endif

  return localpathMg;
}

/**
 * Destroy parameter of a multilocalpath local path
 * @param robotPt The robot
 * @param paramPt The parameter pointer
 */
void lm_destroy_multiLocalPath_params(p3d_rob *robotPt, void *paramPt) {

  return;
}



int p3d_multiLocalPath_update_joint_sampling_activation(p3d_rob* robotPt) {

 int indexJnt = 0;
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
 
    for(int j=0; j<robotPt->mlp->mlpJoints[i]->nbJoints; j++) {
		   indexJnt = robotPt->mlp->mlpJoints[i]->joints[j];
			p3d_jnt_set_is_active_for_planner2(robotPt->joints[indexJnt], FALSE);
		}
    
  }
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if(groupToPlan[i] == 1) {
    		for(int j=0; j<robotPt->mlp->mlpJoints[i]->nbJoints; j++) {
		   indexJnt = robotPt->mlp->mlpJoints[i]->joints[j];
			p3d_jnt_set_is_active_for_planner2(robotPt->joints[indexJnt], TRUE);
		}
    }
  }
  return 0;
}

/**
 * Activate or De-activate the specified group "mlpID" of multilocalpath
 * @param robotPt  The robot
 * @param mlpID The ID of the group
 * @param value The value => 0 : De-activated / 1 : Activated
 */
void p3d_multiLocalPath_set_groupToPlan(p3d_rob* robotPt, int mlpID, int value, int updateJointSampling) {
  
  if ((mlpID < 0) || (mlpID > (robotPt->mlp->nblpGp - 1))) {
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
  if(updateJointSampling){
    p3d_multiLocalPath_update_joint_sampling_activation(robotPt);
  }
  return;
}

void p3d_multiLocalPath_set_groupToPlan_by_name(p3d_rob* robotPt, char* name, int flag) {
int i=0;
	for( i=0; i<robotPt->mlp->nblpGp; i++) {
		if(strcmp(name, robotPt->mlp->mlpJoints[i]->gpName) == 0) {
			p3d_multiLocalPath_set_groupToPlan(robotPt, i, flag);
			return;
		}
	}
	if(i==robotPt->mlp->nblpGp) {
		printf("p3d_multiLocalPath_set_groupToPlan_by_name can not fing the group %s\n",name);
	}
	p3d_multiLocalPath_update_joint_sampling_activation(robotPt);

}

/**
 * Initialize to De-activated all the group of multilocalpath
 * @param robotPt The robot
 */
void p3d_multiLocalPath_init_groupToPlan(p3d_rob* robotPt) {
  for (int i = 0; i < MAX_MULTILOCALPATH_NB; i++) {
    groupToPlan[i] = 0;
  }
  p3d_multiLocalPath_update_joint_sampling_activation(robotPt);
}

/**
 * Acces to the activation value of the specified group of multilocalpath
 * @param robotPt The robot
 * @param mlpID  The ID of the group
 * @return The value => 0 : De-activated / 1 : Activated
 */
int p3d_multiLocalPath_get_value_groupToPlan(p3d_rob* robotPt, const int mlpID) {
  if (mlpID >= 0 && mlpID < robotPt->mlp->nblpGp) {
    return groupToPlan[mlpID];
  } else {
    return 0;
  }
}

int p3d_multilocapath_print_group_info(p3d_rob* robotPt) {

   int i=0;
   int id;
   printf("**********************************\n");
   std::cout << " multilocalpath info\n" << std::endl;
   for( i=0; i<robotPt->mlp->nblpGp; i++) {

     id = p3d_multiLocalPath_get_group_by_name(robotPt, robotPt->mlp->mlpJoints[i]->gpName);
     printf("mlp_group: %s activation: %d\n",robotPt->mlp->mlpJoints[i]->gpName , p3d_multiLocalPath_get_value_groupToPlan(robotPt, id)); 
   }
   printf("**********************************\n");
  return 0;
}

/**
 *  Set to De-activated all the group of multilocalpath
 * @param robotPt The robot
 */
void p3d_multiLocalPath_disable_all_groupToPlan(p3d_rob* robotPt, int updateJointSampling) {
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    groupToPlan[i] = 0;
  }
  if(updateJointSampling){
    p3d_multiLocalPath_update_joint_sampling_activation(robotPt);
  }
}

/**
 * Set to Activated all the group of multilocalpath
 * @param robotPt The robot
 */
void p3d_multiLocalPath_enable_all_groupToPlan(p3d_rob* robotPt) {
  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    groupToPlan[i] = 1;
  }
  p3d_multiLocalPath_update_joint_sampling_activation(robotPt);
}

int p3d_multiLocalPath_get_group_by_name(p3d_rob* robotPt, char* name) {
   int i=0;
   for( i=0; i<robotPt->mlp->nblpGp; i++) {
     if(strcmp(name, robotPt->mlp->mlpJoints[i]->gpName) == 0) {
	return i;
	}
   }

   if(i==robotPt->mlp->nblpGp) {
     printf("p3d_multiLocalPath_get_group_by_name: can not find the group %s\n",name);
   }
   p3d_multiLocalPath_update_joint_sampling_activation(robotPt);
   return 0;
}
