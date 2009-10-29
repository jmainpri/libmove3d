#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"


/*
 *  Extract the local path that contains the configuration
 *  at a given length along a trajectory
 *
 *  In: The trajectory, The length
 *  Out: The local path, the distance along the local path of the
 *       configuration from the beginning of the local path
 */

p3d_localpath *p3d_localpath_from_traj(p3d_localpath *localpathPt,
                                       double length,
                                       double *distlocPt) {
  double l = 0;

  l = localpathPt->length_lp;

  while ((l < length) && (localpathPt != NULL)) {
    localpathPt = localpathPt->next_lp;
    l += localpathPt->length_lp;
  }

  l -= localpathPt->length_lp;
  *distlocPt = length - l;
  return localpathPt;
}



/*
 *  One step of optimization:
 *      2 configurations are randomly chosen on the trajectory
 *      the local planner is called between these configurations
 *      If the local path obtained is collision free and shorter
 *      than the initial portion, it replaces the portion in the
 *      trajectory.
 */

int p3d_optim_traj(p3d_traj *trajPt, double *gain, int *ntest) {
  p3d_rob *robotPt = trajPt->rob;
  p3d_localpath *localpath1Pt = NULL, *localpath2Pt = NULL,
                                *localpath3Pt = NULL, *cur_localpathPt = NULL;
  p3d_localpath *opt_pathPt[3] = {NULL, NULL, NULL};
  p3d_localpath *end_localpath_qiq1Pt, *start_localpath_q1q2Pt,
  *end_localpath_q1q2Pt = NULL, *start_localpath_q2qePt = NULL;
  p3d_localpath *start_new_trajPt = NULL,
                                    *end_new_trajPt = NULL;

  int q2_found = FALSE;
  double l = 0, l1 = 0, l2 = 0, ltot;
  double loc_dist1, lcur = 0, lnext = 0;
  configPt q1 = NULL, q2 = NULL, qinit = NULL, qgoal = NULL;
  double opt_cost[3], init_cost[3];
  double cost[8], cost_tmp[8], cost_min;
  int sort[8], collision[4] = { -1, -1, -1};
  int i, j, k, jmin = 0;
  char opt[3];
  int path_found = FALSE;
  int validIkSol[3] = {1,1,1};
  int flag = 0;
  /* for debugging */
  int i1 = 0, i2 = 0;
  double total_cost = 0.0;

  init_cost[0] = 0;
  *gain = 0.0;

  /* maximal penetration distance allowed */
  /* if trajectory is made of only one local path, no smoothing */
  if (trajPt->nlp < 2)
    return FALSE;

  /* length of trajPt */
  ltot = p3d_ends_and_length_traj(trajPt, &qinit, &qgoal);
  if (ltot <= 3*EPS6) {
    /* trajectory too short */
    p3d_destroy_config(robotPt, qinit);
    p3d_destroy_config(robotPt, qgoal);
    return FALSE;
  }

  /* pick random pairs of configurations. Make sure 0 < l1 < l2 < ltot */
  l2 = p3d_random(EPS6 * ltot, (1 - EPS6) * ltot);
  l1 = p3d_random(.25 * EPS6 * ltot, l2 - .25 * EPS6 * ltot);

  /* look for the first local path containing q1 */
  localpath1Pt = trajPt->courbePt;
  l = localpath1Pt->length_lp;


  while ((l < l1) && (localpath1Pt != NULL)) {
    init_cost[0] += localpath1Pt->cost(robotPt, localpath1Pt);
    localpath1Pt = localpath1Pt->next_lp;
    i1++;
    l += localpath1Pt->length_lp;
  }


  l -= localpath1Pt->length_lp;
  loc_dist1 = l1 - l;
  /* the local path containing q1 has been found */
  /* the portion of the local path containing q1 up to q1 is extracted
     in order to compute its cost */
  end_localpath_qiq1Pt =
    localpath1Pt->extract_sub_localpath(robotPt, localpath1Pt,
                                        0, loc_dist1);
  init_cost[0] += end_localpath_qiq1Pt->cost(robotPt, end_localpath_qiq1Pt);

  if(!p3d_compare_iksol(robotPt->cntrt_manager, trajPt->courbePt->ikSol, localpath1Pt->ikSol)){
    validIkSol[0] = 0;
  }

  localpath2Pt = localpath1Pt;

  if (l2 - l1 > localpath1Pt->length_lp - loc_dist1) {
    /* q2 is not on the same local path as q1 */
    start_localpath_q1q2Pt =
      localpath1Pt->extract_sub_localpath(robotPt,
                                          localpath1Pt, loc_dist1,
                                          localpath1Pt->length_lp);
    lcur = l1 - loc_dist1 + localpath1Pt->length_lp;
  } else {
    /* q1 and q2 are on the same local path */
    start_localpath_q1q2Pt =
      localpath1Pt->extract_sub_localpath(robotPt,
                                          localpath1Pt, loc_dist1,
                                          l2 - l1 + loc_dist1);
    /* compute second configuration */
    q2 = localpath1Pt->config_at_distance(robotPt,
                                          localpath1Pt,
                                          l2 - l1 + loc_dist1);
    if (robotPt->cntrt_manager->cntrts != NULL) {
      p3d_set_and_update_this_robot_conf(robotPt, q2);
      p3d_get_robot_config_into(robotPt, &q2);
    }

    /* copy second part of the path */
    end_localpath_q1q2Pt = start_localpath_q1q2Pt;
    /* copy beginning of thirs part of the path */
    start_localpath_q2qePt =
      localpath1Pt->extract_sub_localpath(robotPt, localpath1Pt,
                                          l2 - l1 + loc_dist1,
                                          localpath1Pt->length_lp);
    q2_found = TRUE;
  }
  /* compute the first configuration */
  q1 = start_localpath_q1q2Pt->
       config_at_distance(robotPt, start_localpath_q1q2Pt, 0);
  if (robotPt->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf_multisol(robotPt, q1,NULL, 0, localpath1Pt->ikSol);
    p3d_get_robot_config_into(robotPt, &q1);
    p3d_set_robot_iksol(robotPt, robotPt->ikSol);
  }

  /* cost of the second part of the trajectory (between q1 and q2)
     initialized */
  init_cost[1] =
    start_localpath_q1q2Pt->cost(robotPt, start_localpath_q1q2Pt);

  i2 = i1;
  while (q2_found == FALSE) {
    i2++;
    localpath2Pt = localpath2Pt->next_lp;
    lnext = lcur + localpath2Pt->length_lp;


    if (lnext < l2) {
      init_cost[1] += localpath2Pt->cost(robotPt, localpath2Pt);
    } else {
      /* the local path containing q2 has been found */
      q2_found = TRUE;
      end_localpath_q1q2Pt =
        localpath2Pt->extract_sub_localpath(robotPt,
                                            localpath2Pt, 0,
                                            l2 - lcur);
      start_localpath_q2qePt =
        localpath2Pt->extract_sub_localpath(robotPt, localpath2Pt,
                                            l2 - lcur,
                                            localpath2Pt->length_lp);

      /* update cost of second and third sections */
      init_cost[1] +=
        end_localpath_q1q2Pt->cost(robotPt, end_localpath_q1q2Pt);
      q2 = localpath2Pt->config_at_distance(robotPt, localpath2Pt,
                                            l2 - lcur);
      if (robotPt->cntrt_manager->cntrts != NULL) {
          p3d_set_and_update_this_robot_conf_multisol(robotPt, q2, NULL, 0, localpath1Pt->ikSol);
          p3d_get_robot_config_into(robotPt, &q2);
          p3d_set_robot_iksol(robotPt, robotPt->ikSol);
        }
    }
    lcur = lnext;
  }

  if(!p3d_compare_iksol(robotPt->cntrt_manager, localpath1Pt->ikSol, localpath2Pt->ikSol)){
    validIkSol[1] = 0;
  }
  /* cost of the last part of the initial path */
  init_cost[2] =
    start_localpath_q2qePt->cost(robotPt, start_localpath_q2qePt);

  localpath3Pt = localpath2Pt->next_lp;

  while (localpath3Pt != NULL) {
    init_cost[2] += localpath3Pt->cost(robotPt, localpath3Pt);
    if(localpath3Pt->next_lp == NULL){//last
      if(!p3d_compare_iksol(robotPt->cntrt_manager, localpath2Pt->ikSol, localpath3Pt->ikSol)){
        validIkSol[2] = 0;
      }
    }
    localpath3Pt = localpath3Pt->next_lp;
  }
  /* At this point:
     - end_localpath_qiq1Pt is a copy of the portion of the local path
     of the initial trajectory containing q1 up to q1
     - start_localpath_q1q2Pt is a copy of the portion of the local
     path of the initial trajectory containing q1 starting at q1

     - end_localpath_q1q2Pt is a copy of the portion of the local path
     of the initial trajectory containing q2 up to q2
     - start_localpath_q2qePt is a copy of the portion of the local
     path of the initial trajectory containing q2 starting at q2

     - localpath1Pt points toward the local path of the initial
     trajectory containing q1
     - localpath2Pt points toward the local path of the initial
     trajectory containing q2

     - init_cost[0], init_cost[1], init_cost[2] are the respective costs of the 3 portion
     of trajectory (qi,q1) (q1,q2) (q2,qe)
  */

  /* the local planner is called between qi and q1, q1 and q2, q2 and qe*/
  opt_pathPt[0] = p3d_local_planner(robotPt, qinit, q1);
  p3d_copy_iksol(robotPt->cntrt_manager, localpath1Pt->ikSol, &((opt_pathPt[0])->ikSol));

  opt_pathPt[1] = p3d_local_planner(robotPt, q1, q2);
  p3d_copy_iksol(robotPt->cntrt_manager, localpath1Pt->ikSol, &((opt_pathPt[1])->ikSol));
  opt_pathPt[2] = p3d_local_planner(robotPt, q2, qgoal);
  p3d_copy_iksol(robotPt->cntrt_manager, localpath2Pt->ikSol, &((opt_pathPt[2])->ikSol));

  p3d_destroy_config(robotPt, q1);
  p3d_destroy_config(robotPt, q2);
  p3d_destroy_config(robotPt, qinit);
  p3d_destroy_config(robotPt, qgoal);

  for (i = 0; i <= 2; i++) {
    if (opt_pathPt[i] == NULL) {
      opt_cost[i] = P3D_HUGE;
    } else {
      opt_cost[i] = opt_pathPt[i]->cost(robotPt, opt_pathPt[i]);
    }
  }

  /* collision checking is performed as late as possible */
  /* there are 8 possible trajectories: we compute the cost for each
     of them and store it in cost[i]. We then sort the trajectories
     by increasing cost. The cheapest one without collision is
     chosen */

  /* cost of each trajectory */
  /* loop over the 8 different trajectories */
  for (i = 0; i <= 7; i++) {
    opt[0] = i & 1;     /* whether first part replaced or not */
    opt[1] = i & 2;     /* whether second part replaced or not */
    opt[2] = i & 4;     /* whether third part replaced or not */
    cost[i] = 0;

    /* loop over the three portions of the path */
    for (j = 0; j <= 2; j++) {
      if (opt[j] != 0) {
        cost[i] += opt_cost[j];
      } else {
        cost[i] += init_cost[j];
      }
      cost_tmp[i] = cost[i];
    }
  }
  /* sort the different trajectories by cost */
  for (i = 0; i <= 7; i++) {
    cost_min = P3D_HUGE;
    for (j = 0; j <= 7; j++) {
      /* look for the smallest cost */
      if (cost_tmp[j] < cost_min) {
        cost_min = cost_tmp[j];
        jmin = j;
      }
    }
    sort[i] = jmin;
    cost_tmp[jmin] = 2 * P3D_HUGE;
  }

  i = 0;
  while (!path_found) {
    /* best path */
    j = sort[i];
    opt[0] = j & 1;
    opt[1] = j & 2;
    opt[2] = j & 4;

    /* collision checking of the new local paths involved */
    k = 0;
    collision[3] = FALSE;
    flag = 0;
    while ((k <= 2) && (collision[3] == FALSE)) {
      /* is the k-th portion of this trajectory the new local path ?*/
      if(!validIkSol[k]){
        flag = 1;
      }
      if (opt[k] != 0) {
        /* if yes, has collision already been tested for the new
           local path ?*/
        if (collision[k] == -1) {
          if (p3d_unvalid_localpath_test(robotPt, opt_pathPt[k], ntest) || flag == 1 ) {            // <- modif Xav
         // if (p3d_col_test_localpath(robotPt, opt_pathPt[k], ntest) || flag == 1 ) {
            collision[k] = 1;
          } else {
            collision[k] = 0;
          }
        }
        if (collision[k] == 1) {
          collision[3] = TRUE;
        }
      }
      k++;
      flag = 0;
    }
    if (collision[3] == FALSE) {
      /* all the new local paths in this trajectory are collision free */
      path_found = TRUE;
    } else {
      /* otherwise check next best trajectory */
      i++;
    }
  }
  /* The best trajectory without collision has been found. We need
     now to concatenate the different parts */
  /* PrintInfo(("  Path chosen: %d %d %d \n", opt[0], opt[1], opt[2])); */

  /* If the best trajectory is the initial one, just return the same */
  if (((opt[0] == 0) && (opt[1] == 0) && (opt[2] == 0))) {
    /* destroy the local paths allocated in this function */
    end_localpath_qiq1Pt->destroy(robotPt, end_localpath_qiq1Pt);
    end_localpath_qiq1Pt = NULL;
    start_localpath_q1q2Pt->destroy(robotPt, start_localpath_q1q2Pt);
    if (end_localpath_q1q2Pt != start_localpath_q1q2Pt) {
      end_localpath_q1q2Pt->destroy(robotPt, end_localpath_q1q2Pt);
    }
    start_localpath_q2qePt->destroy(robotPt, start_localpath_q2qePt);
    start_localpath_q1q2Pt = NULL;
    end_localpath_q1q2Pt = NULL;
    start_localpath_q2qePt = NULL;

    if (opt_pathPt[0] != NULL) {
      opt_pathPt[0]->destroy(robotPt, opt_pathPt[0]);
    }
    if (opt_pathPt[1] != NULL) {
      opt_pathPt[1]->destroy(robotPt, opt_pathPt[1]);
    }
    if (opt_pathPt[2] != NULL) {
      opt_pathPt[2]->destroy(robotPt, opt_pathPt[2]);
    }
    return FALSE;
  }

  /* first portion of the new path */
  if (opt[0] == 0) {
    /*destroy useless local path */
    if (opt_pathPt[0] != NULL) {
      opt_pathPt[0]->destroy(robotPt, opt_pathPt[0]);
      opt_pathPt[0] = NULL;
    }
    /* first portion not replaced */
    cur_localpathPt = trajPt->courbePt;
    while (cur_localpathPt != localpath1Pt) {
      if (start_new_trajPt == NULL) {
        start_new_trajPt = cur_localpathPt->copy(robotPt, cur_localpathPt);
        end_new_trajPt = start_new_trajPt;
      } else {
        end_new_trajPt =
          append_to_localpath(end_new_trajPt,
                              cur_localpathPt->copy(robotPt,
                                                    cur_localpathPt));
      }
      cur_localpathPt = cur_localpathPt->next_lp;
    }

    if (start_new_trajPt == NULL) {
      /* q1 is on the first local path */
      start_new_trajPt = end_localpath_qiq1Pt;
      end_new_trajPt = start_new_trajPt;
    } else {
      end_new_trajPt = append_to_localpath(end_new_trajPt,
                                           end_localpath_qiq1Pt);
    }
  } else {
    /* first local path replaced */
    start_new_trajPt = opt_pathPt[0];
    end_new_trajPt = start_new_trajPt;
    /* destroy useless local path */
    end_localpath_qiq1Pt->destroy(robotPt, end_localpath_qiq1Pt);
    end_localpath_qiq1Pt = NULL;
  }

  /* second portion of the new path */
  if (opt[1] == 0) {
    /* second local path not replaced */
    /* add copy of beginning of second part */
    end_new_trajPt = append_to_localpath(end_new_trajPt,
                                         start_localpath_q1q2Pt);

    /*destroy useless local path */
    if (opt_pathPt[1] != NULL) {
      opt_pathPt[1]->destroy(robotPt, opt_pathPt[1]);
      opt_pathPt[1] = NULL;
    }

    if (localpath2Pt != localpath1Pt) {
      /* q1 and q2 are not on the same local path */
      cur_localpathPt = localpath1Pt->next_lp;
      while (cur_localpathPt != localpath2Pt) {
        end_new_trajPt =
          append_to_localpath(end_new_trajPt,
                              cur_localpathPt->copy(robotPt,
                                                    cur_localpathPt));
        cur_localpathPt = cur_localpathPt->next_lp;
      }
      end_new_trajPt = append_to_localpath(end_new_trajPt,
                                           end_localpath_q1q2Pt);
    }
  } else {
    /* second local path replaced */
    end_new_trajPt = append_to_localpath(end_new_trajPt,
                                         opt_pathPt[1]);
    /* destroy useless local paths */
    start_localpath_q1q2Pt->destroy(robotPt, start_localpath_q1q2Pt);
    if (end_localpath_q1q2Pt != start_localpath_q1q2Pt) {
      end_localpath_q1q2Pt->destroy(robotPt, end_localpath_q1q2Pt);
    }
    start_localpath_q1q2Pt = NULL;
    end_localpath_q1q2Pt = NULL;
  }

  /* third portion of the new path */
  if (opt[2] == 0) {
    /* third local path not replaced */
    end_new_trajPt = append_to_localpath(end_new_trajPt,
                                         start_localpath_q2qePt);

    /*destroy useless local path */
    if (opt_pathPt[2] != NULL) {
      opt_pathPt[2]->destroy(robotPt, opt_pathPt[2]);
      opt_pathPt[2] = NULL;
    }

    cur_localpathPt = localpath2Pt->next_lp;
    while (cur_localpathPt != NULL) {
      end_new_trajPt =
        append_to_localpath(end_new_trajPt,
                            cur_localpathPt->copy(robotPt, cur_localpathPt));
      cur_localpathPt = cur_localpathPt->next_lp;
    }
  } else {
    /* second local path replaced */
    end_new_trajPt = append_to_localpath(end_new_trajPt, opt_pathPt[2]);

    /* destroy useless local paths */
    start_localpath_q2qePt->destroy(robotPt, start_localpath_q2qePt);
    start_localpath_q2qePt = NULL;
  }

  /* If two consecutive portions have not been replaced by the
     corresponding local paths, the local linking them has been cut
     in two pieces. We need to fix that a posteriori */



  /* destroy the initial trajectory and replace it by the new one */
  destroy_list_localpath(robotPt, trajPt->courbePt);
  /* replace it by the new one */
  trajPt->courbePt = start_new_trajPt;
  /* update the number of local paths */
  trajPt->nlp = p3d_compute_traj_nloc(trajPt);
  /* store the parameter range of this trajectory */
  trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);

  total_cost = init_cost[0] + init_cost[1] + init_cost[2];
  *gain = (total_cost - cost[j]) / total_cost;

  return TRUE;
}

/*
 *  p3d_simplify_traj
 *
 *  Simplify a path  by removing parts by which the robot passes several
 *  time. This situation occurs only for Reeds and Shepp paths.
 *
 *  Description:
 *     For each local path composing the path, a specific function is called.
 *     This function removes the intersection between the local path given
 *     as input and the next local path.
 *
 */

void p3d_simplify_traj(p3d_traj *trajPt) {
  p3d_rob *robotPt = trajPt->rob;
  p3d_localpath *localpathPt = trajPt->courbePt, *new_localpathPt = NULL,
                               *nextlpPt = NULL, *prevlpPt = NULL;
  int ntest = 0;

  int need_colcheck = FALSE, replace = FALSE;

  if (localpathPt == NULL)
    return;

  while (localpathPt->next_lp != NULL) {
    replace = FALSE;
    new_localpathPt =
      localpathPt->simplify(robotPt, localpathPt, &need_colcheck);

    /* replace old part by new one only if they are different */
    if (new_localpathPt != localpathPt) {
      if (need_colcheck) {
        /* replace the old part by the new one only if no collision */
        if (!p3d_col_test_traj(robotPt, new_localpathPt, &ntest)) {
          replace = TRUE;
        }
      } else {
        replace = TRUE;
      }
    }

    if (replace) {
      /* replace localpathPt and localpathPt->next_lp by new_localpathPt
      and new_localpathPt->next_lp */

      /* keep a pointer on the rest of the path before and after the
      part to remove */
      nextlpPt = localpathPt->next_lp->next_lp;
      prevlpPt = localpathPt->prev_lp;
      /* destroy first localpathPt->next_lp */
      localpathPt->next_lp->destroy(robotPt, localpathPt->next_lp);
      /* and then destroy localpathPt */
      localpathPt->destroy(robotPt, localpathPt);
      localpathPt = new_localpathPt;

      /* insert the new part into the path */
      if (prevlpPt != NULL) {
        /* we are not at the beginning of the path */
        prevlpPt->next_lp = new_localpathPt;
        new_localpathPt->prev_lp = prevlpPt;
      } else {
        trajPt->courbePt = new_localpathPt;
        new_localpathPt->prev_lp = NULL;
      }
      if (new_localpathPt->next_lp != NULL) {
        /* go to second local path of new part */
        new_localpathPt = new_localpathPt->next_lp;
      }
      new_localpathPt->next_lp = nextlpPt;

      if (nextlpPt != NULL) {
        /* we are not at the end of the path */
        nextlpPt->prev_lp = new_localpathPt;
      }
    }

    if (localpathPt->next_lp != NULL) {
      /* usually localpathPt->next_lp is not NULL, but
         p3d_simplify_manh() can delete "empty" localpaths
         in the trajectory and at the end it might happen
         that localpathPt->next_lp no longer points to a
         localpath after simplifying */
      localpathPt = localpathPt->next_lp;
    }
  }
  /* update the number of local paths */
  trajPt->nlp = p3d_compute_traj_nloc(trajPt);
  trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
}
