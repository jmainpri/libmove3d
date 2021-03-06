/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

#include "Graphic-pkg.h"

#define DEBUG_GRAPH_API 0

int singularityCheck = 0;
/****************************************************/
/* Fonction generant un noeud a partir d'une config */
/* aleatoirement admissible pour le robot           */
/* In : le graphe correspondant                     */
/* Out : le noeud                                   */
/****************************************************/

p3d_node* p3d_APInode_shoot(p3d_graph *graphPt) {
  p3d_rob *robotPt = graphPt->rob;
  configPt q;
  int ADM = 0, singularityCt = 0, speVal = 0;
  int *iksol = NULL;    // <- modif Juan

  q = p3d_alloc_config(robotPt);

  while (ADM == 0) {
    //if we have to shoot a singularity. No singularity shoot if we are is normal mode
    if(singularityCheck && p3d_get_ik_choice() != IK_NORMAL){
      p3d_APInode_shoot_singularity(graphPt->rob, &q, & speVal, &singularityCt, NULL, NULL);
    }else{
      p3d_APInode_shoot_normal(graphPt, &q, TRUE);
    }
    graphPt->nb_q_closed += 1;

    if (!p3d_col_test()) {
      ADM = 1;
      graphPt->nb_q_free += 1;
    }
    (graphPt->nb_test_coll)++;
    if(singularityCheck){
      p3d_unmark_for_singularity(robotPt->cntrt_manager, singularityCt);
    }
  }
  p3d_copy_iksol(robotPt->cntrt_manager, NULL, &iksol);
  if (robotPt->cntrt_manager->cntrts != NULL)
    p3d_get_robot_config_into(robotPt, &q);
  return(p3d_APInode_make_multisol(graphPt, q, iksol));  // <- modif Juan
}

static void p3d_mix_constraints_solutions(p3d_rob* robotPt, p3d_cntrt *ct,configPt * configs,
                                           int** iksol, int * niksol, int pos){
  int i = 0, j = 0, k = 0, h = 0, nbSol = 1, solNum = -1;
  configPt q;
  p3d_cntrt * tmp;

  if (!ct->next_cntrt){//is the last constraint
    if (!ct->markedForSingularity){
      for(i = 0; i < niksol[ct->num]; i++){//for all constraint solutions :
        q =  p3d_get_ikSpecific_config(robotPt->cntrt_manager, ct->num, i);
        solNum = p3d_get_ikSpecific_solution(robotPt->cntrt_manager, ct->num, i);
        iksol[pos+i][ct->num] = solNum;
        for(j = 0; j < ct->npasjnts; j++){
          for(k = 0; k < (ct->pasjnts)[k]->dof_equiv_nbr; k++){
            configs[pos+i][(ct->pasjnts)[j]->index_dof+k] = q[j];//copy the conserned values of Dof
          }
        }
      }
    }
  }else{
    if (ct->markedForSingularity){
      p3d_mix_constraints_solutions(robotPt, ct->next_cntrt, configs, iksol, niksol, pos);//Fill value of next constraint's passive joints
    }else{
      tmp = ct->next_cntrt;
      while(tmp){//get the number of solution for all the next constraints
        if(!tmp->markedForSingularity){
          nbSol *= niksol[tmp->num];
        }
          tmp = tmp->next_cntrt;
      }
      for(i = 0; i < niksol[ct->num]; i++, pos+=nbSol){
        q = p3d_get_ikSpecific_config(robotPt->cntrt_manager, ct->num, i);
        solNum = p3d_get_ikSpecific_solution(robotPt->cntrt_manager, ct->num, i);
        for(j = 0; j < nbSol; j++){
          iksol[pos+j][ct->num] = solNum;
          for(k = 0; k < ct->npasjnts; k++){
            for(h = 0; h < (ct->pasjnts)[k]->dof_equiv_nbr; h++){
              configs[pos+j][(ct->pasjnts)[k]->index_dof+h] = q[k];//copy the conserned values of Dof
            }
          }
        }
        p3d_mix_constraints_solutions(robotPt, ct->next_cntrt, configs, iksol, niksol, pos);//Fill value of next constraint's passive joints
      }
    }
  }
}

/**
 * @brief Genrate a set of nodes following the choosen strategy (Simple, Unique or Multi)
 * @param graphPt the current graph
 * @param nbNodes the number of nodes generated by this function
 * @return an array containing the generated nodes
 */
p3d_node** p3d_APInode_shoot_multisol(p3d_graph *graphPt, int* nbNodes) {
  p3d_rob *robotPt = graphPt->rob;
  int **ikSol = NULL, *nikSol = NULL, i = 0, j = 0, adm = 0, nbSolutions = 1, ikChoice = p3d_get_ik_choice(), singularityCt = -1, speVal = -1, nbMaxSol = 0;
  configPt * configs, q; //array of possible configurations
  p3d_node** N = NULL;

  N = MY_ALLOC(p3d_node*, 10);//we create 10 nodes

  if (ikChoice != IK_NORMAL){
    //if there is at least one constraint
    if (robotPt->cntrt_manager->cntrts != NULL) {
      q = p3d_alloc_config(robotPt);
      if (p3d_is_multisol(robotPt->cntrt_manager, &nbMaxSol)){//if there is a constraint with multi solutions
        while (adm == 0) {
          p3d_reset_iksol(robotPt->cntrt_manager);
          if(singularityCheck){
            p3d_APInode_shoot_singularity(graphPt->rob, &q, &speVal, &singularityCt, NULL, NULL);
          }else{
            p3d_APInode_shoot_normal(graphPt, &q, TRUE);
          }
          //Compute the number of solutions
          if (ikChoice == IK_MULTISOL){
            nikSol = p3d_get_niksol_vector(robotPt->cntrt_manager);//take the number of solution for each constraint
            if (DEBUG_GRAPH_API){
              PrintInfo(("nikSol :\n"));
              for(i = 0; i < robotPt->cntrt_manager->ncntrts; i++){
                PrintInfo(("niksol[%d] = %d\n", i, nikSol[i]));
              }
              PrintInfo(("\n"));
            }
//             nbSolutions = p3d_get_nb_ikSol(robotPt->cntrt_manager);
            for (i = 0, nbSolutions = 1; i < robotPt->cntrt_manager->ncntrts; i++){//compute the number of solutions.
              if((robotPt->cntrt_manager->cntrts[i])->markedForSingularity != 1){
              nbSolutions *= nikSol[i];
              }
            }
          }else{//if IK_UNIQUE
            nbSolutions = 1;
          }
          configs = MY_ALLOC(configPt, nbSolutions);//allocation of configuration array
          ikSol = MY_ALLOC(int*, nbSolutions);//allocation of solution number array
          for(i = 0; i < nbSolutions; i++){
            configs[i] = p3d_copy_config(robotPt,q);
            ikSol[i] = MY_ALLOC(int, robotPt->cntrt_manager->ncntrts);
            for(j = 0; j < robotPt->cntrt_manager->ncntrts; j++){
              ikSol[i][j] = - speVal - 1;
            }
          }
          if(ikChoice == IK_MULTISOL){
            p3d_mix_constraints_solutions(robotPt, robotPt->cntrt_manager->cntrts[0], configs, ikSol, nikSol, 0);
          }else{//IK_UNIQUE
            p3d_copy_iksol(robotPt->cntrt_manager, NULL, &ikSol[0]);
            p3d_get_robot_config_into(robotPt, &configs[0]);
          }
          for(i = 0; DEBUG_GRAPH_API && i < nbSolutions; i++){
            PrintInfo(("configuration %d :\n", i));
            PrintInfo(("Solutions: \n"));
            for(j = 0; j < robotPt->cntrt_manager->ncntrts; j++){
              PrintInfo(("ikSol[%d] = %d \n", j, ikSol[i][j]));
            }
            PrintInfo(("\n"));
            print_config(robotPt, configs[i]);
            PrintInfo(("\n"));
          }
          for(i = 0; i < nbSolutions; i++){
            p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,configs[i]);//we don't have to use constraint because the configurations given take them into account
            graphPt->nb_q_closed += 1;
            if(!p3d_col_test()){//if there is no collision, create the node and count the number of valid nodes.
              adm++;//number of valid nodes
              graphPt->nb_q_free += 1;
              if(adm > 1 && adm % 10 == 0)//Realoc each 10 nodes
                N = MY_REALLOC(N, p3d_node*, adm, adm + 10);
              N[adm-1] = p3d_APInode_make_multisol(graphPt, configs[i], ikSol[i]);
            }else{
              p3d_destroy_config(robotPt,configs[i]);
            }
            (graphPt->nb_test_coll)++;
          }
          MY_FREE(configs, configPt, nbSolutions);
          for(i = 0; i < nbSolutions; i++){
            MY_FREE(ikSol[i], int, robotPt->cntrt_manager->ncntrts);
          }
          MY_FREE(ikSol, int*, nbSolutions);
          if(singularityCheck){
            p3d_unmark_for_singularity(robotPt->cntrt_manager, singularityCt);
          }
        }
        *nbNodes = adm;
        p3d_destroy_config(robotPt,q);
        return N;
      }else{//there is only unique solution constraints
        N[0] = p3d_APInode_shoot(graphPt);
        p3d_destroy_config(robotPt,q);
        *nbNodes = 1;
        return N;
      }
    }else{//do a simple shoot
      N[0] = p3d_APInode_shoot(graphPt);
      *nbNodes = 1;
      return N;
    }
  }else{

    N[0] = p3d_APInode_shoot(graphPt);
    *nbNodes = 1;
    return N;
  }
}


/**
  * @brief Shoot a non singular configuration
  * @param graphPt the current graph
  * @param q the sampled configuration
  */

void p3d_APInode_shoot_normal(p3d_graph *graphPt, configPt* q, int shootPassive){
  int i = 0;
  do {
    p3d_shoot(graphPt->rob, *q, shootPassive);
    i++;
  } while (!p3d_set_and_update_this_robot_conf_with_partial_reshoot(graphPt->rob, *q));//shoot until we have a valid configuration
  p3d_get_robot_config_into(graphPt->rob, q);
  if (DEBUG_GRAPH_API){
    PrintInfo(("configuration tirée :\n"));
    print_config(graphPt->rob, *q);
    PrintInfo(("\n"));
  }
}

/**
 * @brief select randomly one constraint having a singularity and generates nodes corresponding to this configuration
 * @param graphPt the current graph
 * @param q the sampled configuration
 * @param singNum the choosen singularity
 * @param cntrtNum the choosen constraint
 * @param rootConfig a config form whitch the singular configuration will be built. To be set to null if random shoot wanted
 * @param rootIkSol the inverse kinematic solutions for the given rootConfig. To be set to null if random shoot wanted
 * @return 0 on error 1 on success
 */
int p3d_APInode_shoot_singularity(p3d_rob *rob, configPt* q, int *singNum, int *cntrtNum, configPt rootConfig, int * rootIkSol){
  p3d_cntrt_management * cntrt_manager = rob->cntrt_manager;

  if(cntrt_manager->ncntrts == 0){
    printf("No Singular value defined\n");
    p3d_SetStopValue(TRUE);
    return 0;
  }
  
  if (*cntrtNum == -1) {
    do{//take a random constraint having a singularity.
      *cntrtNum = (int)p3d_random(0, cntrt_manager->ncntrts-EPS6);//the constraint to shoot as singularity
    }while(cntrt_manager->cntrts[*cntrtNum]->nSingularities == 0); 
  }
  p3d_mark_for_singularity(cntrt_manager,*cntrtNum);
  int *ikSol = MY_ALLOC(int, rob->cntrt_manager->ncntrts);
  int result = 0;
  int nbMaxShoots = 100, nbShoots = 0;
  do {
    if (rootConfig) {
//            g3d_draw_allwin_active();
      //p3d_copy_config_into(rob, rootConfig, q);
      double robotSize = 0, translationFactor = 0, rotationFactor = 0;
      p3d_get_BB_rob_max_size(rob, &robotSize);
      translationFactor = robotSize/10;
      rotationFactor = robotSize/30;
      
//      p3d_set_and_update_this_robot_conf_without_cntrt(rob, rootConfig);
//      g3d_draw_allwin_active();
//      print_config(rob, rootConfig);
      p3d_gaussian_config2_specific(rob, rootConfig, *q, translationFactor, rotationFactor, true);
//      p3d_set_and_update_this_robot_conf_without_cntrt(rob, *q);
//      print_config(rob, *q);
//      g3d_draw_allwin_active();
      
      p3d_copy_iksol(rob->cntrt_manager, rootIkSol, &ikSol);
      //rootConfig = NULL; //if the singular config obtained from root Config does not satisfy the constraints sample random
      if (DEBUG_GRAPH_API){
        printf("User Singular Config\n");
      }
    }else {
      p3d_shoot(rob, *q, 1);
      for (int i = 0; i < rob->cntrt_manager->ncntrts; i++) {
        ikSol[i] = p3d_get_random_ikSol(rob->cntrt_manager, i);
      }
    }
    p3d_set_robot_config(rob, *q);
    result = p3d_set_robot_singularity(rob, *cntrtNum, singNum);
    p3d_get_robot_config_into(rob, q);
    nbShoots++;
 //         g3d_draw_allwin_active();
  } while ((!result || (!p3d_set_and_update_this_robot_conf_multisol(rob, *q, NULL, 0, ikSol))) && (nbShoots < nbMaxShoots));//shoot until we have a valid configuration
  p3d_set_iksol_elem(*cntrtNum, -(*singNum) - 1);
  MY_FREE(ikSol, int, rob->cntrt_manager->ncntrts);
  if (DEBUG_GRAPH_API){
    PrintInfo(("configuration tirée :\n"));
    print_config(rob, *q);
    PrintInfo(("\n"));
    p3d_print_iksol(cntrt_manager, NULL);
  }
  return 1;
}

p3d_node* p3d_APInode_shoot_nocolltest(p3d_graph *graphPt) {
    p3d_rob *robotPt = graphPt->rob;
  configPt q;
  int ADM = 0, singularityCt = 0, speVal = 0;
  int *iksol = NULL;    // <- modif Juan

  q = p3d_alloc_config(robotPt);

  while (ADM == 0) {
    //if we have to shoot a singularity. No singularity shoot if we are is normal mode
    if(singularityCheck && p3d_get_ik_choice() != IK_NORMAL){
      p3d_APInode_shoot_singularity(graphPt->rob, &q, & speVal, &singularityCt, NULL, NULL);
    }else{
      p3d_APInode_shoot_normal(graphPt, &q, TRUE);
    }

    graphPt->nb_q_closed += 1;

    if(singularityCheck){
      p3d_unmark_for_singularity(robotPt->cntrt_manager, singularityCt);
    }
  }
  p3d_copy_iksol(robotPt->cntrt_manager, NULL, &iksol);
  if (robotPt->cntrt_manager->cntrts != NULL)
    p3d_get_robot_config_into(robotPt, &q);
//     p3d_get_iksol_vector(robotPt->cntrt_manager, iksol);
  // WARNING: iksol does not work, a new vector containing the 1st solution of each constraint must be passed as argument
  return(p3d_APInode_make_multisol(graphPt, q, NULL));
}


/*****************************************************/
/* Compute and set the frame associated with a node  */
/* used for simplified distance metrics              */
/* In : the graph, the node                          */
/* Out :                                             */
/******************************************************/

/****************************************************/
// REFERENCE MOBILE FRAME
// ----------------------
// NOTE : TEMPORARY IMPLEMENTATION :
//        it should be one mob_frame_0 per connected component !!!

/* static p3d_matrix4 st_mob_frame_0 = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}; */

/* void p3d_set_mob_frame_0(p3d_matrix4 mf0) */
/* { */
/*   p3d_mat4Copy(mf0,st_mob_frame_0);  */
/* } */

/* void p3d_get_mob_frame_0(p3d_matrix4 **mf0Pt) */
/* { */
/*   *mf0Pt = &st_mob_frame_0;  */
/* } */




/*******************************************/
/* Fonction creant   un noeud a un graphe  */
/* a partir d'une configuration            */
/* In : le graphe, la configuration,       */
/* Out : le noeud                          */
/*******************************************/
p3d_node* p3d_APInode_make(p3d_graph *graphPt, configPt q) {
  p3d_node *nodePt;

  nodePt = p3d_create_node(graphPt);
  nodePt->q = q;
  //p3d_set_node_rel_mob_frame(graphPt,nodePt);

  /*p3d_SetMobFrameToNode replace the
    p3d_set_node_rel_mob_frame function
   */

  // WARNING : suppose that q has been set and updated
  p3d_SetMobFrameToNode(graphPt,nodePt);
  return(nodePt);
}


/*******************************************/
/* Fonction creant   un noeud a un graphe  */
/* a partir d'une configuration            */
/* In : le graphe, la configuration,       */
/*      le vector iksol                    */
/* Out : le noeud                          */
/*******************************************/
p3d_node* p3d_APInode_make_multisol(p3d_graph *graphPt, configPt q, int *iksol) {
  p3d_node *nodePt;

  nodePt = p3d_create_node(graphPt);
  nodePt->q = q;
  if(iksol){
    p3d_copy_iksol(graphPt->rob->cntrt_manager,iksol,&(nodePt->iksol));
  }

  // p3d_set_node_rel_mob_frame(graphPt,nodePt);
  /*p3d_SetMobFrameToNode replace the
    p3d_set_node_rel_mob_frame function
   */

  // WARNING : suppose that q has been set and updated
  p3d_SetMobFrameToNode(graphPt,nodePt);
  if(singularityCheck){
    nodePt->isSingularity = TRUE;
  }
//   if(p3d_get_costComputation()){
//     nodePt->cost = p3d_GetHriDistCost(graphPt->rob, 1);
//   }
  return(nodePt);
}
// fmodif Juan

/*********************************************/
/* Fonction de desallocation de la structure */
/* node (non insere dans le graphe!)         */
/* In : le noeud                             */
/* Out :                                     */
/*******************************************/
void
p3d_APInode_desalloc(p3d_graph *graphPt, p3d_node *nodePt) {
  p3d_rob * robotPt = graphPt ? graphPt->rob : XYZ_ROBOT;

  if (nodePt->q) {
	p3d_destroy_config(robotPt, nodePt->q);
  }
  if (nodePt->list_closed_flex_sc != NULL) {
    free(nodePt->list_closed_flex_sc);
  }
  if(nodePt->iksol){
    p3d_destroy_specific_iksol(robotPt->cntrt_manager, nodePt->iksol);
    nodePt->iksol = NULL;
  }
  MY_FREE(nodePt, p3d_node, 1);
}


/**********************************************************
 * Function testant la connection entre deux noeuds
 * (remplace la fonction link())
 * Input:  the graph, the two nodes.
 * Output: whether connected or not, distance between nodes.
 ***********************************************************/

int
p3d_APInode_linked(p3d_graph *graphPt, p3d_node *N1,  p3d_node *N2, double *dist) {
  p3d_rob *robotPt = NULL;
  p3d_localpath *localpathPt;
  int ntest = 0, isNoCol = 0, *ikSol = NULL;
  configPt qsave;

  if(graphPt){
   robotPt = graphPt->rob;
  }else{
    robotPt = XYZ_ROBOT;
  }

  /* current position of robot is saved */
  qsave = p3d_get_robot_config(robotPt);

  /* compute the local path using the local method associated to
     the robot */
  if (DEBUG_GRAPH_API){
    printf("API Node Linked :\n");
    p3d_print_iksol(robotPt->cntrt_manager,N1->iksol);
    p3d_print_iksol(robotPt->cntrt_manager,N2->iksol);
  }
  if(!p3d_compare_iksol(robotPt->cntrt_manager, N1->iksol, N2->iksol)){
    p3d_destroy_config(robotPt, qsave);
    return(FALSE);
  }
  p3d_get_non_sing_iksol(robotPt->cntrt_manager, N1->iksol, N2->iksol, &ikSol);
  localpathPt = p3d_local_planner_multisol(robotPt, N1->q, N2->q, ikSol);

  if (localpathPt == NULL) { // Not valid localpath
    p3d_destroy_config(robotPt, qsave);
    return(FALSE);
  }


  if (localpathPt->length != NULL)
      *dist = localpathPt->length(robotPt,localpathPt);
  else{
      PrintInfo(("Warning: created an edge with \
a 0 distance: no localpathPt->length \n"));
      *dist = 0;
       }
  if((p3d_get_SORTING()==P3D_NB_CONNECT)&&
     (p3d_get_MOTION_PLANNER()==P3D_BASIC)) {
    if((*dist > p3d_get_DMAX())&&(LEQ(0.,p3d_get_DMAX()))){ /* ecremage deja fait dans le cas tri par distance... */
      /* the local path is destroyed */
      localpathPt->destroy(robotPt, localpathPt);
      localpathPt = NULL;

      /* The initial position of the robot is recovered */
      p3d_set_robot_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      return(FALSE);
    }
  }
  //start path deform
  if (p3d_get_cycles() == TRUE) {
    if (localpathPt->length != NULL)
      *dist = localpathPt->length(robotPt, localpathPt);
    else {
      PrintInfo(("linked: no distance function specified\n"));
      *dist = 0;
    }
  }
  //end path deform
  isNoCol = !p3d_unvalid_localpath_test(robotPt, localpathPt, &ntest);   // <- modif Juan
//   isNoCol = 1;
  localpathPt->destroy(robotPt, localpathPt);
  if(graphPt){
    graphPt->nb_local_call = graphPt->nb_local_call + 1;
    graphPt->nb_test_coll = graphPt->nb_test_coll + ntest;
  }


  /* The initial position of the robot is recovered */
  p3d_set_robot_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  return(isNoCol);
}

/**
 * @brief Check if two node are connectable
 * @param graphPt the current graph
 * @param N1 first node
 * @param N2 second node
 * @param dist the distance between the nodes
 * @return TRUE if connected FALSE otherwise
 */
int
p3d_APInode_linked_multisol(p3d_graph *graphPt, p3d_node *N1,  p3d_node *N2, double *dist) {
//   if the two nodes are in the same solution class && if singularity test the singClass
  p3d_rob * robotPt = graphPt ? graphPt->rob : XYZ_ROBOT;
  if (p3d_compare_iksol(robotPt->cntrt_manager, N1->iksol, N2->iksol)){
    if (N1->isSingularity || N2->isSingularity){
      if (!p3d_test_singularity_connexion(robotPt->cntrt_manager, N1, N2)){
        return 0;
      }
    }
    return p3d_APInode_linked(graphPt,N1,N2,dist);
  }
  return 0;
}

/*******************************************/
/* Fonction qui retourne la distance entre */
/* 2 noeuds                                */
/*******************************************/
double
p3d_APInode_dist(p3d_graph *graphPt, p3d_node *N1, p3d_node *N2) {
  return(p3d_dist_q1_q2(graphPt->rob, N1->q, N2->q));
}

/*******************************************/
/* Fonction qui retourne la distance entre */
/* 2 noeuds                                */
/*******************************************/
/**
 * @brief compute the distance between two nodes if its are on the same layer
 * @param graphPt the current graph
 * @param N1 the first node
 * @param N2 the second node
 * @return The distance betwwen the nodes
 */
double
p3d_APInode_dist_multisol(p3d_graph *graphPt, p3d_node *N1, p3d_node *N2) {
  int *ikSol = NULL;
  if (p3d_compare_iksol(graphPt->rob->cntrt_manager, N1->iksol, N2->iksol)){//if the two nodes are in the same solution class
    p3d_get_non_sing_iksol(graphPt->rob->cntrt_manager, N1->iksol, N2->iksol, &ikSol);
//     p3d_copy_iksol(graphPt->rob->cntrt_manager, N1->iksol, &ikSol);
    return(p3d_dist_q1_q2_multisol(graphPt->rob, N1->q, N2->q, ikSol));
  }
  return P3D_HUGE;
}


/**********************************************************
 * Function d'expansion locale d'un noeud du graphe.
 * Utilise le DQ de la fonction p3d_get_EXPAND_DQ()
 * pour limiter l'echantillonage aleatoire dans une boite
 * de taille N->q += DQ avant d'appeler la fonction p3d_learn
 * Input:  le noeud et le graphe.
 * Output: TRUE le noeud a ete expanse.
 ***********************************************************/

int p3d_APInode_expand(p3d_graph *graphPt, p3d_node *N, int (*fct_stop)(void),
                       void (*fct_draw)(void)) {
  p3d_rob *robotPt;
  p3d_jnt *jntPt;
  configPt qrandmin, qrandmax, q;
  int njnt, i, j, k;
  double *dq;

  robotPt = graphPt->rob;
  q = N->q;
  njnt = robotPt->njoints;

  dq = p3d_get_EXPAND_DQ();
  if (!p3d_get_EXPAND_OPTION())
    return(FALSE);
  if (dq == NULL)
    return(FALSE);

  PrintInfo(("Debut d'expansion...\n"));

  switch (p3d_get_EXPAND_OPTION()) {
    case P3D_EXPAND_BOX:
      qrandmin = p3d_alloc_config(robotPt);
      qrandmax = p3d_alloc_config(robotPt);

      for (i = 0; i <= njnt; i++) {
        jntPt = robotPt->joints[i];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          k = jntPt->index_dof + j;
          if (p3d_jnt_get_dof_is_user(jntPt, j)) {
            p3d_jnt_get_dof_rand_bounds(jntPt, j, &qrandmin[k], &qrandmax[k]);
            p3d_jnt_set_dof_rand_bounds(jntPt, j, q[k] - dq[k], q[k] + dq[k]);
          }
        }
      }
      p3d_learn(p3d_get_EXPAND_NB_NODES(), fct_stop, fct_draw);

      for (i = 0; i <= njnt; i++) {
        jntPt = robotPt->joints[i];
        for (j = 0; j < jntPt->dof_equiv_nbr; j++) {
          if (p3d_jnt_get_dof_is_user(jntPt, j)) {
            k = jntPt->index_dof + j;
            p3d_jnt_set_dof_rand_bounds(jntPt, j, qrandmin[k], qrandmax[k]);
          }
        }
      }

      p3d_destroy_config(robotPt, qrandmin);
      p3d_destroy_config(robotPt, qrandmax);
      return(TRUE);

    case P3D_EXPAND_RAND_WALK:
      PrintWarning(("Expand Random Walk: not anymore implemented\n"));
      return(TRUE);

    default:
      PrintInfo(("WRONG TYPE OF EXPAND\n"));
  }
  return(FALSE);
}


/*************************************************************
 * reads path in graph and constructs a trajectory
 * Input  : a robot
 *          (everything is in the structure of its XYZ_GRAPH
 * Output : the trajectory
 * SIDE EFFECT  :   new trajectory is "current trajectory"
 * PRECONDITION :   one must have called p3d_graph_search
 **************************************************************/

static  p3d_list_node * addnode_before_list(p3d_node *nodePt, p3d_list_node *listPt);

int GlobalOrdering = FALSE;
p3d_traj *p3d_graph_to_traj(p3d_rob *robotPt) {

  configPt q0, q;
  int iloc = 0, nloc = 0, ntrj = 0, *ikSol = NULL;
  p3d_node *bckw_path;
  p3d_list_node *path = NULL, *last_node, *thisnode, *nextnode, *destr_node;
  char      str[250], sti[250];
  p3d_localpath *localpathPt = NULL;
  p3d_traj *t;
  //ptr_to_localplanner localplanner; /* pointer to local planner function */
  p3d_graph *xyz_graph = NULL;


  /* The list of nodes and edges between the start and goal configurations
  is built */
  xyz_graph = robotPt->GRAPH;

  // search traj in graph
  if(GlobalOrdering == TRUE) {
    if (p3d_OrderingGraphSearch(xyz_graph, p3d_heurist,
                                p3d_valid, p3d_end, DEFAULTGRAPH) == FALSE) return NULL;
  } else {
    if (p3d_graph_search(xyz_graph, p3d_heurist,
                          p3d_valid, p3d_end, DEFAULTGRAPH) == FALSE) return NULL;
  }

  bckw_path = xyz_graph->search_goal;
  last_node = NULL;

  while (bckw_path && bckw_path != xyz_graph->search_start) {
    path = addnode_before_list(bckw_path, last_node);
    last_node = path;
    bckw_path = bckw_path->search_from;
    iloc = iloc + 1;
  }
  if(!bckw_path){
    printf("%s: %d: p3d_graph_to_traj: Error in graph search. bckw_path = NULL\n", __FILE__, __LINE__);
    return NULL;
  }

  path = addnode_before_list(xyz_graph->search_start,
                             last_node);
  nloc = iloc;
  thisnode = path;
  strcpy(str, "globtrj.");
  sprintf(sti, "%s", robotPt->name);
  strcat(str, sti);
  sprintf(sti, "%s", ".");
  strcat(str, sti);
  /* ntrj = p3d_get_desc_number(P3D_TRAJ)+1; */
  ntrj = robotPt->nt;
  sprintf(sti, "%d", ntrj);
  strcat(str, sti);
  p3d_beg_desc(P3D_TRAJ, str);

  for (iloc = 1;iloc <= nloc;iloc++) {
    q0 = thisnode->N->q;
    nextnode = thisnode->next;
    q = nextnode->N->q;
    /* the local path between a node and its next node is reconstructed
    using the local planner stored in the edge linking these nodes */
    //localplanner = array_localplanner[nextnode->N->edge_from->planner];
    if (!p3d_equal_config(robotPt, q0, q)) {
      //Multisol/singularity gestion
      p3d_get_non_sing_iksol(robotPt->cntrt_manager, thisnode->N->iksol, nextnode->N->iksol, &ikSol);
      //localpathPt = localplanner(robotPt, q0, q, ikSol);
			localpathPt = p3d_local_planner_multisol(robotPt, q0, q, ikSol); // modif XB
      if (localpathPt != NULL)
        p3d_add_desc_courbe(localpathPt);
    }
    thisnode = thisnode->next;
  }
  p3d_end_desc();
  t = robotPt->tcur;

#ifdef DPG
  if (t->nlp) {
    t->savelpNum = t->nlp;
    t->trajInGraph = t->courbePt->copy(robotPt, t->courbePt);
    p3d_localpath* saveLp = t->trajInGraph;
    for(p3d_localpath* courbeLp = t->courbePt->next_lp; courbeLp; courbeLp = courbeLp->next_lp, saveLp = saveLp->next_lp){
      saveLp->next_lp = courbeLp->copy(robotPt, courbeLp);
      saveLp->next_lp->prev_lp = saveLp;
    }
  }
#endif

  thisnode = path;
  for (iloc = 1;iloc <= nloc + 1;iloc++) {
    destr_node = thisnode;
    thisnode = thisnode->next;
    MY_FREE(destr_node, p3d_list_node, 1);
  }
  return t;
}

/*
 *  Add a node at the beginning of a list
 *  Input  : the node, the list
 *  Output : the new list
 */

static
p3d_list_node * addnode_before_list(p3d_node *nodePt,  p3d_list_node *listPt) {
  p3d_list_node *new_listPt = NULL;

  new_listPt = MY_ALLOC(p3d_list_node, 1);

  new_listPt->N = nodePt;
  new_listPt->next = listPt;
  new_listPt->prev = NULL;

  if (listPt != NULL) {
    listPt->prev = new_listPt;
  }

  return new_listPt;
}

static p3d_node* p3d_addConfToGraph(p3d_rob* robot, p3d_graph* graph, configPt q, int* ikSol){
  p3d_node* node = p3d_TestConfInGraph(graph, q);
  if(!node){
    node = p3d_APInode_make_multisol(graph, q, ikSol);
    p3d_insert_node(graph, node);
  }
  return node;
}

void p3dAddTrajToGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj){
  p3d_node* initNode = NULL, *endNode = NULL;
  configPt qInit, qEnd;
  bool nodeAlreadyConnected = false;
  for(p3d_localpath* lp = traj->courbePt; lp; lp = lp->next_lp){
    qInit = lp->config_at_param(robot, lp, 0);
    // take into account the constraint.
    p3d_set_and_update_this_robot_conf_multisol(robot, qInit, NULL, 0, lp->ikSol);
    p3d_destroy_config(robot, qInit);
    qInit = p3d_get_robot_config(robot);
    qEnd = lp->config_at_param(robot, lp, lp->length_lp);
    // take into account the constraint.
    p3d_set_and_update_this_robot_conf_multisol(robot, qEnd, NULL, 0, lp->ikSol);
    p3d_destroy_config(robot, qEnd);
    qEnd = p3d_get_robot_config(robot);
    nodeAlreadyConnected = false;
    initNode = NULL;
    endNode = NULL;
    initNode = p3d_TestConfInGraph(graph, qInit);
    if(!initNode){
      printf("QInit n'est pas dans le graph\n");//If qinit is not already in the graph, there is a problem !!!
      return;
    }
    endNode = p3d_TestConfInGraph(graph, qEnd);
    if(!endNode){
      endNode = p3d_addConfToGraph(robot, graph, qEnd, lp->ikSol);
    }
    //connect qInit and qEnd if its not connected yet
    for(p3d_list_edge* lEdge = initNode->edges; lEdge; lEdge = lEdge->next){
      if(lEdge->E->Nf == endNode){
        nodeAlreadyConnected = true;
        break;
      }
    }
    if(!nodeAlreadyConnected){
      p3d_add_node_compco(endNode, initNode->comp, TRUE);
      double dist = p3d_dist_q1_q2_multisol(robot, qInit, qEnd, lp->ikSol);//take the distance between the two nodes
      p3d_create_edges(graph, initNode, endNode, dist);//create edges between the two nodes
    }
  }
}
