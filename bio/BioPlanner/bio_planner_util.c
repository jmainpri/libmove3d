/* This file should  functions  useful
   for specific bio_planning methods */

#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"
#ifdef ENERGY
#include "../bio/BioEnergy/include/Energy-pkg.h"
#endif


int MAX_NNODE_FAILS = 10;

int bio_restrict_sc_shoot(p3d_rob* p3d_robotPt, p3d_node* Nnear, configPt q);
int bio_restrict_sc_shoot_with_NnearInfo(p3d_rob* p3d_robotPt, p3d_node* Nnear, configPt q);
int bio_side_chain_shoot(p3d_rob *robotPt, configPt q);
int bio_ligand_shoot(p3d_rob* robotPt, configPt q);
double bio_compute_ligand_dist(p3d_rob *robotPt, configPt q_i, configPt q_f);
p3d_node *bio_ligand_nearest_neighbor(p3d_rob *robotPt, configPt q, p3d_compco *comp, double* dist);
int bio_get_max_nnode_fail(void);
void bio_set_max_nnode_fail(int nfail);
void bio_free_multimol_collision_report(int_tab** sc_autocol_list, int_tab** scsc_col_list, 
					int_tab** sc_bb_col_list, int_tab** sc_ligand_col_list);
int bio_multimol_collision_report(p3d_rob *robotPt, configPt q, int_tab** sc_autocol_list,
				  int_tab** scsc_col_list , int_tab** sc_bb_col_list, int_tab** sc_ligand_col_list,
					 int* is_ligand_bb_col, int* is_ligand_autocol, int* is_bb_autocol);
void bio_free_monomol_collision_report(int_tab** sc_autocol_list, int_tab** scsc_col_list, 
				       int_tab** sc_bb_col_list);
int bio_monomol_collision_report(p3d_rob *robotPt, configPt q, int_tab** sc_autocol_list,
				  int_tab** scsc_col_list , int_tab** sc_bb_col_list, int* is_bb_autocol) ;
				       
/************************************************************/
/* \brief: function to generate random values for the       */
/*         sidechain dofs, such that their BBoxes do not    */
/*         overlap the ligand BBox  in the current config.  */
/*          The other dofs of q are kept unchanged          */
/* \param robotPt: the robot                                */
/* \param q: only the sidechains dofs for sidechain far from*/
/*        the lignad config are modified                    */
/* \param Nnear: the nearest node which make the shoot      */
/*               restrictif. Needed to store infos in the   */
/*               node structure. If Nnear is NULL, the shoot is*/
/*               performed without storing anything         */
/* \return: 1 if the shoot succeeded, 0 otherwise           */
/************************************************************/
int bio_restrict_sc_shoot(p3d_rob* robotPt, p3d_node* Nnear, configPt q) {
  p3d_jnt** firstjnts_flexible_sc = get_list_firstjnts_flexible_sc(robotPt);
  p3d_jnt* firstjnt_flexsc = firstjnts_flexible_sc[0];
  int i=0,result_test = -1;
  int AAnumber=-1;
  configPt q_sc_shoot =NULL;
  double dist_vector[3];
  int nb_farAA =0;
  int nb_flexible_jnts;

  if(Nnear != NULL){
      nb_flexible_jnts =  get_nb_flexible_sc(robotPt);
      // assert(Nnear->list_closed_flex_sc==NULL);
      Nnear->list_closed_flex_sc = MY_ALLOC(int, nb_flexible_jnts);
  }
  while(firstjnt_flexsc!= NULL) {
    AAnumber = get_AAnumber_from_jnt(firstjnt_flexsc);
    // if the firstjnt_flexsc is .chain_base then  AAnumber = -1.
    // in that case we leave the chain flexible (actually it should be tested...)
    if( (AAnumber!= -1 ) && (dist_ligand_sidechainBBoxes(robotPt, AAnumber, dist_vector)==1)) {
      //the boxes are not overlapping
      if(Nnear != NULL) {
	Nnear->list_closed_flex_sc[i]=0;
      }
      result_test = make_rigid_AA_num(AAnumber);
      nb_farAA++;
      if(result_test == 0) {
	PrintInfo(("Warning: restrictive shoot failed "
		   "because of the make_rigid_AA process\n"));
	return 0;
      }
    }
    else {
          if(Nnear != NULL) {
       	Nnear->list_closed_flex_sc[i]=1;
        }
     }
    i++;
    //   PrintInfo(("Print1: num 1st comp: %d, i: %d\n",XYZ_GRAPH->comp->num,i));
    firstjnt_flexsc =   firstjnts_flexible_sc[i];
  }
  //PrintInfo(("%d far flexible sc AA, on a total of %d\n",nb_farAA++, i));
  q_sc_shoot = p3d_alloc_config(robotPt);
 if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
   bio_side_chain_inbox_shoot(robotPt, Nnear->comp->box_env_small, q_sc_shoot);
 }
 else {
   bio_side_chain_shoot(robotPt, q_sc_shoot);
}
  bio_copy_sch_conf(robotPt,q_sc_shoot,q);
  p3d_destroy_config(robotPt,q_sc_shoot);


  //we set again all  flexible joints as flexible
  firstjnt_flexsc = firstjnts_flexible_sc[0];
  i =0;
  while(firstjnt_flexsc!= NULL) {
    AAnumber = get_AAnumber_from_jnt(firstjnt_flexsc);
    if (AAnumber != -1) {
      result_test = make_flexible_AA_num(AAnumber); 
      if(result_test == 0) {
	PrintInfo(("Warning: restrictive shoot failed "
		   "because of the make_flexible_AA process\n"));
	return 0;
      }
    }
    i++;
    //PrintInfo(("Print2: num 1st comp: %d, i: %d\n",XYZ_GRAPH->comp->num,i));
    firstjnt_flexsc =   firstjnts_flexible_sc[i];
  }
  return 1;
}

/************************************************************/
/* \brief: function to generate random values for the       */
/*         sidechain dofs, such that their BBoxes do not    */
/*         overlap the ligand BBox  in the current config.  */
/*          The other dofs of q are kept unchanged          */
/* \param robotPt: the robot                                */
/* \param q: only the sidechains dofs for sidechain far from*/
/*        the lignad config are modified                    */
/* \param Nnear: the nearest node which make the shoot      */
/*               restrictif. Needed to reuse infos stored   */
/*               in the node structure. Nnear should be non */
/*                NULL                                      */
/* \return: 1 if the shoot succeeded, 0 otherwise           */
/************************************************************/
int bio_restrict_sc_shoot_with_NnearInfo(p3d_rob* robotPt, p3d_node* Nnear, configPt q) {
  p3d_jnt** firstjnts_flexible_sc = get_list_firstjnts_flexible_sc(robotPt);
  p3d_jnt* firstjnt_flexsc = firstjnts_flexible_sc[0];
  int i=0,result_test = -1;
  int AAnumber=-1;
  configPt q_sc_shoot =NULL;
  //double dist_vector[3];
  int nb_farAA =0; 

  while(firstjnt_flexsc!= NULL) {
    AAnumber = get_AAnumber_from_jnt(firstjnt_flexsc);
    if((AAnumber != -1) && (Nnear->list_closed_flex_sc[i]==0)) {
      //the boxes are not overlapping
      result_test = make_rigid_AA_num(AAnumber);
      nb_farAA++;
      if(result_test == 0) {
	PrintInfo(("Warning: restrictive shoot failed "
		   "because of the make_rigid_AA process\n"));
	return 0;
      }
       }
    i++;
    firstjnt_flexsc =   firstjnts_flexible_sc[i];
  }
  // PrintInfo(("%d far flexible sc AA, on a total of %d\n",nb_farAA++, i));
  q_sc_shoot = p3d_alloc_config(robotPt);
 if(p3d_GetExpansionDirectionMethod() == SUBREGION_CS_EXP) {
    bio_side_chain_inbox_shoot(robotPt, Nnear->comp->box_env_small, q_sc_shoot);
 }
 else {
   bio_side_chain_shoot(robotPt, q_sc_shoot);
}

  bio_copy_sch_conf(robotPt,q_sc_shoot,q);
  p3d_destroy_config(robotPt,q_sc_shoot);


  //we set again all  flexible joints as flexible
  firstjnt_flexsc = firstjnts_flexible_sc[0];
  i =0;
  while(firstjnt_flexsc!= NULL) {
    AAnumber = get_AAnumber_from_jnt(firstjnt_flexsc);
    if (AAnumber != -1) {
      result_test = make_flexible_AA_num(AAnumber); 
      if(result_test == 0) {
	PrintInfo(("Warning: restrictive shoot failed "
		   "because of the make_flexible_AA process\n"));
	return 0;
      }
    }
    i++;
    firstjnt_flexsc =   firstjnts_flexible_sc[i];
  }

  return 1;
}
/************************************************************/
/*  \brief: function to generating random values for all the*/
/*         sidechains dofs.                                 */
/* \param robotPt: the robot                                */
/* \param q: only the sidechains dofs are modified          */
/* \return: 1 if the shoot succeeded, 0 otherwise           */
/* \note: the function uses RLG in the case of closed chains*/
/*        and consequently  can change the values of        */
/*        passives dofs                                     */
/************************************************************/
 int bio_side_chain_shoot(p3d_rob *robotPt, configPt q)
{
  int njnt = robotPt->njoints, i, j, k; 
  double vmin, vmax;
  p3d_jnt * jntPt;
  int go_on = 1;

 while(go_on) {    
   for(i=0; i<=njnt; i++) {
     jntPt = robotPt->joints[i];
     for(j=0; j<jntPt->dof_equiv_nbr; j++) {
       k = jntPt->index_dof + j;
       if ((p3d_jnt_get_dof_is_user(jntPt, j)) &&
	   (jntPt->bio_jnt_type == BIO_GAMMA_JNT)){
	 p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	 q[k] = p3d_random(vmin, vmax);
       } else
	 { q[k] = p3d_jnt_get_dof(jntPt, j); }
     }
   }
   if(p3d_get_RLG()) {
     if(p3d_random_loop_generator(robotPt,q)) {
       go_on = 0;
    }
   }
   else {
     go_on = 0;
   }   
 }  
 return(TRUE);
}

/************************************************************
 *  \brief: function to generating random values inside the 
            box_env for all the sidechains dofs.                                 
 * \param robotPt: the robot                                
 * \param box_env: the box inside which configurations are shot
 * \param q: only the sidechains dofs are modified          
 * \return: 1 if the shoot succeeded, 0 otherwise           
 * \note: the function uses RLG in the case of closed chains
 *        and consequently  can change the values of        
 *        passives dofs                                     
 ************************************************************/
int bio_side_chain_inbox_shoot(p3d_rob *robotPt, configPt box_env[], configPt q)
{
  int njnt = robotPt->njoints, i, j, k; 
  double vmin, vmax;
  double s1,s2, rand;
  p3d_jnt * jntPt;
  int go_on = 1;


  if( (box_env == NULL) ||(box_env[0] == NULL)
      ||(box_env[1] == NULL))  {
    PrintInfo(("Error: no box_env_table\n"));
    return FALSE;
  }
  while(go_on) {    
   for(i=0; i<=njnt; i++) {
     jntPt = robotPt->joints[i];
     for(j=0; j<jntPt->dof_equiv_nbr; j++) {
       k = jntPt->index_dof + j;
       if ((p3d_jnt_get_dof_is_user(jntPt, j)) &&
	   (jntPt->bio_jnt_type == BIO_GAMMA_JNT)){
	 p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	 if (p3d_jnt_is_dof_angular(jntPt, j)) {
	   if((box_env[0][k] <-M_PI) || (box_env[0][k] > M_PI) ||
	      (box_env[1][k] <-M_PI) || (box_env[1][k] > M_PI) ) { 
	     PrintInfo(("Warning :Wrong bounds of shooting box\n"));
	   }
	   if(box_env[0][k] <= box_env[1][k]) {
	     q[k] = p3d_random(box_env[0][k],box_env[1][k]);
	   } else {/* we have to shoot randomly in 
	        the union of the 2 intervals [-Pi, max]U[min, Pi]*/
	     s1 =  box_env[1][k]+M_PI;
	     s2= M_PI- box_env[0][k];
	     rand = p3d_random(0.,s1+s2);
	     if(rand <s1) //we are in the second interval 
	       {
		 q[k] = -M_PI+rand; 
	       }
	     else {
	       q[k] = box_env[0][k] + rand - s1;
	     }
	    }
	 }
	 else {
	   q[k] = p3d_random(box_env[0][k],box_env[1][k]);
	 }
       } else {
	 q[k] = p3d_jnt_get_dof(jntPt, j); 
       }
     }
   }
   if(p3d_get_RLG()) {
     if(p3d_random_loop_generator(robotPt,q)) {
       go_on = 0;
     }
   }
   else {
     go_on = 0;
   }   
  }  
  return(TRUE);
}

/************************************************************/
/*  \brief: function to generating random values for the    */
/*         ligand dofs.                                     */
/* \param robotPt: the robot                                */
/* \param q: only the ligand dofs are modified              */
/* \return: 1 if the shoot succeeded, 0 otherwise           */
/* \note: the function uses RLG in the case of closed chains*/
/*        and consequently  can change the values of        */
/*        passives dofs                                     */
/************************************************************/
int bio_ligand_shoot(p3d_rob* robotPt, configPt q) {
  int njnt = robotPt->njoints, i, j, k; 
  double vmin, vmax;
  p3d_jnt * jntPt;
  int go_on = 1;

  if(is_ligand_in_robot(robotPt) ==FALSE){
    PrintInfo(("Warning: try to shoot ligand dofs whereas there is no ligand\n"));
    return FALSE;
  }
 while(go_on) {  
  for(i=robotPt->joints[0]->next_jnt[robotPt->joints[0]->n_next_jnt - 1]->num; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)){
	if(jntPt->bio_jnt_type ==  BIO_OTHER_JNT) {
	  p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	  q[k] = p3d_random(vmin, vmax);
	}
	else{
	  PrintInfo(("Warning:  try to shoot ligand dofs but the type"
		     " of joint is not correct\n"));
	  //in that case we do not change the value of 
	  //the joint but we continue the function
	}
      }
      else{
	q[k] = p3d_jnt_get_dof(jntPt, j); 
      }
    }
  }
  if(p3d_get_RLG()) {
    if(p3d_random_loop_generator(robotPt,q)) {
      go_on = 0;
    }
  }
  else {
    go_on = 0;
  }      
 }
 return(TRUE);
}

/************************************************************
 *  \brief: function to generating random values  inside the 
 *            box_env the for the ligand dofs.                                     
 * \param robotPt: the robot    
 * \param box_env: the box inside which configurations are shot
 * \param q: only the ligand dofs are modified              
 * \return: 1 if the shoot succeeded, 0 otherwise           
 * \note: the function uses RLG in the case of closed chains
 *        and consequently  can change the values of        
 *        passives dofs                                     
 ************************************************************/
int bio_ligand_inbox_shoot(p3d_rob* robotPt, configPt box_env[], configPt q) {
  int njnt = robotPt->njoints, i, j, k; 
  double vmin, vmax;
  double s1,s2, rand;
  p3d_jnt * jntPt;
  int go_on = 1;


  if(is_ligand_in_robot(robotPt) ==FALSE){
    PrintInfo(("Warning: try to shoot ligand dofs whereas there is no ligand\n"));
    return FALSE;
  }
 while(go_on) {  
  for(i=robotPt->joints[0]->next_jnt[robotPt->joints[0]->n_next_jnt - 1]->num; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
	if(jntPt->bio_jnt_type ==  BIO_OTHER_JNT) {
	 if (p3d_jnt_is_dof_angular(jntPt, j)) {
	  p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	   if((box_env[0][k] <-M_PI) || (box_env[0][k] > M_PI) ||
	      (box_env[1][k] <-M_PI) || (box_env[1][k] > M_PI) ) { 
	     PrintInfo(("Warning :Wrong bounds of shooting box\n"));
	   }
	   if(box_env[0][k] <= box_env[1][k]) {
	     q[k] = p3d_random(box_env[0][k],box_env[1][k]);
	   } else {/* we have to shoot randomly in 
	        the union of the 2 intervals [-Pi, max]U[min, Pi]*/
	     s1 =  box_env[1][k]+M_PI;
	     s2= M_PI- box_env[0][k];
	     rand = p3d_random(0.,s1+s2);
	     if(rand <s1) //we are in the second interval 
	       {
		 q[k] = -M_PI+rand; 
	       }
	     else {
	       q[k] = box_env[0][k] + rand - s1;
	     }
	   }
	 }  else {
	   q[k] = p3d_random(box_env[0][k],box_env[1][k]);
	 }
	}  else {
	PrintInfo(("Warning:  try to shoot ligand dofs but the type"
		   " of joint is not correct\n"));
	//in that case we do not change the value of 
	//the joint but we continue the function

      }

    } else { 
	q[k] = p3d_jnt_get_dof(jntPt, j); 
      }
    }
  }
  if(p3d_get_RLG()) {
    if(p3d_random_loop_generator(robotPt,q)) {
      go_on = 0;
    }
  }
  else {
    go_on = 0;
  }      
 }
 return(TRUE);
}

/************************************************************/
/*  \brief: compute the configuration space distance between*/
/*         two configurations taking into account only the  */
/*         ligand dofs.                                     */
/* \param robotPt: the robot                                */
/* \param q_i: initial configuration                        */
/* \param q_f: final configuration                          */
/* \return: the configuration space distance between the two*/
/*          configurations of the ligand                    */
/************************************************************/
double bio_compute_ligand_dist(p3d_rob *robotPt, configPt q_i, configPt q_f) {
  double l = 0.,ljnt = 0.;
  //int i, njnt = robotPt->njoints;
  int j;
  p3d_jnt * jntPt;

 if(is_ligand_in_robot(robotPt) ==FALSE){
    PrintInfo(("Warning: try to compute ligand distances "
	       " whereas there is no ligand\n"));
    return 0.0;
 }
 // for(i=robotPt->joints[0]->next_jnt[robotPt->joints[0]->n_next_jnt - 1]->num; i<=njnt; i++) {
 //  jntPt = robotPt->joints[i];
 jntPt = robotPt->joints[0]->next_jnt[robotPt->joints[0]->n_next_jnt - 1];
 for(j=0; j<jntPt->dof_equiv_nbr; j++) {
   ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, q_i, q_f));
 }
 // }
 l = sqrt(ljnt);
 return l;
}

/************************************************************/
/*  \brief: function to compute the nearest neighbor in a   */
/*          component, of a  given configuration, taking    */
/*          into account only the ligand dofs               */
/* \param robotPt: the robot                                */
/* \param q: the configuration                              */
/* \param comp: the connected component                     */
/* \return: the nearest node of q, NULL if it fails to find */
/*          such a node                                     */
/* \note: there is no filtering of nodes that we always fail*/
/*        to extend has it is done in the current p3d_rrt   */
/*        function                                          */
/************************************************************/
p3d_node *bio_ligand_nearest_neighbor(p3d_rob *robotPt, configPt q, p3d_compco *comp, double *distPt)
{
  p3d_node *Nmin = NULL;
  int SavedDistConfChoice;
  if(is_ligand_in_robot(robotPt) ==FALSE){
    PrintInfo(("Warning: try to compute ligand_nearest_neighbor "
	       " whereas there is no ligand\n"));
    *distPt = 0.;
    return NULL;
  }

  //  Nmin = hrm_nearest_neighbor(robotPt,q,comp,distPt,1);
  SavedDistConfChoice =   p3d_GetDistConfigChoice();
  p3d_SetDistConfigChoice(LIGAND_PROTEIN_DIST);
  Nmin =  NearestWeightNeighbor(robotPt->GRAPH, comp, q);
  p3d_SetDistConfigChoice(SavedDistConfChoice);
  return Nmin;
}


/**************************************************************/
/*  \brief: get the maximal number of extend fails for a node,*/
/*          before beeing rejected as nearest neighbor        */
/*  \return: the maximal number of extend fails for a node    */
/*           (10 by default)                                  */
/**************************************************************/
int bio_get_max_nnode_fail(void) {
  return MAX_NNODE_FAILS;
}


/**************************************************************
 *  \brief: set the maximal number of extend fails for a node,
 *          before beeing rejected as nearest neighbor       
 *  \param nfail :the maximal number of extend fails for a node 
 **************************************************************/
void  bio_set_max_nnode_fail(int nfail) {
 MAX_NNODE_FAILS = nfail;
}


/**************************************************************
 * \brief: desallocate the structures used and allocated in 
 *          the  bio_multimol_collision_report function.
 * \param  sc_autocol_list: the list of side chains in 
 *          autocollision 
 *  param  scsc_col_list: the pairs of side chains in collision
 *          one into the other. scsc_col_list[i] is in collision 
 *          with scsc_col_list[i+1]
 * \param  sc_bb_col_list: list of side_chains in collision with 
 *         the backbone.
 * \param  sc_ligand_col_list: list of side chains in collision 
           with the ligand.
 **************************************************************/
void bio_free_multimol_collision_report(int_tab** sc_autocol_list, int_tab** scsc_col_list, 
					int_tab** sc_bb_col_list, int_tab** sc_ligand_col_list){
 if (*sc_autocol_list != NULL) {
   if( (*sc_autocol_list)->tab!=NULL )
     free((*sc_autocol_list)->tab);
   free(*sc_autocol_list);
   *sc_autocol_list = NULL;
  }
  if (*scsc_col_list != NULL) {
    free((*scsc_col_list)->tab);
    free(*scsc_col_list);
    *scsc_col_list = NULL;
  }
  if (*sc_bb_col_list != NULL) {
    free((*sc_bb_col_list)->tab);
    free(*sc_bb_col_list);
    *sc_bb_col_list = NULL;
  }
 if (*sc_ligand_col_list != NULL) {
   free((*sc_ligand_col_list)->tab);
   free(*sc_ligand_col_list);
   *sc_ligand_col_list = NULL;
 }
}

/**************************************************************
 * \brief: In the case of a multimolecule system (e.g. protein/ligand system)
           and for a configuration q, return the different kind 
           of collision occuring. 
 * \Note:  This function allocate structures which have to be desallocated 
           with the bio_free_multimol_collision_report function.
 * \param  robotPt: the robot
 * \param q: the tested configuration
 * \retval sc_autocol_list: the list of side chains in 
 *         autocollision 
 * \retval scsc_col_list: the pairs of side chains in collision
 *         one into the other. scsc_col_list[i] is in collision 
 *         with scsc_col_list[i+1]
 * \retval sc_bb_col_list: list of side_chains in collision with 
 *         the backbone.
 * \retval sc_ligand_col_list: list of side chains in collision 
 *         with the ligand.
 * \retval is_ligand_bb_col: TRUE if the ligand is in collision 
 *         with the backbone.
 * \retval is_ligand_autocol : TRUE if the ligand is in autocollision
 * \retval is_bb_autocol: TRUE if the backbone is in autocollision
 * \return : TRUE if a collision has been detected, FALSE otherwise
 **************************************************************/
int bio_multimol_collision_report(p3d_rob *robotPt, configPt q, int_tab** sc_autocol_list,
				  int_tab** scsc_col_list , int_tab** sc_bb_col_list, int_tab** sc_ligand_col_list,
				  int* is_ligand_bb_col, int* is_ligand_autocol, int* is_bb_autocol) {
  configPt qsaved;
  int i_col, col_number;
  p3d_poly **p1,**p2;
  *sc_autocol_list = NULL;
  *scsc_col_list = NULL;
  *sc_bb_col_list = NULL;
  *sc_ligand_col_list = NULL;
  (*is_bb_autocol) = 0;
  (*is_ligand_autocol) =0;
  (*is_ligand_bb_col) = 0;

  if(is_ligand_in_robot(robotPt) == FALSE) {
    PrintInfo(("Error : there is only one molecule\n"));
    return FALSE;
  }

  qsaved = p3d_get_robot_config(robotPt);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,q);
  if(p3d_col_test() <= 0) {
    PrintInfo(("no collision reported\n"));
  }
  biocol_report(&col_number, &p1, &p2);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qsaved);
  p3d_destroy_config(robotPt, qsaved);
  if(col_number == 0) {
    //    PrintInfo(("No collision detected \n \n"));
    return FALSE;
  }

  for(i_col=0;i_col<col_number;i_col++) {
    if(p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
      if(p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	if(get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt) == 
	   get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt)) {
	  if(*sc_autocol_list == NULL) {
	    (*sc_autocol_list) = MY_ALLOC(int_tab, 1);
	    (*sc_autocol_list)->size = 0;
	    (*sc_autocol_list)->tab = NULL;
	  }
	  (*sc_autocol_list)->size++;
	  (*sc_autocol_list)->tab = (int*) realloc((*sc_autocol_list)->tab, (*sc_autocol_list)->size*sizeof(int));
	  (*sc_autocol_list)->tab[(*sc_autocol_list)->size-1] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
	}  else {
	  if(*scsc_col_list == NULL) {
	    (*scsc_col_list) = MY_ALLOC(int_tab, 1);
	    (*scsc_col_list)->size = 0;
	    (*scsc_col_list)->tab = NULL;
	  }
	  (*scsc_col_list)->size++;
	  (*scsc_col_list)->tab = (int*) realloc((*scsc_col_list)->tab, 2*((*scsc_col_list)->size)*sizeof(int));
	  (*scsc_col_list)->tab[2*((*scsc_col_list)->size-1)] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
	  (*scsc_col_list)->tab[2*((*scsc_col_list)->size-1)+1] = get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt);
	  //	  PrintInfo(("col between two different side chains \n"));
	}
      }
      else if((p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_OMEGA_JNT) || 
	      (p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PHI_JNT) ||
	      (p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PSI_JNT)) {
	//	PrintInfo(("side chain /col backbone \n"));
	if(*sc_bb_col_list == NULL) {
	  (*sc_bb_col_list) = MY_ALLOC(int_tab, 1);
	  (*sc_bb_col_list)->size = 0;
	  (*sc_bb_col_list)->tab = NULL;
	}
	(*sc_bb_col_list)->size++;
	(*sc_bb_col_list)->tab = (int*) realloc((*sc_bb_col_list)->tab, (*sc_bb_col_list)->size*sizeof(int));
	(*sc_bb_col_list)->tab[(*sc_bb_col_list)->size-1] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
      }
      else  {
	//	PrintInfo(("col ligand / side chain \n"));
	if(*sc_ligand_col_list == NULL) {
	  *sc_ligand_col_list =  MY_ALLOC(int_tab, 1);
	  (*sc_ligand_col_list)->size = 0;
	  (*sc_ligand_col_list)->tab = NULL;
	}
	(*sc_ligand_col_list)->size++;
	(*sc_ligand_col_list)->tab =  (int*) realloc((*sc_ligand_col_list)->tab, ((*sc_ligand_col_list)->size)*sizeof(int));
	(*sc_ligand_col_list)->tab[(*sc_ligand_col_list)->size-1] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
      }
    }
    else if ((p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_OMEGA_JNT) || 
	     (p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PHI_JNT) ||
	     (p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PSI_JNT)) {
      if(p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	//	PrintInfo(("side chain /col backbone \n"));
	if(*sc_bb_col_list == NULL) {
	  (*sc_bb_col_list) = MY_ALLOC(int_tab, 1);
	  (*sc_bb_col_list)->size = 0;
	  (*sc_bb_col_list)->tab = NULL;
	}
	(*sc_bb_col_list)->size++;
	(*sc_bb_col_list)->tab = (int*) realloc((*sc_bb_col_list)->tab, (*sc_bb_col_list)->size*sizeof(int));
	(*sc_bb_col_list)->tab[(*sc_bb_col_list)->size-1] = get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt);
      }
      else if(p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_OTHER_JNT) {
	//	PrintInfo(("col backbone / ligand \n"));
	(*is_ligand_bb_col) = 1;
      }
      else {
	//	PrintInfo(("col backbone-backbone \n"));
	(*is_bb_autocol) =1;
      }
    }
    else {
      if(p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	//	PrintInfo(("col ligand/ side chain \n"));
	if(*sc_ligand_col_list == NULL) {
	  *sc_ligand_col_list =  MY_ALLOC(int_tab, 1);
	  (*sc_ligand_col_list)->size = 0;
	  (*sc_ligand_col_list)->tab = NULL;
	}
	(*sc_ligand_col_list)->size++;
	(*sc_ligand_col_list)->tab =  (int*) realloc ((*sc_ligand_col_list)->tab, ((*sc_ligand_col_list)->size)*sizeof(int));
	(*sc_ligand_col_list)->tab[(*sc_ligand_col_list)->size -1] = get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt);
      }
      else if ((p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_OMEGA_JNT) || 
	       (p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PHI_JNT) ||
	       (p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PSI_JNT)) {
	//	PrintInfo(("col ligand backbone  \n"));
	(*is_ligand_bb_col) = 1;
      }
      else {
	//	PrintInfo(("autocol ligand \n"));
	(*is_ligand_autocol) = 1;
      }
    } 
  }
  //  PrintInfo(("\n"));
  return TRUE;
}
/**************************************************************
 * \brief: desallocate the structures used and allocated in 
 *          the  bio_monomol_collision_report function.
 * \param  sc_autocol_list: the list of side chains in 
 *          autocollision 
 *  param  scsc_col_list: the pairs of side chains in collision
 *          one into the other. scsc_col_list[i] is in collision 
 *          with scsc_col_list[i+1]
 * \param  sc_bb_col_list: list of side_chains in collision with 
 *         the backbone.
 **************************************************************/
void bio_free_monomol_collision_report(int_tab** sc_autocol_list, int_tab** scsc_col_list, 
					int_tab** sc_bb_col_list){
  if (*sc_autocol_list != NULL) {
    free((*sc_autocol_list)->tab);
    free(*sc_autocol_list);
    *sc_autocol_list = NULL;
  }
  if (*scsc_col_list != NULL) {
    free((*scsc_col_list)->tab);
    free(*scsc_col_list);
    *scsc_col_list = NULL;
  }
  if (*sc_bb_col_list != NULL) {
    free((*sc_bb_col_list)->tab);
    free(*sc_bb_col_list);
    *sc_bb_col_list = NULL;
  }
}

/**************************************************************
 * \brief: In the case of a monomolecule system (e.g. single protein with a loop)
           and for a configuration q, return the different kind 
           of collision occuring. 
 * \Note:  This function allocate structures which have to be desallocated 
           with the bio_free_monomol_collision_report function.
 * \param  robotPt: the robot
 * \param q: the tested configuration
 * \retval sc_autocol_list: the list of side chains in 
 *         autocollision 
 * \retval scsc_col_list: the pairs of side chains in collision
 *         one into the other. scsc_col_list[i] is in collision 
 *         with scsc_col_list[i+1]
 * \retval sc_bb_col_list: list of side_chains in collision with 
 *         the backbone.
 * \retval is_bb_autocol: TRUE if the backbone is in autocollision
 * \return : TRUE if a collision has been detected, FALSE otherwise
 **************************************************************/
int bio_monomol_collision_report(p3d_rob *robotPt, configPt q, 	 int_tab **sc_autocol_list,
				 int_tab **scsc_col_list , int_tab **sc_bb_col_list, int *is_bb_autocol) 
{
  configPt qsaved;
  int i_col, col_number;
  p3d_poly **p1,**p2;
  *sc_autocol_list = NULL; // sc_autocol_list->tab = NULL; sc_autocol_list->size = 0;
  *scsc_col_list = NULL;
  *sc_bb_col_list = NULL;
  (*is_bb_autocol) = 0;

  if(is_ligand_in_robot(robotPt) == TRUE) {
    PrintInfo(("Error : there is a ligand or another molecule\n"));
    return FALSE;
  }

  qsaved = p3d_get_robot_config(robotPt);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,q);
  if(p3d_col_test() <= 0) {
    PrintInfo(("No collision detected \n \n"));
    return FALSE;
  }
  biocol_report(&col_number, &p1, &p2);
  p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qsaved);
  p3d_destroy_config(robotPt, qsaved);
  if(col_number == 0) {
    PrintInfo(("No collision reported \n \n"));
    return FALSE;
  }
  for(i_col=0;i_col<col_number;i_col++) {
    if(p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
      if(p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	if(get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt) == 
	   get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt)) {
	  //	  PrintInfo(("Autocol side chain\n"));
	  if(*sc_autocol_list == NULL) {
	    (*sc_autocol_list) = MY_ALLOC(int_tab, 1);
	    (*sc_autocol_list)->size = 0;
	    (*sc_autocol_list)->tab = NULL;
	  }
	  (*sc_autocol_list)->size++;
	  (*sc_autocol_list)->tab =  (int*) realloc((*sc_autocol_list)->tab, (*sc_autocol_list)->size*sizeof(int));
	  (*sc_autocol_list)->tab[(*sc_autocol_list)->size-1] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
	}  else {
	  if(*scsc_col_list == NULL) {
	    (*scsc_col_list) = MY_ALLOC(int_tab, 1);
	    (*scsc_col_list)->size = 0;
	    (*scsc_col_list)->tab = NULL;
	  }
	  (*scsc_col_list)->size++;
	  (*scsc_col_list)->tab = (int*) realloc((*scsc_col_list)->tab, 2*((*scsc_col_list)->size)*sizeof(int));
	  (*scsc_col_list)->tab[2*((*scsc_col_list)->size-1)] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
	  (*scsc_col_list)->tab[2*((*scsc_col_list)->size-1)+1] = get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt);
	  //	  PrintInfo(("col between two different side chains \n"));
	}
      }
      else if((p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_OMEGA_JNT) || 
	      (p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PHI_JNT) ||
	      (p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PSI_JNT)) {
	//	PrintInfo(("side chain /col backbone \n"));
	  if(*sc_bb_col_list == NULL) {
	    (*sc_bb_col_list) = MY_ALLOC(int_tab, 1);
	    (*sc_bb_col_list)->size = 0;
	    (*sc_bb_col_list)->tab = NULL;
	  }
	  (*sc_bb_col_list)->size++;
	  (*sc_bb_col_list)->tab = (int*) realloc((*sc_bb_col_list)->tab, (*sc_bb_col_list)->size*sizeof(int));
	  (*sc_bb_col_list)->tab[(*sc_bb_col_list)->size-1] = get_AAnumber_from_jnt(p1[i_col]->p3d_objPt->jnt);
      }
      else  {
	PrintInfo(("Error in the collision_report\n"));
      }
    }
    else if ((p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_OMEGA_JNT) || 
	     (p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PHI_JNT) ||
	     (p1[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_PSI_JNT)) {
      if(p2[i_col]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	//	PrintInfo(("col backbone / side chain \n"));
	if(*sc_bb_col_list == NULL) {
	  (*sc_bb_col_list) = MY_ALLOC(int_tab, 1);
	  (*sc_bb_col_list)->size = 0;
	  (*sc_bb_col_list)->tab = NULL;
	}
	(*sc_bb_col_list)->size++;
	(*sc_bb_col_list)->tab = (int*) realloc((*sc_bb_col_list)->tab, (*sc_bb_col_list)->size*sizeof(int));
	(*sc_bb_col_list)->tab[(*sc_bb_col_list)->size -1] = get_AAnumber_from_jnt(p2[i_col]->p3d_objPt->jnt);
      }
      else{
	//	PrintInfo(("col backbone_backbone\n"));
	(*is_bb_autocol) = 1;
      }
    }
  }
  //  PrintInfo(("\n"));
  return TRUE;
}

/************************************************************/
/*  \brief: function to copy ordered_array_sch into         */
/*          array_sch with random order                     */
/* \param array_sch: the output                             */
/* \param ordered_array_sch: the input                      */
/* \param num_sch: the number of elements in the array      */
/************************************************************/
void bio_init_array_sch(int *array_sch, int *ordered_array_sch, int num_sch)
{
  int *array_flags;
  int i, j;
  int ind_rem;
  int rand_ind;
  int go_on;

  array_flags = MY_ALLOC(int,num_sch);

  for(i=0; i < num_sch; i++) {
    array_flags[i] = 0;
  }
  
  // copy ordered_array_sch into array_sch with random order
  ind_rem = num_sch;
  while(ind_rem > 0){
    rand_ind = (int) floor(p3d_random(0.0, (double)ind_rem - EPS6));
    i = 0;
    j = 0;
    go_on = 1;
    while(go_on){
      if(array_flags[i + j] == 0) {
	if(i == rand_ind)
	  go_on = 0;
	else
	  i++;
      }
      else
	j++;
    }
    array_flags[i + j] = 1;
    array_sch[--ind_rem] = ordered_array_sch[i + j];
  }

  MY_FREE(array_flags,int,num_sch);
}


/************************************************************/
/*  \brief: function randomly sampling one side-chain       */
/*          and checking collisions                         */
/* \param robotPt: the robot                                */
/* \param index_res: the residue number                     */
/* \param jnt_table: pointer to the protein jnt tables      */
/* \param q: the configuration (modified for the side-chain)*/
/* \return: 0 if collision 1 if not                         */
/************************************************************/
int bio_generate_one_sch_conf_and_checkcoll(p3d_rob *robotPt, int index_res,   
					    Joint_tablespt *jnt_table, configPt q)
{
  int i, j, k;
  double vmin, vmax;
  p3d_jnt *jntPt;


  if(jnt_table[index_res] == NULL) {
    printf("ERROR : bio_generate_one_sch_conf_and_checkcoll : problem with joint tables\n");
    return 0;
  }
  // generate value of each joint in the side-chain
  for(i=0; i < jnt_table[index_res]->n_sc_joints; i++) {
    jntPt = jnt_table[index_res]->sc_joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
      q[k] = p3d_random(vmin, vmax);
      p3d_jnt_set_dof(jntPt, j, q[k]);
    }    
  }

  // update
  // WARNING : a simpler function could be used : update BBox is necessary ? 
  p3d_update_this_robot_pos_without_cntrt(robotPt); 

  // check collision : side-chain - backbone and protein
  // WARNING : provisional check all
  //if(bio_all_molecules_col() > 0)
  //  return 0;
  //if(bio_sc_col(robotPt->num, index_res) > 0)
  if(bio_sc_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0)
    return 0;

  return 1;
}

/************************************************************/
/*  \brief: function randomly sampling one residue backbone */
/*          and checking collisions                         */
/* \param robotPt: the robot                                */
/* \param index_res: the residue number                     */
/* \param jnt_table: pointer to the protein jnt tables      */
/* \param q: the configuration (modified for the side-chain)*/
/* \return: 0 if collision 1 if not                         */
/************************************************************/
int bio_generate_one_bkb_conf_and_checkcoll(p3d_rob *robotPt, int index_res,   
					    Joint_tablespt *jnt_table, configPt q)
{
  int i, j, k;
  double vmin, vmax;
  p3d_jnt *jntPt;
  p3d_cntrt *ct;
  int nint,nint2;
  double intmin[4],intmax[4];
  double intmin2[4],intmax2[4];
  double value;

  if(jnt_table[index_res] == NULL) {
    printf("ERROR : bio_generate_one_bkb_conf_and_checkcoll : problem with joint tables\n");
    return 0;
  }
  // generate value of each joint in the backbone
  for(i=0; i < jnt_table[index_res]->n_bb_joints; i++) {
    jntPt = jnt_table[index_res]->bb_joints[i];
    if(jntPt != NULL) {
      // NOTE: suppose that RLG is set because of H-bonds have been set
      // NOTE: OMEGA joint values are sampled randomly (no RLG)
      if(p3d_get_RLG() && 
	 (jntPt->bio_jnt_type != BIO_OMEGA_JNT)) {
	// SAMPLE USING RLG FOR H-BONDS
	nint = 1;
	p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vmin, &vmax);
	intmin[0] = vmin; 	
	intmax[0] = vmax; 	
	nint2 = 0;	
	for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	     dbl_list_more(robotPt->cntrt_manager->cntrt_call_list);
	     dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
	  ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
	  if((ct->active) && (strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) &&
	     (jntPt->num > ct->argu_i[0]) && (jntPt->num <= ct->argu_i[1])) {
	    // NOTE : irlgd is different for each cntrt !!!	      
	    if(!bio_compute_joint_closure_range(jntPt,ct,-1,&nint2,intmin2,intmax2)) {
	      //printf("Empty closure interval for Hbond cntrt num %d at J%d\n",ct->num,J->num);
	      return -2;
	    }
	    // merge intervals : result in "intmin, intmax"
	    nint = p3d_merge_ang_intervals(nint,nint2,intmin,intmax,intmin2,intmax2);    
	    if(nint == 0) {
	      //printf("Empty intersection of closure interval at J%d\n",J->num);
	      return -2;
	    }
	  }
	}
	// sample value
	// WARNING : suppose only one DOF per jnt
	value = p3d_random_in_several_ordered_intervals(nint,intmin,intmax);
	if(value < -M_PI)
	  value += (2.0*M_PI);
	else if(value > M_PI)
	  value -= (2.0*M_PI);
	
	k = jntPt->index_dof;
	q[k] = value;
	p3d_jnt_set_dof(jntPt, 0, q[k]);
      }
      else {
	// NO H-BONDS
	for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	  k = jntPt->index_dof + j;
	  p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
	  q[k] = p3d_random(vmin, vmax);
	  p3d_jnt_set_dof(jntPt, j, q[k]);
	}    
      }
      // update
      // WARNING : a simpler function could be used : update BBox is necessary ? 
      p3d_update_this_robot_pos_without_cntrt(robotPt); 
      // DEBUG
      //g3d_draw_allwin_active();
    }
  }


  // check collision : side-chain - backbone and protein
  // WARNING : provisional check all
  //if(bio_all_molecules_col() > 0)
  //  return 0;
  if(bio_bb_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0)
    return 0;

  return 1;
}




/************************************************************/
/*  \brief: function randomly sampling one residue backbone */
/*          using an approximation of Ramachandran plots    */
/*          and checking collisions                         */
/* \param robotPt: the robot                                */
/* \param index_res: the residue number                     */
/* \param jnt_table: pointer to the protein jnt tables      */
/* \param q: the configuration (modified for the side-chain)*/
/* \return: 0 if collision 1 if not                         */
/************************************************************/
int bio_generate_one_bkb_conf_ramachandran_and_checkcoll(p3d_rob *robotPt, int index_res,   
							 Joint_tablespt *jnt_table, configPt q)
{
  int i, j, k;
  double vmin_j, vmax_j;
  p3d_jnt *jntPt;
  char AAtype[4]; 
  int curr_phi_zone = -1;
  double value;

  ////////////////////////////////////////////////////
  // CONSTANTS (NOTE: should be moved to a .h file)
  // WARNING : values are given here in degrees 
  //           then they must ve converted to radians
  double min_phi_z1 = -195.0;
  double max_phi_z1 = -30.0;
  double min_phi_z2 = 20.0;
  double max_phi_z2 = 120.0;
  double min_psi_pro = -80.0;
  double max_psi_pro = 210.0;
  double min_psi_z1 = -80.0;
  double max_psi_z1 = 190.0;
  double min_psi_z2 = -50.0;
  double max_psi_z2 = 70.0;
  ////////////////////////////////////////////////////

  if(jnt_table[index_res] == NULL) {
    printf("ERROR : bio_generate_one_bkb_conf_and_checkcoll : problem with joint tables\n");
    return 0;
  }
  
  // get AA type
  jntPt = NULL;
  i = 0;
  while(jntPt == NULL) {
    // WARNING : at least one jnt !!! 
    jntPt = jnt_table[index_res]->bb_joints[i];
    if(jntPt == NULL) i++;
  }
  get_AAtype_from_name(jnt_table[index_res]->bb_joints[i]->name,AAtype);     
  
  // generate value of each joint in the backbone
  for(i=0; i < jnt_table[index_res]->n_bb_joints; i++) {
    jntPt = jnt_table[index_res]->bb_joints[i];
    if(jntPt != NULL) {
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	k = jntPt->index_dof + j;
	p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin_j, &vmax_j);
	// OMEGA 
	if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
	  // limits are defined in the .p3d file
	  value = p3d_random(vmin_j, vmax_j);
	}
	// PHI
	else if(jntPt->bio_jnt_type == BIO_PHI_JNT) {
	  if(strcmp(AAtype,"PRO")==0) {
	    // limits are defined in the .p3d file (VALUE +-15)
	    value = p3d_random(vmin_j, vmax_j);
	    if((value > (-30.0*(M_PI/180.0))) && (value < (-140.0*(M_PI/180.0)))) {
	    printf("WARNING : bio_generate_one_bkb_conf_ramachandran_and_checkcoll : PRO phi outside range : RES NUM.%d , phi = %f\n",
		   index_res,value*(180.0/M_PI));
	    }
	  }
	  else if(strcmp(AAtype,"GLY")==0) {
	    // MAKING SIMPLE : no limits are considered for GLY
	    value = p3d_random(vmin_j, vmax_j);
	  }
	  else {
	    // -- general case --
	    // choose between zone 1 and zone 2
	    if((vmax_j - vmin_j) < (2.0*M_PI - EPS6)) {
	      value = p3d_random(vmin_j, vmax_j);
	      //printf("WARNING : bio_generate_one_bkb_conf_ramachandran_and_checkcoll : Ramachandran is not used\n");
	    }
	    else {
	      value = p3d_random_in_two_intervals(min_phi_z1,max_phi_z1,min_phi_z2,max_phi_z2);
	      if(value <= max_phi_z1) 
		curr_phi_zone = 1;  
	      else
		curr_phi_zone = 2;
	      // convert to radians
	      value *= (M_PI/180.0);
	    }
	  }
	}
	// PSI
	else if(jntPt->bio_jnt_type == BIO_PSI_JNT) {
	  if(strcmp(AAtype,"PRO")==0) {
	    if((vmax_j - vmin_j) < (2.0*M_PI - EPS6)) {
	      value = p3d_random(vmin_j, vmax_j);
	      //printf("WARNING : bio_generate_one_bkb_conf_ramachandran_and_checkcoll : Ramachandran is not used\n");
	    }
	    else {
	      value = p3d_random(min_psi_pro,max_psi_pro);
	      // convert to radians
	      value *= (M_PI/180.0);
	    }
	  }
	  else if(strcmp(AAtype,"GLY")==0) {
	    // MAKING SIMPLE : no limits are considered for GLY
	    value = p3d_random(vmin_j, vmax_j);
	  }
	  else {
	    // -- general case --
	    // choose between zone 1 and zone 2
	    if((vmax_j - vmin_j) < (2.0*M_PI - EPS6)) {
	      value = p3d_random(vmin_j, vmax_j);
	      //printf("WARNING : bio_generate_one_bkb_conf_ramachandran_and_checkcoll : Ramachandran is not used\n");
	    }
	    else {
	      if(curr_phi_zone == 1) {
		value = p3d_random(min_psi_z1,max_psi_z1);	      
	      }
	      else {
		value = p3d_random(min_psi_z2,max_psi_z2);	      
	      }
	      // convert to radians
	      value *= (M_PI/180.0);
	    }
	  }	  
	}
	
	// put into [-180,180] range
	if(value > M_PI)
	  value -= 2.0 * M_PI;
	else if(value < -M_PI)
	  value += 2.0 * M_PI;

	// set value
	q[k] = value;
	p3d_jnt_set_dof(jntPt, j, q[k]);
      }    
    }
  }

  // update
  // WARNING : a simpler function could be used : update BBox is necessary ? 
  p3d_update_this_robot_pos_without_cntrt(robotPt); 

  // check collision : side-chain - backbone and protein
  // WARNING : provisional check all
  //if(bio_all_molecules_col() > 0)
  //  return 0;
  if(bio_bb_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0)
    return 0;

  return 1;
}


/************************************************************/
/*  \brief: function randomly perturbing one side-chain     */
/*          and checking collisions                         */
/* \param robotPt: the robot                                */
/* \param index_res: the residue number                     */
/* \param jnt_table: pointer to the protein jnt tables      */
/* \param q: the configuration (modified for the side-chain)*/
/* \param pct: the factor (0,1] of the joint value rages    */
/* \return: 0 if collision 1 if not                         */
/************************************************************/
int bio_perturb_one_sch_conf_and_checkcoll_1(p3d_rob *robotPt, int index_res,   
					   Joint_tablespt *jnt_table, configPt q, double pct)
{
  int i, j, k;
  double vmin, vmax;
  p3d_jnt *jntPt;
  double inc_v,vari=0.0;


  if(jnt_table[index_res] == NULL) {
    printf("ERROR : bio_perturb_one_sch_conf_and_checkcoll : problem with joint tables\n");
    return 0;
  }
  // generate value of each joint in the side-chain
  for(i=0; i < jnt_table[index_res]->n_sc_joints; i++) {
    jntPt = jnt_table[index_res]->sc_joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
	k = jntPt->index_dof + j;
	p3d_jnt_get_dof_bounds(jntPt, j, &vmin, &vmax);
	// adapt inc_ang to jnt value range
	inc_v = pct * ((vmax - vmin) / 2.0);
	vari = p3d_random(-inc_v, inc_v);
	q[k] = q[k] + vari;
	// punt into jnt range
	if(EQ(vmin,-M_PI) && EQ(vmax,M_PI)) {
	  // put into {-pi,pi} range
	  if(q[k] > M_PI)
	    q[k] -= 2.0*M_PI;
	  if(q[k] < -M_PI)
	    q[k] += 2.0*M_PI;
	}
	else {
	  if(q[k] > vmax)
	    q[k] = vmax;
	  if(q[k] < vmin)
	    q[k] = vmin;
	}
	p3d_jnt_set_dof(jntPt,j,q[k]);
      }    
    }
  }

  // update
  // WARNING : a simpler function could be used : update BBox is necessary ? 
  p3d_update_this_robot_pos_without_cntrt(robotPt); 

  // check collision : side-chain - backbone and protein
  // WARNING : provisional check all
  //if(bio_all_molecules_col() > 0)
  //  return 0;
  //if(bio_sc_col(robotPt->num, index_res) > 0)
  if(bio_sc_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0)
    return 0;

  return 1;
}

/************************************************************/
/*  \brief: function randomly perturbing one side-chain     */
/*          and checking collisions                         */
/* \param robotPt: the robot                                */
/* \param index_res: the residue number                     */
/* \param jnt_table: pointer to the protein jnt tables      */
/* \param q: the configuration (modified for the side-chain)*/
/* \param sigma: the sigma of a normal disribution          */
/* \return: 0 if collision 1 if not                         */
/************************************************************/
int bio_perturb_one_sch_conf_and_checkcoll(p3d_rob *robotPt, int index_res,   
					   Joint_tablespt *jnt_table, configPt q, double sigma)
{
  int i, j, k;
  double vmin, vmax;
  p3d_jnt *jntPt;
  double inc_v,vari=0.0;
  double nr;


  if(jnt_table[index_res] == NULL) {
    printf("ERROR : bio_perturb_one_sch_conf_and_checkcoll : problem with joint tables\n");
    return 0;
  }
  // generate value of each joint in the side-chain
  for(i=0; i < jnt_table[index_res]->n_sc_joints; i++) {
    jntPt = jnt_table[index_res]->sc_joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      if (p3d_jnt_get_dof_is_user(jntPt, j)) {
	k = jntPt->index_dof + j;
	p3d_jnt_get_dof_bounds(jntPt, j, &vmin, &vmax);
	nr = fabs(NormalRand(sigma));
	if(nr >= 1.0) {
	  q[k] = p3d_random(vmin, vmax);
	}
	else {
	  // adapt inc_ang to jnt value range
	  inc_v = nr * ((vmax - vmin) / 2.0);
	  vari = p3d_random(-inc_v, inc_v);
	  q[k] = q[k] + vari;
	  // punt into jnt range
	  if(EQ(vmin,-M_PI) && EQ(vmax,M_PI)) {
	    // put into {-pi,pi} range
	    if(q[k] > M_PI)
	      q[k] -= 2.0*M_PI;
	    if(q[k] < -M_PI)
	      q[k] += 2.0*M_PI;
	  }
	  else {
	    if(q[k] > vmax)
	      q[k] = vmax;
	    if(q[k] < vmin)
	      q[k] = vmin;
	  }
	}
	p3d_jnt_set_dof(jntPt,j,q[k]);
      }    
    }
  }

  // update
  // WARNING : a simpler function could be used : update BBox is necessary ? 
  p3d_update_this_robot_pos_without_cntrt(robotPt); 

  // check collision : side-chain - backbone and protein
  // WARNING : provisional check all
  //if(bio_all_molecules_col() > 0)
  //  return 0;
  //if(bio_sc_col(robotPt->num, index_res) > 0)
  if(bio_sc_col(num_subrobot_AA_from_AAnumber(robotPt,index_res), index_res) > 0)
    return 0;

  return 1;
}


/************************************************************/
/*  \brief: changes the conformation of side-chains         */
/*          in order to avoid collisions                    */
/* \param robotPt: the robot                                */
/* \param q: the configuration (modified)                   */
/* \return: 1 if success 0 if not                           */
/* \note: In the current version, side-chain joint values   */
/*        are randomly sampled in the whole joint range.    */
/*        An imprevement will consist in generating values  */
/*        by random perturbations                           */
/************************************************************/
int bio_deform_schs_avoiding_collision(p3d_rob *robPt, configPt q, int print_messages)
{
  int ic,ij;
  //p3d_jnt *jntPt;
  int col_number;
  p3d_poly **p1,**p2;
  int p1inbkb = 0;
  p3d_jnt **list_jntPt;
  int *ordered_array_sch;
  int *array_sch;
  int num_sch;
  int index_arr;
  int tries_allsch, tries_onesch;
  int procOK=0, procOK2;
  Joint_tablespt *jnt_table;
  int NALLSH, NONESH;

  // PARAMETERS ******
  // WARNING : TWO SETS OF PARAMETERS DEPENDING ON THE CALLING FUNCTION
  if(print_messages) {
    NALLSH = 10;
    NONESH = 1000;
  }
  else {
    NALLSH = 3;
    NONESH = 500;
  }
  // *****************

  // check collisions 
  // WARNING : this test is possibly redundant with a previous one
  biocol_report(&col_number, &p1, &p2);
  
  if(col_number > 0) {
    // alloc memory for lists
    list_jntPt = MY_ALLOC(p3d_jnt *,col_number*2);
    ordered_array_sch = MY_ALLOC(int,col_number*2);
    
    // make list of joints corresponding to colliding bodies
    // WARNING : joints are possibly repeated 
    for(ic=0;ic<col_number;ic++) {
      if(p1[ic]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	list_jntPt[ic*2] = p1[ic]->p3d_objPt->jnt;
	p1inbkb = 0;
      }
      else {
	list_jntPt[ic*2] = NULL;
	p1inbkb = 1;
      }
      if(p2[ic]->p3d_objPt->jnt->bio_jnt_type == BIO_GAMMA_JNT) {
	list_jntPt[(ic*2)+1] = p2[ic]->p3d_objPt->jnt;
      }
      else {
	list_jntPt[(ic*2)+1] = NULL;
	if(p1inbkb == 1) {
	  if(print_messages)
	    printf("CANNOT REMOVE COLLISIONS : collision between two bkb atoms !\n");
	  MY_FREE(list_jntPt,p3d_jnt *,col_number*2);
	  MY_FREE(ordered_array_sch,int,col_number*2);
	  return 0;
	}
      }
    }
    
    // make list of residues whose side-chain collides
    // and deactivate collisions
    // WARNING : residues are possibly repeated 
    num_sch = 0;
    for(ij=0;ij<(col_number*2);ij++) {
      if(list_jntPt[ij] != NULL) {
	ordered_array_sch[num_sch] = get_AAnumber_from_name(list_jntPt[ij]->o->name);
	supress_sc_rigid(num_subrobot_AA_from_AAnumber(robPt,ordered_array_sch[num_sch]), ordered_array_sch[num_sch]);
	num_sch ++;
      }
    }
    
    // list of colliding side-chains with random order
    array_sch = MY_ALLOC(int,num_sch);
    
    // generate side-chains
    tries_allsch = 0;
    procOK = 0;   
    while(!procOK && (tries_allsch < NALLSH)) {    
      tries_allsch ++;  
      // init array of side-chains with random order
      bio_init_array_sch(array_sch,ordered_array_sch,num_sch);
      
      index_arr = 0;
      procOK2 = 1;
      while(procOK2 && index_arr < num_sch) {
	// activate collision of this side-chain
	activate_sc_rigid(num_subrobot_AA_from_AAnumber(robPt,array_sch[index_arr]), array_sch[index_arr]);
	if(bio_sc_col(num_subrobot_AA_from_AAnumber(robPt,array_sch[index_arr]), array_sch[index_arr]) > 0) {
	  // get bioCD jnt table
	  jnt_table =  give_joint_tables(num_subrobot_AA_from_AAnumber(robPt,array_sch[index_arr]));
	  tries_onesch = 0;
	  procOK2 = 0;
	  while(!procOK2 && (tries_onesch < NONESH)) {    
	    tries_onesch ++;  
	    // generate this side-chain conformation and check collision
	    // - one possibility is to sample randomly in the whole joint-intervals
	    //procOK2 = bio_generate_one_sch_conf_and_checkcoll(robPt, array_sch[index_arr], jnt_table, q);
	    // - other possibility is to make a gaussian sampling around the current conformation
	    procOK2 = bio_perturb_one_sch_conf_and_checkcoll(robPt, array_sch[index_arr], jnt_table, 
							     q, ((double)tries_onesch+1.0)/(double)NONESH);
	    
	    // IDEA : INTENAR PRIMERO ELIMINARLA COLISION MOVIENDO UNICAMENTE LA JNT
	    //        IMPLICADA DIRECTAMENTE. SI NI SE CONSIGUE, PROPAGAR A LOS JNTS PRECEDENTES
	    
	    if(procOK2) {
	      if(print_messages)
		printf("REMOVED COLLISIONS for side-chain of residue %d - after %d tries\n",array_sch[index_arr],tries_onesch);
	    }
	  }
	}
	index_arr ++;
      }
      procOK = procOK2 && (index_arr == num_sch);
      if(!procOK) {
	if(print_messages) {
	  printf("CANNOT REMOVE COLLISIONS for side-chain of residue %d\n",array_sch[index_arr-1]);
	printf("RE-STARTIG PROCESS - iteration num %d\n",tries_allsch);
	}
	// re-deactivate collisions of side-chains
	for(index_arr=0;index_arr<num_sch;index_arr++) {
	  supress_sc_rigid(num_subrobot_AA_from_AAnumber(robPt,array_sch[index_arr]), array_sch[index_arr]);
	}
      }
    }
    
    if(!procOK) {
      if(print_messages)
	printf("END OF PROCESS FOR REMOVING SIDE-CHAIN COLLISIONS :  F A I L U R E\n");
    }
    else {
      if(print_messages)
	printf("END OF PROCESS FOR REMOVING SIDE-CHAIN COLLISIONS :  O K\n");
    }
    
    // free memory
    MY_FREE(list_jntPt,p3d_jnt *,col_number*2);
    MY_FREE(ordered_array_sch,int,col_number*2);
    MY_FREE(array_sch,int,num_sch);
  }
    
  return procOK;
}


/************************************************************/
/*  \brief: changes the conformation of the passive params  */
/*          if they are involved in the current collision   */
/* \param robotPt: the robot                                */
/* \param q: the configuration (modified)                   */
/* \return: 1 if success 0 if not                           */
/* \note: The current implementation only considers         */
/*        side-chain torsions as possible passive params.   */
/************************************************************/
int bio_perturb_and_check_passive_params_involved_in_collision(p3d_rob *robotPt, configPt qinv)
{
  int processOK = FALSE;

  if(is_ligand_in_robot(robotPt) == TRUE) {
    printf("\nWARNING : bio_perturb_and_check_passive_params_involved_in_collision :\n          this function is currently implemented for a protein without ligand,\n          if the model contains a ligand, please use the ligRRT algorithm\n\n");
    return FALSE;
  }
  else {
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qinv);
    if(p3d_col_test() <= 0) {
      PrintInfo(("no collision reported\n"));
      return(FALSE);
    }    
    // WARNING : The current implementation considers that the passive params correspond to side-chains
    processOK = bio_deform_schs_avoiding_collision(robotPt,qinv,0);
  }
  return(processOK);
}

/************************************************************/
/*  \brief: gets the list of passive joints                 */
/*          involved in the current collision               */
/* \param robotPt: the robot                                */
/* \param qinv: the invalid configuration                   */
/* \param npassJPt: pointer to the number of joints         */
/* \param passJlistPt: list of joints (allocated here)      */
/* \return: 1 if there are involved passivejoints 0 if not  */
/* \note: The current implementation only considers         */
/*        side-chain torsions as possible passive params.   */
/************************************************************/
int bio_get_list_of_passive_joints_involved_in_collision(p3d_rob *robotPt, configPt qinv, 
							 int *npassJPt, p3d_jnt ***passJlistPt)
{
  int pass_jnts_in_coll = FALSE;
  int_tab *sc_autocol_list = NULL;
  int_tab *scsc_col_list = NULL;
  int_tab *sc_bb_col_list = NULL;
  int_tab *sc_ligand_col_list = NULL;
  int_tab *all_sc_col_list = NULL;
  int  is_bb_autocol;
  int is_ligand_autocol;
  int is_ligand_bb_col;
  int i,j,notinlist;
  Joint_tablespt *jnt_table;

  // CASE 1 : ONE PROTEIN WITH LIGAND
  if(is_ligand_in_robot(robotPt) == TRUE) {
    ////////////////////////////////////////////////////////
/*     printf("\nWARNING : bio_get_list_of_passive_joints_involved_in_collision :\n          this function is currently implemented for a protein without ligand,\n          if the model contains a ligand, please use the ligRRT algorithm\n\n"); */
/*     return FALSE; */
    ////////////////////////////////////////////////////////
    if(!bio_multimol_collision_report(robotPt, qinv,&sc_autocol_list,
				      &scsc_col_list,&sc_bb_col_list,&sc_ligand_col_list,
				      &is_ligand_bb_col,&is_ligand_autocol,&is_bb_autocol)) {
      // NO COLLISION
      bio_free_multimol_collision_report(&sc_autocol_list,&scsc_col_list,&sc_bb_col_list,&sc_ligand_col_list);
      return FALSE;      
    }
    if(is_bb_autocol || is_ligand_bb_col || is_ligand_autocol) {
      // BACKBONE COLLISION
      bio_free_multimol_collision_report(&sc_autocol_list,&scsc_col_list,&sc_bb_col_list,&sc_ligand_col_list);
      return FALSE;      
    }

    //////////////////////////////////////////////////////////////////
    // compute array of all side-chains in collision (MOVE TO FUNCTION)
    if(sc_autocol_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<sc_autocol_list->size ;i++) {
	all_sc_col_list->size++;
	all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	all_sc_col_list->tab[(all_sc_col_list->size)-1] = sc_autocol_list->tab[i]; 
      }
    }
    if(scsc_col_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<scsc_col_list->size ;i++) {
	notinlist = 1;
	for(j=0;(j<all_sc_col_list->size)&& notinlist;j++) {
	  if(all_sc_col_list->tab[j] == scsc_col_list->tab[i])
	    notinlist = 0;
	}
	if(notinlist) {
	  all_sc_col_list->size++;
	  all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	  all_sc_col_list->tab[(all_sc_col_list->size)-1] = scsc_col_list->tab[i]; 
	}
      }
    }
    if(sc_bb_col_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<sc_bb_col_list->size ;i++) {
	notinlist = 1;
	for(j=0;(j<all_sc_col_list->size)&& notinlist;j++) {
	  if(all_sc_col_list->tab[j] == sc_bb_col_list->tab[i])
	    notinlist = 0;
	}
	if(notinlist) {
	  all_sc_col_list->size++;
	  all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	  all_sc_col_list->tab[(all_sc_col_list->size)-1] = sc_bb_col_list->tab[i]; 
	}
      }
    }
    if(sc_ligand_col_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<sc_ligand_col_list->size ;i++) {
	notinlist = 1;
	for(j=0;(j<all_sc_col_list->size)&& notinlist;j++) {
	  if(all_sc_col_list->tab[j] == sc_ligand_col_list->tab[i])
	    notinlist = 0;
	}
	if(notinlist) {
	  all_sc_col_list->size++;
	  all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	  all_sc_col_list->tab[(all_sc_col_list->size)-1] = sc_ligand_col_list->tab[i]; 
	}
      }
    }  
    bio_free_multimol_collision_report(&sc_autocol_list,&scsc_col_list,&sc_bb_col_list,&sc_ligand_col_list);
  }
  // CASE 2 : ONE PROTEIN WITHOUT LIGAND
  else {
    if(!bio_monomol_collision_report(robotPt,qinv,&sc_autocol_list,
				     &scsc_col_list,&sc_bb_col_list,&is_bb_autocol)) {
      // NO COLLISION
      bio_free_monomol_collision_report(&sc_autocol_list,&scsc_col_list,&sc_bb_col_list);
      return FALSE;      
    }
    if(is_bb_autocol) {
      // BACKBONE COLLISION
      bio_free_monomol_collision_report(&sc_autocol_list,&scsc_col_list,&sc_bb_col_list);
      return FALSE;      
    }

    //////////////////////////////////////////////////////////////////
    // compute array of all side-chains in collision (MOVE TO FUNCTION)
    if(sc_autocol_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<sc_autocol_list->size ;i++) {
	all_sc_col_list->size++;
	all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	all_sc_col_list->tab[(all_sc_col_list->size)-1] = sc_autocol_list->tab[i]; 
      }
    }
    if(scsc_col_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<scsc_col_list->size ;i++) {
	notinlist = 1;
	for(j=0;(j<all_sc_col_list->size)&& notinlist;j++) {
	  if(all_sc_col_list->tab[j] == scsc_col_list->tab[i])
	    notinlist = 0;
	}
	if(notinlist) {
	  all_sc_col_list->size++;
	  all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	  all_sc_col_list->tab[(all_sc_col_list->size)-1] = scsc_col_list->tab[i]; 
	}
      }
    }
    if(sc_bb_col_list != NULL) {
      if(all_sc_col_list == NULL) {
	all_sc_col_list = MY_ALLOC(int_tab, 1);
	all_sc_col_list->size = 0;
	all_sc_col_list->tab = NULL;
      }     
      for(i=0; i<sc_bb_col_list->size ;i++) {
	notinlist = 1;
	for(j=0;(j<all_sc_col_list->size)&& notinlist;j++) {
	  if(all_sc_col_list->tab[j] == sc_bb_col_list->tab[i])
	    notinlist = 0;
	}
	if(notinlist) {
	  all_sc_col_list->size++;
	  all_sc_col_list->tab = (int*) realloc(all_sc_col_list->tab,all_sc_col_list->size*sizeof(int));
	  all_sc_col_list->tab[(all_sc_col_list->size)-1] = sc_bb_col_list->tab[i]; 
	}
      }
    }
    bio_free_monomol_collision_report(&sc_autocol_list,&scsc_col_list,&sc_bb_col_list);
  } 
  //////////////////////////////////////////////////////////////////
   
  // create joint list
  if(all_sc_col_list != NULL) {
    for(i=0; i<all_sc_col_list->size; i++) {
      jnt_table =  give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,all_sc_col_list->tab[i]));
      for(j=0; j<jnt_table[all_sc_col_list->tab[i]]->n_sc_joints; j++) {
	if(*npassJPt == 0) {
	  *passJlistPt = MY_ALLOC(p3d_jnt *,1);
	  pass_jnts_in_coll = TRUE;	  
	}
	else {
	  *passJlistPt = MY_REALLOC((*passJlistPt),p3d_jnt *,(*npassJPt),(*npassJPt)+1);
	}
	(*passJlistPt)[*npassJPt] = jnt_table[all_sc_col_list->tab[i]]->sc_joints[j];
	(*npassJPt)++;
      }
    }
  
    // FREE MEMORY
    // free array of all side-chains in collision
    free(all_sc_col_list->tab);
    free(all_sc_col_list);
    all_sc_col_list = NULL;
  }    
  
  return(pass_jnts_in_coll);
}

/**********************************************************************/
/**********************************************************************/
/* Fuctions realted with the management of passive parameters         */ 
/**********************************************************************/
/**********************************************************************/

int bio_set_all_sch_dofs_as_passive_parameters_for_planner(p3d_rob *robotPt)
{
  int i,njnt = robotPt->njoints;
  p3d_jnt * jntPt;
  char AAtype[4];
 
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    if(jntPt->o != NULL) {
      get_AAtype_from_name(jntPt->o->name, AAtype);    
      if(jntPt->bio_jnt_type == BIO_GAMMA_JNT) {
/*       if((jntPt->bio_jnt_type == BIO_GAMMA_JNT) && */
/* 	 (strcmp(AAtype,"CYS")!=0)){ */
	p3d_jnt_set_is_active_for_planner(jntPt, 0);
      }
    }
  }

  p3d_set_flag_passive_parameters_for_planner(TRUE);

  printf("All side-chain torsions have been set as passive parameters\n");
/*   printf("(Excepting for CYS !!!)\n"); */

  return(TRUE);
}

int bio_set_all_sch_dofs_as_active_parameters_for_planner(p3d_rob *robotPt)
{
  int i,njnt = robotPt->njoints;
  p3d_jnt * jntPt;
  
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    if(jntPt->bio_jnt_type == BIO_GAMMA_JNT) {
      p3d_jnt_set_is_active_for_planner(jntPt, 1);
    }
  }

  // WARNING : suppose that there are no other passive params than the side-chain dofs !!!
  p3d_set_flag_passive_parameters_for_planner(FALSE);

  printf("All side-chain torsions have been set as active parameters\n");

  return(TRUE);
}

/**********************************************************************/
/**********************************************************************/
/* Fuctions realted with the definition and operations of  a "triade" */ 
/**********************************************************************/
/**********************************************************************/

static int ntriades = 0;
static p3d_jnt **triade_jntPt_list;
static int *triade_conftype_list;

// NOTE: cannot use jnt_table before initialization of BioCD !!!
//int bio_set_triade(int resSeq1, int resSeq2, int resSeq3)
int bio_set_triade(int jntnum1, int jntnum2, int jntnum3, int conftype)
{
  //  Joint_tablespt *jnt_table;
  p3d_rob *robotPt=NULL;
  p3d_jnt *triade_jntPt[3];
  int i;

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  //jnt_table = give_joint_tables(robotPt->num);

  //triade_jntPt[0] = jnt_table[resSeq1]->bb_joints[1];
  //triade_jntPt[1] = jnt_table[resSeq2]->bb_joints[1];
  //triade_jntPt[2] = jnt_table[resSeq3]->bb_joints[1];
  triade_jntPt[0] = robotPt->joints[jntnum1];
  triade_jntPt[1] = robotPt->joints[jntnum2];
  triade_jntPt[2] = robotPt->joints[jntnum3];

  ntriades++;
  if(ntriades == 1) {
    triade_jntPt_list = MY_ALLOC(p3d_jnt *,3);
    triade_conftype_list = MY_ALLOC(int,1);
  }
  else {
    triade_jntPt_list = MY_REALLOC(triade_jntPt_list,p3d_jnt *,ntriades*3,(ntriades+1)*3);
    triade_conftype_list = MY_REALLOC(triade_conftype_list,int,ntriades,ntriades+1);
  }

  for(i=0; i<3; i++) {
    triade_jntPt_list[(ntriades-1)*3 + i] = triade_jntPt[i];
  }
  triade_conftype_list[ntriades-1] = conftype;

  return 1;
}


int bio_get_triade(int triade_num, p3d_jnt **triade_jntPt, int *triade_conftype)
{
  int i;

  if(triade_num >= ntriades) {
    return 0;
  }
    
  for(i=0; i<3; i++) {
    triade_jntPt[i] = triade_jntPt_list[triade_num*3 + i];
  }
  *triade_conftype = triade_conftype_list[triade_num];

  return 1;
}

/**********************************************************************/

double bio_measure_triade_surface_area(p3d_jnt **triade_jntPt, configPt q)
{
  double area;
  p3d_vector3 pA,pB,pC;
  p3d_vector3 vAB,vAC,vcprod;
  

  // WARNING: suppose that the function "set_and_update" has been called
  //          just before calling this function
  p3d_jnt_get_cur_vect_point(triade_jntPt[0],pA);
  p3d_jnt_get_cur_vect_point(triade_jntPt[1],pB);
  p3d_jnt_get_cur_vect_point(triade_jntPt[2],pC);

  // The area is calutated using vectors: S = 1/2 |AB x AC|
  p3d_vectSub(pB,pA,vAB);
  p3d_vectSub(pC,pA,vAC);
  p3d_vectXprod(vAB,vAC,vcprod);
  area = 0.5 * (double) p3d_vectNorm(vcprod);
  
  return area;
}

/**********************************************************************/
/**********************************************************************/
/* Fuctions realted with distances between atom pairs                 */ 
/**********************************************************************/
/**********************************************************************/

static int npairs = 0;
static p3d_jnt **pairs_jntPt_list;
static int pairs_conftype;

// NOTE: cannot use jnt_table before initialization of BioCD !!!
//int bio_set_triade(int resSeq1, int resSeq2, int resSeq3)
int bio_set_pairs_for_dist(int nump, int *pairslist, int conftype)
{
  //  Joint_tablespt *jnt_table;
  p3d_rob *robotPt=NULL;
  int i;

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  npairs = nump;
  pairs_conftype = conftype;

  pairs_jntPt_list = MY_ALLOC(p3d_jnt *,npairs*2);
  for(i=0; i<(npairs*2); i++) {
    pairs_jntPt_list[i] = robotPt->joints[pairslist[i]];
  }

  return 1;
}


int bio_get_pairs_for_dist(int *nump, p3d_jnt ***pairs_jntPt, int *conftype)
{
  if(npairs == 0)
    return 0;
  
  *nump = npairs;
  *pairs_jntPt = pairs_jntPt_list;
  *conftype = pairs_conftype;

  return 1;
}

/**********************************************************************/

double bio_measure_distance_between_atom_pairs(int nump, p3d_jnt **pairs_jntPt, configPt q)
{
  int i;
  double dist=0.0;
  p3d_vector3 pA,pB,pos_diff;
  
  // WARNING: suppose that the function "set_and_update" has been called
  //          just before calling this function
  for(i=0; i<npairs; i++) {
    p3d_jnt_get_cur_vect_point(pairs_jntPt[2*i],pA);
    p3d_jnt_get_cur_vect_point(pairs_jntPt[(2*i)+1],pB);
    p3d_vectSub(pB,pA,pos_diff);
    dist += (double) p3d_vectNorm(pos_diff);
  }
  
  return dist;
}


/**********************************************************************************/
/**********************************************************************************/
/*        Functions concerning the RMSD to the "goal" conformation                */
/**********************************************************************************/
/**********************************************************************************/

// array to stock the joint coordinates of the "goal" robot
// NOTE: the element 0 corresponts to jnt 1
// - this array is allocated and filled in function load_goal_jnt_coordinates_from_p3d_file
// - WARNING : memory is freed ???!!!
static p3d_vector3 *goal_jnt_coords;
//static double *goal_jnt_v;
static int use_goal_jnt_coords = 0;


void bio_free_goal_jnt_coords(int n_alloc_elems)
{
  //p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  MY_FREE(goal_jnt_coords,p3d_vector3,n_alloc_elems);  
  //  MY_FREE(goal_jnt_v,double,robPt->njoints);  
}

// WARNING : this function only handles 1 jnt per residue (OMEGA)
static int load_goal_jnt_coordinates_from_p3d_file(void)
{
  p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  const char *file_name;
  char *file_name_s=NULL;
  char dir_name[512];
  FILE *goal_p3d_f;
  char  fct[256],namejnt[256],gc;
  double dtab[6];
  int i;
  p3d_vector3 jpos;
  int n_omegas;
  p3d_jnt *jntPt;
  char *jnt_type_name;
  int readnextcoords = 0;
  int indexletter=0;
  
  // select file
  p3d_get_directory(dir_name);
  file_name = fl_show_fselector("Select file with goal structure",dir_name,"*.p3d","");
  if(file_name == NULL) {
    printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : no goal structure selected\n");
    return 0;
  }

  // open file
  file_name_s = strdup( file_name );
  if(!(goal_p3d_f = fopen(file_name_s,"r"))) {
    fl_show_alert("The file could not be opened !",
		  "File does not exist or you do not have the right to open it.",file_name,1);
    return 0;
  }

  // count n_omegas
  n_omegas = 0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     
    // WARNING : only N positions !!!
    if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
    // PRUEBA : RMSD LIPASE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //if(jntPt->bio_jnt_type == BIO_PSI_JNT) {
    // PRUEBA : RMSD LINKER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*     if((jntPt->bio_jnt_type == BIO_OMEGA_JNT) || */
/*        (jntPt->bio_jnt_type == BIO_PHI_JNT)) { */
      n_omegas++;
    }
  }

  // memory allocation (suppose that both "robots" have the same number of jnts)
  goal_jnt_coords = MY_ALLOC(p3d_vector3, n_omegas);
  //goal_jnt_v = MY_ALLOC(double, robPt->njoints);

  // read file and save information
  i = 0;
  while(fscanf(goal_p3d_f,"%s", fct) != EOF) {
    if(i>n_omegas) {
      printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : goal p3d has different number of residues\n");    
      fclose(goal_p3d_f);
      bio_free_goal_jnt_coords(i-1);
      return 0;	
    }

    if(fct[0]=='#') {
      /* comment in file: ignore the line ! */
      do{
	gc=getc(goal_p3d_f);
      }
      while(gc!='\n');
      continue;
    }

    if(strcmp(fct,"p3d_set_name")== 0) {
      if(!read_desc_name(goal_p3d_f,namejnt)) {
	printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : error while reading file\n");    
	fclose(goal_p3d_f);
	bio_free_goal_jnt_coords(i-1);
	return 0;	
      }
      indexletter = 0;
      jnt_type_name = givemeword(namejnt, '.', &indexletter);
      if(strcmp(jnt_type_name,"omega") == 0) {
      // PRUEBA : RMSD LIPASE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      //if(strcmp(jnt_type_name,"psi") == 0) {
      // PRUEBA : RMSD LINKER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*       if((strcmp(jnt_type_name,"omega") == 0) || */
/* 	 (strcmp(jnt_type_name,"phi") == 0)) { */
	readnextcoords = 1;
      }
      else {
	readnextcoords = 0;
      }
    }
    
    if((strcmp(fct,"p3d_set_pos_axe")== 0) && (readnextcoords == 1)) {
      if(!read_desc_double(goal_p3d_f,6,dtab)) {
	printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : error while reading file\n");    
	fclose(goal_p3d_f);
	bio_free_goal_jnt_coords(i-1);
	return 0;	
      }

      jpos[0] = (p3d_matrix_type)dtab[0];
      jpos[1] = (p3d_matrix_type)dtab[1];
      jpos[2] = (p3d_matrix_type)dtab[2];

      p3d_vectCopy(jpos,goal_jnt_coords[i]);
      
      // OPERATIONS FOR JNT VALUES
      // WARNINIG : this function only works with with 1dof joints !!!
/*     if(strcmp(fct,"p3d_set_dof")== 0) { */
/*       if(!read_desc_double(goal_p3d_f,1,dtab)) { */
/* 	printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : error while reading file\n");     */
/* 	fclose(goal_p3d_f); */
/* 	bio_free_goal_jnt_coords(i-1); */
/* 	return 0;	 */
/*       } */

/*       goal_jnt_v[i] = dtab[0]; */

      i++;
    }
  }

  //printf("njoints = %d i = %d\n",robPt->njoints,i);
  if(i<n_omegas) {
    printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : goal p3d has different number of residues\n");    
    fclose(goal_p3d_f);
    bio_free_goal_jnt_coords(i-1);
    return 0;	
  }

  fclose(goal_p3d_f);
  return 1;
}

// WARNING : this function only handles 1 jnt per residue (OMEGA)
static int load_ligand_goal_jnt_coordinates_from_p3d_file(void)
{
  p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  const char *file_name;
  char *file_name_s=NULL;
  char dir_name[512];
  FILE *goal_p3d_f;
  char  fct[256],namejnt[256],gc;
  double dtab[6];
  int i;
  p3d_vector3 jpos;
  int n_lig_jnts;
  char *jnt_type_name;
  int readnextcoords = 0;
  int indexletter=0;
  
  // select file
  p3d_get_directory(dir_name);
  file_name = fl_show_fselector("Select file with goal structure",dir_name,"*.p3d","");
  if(file_name == NULL) {
    printf("ERROR : load_ligand_goal_jnt_coordinates_from_p3d_file : no goal structure selected\n");
    return 0;
  }

  // open file
  file_name_s = strdup( file_name );
  if(!(goal_p3d_f = fopen(file_name_s,"r"))) {
    fl_show_alert("The file could not be opened !",
		  "File does not exist or you do not have the right to open it.",file_name,1);
    return 0;
  }

  // count n_lig_jnts
  n_lig_jnts = robPt->njoints - robPt->joints[0]->next_jnt[robPt->joints[0]->n_next_jnt - 1]->num + 1;
  
  // memory allocation (suppose that both "robots" have the same number of jnts)
  goal_jnt_coords = MY_ALLOC(p3d_vector3, n_lig_jnts);
  //goal_jnt_v = MY_ALLOC(double, robPt->njoints);

  // read file and save information
  i = 0;
  while(fscanf(goal_p3d_f,"%s", fct) != EOF) {
    if(i > n_lig_jnts) {
      printf("ERROR : load_ligand_goal_jnt_coordinates_from_p3d_file : goal p3d ligand model has different number of jnts\n");    
      fclose(goal_p3d_f);
      bio_free_goal_jnt_coords(i-1);
      return 0;	
    }

    if(fct[0]=='#') {
      /* comment in file: ignore the line ! */
      do{
	gc=getc(goal_p3d_f);
      }
      while(gc!='\n');
      continue;
    }

    if(!readnextcoords) {
      if(strcmp(fct,"p3d_set_name")== 0) {
	if(!read_desc_name(goal_p3d_f,namejnt)) {
	  printf("ERROR : load_ligand_goal_jnt_coordinates_from_p3d_file : error while reading file\n");    
	  fclose(goal_p3d_f);
	  bio_free_goal_jnt_coords(i-1);
	  return 0;	
	}
	indexletter = 1;
	jnt_type_name = givemeword(namejnt, '.', &indexletter);
	if(strcmp(jnt_type_name,"lig_base") == 0) {
	  // PRUEBA : RMSD LIPASE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	  //if(strcmp(jnt_type_name,"psi") == 0) {
	  // PRUEBA : RMSD LINKER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	  /*       if((strcmp(jnt_type_name,"omega") == 0) || */
	  /* 	 (strcmp(jnt_type_name,"phi") == 0)) { */
	  readnextcoords = 1;
	}
      }
    }
    
    if((strcmp(fct,"p3d_set_pos_axe")== 0) && (readnextcoords == 1)) {
      if(!read_desc_double(goal_p3d_f,6,dtab)) {
	printf("ERROR : load_ligand_goal_jnt_coordinates_from_p3d_file : error while reading file\n");    
	fclose(goal_p3d_f);
	bio_free_goal_jnt_coords(i-1);
	return 0;	
      }

      jpos[0] = (p3d_matrix_type)dtab[0];
      jpos[1] = (p3d_matrix_type)dtab[1];
      jpos[2] = (p3d_matrix_type)dtab[2];

      p3d_vectCopy(jpos,goal_jnt_coords[i]);
      
      // OPERATIONS FOR JNT VALUES
      // WARNINIG : this function only works with with 1dof joints !!!
/*     if(strcmp(fct,"p3d_set_dof")== 0) { */
/*       if(!read_desc_double(goal_p3d_f,1,dtab)) { */
/* 	printf("ERROR : load_ligand_goal_jnt_coordinates_from_p3d_file : error while reading file\n");     */
/* 	fclose(goal_p3d_f); */
/* 	bio_free_goal_jnt_coords(i-1); */
/* 	return 0;	 */
/*       } */

/*       goal_jnt_v[i] = dtab[0]; */

      i++;
    }
  }

  //printf("njoints = %d i = %d\n",robPt->njoints,i);
  if(i < n_lig_jnts) {
    printf("ERROR : load_ligand_goal_jnt_coordinates_from_p3d_file : goal p3d ligand model has different number of jnts\n");    
    fclose(goal_p3d_f);
    bio_free_goal_jnt_coords(i-1);
    return 0;	
  }

  fclose(goal_p3d_f);
  return 1;
}


///////////////////////////////////////////////

static int load_goal_atom_coordinates_from_p3d_file(void)
{
  p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  const char *file_name;
  char *file_name_s=NULL;
  char dir_name[512];
  FILE *goal_p3d_f;
  char  fct[256],nameatom[256],gc;
  double dtab[6];
  int i;
  p3d_vector3 apos;
  int n_atoms;
  p3d_jnt *jntPt;
  p3d_obj *objPt;
  //p3d_poly *polPt;

  //char *jnt_type_name;
  //int readnextcoords = 0;
  //int indexletter=0;
  
  // select file
  p3d_get_directory(dir_name);
  file_name = fl_show_fselector("Select file with goal structure",dir_name,"*.p3d","");
  if(file_name == NULL) {
    printf("ERROR : load_goal_atom_coordinates_from_p3d_file : no goal structure selected\n");
    return 0;
  }

  // open file
  file_name_s = strdup( file_name );
  if(!(goal_p3d_f = fopen(file_name_s,"r"))) {
    fl_show_alert("The file could not be opened !",
		  "File does not exist or you do not have the right to open it.",file_name,1);
    return 0;
  }

  // count n_atoms
  n_atoms = 0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     
    objPt = jntPt->o;
    if(objPt != NULL) {
      n_atoms += objPt->np;
    }
  }

  // memory allocation (suppose that both "robots" have the same number of atoms)
  goal_jnt_coords = MY_ALLOC(p3d_vector3, n_atoms);

  // read file and save information
  i = 0;
  while(fscanf(goal_p3d_f,"%s", fct) != EOF) {
    if(i>n_atoms) {
      printf("ERROR : load_goal_atom_coordinates_from_p3d_file : goal p3d has different number of atoms\n");    
      fclose(goal_p3d_f);
      bio_free_goal_jnt_coords(i-1);
      return 0;	
    }

    if(fct[0]=='#') {
      /* comment in file: ignore the line ! */
      do{
	gc=getc(goal_p3d_f);
      }
      while(gc!='\n');
      continue;
    }

    if(strcmp(fct,"p3d_set_prim_pos")== 0) {
      if(!read_desc_name(goal_p3d_f,nameatom)) {
	printf("ERROR : load_goal_jnt_coordinates_from_p3d_file : error while reading file\n");    
	fclose(goal_p3d_f);
	bio_free_goal_jnt_coords(i-1);
	return 0;	
      }
      if(!read_desc_double(goal_p3d_f,6,dtab)) {
	printf("ERROR : load_goal_atom_coordinates_from_p3d_file : error while reading file\n");    
	fclose(goal_p3d_f);
	bio_free_goal_jnt_coords(i-1);
	return 0;	
      }

      apos[0] = (p3d_matrix_type)dtab[0];
      apos[1] = (p3d_matrix_type)dtab[1];
      apos[2] = (p3d_matrix_type)dtab[2];

      p3d_vectCopy(apos,goal_jnt_coords[i]);
      
      i++;
    }
  }

  if(i<n_atoms) {
    printf("ERROR : load_goal_atom_coordinates_from_p3d_file : goal p3d has different number of atoms\n");    
    fclose(goal_p3d_f);
    bio_free_goal_jnt_coords(i-1);
    return 0;	
  }

  fclose(goal_p3d_f);
  return 1;
}


int bio_set_goal_jnt_coordinates(void)
{
  p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  // ONLY LIGAND //
  if(num_subrobot_ligand(robPt) == 0) {
    if(load_goal_atom_coordinates_from_p3d_file()) {
      // WARNING : VARIABLES "goal_jnt_coords" ARE ALSO USED FOR ATOM COORDS
      use_goal_jnt_coords = 1;
    }
    else {
      return 0;
    }
  }
  // PROTEIN //
  else {
    // ONLY PROTEIN
    if(is_ligand_in_robot(robPt) == FALSE){
      if(load_goal_jnt_coordinates_from_p3d_file()) {
	use_goal_jnt_coords = 1;
      }
      else {
	return 0;
      }
    }
    // PROTEIN + LIGAND
    else {
      if(load_ligand_goal_jnt_coordinates_from_p3d_file()) {
	use_goal_jnt_coords = 1;
      }
      else {
	return 0;
      }
    }
  }
  return 1;
}


int bio_get_goal_jnt_coordinates(p3d_vector3 **goal_jntcoordsPt)
//int bio_get_goal_jnt_coordinates(double **goal_jntvPt)
{
  if(use_goal_jnt_coords == 0)
    return 0;
  
  *goal_jntcoordsPt = goal_jnt_coords;
  //*goal_jntvPt = goal_jnt_v;

  return 1;

}


/**********************************************************************/

double bio_rmsd_to_goal_jnt_coords(p3d_rob *robPt, p3d_vector3 *goal_jntcoordsPt, configPt q)
//double bio_rmsd_to_goal_jnt_coords(p3d_rob *robPt, double *goal_jntvPt, configPt q)
{
  int i,j,k;  
  int n;
  p3d_jnt *jntPt;
  p3d_vector3 curpos,pos_diff;
  //double cur_vi,di;
  double sum_sqr_di,rmsd;
  double w=1.0, artic_ponderation = 50.0;
  p3d_obj *objPt;
  p3d_poly *polPt;
  
  n = 0;
  sum_sqr_di = 0.0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     

    // ONLY LIGAND //
    if(num_subrobot_ligand(robPt) == 0) {
      objPt = jntPt->o;
      if(objPt != NULL) {
	for(j=0;j<objPt->np;j++) {
	  polPt = objPt->pol[j];
	  for(k=0;k<3;k++) {
	    curpos[k] = polPt->poly->pos[k][3];
	  }    
	  p3d_vectSub(curpos,goal_jntcoordsPt[n],pos_diff);
	  w = 1.0;
	  sum_sqr_di += (w *(SQR((double)pos_diff[0]) + SQR((double)pos_diff[1]) + SQR((double)pos_diff[2])));
      	  n++;
	}
      }
    }
    // PROTEIN //
    else {
      // ONLY PROTEIN
      if(is_ligand_in_robot(robPt) == FALSE){
	// WARNING : only N positions !!!
	if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
	  // PRUEBA : RMSD LIPASE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	  //if(jntPt->bio_jnt_type == BIO_PSI_JNT) {
	  // PRUEBA : RMSD LINKER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	  /*     if((jntPt->bio_jnt_type == BIO_OMEGA_JNT) || */
	  /*        (jntPt->bio_jnt_type == BIO_PHI_JNT)) { */
	  //if(jntPt->type == P3D_ROTATE) {
	  // WARNING: suppose that q is the currenly "updated" configuration 
	  p3d_jnt_get_cur_vect_point(jntPt,curpos);
	  // WARNING: suppose that goal_jntcoordsPt only contains the coordinates of OMEGA jnts  
	  p3d_vectSub(curpos,goal_jntcoordsPt[n],pos_diff);
	  if(jntPt->bio_jnt_type == BIO_PHI_JNT) 
	    w = artic_ponderation;
	  else
	    w = 1.0;
	  sum_sqr_di += (w *(SQR((double)pos_diff[0]) + SQR((double)pos_diff[1]) + SQR((double)pos_diff[2])));      
	  n++;
	}
      }
      // PROTEIN + LIGAND
      else {
	if(jntPt->num >= robPt->joints[0]->next_jnt[robPt->joints[0]->n_next_jnt - 1]->num) {
	  // WARNING: suppose that q is the currenly "updated" configuration 
	  p3d_jnt_get_cur_vect_point(jntPt,curpos);
	  // WARNING: suppose that goal_jntcoordsPt only contains the coordinates of OMEGA jnts  
	  p3d_vectSub(curpos,goal_jntcoordsPt[n],pos_diff);
	  sum_sqr_di += (SQR((double)pos_diff[0]) + SQR((double)pos_diff[1]) + SQR((double)pos_diff[2]));      
	  n++;	  
	}
      }
    }
  }
  rmsd = sqrt(sum_sqr_di/((double)n));
  
  return (rmsd);  
}


/**********************************************************************************/
/**********************************************************************************/
/*        Functions concerning teh RMSD to the "init" conformation                */
/**********************************************************************************/
/**********************************************************************************/

// array to stock the joint coordinates of the "init" conformation
// NOTE: the element 0 corresponts to jnt 1
// - this array is allocated and filled in function load_jnt_coordinates
// - WARNING : memory is freed ???!!!
static p3d_vector3 *init_jnt_coords;
static int use_init_jnt_coords = 0;


void bio_free_init_jnt_coords(int n_alloc_elems)
{
  MY_FREE(init_jnt_coords,p3d_vector3,n_alloc_elems);  
}

// WARNING : this function only handles 1 jnt per residue (OMEGA)
static int load_jnt_coordinates(p3d_vector3 **jnt_coordsPt)
{
  p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  int i;
  int n,n_omegas;
  p3d_jnt *jntPt;
  p3d_vector3 curJpos;
  

  // count n_omegas
  n_omegas = 0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     
    // WARNING : only N positions !!!
    if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
      n_omegas++;
    }
  }

  // memory allocation (suppose that both "robots" have the same number of jnts)
  *jnt_coordsPt = MY_ALLOC(p3d_vector3, n_omegas);
  //init_jnt_v = MY_ALLOC(double, robPt->njoints);

  n=0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     
    // WARNING : only N positions !!!
    if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
      p3d_jnt_get_cur_vect_point(jntPt,curJpos);
      p3d_vectCopy(curJpos,(*jnt_coordsPt)[n]);
      n++;
    }
  }
   
  return 1;
}


int bio_set_init_jnt_coordinates(void)
{
  p3d_rob *robPt;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  
  // set and update the "init" conf
  p3d_set_robot_config(robPt, robPt->ROBOT_POS);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robPt);

  if(load_jnt_coordinates(&init_jnt_coords)) {
    use_init_jnt_coords = 1;
  }
  else {
    return 0;
  }
  return 1;
}


int bio_get_init_jnt_coordinates(p3d_vector3 **init_jntcoordsPt)
//int bio_get_init_jnt_coordinates(double **init_jntvPt)
{
  if(use_init_jnt_coords == 0)
    return 0;
  
  *init_jntcoordsPt = init_jnt_coords;
  //*init_jntvPt = init_jnt_v;

  return 1;

}


/**********************************************************************/

double bio_rmsd_to_init_jnt_coords(p3d_rob *robPt, p3d_vector3 *init_jntcoordsPt, configPt q)
//double bio_rmsd_to_init_jnt_coords(p3d_rob *robPt, double *init_jntvPt, configPt q)
{
  int i;  
  int n;
  p3d_jnt *jntPt;
  p3d_vector3 curJpos,pos_diff;
  //double cur_vi,di;
  double sum_sqr_di,rmsd;
  
  n = 0;
  sum_sqr_di = 0.0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     
    // WARNING : only N positions !!!
    if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
      // WARNING: suppose that q is the currenly "updated" configuration 
      p3d_jnt_get_cur_vect_point(jntPt,curJpos);
      // WARNING: suppose that init_jntcoordsPt only contains the coordinates of OMEGA jnts  
      p3d_vectSub(curJpos,init_jntcoordsPt[n],pos_diff);
      sum_sqr_di += (SQR((double)pos_diff[0]) + SQR((double)pos_diff[1]) + SQR((double)pos_diff[2]));      
      n++;
    }
  }
  rmsd = sqrt(sum_sqr_di/((double)n));

  return (rmsd);  
}


/**********************************************************************/
/**********************************************************************/

double bio_rmsd_between_confs(p3d_rob *robPt, configPt q1, configPt q2)
{
  int i;  
  int n;
  p3d_jnt *jntPt;
  p3d_vector3 curJpos,pos_diff;
  //double cur_vi,di;
  double sum_sqr_di,rmsd;
  p3d_vector3 *q1_jnt_coords;

  p3d_set_robot_config(robPt, q1);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robPt);

  if(!load_jnt_coordinates(&q1_jnt_coords)) {
    printf("ERROR : bio_rmsd_between_confs : can't load q1 coords\n");
    return 0;
  }

  p3d_set_robot_config(robPt, q2);
  p3d_update_this_robot_pos_without_cntrt_and_obj(robPt);

  n = 0;
  sum_sqr_di = 0.0;
  for(i=0;i<robPt->njoints;i++) {
    jntPt = robPt->joints[i+1];     
    // WARNING : only N positions !!!
    if(jntPt->bio_jnt_type == BIO_OMEGA_JNT) {
      p3d_jnt_get_cur_vect_point(jntPt,curJpos);
      // WARNING: suppose that q1_jnt_coords only contains the coordinates of OMEGA jnts  
      p3d_vectSub(curJpos,q1_jnt_coords[n],pos_diff);
      sum_sqr_di += (SQR((double)pos_diff[0]) + SQR((double)pos_diff[1]) + SQR((double)pos_diff[2]));      
      n++;
    }
  }
  rmsd = sqrt(sum_sqr_di/((double)n));

  MY_FREE(q1_jnt_coords,p3d_vector3,n);  

  return (rmsd);  
}


/**********************************************************************/
/**********************************************************************/

void bio_search_max_weight_in_curr_rrt(void)
{
  p3d_rob *robPt;
  p3d_graph *G;
  p3d_compco *CompPt;
  p3d_list_node *ListNode;
  double w,maxw=-1,minw=P3D_HUGE;
  p3d_node *Nmaxw=NULL;
  //  p3d_jnt *triade_jntPt[3];
  //  int triade_conftype;
  p3d_vector3 *goal_jntcoordsPt;
  //double *goal_jntvPt;
  p3d_jnt **pairs_jntPt;
  int nump;
  int conftype;
  int search_min = 0;
#ifdef ENERGY
  int n_coldeg;
#endif

  if(bio_get_goal_jnt_coordinates(&goal_jntcoordsPt)) {
    //if(bio_get_goal_jnt_coordinates(&goal_jntvPt)) {
    conftype = -1;
    search_min = 1;
  }
  /* WARNING : THE MAXIMUM OR MINIMUM WEIGHT IS SEARCHED 
               DEPENDING ON ONE ONLY TRIADE !!! */
  //  if(bio_get_triade(0,triade_jntPt,&triade_conftype)) {
  //    if(triade_conftype == -1) {
  else if(bio_get_pairs_for_dist(&nump,&pairs_jntPt,&conftype)) {
    if(conftype == -1) {
      search_min = 1;
    }
  }
  else {
    search_min = 0;
  }
  
  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  G = XYZ_GRAPH;
  CompPt = G->comp;    // ONLY FOR THE FIRST COMPONENT

  // compute max
  ListNode = CompPt->dist_nodes;
  while(ListNode != NULL) {
    w = ListNode->N->weight;
    if(search_min) {
      if((w != 0.0) && (w < minw)) {
	minw = w;
	Nmaxw = ListNode->N;
      }
    }
    else{
      if(w > maxw) {
	maxw = w;
	Nmaxw = ListNode->N;
      }
    }
    ListNode = ListNode->next;
  }
  if(Nmaxw != NULL) {
    if(search_min)
      printf("Minimum weight            = %f - for node num. %d\n",minw,Nmaxw->num);    
    else
      printf("Maximum weight            = %f - for node num. %d\n",maxw,Nmaxw->num);    
#ifdef ENERGY
    if(Nmaxw->coldeg_q != NULL) {
      n_coldeg = bio_get_num_collective_degrees();
      printf("ColDeg conf :\n");    
      bio_print_coldeg_config(Nmaxw->coldeg_q,n_coldeg);
    }
#endif
    // set farther confromation as GOAL
    p3d_copy_config_into(robPt,Nmaxw->q,&(robPt->ROBOT_GOTO));
    printf("ROBOT_GOTO updated to farther conf. in graph\n");
  }
  else {
    printf("There is a problem with the weights !!!\n");
  }
}


/**********************************************************************/
/**********************************************************************/
// WARNING: THIS FUNCTION CONSIDERS THAT THE 'ROBOT' CONTAINS ONLY ONE MOLECULE !!!
int bio_compute_molecule_center_of_mass(p3d_rob* robPt, p3d_vector3 CoM)
{
  int i,j,k;  
  p3d_jnt *jntPt;
  p3d_obj *objPt;
  p3d_poly *polPt;
  int a_type;
  double a_mass;
  p3d_vector3 a_pos,a_massdotpos;
  int n_atoms = 0;
  double mass_sum = 0.0;
  p3d_vector3 massdotpos_sum = {0.0,0.0,0.0};
  
  for(i=0;i<=robPt->njoints;i++) {
    jntPt = robPt->joints[i];     
    objPt = jntPt->o;
    if(objPt != NULL) {
      for(j=0;j<objPt->np;j++) {
	polPt = objPt->pol[j];
	a_type = (int)polPt->type;
	a_mass = bio_get_atom_mass_from_type(a_type);
	for(k=0;k<3;k++) {
	  a_pos[k] = polPt->poly->pos[k][3];
	}
	p3d_vectScale(a_pos,a_massdotpos,a_mass);	
	p3d_vectAdd(a_massdotpos,massdotpos_sum,massdotpos_sum);
	mass_sum += a_mass;
	n_atoms++;	
      }
    }
  }
  p3d_vectScale(massdotpos_sum,CoM,1.0/mass_sum);	
  
  return 1;
}


// WARNING: THIS FUNCTION CONSIDERS THAT THE 'ROBOT' CONTAINS ONLY ONE MOLECULE !!!
double bio_compute_molecule_radius_of_giration(p3d_rob* robPt)
{
  int i,j,k;  
  p3d_jnt *jntPt;
  p3d_obj *objPt;
  p3d_poly *polPt;
  int a_type;
  double a_mass;
  p3d_vector3 a_pos;
  int n_atoms = 0;
  double mass_sum = 0.0;
  double masssqrdist_sum = 0.0;
  double dist=0.0;
  p3d_vector3 pos_diff;
  p3d_vector3 CoM;
  double RoG;

  bio_compute_molecule_center_of_mass(robPt,CoM);
  // PRUEBA !!!!!!!!
  //printf("\n CoM =");
  //for(k=0;k<3;k++) {
  //  printf(" %f  ",CoM[k]);
  //}
  //printf("\n");
  ////////////////////  
  
  for(i=0;i<=robPt->njoints;i++) {
    jntPt = robPt->joints[i];     
    objPt = jntPt->o;
    if(objPt != NULL) {
      for(j=0;j<objPt->np;j++) {
	polPt = objPt->pol[j];
	a_type = (int)polPt->type;
	a_mass = bio_get_atom_mass_from_type(a_type);
	//a_mass = 1.0;
	for(k=0;k<3;k++) {
	  a_pos[k] = polPt->poly->pos[k][3];
	}
	p3d_vectSub(a_pos,CoM,pos_diff);
	dist = (double) p3d_vectNorm(pos_diff);
	masssqrdist_sum += (a_mass * dist * dist);
	mass_sum += a_mass;
	n_atoms++;	
      }
    }
  }
  RoG = sqrt(masssqrdist_sum/mass_sum);    

  return(RoG);
}


/***************************************************************************/
/* function for that progressively generates a collision-free conformation */
/* WARNING : supposes that the model contains a single sequence of residues*/
/* NOTE : move to a new file bio_sample.c ???                              */
/***************************************************************************/
 
p3d_node* bio_shoot_free_conf(p3d_graph *graphPt)
{
  p3d_rob *robotPt = graphPt->rob;
  p3d_jnt *Jct,*fJbkb,*lJbkb;;
  int fres, lres, ires, fres_ct;
  int i;
  configPt q;
  int tries_bkb, tries_onebkb, tries_allsch, tries_onesch;
  int procOK, procOK2;
  int array_sch[500]; // WARNING : max. loop size limited at 500 residues !
  int ordered_array_sch[500];
  int index_arr;
  int num_sch;
  Joint_tablespt *jnt_table;
  p3d_node *nodePt;
  p3d_cntrt *ct;

  // PARAMETERS ******
  int NALLBKB = 50;
  int NONEBKB = 10;
  int NALLSCH = 100;
  int NONESCH = 100;
  // *****************

  // init q
  q = p3d_alloc_config(robotPt);
  // sample conf (for dofs that are not involved in the flexible sequence)
  p3d_shoot(robotPt,q,0);
  // excepted the loop, all the joints have constant value
  //p3d_copy_config_into(robotPt, p3d_get_robot_config(robotPt), &q);

  // function does not admit cntrts yet
  // WARNING : ONLY WORKS WITH BKB H-BONDS !!!
/*   if((robotPt->cntrt_manager != NULL) && (robotPt->cntrt_manager->ncntrts != 0)) { */
/*     printf("ERROR : bio_shoot_free_conf : function does not admit constrains yet\n"); */
/*     p3d_destroy_config(robotPt,q);     */
/*     return NULL; */
/*   } */

  // identify first residue  
  i = 1;
  while(robotPt->joints[i]->bio_jnt_type != BIO_PHI_JNT)
    i++;
  fres = get_AAnumber_from_jnt(robotPt->joints[i]);
  fres_ct = fres;

  // get "joint table"
  jnt_table = give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,fres));
 
  // identify last residue  
  lres = get_AAnumber_from_jnt(robotPt->joints[robotPt->njoints - 1]);
/*   while(jnt_table[lres]->n_bb_joints < 3) */
/*     lres--; */

  // ordered array of indices of resudues with side-chain  
  i = 0;
  num_sch = 0;
  while((fres + i) <= lres) {
    if(jnt_table[fres + i]->n_sc_joints > 0) {
      ordered_array_sch[num_sch] = jnt_table[fres + i]->namino;  // must be ==  fres + i
      num_sch ++;
    }
    i ++;
  }      

  // deactivate collisions of (mobile) side-chains and backbone
  supress_sc_rigid_sequence(num_subrobot_AA_from_AAnumber(robotPt,fres), fres, lres);
  supress_bb_rigid_sequence(num_subrobot_AA_from_AAnumber(robotPt,fres), fres, lres);

  // generate backbone
  procOK = 0;
  tries_bkb =0;
  ires = fres;
  while(!procOK && (tries_bkb < NALLBKB)) {    
    tries_bkb ++;
    ires = fres_ct;
    procOK2 = 1;
    while(procOK2 && ires <= lres) {
      // activate collision of this backbone
      activate_bb_rigid(num_subrobot_AA_from_AAnumber(robotPt,fres), ires);
      tries_onebkb = 0;
      procOK2 = 0;
      while(!procOK2 && (tries_onebkb < NONEBKB)) { 
	tries_onebkb ++;  
	// generate this side-chain conformation and check collision
	procOK2 = bio_generate_one_bkb_conf_and_checkcoll(robotPt,ires,jnt_table,q);
	//procOK2 = bio_generate_one_bkb_conf_ramachandran_and_checkcoll(robotPt,ires,jnt_table,q);
	// if bkb conf cannot be generated because of RLG then exit
	if(procOK2 == -2) {
	  tries_onebkb = NONEBKB;
	}
      }
      if(procOK2) {
	// check H-bonds
	fJbkb = jnt_table[ires]->bb_joints[0];
	lJbkb = jnt_table[ires]->bb_joints[jnt_table[ires]->n_bb_joints - 1];
	if(robotPt->cntrt_manager->cntrts != NULL) {
	  for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	       dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); 
	       dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
	    ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
	    if((strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) && (ct->active)) {
	      Jct  = robotPt->joints[ct->argu_i[1]];
	      if((Jct->num >= fJbkb->num) && (Jct->num <= lJbkb->num)) {
		if(!(*ct->fct_cntrt)(ct,-1,NULL,0.0)) {
		  procOK2 = 0;
		  //printf("H-bond not satisfied : ct num = %d\n",ct->num);
		}
		else {
		  fres_ct = ires+1;
		  printf("H-bond OK : ct num = %d\n",ct->num);
		}
	      }
	    }
	  }
	}
      }	  
      // next residue
      ires ++;
    }
    procOK = procOK2 && (ires > lres);
    if(!procOK) {
      // deactivate collisions of backbone
      supress_bb_rigid_sequence(num_subrobot_AA_from_AAnumber(robotPt,fres), fres_ct, lres);
    }
    graphPt->nb_q += 1; 
  }

  if(procOK) {
    graphPt->nb_bkb_q_free += 1;
    //printf("\n\n BACKBONE OK !!!!!!!!!!!!!!!!!\n\n");
  }
  else {
    p3d_destroy_config(robotPt,q);    
    return NULL;
  }  

  // generate side-chains
  tries_allsch = 0;
  procOK = 0;   
  while(!procOK && (tries_allsch < NALLSCH)) {    
    tries_allsch ++;  
    // init array of side-chains with random order
    bio_init_array_sch(array_sch, ordered_array_sch, num_sch);
    index_arr = 0;
    procOK2 = 1;
    while(procOK2 && index_arr < num_sch) {
      // activate collision of this side-chain
      activate_sc_rigid(robotPt->num, array_sch[index_arr]);
      tries_onesch = 0;
      procOK2 = 0;
      while(!procOK2 && (tries_onesch < NONESCH)) {    
	tries_onesch ++;  
	// generate this side-chain conformation and check collision
	procOK2 = bio_generate_one_sch_conf_and_checkcoll(robotPt, array_sch[index_arr], jnt_table, q);
      }
      index_arr ++;
    }
    procOK = procOK2 && (index_arr == num_sch);
    if(!procOK) {
      // deactivate collisions of (mobile) side-chains
      supress_sc_rigid_sequence(robotPt->num, fres, lres);
    }
  }

  if(procOK) {
    graphPt->nb_q_free += 1;
    printf("%d VALID CONFORMATIONS\n",graphPt->nb_q_free);
    nodePt = p3d_APInode_make(graphPt,q);
    //p3d_print_node(graphPt,nodePt);
    return(nodePt);
  }
  else {
    p3d_destroy_config(robotPt,q);    
    return NULL;
  }
}


/**
 * p3d_ComputeVdWMaxConf
 * Compute the maximal value of the Van 
 * der waals Radius for which the configuration is
 * valid
 * @param[In]: the given configuration
 * @return: the maximal VdW value valid for the conf 
 * (between 0 and 1)
 */
double p3d_ComputeVdWMaxConf(configPt q) {
  int isInvalidConf = TRUE;
  double vdwCurrent =1.;
  int indCurrRobot = p3d_get_desc_curnum(P3D_ROBOT);
  double vdwSaved =  GetPrevVdw();

  p3d_set_and_update_robot_conf(q);

  bio_resize_molecules(vdwCurrent/GetPrevVdw());
  bio_true_resize_molecules();
  SetPrevVdw(vdwCurrent);
  p3d_col_test_all();
  isInvalidConf = biocol_robot_report(indCurrRobot);
  while((isInvalidConf == TRUE)&&(vdwCurrent>0)) {
    bio_resize_molecules(vdwCurrent/GetPrevVdw());
    bio_true_resize_molecules();
    SetPrevVdw(vdwCurrent);
    p3d_col_test_all();
    isInvalidConf = biocol_robot_report(indCurrRobot);
    vdwCurrent =vdwCurrent -0.01;
  }
  bio_resize_molecules(vdwSaved/GetPrevVdw());
  bio_true_resize_molecules();
  SetPrevVdw(vdwSaved); 
  return vdwCurrent +0.01;
}

