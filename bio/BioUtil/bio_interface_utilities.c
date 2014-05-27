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
 * SimÃ©on, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Bio-pkg.h"

ligand *LIGAND_PT = NULL;

int get_AAtotal_number(p3d_rob* robotPt);
p3d_jnt** get_list_firstjnts_flexible_sc(p3d_rob* robotPt);
int get_nb_flexible_sc(p3d_rob* robotPt);
p3d_jnt *get_AAfirstjnt(int AAnumber);
int get_AAnumber_from_jnt (p3d_jnt* jnt);
int get_AAnumber_from_name(char *name);
void get_AAtype_from_name(char *name, char AAtype[4]);
int get_AAtype_from_AAnumber(int AAnumber, char AAtype[4]);
int get_AAfirstjnt_number(int AAnumber);
int next_joint(p3d_jnt **Outjnt, p3d_jnt **Injnt, int in_branch);
int nb_dof_ligand(p3d_rob* robotPt);
int num_subrobot_ligand(p3d_rob* robotPt);
int num_subrobot_AA_from_AAnumber(p3d_rob* robotPt,int AAnumber);
int is_ligand_in_robot(p3d_rob* robotPt);
int bio_copy_sch_conf(p3d_rob *robPt, configPt q_src, configPt q_dst);
int bio_copy_ligand_conf(p3d_rob *robPt, configPt q_src, configPt q_dst);
int make_rigid_AA_jnt(p3d_jnt* firstAAjnt);
int make_rigid_AA_num(int num); 
int make_flexible_AA_jnt(p3d_jnt* firstAAjnt); 
int make_flexible_AA_num(int num); 
int bio_get_position_meaning_frame(p3d_rob* robotPt, p3d_vector3 pos);
int update_ligand_bounding_box(p3d_rob* robotPt);
int update_sidechain_BBox_from_firstjnt(p3d_rob* robotPt, p3d_jnt* firstAAjnt);
/*int update_sidechain_BBox_from_AAnumber(p3d_rob* robotPt, int  AAnumber);*/
int update_all_sidechain_BBoxes(p3d_rob* robotPt);
int dist_ligand_sidechainBBoxes(p3d_rob* robotPt, int  AAnumber, double * dist_vector);
void PrintInfo_BBoxesdist(p3d_rob* robotPt, int  AAnumber);
void bio_set_current_u_inv(double u_inv);
int bio_get_pdb_atom_number(p3d_poly* poly); // Added by Ron


/*void test_perf(void); */
/*void changer_rigidite(int numero, int nature);*/
/*int is_AA_rigid(int numero, int nature);*/



int bio_get_pdb_atom_number(p3d_poly* poly) {

	char* atom_number_start;
	char* atom_number_end;
	const int invalid_atom_number = -1;
	char atom_number_str[256];

//	printf("name: %s;", poly->poly->name);

	// Atom starts after first '.' char
	atom_number_start = strchr(poly->poly->name, '.');
	// The atom number is invalid if there is no '.' char or if the string is empty after the '.' char
	if( atom_number_start == NULL || atom_number_start[1] == '\0' )
		return invalid_atom_number;

	// Move the pointer past the starting '.' char so it points to the start of the string of the number
	atom_number_start++;

	// Atom number ends before second '.' char or at end of string
	atom_number_end = strchr(atom_number_start, '.');

	if( atom_number_end == NULL )
		strcpy(atom_number_str, atom_number_start);
	else
		strncpy(atom_number_str, atom_number_start, strlen(atom_number_start) - strlen(atom_number_end));

//	printf("num: %s;", atom_number_str);
	return atoi(atom_number_str);
}





/************************************************************/
/* \brief:  get the atom mass from the atom type            */
/* \return: the atom mass, or -1 of unknown atom type       */
/************************************************************/
double bio_get_atom_mass_from_type(int a_type)
{
  switch(a_type) {
  case CARBON: 
    return(C_MASS);
  case OXYGEN:
  case OXYGEN_H:
    return(O_MASS);
  case NITROGEN:
  case NITROGEN_H:
  case NITROGEN_FULL:
    return(N_MASS);
  case HYDROGEN:
    return(H_MASS);
  case SULPHUR:
  case SULPHUR_H:
    return(S_MASS);
  case PHOSPHORUS:
    return(P_MASS);
  case BROMINE:
    return(Br_MASS);
  case IODINE:
    return(I_MASS);
  case FLUORINE:
    return(F_MASS);
  case CHLORINE:
    return(Cl_MASS);
  default:
    printf("ERROR : bio_get_atom_mass_from_type : undefined atom type");
    return(-1.0);
  }
}

/************************************************************/
/* \brief:  get the total number of aminoacid of the loaded */
/*          file                                            */
/* \return: the number of aminoacid in the protein          */
/************************************************************/
int get_AAtotal_number(p3d_rob* robotPt)
{
  return robotPt->nbAA;
}



/************************************************************/
/* \brief:  compute a list of first joints of all flexible  */
/*          side chains.                                    */
/* \parm robotPt: the robot                                 */
/* \return: a list containing first joints of all flexible  */
/*          side chains.                                    */
/* \note: the list must be desallocated outside             */
/*          function.                                       */
/* \note: Warning, the search of flexible sidechains is     */
/*        on the existance of psi joint. Maybe for some     */
/*        cinematic structures the function doesnot work !  */
/************************************************************/
p3d_jnt** get_list_firstjnts_flexible_sc(p3d_rob* robotPt ) {
  return robotPt->list_firstjnts_flexible_sc;
}


/************************************************************/
/* \brief: compute the number of flexible side chains of    */
/*         the robot.                                       */
/* \parm robotPt: the robot                                 */
/* \return: the number of flexible side chains of the robot */
/* \note: Warning, the search of flexible sidechains is     */
/*        on the existance of psi joint. Maybe for some     */
/*        cinematic structures the function doesnot work    */
/************************************************************/
int get_nb_flexible_sc(p3d_rob* robotPt) {

  return    robotPt->nb_flexible_sc;
}


/**************************************************************/
/* \brief:  get the first joint of an acidoamid               */
/* \param AAnumber: the number of the AA, as it is described  */
/*                  in its name                               */
/* \return: a pointer on the first joint of the acidoamid     */
/*          with the number "AAnumber" and NULL if it doesnot */  
/*           exist                                            */
/**************************************************************/
/* p3d_jnt *get_AAfirstjnt(int AAnumber) */
/* { */
/*   int i = 0; */
/*   int found = 0; */
/*   while(i <XYZ_ROBOT->no) { */
/*     i++; */
/*     if(get_AAnumber_from_name(XYZ_ROBOT->o[i-1]->name) == AAnumber) */
/*       { */
/* 	found = 1; */
/* 	i--; */
/* 	return XYZ_ROBOT->o[i]->jnt; */
/*       }      */
/*   } */
/*   return NULL; */
/* } */

p3d_jnt *get_AAfirstjnt(int AAnumber) {
  return XYZ_ROBOT->list_AA_firstjnt[AAnumber];
}


/**************************************************************/
/* \brief: get the AAnumber of an acidoamid with joint        */
/*         jnt                                                */ 
/* \param jnt: one  joint of the acidoamid                    */
/* \return: the number of the AA, as it is described          */
/*                  in its name                               */
/**************************************************************/
int get_AAnumber_from_jnt (p3d_jnt* jntPt) {
  if (jntPt == NULL) {
    return -1;
  }
  return jntPt->bio_AAnumber;
}

/*************************************************************/
/* \brief: get the number of an acidoamid as it is described */
/*                  in its name                              */
/* \param name: the global name of the AA                    */
/*                  (e.g. : all-atoms.GLU.28.0 returns 28)   */
/* \return: the number of the AA                             */
/*************************************************************/
int get_AAnumber_from_name(char *name)
{
  char numAA[7]; /* we are limited to molecules with 9 millions of AA ... */
  char AAtype[4];
  int i=0;
  int j=0;

  get_AAtype_from_name(name, AAtype);
  if((strcmp(AAtype,"cha")==0) || (strcmp(AAtype,"pro")==0)) {
    return -1;
  }
  while(name[i] != '.'){
    if(name[i]== '\0') return -1;
    i++;
  }
  i++;
  while(name[i] != '.'){
    if(name[i]== '\0') return -1;

    i++;}
  /* i is on the second '.' */ 
  i++;
  while((name[i] !='\0' )&&(name[i] != '.') ){
    numAA[j] = name[i];
    i++;
    j++;
  }
  numAA[j] = '\0';
  return(atoi(numAA)); 
}

/*************************************************************/
/* \brief: get the type of AA (among the 20 types of AA) as  */
/*         it is described  in its name  or LIG if it is a   */
/*         ligand.                                           */
/*         (e.g. : all-atoms.GLU.28.0 returns GLU)           */
/* \param name : the global name of the AA                   */
/* \retval AAtype : the type of AA                           */
/*************************************************************/
void get_AAtype_from_name(char *name, char AAtype[4])
{
  char AAlname[4];
  int i = 0;
  while(name[i] != '.'){i++;}
  /* the name of the AA have only 3 characters */
   AAlname[0] = name[i+1]; 
   AAlname[1] = name[i+2]; 
   AAlname[2] = name[i+3]; 
   AAlname[3] = '\0';
   strcpy(AAtype,AAlname); 
}



/***************************************************************/
/* \brief: get the type of AA (among the 20 types of AA) as    */
/*         it is described  in its name. Should fail with  a   */
/*         ligand.                                             */
/* \param AAnumber : the number of an AA as it is described      */
/*                  in its name                                */
/* \retval AAtype : the type of AA                             */
/* \return 1 if the AA with the number 'number' has been found */
/*         0 otherwise                                         */
/***************************************************************/
int get_AAtype_from_AAnumber(int AAnumber, char AAtype[4])
{
  char AAtypel[4];
  int i = 0;
  int j = 0;
  int found = 0;
  while((i <XYZ_ENV->cur_robot->no)&&(found==0))
    {
      if(get_AAnumber_from_name(XYZ_ENV->cur_robot->o[i]->name) == AAnumber)
	{
	  while(XYZ_ENV->cur_robot->o[i]->name[j] != '.'){j++;}
	  AAtypel[0] = XYZ_ENV->cur_robot->o[i]->name[j+1]; 
	  AAtypel[1] = XYZ_ENV->cur_robot->o[i]->name[j+2]; 
	  AAtypel[2] = XYZ_ENV->cur_robot->o[i]->name[j+3]; 
	  AAtypel[3] = '\0';
	  strcpy(AAtype,AAtypel); 
	  found = 1;
	}
      i++;
    }
  return found;
}


/***************************************************************/
/* \brief: get the number of the first joint of the AA         */
/* \param AAnumber : the number of an AA as it is described    */
/*                  in its name                                */
/* \return the number of the first joint of the AA. 0 if joint */
/*          has not been found                                 */
/***************************************************************/
int get_AAfirstjnt_number(int AAnumber) {
  p3d_jnt* jntPt = get_AAfirstjnt(AAnumber);
  if (jntPt != NULL) {
    return jntPt->num;
  }
  return 0;
}

/*old implementation of get_AAfirstjnt_number function*/
/* { */
/*   int i = 0; */
/*   int found = 0; */

/*   while((i<XYZ_ROBOT->njoints)&&(!found)) */
/*     { */
/*       i++; */
/*       if( get_AAnumber_from_name(XYZ_ROBOT->joints[i-1]->name) == AAnumber)  */
/* 	{ */
/* 	  found = 1; */
/* 	  i--; */
/* 	} */
/*     } */
/*   return i; */
/* } */

/***************************************************************/
/* \brief: get the next joint to explore when we explore the   */
/*         tree of cinematic joints.                           */
/* \param Injnt: the current joint                             */
/*        in_branch: the position in width of the current      */
/*                    explored branch  (to verify !)           */
/* \retval Outjnt: the next joint to explore                   */
/* \return the position in width of the branch of the Outjnt   */
/*         in the cinematic tree                               */
/***************************************************************/
int next_joint(p3d_jnt **Outjnt, p3d_jnt **Injnt, int in_branch)
{
  int aRenvoyer = 0;

  /* cas ou l'on a atteint le joint correspondant a l'angle psi */
  /* on sait alors que l'on est au bout de l'acide amine */
  if((((*Injnt))->name[0] == 'p')&&(((*Injnt))->name[1] == 's')) 
    {
      aRenvoyer = -1;
    }
  /* dans le cas ou l'on est au milieu de la chaine lat */
  else 
    {
      if(in_branch > 1)
	{
	  if((*Injnt)->n_next_jnt == 1 )
	    {
	      *Outjnt = *((*Injnt)->next_jnt);
	      aRenvoyer = in_branch;
	    }
	  else 
	    {
	      if((*Injnt)->n_next_jnt == 0 )
		{
		  while((*Injnt)->n_next_jnt != 2)
		    {
		      (*Injnt) = (*Injnt)->prev_jnt;
		    }
		  *Outjnt = (*Injnt)->next_jnt[in_branch-2+1];
		  aRenvoyer = in_branch + 1;
		}
	      else 
		{
		  if((*Injnt)->n_next_jnt > 1)
		    {
		      *Outjnt = (*Injnt)->next_jnt[0];
		      aRenvoyer = 2;
		    }
		}
	    }
	}
      else
	{
	  if((*Injnt)->n_next_jnt == 1 )
	    {
	      *Outjnt = (*Injnt)->next_jnt[0];
	      aRenvoyer = 1;
	    }
	  else 
	    {
	      if((*Injnt)->n_next_jnt == 0 )
		{
		  aRenvoyer = 0;
		}
	      if((*Injnt)->n_next_jnt > 1)
		{
		  *Outjnt = (*Injnt)->next_jnt[0];
		  aRenvoyer = 2;
		}   
	    }
	}
    }
  return aRenvoyer;
}


/***************************************************************/
/* \brief: get the number of ligand dofs                       */
/* \return the number of ligand dofs                           */
/***************************************************************/
int nb_dof_ligand(p3d_rob* robotPt)
{
  return robotPt->nb_ligand_dof;
}


/***************************************************************/
/* \brief: get the  number equal to the position of the        */
/*         subrobot ligand in the robot cinematic              */
/*         the ligand is the last branch on which point the    */
/*         prot_base joint. The other branch are chain_bases   */
/* \return the  number equal to the position of the            */
/*         subrobot ligand in the robot cinematic              */
/*         -1 if there is no ligand                            */
/***************************************************************/
int num_subrobot_ligand(p3d_rob* robotPt)
{
  return robotPt->num_subrobot_ligand;
}


/***************************************************************/
/* \brief: get the number of the subrobot of the AA AAnumber.  */
/*         In the cinematic chain, sub robots are separated by */
/*         chain_base joints.                                  */
/* \param robotPt: the robot                                   */
/* \param AAnumber: the number of the AA as it is described in */
/*        its name                                             */
/* \return the  number of the AA subrobot in the robot         */
/*         cinematic                                           */
/*         -1 if the AA has not been found                     */
/***************************************************************/
int num_subrobot_AA_from_AAnumber(p3d_rob* robotPt,int AAnumber) {
  int joint_number=0, num_subrobot =-1, cur_AAnumber;
  p3d_jnt* cur_jnt;
  char AAtype[4]; 
  //  int num_jnt = robotPt->joints[0]->n_next_jnt;  // nb de joints sous la P3D_BASE
  //  if(num_jnt < 2) return -1; 
  //num_jnt =0
  while(joint_number<robotPt->njoints) {
    cur_jnt = robotPt->joints[joint_number];
    cur_AAnumber = get_AAnumber_from_jnt(cur_jnt);
    if(cur_AAnumber == AAnumber) {
      return  num_subrobot;
    }
    get_AAtype_from_name(cur_jnt->name,AAtype);     
    if(strcmp(AAtype,"cha")==0) {//the joint is a chain_base
      num_subrobot++;
    }
    joint_number++;
  }
  return -1;
}

int num_subrobot_AA_from_AAjnt(p3d_jnt* jntPt) {
  if (jntPt == NULL) {
    return -1;
  }
  return jntPt->num_subrobot;
}


/***************************************************************/
/* \brief:return if there is a ligand in the robot structure   */
/* \return 1 if there is a ligand                              */
/*         0 otherwise                                         */
/***************************************************************/
int is_ligand_in_robot(p3d_rob* robotPt)
{ 
  if(num_subrobot_ligand(robotPt) > 0) {
    return TRUE;
  }
  return FALSE;   
}



/*******************************************************************/
/* \brief: copy the side chain dofs of a configuration into another*/
/*         configuration                                           */
/* \param robPt: the robot                                         */
/* \param q_src: the source configuration from which we copy the   */
/*               side chain parameters                             */
/* \retval q_dst: destination configuration with side chains       */
/*                parametrers equal to those of q_src. Other param */
/*                are kept inchanged.                              */    
/* \return 1                                     		   */
/*******************************************************************/
int bio_copy_sch_conf(p3d_rob *robotPt, configPt q_src, configPt q_dst)
{
  int i;
  p3d_jnt *jntPt;

  for (i=0; i <= robotPt->njoints; i++){
    jntPt = robotPt->joints[i];
    if(jntPt->bio_jnt_type == BIO_GAMMA_JNT) {
      q_dst[jntPt->index_dof] = q_src[jntPt->index_dof];      
    }
  }
  return 1;
}


/*******************************************************************/
/* \brief: copy the ligand dofs of a configuration into another    */
/*         configuration                                           */
/* \param robPt: the robot                                         */
/* \param q_src: the source configuration from which we copy the   */
/*               ligand dofs                                       */
/* \retval q_dst: destination configuration with ligand dofs       */
/*                equal to those of q_src. Other param             */
/*                are kept inchanged.                              */    
/* \return:  1   if the ligand dofs have been found, 0 otherwise   */
/*******************************************************************/
int bio_copy_ligand_conf(p3d_rob *robotPt, configPt q_src, configPt q_dst)
{
  int i, j, k;
  p3d_jnt *jntPt;
  int nb_copy =0;
  if(is_ligand_in_robot(robotPt) ==FALSE){
    PrintInfo(("Warning: try to copy ligand dofs whereas there is no ligand\n"));
    return 0;
  }

  for (i=robotPt->joints[0]->next_jnt[1]->num; i <= robotPt->njoints; i++){
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if(jntPt->bio_jnt_type == BIO_OTHER_JNT) {
	q_dst[k] = q_src[k];
	nb_copy++;     
      }
      else {
	PrintInfo(("Warning: try to copy ligand dofs whereas there is no ligand\n"));
	return 0;
      }
    }
  }
  if(nb_copy !=  nb_dof_ligand(robotPt)) {
    PrintInfo(("Error : %d copies of ligand dof "
	       "whereas the ligand has %d dofs \n",nb_copy, nb_dof_ligand(robotPt)));
    return 0;
  }
  return 1;
}





/************************************************************/
/* \brief:  make an AA or one of its portion rigid          */
/*         (from its jnt)                                   */
/*          The function uses the names given to            */
/*          the angles of a flexible AA                     */
/* \param firstAAjnt: the first joint to make rigid.        */
/*                    Classicaly, should be gamma1          */
/* \return: 1 if the "rigidification" has been done         */
/*          0 otherwise                                     */
/* \note:  do not check if the AA is already rigid          */
/************************************************************/
int make_rigid_AA_jnt(p3d_jnt* firstAAjnt)
{
  p3d_jnt* cur_jnt;
  int jnt_number = firstAAjnt->num;
  //we go to the second joint to consider the side chain :
  jnt_number++;
  cur_jnt = XYZ_ROBOT->joints[jnt_number];
  //if the second joint is phi, we we consider the next joint
  if((cur_jnt != NULL)&&  (cur_jnt->name[1]=='h')){
    jnt_number++;
    cur_jnt = XYZ_ROBOT->joints[jnt_number];
  }
  if((cur_jnt == NULL) || (cur_jnt->name==0)||
     cur_jnt->name[0]!='g') {
    PrintInfo(("Warning: failed to make AA_rigid \n"));
    return 0;
  }
  while(cur_jnt->name[0]=='g') {
    p3d_jnt_set_dof_bounds(cur_jnt,0,cur_jnt->dof_data[0].v,cur_jnt->dof_data[0].v);
    p3d_jnt_set_dof_rand_bounds(cur_jnt,0,cur_jnt->dof_data[0].v,cur_jnt->dof_data[0].v);
    cur_jnt = XYZ_ROBOT->joints[(cur_jnt->num)+1];
  }
  return 1;
}


/************************************************************/
/* \brief:  make an AA  rigid (based on its num)            */
/*          The function is based on the names given to     */
/*          the angles of a flexible AA                     */
/* \param num:  the number of the AA, as it is described    */
/*                  in its name                             */
/* \return: 1 if the "rigidification" has been done         */
/*          0 otherwise                                     */
/* \note:  do not check if the AA is already rigid          */
/************************************************************/
int make_rigid_AA_num(int num)
{
  p3d_jnt* firstAAjnt = get_AAfirstjnt(num);
  return make_rigid_AA_jnt(firstAAjnt);
}


/************************************************************/
/* \brief:  make an AA or one of its portion flexible       */ 
/*         (from its jnt)                                   */
/*          The function uses the names given to            */
/*          the angles of a flexible AA                     */
/* \param firstAAjnt: the first joint to make flexible.     */
/*                    Classicaly, should be gamma1          */
/* \return: 1 if the "flexibilisation" has been done        */
/*          0 otherwise                                     */
/* \note:  do not check if the AA is already flexible       */
/************************************************************/
int make_flexible_AA_jnt(p3d_jnt* firstAAjnt)
{
  p3d_jnt* cur_jnt;
  int jnt_number = firstAAjnt->num;
  //we go to the second joint to consider the side chain :
  jnt_number++;
  cur_jnt = XYZ_ROBOT->joints[jnt_number];
  //if the second joint is phi, we we consider the next joint
  if((cur_jnt != NULL)&&  (cur_jnt->name[1]=='h')){
    jnt_number++;
    cur_jnt = XYZ_ROBOT->joints[jnt_number];
  }
  if((cur_jnt == NULL) || (cur_jnt->name==0)||
     cur_jnt->name[0]!='g') {
    PrintInfo(("Warning: failed to make AA_flexible \n"));
    return 0;
  }
  while(cur_jnt->name[0]=='g') {
    p3d_jnt_set_dof_bounds(cur_jnt,0,-M_PI,M_PI);
    p3d_jnt_set_dof_rand_bounds(cur_jnt,0,-M_PI,M_PI);
    cur_jnt = XYZ_ROBOT->joints[(cur_jnt->num)+1];
  }
  return 1;
}

/************************************************************/
/* \brief:  make an AA  flexible (based on its num)         */
/*          The function is based on the names given to     */
/*          the angles of a flexible AA                     */
/* \param num:  the number of the AA, as it is described    */
/*                  in its name                             */
/* \return: 1 if the "flexibilisation" has been done        */
/*          0 otherwise                                     */
/* \note:  do not check if the AA is already flexible       */
/************************************************************/
int make_flexible_AA_num(int num)
{
  p3d_jnt* firstAAjnt = get_AAfirstjnt(num);
  return make_flexible_AA_jnt(firstAAjnt);
}


/************************************************************/
/* \brief: get the position of a specific frame linked to a */
/*         joint of the robot.                              */
/* \param robotPt: the robot                                */
/* \retval pos: the X,Y,Z positions of the 'meaning frame'  */
/* \return:  1 if it finds a meaning frame 0 otherwise      */
/* \note: If the user has specified through the planner     */
/*        interface a joint to draw, the meaning frame is   */
/*        the position of this joint. Otherwise, it is the  */
/*        joint of the last free flying joint of the robot  */
/*        which is the joint of the ligand if it exists     */
/************************************************************/ 
int bio_get_position_meaning_frame(p3d_rob *robotPt, p3d_vector3 pos)
{
  p3d_jnt *jntPt=NULL;
  int i;
  int indexjnt=0;

  indexjnt = p3d_get_user_drawnjnt();
  if(indexjnt == -1) {
    // if there is a ligand
    if(num_subrobot_ligand(robotPt)>0) {
      // identify ligand freeflying jnt
      // WARNING :
      // - suppose that it is the last freeflying jnt in the robot
      // - suppose that it is placed at the baricenter if the ligand
      for(i=0;i<=robotPt->njoints;i++) {
	if(robotPt->joints[i]->type == P3D_FREEFLYER)
	  jntPt = robotPt->joints[i];
      }
    }
    else {
      printf("ERROR: No jnt has been defined for measuring position\n");
      return 0;
    }
  }
  else {
    // same joint than the one used for drawing the graph
    jntPt = robotPt->joints[indexjnt];
  }

  p3d_jnt_get_cur_vect_point(jntPt,pos);

  return 1;
}
/************************************************************/
/* \brief: update the value of the axes oriented bounding   */
/*         box around the ligand.                           */
/* \return:  1 if an update has been done 0 otherwise       */
/*  note: the structure updated is LIGAND_PT->ligBBox       */
/************************************************************/ 
int update_ligand_bounding_box(p3d_rob* robotPt) {
  int num_jnt = robotPt->joints[0]->next_jnt[1]->num;
  int is_first_jnt = TRUE;
  p3d_jnt* lig_jntPt = NULL;
  double xmin_jnt, xmax_jnt, ymin_jnt, ymax_jnt, zmin_jnt, zmax_jnt;

  if(is_ligand_in_robot(robotPt)==FALSE ) {
    PrintInfo(("Warning: no ligand was found for the BB computation \n"));
    return 0;
  }
  if( LIGAND_PT==NULL ) {
    LIGAND_PT = MY_ALLOC(ligand, 1); 
  }

  while(num_jnt < robotPt->njoints +1) {
    lig_jntPt = robotPt->joints[num_jnt];
    if(is_first_jnt == TRUE) {
      LIGAND_PT->ligBBox.min[0] = lig_jntPt->o->BB.xmin;
      LIGAND_PT->ligBBox.min[1] = lig_jntPt->o->BB.ymin;
      LIGAND_PT->ligBBox.min[2] = lig_jntPt->o->BB.zmin;

      LIGAND_PT->ligBBox.max[0] =  lig_jntPt->o->BB.xmax;
      LIGAND_PT->ligBBox.max[1] =  lig_jntPt->o->BB.ymax;
      LIGAND_PT->ligBBox.max[2] =  lig_jntPt->o->BB.zmax;
      is_first_jnt = FALSE;
    }
    else { 
      
      xmin_jnt= lig_jntPt->o->BB.xmin;
      LIGAND_PT->ligBBox.min[0] = MIN(LIGAND_PT->ligBBox.min[0], xmin_jnt);
      ymin_jnt= lig_jntPt->o->BB.ymin;
      LIGAND_PT->ligBBox.min[1] = MIN(LIGAND_PT->ligBBox.min[1], ymin_jnt);
      zmin_jnt= lig_jntPt->o->BB.zmin;
      LIGAND_PT->ligBBox.min[2] = MIN(LIGAND_PT->ligBBox.min[2], zmin_jnt);
      xmax_jnt= lig_jntPt->o->BB.xmax;
      LIGAND_PT->ligBBox.max[0] = MAX(LIGAND_PT->ligBBox.max[0], xmax_jnt);
      ymax_jnt= lig_jntPt->o->BB.ymax;
      LIGAND_PT->ligBBox.max[1] = MAX(LIGAND_PT->ligBBox.max[1], ymax_jnt);
      zmax_jnt= lig_jntPt->o->BB.zmax;
      LIGAND_PT->ligBBox.max[2] = MAX(LIGAND_PT->ligBBox.max[2], zmax_jnt);
    }
    num_jnt++;
  }
  return  (!is_first_jnt);
}


/************************************************************/
/* \brief: update the value of the axes oriented bounding   */
/*         box around the side chain of the AA of number    */
/*         AAnumber                                         */
/* \return:  1 if an update has been done 0 otherwise       */
/*  note: the structure updated is                          */
/*          joint_table[AAnumber]->ligBBox                  */
/************************************************************/ 
/* int update_sidechain_BBox_from_AAnumber(p3d_rob* robotPt, int  AAnumber)  */
/* { */
/*   p3d_jnt* cur_jnt; */
/*   int jnt_number = get_AAfirstjnt_number(AAnumber); */
/*   int is_first_jnt = TRUE;   */
/*   Joint_tablespt * jnt_table; */
/*   double xmin_jnt, xmax_jnt, ymin_jnt, ymax_jnt, zmin_jnt, zmax_jnt; */
/*   jnt_table =  give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,AAnumber)); */
/*   //we go to the second joint to consider the side chain: */
/*   jnt_number++; */
/*   cur_jnt = XYZ_ROBOT->joints[jnt_number]; */
/*   //if the second joint is phi, we we consider the next joint */
/*   if((cur_jnt != NULL) &&  (cur_jnt->name[1]=='h')){ */
/*     jnt_number++; */
/*     cur_jnt = XYZ_ROBOT->joints[jnt_number]; */
/*   } */
/*   if((cur_jnt == NULL) || (cur_jnt->name==0)|| */
/*      cur_jnt->name[0]!='g') { */
/*     PrintInfo(("Warning:  failed to compute the BBox : the joint is not side chain joint \n")); */
/*     return 0; */
/*   } */

/*   while(cur_jnt->name[0]=='g') { */
/*     if  ((jnt_table[AAnumber] == NULL)) { */
/*       PrintInfo(("Warning:  Error int the jnt_table utilisation \n")); */
/*       return 0; */
/*     } */
/*     if(is_first_jnt == TRUE) { */
/*       jnt_table[AAnumber]->SC_BBox.min[0] = cur_jnt->o->BB.xmin; */
/*       jnt_table[AAnumber]->SC_BBox.min[1] = cur_jnt->o->BB.ymin; */
/*       jnt_table[AAnumber]->SC_BBox.min[2] = cur_jnt->o->BB.zmin; */

/*       jnt_table[AAnumber]->SC_BBox.max[0] =  cur_jnt->o->BB.xmax; */
/*       jnt_table[AAnumber]->SC_BBox.max[1] =  cur_jnt->o->BB.ymax; */
/*       jnt_table[AAnumber]->SC_BBox.max[2] =  cur_jnt->o->BB.zmax; */
/*       is_first_jnt = FALSE; */
/*     } */
/*     else {  */
/*       xmin_jnt= cur_jnt->o->BB.xmin; */
/*       jnt_table[AAnumber]->SC_BBox.min[0] = MIN(jnt_table[AAnumber]->SC_BBox.min[0], xmin_jnt); */
/*       ymin_jnt= cur_jnt->o->BB.ymin; */
/*       jnt_table[AAnumber]->SC_BBox.min[1] = MIN(jnt_table[AAnumber]->SC_BBox.min[1], ymin_jnt); */
/*       zmin_jnt= cur_jnt->o->BB.zmin; */
/*       jnt_table[AAnumber]->SC_BBox.min[2] = MIN(jnt_table[AAnumber]->SC_BBox.min[2], zmin_jnt); */
/*       xmax_jnt= cur_jnt->o->BB.xmax; */
/*       jnt_table[AAnumber]->SC_BBox.max[0] = MAX(jnt_table[AAnumber]->SC_BBox.max[0], xmax_jnt); */
/*       ymax_jnt= cur_jnt->o->BB.ymax; */
/*       jnt_table[AAnumber]->SC_BBox.max[1] = MAX(jnt_table[AAnumber]->SC_BBox.max[1], ymax_jnt); */
/*       zmax_jnt= cur_jnt->o->BB.zmax; */
/*       jnt_table[AAnumber]->SC_BBox.max[2] = MAX(jnt_table[AAnumber]->SC_BBox.max[2], zmax_jnt); */
/*     } */
/*     cur_jnt = robotPt->joints[(cur_jnt->num)+1]; */
/*   } */
/*   return (!is_first_jnt); */
/* } */

/************************************************************/
/* \brief: update the value of the axes oriented bounding   */
/*         box around the side chain of the AA with first   */
/*         firstAAjnt.                                      */
/* \return:  1 if an update has been done 0 otherwise       */
/*  note: the structure updated is                          */
/*          joint_table[AAnumber]->ligBBox                  */
/************************************************************/ 
/* int update_sidechain_BBox_from_firstjnt(p3d_rob* robotPt, p3d_jnt* firstAAjnt)  */
/* { */
/*  int  AAnumber = get_AAnumber_from_jnt(firstAAjnt); */
/*  //Ìf the first_jnt is a prot_base joint or a chain_base joint,  */
/*  //we take the next joint into consideration */
/*  if(AAnumber == -1) { */
/*    return update_sidechain_BBox_from_firstjnt(robotPt, robotPt->joints[(firstAAjnt->num)+1]); */
/*  } */
/*  return update_sidechain_BBox_from_AAnumber(robotPt, AAnumber); */
/* } */


int update_sidechain_BBox_from_firstjnt(p3d_rob* robotPt, p3d_jnt* firstAAjnt) 
{
  int  AAnumber = get_AAnumber_from_jnt(firstAAjnt);
  p3d_jnt* cur_jnt;
  int jnt_number = get_AAfirstjnt_number(AAnumber);
  int is_first_jnt = TRUE;  
  Joint_tablespt * jnt_table;
  double xmin_jnt, xmax_jnt, ymin_jnt, ymax_jnt, zmin_jnt, zmax_jnt;

  if(AAnumber == -1) {
    return update_sidechain_BBox_from_firstjnt(robotPt, robotPt->joints[(firstAAjnt->num)+1]);
  }

  jnt_table =  give_joint_tables(num_subrobot_AA_from_AAjnt(firstAAjnt));
  //we go to the second joint to consider the side chain:
  jnt_number++;
  cur_jnt = robotPt->joints[jnt_number];
  //if the second joint is phi, we we consider the next joint
  if((cur_jnt != NULL) &&  (cur_jnt->name[1]=='h')){
    jnt_number++;
    cur_jnt = robotPt->joints[jnt_number];
  }
  if((cur_jnt == NULL) || (cur_jnt->name==0)||
     cur_jnt->name[0]!='g') {
    PrintInfo(("Warning:  failed to compute the BBox : the joint is not side chain joint \n"));
    return 0;
  }

  while(cur_jnt->name[0]=='g') {
    if  ((jnt_table[AAnumber] == NULL)) {
      PrintInfo(("Warning:  Error int the jnt_table utilisation \n"));
      return 0;
    }
    if(is_first_jnt == TRUE) {
      jnt_table[AAnumber]->SC_BBox.min[0] = cur_jnt->o->BB.xmin;
      jnt_table[AAnumber]->SC_BBox.min[1] = cur_jnt->o->BB.ymin;
      jnt_table[AAnumber]->SC_BBox.min[2] = cur_jnt->o->BB.zmin;

      jnt_table[AAnumber]->SC_BBox.max[0] =  cur_jnt->o->BB.xmax;
      jnt_table[AAnumber]->SC_BBox.max[1] =  cur_jnt->o->BB.ymax;
      jnt_table[AAnumber]->SC_BBox.max[2] =  cur_jnt->o->BB.zmax;
      is_first_jnt = FALSE;
    }
    else { 
      xmin_jnt= cur_jnt->o->BB.xmin;
      jnt_table[AAnumber]->SC_BBox.min[0] = MIN(jnt_table[AAnumber]->SC_BBox.min[0], xmin_jnt);
      ymin_jnt= cur_jnt->o->BB.ymin;
      jnt_table[AAnumber]->SC_BBox.min[1] = MIN(jnt_table[AAnumber]->SC_BBox.min[1], ymin_jnt);
      zmin_jnt= cur_jnt->o->BB.zmin;
      jnt_table[AAnumber]->SC_BBox.min[2] = MIN(jnt_table[AAnumber]->SC_BBox.min[2], zmin_jnt);
      xmax_jnt= cur_jnt->o->BB.xmax;
      jnt_table[AAnumber]->SC_BBox.max[0] = MAX(jnt_table[AAnumber]->SC_BBox.max[0], xmax_jnt);
      ymax_jnt= cur_jnt->o->BB.ymax;
      jnt_table[AAnumber]->SC_BBox.max[1] = MAX(jnt_table[AAnumber]->SC_BBox.max[1], ymax_jnt);
      zmax_jnt= cur_jnt->o->BB.zmax;
      jnt_table[AAnumber]->SC_BBox.max[2] = MAX(jnt_table[AAnumber]->SC_BBox.max[2], zmax_jnt);
    }
    cur_jnt = robotPt->joints[(cur_jnt->num)+1];
  }
  return (!is_first_jnt);
}



/*************************************************************/
/*
 * \brief: update the value of the axes all oriented bounding
 *         box around the side chain of all AA with flexible sc                                      
 * \return:  1 if an all the updates have been done 0 otherwise       
 *  note: the structures updated are all the                          
 *          joint_table[AAnumber]->ligBBox                  
 */
/************************************************************/ 
int update_all_sidechain_BBoxes(p3d_rob* robotPt) {
  int is_ok =1;
  p3d_jnt** firstjnts_flexible_sc = get_list_firstjnts_flexible_sc(robotPt);
  p3d_jnt* firstjnt_flexsc = firstjnts_flexible_sc[0];
  int i =0;
  while(firstjnt_flexsc!= NULL) {
    if (update_sidechain_BBox_from_firstjnt(robotPt, firstjnt_flexsc)== 0) {
      is_ok = 0;
    }
    i++;
    firstjnt_flexsc =   firstjnts_flexible_sc[i];
  }
return is_ok;
}


/************************************************************/
/* \brief: compute a vector containing the X,Y,Z distances  */
/*         between bounding boxes of the lignad and the AA  */
/*        of number AAnumber. return also if the BB boxes   */
/*        are overlapping.                                  */
/*  \param robotPt: the robot                               */
/*  \param AAnumber: the number of the number of the AA, as */
/*          it is described in its name                     */
/*  \retval dist_vector : a 3 dimensional vector with X,Y,Z */
/*          distances between the BBoxes (0 if the projected*/
/*          distance is                                     */
/*  \return 1 if the                                        */
/*         box around the side chain of the AA with first   */
/*         firstAAjnt.                                      */
/* \return:  1 if the boxes are not overlapping, 0 otherwise*/
/*  \note:  the functions updating the values of the BBoxes */
/*          must be called before this function             */
/************************************************************/ 
int dist_ligand_sidechainBBoxes(p3d_rob* robotPt, int  AAnumber, double  dist_vector[])
{
  double d1,d2;
  int i, is_overlapping_box = TRUE;
  Joint_tablespt * jnt_table; 
  jnt_table = give_joint_tables (num_subrobot_AA_from_AAnumber(robotPt,AAnumber));
  if  ((jnt_table[AAnumber] == NULL)) {
    PrintInfo(("Warning:  Error int the jnt_table utilisation \n"));
    return FALSE;
  }
 //successive comparisons along X,Y,Z axes:
  for(i=0;i<3;i++) {
    d1 =  LIGAND_PT->ligBBox.min[i] - jnt_table[AAnumber]->SC_BBox.max[i];
    if(d1>0) {
      dist_vector[i] = d1; 
      is_overlapping_box =FALSE;
    }
    else {
      d2 =  jnt_table[AAnumber]->SC_BBox.min[i] -LIGAND_PT->ligBBox.max[i];
      if(d2>0) {
	dist_vector[i] = d2; 
	is_overlapping_box =FALSE;
      }
      else {
	dist_vector[i] = 0.;
      }
    }
  }
  return (!is_overlapping_box);
}

/************************************************************/
/* \brief: Print the X,Y,Z distances between AA and Ligand  */
/*         BBoxes. Usefull for debuging!                    */
/* \param robotPt: the robot                                */
/* \param AAnumber the number of the number of the AA, as   */
/*        it is described in its name                       */
/************************************************************/
void PrintInfo_BBoxesdist(p3d_rob* robotPt, int  AAnumber) {
  double dist_vector[3];
  int i, is_free;

  is_free= dist_ligand_sidechainBBoxes( robotPt, AAnumber, dist_vector);
  for (i=0;i<3;i++) {
    PrintInfo(("dist_vector[%d]: %f\n",i, dist_vector[i]));
  }
  PrintInfo(("free ok ?: %d\n",is_free));
}



/**************************************************************************/
/*static int BACKBONE = 0; */
/*static int SIDE_CHAIN = 1;*/
/*static int PAS_DE_JOINT_SUIVANT = -1;*/
/*static int NB_TESTS = 100; */
/**************************************************************************/



/*******************************/
/*******************************/
/* fonctions de tests de perf  */
/*******************************/
/*******************************/

/* void test_perf(void) */
/* { */
/*   int i = 0; */
/*   int ncol = 0; */

/*   double tu,ts;  */
/*   p3d_graph *G; */
/*   configPt q; */
  
/*   if(!XYZ_GRAPH) G = p3d_create_graph(); */
/*   else           G = XYZ_GRAPH; */
  
/*   q = p3d_alloc_config(G->rob); */


/*   printf("nombre de configurations generees : %d \n",NB_TESTS); */
/*     ChronoOn(); */
/*   for(i=0;i<NB_TESTS;i++) */
/*     { */
/*       p3d_shoot(G->rob,q,1); */
      
/*       p3d_set_robot_config(G->rob, q); */
/*       p3d_update_this_robot_pos_without_cntrt(G->rob); */
/*       ncol = p3d_col_test(); */
 
/*     } */
/*   ChronoTimes(&tu,&ts); */
/*   G->time = G->time + tu; */
/*   ChronoPrint("Fin Chrono"); */
/*   ChronoOff(); */
  
/*   p3d_destroy_config(G->rob, q); */
  
/* } */

/**************************************************************************/
/**************************************************************************/

/* change the rigidity of a backbone (nature = BAKBONE) */ 
/* or a side-chain (nature = SIDE_CHAIN)                */
/* this fuction doesn't modify the p3d file             */


/* void changer_rigidite(int number, int nature) */
/* { */
/*   int encoreUnJoint = 1; */
/*   int isRigid = is_AA_rigid(number, nature); */
/*   char AAtype[4]; */
/*   p3d_jnt *jnt_AA = get_AAfirstjnt(number); */

/*   get_AAtype_from_AAnumber(number,AAtype); */

/*   // the first AA is different : it has't an "omega" joint   */
/*   // and the "gamma" are jointto le "phi" joint but to theP3D_BASE  */
/*   if((number == 1)&&(nature == SIDE_CHAIN)) */
/*     { // in jnt_AA we have the "phi" joint : we want the "gamma" joint -> we must go up to the P3B_BASE  */
/*       jnt_AA = jnt_AA->prev_jnt->next_jnt[1]; */
/*       encoreUnJoint = jnt_AA->n_next_jnt; */
/*       // this joint is the gamma1 : we modify it now to avoid problems if there is no other gamma // */
/*       jnt_AA->vmax = 3.1415926535897931; */
/*       jnt_AA->vmin = -3.1415926535897931; */
/*       jnt_AA->vmax_rand = 3.1415926535897931; */
/*       jnt_AA->vmin_rand = -3.1415926535897931; */
/*     } */

/*   if(nature == BACKBONE) */
/*     { */
/*       if(isRigid) */
/* 	{ */
/* 	  printf("The backbone of the AA %s is become flexible. \n",AAtype);       */
/* 	} */
/*       else */
/* 	{ */
/* 	  printf("The backbone of the AA %s is become rigide. \n",AAtype);      	   */
/* 	} */
/*     } */
/*   else */
/*     { */
/*       if(isRigid) */
/* 	{ */
/* 	  printf("The side chain of the AA %s is become flexible. \n",AAtype);       */
/* 	} */
/*       else */
/* 	{ */
/* 	  printf("The side chain of the AA %s is become rigide. \n",AAtype);      	   */
/* 	} */
/*     } */
  
/*   // On parcours tous les joints de l'AA referencer par son number */
/*   // on s'arrete dans le cas ou on n'a plus de joints ou alors qd on est dans le premier AA et  */
/*   // lorsque encoreUnJoint vaut zero, ie que l'on atteint le bout de la chaine de gamma  */
/*   while((encoreUnJoint != PAS_DE_JOINT_SUIVANT)&&((number!=1)||(encoreUnJoint != 0 ))) */
/*     {       */
/*       if(isRigid) */
/* 	{ // cas ou l'AA est rigide : le rendre flexible  */
/* 	  if(nature == SIDE_CHAIN) // cas ou l'on touche a une chaine laterale */
/* 	    { */
/* 	      if(jnt_AA->name[0]=='g') */
/* 		{ // alors le joint correspond a une chaine laterale : c du type gamma  */
/* 		  jnt_AA->vmax = 3.1415926535897931; */
/* 		  jnt_AA->vmin = -3.1415926535897931; */
/* 		  jnt_AA->vmax_rand = 3.1415926535897931; */
/* 		  jnt_AA->vmin_rand = -3.1415926535897931; */
/* 		} */
/* 	    } */
/* 	  else // cas ou l'on touche au backbone  */
/* 	    { */
/* 	      if(jnt_AA->name[0]=='p') */
/* 		{ // alors le joint correspond a un backbone de type phi ou psy  */
/* 		  jnt_AA->vmax = 3.1415926535897931; */
/* 		  jnt_AA->vmin = -3.1415926535897931; */
/* 		  jnt_AA->vmax_rand = 3.1415926535897931; */
/* 		  jnt_AA->vmin_rand = -3.1415926535897931; */
/* 		} */
/* 	      if(jnt_AA->name[0]=='o') */
/* 		{ // alors le joint correspond a un backbone de type omega */
/* 		  if(jnt_AA->v > 0) */
/* 		    { */
/* 		      jnt_AA->vmax = 3.3161255787892263; */
/* 		      jnt_AA->vmin = 2.9670597283903604; */
/* 		      jnt_AA->vmax_rand = 3.3161255787892263; */
/* 		      jnt_AA->vmin_rand = 2.9670597283903604; */
/* 		    } */
/* 		  else */
/* 		    { */
/* 		      jnt_AA->vmax = -2.9670597283903604; */
/* 		      jnt_AA->vmin = -3.3161255787892263;	   */
/* 		      jnt_AA->vmax_rand = -2.9670597283903604; */
/* 		      jnt_AA->vmin_rand = -3.3161255787892263;     */
/* 		    } */
/* 		}  */
/* 	    } // fin du else  */

/* 	} // fin if(isRigid)  */
/*       else */
/* 	{ // cas ou l'AA est flexible : le rendre rigide  */
/* 	  if(nature == SIDE_CHAIN) // cas ou l'on touche a une chaine laterale */
/* 	    { */
/* 	      if(jnt_AA->name[0]=='g') */
/* 		{ // alors le joint correspond a une chaine laterale : c du type gamma  */
/* 		  jnt_AA->vmax = jnt_AA->v; */
/* 		  jnt_AA->vmin = jnt_AA->v; */
/* 		  jnt_AA->vmax_rand = jnt_AA->v; */
/* 		  jnt_AA->vmin_rand = jnt_AA->v; */
/* 		} */
/* 	    } */
/* 	  else // cas ou l'on touche au backbone  */
/* 	    { */
/* 	      if(jnt_AA->name[0]!='g') */
/* 		{ // alors le joint correspond a un backbone */
/* 		  jnt_AA->vmax = jnt_AA->v; */
/* 		  jnt_AA->vmin = jnt_AA->v; */
/* 		  jnt_AA->vmax_rand = jnt_AA->v; */
/* 		  jnt_AA->vmin_rand = jnt_AA->v; */
/* 		} */
/* 	    }	   */
/* 	} */
/*       encoreUnJoint = next_joint(&jnt_AA, &jnt_AA, encoreUnJoint); */
/*     } // fin du while */
/* } */

/**************************************************************************/
/**************************************************************************/

/* For the bakbone we use the first joint : if vmin == vmax then it is rigid */
/* for the side Chain : we use the first joint "gamma" */
/* we must treat two cases apart : the case of the first acidoamine */
/* and the case of the GLY (it hasn't a side chaine !)  */

/* int is_AA_rigid(int number, int nature) */
/* {   */
/*   int isRigid = 1; */
/*   int encoreUnJoint = 1; */
/*   p3d_jnt *jnt_AA = get_AAfirstjnt(number); */

/*   if(nature == BACKBONE) */
/*     { */
/*       if(jnt_AA->vmin != jnt_AA->vmax) */
/* 	{ */
/* 	  isRigid = 0; */
/* 	} */
/*     } */
/*   else // in this case we are interesting for side chain  */
/*     { */
/*       if(number==1) */
/* 	{ */
/* 	  if(jnt_AA->prev_jnt->next_jnt[1]->vmin != jnt_AA->prev_jnt->next_jnt[1]->vmax) */
/* 	    { */
/* 	      isRigid = 1; */
/* 	    } */
/* 	} */
/*       else  */
/* 	{ */
/* 	  // case of the GLY  */
/* 	  if(jnt_AA->next_jnt[0]->n_next_jnt == 1) */
/* 	    { */
/* 	      isRigid = 0; */
/* 	    } */
/* 	  else  */
/* 	    { */
/* 	      while(jnt_AA->name[0] != 'g'){ */
/* 		encoreUnJoint = next_joint(&jnt_AA, &jnt_AA, encoreUnJoint); */
/* 	      } */
/* 	      if(jnt_AA->vmin != jnt_AA->vmax) */
/* 		{ */
/* 		  isRigid = 0; */
/* 		} */
/* 	    } */
/* 	} */
/*     } // end of else side chain  */
/*   return isRigid; */
/* } */

/***********************************/
/* virer les collisions embetantes */
/***********************************/

// necessite l'appel a la fonction bio_all_molecules_col_with_report() avt emploi
void afficher_lescollisions(void)
{
  int i = 0;
  int col_number;
  p3d_poly **p1,**p2;

  biocol_report(&col_number, &p1, &p2);
  printf("col_number %d \n",col_number);

  for(i=0;i<col_number;i++)
    {
      printf("joints en collision : %s   %s\n",p1[i]->poly->name,p2[i]->poly->name);
    }

}

static void test_affichage(void)
{
  int nb_col = 0;

  nb_col = bio_all_molecules_col_with_report();

  puts("test de collisions faits");

  printf("nb_col : %d \n",nb_col);
  afficher_lescollisions();
}
