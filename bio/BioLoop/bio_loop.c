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
// modif Juan (for BioMove3d)
/* (jcortes) */
/* WARNING :
   This file is already in testing phase.
   Some functions work well only in particular cases.
*/

#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Util-pkg.h"
#include "Bio-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"

// modif Juan : for saving nodes when move3d "plante" 
extern void p3d_print_node(p3d_graph *G, p3d_node *N);


/*****************************************************************************/

static void bio_loop_set_shells_rint_and_alpha(pp3d_rlg_chain rlgchPt, int njxla, int nja, int i2, double error);

static int bio_generate_active_part_of_loop_wrapper(p3d_cntrt *ct, configPt q);
static int bio_generate_active_part_of_loop(p3d_cntrt *ct, configPt q, p3d_jnt *Jfaj);
static int bio_generate_FREE_active_part_of_loop(p3d_cntrt *ct, configPt q, p3d_jnt *Jfaj);

static int bio_random_active_part_of_loop(p3d_cntrt *ct, configPt q);




/* SAVED CONFORMATION */
/* static configPt bio_saved_conf; */

/*****************************************************************************/

/* PARTIAL RESHOOT :   TO DO !!!
   the configuration of each loop (or part of loop) must be re-generated
   with a partial reshoot keeping the correctly generated part of the global q 
*/

/*************************************************************************/

// NEW FUNCTIONS ///////////////////////////////////////////////////////////
// functions for handling backbone H-bonds

int bio_set_bkb_Hbond(int ifj, int ilj, int index_main_cntrt, 
		      double minNOdist, double maxNOdist, double minHbondang, double maxHbondang)
{
  p3d_rob *r;
  int setok;
  configPt curq;
  p3d_jnt *J,*Jilj,*Jifj;
  double vmin,vmax;
  double Dval[4];
  int Ival[4];
  int isNObond;
  char *aname, firts_a_nameJf[10], firts_a_nameJl[10];
  int indexletter=0;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  curq = p3d_alloc_config(r);

  p3d_get_robot_config_into(r,&curq);

  // get joints
  Jifj = r->joints[ifj];
  Jilj = r->joints[ilj];

  // identify if is an N-O or an O-N bond
  //  -> if there is no H' : the body containing N only contains this atom
  indexletter = 0;
  aname = givemeword(Jifj->o->pol[0]->poly->name, '.', &indexletter);
  strcpy(firts_a_nameJf,aname);
  indexletter = 0;
  aname = givemeword(Jilj->o->pol[0]->poly->name, '.', &indexletter);
  strcpy(firts_a_nameJl,aname);
  
  //if((Jifj->bio_jnt_type == BIO_GAMMA_JNT) && (Jilj->bio_jnt_type == BIO_PSI_JNT)) {
  if((strcmp(firts_a_nameJf,"V-N") == 0) && (strcmp(firts_a_nameJl,"V-C") == 0)) {
    isNObond = 1;   // N-O bond
  }
  //else if((Jilj->bio_jnt_type == BIO_GAMMA_JNT) && (Jifj->bio_jnt_type == BIO_PSI_JNT)) {
  else if((strcmp(firts_a_nameJl,"V-N") == 0) && (strcmp(firts_a_nameJf,"V-C") == 0)) {
    isNObond = 0;   // O-N bond
    Jilj = Jilj->prev_jnt; // last joint is associated to C
  }
  else {
    printf("ERROR : bio_set_bkb_Hbond : wrong H bond setting");
    return 0;
  }

  // create constraint
  Dval[0] = minNOdist;
  Dval[1] = maxNOdist;
  Dval[2] = minHbondang;
  Dval[3] = maxHbondang;
  Ival[0] = ifj;
  Ival[1] = Jilj->num;
  Ival[2] = index_main_cntrt;
  Ival[3] = isNObond;

  // NOTE : the constraint is desactivated (it will be activated during the sampling)  <- NO !!!
  if(!p3d_constraint("bio_bkb_Hbond_cntrt",0,NULL,0,NULL,4,Dval,4,Ival,-1,1)) {
    p3d_set_robot_config(r,curq);
    p3d_update_this_robot_pos_without_cntrt_and_obj(r);
    p3d_destroy_config(r,curq);
    return 0;
  }

/*   if(index_main_cntrt >= 0) { */
    // put on peptide planes
    J = r->joints[ifj];
    while(J != Jilj) {    
      p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
      if(vmax < M_PI)
	p3d_jnt_set_dof(J,0,-M_PI);
      else
	p3d_jnt_set_dof(J,0,M_PI);
      // WARNING : the translator makes the next jnt in the bkb be always J->next_jnt[J->n_next_jnt - 1]
      J = J->next_jnt[J->n_next_jnt - 1];
    }
    p3d_update_this_robot_pos_without_cntrt_and_obj(r);
    
    setok = bio_set_rlg(r->cntrt_manager->ncntrts - 1,Jifj->num,Jilj->num);
    
    if(setok) {
      p3d_active_RLG_flags();
    }
  
    // restore config
    p3d_set_robot_config(r,curq);
    p3d_update_this_robot_pos_without_cntrt_and_obj(r);
/*   } */
/*   else { */
/*     setok = 1; */
/*   } */

  p3d_destroy_config(r,curq);

  return (setok);
}

/*************************************************************************/
/* NOTE : Currently, the diS_bond functions only consider distance constraints.
          Orientation constraints should also be considered in the future
*/
int bio_set_diS_bond(int ifj, int ilj, double minSSdist, double maxSSdist)
{
  double Dval[2];
  int Ival[2];

  // create constraint
  Dval[0] = minSSdist;
  Dval[1] = maxSSdist;
  Ival[0] = ifj;
  Ival[1] = ilj;

  // NOTE : the constraint is desactivated (it will be activated during the sampling)  <- NO !!!
  if(!p3d_constraint("bio_diS_bond_cntrt",0,NULL,0,NULL,2,Dval,2,Ival,-1,1)) {
    return 0;
  }
  return 1;
}

/*************************************************************************/
static int bio_num_loops = 0;

int bio_get_num_loops(void)
{
  return bio_num_loops;
}

/*************************************************************************/

// fixed position and orientation
  
int bio_set_loop(int ifj, int ilj)
{
  p3d_rob *r;
  int i,ifpj,irj;
  int setok;
  configPt curq;
  p3d_jnt *J,*Jilj,*Jbase;
  double vmin,vmax;
  p3d_vector3 posJ,axeJ,axeJp,V,xaxis,yaxis;
  p3d_matrix4 Fabs,Frel,invTbase;
  double Dval[12];

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  // get last loop joint 
  Jilj = r->joints[ilj];

  if(r->joints[ilj+1]->type != P3D_FREEFLYER) {
    //  CASE 1 : SUBCHAINS ARE NOT DEFINED (OLD VERSION)

    // WARNING : THE NEXT RESIDUE TO THE LAST RESIDUE IN THE LOOP MUST BE ATICULATED WITH FIXEWD JOINT VALUES !!!

    // block the joint preceding Jifj, Jilj and the 2 next bkb joints (WARNING : CAN GIVE PROBLEMS)
    J = Jilj;
    for(i=0; i<3; i++) {
      p3d_jnt_set_dof_is_user(J,0,FALSE); 
      J = J->next_jnt[J->n_next_jnt - 1];
    }
    J = r->joints[ifj]->prev_jnt;
    p3d_jnt_set_dof_is_user(J,0,FALSE); 
    
    // get the subchain base-jnt (freeflyer)
    Jbase = Jilj->prev_jnt;
    while(Jbase->type != P3D_FREEFLYER)
      Jbase = Jbase->prev_jnt;
    // breack bkb chain at Jilj
    // WARNING : the translator makes the next jnt in the bkb be always J->next_jnt[J->n_next_jnt - 1]
    p3d_jnt_attach_to_jnt(Jbase,Jilj->next_jnt[Jilj->n_next_jnt - 1]); // this function makes all the necessary
    //p3d_jnt_attach_to_jnt(Jbase,Jilj); // this function makes all the necessary

  }
  else {
    //  CASE 2 : SUBCHAINS ARE DEFINED (NEW VERSION)
    Jbase = r->joints[ilj+2];
  }
    
  // compute end-frame (relative to the subchain base-jnt)
  // WARNING : we suppose that a joint is asociated to each bkb atom !!!
  p3d_jnt_get_cur_vect_point(Jilj,posJ);
  p3d_jnt_get_dof_cur_axis(Jilj,0,axeJ);
  p3d_jnt_get_dof_cur_axis(Jilj->prev_jnt,0,axeJp);
  p3d_vectXprod(axeJp,axeJ,V);
  p3d_vectNormalize(V,xaxis);
  p3d_vectXprod(axeJ,xaxis,V);
  p3d_vectNormalize(V,yaxis);
  
  Fabs[0][0] = xaxis[0]; Fabs[0][1] = yaxis[0]; Fabs[0][2] = axeJ[0]; Fabs[0][3] = posJ[0]; 
  Fabs[1][0] = xaxis[1]; Fabs[1][1] = yaxis[1]; Fabs[1][2] = axeJ[1]; Fabs[1][3] = posJ[1]; 
  Fabs[2][0] = xaxis[2]; Fabs[2][1] = yaxis[2]; Fabs[2][2] = axeJ[2]; Fabs[2][3] = posJ[2]; 
  Fabs[3][0] = 0.0;      Fabs[3][1] = 0.0;      Fabs[3][2] = 0.0;     Fabs[3][3] = 1.0; 
  
  p3d_matInvertXform(Jbase->abs_pos,invTbase); 
  p3d_mat4Mult(invTbase,Fabs,Frel);    
  
  Dval[0]  = Frel[0][0]; Dval[1]  = Frel[1][0]; Dval[2]  = Frel[2][0]; 
  Dval[3]  = Frel[0][1]; Dval[4]  = Frel[1][1]; Dval[5]  = Frel[2][1]; 
  Dval[6]  = Frel[0][2]; Dval[7]  = Frel[1][2]; Dval[8]  = Frel[2][2]; 
  Dval[9]  = Frel[0][3]; Dval[10] = Frel[1][3]; Dval[11] = Frel[2][3]; 
  

/*     Dval[0]  = 1.0; Dval[1]  = 0.0; Dval[2]  = 0.0;  */
/*     Dval[3]  = 0.0; Dval[4]  = 1.0; Dval[5]  = 0.0;  */
/*     Dval[6]  = 0.0; Dval[7]  = 0.0; Dval[8]  = 1.0;  */
/*     Dval[9]  = 0.0; Dval[10] = 0.0; Dval[11] = 0.0; */
/*   } */

  // set constraint (bio-IK)

  curq = p3d_alloc_config(r);

  p3d_get_robot_config_into(r,&curq);

  // put on peptide planes
  J = r->joints[ifj];
  while(J != Jilj) {
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if(vmax < M_PI)
      p3d_jnt_set_dof(J,0,-M_PI);
    else
      p3d_jnt_set_dof(J,0,M_PI);
    J = J->next_jnt[J->n_next_jnt - 1];
  }
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);

  /* last passive joint == Jilj */
  /* base-joint  == irj */  
  /* consider that end-frame is defined by a trasnformation (Frel) from the subchain base-jnt */
  irj = Jbase->num;
  /* first passive joint */
  J = Jilj;
  for(i=0; i<8; i++) {
    J = J->prev_jnt;
  }
  ifpj = J->num;
  /*   if(!p3d_constraint("p3d_6R_bio_ik",1,&ifpj,1,&irj,0,NULL,0,NULL,-1,1)) */
  if(!p3d_constraint("p3d_6R_bio_ik_nopep_new",1,&ifpj,1,&irj,12,Dval,0,NULL,-1,1)) {
    p3d_set_robot_config(r,curq);
    p3d_update_this_robot_pos_without_cntrt_and_obj(r);
    p3d_destroy_config(r,curq);
    return 0;
  }

  // set bio-RLG
  setok = bio_set_rlg(r->cntrt_manager->ncntrts - 1,ifj,ilj);

  if(setok) {
    p3d_active_RLG_flags();
    bio_num_loops++;
  }
  
  // restore config
  p3d_set_robot_config(r,curq);
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);
  p3d_destroy_config(r,curq);

  return (setok);
}


/*************************************************************************/

// ifj = index first joint
// ilj = index last joint


// fixed fosition (with an error)  
int bio_set_loop_pos(int ifj, int ilj, double x, double y, double z, double error)
{
  p3d_rob *r;
  int setok;
  configPt curq;
  p3d_jnt *J,*Jilj;
  double vmin,vmax;
  double vecargd[4];

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  curq = p3d_alloc_config(r);

  p3d_get_robot_config_into(r,&curq);

  //////////////////////////
  //bio_full_check_shell(r,r->joints[3],r->joints[17]);
  //bio_check_shell(r,r->joints[6],r->joints[20]);
  //////////////////////////

  // get last
  Jilj = r->joints[ilj];

  // put on peptide planes
  J = r->joints[ifj];
  while(J != Jilj) {
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if(vmax < M_PI)
      p3d_jnt_set_dof(J,0,-M_PI);
    else
      p3d_jnt_set_dof(J,0,M_PI);
    // WARNING : the translator makes the next jnt in the bkb be always J->next_jnt[J->n_next_jnt - 1]
    J = J->next_jnt[J->n_next_jnt - 1];
  }
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);

  vecargd[0] = x;
  vecargd[1] = y;
  vecargd[2] = z;
  vecargd[3] = error;
  if(!p3d_constraint("p3d_in_sphere",0,NULL,1,&ilj,4,vecargd,0,NULL,-1,1)) {
    p3d_set_robot_config(r,curq);
    p3d_update_this_robot_pos_without_cntrt_and_obj(r);
    p3d_destroy_config(r,curq);
    return 0;
  }

  setok = bio_set_rlg(r->cntrt_manager->ncntrts - 1,ifj,ilj);
  
  if(setok) {
    p3d_active_RLG_flags();
    /* set bio_loop base and end frames */
    // NECESARIO ???
    /* keep configration out of the loop */
    /*     bio_saved_conf = p3d_alloc_config(r); */
    /*     bio_save_conf(r); */
  }
  
  // restore config
  p3d_set_robot_config(r,curq);
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);
  p3d_destroy_config(r,curq);

  return (setok);

}

// fixed position and orientation
int bio_set_loop_ori(int ifj, int ilj, int irj)
{
  p3d_rob *r;
  int i,ifpj;
  int setok;
  configPt curq;
  p3d_jnt *J,*Jilj;
  double vmin,vmax;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  curq = p3d_alloc_config(r);

  p3d_get_robot_config_into(r,&curq);

  //////////////////////////
  //bio_full_check_shell(r,r->joints[3],r->joints[17]);
  //bio_check_shell(r,r->joints[6],r->joints[20]);
  //////////////////////////

  // get last joint position
  Jilj = r->joints[ilj];

  // put on peptide planes
  J = r->joints[ifj];
  while(J != Jilj) {
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if(vmax < M_PI)
      p3d_jnt_set_dof(J,0,-M_PI);
    else
      p3d_jnt_set_dof(J,0,M_PI);
    // WARNING : the translator makes the next jnt in the bkb be always J->next_jnt[J->n_next_jnt - 1]
    J = J->next_jnt[J->n_next_jnt - 1];
  }
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);

  /* last passive joint == Jilj */
  /* reference joint (end-frame) -> irj */  
  /* consider that end-frame is defines as the next joint (in the file) to Jilj */
  //irj = ilj + 1;
  /* first passive joint */
  J = Jilj;
  for(i=0; i<8; i++) {
    J = J->prev_jnt;
  }
  ifpj = J->num;
  /*   if(!p3d_constraint("p3d_6R_bio_ik",1,&ifpj,1,&irj,0,NULL,0,NULL,-1,1)) */
  if(!p3d_constraint("p3d_6R_bio_ik_nopep",1,&ifpj,1,&irj,0,NULL,0,NULL,-1,1)) {
    p3d_set_robot_config(r,curq);
    p3d_update_this_robot_pos_without_cntrt_and_obj(r);
    p3d_destroy_config(r,curq);
    return 0;
  }

  setok = bio_set_rlg(r->cntrt_manager->ncntrts - 1,ifj,ilj);

  if(setok) {
    p3d_active_RLG_flags();
    bio_fix_unfix_loop_end(r,1);
    /* set bio_loop base and end frames */
    // NECESARIO ???
    /* keep configuration out of the loop */
/*     bio_saved_conf = p3d_alloc_config(r); */
/*     bio_save_conf(r); */
  }
  
  // restore config
  p3d_set_robot_config(r,curq);
  p3d_update_this_robot_pos_without_cntrt_and_obj(r);
  p3d_destroy_config(r,curq);

  return (setok);

}


/*****************************************************************************/

void bio_fix_unfix_loop_end(p3d_rob *robPt, int onoff)
{
  int i;
  p3d_cntrt *ct = robPt->cntrt_manager->cntrts[robPt->cntrt_manager->ncntrts - 1];  // <- A MODIFICAR !!!

  if(onoff) {
    ct->argu_i[10] = 1;
    for(i=0; i<6; i++) {
/*       ct->argu_d[i] = p3d_jnt_get_dof(ct->actjnts[0],i); */
      p3d_jnt_set_dof_is_user(ct->actjnts[0],i,0);
    }
  }
  else {
    ct->argu_i[10] = 0;
    for(i=0; i<6; i++) {
      p3d_jnt_set_dof_is_user(ct->actjnts[0],i,1);
    }    
  }
}

int bio_fixed_loop_end(void)     // <- A MODIFICAR ENTERA !!!
{
  p3d_rob *robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  p3d_cntrt *ct = robPt->cntrt_manager->cntrts[robPt->cntrt_manager->ncntrts - 1];  // <- A MODIFICAR !!!

  return(ct->argu_i[10]);
}

/*****************************************************************************/
/*****************************************************************************/

/* ABOUT bioRLG :
   - it is applied to entire amino-acids 
      -> first rotation around a N-CA bond
      -> last rotation around a CA-C bond
     NOTE : MODIF : first atom can be different
   - only phi and psi angles are considered
   - phi and psi are not limited [0, 360]
     ( or at least we consider that the values 180 and 0 are possible 
       -> maximum and minimum elongation )
   - axis direction : from X to X+1
*/


int bio_set_rlg(int ctnum, int ifj, int ilj)
{
  p3d_rob *robPt;
  p3d_cntrt *ct;
  pp3d_rlg_chain_data rlg_data; 
  pp3d_rlg_chain rlgchPt;
  p3d_jnt *Jfaj, *Jlaj, *Jend, *J, *Jp=NULL;  
  int nj;
  double r=0.0, al=0.0;
  int i,i2;
  p3d_vector3 posJ,posJp,pos_diff,axeJ,poslast;
  double vmin,vmax;
  p3d_matrix4 Fabs;
 
  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  ct = robPt->cntrt_manager->cntrts[ctnum];

  Jfaj = robPt->joints[ifj];
  Jend = robPt->joints[ilj];
  if(strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep")==0 || strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep_new")==0) {
    Jlaj = ct->pasjnts[0]->prev_jnt;           // jnt preceding to the first passive jnt
  }
  else if(strcmp(ct->namecntrt,"p3d_in_sphere")==0) {
    Jlaj = robPt->joints[ct->argu_i[0] - 1];   // jnt preceding to the "active" jnt
  }
  else if(strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) {
    Jlaj = robPt->joints[ct->argu_i[1]];   // jnt preceding to the last one
  }
  else {
    return (FALSE);
  }

  nj = 0;
  J = Jlaj;
  while(J != Jfaj) {
    J = J->prev_jnt;
    nj++;
  }
  nj++;

  /*memory allocation */
  if(ct->rlgPt == NULL)
    p3d_generate_rlg(ct);
  else {
    p3d_destroy_rlg(ct->rlgPt);
    PrintError(("RLG setting : double definition of RLG !\n"));
    return (FALSE);
  }
  
  rlgchPt = MY_ALLOC(p3d_rlg_chain,1);
  rlgchPt->nlinksrlgch = nj;
  rlgchPt->rlg_data = MY_ALLOC(pp3d_rlg_chain_data,nj); 

  /* set generation function */
  rlgchPt->rlg_chain_fct = bio_generate_active_part_of_loop_wrapper;
  //rlgchPt->rlg_chain_fct = bio_random_active_part_of_loop;

  /* DESCRIPTION :
     data are computed in two steps :
     1- alloc memory and compute "local" parameters of joits
     2- compute spherical shells
      ## CONSIDERATIONS (new process)
      - max.length with all dihedral angles at 180
      - min.length ???
        NOTE : for more than 4 amino-acids the min.length is 0
      -> compute for full shells !!!
  */

  // end-frame position for max.length (ANGLES AT 180 NOW!)
  if(strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) {
    p3d_mat4Mult(Jend->abs_pos,ct->Tatt,Fabs);
    poslast[0] = Fabs[0][3];
    poslast[1] = Fabs[1][3];
    poslast[2] = Fabs[2][3];    
  }
  else {
    p3d_jnt_get_cur_vect_point(Jend,poslast);
  }

  J = Jlaj;
  // identify peptide bond
  /* NOTE : joint limits are used currently
            however a better solution should be to use identifiers in jnt structure
  */
  i2 = 2;
  p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
  while((vmax-vmin) > M_PI/2.0) {
    J = J->prev_jnt;
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    i2--;
  }
  J = Jlaj;
  i = 0;
  while((J != NULL)&&(J != Jfaj->prev_jnt)) {
    /* flag for the generation of configurations */
    /* this flag is not currently used, but it will be used in the shoot */
    robPt->cntrt_manager->in_cntrt[J->index_dof] = -1;
    Jp = J->next_jnt[J->n_next_jnt - 1];
    if(i2 == 2) {
      rlg_data = MY_ALLOC(p3d_rlg_chain_data,1);
      rlgchPt->rlg_data[nj-i-1] = rlg_data;
      rlgchPt->rlg_data[nj-i-1]->jnt = J;
      rlgchPt->rlg_data[nj-i-1]->num_dof_jnt = 0;
      robPt->cntrt_manager->in_cntrt[J->index_dof] = -1;
      J = J->prev_jnt;
      if((J == NULL)||(J == Jfaj->prev_jnt))
	break;
      i++;
      i2 = 0;
    }
    p3d_jnt_get_cur_vect_point(J,posJ);
    p3d_jnt_get_cur_vect_point(Jp,posJp);
    p3d_jnt_get_dof_cur_axis(J,0,axeJ);
    p3d_vectSub(posJp,posJ,pos_diff);
    r = (double) p3d_vectNorm(pos_diff);
    al = acos(p3d_vectDotProd(axeJ,pos_diff)/(p3d_vectNorm(axeJ)*p3d_vectNorm(pos_diff)));

    /*** Parameters from laj to faj ***/
    rlg_data = MY_ALLOC(p3d_rlg_chain_data,1);
    rlgchPt->rlg_data[nj-i-1] = rlg_data;
    rlgchPt->rlg_data[nj-i-1]->jnt = J;
    rlgchPt->rlg_data[nj-i-1]->num_dof_jnt = 0;
    
    rlgchPt->rlg_data[nj-i-1]->dat[0] = r * sin(al);    /* d to axis */
    rlgchPt->rlg_data[nj-i-1]->dat[1] = r * cos(al);    /* shift */  
    rlgchPt->rlg_data[nj-i-1]->dat[2] = M_PI - al;      /* bond angle */  
    rlgchPt->rlg_data[nj-i-1]->lmax = r;                /* bond length */

    // max.length (ANGLES ARE SET AT 180 NOW !)
    p3d_vectSub(posJp,poslast,pos_diff);
    rlgchPt->rlg_data[nj-i-1]->shell.rext = (double) p3d_vectNorm(pos_diff);
    if(strcmp(ct->namecntrt,"p3d_in_sphere")==0) {
      rlgchPt->rlg_data[nj-i-1]->shell.rext += ct->argu_d[3];
    }
    if(strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) {
      rlgchPt->rlg_data[nj-i-1]->shell.rext += ct->argu_d[1];
    }

    /* updates */
    i2++;
    i++;
    J = J->prev_jnt;
  }

  // for min.length
  J = Jfaj;
  // identify peptide bond
  i2 = 0;
  p3d_jnt_get_dof_bounds(J->next_jnt[J->n_next_jnt - 1]->next_jnt[J->next_jnt[J->n_next_jnt - 1]->n_next_jnt - 1],
			 0,&vmin,&vmax);
  while((vmax-vmin) > M_PI/2.0) {
    J = J->next_jnt[J->n_next_jnt - 1];
    p3d_jnt_get_dof_bounds(J->next_jnt[J->n_next_jnt - 1]->next_jnt[J->next_jnt[J->n_next_jnt - 1]->n_next_jnt - 1],
			   0,&vmin,&vmax);
    i2++;
  }
  ct->argu_i[5] = i2;    // identifier for generation

  // compute shell.rint and shell.alpha
  // case of fixed orientation (p3d_6R_bio_ik_nopep)
  if(strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep")==0 || strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep_new")==0) {
    bio_loop_set_shells_rint_and_alpha(rlgchPt,10,nj,i2,0.0);
  }
  else if(strcmp(ct->namecntrt,"p3d_in_sphere")==0) {
    bio_loop_set_shells_rint_and_alpha(rlgchPt,1,nj,i2,ct->argu_d[3]);
  }
  else if(strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) {
    bio_loop_set_shells_rint_and_alpha(rlgchPt,1,nj,i2,ct->argu_d[1]);
  }
  else {
    MY_FREE(rlgchPt,p3d_rlg_chain,1);
    return (FALSE);
  }

  //////////////////////////////////
/*   for(i=0; i<nj; i++) { */
/*     printf("\n## SHELL J%d ###\n\n",rlgchPt->rlg_data[i]->jnt->num); */
/*     printf("  bond length = %f\n",rlgchPt->rlg_data[i]->lmax); */
/*     printf("  bond angle  = %f\n",rlgchPt->rlg_data[i]->dat[2]); */
/*     printf("  shell.rext  = %f\n",rlgchPt->rlg_data[i]->shell.rext); */
/*     printf("  shell.rint  = %f\n",rlgchPt->rlg_data[i]->shell.rint); */
/*   } */
  //////////////////////////////////
  
  
  ct->rlgPt->rlgchPt = rlgchPt; 
  
  return(TRUE);
}

/*****************************************************************************/
/*****************************************************************************/


static void bio_loop_set_shells_rint_and_alpha(pp3d_rlg_chain rlgchPt, int njxla, int nja, int i2, double error)
{
  /* NOTES :
     -> this is a very simple approach using constant values ( supposed < than real ones !)
     -> other solution could be to make tables with the values of dihedral angles
        corresponding to minimum extension
	the values in these tables are also approximated -> reduce the obtained distance (a %)
  */ 
  
  int i;
  double md[12];

  md[0] = 0.0; 
  if(njxla < 3)
    md[1] = rlgchPt->rlg_data[nja-2]->shell.rext - error;
  if(njxla < 4)
    md[2] = rlgchPt->rlg_data[nja-3]->shell.rext - error;
  
  if(i2 == 0) {
    md[3] = 2.5; 
    md[4] = 2.0; 
    md[5] = 3.3; 
    md[6] = 3.6; 
    md[7] = 2.6; 
    md[8] = 3.0; 
    md[9] = 2.2; 
    md[10] = 1.0; 
    md[11] = 0.8; 
  }
  else if(i2 == 1) {
    md[3] = 2.5; 
    md[4] = 4.0; 
    md[5] = 4.4; 
    md[6] = 3.4; 
    md[7] = 4.1; 
    md[8] = 3.4; 
    md[9] = 2.2; 
    md[10] = 2.0; 
    md[11] = 0.8; 
  }
  else {   // i2 == 2
    md[3] = 3.6; 
    md[4] = 3.8; 
    md[5] = 3.1; 
    md[6] = 4.3; 
    md[7] = 4.0; 
    md[8] = 3.0; 
    md[9] = 3.0; 
    md[10] = 1.8; 
    md[11] = 0.8; 
  }

  for(i=0; i < nja; i++) {
    if((njxla + i) < 13) {
      rlgchPt->rlg_data[nja-i-1]->shell.rint = md[njxla+i-1] - error;
      if(rlgchPt->rlg_data[nja-i-1]->shell.rint < 0.0)
	rlgchPt->rlg_data[nja-i-1]->shell.rint = 0.0;
    }
    else {
      rlgchPt->rlg_data[nja-i-1]->shell.rint = 0.0;
    }

    // consider ALWAYS full shell !!!
    rlgchPt->rlg_data[nja-i-1]->shell.alpha = M_PI;    
  }  

}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

static int bio_get_endframe_pos(p3d_rob *robPt, p3d_cntrt *ct, p3d_vector3 Xe)
{
  p3d_matrix4 Frel,Fabs;

  if(strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep_new")==0) {
    Frel[0][0] = ct->argu_d[0]; Frel[0][1] = ct->argu_d[3]; Frel[0][2] = ct->argu_d[6]; Frel[0][3] = ct->argu_d[9]; 
    Frel[1][0] = ct->argu_d[1]; Frel[1][1] = ct->argu_d[4]; Frel[1][2] = ct->argu_d[7]; Frel[1][3] = ct->argu_d[10]; 
    Frel[2][0] = ct->argu_d[2]; Frel[2][1] = ct->argu_d[5]; Frel[2][2] = ct->argu_d[8]; Frel[2][3] = ct->argu_d[11]; 
    Frel[3][0] = 0.0;           Frel[3][1] = 0.0;           Frel[3][2] = 0.0;           Frel[3][3] = 1.0; 
    
    p3d_mat4Mult(ct->actjnts[0]->abs_pos,Frel,Fabs);
    Xe[0] = Fabs[0][3];
    Xe[1] = Fabs[1][3];
    Xe[2] = Fabs[2][3];
  }
  else if(strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep")==0) {
    p3d_jnt_get_cur_vect_point(ct->actjnts[0], Xe);
  }
  else if(strcmp(ct->namecntrt,"p3d_in_sphere")==0) {
    Xe[0] = ct->argu_d[0];
    Xe[1] = ct->argu_d[1];
    Xe[2] = ct->argu_d[2];
  }
  else if(strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) {
    p3d_mat4Mult(robPt->joints[ct->argu_i[0]]->abs_pos,ct->Tbase,Fabs);
    Xe[0] = Fabs[0][3];
    Xe[1] = Fabs[1][3];
    Xe[2] = Fabs[2][3];
  }  

  return 1;
}


/*****************************************************************************/

int bio_compute_joint_closure_range(p3d_jnt *jntPt, p3d_cntrt *ctPt, int irlgd, 
				    int *nint, double *intmin, double *intmax)
{
  pp3d_rlg_chain rlgchPt;
  int go_on;
  p3d_vector3 Xj,axisj,Xe,Xjp,vjpe;
  p3d_vector3 axis_prevj,axisjxvjpe,nrefp,vfdt;
  static p3d_matrix4 Tshift = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  double distjpe,ang_vjpeaxisj;
  double dist_p,dist_pp,rmlm_pr,rmlM_pr,l_pr;
  double max_lim,min_lim,refofsym;
  double vmin,vmax;
  double absmin;
  int i,j,imin;
  int iv1=0,iv2=0;
  p3d_matrix2 interv1,interv2;

  rlgchPt = ctPt->rlgPt->rlgchPt;

  // identify the index of the rlg_data (when not given as argument)  
  if(irlgd < 0) {
    go_on = 1;
    irlgd = 0;
    while(go_on && (irlgd < rlgchPt->nlinksrlgch)) {
      if(rlgchPt->rlg_data[irlgd]->jnt == jntPt)
	go_on = 0;
      else
	irlgd++;
    }
    if(go_on)
      // the joint is not involved in the constraint !!!
      return 0;
  }

  // end-frame position
  bio_get_endframe_pos(jntPt->rob,ctPt,Xe);

  p3d_jnt_get_cur_vect_point(jntPt, Xj);
  p3d_jnt_get_dof_cur_axis(jntPt,0,axisj);   
  // NOTE : axisj is supposed to be normalized !!! 
  Tshift[0][3] = axisj[0] * rlgchPt->rlg_data[irlgd]->dat[1];
  Tshift[1][3] = axisj[1] * rlgchPt->rlg_data[irlgd]->dat[1];
  Tshift[2][3] = axisj[2] * rlgchPt->rlg_data[irlgd]->dat[1];
  p3d_xformPoint(Tshift,Xj,Xjp);
  p3d_vectSub(Xe,Xjp,vjpe);
    
  distjpe = sqrt(SQR(vjpe[0])+SQR(vjpe[1])+SQR(vjpe[2]));
  ang_vjpeaxisj = acos(p3d_vectDotProd(vjpe,axisj)/(p3d_vectNorm(vjpe)*p3d_vectNorm(axisj)));
  dist_pp = fabs(distjpe * sin(ang_vjpeaxisj));     /* pp -> perpendicular to axis */  
  dist_p = fabs(distjpe * cos(ang_vjpeaxisj));      /* p  -> parallel to axis      */
  
  /* if dist_p > shell.rext -> can't close */
  if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
    //printf("Salgo CAUSA A en jnt %d\n",jntPt->num);
    return 0;
  }

  /* plane proyections */
  l_pr = rlgchPt->rlg_data[irlgd]->dat[0];
  rmlM_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));    
  if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
    rmlm_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p));
  else
    rmlm_pr = 0.0;

  /* if dist_pp > rmlM_pr + l_pr   -> can't close */
  /* if l_pr > rmlM_pr + dist_pp   -> can't close */
  /* if dist_pp + l_pr < rmlm_pr -> can't close */
  //PRUEBAKKKK    
  if((dist_pp > rmlM_pr + l_pr)||
     (l_pr > rmlM_pr + dist_pp)||
     (dist_pp + l_pr < rmlm_pr)) {
    //printf("Salgo CAUSA B en jnt %d\n",jntPt->num);
    return 0;
  }

  /* calculate max_lim of interval */
  if((dist_pp == 0.0) ||  // <- can't have references for angles
     (dist_pp + l_pr <= rmlM_pr))  {
    max_lim = M_PI;
  }
  else {
    max_lim = acos((SQR(dist_pp)+SQR(l_pr)-SQR(rmlM_pr))/(2*dist_pp*l_pr));
  }

  /* calculate min_lim of interval */
  if(rmlm_pr > 0.0) {
    if((dist_pp == 0.0) ||
       (dist_pp >= rmlm_pr + l_pr) ||
       (dist_pp + rmlm_pr <= l_pr)) {
      min_lim = 0.0;
    }
    else {
      min_lim = acos((SQR(dist_pp)+SQR(l_pr)-SQR(rmlm_pr))/(2*dist_pp*l_pr));
    }
  }
  else { 
    min_lim = 0.0;
  }

  //PRUEBAKKKK
  //min_lim = 0.0;
  

  /* calculate intervals */
  p3d_jnt_get_dof_bounds(jntPt,0,&vmin,&vmax);
  if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
    *nint = 1;
    intmin[0] = vmin;
    intmax[0] = vmax;
  }
  else {
    /* compute angle of vjpe in joint's reference */
    // NOTE : a joint preceding faj is needed for references
    p3d_jnt_get_dof_cur_axis(jntPt->prev_jnt,0,axis_prevj);   
    p3d_vectXprod(axis_prevj,axisj,nrefp);
    p3d_vectXprod(axisj,vjpe,axisjxvjpe);
    refofsym = acos(p3d_vectDotProd(nrefp,axisjxvjpe)/(p3d_vectNorm(nrefp)*p3d_vectNorm(axisjxvjpe)));
    p3d_vectXprod(nrefp,axisjxvjpe,vfdt);
    if(!p3d_same_sign_vect(vfdt,axisj))
      refofsym = - refofsym;
    
    if(min_lim <= 0.0) {
      iv1 = p3d_inter_ang_regions_II(vmin,vmax,refofsym-max_lim,refofsym+max_lim,interv1);
      if(!iv1) {
	//printf("Salgo CAUSA C en jnt %d\n",jntPt->num);
	return 0;
      }
      iv2 = 0;
    }
    else {
      iv1 = p3d_inter_ang_regions_II(vmin,vmax,refofsym+min_lim,refofsym+max_lim,interv1);
      iv2 = p3d_inter_ang_regions_II(vmin,vmax,refofsym-max_lim,refofsym-min_lim,interv2);
      if((!iv1)&&(!iv2)) {
	//printf("Salgo CAUSA D en jnt %d\n",jntPt->num);
	return 0; 
      }
    }
    *nint = 0;
    intmin[*nint] = interv1[0][0];
    intmax[*nint] = interv1[0][1];
    (*nint)++;
    if(interv1[1][1] != 23.0) {
      intmin[*nint] = interv1[1][0];
      intmax[*nint] = interv1[1][1];
      (*nint)++;
    }
    if(iv2) {
      intmin[*nint] = interv2[0][0];
      intmax[*nint] = interv2[0][1];
      (*nint)++;
      if(interv2[1][1] != 23.0) {
	intmin[*nint] = interv2[1][0];
	intmax[*nint] = interv2[1][1];
	(*nint)++;
      }
    }
    
    for(i=0; i<*nint; i++) {
      if((intmax[i] - intmin[i]) >= ((2.0*M_PI) - EPS6)) {
	intmin[i] = 0.0;
	intmax[i] = (2.0*M_PI);
      } 
      if(intmax[i] > (2.0*M_PI)) {
	intmin[i] -= (2.0*M_PI);
	intmax[i] -= (2.0*M_PI);
      }
      if(intmin[i] < -(2.0*M_PI)) {
	intmin[i] += (2.0*M_PI);
	intmax[i] += (2.0*M_PI);
      }
    }
    
    /* order intervals */
    j = 0;
    while(j < (*nint-1)) {
      imin = j;
      absmin = intmin[j];
      for(i=j+1; i<*nint; i++) {
	if(intmin[i] < absmin) {
	  imin = i;
	  absmin = intmin[i];
	}
      }
      if(imin != j) {
	intmin[imin] = intmin[j];
	intmin[j] = absmin;
	absmin = intmax[imin];
	intmax[imin] = intmax[j];
	intmax[j] = absmin;
      }
      j++;
    }
  }
  return 1;
}


/*****************************************************************************/
/*****************************************************************************/

int bio_generate_ct_conf(p3d_rob *robotPt, p3d_cntrt *ct, configPt q) 
{
  return(bio_generate_active_part_of_loop(ct,q,NULL));
  //return(bio_generate_FREE_active_part_of_loop(ct,q,NULL));  // modif Juan (for Angela)
}


/*****************************************************************************/


static int bio_generate_active_part_of_loop_OLD(p3d_cntrt *ct, configPt q)
{
  pp3d_rlg_chain rlgchPt;
  p3d_jnt *Jfaj, *Jlaj, *Jej, *J;
  p3d_vector3 Xj,axisj,Xe,Xjp,vjpe;
  p3d_vector3 axis_prevj,axisjxvjpe,nrefp,vfdt;
  p3d_matrix4 Frel,Fabs;
  static p3d_matrix4 Tshift = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  int irlgd,i2;
  double distjpe,ang_vjpeaxisj;
  double dist_p,dist_pp,rmlm_pr,rmlM_pr,l_pr;
  double max_lim,min_lim,refofsym;
  double vmin,vmax,value;
  double intmin[4],intmax[4];
  double absmin;
  int i,j,imin,nint;
  p3d_matrix2 interv1,interv2;
  int iv1=0,iv2=0;
  p3d_rob *robPt;
  //static int nok=0;


  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  /* restore bio_saved_conf */
/*   p3d_copy_config_into(robPt,bio_saved_conf,&q); */
/*   p3d_set_robot_config(robPt,q); */
/*   p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); */

  rlgchPt = ct->rlgPt->rlgchPt;

  Jfaj = rlgchPt->rlg_data[0]->jnt;
  Jlaj = rlgchPt->rlg_data[rlgchPt->nlinksrlgch - 1]->jnt;

  // end-frame position
  if(strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep_new")==0) {
    Frel[0][0] = ct->argu_d[0]; Frel[0][1] = ct->argu_d[3]; Frel[0][2] = ct->argu_d[6]; Frel[0][3] = ct->argu_d[9]; 
    Frel[1][0] = ct->argu_d[1]; Frel[1][1] = ct->argu_d[4]; Frel[1][2] = ct->argu_d[7]; Frel[1][3] = ct->argu_d[10]; 
    Frel[2][0] = ct->argu_d[2]; Frel[2][1] = ct->argu_d[5]; Frel[2][2] = ct->argu_d[8]; Frel[2][3] = ct->argu_d[11]; 
    Frel[3][0] = 0.0;           Frel[3][1] = 0.0;           Frel[3][2] = 0.0;           Frel[3][3] = 1.0; 
    
    p3d_mat4Mult(ct->actjnts[0]->abs_pos,Frel,Fabs);
    Xe[0] = Fabs[0][3];
    Xe[1] = Fabs[1][3];
    Xe[2] = Fabs[2][3];
  }
  else if(strcmp(ct->namecntrt,"p3d_6R_bio_ik_nopep")==0) {
    Jej = ct->actjnts[0];
    p3d_jnt_get_cur_vect_point(Jej, Xe);
  }
  else if(strcmp(ct->namecntrt,"p3d_in_sphere")==0) {
    Xe[0] = ct->argu_d[0];
    Xe[1] = ct->argu_d[1];
    Xe[2] = ct->argu_d[2];
  }
  else if(strcmp(ct->namecntrt,"bio_bkb_Hbond_cntrt")==0) {
    p3d_mat4Mult(robPt->joints[ct->argu_i[0]]->abs_pos,ct->Tbase,Fabs);
    Xe[0] = Fabs[0][3];
    Xe[1] = Fabs[1][3];
    Xe[2] = Fabs[2][3];
  }  

  /* Set end-frame configuration (if active flag) */
/*   if(ct->argu_i[0]) { */
/*     for(i=0; i<6; i++) { */
/*       value = ct->argu_d[i]; */
/*       p3d_jnt_set_dof(Jej,i,value);       */
/*       q[Jej->index_dof + i] = value; */
/*     } */
/*   } */
/*   p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); */  //NECESARIO FUERA DEL IF ???

  /* Generation for faj to laj */
  /* generate for phi and psi and set omega at 180 !!! */
  J = Jfaj;
  irlgd = 0;
  if(ct->argu_i[5] == 0)
    i2 = 0;
  else if(ct->argu_i[5] == 1)
    i2 = 2;
  else
    i2 = 1;
  while((J != NULL)&&(J->prev_jnt != Jlaj)) {
    if(i2 == 2) {
      p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
      if(vmax < M_PI)
	q[J->index_dof] = - M_PI;
      else
	q[J->index_dof] = M_PI;
      irlgd++;
      p3d_jnt_set_dof(J,0,q[J->index_dof]);
      //p3d_update_this_robot_pos(J->rob);    // for draw
      //p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
      // NEW UPDATE ********
      J->prev_jnt->pos_updated = TRUE;
      J->pos_updated = FALSE;
      for(i=0;i<J->n_link_jnt;i++) {
	if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]))
	  break;
      }
      for(i=0;i<J->n_link_jnt;i++) {
	p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]);
      }
      J->abs_pos_modified = FALSE;
      J->abs_pos_before_jnt_modified = FALSE;
      J->mat_modified = FALSE;
      J->pos_updated = FALSE;
      J->prev_jnt->pos_updated = FALSE;
      for(i=0;i<J->n_next_jnt;i++) {
	J->next_jnt[i]->pos_updated = FALSE;
      }
      // *******************
      J = J->next_jnt[J->n_next_jnt - 1];
      if((J == NULL)||(J->prev_jnt == Jlaj))
	break;     
      i2 = 0;
    }

    p3d_jnt_get_cur_vect_point(J, Xj);
    p3d_jnt_get_dof_cur_axis(J,0,axisj);   
    // NOTE : axisj is supposed to be normalized !!! 
    Tshift[0][3] = axisj[0] * rlgchPt->rlg_data[irlgd]->dat[1];
    Tshift[1][3] = axisj[1] * rlgchPt->rlg_data[irlgd]->dat[1];
    Tshift[2][3] = axisj[2] * rlgchPt->rlg_data[irlgd]->dat[1];
    p3d_xformPoint(Tshift,Xj,Xjp);
    p3d_vectSub(Xe,Xjp,vjpe);
    
    distjpe = sqrt(SQR(vjpe[0])+SQR(vjpe[1])+SQR(vjpe[2]));
    ang_vjpeaxisj = acos(p3d_vectDotProd(vjpe,axisj)/(p3d_vectNorm(vjpe)*p3d_vectNorm(axisj)));
    dist_pp = fabs(distjpe * sin(ang_vjpeaxisj));     /* pp -> perpendicular to axis */  
    dist_p = fabs(distjpe * cos(ang_vjpeaxisj));      /* p  -> parallel to axis      */

    /* if dist_p > shell.rext -> can't close */
    if(dist_p > rlgchPt->rlg_data[irlgd]->shell.rext) {
      //printf("Salgo en J%d\n",J->num);
      return 0;
    }

    /* plane proyections */
    l_pr = rlgchPt->rlg_data[irlgd]->dat[0];
    rmlM_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rext)-SQR(dist_p));    
    if(rlgchPt->rlg_data[irlgd]->shell.rint > dist_p) 
      rmlm_pr = sqrt(SQR(rlgchPt->rlg_data[irlgd]->shell.rint)-SQR(dist_p));
    else
      rmlm_pr = 0.0;

    /* if dist_pp > rmlM_pr + l_pr   -> can't close */
    /* if l_pr > rmlM_pr + dist_pp   -> can't close */
    /* if dist_pp + l_pr < rmlm_pr -> can't close */
    //PRUEBAKKKK    
    if((dist_pp > rmlM_pr + l_pr)||
       (l_pr > rmlM_pr + dist_pp)||
       (dist_pp + l_pr < rmlm_pr)) {
      //printf("Salgo en J%d\n",J->num);
      return 0;
    }

    /* calculate max_lim of interval */
    if((dist_pp == 0.0) ||  // <- can't have references for angles
       (dist_pp + l_pr <= rmlM_pr))  {
      max_lim = M_PI;
    }
    else {
      max_lim = acos((SQR(dist_pp)+SQR(l_pr)-SQR(rmlM_pr))/(2*dist_pp*l_pr));
    }

    /* calculate min_lim of interval */
    if(rmlm_pr > 0.0) {
      if((dist_pp == 0.0) ||
	 (dist_pp >= rmlm_pr + l_pr) ||
	 (dist_pp + rmlm_pr <= l_pr)) {
	min_lim = 0.0;
      }
      else {
	min_lim = acos((SQR(dist_pp)+SQR(l_pr)-SQR(rmlm_pr))/(2*dist_pp*l_pr));
      }
    }
    else { 
      min_lim = 0.0;
    }

    //PRUEBAKKKK
    //min_lim = 0.0;

    /* calculate intervals */
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if((min_lim <= 0.0)&&(max_lim >= M_PI)) {
      q[J->index_dof] = p3d_random(vmin,vmax); 
    }
    else {
      /* compute angle of vjpe in joint's reference */
      // NOTE : a joint preceding faj is needed for references
      p3d_jnt_get_dof_cur_axis(J->prev_jnt,0,axis_prevj);   
      p3d_vectXprod(axis_prevj,axisj,nrefp);
      p3d_vectXprod(axisj,vjpe,axisjxvjpe);
      refofsym = acos(p3d_vectDotProd(nrefp,axisjxvjpe)/(p3d_vectNorm(nrefp)*p3d_vectNorm(axisjxvjpe)));
      p3d_vectXprod(nrefp,axisjxvjpe,vfdt);
      if(!p3d_same_sign_vect(vfdt,axisj))
	refofsym = - refofsym;
      
       if(min_lim <= 0.0) {
	 iv1 = p3d_inter_ang_regions_II(vmin,vmax,refofsym-max_lim,refofsym+max_lim,interv1);
	 if(!iv1) {
	   //printf("Salgo en J%d\n",J->num);
	   return 0;
	 }
	 iv2 = 0;
       }
       else {
	 iv1 = p3d_inter_ang_regions_II(vmin,vmax,refofsym+min_lim,refofsym+max_lim,interv1);
	 iv2 = p3d_inter_ang_regions_II(vmin,vmax,refofsym-max_lim,refofsym-min_lim,interv2);
	 if((!iv1)&&(!iv2)) {
	   //printf("Salgo en J%d\n",J->num);
	   return 0; 
	 }
       }
       nint = 0;
       intmin[nint] = interv1[0][0];
       intmax[nint] = interv1[0][1];
       nint++;
       if(interv1[1][1] != 23.0) {
	 intmin[nint] = interv1[1][0];
	 intmax[nint] = interv1[1][1];
         nint++;
       }
       if(iv2) {
	 intmin[nint] = interv2[0][0];
	 intmax[nint] = interv2[0][1];
	 nint++;
	 if(interv2[1][1] != 23.0) {
	   intmin[nint] = interv2[1][0];
	   intmax[nint] = interv2[1][1];
	   nint++;
	 }
       }

       for(i=0; i<nint; i++) {
	 if((intmax[i] - intmin[i]) >= ((2.0*M_PI) - EPS6)) {
	   intmin[i] = 0.0;
	   intmax[i] = (2.0*M_PI);
	 } 
	 if(intmax[i] > (2.0*M_PI)) {
	   intmin[i] -= (2.0*M_PI);
	   intmax[i] -= (2.0*M_PI);
	 }
	 if(intmin[i] < -(2.0*M_PI)) {
	   intmin[i] += (2.0*M_PI);
	   intmax[i] += (2.0*M_PI);
	 }
       }

       /* order intervals */
       j = 0;
       while(j < (nint-1)) {
	 imin = j;
	 absmin = intmin[j];
	 for(i=j+1; i<nint; i++) {
	   if(intmin[i] < absmin) {
	     imin = i;
	     absmin = intmin[i];
	   }
	 }
	 if(imin != j) {
	   intmin[imin] = intmin[j];
	   intmin[j] = absmin;
	   absmin = intmax[imin];
	   intmax[imin] = intmax[j];
	   intmax[j] = absmin;
	 }
	 j++;
       }

       value = p3d_random_in_several_ordered_intervals(nint,intmin,intmax);
       if(value < -M_PI)
	 value += (2.0*M_PI);
       else if(value > M_PI)
	 value -= (2.0*M_PI);

       q[J->index_dof] = value;
    }
          
    p3d_jnt_set_dof(J,0,q[J->index_dof]);
    //p3d_update_this_robot_pos(J->rob);    // for draw
    //p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
    // NEW UPDATE ********
    J->prev_jnt->pos_updated = TRUE;
    J->pos_updated = FALSE;
    for(i=0;i<J->n_link_jnt;i++) {
      if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]))
      break;
    }
    for(i=0;i<J->n_link_jnt;i++) {
      p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]);
    }
    J->abs_pos_modified = FALSE;
    J->abs_pos_before_jnt_modified = FALSE;
    J->mat_modified = FALSE;
    J->pos_updated = FALSE;
    J->prev_jnt->pos_updated = FALSE;
    for(i=0;i<J->n_next_jnt;i++) {
      J->next_jnt[i]->pos_updated = FALSE;
    }
    //printf("%d\n",J->num);
    // *******************
    
    //g3d_draw_allwin_active();

    if(J->next_jnt == NULL)
      J = NULL;
    else
      J = J->next_jnt[J->n_next_jnt - 1];
    irlgd++;
    i2++;
  }

  // last peptide bond
  p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
  if(vmax < M_PI)
    q[J->index_dof] = - M_PI;
  else
    q[J->index_dof] = M_PI;
  p3d_jnt_set_dof(J,0,q[J->index_dof]);
  //p3d_update_this_robot_pos(J->rob);    // for draw
  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
  // NEW UPDATE ********
  //for(i=0;i<J->n_link_jnt;i++) {
  //  if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]));
  //}
  // *******************

  //g3d_draw_allwin_active();

  //nok++;
  //printf("bio_rlg OK %d\n",nok);
  //printf("bio_rlg OK\n");

  return 1;
}

/*****************************************************************************/

static int bio_generate_active_part_of_loop_wrapper(p3d_cntrt *ct, configPt q) {
  return bio_generate_active_part_of_loop(ct, q, NULL);
}

static int bio_generate_active_part_of_loop(p3d_cntrt *ct, configPt q, p3d_jnt *Jfaj)
{
  pp3d_rlg_chain rlgchPt;
  p3d_jnt *Jlaj, *J;
  int irlgd,i2;
  double value;
  int nint,nint2;
  double intmin[4],intmax[4];
  double intmin2[4],intmax2[4];
 //int i,j;
  p3d_rob *robPt;
  int iench;
  p3d_cntrt *ect=NULL;
  double vmin,vmax;
  int go_on,count;
  static int partial_reshoot_count = 0;
  p3d_jnt *Jbase_reshoot = NULL;

  //static int nok=0;


  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  /* restore bio_saved_conf */
/*   p3d_copy_config_into(robPt,bio_saved_conf,&q); */
/*   p3d_set_robot_config(robPt,q); */
/*   p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); */

  // desactivate Hbond constraints 
  for(iench=0; iench<ct->nenchained; iench++) {
    ect = ct->enchained[iench];
    ect->active = 0;
  }

  rlgchPt = ct->rlgPt->rlgchPt;

  if(Jfaj == NULL) {
    Jfaj = rlgchPt->rlg_data[0]->jnt;
  }
  Jlaj = rlgchPt->rlg_data[rlgchPt->nlinksrlgch - 1]->jnt;

  /* Set end-frame configuration (if active flag) */
/*   if(ct->argu_i[0]) { */
/*     for(i=0; i<6; i++) { */
/*       value = ct->argu_d[i]; */
/*       p3d_jnt_set_dof(Jej,i,value);       */
/*       q[Jej->index_dof + i] = value; */
/*     } */
/*   } */
/*   p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); */  //NECESARIO FUERA DEL IF ???

  /* Generation for faj to laj */
  /* generate for phi and psi and set omega at 180 !!! */
  J = Jfaj;
  irlgd = 0;
  if(ct->argu_i[5] == 0)
    i2 = 0;
  else if(ct->argu_i[5] == 1)
    i2 = 2;
  else
    i2 = 1;
  while((J != NULL)&&(J->prev_jnt != Jlaj)) {
    if(i2 == 2) {
     p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
     if(vmax < M_PI)
	q[J->index_dof] = - M_PI;
      else
	q[J->index_dof] = M_PI;
      irlgd++;
      p3d_jnt_set_dof(J,0,q[J->index_dof]);
      p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
      //p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
      // NEW UPDATE ********
/*       J->prev_jnt->pos_updated = TRUE; */
/*       J->pos_updated = FALSE; */
/*       for(i=0;i<J->n_link_jnt;i++) { */
/* 	if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i])) */
/* 	  break; */
/*       } */
/*       for(i=0;i<J->n_link_jnt;i++) { */
/* 	p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]); */
/*       } */
/*       J->abs_pos_modified = FALSE; */
/*       J->abs_pos_before_jnt_modified = FALSE; */
/*       J->mat_modified = FALSE; */
/*       J->pos_updated = FALSE; */
/*       J->prev_jnt->pos_updated = FALSE; */
/*       for(i=0;i<J->n_next_jnt;i++) { */
/* 	J->next_jnt[i]->pos_updated = FALSE; */
/*       } */
/*       // object */
/*       if (J->o != NULL) { */
/* 	update_robot_obj_pos(J->o);  */
/*       } */
      //J->pos_obj_modified = FALSE;    
      // *******************
      J = J->next_jnt[J->n_next_jnt - 1];
      if((J == NULL)||(J->prev_jnt == Jlaj))
	break;     
      i2 = 0;
    }

    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if(vmin != vmax) {
      if(!bio_compute_joint_closure_range(J,ct,irlgd,&nint,intmin,intmax)) {
	//printf("Empty closure interval for main loop at J%d\n",J->num);
	return 0;
      }
      
      for(iench=0; iench<ct->nenchained; iench++) {
	ect = ct->enchained[iench];
	// if the joint associated with the "base" of the H bond has been alrady treated
	// and the joint associated with the "end" of the H bond has not been treated yet
	if((J->num > ect->argu_i[0]) && (J->num <= ect->argu_i[1])) {
	  // NOTE : irlgd is different for each cntrt !!!
	  if(!bio_compute_joint_closure_range(J,ect,-1,&nint2,intmin2,intmax2)) {
	    //printf("Empty closure interval for Hbond cntrt num %d at J%d\n",ect->num,J->num);
	    return 0;
	  }
	  // merge intervals : result in "intmin, intmax"
	  nint = p3d_merge_ang_intervals(nint,nint2,intmin,intmax,intmin2,intmax2);    
	  if(nint == 0) {
	    //printf("Empty intersection of closure interval at J%d\n",J->num);
	    return 0;
	  }
	  // activate Hbond constraints 
	  ect->active = 1;
	}
      }
      
      
      go_on = 1;
      count = 0;
      while(go_on && (count < 10)) {    // 10 SHOULD BE A PARAMETER !!!!
	count++;
	// sample value
	value = p3d_random_in_several_ordered_intervals(nint,intmin,intmax);
	if(value < -M_PI)
	  value += (2.0*M_PI);
	else if(value > M_PI)
	  value -= (2.0*M_PI);
	
	q[J->index_dof] = value;
	
	p3d_jnt_set_dof(J,0,q[J->index_dof]);
	p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
	//p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	// NEW UPDATE ********
	/*     J->prev_jnt->pos_updated = TRUE; */
	/*     J->pos_updated = FALSE; */
	/*     for(i=0;i<J->n_link_jnt;i++) { */
	/*       if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i])) */
	/*       break; */
	/*     } */
	/*     for(i=0;i<J->n_link_jnt;i++) { */
	/*       p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]); */
	/*     } */
	/*     J->abs_pos_modified = FALSE; */
	/*     J->abs_pos_before_jnt_modified = FALSE; */
	/*     J->mat_modified = FALSE; */
	/*     J->pos_updated = FALSE; */
	/*     J->prev_jnt->pos_updated = FALSE; */
	/*     for(i=0;i<J->n_next_jnt;i++) { */
	/*       J->next_jnt[i]->pos_updated = FALSE; */
	/*     } */
	/*      */
	/*     if (J->o != NULL) { */
	/*       update_robot_obj_pos(J->o);  */
	/*     } */
	//J->pos_obj_modified = FALSE;    
	//printf("%d\n",J->num);
	// *******************
	
	// check Hbond constraints of active subchain when generating
	//  fo the joint associated with the "end" of the H bond 
	go_on = 0;
	for(iench=0; iench<ct->nenchained; iench++) {
	  ect = ct->enchained[iench];
	  if(J->num == ect->argu_i[1]) {
	    if(!(*ect->fct_cntrt)(ect,-1,NULL,0.0)) {
	      // restart the generation of the active subchain configuration
	      go_on = 1;
	      break;
	    }
	    else {
	      //printf("Hbond cntrt num %d OK\n",ect->num);
	      Jbase_reshoot = J->next_jnt[J->n_next_jnt - 1];
	    }
	  }
	}
      }
      if(go_on) {
	// restart the generation of the active subchain configuration
	// (other possibility should be to restart from the last Hbond)
	//printf("Hbond cntrt num %d not satisfied at J%d\n",ect->num,J->num);
	if(partial_reshoot_count < 10) {  // 10 SHOULD BE A PARAMETER
	  partial_reshoot_count ++;
	  bio_generate_active_part_of_loop(ct,q,Jbase_reshoot);
	}
	partial_reshoot_count = 0;
	return 0;
      }
      partial_reshoot_count = 0;
      
      ////// 4 DEBUG 	
      //g3d_draw_allwin_active();
      /////////
    }	

    if(J->next_jnt == NULL)
      J = NULL;
    else
      J = J->next_jnt[J->n_next_jnt - 1];
    irlgd++;
    i2++;

  }

  // last peptide bond
  p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
  if(vmax < M_PI)
    q[J->index_dof] = - M_PI;
  else
    q[J->index_dof] = M_PI;
  p3d_jnt_set_dof(J,0,q[J->index_dof]);
  //p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 

  //g3d_draw_allwin_active();

  //nok++;
  //printf("bio_rlg OK %d\n",nok);
  //printf("bio_rlg OK\n");

  return 1;
}


/*****************************************************************************/

static int bio_generate_FREE_active_part_of_loop(p3d_cntrt *ct, configPt q, p3d_jnt *Jfaj)
{
  pp3d_rlg_chain rlgchPt;
  p3d_jnt *Jlaj, *J;
  int irlgd,i2;
  double value;
  int nint,nint2;
  double intmin[4],intmax[4];
  double intmin2[4],intmax2[4];
 //int i,j;
  p3d_rob *robPt;
  int resind;
  int tries_elem;
  int iench;
  p3d_cntrt *ect=NULL;
  double vmin,vmax;
  int go_on,count;
  static int partial_reshoot_count = 0;
  p3d_jnt *Jbase_reshoot = NULL;

  //static int nok=0;


  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  /* restore bio_saved_conf */
/*   p3d_copy_config_into(robPt,bio_saved_conf,&q); */
/*   p3d_set_robot_config(robPt,q); */
/*   p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); */

  // desactivate Hbond constraints 
  for(iench=0; iench<ct->nenchained; iench++) {
    ect = ct->enchained[iench];
    ect->active = 0;
  }

  rlgchPt = ct->rlgPt->rlgchPt;

  if(Jfaj == NULL) {
    Jfaj = rlgchPt->rlg_data[0]->jnt;
  }
  Jlaj = rlgchPt->rlg_data[rlgchPt->nlinksrlgch - 1]->jnt;

  /* Set end-frame configuration (if active flag) */
/*   if(ct->argu_i[0]) { */
/*     for(i=0; i<6; i++) { */
/*       value = ct->argu_d[i]; */
/*       p3d_jnt_set_dof(Jej,i,value);       */
/*       q[Jej->index_dof + i] = value; */
/*     } */
/*   } */
/*   p3d_update_this_robot_pos_without_cntrt_and_obj(robPt); */  //NECESARIO FUERA DEL IF ???

  // active collision for first element
  resind = get_AAnumber_from_jnt(Jfaj);
  activate_bb_rigid(robPt->num, resind);
  tries_elem = 0;

  /* Generation for faj to laj */
  /* generate for phi and psi and set omega at 180 !!! */
  J = Jfaj;
  irlgd = 0;
  if(ct->argu_i[5] == 0)
    i2 = 0;
  else if(ct->argu_i[5] == 1)
    i2 = 2;
  else
    i2 = 1;
  while((J != NULL)&&(J->prev_jnt != Jlaj)) {
    if(i2 == 2) {
     p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
     if(vmax < M_PI)
	q[J->index_dof] = - M_PI;
      else
	q[J->index_dof] = M_PI;
      irlgd++;
      p3d_jnt_set_dof(J,0,q[J->index_dof]);
      p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
      //p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
      // NEW UPDATE ********
/*       J->prev_jnt->pos_updated = TRUE; */
/*       J->pos_updated = FALSE; */
/*       for(i=0;i<J->n_link_jnt;i++) { */
/* 	if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i])) */
/* 	  break; */
/*       } */
/*       for(i=0;i<J->n_link_jnt;i++) { */
/* 	p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]); */
/*       } */
/*       J->abs_pos_modified = FALSE; */
/*       J->abs_pos_before_jnt_modified = FALSE; */
/*       J->mat_modified = FALSE; */
/*       J->pos_updated = FALSE; */
/*       J->prev_jnt->pos_updated = FALSE; */
/*       for(i=0;i<J->n_next_jnt;i++) { */
/* 	J->next_jnt[i]->pos_updated = FALSE; */
/*       } */
/*       // object */
/*       if (J->o != NULL) { */
/* 	update_robot_obj_pos(J->o);  */
/*       } */
      //J->pos_obj_modified = FALSE;    
      // *******************
      J = J->next_jnt[J->n_next_jnt - 1];
      if((J == NULL)||(J->prev_jnt == Jlaj))
	break;     
      i2 = 0;
    }

    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if(vmin != vmax) {
      if(!bio_compute_joint_closure_range(J,ct,irlgd,&nint,intmin,intmax)) {
	//printf("Empty closure interval for main loop at J%d\n",J->num);
	return 0;
      }
      
      for(iench=0; iench<ct->nenchained; iench++) {
	ect = ct->enchained[iench];
	// if the joint associated with the "base" of the H bond has been alrady treated
	// and the joint associated with the "end" of the H bond has not been treated yet
	if((J->num > ect->argu_i[0]) && (J->num <= ect->argu_i[1])) {
	  // NOTE : irlgd is different for each cntrt !!!
	  if(!bio_compute_joint_closure_range(J,ect,-1,&nint2,intmin2,intmax2)) {
	    //printf("Empty closure interval for Hbond cntrt num %d at J%d\n",ect->num,J->num);
	    return 0;
	  }
	  // merge intervals : result in "intmin, intmax"
	  nint = p3d_merge_ang_intervals(nint,nint2,intmin,intmax,intmin2,intmax2);    
	  if(nint == 0) {
	    //printf("Empty intersection of closure interval at J%d\n",J->num);
	    return 0;
	  }
	  // activate Hbond constraints 
	  ect->active = 1;
	}
      }
      
      
      go_on = 1;
      count = 0;
      while(go_on && (count < 10)) {    // 10 SHOULD BE A PARAMETER !!!!
	count++;
	// sample value
	value = p3d_random_in_several_ordered_intervals(nint,intmin,intmax);
	if(value < -M_PI)
	  value += (2.0*M_PI);
	else if(value > M_PI)
	  value -= (2.0*M_PI);
	
	q[J->index_dof] = value;
	
	p3d_jnt_set_dof(J,0,q[J->index_dof]);
	p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
	//p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 
	// NEW UPDATE ********
	/*     J->prev_jnt->pos_updated = TRUE; */
	/*     J->pos_updated = FALSE; */
	/*     for(i=0;i<J->n_link_jnt;i++) { */
	/*       if(p3d_jnt_calc_mat_pos(J->link_jnt_arr[i])) */
	/*       break; */
	/*     } */
	/*     for(i=0;i<J->n_link_jnt;i++) { */
	/*       p3d_jnt_calc_mat_pos(J->link_jnt_arr[i]); */
	/*     } */
	/*     J->abs_pos_modified = FALSE; */
	/*     J->abs_pos_before_jnt_modified = FALSE; */
	/*     J->mat_modified = FALSE; */
	/*     J->pos_updated = FALSE; */
	/*     J->prev_jnt->pos_updated = FALSE; */
	/*     for(i=0;i<J->n_next_jnt;i++) { */
	/*       J->next_jnt[i]->pos_updated = FALSE; */
	/*     } */
	/*      */
	/*     if (J->o != NULL) { */
	/*       update_robot_obj_pos(J->o);  */
	/*     } */
	//J->pos_obj_modified = FALSE;    
	//printf("%d\n",J->num);
	// *******************
	
	// check Hbond constraints of active subchain when generating
	//  fo the joint associated with the "end" of the H bond 
	go_on = 0;
	for(iench=0; iench<ct->nenchained; iench++) {
	  ect = ct->enchained[iench];
	  if(J->num == ect->argu_i[1]) {
	    if(!(*ect->fct_cntrt)(ect,-1,NULL,0.0)) {
	      // restart the generation of the active subchain configuration
	      go_on = 1;
	      break;
	    }
	    else {
	      //printf("Hbond cntrt num %d OK\n",ect->num);
	      Jbase_reshoot = J->next_jnt[J->n_next_jnt - 1];
	    }
	  }
	}
      }
      if(go_on) {
	// restart the generation of the active subchain configuration
	// (other possibility should be to restart from the last Hbond)
	//printf("Hbond cntrt num %d not satisfied at J%d\n",ect->num,J->num);
	if(partial_reshoot_count < 10) {  // 10 SHOULD BE A PARAMETER
	  partial_reshoot_count ++;
	  bio_generate_FREE_active_part_of_loop(ct,q,Jbase_reshoot);
	}
	partial_reshoot_count = 0;
	return 0;
      }
      partial_reshoot_count = 0;
      
      ////// 4 DEBUG 	
      //g3d_draw_allwin_active();
      /////////
    }	

    if(J->next_jnt == NULL)
      J = NULL;
    else
      J = J->next_jnt[J->n_next_jnt - 1];
    irlgd++;
    i2++;

    // ************ collision detection *****************
    // if the last sampled joint value is PHI
    if(i2 == 1) {
      if(bio_bb_col(robPt->num, resind) > 0) {
	//printf("res %d collides\n",resind);
	////// 4 DEBUG 	
	//afficher_lescollisions();		
	/////////
	if(irlgd > 2) {
	  J = J->prev_jnt->prev_jnt->prev_jnt;
	  irlgd -= 3;
	  //i2 = 1; // it's OK
	}
	else {
	  J = Jfaj;
	  irlgd = 0;
	  if(ct->argu_i[5] == 0)
	    i2 = 0;
	  else if(ct->argu_i[5] == 1)
	    i2 = 2;
	  else
	    i2 = 1;
	}
	tries_elem++;
	if(tries_elem > 10) { // 10 SHOULD BE A PARAMETER !!!! ******
	  //printf("No free conf in closure interval at J%d\n",J->num);
	  return 0;
	}
      }
      else {
	//printf("res %d OK\n",resind);
	resind++;
	activate_bb_rigid(robPt->num, resind);
	tries_elem = 0;
      }
    }
    // **************************************************
  }

  // last peptide bond
  p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
  if(vmax < M_PI)
    q[J->index_dof] = - M_PI;
  else
    q[J->index_dof] = M_PI;
  p3d_jnt_set_dof(J,0,q[J->index_dof]);
  //p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
  p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob); 

  //g3d_draw_allwin_active();

  //nok++;
  //printf("bio_rlg OK %d\n",nok);
  //printf("bio_rlg OK\n");

  return 1;
}

static int bio_random_active_part_of_loop(p3d_cntrt *ct, configPt q)
{
  pp3d_rlg_chain rlgchPt;
  p3d_jnt *Jfaj, *Jlaj, *J;
  p3d_rob *robPt;
  double vmin,vmax;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  rlgchPt = ct->rlgPt->rlgchPt;

  Jfaj = rlgchPt->rlg_data[0]->jnt;
  Jlaj = rlgchPt->rlg_data[rlgchPt->nlinksrlgch - 1]->jnt;

  /* Generation for faj to laj */

  J = Jfaj;
  while((J != NULL)&&(J->prev_jnt != Jlaj)) {
    p3d_jnt_get_dof_bounds(J,0,&vmin,&vmax);
    if((vmax - vmin) < M_PI)
      q[J->index_dof] = M_PI;
    else
      q[J->index_dof] = p3d_random(vmin,vmax); 
    p3d_jnt_set_dof(J,0,q[J->index_dof]);
    if(J->next_jnt == NULL)
      J = NULL;
    else
      J = J->next_jnt[J->n_next_jnt - 1];
  }  

  return 1;

}


/**********************************************************************************/
// LOOP GEOMETRY

static int PRINT_LOOP_GEOM = 0;

void bio_set_PRINT_LOOP_GEOM(int val)
{
  PRINT_LOOP_GEOM = val;
}

int bio_get_PRINT_LOOP_GEOM(void)
{
  return(PRINT_LOOP_GEOM);
}


void bio_print_loop_geometry(p3d_cntrt *ct)
{
  p3d_jnt *Jf,*Jl;
  p3d_cntrt *fct;
  int ict;
  p3d_vector3 posJf,posJl;
  p3d_vector3 axisJf,axisJl,vectJfJl,neg_axisJf,vprod1,vprod2;
  double D,Theta,Delta,Rho;
  
  Jl = ct->pasjnts[ct->npasjnts - 1];
  ict = ct->num - 1;
  while((ict > -1) && (strcmp(ct->cntrt_manager->cntrts[ict]->namecntrt,"p3d_in_sphere") == 0))
    ict--;
  fct = ct->cntrt_manager->cntrts[ict + 1];
  Jf = fct->rlgPt->rlgchPt->rlg_data[0]->jnt;

  p3d_jnt_get_cur_vect_point(Jf,posJf);
  p3d_jnt_get_dof_cur_axis(Jf,0,axisJf);
  p3d_jnt_get_cur_vect_point(Jl,posJl);
  p3d_jnt_get_dof_cur_axis(Jl,0,axisJl);
  p3d_vectSub(posJl,posJf,vectJfJl);	
  D = (double) p3d_vectNorm(vectJfJl);
  p3d_vectNeg(axisJf,neg_axisJf);
  Theta = (180.0/M_PI) * acos(p3d_vectDotProd(axisJf,axisJl)/(p3d_vectNorm(axisJf)*p3d_vectNorm(axisJl)));
  p3d_vectXprod(axisJf,axisJl,vprod1);
  if(!p3d_same_sign_vect(vprod1,vectJfJl))
    Theta += 180.0;;
  Delta = (180.0/M_PI) * acos(p3d_vectDotProd(axisJf,vectJfJl)/(p3d_vectNorm(axisJf)*p3d_vectNorm(vectJfJl)));
  // SIGNO !!!???
  // Rho ???
  p3d_vectXprod(axisJf,vectJfJl,vprod1);
  p3d_vectXprod(axisJf,vprod1,vprod2);
  Rho = 90.0 - (180.0/M_PI) * acos(p3d_vectDotProd(axisJl,vprod2)/(p3d_vectNorm(axisJl)*p3d_vectNorm(vprod2)));
  if(Rho < 0.0)
    Rho += 360.0;

  printf("D = %f , Theta = %f , Delta = %f , Rho = %f\n",D,Theta,Delta,Rho);
}

/**********************************************************************************/
/**********************************************************************************/
// TESTS CYCLOOCTANE

static int bio_set_cyclooctane_rlg(int ctnum)
{
  // NOTE : joint frames will be always reachable in RLG !!! 
  return 1;
}


int bio_set_cyclooctane(void)
{
  p3d_rob *r;
  int ifpj,irj;
  int setok;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  ifpj = 3;
  irj = 10;
  if(!p3d_constraint("p3d_6R_bio_ik",1,&ifpj,1,&irj,0,NULL,0,NULL,-1,1)) {
    return 0;
  }
  
  //setok = bio_set_cyclooctane_rlg(r->cntrt_manager->ncntrts - 1);

  setok = 0;

  if(setok) {
    p3d_active_RLG_flags();
  }
  
  return (setok);
}

/**********************************************************************************/
/**********************************************************************************/
// TESTS CYCLOHEPTANE

static int bio_set_cycloheptane_rlg(int ctnum)
{
  // NOTE : joint frames will be always reachable in RLG !!! 
  return 1;
}


int bio_set_cycloheptane(void)
{
  p3d_rob *r;
  int ifpj,irj;
  int setok;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  ifpj = 2;
  irj = 9;
  if(!p3d_constraint("p3d_6R_bio_ik",1,&ifpj,1,&irj,0,NULL,0,NULL,-1,1)) {
    return 0;
  }
  
  //setok = bio_set_cyclooctane_rlg(r->cntrt_manager->ncntrts - 1);

  setok = 0;

  if(setok) {
    p3d_active_RLG_flags();
  }
  
  return (setok);
}


/**********************************************************************************/
/**********************************************************************************/
// DISTANCES

double bio_max_dof_dist_config_backbone_nopep(p3d_rob * robotPt, configPt q1, configPt q2)
{
  double lmax = -P3D_HUGE, ljnt = 0.0, l;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt * jntPt;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    // ONLY CONSIDER ACTIVE JOINTS !!!
    if(robotPt->cntrt_manager->in_cntrt[jntPt->index_dof] != 2) { 
      // WARNING !!! : TO BE MODIFIED DEPENDING ON THE FINAL VERSION OF THE BIO_TRANSLATOR
      if((strcmp(jntPt->name, "PHI") == 0) ||
	 (strcmp(jntPt->name, "PSI") == 0)) {
	for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	  ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, q1, q2));
	}
	if(ljnt > lmax)
	  lmax = ljnt;
      }
    }
  }
  l = sqrt(lmax);

  return l;
}


double bio_average_dof_dist_config_backbone_nopep(p3d_rob * robotPt, configPt q1, configPt q2)
{
  double ljnt = 0.0, l=0.0;
  int i, j, njnt = robotPt->njoints;
  p3d_jnt * jntPt;
  int nj=0;

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    // ONLY CONSIDER ACTIVE JOINTS !!!
    if(robotPt->cntrt_manager->in_cntrt[jntPt->index_dof] != 2) { 
      // WARNING !!! : TO BE MODIFIED DEPENDING ON THE FINAL VERSION OF THE BIO_TRANSLATOR
      if((strcmp(jntPt->name, "PHI") == 0) ||
	 (strcmp(jntPt->name, "PSI") == 0)) {
	for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	  ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, q1, q2));
	}
	nj++;
      }
    }
  }
  if(nj > 0)
    l = sqrt(ljnt) / ((double) nj);

  return l;
}


/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/
// LOOP CONFORMATIONAL SAMPLING 


static int bio_compute_all_bkb_sol_nopep_new(p3d_rob *robotPt, p3d_cntrt *ct, double **pass_configs, int *pass_index);


/* ------------------------------------------------------------------------------ */

// OLD VERSION : decoupled generation/collision for backbone
p3d_node* bio_shoot_loop_OLD(p3d_graph *graphPt)
{
  p3d_rob *robotPt = graphPt->rob;
  p3d_cntrt *ct;
  pp3d_rlg_chain rlgchPt;
  configPt q;
  //int faj, lpj;
  p3d_jnt *Jfaj, *Jlpj, *J;
  int fres, lres;
  int tries_rlg, tries_ik, tries_bkb, tries_allsch, tries_onesch;
  int procOK, procOK2;
  int array_sch[50]; // WARNING : max. loop size limited at 50 residues !
  int ordered_array_sch[50];
  int size_arr = 50;
  int index_arr;
  int nbkbsol = 0;
  int i, j;
  int num_sch;
  Joint_tablespt *jnt_table;
  double **pass_configs;
  int *pass_index;

  // PARAMETERS ******
  int NBKBIK = 1000;
  int NBKBRLG = 1000;
  int NBKBCOLL = 1000;
  int NALLSH = 1000;
  int NONESH = 1000;
  // *****************

  q = p3d_alloc_config(robotPt);
  // excepted the loop, all the joints have constant value
  p3d_copy_config_into(robotPt, p3d_get_robot_config(robotPt), &q);

  // first and last backbone joints
  ct = robotPt->cntrt_manager->cntrts[0];  //  WARNING : suppose ONE ONLY loop
  rlgchPt = ct->rlgPt->rlgchPt;
  Jfaj = rlgchPt->rlg_data[0]->jnt;
  Jlpj = ct->pasjnts[7];  //  WARNING : valid for p3d_set_6R_bio_ik_nopep

  // first and last residues
  fres = get_AAnumber_from_jnt(Jfaj);
  lres = get_AAnumber_from_jnt(Jlpj);

  // init array_sch at 0
  for(i=0; i < size_arr; i++) {
    array_sch[i] = 0;
  }

  // ordered array of indices of resudues with side-chain  
  jnt_table = give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,fres));
  i = 0;
  num_sch = 0;
  while((fres + i) <= lres) {
    if(jnt_table[fres + i]->n_sc_joints > 0) {
      ordered_array_sch[num_sch] = jnt_table[fres + i]->namino;  // must be ==  fres + i
      num_sch ++;
    }
    i ++;
  }      

  // deactivate collisions of (mobile) side-chains
  supress_sc_rigid_sequence(robotPt->num, fres, lres);

  // alloc for passive joint values
  pass_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    pass_configs[i] = (double *)malloc(sizeof(double) * 6);
  }  
  pass_index = (int *)malloc(sizeof(int) * 6);
  
  // generate backbone
  procOK = 0;
  tries_bkb =0;
  while(!procOK && (tries_bkb < NBKBCOLL)) {    
    tries_bkb ++;
    tries_ik = 0;  
    while(!procOK && (tries_ik < NBKBRLG)) {    
      tries_ik ++;
      tries_rlg = 0;
      while(!procOK && (tries_rlg < NBKBIK)) {    
	tries_rlg ++;
	//printf("{RLG ");
	//if(p3d_random_loop_generator(robotPt,q)) {
	if(bio_generate_active_part_of_loop_OLD(ct, q)) {
	  procOK = 1;
	}
	//printf(" RLG} procOK = %d\n",procOK);
	graphPt->nb_q += 1; 
      }
      nbkbsol = 0;
      if(procOK) {
	// compute all the possible bkb conformations
	// ( note that peptide bonds are set to Pi ) 
	//printf("{IK ");
	nbkbsol = bio_compute_all_bkb_sol_nopep_new(robotPt, ct, pass_configs, pass_index);
	//printf(" IK}\n");
	if(nbkbsol > 0) {
	  printf("closed bkb conf\n");
	  procOK = 1;
	}
	else 
	  procOK = 0;
      }
    }
    graphPt->nb_q_closed += nbkbsol;

    // WARNING : KEEPS ONLY FIRST FREE BKB CONFORMATION
    //           (should try every conformation)
    i = 0;
    procOK = 0;   
    while(!procOK && (i < nbkbsol)) {
      // set passive conf
      for(j = 0; j < 6; j++) {
	J = robotPt->joints[pass_index[j]];
	q[J->index_dof] = pass_configs[i][j];
	p3d_jnt_set_dof(J, 0, q[J->index_dof]);
      }
      // update
      if(ENV.getBool(Env::drawGraph)) {
	// WARNING : a simpler function could be used : update BBox is necessary ? 
	p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob);       
	//p3d_update_this_robot_pos(J->rob);    // for draw
      }
      else {
      // *********************
      // FOR DEBUG
      //p3d_update_this_robot_pos(J->rob);    // for draw
      //g3d_draw_allwin_active();      
      //printf("\n--------------------------------------------\n");
      // *********************
      }
      i++;
      // check collision : backbone - protein
      // WARNING : provisional check all
      if(bio_all_molecules_col() > 0)
	procOK = 0;
      else {
	procOK = 1;
	printf("free closed bkb conf\n");
      }     
    }
  }

  for(i = 0; i < 16; i++) {
    free(pass_configs[i]);
  }
  free(pass_configs);
  free(pass_index);  

  if(procOK) {
    graphPt->nb_bkb_q_free += 1;
    //printf("\n\n BACKBONE OK !!!!!!!!!!!!!!!!!\n\n");
  }
  else
    return NULL;
  
  // generate side-chains
  tries_allsch = 0;
  procOK = 0;   
  while(!procOK && (tries_allsch < NALLSH)) {    
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
      while(!procOK2 && (tries_onesch < NONESH)) {    
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
    return(p3d_APInode_make(graphPt,q));
  }
  else
    return NULL;
}

/* ------------------------------------------------------------------------------ */

p3d_node* bio_shoot_loop(p3d_graph *graphPt)
{
  p3d_rob *robotPt = graphPt->rob;
  p3d_cntrt *ct,*ect;
  pp3d_rlg_chain rlgchPt;
  configPt q;
  //int faj, lpj;
  p3d_jnt *Jfaj, *Jlpj, *J;
  int fres, lres;
  int tries_rlg, tries_ik, tries_bkb, tries_allsch, tries_onesch;
  int procOK, procOK2;
  int array_sch[50]; // WARNING : max. loop size limited at 50 residues !
  int ordered_array_sch[50];
  int size_arr = 50;
  int index_arr;
  int nbkbsol = 0, nbkbfree = 0;
  int i, j;
  int num_sch;
  Joint_tablespt *jnt_table;
  double **pass_configs;
  int *pass_index;
  p3d_node *nodePt;
  int iench;
  int indikfreesol[16];
  double mindiff,sqrdiff;
  int indmin=0;

  // PARAMETERS ******
  int NBKBIK = 1000;
  int NBKBRLG = 1000;
  int NBKBCOLL = 1000;
  int NALLSH = 500;
  int NONESH = 50;
  // *****************

  q = p3d_alloc_config(robotPt);
  // excepted the loop, all the joints have constant value
  p3d_copy_config_into(robotPt, p3d_get_robot_config(robotPt), &q);
  
  //p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);  // necessary ???

  // first and last backbone joints
  ct = robotPt->cntrt_manager->cntrts[0];  //  WARNING : suppose ONE ONLY loop
  rlgchPt = ct->rlgPt->rlgchPt;
  Jfaj = rlgchPt->rlg_data[0]->jnt;
  Jlpj = ct->pasjnts[7];  //  WARNING : valid for p3d_set_6R_bio_ik_nopepO(_new)

  // first and last residues
  fres = get_AAnumber_from_jnt(Jfaj);
  lres = get_AAnumber_from_jnt(Jlpj);

  // init array_sch at 0
  for(i=0; i < size_arr; i++) {
    array_sch[i] = 0;
  }

  // ordered array of indices of resudues with side-chain  
  jnt_table = give_joint_tables(num_subrobot_AA_from_AAnumber(robotPt,fres));
  i = 0;
  num_sch = 0;
  while((fres + i) <= lres) {
    if(jnt_table[fres + i]->n_sc_joints > 0) {
      ordered_array_sch[num_sch] = jnt_table[fres + i]->namino;  // must be ==  fres + i
      num_sch ++;
    }
    i ++;
  }      

  // deactivate collisions of (mobile) side-chains
  supress_sc_rigid_sequence(robotPt->num, fres, lres);

  // alloc for passive joint values
  pass_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    pass_configs[i] = (double *)malloc(sizeof(double) * 6);
  }  
  pass_index = (int *)malloc(sizeof(int) * 6);
  
  // generate backbone
  procOK = 0;
  tries_bkb =0;
  while(!procOK && (tries_bkb < NBKBCOLL)) {    
    tries_bkb ++;
    tries_ik = 0;  
    while(!procOK && (tries_ik < NBKBRLG)) {    
      tries_ik ++;
      tries_rlg = 0;
      while(!procOK && (tries_rlg < NBKBIK)) {    
	tries_rlg ++;
	// deactivate collisions of backbone
	supress_bb_rigid_sequence(robotPt->num, fres, lres);
	// RLG with collision detection (backbone elements are activated inside)
	// NOTE : Hbond constraints of active subchain are checked inside this function
	if(bio_generate_FREE_active_part_of_loop(ct, q, NULL)) {
	//if(bio_generate_active_part_of_loop_OLD(ct, q)) {
	  //p3d_set_robot_config(robotPt,q);
	  //p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt); 
	  procOK = 1;
	}
	//printf(" RLG} procOK = %d\n",procOK);
	graphPt->nb_q += 1; 
      }
      nbkbsol = 0;
      if(procOK) {
	// compute all the possible bkb conformations
	// ( note that peptide bonds are set to Pi ) 
	//printf("{IK ");
	nbkbsol = bio_compute_all_bkb_sol_nopep_new(robotPt, ct, pass_configs, pass_index);
	//printf(" IK}\n");
	if(nbkbsol > 0)
	  procOK = 1;
	else 
	  procOK = 0;
      }
      //if(nbkbsol > 0)
      //printf("nbkbsol = %d at tries_rlg = %d\n",nbkbsol,tries_rlg);
    }
    graphPt->nb_q_closed += nbkbsol;

    //PrintInfo(("Sampled_bkb_confs = %5d     Closed_bkb_confs = %5d    \r",graphPt->nb_q,graphPt->nb_q_closed));
    //printf("Sampled_bkb_confs = %5d     Closed_bkb_confs = %5d    \r",graphPt->nb_q,graphPt->nb_q_closed);

    if(procOK) {
      // activate passive subchain of backbone
      activate_bb_rigid_sequence(robotPt->num, fres, lres);
      
      // WARNING : KEEPS ONLY FIRST FREE BKB CONFORMATION
      //           (should try every conformation)
      i = 0;
      procOK = 0;  
      nbkbfree = 0;
       //while(!procOK && (i < nbkbsol)) {
      while(i < nbkbsol) {
	// set passive conf
	for(j = 0; j < 6; j++) {
	  J = robotPt->joints[pass_index[j]];
	  q[J->index_dof] = pass_configs[i][j];
	  //printf("q[%d] = %f , ",J->index_dof,q[J->index_dof]);
	  p3d_jnt_set_dof(J, 0, q[J->index_dof]);
	}
	//printf("\n");
	// update
	if(ENV.getBool(Env::drawGraph)) {
	  // WARNING : a simpler function could be used : update BBox is necessary ? 
	  //p3d_update_this_robot_pos_without_cntrt_and_obj(J->rob);       
	  p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
	}
	else {
	  // *********************
	  // FOR DEBUG
	  p3d_update_this_robot_pos_without_cntrt(J->rob);    // for draw
	  g3d_draw_allwin_active();      
	  //printf("\n--------------------------------------------\n");
	  // *********************
	}

	// check Hbond constraints of passive subchain and keep only the good ones
	procOK = 1;
	for(iench=0; iench<ct->nenchained; iench++) {
	  ect = ct->enchained[iench];
	  if(ct->pasjnts[0]->num <= ect->argu_i[1]) {
	    if(!(*ect->fct_cntrt)(ect,-1,NULL,0.0)) {
	      if(ENV.getBool(Env::drawGraph)) printf("Hbond cntrt num %d not satisfied at J%d\n",ect->num,J->num);
	      procOK = 0;
	      break;
	    }
	    else {
	      procOK = 1;     
	      if(ENV.getBool(Env::drawGraph)) printf("Hbond cntrt num %d OK\n",ect->num);
	    }
	  }
	}
	// check collision : backbone - protein
	// WARNING : provisional check all
	// SHOULD CHECK ONLY FOR BKB ELEMENTS IN PASSIVE SUBCHAIN
	if(procOK) {
	  if(bio_all_molecules_col() > 0) {
	    procOK = 0;
	    ////// 4 DEBUG 	
	    //if(G3D_DRAW_GRAPH) afficher_lescollisions();		
	    /////////
	  }
	  else {
	    procOK = 1;
	    indikfreesol[nbkbfree] = i;
	    nbkbfree++;	    
	  }	    
	  printf("BKB free = %d\n",nbkbfree);
	}
	i++;
      }
      
      if(nbkbfree > 0) {
	procOK = 1;
      }
      if(nbkbfree > 1) {
	// keep closest config to robotPt->ROBOT_POS
	mindiff = P3D_HUGE;
	for(i=0; i<nbkbfree; i++) {
	  sqrdiff = 0.0;
	  for(j = 0; j < 6; j++) {
	    J = robotPt->joints[pass_index[j]];
	    sqrdiff += SQR(robotPt->ROBOT_POS[J->index_dof] - pass_configs[indikfreesol[i]][j]);
	  }
	  if(sqrdiff < mindiff) {
	    mindiff = sqrdiff;
	    indmin = indikfreesol[i];
	    //printf("indmin = %d , i = %d\n",indmin,i);
	  }
	}
	for(j = 0; j < 6; j++) {
	  J = robotPt->joints[pass_index[j]];
	  q[J->index_dof] = pass_configs[indmin][j];
	  //printf("q[%d] = %f , ",J->index_dof,q[J->index_dof]);
	  p3d_jnt_set_dof(J, 0, q[J->index_dof]);
	}
	//printf("\n");
	p3d_update_this_robot_pos_without_cntrt(J->rob);
      }
    }
  }

  for(i = 0; i < 16; i++) {
    free(pass_configs[i]);
  }
  free(pass_configs);
  free(pass_index);  

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
  while(!procOK && (tries_allsch < NALLSH)) {    
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
      while(!procOK2 && (tries_onesch < NONESH)) {    
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


/* ------------------------------------------------------------------------------ */

/* int bio_readresind(pp3d_jnt jntPt) */
/* { */
/*  char *s=jntPt->name; */
/*  int indexletter=0; */
/*  int namino; */

/*  givemeword(s, '.', &indexletter); */
/*  givemeword(s, '.', &indexletter); */
/*  sscanf(givemeword(s, '\0', &indexletter), "%d", &namino); */
/*  return namino; */
/* } */

/* ------------------------------------------------------------------------------ */


static int bio_compute_all_bkb_sol_nopep_new(p3d_rob *robotPt, p3d_cntrt *ct, double **pass_configs, int *pass_index)
{
  double **sol_configs;
  int nsol, i;
  int ns, nvalidsol;
  int fail, inci;
  double vmin, vmax;

  //p3d_update_this_robot_pos_without_cntrt_and_obj(robotPt);

  sol_configs = (double **)malloc(sizeof(double *) * 16);
  for(i = 0; i < 16; i++) {
    sol_configs[i] = (double *)malloc(sizeof(double) * 6);
  }

  /* compute all ik solutions */
  nsol = bio_compute_ik_nopep_new(ct,sol_configs);
  //nsol = bio_compute_ik_nopep(ct,sol_configs);

  ns = 0;
  nvalidsol = 0;
  while(ns < nsol) {
    fail = 0;
    inci = 0;
    for(i=0; (i<8) && (!fail); i++) {
      if((i == 2)||(i == 5)) {
	inci++;
      }
      else {
	p3d_jnt_get_dof_bounds(ct->pasjnts[i],0, &vmin, &vmax);
	if((sol_configs[ns][i-inci] <= vmax)&&(sol_configs[ns][i-inci] >= vmin)) {
	  pass_configs[nvalidsol][i-inci] = sol_configs[ns][i-inci];
	  pass_index[i-inci] = ct->pasjnts[i]->num;   // <- necessary only once (but nevermind)
	  if(i == 7)
	    nvalidsol++;
	}
	else {
	  fail = 1;
	} 
      }
    }  
    ns++;
  }

  for(i = 0; i < 16; i++) {
    free(sol_configs[i]);
  }
  free(sol_configs);

  return nvalidsol;  
}



////////////////////////////////////////
// STATISTICS

void bio_loop_graph_statistics(void)
{
  p3d_graph *G;
  p3d_compco *CompPt;
  p3d_list_node *ListNode;
  p3d_jnt *jntPt;
  p3d_rob *robPt;
  p3d_vector3 posJ,posJ_ref,pos_diff;
  double dist,max_dist,add_dist,avr_dist,add_diff,std_dev;
  int nnodes;
  p3d_node *Nfar=NULL;
  //FILE *fp;

  // THIS IS A VERY PARTICULAR FUNCTION USED FOR PROTEIN LOOPS
  // IT COMPUTES STATISTICS ON THE POSITIONS OF THE CENTRAL ATOM
  // FOR NODES IN AN RRT

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  if((robPt->cntrt_manager != NULL)&&(robPt->cntrt_manager->cntrts != NULL)) { // LOOP
    // the joint must be set using the button "Draw graph"
    if(p3d_get_user_drawnjnt() == -1) {
      printf("No jnt has been defined for computing statistics\n");
      return;
    }
    
    jntPt = robPt->joints[p3d_get_user_drawnjnt()];
  }
  else {  // NO LOOP (WARNING EVEN IF THERE IS A LIGAND !!!!) 
    // WARNING : the joint we choice to mesure distance is :
    //           - the last jnt in list next_jnt of jnt0
    //           - a freeflying
    jntPt = robPt->joints[0]->next_jnt[XYZ_ROBOT->joints[0]->n_next_jnt - 1];
  }

  //fp= fopen("centralatompos.pdb","w");
  //fprintf(fp,"# explored positions of atom : %s\n\n",jntPt->o->pol[0]->poly->name);
  
  p3d_set_and_update_this_robot_conf_without_cntrt(robPt,robPt->ROBOT_POS);
  p3d_jnt_get_cur_vect_point(jntPt,posJ_ref);

  G = XYZ_GRAPH;
  CompPt = G->comp;    // ONLY FOR THE FIRST COMPONENT

  // compute max and average
  ListNode = CompPt->dist_nodes;
  nnodes = 0;
  max_dist = 0.0;
  add_dist = 0.0;
  while(ListNode != NULL) {
    nnodes++;
    p3d_set_and_update_this_robot_conf_without_cntrt(robPt,ListNode->N->q);
    p3d_jnt_get_cur_vect_point(jntPt,posJ);
    // print PDB atom line
/*     fprintf(fp,"%-6s","HETATM"); */
/*     fprintf(fp,"%5d", 0);     */
/*     fprintf(fp,"%2s","  "); */
/*     fprintf(fp,"%-3s","CA"); */
/*     fprintf(fp,"%4s", "MID"); */
/*     fprintf(fp,"%1s"," "); */
/*     fprintf(fp,"%1s", "X"); */
/*     fprintf(fp,"%4d", 0);     */
/*     fprintf(fp,"%12.3f", posJ[0]); */
/*     fprintf(fp,"%8.3f", posJ[1]); */
/*     fprintf(fp,"%8.3f", posJ[2]); */
/*     fprintf(fp,"%26s",""); */
/*     fprintf(fp,"\n");    */
    //////////
    p3d_vectSub(posJ_ref,posJ,pos_diff);
    dist = (double) p3d_vectNorm(pos_diff);
    add_dist += dist;
    if(dist > max_dist) {
      max_dist = dist;
      Nfar = ListNode->N;
    }
    ListNode = ListNode->next;
  }
  avr_dist = add_dist/((double) nnodes);
  // compute variance
  ListNode = CompPt->dist_nodes;
  add_diff = 0.0;
  while(ListNode != NULL) {
    p3d_set_and_update_this_robot_conf_without_cntrt(robPt,ListNode->N->q);
    p3d_jnt_get_cur_vect_point(jntPt,posJ);
    p3d_vectSub(posJ_ref,posJ,pos_diff);
    dist = (double) p3d_vectNorm(pos_diff);
    add_diff += SQR(dist - avr_dist);
    ListNode = ListNode->next;   
  }
  std_dev = sqrt((1.0/(((double)nnodes) - 1.0)) * add_diff);

  //fclose(fp);	

  printf("//// STATISTICS ////  (position variation of J%d frame)\n",p3d_get_user_drawnjnt());
  printf(" average            = %f\n",avr_dist);
  printf(" maximum            = %f\n",max_dist);
  printf(" standard deviation = %f\n",std_dev);

  printf("Farther conformation\n"); 
  p3d_print_node(robPt->GRAPH, Nfar);

  // set farther confromation as GOAL
  p3d_copy_config_into(robPt,Nfar->q,&(robPt->ROBOT_GOTO));
  printf("ROBOT_GOTO updated to farther conf. in graph\n");
}

////////////////////////////////////////

