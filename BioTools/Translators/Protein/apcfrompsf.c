// INCLUDES

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "defs.h"
#include "atoms.h"
#include "protein.h"
#include "mathfcts.h"
#include "atomhandling.h"
#include "apc.h"


/**********************************************************************/
// ALLOC-FREE FUNCTIONS

static void alloc_and_init_apc_rigid_struct(apc_rigid **apc_rigidPtPt)
{
  apc_rigid *apc_rigidPt;

  apc_rigidPt = (apc_rigid *) malloc(sizeof(apc_rigid));

  apc_rigidPt->natoms = 0;
  apc_rigidPt->atomlist = NULL;
  apc_rigidPt->nintervals = 0;

  *apc_rigidPtPt = apc_rigidPt;
}


static void alloc_and_init_apc_rlelem_struct(apc_rigidlistelem **apc_rlelemPtPt,
					     apc_rigid **apc_rigidPtPt,
					     apc_rigid *apc_prevrigidPt)
{
  apc_rigid *apc_rigidPt;
  apc_rigidlistelem *apc_rlelemPt;

  apc_rlelemPt = (apc_rigidlistelem *) malloc(sizeof(apc_rigidlistelem));
  alloc_and_init_apc_rigid_struct(apc_rigidPtPt);
  apc_rigidPt = *apc_rigidPtPt;

  apc_rlelemPt->rigid = apc_rigidPt;
  apc_rigidPt->inlist = apc_rlelemPt;
  apc_rlelemPt->next = NULL;
  apc_rlelemPt->nnext = 0;
  apc_rlelemPt->prev = apc_prevrigidPt;  

  *apc_rlelemPtPt = apc_rlelemPt;
}


static void alloc_and_init_apc_sidechain_struct(apc_sidechain **apc_schPtPt)
{
  apc_sidechain *apc_schPt;

  apc_schPt = (apc_sidechain *) malloc(sizeof(apc_sidechain));

  apc_schPt->nrigids = 0;
  apc_schPt->firstrigid = NULL;
  apc_schPt->refbkbrigid = NULL;

  *apc_schPtPt = apc_schPt;
}


static void alloc_and_init_apc_backbone_struct(apc_backbone **apc_bkbPtPt)
{
  apc_backbone *apc_bkbPt;

  apc_bkbPt = (apc_backbone *) malloc(sizeof(apc_backbone));

  apc_bkbPt->nrigids = 0;
  apc_bkbPt->firstrigid = NULL;
  apc_bkbPt->lastrigid = NULL;

  *apc_bkbPtPt = apc_bkbPt;
}


static void alloc_and_init_apc_struct(apc **apcPtPt)
{
  apc *apcPt;

  apcPt = (apc *) malloc(sizeof(apc));

  apcPt->backbone = NULL;
  apcPt->nsidechains = 0;
  apcPt->sidechains = NULL;

  *apcPtPt = apcPt;
}


void free_apc(apc *apcPt)
{

  // TO DO

}


/**********************************************************************/
// LIST MANAGEMENT

static void insert_pointer_in_list(void *thePt, void ***listPt, int *nelems)
{
  if(*nelems == 0) {
    *listPt = (void **) malloc(sizeof(void *));
  }
  else {
    *listPt = (void **) realloc(*listPt,sizeof(void *) * (*nelems + 1));
  }
  (*listPt)[*nelems] = thePt;
  (*nelems)++;
}

/**********************************************************************/

static void insert_rigid_in_apc_bkb_lists(apc *apcPt, apc_rigidlistelem *apc_rlelemPt)
{
  if(apcPt->backbone->firstrigid != NULL) {
    apc_rlelemPt->prev = apcPt->backbone->lastrigid->rigid;
  }
  else {
    apcPt->backbone->firstrigid = apc_rlelemPt;
    apc_rlelemPt->prev = NULL;
  }
  if(apcPt->backbone->lastrigid != NULL) {
    insert_pointer_in_list(apc_rlelemPt->rigid,(void ***)&(apcPt->backbone->lastrigid->next),
			   &(apcPt->backbone->lastrigid->nnext));
  }
  apcPt->backbone->lastrigid = apc_rlelemPt;
  apcPt->backbone->nrigids++;
}

/**********************************************************************/

/* static void insert_apc_atom_in_list(apc_atom *apc_atomPt, apc_atom ***apc_atomlist, int *natoms) */
/* { */
/*   if(*natoms == 0) { */
/*     *apc_atomlist = (apc_atom **) malloc(sizeof(apc_atom *)); */
/*   } */
/*   else { */
/*     *apc_atomlist = (apc_atom **) realloc(*apc_atomlist,sizeof(apc_atom *) * (*natoms + 1)); */
/*   } */
/*   (*apc_atomlist)[*natoms] = apc_atomPt; */
/*   (*natoms)++; */
/* } */

/**********************************************************************/

static void insert_atom_in_apc_rigid(apc_rigid *apc_rigidPt, atom *atomPt)
{
  apc_atom *apc_atomPt;
  matrix4 invT;

  apc_atomPt = (apc_atom *) malloc(sizeof(apc_atom));
  apc_atomPt->atomPt = atomPt;

  // set flag inapc
  atomPt->inapc = 1;

  // compute ralative position in rigid frame
  inverse_transf(apc_rigidPt->Tabs,invT);
  mat4vec3MultPos(invT,atomPt->pos,apc_atomPt->posrelframe);

  insert_pointer_in_list(apc_atomPt,(void ***)&(apc_rigidPt->atomlist),&(apc_rigidPt->natoms));
}


/**********************************************************************/
// DIHEDRAL ANGLE MANAGEMENT

static void set_dihedang_bounds(apc_rigid *apc_rigidPt, int ninterv, double *l_interv, double *u_interv)
{
  int i;

  // NOTE : currently, only one interval is considered !!!

  // put qi in [0,2*PI)
  if(apc_rigidPt->qi < 0.0)
    apc_rigidPt->qi += 2*PI;

  // joint variable intervals
  apc_rigidPt->nintervals = ninterv;

  if(ninterv == 1) {
    // NOTE : bounds can be modified form peptide bonds
    if(apc_rigidPt->qi < l_interv[0])
      apc_rigidPt->q_interv[0].vmin = apc_rigidPt->qi;
    else
      apc_rigidPt->q_interv[0].vmin = l_interv[0];      
    if(apc_rigidPt->qi > u_interv[0])
      apc_rigidPt->q_interv[0].vmax = apc_rigidPt->qi;
    else
      apc_rigidPt->q_interv[0].vmax = u_interv[0];
  }
  else {
    for(i=0; i<ninterv; i++) {
      apc_rigidPt->q_interv[0].vmin = l_interv[0];      
      apc_rigidPt->q_interv[0].vmax = u_interv[0];
    }
  }
 }


/**********************************************************************/
// FRAMES MANAGEMENT

static int compute_apc_bkb_base_frame(apc *apcPt, residue *resPt)
{
  double *a1pos,*a2pos,*a3pos;
  double posdiff[3],zaxis[3],nextzaxis[3],xaxis[3];

  // check that a reference frame can be computed 
  if(resPt->prev == NULL) {
    printf("ERROR : cannot generate apc : no previous residue in chain to residue %d\n",
	   resPt->resSeq);
    return -1;
  }
  
  get_C_pos(resPt->prev,&a1pos);
  get_N_pos(resPt,&a2pos);
  get_CA_pos(resPt,&a3pos);

  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,zaxis);
  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,nextzaxis);  
  normalized_vectXprod(zaxis,nextzaxis,xaxis);

  compute_frame(a2pos,xaxis,zaxis,apcPt->backbone->Tbase);

  return 1;
}


static int compute_apc_bkb_end_frame(apc *apcPt, residue *resPt)
{
  double *a1pos,*a2pos,*a3pos;
  double posdiff[3],zaxis[3],nextzaxis[3],xaxis[3];

  // check that a reference frame can be computed 
  if(resPt->next == NULL) {
    printf("ERROR : cannot generate apc : no next residue in chain to residue %d\n",
	   resPt->resSeq);
    return -1;
  }
  
  get_C_pos(resPt,&a1pos);
  get_N_pos(resPt->next,&a2pos);
  get_CA_pos(resPt->next,&a3pos);

  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,zaxis);
  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,nextzaxis);  
  normalized_vectXprod(zaxis,nextzaxis,xaxis);

  compute_frame(a2pos,xaxis,zaxis,apcPt->backbone->Tend);

  return 1;
}


static void compute_apc_bkb_rigid_transformations(apc *apcPt, apc_rigid *apc_rigidPt)
{
  matrix4 invT,Tabs0,Tref;

  // VERIFICAR !!! 
  // PONER EN FUNCION A PARTE
  transf_rotz(apc_rigidPt->qi,apc_rigidPt->Ti);
  inverse_transf(apc_rigidPt->Ti,invT);
  mat4Mult(apc_rigidPt->Tabs,invT,Tabs0);
  if(apc_rigidPt->inlist->prev == NULL) {
    mat4Copy(apcPt->backbone->Tbase,Tref);
  }
  else {
    mat4Copy(apc_rigidPt->inlist->prev->Tabs,Tref);
  }
  inverse_transf(Tref,invT);
  mat4Mult(invT,Tabs0,apc_rigidPt->pT0);
  inverse_transf(apc_rigidPt->pT0,apc_rigidPt->nT0);
}


static void compute_apc_sch_rigid_transformations(apc_rigid *apc_rigidPt, apc_rigid *apc_refrigidPt)
{
  matrix4 invT,Tabs0,Tref;

  // VERIFICAR !!! 
  // PONER EN FUNCION A PARTE
  transf_rotz(apc_rigidPt->qi,apc_rigidPt->Ti);
  inverse_transf(apc_rigidPt->Ti,invT);
  mat4Mult(apc_rigidPt->Tabs,invT,Tabs0);
  mat4Copy(apc_refrigidPt->Tabs,Tref);
  inverse_transf(Tref,invT);
  mat4Mult(invT,Tabs0,apc_rigidPt->pT0);
  inverse_transf(apc_rigidPt->pT0,apc_rigidPt->nT0);
}


/**********************************************************************/
// BACKBONE FUNCTIONS

static int generate_first_bkb_rigid(apc *apcPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigidPt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // NOTE :
  // the position of the atom N of the first residue in the apc is unchanged,
  // thus it doen not need to be in the apc struct (atom->inapc = 0)
  if(apcPt->backbone->nrigids == 0) { // first residue in apc
    return 1;
  }

  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigidPt,NULL);
  insert_rigid_in_apc_bkb_lists(apcPt,apc_rlelemPt);

  // compute dihedral angle and abs. frame
  get_CA_pos(resPt->prev,&a1pos);
  get_C_pos(resPt->prev,&a2pos);
  get_N_pos(resPt,&a3pos);
  get_CA_pos(resPt,&a4pos);

  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigidPt->qi),apc_rigidPt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = OMEGAMIN;
  u_interv[1] = OMEGAMAX;
  set_dihedang_bounds(apc_rigidPt,1,l_interv,u_interv);

  // local transformations
  compute_apc_bkb_rigid_transformations(apcPt,apc_rigidPt);
  
  // include atoms
  switch(resPt->resType) {
  case GLY: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLY_N]);
    break;
  case GLYH: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLYH_N]);
    break;
  // NOTE : for PROLINE : although phi is currently considered
  //        as fixed, the joint is created for an easier management 
  //        of rigids and for possible future developments
  case PRO: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_N]);
    break;
  case PROH: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_N]);
    break;
  default:
    if(resPt->flagH == 0) {
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[gen_N]);
    }
    else {
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[genH_N]);
    }
  }

  return 1;
}


static int generate_second_bkb_rigid_and_init_sidechain(apc *apcPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigidPt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  apc_sidechain *apc_schPt;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigidPt,NULL);
  insert_rigid_in_apc_bkb_lists(apcPt,apc_rlelemPt);

  // compute dihedral angle and abs. frame
  get_C_pos(resPt->prev,&a1pos);
  get_N_pos(resPt,&a2pos);
  get_CA_pos(resPt,&a3pos);
  get_C_pos(resPt,&a4pos);

  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigidPt->qi),apc_rigidPt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigidPt,1,l_interv,u_interv);

  // local transformations
  compute_apc_bkb_rigid_transformations(apcPt,apc_rigidPt);
  
  // include atoms
  switch(resPt->resType) {
  case GLY: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLY_CA]);
    break;
  case GLYH: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLYH_CA]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLYH_1HA]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLYH_2HA]);
    break;
  // NOTE : for PROLINE : the ring is considered rigid
  case PRO: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_CA]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_CB]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_CG]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_CD]);
    break;
  case PROH: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_CA]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_HA]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_CB]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_1HB]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_2HB]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_CG]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_1HG]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_2HG]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_CD]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_1HD]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_2HD]);
    break;
  default:
    if(resPt->flagH == 0) {
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[gen_CA]);
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[gen_CB]);
    }
    else {
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[genH_CA]);
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[genH_HA]);
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[genH_CB]);
    }
    // alloc and init sidechain
    alloc_and_init_apc_sidechain_struct(&apc_schPt);
    apc_schPt->refbkbrigid = apc_rigidPt;
    insert_pointer_in_list(apc_schPt,(void ***)&(apcPt->sidechains),&(apcPt->nsidechains));
  }

  return 1;
}


static int generate_third_bkb_rigid(apc *apcPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigidPt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigidPt,NULL);
  insert_rigid_in_apc_bkb_lists(apcPt,apc_rlelemPt);

  // compute dihedral angle and abs. frame
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  get_C_pos(resPt,&a3pos);
  get_N_pos(resPt->next,&a4pos);

  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigidPt->qi),apc_rigidPt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigidPt,1,l_interv,u_interv);

  // local transformations
  compute_apc_bkb_rigid_transformations(apcPt,apc_rigidPt);
  
  // include atoms
  switch(resPt->resType) {
  case GLY: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLY_C]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLY_O]);
    break;
  case GLYH: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLYH_C]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[GLYH_O]);
    break;
  case PRO: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_C]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PRO_O]);
    break;
  case PROH: 
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_C]);
    insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[PROH_O]);
    break;
  default:
    if(resPt->flagH == 0) {
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[gen_C]);
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[gen_O]);
    }
    else {
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[genH_C]);
      insert_atom_in_apc_rigid(apc_rigidPt,resPt->bkbAlist[genH_O]);
    }
  }

  return 1;
}


static int generate_residue_backbone_rigids(apc *apcPt, residue *resPt)
{
  if((generate_first_bkb_rigid(apcPt,resPt) <0) ||
     (generate_second_bkb_rigid_and_init_sidechain(apcPt,resPt) <0) ||
     (generate_third_bkb_rigid(apcPt,resPt) <0)) {
    return -1;
  }
   
  return 1;
}

/**********************************************************************/

static int generate_apc_backbone_and_init_sidechains(apc *apcPt, residue *fresPt, residue *lresPt)
{
  residue *resPt;

  alloc_and_init_apc_backbone_struct(&(apcPt->backbone));

  // compute base and end frames
  if((compute_apc_bkb_base_frame(apcPt,fresPt) < 0) ||
     (compute_apc_bkb_end_frame(apcPt,lresPt) < 0)) {
    return -1;
  }

  // generate rigids
  resPt = fresPt;
  while(resPt != lresPt->next) {
    if(generate_residue_backbone_rigids(apcPt,resPt) < 0) {
      return -1;
    }
    resPt = resPt->next;
  }

  return 1;
}

/**********************************************************************/
// SIDE-CHAIN FUNCTIONS
// NOTE : functions for PDB with H are not writen yet <- TO DO

static int generate_ALAH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_ARG_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt,*apc_rigid3Pt,*apc_rigid4Pt,*apc_rigid5Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[ARG_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[ARG_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[ARG_CG]->pos;
  // NOTE : ref for dihedang : CD
  a4pos = resPt->schAlist[ARG_CD]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[ARG_CD]);

  // -- third side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid3Pt,apc_rigid2Pt);
  insert_pointer_in_list(apc_rigid3Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma3)
  a1pos = resPt->bkbAlist[gen_CB]->pos;
  a2pos = resPt->schAlist[ARG_CG]->pos;
  a3pos = resPt->schAlist[ARG_CD]->pos;
  // NOTE : ref for dihedang : NE
  a4pos = resPt->schAlist[ARG_NE]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid3Pt->qi),apc_rigid3Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid3Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid3Pt,apc_rigid2Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[ARG_NE]);

  // -- fourth side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid4Pt,apc_rigid3Pt);
  insert_pointer_in_list(apc_rigid4Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma4)
  a1pos = resPt->schAlist[ARG_CG]->pos;
  a2pos = resPt->schAlist[ARG_CD]->pos;
  a3pos = resPt->schAlist[ARG_NE]->pos;
  // NOTE : ref for dihedang : CZ
  a4pos = resPt->schAlist[ARG_CZ]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid4Pt->qi),apc_rigid4Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid4Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid4Pt,apc_rigid3Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid4Pt,resPt->schAlist[ARG_CZ]);

  // -- fifth side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid5Pt,apc_rigid4Pt);
  insert_pointer_in_list(apc_rigid5Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma4)
  a1pos = resPt->schAlist[ARG_CD]->pos;
  a2pos = resPt->schAlist[ARG_NE]->pos;
  a3pos = resPt->schAlist[ARG_CZ]->pos;
  // NOTE : ref for dihedang : NH1
  a4pos = resPt->schAlist[ARG_NH1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid5Pt->qi),apc_rigid5Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid5Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid5Pt,apc_rigid4Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid5Pt,resPt->schAlist[ARG_NH1]);
  insert_atom_in_apc_rigid(apc_rigid5Pt,resPt->schAlist[ARG_NH2]);

  return 1;
}


static int generate_ARGH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_ASN_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[ASN_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[ASN_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[ASN_CG]->pos;
  // NOTE : ref for dihedang : OD1
  a4pos = resPt->schAlist[ASN_OD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[ASN_OD1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[ASN_ND2]);

  return 1;
}


static int generate_ASNH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_ASP_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[ASP_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[ASP_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[ASP_CG]->pos;
  // NOTE : ref for dihedang : OD1
  a4pos = resPt->schAlist[ASP_OD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[ASP_OD1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[ASP_OD2]);

  return 1;
}


static int generate_ASPH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_CYS_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : SG
  a4pos = resPt->schAlist[CYS_SG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[CYS_SG]);

  return 1;
}


static int generate_CYSH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_GLN_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt,*apc_rigid3Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[GLN_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[GLN_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[GLN_CG]->pos;
  // NOTE : ref for dihedang : CD
  a4pos = resPt->schAlist[GLN_CD]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[GLN_CD]);

  // -- third side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid3Pt,apc_rigid2Pt);
  insert_pointer_in_list(apc_rigid3Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma3)
  a1pos = resPt->bkbAlist[gen_CB]->pos;
  a2pos = resPt->schAlist[GLN_CG]->pos;
  a3pos = resPt->schAlist[GLN_CD]->pos;
  // NOTE : ref for dihedang : OE1
  a4pos = resPt->schAlist[GLN_OE1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid3Pt->qi),apc_rigid3Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid3Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid3Pt,apc_rigid2Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[GLN_OE1]);
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[GLN_NE2]);

  return 1;
}


static int generate_GLNH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_GLU_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt,*apc_rigid3Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[GLU_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[GLU_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[GLU_CG]->pos;
  // NOTE : ref for dihedang : CD
  a4pos = resPt->schAlist[GLU_CD]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[GLU_CD]);

  // -- third side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid3Pt,apc_rigid2Pt);
  insert_pointer_in_list(apc_rigid3Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma3)
  a1pos = resPt->bkbAlist[gen_CB]->pos;
  a2pos = resPt->schAlist[GLU_CG]->pos;
  a3pos = resPt->schAlist[GLU_CD]->pos;
  // NOTE : ref for dihedang : OE1
  a4pos = resPt->schAlist[GLU_OE1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid3Pt->qi),apc_rigid3Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid3Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid3Pt,apc_rigid2Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[GLU_OE1]);
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[GLU_OE2]);

  return 1;
}


static int generate_GLUH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_HIS_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[HIS_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[HIS_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[HIS_CG]->pos;
  // NOTE : ref for dihedang : ND1
  a4pos = resPt->schAlist[HIS_ND1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[HIS_ND1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[HIS_CD2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[HIS_CE1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[HIS_NE2]);

  return 1;
}


static int generate_HISH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_ILE_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG1
  a4pos = resPt->schAlist[ILE_CG1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[ILE_CG2]);
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[ILE_CG1]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[ILE_CG1]->pos;
  // NOTE : ref for dihedang : CD1
  a4pos = resPt->schAlist[ILE_CD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[ILE_CD1]);

  return 1;
}


static int generate_ILEH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_LEU_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[LEU_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[LEU_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[LEU_CG]->pos;
  // NOTE : ref for dihedang : CD1
  a4pos = resPt->schAlist[LEU_CD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[LEU_CD1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[LEU_CD2]);

  return 1;
}


static int generate_LEUH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_LYS_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt,*apc_rigid3Pt,*apc_rigid4Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[LYS_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[LYS_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[LYS_CG]->pos;
  // NOTE : ref for dihedang : CD
  a4pos = resPt->schAlist[LYS_CD]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[LYS_CD]);

  // -- third side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid3Pt,apc_rigid2Pt);
  insert_pointer_in_list(apc_rigid3Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma3)
  a1pos = resPt->bkbAlist[gen_CB]->pos;
  a2pos = resPt->schAlist[LYS_CG]->pos;
  a3pos = resPt->schAlist[LYS_CD]->pos;
  // NOTE : ref for dihedang : CE
  a4pos = resPt->schAlist[LYS_CE]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid3Pt->qi),apc_rigid3Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid3Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid3Pt,apc_rigid2Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[LYS_CE]);

  // -- fourth side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid4Pt,apc_rigid3Pt);
  insert_pointer_in_list(apc_rigid4Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma4)
  a1pos = resPt->schAlist[LYS_CG]->pos;
  a2pos = resPt->schAlist[LYS_CD]->pos;
  a3pos = resPt->schAlist[LYS_CE]->pos;
  // NOTE : ref for dihedang : NZ
  a4pos = resPt->schAlist[LYS_NZ]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid4Pt->qi),apc_rigid4Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid4Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid4Pt,apc_rigid3Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid4Pt,resPt->schAlist[LYS_NZ]);

  return 1;
}


static int generate_LYSH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_MET_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
   apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt,*apc_rigid3Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[MET_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[MET_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[MET_CG]->pos;
  // NOTE : ref for dihedang : SD
  a4pos = resPt->schAlist[MET_SD]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[MET_SD]);

  // -- third side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid3Pt,apc_rigid2Pt);
  insert_pointer_in_list(apc_rigid3Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma3)
  a1pos = resPt->bkbAlist[gen_CB]->pos;
  a2pos = resPt->schAlist[MET_CG]->pos;
  a3pos = resPt->schAlist[MET_SD]->pos;
  // NOTE : ref for dihedang : CE
  a4pos = resPt->schAlist[MET_CE]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid3Pt->qi),apc_rigid3Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid3Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid3Pt,apc_rigid2Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid3Pt,resPt->schAlist[MET_CE]);

  return 1;
}


static int generate_METH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_PHE_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[PHE_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[PHE_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[PHE_CG]->pos;
  // NOTE : ref for dihedang : CD1
  a4pos = resPt->schAlist[PHE_CD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[PHE_CD1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[PHE_CD2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[PHE_CE1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[PHE_CE2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[PHE_CZ]);

  return 1;
}


static int generate_PHEH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_SER_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : OG
  a4pos = resPt->schAlist[SER_OG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[SER_OG]);

  return 1;
}


static int generate_SERH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_THR_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : OG1
  a4pos = resPt->schAlist[THR_OG1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[THR_OG1]);
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[THR_CG2]);

  return 1;
}


static int generate_THRH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_TRP_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[TRP_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[TRP_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[TRP_CG]->pos;
  // NOTE : ref for dihedang : CD1
  a4pos = resPt->schAlist[TRP_CD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CD1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CD2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_NE1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CE2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CE3]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CZ2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CZ3]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TRP_CH2]);

  return 1;
}


static int generate_TRPH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_TYR_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt,*apc_rigid2Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG
  a4pos = resPt->schAlist[TYR_CG]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[TYR_CG]);

  // -- second side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid2Pt,apc_rigid1Pt);
  insert_pointer_in_list(apc_rigid2Pt,(void ***)&(apc_rlelemPt->prev->inlist->next),&(apc_rlelemPt->prev->inlist->nnext));
  apc_schPt->nrigids++;

  // compute dihedral angle and abs. frame (gamma2)
  get_CA_pos(resPt,&a1pos);
  a2pos = resPt->bkbAlist[gen_CB]->pos;
  a3pos = resPt->schAlist[TYR_CG]->pos;
  // NOTE : ref for dihedang : CD1
  a4pos = resPt->schAlist[TYR_CD1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid2Pt->qi),apc_rigid2Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid2Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid2Pt,apc_rigid1Pt);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TYR_CD1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TYR_CD2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TYR_CE1]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TYR_CE2]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TYR_CZ]);
  insert_atom_in_apc_rigid(apc_rigid2Pt,resPt->schAlist[TYR_OH]);

  return 1;
}


static int generate_TYRH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


static int generate_VAL_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  apc_rigidlistelem *apc_rlelemPt;
  apc_rigid *apc_rigid1Pt;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double l_interv[MAX_NINTERVALS],u_interv[MAX_NINTERVALS]; 

  // -- first side-chain rigid --

  // alloc and init structures
  alloc_and_init_apc_rlelem_struct(&apc_rlelemPt,&apc_rigid1Pt,NULL);
  apc_schPt->firstrigid = apc_rlelemPt;
  apc_schPt->nrigids++;

  // pointer in reference backbone rigid to first side-chain rigid 
  insert_pointer_in_list(apc_rigid1Pt,(void ***)&(apc_schPt->refbkbrigid->inlist->next),
			 &(apc_schPt->refbkbrigid->inlist->nnext));

  // compute dihedral angle and abs. frame (gamma1)
  get_N_pos(resPt,&a1pos);
  get_CA_pos(resPt,&a2pos);
  a3pos = resPt->bkbAlist[gen_CB]->pos;
  // NOTE : ref for dihedang : CG1
  a4pos = resPt->schAlist[VAL_CG1]->pos;
  
  compute_dihedang_and_frame(a1pos,a2pos,a3pos,a4pos,&(apc_rigid1Pt->qi),apc_rigid1Pt->Tabs);

  // set dihedral angle bounds
  l_interv[0] = 0.0;
  u_interv[1] = 2*PI;
  set_dihedang_bounds(apc_rigid1Pt,1,l_interv,u_interv);

  // local transformations
  compute_apc_sch_rigid_transformations(apc_rigid1Pt,apc_schPt->refbkbrigid);

  // include atoms
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[VAL_CG1]);
  insert_atom_in_apc_rigid(apc_rigid1Pt,resPt->schAlist[VAL_CG2]);

  return 1;
}


static int generate_VALH_sidechain(apc_sidechain *apc_schPt, residue *resPt)
{
  printf("ERROR : cannot generate apc : PDB contains H\n"); 
  return -1;
}


/**********************************************************************/

static int generate_residue_sidechain(apc_sidechain *apc_schPt)
{
  residue *resPt;
  int state;

  // identify residue
  resPt = apc_schPt->refbkbrigid->atomlist[0]->atomPt->residuePt;

  state = 1;
  // generate sidechain by residue type
  switch(resPt->resType) {
  case ALA:
    // no side-chain in this case 
    break;
  case ALAH: 
    state = generate_ALAH_sidechain(apc_schPt,resPt);
    break;
  case ARG:
    state = generate_ARG_sidechain(apc_schPt,resPt);
    break;    
  case ARGH: 
    state = generate_ARGH_sidechain(apc_schPt,resPt);
    break;    
  case ASN: 
    state = generate_ASN_sidechain(apc_schPt,resPt);
    break;
  case ASNH: 
    state = generate_ASNH_sidechain(apc_schPt,resPt);
    break;
  case ASP: 
    state = generate_ASP_sidechain(apc_schPt,resPt);
    break;
  case ASPH: 
    state = generate_ASPH_sidechain(apc_schPt,resPt);
    break;
  case CYS: 
    state = generate_CYS_sidechain(apc_schPt,resPt);
    break;
  case CYSH: 
    state = generate_CYSH_sidechain(apc_schPt,resPt);
    break;
  case GLN: 
    state = generate_GLN_sidechain(apc_schPt,resPt);
    break;
   case GLNH: 
    state = generate_GLNH_sidechain(apc_schPt,resPt);
    break;
  case GLU: 
    state = generate_GLU_sidechain(apc_schPt,resPt);
    break;
  case GLUH: 
    state = generate_GLUH_sidechain(apc_schPt,resPt);
    break;
  case GLY: 
  case GLYH: 
    // no side-chain in this case 
    break;
  case HIS: 
    state = generate_HIS_sidechain(apc_schPt,resPt);
    break;
   case HISH: 
    state = generate_HISH_sidechain(apc_schPt,resPt);
    break;
  case ILE: 
    state = generate_ILE_sidechain(apc_schPt,resPt);
    break;
  case ILEH: 
    state = generate_ILEH_sidechain(apc_schPt,resPt);
    break;
  case LEU: 
    state = generate_LEU_sidechain(apc_schPt,resPt);
    break;
  case LEUH: 
    state = generate_LEUH_sidechain(apc_schPt,resPt);
    break;
  case LYS: 
    state = generate_LYS_sidechain(apc_schPt,resPt);
    break;
  case LYSH: 
    state = generate_LYSH_sidechain(apc_schPt,resPt);
    break;
  case MET: 
    state = generate_MET_sidechain(apc_schPt,resPt);
    break;
  case METH: 
    state = generate_METH_sidechain(apc_schPt,resPt);
    break;
  case PHE: 
    state = generate_PHE_sidechain(apc_schPt,resPt);
    break;
  case PHEH: 
    state = generate_PHEH_sidechain(apc_schPt,resPt);
    break;
  case PRO: 
  case PROH: 
    // no side-chain in this case 
    break;
  case SER: 
    state = generate_SER_sidechain(apc_schPt,resPt);
    break;
  case SERH: 
    state = generate_SERH_sidechain(apc_schPt,resPt);
    break;
  case THR: 
    state = generate_THR_sidechain(apc_schPt,resPt);
    break;
  case THRH: 
    state = generate_THRH_sidechain(apc_schPt,resPt);
    break;
  case TRP: 
    state = generate_TRP_sidechain(apc_schPt,resPt);
    break;
  case TRPH: 
    state = generate_TRPH_sidechain(apc_schPt,resPt);
    break;
  case TYR: 
    state = generate_TYR_sidechain(apc_schPt,resPt);
    break;
  case TYRH: 
    state = generate_TYRH_sidechain(apc_schPt,resPt);
    break;
  case VAL: 
    state = generate_VAL_sidechain(apc_schPt,resPt);
    break;
  case VALH: 
    state = generate_VALH_sidechain(apc_schPt,resPt);
    break;
  }

  return state;
}


static int generate_apc_sidechains(apc *apcPt)
{
  int i;

  for(i=0; i<apcPt->nsidechains; i++) {
    if(generate_residue_sidechain(apcPt->sidechains[i]) < 0) {
      return -1;
    }   
  }
  return 1;
}

/**********************************************************************/
// MAIN FUNCTION

int articulate_peptide_chain_segment(apc **apcPtPt, residue *fresPt, residue *lresPt)
{
  residue *resPt;

  // verify order of input residues and change if wrong 
  if(fresPt->resSeq > lresPt->resSeq) {
    resPt = fresPt;
    fresPt = lresPt;
    lresPt = resPt;
  }
  
  // check that the segment contains at least 3 residues
  if((fresPt->resSeq + 3) > lresPt->resSeq) {
    printf("ERROR : in articulate_peptide_chain_segment (residue %d, residue %d)\n      : the segment must contain at least 3 residues",
	   fresPt->resSeq,lresPt->resSeq);
    return -1;
  }
  
  // create apc structure
  alloc_and_init_apc_struct(apcPtPt);

  // generate backbone (and init sidechains)
  if(generate_apc_backbone_and_init_sidechains(*apcPtPt,fresPt,lresPt) < 0) {
    free_apc(*apcPtPt);
    return -1;
  }

  // generate side chains
  if(generate_apc_sidechains(*apcPtPt) < 0) {
    free_apc(*apcPtPt);
    return -1;
  }

  return 1;
}

/**********************************************************************/
// TESTING FUNCTIONS

static void write_atom(FILE *fPt, atom *aPt)
{
  int int_first_char;

  fprintf(fPt,"%-6s","ATOM");
  fprintf(fPt,"%5d", aPt->serial);
  int_first_char = atoi(&aPt->name[0]);
  if((int_first_char > 0) && (int_first_char < 4))
    fprintf(fPt,"%1s"," ");
  else
    fprintf(fPt,"%2s","  ");
  fprintf(fPt,"%-3s",aPt->name);
  if((int_first_char > 0) && (int_first_char < 4) && (aPt->name[3] == '\0'))
    fprintf(fPt,"%1s"," ");
  fprintf(fPt,"%4s", aPt->residuePt->resName);
  fprintf(fPt,"%1s"," ");
  fprintf(fPt,"%1s", aPt->residuePt->chainID);
  fprintf(fPt,"%4d", aPt->residuePt->resSeq);
  fprintf(fPt,"%12.3f", aPt->pos[0]);
  fprintf(fPt,"%8.3f", aPt->pos[1]);
  fprintf(fPt,"%8.3f", aPt->pos[2]);
  fprintf(fPt,"%26s","");
  fprintf(fPt,"\n");
}


/**********************************************************************/

static void write_apc_rigid_atoms(FILE *pdboutfile, apc_rigid *rigidPt)
{
  int i;
  atom *atomPt;

  for(i=0; i<rigidPt->natoms; i++) {
    atomPt = rigidPt->atomlist[i]->atomPt;
    write_atom(pdboutfile,atomPt);
  }

  // iterate : for each residue, write side-chain before Ca
  for(i=(rigidPt->inlist->nnext - 1); i>=0; i--) {
    write_apc_rigid_atoms(pdboutfile,rigidPt->inlist->next[i]);
  }
}


void write_pdb_from_apc(FILE *pdboutfile, apc *apcPt)
{
  apc_rigid *rigidPt;

  rigidPt = apcPt->backbone->firstrigid->rigid;
  write_apc_rigid_atoms(pdboutfile,rigidPt);
}
