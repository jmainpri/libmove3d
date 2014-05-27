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
/**********************************************************************/
// INCLUDES

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "defs.h"
#include "atoms.h"
#include "protein.h"
#include "atomhandling.h"
#include "mathfcts.h"
#include "list.h"
#include "pdbFormat.h"

#define TRUE 1
#define FALSE 0

#define COLLISION_DETECTION_CHAR 'V'
#define NO_COLLISION_DETECTION_CHAR 'X'

#define RIGID 0
#define LFLEX 1
#define MFLEX 2
#define FFLEX 3

extern int CONVERT_AMBER_FLAG;

/**********************************************************************/
// EXTERN FUNCTIONS

extern int fill_protein_struct(FILE *pdbfile, char *pdbfilename, protein *protPt);
extern void free_protein(protein *protPt);

extern int protein_get_res(protein* proteinPt, int subchInd, int resSeq, residue** resPtPt);
extern void res_get_nextRes(residue* resPt, residue** nextResPtPt);
extern int util_protein_is_segment(protein* protPt, int subchInd, int start_resSeq, int end_resSeq);
extern int resName_to_resType(char *resName, residueTypes *resTypePt);

/**********************************************************************/

// SUBCH SPEC DATA IN AAA (temporary)
typedef struct s_subCh_aaa_data {  
  int start_resSeq;       
  int end_resSeq;         
  int_list bkb_flex_level;  
  int_list sch_flex_level;  
} subCh_aaa_data;

typedef struct s_subCh_aaa_list { 
  int nsubCh_spec;                  
  subCh_aaa_data** subCh_spec_list; 
} subCh_aaa_list;


/**********************************************************************/
// GLOBAL VARIABLES

// call arguments
//  - files
static char pdbfilename[255],aaafilename[255],p3dfilename[255];
static FILE *pdbfile, *aaafile, *p3dfile;
//  - options
//     NONE

// joints
static int global_indJ = 0;
static int bkb_ref_indJ = 0;

// flag
static int first_omega_after_FFsubchain = 0;

/**********************************************************************/
// GENERAL FUNCTIONS

static int read_call_arguments(int argc, char **argv)
{
   if(argc < 4)
   {
      printf("Not enought arguments\n");
      return -1;
   }
   
   if (argc==5) {
     if (strcmp(argv[1],"NO_AMBER")!=0) {
       return -1;
     }
     else {
       CONVERT_AMBER_FLAG = FALSE;
       if((!strcpy(pdbfilename,argv[2])) ||
	  (!strcpy(aaafilename,argv[3])) ||
	  (!strcpy(p3dfilename,argv[4])))
	 return -1;
       else
	 return 1;
     }
   }
   
   
   if((!strcpy(pdbfilename,argv[1])) ||
      (!strcpy(aaafilename,argv[2])) ||
      (!strcpy(p3dfilename,argv[3])))
     return -1;

   return 1;
}

/**********************************************************************/

static int right_resName(char *resName)
{
  return ((strcmp(resName,"ALA") == 0 ) ||
	  (strcmp(resName,"ARG") == 0 ) ||
	  (strcmp(resName,"ASN") == 0 ) ||
	  (strcmp(resName,"ASP") == 0 ) ||
	  (strcmp(resName,"CYS") == 0 ) ||
	  (strcmp(resName,"GLN") == 0 ) ||
	  (strcmp(resName,"GLU") == 0 ) ||
	  (strcmp(resName,"GLY") == 0 ) ||
	  (strcmp(resName,"HIS") == 0 ) ||
	  (strcmp(resName,"ILE") == 0 ) ||
	  (strcmp(resName,"LEU") == 0 ) ||
	  (strcmp(resName,"LYS") == 0 ) ||
	  (strcmp(resName,"MET") == 0 ) ||
	  (strcmp(resName,"PHE") == 0 ) ||
	  (strcmp(resName,"PRO") == 0 ) ||
	  (strcmp(resName,"SER") == 0 ) ||
	  (strcmp(resName,"THR") == 0 ) ||
	  (strcmp(resName,"TRP") == 0 ) ||
	  (strcmp(resName,"TYR") == 0 ) ||
	  (strcmp(resName,"VAL") == 0 ));
}



/**********************************************************************/
/**********************************************************************/

// SPECIFIC AAA FUNCTION (temporary)

static void subCh_aaa_list_init(subCh_aaa_list* list){
  list->nsubCh_spec = 0;
  list->subCh_spec_list = NULL;
}

static void subCh_aaa_list_free(subCh_aaa_list* list){
  if (list->nsubCh_spec!=0) {
    free(list->subCh_spec_list);
    list->subCh_spec_list = NULL;
  }
  list->nsubCh_spec = 0;
}

static void aaa_add_subch(subCh_aaa_list* list){
  if( list->nsubCh_spec==0) {
    list->subCh_spec_list = (subCh_aaa_data**)malloc(sizeof(subCh_aaa_data*));
  }
  else {
    list->subCh_spec_list = (subCh_aaa_data**)realloc(list->subCh_spec_list,sizeof(subCh_aaa_data*) * (list->nsubCh_spec + 1));
  }
  list->subCh_spec_list[list->nsubCh_spec] = (subCh_aaa_data*)malloc(sizeof(subCh_aaa_data));
  int_list_init(&(list->subCh_spec_list[list->nsubCh_spec]->bkb_flex_level));
  int_list_init(&(list->subCh_spec_list[list->nsubCh_spec]->sch_flex_level));
  list->nsubCh_spec++; // Aumenta num elementos de la lista.
}

static void aaa_set_subCh_start(subCh_aaa_list* list, int resSeq){
  if (list->nsubCh_spec>0)
    list->subCh_spec_list[list->nsubCh_spec-1]->start_resSeq = resSeq;
}

static void aaa_set_subCh_end(subCh_aaa_list* list, int resSeq){
  if (list->nsubCh_spec>0)
    list->subCh_spec_list[list->nsubCh_spec-1]->end_resSeq = resSeq;  
}

/* static void aaa_insert_art_bkb(subCh_aaa_list* list, int resSeq){ */
/*   if (list->nsubCh_spec>0) */
/*     int_list_insert(&(list->subCh_spec_list[list->nsubCh_spec-1]->art_bkb_list),resSeq); */
/* } */

/* static void aaa_insert_art_sch(subCh_aaa_list* list, int resSeq){ */
/*   if (list->nsubCh_spec>0) */
/*     int_list_insert(&(list->subCh_spec_list[list->nsubCh_spec-1]->art_sch_list),resSeq); */
/* } */

static int read_AA_from_file(const char* AAA_filename, subCh_aaa_list* subCh_spec_list)
{
  char aaaline[50];
  char piece1[5],piece2[10],name[3];
  residueTypes resType;
  char res_or_command[4];
  int resSeq;
  static FILE *aaafile;
  int error = FALSE;
  int start = FALSE;

  aaafile = fopen(AAA_filename, "r");
  if (aaafile == NULL) {
    printf("aaa file cannot be open\n");
    return FALSE;
  }
  
  while (!error && !start && fgets(aaaline,50,aaafile)) { 
    sscanf(aaaline,"%s",res_or_command);
    if(strcmp(res_or_command,"SUB") == 0)
      start = TRUE; 
    else if (resName_to_resType(res_or_command,&resType)){
      if(start)
	error = TRUE; 
    }
  }

  if (start){

    do{
      
      sscanf(aaaline,"%s",res_or_command); 

      if(strcmp(res_or_command,"SUB") == 0){
	aaa_set_subCh_end(subCh_spec_list,resSeq);
	aaa_add_subch(subCh_spec_list); 
	start = TRUE;
      }
      
      else if(resName_to_resType(res_or_command,&resType)) {
	strcpy(piece1,"    ");
	strncpy(piece1,aaaline+4,4); 
	if(!sscanf(piece1,"%d",&resSeq)){ 
	  fclose(aaafile);
	  return FALSE;
	}
	
	if(start){ 
	  aaa_set_subCh_start(subCh_spec_list,resSeq); 
	  start = FALSE;
	}
	
	strcpy(piece2,"         ");  
	strncpy(piece2,aaaline+9,9); 
	if(!sscanf(piece2,"%s",name)) { 
	  fclose(aaafile);
	  return FALSE;
	}
	if(strcmp(name,"rigid-bkb") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->bkb_flex_level),RIGID);
	else if(strcmp(name,"Lflex-bkb") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->bkb_flex_level),LFLEX);
	else if(strcmp(name,"Mflex-bkb") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->bkb_flex_level),MFLEX);
	else if(strcmp(name,"Fflex-bkb") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->bkb_flex_level),FFLEX);
	else {
	  return FALSE;
	  fclose(aaafile);
	}
	strcpy(piece2,"         ");
	strncpy(piece2,aaaline+19,9);  
	if(!sscanf(piece2,"%s",name)) {
	  fclose(aaafile);
	  return FALSE;
	}
	if(strcmp(name,"rigid-sch") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->sch_flex_level),RIGID);
	else if(strcmp(name,"Lflex-sch") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->sch_flex_level),LFLEX);
	else if(strcmp(name,"Mflex-sch") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->sch_flex_level),MFLEX);
	else if(strcmp(name,"Fflex-sch") == 0) 
	  int_list_insert(&(subCh_spec_list->subCh_spec_list[subCh_spec_list->nsubCh_spec-1]->sch_flex_level),FFLEX);
	else {
	  return FALSE;
	  fclose(aaafile);
	}
      }
      
    }while (fgets(aaaline,50,aaafile));
    
    aaa_set_subCh_end(subCh_spec_list,resSeq); 
    fclose(aaafile);
    return TRUE;
  }
  
  else{ 
    printf("invalid aaa file\n");
    return FALSE;
  }
}


/**********************************************************************/
// GENERAL WRITING FUNCTIONS

// function programmed and required by V.Ruiz (BCD)
/* returns a chain (max. 20 charcters) with the characters following
the *index position of the parameter string until the first instance 
of the second parameter is found.
index is returned with the position after the the first instance of c
*/ 

static char *givemeword(char string[], char c, int *index)
{  
  static char cadena[MAX_NAME_LENGTH + 1];
  int i=0;
  
  while ((string[*index]!= c) && (string[*index]!= '\0')){
    cadena[i++]=string[(*index)++];
  }
  if (i > (MAX_NAME_LENGTH - 1)){
    printf("ERROR en givemeword\n");//QUITAR
    printf("cadena=%s , caracter=", string);putchar(c);printf("\n");
  }
  cadena[i]='\0';
  (*index)++;
  return cadena;
}


/**********************************************************************/


static void write_p3d_head(protein *protPt)
{
  fprintf(p3dfile,"p3d_beg_desc P3D_ENV %s\n\n",p3dfilename);
}


static void write_p3d_end(protein *protPt)
{
  double dist_fNlC;
  double vdiff[3];
  atom *fatom, *latom;
  residue *lres;

  fatom = protPt->chainlist[0]->reslist[0]->bkbAlist[0];
  lres = protPt->chainlist[protPt->nchains - 1]->reslist[protPt->chainlist[protPt->nchains - 1]->nresidues - 1];
  latom= lres->bkbAlist[0];

  // for env-box dimensions
  vectSub(latom->pos,fatom->pos,vdiff);
  dist_fNlC = vectNorm(vdiff);

  fprintf(p3dfile,"p3d_end_desc\n\n");     
  fprintf(p3dfile,"\n\np3d_set_env_box %f %f %f %f %f %f\n\n",
	  fatom->pos[0] - 2.0*dist_fNlC, fatom->pos[0] + 2.0*dist_fNlC,
	  fatom->pos[1] - 2.0*dist_fNlC, fatom->pos[1] + 2.0*dist_fNlC,
	  fatom->pos[2] - 2.0*dist_fNlC, fatom->pos[2] + 2.0*dist_fNlC);  

}


/**********************************************************************/

static void write_protein_head(protein *protPt)
{
  int indexletter=0;
  atom *fatom;
  int indref;
  int numprot;
  
  fatom = protPt->chainlist[0]->reslist[0]->bkbAlist[0];

  // ONLY ONE ROBOT
  // robot mane must end with ".pep" (required by BCD)
  fprintf(p3dfile,"p3d_beg_desc P3D_ROBOT %s.pep\n\n",givemeword(protPt->name,'.',&indexletter));

  // base-joint
  // NOTE : A free-flying joint is defined as base-joint of a protein.
  //        By default, this joint is static.
  //        The frame origin corresponds with the origin of the first backbone atom (in the list)  
  
  indref = 0; // the joint is relative to jnt[0] 
  numprot = 1; // WARNING : ONLY ONE PROTEIN NOW !!!
  
  fprintf(p3dfile,"p3d_beg_desc_jnt P3D_FREEFLYER    # %s # J%d\n","PROTEIN BASE-JOINT",++global_indJ);
  // IF LIGAND 
  //  fprintf(p3dfile,"  p3d_set_name .lig_base.%d\n",numlig);
  fprintf(p3dfile,"  p3d_set_name .prot_base.%d\n",numprot);
  fprintf(p3dfile,"  p3d_set_pos_axe %f %f %f 0 0 0\n",fatom->pos[0],fatom->pos[1],fatom->pos[2]);
  fprintf(p3dfile,"  p3d_set_prev_jnt %d\n",indref);
  fprintf(p3dfile,"  p3d_set_dof 0 0 0 0 0 0\n");
  fprintf(p3dfile,"  p3d_set_dof_vmin 0 0 0 0 0 0\n");
  fprintf(p3dfile,"  p3d_set_dof_vmax 0 0 0 0 0 0\n");
  fprintf(p3dfile,"p3d_end_desc\n\n");   

}

/**********************************************************************/

static void write_joint(char *Jname, int indJ, int indref, double *Jpos, double *Jax, double v, double v0, double vmin, double vmax)
{
  fprintf(p3dfile,"p3d_beg_desc_jnt P3D_ROTATE   # J%d\n",indJ);
  fprintf(p3dfile,"  p3d_set_name %s\n",Jname);
  fprintf(p3dfile,"  p3d_set_pos_axe %f %f %f %f %f %f\n",
	  Jpos[0],Jpos[1],Jpos[2],Jax[0],Jax[1],Jax[2]);
  fprintf(p3dfile,"  p3d_set_prev_jnt %d\n",indref);
  fprintf(p3dfile,"  p3d_set_dof %f\n",v);
  fprintf(p3dfile,"  p3d_set_dof_pos0 %f\n",v0);
  fprintf(p3dfile,"  p3d_set_dof_vmin %f\n",vmin);
  fprintf(p3dfile,"  p3d_set_dof_vmax %f\n",vmax);
  fprintf(p3dfile,"p3d_end_desc\n\n");   
}

/**********************************************************************/
// WRITING BKB JOINTS

static void write_omega_jnt(residue *resPt, int indrefbkb, int bkb_flex_level)
{
  int art_bkb;
  atom *a1,*a2,*a3,*a4;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double posdiff[3],axis1[3],axis2[3],axis3[3];
  double dihedang;
  double vmin,vmax;
  char   jname[256];
  char   cadchar[256];

  // NOTE : omega is either rigid or flexible (NO DIFFERENT FLEXIBILITY LEVELS)
  art_bkb = (bkb_flex_level != RIGID);

  a1 = get_CA(resPt->prev);
  a2 = get_C(resPt->prev);
  a3 = get_N(resPt);
  a4 = get_CA(resPt);

  a1pos = a1->pos;
  a2pos = a2->pos;
  a3pos = a3->pos;
  a4pos = a4->pos;

  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,axis1);
  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);
  vectSub(a4pos,a3pos,posdiff);
  vectNormalize(posdiff,axis3);

  dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
  if(art_bkb && !first_omega_after_FFsubchain) {
    if(dihedang > 0.0) {
      if(dihedang > OMEGAMAX*(180/PI))
	vmax = dihedang;
      else
	vmax = OMEGAMAX*(180/PI);
      if(dihedang < OMEGAMIN*(180/PI))
	vmin = dihedang;
      else
	vmin = OMEGAMIN*(180/PI);      
    }
    else {
      if(dihedang > -OMEGAMIN*(180/PI))
	vmax = dihedang;
      else
	vmax = -OMEGAMIN*(180/PI);
      if(dihedang < -OMEGAMAX*(180/PI))
	vmin = dihedang;
      else
	vmin = -OMEGAMAX*(180/PI);      
    }	
    //vmin = OMEGAMIN*(180/PI);
    //vmax = OMEGAMAX*(180/PI);
  }
  else {
    vmin = vmax = dihedang;
  } 

  first_omega_after_FFsubchain = 0;

  // update global_indJ
  global_indJ ++;

  strcpy(jname,"omega");
  sprintf(cadchar,".%s",resPt->resName);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d",resPt->resSeq);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d-%d-%d-%d",a1->serial,a2->serial,a3->serial,a4->serial);
  strcat(jname,cadchar);
  write_joint(jname,global_indJ,bkb_ref_indJ,a3pos,axis2,dihedang,dihedang,vmin,vmax);
 
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  
}

static void write_phi_jnt(residue *resPt, int indrefbkb, int bkb_flex_level)
{
  atom *a1,*a2,*a3,*a4;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double posdiff[3],axis1[3],axis2[3],axis3[3];
  double dihedang;
  double vmin,vmax;
  char   jname[256];
  char   cadchar[256];
  int    canref = 1;

  if(resPt->prev != NULL) {
    a1 = get_C(resPt->prev);
    a1pos = a1->pos;
  }
  else 
    canref = 0;

  a2 = get_N(resPt);
  a3 = get_CA(resPt);
  a4 = get_C(resPt);

  a2pos = a2->pos;
  a3pos = a3->pos;
  a4pos = a4->pos;

  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);

  if(canref) {
    vectSub(a2pos,a1pos,posdiff);
    vectNormalize(posdiff,axis1);
    vectSub(a4pos,a3pos,posdiff);
    vectNormalize(posdiff,axis3);

    // NOTE : for PROLINE : we consider phi ~fixed at the modeling value !!!

    dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
    if(bkb_flex_level != RIGID) {
      if((resPt->resType == PRO) || (resPt->resType == PROH)) {
	fprintf(p3dfile,"#WARNING : PROLINE : PHI IS CONSTRAINED !\n");
	vmin = dihedang - 10.0;
	vmax = dihedang + 10.0;
      }
      else {
	if(bkb_flex_level == FFLEX) {
	  vmin = -180.0;
	  vmax =  180.0;
	}
	else if(bkb_flex_level == MFLEX) {
	  vmin = dihedang - 30.0;
	  vmax = dihedang + 30.0;
	}
	else { // (bkb_flex_level == LFLEX) 
	  vmin = dihedang - 10.0;
	  vmax = dihedang + 10.0;
	}	
      }
    }
    else {
      vmin = vmax = dihedang;
    } 
  }
  else {
    // NOTE : this angle could be variable, but it can't be refered
    vmin = vmax = dihedang = 0.0;
  }

  // update global_indJ
  global_indJ ++;

  strcpy(jname,"phi");
  sprintf(cadchar,".%s",resPt->resName);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d",resPt->resSeq);
  strcat(jname,cadchar);
  if(canref)
    sprintf(cadchar,".%d-%d-%d-%d",a1->serial,a2->serial,a3->serial,a4->serial);
  else
    sprintf(cadchar,".%d-%d-%d-%d",0,0,0,0);
  strcat(jname,cadchar);
  write_joint(jname,global_indJ,bkb_ref_indJ,a3pos,axis2,dihedang,dihedang,vmin,vmax);
 
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  
}

static void write_psi_jnt(residue *resPt, int indrefbkb, int bkb_flex_level)
{
  atom *a1,*a2,*a3,*a4;
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double posdiff[3],axis1[3],axis2[3],axis3[3];
  double dihedang;
  double vmin,vmax;
  char   jname[256];
  char   cadchar[256];
  int    canref = 1;

  a1 = get_N(resPt);
  a2 = get_CA(resPt);
  a3 = get_C(resPt);

  a1pos = a1->pos;
  a2pos = a2->pos;
  a3pos = a3->pos;

  if(resPt->next != NULL) {
    a4 = get_N(resPt->next);
    a4pos = a4->pos;
  }
  else
    canref = 0;

  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);
  
  if(canref) {
    vectSub(a2pos,a1pos,posdiff);
    vectNormalize(posdiff,axis1);
    vectSub(a4pos,a3pos,posdiff);
    vectNormalize(posdiff,axis3);

    dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
    if(bkb_flex_level != RIGID) {
      if(bkb_flex_level == FFLEX) {
	vmin = -180.0;
	vmax =  180.0;
      }
      else if(bkb_flex_level == MFLEX) {
	vmin = dihedang - 30.0;
	vmax = dihedang + 30.0;
      }
      else { // (bkb_flex_level == LFLEX) 
	vmin = dihedang - 10.0;
	vmax = dihedang + 10.0;
      }	
    }
    else {
      vmin = vmax = dihedang;
    } 
  }
  else {
    // NOTE : this angle could be variable, but it can't be refered
    vmin = vmax = dihedang = 0.0;
  }

  // update global_indJ
  global_indJ ++;

  strcpy(jname,"psi");
  sprintf(cadchar,".%s",resPt->resName);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d",resPt->resSeq);
  strcat(jname,cadchar);
  if(canref)
    sprintf(cadchar,".%d-%d-%d-%d",a1->serial,a2->serial,a3->serial,a4->serial);
  else
    sprintf(cadchar,".%d-%d-%d-%d",0,0,0,0);
  strcat(jname,cadchar);
  write_joint(jname,global_indJ,bkb_ref_indJ,a3pos,axis2,dihedang,dihedang,vmin,vmax);
 
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  
}

static void write_gamma_jnt(residue *resPt, int sch_flex_level, int indref, char *name,
			    atom *a1, atom *a2, atom *a3, atom *a4)
{
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double posdiff[3],axis1[3],axis2[3],axis3[3];
  double dihedang;
  double vmin,vmax;
  char   jname[256];
  char   cadchar[256];  

  a1pos = a1->pos;
  a2pos = a2->pos;
  a3pos = a3->pos;
  a4pos = a4->pos;

  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,axis1);
  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);
  vectSub(a4pos,a3pos,posdiff);
  vectNormalize(posdiff,axis3);
  
  dihedang = compute_dihedang(axis3,axis2,axis1)*(180/PI);
  if(sch_flex_level == FFLEX) {
    vmin = -180.0;
    vmax =  180.0;
  }
  else if(sch_flex_level == MFLEX) {
    vmin = dihedang - 30.0;
    vmax = dihedang + 30.0;
  }
  else { // (sch_flex_level == LFLEX) 
    vmin = dihedang - 10.0;
    vmax = dihedang + 10.0;
  }	
  
  // update global_indJ
  global_indJ ++;
  
  strcpy(jname,name);
  sprintf(cadchar,".%s",resPt->resName);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d",resPt->resSeq);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d-%d-%d-%d",a1->serial,a2->serial,a3->serial,a4->serial);
  strcat(jname,cadchar);
  write_joint(jname,global_indJ,indref,a3pos,axis2,dihedang,dihedang,vmin,vmax);   

}


/**********************************************************************/

static void write_HPrime(atom *aPt){
  
  int i;
  atom* bondedH = NULL;
  int serial;
  int exit = FALSE;

  for (i=0; i<aPt->nbondedA && !exit; i++) {
    if (aPt->bondedAlist[i]->atomType==HYDROGEN_P) {
      bondedH = aPt->bondedAlist[i];

      if (CONVERT_AMBER_FLAG) {
	exit = !get_amber_serial(bondedH, &serial);
	if (exit) printf("ERROR while trying to create p3d atom %d with amber serial\n", aPt->serial);
      }
      else {
	serial = bondedH->serial;
      }

      if (!exit) {
	fprintf(p3dfile," p3d_add_desc_sphere X-%s.%d %f  P3D_GHOST\n",bondedH->name,serial,H_VDWR);    
	fprintf(p3dfile," p3d_set_prim_pos X-%s.%d %f %f %f 0.0 0.0 0.0\n",
		bondedH->name,serial,bondedH->pos[0],bondedH->pos[1],bondedH->pos[2]);
	fprintf(p3dfile," p3d_set_prim_color X-%s.%d %s\n",bondedH->name,serial,H_COLOR);
      }
    }
  }
}

/**********************************************************************/
// WRITING ATOMS 

// BE CAREFULL ! Each time a body involving H' is going
// to be made, the existence of this H' atom must be
// tested (because H' atoms are present in some models
// and missing in others)

static void write_atom(atom *aPt)
{
  int exit = FALSE;
  double radius;
  char* color;
  int possible_HPrime = FALSE;
  int serial = aPt->serial;
  char collision_Detection_Type = COLLISION_DETECTION_CHAR;

  switch(aPt->atomType) {
  case CARBON:
    radius = C_VDWR;
    color = C_COLOR;
    break;
  case NITROGEN:
  case NITROGEN_H:
  case NITROGEN_FULL:
    possible_HPrime = aPt->residuePt->flagH;
    radius = N_VDWR;
    color = N_COLOR;
    break;
  case OXYGEN:
  case OXYGEN_H:
    // OXT MUST NOT BE CONSIDERED BY THE BIOCD 
    if (strcmp(aPt->name,"OXT")== 0) {
	collision_Detection_Type = NO_COLLISION_DETECTION_CHAR;      
    }
    possible_HPrime = aPt->residuePt->flagH;
    radius = O_VDWR;
    color = O_COLOR;
    break; 
  case SULPHUR:
  case SULPHUR_H:
    possible_HPrime = aPt->residuePt->flagH;
    radius = S_VDWR;
    color = S_COLOR;
    break; 
  case HYDROGEN:
    radius = H_VDWR;
    color = H_COLOR;
    break;
  default: 
    exit = TRUE; 
  }


  if (!exit) {

    if (CONVERT_AMBER_FLAG) {
      exit = !get_amber_serial(aPt, &serial);
      if (exit) printf("ERROR while trying to create p3d atom %d with amber serial\n", aPt->serial);
    }
    
    if (!exit) {
      fprintf(p3dfile," p3d_add_desc_sphere %c-%s.%d %f\n",collision_Detection_Type,aPt->name,serial,radius);    
      fprintf(p3dfile," p3d_set_prim_pos %c-%s.%d %f %f %f 0.0 0.0 0.0\n",
	      collision_Detection_Type,aPt->name,serial,aPt->pos[0],aPt->pos[1],aPt->pos[2]);
      fprintf(p3dfile," p3d_set_prim_color %c-%s.%d %s\n",collision_Detection_Type,aPt->name,serial,color);
      
      if (possible_HPrime) {
	write_HPrime(aPt); 
      }
    }
  }
  
}





/**********************************************************************/

static void write_ALAH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : 1HB
    a4 = resPt->schAlist[ALAH_1HB];

    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ALAH_1HB]);
  write_atom(resPt->schAlist[ALAH_2HB]);
  write_atom(resPt->schAlist[ALAH_3HB]);
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ARG_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[ARG_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARG_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[ARG_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[ARG_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARG_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[gen_CB];
    a2 = resPt->schAlist[ARG_CG];
    a3 = resPt->schAlist[ARG_CD];
    // NOTE : ref for dihedang : NE
    a4 = resPt->schAlist[ARG_NE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARG_NE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma4 
    a1 = resPt->schAlist[ARG_CG];
    a2 = resPt->schAlist[ARG_CD];
    a3 = resPt->schAlist[ARG_NE];
    // NOTE : ref for dihedang : CZ
    a4 = resPt->schAlist[ARG_CZ];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma4",a1,a2,a3,a4);
  }

  // write fourth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-4",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARG_CZ]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma5 
    a1 = resPt->schAlist[ARG_CD];
    a2 = resPt->schAlist[ARG_NE];
    a3 = resPt->schAlist[ARG_CZ];
    // NOTE : ref for dihedang : NH1
    a4 = resPt->schAlist[ARG_NH1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma5",a1,a2,a3,a4);
  }

  // write fifth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-5",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARG_NH1]);    
  write_atom(resPt->schAlist[ARG_NH2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ARGH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    // NOTE : ref for dihedang : CG
    a3 = resPt->bkbAlist[genH_CB];
    a4 = resPt->schAlist[ARGH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARGH_1HB]);    
  write_atom(resPt->schAlist[ARGH_2HB]);    
  write_atom(resPt->schAlist[ARGH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ARGH_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[ARGH_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARGH_1HG]);    
  write_atom(resPt->schAlist[ARGH_2HG]);    
  write_atom(resPt->schAlist[ARGH_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[ARGH_CG];
    a3 = resPt->schAlist[ARGH_CD];
    // NOTE : ref for dihedang : NE
    a4 = resPt->schAlist[ARGH_NE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARGH_1HD]);    
  write_atom(resPt->schAlist[ARGH_2HD]);    
  write_atom(resPt->schAlist[ARGH_NE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma4 
    a1 = resPt->schAlist[ARGH_CG];
    a2 = resPt->schAlist[ARGH_CD];
    a3 = resPt->schAlist[ARGH_NE];
    // NOTE : ref for dihedang : CZ
    a4 = resPt->schAlist[ARGH_CZ];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma4",a1,a2,a3,a4);
  }

  // write fourth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-4",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARGH_CZ]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma5 
    a1 = resPt->schAlist[ARGH_CD];
    a2 = resPt->schAlist[ARGH_NE];
    a3 = resPt->schAlist[ARGH_CZ];
    // NOTE : ref for dihedang : NH1
    a4 = resPt->schAlist[ARGH_NH1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma5",a1,a2,a3,a4);
  }

  // write fifth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-5",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ARGH_NH1]);    
  write_atom(resPt->schAlist[ARGH_NH2]);    
  // H' atoms
/*   if(resPt->schAlist[ARGH_1HH1]!=NULL) {write_atom(resPt->schAlist[ARGH_1HH1]);}    */
/*   if(resPt->schAlist[ARGH_2HH1]!=NULL) {write_atom(resPt->schAlist[ARGH_2HH1]);}     */
/*   if(resPt->schAlist[ARGH_1HH2]!=NULL) {write_atom(resPt->schAlist[ARGH_1HH2]);}     */
/*   if(resPt->schAlist[ARGH_2HH2]!=NULL) {write_atom(resPt->schAlist[ARGH_2HH2]);}     */
/*   if(resPt->schAlist[ARGH_HE]!=NULL) {write_atom(resPt->schAlist[ARGH_HE]);}     */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ASN_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[ASN_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASN_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[ASN_CG];
    // NOTE : ref for dihedang : OD1
    a4 = resPt->schAlist[ASN_OD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASN_OD1]);    
  write_atom(resPt->schAlist[ASN_ND2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ASNH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[ASNH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASNH_1HB]);    
  write_atom(resPt->schAlist[ASNH_2HB]);    
  write_atom(resPt->schAlist[ASNH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ASNH_CG];
    // NOTE : ref for dihedang : OD1
    a4 = resPt->schAlist[ASNH_OD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASNH_OD1]);    
  write_atom(resPt->schAlist[ASNH_ND2]);      
  // H' atoms
/*   if(resPt->schAlist[ASNH_1HD2]!=NULL) {write_atom(resPt->schAlist[ASNH_1HD2]);} */
/*   if(resPt->schAlist[ASNH_2HD2]!=NULL) {write_atom(resPt->schAlist[ASNH_2HD2]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");  
}


static void write_ASP_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[ASP_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASP_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[ASP_CG];
    // NOTE : ref for dihedang : OD1
    a4 = resPt->schAlist[ASP_OD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASP_OD1]);    
  write_atom(resPt->schAlist[ASP_OD2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ASPH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[ASPH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASPH_1HB]);    
  write_atom(resPt->schAlist[ASPH_2HB]);    
  write_atom(resPt->schAlist[ASPH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ASPH_CG];
    // NOTE : ref for dihedang : OD1
    a4 = resPt->schAlist[ASPH_OD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ASPH_OD1]);    
  write_atom(resPt->schAlist[ASPH_OD2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");  
}


static void write_CYS_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : SG
    a4 = resPt->schAlist[CYS_SG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[CYS_SG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_CYSH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : SG
    a4 = resPt->schAlist[CYSH_SG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[CYSH_1HB]);    
  write_atom(resPt->schAlist[CYSH_2HB]);      
  write_atom(resPt->schAlist[CYSH_SG]);    
  //  write_atom(resPt->schAlist[CYSH_HG]);
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_GLN_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[GLN_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLN_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[GLN_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[GLN_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLN_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[gen_CB];
    a2 = resPt->schAlist[GLN_CG];
    a3 = resPt->schAlist[GLN_CD];
    // NOTE : ref for dihedang : OE1
    a4 = resPt->schAlist[GLN_OE1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLN_OE1]);    
  write_atom(resPt->schAlist[GLN_NE2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_GLNH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[GLNH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLNH_1HB]);    
  write_atom(resPt->schAlist[GLNH_2HB]);    
  write_atom(resPt->schAlist[GLNH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[GLNH_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[GLNH_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLNH_1HG]);    
  write_atom(resPt->schAlist[GLNH_2HG]);    
  write_atom(resPt->schAlist[GLNH_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[GLNH_CG];
    a3 = resPt->schAlist[GLNH_CD];
    // NOTE : ref for dihedang : OE1
    a4 = resPt->schAlist[GLNH_OE1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLNH_OE1]);    
  write_atom(resPt->schAlist[GLNH_NE2]);    
  // H' atoms
/*   if(resPt->schAlist[GLNH_1HE2]!=NULL) {write_atom(resPt->schAlist[GLNH_1HE2]);} */
/*   if(resPt->schAlist[GLNH_2HE2]!=NULL) {write_atom(resPt->schAlist[GLNH_2HE2]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_GLU_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
   a4 = resPt->schAlist[GLU_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLU_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[GLU_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[GLU_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLU_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[gen_CB];
    a2 = resPt->schAlist[GLU_CG];
    a3 = resPt->schAlist[GLU_CD];
    // NOTE : ref for dihedang : OE1
    a4 = resPt->schAlist[GLU_OE1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLU_OE1]);    
  write_atom(resPt->schAlist[GLU_OE2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_GLUH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[GLUH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLUH_1HB]);    
  write_atom(resPt->schAlist[GLUH_2HB]);    
  write_atom(resPt->schAlist[GLUH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[GLUH_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[GLUH_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLUH_1HG]);    
  write_atom(resPt->schAlist[GLUH_2HG]);    
  write_atom(resPt->schAlist[GLUH_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[GLUH_CG];
    a3 = resPt->schAlist[GLUH_CD];
    // NOTE : ref for dihedang : OE1
    a4 = resPt->schAlist[GLUH_OE1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[GLUH_OE1]);    
  write_atom(resPt->schAlist[GLUH_OE2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_HIS_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[HIS_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[HIS_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[HIS_CG];
    // NOTE : ref for dihedang : ND1
    a4 = resPt->schAlist[HIS_ND1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[HIS_ND1]);    
  write_atom(resPt->schAlist[HIS_CD2]);      
  write_atom(resPt->schAlist[HIS_CE1]);    
  write_atom(resPt->schAlist[HIS_NE2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_HISH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[HISH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[HISH_1HB]);    
  write_atom(resPt->schAlist[HISH_2HB]);    
  write_atom(resPt->schAlist[HISH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[HISH_CG];
    // NOTE : ref for dihedang : ND1
    a4 = resPt->schAlist[HISH_ND1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[HISH_ND1]);    
  write_atom(resPt->schAlist[HISH_CD2]);      
  write_atom(resPt->schAlist[HISH_HD2]);      
  write_atom(resPt->schAlist[HISH_CE1]);    
  write_atom(resPt->schAlist[HISH_NE2]);      
  write_atom(resPt->schAlist[HISH_HE1]);
  // H' atoms      
/*   if (resPt->schAlist[HISH_HD1]) {write_atom(resPt->schAlist[HISH_HD1]);} */
/*   if (resPt->schAlist[HISH_HE2]) {write_atom(resPt->schAlist[HISH_HE2]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ILE_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG1
    a4 = resPt->schAlist[ILE_CG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILE_CG2]);    
  write_atom(resPt->schAlist[ILE_CG1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[ILE_CG1];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[ILE_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILE_CD1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ILEH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG1
    a4 = resPt->schAlist[ILEH_CG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_CG2]);    
  write_atom(resPt->schAlist[ILEH_HB]);    
  write_atom(resPt->schAlist[ILEH_CG1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2-2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ILEH_CG2];
    // NOTE : ref for dihedang : 1HG2
    a4 = resPt->schAlist[ILEH_1HG2];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2-2",a1,a2,a3,a4);
  }

  // write second-2 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_1HG2]);    
  write_atom(resPt->schAlist[ILEH_2HG2]);    
  write_atom(resPt->schAlist[ILEH_3HG2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2-1 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ILEH_CG1];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[ILEH_CD1];
    // gamma2-1 is refered to the gamma1 (global_indJ - 1)
    write_gamma_jnt(resPt,art_sch,(global_indJ - 1),"gamma2-1",a1,a2,a3,a4);
  }

  // write second-1 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_1HG1]);    
  write_atom(resPt->schAlist[ILEH_2HG1]);    
  write_atom(resPt->schAlist[ILEH_CD1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[ILEH_CG1];
    a3 = resPt->schAlist[ILEH_CD1];
    // NOTE : ref for dihedang : 1HD1
    a4 = resPt->schAlist[ILEH_1HD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_1HD1]);    
  write_atom(resPt->schAlist[ILEH_2HD1]);    
  write_atom(resPt->schAlist[ILEH_3HD1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_ILEH_sidechain_NEW(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG1
    a4 = resPt->schAlist[ILEH_CG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_CG2]);    
  write_atom(resPt->schAlist[ILEH_HB]);    
  write_atom(resPt->schAlist[ILEH_CG1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2-1 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ILEH_CG1];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[ILEH_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2-1",a1,a2,a3,a4);
  }

  // write second-2 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_1HG1]);    
  write_atom(resPt->schAlist[ILEH_2HG1]);    
  write_atom(resPt->schAlist[ILEH_CD1]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2-2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[ILEH_CG2];
    // NOTE : ref for dihedang : 1HG2
    a4 = resPt->schAlist[ILEH_1HG2];
    // gamma2-2 is refered to the gamma1 (global_indJ - 1)
    write_gamma_jnt(resPt,art_sch,(global_indJ - 1),"gamma2-2",a1,a2,a3,a4);
  }

  // write second-1 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_1HG2]);    
  write_atom(resPt->schAlist[ILEH_2HG2]);    
  write_atom(resPt->schAlist[ILEH_3HG2]);  
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[ILEH_CG1];
    a3 = resPt->schAlist[ILEH_CD1];
    // NOTE : ref for dihedang : 1HD1
    a4 = resPt->schAlist[ILEH_1HD1];
    // gamma3 is refered to the gamma2-1 (global_indJ - 1)
    write_gamma_jnt(resPt,art_sch,(global_indJ - 1),"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[ILEH_1HD1]);    
  write_atom(resPt->schAlist[ILEH_2HD1]);    
  write_atom(resPt->schAlist[ILEH_3HD1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_LEU_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[LEU_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LEU_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[LEU_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[LEU_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LEU_CD1]);    
  write_atom(resPt->schAlist[LEU_CD2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_LEUH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[LEUH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LEUH_1HB]);    
  write_atom(resPt->schAlist[LEUH_2HB]);    
  write_atom(resPt->schAlist[LEUH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[LEUH_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[LEUH_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LEUH_HG]);    
  write_atom(resPt->schAlist[LEUH_CD1]);    
  write_atom(resPt->schAlist[LEUH_CD2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");  

  if(art_sch != RIGID) {
    // write joint gamma3-1 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[LEUH_CG];
    a3 = resPt->schAlist[LEUH_CD1];
    // NOTE : ref for dihedang : 1HD1
    a4 = resPt->schAlist[LEUH_1HD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3-1",a1,a2,a3,a4);
  }

  // write third-1 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LEUH_1HD1]);    
  write_atom(resPt->schAlist[LEUH_2HD1]);      
  write_atom(resPt->schAlist[LEUH_3HD1]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");  

  if(art_sch != RIGID) {
    // write joint gamma3-2 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[LEUH_CG];
    a3 = resPt->schAlist[LEUH_CD2];
    // NOTE : ref for dihedang : 1HD2
    a4 = resPt->schAlist[LEUH_1HD2];
    // gamma3-2 is refered to the gamma2 (global_indJ - 1)
    write_gamma_jnt(resPt,art_sch,(global_indJ - 1),"gamma3-2",a1,a2,a3,a4);
  }

  // write third-2 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LEUH_1HD2]);    
  write_atom(resPt->schAlist[LEUH_2HD2]);      
  write_atom(resPt->schAlist[LEUH_3HD2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");  
}


static void write_LYS_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[LYS_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYS_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[LYS_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[LYS_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYS_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[gen_CB];
    a2 = resPt->schAlist[LYS_CG];
    a3 = resPt->schAlist[LYS_CD];
    // NOTE : ref for dihedang : CE
    a4 = resPt->schAlist[LYS_CE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYS_CE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma4 
    a1 = resPt->schAlist[LYS_CG];
    a2 = resPt->schAlist[LYS_CD];
    a3 = resPt->schAlist[LYS_CE];
    // NOTE : ref for dihedang : NZ
    a4 = resPt->schAlist[LYS_NZ];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma4",a1,a2,a3,a4);
  }

  // write fourth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-4",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYS_NZ]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_LYSH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    // NOTE : ref for dihedang : CG
    a3 = resPt->bkbAlist[genH_CB];
    a4 = resPt->schAlist[LYSH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYSH_1HB]);    
  write_atom(resPt->schAlist[LYSH_2HB]);    
  write_atom(resPt->schAlist[LYSH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[LYSH_CG];
    // NOTE : ref for dihedang : CD
    a4 = resPt->schAlist[LYSH_CD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYSH_1HG]);    
  write_atom(resPt->schAlist[LYSH_2HG]);    
  write_atom(resPt->schAlist[LYSH_CD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[LYSH_CG];
    a3 = resPt->schAlist[LYSH_CD];
    // NOTE : ref for dihedang : CE
    a4 = resPt->schAlist[LYSH_CE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYSH_1HD]);    
  write_atom(resPt->schAlist[LYSH_2HD]);    
  write_atom(resPt->schAlist[LYSH_CE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma4 
    a1 = resPt->schAlist[LYSH_CG];
    a2 = resPt->schAlist[LYSH_CD];
    a3 = resPt->schAlist[LYSH_CE];
    // NOTE : ref for dihedang : NZ
    a4 = resPt->schAlist[LYSH_NZ];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma4",a1,a2,a3,a4);
  }

  // write fourth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-4",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[LYSH_1HE]);    
  write_atom(resPt->schAlist[LYSH_2HE]);    
  write_atom(resPt->schAlist[LYSH_NZ]);    
  // H' atoms
/*   if(resPt->schAlist[LYSH_1HZ]!=NULL) {write_atom(resPt->schAlist[LYSH_1HZ]);} */
/*   if(resPt->schAlist[LYSH_2HZ]!=NULL) {write_atom(resPt->schAlist[LYSH_2HZ]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_MET_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[MET_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[MET_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[MET_CG];
    // NOTE : ref for dihedang : SD
    a4 = resPt->schAlist[MET_SD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[MET_SD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[gen_CB];
    a2 = resPt->schAlist[MET_CG];
    a3 = resPt->schAlist[MET_SD];
    // NOTE : ref for dihedang : CE
    a4 = resPt->schAlist[MET_CE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[MET_CE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}

static void write_METH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    // NOTE : ref for dihedang : CG
    a3 = resPt->bkbAlist[genH_CB];
    a4 = resPt->schAlist[METH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[METH_1HB]);    
  write_atom(resPt->schAlist[METH_2HB]);    
  write_atom(resPt->schAlist[METH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[METH_CG];
    // NOTE : ref for dihedang : SD
    a4 = resPt->schAlist[METH_SD];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[METH_1HG]);    
  write_atom(resPt->schAlist[METH_2HG]);    
  write_atom(resPt->schAlist[METH_SD]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma3 
    a1 = resPt->bkbAlist[genH_CB];
    a2 = resPt->schAlist[METH_CG];
    a3 = resPt->schAlist[METH_SD];
    // NOTE : ref for dihedang : CE
    a4 = resPt->schAlist[METH_CE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma3",a1,a2,a3,a4);
  }

  // write third sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-3",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[METH_CE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma4 
    a1 = resPt->schAlist[METH_CG];
    a2 = resPt->schAlist[METH_SD];
    a3 = resPt->schAlist[METH_CE];
    // NOTE : ref for dihedang : 1HE
    a4 = resPt->schAlist[METH_1HE];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma4",a1,a2,a3,a4);
  }

  // write fourth sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-4",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[METH_1HE]);    
  write_atom(resPt->schAlist[METH_2HE]);    
  write_atom(resPt->schAlist[METH_3HE]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_PHE_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[PHE_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[PHE_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[PHE_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[PHE_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[PHE_CD1]);    
  write_atom(resPt->schAlist[PHE_CD2]);      
  write_atom(resPt->schAlist[PHE_CE1]);    
  write_atom(resPt->schAlist[PHE_CE2]);      
  write_atom(resPt->schAlist[PHE_CZ]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_PHEH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[PHEH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[PHEH_1HB]);    
  write_atom(resPt->schAlist[PHEH_2HB]);    
  write_atom(resPt->schAlist[PHEH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[PHEH_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[PHEH_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[PHEH_CD1]);    
  write_atom(resPt->schAlist[PHEH_CD2]);      
  write_atom(resPt->schAlist[PHEH_HD1]);      
  write_atom(resPt->schAlist[PHEH_HD2]);      
  write_atom(resPt->schAlist[PHEH_CE1]);    
  write_atom(resPt->schAlist[PHEH_CE2]);      
  write_atom(resPt->schAlist[PHEH_CZ]);      
  write_atom(resPt->schAlist[PHEH_HE1]);      
  write_atom(resPt->schAlist[PHEH_HE2]);      
  write_atom(resPt->schAlist[PHEH_HZ]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_SER_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : OG
    a4 = resPt->schAlist[SER_OG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[SER_OG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_SERH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : OG
    a4 = resPt->schAlist[SERH_OG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[SERH_1HB]);    
  write_atom(resPt->schAlist[SERH_2HB]);      
  write_atom(resPt->schAlist[SERH_OG]);
  // H' atoms
/*   if(resPt->schAlist[SERH_HG]!=NULL) {write_atom(resPt->schAlist[SERH_HG]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_THR_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : OG1
    a4 = resPt->schAlist[THR_OG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[THR_OG1]);    
  write_atom(resPt->schAlist[THR_CG2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}

static void write_THRH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : OG1
    a4 = resPt->schAlist[THRH_OG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[THRH_HB]);    
  write_atom(resPt->schAlist[THRH_OG1]);    
  write_atom(resPt->schAlist[THRH_CG2]);
  // H' atoms
/*   if(resPt->schAlist[THRH_HG1]) {write_atom(resPt->schAlist[THRH_HG1]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[THRH_CG2];
    // NOTE : ref for dihedang : 1HG2
    a4 = resPt->schAlist[THRH_1HG2];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[THRH_1HG2]);    
  write_atom(resPt->schAlist[THRH_2HG2]);    
  write_atom(resPt->schAlist[THRH_3HG2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_TRP_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[TRP_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TRP_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[TRP_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[TRP_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TRP_CD1]);    
  write_atom(resPt->schAlist[TRP_CD2]);      
  write_atom(resPt->schAlist[TRP_NE1]);    
  write_atom(resPt->schAlist[TRP_CE2]);      
  write_atom(resPt->schAlist[TRP_CE3]);      
  write_atom(resPt->schAlist[TRP_CZ2]);      
  write_atom(resPt->schAlist[TRP_CZ3]);      
  write_atom(resPt->schAlist[TRP_CH2]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_TRPH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[TRPH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TRPH_1HB]);    
  write_atom(resPt->schAlist[TRPH_2HB]);    
  write_atom(resPt->schAlist[TRPH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[TRPH_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[TRPH_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TRPH_CD1]);    
  write_atom(resPt->schAlist[TRPH_CD2]);      
  write_atom(resPt->schAlist[TRPH_HD1]);      
  write_atom(resPt->schAlist[TRPH_NE1]);    
  write_atom(resPt->schAlist[TRPH_CE2]);      
  write_atom(resPt->schAlist[TRPH_CE3]);      
  write_atom(resPt->schAlist[TRPH_HE3]);      
  write_atom(resPt->schAlist[TRPH_CZ2]);      
  write_atom(resPt->schAlist[TRPH_CZ3]);      
  write_atom(resPt->schAlist[TRPH_CH2]);      
  write_atom(resPt->schAlist[TRPH_HZ2]);      
  write_atom(resPt->schAlist[TRPH_HZ3]);      
  write_atom(resPt->schAlist[TRPH_HH2]);      
  // H' atoms
/*   if (resPt->schAlist[TRPH_HE1]!=NULL) {write_atom(resPt->schAlist[TRPH_HE1]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_TYR_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[TYR_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TYR_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[gen_CB];
    a3 = resPt->schAlist[TYR_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[TYR_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TYR_CD1]);    
  write_atom(resPt->schAlist[TYR_CD2]);      
  write_atom(resPt->schAlist[TYR_CE1]);    
  write_atom(resPt->schAlist[TYR_CE2]);      
  write_atom(resPt->schAlist[TYR_CZ]);      
  write_atom(resPt->schAlist[TYR_OH]);      
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_TYRH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG
    a4 = resPt->schAlist[TYRH_CG];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TYRH_1HB]);    
  write_atom(resPt->schAlist[TYRH_2HB]);    
  write_atom(resPt->schAlist[TYRH_CG]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[TYRH_CG];
    // NOTE : ref for dihedang : CD1
    a4 = resPt->schAlist[TYRH_CD1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2",a1,a2,a3,a4);
  }

  // write second sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[TYRH_CD1]);    
  write_atom(resPt->schAlist[TYRH_CD2]);      
  write_atom(resPt->schAlist[TYRH_CE1]);    
  write_atom(resPt->schAlist[TYRH_CE2]);      
  write_atom(resPt->schAlist[TYRH_HD1]);      
  write_atom(resPt->schAlist[TYRH_HD2]);      
  write_atom(resPt->schAlist[TYRH_HE1]);      
  write_atom(resPt->schAlist[TYRH_HE2]);      
  write_atom(resPt->schAlist[TYRH_CZ]);      
  write_atom(resPt->schAlist[TYRH_OH]);      
  // H' atoms
/*   if (resPt->schAlist[TYRH_HH]!=NULL) {write_atom(resPt->schAlist[TYRH_HH]);} */
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_VAL_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[gen_CB];
    // NOTE : ref for dihedang : CG1
    a4 = resPt->schAlist[VAL_CG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[VAL_CG1]);    
  write_atom(resPt->schAlist[VAL_CG2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}

static void write_VALH_sidechain(residue *resPt, int art_sch, int wb)
{
  atom *a1,*a2,*a3,*a4;

  if(art_sch != RIGID) {
    // write joint gamma1 
    a1 = get_N(resPt);
    a2 = get_CA(resPt);
    a3 = resPt->bkbAlist[genH_CB];
    // NOTE : ref for dihedang : CG1
    a4 = resPt->schAlist[VALH_CG1];
    // gamma1 is refered to the last bkb joint 
    write_gamma_jnt(resPt,art_sch,bkb_ref_indJ,"gamma1",a1,a2,a3,a4);
  }

  // write first sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[VALH_HB]);    
  write_atom(resPt->schAlist[VALH_CG1]);    
  write_atom(resPt->schAlist[VALH_CG2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   

  if(art_sch != RIGID) {
    // write joint gamma2-1 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[VALH_CG1];
    // NOTE : ref for dihedang : 1HG1
    a4 = resPt->schAlist[VALH_1HG1];
    write_gamma_jnt(resPt,art_sch,global_indJ,"gamma2-1",a1,a2,a3,a4);
  }

  // write second-1 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2-1",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[VALH_1HG1]);    
  write_atom(resPt->schAlist[VALH_2HG1]);    
  write_atom(resPt->schAlist[VALH_3HG1]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   


  if(art_sch != RIGID) {
    // write joint gamma2-2 
    a1 = get_CA(resPt);
    a2 = resPt->bkbAlist[genH_CB];
    a3 = resPt->schAlist[VALH_CG2];
    // NOTE : ref for dihedang : 1HG2
    a4 = resPt->schAlist[VALH_1HG2];
    // gamma2-2 is refered to the gamma1 (global_indJ - 1)
    write_gamma_jnt(resPt,art_sch,(global_indJ - 1),"gamma2-2",a1,a2,a3,a4);
  }

  // write second-2 sch rigid
  if(wb) fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","side-chain-2-2",
	  resPt->resName,resPt->resSeq,resPt->chainID);
  write_atom(resPt->schAlist[VALH_1HG2]);    
  write_atom(resPt->schAlist[VALH_2HG2]);    
  write_atom(resPt->schAlist[VALH_3HG2]);    
  if(wb) fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_sidechain(residue *resPt, int sch_flex_level, int wb)
{
  switch(resPt->resType) {
  case ALA:
    // no side-chain in these cases 
    break;
  case ALAH: 
    write_ALAH_sidechain(resPt,sch_flex_level,wb);
    break;
  case ARG:
    write_ARG_sidechain(resPt,sch_flex_level,wb);
    break;    
  case ARGH: 
    write_ARGH_sidechain(resPt,sch_flex_level,wb);
    break;    
  case ASN: 
    write_ASN_sidechain(resPt,sch_flex_level,wb);
    break;
  case ASNH: 
    write_ASNH_sidechain(resPt,sch_flex_level,wb);
    break;
  case ASP: 
    write_ASP_sidechain(resPt,sch_flex_level,wb);
    break;
  case ASPH: 
    write_ASPH_sidechain(resPt,sch_flex_level,wb);
    break;
  case CYS: 
    write_CYS_sidechain(resPt,sch_flex_level,wb);
    break;
  case CYSH: 
    write_CYSH_sidechain(resPt,sch_flex_level,wb);
    break;
  case GLN: 
    write_GLN_sidechain(resPt,sch_flex_level,wb);
    break;
   case GLNH: 
    write_GLNH_sidechain(resPt,sch_flex_level,wb);
    break;
  case GLU: 
    write_GLU_sidechain(resPt,sch_flex_level,wb);
    break;
  case GLUH: 
    write_GLUH_sidechain(resPt,sch_flex_level,wb);
    break;
  case GLY: 
  case GLYH: 
    // no side-chain in these cases 
    break;
  case HIS: 
    write_HIS_sidechain(resPt,sch_flex_level,wb);
    break;
   case HISH: 
    write_HISH_sidechain(resPt,sch_flex_level,wb);
    break;
  case ILE: 
    write_ILE_sidechain(resPt,sch_flex_level,wb);
    break;
  case ILEH: 
    write_ILEH_sidechain(resPt,sch_flex_level,wb);
    break;
  case LEU: 
    write_LEU_sidechain(resPt,sch_flex_level,wb);
    break;
  case LEUH: 
    write_LEUH_sidechain(resPt,sch_flex_level,wb);
    break;
  case LYS: 
    write_LYS_sidechain(resPt,sch_flex_level,wb);
    break;
  case LYSH: 
    write_LYSH_sidechain(resPt,sch_flex_level,wb);
    break;
  case MET: 
    write_MET_sidechain(resPt,sch_flex_level,wb);
    break;
  case METH: 
    write_METH_sidechain(resPt,sch_flex_level,wb);
    break;
  case PHE: 
    write_PHE_sidechain(resPt,sch_flex_level,wb);
    break;
  case PHEH: 
    write_PHEH_sidechain(resPt,sch_flex_level,wb);
    break;
  case PRO: 
  case PROH: 
    // no side-chain in these cases 
    break;
  case SER: 
    write_SER_sidechain(resPt,sch_flex_level,wb);
    break;
  case SERH: 
    write_SERH_sidechain(resPt,sch_flex_level,wb);
    break;
  case THR: 
    write_THR_sidechain(resPt,sch_flex_level,wb);
    break;
  case THRH: 
    write_THRH_sidechain(resPt,sch_flex_level,wb);
    break;
  case TRP: 
    write_TRP_sidechain(resPt,sch_flex_level,wb);
    break;
  case TRPH: 
    write_TRPH_sidechain(resPt,sch_flex_level,wb);
    break;
  case TYR: 
    write_TYR_sidechain(resPt,sch_flex_level,wb);
    break;
  case TYRH: 
    write_TYRH_sidechain(resPt,sch_flex_level,wb);
    break;
  case VAL: 
    write_VAL_sidechain(resPt,sch_flex_level,wb);
    break;
  case VALH: 
    write_VALH_sidechain(resPt,sch_flex_level,wb);
    break;
  }
}


/**********************************************************************/

static void write_body_with_all_atoms(residue *resPt)
{
  // write body head
  fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","all-atoms",
	  resPt->resName,resPt->resSeq,resPt->chainID);

  // NOTE : terminal atoms are not considered yet !!!

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    write_atom(resPt->bkbAlist[GLY_N]);
    write_atom(resPt->bkbAlist[GLY_CA]);
    write_atom(resPt->bkbAlist[GLY_C]);
    write_atom(resPt->bkbAlist[GLY_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[GLY_OXT]);
    break;
  case GLYH: 
    write_atom(resPt->bkbAlist[GLYH_N]);
    write_atom(resPt->bkbAlist[GLYH_CA]);
    write_atom(resPt->bkbAlist[GLYH_1HA]);
    write_atom(resPt->bkbAlist[GLYH_2HA]);
    write_atom(resPt->bkbAlist[GLYH_C]);
    write_atom(resPt->bkbAlist[GLYH_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[GLYH_OXT]);
    // H' atoms
/*     if (resPt->bkbAlist[GLYH_H]) {write_atom(resPt->bkbAlist[GLYH_H]);} */
/*     if (resPt->bkbAlist[GLYH_H_Nterm]) {write_atom(resPt->bkbAlist[GLYH_H_Nterm]);} */
    break;
  case PRO: 
    write_atom(resPt->bkbAlist[PRO_N]);
    write_atom(resPt->bkbAlist[PRO_CA]);
    write_atom(resPt->bkbAlist[PRO_CB]);
    write_atom(resPt->bkbAlist[PRO_CG]);
    write_atom(resPt->bkbAlist[PRO_CD]);
    write_atom(resPt->bkbAlist[PRO_C]);
    write_atom(resPt->bkbAlist[PRO_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[PRO_OXT]);
    break;
  case PROH: 
    write_atom(resPt->bkbAlist[PROH_N]);
    write_atom(resPt->bkbAlist[PROH_CA]);
    write_atom(resPt->bkbAlist[PROH_HA]);
    write_atom(resPt->bkbAlist[PROH_CB]);
    write_atom(resPt->bkbAlist[PROH_1HB]);
    write_atom(resPt->bkbAlist[PROH_2HB]);
    write_atom(resPt->bkbAlist[PROH_CG]);
    write_atom(resPt->bkbAlist[PROH_1HG]);
    write_atom(resPt->bkbAlist[PROH_2HG]);
    write_atom(resPt->bkbAlist[PROH_CD]);
    write_atom(resPt->bkbAlist[PROH_1HD]);
    write_atom(resPt->bkbAlist[PROH_2HD]);
    write_atom(resPt->bkbAlist[PROH_C]);
    write_atom(resPt->bkbAlist[PROH_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[PROH_OXT]);
    break;
  default:
    if(resPt->flagH == 0) {
      write_atom(resPt->bkbAlist[gen_N]);
      write_atom(resPt->bkbAlist[gen_CA]);
      write_atom(resPt->bkbAlist[gen_CB]);
      write_sidechain(resPt,RIGID,0);   // 0 -> rigid side-chain
      write_atom(resPt->bkbAlist[gen_C]);
      write_atom(resPt->bkbAlist[gen_O]);
      if (resPt->flagCterm) write_atom(resPt->bkbAlist[gen_OXT]);
    }
    else {
      write_atom(resPt->bkbAlist[genH_N]);
      write_atom(resPt->bkbAlist[genH_CA]);
      write_atom(resPt->bkbAlist[genH_HA]);
      write_atom(resPt->bkbAlist[genH_CB]);
      write_sidechain(resPt,RIGID,0);   // 0 -> rigid side-chain
      write_atom(resPt->bkbAlist[genH_C]);
      write_atom(resPt->bkbAlist[genH_O]);
      if (resPt->flagCterm) write_atom(resPt->bkbAlist[genH_OXT]);
      // H' atoms
/*       if (resPt->bkbAlist[genH_H]!=NULL) {write_atom(resPt->bkbAlist[genH_H]);} */
/*       if (resPt->bkbAlist[genH_H_Nterm]!=NULL) {write_atom(resPt->bkbAlist[genH_H_Nterm]);} */
    }
  }

  fprintf(p3dfile,"p3d_end_desc\n\n");   
}

static void write_body_with_first_bkb_rigid(residue *resPt)
{
  // write body head
  fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","first-bkb-rigid",
	  resPt->resName,resPt->resSeq,resPt->chainID);

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    write_atom(resPt->bkbAlist[GLY_N]);
    break;
  case GLYH: 
    write_atom(resPt->bkbAlist[GLYH_N]);
    // H' atoms
/*     if (resPt->bkbAlist[GLYH_H]!=NULL) {write_atom(resPt->bkbAlist[GLYH_H]);} */
/*     if (resPt->bkbAlist[GLYH_H_Nterm]!=NULL) {write_atom(resPt->bkbAlist[GLYH_H_Nterm]);} */
    break;
  case PRO: 
    write_atom(resPt->bkbAlist[PRO_N]);
    break;
  case PROH: 
    write_atom(resPt->bkbAlist[PROH_N]);
    break;
  default:
    if(resPt->flagH == 0) {
      write_atom(resPt->bkbAlist[gen_N]);
    }
    else {
      write_atom(resPt->bkbAlist[genH_N]);
      // H' atoms
/*       if (resPt->bkbAlist[genH_H]) {write_atom(resPt->bkbAlist[genH_H]);} */
/*       if (resPt->bkbAlist[genH_H_Nterm]) {write_atom(resPt->bkbAlist[genH_H_Nterm]);} */
    }
  }

  fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_body_with_second_bkb_rigid(residue *resPt)
{
  // WARNING : for PROLINE : if joint phi is not writen then
  //           there is no second rigid !!!

  // write body head
  fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","second-bkb-rigid",
	  resPt->resName,resPt->resSeq,resPt->chainID);

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    write_atom(resPt->bkbAlist[GLY_CA]);
    break;
  case GLYH: 
    write_atom(resPt->bkbAlist[GLYH_CA]);
    write_atom(resPt->bkbAlist[GLYH_1HA]);
    write_atom(resPt->bkbAlist[GLYH_2HA]);
    break;
  case PRO: 
    write_atom(resPt->bkbAlist[PRO_CA]);
    write_atom(resPt->bkbAlist[PRO_CB]);
    write_atom(resPt->bkbAlist[PRO_CG]);
    write_atom(resPt->bkbAlist[PRO_CD]);
    break;
  case PROH: 
    write_atom(resPt->bkbAlist[PROH_CA]);
    write_atom(resPt->bkbAlist[PROH_HA]);
    write_atom(resPt->bkbAlist[PROH_CB]);
    write_atom(resPt->bkbAlist[PROH_1HB]);
    write_atom(resPt->bkbAlist[PROH_2HB]);
    write_atom(resPt->bkbAlist[PROH_CG]);
    write_atom(resPt->bkbAlist[PROH_1HG]);
    write_atom(resPt->bkbAlist[PROH_2HG]);
    write_atom(resPt->bkbAlist[PROH_CD]);
    write_atom(resPt->bkbAlist[PROH_1HD]);
    write_atom(resPt->bkbAlist[PROH_2HD]);
    break;
  default:
    if(resPt->flagH == 0) {
      write_atom(resPt->bkbAlist[gen_CA]);
      write_atom(resPt->bkbAlist[gen_CB]);
    }
    else {
      write_atom(resPt->bkbAlist[genH_CA]);
      write_atom(resPt->bkbAlist[genH_HA]);
      write_atom(resPt->bkbAlist[genH_CB]);
    }
  }

  fprintf(p3dfile,"p3d_end_desc\n\n");   
}


static void write_body_with_third_bkb_rigid(residue *resPt)
{
  // write body head
  fprintf(p3dfile,"p3d_beg_desc P3D_BODY %s.%s.%d.%s\n","third-bkb-rigid",
	  resPt->resName,resPt->resSeq,resPt->chainID);

  // write atoms
  switch(resPt->resType) {
  case GLY: 
    write_atom(resPt->bkbAlist[GLY_C]);
    write_atom(resPt->bkbAlist[GLY_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[GLY_OXT]);
    break;
  case GLYH: 
    write_atom(resPt->bkbAlist[GLYH_C]);
    write_atom(resPt->bkbAlist[GLYH_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[GLYH_OXT]);
    break;
  case PRO: 
    write_atom(resPt->bkbAlist[PRO_C]);
    write_atom(resPt->bkbAlist[PRO_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[PRO_OXT]);
    break;
  case PROH: 
    write_atom(resPt->bkbAlist[PROH_C]);
    write_atom(resPt->bkbAlist[PROH_O]);
    if (resPt->flagCterm) write_atom(resPt->bkbAlist[PROH_OXT]);
    break;
  default:
    if(resPt->flagH == 0) {
      write_atom(resPt->bkbAlist[gen_C]);
      write_atom(resPt->bkbAlist[gen_O]);
      if (resPt->flagCterm) write_atom(resPt->bkbAlist[gen_OXT]);
    }
    else {
      write_atom(resPt->bkbAlist[genH_C]);
      write_atom(resPt->bkbAlist[genH_O]);
      if (resPt->flagCterm) write_atom(resPt->bkbAlist[genH_OXT]);
    }
  }

  fprintf(p3dfile,"p3d_end_desc\n\n");   
}


/**********************************************************************/

static int write_residue(residue *resPt, int bkb_flex_level, int sch_flex_level, int indrefbkb) 
{

  if(resPt->prev != NULL) {
    // next function modifies bkb_ref_indJ
    write_omega_jnt(resPt,indrefbkb,bkb_flex_level);
  }
  else {
    bkb_ref_indJ = indrefbkb;
  }

  if((bkb_flex_level == RIGID)&&(sch_flex_level == RIGID)) {
    // all the atoms form one onlty body if everithing is rigid
    write_body_with_all_atoms(resPt);
  }
  else {
    // NOTE : I have modified Move3D (function p3d_jnt_set_object) 
    //        for accepting the definition of several bodies without  
    //        joints between them

    // NOTE : BCD requires particular order in the .p3d file
    //        thus, when bkb_flex_level == RIGID (even if sch_flex_level == RIGID)
    //        the joint psi (fixed or not) is necessary

    write_body_with_first_bkb_rigid(resPt);
    if(bkb_flex_level != RIGID) {
      write_phi_jnt(resPt,indrefbkb,bkb_flex_level);
    }
    write_body_with_second_bkb_rigid(resPt);
    write_sidechain(resPt,sch_flex_level,1);
    write_psi_jnt(resPt,indrefbkb,bkb_flex_level);
    write_body_with_third_bkb_rigid(resPt);
  }
  return 1; 
}


/**********************************************************************/

static void compute_subchain_refF(residue *start_resPt, residue *end_resPt, matrix4 F)
{  
  residue *resPt=NULL,*next_resPt=NULL,*midresPt=NULL;
  int i, n_res;
  double *a_pos[4];
  double cp[3],bp[3],ep[3],axis[3],zaxis[3],xaxis[3],yaxis[3];

  n_res = end_resPt->resSeq - start_resPt->resSeq + 1;

  midresPt = start_resPt;
  for(i=0; i < ((int) floor((double)n_res/2.0)); i++) {
    midresPt = midresPt->next;
  }
  
  // compute centroid of CA pos
  cp[0] = 0.0; cp[1] = 0.0; cp[2] = 0.0; 
  next_resPt = start_resPt;
  do{
    resPt = next_resPt;
    get_CA_pos(resPt,&(a_pos[0]));  
    vectAdd(a_pos[0],cp,cp);
    next_resPt = resPt->next;
  }while (resPt!=end_resPt);
  vectScale(cp,cp,(1.0/(double)n_res));
  
  // compute z-axis
  if(n_res < 8) {
    // NOTE : for small segments, the z-axis is simply computed from the first and last residues
    get_CA_pos(end_resPt,&(a_pos[0]));
    get_CA_pos(start_resPt,&(a_pos[1]));
    vectSub(a_pos[0],a_pos[1],axis);
    vectNormalize(axis,zaxis);
  }
  else {
    // NOTE : for long segments, the z-axis is computed from the CA positions of the 8 middle residues
    //        this procedure is particularly suited to helice

    resPt = midresPt->next;
    for(i=0; i<4;  i++) {
      get_CA_pos(resPt,&(a_pos[i]));
      resPt = resPt->next;
    }
    
    vectCopy(a_pos[0],bp);
    for(i = 1 ; i < 4 ; i++)
      vectAdd(a_pos[i],bp,bp);
    vectScale(bp,bp,1.0/4.0);
  
    resPt = midresPt;
    for(i=0; i<4;  i++) {
      get_CA_pos(resPt,&(a_pos[i]));
      resPt = resPt->prev;
    }
    
    vectCopy(a_pos[0],ep);
    for(i = 1 ; i < 4 ; i++)
      vectAdd(a_pos[i],ep,ep);
    vectScale(ep,ep,1.0/4.0);
    
    vectSub(ep,bp,axis);
    vectNormalize(axis,zaxis);
  }

  // compute x-axis
  // process :
  // 1- compute 'axis' from origin middle CA atoms
  // 2- vectorial product of zaxis and 'axis' = yaxis
  // 3- vectorial product of yaxis and zaxis  = xaxis
  vectSub(a_pos[0],cp,axis);
  vectNormalize(axis,axis);
  normalized_vectXprod(zaxis,axis,yaxis);  
  normalized_vectXprod(yaxis,zaxis,xaxis);  

  // set refF
  compute_frame(cp,xaxis,zaxis,F);

}

/**********************************************************************/

static int write_peptide_chain(protein* protPt, 
			       int index_prot_subchain, int index_subchain, 
			       int start_resSeq, int end_resSeq,
			       int_list* bkb_flex_level_list, int_list* sch_flex_level_list)
{
  int bkb_flex;
  int sch_flex;
  int current_resSeq;
  residue* start_resPt = NULL;
  residue* end_resPt = NULL;
  residue* current_resPt = NULL;
  residue* next_resPt = NULL;
  int indref;
  int numprot;
  double *Npos,*CApos,*Cpos;
  double posdiff[3],prev_zaxis[3];
  //double xaxis[3],zaxis[3];
  matrix4 F;
  //double FFdofs[6];
  int i;

  // connectivity test
  if(!protein_get_res(protPt, index_prot_subchain, start_resSeq, &start_resPt)){ 
    printf("ERROR : starting sequence number %d is not defined in protein %s\n",
	   start_resSeq,protPt->name);  
  }
  else if(!protein_get_res(protPt, index_prot_subchain, end_resSeq, &end_resPt)){ 
    printf("ERROR : ending sequence number %d is not defined in protein %s\n",
	   end_resSeq,protPt->name);  
  }
  else if(!util_protein_is_segment(protPt, index_prot_subchain, start_resSeq,end_resSeq)){ 
    printf("ERROR : subchain bounding residues %d and %d are not in a same segment of %s\n",
	   start_resSeq,end_resSeq,protPt->name);
  }

  // compute the base-frame 
  get_N_pos(start_resPt,&Npos);
  get_CA_pos(start_resPt,&CApos);
  if((index_subchain > 0) && (start_resPt->prev != NULL)) {
    get_C_pos(start_resPt->prev,&Cpos);
    vectSub(Npos,Cpos,posdiff);
    vectNormalize(posdiff,prev_zaxis);
  }
  else {
    prev_zaxis[0] = 0.0; prev_zaxis[1] = 0.0; prev_zaxis[2] = 1.0;
  }
  // OLD //////////////////////////////////////// 
  /*   vectSub(CApos,Npos,posdiff); */
  /*   vectNormalize(posdiff,zaxis); */
  /*   normalized_vectXprod(prev_zaxis,zaxis,xaxis); */
  /*   compute_frame(CApos, xaxis, zaxis, F); */
  
  // extract FFjnt parameters */
  /*   mat4ExtractPosReverseOrder(F,&(FFdofs[0]),&(FFdofs[1]),&(FFdofs[2]),&(FFdofs[3]),&(FFdofs[4]),&(FFdofs[5])); */
  ///////////////////////////////////////////////
    
  // NEW //////////////////////////////////////// 
  compute_subchain_refF(start_resPt,end_resPt,F);  
  ///////////////////////////////////////////////


  // fictive joint at the end of previous subchain defining the position of the next N 
  if(index_subchain > 0) {
    fprintf(p3dfile,"# FICTIVE JOINT : necessary for loops \n");
    fprintf(p3dfile,"p3d_beg_desc_jnt P3D_ROTATE   # J%d\n",++global_indJ);
    fprintf(p3dfile,"  p3d_set_name end_chain.%d.%d\n",numprot,index_subchain-1);
    fprintf(p3dfile,"  p3d_set_pos_axe %f %f %f %f %f %f\n",
	    Npos[0],Npos[1],Npos[2],prev_zaxis[0],prev_zaxis[1],prev_zaxis[2]);
    fprintf(p3dfile,"  p3d_set_prev_jnt %d\n",bkb_ref_indJ);
    fprintf(p3dfile,"  p3d_set_is_user 0\n");
    fprintf(p3dfile,"  p3d_set_dof %f\n",0.0);
    fprintf(p3dfile,"  p3d_set_dof_pos0 %f\n",0.0);
    fprintf(p3dfile,"  p3d_set_dof_vmin %f\n",0.0);
    fprintf(p3dfile,"  p3d_set_dof_vmax %f\n",0.0);
    fprintf(p3dfile,"p3d_end_desc\n\n");   

    first_omega_after_FFsubchain = 1;
  }

  //fprintf(p3dfile,"## CHAIN : %s -- subchain : %d\n\n",start_resPt->chainID,start_resPt->subchainind);
  fprintf(p3dfile,"## CHAIN : %s -- subchain : %d\n\n",start_resPt->chainID,index_subchain);

  // NOTE : The first joint frame is relative to the protein base-joint
  //        This joint is a free-flying joint.

  indref = 1; // the protein base-joint is jnt[1] 
  numprot = 1; // WARNING : ONLY ONE PROTEIN NOW !!!

  fprintf(p3dfile,"p3d_beg_desc_jnt P3D_FREEFLYER    # %s # J%d\n","(SUB)CHAIN BASE-JOINT",++global_indJ);
  //fprintf(p3dfile,"  p3d_set_name .chain_base.%d.%s.%d\n",numprot,start_resPt->chainID,start_resPt->subchainind);
  if((index_subchain > 0) && (start_resPt->prev != NULL)) 
    fprintf(p3dfile,"  p3d_set_name chain_base.%d.%d\n",numprot,index_subchain);
  else    
    fprintf(p3dfile,"  p3d_set_name .chain_base.%d.%d\n",numprot,index_subchain);
/*   fprintf(p3dfile,"  p3d_set_pos_axe %f %f %f %f %f %f\n",0.0,0.0,0.0,0.0,0.0,0.0); */ 
  fprintf(p3dfile,"  p3d_set_pos_axe %f %f %f %f %f %f\n",F[0][3],F[1][3],F[2][3],F[0][2],F[1][2],F[2][2]);
  fprintf(p3dfile,"  p3d_set_prev_jnt %d\n",indref);
  fprintf(p3dfile,"  p3d_set_is_user 0 0 0 0 0 0\n");
/*   fprintf(p3dfile,"  p3d_set_dof %f %f %f %f %f %f\n", */
/* 	  FFdofs[0],FFdofs[1],FFdofs[2],FFdofs[3]*(180/PI),FFdofs[4]*(180/PI),FFdofs[5]*(180/PI)); */
/*   fprintf(p3dfile,"  p3d_set_dof_pos0 %f %f %f %f %f %f\n", */
/* 	  FFdofs[0],FFdofs[1],FFdofs[2],FFdofs[3]*(180/PI),FFdofs[4]*(180/PI),FFdofs[5]*(180/PI)); */
  fprintf(p3dfile,"  p3d_set_dof %f %f %f %f %f %f\n",
	  0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  fprintf(p3dfile,"  p3d_set_dof_pos0 %f %f %f %f %f %f\n",
	  0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  fprintf(p3dfile,"  p3d_set_dof_vmin -50 -50 -50 -180 -180 -180\n");
  fprintf(p3dfile,"  p3d_set_dof_vmax  50  50  50  180  180  180\n");
  fprintf(p3dfile,"p3d_end_desc\n\n");   

  indref = global_indJ;
  // update bkb_ref_indJ
  bkb_ref_indJ = global_indJ;  

  next_resPt = start_resPt;
  // write each residue
  i = 0;
  do{ 
    current_resPt = next_resPt;
    current_resSeq = current_resPt->resSeq;
    
    bkb_flex = int_list_get_value(bkb_flex_level_list,i);
    
    sch_flex = int_list_get_value(sch_flex_level_list,i);
    
    if(write_residue(current_resPt,bkb_flex,sch_flex,indref) < 0) {
      return -1;
    }
    indref = bkb_ref_indJ;  // bkb_ref_indJ is modofied in function write_residue

    res_get_nextRes(current_resPt,&next_resPt); 
    
    i++;

  }while (current_resPt!=end_resPt && next_resPt!=NULL);
  
  return 1;
}

/**********************************************************************/

static int write_p3d_file(protein *protPt, subCh_aaa_list *subCh_specPt) 
{
  int i, index_prot_subchain;  

  // write .p3d head
  write_p3d_head(protPt);

  // write protein head
  write_protein_head(protPt);

  // write each chain 
  // WARNING : all the subchains must be defined in the aaa file !!! 
  index_prot_subchain = 0;
  for(i=0;i<subCh_specPt->nsubCh_spec;i++){
    if(i>0) {
      if(subCh_specPt->subCh_spec_list[i]->start_resSeq != (subCh_specPt->subCh_spec_list[i-1]->end_resSeq + 1))
	index_prot_subchain++;
    }
    if(write_peptide_chain(protPt,index_prot_subchain,i,
			   subCh_specPt->subCh_spec_list[i]->start_resSeq,
			   subCh_specPt->subCh_spec_list[i]->end_resSeq,
			   &(subCh_specPt->subCh_spec_list[i]->bkb_flex_level),
			   &(subCh_specPt->subCh_spec_list[i]->sch_flex_level)) < 0)

      return -1;
  }
  
  // write .p3d end
  write_p3d_end(protPt);

  return 1;
}

/**********************************************************************/
/**********************************************************************/
// MAIN
/**********************************************************************/
/**********************************************************************/

int main(int argc, char **argv)
{
  protein *protPt;
  subCh_aaa_list subCh_spec;            

  if(read_call_arguments(argc, argv) < 0) {
    printf("Usage: pdbtop3d <pdbfile> <aaafile> <p3dfile>\n");
    return -1;
  }

  // open files
  pdbfile = fopen(pdbfilename, "r");
  if (pdbfile == NULL) {
    printf("pdb file cannot be open\n");
    return -1;
  }
  aaafile = fopen(aaafilename, "r");
  if (aaafile == NULL) {
    printf("aaa file cannot be open\n");
    fclose(pdbfile);
    return -1;
  }
  p3dfile = fopen(p3dfilename, "w");
  if (p3dfile == NULL) {
    printf("p3d file cannot be open\n");
    fclose(pdbfile);
    fclose(aaafile);
    return -1;
  }

  // alloc protein
  protPt = (protein *) malloc(sizeof(protein));

  // read PDB and write protein data structure
  if(fill_protein_struct(pdbfile,pdbfilename,protPt) < 0) {
    printf("ERROR while writing protein data structure from PDB\n");
    free_protein(protPt);  
    fclose(pdbfile);
    fclose(aaafile);
    fclose(p3dfile);
    return -1;
  }
  printf("PSF initialization OK  ...\n");

  // alloc aaa_struct
  subCh_aaa_list_init(&subCh_spec);

  // read AAA and write mech.spec.
  if ( !read_AA_from_file(aaafilename,&subCh_spec) ) {
    free_protein(protPt);  
    fclose(pdbfile);
    fclose(aaafile);
    fclose(p3dfile);
    return -1;
  }
  printf("AAA description loaded ...\n");

  // write .p3d file from protein data structure and .aaa file
  if(write_p3d_file(protPt,&subCh_spec) < 0) {
    printf("ERROR while writing .p3d file from protein data structure and .aaa file\n");
    free_protein(protPt);  
    fclose(pdbfile);
    fclose(aaafile);
    fclose(p3dfile);
    return -1;    
  }
  printf("P3D file written ...\n");
 
  free_protein(protPt);  
  fclose(pdbfile);
  fclose(aaafile);
  fclose(p3dfile);  
  return 1;
}
