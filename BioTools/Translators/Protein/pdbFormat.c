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
// INCLUDES

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pdbFormat.h"

#define TRUE 1
#define FALSE 0

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
/**********************************************************************/
// TRANSLATION //

void translate_pdb_res_name(char* pdb_res_name, char* psf_res_name, formatTypes pdb_format) {

  strncpy(psf_res_name, pdb_res_name, 3);
 
  switch (pdb_format) {
  case AMBER :
    if ((strcmp(pdb_res_name,"HIE")==0)
	|| (strcmp(pdb_res_name,"HIP")==0)
	|| (strcmp(pdb_res_name,"HID")==0)) {
      strncpy(psf_res_name,"HIS",3);
    }
    else if (strcmp(pdb_res_name, "LYN")==0) {
      strncpy(psf_res_name,"LYS",3);
    }
    else if ((strcmp(pdb_res_name, "CYM")==0)
	|| (strcmp(pdb_res_name, "CYX")==0)) {
      strncpy(psf_res_name,"CYS",3);
    }
    else if (strcmp(pdb_res_name, "ASH")==0) {
      strncpy(psf_res_name,"ASP",3);
    }
    else if (strcmp(pdb_res_name, "GLH")==0) {
      strncpy(psf_res_name,"GLU",3);
    }
    break;
  case INSIGHT:
    break;
  }  

}

/**********************************************************************/

void translate_pdb_atom_name(char* pdb_atom_name, char* psf_atom_name,
			     residueTypes resType, formatTypes pdb_format) {
  
  
  strncpy(psf_atom_name, pdb_atom_name, 4);
 
  switch (pdb_format) {
  case AMBER:
    switch(resType) {
    case ALAH:
    case VALH:
      break;
    case METH:
      if(strcmp(pdb_atom_name,"2HB") == 0) {strncpy(psf_atom_name,"1HB",4);}
      else if(strcmp(pdb_atom_name,"3HB") == 0) {strncpy(psf_atom_name,"2HB",4);}
      else if(strcmp(pdb_atom_name,"2HG") == 0) {strncpy(psf_atom_name,"1HG",4);}
      else if(strcmp(pdb_atom_name,"3HG") == 0) {strncpy(psf_atom_name,"2HG",4);}
      break;
    default:
      if(strcmp(pdb_atom_name,"2HA") == 0) {strncpy(psf_atom_name,"1HA",4);}
      else if(strcmp(pdb_atom_name,"3HA") == 0) {strncpy(psf_atom_name,"2HA",4);}
      else if(strcmp(pdb_atom_name,"2HB") == 0) {strncpy(psf_atom_name,"1HB",4);}
      else if(strcmp(pdb_atom_name,"3HB") == 0) {strncpy(psf_atom_name,"2HB",4);}
      else if(strcmp(pdb_atom_name,"2HD") == 0) {strncpy(psf_atom_name,"1HD",4);}
      else if(strcmp(pdb_atom_name,"3HD") == 0) {strncpy(psf_atom_name,"2HD",4);}
      else if(strcmp(pdb_atom_name,"2HG") == 0) {strncpy(psf_atom_name,"1HG",4);}
      else if(strcmp(pdb_atom_name,"3HG") == 0) {strncpy(psf_atom_name,"2HG",4);}
      else if(strcmp(pdb_atom_name,"2HE") == 0) {strncpy(psf_atom_name,"1HE",4);}
      else if(strcmp(pdb_atom_name,"3HE") == 0) {strncpy(psf_atom_name,"2HE",4);}
      else if(strcmp(pdb_atom_name,"2HG1") == 0) {strncpy(psf_atom_name,"1HG1",4);}
      else if(strcmp(pdb_atom_name,"3HG1") == 0) {strncpy(psf_atom_name,"2HG1",4);}
    }
    break;
  case INSIGHT:
    break;
  }
}

/**********************************************************************/
/**********************************************************************/
// UPDATE AMBER SERIAL //

static int check_amber_bkb_atoms(residue* resPt){

  int n_amb_bkb;
  int n_noOXT_bkb;

  if (!resPt->flagH)
    return FALSE;

  if (!resPt->flagCterm) {
    n_noOXT_bkb = resPt->nbkbatoms-1;
  }
  else {
    n_noOXT_bkb = resPt->nbkbatoms;
  }
  
  switch(resPt->resType) {
  case GLYH:
    n_amb_bkb = AMB_N_GLYH_BKB_ATOMS;
    break;
  case PROH:
    n_amb_bkb = AMB_N_PROH_BKB_ATOMS;
    break;
  default:
    n_amb_bkb = AMB_N_GENH_BKB_ATOMS;
  }

  if ((n_noOXT_bkb>n_amb_bkb && !resPt->flagCterm)
      || (n_noOXT_bkb>n_amb_bkb+1 && resPt->flagCterm)) {
    printf("WARNING (res %d): some backbone atoms will not be translated in AMBER standard\n", resPt->resSeq);
    return TRUE;
  }
  else if (n_noOXT_bkb<n_amb_bkb) {
    printf("ERROR (res %d): missing backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else 
    return TRUE;
}

static int check_amber_sch_atoms(residue* resPt){

  int n_amb_sch;

  if (!resPt->flagH)
    return FALSE;

  switch(resPt->resType) {
  case ALAH:
    n_amb_sch = AMB_N_ALAH_SCH_ATOMS; break;
  case ARGH:
    n_amb_sch = AMB_N_ARGH_SCH_ATOMS; break;
  case ASNH:
    n_amb_sch = AMB_N_ASNH_SCH_ATOMS; break;
  case ASPH:
    n_amb_sch = AMB_N_ASPH_SCH_ATOMS; break;
  case CYSH:
    n_amb_sch = AMB_N_CYSH_SCH_ATOMS; break;
   case GLNH:
    n_amb_sch = AMB_N_GLNH_SCH_ATOMS; break;
   case GLUH:
    n_amb_sch = AMB_N_GLUH_SCH_ATOMS; break;
   case GLYH:
    n_amb_sch = AMB_N_GLYH_SCH_ATOMS; break;
  case HISH:
    n_amb_sch = AMB_N_HIEH_SCH_ATOMS; break;
  case ILEH:
    n_amb_sch = AMB_N_ILEH_SCH_ATOMS; break;
  case LEUH:
    n_amb_sch = AMB_N_LEUH_SCH_ATOMS; break;
  case LYSH:
    n_amb_sch = AMB_N_LYSH_SCH_ATOMS; break;
  case METH:
    n_amb_sch = AMB_N_METH_SCH_ATOMS; break;
  case PHEH:
    n_amb_sch = AMB_N_PHEH_SCH_ATOMS; break;
  case PROH:
    n_amb_sch = AMB_N_PROH_SCH_ATOMS; break;
  case SERH:
    n_amb_sch = AMB_N_SERH_SCH_ATOMS; break;
  case THRH:
    n_amb_sch = AMB_N_THRH_SCH_ATOMS; break;
  case TRPH:
    n_amb_sch = AMB_N_TRPH_SCH_ATOMS; break;
  case TYRH:
    n_amb_sch = AMB_N_TYRH_SCH_ATOMS; break;
  case VALH:
    n_amb_sch = AMB_N_VALH_SCH_ATOMS; break;
  default:
    return FALSE;
  }
  
  if (resPt->nschatoms>n_amb_sch && resPt->resType!=HISH) {
    printf("WARNING (res %d): some side chain atoms will not be translated in AMBER standard\n", resPt->resSeq);
    return TRUE;
  }
  else if (resPt->nschatoms<n_amb_sch) {
    printf("ERROR (res %d): missing side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else
    return TRUE;
  
}

/**********************************************************************/

static void update_Nterm_PROH_amber_serials(residue* resPt, int first_amber_ind, int offset) {

  int i;
  int H_Nterm_offset = 0;
  atom* aPt = NULL;

  for (i=0; i<resPt->nbkbatoms; i++) {
    aPt = resPt->bkbAlist[i];
    if (aPt!=NULL) {
      if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[PROH_N]) {
      	H_Nterm_offset++;
	aPt->amber_index=(first_amber_ind+H_Nterm_offset);
      }
      else if (aPt->amber_index!=AMB_NONE
	       && aPt->amber_index>first_amber_ind){
	aPt->amber_index+=offset;
      }
    }
  }
  
  if (H_Nterm_offset!=offset) {
    printf("WARNING : possible errors while renumbering atoms in amber format\n");
  }

}

/**********************************************************************/

static void update_Nterm_amber_serials(residue* resPt, int first_amber_ind, int offset) {

  int i;
  int H_Nterm_offset = 0;
  atom* aPt = NULL;

  for (i=0; i<resPt->nbkbatoms; i++) {
    aPt = resPt->bkbAlist[i];
    if (aPt!=NULL) {
      if ( aPt->amber_index!=AMB_NONE
	   && aPt->amber_index>first_amber_ind) {
	aPt->amber_index+=offset;
      }
      else if (aPt->amber_index==first_amber_ind) {
	aPt->amber_index+=H_Nterm_offset;
	H_Nterm_offset++;
      }
    }
  }
  
  for (i=0; i<resPt->nschatoms; i++) {
    aPt = resPt->schAlist[i];
    if (aPt!=NULL) {
      if (aPt->amber_index!= AMB_NONE
	  && aPt->amber_index>first_amber_ind) {
	aPt->amber_index+=offset;
      }
      else if (aPt->amber_index==first_amber_ind) {
	aPt->amber_index+=H_Nterm_offset;
	H_Nterm_offset++;
      }
    }
  }
  
  if (H_Nterm_offset-1!=offset) {
    printf("WARNING : possible errors while renumbering atoms in amber format\n");
  }

}

/**********************************************************************/

static void translate_amber_index_ASPH_ASHH(residue* resPt, int firstSer, int* lastSer) {
  
  int nmax_bkb_atoms = resPt->nbkbatoms;
  int nmax_sch_atoms = resPt->nschatoms;
  int index;
  int i;

  for (i=0; i<nmax_bkb_atoms; i++) {
    if (resPt->bkbAlist[i]!=NULL 
	&& resPt->bkbAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->bkbAlist[i]->amber_index;
      if (index==AMB_ASPH_N) resPt->bkbAlist[i]->amber_index=AMB_ASHH_N+firstSer;
      else if (index==AMB_ASPH_H) resPt->bkbAlist[i]->amber_index=AMB_ASHH_H+firstSer;
      else if (index==AMB_ASPH_CA) resPt->bkbAlist[i]->amber_index=AMB_ASHH_CA+firstSer;
      else if (index==AMB_ASPH_HA) resPt->bkbAlist[i]->amber_index=AMB_ASHH_HA+firstSer;
      else if (index==AMB_ASPH_CB) resPt->bkbAlist[i]->amber_index=AMB_ASHH_CB+firstSer;
      else if (index==AMB_ASPH_C) resPt->bkbAlist[i]->amber_index=AMB_ASHH_C+firstSer;
      else if (index==AMB_ASPH_O) resPt->bkbAlist[i]->amber_index=AMB_ASHH_O+firstSer;
      else if (resPt->flagCterm && index==AMB_ASPH_OXT) resPt->bkbAlist[i]->amber_index=AMB_ASHH_OXT+firstSer;
    }
  }

  for (i=0; i<nmax_sch_atoms; i++) {
    if (resPt->schAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->schAlist[i]->amber_index;
      if (index==AMB_ASPH_2HB) resPt->schAlist[i]->amber_index=AMB_ASHH_2HB+firstSer;
      else if (index==AMB_ASPH_3HB) resPt->schAlist[i]->amber_index=AMB_ASHH_3HB+firstSer;
      else if (index==AMB_ASPH_CG) resPt->schAlist[i]->amber_index=AMB_ASHH_CG+firstSer;
      else if (index==AMB_ASPH_OD1) resPt->schAlist[i]->amber_index=AMB_ASHH_OD1+firstSer;
      else if (index==AMB_ASPH_OD2) resPt->schAlist[i]->amber_index=AMB_ASHH_OD2+firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,AMB_ASHH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_ASHH_OXT+firstSer;
  else *lastSer = AMB_ASHH_O+firstSer;
  
  if (resPt->flagNterm)
    (*lastSer++);
}

/**********************************************************************/

static void translate_amber_index_GLUH_GLHH(residue* resPt, int firstSer, int* lastSer) {

  int nmax_bkb_atoms = resPt->nbkbatoms;
  int nmax_sch_atoms = resPt->nschatoms;
  int index;
  int i;

  for (i=0; i<nmax_bkb_atoms; i++) {
    if (resPt->bkbAlist[i]!=NULL 
	&& resPt->bkbAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->bkbAlist[i]->amber_index;
      if (index==AMB_GLUH_N) resPt->bkbAlist[i]->amber_index=AMB_GLHH_N+firstSer;
      else if (index==AMB_GLUH_H) resPt->bkbAlist[i]->amber_index=AMB_GLHH_H+firstSer;
      else if (index==AMB_GLUH_HA) resPt->bkbAlist[i]->amber_index=AMB_GLHH_HA+firstSer;
      else if (index==AMB_GLUH_CB) resPt->bkbAlist[i]->amber_index=AMB_GLHH_CB+firstSer;
      else if (index==AMB_GLUH_C) resPt->bkbAlist[i]->amber_index=AMB_GLHH_C+firstSer;
      else if (index==AMB_GLUH_O) resPt->bkbAlist[i]->amber_index=AMB_GLHH_O+firstSer;
      else if (resPt->flagCterm && index==AMB_GLUH_OXT) resPt->bkbAlist[i]->amber_index=AMB_GLHH_OXT+firstSer;
    }
  }

  for (i=0; i<nmax_sch_atoms; i++) {
    if (resPt->schAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->schAlist[i]->amber_index;
      if (index==AMB_GLUH_2HB) resPt->schAlist[i]->amber_index=AMB_GLHH_2HB+firstSer;
      else if (index==AMB_GLUH_3HB) resPt->schAlist[i]->amber_index=AMB_GLHH_3HB+firstSer;
      else if (index==AMB_GLUH_CG) resPt->schAlist[i]->amber_index=AMB_GLHH_CG+firstSer;
      else if (index==AMB_GLUH_2HG) resPt->schAlist[i]->amber_index=AMB_GLHH_2HG+firstSer;
      else if (index==AMB_GLUH_3HG) resPt->schAlist[i]->amber_index=AMB_GLHH_3HG+firstSer;
      else if (index==AMB_GLUH_CD) resPt->schAlist[i]->amber_index=AMB_GLHH_CD+firstSer;
      else if (index==AMB_GLUH_OE1) resPt->schAlist[i]->amber_index=AMB_GLHH_OE1+firstSer;
      else if (index==AMB_GLUH_OE2) resPt->schAlist[i]->amber_index=AMB_GLHH_OE2+firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,AMB_GLHH_H,2);
  if (resPt->flagCterm)*lastSer = AMB_GLHH_OXT+firstSer;
  else *lastSer = AMB_GLHH_O+firstSer;
  if (resPt->flagNterm)
   (*lastSer++);

}

/**********************************************************************/

static void translate_amber_index_LYSH_LYNH(residue* resPt, int firstSer, int* lastSer) {

  int nmax_bkb_atoms = resPt->nbkbatoms;
  int nmax_sch_atoms = resPt->nschatoms;
  int index;
  int i;

  for (i=0; i<nmax_bkb_atoms; i++) {
    if (resPt->bkbAlist[i]!=NULL 
	&& resPt->bkbAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->bkbAlist[i]->amber_index;
      if (index==AMB_LYSH_N) resPt->bkbAlist[i]->amber_index=AMB_LYNH_N+firstSer;
      else if (index==AMB_LYSH_H) resPt->bkbAlist[i]->amber_index=AMB_LYNH_H+firstSer;
      else if (index==AMB_LYSH_CA) resPt->bkbAlist[i]->amber_index=AMB_LYNH_CA+firstSer;
      else if (index==AMB_LYSH_HA) resPt->bkbAlist[i]->amber_index=AMB_LYNH_HA+firstSer;
      else if (index==AMB_LYSH_CB) resPt->bkbAlist[i]->amber_index=AMB_LYNH_CB+firstSer;
      else if (index==AMB_LYSH_C) resPt->bkbAlist[i]->amber_index=AMB_LYNH_C+firstSer;
      else if (index==AMB_LYSH_O) resPt->bkbAlist[i]->amber_index=AMB_LYNH_O+firstSer;
      else if (resPt->flagCterm && index==AMB_LYSH_OXT) resPt->bkbAlist[i]->amber_index=AMB_LYNH_OXT+firstSer;
    }
  }

  for (i=0; i<nmax_sch_atoms; i++) {
    if (resPt->schAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->schAlist[i]->amber_index;
      if (index==AMB_LYSH_2HB) resPt->schAlist[i]->amber_index=AMB_LYNH_2HB+firstSer;
      else if (index==AMB_LYSH_3HB) resPt->schAlist[i]->amber_index=AMB_LYNH_3HB+firstSer;
      else if (index==AMB_LYSH_CG) resPt->schAlist[i]->amber_index=AMB_LYNH_CG+firstSer;
      else if (index==AMB_LYSH_2HG) resPt->schAlist[i]->amber_index=AMB_LYNH_2HG+firstSer;
      else if (index==AMB_LYSH_3HG) resPt->schAlist[i]->amber_index=AMB_LYNH_3HG+firstSer;
      else if (index==AMB_LYSH_CD) resPt->schAlist[i]->amber_index=AMB_LYNH_CD+firstSer;
      else if (index==AMB_LYSH_2HD) resPt->schAlist[i]->amber_index=AMB_LYNH_2HD+firstSer;
      else if (index==AMB_LYSH_3HD) resPt->schAlist[i]->amber_index=AMB_LYNH_3HD+firstSer;
      else if (index==AMB_LYSH_CE) resPt->schAlist[i]->amber_index=AMB_LYNH_CE+firstSer;
      else if (index==AMB_LYSH_2HE) resPt->schAlist[i]->amber_index=AMB_LYNH_2HE+firstSer;
      else if (index==AMB_LYSH_3HE) resPt->schAlist[i]->amber_index=AMB_LYNH_3HE+firstSer;
      else if (index==AMB_LYSH_NZ) resPt->schAlist[i]->amber_index=AMB_LYNH_NZ+firstSer;
      else if (index==AMB_LYSH_1HZ) resPt->schAlist[i]->amber_index=AMB_LYNH_1HZ+firstSer;
      else if (index==AMB_LYSH_2HZ) resPt->schAlist[i]->amber_index=AMB_LYNH_2HZ+firstSer;
    }

  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,AMB_LYNH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_LYNH_OXT+firstSer;
  else *lastSer = AMB_LYNH_O+firstSer;
  
  if (resPt->flagNterm)
   (*lastSer++);

}

/**********************************************************************/

static void translate_amber_index_HIPH_HIEH(residue* resPt, int firstSer, int* lastSer) {

  int nmax_bkb_atoms = resPt->nbkbatoms;
  int nmax_sch_atoms = resPt->nschatoms;
  int index;
  int i;

  for (i=0; i<nmax_bkb_atoms; i++) {
    if (resPt->bkbAlist[i]!=NULL 
	&& resPt->bkbAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->bkbAlist[i]->amber_index;
      if (index==AMB_HIPH_N) resPt->bkbAlist[i]->amber_index=AMB_HIEH_N+firstSer;
      else if (index==AMB_HIPH_H) resPt->bkbAlist[i]->amber_index=AMB_HIEH_H+firstSer;
      else if (index==AMB_HIPH_CA) resPt->bkbAlist[i]->amber_index=AMB_HIEH_CA+firstSer;
      else if (index==AMB_HIPH_HA) resPt->bkbAlist[i]->amber_index=AMB_HIEH_HA+firstSer;
      else if (index==AMB_HIPH_CB) resPt->bkbAlist[i]->amber_index=AMB_HIEH_CB+firstSer;
      else if (index==AMB_HIPH_C) resPt->bkbAlist[i]->amber_index=AMB_HIEH_C+firstSer;
      else if (index==AMB_HIPH_O) resPt->bkbAlist[i]->amber_index=AMB_HIEH_O+firstSer;
      else if (resPt->flagCterm && index==AMB_HIPH_OXT) resPt->bkbAlist[i]->amber_index=AMB_HIEH_OXT+firstSer;
    }
  }

  for (i=0; i<nmax_sch_atoms; i++) {
    if (resPt->schAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->schAlist[i]->amber_index;
      if (index==AMB_HIPH_2HB) resPt->schAlist[i]->amber_index=AMB_HIEH_2HB+firstSer;
      else if (index==AMB_HIPH_3HB) resPt->schAlist[i]->amber_index=AMB_HIEH_3HB+firstSer;
      else if (index==AMB_HIPH_CG)  resPt->schAlist[i]->amber_index=AMB_HIEH_CG+firstSer;
      else if (index==AMB_HIPH_ND1) resPt->schAlist[i]->amber_index=AMB_HIEH_ND1+firstSer;
      else if (index==AMB_HIPH_CE1) resPt->schAlist[i]->amber_index=AMB_HIEH_CE1+firstSer;
      else if (index==AMB_HIPH_HE1) resPt->schAlist[i]->amber_index=AMB_HIEH_HE1+firstSer;
      else if (index==AMB_HIPH_NE2) resPt->schAlist[i]->amber_index=AMB_HIEH_NE2+firstSer;
      else if (index==AMB_HIPH_CD2) resPt->schAlist[i]->amber_index=AMB_HIEH_CD2+firstSer;
      else if (index==AMB_HIPH_HD2) resPt->schAlist[i]->amber_index=AMB_HIEH_HD2+firstSer;
      else if (index==AMB_HIPH_HE2) resPt->schAlist[i]->amber_index=AMB_HIEH_HE2+firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,AMB_HIEH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_HIEH_OXT+firstSer;
  else *lastSer = AMB_HIEH_O+firstSer;
  
  if (resPt->flagNterm)
   (*lastSer++);
}

/**********************************************************************/

static void translate_amber_index_HIPH_HIDH(residue* resPt, int firstSer, int* lastSer) {

  int nmax_bkb_atoms = resPt->nbkbatoms;
  int nmax_sch_atoms = resPt->nschatoms;
  int index;
  int i;

  for (i=0; i<nmax_bkb_atoms; i++) {
    if (resPt->bkbAlist[i]!=NULL 
	&& resPt->bkbAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->bkbAlist[i]->amber_index;
      if (index==AMB_HIPH_N) resPt->bkbAlist[i]->amber_index=AMB_HIDH_N+firstSer;
      else if (index==AMB_HIPH_H) resPt->bkbAlist[i]->amber_index=AMB_HIDH_H+firstSer;
      else if (index==AMB_HIPH_CA) resPt->bkbAlist[i]->amber_index=AMB_HIDH_CA+firstSer;
      else if (index==AMB_HIPH_HA) resPt->bkbAlist[i]->amber_index=AMB_HIDH_HA+firstSer;
      else if (index==AMB_HIPH_CB) resPt->bkbAlist[i]->amber_index=AMB_HIDH_CB+firstSer;
      else if (index==AMB_HIPH_C) resPt->bkbAlist[i]->amber_index=AMB_HIDH_C+firstSer;
      else if (index==AMB_HIPH_O) resPt->bkbAlist[i]->amber_index=AMB_HIDH_O+firstSer;
      else if (index==AMB_HIPH_OXT) resPt->bkbAlist[i]->amber_index=AMB_HIDH_OXT+firstSer;
    }
  }

  for (i=0; i<nmax_sch_atoms; i++) {
    if (resPt->schAlist[i]->amber_index!=AMB_NONE) {
      index = resPt->schAlist[i]->amber_index;
      if (index==AMB_HIPH_2HB) resPt->schAlist[i]->amber_index=AMB_HIDH_2HB+firstSer;
      else if (index==AMB_HIPH_3HB) resPt->schAlist[i]->amber_index=AMB_HIDH_3HB+firstSer;
      else if (index==AMB_HIPH_CG) resPt->schAlist[i]->amber_index=AMB_HIDH_CG+firstSer;
      else if (index==AMB_HIPH_ND1) resPt->schAlist[i]->amber_index=AMB_HIDH_ND1+firstSer;
      else if (index==AMB_HIPH_CE1) resPt->schAlist[i]->amber_index=AMB_HIDH_CE1+firstSer;
      else if (index==AMB_HIPH_HE1) resPt->schAlist[i]->amber_index=AMB_HIDH_HE1+firstSer;
      else if (index==AMB_HIPH_NE2) resPt->schAlist[i]->amber_index=AMB_HIDH_NE2+firstSer;
      else if (index==AMB_HIPH_CD2) resPt->schAlist[i]->amber_index=AMB_HIDH_CD2+firstSer;
      else if (index==AMB_HIPH_HD1) resPt->schAlist[i]->amber_index=AMB_HIDH_HD1+firstSer;
      else if (index==AMB_HIPH_HD2) resPt->schAlist[i]->amber_index=AMB_HIDH_HD2+firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,AMB_HIDH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_HIDH_OXT+firstSer;
  else *lastSer = AMB_HIDH_O+firstSer;
  
  if (resPt->flagNterm)
   (*lastSer++);

}

/**********************************************************************/

static int update_amber_serial_PROH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;

  resPt->bkbAlist[PROH_N]->amber_index=AMB_PROH_N; 
  resPt->bkbAlist[PROH_CD]->amber_index=AMB_PROH_CD; 
  resPt->bkbAlist[PROH_1HD]->amber_index=AMB_PROH_2HD; 
  resPt->bkbAlist[PROH_2HD]->amber_index=AMB_PROH_3HD; 
  resPt->bkbAlist[PROH_CG]->amber_index=AMB_PROH_CG; 
  resPt->bkbAlist[PROH_1HG]->amber_index=AMB_PROH_2HG; 
  resPt->bkbAlist[PROH_2HG]->amber_index=AMB_PROH_3HG; 
  resPt->bkbAlist[PROH_CB]->amber_index=AMB_PROH_CB; 
  resPt->bkbAlist[PROH_1HB]->amber_index=AMB_PROH_2HB; 
  resPt->bkbAlist[PROH_2HB]->amber_index=AMB_PROH_3HB;
  resPt->bkbAlist[PROH_CA]->amber_index=AMB_PROH_CA; 
  resPt->bkbAlist[PROH_HA]->amber_index=AMB_PROH_HA; 
  resPt->bkbAlist[PROH_C]->amber_index=AMB_PROH_C; 
  resPt->bkbAlist[PROH_O]->amber_index=AMB_PROH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[PROH_OXT]->amber_index=AMB_PROH_OXT;}

  /*   AMB_PROH_N;  */
  /*   AMB_PROH_CD;  */
  /*   AMB_PROH_2HD;  */
  /*   AMB_PROH_3HD;  */
  /*   AMB_PROH_CG;  */
  /*   AMB_PROH_2HG;  */
  /*   AMB_PROH_3HG;  */
  /*   AMB_PROH_CB;  */
  /*   AMB_PROH_2HB;  */
  /*   AMB_PROH_3HB; */
  /*   AMB_PROH_CA;  */
  /*   AMB_PROH_HA;  */
  /*   AMB_PROH_C;  */
  /*   AMB_PROH_O;  */
  /*   AMB_PROH_OXT; */

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_PROH_amber_serials(resPt,firstSer+AMB_PROH_N,2);
  
  if (resPt->flagCterm)*lastSer = AMB_PROH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[PROH_O]->amber_index;

  return TRUE;
}

/**********************************************************************/

static int update_amber_serial_GLYH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;

  resPt->bkbAlist[GLYH_N]->amber_index=AMB_GLYH_N;
  // AMB_GLYH_H --> H'
  resPt->bkbAlist[GLYH_CA]->amber_index=AMB_GLYH_CA; 
  resPt->bkbAlist[GLYH_1HA]->amber_index=AMB_GLYH_2HA; 
  resPt->bkbAlist[GLYH_2HA]->amber_index=AMB_GLYH_3HA; 
  resPt->bkbAlist[GLYH_C]->amber_index=AMB_GLYH_C; 
  resPt->bkbAlist[GLYH_O]->amber_index=AMB_GLYH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[GLYH_OXT]->amber_index=AMB_GLYH_OXT;}

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[GLYH_N]) {
      aPt->amber_index=AMB_GLYH_H;
      end = !resPt->flagNterm;;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_GLYH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_GLYH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[GLYH_O]->amber_index;

  return TRUE;

  /*   AMB_GLYH_N; */
  /*   AMB_GLYH_H; */
  /*   AMB_GLYH_CA;  */
  /*   AMB_GLYH_2HA;  */
  /*   AMB_GLYH_3HA;  */
  /*   AMB_GLYH_C;  */
  /*   AMB_GLYH_O;  */
  /*   AMB_GLYH_OXT; */
}

/**********************************************************************/

static int update_amber_serial_ALAH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  
  resPt->bkbAlist[genH_N]->amber_index=AMB_ALAH_N;
  // AMB_ALAH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_ALAH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_ALAH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_ALAH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_ALAH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_ALAH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_ALAH_OXT;}

  resPt->schAlist[ALAH_1HB]->amber_index=AMB_ALAH_1HB; 
  resPt->schAlist[ALAH_2HB]->amber_index=AMB_ALAH_2HB; 
  resPt->schAlist[ALAH_3HB]->amber_index=AMB_ALAH_3HB; 

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_ALAH_H; 
      end = !resPt->flagNterm;
    }
    if(!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  
  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_ALAH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_ALAH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_ALAH_N; */
  /*   AMB_ALAH_H; */
  /*   AMB_ALAH_CA;  */
  /*   AMB_ALAH_HA;  */
  /*   AMB_ALAH_CB;  */
  /*   AMB_ALAH_1HB;  */
  /*   AMB_ALAH_2HB;  */
  /*   AMB_ALAH_3HB;  */
  /*   AMB_ALAH_C; */
  /*   AMB_ALAH_O;  */
  /*   AMB_ALAH_OXT; */
}

/**********************************************************************/

static int update_amber_serial_ARGH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_ARGH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_NE_HPrime_left = 1;
  int n_NH1_HPrime_left = 2;
  int n_NH2_HPrime_left = 2;
  
  resPt->bkbAlist[genH_N]->amber_index=AMB_ARGH_N;
  // AMB_ARGH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_ARGH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_ARGH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_ARGH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_ARGH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_ARGH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_ARGH_OXT;}

  resPt->schAlist[ARGH_1HB]->amber_index=AMB_ARGH_2HB; 
  resPt->schAlist[ARGH_2HB]->amber_index=AMB_ARGH_3HB; 
  resPt->schAlist[ARGH_CG]->amber_index=AMB_ARGH_CG; 
  resPt->schAlist[ARGH_1HG]->amber_index=AMB_ARGH_2HG; 
  resPt->schAlist[ARGH_2HG]->amber_index=AMB_ARGH_3HG; 
  resPt->schAlist[ARGH_CD]->amber_index=AMB_ARGH_CD; 
  resPt->schAlist[ARGH_1HD]->amber_index=AMB_ARGH_2HD; 
  resPt->schAlist[ARGH_2HD]->amber_index=AMB_ARGH_3HD; 
  resPt->schAlist[ARGH_NE]->amber_index=AMB_ARGH_NE; 
  // AMB_ARGH_HE --> H' 
  resPt->schAlist[ARGH_CZ]->amber_index=AMB_ARGH_CZ; 
  resPt->schAlist[ARGH_NH1]->amber_index=AMB_ARGH_NH1; 
  // AMB_ARGH_1HH1 --> H'
  // AMB_ARGH_2HH1 --> H'
  resPt->schAlist[ARGH_NH2]->amber_index=AMB_ARGH_NH2;
  // AMB_ARGH_1HH2 --> H'
  // AMB_ARGH_2HH2 --> H'

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_ARGH_H; 
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;

  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[ARGH_NE]) {
      aPt->amber_index=AMB_ARGH_HE;
      n_NE_HPrime_left--;
    }
    else if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[ARGH_NH1]) {
      if (n_NH1_HPrime_left==2) aPt->amber_index=AMB_ARGH_1HH1;
      else if (n_NH1_HPrime_left==1) aPt->amber_index=AMB_ARGH_2HH1;
      n_NH1_HPrime_left--;      
    }
    else if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[ARGH_NH2]) {
      if (n_NH2_HPrime_left==2) aPt->amber_index=AMB_ARGH_1HH2;
      else if (n_NH2_HPrime_left==1) aPt->amber_index=AMB_ARGH_2HH2;
      n_NH2_HPrime_left--;      
    }
    end = ((n_NE_HPrime_left==0) && (n_NH1_HPrime_left==0) && (n_NH2_HPrime_left==0));
    if (!end) index_sch++;
  }

  if (!end) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_ARGH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_ARGH_OXT+firstSer;
  else  *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_ARGH_N; */
  /*   AMB_ARGH_H; */
  /*   AMB_ARGH_CA;  */
  /*   AMB_ARGH_HA;  */
  /*   AMB_ARGH_CB;  */
  /*   AMB_ARGH_2HB;  */
  /*   AMB_ARGH_3HB;  */
  /*   AMB_ARGH_CG;  */
  /*   AMB_ARGH_2HG;  */
  /*   AMB_ARGH_3HG;  */
  /*   AMB_ARGH_CD;  */
  /*   AMB_ARGH_2HD;  */
  /*   AMB_ARGH_3HD;  */
  /*   AMB_ARGH_NE;  */
  /*   AMB_ARGH_HE; */
  /*   AMB_ARGH_CZ;  */
  /*   AMB_ARGH_NH1;  */
  /*   AMB_ARGH_1HH1; */
  /*   AMB_ARGH_2HH1; */
  /*   AMB_ARGH_NH2; */
  /*   AMB_ARGH_1HH2; */
  /*   AMB_ARGH_2HH2; */
  /*   AMB_ARGH_C; */
  /*   AMB_ARGH_O;  */
  /*   AMB_ARGH_OXT; */
  /*-- H' ---*/
  
}

/**********************************************************************/

static int update_amber_serial_ASNH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_ASNH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_ND2_HPrime_left = 2;
  
  resPt->bkbAlist[genH_N]->amber_index=AMB_ASNH_N;
  // AMB_ASNH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_ASNH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_ASNH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_ASNH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_ASNH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_ASNH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_ASNH_OXT;}
  
  resPt->schAlist[ASNH_1HB]->amber_index=AMB_ASNH_2HB; 
  resPt->schAlist[ASNH_2HB]->amber_index=AMB_ASNH_3HB; 
  resPt->schAlist[ASNH_CG]->amber_index=AMB_ASNH_CG; 
  resPt->schAlist[ASNH_OD1]->amber_index=AMB_ASNH_OD1; 
  resPt->schAlist[ASNH_ND2]->amber_index=AMB_ASNH_ND2;
  // AMB_ASNH_1HD2 --> H'
  // AMB_ASNH_2HD2 --> H'

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_ASNH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;

  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[ASNH_ND2]) {
      if (n_ND2_HPrime_left==2) {
	aPt->amber_index=AMB_ASNH_1HD2;
	n_ND2_HPrime_left--;
      }
      else if (n_ND2_HPrime_left==1) {
	aPt->amber_index=AMB_ASNH_2HD2;
	n_ND2_HPrime_left--;
      }
    }
    end = (n_ND2_HPrime_left==0);
    if (!end) index_sch++;
  }

  if (!end) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_ASNH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_ASNH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_ASNH_N; */
  /*   AMB_ASNH_H; */
  /*   AMB_ASNH_CA;  */
  /*   AMB_ASNH_HA;  */
  /*   AMB_ASNH_CB;  */
  /*   AMB_ASNH_2HB;  */
  /*   AMB_ASNH_3HB;  */
  /*   AMB_ASNH_CG;  */
  /*   AMB_ASNH_OD1;  */
  /*   AMB_ASNH_ND2; */
  /*   AMB_ASNH_1HD2; */
  /*   AMB_ASNH_2HD2; */
  /*   AMB_ASNH_C; */
  /*   AMB_ASNH_O;  */
  /*   AMB_ASNH_OXT; */

}

/**********************************************************************/

static int update_amber_serial_ASPH(residue* resPt, int firstSer, int* lastSer){
  // !!! = ASP, reste ASH

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_ASPH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_OD2_HPrime_left = 1;
   
  resPt->bkbAlist[genH_N]->amber_index=AMB_ASPH_N;
  // AMB_ASPH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_ASPH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_ASPH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_ASPH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_ASPH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_ASPH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_ASPH_OXT;}
  
  resPt->schAlist[ASPH_1HB]->amber_index=AMB_ASPH_2HB; 
  resPt->schAlist[ASPH_2HB]->amber_index=AMB_ASPH_3HB; 
  resPt->schAlist[ASPH_CG]->amber_index=AMB_ASPH_CG; 
  resPt->schAlist[ASPH_OD1]->amber_index=AMB_ASPH_OD1; 
  resPt->schAlist[ASPH_OD2]->amber_index=AMB_ASPH_OD2;

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_ASPH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;

  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[ASPH_OD2]) {
      aPt->amber_index=AMB_ASHH_HD2;
      n_OD2_HPrime_left--;
    }
    end = (n_OD2_HPrime_left==0);
    if (!end) index_sch++;
  }
  
  if (end) {
    translate_amber_index_ASPH_ASHH(resPt,firstSer,lastSer);
  }
  else {
    for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
      if (resPt->bkbAlist[atomInd]!=NULL
	  && resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
	resPt->bkbAlist[atomInd]->amber_index += firstSer;
      }
    }
    for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
      if (resPt->schAlist[atomInd]!=NULL
	  && resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
	resPt->schAlist[atomInd]->amber_index += firstSer;
      }
    }

    if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_ASPH_H,2);
  
    if (resPt->flagCterm)*lastSer = AMB_ASPH_OXT+firstSer;
    else *lastSer = resPt->bkbAlist[genH_O]->amber_index;
  }

  return TRUE;

  /*   AMB_ASPH_N; */
  /*   AMB_ASPH_H */
  /*   AMB_ASPH_CA;  */
  /*   AMB_ASPH_HA;  */
  /*   AMB_ASPH_CB;  */
  /*   AMB_ASPH_2HB;  */
  /*   AMB_ASPH_3HB;  */
  /*   AMB_ASPH_CG;  */
  /*   AMB_ASPH_OD1;  */
  /*   AMB_ASPH_OD2; */
  /*   AMB_ASPH_C; */
  /*   AMB_ASPH_O;  */
  /*   AMB_ASPH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_CYSH(residue* resPt, int firstSer, int* lastSer){
  // !!! = CYS, restent CYM, CYX

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int end_sch = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_CYSH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;

  resPt->bkbAlist[genH_N]->amber_index=AMB_CYSH_N;
  // AMB_CYSH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_CYSH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_CYSH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_CYSH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_CYSH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_CYSH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_CYSH_OXT;}
  
  resPt->schAlist[CYSH_1HB]->amber_index=AMB_CYSH_2HB; 
  resPt->schAlist[CYSH_2HB]->amber_index=AMB_CYSH_3HB; 
  resPt->schAlist[CYSH_SG]->amber_index=AMB_CYSH_SG;
  // AMB_CYSH_HG --> H'
  
  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_CYSH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  // sch
  while(index_sch<index_sch_end && !end_sch) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[CYSH_SG]) {
      aPt->amber_index=AMB_CYSH_HG;
      end_sch = TRUE;
    }
    else
      index_sch++;
  }
  if (!end_sch) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_CYSH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_CYSH_OXT+firstSer;
  else  *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_CYSH_N; */
  /*   AMB_CYSH_H; */
  /*   AMB_CYSH_CA;  */
  /*   AMB_CYSH_HA;  */
  /*   AMB_CYSH_CB;  */
  /*   AMB_CYSH_2HB;  */
  /*   AMB_CYSH_3HB;  */
  /*   AMB_CYSH_SG; */
  /*   AMB_CYSH_HG; */
  /*   AMB_CYSH_C; */
  /*   AMB_CYSH_O;  */
  /*   AMB_CYSH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_GLNH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_GLNH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_NE2_HPrime_left = 2;

  resPt->bkbAlist[genH_N]->amber_index=AMB_GLNH_N;
  // AMB_GLNH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_GLNH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_GLNH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_GLNH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_GLNH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_GLNH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_GLNH_OXT;}
  
  resPt->schAlist[GLNH_1HB]->amber_index=AMB_GLNH_2HB; 
  resPt->schAlist[GLNH_2HB]->amber_index=AMB_GLNH_3HB; 
  resPt->schAlist[GLNH_CG]->amber_index=AMB_GLNH_CG; 
  resPt->schAlist[GLNH_1HG]->amber_index=AMB_GLNH_2HG; 
  resPt->schAlist[GLNH_2HG]->amber_index=AMB_GLNH_3HG; 
  resPt->schAlist[GLNH_CD]->amber_index=AMB_GLNH_CD; 
  resPt->schAlist[GLNH_OE1]->amber_index=AMB_GLNH_OE1; 
  resPt->schAlist[GLNH_NE2]->amber_index=AMB_GLNH_NE2;
  // AMB_GLNH_1HE2 --> H'
  // AMB_GLNH_2HE2 --> H'
  
  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_GLNH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;

  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[GLNH_NE2]) {
      if (n_NE2_HPrime_left==2) {
	aPt->amber_index=AMB_GLNH_1HE2;
	n_NE2_HPrime_left--;      
      }
      else if (n_NE2_HPrime_left==1) {
	aPt->amber_index=AMB_GLNH_2HE2;
	n_NE2_HPrime_left--;      
      }
    }
    end = (n_NE2_HPrime_left==0);
    if (!end) index_sch++;
  }
  if (!end) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_GLNH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_GLNH_OXT+firstSer;
  else  *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_GLNH_N; */
  /*   AMB_GLNH_H; */
  /*   AMB_GLNH_CA;  */
  /*   AMB_GLNH_HA;  */
  /*   AMB_GLNH_CB;  */
  /*   AMB_GLNH_2HB;  */
  /*   AMB_GLNH_3HB;  */
  /*   AMB_GLNH_CG;  */
  /*   AMB_GLNH_2HG;  */
  /*   AMB_GLNH_3HG;  */
  /*   AMB_GLNH_CD;  */
  /*   AMB_GLNH_OE1;  */
  /*   AMB_GLNH_NE2; */
  /*   AMB_GLNH_1HE2; */
  /*   AMB_GLNH_2HE2; */
  /*   AMB_GLNH_C; */
  /*   AMB_GLNH_O;  */
  /*   AMB_GLNH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_GLUH(residue* resPt, int firstSer, int* lastSer){
  // !!! = GLU, reste GLH
  
  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_GLUH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_OE2_HPrime_left = 1;

  resPt->bkbAlist[genH_N]->amber_index=AMB_GLUH_N;
  // AMB_GLUH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_GLUH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_GLUH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_GLUH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_GLUH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_GLUH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_GLUH_OXT;}
  
  resPt->schAlist[GLUH_1HB]->amber_index=AMB_GLUH_2HB; 
  resPt->schAlist[GLUH_2HB]->amber_index=AMB_GLUH_3HB; 
  resPt->schAlist[GLUH_CG]->amber_index=AMB_GLUH_CG; 
  resPt->schAlist[GLUH_1HG]->amber_index=AMB_GLUH_2HG; 
  resPt->schAlist[GLUH_2HG]->amber_index=AMB_GLUH_3HG; 
  resPt->schAlist[GLUH_CD]->amber_index=AMB_GLUH_CD; 
  resPt->schAlist[GLUH_OE1]->amber_index=AMB_GLUH_OE1; 
  resPt->schAlist[GLUH_OE2]->amber_index=AMB_GLUH_OE2;
  
  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_GLUH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;

  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[GLUH_OE2]) {
      aPt->amber_index=AMB_GLHH_HE2;
      n_OE2_HPrime_left--;
    }
    end = (n_OE2_HPrime_left==0);
    if (!end) index_sch++;
  }

  if (end) {
    translate_amber_index_GLUH_GLHH(resPt,firstSer,lastSer);
  }
  else {
    for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
      if (resPt->bkbAlist[atomInd]!=NULL
	  && resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
	resPt->bkbAlist[atomInd]->amber_index += firstSer;
      }
    }
    for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
      if (resPt->schAlist[atomInd]!=NULL
	  && resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
	resPt->schAlist[atomInd]->amber_index += firstSer;
      }
    }
    
    if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_GLUH_H,2);
  
    if (resPt->flagCterm)*lastSer = AMB_GLUH_OXT+firstSer;
    else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  }

  return TRUE;

  /*  AMB_GLUH_N; */
  /*   AMB_GLUH_H; */
  /*   AMB_GLUH_CA;  */
  /*   AMB_GLUH_HA;  */
  /*   AMB_GLUH_CB;  */
  /*   AMB_GLUH_2HB;  */
  /*   AMB_GLUH_3HB;  */
  /*   AMB_GLUH_CG;  */
  /*   AMB_GLUH_2HG;  */
  /*   AMB_GLUH_3HG;  */
  /*   AMB_GLUH_CD;  */
  /*   AMB_GLUH_OE1;  */
  /*   AMB_GLUH_OE2; */
  /*   AMB_GLUH_C; */
  /*   AMB_GLUH_O;  */
  /*   AMB_GLUH_OXT; */
}

/**********************************************************************/

static int update_amber_serial_HISH(residue* resPt, int firstSer, int* lastSer){
  // !!!! = HIP, restent HIE et HID
  
  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_HISH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_NE2_HPrime_left = 1;
  int n_ND1_HPrime_left = 1;

  resPt->bkbAlist[genH_N]->amber_index=AMB_HIPH_N;
  // AMB_HIPH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_HIPH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_HIPH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_HIPH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_HIPH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_HIPH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_HIPH_OXT;}
  
  resPt->schAlist[HISH_1HB]->amber_index=AMB_HIPH_2HB; 
  resPt->schAlist[HISH_2HB]->amber_index=AMB_HIPH_3HB; 
  resPt->schAlist[HISH_CG]->amber_index=AMB_HIPH_CG; 
  resPt->schAlist[HISH_ND1]->amber_index=AMB_HIPH_ND1; 
  resPt->schAlist[HISH_CE1]->amber_index=AMB_HIPH_CE1; 
  resPt->schAlist[HISH_HE1]->amber_index=AMB_HIPH_HE1; 
  resPt->schAlist[HISH_NE2]->amber_index=AMB_HIPH_NE2;
  resPt->schAlist[HISH_CD2]->amber_index=AMB_HIPH_CD2; 
  resPt->schAlist[HISH_HD2]->amber_index=AMB_HIPH_HD2; 

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_HIPH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;
  
  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[HISH_ND1]) {
      aPt->amber_index=AMB_HIPH_HD1;
      n_ND1_HPrime_left--;
    }
    else if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[HISH_NE2]) {
      aPt->amber_index=AMB_HIPH_HE2;
      n_NE2_HPrime_left--;
    }
    end = ((n_NE2_HPrime_left==0) && (n_ND1_HPrime_left==0));
    if (!end) index_sch++;
  }

  if (!end) {
    if ((n_NE2_HPrime_left!=0) && (n_ND1_HPrime_left!=0)) {
      printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
      return FALSE;      
    }
    else if ((n_NE2_HPrime_left==0) && (n_ND1_HPrime_left==1)) {
      translate_amber_index_HIPH_HIEH(resPt,firstSer,lastSer);
    }
    else if ((n_NE2_HPrime_left==1) && (n_ND1_HPrime_left==0)) {
      translate_amber_index_HIPH_HIDH(resPt,firstSer,lastSer);    
    }
    return TRUE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
   }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_HIPH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_HIPH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_HIPH_N; */
  /*   AMB_HIPH_H;  */
  /*   AMB_HIPH_CA;  */
  /*   AMB_HIPH_HA;  */
  /*   AMB_HIPH_CB;  */
  /*   AMB_HIPH_2HB;  */
  /*   AMB_HIPH_3HB;  */
  /*   AMB_HIPH_CG;  */
  /*   AMB_HIPH_ND1;  */
  /*   AMB_HIPH_HD1;  */
  /*   AMB_HIPH_CE1;  */
  /*   AMB_HIPH_HE1;  */
  /*   AMB_HIPH_NE2; */
  /*   AMB_HIPH_HE2;  */
  /*   AMB_HIPH_CD2;  */
  /*   AMB_HIPH_HD2;  */
  /*   AMB_HIPH_C; */
  /*   AMB_HIPH_O;  */
  /*   AMB_HIPH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_ILEH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;

  resPt->bkbAlist[genH_N]->amber_index=AMB_ILEH_N;
  // AMB_ILEH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_ILEH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_ILEH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_ILEH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_ILEH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_ILEH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_ILEH_OXT;}
  
  resPt->schAlist[ILEH_HB]->amber_index=AMB_ILEH_HB; 
  resPt->schAlist[ILEH_CG2]->amber_index=AMB_ILEH_CG2; 
  resPt->schAlist[ILEH_1HG2]->amber_index=AMB_ILEH_1HG2; 
  resPt->schAlist[ILEH_2HG2]->amber_index=AMB_ILEH_2HG2; 
  resPt->schAlist[ILEH_3HG2]->amber_index=AMB_ILEH_3HG2; 
  resPt->schAlist[ILEH_CG1]->amber_index=AMB_ILEH_CG1; 
  resPt->schAlist[ILEH_1HG1]->amber_index=AMB_ILEH_2HG1; 
  resPt->schAlist[ILEH_2HG1]->amber_index=AMB_ILEH_3HG1; 
  resPt->schAlist[ILEH_CD1]->amber_index=AMB_ILEH_CD1;
  resPt->schAlist[ILEH_1HD1]->amber_index=AMB_ILEH_1HD1; 
  resPt->schAlist[ILEH_2HD1]->amber_index=AMB_ILEH_2HD1; 
  resPt->schAlist[ILEH_3HD1]->amber_index=AMB_ILEH_3HD1;

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_ILEH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_ILEH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_ILEH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_ILEH_N; */
  /*   AMB_ILEH_H;  */
  /*   AMB_ILEH_CA;  */
  /*   AMB_ILEH_HA;  */
  /*   AMB_ILEH_CB;  */
  /*   AMB_ILEH_HB;  */
  /*   AMB_ILEH_CG2;  */
  /*   AMB_ILEH_1HG2;  */
  /*   AMB_ILEH_2HG2;  */
  /*   AMB_ILEH_3HG2;  */
  /*   AMB_ILEH_CG1;  */
  /*   AMB_ILEH_2HG1;  */
  /*   AMB_ILEH_3HG1;  */
  /*   AMB_ILEH_CD1; */
  /*   AMB_ILEH_1HD1;  */
  /*   AMB_ILEH_2HD1;  */
  /*   AMB_ILEH_3HD1; */
  /*   AMB_ILEH_C; */
  /*   AMB_ILEH_O;  */
  /*   AMB_ILEH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_LEUH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;

  resPt->bkbAlist[genH_N]->amber_index=AMB_LEUH_N;
  // AMB_LEUH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_LEUH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_LEUH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_LEUH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_LEUH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_LEUH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_LEUH_OXT;}
  
  resPt->schAlist[LEUH_1HB]->amber_index=AMB_LEUH_1HB; 
  resPt->schAlist[LEUH_2HB]->amber_index=AMB_LEUH_2HB; 
  resPt->schAlist[LEUH_CG]->amber_index=AMB_LEUH_CG; 
  resPt->schAlist[LEUH_HG]->amber_index=AMB_LEUH_HG; 
  resPt->schAlist[LEUH_CD1]->amber_index=AMB_LEUH_CD1; 
  resPt->schAlist[LEUH_1HD1]->amber_index=AMB_LEUH_1HD1; 
  resPt->schAlist[LEUH_2HD1]->amber_index=AMB_LEUH_2HD1; 
  resPt->schAlist[LEUH_3HD1]->amber_index=AMB_LEUH_3HD1; 
  resPt->schAlist[LEUH_CD2]->amber_index=AMB_LEUH_CD2;
  resPt->schAlist[LEUH_1HD2]->amber_index=AMB_LEUH_1HD2; 
  resPt->schAlist[LEUH_2HD2]->amber_index=AMB_LEUH_2HD2; 
  resPt->schAlist[LEUH_3HD2]->amber_index=AMB_LEUH_3HD2;

   /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_LEUH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
   
  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_LEUH_H,2);

  if (resPt->flagCterm)*lastSer = AMB_LEUH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_LEUH_N; */
  /*   AMB_LEUH_H;  */
  /*   AMB_LEUH_CA;  */
  /*   AMB_LEUH_HA;  */
  /*   AMB_LEUH_CB;  */
  /*   AMB_LEUH_1HB;  */
  /*   AMB_LEUH_2HB;  */
  /*   AMB_LEUH_CG;  */
  /*   AMB_LEUH_HG;  */
  /*   AMB_LEUH_CD1;  */
  /*   AMB_LEUH_1HD1;  */
  /*   AMB_LEUH_2HD1;  */
  /*   AMB_LEUH_3HD1;  */
  /*   AMB_LEUH_CD2; */
  /*   AMB_LEUH_1HD2;  */
  /*   AMB_LEUH_2HD2;  */
  /*   AMB_LEUH_3HD2; */
  /*   AMB_LEUH_C; */
  /*   AMB_LEUH_O;  */
  /*   AMB_LEUH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_LYSH(residue* resPt, int firstSer, int* lastSer){
  // !!! = LYS, reste LYN
 
  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_LYSH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  int n_NZ_HPrime_left = 3;

  resPt->bkbAlist[genH_N]->amber_index=AMB_LYSH_N;
  // AMB_LYSH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_LYSH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_LYSH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_LYSH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_LYSH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_LYSH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_LYSH_OXT;}
 
  resPt->schAlist[LYSH_1HB]->amber_index=AMB_LYSH_2HB; 
  resPt->schAlist[LYSH_2HB]->amber_index=AMB_LYSH_3HB; 
  resPt->schAlist[LYSH_CG]->amber_index=AMB_LYSH_CG; 
  resPt->schAlist[LYSH_1HG]->amber_index=AMB_LYSH_2HG; 
  resPt->schAlist[LYSH_2HG]->amber_index=AMB_LYSH_3HG; 
  resPt->schAlist[LYSH_CD]->amber_index=AMB_LYSH_CD; 
  resPt->schAlist[LYSH_1HD]->amber_index=AMB_LYSH_2HD; 
  resPt->schAlist[LYSH_2HD]->amber_index=AMB_LYSH_3HD;
  resPt->schAlist[LYSH_CE]->amber_index=AMB_LYSH_CE; 
  resPt->schAlist[LYSH_1HE]->amber_index=AMB_LYSH_2HE; 
  resPt->schAlist[LYSH_2HE]->amber_index=AMB_LYSH_3HE; 
  resPt->schAlist[LYSH_NZ]->amber_index=AMB_LYSH_NZ;
  // AMB_LYSH_1HZ
  // AMB_LYSH_2HZ
  // AMB_LYSH_3HZ

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_LYSH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  else end = FALSE;
  
  // sch
  while(index_sch<index_sch_end && !end) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[LYSH_NZ]) {
      if (n_NZ_HPrime_left==3) {
	aPt->amber_index=AMB_LYSH_1HZ;
	n_NZ_HPrime_left--;
      }
      else if (n_NZ_HPrime_left==2) {
	aPt->amber_index=AMB_LYSH_2HZ;
	n_NZ_HPrime_left--;
      }
      else if (n_NZ_HPrime_left==1) {
	aPt->amber_index=AMB_LYSH_3HZ;
	n_NZ_HPrime_left--;
      }
    }
    end = (n_NZ_HPrime_left==0);
    if (!end) index_sch++;
  }
 
  if (!end) {
    if (n_NZ_HPrime_left==1) {
      translate_amber_index_LYSH_LYNH(resPt,firstSer,lastSer);
    }
    else {
      printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
      return FALSE;
    }
  }
  else {
    for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
      if (resPt->bkbAlist[atomInd]!=NULL
	  && resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
	resPt->bkbAlist[atomInd]->amber_index += firstSer;
      }
    }
    for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
      if (resPt->schAlist[atomInd]!=NULL
	  && resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
	resPt->schAlist[atomInd]->amber_index += firstSer;
      }
    }
    
    if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_LYSH_H,2);
    
    if (resPt->flagCterm)*lastSer = AMB_LYSH_OXT+firstSer;
    else *lastSer = resPt->bkbAlist[genH_O]->amber_index;
  
  }

  return TRUE;

  /*   AMB_LYSH_N; */
  /*   AMB_LYSH_H; */
  /*   AMB_LYSH_CA;  */
  /*   AMB_LYSH_HA;  */
  /*   AMB_LYSH_CB;  */
  /*   AMB_LYSH_2HB;  */
  /*   AMB_LYSH_3HB;  */
  /*   AMB_LYSH_CG;  */
  /*   AMB_LYSH_2HG;  */
  /*   AMB_LYSH_3HG;  */
  /*   AMB_LYSH_CD;  */
  /*   AMB_LYSH_2HD;  */
  /*   AMB_LYSH_3HD; */
  /*   AMB_LYSH_CE;  */
  /*   AMB_LYSH_2HE;  */
  /*   AMB_LYSH_3HE;  */
  /*   AMB_LYSH_NZ; */
  /*   AMB_LYSH_HZ1; */
  /*   AMB_LYSH_HZ2; */
  /*   AMB_LYSH_HZ3; */
  /*   AMB_LYSH_C; */
  /*   AMB_LYSH_O;  */
  /*   AMB_LYSH_OXT; */

}
/**********************************************************************/

static int update_amber_serial_METH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;

  resPt->bkbAlist[genH_N]->amber_index=AMB_METH_N;
  // AMB_METH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_METH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_METH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_METH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_METH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_METH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_METH_OXT;}
  
  resPt->schAlist[METH_1HB]->amber_index=AMB_METH_2HB; 
  resPt->schAlist[METH_2HB]->amber_index=AMB_METH_3HB; 
  resPt->schAlist[METH_CG]->amber_index=AMB_METH_CG; 
  resPt->schAlist[METH_1HG]->amber_index=AMB_METH_2HG; 
  resPt->schAlist[METH_2HG]->amber_index=AMB_METH_3HG; 
  resPt->schAlist[METH_SD]->amber_index=AMB_METH_SD; 
  resPt->schAlist[METH_CE]->amber_index=AMB_METH_CE; 
  resPt->schAlist[METH_1HE]->amber_index=AMB_METH_1HE; 
  resPt->schAlist[METH_2HE]->amber_index=AMB_METH_2HE; 
  resPt->schAlist[METH_3HE]->amber_index=AMB_METH_3HE; 

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_METH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_METH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_METH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;
  
  /*   AMB_METH_N; */
  /*   AMB_METH_H; */
  /*   AMB_METH_CA;  */
  /*   AMB_METH_HA;  */
  /*   AMB_METH_CB;  */
  /*   AMB_METH_2HB;  */
  /*   AMB_METH_3HB;  */
  /*   AMB_METH_CG;  */
  /*   AMB_METH_2HG;  */
  /*   AMB_METH_3HG;  */
  /*   AMB_METH_SD;  */
  /*   AMB_METH_CE;  */
  /*   AMB_METH_1HE;  */
  /*   AMB_METH_2HE;  */
  /*   AMB_METH_3HE;  */
  /*   AMB_METH_C; */
  /*   AMB_METH_O;  */
  /*   AMB_METH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_PHEH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
 
  resPt->bkbAlist[genH_N]->amber_index=AMB_PHEH_N;
  // AMB_PHEH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_PHEH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_PHEH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_PHEH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_PHEH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_PHEH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_PHEH_OXT;}
  
  resPt->schAlist[PHEH_1HB]->amber_index=AMB_PHEH_2HB; 
  resPt->schAlist[PHEH_2HB]->amber_index=AMB_PHEH_3HB; 
  resPt->schAlist[PHEH_CG]->amber_index=AMB_PHEH_CG; 
  resPt->schAlist[PHEH_CD1]->amber_index=AMB_PHEH_CD1; 
  resPt->schAlist[PHEH_HD1]->amber_index=AMB_PHEH_HD1; 
  resPt->schAlist[PHEH_CE1]->amber_index=AMB_PHEH_CE1; 
  resPt->schAlist[PHEH_HE1]->amber_index=AMB_PHEH_HE1; 
  resPt->schAlist[PHEH_CZ]->amber_index=AMB_PHEH_CZ; 
  resPt->schAlist[PHEH_HZ]->amber_index=AMB_PHEH_HZ; 
  resPt->schAlist[PHEH_CE2]->amber_index=AMB_PHEH_CE2; 
  resPt->schAlist[PHEH_HE2]->amber_index=AMB_PHEH_HE2; 
  resPt->schAlist[PHEH_CD2]->amber_index=AMB_PHEH_CD2; 
  resPt->schAlist[PHEH_HD2]->amber_index=AMB_PHEH_HD2;

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_PHEH_H;
      end = !resPt->flagNterm;
    }
    if (!end)  index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_PHEH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_PHEH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_PHEH_N; */
  /*   AMB_PHEH_H; */
  /*   AMB_PHEH_CA;  */
  /*   AMB_PHEH_HA;  */
  /*   AMB_PHEH_CB;  */
  /*   AMB_PHEH_2HB;  */
  /*   AMB_PHEH_3HB;  */
  /*   AMB_PHEH_CG;  */
  /*   AMB_PHEH_CD1;  */
  /*   AMB_PHEH_HD1;  */
  /*   AMB_PHEH_CE1;  */
  /*   AMB_PHEH_HE1;  */
  /*   AMB_PHEH_CZ;  */
  /*   AMB_PHEH_HZ;  */
  /*   AMB_PHEH_CE2;  */
  /*   AMB_PHEH_HE2;  */
  /*   AMB_PHEH_CD2;  */
  /*   AMB_PHEH_HD2; */
  /*   AMB_PHEH_C; */
  /*   AMB_PHEH_O;  */
  /*   AMB_PHEH_OXT; */
}

/**********************************************************************/

static int update_amber_serial_SERH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int end_sch = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_SERH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
 
  resPt->bkbAlist[  genH_N]->amber_index=AMB_SERH_N;
  // AMB_SERH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_SERH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_SERH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_SERH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_SERH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_SERH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_SERH_OXT;}
  
  resPt->schAlist[SERH_1HB]->amber_index=AMB_SERH_2HB; 
  resPt->schAlist[SERH_2HB]->amber_index=AMB_SERH_3HB; 
  resPt->schAlist[SERH_OG]->amber_index=AMB_SERH_OG;
  // AMB_SERH_HG --> H'

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_SERH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  // sch
  while(index_sch<index_sch_end && !end_sch) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[SERH_OG]) {
      aPt->amber_index=AMB_SERH_HG;
      end_sch = TRUE;
    }
    else
      index_sch++;
  }
  if (!end_sch) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_SERH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_SERH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_SERH_N; */
  /*   AMB_SERH_H; */
  /*   AMB_SERH_CA;  */
  /*   AMB_SERH_HA;  */
  /*   AMB_SERH_CB;  */
  /*   AMB_SERH_2HB;  */
  /*   AMB_SERH_3HB;  */
  /*   AMB_SERH_OG; */
  /*   AMB_SERH_HG; */
  /*   AMB_SERH_C; */
  /*   AMB_SERH_O;  */
  /*   AMB_SERH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_THRH(residue* resPt, int firstSer, int* lastSer){
  
  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int end_sch = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_THRH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;

  resPt->bkbAlist[ genH_N]->amber_index=AMB_THRH_N;
  // AMB_THRH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_THRH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_THRH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_THRH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_THRH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_THRH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_THRH_OXT;}
  
  resPt->schAlist[THRH_HB]->amber_index=AMB_THRH_HB; 
  resPt->schAlist[THRH_CG2]->amber_index=AMB_THRH_CG2; 
  resPt->schAlist[THRH_1HG2]->amber_index=AMB_THRH_1HG2; 
  resPt->schAlist[THRH_2HG2]->amber_index=AMB_THRH_2HG2; 
  resPt->schAlist[THRH_3HG2]->amber_index=AMB_THRH_3HG2;
  resPt->schAlist[THRH_OG1]->amber_index=AMB_THRH_OG1; 
  // AMB_THRH_HG1 --> H'

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_THRH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  // sch
  while(index_sch<index_sch_end && !end_sch) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[THRH_OG1]) {
      aPt->amber_index=AMB_THRH_HG1;
      end_sch = TRUE;
    }
    else
      index_sch++;
  }
  if (!end_sch) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }
  
  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_THRH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_THRH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*   AMB_THRH_N; */
  /*   AMB_THRH_H; */
  /*   AMB_THRH_CA;  */
  /*   AMB_THRH_HA;  */
  /*   AMB_THRH_CB;  */
  /*   AMB_THRH_HB;  */
  /*   AMB_THRH_CG2;  */
  /*   AMB_THRH_1HG2;  */
  /*   AMB_THRH_2HG2;  */
  /*   AMB_THRH_3HG2; */
  /*   AMB_THRH_OG1;  */
  /*   AMB_THRH_HG1; */
  /*   AMB_THRH_C; */
  /*   AMB_THRH_O;  */
  /*   AMB_THRH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_TRPH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int end_sch = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_TRPH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;

  resPt->bkbAlist[  genH_N]->amber_index=AMB_TRPH_N;
  // AMB_TRPH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_TRPH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_TRPH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_TRPH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_TRPH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_TRPH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_TRPH_OXT;}
  
  resPt->schAlist[TRPH_1HB]->amber_index=AMB_TRPH_2HB; 
  resPt->schAlist[TRPH_2HB]->amber_index=AMB_TRPH_3HB; 
  resPt->schAlist[TRPH_CG]->amber_index=AMB_TRPH_CG; 
  resPt->schAlist[TRPH_CD1]->amber_index=AMB_TRPH_CD1; 
  resPt->schAlist[TRPH_HD1]->amber_index=AMB_TRPH_HD1; 
  resPt->schAlist[TRPH_NE1]->amber_index=AMB_TRPH_NE1; 
  // AMB_TRPH_HE1 --> H'
  resPt->schAlist[TRPH_CE2]->amber_index=AMB_TRPH_CE2; 
  resPt->schAlist[TRPH_CZ2]->amber_index=AMB_TRPH_CZ2; 
  resPt->schAlist[TRPH_HZ2]->amber_index=AMB_TRPH_HZ2; 
  resPt->schAlist[TRPH_CH2]->amber_index=AMB_TRPH_CH2; 
  resPt->schAlist[TRPH_HH2]->amber_index=AMB_TRPH_HH2;
  resPt->schAlist[TRPH_CZ3]->amber_index=AMB_TRPH_CZ3; 
  resPt->schAlist[TRPH_HZ3]->amber_index=AMB_TRPH_HZ3; 
  resPt->schAlist[TRPH_CE3]->amber_index=AMB_TRPH_CE3; 
  resPt->schAlist[TRPH_HE3]->amber_index=AMB_TRPH_HE3;
  resPt->schAlist[TRPH_CD2]->amber_index=AMB_TRPH_CD2; 

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_TRPH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  // sch
  while(index_sch<index_sch_end && !end_sch) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[TRPH_NE1]) {
      aPt->amber_index=AMB_TRPH_HE1;
      end_sch = TRUE;
    }
    else
      index_sch++;
  }
  if (!end_sch) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_TRPH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_TRPH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;
  /*   AMB_TRPH_N; */
  /*   AMB_TRPH_H; */
  /*   AMB_TRPH_CA;  */
  /*   AMB_TRPH_HA;  */
  /*   AMB_TRPH_CB;  */
  /*   AMB_TRPH_2HB;  */
  /*   AMB_TRPH_3HB;  */
  /*   AMB_TRPH_CG;  */
  /*   AMB_TRPH_CD1;  */
  /*   AMB_TRPH_HD1;  */
  /*   AMB_TRPH_NE1;  */
  /*   AMB_TRPH_HE1; */
  /*   AMB_TRPH_CE2;  */
  /*   AMB_TRPH_CZ2;  */
  /*   AMB_TRPH_HZ2;  */
  /*   AMB_TRPH_CH2;  */
  /*   AMB_TRPH_HH2; */
  /*   AMB_TRPH_CZ3;  */
  /*   AMB_TRPH_HZ3;  */
  /*   AMB_TRPH_CE3;  */
  /*   AMB_TRPH_HE3; */
  /*   AMB_TRPH_CD2;  */
  /*   AMB_TRPH_C; */
  /*   AMB_TRPH_O;  */
  /*   AMB_TRPH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_TYRH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int end_sch = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_sch = N_TYRH_SCH_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;
  int index_sch_end = resPt->nschatoms;
  
  resPt->bkbAlist[  genH_N]->amber_index=AMB_TYRH_N;
  // AMB_TYRH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_TYRH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_TYRH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_TYRH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_TYRH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_TYRH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_TYRH_OXT;}
  
  resPt->schAlist[TYRH_1HB]->amber_index=AMB_TYRH_2HB; 
  resPt->schAlist[TYRH_2HB]->amber_index=AMB_TYRH_3HB; 
  resPt->schAlist[TYRH_CG]->amber_index=AMB_TYRH_CG; 
  resPt->schAlist[TYRH_CD1]->amber_index=AMB_TYRH_CD1; 
  resPt->schAlist[TYRH_HD1]->amber_index=AMB_TYRH_HD1; 
  resPt->schAlist[TYRH_CE1]->amber_index=AMB_TYRH_CE1; 
  resPt->schAlist[TYRH_HE1]->amber_index=AMB_TYRH_HE1; 
  resPt->schAlist[TYRH_CZ]->amber_index=AMB_TYRH_CZ; 
  resPt->schAlist[TYRH_OH]->amber_index=AMB_TYRH_OH;
  // AMB_TYRH_HH --> H'
  resPt->schAlist[TYRH_CE2]->amber_index=AMB_TYRH_CE2; 
  resPt->schAlist[TYRH_HE2]->amber_index=AMB_TYRH_HE2; 
  resPt->schAlist[TYRH_CD2]->amber_index=AMB_TYRH_CD2; 
  resPt->schAlist[TYRH_HD2]->amber_index=AMB_TYRH_HD2; 
  
  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_TYRH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  // sch
  while(index_sch<index_sch_end && !end_sch) {
    aPt = resPt->schAlist[index_sch];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->schAlist[TYRH_OH]) {
      aPt->amber_index=AMB_TYRH_HH;
      end_sch = TRUE;
    }
    else
      index_sch++;
  }
  if (!end_sch) {
    printf("ERROR (res %d): missing H' side chain atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_TYRH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_TYRH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;
  /*   AMB_TYRH_N; */
  /*   AMB_TYRH_H; */
  /*   AMB_TYRH_CA;  */
  /*   AMB_TYRH_HA;  */
  /*   AMB_TYRH_CB;  */
  /*   AMB_TYRH_2HB;  */
  /*   AMB_TYRH_3HB;  */
  /*   AMB_TYRH_CG;  */
  /*   AMB_TYRH_CD1;  */
  /*   AMB_TYRH_HD1;  */
  /*   AMB_TYRH_CE1;  */
  /*   AMB_TYRH_HE1;  */
  /*   AMB_TYRH_CZ;  */
  /*   AMB_TYRH_OH; */
  /*   AMB_TYRH_HH; */
  /*   AMB_TYRH_CE2;  */
  /*   AMB_TYRH_HE2;  */
  /*   AMB_TYRH_CD2;  */
  /*   AMB_TYRH_HD2;  */
  /*   AMB_TYRH_C; */
  /*   AMB_TYRH_O;  */
  /*   AMB_TYRH_OXT; */
}
/**********************************************************************/

static int update_amber_serial_VALH(residue* resPt, int firstSer, int* lastSer){

  int atomInd;
  atom* aPt = NULL;
  int end = FALSE;
  int index_bkb = N_GENH_BKB_ATOMS;
  int index_bkb_end = resPt->nbkbatoms;

  resPt->bkbAlist[genH_N]->amber_index=AMB_VALH_N;
  // AMB_VALH_H --> H'
  resPt->bkbAlist[genH_CA]->amber_index=AMB_VALH_CA; 
  resPt->bkbAlist[genH_HA]->amber_index=AMB_VALH_HA; 
  resPt->bkbAlist[genH_CB]->amber_index=AMB_VALH_CB; 
  resPt->bkbAlist[genH_C]->amber_index=AMB_VALH_C;
  resPt->bkbAlist[genH_O]->amber_index=AMB_VALH_O; 
  if (resPt->flagCterm) { resPt->bkbAlist[genH_OXT]->amber_index=AMB_VALH_OXT;}
  
  resPt->schAlist[VALH_HB]->amber_index=AMB_VALH_HB;
  resPt->schAlist[VALH_CG1]->amber_index=AMB_VALH_CG1; 
  resPt->schAlist[VALH_1HG1]->amber_index=AMB_VALH_1HG1; 
  resPt->schAlist[VALH_2HG1]->amber_index=AMB_VALH_2HG1; 
  resPt->schAlist[VALH_3HG1]->amber_index=AMB_VALH_3HG1; 
  resPt->schAlist[VALH_CG2]->amber_index=AMB_VALH_CG2; 
  resPt->schAlist[VALH_1HG2]->amber_index=AMB_VALH_1HG2; 
  resPt->schAlist[VALH_2HG2]->amber_index=AMB_VALH_2HG2; 
  resPt->schAlist[VALH_3HG2]->amber_index=AMB_VALH_3HG2; 

  /***** H' *****/
  // bkb
  while(index_bkb<index_bkb_end && !end) {
    aPt = resPt->bkbAlist[index_bkb];
    if (aPt->atomType==HYDROGEN_P && aPt->bondedAlist[0]==resPt->bkbAlist[genH_N]) {
      aPt->amber_index=AMB_VALH_H;
      end = !resPt->flagNterm;
    }
    if (!end) index_bkb++;
  }
  if (!end && !resPt->flagNterm) {
    printf("ERROR (res %d): missing H' backbone atoms - impossible to build AMBER datas\n", resPt->resSeq);
    return FALSE;
  }

  for (atomInd=0; atomInd<resPt->nbkbatoms; atomInd++) {
    if (resPt->bkbAlist[atomInd]!=NULL
	&& resPt->bkbAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->bkbAlist[atomInd]->amber_index += firstSer;
    }
  }
  for (atomInd=0; atomInd<resPt->nschatoms; atomInd++) {
    if (resPt->schAlist[atomInd]!=NULL
	&& resPt->schAlist[atomInd]->amber_index!=AMB_NONE) {
      resPt->schAlist[atomInd]->amber_index += firstSer;
    }
  }

  if (resPt->flagNterm) update_Nterm_amber_serials(resPt,firstSer+AMB_VALH_H,2);
  
  if (resPt->flagCterm)*lastSer = AMB_VALH_OXT+firstSer;
  else *lastSer = resPt->bkbAlist[genH_O]->amber_index;

  return TRUE;

  /*  AMB_VALH_N; */
  /*   AMB_VALH_H; */
  /*   AMB_VALH_CA;  */
  /*   AMB_VALH_HA;  */
  /*   AMB_VALH_CB;  */
  /*   AMB_VALH_HB; */
  /*   AMB_VALH_CG1;  */
  /*   AMB_VALH_1HG1;  */
  /*   AMB_VALH_2HG1;  */
  /*   AMB_VALH_3HG1;  */
  /*   AMB_VALH_CG2;  */
  /*   AMB_VALH_1HG2;  */
  /*   AMB_VALH_2HG2;  */
  /*   AMB_VALH_3HG2;  */
  /*   AMB_VALH_C; */
  /*   AMB_VALH_O;  */
  /*   AMB_VALH_OXT; */
}

/**********************************************************************/

int update_amber_serial(residue* resPt, int firstSer, int* lastSer) {
  
  if (check_amber_bkb_atoms(resPt) 
      && check_amber_sch_atoms(resPt)) {

    switch(resPt->resType) {
    case ALAH:
      return update_amber_serial_ALAH(resPt, firstSer, lastSer); 
    case ARGH:
      return update_amber_serial_ARGH(resPt, firstSer, lastSer); 
    case ASNH:
      return update_amber_serial_ASNH(resPt, firstSer, lastSer); 
    case ASPH:
      return update_amber_serial_ASPH(resPt, firstSer, lastSer); 
    case CYSH:
      return update_amber_serial_CYSH(resPt, firstSer, lastSer);  
    case GLNH:
      return update_amber_serial_GLNH(resPt, firstSer, lastSer);  
    case GLUH:
      return update_amber_serial_GLUH(resPt, firstSer, lastSer);  
    case GLYH:
      return update_amber_serial_GLYH(resPt, firstSer, lastSer);
    case HISH:
      return update_amber_serial_HISH(resPt, firstSer, lastSer);  
    case ILEH:
      return update_amber_serial_ILEH(resPt, firstSer, lastSer); 
    case LEUH:
      return update_amber_serial_LEUH(resPt, firstSer, lastSer); 
    case LYSH:
      return update_amber_serial_LYSH(resPt, firstSer, lastSer); 
    case METH:
      return update_amber_serial_METH(resPt, firstSer, lastSer); 
    case PHEH:
      return update_amber_serial_PHEH(resPt, firstSer, lastSer); 
    case PROH:
      return update_amber_serial_PROH(resPt, firstSer, lastSer);
    case SERH:
      return update_amber_serial_SERH(resPt, firstSer, lastSer); 
    case THRH:
      return update_amber_serial_THRH(resPt, firstSer, lastSer);  
    case TRPH:
      return update_amber_serial_TRPH(resPt, firstSer, lastSer); 
    case TYRH:
      return update_amber_serial_TYRH(resPt, firstSer, lastSer);  
    case VALH:
      return update_amber_serial_VALH(resPt, firstSer, lastSer); 
    default:
      return FALSE;
    }
  }
  else {
    return FALSE;
  }

}

/**********************************************************************/

int get_amber_serial(atom* aPt, int* serial) {
  if (aPt->amber_index!=AMB_NONE) {
    *serial = aPt->amber_index;
    return TRUE;
  }
  else return FALSE;
}

/**********************************************************************/
/**********************************************************************/

static void translate_psf_atom_name(atom* aPt, char *atomname, formatTypes pdb_format)
{
  switch(pdb_format) {
  case AMBER:
    switch(aPt->residuePt->resType) {
    case GLYH:
      if(strcmp(aPt->name,"1HA") == 0) 
	strcpy(atomname, "2HA");
      else if(strcmp(aPt->name,"2HA") == 0) 
	strcpy(atomname, "3HA");
      else if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    case PROH:
      if(strcmp(aPt->name,"1HB") == 0) 
	strcpy(atomname, "2HB");
      else if(strcmp(aPt->name,"2HB") == 0) 
	strcpy(atomname, "3HB");
      else if(strcmp(aPt->name,"1HG") == 0) 
	strcpy(atomname, "2HG");
      else if(strcmp(aPt->name,"2HG") == 0) 
	strcpy(atomname, "3HG");
      else if(strcmp(aPt->name,"1HD") == 0) 
	strcpy(atomname, "2HD");
      else if(strcmp(aPt->name,"2HD") == 0) 
	strcpy(atomname, "3HD");
      // H N-TERM ???
      else
	strcpy(atomname, aPt->name);
      break;
    case ARGH:
      if(strcmp(aPt->name,"1HB") == 0) 
	strcpy(atomname, "2HB");
      else if(strcmp(aPt->name,"2HB") == 0) 
	strcpy(atomname, "3HB");
      else if(strcmp(aPt->name,"1HG") == 0) 
	strcpy(atomname, "2HG");
      else if(strcmp(aPt->name,"2HG") == 0) 
	strcpy(atomname, "3HG");
      else if(strcmp(aPt->name,"1HD") == 0) 
	strcpy(atomname, "2HD");
      else if(strcmp(aPt->name,"2HD") == 0) 
	strcpy(atomname, "3HD");
      else if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    case ASNH:
    case ASPH:
    case CYSH:
    case HISH:
    case LEUH:
    case PHEH:
    case SERH:
    case TRPH:
    case TYRH:
      if(strcmp(aPt->name,"1HB") == 0) 
	strcpy(atomname, "2HB");
      else if(strcmp(aPt->name,"2HB") == 0) 
	strcpy(atomname, "3HB");
      else if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    case GLNH:
    case GLUH:
    case METH:
      if(strcmp(aPt->name,"1HB") == 0) 
	strcpy(atomname, "2HB");
      else if(strcmp(aPt->name,"2HB") == 0) 
	strcpy(atomname, "3HB");
      else if(strcmp(aPt->name,"1HG") == 0) 
	strcpy(atomname, "2HG");
      else if(strcmp(aPt->name,"2HG") == 0) 
	strcpy(atomname, "3HG");
      else if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    case ILEH:
      if(strcmp(aPt->name,"1HG1") == 0) 
	strcpy(atomname, "2HG1");
      else if(strcmp(aPt->name,"2HG1") == 0) 
	strcpy(atomname, "3HG1");
      else if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    case LYSH:
      if(strcmp(aPt->name,"1HB") == 0) 
	strcpy(atomname, "2HB");
      else if(strcmp(aPt->name,"2HB") == 0) 
	strcpy(atomname, "3HB");
      else if(strcmp(aPt->name,"1HG") == 0) 
	strcpy(atomname, "2HG");
      else if(strcmp(aPt->name,"2HG") == 0) 
	strcpy(atomname, "3HG");
      else if(strcmp(aPt->name,"1HD") == 0) 
	strcpy(atomname, "2HD");
      else if(strcmp(aPt->name,"2HD") == 0) 
	strcpy(atomname, "3HD");
      else if(strcmp(aPt->name,"1HE") == 0) 
	strcpy(atomname, "2HE");
      else if(strcmp(aPt->name,"2HE") == 0) 
	strcpy(atomname, "3HE");
      else if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    default:
      if(strcmp(aPt->name,"1H") == 0) 
	strcpy(atomname, "H2");
      else if(strcmp(aPt->name,"2H") == 0) 
	strcpy(atomname, "H3");
      else
	strcpy(atomname, aPt->name);
      break;
    }
    break;
  case INSIGHT:
    strcpy(atomname, aPt->name);
    break;
  }
}

/**********************************************************************/

static void translate_psf_res_name(residue *resPt, char *resName, formatTypes pdb_format) {
  
  atom* aPt = NULL;
  int i;
  int possible_HID = FALSE;
  int possible_HIE = FALSE;
  int possible_LYS = FALSE;
  int possible_ASH = FALSE;
  int possible_GLH = FALSE;  

  strcpy(resName, resPt->resName);

  switch(pdb_format) {

  case AMBER:
    switch (resPt->resType) {
    case HISH:
      for (i=N_HISH_SCH_ATOMS; i<resPt->nschatoms; i++){
	aPt = resPt->schAlist[i];
	if (strncmp(aPt->name, "HD1",4)==0) possible_HID = TRUE;
	else if (strncmp(aPt->name, "HE2",4)==0) possible_HIE = TRUE;
      }
      if (possible_HIE && possible_HID) strncpy(resName, "HIP", 3);
      else if (possible_HIE) strncpy(resName, "HIE", 3);
      else if (possible_HID) strncpy(resName, "HID", 3);
      break;
    case LYSH:
      for (i=N_LYSH_SCH_ATOMS; i<resPt->nschatoms; i++){
	aPt = resPt->schAlist[i];
	if (strncmp(aPt->name, "3HZ",4)==0) possible_LYS = TRUE;
      }
      if (!possible_LYS) strncpy(resName, "LYN", 3);
      break;
    case ASPH:
      for (i=N_ASPH_SCH_ATOMS; i<resPt->nschatoms; i++){
	aPt = resPt->schAlist[i];
	if (strncmp(aPt->name, "HD2",4)==0) possible_ASH = TRUE;
      }
      if (possible_ASH) strncpy(resName, "ASH", 3);
      break;
    case GLUH:
      for (i=N_GLUH_SCH_ATOMS; i<resPt->nschatoms; i++){
	aPt = resPt->schAlist[i];
	if (strncmp(aPt->name, "HE2",4)==0) possible_GLH = TRUE;
      }
      if (possible_GLH) strncpy(resName, "GLH", 3);
      break;
    default:
      break;
    }

  case INSIGHT:
    break;
  }

}

/**********************************************************************/

static void write_atom(FILE *fPt, atom *aPt, formatTypes format) {
  
  int int_first_char;
  char atomname[5];
  char resname[4];

  translate_psf_atom_name(aPt,atomname,format);
  translate_psf_res_name(aPt->residuePt,resname,format);
  
  fprintf(fPt,"%-6s","ATOM");
  fprintf(fPt,"%5d", aPt->amber_index);
  int_first_char = atoi(&atomname[0]);
  if((strlen(atomname) == 4) ||
     ((int_first_char > 0) && (int_first_char < 4)))
    fprintf(fPt,"%1s"," ");
  else
    fprintf(fPt,"%2s","  ");
  fprintf(fPt,"%-3s",atomname);
  if((int_first_char > 0) && (int_first_char < 4) && (atomname[3] == '\0'))
    fprintf(fPt,"%1s"," ");
  fprintf(fPt,"%4s", resname);
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

void write_residue_with_amber_order(FILE* pdboutfile, residue* resPt) {

  atom** atomList = NULL;
  int nb_atoms = 0;
  atom* aPt = NULL;
  atom* tmp_aPt = NULL;
  int i,j,k;
  int found = FALSE;

  // copy bkb and sch array
  for (i=0; i<resPt->nbkbatoms; i++) {
    aPt = resPt->bkbAlist[i];
    if (aPt!=NULL && aPt->amber_index!=AMB_NONE) {
      found = FALSE;
      j = 0;
      while( j<nb_atoms && !found) {
	if(atomList[j]->amber_index>aPt->amber_index) {
	  tmp_aPt = atomList[nb_atoms-1];
	  insert_pointer_in_list((void*)tmp_aPt,(void***)&atomList, &nb_atoms);
	  for(k=nb_atoms-2; k>j-1; k--) atomList[k+1]=atomList[k];	  
	  found = TRUE;
	}
	else {
	  j++;
	}
      }
      if (found) atomList[j] = aPt;
      else insert_pointer_in_list((void*)aPt,(void***)&atomList, &nb_atoms);   
    }
  }

  for (i=0; i<resPt->nschatoms; i++) {
    aPt = resPt->schAlist[i];
    if (aPt!=NULL && aPt->amber_index!=AMB_NONE) {
      found = FALSE;
      j = 0;
      while( j<nb_atoms && !found) {
	if(atomList[j]->amber_index>aPt->amber_index) {
	  tmp_aPt = atomList[nb_atoms-1];
	  insert_pointer_in_list((void*)tmp_aPt,(void***)&atomList, &nb_atoms);
	  for(k=nb_atoms-2; k>j-1; k--) atomList[k+1]=atomList[k];	  
	  found = TRUE;
	}
	else {
	  j++;
	}
      }
      if (found) atomList[j] = aPt;
      else insert_pointer_in_list((void*)aPt,(void***)&atomList, &nb_atoms);   
    }
  }
  
  // print list
  for (i=0; i<nb_atoms; i++) {
    write_atom(pdboutfile, atomList[i], AMBER);
  }
}


/**********************************************************************/
/**********************************************************************/
// FILE SCANNING //

static int is_amber_specific_resName(char* resName) {
  
  if (strcmp(resName, "HID")==0) return TRUE;
  if (strcmp(resName, "HIE")==0) return TRUE;
  if (strcmp(resName, "HIP")==0) return TRUE;
  if (strcmp(resName, "LYN")==0) return TRUE;
  if (strcmp(resName, "CYM")==0) return TRUE;
  if (strcmp(resName, "CYX")==0) return TRUE;
  if (strcmp(resName, "ASH")==0) return TRUE;
  if (strcmp(resName, "GLH")==0) return TRUE;
  
  return FALSE;
}

static int is_insight_specific_resName(char* resName) {
  
  if (strcmp(resName, "HIS")==0) return TRUE;
  return FALSE;
}

static int is_amber_specific_atom(char* read_resName, char* read_atomName) {

  if((strcmp(read_resName,"ASP")==0)
     ||(strcmp(read_resName,"ASN")==0)
     || (strcmp(read_resName,"CYS")==0)
     || (strcmp(read_resName,"LEU")==0)
     || (strcmp(read_resName,"PHE")==0)
     || (strcmp(read_resName,"SER")==0)
     || (strcmp(read_resName,"TRP")==0)
     || (strcmp(read_resName,"TYR")==0)) {
    if (strcmp(read_atomName,"3HB")==0) return TRUE;
  }

  if((strcmp(read_resName,"GLN")==0)
     || (strcmp(read_resName,"GLU")==0)
     || (strcmp(read_resName,"MET")==0)) {
    if (strcmp(read_atomName,"3HB")==0) return TRUE;
    if (strcmp(read_atomName,"3HG")==0) return TRUE;
  }

  if(strcmp(read_resName,"GLY")==0) {
    if (strcmp(read_atomName,"3HA")==0) return TRUE;
  }

  if(strcmp(read_resName,"ILE")==0) {
    if (strcmp(read_atomName,"3HG")==0) return TRUE;
  }

  if(strcmp(read_resName,"LYS")==0) {
    if (strcmp(read_atomName,"3HB")==0) return TRUE;
    if (strcmp(read_atomName,"3HG")==0) return TRUE;
    if (strcmp(read_atomName,"3HD")==0) return TRUE;
    if (strcmp(read_atomName,"3HE")==0) return TRUE;
  }

  if((strcmp(read_resName,"PRO")==0)
     || (strcmp(read_resName,"ARG")==0)){
    if (strcmp(read_atomName,"3HB")==0) return TRUE;
    if (strcmp(read_atomName,"3HG")==0) return TRUE;
    if (strcmp(read_atomName,"3HD")==0) return TRUE;
  }

  return FALSE;
}

static int scanLine(FILE* pdbfile, char* read_resName, char* read_atomName) {

  char *pdbline;
  char piece[10];
  char rec[10];
  char returned_s[100];
  int atomLine = FALSE;
  int empty_or_invalid = FALSE;
  
  while (!atomLine && !empty_or_invalid) {
    pdbline = fgets(returned_s,100,pdbfile);
    
    if(pdbline == NULL) {
      empty_or_invalid = TRUE;
    }
    else {
      strcpy(piece,"         ");
      strncpy(piece,pdbline,6);
      if((sscanf(piece,"%s",rec) >= 0)
	 && (strcmp(rec,"ATOM") == 0)) {
	atomLine = TRUE;
	
	strcpy(piece,"         ");
	strncpy(piece,pdbline+12,4);
	if(sscanf(piece,"%s",read_atomName) < 0) {
	  empty_or_invalid = TRUE;
	}
	else {
	  strcpy(piece,"         ");
	  strncpy(piece,pdbline+17,3);
	  if(sscanf(piece,"%s",read_resName) < 0) {
	    empty_or_invalid = TRUE;
	  }
	}
	
      }
    }
  }

  return (!empty_or_invalid);
}

static int is_ambiguous_res(char* resName) {
  if (strcmp(resName, "ALA")==0) return TRUE;
  if (strcmp(resName, "THR")==0) return TRUE;
  if (strcmp(resName, "VAL")==0) return TRUE;

  return FALSE;
}
 
int scanFile(FILE* pdbfile, formatTypes* pdb_format) {
  
  char read_resName[4];
  char resName[4];
  char read_atomName[5];
  int not_determined = TRUE;
  int not_end = TRUE;

  fseek(pdbfile,0,0);

  // first atom
  if (!scanLine(pdbfile, read_resName, read_atomName)) {
    fseek(pdbfile,0,0);
    return FALSE;
  }
  if (is_amber_specific_resName(read_resName)) {
    *pdb_format = AMBER;
    fseek(pdbfile,0,0);
    return TRUE;
  }
  else if (is_insight_specific_resName(read_resName)){
    *pdb_format = INSIGHT;
    fseek(pdbfile,0,0);
    return TRUE;
  }
  else {
    strcpy(resName, read_resName);

    do {
      if (is_amber_specific_atom(resName, read_atomName)) {
	*pdb_format = AMBER;
	not_determined = FALSE;
      }
      else {

	not_end = scanLine(pdbfile, read_resName, read_atomName);

	if (not_end && (strcmp(read_resName, resName)!=0)) {
	  
	  if (is_ambiguous_res(resName)) {

	    if (is_amber_specific_resName(read_resName)) {
	      *pdb_format = AMBER;
	      not_determined = FALSE;
	    }
	    else if (is_insight_specific_resName(read_resName)) {
	      *pdb_format = INSIGHT;
	      not_determined = FALSE;
	    }
	    else {
	      strcpy(resName, read_resName);
	    }
	
	  }
	  else{
	    *pdb_format = INSIGHT;
	    not_determined = FALSE;
	  }
	}
      }
    
    } while (not_determined && not_end);
  }

  fseek(pdbfile,0,0);

  return (!not_determined);
}
