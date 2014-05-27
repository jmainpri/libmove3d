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



/*********************************/
/* TRANSLATION PDB AMBER --> PSF */
/*********************************/

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


/*********************************/
/* TRANSLATION PSF --> PDB AMBER */
/*********************************/

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

/*********************/
/* WRITING FUNCTIONS */
/*********************/

static void write_atom(FILE *fPt, atom *aPt, formatTypes format) {
  
  int int_first_char;
  char atomname[5];
  char resname[4];

  translate_psf_atom_name(aPt,atomname,format);
  translate_psf_res_name(aPt->residuePt,resname,format);
  
  fprintf(fPt,"%-6s","ATOM");
  fprintf(fPt,"%5d", aPt->serial);
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


void write_pdb_without_H_atoms(FILE *pdboutfile, protein *protPt, formatTypes format) {

  int ic,ir,ia;
  AAchain *AAchainPt = NULL;
  residue *resPt = NULL;
  atom* aPt = NULL;

  for(ic = 0; ic < protPt->nchains; ic++) {
    AAchainPt = protPt->chainlist[ic];
    for(ir = 0; ir < AAchainPt->nresidues; ir++) {
      resPt = AAchainPt->reslist[ir];
      for(ia = 0; ia < resPt->nbkbatoms; ia++) {
	aPt = resPt->bkbAlist[ia];
	if (aPt!=NULL && aPt->atomType!=HYDROGEN_P && aPt->atomType!=HYDROGEN)
	  write_atom(pdboutfile,aPt,format);
      }
      for(ia = 0; ia < resPt->nschatoms; ia++) {
	aPt = resPt->schAlist[ia];
	if (aPt!=NULL && aPt->atomType!=HYDROGEN_P && aPt->atomType!=HYDROGEN)
	  write_atom(pdboutfile,aPt,format);
      }
    }
  }
}


/*****************/
/* FILE SCANNING */
/*****************/


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
