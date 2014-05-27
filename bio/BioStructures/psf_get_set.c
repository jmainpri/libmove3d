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


#include "include/psf_defs_bio.h"
#include "include/psf_type.h"

#include "P3d-pkg.h"
#include "Bio-pkg.h"

//////// CONSTRUCTORS ////////////////////////////////////////////////////

// Bio Entities

psf_molecule* init_molecule_struct(char* name) {
  
  psf_molecule* molecule = (psf_molecule*)malloc(sizeof(psf_molecule));
  strncpy(molecule->name, name, PSF_MAX_NAME_LENGTH-1);
  molecule->nproteins = 0;
  molecule->proteinList = NULL;
  molecule->nligands = 0;
  molecule->ligandList = NULL;

  return molecule;
}

/**********************************************************************/

// Protein

psf_protein* init_protein_struct(char* name) {
  
  psf_protein* protPt = (psf_protein*)malloc(sizeof(psf_protein));
  strncpy(protPt->name, name, PSF_MAX_NAME_LENGTH-1);
  protPt->nchains = 0;
  protPt->chainList = NULL;

  return protPt;
}

/**********************************************************************/

// Ligand

psf_ligand* init_ligand_struct(char* fullName) {
  
  int i;

  psf_ligand* ligPt = (psf_ligand*)malloc(sizeof(psf_ligand));
  strncpy(ligPt->name, fullName, PSF_MAX_NAME_LENGTH-1);
  ligPt->natoms = 0;
  ligPt->nrigids = 0;
  ligPt->njoints = 0;
  ligPt->atomList = NULL;
  ligPt->rigidList = NULL;
  ligPt->jointList = NULL;
  for (i=0; i<3; i++)
    ligPt->oref[i] = 0.0;

  return ligPt;
}


/**********************************************************************/

psf_joint* init_joint_struct() {
  
  int i;
  psf_joint* joint;

  joint = (psf_joint*)malloc(sizeof(psf_joint));
  for (i=0; i<4; i++) {
    joint->dihedralAtoms[i] = NULL;
  }
  joint->vmin = -180.0;
  joint->vmax = 180.0;
  joint->value = 0.0;
  joint->rigidIn = NULL;
  joint->rigidOut = NULL;

  return joint;

}

/**********************************************************************/

psf_rigid* init_rigid_struct()
{
  psf_rigid* rigidPt;
  
  rigidPt = (psf_rigid *) malloc(sizeof(psf_rigid));

  rigidPt->natoms = 0;
  rigidPt->noutJnts = 0;
  rigidPt->ninJnts = 0;
  rigidPt->atomList = NULL;
  rigidPt->outJntsList = NULL;
  rigidPt->inJntsList = NULL;
  rigidPt->indexparentj = 0;

  return rigidPt;
}

//////// DESTRUCTORS ////////////////////////////////////////////////////

// Protein

/* void free_protein(psf_protein *protPt) */
/* { */
/*   int numChain; */
/*   AAchain* chain; */
  
/*   int numResidue; */
/*   residue* res; */

/*   int numAtoms; */
/*   atom* atPt; */

/*   for (numChain=0; numChain<protPt->nchains; numChain++) { */
/*     chain=protPt->chainlist[numChain]; */
    
/*     for (numResidue=0; numResidue<chain->nresidues; numResidue++) { */
/*       res = chain->reslist[numResidue]; */
      
/*       for(numAtoms=0; numAtoms<res->nbkbatoms; numAtoms++) { */
/* 	atPt = res->bkbAlist[numAtoms]; */
/* 	remove_all_pointers_from_list((void***)&(atPt->bondedAlist), &(atPt->nbondedA)); */
/* 	free(atPt->bondedAlist); */
/*       } */

/*       for(numAtoms=0; numAtoms<res->nschatoms; numAtoms++) { */
/* 	atPt = res->schAlist[numAtoms]; */
/* 	remove_all_pointers_from_list((void***)&(atPt->bondedAlist), &(atPt->nbondedA)); */
/* 	free(atPt->bondedAlist);	 */
/*       } */

/*       free_pointer_list((void ***)&(res->bkbAlist), &(res->nbkbatoms)); */
/*       free_pointer_list((void ***)&(res->schAlist), &(res->nschatoms)); */

/*     } */

/*     free_pointer_list((void***)&(chain->reslist), &(chain->nresidues)); */
/*   } */
  
/*   free_pointer_list((void***)&(protPt->chainlist), &(protPt->nchains)); */
  
/* } */

static void free_atom(psf_atom *atPt) {
  free_pointer_list((void***)&(atPt->bondedAtomList), &(atPt->nbondedA));
}

static void free_residue(psf_residue* resPt) {

  int numAtoms;
  psf_atom* atPt;

  // unbind
  for(numAtoms=0; numAtoms<resPt->nbkbatoms; numAtoms++) {
    atPt = resPt->bkbAtomList[numAtoms];
    free_atom(atPt);
  }
  
  for(numAtoms=0; numAtoms<resPt->nschatoms; numAtoms++) {
    atPt = resPt->schAtomList[numAtoms];
    free_atom(atPt);
  }

  // free atoms
  for(numAtoms=0; numAtoms<resPt->nbkbatoms; numAtoms++) {
    atPt = resPt->bkbAtomList[numAtoms];
    free(atPt);
    atPt = NULL;
  }
  
  for(numAtoms=0; numAtoms<resPt->nschatoms; numAtoms++) {
    atPt = resPt->schAtomList[numAtoms];
    free(atPt);
    atPt = NULL;
  }
  
  // free lists
  free_pointer_list((void ***)&(resPt->bkbAtomList), &(resPt->nbkbatoms));
  free_pointer_list((void ***)&(resPt->schAtomList), &(resPt->nschatoms));

}

static void free_AAchain(psf_AAchain* chainPt) {
  
  int numResidue;
  psf_residue* resPt;

    for (numResidue=0; numResidue<chainPt->nresidues; numResidue++) {
      resPt = chainPt->resList[numResidue];
      free_residue(resPt);
    }

    for (numResidue=0; numResidue<chainPt->nresidues; numResidue++) {
      resPt = chainPt->resList[numResidue];
      free(resPt);
      resPt = NULL;
    }

    free_pointer_list((void ***)&(chainPt->resList), &(chainPt->nresidues));
}

void free_protein(psf_protein* protPt) {
  
  int numChain;
  psf_AAchain* chainPt;

  for (numChain=0; numChain<protPt->nchains; numChain++) {
    chainPt=protPt->chainList[numChain];
    free_AAchain(chainPt);
  }

  for (numChain=0; numChain<protPt->nchains; numChain++) {
    chainPt=protPt->chainList[numChain];
    free(chainPt);
    chainPt = NULL;
  }
  
  free_pointer_list((void ***)&(protPt->chainList), &(protPt->nchains));
}


/**********************************************************************/

// Ligand

static void free_rigid(psf_rigid *rigidPt) {
  
  free_pointer_list((void***)&(rigidPt->atomList), &(rigidPt->natoms));
  free_pointer_list((void***)&(rigidPt->outJntsList), &(rigidPt->noutJnts));
  free_pointer_list((void***)&(rigidPt->inJntsList), &(rigidPt->ninJnts));

}

void free_ligand(psf_ligand *ligPt) {

  int numRigids;
  int numJoint;
  int numAtoms;
  psf_rigid* rigidPt;
  psf_joint* jointPt;
  psf_atom* atPt;

  for(numRigids=0; numRigids<ligPt->nrigids; numRigids++) {
    rigidPt = ligPt->rigidList[numRigids];
    free_rigid(rigidPt);
  }
  for(numRigids=0; numRigids<ligPt->nrigids; numRigids++) {
    rigidPt = ligPt->rigidList[numRigids];
    free(rigidPt);
    rigidPt = NULL;
  }
  free_pointer_list((void ***)&(ligPt->rigidList), &(ligPt->nrigids));
  
  for(numJoint=0; numJoint<ligPt->njoints; numJoint++) {
    jointPt = ligPt->jointList[numJoint];
    free(jointPt);
    jointPt = NULL;
  }
  free_pointer_list((void ***)&(ligPt->jointList), &(ligPt->njoints));
  
  for(numAtoms=0; numAtoms<ligPt->natoms; numAtoms++) {
    atPt = ligPt->atomList[numAtoms];
    free_atom(atPt);
  }
  for(numAtoms=0; numAtoms<ligPt->natoms; numAtoms++) {
    atPt = ligPt->atomList[numAtoms];
    free(atPt);
    atPt = NULL;
  }
  free_pointer_list((void ***)&(ligPt->atomList), &(ligPt->natoms));

}

/**********************************************************************/

// Molecule

void free_molecule(psf_molecule *moleculePt) {
  
  int numProt;
  int numLig;
  psf_protein* protPt;
  psf_ligand* ligPt;

  for(numProt=0; numProt<moleculePt->nproteins; numProt++) {
    protPt = moleculePt->proteinList[numProt];
    free_protein(protPt);
  }
  for(numProt=0; numProt<moleculePt->nproteins; numProt++) {
    protPt = moleculePt->proteinList[numProt];
    free(protPt);
    protPt = NULL;
  }
  free_pointer_list((void***)&(moleculePt->proteinList), &(moleculePt->nproteins));

  for(numLig=0; numLig<moleculePt->nligands; numLig++) {
    ligPt = moleculePt->ligandList[numLig];
    free_ligand(ligPt);
  }
  for(numLig=0; numLig<moleculePt->nligands; numLig++) {
    ligPt = moleculePt->ligandList[numLig];
    free(ligPt);
    ligPt = NULL;
  }
  free_pointer_list((void***)&(moleculePt->ligandList), &(moleculePt->nligands));

}

//////// GET ////////////////////////////////////////////////////

// FUNCTIONS TO GET BKB ATOMS

psf_atom *get_N(psf_residue *resPt)
{
  switch(resPt->resType) {
  case psf_GLY: 
    return resPt->bkbAtomList[psf_GLY_N];
    break;
  case psf_GLYH: 
    return resPt->bkbAtomList[psf_GLYH_N];
    break;
  case psf_PRO: 
    return resPt->bkbAtomList[psf_PRO_N];
    break;
  case psf_PROH: 
    return resPt->bkbAtomList[psf_PROH_N];
    break;
  default:
    if(resPt->flagH == 0)
      return resPt->bkbAtomList[psf_gen_N];
    else
      return resPt->bkbAtomList[psf_genH_N];
  }
}

/**********************************************************************/

psf_atom *get_CA(psf_residue *resPt)
{
  switch(resPt->resType) {
  case psf_GLY: 
    return resPt->bkbAtomList[psf_GLY_CA];
    break;
  case psf_GLYH: 
    return resPt->bkbAtomList[psf_GLYH_CA];
    break;
  case psf_PRO: 
    return resPt->bkbAtomList[psf_PRO_CA];
    break;
  case psf_PROH: 
    return resPt->bkbAtomList[psf_PROH_CA];
    break;
  default:
    if(resPt->flagH == 0)
      return resPt->bkbAtomList[psf_gen_CA];
    else
      return resPt->bkbAtomList[psf_genH_CA];
  }
}

/**********************************************************************/

psf_atom *get_C(psf_residue *resPt)
{
  switch(resPt->resType) {
  case psf_GLY: 
    return resPt->bkbAtomList[psf_GLY_C];
    break;
  case psf_GLYH: 
    return resPt->bkbAtomList[psf_GLYH_C];
    break;
  case psf_PRO: 
    return resPt->bkbAtomList[psf_PRO_C];
    break;
  case psf_PROH: 
    return resPt->bkbAtomList[psf_PROH_C];
    break;
  default:
    if(resPt->flagH == 0)
      return resPt->bkbAtomList[psf_gen_C];
    else
      return resPt->bkbAtomList[psf_genH_C];
  }
}

/**********************************************************************/

// FUNCTIONS EXTRACTING GEOMETRIC INFORMATION FORM PROTEIN STRUCTURE

void get_N_pos(psf_residue *resPt, double **apos)
{
  switch(resPt->resType) {
  case psf_GLY: 
    *apos = resPt->bkbAtomList[psf_GLY_N]->pos;
    break;
  case psf_GLYH: 
    *apos = resPt->bkbAtomList[psf_GLYH_N]->pos;
    break;
  case psf_PRO: 
    *apos = resPt->bkbAtomList[psf_PRO_N]->pos;
    break;
  case psf_PROH: 
    *apos = resPt->bkbAtomList[psf_PROH_N]->pos;
    break;
  default:
    if(resPt->flagH == 0)
      *apos = resPt->bkbAtomList[psf_gen_N]->pos;
    else
      *apos = resPt->bkbAtomList[psf_genH_N]->pos;
  }
}

/**********************************************************************/

void get_CA_pos(psf_residue *resPt, double **apos)
{
  switch(resPt->resType) {
  case psf_GLY: 
    *apos = resPt->bkbAtomList[psf_GLY_CA]->pos;
    break;
  case psf_GLYH: 
    *apos = resPt->bkbAtomList[psf_GLYH_CA]->pos;
    break;
  case psf_PRO: 
    *apos = resPt->bkbAtomList[psf_PRO_CA]->pos;
    break;
  case psf_PROH: 
    *apos = resPt->bkbAtomList[psf_PROH_CA]->pos;
    break;
  default:
    if(resPt->flagH == 0)
      *apos = resPt->bkbAtomList[psf_gen_CA]->pos;
    else
      *apos = resPt->bkbAtomList[psf_genH_CA]->pos;
  }
}

/**********************************************************************/

void get_C_pos(psf_residue *resPt, double **apos)
{
  switch(resPt->resType) {
  case psf_GLY: 
    *apos = resPt->bkbAtomList[psf_GLY_C]->pos;
    break;
  case psf_GLYH: 
    *apos = resPt->bkbAtomList[psf_GLYH_C]->pos;
    break;
  case psf_PRO: 
    *apos = resPt->bkbAtomList[psf_PRO_C]->pos;
    break;
  case psf_PROH: 
    *apos = resPt->bkbAtomList[psf_PROH_C]->pos;
    break;
  default:
    if(resPt->flagH == 0)
      *apos = resPt->bkbAtomList[psf_gen_C]->pos;
    else
      *apos = resPt->bkbAtomList[psf_genH_C]->pos;
  }
}

/**********************************************************************/

// RESIDUE

int psf_get_res_in_list_by_resSeq(int resSeq, psf_residue** theRes, psf_residue **resList, int nresidues) {

  int i = 0;
  int found = FALSE;
  psf_residue* current_res = NULL;

  while (i<nresidues && !found) {
    current_res = resList[i];
    if (current_res->resSeq == resSeq) {
      found = TRUE;
      *theRes = current_res;
    }
    else {
      i++;
    }  
  }

    return found;
}

/**********************************************************************/

// ATOMS 

int psf_get_atom_in_list_by_serial(int serial, psf_atom** theAtom, psf_atom **atomList, int natoms)
{
  int i = 0;
  int found = FALSE;

  while (i<natoms && !found) {
    if(atomList[i]->serial == serial) {
      *theAtom = atomList[i];
      found = TRUE;
    }
    else i++;
  }
  
  return found;
}

/**********************************************************************/

int psf_get_atom_in_list_by_name(int resSeq, char* atomName, psf_atom** theAtom, psf_residue **resList, int nresidues)
{
  int atomIndex;
  psf_residue* theRes;
  atomTypes type;
  residueTypes resType;

  if (!psf_get_res_in_list_by_resSeq(resSeq, &theRes, resList, nresidues))
    return FALSE;

  resType = theRes->resType;
  
  // Backbone search
  atomIndex = atomName_to_bkbatomindex(atomName, &type, resType);
  if (atomIndex>=0 && theRes->bkbAtomList[atomIndex]!=NULL) {
    // SideChain search
    *theAtom = theRes->bkbAtomList[atomIndex];
  }
  else { 
    atomIndex = atomName_to_schatomindex(atomName, &type, resType);
    if (atomIndex>=0 && theRes->schAtomList[atomIndex]!=NULL) {
      *theAtom = theRes->schAtomList[atomIndex];
    }
    else
      return FALSE;
  }

  return TRUE;
}

/**********************************************************************/

int psf_get_atom_pos_by_serial(psf_protein* protein, int serial ,double* pos) {
  
  int found = FALSE;
  int index;
  int chainIndex = 0;
  int resIndex = 0;
  int nresidues;
  psf_residue** resList = NULL;
  int atomIndex;
  int natoms;
  psf_atom** atomList = NULL;
  psf_atom* theAtom;
  
  while (chainIndex<protein->nchains && !found) {
    
    nresidues = protein->chainList[chainIndex]->nresidues;
    resList = protein->chainList[chainIndex]->resList;
    
    while (resIndex<nresidues && !found) {
      
      // Backbone search
      atomIndex = 0;
      natoms = resList[resIndex]->nbkbatoms;
      atomList = resList[resIndex]->bkbAtomList;
      found = psf_get_atom_in_list_by_serial(serial, &theAtom, atomList, natoms);
      
      if (!found) {
	// SideChain search
	atomIndex = 0;
	natoms = resList[resIndex]->nschatoms;
	atomList = resList[resIndex]->schAtomList;
	found = psf_get_atom_in_list_by_serial(serial, &theAtom, atomList, natoms);
	
	if (!found)
	  resIndex++;
      }
    }
  }  
   
  if (found) {
    for (index=0; index<3; index++) {
      pos[index] = theAtom->pos[index];
    }
  }
  
  return found;
}

/**********************************************************************/

int psf_get_atom_pos_by_name(psf_protein* protein, int resSeq, char* atomName, double* pos) {
  
  int found = FALSE;
  int index;
  int chainIndex = 0;
  int nresidues;
  psf_residue** resList = NULL;
  psf_atom* theAtom;
  
  while (chainIndex<protein->nchains && !found) {
    
    nresidues = protein->chainList[chainIndex]->nresidues;
    resList = protein->chainList[chainIndex]->resList;

    found = psf_get_atom_in_list_by_name(resSeq, atomName, &theAtom, resList, nresidues);
   
    if (!found)
      chainIndex++;
  }
  
  if (found) {
    for (index=0; index<3; index++) {
      pos[index] = theAtom->pos[index];
    }
  }
  
  return found;
}

/**********************************************************************/

int psf_get_atom_type_by_serial(psf_protein* protein, int serial , atomTypes* type) {
  
  int found = FALSE;
  int chainIndex = 0;
  int resIndex = 0;
  int nresidues;
  psf_residue** resList = NULL;
  int atomIndex;
  int natoms;
  psf_atom** atomList = NULL;
  psf_atom* theAtom;
  
  while (chainIndex<protein->nchains && !found) {
    
    nresidues = protein->chainList[chainIndex]->nresidues;
    resList = protein->chainList[chainIndex]->resList;
    
    while (resIndex<nresidues && !found) {
      
      // Backbone search
      atomIndex = 0;
      natoms = resList[resIndex]->nbkbatoms;
      atomList = resList[resIndex]->bkbAtomList;
      found = psf_get_atom_in_list_by_serial(serial, &theAtom, atomList, natoms);
      
      if (!found) {
	// SideChain search
	atomIndex = 0;
	natoms = resList[resIndex]->nschatoms;
	atomList = resList[resIndex]->schAtomList;
	found = psf_get_atom_in_list_by_serial(serial, &theAtom, atomList, natoms);
	
	if (!found)
	  resIndex++;
      }
    }
  }  
   
  if (found) {
    *type = theAtom->atomType;
  }
  
  return found;
}

/**********************************************************************/

int psf_get_atom_type_by_name(psf_protein* protein, int resSeq, char* atomName, int* type) {
  
  int found = FALSE;
  int chainIndex = 0;
  int nresidues;
  psf_residue** resList = NULL;
  psf_atom* theAtom;
  
  while (chainIndex<protein->nchains && !found) {
    
    nresidues = protein->chainList[chainIndex]->nresidues;
    resList = protein->chainList[chainIndex]->resList;

    found = psf_get_atom_in_list_by_name(resSeq, atomName, &theAtom, resList, nresidues);
   
    if (!found)
      chainIndex++;
  }
  
  if (found) {
    *type = theAtom->atomType;
  }
  
  return found;
}



/**********************************************************************/

int is_a_defined_serial(int serial, psf_atom **atomList, int natoms) {

  int i = 0;
  int found = FALSE;

  while (i<natoms && !found) {
    if(atomList[i]->serial == serial) {
      found = TRUE;
    }
    else i++;
  }

  return found;

}

//////// SET ////////////////////////////////////////////////////

