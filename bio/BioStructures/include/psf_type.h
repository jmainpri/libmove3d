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
#ifndef PSF_TYPE
#define PSF_TYPE

#include "psf_defs_bio.h"

#define PSF_MAX_NAME_LENGTH 30

/**********************************************************************
 *                Protein Structure Format (PSF)                      *
 **********************************************************************/

typedef struct s_psf_atom {
  struct s_psf_residue *residuePt;
  int serial;
  char name[5];
  atomTypes atomType;
  double pos[3];
  double vdwR;
  int nbondedA;
  struct s_psf_atom **bondedAtomList;
  struct s_psf_rigid *rigidPt;
} psf_atom;


// PROTEIN

typedef struct s_psf_residue {
  struct s_psf_AAchain *chainPt;
  int resSeq;
  char resName[4];
  residueTypes resType;
  int flagH;      // indicates if it contains hydrogen atoms
  int flagCterm;  // indicates if it is a C terminal residue with OXT
  // NOTE : use pseudo backbone and side-chain
  int nbkbatoms;
  int nschatoms;
  // NOTE : atoms in lists have precise order (defined for BCD)
  struct s_psf_atom **bkbAtomList;
  struct s_psf_atom **schAtomList;
  struct s_psf_residue *nextResidue;
  struct s_psf_residue *prevResidue;
} psf_residue;


typedef struct s_psf_AAchain {
  struct s_psf_protein *proteinPt;
  char chainID[2];
  int subchainind;
  int nresidues;
  struct s_psf_residue **resList;
} psf_AAchain;


typedef struct s_psf_protein {
  char name[PSF_MAX_NAME_LENGTH];
  int nchains;
  struct s_psf_AAchain **chainList;
} psf_protein;


// LIGAND

typedef struct s_psf_rigid {
  int natoms;
  int noutJnts;
  int ninJnts;
  struct s_psf_atom **atomList;
  struct s_psf_joint **outJntsList;
  struct s_psf_joint **inJntsList;
  int indexparentj;
} psf_rigid;

typedef struct s_psf_joint {
  struct s_psf_atom *dihedralAtoms[4];
  double vmin;
  double vmax;
  double value;
  struct s_psf_rigid *rigidIn;
  struct s_psf_rigid *rigidOut;
} psf_joint;


typedef struct s_psf_ligand {
  char name[PSF_MAX_NAME_LENGTH];
  int natoms;
  int nrigids;
  int njoints;
  struct s_psf_atom **atomList;
  struct s_psf_rigid **rigidList;
  struct s_psf_joint **jointList;
  double oref[3];
} psf_ligand;

// MOLECULE

typedef struct s_psf_molecule {
  char name[PSF_MAX_NAME_LENGTH];
  int nproteins;
  int nligands;
  struct s_psf_protein** proteinList;
  struct s_psf_ligand** ligandList;
} psf_molecule;

#endif
