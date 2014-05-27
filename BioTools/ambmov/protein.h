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
#ifndef PROTEIN_H
#define PROTEIN_H

#define MAX_NAME_LENGTH 25


typedef enum {
  ALA, ARG, ASN, ASP, CYS, GLN, GLU, GLY, HIS, ILE, 
  LEU, LYS, MET, PHE, PRO, SER, THR, TRP, TYR, VAL,
  ALAH, ARGH, ASNH, ASPH, CYSH, GLNH, GLUH, GLYH, HISH, ILEH, 
  LEUH, LYSH, METH, PHEH, PROH, SERH, THRH, TRPH, TYRH, VALH 
} residueTypes;

typedef enum {
 SULPHUR, SULPHUR_H, OXYGEN, OXYGEN_H, NITROGEN, NITROGEN_H, NITROGEN_FULL,
 CARBON, HYDROGEN, HYDROGEN_P, BROMINE, IODINE, FLUORINE, PHOSPHORUS, CHLORINE
} atomTypes;


/**********************************************************************
 *                Protein Structure Format (PSF)                      *
 **********************************************************************/

typedef struct s_atom {
  struct s_residue *residuePt;
  int serial;
  char name[5];
  atomTypes atomType;
  double pos[3];
  double vdwR;
  int nbondedA;
  struct s_atom **bondedAlist;
} atom;


typedef struct s_residue {
  struct s_AAchain *chainPt;
  char chainID[2];
  int subchainind;
  int resSeq;
  char resName[4];
  residueTypes resType;
  int flagH;    // indicates if it contains hydrogen atoms
  int flagCterm;// indicates if it is a C terminal residue with OXT
  int flagNterm;// indicates if it is a N terminal residue with 3 H atoms
  // NOTE : use pseudo backbone and side-chain
  int nbkbatoms;
  int nschatoms;
  // NOTE : atoms in lists have precise order (defined for BCD)
  struct s_atom **bkbAlist;
  struct s_atom **schAlist;
  struct s_residue *next;
  struct s_residue *prev;
} residue;


typedef struct s_AAchain {
  struct s_protein *proteinPt;
  int nresidues;
  struct s_residue **reslist;
  //struct s_AAchain *next;
} AAchain;


typedef struct s_protein {
  char name[MAX_NAME_LENGTH];
  int nchains;
  struct s_AAchain **chainlist;
} protein;

#endif
