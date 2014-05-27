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

//#include "protein.h"


#define MAX_NINTERVALS 3

typedef struct s_realinterval {
  double vmin;
  double vmax;
} realinterval;


/**********************************************************************
 *           Articulated Peptide Chain (APC) structures                 *
 **********************************************************************/

typedef struct s_apc_atom {
  vector4 posrelframe;
  struct s_atom *atomPt;  
} apc_atom;


typedef struct s_apc_rigid {
  // atoms
  int natoms;
  struct s_apc_atom **atomlist;
  // dihedral angle
  double qi;
  int nintervals;
  struct s_realinterval q_interv[MAX_NINTERVALS];
  // frame relative to prev rigid when qi=0
  matrix4 pT0;  // constant 
  // frame relative to next rigid when qi=0
  matrix4 nT0;  // = inv(pT0) , constant 
  // frame relative to pT0 
  matrix4 Ti;   // Ti is only function of qi (NECESSARY ???)
  // frame in abs ref
  matrix4 Tabs;
  struct s_apc_rigidlistelem *inlist;
} apc_rigid;


typedef struct s_apc_rigidlistelem {
  struct s_apc_rigid *rigid;
  int nnext;
  struct s_apc_rigid **next;
  struct s_apc_rigid *prev;
} apc_rigidlistelem;


typedef struct s_apc_backbone {
  int nrigids;
  struct s_apc_rigidlistelem *firstrigid;
  struct s_apc_rigidlistelem *lastrigid;
  matrix4 Tbase;
  matrix4 Tend;
} apc_backbone;


typedef struct s_apc_sidechain {
  int nrigids;
  struct s_apc_rigidlistelem *firstrigid;
  struct s_apc_rigid *refbkbrigid;
  //matrix4 Tbase;
} apc_sidechain;


typedef struct s_apc {
  struct s_apc_backbone *backbone;
  int nsidechains;
  struct s_apc_sidechain **sidechains;
} apc;

