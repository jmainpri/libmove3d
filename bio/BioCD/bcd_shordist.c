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
/*******************************************************/
/*********************** bcd_init.c *******************/
/*******************************************************/

/*++++ SHORT TOPOLOGICAL DISTANCE RELATED FUNCTIONS++++ */
/*   ++++++++++++++++++++++++++++++++++++++++++++++++++ */
/*   ++++++++++++++++++++++++++++++++++++++++++++++++++++ 

The material here can be grouped in:

- Permanent data. Definition of constant data like arrays of different kinds of
topological distances of residuals parts indexed by type of residual.
Other permanent data are arrays of ordering functions
and arrays of specific, optimized short-distance interaction functions.
- Functions creating the short topological distance long-term 
structures associated to a residual. The higher level function of this group is
create_residual_SDs, which creates all the short-distance structures of a 
residual by calling to all the other functions.

	*/

#include "P3d-pkg.h"
#include "Bio-pkg.h"	

//#include "include/bcd_init.h"

#include "include/bcd_global.h"
	
	
static short int sc_free, bb_free, prev_bb_free,loop;
static short int new_disulphur; //indicates if this is the first cysteine of the disulphure link
/* current (last) rigid cotaining backbones . Rigids that
contain only a side cahin are not put in this variable */
static Rigid_structure *rigid=NULL;
static Rigid_structure *prev_rigid=NULL; /* previous rigid including backbones*/
static Rigid_structure *sc_rigid=NULL;/* last rigid incluiding a side chain*/
static SDpt *inter_bb, *intra_res, *loops;
static int autocol_index, intercol_index, loop_index;
static Bboxpt last_bb_box, prev_bb_box;
static Bboxpt last_sc_box;
static int disulphur_index;

/* topological ranks for a group of atoms with respect to another structure  
The first element of each vector is the number of atoms 
at topological distance 1 from the other structure.
The second element of each vector is the number of atoms
at topological distance 2 from the other structure.
The third element of each vector is the number of atoms
at topological distance 3 from the other structure.
The fourth element of each vector is the number of atoms
at topological distance 4 or more from the other structure.
*/
typedef int topol_rank[4];


/* topological ranks for the pseudo side chains 
the first element of each vector is the number of atoms of the pseudo side chain
at topological distance 1 from the pseudo backbone.
The second element of each vector is the number of atoms of the pseudo side chain
at topological distance 2 from the pseudo backbone.
The third element of each vector is the number of atoms of the pseudo side chain
at topological distance 3 from the pseudo backbone.
The fourth element of each vector is the number of atoms of the pseudo side chain
at topological distance 4 or more from the pseudo backbone.

*/
static topol_rank sc_rank[]={
{0,0,0,0},  //ALA, not used
{1,1,1,3},  //ARG
{1,2,0,0},  //ASN
{1,2,0,0},  //ASP
{1,0,0,0},  //CYS
{1,1,2,0},  //GLN
{1,1,2,0},  //GLU
{0,0,0,0},  //GLY, not used
{1,2,2,0},  //HIS
{2,1,0,0},  //ILE
{1,2,0,0},  //LEU
{1,1,1,1},  //LYS
{1,1,1,0},  //MET
{1,2,2,1},  //PHE
{0,0,0,0},  //PRO , not used
{1,0,0,0},  //SER
{2,0,0,0},  //THR
{1,2,3,3},  //TRP
{1,2,2,2},  //TYR
{2,0,0,0},  //VAL

{3,0,0,0},  //ALAH, not used
{3,3,3,3},  //ARGH
{3,2,0,0},  //ASNH
{3,2,0,0},  //ASPH
{3,0,0,0},  //CYSH
{3,3,2,0},  //GLNH
{3,3,2,0},  //GLUH
{0,0,0,0},  //GLYH, not used
{3,2,3,1},  //HISH  
{3,6,3,0},  //ILEH
{3,3,6,0},  //LEUH
{3,3,3,3},  //LYSH
{3,3,1,3},  //METH
{3,2,4,4},  //PHEH
{0,0,0,0},  //PROH, not used
{3,0,0,0},  //SERH
{3,3,0,0},  //THRH
{3,2,4,7},  //TRPH
{3,2,4,4},  //TYRH
{3,6,0,0}  //VALH
};

/* distances of the pseudo backbone atoms to the next pseudo backbone*/
static topol_rank to_next_rank[]={
{1,2,2,0},  //ALA
{1,2,2,0},  //ARG
{1,2,2,0},  //ASN
{1,2,2,0},  //ASP
{1,2,2,0},  //CYS
{1,2,2,0},  //GLN
{1,2,2,0},  //GLU
{1,2,1,0},  //GLY *
{1,2,2,0},  //HIS
{1,2,2,0},  //ILE
{1,2,2,0},  //LEU
{1,2,2,0},  //LYS
{1,2,2,0},  //MET
{1,2,2,0},  //PHE
{1,2,2,2},  //PRO *
{1,2,2,0},  //SER
{1,2,2,0},  //THR
{1,2,2,0},  //TRP
{1,2,2,0},  //TYR
{1,2,2,0},  //VAL

{1,2,3,0},  //ALAH
{1,2,3,0},  //ARGH
{1,2,3,0},  //ASNH
{1,2,3,0},  //ASPH
{1,2,3,0},  //CYSH
{1,2,3,0},  //GLNH
{1,2,3,0},  //GLUH
{1,2,3,0},  //GLYH
{1,2,3,0},  //HISH  
{1,2,3,0},  //ILEH
{1,2,3,0},  //LEUH
{1,2,3,0},  //LYSH
{1,2,3,0},  //METH
{1,2,3,0},  //PHEH
{1,2,3,8},  //PROH *
{1,2,3,0},  //SERH
{1,2,3,0},  //THRH
{1,2,3,0},  //TRPH
{1,2,3,0},  //TYRH
{1,2,3,0}   //VALH
};

/* distances of the pseudo backbone atoms to the precedent pseudo backbone*/
static topol_rank to_prev_rank[]={
{1,1,2,1},  //ALA
{1,1,2,1},  //ARG
{1,1,2,1},  //ASN
{1,1,2,1},  //ASP
{1,1,2,1},  //CYS
{1,1,2,1},  //GLN
{1,1,2,1},  //GLU
{1,1,1,1},  //GLY *
{1,1,2,1},  //HIS
{1,1,2,1},  //ILE
{1,1,2,1},  //LEU
{1,1,2,1},  //LYS
{1,1,2,1},  //MET
{1,1,2,1},  //PHE
{1,2,3,1},  //PRO *
{1,1,2,1},  //SER
{1,1,2,1},  //THR
{1,1,2,1},  //TRP
{1,1,2,1},  //TYR
{1,1,2,1},  //VAL

{1,1,3,1},  //ALAH
{1,1,3,1},  //ARGH
{1,1,3,1},  //ASNH
{1,1,3,1},  //ASPH
{1,1,3,1},  //CYSH
{1,1,3,1},  //GLNH
{1,1,3,1},  //GLUH
{1,1,3,1},  //GLYH
{1,1,3,1},  //HISH  
{1,1,3,1},  //ILEH
{1,1,3,1},  //LEUH
{1,1,3,1},  //LYSH
{1,1,3,1},  //METH
{1,1,3,1},  //PHEH
{1,2,6,5},  //PROH *
{1,1,3,1},  //SERH
{1,1,3,1},  //THRH
{1,1,3,1},  //TRPH
{1,1,3,1},  //TYRH
{1,1,3,1}   //VALH
};

/* distances of the pseudo backbone atoms to the side chain*/
static topol_rank to_sc_rank[]={

{1,1,2,1},  //ALA Not used
{1,1,2,1},  //ARG
{1,1,2,1},  //ASN
{1,1,2,1},  //ASP
{1,1,2,1},  //CYS
{1,1,2,1},  //GLN
{1,1,2,1},  //GLU
{1,2,1,1},  //GLY Not used
{1,1,2,1},  //HIS
{1,1,2,1},  //ILE
{1,1,2,1},  //LEU
{1,1,2,1},  //LYS
{1,1,2,1},  //MET
{1,1,2,1},  //PHE
{1,1,2,1},  //PRO Not used
{1,1,2,1},  //SER
{1,1,2,1},  //THR
{1,1,2,1},  //TRP
{1,1,2,1},  //TYR
{1,1,2,1},  //VAL

{1,1,3,1},  //ALAH
{1,1,3,1},  //ARGH
{1,1,3,1},  //ASNH
{1,1,3,1},  //ASPH
{1,1,3,1},  //CYSH
{1,1,3,1},  //GLNH
{1,1,3,1},  //GLUH
{1,1,3,1},  //GLYH Not used
{1,1,3,1},  //HISH  
{1,1,3,1},  //ILEH
{1,1,3,1},  //LEUH
{1,1,3,1},  //LYSH
{1,1,3,1},  //METH
{1,1,3,1},  //PHEH
{1,1,3,1},  //PROH Not used
{1,1,3,1},  //SERH
{1,1,3,1},  //THRH
{1,1,3,1},  //TRPH
{1,1,3,1},  //TYRH
{1,1,3,1}   //VALH
};


/*given a rank vector of topological distances, returns which will
the resulting rank vector if the corresponding atoms are moved away
a topological distance of d */

static int *shift_topo(int d, int *rank){
static int res[4];
int i;


for (i=0; i < d;i++)
	res[i]= 0;

for  (i=d; i < 3;i++)
	res[i] = rank[i - d];
	
res[3]=0;
for  (i= 4 - 1 - d; i < 4;i++)
	res[3] += rank[i];
return res;
}


/* functions that returns an order of the pseudo backbone
atoms with increasing topological distance to the 
previous backbone  */
typedef  p3d_poly **(*reordering_function) (p3d_poly **);



/* function for usual or common residuals (without hydrogens)
 that returns an order
of pseudo backbone atoms with increasing topological distance to the 
previous backbone */
static p3d_poly  **usual_post_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 5 * sizeof(p3d_poly *));
res[0]= input[4];
res[1]= input[2];
res[2]= input[3];
res[3]= input[0];
res[4]= input[1];
return res;
}

/* function for usual or common residuals (with hydrogens)
 that returns an order
of pseudo backbone atoms with increasing topological distance to the 
previous backbone */
static p3d_poly **usualH_post_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 6 * sizeof(p3d_poly *));
res[0]= input[5];
res[1]= input[2];
res[2]= input[3];
res[3]= input[4];
res[4]= input[0];
res[5]= input[1];
return res;
}

/* functions that returns an order of the pseudo backbone
atoms of Glycine with increasing topological distance to the 
previous backbone */
static p3d_poly **gly_post_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 4 * sizeof(p3d_poly *));
res[0]= input[3];
res[1]= input[2];
res[2]= input[0];
res[3]= input[1];
return res;
}


/* functions that returns an order of the pseudo backbone
atoms of Proline with increasing topological distance to the 
previous backbone */
static p3d_poly **pro_post_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 7 * sizeof(p3d_poly *));
res[0]= input[4];
res[1]= input[2];
res[2]= input[6];
res[3]= input[3];
res[4]= input[5];
res[5]= input[0];
res[6]= input[1];
return res;
}


/* functions that returns an order of the pseudo backbone
atoms of Proline (with hydrogens) with increasing topological distance to the 
previous backbone */
static p3d_poly **proH_post_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 14 * sizeof(p3d_poly *));
res[0]= input[5];
res[1]= input[2];
res[2]= input[11];
res[3]= input[3];
res[4]= input[0];
res[5]= input[4];
res[6]= input[8];

res[7]= input[12];
res[8]= input[13];
res[9]= input[6];
res[10]= input[7];
res[11]= input[9];
res[12]= input[10];
res[13]= input[1];
return res;
}


/* functions that returns an order of the pseudo backbone
atoms with increasing topological distance to the 
previous backbone */


static reordering_function post_order[]={


usual_post_ordering,  //ALA  
usual_post_ordering,  //ARG
usual_post_ordering,  //ASN
usual_post_ordering,  //ASP
usual_post_ordering,  //CYS
usual_post_ordering,  //GLN
usual_post_ordering,  //GLU
gly_post_ordering,  //GLY   *
usual_post_ordering,  //HIS
usual_post_ordering,  //ILE
usual_post_ordering,  //LEU
usual_post_ordering,  //LYS
usual_post_ordering,  //MET
usual_post_ordering,  //PHE
pro_post_ordering,  //PRO    *
usual_post_ordering,  //SER
usual_post_ordering,  //THR
usual_post_ordering,  //TRP
usual_post_ordering,  //TYR
usual_post_ordering,  //VAL

usualH_post_ordering,  //ALAH
usualH_post_ordering,  //ARGH
usualH_post_ordering,  //ASNH
usualH_post_ordering,  //ASPH
usualH_post_ordering,  //CYSH
usualH_post_ordering,  //GLNH
usualH_post_ordering,  //GLUH
usualH_post_ordering,  //GLYH
usualH_post_ordering,  //HISH  
usualH_post_ordering,  //ILEH
usualH_post_ordering,  //LEUH
usualH_post_ordering,  //LYSH
usualH_post_ordering,  //METH
usualH_post_ordering,  //PHEH
proH_post_ordering,  //PROH   *
usualH_post_ordering,  //SERH
usualH_post_ordering,  //THRH
usualH_post_ordering,  //TRPH
usualH_post_ordering,  //TYRH
usualH_post_ordering   //VALH
};


/* function for common residual without hydrogens that returns 
an order of the pseudo backbone atoms wit increasing topological
distance to the pseudo side chain of the residual */
static p3d_poly **usual_sc_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 5 * sizeof(p3d_poly *));
res[0]= input[3];
res[1]= input[2];
res[2]= input[0];
res[3]= input[4];
res[4]= input[1];
return res;
}


/* function for common residual with hydrogens that returns 
an order of the pseudo backbone atoms wit increasing topological
distance to the pseudo side chain of the residual */
static p3d_poly **usualH_sc_ordering(p3d_poly **input){
p3d_poly  **res= (p3d_poly **) malloc( 6 * sizeof(p3d_poly *));
res[0]= input[4];
res[1]= input[2];
res[2]= input[5];
res[3]= input[3];
res[4]= input[0];
res[5]= input[1];
return res;
}



/* functions that returns an order of the pseudo backbone
atoms wit increasing topological distance to the 
pseudo side chain of the residual */

static reordering_function sc_order[]={
NULL,  //ALA  not used
usual_sc_ordering,  //ARG
usual_sc_ordering,  //ASN
usual_sc_ordering,  //ASP
usual_sc_ordering,  //CYS
usual_sc_ordering,  //GLN
usual_sc_ordering,  //GLU
NULL,  //GLY  not used
usual_sc_ordering,  //HIS
usual_sc_ordering,  //ILE
usual_sc_ordering,  //LEU
usual_sc_ordering,  //LYS
usual_sc_ordering,  //MET
usual_sc_ordering,  //PHE
NULL,  //PRO  not used
usual_sc_ordering,  //SER
usual_sc_ordering,  //THR
usual_sc_ordering,  //TRP
usual_sc_ordering,  //TYR
usual_sc_ordering,  //VAL

usualH_sc_ordering,  //ALAH
usualH_sc_ordering,  //ARGH
usualH_sc_ordering,  //ASNH
usualH_sc_ordering,  //ASPH
usualH_sc_ordering,  //CYSH
usualH_sc_ordering,  //GLNH
usualH_sc_ordering,  //GLUH
NULL,  //GLYH  not used
usualH_sc_ordering,  //HISH  
usualH_sc_ordering,  //ILEH
usualH_sc_ordering,  //LEUH
usualH_sc_ordering,  //LYSH
usualH_sc_ordering,  //METH
usualH_sc_ordering,  //PHEH
NULL,  //PROH  not used
usualH_sc_ordering,  //SERH
usualH_sc_ordering,  //THRH
usualH_sc_ordering,  //TRPH
usualH_sc_ordering,  //TYRH
usualH_sc_ordering   //VALH
};


/* SD_function's that test internal
collisions in the Proline with hydrogens */
static void autocol_bb_ProH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_pair(lA[1],lA[7]);
check_pair(lA[1],lA[6]);
/* Only one of this collisions are geometrical possible. Thus one of the 
collision test could be avoided if the hydrogen that can collide with the
oxigen is well identified and put in the same number of order*/
}


/* SD_function's that test internal
collisions in the pseudo-backbones */
static SD_function autocol_bb[]={
NULL,  //ALA
NULL,  //ARG
NULL,  //ASN
NULL,  //ASP
NULL,  //CYS
NULL,  //GLN
NULL,  //GLU
NULL,  //GLY
NULL,  //HIS
NULL,  //ILE
NULL,  //LEU
NULL,  //LYS
NULL,  //MET
NULL,  //PHE
NULL,  //PRO topologicaly possible, geometrically impossible
NULL,  //SER
NULL,  //THR
NULL,  //TRP
NULL,  //TYR
NULL,  //VAL

NULL,  //ALAH
NULL,  //ARGH
NULL,  //ASNH
NULL,  //ASPH
NULL,  //CYSH
NULL,  //GLNH
NULL,  //GLUH
NULL,  //GLYH
NULL,  //HISH  
NULL,  //ILEH
NULL,  //LEUH
NULL,  //LYSH
NULL,  //METH
NULL,  //PHEH
autocol_bb_ProH,  //PROH *
NULL,  //SERH
NULL,  //THRH
NULL,  //TRPH
NULL,  //TYRH
NULL   //VALH
};

/*************************************************************
*************************************************************
*************************************************************/




/* SD_function's that test internal collisions in the Arginine
 pseudo-side chain without hydrogens */
static void autocol_sc_arg(SDpt SD){
p3d_poly  **lA=SD->lA;
check_pair(lA[4],lA[0]);
check_pair(lA[5],lA[0]);
}

/* SD_function's that test internal collisions in the Arginine
 pseudo-side chain with hydrogens */
static void autocol_sc_argH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_all(lA + 10, 2,   lA, 5);
check_all(lA + 10, 2,   lA + 6, 2);
check_all(lA + 8, 2,   lA, 2);
check_pair(lA[9],lA[3]);
check_pair(lA[9],lA[4]);
}

/* SD_function's that test internal collisions in the Glu
 pseudo-side chain with hydrogens */
static void autocol_sc_glnH(SDpt SD){
check_all(SD->lA, 2,   (SD->lA) + 6, 2);
}


/* SD_function's that test internal collisions in the Gln
 pseudo-side chain with hydrogens */
static void autocol_sc_gluH(SDpt SD){
check_all(SD->lA, 2,   (SD->lA) + 6, 2);
}

/* SD_function's that test internal collisions in the Ile
 pseudo-side chain with hydrogens */
static void autocol_sc_ileH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_all(lA + 6, 6,   lA + 3, 3);
check_all(lA + 9, 3,   lA, 2); 
}

/* SD_function's that test internal collisions in the leucine
 pseudo-side chain with hydrogens */
static void autocol_sc_leuH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_all(lA + 9, 3,   lA + 6, 3);
check_all(lA + 6, 6,   lA, 2);
}


/* SD_function's that test internal collisions in the lysine
 pseudo-side chain with hydrogens */
static void autocol_sc_lysH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_all(lA + 6, 6,   lA, 2);
check_all(lA + 9, 3,   lA + 3, 2);
}


/* SD_function's that test internal collisions in the metionine
 pseudo-side chain with hydrogens 
 */
static void autocol_sc_metH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_all(lA + 6, 4,   lA, 2);
check_all(lA + 7, 3,   lA + 3, 2);
}

/* SD_function's that test internal collisions in the Phe
 pseudo-side chain with hydrogens 
  SEEMS GEOMETRICALLY IMPOSSIBLE */
static void autocol_sc_pheH(SDpt SD){
check_all((SD->lA) + 5, 2,   SD->lA, 2);
}

/* SD_function's that test internal collisions in the Trp
 pseudo-side chain with hydrogens 
  SEEMS GEOMETRICALLY IMPOSSIBLE */
static void autocol_sc_trpH(SDpt SD){
p3d_poly  **lA=SD->lA;
check_pair(lA[5],lA[0]);
check_pair(lA[5],lA[1]);
}


/* SD_function's that test internal collisions in the tyrosine
 pseudo-side chain with hydrogens
  SEEMS GEOMETRICALLY IMPOSSIBLE */
static void autocol_sc_tyrH(SDpt SD){
check_all((SD->lA) + 7, 2,   SD->lA, 2);
}

/* SD_function's that test internal collisions in the Val
 pseudo-side chain with hydrogens */
static void autocol_sc_valH(SDpt SD){
check_all((SD->lA) + 3, 3,   (SD->lA) + 6, 3);
}



/* SD_function's that test internal
collisions in the pseudo-side-chains */
static SD_function autocol_sc[]={
NULL,  //ALA
autocol_sc_arg,  //ARG
NULL,  //ASN
NULL,  //ASP
NULL,  //CYS
NULL,  //GLN
NULL,  //GLU
NULL,  //GLY
NULL,  //HIS
NULL,  //ILE
NULL,  //LEU
NULL,  //LYS
NULL,  //MET
NULL,  //PHE
NULL,  //PRO
NULL,  //SER
NULL,  //THR
NULL,  //TRP
NULL,  //TYR topologicaly possible, geometrically impossible
NULL,  //VAL

NULL,  //ALAH
autocol_sc_argH,  //ARGH
NULL,  //ASNH
NULL,  //ASPH
NULL,  //CYSH
autocol_sc_glnH,  //GLNH
autocol_sc_gluH,  //GLUH
NULL,  //GLYH
NULL,  //HISH topologicaly possible, seems geometrically impossible  
autocol_sc_ileH,  //ILEH
autocol_sc_leuH,  //LEUH
autocol_sc_lysH,  //LYSH
autocol_sc_metH,  //METH
NULL, //PHEH autocol_sc_pheH not used because it seems geometrically impossible
NULL,  //PROH
NULL,  //SERH
NULL,  //THRH
NULL,  //TRPH autocol_sc_trpH not used because it seems geometrically impossible
NULL,  //TYRH autocol_sc_tyrH not used because it seems geometrically impossible
autocol_sc_valH   //VALH
};


void colisiones_aisladas(int iniA, int nA, int iniB, int nB, char *lA, char *lB){
int i,j;
int lastA= iniA + nA, lastB= iniB + nB;
for (i=iniA; i < lastA;i++)
	for (j=iniB; j < lastB;j++)
		printf("check_pair(%s[%d], %s[%d]);\n", lA, i, lB, j);
}


void write_SD_function(SDpt SD){
char *lA= (char*)"lA";
char *lB= (char*)"lB";

printf ("\n\n");
printf("static void %d_bb_sc_SD_function(SDpt SD){\n", SD->rigidA->lbox[0]->aminotype);

if ( (SD->nA * SD->tB4 > 0) + (SD->tA234 * SD->tB3 > 0) +
	(SD->tA34 * SD->tB2 > 0) + (SD->tA4 * SD->tB1 > 0) > 1){
	printf("p3d_poly  **lA=SD->lA;\n");
	printf("p3d_poly  **lB=SD->lB;\n");
	}
else{
	lA= (char*)"(SD->lA)";
	lB= (char*)"(SD->lB)";
	}

if (SD->nA * SD->tB4 > 2) printf("check_all(%s, %d,   %s + %d, %d);\n",
	lA, SD->nA, lB, SD->tB123, SD->tB4);	
else colisiones_aisladas(0, SD->nA, SD->tB123, SD->tB4, lA, lB);

if (SD->tA234 * SD->tB3 > 2) printf("check_all(%s + %d,  %d, %s + %d, %d);\n",
	lA, SD->tA1, SD->tA234, lB, SD->tB12, SD->tB3 );
else colisiones_aisladas(SD->tA1, SD->tA234, SD->tB12, SD->tB3, lA, lB);

if (SD->tA34 * SD->tB2 > 2) printf("check_all(%s + %d,  %d, %s + %d, %d);\n",
	lA, SD->tA12, SD->tA34, lB, SD->tB1, SD->tB2);
else colisiones_aisladas(SD->tA12, SD->tA34, SD->tB1, SD->tB2, lA, lB);

if (SD->tA4 * SD->tB1 > 2) printf("check_all(%s + %d, %d,  %s , %d);\n",
	lA, SD->tA123, SD->tA4, lB, SD->tB1);
else colisiones_aisladas(SD->tA123, SD->tA4, 0, SD->tB1, lA, lB);
printf ("\n}\n");
}

/* 	TO USE THIS FUNCTION DO NOT ALLOW SD'S OF THE TYPE 	Obb_sc_SD */
void write_SD_functions(){
SDpt SD, *lSD;
int i=0,index,n;
Robot_structure *robot;
for (n=0;n < number_of_bcd_robots; n++){
	robot=bcd_robots[n];
	printf ("robot %d en write sd fucntions\n", n);
	if (robot->polypep){
		printf ("robot %d en write sd fucntions es POLYEPTIDO\n", i);
		lSD= robot->intra_res;
		index= robot->autocol_index;
		for(i=0;i < index; i++){
			SD= *(lSD++);
			if (SD->rigidB) /* it is a side chain-backbone collision */
			write_SD_function(SD);
			}
		}
	} 

}	

/* This function can be used in general to check the topologically possible 
interactions between the two groups lA and lB declared in a SD_struct */ 
void general_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;

/* All pair of atoms composed by an atom of A and an atom of B at 
topological distance of four or more must be evaluated.
Take into account that summing the topological distances from A and B,
the distance between the "bounding atoms" linking A and B will be 
counted twice, and thus we evaluate sums of distances of 5 or more */


/*all atoms from A with those of B at four or more from A */
if (SD->tB4){ // I assume that there is always at least an atom in lA
	check_all(lA, SD->nA,   lB + SD->tB123, SD->tB4);
	}

/* atoms  from A at distance 2 or more those of B at distance 3 from A */
if (SD->tA234 &&  SD->tB3){ 
	check_all(lA + SD->tA1,  SD->tA234, lB + SD->tB12, SD->tB3);
	}
/* atoms  from A at distance 3 or more those of B at distance 2 from A */
if (SD->tA34 &&  SD->tB2){ 
	check_all(lA + SD->tA12,  SD->tA34, lB + SD->tB1, SD->tB2);

	}	
/* atoms  from A at distance 4 or more those of B at distance 1 from A */
if (SD->tA4 &&  SD->tB1){
	check_all(lA + SD->tA123, SD->tA4,  lB , SD->tB1);

	}
}	



static void arg_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 6,   lB + 4, 1);
check_all(lA + 1,  5, lB + 2, 2);
check_all(lA + 2,  4, lB + 1, 1);
check_all(lA + 3, 3,  lB , 1);

}

static void asn_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 3,   lB + 4, 1);
check_all(lA + 1,  2, lB + 2, 2);

}


static void asp_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 3,   lB + 4, 1);
check_all(lA + 1,  2, lB + 2, 2);

}

static void cys_bb_sc_SD_function(SDpt SD){
check_pair((SD->lA)[0], (SD->lB)[4]);

}


static void gln_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 4,   lB + 4, 1);
check_all(lA + 1,  3, lB + 2, 2);
check_pair(lA[2], lB[1]);
check_pair(lA[3], lB[1]);

}

static void glu_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 4,   lB + 4, 1);
check_all(lA + 1,  3, lB + 2, 2);
check_pair(lA[2], lB[1]);
check_pair(lA[3], lB[1]);

}


static void his_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 5,   lB + 4, 1);
check_all(lA + 1,  4, lB + 2, 2);
check_pair(lA[3], lB[1]);
check_pair(lA[4], lB[1]);

}

static void ile_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 3,   lB + 4, 1);
check_pair(lA[2], lB[2]);
check_pair(lA[2], lB[3]);

}

static void leu_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 3,   lB + 4, 1);
check_all(lA + 1,  2, lB + 2, 2);

}

static void lys_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 4,   lB + 4, 1);
check_all(lA + 1,  3, lB + 2, 2);
check_pair(lA[2], lB[1]);
check_pair(lA[3], lB[1]);
check_pair(lA[3], lB[0]);

}

static void met_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 3,   lB + 4, 1);
check_all(lA + 1,  2, lB + 2, 2);
check_pair(lA[2], lB[1]);

}

static void phe_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 6,   lB + 4, 1);
check_all(lA + 1,  5, lB + 2, 2);
check_all(lA + 3,  3, lB + 1, 1);
check_pair(lA[5], lB[0]);

}

static void ser_bb_sc_SD_function(SDpt SD){
check_pair((SD->lA)[0], (SD->lB)[4]);

}

static void thr_bb_sc_SD_function(SDpt SD){
check_pair((SD->lA)[0], (SD->lB)[4]);
check_pair((SD->lA)[1], (SD->lB)[4]);

}

static void trp_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 9,   lB + 4, 1);
check_all(lA + 1,  8, lB + 2, 2);
check_all(lA + 3,  6, lB + 1, 1);
check_all(lA + 6, 3,  lB , 1);

}

static void tyr_bb_sc_SD_function(SDpt SD){
p3d_poly  **lA=SD->lA;
p3d_poly  **lB=SD->lB;
check_all(lA, 7,   lB + 4, 1);
check_all(lA + 1,  6, lB + 2, 2);
check_all(lA + 3,  4, lB + 1, 1);
check_pair(lA[5], lB[0]);
check_pair(lA[6], lB[0]);

}

static void val_bb_sc_SD_function(SDpt SD){
check_pair((SD->lA)[0], (SD->lB)[4]);
check_pair((SD->lA)[1], (SD->lB)[4]);

}


/* SD_function's that test  collisions of the pseudo-backbone with
the pseudo-side chain  */
static SD_function bb_sc_col[]={
NULL,  //ALA
arg_bb_sc_SD_function,  //ARG
asn_bb_sc_SD_function,  //ASN
asp_bb_sc_SD_function,  //ASP
cys_bb_sc_SD_function,  //CYS
gln_bb_sc_SD_function,  //GLN
glu_bb_sc_SD_function,  //GLU
NULL,  //GLY
his_bb_sc_SD_function,  //HIS
ile_bb_sc_SD_function,  //ILE
leu_bb_sc_SD_function,  //LEU
lys_bb_sc_SD_function,  //LYS
met_bb_sc_SD_function,  //MET
phe_bb_sc_SD_function,  //PHE
NULL,  //PRO
ser_bb_sc_SD_function,  //SER
thr_bb_sc_SD_function,  //THR
trp_bb_sc_SD_function,  //TRP
tyr_bb_sc_SD_function,  //TYR
val_bb_sc_SD_function,  //VAL

general_SD_function,  //ALAH
general_SD_function,  //ARGH
general_SD_function,  //ASNH
general_SD_function,  //ASPH
general_SD_function,  //CYSH
general_SD_function,  //GLNH
general_SD_function,  //GLUH
NULL,  //GLYH
general_SD_function,  //HISH  
general_SD_function,  //ILEH
general_SD_function,  //LEUH
general_SD_function,  //LYSH
general_SD_function,  //METH
general_SD_function,  //PHEH
NULL,  //PROH
general_SD_function,  //SERH
general_SD_function,  //THRH
general_SD_function,  //TRPH
general_SD_function,  //TYRH
general_SD_function   //VALH
};


/*  collisions in the residual when the backbone is 
free and the psedudo side chain is blocked (usually 
between oxygen backbone and the side chain). The functions in 
bb_sc_col[] would do the same job but a little more inefficiently in
this situation */
void Obb_sc_SD_function(SDpt SD){
check_all(SD->lA, SD->nA,   SD->lB + SD->tB123, SD->tB4);
}


/* SD_function's that test  collisions of the pseudo-backbone with
the pseudo-side chain 
static SD_function bb_sc_col[]={
NULL,  //ALA
general_SD_function,  //ARG
general_SD_function,  //ASN
general_SD_function,  //ASP
general_SD_function,  //CYS
general_SD_function,  //GLN
general_SD_function,  //GLU
NULL,  //GLY
general_SD_function,  //HIS
general_SD_function,  //ILE
general_SD_function,  //LEU
general_SD_function,  //LYS
general_SD_function,  //MET
general_SD_function,  //PHE
NULL,  //PRO
general_SD_function,  //SER
general_SD_function,  //THR
general_SD_function,  //TRP
general_SD_function,  //TYR
general_SD_function,  //VAL

general_SD_function,  //ALAH
general_SD_function,  //ARGH
general_SD_function,  //ASNH
general_SD_function,  //ASPH
general_SD_function,  //CYSH
general_SD_function,  //GLNH
general_SD_function,  //GLUH
NULL,  //GLYH
general_SD_function,  //HISH  
general_SD_function,  //ILEH
general_SD_function,  //LEUH
general_SD_function,  //LYSH
general_SD_function,  //METH
general_SD_function,  //PHEH
NULL,  //PROH
general_SD_function,  //SERH
general_SD_function,  //THRH
general_SD_function,  //TRPH
general_SD_function,  //TYRH
general_SD_function   //VALH
};

*/

/* fills the topological fields B of a Short Distance structure
using a topol_rank  (see declaration of topol_rank) */
static void SD_top_fieldsB(topol_rank rank,SDpt SD){
SD->tB1= rank[0]; 
SD->tB2= rank[1]; 
SD->tB3= rank[2];
SD->tB4= rank[3];
SD->tB12= SD->tB1 + SD->tB2; 
SD->tB123= SD->tB12 + SD->tB3;
}

/* fills the topological fields A of a Short Distance structure
using a topol_rank (see declaration of topol_rank) */
static void SD_top_fieldsA(topol_rank rank,SDpt SD){

SD->tA1= rank[0]; 
SD->tA4= rank[3];
SD->tA12= SD->tA1 + rank[1]; 
SD->tA123= SD->tA12 + rank[2];
SD->tA34=  rank[2] + SD->tA4; 
SD->tA234= SD->tA34 + rank[1];
SD->nA= SD->tA123 + SD->tA4;
}


/***********************************************************************************
***************************************************************************************
              BUILDING OF THE SHORT TOPOLOGICAL DISTANCE LONG-TERM STRUCTURES
			  ASSOCIATED TO A RESIDUAL
***************************************************************************************
***************************************************************************************/


static SDpt init_SD(){
SDpt SD = (SDpt) malloc(sizeof(SD_struct));
SD->tobefreededA= 1;

/* CRITICAL POINTS: 
lA points always to a  meaningful memory space.Instead,
lB can be NULL. 
Also, only lB points always to a new fresh memory space that
must be freeded in free_SD. Instead, lA can point to a new memory
space or not and must be freeded or not.
There is not any subtle purpose in these facts. You can remove
these assumptions with a little of care from the code without
colateral effects */

SD->rigidB= NULL; 
#ifdef SUPRESSION
SD->active= 1 ;	
#endif	
return SD;
}



/* creates a new Short Distances structure of the type backbone-backbone */
void new_inter_bb_SD(){
SDpt SD;

SD= inter_bb[intercol_index++]= init_SD();
SD->lA= prev_bb_box->lpoly; 
SD->tobefreededA= 0;
SD_top_fieldsA(to_next_rank[ prev_bb_box->aminotype], SD);
SD->lB= (*(post_order[last_bb_box->aminotype])) (last_bb_box->lpoly);
SD_top_fieldsB(to_prev_rank[last_bb_box->aminotype], SD);
prev_rigid->lSD[(prev_rigid->nSD)++] = SD;
SD->rigidA= prev_rigid;
rigid->lSD[(rigid->nSD)++] = SD;
SD->rigidB= rigid;
SD->function= general_SD_function;
}



/* creates SD structure for internal collisions in the residual (usually 
between oxygen backbone and the side chain) used when the backbone is 
free and the psedudo side chain. The usual new_sc_bb_SD function could
also be used but this is more efficient */
void new_Obb_sc_SD(){
SDpt SD;
SD= intra_res[autocol_index++]= init_SD();

SD->lA= last_sc_box->lpoly;
SD->tobefreededA= 0;
SD_top_fieldsA(sc_rank[ last_sc_box->aminotype], SD);

SD->lB= (*(sc_order[last_bb_box->aminotype])) (last_bb_box->lpoly);
SD_top_fieldsB(to_sc_rank[last_bb_box->aminotype], SD);

/*Only a rigid is involved in this SD */
rigid->lSD[(rigid->nSD)++] = SD;
SD->rigidB = SD->rigidA= rigid;
SD->function= Obb_sc_SD_function;
}


void new_autocol_bb_or_sc_SD(Rigid_structure *r,p3d_poly  **list, SD_function function){
SDpt SD;

SD= intra_res[autocol_index++]= init_SD();
SD->lA= list;
SD->tobefreededA= 0;
r->lSD[(r->nSD)++]= SD;
SD->rigidA= r;
SD->function= function;
}


/* creates a new Short Distances structure of the type backbone-side chain */
void new_sc_bb_SD(){
SDpt SD;

SD= intra_res[autocol_index++]= init_SD();

SD->lA= last_sc_box->lpoly;
SD->tobefreededA= 0;
SD_top_fieldsA(sc_rank[ last_sc_box->aminotype], SD);

SD->lB= (*(sc_order[last_bb_box->aminotype])) (last_bb_box->lpoly);
SD_top_fieldsB(to_sc_rank[last_bb_box->aminotype], SD);

sc_rigid->lSD[(sc_rigid->nSD)++] = SD;
SD->rigidA= sc_rigid;
rigid->lSD[(rigid->nSD)++] = SD;
SD->rigidB= rigid;
SD->function= bb_sc_col[last_bb_box->aminotype];
}


void get_init_variables(void){
struct bcd_init_structure *init_var= pass_init_variables();

sc_free = init_var->sc_free;
bb_free = init_var->bb_free;
prev_bb_free = init_var->prev_bb_free;
loop = init_var->loop;
new_disulphur = init_var->new_disulphur;
rigid = init_var->rigid;
prev_rigid = init_var->prev_rigid;
sc_rigid = init_var->sc_rigid;
inter_bb = init_var->inter_bb;
intra_res = init_var->intra_res;
loops = init_var->loops;
autocol_index = init_var->autocol_index;
intercol_index = init_var->intercol_index;
loop_index = init_var->loop_index;
last_bb_box = init_var->last_bb_box;
prev_bb_box = init_var->prev_bb_box;
last_sc_box = init_var->last_sc_box;
 }


/* creates all Short Distance structures for a residual */
void create_residual_SDs(int *autocolindex, int *intercolindex,int *loopindex){
enum aminoacidos tipo;
static int inverse_sc_cys[]= {1,0,0,0};
static int inverse_sc_cysH[]= {1,0,2,0};
SDpt SD;

get_init_variables();
tipo= last_bb_box->aminotype;
if (prev_bb_free || (bb_free && /* avoiding first bb_box case */ prev_bb_box))
	new_inter_bb_SD();
if (bb_free && (autocol_bb[tipo]))
	new_autocol_bb_or_sc_SD(rigid,last_bb_box->lpoly, autocol_bb[tipo]);

if ( bb_free && !(sc_free)){
	if (bb_sc_col[tipo])
		new_Obb_sc_SD(); 
	}

if (sc_free){
	new_sc_bb_SD();
	if (autocol_sc[tipo])
		new_autocol_bb_or_sc_SD(
			sc_rigid, last_sc_box->lpoly, autocol_sc[tipo]);
	}
	
if (loop){ /* the current residual has a disulphur link. It can be with a higher index
	residual (new_disulphur= TRUE) or with an already processed residual (new_disulphur= FALSE) */
	int *rank;
	int i,nsc= last_sc_box->natoms;
	if (tipo == CYS) rank=inverse_sc_cys;
	else rank=inverse_sc_cysH;
	if (new_disulphur){
	/* bb1 - bb2 */
		SD= loops[loop_index++]= init_SD();
		SD->lA= (*(sc_order[tipo])) (last_bb_box->lpoly);
		SD_top_fieldsA(to_sc_rank[tipo], SD);
		rigid->lSD[(rigid->nSD)++] = SD;
		SD->rigidA= rigid;
		SD->function= general_SD_function; // bb_bb_disulphur;
	/* bb1 - cl2 */
		SD= loops[loop_index++]= init_SD();
		SD->lA= (*(sc_order[tipo])) (last_bb_box->lpoly);
		SD_top_fieldsA(to_sc_rank[tipo], SD);
		rigid->lSD[(rigid->nSD)++] = SD;
		SD->rigidA= rigid;
		SD->function= general_SD_function; //bb_sc_disulphur;
	/* cl1 - bb2 */
		SD= loops[loop_index++]= init_SD();
		SD->lA= (p3d_poly **) malloc( nsc * sizeof(p3d_poly *));
		/* in verting order */
		for (i=0; i < nsc;i++)
			SD->lA[i]=last_sc_box->lpoly[nsc - 1 - i];		
		SD_top_fieldsA(rank, SD);
		sc_rigid->lSD[(sc_rigid->nSD)++] = SD;
		SD->rigidA= sc_rigid;
		SD->function= general_SD_function;//sc_bb_disulphur;
	/* cl1 - cl2 */
		SD= loops[loop_index++]= init_SD();
		SD->lA= (p3d_poly **) malloc( nsc * sizeof(p3d_poly *));
		/* in verting order */
		for (i=0;i < nsc;i++)
			SD->lA[i]=last_sc_box->lpoly[nsc - 1 - i];		
		SD_top_fieldsA(rank, SD);
		sc_rigid->lSD[(sc_rigid->nSD)++] = SD;
		SD->rigidA= sc_rigid;
		SD->function= general_SD_function; // sc_sc_disulphur;
		}
	else{
	/* bb1 - bb2 */
		SD= loops[disulphur_index];
		SD->lB= (*(sc_order[tipo])) (last_bb_box->lpoly);
		/* the topological distance to the other cysteine backbone
		are shifted by 2, with respect to the own side chain */
		SD_top_fieldsB(shift_topo(2, to_sc_rank[tipo]), SD);
		rigid->lSD[(rigid->nSD)++] = SD;
		SD->rigidB= rigid;
	/* cl1 - bb2 */
		SD= loops[disulphur_index + 2];
		SD->lB= (*(sc_order[tipo])) (last_bb_box->lpoly);
		/* the topological distance to the other cysteine side chain
		are shifted by 1, with respect to the own side chain */
		SD_top_fieldsB(shift_topo(1, to_sc_rank[tipo]), SD);
		rigid->lSD[(rigid->nSD)++] = SD;
		SD->rigidB= rigid;
	/* bb1 - cl2 */
		SD= loops[disulphur_index + 1];
		SD->lB= (p3d_poly **) malloc( nsc * sizeof(p3d_poly *));
		/* in verting order */
		for (i=0;i < nsc;i++)
			SD->lB[i]=last_sc_box->lpoly[nsc - 1 - i];
		/* the topological distance to the other cysteine backbone
		are shifted by 1, with respect to the other cysteine side chain */
		SD_top_fieldsB(shift_topo(1, rank), SD);
		sc_rigid->lSD[(sc_rigid->nSD)++] = SD;
		SD->rigidB= sc_rigid;
	/* cl1 - cl2 */
		SD= loops[disulphur_index + 3];
		SD->lB= (p3d_poly **) malloc( nsc * sizeof(p3d_poly *));
		/* in verting order */
		for (i=0;i < nsc;i++)
			SD->lB[i]=last_sc_box->lpoly[nsc - 1 - i];
		/* the topological distance to the other cysteine backbone
		are shifted by 1, with respect to the other cysteine side chain */
		SD_top_fieldsB(rank, SD);
		SD->rigidB= sc_rigid;
		sc_rigid->lSD[(sc_rigid->nSD)++] = SD;
		}	
	}
*autocolindex= autocol_index; 
*intercolindex= intercol_index;
*loopindex= loop_index;
}



