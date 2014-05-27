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
 * Sim√©on, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
/****************************************************************************/
/*! \file bio.h
 *
 *  \brief bio structures
 *
 ****************************************************************************/

#ifndef BIO_H
#define BIO_H

#define SUPRESSION
#define HYDROGEN_COMPILATION

//#define HYDROGEN_BOND


/*-----------------------------------------------------------------------------*/
/* BIO JOINT TYPES */
#define BIO_OMEGA_JNT 1
#define BIO_PHI_JNT 2
#define BIO_PSI_JNT 3
#define BIO_GAMMA_JNT 4
//#define BIO_FF_JNT 5
//#define BIO_LIG_JNT 6
#define BIO_OTHER_JNT 7

/*-----------------------------------------------------------------------------*/
/* ATOMIC MASS */
#define C_MASS 12.0107
#define O_MASS 15.9994
#define N_MASS 14.00674
#define H_MASS 1.00794
#define S_MASS 32.066
#define P_MASS 30.973761
#define Br_MASS 79.904
#define F_MASS 18.9984032
#define Cl_MASS 35.4527
#define I_MASS 126.90447

/*-----------------------------------------------------------------------------*/
/* FOR POLYNOMIALS !!! */

#ifdef USE_GSL
#include <gsl/gsl_poly.h>
#endif
#ifdef USE_CLAPACK
// WARNING : ONLY FOR Mac OS X !!!
#ifdef MACOSX
#include <vecLib/vecLib.h>
//#include <f2c.h>
#endif
#endif

#include <p3d.h>
//#include <device.h>

typedef double poly2_coefs [3];

typedef struct s_poly_max16_coefs {
  int order;
  double a_i[17];  // a0 + a1*x + a2*x≤ + a3*x≥ ...
} poly_max16_coefs;

/*-----------------------------------------------------------------------------*/

#define BIO_IK_ERROR_TOL 0.001

/*-----------------------------------------------------------------------------*/
/*! \brief Structure to store date for the 6R_bio_ik method
 */

typedef struct s_bio_6R_ik_data {
  /* constant params */
  double r2, r3, r4, r5, r6, rE;
  double al1, al2, al3, al4, al5, alE;
  double sal1,cal1,sal2,cal2,sal3,cal3,sal4,cal4,sal5,cal5;
  double cal1sal2,sal1sal2,cal1cal2,sal1cal2;
  double r2sal1,r2cal1,r2sal2,r2cal2,sqr_r2_05;
  double r5sal4,r5cal4,cal3sal4,sal3sal4,sal3cal4,cal3cal4,r4cal3sal4,r4sal3sal4,cal5sal5,sqrr4,sqrr5,sqrsal5;
  double coef45_1,coef543_1,coef543_2;
  /* variable params */
  double th7, r7, al7, a7, r0;
  double th1_i;
  double sal7,cal7,sth7,cth7;
  double cth7sal7,sth7sal7,cth7cal7,sth7cal7;
  double coef70_1,coef70_2,coef70_3,coef730_1,coef730_2;
  /* variables for coefs */
  double palcc,palcs,palc1,palsc,palss,pal1c,pal1s,pal11;
  double pbecc,pbec1,pbess,pbe1c,pbe11;
  double pgacc,pgacs,pgac1,pgasc,pgass,pga1c,pga1s,pga11;
  double paet1c,paet1s,paet11;
  double pacc,pacs,pac1,pasc,pass,pa1c,pa1s,pa11_,pa11;
  double pbet1c,pbet11;
  double pbcc,pbc1,pbss,pb1c,pb11_,pb11;
  double pcet1c,pcet1s,pcet11;
  double pccc,pccs,pcc1,pcsc,pcss,pc1c,pc1s,pc11_,pc11;
  double pset11;
  double pscc,pscs,psc1,pssc,psss,ps1c,ps1s,ps11;
  double psiet11;
  double psicc,psics,psic1,psisc,psiss,psi1c,psi1s,psi11;
  double puetcc,puetcs,puetsc,puetss,puets1;
  double pucc,pucs,puc1,pusc,puss,pus1,pu1c,pu1s,pu11;
  double pvetcs,pvetsc,pvets1;
  double pvcc,pvcs,pvc1,pvsc,pvss,pvs1,pv1c,pv1s,pv11;
  double pwetcc,pwetcs,pwetsc,pwetss,pwets1;
  double pwcc,pwcs,pwc1,pwsc,pwss,pws1,pw1c,pw1s,pw11;
  double pletcc,pletcs,pletc1,pletsc,pletss,plet1c,plet1s,plet11;
  double plcc,plcs,plc1,plsc,plss,pls1,pl1c,pl1s,pl11;
  double pmetcc,pmetc1,pmetss,pmet1c,pmet11;
  double pmcc,pmcs,pmc1,pmsc,pmss,pms1,pm1c,pm1s,pm11;
  double pnetcc,pnetcs,pnetc1,pnetsc,pnetss,pnet1c,pnet1s,pnet11;
  double pncc,pncs,pnc1,pnsc,pnss,pns1,pn1c,pn1s,pn11;  
} bio_6R_ik_data;

typedef struct SideChainBBox {
    double min[3];
    double max[3];
} SideChainBBox, *SideChainBBoxPt;


typedef struct Joint_Tables {
  struct jnt* bb_joints[3]; /* pointers to backbone joints */
  struct jnt* sc_joints[6];  /* pointers to side chain joints */
  int namino;
  int n_bb_joints;				/* number of backbone joints */
  int n_sc_joints;			/* number of side chain joints */
  SideChainBBox SC_BBox;    /* Bounding Box of the Side chain of the AA*/
} Joint_tables,*Joint_tablespt;



enum aminoacidos {ALA, ARG, ASN, ASP, CYS, GLN, GLU, GLY, HIS, ILE, LEU,
LYS, MET, PHE, PRO, SER, THR, TRP, TYR, VAL,
ALAH, ARGH, ASNH, ASPH, CYSH, GLNH, GLUH, GLYH, HISH,
ILEH, LEUH,
LYSH, METH, PHEH, PROH, SERH, THRH, TRPH, TYRH, VALH};

typedef struct BBox {
    char name[6];
    double min[3];
    double max[3];
    p3d_poly **lpoly;
    short int bb;
    enum aminoacidos aminotype;
    int namino;
    int natoms;
    int loop;
/*
#ifdef SUPRESSION    
int active; 
#endif 
*/
} Bbox, *Bboxpt;


typedef struct TBox {
    double min[3];
    double max[3];
    Bboxpt *lbox;
    int nbox;
    struct TBox *left;
    struct TBox *right;
    short int leaf;
    short int level;
/*
#ifdef SUPRESSION    
int active; 
#endif 
*/
    //int name;
} boxnode;

typedef boxnode *boxnodept;




/* Structure whose purpose is the test of Short (topological) Distances
interactions 
All topologically posible interactions between the two lists of atoms
 lA and lB will be tested */
typedef struct s_SD_struct {
	void (*function)(struct s_SD_struct*);
	p3d_poly **lA; /* first atom list coming from rigid A*/
	p3d_poly **lB; /* second atom list  coming from rigid B*/
	int tobefreededA; /* indicates if lA points to a new space of memory and,
	therefore, must be freeded. */

/* CRITICAL POINTS: 
lA points always to a  meaningful memory space.Instead,
lB can be NULL. 
Also, only lB points always to a new fresh memory space that
must be ALLWAYS freeded in free_SD. Instead, lA can point to a new memory
space or not, and must be freeded or not. This is the purpose of
tobefreededA. Thus, lists that must
 not be freeded must be put in lA. Otherwise another field 
 tobefreededB must be created
There is not any subtle purpose in these facts. You can remove
these assumptions with a little of care from the code without
colateral effects */
/*	int tobefreededB;  indicates if lB points to a new space of memory and,
	therefore, must be freeded */
	struct rigid_structure *rigidA; /* rigid to which lA belongs */
	struct rigid_structure *rigidB; /* rigid to which lB belongs */

	
#ifdef SUPRESSION
	int active;		/* whether the SD is active */
/*	int activeA;	 rigid A active 
	int activeB;	 rigid B active */
#endif	
	int tA1; /* number of atoms of A at topol. distance 1 from B */
	int tA4; /* number of atoms of A at topol. distance 4 or more from B */
	int tA12; /* number of atoms of A at topol. distance 1 or 2 from B */
	int tA34; /* number of atoms of A at topol. distance 3 or more from B */
	int tA123; /* number of atoms of A at topol. distance 1 or 2  or 3 from B */
	int tA234; /* number of atoms of A at topol. distance 2 or more from B */
	int nA; 	/* number of atoms of A */
	
	int tB1; /* number of atoms of B at topol. distance 1 from A */
	int tB2; /* number of atoms of B at topol. distance 2 from A */
	int tB3; /* number of atoms of B at topol. distance 3 from A */
	int tB4; /* number of atoms of B at topol. distance 4 or more from A */
	int tB12; /* number of atoms of B at topol. distance 1 or 2 from A */
	int tB123; /* number of atoms of B at topol. distance 1, 2 or 3 from A */


} SD_struct;

typedef SD_struct *SDpt;


/* especialized function in charge of executing the collision tests
of a given type of SD structure */
typedef void (*SD_function) (SD_struct *SD);

typedef struct rigid_structure {
#ifdef SUPRESSION
int active;
/*
int index; 
 for rigids containing at least a backbone box: index of autocol with the
		preceding rigid (or -1 if there is no one)
	for rigids containing a side chain, the autocol index of the chain
*/
#endif
int nboxes;
Bboxpt *lbox;
/*double *reference_vec;
double (*reference)[4];
*/
p3d_poly *reference; /* atom whose motion reveals the motion of the rigid */
SDpt *lSD; /* list of SD_struct  in which the rigid participates */
int nSD; /* number of SD structures in which the rigid participates */
#ifdef HYDROGEN_COMPILATION
/* Some rigids have two sentinel atoms. If the rigid changes its configuration
at least one of them moves */
p3d_poly *reference2;
#endif
boxnodept root;

}Rigid_structure;


typedef struct SupBox {
    double min[3];
    double max[3];
		boxnodept *lnode;
    int nnodes;
    struct SupBox *left;
    struct SupBox *right;
		struct SupBox *leftbrother; /*only filled for the right brothers */
    short int leaf;
    short int level;
    int name;
} supnode;

typedef supnode *supnodept;

/*
enum SD_type {bb_intercol, bb_bb_disulphur, bb_sc_disulphur,
sc_bb_disulphur, sc_sc_disulphur,
bb_sc_ala, bb_sc_arg, bb_sc_asn, bb_sc_asp,
bb_sc_cys, bb_sc_gln, bb_sc_glu, bb_sc_gly, bb_sc_his,
 bb_sc_ile, bb_sc_leu, bb_sc_lys, bb_sc_met, bb_sc_phe, bb_sc_pro
 , bb_sc_ser, bb_sc_thr, bb_sc_trp, bb_sc_tyr, bb_sc_val,
 bb_sc_alaH , bb_sc_argH, bb_sc_asnH, bb_sc_aspH,
bb_sc_cysH, bb_sc_glnH, bb_sc_gluH, bb_sc_glyH, bb_sc_hisH,
 bb_sc_ileH, bb_sc_leuH, bb_sc_lysH, bb_sc_metH, bb_sc_pheH, bb_sc_proH
 , bb_sc_serH, bb_sc_thrH, bb_sc_trpH, bb_sc_tyrH, bb_sc_valH};
*/



typedef struct robot_structure {

int polypep;
int to_be_rebuilt;
struct rob  *p3d_robot; /* introduced to allow many robots in a p3d_robot */
int init_joint; /* number of joint at which the robot begins
									one p3d_robot can have several "true robots" */

int colliding;
int nrigids;
Rigid_structure **rigids;
SDpt *inter_bb;
SDpt *intra_res;
//SDpt *autocol_bb;
//SDpt *autocol_cl;
SDpt *loops;
int loop_index;
int intercol_index;
int autocol_index;
boxnodept *roots; 
#ifdef SUPRESSION
int n_active_rigids;
int n_aminos;
Rigid_structure **amino_rigidbb; /* pointers to the rigids that contain the residual bb's */
Rigid_structure **amino_rigidcl; /* pointers to the rigids that contain the residual sc's 
if the amino number does not exist NULL. If the the residual does not own a sc NULL*/
Joint_tablespt *joint_tables;
#endif
supnode *superroot;
}Robot_structure;

typedef struct autocol_struc {

int size;
int nrigid;
p3d_poly **list;
}autocol_struc;

typedef void (*check_pair_function) (p3d_poly *,p3d_poly *);
typedef int (*check_all_function) (p3d_poly **,int,p3d_poly **,int);
typedef short int (*donatouch_function) (double *, double *, double *, double *);

#define NORMAL_BIOCOL_MODE 0
#define SURFACE_DISTANCE_BIOCOL_MODE 1
#define MINIMUM_DISTANCE_BIOCOL_MODE 2

struct bcd_init_structure {
short int sc_free;
short int bb_free;
short int prev_bb_free;
short int loop;
short int new_disulphur;
Rigid_structure *rigid;
Rigid_structure *prev_rigid;
Rigid_structure *sc_rigid;
SDpt *inter_bb;
SDpt *intra_res;
SDpt *loops;
int autocol_index;
int intercol_index;
int loop_index;
Bboxpt last_bb_box;
Bboxpt prev_bb_box;
Bboxpt last_sc_box;
};

typedef struct {
  int *tab;
  int size;
} int_tab;

typedef struct ligandBBox {
    double min[3];
    double max[3];
} ligandBBox, *ligandBBoxPt;


typedef struct ligand {
  ligandBBox ligBBox; // ligand Axes oriented Bounding box
  double current_u_inv; //last invalid parameter  for a ligand localpath motion  
} ligand, *ligandPt;


extern ligand *LIGAND_PT;

#endif
