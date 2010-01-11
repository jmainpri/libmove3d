
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

