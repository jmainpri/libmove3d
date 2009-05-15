/**********************************************************************
 *                            LIGPDBTOP3D                             *
 *                           -------------                            *
 * Generates a .p3d file from a .pdb of the ligand with some added    *
 * lines to specify mobile dihedrals.                                 *
 *                                                                    *
 * By Juan Cortés (jcortes@laas.fr)                                   *
 **********************************************************************/


/**********************************************************************/
// INCLUDES

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**********************************************************************/
// DEFINITIONS (in "defs.h")

#define SQR(x) ((x)*(x))
#define PI 3.14159265358979323846
#define FUZZ  0.00001
#define ABS(x)  (((x) >= 0.0) ? (x) : -(x))
#define EQ(x,y) (ABS((x)-(y)) < FUZZ)

#define MAX_NAME_LENGTH 25

#define COVBOND_FACTOR 0.65

#define N_VDW_BONDS 3

#define INIT_MAX_NUM_PAIRS 200

/////////////////////////////////
#define C_VDWR 1.53
#define N_VDWR 1.38
#define O_VDWR 1.35
#define S_VDWR 1.79
#define H_VDWR 1.10

#define Br_VDWR 1.94
#define Cl_VDWR 1.74
#define F_VDWR 1.29
#define P_VDWR 1.87
#define I_VDWR 2.14
/////////////////////////////////
#define  N_COLOR "Blue"
#define  C_COLOR "Green"
#define  O_COLOR "Red"
#define  S_COLOR "Yellow"
#define  H_COLOR "White"
#define  F_COLOR "Blue2"
#define Br_COLOR "Any 153  83   4"
#define Cl_COLOR "Any 166 247 116"
#define  P_COLOR "Any 174  80 224"
#define  I_COLOR "Any  70  12 102"
/////////////////////////////////

/**********************************************************************/
// BOX DIMENSIONS
#define XBOX_HALF 50.0
#define YBOX_HALF 50.0
#define ZBOX_HALF 50.0

/**********************************************************************/

typedef enum {
  CARBON, NITROGEN, OXYGEN, SULFUR, HYDROGEN,
  FLUORINE, BROMINE, CHLORINE, PHOSPHORUS, IODINE
} atomTypes;

/**********************************************************************/
// STRUCTURES

#define MAXNBONS 6
typedef struct s_atom {
  int serial;
  char name[5];
  atomTypes atomType;
  double vdwR;
  char color[20];
  double pos[3];
  int nbonded;
  struct s_atom **bondedatomlist;   
  struct s_rigid *rigidPt;
  int njoints;
  struct s_joint **jointlist; // when atom is involved in joints
  // this list of flags is a simple tool to check if a bond as been traveled
  int bondflags[MAXNBONS];
  int index_in_rigid;
} atom;

typedef struct s_rigid {
  int natoms;
  int noutjnts;
  int ninjnts;
  struct s_atom **atomlist;
  struct s_joint **outjntslist;
  struct s_joint **injntslist;
  int indexparentj;
  int number_in_p3d;
} rigid;

typedef struct s_joint {
  struct s_atom *atomsindihedral[4];
  double vmin;
  double vmax;
} joint;


typedef struct s_ligand {
  char name[50];
  int natoms;
  int nrigids;
  int njoints;
  struct s_atom **atomlist;
  struct s_rigid **rigidlist;
  struct s_joint **jointlist;
  double oref[3];
} ligand;


/**********************************************************************/
// GLOBAL VARIABLES

// call arguments
//  - files
static char pdbfilename[255],p3dfilename[255],bcdfilename[255];
static FILE *pdbfile, *p3dfile, *bcdfile;

// number of read lines in PDF file 
static int npdfline = 0;

// counters
static int global_indJ = 0;
static int global_indR = 0;


/**********************************************************************/
/**********************************************************************/
// PDB TO LIGAND STRUCTURE
/**********************************************************************/
/**********************************************************************/

/**********************************************************************/
// ALLOC-FREE FUNCTIONS

static void alloc_and_init_ligand_struct(ligand **ligPtPt)
{
  ligand *ligPt;

  ligPt = (ligand *) malloc(sizeof(ligand));

  ligPt->natoms = 0;
  ligPt->nrigids = 0;
  ligPt->njoints = 0;  
  ligPt->atomlist = NULL;
  ligPt->rigidlist = NULL;
  ligPt->jointlist = NULL;

  *ligPtPt = ligPt;
}

static void alloc_and_init_rigid_struct(rigid **rigidPtPt)
{
  rigid * rigidPt;
  
  rigidPt = (rigid *) malloc(sizeof(rigid));

  rigidPt->natoms = 0;
  rigidPt->noutjnts = 0;
  rigidPt->ninjnts = 0;
  rigidPt->atomlist = NULL;
  rigidPt->outjntslist = NULL;
  rigidPt->injntslist = NULL;
  rigidPt->indexparentj = 0;   // filled when writing rigids in .p3d

  *rigidPtPt = rigidPt;
} 


static void free_ligand(ligand *ligPt)
{

  // TO DO !!! 
  
}

/**********************************************************************/
// LIST MANAGEMENT

static void insert_atom_in_list(atom *atomPt, atom ***atomlist, int *natoms)
{
  if(*natoms == 0) {
    *atomlist = (atom **) malloc(sizeof(atom *));
  }
  else {
    *atomlist = (atom **) realloc(*atomlist,sizeof(atom *) * (*natoms + 1));
  }
  (*atomlist)[*natoms] = atomPt;
  (*natoms)++;
}


static void insert_rigid_in_list(rigid *rigidPt, rigid ***rigidlist, int *nrigids)
{
  if(*nrigids == 0) {
    *rigidlist = (rigid **) malloc(sizeof(rigid *));
  }
  else {
    *rigidlist = (rigid **) realloc(*rigidlist,sizeof(rigid *) * (*nrigids + 1));
  }
  (*rigidlist)[*nrigids] = rigidPt;
  (*nrigids)++;
}


static void insert_joint_in_list(joint *jointPt, joint ***jointlist, int *njoints)
{
  if(*njoints == 0) {
    *jointlist = (joint **) malloc(sizeof(joint *));
  }
  else {
    *jointlist = (joint **) realloc(*jointlist,sizeof(joint *) * (*njoints + 1));
  }
  (*jointlist)[*njoints] = jointPt;
  (*njoints)++;
}

/**********************************************************************/

static int not_in_atom_list(atom *pt, atom **thelist, int nelemslist)
{
  int i;
  
  for(i=0; i<nelemslist; i++) {
    if(pt == thelist[i])
      return 0;
  }

  return 1;
}

/**********************************************************************/
// MATH

static void vectSub(double *a, double *b, double *c)
{  
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
}


static double vectNorm(double *v)
{
  return sqrt((double) ( SQR(v[0]) + SQR(v[1]) + SQR(v[2]) ) );
}


static void vectNormalize(double *src, double *dest)
{
  double l;

  l = vectNorm(src);
  dest[0] = src[0] / l;
  dest[1] = src[1] / l;
  dest[2] = src[2] / l;
}


static void normalized_vectXprod(double *a, double *b, double *c)
{
  double nnc[3];

  nnc[0] = a[1] * b[2] - a[2] * b[1];
  nnc[1] = a[2] * b[0] - a[0] * b[2];
  nnc[2] = a[0] * b[1] - a[1] * b[0];

  vectNormalize(nnc,c);
}


static double vectDotProd(double *a, double *b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


static int same_sign_vect(double *v1, double *v2)
{
  double v1n[3],v2n[3];

  vectNormalize(v1,v1n);
  vectNormalize(v2,v2n);
  if(vectDotProd(v1n,v2n) > 0.0)
    return 1;
  else
    return 0;
}


static void compute_distance(double *P1, double *P2, double *dist)
{
  double vdiff[3];

  vectSub(P1,P2,vdiff);
  *dist = vectNorm(vdiff);
}


static double compute_dihedang(double *nJa, double *tJa, double *pJa)
{
  // WARNING : vectors must be normalized !
  double axesXprod1[3],axesXprod2[3];
  double dirprod[3];
  double dihedang;

  normalized_vectXprod(pJa,tJa,axesXprod1);
  normalized_vectXprod(tJa,nJa,axesXprod2);
  dihedang = acos(vectDotProd(axesXprod1,axesXprod2)) * (180.0 / PI);
  normalized_vectXprod(axesXprod1,axesXprod2,dirprod);
  if(same_sign_vect(tJa,dirprod)) 
    return (dihedang);
  else
    return (-dihedang);
}


static void compute_ligand_centroid(ligand *ligPt, double *point)
{
  double x,y,z,M;
  int i;
  atom *atomPt;

  x = 0.0; y = 0.0; z = 0.0; M = 0.0;
  for(i=0; i<ligPt->natoms; i++) {
    atomPt = ligPt->atomlist[i];
    x += (atomPt->pos[0]) * (atomPt->vdwR);
    y += (atomPt->pos[1]) * (atomPt->vdwR);
    z += (atomPt->pos[2]) * (atomPt->vdwR);
    M += (atomPt->vdwR);
  }
  point[0] = x/M;
  point[1] = y/M;
  point[2] = z/M;
}

/**********************************************************************/
// ATOM MANAGEMENT FUNCTIONS

static int atomName_to_atomType(char *atomName, atomTypes *atomTypePt)
{
  if(strcmp(atomName,"C") == 0) { *atomTypePt = CARBON; return 1; }
  if(strcmp(atomName,"O") == 0) { *atomTypePt = OXYGEN; return 1; }
  if(strcmp(atomName,"N") == 0) { *atomTypePt = NITROGEN; return 1; }
  if(strcmp(atomName,"S") == 0) { *atomTypePt = SULFUR; return 1; }
  if(strcmp(atomName,"H") == 0) { *atomTypePt = HYDROGEN; return 1; }
  if(strcmp(atomName,"F") == 0) { *atomTypePt = FLUORINE; return 1; }
  if(strcmp(atomName,"Br") == 0) { *atomTypePt = BROMINE; return 1; }
  if(strcmp(atomName,"Cl") == 0) { *atomTypePt = CHLORINE; return 1; }
  if(strcmp(atomName,"P") == 0) { *atomTypePt = PHOSPHORUS; return 1; }
  if(strcmp(atomName,"I") == 0) { *atomTypePt = IODINE; return 1; }
  printf("ERROR : atom type %s is not defined",atomName);
  return -1;
}

/**********************************************************************/

static double get_vdwR_by_type(atomTypes atomType)
{
  switch(atomType) {
  case CARBON:
    return C_VDWR;
  case NITROGEN:
    return N_VDWR;
  case OXYGEN:
    return O_VDWR;
  case SULFUR:
    return S_VDWR;
  case HYDROGEN:
    return H_VDWR;
  case FLUORINE: 
    return F_VDWR;    
  case BROMINE:
    return Br_VDWR;
  case CHLORINE:
    return Cl_VDWR;   
  case PHOSPHORUS:
    return P_VDWR;   
  case IODINE:
    return I_VDWR;   
  default:
    return -1;
  }  
}

/**********************************************************************/

static char *get_color_by_type(atomTypes atomType)
{
  switch(atomType) {
  case CARBON:
    return C_COLOR;
  case NITROGEN:
    return N_COLOR;
  case OXYGEN:
    return O_COLOR;
  case SULFUR:
    return S_COLOR;
  case HYDROGEN:
    return H_COLOR;
  case FLUORINE: 
    return F_COLOR;    
  case BROMINE:
    return Br_COLOR;
  case CHLORINE:
    return Cl_COLOR;   
  case PHOSPHORUS:
    return P_COLOR;   
  case IODINE:
    return I_COLOR;   
  default:
    return NULL;
  }  
}

/**********************************************************************/

atom *get_atom_in_list_by_serial(int serial, atom **atomlist, double natoms)
{
  int i;
  atom *atomPt;

  for(i=0; i<natoms; i++) {
    if(atomlist[i]->serial == serial) {
      atomPt = atomlist[i];
      break;
    }      
  }
  return atomPt;
}

/**********************************************************************/

atom *get_closest_atom_to_point(atom **atomlist, double natoms, double *point)
{
  atom *bestatomPt;
  double dist, bestdist;
  int i;

  bestdist = HUGE_VAL;
  bestatomPt = NULL;
  for(i=0; i<natoms; i++) {
    compute_distance(atomlist[i]->pos,point,&dist);
    if(dist < bestdist) {
      bestdist = dist;
      bestatomPt = atomlist[i];
    }
  }
  return bestatomPt;
}

/**********************************************************************/
// BOND MANAGEMENT FUNCTIONS

static void mark_atoms_as_bonded(atom *atomPt_i, atom *atomPt_j)
{
  atom *atomcouple[2];
  int i;

  // init couple
  atomcouple[0] = atomPt_i;
  atomcouple[1] = atomPt_j;
  for(i=0; i<2; i++) {
    insert_atom_in_list(atomcouple[1],&(atomcouple[0]->bondedatomlist),&(atomcouple[0]->nbonded));
    // switch couple
    atomcouple[0] = atomPt_j;
    atomcouple[1] = atomPt_i;
  }
}


static void identify_and_mark_bonded_atoms(atom **atomlist, double natoms)
{
  atom *atomPt_i, *atomPt_j;
  double dist;
  int i,j;
  
  for(i=0; i<(natoms - 1); i++) {
    atomPt_i = atomlist[i];
    for(j=i+1; j<natoms; j++) {
      atomPt_j = atomlist[j];
      compute_distance(atomPt_i->pos,atomPt_j->pos,&dist);
      if(dist < (atomPt_i->vdwR + atomPt_j->vdwR) * COVBOND_FACTOR) {
	mark_atoms_as_bonded(atomPt_i,atomPt_j);
      }
    }
  }
}

/**********************************************************************/

static int bond_has_been_traveled(atom *atomPt_i, atom *atomPt_j)
{
  int i,ibond;
  
  for(i=0; i<(atomPt_i->nbonded); i++) {
    if(atomPt_i->bondedatomlist[i] == atomPt_j) {
      ibond = i;
      break;
    }
  }
  return (atomPt_i->bondflags[ibond]);
}


static int in_joint_axis(atom *atomPt_i, atom *atomPt_j, joint **bondjntPtPt)
{
  joint *jointPt_i,*jointPt_j;
  int i,j;
  
  for(i=0; i<(atomPt_i->njoints); i++) {
    jointPt_i = atomPt_i->jointlist[i];
    for(j=0; j<(atomPt_j->njoints); j++) {
      jointPt_j = atomPt_j->jointlist[j];
      if((jointPt_i == jointPt_j) &&
	 (((jointPt_i->atomsindihedral[1] == atomPt_i) &&
	   (jointPt_i->atomsindihedral[2] == atomPt_j)) ||
	  ((jointPt_i->atomsindihedral[2] == atomPt_i) &&
	   (jointPt_i->atomsindihedral[1] == atomPt_j)))) {
	*bondjntPtPt = jointPt_i;
	return 1;
      }
    }
  }

  *bondjntPtPt = NULL;
  return 0;
}


/**********************************************************************/

static rigid *get_rigid_connected_to_rigid_by_joint(rigid *rigidPt, joint *jointPt)
{
  // NOTE : the connection is made by the atoms in the joint axis
  if(jointPt->atomsindihedral[1]->rigidPt != rigidPt)
    return jointPt->atomsindihedral[1]->rigidPt;
  else
    return jointPt->atomsindihedral[2]->rigidPt;
}

/**********************************************************************/
/**********************************************************************/

// checks the direction of the joint axis 
// and modifies the order of the atoms forming dihedral angle if necessary
static int check_injnt_direction(rigid *rigidPt, joint *jointPt)
{
  int i,j;
  atom *atomPt;

  if(jointPt->atomsindihedral[2]->rigidPt == rigidPt) {
    // right direction
    return 1; 
  }
  else {
    // inverse direction
    for(i=0,j=3; i<2; i++,j--) {
      atomPt = jointPt->atomsindihedral[j];
      jointPt->atomsindihedral[j] = jointPt->atomsindihedral[i];
      jointPt->atomsindihedral[i] = atomPt;
    }
    printf("WARNING : inversed dihedral jnt direction : ");
    for(i=0; i<4; i++) 
      printf("%d ", jointPt->atomsindihedral[i]->serial);
    printf("\n");
    return 0;
  }
}

/**********************************************************************/
/**********************************************************************/

static void insert_next_rigids_in_list(rigid *rigidPt, rigid ***rigidlist, int *nrigids)
{
  rigid *nextrPt;
  int i;
  
  for(i=0; i<rigidPt->noutjnts; i++) {    
    nextrPt = get_rigid_connected_to_rigid_by_joint(rigidPt,rigidPt->outjntslist[i]);
    insert_rigid_in_list(nextrPt,rigidlist,nrigids);
  }
}


/**********************************************************************/
// PDB READING FUNCTIONS

static int read_and_treat_atom(char *pdbline, ligand *ligPt)
{
  char piece[10];
  char typeinchar[3];
  int i;
  atom *atomPt;

  atomPt = (atom *) malloc(sizeof(atom));
  
  // extract only used data
  strcpy(piece,"         ");
  strncpy(piece,pdbline+6,5);
  if(sscanf(piece,"%d",&atomPt->serial) < 0) {free(atomPt); return -1;}
  strcpy(piece,"         ");
  strncpy(piece,pdbline+12,4);
  if(sscanf(piece,"%s",atomPt->name) < 0) {free(atomPt); return -1;}
  strcpy(piece,"         ");
  strncpy(piece,pdbline+30,8);
  if(sscanf(piece,"%lf",&atomPt->pos[0]) < 0) {free(atomPt); return -1;}
  strcpy(piece,"         ");
  strncpy(piece,pdbline+38,8);
  if(sscanf(piece,"%lf",&atomPt->pos[1]) < 0) {free(atomPt); return -1;}
  strcpy(piece,"         ");
  strncpy(piece,pdbline+46,8);
  if(sscanf(piece,"%lf",&atomPt->pos[2]) < 0) {free(atomPt); return -1;} 
  strcpy(piece,"         ");
  strncpy(piece,pdbline+76,2);
  if(sscanf(piece,"%s",typeinchar) < 0) {free(atomPt); return -1;} 

  if(atomName_to_atomType(typeinchar, &atomPt->atomType) < 0) {
    {free(atomPt); return -1;}
  }
  atomPt->vdwR = get_vdwR_by_type(atomPt->atomType);
  strcpy(atomPt->color, get_color_by_type(atomPt->atomType));

  if(atomPt->vdwR == -1) {
    return -1;
  }

  // init rigidPt
  atomPt->rigidPt = NULL;
  // init jointlist
  atomPt->njoints = 0;
  atomPt->jointlist = NULL;
  // init list of bonded atoms
  atomPt->nbonded = 0;
  atomPt->bondedatomlist = NULL;
  // init bondflags
  for(i=0; i<6; i++) {
    atomPt->bondflags[i] = 0;
  }
  
  insert_atom_in_list(atomPt,&(ligPt->atomlist),&(ligPt->natoms));
  return 1;
}

////////////////////////////  BEGIN UNUSED /////////////////////////////////////////
/* RIGID record
   ------------
   CONTAINS : 
     - columns 7-11, 12-16, 17-21 ... : atom serials 
   WARNING  : using 80-character lines -> the rigid can contain up to 14 atoms !!!
   EXAMPLE  :
            1         2         3         4         5         6         7         8
   12345678901234567890123456789012345678901234567890123456789012345678901234567890
   RIGID     7    8    9   10   11                    
*/
static int read_and_treat_rigid(char *pdbline, ligand *ligPt)
{
  char piece[10];
  int state;
  int serial;
  rigid *rigidPt;
  atom *atomPt;
  int i;
  
  alloc_and_init_rigid_struct(&rigidPt);
  
  state = 0;
  i = 0;
  do {
    strcpy(piece,"         ");  
    strncpy(piece,pdbline+6,5*i);
    if(sscanf(piece,"%d",&serial) < 0) {
      state = 1;
    }
    else {
      atomPt = get_atom_in_list_by_serial(serial,ligPt->atomlist,ligPt->natoms);
      if(atomPt == NULL) {
	printf("ERROR : in RIGID definition : atom serial %d is not defined\n",serial);
	return -1;
      }
      insert_atom_in_list(atomPt,&(rigidPt->atomlist),&(rigidPt->natoms));
    }
    i++;
  } while(state == 1);

  insert_rigid_in_list(rigidPt,&(ligPt->rigidlist),&(ligPt->nrigids));
  return 1;
}  
//////////////////////////////  END UNUSED /////////////////////////////////////////


/* JOINT record
   ------------
   CONTAINS :
     - columns  7-11 : atom serial of "base" rigid
     - columns 12-16 : atom serial of connected rigid
     - columns 17-21 : atom serial of connected rigid
     - columns 22-26 : atom serial of connected rigid
     - columns 27-34 : lower joint limit (degrees)
     - columns 35-42 : uper joint limit (degrees)
   NOTE1    : several intervals ???
   NOTE2    : parent joint ???
   WARNING  : up to 3 intervals ???
   EXAMPLE  :
            1         2         3         4         5         6         7         8
   12345678901234567890123456789012345678901234567890123456789012345678901234567890
   JOINT     7   15   17   22 -180.00  180.00
*/
 static int read_and_treat_joint(char *pdbline, ligand *ligPt)
{
  char piece[10];
  joint *jointPt;
  int serial;
  int i;

  jointPt = (joint *) malloc(sizeof(joint));

  strcpy(piece,"         ");
  strncpy(piece,pdbline+7,5);
  if(sscanf(piece,"%d",&serial) < 0) return -1;
  jointPt->atomsindihedral[0] = get_atom_in_list_by_serial(serial,ligPt->atomlist,ligPt->natoms);
  if(jointPt->atomsindihedral[0] == NULL) {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    return -1;
  }
  strcpy(piece,"         ");
  strncpy(piece,pdbline+12,5);
  if(sscanf(piece,"%d",&serial) < 0) return -1;
  jointPt->atomsindihedral[1] = get_atom_in_list_by_serial(serial,ligPt->atomlist,ligPt->natoms);
  if(jointPt->atomsindihedral[1] == NULL) {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    return -1;
  }
  strcpy(piece,"         ");
  strncpy(piece,pdbline+17,5);
  if(sscanf(piece,"%d",&serial) < 0) return -1;
  jointPt->atomsindihedral[2] = get_atom_in_list_by_serial(serial,ligPt->atomlist,ligPt->natoms);
  if(jointPt->atomsindihedral[2] == NULL) {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    return -1;
  }
  strcpy(piece,"         ");
  strncpy(piece,pdbline+22,5);
  if(sscanf(piece,"%d",&serial) < 0) return -1;
  jointPt->atomsindihedral[3] = get_atom_in_list_by_serial(serial,ligPt->atomlist,ligPt->natoms);
  if(jointPt->atomsindihedral[3] == NULL) {
    printf("ERROR : while reading joints : atom serial %d is not defined\n",serial);
    return -1;
  }
  strcpy(piece,"         ");
  strncpy(piece,pdbline+27,7);
  if(sscanf(piece,"%lf",&(jointPt->vmin)) < 0) return -1;
  strcpy(piece,"         ");
  strncpy(piece,pdbline+35,7);
  if(sscanf(piece,"%lf",&(jointPt->vmax)) < 0) return -1;
  strcpy(piece,"         ");

  // set jointPt in atoms involved by this joint
  for(i=0; i<4; i++) {
    insert_joint_in_list(jointPt,&(jointPt->atomsindihedral[i]->jointlist),
			 &(jointPt->atomsindihedral[i]->njoints));
  }
  
  insert_joint_in_list(jointPt,&(ligPt->jointlist),&(ligPt->njoints));
  return 1;
}


static int read_and_treat_pdb_line(FILE *pdbfile, ligand *ligPt)
{
  char *pdbline;
  char piece[10];
  char rec[10];
  char returned_s[100];
  int state;

  pdbline = fgets(returned_s,100,pdbfile);
  if(pdbline == NULL) {
     return -1;
  }
  npdfline++;
    
  strcpy(piece,"         ");
  strncpy(piece,pdbline,6);
  if(sscanf(piece,"%s",rec) < 0) return -2;
  if(strcmp(rec,"ATOM") == 0) 
    state = read_and_treat_atom(pdbline,ligPt);
  else if(strcmp(rec,"RIGID") == 0)
    state = read_and_treat_rigid(pdbline,ligPt);
  else if(strcmp(rec,"JOINT") == 0)
    state = read_and_treat_joint(pdbline,ligPt);
  else 
    state = 0;

  return state;
}

/**********************************************************************/
// RIGID CONSTRUCTION

// WARNING : POSSIBLE PROBLEMS WITH RIGID CYCLES ???
static int construct_rigid_from_root_atom(ligand *ligPt, atom *rootaPt, 
					  atom ***nextrootalist, int *nnextroota)
{
  rigid *rigidPt;
  joint *bondjntPt;
  atom **curatomlist;
  atom **nextatomlist, **auxlist;
  atom *bondedaPt;
  int natoms;
  int nnextatoms;
  int i,j;
  int index_in_rigid = 0;
  
  alloc_and_init_rigid_struct(&rigidPt);

  // insert root atom in rigid
  insert_atom_in_list(rootaPt,&(rigidPt->atomlist),&(rigidPt->natoms));
  rootaPt->rigidPt = rigidPt;
  rootaPt->index_in_rigid = index_in_rigid;
  index_in_rigid++;

  natoms = 1;
  curatomlist = (atom**) malloc(sizeof(atom *));
  curatomlist[0] = rootaPt;

  while(natoms > 0) {
    nnextatoms = 0;
    nextatomlist = NULL;
    for(i=0; i<natoms; i++) {  
      for(j=0; j<curatomlist[i]->nbonded; j++) {  
	bondedaPt = curatomlist[i]->bondedatomlist[j];
	if(bondedaPt->rigidPt == NULL) {
	  // atoms defining the connection between to rigids are those 
	  // in dihedral angle axis (i.e. 2nd and 3rd in the list)
	  if(in_joint_axis(bondedaPt,curatomlist[i],&bondjntPt)) {
	    insert_atom_in_list(bondedaPt,nextrootalist,nnextroota);
	    // set "out-joint" pointer
	    insert_joint_in_list(bondjntPt,&(rigidPt->outjntslist),&(rigidPt->noutjnts));
	  }
	  else {
	    insert_atom_in_list(bondedaPt,&(nextatomlist),&nnextatoms);
	    insert_atom_in_list(bondedaPt,&(rigidPt->atomlist),&(rigidPt->natoms));
	    bondedaPt->rigidPt = rigidPt;
	    bondedaPt->index_in_rigid = index_in_rigid;
	    index_in_rigid++;
	  }
	}
	else {
	  // if bonded atom is alredy in current rigid -> do nothing 
	  // otherwise :
	  if(bondedaPt->rigidPt != rigidPt) {
	    // if the bond has been traveled yet -> set "in-joint" pointer (bodies linked in open chain)
	    if(!bond_has_been_traveled(bondedaPt,curatomlist[i])) {
	      if(in_joint_axis(bondedaPt,curatomlist[i],&bondjntPt)) {
		insert_joint_in_list(bondjntPt,&(rigidPt->injntslist),&(rigidPt->ninjnts));
	      }
	      else {
		// PUEDE SUCEDER !!!???
		printf("Ooo oooooo\n");
	      }
	    }
	    // otherwise :
	    else {
	      printf("WARNING : closed kinematic chain : this case is not automatically treated\n");
	      printf("          A closure constraint must be considerer between atoms %d and %d\n",
		     curatomlist[i]->serial,bondedaPt->serial);
	    }
	  }
	  curatomlist[i]->bondflags[j] = 1;
	}
      }
    }
    natoms = nnextatoms;
    auxlist = curatomlist;
    curatomlist = nextatomlist;
    nextatomlist = auxlist;
    free(nextatomlist);
  }
  free(curatomlist);

  insert_rigid_in_list(rigidPt,&(ligPt->rigidlist),&(ligPt->nrigids));
  return 1;
}

/**********************************************************************/

// generating rigids : two possibilities
//  1.- The rigids are defined by the user (record RIGID)
//  2.- The rigids are automatically identified
static int generate_rigids(ligand *ligPt)
{
  atom **currootalist;
  atom **nextrootalist, **auxlist;
  int nroota;
  int nnextroota;
  int state;
  int i;

  // 1.
  if(ligPt->rigidlist != NULL) {
    // NOT MADE YET !
    // NOTES : - all the atoms should be included in rigids
    //         - the order in which rigids are defined should correspond 
    //           with the order for the articulated model
    printf("SORRY, the rigids are automatically generated in this version.\n");
    return -1;
  }
  // 2.
  else {
    // identify bonded atoms
    identify_and_mark_bonded_atoms(ligPt->atomlist,ligPt->natoms);
    // choose atom for first rigid
    nroota = 1;
    currootalist = (atom**) malloc(sizeof(atom *));
    currootalist[0] = get_closest_atom_to_point(ligPt->atomlist, ligPt->natoms, ligPt->oref);
    if(currootalist[0] == NULL) {
      printf("ERROR : problem computing distances : unable to choose best atom\n");
      return -1;
    }
    // construct rigids iteratively 
    // NOTE : a "breadth-first search" is used to construct the ligand
    //        (i.e. rigids are identified "level by level")
    while(nroota > 0) {
      nnextroota = 0;
      nextrootalist = NULL;
      for(i=0; i<nroota; i++) {
	state = construct_rigid_from_root_atom(ligPt,currootalist[i],&nextrootalist,&nnextroota);
	if(state < 0) {
	  free(nextrootalist);
	  free(currootalist);
	  return -1;
	}
      }
      nroota = nnextroota;
      auxlist = currootalist;
      currootalist = nextrootalist;
      nextrootalist = auxlist;
      free(nextrootalist);
    }
    free(currootalist);
  }
  return 1;
}

  
/**********************************************************************/
// LIGAND STRUCTURE WRITING FUNCTIONS

static int fill_ligand_struct(FILE * pdbfile, ligand *ligPt)
{
  int state;

  // ligand name
  strcpy(ligPt->name,pdbfilename);

  // read all pdb lines and generate basic structures 
  // IMPORTANT NOTE : all ATOM lines must precede JOINT (AND RIGID) lines !!!
  state = 1;
  while(state >= 0) {
    state = read_and_treat_pdb_line(pdbfile,ligPt);
  }

  // compute oref 
  compute_ligand_centroid(ligPt, ligPt->oref);
  
  // generate rigids 
  // NOTE : connections between rigids are also identified within this function
  if(generate_rigids(ligPt) < 0) {
    return -1;
  }
  return 1;
}  


/**********************************************************************/
/**********************************************************************/
// LIGAND STRUCTURE TO P3D
/**********************************************************************/
/**********************************************************************/

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

static void write_p3d_head(FILE *p3dfile, ligand *ligPt)
{
  fprintf(p3dfile,"p3d_beg_desc P3D_ENV %s\n\n",p3dfilename);
}


static void write_p3d_end(FILE *p3dfile, ligand *ligPt)
{
  fprintf(p3dfile,"p3d_end_desc\n\n");     
  fprintf(p3dfile,"\n\np3d_set_env_box %f %f %f %f %f %f\n\n",
	  ligPt->oref[0] - XBOX_HALF, ligPt->oref[0] + XBOX_HALF,
	  ligPt->oref[1] - XBOX_HALF, ligPt->oref[1] + XBOX_HALF,
	  ligPt->oref[2] - XBOX_HALF, ligPt->oref[2] + XBOX_HALF);  
}

/**********************************************************************/

static void write_ligand_head(FILE *p3dfile, ligand *ligPt)
{
  int indexletter=0;
  int indref;
  int numlig;

  // ONLY ONE ROBOT
  // robot mane must end with ".lig" (required by BCD)
  fprintf(p3dfile,"p3d_beg_desc P3D_ROBOT %s.lig\n\n",givemeword(ligPt->name,'.',&indexletter));

  // base-joint
  // NOTE : A free-flying joint is defined as base-joint of a ligand.
  //        Default values are given to joint limits.
  
  indref = 0; // the joint is relative to jnt[0] 
  numlig = 1; // WARNING : ONLY ONE LIGAND NOW !!!
  
  // update global_indJ
  global_indJ++;

  fprintf(p3dfile,"p3d_beg_desc_jnt P3D_FREEFLYER    # %s # J%d\n","LIGAND BASE-JOINT",global_indJ);
  fprintf(p3dfile,"  p3d_set_name .lig_base.%d\n",numlig);
  fprintf(p3dfile,"  p3d_set_pos_axe %f %f %f 0 0 0\n",ligPt->oref[0],ligPt->oref[1],ligPt->oref[2]);
  fprintf(p3dfile,"  p3d_set_prev_jnt %d\n",indref);
  fprintf(p3dfile,"  p3d_set_dof 0 0 0 0 0 0\n");
  fprintf(p3dfile,"  p3d_set_dof_vmin %f %f %f %f %f %f\n",
	  -XBOX_HALF,-YBOX_HALF,-ZBOX_HALF,-180.0,-180.0,-180.0);
  fprintf(p3dfile,"  p3d_set_dof_vmax %f %f %f %f %f %f\n",
	  XBOX_HALF, YBOX_HALF, ZBOX_HALF, 180.0, 180.0, 180.0);
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
  
  // 

}

/**********************************************************************/

static void write_joint_with_updates(FILE *p3dfile, joint *jointPt, int indref)
{
  double *a1pos,*a2pos,*a3pos,*a4pos;
  double posdiff[3],axis1[3],axis2[3],axis3[3];
  double dihedang;
  char   jname[255];
  char   cadchar[255];

  a1pos = jointPt->atomsindihedral[0]->pos;
  a2pos = jointPt->atomsindihedral[1]->pos;
  a3pos = jointPt->atomsindihedral[2]->pos;
  a4pos = jointPt->atomsindihedral[3]->pos;

  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,axis2);
  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,axis1);
  vectSub(a4pos,a3pos,posdiff);
  vectNormalize(posdiff,axis3);

  dihedang = compute_dihedang(axis3,axis2,axis1);

  // update global_indJ
  global_indJ++;

  strcpy(jname,"theta.LIG");
  sprintf(cadchar,".%d",global_indJ);
  strcat(jname,cadchar);
  sprintf(cadchar,".%d-%d-%d-%d",
	  jointPt->atomsindihedral[0]->serial,jointPt->atomsindihedral[1]->serial,
	  jointPt->atomsindihedral[2]->serial,jointPt->atomsindihedral[3]->serial);
  strcat(jname,cadchar);

  write_joint(jname,global_indJ,indref,a3pos,axis2,dihedang,dihedang,jointPt->vmin,jointPt->vmax);
}

/**********************************************************************/

static void write_atom(FILE *p3dfile, atom *atomPt)
{
  fprintf(p3dfile," p3d_add_desc_sphere V-%s.%d %f\n",atomPt->name,atomPt->serial,atomPt->vdwR);    
  fprintf(p3dfile," p3d_set_prim_pos V-%s.%d %f %f %f 0.0 0.0 0.0\n",
	  atomPt->name,atomPt->serial,atomPt->pos[0],atomPt->pos[1],atomPt->pos[2]);
  fprintf(p3dfile," p3d_set_prim_color V-%s.%d %s\n",atomPt->name,atomPt->serial,atomPt->color);
}

/**********************************************************************/

static void write_rigid_with_updates(FILE *p3dfile, rigid *rigidPt)
{
  int numlig;
  int i;
  rigid *nextrPt;
  
  numlig = 1; // WARNING : ONLY ONE LIGAND NOW !!!

  rigidPt->number_in_p3d = global_indR;
  fprintf(p3dfile,"p3d_beg_desc P3D_BODY rigid-%d.LIG.%d\n",global_indR,numlig);
  for(i=0; i<rigidPt->natoms; i++) {    
    write_atom(p3dfile,rigidPt->atomlist[i]);
  }
  fprintf(p3dfile,"p3d_end_desc\n\n");

  // update global_indR
  global_indR ++;	

  // idex of parent joint in next rigids
  // WARNING : this translator currently works ONLY with kinematic trees 
  //           (i.e. rigidPt->ninjnts in ALWAYS 1)
  // NOTE : global_indJ corresponds with the index of the current parent joint
  for(i=0; i<rigidPt->noutjnts; i++) {    
    nextrPt = get_rigid_connected_to_rigid_by_joint(rigidPt,rigidPt->outjntslist[i]);
    nextrPt->indexparentj = global_indJ;
  }
}

/**********************************************************************/

static int write_ligand_kinematic_tree_from_root(FILE *p3dfile, ligand *ligPt,
						 rigid *rootrPt)
{
  rigid **currigidlist;
  rigid **nextrigidlist, **auxlist;
  int nrigids;
  int nnextrigids;
  int i;

  write_rigid_with_updates(p3dfile,rootrPt);

  nrigids = 0;
  currigidlist = NULL;
  // insert next rigids in list
  insert_next_rigids_in_list(rootrPt,&currigidlist,&nrigids);

  while(nrigids > 0) {
    nnextrigids = 0;
    nextrigidlist = NULL;
    for(i=0; i<nrigids; i++) {
      if(currigidlist[i]->ninjnts != 1) {
	printf("ERROR : wrong kinematic structure : a joint can have only one parent\n");
	free(nextrigidlist);
	free(currigidlist);
	return -1;
      }
      check_injnt_direction(currigidlist[i],currigidlist[i]->injntslist[0]);
      write_joint_with_updates(p3dfile,currigidlist[i]->injntslist[0],currigidlist[i]->indexparentj);
      write_rigid_with_updates(p3dfile,currigidlist[i]);
      // insert next rigids in list
      insert_next_rigids_in_list(currigidlist[i],&nextrigidlist,&nnextrigids);
    }
    nrigids = nnextrigids;
    auxlist = currigidlist;
    currigidlist = nextrigidlist;
    nextrigidlist = auxlist;
    free(nextrigidlist);
  }
  free(currigidlist);
      
  return 1;
}

/**********************************************************************/
// 

static int write_p3d_file(FILE *p3dfile, ligand *ligPt)
{
  int state; 

  // write .p3d head
  write_p3d_head(p3dfile,ligPt);

  // write ligand head
  write_ligand_head(p3dfile,ligPt);

  // write rigids and joints 
  state = write_ligand_kinematic_tree_from_root(p3dfile,ligPt,ligPt->rigidlist[0]);
  
  // write .p3d end
  write_p3d_end(p3dfile,ligPt);

  return state;
}

/**********************************************************************/
/**********************************************************************/
// Write BioCD init file (.bcd) 
/**********************************************************************/
/**********************************************************************/

static int generate_unvisited_vdw_bonded_atom_list_outside_rigid(rigid *rigidPt, atom *atomPt,
								 int *nvdwbondedatomsPt,
								 atom ***vdwbondedatomlistPt)
{
  int i,j;
  int level;
  int nbondedatoms_curlevel;
  atom **listbondedatoms_curlevel;
  int nbondedatoms_nextlevel=0;
  atom **listbondedatoms_nextlevel=NULL;
  int ntreatedatoms=0;
  atom **listtreatedatoms=NULL;

  // NOTE : breadth-first search 
  
  //insert_atom_in_list(atomPt,&listtreatedatoms,&ntreatedatoms);
  nbondedatoms_curlevel = atomPt->nbonded;
  listbondedatoms_curlevel = atomPt->bondedatomlist;
  for(level=0; level<N_VDW_BONDS; level++) {    
    for(i=0; i<nbondedatoms_curlevel; i++) {
      insert_atom_in_list(listbondedatoms_curlevel[i],&listtreatedatoms,&ntreatedatoms);
      if((listbondedatoms_curlevel[i]->rigidPt != rigidPt) && 
	 (listbondedatoms_curlevel[i]->rigidPt->number_in_p3d > rigidPt->number_in_p3d)) {
	insert_atom_in_list(listbondedatoms_curlevel[i],vdwbondedatomlistPt,nvdwbondedatomsPt);
      }
      for(j=0; j<listbondedatoms_curlevel[i]->nbonded; j++) {
	if(not_in_atom_list(listbondedatoms_curlevel[i]->bondedatomlist[j],listtreatedatoms,ntreatedatoms)) {
	  insert_atom_in_list(listbondedatoms_curlevel[i]->bondedatomlist[j],&listbondedatoms_nextlevel,&nbondedatoms_nextlevel);
	}
      }
    }

    // free curlevel list    
    if((level > 0) && (nbondedatoms_curlevel > 0)) {
      free(listbondedatoms_curlevel);
      listbondedatoms_curlevel = NULL;
    }

    // update lists
    nbondedatoms_curlevel = nbondedatoms_nextlevel;
    listbondedatoms_curlevel = listbondedatoms_nextlevel;
    nbondedatoms_nextlevel = 0;
    listbondedatoms_nextlevel = NULL;
  }
  // free listtreatedatoms
  free(listtreatedatoms);
  ntreatedatoms = 0;
  
  if((level-1 > 0) && (nbondedatoms_curlevel > 0)) {
    free(listbondedatoms_curlevel);
    listbondedatoms_curlevel = NULL;
  }

  return 1;
}

/**********************************************************************/

static int order_vdw_bonded_atom_list(int natoms, atom **listPt) 
{
  int currigidnum;
  int curatomindex;
  int i,j,k;
  atom *auxatomPt;
  
  for(i=0; i<natoms-1; i++) {
    currigidnum = listPt[i]->rigidPt->number_in_p3d;
    curatomindex = listPt[i]->index_in_rigid;
    for(j=i+1; j<natoms; j++) {      
      if((listPt[j]->rigidPt->number_in_p3d < currigidnum) ||
	 (listPt[i]->index_in_rigid < curatomindex)) {
	auxatomPt = listPt[j];
	for(k=j; k>i; k--) {
	  listPt[k] = listPt[k-1];
	}
	listPt[i] = auxatomPt;
	currigidnum = listPt[i]->rigidPt->number_in_p3d;
	curatomindex = listPt[i]->index_in_rigid;
      }
    }
  }    
  return 1;
}


/**********************************************************************/

static int write_bcd_file(FILE *bcdfile, ligand *ligPt)
{
  int i,j,k;
  int nr;
  rigid *rigidPt;
  atom *atomPt;
  int nvdwbondedatoms = 0;
  atom **vdwbondedatomlist = NULL;
  int totnumpairs = 0;
  int max_num_pairs = INIT_MAX_NUM_PAIRS;
  int *r1list,*r2list,*a1list,*a2list;

  // alloc arrays for collision pairs
  r1list = (int *) malloc(sizeof(int) * max_num_pairs);
  r2list = (int *) malloc(sizeof(int) * max_num_pairs);
  a1list = (int *) malloc(sizeof(int) * max_num_pairs);
  a2list = (int *) malloc(sizeof(int) * max_num_pairs);
  
  // write .bcd head
  // NECESSARY ?
  
  // treat each rigid following the .p3d order
  for(nr=0; nr<ligPt->nrigids; nr++) {
    i = 0;
    while(ligPt->rigidlist[i]->number_in_p3d != nr)
      i++;
    rigidPt = ligPt->rigidlist[i];
    // treat each atom
    for(j=0; j<rigidPt->natoms; j++) {
      atomPt = rigidPt->atomlist[j];
      generate_unvisited_vdw_bonded_atom_list_outside_rigid(rigidPt,atomPt,&nvdwbondedatoms,&vdwbondedatomlist);
      order_vdw_bonded_atom_list(nvdwbondedatoms,vdwbondedatomlist);
      // insert data in arrays for collision pairs
      for(k=0; k<nvdwbondedatoms; k++) {
	r1list[totnumpairs] = nr;
	a1list[totnumpairs] = atomPt->index_in_rigid;
	r2list[totnumpairs] = vdwbondedatomlist[k]->rigidPt->number_in_p3d;
	a2list[totnumpairs] = vdwbondedatomlist[k]->index_in_rigid;
	totnumpairs++;
	if(totnumpairs == max_num_pairs) {
	  max_num_pairs *= 2;
	  r1list = (int *) realloc(r1list, sizeof(int) * max_num_pairs);
	  r2list = (int *) realloc(r2list, sizeof(int) * max_num_pairs);
	  a1list = (int *) realloc(a1list, sizeof(int) * max_num_pairs);
	  a2list = (int *) realloc(a2list, sizeof(int) * max_num_pairs);	  
	}
      }
      // free vdwbondedatomlist
      if(nvdwbondedatoms > 0) {
        free(vdwbondedatomlist);	
        nvdwbondedatoms = 0;		
      }
    }
  }

  // print arrays for collision pairs
  fprintf(bcdfile,"%d\n",totnumpairs);  
  for(i=0; i<totnumpairs; i++) {
    fprintf(bcdfile,"%d ",r1list[i]);
  }
  fprintf(bcdfile,"\n");  
  for(i=0; i<totnumpairs; i++) {
    fprintf(bcdfile,"%d ",a1list[i]);
  }
  fprintf(bcdfile,"\n");  
  for(i=0; i<totnumpairs; i++) {
    fprintf(bcdfile,"%d ",r2list[i]);
  }
  fprintf(bcdfile,"\n");  
  for(i=0; i<totnumpairs; i++) {
    fprintf(bcdfile,"%d ",a2list[i]);
  }
  fprintf(bcdfile,"\n");  

  return 1;
}


/**********************************************************************/
/**********************************************************************/
// MAIN
/**********************************************************************/
/**********************************************************************/

/**********************************************************************/
// GENERAL FUNCTIONS

static int read_call_arguments(int argc, char **argv)
{
   if(argc < 3)
   {
      printf("Not enought arguments\n");
      return -1;
   }

   if((!strcpy(pdbfilename,argv[1])) ||
      (!strcpy(p3dfilename,argv[2])))
     return -1;

   if(argc > 3) {
     sscanf(argv[3],"%d",&global_indJ);
   }  

   return 1;
}

/**********************************************************************/

int main(int argc, char **argv)
{
  ligand *ligPt;
  char *filename_part;
  int indexletter=0;

  if(read_call_arguments(argc, argv) < 0) {
    printf("Usage: ligpdbtop3d <pdbfile> <p3dfile> [index last protein jnt]\n");
    return -1;
  }

  // open files
  pdbfile = fopen(pdbfilename, "r");
  if (pdbfile == NULL) {
    printf("pdb file cannot be open\n");
    return -1;
  }
  p3dfile = fopen(p3dfilename, "w");
  if (p3dfile == NULL) {
    printf("p3d file cannot be open\n");
    fclose(pdbfile);
    return -1;
  }

  // alloc ligand
  alloc_and_init_ligand_struct(&ligPt);

  // read PDB and write ligand data structure
  if(fill_ligand_struct(pdbfile,ligPt) < 0) {
    printf("ERROR while writing ligand data structure from PDB\n");
    free_ligand(ligPt);  
    fclose(pdbfile);
    fclose(p3dfile);
    return -1;
  }
  printf("PSF initialization OK  ...\n");

  // write .p3d file from ligand data structure
  if(write_p3d_file(p3dfile,ligPt) < 0) {
    printf("ERROR while writing .p3d file from ligand data structure\n");
    free_ligand(ligPt);  
    fclose(pdbfile);
    fclose(p3dfile);
    return -1;    
  }
  printf("P3D file written ...\n");

  // write .bcd file from ligand data structure
  // - create bcd file : name = p3dfilename but with .bcd extension
  indexletter = 0;
  filename_part = givemeword(p3dfilename, '.', &indexletter);      
  strcpy(bcdfilename,filename_part);
  strcat(bcdfilename,".bcd");  
  bcdfile = fopen(bcdfilename, "w");
  if (bcdfile == NULL) {
    printf("bcd file cannot be open\n");
    fclose(pdbfile);
    fclose(p3dfile);
    return -1;
  }
  if(write_bcd_file(bcdfile,ligPt) < 0) {
    printf("ERROR while writing .bcd file from ligand data structure\n");
    free_ligand(ligPt);  
    fclose(pdbfile);
    fclose(p3dfile);
    fclose(bcdfile);
    return -1;    
  }
  printf("BCD file written:\n  - the file %s contains data for BioCD initialization\n",bcdfilename);
  
  free_ligand(ligPt);  
  fclose(pdbfile);
  fclose(p3dfile);
  fclose(bcdfile);
  return 1;
}
