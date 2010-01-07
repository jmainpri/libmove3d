/*******************************************************/
/***************** bcd_init.c *******************/
/*******************************************************/

/*++++++++++++++++++ LONG-TERM STRUCTURE BUILDING ++++++++++++++++++++++ */
/*   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
Functions building long-term structures  depending on the type of molecule,
 the joints of the robot (and its bounds), the sequence of amino-acid
residual types (for polypeptides), etc. These structures do not 
change when robots change of configuration. 
The long-term structures related to short topological distance management are created
via create_residual_SDs(), which is defined in bcd_short_dist.c altogether with the functions
that it calls.
The constants that must be changed to allow more residuals, more free 
side-chains or backbones, more disulphur links, etc are at the begining of
the file.


FORESEEN MISFUNCTIONING WHEN A DISULPHUR LINK IS EXPLICITLY INDICATED BETWEEN CYSTEINES 
IN THE SAME RIGID. 
This is a unuseful link in practise, so you can remove it. Otherwise, it does not require
a huge effort to eliminate this bug.
	*/

#include <stdio.h>
#include <string.h>
#include "P3d-pkg.h"
#include "Bio-pkg.h"

#include "include/bcd_global.h"
//#include "include/bcd_shortdist.h"
//#include "include/bcd_resize.h"


#define MAX_IN_RIGID 7000 /* maximum number of elements in the rigid hierchy (that
is maximum number of backbone and sidechain boxes in a same rigid) */
#define MAX_NUMBER_RIGIDS 4100
#define MAX_AUTOCOL 2000 /* maximum number of side chain-backbone SD structures */
#define MAX_INTERCOL 4000 /* maximum number of backbone-backbone SD structures */
#define MAX_LOOP 20 /* maximum number of disulphur links * 4
 (5  disulphur links if its set to 20) */
#define MAX_NUMBER_AMINOS 5000 /* only for issues regarding SUPRESSION mode. to allow very large proteins
you may have to modify some of the five above define sentences */

#define MAX_NUMBER_p3d_ROBOTS 10 
#define MAX_NUMBER_subROBOTS 50 /* maximum number of suborbots in a p3d_rob */
#define MAX_NUMBER_ROBOTS (MAX_NUMBER_subROBOTS * MAX_NUMBER_p3d_ROBOTS) 


#define MAX_IN_AMINO 28 /* maximum number of atoms in an aminoacid residual */
#define MAX_SD_RIGID 600 /* maximum number of Short (topological) Distance structures
 in which a rigid can be involved */

#define MAX_NAME_LENGTH 43 /* Maximal length name of joints, objects, and polyhedres */
/* 43 should suffice also for THE HIDROGEN VERSION, but it must be reviewed*/


/*   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */
/*+++++++ LONG-TERM VARIABLES: the BCD array of robots and auxiliary variables +++++++++++ */
/*   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */
int number_of_bcd_robots; /* number of robots in the bio collision detector */
/* array of robots in the bio collision detector */
Robot_structure *bcd_robots[MAX_NUMBER_ROBOTS]; 
int num_subrobots[MAX_NUMBER_subROBOTS]; /* Stores the number of subrobots of
each p3d_rob */
int robot_p3d[MAX_NUMBER_p3d_ROBOTS]; /* robot_p3d[N] indicates the first
	subrobot corresponding to p3d_rob number N */

 /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Variables and types for reading values from Move3d  */

//serial numbers of the atoms in the pdb format
//static int natom[MAX_IN_AMINO];
static int new_namino;
static int namino;
static char chain[3];  //chain identifier
static char new_chain[3];
//amino name
static char amino[6];
// complete p3d name of the atom
static char polyname[MAX_NAME_LENGTH + 1];

static Joint_tablespt joint_tables;

static char new_amino[6];

//tables for the  atoms (polyhedres) placed in *move3d* order
static p3d_poly *poly_table[MAX_IN_AMINO];
static int natomamino= 0;
//serial number of the amino
int HYDROGEN_ENVIRONEMENT= 0;

#define two_sentinels_condition (type == LEUH) || (type == ILEH) || (type == VALH) 

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */
/* Variables and types required for constructing the preprogrammed collisions 
with the topological and joint information already obtained form MOVE3D structures */
/*   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */

  
static short int sc_free, bb_free, prev_bb_free,loop;
static short int new_disulphur; //indicates if this is the first cysteine of the disulphure link
/* current (last) rigid containing backbones . Rigids that
contain only a side cahin are not put in this variable */
static Rigid_structure *rigid=NULL;
static Rigid_structure *prev_rigid=NULL; /* previous rigid including backbones*/
static Rigid_structure *sc_rigid=NULL;/* last rigid incluiding a side chain*/
static Robot_structure *rob;
static SDpt *inter_bb, *intra_res, *loops;
static int autocol_index, intercol_index, loop_index;
static Bboxpt last_bb_box, prev_bb_box;
static Bboxpt last_sc_box;
static int disulphur_index;

int *second_cys_namino; /* only useful to allow the second cysteine of a disulphur link
		to search the its SD_struct in order to fill the lacking fields. 
		It will be not necessary with Juan protein structures, because
		everything will be directly accesible */

struct bcd_init_structure init_var; 





/* returns a chain (max. 20 charcters) with the characters following
the *index position of the parameter string until the first instance 
of the second parameter is found.
index is returned with the position after the the first instance of c
*/ 

char *givemeword(char string[], char c, int *index){
  
	static char cadena[MAX_NAME_LENGTH + 1];
	int i=0;
	//printf("cad=%s , c=", string);putchar(c);printf("\n");
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


/* gets the amino name and serial number from the name in s.
s is supposed to have the format

		atom-atom-...-atom.aminoname.aminonumber.chainIdentifier
		
*/		
static void readamino(char *s){
	int indexletter=0;
	givemeword(s,(char)'.', &indexletter);
	strcpy(new_amino,givemeword(s,(char) '.', &indexletter));
	sscanf(givemeword(s,(char) '.', &indexletter),"%d", &new_namino);
	strcpy(new_chain,givemeword(s,(char) '\0', &indexletter));
	if (new_chain[0]=='0') new_chain[0]=' ';
		
	}


/* puts the name and position of the atom in tables pos and names] */

static void intable(p3d_poly *polygordo){
	int indexletter=2;
	char atomname[5];
        
        //polygordo->autocol_data= NULL;
        strcpy(atomname, givemeword(polyname,(char) '.',&indexletter));
				/*
To indicate the formation of a disulphure link between two cysteins inside a polypeptide chain,
 the name of the sulphur atoms in each of the cysteins must have the following form:
        SG.B.linked_residual.natom
where B is mnemonic for "bounded", natom is the number of the atom, as always, and 
linked_residual is the number of residual of the cystein to which this atom is linked.
*/

        if (strcmp(atomname,"SG")== 0){
            if (polyname[indexletter]=='B'){
                indexletter += 2;
                sscanf(givemeword(polyname,(char) '\0', &indexletter),"%hd", &loop);
                }
            }
        
        poly_table[natomamino]= polygordo;
		if (natomamino > MAX_IN_AMINO) {
					printf("ERROR in intable\n");//QUITAR
					printf("%s\n",polygordo->poly->name);//QUITAR
					}
				
	/*
        strcpy(names[natomamino], givemeword(polyname,'.',&indexletter));
	sscanf(givemeword(polyname, '\0', &indexletter),"%d", natom+natomamino);

	pos[natomamino][0]=polygordo->poly->pos[0][3];
	pos[natomamino][1]=polygordo->poly->pos[1][3];
	pos[natomamino][2]=polygordo->poly->pos[2][3]; */

	natomamino++;
	}



/* I am assuming here that if a omega joint is freeded at least one of the 
previous or next phi and/or psi joints are also freeded */
static void process_joint_type(pp3d_jnt joint){
char j_type[20];
int jointype= 1; // 0 psi or phi, 1 omega, 2 anyother (gamma)
int indexletter=0;
// MODIF JUAN (TEMPORARY): I've added p3d_jnt_get_dof_is_user(joint, 0) to solve some 
//                         problems related with the new definition of subchains.
//                         This may cause other problems !!!??? 
int free= (p3d_jnt_get_dof_is_user(joint, 0) && ((joint->vmax - joint->vmin) != 0.0));
if (free){
	strcpy(j_type,givemeword(joint->name,(char) '.', &indexletter));
	if ( (strcmp(j_type,"phi")== 0) || (strcmp(j_type,"psi")==0) ){
				bb_free= 1;
				jointype= 0;
				}
	else if ((strcmp(j_type,"omega")) && (joint->name[0]!= '.')) {
				sc_free= 1;
				jointype= 2;
				}
	}
#ifdef SUPRESSION	
if (namino != new_namino){
	joint_tables= rob->joint_tables[new_namino]=
		(Joint_tablespt) malloc(sizeof(Joint_tables));
	joint_tables->n_sc_joints= 0;
	joint_tables->n_bb_joints= 0;
	joint_tables->namino= new_namino;
	}
	
if (jointype == 2){
	joint_tables->sc_joints[joint_tables->n_sc_joints]= joint;
	(joint_tables->n_sc_joints)++;
	 }
else{
	 joint_tables->bb_joints[joint_tables->n_bb_joints]= joint;
	 (joint_tables->n_bb_joints)++;
	 }
#endif
}


static Robot_structure *init_polypep(void);
static void modulos(void);
static Rigid_structure *init_bb_rigid(void);
void bio_resize_molecule(int nrobot, double  shrink);
void free_robot(Robot_structure *robot);
static void create_no_pep(pp3d_rob molecule, int nrobot, int init_joint, double scale);

/* 
When protein is NULL:
	-It means that is not the first time that robot is build
	-parameter init_joint is not used
	-the p3d_robot and p3d_joint information is got from the old
	robots stored in robots[nrobot]
	-we build only one of the robots of the multirobot 
	
When protein is not NULL:
	- Means it is the first time the robot is built
	- parameter init_joint is taken into account
	- we build all the robots of the multirobot
	 */
static void create_polypep(pp3d_rob protein, int nrobot, int init_joint, 
                                      double scale){
pp3d_jnt joint;
pp3d_jnt *jointpt;
p3d_poly *polygordo;
p3d_poly **polygordopt;

int new_robot= 0; /* indicates if the creation of the current 
		robot must be interrupted to begin a new one */
int robot_has_something= 0; /* indicates if the current robot has 
 	already builded meaningful bodies */
long int all= (long int) protein; /* indicates if all the robots in the first parameter will
	be created */

Rigid_structure **rigids;
int j,i;
last_bb_box= last_sc_box= prev_bb_box= NULL;
rigid= prev_rigid= sc_rigid= NULL;
printf("nace el polypeptido %d\n", nrobot);
prev_bb_free=0; bb_free=0;sc_free=0; //new_disulphur= 0;
namino= -1;
natomamino=0;

all= (long int) protein;
if (protein == NULL){	
	init_joint= bcd_robots[nrobot]->init_joint;
	protein= bcd_robots[nrobot]->p3d_robot;
	}
free_robot(bcd_robots[nrobot]);
number_of_bcd_robots++;
bcd_robots[nrobot]= rob = init_polypep();

rob->p3d_robot= protein;
rob->init_joint= init_joint;
 
second_cys_namino= (int *) malloc(sizeof(int) * MAX_LOOP);

intercol_index= autocol_index= loop_index=0;
rigids= rob->rigids;
inter_bb=rob->inter_bb;intra_res=rob->intra_res;loops=rob->loops;
jointpt= protein->joints + init_joint;
for (j= init_joint;j < protein->njoints + 1 && !new_robot; j++){
//multirobot conditon
	joint=*jointpt; // joint is now a pointer to a joint ((*protein).joints))
	new_robot= ((joint->name[0] == '.') && robot_has_something);
	if (joint->o != NULL && !new_robot && joint->o->name[0]!='.'){
		readamino(joint->o->name);
		if ((namino != new_namino) && (robot_has_something)) modulos();
		robot_has_something= 1;//multirobot purpose 
		process_joint_type(joint);/* this instruction must be executed before
									copying new_namino to namino!!! */
		strcpy(amino,new_amino);
		namino=new_namino;
		strcpy(chain,new_chain); // it is not useful. Even for proteins with several chains, each chain is a different robot
		polygordopt= joint->o->pol;
		for (i=1; i<= joint->o->np;i++){
			polygordo= *polygordopt;
			strcpy(polyname, polygordo->poly->name);
			if (polyname[0]== 'V') intable(polygordo);
			polygordopt++;
			}
		}	
	jointpt++;
	}
modulos();
rigid->lbox= (Bboxpt *)
	realloc((void *) rigid->lbox, (sizeof(Bboxpt) * rigid->nboxes));	
	
rob->inter_bb= (SDpt *)
	realloc((void *) rob->inter_bb, (sizeof(SDpt) * intercol_index));
	
rob->intra_res= (SDpt *)
	realloc((void *) rob->intra_res, (sizeof(SDpt) * autocol_index));
	
rob->loops= (SDpt *)
	realloc((void *) rob->loops, (sizeof(SDpt) * loop_index));

rob->rigids= (Rigid_structure **)
	realloc((void *) rob->rigids, (sizeof(Rigid_structure *) * rob->nrigids));
rob->roots= (boxnodept *)
	realloc((void *) rob->roots, (sizeof(boxnodept) * rob->nrigids));

rob->intercol_index= intercol_index;
rob->autocol_index= autocol_index;
rob->loop_index= loop_index;

free(second_cys_namino);

for (i=0, rigids= rob->rigids; i< rob->nrigids;i++){
	rigid= *(rigids++);
	if (rigid->nSD > 0)
	  rigid->lSD= (SDpt *) realloc((void *) rigid->lSD, (sizeof(SDpt) * rigid->nSD));
	else{
		free(rigid->lSD);
		rigid->lSD=NULL;
		}
	}
#ifdef SUPRESSION
rob->n_aminos= namino + 1;
rob->amino_rigidbb= (Rigid_structure **)
    realloc((void *) rob->amino_rigidbb,rob->n_aminos  * sizeof(Rigid_structure *));
rob->amino_rigidcl= (Rigid_structure **)
    realloc((void *) rob->amino_rigidcl, rob->n_aminos * sizeof(Rigid_structure *));
rob->joint_tables= (Joint_tablespt *)
	realloc((void *) rob->joint_tables, rob->n_aminos * sizeof(Joint_tablespt));    
rob->n_active_rigids= rob->nrigids;
#endif
bio_resize_molecule(nrobot, scale);
if (new_robot && all){
	j= j - 1;
	if (protein->joints[j]->name[1] == 'l') 
		create_no_pep(protein, nrobot + 1, j, scale);
	else create_polypep( protein, nrobot + 1, j, scale);
	}
}

static Robot_structure *init_polypep(void){
Robot_structure *robot;
robot= (Robot_structure *) malloc(sizeof(Robot_structure));

robot->inter_bb= (SDpt *) malloc(MAX_INTERCOL * sizeof(SDpt));

robot->intra_res= (SDpt *) malloc(MAX_AUTOCOL * sizeof(SDpt));

robot->loops= (SDpt *) malloc(MAX_LOOP * sizeof(SDpt));


robot->nrigids= 0;
robot->rigids= (Rigid_structure **)
	malloc(MAX_NUMBER_RIGIDS * sizeof(Rigid_structure *));
// roots must be allocated with a calloc, to initialize all pointers with NULL
#ifdef SUPRESSION
robot->amino_rigidbb= (Rigid_structure **)
	calloc(MAX_NUMBER_AMINOS, sizeof(Rigid_structure *));
robot->amino_rigidcl= (Rigid_structure **)
	calloc(MAX_NUMBER_AMINOS, sizeof(Rigid_structure *));
robot->joint_tables= (Joint_tablespt *)
	calloc(MAX_NUMBER_AMINOS, sizeof(Joint_tablespt));
#endif
robot->roots= (boxnodept *)
	calloc(MAX_NUMBER_RIGIDS, sizeof(boxnodept));
robot->superroot= NULL;
robot->polypep= 1;
robot->to_be_rebuilt=1;
return robot;
}

/********************************************************************
********************************************************************
********************************************************************/



static void set_atom(p3d_poly *atom){
int lookat=2;
char c= atom->poly->name[lookat];

atom->matpos= atom->poly->pos;

switch(c){
case 'N': 
 	set_atom_type(atom,NITROGEN_H);
	break;
case 'C':
	if (atom->poly->name[lookat + 1] != 'l') set_atom_type(atom,CARBON);
 	else set_atom_type(atom,CHLORINE); 
	break;
case 'H': 
 	set_atom_type(atom,HYDROGEN);
	break;
case 'S': 
 	set_atom_type(atom,SULPHUR);
	break;
case 'O': 
 	set_atom_type(atom,OXYGEN);
	break;
case 'B': 
 	set_atom_type(atom,BROMINE);
	break;
case 'I': 
 	set_atom_type(atom,IODINE);
	break;
case 'F': 
 	set_atom_type(atom,FLUORINE);
	break;
case 'P': 
 	set_atom_type(atom,PHOSPHORUS);
	break;
default:
	if ((c >= '1') && (c <= '9')) set_atom_type(atom,HYDROGEN);
	else printf("Unknown type of atom in set_atom\n");
	}
}


/* Fills apropiately the pseudo-backbone box list of atoms for
common residuals without hydrogens */
static void common_bb_filling(Bboxpt bbox){
int i,index=0;
p3d_poly *poly;
/* double (*mat)[4];
double *vec; */

i= natomamino-2;
    poly=poly_table[i]; // C bb
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;

i= natomamino-1;
   poly=poly_table[i]; // O bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
   index++;
i=1;		
   poly=poly_table[i]; // Calpha
   set_atom(poly);
   bbox->lpoly[index]= poly;
   index++;
i=2;		
   poly=poly_table[i]; // Cbeta
   set_atom(poly);
   bbox->lpoly[index]= poly;
   index++;
i=0;
   poly=poly_table[i];  // N bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
   index++;


}


/* Fills apropiately the pseudo-backbone box list of atoms for
common residuals with hydrogens */
static void common_bbH_filling(Bboxpt bbox){
int i,index=0;
p3d_poly *poly;
HYDROGEN_ENVIRONEMENT= 1;
/* double (*mat)[4];
double *vec; */
i= natomamino-2;
    poly=poly_table[i]; // C bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= natomamino-1;
    poly=poly_table[i]; // O bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=1;		
    poly=poly_table[i]; // Calpha
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=2;		
    poly=poly_table[i]; // H bb (linked to Calpha)
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=3;		
    poly=poly_table[i]; // Cbeta
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=0;
    poly=poly_table[i];  // N bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;


}

/* Fills apropiately the pseudo-backbone box list of atoms for
the gly residual without hydrogens */
static void gly_filling(Bboxpt bbox){
p3d_poly *poly;
int i,index=0;
/* double (*mat)[4];
double *vec; */
i= natomamino-2;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= natomamino-1;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=1;		
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;

i=0;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
}


/* Fills apropiately the pseudo-backbone box list of atoms for
the proline residual without hydrogens */
static void pro_filling(Bboxpt bbox){
int i,index=0;
p3d_poly *poly;
/* double (*mat)[4];
double *vec; */
i= natomamino-2;
    poly=poly_table[i];   // C bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= natomamino-1;	// O bb
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=1;		
    poly=poly_table[i];  // Calpha
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=2;		
    poly=poly_table[i];  // Cbeta
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=0;
    poly=poly_table[i];	// N bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=3;		
    poly=poly_table[i]; // C gamma
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=4;		
    poly=poly_table[i];  //C delta
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
		
set_atom_type(bbox->lpoly[4],NITROGEN_FULL); 
}

/* Fills apropiately the pseudo-backbone box list of atoms for
the proline residual with hydrogens */
static void proH_filling(Bboxpt bbox){
int i,index=0;
p3d_poly *poly;
/* double (*mat)[4];
double *vec; */
i= natomamino-2;
    poly=poly_table[i];   // C bb
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;

i= natomamino-1;	// O bb
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=1;		
    poly=poly_table[i];  // Calpha
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=2;		
    poly=poly_table[i]; // H bb (linked to Calpha)
	set_atom(poly);
	bbox->lpoly[index]= poly;
	index++;

i=3;		
    poly=poly_table[i]; // Cbeta   
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;
i=0;
    poly=poly_table[i];	// N bb
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;
i=5;		
    poly=poly_table[i]; // H2 linked to Cbeta
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;	
i=4;		
    poly=poly_table[i]; // H1 linked to Cbeta
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;
i=6;		
    poly=poly_table[i]; // Cgamma
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;		
i=7;		
    poly=poly_table[i]; // H1 linked to Cgamma
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;				
i=8;		
    poly=poly_table[i]; // H2 linked to Cgamma
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;		
i=9;		
    poly=poly_table[i]; // Cdelta
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;
	
i=10;		
    poly=poly_table[i]; // H1 linked to Cdelta
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;	
i=11;		
    poly=poly_table[i]; // H2 linked to Cdelta
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;
set_atom_type(bbox->lpoly[5],NITROGEN_FULL);  
}

static void common_sc_filling(Bboxpt bbox){
int i,index=0;
enum aminoacidos type= bbox->aminotype;
p3d_poly *poly;
/* double (*mat)[4];
double *vec; */
for(i= 3; i < natomamino-2; i++){
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
    }
if ((type == THR) || (type == SER))
	set_atom_type(bbox->lpoly[0],OXYGEN_H); 
if (type == TYR)
	set_atom_type(bbox->lpoly[6],OXYGEN_H);
if ((type == ASP) || (type == GLU))
	set_atom_type(bbox->lpoly[bbox->natoms - 1],OXYGEN_H);
}


static void common_scH_filling(Bboxpt bbox){
int i,index=0;
enum aminoacidos type= bbox->aminotype;
p3d_poly *poly;
/* double (*mat)[4];
double *vec; */
for(i= 4; i < natomamino-2; i++){
	poly=poly_table[i];
	set_atom(poly);
	bbox->lpoly[index]= poly;
    index++;
    }
if (type == THRH) 
	set_atom_type(bbox->lpoly[1],OXYGEN_H); 
if (type == SERH)
	set_atom_type(bbox->lpoly[2],OXYGEN_H);
if (type == TYRH)
	set_atom_type(bbox->lpoly[12],OXYGEN_H);
if ((type == ASPH) || (type == GLUH))
	set_atom_type(bbox->lpoly[bbox->natoms - 1],OXYGEN_H);
}

/*
static void sc_ileH_filling(Bboxpt bbox){
p3d_poly *poly;
int i,index=0;


i= 5;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 4;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=9;		
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;

i=6;		
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=7;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 8;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;

i=10;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 11;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 12;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;

i=13;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=14;		
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=15;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
}

static void sc_leuH_filling(Bboxpt bbox){
p3d_poly *poly;
int i,index=0;

i= 4;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 5;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 6;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=7;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
	
i= 8;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 12;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=9;		
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=10;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
	
i= 11;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;

i=13;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=14;		
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=15;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
}

static void sc_valH_filling(Bboxpt bbox){
p3d_poly *poly;
int i,index=0;

i= 4;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 5;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 9;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 6;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=7;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
	
i= 8;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i=10;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
	
i= 11;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
i= 12;
    poly=poly_table[i];
   set_atom(poly);
   bbox->lpoly[index]= poly;
    index++;
}
*/


static Rigid_structure *init_bb_rigid(void){
Rigid_structure *r;

if (rigid){ 
	rigid->lbox= (Bboxpt *)
	  realloc((void *) rigid->lbox, (sizeof(Bboxpt) * rigid->nboxes));
	prev_rigid= rigid;
	}
rigid= r= rob->rigids[rob->nrigids] = (Rigid_structure *) malloc(sizeof(Rigid_structure));
r->lbox= (Bboxpt *) malloc(MAX_IN_RIGID * sizeof(Bboxpt));
r->lSD= (SDpt *) malloc(MAX_SD_RIGID * sizeof(SDpt));
r->nSD= 0;
// ? r->lbox= calloc(MAX_IN_RIGID, sizeof(Bboxpt));
r->reference= poly_table[natomamino - 1];
r->reference2= NULL; //only with HYDROGEN_COMPILATION
r->nboxes= 0;
#ifdef SUPRESSION
r->active= 1;
//r->index= -1;
r->root= NULL;
#endif
(rob->nrigids)++;
return r;
//if (rigid) printf("\t despues del realloc %d \n", (*(rigid->lbox))->namino);
}

static Rigid_structure *init_sc_rigid(void){
Rigid_structure *r;
sc_rigid= r= rob->rigids[rob->nrigids]=(Rigid_structure *) malloc(sizeof(Rigid_structure));
r->lbox= (Bboxpt *) malloc(sizeof(Bboxpt));
r->lSD= (SDpt *) malloc(10 * sizeof(SDpt)); /* A side chain rigid can be involved
in a very limited number of SD */
r->nSD= 0;
r->reference= poly_table[natomamino - 3];
r->reference2= NULL; //only with HYDROGEN_COMPILATION
#ifdef SUPRESSION
r->active= 1;
//r->index= -1;
r->root= NULL;
#endif
(rob->nrigids)++;
return r;
}


static Bboxpt bboxallocate(void){
return (Bbox *) malloc(sizeof(Bbox));
}


static Bboxpt create_bb(int n, enum aminoacidos residual){
Bboxpt bbox;
if (!(last_bb_box)){ //this is the first backbone
	init_bb_rigid();
	bbox= rigid->lbox[rigid->nboxes]=bboxallocate();
	}
else if (prev_bb_free || bb_free){//put in a new rigid
	init_bb_rigid();
	bbox= rigid->lbox[rigid->nboxes]=bboxallocate();
/*
#ifdef SUPRESSION
	rigid->index= intercol_index;
#endif
 */
	}
else {// We allocate the box in the current rigid
	bbox= rigid->lbox[rigid->nboxes]=bboxallocate();
	}
(rigid->nboxes)++;

bbox->lpoly= (p3d_poly **) malloc(n * sizeof(p3d_poly *));

bbox->aminotype= residual;
bbox->bb= 1;
bbox->namino= namino;
bbox->natoms= n;
#ifdef SUPRESSION
rob->amino_rigidbb[namino]= rigid;
//bbox->active= 1;
#endif
bbox->loop= loop;
if (loop){ /* the current residual has a disulphur link. It can be with a higher index
		residual (new_disulphur= TRUE) or with an already processed residual (new_disulphur= FALSE) */
	if (loop > namino){ //new disuplphur link found
		new_disulphur= 1;
		second_cys_namino[loop_index / 4]= loop;
		}
	else{ /* The other cysteine has been already treated.
		We have to rerieve it in loop->bb1,
					to get te correct index */
		int i= 0;
		new_disulphur= 0;
		while (( second_cys_namino[i] != namino) && (i < loop_index)){
			i++;
			}
		if (i == loop_index){ /* we store the index for the side chain */
			 printf("ATTENTION! ERROR IN THE SPECIFICATION OF DISULPHUR LOOPS\n");
			 disulphur_index= -1;
			 }
		else {
			disulphur_index= i * 4;  //we store the index for the side chain 
			}		
		}
	}
prev_bb_box= last_bb_box;
last_bb_box= bbox;
return bbox;
}

static Bboxpt create_sc(int n, enum aminoacidos residual){
Bboxpt bbox;
Rigid_structure *r;
//if ((sc_free) && (residual != PRO) && (residual != GLY) && (residual != ALA)){
            /* There is not cl box creation for these residuals */
if (sc_free){
	r= init_sc_rigid();
	bbox= r->lbox[0]=bboxallocate();
#ifdef SUPRESSION
	rob->amino_rigidcl[namino]= r;
//	bbox->active= 1;
//	r->index= autocol_index;
#endif	
	
	r->nboxes= 1;
	}
else {
	bbox= rigid->lbox[rigid->nboxes]=bboxallocate();
	sc_rigid= rigid;
#ifdef SUPRESSION
	rob->amino_rigidcl[namino]= rigid;
//	bbox->active= 1;
#endif
	(rigid->nboxes)++;
	}
bbox->lpoly= (p3d_poly **) malloc(n * sizeof(p3d_poly *));

bbox->aminotype= residual;
bbox->bb= 0;
bbox->namino= namino;
bbox->natoms= n;
bbox->loop= loop;

last_sc_box= bbox;
return bbox;
}








/*reducir toda esta funcion para que el numero de atomos argumento
de createbb y create_sc venga de un array, y que se utilice
common_bb_filling o common_bbH_filling en funcion de si el numero
de aminoacido > 19 */


static void modulos(void){
enum aminoacidos type;
if (strcmp(amino,"ALA")== 0) {
	if (natomamino == 5)
		common_bb_filling(create_bb(5,ALA));
	else {
		if (natomamino != 9) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,ALAH));
		common_scH_filling(create_sc(3,ALAH));
		}
	}
else if (strcmp(amino,"ARG")== 0){
	if (natomamino == 11){
		common_bb_filling(create_bb(5,ARG));
		common_sc_filling(create_sc(6,ARG));
		}
	else {
		if (natomamino != 18) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,ARGH));
		common_scH_filling(create_sc(12,ARGH));
		}
	}
else if (strcmp(amino,"ASN")== 0){
	if (natomamino == 8){
		common_bb_filling(create_bb(5,ASN));
		common_sc_filling(create_sc(3,ASN));
		}
	else {
		if (natomamino != 11) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,ASNH));
		common_scH_filling(create_sc(5,ASNH));
		}
	}		
else if (strcmp(amino,"ASP")== 0){
	if (natomamino == 8){
		common_bb_filling(create_bb(5,ASP));
		common_sc_filling(create_sc(3,ASP));
		}
	else {
		if (natomamino != 11) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,ASPH));
		common_scH_filling(create_sc(5,ASPH));
		}
	}	
else if (strcmp(amino,"CYS")== 0){
	if (natomamino == 6){
		common_bb_filling(create_bb(5,CYS));
		common_sc_filling(create_sc(1,CYS));
    }
	else{
		if (natomamino != 9) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,CYSH));
		common_scH_filling(create_sc(3,CYSH));
		}
	}
else if (strcmp(amino,"GLN")== 0){
	if (natomamino == 9){
		common_bb_filling(create_bb(5,GLN));
		common_sc_filling(create_sc(4,GLN));
		}
	else {
		if (natomamino != 14) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,GLNH));
		common_scH_filling(create_sc(8,GLNH));
		}
	}
else if (strcmp(amino,"GLU")== 0){
	if (natomamino == 9){
		common_bb_filling(create_bb(5,GLU));
		common_sc_filling(create_sc(4,GLU));
		}
	else{
		if (natomamino != 14) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,GLUH));
		common_scH_filling(create_sc(8,GLUH));
		}
	}
else if (strcmp(amino,"GLY")== 0){
	if (natomamino == 4){
		gly_filling(create_bb(4,GLY));
		}
	else{
		if (natomamino != 6) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,GLYH));
		}
	}
else if (strcmp(amino,"HIS")== 0){
	if (natomamino == 10){
		common_bb_filling(create_bb(5,HIS));
		common_sc_filling(create_sc(5,HIS));
		}
	else{
		if (natomamino != 15) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,HISH));
		common_scH_filling(create_sc(9,HISH));
		}
	}
else if (strcmp(amino,"ILE")== 0){
	if (natomamino == 8){
		common_bb_filling(create_bb(5,ILE));
		common_sc_filling(create_sc(3,ILE));
		}
	else{
		if (natomamino != 18) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,ILEH));
		common_scH_filling(create_sc(12,ILEH));
		//sc_ileH_filling(create_sc(12,ILEH));
		}
	}
else if (strcmp(amino,"LEU")== 0){
	if (natomamino == 8){
		common_bb_filling(create_bb(5,LEU));
		common_sc_filling(create_sc(3,LEU));
		}
	else{
		if (natomamino != 18) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,LEUH));
		common_scH_filling(create_sc(12,LEUH));
		//sc_leuH_filling(create_sc(12,LEUH));
		}
	}
else if (strcmp(amino,"LYS")== 0){
		if (natomamino == 9){
		common_bb_filling(create_bb(5,LYS));
		common_sc_filling(create_sc(4,LYS));
		}
	else{
		if (natomamino != 18) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,LYSH));
		common_scH_filling(create_sc(12,LYSH));
		}
	}
else if (strcmp(amino,"MET")== 0){
	if (natomamino == 8){
		common_bb_filling(create_bb(5,MET));
		common_sc_filling(create_sc(3,MET));
		}
	else{
		if (natomamino != 16) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,METH));
		common_scH_filling(create_sc(10,METH));
		}
	}
else if (strcmp(amino,"PHE")== 0){
	if (natomamino == 11){
		common_bb_filling(create_bb(5,PHE));
		common_sc_filling(create_sc(6,PHE));
		} 
	else{
		if (natomamino != 19) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,PHEH));
		common_scH_filling(create_sc(13,PHEH)); //--->
		} 
	}
else if (strcmp(amino,"PRO")== 0){
	if (natomamino == 7) pro_filling(create_bb(7,PRO));
	else{
		if (natomamino != 14) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		proH_filling(create_bb(14,PROH));
		}
	}
else if (strcmp(amino,"SER")== 0){
	if (natomamino == 6){
		common_bb_filling(create_bb(5,SER));
		common_sc_filling(create_sc(1,SER));
		}
	else {
		if (natomamino != 9) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,SERH));
		common_scH_filling(create_sc(3,SERH));
		}
	}
else if (strcmp(amino,"THR")== 0){
	if (natomamino == 7){
		common_bb_filling(create_bb(5,THR));
		common_sc_filling(create_sc(2,THR));
		} 
	else{
		if (natomamino != 12) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,THRH));
		common_scH_filling(create_sc(6,THRH));
		} 
	}
else if (strcmp(amino,"TRP")== 0){
	if (natomamino == 14){
		common_bb_filling(create_bb(5,TRP));
		common_sc_filling(create_sc(9,TRP));
		} 
	else{
		if (natomamino != 22) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,TRPH));
		common_scH_filling(create_sc(16,TRPH)); //---->
		}
	}	
else if (strcmp(amino,"TYR")== 0){
	if (natomamino == 12){
		common_bb_filling(create_bb(5,TYR));
		common_sc_filling(create_sc(7,TYR));
		} 
	else{
		if (natomamino != 19) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,TYRH));
		common_scH_filling(create_sc(13,TYRH));
		}
	}
else if (strcmp(amino,"VAL")== 0){
	if (natomamino == 7){
		common_bb_filling(create_bb(5,VAL));
		common_sc_filling(create_sc(2,VAL));
		} 
	else{
		if (natomamino != 15) printf("BIG ERROR: the number of atoms of amino number %d of type %s \n", namino,amino);
		common_bbH_filling(create_bb(6,VALH));
		common_scH_filling(create_sc(9,VALH));
		//sc_valH_filling(create_sc(9,VALH));
		}
	}
else {
		printf("La cagaste Burt Lancaster, aminoacido desconocido\n");
		printf("\nERROR: Unknow residual type in modulos() : type = %s\n",amino);
		}
create_residual_SDs(&autocol_index, &intercol_index, &loop_index);
if (sc_free){
	type = last_sc_box->aminotype;
	if (two_sentinels_condition)
	 	sc_rigid->reference2= 
			last_sc_box->lpoly[last_sc_box->natoms - 4];
		}
natomamino=0;
prev_bb_free=bb_free; bb_free=0;sc_free=0;loop=0;
}



/* OLD VERSION FOR ENVIRONEMENTS WITHOUT HYDORGENS */
/*
static void modulos(void){

if (strcmp(amino,"ALA")== 0) {
    common_bb_filling(create_bb(5,ALA));
    }
else if (strcmp(amino,"ARG")== 0){
    common_bb_filling(create_bb(5,ARG));
    common_sc_filling(create_sc(6,ARG));
    }
else if (strcmp(amino,"ASN")== 0){
    common_bb_filling(create_bb(5,ASN));
    common_sc_filling(create_sc(3,ASN));
    }
else if (strcmp(amino,"ASP")== 0){
    common_bb_filling(create_bb(5,ASP));
    common_sc_filling(create_sc(3,ASP));
    }
else if (strcmp(amino,"CYS")== 0){
    common_bb_filling(create_bb(5,CYS));
    common_sc_filling(create_sc(1,CYS));
    }

else if (strcmp(amino,"GLN")== 0){
    common_bb_filling(create_bb(5,GLN));
    common_sc_filling(create_sc(4,GLN));
    }

else if (strcmp(amino,"GLU")== 0){
    common_bb_filling(create_bb(5,GLU));
    common_sc_filling(create_sc(4,GLU));
    }
else if (strcmp(amino,"GLY")== 0){
    gly_filling(create_bb(4,GLY));
    }
else if (strcmp(amino,"HIS")== 0){
    common_bb_filling(create_bb(5,HIS));
    common_sc_filling(create_sc(5,HIS));
    }
else if (strcmp(amino,"ILE")== 0){
    common_bb_filling(create_bb(5,ILE));
    common_sc_filling(create_sc(3,ILE));
    }
else if (strcmp(amino,"LEU")== 0){
    common_bb_filling(create_bb(5,LEU));
    common_sc_filling(create_sc(3,LEU));
    }
else if (strcmp(amino,"LYS")== 0){
    common_bb_filling(create_bb(5,LYS));
    common_sc_filling(create_sc(4,LYS));
    }
else if (strcmp(amino,"MET")== 0){
    common_bb_filling(create_bb(5,MET));
    common_sc_filling(create_sc(3,MET));
    }
else if (strcmp(amino,"PHE")== 0){
    common_bb_filling(create_bb(5,PHE));
    common_sc_filling(create_sc(6,PHE));
    } 
else if (strcmp(amino,"PRO")== 0){
     pro_filling(create_bb(7,PRO));
    }
else if (strcmp(amino,"SER")== 0){
    common_bb_filling(create_bb(5,SER));
    common_sc_filling(create_sc(1,SER));
    } 
else if (strcmp(amino,"THR")== 0){
    common_bb_filling(create_bb(5,THR));
    common_sc_filling(create_sc(2,THR));
    } 
else if (strcmp(amino,"TRP")== 0){
    common_bb_filling(create_bb(5,TRP));
    common_sc_filling(create_sc(9,TRP));
    } 
else if (strcmp(amino,"TYR")== 0){
    common_bb_filling(create_bb(5,TYR));
    common_sc_filling(create_sc(7,TYR));
    } 
else if (strcmp(amino,"VAL")== 0){
    common_bb_filling(create_bb(5,VAL));
    common_sc_filling(create_sc(2,VAL));
    } 
else {
		printf("ERROR in modulos: unknown residual !\n");
		}
natomamino=0;
prev_bb_free=bb_free; bb_free=0;sc_free=0;loop=0;
}
*/

/**********************************************************************************
Reading non-polypeptid robots
**********************************************************************************/
/**********************************************************************************/
int fake_namino= -1; /* fake amino number to keep the allow the same collision
 functions for peptides and non polpetydes. Remember that the amino number of 
the two boxes are checked in usual_collision */


static void prepare_nonPep_joint(pp3d_jnt joint){
Bboxpt bbox;
rigid= rob->rigids[rob->nrigids]=(Rigid_structure *) malloc(sizeof(Rigid_structure));
(rob->nrigids)++;
rigid->lbox= (Bboxpt *) malloc(sizeof(Bboxpt));
rigid->nboxes= 1;
rigid->root= NULL;
rigid->reference2= NULL; //only with HYDROGEN_COMPILATION
rigid->nSD= 0;rigid->lSD= NULL;
#ifdef SUPRESSION
rigid->active= 1;
#endif
rigid->lbox= (Bboxpt *) malloc(sizeof(Bboxpt));
bbox= rigid->lbox[0]=bboxallocate();
bbox->lpoly= (p3d_poly **) malloc((joint->o->np) * sizeof(p3d_poly *));
bbox->aminotype= (enum aminoacidos) -1;
bbox->loop= 0;
bbox->namino= (fake_namino -= 2); /* quick and dirty  
trick to mantain different and not 
interfering amino numbers for non polypetides */
}


static Robot_structure *init_no_pep(void);

static void create_no_pep(pp3d_rob molecule, int nrobot, int init_joint, double scale){
pp3d_jnt joint;
pp3d_jnt *jointpt;
p3d_poly *polygordo;
p3d_poly **polygordopt,**lpoly;
Bboxpt bbox;

Rigid_structure **rigids;
int j,i,atoms_in_joint;
int all; /* holds the original value of the molecule parameter */

rigid= NULL;
printf("nace el no polypetido %d\n", nrobot);

all = (long int) molecule;
if (molecule == NULL){	
	init_joint= bcd_robots[nrobot]->init_joint;
	molecule= bcd_robots[nrobot]->p3d_robot;
	}

free_robot(bcd_robots[nrobot]);
number_of_bcd_robots++;
bcd_robots[nrobot]= rob = init_no_pep(); 

rob->p3d_robot= molecule;
rob->init_joint= init_joint;

rigids= rob->rigids;

jointpt=molecule->joints + init_joint;
for (j= init_joint;j < molecule->njoints + 1; j++){
	joint=*jointpt; // joint is now a pointer to a joint ((*protein).joints))
	if ( (joint->o) && (joint->o->name[0]!='.')){
		polygordopt= joint->o->pol;
		prepare_nonPep_joint(joint);
		atoms_in_joint= 0;
		bbox= rigid->lbox[0];
		for (i=1; i<= joint->o->np;i++){
			polygordo= *polygordopt;
			strcpy(polyname, polygordo->poly->name);
			if (polyname[0]== 'V'){ 
				bbox->lpoly[atoms_in_joint++]= polygordo;
				polygordo->autocol_data= NULL;
				}
			polygordopt++;
			}
		bbox->natoms= atoms_in_joint;
		bbox->lpoly= (p3d_poly **) 
			realloc((void *) bbox->lpoly, atoms_in_joint  * sizeof(p3d_poly *));
		rigid->reference= bbox->lpoly[atoms_in_joint -1];
		for (i=0, lpoly= bbox->lpoly ; i< atoms_in_joint;i++) set_atom(*(lpoly++));
		}	
	jointpt++;
	}
rob->rigids= (Rigid_structure **)
	realloc((void *) rob->rigids, (sizeof(Rigid_structure *) * rob->nrigids));
rob->roots= (boxnodept *)
	realloc((void *) rob->roots, (sizeof(boxnodept) * rob->nrigids));
bio_resize_molecule(nrobot, scale);
}


void my_create_molecule(pp3d_rob rob, int nrobot, int njoint, 
                            double scale){
char robotname[250];
char tipo[250];
int is_a_peptide;
int indexletter=0;
if (rob){ 
	printf("%s \n",rob->name);
	strcpy(robotname, givemeword(rob->name,(char)'.', &indexletter));
	strcpy(tipo, givemeword(rob->name,(char) '\0',&indexletter));
	if (strcmp(tipo,"pep") == 0) is_a_peptide= 1;
		else is_a_peptide= 0;
	}
else is_a_peptide= bcd_robots[nrobot]->polypep;

if (is_a_peptide) create_polypep(rob, nrobot, njoint, scale);
  else create_no_pep(rob, nrobot, njoint, scale);

  
}


static Robot_structure *init_no_pep(void){
Robot_structure *robot;
robot= (Robot_structure *) malloc(sizeof(Robot_structure));
robot->intra_res= NULL;
robot->inter_bb= NULL;
robot->loops= NULL;

robot->nrigids= 0;
robot->loop_index=0;
robot->intercol_index= 0;
robot->autocol_index= 0;
robot->rigids= (Rigid_structure **)
	malloc(MAX_NUMBER_RIGIDS * sizeof(Rigid_structure *));
// roots must be allocated with a calloc, to initialize all pointers with NULL
robot->roots= (boxnodept *)
	calloc(MAX_NUMBER_RIGIDS, sizeof(boxnodept));
robot->superroot= NULL;
robot->polypep= 0;
robot->to_be_rebuilt=1;
return robot;
}

struct bcd_init_structure *pass_init_variables(void){
init_var.sc_free= sc_free;
init_var.bb_free= bb_free;
init_var.prev_bb_free= prev_bb_free;
init_var.loop= loop;
init_var.new_disulphur= new_disulphur;
init_var.rigid= rigid;
init_var.prev_rigid= prev_rigid;
init_var.sc_rigid= sc_rigid;
init_var.inter_bb= inter_bb;
init_var.intra_res= intra_res;
init_var.loops= loops;
init_var.autocol_index= autocol_index;
init_var.intercol_index= intercol_index;
init_var.loop_index= loop_index;
init_var.last_bb_box= last_bb_box;
init_var.prev_bb_box= prev_bb_box;
init_var.last_sc_box= last_sc_box;
return &init_var;
 }
