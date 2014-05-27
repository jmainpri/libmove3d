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
/***************** bcd_resize.c *******************/
/*******************************************************
							 
Definition of radius of atoms, of ensemble radius for atoms pairs,
and resizing functions.
 */
/* ***************************************************** */

#include "P3d-pkg.h"
#include "Bio-pkg.h"

#include "include/bcd_global.h"

static double PrevVdwRadius(1.0);

double GetPrevVdw(void) {
  return PrevVdwRadius;
}

void SetPrevVdw(double prevVdwRadius) {
  PrevVdwRadius = prevVdwRadius;
}

//number of atom types
#define N_ATOM_TYPES 14 

static double VDW_STANDARD[N_ATOM_TYPES]={1.8, 1.8, //sulphur
1.52,1.52, //oxygen
1.55, 1.55, 1.55, //Nitrogen
1.7, // carbon
1.2, //hydrogen
1.85, //bromine
1.98, // iodine
1.47, //fluorine
1.80, //phosphorus
1.75 // Chlorine
};

static double VDW[N_ATOM_TYPES];

/* DONOR- ACCEPTOR
	S-S= 3.8 (ACCORDING TO CCP4) > SUM OF INDIVIDUAL ATOMS
	O-O= 2.75
	N-N= 3.0
	S-O= 2.9
	O-S=  NO RECORD
	S-N= 3.0
	N-S= NO RECORD
	0-N= 2.85
	N-O= 2.9
	*/

static double SUM_VDW_STANDARD_usual[N_ATOM_TYPES][N_ATOM_TYPES]={
// S   S_H     O    O_H    N    N_H   N_F    C     H     B     I     F     P	
{3.60, 3.60, 3.32, 3.32, 3.35, 3.35, 3.35, 3.50, 3.00, 3.65, 3.78, 3.27, 3.60, 3.55},
//SULPHUR
{3.60, 3.60, 3.32, 3.32, 3.35, 3.35, 3.35, 3.50, 3.00, 3.65, 3.78, 3.27, 3.60, 3.55},
//SULPHUR_H
{3.32, 3.32, 3.04, 3.04, 3.07, 3.07, 3.07, 3.22, 2.72, 3.37, 3.50, 2.99, 3.32, 3.27},
//OXYGEN
{3.32, 3.32, 3.04, 3.04, 3.07, 3.07, 3.07, 3.22, 2.72, 3.37, 3.50, 2.99, 3.32, 3.27},
//NITROGEN
{3.35, 3.35, 3.07, 3.07, 3.10, 3.10, 3.10, 3.25, 2.75, 3.40, 3.53, 3.02, 3.35, 3.30},
//NITROGEN_H
{3.35, 3.35, 3.07, 3.07, 3.10, 3.10, 3.10, 3.25, 2.75, 3.40, 3.53, 3.02, 3.35, 3.30},
//NITROGEN_FULL
{3.35, 3.35, 3.07, 3.07, 3.10, 3.10, 3.10, 3.25, 2.75, 3.40, 3.53, 3.02, 3.35, 3.30},
//CARBON
{3.50, 3.50, 3.22, 3.22, 3.25, 3.25, 3.25, 3.40, 2.90, 3.55, 3.68, 3.17, 3.50, 3.45},
//HYDROGEN
{3.00, 3.00, 2.72, 2.72, 2.75, 2.75, 2.75, 2.90, 2.40, 3.05, 3.18, 2.67, 3.00, 2.95},
//BROMINE
{3.65, 3.65, 3.37, 3.37, 3.40, 3.40, 3.40, 3.55, 3.05, 3.70, 3.83, 3.32, 3.65, 3.60},
//IODINE
{3.78, 3.78, 3.50, 3.50, 3.53, 3.53, 3.53, 3.68, 3.18, 3.83, 3.96, 3.45, 3.78, 3.73},
//FLUORINE
{3.27, 3.27, 2.99, 2.99, 3.02, 3.02, 3.02, 3.17, 2.67, 3.32, 3.45, 2.94, 3.27, 3.22},
//PHOSPHORUS
{3.60, 3.60, 3.32, 3.32, 3.35, 3.35, 3.35, 3.50, 3.00, 3.65, 3.78, 3.27, 3.60, 3.55},
//CHLORINE
{3.55, 3.55, 3.27, 3.27, 3.30, 3.30, 3.30, 3.45, 2.95, 3.60, 3.73, 3.22, 3.55, 3.50}
};



/* sum of radius of van der Waal distances (for common pairs)
and minimal distance according to charmm 
http://www.ebi.ac.uk/msd-srv/chempdb/cgi-bin/cgi.pl?FUNCTION=search&ENTITY=LIB_VDW
and allowing hydrogen bonds only between an atom having a H and another not having H.
H'(atom1) Xor H'(atom2)  
*/
static double SUM_VDW_STANDARD_ebi[N_ATOM_TYPES][N_ATOM_TYPES]={
// S   S_H     O    O_H    N    N_H   N_F    C     H     B     I     F    P

//SULPHUR
{3.60, 3.60, 3.32, 3.32, 3.35, 3.35, 3.35, 3.50, 3.00, 3.65, 3.78, 3.27, 3.60, 3.55},
//SULPHUR_H
{3.60, 3.60, 2.90, 3.32, 3.00, 3.35, 3.35, 3.50, 3.00, 3.65, 3.78, 3.27, 3.60, 3.55},
//OXYGEN
{3.32, 2.90, 3.04, 2.75, 3.07, 2.90, 3.07, 3.22, 2.72, 3.37, 3.50, 2.99, 3.32, 3.27},
//OXYGEN_H
{3.32, 3.32, 2.75, 3.04, 2.85, 3.07, 3.07, 3.22, 2.72, 3.37, 3.50, 2.99, 3.32, 3.27},
//NITROGEN
{3.35, 3.00, 3.07, 2.85, 3.10, 3.00, 3.10, 3.25, 2.75, 3.40, 3.53, 3.02, 3.35, 3.30},
//NITROGEN_H
{3.35, 3.35, 2.90, 3.07, 3.00, 3.10, 3.10, 3.25, 2.75, 3.40, 3.53, 3.02, 3.35, 3.30},
//NITROGEN_FULL
{3.35, 3.35, 3.07, 3.07, 3.10, 3.10, 3.10, 3.25, 2.75, 3.40, 3.53, 3.02, 3.35, 3.30},
//CARBON
{3.50, 3.50, 3.22, 3.22, 3.25, 3.25, 3.25, 3.40, 2.90, 3.55, 3.68, 3.17, 3.50, 3.45},
//HYDROGEN
{3.00, 3.00, 2.72, 2.72, 2.75, 2.75, 2.75, 2.90, 2.40, 3.05, 3.18, 2.67, 3.00, 2.95},
//BROMINE
{3.65, 3.65, 3.37, 3.37, 3.40, 3.40, 3.40, 3.55, 3.05, 3.70, 3.83, 3.32, 3.65, 3.60},
//IODINE
{3.78, 3.78, 3.50, 3.50, 3.53, 3.53, 3.53, 3.68, 3.18, 3.83, 3.96, 3.45, 3.78, 3.73},
//FLUORINE
{3.27, 3.27, 2.99, 2.99, 3.02, 3.02, 3.02, 3.17, 2.67, 3.32, 3.45, 2.94, 3.27, 3.22},
//PHOSPHORUS
{3.60, 3.60, 3.32, 3.32, 3.35, 3.35, 3.35, 3.50, 3.00, 3.65, 3.78, 3.27, 3.60, 3.55},
//CHLORINE
{3.55, 3.55, 3.27, 3.27, 3.30, 3.30, 3.30, 3.45, 2.95, 3.60, 3.73, 3.22, 3.55, 3.50}};


static double SUM_VDW[N_ATOM_TYPES][N_ATOM_TYPES];

static double (*SUM_VDW_STANDARD)[N_ATOM_TYPES]=SUM_VDW_STANDARD_usual;

void set_SUM_VDW_to_EBI(){SUM_VDW_STANDARD= SUM_VDW_STANDARD_ebi;}
void set_SUM_VDW_to_usual(){SUM_VDW_STANDARD= SUM_VDW_STANDARD_usual;}


void set_atom_type(p3d_poly *atom, enum atom_type atype){
atom->type= atype;
atom->radii= SUM_VDW[atype];
#ifdef HYDROGEN_BOND
atom->r= VDW + ((int) atype);
#endif
}

void write_wdw_radii(void){
int i, j;
printf("\n");
for (i=0; i < N_ATOM_TYPES;i++){
	printf("{");
	for (j=0; j < N_ATOM_TYPES ; j++)
		printf("%1.2f, ", VDW[i] + VDW[j]);
	printf("},\n");
	}
}

void write_SUM_VDW(void){
int i, j;
printf("\n");
for (i=0; i < N_ATOM_TYPES;i++){
	printf("{");
	for (j=0; j < N_ATOM_TYPES ; j++)
		printf("%1.2f, ", SUM_VDW[i][j]);
	printf("},\n");
	}
}


static void reduce_don_acep(int i,int j,double scale){
double aux= (SUM_VDW[j][i]) * scale;
SUM_VDW[i][j]= aux;
SUM_VDW[j][i]= aux;
}

void reduce_don_acep_mat(double scale){
reduce_don_acep(1,2,scale);
reduce_don_acep(1,4,scale);
reduce_don_acep(2,3,scale);
reduce_don_acep(2,5,scale);
reduce_don_acep(3,4,scale);
reduce_don_acep(4,5,scale);

reduce_don_acep(1,1,scale);
reduce_don_acep(1,3,scale);
reduce_don_acep(1,5,scale);
reduce_don_acep(3,3,scale);
reduce_don_acep(3,5,scale);
reduce_don_acep(5,5,scale); 
}





#ifdef HYDROGEN_BOND

/*
void resize_one_robot(int nrobot, double  shrink){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure **rigids=robot->rigids;
Rigid_structure *rigido;
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int i,j,k,natoms;

//printf("shrink es %f\n", shrink);
for (i=0;i < robot->nrigids;i++){
    rigido= *(rigids++);
    lbox= rigido->lbox;
    for (j=0;j < rigido->nboxes;j++){
        bbox= *(lbox++);
        lpoly= bbox->lpoly;
        natoms= bbox->natoms;
        for (k=0;k < natoms;k++){
            polygordo= *(lpoly++);
            polygordo->r= VDW[polygordo->type] * shrink;
            }
        }
    }

for (i=0; i < N_ATOM_TYPES;i++)
	for (j=0; j < N_ATOM_TYPES ; j++)
		SUM_VDW[i][j]= SUM_VDW_STANDARD[i][j] * shrink;
robot->to_be_rebuilt=1;   
}
*/



void bio_resize_molecules(double shrink){
int i,j;
/* old version of the function
for (i=0;i < number_of_bcd_robots; i++)
    resize_one_robot(i,shrink);
*/
for (i=0;i < number_of_bcd_robots; i++)
    bcd_robots[i]->to_be_rebuilt=1; 
for (i=0; i < N_ATOM_TYPES;i++)
	VDW[i]= VDW_STANDARD[i] * shrink;
for (i=0; i < N_ATOM_TYPES;i++)
	for (j=0; j < N_ATOM_TYPES ; j++)
		SUM_VDW[i][j]= SUM_VDW_STANDARD[i][j] * shrink;
}


void bio_resize_molecule(int nrobot, double  shrink){
bio_resize_molecules(shrink);
} 

#else




/*scale is the fraction of the VdW radius (specified in the 
p3d_poly describing the atom) that will be considered for 
collision (BCD holds its own version of the VdW radii, which 
can be set arbitrarily)	
nrobot is the number of the robot */

void bio_resize_molecule(int nrobot, double  shrink){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure **rigids=robot->rigids;
Rigid_structure *rigido;
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int i,j,k,natoms;

//printf("shrink es %f\n", shrink);
for (i=0;i < robot->nrigids;i++){
    rigido= *(rigids++);
    lbox= rigido->lbox;
    for (j=0;j < rigido->nboxes;j++){
        bbox= *(lbox++);
        lpoly= bbox->lpoly;
        natoms= bbox->natoms;
        for (k=0;k < natoms;k++){
            polygordo= *(lpoly++);
            polygordo->r= polygordo->primitive_data->radius * shrink;
            }
        }
    }
robot->to_be_rebuilt=1;  
}


void bio_resize_molecules(double shrink){
int i;
for (i=0;i < number_of_bcd_robots; i++)
    bio_resize_molecule(i,shrink);
}


void bio_resize_rigid(int nrobot,int nrigid, double  shrink){
Bboxpt *lbox;
Bboxpt bbox;
Rigid_structure *rigido=bcd_robots[nrobot]->rigids[nrigid];
p3d_poly **lpoly;
p3d_poly *polygordo;
int j,k,natoms;


lbox= rigido->lbox;
for (j=0;j < rigido->nboxes;j++){
    bbox= *(lbox++);
    lpoly= bbox->lpoly;
    natoms= bbox->natoms;
    for (k=0;k < natoms;k++){
        polygordo= *(lpoly++);
         polygordo->r=  polygordo->primitive_data->radius * shrink;
        }
    }
bcd_robots[nrobot]->to_be_rebuilt=1;   
}

#endif


/*
This function resizes the sub-robot  number nrobot to the size 
really used by the BCD. It is intended as a help to visualize the actual 
scene, for instance when using algorithms with adaptive resizing of the 
collision VdW radius. Pay attention to the fact that, after using this 
function, a call to bio_resize_molecule will shrink the robot with respect to 
the new values in the p3d_poly's!. 

*/

void bio_true_resize_molecule(int nrobot){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure **rigids=robot->rigids;
Rigid_structure *rigido;
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int i,j,k,natoms;
//printf("shrink es %f\n", shrink);
for (i=0;i < robot->nrigids;i++){
    rigido= *(rigids++);
    lbox= rigido->lbox;
    for (j=0;j < rigido->nboxes;j++){
        bbox= *(lbox++);
        lpoly= bbox->lpoly;
        natoms= bbox->natoms;
        for (k=0;k < natoms;k++){
            polygordo= *(lpoly++);
#ifdef HYDROGEN_BOND
			polygordo->primitive_data->radius=  *(polygordo->r);
#else
            polygordo->primitive_data->radius=  polygordo->r;
#endif
            }
        }
    }
robot->to_be_rebuilt=1;  
}

void bio_true_resize_molecules(){
int i;
for (i=0;i < number_of_bcd_robots; i++)
	bio_true_resize_molecule(i);
}
