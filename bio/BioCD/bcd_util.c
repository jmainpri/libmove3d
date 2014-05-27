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
/********************* bcd_util.c **********************/
/*******************************************************/

/* Most of the higher level and user level functions.
You find also here the functions to rebuild all the robot
 hierarchies.
  */
#include "P3d-pkg.h"
#include "Bio-pkg.h"
#include "Util-pkg.h"

#include "include/bcd_global.h"
//#include "include/bcd_shortdist.h"
//#include "include/bcd_init.h"
//#include "include/bcd_hierarchies.h"
//#include "include/bcd_resize.h"


/*******************************************************/


Robot_structure **get_bcd_robots(void)
{
  return bcd_robots;
} 



/*******************************************************/


/*
Establishes the pairs of atoms inside a non-polypetide 
that must be excluded from consideration in any of the 
modes of BCD.
nrobot is the number of the no polypeptide, nforbidden 
is the number of pairs that will be excluded (and the 
length of the array arguments). The last arguments 
specify the forbidden pairs: the atom number natom1[i] 
of rigid number nrigid1[i] is excluded from interaction 
with the atom number natom2[i] of rigid number nrigid2[i].
If the function is called more than once to the same
 robot, it will discard previous exclusion pairs, and 
 set the new ones. There is no need to call bio_create_molecule
  or bio_create_molecules after using this function
*/
		
void bio_set_autocol(int nrobot, int nforbidden, int  *nrigid1, int *natom1, int  *nrigid2, int *natom2){
Robot_structure *robot=bcd_robots[nrobot];
Rigid_structure **rigids=robot->rigids,**rrigids=robot->rigids;
Rigid_structure *rigido;
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int i,j,k,natoms;
autocol_struct *autocol;
int size;
int index= 0; /* goes thruogh the four argument vectors*/
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
            if ((autocol= polygordo->autocol_data)){
                    free(autocol->list);
                    free(autocol); /* polygordo->autocol_data is eventually set to NULL at the end */
                    }	                
            if ((nrigid1[index]== i) && (natom1[index]==k)){
                    autocol=polygordo->autocol_data= (autocol_struct *) 
                            malloc(sizeof(autocol_struct));
                    autocol->nrigid= i;
                    size=0;
                    autocol->list=(p3d_poly **) malloc(20 * sizeof(p3d_poly *));
                    while ((index < nforbidden) && (nrigid1[index]== i) 
                            && (natom1[index]==k)){
                            autocol->list[size]=
							rrigids[nrigid2[index]]->lbox[0]->lpoly[natom2[index]];
                            size++;
                            index++;
                            }
                    autocol->list= (p3d_poly **) realloc(
                                (void *) autocol->list, size * sizeof(p3d_poly *));
                    autocol->size= size;
                    }
            
            else polygordo->autocol_data=NULL;           
		
            }
        }
    }\
if (index < nforbidden) printf("\nERROR IN bio_set_autocol: something wrong in the arguments of the %d robot",nrobot);
}

#ifdef SUPRESSION

static void rebuild_completely(Robot_structure *robot){
int i,j=  0,nrigids= robot->nrigids;
Rigid_structure *rigid, **rigids= robot->rigids;
for (i=0;i < nrigids; i++){
    rigid= *(rigids++);
    if (rigid->active){
		if (charlatan) printf("reconstruyo rigido %d", i);
		freeboxhierarchy2(rigid);
		robot->roots[j++]=create_rigid_root(rigid);
		//hierarchy(robot->roots[i]);
		}
	}
if (charlatan)	printf("\n");
if (j != robot->n_active_rigids) robot->n_active_rigids= j;
freesuphierarchy(robot);
bio_create_molecule_root(robot);
robot->to_be_rebuilt=0;
}


static void rebuild_checking(Robot_structure *robot){
int i,j= 0,nrigids= robot->nrigids;
// boxnodept root;
double (*mat)[4];
double *vec;
int moved=0;
Rigid_structure  *rigid,**rigids= robot->rigids;
for (i=0;i < nrigids; i++){
    rigid= *(rigids++);
    if (rigid->active) {
		mat= rigid->reference->matpos;
		vec=rigid->reference->pos;
		if (((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3])){
			if (charlatan)  printf("reconstruyo rigido %d", i);
			freeboxhierarchy2(rigid);
			robot->roots[j++]=create_rigid_root(rigid);
			moved= 1;
			//hierarchy(robot->roots[i]);
			}
		else robot->roots[j++]= rigid->root;
		}
    }
if  (charlatan) 	printf("\n");
if (moved || (j != robot->n_active_rigids)){
    robot->n_active_rigids= j;
    freesuphierarchy(robot);
    bio_create_molecule_root(robot);
    }
}


static void rebuild_checkingH(Robot_structure *robot){
int i,j= 0,nrigids= robot->nrigids;
// boxnodept root;
double (*mat)[4];
double *vec;
int moved=0;
Rigid_structure  *rigid,**rigids= robot->rigids;
for (i=0;i < nrigids; i++){
	rigid= *(rigids++);
	if (rigid->active){
		mat= rigid->reference->matpos;
		vec=rigid->reference->pos;
	//#ifdef HYDROGEN_COMPILATION
		if (
		  ((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3])
			||
			(
			  rigid-> reference2 && (vec=rigid->reference2->pos) && (mat= rigid->reference2->matpos)
				&&
				(((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3]))
			)
			){
	/*
	#else
			if (((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3])){
	#endif
	*/
				if (charlatan)  printf("reconstruyo rigido %d", i);
				freeboxhierarchy2(rigid);
				robot->roots[j++]=create_rigid_root(rigid);
				moved= 1;
				//hierarchy(robot->roots[i]);
				}
			else robot->roots[j++]= rigid->root;
			}
		}
if  (charlatan) 	printf("\n");
if (moved || (j != robot->n_active_rigids)){
    robot->n_active_rigids= j;
    freesuphierarchy(robot);
    bio_create_molecule_root(robot);
    }
}


#else

static void rebuild_completely(Robot_structure *robot){
int i,nrigids= robot->nrigids;
Rigid_structure  *rigid,**rigids= robot->rigids;
for (i=0;i < nrigids; i++){
	rigid= *(rigids++);
	if (charlatan) printf("reconstruyo rigido %d", i);
	freeboxhierarchy(robot,i);
	robot->roots[i]=create_rigid_root(rigid);
//hierarchy(robot->roots[i]);
	}
if (charlatan)	printf("\n");
freesuphierarchy(robot);
bio_create_molecule_root(robot);
robot->to_be_rebuilt=0;
}


static void rebuild_checking(Robot_structure *robot){
int i,nrigids= robot->nrigids;
int aux= charlatan;
charlatan= 1;
// boxnodept root;
double (*mat)[4];
double *vec;
int moved=0;
Rigid_structure  *rigid,**rigids= robot->rigids;
for (i=0;i < nrigids; i++){
	rigid= *(rigids++);
	mat= rigid->reference->matpos;
	vec=rigid->reference->pos;
	if (((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3])){
		if (charlatan)  printf("reconstruyo rigido %d", i);
		freeboxhierarchy(robot,i);
		robot->roots[i]=create_rigid_root(rigid);
                moved= 1;
		//hierarchy(robot->roots[i]);
		}
	}
if  (charlatan) 	printf("\n");
if (moved){
    freesuphierarchy(robot);
    bio_create_molecule_root(robot);
    }
charlatan=aux;
}

static void rebuild_checkingH(Robot_structure *robot){
int i,nrigids= robot->nrigids;
// boxnodept root;
double (*mat)[4];
double *vec;
int moved=0;
Rigid_structure  *rigid,**rigids= robot->rigids;
for (i=0;i < nrigids; i++){
	rigid= *(rigids++);
	mat= rigid->reference->matpos;
	vec=rigid->reference->pos;
//#ifdef HYDROGEN_COMPILATION
	if (
		((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3])
		||
		(
		  rigid-> reference2 && (vec=rigid->reference2->pos) && (mat= rigid->reference2->matpos)
			&&
			(((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3]))
		)
		){
/*
#else
	if (((*vec) != mat[0][3]) || ((*(++vec)) != mat[1][3])  || ((*(++vec)) != mat[2][3])){
#endif
*/
		if (charlatan)  printf("reconstruyo rigido %d", i);
		freeboxhierarchy(robot,i);
		robot->roots[i]=create_rigid_root(rigid);
		moved= 1;
		//hierarchy(robot->roots[i]);
		}
	}
if  (charlatan) 	printf("\n");
if (moved){
    freesuphierarchy(robot);
    bio_create_molecule_root(robot);
    }
}

#endif

void rebuild(Robot_structure *robot){
if (robot->to_be_rebuilt) rebuild_completely(robot);
else {
	 if (HYDROGEN_ENVIRONEMENT) rebuild_checkingH(robot);
	 else rebuild_checking(robot);
	 }
}

/* OLD VERSION FOR ENVIRONEMENTS WITHOUT HYDORGENS */
/*
static void rebuild(Robot_structure *robot){
if (robot->to_be_rebuilt) rebuild_completely(robot);
else rebuild_checking(robot);
}
*/



/* can be made static */
void preprogr_collisions(Robot_structure *robot){
SDpt SD=NULL, *lSD= robot->inter_bb;
int index= robot->intercol_index;
int i;


for(i=0;i < index; i++){
SD= *(lSD++);
#ifdef SUPRESSION
	if (SD->active) 
#endif
		{
		general_SD_function(SD);
		if (too_much_collisions()) return;
		}
	}
lSD= robot->intra_res;
index= robot->autocol_index;
for(i=0;i < index; i++){
SD= *(lSD++);
#ifdef SUPRESSION
	if (SD->active) 
#endif
		{
		(*(SD->function)) (SD);
		if (too_much_collisions()) return;
		} 
	}
lSD= robot->loops;
index= robot->loop_index;
for(i=0;i < index; i++,SD=  *(lSD++)){
#ifdef SUPRESSION
	if (SD->active) 
#endif
		{
		general_SD_function(SD);
		if (too_much_collisions()) return;
		}
	}

}

/*
static void original_autocollision(void){
int i,j,nrigids= rob->nrigids;
boxnodept *roots= rob->roots;
for (i=0;i < nrigids; i++)
	for (j=i + 1;j < nrigids; j++)
		collision(roots[i],roots[j]);
}



static void original_autocollision(int nrobot){
int nrigids= bcd_robots[nrobot]->nrigids;
boxnodept *rootsi, *rootsj;
int i,j;
rootsi=  bcd_robots[nrobot]->roots;
for (i=0;i < nrigids; i++, rootsi++){
	rootsj= rootsi + 1;
	for (j= i + 1;j < nrigids; j++, rootsj++)
		collision(*rootsi,*rootsj);
	}
}

*/

			
		
			

/* the free autocol could be made only for non-polypetides */

static void free_bbox(Bboxpt box, int polypep){
int i;
autocol_struct *autocol;
//printf("empiezo el famoso for\n");
//printf("%d %hd \t", box->namino,box->bb);
if (!(polypep))
    for(i= 0; i < box->natoms; i++) 
            if ((autocol= box->lpoly[i]->autocol_data)){
                    free(autocol->list);
                    free(autocol);
                     box->lpoly[i]->autocol_data=NULL;
                    }	
//printf("acabo el famoso for\n");
free(box->lpoly);
free(box);
}


static void free_SD(SDpt SD){
if (SD->tobefreededA) free(SD->lA);
free(SD->lB);
free(SD);
}


static void free_rigid(Rigid_structure *rigido, int  polypep){
int j;
Bboxpt *lbox= rigido->lbox;
for (j=0;j < rigido->nboxes;j++) free_bbox(*(lbox++), polypep); //free_bbox(rigido->lbox[j]);
if (rigido->nSD > 0) free(rigido->lSD);
free(rigido->lbox);
free(rigido);
}


void free_robot(Robot_structure *robot){
int i;
Rigid_structure **rigids;
Joint_tablespt *tables;
SDpt SD, *lSD;
int index;
if (robot==NULL) return;
number_of_bcd_robots--;
printf("estoy freeeando un robot y como polypep es %d\n", robot->polypep);
rigids=robot->rigids;

lSD= robot->inter_bb;
index= robot->intercol_index;
for(i=0;i < index; i++){
	SD=  *(lSD++);
	free_SD(SD);
	}
if (index > 0) free(robot->inter_bb);

lSD= robot->intra_res;
index= robot->autocol_index;
for(i=0;i < index; i++){
	SD=  *(lSD++);
	free_SD(SD);
	}
if (index > 0) free(robot->intra_res);

lSD= robot->loops;
index= robot->loop_index;
for(i=0;i < index; i++){
	SD=  *(lSD++);
	free_SD(SD);
	}
if (index > 0) free(robot->loops);



#ifdef SUPRESSION
for (i=0;i < robot->nrigids;i++) freeboxhierarchy2(*(rigids++));
/*In supression mode the master copy (the one that is freeded and created 
at each time) is inrigis->root. In roots[] there are only copies 
(only of the active ones), and old versions of pointers. */

#else
for (i=0;i < robot->nrigids;i++) freeboxhierarchy(robot,i);
#endif 
freesuphierarchy(robot);
free(robot->roots);
rigids=robot->rigids;
for (i=0;i < robot->nrigids;i++) free_rigid(*(rigids++), robot->polypep);
free(robot->rigids);
#ifdef SUPRESSION
if (robot->polypep){
	tables= robot->joint_tables;
	for (i=0;i < robot->n_aminos;i++,tables++) if (*tables) free(*tables);
//	for (i=0;i < robot->n_aminos;i++) 
//		if (robot->joint_tables[i]) free(robot->joint_tables[i]);
	free(robot->joint_tables);
	free(robot->amino_rigidbb);
	free(robot->amino_rigidcl);
	}
#endif
free(robot);
}




void bio_init_molecules(double scale)
{
  int i;
  pp3d_env currentenv = (p3d_env*) p3d_get_desc_curid(P3D_ENV);
  p3d_rob** env_robots = currentenv->robot;
  int nrobots = p3d_get_desc_number(P3D_ROBOT);
  int prev_nsubrobots = 0;

  if(number_of_bcd_robots)
  {
    printf("ERROR: bio_init_molecules called twice. Nothing done! \n");
    return;
  }
  for (i=0; i < nrobots; i++)
  {
    robot_p3d[i] = number_of_bcd_robots;
    my_create_molecule(env_robots[i], number_of_bcd_robots, 0, scale);
    prev_nsubrobots= num_subrobots[i] = number_of_bcd_robots - prev_nsubrobots;
  }
}


void bio_create_molecule(int i, double scale){
if (number_of_bcd_robots)
	my_create_molecule(NULL, i, 0, scale);
else printf("ERROR: bio_create_molecule called without a previous invocation to bio_init_molecules. Nothing done! \n");
}

void bio_create_molecules(double scale){
int i;
if (number_of_bcd_robots){
	for (i=0; i < number_of_bcd_robots; i++)
		bio_create_molecule(i, scale);
	}
else{
	printf("WARNING: bio_create_molecules called without a previous invocation to bio_init_molecules. Automatic invocation of bio_init_molecules \n");
	bio_init_molecules(scale);
	}
}

 void  p3d_start_bio_col(void){
//aqui podrian estar malloc de collisions1, collisions2 y robots por ejemplo
//int nrigid1[]={ 0, 1, 1, 2, 3};
//int natom1[]= { 0, 0, 0, 0, 0};
//int nrigid2[]={ 1, 2, 5, 3, 4};
//int natom2[]= { 0, 0, 0, 0, 0};
//printf("Estoy en p3d_start_bio_col \n");

bio_set_col_mode(NORMAL_BIOCOL_MODE);
bio_init_molecules(1.0);
//supress_sc_rigid_sequence(0,12,29);
//bio_set_autocol(1, 5, nrigid1,natom1,nrigid2,natom2);
}

/* 
founds all the pairs fulfilling the operation mode condition involving all 
robots. Returns the numbers of collisions in the first two operation modes.
This function should stop as soon as it founds more collisions than requested:
introduce tests after each robot collision checking */

int bio_all_molecules_col(void){
int i,j;
Robot_structure *roboti;
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
//printf("1");
//printf("comienzan las autocolsiones\n");
for (i=0;i < number_of_bcd_robots; i++){
    roboti=bcd_robots[i];
    rebuild(roboti);
//    printf("COLISIONES es %d \n", n_collisions);
//    printf("empiezan las preprogramadas del %d\n",i);
    preprogr_collisions(roboti);
//        printf("COLISIONES es %d \n", n_collisions);
//    printf("terminan las preprogramadas del robot %d\n", i);
    my_robot_autocollision(roboti);
//        printf("despues de la autocolision de %d COLISIONES es %d \n", i, n_collisions);
    }
//printf("acaban las autocolsiones\n");
box_collision= box_coll_general;
for (i=0;i < number_of_bcd_robots; i++)//for (i=0;i < number_of_bcd_robots; roboti= bcd_robots[i++])
	for (j=i + 1;j < number_of_bcd_robots; j++){
   //         printf("empiezan las colisiones de %d con el %d\n", i, j);
	 
		sup_collision(bcd_robots[i]->superroot, bcd_robots[j]->superroot);
//                printf("despues de la autocolision de %d con %d COLISIONES es %d \n", i, j, n_collisions);
                }

return get_n_collisions();
}

/* 
founds all pairs composed of an atom of robot number 
nrobot  and an atom of any  other robot, including 
nrobot. IT ASSUMES THAT ONLY ROBOT nrobot HAS CHANGED 
OF CONFIGURATION SINCE THE LAST REQUEST. More efficient 
in this case than combining the other functions.*/

int bio_molecule_col(int nrobot){
int i;
Robot_structure *robot=bcd_robots[nrobot];
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
rebuild(robot);
preprogr_collisions(robot);
my_robot_autocollision(robot);

box_collision= box_coll_general;

for (i=0;i < nrobot; i++)
		sup_collision(bcd_robots[i]->superroot, robot->superroot);
for (i=nrobot + 1;i < number_of_bcd_robots; i++)
		sup_collision(bcd_robots[i]->superroot, robot->superroot);

return get_n_collisions();
}

/*
Like the bio_all_molecules_col, but also updates collision FLAGS 
in the BCD robot structures (wich can be consulted with 
robot_report, see below).
*/
int bio_all_molecules_col_with_report(void){
int ncolbefore,i,j;
Robot_structure *roboti, *robotj;
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
//printf("comienzan las autocolsiones\n");
for (i=0;i < number_of_bcd_robots; i++){
	//printf("empiezan la autocolision del robot %d\n", i);
    roboti=bcd_robots[i];
    rebuild(roboti);
		roboti->colliding= FALSE;
		ncolbefore= get_n_collisions();
    //printf("empiezan las preprogramadas\n");
    preprogr_collisions(roboti);
    //printf("terminan las preprogramadas\n");
    my_robot_autocollision(roboti);
		if (ncolbefore < get_n_collisions()) {
			roboti->colliding= TRUE;
			//printf("ha encontrado autocolision\n");
			}
    }
//printf("acaban las autocolsiones\n");
box_collision= box_coll_general;
for (i=0;i < number_of_bcd_robots; i++)
	for (j=i + 1,roboti= bcd_robots[i];j < number_of_bcd_robots; j++){
		robotj= bcd_robots[j];
		ncolbefore= get_n_collisions();
    //printf("empiezan las colisiones de %d con el %d\n", i, j);
		sup_collision(bcd_robots[i]->superroot, bcd_robots[j]->superroot);
		if (ncolbefore < get_n_collisions()){
			 roboti->colliding= TRUE;
			 robotj->colliding= TRUE;
			 }
     }
//for (i=0;i < number_of_bcd_robots; i++) printf("collision robot %d: %d\n", i, bcd_robots[i]->colliding);
//printf("Verdaderas colisiones:\n");
return get_n_collisions();
}

/* 
founds all pairs composed of an atom of robot number 
nrobot  and an atom of any  other robot, excluding 
nrobot. */

int bio_molecule_col_no_autocol(int nrobot){
int i;
Robot_structure *roboti;
Robot_structure *robot=bcd_robots[nrobot];

set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
rebuild(robot);

box_collision= box_coll_general;

for (i=0;i < nrobot; i++){
		roboti=bcd_robots[i];
		rebuild(roboti);
		sup_collision(roboti->superroot, robot->superroot);
		}
for (i=nrobot + 1;i < number_of_bcd_robots; i++){
		roboti= bcd_robots[i];
		rebuild(roboti);
		sup_collision(roboti->superroot, robot->superroot);
		}

return get_n_collisions();
}



static int AFFICHAGE_EN_ROUGE = TRUE;
void set_affichage_en_rouge(int affich){AFFICHAGE_EN_ROUGE = affich;}
int get_affichage_en_rouge(void){return AFFICHAGE_EN_ROUGE;}


/* Reports if any of the sub-robots in robot number nrobot is in collision.
Needs a previous call to bio_all_molecules_col_with_report
*/
int biocol_robot_report(int nrobot){
int i= robot_p3d[nrobot], last= i + num_subrobots[nrobot];

while ( i < last && !(bcd_robots[i]->colliding))
	i++;
	
return ((i != last) && AFFICHAGE_EN_ROUGE);
}


/* returns the number of (sub)robots of which BioCD is aware */
int get_number_of_bcd_robots(void){
return number_of_bcd_robots;
}


/*
founds all pairs inside robot number i.
*/
int bio_molecule_autocol(int i){
Robot_structure *roboti;
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
roboti=bcd_robots[i];
rebuild(roboti);
preprogr_collisions(roboti);
my_robot_autocollision(roboti);

return get_n_collisions();
}

/*
founds all pairs consisting of one atom of robot number 
i and other atom of robot number j. i and j must be different!!!
*/
int bio_two_molecule_col(int i, int j){
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
box_collision= box_coll_general;
rebuild(bcd_robots[i]);
rebuild(bcd_robots[j]);
sup_collision(bcd_robots[i]->superroot, bcd_robots[j]->superroot);
return get_n_collisions();
}

//void set_charlatan(int i){ charlatan = i;}

#ifdef SUPRESSION

/*
to supress or inactivate the atoms of a pseudo-side-chains, 
use this function indicating the polypeptide with nrobot, and 
the amino number namino. Thepseudo-side-chain must be a rigid
*/

int supress_sc_rigid_sequence(int nrobot,int first,int last){
Robot_structure *robot= bcd_robots[nrobot];
SDpt SD, *lSD;
Rigid_structure *r;
int namino,i,nSD;

for (namino= first; namino<=last; namino++){
    r=robot ->amino_rigidcl[namino];
	if (r){
		if (r->nboxes > 1) 
		 printf("\n supress_sc_rigid_sequence ERROR: side chain %d does not constitute a rigid\n", namino);
		r->active= 0;
    	//r->lbox[0]->active= 0;
		lSD= r->lSD;
		nSD= r->nSD;
		for (i=0; i< nSD; i++){
			SD=  *(lSD++);
			SD->active= 0;
			}
		}
    }
return 1;
}

/*
Inactivates all the side-chain between aminoacid numbers first 
and last. All side chains must be rigids 
*/  
int activate_sc_rigid_sequence(int nrobot,int first,int last){
Robot_structure *robot= bcd_robots[nrobot];
SDpt SD, *lSD;
Rigid_structure *r;
int namino,i,nSD;

for (namino= first; namino<=last; namino++){
    r=robot->amino_rigidcl[namino];
    if (r){
		if (r->nboxes > 1)
		  printf("\n activate_sc_rigid_sequence ERROR: side chain %d does not constitute a rigid\n", namino);
		r->active= 1;
		//r->lbox[0]->active= 1;
		lSD= r->lSD;
		nSD= r->nSD;
		for (i=0; i< nSD; i++){
			SD= *(lSD++);
			if (SD->rigidA == r){
				if (!(SD->rigidB) || SD->rigidB->active) SD->active= 1;
				}
			else if (SD->rigidA->active) SD->active= 1;
			}
		}
    }
return 1;
}

/* Revert the effect of the previous functions */
int activate_sc_rigid(int nrobot,int namino){
Robot_structure *robot= bcd_robots[nrobot];
SDpt SD, *lSD;
Rigid_structure *r;
int i,nSD;

r=robot->amino_rigidcl[namino];
if (r){
	if (r->nboxes > 1) 
	 printf("\n activate_sc_rigid ERROR: side chain %d does not constitute a rigid\n", namino);
	r->active= 1;
	//r->lbox[0]->active= 1;
	lSD= r->lSD;
	nSD= r->nSD;
	for (i=0; i< nSD; i++){
		SD= *(lSD++);
		if (SD->rigidA == r){
			if (!(SD->rigidB) || SD->rigidB->active) SD->active= 1;
			}
		else if (SD->rigidA->active) SD->active= 1;
		}
	}

return 1;
}

int supress_sc_rigid(int nrobot,int namino){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure *r;
SDpt SD, *lSD;
int i,nSD;

r=robot->amino_rigidcl[namino];
if (r){
	if (r->nboxes > 1)
	 printf("\n supress_sc_rigid ERROR: side chain %d does not constitute a rigid\n", namino);
	r->active= 0;
	//r->lbox[0]->active= 0;
	lSD= r->lSD;
	nSD= r->nSD;
	for (i=0; i< nSD; i++){
		SD=  *(lSD++);
		SD->active= 0;
		}
	}

return 1;
}


/******                            ****
 The equivalent functiones for pseudo-backbones. 
The pseudo-backbone in the arguments, must be rigids.

******                            ****/

int supress_bb_rigid_sequence(int nrobot,int first,int last){
Robot_structure *robot= bcd_robots[nrobot];
SDpt SD, *lSD;
Rigid_structure *r;
int namino,i,nSD;

for (namino= first; namino<=last; namino++){
    r=robot ->amino_rigidbb[namino];
	if (r->nboxes > 1)
	 printf("\n supress_bb_rigid_sequence ERROR: backbone %d does not constitute a rigid\n", namino);
	r->active= 0;
	//r->lbox[0]->active= 0;
	lSD= r->lSD;
	nSD= r->nSD;
	for (i=0; i< nSD; i++){
		SD=  *(lSD++);
		SD->active= 0;
		}
    }
return 1;
}
    
int activate_bb_rigid_sequence(int nrobot,int first,int last){
Robot_structure *robot= bcd_robots[nrobot];
SDpt SD, *lSD;
Rigid_structure *r;
int namino,i,nSD;


for (namino= first; namino<=last; namino++){
    r=robot->amino_rigidbb[namino];
	if (r->nboxes > 1) printf("\n activate_bb_rigid_sequence ERROR: backbone %d does not constitute a rigid\n", namino);
	r->active= 1;
	//r->lbox[0]->active= 1;
	lSD= r->lSD;
	nSD= r->nSD;
	for (i=0; i< nSD; i++){
		SD= *(lSD++);
		if (SD->rigidA == r){
			if (!(SD->rigidB) || SD->rigidB->active) SD->active= 1;
			}
		else if (SD->rigidA->active) SD->active= 1;
		}
    }
return 1;
}

int activate_bb_rigid(int nrobot,int namino){
Robot_structure *robot= bcd_robots[nrobot];
SDpt SD, *lSD;
Rigid_structure *r;
int i,nSD;

r=robot->amino_rigidbb[namino];
if (r->nboxes > 1)
 printf("\n activate_bb_rigid ERROR: backbone %d does not constitute a rigid\n", namino);
r->active= 1;
//r->lbox[0]->active= 1;
lSD= r->lSD;
nSD= r->nSD;
for (i=0; i< nSD; i++){
	SD= *(lSD++);
	if (SD->rigidA == r){
		if (!(SD->rigidB) || SD->rigidB->active) SD->active= 1;
		}
	else if (SD->rigidA->active) SD->active= 1;
	}
return 1;
}

int supress_bb_rigid(int nrobot,int namino){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure *r;
SDpt SD, *lSD;
int i,nSD;

r=robot->amino_rigidbb[namino];
if (r->nboxes > 1) 
 printf("\n supress_bb_rigid ERROR: backbone %d does not constitute a rigid\n", namino);
r->active= 0;
//r->lbox[0]->active= 0;
lSD= r->lSD;
nSD= r->nSD;
for (i=0; i< nSD; i++){
	SD=  *(lSD++);
	SD->active= 0;
	}

return 1;
}

/*
returns a table in which element n provides a pointer to an structure 
describing several aspects of aminoacid number n:
typedef struct Joint_Tables {
    struct jnt* bb_joints[3]; // pointers to backbone joints 
    struct jnt* sc_joints[6];  // pointers to side chain joints 
    int namino;
    int n_bb_joints;				// number of backbone joints 
    int n_sc_joints;			// number of side chain joints 
} Joint_tables,*Joint_tablespt;
*/
Joint_tablespt *give_joint_tables(int nrobot){
return bcd_robots[nrobot]->joint_tables;
}

/*
Return pairs formed by one atom of the pseudo-side-chain of the amino number namino
 of robot number nrobot, and any atom of the same robot (including the pseudo-side-chain) 
 or any other robot of the environment. IT ASSUMES THAT ONLY ROBOT nrobot HAS 
 CHANGED OF CONFIGURATION SINCE THE LAST REQUEST . The pseudo-side-chain MUST be a rigid. 
The pseudo-side-chain could exist or not (Proline, Glycine..), but if it exists,
it MUST be a rigid. (i.e., some degrees of freedom of the pseudo chain are freeded).
 */
int bio_sc_col(int nrobot, int namino){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure *r=robot->amino_rigidcl[namino];
SDpt SD, *lSD;
int i,nSD;
supnodept nodeaux;
boxnodept root;
double *minvec;
double *maxvec;
double *rootmin;
double *rootmax;
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
rebuild(robot);


if (r) 
#ifdef SUPRESSION
	if (r->active)
#endif
	{
	lSD= r->lSD;
	nSD= r->nSD;
	for (i=0; i< nSD; i++){
		SD=  *(lSD++);
#ifdef SUPRESSION
		if (SD->active)
#endif
			(*(SD->function)) (SD); 
		}
//	printf("He acabado las autocolisiones \n");
/*  creation of an auxiliary temporary boxnodept */
	nodeaux= (supnodept) malloc(sizeof(supnode));
	minvec=nodeaux->min;
	maxvec=nodeaux->max;
	root= r->root;
	nodeaux->nnodes= 1;
	rootmin= root->min;rootmax= root->max;
	*minvec= *rootmin;*(++minvec)= *(++rootmin);*(++minvec)= *(++rootmin);
	*maxvec= *rootmax;*(++maxvec)= *(++rootmax);*(++maxvec)= *(++rootmax);
	nodeaux->left=NULL;
	nodeaux->right=NULL; 
	nodeaux->leaf=1;
	nodeaux->leftbrother= NULL;  
	nodeaux->lnode= (boxnodept *) malloc(sizeof(boxnodept));
	nodeaux->lnode[0]= root;
/* end creation */

/* collision */
	box_collision= box_coll_withinProt;
	sup_collision(robot->superroot, nodeaux);	
/* A the end of the recursion there will be a call to leaf_sup_collision(
leafNode, nodeaux) with leafNode being a leaf node containing the same rigid as 
nodeaux and eventually some other rigids. In general, calls to 
leaf_sup_collision with non-leaf arguments will cause calls to collision between
the same rigid, but in this case as the rigid contains only a bbox side chain, 
the only bbox collision will be obviated in the inmediate lower level.
In other cases a temporally supression of the rigid *before rebuild* will suffice,
but il will be necessary to rebuild the side chain independently */

/********** MODIFICATION FOR COLLIDING WITH ALL SUBROBOTS ********/
box_collision= box_coll_general;
for (i=0;i < nrobot; i++)
		sup_collision(bcd_robots[i]->superroot, robot->superroot);
for (i=nrobot + 1;i < number_of_bcd_robots; i++)
		sup_collision(bcd_robots[i]->superroot, robot->superroot);
/**********			END MODIFICATION                      ********/	

/*freeing the temporary node */												
	free(nodeaux->lnode);
	free(nodeaux);
	}
return get_n_collisions();
}





/*
pairs formed by one atom of the pseudo-backbone of the amino number namino 
of robot number nrobot, and any atom of the same robot (including pseudo-backbone).
The pseudo-backbone MUST be a rigid. 
(i.e. if any of the side chain joints are
freeded (when they exist) and at least one degree of freedom of their bb 
is liberated or at least  one degree of freedom of the previous (if any) and one 
of the next (if any) residual backbones are freeded.)
----------------------
The way it works now its is not general. A new node of the high level hierarchy 
is created containing only the backbone is created , but it does not belong to
 the hierarch of the proteine. In fact, inthis hierarchy there is another leaf
node containing also the bb (and perhaps other things).
When the two nodes collide they will be treated as belonging to different
 molecules and then all
low level-collision boxes are tested. No collision is effectively found because
of box_collision= box_coll_withinProt which will exlcude the possibility of 
effectively testing the backbone with itself.
Alternative solutions more general that will work also for the autocollision
of a general rigid with its protein:
1) desactivate temporally the node just before rebuilding the robot. All this 
must be done after executing the SD's
2) More difficult: having leaves of 1 rigid in the high-level hierarchy and
using the corresponding leaf containing the rigid for autocollision. The problem
is how we know which is the leaf containing the rigid.
*/
int bio_bb_col(int nrobot, int namino) {
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure *rbb=robot->amino_rigidbb[namino];
supnodept nodeaux;
boxnodept root;
SDpt SD, *lSD;
int nSD,i;
double *minvec;
double *maxvec;
double *rootmin;
double *rootmax;
set_n_collisions_to_zero();
set_minimum_Sdistance_to_max();
rebuild(robot);

if (rbb->nboxes > 1) printf("\n Autocollision _bb ERROR: The backbone %d does not constitute a rigid\n", namino);
#ifdef SUPRESSION
if (rbb->active)
#endif

	{
	lSD= rbb->lSD;
	nSD= rbb->nSD;
	for (i=0; i< nSD; i++){
		SD=  *(lSD++);
#ifdef SUPRESSION
		if (SD->active)
#endif
			(*(SD->function)) (SD); 
		} 
//	printf("He acabado las autocolisiones \n");
/*  creation of an auxiliary temporary boxnodept */
	nodeaux= (supnodept) malloc(sizeof(supnode));
	minvec=nodeaux->min;
	maxvec=nodeaux->max;
	root= rbb->root;
	nodeaux->nnodes= 1;
	rootmin= root->min;rootmax= root->max;
	*minvec= *rootmin;*(++minvec)= *(++rootmin);*(++minvec)= *(++rootmin);
	*maxvec= *rootmax;*(++maxvec)= *(++rootmax);*(++maxvec)= *(++rootmax);
	nodeaux->left=NULL;
	nodeaux->right=NULL; 
	nodeaux->leaf=1;
	nodeaux->leftbrother= NULL;  
	nodeaux->lnode= (boxnodept *) malloc(sizeof(boxnodept));
	nodeaux->lnode[0]= root;
/* end creation */

/* collision */
	box_collision= box_coll_withinProt;
	sup_collision(robot->superroot, nodeaux);	
/* At the end of the recursion there will be a call to leaf_sup_collision(
leafNode, nodeaux) with leafNode being a leaf node containing the same rigid as 
nodeaux and eventually some other rigids. In general, calls to 
leaf_sup_collision with non-leaf arguments will cause calls to collision between
the same rigid, but in this case as the rigid contains only a bbox side chain, 
the only bbox collision will be obviated in the inmediate lower level.
In other cases a temporally supression of the rigid *before rebuild* will suffice,
but il will be necessary to rebuild the side chain independently */

/*freeing the temporary node */												
	free(nodeaux->lnode);
	free(nodeaux);
	}
return get_n_collisions();
}

/**
* bioGetInvMinLigProtDist
* Get the inverse of the minimal distance
* between the ligand and the protein.
* Function used to get a rough approximation of 
* the energy function (used in cost spaces)
* @return: the inverse of the minimal distance
*/
double bioGetInvMinLigProtDist(void) {
double dist = P3D_HUGE;
double minDist = P3D_HUGE;
int indexligand; 
p3d_poly *p1,*p2;
int currentColMode, i;
currentColMode = bio_get_col_mode();
bio_set_col_mode(MINIMUM_DISTANCE_BIOCOL_MODE);
indexligand = get_number_of_bcd_robots() - 1; 
i =0;
    while(i<indexligand) {
      bio_two_molecule_col(i, indexligand); 
      dist = min_dist_report( &p1, &p2);
//      PrintInfo(("Dist: %f \n", dist));
      minDist = MIN(dist, minDist);
      i++;
      // TEST
      //biocol_report_dist(&n_dist_pairs, &p1, &p2, &distances);
    }
bio_set_col_mode(currentColMode);
//PrintInfo(("minDist: %f \n", minDist));
//return exp(-minDist);
return 1/minDist;
}



/**
* bioGetLigRotaMotionFromInit
* Get the average variation of the roational
* degrees of freedom of the ligand
* @return: the rotationnal variation (Between 
* -Pi and Pi)
*/
double bioGetLigRotaMotionFromInit(p3d_rob* robotPt,  configPt currentConf) {
int indexligand, i, j, k, counter = 0;
configPt ConfigStart;
int njnt = robotPt->njoints;
double deltaTheta;
p3d_jnt * jntPt;

ConfigStart = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
indexligand = get_number_of_bcd_robots() - 1; 
for(i = indexligand; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if ( (p3d_jnt_get_dof_is_user(jntPt, j)) &&  
           (jntPt->bio_jnt_type ==  BIO_OTHER_JNT) && 
           (p3d_jnt_is_dof_angular(jntPt, j)) ) {
deltaTheta += dist_circle(ConfigStart[k], currentConf[k]);

counter++;
	}
     }
  }

p3d_destroy_config(robotPt,ConfigStart);
if(counter !=0) {
return deltaTheta/(counter*M_PI);
}
return 0.;
}

#endif
