/*******************************************************/
/***************** bcd_test.c *******************/
/*******************************************************/

/*
Some testing functions using the functionalities of the BCD. 
This is not really a part of the BCD package, but something built upon it.
The material here is rather dirty. A user-level graphical interface could be
built here */

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Collision-pkg.h"
#include "Bio-pkg.h"

#include "include/bcd_global.h"
//#include "include/bcd_util.h"
//#include "include/bcd_resize.h"
//#include "include/bcd_hierarchies.h"


extern double p3d_random(double a, double b);
extern void p3d_init_random_seed(int seed);

void print_robot(int nrobot){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure **rigids=robot->rigids;
Rigid_structure *rigido;
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int i,j,k,natoms;

//printf("shrink es %f\n", shrink);
printf("nrigids es %d\n", robot->nrigids);
printf("autocol_index es %d\n", robot->autocol_index);
printf("intercol_index es %d\n", robot->intercol_index);
printf("loop_index es %d\n", robot->loop_index);
for (i=0;i < robot->nrigids;i++){
    rigido= *(rigids++);
	printf("\t rigido numero %d tiene %d cajas\n", i, rigido->nboxes);
	printf("\t  reference=%2.2f, %2.2f, %2.2f   reference_vec=%2.2f, %2.2f, %2.2f\n",
	rigido->reference->matpos[0][3],rigido->reference->matpos[1][3],rigido->reference->matpos[2][3],
	rigido->reference->pos[0],rigido->reference->pos[1],rigido->reference->pos[2] );
    lbox= rigido->lbox;
    for (j=0;j < rigido->nboxes;j++){
        bbox= *(lbox++);
      	printf("\t\t tipoamino %d , namino %d, backbone %hd, natoms %d loop %d \n",
		bbox->aminotype, bbox->namino, bbox->bb, bbox->natoms,  bbox->loop); 
		lpoly= bbox->lpoly;
        natoms= bbox->natoms;
        for (k=0;k < natoms;k++){
            polygordo= *(lpoly++);
            printf("A %d %s r=%1.1f pos=%2.1f %2.1f %2.1f  ",
				k, polygordo->poly->name, 
#ifdef HYDROGEN_BOND
				*(polygordo->r),
#else				
				polygordo->r,
#endif
				bbox->lpoly[k]->pos[0],bbox->lpoly[k]->pos[1],bbox->lpoly[k]->pos[2]);
			if ((k==1) || (k==3) || (k==5)|| (k==natoms-1)) printf("\n");
            }
        }
    }
}



static void test_latotale(pp3d_rob protein, double scale){
int first_time= 1;
int i;
set_required_collisions_to_max();
set_n_collisions_to_zero();
charlatan=1;

if (first_time){ 
//	printf("antes de create robot\n");
#ifdef SUPRESSION	
	printf("SUPRESSION ACTIVO\n");
#endif
	bio_set_col_mode(NORMAL_BIOCOL_MODE);
	bio_create_molecules(scale);
	printf("la variable HYDROGEN_ENVIRONEMENT es %d \n", HYDROGEN_ENVIRONEMENT);
	}
			
else bio_resize_molecules(scale);


bio_all_molecules_col_with_report();printf("numero de colisiones es %d \n", get_n_collisions());
for (i=0;i < number_of_bcd_robots; i++) printf("collision robot %d: %d\n", i, bcd_robots[i]->colliding);

printf("\n");
//print_robot(0);
printf("\n");

supress_sc_rigid_sequence(0,12,29);
supress_bb_rigid_sequence(0,12,29);
for (i=12;i <= 29; i++){
	activate_bb_rigid(0, i);
	printf("he activado el backbone %d \n", i);
	bio_bb_col(0,i);
	}
for (i=12;i <= 29; i++){
	activate_sc_rigid(0, i);
	printf("he activado la cadena lateral %d \n", i);
	bio_sc_col(0,i);
	} 
/*	printf("SEGUNDA RONDA\n\n");
supress_sc_rigid_sequence(0,12,29);
supress_bb_rigid_sequence(0,12,29);
for (i=12;i <= 29; i++){
	activate_bb_rigid(0, i);
	printf("he activado el backbone %d \n", i);
	bio_bb_col(0,i);
	}
for (i=12;i <= 29; i++){
	activate_sc_rigid(0, i);
	printf("he activado la cadena lateral %d \n", i);
	bio_sc_col(0,i);
	} 
	printf("STERCERA RONDA\n\n");
supress_sc_rigid_sequence(0,12,29);
supress_bb_rigid_sequence(0,12,29);
for (i=12;i <= 29; i++){
	activate_bb_rigid(0, i);
	printf("he activado el backbone %d \n", i);
	bio_bb_col(0,i);
	}
	
for (i=12;i <= 29; i++){
	activate_sc_rigid(0, i);
	printf("he activado la cadena lateral %d \n", i);
	bio_sc_col(0,i);
	} 	
	
	printf("CUARTA RONDA\n\n");
supress_sc_rigid_sequence(0,12,29);
supress_bb_rigid_sequence(0,12,29);
for (i=12;i <= 29; i++){
	activate_bb_rigid(0, i);
	printf("he activado el backbone %d \n", i);
	bio_bb_col(0,i);
	}	
for (i=12;i <= 29; i++){
	activate_sc_rigid(0, i);
	printf("he activado la cadena lateral %d \n", i);
	bio_sc_col(0,i);
	}  
*/

printf("numero de colisiones es %d \n", get_n_collisions());
}	

static void show_autocol(int nrobot){
Robot_structure *robot= bcd_robots[nrobot];
Rigid_structure **rigids=robot->rigids;
Rigid_structure *rigido;
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int i,j,k,natoms,l;
autocol_struct *autocol;

for (i=0;i < robot->nrigids;i++){
    rigido= *(rigids++);
	printf("\t rigido numero %d tiene %d cajas\n", i, rigido->nboxes);
	lbox= rigido->lbox;
    for (j=0;j < rigido->nboxes;j++){
        bbox= *(lbox++);
      	printf("\t\t  natoms %d \n", bbox->natoms); 
		lpoly= bbox->lpoly;
        natoms= bbox->natoms;
        for (k=0;k < natoms;k++){
            polygordo= *(lpoly++);
			autocol= polygordo->autocol_data;
			if (autocol){
				printf("atomo numero %d , %s tiene %d autocol (numero de rigido %d):\n", k, polygordo->poly->name, autocol->size,autocol->nrigid); 
				for (l=0; l<  autocol->size; l++)
					printf("%s ", autocol->list[l]->poly->name);
				printf("\n");
				}
			}
        }
    }
}


static void simple2(pp3d_rob protein, double scale){
/* mio
int nrigid1[]={ 0, 1, 1, 2, 3};
int natom1[]= { 0, 0, 0, 0, 0};
int nrigid2[]={ 1, 2, 5, 3, 4};
int natom2[]= { 0, 0, 0, 0, 0};
*/
/*benoit */
/*int nrigid1[] = {2,3,4,2,1};
  int natom1[] =  {0,0,0,0,0};
  int nrigid2[] = {3,4,5,1,0}; 
  int natom2[] =  {0,0,0,0,5};
*/
/*  int nrigid1[] = {0,1,2,3,4};
  int natom1[] =  {5,0,0,0,0};
  int nrigid2[] = {1,2,3,4,5}; 
  int natom2[] =  {0,0,0,0,0};
*/
 p3d_poly **p1;
 p3d_poly **p2;	
 //p3d_poly *a1, *a2;
int first_time= 1;
int i, aux;

set_required_collisions_to_max();
set_n_collisions_to_zero();
charlatan=0;
printf("la variable scale es %1.4f \n,", scale);
#ifdef HYDROGEN_BOND
printf("HYDROGEN_BOND esta activo en simple2\n");
#endif
if (first_time){ 
//	printf("antes de create robot\n");
#ifdef SUPRESSION	
	printf("SUPRESSION ACTIVO\n");
#endif
	printf("antes de set autocol\n");

	printf("despues de set autocol\n");
	bio_set_col_mode(NORMAL_BIOCOL_MODE);
	//bio_set_col_mode(MINIMUM_DISTANCE_BIOCOL_MODE);
	bio_init_molecules(scale);
	printf("la variable HYDROGEN_ENVIRONEMENT es %d \n", HYDROGEN_ENVIRONEMENT);
	printf("El Numero de robots es %d \n", number_of_bcd_robots);
	//for (i=0;i < number_of_bcd_robots; i++){printf("pinto robot %d\n", i); print_robot(i);}
//printf("\n\n");

	//bio_true_resize_molecules();
	// bio_set_autocol(1, 5, nrigid1,natom1,nrigid2,natom2); // benoit
	//bio_set_autocol(0, 5, nrigid1,natom1,nrigid2,natom2);
	//show_autocol(1),
	first_time=0;
	printf("HOLA DESDE EL POWERMAC\n");
	}
			
else bio_resize_molecules(scale);
bio_true_resize_molecules();

printf("SI NECESITO CAMBiAR\n");
//bio_all_molecules_col_with_report();printf("numero de colisiones es %d \n", get_n_collisions()); 
//activate_sc_rigid(0, 22);
bio_all_molecules_col_with_report(); 
//write_SD_functions();
/*for (i=0;i < number_of_bcd_robots; i++) print_robot(i);
printf("\n\n"); */
//bio_bb_col(0,29);
for (i=0;i < number_of_bcd_robots; i++) printf("collision robot %d: %d\n", i, bcd_robots[i]->colliding);
biocol_report(&aux, &p1,&p2);
/*for (i=0;i <get_n_collisions(); i++) printf("%s choca con %s\n", p1[i]->poly->name,p2[i]->poly->name); */
//write_wdw_radii();
//for (i=0;i < get_n_collisions(); i++) printf("%s choca con %s\n", collisions1[i]->poly->name,collisions2[i]->poly->name);
/* rebuild(rob);
preprogr_collisions(rob);
bio_molecule_autocol(rob); */ 
//sup_collision(rob->superroot, rob->superroot);
/*min_dist_report(&a1, &a2);
 printf("los mas cercanos son %s y %s\n", a1->poly->name,a2->poly->name); */
printf("numero de colisiones es %d \n", get_n_collisions());
set_required_collisions(1);
}	


/****** 																																	***************/
/****** cont_pos(protein,iteraciones,j,initial)     random_pos(protein) ***************/
/****** 																																		***************/
static void random_pos(pp3d_rob protein){
int i,njoints=protein->njoints;
double min,max,aux;
//printf("zip es %d;", zip);
for (i=1;i<njoints;i++){
	//p3d_get_robot_jnt(i, &current);
	p3d_jnt_get_dof_bounds(protein->joints[i], 0, &min, &max);
	aux=p3d_random(min,max);
	p3d_jnt_set_dof(protein->joints[i], 0, aux);
	//printf("%2.2f, %2.2f, %2.2f\n", protein->joints[i]->vmin,protein->joints[i]->vmax,protein->joints[i]->v);
	}
p3d_update_robot_pos();
}

static void zigzag_change_pos(pp3d_rob protein){
int i,njoints=protein->njoints;
double current,min,max;
static int zip= 1;
//printf("zip es %d;", zip);
for (i=1;i<njoints;i++){
	current= p3d_jnt_get_dof(protein->joints[i], 0);
	p3d_jnt_get_dof_bounds(protein->joints[i], 0, &min, &max);
	p3d_jnt_set_dof(protein->joints[i], 0, current + zip * 0.00001 * (max - min));
	//printf("%2.2f, %2.2f, %2.2f\n", protein->joints[i]->vmin,protein->joints[i]->vmax,protein->joints[i]->v);
	}
p3d_update_robot_pos();
zip= zip * -1;
}

#ifndef HYDROGEN_BOND
void resize_NSO_percent(int nrobot, double  shrink){
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
						if (polygordo->type < CARBON)
            	polygordo->r= polygordo->r * shrink;
            }
        }
    }
robot->to_be_rebuilt=1;  
}
#endif

#define escala_cont_pos 0.0001  // 0.2

static void cont_pos(pp3d_rob protein, int iterations, int current_iter, double *initial){
int i,njoints=protein->njoints;
double init,min,max;
for (i=1;i<njoints;i++){
    p3d_jnt_get_dof_bounds(protein->joints[i], 0, &min, &max);
    init= initial[i];
    p3d_jnt_set_dof(protein->joints[i], 0, 
      init + current_iter * escala_cont_pos * (max - init) / iterations);
    /*if ((i== 470) && ((current_iter == 0) || (current_iter == iterations -1))) printf("%d %2.2f, %2.2f, %2.2f\n",
      current_iter, min, max,p3d_jnt_get_dof(protein->joints[i], 0)); */
	}
p3d_update_robot_pos();
}


/*
static void cont_pos(pp3d_rob protein, int iterations, int current_iter, double *initial){
int i,njoints=protein->njoints;
double init,min,max;
for (i=1;i<njoints;i++){
	//p3d_get_robot_jnt(i, &current);
	min=protein->joints[i]->vmin;
	max=protein->joints[i]->vmax;
	init= initial[i];
	p3d_set_robot_jnt(i,init + current_iter * escala_cont_pos * (max - init) / iterations);
	if ((i== 470) && ((current_iter == 0) || (current_iter == iterations -1))) printf("%d %2.2f, %2.2f, %2.2f\n", current_iter, protein->joints[i]->vmin,protein->joints[i]->vmax,protein->joints[i]->v);
	}
p3d_update_robot_pos();
}
*/

static void restore_conf(double *initial, int njoints){
int i;
for(i=0;i<njoints;i++) p3d_set_robot_jnt(i,initial[i]);
p3d_update_robot_pos();
p3d_init_random_seed(0);
}


static void testhydrogenbonds(pp3d_rob protein, double falsescale){
Robot_structure *rob=bcd_robots[0];
int iteraciones= 100000;
double scale= 0.2; 
double donor_aceptor_scale=  0.85;
int j,njoints=protein->njoints;
double averagecol;
int conf_in_col;
set_required_collisions_to_max();


p3d_init_random_seed(0);
printf ("numero de joints: %d\n", njoints);
printf ("proteina %s, iteraciones %d, escala general para todos los atomos %4.3f, donor_aceptor scale %4.3f\n",
 protein->name, iteraciones,scale,donor_aceptor_scale);
bio_set_col_mode(NORMAL_BIOCOL_MODE);
bio_create_molecules(scale);

bio_all_molecules_col_with_report();
printf("numero de colisiones es %d \n", get_n_collisions());
//write_SUM_VDW();


#ifdef HYDROGEN_BOND
reduce_don_acep_mat(donor_aceptor_scale);
printf("HYDROGEN_BOND esta ACTIVO\n");
#else
resize_NSO_percent(0,donor_aceptor_scale);
printf("HYDROGEN_BOND NO esta ACTIVO\n");
#endif
bio_all_molecules_col_with_report();
//write_SUM_VDW();
printf("numero de colisiones es %d \n", get_n_collisions());

conf_in_col=averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	preprogr_collisions(rob);
	my_robot_autocollision(rob);
	if (get_n_collisions()) conf_in_col++;
	averagecol+= get_n_collisions();
	}
averagecol= averagecol / iteraciones;
printf("s, conf. en colision %d,  %3.2f colisiones\n", 
			conf_in_col, averagecol);
}

static double sistematica2(pp3d_rob protein, double scale,
	int iteraciones, double *initial){
int j,njoints=protein->njoints;
int conf_in_col; 
double inicio,fin, aux, tiempo,averagecol,usual_averagecol,averagecol0;
double tinit,tpos,tposzig,trebuild,tprep,tlongdist,tcol,tcolzig,trebuilt_percent0;
Robot_structure *rob;

p3d_init_random_seed(0);
printf ("numero de joints: %d\n", njoints);
printf ("proteina %s, iteraciones %d, escala %4.3f\n", protein->name, iteraciones,escala_cont_pos);
/*for (i=0;i<njoints;i++){
 printf("%s,%2.2f, %2.2f\n ", protein->joints[i]->name,protein->joints[i]->vmin,protein->joints[i]->vmax);
 } */

charlatan=0;

ChronoOn();
ChronoTimes(&inicio, &aux);
bio_set_col_mode(NORMAL_BIOCOL_MODE);
#ifdef HYDROGEN_BOND
bio_init_molecules(0.8);
printf("compilado con HYDROGEN_BOND", tpos);
#else				
bio_init_molecules(scale);
#endif
rob=bcd_robots[0];
ChronoTimes(&fin, &aux);
tinit= fin - inicio;
printf("inicializacion:  %4.4f segundos\n", tinit);


/**************** unuseful action; only to "inject fuel to the machine" ***********/
/*
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	preprogr_collisions(rob);
	sup_collision(rob->superroot, rob->superroot);
	}
restore_conf(initial,njoints); */
/****************** end of unuseful action ******************************** */

			/*tpos */
ChronoTimes(&inicio, &aux);
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	}
ChronoTimes(&fin, &aux);
tpos= fin - inicio;
printf("pos:  %4.4f segundos\n", tpos);
restore_conf(initial,njoints);
//goto abrevia;
			/*trbuild */
ChronoTimes(&inicio, &aux);
averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	}
ChronoTimes(&fin, &aux);
//ChronoOff();
tiempo= fin - inicio;
averagecol= averagecol / iteraciones;
trebuild= tiempo - tpos;
printf("rebuild:  %4.4f segundos \n", trebuild);
restore_conf(initial,njoints);

			/* tshordistance */
ChronoTimes(&inicio, &aux);
averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	preprogr_collisions(rob);
	averagecol+= get_n_collisions();
//	printf("numero de colisiones es %d \n", get_n_collisions());
	}
ChronoTimes(&fin, &aux);
tiempo= fin - inicio;
averagecol= averagecol / iteraciones;
tprep= tiempo - (tpos + trebuild);
printf("preprog:  %4.4f segundos,  %3.2f colisiones\n", tprep, averagecol);
restore_conf(initial,njoints);

abrevia:
		/* tlongdistance */
ChronoTimes(&inicio, &aux);
conf_in_col=averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	preprogr_collisions(rob);
	my_robot_autocollision(rob);
	if (get_n_collisions()) conf_in_col++;
	averagecol+= get_n_collisions();
//	printf("numero de colisiones es %d \n", get_n_collisions());
	}
ChronoTimes(&fin, &aux);
tiempo= fin - inicio;
averagecol= averagecol / iteraciones;
tlongdist= tiempo - (tpos + trebuild + tprep);
printf("sup:  %4.4f segundos, conf. en colision %d,  %3.2f colisiones\n", 
			tlongdist,conf_in_col, averagecol);
restore_conf(initial,njoints);

tcol= tiempo - tpos;
usual_averagecol=averagecol;


//goto t0;
		/* tposzig */

ChronoTimes(&inicio, &aux);
for (j=0;j<iteraciones;j++){ 
	zigzag_change_pos(protein);
	}
ChronoTimes(&fin, &aux);
tposzig= fin - inicio;
printf("poszig:  %4.4f segundos\n", tposzig);
restore_conf(initial,njoints);

		/* tzig */

ChronoTimes(&inicio, &aux);
averagecol=0;
for (j=0;j<iteraciones;j++){ 
	zigzag_change_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	preprogr_collisions(rob);
	my_robot_autocollision(rob);
	averagecol+= get_n_collisions();
	}
ChronoTimes(&fin, &aux);
tiempo= fin - inicio;
printf("tiempo total escala pequegna %5.2f\n", tiempo);
averagecol= averagecol / iteraciones;
tcolzig= tiempo - tposzig;
restore_conf(initial,njoints);

averagecol0= averagecol;
t0:
		/* t0 */
set_n_collisions_to_zero();
rebuild(rob);
preprogr_collisions(rob);
my_robot_autocollision(rob);
if (get_n_collisions() != averagecol0) printf("\n         NO HAY LAS MISMAS COLISIONES!!!\n\n");
printf("colisiones en posicion inicial %d\n", get_n_collisions());
//goto acaba;
ChronoTimes(&inicio, &aux);
for (j=0;j<iteraciones;j++){ 
	set_n_collisions_to_zero();
	//rebuild(rob);
	preprogr_collisions(rob);
	my_robot_autocollision(rob);
	}
ChronoTimes(&fin, &aux);
tiempo= fin - inicio;
printf("tiempo total escala 0:  %5.2f\n", tiempo);

trebuilt_percent0= (100 *(tcolzig - tiempo ))/ tcolzig;
restore_conf(initial,njoints);

acaba:
//free_robot(rob);

printf("bio\n");
printf("%8s %7s %7s %7s %7s %7s %7s %7s %7s %7s\n",	"col_conf", "ncol", "tinit", "tpos", "tbuild", "tshort", "tlong",
 "tcol", "ncol0", "%build0");
printf(" %7d %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.1f %7.2f\n",
 conf_in_col,usual_averagecol,tinit,tpos,trebuild,tprep,tlongdist,tcol,averagecol0,trebuilt_percent0);


		/* tiempo surf_distance */
bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);
bio_set_surface_d(0.8);
ChronoTimes(&inicio, &aux);
conf_in_col=averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	rebuild(rob);
	preprogr_collisions(rob);
	my_robot_autocollision(rob);
	if (get_n_collisions()) conf_in_col++;
	averagecol+= get_n_collisions();
//	printf("numero de colisiones es %d \n", get_n_collisions());
	}
ChronoTimes(&fin, &aux);
tiempo= fin - inicio;
averagecol= averagecol / iteraciones;
printf("SURF_DISTANCE:  %4.4f segundos,   conf. en colision %d,  %3.2f colisiones\n", 
			tiempo - tpos, conf_in_col, averagecol);
restore_conf(initial,njoints);

		/* tiempo minimum_surf_distance */
bio_set_col_mode(MINIMUM_DISTANCE_BIOCOL_MODE);
ChronoTimes(&inicio, &aux);
conf_in_col=averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	set_n_collisions_to_zero();
	set_minimum_Sdistance_to_max();
	rebuild(rob);
	preprogr_collisions(rob);
	my_robot_autocollision(rob);
	if (get_n_collisions()) conf_in_col++;
	averagecol+= get_n_collisions();
//	printf("numero de colisiones es %d \n", get_n_collisions());
	}
ChronoTimes(&fin, &aux);
tiempo= fin - inicio;
averagecol= averagecol / iteraciones;
printf("MINIMUM_DISTANCE:  %4.4f segundos,   conf. en colision %d,  %3.2f colisiones\n", 
			tiempo - tpos, conf_in_col, averagecol);
restore_conf(initial,njoints);

bio_set_col_mode(NORMAL_BIOCOL_MODE);
ChronoOff();
return tpos;
}

static void kcd(pp3d_rob protein, int iteraciones, double tpos, double *initial){
int  j, ncol;
int conf_in_col;
double tcol,tinit; 
double inicio,fin, aux, tiempo,averagecol;
int col_mode_to_be_set;
//printf ("proteina %s, escala: %1.6f , iteraciones %d\n", protein->name, escala, iteraciones);

ChronoOn();
ChronoTimes(&inicio, &aux);
				col_mode_to_be_set = p3d_col_mode_kcd;
	      set_DO_KCD_GJK(TRUE);
				p3d_filter_switch_filter_mechanism(FALSE);
				p3d_col_set_mode(col_mode_to_be_set);
  			p3d_col_start(col_mode_to_be_set);
ChronoTimes(&fin, &aux);	
tinit= fin - inicio;
printf("inicializacion:  %4.4f segundos\n", tinit);

			
ChronoTimes(&inicio, &aux);
conf_in_col=averagecol=0;
for (j=0;j<iteraciones;j++){ 
	random_pos(protein);
	p3d_col_test();
	//kcd_robot_collides_itself(0 , JUST_BOOL);
	ncol= p3d_col_number();
	if (ncol) conf_in_col++;
	averagecol+= ncol;
	//printf("numero de colisiones es %d \n", ncol);
	}
ChronoTimes(&fin, &aux);
ChronoOff();
tiempo= fin - inicio;
tcol= tiempo - tpos;
averagecol= averagecol / iteraciones;
printf("kcd\n");
printf("%8s %7s %7s %7s %31s\n","col_conf","ncol","tinit","tpos","tcol");
printf(" %7d %7.2f %7.2f %7.2f %31.2f\n", conf_in_col, averagecol, tinit, tpos, tcol);
restore_conf(initial,protein->njoints);
}


static void test_collision(pp3d_rob protein, double scale){
int i,njoints=protein->njoints,iteraciones= 10000;
double *initial= (double *) malloc(njoints * sizeof(double));
double tpos,current;
for (i=0;i<njoints;i++){
// p3d_get_robot_jnt(i, &current);
  current= p3d_jnt_get_dof(protein->joints[i], 0);   
 initial[i]= current;
 }
set_required_collisions_to_max();
printf("MIN_IN_HIGH_HIERCHY es %d \n", giveme_mininhierarchy());
tpos=sistematica2(protein,scale,iteraciones,initial);
set_required_collisions(1);
sistematica2(protein,scale,iteraciones,initial);
kcd(protein, iteraciones,tpos,initial);
}

void prueba(pp3d_rob protein, double scale){
//set_required_collisions(949);
test_collision(protein,scale);
//test_latotale(protein,scale);
//simple2(protein,scale);
//testhydrogenbonds(protein, scale);

}

/*
static void simple(pp3d_rob protein, double scale){
int first_time= 1;
Robot_structure *rob= bcd_robots[0];
set_n_collisions_to_zero();
//free(collisions1);
//free(collisions2);
//collisions1= (p3d_poly **) malloc(MAX_NUMBER_OF_COLLISIONS * sizeof(p3d_poly *));
//collisions2= (p3d_poly **) malloc(MAX_NUMBER_OF_COLLISIONS * sizeof(p3d_poly *));
if (first_time){
//	printf("antes de create robot\n");
	bio_create_molecules(scale);
//	printf("despues de create robot\n");
	first_time=0;
	rebuild(rob);
	// if (charlatan) print_robot(rob);
	preprogr_collisions(rob);
	//printf("despues de preprogr_collisions\n");
	original_autocollision();
//	printf("despues de bio_molecule_autocol\n");
	}
else
	{
	bio_resize_molecules(scale);
	rebuild(rob);
	preprogr_collisions(rob);
	original_autocollision();
	}
printf("numero de colisiones es %d \n", get_n_collisions());
}

*/
/*
static void sistematica(pp3d_rob protein, double scale){
int i, j,njoints=protein->njoints;
int iteraciones=200;
double current,min,max,escala=0.2;
charlatan=0;
for (j=0;j<iteraciones;j++){ 
	for (i=0;i<njoints;i++){
		p3d_get_robot_jnt(i, &current);
		min=protein->joints[i]->vmin;
		max=protein->joints[i]->vmax;
		p3d_set_robot_jnt(i,current + j * escala * (max - current) / iteraciones);
		//printf("%2.2f, %2.2f, %2.2f\n", protein->joints[i]->vmin,protein->joints[i]->vmax,protein->joints[i]->v);
		}
	p3d_update_robot_pos();
	simple(protein, 0.1);
	}
}
*/


/* cosas para acelerar

- quitar algun if (charlatan)
- el create_rigid_root es ineficiente cuando hay una sola caja: contemplar el caso
- la funcion que falta para las colisiones preprogramadas. SINO, 
AL MENOS USAR LA FUNCION CHECK-ALL 
- preprogr_collisions (sobre todo en el caso de que todo sea libre
- hacer la jerarquia del nivel inferior tenga hojas con varias cajas
- poner en p3d_poly o donde sea toda la informacion relativa a un atomo: posicion, 
radio, radio de colision, etc . Asi, todo seria un poco mas sencillo, sobre todo
a la hora construir las estructuras.
Una caja tendria simplemente una tabla de apuntadores a atomos y no listas de varias
cosas. El hecho de que los pseudo-bb y pseudo-cl tuvieran menos memoria iria bien 
para mantener estas estructuras (para poder realizar intra e inter colisiones)
de forma redundante aunque las cajas basicas no son pseudo-bb y cl.
Tener todo en los atomos permitiria organizar las intercolisiones de bb
de la siguiente manera: Asociado a cada bb habria estructuras que nos dirian
que atomos estan a distancia 4 o mas del siguiente, cuales a distancia 3, a 2 
y a 1. Lo mismo para el anterior. (solo haria falta que estuvieran llenas
si forman parte de una posible intercolision. Se pueden rellenar a posteriori
recorriendo los rigidos). Los que esten a 4 o mas de un residuo se verificarian
con todos los del siguiente. Los que esten a 3, con los que estan a 1, a 2, y 3 
o mas. Etc.
- hacerlo todo en funcion de la distancia a los centros, y que se puedan devolver todas las
distancias entre 2 * Rmin y 2 * Rmax (donde Rmax es radio del atomo mas mas grande en el
contexto), y en este margen estaran todas las posibles colisiones.
(que habria que comprobar una a una)

Si necesitaramos hacer todos los atomos cuyas superficie este a una distancia menor que d,
cogemos todos los atomos cuya distancia este a menos de (2 * Rmax) + d.
Lueg habria que verificar una a una a que distancia esta realmente.

Si necesitamos el par cuya superficie este mas cercana, una range search, con rango variable.
Al principio sera infinito y cuando en contremos dos centros mas cercanos con distacia Di
el nuevo rango sera Di < Dmin, y tendremo que incluir Di en la lista si 
Di - 2 Rmax < Dmin - 2 Rmin, es decir si Di - Dmin < 2(Rmax - Rmin). 
Tambien cuando encontramos un Dmin nuevo a lo mejor vale la pena calcular la distancia
entre superficies DSmin. Entonces solo incluimos en la lista Di que cumplen
Di - 2 Rmax < DSmin, es decir Di - DSmin < 2 Rmax
Asi vamos obteniendo una lista de Di's y de pares 
de atomos. Podemos coger el ultimo y calcular la distancia entre sus superficies DSn, y se
miran algunos unos de los ultimos, los que cumplan que (Di - 2 Rmax) <  (Dn - 2 Rmin), 
es decir, Di - Dn < 2 (Rmax - Rmin). Pero modemos restringir mas aun la busqueda
(Di - 2 Rmax) < DSn, es decir Di - Dn < 2 Rmax
Por supuesto si hemos hallado DSmin, entonces quedan descartados los Di que sean menores
que Di - DSmin < 2 Rmax.
Ahora queda otro problema, y es que los Di no estan ordenados en la lista. Sin embargo
hay cierta ordenacion. Los Di se han ido agnadiendo en funcion de cual era el Dmin o
DSmin actual.Los DSmin o Dmin han ido decreciendo, y sabemos que para cada Dmin o DSmin
los Di estan dentro de un cierto margen. Es mas dado un Di en una posicion, podemos acotar
inferiormente el valor otros Di hallados con su mismo DSmin o Dmin, y por tanto esta 
es una cota para todos los Di que le preceden que se pueden excluir si la cota no nos
satisface.Otra posibilidad es tener indicadores de donde empieza cada Dmin o DSmin
nuevo en la lista y su valor, y acabamos un poco antes. Por otro lado, esto nos 
daria tambien una serie de agrupamientos de atomos por intervalos de distancia 
(aun que estos intervalos se intersectarian).Lo malo es que no se nos garantiza que estan
todos los atomos de un intervalo de distancia dado, porque paramos de meterlos cuando 
encontramos una distancia suficientemente pequegna. Olvidalo.
Tambien se podia haber hecho todo de modo mas sencillo y quiza no mucho menos eficiente
calculando cada vez que se necesite DSi en vez de Di.


EN EL CASO DE TODO LIBRE

- el create_rigid_root es ineficiente cuando hay una sola caja: contemplar el caso
-Usar una sola jerarquia. EL ANTERIOR YA NO ES NECESARIO

EN EL CASO DE QUE SOLO NOS INTERESE UN MODO

Sustituir los apuntadores de funciones por las funciones apropiadas

*/
