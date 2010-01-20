/*******************************************************/
/***************** bcd_hierarchies.c *******************/
/*******************************************************/
/* functions that:
- iniatiate and create the two hierachies of BCD
- check the interactions at atom and the hierarchy levels in all the 
BCD modes
- free the hierarchies
 */
/* ***************************************************** */
#include <math.h>
#include <float.h>
#include "include/bcd_global.h"

#define MIN_IN_HIGH_HIERCHY 5
/* This is the minimum number of elments that must be contained in one node of
 the high hierarchy to be (eventually) splitted. Nodes with less elements are
 always leaves */


void (*box_collision)(Bboxpt, Bboxpt);
 /* pointer to a function
to deal with the collision of the leaves of the low hierarchy (collision of 
boxes). The processing depends on whether the two boxes belongs to a same 
peptide chain or not */

int charlatan=0;

/**********************************************************************************
VARIABLES FOR COLLISION REPORTING
**********************************************************************************/
#define MAX_NUMBER_OF_COLLISIONS 1000 /* It must be allways greater than 50 */

static int n_collisions; /* number of pairs satisfying the query */
static int required_collisions= 1; /* BCD stops its calculus when at least required_collisions pairs
 are found (can be more) */
/* (collisions1[i], collisions2[i])
  is a pair of pointers to p3d_poly satisfying the operating mode conditions (up to i < n_collisions). */
static p3d_poly *collisions1[MAX_NUMBER_OF_COLLISIONS]; 
static p3d_poly *collisions2[MAX_NUMBER_OF_COLLISIONS];

// mod noure
static double  distance[MAX_NUMBER_OF_COLLISIONS];
//fmod noure

static p3d_poly *closest_atom1,*closest_atom2;

static int biocol_mode;

/*trash*/
int giveme_mininhierarchy(void){
return MIN_IN_HIGH_HIERCHY;
}


/* 
says if two p3d_poly of a non-polypeptide molecule are
are a valid pair with allowed collision testing
Assumes that the autocol_data list in an atom(p3d_poly) a1 has only atoms
belonging to rigids with a higher index than that to which a1 belongs.
*/
int valid_pair(p3d_poly *poly1, p3d_poly *poly2){
p3d_poly *poly;
autocol_struct *auto1= poly1->autocol_data,*auto2= poly2->autocol_data, *low=NULL;
int j= -1;
if (auto1){
	if (auto2){
		if ((auto1->nrigid < auto2->nrigid)){
			j= 0;
			low= auto1;
			poly= poly2;
			while ((j < low->size) && (low->list[j]!=poly)) j++;
			}
		else{
			j= 0;
			low= auto2;
			poly= poly1;
			while ((j < low->size) && (low->list[j]!=poly)) j++;
			}
		}
	else {
		j= 0;
		low= auto1;
		poly= poly2;
		while ((j < low->size) && (low->list[j]!=poly)) j++;
		}
	}
else if (auto2){
	j= 0;
	low= auto2;
	poly= poly1;
	while ((j < low->size) && (low->list[j]!=poly)) j++;
	}						 
if ((j == -1) || (j == low->size)) return 1;
			else return 0;
	}

/*                                                 */
/* NORMAL_BIOCOL_MODE: Pairs of atoms in collision */
/*                                                 */


/* Checks if the atoma1 and atom a2 collide 
static void atom_collision(p3d_poly **la1, int n1, p3d_poly **la2, int n2){
p3d_poly *a1= la1[n1],*a2= la2[n2];
register double *v1= a1->pos;
register double *v2= a2->pos;
register double f;
if ( ((f= *(v1) - *(v2)) * f) + ((f= *(++v1) - *(++v2)) * f) + 
        ((f= *(++v1) - *(++v2)) * f) <
                                    (f= a1->r + a2->r) * f ){
	collisions1[n_collisions]= a1;
	collisions2[n_collisions]= a2;
	n_collisions++;	
	if (charlatan){
		printf("%s %d choca con ", a1->poly->name);
		printf("%s \n", a2->poly->name);
		} 
	}
}

*/

#ifdef HYDROGEN_BOND
#define SQR_SUM_SIMPLE (f= a1->radii[a2->type]) * f
#define SQR_SUM_DIST (f= a1->radii[a2->type] + surface_d) * f
#define SQR_SUM_MINDIST (f= (r= a1->radii[a2->type]) + minimum_Sdistance) * f
#else
#define SQR_SUM_SIMPLE (f= a1->r + a2->r) * f
#define SQR_SUM_DIST (f= a1->r + a2->r + surface_d) * f
#define SQR_SUM_MINDIST (f= (r= a1->r + a2->r) + minimum_Sdistance) * f
#endif

#define SQR_DIST (*(v1) - *(v2)) * (*(v1) - *(v2)) + (*(v1+1) - *(v2+1)) * (*(v1+1) - *(v2+1)) + (*(v1+2) - *(v2+2)) * (*(v1+2) - *(v2+2))

/*
( ((f= *(v1) - *(v2)) * f) + ((f= *(++v1) - *(++v2)) * f) + 
        ((f= *(++v1) - *(++v2)) * f) <
                                    (f= b1->radius[i] + b2->radius[j] + surface_d) * f )
									
( ((f= *(v1) - *(v2)) * f) + ((f= *(++v1) - *(++v2)) * f) + 
        ((f= *(++v1) - *(++v2)) * f) <
                                    (f= *radiusi + *(radiusj++) + surface_d) * f )

( (d = ((f= *(v1) - *(v2)) * f) + ((f= *(++v1) - *(++v2)) * f) + 
        ((f= *(++v1) - *(++v2)) * f) ) <
                                    (f= (r= b1->radius[i] + b2->radius[j]) + minimum_Sdistance) * f )
*/									
/* Checks if the atoma1 and atom a2 collide */
static void atom_collision(p3d_poly *a1,p3d_poly *a2){
register double *v1= a1->pos;
register double *v2= a2->pos;
register double f;
if ( SQR_DIST < SQR_SUM_SIMPLE ){
	collisions1[n_collisions]= a1;
	collisions2[n_collisions]= a2;
	n_collisions++;	
/*	if (charlatan){
		printf("%s choca con ", a1->poly->name);
		printf("%s \n", a2->poly->name);
		} 
		*/
	}
}

/* It checks if the first n1 atoms in la1
collide with the first n2 atoms in la2 */
static int atom_collisions(p3d_poly **la1,int n1,p3d_poly **la2, int n2){
double  *v1_copy;
p3d_poly **aux_la2;
register p3d_poly *a1,*a2;
register double f, *v1, *v2;

int index_i, index_j;
for (index_i= 0;index_i < n1;index_i++){
	aux_la2= la2;
	v1_copy= (a1= *(la1++))->pos;
	for (index_j= 0;index_j < n2;index_j++){		
		v2=(a2=*(aux_la2++))->pos;
		v1=v1_copy;
		if ( SQR_DIST < SQR_SUM_SIMPLE ){
			collisions1[n_collisions]= a1;
			collisions2[n_collisions]= a2;
			n_collisions++;	
/*			if (charlatan){
				printf("%s choca con ", a1->poly->name);
				printf("%s \n", a2->poly->name);
				}
				*/
			}
		}
	}

 return 0;
}

/* It checks if the bounding boxes defined by (min1 , max1)  and (min2 , max2)
do not touch (do not intersect)*/
short int donatouch_basic(double *min1, double *min2, double *max1, double *max2){

if (*min1 < *min2){
   if (*min2 > *max1) return 1;
   }
else if (*min1 > *max2) return 1;

max1++;max2++;

if (*(++min1) < *(++min2)){
   if (*min2 > *max1) return 1;
   }
else if (*min1 > *max2) return 1;

max1++;max2++;

if (*(++min1) < *(++min2)){
   if (*min2 > *max1) return 1;
   }
else if (*min1 > *max2) return 1;
return 0; /* the bounding boxes intersect */
}

/****************  						***************/
/****************  	OTHER MODES				***************/
/****************  						***************/





/*                                                 */
/* SURFACE_DISTANCE_BIOCOL_MODE:  Pairs of atoms whose Vand der Waal 
surfaces are at a distance minor than a desired one.*/
/*                                                 */

static double surface_d; /* allowed distance between the surfaces of the atoms */

void bio_set_surface_d(double distance){
surface_d= distance;
}
/* Checks if the atoma1 and atom a2 are at suface distance greater than surface_d*/
static void pair_at_sd(p3d_poly *a1,p3d_poly *a2){
register double *v1= a1->pos;
register double *v2= a2->pos;
register double f,squaredistance;
//if ( SQR_DIST < SQR_SUM_DIST ){
squaredistance= SQR_DIST;
if ( ( squaredistance < SQR_SUM_DIST ) && (a1->type != HYDROGEN) && (a2->type != HYDROGEN)){
	collisions1[n_collisions]= a1;
	collisions2[n_collisions]= a2;
	distance[n_collisions]= sqrt(squaredistance);
	n_collisions++;		
/*	if (charlatan){
		printf("%s choca con ", a1->poly->name);
		printf("%s \n", a2->poly->name);
		} */
	}
}
/* It checks if the first n1 atoms in la1
are at surface distance minor than surface_d from the first n2 atoms in la2 */
static int pairs_at_sd(p3d_poly **la1,int n1,p3d_poly **la2, int n2){
double  *v1_copy;
p3d_poly **aux_la2;
register p3d_poly *a1,*a2;
register double f, *v1, *v2, squaredistance;

int index_i, index_j;
for (index_i= 0;index_i < n1;index_i++){
	aux_la2= la2;
	v1_copy= (a1= *(la1++))->pos;
	for (index_j= 0;index_j < n2;index_j++){		
		v2=(a2=*(aux_la2++))->pos;
		v1=v1_copy;
//		if (  SQR_DIST < SQR_SUM_DIST ){
		squaredistance= SQR_DIST;
		if ( ( squaredistance < SQR_SUM_DIST ) && (a1->type != HYDROGEN) && (a2->type != HYDROGEN)){
			collisions1[n_collisions]= a1;
			collisions2[n_collisions]= a2;
			//mod noureddine
			distance[n_collisions]= sqrt(squaredistance);
			//fmod noureddine
			n_collisions++;	
/*			if (charlatan){
				printf("%s choca con ", a1->poly->name);
				printf("%s \n", a2->poly->name);
				} */
			}
		}
	}

 return 0;
}


/* It checks if the bounding boxes defined by (min1 , max1)  and (min2 , max2)
are at a distance greater than surface_d*/
short int enough_Surf_Dist(double *min1, double *min2, double *max1, double *max2){

if (*min1 < *min2){
   if (*min2 > (*max1 + surface_d)) return 1;
   }
else if (*min1 > (*max2 + surface_d)) return 1;

max1++;max2++;

if (*(++min1) < *(++min2)){
   if (*min2 > (*max1 + surface_d)) return 1;
   }
else if (*min1 > (*max2 + surface_d)) return 1;

max1++;max2++;

if (*(++min1) < *(++min2)){
   if (*min2 > (*max1 + surface_d)) return 1;
   }
else if (*min1 > (*max2 + surface_d)) return 1;
return 0; /* the bounding boxes are at distance minor than surface_d */
}

/* minimum distance by now between the surfaces of two atoms */

/*                                                 */
/* MINIMUM_DISTANCE_BIOCOL_MODE: The pair of atom whose Vand der Waal 
surfaces are at a distance minor than any else pair..*/
/*                                                 */
double minimum_Sdistance; 

void set_minimum_Sdistance_to_max(void){
minimum_Sdistance= DBL_MAX;
}

/*
double get_minimum_Sdistance_to_max(void){
return minimum_Sdistance;
}
*/

/* Checks if the atoma1 and atom a2 are at suface distance greater than minimum_Sdistance*/
static void pair_at_min_sd(p3d_poly *a1,p3d_poly *a2){
register double *v1= a1->pos;
register double *v2= a2->pos;
register double d,r,f;
if ( (d= SQR_DIST) < SQR_SUM_MINDIST ){
	closest_atom1= a1;
	closest_atom2= a2;
	minimum_Sdistance= sqrt(d) - r;
	}
}

/* It checks if the bounding boxes defined by (min1 , max1)  and (min2 , max2)
are at a distance greater than minimum_Sdistance*/
static int pairs_at_min_sd(p3d_poly **la1,int n1,p3d_poly **la2, int n2){
double  *v1_copy;
p3d_poly **aux_la2;
register p3d_poly *a1,*a2;
register double d,r,f, *v1, *v2;

int index_i, index_j;
for (index_i= 0;index_i < n1;index_i++){
	aux_la2= la2;
	v1_copy= (a1= *(la1++))->pos;
	for (index_j= 0;index_j < n2;index_j++){		
		v2=(a2=*(aux_la2++))->pos;
		v1=v1_copy;
		if ( (d= SQR_DIST) < SQR_SUM_MINDIST ){
			closest_atom1= a1;
			closest_atom2= a2;
			minimum_Sdistance= sqrt(d) - r;
			}
		}
	} 

 return 0;
}

/* same as pair_at_min_sd for non peptides */
static void pair_at_min_sd_nonpep(p3d_poly *a1,p3d_poly *a2){
register double *v1= a1->pos;
register double *v2= a2->pos;
register double d,r,f;
if ( (d= SQR_DIST) < SQR_SUM_MINDIST ){
	if (valid_pair(a1,a2)){
		closest_atom1= a1;
		closest_atom2= a2;
		minimum_Sdistance= sqrt(d) - r;
		}

	}
}

/* same as pairs_at_min_sd for non peptides */
static int pairs_at_min_sd_nonpep(p3d_poly **la1,int n1,p3d_poly **la2, int n2){
double  *v1_copy;
p3d_poly **aux_la2;
register p3d_poly *a1,*a2;
register double d,r,f, *v1, *v2;

int index_i, index_j;
for (index_i= 0;index_i < n1;index_i++){
	aux_la2= la2;
	v1_copy= (a1= *(la1++))->pos;
	for (index_j= 0;index_j < n2;index_j++){		
		v2=(a2=*(aux_la2++))->pos;
		v1=v1_copy;
		if ( (d= SQR_DIST) < SQR_SUM_MINDIST ){
			if (valid_pair(a1,a2)){
				closest_atom1= a1;
				closest_atom2= a2;
				minimum_Sdistance= sqrt(d) - r;
				}
			}
		}
	} 

 return 0;
}

/* It checks if the bounding boxes defined by (min1 , max1)  and (min2 , max2)
are at a distance greater than minimum_Sdistance*/
short int enough_minS_Dist(double *min1, double *min2, double *max1, double *max2){

if (*min1 < *min2){
   if (*min2 > (*max1 + minimum_Sdistance)) return 1;
   }
else if (*min1 > (*max2 + minimum_Sdistance)) return 1;

max1++;max2++;

if (*(++min1) < *(++min2)){
   if (*min2 > (*max1 + minimum_Sdistance)) return 1;
   }
else if (*min1 > (*max2 + minimum_Sdistance)) return 1;

max1++;max2++;

if (*(++min1) < *(++min2)){
   if (*min2 > (*max1 + minimum_Sdistance)) return 1;
   }
else if (*min1 > (*max2 + minimum_Sdistance)) return 1;
return 0; /* the bounding boxes are at distance minor than minimum_Sdistance */
}



check_pair_function check_pair;
check_all_function check_all;
static donatouch_function donatouch;

static check_pair_function check_pair_functions[3]={atom_collision, pair_at_sd,pair_at_min_sd};
static check_all_function check_all_functions[3]={atom_collisions, pairs_at_sd,pairs_at_min_sd};
static donatouch_function donatouch_functions[3]={donatouch_basic,enough_Surf_Dist,enough_minS_Dist};

/****************  END MODES						***************/

void bio_set_col_mode(int mode){
biocol_mode= mode;
check_all= check_all_functions[mode];
check_pair= check_pair_functions[mode];
donatouch= donatouch_functions[mode];
}

int bio_get_col_mode(void){
return biocol_mode;
}




/*checking the collision of two boxes within the same protein */
void box_coll_withinProt(Bboxpt b1, Bboxpt b2){

int diff;
if ((diff= fabs((float) (b1->namino - b2->namino))) < 2){
    if (!(diff)) return;
    else if (b1->bb && b2->bb) return;
    }
if (b1->loop)
    if (b1->loop == b2->namino) return;

/* the easy way of doing check_all
	{
	int i,j,n1=b1->natoms,n2=b2->natoms;    
	for (i=0; i < n1;i++)
		for (j=0; j < n2;j++)	
			compare(b1, i, b2, j);
	} 
*/
(*check_all) (b1->lpoly, b1->natoms, b2->lpoly, b2->natoms);
}

/*checking the collision of two boxes from different proteines or
from at least a non-poypeptide */
void box_coll_general(Bboxpt b1, Bboxpt b2){
(*check_all) (b1->lpoly, b1->natoms, b2->lpoly, b2->natoms);
}



static boxnodept boxnode_allocate(void){
return (boxnode *) malloc(sizeof(boxnode));
}

void imprimir(boxnodept node){
int i;	
Bboxpt *boxptpt=node->lbox;
Bboxpt boxpt;
//printf("cajas constituyentes nodo (%d):\n", node->name);
for(i=0; i < node->nbox; i++){
     boxpt=*boxptpt;
printf(" %d ",boxpt->namino);
     boxptpt++;
    }
printf("\n");
for(i=0,boxptpt=node->lbox; i < node->nbox; i++){
     boxpt=*boxptpt;
printf("%2.2f\t%2.2f\t%2.2f\t\t%2.2f\t%2.2f\t%2.2f\n",
boxpt->min[0],boxpt->min[1],boxpt->min[2],boxpt->max[0],boxpt->max[1],boxpt->max[2]);
     boxptpt++;
    }
printf("envolvente: ");
printf("%2.2f\t%2.2f\t%2.2f\t\t%2.2f\t%2.2f\t%2.2f\n",
node->min[0],node->min[1],node->min[2],node->max[0],node->max[1],node->max[2]);
}

/* This is not very efficient when there are many rigids containing only one
 box.This is the case for non-polypeptydes right now. An specific function 
 should be written if lage or very articulated non polypeptides are used */
boxnodept create_rigid_root(Rigid_structure *rigido){
Bboxpt *lbox;
Bboxpt bbox;
p3d_poly **lpoly;
p3d_poly *polygordo;
int j,k,natoms;
register double aux,radio, *vec;
double (*mat)[4];
register double localminx,localmaxx,localminy,localmaxy,localminz,localmaxz;
double rigidminx,rigidmaxx,rigidminy,rigidmaxy,rigidminz,rigidmaxz;
boxnodept node=boxnode_allocate();
rigidminx=rigidminy=rigidminz=9000000000.0;
rigidmaxx=rigidmaxy=rigidmaxz=-9000000000.0;


lbox= rigido->lbox;
for (j=0;j < rigido->nboxes;j++){
    bbox= *(lbox++);
    lpoly= bbox->lpoly;
    natoms= bbox->natoms;
	localminx=localminy=localminz=9000000000.0;
	localmaxx=localmaxy=localmaxz=-9000000000.0;
    for (k=0;k < natoms;k++){
        polygordo= *(lpoly++);
#ifdef HYDROGEN_BOND
		radio= *(polygordo->r);
#else
        radio= polygordo->r;
#endif
        vec=  polygordo->pos;
        mat=polygordo->matpos;
        if ((aux=(*vec= mat[0][3])- radio) < localminx){
            localminx= aux;
            if (localminx < rigidminx) rigidminx = localminx;
            }
        if ((aux = *vec + radio) > localmaxx){
            localmaxx= aux;
            if (localmaxx > rigidmaxx) rigidmaxx = localmaxx;
            }
        if ((aux=(*(++vec)= mat[1][3])- radio) < localminy){
            localminy= aux;
            if (localminy < rigidminy) rigidminy = localminy;
            }
        if ((aux = *vec + radio)> localmaxy){
            localmaxy= aux;
            if (localmaxy > rigidmaxy) rigidmaxy = localmaxy;
            }
        if ((aux=(*(++vec)= mat[2][3])- radio) < localminz){
            localminz= aux;
            if (localminz < rigidminz) rigidminz = localminz;
            }
        if ((aux = *vec + radio) > localmaxz){
            localmaxz= aux;
            if (localmaxz > rigidmaxz) rigidmaxz = localmaxz;
            }
        }
	/* assigning min and max vectors to each bbox */
    vec=bbox->min;*(vec++)=localminx;*(vec++)=localminy;*vec=localminz;
    vec=bbox->max;*(vec++)=localmaxx;*(vec++)=localmaxy;*vec=localmaxz;
    }
/* assigning min and max vectors to the rigid */
vec=node->min;*(vec++)=rigidminx;*(vec++)=rigidminy;*vec=rigidminz;
vec=node->max;*(vec++)=rigidmaxx;*(vec++)=rigidmaxy;*vec=rigidmaxz;

node->left=NULL;
node->right=NULL;
node->nbox= rigido->nboxes;
node->lbox= rigido->lbox;
node->leaf=1;
node->level=0;
//node->name=nodename++;
//imprimir(node);
#ifdef SUPRESSION
rigido->root= node;
#endif
return node;    
}

/*
if (((*vec= mat[0][3]) - radio) < localminx){
            localminx= x - radio;
            if (localminx < rigidminx) rigidminx = localminx;
            }
            

        x= *vec= mat[0][3];
        if ((aux= x - radio) < localminx){
            localminx= aux;
            if (localminx < rigidminx) rigidminx = localminx;
            }
        if ((aux= x + radio) > localmaxx){
            localmaxx= aux;;
            if (localmaxx > rigidmaxx) rigidmaxx = localmaxx;
            }
        y= *(++vec)= mat[1][3];
        if ((aux= y - radio) < localminy){
            localminy= aux;
            if (localminy < rigidminy) rigidminy = localminy;
            }
        if ((aux= y + radio) > localmaxy){
            localmaxy= aux;
            if (localmaxy > rigidmaxy) rigidmaxy = localmaxy;
            }
        z= *(++vec)= mat[2][3];
        if ((aux= z - radio) < localminz){
            localminz= aux;
            if (localminz < rigidminz) rigidminz = localminz;
            }
        if ((aux= z + radio) > localmaxz){
            localmaxz= aux;
            if (localmaxz > rigidmaxz) rigidmaxz = localmaxz;
            }
*/


static void split(boxnodept node){
int i,j,splitdim=0, nleft=0, nbox=node->nbox;
Bboxpt *boxptpt= node->lbox;
Bboxpt boxpt;

Bboxpt *leftlbox=(Bboxpt *) malloc(nbox * sizeof(Bboxpt));//cautious memory allocation
Bboxpt *rightlbox=(Bboxpt *) malloc(nbox * sizeof(Bboxpt));//cautious memory allocation
Bboxpt *leftlist=leftlbox; // goes through leftlbox
Bboxpt *rightlist=rightlbox; // goes through rightlbox
boxnodept nodeaux;
double wideness,halfmaxwide, midpoint,maxwide=-1.0;
double max,min;
/* bounding boxes for the sons */
double lmaxx, lmaxy, lmaxz, lminx, lminy,lminz, rmaxx,rmaxy,rmaxz,rminx,rminy,rminz;
double *minvec=node->min;
double *maxvec=node->max;

lmaxx=lmaxy=lmaxz=rmaxx=rmaxy=rmaxz= -10000000.0;
lminx=lminy=lminz=rminx=rminy=rminz= 10000000.0;
for (i=0;i<3;i++,minvec++, maxvec++){
    if ((wideness=(*maxvec - *minvec)) > maxwide){
        maxwide=wideness;
        splitdim=i;
        }
    }
halfmaxwide=maxwide/2.0;
midpoint= node->min[splitdim] + halfmaxwide;
for (i=0;i< nbox;i++){ 
	
    boxpt=*(boxptpt++);
    minvec=boxpt->min;maxvec=boxpt->max;
    max= maxvec[splitdim];min= minvec[splitdim];
    if ((max- min) > halfmaxwide) break;
    if ((min + max)/2 < midpoint){
		*(leftlist++)= boxpt;
		nleft++;
		if (*minvec < lminx) lminx= *minvec; 
		if (*(++minvec) < lminy) lminy= *minvec;
		if (*(++minvec) < lminz) lminz= *minvec;
				
		if (*maxvec > lmaxx) lmaxx= *maxvec;		
		if (*(++maxvec) > lmaxy) lmaxy=*maxvec;
		if (*(++maxvec) > lmaxz) lmaxz= *maxvec;

        }
    else{
		*(rightlist++)= boxpt;
		if (*minvec < rminx) rminx= *minvec; 
		if (*(++minvec) < rminy) rminy= *minvec;
		if (*(++minvec) < rminz) rminz= *minvec;
				
		if (*maxvec > rmaxx) rmaxx= *maxvec; 
		if (*(++maxvec) > rmaxy) rmaxy=*maxvec;
		if (*(++maxvec) > rmaxz) rmaxz= *maxvec;

        }
    }
if (i< nbox){
    *leftlbox=boxpt;
    nleft=1;
	minvec=boxpt->min;maxvec=boxpt->max;
    lminx=*minvec; lminy=*(++minvec);lminz=*(++minvec); lmaxx=*maxvec;lmaxy=*(++maxvec);lmaxz=*(++maxvec);
    boxptpt= node->lbox;
    rightlist=rightlbox;
    for (j=0,boxptpt= node->lbox;j< nbox;j++,boxptpt++){ 
        if (i!=j){
			boxpt=*boxptpt;
			minvec=boxpt->min;maxvec=boxpt->max;
			if (*minvec < rminx) rminx= *minvec;
			if (*(++minvec) < rminy) rminy= *minvec;
			if (*(++minvec) < rminz) rminz= *minvec;
			if (*maxvec > rmaxx) rmaxx= *maxvec;
			if (*(++maxvec) > rmaxy) rmaxy=*maxvec;
			if (*(++maxvec) > rmaxz) rmaxz= *maxvec;
			*(rightlist++)= boxpt;

            }
        }
}


//printf(" HIJOS DE(%d) ---------------:\n", node->name);
if (nleft > 0){
    nodeaux=node->left= boxnode_allocate();
    nodeaux->nbox= nleft;
    minvec=nodeaux->min;maxvec=nodeaux->max;
    *minvec= lminx;*(++minvec)= lminy;*(++minvec)= lminz;*maxvec= lmaxx;*(++maxvec)= lmaxy;*(++maxvec)= lmaxz;
    nodeaux->lbox= (Bboxpt *) realloc((void *) leftlbox, (sizeof(Bboxpt) * nleft)); //freeing unuseed pointers
    nodeaux->left=NULL;
    nodeaux->right=NULL;
//    nodeaux->name=nodename++;
    nodeaux->leaf=1;
    nodeaux->level=node->level + 1;
 //   putchar(nodeaux->name);printf(" %d (LEFT) ", nodeaux->level);
//   imprimir(nodeaux);
    }
if (nbox - nleft > 0){
    nodeaux=node->right= boxnode_allocate();
    nodeaux->nbox= nbox - nleft;
    minvec=nodeaux->min;maxvec=nodeaux->max;
    *minvec= rminx;*(++minvec)= rminy;*(++minvec)= rminz;*maxvec= rmaxx;*(++maxvec)= rmaxy;*(++maxvec)= rmaxz;
    nodeaux->lbox= (Bboxpt *) 
			realloc((void *) rightlbox, (sizeof(Bboxpt) * nodeaux->nbox)); //freeing unuseed pointers 
    nodeaux->left=NULL;
    nodeaux->right=NULL;
//    nodeaux->name=nodename++;
		nodeaux->leaf=1;
    nodeaux->level=node->level + 1;
//    putchar(nodeaux->name);printf(" %d (RIGHT) ", nodeaux->level);
  // imprimir(nodeaux);
    }
if (node->level > 0) free(node->lbox);
node->leaf=0;
}


static void hierarchy1(boxnodept node){
    split(node);
    if ((node->left) && (node->left->nbox > 1)) hierarchy1(node->left);
    if ((node->right) && (node->right->nbox > 1)) hierarchy1(node->right);
    }
	
	
/* static void hierarchy(boxnodept node){ */
/* 		if (node->nbox >1) hierarchy1(node); */
/* } */



/*
short int donatouch(boxnodept node1, boxnodept node2){
register double *min1=node1->min;
register double *max1=node1->max;
register double *min2=node2->min;
register double *max2=node2->max;
int i=0,chocan=1;
while ((chocan) && (i < 3)){
    if (*min1 < *min2){
        if (*min2 > *max1) chocan=0;
        }
    else if (*min1 > *max2) chocan=0;
    min1++;min2++;max1++;max2++;i++;
    }
return !chocan;
}
*/
/*
short int donatouch(boxnodept node1, boxnodept node2){
register double *min1=node1->min;
register double *max1=node1->max;
register double *min2=node2->min;
register double *max2=node2->max;

if (*min1 < *min2){
   if (*min2 > *max1) return 1;
   }
else if (*min1 > *max2) return 1;

min1++;min2++;max1++;max2++;

if (*min1 < *min2){
   if (*min2 > *max1) return 1;
   }
else if (*min1 > *max2) return 1;

min1++;min2++;max1++;max2++;

if (*min1 < *min2){
   if (*min2 > *max1) return 1;
   }
else if (*min1 > *max2) return 1;
return 0;
}
*/
static void collision(boxnodept node1, boxnodept node2){
boxnodept large, small;
if ((*donatouch)(node1->min, node2->min, node1->max,node2->max)) return;

if ((node1->nbox == 1) && (node2->nbox == 1)){
            (*box_collision)(*(node1->lbox),*(node2->lbox));
            return;
            }            
if ( node1->nbox > node2->nbox ){
    large= node1;
    small = node2;
    }
else{
    small= node1;
    large = node2;
    }
if (large->leaf) split(large);
collision(large->left, small);
if (n_collisions >= required_collisions) return;
collision(large->right, small);

}


/* This function assumes that if the node is not a leaf,the two sons of a node 
are not NULL. The original hierarchy has this property. If you change something
that could alter this property, you should use the comented function */
static void freeboxhierarchy1(boxnodept node){
	if (!(node->leaf)){
		freeboxhierarchy1(node->left);
		freeboxhierarchy1(node->right);
		}
	else if (node->level > 0) free(node->lbox);
	free(node);
}

/*
static void freeboxhierarchy1(boxnodept node){
	if (node->left) freeboxhierarchy1(node->left);
	if (node->right) freeboxhierarchy1(node->right);
	if ((node->leaf) && (node->level > 0))  free(node->lbox);
	free(node);

}

*/

#ifdef SUPRESSION
/* like freeboxhierarchy, used when SUPRESSION */
void freeboxhierarchy2(Rigid_structure *r){
if (r->root) freeboxhierarchy1(r->root);
r->root=NULL;
}

#else

void freeboxhierarchy(Robot_structure *robot, int i){
if (robot->roots[i]) freeboxhierarchy1(robot->roots[i]);
robot->roots[i]=NULL;
}
#endif


void bio_create_molecule_root(Robot_structure *robot){
boxnodept *lnode;
boxnodept infnode;
#ifdef SUPRESSION
int j, nrigids= robot->n_active_rigids;
#else
int j, nrigids= robot->nrigids;
#endif
register double *maxvec, *minvec, aux;

double robotminx,robotmaxx,robotminy,robotmaxy,robotminz,robotmaxz;
supnodept node= (supnodept) malloc(sizeof(supnode));

//node->lnode= (boxnodept) malloc(nrigids * sizeof(boxnodept));
robotminx=robotminy=robotminz=9000000000.0;
robotmaxx=robotmaxy=robotmaxz=-9000000000.0;

lnode= robot->roots;
for (j=0;j < nrigids;j++,lnode++){
    infnode= *lnode;
    minvec= infnode->min;
    maxvec= infnode->max;
    if ((aux = *(minvec++)) < robotminx) robotminx = aux;
    if ((aux = *(minvec++)) < robotminy) robotminy = aux;
    if ((aux = *(minvec)) < robotminz) robotminz = aux;
    if ((aux = *(maxvec++)) > robotmaxx) robotmaxx = aux;
    if ((aux = *(maxvec++)) > robotmaxy) robotmaxy = aux;
    if ((aux = *(maxvec)) > robotmaxz) robotmaxz = aux;
    }
	
minvec=node->min;*(minvec++)=robotminx;*(minvec++)=robotminy;*minvec=robotminz;
maxvec=node->max;*(maxvec++)=robotmaxx;*(maxvec++)=robotmaxy;*maxvec=robotmaxz;

node->left=NULL;
node->right=NULL;
node->nnodes= nrigids;

node->lnode= robot->roots;

node->leaf=1;
node->level=0;

//node->name=nodename++;
node->leftbrother= NULL;

robot->superroot=node;
//return node;   
}


void sup_split(supnodept node){
int i,j,splitdim=0, nleft=0, nnodes=node->nnodes;
boxnodept *lnode= node->lnode;
boxnodept infnode;

boxnodept *leftlbox=(boxnodept *) malloc(nnodes * sizeof(boxnodept));//cautious memory allocation
boxnodept *rightlbox=(boxnodept *) malloc(nnodes * sizeof(boxnodept));//cautious memory allocation

boxnodept *leftlist=leftlbox; // goes through leftlbox
boxnodept *rightlist=rightlbox; // goes through rightlbox
supnodept nodeaux;
double wideness,halfmaxwide, midpoint,maxwide=-1.0;
double max,min;
/* bounding boxes for the sons */
double lmaxx, lmaxy, lmaxz, lminx, lminy,lminz, rmaxx,rmaxy,rmaxz,rminx,rminy,rminz;
double *minvec=node->min;
double *maxvec=node->max;

//printf(" hago un SPLIT ");
lmaxx=lmaxy=lmaxz=rmaxx=rmaxy=rmaxz= -10000000.0;
lminx=lminy=lminz=rminx=rminy=rminz= 10000000.0;
for (i=0;i<3;i++,minvec++, maxvec++){
    if ((wideness=(*maxvec - *minvec)) > maxwide){
        maxwide=wideness;
        splitdim=i;
        }
    }
halfmaxwide=maxwide/2.0;
midpoint= node->min[splitdim] + halfmaxwide;

for (i=0;i< nnodes;i++){ 
	
    infnode=*(lnode++);
    minvec=infnode->min;maxvec=infnode->max;
    max= maxvec[splitdim];min= minvec[splitdim];
    if ((max- min) > halfmaxwide) break;
    if ((min + max)/2 < midpoint){
        *(leftlist++)= infnode;
        nleft++;
        if (*minvec < lminx) lminx= *minvec; 
        if (*(++minvec) < lminy) lminy= *minvec;
        if (*(++minvec) < lminz) lminz= *minvec;
        if (*maxvec > lmaxx) lmaxx= *maxvec;
        if (*(++maxvec) > lmaxy) lmaxy=*maxvec;
        if (*(++maxvec) > lmaxz) lmaxz= *maxvec;
        }
    else{
        *(rightlist++)= infnode;
        if (*minvec < rminx) rminx= *minvec;
        if (*(++minvec) < rminy) rminy= *minvec;
        if (*(++minvec) < rminz) rminz= *minvec;
        
        if (*maxvec > rmaxx) rmaxx= *maxvec;
        if (*(++maxvec) > rmaxy) rmaxy=*maxvec;
        if (*(++maxvec) > rmaxz) rmaxz= *maxvec;

        }
    }
if (i< nnodes){
    *leftlbox=infnode;
    nleft=1;
		minvec=infnode->min;maxvec=infnode->max;
    lminx=*minvec; lminy=*(++minvec);lminz=*(++minvec); lmaxx=*maxvec;lmaxy=*(++maxvec);lmaxz=*(++maxvec);
    rightlist=rightlbox;
    for (j=0,lnode= node->lnode;j< nnodes;j++,lnode++){ 
        if (i!=j){
            infnode=*lnode;
            minvec=infnode->min;maxvec=infnode->max;
            if (*minvec < rminx) rminx= *minvec;
            if (*(++minvec) < rminy) rminy= *minvec;
            if (*(++minvec) < rminz) rminz= *minvec;
            if (*maxvec > rmaxx) rmaxx= *maxvec;
            if (*(++maxvec) > rmaxy) rmaxy=*maxvec;
            if (*(++maxvec) > rmaxz) rmaxz= *maxvec;
            *(rightlist++)= infnode;
            }
        }
}
if (nleft > 0){
    nodeaux=node->left= (supnodept) malloc(sizeof(supnode));
    nodeaux->nnodes= nleft;
    minvec=nodeaux->min;maxvec=nodeaux->max;
    *minvec= lminx;*(++minvec)= lminy;*(++minvec)= lminz;*maxvec= lmaxx;*(++maxvec)= lmaxy;*(++maxvec)= lmaxz;
    nodeaux->lnode= (boxnodept *) 
        realloc((void *) leftlbox, (sizeof(boxnodept) * nleft)); //freeing unuseed pointers
    nodeaux->left=NULL;
    nodeaux->right=NULL;
//    nodeaux->name=nodename++;
    nodeaux->leaf=1;
    nodeaux->level=node->level + 1;
    nodeaux->leftbrother= NULL;
 //   putchar(nodeaux->name);printf(" %d (LEFT) ", nodeaux->level);
//    imprimir(nodeaux);
    }
if (nnodes - nleft > 0){
    nodeaux=node->right= (supnodept) malloc(sizeof(supnode));
    nodeaux->nnodes= nnodes - nleft;
    minvec=nodeaux->min;maxvec=nodeaux->max;
    *minvec= rminx;*(++minvec)= rminy;*(++minvec)= rminz;*maxvec= rmaxx;*(++maxvec)= rmaxy;*(++maxvec)= rmaxz;
    nodeaux->lnode= (boxnodept *) 
			realloc((void *) rightlbox, (sizeof(boxnodept) * nodeaux->nnodes)); //freeing unused pointers 
    nodeaux->left=NULL;
    nodeaux->right=NULL;
//    nodeaux->name=nodename++;
    nodeaux->leaf=1;
    nodeaux->level=node->level + 1;
    nodeaux->leftbrother= node->left;
//    putchar(nodeaux->name);printf(" %d (RIGHT) ", nodeaux->level);
 //   imprimir(nodeaux);
    }
if (node->level > 0) free(node->lnode);
node->leaf=0;
}


void sup_hierarchy1(supnodept node){
    sup_split(node);
    if ((node->left) && (node->left->nnodes >= MIN_IN_HIGH_HIERCHY)) sup_hierarchy1(node->left);
    if ((node->right) && (node->right->nnodes >= MIN_IN_HIGH_HIERCHY)) sup_hierarchy1(node->right);
    }
void sup_hierarchy(supnodept node){
    if  (node->nnodes >= MIN_IN_HIGH_HIERCHY) sup_hierarchy1(node);
    }


void leaf_sup_collision(supnodept node1, supnodept node2){
int i,j,nnodes1= node1->nnodes;
boxnodept *lnode1;
boxnodept *lnode2;
if (node1 == node2){
	//printf("ncajas= %d ", nnodes1);
	for (i=0,lnode1= node1->lnode;i < nnodes1; i++,lnode1++)
		for (j=i + 1, lnode2= lnode1 + 1;j < nnodes1; j++){
			if (n_collisions >= required_collisions) return;
			collision(*lnode1,*(lnode2++));
			}
	}
else{
	int nnodes2= node2->nnodes;
	//printf("ncajas1= %d ncajas2= %d ", nnodes1, nnodes2);
	for (i=0,lnode1= node1->lnode;i < nnodes1; i++,lnode1++)
		for (j=0, lnode2= node2->lnode;j < nnodes2; j++){
			if (n_collisions >= required_collisions) return;
			collision(*lnode1,*(lnode2++));
			}
	}
}


void sup_collision(supnodept node1, supnodept node2){
supnodept large, small;
if (node1 == node2){
	if (node1->nnodes < MIN_IN_HIGH_HIERCHY){
				leaf_sup_collision(node1,node2);
				return;
				}   
	if (node1->leaf) sup_split(node1);
	sup_collision(node1->left, node1->left);
	if (n_collisions >= required_collisions) return;
	sup_collision(node1->left, node1->right);
	if (n_collisions >= required_collisions) return;
	sup_collision(node1->right, node1->right);
	}
else{
	if ((*donatouch)(node1->min, node2->min, node1->max,node2->max)) return;

	if ((node1->nnodes < MIN_IN_HIGH_HIERCHY) && (node2->nnodes < MIN_IN_HIGH_HIERCHY)){
				leaf_sup_collision(node1,node2);
				return;
				}            
	if ( node1->nnodes > node2->nnodes ){
		large= node1;
		small = node2;
		}
	else{
		small= node1;
		large = node2;
		}
	if (large->leaf) sup_split(large);
	sup_collision(large->left, small);
	if (n_collisions >= required_collisions) return;
	sup_collision(large->right, small);
	}

}


/* This function assumes that if the node is not a leaf,the two sons of a node 
are not NULL. The original hierarchy has this property. If you change something
that could alter this property, you should use the comented function */
static void freesuphierarchy1(supnodept node){
	if (!(node->leaf)){
		 freesuphierarchy1(node->left);
		 freesuphierarchy1(node->right);
		 }
	else if (node->level > 0)  free(node->lnode);
	free(node);
	
}

void freesuphierarchy(Robot_structure *robot){
if (robot->superroot) freesuphierarchy1(robot->superroot);
robot->superroot=NULL;
}
/*
static void freesuphierarchy1(boxnodept node){
	if (node->left) freesuphierarchy1(node->left);
	if (node->right) freesuphierarchy1(node->right);
	if ((node->leaf) && (node->level > 0))  free(node->lbox);
	free(node);

}

*/



/* It can be made somewhat more efficient */
void my_robot_autocollision(Robot_structure *robot){
p3d_poly *poly1, *poly2;
if (n_collisions >= required_collisions) return;
if (robot->polypep){
    box_collision= box_coll_withinProt;
    sup_collision(robot->superroot, robot->superroot);
    }
else{
	int i,ncolbefore,ncolafter, nrealcol=0;
	box_collision= box_coll_general;
	ncolbefore= n_collisions;
	if (biocol_mode==MINIMUM_DISTANCE_BIOCOL_MODE){
		 check_all= pairs_at_min_sd_nonpep;
		 check_pair= pair_at_min_sd_nonpep;
		 }
	sup_collision(robot->superroot, robot->superroot);
	ncolafter= n_collisions;
	for (i= ncolbefore;i < ncolafter; i++){
            poly1=collisions1[i];
            poly2=collisions2[i];					 
            if (valid_pair(poly1,poly2)){
                collisions1[ncolbefore + nrealcol]=  poly1;
                collisions2[ncolbefore + nrealcol]=  poly2;
                nrealcol++;
                }
            }
	n_collisions= ncolbefore + nrealcol;
  }
if (biocol_mode==MINIMUM_DISTANCE_BIOCOL_MODE) bio_set_col_mode(MINIMUM_DISTANCE_BIOCOL_MODE);
}	



/* The minimum distance pair obtained by MINIMUM_DISTANCE_BIOCOL_MODE */ 
double min_dist_report(p3d_poly **poly1, p3d_poly **poly2){
 *poly1= closest_atom1;
 *poly2= closest_atom2;
 return minimum_Sdistance;
}

/*
The main reporting function in the first two operating modes .
col_number is the number of pairs founds and 
list1 and list2 are tables such that (list1[i], list2[i]) is 
a pair satisfying the operating mode conditions.
*/
void biocol_report(int *col_number, p3d_poly ***list1, p3d_poly ***list2)
{
  *col_number= n_collisions;
  *list1=collisions1;
  *list2=collisions2;
}

//mod noureddine **************************
void biocol_report_dist(int *atom_number, p3d_poly ***list1, p3d_poly ***list2, double **dist)
{
  *atom_number= n_collisions;
  *list1=collisions1;
  *list2=collisions2;
  *dist = distance;
}
//fmod noureddine **************************


void set_required_collisions(int rc){
  if ((MAX_NUMBER_OF_COLLISIONS - 50) > rc) required_collisions=rc;
  else printf("required__collisions is too high. MAX_NUMBER_OF_COLLISIONS must be resized to enable such required__collisions\n");
}

void set_required_collisions_to_max(void){
int rc= MAX_NUMBER_OF_COLLISIONS - 50;
if (rc > 0) required_collisions=rc;
else printf("The constant MAX_NUMBER_OF_COLLISIONS is too low. It must be greater than 50.\n");
}


void set_n_collisions_to_zero(void){
 n_collisions= 0;
 }

int get_n_collisions(void){
 return n_collisions;
 }

 
int too_much_collisions(void){
return n_collisions >= required_collisions;
}
