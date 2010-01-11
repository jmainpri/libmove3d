

	/*******************************************************
        *                                                      *
	*  volInt.c                                            *
	*                                                      *
	*  This code computes volume integrals needed for      *
	*  determining mass properties of polyhedral bodies.   *
	*                                                      *
	*  For more information, see the accompanying README   *
	*  file, and the paper                                 *
	*                                                      *
	*  Brian Mirtich, "Fast and Accurate Computation of    *
	*  Polyhedral Mass Properties," journal of graphics    *
	*  tools, volume 1, number 1, 1996.                    *
	*                                                      *
	*  This source code is public domain, and may be used  *
	*  in any way, shape or form, free of charge.          *
	*                                                      *
	*  Copyright 1995 by Brian Mirtich                     *
	*                                                      *
	*  mirtich@cs.berkeley.edu                             *
	*  http://www.cs.berkeley.edu/~mirtich                 *
        *                                                      *
	*******************************************************/

/*
	Revision history

	26 Jan 1996	Program creation.

	 3 Aug 1996	Corrected bug arising when polyhedron density
			is not 1.0.  Changes confined to function main().
			Thanks to Zoran Popovic for catching this one.

	27 May 1997     Corrected sign error in translation of inertia
	                product terms to center of mass frame.  Changes 
			confined to function main().  Thanks to 
			Chris Hecker.


        Modification JP Saut
        Avril 2008      ajout d'une structure et fusion des trois fonctions initiales 
                        pour n'avoir à utiliser que des variables locales.
                        ajout d'une fonction pour passer du format p3d_polyhedre à VOLINT_POLYHEDRON
*/
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "GraspPlanning-pkg.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

const int X = 0;
const int Y = 1;
const int Z = 2;

// Recopie les sommets et les faces d'une structure poly_polyhedre dans une structure VOLINT_POLYHEDRON.
// Note: dans p3d_polyhedre, les indices des sommets contenus dans les faces commencent à 1 alors
// qu'ils commencent à 0 dans VOLINT_POLYHEDRON.
void readPolyhedron(p3d_polyhedre *poly, VOLINT_POLYHEDRON *p)
{
  #ifdef DEBUG 
   if( poly==NULL || p==NULL )
   {
     printf("%s: %d: readPolyhedron(): entrée(s) vide(s)\n",__FILE__, __LINE__);
     exit(0);
   }
   if(poly->nb_points > MAX_VERTS)
   {
     printf("%s: %d: readPolyhedron(): l'objet (poly_polyhedre) a trop de sommets (%d au lieu de %d): augmentez la valeur de MAX_VERTS dans volInt.h\n", __FILE__, __LINE__, poly->nb_points, MAX_VERTS);
     exit(0);
   }
   if(poly->nb_faces > MAX_FACES)
   {
     printf("%s: %d: readPolyhedron(): l'objet (poly_polyhedre) a trop de faces (%d au lieu de %d): augmentez la valeur de MAX_FACES dans volInt.h\n", __FILE__, __LINE__, poly->nb_faces, MAX_FACES);
     exit(0);
   }
  #endif

  int i, j;
  double dx1, dy1, dz1, dx2, dy2, dz2, nx, ny, nz, len;
  VOLINT_FACE *f;
  poly_index *indices;


  p->numVerts= poly->nb_points;

  for (i = 0; i < p->numVerts; i++)
  {
      p->verts[i][X]=  poly->the_points[i][0];
      p->verts[i][Y]=  poly->the_points[i][1];
      p->verts[i][Z]=  poly->the_points[i][2];
  }

 
  p->numFaces=  poly->nb_faces;

  for (i = 0; i < p->numFaces; i++)
  {
    f = &(p->faces[i]);
    f->poly = p;
    
    indices= poly->the_faces[i].the_indexs_points;

    f->numVerts= poly->the_faces[i].nb_points;
    for (j = 0; j < f->numVerts; j++) 
    {
    	f->verts[j]= indices[j]-1;
    }

    /* compute face normal and offset w from first 3 vertices */
    dx1 = p->verts[f->verts[1]][X] - p->verts[f->verts[0]][X];
    dy1 = p->verts[f->verts[1]][Y] - p->verts[f->verts[0]][Y];
    dz1 = p->verts[f->verts[1]][Z] - p->verts[f->verts[0]][Z];
    dx2 = p->verts[f->verts[2]][X] - p->verts[f->verts[1]][X];
    dy2 = p->verts[f->verts[2]][Y] - p->verts[f->verts[1]][Y];
    dz2 = p->verts[f->verts[2]][Z] - p->verts[f->verts[1]][Z];
    nx = dy1 * dz2 - dy2 * dz1;
    ny = dz1 * dx2 - dz2 * dx1;
    nz = dx1 * dy2 - dx2 * dy1;
    len = sqrt(nx * nx + ny * ny + nz * nz);
    f->norm[X] = nx / len;
    f->norm[Y] = ny / len;
    f->norm[Z] = nz / len;
    f->w = - f->norm[X] * p->verts[f->verts[0]][X]
           - f->norm[Y] * p->verts[f->verts[0]][Y]
           - f->norm[Z] * p->verts[f->verts[0]][Z];

  }

 

}


/*
   ============================================================================
   read in a polyhedron
   ============================================================================
*/
// Fonction d'origine:
/*
void readPolyhedron(char *name, VOLINT_POLYHEDRON *p)
{
  FILE *fp;
  char line[200], *c;
  int i, j, n;
  double dx1, dy1, dz1, dx2, dy2, dz2, nx, ny, nz, len;
  VOLINT_FACE *f;

  
  if (!(fp = fopen(name, "r"))) {
    printf("i/o error\n");
    exit(1);
  }
  
  fscanf(fp, "%d", &p->numVerts);
  printf("Reading in %d vertices\n", p->numVerts);
  for (i = 0; i < p->numVerts; i++)
    fscanf(fp, "%lf %lf %lf", 
	   &p->verts[i][X], &p->verts[i][Y], &p->verts[i][Z]);

  fscanf(fp, "%d", &p->numFaces);
  printf("Reading in %d faces\n", p->numFaces);
  for (i = 0; i < p->numFaces; i++) {
    f = &p->faces[i];
    f->poly = p;
    fscanf(fp, "%d", &f->numVerts);
    for (j = 0; j < f->numVerts; j++) fscanf(fp, "%d", &f->verts[j]);

    // compute face normal and offset w from first 3 vertices 
    dx1 = p->verts[f->verts[1]][X] - p->verts[f->verts[0]][X];
    dy1 = p->verts[f->verts[1]][Y] - p->verts[f->verts[0]][Y];
    dz1 = p->verts[f->verts[1]][Z] - p->verts[f->verts[0]][Z];
    dx2 = p->verts[f->verts[2]][X] - p->verts[f->verts[1]][X];
    dy2 = p->verts[f->verts[2]][Y] - p->verts[f->verts[1]][Y];
    dz2 = p->verts[f->verts[2]][Z] - p->verts[f->verts[1]][Z];
    nx = dy1 * dz2 - dy2 * dz1;
    ny = dz1 * dx2 - dz2 * dx1;
    nz = dx1 * dy2 - dx2 * dy1;
    len = sqrt(nx * nx + ny * ny + nz * nz);
    f->norm[X] = nx / len;
    f->norm[Y] = ny / len;
    f->norm[Z] = nz / len;
    f->w = - f->norm[X] * p->verts[f->verts[0]][X]
           - f->norm[Y] * p->verts[f->verts[0]][Y]
           - f->norm[Z] * p->verts[f->verts[0]][Z];

  }

  fclose(fp);

}
*/

void compMassProperties(VOLINT_POLYHEDRON *p, Mass_properties *mass_prop)
{
  VOLINT_FACE *f;
  double nx, ny, nz;
  int i, j;
  double *n, w;
  double k1, k2, k3, k4;
  double a0, a1, da;
  double b0, b1, db;
  double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
  double a1_2, a1_3, b1_2, b1_3;
  double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
  double Cab, Kab, Caab, Kaab, Cabb, Kabb;

  int A;   /* alpha */
  int B;   /* beta */
  int C;   /* gamma */

  /* projection integrals */
  double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

  /* face integrals */
  double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

  /* volume integrals */
  double T0, T1[3], T2[3], TP[3];

  T0 = T1[X] = T1[Y] = T1[Z] 
     = T2[X] = T2[Y] = T2[Z] 
     = TP[X] = TP[Y] = TP[Z] = 0;


  for (i = 0; i < p->numFaces; i++)
  {

    f = &p->faces[i];

    nx = fabs(f->norm[X]);
    ny = fabs(f->norm[Y]);
    nz = fabs(f->norm[Z]);
    if (nx > ny && nx > nz) C = X;
    else C = (ny > nz) ? Y : Z;
    A = (C + 1) % 3;
    B = (A + 1) % 3;

   /* compute various integrations over projection of face */
   /*****************************************************/
   /************face integral computation****************/


        /*****************************************************/
        /**********projection integral computation************/

           P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

           for (j = 0; j < f->numVerts; j++)
           {
              a0 = f->poly->verts[f->verts[j]][A];
              b0 = f->poly->verts[f->verts[j]][B];
              a1 = f->poly->verts[f->verts[(j+1) % f->numVerts]][A];
              b1 = f->poly->verts[f->verts[(j+1) % f->numVerts]][B];
              da = a1 - a0;
              db = b1 - b0;
              a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
              b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
              a1_2 = a1 * a1; a1_3 = a1_2 * a1; 
              b1_2 = b1 * b1; b1_3 = b1_2 * b1;

              C1 = a1 + a0;
              Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
              Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
              Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
              Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
              Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
              Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

              P1 += db*C1;
              Pa += db*Ca;
              Paa += db*Caa;
              Paaa += db*Caaa;
              Pb += da*Cb;
              Pbb += da*Cbb;
              Pbbb += da*Cbbb;
              Pab += db*(b1*Cab + b0*Kab);
              Paab += db*(b1*Caab + b0*Kaab);
              Pabb += da*(a1*Cabb + a0*Kabb);
           }

           P1 /= 2.0;
           Pa /= 6.0;
           Paa /= 12.0;
           Paaa /= 20.0;
           Pb /= -6.0;
           Pbb /= -12.0;
           Pbbb /= -20.0;
           Pab /= 24.0;
           Paab /= 60.0;
           Pabb /= -60.0;
        /*******end of projection integral computation*********/
        /*****************************************************/


      w = f->w;
      n = f->norm;
      k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

      Fa = k1 * Pa;
      Fb = k1 * Pb;
      Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

      Faa = k1 * Paa;
      Fbb = k1 * Pbb;
      Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb
	     + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

      Faaa = k1 * Paaa;
      Fbbb = k1 * Pbbb;
      Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab 
	       + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
	       + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
  	       + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

      Faab = k1 * Paab;
      Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
      Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
	     + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));


   /**********end of face integral computation************/
   /*****************************************************/

   

    T0 += f->norm[X] * ((A == X) ? Fa : ((B == X) ? Fb : Fc));

    T1[A] += f->norm[A] * Faa;
    T1[B] += f->norm[B] * Fbb;
    T1[C] += f->norm[C] * Fcc;
    T2[A] += f->norm[A] * Faaa;
    T2[B] += f->norm[B] * Fbbb;
    T2[C] += f->norm[C] * Fccc;
    TP[A] += f->norm[A] * Faab;
    TP[B] += f->norm[B] * Fbbc;
    TP[C] += f->norm[C] * Fcca;
  }

  T1[X] /= 2; T1[Y] /= 2; T1[Z] /= 2;
  T2[X] /= 3; T2[Y] /= 3; T2[Z] /= 3;
  TP[X] /= 2; TP[Y] /= 2; TP[Z] /= 2;




  double density = 1.0;  /* assume unit density */
  double mass= density * T0;
  double r[3];            /* center of mass */
  double J[3][3];         /* inertia tensor */

  /* compute center of mass */
  r[X] = T1[X] / T0;
  r[Y] = T1[Y] / T0;
  r[Z] = T1[Z] / T0;

  /* compute inertia tensor */
  J[X][X] = density * (T2[Y] + T2[Z]);
  J[Y][Y] = density * (T2[Z] + T2[X]);
  J[Z][Z] = density * (T2[X] + T2[Y]);
  J[X][Y] = J[Y][X] = - density * TP[X];
  J[Y][Z] = J[Z][Y] = - density * TP[Y];
  J[Z][X] = J[X][Z] = - density * TP[Z];

  /* translate inertia tensor to center of mass */
  J[X][X] -= mass * (r[Y]*r[Y] + r[Z]*r[Z]);
  J[Y][Y] -= mass * (r[Z]*r[Z] + r[X]*r[X]);
  J[Z][Z] -= mass * (r[X]*r[X] + r[Y]*r[Y]);
  J[X][Y] = J[Y][X] += mass * r[X] * r[Y]; 
  J[Y][Z] = J[Z][Y] += mass * r[Y] * r[Z]; 
  J[Z][X] = J[X][Z] += mass * r[Z] * r[X]; 


  mass_prop->mass  = mass;
  mass_prop->volume= T0;
  mass_prop->density= density;
  mass_prop->r[X] = r[X];
  mass_prop->r[Y] = r[Y];
  mass_prop->r[Z] = r[Z];

  mass_prop->J[X][X] = J[X][X];
  mass_prop->J[Y][Y] = J[Y][Y];
  mass_prop->J[Z][Z] = J[Z][Z];
  mass_prop->J[X][Y] = mass_prop->J[Y][X] = J[X][Y]; 
  mass_prop->J[Y][Z] = mass_prop->J[Z][Y] = J[Y][Z]; 
  mass_prop->J[Z][X] = mass_prop->J[X][Z] = J[Z][X]; 


}

void gpCompute_mass_properties(p3d_polyhedre *poly, Mass_properties *mass_prop)
{ 
   VOLINT_POLYHEDRON p;
   readPolyhedron(poly, &p);
   compMassProperties(&p, mass_prop);
}


