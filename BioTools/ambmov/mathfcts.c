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
#include <math.h>
#include "defs.h"
#include "mathfcts.h"

/**********************************************************************/

void vectSub(double *a, double *b, double *c)
{  
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
}


double vectNorm(double *v)
{
  return sqrt((double) ( SQR(v[0]) + SQR(v[1]) + SQR(v[2]) ) );
}


void vectNormalize(double *src, double *dest)
{
  double l;

  l = vectNorm(src);
  dest[0] = src[0] / l;
  dest[1] = src[1] / l;
  dest[2] = src[2] / l;
}


void normalized_vectXprod(double *a, double *b, double *c)
{
  double nnc[3];

  nnc[0] = a[1] * b[2] - a[2] * b[1];
  nnc[1] = a[2] * b[0] - a[0] * b[2];
  nnc[2] = a[0] * b[1] - a[1] * b[0];

  vectNormalize(nnc,c);
}


double vectDotProd(double *a, double *b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


int same_sign_vect(double *v1, double *v2)
{
  double v1n[3],v2n[3];

  vectNormalize(v1,v1n);
  vectNormalize(v2,v2n);
  if(vectDotProd(v1n,v2n) > 0.0)
    return 1;
  else
    return 0;
}


void mat4vec3MultPos(matrix4 a, double *v, vector4 c)
{
  int i;

  for (i = 0; i < 3; i++)
      c[i] = a[i][0] * v[0] + 
             a[i][1] * v[1] +
             a[i][2] * v[2] + 
             a[i][3] * 1.0;
}


void mat4Mult(matrix4 a, matrix4 b, matrix4 c)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      c[i][j] = a[i][0] * b[0][j] + 
                a[i][1] * b[1][j] + 
                a[i][2] * b[2][j] + 
                a[i][3] * b[3][j];
}


void mat4Copy(matrix4 source, matrix4 dest)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) dest[i][j] = source[i][j];
}


void inverse_transf(matrix4 M, matrix4 inv)
{
  /* we invert the rotation part by transposing it */
  inv[0][0] = M[0][0];
  inv[0][1] = M[1][0];
  inv[0][2] = M[2][0];
  inv[1][0] = M[0][1];
  inv[1][1] = M[1][1];
  inv[1][2] = M[2][1];
  inv[2][0] = M[0][2];
  inv[2][1] = M[1][2];
  inv[2][2] = M[2][2];

  /* the new displacement vector is given by:  d' = -(R^-1) * d */
  inv[0][3] = - inv[0][0]*M[0][3] - inv[0][1]*M[1][3] - inv[0][2]*M[2][3];
  inv[1][3] = - inv[1][0]*M[0][3] - inv[1][1]*M[1][3] - inv[1][2]*M[2][3];
  inv[2][3] = - inv[2][0]*M[0][3] - inv[2][1]*M[1][3] - inv[2][2]*M[2][3];

  /* the rest stays the same */
  inv[3][0] = inv[3][1] = inv[3][2] = 0.0;
  inv[3][3] = 1.0;
}


void transf_rotz(double q, matrix4 T)
{
  T[0][0]=cos(q); T[0][1]=-sin(q); T[0][2]=0.0; T[0][3]=0.0; 
  T[1][0]=sin(q); T[1][1]= cos(q); T[1][2]=0.0; T[1][3]=0.0; 
  T[2][0]=   0.0; T[2][1]=    0.0; T[2][2]=1.0; T[2][3]=0.0; 
  T[3][0]=   0.0; T[3][1]=    0.0; T[3][2]=0.0; T[3][3]=1.0; 
}

void inv_transf_rotz(double q, matrix4 T)
{
  T[0][0]= cos(q); T[0][1]=sin(q); T[0][2]=0.0; T[0][3]=0.0; 
  T[1][0]=-sin(q); T[1][1]=cos(q); T[1][2]=0.0; T[1][3]=0.0; 
  T[2][0]=    0.0; T[2][1]=   0.0; T[2][2]=1.0; T[2][3]=0.0; 
  T[3][0]=    0.0; T[3][1]=   0.0; T[3][2]=0.0; T[3][3]=1.0; 
}


/**********************************************************************/

double compute_dihedang(double *nJa, double *tJa, double *pJa)
{
  // WARNING : vectors must be normalized !
  double axesXprod1[3],axesXprod2[3];
  double dirprod[3];
  double dihedang;

  normalized_vectXprod(pJa,tJa,axesXprod1);
  normalized_vectXprod(tJa,nJa,axesXprod2);
  dihedang = acos(vectDotProd(axesXprod1,axesXprod2));
  normalized_vectXprod(axesXprod1,axesXprod2,dirprod);
  if(same_sign_vect(tJa,dirprod)) 
    return (dihedang);
  else
    return (-dihedang);
}


void compute_frame(double *opos, double *xaxis, double *zaxis, matrix4 T)
{
  double yaxis[3];

  normalized_vectXprod(zaxis,xaxis,yaxis);

  T[0][0] = xaxis[0]; T[0][1] = yaxis[0]; T[0][2] = zaxis[0]; T[0][3] = opos[0]; 
  T[1][0] = xaxis[1]; T[1][1] = yaxis[1]; T[1][2] = zaxis[1]; T[1][3] = opos[1]; 
  T[2][0] = xaxis[2]; T[2][1] = yaxis[2]; T[2][2] = zaxis[2]; T[2][3] = opos[2]; 
  T[3][0] = 0.0;      T[3][1] = 0.0;      T[3][2] = 0.0     ; T[3][3] = 1.0; 
}


void compute_dihedang_and_frame(double *a1pos, double *a2pos,double *a3pos, double *a4pos,
				double *dihedang, matrix4 T)
{
  double posdiff[3],zaxis[3],prevzaxis[3],nextzaxis[3],xaxis[3];

  vectSub(a2pos,a1pos,posdiff);
  vectNormalize(posdiff,prevzaxis);
  vectSub(a3pos,a2pos,posdiff);
  vectNormalize(posdiff,zaxis);
  vectSub(a4pos,a3pos,posdiff);
  vectNormalize(posdiff,nextzaxis);  
  normalized_vectXprod(zaxis,nextzaxis,xaxis);

  *dihedang = compute_dihedang(nextzaxis,zaxis,prevzaxis);

  compute_frame(a3pos,xaxis,zaxis,T);
}

void compute_distance(double *P1, double *P2, double *dist)
{
  double vdiff[3];

  vectSub(P1,P2,vdiff);
  *dist = vectNorm(vdiff);
}

