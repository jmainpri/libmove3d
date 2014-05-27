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
#include "Collision-pkg.h"

/** *********************************************************************** **  
 ** *********************************************************************** ** 
 **    Functions about vectors
 ** *********************************************************************** ** 
 ** *********************************************************************** **/

/* ************************************************************************* *  
 * Function:  kcd_vectCopy() 
 *            vector copy:  src => dest
 * ************************************************************************* */
void kcd_vectCopy(kcd_vector3 src, kcd_vector3 dest)
{
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
} /*  End of kcd_vectCopy()  */

/* ************************************************************************* *  
 * Function:  kcd_vectScale() 
 *            vector scaling: k * src => dest
 * ************************************************************************* */
void kcd_vectScale(kcd_vector3 src, kcd_vector3 dest, double k)
{
  dest[0] = k * src[0];
  dest[1] = k * src[1];
  dest[2] = k * src[2];
} /*  End of kcd_vectScale()  */

/* ************************************************************************* *  
 * Function:  kcd_vectAdd() 
 *            vector addition:  a + b => c
 * ************************************************************************* */
void kcd_vectAdd(kcd_vector3 a, kcd_vector3 b, kcd_vector3 c)
{
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
} /*  End of kcd_vectAdd()  */

/* ************************************************************************* *  
 * Function:  kcd_vectEqual() 
 *            Return true if the vectors are exactly equal, false otherwise
 * ************************************************************************* */
int kcd_vectEqual(kcd_vector3 a, kcd_vector3 b)
{
  return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
} /*  End of kcd_vectEqual()  */

/* ************************************************************************* *  
 * Function:  kcd_vectDotProd() 
 *            vector dot product:  return a . b
 * ************************************************************************* */
double kcd_vectDotProd(kcd_vector3 a, kcd_vector3 b)
{
  return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
} /*  End of kcd_vectDotProd()  */

/* ************************************************************************* *  
 * Function:  kcd_vectSub() 
 *            vector subtraction:  a - b => c
 * ************************************************************************* */
void kcd_vectSub(kcd_vector3 a, kcd_vector3 b, kcd_vector3 c)
{  
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
} /*  End of kcd_vectSub()  */

/* ************************************************************************* *  
 * Function:  kcd_vectNorm() 
 *            compute vector norm
 * ************************************************************************* */
double kcd_vectNorm(kcd_vector3 v)
{
  return sqrt((double) ( SQR(v[0]) + SQR(v[1]) + SQR(v[2]) ) );
} /*  End of kcd_vectNorm()  */

/* ************************************************************************* *  
 * Function:  kcd_vectXprod() 
 *            vector cross product:  a x b => c
 * REMARKS :  c should not point to the same vector as a or b!
 * ************************************************************************* */
void kcd_vectXprod(kcd_vector3 a, kcd_vector3 b, kcd_vector3 c)
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
} /*  End of kcd_vectXprod()  */







/** *********************************************************************** **  
 ** *********************************************************************** ** 
 **    Functions about matrices
 ** *********************************************************************** ** 
 ** *********************************************************************** **/


/* ************************************************************************* * 
 * Function :  kcd_mat4Mult() 
 *             matrix multiply:  a * b => c
 * REMARKS  :  c should not point to the same matrix as a or b!
 * ************************************************************************* */
/* inline void kcd_mat4Mult(kcd_matrix4 a, kcd_matrix4 b, kcd_matrix4 c) */
void kcd_mat4Mult(kcd_matrix4 a, kcd_matrix4 b, kcd_matrix4 c)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      c[i][j] = a[i][0] * b[0][j] + 
                a[i][1] * b[1][j] + 
                a[i][2] * b[2][j] + 
                a[i][3] * b[3][j];
} /*  End of kcd_mat4Mult()  */


/* ************************************************************************* * 
 * Function :  kcd_matInvertTransfo  
 *             transformation matrix inversion:  Inverse(M) => inv
 * REMARKS  :  M and inv should not point to the same matrix.  We assume M is
 *             a transformation matrix; this will not properly invert
 * 	       arbitrary 4x4 matrices.
 * 	       Doesn't work for transformations with scaling
 * ************************************************************************* */
void kcd_matInvertTransfo(kcd_matrix4 M, kcd_matrix4 inv)
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
} /*  End of kcd_matInvertTransfo()  */

/* ************************************************************************* * 
 * Function :  kcd_matMultTransfo()  
 *             transformation matrix multiply:  a * b => c
 * REMARKS  :  This routine is much faster than the general 4 x 4 matrix
 *             multiply above, but only works properly if a and b are
 *	       SE(3) transformation matrices.  c should not point to the
 *	       same matrix as a or b! 
 * ************************************************************************* */
void kcd_matMultTransfo(kcd_matrix4 a, kcd_matrix4 b, kcd_matrix4 c)
{
  /* Rc = Ra Rb */
  c[0][0] = a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
  c[0][1] = a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
  c[0][2] = a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];
  c[1][0] = a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
  c[1][1] = a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1];
  c[1][2] = a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];
  c[2][0] = a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
  c[2][1] = a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
  c[2][2] = a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];

  /* Vc = Ra Vb + Va */
  c[0][3] = a[0][0]*b[0][3] + a[0][1]*b[1][3] + a[0][2]*b[2][3] + a[0][3];
  c[1][3] = a[1][0]*b[0][3] + a[1][1]*b[1][3] + a[1][2]*b[2][3] + a[1][3];
  c[2][3] = a[2][0]*b[0][3] + a[2][1]*b[1][3] + a[2][2]*b[2][3] + a[2][3];

  /* the rest */
  c[3][0] = c[3][1] = c[3][2] = 0.0;
  c[3][3] = 1.0;
} /*  End of kcd_matMultTransfo()  */

/* ************************************************************************* * 
 * Function:  kcd_mat3Copy()  
 *            matrix copy:  source => dest
 * ************************************************************************* */
void kcd_mat3Copy(kcd_matrix3 source, kcd_matrix3 dest)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) dest[i][j] = source[i][j];
} /*  End of kcd_mat3Copy()  */

/* ************************************************************************* */ 
/*! 
 \brief       matrix4 copy:  source => dest
*/
/* ************************************************************************* */
void kcd_mat4Copy(kcd_matrix3 source, kcd_matrix3 dest)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) dest[i][j] = source[i][j];
} /*  End of kcd_mat4Copy()  */

/* ************************************************************************* * 
 * Function:  kcd_mat3Add()  
 *            matrix addition:  a + b => c
 * REMARKS :  c can point to the same matrix as a or b
 * ************************************************************************* */
void kcd_mat3Add(kcd_matrix3 a, kcd_matrix3 b, kcd_matrix3 c)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) c[i][j] = a[i][j] + b[i][j];
} /*  End of kcd_mat3Add()  */

/* ************************************************************************* * 
 * Function:  kcd_mat3Transpose()  
 *            matrix transpose:  (source)^T => dest
 * REMARKS :  source and dest must be distinct!    
 * ************************************************************************* */
void kcd_mat3Transpose(kcd_matrix3 source, kcd_matrix3 dest)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) dest[i][j] = source[j][i];
} /*  End of kcd_mat3Transpose()  */






/** *********************************************************************** **  
 ** *********************************************************************** ** 
 **    Functions about matrices and vectors
 ** *********************************************************************** ** 
 ** *********************************************************************** **/


/* ************************************************************************* *  
 * Function:  kcd_Transfo3() 
 *            transform a 3-vector: M * x => x2
 *            This routine does a full transformation of a vector x in R^3.
 * ************************************************************************* */
void kcd_Transfo3(kcd_matrix3 M, kcd_vector3 x, kcd_vector3 x2)
{
  int i;

  for (i = 0; i < 3; i++) 
    x2[i] = M[i][0] * x[0] + M[i][1] * x[1] + M[i][2] * x[2];
} /*  End of kcd_Transfo3  */

/* ************************************************************************* *  
 * Function:  kcd_TransfoVect() 
 *            transform a vector:  M * (v 0) => (v2 0)
 * ************************************************************************* */
void kcd_TransfoVect(kcd_matrix4 M, kcd_vector3 v, kcd_vector3 v2)
{
  int i;
  
  for (i = 0; i < 3; i++) 
    v2[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
} /*  End of kcd_TransfoVect()  */
 
/* ************************************************************************* *  
 * Function:  kcd_TransfoPoint() 
 *            transform a point:  M * (p 1) => (p2 1)
 * ************************************************************************* */
void kcd_TransfoPoint(kcd_matrix4 M, kcd_vector3 p, kcd_vector3 p2)
{
  int i;

  for (i = 0; i < 3; i++) 
    p2[i] = M[i][0] * p[0] + M[i][1] * p[1] + M[i][2] * p[2] + M[i][3];
} /*  End of kcd_TransfoPoint()  */

