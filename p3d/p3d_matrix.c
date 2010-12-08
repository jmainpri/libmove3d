#include "P3d-pkg.h"

//#ifdef P3D_PLANNER
//#include "Planner-pkg.h" //for p3d_random() function
//#endif

#include "Graphic-pkg.h" //for GLfloat type

/*****************************************************************************\
  matrix.c
  --
  Description :

      This is a collection of routines for manipulating matrices and vectors.

\*****************************************************************************/


/*----------------------------- Local Includes -----------------------------*/

//#include <stdio.h>
//#include <math.h>
//#include <p3d_matrix.h>

/*----------------------------- Local Constants -----------------------------*/

#ifndef M_PI
#  define M_PI		3.14159265358979323846
#endif

/*------------------------------ Local Macros -------------------------------*/

/*------------------------------- Local Types -------------------------------*/


/*------------------------ Local Function Prototypes ------------------------*/


/*------------------------------ Local Globals ------------------------------*/

/* identity matrix */

p3d_matrix4 p3d_mat4IDENTITY =
                {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
p3d_matrix3 p3d_mat3IDENTITY =
                {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
p3d_matrix4 p3d_mat4NULL =
                {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
p3d_matrix3 p3d_mat3NULL =
                {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};


/*---------------------------------Functions-------------------------------- */

/*
  =============================================================================

  matrix stuff

  The following operations all deal with transformation matrices, 4 x 4
  matrices corresponding to rigid body transformations of homogeneous
  coordinates.

  =============================================================================
*/



/*****************************************************************************\
 @ p3d_mat4Add()
 -----------------------------------------------------------------------------
 description : matrix add:  a + b => c
 input       :
 output      :
 notes       : c can point to the same matrix as a or b
\*****************************************************************************/
void p3d_mat4Add(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) c[i][j] = a[i][j] + b[i][j];
} /* End of p3d_mat4Add() **/

/*****************************************************************************\
 @ p3d_mat4Sub()
 -----------------------------------------------------------------------------
 description : matrix subtract:  a - b => c
 input       :
 output      :
 notes       : c can point to the same matrix as a or b
\*****************************************************************************/
void p3d_mat4Sub(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) c[i][j] = a[i][j] - b[i][j];
} /* End of p3d_mat4Sub() **/


/*****************************************************************************\
 @ p3d_mat3Sub()
 -----------------------------------------------------------------------------
 description : matrix subtract:  a - b => c
 input       :
 output      :
 notes       : c can point to the same matrix as a or b
\*****************************************************************************/
void p3d_mat3Sub(p3d_matrix3 a, p3d_matrix3 b, p3d_matrix3 c)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) c[i][j] = a[i][j] - b[i][j];
} /* End of p3d_mat3Sub() **/


/*****************************************************************************\
 @ p3d_mat3Add()
 -----------------------------------------------------------------------------
 description : matrix addition:  a + b => c
 input       :
 output      :
 notes       : c can point to the same matrix as a or b
\*****************************************************************************/
void p3d_mat3Add(p3d_matrix3 a, p3d_matrix3 b, p3d_matrix3 c)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) c[i][j] = a[i][j] + b[i][j];
} /* End of p3d_mat3Add() **/

/*****************************************************************************\
 @ p3d_mat4Mult()
 -----------------------------------------------------------------------------
 description : matrix multiply:  a * b => c
 input       :
 output      :
 notes       : c should not point to the same matrix as a or b!
\*****************************************************************************/
void p3d_mat4Mult(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      c[i][j] = a[i][0] * b[0][j] +
                a[i][1] * b[1][j] +
                a[i][2] * b[2][j] +
                a[i][3] * b[3][j];
} /* End of p3d_mat4Mult() **/

/*****************************************************************************\
 @ p3d_matvec4Mult()
 -----------------------------------------------------------------------------
 description : matrix vector multiply:  a * v => c
 input       :
 output      :
 notes       : c should not point to the same vector as v!
\*****************************************************************************/
void p3d_matvec4Mult(p3d_matrix4 a, p3d_vector4 v, p3d_vector4 c)
{
  int i;

  for (i = 0; i < 4; i++)
      c[i] = a[i][0] * v[0] +
             a[i][1] * v[1] +
             a[i][2] * v[2] +
             a[i][3] * v[3];
} /* End of p3d_matvec4Mult() **/

/*****************************************************************************\
 @ p3d_vec3Mat4Mult()
 -----------------------------------------------------------------------------
 description : matrix multiplied by vector :  a * b => c
 input       :
 output      :
 notes       : c should not point to the same matrix as a or b!
\*****************************************************************************/
void p3d_vec3Mat4Mult(double a[3], p3d_matrix4 b, p3d_matrix_type c[3])
{
  int i, j;
  double c3;

  for (j = 0; j < 3; j++)
    c[j] = a[0] * b[j][0] +
      a[1] * b[j][1] +
      a[2] * b[j][2] +
      b[j][3];
  c3 = a[0] * b[3][0] +
      a[1] * b[3][1] +
      a[2] * b[3][2] +
      b[3][3];

  for(i=0;i<3;i++)
    c[i]=c[i]/c3;
} /* End of p3d_mat4Mult() **/



/*****************************************************************************\
 @ p3d_mat3Mult()
 -----------------------------------------------------------------------------
 description : matrix multiply:  a * b => c
 input       :
 output      :
 notes       : c should not point to the same matrix as a or b!
\*****************************************************************************/
void p3d_mat3Mult(p3d_matrix3 a, p3d_matrix3 b, p3d_matrix3 c)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      c[i][j] = a[i][0] * b[0][j] +
                a[i][1] * b[1][j] +
                a[i][2] * b[2][j];
} /* End of p3d_mat3Mult() **/


/*****************************************************************************\
 @ p3d_mat4Copy()
 -----------------------------------------------------------------------------
 description : matrix copy:  source => dest
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_mat4Copy(p3d_matrix4 source, p3d_matrix4 dest)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) dest[i][j] = source[i][j];
} /* End of p3d_mat4Copy() **/

/*****************************************************************************\
 @ p3d_mat3Copy()
 -----------------------------------------------------------------------------
 description : matrix copy:  source => dest
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_mat3Copy(p3d_matrix3 source, p3d_matrix3 dest)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) dest[i][j] = source[i][j];
} /* End of p3d_mat3Copy() **/


/*****************************************************************************\
 @ p3d_mat4Transpose()
 -----------------------------------------------------------------------------
 description : matrix transpose:  (source)^T => dest
 input       :
 output      :
 notes       : source and dest must be distinct!
\*****************************************************************************/
void p3d_mat4Transpose(p3d_matrix4 source, p3d_matrix4 dest)
{
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++) dest[i][j] = source[j][i];
} /* End of p3d_mat4Transpose() **/


/*****************************************************************************\
 @ p3d_mat3Transpose()
 -----------------------------------------------------------------------------
 description : matrix transpose:  (source)^T => dest
 input       :
 output      :
 notes       : source and dest must be distinct!
\*****************************************************************************/
void p3d_mat3Transpose(p3d_matrix3 source, p3d_matrix3 dest)
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++) dest[i][j] = source[j][i];
} /* End of p3d_mat3Transpose() **/


/*****************************************************************************\
 @ p3d_mat4Print()
 -----------------------------------------------------------------------------
 description : matrix print
 input       : If name is non-null, we also print a small title first.
 output      :
 notes       :
\*****************************************************************************/
void p3d_mat4Print(p3d_matrix4 M, const char *name)
{
  int i;

  if (name[0] != '\0') PrintInfo(("matrix %s\n", name));
  for (i = 0; i < 4; i++)
    PrintInfo(("%+10.6f  %+10.6f  %+10.6f  %+10.6f\n",
	   M[i][0], M[i][1], M[i][2], M[i][3]));
} /* End of p3d_mat4Print() **/


/*****************************************************************************\
 @ p3d_mat3Print()
 -----------------------------------------------------------------------------
 description : matrix print
 input       : If name is non-null, we also print a small title first.
 output      :
 notes       :
\*****************************************************************************/
void p3d_mat3Print(p3d_matrix3 M, char *name)
{
  int i;

  if (name[0] != '\0') PrintInfo(("matrix %s\n", name));
  for (i = 0; i < 3; i++)
    PrintInfo(("%+10.6f  %+10.6f  %+10.6f\n",
	   M[i][0], M[i][1], M[i][2]));
} /* End of p3d_mat3Print() **/


/***************************************************************************/
/*
 *! \brief Test the equality between two matrix.
 *
 * \param M1: The first matrix.
 * \param M2: The second matrix.
 *
 * \return whether or not the two matrix are equal.
 */
int p3d_mat4IsEqual(p3d_matrix4 M1, p3d_matrix4 M2)
{
  int i, j;

  for(i=0; i<4; i++) {
    for(j=0; j<4; j++) {
      if (!EQ(M1[i][j], M2[i][j]))
	{ return FALSE; }
    }
  }
  return TRUE;
}


/*****************************************************************************\
 @ p3d_matMultXform()
 -----------------------------------------------------------------------------
 description : transformation matrix multiply:  a * b => c
 input       :
 output      :
 notes       : This routine is much faster than the general 4 x 4 matrix
               multiply above, but only works properly if a and b are
	       SE(3) transformation matrices.  c should not point to the
	       same matrix as a or b!
\*****************************************************************************/
void p3d_matMultXform(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c)
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
} /* End of p3d_matMultXform() **/


/*****************************************************************************\
 @ p3d_matInvertXform()
 -----------------------------------------------------------------------------
 description : transformation matrix inversion:  Inverse(M) => inv
 input       :
 output      :
 notes       : M and inv should not point to the same matrix.  We assume M is
               a transformation matrix; this will not properly invert
	       arbitrary 4x4 matrices.

	       Doesn't work for transformations with scaling
\*****************************************************************************/
void p3d_matInvertXform(p3d_matrix4 M, p3d_matrix4 inv)
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
} /* End of p3d_matInvertXform() **/


/*****************************************************************************\
 @ p3d_matInvertArbitraryXform()
 -----------------------------------------------------------------------------
 description : inversion for arbitrary transformation matrix
 input       :
 output      :
 notes       : the above inversion doesn't work if matrices have scaling in
               them -- this borrowed from PPHIGS implementation at
	       UNC-Chapel Hill
\*****************************************************************************/
int p3d_matInvertArbitraryXform(p3d_matrix4 mat, p3d_matrix4 invmat)
{
    double det;
    p3d_matrix4  cofac;

    cofac[0][0] = (mat[1][1]*mat[2][2] - mat[2][1]*mat[1][2]);
    cofac[0][1] = -(mat[1][0]*mat[2][2] - mat[2][0]*mat[1][2]);
    cofac[0][2] = (mat[1][0]*mat[2][1] - mat[2][0]*mat[1][1]);

    cofac[1][0] = -(mat[0][1]*mat[2][2] - mat[2][1]*mat[0][2]);
    cofac[1][1] = (mat[0][0]*mat[2][2] - mat[2][0]*mat[0][2]);
    cofac[1][2] = -(mat[0][0]*mat[2][1] - mat[2][0]*mat[0][1]);

    cofac[2][0] = (mat[0][1]*mat[1][2] - mat[1][1]*mat[0][2]);
    cofac[2][1] = -(mat[0][0]*mat[1][2] - mat[1][0]*mat[0][2]);
    cofac[2][2] = (mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1]);

    det = mat[0][0]*cofac[0][0] +
      mat[0][1]*cofac[0][1] +
	mat[0][2]*cofac[0][2];

    if (det == 0.)
	return -1;

    invmat[0][0] = cofac[0][0]/det;
    invmat[0][1] = cofac[1][0]/det;
    invmat[0][2] = cofac[2][0]/det;
    invmat[1][0] = cofac[0][1]/det;
    invmat[1][1] = cofac[1][1]/det;
    invmat[1][2] = cofac[2][1]/det;
    invmat[2][0] = cofac[0][2]/det;
    invmat[2][1] = cofac[1][2]/det;
    invmat[2][2] = cofac[2][2]/det;

    invmat[0][3] = -(invmat[0][0]*mat[0][3]+
		     invmat[0][1]*mat[1][3]+
		     invmat[0][2]*mat[2][3]);
    invmat[1][3] = -(invmat[1][0]*mat[0][3]+
		     invmat[1][1]*mat[1][3]+
		     invmat[1][2]*mat[2][3]);
    invmat[2][3] = -(invmat[2][0]*mat[0][3]+
		     invmat[2][1]*mat[1][3]+
		     invmat[2][2]*mat[2][3]);

    invmat[3][0] = invmat[3][1] = invmat[3][2] = 0.;
    invmat[3][3] = 1.0;

    return 0;
} /* End of p3d_matInvertArbitraryXform() **/


/*--------------------------------------------------------------------------*/
/*!
 * \brief Buid the translation with a translation vector.
 *
 * If we have the translation vector \f$ \vec {vect} = [ x \, y \, z ] \f$,
 * we have the following translation matrix:
 * \f[ \left( \begin{array}{cccc}
   1 & 0 & 0 & x \\
   0 & 1 & 0 & y \\
   0 & 0 & 1 & z \\
   0 & 0 & 0 & 1 \end{array} \right) \f]
 *
 * \param  vect: the translation vector.
 *
 * \retval M: the translation matrix.
 */
void p3d_mat4Trans(p3d_matrix4 M, p3d_vector3 vect)
{
  p3d_mat4Copy(p3d_mat4IDENTITY,M);
  M[0][3] = vect[0];
  M[1][3] = vect[1];
  M[2][3] = vect[2];
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Buid the rotation with its axe ang angle.
 *
 * If we have the rotation with the normalize axis
 * \f$ \vec {axe} = [ \Omega_x \, \Omega_y \, \Omega_z ] \f$ and its angle
 * \a t we have the following rotation matrix:
 * \f[ \left( \begin{array}{cccc}
   \Omega_x ^ 2 . (1 - \cos t) + \cos t &
   \Omega_x . \Omega_y . (1 - \cos t) - \Omega_z . \sin t &
   \Omega_x . \Omega_z . (1 - \cos t) + \Omega_y . \sin t & 0 \\
   \Omega_y . \Omega_x . (1 - \cos t) + \Omega_z . \sin t &
   \Omega_y ^ 2 . (1 - \cos t) + \cos t &
   \Omega_y . \Omega_z . (1 - \cos t) - \Omega_x . \sin t & 0 \\
   \Omega_z . \Omega_x . (1 - \cos t) - \Omega_y . \sin t &
   \Omega_z . \Omega_y . (1 - \cos t) + \Omega_x . \sin t &
   \Omega_z ^ 2 . (1 - \cos t) + \cos t & 0  \\
   0 & 0 & 0 & 1 \end{array} \right) \f]
 * Note: If the axis \a axe is not normalize, the function normalize it.
 *
 * \param  axe: the axis of the rotation.
 * \param  t: the angle in radian of the rotation.
 *
 * \retval M: the rotation matrix.
 */
void p3d_mat4Rot(p3d_matrix4 M, p3d_vector3 axe, double t)
{
  double norm, c, s, v;
  double x, y, z;

  x = axe[0];
  y = axe[1];
  z = axe[2];


  p3d_mat4Copy(p3d_mat4IDENTITY,M);

  norm = p3d_vectNorm(axe);
  if (norm == 0.0) {
    return;
  }

  x /= norm;
  y /= norm;
  z /= norm;

  c = cos(t);
  s = sin(t);
  v = 1 - c;

  M[0][0] = x*x*v + c;
  M[1][0] = x*y*v + z*s;
  M[2][0] = x*z*v - y*s;
  M[0][1] = x*y*v - z*s;
  M[1][1] = y*y*v + c;
  M[2][1] = y*z*v + x*s;
  M[0][2] = x*z*v + y*s;
  M[1][2] = y*z*v - x*s;
  M[2][2] = z*z*v + c;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Buid the position matrix with move3d format.
 *
 * Move3D format is 3 translation \f$ (Tx, \, Ty, \, Tz) \f$ and 3 rotations
 * \f$ (Rx, \, Ry, \, Rz) \f$ in that order.
 * This give the following matice:
 * \f$ \left( \begin{array}{cccc}
   \cos Ry . \cos Rz &
   \sin Rx . \sin Ry . \cos Rz - \cos Rx . \sin Rz &
   \cos Rx . \sin Ry . \cos Rz + \sin Rx . \sin Rz & Tx \\
   \cos Ry . \sin Rz &
   \sin Rx . \sin Ry . \sin Rz + \cos Rx . \cos Rz &
   \cos Rx . \sin Ry . \sin Rz - \sin Rx . \cos Rz & Ty \\
   - \sin Ry & \sin Rx . \cos Ry & \cos Rx . \cos Ry & Tz \\
   0 & 0 & 0 & 1 \end{array} \right) \f$
 *
 * \param  Tx: the translation with x axis.
 * \param  Ty: the translation with y axis.
 * \param  Tz: the translation with z axis.
 * \param  Rx: the rotation with x axis.
 * \param  Ry: the rotation with y axis.
 * \param  Rz: the rotation with z axis.
 *
 * \retval M: the placement matrix.
 */
void p3d_mat4Pos(p3d_matrix4 M, double Tx, double Ty, double Tz,
		 double Rx, double Ry, double Rz)
{
  double Sx = sin(Rx);
  double Cx = cos(Rx);
  double Sy = sin(Ry);
  double Cy = cos(Ry);
  double Sz = sin(Rz);
  double Cz = cos(Rz);
  double SyCz = Sy*Cz;
  double SySz = Sy*Sz;

  M[0][0] = Cy*Cz;
  M[0][1] = Sx*SyCz - Cx*Sz;
  M[0][2] = Cx*SyCz + Sx*Sz;
  M[0][3] = Tx;

  M[1][0] = Cy*Sz;
  M[1][1] = Sx*SySz + Cx*Cz;
  M[1][2] = Cx*SySz - Sx*Cz;
  M[1][3] = Ty;

  M[2][0] = -Sy;
  M[2][1] = Sx*Cy;
  M[2][2] = Cx*Cy;
  M[2][3] = Tz;

  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 1;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Buid the position matrix with inverse move3d format.
 *
 *  \note This function is used in the joint position computation.
 *
 *  \warning The computation of the matrix is not the classic order but the
 *           inverse order for the rotation (3 rotations
 *           \f$ (Rz, \, Ry, \, Rx) \f$ in that order and 3 translations
 *           \f$ (Tz, \, Ty, \, Tx) \f$). This give the following matice:
 * \f$ \left( \begin{array}{cccc}
   \cos Ry . \cos Rz & - \cos Ry . \sin Rz & \sin Ry & Tx \\
   \sin Rx . \sin Ry . \cos Rz + \cos Rx . \sin Rz &
   - \sin Rx . \sin Ry . \sin Rz + \cos Rx . \cos Rz &
   - \sin Rx . \cos Ry & Ty \\
   - \cos Rx . \sin Ry . \cos Rz + \sin Rx . \sin Rz &
   \cos Rx . \sin Ry . \sin Rz + \sin Rx . \cos Rz &
   \cos Rx . \cos Ry & Tz \\
   0 & 0 & 0 & 1 \end{array} \right) \f$
 *
 * \param  Tx: the translation with x axis.
 * \param  Ty: the translation with y axis.
 * \param  Tz: the translation with z axis.
 * \param  Rx: the rotation with x axis.
 * \param  Ry: the rotation with y axis.
 * \param  Rz: the rotation with z axis.
 *
 * \retval M: the placement matrix.
 */
void p3d_mat4PosReverseOrder(p3d_matrix4 M, double Tx, double Ty, double Tz,
			     double Rx, double Ry, double Rz)
{
  double Sx = sin(Rx);
  double Cx = cos(Rx);
  double Sy = sin(Ry);
  double Cy = cos(Ry);
  double Sz = sin(Rz);
  double Cz = cos(Rz);
  double SxSy = Sx*Sy;
  double CxSy = Cx*Sy;

  M[0][0] = Cy*Cz;
  M[0][1] = - Cy*Sz;
  M[0][2] = Sy;
  M[0][3] = Tx;

  M[1][0] = SxSy*Cz + Cx*Sz;
  M[1][1] = -SxSy*Sz + Cx*Cz;
  M[1][2] = -Sx*Cy;
  M[1][3] = Ty;

  M[2][0] = -CxSy*Cz + Sx*Sz;
  M[2][1] = CxSy*Sz + Sx*Cz;
  M[2][2] = Cx*Cy;
  M[2][3] = Tz;

  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 1;
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Extract the parameters of a position matrix.
 *
 * Move3D format is 3 translation \f$ (Tx, \, Ty, \, Tz) \f$ and 3 rotations
 * \f$ (Rz, \, Ry, \, Rx) \f$ in that order.
 * This give the following matice:
 * \f$ \left( \begin{array}{cccc}
   \cos Ry . \cos Rz & - \cos Ry . \sin Rz & \sin Ry & Tx \\
   \sin Rx . \sin Ry . \cos Rz + \cos Rx . \sin Rz &
   - \sin Rx . \sin Ry . \sin Rz + \cos Rx . \cos Rz &
   - \sin Rx . \cos Ry & Ty \\
   - \cos Rx . \sin Ry . \cos Rz + \sin Rx . \sin Rz &
   \cos Rx . \sin Ry . \sin Rz + \sin Rx . \cos Rz &
   \cos Rx . \cos Ry & Tz \\
   0 & 0 & 0 & 1 \end{array} \right) \f$
 *
 * The decomposition is not unique. This function return
 * \f$ Ry \in [-\frac{\pi}{2}, \frac{\pi}{2}] \f$.
 */
void p3d_mat4ExtractPosReverseOrder(p3d_matrix4 M,
				    double * Tx, double * Ty, double * Tz,
				    double * Rx, double * Ry, double * Rz)
{

//   double cy;
//
//   (*Ry) = asin(M[0][2]);
//   cy = cos(*Ry);
//   if (EQ(cy, 0)) {
//     (*Rx) = 0.0;
//     if (M[1][0]<0)
//       { (*Rz) = - acos(M[1][1]); }
//     else
//       { (*Rz) =   acos(M[1][1]); }
//   } else {
//     if (M[0][1] < 0)
//       { (*Rz) =   acos(M[0][0] / cy); }
//     else
//       { (*Rz) = - acos(M[0][0] / cy); }
//     if (M[1][2] < 0)
//       { (*Rx) =   acos(M[2][2] / cy); }
//     else
//       { (*Rx) = - acos(M[2][2] / cy); }
//   }
//   *Tx = M[0][3];
//   *Ty = M[1][3];
//   *Tz = M[2][3];
 double cy;
  double epsilon= 10e-6;

  (*Ry)= asin(M[0][2]);
  cy = cos( (*Ry) );
  if( (-epsilon < cy)  &&  (cy < epsilon) )
  {
    (*Rx) = 0.0;
    (*Rz)= atan2( M[1][0], M[1][1] );
  }
  else
  {
    (*Rx)= -atan2( M[1][2], M[2][2] );
    (*Rz)= -atan2( M[0][1], M[0][0] );

    if( (*Ry)<0 && (*Ry)<-M_PI_2 )
      (*Ry)= -M_PI - (*Ry);

    if( (*Ry)>0 && (*Ry)>M_PI_2 )
      (*Ry)= M_PI - (*Ry);
  }

  (*Tx) = M[0][3];
  (*Ty) = M[1][3];
  (*Tz) = M[2][3];
}


//! A partir d'une matrice de transformation homogene, cette fonction extrait les parametres de translation
//! et les angles d'Euler associes a la sous-matrice de rotation.
//! La matrice de rotation est supposee avoir ete calculee a partir des angles d'Euler avec
//! la fonction p3d_mat4PosReverseOrder() de move3D i.e. R= transpose(Rz*Ry*Rx) avec
//! Rx, Ry et Rz les matrices de rotation selon x, y et z.
//! Le deuxieme angle de rotation retourne est choisi pour être entre -pi/2 et pi/2.
//! NOTE: cette fonction a due être ajoutee car la fonction de move3D censee faire la même chose
//! ne marche pas dans tous les cas.
void p3d_mat4ExtractPosReverseOrder2(p3d_matrix4 M,
				    double * tx, double * ty, double * tz,
				    double * ax, double * ay, double * az)
{
  double cy;
  double epsilon= 10e-6;

  (*ay)= asin(M[0][2]);
  cy = cos( (*ay) );
  if( (-epsilon < cy)  &&  (cy < epsilon) )
  {
    (*ax) = 0.0;
    (*az)= atan2( M[1][0], M[1][1] );
  }
  else
  {
    (*ax)= -atan2( M[1][2], M[2][2] );
    (*az)= -atan2( M[0][1], M[0][0] );

    if( (*ay)<0 && (*ay)<-M_PI_2 )
      (*ay)= -M_PI - (*ay);

    if( (*ay)>0 && (*ay)>M_PI_2 )
      (*ay)= M_PI - (*ay);
  }

  (*tx) = M[0][3];
  (*ty) = M[1][3];
  (*tz) = M[2][3];
}

// Calcule l'angle et l'axe d'une rotation à partir d'une matrice de transformation 4x4
// (l'ancienne fonction move3d de base
// ne marche pas pour certains cas singuliers).
// Conversion d'un code en Java de Martin Baker (http://www.euclideanspace.com/index.html).
void p3d_mat4ExtractRot ( p3d_matrix4 M, p3d_vector3 axis, double *angle )
{
 double epsilon = 0.01; // margin to allow for rounding errors

 if ((fabs(M[0][1]-M[1][0])< epsilon)  && (fabs(M[0][2]-M[2][0])< epsilon)  && (fabs(M[1][2]-M[2][1])< epsilon))
 {    // singularity found
   // first check for identity matrix which must have +1 for all terms in leading diagonal
   // and zero in other terms
   if ( (fabs(M[0][1]+M[1][0]) < 0.1)  && (fabs(M[0][2]+M[2][0]) < 0.1)  && (fabs(M[1][2]+M[2][1]) < 0.1)
     && (fabs(M[0][0]+M[1][1]+M[2][2]-3) < 0.1) )
   {
     // this singularity is identity matrix so angle = 0
     // note epsilon is greater in this case since we only have to distinguish between 0 and 180 degrees
     // zero angle, arbitrary axis
     axis[0]= 1;
     axis[1]= 0;
     axis[2]= 0;
     *angle= 0;
     return;
   }

   // otherwise this singularity is angle = 180
   *angle = M_PI;
   axis[0] = (M[0][0]+1)/2;
   if (axis[0] > 0)
   { // can only take square root of positive number, always true for orthogonal matrix
     axis[0] = sqrt(axis[0]);
   }
   else
   {
     axis[0] = 0; // in case matrix has become de-orthogonalised
   }

   axis[1] = (M[1][1]+1)/2;
   if (axis[1] > 0)
   { // can only take square root of positive number, always true for orthogonal matrix
     axis[1] = sqrt(axis[1]);
   }
   else
   {
     axis[1] = 0; // in case matrix has become de-orthogonalised
   }

   axis[2] = (M[2][2]+1)/2;
   if (axis[2] > 0)
   { // can only take square root of positive number, always true for orthogonal matrix
     axis[2] = sqrt(axis[2]);
   }
   else
   {
     axis[2] = 0; // in case matrix has become de-orthogonalised
   }


   int xZero = (fabs(axis[0])<epsilon);
   int yZero = (fabs(axis[1])<epsilon);
   int zZero = (fabs(axis[2])<epsilon);
   int xyPositive = (M[0][1] > 0);
   int xzPositive = (M[0][2] > 0);
   int yzPositive = (M[1][2] > 0);
   if (xZero && !yZero && !zZero)
   { // implements  last 6 rows of above table
     if (!yzPositive)
       axis[1] = -axis[1];
   }
   else
   {
     if (yZero && !zZero)
     {
    if (!xzPositive)
         axis[2] = -axis[2];
     }
     else
     {
       if (zZero)
       {
     if (!xyPositive)
          axis[0] = -axis[0];
       }
     }
   }

   return;
 }


 double s= sqrt((M[2][1] - M[1][2])*(M[2][1] - M[1][2])+(M[0][2] - M[2][0])*(M[0][2] - M[2][0])+(M[1][0] - M[0][1])*(M[1][0] - M[0][1])); // used to normalise

 if (fabs(s) < 0.001) s=1; // prevent divide by zero, should not happen
                           // if matrix is orthogonal and should be
 // caught by singularity test above, but I've left it in just in case
 *angle = acos(( M[0][0] + M[1][1] + M[2][2] - 1)/2);
 axis[0] = (M[2][1] - M[1][2])/s;
 axis[1] = (M[0][2] - M[2][0])/s;
 axis[2] = (M[1][0] - M[0][1])/s;

 return;
}

/*--------------------------------------------------------------------------*/
/*!
 * \brief Buid the rotation axis and the angle between two axis
 *
 * \param  axe1: the first axis (must be normalize)
 * \param  axe2: the second axis (must be normalize)
 *
 * \retval axe: the rotation axis between the two axis
 * \retval t:   the angle of rotation between the two axis
 */
void p3d_vectDeltaRot(p3d_vector3 axe, double * t, p3d_vector3 axe1,
		      p3d_vector3 axe2)
{
  double v_sin, v_cos, scale;

  p3d_vectXprod(axe1, axe2, axe);
  v_sin = p3d_vectNorm(axe);
  v_cos = p3d_vectDotProd(axe1, axe2);

  if (v_sin<EPS6) {
    if (EQ(axe1[0], 0)) {
      axe[0] = 0.0;
      axe[1] =  axe1[2];
      axe[2] = -axe1[1];
    } else {
      axe[0] =  axe1[1];
      axe[1] = -axe1[0];
      axe[2] = 0.0;
    }
    scale = p3d_vectNorm(axe);
    p3d_vectScale(axe, axe, 1/scale);
    if (v_cos>0)
      { *t = 0.0; }
    else
      { *t = M_PI; }
  } else {
    p3d_vectScale(axe, axe, 1/v_sin);
    *t = acos(v_cos);
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Buid the rotation matrix between two rotations matrix difine by
 *        their axes and angles.
 *
 * Use p3d_mat4Rot() to build both matrix \a M1 with \a axe1 and \a t1,
 * \a M2 with \a axe2 and \a t2 and the matrix \a M3 between those two axis.
 * Then return \f$ M = M2 . M3 . M1^{-1} \f$.
 *
 * \param  axe1: the axis of the first rotation.
 * \param  t1: the angle in radian of the first rotation.
 * \param  axe2: the axis of the second rotation.
 * \param  t2: the angle in radian of the second rotation.
 *
 * \retval M: the rotation matrix between the first and second rotation.
 */
void p3d_mat4DeltaRot(p3d_matrix4 M, p3d_vector3 axe1, double t1,
		      p3d_vector3 axe2, double t2)
{
  p3d_matrix4 M1, M2, M3, invM1;
  p3d_vector3 axe;
  double t;

  /* Inverse first rotation */
  p3d_mat4Rot(M1, axe1, t1);
  p3d_matInvertXform(M1, invM1);

  /* Rotation between the two axis */
  p3d_vectDeltaRot(axe, &t, axe1, axe2);
  p3d_mat4Rot(M2, axe, t);
  p3d_matMultXform(M2, invM1, M3);

  /* Second rotation */
  p3d_mat4Rot(M2, axe2, t2);
  p3d_matMultXform(M2, M3, M);
}


/*****************************************************************************\
 @ p3d_matBuildXform()
 -----------------------------------------------------------------------------
 description : build xformation matrix

               axes is a string (e.g. "xyz", "yzy", or "zx") which specifies
	       the order of axes rotations.  For example, "zx" means rotate
	       about the z-axis, then   about the x-axis.  All rotations are
	       w.r.t. to the current frame, not a fixed frame.  angles[i]
	       specifies the amount of rotation in degrees (!) about the
	       axis specified by the ith character of axes.  dx, dy, and dz
	       are the displacement components of the xformation matrix.
	       The matrix is returned in M.
 input       :
 output      :
 notes       : Note that the resulting matrix applies these transformations in
               order from right to left, for premultiplying a column vector
\*****************************************************************************/
void p3d_matBuildXform(char *axes, p3d_matrix_type angles[],
			 p3d_matrix_type dx, p3d_matrix_type dy, p3d_matrix_type dz,
			 p3d_matrix4 M)
{
  p3d_matrix4 Mi, Mold;
  p3d_matrix_type s, c;

  p3d_mat4Copy(p3d_mat4IDENTITY, M);
  for (; *axes; axes++) {
    p3d_mat4Copy(M, Mold);
    p3d_mat4Copy(p3d_mat4IDENTITY, Mi);
    s = sin(*angles * (M_PI / 180.0));
    c = cos(*angles * (M_PI / 180.0));

    angles++;
    if (*axes == 'x') {
      Mi[1][1] = Mi[2][2] = c;
      Mi[1][2] = -s;
      Mi[2][1] = s;
    }
    else if (*axes == 'y') {
      Mi[0][0] = Mi[2][2] = c;
      Mi[2][0] = -s;
      Mi[0][2] = s;
    }
    else {
      Mi[0][0] = Mi[1][1] = c;
      Mi[0][1] = -s;
      Mi[1][0] = s;
    }
    p3d_mat4Mult(Mold, Mi, M);
  }
  M[0][3] = dx;
  M[1][3] = dy;
  M[2][3] = dz;
  M[3][0] = M[3][1] = M[3][2] = 0.0;
  M[3][3] = 1.0;
} /* End of p3d_matBuildXform() **/




/*
  =============================================================================

  xformation stuff

  The following routines transform points and vectors using transformation
  matrices.  Points and vectors are both elements of R^3, but are
  transformed differently.  Points are homogenized prior to transforming,
  meaning that a 1 is added as a fourth coordinate, before multiplying by
  the 4x4 transformation matrix.  For vectors, on the other hand, the
  fourth coordinate is 0.  The fourth coordinate of the transformed
  point or vector is never computed, since it is always 1 or 0
  respectively, and this value is implicitly assumed.

  =============================================================================
*/


/*****************************************************************************\
 @ p3d_xformPoint()
 -----------------------------------------------------------------------------
 description : transform a point:  M * (p 1) => (p2 1)
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_xformPoint(p3d_matrix4 M, p3d_vector3 p, p3d_vector3 p2)
{
  int i;

  for (i = 0; i < 3; i++)
    p2[i] = M[i][0] * p[0] + M[i][1] * p[1] + M[i][2] * p[2] + M[i][3];
} /* End of p3d_xformPoint() **/


/*****************************************************************************\
 @ p3d_xformVect()
 -----------------------------------------------------------------------------
 description : transform a vector:  M * (v 0) => (v2 0)
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_xformVect(p3d_matrix4 M, p3d_vector3 v, p3d_vector3 v2)
{
  int i;

  for (i = 0; i < 3; i++)
    v2[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
} /* End of p3d_xformVect() **/



/*****************************************************************************\
 @ p3d_xform4()
 -----------------------------------------------------------------------------
 description : transform a 4-vector: M * x => x2

	       This routine does a full transformation of a vector x in R^4.
	       Unlike, xformPoint and xformVect, no value is implictly assumed
	       for the fourth coordinate of x; whatever value is there is
	       actually used.  Also, all four components of the resultant
	       vector are computed, not just the first three.
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_xform4(p3d_matrix4 M, p3d_vector4 x, p3d_vector4 x2)
{
  int i;

  for (i = 0; i < 4; i++)
    x2[i] = M[i][0] * x[0] + M[i][1] * x[1] + M[i][2] * x[2] + M[i][3] * x[3];
} /* End of p3d_xform4() **/



/*****************************************************************************\
 @ p3d_xform3()
 -----------------------------------------------------------------------------
 description : transform a 3-vector: M * x => x2

               This routine does a full transformation of a vector x in R^3.
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_xform3(p3d_matrix3 M, p3d_vector3 x, p3d_vector3 x2)
{
  int i;

  for (i = 0; i < 3; i++)
    x2[i] = M[i][0] * x[0] + M[i][1] * x[1] + M[i][2] * x[2];
} /* End of p3d_xform3() **/


/*
  =============================================================================

  vector stuff

  The following operations manipulate vectors.  The vectors are assumed to
  be in R^3.

  =============================================================================
*/


/*****************************************************************************\
 @ p3d_vectCopy()
 -----------------------------------------------------------------------------
 description : vector copy:  src => dest
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_vectCopy(p3d_vector3 src, p3d_vector3 dest)
{
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
} /* End of p3d_vectCopy() **/


/*****************************************************************************\
 @ p3d_vectAdd()
 -----------------------------------------------------------------------------
 description : vector addition:  a + b => c
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_vectAdd(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c)
{
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
} /* End of p3d_vectAdd() **/


/*****************************************************************************\
 @ p3d_vectSub()
 -----------------------------------------------------------------------------
 description : vector subtraction:  a - b => c
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_vectSub(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c)
{
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
} /* End of p3d_vectSub() **/


/*****************************************************************************\
 @ p3d_vectNeg()
 -----------------------------------------------------------------------------
 description : vector negation:  -src => dest
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_vectNeg(p3d_vector3 src, p3d_vector3 dest)
{
  dest[0] = - src[0];
  dest[1] = - src[1];
  dest[2] = - src[2];
} /* End of p3d_vectNeg() **/


/*****************************************************************************\
 @ p3d_vectScale()
 -----------------------------------------------------------------------------
 description : vector scaling: k * src => dest
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_vectScale(p3d_vector3 src, p3d_vector3 dest, p3d_matrix_type k)
{
  dest[0] = k * src[0];
  dest[1] = k * src[1];
  dest[2] = k * src[2];
} /* End of p3d_vectScale() **/


/*****************************************************************************\
 @ p3d_vectNormalize()
 -----------------------------------------------------------------------------
 description : vector normalize: src / |src|  => dest
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_vectNormalize(p3d_vector3 src, p3d_vector3 dest)
{
  p3d_matrix_type l;

  l = p3d_vectNorm(src);
  dest[0] = src[0] / l;
  dest[1] = src[1] / l;
  dest[2] = src[2] / l;
} /* End of p3d_vectNormalize() **/

//! Normalizes a 4D vector.
void p3d_vect4Normalize(p3d_vector4 src, p3d_vector4 dest)
{
  p3d_matrix_type l;

  l = p3d_vect4Norm(src);
  dest[0] = src[0] / l;
  dest[1] = src[1] / l;
  dest[2] = src[2] / l;
  dest[3] = src[3] / l;
} 

//! Returns the euclidean distance between two 3D vectors.
p3d_matrix_type p3d_vectDistance ( p3d_vector3 a,  p3d_vector3 b)
{
  return sqrt( pow(a[0]-b[0],2) + pow(a[1]-b[1],2) + pow(a[2]-b[2],2));
}

/*****************************************************************************\
 @ p3d_vectNorm()
 -----------------------------------------------------------------------------
 description : compute vector norm
 input       :
 output      :
 notes       :
\*****************************************************************************/
p3d_matrix_type p3d_vectNorm(p3d_vector3 v)
{
  return sqrt((double) ( SQR(v[0]) + SQR(v[1]) + SQR(v[2]) ) );
} /* End of p3d_vectNorm() **/

p3d_matrix_type p3d_vect4Norm(p3d_vector4 v)
{
  return sqrt((double) ( SQR(v[0]) + SQR(v[1]) + SQR(v[2]) + SQR(v[3]) ) );
} 

/*****************************************************************************\
 @ p3d_square_of_vectNorm()
 -----------------------------------------------------------------------------
 description : compute square of vector norm
 input       :
 output      :
 notes       :
\*****************************************************************************/
p3d_matrix_type p3d_square_of_vectNorm(p3d_vector3 v)
{
  return (double) ( SQR(v[0]) + SQR(v[1]) + SQR(v[2]) );
} /* End of p3d_square_of_vectNorm() **/


/*****************************************************************************\
 @ p3d_vectEqual()
 -----------------------------------------------------------------------------
 description : strict vector equality
 input       :
 output      : Return true if the vectors are exactly equal, else return false.
 notes       :
\*****************************************************************************/
int p3d_vectEqual(p3d_vector3 a, p3d_vector3 b)
{
  return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
} /* End of p3d_vectEqual() **/


/*****************************************************************************\
 @ p3d_vectDotProd()
 -----------------------------------------------------------------------------
 description : vector dot product:  return a . b
 input       :
 output      :
 notes       :
\*****************************************************************************/
p3d_matrix_type p3d_vectDotProd(p3d_vector3 a, p3d_vector3 b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
} /* End of p3d_vectDotProd() **/


/*****************************************************************************\
 @ p3d_vectXprod()
 -----------------------------------------------------------------------------
 description : vector cross product:  a x b => c
 input       :
 output      :
 notes       : c should not point to the same vector as a or b!
\*****************************************************************************/
void p3d_vectXprod(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c)
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
} /* End of p3d_vectXprod() **/



/*
  =============================================================================

  misc. stuff

  The following are miscellaneous operations for points, vectors, etc.

  =============================================================================
*/


/*****************************************************************************\
 @ p3d_planeDist()
 -----------------------------------------------------------------------------
 description : plane distance

               Compute the distance from a point to a plane.  The plane is
               specified by four coefficients, i.e. the plane equation is

               plane[0] * x + plane[1] * y + plane[2] * z + plane[3] = 0.

               We assume that the first three plane coordinates form a unit
	       vector (the unit normal to the plane).  The "distance" returned
	       is actually a signed distance.  That is, if the point lies on
	       the side of the plane to which the unit normal points, the
	       result is positive.  If the point lies on the other side, the
	       distance is negative.
 input       :
 output      :
 notes       :
\*****************************************************************************/
p3d_matrix_type p3d_planeDist(p3d_vector4 plane, p3d_vector3 point)
{
  return plane[0] * point[0] +
         plane[1] * point[1] +
         plane[2] * point[2] + plane[3];
} /* End of p3d_planeDist() **/


/*****************************************************************************\
 @ p3d_displacePoint()
 -----------------------------------------------------------------------------
 description : displace point:  point + lambda * vect => result

                 Compute the point which is displaced (lambda * vect) from
		 point.
 input       :
 output      :
 notes       :
\*****************************************************************************/
void p3d_displacePoint(p3d_vector3 point, p3d_vector3 vect,
			 p3d_matrix_type lambda, p3d_vector3 result)
{
  result[0] = point[0] + lambda * vect[0];
  result[1] = point[1] + lambda * vect[1];
  result[2] = point[2] + lambda * vect[2];
} /* End of p3d_displacePoint() **/


/*****************************************************************************\
 @ p3d_mat3Det()
 -----------------------------------------------------------------------------
 description : determinat of matrix 3x3
 input       : the matrix
 output      : the determinant
 notes       :
\*****************************************************************************/
double p3d_mat3Det(p3d_matrix3 mat)
{  double det;

   det = mat[0][0]*mat[1][1]*mat[2][2]+mat[0][1]*mat[1][2]*mat[2][0]+mat[0][2]*mat[1][0]*mat[2][1]-
         mat[0][2]*mat[1][1]*mat[2][0]-mat[0][1]*mat[1][0]*mat[2][2]-mat[0][0]*mat[1][2]*mat[2][1];

   return det;
}  

/* modif Juan */
/*****************************************************************************\
 @ p3d_mat4Det()
 -----------------------------------------------------------------------------
 description : determinat of matrix 4x4
 input       : the matrix
 output      : the determinant
 notes       :
\*****************************************************************************/
double p3d_mat4Det(p3d_matrix4 m)
{  double det;

   det = m[0][0]*m[1][1]*m[2][2]*m[3][3]-m[0][0]*m[1][1]*m[2][3]*m[3][2]
        -m[0][0]*m[2][1]*m[1][2]*m[3][3]+m[0][0]*m[2][1]*m[1][3]*m[3][2]
        +m[0][0]*m[3][1]*m[1][2]*m[2][3]-m[0][0]*m[3][1]*m[1][3]*m[2][2]
        -m[1][0]*m[0][1]*m[2][2]*m[3][3]+m[1][0]*m[0][1]*m[2][3]*m[3][2]
        +m[1][0]*m[2][1]*m[0][2]*m[3][3]-m[1][0]*m[2][1]*m[0][3]*m[3][2]
        -m[1][0]*m[3][1]*m[0][2]*m[2][3]+m[1][0]*m[3][1]*m[0][3]*m[2][2]
        +m[2][0]*m[0][1]*m[1][2]*m[3][3]-m[2][0]*m[0][1]*m[1][3]*m[3][2]
        -m[2][0]*m[1][1]*m[0][2]*m[3][3]+m[2][0]*m[1][1]*m[0][3]*m[3][2]
        +m[2][0]*m[3][1]*m[0][2]*m[1][3]-m[2][0]*m[3][1]*m[0][3]*m[1][2]
        -m[3][0]*m[0][1]*m[1][2]*m[2][3]+m[3][0]*m[0][1]*m[1][3]*m[2][2]
        +m[3][0]*m[1][1]*m[0][2]*m[2][3]-m[3][0]*m[1][1]*m[0][3]*m[2][2]
        -m[3][0]*m[2][1]*m[0][2]*m[1][3]+m[3][0]*m[2][1]*m[0][3]*m[1][2];

   return det;
}
/* fmodif Juan */

/* modif Etienne Ferre */
/*****************************************************************************\
 @ p3d_mat3Invert()
 -----------------------------------------------------------------------------
 description : inversion for matrix 3x3
 input       :
 output      : if det ==0 return -1 else return 0
 notes       :
\*****************************************************************************/
int p3d_mat3Invert(p3d_matrix3 mat, p3d_matrix3 invmat)
{
    double det;
    p3d_matrix3  cofac;

    cofac[0][0] = (mat[1][1]*mat[2][2] - mat[2][1]*mat[1][2]);
    cofac[0][1] = -(mat[1][0]*mat[2][2] - mat[2][0]*mat[1][2]);
    cofac[0][2] = (mat[1][0]*mat[2][1] - mat[2][0]*mat[1][1]);

    cofac[1][0] = -(mat[0][1]*mat[2][2] - mat[2][1]*mat[0][2]);
    cofac[1][1] = (mat[0][0]*mat[2][2] - mat[2][0]*mat[0][2]);
    cofac[1][2] = -(mat[0][0]*mat[2][1] - mat[2][0]*mat[0][1]);

    cofac[2][0] = (mat[0][1]*mat[1][2] - mat[1][1]*mat[0][2]);
    cofac[2][1] = -(mat[0][0]*mat[1][2] - mat[1][0]*mat[0][2]);
    cofac[2][2] = (mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1]);

    det = mat[0][0]*cofac[0][0] +
      mat[0][1]*cofac[0][1] +
	mat[0][2]*cofac[0][2];

    if (det == 0.)
	return -1;

    invmat[0][0] = cofac[0][0]/det;
    invmat[0][1] = cofac[1][0]/det;
    invmat[0][2] = cofac[2][0]/det;
    invmat[1][0] = cofac[0][1]/det;
    invmat[1][1] = cofac[1][1]/det;
    invmat[1][2] = cofac[2][1]/det;
    invmat[2][0] = cofac[0][2]/det;
    invmat[2][1] = cofac[1][2]/det;
    invmat[2][2] = cofac[2][2]/det;

    return 0;
} /* End of p3d_mat3Invert() **/

/*****************************************************************************\
 @ p3d_vec3Mat3Mult()
 -----------------------------------------------------------------------------
 description : matrix multiplied by vector :  M * a => b
 input       :
 output      :
 notes       : c should not point to the same matrix as a or b!
\*****************************************************************************/
void p3d_vec3Mat3Mult(p3d_matrix3 M, p3d_vector3 a, p3d_vector3 b)
{
  b[0] = M[0][0]*a[0] + M[0][1]*a[1] + M[0][2]*a[2];
  b[1] = M[1][0]*a[0] + M[1][1]*a[1] + M[1][2]*a[2];
  b[2] = M[2][0]*a[0] + M[2][1]*a[1] + M[2][2]*a[2];

} /* End of p3d_vec3Mat3Mult**/

/* finmodif Etienne Ferre */

/*****************************************************************************\
 @ p3d_isTransfMat()
 -----------------------------------------------------------------------------
 description : test if a matrix is a transformation
 input       :
 output      : return TRUE if it is
 notes       :
\*****************************************************************************/
int p3d_isTransfMat(p3d_matrix4 M)
{
  double a,b;
  double eps = 0.001;
  int i, j;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      {
	a = M[i][0]*M[j][0] + M[i][1]*M[j][1] + M[i][2]*M[j][2];
	b = (i==j);
	if (fabs(a-b)>eps)
	  return FALSE;
      }

  if ((M[3][0]!=0)||(M[3][1]!=0)||(M[3][2]!=0)||(M[3][3]!=1))
    return FALSE;
  return TRUE;
} /* End of p3d_isTransfMat**/
/*****************************************************************************\
 @ p3d_extractScale()
 -----------------------------------------------------------------------------
 description : try to extract a scale factor from a matrix
 input       :
 output      : return TRUE if success
 notes       :
\*****************************************************************************/
int p3d_extractScale(p3d_matrix4 M, double *scale)
{
  double a;
  double eps = 0.001;
  int i, j;
  p3d_matrix3 I3;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      I3[i][j]=M[i][0]*M[j][0] + M[i][1]*M[j][1] + M[i][2]*M[j][2];

  *scale = sqrt(M[0][0]*M[0][0] + M[0][1]*M[0][1] + M[0][2]*M[0][2]);
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      {
	a = M[i][0]*M[j][0] + M[i][1]*M[j][1] + M[i][2]*M[j][2];
	if (i==j)
	  {
	    if (fabs(sqrt(a)-*scale)/(*scale) > eps)
	      return FALSE;
	  }
	else
	  {
	    if (fabs(a)/(*scale)>eps)
	      return FALSE;
	  }
      }

  if ((M[3][0]!=0)||(M[3][1]!=0)||(M[3][2]!=0)||(M[3][3]!=1))
    return FALSE;

  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      M[i][j]/=*scale;
  return TRUE;
} /* End of p3d_extractScale**/

/*****************************************************************************\
 @ p3d_ma4ExtractRotMat()
 -----------------------------------------------------------------------------
 description : extract the rotation matrix from the given homogeneous matrix
 input       : the Homogeneous matrix
 output      : the corresponding rotation matrix
 notes       : c should not point to the same matrix as a or b!
\*****************************************************************************/
void p3d_ma4ExtractRotMat(p3d_matrix4 src, p3d_matrix3 dest)
{
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      dest[i][j] = src[i][j];
    }
  }
} /* End of p3d_ma4ExtractRotMat**/
/*****************************************************************************\
\*****************************************************************************/


//! Inverts a  2x2 matrix.
//! \param mat a 2x2 matrix
//! \param invmat the inverse of mat
int p3d_mat2Invert(p3d_matrix2 mat, p3d_matrix2 invmat)
{
   double det= mat[0][0]*mat[1][1] - mat[1][0]*mat[0][1];
   if( fabs(det) < EPSILON)
   {
      #ifdef DEBUG
        //printf("%s: %d: p3d_mat2Invert(): matrice non inversible\n", __FILE__, __LINE__);
      #endif
      return 0;
   }

   invmat[0][0]=   mat[1][1]/det;    invmat[0][1]=  -mat[0][1]/det;
   invmat[1][0]=  -mat[1][0]/det;    invmat[1][1]=   mat[0][0]/det;

   return 1;
}


//! Adaptation of function p3d_mat4Rot to 3x3 matrices.
//! Computes and fills a rotation matrix for a rotation described by an axis and an angle.
//! \param M the computed rotation matrix
//! \param axis the axis of the desired rotation
//! \param t the angle of the desired rotation
void p3d_mat3Rot(p3d_matrix3 M, p3d_vector3 axis, double t)
{
  double norm, c, s, v;
  double x, y, z;

  x = axis[0];
  y = axis[1];
  z = axis[2];

  p3d_mat3Copy(p3d_mat3IDENTITY,M);

  norm = p3d_vectNorm(axis);
  if (norm == 0.0) {
    return;
  }

  x /= norm;
  y /= norm;
  z /= norm;

  c = cos(t);
  s = sin(t);
  v = 1 - c;

  M[0][0] = x*x*v + c;
  M[1][0] = x*y*v + z*s;
  M[2][0] = x*z*v - y*s;
  M[0][1] = x*y*v - z*s;
  M[1][1] = y*y*v + c;
  M[2][1] = y*z*v + x*s;
  M[0][2] = x*z*v + y*s;
  M[1][2] = y*z*v - x*s;
  M[2][2] = z*z*v + c;
}

//! Extracts the rotation matrix of a transform matrix.
//! \param M a 4x4 homogeneous transform input matrix
//! \param R the output matrix filled with the rotation part of M
void p3d_mat4ExtractRotMatrix( p3d_matrix4 M, p3d_matrix3 R)
{
  R[0][0]= M[0][0];  R[0][1]= M[0][1];  R[0][2]= M[0][2];
  R[1][0]= M[1][0];  R[1][1]= M[1][1];  R[1][2]= M[1][2];
  R[2][0]= M[2][0];  R[2][1]= M[2][1];  R[2][2]= M[2][2];
}

//! Sets the rotation part of a transform matrix from a p3d_matrix3.
//! \param R a 3x3 rotation matrix
//! \param M a 4x4 homogeneous transform matrix
void p3d_mat4SetRotMatrix(p3d_matrix3 R, p3d_matrix4 M)
{
  M[0][0]= R[0][0];  M[0][1]= R[0][1];  M[0][2]= R[0][2];
  M[1][0]= R[1][0];  M[1][1]= R[1][1];  M[1][2]= R[1][2];
  M[2][0]= R[2][0];  M[2][1]= R[2][1];  M[2][2]= R[2][2];
}

//! Computes the transform matrix combining a translation (tx, ty, tz) and a rotation (axis,angle).
void p3d_mat4TransRot( p3d_matrix4 M, double tx, double ty, double tz, p3d_vector3 axis, double angle)
{
   p3d_mat4Rot(M, axis, angle);
   M[0][3]= tx;
   M[1][3]= ty;
   M[2][3]= tz;
}


//! Extracts the translation of a transform matrix.
void p3d_mat4ExtractTrans ( p3d_matrix4 M, p3d_vector3 v)
{
   v[0]= M[0][3];
   v[1]= M[1][3];
   v[2]= M[2][3];
}


//! Computes a vector that is orthogonal to the vector v and normalizes it.
//! The function returns an arbitrary choice for the orthogonal vector.
void p3d_orthogonal_vector(p3d_vector3 v, p3d_vector3 result)
{
    if( p3d_vectNorm(v) < EPSILON )
    {
        printf("%s: %d: p3d_orthogonal_vector(): bad input (vector norm is null).\n",__FILE__,__LINE__);
        result[0]= 1;
        result[1]= 0;
        result[2]= 0;
        return;
    }

    if( fabs(v[2]) <= EPSILON )
    {
        result[0]= 0;
        result[1]= 0;
        result[2]= 1;
        return;
    }
    else
    {
        result[0]= 0;
        result[1]= 1;
        result[2]= -v[1]/v[2];
        p3d_vectNormalize(result, result);
        return;
    }
}

//! Computes the vectors v and w such as (u,v,w) is a direct orthonormal base.
//! The function returns an arbitrary choice.
void p3d_orthonormal_basis(p3d_vector3 u, p3d_vector3 v, p3d_vector3 w)
{
    p3d_orthogonal_vector(u, v);
    p3d_vectXprod(u, v, w);
    p3d_vectNormalize(w, w);
}

//! Converts a quaternion to a rotation matrix.
void p3d_quaternion_to_matrix3(p3d_vector4 q0, p3d_matrix3 R)
{
  p3d_vector4 q;
  double a2, b2, c2, d2, ab, ac, ad, bc, bd, cd;

  p3d_vect4Normalize(q0, q);

  a2= q[0]*q[0];
  b2= q[1]*q[1];
  c2= q[2]*q[2];
  d2= q[3]*q[3];

  ab= q[0]*q[1];
  ac= q[0]*q[2];
  ad= q[0]*q[3];
  bc= q[1]*q[2];
  bd= q[1]*q[3];
  cd= q[2]*q[3];

  R[0][0]= a2 + b2 -c2 - d2;
  R[1][0]= 2*ad + 2*bc;
  R[2][0]= 2*bd - 2*ac;

  R[0][1]= 2*bc - 2*ad;
  R[1][1]= a2 - b2 + c2 - d2;
  R[2][1]= 2*ab + 2*cd;

  R[0][2]= 2*ac + 2*bd;
  R[1][2]= 2*cd - 2*ab;
  R[2][2]= a2 - b2 -c2 + d2;
}

//! Converts a quaternion to the rotation part of a 4x4 transform matrix.
//! The translation part is not modified.
//! NB: it is exactly the same function as p3d_quaternion_to_matrix3 except
//! fot the type of output.
void p3d_quaternion_to_matrix4(p3d_vector4 q0, p3d_matrix4 T)
{
  p3d_vector4 q;
  double a2, b2, c2, d2, ab, ac, ad, bc, bd, cd;

  p3d_vect4Normalize(q0, q);

  a2= q[0]*q[0];
  b2= q[1]*q[1];
  c2= q[2]*q[2];
  d2= q[3]*q[3];

  ab= q[0]*q[1];
  ac= q[0]*q[2];
  ad= q[0]*q[3];
  bc= q[1]*q[2];
  bd= q[1]*q[3];
  cd= q[2]*q[3];

  T[0][0]= a2 + b2 -c2 - d2;
  T[1][0]= 2*ad + 2*bc;
  T[2][0]= 2*bd - 2*ac;

  T[0][1]= 2*bc - 2*ad;
  T[1][1]= a2 - b2 + c2 - d2;
  T[2][1]= 2*ab + 2*cd;

  T[0][2]= 2*ac + 2*bd;
  T[1][2]= 2*cd - 2*ab;
  T[2][2]= a2 - b2 -c2 + d2;

  T[3][0]= T[3][1]= T[3][2]= 0.0;
  T[3][3]= 1.0;
}

//! Converts a rotation matrix to a quaternion.
void p3d_matrix3_to_quaternion(p3d_matrix3 R, p3d_vector4 q)
{
  unsigned int u, v, w;
  int case_;
  double r, n;

  // find the largest diagonal element and jump to the appropriate case
  if ( R[1][1] > R[0][0] )
  {
    if ( R[1][1] > R[2][2] )
    { case_= 1; }
    else
    { case_= 2; }
  }
  else if ( R[2][2] > R[0][0] )
  { case_= 2; }
  else
  { case_= 0; }

  //circular axes swap
  switch(case_)
  {
    case 0: // x y z
      u= 0;
      v= 1;
      w= 2;
    break;
    case 1:  // y z x
      u= 1;
      v= 2;
      w= 0;
    break;
    case 2:  // z x y
      u= 2;
      v= 0;
      w= 1;
    break;
  }

  r = sqrt( 1 + R[u][u] - R[v][v] - R[w][w] );

  if( fabs(r) < 1e-9 ) 
  {
    q[0]= 1.0;
    q[1]= q[2]= q[3]= 0.0;
    return;
  }

  q[u+1] = 0.5*r;
  r = 0.5/r;

  q[0] = (R[w][v] - R[v][w]) * r;
  q[v+1] = (R[u][v] + R[v][u]) * r;
  q[w+1] = (R[w][u] + R[u][w]) * r;

  n= p3d_vect4Norm(q);

 if(isnan(n))
 {
   q[0]= 1.0;
   q[1]= q[2]= q[3]= 0.0;
   return;
 }
  
 q[0]/= n;
 q[1]/= n;
 q[2]/= n;
 q[3]/= n;
}

//! Returns a weighted distance between two homogeneous transform matrices.
//! distance= weightR*(distance between the two quaternions) + weightT*(euclidean distance between the two positions)
//! Note that the distance between the two quaternions is in [0 ; PI]
double p3d_mat4Distance(p3d_matrix4 M1, p3d_matrix4 M2, double weightR, double weightT)
{
  double x;
  p3d_vector3 d;
  p3d_vector4 q1, q2;
  p3d_matrix3 R1, R2;

  p3d_mat4ExtractRotMatrix(M1, R1);
  p3d_mat4ExtractRotMatrix(M2, R2);

  p3d_matrix3_to_quaternion(R1, q1);
  p3d_matrix3_to_quaternion(R2, q2);

  x= q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]; 
  x= MIN(acos(x), acos(-x));

  d[0]= M1[0][3] - M2[0][3];
  d[1]= M1[1][3] - M2[1][3];
  d[2]= M1[2][3] - M2[2][3];

  return ( weightR*x + weightT*p3d_vectNorm(d) );
}

//! Computes a random quaternion.
//! random quaternion: from three points (u1,u2,u3) chosen uniformly at random in [0,1]:
//! h = ( sqrt(1-u1)*sin(2*PI*u2), sqrt(1-u1)*cos(2*PI*u2), sqrt(u1)*sin(2*PI*u3), sqrt(u1)*cos(2*PI*u3) )
void p3d_random_quaternion(p3d_vector4 q)
{
  double u1, u2, u3;

  u1=  p3d_random(0.0, 1.0);
  u2=  p3d_random(0.0, 1.0);
  u3=  p3d_random(0.0, 1.0);

  q[0] =  sqrt(1-u1)*sin(2*M_PI*u2);
  q[1] =  sqrt(1-u1)*cos(2*M_PI*u2);
  q[2] =  sqrt(u1)*sin(2*M_PI*u3);
  q[3] =  sqrt(u1)*cos(2*M_PI*u3);
}

//! Extracts the first column of the rotation matrix of a homogeneous transform matrix.
void p3d_mat4ExtractColumnX(p3d_matrix4 M, p3d_vector3 v)
{
  v[0]= M[0][0];
  v[1]= M[1][0];
  v[2]= M[2][0];
}

//! Extracts the second column of the rotation matrix of a homogeneous transform matrix.
void p3d_mat4ExtractColumnY(p3d_matrix4 M, p3d_vector3 v)
{
  v[0]= M[0][1];
  v[1]= M[1][1];
  v[2]= M[2][1];
}

//! Extracts the third column of the rotation matrix of a homogeneous transform matrix.
void p3d_mat4ExtractColumnZ(p3d_matrix4 M, p3d_vector3 v)
{
  v[0]= M[0][2];
  v[1]= M[1][2];
  v[2]= M[2][2];
}


