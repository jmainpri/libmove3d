#include <math.h>
#include "defs.h"
#include "mathfcts.h"

/**********************************************************************/


void vectCopy(double *src, double *dest)
{
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
} 

void vectAdd(double *a, double *b, double *c)
{
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
} 


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

void vectXprod(double *a, double *b, double *c)
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
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


void vectScale(double *src, double *dest, double k)
{
  dest[0] = k * src[0];
  dest[1] = k * src[1];
  dest[2] = k * src[2];
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


void matMultXform(matrix4 a, matrix4 b, matrix4 c)
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

matrix4 mat4IDENTITY = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

void mat4Rot(matrix4 M, double *axe, double t)
{
  double norm, c, s, v;
  double x, y, z;

  x = axe[0];
  y = axe[1];
  z = axe[2];


  mat4Copy(mat4IDENTITY,M);

  norm = vectNorm(axe);
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

void vectDeltaRot(double *axe, double * t, double *axe1, double *axe2)
{
  double v_sin, v_cos, scale;

  vectXprod(axe1, axe2, axe);
  v_sin = vectNorm(axe);
  v_cos = vectDotProd(axe1, axe2);

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
    scale = vectNorm(axe);
    vectScale(axe, axe, 1/scale);
    if (v_cos>0)
      { *t = 0.0; }
    else
      { *t = M_PI; }
  } else {
    vectScale(axe, axe, 1/v_sin);
    *t = acos(v_cos);
  }
}

void mat4DeltaRot(matrix4 M, double *axe1, double t1, double *axe2, double t2)
{
  matrix4 M1, M2, M3, invM1;
  double axe[3];
  double t;

  /* Inverse first rotation */
  mat4Rot(M1, axe1, t1);
  inverse_transf(M1, invM1);

  /* Rotation between the two axis */
  vectDeltaRot(axe, &t, axe1, axe2);
  mat4Rot(M2, axe, t);
  matMultXform(M2, invM1, M3);

  /* Second rotation */
  mat4Rot(M2, axe2, t2);
  matMultXform(M2, M3, M);
}


/**********************************************************************/

/* Extract the parameters of a position matrix in Moved3D format
   3 translation (Tx,Ty,Tz) and 3 rotations (Rz,Ry,Rx) in this order 
*/
void mat4ExtractPosReverseOrder(matrix4 M, 
				double * Tx, double * Ty, double * Tz, 
				double * Rx, double * Ry, double * Rz)
{
  double cy;
    
  (*Ry) = asin(M[0][2]);
  cy = cos(*Ry);
  if (EQ(cy, 0)) {
    (*Rx) = 0.0;
    if (M[1][0]<0)
      { (*Rz) = - acos(M[1][1]); }
    else
      { (*Rz) =   acos(M[1][1]); }
  } else {
    if (M[0][1] < 0)
      { (*Rz) =   acos(M[0][0] / cy); }
    else
      { (*Rz) = - acos(M[0][0] / cy); }
    if (M[1][2] < 0)
      { (*Rx) =   acos(M[2][2] / cy); }
    else
      { (*Rx) = - acos(M[2][2] / cy); }
  }
  *Tx = M[0][3];
  *Ty = M[1][3];
  *Tz = M[2][3];
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

