#ifndef _p3d_MATRIX_H
#define _p3d_MATRIX_H

typedef double p3d_matrix_type;


typedef p3d_matrix_type p3d_matrix2[2][2];   // modif Juan
typedef p3d_matrix_type p3d_matrix3[3][3];
typedef p3d_matrix_type p3d_matrix4[4][4];
typedef p3d_matrix4 *pp3d_matrix4;

typedef p3d_matrix_type p3d_vector2[2];   // modif Juan
typedef p3d_matrix_type p3d_vector3[3];
typedef p3d_matrix_type p3d_vector4[4];


/*
void p3d_mat4Add(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c);
void p3d_mat4Sub(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c);
void p3d_mat4Mult(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c);
void p3d_mat3Mult(p3d_matrix3 a, p3d_matrix3 b, p3d_matrix3 c);
void p3d_matvec4Mult(p3d_matrix4 a, p3d_vector4 v, p3d_vector4 c);
void p3d_mat4Copy(p3d_matrix4 source, p3d_matrix4 dest);
void p3d_mat3Copy(p3d_matrix3 source, p3d_matrix3 dest);
void p3d_mat4Transpose(p3d_matrix4 source, p3d_matrix4 dest);
void p3d_mat3Transpose(p3d_matrix3 source, p3d_matrix3 dest);
void p3d_mat4Print(p3d_matrix4 M, char *name);
void p3d_mat3Print(p3d_matrix3 M, char *name);
double p3d_mat3Det(p3d_matrix3 mat);
int p3d_mat3Invert(p3d_matrix3 mat, p3d_matrix3 invmat);    <- modif Etienne Ferre
void p3d_vec3Mat3Mult(p3d_matrix3 M, p3d_vector3 a, p3d_vector3 b);    <- modif Etienne Ferre

void p3d_matMultXform(p3d_matrix4 a, p3d_matrix4 b, p3d_matrix4 c);
void p3d_matInvertXform(p3d_matrix4 M, p3d_matrix4 inv);

   inserer par david Brunet
int p3d_matInvertArbitraryXform(p3d_matrix4 M, p3d_matrix4 inv);

void p3d_matBuildXform(char *axes, p3d_matrix_type angles[],
			 p3d_matrix_type dx, p3d_matrix_type dy, p3d_matrix_type dz,
			 p3d_matrix4 M);

void p3d_xformPoint(p3d_matrix4 M, p3d_vector3 p, p3d_vector3 p2);
void p3d_xformVect(p3d_matrix4 M, p3d_vector3 v, p3d_vector3 v2);
void p3d_xform4(p3d_matrix4 M, p3d_vector4 x, p3d_vector4 x2);
void p3d_xform3(p3d_matrix3 M, p3d_vector3 x, p3d_vector3 x2);

void p3d_vectCopy(p3d_vector3 src, p3d_vector3 dest);
void p3d_vectAdd(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c);
void p3d_vectSub(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c);
void p3d_vectNeg(p3d_vector3 src, p3d_vector3 dest);
void p3d_vectScale(p3d_vector3 src, p3d_vector3 dest, p3d_matrix_type k);
void p3d_vectNormalize(p3d_vector3 src, p3d_vector3 dest);

p3d_matrix_type p3d_vectNorm(p3d_vector3 v);
int p3d_vectEqual(p3d_vector3 a, p3d_vector3 b);
p3d_matrix_type p3d_vectDotProd(p3d_vector3 a, p3d_vector3 b);
void p3d_vectXprod(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c);
p3d_matrix_type p3d_planeDist(p3d_vector4 plane, p3d_vector3 point);
void p3d_displacePoint(p3d_vector3 point, p3d_vector3 vect,
			 p3d_matrix_type lambda, p3d_vector3 result);

*/
extern p3d_matrix3 p3d_mat3IDENTITY;
extern p3d_matrix4 p3d_mat4IDENTITY;
extern p3d_matrix4 p3d_mat4NULL;
extern p3d_matrix3 p3d_mat3NULL;

#endif
