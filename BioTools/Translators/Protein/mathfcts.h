#define EPS6 0.000001
#define	FUZZ 0.00001
#define ABS(x)       (((x) >= 0.0) ? (x) : -(x))
#define MIN(x,y)     (((x) < (y)) ? (x) : (y))
#define MAX(x,y)     (((x) > (y)) ? (x) : (y))
#define SQR(x)       ((x)*(x))
#define EQ(x,y)      (ABS((x)-(y)) < FUZZ)

typedef double matrix4[4][4];
typedef double vector4[4];

/**********************************************************************/

extern void vectCopy(double *src, double *dest);
extern void vectAdd(double *a, double *b, double *c);
extern void vectSub(double *a, double *b, double *c);
extern double vectNorm(double *v);
extern void vectNormalize(double *src, double *dest);
extern void vectXprod(double *a, double *b, double *c);
extern void normalized_vectXprod(double *a, double *b, double *c);
extern double vectDotProd(double *a, double *b);
extern int same_sign_vect(double *v1, double *v2);
extern void vectScale(double *src, double *dest, double k);

extern void mat4vec3MultPos(matrix4 a, double *v, vector4 c);
extern void mat4Mult(matrix4 a, matrix4 b, matrix4 c);
extern void matMultXform(matrix4 a, matrix4 b, matrix4 c);
extern void mat4Copy(matrix4 source, matrix4 dest);

extern void inverse_transf(matrix4 M, matrix4 inv);
extern void transf_rotz(double q, matrix4 T);
extern void inv_transf_rotz(double q, matrix4 T);

extern void mat4Rot(matrix4 M, double *axe, double t);
extern void vectDeltaRot(double *axe, double * t, double *axe1, double *axe2);
extern void mat4DeltaRot(matrix4 M, double *axe1, double t1, double *axe2, double t2);
extern void mat4ExtractPosReverseOrder(matrix4 M, 
				       double * Tx, double * Ty, double * Tz, 
				       double * Rx, double * Ry, double * Rz);

extern double compute_dihedang(double *nJa, double *tJa, double *pJa);
extern void compute_frame(double *opos, double *xaxis, double *zaxis, matrix4 T);
extern void compute_dihedang_and_frame(double *a1pos, double *a2pos,double *a3pos, double *a4pos,
				       double *dihedang, matrix4 T);
extern void compute_distance(double *P1, double *P2, double *dist);

