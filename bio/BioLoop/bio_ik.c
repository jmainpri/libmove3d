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
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Bio-pkg.h"


/*******************************************************************************/

static int p3d_obtain_and_set_DH_constant_params(p3d_cntrt *ct, bio_6R_ik_data *ikdata);
static int p3d_obtain_and_set_DH_constant_params_nopep(p3d_cntrt *ct, bio_6R_ik_data *ikdata);
static int bio_set_ik_constant_params(p3d_cntrt *ct, int nopepflg);
static int bio_set_ik_be_frames_transf(p3d_cntrt *ct);

static int bio_compute_params_of_0and7(p3d_cntrt *ct, p3d_matrix4 T0, p3d_matrix4 T6);
static int bio_set_ik_variable_params(p3d_cntrt *ct, p3d_matrix4 T0, p3d_matrix4 T6);

static int bio_compute_ik_solutions(p3d_cntrt *ct, double **sol_configs);
#ifdef USE_GSL
static int bio_compute_ik_solutions_with_GSL(p3d_cntrt *ct, double **sol_configs);
#endif
#ifdef USE_CLAPACK
static int bio_compute_ik_solutions_with_LAPACK(p3d_cntrt *ct, double **sol_configs);

#ifdef __cplusplus
extern "C" {
#endif
  extern int dggev_(char *jobvl, char *jobvr, long int *n, double *a,
    long int *lda, double *b, long int *ldb, double *alphar, 
    double *alphai, double *beta, double *vl, long int *ldvl, 
    double *vr, long int *ldvr, double *work, long int *lwork, 
    long int *info); 
#ifdef __cplusplus
}
#endif
#endif

static void bio_get_T0(p3d_cntrt *ct, p3d_matrix4 T0);
static void bio_get_T6(p3d_cntrt *ct, p3d_matrix4 T6);
static void bio_get_T6_new(p3d_cntrt *ct, p3d_matrix4 T6);

static void bio_compute_coefsIJKL(bio_6R_ik_data *ikd,
				  double *K2cc,double *K2cs,double *K2c1,double *K2sc,double *K2ss,double *K21c,double *K21s,double *K211,
				  double *I3cc,double *I3cs,double *I3c1,double *I3sc,double *I3ss,double *I3s1,double *I31c,double *I31s,double *I311,
				  double *J3cc,double *J3cs,double *J3c1,double *J3sc,double *J3ss,double *J3s1,double *J31c,double *J31s,double *J311,
				  double *K3cc,double *K3cs,double *K3c1,double *K3sc,double *K3ss,double *K3s1,double *K31c,double *K31s,double *K311);

static void bio_compute_M23_N23_elems(double K2cc,double K2cs,double K2c1,double K2sc,double K2ss,double K21c,double K21s,double K211,
				      double I3cc,double I3cs,double I3c1,double I3sc,double I3ss,double I3s1,double I31c,double I31s,double I311,
				      double J3cc,double J3cs,double J3c1,double J3sc,double J3ss,double J3s1,double J31c,double J31s,double J311,
				      double K3cc,double K3cs,double K3c1,double K3sc,double K3ss,double K3s1,double K31c,double K31s,double K311,
				      poly2_coefs m16, poly2_coefs m17, poly2_coefs m18, poly2_coefs m22, poly2_coefs m23, poly2_coefs m24,
				      poly2_coefs m32, poly2_coefs m33, poly2_coefs m34, poly2_coefs m36, poly2_coefs m37, poly2_coefs m38,
				      poly2_coefs m42, poly2_coefs m43, poly2_coefs m44, poly2_coefs m46, poly2_coefs m47, poly2_coefs m48,
				      poly2_coefs n16, poly2_coefs n17, poly2_coefs n18, poly2_coefs n22, poly2_coefs n23, poly2_coefs n24,
				      poly2_coefs n32, poly2_coefs n33, poly2_coefs n34, poly2_coefs n36, poly2_coefs n37, poly2_coefs n38,
				      poly2_coefs n42, poly2_coefs n43, poly2_coefs n44, poly2_coefs n46, poly2_coefs n47, poly2_coefs n48);

static void bio_compute_det_M23orN23(poly2_coefs m16, poly2_coefs m17, poly2_coefs m18, 
				     poly2_coefs m22, poly2_coefs m23, poly2_coefs m24,
				     poly2_coefs m32, poly2_coefs m33, poly2_coefs m34,
				     poly2_coefs m36, poly2_coefs m37, poly2_coefs m38,
				     poly2_coefs m42, poly2_coefs m43, poly2_coefs m44,
				     poly2_coefs m46, poly2_coefs m47, poly2_coefs m48,
				     poly_max16_coefs *deter_coefs);

static void bio_set_couples(poly_max16_coefs *sol1, poly_max16_coefs *sol2,
			    double K2cc,double K2cs,double K2c1,double K2sc,double K2ss,double K21c,double K21s,double K211,
			    double I3cc,double I3cs,double I3c1,double I3sc,double I3ss,double I3s1,double I31c,double I31s,double I311,
			    double J3cc,double J3cs,double J3c1,double J3sc,double J3ss,double J3s1,double J31c,double J31s,double J311,
			    double K3cc,double K3cs,double K3c1,double K3sc,double K3ss,double K3s1,double K31c,double K31s,double K311);

static double bio_compute_ecart_sols(double x1, double x2,
				     double K2cc,double K2cs,double K2c1,double K2sc,double K2ss,double K21c,double K21s,double K211,
				     double I3cc,double I3cs,double I3c1,double I3sc,double I3ss,double I3s1,double I31c,double I31s,double I311,
				     double J3cc,double J3cs,double J3c1,double J3sc,double J3ss,double J3s1,double J31c,double J31s,double J311,
				     double K3cc,double K3cs,double K3c1,double K3sc,double K3ss,double K3s1,double K31c,double K31s,double K311);


static int bio_compute_the_solutions(p3d_cntrt *ct, poly_max16_coefs *xx1, poly_max16_coefs *xx2, double **sol_configs);
static int bio_compute_solution_for_couple(p3d_cntrt *ct, double x1, double x2, double *sol);
static int bio_verify_solution(p3d_cntrt *ct, double C3, double S3, p3d_matrix4 T06, p3d_matrix4 T20, p3d_matrix4 T43, p3d_matrix4 T54, p3d_matrix4 T65);


/* FOR POLYNOMIALS !!! */
#ifdef USE_GSL
static int solve_polynomial_eq(poly_max16_coefs *polcfs, poly_max16_coefs *sol);
#endif
static void print_poly16(poly_max16_coefs *pol);
static void copy_poly16(poly_max16_coefs *in, poly_max16_coefs *out);
static void prod_of_8_poly2(double k, poly2_coefs pol1, poly2_coefs pol2, poly2_coefs pol3, 
			    poly2_coefs pol4, poly2_coefs pol5, poly2_coefs pol6, poly2_coefs pol7,
			    poly2_coefs pol8, poly_max16_coefs *prod_pol);
static void add_2_poly16(poly_max16_coefs *pol1, poly_max16_coefs *pol2);


/*******************************************************************************/
/*******************************************************************************/

static int p3d_obtain_and_set_DH_constant_params(p3d_cntrt *ct, bio_6R_ik_data *ikdata)
{
  /* NOTES :
     - the frame-origin of the jnt[i] is defined at the center of the atom in the
       intersection of jnt[i] and jnt[i+1]

       ### verificar sentido de rotacion !!! (usar artmol_new3) ###  
  */

  int i;
  p3d_jnt *jntPt1, *jntPt2;
  double r[5],al[5];
  p3d_vector3 pos_jnt1,pos_jnt2,pos_diff,axe_jnt1,axe_jnt2;

  for(i=0; i<5; i++) {
    jntPt1 = ct->pasjnts[i];
    jntPt2 = ct->pasjnts[i+1];
    p3d_jnt_get_cur_vect_point(jntPt1,pos_jnt1);
    p3d_jnt_get_cur_vect_point(jntPt2,pos_jnt2);
    p3d_jnt_get_dof_cur_axis(jntPt1,0,axe_jnt1);
    p3d_jnt_get_dof_cur_axis(jntPt2,0,axe_jnt2);
    
    p3d_vectSub(pos_jnt2,pos_jnt1,pos_diff);
    r[i] = (double) p3d_vectNorm(pos_diff);

    al[i] = acos(p3d_vectDotProd(axe_jnt1,axe_jnt2)/(p3d_vectNorm(axe_jnt1)*p3d_vectNorm(axe_jnt2)));
  }
    
  ikdata->r2 = r[0];
  ikdata->r3 = r[1];
  ikdata->r4 = r[2];
  ikdata->r5 = r[3];
  ikdata->r6 = r[4];         // NOTE : r6 is not used in the IK method, but it is required next
  ikdata->al1 = al[0];
  ikdata->al2 = al[1];
  ikdata->al3 = al[2];
  ikdata->al4 = al[3];
  ikdata->al5 = al[4];

  return 1;
}

/*******************************************************************************/

/* WARNING : this function must be called with the peptide angles set at 180 */

static int p3d_obtain_and_set_DH_constant_params_nopep(p3d_cntrt *ct, bio_6R_ik_data *ikdata)
{
  /* NOTES :
     - the frame-origin of the jnt[i] is defined at the center of the atom in the
       intersection of jnt[i] and jnt[i+1]

       ### verificar sentido de rotacion !!! (usar artmol_new2) ###  
  */

  p3d_jnt *jntPt1, *jntPt2;
  double r[5],al[5];
  p3d_vector3 pos_jnt1,pos_jnt2,pos_diff,axe_jnt1,axe_jnt2,intpnt;
  p3d_vector3 vc, vp;
  p3d_matrix_type rescop;

  jntPt1 = ct->pasjnts[0];
  jntPt2 = ct->pasjnts[1]; 
  p3d_jnt_get_dof_cur_axis(jntPt1,0,axe_jnt1);
  p3d_jnt_get_cur_vect_point(jntPt1,pos_jnt2);
  p3d_jnt_get_dof_cur_axis(jntPt2,0,axe_jnt2);
  al[0] = acos(p3d_vectDotProd(axe_jnt1,axe_jnt2)/(p3d_vectNorm(axe_jnt1)*p3d_vectNorm(axe_jnt2)));
  jntPt1 = ct->pasjnts[3];
  p3d_jnt_get_dof_cur_axis(jntPt1,0,axe_jnt1);
  p3d_jnt_get_cur_vect_point(jntPt1,pos_jnt1); 
  al[1] = acos(p3d_vectDotProd(axe_jnt1,axe_jnt2)/(p3d_vectNorm(axe_jnt1)*p3d_vectNorm(axe_jnt2)));
  // test coplanarity
  p3d_vectSub(pos_jnt2, pos_jnt1, vc);
  p3d_vectXprod(axe_jnt1,axe_jnt2,vp);
  rescop = p3d_vectDotProd(vc,vp);  
  if(fabs(rescop) > BIO_IK_ERROR_TOL) {
    printf("ERROR : not coplanar lines when computing DH params\n");
    return 0;
  }
  // intersection  
  p3d_lineInt(pos_jnt2,axe_jnt2,pos_jnt1,axe_jnt1,intpnt);
  p3d_vectSub(intpnt,pos_jnt2,pos_diff);
  r[0] = (double) p3d_vectNorm(pos_diff);
  if(!p3d_same_sign_vect(pos_diff,axe_jnt2))
    r[0] = - r[0]; 
  p3d_vectSub(pos_jnt1,intpnt,pos_diff);
  r[1] = (double) p3d_vectNorm(pos_diff);
  if(!p3d_same_sign_vect(pos_diff,axe_jnt1))
    r[1] = - r[1]; 
  jntPt1 = ct->pasjnts[4];
  p3d_jnt_get_dof_cur_axis(jntPt1,0,axe_jnt2);
  al[2] = acos(p3d_vectDotProd(axe_jnt1,axe_jnt2)/(p3d_vectNorm(axe_jnt1)*p3d_vectNorm(axe_jnt2)));
  jntPt2 = ct->pasjnts[6];
  p3d_jnt_get_dof_cur_axis(jntPt2,0,axe_jnt1);
  p3d_jnt_get_cur_vect_point(jntPt2,pos_jnt2);
  al[3] = acos(p3d_vectDotProd(axe_jnt2,axe_jnt1)/(p3d_vectNorm(axe_jnt1)*p3d_vectNorm(axe_jnt2)));
  // test coplanarity
  p3d_vectSub(pos_jnt2, pos_jnt1, vc);
  p3d_vectXprod(axe_jnt2,axe_jnt1,vp);
  rescop = p3d_vectDotProd(vc,vp);  
  if(fabs(rescop) > BIO_IK_ERROR_TOL) {
    printf("ERROR : not coplanar lines when computing DH params\n");
    return 0;
  }
  // intersection  
  p3d_lineInt(pos_jnt1,axe_jnt2,pos_jnt2,axe_jnt1,intpnt);
  p3d_vectSub(intpnt,pos_jnt1,pos_diff);
  r[2] = (double) p3d_vectNorm(pos_diff);
  if(!p3d_same_sign_vect(pos_diff,axe_jnt2))
    r[2] = - r[2]; 
  p3d_vectSub(pos_jnt2,intpnt,pos_diff);
  r[3] = (double) p3d_vectNorm(pos_diff);
   if(!p3d_same_sign_vect(pos_diff,axe_jnt1))
    r[3] = - r[3];  
  jntPt1 = ct->pasjnts[7];
  p3d_jnt_get_dof_cur_axis(jntPt1,0,axe_jnt2);
  p3d_jnt_get_cur_vect_point(jntPt1,pos_jnt1);
  al[4] = acos(p3d_vectDotProd(axe_jnt1,axe_jnt2)/(p3d_vectNorm(axe_jnt1)*p3d_vectNorm(axe_jnt2)));
  p3d_vectSub(pos_jnt2,pos_jnt1,pos_diff);
  r[4] = (double) p3d_vectNorm(pos_diff);
    
  ikdata->r2 = r[0];
  ikdata->r3 = r[1];
  ikdata->r4 = r[2];
  ikdata->r5 = r[3];
  ikdata->r6 = r[4];         // NOTE : r6 is not used in the IK method, but it is required next
  ikdata->al1 = al[0];
  ikdata->al2 = al[1];
  ikdata->al3 = al[2];
  ikdata->al4 = al[3];
  ikdata->al5 = al[4];

  printf("r1 = %f\n",r[0]);
  printf("r2 = %f\n",r[1]);
  printf("r3 = %f\n",r[2]);
  printf("r4 = %f\n",r[3]);
  printf("r5 = %f\n",r[4]);
  printf("al1 = %f\n",al[0]);
  printf("al2 = %f\n",al[1]);
  printf("al3 = %f\n",al[2]);
  printf("al4 = %f\n",al[3]);
  printf("al5 = %f\n",al[4]);

  return 1;
}


/*******************************************************************************/

static int bio_set_ik_constant_params(p3d_cntrt *ct, int nopepflg)
{
  bio_6R_ik_data *ikdata;

  ikdata = MY_ALLOC(bio_6R_ik_data, 1);

  if(nopepflg) {
    if(!p3d_obtain_and_set_DH_constant_params_nopep(ct,ikdata))
      return 0;
  }
  else
    p3d_obtain_and_set_DH_constant_params(ct,ikdata);

  ikdata->sal1 = sin(ikdata->al1);
  ikdata->cal1 = cos(ikdata->al1);
  ikdata->sal2 = sin(ikdata->al2);
  ikdata->cal2 = cos(ikdata->al2);
  ikdata->sal3 = sin(ikdata->al3);
  ikdata->cal3 = cos(ikdata->al3);
  ikdata->sal4 = sin(ikdata->al4);
  ikdata->cal4 = cos(ikdata->al4);
  ikdata->sal5 = sin(ikdata->al5);
  ikdata->cal5 = cos(ikdata->al5);

  ikdata->cal1sal2 = ikdata->cal1 * ikdata->sal2;
  ikdata->sal1sal2 = ikdata->sal1 * ikdata->sal2;
  ikdata->cal1cal2 = ikdata->cal1 * ikdata->cal2;
  ikdata->sal1cal2 = ikdata->sal1 * ikdata->cal2;
    
  ikdata->r2sal1 = ikdata->r2 * ikdata->sal1;
  ikdata->r2cal1 = ikdata->r2 * ikdata->cal1;
  ikdata->r2sal2 = ikdata->r2 * ikdata->sal2;
  ikdata->r2cal2 = ikdata->r2 * ikdata->cal2;
  ikdata->sqr_r2_05 = SQR(ikdata->r2) / 2.0;
  
  ikdata->r5sal4 = ikdata->r5 * ikdata->sal4;
  ikdata->r5cal4 = ikdata->r5 * ikdata->cal4;
  ikdata->cal3sal4 = ikdata->cal3 * ikdata->sal4;
  ikdata->sal3sal4 = ikdata->sal3 * ikdata->sal4;
  ikdata->sal3cal4 = ikdata->sal3 * ikdata->cal4;
  ikdata->cal3cal4 = ikdata->cal3 * ikdata->cal4;
  ikdata->r4cal3sal4 = ikdata->r4 * ikdata->cal3sal4;
  ikdata->r4sal3sal4 = ikdata->r4 * ikdata->sal3sal4;
  ikdata->cal5sal5 =  ikdata->cal5 * ikdata->sal5;
  ikdata->sqrr4 =  SQR(ikdata->r4);
  ikdata->sqrr5 =  SQR(ikdata->r5);
  ikdata->sqrsal5 =  SQR(ikdata->sal5);

  ikdata->coef45_1 = ((ikdata->sqrr4 - ikdata->sqrr5) * ikdata->sal4) / 2.0;
  ikdata->coef543_1 = ikdata->sal5 * ikdata->r4 * SQR(ikdata->sal3) * ikdata->sal4;
  ikdata->coef543_2 = ikdata->coef543_1 * (((ikdata->sqrr4 + ikdata->sqrr5) / 2.0 
					    + ikdata->r4 * ikdata->r5cal4));

  ct->bio_ik_data = ikdata;

  return 1;  
}


/*******************************************************************************/

static int bio_set_ik_be_frames_transf(p3d_cntrt *ct)
{

  /* NOTES FOR BASE-FRAME :
     - frame0 is frame1 rotated of -q1 around z1 
       -> z0 and x0 are computed at every call to de ik_solver 
  */
  
  /* NOTES FOR END-FRAME :
     - a fictive jnt is considered at the end-atom (next to jnt6)
     - TE6 = TTrans(-rE,zE) * TRot(-alE,xE) * TTrans(-r6,z6)
  */

  p3d_jnt *jnt6Pt, *jntEPt;
  double rE,alE;
  p3d_vector3 pos_jnt6,pos_jntE,pos_diff,axe_jnt6,axe_jntE;  
  
  if(ct->npasjnts < 8)
    jnt6Pt = ct->pasjnts[5];
  else
    jnt6Pt = ct->pasjnts[7];
  jntEPt = jnt6Pt->next_jnt[0];
  p3d_jnt_get_cur_vect_point(jnt6Pt,pos_jnt6);
  p3d_jnt_get_cur_vect_point(jntEPt,pos_jntE);
  p3d_jnt_get_dof_cur_axis(jnt6Pt,0,axe_jnt6);
  p3d_jnt_get_dof_cur_axis(jntEPt,0,axe_jntE);
    
  p3d_vectSub(pos_jntE,pos_jnt6,pos_diff);
  rE = (double) p3d_vectNorm(pos_diff);
  alE = acos(p3d_vectDotProd(axe_jnt6,axe_jntE)/(p3d_vectNorm(axe_jnt6)*p3d_vectNorm(axe_jntE)));  
  /* required by RLG to compute max min extension */
  ct->bio_ik_data->rE = rE;
  ct->bio_ik_data->alE = alE;

  /* TE6 ( -> Tatt) */
  ct->Tatt[0][0] = 1.0;
  ct->Tatt[0][1] = 0.0;
  ct->Tatt[0][2] = 0.0;
  ct->Tatt[0][3] = 0.0;
  ct->Tatt[1][0] = 0.0;
  ct->Tatt[1][1] = cos(-alE);
  ct->Tatt[1][2] = -sin(-alE);
  ct->Tatt[1][3] = sin(-alE) * ct->bio_ik_data->r6;
  ct->Tatt[2][0] = 0.0;
  ct->Tatt[2][1] = sin(-alE);
  ct->Tatt[2][2] = cos(-alE);
  ct->Tatt[2][3] = (-cos(-alE) * ct->bio_ik_data->r6) - rE;
  ct->Tatt[3][0] = 0.0;
  ct->Tatt[3][1] = 0.0;
  ct->Tatt[3][2] = 0.0;
  ct->Tatt[3][3] = 1.0;
  
  return 1;
}


int bio_set_ik(p3d_cntrt *ct)
{
  bio_set_ik_constant_params(ct,0);
  bio_set_ik_be_frames_transf(ct);

  return 1;
}

/*******************************************************************************/

int bio_set_ik_nopep(p3d_cntrt *ct)
{
  if(!bio_set_ik_constant_params(ct,1))
    return 0;
  bio_set_ik_be_frames_transf(ct);

  return 1;
}


/*******************************************************************************/
/*******************************************************************************/

static int bio_compute_params_of_0and7(p3d_cntrt *ct, p3d_matrix4 T0, p3d_matrix4 T6)
{
  p3d_matrix4 T0f,iT6,T60,Tr0;
  p3d_vector3 z0,x0,y6,x7,x7Xx0;
  p3d_matrix2 A;
  p3d_vector2 B,unk;
  double cth7,sth7,cal7,sal7;
  double r7,a7,r0,th7,al7,th1_i;
  

  x0[0] = T0[0][0];
  x0[1] = T0[1][0];
  x0[2] = T0[2][0];
  z0[0] = T0[0][2];
  z0[1] = T0[1][2];
  z0[2] = T0[2][2];
  y6[0] = T6[0][1];
  y6[1] = T6[1][1];
  y6[2] = T6[2][1];
  p3d_vectXprod(y6,z0,x7);

  /* compute theta1 initial */
  th1_i = acos(p3d_vectDotProd(x7,x0)/(p3d_vectNorm(x7)*p3d_vectNorm(x0)));
  p3d_vectXprod(x7,x0,x7Xx0);
  if(!p3d_same_sign_vect(x7Xx0,z0))
    th1_i = -th1_i;  

  /* rotation of -theta1 around z0 */
  Tr0[0][0] = cos(-th1_i);
  Tr0[0][1] = -sin(-th1_i);
  Tr0[0][2] = 0.0;
  Tr0[0][3] = 0.0;
  Tr0[1][0] = sin(-th1_i);
  Tr0[1][1] = cos(-th1_i);
  Tr0[1][2] = 0.0;
  Tr0[1][3] = 0.0;
  Tr0[2][0] = 0.0;
  Tr0[2][1] = 0.0;
  Tr0[2][2] = 1.0;
  Tr0[2][3] = 0.0;
  Tr0[3][0] = 0.0;
  Tr0[3][1] = 0.0;
  Tr0[3][2] = 0.0;
  Tr0[3][3] = 1.0;  

  /* transformations */
  p3d_matMultXform(T0,Tr0,T0f);
  p3d_matInvertXform(T6,iT6);
  p3d_matMultXform(iT6,T0f,T60);
  
  /* theta7 */
  cth7 = T60[0][0];
  sth7 = - T60[2][0];
  th7 = acos(cth7);
  if((sin(th7) - sth7) > BIO_IK_ERROR_TOL)
    th7 = -th7;
  
  /* alpha7 */
  cal7 = T60[1][2];
  sal7 = T60[1][1];
  al7 = acos(cal7);
  if((sin(al7) - sal7) > BIO_IK_ERROR_TOL)
    al7 = -al7; 
  
  /* a7 and r0 */
  A[0][0] = cth7;
  A[0][1] = sth7 * sal7;
  A[1][0] = -sth7;
  A[1][1] = cth7 * sal7;
  B[0] = T60[0][3];
  B[1] = T60[2][3];
  p3d_solve_eqsys_2(A,B,unk);
  a7 = unk[0];
  r0 = unk[1];
  
  /* r7 */
  r7 = T60[1][3] - (r0 * cal7);

  /* store variables */
  ct->bio_ik_data->r7 = r7;
  ct->bio_ik_data->a7 = a7;
  ct->bio_ik_data->r0 = r0;
  ct->bio_ik_data->th7 = th7;
  ct->bio_ik_data->al7 = al7;

  ct->bio_ik_data->cth7 = cth7;
  ct->bio_ik_data->sth7 = sth7;
  ct->bio_ik_data->cal7 = cal7;
  ct->bio_ik_data->sal7 = sal7;

  ct->bio_ik_data->th1_i = th1_i;

  return 1;
}


/*******************************************************************************/

static int bio_set_ik_variable_params(p3d_cntrt *ct, p3d_matrix4 T0, p3d_matrix4 T6)
{

  if(!bio_compute_params_of_0and7(ct,T0,T6)) {
    return 0;
  }
  
/*   ct->bio_ik_data->sal7 = sin(ct->bio_ik_data->al7); */
/*   ct->bio_ik_data->cal7 = cos(ct->bio_ik_data->al7); */
/*   ct->bio_ik_data->sth7 = sin(ct->bio_ik_data->th7); */
/*   ct->bio_ik_data->cth7 = cos(ct->bio_ik_data->th7); */

  ct->bio_ik_data->cth7sal7 = ct->bio_ik_data->cth7 * ct->bio_ik_data->sal7;
  ct->bio_ik_data->sth7sal7 = ct->bio_ik_data->sth7 * ct->bio_ik_data->sal7;
  ct->bio_ik_data->cth7cal7 = ct->bio_ik_data->cth7 * ct->bio_ik_data->cal7;
  ct->bio_ik_data->sth7cal7 = ct->bio_ik_data->sth7 * ct->bio_ik_data->cal7;
  
  ct->bio_ik_data->coef70_1 = (ct->bio_ik_data->a7 * ct->bio_ik_data->cth7) 
                              + (ct->bio_ik_data->r0 * ct->bio_ik_data->sth7sal7);
  ct->bio_ik_data->coef70_2 = (ct->bio_ik_data->a7 * ct->bio_ik_data->sth7) 
                              - (ct->bio_ik_data->r0 * ct->bio_ik_data->cth7sal7);
  ct->bio_ik_data->coef70_3 = ct->bio_ik_data->r7 + (ct->bio_ik_data->r0 * ct->bio_ik_data->cal7);
  
  ct->bio_ik_data->coef730_1 = (SQR(ct->bio_ik_data->a7) + SQR(ct->bio_ik_data->r0) 
				+ SQR(ct->bio_ik_data->r7) + SQR(ct->bio_ik_data->r3) 
				+ 2.0 * ct->bio_ik_data->r7 * ct->bio_ik_data->r0 * ct->bio_ik_data->cal7)/2.0;
  ct->bio_ik_data->coef730_2 = (SQR(ct->bio_ik_data->a7) + SQR(ct->bio_ik_data->r0) 
				+ SQR(ct->bio_ik_data->r7) - SQR(ct->bio_ik_data->r3) 
				+ 2.0 * ct->bio_ik_data->r7 * ct->bio_ik_data->r0 * ct->bio_ik_data->cal7)/2.0;

  return 1;
}


/*******************************************************************************/
/*******************************************************************************/

int bio_compute_ik(p3d_cntrt *ct, double **sol_configs) 
{
  int nsol;
  p3d_matrix4 T0, T6;
/*   int i; */

  /* WARNING : we suppose that the jnts have been update !!! */

  /* compute T0 */
  bio_get_T0(ct,T0);
/*   p3d_mat4Print(T0,"T0"); */

  /* compute T6 */
  bio_get_T6(ct,T6);
/*   p3d_mat4Print(T6,"T6"); */

  if(!bio_set_ik_variable_params(ct,T0,T6)) {
    return 0;
  }
  
  nsol = bio_compute_ik_solutions(ct,sol_configs);

  // prints
/*   if(nsol) { */
/*     printf("\n##PARAMETRES##\n"); */
/*     printf("al1 = %f\n",ct->bio_ik_data->al1); */
/*     printf("al2 = %f\n",ct->bio_ik_data->al2); */
/*     printf("al3 = %f\n",ct->bio_ik_data->al3); */
/*     printf("al4 = %f\n",ct->bio_ik_data->al4); */
/*     printf("al5 = %f\n",ct->bio_ik_data->al5); */
/*     printf("r2 = %f\n",ct->bio_ik_data->r2); */
/*     printf("r3 = %f\n",ct->bio_ik_data->r3); */
/*     printf("r4 = %f\n",ct->bio_ik_data->r4); */
/*     printf("r5 = %f\n",ct->bio_ik_data->r5); */
/*     printf("r7 = %f\n",ct->bio_ik_data->r7); */
/*     printf("a7 = %f\n",ct->bio_ik_data->a7); */
/*     printf("r0 = %f\n",ct->bio_ik_data->r0); */
/*     printf("al7 = %f\n",ct->bio_ik_data->al7); */
/*     printf("th7 = %f\n",ct->bio_ik_data->th7); */
/*     printf("\n##RESULTS##\n"); */
/*     for(i=0; i<nsol; i++) { */
/*       printf("solution %d\n",i); */
/*       printf("q1 = %f\n",sol_configs[i][0]); */
/*       printf("q2 = %f\n",sol_configs[i][1]); */
/*       printf("q3 = %f\n",sol_configs[i][2]); */
/*       printf("q4 = %f\n",sol_configs[i][3]); */
/*       printf("q5 = %f\n",sol_configs[i][4]); */
/*       printf("q6 = %f\n",sol_configs[i][5]); */
/*     } */
/*  } */

  return (nsol);
}


int bio_compute_ik_nopep(p3d_cntrt *ct, double **sol_configs) 
{
  int nsol;
  p3d_matrix4 T0, T6;
  int i,j;

  /* WARNING : we suppose that the jnts have been update !!! */

  /* compute T0 */
  bio_get_T0(ct,T0);
/*   p3d_mat4Print(T0,"T0"); */

  /* compute T6 */
  bio_get_T6(ct,T6);
/*   p3d_mat4Print(T6,"T6"); */

  if(!bio_set_ik_variable_params(ct,T0,T6)) {
    return 0;
  }
  
  nsol = bio_compute_ik_solutions(ct,sol_configs);

  /* some frames are rotated of Pi in realtion to DH definition
     therefore the solution must be corrected */
  /* OJO! : solucion un poco chapuza !!!*/
  for(i=0; i<nsol; i++) {
    if(ct->bio_ik_data->r2 < 0) 
      j = 2;
    else
      j = 1;
    sol_configs[i][j] += M_PI;
    if(sol_configs[i][j] > M_PI)
      sol_configs[i][j] -= 2.0*M_PI;
    if(ct->bio_ik_data->r4 < 0) 
      j = 4;
    else
      j = 3;
    sol_configs[i][j] += M_PI;
    if(sol_configs[i][j] > M_PI)
      sol_configs[i][j] -= 2.0*M_PI;
  }

  return (nsol);
}


int bio_compute_ik_nopep_new(p3d_cntrt *ct, double **sol_configs) 
{
  int nsol;
  p3d_matrix4 T0, T6;
  int i,j;

  /* WARNING : we suppose that the jnts have been update !!! */

  /* compute T0 */
  bio_get_T0(ct,T0);
/*   p3d_mat4Print(T0,"T0"); */

  /* compute T6 */
  bio_get_T6_new(ct,T6);
/*   p3d_mat4Print(T6,"T6"); */

  if(!bio_set_ik_variable_params(ct,T0,T6)) {
    return 0;
  }
  
  nsol = bio_compute_ik_solutions(ct,sol_configs);

  /* some frames are rotated of Pi in realtion to DH definition
     therefore the solution must be corrected */
  /* OJO! : solucion un poco chapuza !!!*/
  for(i=0; i<nsol; i++) {
    if(ct->bio_ik_data->r2 < 0) 
      j = 2;
    else
      j = 1;
    sol_configs[i][j] += M_PI;
    if(sol_configs[i][j] > M_PI)
      sol_configs[i][j] -= 2.0*M_PI;
    if(ct->bio_ik_data->r4 < 0) 
      j = 4;
    else
      j = 3;
    sol_configs[i][j] += M_PI;
    if(sol_configs[i][j] > M_PI)
      sol_configs[i][j] -= 2.0*M_PI;
  }

  return (nsol);
}


/*******************************************************************************/

static void bio_get_T0(p3d_cntrt *ct, p3d_matrix4 T0)
{
  p3d_vector3 x0,y0,z0,posO0,z2,x1;
  double theta1;
  
  /* NOTE : posO0 == posO1 , z0 == z1 */
  p3d_jnt_get_cur_vect_point(ct->pasjnts[0],posO0);
  p3d_jnt_get_dof_cur_axis(ct->pasjnts[0],0,z0);  

  p3d_jnt_get_dof_cur_axis(ct->pasjnts[1],0,z2);  
  p3d_vectXprod(z0,z2,y0);
  p3d_vectNormalize(y0,x1);
  theta1 = p3d_jnt_get_dof(ct->pasjnts[0],0);
  p3d_rotate_vect_around_axis(z0,-theta1,x1,x0); 
  
  p3d_vectXprod(z0,x0,z2);
  p3d_vectNormalize(z2,y0);

  T0[0][0] = x0[0];  T0[0][1] = y0[0];  T0[0][2] = z0[0];  T0[0][3] = posO0[0];
  T0[1][0] = x0[1];  T0[1][1] = y0[1];  T0[1][2] = z0[1];  T0[1][3] = posO0[1];
  T0[2][0] = x0[2];  T0[2][1] = y0[2];  T0[2][2] = z0[2];  T0[2][3] = posO0[2];
  T0[3][0] = 0.0;    T0[3][1] = 0.0;    T0[3][2] = 0.0;    T0[3][3] = 1.0; 

}

/*******************************************************************************/

static void bio_get_T6(p3d_cntrt *ct, p3d_matrix4 T6)
{

  /* NOTA : POR EL MOMENTO TOMO LOS EJES Z Y X "TAL CUAL" DEL ACT_JNT 
            PERO EN LA APLICACION FINAL HABRA QUE XALCULARLOS A PARTIR 
            DEL BACKBONE DESPUES DEL BUCLE (O DE OTROS DATOS DE ENTRADA 
	    DEL BUCLE -> NOTAS DE BATES
  */

  p3d_mat4Mult(ct->actjnts[0]->abs_pos,ct->Tatt,T6);

}


static void bio_get_T6_new(p3d_cntrt *ct, p3d_matrix4 T6)
{
  p3d_matrix4 Fabs,Frel;

  Frel[0][0] = ct->argu_d[0]; Frel[0][1] = ct->argu_d[3]; Frel[0][2] = ct->argu_d[6]; Frel[0][3] = ct->argu_d[9]; 
  Frel[1][0] = ct->argu_d[1]; Frel[1][1] = ct->argu_d[4]; Frel[1][2] = ct->argu_d[7]; Frel[1][3] = ct->argu_d[10]; 
  Frel[2][0] = ct->argu_d[2]; Frel[2][1] = ct->argu_d[5]; Frel[2][2] = ct->argu_d[8]; Frel[2][3] = ct->argu_d[11]; 
  Frel[3][0] = 0.0;           Frel[3][1] = 0.0;           Frel[3][2] = 0.0;           Frel[3][3] = 1.0; 

  p3d_mat4Mult(ct->actjnts[0]->abs_pos,Frel,Fabs);
  p3d_mat4Mult(Fabs,ct->Tatt,T6);

}


/*******************************************************************************/

/* NOTE : sol_configs must be an array 16 of arrays 6 of double !!! */
static int bio_compute_ik_solutions(p3d_cntrt *ct, double **sol_configs)
{
#ifdef USE_GSL
#ifndef USE_CLAPACK
  return(bio_compute_ik_solutions_with_GSL(ct,sol_configs));
#endif
#endif
 
#ifdef USE_CLAPACK
#ifdef USE_GSL
  printf("WARNING : GSL and CLAPACK are both linked : only using CLAPACK in bio_compute_ik_solutions\n");
#endif
  return(bio_compute_ik_solutions_with_LAPACK(ct,sol_configs));
#endif

  return 0;
}


#ifdef USE_CLAPACK
static int bio_compute_ik_solutions_with_LAPACK(p3d_cntrt *ct, double **sol_configs)
{
  /* coefsIJKL */
  double K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211;
/*   double L2cc,L2cs,L2c1,L2sc,L2ss,L21c,L21s,L211; */
  double I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311;
  double J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311;
  double K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311;
/*   double L3cc,L3cs,L3c1,L3sc,L3ss,L3s1,L31c,L31s,L311; */
  /* matrix elems */
  poly2_coefs M23_16,M23_17,M23_18,M23_22,M23_23,M23_24,M23_32,M23_33,M23_34,M23_36,M23_37,M23_38,M23_42,M23_43,M23_44,M23_46,M23_47,M23_48;
  poly2_coefs N23_16,N23_17,N23_18,N23_22,N23_23,N23_24,N23_32,N23_33,N23_34,N23_36,N23_37,N23_38,N23_42,N23_43,N23_44,N23_46,N23_47,N23_48;

  // for LAPACK
  char jobvl = 'N', jobvr = 'N';
  long int n = 16;
  double A[256], B[256];
  long int lda = 16, ldb = 16;
  double vl[256], vr[256];
  long int ldvl = 1, ldvr = 1;
  double alphar[16],alphai[16],beta[16];
  long int lwork = 1136;
  double work[1136];
  long int info;
  /* x1 and x2 solution vectors */
  poly_max16_coefs *xx1,*xx2;

  int nsol;
  bio_6R_ik_data *ikd = ct->bio_ik_data;
  int i,j;


  bio_compute_coefsIJKL(ikd,&K2cc,&K2cs,&K2c1,&K2sc,&K2ss,&K21c,&K21s,&K211,
			&I3cc,&I3cs,&I3c1,&I3sc,&I3ss,&I3s1,&I31c,&I31s,&I311,
			&J3cc,&J3cs,&J3c1,&J3sc,&J3ss,&J3s1,&J31c,&J31s,&J311,
			&K3cc,&K3cs,&K3c1,&K3sc,&K3ss,&K3s1,&K31c,&K31s,&K311);

  bio_compute_M23_N23_elems(K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211,
			    I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311,
			    J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311,
			    K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311,
			    M23_16,M23_17,M23_18,M23_22,M23_23,M23_24,M23_32,M23_33,M23_34,
			    M23_36,M23_37,M23_38,M23_42,M23_43,M23_44,M23_46,M23_47,M23_48,
			    N23_16,N23_17,N23_18,N23_22,N23_23,N23_24,N23_32,N23_33,N23_34,
			    N23_36,N23_37,N23_38,N23_42,N23_43,N23_44,N23_46,N23_47,N23_48);


  // solve xx1
  xx1 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));  

  /* matrix A */
  for(i=0; i<256; i++)
    A[i] = 0.0; 
  
  for(i=0; i<8; i++)
    A[128+(16*i)+i] = 1.0;

  for(i=0; i<2; i++) {
    for(j=0; j<2; j++) {
      A[12+(j*12)+(i*128)] =        0.0; A[28+(j*12)+(i*128)] =        0.0; A[44+(j*12)+(i*128)]  =        0.0; 
      A[76+(j*12)+(i*128)] = -M23_16[i]; A[92+(j*12)+(i*128)] = -M23_17[i]; A[108+(j*12)+(i*128)] = -M23_18[i];  
      A[13+(j*12)+(i*128)] = -M23_22[i]; A[29+(j*12)+(i*128)] = -M23_23[i]; A[45+(j*12)+(i*128)]  = -M23_24[i];
      A[77+(j*12)+(i*128)] =        0.0; A[93+(j*12)+(i*128)] =        0.0; A[109+(j*12)+(i*128)] =        0.0;  
      A[14+(j*12)+(i*128)] = -M23_32[i]; A[30+(j*12)+(i*128)] = -M23_33[i]; A[46+(j*12)+(i*128)]  = -M23_34[i];
      A[78+(j*12)+(i*128)] = -M23_36[i]; A[94+(j*12)+(i*128)] = -M23_37[i]; A[110+(j*12)+(i*128)] = -M23_38[i];  
      A[15+(j*12)+(i*128)] = -M23_42[i]; A[31+(j*12)+(i*128)] = -M23_43[i]; A[47+(j*12)+(i*128)]  = -M23_44[i];
      A[79+(j*12)+(i*128)] = -M23_46[i]; A[95+(j*12)+(i*128)] = -M23_47[i]; A[111+(j*12)+(i*128)] = -M23_48[i];  
    }
  }      

  /* matrix B */
  for(i=0; i<256; i++)
      B[i] = 0.0; 

  for(i=0; i<8; i++)
    B[(16*i)+i] = 1.0;

  for(i=0; i<2; i++) {
    B[140+(i*12)] =       0.0; B[156+(i*12)] =       0.0; B[172+(i*12)] =       0.0; 
    B[204+(i*12)] = M23_16[2]; B[220+(i*12)] = M23_17[2]; B[236+(i*12)] = M23_18[2];  
    B[141+(i*12)] = M23_22[2]; B[157+(i*12)] = M23_23[2]; B[173+(i*12)] = M23_24[2];
    B[205+(i*12)] =       0.0; B[221+(i*12)] =       0.0; B[237+(i*12)] =       0.0;  
    B[142+(i*12)] = M23_32[2]; B[158+(i*12)] = M23_33[2]; B[174+(i*12)] = M23_34[2];
    B[206+(i*12)] = M23_36[2]; B[222+(i*12)] = M23_37[2]; B[238+(i*12)] = M23_38[2];  
    B[143+(i*12)] = M23_42[2]; B[159+(i*12)] = M23_43[2]; B[175+(i*12)] = M23_44[2];
    B[207+(i*12)] = M23_46[2]; B[223+(i*12)] = M23_47[2]; B[239+(i*12)] = M23_48[2];  
  }

  /* LAPACK */
  /* int dggev_(char *jobvl, char *jobvr, integer *n, doublereal *a,
                integer *lda, doublereal *b, integer *ldb, doublereal *alphar, 
		doublereal *alphai, doublereal *beta, doublereal *vl, integer *ldvl, 
		doublereal *vr, integer *ldvr, doublereal *work, integer *lwork, 
		integer *info); 
  */
  dggev_(&jobvl,&jobvr,&n,A,&lda,B,&ldb,alphar,alphai,beta,vl,&ldvl,vr,&ldvr,work,&lwork,&info); 

  nsol = 0;
  for(i=0; i<16; i++) {
    if(alphai[i] == 0.0) {  // ??? +- EPS ???
      xx1->a_i[nsol] = alphar[i]/beta[i];
      nsol++; 
    }
  }
  xx1->order = nsol - 1;      
    

  // solve xx2
  xx2 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));  
  
  /* matrix A */
  for(i=0; i<256; i++)
    A[i] = 0.0; 
  
  for(i=0; i<8; i++)
    A[128+(16*i)+i] = 1.0;

  for(i=0; i<2; i++) {
    for(j=0; j<2; j++) {
      A[12+(j*12)+(i*128)] =        0.0; A[28+(j*12)+(i*128)] =        0.0; A[44+(j*12)+(i*128)]  =        0.0; 
      A[76+(j*12)+(i*128)] = -N23_16[i]; A[92+(j*12)+(i*128)] = -N23_17[i]; A[108+(j*12)+(i*128)] = -N23_18[i];  
      A[13+(j*12)+(i*128)] = -N23_22[i]; A[29+(j*12)+(i*128)] = -N23_23[i]; A[45+(j*12)+(i*128)]  = -N23_24[i];
      A[77+(j*12)+(i*128)] =        0.0; A[93+(j*12)+(i*128)] =        0.0; A[109+(j*12)+(i*128)] =        0.0;  
      A[14+(j*12)+(i*128)] = -N23_32[i]; A[30+(j*12)+(i*128)] = -N23_33[i]; A[46+(j*12)+(i*128)]  = -N23_34[i];
      A[78+(j*12)+(i*128)] = -N23_36[i]; A[94+(j*12)+(i*128)] = -N23_37[i]; A[110+(j*12)+(i*128)] = -N23_38[i];  
      A[15+(j*12)+(i*128)] = -N23_42[i]; A[31+(j*12)+(i*128)] = -N23_43[i]; A[47+(j*12)+(i*128)]  = -N23_44[i];
      A[79+(j*12)+(i*128)] = -N23_46[i]; A[95+(j*12)+(i*128)] = -N23_47[i]; A[111+(j*12)+(i*128)] = -N23_48[i];  
    }
  }      


  /* matrix B */
  for(i=0; i<256; i++)
      B[i] = 0.0; 

  for(i=0; i<8; i++)
    B[(16*i)+i] = 1.0;

  for(i=0; i<2; i++) {
    B[140+(i*12)] =       0.0; B[156+(i*12)] =       0.0; B[172+(i*12)] =       0.0; 
    B[204+(i*12)] = N23_16[2]; B[220+(i*12)] = N23_17[2]; B[236+(i*12)] = N23_18[2];  
    B[141+(i*12)] = N23_22[2]; B[157+(i*12)] = N23_23[2]; B[173+(i*12)] = N23_24[2];
    B[205+(i*12)] =       0.0; B[221+(i*12)] =       0.0; B[237+(i*12)] =       0.0;  
    B[142+(i*12)] = N23_32[2]; B[158+(i*12)] = N23_33[2]; B[174+(i*12)] = N23_34[2];
    B[206+(i*12)] = N23_36[2]; B[222+(i*12)] = N23_37[2]; B[238+(i*12)] = N23_38[2];  
    B[143+(i*12)] = N23_42[2]; B[159+(i*12)] = N23_43[2]; B[175+(i*12)] = N23_44[2];
    B[207+(i*12)] = N23_46[2]; B[223+(i*12)] = N23_47[2]; B[239+(i*12)] = N23_48[2];  
  }

  /* LAPACK */
  /* int dggev_(char *jobvl, char *jobvr, integer *n, doublereal *a,
                integer *lda, doublereal *b, integer *ldb, doublereal *alphar, 
		doublereal *alphai, doublereal *beta, doublereal *vl, integer *ldvl, 
		doublereal *vr, integer *ldvr, doublereal *work, integer *lwork, 
		integer *info); 
  */
  

  dggev_(&jobvl,&jobvr,&n,A,&lda,B,&ldb,alphar,alphai,beta,vl,&ldvl,vr,&ldvr,work,&lwork,&info); 
  nsol = 0;
  for(i=0; i<16; i++) {
    if(alphai[i] == 0.0) {  // ??? +- EPS ???
      xx2->a_i[nsol] = alphar[i]/beta[i];
      nsol++; 
    }
  }
  xx2->order = nsol - 1;      

  ////////
  
  if((xx1->order != xx2->order) || (xx1->order == 0)) {
    free(xx1);
    free(xx2); 
    return 0;
  }   

  bio_set_couples(xx1,xx2,
		  K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211,
		  I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311,
		  J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311,
		  K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311);

  nsol = bio_compute_the_solutions(ct,xx1,xx2,sol_configs);

  free(xx1);
  free(xx2); 

  return (nsol); 
}
#endif

#ifdef USE_GSL
static int bio_compute_ik_solutions_with_GSL(p3d_cntrt *ct, double **sol_configs)
{
  /* coefsIJKL */
  double K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211;
/*   double L2cc,L2cs,L2c1,L2sc,L2ss,L21c,L21s,L211; */
  double I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311;
  double J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311;
  double K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311;
/*   double L3cc,L3cs,L3c1,L3sc,L3ss,L3s1,L31c,L31s,L311; */
  /* matrix elems */
  poly2_coefs M23_16,M23_17,M23_18,M23_22,M23_23,M23_24,M23_32,M23_33,M23_34,M23_36,M23_37,M23_38,M23_42,M23_43,M23_44,M23_46,M23_47,M23_48;
  poly2_coefs N23_16,N23_17,N23_18,N23_22,N23_23,N23_24,N23_32,N23_33,N23_34,N23_36,N23_37,N23_38,N23_42,N23_43,N23_44,N23_46,N23_47,N23_48;
  /* polynomoals coefs */
  poly_max16_coefs *equ_23,*equp_23;
  /* x1 and x2 solution vectors */
  poly_max16_coefs *xx1,*xx2;

  int nsol;
  bio_6R_ik_data *ikd = ct->bio_ik_data;


  bio_compute_coefsIJKL(ikd,&K2cc,&K2cs,&K2c1,&K2sc,&K2ss,&K21c,&K21s,&K211,
			&I3cc,&I3cs,&I3c1,&I3sc,&I3ss,&I3s1,&I31c,&I31s,&I311,
			&J3cc,&J3cs,&J3c1,&J3sc,&J3ss,&J3s1,&J31c,&J31s,&J311,
			&K3cc,&K3cs,&K3c1,&K3sc,&K3ss,&K3s1,&K31c,&K31s,&K311);

  bio_compute_M23_N23_elems(K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211,
			    I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311,
			    J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311,
			    K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311,
			    M23_16,M23_17,M23_18,M23_22,M23_23,M23_24,M23_32,M23_33,M23_34,
			    M23_36,M23_37,M23_38,M23_42,M23_43,M23_44,M23_46,M23_47,M23_48,
			    N23_16,N23_17,N23_18,N23_22,N23_23,N23_24,N23_32,N23_33,N23_34,
			    N23_36,N23_37,N23_38,N23_42,N23_43,N23_44,N23_46,N23_47,N23_48);

  /* compute sol1 */
  equ_23 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));

  bio_compute_det_M23orN23(M23_16,M23_17,M23_18,M23_22,M23_23,M23_24,
			   M23_32,M23_33,M23_34,M23_36,M23_37,M23_38,
			   M23_42,M23_43,M23_44,M23_46,M23_47,M23_48,
			   equ_23); 

  if(equ_23->order != 16) {
   free(equ_23);
   return 0;
  }

  // solve  
  xx1 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));  

  if((!solve_polynomial_eq(equ_23,xx1)) || (xx1->order < 0)) { 
    free(equ_23);
    free(xx1);
    return 0;
  }
  free(equ_23);

  /* compute sol2 */
  equp_23 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));
  
  bio_compute_det_M23orN23(N23_16,N23_17,N23_18,N23_22,N23_23,N23_24,
			   N23_32,N23_33,N23_34,N23_36,N23_37,N23_38,
			   N23_42,N23_43,N23_44,N23_46,N23_47,N23_48,
			   equp_23);
  
  if(equp_23->order != 16) {
    free(xx1);
    free(equp_23);
    return 0;
  } 

  // solve  
  xx2 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));  
  
  if((!solve_polynomial_eq(equp_23,xx2)) || (xx2->order < 0)) { 
    free(equp_23);
    free(xx1);
    free(xx2);
    return 0;
  }
  free(equp_23);
  
  if(xx1->order != xx2->order) {
    free(xx1);
    free(xx2); 
    return 0;
  }   

  bio_set_couples(xx1,xx2,
		  K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211,
		  I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311,
		  J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311,
		  K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311);

  nsol = bio_compute_the_solutions(ct,xx1,xx2,sol_configs);

  free(xx1);
  free(xx2); 

  return (nsol); 
}
#endif

/*******************************************************************************/

static void bio_compute_coefsIJKL(bio_6R_ik_data *ikd,
				  double *K2cc,double *K2cs,double *K2c1,double *K2sc,double *K2ss,double *K21c,double *K21s,double *K211,
				  double *I3cc,double *I3cs,double *I3c1,double *I3sc,double *I3ss,double *I3s1,double *I31c,double *I31s,double *I311,
				  double *J3cc,double *J3cs,double *J3c1,double *J3sc,double *J3ss,double *J3s1,double *J31c,double *J31s,double *J311,
				  double *K3cc,double *K3cs,double *K3c1,double *K3sc,double *K3ss,double *K3s1,double *K31c,double *K31s,double *K311)
{
  
  double Ducc,Ducs,Duc1,Dusc,Duss,Dus1,Du1c,Du1s,Du11;
  double Dvcc,Dvcs,Dvc1,Dvsc,Dvss,Dvs1,Dv1c,Dv1s,Dv11;
  double Dwcc,Dwcs,Dwc1,Dwsc,Dwss,Dws1,Dw1c,Dw1s,Dw11;
  double Dlcc,Dlcs,Dlc1,Dlsc,Dlss,Dls1,Dl1c,Dl1s,Dl11;
  double Dmcc,Dmcs,Dmc1,Dmsc,Dmss,Dms1,Dm1c,Dm1s,Dm11;
  double Dncc,Dncs,Dnc1,Dnsc,Dnss,Dns1,Dn1c,Dn1s,Dn11;
  
  
  //###############################################################################
  //# Coefficients des 14 termes de base                                          #
  //###############################################################################
  
  //# Coefficients de alpha #
  ikd->palcc = (ikd->cal1sal2) * (ikd->sth7cal7);
  ikd->palcs = (ikd->cal1sal2) * (ikd->cth7);
  ikd->palc1 = -(ikd->sal1sal2) * (ikd->sth7sal7);
  ikd->palsc = (ikd->sal2) * (ikd->cth7);
  ikd->palss = -(ikd->sal2) * (ikd->sth7cal7);
  //ikd->pals1 = 0;
  ikd->pal1c = (ikd->sal1cal2) * (ikd->sth7cal7);
  ikd->pal1s = (ikd->sal1cal2) * (ikd->cth7);
  ikd->pal11 = (ikd->cal1cal2) * (ikd->sth7sal7);

  //# Coefficients de beta #
  ikd->pbecc = -(ikd->cal1sal2) * (ikd->sal7);
  //ikd->pbecs = 0;
  ikd->pbec1 = -(ikd->sal1sal2) * (ikd->cal7);
  //ikd->pbesc = 0;
  ikd->pbess = (ikd->sal2) * (ikd->sal7);
  //ikd->pbes1 = 0;
  ikd->pbe1c = -(ikd->sal1cal2) * (ikd->sal7);
  //ikd->pbe1s = 0;
  ikd->pbe11 = (ikd->cal1cal2) * (ikd->cal7);
  
  //# Coefficients de gamma #
  ikd->pgacc = (ikd->cal1sal2) * (ikd->cth7cal7);
  ikd->pgacs = -(ikd->cal1sal2) * (ikd->sth7);
  ikd->pgac1 = -(ikd->sal1sal2) * (ikd->cth7sal7);
  ikd->pgasc = -(ikd->sal2) * (ikd->sth7);
  ikd->pgass = -(ikd->sal2) * (ikd->cth7cal7);
  //ikd->pgas1 = 0;
  ikd->pga1c = (ikd->sal1cal2) * (ikd->cth7cal7);
  ikd->pga1s = -(ikd->sal1cal2) * (ikd->sth7);
  ikd->pga11 = (ikd->cal1cal2) * (ikd->cth7sal7);
  
  //# Coefficients de a #
  //ikd->paetcc = 0;
  //ikd->paetcs = 0;
  //ikd->paetc1 = 0;
  //ikd->paetsc = 0;
  //ikd->paetss = 0;
  //ikd->paets1 = 0;
  ikd->paet1c = (ikd->r2sal1) * (ikd->sth7cal7);
  ikd->paet1s = (ikd->r2sal1) * (ikd->cth7);
  ikd->paet11 = (ikd->r2cal1) * (ikd->sth7sal7);

  ikd->pacc = (ikd->r3) * (ikd->palcc);
  ikd->pacs = (ikd->r3) * (ikd->palcs);
  ikd->pac1 = (ikd->r3) * (ikd->palc1);
  ikd->pasc = (ikd->r3) * (ikd->palsc);
  ikd->pass = (ikd->r3) * (ikd->palss);
  //ikd->pas1 = 0;
  ikd->pa1c = (ikd->paet1c) + (ikd->r3) * (ikd->pal1c);
  ikd->pa1s = (ikd->paet1s) + (ikd->r3) * (ikd->pal1s);
  ikd->pa11_ = (ikd->paet11) + (ikd->r3) * (ikd->pal11);
  ikd->pa11 = (ikd->pa11_) + (ikd->coef70_1);
  
  //# Coefficients de b #
  //ikd->pbetcc = 0;
  //ikd->pbetcs = 0;
  //ikd->pbetc1 = 0;
  //ikd->pbetsc = 0;
  //ikd->pbetss = 0;
  //ikd->pbets1 = 0;
  ikd->pbet1c = -(ikd->r2sal1) * (ikd->sal7);
  //ikd->pbet1s = 0;
  ikd->pbet11 = (ikd->r2cal1) * (ikd->cal7);

  ikd->pbcc = (ikd->r3) * (ikd->pbecc);
  //ikd->pbcs = 0;
  ikd->pbc1 = (ikd->r3) * (ikd->pbec1);
  //ikd->pbsc = 0;
  ikd->pbss = (ikd->r3) * (ikd->pbess);
  //ikd->pbs1 = 0;
  ikd->pb1c = (ikd->pbet1c) + (ikd->r3) * (ikd->pbe1c);
  //ikd->pb1s = 0;
  ikd->pb11_ = (ikd->pbet11) + (ikd->r3) * (ikd->pbe11);
  ikd->pb11 = (ikd->pb11_) + (ikd->coef70_3);
  
  //# Coefficients de c #
  //ikd->pcetcc = 0;
  //ikd->pcetcs = 0;
  //ikd->pcetc1 = 0;
  //ikd->pcetsc = 0;
  //ikd->pcetss = 0;
  //ikd->pcets1 = 0;
  ikd->pcet1c = (ikd->r2sal1) * (ikd->cth7cal7);
  ikd->pcet1s = -(ikd->r2sal1) * (ikd->sth7);
  ikd->pcet11 = (ikd->r2cal1) * (ikd->cth7sal7);
  
  ikd->pccc = (ikd->r3) * (ikd->pgacc);
  ikd->pccs = (ikd->r3) * (ikd->pgacs);
  ikd->pcc1 = (ikd->r3) * (ikd->pgac1);
  ikd->pcsc = (ikd->r3) * (ikd->pgasc);
  ikd->pcss = (ikd->r3) * (ikd->pgass);
  //ikd->pcs1 = 0;
  ikd->pc1c = (ikd->pcet1c) + (ikd->r3) * (ikd->pga1c);
  ikd->pc1s = (ikd->pcet1s) + (ikd->r3) * (ikd->pga1s);
  ikd->pc11_ = (ikd->pcet11) + (ikd->r3) * (ikd->pga11);
  ikd->pc11 = (ikd->pc11_) - (ikd->coef70_2);

  //# Coefficients de s #
  //ikd->psetcc = 0;
  //ikd->psetcs = 0;
  //ikd->psetc1 = 0;
  //ikd->psetsc = 0;
  //ikd->psetss = 0;
  //ikd->psets1 = 0;
  //ikd->pset1c = 0;
  //ikd->pset1s = 0;
  ikd->pset11 = (ikd->r2cal2);

  ikd->pscc = (ikd->coef70_1) * (ikd->palcc) + (ikd->coef70_3) * (ikd->pbecc) - (ikd->coef70_2) * (ikd->pgacc);
  ikd->pscs = (ikd->coef70_1) * (ikd->palcs) - (ikd->coef70_2) * (ikd->pgacs);
  ikd->psc1 = (ikd->coef70_1) * (ikd->palc1) + (ikd->coef70_3) * (ikd->pbec1) - (ikd->coef70_2) * (ikd->pgac1);
  ikd->pssc = (ikd->coef70_1) * (ikd->palsc) - (ikd->coef70_2) * (ikd->pgasc);
  ikd->psss = (ikd->coef70_1) * (ikd->palss) + (ikd->coef70_3) * (ikd->pbess) - (ikd->coef70_2) * (ikd->pgass);
  //ikd->pss1 = 0;
  ikd->ps1c = (ikd->coef70_1) * (ikd->pal1c) + (ikd->coef70_3) * (ikd->pbe1c) - (ikd->coef70_2) * (ikd->pga1c);
  ikd->ps1s = (ikd->coef70_1) * (ikd->pal1s) - (ikd->coef70_2) * (ikd->pga1s);
  ikd->ps11 = (ikd->pset11) + (ikd->coef70_1) * (ikd->pal11) + (ikd->coef70_3) * (ikd->pbe11) - (ikd->coef70_2) * (ikd->pga11) + (ikd->r3);
  
  //# Coefficients de sigma #
  //ikd->psietcc = 0;
  //ikd->psietcs = 0;
  //ikd->psietc1 = 0;
  //ikd->psietsc = 0;
  //ikd->psietss = 0;
  //ikd->psiets1 = 0;
  //ikd->psiet1c = 0;
  //ikd->psiet1s = 0;
  ikd->psiet11 = (ikd->sqr_r2_05);
  
  ikd->psicc = (ikd->coef70_1) * (ikd->pacc) + (ikd->coef70_3) * (ikd->pbcc) - (ikd->coef70_2) * (ikd->pccc);
  ikd->psics = (ikd->coef70_1) * (ikd->pacs) - (ikd->coef70_2) * (ikd->pccs);
  ikd->psic1 = (ikd->coef70_1) * (ikd->pac1) + (ikd->coef70_3) * (ikd->pbc1) - (ikd->coef70_2) * (ikd->pcc1);
  ikd->psisc = (ikd->coef70_1) * (ikd->pasc) - (ikd->coef70_2) * (ikd->pcsc);
  ikd->psiss = (ikd->coef70_1) * (ikd->pass) + (ikd->coef70_3) * (ikd->pbss) - (ikd->coef70_2) * (ikd->pcss);
  //ikd->psis1 = 0;
  ikd->psi1c = (ikd->coef70_1) * (ikd->pa1c) + (ikd->coef70_3) * (ikd->pb1c) - (ikd->coef70_2) * (ikd->pc1c);
  ikd->psi1s = (ikd->coef70_1) * (ikd->pa1s) - (ikd->coef70_2) * (ikd->pc1s);
  ikd->psi11 = (ikd->psiet11) + (ikd->r3) * (ikd->pset11) + (ikd->coef70_1) * (ikd->pa11_) + (ikd->coef70_3) * (ikd->pb11_) - (ikd->coef70_2) * (ikd->pc11_) + (ikd->coef730_1);

  //# Coefficients de u #
  ikd->puetcc =  - (ikd->r2sal2) * (ikd->cth7);
  ikd->puetcs = (ikd->r2sal2) * (ikd->sth7cal7);
  //ikd->puetc1 = 0;
  ikd->puetsc = (ikd->r2) * (ikd->cal1sal2) * (ikd->sth7cal7);
  ikd->puetss = (ikd->r2) * (ikd->cal1sal2) * (ikd->cth7);
  ikd->puets1 =  - (ikd->r2) * (ikd->sal1sal2) * (ikd->sth7sal7);
  //ikd->puet1c = 0;
  //ikd->puet1s = 0;
  //ikd->puet11 = 0;
  
  ikd->pucc = (ikd->puetcc) - (ikd->coef70_2) * (ikd->pbecc) - (ikd->coef70_3) * (ikd->pgacc);
  ikd->pucs = (ikd->puetcs) - (ikd->coef70_3) * (ikd->pgacs);
  ikd->puc1 = -(ikd->coef70_2) * (ikd->pbec1) - (ikd->coef70_3) * (ikd->pgac1);
  ikd->pusc = (ikd->puetsc) - (ikd->coef70_3) * (ikd->pgasc);
  ikd->puss = (ikd->puetss) - (ikd->coef70_2) * (ikd->pbess) - (ikd->coef70_3) * (ikd->pgass);
  ikd->pus1 = (ikd->puets1);
  ikd->pu1c = -(ikd->coef70_2) * (ikd->pbe1c) - (ikd->coef70_3) * (ikd->pga1c);
  ikd->pu1s = -(ikd->coef70_3) * (ikd->pga1s);
  ikd->pu11 = -(ikd->coef70_2) * (ikd->pbe11) - (ikd->coef70_3) * (ikd->pga11);
  
  //# Coefficients de v #
  //ikd->pvetcc = 0;
  ikd->pvetcs = -(ikd->r2sal2) * (ikd->sal7);
  //ikd->pvetc1 = 0;
  ikd->pvetsc = -(ikd->r2) * (ikd->cal1sal2) * (ikd->sal7);
  //ikd->pvetss = 0;
  ikd->pvets1 = -(ikd->r2) * (ikd->sal1sal2) * (ikd->cal7);
  //ikd->pvet1c = 0;
  //ikd->pvet1s = 0;
  //ikd->pvet11 = 0;
  
  ikd->pvcc = (ikd->coef70_1) * (ikd->pgacc) + (ikd->coef70_2) * (ikd->palcc);
  ikd->pvcs = (ikd->pvetcs) + (ikd->coef70_1) * (ikd->pgacs) + (ikd->coef70_2) * (ikd->palcs);
  ikd->pvc1 = (ikd->coef70_1) * (ikd->pgac1) + (ikd->coef70_2) * (ikd->palc1);
  ikd->pvsc = (ikd->pvetsc) + (ikd->coef70_1) * (ikd->pgasc) + (ikd->coef70_2) * (ikd->palsc);
  ikd->pvss = (ikd->coef70_1) * (ikd->pgass) + (ikd->coef70_2) * (ikd->palss);
  ikd->pvs1 = (ikd->pvets1);
  ikd->pv1c = (ikd->coef70_1) * (ikd->pga1c) + (ikd->coef70_2) * (ikd->pal1c);
  ikd->pv1s = (ikd->coef70_1) * (ikd->pga1s) + (ikd->coef70_2) * (ikd->pal1s);
  ikd->pv11 = (ikd->coef70_1) * (ikd->pga11) + (ikd->coef70_2) * (ikd->pal11);
  
  //# Coefficients de w #
  ikd->pwetcc = (ikd->r2sal2) * (ikd->sth7);
  ikd->pwetcs = (ikd->r2sal2) * (ikd->cth7cal7);
  //ikd->pwetc1 = 0;
  ikd->pwetsc = (ikd->r2) * (ikd->cal1sal2) * (ikd->cth7cal7);
  ikd->pwetss = -(ikd->r2) * (ikd->cal1sal2) * (ikd->sth7);
  ikd->pwets1 = -(ikd->r2) * (ikd->sal1sal2) * (ikd->cth7sal7);
  //ikd->pwet1c = 0;
  //ikd->pwet1s = 0;
  //ikd->pwet11 = 0;
  
  ikd->pwcc = (ikd->pwetcc) - (ikd->coef70_1) * (ikd->pbecc) + (ikd->coef70_3) * (ikd->palcc);
  ikd->pwcs = (ikd->pwetcs) + (ikd->coef70_3) * (ikd->palcs);
  ikd->pwc1 = -(ikd->coef70_1) * (ikd->pbec1) + (ikd->coef70_3) * (ikd->palc1);
  ikd->pwsc = (ikd->pwetsc) + (ikd->coef70_3) * (ikd->palsc);
  ikd->pwss = (ikd->pwetss) - (ikd->coef70_1) * (ikd->pbess) + (ikd->coef70_3) * (ikd->palss);
  ikd->pws1 = (ikd->pwets1);
  ikd->pw1c = -(ikd->coef70_1) * (ikd->pbe1c) + (ikd->coef70_3) * (ikd->pal1c);
  ikd->pw1s = (ikd->coef70_3) * (ikd->pal1s);
  ikd->pw11 = -(ikd->coef70_1) * (ikd->pbe11) + (ikd->coef70_3) * (ikd->pal11);
  
  //# Coefficients de l #
  ikd->pletcc = (ikd->sqr_r2_05) * (ikd->cal1sal2) * (ikd->sth7cal7);
  ikd->pletcs = (ikd->sqr_r2_05) * (ikd->cal1sal2) * (ikd->cth7);
  ikd->pletc1 = -(ikd->sqr_r2_05) * (ikd->sal1sal2) * (ikd->sth7sal7);
  ikd->pletsc = (ikd->sqr_r2_05) * (ikd->sal2) * (ikd->cth7);
  ikd->pletss = -(ikd->sqr_r2_05) * (ikd->sal2) * (ikd->sth7cal7);
  //ikd->plets1 = 0;
  ikd->plet1c = -(ikd->sqr_r2_05) * (ikd->sal1cal2) * (ikd->sth7cal7);
  ikd->plet1s = -(ikd->sqr_r2_05) * (ikd->sal1cal2) * (ikd->cth7);
  ikd->plet11 = -(ikd->sqr_r2_05) * (ikd->cal1cal2) * (ikd->sth7sal7);

  ikd->plcc = (ikd->pletcc) + (ikd->coef70_3) * (ikd->pwetcc) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->palcc) + (ikd->coef70_3) * (ikd->pbecc) - (ikd->coef70_2) * (ikd->pgacc)) + (ikd->coef730_2) * (ikd->palcc);
  ikd->plcs = (ikd->pletcs) + (ikd->coef70_3) * (ikd->pwetcs) + (ikd->coef70_2) * (ikd->pvetcs) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->palcs) - (ikd->coef70_2) * (ikd->pgacs)) + (ikd->coef730_2) * (ikd->palcs);
  ikd->plc1 = (ikd->pletc1) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->palc1) + (ikd->coef70_3) * (ikd->pbec1) - (ikd->coef70_2) * (ikd->pgac1)) + (ikd->coef730_2) * (ikd->palc1);
  ikd->plsc = (ikd->pletsc) + (ikd->coef70_3) * (ikd->pwetsc) + (ikd->coef70_2) * (ikd->pvetsc) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->palsc) - (ikd->coef70_2) * (ikd->pgasc)) + (ikd->coef730_2) * (ikd->palsc);
  ikd->plss = (ikd->pletss) + (ikd->coef70_3) * (ikd->pwetss) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->palss) + (ikd->coef70_3) * (ikd->pbess) - (ikd->coef70_2) * (ikd->pgass)) + (ikd->coef730_2) * (ikd->palss);
  ikd->pls1 = (ikd->coef70_3) * (ikd->pwets1) + (ikd->coef70_2) * (ikd->pvets1);
  ikd->pl1c = (ikd->plet1c) - (ikd->r3) * (ikd->paet1c) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->pal1c) + (ikd->coef70_3) * (ikd->pbe1c) - (ikd->coef70_2) * (ikd->pga1c)) + (ikd->coef730_2) * (ikd->pal1c);
  ikd->pl1s = (ikd->plet1s) - (ikd->r3) * (ikd->paet1s) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->pal1s) - (ikd->coef70_2) * (ikd->pga1s)) + (ikd->coef730_2) * (ikd->pal1s);
  ikd->pl11 = (ikd->plet11) - (ikd->r3) * (ikd->paet11) - (ikd->coef70_1) * (ikd->pset11) - (ikd->coef70_1) * ((ikd->coef70_1) * (ikd->pal11) + (ikd->coef70_3) * (ikd->pbe11) - (ikd->coef70_2) * (ikd->pga11)) + (ikd->coef730_2) * (ikd->pal11) - (ikd->r3) * (ikd->coef70_1);

  //# Coefficients de m #
  ikd->pmetcc = -(ikd->sqr_r2_05) * (ikd->cal1sal2) * (ikd->sal7);
  //ikd->pmetcs = 0;
  ikd->pmetc1 = -(ikd->sqr_r2_05) * (ikd->sal1sal2) * (ikd->cal7);
  //ikd->pmetsc = 0;
  ikd->pmetss = (ikd->sqr_r2_05) * (ikd->sal2) * (ikd->sal7);
  //ikd->pmets1 = 0;
  ikd->pmet1c = (ikd->sqr_r2_05) * (ikd->sal1cal2) * (ikd->sal7);
  //ikd->pmet1s = 0;
  ikd->pmet11 = -(ikd->sqr_r2_05) * (ikd->cal1cal2) * (ikd->cal7);
  
  ikd->pmcc = (ikd->pmetcc) - (ikd->coef70_2) * (ikd->puetcc) - (ikd->coef70_1) * (ikd->pwetcc) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->palcc) + (ikd->coef70_3) * (ikd->pbecc) - (ikd->coef70_2) * (ikd->pgacc)) + (ikd->coef730_2) * (ikd->pbecc);
  ikd->pmcs = -(ikd->coef70_2) * (ikd->puetcs) - (ikd->coef70_1) * (ikd->pwetcs) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->palcs) - (ikd->coef70_2) * (ikd->pgacs));
  ikd->pmc1 = (ikd->pmetc1) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->palc1) + (ikd->coef70_3) * (ikd->pbec1) - (ikd->coef70_2) * (ikd->pgac1)) + (ikd->coef730_2) * (ikd->pbec1);
  ikd->pmsc = -(ikd->coef70_2) * (ikd->puetsc) - (ikd->coef70_1) * (ikd->pwetsc) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->palsc) - (ikd->coef70_2) * (ikd->pgasc));
  ikd->pmss = (ikd->pmetss) - (ikd->coef70_2) * (ikd->puetss) - (ikd->coef70_1) * (ikd->pwetss) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->palss) + (ikd->coef70_3) * (ikd->pbess) - (ikd->coef70_2) * (ikd->pgass)) + (ikd->coef730_2) * (ikd->pbess);
  ikd->pms1 = -(ikd->coef70_2) * (ikd->puets1) - (ikd->coef70_1) * (ikd->pwets1);
  ikd->pm1c = (ikd->pmet1c) - (ikd->r3) * (ikd->pbet1c) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->pal1c) + (ikd->coef70_3) * (ikd->pbe1c) - (ikd->coef70_2) * (ikd->pga1c)) + (ikd->coef730_2) * (ikd->pbe1c);
  ikd->pm1s =  -(ikd->coef70_3) * ((ikd->coef70_1) * (ikd->pal1s) - (ikd->coef70_2) * (ikd->pga1s));
  ikd->pm11 = (ikd->pmet11) - (ikd->r3) * (ikd->pbet11) - (ikd->coef70_3) * (ikd->pset11) - (ikd->coef70_3) * ((ikd->coef70_1) * (ikd->pal11) + (ikd->coef70_3) * (ikd->pbe11) - (ikd->coef70_2) * (ikd->pga11)) + (ikd->coef730_2) * (ikd->pbe11) - (ikd->r3) * (ikd->coef70_3);
  
  //# Coefficients de n #
  
  ikd->pnetcc = (ikd->sqr_r2_05) * (ikd->cal1sal2) * (ikd->cth7cal7);
  ikd->pnetcs = -(ikd->sqr_r2_05) * (ikd->cal1sal2) * (ikd->sth7);
  ikd->pnetc1 = -(ikd->sqr_r2_05) * (ikd->sal1sal2) * (ikd->cth7sal7);
  ikd->pnetsc = -(ikd->sqr_r2_05) * (ikd->sal2) * (ikd->sth7);
  ikd->pnetss = -(ikd->sqr_r2_05) * (ikd->sal2) * (ikd->cth7cal7);
  //ikd->pnets1 = 0;
  ikd->pnet1c = -(ikd->sqr_r2_05) * (ikd->sal1cal2) * (ikd->cth7cal7);
  ikd->pnet1s = (ikd->sqr_r2_05) * (ikd->sal1cal2) * (ikd->sth7);
  ikd->pnet11 = -(ikd->sqr_r2_05) * (ikd->cal1cal2) * (ikd->cth7sal7);
  
  ikd->pncc = (ikd->pnetcc) - (ikd->coef70_3) * (ikd->puetcc) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->palcc) + (ikd->coef70_3) * (ikd->pbecc) - (ikd->coef70_2) * (ikd->pgacc)) + (ikd->coef730_2) * (ikd->pgacc);
  ikd->pncs = (ikd->pnetcs) - (ikd->coef70_3) * (ikd->puetcs) + (ikd->coef70_1) * (ikd->pvetcs) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->palcs) - (ikd->coef70_2) * (ikd->pgacs)) + (ikd->coef730_2) * (ikd->pgacs);
  ikd->pnc1 = (ikd->pnetc1) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->palc1) + (ikd->coef70_3) * (ikd->pbec1) - (ikd->coef70_2) * (ikd->pgac1)) + (ikd->coef730_2) * (ikd->pgac1);
  ikd->pnsc = (ikd->pnetsc) - (ikd->coef70_3) * (ikd->puetsc) + (ikd->coef70_1) * (ikd->pvetsc) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->palsc) - (ikd->coef70_2) * (ikd->pgasc)) + (ikd->coef730_2) * (ikd->pgasc);
  ikd->pnss = (ikd->pnetss) - (ikd->coef70_3) * (ikd->puetss) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->palss) + (ikd->coef70_3) * (ikd->pbess) - (ikd->coef70_2) * (ikd->pgass)) + (ikd->coef730_2) * (ikd->pgass);
  ikd->pns1 = -(ikd->coef70_3) * (ikd->puets1) + (ikd->coef70_1) * (ikd->pvets1);
  ikd->pn1c = (ikd->pnet1c) - (ikd->r3) * (ikd->pcet1c) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->pal1c) + (ikd->coef70_3) * (ikd->pbe1c) - (ikd->coef70_2) * (ikd->pga1c)) + (ikd->coef730_2) * (ikd->pga1c);
  ikd->pn1s = (ikd->pnet1s) - (ikd->r3) * (ikd->pcet1s) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->pal1s) - (ikd->coef70_2) * (ikd->pga1s)) + (ikd->coef730_2) * (ikd->pga1s);
  ikd->pn11 = (ikd->pnet11) - (ikd->r3) * (ikd->pcet11) + (ikd->coef70_2) * (ikd->pset11) + (ikd->coef70_2) * ((ikd->coef70_1) * (ikd->pal11) + (ikd->coef70_3) * (ikd->pbe11) - (ikd->coef70_2) * (ikd->pga11)) + (ikd->coef730_2) * (ikd->pga11) + (ikd->r3) * (ikd->coef70_2);
  
  
  //###############################################################################
  //# Coefficients de Du, Dv, Dw, Dl, Dm, Dn                                      #
  //###############################################################################
  
  //# Coefficients de Du #
  Ducc = (ikd->r5sal4) * (ikd->pucc);
  Ducs = (ikd->r5sal4) * (ikd->pucs);
  Duc1 = (ikd->r5sal4) * (ikd->puc1);
  Dusc = (ikd->r5sal4) * (ikd->pusc);
  Duss = (ikd->r5sal4) * (ikd->puss);
  Dus1 = (ikd->r5sal4) * (ikd->pus1);
  Du1c = (ikd->r5sal4) * (ikd->pu1c);
  Du1s = (ikd->r5sal4) * (ikd->pu1s);
  Du11 = (ikd->r5sal4) * (ikd->pu11);
  
  //# Coefficients de Dv #
  Dvcc = (ikd->r5sal4) * (ikd->pvcc);
  Dvcs = (ikd->r5sal4) * (ikd->pvcs);
  Dvc1 = (ikd->r5sal4) * (ikd->pvc1);
  Dvsc = (ikd->r5sal4) * (ikd->pvsc);
  Dvss = (ikd->r5sal4) * (ikd->pvss);
  Dvs1 = (ikd->r5sal4) * (ikd->pvs1);
  Dv1c = (ikd->r5sal4) * (ikd->pv1c);
  Dv1s = (ikd->r5sal4) * (ikd->pv1s);
  Dv11 = (ikd->r5sal4) * (ikd->pv11);
  
  //# Coefficients de Dw #
  Dwcc = (ikd->r5sal4) * (ikd->pwcc);
  Dwcs = (ikd->r5sal4) * (ikd->pwcs);
  Dwc1 = (ikd->r5sal4) * (ikd->pwc1);
  Dwsc = (ikd->r5sal4) * (ikd->pwsc);
  Dwss = (ikd->r5sal4) * (ikd->pwss);
  Dws1 = (ikd->r5sal4) * (ikd->pws1);
  Dw1c = (ikd->r5sal4) * (ikd->pw1c);
  Dw1s = (ikd->r5sal4) * (ikd->pw1s);
  Dw11 = (ikd->r5sal4) * (ikd->pw11);

  //# Coefficients de Dl #
  Dlcc = (ikd->sal4) * (ikd->plcc) - (ikd->r4cal3sal4) * (ikd->pacc) - (ikd->coef45_1) * (ikd->palcc);
  Dlcs = (ikd->sal4) * (ikd->plcs) - (ikd->r4cal3sal4) * (ikd->pacs) - (ikd->coef45_1) * (ikd->palcs);
  Dlc1 = (ikd->sal4) * (ikd->plc1) - (ikd->r4cal3sal4) * (ikd->pac1) - (ikd->coef45_1) * (ikd->palc1);
  Dlsc = (ikd->sal4) * (ikd->plsc) - (ikd->r4cal3sal4) * (ikd->pasc) - (ikd->coef45_1) * (ikd->palsc);
  Dlss = (ikd->sal4) * (ikd->plss) - (ikd->r4cal3sal4) * (ikd->pass) - (ikd->coef45_1) * (ikd->palss);
  Dls1 = (ikd->sal4) * (ikd->pls1);
  Dl1c = (ikd->sal4) * (ikd->pl1c) - (ikd->r4cal3sal4) * (ikd->pa1c) - (ikd->coef45_1) * (ikd->pal1c);
  Dl1s = (ikd->sal4) * (ikd->pl1s) - (ikd->r4cal3sal4) * (ikd->pa1s) - (ikd->coef45_1) * (ikd->pal1s);
  Dl11 = (ikd->sal4) * (ikd->pl11) - (ikd->r4cal3sal4) * (ikd->pa11) - (ikd->coef45_1) * (ikd->pal11);
 
  //# Coefficients de Dm #
  Dmcc = (ikd->sal4) * (ikd->pmcc) - (ikd->r4cal3sal4) * (ikd->pbcc) - (ikd->coef45_1) * (ikd->pbecc);
  Dmcs = (ikd->sal4) * (ikd->pmcs);
  Dmc1 = (ikd->sal4) * (ikd->pmc1) - (ikd->r4cal3sal4) * (ikd->pbc1) - (ikd->coef45_1) * (ikd->pbec1);
  Dmsc = (ikd->sal4) * (ikd->pmsc);
  Dmss = (ikd->sal4) * (ikd->pmss) - (ikd->r4cal3sal4) * (ikd->pbss) - (ikd->coef45_1) * (ikd->pbess);
  Dms1 = (ikd->sal4) * (ikd->pms1);
  Dm1c = (ikd->sal4) * (ikd->pm1c) - (ikd->r4cal3sal4) * (ikd->pb1c) - (ikd->coef45_1) * (ikd->pbe1c);
  Dm1s = (ikd->sal4) * (ikd->pm1s);
  Dm11 = (ikd->sal4) * (ikd->pm11) - (ikd->r4cal3sal4) * (ikd->pb11) - (ikd->coef45_1) * (ikd->pbe11);
  
  //# Coefficients de Dn #
  Dncc = (ikd->sal4) * (ikd->pncc) - (ikd->r4cal3sal4) * (ikd->pccc) - (ikd->coef45_1) * (ikd->pgacc);
  Dncs = (ikd->sal4) * (ikd->pncs) - (ikd->r4cal3sal4) * (ikd->pccs) - (ikd->coef45_1) * (ikd->pgacs);
  Dnc1 = (ikd->sal4) * (ikd->pnc1) - (ikd->r4cal3sal4) * (ikd->pcc1) - (ikd->coef45_1) * (ikd->pgac1);
  Dnsc = (ikd->sal4) * (ikd->pnsc) - (ikd->r4cal3sal4) * (ikd->pcsc) - (ikd->coef45_1) * (ikd->pgasc);
  Dnss = (ikd->sal4) * (ikd->pnss) - (ikd->r4cal3sal4) * (ikd->pcss) - (ikd->coef45_1) * (ikd->pgass);
  Dns1 = (ikd->sal4) * (ikd->pns1);
  Dn1c = (ikd->sal4) * (ikd->pn1c) - (ikd->r4cal3sal4) * (ikd->pc1c) - (ikd->coef45_1) * (ikd->pga1c);
  Dn1s = (ikd->sal4) * (ikd->pn1s) - (ikd->r4cal3sal4) * (ikd->pc1s) - (ikd->coef45_1) * (ikd->pga1s);
  Dn11 = (ikd->sal4) * (ikd->pn11) - (ikd->r4cal3sal4) * (ikd->pc11) - (ikd->coef45_1) * (ikd->pga11);
  
  
  //###############################################################################
  //# Coefficients de Ii, Ji, Ki, Li pour i= 1,2,3                                  #
  //###############################################################################

  //# Tous les coefs pou i=1 sont 0 !
  
  //# Coefficients de I2 #
  
  //# Coefficients de J2 #
  
  // Coefs. I2 et J2 = 0 ! 
    
  //# Coefficients de K2 #
  
  *K2cc = (ikd->coef543_1) * (ikd->psicc);
  *K2cs = (ikd->coef543_1) * (ikd->psics);
  *K2c1 = (ikd->coef543_1) * (ikd->psic1);
  *K2sc = (ikd->coef543_1) * (ikd->psisc);
  *K2ss = (ikd->coef543_1) * (ikd->psiss);
  //*K2s1 = 0;
  *K21c = (ikd->coef543_1) * (ikd->psi1c);
  *K21s = (ikd->coef543_1) * (ikd->psi1s);
  *K211 = (ikd->coef543_1) * (ikd->psi11) - (ikd->coef543_2);
  
  //# Coefficients de L2  #
  /*  L2cc = (ikd->coef543_1) * (ikd->psicc);  */
  /*  L2cs = (ikd->coef543_1) * (ikd->psics);  */
  /*  L2c1 = (ikd->coef543_1) * (ikd->psic1);  */
  /*  L2sc = (ikd->coef543_1) * (ikd->psisc);  */
  /*  L2ss = (ikd->coef543_1) * (ikd->psiss);  */
  /*  //L2s1 = 0;  */
  /*  L21c = (ikd->coef543_1) * (ikd->psi1c);  */
  /*  L21s = (ikd->coef543_1) * (ikd->psi1s);  */
  /*  L211 = (ikd->coef543_1) * (ikd->psi11) - (ikd->coef543_2);  */
    
  /*   L2cc = K2cc; */
  /*   L2cs = K2cs; */
  /*   L2c1 = K2c1; */
  /*   L2sc = K2sc; */
  /*   L2ss = K2ss; */
  /*   //L2s1 = 0; */
  /*   L21c = K21c; */
  /*   L21s = K21s; */
  /*   L211 = K211; */
    
  //# Coefficients de I3 #
  *I3cc = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvcc + (ikd->sal5) * Dwcc + Dlcc);
  *I3cs = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvcs + (ikd->sal5) * Dwcs + Dlcs);
  *I3c1 = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvc1 + (ikd->sal5) * Dwc1 + Dlc1);
  *I3sc = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvsc + (ikd->sal5) * Dwsc + Dlsc);
  *I3ss = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvss + (ikd->sal5) * Dwss + Dlss);
  *I3s1 = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvs1 + (ikd->sal5) * Dws1 + Dls1);
  *I31c = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dv1c + (ikd->sal5) * Dw1c + Dl1c);
  *I31s = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dv1s + (ikd->sal5) * Dw1s + Dl1s);
  *I311 = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dv11 + (ikd->sal5) * Dw11 + Dl11);
  
  //# Coefficients de J3 #
  *J3cc = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvcc - (ikd->sal5) * Dwcc + Dlcc);
  *J3cs = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvcs - (ikd->sal5) * Dwcs + Dlcs);
  *J3c1 = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvc1 - (ikd->sal5) * Dwc1 + Dlc1);
  *J3sc = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvsc - (ikd->sal5) * Dwsc + Dlsc);
  *J3ss = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvss - (ikd->sal5) * Dwss + Dlss);
  *J3s1 = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dvs1 - (ikd->sal5) * Dws1 + Dls1);
  *J31c = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dv1c - (ikd->sal5) * Dw1c + Dl1c);
  *J31s = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dv1s - (ikd->sal5) * Dw1s + Dl1s);
  *J311 = -(ikd->r4sal3sal4) * (-(ikd->cal5) * Dv11 - (ikd->sal5) * Dw11 + Dl11);
    
  //# Coefficients de K3 #
  *K3cc = -(ikd->r4sal3sal4) * ((ikd->cal5) * Ducc + Dmcc);
  *K3cs = -(ikd->r4sal3sal4) * ((ikd->cal5) * Ducs + Dmcs);
  *K3c1 = -(ikd->r4sal3sal4) * ((ikd->cal5) * Duc1 + Dmc1);
  *K3sc = -(ikd->r4sal3sal4) * ((ikd->cal5) * Dusc + Dmsc);
  *K3ss = -(ikd->r4sal3sal4) * ((ikd->cal5) * Duss + Dmss);
  *K3s1 = -(ikd->r4sal3sal4) * ((ikd->cal5) * Dus1 + Dms1);
  *K31c = -(ikd->r4sal3sal4) * ((ikd->cal5) * Du1c + Dm1c);
  *K31s = -(ikd->r4sal3sal4) * ((ikd->cal5) * Du1s + Dm1s);
  *K311 = -(ikd->r4sal3sal4) * ((ikd->cal5) * Du11 + Dm11);
  
  //# Coefficients de L3 #
  /* L3cc = (ikd->r4sal3sal4)*((ikd->cal5)*Ducc + Dmcc); */
  /* L3cs = (ikd->r4sal3sal4)*((ikd->cal5)*Ducs + Dmcs); */
  /* L3c1 = (ikd->r4sal3sal4)*((ikd->cal5)*Duc1 + Dmc1); */
  /* L3sc = (ikd->r4sal3sal4)*((ikd->cal5)*Dusc + Dmsc); */
  /* L3ss = (ikd->r4sal3sal4)*((ikd->cal5)*Duss + Dmss); */
  /* L3s1 = (ikd->r4sal3sal4)*((ikd->cal5)*Dus1 + Dms1); */
  /* L31c = (ikd->r4sal3sal4)*((ikd->cal5)*Du1c + Dm1c); */
  /* L31s = (ikd->r4sal3sal4)*((ikd->cal5)*Du1s + Dm1s); */
  /* L311 = (ikd->r4sal3sal4)*((ikd->cal5)*Du11 + Dm11); */
  
  /*   L3cc = - K3cc; */
  /*   L3cs = - K3cs; */
  /*   L3c1 = - K3c1; */
  /*   L3sc = - K3sc; */
  /*   L3ss = - K3ss; */
  /*   L3s1 = - K3s1; */
  /*   L31c = - K31c; */
  /*   L31s = - K31s; */
  /*   L311 = - K311; */
 
}


static void bio_compute_M23_N23_elems(double K2cc,double K2cs,double K2c1,double K2sc,double K2ss,double K21c,double K21s,double K211,
				      double I3cc,double I3cs,double I3c1,double I3sc,double I3ss,double I3s1,double I31c,double I31s,double I311,
				      double J3cc,double J3cs,double J3c1,double J3sc,double J3ss,double J3s1,double J31c,double J31s,double J311,
				      double K3cc,double K3cs,double K3c1,double K3sc,double K3ss,double K3s1,double K31c,double K31s,double K311,
				      poly2_coefs m16, poly2_coefs m17, poly2_coefs m18, poly2_coefs m22, poly2_coefs m23, poly2_coefs m24,
				      poly2_coefs m32, poly2_coefs m33, poly2_coefs m34, poly2_coefs m36, poly2_coefs m37, poly2_coefs m38,
				      poly2_coefs m42, poly2_coefs m43, poly2_coefs m44, poly2_coefs m46, poly2_coefs m47, poly2_coefs m48,
				      poly2_coefs n16, poly2_coefs n17, poly2_coefs n18, poly2_coefs n22, poly2_coefs n23, poly2_coefs n24,
				      poly2_coefs n32, poly2_coefs n33, poly2_coefs n34, poly2_coefs n36, poly2_coefs n37, poly2_coefs n38,
				      poly2_coefs n42, poly2_coefs n43, poly2_coefs n44, poly2_coefs n46, poly2_coefs n47, poly2_coefs n48)
{

  double ck2_1,ck2_2,ck2_3,ck2_4,ck2_5,ck2_6,ck2_7,ck2_8,ck2_9;
/*   double cl2_1,cl2_2,cl2_3,cl2_4,cl2_5,cl2_6,cl2_7,cl2_8,cl2_9; */
  double ci3_1,ci3_2,ci3_3,ci3_4,ci3_5,ci3_6,ci3_7,ci3_8,ci3_9;
  double cj3_1,cj3_2,cj3_3,cj3_4,cj3_5,cj3_6,cj3_7,cj3_8,cj3_9;
  double ck3_1,ck3_2,ck3_3,ck3_4,ck3_5,ck3_6,ck3_7,ck3_8,ck3_9;
/*   double cl3_1,cl3_2,cl3_3,cl3_4,cl3_5,cl3_6,cl3_7,cl3_8,cl3_9; */

  /* calculs intermediaires */

  ck2_1 = K21c-K2cc+K211-K2c1;
  ck2_2 = -K21c+K2cc+K211-K2c1;
  ck2_3 = K21c+K2cc+K211+K2c1;
  ck2_4 = -K21c-K2cc+K211+K2c1;
  ck2_5 = 2*K21s-2*K2cs;
  ck2_6 = 2*K2sc;
  ck2_7 = -2*K2sc;
  ck2_8 = 2*K21s+2*K2cs;
  ck2_9 = 4*K2ss;

/*   cl2_1 = L21c-L2cc+L211-L2c1; */
/*   cl2_2 = -L21c+L2cc+L211-L2c1; */
/*   cl2_3 = L21c+L2cc+L211+L2c1; */
/*   cl2_4 = -L21c-L2cc+L211+L2c1; */
/*   cl2_5 = 2*L21s-2*L2cs; */
/*   cl2_6 = 2*L2sc; */
/*   cl2_7 = -2*L2sc; */
/*   cl2_8 = 2*L21s+2*L2cs; */
/*   cl2_9 = 4*L2ss; */

  ci3_1 = I31c-I3cc+I311-I3c1;
  ci3_2 = -I31c+I3cc+I311-I3c1;
  ci3_3 = I31c+I3cc+I311+I3c1;
  ci3_4 = -I31c-I3cc+I311+I3c1;
  ci3_5 = 2*I31s-2*I3cs;
  ci3_6 = 2*I3sc+2*I3s1;
  ci3_7 = -2*I3sc+2*I3s1;
  ci3_8 = 2*I31s+2*I3cs;
  ci3_9 = 4*I3ss;

  cj3_1 = J31c-J3cc+J311-J3c1;
  cj3_2 = -J31c+J3cc+J311-J3c1;
  cj3_3 = J31c+J3cc+J311+J3c1;
  cj3_4 = -J31c-J3cc+J311+J3c1;
  cj3_5 = 2*J31s-2*J3cs;
  cj3_6 = 2*J3sc+2*J3s1;
  cj3_7 = -2*J3sc+2*J3s1;
  cj3_8 = 2*J31s+2*J3cs;
  cj3_9 = 4*J3ss;

  ck3_1 = K31c-K3cc+K311-K3c1;
  ck3_2 = -K31c+K3cc+K311-K3c1;
  ck3_3 = K31c+K3cc+K311+K3c1;
  ck3_4 = -K31c-K3cc+K311+K3c1;
  ck3_5 = 2*K31s-2*K3cs;
  ck3_6 = 2*K3sc+2*K3s1;
  ck3_7 = -2*K3sc+2*K3s1;
  ck3_8 = 2*K31s+2*K3cs;
  ck3_9 = 4*K3ss;

/*   cl3_1 = L31c-L3cc+L311-L3c1; */
/*   cl3_2 = -L31c+L3cc+L311-L3c1; */
/*   cl3_3 = L31c+L3cc+L311+L3c1; */
/*   cl3_4 = -L31c-L3cc+L311+L3c1; */
/*   cl3_5 = 2*L31s-2*L3cs; */
/*   cl3_6 = 2*L3sc+2*L3s1; */
/*   cl3_7 = -2*L3sc+2*L3s1; */
/*   cl3_8 = 2*L31s+2*L3cs; */
/*   cl3_9 = 4*L3ss; */


  /* matrix_element = me0 + me1 * x + me2 * x^2 */

  /** M23 **/

  m16[0] = ck2_1;
  m16[1] = ck2_5; 
  m16[2] = ck2_2;

  m17[0] = ck2_6;
  m17[1] = ck2_9; 
  m17[2] = ck2_7;

  m18[0] = ck2_3;
  m18[1] = ck2_8; 
  m18[2] = ck2_4;

/*   m22[0] = cl2_1; */
/*   m22[1] = cl2_5;  */
/*   m22[2] = cl2_2; */

/*   m23[0] = cl2_6; */
/*   m23[1] = cl2_9;  */
/*   m23[2] = cl2_7; */

/*   m24[0] = cl2_3; */
/*   m24[1] = cl2_8;  */
/*   m24[2] = cl2_4; */

  m22[0] = m16[0];
  m22[1] = m16[1];
  m22[2] = m16[2];

  m23[0] = m17[0];
  m23[1] = m17[1];
  m23[2] = m17[2];

  m24[0] = m18[0];
  m24[1] = m18[1];
  m24[2] = m18[2];

  m32[0] = cj3_1;
  m32[1] = cj3_5; 
  m32[2] = cj3_2;

  m33[0] = cj3_6;
  m33[1] = cj3_9; 
  m33[2] = cj3_7;

  m34[0] = cj3_3;
  m34[1] = cj3_8; 
  m34[2] = cj3_4;

  m36[0] = ck3_1;
  m36[1] = ck3_5; 
  m36[2] = ck3_2;

  m37[0] = ck3_6;
  m37[1] = ck3_9; 
  m37[2] = ck3_7;

  m38[0] = ck3_3;
  m38[1] = ck3_8; 
  m38[2] = ck3_4;

/*   m42[0] = cl3_1; */
/*   m42[1] = cl3_5;  */
/*   m42[2] = cl3_2; */

/*   m43[0] = cl3_6; */
/*   m43[1] = cl3_9;  */
/*   m43[2] = cl3_7; */

/*   m44[0] = cl3_3; */
/*   m44[1] = cl3_8;  */
/*   m44[2] = cl3_4; */

  m42[0] = - m36[0];
  m42[1] = - m36[1];
  m42[2] = - m36[2];

  m43[0] = - m37[0]; 
  m43[1] = - m37[1];
  m43[2] = - m37[2];
  
  m44[0] = - m38[0];
  m44[1] = - m38[1];
  m44[2] = - m38[2];
  
  m46[0] = ci3_1;
  m46[1] = ci3_5; 
  m46[2] = ci3_2;
  
  m47[0] = ci3_6;
  m47[1] = ci3_9; 
  m47[2] = ci3_7;
  
  m48[0] = ci3_3;
  m48[1] = ci3_8; 
  m48[2] = ci3_4;


  /** N23 **/

  n16[0] = ck2_4;
  n16[1] = ck2_7; 
  n16[2] = ck2_2;

  n17[0] = ck2_8;
  n17[1] = ck2_9; 
  n17[2] = ck2_5;

  n18[0] = ck2_3;
  n18[1] = ck2_6; 
  n18[2] = ck2_1;

/*   n22[0] = cl2_4; */
/*   n22[1] = cl2_7;  */
/*   n22[2] = cl2_2; */

/*   n23[0] = cl2_8; */
/*   n23[1] = cl2_9;  */
/*   n23[2] = cl2_5; */

/*   n24[0] = cl2_3; */
/*   n24[1] = cl2_6;  */
/*   n24[2] = cl2_1; */

  n22[0] = n16[0];
  n22[1] = n16[1];
  n22[2] = n16[2];
  
  n23[0] = n17[0];
  n23[1] = n17[1];
  n23[2] = n17[2];

  n24[0] = n18[0];
  n24[1] = n18[1];
  n24[2] = n18[2];
  
  n32[0] = cj3_4;
  n32[1] = cj3_7; 
  n32[2] = cj3_2;
  
  n33[0] = cj3_8;
  n33[1] = cj3_9; 
  n33[2] = cj3_5;
  
  n34[0] = cj3_3;
  n34[1] = cj3_6; 
  n34[2] = cj3_1;
  
  n36[0] = ck3_4;
  n36[1] = ck3_7; 
  n36[2] = ck3_2;
  
  n37[0] = ck3_8;
  n37[1] = ck3_9; 
  n37[2] = ck3_5;
  
  n38[0] = ck3_3;
  n38[1] = ck3_6; 
  n38[2] = ck3_1;
  
/*   n42[0] = cl3_4; */
/*   n42[1] = cl3_7;  */
/*   n42[2] = cl3_2; */

/*   n43[0] = cl3_8; */
/*   n43[1] = cl3_9;  */
/*   n43[2] = cl3_5; */

/*   n44[0] = cl3_3; */
/*   n44[1] = cl3_6;  */
/*   n44[2] = cl3_1; */

  n42[0] = - n36[0];
  n42[1] = - n36[1];
  n42[2] = - n36[2];
  
  n43[0] = - n37[0]; 
  n43[1] = - n37[1];
  n43[2] = - n37[2];
  
  n44[0] = - n38[0];
  n44[1] = - n38[1];
  n44[2] = - n38[2];
  
  n46[0] = ci3_4;
  n46[1] = ci3_7; 
  n46[2] = ci3_2;
  
  n47[0] = ci3_8;
  n47[1] = ci3_9; 
  n47[2] = ci3_5;
  
  n48[0] = ci3_3;
  n48[1] = ci3_6; 
  n48[2] = ci3_1;

  // PRINTS  
/*   printf("n16 = %f, %f, %f\n",n16[0],n16[1],n16[2]); */
/*   printf("n17 = %f, %f, %f\n",n17[0],n17[1],n17[2]); */
/*   printf("n18 = %f, %f, %f\n",n18[0],n18[1],n18[2]); */
/*   printf("n22 = %f, %f, %f\n",n22[0],n22[1],n22[2]); */
/*   printf("n23 = %f, %f, %f\n",n23[0],n23[1],n23[2]); */
/*   printf("n24 = %f, %f, %f\n",n24[0],n24[1],n24[2]); */
/*   printf("n32 = %f, %f, %f\n",n32[0],n32[1],n32[2]); */
/*   printf("n33 = %f, %f, %f\n",n33[0],n33[1],n33[2]); */
/*   printf("n34 = %f, %f, %f\n",n34[0],n34[1],n34[2]); */
/*   printf("n36 = %f, %f, %f\n",n36[0],n36[1],n36[2]); */
/*   printf("n37 = %f, %f, %f\n",n37[0],n37[1],n37[2]); */
/*   printf("n38 = %f, %f, %f\n",n38[0],n38[1],n38[2]); */
/*   printf("n42 = %f, %f, %f\n",n42[0],n42[1],n42[2]); */
/*   printf("n43 = %f, %f, %f\n",n43[0],n43[1],n43[2]); */
/*   printf("n44 = %f, %f, %f\n",n44[0],n44[1],n44[2]); */
/*   printf("n46 = %f, %f, %f\n",n46[0],n46[1],n46[2]); */
/*   printf("n47 = %f, %f, %f\n",n47[0],n47[1],n47[2]); */
/*   printf("n48 = %f, %f, %f\n",n48[0],n48[1],n48[2]); */

}


/*******************************************************************************/

static void bio_compute_det_M23orN23(poly2_coefs m16, poly2_coefs m17, poly2_coefs m18, 
				     poly2_coefs m22, poly2_coefs m23, poly2_coefs m24,
				     poly2_coefs m32, poly2_coefs m33, poly2_coefs m34,
				     poly2_coefs m36, poly2_coefs m37, poly2_coefs m38,
				     poly2_coefs m42, poly2_coefs m43, poly2_coefs m44,
				     poly2_coefs m46, poly2_coefs m47, poly2_coefs m48,
				     poly_max16_coefs *deter_coefs)
{
  poly_max16_coefs *pol16cfs,*addpol;
  
  pol16cfs = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));
  addpol = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));
  
  prod_of_8_poly2(1.0, m33, m33, m22, m24, m16, m16, m48, m48, addpol);
  
  prod_of_8_poly2(-1.0, m33, m22, m22, m44, m46, m17, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m24, m42, m16, m37, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m22, m24, m42, m16, m47, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m22, m24, m42, m36, m17, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m24, m42, m36, m47, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m24, m42, m46, m17, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m22, m24, m42, m46, m37, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m33, m33, m22, m24, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m33, m22, m24, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m33, m22, m24, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m33, m22, m24, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m33, m22, m24, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m33, m22, m24, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m33, m24, m23, m32, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m32, m16, m16, m48, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m32, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m24, m23, m32, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m32, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m24, m23, m32, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m32, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m33, m22, m24, m43, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m33, m22, m24, m43, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m34, m22, m22, m16, m16, m48, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m22, m22, m44, m16, m37, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m22, m44, m16, m47, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m22, m44, m36, m17, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m22, m22, m44, m36, m47, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m22, m44, m46, m37, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m33, m22, m24, m43, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m24, m43, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m33, m22, m24, m43, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m22, m24, m43, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m22, m24, m43, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m33, m22, m24, m43, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m42, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m24, m23, m42, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m24, m23, m42, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m42, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m24, m23, m42, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m42, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m33, m24, m23, m42, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m33, m24, m23, m42, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m32, m22, m44, m16, m37, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m32, m22, m44, m16, m47, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m32, m22, m44, m36, m17, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m32, m22, m44, m36, m47, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m32, m22, m44, m46, m17, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m32, m22, m44, m46, m37, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m32, m24, m43, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m32, m24, m43, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m32, m24, m43, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m32, m24, m43, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m32, m24, m43, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m22, m34, m16, m37, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m22, m34, m16, m47, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m22, m34, m36, m17, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m22, m34, m36, m47, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m22, m34, m46, m17, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m22, m34, m46, m37, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m24, m32, m16, m37, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m24, m32, m16, m47, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m24, m32, m36, m17, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m24, m32, m36, m47, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m24, m32, m46, m17, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m24, m32, m46, m37, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m24, m33, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m22, m24, m33, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m22, m24, m33, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m24, m23, m32, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m24, m23, m32, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m24, m23, m32, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m43, m43, m22, m24, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m43, m22, m24, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m43, m22, m24, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m43, m22, m24, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m43, m22, m24, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m43, m22, m24, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m43, m22, m24, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m43, m24, m23, m42, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m24, m23, m42, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m24, m23, m42, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m24, m23, m42, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m24, m23, m42, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m43, m24, m23, m42, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m43, m24, m23, m42, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m42, m22, m34, m16, m37, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m42, m22, m34, m16, m47, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m42, m22, m34, m36, m17, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m42, m22, m34, m36, m47, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m42, m22, m34, m46, m17, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m42, m22, m34, m46, m37, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m44, m22, m22, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m34, m22, m22, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m34, m22, m22, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m34, m22, m22, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m34, m22, m22, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m34, m22, m22, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m34, m22, m22, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(4.0, m34, m22, m24, m32, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m24, m32, m16, m16, m48, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m24, m32, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m24, m32, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m24, m32, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m24, m32, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m24, m32, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m22, m44, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m22, m44, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m22, m44, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m22, m22, m44, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m22, m44, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m22, m22, m44, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m22, m22, m44, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m22, m44, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m24, m42, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m24, m42, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m24, m42, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m22, m24, m42, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m34, m22, m24, m42, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m22, m24, m42, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m22, m24, m42, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m34, m22, m24, m42, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m22, m16, m24, m43, m36, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m22, m16, m24, m43, m36, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m22, m16, m24, m43, m46, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m22, m16, m24, m43, m46, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m22, m16, m16, m24, m43, m37, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m22, m16, m16, m24, m43, m38, m47, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m16, m24, m23, m42, m36, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m16, m24, m23, m42, m36, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m16, m24, m23, m42, m46, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m16, m24, m23, m42, m46, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m34, m16, m16, m24, m23, m42, m37, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m34, m16, m16, m24, m23, m42, m38, m47, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m24, m32, m32, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m32, m16, m16, m48, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m32, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m32, m32, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m32, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m32, m32, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m32, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m32, m22, m44, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m24, m32, m22, m44, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m24, m32, m22, m44, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m32, m22, m44, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m24, m32, m22, m44, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m32, m22, m44, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m32, m22, m44, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m32, m22, m44, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m24, m24, m32, m42, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m24, m32, m42, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m24, m32, m42, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m42, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m24, m32, m42, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m42, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m32, m42, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m24, m24, m32, m42, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m32, m16, m43, m36, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m16, m43, m36, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m16, m43, m46, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m32, m16, m43, m46, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m32, m16, m16, m43, m37, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m32, m16, m16, m43, m38, m47, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m22, m23, m33, m34, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m34, m16, m16, m48, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m34, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m33, m34, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m34, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m33, m34, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m34, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m44, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m33, m44, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m33, m44, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m44, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m33, m44, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m44, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m33, m44, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m33, m44, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m23, m23, m32, m34, m16, m46, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m34, m16, m16, m48, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m34, m16, m47, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m32, m34, m16, m47, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m34, m46, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m32, m34, m46, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m34, m46, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m44, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m32, m44, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m32, m44, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m44, m16, m47, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m32, m44, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m44, m46, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m32, m44, m46, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m32, m44, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m22, m22, m34, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m22, m22, m34, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m22, m22, m34, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m22, m24, m32, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m22, m24, m32, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m22, m24, m32, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m44, m44, m22, m22, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m44, m22, m22, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m44, m22, m22, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m44, m22, m22, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m44, m22, m22, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m44, m22, m22, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(4.0, m44, m22, m24, m42, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m44, m22, m24, m42, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m44, m22, m24, m42, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m44, m22, m24, m42, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m44, m22, m24, m42, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m44, m22, m24, m42, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m44, m22, m24, m42, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m22, m16, m24, m33, m36, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m22, m16, m24, m33, m36, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m22, m16, m24, m33, m46, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m22, m16, m24, m33, m46, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m22, m16, m16, m24, m33, m37, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m22, m16, m16, m24, m33, m38, m47, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m16, m24, m23, m32, m36, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m16, m24, m23, m32, m36, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m16, m24, m23, m32, m46, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m16, m24, m23, m32, m46, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m44, m16, m16, m24, m23, m32, m37, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m44, m16, m16, m24, m23, m32, m38, m47, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m42, m22, m34, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m42, m22, m34, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m42, m22, m34, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m32, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m32, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m42, m32, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m24, m24, m42, m42, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m42, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m42, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m42, m42, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m42, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m42, m42, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m42, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m16, m33, m36, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m42, m16, m33, m36, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m42, m16, m33, m46, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m16, m33, m46, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m24, m24, m42, m16, m16, m33, m37, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m24, m24, m42, m16, m16, m33, m38, m47, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m34, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m43, m34, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m43, m34, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m34, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m43, m34, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m34, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m43, m34, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m34, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(2.0, m22, m23, m43, m44, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m44, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m44, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m43, m44, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m44, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m22, m23, m43, m44, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m22, m23, m43, m44, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m34, m16, m46, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m42, m34, m16, m16, m48, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m42, m34, m16, m37, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m34, m16, m37, m48, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m42, m34, m36, m46, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m34, m36, m17, m47, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m42, m34, m36, m17, m17, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m34, m36, m16, m18, m48, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-2.0, m23, m23, m42, m44, m16, m36, m18, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m44, m16, m16, m38, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m44, m16, m37, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m42, m44, m16, m37, m38, m17, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m44, m36, m36, m18, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(-1.0, m23, m23, m42, m44, m36, m17, m37, m18, pol16cfs);
  add_2_poly16(pol16cfs,addpol);
  prod_of_8_poly2(1.0, m23, m23, m42, m44, m36, m17, m17, m38, pol16cfs);
  add_2_poly16(pol16cfs,addpol);

  //COPY(addpol, deter_coefs)
  copy_poly16(addpol, deter_coefs);

  free(pol16cfs);
  free(addpol);

}


/*******************************************************************************/

static void bio_set_couples(poly_max16_coefs *sol1, poly_max16_coefs *sol2,
			    double K2cc,double K2cs,double K2c1,double K2sc,double K2ss,double K21c,double K21s,double K211,
			    double I3cc,double I3cs,double I3c1,double I3sc,double I3ss,double I3s1,double I31c,double I31s,double I311,
			    double J3cc,double J3cs,double J3c1,double J3sc,double J3ss,double J3s1,double J31c,double J31s,double J311,
			    double K3cc,double K3cs,double K3c1,double K3sc,double K3ss,double K3s1,double K31c,double K31s,double K311)
{
  poly_max16_coefs *xx1,*xx2;
  int i,j;
  int ordxx;
  double ecart,thisecart;
  int coupi=0,coupj=0;

  xx1 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));
  xx2 = (poly_max16_coefs *)malloc(sizeof(poly_max16_coefs));

  ordxx = 0;
  for(i = 0; i <= sol1->order; i++) {
    ecart = 10E10;
    for(j = 0; j <= sol2->order; j++) {
      thisecart = fabs(bio_compute_ecart_sols(sol1->a_i[i],sol2->a_i[j],
					      K2cc,K2cs,K2c1,K2sc,K2ss,K21c,K21s,K211,
					      I3cc,I3cs,I3c1,I3sc,I3ss,I3s1,I31c,I31s,I311,
					      J3cc,J3cs,J3c1,J3sc,J3ss,J3s1,J31c,J31s,J311,
					      K3cc,K3cs,K3c1,K3sc,K3ss,K3s1,K31c,K31s,K311));
      if(thisecart < ecart) {
	ecart = thisecart;
	coupi = i;
	coupj = j;
      }
    }
    xx1->a_i[ordxx] = sol1->a_i[coupi];
    xx2->a_i[ordxx] = sol2->a_i[coupj];
    ordxx++;
  }
  xx1->order = xx2->order = ordxx - 1;
  
  copy_poly16(xx1,sol1);
  copy_poly16(xx2,sol2);

  free(xx1);
  free(xx2);
}



/*******************************************************************************/

static double bio_compute_ecart_sols(double x1, double x2,
				     double K2cc,double K2cs,double K2c1,double K2sc,double K2ss,double K21c,double K21s,double K211,
				     double I3cc,double I3cs,double I3c1,double I3sc,double I3ss,double I3s1,double I31c,double I31s,double I311,
				     double J3cc,double J3cs,double J3c1,double J3sc,double J3ss,double J3s1,double J31c,double J31s,double J311,
				     double K3cc,double K3cs,double K3c1,double K3sc,double K3ss,double K3s1,double K31c,double K31s,double K311)
{
  double ecart;
  double K2_1,I3_1,J3_1,K3_1;
  double sqrx1,sqrx2;
  double dx1,dx2,px12,px22,nx12,nx22;
  double maxvar;

  dx1 = 2.0*x1;
  dx2 = 2.0*x2;
  sqrx1 = SQR(x1);
  sqrx2 = SQR(x2);
  px12 = (1+sqrx1);
  px22 = (1+sqrx2);
  nx12 = (1-sqrx1);
  nx22 = (1-sqrx2);
  
  K2_1 = (K2cc*nx12+K2cs*dx1+K2c1*px12)*nx22+(K2sc*nx12+K2ss*dx1)*dx2+(K21c*nx12+K21s*dx1+K211*px12)*px22;
  maxvar = fabs(K2_1);
								
  //L2_1 = K2_1;
  
  I3_1 = (I3cc*nx12+I3cs*dx1+I3c1*px12)*nx22+(I3sc*nx12+I3ss*dx1+I3s1*px12)*dx2+(I31c*nx12+I31s*dx1+I311*px12)*px22;
  if(fabs(I3_1) > maxvar)
    maxvar = fabs(I3_1);
  
  J3_1 = (J3cc*nx12+J3cs*dx1+J3c1*px12)*nx22+(J3sc*nx12+J3ss*dx1+J3s1*px12)*dx2+(J31c*nx12+J31s*dx1+J311*px12)*px22;
  if(fabs(J3_1) > maxvar)
    maxvar = fabs(J3_1);
  
  K3_1 = (K3cc*nx12+K3cs*dx1+K3c1*px12)*nx22+(K3sc*nx12+K3ss*dx1+K3s1*px12)*dx2+(K31c*nx12+K31s*dx1+K311*px12)*px22;
  if(fabs(K3_1) > maxvar)
    maxvar = fabs(K3_1);
  
  //L3_1 = -K3_1;

  K2_1 /= maxvar;
  I3_1 /= maxvar;
  J3_1 /= maxvar;
  K3_1 /= maxvar;
  
  ecart = (SQR(J3_1)+SQR(K3_1))*(2*K2_1)-(SQR(K2_1))*(K3_1*(J3_1-I3_1));

  return (ecart);
}


/*******************************************************************************/

static int bio_compute_the_solutions(p3d_cntrt *ct, poly_max16_coefs *xx1, poly_max16_coefs *xx2, double **sol_configs)
{
  int i,nsol;

  nsol = 0;
  for(i = 0; i < (xx1->order + 1); i++) {
    if(bio_compute_solution_for_couple(ct,xx1->a_i[i],xx2->a_i[i],sol_configs[i])) {
      nsol++;
    }
    else {
      i--;
    }
  }

  return (nsol);
}


/*******************************************************************************/

static int bio_compute_solution_for_couple(p3d_cntrt *ct, double x1, double x2, double *sol)
{
  double sqrx1,sqrx2,dx1,dx2,fx1x2,sqrx1sqrx2,denx1x2;
  double pal_1,pbe_1,pga_1,pa_1,pb_1,pc_1,ps_1,psi_1,pu_1,pv_1,pw_1,pl_1,pm_1,pn_1;
  double psp,pgap,psip,pcp,pwp,pnp,plabep,pmubep,plabp,plavp,pmuvp,plamp,pmump,palp,pbep;
  p3d_matrix4 T21,T10,T06,T65,T54,T43,T20,T26,T25,T24,T23;
  static double C1,S1,C2,S2,C3,S3,C4,S4,C5,S5,C6,S6;
  bio_6R_ik_data *ikd = ct->bio_ik_data;
  int i;

  sqrx1 = SQR(x1);
  sqrx2 = SQR(x2);
  dx1 = 2*x1;
  dx2 = 2*x2;
  fx1x2 = 4*x2*x1;
  sqrx1sqrx2 = sqrx1*sqrx2;
  denx1x2 = (1+sqrx1)*(1+sqrx2);


  //###############################################################################
  //# Calcul des solutions liees a un couple compatible (x1, x2)                  #
  //###############################################################################
  
  //# fonctions de x1 et x2

  pal_1 = (ikd->palcc + ikd->palc1 + ikd->pal1c + ikd->pal11
	    + (ikd->palcs + ikd->pal1s + (-ikd->palcs + ikd->pal1s)*sqrx2)*dx1
	    + (ikd->palsc - ikd->palsc*sqrx1)*dx2
	    + (-ikd->palcc + ikd->palc1 - ikd->pal1c + ikd->pal11)*sqrx1
	    + (-ikd->palcc - ikd->palc1 + ikd->pal1c + ikd->pal11)*sqrx2
	    + (ikd->palcc - ikd->palc1 - ikd->pal1c + ikd->pal11)*sqrx1sqrx2
	    + ikd->palss*fx1x2)/denx1x2;

  pbe_1 = (ikd->pbecc + ikd->pbec1 + ikd->pbe1c + ikd->pbe11
	    + (-ikd->pbecc + ikd->pbec1 - ikd->pbe1c + ikd->pbe11)*sqrx1
	    + (-ikd->pbecc - ikd->pbec1 + ikd->pbe1c + ikd->pbe11)*sqrx2
	    + (ikd->pbecc - ikd->pbec1 - ikd->pbe1c + ikd->pbe11)*sqrx1sqrx2
	    + ikd->pbess*fx1x2)/denx1x2;

  pga_1 = (ikd->pgacc + ikd->pgac1 + ikd->pga1c + ikd->pga11
	    + (ikd->pgacs + ikd->pga1s + (-ikd->pgacs + ikd->pga1s)*sqrx2)*dx1
	    + (ikd->pgasc - ikd->pgasc*sqrx1)*dx2
	    + (-ikd->pgacc + ikd->pgac1 - ikd->pga1c + ikd->pga11)*sqrx1
	    + (-ikd->pgacc - ikd->pgac1 + ikd->pga1c + ikd->pga11)*sqrx2
	    + (ikd->pgacc - ikd->pgac1 - ikd->pga1c + ikd->pga11)*sqrx1sqrx2
	    + ikd->pgass*fx1x2)/denx1x2;

  pa_1 = (ikd->pacc + ikd->pac1 + ikd->pa1c + ikd->pa11
	   + (ikd->pacs + ikd->pa1s + (-ikd->pacs + ikd->pa1s)*sqrx2)*dx1
	   + (ikd->pasc - ikd->pasc*sqrx1)*dx2
	   + (-ikd->pacc + ikd->pac1 - ikd->pa1c + ikd->pa11)*sqrx1
	   + (-ikd->pacc - ikd->pac1 + ikd->pa1c + ikd->pa11)*sqrx2
	   + (ikd->pacc - ikd->pac1 - ikd->pa1c + ikd->pa11)*sqrx1sqrx2
	   + ikd->pass*fx1x2)/denx1x2;

  pb_1 = (ikd->pbcc + ikd->pbc1 + ikd->pb1c + ikd->pb11
	   + (-ikd->pbcc + ikd->pbc1 - ikd->pb1c + ikd->pb11)*sqrx1
	   + (-ikd->pbcc - ikd->pbc1 + ikd->pb1c + ikd->pb11)*sqrx2
	   + (ikd->pbcc - ikd->pbc1 - ikd->pb1c + ikd->pb11)*sqrx1sqrx2
	   + ikd->pbss*fx1x2)/denx1x2;

  pc_1 = (ikd->pccc + ikd->pcc1 + ikd->pc1c + ikd->pc11
	   + (ikd->pccs + ikd->pc1s + (-ikd->pccs + ikd->pc1s)*sqrx2)*dx1
	   + (ikd->pcsc - ikd->pcsc*sqrx1)*dx2
	   + (-ikd->pccc + ikd->pcc1 - ikd->pc1c + ikd->pc11)*sqrx1
	   + (-ikd->pccc - ikd->pcc1 + ikd->pc1c + ikd->pc11)*sqrx2
	   + (-ikd->pc1c + ikd->pccc - ikd->pcc1 + ikd->pc11)*sqrx1sqrx2
	   + ikd->pcss*fx1x2)/denx1x2;

  ps_1 = (ikd->pscc + ikd->psc1 + ikd->ps1c + ikd->ps11
	   + (ikd->pscs + ikd->ps1s + (-ikd->pscs + ikd->ps1s)*sqrx2)*dx1
	   + (ikd->pssc - ikd->pssc*sqrx1)*dx2
	   + (-ikd->pscc + ikd->psc1 - ikd->ps1c + ikd->ps11)*sqrx1
	   + (-ikd->pscc - ikd->psc1 + ikd->ps1c + ikd->ps11)*sqrx2
	   + (-ikd->ps1c + ikd->pscc - ikd->psc1 + ikd->ps11)*sqrx1sqrx2
	   + ikd->psss*fx1x2)/denx1x2;

  psi_1 = (ikd->psicc + ikd->psic1 + ikd->psi1c + ikd->psi11
	   + (ikd->psics + ikd->psi1s + (-ikd->psics + ikd->psi1s)*sqrx2)*dx1
	   + (ikd->psisc - ikd->psisc*sqrx1)*dx2
	   + (-ikd->psicc + ikd->psic1 - ikd->psi1c + ikd->psi11)*sqrx1
	   + (-ikd->psicc - ikd->psic1 + ikd->psi1c + ikd->psi11)*sqrx2
	   + (-ikd->psi1c + ikd->psicc - ikd->psic1 + ikd->psi11)*sqrx1sqrx2
	   + ikd->psiss*fx1x2)/denx1x2;

  pu_1 = (ikd->pucc + ikd->puc1 + ikd->pu1c + ikd->pu11
	   + (ikd->pucs + ikd->pu1s + (-ikd->pucs + ikd->pu1s)*sqrx2)*dx1
	   + (ikd->pusc + ikd->pus1 + (-ikd->pusc + ikd->pus1)*sqrx1)*dx2
	   + (-ikd->pucc + ikd->puc1 - ikd->pu1c + ikd->pu11)*sqrx1
	   + (-ikd->pucc - ikd->puc1 + ikd->pu1c + ikd->pu11)*sqrx2
	   + (-ikd->pu1c + ikd->pucc - ikd->puc1 + ikd->pu11)*sqrx1*sqrx2
	   + ikd->puss*4*x2*x1)/denx1x2;

  pv_1 = (ikd->pvcc + ikd->pvc1 + ikd->pv1c + ikd->pv11
	   + (ikd->pvcs + ikd->pv1s + (-ikd->pvcs + ikd->pv1s)*sqrx2)*dx1
	   + (ikd->pvsc + ikd->pvs1 + (-ikd->pvsc + ikd->pvs1)*sqrx1)*dx2
	   + (-ikd->pvcc + ikd->pvc1 - ikd->pv1c + ikd->pv11)*sqrx1
	   + (-ikd->pvcc - ikd->pvc1 + ikd->pv1c + ikd->pv11)*sqrx2
	   + (-ikd->pv1c + ikd->pvcc - ikd->pvc1 + ikd->pv11)*sqrx1*sqrx2
	   + ikd->pvss*4*x2*x1)/denx1x2;

  pw_1 = (ikd->pwcc + ikd->pwc1 + ikd->pw1c + ikd->pw11
	   + (ikd->pwcs + ikd->pw1s + (-ikd->pwcs + ikd->pw1s)*sqrx2)*dx1
	   + (ikd->pwsc + ikd->pws1 + (-ikd->pwsc + ikd->pws1)*sqrx1)*dx2
	   + (-ikd->pwcc + ikd->pwc1 - ikd->pw1c + ikd->pw11)*sqrx1
	   + (-ikd->pwcc - ikd->pwc1 + ikd->pw1c + ikd->pw11)*sqrx2
	   + (-ikd->pw1c + ikd->pwcc - ikd->pwc1 + ikd->pw11)*sqrx1*sqrx2
	   + ikd->pwss*4*x2*x1)/denx1x2;

  pl_1 = (ikd->plcc + ikd->plc1 + ikd->pl1c + ikd->pl11
	   + (ikd->plcs + ikd->pl1s + (-ikd->plcs + ikd->pl1s)*sqrx2)*dx1
	   + (ikd->plsc + ikd->pls1 + (-ikd->plsc + ikd->pls1)*sqrx1)*dx2
	   + (-ikd->plcc + ikd->plc1 - ikd->pl1c + ikd->pl11)*sqrx1
	   + (-ikd->plcc - ikd->plc1 + ikd->pl1c + ikd->pl11)*sqrx2
	   + (-ikd->pl1c + ikd->plcc - ikd->plc1 + ikd->pl11)*sqrx1*sqrx2
	   + ikd->plss*4*x2*x1)/denx1x2;

  pm_1 = (ikd->pmcc + ikd->pmc1 + ikd->pm1c + ikd->pm11
	   + (ikd->pmcs + ikd->pm1s + (-ikd->pmcs + ikd->pm1s)*sqrx2)*dx1
	   + (ikd->pmsc + ikd->pms1 + (-ikd->pmsc + ikd->pms1)*sqrx1)*dx2
	   + (-ikd->pmcc + ikd->pmc1 - ikd->pm1c + ikd->pm11)*sqrx1
	   + (-ikd->pmcc - ikd->pmc1 + ikd->pm1c + ikd->pm11)*sqrx2
	   + (-ikd->pm1c + ikd->pmcc - ikd->pmc1 + ikd->pm11)*sqrx1*sqrx2
	   + ikd->pmss*4*x2*x1)/denx1x2;

  pn_1 = (ikd->pncc + ikd->pnc1 + ikd->pn1c + ikd->pn11
	   + (ikd->pncs + ikd->pn1s + (-ikd->pncs + ikd->pn1s)*sqrx2)*dx1
	   + (ikd->pnsc + ikd->pns1 + (-ikd->pnsc + ikd->pns1)*sqrx1)*dx2
	   + (-ikd->pncc + ikd->pnc1 - ikd->pn1c + ikd->pn11)*sqrx1
	   + (-ikd->pncc - ikd->pnc1 + ikd->pn1c + ikd->pn11)*sqrx2
	   + (-ikd->pn1c + ikd->pncc - ikd->pnc1 + ikd->pn11)*sqrx1*sqrx2
	   + ikd->pnss*4*x2*x1)/denx1x2;
  
  // ----------------------
  // ----------------------
  //# pour q1

  C1 = (1 - sqrx1)/(1 + sqrx1);
  S1 = dx1/(1 + sqrx1);

  // ----------------------
  //# pour q2

  C2 = (1 - sqrx2)/(1 + sqrx2);
  S2 = dx2/(1 + sqrx2);

  // ----------------------
  //# pour q4

  C4 = (ps_1 + ((ikd->r4) + (ikd->r5cal4)) * (ikd->cal3))/((ikd->r5) * (ikd->sal3sal4));
  S4 = -((ps_1 + (ikd->r4) * (ikd->cal3))*(pn_1 + psi_1 * pga_1) + (ikd->r5)*(((ikd->r4) * (ikd->cal4) + (ikd->r5))*(pc_1-pga_1 * ps_1) + (ikd->cal5)*(2.0 * psi_1-SQR(ps_1))))/((ikd->r4) * (ikd->r5) * (ikd->sal3sal4) * pw_1);

  // ----------------------
  //# pour q5
  
  //# Calcul des termes prime utiles #
  
  psp = (ikd->r5) * (ikd->sal3sal4) * C4 - ((ikd->r4) + (ikd->r5cal4)) * (ikd->cal3);
  pgap = -(ikd->sal3sal4) * C4 + (ikd->cal3cal4);
  psip = ((ikd->sqrr4) + (ikd->sqrr5))/2.0 + (ikd->r4) * (ikd->r5cal4);
  pcp = -(ikd->r4) * (ikd->cal4)-(ikd->r5);
  pwp = -(ikd->r4) * (ikd->sal3sal4) * S4;
  pnp = (-((ikd->sqrr4) - (ikd->sqrr5))/2.0 * (ikd->sal3sal4)) * C4 + (-(ikd->sqrr4) - (ikd->sqrr5))/2.0 * (ikd->cal3cal4) - (ikd->r4) * (ikd->r5) * (ikd->cal3);
  
  plabep = (ikd->sal3cal4) * C4 + (ikd->cal3sal4);
  pmubep = -(ikd->sal3) * S4;
  
  plabp = -(ikd->r4) * (ikd->sal4);
  //pmubp = 0.0;
  
  plavp = ((ikd->r4) * (ikd->cal4) + (ikd->r5)) * (ikd->sal3) * S4;
  pmuvp = ((ikd->r4) + (ikd->r5cal4)) * (ikd->sal3) * C4 + (ikd->r5) * (ikd->cal3sal4);
  
  plamp = (((ikd->sqrr4) + (ikd->sqrr5))/2.0 * (ikd->sal3cal4) + (ikd->r4) * (ikd->r5) * (ikd->sal3)) * C4
     + (-(ikd->sqrr4) + (ikd->sqrr5))/2.0 * (ikd->cal3sal4);
  pmump = -(((ikd->sqrr4) + (ikd->sqrr5))/2.0 + (ikd->r4) * (ikd->r5cal4)) * (ikd->sal3) * S4;  
 
  if((ikd->sal5) != 0.0) { 
    if(pwp != 0.0) {
      C5 = (pmubep * (pc_1 - (ikd->cal5) * pcp))/((ikd->sal5) * pwp);
      S5 = (-plabep * (pc_1 - (ikd->cal5) * pcp) + plabp * (pga_1-(ikd->cal5) * pgap))/((ikd->sal5) * pwp);
    }
    else if((psp * pgap - pcp) != 0.0) {
      C5 = (pmubep * ((ikd->sal5) * pw_1 - (ikd->cal5sal5) * pwp) - pmuvp * (pga_1 - (ikd->cal5) * pgap) * (ikd->sal5))/((ikd->sqrsal5) * (psp * pgap-pcp));
      S5 = (-plabep * ((ikd->sal5) * pw_1 - (ikd->cal5sal5) * pwp) + plavp * (pga_1 - (ikd->cal5) * pgap) * (ikd->sal5))/((ikd->sqrsal5) * (psp * pgap-pcp));
    }
    else if((pnp + psip * pgap) != 0.0) {
      C5 = (-pmuvp * (pc_1 - (ikd->cal5) * pcp))/((ikd->sqrsal5) * (pnp + psip * pgap));
      S5 = (-plabp * ((ikd->sal5) * pw_1 - (ikd->cal5sal5) * pwp) + plavp * (pc_1 - (ikd->cal5) * pcp))/((ikd->sqrsal5) * (pnp + psip * pgap));
    }
    else if((psp * pnp + psip * pcp) != 0.0) {
      C5 = (pmuvp * ((ikd->sqrsal5) * pn_1 - (ikd->sqrsal5) * (ikd->cal5) * pnp)-pmump * ((ikd->sal5) * pw_1 - (ikd->cal5sal5) * pwp) * (ikd->sal5))/((ikd->sal5) * (ikd->sqrsal5) * (psp * pnp + psip * pcp));
      S5 = (-plavp * ((ikd->sqrsal5) * pn_1 - (ikd->sqrsal5) * (ikd->cal5) * pnp) + plamp * ((ikd->sal5) * pw_1 - (ikd->cal5sal5) * pwp) * (ikd->sal5))/((ikd->sal5) * (ikd->sqrsal5) * (psp * pnp + psip * pcp));
    }
  }
  else {
    //else #A FAIRE 
  }
  
  // ----------------------
  //# pour q6
  
  //# Calcul des autres termes prime utiles #
  
  palp = C5 * (ikd->sal3) * S4 + S5 * ((ikd->sal3cal4) * C4 + (ikd->cal3sal4));
  pbep = -S5 * (ikd->sal3) * S4 + C5 * ((ikd->sal3cal4) * C4 + (ikd->cal3sal4));

  C6 = (palp * pal_1 + ((ikd->cal5) * pbep + (ikd->sal5) * pgap) * pbe_1)/
    (SQR(palp) + SQR((ikd->cal5) * pbep + (ikd->sal5) * pgap));
  S6 = (-palp * pbe_1 + ((ikd->cal5) * pbep + (ikd->sal5) * pgap) * pal_1)/
    (SQR(palp) + SQR((ikd->cal5) * pbep + (ikd->sal5) * pgap));

  // ----------------------
  //# pour q3

  p3d_set_matrix4_values(T21,
			 C2,(ikd->cal1) * S2,(ikd->sal1) * S2,0,
			 -S2,(ikd->cal1) * C2,(ikd->sal1) * C2,0,
			 0,-(ikd->sal1),(ikd->cal1),-(ikd->r2),
			 0,0,0,1);

  p3d_set_matrix4_values(T10,
			 C1,S1,0,0,
			 -S1,C1,0,0,
			 0,0,1,0,
			 0,0,0,1);

  p3d_set_matrix4_values(T06,
			 (ikd->cth7),0,-(ikd->sth7),-(ikd->a7),
			 -(ikd->cal7) * (ikd->sth7),(ikd->sal7),-(ikd->cal7) * (ikd->cth7),-(ikd->r7) * (ikd->sal7),
			 (ikd->sal7) * (ikd->sth7),(ikd->cal7),(ikd->sal7) * (ikd->cth7),-(ikd->r7) * (ikd->cal7) - (ikd->r0),
			 0,0,0,1);

  p3d_set_matrix4_values(T65,
			 C6,(ikd->cal5) * S6,(ikd->sal5) * S6,0,
			 -S6,(ikd->cal5) * C6,(ikd->sal5) * C6,0,
			 0,-(ikd->sal5),(ikd->cal5),0,
			 0,0,0,1);

  p3d_set_matrix4_values(T54,
			 C5,(ikd->cal4) * S5,(ikd->sal4) * S5,0,
			 -S5,(ikd->cal4) * C5,(ikd->sal4) * C5,0,
			 0,-(ikd->sal4),(ikd->cal4),-(ikd->r5),
			 0,0,0,1);

  p3d_set_matrix4_values(T43,
			 C4,(ikd->cal3) * S4,(ikd->sal3) * S4,0,
			 -S4,(ikd->cal3) * C4,(ikd->sal3) * C4,0,
			 0,-(ikd->sal3),(ikd->cal3),-(ikd->r4),
			 0,0,0,1);

  p3d_mat4Mult(T21,T10,T20);
  p3d_mat4Mult(T20,T06,T26);
  p3d_mat4Mult(T26,T65,T25);
  p3d_mat4Mult(T25,T54,T24);
  p3d_mat4Mult(T24,T43,T23);

  C3 = T23[0][0];
  S3 = -T23[0][1];

  // VERIFICATION
  if(!bio_verify_solution(ct,C3,S3,T06,T20,T43,T54,T65)) {
    return 0; 
  }

  //###############################################################################

  // soluion in (-PI,PI) 
  sol[0] = atan2(S1,C1) - ikd->th1_i;
/*   if(sol[0] < -M_PI) */
/*     sol[0] += (2.0*M_PI); */
  sol[1] = atan2(S2,C2);
/*   if(sol[1] < 0.0) */
/*     sol[1] += (2.0*M_PI); */
  sol[2] = atan2(S3,C3);
/*   if(sol[2] < 0.0) */
/*     sol[2] += (2.0*M_PI); */
  sol[3] = atan2(S4,C4);
/*   if(sol[3] < 0.0) */
/*     sol[3] += (2.0*M_PI); */
  sol[4] = atan2(S5,C5);
/*   if(sol[4] < 0.0) */
/*     sol[4] += (2.0*M_PI); */
  sol[5] = atan2(S6,C6);
/*   if(sol[5] < 0.0) */
/*     sol[5] += (2.0*M_PI); */

  // soluion in (-PI,PI) 
  for(i=0; i<6; i++) {
    if(sol[i] < -M_PI)
     sol[i] += (2.0*M_PI);
    if(sol[i] > M_PI)
     sol[i] -= (2.0*M_PI);    
  }

  return 1;
}


/*******************************************************************************/

static int bio_verify_solution(p3d_cntrt *ct, double C3, double S3, p3d_matrix4 T06, p3d_matrix4 T20, p3d_matrix4 T43, p3d_matrix4 T54, p3d_matrix4 T65)
{
  p3d_matrix4 TB,TA;
  bio_6R_ik_data *ikd = ct->bio_ik_data;

  p3d_set_matrix4_values(TB,
			 C3,(ikd->cal2)*S3,(ikd->sal2)*S3,0,
			 -S3,(ikd->cal2)*C3,(ikd->sal2)*C3,0,
			 0,-(ikd->sal2),(ikd->cal2),-(ikd->r3),
			 0,0,0,1);

  p3d_mat4Mult(TB,T20,TA);
  p3d_mat4Mult(T43,TA,TB);
  p3d_mat4Mult(T54,TB,TA);
  p3d_mat4Mult(T65,TA,TB);
 
  p3d_mat4Mult(TB,T06,TA);
  if(SQR(TA[0][1]) + SQR(TA[0][2]) + SQR(TA[0][3]) + 
     SQR(TA[1][0]) + SQR(TA[1][2]) + SQR(TA[1][3]) + 
     SQR(TA[2][0]) + SQR(TA[2][1]) + SQR(TA[2][3]) > BIO_IK_ERROR_TOL) {
    return 0;
  }
  else {
    return 1;
  }

}

/*******************************************************************************/



/*******************************************************************************/
/* FOR POLYNOMIALS !!! */

#ifdef USE_GSL
static int solve_polynomial_eq(poly_max16_coefs *polcfs, poly_max16_coefs *sol)
{
  int i,nsols;
  int polsize;
  gsl_poly_complex_workspace * w;
  double z[32];
  double maxcoef,mincoef;

  /* verify polynomial conditioning */
  maxcoef = -1E10;
  mincoef =  1E10;
  for(i = 0; i < 16; i++) { 
    if(polcfs->a_i[i] > maxcoef)
      maxcoef = polcfs->a_i[i];
    if(polcfs->a_i[i] < mincoef)
      mincoef = polcfs->a_i[i];
  }

  /* NOTE : I've made tests and none solution is found with conditionong > 100000 
            but there are problems from 1000
  */
  //printf("maxcoef = %f, mincoef = %f, condit = %f\n",maxcoef,mincoef,maxcoef/mincoef);
/*   if((maxcoef/mincoef) > 10000.0) { */
/*     sol->order = - 1; */
/*     return 0; */
/*   } */

  polsize = polcfs->order + 1;
  w = gsl_poly_complex_workspace_alloc(polsize);

  gsl_poly_complex_solve (polcfs->a_i, polsize, w, z);

  gsl_poly_complex_workspace_free(w);

  nsols = 0;
  for (i = 0; i < 16; i++) { 
    if(fabs(z[2*i+1]) < BIO_IK_ERROR_TOL) {
      sol->a_i[nsols] = z[2*i];
      nsols++; 
    }
  }
  sol->order = nsols - 1;

/*   if(nsols && ((maxcoef/mincoef) > 100000.0)) */
/*     printf("Solving with conditionig > 100000\n"); */

  return 1;
}
#endif

/*************************************************************************************/

static void copy_poly16(poly_max16_coefs *in, poly_max16_coefs *out)
{
  int i;

  for(i = 0; i <= in->order; i++) {
    out->a_i[i] = in->a_i[i];
  }
  out->order = in->order;
}


/*************************************************************************************/

static void print_poly16(poly_max16_coefs *pol)
{
  
  int i;
  
  printf("\npoly.order = %d\n\n", pol->order);
  for(i=0; i <= pol->order; i++) {
    printf("a%d = %f\n", i, pol->a_i[i]);
  }
  printf("\n");
}


/*************************************************************************************/

static void prod_of_8_poly2(double k, poly2_coefs pol1, poly2_coefs pol2, poly2_coefs pol3, 
			    poly2_coefs pol4, poly2_coefs pol5, poly2_coefs pol6, poly2_coefs pol7,
			    poly2_coefs pol8, poly_max16_coefs *prod_pol)
{

  int ipol,oprod,i;
  double *prodcfs,*nprodcfs,*paux;
  double matcoefs[8][3];

  prodcfs = (double *)malloc(sizeof(double) * 17);
  nprodcfs = (double *)malloc(sizeof(double) * 17);
  
  matcoefs[0][0] = prodcfs[0] = pol1[0]; 
  matcoefs[0][1] = prodcfs[1] = pol1[1]; 
  matcoefs[0][2] = prodcfs[2] = pol1[2]; 
  matcoefs[1][0] = pol2[0]; 
  matcoefs[1][1] = pol2[1]; 
  matcoefs[1][2] = pol2[2]; 
  matcoefs[2][0] = pol3[0]; 
  matcoefs[2][1] = pol3[1]; 
  matcoefs[2][2] = pol3[2]; 
  matcoefs[3][0] = pol4[0]; 
  matcoefs[3][1] = pol4[1]; 
  matcoefs[3][2] = pol4[2]; 
  matcoefs[4][0] = pol5[0]; 
  matcoefs[4][1] = pol5[1]; 
  matcoefs[4][2] = pol5[2]; 
  matcoefs[5][0] = pol6[0]; 
  matcoefs[5][1] = pol6[1]; 
  matcoefs[5][2] = pol6[2]; 
  matcoefs[6][0] = pol7[0]; 
  matcoefs[6][1] = pol7[1]; 
  matcoefs[6][2] = pol7[2]; 
  matcoefs[7][0] = pol8[0]; 
  matcoefs[7][1] = pol8[1]; 
  matcoefs[7][2] = pol8[2]; 

  for(ipol=1; ipol < 8; ipol++) {
    oprod = 2*(ipol + 1);
    nprodcfs[0] = prodcfs[0]*matcoefs[ipol][0];
    nprodcfs[1] = prodcfs[0]*matcoefs[ipol][1] + prodcfs[1]*matcoefs[ipol][0];
    for(i=2; i < (oprod-1); i++) {
      nprodcfs[i] = prodcfs[i-2]*matcoefs[ipol][2] + prodcfs[i-1]*matcoefs[ipol][1] + prodcfs[i]*matcoefs[ipol][0];
    }
    nprodcfs[oprod-1] = prodcfs[oprod-3]*matcoefs[ipol][2] + prodcfs[oprod-2]*matcoefs[ipol][1];
    nprodcfs[oprod] = prodcfs[oprod-2]*matcoefs[ipol][2];
    //COPY(nprodcfs,prodcfs)
    paux = prodcfs;
    prodcfs = nprodcfs;
    nprodcfs = paux;
  }

  //COPY(prodcfs,prod_pol) and mult k
  for(i=0; i<17; i++) {
    prod_pol->a_i[i] = prodcfs[i] * k;
  }
  //obtain order
  i = 16;
  while(prodcfs[i] == 0.0) {
    i--;
  }
  prod_pol->order = i;

  free(prodcfs);
  free(nprodcfs);
}


/*************************************************************************************/

/* la solition est copiee dans pol2 ! */
static void add_2_poly16(poly_max16_coefs *pol1, poly_max16_coefs *pol2)
{

  int i;
  poly_max16_coefs *lonpol,*shopol;
  
  if(pol1->order > pol1->order) {
    lonpol = pol1;
    shopol = pol2;
  }
  else {
    lonpol = pol2;
    shopol = pol1;
  }

  for(i = 0; i <= lonpol->order; i++) {
    if(i <= shopol->order) 
      pol2->a_i[i] = lonpol->a_i[i] + shopol->a_i[i];
    else
      pol2->a_i[i] = lonpol->a_i[i];
  }
  pol2->order = lonpol->order;
}

/*******************************************************************************/
/*******************************************************************************/
/* FOR MATRIX !!!*/

void p3d_set_matrix4_values(p3d_matrix4 M,
			    double m11, double m12, double m13, double m14,
			    double m21, double m22, double m23, double m24,
			    double m31, double m32, double m33, double m34,
			    double m41, double m42, double m43, double m44)
{

  M[0][0] = m11;   M[0][1] = m12; M[0][2] = m13;  M[0][3] = m14; 
  M[1][0] = m21;   M[1][1] = m22; M[1][2] = m23;  M[1][3] = m24; 
  M[2][0] = m31;   M[2][1] = m32; M[2][2] = m33;  M[2][3] = m34; 
  M[3][0] = m41;   M[3][1] = m42; M[3][2] = m43;  M[3][3] = m44; 

}

/*******************************************************************************/

void p3d_rotate_vect_around_axis(p3d_vector3 axis, double theta, p3d_vector3 v_in, p3d_vector3 v_out)
{
  double ct,st,vt;
  p3d_vector3 naxis;
  p3d_matrix3 Mrot;

  ct = cos(theta);
  st = sin(theta);
  vt = 1.0 - ct;
  p3d_vectNormalize(axis,naxis);
  
  Mrot[0][0] = (SQR(naxis[0]) * vt) + ct;
  Mrot[0][1] = (naxis[0] * naxis[1] * vt) - (naxis[2] * st);
  Mrot[0][2] = (naxis[0] * naxis[2] * vt) + (naxis[1] * st);
  Mrot[1][0] = (naxis[0] * naxis[1] * vt) + (naxis[2] * st);
  Mrot[1][1] = (SQR(naxis[1]) * vt) + ct;
  Mrot[1][2] = (naxis[1] * naxis[2] * vt) - (naxis[0] * st);
  Mrot[2][0] = (naxis[0] * naxis[2] * vt) - (naxis[1] * st);
  Mrot[2][1] = (naxis[1] * naxis[2] * vt) + (naxis[0] * st);
  Mrot[2][2] = (SQR(naxis[2]) * vt) + ct;

  p3d_vec3Mat3Mult(Mrot,v_in,v_out);

}

/*******************************************************************************/

/*******************************************************************************/

