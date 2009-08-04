/*
 * Copyright (c) 1997 LAAS/CNRS -- RIA --
 * Daniel SIDOBRE -- mai 2002
 */

/* #include <errno.h> */

#include <math.h>
#include <stdio.h>


#ifndef M_PI
#define	M_PI		3.14159265358979323846
#endif
#ifndef M_PI_2
#define	M_PI_2		1.57079632679489661923
#endif
#ifndef M_PI_4
#define	M_PI_4		0.78539816339744830962
#endif


#include "gb.h"
#include "SProto_gbModeles.h"

char* Gb_statusMGI_s(Gb_statusMGI u)
{
  switch (u) {
  case MGI_OK : return (char*)"MGI_OK";
  case MGI_ERROR : return (char*)"MGI_ERROR";
  case MGI_APPROXIMATE : return (char*)"MGI_APPROXIMATE";
  case   MGI_SINGULAR : return (char*)"MGI_SINGULAR";
  default : return (char*)"ERROR_SWITCH";
  }
}

#define SIGN(x) (((x) < 0 ) ? (-1) : (1))

double Gb_atan2_offset(double y, double x, double offset, double oldQ)
{
  double a = atan2(y, x) + offset;
  if ( (a - oldQ) > M_PI) {
    a -= 2*M_PI;
  } else {
    if ( (a - oldQ) <= -M_PI)
      a += 2*M_PI;
  }
  return a;
}

double Gb_atan2(double y, double x)
{
  if (x == 0.) {
    if (y == 0.) {
      return 0.;
    } else {
      if (y < 0)
	return - M_PI / 2.;
      else
	return M_PI / 2.;
    }
  } else {
    return atan2(y, x);
  }
}

void Gb_dataMGD_print(FILE* output, Gb_dataMGD *d)
{
  fprintf(output, "Gb_dataMGD= \n");
  fprintf(output, "  d1-12= %g %g %g  %g %g %g  %g %g %g  %g %g %g\n",
	  d->d1, d->d2, d->d3, d->d4, d->d5, d->d6,
	  d->d7, d->d8, d->d9, d->d10, d->d11, d->d12);
  fprintf(output, "  c1-c6= %g %g %g  %g %g %g\n",
	  d->c1, d->c2, d->c3, d->c4, d->c5, d->c6);
  fprintf(output, "  s1-s6= %g %g %g  %g %g %g\n",
	  d->s1, d->s2, d->s3, d->s4, d->s5, d->s6);
  fprintf(output, "  c23= %g s23= %g  e1-e3= %d %d %d\n",
	  d->c23, d->s23, d->e1, d->e2, d->e3);
}


/*
 * compute the direct geometric model of a 6R arm robot
 * Inputs :
 *   bras : constants of the arm
 *   eq : articulation coordinates
 * output :
 *   d : Gb_dataMGD structure for computation
 *   th : cartesian coordinates
 */
void Gb_MGD6rTh(Gb_6rParameters* bras, Gb_q6* eq,
				     Gb_dataMGD* d, Gb_th* th)
{
  double m;
  d->c1 = cos(eq->q1 - bras->of1);
  d->c2 = cos(eq->q2 - bras->of2);
  d->c3 = cos(eq->q3 - bras->of3);
  d->c4 = cos(eq->q4 - bras->of4);
  d->c5 = cos(eq->q5 - bras->of5);
  d->c6 = cos(eq->q6 - bras->of6);
  d->s1 = sin(eq->q1 - bras->of1);
  d->s2 = sin(eq->q2 - bras->of2);
  d->s3 = sin(eq->q3 - bras->of3);
  d->s4 = sin(eq->q4 - bras->of4);
  d->s5 = sin(eq->q5 - bras->of5);
  d->s6 = sin(eq->q6 - bras->of6);
/*   Gb_dataMGD_print(stdout, d); */
  d->c23 = d->c2 * d->c3 - d->s2 * d->s3;
  d->s23 = d->s2 * d->c3 + d->c2 * d->s3;
  d->d1  = d->c2 * bras->a2;
  d->d2  = d->s2 * bras->a2;
  d->d3  = d->c5 * d->c6;
  d->d4  =-d->s5 * d->c6;
  d->d5  = d->c4 * d->d3 - d->s4 * d->s6;
  d->d6  = d->s4 * d->d3 + d->c4 * d->s6;
  d->d7  = d->c4 * d->s5;
  d->d8  = d->s4 * d->s5;
  d->d9    = d->c23 * d->d5 + d->s23 * d->d4;
  th->vx.z = d->s23 * d->d5 - d->c23 * d->d4;
  d->d10   = d->c23 * d->d7 + d->s23 * d->c5;
  th->vz.z = d->s23 * d->d7 - d->c23 * d->c5;
  d->d11   = d->s23 * bras->r4 + d->d1;
  th->vp.z =-d->c23 * bras->r4 + d->d2;
  th->vx.x = d->c1 * d->d9  +  d->s1 * d->d6;
  th->vx.y = d->s1 * d->d9  -  d->c1 * d->d6;
  th->vz.x = d->c1 * d->d10 +  d->s1 * d->d8;
  th->vz.y = d->s1 * d->d10 -  d->c1 * d->d8;
  th->vp.x = d->c1 * d->d11;
  th->vp.y = d->s1 * d->d11;
  d->e1 = (d->d11 >= 0) ? 1. : -1.;
  d->e2 = (d->c3  >= 0) ? 1. : -1.;
  d->e3 = (d->s5  >= 0) ? 1. : -1.;

  m = th->vx.x * th->vx.x + th->vx.y * th->vx.y + th->vx.z * th->vx.z;
  th->vx.x /= m;
  th->vx.y /= m;
  th->vx.z /= m;
  Gb_v3_cross_product (&(th->vz), &(th->vx), &(th->vy));
  m = th->vy.x * th->vy.x + th->vy.y * th->vy.y + th->vy.z * th->vy.z;
  th->vy.x /= m;
  th->vy.y /= m;
  th->vy.z /= m;
  Gb_v3_cross_product(&(th->vx), &(th->vy), &(th->vz));
}

/*
 * compute the configuration e1 e2 e3 for a 6R arm robot
 * Inputs :
 *   bras : constants of the arm
 *   eq : articulation coordinates
 * output :
 *   e1 : 1 ou -1
 *   e2 : 1 ou -1
 *   e3 : 1 ou -1
 */
void Gb_MGD6r_gete1e2e3(Gb_6rParameters* bras, Gb_q6* eq,
			int* e1, int* e2, int* e3)
{
  double c2 = cos(eq->q2 - bras->of2);
  double c3 = cos(eq->q3 - bras->of3);
  double s2 = sin(eq->q2 - bras->of2);
  double s3 = sin(eq->q3 - bras->of3);
  double s5 = sin(eq->q5 - bras->of5);
  double s23 = s2 * c3 + c2 * s3;
  double d1  = c2 * bras->a2;
  double d11   = s23 * bras->r4 + d1;
  *e1 = (d11 >= 0) ? 1. : -1.;
  *e2 = (c3  >= 0) ? 1. : -1.;
  *e3 = (s5  >= 0) ? 1. : -1.;
}

/*
 * compute the direct geometric model of a 6R arm robot and the
 *   intermediates matrix
 * Inputs :
 *   bras : constants of the arm
 *   eq : articulation coordinates
 * output :
 *   d : Gb_dataMGD structure for computation
 *   th01, th02, th03, th04, th05, th06 : cartesian coordinates
 *
 */
void Gb_MGD6r_6Th(Gb_6rParameters* bras, Gb_q6* eq, Gb_dataMGD* d,
                Gb_th* th01, Gb_th* th02, Gb_th* th03,
		Gb_th* th04, Gb_th* th05, Gb_th* th06)
{
  Gb_MGD6rTh(bras, eq, d, th06);

  th02->vx.x = d->c1 * d->c2;
  th02->vx.y = d->s1 * d->c2;
  th02->vx.z = d->s2;
  th02->vy.x =-d->c1 * d->s2;
  th02->vy.y =-d->s1 * d->s2;
  th02->vy.z = d->c2;
  th02->vz.x = d->s1;
  th02->vz.y =-d->c1;
  th02->vz.z = 0;
  th02->vp.x = 0;
  th02->vp.y = 0;
  th02->vp.z = 0;

  th03->vx.x = th02->vx.x * d->c3 + th02->vy.x * d->s3;
  th03->vx.y = th02->vx.y * d->c3 + th02->vy.y * d->s3;
  th03->vx.z = d->s23;
  th03->vy.x =-th02->vx.x * d->s3 + th02->vy.x * d->c3;
  th03->vy.y =-th02->vx.y * d->s3 + th02->vy.y * d->c3;
  th03->vy.z = d->c23;
  th03->vz.x = d->s1;
  th03->vz.y =-d->c1;
  th03->vz.z = 0;
  th03->vp.x = th02->vx.x * bras->a2;
  th03->vp.y = th02->vx.y * bras->a2;
  th03->vp.z = d->s2 * bras->a2;

  th04->vx.x = th03->vx.x * d->c4 + d->s1 * d->s4;
  th04->vx.y = th03->vx.y * d->c4 - d->c1 * d->s4;
  th04->vx.z = d->s23 * d->c4;
  th04->vy.x =-th03->vx.x * d->s4 + d->s1 * d->c4;
  th04->vy.y =-th03->vx.y * d->s4 - d->c1 * d->c4;
  th04->vy.z =-d->s23 * d->s4;
  th04->vz.x =-th03->vy.x;
  th04->vz.y =-th03->vy.y;
  th04->vz.z =-d->c23;
  th04->vp.x =-th03->vy.x * bras->r4 + th03->vp.x;
  th04->vp.y =-th03->vy.y * bras->r4 + th03->vp.y;
  th04->vp.z =-d->c23 * bras->r4 + th03->vp.z;

  th05->vx.x = th04->vx.x * d->c5 - th04->vz.x * d->s5;
  th05->vx.y = th04->vx.y * d->c5 - th04->vz.y * d->s5;
  th05->vx.z = th04->vx.z * d->c5 + d->c23 * d->s5;
  th05->vy.x =-th04->vx.x * d->s5 - th04->vz.x * d->c5;
  th05->vy.y =-th04->vx.y * d->s5 - th04->vz.y * d->c5;
  th05->vy.z =-th04->vx.z * d->s5 + d->c23 * d->c5;
  th05->vz.x = th04->vy.x;
  th05->vz.y = th04->vy.y;
  th05->vz.z = th04->vy.z;
  th05->vp.x = th04->vp.x;
  th05->vp.y = th04->vp.y;
  th05->vp.z = th04->vp.z;

#ifdef DEBUG_GB
#define ltest(a,b) if (fabs(a-(b)) > 1e-10)\
    fprintf(stderr, "Gb_MGD6r_6Th Pb %s %g %g\n", #a , a, b)
  ltest(th06->vx.x,  th05->vx.x * d->c6 + th05->vz.x * d->s6);
  ltest(th06->vx.y,  th05->vx.y * d->c6 + th05->vz.y * d->s6);
  ltest(th06->vx.z,  th05->vx.z * d->c6 + th05->vz.z * d->s6);
  ltest(th06->vy.x, -th05->vx.x * d->s6 + th05->vz.x * d->c6);
  ltest(th06->vy.y, -th05->vx.y * d->s6 + th05->vz.y * d->c6);
  ltest(th06->vy.z, -th05->vx.z * d->s6 + th05->vz.z * d->c6);
  ltest(th06->vz.x, -th05->vy.x);
  ltest(th06->vz.y, -th05->vy.y);
  ltest(th06->vz.z, -th05->vy.z);
  ltest(th06->vp.x,  th05->vp.x);
  ltest(th06->vp.y,  th05->vp.y);
  ltest(th06->vp.z,  th05->vp.z);
#undef ltest
#endif
}


/*
 * Gb_MGI6rTh : compute the inverse geometric model of a 6R arm
 *  Allways return a value in sq. If the value is an exact value the
 *   function return MGI_OK or MGI_SINGULAR if it is a singular
 *   position. If the value is an approximate value the function
 *   return MGI_APPROXIMATE.
 *  The solution returned is defined by e1, e2 and e3.
 *  For singular position the function return the solution closer to
 *   the old_q value.
 *
 * input :
 *   bras : constant of the arm
 *   eth : cartesien coordinate
 *   e1, e2, e3 : defined the solution returned (problem has 8 solution)
 * output:
 *   d : Gb_dataMGD structure for computation
 *   sq : articulation coordinates
 * return : MGI_OK, MGI_SINGULAR or MGI_APPROXIMATE
 */
Gb_statusMGI Gb_MGI6rTh(Gb_6rParameters* bras, Gb_th* eth,
				int e1, int e2, int e3, Gb_q6* old_q,
				Gb_dataMGD* d, Gb_q6* sq)
{
  double a2, r4;
  Gb_statusMGI ret = MGI_OK;
  a2 = bras->a2;
  r4 = bras->r4;
  d->d11 = e1 * sqrt(eth->vp.x * eth->vp.x + eth->vp.y * eth->vp.y);
  if (((d->d11 < 0.0) ? -d->d11 : d->d11) < bras->epsilon) {
    ret = MGI_SINGULAR;
 //   sq->q1 = old_q->q1 - bras->of1;
    sq->q1 = old_q->q1;
    d->s1 = sin(sq->q1 - bras->of1);
    d->c1 = cos(sq->q1 - bras->of1);
  } else {
    d->s1 = eth->vp.y / d->d11;
    d->c1 = eth->vp.x / d->d11;
    //    sq->q1 = Gb_atan2 (d->s1, d->c1) + bras->of1;
    sq->q1 = Gb_atan2_offset(d->s1, d->c1, bras->of1, old_q->q1);
  }
  d->d10 = d->c1 * eth->vz.x + d->s1 * eth->vz.y;
  d->d8  = d->s1 * eth->vz.x - d->c1 * eth->vz.y;
  d->d9  = d->c1 * eth->vx.x + d->s1 * eth->vx.y;
  d->d6  = d->s1 * eth->vx.x - d->c1 * eth->vx.y;
  d->d12 = d->d11*d->d11 + eth->vp.z*eth->vp.z;
  d->s3 = (d->d12 - r4*r4 - a2*a2)
    / (2.0 * a2 * r4);
  if ((d->s3 < (-1.0+bras->epsilon)) || (d->s3 > (1.0-bras->epsilon))) {
    /*    printf("Pb MGI_SINGULAR s3 1-S3 1+S3 // l=  %g  %g  %g // %G\n",
     *	   d->s3, 1-d->s3, 1+d->s3, sqrt(d->d11*d->d11 + eth->p.z*eth->p.z));
     */
    if (d->s3 < -1.-bras->epsilon) {
      ret = MGI_APPROXIMATE;
      d->s3 = -1.;
    } else if (d->s3 < -1) {
      ret = MGI_SINGULAR;
      d->s3 = -1.;
    } else if ( d->s3 < -1.+bras->epsilon) {
      ret = MGI_SINGULAR;
    } else if (d->s3 > 1.+bras->epsilon) {
      ret = MGI_APPROXIMATE;
      d->s3 = 1.;
    } else if (d->s3 > 1) {
      ret = MGI_SINGULAR;
      d->s3 = 1.;
    } else if (d->s3 > 1-bras->epsilon) {
      ret = MGI_SINGULAR;
    }
    /*    if (d->s3 >= 0) {
      d->c2 = d->d11 / (a2 + r4);
      d->s2 = eth->p.z / (a2 + r4);
    } else {
      d->c2 = d->d11 / (a2 - r4);
      d->s2 = eth->p.z / (a2 - r4);
    } */
  }
  d->c3 = e2 * sqrt(1 - d->s3 * d->s3);
  //  d->d12 = d->d11 * d->d11 + eth->vp.z * eth->vp.z;
  /* d->d12 = a2 * a2 + r4 * r4 + 2 * d->s3 * a2 * r4; */
  if (((d->d12 < 0.0) ? -d->d12 : d->d12) < bras->epsilon) {
    ret = MGI_SINGULAR; /* uniquement possible pour bras avec a2=r4 */
    sq->q2 = old_q->q2 - bras->of2;
  } else {
    d->c2 = ((d->s3 * r4 + a2) * d->d11 - d->c3 * r4 * eth->vp.z) / d->d12;
    d->s2 = (d->c3 * r4 * d->d11 + (d->s3 * r4 + a2) * eth->vp.z) / d->d12;
    sq->q2 = Gb_atan2(d->s2, d->c2);
  }
  sq->q3 = Gb_atan2(d->s3, d->c3);
  d->s23 = sin(sq->q2 + sq->q3);
  d->c23 = cos(sq->q2 + sq->q3);
  sq->q2 += bras->of2;
  sq->q3 += bras->of3;
  if ( (sq->q2 - old_q->q2) > M_PI) {
    //printf("1-- q2= %g  q2old= %g ", sq->q2, old_q->q2);
    sq->q2 -= 2*M_PI;
		//printf("q2next %g\n",sq->q2);
  }
  if ( (sq->q2 - old_q->q2) <= -M_PI) {
    //printf("2-- q2= %g  q2old= %g ", sq->q2, old_q->q2);
    sq->q2 += 2*M_PI;
		//printf("q2next %g\n",sq->q2);
  }
  if ( (sq->q3 - old_q->q3) > M_PI) sq->q3 -= 2*M_PI;
  if ( (sq->q3 - old_q->q3) <= -M_PI) sq->q3 += 2*M_PI;
  if (fabs(sq->q2) > 2*M_PI) {
   // printf("q2= %g  q2old= %g\n", sq->q2, old_q->q2);
	}
  d->d7 = d->c23 * d->d10 + d->s23 * eth->vz.z;
  d->c5 = d->s23 * d->d10 - d->c23 * eth->vz.z;
  d->d5 = d->c23 * d->d9  + d->s23 * eth->vx.z;
  d->d4 = d->s23 * d->d9  - d->c23 * eth->vx.z;
  d->s5 = e3 * sqrt(d->d7 * d->d7 + d->d8 * d->d8);
  //  sq->q5 = Gb_atan2(d->s5, d->c5) + bras->of5;
  sq->q5 = Gb_atan2_offset(d->s5, d->c5, bras->of5, old_q->q5);
  if (((d->s5 < 0.0) ? -d->s5 : d->s5) < bras->epsilon) {
    if (ret != MGI_APPROXIMATE) ret = MGI_SINGULAR;
  //  sq->q4 = old_q->q4 - bras->of4;
    sq->q4 = old_q->q4 ;
    d->c4 = cos(sq->q4 - bras->of4);
    d->s4 = sin(sq->q4 - bras->of4);
  } else {
    d->c4 = d->d7 / d->s5;
    d->s4 = d->d8 / d->s5;
  }
  //  sq->q4 = Gb_atan2(d->s4, d->c4) + bras->of4;
  sq->q4 = Gb_atan2_offset(d->s4, d->c4, bras->of4, old_q->q4);
  d->d3 = d->c4 * d->d5 + d->s4 * d->d6;
  d->s6 =-d->s4 * d->d5 + d->c4 * d->d6;
  d->c6 = d->c5 * d->d3 - d->s5 * d->d4;
  sq->q6 = Gb_atan2_offset(d->s6, d->c6, bras->of6, old_q->q6);
  return ret;
}

double Gb_q6Dist(Gb_q6* p, Gb_q6* q)
{
  double res = 0;

  res += (p->q1 - q->q1) * (p->q1 - q->q1);
  res += (p->q2 - q->q2) * (p->q2 - q->q2);
  res += (p->q3 - q->q3) * (p->q3 - q->q3);
  res += (p->q4 - q->q4) * (p->q4 - q->q4);
  res += (p->q5 - q->q5) * (p->q5 - q->q5);
  res += (p->q6 - q->q6) * (p->q6 - q->q6);
  res = sqrt(res);
  return res;
}

//  Gb_MGI6rTh_O choose automatically the solution
//  Attention : to rewrite without calling 10 times MGI !
Gb_statusMGI Gb_MGI6rTh_O(Gb_6rParameters* bras, Gb_th* eth,
			  Gb_q6* old_q,
			  Gb_dataMGD* d, Gb_q6* sq)
{
  int e1, e2, e3;
  Gb_statusMGI status;
  double errMin=  1e10;
  double erreur;
  int eee[8][3] = {
    {  1,  1,  1 },
    {  1,  1, -1 },
    {  1, -1,  1 },
    {  1, -1, -1 },
    { -1,  1,  1 },
    { -1,  1, -1 },
    { -1, -1,  1 },
    { -1, -1, -1 }
  };
  int i, iMin = -1;

  Gb_MGD6r_gete1e2e3(bras, old_q, &e1, &e2, &e3);

  status = Gb_MGI6rTh(bras, eth, e1, e2, e3, old_q, d, sq);
  if (status == MGI_OK || status == MGI_SINGULAR)
    return status;

  for (i=0; i<8; i++) {
    status =
      Gb_MGI6rTh(bras, eth,  eee[i][0],  eee[i][1], eee[i][2], old_q, d, sq);
    if (status == MGI_OK || status == MGI_SINGULAR) {
      erreur= Gb_q6Dist(sq, old_q);
      if (erreur < errMin) {
				errMin = erreur;
				iMin = i;
      }
    }
  }
  if (iMin < 0) return MGI_ERROR;
  // not optimum but more simple to garantee d is correct
  status =
    Gb_MGI6rTh(bras, eth,  eee[i][0],  eee[i][1], eee[i][2], old_q, d, sq);
  return status;
}

/*
 *  Gb_MDD6r calcule le mod�le diff�rentiel direct
 *
 *   ATTENTION  d  doit avoir �t� initialis� par le calcul du mod�le
 *     g�om�trique direct : fonction Gb_MGD6rTh
 */
void Gb_MDD6r(Gb_6rParameters* bras, Gb_dataMGD* d, Gb_th* t06,
	       Gb_jac* jac)
{
  double r4 = bras->r4;
  jac->c1.x  =-t06->vp.y;
  jac->c1.y  = t06->vp.x;
  jac->c1.z  = 0;
  jac->c1.rx = 0;
  jac->c1.ry = 0;
  jac->c1.rz = 1;
  jac->c2.x  =-d->c1 * t06->vp.z;
  jac->c2.y  =-d->s1 * t06->vp.z;
  jac->c2.z  = d->s1 * t06->vp.y + d->c1 * t06->vp.x;
  jac->c2.rx = d->s1;
  jac->c2.ry =-d->c1;
  jac->c2.rz = 0;
  jac->c3.x  = d->c1 * d->c23 * r4;
  jac->c3.y  = d->s1 * d->c23 * r4;
  jac->c3.z  = d->s23 * r4;
  jac->c3.rx = d->s1;
  jac->c3.ry =-d->c1;
  jac->c3.rz = 0;
  jac->c4.x  = 0;
  jac->c4.y  = 0;
  jac->c4.z  = 0;
  jac->c4.rx = d->c1 * d->s23;
  jac->c4.ry = d->s1 * d->s23;
  jac->c4.rz =-d->c23;
  jac->c5.x  = 0;
  jac->c5.y  = 0;
  jac->c5.z  = 0;
  jac->c5.rx = d->s6 * t06->vx.x + d->c6 * t06->vy.x;
  jac->c5.ry = d->s6 * t06->vx.y + d->c6 * t06->vy.y;
  jac->c5.rz = d->s6 * t06->vx.z + d->c6 * t06->vy.z;
  jac->c6.x  = 0;
  jac->c6.y  = 0;
  jac->c6.z  = 0;
  jac->c6.rx = t06->vz.x;
  jac->c6.ry = t06->vz.y;
  jac->c6.rz = t06->vz.z;
}

