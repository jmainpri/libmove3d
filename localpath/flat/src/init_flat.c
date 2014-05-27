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
#include "Localpath-pkg.h"
// specifique pour des sources tierces
/*#include "p3d_sys.h"
#include "struct_flat.h"
#include "init_flat_proto.h"
#include "general_flat_proto.h"
*/
/*---------------------------------------------------------------------*/
/**/
/*   forthDerivativeInt --*/
/**/
/*   Retourne : la derivee quatrieme de la fonction a integrer*/
/**/
/*--------------------------------------------------------------------*/

static double forthDerivativeInt(FLAT_STR *flatPt, double u)
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;
  double flat_beta = b/a;
  double flat_beta2 = flat_beta*flat_beta;


  double t1, t4, t5, t9, t10, t12, t17, t21, t24, t25, t27, t29, 
    t35, t44;

  t1 = cos(u);
  t4 = 1.0+flat_beta2+2.0*flat_beta*t1;
  t5 = sqrt(t4);
  t9 = pow(sin(u),2.0);
  t10 = t4*t4;
  t12 = t5/t10;
  t17 = t5/t10/t4;
  t21 = t1*t1;
  t24 = t9*t9;
  t25 = t10*t10;
  t27 = t5/t25;
  t29 = flat_beta2*flat_beta;
  t35 = flat_beta2*flat_beta2;
  t44 = t1/t5+8.0*t9*t12*flat_beta-66.0*t1*t17*flat_beta2*t9-7.0*t21*t12*flat_beta-60.0*t24*t27*t29+105.0*t1*t5/t25/t4*t35*t24+90.0*t21*t27*t29*t9+9.0*t21*t1*t17*flat_beta2;
  return t44;
}


/****************************************************************************/
/*          Fonction qu'on integre pour le calcul de L                     */
/***************************************************************************/

double functionToBeIntegrated(FLAT_STR *flatPt, double x)
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  static double s;
  s = a*cos(x)/sqrt(a*a + b*b + 2*a*b*cos(x));
  return s;
}

/*--------------------------------------------------------------------*/
/**/
/*   intSimpson -- integrale entre a et b par la formule de Simpson*/
/**/
/*------------------------------------------------------------------*/

double intSimpson(FLAT_STR *flatPt, double a, double b)
{
  double diff4, h, sum, sum1=0, x;
  int nb_step, i;

  diff4 = 2*fabs(forthDerivativeInt(flatPt, .5*(a+b)));
  if (diff4 < 1)
    diff4 = 1.0;
  /* pas de discretisation */
  h = pow(4*EPS/(FLAT_NUMBER_L-1)/diff4,.25);
  /* nombre de pas */
  nb_step = (int)ceil(fabs(b-a)/(2*h));
  /* pas d'integration */
  h = .5*(b-a)/nb_step;
  
  x = a;
  sum = functionToBeIntegrated(flatPt, x)/3.0;

  for (i=1; i<=nb_step; i++){
    x += h;
    sum += 4*functionToBeIntegrated(flatPt, x)/3.0;
    x += h;
    sum1 = functionToBeIntegrated(flatPt, x)/3.0;
    sum += 2*sum1;
  }
  sum -= sum1;
  return h*sum;
}

/***********************************************************************/
/*         Calcul des vecteurs de L                                    */
/***********************************************************************/
void calcul_array_L(FLAT_STR *flatPt)
{
  double b = flatPt->distAxleToAxis.l2;
  double phi_max = flatPt->phi_max;
  double sum;
  int i, cont, lar;
  lar = (FLAT_NUMBER_L - 1)/2;
  flatPt->array_L[0] = 0.0;
  flatPt->array_L[lar+1] = 0.0;

  for (i=1; i<=lar; i++){
    sum = b*intSimpson(flatPt, -(i-1)*phi_max/lar,-i*phi_max/lar);
    flatPt->array_L[lar+1-i]=flatPt->array_L[lar+2-i]+sum;
  }
  for (cont = 1; cont <= lar; cont++){
      
    flatPt->array_L[lar+cont+1] = 
      -flatPt->array_L[lar-cont+1];
  }
}


/*----------------------------------------------------------------------*/
/**/
/*   calcul_array_phi -- Remplissage du tableau d'echantillonnage de phi*/
/**/
/*--------------------------------------------------------------------*/

void calcul_array_phi(FLAT_STR *flatPt)
{
  int i, n;
  double dKappa, kappa_goal, kappa_cur, phi_cur, dPhi_dKappa;

  /* calcul de la courbure max */
  flatPt->maxCurvature = -kappa(flatPt, flatPt->phi_max);
  n = (FLAT_NUMBER_C-1)/2;
  flatPt->array_phi[0] = flatPt->phi_max;
  dKappa = flatPt->maxCurvature/n;
  phi_cur = flatPt->phi_max;

  for (i=1; i < n; i++){
    kappa_goal = (i-n)*dKappa;
    kappa_cur = (i-1-n)*dKappa;

    while (fabs(kappa_goal - kappa_cur) >=EPS){
      /* derivee de phi par rapport a kappa */
      dPhi_dKappa = 1/diff_kappa(flatPt, phi_cur);
      phi_cur += dPhi_dKappa*(kappa_goal-kappa_cur);
      kappa_cur = kappa(flatPt, phi_cur);
    }
    /* kappa_cur = kappa_goal */
    flatPt->array_phi[i] = phi_cur;
  }
  /* remplissage de la deuxieme moitie par symetrie */
  flatPt->array_phi[n] = 0.0;
  for (i=1; i<=n; i++){
    flatPt->array_phi[n+i] = -flatPt->array_phi[n-i];
  }
}  

/*****************************************************************************/
/* Calcul de C_MAX et de BETA.                                               */
/*****************************************************************************/
FLAT_STR *initializeFlatValue(GENPOS_TRAILER_CONNECTION *robotSizePt)
{
  double phiMax = robotSizePt->maxAngle;
  double a, b;
  FLAT_STR *flatPt = NULL;

  a = robotSizePt->l1;
  b = robotSizePt->l2;

  if ((a==0) && (b==0)){
    return NULL;
  }

  if ((flatPt = MY_ALLOC(FLAT_STR,1)) == NULL){
    return NULL;
  }

/* modif ejs */
  
  flatPt->distAxleToAxis.l1 = robotSizePt->l1;
  flatPt->distAxleToAxis.l2 = robotSizePt->l2;
  flatPt->phi_max = phiMax;
  
/*modif ejs*/
  if (flatPt->distAxleToAxis.l1 >= NEAR_ZERO ){
    /* initialiation des tableaux de discretisation */
    calcul_array_L(flatPt);
    calcul_array_phi(flatPt);
  }
  else {
    flatPt->maxCurvature = - kappa(flatPt, flatPt->phi_max);
  }
  return flatPt;
}

/*****************************************************************************/
/* Calcul de C_MAX et de BETA.                                               */
/*****************************************************************************/
void initializeFlatValueAtAddress(GENPOS_TRAILER_CONNECTION *robotSizePt, 
				  FLAT_STR* flatPt)
{
  double phiMax = robotSizePt->maxAngle;
  double a, b;

  a = robotSizePt->l1;
  b = robotSizePt->l2;

  if ((a==0) && (b==0)){
    return;
  }

  flatPt->distAxleToAxis.l1 = robotSizePt->l1;
  flatPt->distAxleToAxis.l2 = robotSizePt->l2;
  flatPt->phi_max = phiMax;
  
  if (flatPt->distAxleToAxis.l1 >= NEAR_ZERO ){
    /* initialiation des tableaux de discretisation */
    calcul_array_L(flatPt);
    calcul_array_phi(flatPt);
  }
  else {
    flatPt->maxCurvature = - kappa(flatPt, flatPt->phi_max);
  }
  return;
}

/*
 *  destroyFlatStruct --
 *
 *  destroy the structure flat_str and free arrays and pointers
 */

void destroyFlatStruct(FLAT_STR *flatPt)
{
  free(flatPt);
}
