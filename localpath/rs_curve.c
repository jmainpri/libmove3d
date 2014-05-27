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
//#include "Util-pkg.h"
//#include "P3d-pkg.h"
#include "Localpath-pkg.h"

static double pi2 = .5*M_PI;

static double _mod2pi_a;
#define MOD2PI(r)    (((_mod2pi_a = fmod((double)(r),M_2PI)) > 0.0) ? _mod2pi_a : (_mod2pi_a+M_2PI))

static int courbe_rs(int num , Float t, Float u, Float v , Float r, pStconfig conf , pStcourbe co);
static pStcourbe fct_courbe(Float r, int ty , int se , Float val , pStconfig sit , pStcourbe cur );

static int       Curve_Type = RS_ALL;
static int       Curve_Num;
static Float     Curve_U,Curve_T,Curve_V;


int RS_select_curve(int typ)
{
    int old = Curve_Type;

    if((typ == RS_ALL)||(typ == RS_NO_CUSP)||(typ == RS_WITH_CUSP)||
       (typ == RS_DUBINS)||((typ >= 1) &&(typ <= 48)))
	Curve_Type = typ;
    else Curve_Type = RS_ALL;
    
    return(old);
}

void RS_get_curve_info(int *num,Float *t, Float *u, Float *v)
{
    *num = Curve_Num;
    *t   = Curve_T;
    *u   = Curve_U;
    *v   = Curve_V;
}
	
/********************************************************************
  calcule la courbe de R&S entre 2 config
  retourne la distance min entre les 2 config
    *arg1  : c_o  -  config init
    *arg2  : c_b  -  config fin
    *arg3  : r    -  rayon de gyration
    *arg4  : co   -  pointeur sur la structure de courbe
  *******************************************************************/

Float RS_curve(pStconfig  c_o , pStconfig c_b , Float r, pStcourbe courbe)
{
  int      numero;
  Float    longueur_rs , t , u , v;
  Stconfig c_calcul;

  c_calcul.x = c_o->x; c_calcul.y = c_o->y; c_calcul.z = c_o->z;

  /* LONGUEUR DE R&S ENTRE c_origine ET c_but */
  /*
  if(FEQ(c_o->x,c_b->x,EPS6)&& FEQ(c_o->y,c_b->y,EPS6)&& FEQ(c_o->z,c_b->z,EPS4))
    longueur_rs = 0.;
  else 
  */
  switch(Curve_Type) {
  case RS_ALL:
      longueur_rs = reed_shepp(c_o , c_b , r, &numero , &t , &u , &v);
      break;
/* Debut modif Fabien */
  case RS_DUBINS:
      longueur_rs = dubins(c_o , c_b , r, &numero , &t , &u , &v);
      break;
/* Fin modif Fabien */
  case RS_NO_CUSP:
      longueur_rs = reed_shepp_no_cusp(c_o,c_b,r,&numero,&t,&u,&v);
      break;
  case RS_WITH_CUSP:
      longueur_rs = reed_shepp_with_cusp(c_o,c_b,r,&numero,&t,&u,&v);
      break;
  default:
      numero = Curve_Type;
      longueur_rs = reed_shepp48(c_o,c_b,r,numero,&t,&u,&v);
      break;
  }

  /* NIC 22/3 integre au cas ou la fct courbe_rs retourne FALSE 
    
  if(FEQ(longueur_rs , 0.0 , EPS6)) {
    courbe->cd.x = c_o->x; courbe->cd.y = c_o->y; courbe->cd.z = c_o->z;
    courbe->cf.x = c_b->x; courbe->cf.y = c_b->y; courbe->cf.z = c_b->z;
    courbe->sens=1; courbe->val=0.; 
    courbe->centre_x = courbe->centre_y = 0.; courbe->r = r;
    courbe->suiv = NULL;
    return(0.0);
  }
  */

  if(longueur_rs == infini) return(infini);

  /* TYPE DE COURBE CCSCC|| PAR R&S */

  Curve_Num = numero; Curve_T   = t; Curve_U   = u;  Curve_V   = v;

  if(courbe_rs(numero , t , u , v , r, &c_calcul , courbe))
      return(longueur_rs);
  else {
      PrintInfo(("Warning (%f %f %f)(%f %f %f)!!\n",c_o->x,c_o->y,c_o->z,
	     c_b->x,c_b->y,c_b->z));
      courbe->cd.x = c_o->x; courbe->cd.y = c_o->y; courbe->cd.z = c_o->z;
      courbe->cf.x = c_b->x; courbe->cf.y = c_b->y; courbe->cf.z = c_b->z;
      courbe->sens=1; courbe->val=0.; 
      courbe->type = 3;
      courbe->centre_x = courbe->centre_y = 0.; courbe->r = r;
      courbe->suiv = NULL;
      courbe->valid = TRUE; /* NIC 27/3 */
      return(0.0);
  }
}

/****************************************************************************

 Procedure  : courbe -  calcule la liste de fct C S | de la courbe
 de R&S  correspondante avec trois parametres d'entree.

    *arg1  : num  -  numero de la courbe ( de 1 a 48 )
    *arg2  : t  -  valeur de la longueur du 1er arc de courbe
    *arg3  : u  -  valeur de la longueur de la 2eme portion de courbe
    *arg4  : v  -  valeur de la longueur de la ieme portion de courbe
    *arg5  : r  -  rayon de gyration
    *arg5  : conf  -  pointeur sur la config de depart
    *arg6  : co  -  pointeur sur la liste de portions de courbes
 ****************************************************************************/


static int courbe_rs(int num , Float t, Float u, Float v , Float r, pStconfig conf , pStcourbe co)
{
  int gauche , droite , ligne , av ,ar;
  
  droite = 1; gauche = 2; ligne = 3; av = 1; ar = -1;
  co->prec = NULL; /*NIC*/

  switch(num) {
    /**         C | C | C   ****/
  case 1 :
    /*PrintInfo(("Courbe de type = L+ R- L+ \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , ar , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 2 :
    /*PrintInfo(("Courbe de type = L- R+ L- \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , av , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 3 :
    /*PrintInfo(("Courbe de type = R+ L- R+ \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 4 :
    /*PrintInfo(("Courbe de type = R- L+ R- \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , av , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
    /**         C | C C      ****/
  case 5 :
    /*PrintInfo(("Courbe de type = L+ R- L- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 6 :
    /*PrintInfo(("Courbe de type = L- R+ L+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , av , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 7 :
    /*PrintInfo(("Courbe de type = r+ l- r-  \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 8 :
    /*PrintInfo(("Courbe de type = r- l+ r+ \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , av , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
    /**           C S C       *****/
  case 9 :
    /*PrintInfo(("Courbe de type = l+ s+ l+  \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,ligne ,  av , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 10 :
    /*PrintInfo(("Courbe de type = r+ s+ r+ \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,ligne ,  av , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 11 :
    /*PrintInfo(("Courbe de type = l- s- l- \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,ligne ,  ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 12 :
    /*PrintInfo(("Courbe de type = r- s- r- \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,ligne ,  ar , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 13 :
    /*PrintInfo(("Courbe de type = l+ s+ r+ \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,ligne ,  av , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 14 :
    /*PrintInfo(("Courbe de type =  r+ s+ l+ \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,ligne ,  av , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 15 :
    /*PrintInfo(("Courbe de type = l- s- r- \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,ligne ,  ar , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 16 :
    /*PrintInfo(("Courbe de type = r- s- l- \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,ligne ,  ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
             /*** C Cu | Cu C ***/
  case 17 :
    /*PrintInfo(("Courbe de type = l+ ru+ lu- r- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , av , u , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 18 :
    /*PrintInfo(("Courbe de type = r+ lu+ ru- l- \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , av , u , conf , co);
    co = fct_courbe(r,droite , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 19 :
    /*PrintInfo(("Courbe de type = l- ru- lu+ r+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , ar , u , conf , co);
    co = fct_courbe(r,gauche , av , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 20 :
    /*PrintInfo(("Courbe de type = r- lu- ru+ l+ \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , av , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
         /*** C | Cu Cu | C  ***/
  case 21 :
    /*PrintInfo(("Courbe de type = l+ ru- lu- r+ \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 22 :
    /*PrintInfo(("Courbe de type = r+ lu- ru- l+ \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , ar , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 23 :
    /*PrintInfo(("Courbe de type = l- ru+ lu+ r- \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , av , u , conf , co);
    co = fct_courbe(r,gauche , av , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 24 :
    /*PrintInfo(("Courbe de type = r- lu+ ru+ l- \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , av , u , conf , co);
    co = fct_courbe(r,droite , av , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
             /*** C | C2 S C  ***/
  case 25 :
    /*PrintInfo(("Courbe de type = l+ r2- s- l- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , ar ,pi2, conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 26 :
    /*PrintInfo(("Courbe de type = r+ l2- s- r- \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , ar ,pi2, conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 27 :
    /*PrintInfo(("Courbe de type = l- r2+ s+ l+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , av ,pi2, conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 28 :
    /*PrintInfo(("Courbe de type = r- l2+ s+ r+ \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , av ,pi2, conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 29 :
    /*PrintInfo(("Courbe de type = l+ r2- s- r- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , ar ,pi2, conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 30 :
    /*PrintInfo(("Courbe de type = r+ l2- s- l- \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , ar ,pi2, conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 31 :
    /*PrintInfo(("Courbe de type = l- r2+ s+ r+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , av ,pi2, conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 32 :
    /*PrintInfo(("Courbe de type = r- l2+ s+ l+ \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , av ,pi2, conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
             /*** C | C2 S C2 | C  ***/
  case 33 :
    /*PrintInfo(("Courbe de type = l+ r2- s- l2- r+ \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , ar ,pi2, conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar ,pi2, conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 34 :
    /*PrintInfo(("Courbe de type = r+ l2- s- r2- l+  \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , ar ,pi2, conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,droite , ar ,pi2, conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 35 :
    /*PrintInfo(("Courbe de type = l- r2+ s+ l2+ r- \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , av ,pi2, conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,gauche , av ,pi2, conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 36 :
    /*PrintInfo(("Courbe de type = r- l2+ s+ r2+ l- \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , av ,pi2, conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,droite , av ,pi2, conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
              /***  C C | C  ****/
  case 37 :
    /*PrintInfo(("Courbe de type = l+ r+ l- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,droite , av , u, conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 38 :
    /*PrintInfo(("Courbe de type = r+ l+ r- \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,gauche , av , u, conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 39 :
    /*PrintInfo(("Courbe de type = l- r- l+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,droite , ar , u, conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 40 :
    /*PrintInfo(("Courbe de type = r- l- r+\n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,gauche , ar , u , conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
              /*** C S C2 | C  ***/
  case 41 :
    /*PrintInfo(("Courbe de type = l+ s+ r2+ l- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,droite , av ,pi2, conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 42 :
    /*PrintInfo(("Courbe de type = r+ s+ l2+ r- \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,gauche , av ,pi2, conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 43 :
    /*PrintInfo(("Courbe de type = l- s- r2- l+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,droite , ar ,pi2, conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  case 44 :
    /*PrintInfo(("Courbe de type = r- s- l2- r+ \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar ,pi2, conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 45 :
    /*PrintInfo(("Courbe de type = l+ s+ l2+ r- \n"));*/
    co = fct_courbe(r,gauche , av , t , conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,gauche , av ,pi2, conf , co);
    co = fct_courbe(r,droite , ar , v , conf , co);
    break;
  case 46 :
    /*PrintInfo(("Courbe de type = r+ s+ r2+ l- \n"));*/
    co = fct_courbe(r,droite , av , t , conf , co);
    co = fct_courbe(r,ligne  , av , u , conf , co);
    co = fct_courbe(r,droite , av ,pi2, conf , co);
    co = fct_courbe(r,gauche , ar , v , conf , co);
    break;
  case 47 :
    /*PrintInfo(("Courbe de type = l- s- l2- r+ \n"));*/
    co = fct_courbe(r,gauche , ar , t , conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,gauche , ar ,pi2, conf , co);
    co = fct_courbe(r,droite , av , v , conf , co);
    break;
  case 48 :
    /*PrintInfo(("Courbe de type = r- s- r2- l+ \n"));*/
    co = fct_courbe(r,droite , ar , t , conf , co);
    co = fct_courbe(r,ligne  , ar , u , conf , co);
    co = fct_courbe(r,droite , ar ,pi2, conf , co);
    co = fct_courbe(r,gauche , av , v , conf , co);
    break;
  default:
    fprintf(stderr, "MP: courbe_rs: Erreur dans courbe : type %d inconnu\n",num);
  }

  co = co->prec; 
  if(co) {co->suiv = NULL; return(TRUE);}
  else   return(FALSE); /* toutes les portions sont trop courtes */
}


/*----------------------------------------------------------------------

 Procedure  : fct_courbe -  
    *arg1  : r   -  rayon de la courbe 
    *arg1  : ty  -  type de courbe ( 1/2/3 -> droite/gauche/segment )
    *arg2  : se  -  sens de marche ( 1/-1   -> avant/arriere )
    *arg3  : val  - longueur de la portion de courbe en question
    *arg4  : sit  - pointeur sur situation du debut de courbe
    *arg5  : cur  - pointeur sur courbe
----------------------------------------------------------------------*/
static pStcourbe fct_courbe(Float r, int ty , int se , Float val , pStconfig sit , pStcourbe cur )
{
  pStcourbe  toto;
  Float      va;

  /* CAS OU LA VALEUR = 0 */
  
  if ((ty == 3)||(EQ(r,.0))) va = val;
  else                       va = val * r;

  if(FEQ(va , 0.0 , EPS5)) return(cur);

    /** config initiale **/
  cur->cd.x = sit->x; cur->cd.y = sit->y; cur->cd.z = sit->z;
  cur->sens = se;
  cur->val  = va;
  cur->r    = r;
  cur->valid = TRUE; /* NIC 27/3 */

  switch(ty ) {
    /*******   cas d'un arc du cercle a droite  ***********/
  case 1 :
    cur->type = 1;

    /** calcul du centre du cercle **/
    cur->centre_x = sit->x + r*(Float)sin((double)sit->z);
    cur->centre_y = sit->y - r*(Float)cos((double)sit->z);

    /** calcul du point extremite pour un arc de longueur val **/
    va = atan2((cur->cd.y -cur->centre_y),(cur->cd.x -cur->centre_x));
    if( se == 1 ) va = ( va - val);
    else          va = ( va + val);
    va = MOD2PI(va);

    cur->cf.x = cur->centre_x + r*(Float)cos((double)va);
    cur->cf.y = cur->centre_y + r*(Float)sin((double)va);
    val       =  cur->cd.z - se*val;
    cur->cf.z = MOD2PI(val);
    break;
    
    /******   cas d'un arc du cercle a gauche  ***********/
  case 2 :
    cur->type = 2;
    
    /** calcul du centre du cercle **/
    cur->centre_x = sit->x - r*(Float)sin((double)sit->z);
    cur->centre_y = sit->y + r*(Float)cos((double)sit->z);
    
    /** calcul du point extremite pour un arc de longueur val **/
    va = atan2((cur->cd.y -cur->centre_y),(cur->cd.x -cur->centre_x));
    if( se == 1 )  va =( va + val);
    else           va =( va - val);
    va = MOD2PI(va);

    cur->cf.x = cur->centre_x + r*(Float)cos((double)va);
    cur->cf.y = cur->centre_y + r*(Float)sin((double)va);
    val       = cur->cd.z + se*val;
    cur->cf.z = MOD2PI(val);
    break;

    /******   cas d'un segment de droite      ***********/
  case 3 :
    cur->type = 3;
    cur->centre_x = cur->centre_y = 0.;
    cur->cf.x = cur->cd.x + se*val*(Float)cos((double)cur->cd.z);
    cur->cf.y = cur->cd.y + se*val*(Float)sin((double)cur->cd.z);
    cur->cf.z = cur->cd.z;
    break;
  }

  /** on met a jour la nouvelle config de depart 
    pour la prochaine portion de courbe + misea jour pointeur **/
  sit->x = cur->cf.x; sit->y = cur->cf.y; sit->z = cur->cf.z;

  toto = cur;  cur++;  toto->suiv = cur;  cur->prec = toto; /* nissoux-essai */ /* cur->suiv = NULL; */
  return( cur );
}

/******************************************************************************/
