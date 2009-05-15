//#include "Util-pkg.h"
//#include "P3d-pkg.h"
#include "Localpath-pkg.h"

static double c_c_c(double x , double y , double phi , double radius , double *t , double *u ,double *v);
static double c_cc(double x , double y , double phi , double radius , double *t , double *u , double *v);
static double csca(double x , double y , double phi , double radius , double *t , double *u ,double* v);
static double cscb(double x ,  double y , double phi , double radius , double *t , double *u ,double *v);
static double ccu_cuc(double x , double y , double phi , double radius , double *t , double *u ,double* v);
static double c_cucu_c(double x , double y , double phi , double radius , double *t , double *u , double *v);
static double c_c2sca(double x , double y , double phi , double radius , double *t , double *u ,double *v);
static double c_c2scb(double x , double y , double phi , double radius , double *t , double *u , double *v);
static double c_c2sc2_c(double x , double y , double phi , double radius , double *t , double *u, double *v);
static double cc_c(double x , double y  , double phi , double radius , double *t , double *u , double *v);
static double csc2_ca(double x , double y , double phi , double radius , double *t , double *u , double *v);
static double csc2_cb(double x , double y , double phi , double radius , double *t , double *u ,double *v);

static double mod2pi(double x);

/*************************************************************************/
/* calcule la longueur min entre c1 et c2 .                              */
/* Pour cela il faut passer en revue les 48 cas de R&S pour              */
/* determiner le min et la structure CCSCC|| correspondante.             */
/* Cette fonction retourne la longueur.                                  */
/*************************************************************************/

double RS_length(Stconfig *c1, Stconfig *c2, double radius)
{
  double t,u,v;
  int   n;

  return(reed_shepp(c1,c2,radius,&n,&t,&u,&v));
}


/**************************************************************************/
/* calcule la longueur min entre c1 et c2 .                               */
/* Pour cela il faut passer en revue les 48 cas de R&S pour               */
/* determiner le min et la structure CCSCC|| correspondante.              */
/* Cette fonction retourne la longueur.                                   */
/*                                                                        */
/* longueur = longueur a parcourir entre c1 et c2                         */
/* num = numero de la structure CCSCC||                                   */
/*                                                                        */
/*    *arg1  : c1  -  config. de depart                                   */
/*    *arg2  : c2  -  config. d'arrivee                                   */
/*    *arg3  : radius rayon de gyration                                   */
/**************************************************************************/

double reed_shepp(Stconfig *c1 , Stconfig *c2, double radius , int* numero , double *t_r , double *u_r , double *v_r)
{
  double x , y , phi;
  double t , u , v , t1 , u1 , v1;
  int num;
  double var , var_d, theta , alpha , dx , dy , r, longueur;
  
  /* Changement de repere,les courbes sont toujours calculees 
     de (0 0 0)--> (x , y , phi) */
  dx       = (double)(c2->x - c1->x);
  dy       = (double)(c2->y - c1->y);
  var      = (double)(c2->z -c1->z);
  r        = (double)radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (double)(c1->z);
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;


  t1 = u1 = v1 = 0.0;

  if (fabs(var) <= M_PI)    phi = var;
  else {
    if (c2->z >= c1->z) phi = var - M_2PI;
    else                 phi = mod2pi(var);}

  if(FEQ(r,0.0,EPS4)) {
    
    longueur = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
    var = t1+v1; num = 9; t = t1; u = u1; v = v1;

    csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
    if((t1+v1) < (t+v)){num = 10;t = t1; u = u1; v = v1;}
    csca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- */
    if((t1+v1) < (t+v)){num = 11;t = t1; u = u1; v = v1;}
    csca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- */
    if((t1+v1) < (t+v)){num = 12;t = t1; u = u1; v = v1;}
    cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
    if((t1+v1) < (t+v)){num = 13;t = t1; u = u1; v = v1;}
    cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
    if((t1+v1) < (t+v)){num = 14;t = t1; u = u1; v = v1;}
    cscb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- */
    if((t1+v1) < (t+v)){num = 15;t = t1; u = u1; v = v1;}
    cscb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- */
    if((t1+v1) < (t+v)){num = 16;t = t1; u = u1; v = v1;}

    *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
    *numero = num;
    if(longueur == infini) PrintInfo(("infini0\n"));
    return((double)longueur);
  }
                      /****  C | C | C ***/

  longueur = c_c_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- l+ */
  num = 1; t = t1; u = u1; v = v1;

  var = c_c_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ l- */
  if (var < longueur){longueur = var; num = 2; t = t1; u = u1; v = v1;}

  var = c_c_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- r+ */
  if (var < longueur){longueur = var; num = 3; t = t1; u = u1; v = v1;}

  var = c_c_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ r- */
  if (var < longueur){longueur = var; num = 4; t = t1; u = u1; v = v1;}

                     /****  C | C C  ***/

  var = c_cc(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- l- */
  if (var < longueur){longueur = var; num = 5; t = t1; u = u1; v = v1;}

  var = c_cc(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ l+ */
  if (var < longueur){longueur = var; num = 6; t = t1; u = u1; v = v1;}

  var = c_cc(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- r- */
  if (var < longueur){longueur = var; num = 7; t = t1; u = u1; v = v1;}

  var = c_cc(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ r+ */
  if (var < longueur){longueur = var; num = 8; t = t1; u = u1; v = v1;}

                   /****  C S C ****/

  var = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
  if (var < longueur){longueur = var; num = 9; t = t1; u = u1; v = v1;}

  var = csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
  if (var < longueur){longueur = var; num = 10;t = t1; u = u1; v = v1;}

  var = csca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- */
  if (var < longueur){longueur = var; num = 11;t = t1; u = u1; v = v1;}

  var = csca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- */
  if (var < longueur){longueur = var; num = 12;t = t1; u = u1; v = v1;}

  
  var = cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
  if (var < longueur){longueur = var; num = 13;t = t1; u = u1; v = v1;}

  var = cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
  if (var < longueur){longueur = var; num = 14;t = t1; u = u1; v = v1;}

  var = cscb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- */
  if (var < longueur){longueur = var; num = 15;t = t1; u = u1; v = v1;}

  var = cscb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- */
  if (var < longueur){longueur = var; num = 16;t = t1; u = u1; v = v1;}

                      /*** C Cu | Cu C ***/
  var = ccu_cuc(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r+ l- r- */
  if (var < longueur){longueur = var; num = 17;t = t1; u = u1; v = v1;}

  var = ccu_cuc(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l+ r- l- */
  if (var < longueur){longueur = var; num = 18;t = t1; u = u1; v = v1;}

  var = ccu_cuc(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r- l+ r+ */
  if (var < longueur){longueur = var; num = 19;t = t1; u = u1; v = v1;}

  var = ccu_cuc(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l- r+ l+ */
  if (var < longueur){longueur = var; num = 20;t = t1; u = u1; v = v1;}
  
                    /*** C | Cu Cu | C  ***/
  var = c_cucu_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- l- r+ */
  if (var < longueur){longueur = var;num = 21; t = t1; u = u1; v = v1;}

  var = c_cucu_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- r- l+ */
  if (var < longueur){longueur = var; num = 22;t = t1; u = u1; v = v1;}

  var = c_cucu_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ l+ r- */
  if (var < longueur){longueur = var;num = 23; t = t1; u = u1; v = v1;}

  var = c_cucu_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ r+ l- */
  if (var < longueur){longueur = var;num = 24; t = t1; u = u1; v = v1;}
  
                 /*** C | C2 S C  ***/
  var = c_c2sca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- s- l- */
  if (var < longueur){longueur = var;num = 25; t = t1; u = u1; v = v1;}

  var = c_c2sca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- s- r- */
  if (var < longueur){longueur = var;num = 26; t = t1; u = u1; v = v1;}

  var = c_c2sca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ s+ l+ */
  if (var < longueur){longueur = var;num = 27; t = t1; u = u1; v = v1;}

  var = c_c2sca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ s+ r+ */
  if (var < longueur){longueur = var;num = 28; t = t1; u = u1; v = v1;}

  var = c_c2scb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- s- r- */
  if (var < longueur){longueur = var; num = 29; t = t1; u = u1; v = v1;}

  var = c_c2scb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- s- l- */
  if (var < longueur){longueur = var; num = 30; t = t1; u = u1; v = v1;}

  var = c_c2scb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ s+ r+ */
  if (var < longueur){longueur = var; num = 31; t = t1; u = u1; v = v1;}

  var = c_c2scb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ s+ l+ */
  if (var < longueur){longueur = var; num = 32; t = t1; u = u1; v = v1;}

              /*** C | C2 S C2 | C  ***/

  var = c_c2sc2_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- s- l- r+ */
  if (var < longueur){longueur = var; num = 33; t = t1; u = u1; v = v1;}

  var = c_c2sc2_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- s- r- l+ */
  if (var < longueur){longueur = var; num = 34; t = t1; u = u1; v = v1;}

  var = c_c2sc2_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ s+ l+ r- */
  if (var < longueur){longueur = var; num = 35; t = t1; u = u1; v = v1;}

  var = c_c2sc2_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ s+ r+ l- */
  if (var < longueur){longueur = var; num = 36; t = t1; u = u1; v = v1;}  
  
               /***  C C | C  ****/

  var = cc_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r+ l- */
  if (var < longueur){longueur = var; num = 37; t = t1; u = u1; v = v1;}

  var = cc_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l+ r- */
  if (var < longueur){longueur = var; num = 38; t = t1; u = u1; v = v1;}

  var = cc_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r- l+ */
  if (var < longueur){longueur = var; num = 39; t = t1; u = u1; v = v1;}

  var = cc_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l- r+ */
  if (var < longueur){longueur = var; num = 40; t = t1; u = u1; v = v1;}

              /*** C S C2 | C  ***/

  var = csc2_ca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ l- */
  if (var < longueur){longueur = var; num = 41; t = t1; u = u1; v = v1;}

  var = csc2_ca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ r- */
  if (var < longueur){longueur = var; num = 42; t = t1; u = u1; v = v1;}

  var = csc2_ca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- l+ */
  if (var < longueur){longueur = var; num = 43; t = t1; u = u1; v = v1;}

  var = csc2_ca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- r+ */
  if (var < longueur){longueur = var; num = 44; t = t1; u = u1; v = v1;}

  var = csc2_cb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ r- */
  if (var < longueur){longueur = var; num = 45; t = t1; u = u1; v = v1;}

  var = csc2_cb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ l- */
  if (var < longueur){longueur = var; num = 46; t = t1; u = u1; v = v1;}

  var = csc2_cb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- r+ */
  if (var < longueur){longueur = var; num = 47; t = t1; u = u1; v = v1;}

  var = csc2_cb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- l+ */
  if (var < longueur){longueur = var; num = 48; t = t1; u = u1; v = v1;}

  *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
  *numero = num;

  return((double)longueur);
}

/****************************************************************/

double reed_shepp48(Stconfig *c1 , Stconfig *c2, double radius , int numero , double *t_r , double *u_r , double *v_r)
{
  double x , y , phi;
  double t1 , u1 , v1;
  double var , var_d, theta , alpha , dx , dy , r, longueur;
  
  longueur = 0.0;

  /* Numero compris entre 0 et 48 */

  if((numero < 1) || (numero > 48)) {
      *t_r = *u_r = *v_r = 0.0;
      return(longueur);
  }
  
  /* Changement de repere,les courbes sont toujours calculees 
     de (0 0 0)--> (x , y , phi) */
  dx       = (double)(c2->x - c1->x);
  dy       = (double)(c2->y - c1->y);
  var      = (double)(c2->z -c1->z);
  r        = (double)radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (double)(c1->z);
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;

  if (fabs(var) <= M_PI)    phi = var;
  else {
    if (c2->z >= c1->z) phi = var - M_2PI;
    else                 phi = mod2pi(var);}


  t1 = u1 = v1 = 0.0;

  switch(numero) {
                      /****  C | C | C ***/
  case 1:
      longueur = c_c_c(x,y,phi,r,&t1,&u1,&v1); /*l+ r- l+ */
      break;

  case 2:
      longueur = c_c_c(-x,y,-phi,r,&t1,&u1,&v1); /*l- r+ l- */
      break;

  case 3:
      longueur = c_c_c(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l- r+ */
      break;

  case 4:
      longueur = c_c_c(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l+ r- */
      break;

                     /****  C | C C  ***/

  case 5:
      longueur = c_cc(x,y,phi,r,&t1,&u1,&v1); /*l+ r- l- */
      break;

  case 6:
      longueur = c_cc(-x,y,-phi,r,&t1,&u1,&v1); /*l- r+ l+ */
      break;
  case 7:
      longueur = c_cc(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l- r- */
      break;
  case 8:
      longueur = c_cc(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l+ r+ */
      break;

                   /****  C S C ****/
  case 9:
      longueur = csca(x,y,phi,r,&t1,&u1,&v1); /*l+ s+ l+ */
      break;
  case 10:
      longueur = csca(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ s+ r+ */
      break;
  case 11:
      longueur = csca(-x,y,-phi,r,&t1,&u1,&v1); /*l- s- l- */
      break;
  case 12:
      longueur = csca(-x ,-y,phi,r,&t1,&u1,&v1); /*r- s- r- */
      break;
  case 13:  
      longueur = cscb(x,y,phi,r,&t1,&u1,&v1); /*l+ s+ r+ */
      break;
  case 14:
      longueur = cscb(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ s+ l+ */
      break;
  case 15:
      longueur = cscb(-x,y,-phi,r,&t1,&u1,&v1); /*l- s- r- */
      break;
  case 16:
      longueur = cscb(-x ,-y,phi,r,&t1,&u1,&v1); /*r- s- l- */
      break;

                      /*** C Cu | Cu C ***/
  case 17:
      longueur = ccu_cuc(x,y,phi,r,&t1,&u1,&v1); /*l+ r+ l- r-*/
      break;
  case 18:
      longueur = ccu_cuc(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l+ r- l-*/
      break;
  case 19:
      longueur = ccu_cuc(-x,y,-phi,r,&t1,&u1,&v1); /*l- r- l+ r+ */
      break;
  case 20:
      longueur = ccu_cuc(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l- r+ l+ */
      break;
  
                    /*** C | Cu Cu | C  ***/
  case 21:
      longueur = c_cucu_c(x,y,phi,r,&t1,&u1,&v1); /*l+ r- l- r+ */
      break;

  case 22:
      longueur = c_cucu_c(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l- r- l+ */
      break;
  case 23:
      longueur = c_cucu_c(-x,y,-phi,r,&t1,&u1,&v1); /*l- r+ l+ r- */
      break;
  case 24:
      longueur = c_cucu_c(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l+ r+ l- */
      break;
  
                 /*** C | C2 S C  ***/
  case 25:
      longueur = c_c2sca(x,y,phi,r,&t1,&u1,&v1); /*l+ r- s- l- */
      break;
  case 26:
      longueur = c_c2sca(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l- s- r- */
      break;
  case 27:
      longueur = c_c2sca(-x,y,-phi,r,&t1,&u1,&v1); /*l- r+ s+ l+ */
      break;
  case 28:
      longueur = c_c2sca(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l+ s+ r+ */
      break;
  case 29:
      longueur = c_c2scb(x,y,phi,r,&t1,&u1,&v1); /*l+ r- s- r- */
      break;

  case 30:
      longueur = c_c2scb(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l- s- l- */
      break;

  case 31:
      longueur = c_c2scb(-x,y,-phi,r,&t1,&u1,&v1); /*l- r+ s+ r+ */
      break;
  case 32:
      longueur = c_c2scb(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l+ s+ l+ */
      break;

              /*** C | C2 S C2 | C  ***/
  case 33:
      longueur = c_c2sc2_c(x,y,phi,r,&t1,&u1,&v1); /*l+ r- s- l- r+ */
      break;
  case 34:
      longueur = c_c2sc2_c(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l- s- r- l+ */
      break;
  case 35:
      longueur = c_c2sc2_c(-x,y,-phi,r,&t1,&u1,&v1); /*l- r+ s+ l+ r- */
      break;
  case 36:
      longueur = c_c2sc2_c(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l+ s+ r+ l- */
      break;
  
               /***  C C | C  ****/
  case 37:
      longueur = cc_c(x,y,phi,r,&t1,&u1,&v1); /*l+ r+ l- */
      break;
  case 38:
      longueur = cc_c(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ l+ r- */
      break;
  case 39:
      longueur = cc_c(-x,y,-phi,r,&t1,&u1,&v1); /*l- r- l+ */
      break;
  case 40:
      longueur = cc_c(-x ,-y,phi,r,&t1,&u1,&v1); /*r- l- r+ */
      break;
              /*** C S C2 | C  ***/
  case 41:
      longueur = csc2_ca(x,y,phi,r,&t1,&u1,&v1); /*l+ s+ r+ l- */
      break;
  case 42:
      longueur = csc2_ca(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ s+ l+ r- */
      break;
  case 43:
      longueur = csc2_ca(-x,y,-phi,r,&t1,&u1,&v1); /*l- s- r- l+ */
      break;
  case 44:
      longueur = csc2_ca(-x ,-y,phi,r,&t1,&u1,&v1); /*r- s- l- r+ */
      break;
  case 45:
      longueur = csc2_cb(x,y,phi,r,&t1,&u1,&v1); /*l+ s+ l+ r- */
      break;
  case 46:
      longueur = csc2_cb(x ,-y,-phi,r,&t1,&u1,&v1); /*r+ s+ r+ l- */
      break;
  case 47:
      longueur = csc2_cb(-x,y,-phi,r,&t1,&u1,&v1); /*l- s- l- r+ */
      break;
  case 48:
      longueur = csc2_cb(-x ,-y,phi,r,&t1,&u1,&v1); /*r- s- r- l+ */
      break;
  }
  *t_r = (double)t1;  *u_r = (double)u1;  *v_r = (double)v1;

  return((double)longueur);
}


/********************************************************************/

double reed_shepp_no_cusp(Stconfig *c1 , Stconfig *c2, double radius , int *numero , double *t_r , double *u_r , double *v_r)

{
  double x , y , phi;
  double t , u , v , t1 , u1 , v1;
  int num;
  double var , var_d, theta , alpha , dx , dy , r, longueur;
  
  /* Changement de repere,les courbes sont toujours calculees 
     de (0 0 0)--> (x , y , phi) */
  dx       = (double)(c2->x - c1->x);
  dy       = (double)(c2->y - c1->y);
  var      = (double)(c2->z -c1->z);
  r        = (double)radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (double)(c1->z);
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;


  if (fabs(var) <= M_PI)    phi = var;
  else {
    if (c2->z >= c1->z) phi = var - M_2PI;
    else                 phi = mod2pi(var);}


  t1 = u1 = v1 = 0.0;

  if(FEQ(r,0.0,EPS4)) {
    
    longueur = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
    var = t1+v1; num = 9; t = t1; u = u1; v = v1;

    csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
    if((t1+v1) < (t+v)){num = 10;t = t1; u = u1; v = v1;}
    csca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- */
    if((t1+v1) < (t+v)){num = 11;t = t1; u = u1; v = v1;}
    csca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- */
    if((t1+v1) < (t+v)){num = 12;t = t1; u = u1; v = v1;}
    cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
    if((t1+v1) < (t+v)){num = 13;t = t1; u = u1; v = v1;}
    cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
    if((t1+v1) < (t+v)){num = 14;t = t1; u = u1; v = v1;}
    cscb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- */
    if((t1+v1) < (t+v)){num = 15;t = t1; u = u1; v = v1;}
    cscb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- */
    if((t1+v1) < (t+v)){num = 16;t = t1; u = u1; v = v1;}

    *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
    *numero = num;
    return((double)longueur);
  }
  
  /****  C S C ****/

  longueur = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
  num = 9; t = t1; u = u1; v = v1;

  var = csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
  if (var < longueur){longueur = var; num = 10;t = t1; u = u1; v = v1;}

  var = csca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- */
  if (var < longueur){longueur = var; num = 11;t = t1; u = u1; v = v1;}

  var = csca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- */
  if (var < longueur){longueur = var; num = 12;t = t1; u = u1; v = v1;}

  
  var = cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
  if (var < longueur){longueur = var; num = 13;t = t1; u = u1; v = v1;}

  var = cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
  if (var < longueur){longueur = var; num = 14;t = t1; u = u1; v = v1;}

  var = cscb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- */
  if (var < longueur){longueur = var; num = 15;t = t1; u = u1; v = v1;}

  var = cscb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- */
  if (var < longueur){longueur = var; num = 16;t = t1; u = u1; v = v1;}

  *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
  *numero = num;

  return((double)longueur);
}

/****************************************************************/

double reed_shepp_with_cusp(Stconfig *c1 , Stconfig *c2, double radius , int *numero , double *t_r , double *u_r , double *v_r)
{
  double x , y , phi;
  double t , u , v , t1 , u1 , v1;
  int num;
  double var , var_d, theta , alpha , dx , dy , r, longueur;
  
  /* Changement de repere,les courbes sont toujours calculees 
     de (0 0 0)--> (x , y , phi) */
  dx       = (double)(c2->x - c1->x);
  dy       = (double)(c2->y - c1->y);
  var      = (double)(c2->z -c1->z);
  r        = (double)radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (double)(c1->z);
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;



  if (fabs(var) <= M_PI)    phi = var;
  else {
    if (c2->z >= c1->z) phi = var- 2*M_PI;
    else                 phi = mod2pi(var);}


  t1 = u1 = v1 = 0.0;

  if(FEQ(r,0.0,EPS4)) {
    
    longueur = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
    var = t1+v1; num = 9; t = t1; u = u1; v = v1;

    csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
    if((t1+v1) < (t+v)){num = 10;t = t1; u = u1; v = v1;}
    csca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- */
    if((t1+v1) < (t+v)){num = 11;t = t1; u = u1; v = v1;}
    csca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- */
    if((t1+v1) < (t+v)){num = 12;t = t1; u = u1; v = v1;}
    cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
    if((t1+v1) < (t+v)){num = 13;t = t1; u = u1; v = v1;}
    cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
    if((t1+v1) < (t+v)){num = 14;t = t1; u = u1; v = v1;}
    cscb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- */
    if((t1+v1) < (t+v)){num = 15;t = t1; u = u1; v = v1;}
    cscb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- */
    if((t1+v1) < (t+v)){num = 16;t = t1; u = u1; v = v1;}

    *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
    *numero = num;
    return((double)longueur);
  }

  /****  C | C | C ***/

  longueur = c_c_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- l+ */
  num = 1; t = t1; u = u1; v = v1;

  var = c_c_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ l- */
  if (var < longueur){longueur = var; num = 2; t = t1; u = u1; v = v1;}

  var = c_c_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- r+ */
  if (var < longueur){longueur = var; num = 3; t = t1; u = u1; v = v1;}

  var = c_c_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ r- */
  if (var < longueur){longueur = var; num = 4; t = t1; u = u1; v = v1;}

                     /****  C | C C  ***/

  var = c_cc(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- l- */
  if (var < longueur){longueur = var; num = 5; t = t1; u = u1; v = v1;}

  var = c_cc(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ l+ */
  if (var < longueur){longueur = var; num = 6; t = t1; u = u1; v = v1;}

  var = c_cc(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- r- */
  if (var < longueur){longueur = var; num = 7; t = t1; u = u1; v = v1;}

  var = c_cc(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ r+ */
  if (var < longueur){longueur = var; num = 8; t = t1; u = u1; v = v1;}

                      /*** C Cu | Cu C ***/
  var = ccu_cuc(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r+ l- r- */
  if (var < longueur){longueur = var; num = 17;t = t1; u = u1; v = v1;}

  var = ccu_cuc(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l+ r- l- */
  if (var < longueur){longueur = var; num = 18;t = t1; u = u1; v = v1;}

  var = ccu_cuc(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r- l+ r+ */
  if (var < longueur){longueur = var; num = 19;t = t1; u = u1; v = v1;}

  var = ccu_cuc(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l- r+ l+ */
  if (var < longueur){longueur = var; num = 20;t = t1; u = u1; v = v1;}
  
                    /*** C | Cu Cu | C  ***/
  var = c_cucu_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- l- r+ */
  if (var < longueur){longueur = var;num = 21; t = t1; u = u1; v = v1;}

  var = c_cucu_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- r- l+ */
  if (var < longueur){longueur = var; num = 22;t = t1; u = u1; v = v1;}

  var = c_cucu_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ l+ r- */
  if (var < longueur){longueur = var;num = 23; t = t1; u = u1; v = v1;}

  var = c_cucu_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ r+ l- */
  if (var < longueur){longueur = var;num = 24; t = t1; u = u1; v = v1;}
  
                 /*** C | C2 S C  ***/
  var = c_c2sca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- s- l- */
  if (var < longueur){longueur = var;num = 25; t = t1; u = u1; v = v1;}

  var = c_c2sca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- s- r- */
  if (var < longueur){longueur = var;num = 26; t = t1; u = u1; v = v1;}

  var = c_c2sca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ s+ l+ */
  if (var < longueur){longueur = var;num = 27; t = t1; u = u1; v = v1;}

  var = c_c2sca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ s+ r+ */
  if (var < longueur){longueur = var;num = 28; t = t1; u = u1; v = v1;}

  var = c_c2scb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- s- r- */
  if (var < longueur){longueur = var; num = 29; t = t1; u = u1; v = v1;}

  var = c_c2scb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- s- l- */
  if (var < longueur){longueur = var; num = 30; t = t1; u = u1; v = v1;}

  var = c_c2scb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ s+ r+ */
  if (var < longueur){longueur = var; num = 31; t = t1; u = u1; v = v1;}

  var = c_c2scb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ s+ l+ */
  if (var < longueur){longueur = var; num = 32; t = t1; u = u1; v = v1;}

              /*** C | C2 S C2 | C  ***/

  var = c_c2sc2_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r- s- l- r+ */
  if (var < longueur){longueur = var; num = 33; t = t1; u = u1; v = v1;}

  var = c_c2sc2_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l- s- r- l+ */
  if (var < longueur){longueur = var; num = 34; t = t1; u = u1; v = v1;}

  var = c_c2sc2_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r+ s+ l+ r- */
  if (var < longueur){longueur = var; num = 35; t = t1; u = u1; v = v1;}

  var = c_c2sc2_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l+ s+ r+ l- */
  if (var < longueur){longueur = var; num = 36; t = t1; u = u1; v = v1;}  
  
               /***  C C | C  ****/

  var = cc_c(x , y , phi , r , &t1 , &u1 , &v1); /* l+ r+ l- */
  if (var < longueur){longueur = var; num = 37; t = t1; u = u1; v = v1;}

  var = cc_c(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ l+ r- */
  if (var < longueur){longueur = var; num = 38; t = t1; u = u1; v = v1;}

  var = cc_c(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- r- l+ */
  if (var < longueur){longueur = var; num = 39; t = t1; u = u1; v = v1;}

  var = cc_c(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- l- r+ */
  if (var < longueur){longueur = var; num = 40; t = t1; u = u1; v = v1;}

              /*** C S C2 | C  ***/

  var = csc2_ca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ l- */
  if (var < longueur){longueur = var; num = 41; t = t1; u = u1; v = v1;}

  var = csc2_ca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ r- */
  if (var < longueur){longueur = var; num = 42; t = t1; u = u1; v = v1;}

  var = csc2_ca(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- r- l+ */
  if (var < longueur){longueur = var; num = 43; t = t1; u = u1; v = v1;}

  var = csc2_ca(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- l- r+ */
  if (var < longueur){longueur = var; num = 44; t = t1; u = u1; v = v1;}

  var = csc2_cb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ r- */
  if (var < longueur){longueur = var; num = 45; t = t1; u = u1; v = v1;}

  var = csc2_cb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ l- */
  if (var < longueur){longueur = var; num = 46; t = t1; u = u1; v = v1;}

  var = csc2_cb(-x , y , -phi , r , &t1 , &u1 , &v1); /* l- s- l- r+ */
  if (var < longueur){longueur = var; num = 47; t = t1; u = u1; v = v1;}

  var = csc2_cb(-x ,-y , phi , r , &t1 , &u1 , &v1); /* r- s- r- l+ */
  if (var < longueur){longueur = var; num = 48; t = t1; u = u1; v = v1;}

  *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
  *numero = num;

  return((double)longueur);
}

/* Debut modif Fabien */

/**************************************************************************/
/* calcule la longueur min entre c1 et c2 .                               */
/* Pour cela il faut passer en revue les 4 cas de Dubins pour             */
/* determiner le min et la structure CSC|| correspondante.                */
/* Cette fonction retourne la longueur.                                   */
/*                                                                        */
/* longueur = longueur a parcourir entre c1 et c2                         */
/* num = numero de la structure CCSCC|| (seules 9 10 13 14)               */
/*                                                                        */
/*    *arg1  : c1  -  config. de depart                                   */
/*    *arg2  : c2  -  config. d'arrivee                                   */
/*    *arg3  : radius rayon de gyration                                   */
/**************************************************************************/

double dubins(Stconfig *c1 , Stconfig *c2, double radius , int* numero , double *t_r , double *u_r , double *v_r)
{
  double x , y , phi;
  double t , u , v , t1 , u1 , v1;
  int num;
  double var , var_d, theta , alpha , dx , dy , r, longueur;
  
  /* Changement de repere,les courbes sont toujours calculees 
     de (0 0 0)--> (x , y , phi) */
  dx       = (double)(c2->x - c1->x);
  dy       = (double)(c2->y - c1->y);
  var      = (double)(c2->z -c1->z);
  r        = (double)radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (double)(c1->z);
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;


  t1 = u1 = v1 = 0.0;

  if (fabs(var) <= M_PI)    phi = var;
  else {
    if (c2->z >= c1->z) phi = var - M_2PI;
    else                 phi = mod2pi(var);}

  if(FEQ(r,0.0,EPS4)) {
    
    longueur = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
    var = t1+v1; num = 9; t = t1; u = u1; v = v1;

    csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
    if((t1+v1) < (t+v)){num = 10;t = t1; u = u1; v = v1;}
    cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
    if((t1+v1) < (t+v)){num = 13;t = t1; u = u1; v = v1;}
    cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
    if((t1+v1) < (t+v)){num = 14;t = t1; u = u1; v = v1;}

    *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
    *numero = num;
    if(longueur == infini) PrintInfo(("infini0\n"));
    return((double)longueur);
  }
 
                   /****  C S C ****/

  longueur = csca(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ l+ */
  num = 9; t = t1; u = u1; v = v1;

  var = csca(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ r+ */
  if (var < longueur){longueur = var; num = 10;t = t1; u = u1; v = v1;}
  
  var = cscb(x , y , phi , r , &t1 , &u1 , &v1); /* l+ s+ r+ */
  if (var < longueur){longueur = var; num = 13;t = t1; u = u1; v = v1;}

  var = cscb(x ,-y , -phi , r , &t1 , &u1 , &v1); /* r+ s+ l+ */
  if (var < longueur){longueur = var; num = 14;t = t1; u = u1; v = v1;}


  *t_r = (double)t;  *u_r = (double)u;  *v_r = (double)v;
  *numero = num;

  return((double)longueur);
}

/* Fin modif Fabien */

/*********************************************************************************/
/*********************************************************************************/


        /****  C | C | C ***/
static double c_c_c(double x , double y , double phi , double radius , double *t , double *u ,double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  
  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4)) return(infini);
  u1 = sqrt(a*a + b*b);
  if (u1 > (4.0 * radius)) return(infini);
  else{
    theta = atan2(b , a);
    alpha = acos(u1/(4.0 * radius));
    va = M_PI_2 + alpha;
    *t = mod2pi(va + theta);
    *u = mod2pi(2.0 * (M_PI - va));
    *v = mod2pi(phi - (*t) - (*u));
    long_rs = radius*(*t + *u + *v);
    return(long_rs);
  }
}

      /****  C | C C  *****/
static double c_cc(double x , double y , double phi , double radius , double *t , double *u , double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  
  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4)) return(infini);
  u1 = sqrt(a*a + b*b);
  if (u1 > 4*radius) return(infini);
  else{
    theta = atan2(b , a);
    alpha = acos(u1/(4.0 * radius));
    va = M_PI_2 + alpha;
    *t = mod2pi(va + theta);
    *u = mod2pi(2.0 * (M_PI - va));
    *v = mod2pi(*t + *u - phi);
    long_rs = radius*(*t + *u + *v);
    return(long_rs);
  }
}

     /****  C S C  ****/
static double csca(double x , double y , double phi , double radius , double *t , double *u ,double* v)
{
  double a , b , long_rs;
  
  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  *u = sqrt(a*a + b*b);
  *t = mod2pi(atan2(b , a));
  *v = mod2pi(phi - *t);
  long_rs = radius*(*t + *v) + *u;
  return(long_rs);
}

static double cscb(double x ,  double y , double phi , double radius , double *t , double *u ,double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  
  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va) return(infini);
  else{
    theta = atan2(b , a);
    *u = sqrt(u1*u1 - va*va);
    alpha = atan2(va , *u);
    *t = mod2pi(theta + alpha);
    *v = mod2pi(*t - phi);
    long_rs = radius*(*t + *v) + *u;
    return(long_rs);
  }
}

           /*** C Cu | Cu C  ***/
static double ccu_cuc(double x , double y , double phi , double radius , double *t , double *u ,double* v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  
  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4)) return(infini);
  u1 = sqrt(a*a + b*b);
  va = 4.0 * radius;
  if (u1 > va) return(infini);
  else{
    theta = atan2(b , a);
    if (u1 > (2.0 *radius)) {
      alpha = acos(u1/(4.0 * radius) - 0.5);
      *t = mod2pi(M_PI_2 + theta - alpha);
      *u = mod2pi(M_PI - alpha);
      *v = mod2pi(phi - *t + 2.0 * (*u));
    }
    else{
      alpha = acos(0.5 + u1/(4.0 * radius)); 
      *t = mod2pi(M_PI_2 + theta + alpha);
      *u = mod2pi(alpha);
      *v = mod2pi(phi - *t + 2.0 * (*u));
    }
    long_rs = radius * (2.0 *(*u) + *t + *v);
    return(long_rs);
  }
}

/****************  C | Cu Cu | C  **************/
static double c_cucu_c(double x , double y , double phi , double radius , double *t , double *u , double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  double toto;

  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4)) return(infini);
  u1 = sqrt(a*a + b*b);
  if (u1 > 6.0 * radius)  return(infini);
  else{
    theta = atan2(b , a);
    va = 1.25 -  (u1*u1) / (16.0 * radius * radius);
    if ((va<0.)||(va>1.)) return(infini);
    else{
      *u = acos(va);
      toto = sin(*u);
      if (FEQ(toto ,0.0 ,EPS3)) toto = 0.;
      alpha = asin(radius * toto * 2.0/u1);
      *t = mod2pi(M_PI_2 + theta + alpha);
      *v = mod2pi(*t - phi);
      long_rs = radius*(2.0 *(*u) + *t + *v);
      return(long_rs);
    }
  }
}

/****************  C | C2 S C ******************/
static double c_c2sca(double x , double y , double phi , double radius , double *t , double *u ,double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va) return(infini);
  else{
    theta = atan2(b , a);
    *u = sqrt(u1*u1 - va*va) - va;
    if (*u < 0.0) return(infini);
    else{
      alpha = atan2(va , (*u + va));
      *t = mod2pi(M_PI_2 + theta + alpha);
      *v = mod2pi(*t +M_PI_2 - phi);
      long_rs = radius*(*t + M_PI_2 + *v) + *u;
      return(long_rs);
    }
  }
}


static double c_c2scb(double x , double y , double phi , double radius , double *t , double *u , double *v)
{
  double a , b , u1 , theta , long_rs , va;
  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) +1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va) return(infini);
  else{
    theta = atan2(b , a);
    *t = mod2pi(M_PI_2 + theta);
    *u = u1 - va;
    *v = mod2pi(phi - *t - M_PI_2);
    long_rs = radius*(*t + M_PI_2 + *v) + *u;
    return(long_rs);
  }
}

/****************  C | C2 S C2 | C  ***********/
static double c_c2sc2_c(double x , double y , double phi , double radius , double *t , double *u, double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  
  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  u1 = sqrt(a*a + b*b);
  va = 4.0 * radius;
  if (u1 < va) return(infini);
  else{
    theta = atan2(b , a);
    *u = sqrt(u1*u1 - va*radius) - va;
    if (*u < 0.0) return(infini);
    else{
      alpha = atan2(2*radius , (*u + va));
      *t = mod2pi(M_PI_2 + alpha + theta);
      *v = mod2pi(*t - phi);
      long_rs = radius*(*t + M_PI + *v) + *u;
      return(long_rs);
    }
  }
}

/****************  C C | C  ****************/
static double cc_c(double x , double y , double phi , double radius , double *t , double *u , double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  static double toto;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) -1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4)) return(infini);
  u1 = sqrt(a*a + b*b);
  va = 4.0 * radius;
  if (u1 > va) return(infini);
  else{
    theta = atan2(b , a);
    *u = acos(1.0 -  (2.0 * u1 * u1)/(va*va));
    toto = sin(*u);
    if (FEQ(toto ,0.0 ,EPS3)) toto = 0.0;
    /*BMIC*/
    if (FEQ(toto ,0.0 ,EPS3) &&FEQ(u1 ,0.0 ,EPS3)) return(infini);
    alpha = asin((2.0 * radius *toto)/(u1));
    /*EMIC*/
    *t = mod2pi(M_PI_2 + theta - alpha);
    *v = mod2pi(*t - *u - phi);
    long_rs = radius*(*t + *u + *v);
    return(long_rs);
  }
}

/****************  C S C2 | C  ************/
static double csc2_ca(double x , double y , double phi , double radius , double *t , double *u , double *v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  
  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) -1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va) return(infini);
  else{
    theta = atan2(b , a);
    *u = sqrt(u1*u1 - va*va) - va;
    if (*u < 0.0) return(infini);
    else{
      alpha = atan2((*u + va), va);
      *t = mod2pi(M_PI_2 + theta - alpha);
      *v = mod2pi(*t - M_PI_2 - phi);
      long_rs = radius * (*t + M_PI_2 + *v) + *u;
      return(long_rs);
    }
  }
}

static double csc2_cb(double x , double y , double phi , double radius , double *t , double *u ,double *v)
{
  double a , b , u1 , theta , long_rs , va;
  
  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va) return(infini);
  else{
    theta = atan2(b , a);
    *u = u1 - va;
    *t = mod2pi(theta);
    *v = mod2pi(phi - M_PI_2 - *t);
    long_rs = radius*(*t + M_PI_2 + *v) + *u;
    return(long_rs);
  }
}

/*******************************************************************************/

static double mod2pi(double a)
{
  return((double)((a=fmod(a, M_2PI))>0.0)?a:(FEQ(a,0.0,EPSILON)?0.0:(a + M_2PI)));
}

/*******************************************************************************/

