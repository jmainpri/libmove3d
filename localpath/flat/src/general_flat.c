#include "Localpath-pkg.h"
// specifique pour des sources tierces
/*#include "p3d_sys.h"
#include "struct_flat.h"
#include "init_flat_proto.h"
#include "general_flat_proto.h"
*/


FLAT_STR flat_str;

double angleLimit(double angle)
{
  while (angle < -M_PI){
    angle += 2*M_PI;
  }
  while (angle > M_PI){
    angle -= 2*M_PI;
  }
  return angle;
}

/*****************************************************************************/

/*************************************************************************/
/*                  Interpolation pour la fonction L                     */
/*************************************************************************/

double flatEllipticIntegral(FLAT_STR *flatPt, double x)
{
  static int n = (FLAT_NUMBER_L-1)/2;
  int i;
  double x0, x1, y0, y1, y2, h, L;
  
/* modif ejs temporaire */
  if (flatPt->distAxleToAxis.l1 == 0){
    return 0.;
  }

  if (fabs(x) > flatPt->phi_max){
    L = SIGN(x)*flatPt->array_L[FLAT_NUMBER_L];
    return L;
  }
  
  i = (int)floor(n*(x/flatPt->phi_max+1)+1.5);
  
  x0 = (i-n-1)*flatPt->phi_max/n;
  x1 = .5*(x0+x);
  
  h = (x1-x0);
  
  y0 = functionToBeIntegrated(flatPt, x0);
  y1 = functionToBeIntegrated(flatPt, x1);
  y2 = functionToBeIntegrated(flatPt, x);
  
  if ((i>FLAT_NUMBER_L) || (i<0)){
    printf(" flatEllipticIntegral: i=%d, phi=%10.5f\n", i, x);
  }
  L = flatPt->array_L[i]+h*(y0/3.0+4*y1/3.0+y2/3.0)*
    flatPt->distAxleToAxis.l2;
  return L;
} 

/*----------------------------------------------------------------------
 *
 *   kappa -- relation donnant la courbure en fonction de l'angle phi
 *
 *--------------------------------------------------------------------*/

double kappa(FLAT_STR *flatPt, double phi)
{
/* this was a version only for offaxle*/
/*   double curv; */
/*   double a = flatPt->distAxleToAxis.l1; */
/*   double b = flatPt->distAxleToAxis.l2; */
/*   double beta = b/a; */
/*   double beta2 = beta*beta; */

/*   curv = -sin(phi)/ */
/*     (cos(phi)*a*sqrt(1+beta2+2*beta*cos(phi))+ */
/*      flatEllipticIntegral(phi)*sin(phi)); */
/*   return curv; */
  double curv;
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  curv = -sin(phi)/
    (cos(phi)*sqrt(a*a+b*b+2*a*b*cos(phi))+
     flatEllipticIntegral(flatPt, phi)*sin(phi));
  return curv;
}

double diff_kappa(FLAT_STR *flatPt, double phi)
{
  double t1, t2, t3, t7, t9, t10, t12, t15, t22;

  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  t1 = cos(phi);
  t2 = a*a;
  t3 = b*b;
  t7 = sqrt(t2+t3+2.0*a*b*t1);
  t9 = flatEllipticIntegral(flatPt, phi);
  t10 = sin(phi);
  t12 = t1*t7+t9*t10;
  t15 = t12*t12;
  t22 = -t1/t12+t10/t15*(-t10*t7+t9*t1);
  
  return t22;
}

double diff2_kappa(FLAT_STR *flatPt, double phi)
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double t1, t2, t3, t4, t5, t8, t9, t10, t11, t12, t15, t16, t20, t25, 
    t28, t29, t32, t37;

  t1 = sin(phi);
  t2 = cos(phi);
  t3 = a*a;
  t4 = b*b;
  t5 = a*b;
  t8 = sqrt(t3+t4+2.0*t5*t2);
  t9 = t8*t2;
  t10 = flatEllipticIntegral(flatPt, phi);
  t11 = t10*t1;
  t12 = t9+t11;
  t15 = t12*t12;
  t16 = 1/t15;
  t20 = -t1*t8+t10*t2;
  t25 = t20*t20;
  t28 = t1*t1;
  t29 = 1/t8;
  t32 = t2*t2;
  t37 = t1/t12+2.0*t2*t16*t20-2.0*t1/t15/t12*t25+t1*t16*(-t9+t28*t29*t5+t5*
t32*t29-t11);

  return t37;
}

/*----------------------------------------------------------------------
 *  
 *   curvToPhi -- calcul de phi en fonction de la courbure
 *
 *   Description : A partir du tableau d'echantillonnage de phi en fonction
 *                 de la courbure, on calcule une interpolation de degre trois
 *                 dont les derivees coincident aux bornes de l'intervalle.
 *                 L'erreur est majoree par 7h^4/8*M4, ou M4 est un majorant 
 *                 de la derivee 4e de la fonction a interpoler, et h le pas
 *                 d'echantillonnage.
 *
 *                 D'apres des calculs issus de Maple, M4 < 50
 *
 *--------------------------------------------------------------------*/

double curvToPhi(FLAT_STR *flatPt, double x)
{  
  double x0, y0, z0, x1, y1, z1;
  double h, h2, h3, dx, dx2, dx3, dy, phi;
  static int n = (FLAT_NUMBER_C-1)/2;
  int i;

  if (fabs(x)>flatPt->maxCurvature)
    {
      return(-SIGN(x)*(flatPt->phi_max+.1));
    }

  /* determination de l'intervalle d'interpolation */
  i = (int)floor(n*x/flatPt->maxCurvature+n);

  /* valeurs du tableau */
  x0 = (i-n)*flatPt->maxCurvature/n;
  x1 = (i+1-n)*flatPt->maxCurvature/n;
  y0 = flatPt->array_phi[i];
  y1 = flatPt->array_phi[i+1];

  /* derivee de phi par rapport a la courbure aux extremites */
  z0 = 1/diff_kappa(flatPt, y0);
  z1 = 1/diff_kappa(flatPt, y1);

  h = x1-x0;
  h2 = h*h;
  h3 = h2*h;
  
  dx = x-x0;
  dx2 = dx*dx;
  dx3 = dx*dx2;

  dy = y1-y0;
  
  phi = -(2*dy-h*(z0+z1))*dx3/h3 - (-3*dy*h+z1*h2+2*z0*h2)*dx2/h3
    +z0*dx+y0;

  return phi;
}


#if(0)
/******************************************************************************/ 
/* Conversion de Config en TR_FLAT_CONFIG_STR                                           */ 
/*  l'unite' de mesure de distance dans l'espace des TR_FLAT_CONFIG_STR est la longueur  */ 
/*  de l2: distance entre l'essieu de Hilare et celui de la remorque.         */ 
/******************************************************************************/ 

/*  defined in trPlannerStruct.h */
typedef struct TR_CONFIG_STR{
  double   xq;     /* abscisse du milieu de l'essieu du robot */
  double   yq;     /* ordonnee du milieu de l'essieu du robot */
  double   teta_1; /* angle absolu du robot */
  double   teta_2; /* angle absolu de la remorque */
    int aleat;
    int unused;
} TR_CONFIG_STR;
 

void conv_conf_fconf(TR_CONFIG_STR *conf, TR_FLAT_CONFIG_STR *fconf)
{
    
     double a = flatPt->distAxleToAxis.l1;
     double b = flatPt->distAxleToAxis.l2;
     double a2 = a*a;
     double b2 = b*b;
     double alpha = conf->teta_1;
     double beta = conf->teta_2;
     double phi = angleLimit(beta-alpha);


     fconf->xp = conf->xq - b*cos(beta) - flatEllipticIntegral(flatPt, phi)*
       (b*sin(beta) + a*sin(alpha))/sqrt(a2 + b2 + 2*a*b*cos(phi));

     fconf->yp = conf->yq - b*sin(beta) + flatEllipticIntegral(flatPt, phi)*
       (a*cos(alpha) + b*cos(beta))/sqrt(a2 + b2 + 2*a*b*cos(phi));

     fconf->tau = atan2((a*sin(alpha) + b*sin(beta)),
			   (a*cos(alpha) + b*cos(beta)));

     fconf->kappa = kappa(flatPt, phi);
}

#endif

double diff_phi(FLAT_STR *flatPt, double *h_deriv, int sens)
{
  double  hx2 = h_deriv[4];
  double  hy3 = h_deriv[7];
  double  hx1 = h_deriv[2];
  double  hy1 = h_deriv[3];
  double  hx3 = h_deriv[6];
  double  hy2 = h_deriv[5];
  double  t62 = hx1*hx1;
  double  t63 = hy1*hy1;
  double  t64 = t62+t63; /* hx'^2 + hy'^2 */
  double  det = hx1*hy2-hx2*hy1; /* hx'hy''-hx''hy' */
  double  t3 = pow(t64,-1.5);
  double  curv = sens*det*t3;
  double  phi = curvToPhi(flatPt, curv);

  double  curv1 = sens*(t3*((hx1*hy3-hx3*hy1)-1.5*det*2*(hx1*hx2+hy1*hy2)/t64));
  double  kappa1 = diff_kappa(flatPt, phi);
  double  phi1 = curv1/kappa1;

  return phi1;
}
/*----------------------------------------------------------------------
 *
 *   Les derivees 2e et 3e de phi
 *
 *--------------------------------------------------------------------*/

double diff2_phi(FLAT_STR *flatPt, double *h_deriv, int sens)
{
  double t1, t2, t4, t8, t10, t11, t15, t16, t17, t18, t23, t24, t27,
    t28, t33, t34, t35, t38, t40, t46, t47, t48, t49, t50, t53, t55, t56,
    t57, t58, t59, t61, t62, t63, t65, t66, t68, t69, t73, t74, t77, t81,
    t83, t84, t87, t93, t95, t105, t111, t119;
  double curv;
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;
  
  t1 = h_deriv[4];
  t2 = h_deriv[7];
  t4 = h_deriv[2];
  t8 = h_deriv[3];
  t10 = h_deriv[6];
  t11 = h_deriv[5];
  t15 = t4*t4;
  t16 = t8*t8;
  t17 = t15+t16; /* hx'^2+hy'^2 */
  t18 = pow(t17,-0.15E1);
  t33 = sens*(t4*t11-t1*t8); /* hx'hy''-hx''hy' */

  curv = t33*t18;
  
  t23 = sens*(t4*t2-t10*t8);
  t24 = pow(t17,-0.25E1);
  t27 = 2.0*t4*t1+2.0*t8*t11;
  t28 = t24*t27;
  t34 = pow(t17,-0.35E1);
  t35 = t27*t27;
  t38 = t1*t1;
  t40 = t11*t11;
  t46 = curvToPhi(flatPt, curv);
  t47 = cos(t46);
  t48 = a*a;
  t49 = b*b;
  t50 = a*b;
  t53 = sqrt(t48+t49+2.0*t50*t47);
  t55 = flatEllipticIntegral(flatPt, t46);
  t56 = sin(t46);
  t57 = t55*t56;
  t58 = t47*t53+t57;
  t59 = 1/t58;
  t61 = t58*t58;
  t62 = 1/t61;
  t63 = t56*t62;
  t65 = t55*t47;
  t66 = -t56*t53+t65;
  t68 = -t47*t59+t63*t66;
  t69 = 1/t68;
  t73 = t23*t18-0.15E1*t33*t28;
  t74 = t68*t68;
  t77 = t56*t73;
  t81 = t69*t53;
  t83 = 1/t53;
  t84 = t47*t83;
  t87 = t73*t69;
  t93 = -t77*t81-t84*a*b*t56*t87+t50*t84*t56+t65*t87;
  t95 = t47*t73;
  t105 = t56*t56;
  t111 = t47*t47;
  t119 = (sens*(t1*t2+t4*h_deriv[9]-h_deriv[8]*t8-t10*t11)*t18-0.3E1*t23*t28+0.375E1*t33*t34*t35-0.15E1*t33*t24*(2.0*t38+2.0*t4*t10+2.0*t40+2.0*t8*t2))*t69-t73/t74*(t77*t69*t59+t47*t62*t93+t95*t69*t62*t66-2.0*t56/t61/t58*t66*t93+t63*(-t95*t81+t105*t83*a*b*t73*t69+t50*t111*t83-t57*t87));

  return t119;
}

double diff3_phi(FLAT_STR *flatPt, double *h_deriv, int sens)
{
  double t1, t2, t4, t8, t10, t11, t15, t16, t17, t18, t20, t24, t27,
    t28, t31, t32, t37, t38, t39, t40, t42, t44, t46, t47, t52, t53, t68,
    t69, t70, t71, t72, t74, t75, t77, t78, t79, t80, t81, t83, t84, t85,
    t87, t88, t90, t91, t97, t98, t99, t103, t104, t105, t107, t108, t110,
    t111, t112, t113, t114, t116, t120, t122, t123, t124, t127, t128,
    t132, t134, t135, t137, t138, t139, t142, t144, t149, t151, t152,
    t153, t154, t156, t163, t165, t168, t172, t175, t177, t181, t188,
    t192, t195, t206, t208, t210, t214, t226, t259, t261, t263;
  double curv;
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;
  
  t1 = h_deriv[4];
  t2 = h_deriv[9];
  t4 = h_deriv[2];
  t8 = h_deriv[3];
  t10 = h_deriv[8];
  t11 = h_deriv[5];
  t20 = h_deriv[7];
  t24 = h_deriv[6];
  t15 = t4*t4;
  t16 = t8*t8;
  t17 = t15+t16;  /* hx'^2+hy'^2 */
  t18 = pow(t17,-0.15E1);
  t52 = sens*(t4*t11-t1*t8); /* hx'hy''-hx''hy' */

  curv = t52*t18;

  t27 = sens*(t1*t20+t4*t2-t10*t8-t24*t11);
  t28 = pow(t17,-0.25E1);
  t31 = 2.0*t4*t1+2.0*t8*t11;
  t32 = t28*t31;
  t37 = sens*(t4*t20-t24*t8);
  t38 = pow(t17,-0.35E1);
  t39 = t31*t31;
  t40 = t38*t39;
  t42 = t1*t1;
  t44 = t11*t11;
  t46 = 2.0*t42+2.0*t4*t24+2.0*t44+2.0*t8*t20;
  t47 = t28*t46;
  t53 = pow(t17,-0.45E1);
  t68 = curvToPhi(flatPt, curv);
  t69 = cos(t68);
  t70 = a*a;
  t71 = b*b;
  t72 = a*b;
  t74 = t70+t71+2.0*t72*t69;
  t75 = sqrt(t74);
  t77 = flatEllipticIntegral(flatPt, t68);
  t78 = sin(t68);
  t79 = t77*t78;
  t80 = t69*t75+t79;
  t81 = 1/t80;
  t83 = t80*t80;
  t84 = 1/t83;
  t85 = t78*t84;
  t87 = t77*t69;
  t88 = -t78*t75+t87;
  t90 = -t69*t81+t85*t88;
  t91 = 1/t90;
  t97 = t27*t18-0.3E1*t37*t32+0.375E1*t52*t40-0.15E1*t52*t47;
  t98 = t90*t90;
  t99 = 1/t98;
  t103 = t37*t18-0.15E1*t52*t32;
  t104 = t78*t103;
  t105 = t91*t81;
  t107 = t69*t84;
  t108 = t91*t75;
  t110 = 1/t75;
  t111 = t69*t110;
  t112 = t111*a;
  t113 = b*t78;
  t114 = t103*t91;
  t116 = t112*t113*t114;
  t120 = -t104*t108-t116+t72*t111*t78+t87*t114;
  t122 = t69*t103;
  t123 = t91*t84;
  t124 = t123*t88;
  t127 = 1/t83/t80;
  t128 = t78*t127;
  t132 = t78*t78;
  t134 = t132*t110*a;
  t135 = b*t103;
  t137 = t134*t135*t91;
  t138 = t69*t69;
  t139 = t138*t110;
  t142 = -t122*t108+t137+t72*t139-t79*t114;
  t144 = t104*t105+t107*t120+t122*t124-2.0*t128*t88*t120+t85*t142;
  t149 = t144*t144;
  t151 = t103*t99;
  t152 = t103*t103;
  t153 = t69*t152;
  t154 = t99*t81;
  t156 = t78*t97;
  t163 = t120*t120;
  t165 = t99*t75;
  t168 = t165*t144;
  t172 = t110*a;
  t175 = t74*t74;
  t177 = t75/t175;
  t181 = t152*t99;
  t188 = t97*t91;
  t192 = t99*t144;
  t195 = t70*t71;
  t206 = t151*t144;
  t208 = -t153*t165-t156*t108+t104*t168+2.0*t132*t152*t99*t172*b-t69*t177*t70*t71*t132*t181-t139*a*b*t152*t99-t112*t113*t188+t111*t72*t104*t192-t137+t195*t69*t177*t132*t114+2.0*t72*t138*t110*t103*t91-t79*t181+t87*t188-t87*t206;
  t210 = t78*t152;
  t214 = t69*t97;
  t226 = t83*t83;
  t259 = t210*t165-t214*t108+t122*t168+3.0*t153*t99*t172*t113+t132*t78*t177*t70*t71*t152*t99+t134*b*t97*t91-t134*t135*t192-3.0*t116+t195*t138*t177*t78*t114-t87*t181-t79*t188+t79*t206;
  t261 = t153*t154+t156*t105-t104*t154*t144-2.0*t104*t123*t120-2.0*t69*t127*t163+t107*t208-t210*t99*t84*t88+t214*t124-t122*t99*t84*t88*t144-4.0*t122*t91*t127*t88*t120+2.0*t122*t123*t142+6.0*t78/t226*t88*t163-4.0*t128*t142*t120-2.0*t128*t88*t208+t85*t259;
  t263 = (sens*(2.0*t1*t2+t4*h_deriv[11]-h_deriv[10]*t8-2.0*t10*t11)*t18-0.45E1*t27*t32+0.1125E2*t37*t40-0.45E1*t37*t47-0.13125E2*t52*t53*t39*t31+0.1125E2*t52*t38*t31*t46-0.15E1*t52*t28*(6.0*t1*t24+2.0*t4*t10+6.0*t11*t20+2.0*t8*t2))*t91-2.0*t97*t99*t144+2.0*t103/t98/t90*t149-t151*t261;

  return t263;
}


#if(0)
/*****************************************************************************/
/* Conversion de TR_FLAT_CONFIG_STR en TR_CONFIG_STR                         */
/*****************************************************************************/

void conv_fconf_conf(TR_FLAT_CONFIG_STR *fconf, TR_CONFIG_STR *conf)
{
     double beta, alpha, delta, teta;                  /* delta = beta - alpha */
     double a = flatPt->distAxleToAxis.l1;
     double b = flatPt->distAxleToAxis.l2;
    

     delta = curvToPhi(flatPt, fconf->kappa);
     teta =  fconf->tau;
    
     alpha = atan(((a + b*cos(delta))*tan(teta) - b*sin(delta))/(a + b*sin(delta)*tan(teta) + b*cos(delta)));

     if (((cos(teta)*(a*cos(alpha)+b*cos(delta+alpha))+
	   sin(teta)*(a*sin(alpha)+b*sin(alpha+delta))))>0){
       conf->teta_1 = alpha; 
     }
     else
       {
 conf->teta_1 = angleLimit(alpha + M_PI);
 alpha = conf->teta_1;
       }
     conf->teta_2 = alpha + delta;
     beta = conf->teta_2;

     conf->xq = fconf->xp + b*cos(beta) + flatEllipticIntegral(flatPt, delta)*
       (b*sin(beta) + a*sin(alpha))/sqrt(pow(a,2) + pow(b,2) + 
					 2*a*b*cos(delta)) ;
     conf->yq = fconf->yp + b*sin(beta) - flatEllipticIntegral(flatPt, delta)*
       (a*cos(alpha) + b*cos(beta))/sqrt(pow(a,2) + pow(b,2) + 
					 2*a*b*cos(delta));
}

#endif

/*****************************************************************************/
/* Deduction  de TR_FLAT_CONFIG_STR a partir de la courbe "curve(t)" a un instant donne'*/
/* "sens" indique le sens de la marche pour un accroissement du parametre:   */
/* 0 en avant 1 en arriere.                                                  */
/*****************************************************************************/

void flat_conv_curve_fconf(double *Tab_curve, 
			   TR_FLAT_CONFIG_STR *fconf, 
			   int sens)
{
  double curve_prime_x, curve_prime_y;
  
  curve_prime_x = Tab_curve[2];
  curve_prime_y = Tab_curve[3];
  
  
  fconf->xp = Tab_curve[0];
  fconf->yp = Tab_curve[1];
  if(!curve_prime_x){
    fconf->tau = M_PI/2.0;
    if(curve_prime_y < 0.0) fconf->tau *= -1.0;
  }
  else{
    fconf->tau = atan(curve_prime_y/curve_prime_x);
    if(curve_prime_x < 0.0){
      fconf->tau += M_PI;
    }
  }
  if(sens){
    fconf->tau = angleLimit(fconf->tau + M_PI);
    fconf->kappa = -(curve_prime_x*Tab_curve[5] - curve_prime_y*Tab_curve[4])/
      pow(curve_prime_x*curve_prime_x + curve_prime_y*curve_prime_y, 1.5);
  }
  else{
    fconf->tau = angleLimit(fconf->tau);
    fconf->kappa = (curve_prime_x*Tab_curve[5] - curve_prime_y*Tab_curve[4])/
      pow(curve_prime_x*curve_prime_x + curve_prime_y*curve_prime_y, 1.5);
  }
}

static void flatHilareAlpha(double u, int deriv_order, double *Tab_alpha)
{
  /* a et b sont respectivement les derivees troisiemes de alpha en 0 et 1*/ 
  double u2, u3, u4, u5;
  static double   a3 = 10;
  static double a4 = -15;
  static double a5 = 6;

  u2 = u*u;
  u3 = u*u2;
  u4 = u*u3;
  u5 = u*u4;
  
  
  /* alpha(u) */
  Tab_alpha[0] = a3*u3 + a4*u4 + a5*u5;
  /* alpha_prime(u) */
  Tab_alpha[1] = 3.0*a3*u2 + 4.0*a4*u3 + 5.0*a5*u4;
  /* alpha_sec(u) */
  Tab_alpha[2] = 6.0*a3*u + 12.0*a4*u2 + 20.0*a5*u3;

  if (deriv_order < 3)
    return; 

  /* alpha_tierce(u) */
  Tab_alpha[3] = 6.0*a3 + 24.0*a4*u + 60.0*a5*u2;
  if (deriv_order < 4)
    return; 

  /* alpha_quatre(u) */
  Tab_alpha[4] = 24.0*a4 + 120.0*a5*u;
  if (deriv_order < 5)
    return; 

  /* alpha_cinq(u) */
  Tab_alpha[5] = 120.0*a5;
}


static void flatTrailerAlpha(double u, double a, double b, int deriv_order, 
			     double *Tab_alpha)
{
  /* a et b sont respectivement les derivees troisiemes de alpha en 0 et 1*/ 
  double u2, u3, u4, u5, u6, u7;
  double a3, a4, a5, a6, a7;
  u2 = u*u;
  u3 = u*u2;
  u4 = u*u3;
  u5 = u*u4;
  u6 = u*u5;
  u7 = u*u6;
  
  a3 = a/6.0;
  a4 = -b/6.0 - 2.0*a/3.0+35.0;
  a5 = .5*b + a - 84.0;
  a6 = -.5*b - 2.0*a/3 + 70.0;
  a7 = b/6.0 + a/6.0 - 20.0;
  
  /* alpha(u) */
  Tab_alpha[0] = a3*u3 + a4*u4 + a5*u5 + a6*u6 + a7*u7;
  /* alpha_prime(u) */
  Tab_alpha[1] = 3.0*a3*u2 + 4.0*a4*u3 + 5.0*a5*u4 + 6.0*a6*u5 + 7.0*a7*u6;
  /* alpha_sec(u) */
  Tab_alpha[2] = 6.0*a3*u + 12.0*a4*u2 + 20.0*a5*u3 + 30.0*a6*u4 + 
    42.0*a7*u5;

  if (deriv_order < 3)
    return; 

  /* alpha_tierce(u) */
  Tab_alpha[3] = 6.0*a3 + 24.0*a4*u + 60.0*a5*u2 + 120.0*a6*u3 + 
    210.0*a7*u4;

  if (deriv_order < 4)
    return; 

  /* alpha_quatre(u) */
  Tab_alpha[4] = 24.0*a4 + 120.0*a5*u + 360.0*a6*u2 + 840.0*a7*u3;

  if (deriv_order < 5)
    return; 

  /* alpha_cinq(u) */
  Tab_alpha[5] = 120.0*a5 + 720.0*a6*u + 2520.0*a7*u2;
}


/*--------------------------------------------------
 * 
 *   -- gamma
 *
 *        Description : position et derivees jusqu'a l'ordre 5 d'un point 
 *                              sur un cercle (droite)
 *
 *------------------------------------------------*/

void flatGamma(TR_FLAT_CONFIG_STR *fconf, double s, double deriv_order,
	       double *Tab_gamma)
{
  double Sin, Cos, teta, kappa;
  
  teta = fconf->tau;
  kappa = fconf->kappa;
  
  
  /* le cas d'un cercle */
  if(fabs(kappa) > NEAR_ZERO){
    Sin = sin(teta + kappa*s);
    Cos = cos(teta + kappa*s);
    /* gamma_x */
    Tab_gamma[0] = fconf->xp + (Sin - sin(teta))/kappa;
    /* gamma_y */
    Tab_gamma[1] = fconf->yp - (Cos - cos(teta))/kappa;
    /* gamma_prime_x */
    Tab_gamma[2] = Cos;
    /* gamma_prime_y */
    Tab_gamma[3] = Sin;
  }
  /* approximation par une droite */
  else{
    Sin = sin(teta);
    Cos = cos(teta);
    /* gamma_x */
    Tab_gamma[0] = fconf->xp + s*Cos - .5*kappa*s*s*Sin ;
    /* gamma_y */
    Tab_gamma[1] = fconf->yp + s*Sin + .5*kappa*s*s*Cos;
    /* gamma_prime_x */
    Tab_gamma[2] = Cos - kappa*s*Sin;
    /* gamma_prime_y */
    Tab_gamma[3] = Sin + kappa*s*Cos;
  }
  
  /* gamma_sec_x */
  Tab_gamma[4] = -kappa*Sin;
  /* gamma_sec_y */
  Tab_gamma[5] = kappa*Cos;

  if (deriv_order < 3)
    return;

  /* gamma_tierce_x */
  Tab_gamma[6] = -kappa*kappa*Cos;
  /* gamma_tierce_y */
  Tab_gamma[7] = -kappa*kappa*Sin;

  if (deriv_order < 4)
    return;

  /* gamma_quatre_x */
  Tab_gamma[8] = kappa*kappa*kappa*Sin;
  /* gamma_quatre_y */
  Tab_gamma[9] = -kappa*kappa*kappa*Cos;

  if (deriv_order < 5)
    return;

  /* gamma_cinq_x */
  Tab_gamma[10] = kappa*kappa*kappa*kappa*Cos;
  /* gamma_cinq_y */
  Tab_gamma[11] = kappa*kappa*kappa*kappa*Sin;
}

/*****************************************************************************/
/* Calcul de v2 l'abscisse curviligne sur le cercle Gamma1 de l'intersection */
/* de la droite O1M2 et Gamma_1                                              */
/*****************************************************************************/

double flatV2(TR_FLAT_CONFIG_STR *fconf1, TR_FLAT_CONFIG_STR *fconf2)
{
     double O1M1_x, O1M1_y, O1M2_x, O1M2_y;

     double teta, kappa, Phi;
     double norme_O1M1, norme_O1M2, scalaire_O1M1_O1M2;

     teta = fconf1->tau;
     kappa = fconf1->kappa;

     /* La premiere conf definit un cercle */
     if(fabs(kappa) > NEAR_ZERO){
       
          O1M1_x = sin(teta)/kappa;
	  O1M1_y = -cos(teta)/kappa;

	  O1M2_x = fconf2->xp - fconf1->xp + sin(teta)/kappa;
	  O1M2_y = fconf2->yp - fconf1->yp - cos(teta)/kappa;

	  scalaire_O1M1_O1M2 = O1M1_x*O1M2_x + O1M1_y*O1M2_y;
	  norme_O1M1 = sqrt(O1M1_x*O1M1_x + O1M1_y*O1M1_y);
	  norme_O1M2 = sqrt(O1M2_x*O1M2_x + O1M2_y*O1M2_y);
	  
	  Phi = acos(scalaire_O1M1_O1M2 / (norme_O1M1*norme_O1M2));
	  
	  if((O1M2_x*cos(teta) + O1M2_y*sin(teta)) < 0.0) Phi *= -1.0;

	  return(Phi/fabs(kappa));
     }
     /* La premiere conf definie une droite */
     else{
          return((fconf2->xp - fconf1->xp)*cos(teta) + 
		 (fconf2->yp - fconf1->yp)*sin(teta));
     }
}

/*V1 compute the length between M2 and his projection on Gamma1 */
double flatV1(TR_FLAT_CONFIG_STR *fconf1, TR_FLAT_CONFIG_STR *fconf2)
{
     double O1M1_x, O1M1_y, O1M2_x, O1M2_y;

     double teta, kappa;
     double norme_O1M1, norme_O1M2, scalaire_O1M1_O1M2;

     double norme_M1_M2_carree;
     double v2;

     teta = fconf1->tau;
     kappa = fconf1->kappa;  

     /* La premiere conf definit un cercle */
     if(fabs(kappa) > NEAR_ZERO){
       
          O1M1_x = sin(teta)/kappa;
	  O1M1_y = -cos(teta)/kappa;

	  O1M2_x = fconf2->xp - fconf1->xp + sin(teta)/kappa;
	  O1M2_y = fconf2->yp - fconf1->yp - cos(teta)/kappa;

	  scalaire_O1M1_O1M2 = O1M1_x*O1M2_x + O1M1_y*O1M2_y;
	  norme_O1M1 = sqrt(O1M1_x*O1M1_x + O1M1_y*O1M1_y);
	  norme_O1M2 = sqrt(O1M2_x*O1M2_x + O1M2_y*O1M2_y);
	  

	  return(norme_O1M2-norme_O1M1);
     }
     /* La premiere conf definit une droite */
     else{
       norme_M1_M2_carree = (fconf2->xp - fconf1->xp)*(fconf2->xp - fconf1->xp)
			 +(fconf2->yp - fconf1->yp)*(fconf2->yp - fconf1->yp);

       v2 =(fconf2->xp - fconf1->xp)*cos(teta) + 
	      (fconf2->yp - fconf1->yp)*sin(teta);
       return(sqrt(norme_M1_M2_carree - v2*v2));
     }
}


/*
 *  L'expression des coordonnees et de leurs derivees de la trace du 
 *  centre de Hilare.
 *
 *  Entree: les courbe canoniques des configurations initiales et 
 *          finales, le parametre u (entre 0 et 1) la projection v2
 *          l'ordre de derivation
 *
 *  Sortie: Le tableau des sorties plates et de leurs derivees
 */

void flatHilareCombination(TR_FLAT_CONFIG_STR *fconf_initPt, 
			   TR_FLAT_CONFIG_STR *finalFlatConfPt, double u, 
			   double v2, int deriv_order, double *Tab_gamma)
{
  double tab_gamma_1[2*(MAX_DERIV_ORDER_IN_COMBINATION+1)];
  double tab_gamma_2[2*(MAX_DERIV_ORDER_IN_COMBINATION+1)];
  double tab_alpha[MAX_DERIV_ORDER_IN_COMBINATION+1];
  double gamma2_1_x, gamma2_1_y;
  double gamma2_1_x_prime, gamma2_1_y_prime;
  double gamma2_1_x_sec, gamma2_1_y_sec;
  double gamma2_1_x_tierce, gamma2_1_y_tierce;
  
  flatGamma(fconf_initPt, u*v2, deriv_order, tab_gamma_1);
  flatGamma(finalFlatConfPt, (u-1.0)*v2, deriv_order, tab_gamma_2);
  flatHilareAlpha(u, deriv_order, tab_alpha);
  
  gamma2_1_x = tab_gamma_2[0] - tab_gamma_1[0];
  gamma2_1_y = tab_gamma_2[1] - tab_gamma_1[1];
  /* x */
  Tab_gamma[0] = tab_alpha[0]*gamma2_1_x + tab_gamma_1[0];
  /* y */
  Tab_gamma[1] = tab_alpha[0]*gamma2_1_y + tab_gamma_1[1];    
  
  gamma2_1_x_prime = tab_gamma_2[2] - tab_gamma_1[2];
  gamma2_1_y_prime = tab_gamma_2[3] - tab_gamma_1[3];
  /* x_prime */
  Tab_gamma[2] = tab_alpha[1]*gamma2_1_x + 
    tab_alpha[0]*v2*gamma2_1_x_prime + v2*tab_gamma_1[2];
  /* y_prime */
  Tab_gamma[3] = tab_alpha[1]*gamma2_1_y + 
    tab_alpha[0]*v2*gamma2_1_y_prime + v2*tab_gamma_1[3];
  
  gamma2_1_x_sec = tab_gamma_2[4] - tab_gamma_1[4];
  gamma2_1_y_sec = tab_gamma_2[5] - tab_gamma_1[5];
  /* x_sec */
  Tab_gamma[4] = tab_alpha[2]*gamma2_1_x + 
    2.0*tab_alpha[1]*v2*gamma2_1_x_prime +
    tab_alpha[0]*v2*v2*gamma2_1_x_sec +
    v2*v2*tab_gamma_1[4];
  /* y_sec */
  Tab_gamma[5] = tab_alpha[2]*gamma2_1_y + 
    2.0*tab_alpha[1]*v2*gamma2_1_y_prime +
    tab_alpha[0]*v2*v2* gamma2_1_y_sec +
    v2*v2*tab_gamma_1[5];     
  
  if (deriv_order < 3)
    return;
  
  gamma2_1_x_tierce = tab_gamma_2[6] - tab_gamma_1[6];
  gamma2_1_y_tierce = tab_gamma_2[7] - tab_gamma_1[7];
  /* x_tierce */
  Tab_gamma[6] = tab_alpha[3]*gamma2_1_x + 
    3.0*tab_alpha[2]*v2*gamma2_1_x_prime +
    3.0*tab_alpha[1]*v2*v2*gamma2_1_x_sec +
    tab_alpha[0]*v2*v2*v2* gamma2_1_x_tierce +
    v2*v2*v2*tab_gamma_1[6];
  /* y_tierce */
  Tab_gamma[7] = tab_alpha[3]*gamma2_1_y + 
    3.0*tab_alpha[2]*v2*gamma2_1_y_prime +
    3.0*tab_alpha[1]*v2*v2* gamma2_1_y_sec +
    tab_alpha[0]*v2*v2*v2*gamma2_1_y_tierce +
    v2*v2*v2*tab_gamma_1[7];
  
  if (deriv_order < 4)
    return;
  
  /* x_quatre */
  Tab_gamma[8] = tab_alpha[4]*gamma2_1_x + 
    4.0*tab_alpha[3]*v2*gamma2_1_x_prime +
    6.0*tab_alpha[2]*v2*v2*gamma2_1_x_sec +
    4.0*tab_alpha[1]*v2*v2*v2*gamma2_1_x_tierce +
    tab_alpha[0]*v2*v2*v2*v2*(tab_gamma_2[8] - tab_gamma_1[8]) +
    v2*v2*v2*v2*tab_gamma_1[8];
  /* y_quatre */
  Tab_gamma[9] = tab_alpha[4]*gamma2_1_y + 
    4.0*tab_alpha[3]*v2*gamma2_1_y_prime +
    6.0*tab_alpha[2]*v2*v2* gamma2_1_y_sec +
    4.0*tab_alpha[1]*v2*v2*v2*gamma2_1_y_tierce +
    tab_alpha[0]*v2*v2*v2*v2*(tab_gamma_2[9] - tab_gamma_1[9]) +
    v2*v2*v2*v2*tab_gamma_1[9];
  
  if (deriv_order < 5)
    return;
  
  /* x_cinq */
  Tab_gamma[10] = tab_alpha[5]*gamma2_1_x + 
    5.0*tab_alpha[4]*v2*gamma2_1_x_prime +
    10.0*tab_alpha[3]*v2*v2*gamma2_1_x_sec +
    10.0*tab_alpha[2]*v2*v2*v2*gamma2_1_x_tierce +
    5.0*tab_alpha[1]*v2*v2*v2*v2*(tab_gamma_2[8] - tab_gamma_1[8]) +
    tab_alpha[0]*v2*v2*v2*v2*v2*(tab_gamma_2[10] - tab_gamma_1[10])+
    v2*v2*v2*v2*v2*tab_gamma_1[10];
  /* y_cinq */
  Tab_gamma[11] = tab_alpha[5]*gamma2_1_y + 
    5.0*tab_alpha[4]*v2*gamma2_1_y_prime +
    10.0*tab_alpha[3]*v2*v2*gamma2_1_y_sec +
    10.0*tab_alpha[2]*v2*v2*v2*gamma2_1_y_tierce +
    5.0*tab_alpha[1]*v2*v2*v2*v2*(tab_gamma_2[9] - tab_gamma_1[9]) +
    tab_alpha[0]*v2*v2*v2*v2*v2*(tab_gamma_2[11] - tab_gamma_1[11])+
    v2*v2*v2*v2*v2*tab_gamma_1[11];
}

/*
 *  L'expression des coordonnees et de leurs derivees de la trace du point   
 *  central de l'essieu arriere de la remorque.                              
 *
 *  Entree: les courbe canoniques des configurations initiales et 
 *          finales, le parametre u (entre 0 et 1) la projection v2
 *          les derivees troisieme de alpha en 0 et en 1, l'ordre de 
 *          derivation
 *
 *  Sortie: Le tableau des sorties plates et de leurs derivees
 */

void flatTrailerCombination(TR_FLAT_CONFIG_STR *fconf_initPt, 
			    TR_FLAT_CONFIG_STR *finalFlatConfPt, double u, 
			    double v2, double alpha_0, double alpha_1,
			    int deriv_order, double *Tab_gamma)
{
  double tab_gamma_1[2*(MAX_DERIV_ORDER_IN_COMBINATION+1)];
  double tab_gamma_2[2*(MAX_DERIV_ORDER_IN_COMBINATION+1)];
  double tab_alpha[MAX_DERIV_ORDER_IN_COMBINATION+1];
  double gamma2_1_x, gamma2_1_y;
  double gamma2_1_x_prime, gamma2_1_y_prime;
  double gamma2_1_x_sec, gamma2_1_y_sec;
  double gamma2_1_x_tierce, gamma2_1_y_tierce;
  
  flatGamma(fconf_initPt, u*v2, deriv_order, tab_gamma_1);
  flatGamma(finalFlatConfPt, (u-1.0)*v2, deriv_order, tab_gamma_2);
  flatTrailerAlpha(u, alpha_0, alpha_1, deriv_order, tab_alpha);
  
  
  gamma2_1_x = tab_gamma_2[0] - tab_gamma_1[0];
  gamma2_1_y = tab_gamma_2[1] - tab_gamma_1[1];
  /* x */
  Tab_gamma[0] = tab_alpha[0]*gamma2_1_x + tab_gamma_1[0];
  /* y */
  Tab_gamma[1] = tab_alpha[0]*gamma2_1_y + tab_gamma_1[1];    
  
  gamma2_1_x_prime = tab_gamma_2[2] - tab_gamma_1[2];
  gamma2_1_y_prime = tab_gamma_2[3] - tab_gamma_1[3];
  /* x_prime */
  Tab_gamma[2] = tab_alpha[1]*gamma2_1_x + 
    tab_alpha[0]*v2*gamma2_1_x_prime + v2*tab_gamma_1[2];
  /* y_prime */
  Tab_gamma[3] = tab_alpha[1]*gamma2_1_y + 
    tab_alpha[0]*v2*gamma2_1_y_prime + v2*tab_gamma_1[3];
  
  gamma2_1_x_sec = tab_gamma_2[4] - tab_gamma_1[4];
  gamma2_1_y_sec = tab_gamma_2[5] - tab_gamma_1[5];
  /* x_sec */
  Tab_gamma[4] = tab_alpha[2]*gamma2_1_x + 
    2.0*tab_alpha[1]*v2*gamma2_1_x_prime +
    tab_alpha[0]*v2*v2*gamma2_1_x_sec +
    v2*v2*tab_gamma_1[4];
  /* y_sec */
  Tab_gamma[5] = tab_alpha[2]*gamma2_1_y + 
    2.0*tab_alpha[1]*v2*gamma2_1_y_prime +
    tab_alpha[0]*v2*v2* gamma2_1_y_sec +
    v2*v2*tab_gamma_1[5];     
  
  if (deriv_order < 3)
    return;
  
  gamma2_1_x_tierce = tab_gamma_2[6] - tab_gamma_1[6];
  gamma2_1_y_tierce = tab_gamma_2[7] - tab_gamma_1[7];
  /* x_tierce */
  Tab_gamma[6] = tab_alpha[3]*gamma2_1_x + 
    3.0*tab_alpha[2]*v2*gamma2_1_x_prime +
    3.0*tab_alpha[1]*v2*v2*gamma2_1_x_sec +
    tab_alpha[0]*v2*v2*v2* gamma2_1_x_tierce +
    v2*v2*v2*tab_gamma_1[6];
  /* y_tierce */
  Tab_gamma[7] = tab_alpha[3]*gamma2_1_y + 
    3.0*tab_alpha[2]*v2*gamma2_1_y_prime +
    3.0*tab_alpha[1]*v2*v2* gamma2_1_y_sec +
    tab_alpha[0]*v2*v2*v2*gamma2_1_y_tierce +
    v2*v2*v2*tab_gamma_1[7];
  
  if (deriv_order < 4)
    return;
  
  /* x_quatre */
  Tab_gamma[8] = tab_alpha[4]*gamma2_1_x + 
    4.0*tab_alpha[3]*v2*gamma2_1_x_prime +
    6.0*tab_alpha[2]*v2*v2*gamma2_1_x_sec +
    4.0*tab_alpha[1]*v2*v2*v2*gamma2_1_x_tierce +
    tab_alpha[0]*v2*v2*v2*v2*(tab_gamma_2[8] - tab_gamma_1[8]) +
    v2*v2*v2*v2*tab_gamma_1[8];
  /* y_quatre */
  Tab_gamma[9] = tab_alpha[4]*gamma2_1_y + 
    4.0*tab_alpha[3]*v2*gamma2_1_y_prime +
    6.0*tab_alpha[2]*v2*v2* gamma2_1_y_sec +
    4.0*tab_alpha[1]*v2*v2*v2*gamma2_1_y_tierce +
    tab_alpha[0]*v2*v2*v2*v2*(tab_gamma_2[9] - tab_gamma_1[9]) +
    v2*v2*v2*v2*tab_gamma_1[9];
  
  if (deriv_order < 5)
    return;
  
  /* x_cinq */
  Tab_gamma[10] = tab_alpha[5]*gamma2_1_x + 
    5.0*tab_alpha[4]*v2*gamma2_1_x_prime +
    10.0*tab_alpha[3]*v2*v2*gamma2_1_x_sec +
    10.0*tab_alpha[2]*v2*v2*v2*gamma2_1_x_tierce +
    5.0*tab_alpha[1]*v2*v2*v2*v2*(tab_gamma_2[8] - tab_gamma_1[8]) +
    tab_alpha[0]*v2*v2*v2*v2*v2*(tab_gamma_2[10] - tab_gamma_1[10])+
    v2*v2*v2*v2*v2*tab_gamma_1[10];
  /* y_cinq */
  Tab_gamma[11] = tab_alpha[5]*gamma2_1_y + 
    5.0*tab_alpha[4]*v2*gamma2_1_y_prime +
    10.0*tab_alpha[3]*v2*v2*gamma2_1_y_sec +
    10.0*tab_alpha[2]*v2*v2*v2*gamma2_1_y_tierce +
    5.0*tab_alpha[1]*v2*v2*v2*v2*(tab_gamma_2[9] - tab_gamma_1[9]) +
    tab_alpha[0]*v2*v2*v2*v2*v2*(tab_gamma_2[11] - tab_gamma_1[11])+
    v2*v2*v2*v2*v2*tab_gamma_1[11];
}

double curvature(double *h_deriv, int sens)
{
  double curv, hx1, hy1, hx2, hy2;

  hx1 = h_deriv[2];
  hy1 = h_deriv[3];
  hx2 = h_deriv[4];
  hy2 = h_deriv[5];

  curv = sens*(hx1*hy2-hx2*hy1)/pow(hx1*hx1+hy1*hy1,1.5);

  return curv;
}

double get_max_curvature(FLAT_STR *flatPt){
  return flatPt->maxCurvature;
}

/*
 *  printTabLocalPath --
 *
 *  print an array of local paths in a file in a format that can be read
 *  by trPilo.
 *
 */

void printTabLocalPathInFile(FILE *file, FLAT_TAB_LOCAL_PATH *tabLocalPathPt)
{
  int size = tabLocalPathPt->tabSize;
  int i;

  fprintf(file, "%d\n\n", size);

  for (i=0; i<size; i++){
    fprintf(file, "%30.20f  %30.20f  %30.20f  %30.20f\n", 
	    tabLocalPathPt->tab[i].initFlatConf.xp,
	    tabLocalPathPt->tab[i].initFlatConf.yp,
	    tabLocalPathPt->tab[i].initFlatConf.tau,
	    tabLocalPathPt->tab[i].initFlatConf.kappa);

    fprintf(file, "%30.20f  %30.20f  %30.20f  %30.20f\n", 
	    tabLocalPathPt->tab[i].finalFlatConf.xp,
	    tabLocalPathPt->tab[i].finalFlatConf.yp,
	    tabLocalPathPt->tab[i].finalFlatConf.tau,
	    tabLocalPathPt->tab[i].finalFlatConf.kappa);

    fprintf(file, "%30.20f\n", tabLocalPathPt->tab[i].alpha_0);
    fprintf(file, "%30.20f\n", tabLocalPathPt->tab[i].alpha_1);
    fprintf(file, "%30.20f\n", tabLocalPathPt->tab[i].u_start);
    fprintf(file, "%30.20f\n\n", tabLocalPathPt->tab[i].u_end);
  }
}
