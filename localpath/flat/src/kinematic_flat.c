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
/* file kinematic_flat.c\src\flat_offaxle */
#include "Localpath-pkg.h"



/* 
 *  -- flatPhiAndLDeriv
 *
 *  Compute the values of phi, L (the elliptic integral and their derivatives
 *  up to order 3
 */

static int flatPhiAndLDeriv(FLAT_STR *flatPt, double kappa, 
			     double* tabPhi, double *tabL)
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;
  int result = TRUE;
  double phi = curvToPhi(flatPt, kappa);
  
  double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t13, t17, t18, t20, t21, t22, t24, t28, t29, t31, t32, t34, t35, t36, t38, t39, t40, t41, t42, t43, t44, t45, t46, t47, t48, t50, t51, t52, t53, t55, t56, t57, t60, t62, t63, t66, t67, t68, t70, t71, t73, t76, t77, t78, t81, t83, t84, t85, t88, t90, t91, t93, t94, t95, t96, t97, t99, t101, t102, t103, t105, t106, t107, t108, t110, t113, t115, t119, t120, t122, t124, t125, t127, t128, t131, t132, t133, t134, t135, t136, t138, t140, t141, t144, t145, t148, t150, t153, t154, t156, t158, t160, t163, t167, t168, t175, t177, t178, t179, t187, t189, t191, t201, t203, t213, t214, t217, t219, t220, t224, t225, t228, t230, t231, t234, t235, t242, t243, t245, t246, t248, t249, t252, t254, t260, t265, t267, t269, t276, t278, t282, t285, t289, t292, t293, t296, t299, t304, t307, t316, t323, t328, t331, t335, t337, t338, t343, t356, t379, t397, t407, t415, t422, t438, t459;

  if (fabs(phi) > flatPt->phi_max){
    result = FALSE;
  }
      t1 = cos(phi);
      t2 = a*a;
      t3 = b*b;
      t4 = a*b;
      t6 = t2+t3+2.0*t4*t1;
      t7 = sqrt(t6);
      t8 = 1/t7;
      t9 = t1*t8;
      t10 = sin(phi);
      t13 = 1/t7/t6;
      t18 = t10*t10;
      t21 = t6*t6;
      t28 = t1*t1;
      tabL[0] = flatEllipticIntegral(flatPt, phi);
      tabL[1] = t4*t9;
      tabL[2] = t4*(-t10*t8+t1*t13*t4*t10);
      tabL[3] = t4*(-t9-2.0*t18*t13*t4+3.0*t1/t7/t21*t2*t3*t18+t28*t13*t4);

      t1 = a*a;
      t2 = b*b;
      t3 = a*b;
      t5 = cos(phi);
      t6 = t3*t5;
      t7 = t1+t2+2.0*t6;
      t8 = sqrt(t7);
      t9 = sin(phi);
      t10 = t9*t9;
      t11 = t8*t10;
      t17 = t8*a;
      t18 = b*t5;
      t20 = tabL[0];
      t21 = t20*t20;
      t22 = t8*t21;
      t24 = t5*t20;
      t29 = t20*t9;
      t31 = t10*t9;
      t32 = t20*t31;
      t34 = -t11*t1-t11*t2-2.0*t11*t6+t8*t1+t8*t2+2.0*t17*t18+t22*t10+2.0*t24*
t9*t1+2.0*t24*t9*t2+4.0*t29*t3-4.0*t32*t3;
      t35 = tabL[1];
      t36 = t10*t35;
      t38 = t10*t5;
      t39 = t38*t3;
      t40 = -t36*t8+t1+t2+t39+2.0*t6;
      t41 = 1/t40;
      t42 = t34*t41;
      t43 = 1/t8;
      t44 = t43*t31;
      t45 = t1*a;
      t46 = t44*t45;
      t47 = b*t34;
      t48 = t47*t41;
      t50 = t8*t9;
      t51 = t50*t1;
      t52 = t5*t34;
      t53 = t52*t41;
      t55 = t2*b;
      t56 = t44*t55;
      t57 = a*t34;
      t60 = t50*t2;
      t62 = t44*t5;
      t63 = t1*t2;
      t66 = t5*t5;
      t67 = t50*t66;
      t68 = t3*t42;
      t70 = t8*t31;
      t71 = t70*t34;
      t73 = t41*a*b;
      t76 = t43*t45*b;
      t77 = t9*t34;
      t78 = t77*t41;
      t81 = t43*t55*a;
      t83 = t43*t1;
      t84 = t83*t2;
      t85 = t5*t9;
      t88 = t17*b;
      t90 = t43*t21;
      t91 = t90*t31;
      t93 = -t46*t48+2.0*t51*t53-t56*t57*t41+2.0*t60*t53-2.0*t62*t63*t42+4.0*
t67*t68-2.0*t71*t73+t76*t78+t81*t78+2.0*t84*t85*t42+2.0*t88*t78+t91*t68;
      t94 = t8*t20;
      t95 = t94*t10;
      t96 = t35*t34;
      t97 = t96*t41;
      t99 = t22*t9;
      t101 = t10*t34;
      t102 = t41*t20;
      t103 = t102*t1;
      t105 = t5*t35;
      t106 = t105*t34;
      t107 = t41*t9;
      t108 = t107*t1;
      t110 = t66*t20;
      t113 = t102*t2;
      t115 = t107*t2;
      t119 = t9*a;
      t120 = t119*b;
      t122 = t24*t34;
      t124 = t31*a;
      t125 = t124*b;
      t127 = t20*t10;
      t128 = t127*a;
      t131 = -2.0*t95*t97-2.0*t99*t53+2.0*t101*t103-2.0*t106*t108-2.0*t110*t42*
t1+2.0*t101*t113-2.0*t106*t115-2.0*t110*t42*t2-4.0*t97*t120-4.0*t122*t73+4.0*
t97*t125+12.0*t128*t18*t42;
      t132 = t93+t131;
      t133 = t132*t41;
      t134 = t40*t40;
      t135 = 1/t134;
      t136 = t34*t135;
      t138 = t9*t35*t8;
      t140 = tabL[2];
      t141 = t10*t140;
      t144 = t31*t35;
      t145 = t144*t43;
      t148 = t9*t66*a;
      t150 = t31*t34;
      t153 = 2.0*t138*t53+t141*t42*t8-t145*t68-2.0*t148*t48+t150*t73+2.0*t3*t78
;
      t154 = t136*t153;
      t156 = t3*t133;
      t158 = t34*t34;
      t160 = t5*t158*t135;
      t163 = t140*t158*t135;
      t167 = t135*t153;
      t168 = t77*t167;
      t175 = t9*t132*t41;
      t177 = t135*a;
      t178 = b*t153;
      t179 = t177*t178;
      t187 = 4.0*t67*t156-t76*t160+4.0*t163*t120-2.0*t83*t2*t5*t168-t81*t168+
2.0*t84*t85*t133-t76*t168+t81*t175+2.0*t71*t179+t76*t175+2.0*t44*t5*t1*t2*t34*
t167;
      t189 = t5*t132*t41;
      t191 = t52*t167;
      t201 = b*t132*t41;
      t203 = t47*t167;
      t213 = t9*t158*t135;
      t214 = t20*t1;
      t217 = 2.0*t60*t189-2.0*t60*t191-2.0*t51*t191-t56*a*t132*t41+t56*t57*t167
+2.0*t51*t189-t46*t201+t46*t203-2.0*t62*t63*t133-4.0*t50*t66*a*t203-2.0*t70*
t132*t73-8.0*t213*t214*t5;
      t219 = t10*t158;
      t220 = t135*t35;
      t224 = t5*t140*t158;
      t225 = t135*t9;
      t228 = t35*t35;
      t230 = t158*t135;
      t231 = t230*t10;
      t234 = t35*t158;
      t235 = t135*t5;
      t242 = b*t158;
      t243 = t242*t235;
      t245 = t66*t158;
      t246 = t245*t135;
      t248 = t66*t35;
      t249 = t230*t1;
      t252 = t234*t135;
      t254 = -4.0*t219*t220*t1+2.0*t224*t225*t1+2.0*t8*t228*t231+8.0*t94*t9*
t234*t235+2.0*t95*t163-2.0*t22*t231-5.0*t90*t10*a*t243+2.0*t22*t246+4.0*t248*
t249-4.0*t163*t125-24.0*t252*t39;
      t260 = t242*t135;
      t265 = 1/t8/t7;
      t267 = t10*t10;
      t269 = t63*t230;
      t276 = t3*t230;
      t278 = t1*t1;
      t282 = t66*t5;
      t285 = t177*b;
      t289 = t135*t1;
      t292 = t8*t66;
      t293 = t230*t2;
      t296 = -24.0*t29*a*b*t66*t230+12.0*t32*a*t260-2.0*t84*t246-2.0*t88*t160-
t265*t21*t267*t269-4.0*t43*t20*t124*t242*t220+14.0*t11*t5*t276-t265*t278*t2*
t231-4.0*t8*t282*t158*t285-4.0*t43*t267*t158*t289*t2-2.0*t292*t293+2.0*t11*t293
;
      t299 = t265*t267;
      t304 = t2*t2;
      t307 = t43*t10;
      t316 = t96*t135;
      t323 = t35*t132*t41;
      t328 = 2.0*t299*t5*t45*t55*t230+t299*t304*t249+5.0*t307*t45*t243+12.0*
t128*t18*t133-12.0*t127*t3*t191+t299*t278*t293+4.0*t316*t119*t178-4.0*t24*t132*
t73+4.0*t122*t179+4.0*t323*t125+2.0*t106*t225*t2*t153;
      t331 = t110*t34;
      t335 = t10*t132;
      t337 = t101*t135;
      t338 = t20*t2;
      t343 = t105*t132;
      t356 = -2.0*t110*t133*t2+2.0*t331*t135*t2*t153+2.0*t335*t113-2.0*t337*
t338*t153+2.0*t331*t289*t153-2.0*t343*t115-4.0*t323*t120-4.0*t316*t124*t178-2.0
*t292*t249+2.0*t11*t249+5.0*t307*t55*a*t158*t235+10.0*t307*t66*t269;
      t379 = -t265*t304*t1*t231-t81*t160-2.0*t265*t45*t55*t38*t230+4.0*t84*t231
+2.0*t335*t103-2.0*t337*t214*t153-2.0*t343*t108+2.0*t106*t225*t1*t153-2.0*t110*
t133*t1-2.0*t95*t323+2.0*t95*t96*t167;
      t397 = -2.0*t99*t189-t90*t124*t203+2.0*t99*t191+2.0*t88*t175-2.0*t88*t168
+t91*t156+8.0*t252*t6-4.0*t29*t158*t285-8.0*t213*t338*t5-4.0*t219*t220*t2+2.0*
t224*t225*t2+4.0*t248*t293;
      t407 = t153*t153;
      t415 = t135*t8;
      t422 = t8*t158*t135;
      t438 = 2.0*t138*t189-2.0*t138*t191-2.0*t245*t220*t8-4.0*t9*t140*t158*t415
*t5+5.0*t36*t43*t5*t276+2.0*t36*t422+t141*t133*t8-t141*t34*t415*t153-t10*tabL[3]*t422+2.0*t31*t140*t158*t135*t43*t3-t145*t156;
      t459 = t144*t43*a*t203+t267*t35*t265*t269-2.0*t148*t201+2.0*t148*t203+2.0
*t282*t158*t285-7.0*t38*a*t260+t31*t132*t73-t150*t135*t3*t153+2.0*t3*t175-2.0*
t120*t154-2.0*t3*t160;
      tabPhi[0] = phi;
      tabPhi[1] = -t42;
      tabPhi[2] = -t133+t154;
      tabPhi[3] = -(t187+t217+t254+t296+t328+t356+t379+t397)*t41+2.0*t132*t135*
t153-2.0*t34/t134/t40*t407+t136*(t438+t459);

      return result;
}



/*
 *  coef_dv --
 *
 *  Compute the ratio between the linear velocity of the robot and the 
 *  pseudo-velocity udot for the off axle trailer connection.
 */

static double coef_dv(FLAT_STR *flatPt,
		      double *h_deriv, int sens, 
		      double *tabPhi, /* phi and its derivatives */
		      double *tabL)   /* elliptic integ and its derivatives */
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double dv;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t17, t18, t20, t21, t22, t23, t25, t26, t27, t28, t29, t31, t32, t33, t35, t38, t39, t47, t55, t56, t57, t67, t68, t70, t71, t73, t74, t75, t77, t78, t79, t80, t82, t83, t85, t86, t90, t91, t92, t95, t97, t98, t100, t101, t102, t104, t106, t116, t124, t128, t131, t132, t144;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];

      t1 = hx1;
      t2 = atan2(hy1,hx1);
      t3 = hy2;
      t5 = hx2;
      t6 = hy1;
      t8 = t1*t3-t5*t6;
      t9 = sens*t8;
      t10 = t1*t1;
      t11 = t6*t6;
      t12 = t10+t11;
      t13 = sqrt(t12);
      t15 = 1/t13/t12;
      t16 = t9*t15;
      t17 = tabPhi[0];
      t18 = t2+t17;
      t20 = a*sin(t18);
      t21 = sin(t2);
      t22 = b*t21;
      t23 = t20+t22;
      t25 = a*cos(t18);
      t26 = cos(t2);
      t27 = b*t26;
      t28 = t25+t27;
      t29 = atan2(t23,t28);
      t31 = b*sin(t29);
      t32 = t28*t28;
      t33 = t23*t23;
      t35 = 1/(t32+t33);
      t38 = t8/t12;
      t39 = tabPhi[1];
      t47 = t12*t12;
      t55 = sens*(t1*hy3-hx3*t6)*t15-3.0/2.0*t9/t13/t47*(2.0*t1*t5+2.0*t6*t3);
      t56 = t39*t55;
      t57 = t38+t56;
      t67 = t28*t35*(t25*t57+t27*t38)-t23*t35*(-t20*t57-t22*t38);
      t68 = t31*t67;
      t70 = tabL[1]*t39;
      t71 = t2-t17;
      t73 = b*sin(t71);
      t74 = a*t21;
      t75 = t73+t74;
      t77 = b*cos(t71);
      t78 = a*t26;
      t79 = t77+t78;
      t80 = atan2(t75,t79);
      t82 = a*sin(t80);
      t83 = t31+t82;
      t85 = a*a;
      t86 = b*b;
      t90 = t85+t86+2.0*a*b*cos(t17);
      t91 = sqrt(t90);
      t92 = 1/t91;
      t95 = tabL[0];
      t97 = b*cos(t29);
      t98 = t97*t67;
      t100 = a*cos(t80);
      t101 = t79*t79;
      t102 = t75*t75;
      t104 = 1/(t101+t102);
      t106 = t38-t56;
      t116 = t79*t104*(t77*t106+t78*t38)-t75*t104*(-t73*t106-t74*t38);
      t124 = 1/t91/t90*a;
      t128 = b*sin(t17)*t56;
      t131 = pow(t1-t68+t70*t55*t83*t92+t95*(t98+t100*t116)*t92+t95*t83*t124*t128,2.0);
      t132 = t97+t100;
      t144 = pow(t6+t98-t70*t55*t132*t92-t95*(-t68-t82*t116)*t92-t95*t132*t124*t128,2.0);
      dv = sens*sqrt(t131+t144);

      return dv;
}

/*
 *  coef_dw --
 *
 *  Compute the ratio between the angular velocity of the robot and the 
 *  pseudo-velocity udot for the off axle trailer connection.
 */

static double coef_dw(FLAT_STR *flatPt, 
		      double *h_deriv, 
		      int sens,
		      double *tabPhi, /* phi and its derivatives */
		      double *tabL)   /* elliptic integ and its derivatives */

{
  double dw;
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t18, t20, t22, t23, t24, t26, t28, t29, t30, t32, t35, t44, t54;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];

      t1 = atan2(hy1,hx1);
      t2 = hx1;
      t3 = hy2;
      t5 = hx2;
      t6 = hy1;
      t8 = t2*t3-t5*t6;
      t9 = sens*t8;
      t10 = t2*t2;
      t11 = t6*t6;
      t12 = t10+t11;
      t13 = sqrt(t12);
      t15 = 1/t13/t12;
      t16 = t9*t15;
      t18 = t1-tabPhi[0];
      t20 = b*cos(t18);
      t22 = a*cos(t1);
      t23 = t20+t22;
      t24 = t23*t23;
      t26 = b*sin(t18);
      t28 = a*sin(t1);
      t29 = t26+t28;
      t30 = t29*t29;
      t32 = 1/(t24+t30);
      t35 = t8/t12;
      t44 = t12*t12;
      t54 = t35-tabPhi[1]*(sens*(t2*hy3-hx3*t6)*t15-3.0/2.0*t9/t13/t44*(2.0*t2*t5+2.0*t6*t3));
      dw = t23*t32*(t20*t54+t22*t35)-t29*t32*(-t26*t54-t28*t35);

  return dw;
}

/* 
 *  Computes the derivative of dv wrt udot for the off axle trailer connection.
 */

static double coef_delta_v(FLAT_STR *flatPt,
			   double *h_deriv, int sens,
			   double *tabPhi, /* phi and its derivatives */
			   double *tabL)   /* elliptic integ and its deriv */

{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t17, t18, t20, t21, t22, t23, t25, t26, t27, t28, t29, t31, t32, t33, t34, t35, t36, t37, t38, t39, t40, t42, t44, t45, t47, t49, t52, t53, t55, t56, t57, t60, t62, t65, t67, t68, t69, t70, t71, t73, t74, t75, t77, t78, t79, t80, t82, t83, t85, t86, t88, t90, t91, t92, t95, t97, t98, t100, t101, t102, t103, t104, t105, t106, t109, t111, t114, t116, t118, t119, t121, t124, t125, t126, t127, t128, t130, t131, t132, t137, t138, t140, t141, t143, t144, t146, t148, t149, t150, t151, t155, t158, t160, t161, t163, t164, t165, t166, t180, t183, t185, t190, t191, t192, t194, t195, t211, t212, t214, t215, t216, t217, t219, t227, t229, t231, t232, t233, t235, t236, t240, t243, t245, t262, t269, t272, t274, t276, t277, t280, t282, t284, t286, t288, t289, t312;

  double delta_v;


  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];
  double hx4 = h_deriv[8];
  double hy4 = h_deriv[9];

      t1 = hx1;
      t2 = atan2(hy1,hx1);
      t3 = hy2;
      t5 = hx2;
      t6 = hy1;
      t8 = t1*t3-t5*t6;
      t9 = sens*t8;
      t10 = t1*t1;
      t11 = t6*t6;
      t12 = t10+t11;
      t13 = sqrt(t12);
      t15 = 1/t13/t12;
      t16 = t9*t15;
      t17 = tabPhi[0];
      t18 = t2+t17;
      t20 = a*sin(t18);
      t21 = sin(t2);
      t22 = b*t21;
      t23 = t20+t22;
      t25 = a*cos(t18);
      t26 = cos(t2);
      t27 = b*t26;
      t28 = t25+t27;
      t29 = atan2(t23,t28);
      t31 = b*sin(t29);
      t32 = t28*t28;
      t33 = t23*t23;
      t34 = t32+t33;
      t35 = 1/t34;
      t36 = t28*t35;
      t37 = 1/t12;
      t38 = t8*t37;
      t39 = tabPhi[1];
      t40 = hy3;
      t42 = hx3;
      t44 = t1*t40-t42*t6;
      t45 = sens*t44;
      t47 = t12*t12;
      t49 = 1/t13/t47;
      t52 = 2.0*t1*t5+2.0*t6*t3;
      t53 = t49*t52;
      t55 = t45*t15-3.0/2.0*t9*t53;
      t56 = t39*t55;
      t57 = t38+t56;
      t60 = t25*t57+t27*t38;
      t62 = t23*t35;
      t65 = -t20*t57-t22*t38;
      t67 = t36*t60-t62*t65;
      t68 = t31*t67;
      t69 = tabL[1];
      t70 = t69*t39;
      t71 = t2-t17;
      t73 = b*sin(t71);
      t74 = a*t21;
      t75 = t73+t74;
      t77 = b*cos(t71);
      t78 = a*t26;
      t79 = t77+t78;
      t80 = atan2(t75,t79);
      t82 = a*sin(t80);
      t83 = t31+t82;
      t85 = a*a;
      t86 = b*b;
      t88 = cos(t17);
      t90 = t85+t86+2.0*a*b*t88;
      t91 = sqrt(t90);
      t92 = 1/t91;
      t95 = tabL[0];
      t97 = b*cos(t29);
      t98 = t97*t67;
      t100 = a*cos(t80);
      t101 = t79*t79;
      t102 = t75*t75;
      t103 = t101+t102;
      t104 = 1/t103;
      t105 = t79*t104;
      t106 = t38-t56;
      t109 = t77*t106+t78*t38;
      t111 = t75*t104;
      t114 = -t73*t106-t74*t38;
      t116 = t105*t109-t111*t114;
      t118 = t98+t100*t116;
      t119 = t95*t118;
      t121 = t95*t83;
      t124 = 1/t91/t90*a;
      t125 = t121*t124;
      t126 = sin(t17);
      t127 = b*t126;
      t128 = t127*t56;
      t130 = t1-t68+t70*t55*t83*t92+t119*t92+t125*t128;
      t131 = t130*t130;
      t132 = t97+t100;
      t137 = -t68-t82*t116;
      t138 = t95*t137;
      t140 = t95*t132;
      t141 = t140*t124;
      t143 = t6+t98-t70*t55*t132*t92-t138*t92-t141*t128;
      t144 = t143*t143;
      t146 = sqrt(t131+t144);
      t148 = t67*t67;
      t149 = t97*t148;
      t150 = t34*t34;
      t151 = 1/t150;
      t155 = 2.0*t28*t65+2.0*t23*t60;
      t158 = t57*t57;
      t160 = t44*t37;
      t161 = 1/t47;
      t163 = t8*t161*t52;
      t164 = tabPhi[2];
      t165 = t55*t55;
      t166 = t164*t165;
      t180 = t52*t52;
      t183 = t5*t5;
      t185 = t3*t3;
      t190 = sens*(t5*t40+t1*hy4-hx4*t6-t42*t3)*t15-3.0*t45*t53+15.0/4.0*t9/t13/t47/t12*t180-3.0/2.0*t9*t49*(2.0*t183+2.0*t1*t42+2.0*t185+2.0*t6*t40);
      t191 = t39*t190;
      t192 = t160-t163+t166+t191;
      t194 = t8*t8;
      t195 = t194*t161;
      t211 = -t28*t151*t60*t155+t36*(-t20*t158+t25*t192-t22*t195+t27*t160-t27*t163)+t23*t151*t65*t155-t62*(-t25*t158-t20*t192-t27*t195-t22*t160+t22*t163);
      t212 = t31*t211;
      t214 = t39*t39;
      t215 = tabL[2]*t214;
      t216 = t165*t83;
      t217 = t216*t92;
      t219 = t69*t164;
      t227 = t69*t214;
      t229 = t124*t127;
      t231 = t31*t148;
      t232 = t97*t211;
      t233 = t116*t116;
      t235 = t103*t103;
      t236 = 1/t235;
      t240 = 2.0*t79*t114+2.0*t75*t109;
      t243 = t106*t106;
      t245 = t160-t163-t166-t191;
      t262 = -t79*t236*t109*t240+t105*(-t73*t243+t77*t245-t74*t195+t78*t160-t78*t163)+t75*t236*t114*t240-t111*(-t77*t243-t73*t245-t78*t195-t74*t160+t74*t163);
      t269 = t90*t90;
      t272 = 1/t91/t269*t85;
      t274 = t126*t126;
      t276 = t214*t165;
      t277 = t86*t274*t276;
      t280 = b*t88*t276;
      t282 = t127*t166;
      t284 = t127*t191;
      t286 = t5-t149-t212+t215*t217+t219*t217+t70*t190*t83*t92+2.0*t70*t55*t118*t92+2.0*t227*t216*t229+t95*(-t231+t232-t82*t233+t100*t262)*t92+2.0*t119*t124*t128+3.0*t121*t272*t277+t125*t280+t125*t282+t125*t284;
      t288 = t165*t132;
      t289 = t288*t92;
      t312 = t3-t231+t232-t215*t289-t219*t289-t70*t190*t132*t92-2.0*t70*t55*t137*t92-2.0*t227*t288*t229-t95*(-t149-t212-t100*t233-t82*t262)*t92-2.0*t138*t124*t128-3.0*t140*t272*t277-t141*t280-t141*t282-t141*t284;
      delta_v = sens*(1/t146*(2.0*t130*t286+2.0*t143*t312)/2);

      return delta_v;
}

/* 
 *  Computes the derivative of dw wrt udot for the off axle trailer connection.
 */

static double coef_delta_w(FLAT_STR *flatPt,
			   double *h_deriv, int sens,
			   double *tabPhi, /* phi and its derivatives */
			   double *tabL)   /* elliptic integ and its deriv */

{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double delta_w;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t18, t20, t22, t23, t24, t26, t28, t29, t30, t31, t32, t33, t35, t36, t37, t38, t40, t42, t43, t45, t47, t50, t51, t53, t55, t58, t61, t64, t67, t69, t71, t72, t74, t76, t91, t94, t96, t103, t105, t106;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];
  double hx4 = h_deriv[8];
  double hy4 = h_deriv[9];

      t1 = atan2(hy1,hx1);
      t2 = hx1;
      t3 = hy2;
      t5 = hx2;
      t6 = hy1;
      t8 = t2*t3-t5*t6;
      t9 = sens*t8;
      t10 = t2*t2;
      t11 = t6*t6;
      t12 = t10+t11;
      t13 = sqrt(t12);
      t15 = 1/t13/t12;
      t16 = t9*t15;
      t18 = t1-tabPhi[0];
      t20 = b*cos(t18);
      t22 = a*cos(t1);
      t23 = t20+t22;
      t24 = t23*t23;
      t26 = b*sin(t18);
      t28 = a*sin(t1);
      t29 = t26+t28;
      t30 = t29*t29;
      t31 = t24+t30;
      t32 = t31*t31;
      t33 = 1/t32;
      t35 = 1/t12;
      t36 = t8*t35;
      t37 = tabPhi[1];
      t38 = hy3;
      t40 = hx3;
      t42 = t2*t38-t40*t6;
      t43 = sens*t42;
      t45 = t12*t12;
      t47 = 1/t13/t45;
      t50 = 2.0*t2*t5+2.0*t6*t3;
      t51 = t47*t50;
      t53 = t43*t15-3.0/2.0*t9*t51;
      t55 = t36-t37*t53;
      t58 = t20*t55+t22*t36;
      t61 = -t26*t55-t28*t36;
      t64 = 2.0*t23*t61+2.0*t29*t58;
      t67 = 1/t31;
      t69 = t55*t55;
      t71 = t42*t35;
      t72 = 1/t45;
      t74 = t8*t72*t50;
      t76 = t53*t53;
      t91 = t50*t50;
      t94 = t5*t5;
      t96 = t3*t3;
      t103 = t71-t74-tabPhi[2]*t76-t37*(sens*(t5*t38+t2*hy4-hx4*t6-t40*t3)*t15-3.0*t43*t51+15.0/4.0*t9/t13/t45/t12*t91-3.0/2.0*t9*t47*(2.0*t94+2.0*t2*t40+2.0*t96+2.0*t6*t38));
      t105 = t8*t8;
      t106 = t105*t72;
      delta_w = -t23*t33*t58*t64+t23*t67*(-t26*t69+t20*t103-t28*t106+t22*t71-t22*t74)+t29*t33*t61*t64-t29*t67*(-t20*t69-t26*t103-t22*t106-t28*t71+t28*t74);

  return delta_w;
}

/*
 *  coef_dv_trailer --
 *
 *  Compute the ratio between the linear velocity of the trailer and the 
 *  pseudo-velocity udot for the off axle trailer connection.
 */

static double coef_dv_trailer(FLAT_STR *flatPt,
			      double *h_deriv, int sens, 
			      double *tabPhi, /* phi and its derivatives */
			      double *tabL)   /* elliptic integ and 
						 its derivatives */
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;
  
  double dv;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t17, t18, t20, t21, t22, t23, t25, t26, t27, t28, t29, t31, t32, t33, t35, t38, t39, t47, t55, t56, t57, t67, t68, t70, t71, t73, t74, t75, t77, t78, t79, t80, t82, t83, t85, t86, t90, t91, t92, t95, t97, t98, t99, t101, t103, t113, t116, t117, t124, t128, t131, t132, t144;
  
  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];

  t1 = hx1;
  t2 = atan2(hy1,hx1);
  t3 = hy2;
  t5 = hx2;
  t6 = hy1;
  t8 = t1*t3-t5*t6;
  t9 = sens*t8;
  t10 = t1*t1;
  t11 = t6*t6;
  t12 = t10+t11;
  t13 = sqrt(t12);
  t15 = 1/t13/t12;
  t16 = t9*t15;
  t17 = tabPhi[0];
  t18 = t2-t17;
  t20 = b*sin(t18);
  t21 = sin(t2);
  t22 = a*t21;
  t23 = t20+t22;
  t25 = b*cos(t18);
  t26 = cos(t2);
  t27 = a*t26;
  t28 = t25+t27;
  t29 = atan2(t23,t28);
  t31 = a*sin(t29);
  t32 = t28*t28;
  t33 = t23*t23;
  t35 = 1/(t32+t33);
  t38 = t8/t12;
  t39 = tabPhi[1];
  t47 = t12*t12;
  t55 = sens*(t1*hy3-hx3*t6)*t15-3.0/2.0*t9/t13/t47*(2.0*t1*t5+2.0*t6*t3);
  t56 = t39*t55;
  t57 = t38-t56;
  t67 = t28*t35*(t25*t57+t27*t38)-t23*t35*(-t20*t57-t22*t38);
  t68 = t31*t67;
  t70 = tabL[1]*t39;
  t71 = t2+t17;
  t73 = a*sin(t71);
  t74 = b*t21;
  t75 = t73+t74;
  t77 = a*cos(t71);
  t78 = b*t26;
  t79 = t77+t78;
  t80 = atan2(t75,t79);
  t82 = b*sin(t80);
  t83 = t82+t31;
  t85 = a*a;
  t86 = b*b;
  t90 = t85+t86+2.0*a*b*cos(t17);
  t91 = sqrt(t90);
  t92 = 1/t91;
  t95 = tabL[0];
  t97 = b*cos(t80);
  t98 = t79*t79;
  t99 = t75*t75;
  t101 = 1/(t98+t99);
  t103 = t38+t56;
  t113 = t79*t101*(t77*t103+t78*t38)-t75*t101*(-t73*t103-t74*t38);
  t116 = a*cos(t29);
  t117 = t116*t67;
  t124 = 1/t91/t90*a;
  t128 = b*sin(t17)*t56;
  t131 = pow(t1+t68+t70*t55*t83*t92+t95*(t97*t113+t117)*t92+t95*t83*t124*t128,2.0);
  t132 = t97+t116;
  t144 = pow(t6-t117-t70*t55*t132*t92-t95*(-t82*t113-t68)*t92-t95*t132*t124*t128,2.0);
  dv = sqrt(t131+t144);

  return dv;
}

/*
 *  coef_dw --
 *
 *  Compute the ratio between the angular velocity of the trailer and the 
 *  pseudo-velocity udot for the off axle trailer connection.
 */

static double coef_dw_trailer(FLAT_STR *flatPt,
			      double *h_deriv, int sens,
			      double *tabPhi, /* phi and its derivatives */
			      double *tabL)   /* elliptic integ and its 
						 derivatives */

{
  double dw;
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t18, t20, t22, t23, t24, t26, t28, t29, t30, t32, t35, t44, t54;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];

  t1 = atan2(hy1,hx1);
  t2 = hx1;
  t3 = hy2;
  t5 = hx2;
  t6 = hy1;
  t8 = t2*t3-t5*t6;
  t9 = sens*t8;
  t10 = t2*t2;
  t11 = t6*t6;
  t12 = t10+t11;
  t13 = sqrt(t12);
  t15 = 1/t13/t12;
  t16 = t9*t15;
  t18 = t1+tabPhi[0];
  t20 = a*cos(t18);
  t22 = b*cos(t1);
  t23 = t20+t22;
  t24 = t23*t23;
  t26 = a*sin(t18);
  t28 = b*sin(t1);
  t29 = t26+t28;
  t30 = t29*t29;
  t32 = 1/(t24+t30);
  t35 = t8/t12;
  t44 = t12*t12;
  t54 = t35+tabPhi[1]*(sens*(t2*hy3-hx3*t6)*t15-3.0/2.0*t9/t13/t44*(2.0*t2*t5+2.0*t6*t3));
  dw = t23*t32*(t20*t54+t22*t35)-t29*t32*(-t26*t54-t28*t35);

  return dw;
}

/* 
 *  Computes the derivative of dv_trailer wrt udot
 */

static double coef_delta_v_trailer(FLAT_STR *flatPt,
				   double *h_deriv, int sens,
				   double *tabPhi, /* phi and its deriv */
				   double *tabL)   /* elliptic integ and 
						      its deriv */

{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double delta_v;

    double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t17, t18, t20, t21, t22, t23, t25, t26, t27, t28, t29, t31, t32, t33, t34, t35, t36, t37, t38, t39, t40, t42, t44, t45, t47, t49, t52, t53, t55, t56, t57, t60, t62, t65, t67, t68, t69, t70, t71, t73, t74, t75, t77, t78, t79, t80, t82, t83, t85, t86, t88, t90, t91, t92, t95, t97, t98, t99, t100, t101, t102, t103, t106, t108, t111, t113, t116, t117, t118, t119, t121, t124, t125, t126, t127, t128, t130, t131, t132, t137, t138, t140, t141, t143, t144, t146, t148, t149, t150, t151, t155, t158, t160, t161, t163, t164, t165, t166, t180, t183, t185, t190, t191, t192, t194, t195, t211, t212, t214, t215, t216, t217, t219, t227, t229, t231, t233, t234, t238, t241, t243, t260, t262, t263, t269, t272, t274, t276, t277, t280, t282, t284, t286, t288, t289, t312;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];
  double hx4 = h_deriv[8];
  double hy4 = h_deriv[9];

  t1 = hx1;
  t2 = atan2(hy1,hx1);
  t3 = hy2;
  t5 = hx2;
  t6 = hy1;
  t8 = t1*t3-t5*t6;
  t9 = sens*t8;
  t10 = t1*t1;
  t11 = t6*t6;
  t12 = t10+t11;
  t13 = sqrt(t12);
  t15 = 1/t13/t12;
  t16 = t9*t15;
  t17 = tabPhi[0];
  t18 = t2-t17;
  t20 = b*sin(t18);
  t21 = sin(t2);
  t22 = a*t21;
  t23 = t20+t22;
  t25 = b*cos(t18);
  t26 = cos(t2);
  t27 = a*t26;
  t28 = t25+t27;
  t29 = atan2(t23,t28);
  t31 = a*sin(t29);
  t32 = t28*t28;
  t33 = t23*t23;
  t34 = t32+t33;
  t35 = 1/t34;
  t36 = t28*t35;
  t37 = 1/t12;
  t38 = t8*t37;
  t39 = tabPhi[1];
  t40 = hy3;
  t42 = hx3;
  t44 = t1*t40-t42*t6;
  t45 = sens*t44;
  t47 = t12*t12;
  t49 = 1/t13/t47;
  t52 = 2.0*t1*t5+2.0*t6*t3;
  t53 = t49*t52;
  t55 = t45*t15-3.0/2.0*t9*t53;
  t56 = t39*t55;
  t57 = t38-t56;
  t60 = t25*t57+t27*t38;
  t62 = t23*t35;
  t65 = -t20*t57-t22*t38;
  t67 = t36*t60-t62*t65;
  t68 = t31*t67;
  t69 = tabL[1];
  t70 = t69*t39;
  t71 = t2+t17;
  t73 = a*sin(t71);
  t74 = b*t21;
  t75 = t73+t74;
  t77 = a*cos(t71);
  t78 = b*t26;
  t79 = t77+t78;
  t80 = atan2(t75,t79);
  t82 = b*sin(t80);
  t83 = t82+t31;
  t85 = a*a;
  t86 = b*b;
  t88 = cos(t17);
  t90 = t85+t86+2.0*a*b*t88;
  t91 = sqrt(t90);
  t92 = 1/t91;
  t95 = tabL[0];
  t97 = b*cos(t80);
  t98 = t79*t79;
  t99 = t75*t75;
  t100 = t98+t99;
  t101 = 1/t100;
  t102 = t79*t101;
  t103 = t38+t56;
  t106 = t77*t103+t78*t38;
  t108 = t75*t101;
  t111 = -t73*t103-t74*t38;
  t113 = t102*t106-t108*t111;
  t116 = a*cos(t29);
  t117 = t116*t67;
  t118 = t97*t113+t117;
  t119 = t95*t118;
  t121 = t95*t83;
  t124 = 1/t91/t90*a;
  t125 = t121*t124;
  t126 = sin(t17);
  t127 = b*t126;
  t128 = t127*t56;
  t130 = t1+t68+t70*t55*t83*t92+t119*t92+t125*t128;
  t131 = t130*t130;
  t132 = t97+t116;
  t137 = -t82*t113-t68;
  t138 = t95*t137;
  t140 = t95*t132;
  t141 = t140*t124;
  t143 = t6-t117-t70*t55*t132*t92-t138*t92-t141*t128;
  t144 = t143*t143;
  t146 = sqrt(t131+t144);
  t148 = t67*t67;
  t149 = t116*t148;
  t150 = t34*t34;
  t151 = 1/t150;
  t155 = 2.0*t28*t65+2.0*t23*t60;
  t158 = t57*t57;
  t160 = t44*t37;
  t161 = 1/t47;
  t163 = t8*t161*t52;
  t164 = tabPhi[2];
  t165 = t55*t55;
  t166 = t164*t165;
  t180 = t52*t52;
  t183 = t5*t5;
  t185 = t3*t3;
  t190 = sens*(t5*t40+t1*hy4-hx4*t6-t42*t3)*t15-3.0*t45*t53+15.0/4.0*t9/t13/t47/t12*t180-3.0/2.0*t9*t49*(2.0*t183+2.0*t1*t42+2.0*t185+2.0*t6*t40);
  t191 = t39*t190;
  t192 = t160-t163-t166-t191;
  t194 = t8*t8;
  t195 = t194*t161;
  t211 = -t28*t151*t60*t155+t36*(-t20*t158+t25*t192-t22*t195+t27*t160-t27*t163)+t23*t151*t65*t155-t62*(-t25*t158-t20*t192-t27*t195-t22*t160+t22*t163);
  t212 = t31*t211;
  t214 = t39*t39;
  t215 = tabL[2]*t214;
  t216 = t165*t83;
  t217 = t216*t92;
  t219 = t69*t164;
  t227 = t69*t214;
  t229 = t124*t127;
  t231 = t113*t113;
  t233 = t100*t100;
  t234 = 1/t233;
  t238 = 2.0*t79*t111+2.0*t75*t106;
  t241 = t103*t103;
  t243 = t160-t163+t166+t191;
  t260 = -t79*t234*t106*t238+t102*(-t73*t241+t77*t243-t74*t195+t78*t160-t78*t163)+t75*t234*t111*t238-t108*(-t77*t241-t73*t243-t78*t195-t74*t160+t74*t163);
  t262 = t31*t148;
  t263 = t116*t211;
  t269 = t90*t90;
  t272 = 1/t91/t269*t85;
  t274 = t126*t126;
  t276 = t214*t165;
  t277 = t86*t274*t276;
  t280 = b*t88*t276;
  t282 = t127*t166;
  t284 = t127*t191;
  t286 = t5+t149+t212+t215*t217+t219*t217+t70*t190*t83*t92+2.0*t70*t55*t118*t92+2.0*t227*t216*t229+t95*(-t82*t231+t97*t260-t262+t263)*t92+2.0*t119*t124*t128+3.0*t121*t272*t277+t125*t280+t125*t282+t125*t284;
  t288 = t165*t132;
  t289 = t288*t92;
  t312 = t3+t262-t263-t215*t289-t219*t289-t70*t190*t132*t92-2.0*t70*t55*t137*t92-2.0*t227*t288*t229-t95*(-t97*t231-t82*t260-t149-t212)*t92-2.0*t138*t124*t128-3.0*t140*t272*t277-t141*t280-t141*t282-t141*t284;
  delta_v = 1/t146*(2.0*t130*t286+2.0*t143*t312)/2;

  return delta_v;
}

/* 
 *  Computes the derivative of dw_trailer wrt udot for the off axle trailer
 *  connection.
 */

static double coef_delta_w_trailer(FLAT_STR *flatPt,
				   double *h_deriv, int sens,
				   double *tabPhi, /* phi and its deriv */
				   double *tabL)   /* elliptic integ and 
						      its deriv */

{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double delta_w;


  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t18, t20, t22, t23, t24, t26, t28, t29, t30, t31, t32, t33, t35, t36, t37, t38, t40, t42, t43, t45, t47, t50, t51, t53, t55, t58, t61, t64, t67, t69, t71, t72, t74, t76, t91, t94, t96, t103, t105, t106;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];
  double hx4 = h_deriv[8];
  double hy4 = h_deriv[9];

  t1 = atan2(hy1,hx1);
  t2 = hx1;
  t3 = hy2;
  t5 = hx2;
  t6 = hy1;
  t8 = t2*t3-t5*t6;
  t9 = sens*t8;
  t10 = t2*t2;
  t11 = t6*t6;
  t12 = t10+t11;
  t13 = sqrt(t12);
  t15 = 1/t13/t12;
  t16 = t9*t15;
  t18 = t1+tabPhi[0];
  t20 = a*cos(t18);
  t22 = b*cos(t1);
  t23 = t20+t22;
  t24 = t23*t23;
  t26 = a*sin(t18);
  t28 = b*sin(t1);
  t29 = t26+t28;
  t30 = t29*t29;
  t31 = t24+t30;
  t32 = t31*t31;
  t33 = 1/t32;
  t35 = 1/t12;
  t36 = t8*t35;
  t37 = tabPhi[1];
  t38 = hy3;
  t40 = hx3;
  t42 = t2*t38-t40*t6;
  t43 = sens*t42;
  t45 = t12*t12;
  t47 = 1/t13/t45;
  t50 = 2.0*t2*t5+2.0*t6*t3;
  t51 = t47*t50;
  t53 = t43*t15-3.0/2.0*t9*t51;
  t55 = t36+t37*t53;
  t58 = t20*t55+t22*t36;
  t61 = -t26*t55-t28*t36;
  t64 = 2.0*t23*t61+2.0*t29*t58;
  t67 = 1/t31;
  t69 = t55*t55;
  t71 = t42*t35;
  t72 = 1/t45;
  t74 = t8*t72*t50;
  t76 = t53*t53;
  t91 = t50*t50;
  t94 = t5*t5;
  t96 = t3*t3;
  t103 = t71-t74+tabPhi[2]*t76+t37*(sens*(t5*t38+t2*hy4-hx4*t6-t40*t3)*t15-3.0*t43*t51+15.0/4.0*t9/t13/t45/t12*t91-3.0/2.0*t9*t47*(2.0*t94+2.0*t2*t40+2.0*t96+2.0*t6*t38));
  t105 = t8*t8;
  t106 = t105*t72;
  delta_w = -t23*t33*t58*t64+t23*t67*(-t26*t69+t20*t103-t28*t106+t22*t71-t22*t74)+t29*t33*t61*t64-t29*t67*(-t20*t69-t26*t103-t22*t106-t28*t71+t28*t74);

  return delta_w;
}



/*
 *   deltaPrime_v --        
 *
 *        derivatives of delta_v
 *
 */

static double deltaPrime_v(FLAT_STR *flatPt,
			   double *h_deriv, int sens,
			   double *tabPhi, /* phi and its derivatives */
			   double *tabL)   /* elliptic integ and its deriv */
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double ddelta_v = 0;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t17, t18, t20, t21, t22, t23, t25, t26, t27, t28, t29, t31, t32, t33, t34, t35, t36, t37, t38, t39, t40, t42, t44, t45, t47, t49, t52, t53, t55, t56, t57, t60, t62, t65, t67, t68, t69, t70, t71, t73, t74, t75, t77, t78, t79, t80, t82, t83, t84, t85, t86, t88, t90, t91, t92, t95, t97, t98, t100, t101, t102, t103, t104, t105, t106, t109, t111, t114, t116, t118, t119, t121, t124, t125, t126, t127, t128, t130, t131, t132, t133, t137, t138, t140, t141, t143, t144, t145, t146, t149, t150, t151, t152, t153, t156, t159, t161, t162, t163, t164, t165, t166, t167, t169, t171, t174, t175, t178, t180, t181, t182, t184, t186, t188, t189, t191, t192, t193, t195, t196, t200, t202, t210, t212, t213, t214, t215, t216, t217, t218, t220, t223, t228, t230, t232, t233, t234, t236, t237, t238, t241, t244, t246, t251, t253, t261, t263, t265, t266, t268, t270, t273, t274, t275, t276, t277, t278, t280, t281, t283, t285, t287, t289, t290, t294, t303, t304, t306, t308, t313, t316, t319, t321, t323, t328, t330, t332, t337, t339, t341, t343, t344, t346, t347, t348, t349, t350, t352, t365, t381, t382, t383, t386, t388, t391, t420, t421, t422, t423, t424, t425, t430, t438, t439, t443, t444, t445, t446, t447, t451, t455, t456, t460, t463, t464, t466, t468, t469, t471, t473, t476, t478, t484, t489, t491, t493, t495, t502, t504, t507, t508, t509, t514, t519, t520, t521, t522, t524, t527, t529, t534, t536, t538, t543, t545, t547, t579, t584, t587, t588, t589, t595, t601, t602, t621, t646;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];
  double hx4 = h_deriv[8];
  double hy4 = h_deriv[9];
  double hx5 = h_deriv[10];
  double hy5 = h_deriv[11];

      t1 = hx1;
      t2 = atan2(hy1,hx1);
      t3 = hy2;
      t5 = hx2;
      t6 = hy1;
      t8 = t1*t3-t5*t6;
      t9 = sens*t8;
      t10 = t1*t1;
      t11 = t6*t6;
      t12 = t10+t11;
      t13 = sqrt(t12);
      t15 = 1/t13/t12;
      t16 = t9*t15;
      t17 = tabPhi[0];
      t18 = t2+t17;
      t20 = a*sin(t18);
      t21 = sin(t2);
      t22 = b*t21;
      t23 = t20+t22;
      t25 = a*cos(t18);
      t26 = cos(t2);
      t27 = b*t26;
      t28 = t25+t27;
      t29 = atan2(t23,t28);
      t31 = b*sin(t29);
      t32 = t28*t28;
      t33 = t23*t23;
      t34 = t32+t33;
      t35 = 1/t34;
      t36 = t28*t35;
      t37 = 1/t12;
      t38 = t8*t37;
      t39 = tabPhi[1];
      t40 = hy3;
      t42 = hx3;
      t44 = t1*t40-t42*t6;
      t45 = sens*t44;
      t47 = t12*t12;
      t49 = 1/t13/t47;
      t52 = 2.0*t1*t5+2.0*t6*t3;
      t53 = t49*t52;
      t55 = t45*t15-3.0/2.0*t9*t53;
      t56 = t39*t55;
      t57 = t38+t56;
      t60 = t25*t57+t27*t38;
      t62 = t23*t35;
      t65 = -t20*t57-t22*t38;
      t67 = t36*t60-t62*t65;
      t68 = t31*t67;
      t69 = tabL[1];
      t70 = t69*t39;
      t71 = t2-t17;
      t73 = b*sin(t71);
      t74 = a*t21;
      t75 = t73+t74;
      t77 = b*cos(t71);
      t78 = a*t26;
      t79 = t77+t78;
      t80 = atan2(t75,t79);
      t82 = a*sin(t80);
      t83 = t31+t82;
      t84 = t55*t83;
      t85 = a*a;
      t86 = b*b;
      t88 = cos(t17);
      t90 = t85+t86+2.0*a*b*t88;
      t91 = sqrt(t90);
      t92 = 1/t91;
      t95 = tabL[0];
      t97 = b*cos(t29);
      t98 = t97*t67;
      t100 = a*cos(t80);
      t101 = t79*t79;
      t102 = t75*t75;
      t103 = t101+t102;
      t104 = 1/t103;
      t105 = t79*t104;
      t106 = t38-t56;
      t109 = t77*t106+t78*t38;
      t111 = t75*t104;
      t114 = -t73*t106-t74*t38;
      t116 = t105*t109-t111*t114;
      t118 = t98+t100*t116;
      t119 = t95*t118;
      t121 = t95*t83;
      t124 = 1/t91/t90*a;
      t125 = t121*t124;
      t126 = sin(t17);
      t127 = b*t126;
      t128 = t127*t56;
      t130 = t1-t68+t70*t84*t92+t119*t92+t125*t128;
      t131 = t130*t130;
      t132 = t97+t100;
      t133 = t55*t132;
      t137 = -t68-t82*t116;
      t138 = t95*t137;
      t140 = t95*t132;
      t141 = t140*t124;
      t143 = t6+t98-t70*t133*t92-t138*t92-t141*t128;
      t144 = t143*t143;
      t145 = t131+t144;
      t146 = sqrt(t145);
      t149 = t67*t67;
      t150 = t97*t149;
      t151 = t34*t34;
      t152 = 1/t151;
      t153 = t28*t152;
      t156 = 2.0*t28*t65+2.0*t23*t60;
      t159 = t57*t57;
      t161 = t44*t37;
      t162 = 1/t47;
      t163 = t8*t162;
      t164 = t163*t52;
      t165 = tabPhi[2];
      t166 = t55*t55;
      t167 = t165*t166;
      t169 = hy4;
      t171 = hx4;
      t174 = t5*t40+t1*t169-t171*t6-t42*t3;
      t175 = sens*t174;
      t178 = t47*t12;
      t180 = 1/t13/t178;
      t181 = t52*t52;
      t182 = t180*t181;
      t184 = t5*t5;
      t186 = t3*t3;
      t188 = 2.0*t184+2.0*t1*t42+2.0*t186+2.0*t6*t40;
      t189 = t49*t188;
      t191 = t175*t15-3.0*t45*t53+15.0/4.0*t9*t182-3.0/2.0*t9*t189;
      t192 = t39*t191;
      t193 = t161-t164+t167+t192;
      t195 = t8*t8;
      t196 = t195*t162;
      t200 = -t20*t159+t25*t193-t22*t196+t27*t161-t27*t164;
      t202 = t23*t152;
      t210 = -t25*t159-t20*t193-t27*t196-t22*t161+t22*t164;
      t212 = -t153*t60*t156+t36*t200+t202*t65*t156-t62*t210;
      t213 = t31*t212;
      t214 = tabL[2];
      t215 = t39*t39;
      t216 = t214*t215;
      t217 = t166*t83;
      t218 = t217*t92;
      t220 = t69*t165;
      t223 = t191*t83*t92;
      t228 = t69*t215;
      t230 = t124*t127;
      t232 = t31*t149;
      t233 = t97*t212;
      t234 = t116*t116;
      t236 = t103*t103;
      t237 = 1/t236;
      t238 = t79*t237;
      t241 = 2.0*t79*t114+2.0*t75*t109;
      t244 = t106*t106;
      t246 = t161-t164-t167-t192;
      t251 = -t73*t244+t77*t246-t74*t196+t78*t161-t78*t164;
      t253 = t75*t237;
      t261 = -t77*t244-t73*t246-t78*t196-t74*t161+t74*t164;
      t263 = -t238*t109*t241+t105*t251+t253*t114*t241-t111*t261;
      t265 = -t232+t233-t82*t234+t100*t263;
      t266 = t95*t265;
      t268 = t119*t124;
      t270 = t90*t90;
      t273 = 1/t91/t270*t85;
      t274 = t121*t273;
      t275 = t126*t126;
      t276 = t86*t275;
      t277 = t215*t166;
      t278 = t276*t277;
      t280 = b*t88;
      t281 = t280*t277;
      t283 = t127*t167;
      t285 = t127*t192;
      t287 = t5-t150-t213+t216*t218+t220*t218+t70*t223+2.0*t70*t55*t118*t92+2.0*t228*t217*t230+t266*t92+2.0*t268*t128+3.0*t274*t278+t125*t281+t125*t283+t125*t285;
      t289 = t166*t132;
      t290 = t289*t92;
      t294 = t191*t132*t92;
      t303 = -t150-t213-t100*t234-t82*t263;
      t304 = t95*t303;
      t306 = t138*t124;
      t308 = t140*t273;
      t313 = t3-t232+t233-t216*t290-t220*t290-t70*t294-2.0*t70*t55*t137*t92-2.0*t228*t289*t230-t304*t92-2.0*t306*t128-3.0*t308*t278-t141*t281-t141*t283-t141*t285;
      t316 = pow(2.0*t130*t287+2.0*t143*t313,2.0);
      t319 = t287*t287;
      t321 = 1/t151/t34;
      t323 = t156*t156;
      t328 = t65*t65;
      t330 = t60*t60;
      t332 = 2.0*t328+2.0*t28*t210+2.0*t330+2.0*t23*t200;
      t337 = t159*t57;
      t339 = t57*t193;
      t341 = t174*t37;
      t343 = t44*t162*t52;
      t344 = 1/t178;
      t346 = t8*t344*t181;
      t347 = t163*t188;
      t348 = tabPhi[3];
      t349 = t166*t55;
      t350 = t348*t349;
      t352 = t165*t55*t191;
      t365 = t47*t47;
      t381 = sens*(2.0*t5*t169+t1*hy5-hx5*t6-2.0*t171*t3)*t15-9.0/2.0*t175*t53+45.0/4.0*t45*t182-9.0/2.0*t45*t189-105.0/8.0*t9/t13/t365*t181*t52+45.0/4.0*t9*t180*t52*t188-3.0/2.0*t9*t49*(6.0*t5*t42+2.0*t1*t171+6.0*t3*t40+2.0*t6*t169);
      t382 = t39*t381;
      t383 = t341-2.0*t343+2.0*t346-t347+t350+3.0*t352+t382;
      t386 = t195*t8*t344;
      t388 = t163*t44;
      t391 = t195*t344*t52;
      t420 = 2.0*t28*t321*t60*t323-2.0*t153*t200*t156-t153*t60*t332+t65*t35*t200+t36*(-t25*t337-3.0*t20*t339+t25*t383-t27*t386-3.0*t22*t388+3.0*t22*t391+t27*t341-2.0*t27*t343+2.0*t27*t346-t27*t347)-2.0*t23*t321*t65*t323+2.0*t202*t210*t156+t202*t65*t332-t60*t35*t210-t62*(t20*t337-3.0*t25*t339-t20*t383+t22*t386-3.0*t27*t388+3.0*t27*t391-t22*t341+2.0*t22*t343-2.0*t22*t346+t22*t347);
      t421 = t31*t420;
      t422 = t149*t67;
      t423 = t31*t422;
      t424 = t166*t118;
      t425 = t424*t92;
      t430 = t214*t39*t349;
      t438 = t165*t349*t39;
      t439 = t276*t438;
      t443 = t215*t39;
      t444 = t69*t443;
      t445 = t349*t83;
      t446 = t444*t445;
      t447 = t273*t276;
      t451 = t124*t280;
      t455 = tabL[3]*t443;
      t456 = t445*t92;
      t460 = t220*t55;
      t463 = t215*t55*t191;
      t464 = t280*t463;
      t466 = t127*t382;
      t468 = -t421+t423+3.0*t220*t425+6.0*t228*t424*t230+3.0*t430*t83*t92*t165+3.0*t70*t191*t118*t92+9.0*t274*t439+3.0*t268*t283+3.0*t268*t285+9.0*t446*t447+9.0*t119*t273*t278+3.0*t446*t451+3.0*t268*t281+t455*t456+3.0*t266*t124*t128+3.0*t460*t223+3.0*t125*t464+t125*t466;
      t469 = t276*t463;
      t471 = t127*t350;
      t473 = t127*t352;
      t476 = t443*t349;
      t478 = t86*t126*t476*t88;
      t484 = 1/t91/t270/t90*t85*a;
      t489 = t86*b*t275*t126*t476;
      t491 = t216*t55;
      t493 = t127*t476;
      t495 = t280*t438;
      t502 = t124*t127*t191;
      t504 = t214*t443;
      t507 = t67*t212;
      t508 = t97*t507;
      t509 = t69*t348;
      t514 = t124*t127*t39;
      t519 = t97*t422;
      t520 = t31*t507;
      t521 = t97*t420;
      t522 = t234*t116;
      t524 = t116*t263;
      t527 = 1/t236/t103;
      t529 = t241*t241;
      t534 = t114*t114;
      t536 = t109*t109;
      t538 = 2.0*t534+2.0*t79*t261+2.0*t536+2.0*t75*t251;
      t543 = t244*t106;
      t545 = t106*t246;
      t547 = t341-2.0*t343+2.0*t346-t347-t350-3.0*t352-t382;
      t579 = 2.0*t79*t527*t109*t529-2.0*t238*t251*t241-t238*t109*t538+t114*t104*t251+t105*(-t77*t543-3.0*t73*t545+t77*t547-t78*t386-3.0*t74*t388+3.0*t74*t391+t78*t341-2.0*t78*t343+2.0*t78*t346-t78*t347)-2.0*t75*t527*t114*t529+2.0*t253*t261*t241+t253*t114*t538-t109*t104*t261-t111*(t73*t543-3.0*t77*t545-t73*t547+t74*t386-3.0*t78*t388+3.0*t78*t391-t74*t341+2.0*t74*t343-2.0*t74*t346+t74*t347);
      t584 = 9.0*t274*t469+t125*t471+3.0*t125*t473+9.0*t274*t478+15.0*t121*t484*t489+3.0*t491*t223-t125*t493+3.0*t125*t495+t70*t381*t83*t92+6.0*t228*t84*t502+t42+3.0*t504*t445*t230-3.0*t508+t509*t456+3.0*t216*t425+6.0*t220*t445*t514+3.0*t70*t55*t265*t92+t95*(-t519-3.0*t520+t521-t100*t522-3.0*t82*t524+t100*t579)*t92;
      t587 = t313*t313;
      t588 = t349*t132;
      t589 = t588*t92;
      t595 = t444*t588;
      t601 = t166*t137;
      t602 = t601*t92;
      t621 = t521-t509*t589-3.0*t306*t281-3.0*t70*t191*t137*t92-9.0*t595*t447-3.0*t430*t132*t92*t165-t455*t589-3.0*t220*t602-6.0*t228*t601*t230-3.0*t70*t55*t303*t92-6.0*t220*t588*t514-3.0*t595*t451-3.0*t304*t124*t128-3.0*t504*t588*t230-3.0*t216*t602-15.0*t140*t484*t489-3.0*t141*t473-9.0*t308*t469;
      t646 = -3.0*t141*t464+t141*t493-9.0*t308*t478-9.0*t308*t439-t141*t471-t141*t466-3.0*t141*t495-3.0*t306*t283-3.0*t460*t294-3.0*t491*t294-t70*t381*t132*t92-3.0*t306*t285-6.0*t228*t133*t502-9.0*t138*t273*t278+t40-t95*(t423-3.0*t508-t421+t82*t522-3.0*t100*t524-t82*t579)*t92-t519-3.0*t520;
      ddelta_v = sens*(-1/t146/t145*t316/4+1/t146*(2.0*t319+2.0*t130*(t468+t584)+2.0*t587+2.0*t143*(t621+t646))/2);

  return ddelta_v;
}

/*
 *   deltaPrime_w --        
 *
 *        derivatives of delta_w
 *
 */

static double deltaPrime_w(FLAT_STR *flatPt,
			   double *h_deriv, int sens,
			   double *tabPhi, /* phi and its derivatives */
			   double *tabL)   /* elliptic integ and its deriv */
{
  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  double ddelta_w=0;

  double t1, t2, t3, t5, t6, t8, t9, t10, t11, t12, t13, t15, t16, t18, t20, t22, t23, t24, t26, t28, t29, t30, t31, t32, t34, t36, t37, t38, t39, t41, t43, t44, t46, t48, t51, t52, t54, t56, t59, t62, t65, t66, t69, t70, t71, t73, t74, t75, t76, t77, t78, t81, t83, t86, t87, t90, t92, t93, t94, t96, t98, t100, t101, t103, t105, t107, t108, t112, t115, t121, t123, t125, t128, t132, t134, t136, t138, t139, t141, t142, t160, t178, t181, t183, t186, t197;

  double hx1 = h_deriv[2];
  double hy1 = h_deriv[3];
  double hx2 = h_deriv[4];
  double hy2 = h_deriv[5];
  double hx3 = h_deriv[6];
  double hy3 = h_deriv[7];
  double hx4 = h_deriv[8];
  double hy4 = h_deriv[9];
  double hx5 = h_deriv[10];
  double hy5 = h_deriv[11];

      t1 = atan2(hy1,hx1);
      t2 = hx1;
      t3 = hy2;
      t5 = hx2;
      t6 = hy1;
      t8 = t2*t3-t5*t6;
      t9 = sens*t8;
      t10 = t2*t2;
      t11 = t6*t6;
      t12 = t10+t11;
      t13 = sqrt(t12);
      t15 = 1/t13/t12;
      t16 = t9*t15;
      t18 = t1-tabPhi[0];
      t20 = b*cos(t18);
      t22 = a*cos(t1);
      t23 = t20+t22;
      t24 = t23*t23;
      t26 = b*sin(t18);
      t28 = a*sin(t1);
      t29 = t26+t28;
      t30 = t29*t29;
      t31 = t24+t30;
      t32 = t31*t31;
      t34 = 1/t32/t31;
      t36 = 1/t12;
      t37 = t8*t36;
      t38 = tabPhi[1];
      t39 = hy3;
      t41 = hx3;
      t43 = t2*t39-t41*t6;
      t44 = sens*t43;
      t46 = t12*t12;
      t48 = 1/t13/t46;
      t51 = 2.0*t2*t5+2.0*t6*t3;
      t52 = t48*t51;
      t54 = t44*t15-3.0/2.0*t9*t52;
      t56 = t37-t38*t54;
      t59 = t20*t56+t22*t37;
      t62 = -t26*t56-t28*t37;
      t65 = 2.0*t23*t62+2.0*t29*t59;
      t66 = t65*t65;
      t69 = 1/t32;
      t70 = t23*t69;
      t71 = t56*t56;
      t73 = t43*t36;
      t74 = 1/t46;
      t75 = t8*t74;
      t76 = t75*t51;
      t77 = tabPhi[2];
      t78 = t54*t54;
      t81 = hy4;
      t83 = hx4;
      t86 = t5*t39+t2*t81-t83*t6-t41*t3;
      t87 = sens*t86;
      t90 = t46*t12;
      t92 = 1/t13/t90;
      t93 = t51*t51;
      t94 = t92*t93;
      t96 = t5*t5;
      t98 = t3*t3;
      t100 = 2.0*t96+2.0*t2*t41+2.0*t98+2.0*t6*t39;
      t101 = t48*t100;
      t103 = t87*t15-3.0*t44*t52+15.0/4.0*t9*t94-3.0/2.0*t9*t101;
      t105 = t73-t76-t77*t78-t38*t103;
      t107 = t8*t8;
      t108 = t107*t74;
      t112 = -t26*t71+t20*t105-t28*t108+t22*t73-t22*t76;
      t115 = t62*t62;
      t121 = -t20*t71-t26*t105-t22*t108-t28*t73+t28*t76;
      t123 = t59*t59;
      t125 = 2.0*t115+2.0*t23*t121+2.0*t123+2.0*t29*t112;
      t128 = 1/t31;
      t132 = t71*t56;
      t134 = t56*t105;
      t136 = t86*t36;
      t138 = t43*t74*t51;
      t139 = 1/t90;
      t141 = t8*t139*t93;
      t142 = t75*t100;
      t160 = t46*t46;
      t178 = t136-2.0*t138+2.0*t141-t142-tabPhi[3]*t78*t54-3.0*t77*t54*t103-t38*(sens*(2.0*t5*t81+t2*hy5-hx5*t6-2.0*t83*t3)*t15-9.0/2.0*t87*t52+45.0/4.0*t44*t94-9.0/2.0*t44*t101-105.0/8.0*t9/t13/t160*t93*t51+45.0/4.0*t9*t92*t51*t100-3.0/2.0*t9*t48*(6.0*t5*t41+2.0*t2*t83+6.0*t3*t39+2.0*t6*t81));
      t181 = t107*t8*t139;
      t183 = t75*t43;
      t186 = t107*t139*t51;
      t197 = t29*t69;
      ddelta_w = 2.0*t23*t34*t59*t66-2.0*t70*t112*t65-t70*t59*t125+t62*t128*t112+t23*t128*(-t20*t132-3.0*t26*t134+t20*t178-t22*t181-3.0*t28*t183+3.0*t28*t186+t22*t136-2.0*t22*t138+2.0*t22*t141-t22*t142)-2.0*t29*t34*t62*t66+2.0*t197*t121*t65+t197*t62*t125-t59*t128*t121-t29*t128*(t26*t132-3.0*t20*t134-t26*t178+t28*t181-3.0*t22*t183+3.0*t22*t186-t28*t136+2.0*t28*t138-2.0*t28*t141+t28*t142);

  return ddelta_w;
}

/*
 *   deltaPrime_off --        
 *
 *        derivatives of delta for the off axis trailer connection
 *
 */

int deltaPrime_off(FLAT_STR *flatPt,
		   FLAT_LOCAL_PATH_STR *pathPt, 
		   double localParam, 
		   double *ddelta_v, double *ddelta_w)
{
  double velCoef = pathPt->velCoeff;
  double timeScaling = pathPt->velCoeff;
  double h_deriv[12];
  int sens;
  double tabPhi[4];
  double tabL[4];
  double hx1;
  double hy1;
  double hx2;
  double hy2;
  double kappa;
  int result = TRUE;

  sens = pathPt->v2 > 0 ? 1 : -1;

  /* calcul des derivees de la courbe des sorties plates */
  flatTrailerCombination(&(pathPt->initFlatConf),
			 &(pathPt->finalFlatConf),
			 localParam, pathPt->v2,
			 pathPt->alpha_0, pathPt->alpha_1,
			 5, h_deriv);

  /* transformation des derivees par rapport au parametre local
     en derivees par rapport au parametre global */

  h_deriv[2] /= timeScaling;
  h_deriv[3] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[4] /= timeScaling;
  h_deriv[5] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[6] /= timeScaling;
  h_deriv[7] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[8] /= timeScaling;
  h_deriv[9] /= timeScaling;

  timeScaling *= velCoef;

  h_deriv[10] /= timeScaling;
  h_deriv[11] /= timeScaling;
  
  hx1 = h_deriv[2];
  hy1 = h_deriv[3];
  hx2 = h_deriv[4];
  hy2 = h_deriv[5];

  kappa = sens*(hx1*hy2-hx2*hy1)/pow(hx1*hx1+hy1*hy1,1.5);

  /* computation of phi and L and their derivatives */
  result = flatPhiAndLDeriv(flatPt, kappa, tabPhi, tabL);

  *ddelta_v = deltaPrime_v(flatPt, h_deriv, sens, tabPhi, tabL);
  *ddelta_w = deltaPrime_w(flatPt, h_deriv, sens, tabPhi, tabL);

  return result;
}

/*
 *   deltaPrime_over --        
 *
 *        derivatives of delta for the centered trailer connection
 *
 */

void deltaPrime_over(FLAT_STR *flatPt,
		     FLAT_LOCAL_PATH_STR *pathPt, 
		     double u, 
		     double *deltaPrime_v, 
		     double *deltaPrime_w)
{
  double deriv[12];
  double timeScaling;
  int sens;
  TR_FLAT_CONFIG_STR * q_init, * q_fin;
  double v2, alpha_0, alpha_1, velCoef;
  double hx_p, hy_p, hx_s, hy_s, hx_t, hy_t, hx_q, hy_q, hx_5, hy_5;
  double t1, t2, t3, t4, t5, t6, t9, t11, t13, t14, t15, 
    t17, t20, 
    t22, t23, t24, t25, t31, t33, t34, t36, 
    t38, t41, t42, t43, 
    t46, t49, t53, t55, t57, t60,
    t68, t71, t73, t77, t79, t84, t95, 
    t103, t104, t105, t107, t108, t109, t111, 
    t112, t115, t116, t119, t120, t122;

  double l = flatPt->distAxleToAxis.l2;
  
  q_init = &(pathPt->initFlatConf);
  q_fin = &(pathPt->finalFlatConf);
  v2 = pathPt->v2;
  velCoef = pathPt->velCoeff;
  alpha_0 = pathPt->alpha_0;
  alpha_1 = pathPt->alpha_1;
  sens = (v2 > 0) ? 1 : -1;
  timeScaling = velCoef;

  flatTrailerCombination(q_init, q_fin, u, v2, alpha_0, alpha_1, 5, deriv);

  hx_p = deriv[2]/timeScaling;
  hy_p = deriv[3]/timeScaling;
  timeScaling *= velCoef;

  hx_s = deriv[4]/timeScaling;
  hy_s = deriv[5]/timeScaling;
  timeScaling *= velCoef;

  hx_t = deriv[6]/timeScaling;
  hy_t = deriv[7]/timeScaling;
  timeScaling *= velCoef;
  hx_q = deriv[8]/timeScaling;
  hy_q = deriv[9]/timeScaling;
  timeScaling *= velCoef;

  hx_5 = deriv[10]/timeScaling;
  hy_5 = deriv[11]/timeScaling;

  t1 = hx_p;
  t2 = t1*t1;
  t3 = hy_p;
  t4 = t3*t3;
  t5 = t2+t4;
  t6 = l*l;
  t9 = hy_s;
  t11 = hx_s;
  t13 = t1*t9-t3*t11;
  t14 = t13*t13;
  t15 = t5*t5;
  t17 = 1/t15/t5;
  t20 = 1.0+t6*t14*t17;
  t22 = sqrt(t5*t20);
  t23 = 1/t15;
  t24 = t20*t20;
  t25 = 1/t24;
  t31 = 2.0*t1*t11+2.0*t3*t9;
  t33 = t13*t17;
  t34 = hy_t;
  t36 = hx_t;
  t38 = t1*t34-t3*t36;
  t41 = t15*t15;
  t42 = 1/t41;
  t43 = t14*t42;
  t46 = 2.0*t6*t33*t38-3.0*t6*t43*t31;
  t49 = pow(t31*t20+t5*t46,2.0);
  t53 = t11*t11;
  t55 = t9*t9;
  t57 = 2.0*t53+2.0*t1*t36+2.0*t55+2.0*t3*t34;
  t60 = t38*t38;
  t68 = hy_q;
  t71 = hx_q;
  t73 = t11*t34+t1*t68-t9*t36-t3*t71;
  t77 = 1/t41/t5;
  t79 = t31*t31;
  t84 = 2.0*t6*t60*t17-12.0*t6*t13*t42*t38*t31+2.0*t6*t33*t73+12.0*t6*t14*
    t77*t79-3.0*t6*t43*t57;
  t95 = t13;
  t103 = sens*t38;
  t104 = sqrt(t5);
  t105 = t104*t23;
  t107 = sens*t13;
  t108 = t104*t17;
  t109 = t108*t31;
  t111 = t103*t105-3.0/2.0*t107*t109;
  t112 = t46*t46;
  t115 = l*t25;
  t116 = sens*t73;
  t119 = t104*t42;
  t120 = t119*t79;
  t122 = t108*t57;
  *deltaPrime_v = -sens*t22*t23*t25*t49/4+sens/t22*(t57*t20+2.0*t31*t46+t5*
						   t84)/2;
  *deltaPrime_w = t73/t5-2.0*t38*t23*t31+2.0*t95*t17*t79-t95*t23*t57+
    2.0*l/t24/t20*t111*t112-2.0*t115*(t116*t105-3.0*t103*t109+15.0/4.0*t107*t120
				      -3.0/2.0*t107*t122)*t46-t115*t111*t84+l/t20*(sens*(2.0*t11*t68+t1*hy_5-2.0*
											 t9*t71-t3*hx_5)*t105-9.0/2.0*t116*t109+45.0/4.0*t103*t120-9.0/2.0*t103*t122
										   -105.0/8.0*t107*t104*t77*t79*t31+45.0/4.0*t107*t119*t31*t57-3.0/2.0*t107*t108*(
																				  6.0*t11*t36+2.0*t1*t71+6.0*t9*t34+2.0*t3*t68));
  
}

/*
 *   deltaPrime --        
 *
 *        derivatives of delta for bothh trailer connections.
 *
 */

int flatTrailerDeltaPrime(FLAT_STR *flatPt,
			  FLAT_LOCAL_PATH_STR *pathPt, 
			  double localParam, 
			  double *ddelta_v, 
			  double *ddelta_w)
{
  double l1 = flatPt->distAxleToAxis.l1;
  int result = TRUE;

  if (l1 == 0){
    deltaPrime_over(flatPt, pathPt, localParam, ddelta_v, ddelta_w);
  }
  else{
    result = deltaPrime_off(flatPt, pathPt, localParam, ddelta_v, ddelta_w);
  }
  return result;
}


/*
 *  accelCoefficient_on --
 *
 *      Compute the velocities and accelerations of the robot wrt
 *      pseudo-velocity and accelerations for the over-axis trailer
 *      connection.
 *                                                                        
 *      v = d_v udot,  vdot = d_v udotdot + delta_v udot^2                
 *      w = d_w udot,  wdot = d_w udotdot + delta_w udot^2                
 *
 *      
 *      The coefficients are stored in an array on the following order: 
 *      (d_v, delta_v, d_w, delta_w)
 *                                                                        
 */

static void accelCoefficient_on(FLAT_STR *flatPt,
				FLAT_LOCAL_PATH_STR *pathPt, 
				double u, double *coeff)
{
  double gammaDeriv[12];
  double timeScaling;
  int sens;
  TR_FLAT_CONFIG_STR * q_init, * q_fin;
  double v2, alpha_0, alpha_1, velCoef;
  double t1, t2, t3, t4, t5, t6, t9, t11, t13, t14, t15, t17, t20, t22, 
    t28, t31, t33, t35, t38, t39, t43, t47, t48, t51, t52, t53, t54, t55, 
    t57, t58, t59, t61, t68, t84, t87, t89;

  double l = flatPt->distAxleToAxis.l2;
  
  q_init = &(pathPt->initFlatConf);
  q_fin = &(pathPt->finalFlatConf);
  v2 = pathPt->v2;
  velCoef = pathPt->velCoeff;
  alpha_0 = pathPt->alpha_0;
  alpha_1 = pathPt->alpha_1;
  sens = (v2 > 0) ? 1 : -1;
  timeScaling = velCoef;

  flatTrailerCombination(q_init, q_fin, u, v2, alpha_0, alpha_1, 5, 
			 gammaDeriv);

  
  /* calculs des coefficients dv, dw, delta_v, delta_w donnant vitesses et 
     accelerations en fonction de du/dt et d2u/dt2 
     Ces calculs ont ete realises par Maple, ils se trouvent dans le fichier
     maple/calculs.map*/
  t1 = gammaDeriv[2]/timeScaling;
  t2 = t1*t1;
  t3 = gammaDeriv[3]/timeScaling;
  timeScaling*=velCoef;
  t4 = t3*t3;
  t5 = t2+t4;
  t6 = l*l;
  t9 = gammaDeriv[5]/timeScaling;
  t11 = gammaDeriv[4]/timeScaling;
  timeScaling*=velCoef;
  t13 = t1*t9-t3*t11;
  t14 = t13*t13;
  t15 = t5*t5;
  t17 = 1/t15/t5;
  t20 = 1.0+t6*t14*t17;
  t22 = sqrt(t5*t20);
  t28 = 2.0*t1*t11+2.0*t3*t9;
  t31 = gammaDeriv[7]/timeScaling;
  t33 = gammaDeriv[6]/timeScaling;
  timeScaling*=velCoef;
  t35 = t1*t31-t3*t33;
  t38 = t15*t15;
  t39 = 1/t38;
  t43 = 2.0*t6*t13*t17*t35-3.0*t6*t14*t39*t28;
  t47 = t13;
  t48 = 1/t5;
  t51 = l/t20;
  t52 = sens*t35;
  t53 = sqrt(t5);
  t54 = 1/t15;
  t55 = t53*t54;
  t57 = sens*t13;
  t58 = t53*t17;
  t59 = t58*t28;
  t61 = t52*t55-3.0/2.0*t57*t59;
  t68 = t20*t20;
  t84 = t28*t28;
  t87 = t11*t11;
  t89 = t9*t9;
  coeff[0] = sens*t22;
  coeff[1] = sens/t22*(t28*t20+t5*t43)/2;
  coeff[2] = t47*t48+t51*t61;
  coeff[3] = t35*t48-t47*t54*t28-l/t68*t61*t43+t51*(sens*(t11*t31+t1*
							    gammaDeriv[9]/timeScaling-t9*t33-t3*gammaDeriv[8]/timeScaling)*t55-3.0*t52*t59+15.0/4.0*t57*t53*t39*t84-3.0/2.0*t57
						      *t58*(2.0*t87+2.0*t1*t33+2.0*t89+2.0*t3*t31));
}

/*
 *  accelCoefficient --
 *
 *      Compute the velocities and accelerations of the robot wrt
 *      pseudo-velocity and accelerations for both trailer connections.
 *                                                                        
 *      v = d_v udot,  vdot = d_v udotdot + delta_v udot^2                
 *      w = d_w udot,  wdot = d_w udotdot + delta_w udot^2                
 *
 *      
 *      The coefficients are stored in an array on the following order: 
 *      (d_v, delta_v, d_w, delta_w)
 *                                                                        
 */

int flatTrailerAccelCoefficient(FLAT_STR *flatPt,
				FLAT_LOCAL_PATH_STR *pathPt, 
				double localParam, 
				double *coeff)
{
  double velCoef = pathPt->velCoeff;
  double timeScaling = pathPt->velCoeff;
  double h_deriv[10];
  double tabPhi[4];
  double tabL[4];
  double hx1;
  double hy1;
  double hx2;
  double hy2;
  double kappa;
  double l1 = flatPt->distAxleToAxis.l1;
  int result = TRUE;

  int sens = pathPt->v2 > 0 ? 1 : -1;

  /* calcul des derivees de la courbe des sorties plates */
  flatTrailerCombination(&(pathPt->initFlatConf),
			 &(pathPt->finalFlatConf),
			 localParam, pathPt->v2,
			 pathPt->alpha_0, pathPt->alpha_1,
			 4, h_deriv);

  /* transformation des derivees par rapport au parametre local
     en derivees par rapport au parametre global */

  h_deriv[2] /= timeScaling;
  h_deriv[3] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[4] /= timeScaling;
  h_deriv[5] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[6] /= timeScaling;
  h_deriv[7] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[8] /= timeScaling;
  h_deriv[9] /= timeScaling;

  if (fabs(h_deriv[2]) < 1e-10)
    h_deriv[2] = SIGN(h_deriv[2])*1e-10;

  hx1 = h_deriv[2];
  hy1 = h_deriv[3];
  hx2 = h_deriv[4];
  hy2 = h_deriv[5];

  kappa = sens*(hx1*hy2-hx2*hy1)/pow(hx1*hx1+hy1*hy1,1.5);

  if (l1 != 0){
    /* computation of phi and L and their derivatives */
    result = flatPhiAndLDeriv(flatPt, kappa, tabPhi, tabL);
    
    coeff[0] = coef_dv(flatPt, h_deriv, sens, tabPhi, tabL);
    coeff[1] = coef_delta_v(flatPt, h_deriv, sens, tabPhi, tabL);
    coeff[2] = coef_dw(flatPt, h_deriv,sens, tabPhi, tabL);
    coeff[3] = coef_delta_w(flatPt, h_deriv, sens, tabPhi, tabL);
  }
  else {
    accelCoefficient_on(flatPt, pathPt, localParam, coeff);
  }
  return result;
}

/**************************************************************************
 *  
 *      Fonctions donnant les vitesses et accelerations du robot en       
 *      fonction des pseudo-vitesse et accelerations                      
 *                                                                        
 *      v = d_v udot,  vdot = d_v udotdot + delta_v udot^2                
 *      w = d_w udot,  wdot = d_w udotdot + delta_w udot^2                
 *
 *      Les resultats sont places dans un tableau de 4 doubles dans l'ordre    
 *       suivant : (d_v, delta_v, d_w, delta_w)
 *                                                                        
 ************************************************************************/

void flatTrailerAccelCoefficientTrailer(FLAT_STR *flatPt,
					FLAT_LOCAL_PATH_STR *pathPt, 
					double localParam, 
					double *coeff)
{
  double velCoef = pathPt->velCoeff;
  double timeScaling = pathPt->velCoeff;
  double h_deriv[10];
  double tabPhi[4];
  double tabL[4];
  double hx1;
  double hy1;
  double hx2;
  double hy2;
  double hx3;
  double hy3;
  double kappa;

  double l1 = flatPt->distAxleToAxis.l1;  
  
  int sens = pathPt->v2 > 0 ? 1 : -1;
  
  /* calcul des derivees de la courbe des sorties plates */
  flatTrailerCombination(&(pathPt->initFlatConf),
			 &(pathPt->finalFlatConf),
			 localParam, pathPt->v2,
			 pathPt->alpha_0, pathPt->alpha_1,
			 4, h_deriv);
  
  /* transformation des derivees par rapport au parametre local
     en derivees par rapport au parametre global */
  
  h_deriv[2] /= timeScaling;
  h_deriv[3] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[4] /= timeScaling;
  h_deriv[5] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[6] /= timeScaling;
  h_deriv[7] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[8] /= timeScaling;
  h_deriv[9] /= timeScaling;

  if (fabs(h_deriv[2]) < 1e-10)
    h_deriv[2] = SIGN(h_deriv[2])*1e-10;

  hx1 = h_deriv[2];
  hy1 = h_deriv[3];
  hx2 = h_deriv[4];
  hy2 = h_deriv[5];
  hx3 = h_deriv[6];
  hy3 = h_deriv[7];

  kappa = sens*(hx1*hy2-hx2*hy1)/pow(hx1*hx1+hy1*hy1,1.5);
  if (l1 != 0 ){
    /* computation of phi and L and their derivatives */
    flatPhiAndLDeriv(flatPt, kappa, tabPhi, tabL);
    
    coeff[0] = coef_dv_trailer(flatPt, h_deriv, sens, tabPhi, tabL);
    coeff[1] = coef_delta_v_trailer(flatPt, h_deriv, sens, tabPhi, tabL);
    coeff[2] = coef_dw_trailer(flatPt, h_deriv,sens, tabPhi, tabL);
    coeff[3] = coef_delta_w_trailer(flatPt, h_deriv, sens, tabPhi, tabL);
  }
  else {
    coeff[0] = sqrt(hx1*hx1 + hy1*hy1);
    coeff[1] = sqrt(hx2*hx2 + hy2*hy2);
    coeff[2] = fabs((hx1*hy2 - hy1*hx2)/(hx1*hx1 + hy1*hy1));
    coeff[3] =  fabs((hx1*hy3 - hy1*hx3)/(hx1*hx1 + hy1*hy1)
		     - 2*(hx1*hy2 - hy1*hx2)*(hx1*hx2 + hy1*hy2)
		     /pow((hx1*hx1 + hy1*hy1),2.));
  }
}

#if(0)

/*----------------------------------------------------------------------
 *
 *   diff_phi4 --  Derivee 4eme de la fonction phi(curv)
 *
 *      Le maximum de cette fonction permet de majorer l'erreur sur 
 *      l'interpolation
 *
 *--------------------------------------------------------------------*/

double phi_diff4(FLAT_STR *flatPt,
		 double phi)
{

  double t1, t2, t3, t4, t5, t7, t8, t10, t11, t12, t13, t14, t16,
    t17, t18, t20, t21, t23, t24, t25, t27, t28, t30, t32, t33, t34, t35,
    t37, t41, t43, t44, t47, t48, t49, t52, t53, t55, t56, t57, t60, t62,
    t63, t67, t68, t69, t70, t72, t73, t75, t77, t78, t81, t85, t86, t87,
    t89, t91, t92, t94, t96, t97, t99, t100, t101, t102, t109, t112, t114,
    t115, t117, t119, t120, t121, t131, t132, t133, t134, t136, t137,
    t138, t140, t141, t142, t147, t149, t157, t158, t159, t163, t172,
    t174, t178, t182, t188, t192, t200, t202, t204, t206, t207, t210,
    t212, t220, t248, t271, t272, t291, t319, t322;

  double a = flatPt->distAxleToAxis.l1;
  double b = flatPt->distAxleToAxis.l2;

  t1 = phi;
  t2 = cos(t1);
  t3 = a*a;
  t4 = b*b;
  t5 = a*b;
  t7 = t3+t4+2.0*t5*t2;
  t8 = sqrt(t7);
  t10 = flatEllipticIntegral(flatPt, t1);
  t11 = sin(t1);
  t12 = t10*t11;
  t13 = t2*t8+t12;
  t14 = 1/t13;
  t16 = t13*t13;
  t17 = 1/t16;
  t18 = t11*t17;
  t20 = t10*t2;
  t21 = -t11*t8+t20;
  t23 = -t2*t14+t18*t21;
  t24 = t23*t23;
  t25 = t24*t24;
  t27 = 1/t23;
  t28 = t11*t27;
  t30 = t2*t17;
  t32 = 1/t8;
  t33 = t2*t32;
  t34 = t33*a;
  t35 = b*t11;
  t37 = t34*t35*t27;
  t41 = -t28*t8-t37+t5*t33*t11+t20*t27;
  t43 = t2*t27;
  t44 = t17*t21;
  t47 = 1/t16/t13;
  t48 = t11*t47;
  t49 = t21*t41;
  t52 = t11*t11;
  t53 = t52*t32;
  t55 = t53*t5*t27;
  t56 = t2*t2;
  t57 = t56*t32;
  t60 = -t43*t8+t55+t5*t57-t12*t27;
  t62 = t28*t14+t30*t41+t43*t44-2.0*t48*t49+t18*t60;
  t63 = t62*t62;
  t67 = 1/t24/t23;
  t68 = t67*t62;
  t69 = 1/t24;
  t70 = t2*t69;
  t72 = t11*t69;
  t73 = t14*t62;
  t75 = t17*t41;
  t77 = t2*t47;
  t78 = t41*t41;
  t81 = t8*t62;
  t85 = t32*a*b;
  t86 = t52*t69*t85;
  t87 = t7*t7;
  t89 = t8/t87;
  t91 = t2*t89*t3;
  t92 = t4*t52;
  t94 = t91*t92*t69;
  t96 = t57*t5*t69;
  t97 = t69*t62;
  t99 = t34*t35*t97;
  t100 = t3*t4;
  t101 = t100*t2;
  t102 = t89*t52;
  t109 = -t70*t8+t72*t81+2.0*t86-t94-t96+t99-t55+t101*t102*t27+2.0*t5*t57*
    t27-t12*t69-t20*t97;
  t112 = t44*t62;
  t114 = t47*t21;
  t115 = t114*t41;
  t117 = t17*t60;
  t119 = t16*t16;
  t120 = 1/t119;
  t121 = t11*t120;
  t131 = t5*t11;
  t132 = t70*t32*t131;
  t133 = t52*t11;
  t134 = t133*t89;
  t136 = t134*t100*t69;
  t137 = t53*a;
  t138 = b*t69;
  t140 = t137*t138*t62;
  t141 = t100*t56;
  t142 = t89*t11;
  t147 = t72*t8+t70*t81+3.0*t132+t136-t140-3.0*t37+t141*t142*t27-t20*t69+t12*t97;
  t149 = t70*t14-t72*t73-2.0*t28*t75-2.0*t77*t78+t30*t109-t72*t44-t70*t112-4.0*t43*t115+2.0*t43*t117+6.0*t121*t21*t78-4.0*t48*t60*t41-2.0*t48*t21*t109+t18*t147;
  t157 = t3*a;
  t158 = t4*b;
  t159 = t157*t158;
  t163 = t8/t87/t7;
  t172 = t67*t63;
  t174 = t69*t149;
  t178 = t52*t52;
  t182 = b*t67;
  t188 = t2*t67;
  t192 = 3.0*t159*t56*t163*t52*t69+t100*t56*t2*t89*t69+3.0*t20*t68-2.0*t12*t172+t12*t174-t141*t142*t97+3.0*t178*t163*t159*t67+2.0*t137*t182*t63-t137*t138*t149+t12*t67+6.0*t188*t89*t100*t52;
  t200 = t52*t67;
  t202 = t8*t63;
  t204 = t8*t149;
  t206 = t188*t32;
  t207 = t62*a;
  t210 = t11*t67;
  t212 = 3.0*t56*t67*t85-3.0*t134*t3*t4*t67*t62+t188*t8-4.0*t200*t85-2.0*t188*t202+t70*t204-9.0*t206*t207*t35-3.0*t210*t81+4.0*t99-5.0*t94-4.0*t96+3.0*t86;
  t220 = t78*t41;
  t248 = -3.0*t5*t56*t32*t69*t62-t20*t67+2.0*t20*t172-t20*t174-t101*t102*t97+3.0*t159*t2*t163*t133*t69+4.0*t141*t142*t69-2.0*t34*t35*t172+t34*t35*t174+3.0*t12*t68+3.0*t57*a*t182*t62+3.0*t91*t92*t68;
  t271 = -3.0*t2*t163*t157*t158*t133*t67-3.0*t56*t89*t3*t4*t11*t67+3.0*t133*t67*t89*t3*t4+3.0*t188*t81+7.0*t206*t131-2.0*t210*t202+t72*t204-6.0*t200*t32*t207*b+t210*t8-7.0*t132+t140-2.0*t136;
  t272 = t248+t271;
  t291 = -6.0*t48*t147*t41-6.0*t48*t60*t109+18.0*t121*t49*t109+t18*(t192+t212)+18.0*t121*t60*t78-24.0*t11/t119/t13*t21*t220-2.0*t48*t21*t272+3.0*t43*t17*t147-6.0*t77*t41*t109-3.0*t188*t73-3.0*t70*t75+6.0*t28*t47*t78-6.0*t43*t114*t109-12.0*t43*t47*t60*t41+18.0*t43*t120*t21*t78;
  t319 = -t70*t44*t149-3.0*t70*t117*t62+6.0*t70*t47*t21*t62*t41+2.0*t188*t44*t63-t188*t44+6.0*t72*t115-3.0*t72*t117+3.0*t210*t112+t30*t272+6.0*t2*t120*t220-3.0*t28*t17*t109-t72*t14*t149+3.0*t72*t17*t62*t41+2.0*t210*t14*t63-t210*t14;
  t322 = -6.0/t25*t63*t62+6.0*t68*t149-t69*(t291+t319);

  return t322;
}

#endif


/*
 *
 *          Kinematic coefficients for hilare without trailer
 *
 */



static double coef_dv_hilare(double *h_deriv, int sens)
{
  double t2, t4, t6, dv;

  t2 = pow(h_deriv[2],2.0);
  t4 = pow(h_deriv[3],2.0);
  t6 = sqrt(t2+t4);
  dv = sens*t6;

  return dv;
}

static double coef_delta_v_hilare(double *h_deriv, int sens)
{
  double t1, t2, t3, t4, t6, delta_v;

  t1 = h_deriv[2];
  t2 = t1*t1;
  t3 = h_deriv[3];
  t4 = t3*t3;
  t6 = sqrt(t2+t4);
  delta_v = sens/t6*(2.0*t1*h_deriv[4]+2.0*t3*h_deriv[5])/2;

  return delta_v;
}

static double coef_dw_hilare(double *h_deriv,int sens)
{
  double t1, t4, t8, t9, dw;

  t1 = h_deriv[2];
  t4 = h_deriv[3];
  t8 = t1*t1;
  t9 = t4*t4;
  dw = (t1*h_deriv[5]-t4*h_deriv[4])/(t8+t9);

  return dw;
}

static double coef_delta_w_hilare(double *h_deriv, int sens)
{
  double t1, t4, t8, t9, t10, t13, t15, t18, delta_w;

  t1 = h_deriv[2];
  t4 = h_deriv[3];
  t8 = t1*t1;
  t9 = t4*t4;
  t10 = t8+t9;
  t13 = h_deriv[5];
  t15 = h_deriv[4];
  t18 = t10*t10;
  delta_w = (t1*h_deriv[7]-t4*h_deriv[6])/t10-
    (t1*t13-t4*t15)/t18*(2.0*t1*t15+2.0*t4*t13);
  
  return delta_w;
}

static double coef_ddelta_v_hilare(double *h_deriv, int sens)
{
  double t1, t2, t3, t4, t5, t6, t10, t12, t15, t19, t22, ddelta_v;

  t1 = h_deriv[2];
  t2 = t1*t1;
  t3 = h_deriv[3];
  t4 = t3*t3;
  t5 = t2+t4;
  t6 = sqrt(t5);
  t10 = h_deriv[4];
  t12 = h_deriv[5];
  t15 = pow(2.0*t1*t10+2.0*t3*t12,2.0);
  t19 = t10*t10;
  t22 = t12*t12;
  ddelta_v = -sens/t6/t5*t15/4+sens/t6*
    (2.0*t19+2.0*t1*h_deriv[6]+2.0*t22+2.0*t3*h_deriv[7])/2;
  
  return ddelta_v;
}

static double coef_ddelta_w_hilare(double *h_deriv, int sens)
{
  double t1, t2, t4, t7, t8, t10, t14, t15, t16, t22, t23, t27, t31, t35, 
    t38, t40, ddelta_w;
  
  t1 = h_deriv[4];
  t2 = h_deriv[7];
  t4 = h_deriv[2];
  t7 = h_deriv[5];
  t8 = h_deriv[6];
  t10 = h_deriv[3];
  t14 = t4*t4;
  t15 = t10*t10;
  t16 = t14+t15;
  t22 = t16*t16;
  t23 = 1/t22;
  t27 = 2.0*t4*t1+2.0*t10*t7;
  t31 = t4*t7-t1*t10;
  t35 = t27*t27;
  t38 = t1*t1;
  t40 = t7*t7;
  ddelta_w = (t1*t2+t4*h_deriv[9]-t7*t8-t10*h_deriv[8])/t16-2.0*(t4*t2-t10*t8)*
    t23*t27+2.0*t31/t22/t16*t35-t31*t23*
    (2.0*t38+2.0*t4*t8+2.0*t40+2.0*t10*t2);

  return ddelta_w;
}



/*
 *  flatHilareAccelCoefficient --
 *
 *      Compute the velocities and accelerations of the robot wrt
 *      pseudo-velocity and accelerations for Hilare without trailer
 *                                                                        
 *      v = d_v udot,  vdot = d_v udotdot + delta_v udot^2                
 *      w = d_w udot,  wdot = d_w udotdot + delta_w udot^2                
 *
 *      
 *      The coefficients are stored in an array on the following order: 
 *      (d_v, delta_v, d_w, delta_w)
 *                                                                        
 */

int flatHilareAccelCoefficient(FLAT_LOCAL_PATH_STR *pathPt, 
			       double localParam, 
			       double *coeff)
{
  double velCoef = pathPt->velCoeff;
  double timeScaling = pathPt->velCoeff;
  double h_deriv[8];
  int result = TRUE;

  int sens = pathPt->v2 > 0 ? 1 : -1;

  /* calcul des derivees de la courbe des sorties plates */
  flatHilareCombination(&(pathPt->initFlatConf),
			&(pathPt->finalFlatConf),
			localParam, pathPt->v2,
			3, h_deriv);

  /* transformation des derivees par rapport au parametre local
     en derivees par rapport au parametre global */
  
  h_deriv[2] /= timeScaling;
  h_deriv[3] /= timeScaling;
  timeScaling *= velCoef;
  
  h_deriv[4] /= timeScaling;
  h_deriv[5] /= timeScaling;
  timeScaling *= velCoef;
  
  h_deriv[6] /= timeScaling;
  h_deriv[7] /= timeScaling;
  timeScaling *= velCoef;
  
  if (fabs(h_deriv[2]) < 1e-10)
    h_deriv[2] = SIGN(h_deriv[2])*1e-10;

  coeff[0] = coef_dv_hilare(h_deriv, sens);
  coeff[1] = coef_delta_v_hilare(h_deriv, sens);
  coeff[2] = coef_dw_hilare(h_deriv,sens);
  coeff[3] = coef_delta_w_hilare(h_deriv, sens);

  return result;
}


int flatHilareDeltaPrime(FLAT_LOCAL_PATH_STR *pathPt, 
			 double localParam, 
			 double *ddelta_v, double *ddelta_w)
{
  double velCoef = pathPt->velCoeff;
  double timeScaling = pathPt->velCoeff;
  double h_deriv[10];
  int sens;
  int result = TRUE;

  sens = pathPt->v2 > 0 ? 1 : -1;

  /* calcul des derivees de la courbe des sorties plates */
  flatHilareCombination(&(pathPt->initFlatConf),
			&(pathPt->finalFlatConf),
			localParam, pathPt->v2,
			4, h_deriv);

  /* transformation des derivees par rapport au parametre local
     en derivees par rapport au parametre global */

  h_deriv[2] /= timeScaling;
  h_deriv[3] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[4] /= timeScaling;
  h_deriv[5] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[6] /= timeScaling;
  h_deriv[7] /= timeScaling;
  timeScaling *= velCoef;

  h_deriv[8] /= timeScaling;
  h_deriv[9] /= timeScaling;

  timeScaling *= velCoef;

  *ddelta_v = coef_ddelta_v_hilare(h_deriv, sens);
  *ddelta_w = coef_ddelta_w_hilare(h_deriv, sens);

  return result;
}

