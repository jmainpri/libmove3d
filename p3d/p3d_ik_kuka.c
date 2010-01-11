#include "P3d-pkg.h"

double coseta4s(double x, double y, double z, double d3, double d5) {
  return((x * x + y * y + z * z - d3 * d3 - d5 * d5) / d5 / d3 / 0.2e1);
}

double g1p(double d3, double d5, double eta2, double eta3, double eta4) {
  double tmp = sin(eta2);
  return((-cos(eta2) * sin(eta3) * sin(eta4) - tmp  * cos(eta4)) * d5 - tmp * d3);
}

double g2p(double d3, double d5, double eta2, double eta3, double eta4) {
  return(cos(eta3) * sin(eta4) * d5);
}

double g3p(double d3, double d5, double eta2, double eta3, double eta4) {
  double tmp = cos(eta2);
  return(-d5 * sin(eta2) * sin(eta3) * sin(eta4) + cos(eta4) * tmp * d5 + tmp * d3);
}

double xWrist(double d7, double t13, double t14) {
  return(t14);
}

double yWrist(double d7, double t23, double t24) {
  return(t24);
}

double zWrist(double d7, double t33, double t34) {
  return(t34);
}

/*left arm = -1, for right arm = 1*/
double denomEta2Solve(double z, double d3, double d5, double eta3, double eta4, int arm) {
  return(d5 * cos(eta4) + arm * z + d3);
}

double numerEta2Solve1(double z, double d3, double d5, double eta3, double eta4) {
  double tmp1 = cos(eta3), tmp2 = cos(eta4);
  double tmp3 = sqrt(d5 * d5 * (1 + tmp1 * tmp1 * (tmp2 * tmp2 - 1)) - z * z + d3 * (d3 + 0.2e1 * d5 * tmp2));
  return(-sin(eta4) * sin(eta3) * d5 + tmp3);
}

double numerEta2Solve2(double z, double d3, double d5, double eta3, double eta4) {
  double tmp1 = cos(eta3), tmp2 = cos(eta4);
  double tmp3 = sqrt(d5 * d5 * (1 + tmp1 * tmp1 * (tmp2 * tmp2 - 1)) - z * z + d3 * (d3 + 0.2e1 * d5 * tmp2));
  return(-sin(eta4) * sin(eta3) * d5 - tmp3);
}

/*left arm = -1, for right arm = 1*/
double r11eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = cos(eta4), t2 = cos(eta1), t3 = t2 * t1, t4 = cos(eta2), t5 = sin(eta3);
  double t6 = t4 * t5, t8 = sin(eta1), t9 = t1 * t8, t10 = cos(eta3), t12 = sin(eta2);
  double t14 = sin(eta4);
  return((t3 * t6 + t9 * t10 - t2 * t12 * t14) * r11 + (t9 * t6 - t3 * t10 - t8 * t12 * t14) * r21 * arm + (t12 * t5 * t1 + t4 * t14) * r31 * arm);
}

/*left arm = -1, for right arm = 1*/
double r12eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = cos(eta4), t2 = cos(eta1), t3 = t1 * t2, t4 = cos(eta2), t5 = sin(eta3);
  double t6 = t4 * t5, t8 = sin(eta1), t9 = t1 * t8, t10 = cos(eta3), t12 = sin(eta2);
  double t14 = sin(eta4);
  return((t3 * t6 + t10 * t9 - t2 * t12 * t14) * r12 + (t9 * t6 - t3 * t10 - t8 * t12 * t14) * r22 * arm + (t12 * t5 * t1 + t4 * t14) * r32 * arm);
}

/*left arm = -1, for right arm = 1*/
double r13eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = cos(eta4), t2 = cos(eta1), t3 = t2 * t1, t4 = cos(eta2), t5 = sin(eta3);
  double t6 = t4 * t5, t8 = sin(eta1), t9 = t1 * t8, t10 = cos(eta3), t12 = sin(eta2);
  double t14 = sin(eta4);
  return((t3 * t6 + t10 * t9 - t2 * t12 * t14) * r13 + (t9 * t6 - t10 * t3 - t8 * t12 * t14) * r23 * arm + (t12 * t5 * t1 + t4 * t14) * r33 * arm);
}

/*left arm = -1, for right arm = 1*/
double r21eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = sin(eta4), t2 = cos(eta1), t3 = t2 * t1, t4 = cos(eta2), t5 = sin(eta3);
  double t6 = t4 * t5, t8 = sin(eta1), t9 = t1 * t8, t10 = cos(eta3), t12 = sin(eta2);
  double t14 = cos(eta4);
  return((-t3 * t6 - t9 * t10 - t2 * t12 * t14) * r11 + (-t9 * t6 + t3 * t10 - t8 * t12 * t14) * r21 * arm + (-t12 * t5 * t1 + t4 * t14) * r31 * arm);
}

/*left arm = -1, for right arm = 1*/
double r22eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = sin(eta4), t2 = cos(eta1), t3 = t2 * t1, t4 = cos(eta2), t5 = sin(eta3);
  double t6 = t4 * t5, t8 = sin(eta1), t9 = t8 * t1, t10 = cos(eta3), t12 = sin(eta2);
  double t14 = cos(eta4);
  return((-t3 * t6 - t9 * t10 - t2 * t12 * t14) * r12 + (-t9 * t6 + t3 * t10 - t8 * t12 * t14) * r22 * arm + (-t12 * t5 * t1 + t4 * t14) * r32 * arm);
}

/*left arm = -1, for right arm = 1*/
double r23eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = sin(eta4), t2 = cos(eta1), t3 = t1 * t2, t4 = cos(eta2), t5 = sin(eta3);
  double t6 = t4 * t5, t8 = sin(eta1), t9 = t1 * t8, t10 = cos(eta3), t12 = sin(eta2);
  double t14 = cos(eta4);
  return((-t3 * t6 - t9 * t10 - t2 * t12 * t14) * r13 + (-t9 * t6 + t10 * t3 - t8 * t12 * t14) * r23 * arm + (-t12 * t5 * t1 + t4 * t14) * r33 * arm);
}

/*left arm = -1, for right arm = 1*/
double r31eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = cos(eta1), t2 = cos(eta2), t4 = cos(eta3), t6 = sin(eta1), t7 = sin(eta3);
  double t16 = sin(eta2);
  return((-t1 * t2 * t4 + t7 * t6) * r11 + (-t6 * t2 * t4 - t1 * t7) * r21 * arm - t16 * t4 * r31 * arm);
}

/*left arm = -1, for right arm = 1*/
double r32eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = cos(eta1), t2 = cos(eta2), t4 = cos(eta3), t6 = sin(eta1), t7 = sin(eta3);
  double t16 = sin(eta2);
  return((-t1 * t2 * t4 + t6 * t7) * r12 + (-t6 * t2 * t4 - t7 * t1) * r22 * arm - t16 * t4 * r32 * arm);
}

/*left arm = -1, for right arm = 1*/
double r33eval(double r11, double r12, double r13,
               double r21, double r22, double r23,
               double r31, double r32, double r33,
               double eta1, double eta2, double eta3,
               double eta4, int arm) {
  double t1 = cos(eta1), t2 = cos(eta2), t4 = cos(eta3), t6 = sin(eta1), t7 = sin(eta3);
  double t16 = sin(eta2);
  return((-t1 * t2 * t4 + t7 * t6) * r13 + (-t6 * t2 * t4 - t1 * t7) * r23 * arm - t16 * t4 * r33 * arm);
}

double eta6SolutionA(double r13, double r23, double r33) {
  double tmp = sqrt(r13 * r13 + r33 * r33);
  return(atan2(r23, tmp));
}

double eta6SolutionB(double r13, double r23, double r33) {
  double tmp = sqrt(r13 * r13 + r33 * r33);
  return(atan2(r23, -tmp));
}

/*left arm = -1, for right arm = 1*/
double eta7Solution(double r21, double r22, double e6, int arm) {
  double tmp = 0.1e1 / cos(e6);
  return(atan2(r21 * tmp * arm, r22 * tmp * arm));
}

double eta5Solution(double r13, double r33, double e6) {
  double tmp = 0.1e1 / cos(e6);
  return(atan2(r33 * tmp, -r13 * tmp));
}

/*left arm = -1, for right arm = 1*/
/*solution = [1-8]*/
int ikKUKAArmSolverUnique(double fixedAngle, double aArray[7], double alphaArray[7],
                          double dArray[7], double thetaArray[7], double posArray[4][4],
                          double phiArray[7], int arm, int solution) {
  int phiValid = 1;
  double n1, r13, r23, r33, r21, r22, de, cosEta4, x, y, z, g1e, g2e;

  if(solution > 8 || solution < 1){
    return -1; //incorrect parameters
  }

  phiArray[2] = fixedAngle; // 2
  x = posArray[0][3];
  y = posArray[1][3];
  z = posArray[2][3];
  cosEta4 = coseta4s(x, y, z, dArray[2], dArray[4]);
  if (cosEta4 * cosEta4 <= 0.10001e1) {
    if (0.1e1 < cosEta4 * cosEta4) {
      cosEta4 = 0.1e1;
//       return -4;
    }
    phiArray[3] = solution < 5 ? acos(cosEta4) : -acos(cosEta4); // 3
  } else {
    phiValid = 0;
    return 0;
  }

  //1
  if (solution < 5){
    de = denomEta2Solve(z, dArray[2], dArray[4], phiArray[2], phiArray[3], arm);
    if (de == 0.0e0) {
      phiArray[1] = 0.3141592654e1;
      return -2;
    } else {
      n1 = numerEta2Solve1(z, dArray[2], dArray[4], phiArray[2], phiArray[3]);
      if (isnan(n1) == 0) {
        if(solution == 1 || solution == 2){ //sol 1 or 2
          phiArray[1] = 0.2e1 * atan(n1 / de);
        }else{// sol 3 or 4
          phiArray[1] = 0.2e1 * atan(numerEta2Solve2(z, dArray[2], dArray[4], phiArray[2], phiArray[3]) / de);
        }
      } else {
        phiValid = 0;
        return 0;
      }
    }
  }else{
    de = denomEta2Solve(z, dArray[2], dArray[4], phiArray[2], phiArray[3], arm);
    if (de == 0.0e0) {
      phiArray[1] = 0.3141592654e1;
      return -2;
    } else {
      n1 = numerEta2Solve1(z, dArray[2], dArray[4], phiArray[2], phiArray[3]);
      if (isnan(n1) == 0) {
        if (solution == 5 || solution == 6){// sol 5 or 6
          phiArray[1] = 0.2e1 * atan(n1 / de);
        }else{ //sol 7 or 8
          phiArray[1] = 0.2e1 * atan(numerEta2Solve2(z, dArray[2], dArray[4], phiArray[2], phiArray[3]) / de);
        }
      } else {
        phiValid = 0;
        return 0;
      }
    }
  }
  if (x == 0.0e0 && y == 0.0e0) {
    phiArray[0] = 0.0e0; // 0
    return -3;
  }else{
    if (phiValid != 0){
      g1e = g1p(dArray[2], dArray[4], phiArray[1], phiArray[2], phiArray[3]);
      g2e = g2p(dArray[2], dArray[4], phiArray[1], phiArray[2], phiArray[3]);
      phiArray[0] = atan2(g1e * y * arm - g2e * x, g1e * x + g2e * y * arm); // 0 
      r13 = r13eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0], phiArray[1], phiArray[2], phiArray[3], arm);
      r23 = r23eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0], phiArray[1], phiArray[2], phiArray[3], arm);
      r33 = r33eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0], phiArray[1], phiArray[2], phiArray[3], arm);
      r21 = r21eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0], phiArray[1], phiArray[2], phiArray[3], arm);
      r22 = r22eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0], phiArray[1], phiArray[2], phiArray[3], arm);
      if (solution == 1 || solution == 3 || solution == 5 || solution == 7){// sol = 1 or 3 or 5 or 7
        phiArray[5] = eta6SolutionA(r13, r23, r33); // 5
      }else{// sol = 2 or 4 or 6 or 8
        phiArray[5] = eta6SolutionB(r13, r23, r33); // 5
      }
      phiArray[4] = eta5Solution(r13, r33, phiArray[5]); // 4
      phiArray[6] = eta7Solution(r21, r22, phiArray[5], arm); // 6
    }
  }
  return 1;
}

/*left arm = -1, for right arm = 1*/
int ikKUKAArmSolver(double fixedAngle, double aArray[7], double alphaArray[7],
                    double dArray[7], double thetaArray[7], double posArray[4][4],
                    double phiArray[8][7], int phiArrayValid[8], int arm) {
  int analyticError, i;
  double n1, r13, r23, r33, r21, r22, de, cosEta4, x, y, z, g1e, g2e;

  phiArray[0][2] = fixedAngle;
  phiArray[1][2] = fixedAngle;
  phiArray[2][2] = fixedAngle;
  phiArray[3][2] = fixedAngle;
  phiArray[4][2] = fixedAngle;
  phiArray[5][2] = fixedAngle;
  phiArray[6][2] = fixedAngle;
  phiArray[7][2] = fixedAngle;
  for (i = 1; i <= 8; i++)
    phiArrayValid[i - 1] = 1;
  analyticError = 0;
  x = posArray[0][3];
  y = posArray[1][3];
  z = posArray[2][3];
  cosEta4 = coseta4s(x, y, z, dArray[2], dArray[4]);
  if (cosEta4 * cosEta4 <= 0.10001e1) {
    if (0.1e1 < cosEta4 * cosEta4)
      cosEta4 = 0.1e1;
    phiArray[0][3] = acos(cosEta4);
    phiArray[1][3] = phiArray[0][3];
    phiArray[2][3] = phiArray[0][3];
    phiArray[3][3] = phiArray[0][3];
    phiArray[4][3] = -phiArray[0][3];
    phiArray[5][3] = phiArray[4][3];
    phiArray[6][3] = phiArray[4][3];
    phiArray[7][3] = phiArray[4][3];
  } else {
    for (i = 1; i <= 8; i++)
      phiArrayValid[i - 1] = 0;
    analyticError = 1;
    return(analyticError);
  }
  de = denomEta2Solve(z, dArray[2], dArray[4], phiArray[0][2], phiArray[0][3], arm);
  if (de == 0.0e0) {
    phiArray[0][1] = 0.3141592654e1;
    phiArray[1][1] = 0.3141592654e1;
    phiArray[2][1] = 0.3141592654e1;
    phiArray[3][1] = 0.3141592654e1;
  } else {
    n1 = numerEta2Solve1(z, dArray[2], dArray[4], phiArray[0][2], phiArray[0][3]);
    if (isnan(n1) == 0) {
      phiArray[0][1] = 0.2e1 * atan(n1 / de);
      phiArray[1][1] = phiArray[0][1];
      phiArray[2][1] = 0.2e1 * atan(numerEta2Solve2(z, dArray[2], dArray[4], phiArray[2][2], phiArray[2][3]) / de);
      phiArray[3][1] = phiArray[2][1];
    } else {
      phiArrayValid[0] = 0;
      phiArrayValid[1] = 0;
      phiArrayValid[2] = 0;
      phiArrayValid[3] = 0;
    }
  }
  de = denomEta2Solve(z, dArray[2], dArray[4], phiArray[4][2], phiArray[4][3], arm);
  if (de == 0.0e0) {
    phiArray[4][1] = 0.3141592654e1;
    phiArray[5][1] = 0.3141592654e1;
    phiArray[6][1] = 0.3141592654e1;
    phiArray[7][1] = 0.3141592654e1;
  } else {
    n1 = numerEta2Solve1(z, dArray[2], dArray[4], phiArray[4][2], phiArray[4][3]);
    if (isnan(n1) == 0) {
      phiArray[4][1] = 0.2e1 * atan(n1 / de);
      phiArray[5][1] = phiArray[4][1];
      phiArray[6][1] = 0.2e1 * atan(numerEta2Solve2(z, dArray[2], dArray[4], phiArray[6][2], phiArray[6][3]) / de);
      phiArray[7][1] = phiArray[6][1];
    } else {
      phiArrayValid[4] = 0;
      phiArrayValid[5] = 0;
      phiArrayValid[6] = 0;
      phiArrayValid[7] = 0;
    }
  }
  if (x == 0.0e0 && y == 0.0e0) {
    phiArray[0][0] = 0.0e0;
    phiArray[1][0] = 0.0e0;
    phiArray[2][0] = 0.0e0;
    phiArray[3][0] = 0.0e0;
    phiArray[4][0] = 0.0e0;
    phiArray[5][0] = 0.0e0;
    phiArray[6][0] = 0.0e0;
    phiArray[7][0] = 0.0e0;
  } else {
    if (phiArrayValid[0] != 0) {
      g1e = g1p(dArray[2], dArray[4], phiArray[0][1], phiArray[0][2], phiArray[0][3]);
      g2e = g2p(dArray[2], dArray[4], phiArray[0][1], phiArray[0][2], phiArray[0][3]);
      phiArray[0][0] = atan2(g1e * y * arm - g2e * x, g1e * x + g2e * y * arm);
      phiArray[1][0] = phiArray[0][0];
      g1e = g1p(dArray[2], dArray[4], phiArray[2][1], phiArray[2][2], phiArray[2][3]);
      g2e = g2p(dArray[2], dArray[4], phiArray[2][1], phiArray[2][2], phiArray[2][3]);
      phiArray[2][0] = atan2(g1e * y * arm - g2e * x, g1e * x + g2e * y * arm);
      phiArray[3][0] = phiArray[2][0];
      r13 = r13eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0][0], phiArray[0][1], phiArray[0][2], phiArray[0][3], arm);
      r23 = r23eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0][0], phiArray[0][1], phiArray[0][2], phiArray[0][3], arm);
      r33 = r33eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0][0], phiArray[0][1], phiArray[0][2], phiArray[0][3], arm);
      r21 = r21eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0][0], phiArray[0][1], phiArray[0][2], phiArray[0][3], arm);
      r22 = r22eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[0][0], phiArray[0][1], phiArray[0][2], phiArray[0][3], arm);
      phiArray[0][5] = eta6SolutionA(r13, r23, r33);
      phiArray[0][4] = eta5Solution(r13, r33, phiArray[0][5]);
      phiArray[0][6] = eta7Solution(r21, r22, phiArray[0][5], arm);
      phiArray[1][5] = eta6SolutionB(r13, r23, r33);
      phiArray[1][4] = eta5Solution(r13, r33, phiArray[1][5]);
      phiArray[1][6] = eta7Solution(r21, r22, phiArray[1][5], arm);
      r13 = r13eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[2][0], phiArray[2][1], phiArray[2][2], phiArray[2][3], arm);
      r23 = r23eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[2][0], phiArray[2][1], phiArray[2][2], phiArray[2][3], arm);
      r33 = r33eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[2][0], phiArray[2][1], phiArray[2][2], phiArray[2][3], arm);
      r21 = r21eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[2][0], phiArray[2][1], phiArray[2][2], phiArray[2][3], arm);
      r22 = r22eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[2][0], phiArray[2][1], phiArray[2][2], phiArray[2][3], arm);
      phiArray[2][5] = eta6SolutionA(r13, r23, r33);
      phiArray[2][4] = eta5Solution(r13, r33, phiArray[2][5]);
      phiArray[2][6] = eta7Solution(r21, r22, phiArray[2][5], arm);
      phiArray[3][5] = eta6SolutionB(r13, r23, r33);
      phiArray[3][4] = eta5Solution(r13, r33, phiArray[3][5]);
      phiArray[3][6] = eta7Solution(r21, r22, phiArray[3][5], arm);
    }
    if (phiArrayValid[4] != 0) {
      g1e = g1p(dArray[2], dArray[4], phiArray[4][1], phiArray[4][2], phiArray[4][3]);
      g2e = g2p(dArray[2], dArray[4], phiArray[4][1], phiArray[4][2], phiArray[4][3]);
      phiArray[4][0] = atan2(g1e * y * arm - g2e * x, g1e * x + g2e * y * arm);
      phiArray[5][0] = phiArray[4][0];
      g1e = g1p(dArray[2], dArray[4], phiArray[6][1], phiArray[6][2], phiArray[6][3]);
      g2e = g2p(dArray[2], dArray[4], phiArray[6][1], phiArray[6][2], phiArray[6][3]);
      phiArray[6][0] = atan2(g1e * y * arm - g2e * x, g1e * x + g2e * y * arm);
      phiArray[7][0] = phiArray[6][0];
      r13 = r13eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[4][0], phiArray[4][1], phiArray[4][2], phiArray[4][3], arm);
      r23 = r23eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[4][0], phiArray[4][1], phiArray[4][2], phiArray[4][3], arm);
      r33 = r33eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[4][0], phiArray[4][1], phiArray[4][2], phiArray[4][3], arm);
      r21 = r21eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[4][0], phiArray[4][1], phiArray[4][2], phiArray[4][3], arm);
      r22 = r22eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[4][0], phiArray[4][1], phiArray[4][2], phiArray[4][3], arm);
      phiArray[4][5] = eta6SolutionA(r13, r23, r33);
      phiArray[4][4] = eta5Solution(r13, r33, phiArray[4][5]);
      phiArray[4][6] = eta7Solution(r21, r22, phiArray[4][5], arm);
      phiArray[5][5] = eta6SolutionB(r13, r23, r33);
      phiArray[5][4] = eta5Solution(r13, r33, phiArray[5][5]);
      phiArray[5][6] = eta7Solution(r21, r22, phiArray[5][5], arm);
      r13 = r13eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[6][0], phiArray[6][1], phiArray[6][2], phiArray[6][3], arm);
      r23 = r23eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[6][0], phiArray[6][1], phiArray[6][2], phiArray[6][3], arm);
      r33 = r33eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[6][0], phiArray[6][1], phiArray[6][2], phiArray[6][3], arm);
      r21 = r21eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[6][0], phiArray[6][1], phiArray[6][2], phiArray[6][3], arm);
      r22 = r22eval(posArray[0][0], posArray[0][1], posArray[0][2], posArray[1][0], posArray[1][1], posArray[1][2], posArray[2][0], posArray[2][1], posArray[2][2], phiArray[6][0], phiArray[6][1], phiArray[6][2], phiArray[6][3], arm);
      phiArray[6][5] = eta6SolutionA(r13, r23, r33);
      phiArray[6][4] = eta5Solution(r13, r33, phiArray[6][5]);
      phiArray[6][6] = eta7Solution(r21, r22, phiArray[6][5], arm);
      phiArray[7][5] = eta6SolutionB(r13, r23, r33);
      phiArray[7][4] = eta5Solution(r13, r33, phiArray[7][5]);
      phiArray[7][6] = eta7Solution(r21, r22, phiArray[7][5], arm);
    }
  }
  return(analyticError);
}
