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
#include "P3d-pkg.h"
#include "Localpath-pkg.h"

#include <iostream>
using namespace std;

/**
 * Compute the inverse kinematics for the
 * pr2 arms
 * @param posArray[][] input end-effector pose
 * @param solution input class of solution [1..8]
 * @param phiArray[] ouput joint values
 * @return 0 success, 1 fail, 2 approximate, 3 singular.
 */
int ikPr2ArmSolverUnique(double fixedAngle, double min[7], double max[7] , double posArray[4][4], double phiArray[7]) 
{
  Gb_q7 q, qMin, qMax;
  Gb_th th07;
  double epsilon;
  Gb_statusMGI status;
  int nbsolution; 
  int result;
  
  double a1=0.1; 
  double r3=0.4;  
  double r5=0.321;
  
  Gb_q7 qsol[32];
  
  qMin.q1 = min[0]; 
  qMin.q2 = min[1];
  qMin.q3 = min[2];
  qMin.q4 = min[3];
  qMin.q5 = min[4];
  qMin.q6 = min[5];
  qMin.q7 = min[6];
  
  qMax.q1 = max[0]; 
  qMax.q2 = max[1];
  qMax.q3 = max[2];
  qMax.q4 = max[3];
  qMax.q5 = max[4];
  qMax.q6 = max[5];
  qMax.q7 = max[6];

  q.q1 = (max[0] - min[0])/2;
  q.q2 = (max[1] - min[1])/2;
  q.q3 = fixedAngle;
  q.q4 = (max[3] - min[3])/2;
  q.q5 = (max[4] - min[4])/2;
  q.q6 = (max[5] - min[5])/2;
  q.q7 = (max[6] - min[6])/2;

  th07.vx.x = posArray[0][0];
  th07.vx.y = posArray[1][0];
  th07.vx.z = posArray[2][0];
  th07.vy.x = posArray[0][1];
  th07.vy.y = posArray[1][1];
  th07.vy.z = posArray[2][1];
  th07.vz.x = posArray[0][2];
  th07.vz.y = posArray[1][2];
  th07.vz.z = posArray[2][2];
  th07.vp.x = posArray[0][3];
  th07.vp.y = posArray[1][3];
  th07.vp.z = posArray[2][3];

//   printf("pr2_mgi_q3_8\n");
  status = pr2_mgi_q3_8(&th07, &q, a1, r3, r5, &qMin, &qMax, epsilon, qsol, &nbsolution) ;
  
  switch(status)
  {
    case MGI_OK:
//       printf("MGI_OK\n");
    	result= 0;
    break;
    case MGI_ERROR: 
//     	printf("MGI_ERROR\n");
    	result= 1;
    break;
    case MGI_APPROXIMATE: 
//     	printf("MGI_APPROXIMATE\n");
    	result= 2;
    break;
    case MGI_SINGULAR:
//     	printf("MGI_SINGULAR\n"); 
    	result= 3;
    break;
  } 
  
  if (nbsolution == 0) 
  {
//    printf("MGI_ERROR = No solution\n");
   return 1;
  }
  
  if(status == MGI_ERROR) {
    return 1;
  }

  phiArray[0]= qsol[0].q1;
  phiArray[1]= qsol[0].q2;
  phiArray[2]= qsol[0].q3;
  phiArray[3]= qsol[0].q4;
  phiArray[4]= qsol[0].q5;
  phiArray[5]= qsol[0].q6;
  phiArray[6]= qsol[0].q7;

//   cout << "------------------" << endl;
//   cout << "phiArray[0] = " << phiArray[0] << endl;
//   cout << "phiArray[1] = " << phiArray[1] << endl;
//   cout << "phiArray[2] = " << phiArray[2] << endl;
//   cout << "phiArray[3] = " << phiArray[3] << endl;
//   cout << "phiArray[4] = " << phiArray[4] << endl;
//   cout << "phiArray[5] = " << phiArray[5] << endl;
//   cout << "phiArray[6] = " << phiArray[6] << endl;

  return result;
}
