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
int ikPr2ArmSolverUnique(double fixedAngle, double min[7], double max[7] , double posArray[4][4], double phiArray[7], int solution) 
{
  Gb_q7 q, qMin, qMax;
  Gb_th th07;
  double epsilon;
  Gb_statusMGI status;
//  int e1, e2, e3;
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
  
//  epsilon= 1e-7;
//  r3 = 0.4;
//  r5 = 0.39;
  
  q.q1 = 20 * M_PI / 180;
  q.q2 = 20 * M_PI / 180;
  q.q3 = fixedAngle;
  q.q4 = -20 * M_PI / 180;
  q.q5 = 20 * M_PI / 180;
  q.q6 = -20 * M_PI / 180;
  q.q7 = 0 * M_PI / 180;
  
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

  printf("pr2_mgi_q3_8\n");
  status = pr2_mgi_q3_8(&th07, &q, a1, r3, r5, &qMin, &qMax, epsilon, qsol, &nbsolution) ;
  
  switch(status)
  {
    case MGI_OK: 
      printf("MGI_OK\n");
    	result= 0;
    break;
    case MGI_ERROR: 
    	printf("MGI_ERROR\n");
    	result= 1;
    break;
    case MGI_APPROXIMATE: 
    	printf("MGI_APPROXIMATE\n");
    	result= 2;
    break;
    case MGI_SINGULAR:
    	printf("MGI_SINGULAR\n"); 
    	result= 3;
    break;
  } 
  
  if (nbsolution == 0) 
  {
   printf("MGI_ERROR = No solution\n");
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
  
  cout << "------------------" << endl;
  cout << "phiArray[0] = " << (180/M_PI)*phiArray[0] << endl;
  cout << "phiArray[1] = " << (180/M_PI)*phiArray[1] << endl;
  cout << "phiArray[2] = " << (180/M_PI)*phiArray[2] << endl;
  cout << "phiArray[3] = " << (180/M_PI)*phiArray[3] << endl;
  cout << "phiArray[4] = " << (180/M_PI)*phiArray[4] << endl;
  cout << "phiArray[5] = " << (180/M_PI)*phiArray[5] << endl;
  cout << "phiArray[6] = " << (180/M_PI)*phiArray[6] << endl;

  return result;
}

/**
 * Compute all the inverse kinematics solution for the pr2 arm
 * @param posArray[][] input end-effector pose
 * @param valid[i] 1 if the solution i is valid, 0 otherwise
 * @param phiArray[][] the 8 solutions
 * @return 1 if at least one solution exist, 0 otherwise
 */
int ikPr2ArmSolver(double fixedAngle, double posArray[4][4], double phiArray[8][7], int valid[8])
{
 int cpt = 0;
 for(int i=0; i<8; i++) {
	 //NB: ikLWRArmSolverUnique returns 0 if valid
	 //valid[i] = ikLWRArmSolverUnique(fixedAngle, posArray, phiArray[i], i);
	 if(valid[i] == 0) {	
		valid[i] = 1;	 
		cpt ++;
	 }
         else
         { valid[i] = 0; }
 }
 if(cpt == 0) {
	return 0;
 }
 return 1;
} 
