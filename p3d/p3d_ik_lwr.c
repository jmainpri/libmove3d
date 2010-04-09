#include "P3d-pkg.h"
#include "Localpath-pkg.h"

/**
 * Compute the inverse kinematics for the Kuka LWR arm given a specified class of solution
 * @param posArray[][] input end-effector pose
 * @param solution input class of solution [1..8]
 * @param phiArray[] ouput joint values
 * @return 0 success, 1 fail, 2 approximate, 3 singular.
 */
int ikLWRArmSolverUnique(double fixedAngle, double posArray[4][4], double phiArray[7], int solution) 
{
  Gb_q7 q, qs;
  double r3, r5;
  Gb_th th07;
  double epsilon;
  Gb_statusMGI status;
  int e1, e2, e3;
  int solutionArray[8][3]; 
  int result;
  
  epsilon= 1e-7;
  r3 = 0.4;
  r5 = 0.39;
  
  q.q1 = M_PI / 7.;
  q.q2 =-M_PI / 5.;
  //q.q3 = M_PI / 7.;
  q.q3 = fixedAngle;
  q.q4 =-M_PI / 15.;
  q.q5 = M_PI / 7.;
  q.q6 =-M_PI / 8.;
  q.q7 = M_PI / 7.;

  
  solutionArray[0][0] = 1;
  solutionArray[0][1] = 1;
  solutionArray[0][2] = 1;
  solutionArray[1][0] = 1;
  solutionArray[1][1] = 1;
  solutionArray[1][2] = -1;
  solutionArray[2][0] = 1;
  solutionArray[2][1] = -1;
  solutionArray[2][2] = 1;
  solutionArray[3][0] = 1;
  solutionArray[3][1] = -1;
  solutionArray[3][2] = -1;
  solutionArray[4][0] = -1;
  solutionArray[4][1] = 1;
  solutionArray[4][2] = 1;
  solutionArray[5][0] = -1;
  solutionArray[5][1] = 1;
  solutionArray[5][2] = -1;
  solutionArray[6][0] = -1;
  solutionArray[6][1] = -1;
  solutionArray[6][2] = 1;
  solutionArray[7][0] = -1;
  solutionArray[7][1] = -1;
  solutionArray[7][2] = -1; 
  
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

  
  e1= solutionArray[solution][0];
  e2= solutionArray[solution][1]; 
  e3= solutionArray[solution][2];

  status = kukaLBR_mgi_q_e(&th07, &q, r3, r5, epsilon, e1, e2, e3, &qs);  

  switch(status)
  {
    case MGI_OK: 
    	//printf("MGI_OK\n");
    	result= 0;
    break;
    case MGI_ERROR: 
    	//printf("MGI_ERROR\n");
    	result= 1;
    break;
    case MGI_APPROXIMATE: 
    	//printf("MGI_APPROXIMATE\n");
    	result= 2;
    break;
    case MGI_SINGULAR:
    	//printf("MGI_SINGULAR\n"); 
    	result= 3;
    break;
  } 
  
  
  if(status == MGI_ERROR) {
		return 1;
  }
	
  phiArray[0]= qs.q1;
  phiArray[1]= qs.q2;
  phiArray[2]= qs.q3;
  phiArray[3]= qs.q4;
  phiArray[4]= qs.q5;
  phiArray[5]= qs.q6;
  phiArray[6]= qs.q7;

  return result;
}

/**
 * Compute all the inverse kinematics solution for the Kuka LWR arm
 * @param posArray[][] input end-effector pose
 * @param valid[i] 1 if the solution i is valid, 0 otherwise
 * @param phiArray[][] the 8 solutions
 * @return 1 if at least one solution exist, 0 otherwise
 */
int ikLWRArmSolver(double fixedAngle, double posArray[4][4], double phiArray[8][7], int valid[8])
{
 int cpt = 0;
 for(int i=0; i<8; i++) {
	 //NB: ikLWRArmSolverUnique returns 0 if valid
	 valid[i] = ikLWRArmSolverUnique(fixedAngle, posArray, phiArray[i], i);
	 if(valid[i] == 0) {	
		valid[i] = 1;	 
		cpt ++;
	 }
 }
 if(cpt == 0) {
	return 0;
 }
 return 1;
} 
