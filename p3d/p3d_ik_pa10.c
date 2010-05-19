#include "P3d-pkg.h"

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif

#include "../include/arm_models/pa10Const.h"

/**
 * Compute the inverse kinematics for the PA10-6C arm given a specified class of solution
 * @param posArray[][] input end-effector pose
 * @param solution input class of solution [1..8]
 * @param phiArray[] ouput joint values
 * @return 0 success, 1 fail, 2 approximate, 3 singular.
 */
int ikPA10ArmSolverUnique( double posArray[4][4], int solution, double phiArray[6])
{
	Gb_6rParameters arm_parameters;
	Gb_th eth;
	Gb_dataMGD d;
  Gb_q6 sq;
	Gb_q6 old_q;
	int solutionArray[8][3];
	Gb_statusMGI status;
	//matrix of solutions
  /*
	Sol 1 = 1 1 1
	Sol 2 = 1 1 -1
	Sol 3 = 1 -1 1
	Sol 4 = 1 -1 -1
	Sol 5 = -1 1 1
	Sol 6 = -1 1 -1
	Sol 7 = -1 -1 1
	Sol 8 = -1 -1 -1
	*/
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

	arm_parameters.a2 = PA10_ARM_A2;
	arm_parameters.r4 = PA10_ARM_R4;
	arm_parameters.epsilon = PA10_ARM_EPSILON;
	arm_parameters.of1 = PA10_ARM_OF1;
	arm_parameters.of2 = PA10_ARM_OF2;
	arm_parameters.of3 = PA10_ARM_OF3;
	arm_parameters.of4 = PA10_ARM_OF4;
	arm_parameters.of5 = PA10_ARM_OF5;
	arm_parameters.of6 = PA10_ARM_OF6;

	eth.vx.x = posArray[0][0];
	eth.vx.y = posArray[1][0];
	eth.vx.z = posArray[2][0];
	eth.vy.x = posArray[0][1];
	eth.vy.y = posArray[1][1];
	eth.vy.z = posArray[2][1];
	eth.vz.x = posArray[0][2];
	eth.vz.y = posArray[1][2];
	eth.vz.z = posArray[2][2];
	eth.vp.x = posArray[0][3];
	eth.vp.y = posArray[1][3];
	eth.vp.z = posArray[2][3];

#ifdef P3D_LOCALPATH
	Gb_q6_set(&old_q, (M_PI / 180.0) * PA10_Q1_INIT, (M_PI / 180.0) * PA10_Q2_INIT,(M_PI / 180.0) * PA10_Q3_INIT,(M_PI / 180.0) * PA10_Q4_INIT,(M_PI / 180.0) * PA10_Q5_INIT,(M_PI / 180.0) * PA10_Q6_INIT);

	status = Gb_MGI6rTh(&arm_parameters, &eth, solutionArray[solution][0], solutionArray[solution][1], solutionArray[solution][2], &old_q, &d, &sq);
  
	if(status == MGI_ERROR) {
		return 1;
	}
	Gb_q6_get(&sq, &phiArray[0], &phiArray[1], &phiArray[2], &phiArray[3], &phiArray[4], &phiArray[5]);
#endif
	return status;
}

/**
 * Compute all the inverse kinematics solution for the PA10-6C arm
 * @param posArray[][] input end-effector pose
 * @param valid[i] 1 if the solution i is valid, 0 otherwise
 * @param phiArray[][] the 8 solutions
 * @return 1 if at least one solution exist, 0 otherwise
 */
int ikPA10ArmSolver( double posArray[4][4], int valid[8], double phiArray[8][6])
{
 int cpt = 0;
 for(int i=0; i<8; i++) {
	 valid[i] = ikPA10ArmSolverUnique( posArray, i, phiArray[i]);
	 if(valid[i] == 1) {
		cpt ++;
	 }
 }
 if(cpt == 0) {
	return 0;
 }
 return 1;
}
