extern int ikKUKAArmSolver(double fixedAngle, double aArray[7], double alphaArray[7],
                             double dArray[7], double thetaArray[7], double posArray[4][4],
                             double phiArray[8][7], int phiArrayValid[8], int arm);

extern int ikKUKAArmSolverUnique(double fixedAngle, double aArray[7], double alphaArray[7],
                                   double dArray[7], double thetaArray[7], double posArray[4][4],
                                   double phiArray[7], int arm, int solution);
