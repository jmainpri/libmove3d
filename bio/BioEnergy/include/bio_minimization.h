
#ifndef BIO_MINIMIZATION_H
#define BIO_MINIMIZATION_H

#include "bioenergy_common.h"

typedef enum {
 minimSDCG, minimSD, minimCG
} minimizationType;

int CalculateEnergy( p3d_rob *mol, double *outTotalEnergy);
int MolecularDynamics();
int EnergyMinimization( p3d_rob *mol, char *optionalOutputPDBFileName, double *energyAfterMinimization );
int EnergyMinimizationUsingAmbmov( p3d_rob *robotPt, double *tot_energyAfterMinimization);
int InitializeProtein( char *proteinFileName, char *ligandFileName, void (* AlertTheUserMethod)(char*, char*, char*), int (* QuestionTheUserMethod)(char*, int ) );
void SetMinimizationMethod( minimizationType newMinimizationType  );
minimizationType GetMinimizationMethod();

void SetNbMinimizationCycles( int nbOfMinimizationCycles );
int GetNbMinimizationCycles();

int ComputeDistances( int *, double*, double* );

#endif /* #ifndef BIO_MINIMIZATION_H */
