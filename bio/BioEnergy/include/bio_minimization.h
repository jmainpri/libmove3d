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
