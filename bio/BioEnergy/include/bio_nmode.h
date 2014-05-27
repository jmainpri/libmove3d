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

#ifndef BIO_NMODE_H
#define BIO_NMODE_H

#include "bioenergy_common.h"
#include <device.h>


//This enum is used in the code to specify whether the user is exploring the normal mode displacements for the Protein or for the Ligand
typedef enum {
 explorationPROTEIN, explorationLIGAND
} explorationType;


int InitializeNormalModes( explorationType whichExplorationModeToInitialize, void (* AlertTheUserMethod)(char*, char*, char*), int (* QuestionTheUserMethod)(char*, int )  );
int dof5SliderMoved( int sliderNb, double sliderValue );
int SetCurrentEigenVector( int eignV );

void SetCurrentExplorationMode( explorationType newExplorationType );
explorationType GetCurrentExplorationMode();

// collective degrees
void bio_set_num_collective_degrees(int n);
int bio_get_num_collective_degrees(void);

int bio_infer_q_from_coldeg_conf(p3d_rob *robPt, configPt *q, double *coldeg_qinter, int n_coldeg);


/*************************************************************/
// functions for normal mode and torsions values calculations
void bio_flex_molecule( double alpha);
//void bio_flex_modes( double alpha);


#endif /* #ifndef BIO_NMODE_H */



