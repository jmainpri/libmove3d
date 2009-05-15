
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



