#ifndef BIO_ALLOCATIONS_H
#define BIO_ALLOCATIONS_H


#include "bioenergy_common.h"

void alloc_displacement_struct (displacement **disPtPt, int nbOfAtoms);
void alloc_vector_struct (vector **vecPtPt, int nbOfAtoms);
void alloc_torsion_struct (torsion **torPtPt, int nbOfJoints);
void free_displacement_struct (displacement *disPt);
void free_vector_struct (vector *vecPt);
void free_torsion_struct (torsion *torPt);

#endif /* #ifndef BIO_ALLOCATIONS_H*/
