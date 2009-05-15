// Description: 
//   This file contains the memory allocation/deallocation methods.
//   There should be no local variables.

#include "include/bio_allocations.h"


void alloc_displacement_struct (displacement **disPtPt, int nbOfAtoms)
{
	displacement * disPt;
	coord ***cPtPtPt;
	coord **cPtPt = NULL;
	int i,j;
	
	disPt = (displacement*) malloc (sizeof(displacement));
	disPt->nbOfEigenVec = 0;
	
	disPt->nbOfAtoms = nbOfAtoms;
	
	cPtPtPt = (coord***) malloc ( ( TOTAL_NO_EIGENVECTORS + 1) * sizeof(coord**) );
	disPt->cooPtPtPt = cPtPtPt;  
	
	for (i = 1; i<= TOTAL_NO_EIGENVECTORS; i++) 
	{
		cPtPtPt[i] = (coord**) malloc( (disPt->nbOfAtoms + 1) * sizeof(coord*) );
		cPtPt = cPtPtPt[i];
		for (j=1; j<= disPt->nbOfAtoms; j++)
		{
			cPtPt[j] =  (coord*) malloc(sizeof(coord)); 
			cPtPt[j]->x = 0.0;
			cPtPt[j]->y = 0.0;
			cPtPt[j]->z = 0.0;
		}
	}
	*disPtPt = disPt;
}


void alloc_vector_struct (vector **vecPtPt, int nbOfAtoms)
{
	vector *vecPt;
	int i;
	
	vecPt = (vector*) malloc (sizeof(vector));
	vecPt -> nbOfAtoms = nbOfAtoms;
	
	vecPt->pos = (coord**) malloc ( (vecPt -> nbOfAtoms + 1) * sizeof(coord*) );
	for (i=1; i<= vecPt -> nbOfAtoms; i++)
	{
		vecPt->pos[i] = (coord*) malloc(sizeof(coord));
		vecPt->pos[i]->x = 0.0;
		vecPt->pos[i]->y = 0.0;
		vecPt->pos[i]->z = 0.0;
	}
	*vecPtPt = vecPt;
}

void alloc_torsion_struct (torsion **torPtPt, int nbOfJoints)
{
	torsion *torPt;
	int i,j;
	int *numPt;
	
	torPt = (torsion*) malloc( sizeof(torsion));
	torPt->njoint = nbOfJoints;
	
	torPt->nPtPt = (int**) malloc( ( torPt->njoint + 1 ) * sizeof(int*) );
	
	for(i=0; i<= torPt->njoint ; i++)
	{
		torPt->nPtPt[i] = (int*) malloc (4 * sizeof(int));
		numPt = torPt->nPtPt[i];
		for (j=0; j<=3; j++)
		{
			numPt[j]=0;
		}
	}
	*torPtPt = torPt;
}

/******************* free ***************************/

void free_displacement_struct (displacement *disPt)
{
	int i,j;
	coord **cPtPt;
	
	for (j=1; j<= TOTAL_NO_EIGENVECTORS; j++)
	{
		cPtPt = disPt->cooPtPtPt[j];
		for (i=1; i<= disPt->nbOfAtoms; i++) 
		{
			free (cPtPt[i]);
		}
		free (cPtPt);
	}
	free (disPt->cooPtPtPt);  
	free (disPt);
}


void free_vector_struct (vector *vecPt)
{
	int i;
	
	for (i=1; i<= vecPt->nbOfAtoms; i++)
	{
		free (vecPt->pos[i]);
	}
	free (vecPt->pos);
	free (vecPt);
}

void free_torsion_struct (torsion *torPt)
{
	int i;
	
	for (i=1; i<= torPt->njoint; i++) 
	{
		free (torPt->nPtPt[i]);
	}
	free (torPt->nPtPt);  
	free (torPt);
}
