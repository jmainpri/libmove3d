// Description:
//    This file contains constant and variable definitions that are used within the BioEnergy module and is intended
//       and a link between the interface and the actual computing code

#ifndef BIOENERGY_COMMON_H
#define BIOENERGY_COMMON_H

#include "P3d-pkg.h"

#include <basics.h>
#include <stringExtra.h>

#define NB_OF_CHAR 300
#define NB_OF_LIB  4
#define NB_MAXCYC 100


#define TOTAL_NO_EIGENVECTORS 56	//This is the number of eigenvectors calculated.
								// For the moment this 56 value is hard-coded
#define NO_OF_COLLECTIVE_DEGREES 56    //This is the number of eigenvectors to be used, out of the TOTAL_NO_EIGENVECTORS available

static const int NB_OF_ENERGY_TERMS = 51;  //This constant comes from "mini.f", variable "ene" of function "fortrancodeenergyminimisation"


/* for normal modes and torsion values calculations */
typedef struct s_displacement{
int nbOfAtoms;
int nbOfEigenVec;
struct s_coord ***cooPtPtPt;
}displacement;

typedef struct s_vector{
int nbOfAtoms;
struct s_coord **pos;
}vector;

typedef struct s_torsion{
int **nPtPt;
int njoint;
}torsion;

typedef struct s_coord {
  double x, y, z;
} coord;


//The interface will allow the user to select on of these functioning modes:
// a) With a protein and no ligand, b) with a P and a non-protein ligend and c) with a P and L as protein
typedef enum {
 ligtypeNONE, ligtypeNON_PROTEIN, ligtypePROTEIN
} ligandType;




/* to take the data from p3d structures */
void GetAtomsPositionsFromLinearVectorToNx3matrix(double *coo_x, int natom, vector *vec);
void get_displacement(double *vec_x, int nvec, int natom, displacement *dis);

/* to calculate the torsions values */
void GetTorsionAnglesFromCartesianCoordinates ( vector *vec, torsion *tor, double *value);
void get_collective_degrees (displacement * dis, vector *vec, torsion *tor, double **col_d);


void GetAtomNumbersForDihedralsOfEachJoint (p3d_rob *mol, torsion *torPt, int firstJoint, int lastJoint );


int GetTotalNumberOfJoints();
int ComputeTotalNumberOfAtoms();
int GetFirstAndLastJointOfMainProteinAndLigand( int *firstJointProtein, int *lastJointProtein, int *firstJointLigand, int *lastJointProteinLigand );
void GetFirstAndLastJointOfLigand( int *firstJoint, int *lastJoint);
void GetFirstAndLastJointOfMainProtein( int *firstJoint, int *lastJoint);
int ComputeNumberOfAtomsOfMainProtein();
int ComputeNumberOfAtomsOfLigand();



//Methods that are available to the interface
void SetCurrentLigandType( ligandType newLigandType );
ligandType GetCurrentLigandType();

void SetCurrentFilesForProteinAndLigand( char* proteinFileName, char* ligandFileName );
//Warning. The method below returns copies. The memory is allocated with malloc so it shour be freed
void GetCurrentFilesForProteinAndLigand( char** proteinFileName, char** ligandFileName );
char *GetProteinPDBFile();
char *GetLigandPDBFile();

void get_torsions (p3d_rob *mol, torsion *torPt);

//File name manipulation
void GetFileNameWithPathWithoutExtension( char *fullFileName, char **outFileName );



#endif /* #ifndef BIOENERGY_COMMON_H */
