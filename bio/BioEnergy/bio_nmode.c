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
/*!
 *  \file bio_nmode.c
 *
 *    \brief Methods for using the Normal modes as given by the Sander program of AMBER suite
 *
 */


#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "Bio-pkg.h"
#include "Util-pkg.h"

#include "include/bio_nmode.h"
#include "include/bio_2amber.h"
#include "include/amb_eln.h"
#include "include/bio_allocations.h"

#define USE_METHOD_2  1

/* NOTE : ELNEMO PARAMETERS ARE IN pdbmat.f
   -> cutoff has been set to 6 (instead of 8 [default]) 
*/

static double NMMultiplicationCoef = 100.0;

// number of normal modes considered (for the exploration)
static int num_collective_degrees =  20; // by default

void set_displacement_along_CD( double alph, double *angle, int nbOfJoints, double *angle_d, double *outAngle);
static void set_displaced_atom_position(coord **c_disPt, vector *vec, vector *vec_dPt) ;
static void GetTorsionAnglesDifference ( double *currentInitialAngles, double *anglesWithAppliedNM, int nbOfJoints, double *dif, int i);

void SubstractFromTheLigandAtomIndexesTheNumberOfAtomsOfMainProtein (torsion *initialTorsions, int nbAtomsOfMainProtein);
void bio_flex_molecule_nmodes ( double alpha, int firstJoint, int lastJoint );

void get_coordinates_Prot (double *x_coo);
void get_coordinates_Lig ( int nbAtomsOfMainProtein,double *x_coo);

static inline void CopyVectorOfDoubles( double* sourceVector, double*destinationVector, int length);

 /* to read the data from files */
int ReadNormalModesFromFile(FILE *fmat, displacement *dis);

#ifdef USE_METHOD_2
void ComputeNMDisplacementsMethod2( displacement *normalModesDirections, double *amplitudes, double *newAngles );


//The <normalModesDirections> variable will contain the directions for each atom for each normal mode
//  as given by ElNemo
displacement *normalModesDirections = NULL;
vector *vecPt = NULL;
torsion *torPt = NULL;
#endif


static int set_atom_nums_for_FFref(int fjntnum);    // modif Juan
static int get_atom_nums_for_FFref(int **atom_nums); // modif Juan 
static int compute_and_set_baseFF_from_3atom_pos(p3d_rob *robPt, p3d_vector3 N_pos, p3d_vector3 CA_pos, p3d_vector3 C_pos, double amplification); // modif Juan


//######################
//## Local variables

//This indicates the index of the last slider that was moved
static int currentSliderNumberFor5dof = -1;	//This should be -1 at start
static int currentEigenVectorNumber = 7;	//This is actually currentSliderNumberFor5dof + 6

//This set of variables does not need to be initialized. They will only be pointing 
//   alternatively towards the proteinXXXXX or ligandXXXXXX  correspondant.
double **currentColectiveDegrees;
static  double *currentInitialAngles = NULL; 
static  double *currentInitialAnglesFor1NM = NULL;
static  double *currentCurrentAngles = NULL; 

//This holds the respective information concerning the main protein
double *proteinColectiveDegrees[ TOTAL_NO_EIGENVECTORS + 1 ];
static  double *proteinInitialAngles = NULL; 
static  double *proteinInitialAnglesFor1NM = NULL;
static  double *proteinCurrentAngles = NULL; 

//This holds the respective information concerning the ligand
double *ligandColectiveDegrees[ TOTAL_NO_EIGENVECTORS + 1 ];
static  double *ligandInitialAngles = NULL; 
static  double *ligandInitialAnglesFor1NM = NULL;
static  double *ligandCurrentAngles = NULL; 

static explorationType currentExploration = explorationPROTEIN;

void (* AlertTheUser)(char* title, char* text1, char* text2);
int (* QuestionTheUser)(char* question, int defaultBtn);


//Number of the N, CA and C atom of the 1st residue in the chain.
//Needed to to compute the location of the FF jnt  
static int atom_nums_for_FFref[3] = {-1,-1,-1};  // modif Juan

#ifdef USE_METHOD_2
//position of the 3 atoms that define the base FF jnt of the chain
static p3d_vector3 N_pos_for_FFref;
static p3d_vector3 CA_pos_for_FFref;
static p3d_vector3 C_pos_for_FFref;
static int atoms_pos_for_FFref_OK = 0;
#endif


//This method is called from the user interface. The parameter specifies:0-Protein, 1-Ligand
/**
 * @brief Should be called before using any other public method
 * @param whichExplorationModeToInitialize 
 * @return 
 */
int InitializeNormalModes( explorationType whichExplorationModeToInitialize, void (* AlertTheUserMethod)(char*, char*, char*), int (* QuestionTheUserMethod)(char*, int )  )
{
#ifndef USE_METHOD_2
	displacement *normalModesDirections = NULL;
	vector *vecPt = NULL;
	torsion *torPt = NULL;
#endif
	FILE *fmat = NULL;
	char file_eigenv[NB_OF_CHAR], *fileNameWithoutExtension;
	double *vec_x = NULL;
	double *px_coo = NULL;
	p3d_rob  *mol;
	int lenPdbFileName;
	int number_vec = 0, nbOfJoints = 0, nbOfAtoms = 0;
	int i =0, needToCreateFile = FALSE, fileExistsAlready = FALSE;
	int firstJoint, lastJoint;
	//nbAtomsMainMolecule is needed for the ligand also
	int nbAtomsMainMolecule = ComputeNumberOfAtomsOfMainProtein();


	switch( whichExplorationModeToInitialize )
	{
		case explorationPROTEIN:
			//Make sure that we cannot call this Initialization method twice.
			if ( proteinCurrentAngles != NULL )
			{
				PrintError(("Method InitializeNormalModes should only be called ONCE for the protein and ONCE for the ligand. Since there was an attempt to call it twice for the PROTEIN, the program will now terminate !"));
				exit(1);
			}
			GetFirstAndLastJointOfMainProtein( &firstJoint, &lastJoint);
			nbOfJoints = lastJoint - firstJoint + 1;
			nbOfAtoms = ComputeNumberOfAtomsOfMainProtein();

			currentColectiveDegrees 	= proteinColectiveDegrees;
			currentInitialAngles 		= proteinInitialAngles;
			currentInitialAnglesFor1NM = proteinInitialAnglesFor1NM;
			currentCurrentAngles 		= proteinCurrentAngles;
			set_atom_nums_for_FFref(firstJoint);  // modif Juan
			break;
		case explorationLIGAND:
			//Make sure that we cannot call this Initialization method twice.
			if ( ligandCurrentAngles != NULL )
			{
				PrintError(("Method InitializeNormalModes should only be called ONCE for the protein and ONCE for the ligand. Since there was an attempt to call it twice for the LIGAND, the program will now terminate !"));
				exit(1);
			}
			GetFirstAndLastJointOfLigand( &firstJoint, &lastJoint);
			nbOfJoints = lastJoint - firstJoint + 1;
			nbOfAtoms = ComputeNumberOfAtomsOfLigand();

			currentColectiveDegrees 	= ligandColectiveDegrees;
			currentInitialAngles 		= ligandInitialAngles;
			currentInitialAnglesFor1NM = ligandInitialAnglesFor1NM;
			currentCurrentAngles 		= ligandCurrentAngles;
			break;
		default:
			PrintError(("Unknow exploration !\n"));
			return -1;
	}

	fileNameWithoutExtension = MY_ALLOC(char, NB_OF_CHAR);
	px_coo = MY_ALLOC (double,(nbOfAtoms +1)* 3);

	alloc_vector_struct (&vecPt, nbOfAtoms);
	alloc_torsion_struct(&torPt, nbOfJoints );
	alloc_displacement_struct (&normalModesDirections, nbOfAtoms);

// 	if ( GetCurrentLigandType() != ligtypeNONE )
// 	{
// 		if (torPt_Lig == NULL)
// 			alloc_torsion_struct(&torPt_Lig, nbOfJoints );
// 	}

	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	GetAtomNumbersForDihedralsOfEachJoint(mol, torPt, firstJoint, lastJoint );

		

	switch( whichExplorationModeToInitialize )
	{
		case explorationPROTEIN:
			GetFileNameWithPathWithoutExtension( GetProteinPDBFile(), &fileNameWithoutExtension );
			//Get the Cartesian coordinates of the molecule
			get_coordinates_Prot(px_coo);
			break;
		case explorationLIGAND:
			GetFileNameWithPathWithoutExtension( GetLigandPDBFile(), &fileNameWithoutExtension );
			//Get the Cartesian coordinates of the molecule
			get_coordinates_Lig( nbAtomsMainMolecule , px_coo);
			SubstractFromTheLigandAtomIndexesTheNumberOfAtomsOfMainProtein ( torPt, nbAtomsMainMolecule );
			break;
		default:
			PrintError(("Unknow exploration.\n"));
			return -1;
	}


	/* files names */
	lenPdbFileName = strlen ( fileNameWithoutExtension );
	strcpy(file_eigenv, fileNameWithoutExtension);
	strcat(file_eigenv, ".eigenrtb\0");
	
	/* normal mode calculations */  
	
	needToCreateFile = TRUE;
	fmat = fopen(file_eigenv,"r");
	if(fmat != NULL) 
	{
		fileExistsAlready = TRUE; 
		fclose (fmat); 
	}

	if ( fileExistsAlready )
	{
		needToCreateFile = fl_show_question("File eigenrtb already exists. Do you want to generate it again?",0);
	}
	
	
	if (needToCreateFile )
	{
		vec_x = MY_ALLOC (double, 3 * nbOfAtoms * TOTAL_NO_EIGENVECTORS );

		number_vec = TOTAL_NO_EIGENVECTORS;

		/*     DIAGonalisation of a matrix with the RTB method.
		************************************************
		Required eigenvectors are supposed to be well described by rigid body motions (Rotations-Translations)
		of blocks of atoms buildt with sets of consecutive residues, as defined in the corresponding PDB file.
		***Output***: matrix: eigenrtb, with the eigenvalues and eigenvectors, in x,y,z "CERFACS" format.*/
		diagrtb_( fileNameWithoutExtension, &lenPdbFileName,px_coo,&nbOfAtoms,vec_x,&number_vec);

		//Get the displacements into a structure of our own (ie. normalModesDirections) instead of a linear vector (vec_x)
		get_displacement(vec_x, TOTAL_NO_EIGENVECTORS, nbOfAtoms, normalModesDirections);

		MY_FREE(vec_x, double, 3 * nbOfAtoms * TOTAL_NO_EIGENVECTORS );
	}
	else
	{	//We simply read the existing file
		fmat = fopen(file_eigenv,"r");
		PrintInfo(("Reading normal modes from file:  %s \n", file_eigenv));
		ReadNormalModesFromFile(fmat, normalModesDirections);
		fclose (fmat);
	}
	
	
	GetAtomsPositionsFromLinearVectorToNx3matrix(px_coo, nbOfAtoms, vecPt);
	MY_FREE(px_coo,double,3*(nbOfAtoms+1));
	
	//TODO Global variables. Allocated here, but never deallocated. Think of this
	//########## Allocation region ###########################
	currentInitialAngles = MY_ALLOC(double, nbOfJoints + 1);
	currentCurrentAngles = (double*) malloc(( (lastJoint - firstJoint + 1) + 1) * sizeof(double));
	currentCurrentAngles =       (double*)malloc( ((lastJoint - firstJoint + 1) + 1) * sizeof(double));
	currentInitialAnglesFor1NM = (double*)malloc( ((lastJoint - firstJoint + 1) + 1) * sizeof(double));
	for (i = 1; i<= TOTAL_NO_EIGENVECTORS ; i++)
	{
		currentColectiveDegrees[i] = MY_ALLOC (double, nbOfJoints + 1 );
	}
	//########## End of Allocation region ###########################

	GetTorsionAnglesFromCartesianCoordinates(vecPt, torPt, currentInitialAngles);

	get_collective_degrees(normalModesDirections,vecPt,torPt, currentColectiveDegrees);

#ifndef USE_METHOD_2
	free_displacement_struct (normalModesDirections);
  	free_vector_struct(vecPt);
	free_torsion_struct(torPt);
#endif

	MY_FREE(fileNameWithoutExtension, char, NB_OF_CHAR);

	CopyVectorOfDoubles( currentInitialAngles, currentInitialAnglesFor1NM, (lastJoint - firstJoint + 1) + 1);

	//Make sure we get our pointers back after having initialized them
	switch( whichExplorationModeToInitialize )
	{
		case explorationPROTEIN:
			proteinInitialAngles		= currentInitialAngles;
			proteinInitialAnglesFor1NM = currentInitialAnglesFor1NM;
			proteinCurrentAngles		= currentCurrentAngles;
			break;
		case explorationLIGAND:
			ligandInitialAngles 		= currentInitialAngles;
			ligandInitialAnglesFor1NM = currentInitialAnglesFor1NM;
			ligandCurrentAngles 		= currentCurrentAngles;
			break;
		default:
			PrintError(("Unknow exploration.\n"));
			return -1;
	}

	return 0; //success
}



//This functions computes the collective degrees of freedom expressed in internal coordinates (from the eigenvectors in cartesian coordinates)
//Actually, what it computes is the change in torsion angles given by the NM directions. The changes are stored and
//  are applied with a certain amplitude when the sliders are moved.
void get_collective_degrees (displacement * inDis, vector *inVec, torsion *inTor, double **outColDeg)
{
	double *angle_1;
	vector *vec_dPt;
	int i = 0, nbOfJoints = inTor->njoint;
	
	alloc_vector_struct( &vec_dPt, inVec->nbOfAtoms );
	angle_1 = MY_ALLOC (double, nbOfJoints + 1);
	
	for ( i = 1; i <= NO_OF_COLLECTIVE_DEGREES; i++)
	{
		set_displaced_atom_position( inDis->cooPtPtPt[ i ], inVec, vec_dPt); 
		GetTorsionAnglesFromCartesianCoordinates ( vec_dPt, inTor, angle_1); 
		
		GetTorsionAnglesDifference ( currentInitialAngles, angle_1, inTor->njoint, outColDeg[i], i);
	}
	
	MY_FREE(angle_1,double, nbOfJoints + 1);
	free_vector_struct( vec_dPt );
}






//////////////////////////////////////////
////////     NORMAL MODE  /////////////////

int SetCurrentEigenVector( int eignV )
{
	currentEigenVectorNumber = eignV;
	return 0;
}


//Called from the user interface.
void bio_flex_molecule( double alpha)
{
	p3d_jnt *jPt;
	double *newAngles;
	int i,currentJoint;
	
	p3d_rob *mol;
	double val;
	
	int nbJoints = 0, firstJoint, lastJoint;

#ifdef USE_METHOD_2
	double amplitudes[TOTAL_NO_EIGENVECTORS];
#endif
	
	switch( currentExploration )
	{
		case explorationPROTEIN:
			GetFirstAndLastJointOfMainProtein( &firstJoint, &lastJoint);
			nbJoints = lastJoint - firstJoint + 1;
			break;
		case explorationLIGAND:
			GetFirstAndLastJointOfLigand( &firstJoint, &lastJoint);
			nbJoints = lastJoint - firstJoint + 1;
			break;
		default:
			PrintError(("Unknow exploration.\n"));
			return;
	}

	newAngles = MY_ALLOC(double, nbJoints + 1);

	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

#ifdef USE_METHOD_2
	for(i = 1 ; i <= TOTAL_NO_EIGENVECTORS; i++)
	{	//The amplitudes vector will be all zeros except the currentEigenVector
		amplitudes[i] = ( i == currentEigenVectorNumber ) ? alpha : 0.0;
	}

	//Compute the new torsion angles according to the new method and store the result in the newAngles vector
	ComputeNMDisplacementsMethod2( normalModesDirections, amplitudes, newAngles );
	//Before applying these new angles we want to amplify the difference between the 
	// old angles and the new ones (ie.: new = old + (new-old)*coef instead of new = new)
	for(currentJoint = firstJoint; currentJoint <= lastJoint; currentJoint++)
	{
		newAngles[currentJoint - firstJoint + 1] = currentInitialAngles[currentJoint - firstJoint + 1] + ( newAngles[currentJoint - firstJoint + 1] - currentInitialAngles[currentJoint - firstJoint + 1] ) * NMMultiplicationCoef;
	}
	//compute_and_set_baseFF_from_3atom_pos(mol,N_pos_for_FFref,CA_pos_for_FFref,C_pos_for_FFref,NMMultiplicationCoef); // modif Juan
	
#else
	set_displacement_along_CD (alpha, currentInitialAngles, nbJoints,currentColectiveDegrees[ currentEigenVectorNumber ], newAngles);
#endif
  

	//TODO. DEBUG HERE ALSO

	for( currentJoint = firstJoint; currentJoint <= lastJoint; currentJoint++)
	{
		jPt = mol->joints[currentJoint];
		//if (angle_d[i-currentFirstJoint+1] != 0){
		// numPt = numPtPt[i-currentFirstJoint+1];
		//printf (" number of joint %d %d %d %d %d \n",(i+3), numPt[0],  numPt[1], numPt[2],numPt[3]);
		// printf(" dif angle %d %d %f \n ", i, (i+3),  angle_d[i]*180/3.14);
		//}
		val = newAngles[currentJoint-firstJoint+1];
		//val = *(++angle_d)*180/3.14;
		// if ((((mol->joints[i+3]->v)*180/3.14)-( angle[i]*180/3.14))*
		//	 (((mol->joints[i+3]->v)*180/3.14)-( angle[i]*180/3.14)) >= 0.7)
		//printf ("  joint n = %d %f %f \n", i, (mol->joints[i+3]->v)*180/3.14, angle[i]*180/3.14);
		p3d_jnt_set_dof(jPt, 0, val);
	}

	p3d_update_this_robot_pos_without_cntrt(mol);
	g3d_draw_allwin_active( );

	MY_FREE(newAngles, double, nbJoints + 1);
}


//This function is called from the user interface. It should return 0 on success or -1 otherwise
int dof5SliderMoved( int sliderNb, double sliderValue )
{
	int firstJoint, lastJoint, nbJoints;

	switch( currentExploration )
	{
		case explorationPROTEIN:
			GetFirstAndLastJointOfMainProtein( &firstJoint, &lastJoint);
			nbJoints = lastJoint - firstJoint + 1;
			break;
		case explorationLIGAND:
			GetFirstAndLastJointOfLigand( &firstJoint, &lastJoint);
			nbJoints = lastJoint - firstJoint + 1;
			break;
		default:
			PrintError(("Unknow exploration.\n"));
			return -1;
	}

	if ( (currentSliderNumberFor5dof != -1) && (currentSliderNumberFor5dof != sliderNb) )
	{
		CopyVectorOfDoubles( currentCurrentAngles, currentInitialAnglesFor1NM, nbJoints + 1);
		//printf ("\E[34m  angle = 1, number_mode %d  \033[0m  \n", sliderNb);
	}
	currentSliderNumberFor5dof = sliderNb;

	//This 6+nbSlider comes from the fact that in the interface we display NM7 through to NM11 as the 5 sliders
	currentEigenVectorNumber = 6 + sliderNb;
	bio_flex_molecule_nmodes( sliderValue, firstJoint, lastJoint ); 

	return 0;
}

static inline void CopyVectorOfDoubles( double* sourceVector, double*destinationVector, int length)
{
	int i = 0;
	for(i=0;i<length;i++)
		destinationVector[i]=sourceVector[i];
}

void bio_flex_molecule_nmodes ( double alpha, int firstJoint, int lastJoint )
{
	p3d_jnt *jPt;
	p3d_rob *mol;

	int i,currentJoint;
	double val;
#ifdef USE_METHOD_2
	double amplitudes[TOTAL_NO_EIGENVECTORS];
#endif

	//double val0,inc;

	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

#ifdef USE_METHOD_2
	for(i = 1 ; i <= TOTAL_NO_EIGENVECTORS; i++)
	{	//The amplitudes vector will be all zeros except the currentEigenVector
		amplitudes[i] = ( i == currentEigenVectorNumber ) ? alpha : 0.0;
	}

	//Compute the new torsion angles according to the new method and store the result in the newAngles vector
	ComputeNMDisplacementsMethod2( normalModesDirections, amplitudes, currentCurrentAngles );

	//Before applying these new angles we want to amplify the difference between the 
	// old angles and the new ones (ie.: new = old + (new-old)*coef instead of new = new)
	for(currentJoint = firstJoint; currentJoint <= lastJoint; currentJoint++)
	{
		currentCurrentAngles[currentJoint - firstJoint + 1] = currentInitialAngles[currentJoint - firstJoint + 1] + ( currentCurrentAngles[currentJoint - firstJoint + 1] - currentInitialAngles[currentJoint - firstJoint + 1] ) * NMMultiplicationCoef;
	}
	//compute_and_set_baseFF_from_3atom_pos(mol,N_pos_for_FFref,CA_pos_for_FFref,C_pos_for_FFref,NMMultiplicationCoef); // modif Juan

#else
	set_displacement_along_CD (alpha, currentInitialAnglesFor1NM, (lastJoint - firstJoint + 1), currentColectiveDegrees[ currentEigenVectorNumber ], currentCurrentAngles);
#endif


	for(i = firstJoint; i <= lastJoint; i++)
	{
		jPt = mol->joints[ i ];
		val = currentCurrentAngles[i - firstJoint + 1];		
		p3d_jnt_set_dof(jPt, 0, val);
		//val0 = p3d_jnt_get_dof(jPt,0);	
		//inc = (val-val0)*180/M_PI;
		//if((jPt->num > 1100) && (jPt->num < 1130))
		//  printf("jntnum : %d  ;  val0 = %f ; val = %f  ; inc = %f\n",jPt->num,val0,val,inc);
	}
	p3d_update_this_robot_pos_without_cntrt(mol);
	g3d_draw_allwin_active( );

//	 free(angle);
}


//###########################################################################
//###########################################################################
//###########################################################################

//in: normalModesDirections, amplitudes
//out: newAngles
void ComputeNMDisplacementsMethod2( displacement *normalModesDirections, double *amplitudes, double *newAngles )
{
#ifdef USE_METHOD_2
	vector *vecNewAtomPositions;

	//declare a structure to hold the sum of the vectors
	int nbAtomsMainMolecule = ComputeNumberOfAtomsOfMainProtein();
	
	struct s_vector *sumOfDirections;
	int indexAtom, indexNM;
	struct s_coord *posSum, *posOneDisplacement;
	
	int *ref_atom_nums = NULL;
	
	alloc_vector_struct ( &sumOfDirections, nbAtomsMainMolecule);


	//Do the sum of the NormalModes vectors with their corresponding amplitudes
	for( indexAtom = 1 ; indexAtom <= nbAtomsMainMolecule; indexAtom++ )
	{
		posSum = sumOfDirections->pos[indexAtom];
		posSum->x = posSum->y = posSum->z = 0.0;

		for(indexNM = 1 ; indexNM <= TOTAL_NO_EIGENVECTORS; indexNM ++ )
		{
			posOneDisplacement = normalModesDirections->cooPtPtPt[indexNM][indexAtom];

			posSum->x += amplitudes[indexNM] * posOneDisplacement->x;
			posSum->y += amplitudes[indexNM] * posOneDisplacement->y;
			posSum->z += amplitudes[indexNM] * posOneDisplacement->z;
		}
/*		//Multiply the difference by some coefficient in order to make its effects more visible
		posSum->x *= 20;
		posSum->y *= 20;
		posSum->z *= 20;*/
	}

	//Now that we have computed the directions, compute the change in the torsion angles
	//  if these changes were to be applied
	alloc_vector_struct( &vecNewAtomPositions, vecPt->nbOfAtoms );

	set_displaced_atom_position(sumOfDirections->pos, vecPt, vecNewAtomPositions); 
	GetTorsionAnglesFromCartesianCoordinates ( vecNewAtomPositions, torPt, newAngles); 

	// modif Juan
	// save the new position of the 3 atoms that define the base FF jnt of the chain
	get_atom_nums_for_FFref(&ref_atom_nums);
	if(ref_atom_nums[0] != -1) {
	  // static global variables
	  
	  N_pos_for_FFref[0] = vecNewAtomPositions->pos[ref_atom_nums[0]]->x;
	  N_pos_for_FFref[1] = vecNewAtomPositions->pos[ref_atom_nums[0]]->y;
	  N_pos_for_FFref[2] = vecNewAtomPositions->pos[ref_atom_nums[0]]->z;
	  CA_pos_for_FFref[0] = vecNewAtomPositions->pos[ref_atom_nums[1]]->x;
	  CA_pos_for_FFref[1] = vecNewAtomPositions->pos[ref_atom_nums[1]]->y;
	  CA_pos_for_FFref[2] = vecNewAtomPositions->pos[ref_atom_nums[1]]->z;
	  C_pos_for_FFref[0] = vecNewAtomPositions->pos[ref_atom_nums[2]]->x;
	  C_pos_for_FFref[1] = vecNewAtomPositions->pos[ref_atom_nums[2]]->y;
	  C_pos_for_FFref[2] = vecNewAtomPositions->pos[ref_atom_nums[2]]->z;
	  atoms_pos_for_FFref_OK = 1;
	}
	else {
	  atoms_pos_for_FFref_OK = 0;
	}
	// fmodif Juan

	free_vector_struct( vecNewAtomPositions );
	free_vector_struct( sumOfDirections );
#endif
}


//###########################################################################
//###########################################################################
//###########################################################################



// "alph" is the amplitude applied to the collective degree
// "angle" contains the initial torsion angles 
// "tor" is used only for obtaining the number of joints
// "angle_d" is the result of the get_collective_degrees 
void set_displacement_along_CD ( double alph, double *angle, int nbOfJoints, double *angle_d, double *outAngleDisplacement)
{
  int i;
	// NOTE : Should this method change, please update also the "bio_infer_q_from_coldeg_conf" method

  for (i=1; i<= nbOfJoints; i++){
    // TODO : 20 shoud be a parameter !!! (Maybe a paramter in the interface, or .. something related to the theory)
    outAngleDisplacement[i] = angle[i] + angle_d[i]*alph*20;
  }
}



void bio_set_num_collective_degrees(int n)
{
  num_collective_degrees = n;
}

int bio_get_num_collective_degrees(void)
{
  return (num_collective_degrees);
}


/*
//  gets the conformation (array of dihedral angles) corresponding to a given
//    value of the collective degrees 
int get_conformation_from_coldeg(double *alph, double *angle_0, torsion *tor, double **col_deg, double *angle)
{
	int i,j;
	int n_modes,fst_mode;
	
	fst_mode = 7;      
	n_modes = bio_get_num_collective_degrees();
	
	for(i=1; i<= tor->njoint; i++)
	{
		angle[i] = angle_0[i];
		for(j=0; j<n_modes; j++) 
		{
			// NOTE : 20 should be a parameter !!!
			angle[i] += col_deg[fst_mode+j][i]*alph[j]*20;
		}
	}
  	return 1;
}
*/



/*
	This method is called from outside (not from the user interface)
Retourne 0 ou 1
- les arguments d'entree sont :
  - robPt : le robot
  - n_coldeg : le nombre de degres collectifs
  - *coldeg_q : vecteur de longueur n_coldeg avec les valeurs 
                     des degres collectifs
- l'argument de sortie est :
  - *q : la configuration obtenue
*/
int bio_infer_q_from_coldeg_conf(p3d_rob *robPt, configPt *q, double *coldeg_q, int n_coldeg)
{
	int currentJoint;
	int numberOfJoints, firstJoint, lastJoint;
	int firstNormalModeToConsider=7;
#ifdef USE_METHOD_2
	int i;
	double amplitudes[TOTAL_NO_EIGENVECTORS];
#else
	int currentNM;
#endif

	//We are currently working ONLY on the main protein so we can obtain the number of joint with
	GetFirstAndLastJointOfMainProtein( &firstJoint, &lastJoint );
	numberOfJoints = lastJoint - firstJoint + 1;


#ifdef USE_METHOD_2
	for(i = 1 ; i < firstNormalModeToConsider ; i++)
		amplitudes[i] = 0.0;
	for(i = firstNormalModeToConsider ; i < (firstNormalModeToConsider + n_coldeg) ; i++)
		amplitudes[i] = coldeg_q[ i - firstNormalModeToConsider];
	for(i = firstNormalModeToConsider + n_coldeg ; i <= TOTAL_NO_EIGENVECTORS ; i++)
		amplitudes[i] = 0.0;

	//Compute the new torsion angles according to the new method and store the result in the newAngles vector
	ComputeNMDisplacementsMethod2( normalModesDirections, amplitudes, proteinCurrentAngles );

	//Before applying these new angles we want to amplify the difference between the 
	// old angles and the new ones (ie.: new = old + (new-old)*coef instead of new = new)
	for(currentJoint = firstJoint; currentJoint <= lastJoint; currentJoint++)
	{
	    proteinCurrentAngles[currentJoint - firstJoint + 1] = proteinInitialAngles[currentJoint - firstJoint + 1] + 
	      ( proteinCurrentAngles[currentJoint - firstJoint + 1] - proteinInitialAngles[currentJoint - firstJoint + 1] ) * NMMultiplicationCoef;
	}
	//compute_and_set_baseFF_from_3atom_pos(robPt,N_pos_for_FFref,CA_pos_for_FFref,C_pos_for_FFref,NMMultiplicationCoef); // modif Juan

#else

	//First compute the local values 
	for(currentJoint=1; currentJoint<= numberOfJoints; currentJoint++)
	{
		//Initialize with the starting conformation
		proteinCurrentAngles[currentJoint] = proteinInitialAngles[currentJoint];

		//And then add the proposed changes 
		for(currentNM=0; currentNM < n_coldeg; currentNM++) 
		{	// NOTE : 20 should be a parameter !!!
			// NOTE : Should this line here change, please update also the "set_displacement_along_CD" method
			proteinCurrentAngles[ currentJoint ] += proteinColectiveDegrees[ firstNormalModeToConsider + currentNM ][currentJoint] * coldeg_q[currentNM] * 20;
		}
	}

#endif


	//Then set the robot configuration
	for(currentJoint = firstJoint; currentJoint <= lastJoint; currentJoint++)
	{
		p3d_jnt_set_dof(robPt->joints[ currentJoint ], 0, proteinCurrentAngles[currentJoint - firstJoint + 1] );
	}

	// Then get the robot configuration into the output variable
	p3d_get_robot_config_into( robPt, q);

	return 1;	//successful exit
}


//This method reads the NormalModes from the <fmat> file pointer
int ReadNormalModesFromFile(FILE *fmat, displacement *dis)
{
	char oneLine[300];		//Used for reading a line from the file
	char vectorSymbol[6]; 	//This is only used for reading the "VECTOR" symbol from the file
	
	coord ***cPtPtPt;
	coord **cPtPt = NULL;
	int currentAtom = 0; 
	int currentEigenVect = 0;
	double tempDblX, tempDblY, tempDblZ;
	
	cPtPtPt = dis->cooPtPtPt;
	
		//TODO Here we should add a little more error messages in case it crashes. Like Line number, or something usefull (not urgent)
	while (fgets(oneLine,300,fmat)) 
	{
		sscanf (oneLine,"%6s", vectorSymbol);
	
		if (strncmp(vectorSymbol, "VECTOR", 6) != 0 )
		{
			currentAtom++;
			sscanf(oneLine,"%lf %lf %lf", &tempDblX, &tempDblY, &tempDblZ);
			 cPtPt[currentAtom]->x = tempDblX;
			 cPtPt[currentAtom]->y = tempDblY;
			 cPtPt[currentAtom]->z = tempDblZ;
			//printf("%f %f %f\n", cPt[i].x, cPt[i].y, cPt[i].z);
		}
		else
		{
			currentEigenVect++;
			cPtPt = cPtPtPt[currentEigenVect];
			//printf ("  number of atoms %d \n ", i);
			if ( currentAtom > 0 )
			{ //We avoid getting throuh here the first time
				dis->nbOfAtoms = currentAtom; 
			}
			currentAtom = 0;
			/*printf("VECTOR %d\n", n_vec);*/
		}
	}
	dis->nbOfEigenVec = currentEigenVect;
	return 1;
}


//This function transforms an array from a 1 dimensional array to a 2 dimensional array:
//  0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 => x=0.0  y=0.0  z=0.0
//                                                                         x=0.0  y=0.0  z=0.0
void get_displacement(double *vec_x, int nvec, int natom, displacement *dis)
{
  coord ***cPtPtPt;
  coord **cPtPt;
  int i, j; 

  dis->nbOfEigenVec = nvec; 
  dis->nbOfAtoms = natom; 

  cPtPtPt = dis->cooPtPtPt;

  for (i=1;i <= dis->nbOfEigenVec;i++){
    cPtPt = cPtPtPt[i];
    for(j=1;j <= dis->nbOfAtoms;j++){
      cPtPt[j]->x = vec_x[(i-1)*3* dis->nbOfAtoms +(j-1)*3+1];
      cPtPt[j]->y = vec_x[(i-1)*3* dis->nbOfAtoms +(j-1)*3+2];
      cPtPt[j]->z = vec_x[(i-1)*3* dis->nbOfAtoms +(j-1)*3+3];
    }
  }
}

//This method extracts the coordinates of each atom 
//  from a vector of the form  x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,... into  =>  [  x,y,z  ]
//                                                                                                   [   x,y,z ]
void GetAtomsPositionsFromLinearVectorToNx3matrix(double *coordinatesVector, int nbAtoms, vector *atomPositions)
{
  coord **posPt; 
  int i;

  atomPositions->nbOfAtoms = nbAtoms;

  posPt = atomPositions->pos;

  for (i = 1; i <= atomPositions->nbOfAtoms ; i++){
    posPt[i]->x = coordinatesVector[ (i-1) * 3];
    posPt[i]->y = coordinatesVector[ (i-1) * 3 +1];
    posPt[i]->z = coordinatesVector[ (i-1) * 3 +2];
    //printf ("%d %f %f %f \n",i, posPt[i].x, posPt[i].y, posPt[i].z);
  }
}





//This function computes the torsion values from the atom positions 
void GetTorsionAnglesFromCartesianCoordinates (vector *atomPositions, torsion *tor, double *value) 
{
  int i;
  int number1, number2, number3, number4;

  double xat1, xat2, xat3, xat4;
  double yat1, yat2, yat3, yat4;
  double zat1, zat2, zat3, zat4;

  double x1, x2, y1, y2, z1, z2; 

  double ux1, uy1, uz1;
  double ux2, uy2, uz2;
  double u, u1, u2;
  double a;

  int **numPtPt;
  int  *numPt;

  coord **c_pos;

  c_pos = atomPositions->pos;
  
  /************* calculation of torsion value **********/
  
  numPtPt = tor->nPtPt;
  for (i=1; i<= tor->njoint; i++){
    numPt = numPtPt[i];
    
    number1 = numPt[0];
    number2 = numPt[1];
    number3 = numPt[2];       
    number4 = numPt[3];
    
    //printf (" %d %d %d %d %d \n", i, number1, number2, number3, number4); 
    
    xat1 = c_pos[number1]->x;  
    xat2 = c_pos[number2]->x;
    xat3 = c_pos[number3]->x;      
    xat4 = c_pos[number4]->x;
    
    yat1 = c_pos[number1]->y;  
    yat2 = c_pos[number2]->y;
    yat3 = c_pos[number3]->y;      
    yat4 = c_pos[number4]->y;
    
    zat1 = c_pos[number1]->z;  
    zat2 = c_pos[number2]->z;
    zat3 = c_pos[number3]->z;      
    zat4 = c_pos[number4]->z;
    
    //printf (" coord atom i = %d %f %f %f %f\n %f %f %f %f\n %f %f %f %f \n", i, xat1, xat2, xat3, xat4, 
    //yat1, yat2, yat3, yat4, zat1, zat2, zat3, zat4);
    
    
    x1=xat2-xat1;
    y1=yat2-yat1;
    z1=zat2-zat1;
    x2=xat3-xat2;
    y2=yat3-yat2;
    z2=zat3-zat2;
    
    ux1=y1*z2-z1*y2;
    uy1=z1*x2-x1*z2;
    uz1=x1*y2-y1*x2;
    
    x1=xat4-xat3;
    y1=yat4-yat3;
    z1=zat4-zat3;
    
    ux2=z1*y2-y1*z2;
    uy2=x1*z2-z1*x2;
    uz2=y1*x2-x1*y2;
    
    u1=ux1*ux1+uy1*uy1+uz1*uz1;
    u2=ux2*ux2+uy2*uy2+uz2*uz2;
    u=u1*u2;
    
    if (u != 0.0) {
      a=(ux1*ux2+uy1*uy2+uz1*uz2)/sqrt(u);
      if (a < -1.0) a = -1.0;
      if (a >  1.0) a =  1.0;
      value[i] = acos(a);
      if (ux1*(uy2*z2-uz2*y2)+uy1*(uz2*x2-ux2*z2)+
	  uz1*(ux2*y2-uy2*x2) < 0.0) value[i] = -acos(a);
      //printf(" val_tor i = %d %f\n", i, value[i]);   

//	Value[i] -> put it into the move3d model -> new coordinates for 4th atom -> new torsion with 3 atom move3d and 1 from amber


//	p3d_jnt_set_dof(robPt->joints[ currentJoint ], 0, proteinCurrentAngles[currentJoint - firstJoint + 1] );
    }
    else
      PrintError(("Error in coordinates of atoms %d %d %d %d\n", 
	      number1, number2, number3, number4));
  }
}


//We have a matrix with all the eigen vectors in the variable "dis" (ie. normal modes)
// and we take the column "whatEigenVector" with which we update the "vec"
// and we place the result in "vec_dPt"
static void set_displaced_atom_position(coord **NMDirections, vector *vec, vector *vec_dPt) 
{
  int i;

  coord **c_in;
  coord **c_dPt;


  c_in = vec->pos;
  c_dPt = vec_dPt->pos;
  
  for (i=1; i<= vec->nbOfAtoms; i++)
    {
      /*printf (" int pos %d %f %f %f \n", i, c_in[i].x, c_in[i].y, c_in[i].z); */          

      c_dPt[i]->x = c_in[i]->x + NMDirections[i]->x ;
      c_dPt[i]->y = c_in[i]->y + NMDirections[i]->y ;
      c_dPt[i]->z = c_in[i]->z + NMDirections[i]->z ; 		

      /*printf (" set dis %f %f %f \n", cPt[i].x, cPt[i].y, cPt[i].z);*/ 
    }	 
  vec_dPt->nbOfAtoms = vec->nbOfAtoms;
}

//In "currentInitialAngles" we have the torsion angles corresponding to the initial positions of atoms
// In "anglesWithAppliedNM" we have the torsion angles corresponding to the displaced positions of atoms
// "dif" will contain the difference between currentInitialAngles and angle_1 
static void GetTorsionAnglesDifference ( double *currentInitialAngles, double *anglesWithAppliedNM, int nbOfJoints, double *dif, int index){

  int i;

  printf("\nDIF ANGLE FOR MODE %d\n\n",index);

  for (i=1; i<= nbOfJoints; i++){
    /* particularities around 0 and 180 degrees 
       for example:
       currentInitialAngles[i] = -5   and anglesWithAppliedNM[i] = 5
       currentInitialAngles[i] = -175 and anglesWithAppliedNM[i] = 175 
    */

    if ((currentInitialAngles[i]*anglesWithAppliedNM[i]) < 0){
      //if(index<=7)
	printf (" angl_0 %f angl_1 %f\n ", 
		(currentInitialAngles[i]*180.0/PI),(anglesWithAppliedNM[i]*180.0/PI));      
	if (fabs(currentInitialAngles[i]) < PI/2){
	  dif[i] = fabs(currentInitialAngles[i])+fabs(anglesWithAppliedNM[i]);  
	  if (anglesWithAppliedNM[i] < 0) dif[i] = -dif[i];
	  //if(index<=7)
	    printf (" %d angl_0 %f angl_1 %f dif %f \n", 
		    i,(currentInitialAngles[i]*180.0/PI),(anglesWithAppliedNM[i]*180.0/PI), (dif[i]*180.0/PI));
	}
	else {
	  dif[i] = 2*PI-(fabs(currentInitialAngles[i])+fabs(anglesWithAppliedNM[i]));
	  if (anglesWithAppliedNM[i] > 0) dif[i] = -dif[i];
	  //if(index<=7)
	    printf (" %d result angl_0 %f angl_1 %f dif %f \n", 
		    i,(fabs(currentInitialAngles[i])*180.0/PI),(fabs(anglesWithAppliedNM[i])*180.0/PI), (dif[i]*180.0/PI));
	}
    }
    else {
      dif[i] = currentInitialAngles[i] - anglesWithAppliedNM[i];
      if (((dif[i]*180.0/PI)*(dif[i]*180.0/PI))> 1.0) 
	//if(index<=7)
	  printf (" dif %d %f  %f  %f\n",i,(dif[i]*180.0/PI),(currentInitialAngles[i]*180.0/PI),(anglesWithAppliedNM[i]*180.0/PI));
    }
  }
  
  /* for (j=1; j<= tor->njoint; j++){
  printf (" dif %d %f  %f  %f\n",j,(dif[j]*180.0/PI),(currentInitialAngles[j]*180.0/PI),(anglesWithAppliedNM[j]*180.0/PI));
   }*/
}















//TODO The two methods here below should be grouped within only one. Actually all three of them, with get_coordinates from bio_minimisation
void get_coordinates_Prot (double *x_coo)
{
	p3d_poly **ptpoly;
	int i,j,k;
	int number = 0;
	char *nbStartIndex;
/*	int length = 0;
	char *end_pos;	*/
	int firstJoint, lastJoint;

	p3d_rob  *mol;
	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

	GetFirstAndLastJointOfMainProtein( &firstJoint, &lastJoint );

	for (i = firstJoint - 2; i <= lastJoint + 1; i++)
	{
		ptpoly = mol->joints[i]->o->pol;
		//printf (" number of joint Prot %d \n", (i+2));
		
		for(j=0; j< mol->joints[i]->o->np ; j++)
		{
			nbStartIndex = strchr(ptpoly[j]->poly->name, '.');
			if ( nbStartIndex == NULL )
			{
				PrintError(("The ptpoly[j]->poly->name expression is not correct !!!"));
				exit(2);
			}
			sscanf (nbStartIndex + 1,"%d",&number);
			
			for (k= 0; k<=2; k++)
			{
				x_coo[3*(number-1)+k] = ptpoly[j]->poly->pos[k][3];
			}
			//printf (" number of atom %d \n", number);
			//number_at++;
		}
	}
}


//TODO A FAIRE DEBUG
void get_coordinates_Lig (int nbAtomsOfProtein, double *x_coo)
{
	p3d_poly **ptpoly;
	int i,j,k;
	int number = 0;
	char *nbStartIndex;
	int number_at_Lig =0;
	int firstJoint, lastJoint;

	p3d_rob  *mol;
	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

	GetFirstAndLastJointOfLigand( &firstJoint, &lastJoint );


//  for (i=((torPt_Prot->njoint)+5); i<=(mol->njoints);i++){
	for (i = firstJoint - 2; i <= lastJoint + 1;i++)
	{
		ptpoly = mol->joints[i]->o->pol;
		// printf (" number of joint Lig %d \n",i);  
		for(j=0; j< mol->joints[i]->o->np; j++)
		{
			nbStartIndex = index(ptpoly[j]->poly->name, '.');
			if ( nbStartIndex == NULL )
			{
				PrintError(("The ptpoly[j]->poly->name expression is not correct (%s) !!!\n", ptpoly[j]->poly->name));
				exit(2);
			}
			sscanf (nbStartIndex + 1,"%d",&number);

			number_at_Lig = number - nbAtomsOfProtein;
			for (k= 0; k<=2; k++)
			{
				x_coo[3*(number_at_Lig-1)+k] = ptpoly[j]->poly->pos[k][3];
			}
			//number_at++;
			//printf (" %d %d %d %f %f %f \n ", number_at,number_at_Lig, number,
			//      x_coo[3*(number_at_Lig-1)],x_coo[3*(number_at_Lig-1)+1],
			//      x_coo[3*(number_at_Lig-1)+2]);
		}
	}
}




void SubstractFromTheLigandAtomIndexesTheNumberOfAtomsOfMainProtein(torsion *initialTorsions, int nbAtomsOfMainProtein)
{
	int i;
	int **numPtPt;
	int *numPt;
	
	numPtPt = initialTorsions->nPtPt; 
 
	for (i=1; i<= initialTorsions->njoint; i++)
	{
		numPt = numPtPt[i];
		numPt[0]= numPt[0] - nbAtomsOfMainProtein;
		numPt[1]= numPt[1] - nbAtomsOfMainProtein;
		numPt[2]= numPt[2] - nbAtomsOfMainProtein;
		numPt[3]= numPt[3] - nbAtomsOfMainProtein;
		//printf (" %d %d %d %d %d \n ", 
		//i, numPt_cng[0], numPt_cng[1], numPt_cng[2], numPt_cng[3]);
	}
}





//###############################################################################################
//############ HELPER METHODS


/**************************************************/
// This computes the number of atoms of the system Prot + Ligand (if it exists)
int ComputeTotalNumberOfAtoms()
{
	//TODO put some global variables to avoid computing all the time
	p3d_rob *mol;
	p3d_jnt **ptjoint;
	int i;
	int number = 0;
	
	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	
	ptjoint = mol->joints;
	for (i= 2; i<= mol->njoints; i++)
	{
	  if(ptjoint[i]->o != NULL)
	    number += ptjoint[i]->o->np; 
	}
	return number;
}

//These variables will be used so that subsequent calls to the GetFirstAndLastJointOfMainProteinAndLigand
// do not require the same computation
int internalFirstJointProtein=0, internalLastJointProtein=0, internalFirstJointLigand=0, internalLastJointLigand=0;

int GetFirstAndLastJointOfMainProteinAndLigand( int *firstJointProtein, int *lastJointProtein, int *firstJointLigand, int *lastJointLigand )
{
	p3d_rob *mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	int i = 0;

/*	char name_joint[300], *workBuf;
	char *word1, *word2, *word3, *word4;*/
	int n_j_Prot = 0;
	int n_j_Lig = 0;

	int flagFinishedProtein = FALSE;
	int nbLastJointOfProtein = 0;
	
	int nbOfJointsBetweenLigandAndProteinNotToBeConsidered = 3;

	if ( internalFirstJointProtein != 0)
	{	//It means that we have already computed the values so we can return them directly
		*firstJointProtein = internalFirstJointProtein;
		*lastJointProtein = internalLastJointProtein;
		*firstJointLigand = internalFirstJointLigand;
		*lastJointLigand = internalLastJointLigand;
		return 1;
	}

	if ( GetCurrentLigandType() == ligtypeNON_PROTEIN )
	{	nbOfJointsBetweenLigandAndProteinNotToBeConsidered = 2;	}
	if ( GetCurrentLigandType() == ligtypePROTEIN )
	{	nbOfJointsBetweenLigandAndProteinNotToBeConsidered = 3;	}
	 
	*firstJointProtein = *lastJointProtein = *firstJointLigand = *lastJointLigand = 0;

	for (i=4; i < mol->njoints; i++)
	{
		if ( flagFinishedProtein == FALSE ) 
		{
			n_j_Prot++;
			if ( strstr(mol->joints[i]->name, "0-0-0-0") != NULL )
			{
				flagFinishedProtein = TRUE;
				nbLastJointOfProtein = i -1;
				//printf (" end protein %d \n", nbLastJointOfProtein );
			}

/*			if(word4 == NULL) {
			  PrintError(("CAN'T ADAPT P3D MODEL FROM AMBER MINIMIZATION : OLD FORMAT\n"));
			  return 0;
			}*/
		}
		else if ( i > (nbLastJointOfProtein + nbOfJointsBetweenLigandAndProteinNotToBeConsidered)
			 && ( i < (mol->njoints - 1)) ) 
		{
			n_j_Lig++;
		}
	}//for


	(*firstJointProtein) = 4;
	(*lastJointProtein) = nbLastJointOfProtein;
	if ( nbLastJointOfProtein == 0 )
	{	//It means that we have looped throught all the joints and we have not yet found the "0-0-0-0"
		// In this case we can consider that the main Protein spans the entire "robot"
		(*lastJointProtein) = mol->njoints - 1;
	}
	if ( GetCurrentLigandType() != ligtypeNONE )
	{	//if we have a ligand also, set the appropriate variables
		// But not before making a safety check
		if ( nbLastJointOfProtein + nbOfJointsBetweenLigandAndProteinNotToBeConsidered + 1 < mol->njoints ) 
		{
			*firstJointLigand = nbLastJointOfProtein + nbOfJointsBetweenLigandAndProteinNotToBeConsidered + 1;
			*lastJointLigand = *firstJointLigand + n_j_Lig;
		}
		else
		{
			PrintError(("\E[31m Could not find the start and end joint of the ligand !! \033[0m  \n"));
			return 0;
		}
	}

	//Make sure we store the computed values so that we do not recompute them 
	// for each subsequent call.
	internalFirstJointProtein = *firstJointProtein;
	internalLastJointProtein = *lastJointProtein;
	internalFirstJointLigand = *firstJointLigand;
	internalLastJointLigand = *lastJointLigand;

	PrintInfo(("\E[34m Protein joints: %d -> %d \n Ligand joints: %d -> %d \E[0m\n", *firstJointProtein, *lastJointProtein, *firstJointLigand, *lastJointLigand));

	return 1;
}


void GetFirstAndLastJointOfMainProtein( int *firstJoint, int *lastJoint)
{
	int fl, ll;
	GetFirstAndLastJointOfMainProteinAndLigand( firstJoint, lastJoint, &fl, &ll );
}
void GetFirstAndLastJointOfLigand( int *firstJoint, int *lastJoint)
{
	int fp, lp;
	GetFirstAndLastJointOfMainProteinAndLigand( &fp, &lp, firstJoint, lastJoint);
}




// This computes the number of atoms of only the main Protein
int ComputeNumberOfAtomsOfMainProtein()
{
	//TODO put some global variables to avoid computing all the time
	p3d_rob *mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	int numberOfAtoms = 0, i = 0;
	
	int firstJoint, lastJoint;
	GetFirstAndLastJointOfMainProtein( &firstJoint, &lastJoint);

	//The "-1" and "+1" comes from the fact that the starting and ending joints ignore the joints named "0-0-0-0", but 
	// for the counting of atoms we need those too
	for ( i = firstJoint - 2; i <= lastJoint + 1; i++)
	{	
		numberOfAtoms += mol->joints[i]->o->np; 
	}

	return numberOfAtoms;
}


// This computes the number of atoms of only the ligand, may it be a protein or not.
int ComputeNumberOfAtomsOfLigand()
{
	//TODO put some global variables to avoid computing all the time
	p3d_rob *mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	int numberOfAtoms = 0, i = 0;
	
	int firstJoint, lastJoint;
	GetFirstAndLastJointOfLigand( &firstJoint, &lastJoint);

	//The "-1" and "+1" comes from the fact that the starting and ending joints ignore the joints named "0-0-0-0", but 
	// for the counting of atoms we need those too
	for ( i = firstJoint - 1; i < lastJoint + 1; i++)
	{	
		numberOfAtoms += mol->joints[i]->o->np; 
	}

	return numberOfAtoms;
}

int GetTotalNumberOfJoints()
{
   p3d_rob *mol;
   mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);    
   return mol->njoints;  //Get the number of joints from the current robot = molecule
}


explorationType GetCurrentExplorationMode()
{
	return currentExploration;
}
void SetCurrentExplorationMode( explorationType newExplorationType )
{
	currentExploration = newExplorationType;;
}




// modif Juan
// functions related with the atoms that define the frame of the FF jnt of the peptide chain
// WARNING : these functions suppose that there is only one chain !!!
static int set_atom_nums_for_FFref(int fjntnum)
{
  p3d_rob *robotPt;
  int fres;
  Joint_tablespt *jnt_table;
  p3d_jnt *jntPt;
  char jntname[256];
  char *w,*w_n1,*w_n2,*w_n3;

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  fres = get_AAnumber_from_jnt(robotPt->joints[fjntnum]);

  jnt_table = give_joint_tables(robotPt->num);

  // psi jnt of the 1st residue
  jntPt = jnt_table[fres]->bb_joints[jnt_table[fres]->n_bb_joints - 1];
  strcpy(jntname, jntPt->name);

  w = strtok(jntname, ".");   
  w = strtok(NULL, "."); 
  w = strtok(NULL, ".");
  w_n1 = strtok(NULL,"-");
  w_n2 = strtok(NULL,"-");      
  w_n3 = strtok(NULL,"-");

  sscanf (w_n1,"%d",&atom_nums_for_FFref[0]);
  sscanf (w_n2,"%d",&atom_nums_for_FFref[1]);
  sscanf (w_n3,"%d",&atom_nums_for_FFref[2]);

  return 1;  
}

static int get_atom_nums_for_FFref(int **atom_nums)
{
  if(atom_nums_for_FFref[0] == -1) {
    *atom_nums = NULL;
    return 0;
  }
  else {
    *atom_nums = atom_nums_for_FFref;
    return 1;
  }  
}


// function re-computing the base frame of the chain and setting the FF jnt values 
// NOTE : this function could be generalized and placed in other file
static int compute_and_set_baseFF_from_3atom_pos(p3d_rob *robPt, p3d_vector3 N_pos, p3d_vector3 CA_pos, p3d_vector3 C_pos, double amplification)
{
  p3d_vector3 posdiff,prev_zaxis;
  p3d_vector3 xaxis,yaxis,zaxis;
  p3d_matrix4 F;
  double newFFdofs[6];
  static int first_time = 1;
  static double initFFdofs[6];
  p3d_jnt *jntPt,*base_jntPt;
  double vmin,vmax,vnew;
  int i;
  //char s[20];
  int indexletter = 1;

  /* NOTE: this part of the function is very similar to the exiting function 
     bio_compute_Fb_from_3atom_pos in bio_track_loop.c */
  p3d_vectSub(N_pos,C_pos,posdiff);
  p3d_vectNormalize(posdiff,prev_zaxis);
  p3d_vectSub(CA_pos,N_pos,posdiff);
  p3d_vectNormalize(posdiff,zaxis);
  p3d_vectXprod(prev_zaxis,zaxis,posdiff);
  p3d_vectNormalize(posdiff,xaxis);
  p3d_vectXprod(zaxis,xaxis,posdiff);
  p3d_vectNormalize(posdiff,yaxis);
    
  F[0][0] = xaxis[0]; F[0][1] = yaxis[0]; F[0][2] = zaxis[0]; F[0][3] = CA_pos[0]; 
  F[1][0] = xaxis[1]; F[1][1] = yaxis[1]; F[1][2] = zaxis[1]; F[1][3] = CA_pos[1]; 
  F[2][0] = xaxis[2]; F[2][1] = yaxis[2]; F[2][2] = zaxis[2]; F[2][3] = CA_pos[2]; 
  F[3][0] = 0.0;      F[3][1] = 0.0;      F[3][2] = 0.0     ; F[3][3] = 1.0; 

  /*------------*/
  
  // extract the new FF jnt values (WITHOUT AMPLIFICATION)
  p3d_mat4ExtractPosReverseOrder(F,&(newFFdofs[0]),&(newFFdofs[1]),&(newFFdofs[2]),&(newFFdofs[3]),&(newFFdofs[4]),&(newFFdofs[5]));

  // identify (1st) subchain base jnt
  // WARNING : suppose only one chain !!!
  jntPt = robPt->joints[1];
  base_jntPt = NULL;
  while((base_jntPt == NULL) && (jntPt != NULL)) {
    indexletter = 1;
    if(strcmp(givemeword(jntPt->name, '.', &indexletter),"chain_base") == 0)
      base_jntPt = jntPt;
    else
      jntPt = jntPt->next_jnt[jntPt->n_next_jnt - 1];
  }
  
  if(first_time) {
    first_time = 0;
    // WARNING : suppose that the "robot" is in INITIAL configuration
    // extract initial FF jnt values (NOTE: THIS OPERATION SHOULD BE MADE ONLY ONCE !!!)
    for(i=0; i<6; i++) {
      initFFdofs[i] = p3d_jnt_get_dof(base_jntPt,i);
    }
  }

  if(base_jntPt != NULL) {
    // set jnt values
    for(i=0; i<6; i++) {
      // NOTE: the jnt frame is defined in abs coordinates
      // new AMPLIFIED FF jnt values
      //vnew = base_jntPt->dof_data[i].old_v + (newFFdofs[i] - base_jntPt->dof_data[i].old_v) * amplification;
      vnew = initFFdofs[i] + (newFFdofs[i] - initFFdofs[i]) * amplification;
      p3d_jnt_get_dof_bounds(base_jntPt,i,&vmin,&vmax);
      if((i < 3) || !p3d_jnt_is_dof_circular(base_jntPt,i)) {
	if((vmax < vnew) || (vmin > vnew)) {
	  printf("ERROR : compute_and_set_baseFF_from_3atom_pos : exceeded jnt limints : J%d\n",base_jntPt->num);
	  return 0;
	}
      }
      else {
	if(vnew < vmin)
	  vnew += 2.0*M_PI;
	else if(vnew > vmax)
	  vnew -= 2.0*M_PI;
      }
      p3d_jnt_set_dof(base_jntPt,i,vnew);
    }
  }
  else {
    printf("ERROR : compute_and_set_baseFF_from_3atom_pos : unable to identify chain base joint\n");
    return 0;
  }
  return 1;
}

// fmodif Juan
