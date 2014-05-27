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
#include "P3d-pkg.h"
#include "include/bioenergy_common.h"





//The currentLigandType variable indicates (as the name so clearly suggests :P ) the type of ligand currently used
static ligandType currentLigandType = ligtypeNONE;

//This will hold the name of the PDB protein file
char *currentProteinPDBFile;
//This will hold the name of the PDB ligand file (if there is a ligand)
char *currentLigandPDBFile;









void SetCurrentLigandType( ligandType newLigandType )
{
	currentLigandType = newLigandType;
}
ligandType GetCurrentLigandType()
{
	return currentLigandType;
}


void SetCurrentFilesForProteinAndLigand( char* proteinFileName, char* ligandFileName )
{
	currentProteinPDBFile = strdup( proteinFileName );
	if ( ligandFileName != NULL)
		currentLigandPDBFile = strdup( ligandFileName );
	else
		currentLigandPDBFile = NULL;
}

//Warning ! For the methods GetProteinPDBFile and GetLigandPDBFile make sure you DO NOT 
// alter the content of the returned string. Be sure to make a copy 
char *GetProteinPDBFile()
{
	return currentProteinPDBFile;
}
char *GetLigandPDBFile()
{
	return currentProteinPDBFile;
}
/**
 * @brief This method below returns copies. The memory is allocated with malloc so it shour be freed
 * @param proteinFileName 
 * @param ligandFileName 
 */
void GetCurrentFilesForProteinAndLigand( char** proteinFileName, char** ligandFileName )
{
	char *copyLigand = NULL, *copyProtein = NULL;
	
	copyProtein = strdup( currentProteinPDBFile );
	*proteinFileName = copyProtein;
	
	if ( currentLigandPDBFile != NULL)
	{
		copyLigand = strdup( currentLigandPDBFile );
		*ligandFileName = copyLigand;
	}
	else
	{
		*ligandFileName = NULL;
	}
}


// void get_torsions (p3d_rob *mol, torsion *torPt)
// {
//   p3d_jnt **ptjoint;
//   char *name_joint;
//   char *word1;
//   char *word2;
//   char *word3;
//   char *w_n1;
//   char *w_n2;
//   char *w_n3;
//   char *w_n4;
//   int i;
//   int n_j = 0;
//   int **numPtPt;
//   int *numPt;
// 
// 
//   numPtPt = torPt->nPtPt;
//   ptjoint = mol->joints;
// 
//   for (i=1; i<((mol->njoints)-3); i++){
//     n_j++;
//     numPt = numPtPt[i];
//     name_joint = strdup(ptjoint[i+3]->name);
//     printf (" name of joint %s \n", name_joint);
//     word1 = strtok(name_joint, ".");   
//     word2 = strtok(NULL, "."); 
//     word3 = strtok(NULL, ".");
//     w_n1 = strtok(NULL,"-");
//     w_n2 = strtok(NULL,"-");
//     w_n3 = strtok(NULL,"-");
//     w_n4 = strtok(NULL," ");
//     sscanf (w_n1,"%d",&numPt[0]);
//     sscanf (w_n2,"%d",&numPt[1]);
//     sscanf (w_n3,"%d",&numPt[2]);
//     sscanf (w_n4,"%d",&numPt[3]);
//     //printf (" %d %d %d %d %d \n",i, numPt[0], numPt[1], numPt[2], numPt[3]);
//     free(name_joint);
//   }
//   torPt->njoint = n_j;
// 	/*     
// 	       if (length == 0)
// 	       printf (" name of joint is not completed \n");
// 	number++;
// 	sscanf (w_n,"%d",&numPt[number]);
// 	printf (" number %d \n", numPt[number]);
// 	for (i=0; i <= MAX_SIZE_NUMBER; i++){
// 	  w_n[i]= " ";
// 	}
// 	l=0;
//  	*/
// }


/**
 * @brief The memory for the outFileName has to be allocated by the user ! It should be large enough to contain the trailing '\0'
 * @param fullFileName 
 * @param outFileName 
 */
void GetFileNameWithPathWithoutExtension( char *fullFileName, char **outFileName )
{
	char *position, *tempPtr;
	int size;
	tempPtr = *outFileName;
	
	if ( fullFileName == NULL)
	{
		*outFileName[0] = '\0';
		return;
	}

	position = (char*)rindex(fullFileName, '.');
	
	if ( position != NULL )
	{
		size = position - fullFileName;
		strncpy( tempPtr, fullFileName, size);
		tempPtr[ size ] = '\0';
	}
	else
	{	//Since we did not find any dot . in the name, we simply return all the filename
		strcpy( tempPtr, fullFileName);
	}
}
