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
#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#define FALSE 0
#define TRUE 1
#define MAX_LINE_OF_FILE_LENGTH 1000




void help(char* progName);
void ActualProcessing( FILE *pdbFile, FILE *nmFile, FILE *outFile, double amplitude );



int main (int argc, char* argv[])
{
  char op;
  char *pdbFileName = NULL, *nmFileName = NULL, *outFileName = NULL;
  FILE *pdbFile = NULL, *nmFile = NULL, *outFile = NULL;
  char tmpLineOfFile[MAX_LINE_OF_FILE_LENGTH];
  char vectorSymbol[6]; 	//This is only used for reading the "VECTOR" symbol from the file

	int currentVector = -1; //Has to be initialized with -1 because we do first a   +=1
	int foundNeededVector = FALSE;
	int whichVector = -1;
	double amplitude = 1.0;
	int paramLen = 0;
//   char readCmd[20];
//   ulong readOffset, readLength;
//   char Cmd;
//   int noOfParamsRead = 0, sourceType = -1;



  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //%% PARSE THE PARAMETERS
  if (argc == 1)
  {
    help( argv[0] );
    return 0;
  }

  while ((op = getopt(argc, argv, "ho:p:n:v:a:")) != EOF) switch(op)
  {
  case 'h':
    help(argv[0]);
    return 0;
    
  case 'o':
	paramLen = strlen(optarg);
	outFileName = (char*)malloc(paramLen);
	strncpy(outFileName, optarg, paramLen);
	outFileName[paramLen]='\0';
    break;

  case 'p':
	paramLen = strlen(optarg);
	pdbFileName = (char*)malloc(paramLen + 1);
	strncpy(pdbFileName, optarg, paramLen);
	pdbFileName[paramLen] = '\0';
    break;

  case 'n':
	paramLen = strlen(optarg);
	nmFileName = (char*)malloc(paramLen);
	strncpy(nmFileName, optarg, paramLen);
	nmFileName[paramLen]='\0';
    break;

  case 'v':
	if ( sscanf(optarg, "%d", &whichVector) != 1)
	{
		printf("Error while parsing the -v parameter !");
		exit(1);
	}
    break;

  case 'a':
	if ( sscanf(optarg, "%lf", &amplitude) != 1)
	{
		printf("Error while parsing the -a parameter !");
		exit(1);
	}
    break;

  default:
    fprintf(stderr, "\E[31m Parameter '%c' not recognized !\033[0m\n", op);
    help(argv[0]);
    goto ExitWithError;
  }

  if ( (outFileName == NULL) ||(pdbFileName == NULL) ||(nmFileName == NULL) )
  {
    fprintf(stderr, "\E[31m All 3 filenames must be provided !\033[0m\n");
    help(argv[0]);
    goto ExitWithError;
  }
	if ( whichVector == -1 )
	{
		fprintf(stderr, "\E[31m The vector number needs to be specified !\033[0m\n");
		help(argv[0]);
		goto ExitWithError;
	}
	if ( ( whichVector < 0 ) || ( whichVector > 55 ) )
	{
		fprintf(stderr, "\E[31m The vector number has to be between 0 and 55 !\033[0m\n");
		goto ExitWithError;
	}

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //%% OPEN FILES
	if ( (pdbFile = fopen(pdbFileName, "r")) == NULL)
	{
		fprintf(stderr, "Failed to open PDB file: %s (%s) \n", pdbFileName, strerror(errno));
		goto ExitWithError;
	}
	if ( (nmFile = fopen(nmFileName, "r")) == NULL)
	{
		fprintf(stderr, "Failed to open Normal Modes file: %s (%s) \n", nmFileName, strerror(errno));
		goto ExitWithError;
	}
	if ( (outFile = fopen(outFileName, "w")) == NULL)
	{
		fprintf(stderr, "Failed to open OUT file: %s (%s) \n", outFileName, strerror(errno));
		goto ExitWithError;
	}


  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //%% Position the file pointers

	//The PDB file needs not be positioned
	// The Normal Modes file needs to be positioned at the correct VECTOR
	while (fgets(tmpLineOfFile,MAX_LINE_OF_FILE_LENGTH,nmFile)) 
	{
		sscanf (tmpLineOfFile,"%6s", vectorSymbol);

		if (strncmp(vectorSymbol, "VECTOR", 6) == 0 )
		{	
			currentVector ++;
			if ( currentVector >= whichVector )
			{	//Exit from this loop
				foundNeededVector = TRUE;
				break;
			}
		}
	}
	if (! foundNeededVector )
	{
		fprintf(stderr, "\E[31m The needed vector (%d) could not be found. Please check the Normal Modes file !\033[0m\n", whichVector);
		goto ExitWithError;
	}

	printf("\E[34m Outputing to file: %s\E[0m\n", outFileName);

	//Do the actual processing (ie. pdb+nm => out )
	try
	{
		ActualProcessing( pdbFile, nmFile, outFile, amplitude );
	}
	catch( int someError)
	{
		fprintf(stderr, "\E[31m Some error occured: %s !\033[0m\n", strerror(someError) );
		goto ExitWithError;
	}


  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //%% CLEAN UP
  fclose(pdbFile);
  fclose(nmFile);
  fclose(outFile);
  if (nmFileName != NULL) free(nmFileName);
  if (outFileName != NULL) free(outFileName);
  if (pdbFileName != NULL) free(pdbFileName);

  return 0;

 ExitWithError:
  if (pdbFile != NULL) { fclose(pdbFile); }
  if (nmFile != NULL) { fclose(nmFile); }
  if (outFile != NULL) { fclose(outFile); }
  if (nmFileName != NULL) free(nmFileName);
  if (outFileName != NULL) free(outFileName);
  if (pdbFileName != NULL) free(pdbFileName);
  return 1;
}




//This method is called with all the file pointers correctly positioned
void ActualProcessing( FILE *pdbFile, FILE *nmFile, FILE *outFile, double amplitude )
{
	char tmpLineOfPdb[MAX_LINE_OF_FILE_LENGTH];
	char tmpLineOfNm[MAX_LINE_OF_FILE_LENGTH];
	char tmpStartOfLine[31], tmpEndOfLine[40];
	double pdbX, pdbY, pdbZ, nmX, nmY, nmZ;
	int currentPDBFileLine = 0;
	char atomSymbol[4]; 	//This is only used for reading the "ATOM" symbol from the file

	//In here all we need to do is read a line from the PDB file and a line from the nmFile
	// and then write their "sum" in the outFile

	while ( fgets(tmpLineOfPdb, MAX_LINE_OF_FILE_LENGTH, pdbFile)) 
	{
		currentPDBFileLine += 1;

		sscanf (tmpLineOfPdb,"%4s", atomSymbol);
		if (strncmp(atomSymbol, "ATOM", 4) != 0 )
		{
			//If this line from the PDB file does not contain a description of the atom, then 
			//  if we recognize the beginning, we just copy it in the output file
			if ( 	(strncmp(atomSymbol, "REMA", 4) == 0 ) ||
				(strncmp(atomSymbol, "TER", 3) == 0 ) ||
				(strncmp(atomSymbol, "END", 3) == 0 ) )
			{
				fprintf( outFile, tmpLineOfPdb );
			}
			else
			{
				fprintf(stderr, "\E[31m Unknown start of line in PDB file (%s) at line %d !\033[0m\n", atomSymbol, currentPDBFileLine);
			}
			continue;	//we ignore this line and continue to read the file
		}


		//Now that we are sure that we have a valid PDB file, read its information
		strncpy(tmpStartOfLine, tmpLineOfPdb, 30);
		tmpStartOfLine[30]='\0';
		sscanf(tmpLineOfPdb + 30, "%8lf%8lf%8lf", &pdbX, &pdbY, &pdbZ);
		strcpy(tmpEndOfLine, tmpLineOfPdb + 54 );

		//Debug print
		//printf("PDB:  %s%8.3lf%8.3lf%8.3lf \n", tmpLineOfPdb+30, pdbX, pdbY, pdbZ );


		//Now we can read the Normal Modes info also
		if (! fgets(tmpLineOfNm, MAX_LINE_OF_FILE_LENGTH, nmFile) ) 
		{
			printf("The Normal modes file ended sooner than expected !\n");
			throw(1);
		}

		sscanf(tmpLineOfNm,"%lf %lf %lf", &nmX, &nmY, &nmZ);
		//Debug print
		//printf("NM :  %s%17.8lf%17.8lf%17.8lf \n", tmpLineOfNm, nmX, nmY, nmZ );
	
		//Debug print
		//printf("OUT:  %s%8lf%8lf%8lf%s\n", tmpStartOfLine, pdbX + nmX, pdbY + nmY, pdbZ + nmZ, tmpEndOfLine );
		fprintf(outFile, "%s%8.3lf%8.3lf%8.3lf%s", tmpStartOfLine, pdbX + ( nmX * amplitude), pdbY + (nmY * amplitude), pdbZ + (nmZ * amplitude), tmpEndOfLine );

		//Debug print
		//printf("%10lf+%10lf=%10lf    %10lf+%10lf=%10lf    %10lf+%10lf=%10lf   \n", pdbX, nmX, pdbX + nmX, pdbY, nmY, pdbY + nmY , pdbZ , nmZ, pdbZ + nmZ);
	}

}



void help(char* progName)
{
  printf("  PDB + Normal Modes                          \n");
  printf("                                                     \n");
  printf(" --some more detailed description to be added -- \n");
  printf("                                                     \n");
  printf("Options:                                             \n");
  printf("  -o <file>  :specify the name of the output file    \n");
  printf("  -p <file>  :specify the name of the PDB input file. \n");
  printf("  -n <file>  :specify the name of the Normal Modes input file. \n");
  printf("  -v <nb>  :Which vector should be used from the Normal Modes file (.eigenrtb) \n");
  printf("  [-a <nb>]  :Amplitude to apply (eg. 2.333) \n");
  printf("                                                     \n");
  printf("  -h         :shows this help                        \n");
  printf("                                                     \n");
  printf("                                                     \n");
  printf("                                                     \n");
}
