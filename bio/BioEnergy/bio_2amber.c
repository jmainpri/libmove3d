// Description: 
//   This file contains all the methods that have been created for the interaction to AMBER

#include "include/bio_2amber.h"




                                  /*************************/
void 
AMBER_addPath( STRING dir_name )
{
  /* version of oCmd_addPath fuction */

  BasicsAddDirectory( dir_name, 0 );        
}
 
			            /********************/
OBJEKT 
AMBER_add_source(char *filename)
{
   /* version of oCmd_source function */

  FILE            *fCmds;

    fCmds = FOPENCOMPLAIN( filename, "r" );
    if ( fCmds != NULL ) {
      if ( bINPUTMAXDEPTH() ) {
            VP0(( "Source commands are nested too deep!\n" ));

        } else {
                /* Push the file onto the input file stack.  The main */
                /* parsing routine will continue parsing until there are */
                /* no more files on the input file stack. */
                
                VP0(( "----- Source: %s\n", GsBasicsFullName ));
                INPUTPUSHFILE( fCmds );
                VP0(( "----- Source of %s done\n", GsBasicsFullName ));
        }
     }
    return(NULL);
}
                                  /************************/
		    
OBJEKT 
AMBER_loadPdb(  FILE *fPdb)
{
  /*
   * version of function oCmd_loadPdb  
   * Read the PDB file and create the UNIT 
   */

  UNIT          uUnit;
  
  if ( fPdb == NULL ) return(NULL);
 
  uUnit = uPdbRead( fPdb, NULL );
  return((OBJEKT)uUnit);
}


   
OBJEKT AMBER_loadPrep(char*file_prep)
{
  /* version of function  oCmd_loadAmberPrep */

  DICTIONARY      dUnits;
  DICTLOOP        dlLoop ;
  UNIT            uUnit = NULL;
  STRING          sName;
  LOOP            lResidues;
  RESIDUE         rRes;

  /* Read an AMBER PREP file into a dictionary.*/

  dUnits = dAmberReadPrepFile(file_prep);

  if ( dUnits != NULL ) {
    dlLoop = ydlDictionaryLoop( dUnits );

      /* for move3d version only one UNIT with one residue loaded */

    uUnit=(UNIT)yPDictionaryNext( dUnits, &dlLoop );
    strcpy( sName, sContainerName((CONTAINER) uUnit) );
    printf( "Loaded UNIT: %s\n", sName );

                /* Set the name for the UNIT */

    ContainerSetName( (CONTAINER) uUnit, sName );

                /* Set the name for the only residue in the unit */

    lResidues = lLoop( (OBJEKT)uUnit, RESIDUES );
    rRes = (RESIDUE)oNext(&lResidues);
    ContainerSetName( (CONTAINER) rRes, sName );
    VariableSet( (char*)sName, (OBJEKT) uUnit );       /* adds 1 REF */
    
    if (uUnit != NULL)
      return((OBJEKT)uUnit);
    else{
      printf ("-- no UNIT  loaded \n");
      return(NULL);
    }
  }
  else
    return(NULL);
}



/**
 * @brief Method for loading a Mol2 file as an Amber UNIT
 * This method was imported from amber/src/leap/src/leap/commands.c file and had an initial name of "oCmd_loadMol2"
 * @param file_prep Name of file to be loaded
 * @return An UNIT object
 */
OBJEKT AMBER_loadMol2( char *mol2FileName )
{
	FILE            *fMol2;
	UNIT            uUnit;

	if ((fMol2 = fopen(mol2FileName, "r")) == NULL) 
	{
		return NULL;
	}

	printf("\nLoading Mol2 file: %s\n", mol2FileName);
	uUnit = uTriposReadUnit( fMol2 );
	fclose(fMol2);
	
	return((OBJEKT)uUnit);
}


     
                               /***********************/
void 
AMBER_saveParm(UNIT    uUnit,char *name_first )
{  
  /* version of function oCmd_saveAmberParm */

  char   file_parm[1000];
  char   file_coord[1000]; 
  FILE *fparm, *fcoord;

  /* files names */

  strcpy (file_parm, name_first);
  strcpy (file_coord, name_first);
  strcat(file_parm,".parm");
  strcat(file_coord,".coord");
   
  if ( iParmLibSize(GplAllParameters) == 0 ) {
    printf( "There are no parameter sets loaded\n\n");
  }   
  fparm = FOPENCOMPLAIN(file_parm , "w" );
  if ( fparm == NULL ) 
    printf( " Could not open file: parm \n"); 
  
  fcoord = FOPENCOMPLAIN( file_coord, "w" ); 
  if ( fcoord == NULL ) 
    printf( "Could not open file: coord \n"); 
  
  TurnOffDisplayerUpdates();
  UnitSaveAmberParmFile( uUnit, fparm, fcoord, 
			 GplAllParameters, FALSE, FALSE );
  TurnOnDisplayerUpdates();

  fclose( fparm );
  fclose( fcoord );
}
                                      /********************/

OBJEKT
AMBER_loadParams(char *frcmod)
{
  // version of function  oCmd_loadAmberParams

   PARMSET         psParms;
   FILE            *fIn;

   fIn = FOPENCOMPLAIN( frcmod, "r" );
    if ( fIn == NULL )
        return(NULL);
    psParms = psAmberReadParmSet( fIn, frcmod);
    if ( psParms != NULL ) {
        ParmLibAddParmSet( GplAllParameters, psParms );
        ParmLibDefineDefault( GplAllParameters );
    } else
        printf( "-- no parameters loaded" );
    return((OBJEKT)psParms);
}

                                   /*************************/

void
AMBER_savePdb( OBJEKT objProt, char *name_first)
{
  /* 
   *  version of function  oCmd_savePdb
   *  creates .pdb file with all atoms including H-s  
   */
  
  FILE *fOut;
  char *file_pdb = NULL;

  strcpy (file_pdb, name_first);
  strcat(file_pdb,".pdb");

  fOut = FOPENCOMPLAIN(file_pdb,"w");
 
  if ( fOut != NULL ){
    PdbWrite( fOut, (UNIT) objProt );
    fclose( fOut );
  }else 
    printf ("--- no protein.pdb created");
}

OBJEKT 
AMBER_Combine(OBJEKT  objProt, OBJEKT objLig)
{
  // version of function oCmd_combine 

  UNIT            uCombined, uCurrent;
  LOOP            lTemp;
  RESIDUE         rRes = NULL;
  OBJEKT          oObj;


  /* add only one uCurrent unit (Ligand) to uCombined unit (Protein)  */

    uCombined = (UNIT)objProt;
    oObj = objLig;
    if ( iObjectType( oObj ) != UNITid ) {
      printf( "%s:  is type %s\n", 
	      "Ligand", sObjectType(oObj) );
    }
    uCurrent = (UNIT)objLig;
    printf( "  Ligand: %s\n", sContainerName((CONTAINER) uCurrent) );

    if ( uCombined == NULL ) {
      printf( "--- no Protein unit \n" );
    } else {
       printf( "Joining Protein and Ligand \n" );
      UnitJoin( uCombined, uCurrent );
    }

    if ( uCombined == NULL ) {
        printf( "No UNITS, so no combine performed\n" );
        return(NULL);
	}   

              /* Define PDB sequence */
     
    lTemp = lLoop( (OBJEKT)uCombined, RESIDUES );
    while ( rRes == (RESIDUE)oNext(&lTemp) ) {
        ResidueSetPdbSequence( rRes, iContainerSequence((CONTAINER) rRes) );
    }
   
    return((OBJEKT)uCombined);
}


