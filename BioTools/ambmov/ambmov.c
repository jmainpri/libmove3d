#include <malloc.h>
#include <string.h>

#include "ambmov.h"

#define PrintInfo(p) { (void)printf("\E[34m\E[7m"); (void)printf p;(void)printf("\E[0m\n"); }
#define PrintError(p) { (void)printf("\E[31m\E[7m"); (void)printf p;(void)printf("\E[0m\n"); }

#define DEFAULT_MINIMIZATION_CYCLES 10

//Fortran methods correspondence:
// readmemorysizesfromparmfile_ = read_inp_,
//readfromparmfile_ = read_parm_,
// fortrancodeenergyminimisation_ 

extern void readmemorysizesfromparmfile_( int *lastr,int *lasti,int *lasth,int *lastpr,int *lastrst,int *lastist, char *name, int *len_mane, int *keepLigandFixedDuringMinimization);
//extern void readmemorysizesfromparmfile_( int *lastr,int *lasti,int *lasth,int *lastpr,int *lastrst,int *lastist, char *name, int *len_mane, char *fixed_atoms_mask, int *mask_len);
extern void readfromparmfile_(double *px, int *pix,STRING *pih, int *pipairs,double *pr_stack,int *pi_stack);
extern void fortrancodeenergyminimisation_(double *px_coo, double *px, int *pix, STRING *pih, int *pipairs,
			double *pr_stack,int *pi_stack,double *penergy, double *pm_coo, int *pnumber_at);
extern   void writecoord_(double *x_coo, int *number_att); 
extern   void rdparm2_(double *px, int *pix,char *pih, int *pipairs,double *pr_stack, int *num_file, int *pi_stack);
extern   int removeH (char *pdbfilename ); 

// ParseInit and Basics -- functions of AMBER in AMBER_SRC/leap/src/leap
	     
extern void    ParseInit( RESULTt *rPResult );
extern void    BasicsInitialize();



OBJEKT         AMBER_add_source(char *filename);
OBJEKT         AMBER_loadPdb(FILE *fPdb);
OBJEKT         AMBER_Combine(OBJEKT  objProt, OBJEKT objLig);
OBJEKT         AMBER_loadParams( );
OBJEKT         AMBER_loadPrep(char *file_prep);
OBJEKT         AMBER_loadMol2( char *mol2FileName );
void           AMBER_saveParm(UNIT    uUnit);
void           AMBER_addPath( STRING dir_name );
void           AMBER_savePdb( OBJEKT objProt );

OBJEKT         objProt   = NULL;
OBJEKT         objLig    = NULL;
OBJEKT         ParmsLig  = NULL;
OBJEKT         ProtLig   = NULL;
RESULTt	       rResult;

int            keepLigandFixedDuringMinimization = 0;	//We do not know yet if we have a ligand or not
int	       useMol2InsteadOfPrep = 1;	//Implicitly use Mol2 as intermediary format
int            number_at = 0;
double         energy [51];
double         *px_coo   = NULL ;
PARAMET        *parmet   = NULL;


char file_parm [NB_OF_CHAR];
char file_coord [NB_OF_CHAR];
char ipfilename[NB_OF_CHAR];
char ilfilename[NB_OF_CHAR];
char ofilename[NB_OF_CHAR];
char file_delH[NB_OF_CHAR];
char *proteinFileNameNoExt;

FILE *fPDBwithoutH = NULL;
FILE *fLigand = NULL;
FILE *fout = NULL;
FILE *fOut_parm = NULL;
FILE *fCrd_coord = NULL;



void DoPause()
{
	char unused[100];
	PrintInfo(("Pause... Type a character and then Enter to continue ..."));scanf("%s", unused);
}


void load_force_fields( )
{ 
					/************************/			   
	/* For LIGAND leaprc.gaff is used ("ff" force field).
	To specify force field for PROTEIN a link is used. For example:
	cd $AMBERHOME/dat/leap/cmd
	ln -s leaprc.ff03 leaprc.
	Then leaprc.ff03 will become the default.*/ 
	
	char          *flibLig = "leaprc.gaff"; 
	char          *system_env;
	char          tmpchar[NB_OF_CHAR];
	int           i;
	char *prep = "/dat/leap/prep";
	char *lib =  "/dat/leap/lib";
	char *parm= "/dat/leap/parm";
	char *cmd= "/dat/leap/cmd";
	char          *lib_path[NB_OF_LIB] =  {prep,lib,parm,cmd};
	
	BasicsInitialize();
	system_env = (char *) getenv("AMBERHOME");
	if (system_env != NULL) 
	{
		for(i = 0; i < NB_OF_LIB ; i++)
		{
			tmpchar[0] = '\0';
			strcpy(tmpchar, system_env);
			strcat(tmpchar, lib_path[i]);
			AMBER_addPath(tmpchar);
		}
	}
	
	AMBER_add_source(flibLig);
	
	ParseInit(&rResult);
}



void save_pdb_min ( )
{
	char str_exe[NB_OF_CHAR];
	char cmdToExecute[NB_OF_CHAR*3];
	char *system_env;
	
	system_env = (char *) getenv("AMBERHOME");
	if (system_env != NULL) 
	{
		str_exe[0] = '\0';
		strcpy(str_exe,system_env);
		strcat(str_exe, "/exe/ambpdb");
	}
	else
	{
		PrintError(("\E[31m system variable $AMBERHOME is not defined \n"));
		exit(2);
	}
	
	sprintf(cmdToExecute,"%s -p %s.parm < %s.restrt > %s ", str_exe, proteinFileNameNoExt, proteinFileNameNoExt, ofilename );
	
	PrintInfo(("\n\n Executing: %s  \n", cmdToExecute));
	system( cmdToExecute );
}


void load_ligand_pdb(FILE *flinp)
{  
	char antechamber_tmpchar[NB_OF_CHAR];
	char parmch_tmpchar[NB_OF_CHAR];
	char strchar[NB_OF_CHAR];
	char strrchar[NB_OF_CHAR];
	char tempLigandFile[NB_OF_CHAR];
	char file_frcmod[NB_OF_CHAR];
	char extension[10], format[10];
	FILE *fprep;
//	FILE *fparm;
	char *system_env;
	char *file_lig;
	char file_pdb[NB_OF_CHAR];
	char commandToExecute[200];
	
	system_env = (char *) getenv("AMBERHOME");
	if (system_env != NULL) 
	{
		antechamber_tmpchar[0] = '\0';
		parmch_tmpchar[0]= '\0';
		strcpy(antechamber_tmpchar, system_env);
		strcpy(parmch_tmpchar, system_env);
		strcat(antechamber_tmpchar, "/exe/antechamber");
		strcat(parmch_tmpchar, "/exe/parmchk");
	}
	else
	{
		PrintError( ("\E[31m system variable $AMBERHOME is not defined \n"));
		exit(2);
	}

	if ( ! useMol2InsteadOfPrep )
	{
		strcpy(format, "prepi");
		strcpy(extension, ".prep");
	}
	else
	{
		strcpy(format, "mol2");
		strcpy(extension, ".mol2");
	}

	
	file_lig = strtok (ilfilename,".");
	strcpy (tempLigandFile, file_lig);
	strcpy (file_frcmod, file_lig);
	strcpy (file_pdb, file_lig);
	strcat(tempLigandFile,extension);
	strcat(file_frcmod,".parms");
	strcat(file_pdb,".pdb");
	
	fprep = fopen(tempLigandFile,"r");
	if(fprep == NULL) 
	{
		sprintf(strchar,"%s -i %s -fi pdb -o %s -fo %s -c mul -pf y -s 2", antechamber_tmpchar,file_pdb,tempLigandFile, format );
		PrintInfo(("\n\nExecuting: %s ", strchar));
		system(strchar);   
		sprintf(strrchar, "%s -i %s -f prepi -o %s",parmch_tmpchar,tempLigandFile,file_frcmod ) ;
		PrintInfo(("\n\nExecuting: %s ", strrchar));
		system(strrchar);
		PrintInfo(("\n\n ... done."));
	}
	else
	{
		PrintInfo(("\n\n File %s already exists, so I will NOT execute <antechamber> and <parmchk>.", tempLigandFile));
		fclose(fprep);
	}

	strcpy( commandToExecute, "rm -f ANTECHAMBER* ATOMTYPE.INF BCCTYPE.INF NEWPDB.PDB PREP.INF divcon.* leap.log");
	PrintInfo(("\n\nExecuting: %s ", commandToExecute));
	system( commandToExecute );

	

	if ( ! useMol2InsteadOfPrep )
	{
		objLig = AMBER_loadPrep(tempLigandFile);    
	}
	else
	{
		objLig = AMBER_loadMol2(tempLigandFile);    
	}
	ParmsLig = AMBER_loadParams(file_frcmod);
}

 

static double* read_coordinates(int *pt_number_at)
{
	char coo_line[NB_OF_CHAR];
	int n_line = 0;
	FILE *fcoo;
	double *x_coo = NULL;
	char temp[NB_OF_CHAR];
	int i,j;
	
	fcoo = fopen (file_coord, "r");
	
	if ( fcoo != NULL ) 
	{
		while (fgets(coo_line,1000,fcoo)) 
		{
			i = n_line -1;
			if (i < 0)
			{
				sscanf(coo_line,"%s", temp);
			}
			else if (i == 0)
			{
				sscanf(coo_line,"%d", pt_number_at);
				if (*pt_number_at == 0 ) 	
					return(NULL);
				x_coo = (double*) malloc(*pt_number_at * 3 * sizeof(double));
			}
			else if (i > 0)
			{
				j = (n_line-2)*6;
				sscanf(coo_line,"%lf %lf %lf %lf %lf %lf",&x_coo[j],&x_coo[j+1], &x_coo[j+2],
					&x_coo[j+3],&x_coo[j+4], &x_coo[j+5]);
			}
			n_line++;
		}
		fclose (fcoo);
		return x_coo;
	}
	else 
	{
		PrintError(( "-- no file coord \n"));
	}
	return (NULL);
}  


void energy_init ()
{
	/*
	*  readmemorysizesfromparmfile_ --  fortran function reads first part of parm file, calculates size memory
	*  readfromparmfile_ -- fortran function reads second part of parm file after allocation of memory 
	*/ 
	//  int num_file = 8;
	int len_name = 0;
	int lastr = 1;
	int lasti = 1;
	int lasth = 1;
	int lastpr = 1;
	int lastrst = 1;
	int lastist = 1;
//	int atomsMaskLength=0;
	char *coordFileName, *refcFileName, sysCommand[500];
	
	len_name = strlen(proteinFileNameNoExt );
	PrintInfo(("\n\n Reading memory sizes from file \n")); 
//	readmemorysizesfromparmfile_(&lastr,&lasti,&lasth,&lastpr,&lastrst,&lastist, proteinFileNameNoExt, &len_name, NULL, &atomsMaskLength);
	readmemorysizesfromparmfile_(&lastr,&lasti,&lasth,&lastpr,&lastrst,&lastist, proteinFileNameNoExt, &len_name, &keepLigandFixedDuringMinimization);

	//DoPause();	
	
	parmet = (PARAMET*) malloc (sizeof(PARAMET));
	parmet->px = (double*) malloc( (lastr) * sizeof(double));
	parmet->pix = (int*) malloc( (lasti) * sizeof(int));
	parmet->pih = (STRING*) malloc( (lasth) * sizeof(STRING));
	parmet->pipairs = (int*) malloc( (lastpr) * sizeof(int));
	parmet->pr_stack = (double*) malloc( (lastrst) * sizeof(double));
	parmet->pi_stack = (int*) malloc( (lastist) * sizeof(int));

	//Actually, here, before starting the reading of the parm file
	// we have to copy the ".coord" file into the ".refc" file because
	// we need the ligand to be kept fixed
	coordFileName = (char*)malloc( len_name + 10 );
	refcFileName = (char*)malloc( len_name + 10 );
	strcpy( coordFileName, proteinFileNameNoExt );
	strcat( coordFileName, ".coord" );
	strcpy( refcFileName, proteinFileNameNoExt );
	strcat( refcFileName, ".refc" );
	strcpy( file_coord, coordFileName);

	sprintf(sysCommand, "cp %s %s", coordFileName, refcFileName);
	PrintInfo(("\n\n Executing: %s \n", sysCommand)); 
	system( sysCommand );
	
	
	PrintInfo(("\n\n Reading from the parm file \n")); 
	//rdparm2_(parmet->px,parmet->pix,parmet->pih,parmet->pipairs,parmet->pr_stack,&num_file,parmet->pi_stack);
	readfromparmfile_(parmet->px,parmet->pix,parmet->pih,parmet->pipairs,parmet->pr_stack,parmet->pi_stack);

	// stef
	//PARMpt = &PARM;
}


void energy_calcul (double *px_coo, double *energy, int number_at )
{

	// m_coo -- coordinates of atoms after minimization
	double *pm_coo = NULL;
	
	pm_coo = (double*) malloc(number_at * 3 * sizeof(double)); 
	// fortrancodeenergyminimisation_ -- fortran function to run energy calculation, minimization calculations or MD. 
	
	PrintInfo(("\n\n Starting energy minimization: "));

	fortrancodeenergyminimisation_(px_coo, parmet->px,parmet->pix,parmet->pih,parmet->pipairs,parmet->pr_stack, parmet->pi_stack, 
		energy, pm_coo, &number_at);
	
	PrintInfo(("\n\n Finished minimization: "));
} 

void PARAMET_free ( PARAMET *PARM )
{
  free(PARM->px);
  free(PARM->pix);
  free(PARM->pih);
  free(PARM->pipairs);
  free(PARM->pr_stack);
  free(PARM->pi_stack);

  free (PARM);
}


void set_steepest_descent_conjugate_gradient( void ){
  mdi_.imin = 1;
  mdi_.ntmin =1;
}
void set_conjugate_gradient( void ){
  mdi_.imin = 1;
  mdi_.ntmin = 0;
}
void set_steepest_descent  ( void ){
  mdi_.imin = 1;
  mdi_.ntmin = 2;
}
void interactions_omission (int NTF_amber)
{
	/*
	*NTF_amber 
	* = 1 complete interaction is calculated (default);
	* = 2 bond interactions involving H-atoms omitted, 
		use with NTC=2 -- bonds involving hydrogen are constrained;
	* = 3 all the bond interactions are omitted,
		use with NTC=3 -- all bonds are constrained;
	* = 4 angle involving H-atoms and all bonds are omitted;
	* = 5 all bond and angle interactions are omitted;
	* = 6 dihedrals involving H-atoms and all bonds and all angle interactions are omitted;
	* = 7 all bond, angle and dihedral interactions are omitted;
	* = 8 all bond, angle, dihedral and non-bonded interactions are omitted;  
	*/
	
	mdi_.ntf = NTF_amber;
	if (NTF_amber == 2) mdi_.ntc = 2;
	if (NTF_amber == 3) mdi_.ntc = 3;
}
void set_molecular_dynamics ( void ){ 
  mdi_.imin = 0;
}
void set_number_maxcyc (int number_maxcyc ) {
  mdi_.maxcyc = number_maxcyc;
}
void set_number_ncyc (int number_ncyc ) {
  mdi_.ncyc = number_ncyc;
}
int get_number_maxcyc (void) {
  return  mdi_.maxcyc;
}
int get_number_ncyc (void) {
 return  mdi_.ncyc;
}


void energy_cal( )
{  
	double         *px_coo   = NULL ;
	//   double         *px_coo_pdb   = NULL ;
	int number_maxcyc = 0;
	//   int i;
	
	set_number_maxcyc(number_maxcyc);
	px_coo = read_coordinates(&number_at); 
	energy_calcul (px_coo, energy, number_at ); 
	printf ("\nTotal energy of molecule = %f \n", energy[0]);
	printf ("Bond energy of molecule = %f \n", energy[5]);
	printf ("Angle energy of molecule = %f \n", energy[6]);
}

/*************************/

OBJEKT AMBER_loadPdb(  FILE *fPdb)
{
	UNIT          uUnit;
	/* Read the PDB file and create the UNIT */
	uUnit = uPdbRead( fPdb, NULL );
	return((OBJEKT)uUnit);
}


		            /********************/
OBJEKT AMBER_add_source(char *filename)
{
	FILE            *fCmds;
	fCmds = FOPENCOMPLAIN( filename, "r" );
	if ( fCmds != NULL ) 
	{
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

OBJEKT AMBER_loadPrep(char *file_prep)
{
	DICTIONARY      dUnits;
	DICTLOOP        dlLoop ;
	UNIT            uUnit = NULL;
	STRING          sName;
	LOOP            lResidues;
	RESIDUE         rRes;
	
	/* Read an AMBER PREP file into a dictionary.*/
	dUnits = dAmberReadPrepFile(file_prep);
	if ( dUnits != NULL )
	{
		dlLoop = ydlDictionaryLoop( dUnits );
		uUnit=(UNIT)yPDictionaryNext( dUnits, &dlLoop );
		strcpy( sName, sContainerName((CONTAINER) uUnit) );
		PrintInfo(( "Loaded UNIT: %s", sName ));
		/* Set the name for the UNIT */
		ContainerSetName( (CONTAINER) uUnit, sName );
		/* Set the name for the only residue in the unit */
		lResidues = lLoop( (OBJEKT)uUnit, RESIDUES );
		rRes = (RESIDUE)oNext(&lResidues);
		ContainerSetName( (CONTAINER) rRes, sName );    
		VariableSet( sName, (OBJEKT) uUnit );       /* adds 1 REF */
		return((OBJEKT)uUnit);
	}
	if (uUnit == NULL)
	PrintError( ("-- no UNIT  loaded \n"));
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
		PrintError(("\n Cannot open mol2 file %s, exiting ... \n", mol2FileName));
		exit(0);
	}

	PrintInfo(("Loading Mol2 file: %s", mol2FileName));		
	uUnit = uTriposReadUnit( fMol2 );
	fclose(fMol2);
	
	return((OBJEKT)uUnit);
}

                               /***********************/
void AMBER_saveParm(UNIT    uUnit )
{  
  strcpy (file_parm, proteinFileNameNoExt);
  strcpy (file_coord,proteinFileNameNoExt);
  strcat (file_parm,".parm");
  strcat (file_coord,".coord");

    if ( iParmLibSize(GplAllParameters) == 0 ) {
     PrintError(( "There are no parameter sets loaded\n\n"));
        }   
     fOut_parm = FOPENCOMPLAIN( file_parm , "w" );
    
     if ( fOut_parm == NULL ) 
       PrintError(( " Could not open file: parm \n")); 
                
     fCrd_coord = FOPENCOMPLAIN( file_coord, "w" ); 
     if ( fCrd_coord == NULL ) {
       PrintError(( "Could not open file: coord \n")); 
       fclose( fOut_parm );
     }
     TurnOffDisplayerUpdates();
     UnitSaveAmberParmFile( uUnit, fOut_parm, fCrd_coord, GplAllParameters, FALSE, FALSE );
     TurnOnDisplayerUpdates();
     fclose( fOut_parm );
     fclose( fCrd_coord );
}
                                      /********************/

OBJEKT AMBER_loadParams(char*frcmod)
{
	PARMSET         psParms;
	FILE *fparm;
	
	fparm = fopen (frcmod, "r");
	if ( fparm == NULL )
		return(NULL);
	
	psParms = psAmberReadParmSet( fparm,frcmod);
	if ( psParms != NULL ) 
	{
		ParmLibAddParmSet( GplAllParameters, psParms );
		ParmLibDefineDefault( GplAllParameters );
	} else 
	{
		PrintError(( "-- no parameters loaded" ));
	}
	return((OBJEKT)psParms);
}

                                   /*************************/

OBJEKT AMBER_Combine(OBJEKT  objProt, OBJEKT objLig)
{
	UNIT            uCombined, uCurrent;
	LOOP            lTemp;
	RESIDUE         rRes = NULL;
	OBJEKT          oObj;
	uCombined = (UNIT)objProt;
	oObj = objLig;
	if ( iObjectType( oObj ) != UNITid ) 
	{
		PrintInfo(( "%s:  is type %s", "Ligand", sObjectType(oObj) ));
	}
	uCurrent = (UNIT)oCopy(oObj );
	PrintInfo(( "  Sequence: %s", sContainerName((CONTAINER) uCurrent) ));
	
	if ( uCombined == NULL ) 
	{
		printf( "Copying the first UNIT\n" );
		uCombined = uCurrent;
	} else 
	{
		printf( "Copied a subsequent UNIT\n" );
		printf( "Joining two UNITS \n" );
		UnitJoin( uCombined, uCurrent );
	}
	
	if ( uCombined == NULL ) 
	{
		printf( "No UNITS, so no combine performed\n" );
		return(NULL);
	}
	lTemp = lLoop( (OBJEKT)uCombined, RESIDUES );
	while ( rRes == (RESIDUE)oNext(&lTemp) ) 
	{
		ResidueSetPdbSequence( rRes, iContainerSequence((CONTAINER) rRes) );
	}
	
	return((OBJEKT)uCombined);
}

                                    /***************************/
void AMBER_addPath( STRING dir_name )
{
	BasicsAddDirectory( dir_name, 0 );
}
                                    /***************************/


/**
 * This method does (as its name suggests) the following:
 *  Initialize all the necessary structures so that an energy calculatio is possible
 *  and then launch this calculation.
 * @param nameOfPDBFile Name of the file for which the energy will be displayed
 */
void DisplayEnergyForThisPDBFile( char* nameOfPDBFile )
{
	FILE *fp;
	char* nameCopy = strdup( nameOfPDBFile );
	
	proteinFileNameNoExt = strtok( nameCopy,"." );

	PrintInfo(("\n\n Loading force fields: "));
	PrintInfo(("WARNING !! The energy calculation does NOT work with ligands !!! "));
	load_force_fields( );

	PrintInfo(("\n\n Loading PDB: "));
	if ((fp = fopen(nameOfPDBFile, "r")) == NULL)
	{
		PrintError(("\n Cannot open the given file ! \n", file_delH));
		return;
	}

	objProt = AMBER_loadPdb ( fp );
	fclose(fp);

	PrintInfo(("\n\n saving PARM: "));
	AMBER_saveParm( (UNIT) objProt);

	PrintInfo(("\n\n Energy Init: "));
  	energy_init ( );
  	px_coo = read_coordinates(&number_at);

	set_steepest_descent_conjugate_gradient(); 
	set_number_maxcyc( 0 );
	energy_calcul (px_coo, energy, number_at );

	PARAMET_free(parmet);

	PrintInfo(("\n\nFile: %s\nEnergy: %f \n", nameOfPDBFile, energy[0]));

}


int main(int argc, char *argv[])
{
	int i, nbMinimizationCycles = DEFAULT_MINIMIZATION_CYCLES;
	FILE *fout;
	
	if ( argc <= 2 )
	{
		printf("Usage: ambmov -ip <pdbfile for protein> -il <pdbfile for ligand> -o <out pdbfile>\n");
		printf("  Optional parameters:                                                            \n");
		printf("              -n <nb minimization cycles (10)>                                    \n");
		printf("              -prep   : use \"prep\" intermediary file instead of \"mol2\"        \n");
		printf("                                                                                  \n");
		printf("  Ambmov can also be used for computing the energy of a given PDB file:     \n");
		printf("        ambmov -energy <pdbFile>                                              \n\n");

		exit(0);
	}
	//First parse the parameters. Look for the 3 needed files
	for (i = 1; i < argc; i += 2) 
	{
		if (strcmp(argv[i], "-energy") == 0)
		{
			//Store the name of the PDBfile in some variable
			strcpy(ipfilename, argv[i + 1]);
			//Let the user know what we are doing:
			PrintInfo(("Received -energy parameter. The program will exit after having displayed the energy !"));
			//Do the actual thing
			DisplayEnergyForThisPDBFile( ipfilename );
			//And then keep our promise and exit
			exit(0);
		}
		if (strcmp(argv[i], "-ip") == 0)
		{
			strcpy(ipfilename, argv[i + 1]);
			//Remove the hydrogens from the initial PDB
			removeH (ipfilename);
			//Compute the name of the pdb without the hydrogens
			proteinFileNameNoExt = strtok( ipfilename,"." );
			strcpy(file_delH,proteinFileNameNoExt);
			strcat(file_delH,"_delH.pdb");
			fPDBwithoutH = fopen(file_delH, "r");
			//And verify that the pdb file without hydrogens does exist
			if ((fPDBwithoutH = fopen(file_delH, "r")) == NULL) 
			{
				PrintError(("\n Cannot open pdb withot Hs file %s, exit \n", file_delH));
				exit(1);
			}
		} 
		if (strcmp(argv[i], "-il") == 0)
		{
			/* FRAG or FRA in the place of residue label in ligand pdb file
				and UNIT FRA in prep file
				to restrain all atoms of ligand in the initial positions */
			strcpy(ilfilename, argv[i + 1]);
			if ((fLigand = fopen(ilfilename, "r")) == NULL) 
			{
				PrintError(("\n Cannot open ligand pdb file %s, exit \n", ilfilename));
				exit(1);
			}
		}
		if (strcmp(argv[i], "-o") == 0)
		{
			strcpy(ofilename, argv[i + 1]);
		}
		if (strcmp(argv[i], "-n") == 0)
		{
			//Try to read the nb of minimization cycles from the given argument
			if (sscanf( argv[i + 1], "%d", &nbMinimizationCycles) != 1)
			{	//If the reading fails, set the default nb
				nbMinimizationCycles = DEFAULT_MINIMIZATION_CYCLES;
			}
		}
		if (strcmp(argv[i], "-prep") == 0)
		{
			useMol2InsteadOfPrep = 0;
		}
	}	//for (i=1; ...
	
	//Now that we have finished reading all the parameters, 
	// we have also opened the input files (protein and ligand)
   
	PrintInfo(("\n\n Loading force fields: "));
	load_force_fields( );

	PrintInfo(("\n\n Loading PDB: "));
	objProt = AMBER_loadPdb ( fPDBwithoutH );
	fclose(fPDBwithoutH);
	PrintInfo(("\n\n finished loading PDB: "));


	if (fLigand != NULL )
	{
		load_ligand_pdb (fLigand);
		fclose(fLigand);
		ProtLig = AMBER_Combine (objProt, objLig);
		keepLigandFixedDuringMinimization = 1;	//We have a ligand, so keep it fixed
		PrintInfo(("\n\n saving PARM for the Protein Ligand combination: "));
		AMBER_saveParm( (UNIT) ProtLig); 
	}
	else
	{
		PrintInfo(("\n\n saving PARM: "));
	    	AMBER_saveParm( (UNIT) objProt); 
	}
 
	PrintInfo(("\n\n Energy _ init: "));
  	energy_init ( ); 
	
	PrintInfo(("\n\n Reading coordinates: "));
  	px_coo = read_coordinates(&number_at);

	set_steepest_descent_conjugate_gradient(); 

	set_number_maxcyc( nbMinimizationCycles );

	energy_calcul (px_coo, energy, number_at ); 

	save_pdb_min ( );

	// save total energy in file
	fout = fopen("totE.ambmovout", "w");
	fprintf(fout, "%f",energy[0]);
	fclose(fout);

	PARAMET_free(parmet);

	// printf ("total energy of molecule %f n", energy);

	PrintInfo(("\n\n Finished ... \n")); 

 return 0;
}


