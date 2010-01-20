#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"

#include "Util-pkg.h"
#include "Bio-pkg.h"

#include "include/bio_minimization.h"
#include "include/bio_2amber.h"
#include "include/amb_eln.h"
#include "include/bio_allocations.h"


/**********************************************************************************/

static void WriteCoordFile( char* fileName, double* atmCoords, int nb_atoms );

/**********************************************************************************/

void PutStressOnMemoryAllocation()
{
	//This method does nothing usefull, but it allocates a lot of memory in the hope
	//  that this will reveal some allocation faults.
	long amount = 10 * 1000 * 1000;
	long onePacketSize = 1000;
	//The configuration of the above parameters should suffice

	typedef struct piggy_struct{
	unsigned long id;
	void *mem;
	struct piggy_struct *next;
	}piggy;

	long soFar = 0;
	piggy *newPiggy=NULL, *headPiggy=NULL;

	//** Actual start of code

	PrintInfo(("Starting PutStressOnMemoryAllocation() ... \n"));

	//Initialize the headPiggy
	headPiggy = malloc( sizeof(piggy) );
	if (headPiggy == NULL) 
	{
		PrintError(("PutStressOnMemoryAllocation failed: Could not allocate the headPiggy!\n"));
		return;
	}
	headPiggy->next = NULL;
	headPiggy->id = 0;
	headPiggy->mem = NULL;//No load for this piggy

	while ( soFar < amount )
	{
		//allocate a new piggy
		newPiggy = malloc( sizeof(piggy) );
		if (newPiggy == NULL) 
		{
			PrintError(("PutStressOnMemoryAllocation failed: Could not allocate a new piggy with id %lu!\n", headPiggy->id + 1));
			//We will not "return" here because we still have to deallocate all the other piggies.
			break;
		}	
		//Allocate the load for this piggy
		newPiggy->mem = malloc( onePacketSize );
		if (newPiggy->mem == NULL) 
		{
			PrintError(("PutStressOnMemoryAllocation failed: Could not allocate load for a new piggy of %ld after %ld allocated already!\n", onePacketSize, soFar));
			//We will not "return" here because we still have to deallocate all the other piggies.
			break;
		}	
		newPiggy->next = headPiggy;
		newPiggy->id = headPiggy->id +1;
		headPiggy = newPiggy;
		soFar += onePacketSize;
	}

	//At this point we have either allocated all necessary piggies, 
	//  or we have encountered an error while allocating.
	// Either way, we still have to clean up before exiting, so: deallocate
	while (headPiggy != NULL)
	{
		newPiggy = headPiggy->next;
		if ( headPiggy->mem != NULL)
			free(headPiggy->mem);
		free(headPiggy);
		headPiggy = newPiggy;
	}
	
	PrintInfo(("... PutStressOnMemoryAllocation() finished ! \n"));

	//Clean exit
}





// functions to set the parameters of potential energy function and minimization
void internalSaveMoleculeAsPDBAfterMinimization( char *outputFileName );

void interactions_omission (int NTF_amber);			
void set_steepest_descent_conjugate_gradient( void );
void set_conjugate_gradient( void );
void set_energy_calculation( void );
void set_steepest_descent  ( void );
void set_number_maxcyc (int number_maxcyc );

void energy_init(char *prefixOfIntermediaryFiles);


int InitializeLigand( char *ligandFileName );
int InitializeLigand_NonProtein( char *ligandFileName );
static void    load_force_fields( );

// functions for energy calculation and minimization 
void minimization_calcul (double *px_coo, double *energy, double *pm_coo );
void energy_calcul (double *px_coo, double *energy);
void  PARAMET_free ( PARAMET *PARM );

void WriteRefcFileIfNeeded( double *px_coo, int nbAtoms );

void UpdateRobotConfigWithNewCartesianCoordinates (p3d_rob *mol,double *m_coo) ;

void get_coordinates (p3d_rob *mol, double *x_coo);

// function to set molecular dynamics calculations
void set_molecular_dynamics ( void );

void test_coordinates ( double *pm_coo, torsion *torPt, double *pup_coo);

//Local variables
static int          flagLoadedForceFields = FALSE;
static OBJEKT       objProt = NULL;  
static OBJEKT       objLig    = NULL;
static PARAMET     *parmet   = NULL;

minimizationType currentMinimizationType = minimSDCG;
int internalNbOfMinizationCycles = 10;	//This is only the initial value. It can be changed from the interface

static int keepLigandFixedDuringMinimization = 1; //Should be 1 for TRUE and 0 for FALSE. It shold NOT be changed ar runtime since it will crash the minimization (or give erroneus results)



//The "prefixOfIntermediaryFiles" variable holds a file name without extention that is used as root
// for the construction of all intermediary files. It is initialized in the InitializeProtein method
char   prefixOfIntermediaryFiles[2*NB_OF_CHAR];



//These pointers will be initialized with some external methods
void (* AlertTheUser)(char* title, char* text1, char* text2);
int (* QuestionTheUser)(char* question, int defaultBtn);





//This function is called from the user interface with a valid file name.
// It should return 0 if successfull or -1 otherwise
// This method is called before InitializeLigand
int InitializeProtein( char *proteinFileName, char *ligandFileName, void (* AlertTheUserMethod)(char*, char*, char*), int (* QuestionTheUserMethod)(char*, int ) )
{
	FILE         *fPdb = NULL;
	OBJEKT     ProtLig = NULL;
	char  proteinFilePrefix[NB_OF_CHAR], ligandFilePrefix[NB_OF_CHAR];
	char    dir[NB_OF_CHAR]; 
	char*systemCommand;

	AlertTheUser = AlertTheUserMethod;
	QuestionTheUser = QuestionTheUserMethod;

	get_short_file_name((char*)proteinFileName,proteinFilePrefix,NB_OF_CHAR);

	/* loading of force fields and initialization */
	load_force_fields();

	fPdb = fopen( proteinFileName, "r");
	/* generation of AMBER structures for protein */
	objProt = AMBER_loadPdb(fPdb);
	fclose(fPdb);
	
	if ( ( GetCurrentLigandType() == ligtypeNON_PROTEIN ) ||
		( GetCurrentLigandType() == ligtypePROTEIN ))
	{
		//If a ligand is present, then call the Ligand Initialization also
		if ( InitializeLigand( ligandFileName ) != 0)
			return -1;

		get_short_file_name((char*)ligandFileName,ligandFilePrefix,NB_OF_CHAR);

		/* creation of one OBJEKT for protein and ligand */
		ProtLig = AMBER_Combine(objProt, objLig);
	}
	else 
	{  ProtLig = objProt; }
	
	strcpy(dir , fl_get_directory());

	if ( GetCurrentLigandType() == ligtypeNONE ) 
	{	sprintf(prefixOfIntermediaryFiles,"%s/%s", dir, proteinFilePrefix);
		//      sprintf(prefixOfIntermediaryFiles,"%s/%s","/tmp/astefani_move3d",prot_prefix);
	}
	else 
	{	sprintf(prefixOfIntermediaryFiles,"%s/%s_%s", dir, proteinFilePrefix, ligandFilePrefix); 
		//    sprintf(prefixOfIntermediaryFiles,"%s/%s_%s","/tmp/astefani_move3d",prot_prefix,lig_prefix); 
	}
	
	/* creation of .parm and .coord files */  
	AMBER_saveParm((UNIT) ProtLig, prefixOfIntermediaryFiles); 

	//Before initializing the Energy module, we should create the ".refc" file from the ".coord" file
	// The coord file contains the initial coordinates of the protein ligand ensamble
	// The refc file will be used for maintaining the ligand fixed during minimization
	systemCommand = (char*)malloc(sizeof(char) * ( 2*strlen(prefixOfIntermediaryFiles) + 20) );
	systemCommand[0] = 0;
	sprintf(systemCommand, "cp %s.coord %s.refc", prefixOfIntermediaryFiles, prefixOfIntermediaryFiles);
	PrintInfo(("Executing shell: %s\n", systemCommand));
	system(systemCommand);
	free(systemCommand);

	energy_init (prefixOfIntermediaryFiles);
	Destroy(&ProtLig);
 
	//Let the Normal Mode module know what the protein and ligand files are
	SetCurrentFilesForProteinAndLigand( proteinFileName, ligandFileName);

	return 0;	//0=success
}


//This method is called from InitializeProtein. It should return 0 on success and -1 otherwise
int InitializeLigand( char *ligandFileName )
{
	FILE         *fPdb = NULL;
	
	if ( GetCurrentLigandType() == ligtypePROTEIN)
	{
		//####################################
		//######## PROTEIN LIGAND initialization
	
		/* generation of AMBER structures for second protein (ligand) */  
		fPdb = fopen(ligandFileName, "r");
		objLig = AMBER_loadPdb(fPdb);
		fclose( fPdb );

		//######## End of PROTEIN LIGAND initialization
		//###########################################
	}
	else
	{
		//####################################
		//######## NON-PROTEIN LIGAND initialization
		if ( InitializeLigand_NonProtein( ligandFileName ) != 0)
			return -1;
	
		//######## End of PROTEIN LIGAND initialization
		//###########################################
	}

	return 0;
}


//This method is called from InitializeLigand. It should return 0 on success and -1 otherwise
int InitializeLigand_NonProtein( char *ligandFileName )
{
	char *file_l = NULL;
	char file_prep[NB_OF_CHAR];
	char file_frcmod[NB_OF_CHAR];
	char *fileNameNoExtension = NULL;
	
	FILE *fprep = NULL;
	int needToCreateTheFile = TRUE;
	
	char *system_env;
	char antechamber_tmpchar[2*NB_OF_CHAR];
	char parmch_tmpchar[2*NB_OF_CHAR];
	char strchar[3*NB_OF_CHAR];
	char strrchar[3*NB_OF_CHAR];
	
	OBJEKT  ParmsLig  = NULL;
	


	/* files names */ 
	fileNameNoExtension = MY_ALLOC(char, strlen(ligandFileName) + 1);
	fileNameNoExtension[0] = 0;
	GetFileNameWithPathWithoutExtension( ligandFileName, &fileNameNoExtension );

	//TODO Some file prefixes are required here. See where these files are created and move them properly.
	//TODO One day these files should be moved to some (configurable) folder	
	if ( strlen( fileNameNoExtension ) > (NB_OF_CHAR - 10) )
	{
		PrintError(("The file name: %s\n is too long and cannot be processed.\n", file_l ));
		exit(-1); 
	}
	strcpy( file_prep, fileNameNoExtension );
	strcpy( file_frcmod, fileNameNoExtension );
	strcat(file_prep,".mol2");//".prep");

	//Actually, I think that the "parms" should be written in a file bearing the name 
	// of the protein ligand combination because if we write it with the name of the ligand
	// only, the content will not be available to the 
	strcat(file_frcmod,".parms");

	MY_FREE(fileNameNoExtension, char, strlen(ligandFileName) );

	
	system_env = (char *) getenv("AMBERHOME");	//No need for deallocation
	if (system_env == NULL) {
		//TODO. Maybe this verification should be done upon diplaying this form and not in the middle of the execution.
		// this way we will save ourselves a lot of debugging. Also, move the loading of the force fields in the same place (at the beginning).
		PrintError(("\E[31m ERROR: system environment variable AMBERHOME isn't defined. \n"));
		PrintError(("The AMBER related section cannot function properly. \n"));
		
		AlertTheUser("Error !","System environment variable AMBERHOME isn't defined.","The AMBER related section cannot be executed.");
		return -1;
	}

	//Check whether the file exists by opening it
	fprep = fopen(file_prep,"r");
	if(fprep != NULL) 
	{	//If it does exist, then do not forget to close it and then ask the user for further directions
		fclose(fprep);
		needToCreateTheFile = (QuestionTheUser("Intermediary file for ligand already exists. Do you want to generate it again?",0));
	}
	
	if (needToCreateTheFile)
	{
		/* creation of .prep and .parms files for ligand */
		antechamber_tmpchar[0] = '\0';
		parmch_tmpchar[0]= '\0';
		strcpy(antechamber_tmpchar, system_env);
		strcpy(parmch_tmpchar, system_env);
		strcat(antechamber_tmpchar, "/exe/antechamber");
		strcat(parmch_tmpchar, "/exe/parmchk");
		
		sprintf(strchar,"%s -i %s -fi pdb -o %s -fo mol2 -c mul -pf y ", 
			antechamber_tmpchar, ligandFileName, file_prep );
		PrintInfo(("\n\nExecuting shell command: %s\n\n", strchar));
		system(strchar);
		sprintf(strrchar, "%s -i %s -f mol2 -o %s", parmch_tmpchar, file_prep, file_frcmod ) ;
		PrintInfo(("\n\nExecuting shell command: %s\n\n", strrchar));
		system(strrchar);

		//TODO Do not forget to add here the same prefixes like above
		sprintf(strchar,"rm -f ANTECHAMBER* ATOMTYPE.INF BCCTYPE.INF NEWPDB.PDB PREP.INF divcon.* leap.log" );
		PrintInfo(("\n\nExecuting shell command: %s\n\n", strchar));
		system(strchar);
	}
	
	/* generation of AMBER structures for ligand */
	
	ParmsLig = AMBER_loadParams(file_frcmod);
	//objLig = AMBER_loadPrep(file_prep);
	objLig = AMBER_loadMol2(file_prep);

	//ACTUALLY, we should NOT destroy the object, because things will not work
	// when there are parameters that need to be loaded.Destroy(&ParmsLig)

	return 0; //Success
}

void energy_init (char *prefixOfIntermediaryFiles)  
{
  /*
   *  read_inp_ --  fortran function reads first part of parm file, calculates size memory
   *  read_parm_ -- fortran function reads second part of parm file after allocation of memory 
   */ 
  
  int len_name = 0;
  int lastr = 1;
  int lasti = 1;
  int lasth = 1;
  int lastpr = 1;
  int lastrst = 1;
  int lastist = 1;

 len_name = strlen(prefixOfIntermediaryFiles);

 readmemorysizesfromparmfile_(&lastr,&lasti,&lasth,&lastpr,&lastrst,&lastist, prefixOfIntermediaryFiles, &len_name, &keepLigandFixedDuringMinimization);
 parmet = (PARAMET*) malloc (sizeof(PARAMET));
 parmet->px = (double*) malloc( lastr * sizeof(double));
 parmet->pix = (int*) malloc( lasti * sizeof(int));
 parmet->pih = (STRING*) malloc( lasth * sizeof(STRING));
 parmet->pipairs = (int*) malloc( lastpr * sizeof(int));
 parmet->pr_stack = (double*) malloc( lastrst * sizeof(double));
 parmet->pi_stack = (int*) malloc( lastist * sizeof(int));
 
 readfromparmfile_(parmet->px,parmet->pix,parmet->pih,parmet->pipairs,parmet->pr_stack,parmet->pi_stack);

 //PARMpt = &PARM;
}








//This method is called from the interface and should return 0 on OK, and -1 otherwise
int CalculateEnergy(p3d_rob *mol, double *totE)
{
	double   *px_coo   = NULL ;
	//double   *px_coo_pdb   = NULL ;
	//int i;
	double *energy;	//Vector of energies to be filled by the Fortran code
	int nbAtoms = 0;
	
	set_energy_calculation( );
	
	nbAtoms = ComputeTotalNumberOfAtoms();
	
	energy = MY_ALLOC (double,NB_OF_ENERGY_TERMS);
	px_coo = MY_ALLOC (double,nbAtoms*3);
	
	get_coordinates( mol, px_coo );
	
	WriteRefcFileIfNeeded( px_coo, nbAtoms );

   //writecoord_ (px_coo, &number_at);   
   energy_calcul (px_coo, energy); 

   printf("\nTotal energy of molecule = %f \n", energy[0]);

   *totE = energy[0];

   /*
     for (i=0; i<number_at*3; i++) {
     if ((*(px_coo++)-*(px_coo_pdb++))*(*(px_coo++)-*(px_coo_pdb++)) > 1.0)
     if (*(px_coo++) == 0.0 )
     printf (" problem in atom number %d \n", (i/3+1)); 
     printf ("  %d %f %f \n ",i,*(px_coo++), *(px_coo_pdb++));
   }
   */
	MY_FREE(energy,double,NB_OF_ENERGY_TERMS);
	MY_FREE(px_coo,double,nbAtoms*3);

	return 0;
}


/**
 * @brief This method calls the ambmov program for energy minimization
 * @param robotPt Robot configuration to be minimized
 * @param tot_energyAfterMinimization This is a pointer to the value of the total energy
 * @return 0 on success       !=0 otherwise
 */
// NOTE: this function supposes that the conformtion to be minimized has been set and updated
int EnergyMinimizationUsingAmbmov( p3d_rob *robotPt, double *tot_energyAfterMinimization)
{
  char str[512];
  FILE *fPDBin,*fE;
  int niter = 1000;

  // write conf in pdb file
  fPDBin = fopen("curconfnomin.pdb", "w");
  translate_conf_to_pdb(robotPt, fPDBin);
  fclose(fPDBin);
  // call ambmov
  sprintf(str,"%s/BioTools/ambmov/ambmov -ip curconfnomin.pdb -o curconfmin.pdb -n %d",getenv("HOME_MOVE3D"),niter);
  system(str);

  // read energy value after minimization : totE.ambmovout
  if((fE = fopen("totE.ambmovout", "r")) == NULL) {
    *tot_energyAfterMinimization = P3D_HUGE;
    printf("WARNING : EnergyMinimizationUsingAmbmov : structure cannot be minimized\n");
    // remove files
    sprintf(str,"/bin/rm curconf*");
    system(str);    
    return -1;
  }
  //fgets(str,50,fE);  
  fscanf(fE,"%lf",tot_energyAfterMinimization);
  fclose(fE);

  // remove files
  sprintf(str,"/bin/rm curconf* totE.ambmovout");
  system(str);

  return 0;
}



/**
 * @brief This method calls the Energy Minimization methods of the Sander module from AMBER
 * This method is called from the interface.
 * @param mol Robot configuration to be minimized
 * @param outputPDFFileName  Optional parameter. If given, the minimized configuration will be written into this file as a PDB
 * @param energyAfterMinimization This is a pointer to a vector of several energy values. These values are given by amber after the minimization (so there is no passing through Move3D for this one
 * @return 0 on success       !=0 otherwise
 */
int EnergyMinimization( p3d_rob *mol, char *outputPDBFileName, double *energyAfterMinimization)
{
	double         *px_coo   = NULL ;
	double *m_coo = NULL;  // coordinates of atoms after minimization
	int nbAtoms = 0;

	if ( objProt == NULL) 
	{	
		PrintError(("The Bio Energy module has not been initialized. The EnergyMinimization cannot be performed.\nPlease use the BioEnergy window to perform the initialization !"));
		return -1;
	}

	//Find out the number of atoms in the molecule.
	nbAtoms = ComputeTotalNumberOfAtoms();
	
	px_coo = MY_ALLOC (double,nbAtoms*3);
	m_coo =  MY_ALLOC (double,nbAtoms*3);
	
	get_coordinates(mol,px_coo);



	WriteRefcFileIfNeeded( px_coo, nbAtoms );



	mdi_.nscm = 100000;
	set_number_maxcyc( internalNbOfMinizationCycles );
	
	switch ( currentMinimizationType ){
	case minimSDCG: 
		set_steepest_descent_conjugate_gradient();
		break;
	case minimSD:
		set_steepest_descent ();
		break;
	case minimCG: 
		set_conjugate_gradient();
		break;
	default: //Make sure we have received good parameters
		PrintError(("arg received with incorrect value. \n"));
		AlertTheUser("Error !","Unknown minimisation method required or parameter\n out of range!","");
		return -1;
	}

	//      printf("%f\n\n",px_coo);
	//writecoord_ (px_coo, &number_at);
	fortrancodeenergyminimisation_(px_coo, parmet->px,parmet->pix,parmet->pih,parmet->pipairs,parmet->pr_stack, parmet->pi_stack, 
		energyAfterMinimization, m_coo, &nbAtoms);

	printf ("\nTotal energy of molecule = ini%f \n", energyAfterMinimization[0]);

	//Update the robot configuration with the minimized one
	UpdateRobotConfigWithNewCartesianCoordinates (mol,m_coo);

	if ( outputPDBFileName != NULL)
	{
		internalSaveMoleculeAsPDBAfterMinimization( outputPDBFileName );
	}

	MY_FREE(px_coo,double,nbAtoms*3);
	MY_FREE(m_coo,double,nbAtoms*3);
	
	return 0;
}

/**
 * @brief This should NOT be used externally. It is to be called imediately after an energy minimization
	because it needs a file that is created durint the minimization (the .restrt file)	
 * @param outputFileName Name of the file into which the molecule will be written in PDB format
 */
void internalSaveMoleculeAsPDBAfterMinimization( char *outputFileName )
{
	char str_exe[NB_OF_CHAR];
	char cmdToExecute[NB_OF_CHAR*3];
	char *system_env;

	//We are sure this is not null because the validation performed when loading the PDB file
	system_env = (char *) getenv("AMBERHOME");
	str_exe[0] = '\0';
	strcpy(str_exe,system_env);
	strcat(str_exe, "/exe/ambpdb");	//We will execute the "ambpdb" program

	sprintf(cmdToExecute,"%s -p %s.parm < %s.restrt > %s ", str_exe, prefixOfIntermediaryFiles, prefixOfIntermediaryFiles, outputFileName );
	
	PrintInfo(("\n\n\E[34m\E[7m Executing: %s  \E[0m\n\n", cmdToExecute));
	system( cmdToExecute );
}




//This method updates the *mol robot configuration according to the received 
// coordinates in m_coo.
void UpdateRobotConfigWithNewCartesianCoordinates (p3d_rob *mol,double *m_coo) 
{
	vector *vecPt = NULL;
	torsion *torPt_Prot = NULL;
	torsion *torPt_Lig = NULL;
	
	double *angle_Prot = NULL;
	double *angle_Lig = NULL;  
	p3d_jnt *jPt;
	int i;
	int nbJointsProt = 0, nbJointsLig = 0;
	int firstJointProtein, lastJointProtein, firstJointLigand, lastJointLigand;
	int nbTotalOfAtoms = ComputeTotalNumberOfAtoms();

	if(!GetFirstAndLastJointOfMainProteinAndLigand( &firstJointProtein, &lastJointProtein, &firstJointLigand, &lastJointLigand )) 
	{
		PrintError(("Could not get GetFirstAndLastJointOfMainProteinAndLigand =>UpdateRobotConfigWithNewCartesianCoordinates(...) failed ! "));
		return;
	}


	//==============================
	//Do the adaptation of the new coordinates into the robot for the protein

	nbJointsProt = lastJointProtein - firstJointProtein + 1;

	alloc_vector_struct (&vecPt, ComputeTotalNumberOfAtoms());
	alloc_torsion_struct(&torPt_Prot, nbJointsProt );

	//Get the positions from m_coo into vecPt
	GetAtomsPositionsFromLinearVectorToNx3matrix(m_coo, nbTotalOfAtoms, vecPt);
	
	//Get the old (ie. current) torsion angles
	GetAtomNumbersForDihedralsOfEachJoint (mol, torPt_Prot, firstJointProtein, lastJointProtein );

	angle_Prot = MY_ALLOC ( double, nbJointsProt + 1 );
	//Transform the new coordinates into new torsion angles that will be stored in angle_Prot
	GetTorsionAnglesFromCartesianCoordinates(vecPt, torPt_Prot, angle_Prot);

	
	//Here we adapt our model of the protein with the newly obtained coordinates
	for(i = firstJointProtein; i <= lastJointProtein; i++)
	{
		jPt = mol->joints[i];
//		PrintInfo(("P:i=%d        jPt->v=%lf     jPt->dof_data[0].v=%lf    newVal=%lf\n", i, jPt->v, jPt->dof_data[0].v, angle_Prot[ i - firstJointProtein +1 ]));
		p3d_jnt_set_dof(jPt, 0, angle_Prot[ i - firstJointProtein +1 ]);
	}


	free_torsion_struct(torPt_Prot);
	MY_FREE(angle_Prot, double, nbJointsProt + 1);


	//==============================
	//Do the adaptation of the new coordinates into the robot for the ligand also if it is necessary
	if ( GetCurrentLigandType() != ligtypeNONE )
	{
		nbJointsLig = lastJointLigand  - firstJointLigand +1;
		if (torPt_Lig == NULL)
			alloc_torsion_struct(&torPt_Lig, nbJointsLig );

		GetAtomNumbersForDihedralsOfEachJoint (mol, torPt_Lig, firstJointLigand, lastJointLigand );

		angle_Lig = MY_ALLOC ( double, nbJointsLig + 1 );
		GetTorsionAnglesFromCartesianCoordinates(vecPt, torPt_Lig, angle_Lig);

		for(i = firstJointLigand; i <= lastJointLigand; i++)
		{
			jPt = mol->joints[i];
//			PrintInfo(("L:i=%d        jPt->v=%lf     jPt->dof_data[0].v=%lf    newVal=%lf\n", i, jPt->v, jPt->dof_data[0].v, angle_Lig[ i - firstJointLigand + 1]));
			p3d_jnt_set_dof(jPt, 0, angle_Lig[ i - firstJointLigand + 1]);
		}

		free_torsion_struct(torPt_Lig);	
		MY_FREE(angle_Lig, sizeof(double), nbJointsLig + 1 );	
	}

	//This structure is used for the ligand also, so we deallocated it here
  	free_vector_struct(vecPt);
	
	//mol_up = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	//up_coo = get_coordinates(mol_up);
	//test_coordinates (m_coo, torPt, up_coo);
}

//This method is called from the user interface. It should return 0 if succesfull or -1 otherwise
int MolecularDynamics()
{
	double         *px_coo   = NULL ;
	double *energy;
	int nbAtoms = 0;
	p3d_rob *mol;
	
	set_molecular_dynamics( );
	
	nbAtoms = ComputeTotalNumberOfAtoms();
	
	energy = MY_ALLOC (double,NB_OF_ENERGY_TERMS);
	px_coo = MY_ALLOC (double,nbAtoms*3);
	
	mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
	get_coordinates(mol,px_coo);
	energy_calcul (px_coo, energy );
	
	MY_FREE(energy,double,NB_OF_ENERGY_TERMS);
	MY_FREE(px_coo,double,nbAtoms*3);

	return 0; //success
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

void set_steepest_descent_conjugate_gradient( void )
{
  mdi_.imin = 1;
  mdi_.ntmin =1;
}

void set_energy_calculation( void )
{
  mdi_.imin = 1;
  mdi_.maxcyc = 0;
}

void set_steepest_descent  ( void ){
  mdi_.imin = 1;
  mdi_.ntmin = 2;
}

void set_conjugate_gradient( void ){
  mdi_.imin = 1;
  mdi_.ntmin = 0;
}

void set_molecular_dynamics ( void ){ 
  mdi_.imin = 0;
}

void interactions_omission (int NTF_amber){
  /*
   *NTF_amber 
   *
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




//TODO. At this function I did not look
void load_force_fields( )
{ 
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
  RESULTt	       rResult;

	if (flagLoadedForceFields == TRUE)
	{	//We do not need to execute this method twice.
		return;
	}
	flagLoadedForceFields = TRUE;

	system_env = (char *) getenv("AMBERHOME");
	if (system_env == NULL) 
	{
		PrintError(("System variable AMBERHOME not found ! The execution cannot continue.\n"));
		exit(2);
	}

  /* Initialization of Leap AMBER module */  

	BasicsInitialize();

    for(i = 0; i < NB_OF_LIB ; i++){
      tmpchar[0] = '\0';
      strcpy(tmpchar, system_env);
      strcat(tmpchar, lib_path[i]);
       AMBER_addPath(tmpchar);
    }

  AMBER_add_source(flibLig);

  /* Initialize the parser.
   * If SbStartup is TRUE the execute the LEAPRC script.
   */
  ParseInit(&rResult);
}




void energy_calcul (double *px_coo, double *energy )
{
 // pm_coo -- coordinates of atoms after minimization do not change if only energy is calculated
  double *pm_coo;
  int nbOfAtoms = ComputeTotalNumberOfAtoms();
  pm_coo = MY_ALLOC(double, nbOfAtoms*3);
  
  // min_energy_ -- fortran function to run energy calculation, minimization calculations or MD. 
  fortrancodeenergyminimisation_(px_coo, parmet->px,parmet->pix,parmet->pih,parmet->pipairs,parmet->pr_stack, parmet->pi_stack, 
	      energy, pm_coo, &nbOfAtoms);

  MY_FREE(pm_coo,double, nbOfAtoms*3);
}



//TODO at this I did not look
void 
PARAMET_free ( PARAMET *PARM )
{

  free(PARM->px);
  free(PARM->pix);
  free(PARM->pih);
  free(PARM->pipairs);
  free(PARM->pr_stack);
  free(PARM->pi_stack);
  
  free (PARM);
}  




//###################################################################################3
//######## GET COORDINATES
//TODO Why do we need 3 methods for get_coordinates. Write some comment here
void get_coordinates (p3d_rob *mol, double *x_coo)
{
  p3d_jnt **ptjoint;
  p3d_poly **ptpoly;
  int i,j;
  int k;
  int number = 0;
  char *nbStartIndex;
  int number_at = 0;
  
  ptjoint = mol->joints;
  for (i= 2; i<= mol->njoints; i++){
    if(ptjoint[i]->o != NULL) {
    
      ptpoly = ptjoint[i]->o->pol;
    
      // printf("*****total=%d *****\n",ptjoint[i]->o->np);
      for(j=0; j< ptjoint[i]->o->np ; j++){
	//printf("*****total=%d *****\n",ptjoint[i]->o->np);
	//pair_at_min_sd(ptjoint[i],ptjoint[i+1]);
	
	nbStartIndex = strchr(ptpoly[j]->poly->name, '.');
	if ( nbStartIndex == NULL )
	  {
	    PrintError(("The ptpoly[j]->poly->name expression is not correct !!!"));
	    exit(2);
	  }
	sscanf (nbStartIndex + 1,"%d",&number);
	

	for (k= 0; k<=2; k++){
	  x_coo[3*(number-1)+k] = ptpoly[j]->poly->pos[k][3];
	  
	  //printf("number %d ---- coordinate %f\n", number, x_coo[3*(number-1)+k]);
	}
	// printf("%s--- %d\n",word,number);
	number_at++;
      }
    }
  }
  printf("Number of atoms in p3d file %d\n",number_at);
}



//#### end of section
//#######################################################################################

/**
 * @brief This method finds the four atoms corresponding to each joint=dihedral angle
 * @param mol The robot configuration. Used for getting the name of each joint
 * @param torPt This structure contains the 4 atoms for each joint
 * @param firstJoint index of first joint to consider
 * @param lastJoint index of last joint to be considered
 */
void GetAtomNumbersForDihedralsOfEachJoint (p3d_rob *mol, torsion *torPt, int firstJoint, int lastJoint )
{
	char *nameCopy, *tempPtr;
	char *word1, *word2, *word3;
	char *w_n1, *w_n2, *w_n3, *w_n4;
	int **numPtPt;
	int *numPt;
	int i;

	nameCopy = MY_ALLOC( char, 200 );
	tempPtr = nameCopy;
	numPtPt = torPt->nPtPt;
	for( i = firstJoint; i <= lastJoint; i++)
	{
		nameCopy = tempPtr;
		strcpy(nameCopy, mol->joints[i]->name );
		//printf (" name of joint %s \n", name_joint);
		
		numPt = numPtPt[ i - firstJoint +1 ];
		word1 = strtok(nameCopy, ".");   
		word2 = strtok(NULL, "."); 
		word3 = strtok(NULL, ".");
		w_n1 = strtok(NULL,"-");
		w_n2 = strtok(NULL,"-");      
		w_n3 = strtok(NULL,"-");
		w_n4 = strtok(NULL," ");
		if ( w_n4 == NULL )
		{
			PrintError(("\nThe name of the joint: >%s< does not have the expected format: str.str.nb.nb-nb-nb-nb\nCannot obtain the atoms corresponding to the torsion angle => The application will stop.\n", mol->joints[i]->name));
			exit(2);
		}
		sscanf (w_n1,"%d",&numPt[0]);
		sscanf (w_n2,"%d",&numPt[1]);
		sscanf (w_n3,"%d",&numPt[2]);
		sscanf (w_n4,"%d",&numPt[3]);
		//printf ("%d %d %d %d %d \n", i, numPt_Prot[0], numPt_Prot[1], numPt_Prot[2], numPt_Prot[3]);
    	}
	nameCopy = tempPtr;
	MY_FREE( nameCopy, char, 200 );
}





//This  function was used to test the differences between the minimized coordinates and the adapted coordinates (ie. minimized structure put in the robot)
void test_coordinates ( double *m_coo, torsion *torPt, double *up_coo) 
{
  int i,k;
  int number1, number2, number3, number4;
  int **numPtPt;
  int  *numPt;
  double *x_n1;
  double *x_n2;
  double *x_n3;
  double *x_n4;

  x_n1 = (double*) malloc(3 * sizeof(double)); 
  x_n2 = (double*) malloc(3 * sizeof(double)); 
  x_n3 = (double*) malloc(3 * sizeof(double)); 
  x_n4 = (double*) malloc(3 * sizeof(double)); 
  
  numPtPt = torPt->nPtPt;
    
  for (i=1; i<= torPt->njoint; i++){
      numPt = numPtPt[i];
   
      number1 = numPt[0];
      number2 = numPt[1];
      number3 = numPt[2];       
      number4 = numPt[3];

       for (k= 0; k<=2; k++){
	 x_n1[k] = m_coo[3*(number1-1)+k]- up_coo[3*(number1-1)+k];
	 x_n2[k] = m_coo[3*(number2-1)+k]- up_coo[3*(number2-1)+k];
	 x_n3[k] = m_coo[3*(number3-1)+k]- up_coo[3*(number3-1)+k];
	 x_n4[k] = m_coo[3*(number4-1)+k]- up_coo[3*(number4-1)+k];
       }

       printf (" %d %f %f %f \n",number1,x_n1[0],x_n1[1], x_n1[2]);
       printf (" %d %f %f %f \n",number2,x_n2[0],x_n2[1], x_n2[2]);
       printf (" %d %f %f %f \n",number3,x_n3[0],x_n3[1], x_n3[2]);
       printf (" %d %f %f %f \n",number4,x_n4[0],x_n4[1], x_n4[2]);
       //printf (" numbers %d %d %d %d \n", number1, number2, number3, number4);
       
  }
}



void SetMinimizationMethod( minimizationType newMinimizationType  )
{
	currentMinimizationType = newMinimizationType;
}
minimizationType GetMinimizationMethod()
{
	return currentMinimizationType;
}
void SetNbMinimizationCycles( int nbOfMinimizationCycles )
{
	internalNbOfMinizationCycles = nbOfMinimizationCycles;
	set_number_maxcyc( internalNbOfMinizationCycles );
}
int GetNbMinimizationCycles()
{
	return internalNbOfMinizationCycles;
}



int ComputeDistances( int *n_pairs, double* min_dist, double* avr_dist)
{
	p3d_poly **p1,**p2;
	double *distances;
	int i, indexLigandRob;
	int n_dist_pairs;
	double sum = 0;
	//int in_collision;
	
	if (bio_all_molecules_col() > 0)
	{ 
		return FALSE;
	}
	
	bio_set_col_mode(SURFACE_DISTANCE_BIOCOL_MODE);
	bio_set_surface_d(1.0);  //In angstroms
	bio_all_molecules_col();

	//If there is a ligand, then get only the information related to the 
	// protein-ligand interaction
	if ( GetCurrentLigandType() != ligtypeNONE )
	{
		//We suppose that the is only one ligand and that it is the last robot
		indexLigandRob = get_number_of_bcd_robots() - 1;
		bio_molecule_col_no_autocol( indexLigandRob );
	}

	biocol_report_dist(&n_dist_pairs, &p1, &p2, &distances);
	bio_set_col_mode(NORMAL_BIOCOL_MODE);
	/////////////////////////

	*n_pairs = n_dist_pairs;
	
	//for(i = 0; i <n_dist_pairs ; i++){  // correct. Juan (c'etatit n_coll)
	//  printf("la paire  %s   %s est a distance : %f �\n", p1[i]->poly->name, p2[i]->poly->name, distances[i] );
	//}
	
	*min_dist = distances[0];
	for(i = 1; i <n_dist_pairs; i++)
	{
		sum += distances[i];
		if(distances[i] < *min_dist)
			*min_dist = distances[i];
	}
	*avr_dist = sum / n_dist_pairs;

	return TRUE;
}




/**
 * This method is called before each minimization and energy computation
 * @param px_coo See WriteCoordFile
 * @param nbAtoms See WriteCoordFile
 */
void WriteRefcFileIfNeeded( double *px_coo, int nbAtoms )
{
	char* tempFileName;

	if ( keepLigandFixedDuringMinimization )
	{	
		//Now, before doing the minimization we have to do one more thing:
		//If the "keepLigandFixedDuringMinimization" variable is set to 1
		// we have to prepare the coordinates where we want the ligand to be kept at.
		// For that we will create a ".refc" file with the desired coordinates
		tempFileName = (char*)malloc( sizeof(char) * ( strlen(prefixOfIntermediaryFiles) + 10) );
		tempFileName[0] = 0;
		strcpy( tempFileName, prefixOfIntermediaryFiles );
		strcat( tempFileName, ".refc" );
		WriteCoordFile( tempFileName, px_coo, nbAtoms );
		free( tempFileName );
	}
}


/**
 * This method writes a ".coord" type like file (same as .refc and .restrt)
 * @param fileName Name of the file to be created
 * @param atmCoords Linear vector with atoms coordinates [x1,y1,z1,x2,y2,z2,x3,...]
 * @param nb_atoms The number of atoms
 */
static void WriteCoordFile( char* fileName, double* atmCoords, int nb_atoms )
{
	FILE *fp;
	int index;

	if ( ( fp = fopen( fileName, "w") ) == NULL)
	{
		PrintError(("Failed to write Coord type file %s.\n Some parts of the application might fail because of this !!", fileName));
		return;
	}

	fprintf(fp, "\n");		//This is for the title part
	fprintf(fp, "%6d\n", nb_atoms);	//Nb of atoms
	for(index=0; index < nb_atoms; index++ )
	{
		fprintf( fp, "%12.7lf%12.7lf%12.7lf\n", atmCoords[index*3], atmCoords[index*3 + 1], atmCoords[index*3 + 2] );
		if ( index < 20 )
			printf( "%lf    %lf   %lf\n", atmCoords[index*3], atmCoords[index*3 + 1], atmCoords[index*3 + 2] );
	}

	fclose(fp);
}

