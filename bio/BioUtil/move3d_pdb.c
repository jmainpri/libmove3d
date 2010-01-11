#include <stdio.h>
#include <string.h>

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "Bio-pkg.h"

#ifdef ENERGY
#include "../BioEnergy/include/Energy-pkg.h"
#endif

/*  
#include "polyhedre.h"
#include "p3d.h"
#include "device.h"
*/
 

//number of the atom within the amino in the **move3d** format
static int natomamino=0;
//serial number of the amino
static int namino=1;
//tables for the position of the amino atoms placed in *move3d* order
static double pos[300][3];
//serial numbers of the atoms in the pdb format
static int natom[300];
static int new_namino=1;

typedef  char word[6];


static word chain;  //chain identifier
//amino name
static word amino;
// complete p3d name of the atom
static char polyname[25];
//tables for the names of the amino atoms placed in *move3d* order
static word names[300];
static word new_amino;
static word new_chain;



/* returns a chain (max. 20 charcters) with the characters following
the *index position of the parameter string until the first instance 
of the second parameter is found.
index is returned with the position after the the first instance of c
*/ 

/* char *givemeword(char string[], char c, int *index){ */

/* 	static char cadena[20]; */
/* 	int i=0; */
/* 	while ((string[*index]!= c) && (string[*index]!= '\0')){ */
/* 		cadena[i++]=string[(*index)++]; */
/* 		} */
/* 	cadena[i]='\0'; */
/* 	(*index)++; */
/* 	return cadena; */
/* 	} */


/* gets the amino name and serial number from the name in chain.
chain is supposed to have the format

		atom-atom-...-atom.aminoname.aminonumber.chainIdentifier
		
*/		
static void readamino(char *s){
	int indexletter=0;
	givemeword(s, '.', &indexletter);
	strcpy(new_amino,givemeword(s, '.', &indexletter));
	if(strcmp(new_amino,"LIG") != 0) {
	  sscanf(givemeword(s, '.', &indexletter),"%d", &new_namino);
	  strcpy(new_chain,givemeword(s, '\0', &indexletter));
	  if (new_chain[0]=='0') new_chain[0]=' ';
	}
	else {
	  new_namino = -1;
	  strcpy(new_chain,"L");
	}
		
}


/* puts the name and position of the atom in tables pos and names] */

static void intable(p3d_poly *polygordo){
	int indexletter=2;
	
	strcpy(names[natomamino], givemeword(polyname,'.',&indexletter));

	if(strcmp(new_amino,"LIG") != 0) {
	  sscanf(givemeword(polyname, '\0', &indexletter),"%d", natom+natomamino);
	}
	else {
	  natom[natomamino] = natomamino + 1;
	}
	pos[natomamino][0]=polygordo->poly->pos[0][3];
	pos[natomamino][1]=polygordo->poly->pos[1][3];
	pos[natomamino][2]=polygordo->poly->pos[2][3];

	natomamino++;
	}


void common_infile(FILE *fp){	
int i;
int int_first_char;

for(i=0; i<2; i++){
	fprintf(fp,"%-6s","ATOM");
	fprintf(fp,"%5d", natom[i]);
	int_first_char = atoi(&names[i][0]);
	if((strlen(names[i]) == 4) ||
	   ((int_first_char > 0) && (int_first_char < 4)))
	  fprintf(fp,"%1s"," ");
	else
	  fprintf(fp,"%2s","  ");
	fprintf(fp,"%-3s",names[i]);
	if((int_first_char > 0) && (int_first_char < 4) && (names[i][3] == '\0'))
	  fprintf(fp,"%1s"," ");
	fprintf(fp,"%4s", amino);
	fprintf(fp,"%1s"," ");
	fprintf(fp,"%1s", chain);
	fprintf(fp,"%4d", namino);
	fprintf(fp,"%12.3f", pos[i][0]);
	fprintf(fp,"%8.3f", pos[i][1]);
	fprintf(fp,"%8.3f", pos[i][2]);
	fprintf(fp,"%26s","");
	fprintf(fp,"\n");
	}
for(i=natomamino-2; i<natomamino; i++){
	fprintf(fp,"%-6s","ATOM");
	fprintf(fp,"%5d", natom[i]);
	int_first_char = atoi(&names[i][0]);
	if((strlen(names[i]) == 4) ||
	   ((int_first_char > 0) && (int_first_char < 4)))
	  fprintf(fp,"%1s"," ");
	else
	  fprintf(fp,"%2s","  ");
	fprintf(fp,"%-3s",names[i]);
	if((int_first_char > 0) && (int_first_char < 4) && (names[i][3] == '\0'))
	  fprintf(fp,"%1s"," ");
	fprintf(fp,"%4s", amino);
	fprintf(fp,"%1s"," ");
	fprintf(fp,"%1s", chain);
	fprintf(fp,"%4d", namino);
	fprintf(fp,"%12.3f", pos[i][0]);
	fprintf(fp,"%8.3f", pos[i][1]);
	fprintf(fp,"%8.3f", pos[i][2]);
	fprintf(fp,"%26s","");
	fprintf(fp,"\n");
	}
for(i=2; i< natomamino-2; i++){
	fprintf(fp,"%-6s","ATOM");
	fprintf(fp,"%5d", natom[i]);
	int_first_char = atoi(&names[i][0]);
	if((strlen(names[i]) == 4) ||
	   ((int_first_char > 0) && (int_first_char < 4)))
	  fprintf(fp,"%1s"," ");
	else
	  fprintf(fp,"%2s","  ");
	fprintf(fp,"%-3s",names[i]);
	if((int_first_char > 0) && (int_first_char < 4) && (names[i][3] == '\0'))
	  fprintf(fp,"%1s"," ");
	fprintf(fp,"%4s", amino);
	fprintf(fp,"%1s"," ");
	fprintf(fp,"%1s", chain);
	fprintf(fp,"%4d", namino);
	fprintf(fp,"%12.3f", pos[i][0]);
	fprintf(fp,"%8.3f", pos[i][1]);
	fprintf(fp,"%8.3f", pos[i][2]);
	fprintf(fp,"%26s","");
	fprintf(fp,"\n");
	}
}

void write_atom(FILE *fp, int i){
int int_first_char;

	fprintf(fp,"%-6s","ATOM");
	fprintf(fp,"%5d", natom[i]);
	int_first_char = atoi(&names[i][0]);
	if((strlen(names[i]) == 4) ||
	   ((int_first_char > 0) && (int_first_char < 4)))
	  fprintf(fp,"%1s"," ");
	else
	  fprintf(fp,"%2s","  ");
	fprintf(fp,"%-3s",names[i]);
	if((int_first_char > 0) && (int_first_char < 4) && (names[i][3] == '\0'))
	  fprintf(fp,"%1s"," ");
	fprintf(fp,"%4s", amino);
	fprintf(fp,"%1s"," ");
	fprintf(fp,"%1s", chain);
	fprintf(fp,"%4d", namino);
	fprintf(fp,"%12.3f", pos[i][0]);
	fprintf(fp,"%8.3f", pos[i][1]);
	fprintf(fp,"%8.3f", pos[i][2]);
	fprintf(fp,"%26s","");
	fprintf(fp,"\n");
	}

void ILE_infile(FILE *fp){	
int i;

for(i=0; i<2; i++)
	write_atom(fp, i);
for(i=natomamino-2; i<natomamino; i++)
	write_atom(fp, i);
write_atom(fp, 2);
write_atom(fp, 4);
write_atom(fp, 3);
write_atom(fp, 5);
}

/* writes into the argument file all the registers of the
amino previously read */

void infile(FILE *fp)
{
  //if (strcmp(amino,"ILE")!=0)
  common_infile(fp);
  //else
  //ILE_infile(fp);
  natomamino=0;
}


void translate_conf_to_pdb(pp3d_rob protein, FILE *fp)
{
  pp3d_jnt joint;
  pp3d_jnt *jointpt;
  p3d_poly *polygordo;
  p3d_poly **polygordopt;  
  int j,i;
  int firstamino;

  jointpt=protein->joints;
  firstamino = 1;
  for (j=0;j < protein->njoints + 1; j++){
    joint=*jointpt; // joint is now a pointer to a joint ((*protein).joints))
    if (joint->o != NULL) {
      if(joint->o->name[0]!='.') {
	readamino(joint->o->name);
	if ((namino != new_namino) && (!firstamino)) 
	  infile(fp);
	strcpy(amino,new_amino);
	namino=new_namino;
	strcpy(chain,new_chain);
	polygordopt= joint->o->pol;
	for (i=1; i<= joint->o->np;i++){
	  polygordo= *polygordopt;
	  strcpy(polyname, polygordo->poly->name);
	  // NOTE: all the atoms are written  
	  //if (polyname[0]== 'V') 
	    intable(polygordo);
	  polygordopt++;
	}
	firstamino = 0;
      }	
    }
    jointpt++;
  }
  infile(fp);

}

/* writes into the argument file the configuration of the first argument
protein in pdb format 
The minimizationType parameter specifies whether a minimization will be used, and 
 if so, which one. The possible values are:
trajExpMinNONE
trajExpMinAMBERSaveFromM3D
trajExpMinAMBERSaveFromAmber
trajExpMinRRT
*/
static void move3d_to_pdb(pp3d_rob protein, char *filename, int filenumber, trajExportMinimizationType minimizationType)
{
	FILE *fp;
	char *newFileName;
	char *suffix = (char*)"_m3d_beforeMin";

#ifdef ENERGY
	double enAfterMinDirectlyFromAmber[ NB_OF_ENERGY_TERMS ];
	double energyBeforeMinimization = 0.0, energyAfterMinimization = 0.0;

	configPt qs;//Used with the RRT minimization
	
	//Amber returns an array of energies. The value in the [0] position is the total energy
	int *someNullIntPtr=NULL;
	int nbAtomsNear = 0;
	double minAtomDist, avgAtomDist;


	char* systemCommand, *tempChar;
	FILE *statsFile;
	char *statisticsFileName;

	if ( minimizationType != trajExpMinNONE )  // modif Juan : problem if AMBER is not initialized
	  CalculateEnergy( protein, &energyBeforeMinimization);

	if (! ComputeDistances( &nbAtomsNear, &minAtomDist, &avgAtomDist) )
	{
		nbAtomsNear = 0;
		minAtomDist = 0.0;
		avgAtomDist = 0.0;
	}
	if (nbAtomsNear == 0)
	{	minAtomDist = 0.0;	avgAtomDist = 0.0;	}
#endif
	if ( minimizationType != trajExpMinNONE )
	{ 	//For debug reasons, do a normal save of the PDB first if minimizations involved

		//This complicated thing is meant to obtain something like "0001_m3d.pdb" from  "0001.pdb"
		newFileName = (char*)malloc( strlen( filename) + strlen(suffix) + 1 );
		strcpy( newFileName, filename );
		strcpy( newFileName + strlen( filename) - 4, suffix );
		strcpy( newFileName + strlen( filename) - 4 + strlen( suffix ), ".pdb\0" );
			
		printf("newFileName=%s\n", newFileName);

		fp= fopen(newFileName,"w");
		fprintf(fp,"MODEL        %d\n", filenumber);
#ifdef ENERGY
		fprintf(fp,"REMARK   File generated before minimization\n");
		fprintf(fp,"REMARK   Energy=       %lf\n", energyBeforeMinimization);
#endif
		translate_conf_to_pdb(protein, fp);
		fclose(fp);
	}

#ifdef ENERGY
	switch( minimizationType )
	{
		case trajExpMinNONE:
			//Well,, nothing to be done here
			break;
		case trajExpMinAMBERSaveFromM3D:
			//Do only the minimization and the save will be performed below
			EnergyMinimization( protein, NULL, enAfterMinDirectlyFromAmber );	
			break;
		case trajExpMinAMBERSaveFromAmber:
			//Do the minimization and the save from Amber
			EnergyMinimization( protein, filename, enAfterMinDirectlyFromAmber );
			break;
		case trajExpMinRRT: 
			//Do the minimization with RRT and the save will be done below
			qs = p3d_get_robot_config(protein);
			bio_rrt_E_minimize( qs, someNullIntPtr, NULL, NULL);//fct_stop,fct_draw );
			break;
		default:
			PrintError(("Unknown minimizationType parameter value ! The results could be wrong !!"));
			break;
	}

	if (  minimizationType != trajExpMinNONE )
	{
	  	CalculateEnergy( protein, &energyAfterMinimization);
	}
#else
	//Nothing to be done if there is no energy module
	minimizationType = trajExpMinNONE;
#endif

	if (  minimizationType != trajExpMinAMBERSaveFromAmber )
	{	//If we have not already saved the molecule using Amber, 
		// save it the M3D way

		fp= fopen(filename,"w");
		translate_conf_to_pdb(protein, fp);
		fclose(fp);
	}

	//Now that we have obtained the transformed file,
	// we will write some extra information into it

#ifdef ENERGY
	//Create a temporary file and write in it all the extra information that we want to add
	// at the beginning of the PDB file
	fp= fopen("tmpfile","w");
	fprintf(fp,"MODEL        %d\n",filenumber);
	fprintf(fp,"REMARK   File generated ");
	switch( minimizationType )
	{
		case trajExpMinNONE:
			fprintf(fp, "with no minimization" );
			break;
		case trajExpMinAMBERSaveFromM3D:
			fprintf(fp, "after minimization with Amber method (save from M3D)" );
			break;
		case trajExpMinAMBERSaveFromAmber:
			fprintf(fp, "after minimization with Amber method (save from Amber)" );
			break;
		case trajExpMinRRT:
			fprintf(fp, "after minimization with RRT method" );
			break;
	}
	fprintf(fp,"method \n" );
	fprintf(fp,"REMARK   Energy before minimization =       %lf\n", energyBeforeMinimization);
	fprintf(fp,"REMARK   Energy after minimization ( straight from Amber )    =	%lf\n", enAfterMinDirectlyFromAmber[0]);
	fprintf(fp,"REMARK   Energy after minimization (after adaptation in M3D) =	%lf\n", energyAfterMinimization);
	fprintf(fp,"REMARK   Note that a 0 value means <not computed>\n");
	fclose( fp );

	//Now do some file manipulation in order to concatenate these two files 
	// (ie. the PDB and the tmp file)
	systemCommand = (char*)malloc(sizeof(char) * ( strlen( filename ) * 5 + 50 ));
	sprintf(systemCommand, "mv %s %s_old ; cat tmpfile %s_old > %s ; rm %s_old", filename, filename, filename, filename, filename );
	printf("\33[34mExecuting shell command:\n%s\33[0m\n", systemCommand);
	system(systemCommand);
	free(systemCommand);
	//Remove the temporary file
	remove( "tmpfile");


//***********************************
//********One file STATISTICS 
	statisticsFileName = (char*)malloc(sizeof(char) * strlen( filename ) + 20);
	strcpy( statisticsFileName, filename );
	tempChar = strrchr( statisticsFileName, '/');
	if ( ( tempChar ) == NULL )
	{	//There is no path prefixing the given file name
		strcpy( statisticsFileName, "traj_statistics.txt" );
	}
	else
	{	//we place the statistics file in the output folder
		strcpy( statisticsFileName + (int)(tempChar - statisticsFileName) + 1, "traj_statistics.txt\0" );
	}
	//Delete the old file if it exists
	if (filenumber == 0)
	{	remove( statisticsFileName );	}
	//Open the statistics file
	if ( (statsFile =  fopen(statisticsFileName,"a") ) == NULL)
	{ PrintError(("There was an error while opening the statistics file: %s\nNo statistics will be saved.", statisticsFileName));}
	if (filenumber == 0)
	{
		fprintf(statsFile, "#1=Step nb   2=EnergyBefore minimization     3=EnergyAfter Minim from Amber \n");
		fprintf(statsFile, "#4=energyAfter from Move3D    5=NbAtomsClose     6=MinAtomDist   7=AvgAtomDist \n");
		fprintf(statsFile, "#8=VDWAALS   9=EEL   10=HBOND    11=BOND   12=ANGLE    13=DIHED \n");
		fprintf(statsFile, "#14= 1-4VDW    15= 1-4EEL   16=RESTRAINT\n");
		fprintf(statsFile, "#1     	2              	3              	4              	5              	6              	7              	8              	9              	10             	11             	12             	13             	14             	15             	16n");
	}
	fprintf(statsFile, "%6d	%15.5lf	%15.5lf	%15.5lf	%15d	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf	%15.5lf\n", filenumber, energyBeforeMinimization, enAfterMinDirectlyFromAmber[0], energyAfterMinimization, nbAtomsNear, minAtomDist, avgAtomDist, enAfterMinDirectlyFromAmber[1], enAfterMinDirectlyFromAmber[2], enAfterMinDirectlyFromAmber[3], enAfterMinDirectlyFromAmber[4], enAfterMinDirectlyFromAmber[5], enAfterMinDirectlyFromAmber[6], enAfterMinDirectlyFromAmber[7], enAfterMinDirectlyFromAmber[8], enAfterMinDirectlyFromAmber[9] );
	fclose(statsFile);

	free( statisticsFileName );
//***********************************
//***********************************

#endif

}

/* writes a number of files, which are the translation of the current
trajectory of the robot into pdb format. The trajectory is sampled with
a density given by the dmax chosen by the user */

int traj_to_pdb(p3d_rob *robotPt, double dmax, char *radix)
{
  int filenumber=0;
  char whole_name[256];
  
  double u=0.0; 
  double du, umax; /* parameters along the local path */
  configPt q,qp;
  int njnt = robotPt->njoints;
  double *distances;
  int i, end_localpath = 0;
  pp3d_localpath localpathPt;

  if(robotPt->tcur == NULL){
    PrintInfo(("traj_to_pdb: no current trajectory\n"));
    return 0;
  }

  localpathPt = robotPt->tcur->courbePt;
  distances = MY_ALLOC(double, njnt+1);
  qp = p3d_alloc_config(robotPt);

  while (localpathPt != NULL){
    umax = localpathPt->range_param;
    
    while (end_localpath < 2){
      /* begin modif Carl */
      /* dmax = p3d_get_env_dmax(); */ 

      /* end modif Carl */
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      if(u == 0.0) {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt(robotPt);
	// DEBUG
	//g3d_draw_allwin_active();
      }
      else {
	p3d_set_and_update_this_robot_conf_multisol(robotPt,q,qp,P3D_HUGE, NULL);
	// DEBUG
	//g3d_draw_allwin_active();
      }

      /* apply cntrts to qp */
      p3d_get_robot_config_into(robotPt, &qp);
      p3d_destroy_config(robotPt, q);
      
      
      sprintf(whole_name,"%s/%4.4d%s", radix, filenumber,".pdb");
      
      move3d_to_pdb(robotPt, whole_name, filenumber, trajExpMinNONE);
      
      filenumber++;
      
      
      for (i=0; i<=njnt; i++){
	distances[i] = dmax;
      }
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
      u+=du;
      if (u > umax-EPS6){
	u = umax;
	end_localpath++;
      }
    }
    localpathPt = localpathPt->next_lp;
    end_localpath = 0;
    u = 0;
  }
  p3d_destroy_config(robotPt, qp);
  MY_FREE(distances, double, njnt+1);
  return 1;

}


// modif Juan
// writes a pdb file containing a MODEL for each node
int graph_to_pdb(void)
{
  p3d_rob *robotPt;
  p3d_graph *G;
  p3d_list_node *node;
  configPt q;
  FILE *fp;
  int inode;

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  G = robotPt->GRAPH;
  if((G == NULL) || (G->nodes == NULL)) {
    printf("empty graph !!!");
    return 0;
  }
  
  fp = fopen("nodes.pdb","w"); 
      
  node = G->nodes;
  inode = 0;

  while(node != NULL) {
    inode ++;
    q = node->N->q;
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt, q);
    fprintf(fp,"MODEL        %d\n",inode);
    translate_conf_to_pdb(robotPt, fp);    
    node = node->next;
  }
  
  fclose(fp);

  return 1;
}
// fmodif Juan


/* return the number of steps intrajectory produced determined by a dmax given as parameter */

int traj_counter(p3d_rob *robotPt, double dmax){
  double u=0.0; 
  double du, umax; /* parameters along the local path */
  configPt q,qp;
  int njnt = robotPt->njoints;
  double *distances;
  int i, end_localpath = 0;
  pp3d_localpath localpathPt;
  int counter=0;

  if(robotPt->tcur == NULL){
    PrintInfo(("traj_counter: no current trajectory\n"));
    return 0;
  }
  localpathPt = robotPt->tcur->courbePt;
  distances = MY_ALLOC(double, njnt+1);
  qp = p3d_alloc_config(robotPt);

  while (localpathPt != NULL){
    umax = localpathPt->range_param;   
    while (end_localpath < 2){
      /* begin modif Carl */
      /* dmax = p3d_get_env_dmax(); */ 

      /* end modif Carl */
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      if(u == 0.0) {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt(robotPt);
	// DEBUG
	//g3d_draw_allwin_active();
      }
      else {
	p3d_set_and_update_this_robot_conf_multisol(robotPt,q,qp,P3D_HUGE, NULL);
	// DEBUG
	//g3d_draw_allwin_active();
      }
      /* apply cntrts to qp */
      p3d_get_robot_config_into(robotPt, &qp);
      p3d_destroy_config(robotPt, q);    
      
      counter++;  
      
      for (i=0; i<=njnt; i++){
	distances[i] = dmax;
      }
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
      u+=du;
      if (u > umax-EPS6){
	u = umax;
	end_localpath++;
      }
    }
    localpathPt = localpathPt->next_lp;
    end_localpath = 0;
    u = 0;
  }
  p3d_destroy_config(robotPt, qp);
  MY_FREE(distances, double, njnt+1);
  return counter;

}


void serchDmaxAndWrite(pp3d_rob protein, char *radix, int desired){
double step;
int n;
int prec_n;
int i;

//printf("he entrado en SearchD\n");
step = p3d_get_env_graphic_dmax();
//printf("despues de asignar step\n");
n = traj_counter(protein, step);
//printf("despues de la primera llamada a ntraj\n");
prec_n= n;

i=0;
while ((n > desired) && (i< 10)){
	step = step * 2.0;
	prec_n= n;
	n = traj_counter(protein, step);
	i++;
	}
if (i>0){
	step= step / 2.0;
	n= prec_n;
	}
i=0;
while ((n > desired) && (i< 10)){
	step = step *1.2;
	prec_n= n;
	n = traj_counter(protein, step);
	i++;
	}
i=0;
while ((n < desired) && (i< 10)){
	step = step / 2.0;
	prec_n= n;
	n = traj_counter(protein, step);
	i++;
	}
if (i>0){
	step= step * 2.0;
	n= prec_n;
	}
i=0;	
while ((n < desired) && (i< 10)){
	step = step / 1.2;
	prec_n= n;
	n = traj_counter(protein, step);
	i++;
	}
traj_to_pdb(protein,step, radix);
}

//For a description of the <useMinimization>, <useAmberForPDBSave> parameters, 
//   please look at the move3d_to_pdb() method
int bio_write_pdbs_unifom_dep(pp3d_rob robotPt, char *radix, double want_dep_dist, trajExportMinimizationType minimizationType)
{
  int filenumber=0;
  char whole_name[256];
  
  double u=0.0; 
  double du, umax=0.0; /* parameters along the local path */
  configPt q,qp;
  int njnt = robotPt->njoints;
  double *distances;
  int i, end_localpath = 0;
  pp3d_localpath localpathPt, last_localpathPt=NULL;
  double dep_dist = 0.0;
  p3d_vector3 pos, ref_pos, pos_diff;
  int first_conf = 1;

  double dmax = p3d_get_env_graphic_dmax();


  if(robotPt->tcur == NULL){
    PrintInfo(("bio_write_pdbs_unifom_dep: no current trajectory\n"));
    return 0;
  }

  localpathPt = robotPt->tcur->courbePt;
  distances = MY_ALLOC(double, njnt+1);
  qp = p3d_alloc_config(robotPt);

  while (localpathPt != NULL){
    umax = localpathPt->range_param;
    end_localpath = 0;    
    u = 0;
    while (!end_localpath){
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      if(u == 0.0) {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt(robotPt);
	// DEBUG
	//g3d_draw_allwin_active();
      }
      else {
	p3d_set_and_update_this_robot_conf_multisol(robotPt,q,qp,P3D_HUGE, NULL);
	// DEBUG
	//g3d_draw_allwin_active();
      }

      // get position of "chosen" frame
      if(!bio_get_position_meaning_frame(robotPt,pos)) {
	return 0;
      }

      if(first_conf) {
	// ref_pos <- pos
	p3d_vectCopy(pos,ref_pos);
	dep_dist = 0.0;
	first_conf = 0;	
      }
      else {
	// compute diplacement
	p3d_vectSub(pos,ref_pos,pos_diff);
	dep_dist = p3d_vectNorm(pos_diff);
      }

      // approximation : deplacement is always greater than wanted
      if(dep_dist >= want_dep_dist) {
	sprintf(whole_name,"%s/%4.4d%s", radix, filenumber,".pdb");
	move3d_to_pdb(robotPt, whole_name, filenumber, minimizationType);
	filenumber++;
	// ref_pos <- pos
	p3d_vectCopy(pos,ref_pos);
      }

      /* apply cntrts to qp */
      p3d_get_robot_config_into(robotPt, &qp);
      p3d_destroy_config(robotPt, q);
      
      for (i=0; i<=njnt; i++){
	distances[i] = dmax;
      }
      du = localpathPt->stay_within_dist(robotPt, localpathPt,
					 u, FORWARD, distances);
      u+=du;
      if (u > umax-EPS6){
	u = umax;
	end_localpath = 1;
      }
    }
    last_localpathPt = localpathPt;
    localpathPt = localpathPt->next_lp;
  }
  // last config
  q = last_localpathPt->config_at_param(robotPt, last_localpathPt, umax);
  p3d_set_and_update_this_robot_conf_multisol(robotPt,q,qp,P3D_HUGE, NULL);
  sprintf(whole_name,"%s/%4.4d%s", radix, filenumber,".pdb");      
  move3d_to_pdb(robotPt, whole_name, filenumber, minimizationType);

  p3d_destroy_config(robotPt, q);
  p3d_destroy_config(robotPt, qp);
  MY_FREE(distances, double, njnt+1);
  return 1;
}

///////////////////////
//For a description of the <useMinimization>, <useAmberForPDBSave> parameters, 
//   please look at the move3d_to_pdb() method
int bio_write_pdbs_of_N_in_path(pp3d_rob robotPt, char *radix, trajExportMinimizationType minimizationType)
{
  int filenumber=0;
  char whole_name[256];
  
  double u=0.0; 
  configPt q;
  pp3d_localpath localpathPt;


  if(robotPt->tcur == NULL){
    PrintInfo(("bio_write_pdbs_of_N_in_path: no current trajectory\n"));
    return 0;
  }

  localpathPt = robotPt->tcur->courbePt;

  while (localpathPt != NULL){
    u = 0.0;
    q = localpathPt->config_at_param(robotPt, localpathPt, u);
    p3d_set_robot_config(robotPt,q);
    p3d_update_this_robot_pos_without_cntrt(robotPt);

    sprintf(whole_name,"%s/%4.4d%s", radix, filenumber,".pdb");
    move3d_to_pdb(robotPt, whole_name, filenumber, minimizationType);
    filenumber++;

    p3d_destroy_config(robotPt, q);

    localpathPt = localpathPt->next_lp;
  }

  // last config
  q = robotPt->ROBOT_GOTO;
  p3d_set_robot_config(robotPt,q);
  p3d_update_this_robot_pos_without_cntrt(robotPt);
  sprintf(whole_name,"%s/%4.4d%s", radix, filenumber,".pdb");      
  move3d_to_pdb(robotPt, whole_name, filenumber, minimizationType);

  p3d_destroy_config(robotPt, q);

  return 1;
}



// write a pdbfile to display the positions of the ligand's barycenter 
// for all the nodes in the tree
void bio_write_baryplot(void)
{
  FILE *fp;
  p3d_rob *robPt;
  p3d_graph *GPt;
  p3d_compco* compPt;
  p3d_list_node *ListNode;
  p3d_node *N;
  p3d_jnt *refjnt=NULL;
  int indexjnt=0;
  p3d_vector3 posJ;

  robPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  GPt = robPt->GRAPH;
  compPt = GPt->search_start->comp;
  ListNode = compPt->dist_nodes;

  indexjnt = p3d_get_user_drawnjnt();
  if(indexjnt != -1) {
    refjnt = XYZ_ROBOT->joints[indexjnt];
  }
  else {  
    if(p3d_col_get_mode() == p3d_col_mode_bio) {
      refjnt = XYZ_ROBOT->joints[0]->next_jnt[XYZ_ROBOT->joints[0]->n_next_jnt - 1];
    }
    else {
      printf("WARNING : bio_write_baryplot : unable to determine refjnt\n");
      return;
    }
  }


  fp= fopen("baryplot.pdb","w");
  
  while(ListNode != NULL) {
    N = ListNode->N;    
    p3d_set_and_update_this_robot_conf_without_cntrt(robPt,N->q);
    p3d_jnt_get_cur_vect_point(refjnt,posJ);


    // print PDB atom line
    fprintf(fp,"%-6s","HETATM");
    fprintf(fp,"%5d", N->num);    
    fprintf(fp,"%2s","  ");
    if(N->n_fail_extend < 5)
      fprintf(fp,"%-3s","C");
    else if (N->n_fail_extend < 10)
      fprintf(fp,"%-3s","N");
    else 
      fprintf(fp,"%-3s","0");
    fprintf(fp,"%4s", "LBC");
    fprintf(fp,"%1s"," ");
    fprintf(fp,"%1s", "X");
    fprintf(fp,"%4d", 0);    
    fprintf(fp,"%12.3f", posJ[0]);
    fprintf(fp,"%8.3f", posJ[1]);
    fprintf(fp,"%8.3f", posJ[2]);
    fprintf(fp,"%26s","");
    fprintf(fp,"\n");   
    fprintf(fp,"END    \n");   

    ListNode = ListNode->next;
  }

  fclose(fp);	
}
