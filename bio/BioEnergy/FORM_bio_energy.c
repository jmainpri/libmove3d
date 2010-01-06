
// Description: 
//    This file should contain ONLY INTERFACE elements and the associated calls to their respective functions.
//    The functions themselves should NOT be written here, but in the appropriate .c file (nmode or minimization)
//    The common variables declarations should be placed in the bioenergy_common.h file.
//    Also, please keep in mind the flow of information (ie. control the user's actions at any time).



#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"

#include "Planner-pkg.h"

#include "Bio-pkg.h"

#include "include/Energy-pkg.h"

 
/*********************/
// Choose ligand type section
static     FL_OBJECT  *GROUP_LIGAND_TYPE_OBJ; 
static     FL_OBJECT  *PROTEIN_FILE_NAME_OBJ; 


/*********************/
// normal mode data

FL_OBJECT *AMPLITUDE_NORMAL_MODE_SLIDER_OBJ;
FL_OBJECT *NUMBER_VECTOR_SLIDER_OBJ;
FL_OBJECT *NORMAL_MODE_ONE_SLIDER_OBJ;
FL_OBJECT *NORMAL_MODE_TWO_SLIDER_OBJ;
FL_OBJECT *NORMAL_MODE_THREE_SLIDER_OBJ;
FL_OBJECT *NORMAL_MODE_FOUR_SLIDER_OBJ;
FL_OBJECT *NORMAL_MODE_FIVE_SLIDER_OBJ;

FL_OBJECT *ELNEMO_PROT_BTN_OBJ;
FL_OBJECT *ELNEMO_LIG_BTN_OBJ;

FL_OBJECT *ELNEMO_COLDEG_PLANNER_OBJ;
FL_OBJECT *ELNEMO_GOAL_P3D_OBJ;

FL_OBJECT *BTN_PRECALCULATIONS_OBJ = NULL;
//FL_OBJECT *FILE_ELNEMO_BTN_OBJ;




/**************************/
// energy calculation data

FL_FORM    *BIO_ENERGY_FORM = NULL;  
	
static     FL_OBJECT  *PDB_FILE;			
static     FL_OBJECT  *ATOMES;			
static     FL_OBJECT  *PRE_FILES;
static     FL_OBJECT  *PRE_FILES1;
static     FL_OBJECT  *RADIO_NO_LIGAND_OBJ;
static     FL_OBJECT  *PARAMETERS;				
static     FL_OBJECT  *PARM_COORD;
static     FL_OBJECT  *CALCOLATION;
static     FL_OBJECT  *E_ALONG_PATH;
static     FL_OBJECT  *MINIMIZATIONDM;
static     FL_OBJECT  *GROUP_MINIMIZATION; 
static     FL_OBJECT  *MINI_ENE_OBJ; 
static     FL_OBJECT  *MINI_SD_OBJ;
static     FL_OBJECT  *MINI_CG_OBJ;
static     FL_OBJECT  *MINI_SD_OBJ;
static     FL_OBJECT  *MINI_MD_OBJ;
static     FL_OBJECT  *MINI_START_OBJ;
static     FL_OBJECT  *MINI_NM_OF_CYCLES;
static     FL_OBJECT  *MINI_RRT_OBJ;
static     FL_OBJECT  *LIG_FRAME_OBJ;
static     FL_OBJECT  *LIG_BOX_OBJ;
static     FL_OBJECT  *PROT_FRAME_OBJ;
static     FL_OBJECT  *PROT_BOX_OBJ;
static     FL_OBJECT  *COMB_FRAME_OBJ;
static     FL_OBJECT  *COMB_BOX_OBJ;
static     FL_OBJECT  *ENER_FRAME_OBJ;
static     FL_OBJECT  *ENER_BOX_OBJ;
//static     FL_OBJECT  *ob;



extern void g3d_draw_allwin_active( );
extern  void pair_at_min_sd(p3d_poly *a1,p3d_poly *a2);	

static void CB_radioNormalModeExploration_OnChange(FL_OBJECT *ob, long arg);
static void CB_btnPreComputation_OnClick(FL_OBJECT *ob, long arg);
static void CB_amplitude_normal_mode_slider_obj(FL_OBJECT *object, long arg);
static void CB_normal_mode_one_slider_obj(FL_OBJECT *object, long arg);
static void CB_number_vector_slider_obj(FL_OBJECT *object, long arg);
static void CB_normal_mode_one_slider_obj(FL_OBJECT *object, long arg);
static void CB_normal_mode_two_slider_obj(FL_OBJECT *object, long arg);
static void CB_normal_mode_three_slider_obj(FL_OBJECT *object, long arg);
static void CB_normal_mode_four_slider_obj(FL_OBJECT *object, long arg);
static void CB_normal_mode_five_slider_obj(FL_OBJECT *object, long arg);

static void CB_coldeg_planner_btn_obj(FL_OBJECT *object, long arg);
static void CB_goal_p3d_btn_obj(FL_OBJECT *object, long arg);

static int QuestionUser( char *question, int defaultBtn );
static void AlertUser( char *title, char *text1, char *text2);

// CB functions for energy calculation 
static void    CB_btnLoadProtein_OnClick(FL_OBJECT *ob, long arg);
static void    CB_RadioChooseLigandType_OnChange(FL_OBJECT *ob, long ar);
static void    CB_btnComputeEnergy_OnClick(FL_OBJECT *ob, long arg);
static void    CB_btnEAlongPath(FL_OBJECT *ob, long arg);
static void    CB_mol_dynamique(FL_OBJECT *ob, long arg);
static void    CB_radioMinimizationType_OnChange(FL_OBJECT *ob, long arg); 
static void    CB_DoMinimization_OnClick (FL_OBJECT *ob, long arg);
static void    CB_NbOfCycles_OnClick(FL_OBJECT *ob, long arg);
static void    CB_RRT_minimization(FL_OBJECT *ob, long arg); 


// TO BE MOVED TO ANOTHER FILE !!!
/************************************************************************/
static int bio_compute_potential_energy_along_path( p3d_rob *robotPt, p3d_traj *trjPt, FILE *fEpath);


//Miscelaneous methods
void myEnableDisableObject( FL_OBJECT *obj, int enable );


static void    CB_btnTest_OnClick(FL_OBJECT *ob, long arg)
{
	FL_FORM *tempForm;

	tempForm =fl_get_app_mainform();
   	fl_set_app_mainform(BIO_ENERGY_FORM);

	fl_show_alert("Notification", "Nothing happens here !", "", 0);

   	fl_set_app_mainform(tempForm);
}






void g3d_create_bio_energy_form(void) 
{ 
	//The currentY variable will be used as a sort of cursor that indicates the current Y
	// position within the form. Its use will allow easier arrangements of objects on the form
	int currentY = 0;
	char tempLabel[100];
  FL_OBJECT *obj;

  BIO_ENERGY_FORM = fl_bgn_form(FL_UP_BOX,400.0,650.0);

//=====================================
//   Choose type of ligand section
	currentY += 10;

  GROUP_LIGAND_TYPE_OBJ = fl_bgn_group();
  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,currentY+5,380,40,""); 
  obj = fl_add_box(FL_FLAT_BOX,40,currentY,110,10,"Choose ligand type :");

  RADIO_NO_LIGAND_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,40,currentY+10,50,30,"None");
  fl_set_button(RADIO_NO_LIGAND_OBJ,1);
  fl_set_object_color(RADIO_NO_LIGAND_OBJ,FL_MCOL,FL_GREEN);
  fl_set_call_back(RADIO_NO_LIGAND_OBJ,CB_RadioChooseLigandType_OnChange,0);

  PRE_FILES1 = fl_add_checkbutton(FL_RADIO_BUTTON,150,currentY+10,50,30,"Non-Protein");
  fl_set_object_color(PRE_FILES1,FL_MCOL,FL_GREEN);
  fl_set_call_back(PRE_FILES1,CB_RadioChooseLigandType_OnChange,1);

  PRE_FILES  = fl_add_checkbutton(FL_RADIO_BUTTON,250,currentY+10,50,30,"Protein");
  fl_set_object_color(PRE_FILES,FL_MCOL,FL_GREEN);
  fl_set_call_back(PRE_FILES ,CB_RadioChooseLigandType_OnChange,2);

  //GROUP_LIGAND_TYPE_OBJ = 
  fl_end_group();

//=====================================
//   Load protein and ligand section
	currentY += 50;
  PDB_FILE = fl_add_button(FL_NORMAL_BUTTON, 10,currentY,100,30,  "Protein" );
  fl_set_call_back(PDB_FILE,CB_btnLoadProtein_OnClick,0); 

  PROTEIN_FILE_NAME_OBJ = fl_add_input(FL_NORMAL_INPUT,120,currentY+5,270,22, "");
  fl_set_input(PROTEIN_FILE_NAME_OBJ, "<- Load protein PDB file");
  fl_deactivate_object(PROTEIN_FILE_NAME_OBJ);

//=====================================
//   create_calcul_energy_obj  section

	currentY += 50;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,currentY,380,45,""); 
  obj = fl_add_box(FL_FLAT_BOX,40,currentY-5,90,10,"Energy related:");

  CALCOLATION = fl_add_button(FL_NORMAL_BUTTON, 15,currentY+10,100,30,"Calculate energy ");
  fl_set_call_back(CALCOLATION, CB_btnComputeEnergy_OnClick, 0);	

  E_ALONG_PATH = fl_add_button(FL_NORMAL_BUTTON, 120.0,currentY+10,100.0,30.0,
			       "Energy along Path ");
  fl_set_call_back(E_ALONG_PATH, CB_btnEAlongPath, 0);	

  MINIMIZATIONDM = fl_add_button(FL_NORMAL_BUTTON, 285,currentY+10,100,30, "Molecular Dynamics");
 fl_set_call_back(MINIMIZATIONDM, CB_mol_dynamique, 0);	

//=====================================
//   create_minimization_obj  section

	currentY += 65;	//200
  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,currentY,300,30,""); 
  obj = fl_add_box(FL_FLAT_BOX,30,currentY-5,120,10,"Classic Minimization :");

  GROUP_MINIMIZATION = fl_bgn_group();
  MINI_ENE_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,15,currentY,50,30,"SD+CG");
  fl_set_button(MINI_ENE_OBJ, 1);
  fl_set_object_color(MINI_ENE_OBJ,FL_MCOL,FL_GREEN);
   fl_set_call_back(MINI_ENE_OBJ,CB_radioMinimizationType_OnChange,0);
  MINI_SD_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,80,currentY,50,30,"SD");
  fl_set_object_color(MINI_SD_OBJ,FL_MCOL,FL_GREEN);
   fl_set_call_back(MINI_SD_OBJ,CB_radioMinimizationType_OnChange,1);
  MINI_CG_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,120,currentY,50,30,"CG");
  fl_set_object_color(MINI_CG_OBJ,FL_MCOL,FL_GREEN);
   fl_set_call_back(MINI_CG_OBJ,CB_radioMinimizationType_OnChange,2);

  sprintf(tempLabel, "Cyc:%d", GetNbMinimizationCycles());
  MINI_NM_OF_CYCLES = fl_add_button(FL_PUSH_BUTTON,180,currentY+5,66,20,tempLabel);
   fl_set_call_back(MINI_NM_OF_CYCLES,CB_NbOfCycles_OnClick,0);

  MINI_START_OBJ = fl_add_button(FL_PUSH_BUTTON,255,currentY+5,50,20,"Start");
   fl_set_call_back(MINI_START_OBJ,CB_DoMinimization_OnClick,0);

  MINI_RRT_OBJ = fl_add_button(FL_PUSH_BUTTON,315,currentY,70,30,"RRT-based\nMinimization");
   fl_set_call_back(MINI_RRT_OBJ,CB_RRT_minimization,0);
  //GROUP_MINIMIZATION = 
  fl_end_group();


//=====================================
//   Normal modes section

	currentY += 50;//250;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,10,currentY,380,380,""); 
  obj = fl_add_box(FL_FLAT_BOX,50,currentY-5,80,10,"Normal modes :");

   //////////////////////

  BTN_PRECALCULATIONS_OBJ = fl_add_button(FL_PUSH_BUTTON, 20,currentY+15,100,25, "PreComputations"); 
  fl_set_call_back(BTN_PRECALCULATIONS_OBJ, CB_btnPreComputation_OnClick,0);

 obj = fl_bgn_group();
  ELNEMO_PROT_BTN_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,130,currentY+15,45,25,"Protein");
  fl_set_object_color(ELNEMO_PROT_BTN_OBJ, FL_MCOL, FL_GREEN);
   fl_set_call_back(ELNEMO_PROT_BTN_OBJ,CB_radioNormalModeExploration_OnChange,0);
  ELNEMO_LIG_BTN_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,190,currentY+15,45,25,"Ligand");
  fl_set_object_color(ELNEMO_LIG_BTN_OBJ, FL_MCOL, FL_GREEN);
   fl_set_call_back(ELNEMO_LIG_BTN_OBJ,CB_radioNormalModeExploration_OnChange,1);
  //obj = 
  fl_end_group();

  ELNEMO_COLDEG_PLANNER_OBJ = fl_add_button(FL_PUSH_BUTTON, 310,currentY+13,70,30, "Explore\nCollDegs"); 
  fl_set_call_back(ELNEMO_COLDEG_PLANNER_OBJ, CB_coldeg_planner_btn_obj,0);

  ELNEMO_GOAL_P3D_OBJ = fl_add_button(FL_PUSH_BUTTON, 260,currentY+13,50,30, "Select\ngoal p3d"); 
  fl_set_call_back(ELNEMO_GOAL_P3D_OBJ, CB_goal_p3d_btn_obj,0);

//////////////Frame for 1DOF exploration
	currentY += 50;//300;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,currentY,360,90,""); 
  obj = fl_add_box(FL_FLAT_BOX,40,currentY-10,140,30,"");
  obj = fl_add_checkbutton(FL_RADIO_BUTTON,40,currentY-15,50,30,"1DOF exploration:");
//   fl_set_call_back(obj,CB_radioNormalModeExploration_OnChange,0);

  NUMBER_VECTOR_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+20,340,20,"Eigenvector");
  fl_set_object_lalign(NUMBER_VECTOR_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step( NUMBER_VECTOR_SLIDER_OBJ,1);
  fl_set_slider_bounds( NUMBER_VECTOR_SLIDER_OBJ,7,30);
  fl_set_object_callback( NUMBER_VECTOR_SLIDER_OBJ,CB_number_vector_slider_obj,0);

  AMPLITUDE_NORMAL_MODE_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+60,340,20,"Amplitude");
  fl_set_object_lalign(AMPLITUDE_NORMAL_MODE_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step(AMPLITUDE_NORMAL_MODE_SLIDER_OBJ,0.01);
  fl_set_slider_bounds(AMPLITUDE_NORMAL_MODE_SLIDER_OBJ,-1.0,1.0);
  fl_set_slider_value(AMPLITUDE_NORMAL_MODE_SLIDER_OBJ,0.0);
  fl_set_object_callback(AMPLITUDE_NORMAL_MODE_SLIDER_OBJ,CB_amplitude_normal_mode_slider_obj,0);


//////////////Frame for 5DOF exploration
	currentY += 105;//415;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,20,currentY,360,215,""); 
  obj = fl_add_box(FL_FLAT_BOX,40,currentY-10,140,30,"");
  obj = fl_add_checkbutton(FL_RADIO_BUTTON,40,currentY-15,50,30,"5DOF exploration:");
//   fl_set_call_back(obj,CB_radioNormalModeExploration_OnChange,1);
  
  NORMAL_MODE_ONE_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+25,340,20,"NM7 - amplitude");
  fl_set_object_lalign(NORMAL_MODE_ONE_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step( NORMAL_MODE_ONE_SLIDER_OBJ,0.01);
  fl_set_slider_bounds( NORMAL_MODE_ONE_SLIDER_OBJ,-1.0,1.0);
  fl_set_slider_value( NORMAL_MODE_ONE_SLIDER_OBJ,0.0);
  fl_set_object_callback( NORMAL_MODE_ONE_SLIDER_OBJ,CB_normal_mode_one_slider_obj,0);

  NORMAL_MODE_TWO_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+65,340,20,"NM8 - amplitude");
  fl_set_object_lalign(NORMAL_MODE_TWO_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step( NORMAL_MODE_TWO_SLIDER_OBJ,0.01);
  fl_set_slider_bounds( NORMAL_MODE_TWO_SLIDER_OBJ,-1.0,1.0);
  fl_set_slider_value( NORMAL_MODE_TWO_SLIDER_OBJ,0.0);
  fl_set_object_callback( NORMAL_MODE_TWO_SLIDER_OBJ,CB_normal_mode_two_slider_obj,0);


  NORMAL_MODE_THREE_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+105,340,20,"NM9 - amplitude");
  fl_set_object_lalign(NORMAL_MODE_THREE_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step( NORMAL_MODE_THREE_SLIDER_OBJ,0.01);
  fl_set_slider_bounds( NORMAL_MODE_THREE_SLIDER_OBJ,-1.0,1.0);
  fl_set_slider_value( NORMAL_MODE_THREE_SLIDER_OBJ,0.0);
  fl_set_object_callback( NORMAL_MODE_THREE_SLIDER_OBJ,CB_normal_mode_three_slider_obj,0);

  NORMAL_MODE_FOUR_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+145,340,20,"NM10 - amplitude");
  fl_set_object_lalign(NORMAL_MODE_FOUR_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step( NORMAL_MODE_FOUR_SLIDER_OBJ,0.01);
  fl_set_slider_bounds( NORMAL_MODE_FOUR_SLIDER_OBJ,-1.0,1.0);
  fl_set_slider_value( NORMAL_MODE_FOUR_SLIDER_OBJ,0.0);
  fl_set_object_callback( NORMAL_MODE_FOUR_SLIDER_OBJ,CB_normal_mode_four_slider_obj,0);

  NORMAL_MODE_FIVE_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,30,currentY+185,340,20,"NM11 - amplitude");
  fl_set_object_lalign(NORMAL_MODE_FIVE_SLIDER_OBJ,FL_ALIGN_LEFT_TOP);
  fl_set_slider_step( NORMAL_MODE_FIVE_SLIDER_OBJ,0.01);
  fl_set_slider_bounds( NORMAL_MODE_FIVE_SLIDER_OBJ,-1.0,1.0);
  fl_set_slider_value( NORMAL_MODE_FIVE_SLIDER_OBJ,0.0);
  fl_set_object_callback( NORMAL_MODE_FIVE_SLIDER_OBJ,CB_normal_mode_five_slider_obj,0);


	currentY += 230;
  obj = fl_add_button(FL_NORMAL_BUTTON, 10,currentY,200,30,  "Test button" );
  fl_set_call_back(obj, CB_btnTest_OnClick, 0); 

  fl_end_form();



//Disable the objects that are only available after loading the protein
myEnableDisableObject( CALCOLATION, FALSE);
myEnableDisableObject( E_ALONG_PATH, FALSE);
myEnableDisableObject( MINI_ENE_OBJ, FALSE);
myEnableDisableObject( MINI_SD_OBJ, FALSE);
myEnableDisableObject( MINI_START_OBJ, FALSE);
myEnableDisableObject( MINI_CG_OBJ, FALSE);
myEnableDisableObject( MINIMIZATIONDM, FALSE);


myEnableDisableObject( BTN_PRECALCULATIONS_OBJ, FALSE);
myEnableDisableObject( AMPLITUDE_NORMAL_MODE_SLIDER_OBJ, FALSE);
myEnableDisableObject( NUMBER_VECTOR_SLIDER_OBJ, FALSE);
myEnableDisableObject( ELNEMO_PROT_BTN_OBJ, FALSE);
myEnableDisableObject( ELNEMO_LIG_BTN_OBJ, FALSE);
myEnableDisableObject( ELNEMO_COLDEG_PLANNER_OBJ, FALSE);
myEnableDisableObject( ELNEMO_GOAL_P3D_OBJ, FALSE);
myEnableDisableObject( NORMAL_MODE_ONE_SLIDER_OBJ, FALSE);
myEnableDisableObject( NORMAL_MODE_TWO_SLIDER_OBJ, FALSE);
myEnableDisableObject( NORMAL_MODE_THREE_SLIDER_OBJ, FALSE);
myEnableDisableObject( NORMAL_MODE_FOUR_SLIDER_OBJ, FALSE);
myEnableDisableObject( NORMAL_MODE_FIVE_SLIDER_OBJ, FALSE);
}

void myEnableDisableObject( FL_OBJECT *obj, int enable )
{
	if (enable)
	{
		//Call the actual XForms deactivation method
		fl_activate_object( obj );
		//and then change the color so that the user makes a difference
		fl_set_object_color( obj, FL_BUTTON_COL1, FL_BUTTON_COL1);
	}
	else
	{
		//Call the actual XForms activation method
		fl_deactivate_object( obj );
		//and then change the color so that the user makes a difference
		fl_set_object_color( obj, FL_MCOL, FL_MCOL);
	}
}


void 
g3d_delete_bio_energy_form(void)
{
  
  fl_free_object(PDB_FILE); 
  fl_free_object(ATOMES); 
  fl_free_object(PRE_FILES);
  fl_free_object(PRE_FILES1);
  fl_free_object(PARAMETERS); 
  fl_free_object(PARM_COORD);
  fl_free_object(CALCOLATION); 
  fl_free_object(E_ALONG_PATH); 
  fl_free_object(MINIMIZATIONDM);
  fl_free_object(MINI_ENE_OBJ); 
  fl_free_object(MINI_SD_OBJ); 
  fl_free_object(MINI_START_OBJ); 
  fl_free_object(MINI_CG_OBJ); 
  fl_free_object(MINI_MD_OBJ); 
  fl_free_object(PROT_FRAME_OBJ); 
  fl_free_object(PROT_BOX_OBJ);
  fl_free_object(LIG_FRAME_OBJ); 
  fl_free_object(LIG_BOX_OBJ);
  fl_free_object(COMB_FRAME_OBJ); 
  fl_free_object(COMB_BOX_OBJ);
  fl_free_object(ENER_FRAME_OBJ); 
  fl_free_object(ENER_BOX_OBJ);

  // TO DO : FREE OF ELNEMO OBJs !!!! 

  // fl_free_object(ob);
  fl_free_form(BIO_ENERGY_FORM);

}


void 
CB_btnLoadProtein_OnClick (FL_OBJECT *ob, long arg)
{ 
	FILE *fTest;
	char  *proteinFile = NULL, *ligandFile = NULL;
	const char *tempPtr;
	static char messageToAskForLigand[200];	//Make sure this is greater then the message length below
	FL_FORM *tempForm;

	tempForm =fl_get_app_mainform();
   	fl_set_app_mainform(BIO_ENERGY_FORM);
	
	//##############################################################3
	//######### Ask for the PROTEIN FILE
	
 
#ifdef ALIN
  	//tempPtr =strdup( "/home/astefani/BioMove3D/demo/ProteinModels/ar/a8r_prot.pdb");
  	tempPtr = "/home/astefani/BioMove3D/demo/ProteinModels/xyl_min.pdb";
#else

	/* Ask the user for the file to be open  */
	tempPtr = fl_show_fselector("Enter the protein's PDB file:","../../Energy/","*.pdb", ""); 
#endif



	if ( tempPtr == NULL )
	{
	   	fl_set_app_mainform(tempForm);
		return;	//The user has pressed CANCEL in the file selector window
	}
	proteinFile = strdup( tempPtr );
	if (! (fTest = fopen(proteinFile, "r") ) )
	{
		fl_show_alert("The file could not be opened !","File does not exist or you do not have the right to open it.",proteinFile,1);
		return;
	}
	fclose(fTest);

	//##############################################################3
	//######### If it is the case ask for the LIGAND also
	if ( ( GetCurrentLigandType() == ligtypeNON_PROTEIN ) ||
		( GetCurrentLigandType() == ligtypePROTEIN ))
	{
		if ( GetCurrentLigandType() == ligtypeNON_PROTEIN )
			strcpy(messageToAskForLigand, "Enter PDB file for NON-protein ligand ...");
		else	
			strcpy(messageToAskForLigand, "Enter PDB file for the second Protein ...");

		#ifdef ALIN
			tempPtr = strdup("/home/astefani/BioMove3D/demo/ProteinModels/ar/a8r_lig.pdb");
		#else
			//Ask the user for the filename
			tempPtr = fl_show_fselector(messageToAskForLigand,"../../Energy/","*.pdb", "");
		#endif


		if ( tempPtr == NULL )
		{
		   	fl_set_app_mainform(tempForm);
			return;	//The user has pressed CANCEL in the file selector window
		}
		ligandFile = strdup( tempPtr );
		if (! (fTest = fopen(ligandFile, "r") ) )
		{
			fl_show_alert("The file could not be opened !","File does not exist or you do not have the right to open it.",ligandFile,1);
			fl_set_app_mainform(tempForm);
			return;
		}
		fclose(fTest);
	}

	if ( InitializeProtein( proteinFile, ligandFile, AlertUser, QuestionUser ) != 0 )
	{	//It means that the execution failed, so I think we should warn the user
		fl_show_alert("The initialization failed !","","",1);
		fl_set_app_mainform(tempForm);
		return;
	}	

   	fl_set_app_mainform(tempForm);
	
	fl_set_input(PROTEIN_FILE_NAME_OBJ, proteinFile);


	//If all went well and we got through to here, then we can enable some more buttons for the user to click
	myEnableDisableObject( CALCOLATION, TRUE);
	myEnableDisableObject( E_ALONG_PATH, TRUE);
	myEnableDisableObject( MINI_ENE_OBJ, TRUE);
	myEnableDisableObject( MINI_SD_OBJ, TRUE);
	myEnableDisableObject( MINI_START_OBJ, TRUE);
	myEnableDisableObject( MINI_CG_OBJ, TRUE);
	myEnableDisableObject( MINIMIZATIONDM, TRUE);
	myEnableDisableObject( ELNEMO_PROT_BTN_OBJ, TRUE);


	//Now deactivate the button for loading the protein PDB files
	// because it does not make sense to change them. Also, we 
	// will disable the "Choose ligand type" radio buttons cause the user can 
	// mess up the code if he changes something
	myEnableDisableObject( PDB_FILE, FALSE);
	myEnableDisableObject( PRE_FILES1, FALSE);
	myEnableDisableObject( PRE_FILES, FALSE);
	myEnableDisableObject( RADIO_NO_LIGAND_OBJ, FALSE);


	//If there is a ligand chosen and is of type Protein, then allow also the Normal mode for the ligand
	if ( GetCurrentLigandType() == ligtypePROTEIN )
	{
		myEnableDisableObject( ELNEMO_LIG_BTN_OBJ, TRUE);
	}
}


void CB_RadioChooseLigandType_OnChange(FL_OBJECT *ob, long arg)
{
	switch( arg )
	{
		case 0:
			SetCurrentLigandType( ligtypeNONE);
			break;
		case 1:
			SetCurrentLigandType( ligtypeNON_PROTEIN);
			break;
		case 2:
			SetCurrentLigandType( ligtypePROTEIN);
			break;
	}
} 
 
void
CB_btnComputeEnergy_OnClick(FL_OBJECT *ob, long arg)
{  
  p3d_rob *mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  double totE;

  if ( CalculateEnergy(mol,&totE) != 0 )
	{	//It means that the execution failed, so I think we should warn the user
		fl_show_alert("The execution failed !","","",1);
	}
}

void
CB_btnEAlongPath(FL_OBJECT *ob, long arg)
{  
  p3d_rob *robotPt;
  p3d_traj *trjPt;
  FILE *fEpath;
  char  *EpathFileName = NULL;
  const char *tempPtr = NULL;
  
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  trjPt = robotPt->tcur;
  if(trjPt == NULL) {
    PrintInfo(("bio_evaluate_traj : no current trajectory\n"));
    return;
  }
  
  tempPtr = fl_show_fselector("Enter a name for the file saving E along the path:","../../Energy/","*.txt", ""); 
  //Check whether the file can be opened or not
  if ( tempPtr == NULL )
    return;	//The user has pressed CANCEL in the file selector window
  
  EpathFileName = strdup( tempPtr );
  if (! (fEpath = fopen(EpathFileName, "w") ) ) {
    fl_show_alert("The file could not be opened !",
		  "File does not exist or you do not have the right to open it.",EpathFileName,1);
    return;
  }
  
  bio_compute_potential_energy_along_path(robotPt,trjPt,fEpath);
  
  fclose(fEpath);
  
  PrintInfo(("E along current trajectory saved to file: %s\n", EpathFileName));
}

void    
CB_mol_dynamique(FL_OBJECT *ob, long arg)
{
	if ( MolecularDynamics() != 0 )
	{	//It means that the execution failed, so I think we should warn the user
		fl_show_alert("The execution failed !","","",1);
	}
}


static void CB_radioMinimizationType_OnChange (FL_OBJECT *ob, long arg)
{
	switch (arg){
	case 0: 
		SetMinimizationMethod( minimSDCG );
		break;
	case 1:
		SetMinimizationMethod( minimSD );
		break;
	case 2: 
		SetMinimizationMethod( minimCG );
		break;
	default: //Make sure we have received good parameters
		PrintError(("arg received with incorrect value. \n"));
		fl_show_alert("Error !","Unknown minimisation method required or parameter\n out of range!","",1);
	}
}


static void CB_NbOfCycles_OnClick(FL_OBJECT *ob, long arg)
{
	char userInput[100];
	const char *s;
	int newValue;
	FL_FORM *tempForm;

	tempForm =fl_get_app_mainform();
   	fl_set_app_mainform(BIO_ENERGY_FORM);

	strcpy(userInput,(s=fl_show_input("Give new number of minimization cycles:","")) ? s:"");

	if ( (strlen(userInput) > 0) && ( sscanf(userInput, "%d", &newValue) == 1) && ( (newValue >= 1) && (newValue<=100000)) )
	{
		SetNbMinimizationCycles( newValue );
		sprintf(userInput, "Cyc:%d", newValue);
		fl_set_object_label(MINI_NM_OF_CYCLES, userInput);
	}
	fl_set_button(MINI_NM_OF_CYCLES, 0);

   	fl_set_app_mainform(tempForm);
}


static void CB_DoMinimization_OnClick (FL_OBJECT *ob, long arg)
{
  double energ;
	p3d_rob *mol = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

	if ( EnergyMinimization( mol, NULL, &energ ) != 0 )
	{	//It means that the execution failed, so I think we should warn the user
		fl_show_alert("The execution failed !","","",1);
	}

	//update the onscreen configuration
	p3d_update_this_robot_pos_without_cntrt(mol);
	g3d_draw_allwin_active( );

	fl_set_button(MINI_START_OBJ, 0);
}


/* booleen d'arret */
static int STOP = FALSE;

static void CB_RRT_minimization (FL_OBJECT *ob, long arg)
{
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  configPt qs;
  int res;
  int *iksols=NULL;
   
  STOP = FALSE;  
  //qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  qs = p3d_get_robot_config(robotPt);

  //MY_ALLOC_INFO("Avant la creation du graphe");
  res = bio_rrt_E_minimize(qs,iksols,fct_stop,fct_draw);

  p3d_destroy_config(robotPt, qs);

  p3d_del_graph(robotPt->GRAPH);

  fl_set_button(ob,0);

  //  g3d_draw_allwin_active();
  fl_ringbell(0);
}





static void CB_normal_mode_one_slider_obj(FL_OBJECT *object, long arg)
{
  double val;
  val = fl_get_slider_value(NORMAL_MODE_ONE_SLIDER_OBJ);
  dof5SliderMoved( 1, val );
}
static void CB_normal_mode_two_slider_obj(FL_OBJECT *object, long arg)
{
  double val;
  val = fl_get_slider_value(NORMAL_MODE_TWO_SLIDER_OBJ);
  dof5SliderMoved( 2, val );
}
static void CB_normal_mode_three_slider_obj(FL_OBJECT *object, long arg)
{
  double val;
  val = fl_get_slider_value(NORMAL_MODE_THREE_SLIDER_OBJ);
  dof5SliderMoved( 3, val );
}
static void CB_normal_mode_four_slider_obj(FL_OBJECT *object, long arg)
{
  double val;
  val = fl_get_slider_value(NORMAL_MODE_FOUR_SLIDER_OBJ);
  dof5SliderMoved( 4, val );
}
static void CB_normal_mode_five_slider_obj(FL_OBJECT *object, long arg)
{
  double val;
  val = fl_get_slider_value(NORMAL_MODE_FIVE_SLIDER_OBJ);
  dof5SliderMoved( 5, val );
}



static void CB_number_vector_slider_obj(FL_OBJECT *object, long arg)
{
	int sliderValue = fl_get_slider_value(NUMBER_VECTOR_SLIDER_OBJ);
  	SetCurrentEigenVector( sliderValue );
}


static void CB_amplitude_normal_mode_slider_obj(FL_OBJECT *object, long arg)
{
  double val;

  val = fl_get_slider_value(AMPLITUDE_NORMAL_MODE_SLIDER_OBJ);
 
  bio_flex_molecule( val); 
}



static int proteinNMInitializationAlreadyDone = FALSE;
static int ligandNMInitializationAlreadyDone = FALSE;
static void CB_radioNormalModeExploration_OnChange(FL_OBJECT *ob, long arg)
{
	int newStateOfObjects = FALSE;

	switch( arg )
	{	
		case 0:
			newStateOfObjects = proteinNMInitializationAlreadyDone;
			if ( proteinNMInitializationAlreadyDone ) { SetCurrentExplorationMode( explorationPROTEIN ); }
			break;
		case 1:
			newStateOfObjects = ligandNMInitializationAlreadyDone;
			if ( ligandNMInitializationAlreadyDone ) { SetCurrentExplorationMode( explorationLIGAND ); }
			break;
		default:
			printf("Unknow argument value. Please see file %s at line %d !", __FILE__, __LINE__);
			exit( (int)2);
	}	

	//Enable the 1DOF sliders
	myEnableDisableObject( AMPLITUDE_NORMAL_MODE_SLIDER_OBJ, newStateOfObjects);
	myEnableDisableObject( NUMBER_VECTOR_SLIDER_OBJ, newStateOfObjects);
	
	//Enable the 5DOF sliders
	myEnableDisableObject( NORMAL_MODE_ONE_SLIDER_OBJ, newStateOfObjects);
	myEnableDisableObject( NORMAL_MODE_TWO_SLIDER_OBJ, newStateOfObjects);
	myEnableDisableObject( NORMAL_MODE_THREE_SLIDER_OBJ, newStateOfObjects);
	myEnableDisableObject( NORMAL_MODE_FOUR_SLIDER_OBJ, newStateOfObjects);
	myEnableDisableObject( NORMAL_MODE_FIVE_SLIDER_OBJ, newStateOfObjects);

	//The state of the PreCalculation button will always be the invers of the others
	myEnableDisableObject( BTN_PRECALCULATIONS_OBJ, !newStateOfObjects);
}

static void CB_btnPreComputation_OnClick(FL_OBJECT *ob, long arg)
{
	int initializationResult = -1;
	//If the Protein radio button is selected
	if ( fl_get_button(ELNEMO_PROT_BTN_OBJ) )
	{
		initializationResult = InitializeNormalModes( explorationPROTEIN, AlertUser, QuestionUser );
		proteinNMInitializationAlreadyDone = TRUE;
	}
	else 	//If the Ligand radio button is selected
	if ( fl_get_button(ELNEMO_LIG_BTN_OBJ) )
	{
		initializationResult = InitializeNormalModes( explorationLIGAND, AlertUser, QuestionUser  );
		ligandNMInitializationAlreadyDone = TRUE;
	}	
	else
		printf("Unknow buttons states !!! Please see file %s at line %d !", __FILE__, __LINE__);

	if ( initializationResult != 0 )
	{	//It means that the execution failed, so I think we should warn the user
		fl_show_alert("The initialization failed !","","",1);
		return;
	}

	// Enable exploration
	myEnableDisableObject( ELNEMO_COLDEG_PLANNER_OBJ, TRUE);
	myEnableDisableObject( ELNEMO_GOAL_P3D_OBJ, TRUE);

	//Enable the 1DOF sliders
	myEnableDisableObject( AMPLITUDE_NORMAL_MODE_SLIDER_OBJ, TRUE);
	myEnableDisableObject( NUMBER_VECTOR_SLIDER_OBJ, TRUE);
	
	//Enable the 5DOF sliders
	myEnableDisableObject( NORMAL_MODE_ONE_SLIDER_OBJ, TRUE);
	myEnableDisableObject( NORMAL_MODE_TWO_SLIDER_OBJ, TRUE);
	myEnableDisableObject( NORMAL_MODE_THREE_SLIDER_OBJ, TRUE);
	myEnableDisableObject( NORMAL_MODE_FOUR_SLIDER_OBJ, TRUE);
	myEnableDisableObject( NORMAL_MODE_FIVE_SLIDER_OBJ, TRUE);

	myEnableDisableObject( BTN_PRECALCULATIONS_OBJ, FALSE);
	fl_set_button(BTN_PRECALCULATIONS_OBJ, 0);
	

	//If the Protein radio button is selected
	if ( fl_get_button(ELNEMO_PROT_BTN_OBJ) )
	{
		proteinNMInitializationAlreadyDone = TRUE;
	}
	else 	//If the Ligand radio button is selected
	if ( fl_get_button(ELNEMO_LIG_BTN_OBJ) )
	{
		ligandNMInitializationAlreadyDone = TRUE;
	}	
	else
		printf("Unknow buttons states !!! Please see file %s at line %d !", __FILE__, __LINE__);
}


static void CB_coldeg_planner_btn_obj(FL_OBJECT *object, long arg)
{
  if (fl_get_button(object)) {  
    bio_explore_coldeg();
    p3d_identify_farther_nodes();
  }
  else { 
    CB_stop_obj(NULL,0); 
    //CB_del_param_obj(ob,0); 
  }   
}

static void CB_goal_p3d_btn_obj(FL_OBJECT *object, long arg)
{
  if (fl_get_button(object)) {  
    if(bio_set_goal_jnt_coordinates())
      printf("DONE : goal jnt coordinates have been set\n");
    else
      printf("ERROR : goal jnt coordinates cannot be set\n");      
    fl_set_button(object,0);    
  }
}

static int QuestionUser( char *question, int defaultBtn )
{
	FL_FORM *tempForm;
	int reponse = 0;

	tempForm =fl_get_app_mainform();
   	fl_set_app_mainform(BIO_ENERGY_FORM);
	
	reponse = fl_show_question( question, defaultBtn);

   	fl_set_app_mainform(tempForm);
	return reponse;
}

static void AlertUser( char *title, char *text1, char *text2)
{
	FL_FORM *tempForm;

	tempForm =fl_get_app_mainform();
   	fl_set_app_mainform(BIO_ENERGY_FORM);
	
	fl_show_alert(title,text1,text2,1);

   	fl_set_app_mainform(tempForm);
}


// TO BE MOVED TO ANOTHER FILE !!!
/************************************************************************/
/* function computing the potential energy along a trajectory
   and saving values in a file */
static int bio_compute_potential_energy_along_path( p3d_rob *robotPt, p3d_traj *trjPt, FILE *fEpath)
{
  double u=0.0; 
  double du, umax=0.0; /* parameters along the local path */
  configPt q,qp;
  int njnt = robotPt->njoints;
  double *distances;
  int i, end_localpath = 0;
  pp3d_localpath localpathPt, last_localpathPt=NULL;
  double dep_dist = 0.0;
  p3d_vector3 pos, ref_pos, ref_pos0, pos_diff;
  int first_conf = 1;
  double want_dep_dist;
  double dmax = p3d_get_env_graphic_dmax();
  double potE;
  
  // NOTE : the step is defined by the slider in FORM BIO
  bio_col_get_step_deplacement(&want_dep_dist);

  localpathPt = trjPt->courbePt;
  distances = MY_ALLOC(double, njnt+1);
  qp = p3d_alloc_config(robotPt);


  while (localpathPt != NULL){
    umax = localpathPt->range_param;
    
    while (end_localpath < 2){
      /* position of the robot corresponding to parameter u */
      q = localpathPt->config_at_param(robotPt, localpathPt, u);
      if(u == 0.0) {
	p3d_set_robot_config(robotPt,q);
	p3d_update_this_robot_pos_without_cntrt(robotPt);
	// DEBUG
	//g3d_draw_allwin_active();
	if(first_conf) {
	  // initial conf
	  CalculateEnergy(robotPt,&potE);
	  dep_dist = 0.0;
	  // copy energy and displacement in file
	  fprintf(fEpath,"deplacement = %f , Epot = %f\n",dep_dist,potE);
	}
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
	p3d_vectCopy(pos,ref_pos0);
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
	// compute energy
	CalculateEnergy(robotPt,&potE);
	p3d_vectSub(pos,ref_pos0,pos_diff);
	dep_dist = p3d_vectNorm(pos_diff);
	// copy energy and displacement in file
	fprintf(fEpath,"deplacement = %f , Epot = %f\n",dep_dist,potE);
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
	end_localpath++;
      }
    }
    last_localpathPt = localpathPt;
    localpathPt = localpathPt->next_lp;
    end_localpath = 0;
    u = 0;
  }
  // last config
  q = last_localpathPt->config_at_param(robotPt, last_localpathPt, umax);
  p3d_set_and_update_this_robot_conf_multisol(robotPt,q,qp,P3D_HUGE, NULL);
  // compute energy
  CalculateEnergy(robotPt,&potE);
  p3d_vectSub(pos,ref_pos0,pos_diff);
  dep_dist = p3d_vectNorm(pos_diff);
  // copy energy and displacement in file
  fprintf(fEpath,"deplacement = %f , Epot = %f\n",dep_dist,potE);

  p3d_destroy_config(robotPt, q);
  p3d_destroy_config(robotPt, qp);
  MY_FREE(distances, double, njnt+1);
  return 1;
}
