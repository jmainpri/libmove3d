#include "P3d-pkg.h"
#include "Bio-pkg.h"

#define MAX_JOINT_LINE_LENGTH 81
#define MAX_ATOM_INPUT_LENGTH 6
#define MAX_VVALUES_INPUT_LENGTH 8

//////// PRIVATE TYPES ////////////////////////////////////////////////////

typedef struct {
  int dihedralSerial[4];
  double vmin;
  double vmax;
} tmp_joint;

typedef struct {
  int nTmpJoints;
  tmp_joint** tmpJointList;
  psf_ligand* ligand;
  const char* file_to_save;
  const char* file_to_load;
} tmp_joint_list;

// PRIVATE FUNCTIONS ///////////////////////////////////////////////////////////

static void clear_joint_input_fields();
static void clear_tmp_joint_list(tmp_joint_list* joint_list);

static void activate_save_in_file();
static void deactivate_save_in_file();
static void activate_load_from_file();
static void deactivate_load_from_file();
static void deactivate_other_inputs_and_browser();
static void activate_other_inputs_and_browser();

static char* get_serial_txt(psf_atom* theAtom, int length);
static char* get_txt_atom(psf_atom* aPt, int length);

static int save_tmp_joint_from_browser(tmp_joint_list* joint_list);
static void load_tmp_joint_in_browser(tmp_joint_list* joint_list);

static int update_joints_in_ligand(tmp_joint_list* joint_list);
static int update_joints_in_all_ligands(tmp_joint_list** ligand_joints_list, int nb_ligand_in_list);
static void add_dihedral_atoms(FL_OBJECT *object, long user_data);

static void cb_select_atom_serial_0(FL_OBJECT *object, long user_data);
static void cb_select_atom_serial_1(FL_OBJECT *object, long user_data);
static void cb_select_atom_serial_2(FL_OBJECT *object, long user_data);
static void cb_select_atom_serial_3(FL_OBJECT *object, long user_data);

static void cb_toggle_save(FL_OBJECT *object, long user_data);
static void cb_toggle_load(FL_OBJECT *object, long user_data);

static void populate_joint_lig_browser(FL_OBJECT* browser, psf_atom** list, int nb);

static void cb_select_file_to_save(FL_OBJECT *object, long user_data);
static void cb_select_file_to_load(FL_OBJECT *object, long user_data);

static void cb_update_save_fileName(FL_OBJECT *object, long user_data);
static void cb_update_load_fileName(FL_OBJECT *object, long user_data);
static void cb_update_joint_lig_browser(FL_OBJECT *object, long user_data);


// GLOBAL VARIABLES ////////////////////////////////////////////////////////////

static JOINT_LIG_FORM* new_joint_lig_Form; 

static psf_atom** current_atoms_list = NULL;
static int natoms_in_list = 0;

static tmp_joint_list** current_joint_lists = NULL;
static int njoint_lists = 0;

static int current_choice = 0;


// USEFUL FUNCTIONS ////////////////////////////////////////////////////////////

static void clear_joint_input_fields() {

  int i;  

  for (i=0; i<4; i++) {
    fl_set_input(new_joint_lig_Form->INPUT_ATOM[i], "");
  }
  fl_set_input(new_joint_lig_Form->INPUT_VVALUES[0], "-180.0");
  fl_set_input(new_joint_lig_Form->INPUT_VVALUES[1], "180.0");
  
}

/**********************************************************************/

static void clear_tmp_joint_list(tmp_joint_list* joint_list) {

  int i;

  for (i=0; i<joint_list->nTmpJoints; i++)
    free(joint_list->tmpJointList[i]);
  free_pointer_list((void***)&(joint_list->tmpJointList), &(joint_list->nTmpJoints));
}

/**********************************************************************/

static void activate_save_in_file() {
    fl_activate_object(new_joint_lig_Form->INPUT_SAVE);
    fl_activate_object(new_joint_lig_Form->BROWSE_SAVE);
}

/**********************************************************************/

static void deactivate_save_in_file() {
    fl_set_button(new_joint_lig_Form->CHECK_SAVE,0);
    fl_set_input(new_joint_lig_Form->INPUT_SAVE, "");
    fl_deactivate_object(new_joint_lig_Form->INPUT_SAVE);
    fl_deactivate_object(new_joint_lig_Form->BROWSE_SAVE);
}

/**********************************************************************/

static void activate_load_from_file() {
    fl_activate_object(new_joint_lig_Form->INPUT_LOAD);
    fl_activate_object(new_joint_lig_Form->BROWSE_LOAD);
}

/**********************************************************************/

static void deactivate_load_from_file() {
    fl_set_button(new_joint_lig_Form->CHECK_LOAD,0);
    fl_set_input(new_joint_lig_Form->INPUT_LOAD, "");
    fl_deactivate_object(new_joint_lig_Form->INPUT_LOAD);
    fl_deactivate_object(new_joint_lig_Form->BROWSE_LOAD);
}

/**********************************************************************/

static void deactivate_other_inputs_and_browser() {

  int i;

  clear_joint_input_fields();
  fl_clear_browser(new_joint_lig_Form->BROWSER_DIHEDRE);
  clear_tmp_joint_list(current_joint_lists[current_choice-1]);
  fl_deactivate_object(new_joint_lig_Form->BROWSER_DIHEDRE);
  for (i=0; i<4; i++) {
    fl_deactivate_object(new_joint_lig_Form->ADD_ATOM[i]);
    fl_deactivate_object(new_joint_lig_Form->INPUT_ATOM[i]);
  }
  for (i=0; i<2; i++) {
    fl_deactivate_object(new_joint_lig_Form->INPUT_VVALUES[i]);
  }
  fl_deactivate_object(new_joint_lig_Form->ADD);
  fl_deactivate_object(new_joint_lig_Form->REMOVE);
}


/**********************************************************************/

static void activate_other_inputs_and_browser() {

  int i;

  fl_activate_object(new_joint_lig_Form->BROWSER_DIHEDRE);
  for (i=0; i<4; i++) {
    fl_activate_object(new_joint_lig_Form->ADD_ATOM[i]);
    fl_activate_object(new_joint_lig_Form->INPUT_ATOM[i]);
  }
  for (i=0; i<2; i++) {
    fl_activate_object(new_joint_lig_Form->INPUT_VVALUES[i]);
  }
  fl_activate_object(new_joint_lig_Form->ADD);
  fl_activate_object(new_joint_lig_Form->REMOVE);
}

/**********************************************************************/

static char* get_serial_txt(psf_atom* theAtom, int length) {

  char* txt = (char*)malloc(length*sizeof(char));
  snprintf(txt, length,"%d",theAtom->serial);

  return txt;
}

/**********************************************************************/

static char* get_txt_atom(psf_atom* aPt, int length) {
  
  char* txt = (char*)malloc(length*sizeof(char));
   
  snprintf(txt, length,"%d\t%s\t\t[%f %f% f]",
	   aPt->serial,
	   aPt->name,
	   aPt->pos[0],
	   aPt->pos[1],
	   aPt->pos[2]);
 
  return txt;
}

/**********************************************************************/

static int save_tmp_joint_from_browser(tmp_joint_list* joint_list) {
  
  int read_ok;
  int total_lines;
  int line_number;
  tmp_joint* read_joint;

  //printf("debut save\n");

  clear_tmp_joint_list(joint_list);

  total_lines = fl_get_browser_maxline(new_joint_lig_Form->BROWSER_DIHEDRE); 
  
  if (total_lines == 0) {
    return TRUE;
  }

  line_number = 1;

  do {
    
    read_ok = FALSE;

    read_joint = (tmp_joint*)malloc(sizeof(tmp_joint));

    if (sscanf(fl_get_browser_line(new_joint_lig_Form->BROWSER_DIHEDRE, line_number),
	       "%d %d %d %d %lf %lf",
	       &read_joint->dihedralSerial[0],
	       &read_joint->dihedralSerial[1],
	       &read_joint->dihedralSerial[2],
	       &read_joint->dihedralSerial[3],
	       &read_joint->vmin,
	       &read_joint->vmax) >=0) {
      read_ok = TRUE;
      insert_pointer_in_list((void*)read_joint,
			     (void***)&(joint_list->tmpJointList),
			     &(joint_list->nTmpJoints));
      line_number++;
    }
    else {
      free(read_joint);
    }

  } while (read_ok && line_number<=total_lines);
  
  if (!read_ok) {
    clear_tmp_joint_list(joint_list);
  }

  //printf("fin save\n");
  return read_ok;
}

/**********************************************************************/

static void load_tmp_joint_in_browser(tmp_joint_list* joint_list) {

  tmp_joint* read_joint;
  char txtline[MAX_JOINT_LINE_LENGTH];
  int lineIndex;

  //printf("Debut load\n");

  fl_clear_browser(new_joint_lig_Form->BROWSER_DIHEDRE);

  if (joint_list->file_to_load != NULL) {
    activate_load_from_file();
    fl_set_input(new_joint_lig_Form->INPUT_LOAD, joint_list->file_to_load);
    deactivate_other_inputs_and_browser();
  }
  else {

    deactivate_load_from_file();

    for (lineIndex=0; lineIndex<joint_list->nTmpJoints; lineIndex++) {
      read_joint = joint_list->tmpJointList[lineIndex];
      snprintf(txtline, MAX_JOINT_LINE_LENGTH-1, "%d\t%d\t%d\t%d\t\t%f\t%f",
	       read_joint->dihedralSerial[0],
	       read_joint->dihedralSerial[1],
	       read_joint->dihedralSerial[2],
	       read_joint->dihedralSerial[3],
	       read_joint->vmin,
	       read_joint->vmax);
      
      fl_add_browser_line(new_joint_lig_Form->BROWSER_DIHEDRE,txtline);
    }
  }
  
  if (joint_list->file_to_save != NULL) {
    activate_save_in_file();
    fl_set_input(new_joint_lig_Form->INPUT_SAVE, joint_list->file_to_save);
  }
  else {
    deactivate_save_in_file();
  }
    
}

// UPDATE FUNCTIONS ////////////////////////////////////////////////////////////

static int update_joints_in_ligand(tmp_joint_list* joint_list) {
  
  int no_error = FALSE;
  int jointIndex;
  int atomIndex;
  int atom_exist;
  psf_atom* theAtom = NULL;
  tmp_joint* current_tmp_joint;
  psf_joint* newJoint;

  if (joint_list->file_to_load != NULL) {
    // load dihedrals specification from .jnt file
    no_error = load_joint_desc_from_file(joint_list->file_to_load, joint_list->ligand);
  }

  else {

    // load dihedrals specification thanks to
    // the description given in the form
    
    jointIndex = 0;  
    
    if (joint_list->nTmpJoints == 0)
      return TRUE;
    
    do {
    
      atomIndex = 0;
      atom_exist = TRUE;
      current_tmp_joint = joint_list->tmpJointList[jointIndex];
      
      // create new joint
      newJoint = init_joint_struct();
      
      // define atoms in dihedral
      while (atom_exist && atomIndex<4) {
	atom_exist = psf_get_atom_in_list_by_serial(current_tmp_joint->dihedralSerial[atomIndex],
						&theAtom,
						joint_list->ligand->atomList,
						joint_list->ligand->natoms);
	if (atom_exist) {
	  newJoint->dihedralAtoms[atomIndex] = theAtom;
	  atomIndex++;
	}
      }
      
      if (atom_exist) {
	// define dihedral max and min angles
	newJoint->vmin = current_tmp_joint->vmin;
	newJoint->vmax = current_tmp_joint->vmax;
	// insert new joint in ligand
	insert_pointer_in_list((void*)newJoint,
			       (void***)&(joint_list->ligand->jointList),
			       &(joint_list->ligand->njoints));
	
	jointIndex++;
      }
      else {
	free(newJoint);
	free_pointer_list((void***)&(joint_list->ligand->jointList), &(joint_list->ligand->njoints));
      }
      
    } while (atom_exist && jointIndex<joint_list->nTmpJoints);
    
    no_error = atom_exist;

    if (no_error && (joint_list->file_to_save != NULL)) {
      // save dihedrals specification in .jnt file
      //printf("SAUVEGARDE\n");
      no_error = save_joint_desc_in_file(joint_list->file_to_save, joint_list->ligand);
    }

  }
  
  return no_error;
}

/**********************************************************************/

static int update_joints_in_all_ligands(tmp_joint_list** ligand_joints_list, int nb_ligand_in_list) {


  int jointListIndex;
  int updated;
  
  updated = FALSE;
  jointListIndex = 0;

  do {
    
    updated = update_joints_in_ligand(ligand_joints_list[jointListIndex]);
    if (updated)
      jointListIndex++;
    
  } while (jointListIndex<nb_ligand_in_list && updated);

  return (updated);
}

// CALLBACK FUNCTIONS //////////////////////////////////////////////////////////

static void remove_dihedral_atoms(FL_OBJECT *object, long user_data) {

  int line_Number;
  
  line_Number = fl_get_browser(new_joint_lig_Form->BROWSER_DIHEDRE);

  if (line_Number !=0 ) {
    fl_delete_browser_line(new_joint_lig_Form->BROWSER_DIHEDRE, line_Number);
    clear_joint_input_fields();
  }

}

/**********************************************************************/

static void add_dihedral_atoms(FL_OBJECT *object, long user_data) {
  
  char txtline[MAX_JOINT_LINE_LENGTH];
  char serials[4][MAX_ATOM_INPUT_LENGTH];
  char vvalues[2][MAX_VVALUES_INPUT_LENGTH];
  int int_value[2] = {-180.0, 180.0};

  int index = 0;
  int datas_OK = TRUE;

  for (index=0; index<4; index++)
    strncpy(serials[index],"",MAX_ATOM_INPUT_LENGTH-1);

  for (index=0; index<2; index++)
    strncpy(vvalues[index],"",MAX_ATOM_INPUT_LENGTH-1);

  index = 0;
  while (datas_OK && index<4) {
    snprintf(serials[index], MAX_ATOM_INPUT_LENGTH, "%s", fl_get_input(new_joint_lig_Form->INPUT_ATOM[index]));

    if (strncmp(serials[index],"", MAX_ATOM_INPUT_LENGTH-1) == 0) {
      datas_OK = FALSE;
    }
    else if (!is_a_defined_serial(atoi(serials[index]),
				  current_atoms_list,
				  natoms_in_list)) {
      datas_OK = FALSE;
      fl_show_alert("One or several serial(s) is(are) wrong !","","",1);
    }
    else {
      index++;
    }
  }

  index = 0;
  while (datas_OK && index<2) {
    snprintf(vvalues[index], MAX_VVALUES_INPUT_LENGTH, "%s", fl_get_input(new_joint_lig_Form->INPUT_VVALUES[index]));
    //printf("\"%s\"\n", vvalues[index]);
    if (strncmp(vvalues[index], "", MAX_VVALUES_INPUT_LENGTH-1) == 0) {
      datas_OK = FALSE;
    }
    else {
      int_value[index] = atoi(vvalues[index]);
      index++;
    }
  }

  if ((int_value[0]>int_value[1])
      ||(int_value[1]>180.0)
      ||(int_value[0]<-180.0)) {
    datas_OK = FALSE;
    fl_show_alert("Wrong values for Vmin and/or Vmax","","",1);
  }
  
  if (datas_OK) {
    snprintf(txtline, MAX_JOINT_LINE_LENGTH-1, "%s\t%s\t%s\t%s\t\t%s\t%s",
	     serials[0],
	     serials[1],
	     serials[2],
	     serials[3],
	     vvalues[0],
	     vvalues[1]);
  
    fl_add_browser_line(new_joint_lig_Form->BROWSER_DIHEDRE,txtline);
    clear_joint_input_fields();
  }
}

/**********************************************************************/

static void cb_select_atom_serial_0(FL_OBJECT *object, long user_data) {

  int line_Number;
  char* serial_txt;

  if (current_atoms_list != NULL) {
    line_Number = fl_get_browser(new_joint_lig_Form->BROWSER_ATOMS);
    if (line_Number!=0) {
      serial_txt = get_serial_txt(current_atoms_list[line_Number-1], MAX_ATOM_INPUT_LENGTH);
      fl_set_input(new_joint_lig_Form->INPUT_ATOM[0], serial_txt);
      //free(serial_txt);
    }
  }
}
/**********************************************************************/

static void cb_select_atom_serial_1(FL_OBJECT *object, long user_data) {

  int line_Number;
  char* serial_txt;
  
  if (current_atoms_list != NULL) {
    line_Number = fl_get_browser(new_joint_lig_Form->BROWSER_ATOMS);
    if (line_Number!=0) {
      serial_txt = get_serial_txt(current_atoms_list[line_Number-1], MAX_ATOM_INPUT_LENGTH);
      fl_set_input(new_joint_lig_Form->INPUT_ATOM[1], serial_txt);
      //free(serial_txt);
    }
  }
}

/**********************************************************************/

static void cb_select_atom_serial_2(FL_OBJECT *object, long user_data) {

  int line_Number;
  char* serial_txt;
  
  if (current_atoms_list != NULL) {
    line_Number = fl_get_browser(new_joint_lig_Form->BROWSER_ATOMS);
    if (line_Number!=0) {
      serial_txt = get_serial_txt(current_atoms_list[line_Number-1], MAX_ATOM_INPUT_LENGTH);
      fl_set_input(new_joint_lig_Form->INPUT_ATOM[2], serial_txt);
      //free(serial_txt);
    }
  }
}

/**********************************************************************/

static void cb_select_atom_serial_3(FL_OBJECT *object, long user_data) {

  int line_Number;
  char* serial_txt;
  
  if (current_atoms_list != NULL) {
    line_Number = fl_get_browser(new_joint_lig_Form->BROWSER_ATOMS);
    if (line_Number!=0) {
      serial_txt = get_serial_txt(current_atoms_list[line_Number-1], MAX_ATOM_INPUT_LENGTH);
      fl_set_input(new_joint_lig_Form->INPUT_ATOM[3], serial_txt);
      //free(serial_txt);
    }
  }
}

/**********************************************************************/

static void cb_toggle_save(FL_OBJECT *object, long user_data) {
  
  if (fl_get_button(new_joint_lig_Form->CHECK_SAVE)) {
    activate_save_in_file();
    deactivate_load_from_file();
  }
  else {
    deactivate_save_in_file();
  }
}

/**********************************************************************/

static void cb_toggle_load(FL_OBJECT *object, long user_data) {

  if (fl_get_button(new_joint_lig_Form->CHECK_LOAD)) {
    activate_load_from_file();
    deactivate_save_in_file();
    deactivate_other_inputs_and_browser();
  }
  else {
    deactivate_load_from_file();
    activate_other_inputs_and_browser();
  }
}

/**********************************************************************/

static void cb_select_file_to_save(FL_OBJECT *object, long user_data) {

  const char* fullName;

  fullName = fl_show_fselector("Save joint description","","*.jnt","");
  if (fullName != NULL) {
    fl_set_input(new_joint_lig_Form->INPUT_SAVE, fullName);
    fl_call_object_callback(new_joint_lig_Form->INPUT_SAVE);
  }
}

/**********************************************************************/

static void cb_select_file_to_load(FL_OBJECT *object, long user_data) {

  const char* fullName;

  fullName = fl_show_fselector("Load joint description","","*.jnt","");
  if (fullName != NULL) {
    fl_set_input(new_joint_lig_Form->INPUT_LOAD, fullName);
    fl_call_object_callback(new_joint_lig_Form->INPUT_LOAD);
    clear_joint_input_fields();
    clear_tmp_joint_list(current_joint_lists[current_choice-1]);
  }
}

/**********************************************************************/

static void cb_update_save_fileName(FL_OBJECT *object, long user_data) {
  current_joint_lists[current_choice-1]->file_to_save = fl_get_input(new_joint_lig_Form->INPUT_SAVE);
}

/**********************************************************************/

static void cb_update_load_fileName(FL_OBJECT *object, long user_data) {
  current_joint_lists[current_choice-1]->file_to_load = fl_get_input(new_joint_lig_Form->INPUT_LOAD);
}

/**********************************************************************/

static void cb_update_joint_lig_browser(FL_OBJECT *object, long user_data) {

  int choiceInd = fl_get_choice(new_joint_lig_Form->CHOICE);
  int saved = FALSE;

  if (choiceInd != 0) {
    saved = save_tmp_joint_from_browser(current_joint_lists[current_choice-1]);
    if (saved) {
      current_choice = choiceInd;
      populate_joint_lig_browser(new_joint_lig_Form->BROWSER_ATOMS,
				 current_joint_lists[current_choice-1]->ligand->atomList,
				 current_joint_lists[current_choice-1]->ligand->natoms);
      clear_joint_input_fields();
      load_tmp_joint_in_browser(current_joint_lists[current_choice-1]);
    }
    else {
      fl_show_alert("WARNING : error while trying to save current dihedral description","","",1);
    }
  }

}

// JOINT LIG FORM ////////////////////////////////////////////////////////////////////

static void populate_joint_lig_browser(FL_OBJECT* browser, psf_atom** list, int nb) {

  int line;
  
  current_atoms_list = list;
  natoms_in_list = nb;
  //printf("nb atomes = %d\n", nb);

  fl_clear_browser(browser);
  for (line=1; line<=natoms_in_list; line++) {
    fl_add_browser_line(new_joint_lig_Form->BROWSER_ATOMS,
			get_txt_atom(current_atoms_list[line-1], MAX_JOINT_LINE_LENGTH));
  }

}
/**********************************************************************/

void create_joint_lig_Form(psf_ligand** ligand_list, int nb_ligand) {

  int choiceInd;
  int i;

  // Alloc tmp joint lists
  njoint_lists = nb_ligand;
  current_joint_lists = (tmp_joint_list**)malloc(njoint_lists*sizeof(tmp_joint_list*)); 

  // Create form
  new_joint_lig_Form = (JOINT_LIG_FORM*)malloc(sizeof(JOINT_LIG_FORM));

  new_joint_lig_Form->CURRENT_FORM = fl_bgn_form(FL_UP_BOX,380.0,610.0); 

  new_joint_lig_Form->CHOICE = fl_add_choice(FL_DROPLIST_CHOICE, 10.0, 10.0, 350.0, 20.0, "");
  for (choiceInd=0; choiceInd<nb_ligand; choiceInd++) {
    fl_addto_choice(new_joint_lig_Form->CHOICE, ligand_list[choiceInd]->name);
    // Init associated tmp joint
    current_joint_lists[choiceInd] = (tmp_joint_list*)malloc(sizeof(tmp_joint_list));
    current_joint_lists[choiceInd]->nTmpJoints = 0;
    current_joint_lists[choiceInd]->tmpJointList = NULL;
    current_joint_lists[choiceInd]->ligand = ligand_list[choiceInd];
    current_joint_lists[choiceInd]->file_to_save = NULL;
    current_joint_lists[choiceInd]->file_to_load = NULL;
  }
  fl_set_object_callback(new_joint_lig_Form->CHOICE, cb_update_joint_lig_browser, 0);

  new_joint_lig_Form->FRAME = fl_add_box(FL_FRAME_BOX,10.0,50.0,360.0,435.0,"");

  new_joint_lig_Form->BROWSER_ATOMS = fl_add_browser(FL_HOLD_BROWSER,15.0,55.0,350.0,200.0,"");
  fl_set_browser_fontsize(new_joint_lig_Form->BROWSER_ATOMS, FL_NORMAL_SIZE);
  current_choice = 1;
  populate_joint_lig_browser(new_joint_lig_Form->BROWSER_ATOMS,
			     current_joint_lists[0]->ligand->atomList,
			     current_joint_lists[0]->ligand->natoms);
  
  new_joint_lig_Form->ADD_ATOM[0] = fl_add_button(FL_NORMAL_BUTTON,15.0,260.0,50.0,20.0,"@2>");
  fl_set_object_callback(new_joint_lig_Form->ADD_ATOM[0], cb_select_atom_serial_0, 0); 
  new_joint_lig_Form->ADD_ATOM[1] = fl_add_button(FL_NORMAL_BUTTON,75.0,260.0,50.0,20.0,"@2>");
  fl_set_object_callback(new_joint_lig_Form->ADD_ATOM[1], cb_select_atom_serial_1, 0); 
  new_joint_lig_Form->ADD_ATOM[2] = fl_add_button(FL_NORMAL_BUTTON,135.0,260.0,50.0,20.0,"@2>");
  fl_set_object_callback(new_joint_lig_Form->ADD_ATOM[2], cb_select_atom_serial_2, 0); 
  new_joint_lig_Form->ADD_ATOM[3] = fl_add_button(FL_NORMAL_BUTTON,195.0,260.0,50.0,20.0,"@2>");
  fl_set_object_callback(new_joint_lig_Form->ADD_ATOM[3], cb_select_atom_serial_3, 0); 
  
  fl_add_box(FL_FLAT_BOX,255.0,260.0,50.0,20.0,"- Vmin -");
  fl_add_box(FL_FLAT_BOX,315.0,260.0,50.0,20.0,"- Vmax -");

  new_joint_lig_Form->INPUT_ATOM[0]=fl_add_input(FL_INT_INPUT,15.0,280.0,50.0,20.0,"");
  new_joint_lig_Form->INPUT_ATOM[1]=fl_add_input(FL_INT_INPUT,75.0,280.0,50.0,20.0,"");
  new_joint_lig_Form->INPUT_ATOM[2]=fl_add_input(FL_INT_INPUT,135.0,280.0,50.0,20.0,""); 
  new_joint_lig_Form->INPUT_ATOM[3]=fl_add_input(FL_INT_INPUT,195.0,280.0,50.0,20.0,"");
  for (i=0; i<4; i++) {
    fl_set_input_maxchars(new_joint_lig_Form->INPUT_ATOM[i], MAX_ATOM_INPUT_LENGTH);
  }

  new_joint_lig_Form->INPUT_VVALUES[0]=fl_add_input(FL_FLOAT_INPUT,255.0,280.0,50.0,20.0,"");
  new_joint_lig_Form->INPUT_VVALUES[1]=fl_add_input(FL_FLOAT_INPUT,315.0,280.0,50.0,20.0,"");
  clear_joint_input_fields();
  for (i=0; i<2; i++) {
    fl_set_input_maxchars(new_joint_lig_Form->INPUT_VVALUES[i], MAX_VVALUES_INPUT_LENGTH);
  }

  new_joint_lig_Form->ADD = fl_add_button(FL_NORMAL_BUTTON,15.0,310.0,350.0,30.0,"ADD Dihedral atoms");
  fl_set_object_callback(new_joint_lig_Form->ADD, add_dihedral_atoms, 0);

  new_joint_lig_Form->REMOVE = fl_add_button(FL_NORMAL_BUTTON,15.0,450.0,350.0,30.0,"REMOVE Dihedral atoms");
  fl_set_object_callback(new_joint_lig_Form->REMOVE, remove_dihedral_atoms, 0);

  /*   fl_add_box(FL_FLAT_BOX,15.0,375.0,390.0,30.0,"Choose one or several residue(s)and\ */
  /*  change its(their) articulation properties by\n clicking on <Rigid/Mobile Backbone> or\ */
  /*  <Rigid/Mobile Side chain>"); */

  new_joint_lig_Form->BROWSER_DIHEDRE = fl_add_browser(FL_HOLD_BROWSER,15.0,345.0,350.0,100.0,"");
  fl_set_browser_fontsize(new_joint_lig_Form->BROWSER_DIHEDRE, FL_NORMAL_SIZE);

  new_joint_lig_Form->FRAME_SAVE_LOAD = fl_add_box(FL_FRAME_BOX,10.0,495.0,360.0,60.0,"");
  new_joint_lig_Form->CHECK_SAVE = fl_add_checkbutton(FL_PUSH_BUTTON, 15.0, 500.0, 60.0, 20.0, "save in file");
  fl_set_object_callback(new_joint_lig_Form->CHECK_SAVE, cb_toggle_save, 0);

  new_joint_lig_Form->CHECK_LOAD = fl_add_checkbutton(FL_PUSH_BUTTON, 15.0, 530.0, 60.0, 20.0, "load from file");
  fl_set_object_callback(new_joint_lig_Form->CHECK_LOAD, cb_toggle_load, 0);

  new_joint_lig_Form->INPUT_SAVE = fl_add_input(FL_NORMAL_INPUT,95.0,500.0,190.0,20.0,"");
  fl_set_input_return(new_joint_lig_Form->INPUT_SAVE,FL_RETURN_CHANGED);
  fl_set_object_callback(new_joint_lig_Form->INPUT_SAVE, cb_update_save_fileName, 0);
  fl_deactivate_object(new_joint_lig_Form->INPUT_SAVE);

  new_joint_lig_Form->INPUT_LOAD = fl_add_input(FL_NORMAL_INPUT,95.0,530.0,190.0,20.0,"");
  fl_set_input_return(new_joint_lig_Form->INPUT_LOAD,FL_RETURN_CHANGED);
  fl_set_object_callback(new_joint_lig_Form->INPUT_LOAD, cb_update_load_fileName, 0);
  fl_deactivate_object(new_joint_lig_Form->INPUT_LOAD);

  new_joint_lig_Form->BROWSE_SAVE = fl_add_button(FL_NORMAL_BUTTON,295.0,500.0,65.0,20.0,"Browse...");
  fl_set_object_callback(new_joint_lig_Form->BROWSE_SAVE, cb_select_file_to_save, 0);
  fl_deactivate_object(new_joint_lig_Form->BROWSE_SAVE);

  new_joint_lig_Form->BROWSE_LOAD = fl_add_button(FL_NORMAL_BUTTON,295.0,530.0,65.0,20.0,"Browse...");
  fl_set_object_callback(new_joint_lig_Form->BROWSE_LOAD, cb_select_file_to_load, 0);
  fl_deactivate_object(new_joint_lig_Form->BROWSE_LOAD);


  new_joint_lig_Form->VALIDATE = fl_add_button(FL_NORMAL_BUTTON,200.0,565.0,80.0,30.0,"OK");

  new_joint_lig_Form->CANCEL = fl_add_button(FL_NORMAL_BUTTON,290.0,565.0,80.0,30.0,"Annuler");

  fl_end_form();
}

/**********************************************************************/

int do_joint_lig_Form() {

  FL_OBJECT *clicked = NULL;
  int state = FALSE;
  int saved_without_error = TRUE;
  int loaded_without_error = TRUE;
  int joint_list_updated = FALSE;
  int i;

  fl_show_form(new_joint_lig_Form->CURRENT_FORM, FL_PLACE_FREE, FL_FULLBORDER,"Dihedral in Ligand");

  while ((clicked!=new_joint_lig_Form->VALIDATE) && (clicked!=new_joint_lig_Form->CANCEL))
    clicked = fl_do_forms();
  
  state = (clicked==new_joint_lig_Form->VALIDATE);
  fl_hide_form(new_joint_lig_Form->CURRENT_FORM);

  if (state == TRUE) {
    save_tmp_joint_from_browser(current_joint_lists[current_choice-1]);
    joint_list_updated = update_joints_in_all_ligands(current_joint_lists, njoint_lists);
  }

  // free the form and all objects in it
  fl_free_object(new_joint_lig_Form->BROWSER_ATOMS);  
  fl_free_object(new_joint_lig_Form->BROWSER_DIHEDRE);  
  fl_free_object(new_joint_lig_Form->FRAME);  
  for (i=0; i<4; i++)
    fl_free_object(new_joint_lig_Form->ADD_ATOM[i]);  
  for (i=0; i<4; i++)
    fl_free_object(new_joint_lig_Form->INPUT_ATOM[i]);  
  for (i=0; i<2; i++)
    fl_free_object(new_joint_lig_Form->INPUT_VVALUES[i]);  
  fl_free_object(new_joint_lig_Form->ADD);  
  fl_free_object(new_joint_lig_Form->REMOVE);  
  fl_free_object(new_joint_lig_Form->FRAME_SAVE_LOAD);
  fl_free_object(new_joint_lig_Form->CHECK_SAVE);
  fl_free_object(new_joint_lig_Form->CHECK_LOAD);
  fl_free_object(new_joint_lig_Form->INPUT_SAVE);
  fl_free_object(new_joint_lig_Form->INPUT_LOAD);
  fl_free_object(new_joint_lig_Form->BROWSE_SAVE);
  fl_free_object(new_joint_lig_Form->BROWSE_LOAD);
  fl_free_object(new_joint_lig_Form->VALIDATE);  
  fl_free_object(new_joint_lig_Form->CANCEL);  

  fl_free_form(new_joint_lig_Form->CURRENT_FORM);

  free(new_joint_lig_Form);
  
  for (i=0; i<njoint_lists; i++) {
    clear_tmp_joint_list(current_joint_lists[i]);
    free(current_joint_lists[i]);
  }
  free(current_joint_lists);
  njoint_lists = 0;
  
  return (state && joint_list_updated && saved_without_error && loaded_without_error);  
}
