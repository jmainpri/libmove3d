
#include "P3d-pkg.h"
#include "Bio-pkg.h"

// PRIVATE FUNCTIONS ///////////////////////////////////////////////////////////

static void activate_save_in_file();
static void deactivate_save_in_file();
static void activate_load_from_file();
static void deactivate_load_from_file();

static const char* art_Bkb_state(int art);
static const char* art_Sch_state(int art);
static char* get_txt_residue(AAA_residue_data *AAA_Residue);

static void cb_toggle_Bkb_Art(FL_OBJECT *object, long user_data);
static void cb_toggle_Sch_Art(FL_OBJECT *object, long user_data);
static void cb_toggle_All_Bkb_Art(FL_OBJECT *object, long user_data);
static void cb_toggle_All_Sch_Art(FL_OBJECT *object, long user_data);
static void cb_toggle_save(FL_OBJECT *object, long user_data);
static void cb_toggle_load(FL_OBJECT *object, long user_data);
static void cb_load_aaa_desc_from_file(FL_OBJECT *object, long user_data);

static void cb_select_file_to_save(FL_OBJECT *object, long user_data);
static void cb_select_file_to_load(FL_OBJECT *object, long user_data);

static void cb_update_AAA_Browser(FL_OBJECT *object, long user_data);
static void populate_AAA_Browser(FL_OBJECT* browser,AAA_residue_data_list* AAA_List);

// GLOBAL VARIABLES ////////////////////////////////////////////////////////////

static AAA_FORM *new_AAA_Form; 

static AAA_protein_data_list *current_AAA_protein_list = NULL;

static file_name_list* file_to_save_array = NULL;
static file_name_list* file_to_load_array = NULL;

static int current_choice = 0;

// USEFUL FUNCTIONS ////////////////////////////////////////////////////////////

static void activate_save_in_file() {
    fl_activate_object(new_AAA_Form->INPUT_SAVE);
    fl_activate_object(new_AAA_Form->BROWSE_SAVE);
}

/**********************************************************************/

static void deactivate_save_in_file() {
    fl_set_button(new_AAA_Form->CHECK_SAVE,0);
    fl_set_input(new_AAA_Form->INPUT_SAVE, "");
    fl_deactivate_object(new_AAA_Form->INPUT_SAVE);
    fl_deactivate_object(new_AAA_Form->BROWSE_SAVE);
}

/**********************************************************************/

static void activate_load_from_file() {
    fl_activate_object(new_AAA_Form->INPUT_LOAD);
    fl_activate_object(new_AAA_Form->BROWSE_LOAD);
}

/**********************************************************************/

static void deactivate_load_from_file() {
    fl_set_button(new_AAA_Form->CHECK_LOAD,0);
    fl_set_input(new_AAA_Form->INPUT_LOAD, "");
    fl_deactivate_object(new_AAA_Form->INPUT_LOAD);
    fl_deactivate_object(new_AAA_Form->BROWSE_LOAD);
}

/**********************************************************************/

static const char* art_Bkb_state(int art) {

  if (art == RIGID) return "Rigid bkb";
  else if (art == MOBILE) return "Mobile bkb";
  else return "";
}

/**********************************************************************/

static const char* art_Sch_state(int art) {

  if (art == RIGID) return "Rigid SCh";
  else if (art == MOBILE) return "Mobile SCh";
  else return "";
}

/**********************************************************************/

static char* get_txt_residue(AAA_residue_data *AAA_Residue) {
  
  char* txt = (char*)malloc(MAX_LINE_LENGTH*sizeof(char));
  const char* lineColor;
 
  if ((AAA_Residue->art_bkb == MOBILE) || (AAA_Residue->art_sch == MOBILE))
    lineColor = MOBILE_COLOR;
  else
    lineColor = RIGID_COLOR;
  
  snprintf(txt, MAX_LINE_LENGTH-1,"%s %s\t%d\t\t[%s]\t\t[%s]",
	   lineColor,
	   AAA_Residue->resName,
	   AAA_Residue->resSeq,
	   art_Bkb_state(AAA_Residue->art_bkb),
	   art_Sch_state(AAA_Residue->art_sch));
  
  return txt;
}


// CALLBACK FUNCTIONS //////////////////////////////////////////////////////////

static void cb_toggle_Bkb_Art(FL_OBJECT *object, long user_data) {
  
  int line_Number;
  
  if (current_AAA_protein_list->AAA_proteinList[current_choice-1] != NULL) {
    
    line_Number = fl_get_browser(new_AAA_Form->BROWSER);
    if (line_Number!=0) {
      aaa_toggle_Bkb_Art(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]);
      fl_freeze_form(new_AAA_Form->CURRENT_FORM);
      fl_replace_browser_line(new_AAA_Form->BROWSER, line_Number,
			      get_txt_residue(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]));
      fl_unfreeze_form(new_AAA_Form->CURRENT_FORM); 
    }
  }

}

/**********************************************************************/

static void cb_toggle_Sch_Art(FL_OBJECT *object, long user_data) {

  int line_Number;
  
  if (current_AAA_protein_list->AAA_proteinList[current_choice-1] != NULL) {
    line_Number = fl_get_browser(new_AAA_Form->BROWSER);
    if (line_Number!=0) {
      aaa_toggle_Sch_Art(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]);
      fl_freeze_form(new_AAA_Form->CURRENT_FORM);
      fl_replace_browser_line(new_AAA_Form->BROWSER, line_Number,
			      get_txt_residue(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]));
      fl_unfreeze_form(new_AAA_Form->CURRENT_FORM); 
    }
  }
}

/**********************************************************************/

static void cb_toggle_All_Bkb_Art(FL_OBJECT *object, long user_data) {
  
  int line_Number;
  int nb_res;

  if (current_AAA_protein_list->AAA_proteinList[current_choice-1] != NULL) {
    nb_res = current_AAA_protein_list->AAA_proteinList[current_choice-1]->nb_AAA_residues;
    fl_freeze_form(new_AAA_Form->CURRENT_FORM);
    for (line_Number=1; line_Number<=nb_res; line_Number++) {
      aaa_toggle_Bkb_Art(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]);
      fl_replace_browser_line(new_AAA_Form->BROWSER, line_Number,
			      get_txt_residue(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]));
    }
    fl_unfreeze_form(new_AAA_Form->CURRENT_FORM); 
  }

}

/**********************************************************************/

static void cb_toggle_All_Sch_Art(FL_OBJECT *object, long user_data) {

  int line_Number;
  int nb_res;
  
  if (current_AAA_protein_list->AAA_proteinList[current_choice-1] != NULL) {
    nb_res = current_AAA_protein_list->AAA_proteinList[current_choice-1]->nb_AAA_residues;
    fl_freeze_form(new_AAA_Form->CURRENT_FORM);
    for (line_Number=1; line_Number<=nb_res; line_Number++) {
      aaa_toggle_Sch_Art(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]);
      fl_replace_browser_line(new_AAA_Form->BROWSER, line_Number,
			      get_txt_residue(current_AAA_protein_list->AAA_proteinList[current_choice-1]->AAA_residueList[line_Number-1]));

    }  
    fl_unfreeze_form(new_AAA_Form->CURRENT_FORM); 
  }
}

/**********************************************************************/

static void cb_toggle_save(FL_OBJECT *object, long user_data) {
  
  if (fl_get_button(new_AAA_Form->CHECK_SAVE)) {
    activate_save_in_file();
    deactivate_load_from_file();
  }
  else {
    deactivate_save_in_file();
  }
}

/**********************************************************************/

static void cb_toggle_load(FL_OBJECT *object, long user_data) {

  if (fl_get_button(new_AAA_Form->CHECK_LOAD)) {
    activate_load_from_file();
    deactivate_save_in_file();
  }
  else {
    deactivate_load_from_file();
  }
}

/**********************************************************************/

static void cb_select_file_to_save(FL_OBJECT *object, long user_data) {

  const char* fullName;

  fullName = fl_show_fselector("Save aaa description","","*.aaa","");
  if (fullName != NULL) {
    fl_set_input(new_AAA_Form->INPUT_SAVE, fullName);
    fl_call_object_callback(new_AAA_Form->INPUT_SAVE);
  }
}

/**********************************************************************/

static void cb_select_file_to_load(FL_OBJECT *object, long user_data) {

  const char* fullName;

  fullName = fl_show_fselector("Load aaa description","","*.aaa","");
  if (fullName != NULL) {
    fl_set_input(new_AAA_Form->INPUT_LOAD, fullName);
    fl_call_object_callback(new_AAA_Form->INPUT_LOAD);
  }
}

/**********************************************************************/

static void cb_update_save_fileName(FL_OBJECT *object, long user_data) {
  file_to_save_array->name_list[current_choice-1] = fl_get_input(new_AAA_Form->INPUT_SAVE);
  printf("updated : %s\n",file_to_save_array->name_list[current_choice-1] );
}


/**********************************************************************/

static void cb_update_AAA_Browser(FL_OBJECT *object, long user_data) {

  int choiceInd = fl_get_choice(new_AAA_Form->CHOICE);
  if (choiceInd != 0) {
    current_choice = choiceInd;
    populate_AAA_Browser(new_AAA_Form->BROWSER, current_AAA_protein_list->AAA_proteinList[choiceInd-1]);
    if (file_to_save_array->name_list[choiceInd-1] != NULL) 
      fl_set_input(new_AAA_Form->INPUT_SAVE, file_to_save_array->name_list[choiceInd-1]);
    if (file_to_load_array->name_list[choiceInd-1] != NULL) 
      fl_set_input(new_AAA_Form->INPUT_LOAD, file_to_load_array->name_list[choiceInd-1]);
  }
}

/**********************************************************************/

static void cb_load_aaa_desc_from_file(FL_OBJECT *object, long user_data) {
  
  int read_OK = FALSE;
  AAA_residue_data_list AAA_desc;
  AAA_desc.nb_AAA_residues = 0;
  AAA_desc.AAA_residueList = NULL;

  file_to_load_array->name_list[current_choice-1] = fl_get_input(new_AAA_Form->INPUT_LOAD);
  read_OK = read_aaa_desc_from_file(file_to_load_array->name_list[current_choice-1], &AAA_desc);
  if (read_OK) {
    update_AAA_residue_data_list(&AAA_desc, current_AAA_protein_list->AAA_proteinList[current_choice-1]);
    populate_AAA_Browser(new_AAA_Form->BROWSER, current_AAA_protein_list->AAA_proteinList[current_choice-1]);
    deactivate_load_from_file();
  }
  else {
    fl_show_alert("Error : impossible to load aaa file","","",1); 
  }
}

// AAA FORM ////////////////////////////////////////////////////////////////////

static void populate_AAA_Browser(FL_OBJECT* browser,AAA_residue_data_list* AAA_List) {

  int line;
  
  fl_freeze_form(new_AAA_Form->CURRENT_FORM);
  fl_clear_browser(browser);
  for (line=1; line<=AAA_List->nb_AAA_residues; line++) {
    fl_add_browser_line(browser, get_txt_residue(AAA_List->AAA_residueList[line-1]));
  }
  fl_unfreeze_form(new_AAA_Form->CURRENT_FORM);

}

/**********************************************************************/

void create_AAA_Form(AAA_protein_data_list* AAA_protein_list) {

  int choiceInd;
  
  //  file_to_save_array = init_file_name_list(AAA_protein_list->nb_AAA_protein);
  file_to_save_array = init_file_name_list();
  file_to_save_array->name_list = (const char**)malloc(sizeof(const char*));
  //  file_to_load_array = init_file_name_list(AAA_protein_list->nb_AAA_protein);
  file_to_load_array = init_file_name_list();
  file_to_load_array->name_list = (const char**)malloc(sizeof(const char*));
  
  current_AAA_protein_list = copy_AAA_protein_data_list(AAA_protein_list);

  new_AAA_Form = (AAA_FORM*)malloc(sizeof(AAA_FORM));

  new_AAA_Form->CURRENT_FORM = fl_bgn_form(FL_UP_BOX,430.0,575.0); 

  new_AAA_Form->CHOICE = fl_add_choice(FL_DROPLIST_CHOICE, 10.0, 10.0, 400.0, 20.0, "");
  for (choiceInd=0; choiceInd<AAA_protein_list->nb_AAA_protein; choiceInd++) {
    fl_addto_choice(new_AAA_Form->CHOICE, AAA_protein_list->AAA_proteinList[choiceInd]->proteinName);
  }
  fl_set_object_callback(new_AAA_Form->CHOICE, cb_update_AAA_Browser, 0);

  new_AAA_Form->FRAME = fl_add_box(FL_FRAME_BOX,10.0,40.0,410.0,410.0,"");
  new_AAA_Form->BROWSER = fl_add_browser(FL_HOLD_BROWSER,15.0,55.0,400.0,300.0,"");
  fl_set_browser_fontsize(new_AAA_Form->BROWSER,FL_NORMAL_SIZE);
  current_choice = 1;
  populate_AAA_Browser(new_AAA_Form->BROWSER, AAA_protein_list->AAA_proteinList[0]);


  new_AAA_Form->TOGGLE_BKB = fl_add_button(FL_NORMAL_BUTTON,15.0,360.0,200.0,20.0,"Rigid / Mobile Backbone");
  fl_set_object_callback(new_AAA_Form->TOGGLE_BKB, cb_toggle_Bkb_Art, 0);

  new_AAA_Form->TOGGLE_ALL_BKB = fl_add_button(FL_NORMAL_BUTTON,15.0,380.0,200.0,20.0,"TOGLLE ALL (Backbone)");
  fl_set_object_callback(new_AAA_Form->TOGGLE_ALL_BKB, cb_toggle_All_Bkb_Art, 0);

  new_AAA_Form->TOGGLE_SCH = fl_add_button(FL_NORMAL_BUTTON,215.0,360.0,200.0,20.0,"Rigid / Mobile Side chain");
  fl_set_object_callback(new_AAA_Form->TOGGLE_SCH, cb_toggle_Sch_Art, 0);

  new_AAA_Form->TOGGLE_ALL_SCH = fl_add_button(FL_NORMAL_BUTTON,215.0,380.0,200.0,20.0,"TOGLLE ALL (Side chain)");
  fl_set_object_callback(new_AAA_Form->TOGGLE_ALL_SCH, cb_toggle_All_Sch_Art, 0);

  fl_add_box(FL_FLAT_BOX,15.0,415.0,390.0,30.0,"Choose one or several residue(s)and\
 change its(their) articulation properties by\n clicking on <Rigid/Mobile Backbone> or\
 <Rigid/Mobile Side chain>");

  new_AAA_Form->FRAME_SAVE_LOAD = fl_add_box(FL_FRAME_BOX,10.0,455.0,410.0,60.0,"");
  new_AAA_Form->CHECK_SAVE = fl_add_checkbutton(FL_PUSH_BUTTON, 15.0, 460.0, 60.0, 20.0, "save in file");
  fl_set_object_callback(new_AAA_Form->CHECK_SAVE, cb_toggle_save, 0);

  new_AAA_Form->CHECK_LOAD = fl_add_checkbutton(FL_PUSH_BUTTON, 15.0, 490.0, 60.0, 20.0, "load from file");
  fl_set_object_callback(new_AAA_Form->CHECK_LOAD, cb_toggle_load, 0);

  new_AAA_Form->INPUT_SAVE = fl_add_input(FL_NORMAL_INPUT,95.0,460.0,235.0,20.0,"");
  fl_set_input_return(new_AAA_Form->INPUT_SAVE,FL_RETURN_CHANGED);
  fl_set_object_callback(new_AAA_Form->INPUT_SAVE, cb_update_save_fileName, 0);
  fl_deactivate_object(new_AAA_Form->INPUT_SAVE);

  new_AAA_Form->INPUT_LOAD = fl_add_input(FL_NORMAL_INPUT,95.0,490.0,235.0,20.0,"");
  fl_set_input_return(new_AAA_Form->INPUT_LOAD,FL_RETURN_END);
  fl_set_object_callback(new_AAA_Form->INPUT_LOAD, cb_load_aaa_desc_from_file, 0);
  fl_deactivate_object(new_AAA_Form->INPUT_LOAD);

  new_AAA_Form->BROWSE_SAVE = fl_add_button(FL_NORMAL_BUTTON,345.0,460.0,65.0,20.0,"Browse...");
  fl_set_object_callback(new_AAA_Form->BROWSE_SAVE, cb_select_file_to_save, 0);
  fl_deactivate_object(new_AAA_Form->BROWSE_SAVE);

  new_AAA_Form->BROWSE_LOAD = fl_add_button(FL_NORMAL_BUTTON,345.0,490.0,65.0,20.0,"Browse...");
  fl_set_object_callback(new_AAA_Form->BROWSE_LOAD, cb_select_file_to_load, 0);
  fl_deactivate_object(new_AAA_Form->BROWSE_LOAD);

  new_AAA_Form->VALIDATE = fl_add_button(FL_NORMAL_BUTTON,250.0,525.0,80.0,30.0,"OK");
  //fl_set_object_callback(new_AAA_Form->VALIDATE, validate, 0);

  new_AAA_Form->CANCEL = fl_add_button(FL_NORMAL_BUTTON,340.0,525.0,80.0,30.0,"Annuler");
  //fl_set_object_callback(new_AAA_Form->CANCEL, cancel, 0);

  fl_end_form();
}

/**********************************************************************/

int do_AAA_Form(AAA_protein_data_list* AAA_protein_list) {

  FL_OBJECT *clicked = NULL;
  int state = FALSE;
  int protInd;
  int error = FALSE;

  fl_show_form(new_AAA_Form->CURRENT_FORM, FL_PLACE_FREE, FL_FULLBORDER,"AAA Parameters");

  while ((clicked!=new_AAA_Form->VALIDATE) && (clicked!=new_AAA_Form->CANCEL))
    clicked = fl_do_forms();
  
  state =  (clicked==new_AAA_Form->VALIDATE);
  fl_hide_form(new_AAA_Form->CURRENT_FORM);

  // free the form and all objects in it
  fl_free_object(new_AAA_Form->VALIDATE);  
  fl_free_object(new_AAA_Form->CANCEL);  
  fl_free_object(new_AAA_Form->CHOICE);  
  fl_free_object(new_AAA_Form->BROWSER);  
  fl_free_object(new_AAA_Form->FRAME);   
  fl_free_object(new_AAA_Form->TOGGLE_BKB);  
  fl_free_object(new_AAA_Form->TOGGLE_SCH);
  fl_free_object(new_AAA_Form->TOGGLE_ALL_BKB);  
  fl_free_object(new_AAA_Form->TOGGLE_ALL_SCH);
  fl_free_object(new_AAA_Form->FRAME_SAVE_LOAD);
  fl_free_object(new_AAA_Form->CHECK_SAVE);
  fl_free_object(new_AAA_Form->CHECK_LOAD);
  fl_free_object(new_AAA_Form->INPUT_SAVE);
  fl_free_object(new_AAA_Form->INPUT_LOAD);
  fl_free_object(new_AAA_Form->BROWSE_SAVE);
  fl_free_object(new_AAA_Form->BROWSE_LOAD);
  fl_free_form(new_AAA_Form->CURRENT_FORM);

  free(new_AAA_Form);

  // update the original AAA_List if OK and save aaa file if necessary
  if (state == TRUE) {
    update_AAA_protein_data_list(current_AAA_protein_list, AAA_protein_list);
  
    protInd = 0;
    while (protInd<file_to_save_array->nb_file_name && !error) {
      
      error = FALSE;
      if ((file_to_save_array->name_list[protInd] != NULL)
	  && (strcmp(file_to_save_array->name_list[protInd],"") != 0)) {
	error = !save_aaa_desc_in_file(file_to_save_array->name_list[protInd],
				       current_AAA_protein_list->AAA_proteinList[protInd]);
      }
      
      if (!error)
	protInd++;
      else {
	fl_show_alert("Error : impossible to save aaa file","","",1); 
      }
    }
  }

  //free file name lists
  free_file_name_list(file_to_save_array);
  file_to_save_array = NULL;
  free_file_name_list(file_to_load_array);
  file_to_load_array = NULL;

  // free current list
  free_AAA_protein_List(current_AAA_protein_list);
  current_AAA_protein_list = NULL;

  return state;
  
}
