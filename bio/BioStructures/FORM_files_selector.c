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
#include "Bio-pkg.h"
#include "Move3d-pkg.h"

// GLOBAL VARIABLES ////////////////////////////////////////////////////////////

static FILE_SELECTOR_FORM *new_file_selector_Form = NULL; 
static char file_directory[200] = "./";

// PRIVATE FUNCTIONS ///////////////////////////////////////////////////////////

static int is_already_in_list(const char* name);

static int match_extension (const char* fileName, const char* fileExtension);
static int is_P3D_file(const char* file);
static int is_PDB_file(const char* file);

static int update_file_list (file_name_list* file_list);

static void add_file_in_browser(FL_OBJECT *object, long user_data);
static void remove_file_from_browser(FL_OBJECT *object, long user_data);

static int p3d_file_selected = FALSE;

// USEFUL FUNCTIONS ////////////////////////////////////////////////////////////

static int is_already_in_list(const char* name) {
  
  int i=1;
  int found = FALSE;
  int total_lines = fl_get_browser_maxline(new_file_selector_Form->BROWSER);

  if (total_lines == 0)
    return FALSE;

  while (i<=total_lines && !found) {
    if (strcmp(name, fl_get_browser_line(new_file_selector_Form->BROWSER, i)) == 0) {
      found = TRUE;
    }
    else {
      i++;
    }
  }

  return found;
}

/***************************************************************/

static int match_extension (const char* fileName, const char* fileExtension) {

  int lengthFileName;
  int lengthExtension;
  int end = FALSE;
  int match = FALSE;

  lengthFileName = strlen(fileName);
  lengthExtension = strlen(fileExtension);

  if (lengthExtension < lengthFileName) {
    
    do {
      
      lengthFileName--;
      lengthExtension--;
      
      end = (fileName[lengthFileName]=='.');
      
      if (!end) {
	match = TRUE;
	if (fileName[lengthFileName]!=fileExtension[lengthExtension])
	  match = FALSE;
      }

    } while (lengthExtension>=0
	     && !end
	     && match);
           
  }
  
  return (match);
}

/***************************************************************/

static int is_P3D_file(const char* file) {
  return (match_extension(file, "p3d")
	  || match_extension(file, "P3D"));
}

/***************************************************************/

static int is_PDB_file(const char* file) {
  return (match_extension(file, "pdb")
	  || match_extension(file, "PDB"));
}

// UPDATE FUNCTIONS ////////////////////////////////////////////////////////////

static int update_file_list (file_name_list* file_list) {

  int total_lines;
  const char* file_name;
  int line_number;

  total_lines = fl_get_browser_maxline(new_file_selector_Form->BROWSER); 
  
  if (total_lines == 0) {
    return TRUE;
  }

  line_number = 1;

  do {
    
    file_name = fl_get_browser_line(new_file_selector_Form->BROWSER, line_number);
    if (file_name != NULL) {
      insert_pointer_in_list((void*)file_name,
			     (void***)&(file_list->name_list),
			     &(file_list->nb_file_name));
      line_number++;
    }

  } while ((file_name!=NULL) && line_number<=total_lines);
  
  return (file_name != NULL);
}

// CALLBACK FUNCTIONS //////////////////////////////////////////////////////////

static int onFormClose(FL_FORM *form, void *argument)
{	//From this method one can return FL_IGNORE 
	//  to prevent the window from closing
	//  or FL_OK to allow the window to close.
	
	//Instead of closing the window, we will simulate a click on 
	// the Cancel button
	fl_trigger_object(new_file_selector_Form->CANCEL);
	return(FL_IGNORE);
}
 
static void add_file_in_browser(FL_OBJECT *object, long user_data) {

  int answer = TRUE;

  const char* selected_file = NULL;


#ifdef ALIN
  selected_file = "/home/astefani/BioMove3D/demo/ProteinModels/xyl_min.p3d";
//  	proteinFile = "/home/astefani/BioMove3D/demo/ProteinModels/xyl_min.pdb";
#else
  selected_file = fl_show_fselector("Add pdb/p3d files",file_directory,"*.p??","");
#endif



  if (selected_file != NULL) {
    if (is_PDB_file(selected_file)) {
      if (is_already_in_list(selected_file)) {
	fl_show_alert("This file has already been chosen !","","",1);
      }
      else {
	fl_add_browser_line(new_file_selector_Form->BROWSER, selected_file);
	if (fl_get_browser_maxline(new_file_selector_Form->BROWSER)>1)
	  fl_activate_object(new_file_selector_Form->INPUT_NAME);
      }
    }
    else if (is_P3D_file(selected_file)) {
      if (fl_get_browser_maxline(new_file_selector_Form->BROWSER)>0) {
	answer = fl_show_question("WARNING : you can choose only one p3d file.\nThe other selected files are about to be removed.\n Do you really want to continue ?",0);
      }
      if (answer) {
	p3d_file_selected = TRUE;
	fl_clear_browser(new_file_selector_Form->BROWSER);
	fl_add_browser_line(new_file_selector_Form->BROWSER, selected_file);
	fl_deactivate_object(new_file_selector_Form->ADD);
	fl_deactivate_object(new_file_selector_Form->INPUT_NAME);
      }
    }
    else {
      fl_show_alert("The selected file is not a PDB or a P3D one !","","",1);
    }
  }
}

/***************************************************************/

static void remove_file_from_browser(FL_OBJECT *object, long user_data) {

  int selected_line =  fl_get_browser(new_file_selector_Form->BROWSER);
  if (selected_line != 0) {
    fl_delete_browser_line(new_file_selector_Form->BROWSER, selected_line);
    if (p3d_file_selected) {
      fl_activate_object(new_file_selector_Form->ADD);
      p3d_file_selected = FALSE;
    }
    if (fl_get_browser_maxline(new_file_selector_Form->BROWSER)<2)
      fl_deactivate_object(new_file_selector_Form->INPUT_NAME);
  }
}

// FILE SELECTOR FORM //////////////////////////////////////////////////////////

void create_file_selector_Form() {

  new_file_selector_Form = (FILE_SELECTOR_FORM*)malloc(sizeof(FILE_SELECTOR_FORM));

  new_file_selector_Form->CURRENT_FORM = fl_bgn_form(FL_UP_BOX,400.0,300.0); 

  new_file_selector_Form->FRAME = fl_add_box(FL_FRAME_BOX,10.0,10.0,380.0,240.0,"");

  new_file_selector_Form->INPUT_NAME = fl_add_input(FL_NORMAL_INPUT,110.0,15.0,275.0,20.0,"Environment name:");
  //fl_set_input(new_file_selector_Form->INPUT_NAME,"");
  fl_deactivate_object(new_file_selector_Form->INPUT_NAME);

  new_file_selector_Form->BROWSER = fl_add_browser(FL_HOLD_BROWSER,15.0,40.0,370.0,165.0,"");
  fl_set_browser_fontsize(new_file_selector_Form->BROWSER,FL_NORMAL_SIZE);

  new_file_selector_Form->ADD = fl_add_button(FL_NORMAL_BUTTON,20.0,215.0,170.0,30.0,"Add file ...");
  fl_set_object_callback(new_file_selector_Form->ADD,add_file_in_browser,0);
  
  new_file_selector_Form->REMOVE = fl_add_button(FL_NORMAL_BUTTON,210.0,215,170.0,30.0,"Remove selected file");
  fl_set_object_callback(new_file_selector_Form->REMOVE,remove_file_from_browser,0);
  
  new_file_selector_Form->VALIDATE = fl_add_button(FL_NORMAL_BUTTON,220.0,260.0,80.0,30.0,"OK");
  
  new_file_selector_Form->CANCEL = fl_add_button(FL_NORMAL_BUTTON,310.0,260.0,80.0,30.0,"Cancel");

  fl_set_form_atclose(new_file_selector_Form->CURRENT_FORM, onFormClose, 0);

  fl_end_form();
}

/**********************************************************************/

int do_file_selector_Form(const char* default_directory, file_name_list* file_list, char* name, int nameLength, int* p3d_file_source) {

  FL_OBJECT *clicked = NULL;
  int state = FALSE;
  int updated = FALSE;
//  int name_has_to_be_specified = FALSE;

  *p3d_file_source = FALSE;

   fl_set_form_icon(new_file_selector_Form->CURRENT_FORM, GetApplicationIcon( ), 0);

  snprintf(file_directory, 199, "%s", default_directory);
  fl_show_form(new_file_selector_Form->CURRENT_FORM, FL_PLACE_CENTERFREE, FL_FULLBORDER,"Source file(s)");

  while ((clicked!=new_file_selector_Form->VALIDATE)
	 && (clicked!=new_file_selector_Form->CANCEL)) {
    clicked = fl_do_forms();
    if (fl_get_browser_maxline(new_file_selector_Form->BROWSER)>1
	&& (strcmp(fl_get_input(new_file_selector_Form->INPUT_NAME),"")==0)) {
      clicked = NULL;
      fl_show_alert("You have to specify an environment name because\nmore than one molecule are about to be loaded.","","",1);
    }
  }

  state =  (clicked==new_file_selector_Form->VALIDATE);
  fl_hide_form(new_file_selector_Form->CURRENT_FORM);

  if (state) {
    *p3d_file_source = p3d_file_selected;
    updated = update_file_list(file_list);
    strncpy(name,fl_get_input(new_file_selector_Form->INPUT_NAME), nameLength);

    if (!updated) {
      fl_show_alert("A problem occured while trying to get file source names","","",1);
    }
  }

  // free the form and all objects in it
  fl_free_object(new_file_selector_Form->VALIDATE);  
  fl_free_object(new_file_selector_Form->CANCEL);  
  fl_free_object(new_file_selector_Form->ADD);  
  fl_free_object(new_file_selector_Form->REMOVE);  
  fl_free_object(new_file_selector_Form->BROWSER);  
  fl_free_object(new_file_selector_Form->INPUT_NAME);   
  fl_free_object(new_file_selector_Form->FRAME);   
  fl_free_form(new_file_selector_Form->CURRENT_FORM);

  free(new_file_selector_Form);

  return (state && updated);
  
}
