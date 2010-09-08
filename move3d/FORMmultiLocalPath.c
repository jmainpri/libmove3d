
#ifdef MULTILOCALPATH
//File Created By Xavier BROQUERE
//On 29/10/2007
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

//FL_OBJECTS declaration
extern FL_FORM * MULTILOCALPATH_FORM;
extern FL_OBJECT * MULTILOCALPATH_OBJ;//The button creating this form. (in planner Form)
static FL_OBJECT *MULTILOCALPATH_LIST = NULL;
FL_OBJECT  *MGGRAPH_OBJ[MAX_MULTILOCALPATH_NB];
static FL_OBJECT * CANCEL_BUTTON = NULL;
static FL_OBJECT * DEACTIVATE_BUTTON = NULL;

//static functions declaration
//create
static void g3d_create_multiLocalPathList_obj(void);

   //callbacks
static int CB_multiLocalPathForm_OnClose(FL_FORM *form, void *arg);
static void CB_cancel_button(FL_OBJECT *ob, long arg);
static void CB_deactivate_button(FL_OBJECT *ob, long arg);
static void CB_multiLocalPathList_obj(FL_OBJECT *ob, long mgID);

  //delete


//initialisation
/****************************************************************************/
/** \brief Create the multilocalpath form.
 */
/****************************************************************************/
void g3d_create_multiLocalPath_form(void){
	g3d_create_form(&MULTILOCALPATH_FORM,400,200,FL_UP_BOX);

	g3d_create_multiLocalPathList_obj();
	g3d_create_button(&CANCEL_BUTTON,FL_NORMAL_BUTTON,50.0,30.0,"Cancel",(void**)&MULTILOCALPATH_FORM,1);
	g3d_create_button(&DEACTIVATE_BUTTON,FL_NORMAL_BUTTON,50.0,30.0,"Deactivate",(void**)&MULTILOCALPATH_FORM,1);
	fl_set_call_back(CANCEL_BUTTON,CB_cancel_button,0);
	fl_set_call_back(DEACTIVATE_BUTTON,CB_deactivate_button,0);
	fl_end_form();
	fl_set_form_icon(MULTILOCALPATH_FORM, GetApplicationIcon(), 0);
	fl_set_form_atclose(MULTILOCALPATH_FORM, CB_multiLocalPathForm_OnClose, 0);
}


/****************************************************************************/
/** \brief CallBack for the cancel button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_cancel_button(FL_OBJECT *ob, long arg){
	g3d_destroy_multiLocalPath_Form();
	
	fl_set_button(MULTILOCALPATH_OBJ,0);//release the button path_Deformation on planner FORM
}


/****************************************************************************/
/** \brief CallBack for the cancel button.
 \param *ob a pointer on the FL_OBJECT
 \param arg argument for the call back function (not used)
 */
/****************************************************************************/
static void CB_deactivate_button(FL_OBJECT *ob, long arg){

  int i=0;
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  p3d_multiLocalPath_disable_all_groupToPlan(r);
	
  for(i=0; i<r->mlp->nblpGp; i++) {
    
    fl_set_button(MGGRAPH_OBJ[i],0);
    
  }
  
}
/****************************************************************************/
/** \brief This function is called when the "multiLocalPath" window is closed from the X button
  If we do not use this call back, XForms tries to close the entire application
    and we do not want that. Instead we will just click on the Cancel button
 \param *form a pointer on the FL_FORM
 \param *arg argument for the call back function (not used)
 \return FL_IGNORE.
 */
/****************************************************************************/
static int CB_multiLocalPathForm_OnClose(FL_FORM *form, void *arg)
{
  //Call the fonction closing tthe form.
	g3d_destroy_multiLocalPath_Form();
	fl_set_button(MULTILOCALPATH_OBJ,0);//release the button multiLocalPath on planner FORM
  //If we return FL_OK, the application will continue to try to shut down itself
  //   if however we return FL_IGNORE, the application will not continue this event
	return FL_IGNORE;
}

//destruction
/****************************************************************************/
/** \brief method is used to destroy the cancel button.
 */
/****************************************************************************/
void g3d_destroy_multiLocalPath_Form(void){
	fl_hide_form(MULTILOCALPATH_FORM);
	//fl_free_form(MULTILOCALPATH_FORM);
}

static void g3d_create_multiLocalPathList_obj(void) {
	p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
	g3d_create_labelframe(&MULTILOCALPATH_LIST,FL_ENGRAVED_FRAME,-1,-1,"List of groups",(void**)&MULTILOCALPATH_FORM, 1);
  int i=0;
 	for(i=0; i<r->mlp->nblpGp; i++) {
		g3d_create_checkbutton(&MGGRAPH_OBJ[i],FL_PUSH_BUTTON,-1,-1,r->mlp->mlpJoints[i]->gpName,(void**)&MULTILOCALPATH_LIST, 0);
		fl_set_object_color(MGGRAPH_OBJ[i],FL_MCOL,FL_GREEN);
		if( p3d_multiLocalPath_get_value_groupToPlan( r, i) == 1) {
			fl_set_button(MGGRAPH_OBJ[i],1);
		} else {
			fl_set_button(MGGRAPH_OBJ[i],0);

		}
		fl_set_call_back(MGGRAPH_OBJ[i],CB_multiLocalPathList_obj, i);
	}

}

static void CB_multiLocalPathList_obj(FL_OBJECT *ob, long mgID) {
	int value;
	p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
	value = fl_get_button(ob);
	//printf("graph %d value %d\n",mgID,	value );
 	p3d_multiLocalPath_set_groupToPlan(r, mgID, value);
}

#endif


