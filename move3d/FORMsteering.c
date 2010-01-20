#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"

extern FL_OBJECT  *steering_obj;

FL_FORM *STEERING_FORM;
static FL_OBJECT  *GROUP;
static FL_OBJECT  *obj1;
static FL_OBJECT  *obj2;
FL_OBJECT  **BUTTON_TAB_OBJ;

static void CB_button_tab_obj(FL_OBJECT *ob, long arg)
{
  p3d_local_set_planner((p3d_localplanner_type) arg);
}  


void g3d_create_steering_form(void)
{int i;
 double s;

  s = P3D_NB_LOCAL_PLANNER*20.0 + 20;

  BUTTON_TAB_OBJ = (FL_OBJECT  **) malloc(P3D_NB_LOCAL_PLANNER*sizeof(FL_OBJECT  *));

  STEERING_FORM = fl_bgn_form(FL_UP_BOX,200.0,s+30);

  obj1 = fl_add_frame(FL_ENGRAVED_FRAME,10,15,180,s,""); 
  obj2 = fl_add_box(FL_FLAT_BOX,55,10,90,10,"Steering methods");

  GROUP = fl_bgn_group();

  for(i=0;i<P3D_NB_LOCAL_PLANNER;i++){
    BUTTON_TAB_OBJ[i] = fl_add_checkbutton(FL_RADIO_BUTTON,45,25+i*20.0,55,25,p3d_local_getname_planner((p3d_localplanner_type) i));
    fl_set_object_color(BUTTON_TAB_OBJ[i],FL_MCOL,FL_GREEN);
    fl_set_call_back(BUTTON_TAB_OBJ[i],CB_button_tab_obj,i);
  }

  fl_set_button(BUTTON_TAB_OBJ[p3d_local_get_planner()],1);

  //GROUP = 
  fl_end_group();

  fl_end_form();
}


/*****************************************************************/

/* fonctions de destruction des objets forms */
void g3d_delete_steering_form(void)
{int n,i;

 n = 4; 

  if(fl_get_button(steering_obj)){fl_hide_form(STEERING_FORM);}

  fl_free_object(obj1);
  fl_free_object(obj2);
  fl_free_object(GROUP);

  for(i=0;i<n;i++){
    fl_free_object(BUTTON_TAB_OBJ[i]);
  }
 
  fl_free_form(STEERING_FORM);

  free(BUTTON_TAB_OBJ);
}



/*************************************************************************/
