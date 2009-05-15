#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

#define NBUTTON_PER_LINE 3

MENU_FILTER *FILTER_FORM;

extern MENU_ROBOT *ROBOTS_FORM;

static void g3d_create_filter_bounds_obj(void);
static void g3d_create_filter_cancel_obj(void);
static void g3d_create_filter_ok_obj(void);

void g3d_create_filterbox_form(int ir)
{
  int n;
  double s;

  p3d_sel_desc_num(P3D_ROBOT,ir);

  n = calc_real_dof();
  s = (n/NBUTTON_PER_LINE+1)*20.0+50.0;

  FILTER_FORM[ir].ROBOT_FORM = fl_bgn_form(FL_UP_BOX,80*NBUTTON_PER_LINE+20,s);
  /* creation of all buttons: */
  g3d_create_filter_bounds_obj();
  g3d_create_filter_cancel_obj();
  g3d_create_filter_ok_obj();
  fl_end_form();
}

/*******************************************************/
static FL_FORM *JNT_INPUT_FORM;
static FL_OBJECT      *INPUT_MIN_OBJ;
static FL_OBJECT      *INPUT_MAX_OBJ;

static double new_min[MAX_DDLS]; /* angles are expressed in degree */
static double new_max[MAX_DDLS]; /* angles are expressed in degree */
static double old_min[MAX_DDLS]; /* angles are expressed in degree */
static double old_max[MAX_DDLS]; /* angles are expressed in degree */

void FORMfilter_init_temp_jntdata(void)
{
  int i,j,k,njnt;
  double min, max;
  p3d_jnt * jntPt;
  p3d_rob * robotPt;

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  njnt = p3d_get_robot_njnt();
  for(i=0;i<=njnt;i++) {
    jntPt =  robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof+j;
      p3d_jnt_get_dof_bounds_deg(jntPt, j, &min, &max);
      new_min[k] = old_min[k] = min;
      new_max[k] = old_max[k] = max;
    }
  }
}

static void CB_filter_bounds_cancel(FL_OBJECT *ob, long arg)
{
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  int jntnr = (int)arg;

  fl_set_button(FILTER_FORM[ir].BOUNDS_OBJ[jntnr],0);
  fl_hide_form(JNT_INPUT_FORM);
  fl_free_form(JNT_INPUT_FORM);
}

FL_OBJECT *confirm_min;
FL_OBJECT *confirm_max;

static void CB_filter_min_confirm(FL_OBJECT *ob, long arg)
{
  int jntnr = (int)arg;
  
  new_min[jntnr]=atof(fl_get_input(INPUT_MIN_OBJ));
  fl_set_button(confirm_min,0);
  printf("new min = %f, new max = %f\n",new_min[jntnr],new_max[jntnr]);
}

static void CB_filter_max_confirm(FL_OBJECT *ob, long arg)
{
  int jntnr = (int)arg;

  new_max[jntnr]=atof(fl_get_input(INPUT_MAX_OBJ)); 
  fl_set_button(confirm_max,0);
  printf("new min = %f, new max = %f\n",new_min[jntnr],new_max[jntnr]);
}

static void CB_filter_bounds_obj(FL_OBJECT *ob, long arg)
{
  int i_dof, jntnr = (int)arg;
  FL_OBJECT *cancel;
  char      str[20];
  p3d_jnt * jntPt;
  p3d_rob * robotPt;

  JNT_INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,170.0,130.0);

  INPUT_MIN_OBJ  = fl_add_box(FL_UP_BOX,10.0,10.0,90.0,30.0,"");
  INPUT_MIN_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,90.0,30.0,
                                "Min.");
  fl_set_input_color(INPUT_MIN_OBJ, 0, 0); 

  INPUT_MAX_OBJ  = fl_add_box(FL_UP_BOX,10.0,50.0,90.0,30.0,"");
  INPUT_MAX_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,50.0,90.0,30.0,
                                "Max.");
  fl_set_input_color(INPUT_MAX_OBJ, 0, 0); 

  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  jntPt = p3d_robot_dof_to_jnt(robotPt, jntnr, &i_dof);
  sprintf(str,"Joint (%i) %s done", jntPt->num, 
	  p3d_jnt_get_dof_name(jntPt, i_dof));

  cancel = fl_add_button(FL_PUSH_BUTTON, 10,90,80,30,str);
  fl_set_object_callback(cancel,CB_filter_bounds_cancel,jntnr);
  confirm_min = fl_add_button(FL_PUSH_BUTTON, 130,10,30,30,"OK");
  fl_set_object_callback(confirm_min,CB_filter_min_confirm,jntnr);
  confirm_max = fl_add_button(FL_PUSH_BUTTON, 130,50,30,30,"OK");
  fl_set_object_callback(confirm_max,CB_filter_max_confirm,jntnr);
  fl_end_form();

  fl_show_form(JNT_INPUT_FORM,FL_PLACE_SIZE,TRUE,""); 
  /* printf("jntnr should be: %i\n",jntnr); */

}

static void g3d_create_filter_bounds_obj(void)
{
  char      str[30];
  int       njnt,i,j,k,m,ir,ord;
  p3d_rob *robotPt;
  p3d_jnt *jntPt;

  njnt = p3d_get_robot_njnt();
  ir = p3d_get_desc_curnum(P3D_ROBOT); 
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  ord = 0; m = 0;
  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      if(p3d_jnt_get_dof_is_user(jntPt, j)) {
	strcpy(str, p3d_jnt_get_dof_name(jntPt, j));
	FILTER_FORM[ir].BOUNDS_OBJ[k] =
	  fl_add_button(FL_PUSH_BUTTON,10.0+ord*80.,
			10.+20.0*((j+jntPt->index_user_dof)/NBUTTON_PER_LINE),
			80.0,20.0,str); 
	ord=(ord+1)%NBUTTON_PER_LINE;
	fl_set_call_back(FILTER_FORM[ir].BOUNDS_OBJ[k],
			 CB_filter_bounds_obj,k);
      } else
	{ FILTER_FORM[ir].BOUNDS_OBJ[k]=NULL; }
    }
  }
}

/*******************************************************/

void  CB_display_filter_cancel_obj(FL_OBJECT *ob, long arg)
{ int ir = p3d_get_desc_curnum(P3D_ROBOT);

 fl_hide_form(FILTER_FORM[ir].ROBOT_FORM);
 fl_set_button(ROBOTS_FORM[ir].ADAPT_FILTERBOX_OBJ,0);
}

static void g3d_create_filter_cancel_obj(void)
{
  void  CB_display_filter_cancel_obj(FL_OBJECT *ob, long arg);
  int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
  
  n = calc_real_dof();
  
  FILTER_FORM[ir].CANCEL_OBJ = 
    fl_add_button(FL_PUSH_BUTTON,10.0,5.+20.0*(n/NBUTTON_PER_LINE+1), 
		  80*(NBUTTON_PER_LINE/2.0), 30.0,"Cancel");
  fl_set_call_back(FILTER_FORM[ir].CANCEL_OBJ,CB_display_filter_cancel_obj,0);
}

/*******************************************************/

static FL_FORM       *INPUT_FORM;

double old_x1,old_x2,old_y1,old_y2,old_z1,old_z2;

void CB_filter_confirm(FL_OBJECT *ob, long arg)
{
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  int i,njnt,j,k;
  double     f_min,f_max; /* angles expressed in degree */
  p3d_rob *robotPt;
  p3d_jnt *jntPt;
  configPt robot_pos_deg;
  
  /* confirmed => re-initialize collision detector */
  p3d_col_stop();              
  p3d_col_start_current();     
  
  /* this is current robot */
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  njnt = p3d_get_robot_njnt();
  
  p3d_set_robot_in_joint_limits(robotPt, robotPt->ROBOT_POS);
  p3d_set_robot_in_joint_limits(robotPt, robotPt->ROBOT_GOTO);
  /* convert the position of the current robot in degree */
  robot_pos_deg = p3d_copy_config_rad_to_deg(robotPt, robotPt->ROBOT_POS);

  for(i=0; i<=njnt; i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof + j;
      p3d_jnt_get_dof_bounds_deg(jntPt, j, &f_min, &f_max);
      if (ROBOTS_FORM[ir].POSITION_OBJ[k] != NULL) {
	fl_set_slider_bounds(ROBOTS_FORM[ir].POSITION_OBJ[k], 
			     f_min, f_max);
	fl_set_slider_value(ROBOTS_FORM[ir].POSITION_OBJ[k],
			    robot_pos_deg[k]);

      }
    }
  }
  p3d_destroy_config(robotPt, robot_pos_deg);

  fl_hide_form(INPUT_FORM);
  fl_free_form(INPUT_FORM);
  fl_hide_form(FILTER_FORM[ir].ROBOT_FORM);
  fl_set_button(ROBOTS_FORM[ir].ADAPT_FILTERBOX_OBJ,0);
  g3d_draw_allwin_active();
}

void CB_filter_cancel(FL_OBJECT *ob, long arg)
{
  int i,j,n,k;
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  pp3d_rob robotPt;
  p3d_jnt * jntPt;

  /* assign old limits to the joints */
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  n = p3d_get_robot_njnt();
  for(i=0;i<=n;i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof+j;
      p3d_jnt_set_dof_bounds_deg(jntPt, j, old_min[k], old_max[k]);
    }
  }
  p3d_update_robot_pos();

  /* put old filterbox back */
  p3d_set_filterbox(old_x1,old_x2,old_y1,old_y2,old_z1,old_z2);
  g3d_draw_allwin_active();

  fl_hide_form(INPUT_FORM);
  fl_free_form(INPUT_FORM);
  fl_set_button(FILTER_FORM[ir].OK_OBJ,0);
}

void CB_display_filter_ok_obj(FL_OBJECT *ob, long arg)
{ 
  int i,n,j,k;
  int ir = p3d_get_desc_curnum(P3D_ROBOT);
  FL_OBJECT *cancel;
  FL_OBJECT *confirm;
  pp3d_rob robotPt;
  p3d_jnt * jntPt;

  /* assign new limits to the joints */
  robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  n = p3d_get_robot_njnt();

  for(i=0;i<=n;i++) {
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      k = jntPt->index_dof+j;
      p3d_jnt_set_dof_bounds_deg(jntPt, j, old_min[k], old_max[k]);
    }
  }
  p3d_update_robot_pos();

  /* compute a new filterbox */
  p3d_get_filter_box(&old_x1,&old_x2,&old_y1,&old_y2,&old_z1,&old_z2);
  p3d_filter_cleanup();
  p3d_filter_init_robot_box();
  /* draw the new filterbox */
  fl_set_button(ROBOTS_FORM[ir].DISPLAY_FILTERBOX_OBJ,1);
  p3d_filter_set_draw_robot_box(TRUE);
  g3d_draw_allwin_active();
  /* ask for confirmation */
  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,100.0,40.0);
  cancel = fl_add_button(FL_PUSH_BUTTON, 10,10,40,20,"Cancel");
  fl_set_object_callback(cancel,CB_filter_cancel,0);
  confirm = fl_add_button(FL_PUSH_BUTTON, 50,10,40,20,"Apply");
  fl_set_object_callback(confirm,CB_filter_confirm,0);
  fl_end_form();
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
  fl_set_button(confirm,0);
  fl_set_button(cancel,0);
}

static void g3d_create_filter_ok_obj(void)
{void  CB_display_filter_ok_obj(FL_OBJECT *ob, long arg);
 int n,ir = p3d_get_desc_curnum(P3D_ROBOT); 
 
 n = calc_real_dof();

 FILTER_FORM[ir].OK_OBJ = fl_add_button(FL_PUSH_BUTTON,
					10.0+80*(NBUTTON_PER_LINE/2.0),
					5.+20.0*(n/NBUTTON_PER_LINE+1),
					80*(NBUTTON_PER_LINE/2.0),30.0,
					"OK");
 fl_set_call_back(FILTER_FORM[ir].OK_OBJ,CB_display_filter_ok_obj,0);


}

/*******************************************************/
void g3d_delete_filter_bounds(int ir)
{ int n,i;
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  n = robotPt->nb_dof;
  for(i=0;i<n;i++)
    {
      if(FILTER_FORM[ir].BOUNDS_OBJ[i])
	fl_free_object(FILTER_FORM[ir].BOUNDS_OBJ[i]);
    }
}

void g3d_delete_filter_cancel(int ir)
{
  fl_free_object(FILTER_FORM[ir].CANCEL_OBJ);
}

void g3d_delete_filter_ok(int ir)
{ 
  fl_free_object(FILTER_FORM[ir].OK_OBJ);
}


void g3d_delete_filter_form(int ir)
{int numr = p3d_get_desc_curnum(P3D_ROBOT);

  if((fl_get_button(ROBOTS_FORM[ir].ADAPT_FILTERBOX_OBJ)) && (numr == ir)){fl_hide_form(FILTER_FORM[ir].ROBOT_FORM);}

  p3d_sel_desc_num(P3D_ROBOT,ir);

  g3d_delete_filter_bounds(ir);
  g3d_delete_filter_cancel(ir);
  g3d_delete_filter_ok(ir);

  fl_free_form(FILTER_FORM[ir].ROBOT_FORM);

  p3d_sel_desc_num(P3D_ROBOT,numr);
}
