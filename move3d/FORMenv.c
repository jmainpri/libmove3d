#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

extern FL_OBJECT  *envparams_obj;	// KINEO-DEV :doit �tre d�clar� dans un .h !!

FL_FORM					*ENV_FORM = NULL;
FL_FORM					*KCD_FORM = NULL;
FL_FORM					*KCD_FORM_2 = NULL;
FL_OBJECT				*TOL_OBJ;
FL_OBJECT				*VOL_OBJ;
FL_OBJECT				*KCD_OBJ;
FL_OBJECT       *KCD_PRINT_COL;
FL_OBJECT		                *KCD_OBJ_2;
FL_OBJECT                               *DMAX_SLIDER_OBJ; // modif Pepijn
FL_OBJECT                               *REL_ERROR_SLIDER_OBJ;
FL_OBJECT                               *MICROCOLLISION_OBJ; //Modif Pepijn/Etienne
FL_OBJECT				*KCD_ALL_OBB_OBJ;
FL_OBJECT				*KCD_ROB_OBB_OBJ;
FL_OBJECT				*KCD_AABB_TREE_OBJ;
FL_OBJECT				*KCD_ALL_AABB_OBJ;
/*modif pepijn aug 2001*/
FL_OBJECT			        *KCD_ROB_ALL_OBJ;
FL_OBJECT			        *KCD_ROB_ROB_OBJ;
FL_OBJECT			        *KCD_ROB_AUTO_OBJ;
FL_OBJECT			        *KCD_ROB_ENV_OBJ;
FL_OBJECT			        *KCD_EVERYTHING_OBJ; 
FL_OBJECT			        *KCD_NO_REPORT;
FL_OBJECT			        *KCD_JUST_BOOL;
FL_OBJECT			        *KCD_DIST_EST;
FL_OBJECT			        *KCD_DIST_EXACT;
FL_OBJECT			        *KCD_SHOW_INFO_OBJ; 
FL_OBJECT			        *KCD_SHOW_INFO_2_OBJ; 

static FL_OBJECT  *GROUP2; 
static FL_OBJECT  *GROUP3; 
static FL_OBJECT      *INPUT_OBJ;
static FL_OBJECT      *PARAM_GROUP;
static FL_FORM       *INPUT_FORM;



static int KCD_CHOICE_IS_ACTIVE = FALSE;

int g3d_get_KCD_CHOICE_IS_ACTIVE()
{
  return(KCD_CHOICE_IS_ACTIVE);
}



/* actions manipulating the collision checkers */
static void CB_activate_vcollide(FL_OBJECT *ob, long arg);
static void CB_activate_vcollide_sel(FL_OBJECT *ob, long arg);
/* D�but modification Fabien */
static void CB_col_traj_method(FL_OBJECT *ob, long arg);
static void CB_BB_computation_method(FL_OBJECT *ob, long arg);
static void CB_BB_selection_method(FL_OBJECT *ob, long arg);
/* Fin modification Fabien */

/* actions for choice by user of seed for random seletion */
static void CB_act_get_seed(FL_OBJECT *ob, long arg);
static void CB_repete_input_obj(FL_OBJECT *ob, long arg);
static void CB_seed_input_obj(FL_OBJECT *ob, long arg);
/* actions for choice by user of number of calls for static collision */
static void CB_act_get_repete(FL_OBJECT *ob, long arg);
static void CB_act_get_call(FL_OBJECT *ob, long arg);
static void CB_call_input_obj(FL_OBJECT *ob, long arg);
/* actions for timing of static collision tests */
static void CB_act_test_coll(FL_OBJECT *ob, long arg);
/* actions for timing of semi-dynamic collision test */
static void CB_act_test_path(FL_OBJECT *ob, long arg);
/* actions for timing of semi-dynamic collision test for user-defined
   initial and goal configurations */
static void CB_act_test_upath(FL_OBJECT *ob, long arg);

/* creates buttons manipulating the collision checkers */
static void g3d_create_kcd_graphics_form(void);
static void g3d_create_kcd_print_object_in_col(void);
static void g3d_create_kcd_which_test_form(void);
static void g3d_create_kcd_which_test(void);
static void g3d_create_kcd_graphics(void);
static void g3d_create_v_collide(void);
static void g3d_create_v_collide_sel(void);
static void g3d_create_col_traj_method(void); /* Modification Fabien */
static void g3d_create_BB_method(void);       /* Modification Fabien */
static void g3d_create_button_microcollision(void); // modif Pepijn/Etienne
static void g3d_create_kcd_dist_choice(void);//modif pepijn

/* creates button for choice by user of seed for random seletion */
/*    and  button for choice by user of number of static calls */
static void g3d_create_random_call_and_seed(void);
/* creates buttons for timing of static collision tests */
static void g3d_create_static(void);
/* creates buttons for timing of semi-dynamic collision test */
static void g3d_create_semi_dynamic(void);




#define P3D_KCD_ALL_NO_REPORT 0 
#define P3D_KCD_ALL_JUST_BOOL 1
#define P3D_KCD_ALL_DISTANCE_ESTIMATE 2
#define P3D_KCD_ALL_DISTANCE_EXACT 3
#define P3D_KCD_ROB_ALL 10
#define P3D_KCD_ROB_ROB 20
#define P3D_KCD_ROB_AUTO 30
#define P3D_KCD_ROB_ENV 40
#define P3D_KCD_EVERYTHING 50
static int kcd_cur_with_report = P3D_KCD_ALL_DISTANCE_ESTIMATE;
static int kcd_cur_mode = P3D_KCD_ROB_ALL ;


/*************************/
/* Env form creation     */
/*************************/

/* env_form permet de tester les validations de trajectoire */

void g3d_create_env_form(void)
{
  ENV_FORM = fl_bgn_form(FL_UP_BOX,280.0,600.0);
  /* g3d_create_checkcoll_obj(); */
  g3d_create_kcd_graphics();
  g3d_create_kcd_print_object_in_col();
  g3d_create_kcd_which_test();
  g3d_create_v_collide();
  g3d_create_v_collide_sel();
  g3d_create_col_traj_method();  /* Modification Fabien */
  g3d_create_BB_method();        /* Modification Fabien */

  g3d_create_random_call_and_seed();
  g3d_create_static();

  g3d_create_semi_dynamic();

  g3d_create_button_microcollision(); //Modif Pepijn/Etienne
  g3d_create_tolerance_slider(); // Modification Carl/Pepijn
  g3d_create_volume_slider(); // Modification Carl
  g3d_create_dmax_slider(); // Modification Pepijn

  fl_end_form();

  g3d_create_kcd_graphics_form();
  g3d_create_kcd_which_test_form();
}

/* buttons manipulating the collision checkers */
static FL_OBJECT *VCOLLIDE_OBJ;
static FL_OBJECT *VCOLLIDESEL_OBJ;

/* D�but modification Fabien : 
   param�trage collision des trajectoires et boites englobantes */

/* Bounding box parameters interface */
static FL_OBJECT *BB_SELECTION_OBJ;
static FL_OBJECT *BB_COMPUTATION_OBJ;
/* Trajectory collision checker */
static FL_OBJECT *COL_TARJ_METHOD_OBJ;

/* Fin modification Fabien */

/* button for choice by user of seed for random seletion */
static FL_OBJECT *SET_SEED_COLL_OBJ;
/* button for choice by user of number of calls to static collision 
   on same configuration */
static FL_OBJECT *SET_REPETE_COLL_OBJ;
/* button for choice by user of number of configurations for which 
   static collision is called */
static FL_OBJECT *SET_CALL_COLL_OBJ;
/* buttons for timing of static collision tests */
static FL_OBJECT *TEST_CUR_COLL_OBJ;
// static FL_OBJECT *COMP_I_V_COLL_OBJ;
/* buttons for timing of semi-dynamic collision test */
static FL_OBJECT *TEST_CUR_PATH_OBJ;
static FL_OBJECT *TEST_CUR_UPATH_OBJ;
// static FL_OBJECT *COMP_I_V_UPATH_OBJ;

static void CB_cancel(FL_OBJECT *ob, long arg)
{
   fl_hide_form(INPUT_FORM);
   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}  

static void CB_repete_input_obj(FL_OBJECT *ob, long arg)
{ 
   set_repete(atoi(fl_get_input(INPUT_OBJ)));

   fl_hide_form(INPUT_FORM);

   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}

static void CB_seed_input_obj(FL_OBJECT *ob, long arg)
{ 
   set_seed(atoi(fl_get_input(INPUT_OBJ)));

   fl_hide_form(INPUT_FORM);

   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}

static void CB_act_get_seed(FL_OBJECT *ob, long arg)
{
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Seed");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Cancel");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_seed_input_obj,0);
  
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
  fl_set_button(SET_SEED_COLL_OBJ,0);
}

static void CB_call_input_obj(FL_OBJECT *ob, long arg)
{ 
   set_call(atoi(fl_get_input(INPUT_OBJ)));

   fl_hide_form(INPUT_FORM);

   fl_free_object(INPUT_OBJ);
   fl_free_form(INPUT_FORM);
}

static void CB_act_get_repete(FL_OBJECT *ob, long arg)
{
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Repetition Number");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Cancel");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_repete_input_obj,0);
  
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
  fl_set_button(SET_REPETE_COLL_OBJ,0);
}


static void CB_act_get_call(FL_OBJECT *ob, long arg)
{
  FL_OBJECT *cancel;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX,90.0,75.0);
  INPUT_OBJ  = fl_add_box(FL_UP_BOX,0.0,0.0,90.0,75.0,"");
  INPUT_OBJ  = fl_add_input(FL_NORMAL_INPUT,40.0,10.0,40.0,30.0,
                                "Call Number");
  fl_set_input_color(INPUT_OBJ, 0, 0); 
  cancel = fl_add_button(FL_PUSH_BUTTON, 40,45,40,20,"Cancel");
  fl_set_object_callback(cancel,CB_cancel,0);
  fl_end_form();
  
  fl_set_object_callback(INPUT_OBJ,CB_call_input_obj,0);
  
  fl_show_form(INPUT_FORM,FL_PLACE_SIZE,TRUE,"");
  fl_set_button(SET_CALL_COLL_OBJ,0);
}


static void CB_kcd_which_test_button(FL_OBJECT *ob, long arg)
{
  int val = fl_get_button(ob);

  if(p3d_col_get_mode() == p3d_col_mode_kcd)
    {
      if(val) 
	{
	  KCD_CHOICE_IS_ACTIVE = TRUE;
	  fl_show_form(KCD_FORM_2,FL_PLACE_SIZE,TRUE,"KCD Choice");	
	}
      else
	{
	  KCD_CHOICE_IS_ACTIVE = FALSE;
	  fl_hide_form(KCD_FORM_2);	
	}
    }
  else
    { 
      KCD_CHOICE_IS_ACTIVE = FALSE;
      printf("WARNING: current collision checker is not KCD !\n");
    }
}
static void g3d_delete_kcd_which_test_button(void)
{int val = fl_get_button(KCD_OBJ_2);

 if(val) fl_hide_form(KCD_FORM_2);

 fl_free_object(KCD_ROB_ALL_OBJ);  
 fl_free_object(KCD_ROB_ROB_OBJ); 
 fl_free_object(KCD_ROB_ENV_OBJ);  
 fl_free_object(KCD_ROB_AUTO_OBJ);
 fl_free_object(KCD_EVERYTHING_OBJ);
 fl_free_object(REL_ERROR_SLIDER_OBJ);
 fl_free_object(KCD_NO_REPORT);
 fl_free_object(KCD_JUST_BOOL);
 fl_free_object(KCD_DIST_EST);
 fl_free_object(KCD_DIST_EXACT);
 fl_free_object(KCD_SHOW_INFO_OBJ); 
 fl_free_object(KCD_SHOW_INFO_2_OBJ); 
 fl_free_form(KCD_FORM_2);
}




static void CB_kcd_graphics(FL_OBJECT *ob, long arg)
{int val = fl_get_button(ob);

  if(val) fl_show_form(KCD_FORM,FL_PLACE_SIZE,TRUE,
		       "KCD");
  else    fl_hide_form(KCD_FORM);
}

static void g3d_delete_kcd_graphics_form(void)
{int val = fl_get_button(KCD_OBJ);

 if(val) fl_hide_form(KCD_FORM);
 fl_free_object(KCD_ALL_OBB_OBJ);
 fl_free_object(KCD_ROB_OBB_OBJ);
 fl_free_object(KCD_AABB_TREE_OBJ);
 fl_free_object(KCD_ALL_AABB_OBJ);
 fl_free_form(KCD_FORM);
}

//static void CB_activate_icollide(FL_OBJECT *ob, long arg)
//{
//	KINEO-DEV : no more i collide
//}

static void CB_activate_vcollide(FL_OBJECT *ob, long arg)
{
  p3d_BB_set_mode_close();
  fl_set_choice(BB_COMPUTATION_OBJ, COMPUTE_BB_CLOSE+1);
  p3d_filter_setoff_linkfilter();
  p3d_col_start(p3d_col_mode_v_collide);
  fl_set_button(VCOLLIDE_OBJ,0);
}

static void CB_activate_vcollide_sel(FL_OBJECT *ob, long arg)
{
  p3d_BB_set_mode_close();
  fl_set_choice(BB_COMPUTATION_OBJ, COMPUTE_BB_CLOSE+1);
  p3d_filter_seton_linkfilter();
  p3d_col_start(p3d_col_mode_v_collide);
  fl_set_button(VCOLLIDESEL_OBJ,0);
}

/* D�but modification Fabien */
static void set_BB_selection_method(p3d_BB_selection_type type)
{
  switch(type) {
    case ACTIVATE_BB_ALL:
      fl_activate_object(BB_COMPUTATION_OBJ);
      fl_set_object_color(BB_COMPUTATION_OBJ, FL_COL1, FL_BLACK);
      break;
    case DEACTIVATE_BB:
      fl_deactivate_object(BB_COMPUTATION_OBJ);
      fl_set_object_color(BB_COMPUTATION_OBJ, FL_INACTIVE_COL,FL_INACTIVE_COL);
      break;
  }
  if (type != p3d_BB_get_selection_method()) {
    p3d_BB_set_selection_method(type);
    p3d_col_activate_env(); 
  }
  fl_set_choice(BB_SELECTION_OBJ, type+1);
}

static void CB_BB_selection_method(FL_OBJECT *ob, long arg)
{
  p3d_BB_selection_type type;

  type = (p3d_BB_selection_type)(fl_get_choice(BB_SELECTION_OBJ)-1);
  set_BB_selection_method(type);
}

static void CB_BB_computation_method(FL_OBJECT *ob, long arg)
{
  p3d_BB_compute_type type;

  type = (p3d_BB_compute_type)(fl_get_choice(BB_COMPUTATION_OBJ)-1);
  p3d_BB_set_computation_method(type);
}

static void CB_col_traj_method(FL_OBJECT *ob, long arg)
{
  p3d_traj_test_type type;

  type = (p3d_traj_test_type)(fl_get_choice(COL_TARJ_METHOD_OBJ)-1);
  p3d_col_env_set_traj_method(type);
}
/* Fin modification Fabien */

static void CB_kcd_all_obb(FL_OBJECT *ob, long arg)
{int val;
 val = fl_get_button(KCD_ALL_OBB_OBJ);
  g3d_set_kcd_draw_all_obbs(val);
  g3d_draw_allwin_active();
}

static void CB_kcd_rob_obb(FL_OBJECT *ob, long arg)
{int val;
 val = fl_get_button(KCD_ROB_OBB_OBJ);
  g3d_set_kcd_draw_robot_obbs(val);
  g3d_draw_allwin_active();
}

static void CB_kcd_aabb_tree(FL_OBJECT *ob, long arg)
{int val;
 val = fl_get_button(KCD_AABB_TREE_OBJ);
  g3d_set_kcd_draw_aabb_hier(val);
  g3d_draw_allwin_active();
}


static void CB_kcd_all_aabb(FL_OBJECT *ob, long arg)
{int val;
 val = fl_get_button(KCD_ALL_AABB_OBJ);
  g3d_set_kcd_draw_all_aabbs(val);
  g3d_draw_allwin_active();
}

static void g3d_create_kcd_graphics_form(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
  KCD_FORM = fl_bgn_form(FL_UP_BOX,270.0,150.0);
  KCD_ALL_OBB_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      10.0,80.0,120.0,60.0,
			      "draw all static \nOBB(tree)s");
  fl_set_call_back(KCD_ALL_OBB_OBJ,CB_kcd_all_obb,0);
  fl_set_button(KCD_ALL_OBB_OBJ, 0);
 
  KCD_ROB_OBB_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      140.0,80.0,120.0,60.0,
			      "draw OBB(tree)s \nof current robot");
  fl_set_call_back(KCD_ROB_OBB_OBJ,CB_kcd_rob_obb,0);
  fl_set_button(KCD_ROB_OBB_OBJ, 0);
 
  KCD_AABB_TREE_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      140.0,10.0,120.0,60.0,
			      "draw static \nAABB tree");
  fl_set_call_back(KCD_AABB_TREE_OBJ,CB_kcd_aabb_tree,0);
  fl_set_button(KCD_AABB_TREE_OBJ, 0);
 
  KCD_ALL_AABB_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      10.0,10.0,120.0,60.0,
			      "draw all AABBs \nof static primitives");
  fl_set_call_back(KCD_ALL_AABB_OBJ,CB_kcd_all_aabb,0);
  fl_set_button(KCD_ALL_AABB_OBJ, 0);



  fl_end_form();

}

static void CB_kcd_print_object_in_col(FL_OBJECT *ob, long arg){
  p3d_obj *o1Pt, *o2Pt;
  p3d_col_test_choice();
  p3d_kcd_get_pairObjInCollision ( &o1Pt, &o2Pt );
  printf("colliding pair: %s %s\n", o1Pt->name, o2Pt->name);
}

static void g3d_create_kcd_which_test_form(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
 KCD_FORM_2 = fl_bgn_form(FL_UP_BOX,300.0,470.0); 
 g3d_create_kcd_dist_choice();
 fl_end_form();
}

/* -> new window with buttons for KCD choice */
static void g3d_create_kcd_which_test(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
  KCD_OBJ_2 = fl_add_button(FL_PUSH_BUTTON,
			      10.0,60.0,120.0,40.0,
			      "KCD choice");
  fl_set_call_back(KCD_OBJ_2,CB_kcd_which_test_button,0);
}

/* -> new window with buttons for KCD graphics */
static void g3d_create_kcd_graphics(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
  KCD_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      10.0,10.0,80.0,40.0,
			      "KCD graphics");
  fl_set_call_back(KCD_OBJ,CB_kcd_graphics,0);
}

static void g3d_create_kcd_print_object_in_col(void)
{
  KCD_PRINT_COL = fl_add_button(FL_NORMAL_BUTTON,
            95.0,10.0,30.0,40.0,
            "print\nKCD\ncol");
  fl_set_call_back(KCD_PRINT_COL,CB_kcd_print_object_in_col,0);
}

/* KINEO DEV : no more i collide.
 -> activates I_COLLIDE, and thus deactivates the other collision checkers,
   -> prepares data if not done before 
static void g3d_create_i_collide(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
 ICOLLIDE_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      10.0,60.0,120.0,40.0,
			      "Activate Icollide");
  fl_set_call_back(ICOLLIDE_OBJ,CB_activate_icollide,0);
}
*/
/* -> activates V_COLLIDE, and thus deactivates the other collision checkers,
   -> prepares data if not done before */
static void g3d_create_v_collide(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
 VCOLLIDE_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      150.0,110.0,120.0,40.0,
			      "Activate Vcollide by Device");
  fl_set_call_back(VCOLLIDE_OBJ,CB_activate_vcollide,0);
}

/* -> activates V_COLLIDE, and thus deactivates the other collision checkers,
   -> prepares data if not done before */
static void g3d_create_v_collide_sel(void)
{void CB_checkcoll_obj(FL_OBJECT *ob, long arg);
 VCOLLIDESEL_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      10.0,110.0,120.0,40.0,
			      "Activate Vcollide by Link");
  fl_set_call_back(VCOLLIDESEL_OBJ,CB_activate_vcollide_sel,0);
}

/* begin modif Carl */

static void  CB_vol_obj(FL_OBJECT *ob, long arg)
{double val = fl_get_slider_value(ob);

 p3d_col_set_user_defined_small_volume(val); 
}

void g3d_create_volume_slider()
{
  double v,vmax,vmin,val;



  p3d_col_get_user_defined_small_volume(&v); /* initial value */
  /* printf("v=%f\n",v); */
  val = v;
  if(v==0.0) 
    {
      vmax =  p3d_get_env_dmax() *10;
      //      vmax = 10*(v+10.);
    }
  else 
    vmax = 10*v;
  
  vmin = 0.0;

  fl_add_box(FL_UP_BOX,10.0,515.0,40.0,30.0,"Vol.");
  VOL_OBJ = fl_add_valslider(FL_HOR_SLIDER,50.0,515.0,220.0,30.0,"");
  fl_set_slider_bounds(VOL_OBJ,vmin,vmax);
  fl_set_object_lsize(VOL_OBJ,FL_MEDIUM_SIZE);
  fl_set_slider_value(VOL_OBJ,val);
  fl_set_call_back(VOL_OBJ,CB_vol_obj,0);
}

void g3d_reinit_volume_slider()
{
  double v,vmax,vmin,val;
  p3d_col_get_user_defined_small_volume(&v); /* initial value */
  /* printf("v=%f\n",v); */
  val = v;
  if(v==0.0) 
    vmax = 10*(v+10.);
  else 
    vmax = 10*v;
  vmin = v;

  fl_set_slider_bounds(VOL_OBJ,vmin,vmax);
  fl_set_slider_value(VOL_OBJ,val);
}


/* Modifications Pepijn june 2001
 * concerning the tolerance slider and the dmax slider
 */

static void  CB_rel_error_slider_obj(FL_OBJECT *ob, long arg)
{
 double val = fl_get_slider_value(ob);
 /* set dmax to given value */
 p3d_col_set_relative_error(val);
 /* dmax is not used anymore in kcd so we just have to set the
  * environment variable
  */
}



static void  CB_dmax_slider_obj(FL_OBJECT *ob, long arg)
{
 double val = fl_get_slider_value(ob);
 /* set dmax to given value */
 p3d_set_env_dmax(val);
 /* dmax is not used anymore in kcd so we just have to set the
  * environment variable
  */
}

static void  CB_tol_obj(FL_OBJECT *ob, long arg)
{
 double val = fl_get_slider_value(ob);
 /* set object tolerance to given value */
 p3d_set_env_object_tolerance(val);
 /* p3d_col_set_tolerance() will set the tolerance for 
  * collision checkers who are capable of dealing with tolerance
  * in june 2001 it is only used for KCD
  */
 p3d_col_set_tolerance(val); 
}

void g3d_create_tolerance_slider()
 {
  double v,vmax,vmin,val;
  v = p3d_get_env_object_tolerance();
  p3d_col_set_tolerance(v); 
  /*  printf("v=%f\n",v); */
  val = v;
 
  vmin = 0.0;
  vmax = 10*v;
  if(vmax == 0.0)
    {
      vmax = 500;
    } 
  fl_add_box(FL_UP_BOX,10.0,485.0,40.0,30.0,"Tol.");
  TOL_OBJ = fl_add_valslider(FL_HOR_SLIDER,50.0,485.0,220.0,30.0,"");
  fl_set_slider_bounds(TOL_OBJ,vmin,vmax);
  fl_set_object_lsize(TOL_OBJ,FL_MEDIUM_SIZE);
  fl_set_slider_value(TOL_OBJ,val);
  fl_set_call_back(TOL_OBJ,CB_tol_obj,0);
 } 

void g3d_reinit_tolerance_slider()
{
  double v,vmax,vmin,val;
  v = p3d_get_env_object_tolerance(); 
  p3d_col_set_tolerance(v); 
  /*  printf("v=%f\n",v); */
  val = v;
  vmax = 10*v;
  vmin = 0.0;
  fl_set_slider_bounds(TOL_OBJ,vmin,vmax);
  fl_set_slider_value(TOL_OBJ,val);
}

/* end modif Carl */

void g3d_create_dmax_slider()
{
  double v,vmax,vmin,val;
  /* the initial value is still the same as tolerance
   * need new fonctions to set initial value
   */
  v = p3d_get_env_dmax(); /* initial value */
 
  /* printf("v=%f\n",v); */
  val = v;
  vmax = 10*v;
  vmin = EPS4; /* Dmax may not be zero or smaller */

  fl_add_box(FL_UP_BOX,10.0,545.0,40.0,30.0,"dmax.");
  DMAX_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,50.0,545.0,220.0,30.0,"");
  fl_set_slider_bounds(DMAX_SLIDER_OBJ,vmin,vmax);
  fl_set_object_lsize(DMAX_SLIDER_OBJ,FL_MEDIUM_SIZE);
  fl_set_slider_value(DMAX_SLIDER_OBJ,val);
  fl_set_call_back(DMAX_SLIDER_OBJ,CB_dmax_slider_obj,0);
}
 
void g3d_create_rel_error_slider()
{
  double vmax,vmin,val;
  /* the initial value is still the same as tolerance
   * need new fonctions to set initial value
   */
  /* printf("v=%f\n",v); */
  val = 0;
  vmax = 99.9;
  vmin = 0; /* Dmax may not be zero or smaller */

  fl_add_box(FL_UP_BOX,10.0,405.0,60.0,30.0,"Error %");
  REL_ERROR_SLIDER_OBJ = fl_add_valslider(FL_HOR_SLIDER,70.0,405.0,220.0,30.0,"");
  fl_set_slider_bounds( REL_ERROR_SLIDER_OBJ ,vmin,vmax);
  fl_set_object_lsize( REL_ERROR_SLIDER_OBJ ,FL_MEDIUM_SIZE);
  fl_set_slider_value( REL_ERROR_SLIDER_OBJ,val);
  fl_set_call_back( REL_ERROR_SLIDER_OBJ ,CB_rel_error_slider_obj,0);
}
 

void g3d_reinit_dmax_slider()
{
  double v,vmax,vmin,val;
  v = p3d_get_env_dmax(); /* initial value */
  /* printf("v=%f\n",v); */
  val = v;
  vmax = 10*v;
  vmin = EPS4;
  
  fl_set_slider_bounds(DMAX_SLIDER_OBJ,vmin,vmax);
  fl_set_slider_value(DMAX_SLIDER_OBJ,val);
}




/* permet de tracer ou non le graphe courant */
static void CB_microcollision(FL_OBJECT *ob, long arg)
{
  int micro_col = p3d_col_get_microcollision();
  double dmax = fl_get_slider_value(DMAX_SLIDER_OBJ);
  double vmin,vmax;
  fl_get_slider_bounds(DMAX_SLIDER_OBJ,&vmin,&vmax);
  micro_col = !micro_col;
  p3d_col_set_microcollision(micro_col);
  if ( (p3d_col_get_mode()!=p3d_col_mode_kcd) && micro_col )
    { 
      printf("Warning : collision checker must be KCD and using distances\n");
    }
  if(micro_col)
    {
      printf("Microcollision avoidance enabled\n");    
      dmax /=100;
      printf("dmax is changed : old dmax = %f, new dmax = %f\n", dmax*100, dmax);
      fl_set_slider_bounds(DMAX_SLIDER_OBJ,vmin,vmax/100);
    }
  else
    {
      printf("Microcollision avoidance disabled\n");
      dmax *= 100;
      printf("dmax is changed : old dmax = %f, new dmax = %f\n", dmax/100, dmax);
      fl_set_slider_bounds(DMAX_SLIDER_OBJ,vmin,vmax*100);
    }
  /* set dmax to new value */
  p3d_set_env_dmax(dmax);
  fl_set_slider_value(DMAX_SLIDER_OBJ,dmax);
  //fl_set_button( MICROCOLLISION_OBJ,micro_col);	 
}


static void g3d_create_button_microcollision(void)
{
  MICROCOLLISION_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,15,455,55,30,"Microcollision Avoidance");
  fl_set_object_color(MICROCOLLISION_OBJ ,FL_MCOL,FL_GREEN);
  fl_set_call_back(MICROCOLLISION_OBJ,CB_microcollision,0);
}



static void CB_kcd_show_info(FL_OBJECT *ob, long arg)
{
  int show_info = g3d_get_kcd_show_info();
  if(show_info == 3)
    show_info = 2;
  else if(show_info == 2)
    show_info = 3;
  else if(show_info == 1)
    show_info = 0;
  else
    show_info = 1;
  g3d_set_kcd_show_info(show_info);
}

static void CB_kcd_show_info_2(FL_OBJECT *ob, long arg)
{
  int show_info = g3d_get_kcd_show_info();
 if(show_info == 3)
    show_info = 1;
  else if(show_info == 2)
    show_info = 0;
  else if(show_info == 1)
    show_info = 3;
  else
    show_info = 2;
  g3d_set_kcd_show_info(show_info);
}

/* end modif Pepijn */



/* D�but modification Fabien */
static void g3d_create_col_traj_method(void)
{
  FL_OBJECT * obj;
  obj = fl_add_box(FL_FLAT_BOX, 145, 72, 55, 28, "Traj col.\n  method");
  fl_set_object_lalign(obj, FL_ALIGN_LEFT+FL_ALIGN_INSIDE);
  COL_TARJ_METHOD_OBJ = fl_add_choice(FL_NORMAL_CHOICE,
				      200.0,72.0,70.0,28.0,"");
  fl_set_object_boxtype(COL_TARJ_METHOD_OBJ, FL_BORDER_BOX);
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "classic");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "dichotomic");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "classic +\nautocol");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "dichotomic\n+ autocol");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "classic + others");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "dicho.+ others");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "classic + others\n+ autocol");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "dicho.+ others\n+ autocol");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "classic current");
  fl_addto_choice(COL_TARJ_METHOD_OBJ, "dichotomic +\ncurrent");
  fl_set_call_back(COL_TARJ_METHOD_OBJ,CB_col_traj_method,0);
  fl_set_choice(COL_TARJ_METHOD_OBJ, p3d_col_env_get_traj_method()+1);
}

static void g3d_create_BB_method(void)
{
  FL_OBJECT * obj;

  obj = fl_add_box(FL_FLAT_BOX, 150, 20, 60, 20, "Selection");
  fl_set_object_lalign(obj, FL_ALIGN_LEFT+FL_ALIGN_INSIDE);
  BB_SELECTION_OBJ = fl_add_choice(FL_NORMAL_CHOICE, 220.0,20.0,47.0,20.0,"");
  fl_set_object_boxtype(BB_SELECTION_OBJ, FL_BORDER_BOX);
  fl_addto_choice(BB_SELECTION_OBJ, "all");
  fl_addto_choice(BB_SELECTION_OBJ, "none");
  fl_set_call_back(BB_SELECTION_OBJ, CB_BB_selection_method,0);
  fl_set_choice(BB_SELECTION_OBJ, p3d_BB_get_selection_method()+1);

  obj = fl_add_box(FL_FLAT_BOX, 150, 43, 60, 20, "Computation");
  fl_set_object_lalign(obj, FL_ALIGN_LEFT+FL_ALIGN_INSIDE);
  BB_COMPUTATION_OBJ = fl_add_choice(FL_NORMAL_CHOICE, 
				     220.0,43.0,47.0,20.0,"");
  fl_set_object_boxtype(BB_COMPUTATION_OBJ, FL_BORDER_BOX);
  fl_addto_choice(BB_COMPUTATION_OBJ, "close");
  fl_addto_choice(BB_COMPUTATION_OBJ, "large");
  fl_addto_choice(BB_COMPUTATION_OBJ, "col.");
  fl_set_call_back(BB_COMPUTATION_OBJ, CB_BB_computation_method, 0);
  fl_set_choice(BB_COMPUTATION_OBJ, p3d_BB_get_computation_method()+1);

  fl_add_frame(FL_ENGRAVED_FRAME,150,10,120,57,"");
  fl_add_box(FL_FLAT_BOX, 170, 6, 80, 8, "Bounding Box");
}

/* Fin modification Fabien */

void g3d_delete_env_form(void)
{
  if(fl_get_button(envparams_obj)){fl_hide_form(ENV_FORM);}
  /*   fl_free_object(CHECKCOLL_OBJ); */
  g3d_delete_kcd_graphics_form();
  g3d_delete_kcd_which_test_button();
  fl_free_object(KCD_OBJ);
  fl_free_object(KCD_OBJ_2);
  fl_free_object(VCOLLIDE_OBJ);
  fl_free_object(VCOLLIDESEL_OBJ);
  /* D�but modification Fabien */
  fl_free_object(COL_TARJ_METHOD_OBJ);
  fl_free_object(BB_COMPUTATION_OBJ);
  fl_free_object(BB_SELECTION_OBJ);
  /* Fin modification Fabien */

  fl_free_object(TEST_CUR_COLL_OBJ);
//  fl_free_object(COMP_I_V_COLL_OBJ);
  fl_free_object(TOL_OBJ);
  fl_free_object(VOL_OBJ);
  fl_free_object(DMAX_SLIDER_OBJ); // modif Pepijn
  fl_free_object(MICROCOLLISION_OBJ);

  fl_free_form(ENV_FORM);
}

static void CB_act_test_coll(FL_OBJECT *ob, long arg)
{
  /* choose 100 configurations, test them on static collision 
     with currently active collision detector */
  if((p3d_BB_get_computation_method() == COMPUTE_BB_COL)  &&
     (p3d_col_get_mode()==p3d_col_mode_v_collide))
    printf("error: change box type\n");
  else
    test_current_coll_call_times();
  fl_set_button(TEST_CUR_COLL_OBJ,0);
}

/* KINEO DEV: no more I-collide 
static void CB_act_comp_coll(FL_OBJECT *ob, long arg)  
{ 
  // choose 100 configurations, test them on static collision  
  //   with collision detectors I_COLLIDE and V_COLLIDE  
  if(box_type_in_use == BB_COLL)  
    printf("error: change box type \n"); 
  else  
    compare_I_V_collide_call_times();  
  fl_set_button(COMP_I_V_COLL_OBJ,0);       
} 
*/

static void CB_act_test_path(FL_OBJECT *ob, long arg)
{
  /*  test_current_coll_test along a path  */

  
  if((p3d_BB_get_computation_method() == COMPUTE_BB_COL) &&
     (p3d_col_get_mode()==p3d_col_mode_v_collide))
    printf("error: change box type\n");
  else
    test_path_coll_test();
  fl_set_button(TEST_CUR_PATH_OBJ,0);
}

/* KINEO DEV: no more I-collide
static void CB_act_comp_path(FL_OBJECT *ob, long arg)
{
  //  compare_I_V_collide along a path
  if(box_type_in_use == BB_COLL)\
    printf("error: change box type\n");
  else
    compare_I_V_path_collide();
  fl_set_button(COMP_I_V_PATH_OBJ,0);
}
*/


/* choix de la strategie globale */
static void CB_which_kcd_mode(FL_OBJECT *ob, long arg)
{
  if(arg == 0) { kcd_cur_mode = P3D_KCD_ROB_ALL; }
  if(arg == 1) { kcd_cur_mode = P3D_KCD_ROB_ROB; }
  if(arg == 2) { kcd_cur_mode = P3D_KCD_ROB_AUTO; }
  if(arg == 3) { kcd_cur_mode = P3D_KCD_ROB_ENV; }
  if(arg == 4) { kcd_cur_mode = P3D_KCD_EVERYTHING; }
  set_kcd_which_test((p3d_type_col_choice) (kcd_cur_mode+kcd_cur_with_report)); 
} 

/* choix de la strategie globale */
static void CB_which_kcd_precision(FL_OBJECT *ob, long arg)
{
  if(arg == 0) { kcd_cur_with_report = P3D_KCD_ALL_NO_REPORT;}
  if(arg == 1) { kcd_cur_with_report = P3D_KCD_ALL_JUST_BOOL;}
  if(arg == 2) { kcd_cur_with_report = P3D_KCD_ALL_DISTANCE_ESTIMATE;}
  if(arg == 3) { kcd_cur_with_report = P3D_KCD_ALL_DISTANCE_EXACT;}

  set_kcd_which_test((p3d_type_col_choice) (kcd_cur_mode+kcd_cur_with_report)); 
} 


static void g3d_create_kcd_dist_choice(void)
{
  FL_OBJECT *obj;
  FL_OBJECT *obj_2;
  FL_OBJECT *obj_3;

  int base =  20;

  /* KCD MODE */
  obj = fl_add_frame(FL_ENGRAVED_FRAME,5,15,290,150,""); 
  obj = fl_add_box(FL_FLAT_BOX,110,5,70,30," KCD MODE ");

  GROUP2 = fl_bgn_group();

  KCD_ROB_ALL_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,20,base,55,30,"COMPLETE test for CURRENT ROBOT");
  fl_set_object_color( KCD_ROB_ALL_OBJ  ,FL_MCOL,FL_BLUE);
  fl_set_call_back( KCD_ROB_ALL_OBJ ,CB_which_kcd_mode,0);

  KCD_ROB_ROB_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+30,55,30,"ROBOT 0 against ROBOT 1");
  fl_set_object_color( KCD_ROB_ROB_OBJ ,FL_MCOL,FL_BLUE);
  fl_set_call_back( KCD_ROB_ROB_OBJ  ,CB_which_kcd_mode,1);

  KCD_ROB_AUTO_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+60,55,30,"AUTOCOLLISION test for CURRENT ROBOT");
  fl_set_object_color( KCD_ROB_AUTO_OBJ ,FL_MCOL,FL_BLUE);
  fl_set_call_back( KCD_ROB_AUTO_OBJ ,CB_which_kcd_mode,2);

  KCD_ROB_ENV_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+90,55,30,"CURRENT ROBOT against ENVIRONMENT");
  fl_set_object_color(KCD_ROB_ENV_OBJ ,FL_MCOL,FL_BLUE);
  fl_set_call_back(KCD_ROB_ENV_OBJ ,CB_which_kcd_mode,3);

  KCD_EVERYTHING_OBJ = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+120,55,30,"COMPLETE test for ALL ROBOTS");
  fl_set_object_color(  KCD_EVERYTHING_OBJ ,FL_MCOL,FL_BLUE);
  fl_set_call_back(  KCD_EVERYTHING_OBJ ,CB_which_kcd_mode,4);

  GROUP2 = fl_end_group();

  fl_set_button(KCD_ROB_ALL_OBJ,1);

  /* KCD WITH_REPORT */
  obj_2 = fl_add_frame(FL_ENGRAVED_FRAME,5,185,290,140,""); 
  obj_2 = fl_add_box(FL_FLAT_BOX,100,170,100,30," KCD PRECISION ");

  GROUP3 = fl_bgn_group();

  base = 190;

  KCD_NO_REPORT = fl_add_checkbutton(FL_RADIO_BUTTON,20,base,55,30,"NO REPORT");
  fl_set_object_color( KCD_NO_REPORT  ,FL_MCOL,FL_GREEN);
  fl_set_call_back( KCD_NO_REPORT ,CB_which_kcd_precision,0);

  KCD_JUST_BOOL = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+30,55,30,"JUST BOOL");
  fl_set_object_color(KCD_JUST_BOOL  ,FL_MCOL,FL_GREEN);
  fl_set_call_back( KCD_JUST_BOOL  ,CB_which_kcd_precision,1);

  KCD_DIST_EST = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+60,55,30,"ESTIMATED DISTANCES");
  fl_set_object_color(KCD_DIST_EST ,FL_MCOL,FL_GREEN);
  fl_set_call_back( KCD_DIST_EST ,CB_which_kcd_precision,2);

  KCD_DIST_EXACT = fl_add_checkbutton(FL_RADIO_BUTTON,20,base+90,55,30,"EXACT DISTANCES");
  fl_set_object_color( KCD_DIST_EXACT,FL_MCOL,FL_GREEN);
  fl_set_call_back(KCD_DIST_EXACT ,CB_which_kcd_precision,3);

  GROUP3 = fl_end_group();

  fl_set_button(KCD_DIST_EST,1);

/* EXACT DISTANCES ONLY */

  obj_3 = fl_add_frame(FL_ENGRAVED_FRAME,5,340,290,110,""); 
  obj_3 = fl_add_box(FL_FLAT_BOX,80,330,150,25," ONLY FOR EXACT DISTANCES ");

  base = 345;
  KCD_SHOW_INFO_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,20,base,55,30,"DRAW CLOSEST POINTS");
  fl_set_object_color( KCD_SHOW_INFO_OBJ ,FL_MCOL,FL_RED);
  fl_set_call_back(   KCD_SHOW_INFO_OBJ ,CB_kcd_show_info,0);

  KCD_SHOW_INFO_2_OBJ = fl_add_checkbutton(FL_PUSH_BUTTON,20,base+30,55,30,"DRAW CLOSEST POINTS AND PRINT INFORMATION");
  fl_set_object_color( KCD_SHOW_INFO_2_OBJ ,FL_MCOL,FL_RED);
  fl_set_call_back(   KCD_SHOW_INFO_2_OBJ ,CB_kcd_show_info_2,0);

  g3d_create_rel_error_slider();

}















static void CB_act_test_upath(FL_OBJECT *ob, long arg)
{
  /*  test_current_coll_test along a path between user-defined Q_i and Q_g */
  if((p3d_BB_get_computation_method() == COMPUTE_BB_COL) &&
     (p3d_col_get_mode()==p3d_col_mode_v_collide))
    printf("error: change box type\n");
  else
    test_upath_coll_test();
  fl_set_button(TEST_CUR_UPATH_OBJ,0);
}

/* KINEO DEV: no more I-collide
static void CB_act_comp_upath(FL_OBJECT *ob, long arg)
{
  //  compare_I_V_collide along a path between user-defined Q_i and Q_g
  if(box_type_in_use == BB_COLL)
    printf("error: change box type\n");
  else
    compare_I_V_upath_collide();
  fl_set_button(COMP_I_V_UPATH_OBJ,0);
}
*/

static void g3d_create_random_call_and_seed(void)
{
  FL_OBJECT *obj;

  obj = fl_add_frame(FL_ENGRAVED_FRAME,8,160,264,52,""); 
  obj = fl_add_box(FL_FLAT_BOX,100,157,80,10,"Init Params");

  PARAM_GROUP = fl_bgn_group();
  SET_SEED_COLL_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      15.0,170.0,80.0,40.0,
			      "Set Seed");
  SET_REPETE_COLL_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      100.0,170.0,80.0,40.0,
			      "Set Repete");
  SET_CALL_COLL_OBJ = fl_add_button(FL_PUSH_BUTTON,
			      190.0,170.0,80.0,40.0,
			      "Mets Seuil");
  PARAM_GROUP = fl_end_group();

  fl_set_object_callback(SET_CALL_COLL_OBJ,CB_act_get_call,0);
  fl_set_object_callback(SET_REPETE_COLL_OBJ,CB_act_get_repete,0);
  fl_set_object_callback(SET_SEED_COLL_OBJ,CB_act_get_seed,0);
}

static void g3d_create_static(void)
{
  FL_OBJECT *obj;
  void CB_checkcoll_obj(FL_OBJECT *ob, long arg);

  obj = fl_add_frame(FL_ENGRAVED_FRAME,8,390,264,52,""); 
  obj = fl_add_box(FL_FLAT_BOX,80,385,140,10,"Random Static Collision Test");

  PARAM_GROUP = fl_bgn_group();
  TEST_CUR_COLL_OBJ = fl_add_button(FL_PUSH_BUTTON,
				    15.0,400.0,120.0,40.0,
				    "Current Checker");
  /*  COMP_I_V_COLL_OBJ = fl_add_button(FL_PUSH_BUTTON,\
				    145.0,400.0,120.0,40.0,\
				    "Compare I-V COLLIDE");   */
  PARAM_GROUP = fl_end_group();

  fl_set_call_back(TEST_CUR_COLL_OBJ,CB_act_test_coll,0);
//  fl_set_call_back(COMP_I_V_COLL_OBJ,CB_act_comp_coll,0);
}

static void g3d_create_semi_dynamic(void)
{
  FL_OBJECT *obj;
  void CB_checkcoll_obj(FL_OBJECT *ob, long arg);

  obj = fl_add_frame(FL_ENGRAVED_FRAME,8,230,264,140,""); 
  obj = fl_add_box(FL_FLAT_BOX,80,226,140,10,"Semi-Dynamic Collision Test");

  PARAM_GROUP = fl_bgn_group();

  obj = fl_add_box(FL_FLAT_BOX,80,252,140,10,"User-defined Configurations");
  /* current clasher, user path */
  TEST_CUR_UPATH_OBJ = fl_add_button(FL_PUSH_BUTTON,
				     15.0,265.0,120.0,40.0,
				     "Current Checker");
  /* compare clashers, user path */
  /*  COMP_I_V_UPATH_OBJ = fl_add_button(FL_PUSH_BUTTON,\
				     145.0,265.0,120.0,40.0,\
				     "Compare I-V COLLIDE"); */

  obj = fl_add_box(FL_FLAT_BOX,90,312,120,10,"Random Local Path");
  /* current clasher, random path */
  TEST_CUR_PATH_OBJ = fl_add_button(FL_PUSH_BUTTON,
				    15.0,325.0,120.0,40.0,
				    "Current Checker");
  /* compare clashers, random path */
  /*  COMP_I_V_PATH_OBJ = fl_add_button(FL_PUSH_BUTTON,\
				    145.0,325.0,120.0,40.0,\
				    "Compare I-V COLLIDE");\ */

  PARAM_GROUP = fl_end_group();

  /* current clasher, random path */
  fl_set_call_back(TEST_CUR_PATH_OBJ,CB_act_test_path,0);
  /* compare clashers, random path */
//  fl_set_call_back(COMP_I_V_PATH_OBJ,CB_act_comp_path,0);
  /* current clasher, user path */
  fl_set_call_back(TEST_CUR_UPATH_OBJ,CB_act_test_upath,0);
  /* compare clashers, user path */
//  fl_set_call_back(COMP_I_V_UPATH_OBJ,CB_act_comp_upath,0);
}


