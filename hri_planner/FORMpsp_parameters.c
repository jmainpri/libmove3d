#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"


FL_FORM  *PSP_PARAMETERS_FORM = NULL;
int PSP_MA_SEGMENTS  = 11;
int PSP_MA_LAYERS    = 3;
int PSP_SRCH_MTD[3]  = {PSP_FRONT,PSP_ORDERED,PSP_BCF};
int PSP_SRCH_TYPE    = 0;
int PSP_SRCH_GOAL    = 0;
int PSP_NEXT_TASK = PSP_NO_TASK;
double PSP_PS_TRSHLD = 70.0;
double PSP_DIFF_PS_TRSHLD = 5.0;
double PSP_DIST2OBJ_TRSHLD = 0.3;
int PSP_STOP_DEEP = 5;

static void g3d_create_model_area_bars(void);
static void g3d_create_shows_obj(void);
static void g3d_create_select_animated_element_obj(void);
static void g3d_create_cone_bars(void);
static void g3d_create_btns(void);
static void g3d_create_element_choice_obj(void);
static void g3d_create_camera_bars(void);
static void g3d_create_cam_objs(void);
static void g3d_create_win_mode_objs(void);

static void fill_bodies(void);
static void fill_methods(void);

static void psp_update_objects();

static void g3d_delete_bars_obj(void);
static void g3d_delete_shows_obj(void);
/************************/
/* Option form creation */
/************************/
void g3d_create_psp_parameters_form(void)
{
  PSP_PARAMETERS_FORM = fl_bgn_form(FL_UP_BOX,370.0,520.0);
  g3d_create_element_choice_obj();
  g3d_create_btns();
  g3d_create_model_area_bars();
  g3d_create_shows_obj();
  g3d_create_select_animated_element_obj();
  g3d_create_cone_bars();
  g3d_create_camera_bars();
  g3d_create_cam_objs();
  g3d_create_win_mode_objs();
  fl_end_form();
  //psp_update_objects();
}

static FL_OBJECT  *BR_PSP_MA_ANGLE;
static FL_OBJECT  *BR_PSP_MA_MAX_DIST;
static FL_OBJECT  *BR_PSP_MA_MIN_DIST;

static FL_OBJECT  *BR_PSP_MA_SEGMENTS;
static FL_OBJECT  *BR_PSP_MA_LAYERS;
static FL_OBJECT  *DL_PSP_SRCH_TRSHLD;
static FL_OBJECT  *CH_SRCH_MTD;
static FL_OBJECT  *CH_SRCH_TYPE;
static FL_OBJECT  *CH_SRCH_GOAL;
static FL_OBJECT  *PST_PSP_POS;

static FL_OBJECT  *BR_PSP_DIST2OBJ_TRSHLD;
static FL_OBJECT  *BR_PSP_DIFF_PS_TRSHLD;

static FL_OBJECT  *BR_PSP_VF_HANGLE;
static FL_OBJECT  *BR_PSP_VF_VANGLE;

static FL_OBJECT  *BR_CAM_X;
static FL_OBJECT  *BR_CAM_Y;
static FL_OBJECT  *BR_CAM_Z;
static FL_OBJECT  *CH_CAM_BODY;
static FL_OBJECT  *BR_CAM_PAN;
static FL_OBJECT  *BR_CAM_TILT;

static FL_OBJECT  *GROUP_CAM;
static FL_OBJECT  *RBTN_CAM_X;
static FL_OBJECT  *RBTN_CAM_Y;
static FL_OBJECT  *RBTN_CAM_Z;

static FL_OBJECT  *BTN_PSP_INIT_VALUES;

static FL_OBJECT  *CHKBTN_OBJECTIF;
static FL_OBJECT  *CHKBTN_SHOW_MODEL_AREA;
static FL_OBJECT  *CHKBTN_SHOW_CONE;
static FL_OBJECT  *CHKBTN_BODY_OBJECTIF;

static FL_OBJECT  *ACTUAL_ANIM_ELEMENT;

static FL_OBJECT  *ELEMENTGROUP;
static FL_OBJECT  *RBTN_ROBOT;
static FL_OBJECT  *RBTN_OBJECT;

static FL_OBJECT  *ELEMENTGROUP2;
static FL_OBJECT  *RBTN_WIN_MODE_0;
static FL_OBJECT  *RBTN_WIN_MODE_1;
static FL_OBJECT  *RBTN_WIN_MODE_2;
static FL_OBJECT  *BTN_PSP_GO_FOR_OBJECT;
static FL_OBJECT  *BTN_PSP_SHOW_CONFS;
static FL_OBJECT  *BTN_PSP_SET_ROBOT;
//static FL_OBJECT  *CHKBTN_WITH_CAMERA;//later addition

static FL_OBJECT  *CH_TASK;

static p3d_rob *sel_robot;
static p3d_obj *sel_object;
static p3d_rob *tHuman;
static p3d_rob *tRobot;

/********************************************************************************************************************/  
/****************************************** PSP PARAMETERS INTERFACE ************************************************/
/********************************************************************************************************************/

void g3d_show_psp_parameters_form(void)
{ 
  fl_show_form(PSP_PARAMETERS_FORM,
               FL_PLACE_SIZE,TRUE, "PSP Parameters");
  psp_update_objects();
}

void g3d_hide_psp_parameters_form(void)
{ 
  fl_hide_form(PSP_PARAMETERS_FORM);
}

/***********************************/

static void psp_update_objects()
{
	if(fl_get_button(RBTN_ROBOT))
	{
		fl_show_object(CHKBTN_SHOW_CONE);
		fl_set_button(CHKBTN_SHOW_MODEL_AREA,p3d_is_pos_area_showed(sel_robot));
		fl_set_button(CHKBTN_SHOW_CONE,p3d_is_view_field_showed(sel_robot));
		fl_set_button(CHKBTN_OBJECTIF,p3d_get_rob_select_status(sel_robot));
		fl_set_button(CHKBTN_BODY_OBJECTIF,p3d_get_obj_select_status(sel_object));
		fl_set_slider_value(BR_PSP_MA_ANGLE, sel_robot->angle_range);
		fl_set_slider_value(BR_PSP_MA_MAX_DIST, sel_robot->max_pos_range);
		fl_set_slider_value(BR_PSP_MA_MIN_DIST, sel_robot->min_pos_range);
		fl_set_slider_value(BR_PSP_MA_SEGMENTS, PSP_MA_SEGMENTS);
		fl_set_slider_value(BR_PSP_MA_LAYERS, PSP_MA_LAYERS);
		fl_set_slider_value(BR_PSP_VF_HANGLE, sel_robot->cam_h_angle);
		fl_set_slider_value(BR_PSP_VF_VANGLE, sel_robot->cam_v_angle);  
		fl_set_slider_value(BR_CAM_X,sel_robot->cam_pos[0]);
		fl_set_slider_value(BR_CAM_Y,sel_robot->cam_pos[1]);
		fl_set_slider_value(BR_CAM_Z,sel_robot->cam_pos[2]); 
		fl_set_slider_value(BR_CAM_PAN,sel_robot->cam_pan); 
		fl_set_slider_value(BR_CAM_TILT,sel_robot->cam_tilt);
		switch (sel_robot->cam_axe)
		{
			case 0:
				fl_set_button(RBTN_CAM_X, 1);
				break;
			case 1:
				fl_set_button(RBTN_CAM_Y, 1);
				break;
			case 2:
				fl_set_button(RBTN_CAM_Z, 1);
				break;
		}
		fill_bodies();
	}
	else
	{
		fl_set_slider_value(BR_PSP_MA_MAX_DIST, sel_object->max_pos_range);
		fl_set_slider_value(BR_PSP_MA_MIN_DIST, sel_object->min_pos_range);
		fl_set_button(CHKBTN_SHOW_MODEL_AREA,sel_object->show_pos_area);
		fl_set_button(CHKBTN_OBJECTIF,p3d_get_obj_select_status(sel_object));
		fl_hide_object(CHKBTN_SHOW_CONE);
	}
}

static void fill_methods(void)
{
  fl_addto_choice(CH_SRCH_MTD, "Around|Front");
  fl_addto_choice(CH_SRCH_TYPE,"Ordered|Secuential|Random|Ran. List");
  fl_addto_choice(CH_SRCH_GOAL,"FFFO|BCF|DEEP");
  fl_addto_choice(CH_TASK,"No Task|To Give|To Pick");
	
}

static void CB_update_model_area_bars(FL_OBJECT *ob, long arg)
{
  char oblabel[50];
  double val_bar;
  int zchoice;
  if (arg<7)
    val_bar = fl_get_slider_value(ob);
	
  switch (arg)
  {
    case 0:
      sel_robot->angle_range = val_bar;
      break;
    case 1:
      if(fl_get_button(RBTN_ROBOT))
			{
				if (val_bar > sel_robot->min_pos_range)
				{
					sel_robot->max_pos_range = val_bar;
					sel_robot->o[0]->max_pos_range =  val_bar;
				}
			}
      else
				if (val_bar > sel_object->min_pos_range)
					sel_object->max_pos_range = val_bar;	
      break;
    case 2:
      if(fl_get_button(RBTN_ROBOT))
			{
				if (val_bar < sel_robot->max_pos_range)
				{
					sel_robot->min_pos_range = val_bar;
					sel_robot->o[0]->min_pos_range = val_bar;	  
				}
				
			}
      else
				if (val_bar < sel_object->max_pos_range)
					sel_object->min_pos_range = val_bar;
      break; 
    case 3:
      sel_robot->cam_h_angle = val_bar;
      break;
    case 4:
      sel_robot->cam_v_angle = val_bar;
      break;      
    case 5:
			PSP_MA_SEGMENTS  = val_bar;
			break;
    case 6:
			PSP_MA_LAYERS    = val_bar;
			break;       
    case 7:
      val_bar = fl_get_dial_value(ob);
      sprintf(oblabel,"Perception \%:\n %f",val_bar);
      //printf("%s \n", oblabel); 
      fl_set_object_label(ob,oblabel);
      PSP_PS_TRSHLD = val_bar;
      break;
    case 8:
      zchoice = fl_get_choice(ob);
      PSP_SRCH_MTD[PSP_SRCHM_METHOD] = zchoice-1;
      break;
    case 9:
      zchoice = fl_get_choice(ob);
      PSP_SRCH_MTD[PSP_SRCHM_TYPE]   = zchoice-1;
      break;    
    case 10:
      zchoice = fl_get_choice(ob);
      PSP_SRCH_MTD[PSP_SRCHM_GOAL]   = zchoice-1;
      break;  
    case 11:
			PSP_DIST2OBJ_TRSHLD  = val_bar;
			break;
    case 12:
			PSP_DIFF_PS_TRSHLD  = val_bar;
			break;
    case 13:
      zchoice = fl_get_choice(ob);
      PSP_NEXT_TASK = zchoice-1;
      printf ("%i\n",PSP_NEXT_TASK);
      break;     
  }
	
  g3d_draw_allwin_active();
  g3d_refresh_allwin_active();
}

static void CB_task_mode (FL_OBJECT *ob, long arg)
{
	
  switch (arg)
  {
    case 0:
      PSP_NEXT_TASK = PSP_NO_TASK;
      break;
    case 1: 
      PSP_NEXT_TASK = PSP_GIVE_TASK;
      break;   
    case 2:
      PSP_NEXT_TASK = PSP_PICK_TASK;
      break;   
  }
	
}

static void g3d_create_model_area_bars(void)
{  
  FL_OBJECT *obj;
  FL_OBJECT *frame2,*frame3;
  obj = fl_add_frame(FL_ENGRAVED_FRAME,30,210,280,90,""); 
  obj = fl_add_box(FL_FLAT_BOX,45,205,60,10,"Modeling Area"); 
  fl_set_object_lsize(obj, FL_TINY_SIZE);
  fl_set_object_lstyle(obj, FL_BOLD_STYLE);
	
	
  double distoriY=220.0;
  BR_PSP_MA_ANGLE = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY,245.0,10.0,"Angle");
  fl_set_slider_step(BR_PSP_MA_ANGLE,0.05); 
  fl_set_slider_bounds(BR_PSP_MA_ANGLE,0,M_PI); 
  fl_set_object_callback(BR_PSP_MA_ANGLE,CB_update_model_area_bars,0); 
  
  BR_PSP_MA_MAX_DIST = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY+25,245.0,10.0,"Max Distance");
  fl_set_slider_step( BR_PSP_MA_MAX_DIST,0.05); 
  fl_set_slider_bounds(BR_PSP_MA_MAX_DIST,0,7); 
  fl_set_object_callback(BR_PSP_MA_MAX_DIST,CB_update_model_area_bars,1); 
	
  BR_PSP_MA_MIN_DIST = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY+50,245.0,10.0,"Min Distance");
  fl_set_slider_step( BR_PSP_MA_MIN_DIST,0.05); 
  fl_set_slider_bounds(BR_PSP_MA_MIN_DIST,0,7); 
  fl_set_object_callback(BR_PSP_MA_MIN_DIST,CB_update_model_area_bars,2); 
	
	
	
  //for the searching method
  frame2 = fl_add_frame(FL_ENGRAVED_FRAME,220,310,140,185,""); 
  frame2 = fl_add_box(FL_FLAT_BOX,245,305,60,10,"Search Method"); 
  fl_set_object_lsize(frame2, FL_TINY_SIZE);
  fl_set_object_lstyle(frame2, FL_BOLD_STYLE);
	
  BR_PSP_MA_SEGMENTS = fl_add_valslider(FL_HOR_SLIDER,230.0,distoriY+100,125.0,10.0,"Segments");
  fl_set_slider_step( BR_PSP_MA_SEGMENTS,1); 
  fl_set_slider_value( BR_PSP_MA_SEGMENTS,11); 
  fl_set_slider_bounds(BR_PSP_MA_SEGMENTS,1,100); 
  fl_set_object_callback(BR_PSP_MA_SEGMENTS,CB_update_model_area_bars,5);
	
  BR_PSP_MA_LAYERS = fl_add_valslider(FL_HOR_SLIDER,230.0,distoriY+125,125.0,10.0,"Layers");
  fl_set_slider_step( BR_PSP_MA_LAYERS,1); 
  fl_set_slider_value( BR_PSP_MA_LAYERS,4); 
  fl_set_slider_bounds(BR_PSP_MA_LAYERS,1,50); 
  fl_set_object_callback(BR_PSP_MA_LAYERS,CB_update_model_area_bars,6);
	
  CH_SRCH_MTD  = fl_add_choice(FL_NORMAL_CHOICE,280.0,distoriY+210,70.0,20.0,"Method:");
  CH_SRCH_TYPE = fl_add_choice(FL_NORMAL_CHOICE,280.0,distoriY+230,70.0,20.0,"Type:");
  CH_SRCH_GOAL = fl_add_choice(FL_NORMAL_CHOICE,280.0,distoriY+250,70.0,20.0,"Stop:");
	
  /// for thresholds
	
  frame3 = fl_add_frame(FL_ENGRAVED_FRAME,30,distoriY+170,180,120,""); 
  frame3 = fl_add_box(FL_FLAT_BOX,35,distoriY+165,60,10,"Task Thresholds"); 
  fl_set_object_lsize(frame3, FL_TINY_SIZE);
  fl_set_object_lstyle(frame3, FL_BOLD_STYLE);
	
  DL_PSP_SRCH_TRSHLD = fl_add_dial(FL_NORMAL_DIAL,240,distoriY+150,30,30,"Perception\nThreshold");
  fl_set_dial_step(DL_PSP_SRCH_TRSHLD,0.3);
  fl_set_dial_bounds(DL_PSP_SRCH_TRSHLD,0,100);
  fl_set_dial_value(DL_PSP_SRCH_TRSHLD,80.0);
  fl_set_object_callback(DL_PSP_SRCH_TRSHLD,CB_update_model_area_bars,7);
  fl_set_object_lsize(DL_PSP_SRCH_TRSHLD, FL_TINY_SIZE);
	
	
  BR_PSP_DIST2OBJ_TRSHLD = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY+180,145.0,10.0,"Dist. to obj.");
  fl_set_slider_step(BR_PSP_DIST2OBJ_TRSHLD,0.05); 
  fl_set_slider_bounds(BR_PSP_DIST2OBJ_TRSHLD,0,1.5); 
  fl_set_object_callback(BR_PSP_DIST2OBJ_TRSHLD,CB_update_model_area_bars,11); 
  
  BR_PSP_DIFF_PS_TRSHLD = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY+205,145.0,10.0,"Diff. %  Perc.");
  fl_set_slider_step(BR_PSP_DIFF_PS_TRSHLD,0.5); 
  fl_set_slider_bounds(BR_PSP_DIFF_PS_TRSHLD,0,100);   
  fl_set_object_callback(BR_PSP_DIFF_PS_TRSHLD,CB_update_model_area_bars,12); 
	
	
  CH_TASK  = fl_add_choice(FL_NORMAL_CHOICE,280.0,distoriY+150,70.0,20.0,"Task:");
	
  fill_methods();
  
  fl_set_choice(CH_TASK,1);
  fl_set_object_lsize(CH_TASK, FL_TINY_SIZE);
  fl_set_choice_fontsize(CH_TASK, FL_TINY_SIZE);
	
	
  fl_set_choice(CH_SRCH_MTD,1);
  fl_set_object_lsize(CH_SRCH_MTD, FL_TINY_SIZE);
  fl_set_choice_fontsize(CH_SRCH_MTD, FL_TINY_SIZE);
	
  fl_set_choice(CH_SRCH_TYPE,1);
  fl_set_object_lsize(CH_SRCH_TYPE, FL_TINY_SIZE);
  fl_set_choice_fontsize(CH_SRCH_TYPE, FL_TINY_SIZE);
	
  fl_set_choice(CH_SRCH_GOAL,2);
  fl_set_object_lsize(CH_SRCH_GOAL, FL_TINY_SIZE);
  fl_set_choice_fontsize(CH_SRCH_GOAL, FL_TINY_SIZE);
	
  fl_set_object_callback(CH_SRCH_MTD,CB_update_model_area_bars,8);
  fl_set_object_callback(CH_SRCH_TYPE,CB_update_model_area_bars,9);
  fl_set_object_callback(CH_SRCH_GOAL,CB_update_model_area_bars,10);
  fl_set_object_callback(CH_TASK,CB_update_model_area_bars,13);
	
	
	/* PST_PSP_POS = fl_add_positioner(FL_NORMAL_POSITIONER,40, distoriY+180, 100,100,"robot pos");
	 fl_set_positioner_xvalue( PST_PSP_POS, 0);
	 fl_set_positioner_xbounds( PST_PSP_POS, -20,20);
	 fl_set_positioner_yvalue( PST_PSP_POS, 0);
	 fl_set_positioner_ybounds( PST_PSP_POS, -20,20);
	 */
	
}


static void g3d_create_cone_bars(void)
{  
  FL_OBJECT *obj2;
  
  obj2 = fl_add_frame(FL_ENGRAVED_FRAME,30,310,180,70,""); 
  obj2 = fl_add_box(FL_FLAT_BOX,45,305,60,10,"View Field"); 
  fl_set_object_lsize(obj2, FL_TINY_SIZE);
  fl_set_object_lstyle(obj2, FL_BOLD_STYLE);
  
  double distoriY=325.0;
  BR_PSP_VF_HANGLE = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY,145.0,10.0,"Horizontal Angle");
  fl_set_slider_step(BR_PSP_VF_HANGLE,0.05); 
  fl_set_slider_bounds(BR_PSP_VF_HANGLE,0,M_PI); 
  fl_set_object_callback(BR_PSP_VF_HANGLE,CB_update_model_area_bars,3); 
  
  BR_PSP_VF_VANGLE = fl_add_valslider(FL_HOR_SLIDER,40.0,distoriY+25,145.0,10.0,"Vertical Angle");
  fl_set_slider_step(BR_PSP_VF_VANGLE,0.05); 
  fl_set_slider_bounds(BR_PSP_VF_VANGLE,0,M_PI);   
  fl_set_object_callback(BR_PSP_VF_VANGLE,CB_update_model_area_bars,4); 
}
/***************************************/
static void CB_btns_obj(FL_OBJECT *ob, long arg)
{
  p3d_vector4 jointcenter;
  p3d_matrix4 newpos={{0}};
  configPt q1;
  //printf("printing %i\n",arg);
  switch(arg)
	{
    case 0:
      newpos[0][0]=1.;
      newpos[1][1]=1.;
      newpos[2][2]=1.;
      newpos[3][3]=1.;
      
      newpos[0][3]=-1.;
      newpos[1][3]=-1.;
      newpos[2][3]=0.;
      
      fl_set_button(ob,0);
      //p3d_init_robot_parameters();
      //p3d_set_rob_cam_parameters(PSP_ROBOT,-0.20,.0,-0.10,3.0,7.0,0.7,1.05,9,0,.5,.0);//jido arm
      //p3d_set_rob_cam_parameters(PSP_ROBOT,.05,-.10,.0,3.0,7.0,1.10,1.35,16,2,.0,.05);//hrp2
      //psp_chng_show_autohide_st();
      //psp_chng_show_sphere_st();
      //PSP_NUM_OBJECTS = 0;
      //printf("Sphere Active\n");
      //p3d_set_obst_pos("searchball", -1,-1,0,0,0,0);
      //p3d_set_obst_pos_by_mat("searchball", newpos);
      //if(psp_select_target_to_view_by_name("COFFEETABLE"))
      //psp_srch_for_target_obj(PSP_ROBOT, PSP_MA_SEGMENTS, PSP_MA_LAYERS,PSP_SRCH_MTD[PSP_SRCHM_METHOD]+1,&PSP_SRCH_MTD,PSP_PS_TRSHLD,BTSET);
      //psp_deselect_all();
      //psp_update_objects();
      //g3d_draw_allwin_active();
      PSP_NUM_OBJECTS=0;
			
      break;
    case 1:
      //psr_get_obj_list();
      //psr_get_human_left_pointing(tHuman,tRobot);
      // psr_get_human_pointing_from_joint_number(tHuman,tRobot,56);
      //psr_get_pointing_from_joint(tRobot,tHuman->joints[56],0);
      // psp_srch_3D_model_pt_obj(tRobot,sel_object,20,0,BTSET);
      //psp_goto_look_obj(tRobot,sel_object,40,20,1,BTSET);
      //p3d_select_robot_to_view(tHuman);
      //psp_test_actual_robot_pos(tRobot,tHuman,BTSET);
      //p3d_select_robot_to_view(BTSET->human[BTSET->actual_human]->HumanPt);
      //psp_set_device_pos_by_name("CUPBOARDTABLE",3,-3.5, 1, 2);
      //if(psp_select_target_to_view_by_name("TRASHBIN"))
      psp_srch_for_target_obj(PSP_ROBOT, PSP_MA_SEGMENTS, PSP_MA_LAYERS,PSP_SRCH_MTD[PSP_SRCHM_METHOD]+1,PSP_SRCH_MTD,PSP_PS_TRSHLD,BTSET);
      // psp_deselect_all();
      //psr_get_joint_attention(BTSET,PSP_PS_TRSHLD);
      /////////
      //PSP_NUM_OBJECTS =  psu_get_num_objects_near(PSP_ROBOT, 4.0, 0,oListJnt);
      //psr_get_obj_list_multi(PSP_ROBOT, oListJnt, PSP_NUM_OBJECTS, oList, &nObj,PSP_PS_TRSHLD); 
      // fl_set_button(ob,0);
      ///////
     // q1 =  p3d_get_robot_config(PSP_ROBOT);
			
    //  p3d_get_object_center(tHuman->o[tHuman->cam_body_index], jointcenter);
   //   printf(" point to head %f %f %f %f \n", jointcenter[0], jointcenter[1], jointcenter[2], jointcenter[3]);
      //if(psp_set_pan_tilt(PSP_ROBOT,jointcenter,&q1))
   //   printf(" YES  tilt %f pan %f \n", q1[ROBOTq_TILT], q1[ROBOTq_PAN]);
      // else
      //	printf(" NO \n");
      //q1[ROBOTq_TILT] += .2;
     // q1[ROBOTq_PAN]  += 0.2;    
     // p3d_set_and_update_this_robot_conf(PSP_ROBOT,q1);	
			
      
     // free(q1);
      break;
    case 2:
      psp_chng_show_st();
      g3d_draw_allwin_active();
      //if(psp_select_target_to_view_by_name("CUPBOARDTABLE"))
      //	psp_srch_for_target_obj(PSP_ROBOT, PSP_MA_SEGMENTS, PSP_MA_LAYERS,PSP_SRCH_MTD[PSP_SRCHM_METHOD]+1,&PSP_SRCH_MTD,PSP_PS_TRSHLD,BTSET);
      //psp_deselect_all();
      break;
    case 3:
      PSP_ROBOT = sel_robot;
      psp_update_objects();
      g3d_draw_allwin_active();
      fl_set_button(ob,0);
      break; 
	}
}

static void g3d_create_btns(void)
{
	
  BTN_PSP_INIT_VALUES = fl_add_button(FL_PUSH_BUTTON,300,10,50,30,"Init Vals");  
  fl_set_call_back(BTN_PSP_INIT_VALUES,CB_btns_obj,0);
  fl_set_object_lsize(BTN_PSP_INIT_VALUES, FL_TINY_SIZE);
	
  BTN_PSP_GO_FOR_OBJECT = fl_add_button(FL_PUSH_BUTTON,300,40,50,30,"GO");
  fl_set_call_back(BTN_PSP_GO_FOR_OBJECT,CB_btns_obj,1);
	
  BTN_PSP_SHOW_CONFS  = fl_add_button(FL_PUSH_BUTTON,300,70,50,30,"Show Confs");
  fl_set_call_back(BTN_PSP_SHOW_CONFS,CB_btns_obj,2);
  fl_set_object_lsize(BTN_PSP_SHOW_CONFS, FL_TINY_SIZE);
	
  BTN_PSP_SET_ROBOT = fl_add_button(FL_PUSH_BUTTON,60,75,40,20,"Set");
  fl_set_call_back(BTN_PSP_SET_ROBOT,CB_btns_obj,3);
  fl_set_object_lsize(BTN_PSP_SET_ROBOT, FL_TINY_SIZE);
	
}


/********************************************/



static void fill_elements()
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob *currobotPt;
  p3d_obj *objPt;
  int i,isHum,isRob, fstObj=0;
	
  //clear_elements
  fl_clear_choice(ACTUAL_ANIM_ELEMENT);
  tHuman = NULL;
  if (fl_get_button(RBTN_ROBOT))
	{
		for(i=0; i<envPt->nr; i++)
		{
			currobotPt=envPt->robot[i];
			isHum = !strncmp(currobotPt->name,"human",5);
			isRob = strcmp("robot",currobotPt->name);
			//if(isHum || !isRob)
			//  {
			if (i==0)
				sel_robot = currobotPt;
			fl_addto_choice(ACTUAL_ANIM_ELEMENT,currobotPt->name);
			if (isHum && !tHuman)
				tHuman = currobotPt;
			if (!isRob)
				tRobot = currobotPt;
			//   } 
		}
	}
  else
	{
		for(i=0; i<envPt->no; i++)
		{
			objPt=envPt->o[i];
			
			if(!strstr(objPt->name,"obs") && !strstr(objPt->name,"segment"))
				// if(strstr(objPt->name,"furn."))
	    {
	      if (!fstObj)
					sel_object = objPt;
	      fl_addto_choice(ACTUAL_ANIM_ELEMENT,objPt->name);
	      fstObj++;
	    }
		}
		if (fstObj==0)
			printf("No selectable Objetc found\n");
	}
	
	
}



static void CB_select_animated_element_obj(FL_OBJECT *ob, long arg)
{
  int i;
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  p3d_rob *currobotPt;
	if (fl_get_button(RBTN_ROBOT))
	{
		for(i=0; i<envPt->nr; i++)
		{
			currobotPt=envPt->robot[i];
			if(!strcmp(fl_get_choice_text(ob),currobotPt->name))
				sel_robot = currobotPt;
		}
		//sel_robot = p3d_get_robot_by_name(fl_get_choice_text(ob));
		//psp_update_objects();
		fill_bodies();
	}
	else
	{
		
		sel_object = p3d_get_obst_by_name((char *)fl_get_choice_text(ob));
		
	}  
	psp_update_objects();
	g3d_draw_allwin_active();
	// psp_update_objects(); 
}

static void g3d_create_select_animated_element_obj(void)
{
	
  ACTUAL_ANIM_ELEMENT = fl_add_choice(FL_NORMAL_CHOICE,10.0,55.,70.0,20.0,"");
	
  fill_elements();
	
  fl_set_choice(ACTUAL_ANIM_ELEMENT,1);
  fl_set_call_back(ACTUAL_ANIM_ELEMENT,CB_select_animated_element_obj,0);
  fl_set_object_lsize(ACTUAL_ANIM_ELEMENT, FL_TINY_SIZE);
  fl_set_choice_fontsize(ACTUAL_ANIM_ELEMENT, FL_TINY_SIZE);
}

/****************************************/

static void CB_shows_obj(FL_OBJECT *ob, long arg)
{
  switch (arg)
	{
    case 0:
      if (fl_get_button(RBTN_ROBOT))
			{ 
				p3d_set_visible_robot_pos_area(sel_robot, fl_get_button(ob));
			}
      else
			{
				sel_object->show_pos_area = fl_get_button(ob);
			}
      break;
    case 1:
			p3d_set_visible_robot_view_field(sel_robot, fl_get_button(ob));
      break;
    case 2:
      if (fl_get_button(RBTN_ROBOT)) 
			{
				if (fl_get_button(ob))
					p3d_select_robot_to_view(sel_robot);
				else
					p3d_deselect_robot_to_view(sel_robot);
			}
      else
			{
				sel_object->caption_selected = fl_get_button(ob);
				if (sel_object->caption_selected)
					PSP_NUM_OBJECTS++;
				else
					if(PSP_NUM_OBJECTS>0)
						PSP_NUM_OBJECTS--;
				printf("num objects %i\n",PSP_NUM_OBJECTS);
			}
      break;
	}
	
	g3d_draw_allwin_active();
	
}

static void g3d_create_shows_obj(void)
{
  CHKBTN_OBJECTIF = fl_add_checkbutton(FL_PUSH_BUTTON,10,95,20,20,"Objectif");
  fl_set_call_back(CHKBTN_OBJECTIF, CB_shows_obj, 2);
	
  CHKBTN_SHOW_MODEL_AREA = fl_add_checkbutton(FL_PUSH_BUTTON,10,110,20,20,"Model Area");
  fl_set_call_back(CHKBTN_SHOW_MODEL_AREA, CB_shows_obj, 0);
	
  CHKBTN_SHOW_CONE = fl_add_checkbutton(FL_PUSH_BUTTON,10,125,20,20,"Cone");
  fl_set_call_back(CHKBTN_SHOW_CONE, CB_shows_obj, 1);
}
/****************************************/

// *RBTN_OBJECTIF;
// RBTN_WITH_CAMERA;
static void CB_element_choice_obj(FL_OBJECT *ob, long arg)
{
	
  fill_elements();
  if (arg==0)
	{
		printf("Robot Elements Activated\n");
		fl_activate_object(BR_PSP_MA_ANGLE);
		fl_activate_object(CHKBTN_SHOW_CONE);
		fl_activate_object(BR_PSP_VF_HANGLE);
		fl_activate_object(BR_PSP_VF_VANGLE);
		fl_activate_object(BR_CAM_X);
		fl_activate_object(BR_CAM_Y);
		fl_activate_object(BR_CAM_Z);
		fl_activate_object(CH_CAM_BODY);
	}
  else
	{
		fl_deactivate_object(BR_PSP_MA_ANGLE);
		fl_deactivate_object(CHKBTN_SHOW_CONE);
		fl_deactivate_object(BR_PSP_VF_HANGLE);
		fl_deactivate_object(BR_PSP_VF_VANGLE);
		fl_deactivate_object(BR_CAM_X);
		fl_deactivate_object(BR_CAM_Y);
		fl_deactivate_object(BR_CAM_Z);
		fl_deactivate_object(CH_CAM_BODY);
		
		printf("Objects Activated\n");
		
	}
  psp_update_objects();
	
}


static void g3d_create_element_choice_obj(void)
{
  ELEMENTGROUP = fl_bgn_group();
  RBTN_ROBOT = fl_add_checkbutton(FL_RADIO_BUTTON,10,10,20,20,"ROBOTS");
  fl_set_object_color(RBTN_ROBOT,FL_MCOL,FL_BLUE);
  fl_set_call_back(RBTN_ROBOT,CB_element_choice_obj,0);
  RBTN_OBJECT = fl_add_checkbutton(FL_RADIO_BUTTON,10,30,20,20,"OBJECTS");
  fl_set_object_color(RBTN_OBJECT,FL_MCOL,FL_BLUE);
  fl_set_call_back(RBTN_OBJECT,CB_element_choice_obj,1);
  ELEMENTGROUP = fl_end_group();
  fl_set_button(RBTN_ROBOT,1);
} 

/*********************************************/

static void CB_update_cam_bars(FL_OBJECT *ob, long arg)
{
	
  double val_bar = fl_get_slider_value(ob);
  if (arg<3)
    sel_robot->cam_pos[arg] = val_bar;
  else
	{
		if (arg==3)
			sel_robot->cam_pan  = val_bar;
		else
			sel_robot->cam_tilt = val_bar;
		
	}
  p3d_update_rob_cam_parameters(sel_robot);
  g3d_draw_allwin_active();
  g3d_refresh_allwin_active();
	
	
}


static void g3d_create_camera_bars(void)
{  
  FL_OBJECT *objcam;
  double distoriY=20.0;
  double distoriX=110.0;
  int ecart=25;
  int barpos= 105;
	
  objcam = fl_add_frame(FL_ENGRAVED_FRAME,distoriX-5,distoriY-5,180,185,""); 
  objcam = fl_add_box(FL_FLAT_BOX,distoriX,distoriY-7,60,10,"Camera"); 
  fl_set_object_lsize(objcam, FL_TINY_SIZE);
  fl_set_object_lstyle(objcam, FL_BOLD_STYLE);  
	
  BR_CAM_PAN = fl_add_valslider(FL_HOR_SLIDER,distoriX,distoriY+barpos-(ecart*2),170.0,10.0,"Pan");
  fl_set_slider_step(BR_CAM_PAN,0.05); 
  fl_set_slider_bounds(BR_CAM_PAN,-M_PI,M_PI); 
  fl_set_object_callback(BR_CAM_PAN,CB_update_cam_bars,3); 
	
  BR_CAM_TILT = fl_add_valslider(FL_HOR_SLIDER,distoriX,distoriY+barpos-ecart,170.0,10.0,"Tilt");
  fl_set_slider_step(BR_CAM_TILT,0.05); 
  fl_set_slider_bounds(BR_CAM_TILT,-M_PI,M_PI); 
  fl_set_object_callback(BR_CAM_TILT,CB_update_cam_bars,4); 
	
  BR_CAM_X = fl_add_valslider(FL_HOR_SLIDER,distoriX,distoriY+barpos,170.0,10.0,"CamX");
  fl_set_slider_step(BR_CAM_X,0.05); 
  fl_set_slider_bounds(BR_CAM_X,-0.5,0.5); 
  fl_set_object_callback(BR_CAM_X,CB_update_cam_bars,0); 
  
  BR_CAM_Y = fl_add_valslider(FL_HOR_SLIDER,distoriX,distoriY+barpos+ecart,170.0,10.0,"CamY");
  fl_set_slider_step(BR_CAM_Y,0.05); 
  fl_set_slider_bounds(BR_CAM_Y,-0.5,0.5); 
  fl_set_object_callback(BR_CAM_Y,CB_update_cam_bars,1); 
	
  BR_CAM_Z = fl_add_valslider(FL_HOR_SLIDER,distoriX,distoriY+barpos+(ecart*2),170.0,10.0,"CamZ");
  fl_set_slider_step(BR_CAM_Z,0.05); 
  fl_set_slider_bounds(BR_CAM_Z,-0.5,0.5); 
  fl_set_object_callback(BR_CAM_Z,CB_update_cam_bars,2); 
	
  g3d_draw_allwin_active();
}

/*********************************************/

static void fill_bodies(void)
{
  p3d_obj *objPt;
  int i, selected = -1;
	
  fl_clear_choice(CH_CAM_BODY);
  for(i=0; i<sel_robot->no; i++) {
    objPt=sel_robot->o[i];
    if (objPt==NULL)
      printf("No Object for %i\n",i);
    else
		{
			fl_addto_choice(CH_CAM_BODY,objPt->name);
			if (p3d_get_obj_select_status(objPt))
				selected = i;
		}
  }
  if (selected==-1)
    selected=0;
	
  sel_object = sel_robot->o[sel_robot->cam_body_index];
  fl_set_choice(CH_CAM_BODY,sel_robot->cam_body_index+1);
	
  fl_set_button(CHKBTN_BODY_OBJECTIF, p3d_get_obj_select_status(sel_object));
	
	// psp_update_objects();
	// g3d_draw_allwin_active();
}


static void CB_cam_ref(FL_OBJECT *ob, long arg)
{
  sel_robot->cam_axe = arg;
  p3d_update_rob_cam_parameters(sel_robot);
  g3d_refresh_allwin_active();
}


static void CB_select_body_obj(FL_OBJECT *ob, long arg)
{
  if (arg==0)
	{
		set_robot_camera_body(sel_robot,fl_get_choice(ob)-1);
		sel_object = p3d_get_body_from_robot(sel_robot, fl_get_choice_text(ob));
		fl_set_button(CHKBTN_BODY_OBJECTIF, p3d_get_obj_select_status(sel_object));
	}
  else
	{
		p3d_set_body_selection(sel_robot,fl_get_choice(CH_CAM_BODY)-1,fl_get_button(ob));
		if(fl_get_button(ob))
			PSP_NUM_OBJECTS++;
		else
			if(PSP_NUM_OBJECTS>0)
				PSP_NUM_OBJECTS--;
		printf("num objects %i\n",PSP_NUM_OBJECTS);
		
	}
	
  g3d_draw_allwin_active();
}

static void g3d_create_cam_objs(void)
{
	
  CH_CAM_BODY = fl_add_choice(FL_NORMAL_CHOICE,140.0,25.,100.0,20.0,"Body");
  CHKBTN_BODY_OBJECTIF =  fl_add_checkbutton(FL_PUSH_BUTTON,240,25,20,20,"Sel.");
  
	
  GROUP_CAM = fl_bgn_group();
	
  RBTN_CAM_X =  fl_add_checkbutton(FL_RADIO_BUTTON,120,50,20,20,"X");
	fl_set_object_color(RBTN_CAM_X,FL_MCOL,FL_BLUE);
	fl_set_call_back(RBTN_CAM_X,CB_cam_ref,0);
	fl_set_object_lsize(RBTN_CAM_X, FL_TINY_SIZE);
	
  RBTN_CAM_Y =  fl_add_checkbutton(FL_RADIO_BUTTON,170,50,20,20,"Y");
	fl_set_object_color(RBTN_CAM_Y,FL_MCOL,FL_BLUE);
	fl_set_call_back(RBTN_CAM_Y,CB_cam_ref,1);
	fl_set_object_lsize(RBTN_CAM_Y, FL_TINY_SIZE);
	
  RBTN_CAM_Z =  fl_add_checkbutton(FL_RADIO_BUTTON,220,50,20,20,"Z");
	fl_set_object_color(RBTN_CAM_Z,FL_MCOL,FL_BLUE);
	fl_set_call_back(RBTN_CAM_Z,CB_cam_ref,2);
	fl_set_object_lsize(RBTN_CAM_Z, FL_TINY_SIZE);
	
	GROUP_CAM = fl_end_group();
	
	fl_set_button(RBTN_CAM_X, 1);
	
  fill_bodies();
	//psp_update_objects();
	//g3d_draw_allwin_active();
	
  fl_set_choice(CH_CAM_BODY,1);
  fl_set_call_back(CH_CAM_BODY,CB_select_body_obj,0);
  fl_set_object_lsize(CH_CAM_BODY, FL_TINY_SIZE);
  fl_set_choice_fontsize(CH_CAM_BODY, FL_TINY_SIZE);
	
  fl_set_call_back(CHKBTN_BODY_OBJECTIF, CB_select_body_obj, 1);  
  fl_set_object_lsize(CHKBTN_BODY_OBJECTIF, FL_TINY_SIZE);
}

/*********************************************/


static void CB_win_mode(FL_OBJECT *ob, long arg)
{
	G3D_Window *persp_win=g3d_get_win_by_name("Perspective");
	
	if (persp_win==NULL)
		persp_win=g3d_get_win_by_name("Move3D");
	
	
	 g3d_set_win_draw_mode(persp_win, (g3d_window_draw_mode)arg);
	
	g3d_draw_allwin_active();
}

static void g3d_create_win_mode_objs(void)
{
  ELEMENTGROUP2 = fl_bgn_group();
  RBTN_WIN_MODE_0 =  fl_add_checkbutton(FL_RADIO_BUTTON,280,100,20,20,"Normal");
	fl_set_object_color(RBTN_WIN_MODE_0,FL_MCOL,FL_BLUE);
	fl_set_call_back(RBTN_WIN_MODE_0,CB_win_mode,0);
  RBTN_WIN_MODE_1 =  fl_add_checkbutton(FL_RADIO_BUTTON,280,115,20,20,"Objectif");
  fl_set_object_color(RBTN_WIN_MODE_1,FL_MCOL,FL_BLUE);
  fl_set_call_back(RBTN_WIN_MODE_1,CB_win_mode,1);
  RBTN_WIN_MODE_2 =  fl_add_checkbutton(FL_RADIO_BUTTON,280,130,20,20,"Difference");
  fl_set_object_color(RBTN_WIN_MODE_2,FL_MCOL,FL_BLUE);
  fl_set_call_back(RBTN_WIN_MODE_2,CB_win_mode,2);
  ELEMENTGROUP2 = fl_end_group();
	fl_set_button(RBTN_WIN_MODE_0, 1);
}

/*******************DELETES**********************/

static void g3d_delete_shows_obj(void)
{
  fl_free_object(CHKBTN_SHOW_MODEL_AREA); 
  fl_free_object(CHKBTN_SHOW_CONE); 
  fl_free_object(ACTUAL_ANIM_ELEMENT); 
	
}
static void g3d_delete_bars_obj(void)
{
  
  fl_free_object(BR_PSP_MA_ANGLE);
  fl_free_object(BR_PSP_MA_MAX_DIST);
  fl_free_object(BR_PSP_MA_MIN_DIST);
  fl_free_object(BR_PSP_VF_HANGLE);
  fl_free_object(BR_PSP_VF_VANGLE);
  fl_free_object(BR_CAM_X);
  fl_free_object(BR_CAM_Y);
  fl_free_object(BR_CAM_Z);
  fl_free_object(BR_CAM_PAN);
  fl_free_object(BR_CAM_TILT);
  fl_free_object(CH_CAM_BODY);
}

void g3d_delete_psp_parameters_form(void)
{
  g3d_delete_shows_obj();
  g3d_delete_bars_obj();
  fl_free_form(PSP_PARAMETERS_FORM);
}


p3d_obj* p3d_get_body_from_robot(p3d_rob *robotPt, const char* name)
{
  int i;
	
	
	for(i=0;i<robotPt->no;i++)
	{
		if(strcmp(name,robotPt->o[i]->name) == 0)
			return(robotPt->o[i]);
	}
	
	return NULL;
}

