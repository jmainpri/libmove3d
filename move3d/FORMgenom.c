#include "Manipulation.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "gbM/Proto_gbModeles.h"
#include "lightPlanner.h"
#include "lightPlannerApi.h"
#include <gp_grasp_generation_proto.h>
#include <gp_grasp_generation_proto.h>
#ifdef DPG
#include "p3d_chanEnv_proto.h"
#endif
#include "planner_cxx/plannerFunctions.hpp"
#include <list>
#include <string>
#include <iostream>


// #if defined(MULTILOCALPATH) && defined(GRASP_PLANNING) && defined(LIGHT_PLANNER)

//#define OBJECT_NAME "DUPLO_OBJECT"
//#define OBJECT_NAME "WOODEN_OBJECT"
#define OBJECT_NAME "GREY_TAPE"
//#define OBJECT_NAME "YELLOW_BOTTLE"
#define SUPPORT_NAME "HRP2TABLE"
#define HUMAN_NAME "ACHILE_HUMAN1"
#define CAMERA_JNT_NAME "Tilt"
#define CAMERA_FOV 80.0

static Manipulation_JIDO *manipulation= NULL;

static double QCUR[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double QGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double XCUR[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double XGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


static int FORMGENOM_CARTESIAN = 0;
static int FORMGENOM_OBJECTGRABED = 0;
/* --------- FORM VARIABLES ------- */
FL_FORM  * GENOM_FORM = NULL;
static FL_OBJECT * GENOMGROUP = NULL;


static FL_OBJECT * BT_SET_Q_OBJ = NULL;
static FL_OBJECT * BT_SET_X_OBJ = NULL;



static FL_OBJECT * BT_ARM_GOTO_Q_OBJ = NULL;
static FL_OBJECT * BT_SIMPLE_GRASP_PLANNER_OBJ = NULL;
static FL_OBJECT * BT_FIND_GRASP_CONFIG_AND_COMP_TRAJ_OBJ = NULL;
static FL_OBJECT * BT_COMP_TRAJ_CONFIGS_OBJ = NULL;
static FL_OBJECT  *INPUT_OBJ1, *INPUT_OBJ2 = NULL;
static FL_FORM *INPUT_FORM = NULL;
static FL_OBJECT *DELETE_GRAPHS = NULL;
static FL_OBJECT * BT_SET_CARTESIAN = NULL;
static FL_OBJECT * BT_GRAB_OBJECT = NULL;
static FL_OBJECT * BT_RELEASE_OBJECT = NULL;

static FL_OBJECT * BT_PICK_UP_TAKE = NULL;
static FL_OBJECT * BT_PLACE = NULL;
static FL_OBJECT * BT_CONSTRUCTPRM = NULL;
#ifdef DPG
static FL_OBJECT * BT_CHECKCOLONTRAJ = NULL;
static FL_OBJECT * BT_REPLANCOLTRAJ = NULL;
#endif
/* ---------- FUNCTION DECLARATIONS --------- */
static void initManipulationGenom();
static void g3d_create_genom_group(void);
static void genomDraw();
static void genomKey();

static void CB_genomSetQ_obj(FL_OBJECT *obj, long arg);
static void CB_genomSetX_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmGotoQ_obj(FL_OBJECT *obj, long arg);
static void CB_genomFindSimpleGraspConfiguration_obj(FL_OBJECT *obj, long arg);

static void CB_genomPickUp_gotoObject(FL_OBJECT *obj, long arg);



static void CB_genomCleanRoadmap_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmComputePRM_obj(FL_OBJECT *obj, long arg);
static void CB_set_cartesian(FL_OBJECT *obj, long arg);
static void CB_grab_object(FL_OBJECT *obj, long arg);
static void CB_release_object(FL_OBJECT *obj, long arg);

static void CB_genomPickUp_takeObject(FL_OBJECT *obj, long arg);
static void CB_genomPickUp_placeObject(FL_OBJECT *obj, long arg);
static void CB_genomPlaceObject(FL_OBJECT *obj, long arg);
#ifdef DPG
static void CB_checkColOnTraj(FL_OBJECT *obj, long arg);
static void CB_replanColTraj(FL_OBJECT *obj, long arg);
#endif
/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_genom_form(void) {
	GENOM_FORM = fl_bgn_form(FL_UP_BOX, 150, 750);
	g3d_create_genom_group();
	fl_end_form();
}

void g3d_show_genom_form(void) {
	fl_show_form(GENOM_FORM, FL_PLACE_SIZE, TRUE, "Genom Requests");
}

void g3d_hide_genom_form(void) {
	fl_hide_form(GENOM_FORM);
}

void g3d_delete_genom_form(void) {
	fl_free_form(GENOM_FORM);
}

/* -------------------- MAIN GROUP --------------------- */
static void initManipulationGenom() {
  if (manipulation == NULL) {
	p3d_rob * robotPt= p3d_get_robot_by_name(GP_ROBOT_NAME);
	manipulation= new Manipulation_JIDO(robotPt, GP_GRIPPER);
  }
  return;
}

static void g3d_create_genom_group(void) {
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 720, "Genom Requests");

	GENOMGROUP = fl_bgn_group();

	x= 15;
	y= 30;
	w= 120;
	h= 40;
	dy= h + 10;

	BT_SET_Q_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Set arm Q");
	fl_set_call_back(BT_SET_Q_OBJ, CB_genomSetQ_obj, 1);

        y+= dy;
	BT_SET_X_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Set arm X");
	fl_set_call_back(BT_SET_X_OBJ, CB_genomSetX_obj, 1);

	 y+= dy;
         BT_SET_CARTESIAN = fl_add_button(FL_RADIO_BUTTON, x, y, w, h, "Set Cartesian");
	fl_set_call_back(BT_SET_CARTESIAN, CB_set_cartesian, 1);
	fl_set_object_color(BT_SET_CARTESIAN,FL_MCOL,FL_GREEN);
	fl_set_button(BT_SET_CARTESIAN, FALSE);


	 y+= dy;
         BT_GRAB_OBJECT = fl_add_button(FL_RADIO_BUTTON, x, y, w, h, "Grab Object");
	fl_set_call_back(BT_GRAB_OBJECT, CB_grab_object, 1);
	fl_set_object_color(BT_GRAB_OBJECT,FL_MCOL,FL_GREEN);
	fl_set_button(BT_GRAB_OBJECT, FALSE);

		 y+= dy;
         BT_RELEASE_OBJECT = fl_add_button(FL_RADIO_BUTTON, x, y, w, h, "Release Object");
	fl_set_call_back(BT_RELEASE_OBJECT, CB_release_object, 1);
	fl_set_object_color(BT_RELEASE_OBJECT,FL_MCOL,FL_GREEN);
	fl_set_button(BT_RELEASE_OBJECT, FALSE);
	
	y+= dy;
	DELETE_GRAPHS =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Delete Graphs");
	fl_set_call_back(DELETE_GRAPHS, CB_genomCleanRoadmap_obj, 1);

        y+= dy;
	BT_ARM_GOTO_Q_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Goto Q");
//	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
	fl_set_call_back(BT_ARM_GOTO_Q_OBJ, CB_genomArmGotoQ_obj, 1);

	y+= dy;
        BT_FIND_GRASP_CONFIG_AND_COMP_TRAJ_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Pick Up (goto)");
        fl_set_call_back(BT_FIND_GRASP_CONFIG_AND_COMP_TRAJ_OBJ, CB_genomPickUp_gotoObject, 1);


	y+= dy;
        BT_PICK_UP_TAKE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Pick Up (take)");
        fl_set_call_back(BT_PICK_UP_TAKE, CB_genomPickUp_takeObject, 1);
	
	y+= dy;
        BT_PLACE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Pick Up (place)");
        fl_set_call_back(BT_PLACE, CB_genomPickUp_placeObject, 1);	

	y+= dy;
        BT_PLACE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Place Object from free");
        fl_set_call_back(BT_PLACE, CB_genomPlaceObject, 1);

// 	y+= dy;
// 	BT_SIMPLE_GRASP_PLANNER_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Simple grasp config");
// 	fl_set_call_back(BT_SIMPLE_GRASP_PLANNER_OBJ, CB_genomFindSimpleGraspConfiguration_obj, 1);

//   y+= dy;
//   BT_COMP_TRAJ_CONFIGS_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Compute traj from configs");
//   fl_set_call_back(BT_COMP_TRAJ_CONFIGS_OBJ, CB_genomComputeTrajFromConfigs_obj, 1);


//   y+= dy;
//   COMPUTE_PRM =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Compute PRM");
//   fl_set_call_back(COMPUTE_PRM, CB_genomArmComputePRM_obj, 1);
//   y+= dy;
//   CHECK_COL_ON_TRAJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Check traj col");
//   fl_set_call_back(CHECK_COL_ON_TRAJ, CB_genomCheckCollisionOnTraj_obj, 1);
  BT_CONSTRUCTPRM =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Build PRM");
  fl_set_call_back(BT_CONSTRUCTPRM, CB_genomArmComputePRM_obj, 0);
#ifdef DPG
  y+= dy;
  BT_CHECKCOLONTRAJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Check col Traj");
  fl_set_call_back(BT_CHECKCOLONTRAJ, CB_checkColOnTraj, 0);
  y+= dy;
  BT_REPLANCOLTRAJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Replan Col Traj");
  fl_set_call_back(BT_REPLANCOLTRAJ, CB_replanColTraj, 0);
  y+= dy;
#endif
  
  fl_end_group();
}

static void CB_read_Q(FL_OBJECT *ob, long arg) {
 int result;
 const char *str;
 p3d_rob *robotPt = NULL;

 


 robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
 FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));

 str= fl_get_input(INPUT_OBJ2);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &QGOAL[0], &QGOAL[1], &QGOAL[2], &QGOAL[3], &QGOAL[4], &QGOAL[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: q1 q2 q3 q4 q5 q6\n", 1);
   printf("Expected format of QGOAL input: q1 q2 q3 q4 q5 q6\n");
   QGOAL[0]= QGOAL[1]= QGOAL[2]= QGOAL[3]= QGOAL[4]= QGOAL[5]= 0.0;
 }
//  result= genomSetArmQ(robotPt, QGOAL[0], QGOAL[1], QGOAL[2], QGOAL[3], QGOAL[4], QGOAL[5]);
 if(result==1) {
         printf("Configuration is not reachable by virtual object:%f %f %f %f %f %f\n", QGOAL[0], QGOAL[1], QGOAL[2], QGOAL[3], QGOAL[4], QGOAL[5]);
 }
 else {
         p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_GOTO);
 }



 str= fl_get_input(INPUT_OBJ1);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &QCUR[0], &QCUR[1], &QCUR[2], &QCUR[3], &QCUR[4], &QCUR[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: q1 q2 q3 q4 q5 q6\n", 1);
   printf("Expected format of Q input: q1 q2 q3 q4 q5 q6\n");
   QCUR[0]= QCUR[1]= QCUR[2]= QCUR[3]= QCUR[4]= QCUR[5]= 0.0;
 }
 printf("%lf %lf %lf %lf %lf %lf \n", QCUR[0], QCUR[1], QCUR[2], QCUR[3], QCUR[4], QCUR[5]);


//  result = genomSetArmQ(robotPt, QCUR[0], QCUR[1], QCUR[2], QCUR[3], QCUR[4], QCUR[5]);
 if(result==1) {
         printf("Configuration is not reachable by virtual object:%f %f %f %f %f %f\n", QCUR[0], QCUR[1], QCUR[2], QCUR[3], QCUR[4], QCUR[5]);
 }
 else {
         p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_POS);
 }


 g3d_draw_allwin_active();

 fl_hide_form(INPUT_FORM);
 fl_free_object(INPUT_OBJ1);
 fl_free_object(INPUT_OBJ2);
 fl_free_form(INPUT_FORM);
}

static void CB_read_X(FL_OBJECT *ob, long arg) {
 int result;
 const char *str;
 p3d_rob *robotPt = NULL;

	if (manipulation== NULL) {
	  initManipulationGenom();
	}
	
 robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
 FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));

 str= fl_get_input(INPUT_OBJ2);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &XGOAL[0], &XGOAL[1], &XGOAL[2], &XGOAL[3], &XGOAL[4], &XGOAL[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: x y z rx ry rz\n", 1);
   printf("Expected format of XGOAL input: x y z rx ry rz\n");
   XGOAL[0]= XGOAL[1]= XGOAL[2]= XGOAL[3]= XGOAL[4]= XGOAL[5]= 0.0;
 }
 manipulation->setArmX(XGOAL[0], XGOAL[1], XGOAL[2], XGOAL[3], XGOAL[4], XGOAL[5]);
 p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_GOTO);


 str= fl_get_input(INPUT_OBJ1);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &XCUR[0], &XCUR[1], &XCUR[2], &XCUR[3], &XCUR[4], &XCUR[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: x y z rx ry rz\n", 1);
   printf("Expected format of X input: x y z rx ry rz6\n");
   XCUR[0]= XCUR[1]= XCUR[2]= XCUR[3]= XCUR[4]= XCUR[5]= 0.0;
 }
 printf("%lf %lf %lf %lf %lf %lf \n", XCUR[0], XCUR[1], XCUR[2], XCUR[3], XCUR[4], XCUR[5]);

 manipulation->setArmX(XCUR[0], XCUR[1], XCUR[2], XCUR[3], XCUR[4], XCUR[5]);
 
 g3d_draw_allwin_active();
 fl_hide_form(INPUT_FORM);
 fl_free_object(INPUT_OBJ1);
 fl_free_object(INPUT_OBJ2);
 fl_free_form(INPUT_FORM);
}

static void CB_cancel(FL_OBJECT *ob, long arg) {
  fl_hide_form(INPUT_FORM);
  fl_free_object(INPUT_OBJ1);
  fl_free_object(INPUT_OBJ2);
  fl_free_form(INPUT_FORM);
}

static void CB_genomSetQ_obj(FL_OBJECT *obj, long arg) {
  int x, y, w, h;
  FL_OBJECT *ok, *cancel;
  configPt q0= NULL;
  p3d_rob *robotPt = NULL;
  char str1[128], str2[128];

  	if (manipulation== NULL) {
	  initManipulationGenom();
	}

  robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  q0= p3d_get_robot_config(robotPt);
  
  manipulation->getArmQ(&QCUR[0], &QCUR[1], &QCUR[2], &QCUR[3], &QCUR[4], &QCUR[5]);
  p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_GOTO);
  manipulation->getArmQ( &QGOAL[0], &QGOAL[1], &QGOAL[2], &QGOAL[3], &QGOAL[4], &QGOAL[5]);
  p3d_set_and_update_this_robot_conf(robotPt, q0);
  p3d_set_ROBOT_START(q0);
  p3d_destroy_config(robotPt, q0);

  sprintf(str1, "%lf     %lf     %lf    %lf     %lf     %lf", QCUR[0], QCUR[1], QCUR[2], QCUR[3], QCUR[4], QCUR[5]);
  sprintf(str2, "%lf     %lf     %lf    %lf     %lf     %lf", QGOAL[0],QGOAL[1],QGOAL[2],QGOAL[3],QGOAL[4],QGOAL[5]);

  x= 110;
  y= 30;
  w= 380;
  h= 40;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX, (int)(1.5*w), 5*h);
  fl_add_text(FL_NORMAL_TEXT,0,0,400,40,"ENTER ARM START AND GOAL CONFIGURATIONS (current values are displayed):");

  INPUT_OBJ1  = fl_add_input(FL_NORMAL_INPUT, x, y, w, h,"Qstart : \n(6 angles in radians)");
  fl_set_input_color(INPUT_OBJ1, 0, 0);
  fl_set_input(INPUT_OBJ1, str1);

  INPUT_OBJ2  = fl_add_input(FL_NORMAL_INPUT, x, y+2*h, w, h,"Qgoal : \n(6 angles in radians)");
  fl_set_input_color(INPUT_OBJ2, 0, 0);
  fl_set_input(INPUT_OBJ2, str2);

  ok = fl_add_button(FL_BUTTON, x, y+3*h+20, 60, 20, "OK");
  fl_set_object_callback(ok, CB_read_Q, 0);
  cancel = fl_add_button(FL_PUSH_BUTTON, x+90, y+3*h+20, 60, 20, "Cancel");
  fl_set_object_callback(cancel, CB_cancel, 0);

  fl_end_form();

  fl_show_form(INPUT_FORM, FL_PLACE_SIZE, TRUE, "");
}

static void CB_genomSetX_obj(FL_OBJECT *obj, long arg) {
  int x, y, w, h;
  FL_OBJECT *ok, *cancel;
  configPt q0= NULL;
  p3d_rob *robotPt = NULL;
  char str1[128], str2[128];

    	if (manipulation== NULL) {
	  initManipulationGenom();
	}


  robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  q0= p3d_get_robot_config(robotPt);
  manipulation->getArmX(&XCUR[0], &XCUR[1], &XCUR[2], &XCUR[3], &XCUR[4], &XCUR[5]);

  p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_GOTO);
  manipulation->getArmX( &XGOAL[0], &XGOAL[1], &XGOAL[2], &XGOAL[3], &XGOAL[4], &XGOAL[5]);
  p3d_set_and_update_this_robot_conf(robotPt, q0);
  p3d_set_ROBOT_START(q0);
  p3d_destroy_config(robotPt, q0);

  sprintf(str1, "%lf     %lf     %lf    %lf     %lf     %lf", XCUR[0], XCUR[1], XCUR[2], XCUR[3], XCUR[4], XCUR[5]);
  sprintf(str2, "%lf     %lf     %lf    %lf     %lf     %lf", XGOAL[0],XGOAL[1],XGOAL[2],XGOAL[3],XGOAL[4],XGOAL[5]);

  x= 110;
  y= 30;
  w= 380;
  h= 40;

  INPUT_FORM = fl_bgn_form(FL_FLAT_BOX, (int)(1.5*w), 5*h);
  fl_add_text(FL_NORMAL_TEXT,0,0,400,40,"ENTER ARM START AND GOAL END EFFECTOR POSES (current values are displayed):");

  INPUT_OBJ1  = fl_add_input(FL_NORMAL_INPUT, x, y, w, h,"Xstart : \n(position \n+ Euler angles \n(in degrees))");
  fl_set_input_color(INPUT_OBJ1, 0, 0);
  fl_set_input(INPUT_OBJ1, str1);

  INPUT_OBJ2  = fl_add_input(FL_NORMAL_INPUT, x, y+2*h, w, h,"Xgoal : \n(position \n+ Euler angles \n(in degrees))");
  fl_set_input_color(INPUT_OBJ2, 0, 0);
  fl_set_input(INPUT_OBJ2, str2);

  ok = fl_add_button(FL_BUTTON, x, y+3*h+20, 60, 20, "OK");
  fl_set_object_callback(ok, CB_read_X, 0);
  cancel = fl_add_button(FL_PUSH_BUTTON, x+90, y+3*h+20, 60, 20, "Cancel");
  fl_set_object_callback(cancel, CB_cancel, 0);

  fl_end_form();

  fl_show_form(INPUT_FORM, FL_PLACE_SIZE, TRUE, "");
}

static void CB_genomArmGotoQ_obj(FL_OBJECT *obj, long arg) {

	int nbPositions = 0;
	
	if (manipulation== NULL) {
	  initManipulationGenom();
	}

	if(FORMGENOM_CARTESIAN == 1) {
	  manipulation->setArmCartesian(true);
	} else {
	  manipulation->setArmCartesian(false);
	}
	manipulation->armPlanTask(ARM_FREE,manipulation->robotStart(),manipulation->robotGoto(),(char*)"", manipulation->lp, manipulation->positions);
	fl_set_button(BT_ARM_GOTO_Q_OBJ,0);
        return;
}




//! Plans a path to go from the currently defined ROBOT_POS config to the specified end effector pose defined for the arm only.
//! End effector pose is given in world coordinates (lenghts in meters, angles in radians).
//! \return 0 in case of success, !=0 otherwise
int genomArmGotoX(p3d_rob* robotPt, int cartesian, double x, double y, double z, double rx, double ry, double rz, std::vector <int> lp, std::vector < std::vector <double> > positions, int *nbPositions)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: genomArmGotoX(): robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
  if (manipulation== NULL) {
	  initManipulationGenom();
  }

  
  manipulation->setArmX( x, y, z, rx, ry, rz);
  p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_GOTO);

  	if(FORMGENOM_CARTESIAN == 1) {
	  manipulation->setArmCartesian(true);
	} else {
	  manipulation->setArmCartesian(false);
	}


  return manipulation->armPlanTask(ARM_FREE,manipulation->robotStart(), manipulation->robotGoto(), (char*)"", manipulation->lp, manipulation->positions);
}

static void CB_genomCleanRoadmap_obj(FL_OBJECT *obj, long arg){
	if (manipulation== NULL) {
	  initManipulationGenom();
	}
       manipulation->cleanRoadmap();
       manipulation->cleanTraj();
}

static void CB_genomArmComputePRM_obj(FL_OBJECT *obj, long arg){
  if (manipulation== NULL) {
	  initManipulationGenom();
  }
  manipulation->armComputePRM();
}

#ifdef DPG
void CB_checkColOnTraj(FL_OBJECT *obj, long arg){
  if (manipulation== NULL) {
	  initManipulationGenom();
	}
  manipulation->checkCollisionOnTraj();
}
void CB_replanColTraj(FL_OBJECT *obj, long arg){
  if (manipulation== NULL) {
	  initManipulationGenom();
	}
//  manipulation->replanCollidingTraj(XYZ_ROBOT, 0, <#double *armConfig#>, <#int currentLpId#>, <#int [] lp#>, <#Gb_q6 [] positions#>, <#int *nbPositions#>);
}
#endif


static void CB_set_cartesian(FL_OBJECT *obj, long arg) {
	FORMGENOM_CARTESIAN= !FORMGENOM_CARTESIAN;

	if(FORMGENOM_CARTESIAN)
	{  fl_set_button(BT_SET_CARTESIAN, TRUE);
fl_set_object_color(BT_SET_CARTESIAN,FL_MCOL,FL_GREEN);
	}
	else
	{  fl_set_button(BT_SET_CARTESIAN, FALSE);
fl_set_object_color(BT_SET_CARTESIAN, FL_INACTIVE_COL,FL_INACTIVE_COL);
	}
	printf("cartesian = %d\n",FORMGENOM_CARTESIAN);
}

static void CB_grab_object(FL_OBJECT *obj, long arg) {
	fl_set_button(BT_GRAB_OBJECT, TRUE);
	fl_set_object_color(BT_GRAB_OBJECT,FL_MCOL,FL_GREEN);
	fl_set_button(BT_RELEASE_OBJECT, FALSE);
        fl_set_object_color(BT_RELEASE_OBJECT, FL_INACTIVE_COL,FL_INACTIVE_COL);

	if (manipulation== NULL) {
	  initManipulationGenom();
	}
	FORMGENOM_OBJECTGRABED = 1;
	if(manipulation->grabObject((char*)OBJECT_NAME)!=0){
	  FORMGENOM_OBJECTGRABED = 0;
	}

	printf("grab = %d\n",FORMGENOM_OBJECTGRABED);
	return;
}

static void CB_release_object(FL_OBJECT *obj, long arg) {
	fl_set_button(BT_GRAB_OBJECT, FALSE);
        fl_set_object_color(BT_GRAB_OBJECT, FL_INACTIVE_COL,FL_INACTIVE_COL);
	fl_set_button(BT_RELEASE_OBJECT, TRUE);
	fl_set_object_color(BT_RELEASE_OBJECT,FL_MCOL,FL_GREEN);
	
        FORMGENOM_OBJECTGRABED = 0;
	if (manipulation== NULL) {
	  initManipulationGenom();
	}
	manipulation->releaseObject();
	printf("grab = %d\n",FORMGENOM_OBJECTGRABED);
	return;
}



//#ifdef DPG
////! \brief Check if the current path is in collision or not
////! \return 1 in case of collision, 0 otherwise
//int genomCheckCollisionOnTraj(p3d_rob* robotPt, int cartesian, int currentLpId) {
//  configPt qi = NULL, qf = NULL;
//  static p3d_traj *traj = NULL;
////   int ntest=0;
////   double gain;
//
//  XYZ_ENV->cur_robot= robotPt;
//  //initialize and get the current linear traj
//  if (!traj){
//    if(robotPt->nt < robotPt->tcur->num - 2){
//      return 1;
//    }else{
//      traj = robotPt->t[robotPt->tcur->num - 2];
//    }
//  }
//  if(cartesian == 0) {
//    /* plan in the C_space */
//    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
//    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-arm_lin", 1) ;
//    deactivateCcCntrts(robotPt, -1);
//  } else {
//    /* plan in the cartesian space */
//    qi = p3d_alloc_config(robotPt);
//    qf = p3d_alloc_config(robotPt);
//    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
//    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-ob_lin", 1) ;
//    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &qi);
//    p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &qf);
//    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qi);
//    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qf);
//    p3d_copy_config_into(robotPt, qi, &robotPt->ROBOT_POS);
//    p3d_copy_config_into(robotPt, qf, &robotPt->ROBOT_GOTO);
//    p3d_destroy_config(robotPt, qi);
//    p3d_destroy_config(robotPt, qf);
//    if(robotPt->nbCcCntrts!=0) {
//      p3d_activateCntrt(robotPt, robotPt->ccCntrts[0]);
//    }
//  }
//  if (currentLpId > traj->nlp){
//    return 1;
//  }
//  p3d_localpath* currentLp = traj->courbePt;
//  for(int i = 0; i < currentLpId; i++){
//    currentLp = currentLp->next_lp;
//  }
//  return checkForCollidingPath(robotPt, traj, currentLp);
//}
//
////! Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
////! \return 0 in case of success, !=0 otherwise
//int genomReplanCollidingTraj(p3d_rob* robotPt, int cartesian, double* armConfig, int currentLpId, int lp[], Gb_q6 positions[],  int *nbPositions) {
//  configPt qi = NULL, qf = NULL;
//  static p3d_traj *traj = NULL;
//  int ntest=0;
//  double gain;
//
//  XYZ_ENV->cur_robot= robotPt;
//  //initialize and get the current linear traj
//  if (!traj){
//    if(robotPt->nt < robotPt->tcur->num - 2){
//      return 1;
//    }else{
//      traj = robotPt->t[robotPt->tcur->num - 2];
//    }
//  }
//  if(cartesian == 0) {
//    /* plan in the C_space */
//    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
//    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-arm_lin", 1) ;
//    deactivateCcCntrts(robotPt, -1);
//  } else {
//    /* plan in the cartesian space */
//    qi = p3d_alloc_config(robotPt);
//    qf = p3d_alloc_config(robotPt);
//    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
//    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-ob_lin", 1) ;
//    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &qi);
//    p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &qf);
//    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qi);
//    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qf);
//    p3d_copy_config_into(robotPt, qi, &robotPt->ROBOT_POS);
//    p3d_copy_config_into(robotPt, qf, &robotPt->ROBOT_GOTO);
//    p3d_destroy_config(robotPt, qi);
//    p3d_destroy_config(robotPt, qf);
//    if(robotPt->nbCcCntrts!=0) {
//      p3d_activateCntrt(robotPt, robotPt->ccCntrts[0]);
//    }
//  }
//  if (currentLpId > traj->nlp){
//    return 1;
//  }
//  p3d_localpath* currentLp = traj->courbePt;
//  for(int i = 0; i < currentLpId; i++){
//    currentLp = currentLp->next_lp;
//  }
//  configPt currentConfig = p3d_get_robot_config(robotPt);
//  int j = 0, returnValue = 0, optimized = traj->isOptimized;
//  if(optimized){
//    p3dAddTrajToGraph(robotPt, robotPt->GRAPH, traj);
//  }
//  do{
//    printf("Test %d\n", j);
//    j++;
//    returnValue = replanForCollidingPath(robotPt, traj, robotPt->GRAPH, currentConfig, currentLp, optimized);
//    traj = robotPt->tcur;
//    currentLp = traj->courbePt;
//  }while(returnValue != 1 && returnValue != 0);
//  if (optimized && j > 1){
//    optimiseTrajectory(100,6);
//  }
//  if(j > 1){//There is a new traj
//    /* COMPUTE THE SOFTMOTION TRAJECTORY */
//    traj = robotPt->tcur;
//    if(!traj) {
//      printf("SoftMotion : ERREUR : no current traj\n");
//      return 1;
//    }
//    if(!traj || traj->nlp < 1) {
//        printf("Optimization with softMotion not possible: current trajectory contains one or zero local path\n");
//      return 1;
//    }
////     if(p3d_optim_traj_softMotion(traj, true, &gain, &ntest, lp, positions, nbPositions) == 1){
////       printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
////       return 1;
////     }
//    //peut etre ajouter un return specific pour savoir qu'il y'a une nouvelle traj
//  }
//  return 0;
//}
//#endif



void genomDraw()
{
 	if (manipulation== NULL) {
	  initManipulationGenom();
	}
	manipulation->draw();
}


void genomKey()
{
  if (manipulation== NULL) {
	  initManipulationGenom();
  }
//   manipulation->setCapture(!(manipulation->getCapture()));
//   printf("CAPTURE= %d\n", manipulation->getCapture());

//   manipulation->displayPlacements= !manipulation->displayPlacements;
  manipulation->centerCamera();
  g3d_draw_allwin_active();
}


static void CB_genomFindSimpleGraspConfiguration_obj(FL_OBJECT *obj, long arg) {
//  double pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6;

 double q1, q2, q3, q4, q5, q6;
 q1= q2= q3= q4= q5= q6= 0.0;
 p3d_rob *curRobotPt= NULL, *robotPt = NULL, *hand_robotPt= NULL;

 	if (manipulation== NULL) {
	  initManipulationGenom();
	}

 curRobotPt=  (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
 robotPt= p3d_get_robot_by_name(GP_ROBOT_NAME);
// set the object
//  manipulation->findSimpleGraspConfiguration(&q1, &q2, &q3, &q4, &q5, &q6);

 hand_robotPt= p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME);

//  genomFindGraspConfiguration(robotPt, hand_robotPt, OBJECT_NAME, &q1, &q2, &q3, &q4, &q5, &q6);
// genomFindPregraspAndGraspConfiguration(robotPt, hand_robotPt, (char*)OBJECT_NAME, 0.0, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6);
// manipulation->findPregraspAndGraspConfiguration()
    manipulation->setArmQ(q1, q2, q3, q4, q5, q6);
    g3d_win *win= NULL;
    win= g3d_get_cur_win();

    win->fct_draw2= &(genomDraw);
    win->fct_key2= &(genomKey);

    g3d_draw_allwin();
    g3d_draw_allwin_active();


 
   p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_POS);
   XYZ_ENV->cur_robot= curRobotPt;
}



int genomSetInterfaceQuality() {
  g3d_win * win = NULL;
  win = g3d_get_cur_win();

  if(win->vs.displayFloor == FALSE) {
          win->vs.displayShadows = TRUE;
          win->vs.displayFloor = TRUE;
          win->vs.displayTiles = TRUE;
          win->vs.displayWalls = TRUE;
  } else {
          win->vs.displayShadows = FALSE;
          win->vs.displayFloor = FALSE;
          win->vs.displayTiles = FALSE;
          win->vs.displayWalls = FALSE;

  }
  return 0;
}


// static void CB_genomComputeTrajFromConfigs_obj(FL_OBJECT *obj, long arg) {
//         int cartesian = FORMGENOM_CARTESIAN;
//         int i, r, nr, itraj;
//         p3d_rob *robotPt = NULL;
//         r = p3d_get_desc_curnum(P3D_ROBOT);
//         nr= p3d_get_desc_number(P3D_ROBOT);
// //         p3d_traj * trajs[20];
// 
//         for(i=0; i<nr; i++){
//                 robotPt= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
//                 if(strcmp(GP_ROBOT_NAME, robotPt->name)==0){
//                         break;
//                 }
//         }
// 
//         int lp[10000];
//         Gb_q6 positions[10000];
//         int nbPositions = 0;
// 
// //         configPt qi = NULL, qf = NULL;
// //         int result;
//         p3d_traj *traj = NULL;
//         int ntest=0;
//         configPt q1 = NULL, q2 = NULL;
//         double gain;
// 
//         XYZ_ENV->cur_robot= robotPt;
//         FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
// 
//         deleteAllGraphs();
//    // deactivate collisions for all robots:
// //         for(i=0; i<XYZ_ENV->nr; i++) {
// //                 if(XYZ_ENV->robot[i]==robotPt){
// //                         continue;
// //                 } else {
// //                         p3d_col_deactivate_robot(XYZ_ENV->robot[i]);
// //                 }
// //         }
// 
//         if(robotPt!=NULL) {
//                 while(robotPt->nt!=0)
//                 {   p3d_destroy_traj(robotPt, robotPt->t[0]);  }
//                 FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
//         }
//         printf("il y a %d configurations\n", robotPt->nconf);
//         for(itraj = 0; itraj < robotPt->nconf-1; itraj++) {
//                 q1 = robotPt->conf[itraj]->q;
//                 q2 = robotPt->conf[itraj+1]->q;
//                 genomComputePathBetweenTwoConfigs(robotPt, cartesian, q1, q2);
//         }
// 
//         GP_ConcateneAllTrajectories(robotPt);
//         robotPt->tcur= robotPt->t[0];
// 
//         /* COMPUTE THE SOFTMOTION TRAJECTORY */
//         traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
//         if(!traj) {
//                 printf("SoftMotion : ERREUR : no current traj\n");
//                 return;
//         }
//         if(!traj || traj->nlp < 1) {
//                 printf("Optimization with softMotion not possible: current trajectory   contains one or zero local path\n");
//                 return;
//         }
//         if(p3d_optim_traj_softMotion(traj, true, &gain, &ntest, lp, positions, &nbPositions) == 1){
//                 printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
//                 return;
//         }
//         return;
// 
//         fl_set_button(BT_ARM_GOTO_Q_OBJ,0);
//         return;
// 
// }

static void CB_genomGraspObject(FL_OBJECT *obj, long arg) {
     double distance = 0.1;
      double pre_q1, pre_q2, pre_q3,pre_q4, pre_q5, pre_q6, q1, q2, q3, q4, q5, q6;

      if (manipulation== NULL) {
	  initManipulationGenom();
      }
       manipulation->setObjectToManipulate((char*)OBJECT_NAME);
       if(manipulation->isObjectGraspable((char*)OBJECT_NAME) == false) {
	  std::cout << "this object is not graspable " << std::endl;
	  return;
       }

       manipulation->findPregraspAndGraspConfiguration(distance, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6);
       manipulation->setArmQ(q1, q2, q3, q4, q5, q6);
       g3d_draw_allwin_active();
}


static void CB_genomPickUp_gotoObject(FL_OBJECT *obj, long arg) {





        int nbPositions = 0;
//         double x, y, theta;
	if (manipulation== NULL) {
	  initManipulationGenom();
	}

	if(FORMGENOM_CARTESIAN == 1) {
	  manipulation->setArmCartesian(true);
	} else {
	  manipulation->setArmCartesian(false);
	}

        manipulation->setObjectToManipulate((char*)OBJECT_NAME);
        manipulation->setSupport((char*)SUPPORT_NAME);
        manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
        manipulation->setCameraFOV(CAMERA_FOV);
        manipulation->setCameraImageSize(200, 200);
        manipulation->armPlanTask(ARM_PICK_GOTO,manipulation->robotStart(), manipulation->robotGoto(), (char*)OBJECT_NAME,  manipulation->lp,  manipulation->positions);


        g3d_win *win= NULL;
        win= g3d_get_cur_win();
        win->fct_draw2= &(genomDraw);
        win->fct_key1= &(genomKey);
	g3d_draw_allwin_active();
	return;
}

static void CB_genomPickUp_takeObject(FL_OBJECT *obj, long arg) {

        int nbPositions = 0;

	if (manipulation== NULL) {
	  initManipulationGenom();
	}

	if(FORMGENOM_CARTESIAN == 1) {
	  manipulation->setArmCartesian(true);
	} else {
	  manipulation->setArmCartesian(false);
	}

        manipulation->setObjectToManipulate((char*)OBJECT_NAME);
	
        manipulation->armPlanTask(ARM_PICK_TAKE_TO_FREE,manipulation->robotStart(), manipulation->robotGoto(), (char*)OBJECT_NAME,  manipulation->lp,  manipulation->positions);

	g3d_draw_allwin_active();
	return;
}


static void CB_genomPickUp_placeObject(FL_OBJECT *obj, long arg) {

  int nbPositions;

  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    manipulation->setArmCartesian(true);
  } else {
    manipulation->setArmCartesian(false);
  }
  
  manipulation->setObjectToManipulate((char*)OBJECT_NAME);
  manipulation->setSupport((char*)SUPPORT_NAME);
  manipulation->setHuman((char*)HUMAN_NAME);

  manipulation->armPlanTask(ARM_PICK_TAKE_TO_PLACE,manipulation->robotStart(), manipulation->robotGoto(), (char*)OBJECT_NAME, manipulation->lp, manipulation->positions);

  g3d_draw_allwin_active();

  return;
}

static void CB_genomPlaceObject(FL_OBJECT *obj, long arg) {

  int nbPositions = 0;

  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    manipulation->setArmCartesian(true);
  } else {
    manipulation->setArmCartesian(false);
  }
  
  manipulation->setObjectToManipulate((char*)OBJECT_NAME);
  manipulation->setSupport((char*)SUPPORT_NAME);
  manipulation->setHuman((char*)HUMAN_NAME);

  manipulation->armPlanTask(ARM_PLACE_FROM_FREE,manipulation->robotStart(), manipulation->robotGoto(), (char*)OBJECT_NAME, manipulation->lp, manipulation->positions);

  g3d_draw_allwin_active();

  return;
}



// #endif


