#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "../other_libraries/gbM/src/Proto_gbModeles.h"
#include <list>
#include <string>
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"


#if defined(MULTILOCALPATH) && defined(GRASP_PLANNING)
static double QCUR[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double QGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double XCUR[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double XGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

static int INIT_IS_DONE= FALSE;
static p3d_obj *OBJECT= NULL;
//static p3d_polyhedre *POLYHEDRON= NULL; // the polyhedron associated to the object
static gpHand_properties HAND;  // information about the used hand
//static p3d_vector3 CMASS; // object's center of mass
//static p3d_matrix3 IAXES; // object's main inertia axes
//static double IAABB[6]; // bounding box aligned on the object's inertia axes
static std::list<gpGrasp> GRASPLIST;
static gpGrasp GRASP;   // the current grasp
static std::list<gpPose> POSELIST;
p3d_matrix4 EEFRAME, GFRAME;


/* --------- FORM VARIABLES ------- */
FL_FORM  * GENOM_FORM = NULL;
static FL_OBJECT * GENOMGROUP;
static FL_OBJECT * BT_SET_Q_OBJ;
static FL_OBJECT * BT_SET_X_OBJ;
static FL_OBJECT * BT_ARM_GOTO_Q_OBJ;
static FL_OBJECT * BT_SIMPLE_GRASP_PLANNER_OBJ;
static FL_OBJECT  *INPUT_OBJ1, *INPUT_OBJ2;
static FL_FORM *INPUT_FORM;

/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_genom_group(void);
static int genomInitGraspPlanning(char *hand_type_name);
static void genomDraw();

static void CB_genomSetQ_obj(FL_OBJECT *obj, long arg);
static void CB_genomSetX_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmGotoQ_obj(FL_OBJECT *obj, long arg);
static void CB_genomFindSimpleGraspConfiguration_obj(FL_OBJECT *obj, long arg);



/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_genom_form(void)
{
	GENOM_FORM = fl_bgn_form(FL_UP_BOX, 150, 340);
	g3d_create_genom_group();
	fl_end_form();
}

void g3d_show_genom_form(void)
{
	fl_show_form(GENOM_FORM, FL_PLACE_SIZE, TRUE, "Genom Requests");
}

void g3d_hide_genom_form(void)
{
	fl_hide_form(GENOM_FORM);
}

void g3d_delete_genom_form(void)
{
	fl_free_form(GENOM_FORM);
}


/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_genom_group(void)
{
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 310, "Genom Requests");

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
	BT_ARM_GOTO_Q_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Goto Q");
//	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
	fl_set_call_back(BT_ARM_GOTO_Q_OBJ, CB_genomArmGotoQ_obj, 1);

        y+= dy;
        BT_SIMPLE_GRASP_PLANNER_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Simple grasp config");
	fl_set_call_back(BT_SIMPLE_GRASP_PLANNER_OBJ, CB_genomFindSimpleGraspConfiguration_obj, 1);

	fl_end_group();
}

static void CB_read_Q(FL_OBJECT *ob, long arg) {
 int result;
 const char *str;
 p3d_rob *robotPt = NULL;


 robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

 str= fl_get_input(INPUT_OBJ2);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &QGOAL[0], &QGOAL[1], &QGOAL[2], &QGOAL[3], &QGOAL[4], &QGOAL[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: q1 q2 q3 q4 q5 q6\n", 1);
   printf("Expected format of QGOAL input: q1 q2 q3 q4 q5 q6\n");
   QGOAL[0]= QGOAL[1]= QGOAL[2]= QGOAL[3]= QGOAL[4]= QGOAL[5]= 0.0;
 }
 result= genomSetArmQ(robotPt, QGOAL[0], QGOAL[1], QGOAL[2], QGOAL[3], QGOAL[4], QGOAL[5]);
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


 result = genomSetArmQ(robotPt, QCUR[0], QCUR[1], QCUR[2], QCUR[3], QCUR[4], QCUR[5]);
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


 robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);

 str= fl_get_input(INPUT_OBJ2);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &XGOAL[0], &XGOAL[1], &XGOAL[2], &XGOAL[3], &XGOAL[4], &XGOAL[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: x y z rx ry rz\n", 1);
   printf("Expected format of XGOAL input: x y z rx ry rz\n");
   XGOAL[0]= XGOAL[1]= XGOAL[2]= XGOAL[3]= XGOAL[4]= XGOAL[5]= 0.0;
 }
 p3d_set_virtual_object_pose2(robotPt, XGOAL[0], XGOAL[1], XGOAL[2], XGOAL[3], XGOAL[4], XGOAL[5]);
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


 p3d_set_virtual_object_pose2(robotPt, XCUR[0], XCUR[1], XCUR[2], XCUR[3], XCUR[4], XCUR[5]);


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

  robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  q0= p3d_get_robot_config(robotPt);
  genomGetArmQ(robotPt, &QCUR[0], &QCUR[1], &QCUR[2], &QCUR[3], &QCUR[4], &QCUR[5]);
  p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_GOTO);
  genomGetArmQ(robotPt, &QGOAL[0], &QGOAL[1], &QGOAL[2], &QGOAL[3], &QGOAL[4], &QGOAL[5]);
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

  robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  q0= p3d_get_robot_config(robotPt);
  p3d_get_virtual_object_pose2(robotPt, &XCUR[0], &XCUR[1], &XCUR[2], &XCUR[3], &XCUR[4], &XCUR[5]);
  p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_GOTO);
  p3d_get_virtual_object_pose2(robotPt, &XGOAL[0], &XGOAL[1], &XGOAL[2], &XGOAL[3], &XGOAL[4], &XGOAL[5]);
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
	int cartesian = 0;
	int i, r, nr;
	p3d_rob *robotPt = NULL;
	r = p3d_get_desc_curnum(P3D_ROBOT);
	nr= p3d_get_desc_number(P3D_ROBOT);

	for(i=0; i<nr; i++){
		robotPt= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
		if(strcmp("ROBOT", robotPt->name)==0){
			break;
		}
	}

	int lp[10000];
	Gb_q6 positions[10000];
	int nbPositions = 0;

	genomArmGotoQ(robotPt, cartesian, lp, positions, &nbPositions);
	fl_set_button(BT_ARM_GOTO_Q_OBJ,0);
	return;
}

int genomArmGotoQ(p3d_rob* robotPt, int cartesian, int lp[], Gb_q6 positions[],  int *nbPositions) {
	configPt qi = NULL, qf = NULL;
	int result;
	p3d_traj *traj = NULL;
	int ntest=0;
	unsigned int i = 0;
	double gain;

	XYZ_ENV->cur_robot= robotPt;
	deleteAllGraphs();
   // deactivate collisions for all robots:
	for(i=0; i<(unsigned int) XYZ_ENV->nr; i++) {
		if(XYZ_ENV->robot[i]==robotPt){
			continue;
		} else {
			p3d_col_deactivate_robot(XYZ_ENV->robot[i]);
		}
	}


	if(robotPt!=NULL) {
		while(robotPt->nt!=0)
		{   p3d_destroy_traj(robotPt, robotPt->t[0]);  }
		FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
	}

	if(cartesian == 0) {
	/* plan in the C_space */
		p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
		p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-arm_lin", 1) ;
// 		if(robotPt->nbCcCntrts!=0) {
// 			p3d_desactivateCntrt(robotPt, robotPt->ccCntrts[0]);
// 		}
 		deactivateCcCntrts(robotPt, -1);
//  		p3d_desactivateAllCntrts(robotPt);

	} else {
		/* plan in the cartesian space */
		qi = p3d_alloc_config(robotPt);
		qf = p3d_alloc_config(robotPt);
		p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
		p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, "jido-ob_lin", 1) ;
		p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &qi);
		p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &qf);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qi);
		p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qf);
		p3d_copy_config_into(robotPt, qi, &robotPt->ROBOT_POS);
		p3d_copy_config_into(robotPt, qf, &robotPt->ROBOT_GOTO);
		p3d_destroy_config(robotPt, qi);
		p3d_destroy_config(robotPt, qf);
		if(robotPt->nbCcCntrts!=0) {
			p3d_activateCntrt(robotPt, robotPt->ccCntrts[0]);
		}
	}
	/* Init RRT */
	ENV.setBool(Env::biDir,true);
	ENV.setInt(Env::NbTry, 100000);
	ENV.setInt(Env::MaxExpandNodeFail, 30000);
	ENV.setInt(Env::maxNodeCompco, 100000);
	ENV.setExpansionMethod(Env::Connect);
// 	ENV.setExpansionMethod(Env::Extend);


	if(p3d_equal_config(robotPt, robotPt->ROBOT_POS, robotPt->ROBOT_GOTO)) {
		printf("genomArmGotoQ: Start and goal configurations are the same.\n");
		return 1;
	}
	p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
	result= p3d_specific_search("out.txt");
  // optimizes the trajectory:
	//CB_start_optim_obj(NULL, 0);
  // reactivate collisions for all other robots:
	for(i=0; i<(unsigned int) XYZ_ENV->nr; i++) {
		if(XYZ_ENV->robot[i]==robotPt){
			continue;
		} else {
			p3d_col_activate_robot(XYZ_ENV->robot[i]);
		}
	}
	p3d_SetTemperatureParam(1.0);
	if(!result){
		printf("The planner could not find a path to fold the arm.\n");
		return 1;
	}
	robotPt->tcur= robotPt->t[0];

	/* COMPUTE THE SOFTMOTION TRAJECTORY */
	traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	if(!traj) {
		printf("SoftMotion : ERREUR : no current traj\n");
		return 1;
	}
	if(!traj || traj->nlp < 1) {
			printf("Optimization with softMotion not possible: current trajectory	contains one or zero local path\n");
		return 1;
	}
	if(p3d_optim_traj_softMotion(traj, 1, &gain, &ntest, lp, positions, nbPositions) == 1){
		printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
		return 1;
	}
	return 0;
}


static void CB_genomFindSimpleGraspConfiguration_obj(FL_OBJECT *obj, long arg) {
 double q1, q2, q3, q4, q5, q6;
 p3d_rob *robotPt = NULL;

 robotPt= p3d_get_robot_by_name("ROBOT");

 genomFindSimpleGraspConfiguration(robotPt, "object", 1000, &q1, &q2, &q3, &q4, &q5, &q6);

 genomSetArmQ(robotPt, q1, q2, q3, q4, q5, q6);
}

//! Sets the robot's arm configuration with the given values (in radians).
//! NB: The respect of joint bounds is verified.
//! \param robot pointer to the robot
//! \param q1 value of joint #1
//! \param q2 value of joint #2
//! \param q3 value of joint #3
//! \param q4 value of joint #4
//! \param q5 value of joint #5
//! \param q6 value of joint #6
//! \return 0 in case of success, 1 otherwise
int genomSetArmQ(p3d_rob *robot, double q1, double q2, double q3, double q4, double q5, double q6)
{
  if(robot==NULL)
  {
    printf("%s: %d: genomSetArmQ(): robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  int isValid, verbose= 1; //set verbose to 1/0 to enable/disable printf
	int result;
  double qmin, qmax;
  p3d_jnt *armJoint= NULL;
  configPt q= NULL;

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);

  ////////////////////////q1////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT1);
  if(armJoint==NULL)
  {
    p3d_destroy_config(robot, q);
    return 1;
  }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q1 > qmax)
  {
    q1-= 2*M_PI;
    if( (q1 < qmin) || (q1 > qmax) )
    {  isValid= 0; }
  }
  if(q1 < qmin)
  {
    q1+= 2*M_PI;
    if( (q1 < qmin) || (q1 > qmax) )
    {  isValid= 0; }
  }
  if(isValid)
  { q[armJoint->index_dof]=  q1;   }
  else
  {
    if(verbose)
    {
      printf("%s: %d: genomSetArmQ(): q1 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q1,qmin,qmax);
    }
    p3d_destroy_config(robot, q);
    return 1;
  }
  /////////////////////////////////////////////////////


  ////////////////////////q2////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT2);
  if(armJoint==NULL)
  {
    p3d_destroy_config(robot, q);
    return 1;
  }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q2 > qmax)
  {
    q2-= 2*M_PI;
    if( (q2 < qmin) || (q2 > qmax) )
    {  isValid= 0; }
  }
  if(q2 < qmin)
  {
    q2+= 2*M_PI;
    if( (q2 < qmin) || (q2 > qmax) )
    {  isValid= 0; }        }
  if(isValid)
  { q[armJoint->index_dof]=  q2;   }
  else
  {
    if(verbose)
    {
        printf("%s: %d: genomSetArmQ(): q2 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q2,qmin,qmax);
    }
    p3d_destroy_config(robot, q);
    return 1;
  }
  /////////////////////////////////////////////////////


  ////////////////////////q3////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT3);
  if(armJoint==NULL)
  {
    p3d_destroy_config(robot, q);
    return 1;
  }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q3 > qmax)
  {
    q3-= 2*M_PI;
    if( (q3 < qmin) || (q3 > qmax) )
    {  isValid= 0; }
  }
  if(q3 < qmin)
  {
    q3+= 2*M_PI;
    if( (q3 < qmin) || (q3 > qmax) )
    {  isValid= 0; }
  }
  if(isValid)
  { q[armJoint->index_dof]=  q3;   }
  else
  {
    if(verbose)
    {
        printf("%s: %d: genomSetArmQ(): q3 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q3,qmin,qmax);
    }
    p3d_destroy_config(robot, q);
    return 1;
  }
  /////////////////////////////////////////////////////


  ////////////////////////q4////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT4);
  if(armJoint==NULL)
  {
    p3d_destroy_config(robot, q);
    return 1;
  }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q4 > qmax)
  {
    q4-= 2*M_PI;
    if( (q4 < qmin) || (q4 > qmax) )
    {  isValid= 0; }
  }
  if(q4 < qmin)
  {
    q4+= 2*M_PI;
    if( (q4 < qmin) || (q4 > qmax) )
    {  isValid= 0; }
  }
  if(isValid)
  { q[armJoint->index_dof]=  q4;   }
  else
  {
    if(verbose)
    {
        printf("%s: %d: genomSetArmQ(): q4 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q4,qmin,qmax);
    }
    p3d_destroy_config(robot, q);
    return 1;
  }
  /////////////////////////////////////////////////////


  ////////////////////////q5////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT5);
  if(armJoint==NULL)
  {
    p3d_destroy_config(robot, q);
    return 1;
  }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q5 > qmax)
  {
    q5-= 2*M_PI;
    if( (q5 < qmin) || (q5 > qmax) )
    {  isValid= 0; }
  }
  if(q5 < qmin)
  {
    q5+= 2*M_PI;
    if( (q5 < qmin) || (q5 > qmax) )
    {  isValid= 0; }
  }
  if(isValid)
  { q[armJoint->index_dof]=  q5;   }
  else
  {
    if(verbose)
    {
        printf("%s: %d: genomSetArmQ(): q5 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q5,qmin,qmax);
    }
    p3d_destroy_config(robot, q);
    return 1;
  }
  /////////////////////////////////////////////////////


  ////////////////////////q6////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(robot, GP_WRISTJOINT);
  if(armJoint==NULL)
  {
    p3d_destroy_config(robot, q);
    return 1;
  }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q6 > qmax)
  {
    q6-= 2*M_PI;
    if( (q6 < qmin) || (q6 > qmax) )
    {  isValid= 0; }
  }
  if(q6 < qmin)
  {
    q6+= 2*M_PI;
    if( (q6 < qmin) || (q6 > qmax) )
    {  isValid= 0; }
  }
  if(isValid)
  { q[armJoint->index_dof]=  q6;   }
  else
  {
    if(verbose)
    {
        printf("%s: %d: genomSetArmQ(): q6 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q6,qmin,qmax);
    }
    p3d_destroy_config(robot, q);
    return 1;
  }

//   if(robot->nbCcCntrts!=0) {
//     p3d_desactivateCntrt(robot, robot->ccCntrts[0]);
//   }

//   p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, q);
  deactivateCcCntrts(robot, -1);
  result= p3d_set_and_update_this_robot_conf(robot, q);

  p3d_destroy_config(robot, q);

	return !result;

//   if(robot->nbCcCntrts!=0) {
//     p3d_activateCntrt(robot, robot->ccCntrts[0]);
//   }

//   return 0;
}


//! Gets the robot's arm configuration (in radians).
//! \param robot pointer to the robot
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 0 in case of success, 1 otherwise
int genomGetArmQ(p3d_rob *robot, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6)
{
  if(robot==NULL)
  {
    printf("%s: %d: genomGetArmQ(): robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  p3d_jnt *armJoint= NULL;

  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT1);
  if(armJoint==NULL)
  {  return 0; }
  *q1= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT2);
  if(armJoint==NULL)
  {  return 0; }
  *q2= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT3);
  if(armJoint==NULL)
  {  return 0; }
  *q3= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT4);
  if(armJoint==NULL)
  {  return 0; }
  *q4= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(robot, GP_ARMJOINT5);
  if(armJoint==NULL)
  {  return 0; }
  *q5= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(robot, GP_WRISTJOINT);
  if(armJoint==NULL)
  {  return 0; }
  *q6= armJoint->dof_data[0].v;


  return 0;
}



//! Initializes all that will be required to compute grasps for a given object with a given hand.
//! \param hand_type_name name of the hand type that will be used
//! \return 0 in case of success, 1 otherwise
int genomInitGraspPlanning(char *hand_type_name)
{
  if(p3d_col_get_mode()!=p3d_col_mode_pqp)  {
    printf("%s: %d: The collision detector MUST be PQP to use graspPlanning module.\n",__FILE__,__LINE__);
    return 1;
  }

  if(strcmp(hand_type_name, "GRIPPER")==0)  {
    HAND.initialize(GP_GRIPPER);
  }
  else {
         if(strcmp(hand_type_name, "SAHAND_RIGHT")==0)  {
           HAND.initialize(GP_SAHAND_RIGHT);
         }
         else {
           printf("%s: %d: unknown hand type: \"%s\".\n",__FILE__,__LINE__,hand_type_name);
           return 1;
         }
  }

  INIT_IS_DONE= TRUE;

  return 0;
}

//! Computes a list of grasp for the given robot (freeflyer hand robot).
//! NB: genomInitGraspPlanning must have been called before
//! \param robot pointer to the robot
//! \param object_name name of the object
//! \return 0 in case of success, 1 otherwise
int genomComputeObjectGraspList(p3d_rob *hand_robot, char *object_name, int maxNbGrasps)
{
  int i;
  p3d_polyhedre *polyhedron= NULL; // the polyhedron associated to the object
//   gpHand_properties hand;  // information about the used hand
  p3d_vector3 cmass; // object's center of mass
  p3d_matrix3 iaxes; // object's main inertia axes
  double iaabb[6]; // bounding box aligned on the object's inertia axes

  if( INIT_IS_DONE==FALSE )  {
    printf("%s: %d: call genomInitGraspPlanning() first .\n",__FILE__,__LINE__);
    return 1;
  }

  HAND.max_nb_grasp_frames= maxNbGrasps;

  OBJECT= NULL;
  OBJECT= p3d_get_obst_by_name(object_name);

  if(OBJECT==NULL)  {
    printf("%s: %d: There is no object with name \"%s\".\n", __FILE__, __LINE__, object_name);
    return 1;
  }


  polyhedron= OBJECT->pol[0]->poly;
  poly_build_planes(polyhedron);

  Mass_properties mass_prop;
  gpCompute_mass_properties(polyhedron, &mass_prop);
  gpCompute_inertia_axes(&mass_prop, iaxes);
  p3d_vectCopy(mass_prop.r, cmass);
  gpInertia_AABB(polyhedron, cmass, iaxes, iaabb);

  printf("center of mass: \n\t %f %f %f \n", cmass[0], cmass[1], cmass[2] );
  printf("inertia axes: \n\t %f %f %f \n", iaxes[0][0], iaxes[0][1], iaxes[0][2] );
  printf("\t %f %f %f \n", iaxes[1][0], iaxes[1][1], iaxes[1][2] );
  printf("\t %f %f %f \n", iaxes[2][0], iaxes[2][1], iaxes[2][2] );


  // deactivate collisions for all robots except for the two of them needed by the grasp planner:
  for(i=0; i<XYZ_ENV->nr; i++) {
    if(XYZ_ENV->robot[i]==hand_robot) {
       continue;
    }
    else  {
      p3d_col_deactivate_robot(XYZ_ENV->robot[i]);
   }
  }

  printf("Collisions are deactivated for other robots.\n");

  gpGrasp_generation(hand_robot, OBJECT, 0, cmass, iaxes, iaabb, HAND, HAND.translation_step, HAND.nb_directions, HAND.rotation_step, GRASPLIST);

  printf("Before collision filter: %d grasps.\n", GRASPLIST.size());
  gpGrasp_collision_filter(GRASPLIST, hand_robot, OBJECT, HAND);
  printf("After collision filter: %d grasps.\n", GRASPLIST.size());
  gpGrasp_stability_filter(GRASPLIST);
  printf("After stability filter: %d grasps.\n", GRASPLIST.size());

  gpGrasp_context_collision_filter(GRASPLIST, hand_robot, OBJECT, HAND);
  printf("For the current collision context: %d grasps.\n", GRASPLIST.size());
  p3d_col_deactivate_robot(hand_robot);

  //redraw();

  if(GRASPLIST.empty())  {
    printf("genomComputeObjectGraspList(): No grasp was found.\n");
    return 1;
  }

  return 1;
}

int genomFindGraspConfiguration(p3d_rob *robotPt, double q[6])
{
  if( INIT_IS_DONE==FALSE )  {
    printf("%s: %d: call genomInitGraspPlanning() first .\n",__FILE__,__LINE__);
    return 1;
  }


  gpGrasp grasp;

  configPt qcurrent= NULL;
  configPt qresult= NULL;

  qcurrent= p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &qcurrent);

  qresult= gpFind_grasp_from_base_configuration(robotPt, OBJECT, GRASPLIST, GP_PA10, qcurrent, grasp, HAND);
  p3d_destroy_config(robotPt, qcurrent);

  if(qresult==NULL)  {
    return 1;
  }

  p3d_set_and_update_this_robot_conf(robotPt, qresult);
  genomGetArmQ(robotPt, &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]);

  p3d_destroy_config(robotPt, qresult);

  return 0;
}

int genomSetInterfaceQuality() {
	g3d_win * win = NULL;
	win = g3d_get_cur_win();

	if(win->displayFloor == FALSE) {
		win->displayShadows = TRUE;
		win->displayFloor = TRUE;
		win->displayTiles = TRUE;
		win->displayWalls = TRUE;
	} else {
		win->displayShadows = FALSE;
		win->displayFloor = FALSE;
		win->displayTiles = FALSE;
		win->displayWalls = FALSE;


	}
	return 0;
	}


//! Finds a configuration to grasp an object with a simple shape (cylinder, bottle, box, etc.).
//! All the grasps configurations are computed so that the hand grasps the object vertically.
//! \param robotPt pointer to the robot
//! \param object_name name of the object to grasp
//! \param nbTries maximal number of grasps that will be tested
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 0 in case of success,
//!         1 in case no grasp was found and the object is unreachable
//!         2 in case no grasp was found but the object is reachable
int genomFindSimpleGraspConfiguration(p3d_rob *robotPt, char *object_name, int nbTries, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6)
{
  if(robotPt==NULL) {
    printf("%s: %d: genomFindSimpleGraspConfiguration(): input robot is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  int i, result;
  int unreachable;
  double x, y, z, theta, radius, r1, r2, r3;
  p3d_vector3 p;
  p3d_matrix4 object_pose;
  p3d_polyhedre *polyhedron= NULL;
  p3d_vector3 cmass, cmass_abs; // object's center of mass
  p3d_matrix3 iaxes; // object's main inertia axes
  p3d_vector3 iaxes_abs[3];
  double iaabb[6]; // bounding box aligned on the object's inertia axes
  configPt q= NULL;
  gpHand_properties hand;

  hand.initialize(GP_GRIPPER);

  OBJECT= NULL;
  OBJECT= p3d_get_obst_by_name(object_name);

  if(OBJECT==NULL) {
    printf("%s: %d: There is no object with name \"%s\".\n", __FILE__, __LINE__, object_name);
    return 1;
  }

  polyhedron= OBJECT->pol[0]->poly;
  p3d_get_poly_pos(OBJECT->pol[0]->poly, object_pose);

  Mass_properties mass_prop;
  gpCompute_mass_properties(polyhedron, &mass_prop);
  gpCompute_inertia_axes(&mass_prop, iaxes);
  p3d_vectCopy(mass_prop.r, cmass);
  gpInertia_AABB(polyhedron, cmass, iaxes, iaabb);

/*  printf("center of mass: \n\t %f %f %f \n", cmass[0], cmass[1], cmass[2] );
  printf("inertia axes: \n\t %f %f %f \n", iaxes[0][0], iaxes[0][1], iaxes[0][2] );
  printf("\t %f %f %f \n", iaxes[1][0], iaxes[1][1], iaxes[1][2] );
  printf("\t %f %f %f \n", iaxes[2][0], iaxes[2][1], iaxes[2][2] );*/

  p3d_xformPoint(object_pose, cmass, cmass_abs);
  for(i=0; i<3; i++) {
    p[0]= iaxes[0][i];
    p[1]= iaxes[1][i];
    p[2]= iaxes[2][i];
    p3d_xformVect(object_pose, p, iaxes_abs[i]);
  }

  unreachable= TRUE;

//   g3d_win *win= NULL;
//   win= g3d_get_cur_win();
//   win->fct_draw2= &(genomDraw);
  q= p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &q);

  radius= 0;
  for(i=0; i<nbTries; i++)   {
     radius+= 0.001;
     if(radius > 1.0) radius= 1.0;

     r1= p3d_random( radius*iaabb[0], radius*iaabb[1] );
     r2= p3d_random( radius*iaabb[2], radius*iaabb[3] );
     r3= p3d_random( radius*iaabb[4], radius*iaabb[5] );

     x= cmass_abs[0] + r1*iaxes_abs[0][0] + r2*iaxes_abs[0][1] + r3*iaxes_abs[0][2];
     y= cmass_abs[1] + r1*iaxes_abs[1][0] + r2*iaxes_abs[1][1] + r3*iaxes_abs[1][2];
     z= cmass_abs[2] + r1*iaxes_abs[2][0] + r2*iaxes_abs[2][1] + r3*iaxes_abs[2][2];

     theta= p3d_random(0, 2*M_PI);

     result= p3d_set_virtual_object_pose2(robotPt, x, y, z, 0, 0, theta);

     if(result==TRUE) {
        unreachable= FALSE;
        gpOpen_hand(robotPt, hand);

        if(p3d_col_test()) {
          continue;
        }
        else {
          genomGetArmQ(robotPt, q1, q2, q3, q4, q5, q6);
          p3d_set_and_update_this_robot_conf(robotPt, q);
          p3d_destroy_config(robotPt, q);
          return 0;
        }
    }
  }

  p3d_destroy_config(robotPt, q);

  if(unreachable==TRUE) {
    return 1;
  }
  else {
    return 2;
  }

}

void genomDraw()
{
  static int firstTime= TRUE;

//   if(firstTime==TRUE) {
//     firstTime= FALSE;
//     g3d_win *win= NULL;
//     win= g3d_get_cur_win();
//     win->fct_draw2= &(genomDraw);
//   }


//   GRASP.draw(0.05);
HAND.draw(GFRAME);

   g3d_draw_p3d_polyhedre(OBJECT->pol[0]->poly);
   draw_frame(EEFRAME, 0.05);
   draw_frame(GFRAME, 0.1);

}


#endif


