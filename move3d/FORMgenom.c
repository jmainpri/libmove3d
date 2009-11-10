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

static double Q[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double QGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


/* --------- FORM VARIABLES ------- */
FL_FORM  * GENOM_FORM = NULL;
static FL_OBJECT * GENOMGROUP;
static FL_OBJECT * BT_SET_Q_OBJ;
static FL_OBJECT * BT_ARM_GOTO_Q_OBJ;
static FL_OBJECT  *INPUT_OBJ1, *INPUT_OBJ2;
static FL_FORM *INPUT_FORM;

/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_genom_group(void);
static int genomArmGotoQ(p3d_rob* robotPt, int cartesian);
static int genomSetArmQ(p3d_rob *robot, double q1, double q2, double q3, double q4, double q5, double q6);
static int genomGetArmConfiguration(p3d_rob *robot, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);

static void CB_genomSetQ_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmGotoQ_obj(FL_OBJECT *obj, long arg);


#ifdef MULTILOCALPATH
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
	BT_ARM_GOTO_Q_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Goto Q");
//	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
	fl_set_call_back(BT_ARM_GOTO_Q_OBJ, CB_genomArmGotoQ_obj, 1);

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
 genomSetArmQ(robotPt, QGOAL[0], QGOAL[1], QGOAL[2], QGOAL[3], QGOAL[4], QGOAL[5]);
 p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_GOTO);


 str= fl_get_input(INPUT_OBJ1);
 result= sscanf(str, "%lf %lf %lf %lf %lf %lf", &Q[0], &Q[1], &Q[2], &Q[3], &Q[4], &Q[5]);
 if(result!=6)
 {
   fl_show_question("Expected format: q1 q2 q3 q4 q5 q6\n", 1);
   printf("Expected format of Q input: q1 q2 q3 q4 q5 q6\n");
   Q[0]= Q[1]= Q[2]= Q[3]= Q[4]= Q[5]= 0.0;
 }
 printf("%lf %lf %lf %lf %lf %lf \n", Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]);


 genomSetArmQ(robotPt, Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]);


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
  genomGetArmConfiguration(robotPt, &Q[0], &Q[1], &Q[2], &Q[3], &Q[4], &Q[5]);
  p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_GOTO);
  genomGetArmConfiguration(robotPt, &QGOAL[0], &QGOAL[1], &QGOAL[2], &QGOAL[3], &QGOAL[4], &QGOAL[5]);
  p3d_set_and_update_this_robot_conf(robotPt, q0);
  p3d_set_ROBOT_START(q0);
  p3d_destroy_config(robotPt, q0);


  sprintf(str1, "%lf     %lf     %lf    %lf     %lf     %lf", Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]);
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

	genomArmGotoQ(robotPt, cartesian);
	fl_set_button(BT_ARM_GOTO_Q_OBJ,0);
	return;
}

static int genomArmGotoQ(p3d_rob* robotPt, int cartesian) {
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
		if(robotPt->nbCcCntrts!=0) {
			p3d_desactivateCntrt(robotPt, robotPt->ccCntrts[0]);
		}
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
// 	ENV.setExpansionMethod(Env::Connect);
	ENV.setExpansionMethod(Env::Extend);


	if(p3d_equal_config(robotPt, robotPt->ROBOT_POS, robotPt->ROBOT_GOTO)) {
		printf("genomArmGotoQ: Start and goal configurations are the same.\n");
		return 1;
	}
	p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
	result= p3d_specific_search("out.txt");
  // optimizes the trajectory:
	CB_start_optim_obj(NULL, 0);
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
	if(p3d_optim_traj_softMotion(traj, 1, &gain, &ntest) == 1){
		printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
		return 1;
	}
	return 0;
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
  double qmin, qmax;
  p3d_jnt *armJoint= NULL;
  configPt q= NULL;

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);

  ////////////////////////q1////////////////////////////
  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT1);
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
  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT2);
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
  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT3);
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
  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT4);
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
  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT5);
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
  armJoint= get_robot_jnt_by_name(robot, GP_WRISTJOINT);
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

  if(robot->nbCcCntrts!=0) {
    p3d_desactivateCntrt(robot, robot->ccCntrts[0]);
  }

  p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, q);

  p3d_set_and_update_this_robot_conf(robot, q);

  p3d_destroy_config(robot, q);

  if(robot->nbCcCntrts!=0) {
    p3d_activateCntrt(robot, robot->ccCntrts[0]);
  }

  return 0;
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
int genomGetArmConfiguration(p3d_rob *robot, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6)
{
  if(robot==NULL)
  {
    printf("%s: %d: genomGetArmConfiguration(): robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  p3d_jnt *armJoint= NULL;

  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT1);
  if(armJoint==NULL)
  {  return 0; }
  *q1= armJoint->dof_data[0].v;

  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT2);
  if(armJoint==NULL)
  {  return 0; }
  *q2= armJoint->dof_data[0].v;

  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT3);
  if(armJoint==NULL)
  {  return 0; }
  *q3= armJoint->dof_data[0].v;

  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT4);
  if(armJoint==NULL)
  {  return 0; }
  *q4= armJoint->dof_data[0].v;

  armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT5);
  if(armJoint==NULL)
  {  return 0; }
  *q5= armJoint->dof_data[0].v;

  armJoint= get_robot_jnt_by_name(robot, GP_WRISTJOINT);
  if(armJoint==NULL)
  {  return 0; }
  *q6= armJoint->dof_data[0].v;


  return 0;
}


#endif
