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


/* --------- FORM VARIABLES ------- */
FL_FORM  * GENOM_FORM = NULL;
static FL_OBJECT * GENOMGROUP;
static FL_OBJECT * BT_ARM_GOTO_Q_OBJ;


/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_genom_group(void);
static int genomArmGotoQ(p3d_rob* robotPt, int cartesian);

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
	BT_ARM_GOTO_Q_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Goto Q");
//	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");

	fl_set_call_back(BT_ARM_GOTO_Q_OBJ, CB_genomArmGotoQ_obj, 1);

	fl_end_group();
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
	ENV.setExpansionMethod(Env::Connect);

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

#endif
