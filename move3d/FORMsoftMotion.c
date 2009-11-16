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
FL_FORM  * SOFT_MOTION_FORM = NULL;
static FL_OBJECT * BT_COMP_TRAJ_OBJ;
static FL_OBJECT * BT_PLOT_Q_WRITE_FILE_OBJ;
static FL_OBJECT * MOTIONGROUP;
static FL_OBJECT  *BT_LOAD_TRAJ_OBJ;
static FL_OBJECT  *BT_PLAY_TRAJ_OBJ;
static FL_OBJECT  *BT_TEST_OBJ;

static bool write_file= false;
static char fileTraj[128];
configPt configTraj[10000];
static int nbConfigTraj = 0;
static char file_directory[512];
static int PLOT_Q_ARM = 0;

/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_soft_motion_group(void);
static void draw_trajectory_ptp();

static void CB_softMotion_compute_traj_obj(FL_OBJECT *obj, long arg);
static void CB_softMotion_write_file_obj(FL_OBJECT *obj, long arg);
static void CB_load_traj_obj(FL_OBJECT *obj, long arg);
static void CB_play_traj_obj(FL_OBJECT *obj, long arg);
static void CB_test_obj(FL_OBJECT *obj, long arg);

static int NB_TRAJPTP_CONFIG= 0;
static configPt TRAJPTP_CONFIG[200];

#ifdef MULTILOCALPATH
/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_soft_motion_form(void) {
	SOFT_MOTION_FORM = fl_bgn_form(FL_UP_BOX, 150, 340);
	g3d_create_soft_motion_group();
	fl_end_form();
}

void g3d_show_soft_motion_form(void) {
	fl_show_form(SOFT_MOTION_FORM, FL_PLACE_SIZE, TRUE, "Soft Motion");
}

void g3d_hide_soft_motion_form(void) {
	fl_hide_form(SOFT_MOTION_FORM);
}

void g3d_delete_soft_motion_form(void) {
	fl_free_form(SOFT_MOTION_FORM);
}

/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_soft_motion_group(void) {
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 310, "Soft Motion");

	MOTIONGROUP = fl_bgn_group();

	x= 15;
	y= 30;
	w= 120;
	h= 40;
	dy= h + 10;
	BT_COMP_TRAJ_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Compute softMotion traj");
	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
	BT_LOAD_TRAJ_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "Load qi Trajectory");
	BT_PLAY_TRAJ_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 3*dy, w, h, "Play qi Trajectory");
	BT_TEST_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 4*dy, w, h, "Test");

	fl_set_call_back(BT_COMP_TRAJ_OBJ, CB_softMotion_compute_traj_obj, 1);

	fl_set_call_back(BT_PLOT_Q_WRITE_FILE_OBJ, CB_softMotion_write_file_obj, 1);
	fl_set_object_color(BT_PLOT_Q_WRITE_FILE_OBJ,FL_MCOL,FL_GREEN);
	fl_set_button(BT_PLOT_Q_WRITE_FILE_OBJ, FALSE);

	fl_set_call_back(BT_LOAD_TRAJ_OBJ, CB_load_traj_obj, 1);
	fl_set_call_back(BT_PLAY_TRAJ_OBJ, CB_play_traj_obj, 1);
	fl_set_call_back(BT_TEST_OBJ, CB_test_obj, 1);

	fl_end_group();
}

static void CB_softMotion_compute_traj_obj(FL_OBJECT *ob, long arg) {
	void (*fct_draw)(void);
	p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	int ir = p3d_get_desc_curnum(P3D_ROBOT);
	int ntest=0;
	double gain,gaintot=1.;


	if(!traj) {
		printf("Soft Motion : ERREUR : no current traj\n");
		return ;
	}

	fct_draw = &(g3d_draw_allwin_active);


	if(!traj || traj->nlp < 1) {
		if(ob){//if is not a automatic call
			printf("Optimization not possible: current trajectory\
					contains one or zero local path\n");
		}
		fl_set_button(BT_COMP_TRAJ_OBJ,0);
		return;
	}
	if(ob){fl_set_cursor(FL_ObjWin(ob), XC_watch);}

	fct_draw = &(g3d_draw_allwin_active);

	if(p3d_optim_traj_softMotion(traj, write_file, &gain, &ntest)){
		gaintot = gaintot*(1.- gain);
		/* position the robot at the beginning of the optimized trajectory */
		position_robot_at_beginning(ir, traj);
	}
	if (fct_draw){(*fct_draw)();}

	g3d_win *win= NULL;
	win= g3d_get_cur_win();
	win->fct_draw2 = &(draw_trajectory_ptp);

	g3d_draw_allwin_active();
	if(ob){fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);fl_set_button(ob,0);}
	return;

}

static void CB_softMotion_write_file_obj(FL_OBJECT *obj, long arg) {
	write_file= !write_file;

	if(write_file)
	{  fl_set_button(BT_PLOT_Q_WRITE_FILE_OBJ, TRUE);  }
	else
	{  fl_set_button(BT_PLOT_Q_WRITE_FILE_OBJ, FALSE); }

}

static void CB_test_obj(FL_OBJECT *ob, long arg) {
	if(ob){fl_set_cursor(FL_ObjWin(ob), XC_watch);}



	if(ob){fl_set_cursor(FL_ObjWin(ob), FL_DEFAULT_CURSOR);fl_set_button(ob,0);}
}

void sm_set_PLOT_Q_ARM(int value) {
	if(value == 1) {
		PLOT_Q_ARM = 1;
	} else {
		PLOT_Q_ARM = 0;
	}
	return;
}

void draw_trajectory_ptp() {
	int i;
	p3d_vector3 p1;
	p3d_vector3 p2;
	if(TRAJPTP_CONFIG[0]==NULL || NB_TRAJPTP_CONFIG<=0)
	{  return;  }

// 	g3d_set_color_mat(Red, NULL);
// 	for(i=0; i<nb_configs; i++)
// 	{  gpDraw_solid_sphere(configs[i][6], configs[i][7], 1.0, 0.07, 10);  }

// 	g3d_set_color_mat(Green, NULL);
	//
// 	glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT);
// 	glBegin(GL_LINES);
// 	for(i=1; i<NB_TRAJPTP_CONFIG; i++)
// 	{
// 		glVertex3d(TRAJPTP_CONFIG[i-1][21], TRAJPTP_CONFIG[i-1][22], TRAJPTP_CONFIG[i-1][23]);
// 		glVertex3d(TRAJPTP_CONFIG[i][21], TRAJPTP_CONFIG[i][22], TRAJPTP_CONFIG[i][23]);
// 	}
// 	glEnd();
// 	glPopAttrib();

	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	glColor3f(0,1,0);
	for(i=1; i<NB_TRAJPTP_CONFIG; i++) {
		for(int j=0; j<3; j++) {
			p1[j] = TRAJPTP_CONFIG[i-1][21 +j];
			p2[j] = TRAJPTP_CONFIG[i][21 +j];
		}
		g3d_draw_cylinder(p1, p2, 0.0005, 16);
	}
	glPopAttrib();
}

int p3d_optim_traj_softMotion(p3d_traj *trajPt, int param_write_file, double *gain, int *ntest) {
	p3d_rob *robotPt = trajPt->rob;
	p3d_traj *trajSmPTPPt = NULL;
	p3d_traj *trajSmPt = NULL;
	p3d_localpath *end_trajSmPt = NULL;

	p3d_localpath *localpathMlp1Pt = trajPt->courbePt;
	p3d_localpath *localpathMlp2Pt = localpathMlp1Pt->next_lp;

	p3d_localpath *localpath1Pt = NULL;
	p3d_localpath *localpath2Pt = NULL;
	p3d_localpath *localpathTransPt = NULL;
	p3d_localpath *localpathTmp1Pt = NULL;
	p3d_localpath *localpathTmp2Pt = NULL;

	configPt q1 = NULL, q2 = NULL, qinit = NULL, qgoal = NULL;
	configPt q_init = NULL, q_end = NULL;

	double ltot = 0.0;
	double cost = 0.0;
	int firstLpSet = 0;
	int nlp = 0, iGraph=0;
	int *iksol=NULL;
	p3d_softMotion_data* softMotion_data_lp1 = NULL;
	p3d_softMotion_data* softMotion_data_lp2 = NULL;
	p3d_softMotion_data* softMotion_data_lpTrans = NULL;

		// To save the traj into a file
	int IGRAPH_JIDO_OB = 0;
	int IGRAPH_JIDO_OB_LIN = 0;
	int IGRAPH_JIDO_ARM_LIN = 0;
	int IGRAPH_JIDO_ARM = 0;
	int IGRAPH_INPUT = 0;
	int IGRAPH_OUTPUT = 0;
	/* length of trajPt */
	ltot = p3d_ends_and_length_traj(trajPt, &qinit, &qgoal);
	if (ltot<= 3*EPS6) {
		/* trajectory too short */
		p3d_destroy_config(robotPt, qinit);
		p3d_destroy_config(robotPt, qgoal);
		return FALSE;
	}

		// Find the groups ID
	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-ob_lin") == 0) {
			IGRAPH_JIDO_OB_LIN = iGraph;
		}
	}

	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-ob") == 0) {
			IGRAPH_JIDO_OB = iGraph;
		}
	}

	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-arm_lin") == 0) {
			IGRAPH_JIDO_ARM_LIN = iGraph;
		}
	}

	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-arm") == 0) {
			IGRAPH_JIDO_ARM = iGraph;
		}
	}

		// Clear static variable
	if (TRAJPTP_CONFIG[0]!=NULL) {
		for(int i=0; i<NB_TRAJPTP_CONFIG; i++) {
			p3d_destroy_config(robotPt, TRAJPTP_CONFIG[i]);
			TRAJPTP_CONFIG[i] = NULL;
		}
	}
	NB_TRAJPTP_CONFIG = 0;

int nbNonNullLp = 0;
	///////////////////////////////////////////////////////////////////////////
	////  CONVERT LINEAR TRAJECTORY TO SOFTMOTION POINT TO POINT TRAJECTORY ///
	///////////////////////////////////////////////////////////////////////////

	/* Create the softMotion trajectory */
	trajSmPTPPt = p3d_create_empty_trajectory(robotPt);

	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_JIDO_OB_LIN];
	if(localpath1Pt!=NULL) {
		IGRAPH_INPUT = IGRAPH_JIDO_OB_LIN;
		IGRAPH_OUTPUT = IGRAPH_JIDO_OB;
		nbNonNullLp ++;
	}
	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_JIDO_OB];
	if(localpath1Pt!=NULL) {
		IGRAPH_INPUT = IGRAPH_JIDO_OB;
		IGRAPH_OUTPUT = IGRAPH_JIDO_OB;
		nbNonNullLp ++;
	}
	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_JIDO_ARM_LIN];
	if(localpath1Pt!=NULL) {
		IGRAPH_INPUT = IGRAPH_JIDO_ARM_LIN;
		IGRAPH_OUTPUT = IGRAPH_JIDO_ARM;
		nbNonNullLp ++;
	}
	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_JIDO_ARM];
	if(localpath1Pt!=NULL) {
		IGRAPH_INPUT = IGRAPH_JIDO_ARM;
		IGRAPH_OUTPUT = IGRAPH_JIDO_ARM;
		nbNonNullLp ++;
	}

	if(nbNonNullLp != 1) {
		printf("softMotion compute traj ERROR non null lp !=1\n");
		return FALSE;
	}
	p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
	p3d_multiLocalPath_set_groupToPlan(robotPt, IGRAPH_OUTPUT, 1);

	q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
	q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);

// 	p3d_desactivateAllCntrts(robotPt);
// 	for(int i = 0; i < localpathMlp1Pt->nbActiveCntrts; i++){
// 		p3d_activateCntrt(robotPt, robotPt->cntrt_manager->cntrts[localpathMlp1Pt->activeCntrts[i]]);
// 	}

	trajSmPTPPt->courbePt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->mlpLocalpath[IGRAPH_INPUT]->ikSol);

	trajSmPTPPt->courbePt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
	trajSmPTPPt->courbePt->mlpLocalpath[IGRAPH_OUTPUT]->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
	for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
		trajSmPTPPt->courbePt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
		trajSmPTPPt->courbePt->mlpLocalpath[IGRAPH_OUTPUT]->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
	}

	end_trajSmPt = trajSmPTPPt->courbePt;
	TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_init);
	NB_TRAJPTP_CONFIG ++;
	TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_end);
	NB_TRAJPTP_CONFIG ++;
	localpathMlp1Pt = localpathMlp1Pt->next_lp;

	while(localpathMlp1Pt != NULL){
		localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_INPUT];
		if(localpath1Pt!= NULL) {
			q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
			q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);
			localpathTmp1Pt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->mlpLocalpath[IGRAPH_INPUT]->ikSol);

			localpathTmp1Pt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
			localpathTmp1Pt->mlpLocalpath[IGRAPH_OUTPUT]->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
			for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
				localpathTmp1Pt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
				localpathTmp1Pt->mlpLocalpath[IGRAPH_OUTPUT]->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
			}

			TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_end);
			NB_TRAJPTP_CONFIG ++;
			end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
		}
		localpathMlp1Pt = localpathMlp1Pt->next_lp;
	} // END WHILE (localpathMlp1Pt != NULL)



	/* update the number of local paths */
	trajSmPTPPt->nlp = p3d_compute_traj_nloc(trajSmPTPPt);
	trajSmPTPPt->range_param = p3d_compute_traj_rangeparam(trajSmPTPPt);
	g3d_add_traj((char*)"traj_SoftMotion_PTP", trajSmPTPPt->num);

	/////////////////////////////////////////////////////
	////  CREATE TRAJECTORY WITH REMOVED HALT AT NODES///
	/////////////////////////////////////////////////////


	/////////////////////////////////
	//////  INITIALIZE VARIABLES  ////
	//////////////////////////////////
	if(softMotion_data_lp1 == NULL) {
		softMotion_data_lp1 = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
	}
	if(softMotion_data_lp2 == NULL) {
		softMotion_data_lp2 = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
	}
	if(softMotion_data_lpTrans == NULL) {
		softMotion_data_lpTrans = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
	}

	/* Create the softMotion trajectory */
	trajSmPt = p3d_create_empty_trajectory(robotPt);
	localpathMlp1Pt = trajSmPTPPt->courbePt;
	localpathMlp2Pt = trajSmPTPPt->courbePt->next_lp;
	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];



	softMotion_data_copy_into(robotPt, localpath1Pt->specific.softMotion_data, softMotion_data_lp1);

	for(int v = 0; v<softMotion_data_lp1->nbDofs; v++) {
		softMotion_data_lp1->specific->velInit[v] = 0.0;
		softMotion_data_lp1->specific->velEnd[v] = 0.0;
		softMotion_data_lp1->specific->accInit[v] = 0.0;
		softMotion_data_lp1->specific->accEnd[v] = 0.0;
	}

		/*
	* There are three localpath like xarm module on Jido (see IROS08 paper "Soft Motion Trajectory Planner For Service Manipulator Robot")
	* The one localpathTmp1Pt is the first motion, localpathTmpTrans is the the transition motion, localpathTmp2Pt is the third motion
		*/
	nlp = 0;
	firstLpSet = 0;

	int collision = FALSE;
	/* We add the three fisrt segment to the trajectory */

	trajSmPt->courbePt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt, 0.0, localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);
	end_trajSmPt = trajSmPt->courbePt;

	while(localpathMlp1Pt != NULL) {
		localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];
		q1 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_init);
		q2 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_end);

		if (localpathMlp1Pt->next_lp == NULL) {
			/* It's the last localpath */



				if(nlp == 0) {
					localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
							(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
							 (double)localpath1Pt->specific.softMotion_data->specific->motionTime);
				} else {
					localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
							(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
							 (double)localpath1Pt->specific.softMotion_data->specific->motionTime);
				}
			end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);

		} else {
			localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
					(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
					 (double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);

			softMotion_data_copy_into(robotPt, localpathTmp1Pt->specific.softMotion_data, softMotion_data_lp1);

			localpathMlp2Pt = localpathMlp1Pt->next_lp;
			localpath2Pt = localpathMlp2Pt->mlpLocalpath[IGRAPH_OUTPUT];
			localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt,
					(double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
					 (double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);

			softMotion_data_copy_into(robotPt, localpathTmp2Pt->specific.softMotion_data, softMotion_data_lp2);

			/* Set Transition motion */
			if(softMotion_data_lpTrans == NULL) {
				softMotion_data_lpTrans = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
			}

			softMotion_data_lpTrans->q_init = localpath1Pt->config_at_distance(robotPt, localpath1Pt, (double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);
			softMotion_data_lpTrans->q_end =  localpath2Pt->config_at_distance(robotPt, localpath2Pt, (double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);

			/* Jmax, Amax and Vmax must have the same ratio between lpTmp1 and lpTmp2 */
			for(int v=0; v<softMotion_data_lp1->nbDofs; v++) {
			softMotion_data_lpTrans->specific->J_max[v] = MAX(softMotion_data_lp1->specific->J_max[v], softMotion_data_lp2->specific->J_max[v]);
			softMotion_data_lpTrans->specific->A_max[v] = MAX(softMotion_data_lp1->specific->A_max[v], softMotion_data_lp2->specific->A_max[v]);
			softMotion_data_lpTrans->specific->V_max[v] = MAX(softMotion_data_lp1->specific->V_max[v], softMotion_data_lp2->specific->V_max[v]);

			softMotion_data_lpTrans->specific->velInit[v] = softMotion_data_lp1->specific->motion[v].FC.v;
			softMotion_data_lpTrans->specific->velEnd[v] = softMotion_data_lp2->specific->motion[v].IC.v;
			softMotion_data_lpTrans->specific->accInit[v] = softMotion_data_lp1->specific->motion[v].FC.a;
			softMotion_data_lpTrans->specific->accEnd[v] = softMotion_data_lp2->specific->motion[v].IC.a;
			}

			q1 = localpath1Pt->config_at_distance(robotPt, localpath1Pt, (double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);
			q2 =  localpath2Pt->config_at_distance(robotPt, localpath2Pt, (double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);

			softMotion_data_lpTrans->isPTP = FALSE;
			localpathTransPt = p3d_softMotion_localplanner(robotPt, IGRAPH_OUTPUT, softMotion_data_lpTrans, q1, q2, q2, localpath1Pt->ikSol);

			if(localpathTransPt != NULL){
				collision = p3d_unvalid_localpath_test(robotPt, localpathTransPt, ntest);
			}
			if(localpathTransPt==NULL || collision == TRUE) {

				printf("localpathTransPt : stop motion localpath = %p , collision = %d\n", localpathTransPt,collision);


				localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
				/* We add the both original localpaths (with stop motion) */
				localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
						(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
						 (double)localpath1Pt->specific.softMotion_data->specific->motionTime);
				localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt, 0.0,
						(double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);

				end_trajSmPt  = append_to_localpath(end_trajSmPt , localpathTmp1Pt);
				nlp++;
				end_trajSmPt  = append_to_localpath(end_trajSmPt , localpathTmp2Pt);
				nlp++;

				collision = FALSE;
			} else {
				localpathTransPt->nbActiveCntrts = localpath1Pt->nbActiveCntrts;
				for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
					localpathTransPt->activeCntrts[v] = localpath1Pt->activeCntrts[v];
					localpathTransPt->activeCntrts[v] = localpath1Pt->activeCntrts[v];
				}
				/* Transition motion is OK */
				end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
				nlp++;
				end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTransPt);
				nlp++;
			}
		}

		cost += end_trajSmPt->cost(robotPt, end_trajSmPt);
		localpathMlp1Pt = localpathMlp1Pt->next_lp;
		localpathTmp2Pt = NULL;
		localpathTmp1Pt = NULL;
		softMotion_data_lpTrans = NULL;

	}



	/* update the number of local paths */
	trajSmPt->nlp = p3d_compute_traj_nloc(trajSmPt);
	trajSmPt->range_param = p3d_compute_traj_rangeparam(trajSmPt);
	g3d_add_traj((char*)"traj_SoftMotion", trajSmPt->num);


	/* Write curve into a file for BLTPLOT */
	if(param_write_file == true) {
	p3d_softMotion_write_curve_for_bltplot(robotPt, trajSmPt, "RefSM.dat", PLOT_Q_ARM) ;
	}


	robotPt->tcur = trajSmPt;
	FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
	return FALSE;
}

static int read_trajectory(p3d_rob* robotPt, FILE *fileptr) {
	nbConfigTraj = 0;
	int index_dof = 11;

	if(nbConfigTraj>0) {
		for(int v=nbConfigTraj-1; v>= 0; v--) {
			p3d_destroy_config(robotPt,configTraj[v]);
		}
		nbConfigTraj = 0;
	}
	/* Read File Variables */
	while(!feof(fileptr)) {
		configTraj[nbConfigTraj] = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
		fscanf(fileptr, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
					 &configTraj[nbConfigTraj][index_dof],&configTraj[nbConfigTraj][index_dof+1],&configTraj[nbConfigTraj][index_dof+2],
			&configTraj[nbConfigTraj][index_dof+3],&configTraj[nbConfigTraj][index_dof+4],&configTraj[nbConfigTraj][index_dof+5]);
		nbConfigTraj ++;
		if (nbConfigTraj>= 10000) {
			printf("File too long\n");

			return TRUE;
		}
	}
	return FALSE;
}

static void read_trajectory_by_name(p3d_rob* robotPt, const char *file) {
	FILE *fdc;
	int ret;
	/* on lit la trajectoire */
		if (fileTraj){
				if(!(fdc=fopen(fileTraj,"r"))) {
					PrintError(("p3d_rw_scenario_read: can't open %s\n",file));
					return;
				}
				ret = read_trajectory(robotPt, fdc);
				fclose(fdc);
		}
	return;
}

static void CB_load_traj_obj(FL_OBJECT *ob, long arg) {
	double qplot[6][10000];
	gnuplot_ctrl * h = NULL;
	int index = 0, v=0, j=0, i=0;
	const char*file = NULL;
	FILE * f = NULL;
	p3d_rob* robotPt= NULL;
	/* lecture du fichier environnement */

	int r, nr;

	r = p3d_get_desc_curnum(P3D_ROBOT);
	nr= p3d_get_desc_number(P3D_ROBOT);

	for(i=0; i<nr; i++){
		robotPt= (p3d_rob *) p3d_sel_desc_num(P3D_ROBOT, i);
		if(strcmp("ROBOT", robotPt->name)==0){
			break;
		}
	}




	p3d_set_directory(file_directory);
	file	= fl_show_fselector("Trajectory filename", file_directory,	"*.traj", "");
	if(file == NULL) {
		printf("no file to load\n");
		return;
	}
	strcpy(fileTraj, file);
	read_trajectory_by_name(robotPt, fileTraj);
	p3d_desactivateAllCntrts(robotPt);
	index = 0;
	for(i=0; i<nbConfigTraj; i++) {
		p3d_set_and_update_this_robot_conf(robotPt, configTraj[i]);
		g3d_draw_allwin_active();
		j = 0;
		for(v=11; v<(11+6); v++) {
			qplot[j][index] = configTraj[i][v];
			j++;
		}
		index = index + 1;
	}
	f = fopen("temp.dat","w");
	for(i=0; i<index; i++){
		fprintf(f,"%d %f %f %f %f %f %f\n",	i, qplot[0][i],qplot[1][i],qplot[2][i],qplot[3][i],qplot[4][i],qplot[5][i]);
	}
	fclose(f);
	h = gnuplot_init();
	if(h == NULL){
		printf("Gnuplot Init problem");
	}
	gnuplot_cmd(h,(char*)"set term wxt");
	gnuplot_cmd(h,(char*)"set xrange [%d:%d]",0,index-1);
	gnuplot_cmd(h,(char*)"set yrange [-pi:pi]");
	gnuplot_cmd(h, (char*)"plot '%s' using 1:2 with lines lt 1 ti \"q1\", '%s' using 1:3 with lines lt 2 ti \"q2\" , '%s' using 1:4 with lines lt 3 ti \"q3\",'%s' using 1:5 with lines lt 4 ti \"q4\", '%s' using 1:6 with lines lt 5 ti \"q5\", '%s' using 1:7 with lines lt 6 ti \"q6\" " , "temp.dat", "temp.dat", "temp.dat", "temp.dat", "temp.dat", "temp.dat");

	fl_set_button(BT_LOAD_TRAJ_OBJ,0);
}

static void CB_play_traj_obj(FL_OBJECT *ob, long arg) {
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

	p3d_desactivateAllCntrts(robotPt);
	for(int i=0; i<nbConfigTraj; i++) {
		p3d_set_and_update_this_robot_conf(robotPt, configTraj[i]);
		g3d_draw_allwin_active();
	}
	fl_set_button(BT_PLAY_TRAJ_OBJ,0);
}


#endif

