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
static bool write_file= false;

/* ---------- FUNCTION DECLARATIONS --------- */
static void g3d_create_soft_motion_group(void);

static void draw_trajectory_ptp();

static void CB_softMotion_compute_traj_obj(FL_OBJECT *obj, long arg);
static void CB_softMotion_write_file_obj(FL_OBJECT *obj, long arg);

static int NB_TRAJPTP_CONFIG= 0;
static configPt TRAJPTP_CONFIG[200];

#ifdef MULTILOCALPATH
/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_soft_motion_form(void)
{
	SOFT_MOTION_FORM = fl_bgn_form(FL_UP_BOX, 150, 240);
	g3d_create_soft_motion_group();
	fl_end_form();
}

void g3d_show_soft_motion_form(void)
{
	fl_show_form(SOFT_MOTION_FORM, FL_PLACE_SIZE, TRUE, "Soft Motion");
}

void g3d_hide_soft_motion_form(void)
{
	fl_hide_form(SOFT_MOTION_FORM);
}

void g3d_delete_soft_motion_form(void)
{
	fl_free_form(SOFT_MOTION_FORM);
}


/* -------------------- MAIN GROUP --------------------- */
static void g3d_create_soft_motion_group(void)
{
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 210, "Soft Motion");

	MOTIONGROUP = fl_bgn_group();

	x= 15;
	y= 30;
	w= 120;
	h= 40;
	dy= h + 10;
	BT_COMP_TRAJ_OBJ = fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Compute softMotion traj");
	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
//	BT_ARM_ONLY_OBJ= fl_add_button(FL_NORMAL_BUTTON, x, y + 2*dy, w, h, "Grasp the object");

	fl_set_call_back(BT_COMP_TRAJ_OBJ, CB_softMotion_compute_traj_obj, 1);

	fl_set_call_back(BT_PLOT_Q_WRITE_FILE_OBJ, CB_softMotion_write_file_obj, 1);
	fl_set_object_color(BT_PLOT_Q_WRITE_FILE_OBJ,FL_MCOL,FL_GREEN);
	fl_set_button(BT_PLOT_Q_WRITE_FILE_OBJ, FALSE);


	fl_end_group();
}

static void CB_softMotion_compute_traj_obj(FL_OBJECT *ob, long arg)
{
	void (*fct_draw)(void);
	p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	int ir = p3d_get_desc_curnum(P3D_ROBOT);
	int i, ntest=0, nb_optim=0;
	double gain,gaintot=1., epsilon = 0.0;


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

	if(p3d_optim_traj_softMotion(traj,&gain, &ntest)){
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

static void CB_softMotion_write_file_obj(FL_OBJECT *obj, long arg)
{
	write_file= !write_file;

	if(write_file)
	{  fl_set_button(BT_PLOT_Q_WRITE_FILE_OBJ, TRUE);  }
	else
	{  fl_set_button(BT_PLOT_Q_WRITE_FILE_OBJ, FALSE); }

}

void draw_trajectory_ptp()
{
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

int p3d_optim_traj_softMotion(p3d_traj *trajPt, double *gain, int *ntest) {
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
	if(write_file == true) {
	p3d_softMotion_write_curve_for_bltplot(robotPt, trajSmPt, "RefSM.dat") ;
	}


	robotPt->tcur = trajSmPt;
	FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
	return FALSE;
}


/*
  	if(trajPt->nlp < 2) {

} else { // IF (trajPt->nlp < 2)

} // END ELSE IF (trajPt->nlp < 2)

}*/

// int p3d_optim_traj_softMotion(p3d_traj *trajPt, double *gain, int *ntest) {
//   	p3d_rob *robotPt = trajPt->rob;
// 		p3d_traj *trajSmPt = NULL;
//
//


//  		for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
// 			if(robotPt->mlp->mlpJoints[iGraph]->gpType == FREEFLYER) {
// 				if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-ob_lin") == 0) {
// 					/* There are three localpath like xarm module on Jido (see IROS08 paper "Soft Motion Trajectory Planner For Service Manipulator Robot")
// 				   * The one localpathTmp1Pt is the first motion, localpathTmpTrans is the the transition motion, localpathTmp2Pt is the third motion
// 					 */
// 					////////////////////////////////
// 					////  INITIALIZE VARIABLES  ////
// 					////////////////////////////////
// 					{
// 					if(softMotion_data_lp1 == NULL) {
// 						softMotion_data_lp1 = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, iGraph);
// 					}
// 					if(softMotion_data_lp2 == NULL) {
// 						softMotion_data_lp2 = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, iGraph);
// 					}
// 					if(softMotion_data_lpTrans == NULL) {
// 						softMotion_data_lpTrans = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, iGraph);
// 					}
// 					}
// 					///////////////////////////////////////
// 					////  COMPUTE THE FIRST LOCALPATH  ////
// 					///////////////////////////////////////
// 					localpath1Pt = localpathMlp1Pt->mlpLocalpath[iGraph];
// 					if (localpath1Pt->type_lp != LINEAR){
// 						PrintError(("p3d_optim_traj_softMotion: local path must be linear\n"));
// 						return NULL;
// 					}
// 					q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
// 					q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);
// 					localpath1SmPt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->mlpLocalpath[iGraph]->ikSol);
//
// 					softMotion_data_copy_into(robotPt, localpath1SmPt->mlpLocalpath[iGraph]->specific.softMotion_data, softMotion_data_lp1);
// 					Gb_v3_set( &softMotion_data_lp1->freeflyer->velLinInit, 0.0, 0.0, 0.0);
// 					Gb_v3_set( &softMotion_data_lp1->freeflyer->velAngInit, 0.0, 0.0, 0.0);
// 					softMotion_params = lm_get_softMotion_lm_param_multilocalpath(robotPt, iGraph);
//
// 					nlp = 0;
// 					firstLpSet = 0;
//
// // 					/* We add the three fisrt segment to the trajectory */
// // 					trajSmPt->courbePt = p3d_extract_softMotion_with_velocities(robotPt, localpath1SmPt, 0.0, (double)localpath1SmPt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3]);
// //  					end_trajSmPt = trajSmPt->courbePt;
//
// 					///////////////////////////////
// 					////  GLOBAL WHILE LOOP    ////
// 					///////////////////////////////
// 				  while(localpathMlp1Pt != NULL){
//
// 						localpathTmp1Pt =  NULL;
// 						localpathTmp2Pt =  NULL;
//
// 						if (localpathMlp1Pt->next_lp == NULL) {
// 							/* It's the last localpath */
// 							if(nlp == 0) {
// 								/* Extract the wole localpath */
// 								// TODO COPY THE LOCALPATH
// 								localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1SmPt, 0.0,
// 										(double)localpath1SmPt->mlpLocalpath[iGraph]->specific.softMotion_data->freeflyer->motionTime);
// 							}
//
// 						} else {  //if (localpathMlp1Pt->next_lp == NULL)
//
//
// 							///////////////////////////////////////
// 							////   EXTRACT THE FIRST LOCALPATH  ///
// 							///////////////////////////////////////
// 							localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1SmPt, 0.0,
// 									(double)localpath1SmPt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
//
// 							///////////////////////////////////////
// 							////  COMPUTE THE SECOND LOCALPATH  ///
// 							///////////////////////////////////////
// 							localpathMlp2Pt = localpathMlp1Pt->next_lp;
// 							q_init = localpathMlp2Pt->config_at_param(robotPt, localpathMlp2Pt, 0.0);
// 							q_end = localpathMlp2Pt->config_at_param(robotPt, localpathMlp2Pt, localpathMlp2Pt->length_lp);
// 							localpath2SmPt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp2Pt->mlpLocalpath[iGraph]->ikSol);
//
// 							softMotion_data_copy_into(robotPt, localpath2SmPt->mlpLocalpath[iGraph]->specific.softMotion_data, softMotion_data_lp2);
// 							Gb_v3_set( &softMotion_data_lp2->freeflyer->velLinInit, 0.0, 0.0, 0.0);
// 							Gb_v3_set( &softMotion_data_lp2->freeflyer->velAngInit, 0.0, 0.0, 0.0);
// 							softMotion_params = lm_get_softMotion_lm_param_multilocalpath(robotPt, iGraph);
//
//
// 							localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// 										(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// 										(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// 							softMotion_data_copy_into(robotPt, localpathTmp1Pt->specific.softMotion_data, softMotion_data_lp1);
//
// 									localpath2Pt = localpath1Pt->next_lp;
//
// 									localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt,
// 										(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// 										(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// 									softMotion_data_copy_into(robotPt, localpathTmp2Pt->specific.softMotion_data, softMotion_data_lp2);
//
// 							/////////////////////////////////////////////////////////////
// 							// Compare the iksol betwwen the two adjacent localpath :  //
// 							/////////////////////////////////////////////////////////////
// 							if(p3d_compare_iksol(robotPt->cntrt_manager, localpathMlp1Pt->ikSol, localpathMlp1Pt->next_lp->ikSol) == TRUE) {
// 								//iksol are equal so it's possbile to compute a transition motion
// 								// printf("p3d_optim_traj_softMotion: iksol are equal\n");
// 								localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
// 								localpathTmp2Pt->destroy(robotPt, localpathTmp2Pt);
//
// 								////////////////////////////////
// 								// Compute Transition Motion  //
// 								////////////////////////////////
// // 									localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// // 										(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 										(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// // 									softMotion_data_copy_into(robotPt, localpathTmp1Pt->specific.softMotion_data, softMotion_data_lp1);
// 								//
// // 									localpath2Pt = localpath1Pt->next_lp;
// 								//
// // 									localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt,
// // 										(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 										(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// // 									softMotion_data_copy_into(robotPt, localpathTmp2Pt->specific.softMotion_data, softMotion_data_lp2);
//
// 								/* Set Transition motion */
// 								softMotion_data_lpTrans->q_init = localpath1SmPt->mlpLocalpath[iGraph]->config_at_distance(robotPt, localpath1SmPt->mlpLocalpath[iGraph], softMotion_data_lp1->freeflyer->motion.TimeCumulM[0][4]);
//
// 								softMotion_data_lpTrans->q_end = localpath2SmPt->mlpLocalpath[iGraph]->config_at_distance(robotPt, localpath2SmPt->mlpLocalpath[iGraph], softMotion_data_lp2->freeflyer->motion.TimeCumulM[0][3]);
//
//
// 									/* Jmax, Amax and Vmax must have the same ratio between lpTmp1 and lpTmp2 */
// 									softMotion_data_lpTrans->freeflyer->J_max_lin = MAX(softMotion_data_lp1->freeflyer->J_max_lin, softMotion_data_lp2->freeflyer->J_max_lin);
// 									softMotion_data_lpTrans->freeflyer->A_max_lin = MAX(softMotion_data_lp1->freeflyer->A_max_lin, softMotion_data_lp2->freeflyer->A_max_lin);
// 									softMotion_data_lpTrans->freeflyer->V_max_lin = MAX(softMotion_data_lp1->freeflyer->V_max_lin, softMotion_data_lp2->freeflyer->V_max_lin);
// 									softMotion_data_lpTrans->freeflyer->J_max_ang = MAX(softMotion_data_lp1->freeflyer->J_max_ang, softMotion_data_lp2->freeflyer->J_max_ang);
// 									softMotion_data_lpTrans->freeflyer->A_max_ang = MAX(softMotion_data_lp1->freeflyer->A_max_ang, softMotion_data_lp2->freeflyer->A_max_ang);
// 									softMotion_data_lpTrans->freeflyer->V_max_ang = MAX(softMotion_data_lp1->freeflyer->V_max_ang, softMotion_data_lp2->freeflyer->V_max_ang);
//
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseLinInit), softMotion_data_lp1->freeflyer->motion.FC[0].x,
// 														softMotion_data_lp1->freeflyer->motion.FC[1].x,
// 														softMotion_data_lp1->freeflyer->motion.FC[2].x);
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseLinEnd), softMotion_data_lp2->freeflyer->motion.IC[0].x,
// 																		softMotion_data_lp2->freeflyer->motion.IC[1].x,
// 																			softMotion_data_lp2->freeflyer->motion.IC[2].x);
//
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velLinInit), softMotion_data_lp1->freeflyer->motion.FC[0].v,
// 														softMotion_data_lp1->freeflyer->motion.FC[1].v,
// 															softMotion_data_lp1->freeflyer->motion.FC[2].v);
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velLinEnd), softMotion_data_lp2->freeflyer->motion.IC[0].v,
// 														softMotion_data_lp2->freeflyer->motion.IC[1].v,
// 															softMotion_data_lp2->freeflyer->motion.IC[2].v);
//
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseAngInit), softMotion_data_lp1->freeflyer->motion.FC[3].x,
// 														softMotion_data_lp1->freeflyer->motion.FC[4].x,
// 															softMotion_data_lp1->freeflyer->motion.FC[5].x);
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseAngEnd), softMotion_data_lp2->freeflyer->motion.IC[3].x,
// 														softMotion_data_lp2->freeflyer->motion.IC[4].x,
// 														softMotion_data_lp2->freeflyer->motion.IC[5].x);
//
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velAngInit), softMotion_data_lp1->freeflyer->motion.FC[3].v,
// 														softMotion_data_lp1->freeflyer->motion.FC[4].v,
// 															softMotion_data_lp1->freeflyer->motion.FC[5].v);
// 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velAngEnd), softMotion_data_lp2->freeflyer->motion.IC[3].v,
// 														softMotion_data_lp2->freeflyer->motion.IC[4].v,
// 														softMotion_data_lp2->freeflyer->motion.IC[5].v);
//
// 									q1 = localpathTmp1Pt->config_at_distance(robotPt, localpathTmp1Pt, (double)localpathTmp1Pt->specific.softMotion_data->freeflyer->motionTime);
// 									q2 = localpathTmp2Pt->config_at_distance(robotPt, localpathTmp2Pt, 0.0);
// 									q3 = localpathTmp2Pt->config_at_distance(robotPt, localpathTmp2Pt, (double)localpathTmp2Pt->specific.softMotion_data->freeflyer->motionTime);
//
// 									//****************************************************************************************************
// 									/* Compute the transition softMotion */
// 									localpathTransPt = p3d_softMotion_localplanner(robotPt, iGraph, softMotion_data_lpTrans, q1, q2, q3, iksol);
// 									//***************************************************************************************************
//
// 									if(localpathTransPt==NULL) {
// 										printf("localpathTmp1Pt==NULL\n");
// 										localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
// 										localpathTmp2Pt->destroy(robotPt, localpathTmp2Pt);
// 																		/* We add the both original localpaths (with stop motion) */
// 										localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// 												(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// 												(double)localpath1Pt->specific.softMotion_data->freeflyer->motionTime);
// 										localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt, 0.0,
// 												(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3]);
// 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
// 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp2Pt);
//
// 									} else {
// 																		/* Transition motion is OK */
// 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
// 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTransPt);
// 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp2Pt);
// 									}
// 								}
//
//
//
//
//
// 								localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1SmPt,
// 										(double)localpath1SmPt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// 										 (double)localpath1SmPt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// 								softMotion_data_copy_into(robotPt, localpathTmp1Pt->specific.softMotion_data, softMotion_data_lp1);
//
// 								localpath2Pt = localpathMlp1Pt->next_lp;
//
//
// 								localpath2SmPt = p3d_local_planner_multisol(robotPt, lin_specificPt->q_init, lin_specificPt->q_end,  iksol);
//
//
//
//
//
//
//
// 							} else { // iksol are not equal, we are passing through a singularity */
// 								/* We add the both original localpaths (with stop motion) */
// 									localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1SmPt,
// 											(double)localpath1SmPt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// 											 (double)localpath1SmPt->specific.softMotion_data->freeflyer->motionTime);
// 									localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2SmPt, 0.0,
// 											(double)localpath2SmPt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3]);
// 									end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
// 									end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp2Pt);
//
//
// 							}
// 						}// END ELSE (localpathMlp1Pt->next_lp != NULL)
//
//
//
//
//
// 					} // END WHILE (localpath1Pt != NULL)
//
// 				} // END IF (strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "jido-ob_lin") == 0)
// 			} // END IF (robotPt->mlp->mlpJoints[iGraph]->gpType == FREEFLYER)
// 		} // END FOR iGraph
//
//
// 		//
// //  			if(robotPt->mlp->mlpJoints[iGraph]->gpType == FREEFLYER) {
// // 				if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "object_lin") == 0) {
// // 					/* There are three localpath like xarm module on Jido (see IROS08 paper "Soft Motion Trajectory Planner For Service Manipulator Robot")
// // 					* The one localpathTmp1Pt is the first motion, localpathTmpTrans is the the transition motion, localpathTmp2Pt is the third motion
// 		// 					*/
// // 					if(softMotion_data_lp1 == NULL) {
// // 						softMotion_data_lp1 = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, iGraph);
// // 					}
// // 					if(softMotion_data_lp2 == NULL) {
// // 						softMotion_data_lp2 = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, iGraph);
// // 					}
// // 					if(softMotion_data_lpTrans == NULL) {
// // 						softMotion_data_lpTrans = p3d_create_softMotion_data_multigraph(robotPt, FREEFLYER, 1, iGraph);
// // 					}
// 		//
// 		// 					/* Compute the fisrt point to point motion */
// // 					localpath1Pt = trajPt->courbePt->mlpLocalpath[iGraph];
// // 					if (localpath1Pt->type_lp != LINEAR){
// // 						PrintError(("p3d_optim_traj_softMotion: local path must be linear\n"));
// // 						return NULL;
// // 					}
// // 					lin_specificPt = localpath1Pt->specific.lin_data;
// // 					q_init = lin_specificPt->q_init;
// // 					q_end = lin_specificPt->q_end;
// 		//
// 		//
// 		// 					/* look for the first local path */
// // 					localpath1Pt = trajPt->courbePt->mlpLocalpath[iGraph];
// // 					softMotion_data_copy_into(robotPt, localpath1Pt->specific.softMotion_data, softMotion_data_lp1);
// 		//
// // 					Gb_v3_set( &softMotion_data_lp1->freeflyer->velLinInit, 0.0, 0.0, 0.0);
// // 					Gb_v3_set( &softMotion_data_lp1->freeflyer->velAngInit, 0.0, 0.0, 0.0);
// // 					softMotion_params = lm_get_softMotion_lm_param_multilocalpath(robotPt, iGraph);
// 		//
// // 					nlp = 0;
// // 					firstLpSet = 0;
// 		//
// 		// 					/* We add the three fisrt segment to the trajectory */
// // 					trajSmPt.courbePt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt, 0.0, (double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3]);
// // 					end_trajSmPt = trajSmPt.courbePt;
//
// // 						q1 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_init);
// // 						q2 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_end);
// //
// // 						if (localpath1Pt->next_lp == NULL) {
// // 							/* It's the last localpath */
// // 							if(nlp == 0) {
// // 								localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// // 									(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 									(double)localpath1Pt->specific.softMotion_data->freeflyer->motionTime);
// // 							} else {
// // 								/* Compare the iksol betwwen the two adjacent localpath :
// // 										if there are not equal we are passing through a singularity */
// // 								if(p3d_compare_iksol(robotPt->cntrt_manager, localpath1Pt->ikSol, localpath1Pt->next_lp->ikSol) == TRUE) {
// // 									 //iksol are equal
// // 									printf("p3d_optim_traj_softMotion: iksol are equal\n");
// // 									localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
// // 									localpathTmp2Pt->destroy(robotPt, localpathTmp2Pt);
// // 									/* We add the both original localpaths (with stop motion) */
// // 									localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// // 											(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 											 (double)localpath1Pt->specific.softMotion_data->freeflyer->motionTime);
// // 									localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt, 0.0,
// // 											(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3]);
// // 									end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
// // 									end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp2Pt);
// //}
// // 								} else {
// // 									iksol = localpath1Pt->ikSol;
// // 									localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// // 										(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 										(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// // 									softMotion_data_copy_into(robotPt, localpathTmp1Pt->specific.softMotion_data, softMotion_data_lp1);
// //
// // 									localpath2Pt = localpath1Pt->next_lp;
// //
// // 									localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt,
// // 										(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 										(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][4]);
// // 									softMotion_data_copy_into(robotPt, localpathTmp2Pt->specific.softMotion_data, softMotion_data_lp2);
// //
//
//
//
// // 									/* Set Transition motion */
// // 									softMotion_data_lpTrans->q_init = localpathTmp1Pt->config_at_distance(robotPt, localpathTmp2Pt, softMotion_data_lp1->freeflyer->motion.TimeCumulM[0][4]);
// // 									softMotion_data_lpTrans->q_end =  localpathTmp2Pt->config_at_distance(robotPt, localpathTmp1Pt, softMotion_data_lp2->freeflyer->motion.TimeCumulM[0][3]);
// //
// // 									/* Jmax, Amax and Vmax must have the same ratio between lpTmp1 and lpTmp2 */
// // 									softMotion_data_lpTrans->freeflyer->J_max_lin = MAX(softMotion_data_lp1->freeflyer->J_max_lin, softMotion_data_lp2->freeflyer->J_max_lin);
// // 									softMotion_data_lpTrans->freeflyer->A_max_lin = MAX(softMotion_data_lp1->freeflyer->A_max_lin, softMotion_data_lp2->freeflyer->A_max_lin);
// // 									softMotion_data_lpTrans->freeflyer->V_max_lin = MAX(softMotion_data_lp1->freeflyer->V_max_lin, softMotion_data_lp2->freeflyer->V_max_lin);
// // 									softMotion_data_lpTrans->freeflyer->J_max_ang = MAX(softMotion_data_lp1->freeflyer->J_max_ang, softMotion_data_lp2->freeflyer->J_max_ang);
// // 									softMotion_data_lpTrans->freeflyer->A_max_ang = MAX(softMotion_data_lp1->freeflyer->A_max_ang, softMotion_data_lp2->freeflyer->A_max_ang);
// // 									softMotion_data_lpTrans->freeflyer->V_max_ang = MAX(softMotion_data_lp1->freeflyer->V_max_ang, softMotion_data_lp2->freeflyer->V_max_ang);
// //
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseLinInit), softMotion_data_lp1->freeflyer->motion.FC[0].x,
// // 														softMotion_data_lp1->freeflyer->motion.FC[1].x,
// // 														softMotion_data_lp1->freeflyer->motion.FC[2].x);
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseLinEnd), softMotion_data_lp2->freeflyer->motion.IC[0].x,
// // 																		softMotion_data_lp2->freeflyer->motion.IC[1].x,
// // 																			softMotion_data_lp2->freeflyer->motion.IC[2].x);
// //
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velLinInit), softMotion_data_lp1->freeflyer->motion.FC[0].v,
// // 														softMotion_data_lp1->freeflyer->motion.FC[1].v,
// // 															softMotion_data_lp1->freeflyer->motion.FC[2].v);
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velLinEnd), softMotion_data_lp2->freeflyer->motion.IC[0].v,
// // 														softMotion_data_lp2->freeflyer->motion.IC[1].v,
// // 															softMotion_data_lp2->freeflyer->motion.IC[2].v);
// //
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseAngInit), softMotion_data_lp1->freeflyer->motion.FC[3].x,
// // 														softMotion_data_lp1->freeflyer->motion.FC[4].x,
// // 															softMotion_data_lp1->freeflyer->motion.FC[5].x);
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->poseAngEnd), softMotion_data_lp2->freeflyer->motion.IC[3].x,
// // 														softMotion_data_lp2->freeflyer->motion.IC[4].x,
// // 														softMotion_data_lp2->freeflyer->motion.IC[5].x);
// //
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velAngInit), softMotion_data_lp1->freeflyer->motion.FC[3].v,
// // 														softMotion_data_lp1->freeflyer->motion.FC[4].v,
// // 															softMotion_data_lp1->freeflyer->motion.FC[5].v);
// // 									Gb_v3_set( &(softMotion_data_lpTrans->freeflyer->velAngEnd), softMotion_data_lp2->freeflyer->motion.IC[3].v,
// // 														softMotion_data_lp2->freeflyer->motion.IC[4].v,
// // 														softMotion_data_lp2->freeflyer->motion.IC[5].v);
// //
// // 									q1 = localpathTmp1Pt->config_at_distance(robotPt, localpathTmp1Pt, (double)localpathTmp1Pt->specific.softMotion_data->freeflyer->motionTime);
// // 									q2 = localpathTmp2Pt->config_at_distance(robotPt, localpathTmp2Pt, 0.0);
// // 									q3 = localpathTmp2Pt->config_at_distance(robotPt, localpathTmp2Pt, (double)localpathTmp2Pt->specific.softMotion_data->freeflyer->motionTime);
// //
// // 									//****************************************************************************************************
// // 									/* Compute the transition softMotion */
// // 									localpathTransPt = p3d_softMotion_localplanner(robotPt, iGraph, softMotion_data_lpTrans, q1, q2, q3, iksol);
// // 									//***************************************************************************************************
// //
// // 									if(localpathTransPt==NULL) {
// // 										printf("localpathTmp1Pt==NULL\n");
// // 										localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
// // 										localpathTmp2Pt->destroy(robotPt, localpathTmp2Pt);
// // 										/* We add the both original localpaths (with stop motion) */
// // 										localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
// // 												(double)localpath1Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3],
// // 												(double)localpath1Pt->specific.softMotion_data->freeflyer->motionTime);
// // 										localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt, 0.0,
// // 												(double)localpath2Pt->specific.softMotion_data->freeflyer->motion.TimeCumulM[0][3]);
// // 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
// // 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp2Pt);
// //
// // 									} else {
// // 										/* Transition motion is OK */
// // 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
// // 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTransPt);
// // 										end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp2Pt);
// // 									}
// // 								}
// // 							} // End Else (localpath1Pt->next_lp == NULL)
// // 							cost += end_trajSmPt->cost(robotPt, end_trajSmPt);
// // 							localpath1Pt = localpath1Pt->next_lp;
// // 							if(localpathTmp1Pt != NULL) {
// // 							localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
// // 							}
// // 							if(localpathTmp2Pt != NULL) {
// // 							localpathTmp2Pt->destroy(robotPt, localpathTmp2Pt);
// // 							}
// // 							localpathTmp2Pt = NULL;
// // 							localpathTmp1Pt = NULL;
// // 							nlp++;
// // 						} // End If (localpath1Pt->next_lp == NULL)
// // 					} // End while (localpath1Pt != NULL)
// // 				} // End If (strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "object") == 0)
// // 			} // End If (robotPt->mlp->mlpJoints[iGraph]->gpType != FREEFLYER)
// // 		} // End For iGraph
//
//
//
// 	} // End Else (traj->nlp > 2)
//
// // 	p3d_destroy_softMotion_data(robotPt, softMotion_data_lp1);
// // 	p3d_destroy_softMotion_data(robotPt, softMotion_data_lp2);
// // 	p3d_destroy_softMotion_data(robotPt, softMotion_data_lpTrans);
// //
// // 	/* destroy the initial trajectory and replace it by the new one */
// //  destroy_list_localpath(robotPt, trajPt->courbePt);
// //  trajPt->courbePt = NULL;
// //  /* replace it by the new one */
// //  trajPt->courbePt = trajSmPt.courbePt;
// //  /* update the number of local paths */
// //  trajPt->nlp = p3d_compute_traj_nloc(trajPt);
// //  /* store the parameter range of this trajectory */
// //  trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
// //
// // 	printf("nlp %d nlpTraj %d\n",nlp,trajPt->nlp);
// //  	/* Write curve into a file for BLTPLOT */
// // 	if ((fileptr = fopen("RefSM.dat","w+"))==NULL) {
// // 		printf("cannot open File RefTP.dat");
// // 	}
// // 	localpath1Pt = trajPt->courbePt;
// // 	fprintf(fileptr,"# i PX.Acc PX.Vel PX.Pos PY.Acc PY.Vel PY.Pos PZ.Acc PZ.Vel PZ.Pos RX.Acc RX.Vel RX.Pos RY.Acc RY.Vel RY.Pos RZ.Acc RZ.Vel RZ.Pos ;\n");
// // 	indexInFile = 0;
// // 	while(localpath1Pt != NULL){
// // 	p3d_softMotion_write_curve_for_bltplot(localpath1Pt, fileptr, &indexInFile) ;
// // 		localpath1Pt = localpath1Pt->next_lp;
// // 	}
// // 	fclose(fileptr);
// // 	printf("File RefSM created\n");
//
// 	return FALSE;
// }




#endif

