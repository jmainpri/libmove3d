#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "gbM/Proto_gbModeles.h"
#include <list>
#include <string>
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/ManipulationStruct.h"
#include "../lightPlanner/proto/Manipulation.h"
#include "../lightPlanner/proto/ManipulationPlanner.hpp"
#include "../lightPlanner/proto/ManipulationUtils.hpp"

static int NB_TRAJPTP_CONFIG= 0;
static configPt TRAJPTP_CONFIG[200];



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

        p3d_rob* robot= (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);;
       	p3d_jnt *armJoint= NULL;

	armJoint= p3d_get_robot_jnt_by_name ( robot, (char*)"RightWrist");

	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	glColor3f(0,1,0);

	for(i=1; i<NB_TRAJPTP_CONFIG; i++) {
		for(int j=0; j<3; j++) {
			p1[j] = TRAJPTP_CONFIG[i-1][armJoint->index_dof +j];
			p2[j] = TRAJPTP_CONFIG[i][armJoint->index_dof +j];
		}
		g3d_draw_cylinder(p1, p2, 0.0005, 16);
	}
	glPopAttrib();
}

int p3d_multilocalpath_switch_to_softMotion_groups(p3d_rob* robotPt) {

	p3d_multiLocalPath_disable_all_groupToPlan(robotPt);

	for(int iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
		  p3d_multiLocalPath_set_groupToPlan(robotPt, iGraph, 1);
		}
	}
//         for(unsigned int u = 0; u < robotPt->armManipulationData->size(); u++) {
// 	  if((*robotPt->armManipulationData)[u].getCartesian()) {
// 	    p3d_multiLocalPath_set_groupToPlan(robotPt, (*robotPt->armManipulationData)[u].getCartesianSmGroup(), 1);
// 	  }
// 	  p3d_multiLocalPath_set_groupToPlan(robotPt, (*robotPt->armManipulationData)[u].getHandSmGroup(), 1);
// 	}

	return 0;
}

int p3d_multilocalpath_switch_to_linear_groups(p3d_rob* robotPt) {
  
  	p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
	for(int iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBody") == 0) {
		  p3d_multiLocalPath_set_groupToPlan(robotPt, iGraph, 1);
		}
	}
	for(unsigned int u = 0; u < robotPt->armManipulationData->size(); u++) {
	  if((*robotPt->armManipulationData)[u].getCartesian()) {
	    p3d_multiLocalPath_set_groupToPlan(robotPt, (*robotPt->armManipulationData)[u].getCartesianGroup(), 1);
	  }
	  p3d_multiLocalPath_set_groupToPlan(robotPt, (*robotPt->armManipulationData)[u].getHandGroup(), 1);
	}
	return 0;
}


int p3d_convert_ptpTraj_to_smoothedTraj(double *gain, int* ntest, p3d_traj *trajSmPTPPt, p3d_traj *trajSmPt) {

  p3d_rob *robotPt = trajSmPTPPt->rob;


	p3d_localpath *end_trajSmPt = NULL;

	p3d_localpath *localpathMlp1Pt = trajSmPTPPt->courbePt;
	p3d_localpath *localpathMlp2Pt = localpathMlp1Pt->next_lp;

	p3d_localpath *localpath1Pt = NULL;
	p3d_localpath *localpath2Pt = NULL;
	p3d_localpath *localpathTransPt = NULL;
	p3d_localpath *localpathTmp1Pt = NULL;
	p3d_localpath *localpathTmp2Pt = NULL;

	configPt q1 = NULL, q2 = NULL;


	double cost = 0.0;
	int firstLpSet = 0;
	int nlp = 0;

	p3d_softMotion_data* softMotion_data_lp1 = NULL;
	p3d_softMotion_data* softMotion_data_lp2 = NULL;
	p3d_softMotion_data* softMotion_data_lpTrans = NULL;


	int IGRAPH_OUTPUT;
	///////////////////////////////////////////////////
	//  CREATE TRAJECTORY WITH REMOVED HALT AT NODES///
	///////////////////////////////////////////////////
	for(int iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
		  IGRAPH_OUTPUT = iGraph;
		}
	}




	///////////////////////////////
	////  INITIALIZE VARIABLES  ////
	////////////////////////////////
	if(softMotion_data_lp1 == NULL) {
		softMotion_data_lp1 = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
	}
	if(softMotion_data_lp2 == NULL) {
		softMotion_data_lp2 = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
	}
	if(softMotion_data_lpTrans == NULL) {
		softMotion_data_lpTrans = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
	}


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

	while(localpathMlp1Pt != NULL && localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT] !=NULL) {
		localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];
		q1 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_init);
		q2 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_end);

		if (localpathMlp1Pt->next_lp == NULL ||localpathMlp1Pt->next_lp->mlpLocalpath[IGRAPH_OUTPUT] == NULL  ) {
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




  
 return FALSE;
}

int p3d_convert_traj_to_softMotion(p3d_traj *trajPt, bool param_write_file, std::vector <int> &lp, std::vector < std::vector <double> > &positions, SM_TRAJ &smTraj) {

        double gain = 0.0;
        int ntest = 0;
        p3d_rob *robotPt = trajPt->rob;
	p3d_traj *trajSmPTPPt = NULL; // the point to point trajectory
        p3d_traj *trajSmPt = NULL; // the smoothed trajectory
	p3d_localpath *end_trajSmPt = NULL;

        configPt qinit = NULL, qgoal = NULL;
	configPt q_init = NULL, q_end = NULL;

	p3d_localpath *localpath1Pt = NULL;
        p3d_localpath *localpathTmp1Pt = NULL;

	double ltot = 0.0;

	p3d_localpath *localpathMlp1Pt = trajPt->courbePt;

	/* length of trajPt */
	ltot = p3d_ends_and_length_traj(trajPt, &qinit, &qgoal);
	if (ltot<= 3*EPS6) {
		/* trajectory too short */
		p3d_destroy_config(robotPt, qinit);
		p3d_destroy_config(robotPt, qgoal);
		return FALSE;
	}
	
        ///////////////////////////////////////////////////////////////////////////
	////  CONVERT LINEAR TRAJECTORY TO SOFTMOTION POINT TO POINT TRAJECTORY ///
	///////////////////////////////////////////////////////////////////////////

	/* Create the softMotion point to point trajectory */
	trajSmPTPPt = p3d_create_empty_trajectory(robotPt);
	q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
	q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);

	p3d_multilocalpath_switch_to_softMotion_groups(robotPt);
	trajSmPTPPt->courbePt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->ikSol);

	trajSmPTPPt->courbePt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
	for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
		trajSmPTPPt->courbePt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
	}

	end_trajSmPt = trajSmPTPPt->courbePt;
	TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_init);
	NB_TRAJPTP_CONFIG ++;
	TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_end);
	NB_TRAJPTP_CONFIG ++;
	localpathMlp1Pt = localpathMlp1Pt->next_lp;

	while(localpathMlp1Pt != NULL){
		localpath1Pt = localpathMlp1Pt;
		if(localpath1Pt!= NULL) {
			q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
			q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);


			p3d_multilocalpath_switch_to_softMotion_groups(robotPt);
			localpathTmp1Pt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->ikSol);

			localpathTmp1Pt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
			for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
				localpathTmp1Pt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
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
	printf("BioMove3D: softMotion point-to-point trajectory OK\n");


	/* Create the softMotion trajectory */
	trajSmPt = p3d_create_empty_trajectory(robotPt);


        p3d_convert_ptpTraj_to_smoothedTraj(&gain, &ntest, trajSmPTPPt, trajSmPt);
	robotPt->tcur = trajSmPt;


        /* Write curve into a file for BLTPLOT */ //ENV.getBool(Env::plotSoftMotionCurve)
	if(param_write_file == true) {
	  p3d_softMotion_export_traj(robotPt, trajSmPTPPt, 0, (char*)"softMotion_PTP_Q.traj",  (char*)"softMotion_PTP_Seg.traj",ENV.getBool(Env::plotSoftMotionCurve),  lp, positions, smTraj) ;
	  smTraj.clear();
	  p3d_softMotion_export_traj(robotPt, trajSmPt, 1, (char*)"softMotion_Smoothed_Q.traj",  (char*)"softMotion_Smoothed_Seg.traj",ENV.getBool(Env::plotSoftMotionCurve),  lp, positions, smTraj) ;
	  //smTraj.print();
	}
	if (fct_draw){(*fct_draw)();}



	g3d_win *win= NULL;
	win= g3d_get_cur_win();
	win->fct_draw2 = &(draw_trajectory_ptp);
	g3d_draw_allwin_active();

	return FALSE;

}




//int p3d_optim_traj_softMotion(p3d_traj *trajPt, bool param_write_file, double *gain, int *ntest, std::vector <int> &lp, std::vector < std::vector <double> > &positions, SM_TRAJ &smTraj) {
// 
//
//  p3d_rob *robotPt = trajPt->rob;
//	p3d_traj *trajSmPTPPt = NULL;
//	p3d_traj *trajSmPt = NULL;
//	p3d_localpath *end_trajSmPt = NULL;
//
//	p3d_localpath *localpathMlp1Pt = trajPt->courbePt;
//	p3d_localpath *localpathMlp2Pt = localpathMlp1Pt->next_lp;
//
//	p3d_localpath *localpath1Pt = NULL;
//	p3d_localpath *localpath2Pt = NULL;
//	p3d_localpath *localpathTransPt = NULL;
//	p3d_localpath *localpathTmp1Pt = NULL;
//	p3d_localpath *localpathTmp2Pt = NULL;
//
//	configPt q1 = NULL, q2 = NULL, qinit = NULL, qgoal = NULL;
//	configPt q_init = NULL, q_end = NULL;
//
//	double ltot = 0.0;
//	double cost = 0.0;
//	int firstLpSet = 0;
//	int nlp = 0, iGraph=0;
//	int *iksol=NULL;
//	p3d_softMotion_data* softMotion_data_lp1 = NULL;
//	p3d_softMotion_data* softMotion_data_lp2 = NULL;
//	p3d_softMotion_data* softMotion_data_lpTrans = NULL;
//	
//	std::vector<int> IGRAPH_OBJECT;
//	std::vector<int> IGRAPH_OBJECT_SM;
//	int IGRAPH_UPBODY_LIN = 0;
//	int IGRAPH_UPBODY_SM = 0;
//        std::vector<int> IGRAPH_INPUT;
//	std::vector<int> IGRAPH_OUTPUT;
//
//
//	int robotType = 0;
//	/* length of trajPt */
//	ltot = p3d_ends_and_length_traj(trajPt, &qinit, &qgoal);
//	if (ltot<= 3*EPS6) {
//		/* trajectory too short */
//		p3d_destroy_config(robotPt, qinit);
//		p3d_destroy_config(robotPt, qgoal);
//		return FALSE;
//	}
//
//	// find the type of the robot and check if the robot is OK
//	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
//		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
//			robotType = 1;
//		}
//	}
//	if(robotType == 0) {
//	  printf("ERROR p3d_optim_traj_softMotion unknow robot type \n");
//	  return 1;
//	}
//
//
//	// Find the groups ID and activate the softMotion groups
//	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
//		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBody") == 0) {
//			IGRAPH_UPBODY_LIN = iGraph;
//		}
//	}
//
//	for(iGraph=0; iGraph<robotPt->mlp->nblpGp; iGraph++) {
//		if(strcmp(robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
//			IGRAPH_UPBODY_SM = iGraph;
//		}
//	}
//        p3d_multiLocalPath_set_groupToPlan(robotPt, IGRAPH_UPBODY_LIN, 0);
//	p3d_multiLocalPath_set_groupToPlan(robotPt, IGRAPH_UPBODY_SM, 1);
//      	for(unsigned int u = 0; u < robotPt->armManipulationData->size(); u++) {
//	  IGRAPH_OBJECT.push_back((*robotPt->armManipulationData)[u].getCartesianGroup());
//	  IGRAPH_OBJECT_SM.push_back((*robotPt->armManipulationData)[u].getCartesianSmGroup());
//	  if((*robotPt->armManipulationData)[u].getCartesian()) {
//	    p3d_multiLocalPath_set_groupToPlan(robotPt, (*robotPt->armManipulationData)[u].getCartesianGroup(), 0);
//	    p3d_multiLocalPath_set_groupToPlan(robotPt, (*robotPt->armManipulationData)[u].getCartesianSmGroup(), 1);
//	  }
//	}
//
//	// Clear static variable
//	if (TRAJPTP_CONFIG[0]!=NULL) {
//		for(int i=0; i<NB_TRAJPTP_CONFIG; i++) {
//			p3d_destroy_config(robotPt, TRAJPTP_CONFIG[i]);
//			TRAJPTP_CONFIG[i] = NULL;
//		}
//	}
//	NB_TRAJPTP_CONFIG = 0;
//	int nbNonNullLp = 0;
//	///////////////////////////////////////////////////////////////////////////
//	////  CONVERT LINEAR TRAJECTORY TO SOFTMOTION POINT TO POINT TRAJECTORY ///
//	///////////////////////////////////////////////////////////////////////////
//
//	/* Create the softMotion trajectory */
//	trajSmPTPPt = p3d_create_empty_trajectory(robotPt);
//
//        
//	
//// 	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OBJECT_LIN];
//// 	if(localpath1Pt!=NULL) {
//// 		IGRAPH_INPUT = IGRAPH_OBJECT_LIN;
//// 		IGRAPH_OUTPUT = IGRAPH_OBJECT_SM;
//// 		nbNonNullLp ++;
//// 	}
//// 	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OBJECT_SM];
//// 	if(localpath1Pt!=NULL) {
//// 		IGRAPH_INPUT = IGRAPH_OBJECT_SM;
//// 		IGRAPH_OUTPUT = IGRAPH_OBJECT_SM;
//// 		nbNonNullLp ++;
//// 	}
//// 	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_UPBODY_LIN];
//// 	if(localpath1Pt!=NULL) {
//// 		IGRAPH_INPUT = IGRAPH_UPBODY_LIN;
//// 		IGRAPH_OUTPUT = IGRAPH_UPBODY_SM;
//// 		nbNonNullLp ++;
//// 	}
//// 	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_UPBODY_SM];
//// 	if(localpath1Pt!=NULL) {
//// 		IGRAPH_INPUT = IGRAPH_UPBODY_SM;
//// 		IGRAPH_OUTPUT = IGRAPH_UPBODY_SM;
//// 		nbNonNullLp ++;
//// 	}
//// 
//// 	if(nbNonNullLp != 1) {
//// 		printf("softMotion compute traj ERROR non null lp !=1\n");
//// 		return FALSE;
//// 	}
//// 
//// 
//// 
//// 	p3d_multiLocalPath_set_groupToPlan(robotPt, IGRAPH_OUTPUT, 1);
//
//	q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
//	q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);
//
//// 	p3d_desactivateAllCntrts(robotPt);
//// 	for(int i = 0; i < localpathMlp1Pt->nbActiveCntrts; i++){
//// 		p3d_activateCntrt(robotPt, robotPt->cntrt_manager->cntrts[localpathMlp1Pt->activeCntrts[i]]);
//// 	}
//
//	trajSmPTPPt->courbePt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->ikSol);
//
//	trajSmPTPPt->courbePt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
//	for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
//		trajSmPTPPt->courbePt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
//	}
//
//	end_trajSmPt = trajSmPTPPt->courbePt;
//	TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_init);
//	NB_TRAJPTP_CONFIG ++;
//	TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_end);
//	NB_TRAJPTP_CONFIG ++;
//	localpathMlp1Pt = localpathMlp1Pt->next_lp;
//
//	while(localpathMlp1Pt != NULL){
//		localpath1Pt = localpathMlp1Pt;
//		if(localpath1Pt!= NULL) {
//			q_init = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, 0.0);
//			q_end = localpathMlp1Pt->config_at_param(robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);
//
//
//
//
//
//			p3d_multilocalpath_switch_to_softMotion_groups(robotPt);
//			p3d_multilocalpath_switch_to_linear_groups(robotPt);
//			
//			localpathTmp1Pt = p3d_local_planner_multisol(robotPt, q_init, q_end,  localpathMlp1Pt->ikSol);
//
//			localpathTmp1Pt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
//			for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
//				localpathTmp1Pt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
//			}
//
//			TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config(robotPt, q_end);
//			NB_TRAJPTP_CONFIG ++;
//			end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
//		}
//		localpathMlp1Pt = localpathMlp1Pt->next_lp;
//	} // END WHILE (localpathMlp1Pt != NULL)
//
//
//
//	/* update the number of local paths */
//	trajSmPTPPt->nlp = p3d_compute_traj_nloc(trajSmPTPPt);
//	trajSmPTPPt->range_param = p3d_compute_traj_rangeparam(trajSmPTPPt);
//	g3d_add_traj((char*)"traj_SoftMotion_PTP", trajSmPTPPt->num);
//
//
//	/////////////////////////////////////////////////////
//	////  CREATE TRAJECTORY WITH REMOVED HALT AT NODES///
//	/////////////////////////////////////////////////////
//
//
//	/////////////////////////////////
//	//////  INITIALIZE VARIABLES  ////
//	//////////////////////////////////
//// 	if(softMotion_data_lp1 == NULL) {
//// 		softMotion_data_lp1 = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
//// 	}
//// 	if(softMotion_data_lp2 == NULL) {
//// 		softMotion_data_lp2 = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
//// 	}
//// 	if(softMotion_data_lpTrans == NULL) {
//// 		softMotion_data_lpTrans = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
//// 	}
//// 
//// 	/* Create the softMotion trajectory */
//// 	trajSmPt = p3d_create_empty_trajectory(robotPt);
//// 	localpathMlp1Pt = trajSmPTPPt->courbePt;
//// 	localpathMlp2Pt = trajSmPTPPt->courbePt->next_lp;
//// 	localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];
//// 
//// 
//// 
//// 	softMotion_data_copy_into(robotPt, localpath1Pt->specific.softMotion_data, softMotion_data_lp1);
//// 
//// 	for(int v = 0; v<softMotion_data_lp1->nbDofs; v++) {
//// 		softMotion_data_lp1->specific->velInit[v] = 0.0;
//// 		softMotion_data_lp1->specific->velEnd[v] = 0.0;
//// 		softMotion_data_lp1->specific->accInit[v] = 0.0;
//// 		softMotion_data_lp1->specific->accEnd[v] = 0.0;
//// 	}
//// 
//// 		/*
//// 	* There are three localpath like xarm module on Jido (see IROS08 paper "Soft Motion Trajectory Planner For Service Manipulator Robot")
//// 	* The one localpathTmp1Pt is the first motion, localpathTmpTrans is the the transition motion, localpathTmp2Pt is the third motion
//// 		*/
//// 	nlp = 0;
//// 	firstLpSet = 0;
//// 
//// 	int collision = FALSE;
//// 	/* We add the three fisrt segment to the trajectory */
//// 
//// 	trajSmPt->courbePt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt, 0.0, localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);
//// 	end_trajSmPt = trajSmPt->courbePt;
//// 
//// 	while(localpathMlp1Pt != NULL) {
//// 		localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];
//// 		q1 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_init);
//// 		q2 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_end);
//// 
//// 		if (localpathMlp1Pt->next_lp == NULL) {
//// 			/* It's the last localpath */
//// 
//// 
//// 
//// 				if(nlp == 0) {
//// 					localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
//// 							(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
//// 							 (double)localpath1Pt->specific.softMotion_data->specific->motionTime);
//// 				} else {
//// 					localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
//// 							(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
//// 							 (double)localpath1Pt->specific.softMotion_data->specific->motionTime);
//// 				}
//// 			end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
//// 
//// 		} else {
//// 			localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
//// 					(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
//// 					 (double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);
//// 
//// 			softMotion_data_copy_into(robotPt, localpathTmp1Pt->specific.softMotion_data, softMotion_data_lp1);
//// 
//// 			localpathMlp2Pt = localpathMlp1Pt->next_lp;
//// 			localpath2Pt = localpathMlp2Pt->mlpLocalpath[IGRAPH_OUTPUT];
//// 			localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt,
//// 					(double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
//// 					 (double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);
//// 
//// 			softMotion_data_copy_into(robotPt, localpathTmp2Pt->specific.softMotion_data, softMotion_data_lp2);
//// 
//// 			/* Set Transition motion */
//// 			if(softMotion_data_lpTrans == NULL) {
//// 				softMotion_data_lpTrans = p3d_create_softMotion_data_multilocalpath(robotPt, IGRAPH_OUTPUT);
//// 			}
//// 
//// 			softMotion_data_lpTrans->q_init = localpath1Pt->config_at_distance(robotPt, localpath1Pt, (double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);
//// 			softMotion_data_lpTrans->q_end =  localpath2Pt->config_at_distance(robotPt, localpath2Pt, (double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);
//// 
//// 			/* Jmax, Amax and Vmax must have the same ratio between lpTmp1 and lpTmp2 */
//// 			for(int v=0; v<softMotion_data_lp1->nbDofs; v++) {
//// 			softMotion_data_lpTrans->specific->J_max[v] = MAX(softMotion_data_lp1->specific->J_max[v], softMotion_data_lp2->specific->J_max[v]);
//// 			softMotion_data_lpTrans->specific->A_max[v] = MAX(softMotion_data_lp1->specific->A_max[v], softMotion_data_lp2->specific->A_max[v]);
//// 			softMotion_data_lpTrans->specific->V_max[v] = MAX(softMotion_data_lp1->specific->V_max[v], softMotion_data_lp2->specific->V_max[v]);
//// 
//// 			softMotion_data_lpTrans->specific->velInit[v] = softMotion_data_lp1->specific->motion[v].FC.v;
//// 			softMotion_data_lpTrans->specific->velEnd[v] = softMotion_data_lp2->specific->motion[v].IC.v;
//// 			softMotion_data_lpTrans->specific->accInit[v] = softMotion_data_lp1->specific->motion[v].FC.a;
//// 			softMotion_data_lpTrans->specific->accEnd[v] = softMotion_data_lp2->specific->motion[v].IC.a;
//// 			}
//// 
//// 			q1 = localpath1Pt->config_at_distance(robotPt, localpath1Pt, (double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[4]);
//// 			q2 =  localpath2Pt->config_at_distance(robotPt, localpath2Pt, (double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);
//// 
//// 			softMotion_data_lpTrans->isPTP = FALSE;
//// 			localpathTransPt = p3d_softMotion_localplanner(robotPt, IGRAPH_OUTPUT, softMotion_data_lpTrans, q1, q2, q2, localpath1Pt->ikSol);
//// 
//// 			if(localpathTransPt != NULL){
//// 				collision = p3d_unvalid_localpath_test(robotPt, localpathTransPt, ntest);
//// 			}
//// 			if(localpathTransPt==NULL || collision == TRUE) {
//// 
//// 				printf("localpathTransPt : stop motion localpath = %p , collision = %d\n", localpathTransPt,collision);
//// 
//// 
//// 				localpathTmp1Pt->destroy(robotPt, localpathTmp1Pt);
//// 				/* We add the both original localpaths (with stop motion) */
//// 				localpathTmp1Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath1Pt,
//// 						(double)localpath1Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3],
//// 						 (double)localpath1Pt->specific.softMotion_data->specific->motionTime);
//// 				localpathTmp2Pt = p3d_extract_softMotion_with_velocities(robotPt, localpath2Pt, 0.0,
//// 						(double)localpath2Pt->specific.softMotion_data->specific->motion[0].TimeCumul[3]);
//// 
//// 				end_trajSmPt  = append_to_localpath(end_trajSmPt , localpathTmp1Pt);
//// 				nlp++;
//// 				end_trajSmPt  = append_to_localpath(end_trajSmPt , localpathTmp2Pt);
//// 				nlp++;
//// 
//// 				collision = FALSE;
//// 			} else {
//// 				localpathTransPt->nbActiveCntrts = localpath1Pt->nbActiveCntrts;
//// 				for(int v=0; v<localpathMlp1Pt->nbActiveCntrts; v++) {
//// 					localpathTransPt->activeCntrts[v] = localpath1Pt->activeCntrts[v];
//// 					localpathTransPt->activeCntrts[v] = localpath1Pt->activeCntrts[v];
//// 				}
//// 				/* Transition motion is OK */
//// 				end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTmp1Pt);
//// 				nlp++;
//// 				end_trajSmPt = append_to_localpath(end_trajSmPt, localpathTransPt);
//// 				nlp++;
//// 			}
//// 		}
//// 
//// 		cost += end_trajSmPt->cost(robotPt, end_trajSmPt);
//// 		localpathMlp1Pt = localpathMlp1Pt->next_lp;
//// 		localpathTmp2Pt = NULL;
//// 		localpathTmp1Pt = NULL;
//// 		softMotion_data_lpTrans = NULL;
//// 
//// 	}
//// 
//// 
//// 
//// 	/* update the number of local paths */
//// 	trajSmPt->nlp = p3d_compute_traj_nloc(trajSmPt);
//// 	trajSmPt->range_param = p3d_compute_traj_rangeparam(trajSmPt);
//// 	g3d_add_traj((char*)"traj_SoftMotion", trajSmPt->num);
//// 
//// 
//	/* Write curve into a file for BLTPLOT */
//	if(param_write_file == true) {
//		p3d_softMotion_write_curve_for_bltplot(robotPt, trajSmPTPPt, (char*)"RefSM.dat", ENV.getBool(Env::plotSoftMotionCurve), lp, positions, segments) ;
//	}
//	if (fct_draw){(*fct_draw)();}
//
//	g3d_win *win= NULL;
//	win= g3d_get_cur_win();
//	win->fct_draw2 = &(draw_trajectory_ptp);
//
//	g3d_draw_allwin_active();
//
//	robotPt->tcur = trajSmPt;
//	FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
//	return FALSE;
//}
