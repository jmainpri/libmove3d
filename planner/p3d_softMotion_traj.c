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

static int NB_TRAJPTP_CONFIG = 0;
static configPt TRAJPTP_CONFIG[200];

using namespace std;

//! returns the vector of the end 
//! of the Constant Velocity Segments
//! The end of the CVS are stored as parameters
//! first :   time for the smoothed softmotion trajectory
//! second :  geometric param along traj (distance)
std::vector< middleOfCVS > m_vectOfCVS;
std::vector< middleOfCVS > getCVS()
{
  return m_vectOfCVS;
}

//! prints the vector of paramteters
//! of middle the Constant Velocity Segments
void print_MiddleOfCVS(void)
{
  cout << " m_vectOfCVS( " << m_vectOfCVS.size() << " ) = " << endl;
  for(unsigned int i=0;i<m_vectOfCVS.size();i++) {
    cout << "First = " << m_vectOfCVS[i].first << " , " << m_vectOfCVS[i].second << endl;
  }
}

//! compute the End of The CSV for the 
//! geometric path
void p3d_computeMiddleOfCVSParam( p3d_localpath* smPath, p3d_localpath* linPath, double trajLength )
{
  middleOfCVS pairTmp;
  pairTmp.first = 0.5 * smPath->length_lp;
  pairTmp.second =  0.5 * linPath->length_lp + trajLength;
  m_vectOfCVS.push_back( pairTmp );
}

//! Test the two trajs
//! each configuration
bool p3d_test_middle_of_CVS( p3d_traj * trajPt,
                          p3d_traj * trajSmPt)
{
  bool res = true;
  p3d_rob* rob = trajPt->rob;
  for(unsigned int i=0;i<m_vectOfCVS.size();i++){
    configPt q1 = p3d_config_at_distance_along_traj(trajSmPt, m_vectOfCVS[i].first) ;   
     p3d_set_and_update_this_robot_conf_multisol(rob, q1, NULL, 0, trajSmPt->courbePt->ikSol);
     p3d_get_robot_config_into(rob, &q1);
 
    configPt q2 = p3d_config_at_distance_along_traj(trajPt,   m_vectOfCVS[i].second) ;  
    p3d_set_and_update_this_robot_conf_multisol(rob, q2, NULL, 0, trajPt->courbePt->ikSol);
    p3d_get_robot_config_into(rob, &q2);



    if( !p3d_equal_config( rob, q1, q2) ){
      printf("config differ in p3d_test_middle_of_CVS\n");    
      printf("ith CVS : %d\n",i);    
      //print_config(rob, q1);
      //print_config(rob, q2);
      res = false;
    }
    else {
      printf("Ith (%d): OK \n",i);
    }
  }
  return res;
}

void
draw_trajectory_ptp () {
  int i;
  p3d_vector3 p1;
  p3d_vector3 p2;
  if (TRAJPTP_CONFIG[0] == NULL || NB_TRAJPTP_CONFIG <= 0) {
    return;
  }
  p3d_rob *robot = (p3d_rob *) p3d_get_desc_curid (P3D_ROBOT);;
  p3d_jnt *armJoint = NULL;
  armJoint = p3d_get_robot_jnt_by_name (robot, (char *) "virtual_object");

  glPushAttrib (GL_LIGHTING_BIT);
  glDisable (GL_LIGHTING);
  glColor3f (0, 1, 0);

  for (i = 1; i < NB_TRAJPTP_CONFIG; i++) {
    for (int j = 0; j < 3; j++) {
      p1[j] = TRAJPTP_CONFIG[i - 1][armJoint->index_dof + j];
      p2[j] = TRAJPTP_CONFIG[i][armJoint->index_dof + j];
    }
    glColor3f (0, 1, 0);
    g3d_draw_cylinder (p1, p2, 0.001, 16);
    glColor3f (0, 0, 1);
    g3d_draw_solid_sphere(p1[0], p1[1], p1[2], 0.005, 16);
    g3d_draw_solid_sphere(p2[0], p2[1], p2[2], 0.005, 16);
  }
  glPopAttrib ();
}

int
p3d_multilocalpath_switch_to_softMotion_groups (p3d_rob * robotPt) {
  p3d_multiLocalPath_disable_all_groupToPlan (robotPt);
  for (int iGraph = 0; iGraph < robotPt->mlp->nblpGp; iGraph++) {
    if (strcmp (robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
      p3d_multiLocalPath_set_groupToPlan (robotPt, iGraph, 1);
    }
  }
  return 0;
}

int
p3d_multilocalpath_switch_to_linear_groups (p3d_rob * robotPt) {
  p3d_multiLocalPath_disable_all_groupToPlan (robotPt);
  for (int iGraph = 0; iGraph < robotPt->mlp->nblpGp; iGraph++) {
    if (strcmp (robotPt->mlp->mlpJoints[iGraph]->gpName, "upBody") == 0) {
      p3d_multiLocalPath_set_groupToPlan (robotPt, iGraph, 1);
    }
  }
  return 0;
}


int
p3d_convert_ptpTraj_to_smoothedTraj (double *gain, int *ntest,
                                     p3d_traj * trajSmPTPPt,
                                     p3d_traj * trajSmPt) {

  p3d_rob *robotPt = trajSmPTPPt->rob;
  p3d_localpath *end_trajSmPt = NULL;
  p3d_localpath *localpathMlp1Pt = trajSmPTPPt->courbePt;
  p3d_localpath *localpathMlp2Pt = localpathMlp1Pt->next_lp;
  p3d_localpath *localpath1Pt = NULL;
  p3d_localpath *localpath2Pt = NULL;
  p3d_localpath *localpathTransPt = NULL;
  p3d_localpath *localpathTmp1Pt = NULL;
  p3d_localpath *localpathTmp2Pt = NULL;
  double cost = 0.0, timeLength = 0.0;
  int firstLpSet = 0, nlp = 0, indexOfCVS = 0, IGRAPH_OUTPUT=0;
  p3d_softMotion_data *softMotion_data_lp1 = NULL;
  p3d_softMotion_data *softMotion_data_lp2 = NULL;
  p3d_softMotion_data *softMotion_data_lpTrans = NULL;


  ///////////////////////////////////////////////////
  //  CREATE TRAJECTORY WITH REMOVED HALT AT NODES///
  ///////////////////////////////////////////////////
  for (int iGraph = 0; iGraph < robotPt->mlp->nblpGp; iGraph++) {
    if (strcmp (robotPt->mlp->mlpJoints[iGraph]->gpName, "upBodySm") == 0) {
      IGRAPH_OUTPUT = iGraph;
    }
  }
  
  ///////////////////////////////
  ////  INITIALIZE VARIABLES  ////
  ////////////////////////////////
  if (softMotion_data_lp1 == NULL) {
    softMotion_data_lp1 =
      p3d_create_softMotion_data_multilocalpath (robotPt, IGRAPH_OUTPUT);
  }
  if (softMotion_data_lp2 == NULL) {
    softMotion_data_lp2 =
      p3d_create_softMotion_data_multilocalpath (robotPt, IGRAPH_OUTPUT);
  }
  if (softMotion_data_lpTrans == NULL) {
    softMotion_data_lpTrans =
      p3d_create_softMotion_data_multilocalpath (robotPt, IGRAPH_OUTPUT);
  }


  localpathMlp1Pt = trajSmPTPPt->courbePt;
  localpathMlp2Pt = trajSmPTPPt->courbePt->next_lp;
  localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];



//       softMotion_data_copy_into(robotPt, localpath1Pt->specific.softMotion_data, softMotion_data_lp1);
//
//       for(int v = 0; v<softMotion_data_lp1->nbDofs; v++) {
//      softMotion_data_lp1->specific->velInit[v] = 0.0;
//      softMotion_data_lp1->specific->velEnd[v] = 0.0;
//      softMotion_data_lp1->specific->accInit[v] = 0.0;
//      softMotion_data_lp1->specific->accEnd[v] = 0.0;
//       }

  /*
   * There are three localpath like xarm module on Jido (see IROS08 paper "Soft Motion Trajectory Planner For Service Manipulator Robot")
   * The one localpathTmp1Pt is the first motion, localpathTmpTrans is the the transition motion, localpathTmp2Pt is the third motion
   */
  nlp = 0;
  firstLpSet = 0;

  int collision = FALSE;
  /* We add the three fisrt segment to the trajectory */

  trajSmPt->courbePt =
    p3d_extract_softMotion_with_velocities (robotPt, localpath1Pt, 0.0,
                                            localpath1Pt->specific.
                                            softMotion_data->specific->
                                            motion[0].TimeCumul[3]);
  end_trajSmPt = trajSmPt->courbePt;
  
  timeLength = end_trajSmPt->length_lp;
  indexOfCVS = 0;

  while (localpathMlp1Pt != NULL
         && localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT] != NULL) {
    localpath1Pt = localpathMlp1Pt->mlpLocalpath[IGRAPH_OUTPUT];
    //q1 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_init);
    //q2 = p3d_copy_config(robotPt, localpath1Pt->specific.softMotion_data->q_end);

    if (localpathMlp1Pt->next_lp == NULL
        || localpathMlp1Pt->next_lp->mlpLocalpath[IGRAPH_OUTPUT] == NULL) {
      /* It's the last localpath */

      localpathTmp1Pt =
	p3d_extract_softMotion_with_velocities (robotPt, localpath1Pt,
						(double)
						localpath1Pt->
						specific.
						softMotion_data->
						specific->motion[0].
						TimeCumul[3],
						(double)
						localpath1Pt->
						specific.
						softMotion_data->
						specific->motionTime);
      
      end_trajSmPt = append_to_localpath (end_trajSmPt, localpathTmp1Pt);
      
      /* Save the instant at the middle of the tvc segment */
      m_vectOfCVS[indexOfCVS].first = timeLength + 0.5*(localpath1Pt->specific.softMotion_data->specific->motion[0].Times.Tvc);
      indexOfCVS++;
      timeLength += localpathTmp1Pt->length_lp;


    } else {
      localpathTmp1Pt =
        p3d_extract_softMotion_with_velocities (robotPt, localpath1Pt,
                                                (double) localpath1Pt->
                                                specific.softMotion_data->
                                                specific->motion[0].
                                                TimeCumul[3],
                                                (double) localpath1Pt->
                                                specific.softMotion_data->
                                                specific->motion[0].
                                                TimeCumul[4]);

      softMotion_data_copy_into (robotPt,
                                 localpathTmp1Pt->specific.
                                 softMotion_data, softMotion_data_lp1);

      localpathMlp2Pt = localpathMlp1Pt->next_lp;
      localpath2Pt = localpathMlp2Pt->mlpLocalpath[IGRAPH_OUTPUT];
      localpathTmp2Pt =
        p3d_extract_softMotion_with_velocities (robotPt, localpath2Pt,
                                                (double) localpath2Pt->
                                                specific.softMotion_data->
                                                specific->motion[0].
                                                TimeCumul[3],
                                                (double) localpath2Pt->
                                                specific.softMotion_data->
                                                specific->motion[0].
                                                TimeCumul[4]);

      softMotion_data_copy_into (robotPt,
                                 localpathTmp2Pt->specific.
                                 softMotion_data, softMotion_data_lp2);
      localpathTmp2Pt->destroy (robotPt, localpathTmp2Pt);
      /* Set Transition motion */
      if (softMotion_data_lpTrans == NULL) {
        softMotion_data_lpTrans =
          p3d_create_softMotion_data_multilocalpath (robotPt,
              IGRAPH_OUTPUT);
      }

      softMotion_data_lpTrans->q_init =
        localpath1Pt->config_at_distance (robotPt, localpath1Pt,
                                          (double) localpath1Pt->specific.
                                          softMotion_data->specific->
                                          motion[0].TimeCumul[4]);
     p3d_set_and_update_this_robot_conf_multisol(robotPt, softMotion_data_lpTrans->q_init, NULL, 0, localpath1Pt->ikSol);
     p3d_get_robot_config_into(robotPt, &(softMotion_data_lpTrans->q_init));

					  
      softMotion_data_lpTrans->q_end =
        localpath2Pt->config_at_distance (robotPt, localpath2Pt,
                                          (double) localpath2Pt->specific.
                                          softMotion_data->specific->
                                          motion[0].TimeCumul[3]);
     p3d_set_and_update_this_robot_conf_multisol(robotPt, softMotion_data_lpTrans->q_end, NULL, 0, localpath2Pt->ikSol);
     p3d_get_robot_config_into(robotPt, &(softMotion_data_lpTrans->q_end));
					  

      /* Jmax, Amax and Vmax must have the same ratio between lpTmp1 and lpTmp2 */
      for (int v = 0; v < softMotion_data_lp1->nbDofs; v++) {
        softMotion_data_lpTrans->specific->J_max[v] =
          MAX (softMotion_data_lp1->specific->J_max[v],
               softMotion_data_lp2->specific->J_max[v]);
        softMotion_data_lpTrans->specific->A_max[v] =
          MAX (softMotion_data_lp1->specific->A_max[v],
               softMotion_data_lp2->specific->A_max[v]);
        softMotion_data_lpTrans->specific->V_max[v] =
          MAX (softMotion_data_lp1->specific->V_max[v],
               softMotion_data_lp2->specific->V_max[v]);

        softMotion_data_lpTrans->specific->velInit[v] =
          softMotion_data_lp1->specific->motion[v].FC.v;
        softMotion_data_lpTrans->specific->velEnd[v] =
          softMotion_data_lp2->specific->motion[v].IC.v;
        softMotion_data_lpTrans->specific->accInit[v] =
          softMotion_data_lp1->specific->motion[v].FC.a;
        softMotion_data_lpTrans->specific->accEnd[v] =
          softMotion_data_lp2->specific->motion[v].IC.a;
      }
     
      softMotion_data_lpTrans->isPTP = FALSE;
      localpathTransPt = p3d_softMotion_localplanner (robotPt, IGRAPH_OUTPUT,
                                     softMotion_data_lpTrans, softMotion_data_lpTrans->q_init, softMotion_data_lpTrans->q_end, softMotion_data_lpTrans->q_end,
                                     localpath1Pt->ikSol);


      if (localpathTransPt != NULL) {
        collision = p3d_unvalid_localpath_test (robotPt, localpathTransPt, ntest);
      }
      if (localpathTransPt == NULL || collision == TRUE) {

        printf("localpathTransPt : stop motion localpath = %p , collision = %d\n", localpathTransPt, collision);


        localpathTmp1Pt->destroy (robotPt, localpathTmp1Pt);
        /* We add the both original localpaths (with stop motion) */
        localpathTmp1Pt = p3d_extract_softMotion_with_velocities (robotPt, localpath1Pt,
                                                  (double)
                                                  localpath1Pt->
                                                  specific.
                                                  softMotion_data->
                                                  specific->motion[0].
                                                  TimeCumul[3],
                                                  (double)
                                                  localpath1Pt->
                                                  specific.
                                                  softMotion_data->
                                                  specific->motionTime);
        localpathTmp2Pt = p3d_extract_softMotion_with_velocities (robotPt, localpath2Pt,
                                                  0.0,
                                                  (double)
                                                  localpath2Pt->
                                                  specific.
                                                  softMotion_data->
                                                  specific->motion[0].
                                                  TimeCumul[3]);

        end_trajSmPt = append_to_localpath (end_trajSmPt, localpathTmp1Pt);
        nlp++;

	/* Save the instant at the middle of the tvc segment */
	m_vectOfCVS[indexOfCVS].first = timeLength + 0.5*(localpath1Pt->specific.softMotion_data->specific->motion[0].Times.Tvc);
	indexOfCVS++;
	timeLength += localpathTmp1Pt->length_lp;

        end_trajSmPt =
          append_to_localpath (end_trajSmPt, localpathTmp2Pt);
        nlp++;
	timeLength += localpathTmp2Pt->length_lp;

        collision = FALSE;
      } else {
        if (localpathTransPt->activeCntrts != NULL) {
          MY_FREE (localpathTransPt->activeCntrts, int,
                   localpathTransPt->nbActiveCntrts);
        }
        localpathTransPt->nbActiveCntrts = localpath1Pt->nbActiveCntrts;
        localpathTransPt->activeCntrts =
          MY_ALLOC (int, localpathTransPt->nbActiveCntrts);
        for (int v = 0; v < localpath1Pt->nbActiveCntrts; v++) {
          localpathTransPt->activeCntrts[v] =
            localpath1Pt->activeCntrts[v];
          localpathTransPt->activeCntrts[v] =
            localpath1Pt->activeCntrts[v];
          //                                        if (localpathMlp1Pt->nbActiveCntrts > localpath1Pt->nbActiveCntrts) {
          //                                                printf("Error");
          //                                        }
        }
        /* Transition motion is OK */
        end_trajSmPt =
          append_to_localpath (end_trajSmPt, localpathTmp1Pt);
        nlp++;
	/* Save the instant at the middle of the tvc segment */
        m_vectOfCVS[indexOfCVS].first = timeLength + 0.5*(localpathTmp1Pt->specific.softMotion_data->specific->motion[0].Times.Tvc);
	indexOfCVS++;
	timeLength += localpathTmp1Pt->length_lp;

        end_trajSmPt =
          append_to_localpath (end_trajSmPt, localpathTransPt);
        nlp++;
	timeLength += localpathTransPt->length_lp;
      }
    }

    cost += end_trajSmPt->cost (robotPt, end_trajSmPt);
    localpathMlp1Pt = localpathMlp1Pt->next_lp;
    localpathTmp2Pt = NULL;
    localpathTmp1Pt = NULL;
    //p3d_destroy_softMotion_data(robotPt, softMotion_data_lpTrans);
    softMotion_data_lpTrans = NULL;
  }
  /* update the number of local paths */
  trajSmPt->nlp = p3d_compute_traj_nloc (trajSmPt);
  trajSmPt->range_param = p3d_compute_traj_rangeparam (trajSmPt);

  trajSmPt->rob->tcur = trajSmPt;
  g3d_add_traj ( (char *) "traj_SoftMotion", trajSmPt->num);

  return FALSE;
}

int
p3d_convert_traj_to_softMotion (p3d_traj * trajPt, bool param_write_file,
                                std::vector < int >&lp,
                                std::vector < std::vector <
                                double > > &positions, SM_TRAJ & smTraj)
                                {
  p3d_rob *robotPt = trajPt->rob;
  p3d_traj *trajSmPTPPt = NULL; // the point to point trajectory
  p3d_traj *trajSmPt = NULL;  // the smoothed trajectory
  p3d_localpath *end_trajSmPt = NULL;
  p3d_localpath *localpath1Pt = NULL;
  p3d_localpath *localpathTmp1Pt = NULL;
  p3d_localpath *localpathMlp1Pt = trajPt->courbePt;
  configPt qinit = NULL, qgoal = NULL;
  configPt q_init = NULL, q_end = NULL;
  double ltot = 0.0, linLength = 0.0, gain = 0.0;
  int ntest = 0, IGRAPH_OUTPUT = 0;
  
  /* length of trajPt */
  ltot = p3d_ends_and_length_traj (trajPt, &qinit, &qgoal);
  p3d_destroy_config (robotPt, qinit);
  p3d_destroy_config (robotPt, qgoal);
  if (ltot <= 3 * EPS6) {
    /* trajectory too short */
    return FALSE;
  }

  for (int i = 0; i < robotPt->mlp->nblpGp; i++) {
    if (strcmp (robotPt->mlp->mlpJoints[i]->gpName, "upBodySm") == 0) {
      IGRAPH_OUTPUT = i;
    }
  }

  ///////////////////////////////////////////////////////////////////////////
  ////  CONVERT LINEAR TRAJECTORY TO SOFTMOTION POINT TO POINT TRAJECTORY ///
  ///////////////////////////////////////////////////////////////////////////
  trajSmPTPPt = p3d_create_empty_trajectory (robotPt);
  q_init = localpathMlp1Pt->config_at_param (robotPt, localpathMlp1Pt, 0.0);
  p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, localpathMlp1Pt->ikSol);
  p3d_get_robot_config_into(robotPt, &q_init);

  q_end = localpathMlp1Pt->config_at_param (robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);
  p3d_set_and_update_this_robot_conf_multisol(robotPt, q_end, NULL, 0, localpathMlp1Pt->ikSol);
  p3d_get_robot_config_into(robotPt, &q_end);
//         printf("**********************************************\n");
//       jntPt = robotPt->joints[34];
//       int k = jntPt->index_dof;
//       printf("Linear\n");
//       printf("q_init: ");
//       printf("%f %f %f  %f %f %f \n",q_init[k],q_init[k+1],q_init[k+2],q_init[k+3],q_init[k+4],q_init[k+5]);
//       printf("q_end: ");
//       printf("%f %f %f  %f %f %f \n",q_end[k],q_end[k+1],q_end[k+2],q_end[k+3],q_end[k+4],q_end[k+5]);

  p3d_multilocalpath_switch_to_softMotion_groups (robotPt);
  trajSmPTPPt->courbePt = p3d_local_planner_multisol (robotPt, q_init, q_end, localpathMlp1Pt->ikSol);

  if (trajSmPTPPt->courbePt->activeCntrts != NULL) {
    MY_FREE (trajSmPTPPt->courbePt->activeCntrts, int, trajSmPTPPt->courbePt->nbActiveCntrts);
  }
  trajSmPTPPt->courbePt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
  trajSmPTPPt->courbePt->activeCntrts = MY_ALLOC (int, trajSmPTPPt->courbePt->nbActiveCntrts);
  for (int v = 0; v < localpathMlp1Pt->nbActiveCntrts; v++) {
    trajSmPTPPt->courbePt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
  }

// Check if init and end are OK after the softMotion convertion
//       p3d_destroy_config (robotPt, q_init);
//       p3d_destroy_config (robotPt, q_end);
//       q_init = trajSmPTPPt->courbePt->config_at_param (robotPt, trajSmPTPPt->courbePt, 0.0);
//       p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, trajSmPTPPt->courbePt->ikSol);
//       p3d_get_robot_config_into(robotPt, &q_init);
// 
//       q_end = trajSmPTPPt->courbePt->config_at_param (robotPt, trajSmPTPPt->courbePt, trajSmPTPPt->courbePt->length_lp);
//       p3d_set_and_update_this_robot_conf_multisol(robotPt, q_end, NULL, 0, trajSmPTPPt->courbePt->ikSol);
//       p3d_get_robot_config_into(robotPt, &q_end);
//       printf("SoftMotion\n");
//       printf("q_init: ");
//       printf("%f %f %f  %f %f %f \n",q_init[k],q_init[k+1],q_init[k+2],q_init[k+3],q_init[k+4],q_init[k+5]);
//       printf("q_end: ");
//       printf("%f %f %f  %f %f %f \n",q_end[k],q_end[k+1],q_end[k+2],q_end[k+3],q_end[k+4],q_end[k+5]);

  end_trajSmPt = trajSmPTPPt->courbePt;
  TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config (robotPt, q_init);
  NB_TRAJPTP_CONFIG++;
  TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config (robotPt, q_end);
  NB_TRAJPTP_CONFIG++;
  p3d_destroy_config (robotPt, q_init);
  p3d_destroy_config (robotPt, q_end);
  
  // Get the length localPath
  m_vectOfCVS.clear();
  p3d_computeMiddleOfCVSParam(end_trajSmPt->mlpLocalpath[IGRAPH_OUTPUT], localpathMlp1Pt, 0.0 );
  linLength = localpathMlp1Pt->length_lp;
  
  localpathMlp1Pt = localpathMlp1Pt->next_lp;

  while (localpathMlp1Pt != NULL) {
    localpath1Pt = localpathMlp1Pt;
    if (localpath1Pt != NULL) {
      p3d_multilocalpath_switch_to_linear_groups (robotPt);
      q_init = localpathMlp1Pt->config_at_param (robotPt, localpathMlp1Pt, 0.0);
      p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, localpathMlp1Pt->ikSol);
      p3d_get_robot_config_into(robotPt, &q_init);

      q_end = localpathMlp1Pt->config_at_param (robotPt, localpathMlp1Pt, localpathMlp1Pt->length_lp);
      p3d_set_and_update_this_robot_conf_multisol(robotPt, q_end, NULL, 0, localpathMlp1Pt->ikSol);
      p3d_get_robot_config_into(robotPt, &q_end);
//       printf("**********************************************\n");
//       jntPt = robotPt->joints[34];
//       int k = jntPt->index_dof;
//            printf("Linear\n");
//       printf("q_init: ");
//       printf("%f %f %f  %f %f %f \n",q_init[k],q_init[k+1],q_init[k+2],q_init[k+3],q_init[k+4],q_init[k+5]);
//       printf("q_end: ");
//       printf("%f %f %f  %f %f %f \n",q_end[k],q_end[k+1],q_end[k+2],q_end[k+3],q_end[k+4],q_end[k+5]);


      p3d_multilocalpath_switch_to_softMotion_groups (robotPt);
      localpathTmp1Pt = p3d_local_planner_multisol (robotPt, q_init, q_end, localpathMlp1Pt->ikSol);

      if (localpathTmp1Pt->activeCntrts != NULL) {
        MY_FREE (localpathTmp1Pt->activeCntrts, int, localpathTmp1Pt->nbActiveCntrts);
      }
      localpathTmp1Pt->nbActiveCntrts = localpathMlp1Pt->nbActiveCntrts;
      localpathTmp1Pt->activeCntrts = MY_ALLOC (int, localpathTmp1Pt->nbActiveCntrts);
      for (int v = 0; v < localpathMlp1Pt->nbActiveCntrts; v++) {
        localpathTmp1Pt->activeCntrts[v] = localpathMlp1Pt->activeCntrts[v];
      }

//       p3d_destroy_config (robotPt, q_init);
//       p3d_destroy_config (robotPt, q_end);
//       q_init = localpathTmp1Pt->config_at_param (robotPt, localpathTmp1Pt, 0.0);
//       p3d_set_and_update_this_robot_conf_multisol(robotPt, q_init, NULL, 0, localpathTmp1Pt->ikSol);
//       p3d_get_robot_config_into(robotPt, &q_init);
// 
//       q_end = localpathTmp1Pt->config_at_param (robotPt, localpathTmp1Pt,  localpathTmp1Pt->length_lp);
//       p3d_set_and_update_this_robot_conf_multisol(robotPt, q_end, NULL, 0, localpathTmp1Pt->ikSol);
//       p3d_get_robot_config_into(robotPt, &q_end);
//       printf("SoftMotion\n");
//       printf("q_init: ");
//       printf("%f %f %f  %f %f %f \n",q_init[k],q_init[k+1],q_init[k+2],q_init[k+3],q_init[k+4],q_init[k+5]);
//       printf("q_end: ");
//       printf("%f %f %f  %f %f %f \n",q_end[k],q_end[k+1],q_end[k+2],q_end[k+3],q_end[k+4],q_end[k+5]);

      TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config (robotPt, q_end);
      p3d_destroy_config (robotPt, q_init);
      p3d_destroy_config (robotPt, q_end);
      NB_TRAJPTP_CONFIG++;
      
      end_trajSmPt = append_to_localpath (end_trajSmPt, localpathTmp1Pt);
      
      p3d_computeMiddleOfCVSParam(localpathTmp1Pt->mlpLocalpath[IGRAPH_OUTPUT], 
                               localpathMlp1Pt, 
                               linLength );
    }
    linLength += localpathMlp1Pt->length_lp;
    localpathMlp1Pt = localpathMlp1Pt->next_lp;
  }       // END WHILE (localpathMlp1Pt != NULL)

  /* update the number of local paths */
  trajSmPTPPt->nlp = p3d_compute_traj_nloc (trajSmPTPPt);
  trajSmPTPPt->range_param = p3d_compute_traj_rangeparam (trajSmPTPPt);
  trajSmPTPPt->rob->tcur = trajSmPTPPt;
  g3d_add_traj ( (char *) "traj_SoftMotion_PTP", trajSmPTPPt->num);
  printf ("BioMove3D: softMotion point-to-point trajectory OK\n");
  printf("ltot = %f\n",ltot);
  //  printf("nlp = ");
  if (param_write_file == true) {
    p3d_softMotion_export_traj (robotPt, trajSmPTPPt, 0,
                                (char *) "softMotion_PTP_Q.traj",
                                (char *) "softMotion_PTP_Seg.traj",
                                ENV.getBool (Env::plotSoftMotionCurve), lp,
                                positions, smTraj);
    }

    if (fct_draw){(*fct_draw)();}
 g3d_win *win= NULL;
 win= g3d_get_cur_win();
 win->fct_draw2 = &(draw_trajectory_ptp);
 g3d_draw_allwin_active();

  ///////////////////////////////////////////////////////////////////////////
  ////  COMPUTE THE SOFTMOTION SMOOTHED TRAJECTORY                        ///
  ///////////////////////////////////////////////////////////////////////////
  trajSmPt = p3d_create_empty_trajectory (robotPt);
  p3d_convert_ptpTraj_to_smoothedTraj (&gain, &ntest, trajSmPTPPt, trajSmPt);
  robotPt->tcur = trajSmPt;
  /* Write curve into a file for BLTPLOT *///ENV.getBool(Env::plotSoftMotionCurve)
  if (param_write_file == true) {
    smTraj.clear ();
    p3d_softMotion_export_traj (robotPt, trajSmPt, 1,
                                (char *) "softMotion_Smoothed_Q.traj",
                                (char *) "softMotion_Smoothed_Seg.traj",
                                ENV.getBool (Env::plotSoftMotionCurve), lp,
                                positions, smTraj);
  }
  

  // smTraj.print();
  print_MiddleOfCVS();
  p3d_test_middle_of_CVS( trajPt , trajSmPt );
  return FALSE;
}


