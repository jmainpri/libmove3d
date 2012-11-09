
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "gbM/Proto_gbModeles.h"
#include <list>
#include <string>
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/ManipulationStruct.hpp"
#include "../lightPlanner/proto/Manipulation.h"
#include "../lightPlanner/proto/ManipulationPlanner.hpp"
#include "../lightPlanner/proto/ManipulationArmData.hpp"

#include "softMotion/softMotionStruct.h"
static int NB_TRAJPTP_CONFIG = 0;
static configPt TRAJPTP_CONFIG[200];


#define SAMPLING_TIME 0.01
using namespace std;

void p3d_addConfigToArrayToDraw(p3d_rob* r, configPt q) {
  TRAJPTP_CONFIG[NB_TRAJPTP_CONFIG] = p3d_copy_config (r, q);
  NB_TRAJPTP_CONFIG++;
  return;
}

void p3d_clearConfigArrayToDraw(p3d_rob* r) {
  for(int i=0; i<NB_TRAJPTP_CONFIG; i++) {
    p3d_destroy_config(r, TRAJPTP_CONFIG[i]);
  }
  NB_TRAJPTP_CONFIG = 0;
  return;
}

//! get las linear traj
p3d_traj* m_lastLinTraj=NULL;
p3d_traj* p3d_get_last_linear_traj()
{
  return m_lastLinTraj;
}


//! returns the vector of the end of the Constant Velocity Segments
//! The end of the CVS are stored as parameters
//! first :   time for the smoothed softmotion trajectory
//! second :  geometric param along traj (distance)
std::vector< middleOfCVS > m_vectOfCVS;

std::vector< middleOfCVS > p3d_getCVS()
{
  return m_vectOfCVS;
}

bool p3d_getMidCVSTimeOnTraj(int id, double& time)
{
  if( (id >= 0) && (id < m_vectOfCVS.size()) )
  {
    time = m_vectOfCVS[id].second.tau;
    return true;
  }
  else
  {
    return false;
  }
}

//! prints the vector of paramteters
//! of middle the Constant Velocity Segments
void print_MiddleOfCVS(void)
{
  cout << " m_vectOfCVS( " << m_vectOfCVS.size() << " ) = " << endl;
  for(unsigned int i=0;i<m_vectOfCVS.size();i++) {
    cout << "First = " << m_vectOfCVS[i].first << " , " << m_vectOfCVS[i].second.tau << " , " << m_vectOfCVS[i].second.s <<  endl;
  }
}

//! fill the middle of CVS structure the End of The CSV for the geometric path
void p3d_computeMiddleOfCVSParam(int lpId, p3d_localpath* smPath, p3d_localpath* linPath, double trajLength )
{
  middleOfCVS pairTmp;
  if(smPath==NULL || linPath==NULL) {
    return;
  }
  pairTmp.first = lpId;
  pairTmp.second.s =  0.5 * linPath->length_lp + trajLength;
  pairTmp.second.tau = 0.5 * smPath->length_lp;
  m_vectOfCVS.push_back( pairTmp );
}

//! Test the two trajs each configuration
bool p3d_test_middle_of_CVS( p3d_traj * trajPt,
                          p3d_traj * trajSmPt)
{
  bool res = true;
  p3d_rob* rob = trajPt->rob;
  
  for(unsigned int i=0;i<m_vectOfCVS.size();i++)
  {
    configPt q1 = p3d_config_at_distance_along_traj(trajSmPt, m_vectOfCVS[i].second.tau);   
     p3d_set_and_update_this_robot_conf_multisol(rob, q1, NULL, 0, trajSmPt->courbePt->ikSol);
     p3d_get_robot_config_into(rob, &q1);
 
    configPt q2 = p3d_config_at_distance_along_traj(trajPt, m_vectOfCVS[i].second.s);  
    p3d_set_and_update_this_robot_conf_multisol(rob, q2, NULL, 0, trajPt->courbePt->ikSol);
    p3d_get_robot_config_into(rob, &q2);

    if( !p3d_equal_config( rob, q1, q2) ){
      printf("config differ in p3d_test_middle_of_CVS\n");    
      printf("s= %f tau = %f ith CVS : %d lpId %d \n", m_vectOfCVS[i].second.s, m_vectOfCVS[i].second.tau, i,  m_vectOfCVS[i].first);    
      //print_config(rob, q1);
      //print_config(rob, q2);
      for (int j=0; j<rob->nb_dof; j++) {
        if(fabs(q1[j] - q2[j]) > EPS6)
        {
          int i_joint=0;
          p3d_jnt* dof_jnt = p3d_robot_dof_to_jnt(rob,j,&i_joint);
          printf("config differ for dof : %d of joint : %s",j,dof_jnt->name);
          printf(" , q1[j] : %f, q2[j] : %f\n",q1[j],q2[j]);
        }
      }
      res = false;
    }
    else {
#ifdef DEBUG_STATUS
      printf("Ith (%d): OK lpId= %d s= %f tau %f\n",i, m_vectOfCVS[i].first, m_vectOfCVS[i].second.s, m_vectOfCVS[i].second.tau );
#endif
    }
  }
  return res;
}

//! Returns the middle of CVS id
int p3d_getQSwitchIDFromMidCVS(double tau, double t_rep, int* id) 
{  
  int i = 0;
  if(m_vectOfCVS.empty()) {
    printf("m_vectOfCVS is empty\n");
    *id =  1;
    return 1;
  }

  for(i=0; (tau+t_rep) > m_vectOfCVS[i].second.tau; i++) {
    if(i== (m_vectOfCVS.size()-1)) {
      printf("WARNING getQSwitchIDFromMidCVS set id to the last localpath \n");
      *id =  m_vectOfCVS[m_vectOfCVS.size()-1].first;
      return 1;
    }
  }
  *id =  m_vectOfCVS[i].first;
  return 0;
}

void
draw_trajectory_ptp () {
  int i;
  p3d_vector3 p1;
  p3d_vector3 p2;
  if (TRAJPTP_CONFIG[0] == NULL || NB_TRAJPTP_CONFIG <= 0) {
    return;
  }
  p3d_rob *robot = (p3d_rob *) p3d_get_desc_curid (P3D_ROBOT);
  
   if(strcmp(robot->name, "JIDOKUKA_ROBOT") ) {
   return;
   }


  
  p3d_jnt *armJoint = NULL;
  armJoint = p3d_get_robot_jnt_by_name (robot, (char *) "virtual_object");
  if(armJoint){
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
//=======
//    glColor3f (0, 1, 0);
//    g3d_draw_cylinder (p1, p2, 0.01, 16);
//    glColor3f (0, 0, 1);
//    //g3d_draw_solid_sphere(p1[0], p1[1], p1[2], 0.05, 16);
//    //g3d_draw_solid_sphere(p2[0], p2[1], p2[2], 0.05, 16);
//     //g3d_draw_solid_sphere(0, 0, 1.5, 0.05, 16);
//>>>>>>> fix some discontinuities when exporting the softMotion trajectory in the array of Q format
  }
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

void p3d_copy_localpath_constraints_into(p3d_rob* robot, p3d_localpath * lpIn,  p3d_localpath *lpOut) {
      if(lpIn==NULL || lpOut==NULL) {
       return;
      }
      
      if (lpOut->activeCntrts != NULL) {
        MY_FREE (lpOut->activeCntrts, int, lpOut->nbActiveCntrts);
      }
      lpOut->nbActiveCntrts = lpIn->nbActiveCntrts;
      lpOut->activeCntrts = MY_ALLOC (int, lpOut->nbActiveCntrts);
      for (int v = 0; v < lpIn->nbActiveCntrts; v++) {
        lpOut->activeCntrts[v] = lpIn->activeCntrts[v];
      }
#if defined(LIGHT_PLANNER)
	lpOut->isCarryingObject = lpIn->isCarryingObject;
  for(unsigned int i = 0; i < robot->armManipulationData->size(); i++ ){
    lpOut->carriedObject[i] = lpIn->carriedObject[i]; /*!< pointer to the carried object (obstacle environment or robot body) */
  }
#endif
      
      return;
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
					    
  p3d_copy_localpath_constraints_into(robotPt,localpathMlp1Pt, trajSmPt->courbePt);
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

       p3d_copy_localpath_constraints_into(robotPt, localpathMlp1Pt, localpathTmp1Pt);


      end_trajSmPt = append_to_localpath (end_trajSmPt, localpathTmp1Pt);
      
      /* Save the instant at the middle of the tvc segment */
      m_vectOfCVS[indexOfCVS].second.tau = timeLength + 0.5*(localpath1Pt->specific.softMotion_data->specific->motion[0].Times.Tvc);
      indexOfCVS++;
      timeLength += localpathTmp1Pt->length_lp;


    } else {
      localpathTmp1Pt = p3d_extract_softMotion_with_velocities (robotPt, localpath1Pt,
                                                (double) localpath1Pt->
                                                specific.softMotion_data->
                                                specific->motion[0].
                                                TimeCumul[3],
                                                (double) localpath1Pt->
                                                specific.softMotion_data->
                                                specific->motion[0].
                                                TimeCumul[4]);

      p3d_copy_localpath_constraints_into(robotPt, localpathMlp1Pt, localpathTmp1Pt);

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

      p3d_copy_localpath_constraints_into(robotPt, localpathMlp2Pt, localpathTmp2Pt);

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
        p3d_copy_localpath_constraints_into(robotPt, localpathMlp1Pt, localpathTransPt);
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

         p3d_copy_localpath_constraints_into(robotPt, localpathMlp1Pt, localpathTmp1Pt);
	 
        localpathTmp2Pt = p3d_extract_softMotion_with_velocities (robotPt, localpath2Pt,
                                                  0.0,
                                                  (double)
                                                  localpath2Pt->
                                                  specific.
                                                  softMotion_data->
                                                  specific->motion[0].
                                                  TimeCumul[3]);
         p3d_copy_localpath_constraints_into(robotPt, localpathMlp2Pt, localpathTmp2Pt);

        end_trajSmPt = append_to_localpath (end_trajSmPt, localpathTmp1Pt);
        nlp++;

	/* Save the instant at the middle of the tvc segment */
	m_vectOfCVS[indexOfCVS].second.tau = timeLength + 0.5*(localpath1Pt->specific.softMotion_data->specific->motion[0].Times.Tvc);
	indexOfCVS++;
	timeLength += localpathTmp1Pt->length_lp;

        end_trajSmPt =
          append_to_localpath (end_trajSmPt, localpathTmp2Pt);
        nlp++;
	timeLength += localpathTmp2Pt->length_lp;

        collision = FALSE;
      } else {
        /* Transition motion is OK */
        end_trajSmPt =
          append_to_localpath (end_trajSmPt, localpathTmp1Pt);
        nlp++;
	/* Save the instant at the middle of the tvc segment */
        m_vectOfCVS[indexOfCVS].second.tau = timeLength + 0.5*(localpathTmp1Pt->specific.softMotion_data->specific->motion[0].Times.Tvc);
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
  g3d_add_traj ( (char *) "traj_SoftMotion", trajSmPt->num , trajSmPt->rob , trajSmPt );

  return FALSE;
}

int
p3d_convert_traj_to_softMotion (p3d_traj * trajPt, bool smooth, bool param_write_file, bool approximate,
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

  int lpId = 1;
  p3d_clearConfigArrayToDraw(robotPt);
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

  p3d_multilocalpath_switch_to_softMotion_groups (robotPt);
  trajSmPTPPt->courbePt = p3d_local_planner_multisol (robotPt, q_init, q_end, localpathMlp1Pt->ikSol);
#ifdef DEBUG_STATUS  
  printf("trajSmPTPPt->courbePt->range_param = %f\n",trajSmPTPPt->courbePt->range_param); 
#endif

  p3d_copy_localpath_constraints_into(robotPt, localpathMlp1Pt, trajSmPTPPt->courbePt);

  end_trajSmPt = trajSmPTPPt->courbePt;
  p3d_addConfigToArrayToDraw(robotPt, q_init);
  p3d_addConfigToArrayToDraw(robotPt, q_end);

  p3d_destroy_config (robotPt, q_init);
  p3d_destroy_config (robotPt, q_end);
  
  // Get the length localPath
  m_vectOfCVS.clear();
  p3d_computeMiddleOfCVSParam(lpId, end_trajSmPt->mlpLocalpath[IGRAPH_OUTPUT], localpathMlp1Pt, 0.0 );
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

      p3d_multilocalpath_switch_to_softMotion_groups (robotPt);
      localpathTmp1Pt = p3d_local_planner_multisol (robotPt, q_init, q_end, localpathMlp1Pt->ikSol);
      
      p3d_copy_localpath_constraints_into(robotPt, localpathMlp1Pt, localpathTmp1Pt);
      p3d_addConfigToArrayToDraw(robotPt, q_end);

      p3d_destroy_config (robotPt, q_init);
      p3d_destroy_config (robotPt, q_end);

      end_trajSmPt = append_to_localpath (end_trajSmPt, localpathTmp1Pt);
      lpId ++;
      p3d_computeMiddleOfCVSParam(lpId, localpathTmp1Pt->mlpLocalpath[IGRAPH_OUTPUT], 
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
  g3d_add_traj ( (char *) "traj_SoftMotion_PTP", trajSmPTPPt->num , trajSmPTPPt->rob , trajSmPTPPt );
#ifdef DEBUG_STATUS
  printf ("BioMove3D: softMotion point-to-point trajectory OK\n");
#endif

  //check the length of the trajectory 
#ifdef DEBUG_STATUS
  printf("ltot = %f\n",ltot);
#endif
  if(ltot < 0.01) {
    smooth = false;
  }
  if( trajSmPTPPt->range_param == 0.0 )
    {
      return FALSE;
    }
 
  //  printf("nlp = ");
//  if (param_write_file == true) {
//    p3d_softMotion_export_traj (robotPt, trajSmPTPPt, 0,
//                                (char *) "softMotion_PTP_Q.traj",
//                                (char *) "softMotion_PTP_Seg.traj",ENV.getBool (Env::writeSoftMotionFiles),
//                                ENV.getBool (Env::plotSoftMotionCurve), SAMPLING_TIME, lp,
//                                positions, smTraj);
//    }


if(smooth == false)
  {
    robotPt->tcur = trajSmPTPPt;
    if (param_write_file == true) {
      smTraj.clear ();
      p3d_softMotion_export_traj (robotPt, trajSmPTPPt, 0,
                                  (char *) "softMotion_PTP_Q.traj",
                                  (char *) "softMotion_PTP_Seg.traj",ENV.getBool (Env::writeSoftMotionFiles),
                                  ENV.getBool (Env::plotSoftMotionCurve), SAMPLING_TIME, lp,
                                  positions, smTraj);
    }

} else {


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
                                (char *) "softMotion_Smoothed_Seg.traj",ENV.getBool (Env::writeSoftMotionFiles),
                                ENV.getBool (Env::plotSoftMotionCurve), SAMPLING_TIME, lp,
                                positions, smTraj);
  }
  
}
///////////////////////////////////////////////////



if(approximate == true) {
   std::vector< std::vector<SM_COND> > discTraj;
   discTraj.resize(positions[0].size());

   for(unsigned int i=0; i<discTraj.size(); i++) {
    discTraj[i].resize(positions.size());
   }

   for(unsigned int i=0; i< positions.size(); i++) {
    for(unsigned int j=0; j<positions[0].size(); j++) {
     discTraj[j][i].x = positions[i][j];
     if(i>0) {
      discTraj[j][i].v = (discTraj[j][i].x - discTraj[j][i-1].x)/SAMPLING_TIME;
      if(i>1) {
       discTraj[j][i].a = (discTraj[j][i].v - discTraj[j][i-1].v)/SAMPLING_TIME;
      } else {
       discTraj[j][i].a = 0.0;
      }
     } else {
      discTraj[j][i].v = 0.0;
      discTraj[j][i].a = 0.0;
     }
    }
   }

   std::vector<double> jmax = smTraj.jmax;
   std::vector<double> amax = smTraj.amax;
   std::vector<double> vmax = smTraj.vmax;

   smTraj.clear();
    /* function approximate :
     *  PARAMETER:
     * std::vector< std::vector<SM_COND> > discTraj,
     * double samplingTime,
     * double errorPosMax,
     * double errorVelMax,
     * int trajId
     * RETURN SM_TRAJ
     */
   smTraj.approximate(discTraj, SAMPLING_TIME, 0.01, 0.1, 36, true);
   smTraj.jmax = jmax;
   smTraj.amax = amax; 
   smTraj.vmax = vmax;  
   if(ENV.getBool (Env::writeSoftMotionFiles)) {
    smTraj.save((char*)"move3dSoftMotion_Seg.traj");
   }
// std::vector<double> maxVel;
// maxVel.resize(7);
// 
// for(uint i=0; i<maxVel.size(); i++) {
//   maxVel[i] = 1.96;
// }
// 
// SM_LIMITS limits;
// limits.maxJerk = 9.0;
// limits.maxAcc = 3.0;
// limits.maxVel = 1.0;
//     
// smTraj.computeMaxTimeScaleVector(maxVel, SAMPLING_TIME, limits);
    
}

//   if(file != NULL) {
//     fclose(file);
//     printf("File %s created\n", "totoXav");
//   }
// 
// 
// 
//  if ((file = fopen("move3dTotoXav2","w+"))==NULL) {
//     printf("cannot open File %s", "move3dtotoXav");
//   }
// 
// if(file != NULL) {
// //  fprintf(file,"%d\n", positions.size());
// // fprintf(file,"%d\n", positions[0].size());
//  for(unsigned int w=0; w<discTraj[0].size();w++){
// // 	  fprintf(file,"%d",w);
// 	  for(int u=0; u<discTraj.size(); u++) {
// 	   fprintf(file,"%f %f %f ", discTraj[u][w].x, discTraj[u][w].v, discTraj[u][w].a);
// 	  }
// 	  fprintf(file,"\n");
// 	}
// }
// 
//   if(file != NULL) {
//     fclose(file);
//     printf("File %s created\n", "totoXav");
//   }


/////////////////////////////////////////////////////

//  smTraj.plot();

//  print_MiddleOfCVS();
  if(smooth == true) {
     p3d_test_middle_of_CVS( trajPt , trajSmPt );
  } else {
     p3d_test_middle_of_CVS( trajPt ,  trajSmPTPPt);
  }

  //// HERE THE EXAMPLE TO USE THE FUNCTION THAT DETERMINE THE INDEX OF Q_SWITCH
  int id=0;
  double tau =  m_vectOfCVS[m_vectOfCVS.size()-1].second.tau /3.0; 
  double t_rep = 2.0; // in second
  p3d_getQSwitchIDFromMidCVS(tau, t_rep, &id); 
#ifdef DEBUG_STATUS
  printf("the index of the q_switch is %d\n", id);
#endif

  m_lastLinTraj = trajPt;

//      if (fct_draw){(*fct_draw)();}
// g3d_win *win= NULL;
// win= g3d_get_cur_win();
// win->fct_draw2 = &(draw_trajectory_ptp);
// g3d_draw_allwin_active();
  return FALSE;
}


