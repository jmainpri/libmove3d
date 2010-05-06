#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "include/hri_bitmap_util.h"
#include "include/hri_bitmap_draw.h"
#include "include/hri_bitmap_cost.h"
#include "include/hri_bitmap_bin_heap.h"
//#include "hrp2_gik.h"


/* similar to M_SQRT2 in math.h*/
#ifndef M_SQRT3
#define M_SQRT3 1.732050807568877294
#endif

#ifndef M_SQRT5
#define M_SQRT5 2.236067977499789696 
#endif


#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

double M3D_to_HRP2_GIK_sitting_Z_shift=0.475;//+0.16;//0.510+0.16;//0.622+0.16;//0.510-0.112=0.398//This is the value which needs to be added to the M3D z value of robot to synchronize with the z value of HRP2_GIK because for M3D the z is the height of foot whereas for HRP2_GIK z is the height of the waist. 
double M3D_to_HRP2_GIK_half_sitting_Z_shift=0.6487;
////double M3D_to_HRP2_GIK_half_sitting_Z_shift=0.8487;
extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting

struct gik_solution curr_gik_sol;
struct gik_solution HRP2_GIK_sol;// It will store entire configuration to reach from the start to the goal of the entire path

struct SOLUTION_CONFIGS_FOR_HRP2 cur_gik_sol_configs;

extern hri_bitmapset * ACBTSET;

extern int SHOW_HRP2_GIK_SOL;

extern int SKIP_FIRST_CONFIG;

static configPt RS_robotq = NULL;    

int M3D_GIK_TEST();

p3d_vector3 robot_hand_reach_goal[1000];
int cur_i;
p3d_vector3 to_reach_target;

//To tmp store the current positions of the hand returned by GIK, remove this to save space and time
double HRP2_hand_pos_sequence[3000][3];
int no_HRP2_hand_pos=0;

int HRP2_GIK_path_calculated=0;

point_co_ordi right_hand_rest_pos;//To store the position of the right hand in the rest position

point_co_ordi point_of_curr_collision;// To stoe current point in 3d for which GIK solution is having collision


int THUMB_UP_CONSTRAINT=0; 
int HRP2_CURRENT_TASK=0;//1 for take object, 2 for put object, 3 for return to rest position
int SKIP_FIRST_CONFIG=0;

int HRP2_HAND_spline_path_calculated=0;

point_co_ordi curr_AStar_path[500];
int no_pts_curr_AStar_path=0;

double z_val_of_grasp_point;

int SHOW_HRP2_ENTIRE_GIK_SOL=1;
double qs_tmp[3];
double qf_tmp[3];

extern p3d_env *envPt;

extern candidate_poins_for_task candidate_points_to_put;
extern candidate_poins_for_task candidate_points_to_show;
extern candidate_poins_for_task candidate_points_to_hide;

extern int NEED_HRP2_VISIBILITY_UPDATE; //For updating the mightability maps

double attSamplingPeriod = 5e-3; // //Set a sampling period: 5ms second is the value used on HRP2, It should be 5e-3
unsigned int NO_DOF_HRP2=0;

    CjrlRobotDynamicsObjectConstructor<
    dynamicsJRLJapan::DynamicMultiBody,
    //dynamicsJRLJapan::HumanoidDynamicMultiBody,
    Chrp2OptHumanoidDynamicRobot,
    dynamicsJRLJapan::JointFreeflyer,
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> jrlRobotFactory;


   CjrlHumanoidDynamicRobot* attRobot;
   dynamicsJRLJapan::HumanoidDynamicMultiBody *aHDMB;


    //std::string path = "./";
    std::string path = "/home/akpandey/HRP2models/";
    //std::string name = "HRP2.wrl";
    std::string name = "HRP2JRLmain.wrl";
    //aHDMB->parserVRML ( path,name,"./HRP2LinkJointRank.xml" );

    //std::string aName="./HRP2Specificities.xml";
    
    std::string aName="/home/akpandey/HRP2models/HRP2Specificities.xml";

    ChppGikStandingRobot* attStandingRobot;
    ChppGikStandingRobot* attSittingRobot;

   

//Create the constraints defining the tasks.

    std::vector<CjrlGikStateConstraint*> state_constraint_tasks;
     std::vector<CjrlGikStateConstraint*> state_constraint_final_tasks; //It will store the final positions of the task not the incremental positions, and will be used for checking the convergence and stopping conditions
   // printf(" After declearing   state_constraint_tasks \n");

    std::vector<ChppGikVectorizableConstraint*> vectorizable_tasks;
  //  printf(" After declearing   vectorizable_tasks \n");
 
   vectorN combined_weights;

   ChppGikPositionConstraint *pc=NULL;
   ChppGikGazeConstraint *lc=NULL;

  ChppGikParallelConstraint *prlc=NULL;
  std::vector<ChppGikInterpolatedElement*> parallel_constraint_interpolated_elements;
  std::vector<ChppGikInterpolatedElement*> look_at_constraint_interpolated_elements;

/*
int createHumanoidRobot()
{
    CjrlRobotDynamicsObjectConstructor<
    dynamicsJRLJapan::DynamicMultiBody,
    //dynamicsJRLJapan::HumanoidDynamicMultiBody,
    Chrp2OptHumanoidDynamicRobot,
    dynamicsJRLJapan::JointFreeflyer,
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> jrlRobotFactory;

    CjrlHumanoidDynamicRobot* attRobot;
    attRobot = jrlRobotFactory.createhumanoidDynamicRobot();

    dynamicsJRLJapan::HumanoidDynamicMultiBody *aHDMB;
    aHDMB = ( dynamicsJRLJapan::HumanoidDynamicMultiBody* ) attRobot;

    //std::string path = "./";
    std::string path = "/home/akpandey/openrobots/OpenHRP/Controller/IOserver/robot/HRP2JRL/model/";
    //std::string name = "HRP2.wrl";
    std::string name = "HRP2JRLmain.wrl";
    aHDMB->parserVRML ( path,name,"./HRP2LinkJointRank.xml" );
    std::string aName="./HRP2Specificities.xml";
    aHDMB->SetHumanoidSpecificitiesFile ( aName );
    aHDMB->SetTimeStep ( attSamplingPeriod );
    aHDMB->setComputeVelocity ( true );
    aHDMB->setComputeMomentum ( true );
    aHDMB->setComputeCoM ( true );
    aHDMB->setComputeAcceleration ( true );
    aHDMB->setComputeZMP ( true );

    aHDMB->setComputeSkewCoM ( false );
    aHDMB->setComputeAccelerationCoM ( false );
    aHDMB->setComputeBackwardDynamics ( false );



    unsigned int nDof = attRobot->numberDof();
    vectorN halfsittingConf ( nDof );

    //Half sitting
    double dInitPos[40] =
    {
        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

        0.0, 0.0, // chest

        0.0, 0.0, // head

        15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

        10.0, // right hand clench

        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

        10.0, // left hand clench

        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
    };

    //waist x y z
    halfsittingConf ( 0 ) = 0.0;
    halfsittingConf ( 1 ) = 0.0;
    halfsittingConf ( 2 ) = 0.6487;
    //waist roll pitch yaw
    halfsittingConf ( 3 ) = 0.0;
    halfsittingConf ( 4 ) = 0.0;
    halfsittingConf ( 5 ) = 0.0;

    //joints
    for ( unsigned int i=6;i<nDof;i++ )
        halfsittingConf ( i ) = dInitPos[i-6]*M_PI/180;

    zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( halfsittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = 1;
    gazeDir[1] = 0;
    gazeDir[2] = 0;

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    attStandingRobot = new ChppGikStandingRobot ( attRobot );
    attStandingRobot->staticState ( halfsittingConf );
}
*/

/*
//AKP : To transform waist to global coordinate
int waist_to_world_Coordinates(vector3d& inCoords, vector3d& outCoords)
{
    matrix4d m = robot->waist()->currentTransformation();
    vector3d p;
    p[0] = inCoords.x;
    p[1] = inCoords.y;
    p[2] = inCoords.z;
    outCoords = m * p;
    return OK;
}
*/

//HRP2 config to M3D config  
int hrp2_to_M3D_ConfigPt( ghrp2_config_t *cfg, configPt m3dconfig)
{
  // specific for HRP2
  int i;
  /*\
  m3dconfig[ MHP_Q_X ] = 0.0;
  m3dconfig[ MHP_Q_Y ] = 0.0;
  m3dconfig[ MHP_Q_Z ] = 0.0;
  m3dconfig[ MHP_Q_RX ] = 0.0;
  m3dconfig[ MHP_Q_RY ] = 0.0;
  m3dconfig[ MHP_Q_RZ ] = 0.0;
  */
  for(i=0; i<16 ; i++)
    m3dconfig[ MHP_Q_RLEG0 + i ] = cfg->angles[i];

  for(i=0; i<7 ; i++)
    m3dconfig[ MHP_Q_RARM0 + i ] = cfg->angles[ GHRP2_RARM_JOINT0 + i];
  
  for(i=0; i<7 ; i++)
    m3dconfig[ MHP_Q_LARM0 + i ] = cfg->angles[ GHRP2_LARM_JOINT0 + i];

//AKP NOTE: There is mismatch in hand configuration of HRP2 model for HRP2 GIK and for biomove3d so the following part is commented

  for(i=0; i<5 ; i++)
    m3dconfig[ MHP_Q_LHAND0 + i ] = cfg->angles[ GHRP2_LHAND_JOINT0 + i];

  for(i=0; i<5 ; i++)
    m3dconfig[ MHP_Q_RHAND0 + i ] = cfg->angles[ GHRP2_RHAND_JOINT0 + i];


}



int show_gik_sol()
{
 int ctr=100;
 while(SHOW_HRP2_GIK_SOL==1)
 {
 int i=0;
 for(i=0;i<curr_gik_sol.no_configs&&SHOW_HRP2_GIK_SOL==1;i++)
  {
  cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[i]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  }
  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);

 }

  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);

}



int hand_reach()
{
 
}

int create_HRP2_robot_for_GIK_in_Move3d(int state) //state 1 = sitting, 2=half sitting
{
   attRobot = jrlRobotFactory.createhumanoidDynamicRobot();

    aHDMB = ( dynamicsJRLJapan::HumanoidDynamicMultiBody* ) attRobot;


    aHDMB->parserVRML ( path,name,"/home/akpandey/HRP2models/HRP2LinkJointRank.xml" );
    //std::string aName="./HRP2Specificities.xml";
    

    aHDMB->SetHumanoidSpecificitiesFile ( aName );
    aHDMB->SetTimeStep ( attSamplingPeriod );
    aHDMB->setComputeVelocity ( true );
    aHDMB->setComputeMomentum ( true );
    aHDMB->setComputeCoM ( true );
    aHDMB->setComputeAcceleration ( true );
    aHDMB->setComputeZMP ( true );

    aHDMB->setComputeSkewCoM ( false );
    aHDMB->setComputeAccelerationCoM ( false );
    aHDMB->setComputeBackwardDynamics ( false );



    unsigned int nDof = attRobot->numberDof();
    NO_DOF_HRP2=nDof;
   
    //Half sitting
   if(state==2)
   {
   vectorN halfsittingConf ( nDof );
    double dInitPos[40] =
    {
        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

        0.0, 0.0, // chest

        0.0, 0.0, // head

        15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

        10.0, // right hand clench

        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

        10.0, //  left hand clench

        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
    };


    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
    //rob_cur_pos[6]
    
    //AKP : Using hrp2 position in move3D to localize the robot, because HRP2 can not localize itself in the environment
    //waist x y z// Robot waist position and orientation in global system and every task and target are expressed in global frame
    halfsittingConf ( 0 ) = rob_cur_pos[6];
    halfsittingConf ( 1 ) = rob_cur_pos[7];
    printf(" Creating half sitting robot, rob_cur_pos[8]=%lf\n",rob_cur_pos[8]);
    ////rob_cur_pos[8]+=rob_cur_pos[8]+M3D_to_HRP2_GIK_half_sitting_Z_shift;//0.6487;//+M3D_to_HRP2_GIK_half_sitting_Z_shift;  
    ////halfsittingConf ( 2 ) = -rob_cur_pos[8]+M3D_to_HRP2_GIK_half_sitting_Z_shift;//0.6487;//+M3D_to_HRP2_GIK_half_sitting_Z_shift;
    halfsittingConf ( 2 ) = 0.6487;//+M3D_to_HRP2_GIK_half_sitting_Z_shift;      
    
    rob_cur_pos[8]=0;//halfsittingConf(2);
     printf(" After resetting rob_cur_pos[8]=%lf\n",rob_cur_pos[8]);
    //MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    ////halfsittingConf ( 0 ) = 0.0;
    ////halfsittingConf ( 1 ) = 0.0;
    ////halfsittingConf ( 2 ) = 0.6487;
     
    
    //waist roll pitch yaw
    halfsittingConf ( 3 ) = 0.0;
    halfsittingConf ( 4 ) = 0.0;
    halfsittingConf ( 5 ) = 0.0;

    // AKP : Using halfsitting configuration to set roll, pitch, yaw of hrp2 model in move3d
    rob_cur_pos[MHP_Q_RX] = halfsittingConf(3);
    rob_cur_pos[MHP_Q_RY] = halfsittingConf(4);
    rob_cur_pos[MHP_Q_RZ] = halfsittingConf(5);

   
    int i=0;
    //joints
    for (  i=6;i<nDof;i++ )
        halfsittingConf ( i ) = dInitPos[i-6]*M_PI/180;

     
     //Assuming that half sitting configuration of HRP2 is synchronized with real HRP2, the half sitting HRP2 model will be copied from the move3d model
    
  for(i=0; i<16 ; i++)
    	halfsittingConf(i+6) = rob_cur_pos[ MHP_Q_RLEG0 + i ];

  for(i=0; i<7 ; i++)
    halfsittingConf(i + GHRP2_RARM_JOINT0 + 6) = rob_cur_pos[ MHP_Q_RARM0 + i ];
  
  for(i=0; i<7 ; i++)
   {
    
   halfsittingConf(i+GHRP2_LARM_JOINT0+6)=rob_cur_pos[ MHP_Q_LARM0 + i ] ;
   }

    p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos);
    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */

    zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( halfsittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = cos(10*M_PI/180);
    gazeDir[1] = 0;
    gazeDir[2] = -sin(10*M_PI/180);

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    attStandingRobot = new ChppGikStandingRobot ( attRobot );
    attStandingRobot->staticState ( halfsittingConf );
   }//END  if(state==2) for halfsitting 
   
   if(state==1) //For sitting robot
   {
     vectorN sittingConf(nDof);
    
    double dInitPos[40] =
    {
        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

        0.0, 0.0, // chest

        0.0, 0.0, // head

        15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

        10.0, // right hand clench

        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

        10.0, // left hand clench

        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
    };
  //joints
    for ( unsigned int i=6;i<nDof;i++ )
        sittingConf ( i ) = dInitPos[i-6]*M_PI/180;

    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
    //rob_cur_pos[6]
    
    //AKP : Using hrp2 position in move3D to localize the robot, because HRP2 can not localize itself in the environment
    //waist x y z// Robot waist position and orientation in global system and every task and target are expressed in global frame
    sittingConf ( 0 ) = rob_cur_pos[MHP_Q_X];
    sittingConf ( 1 ) = rob_cur_pos[MHP_Q_Y];
    ////////sittingConf ( 2 ) = rob_cur_pos[MHP_Q_Z] + M3D_to_HRP2_GIK_sitting_Z_shift;//0.6487;//M3D_to_HRP2_GIK_sitting_Z_shift;   
    sittingConf ( 2 ) = M3D_to_HRP2_GIK_sitting_Z_shift;//0.6487;//M3D_to_HRP2_GIK_sitting_Z_shift;   
    printf(" Creating sitting robot, rob_cur_pos[8]=%lf and sittingConf(2)=%lf\n",rob_cur_pos[MHP_Q_Z], sittingConf(2));
    //MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    ////halfsittingConf ( 0 ) = 0.0;
    ////halfsittingConf ( 1 ) = 0.0;
    ////halfsittingConf ( 2 ) = 0.6487;
     
    
    //waist roll pitch yaw
    sittingConf ( 3 ) = 0;////rob_cur_pos[MHP_Q_RX];
    sittingConf ( 4 ) = 0;////rob_cur_pos[MHP_Q_RY];
    sittingConf ( 5 ) = 0;////rob_cur_pos[MHP_Q_RZ];

    //waist roll pitch yaw
    sittingConf ( 3 ) = rob_cur_pos[MHP_Q_RX];
    sittingConf ( 4 ) = rob_cur_pos[MHP_Q_RY];
    sittingConf ( 5 ) = rob_cur_pos[MHP_Q_RZ];

    
    //Assuming that sitting configuration of HRP2 is synchronized with real HRP2, the sitting HRP2 model will be copied from the move3d model
    int i;
  for(i=0; i<16 ; i++)
    	sittingConf(i+6) = rob_cur_pos[ MHP_Q_RLEG0 + i ];

  for(i=0; i<7 ; i++)
    sittingConf(i + GHRP2_RARM_JOINT0 + 6) = rob_cur_pos[ MHP_Q_RARM0 + i ];
  
  for(i=0; i<7 ; i++)
   {
    
   sittingConf(i+GHRP2_LARM_JOINT0+6)=rob_cur_pos[ MHP_Q_LARM0 + i ] ;
   }


//AKP NOTE: There is some mismatch in the model of HRP2 for GIK and the HRP2 model in biomove3d
   for(i=0; i<5 ; i++)
   sittingConf(i+ GHRP2_LHAND_JOINT0+6) = rob_cur_pos[ MHP_Q_LHAND0 + i ] ;

  for(i=0; i<5 ; i++)
  sittingConf(i+ GHRP2_RHAND_JOINT0+6) = rob_cur_pos[ MHP_Q_RHAND0 + i ] ;
  
   zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( sittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = cos(10*M_PI/180);
    gazeDir[1] = 0;
    gazeDir[2] = -sin(10*M_PI/180);

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    attStandingRobot = new ChppGikStandingRobot ( attRobot );
    attStandingRobot->staticState ( sittingConf );
   
    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
   
   }
 
 //g3d_draw_env();
 //fl_check_forms();
 //g3d_draw_allwin_active();

}

int create_HRP2_robot(int state) //state 1 = sitting, 2=half sitting
{
   attRobot = jrlRobotFactory.createhumanoidDynamicRobot();

    aHDMB = ( dynamicsJRLJapan::HumanoidDynamicMultiBody* ) attRobot;


    aHDMB->parserVRML ( path,name,"/home/akpandey/HRP2models/HRP2LinkJointRank.xml" );
    //std::string aName="./HRP2Specificities.xml";
    

    aHDMB->SetHumanoidSpecificitiesFile ( aName );
    aHDMB->SetTimeStep ( attSamplingPeriod );
    aHDMB->setComputeVelocity ( true );
    aHDMB->setComputeMomentum ( true );
    aHDMB->setComputeCoM ( true );
    aHDMB->setComputeAcceleration ( true );
    aHDMB->setComputeZMP ( true );

    aHDMB->setComputeSkewCoM ( false );
    aHDMB->setComputeAccelerationCoM ( false );
    aHDMB->setComputeBackwardDynamics ( false );



    unsigned int nDof = attRobot->numberDof();
    NO_DOF_HRP2=nDof;
   
    //Half sitting
   if(state==2)
   {
   vectorN halfsittingConf ( nDof );
    double dInitPos[40] =
    {
        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

        0.0, 0.0, // chest

        0.0, 0.0, // head

        15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

        10.0, // right hand clench

        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

        10.0, //  left hand clench

        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
    };


    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
    //rob_cur_pos[6]
    
    //AKP : Using hrp2 position in move3D to localize the robot, because HRP2 can not localize itself in the environment
    //waist x y z// Robot waist position and orientation in global system and every task and target are expressed in global frame
    halfsittingConf ( 0 ) = rob_cur_pos[6];
    halfsittingConf ( 1 ) = rob_cur_pos[7];
    printf(" Creating half sitting robot, rob_cur_pos[8]=%lf\n",rob_cur_pos[8]);
    ////rob_cur_pos[8]+=rob_cur_pos[8]+M3D_to_HRP2_GIK_half_sitting_Z_shift;//0.6487;//+M3D_to_HRP2_GIK_half_sitting_Z_shift;  
    ////halfsittingConf ( 2 ) = -rob_cur_pos[8]+M3D_to_HRP2_GIK_half_sitting_Z_shift;//0.6487;//+M3D_to_HRP2_GIK_half_sitting_Z_shift;
    halfsittingConf ( 2 ) = 0.6487;//+M3D_to_HRP2_GIK_half_sitting_Z_shift;      
    
    rob_cur_pos[8]=0;//halfsittingConf(2);
     printf(" After resetting rob_cur_pos[8]=%lf\n",rob_cur_pos[8]);
    //MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    ////halfsittingConf ( 0 ) = 0.0;
    ////halfsittingConf ( 1 ) = 0.0;
    ////halfsittingConf ( 2 ) = 0.6487;
     
    
    //waist roll pitch yaw
    halfsittingConf ( 3 ) = 0.0;
    halfsittingConf ( 4 ) = 0.0;
    halfsittingConf ( 5 ) = 0.0;

    // AKP : Using halfsitting configuration to set roll, pitch, yaw of hrp2 model in move3d
    rob_cur_pos[MHP_Q_RX] = halfsittingConf(3);
    rob_cur_pos[MHP_Q_RY] = halfsittingConf(4);
    rob_cur_pos[MHP_Q_RZ] = halfsittingConf(5);

   
    int i=0;
    //joints
    for (  i=6;i<nDof;i++ )
        halfsittingConf ( i ) = dInitPos[i-6]*M_PI/180;

     // AKP : Using halfsitting configuration to set joints of hrp2 model in move3d
   for(i=0; i<12 ; i++)
    rob_cur_pos[ MHP_Q_RLEG0 + i ] = halfsittingConf[i + GHRP2_RLEG_JOINT0 + 6];

   for(i=0; i<7 ; i++)
   rob_cur_pos[ MHP_Q_RARM0 + i ]= halfsittingConf(i + GHRP2_RARM_JOINT0 + 6);
  
  for(i=0; i<7 ; i++)
   {
    
   rob_cur_pos[ MHP_Q_LARM0 + i ]=halfsittingConf(i+GHRP2_LARM_JOINT0+6) ;
   }

/*
   for(i=0; i<5 ; i++)
   rob_cur_pos[ MHP_Q_LHAND0 + i ]=halfsittingConf(i+ GHRP2_LHAND_JOINT0+6) ;

  for(i=0; i<5 ; i++)
  rob_cur_pos[ MHP_Q_RHAND0 + i ]=halfsittingConf(i+ GHRP2_RHAND_JOINT0+6) ;
*/
  //****AKP NOTE: There is some mismatch in the hand/grip configurations of HRP2 model in Biomove3d and in GIK for HRP2. For the initialization value above, the HRP2 model in Biomove3D closes the parallel gripper completely which infact results into self collision, so taking the values from Biomove3d model to initialize the HRP2 GIK model.
   for(i=0; i<5 ; i++)
   halfsittingConf(i+ GHRP2_LHAND_JOINT0+6) = rob_cur_pos[ MHP_Q_LHAND0 + i ] ;

   for(i=0; i<5 ; i++)
   halfsittingConf(i+ GHRP2_RHAND_JOINT0+6) = rob_cur_pos[ MHP_Q_RHAND0 + i ] ;
  

    p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos);
    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */

    zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( halfsittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = cos(10*M_PI/180);
    gazeDir[1] = 0;
    gazeDir[2] = -sin(10*M_PI/180);

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    attStandingRobot = new ChppGikStandingRobot ( attRobot );
    attStandingRobot->staticState ( halfsittingConf );
   }//END  if(state==2) for halfsitting 
   
   if(state==1) //For sitting robot
   {
     vectorN sittingConf(nDof);
    
    double dInitPos[40] =
    {
        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

        0.0, 0.0, // chest

        0.0, 0.0, // head

        15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

        10.0, // right hand clench

        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

        10.0, // left hand clench

        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
    };
  //joints
    for ( unsigned int i=6;i<nDof;i++ )
        sittingConf ( i ) = dInitPos[i-6]*M_PI/180;

    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
    //rob_cur_pos[6]
    
    //AKP : Using hrp2 position in move3D to localize the robot, because HRP2 can not localize itself in the environment
    //waist x y z// Robot waist position and orientation in global system and every task and target are expressed in global frame
    sittingConf ( 0 ) = rob_cur_pos[MHP_Q_X];
    sittingConf ( 1 ) = rob_cur_pos[MHP_Q_Y];
    ////////sittingConf ( 2 ) = rob_cur_pos[MHP_Q_Z] + M3D_to_HRP2_GIK_sitting_Z_shift;//0.6487;//M3D_to_HRP2_GIK_sitting_Z_shift;
    sittingConf ( 2 ) = M3D_to_HRP2_GIK_sitting_Z_shift;//0.6487;//M3D_to_HRP2_GIK_sitting_Z_shift;        
    printf(" Creating sitting robot, rob_cur_pos[8]=%lf\n",rob_cur_pos[8]);
    //MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    ////halfsittingConf ( 0 ) = 0.0;
    ////halfsittingConf ( 1 ) = 0.0;
    ////halfsittingConf ( 2 ) = 0.6487;
     
    
    //waist roll pitch yaw
    sittingConf ( 3 ) = rob_cur_pos[MHP_Q_RX];
    sittingConf ( 4 ) = rob_cur_pos[MHP_Q_RY];
    sittingConf ( 5 ) = rob_cur_pos[MHP_Q_RZ];

 
    int i;
    for(i=0; i<16 ; i++)
    { 
    	sittingConf(i+6) = rob_cur_pos[ MHP_Q_RLEG0 + i ];
        printf(" joint = %d, val= %lf \n ",MHP_Q_RLEG0 + i,rob_cur_pos[ MHP_Q_RLEG0 + i ]);
    }
  for(i=0; i<7 ; i++)
   {
     sittingConf(i + GHRP2_RARM_JOINT0 + 6) = rob_cur_pos[ MHP_Q_RARM0 + i ];
     printf(" joint = %d, val= %lf \n ",MHP_Q_RARM0 + i,rob_cur_pos[ MHP_Q_RARM0 + i]);

   } 
  for(i=0; i<7 ; i++)
   {
    
   sittingConf(i+GHRP2_LARM_JOINT0+6)=rob_cur_pos[ MHP_Q_LARM0 + i ] ;
   printf(" joint = %d, val= %lf \n ",MHP_Q_LARM0 + i,rob_cur_pos[ MHP_Q_LARM0 + i]);

   }

   for(i=0; i<5 ; i++)
   sittingConf(i+ GHRP2_LHAND_JOINT0+6) = rob_cur_pos[ MHP_Q_LHAND0 + i ] ;

  for(i=0; i<5 ; i++)
  sittingConf(i+ GHRP2_RHAND_JOINT0+6) = rob_cur_pos[ MHP_Q_RHAND0 + i ] ;

  ////printf(
  
   zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( sittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = cos(10*M_PI/180);
    gazeDir[1] = 0;
    gazeDir[2] = -sin(10*M_PI/180);

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    attStandingRobot = new ChppGikStandingRobot ( attRobot );
    attStandingRobot->staticState ( sittingConf );
   
    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
   
   }
 
 //g3d_draw_env();
 //fl_check_forms();
 //g3d_draw_allwin_active();

}

int Hand_Clench_without_GIK ( int for_hand, double clench )
{
    //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


	curr_gik_sol.no_configs=0;

    vector3d dummy;
    double clenchDuration = 2.0;
    CjrlHand* hand;
    if ( for_hand==2 )
        hand =  attStandingRobot->robot()->rightHand();
    else
        hand =  attStandingRobot->robot()->leftHand();

    unsigned int nSamples = ChppGikTools::timetoRank ( 0.0,clenchDuration, attSamplingPeriod ) +1;
    vectorN interpolationVector ( nSamples );
    bool contn = ChppGikTools::minJerkCurve ( clenchDuration,attSamplingPeriod,attStandingRobot->robot()->getHandClench ( hand ) ,0.0,0.0,clench,interpolationVector );

    if ( !contn )
        return false;
ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    gMotion.clear();
    double val;
    printf(" no. of samples for hand clench = %d \n",nSamples);
    for ( unsigned int i=0; i<nSamples; i++ )
    {

        val = interpolationVector ( i );
        /*if ( val > 1-1e-5 )
            val = 1-1e-5;
        else
            if ( val < 1e-5 )
                val = 1e-5;*/
        attStandingRobot->robot()->setHandClench ( hand, val );
        printf(" hand clench val = %lf \n",val);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration(), dummy, dummy, dummy, dummy );

 const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
        return false;

        //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point
    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }
        
        
        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      
//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
      //  printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
        p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        //RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
    // printf(" Before calling hrp2_to_M3D_ConfigPt()\n");
     hrp2_to_M3D_ConfigPt(&gik_sol,RS_robotq);
   //  printf(" After calling hrp2_to_M3D_ConfigPt()\n");
//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     ////curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
     
     curr_gik_sol.configs[curr_gik_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

     p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     hrp2_to_M3D_ConfigPt(&gik_sol,curr_gik_sol.configs[curr_gik_sol.no_configs]);
    // printf(" Returned from hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     curr_gik_sol.configs[curr_gik_sol.no_configs][6]=motionConfig[0];
     curr_gik_sol.configs[curr_gik_sol.no_configs][7]=motionConfig[1];

     if(HRP2_CURRENT_STATE==1)//Means sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_sitting_Z_shift;
     
     if(HRP2_CURRENT_STATE==2)//Means half sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_half_sitting_Z_shift;
     
     //curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-0.6487;
     curr_gik_sol.configs[curr_gik_sol.no_configs][9]=motionConfig[3];
     curr_gik_sol.configs[curr_gik_sol.no_configs][10]=motionConfig[4];
     curr_gik_sol.configs[curr_gik_sol.no_configs][11]=motionConfig[5];
////p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);

     curr_gik_sol.no_configs++;
    }

    //attStandingRobot->staticState ( backupConfig );
    return true;
}


int Hand_Clench ( int for_hand, double clench )
{
    //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
 

    vector3d dummy;
    double clenchDuration = 0.5;
    CjrlHand* hand;
    if ( for_hand==2 )
        hand =  attStandingRobot->robot()->rightHand();
    else
        hand =  attStandingRobot->robot()->leftHand();

    unsigned int nSamples = ChppGikTools::timetoRank ( 0.0,clenchDuration, attSamplingPeriod ) +1;
    vectorN interpolationVector ( nSamples );
    bool contn = ChppGikTools::minJerkCurve ( clenchDuration,attSamplingPeriod,attStandingRobot->robot()->getHandClench ( hand ) ,0.0,0.0,clench,interpolationVector );

    if ( !contn )
        return false;
ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    gMotion.clear();
    double val;
    printf(" no. of samples for hand clench = %d \n",nSamples);

    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

    for ( unsigned int i=0; i<nSamples; i++ )
    {

        val = interpolationVector ( i );
        if ( val > 1-1e-5 )
            val = 1-1e-5;
        else
            if ( val < 1e-5 )
                val = 1e-5;
        attStandingRobot->robot()->setHandClench ( hand, val );
        ////printf(" for sample %d, hand clench val = %lf \n",i,val);


       //********** AKP: Part added by Oussama getting a valid and stable ZMP
    	matrix4d tempInv,tempM4;
	tempM4 =  attStandingRobot->robot()->waist()->currentTransformation();
	MAL_S4x4_INVERSE(tempM4,tempInv,double);
	vector3d zmpWorObs = attStandingRobot->robot()->zeroMomentumPoint();
        vector3d zmpwstObs;
	MAL_S4x4_C_eq_A_by_B(zmpwstObs,tempInv,zmpWorObs);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,zmpwstObs,dummy,zmpWorObs );
        //************

        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
      //  printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();


        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration(), dummy, dummy, dummy, dummy );

 ////const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
       {
        printf(" Can not get s, so returning\n");
        MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
        return false;
       }
        //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point
    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }
        
        
        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      

        //***** Storing for executing real HRP2
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        }
        else
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=0;//attSamplingPeriod;
        } 

/*
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        } 
*/
        ////printf("******* Incrementing cur_gik_sol_configs.no_configs=%d \n",cur_gik_sol_configs.no_configs);
        cur_gik_sol_configs.no_configs++;


//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
      //  printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
       //// p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
     
    }
MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
    //attStandingRobot->staticState ( backupConfig );
    return true;
}


int Hand_Clench_old ( int for_hand, double clench )
{
    //vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


	curr_gik_sol.no_configs=0;

    vector3d dummy;
    double clenchDuration = 0.5;
    CjrlHand* hand;
    if ( for_hand==2 )
        hand =  attStandingRobot->robot()->rightHand();
    else
        hand =  attStandingRobot->robot()->leftHand();

    unsigned int nSamples = ChppGikTools::timetoRank ( 0.0,clenchDuration, attSamplingPeriod ) +1;
    vectorN interpolationVector ( nSamples );
    bool contn = ChppGikTools::minJerkCurve ( clenchDuration,attSamplingPeriod,attStandingRobot->robot()->getHandClench ( hand ) ,0.0,0.0,clench,interpolationVector );

    if ( !contn )
        return false;
ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    gMotion.clear();
    double val;
    printf(" no. of samples for hand clench = %d \n",nSamples);

    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

    for ( unsigned int i=0; i<nSamples; i++ )
    {

        val = interpolationVector ( i );
        if ( val > 1-1e-5 )
            val = 1-1e-5;
        else
            if ( val < 1e-5 )
                val = 1e-5;
        attStandingRobot->robot()->setHandClench ( hand, val );
        ////printf(" for sample %d, hand clench val = %lf \n",i,val);


       //********** AKP: Part added by Oussama getting a valid and stable ZMP
    	matrix4d tempInv,tempM4;
	tempM4 =  attStandingRobot->robot()->waist()->currentTransformation();
	MAL_S4x4_INVERSE(tempM4,tempInv,double);
	vector3d zmpWorObs = attStandingRobot->robot()->zeroMomentumPoint();
        vector3d zmpwstObs;
	MAL_S4x4_C_eq_A_by_B(zmpwstObs,tempInv,zmpWorObs);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,zmpwstObs,dummy,zmpWorObs );
        //************

        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
      //  printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();


        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration(), dummy, dummy, dummy, dummy );

 ////const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
       {
        printf(" Can not get s, so returning\n");
        MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
        return false;
       }
        //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point
    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }
        
        
        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      

        //***** Storing for executing real HRP2
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        }
        else
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=0;//attSamplingPeriod;
        } 

/*
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        } 
*/
        ////printf("******* Incrementing cur_gik_sol_configs.no_configs=%d \n",cur_gik_sol_configs.no_configs);
        cur_gik_sol_configs.no_configs++;


//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
      //  printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
       //// p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
       p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
        //RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
    // printf(" Before calling hrp2_to_M3D_ConfigPt()\n");
     hrp2_to_M3D_ConfigPt(&gik_sol,rob_cur_pos);
   //  printf(" After calling hrp2_to_M3D_ConfigPt()\n");
//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     ////curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
     
     curr_gik_sol.configs[curr_gik_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

     p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     hrp2_to_M3D_ConfigPt(&gik_sol,curr_gik_sol.configs[curr_gik_sol.no_configs]);
    // printf(" Returned from hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     curr_gik_sol.configs[curr_gik_sol.no_configs][6]=motionConfig[0];
     curr_gik_sol.configs[curr_gik_sol.no_configs][7]=motionConfig[1];
     
     if(HRP2_CURRENT_STATE==1)//Means sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_sitting_Z_shift;
     if(HRP2_CURRENT_STATE==2)//Means half sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_half_sitting_Z_shift;
    
     //curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-0.6487;
     curr_gik_sol.configs[curr_gik_sol.no_configs][9]=motionConfig[3];
     curr_gik_sol.configs[curr_gik_sol.no_configs][10]=motionConfig[4];
     curr_gik_sol.configs[curr_gik_sol.no_configs][11]=motionConfig[5];
////p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);

     curr_gik_sol.no_configs++;
    }
MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
    //attStandingRobot->staticState ( backupConfig );
    return true;
}
//int 
//int reset_thumb_up_constraint=0;
int generate_interpolated_orientation_constraint()
{

}

int HRP2_look_at(p3d_vector3 look_at_point_in_global_frame, double task_duration, int state, int use_body_part)//use_body_part=0 means use only the head, 1 means use upper body, 2 means use whole body. NOTE: Option 0 for use body part is not implemented currently
{
  printf(" Inside HRP2_look_at with look_at_point_in_global_frame=(%lf,%lf,%lf)\n",look_at_point_in_global_frame[0],look_at_point_in_global_frame[1],look_at_point_in_global_frame[2]);
  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


	curr_gik_sol.no_configs=0;


	ChppGikSolver gikSolver(*attRobot);

//Select the root of the robot at the right foot (reduces computation complexity)
    gikSolver.rootJoint(*attRobot->rightFoot());

    vector3d targetPoint, localPoint, p, look_point, req_hand_orient;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

//Create the constraints defing the tasks.

    std::vector<CjrlGikStateConstraint*> state_constraint_tasks;
    std::vector<CjrlGikStateConstraint*> state_constraint_final_tasks; //It will store the final positions of the task not the incremental positions, and will be used for checking the convergence and stopping conditions
   // printf(" After declearing   state_constraint_tasks \n");

    std::vector<ChppGikVectorizableConstraint*> vectorizable_tasks;
  //  printf(" After declearing   vectorizable_tasks \n");

   
   
   //// First priority: Foot on the ground
   //// AKP : The foot which will be used for foot on ground constraint should be opposite to the foot which has been selected as the root of the robot. Because the root of the robot will always be fixed on the ground and the another foot could move which we want to be fixed on the ground also.
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    ////Second priority: static Center of Mass
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;
   


    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d gloabl_inReachTarget, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

   
    ////configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    ////p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
    
     look_point[0]=look_at_point_in_global_frame[0];
     look_point[1]=look_at_point_in_global_frame[1];
     look_point[2]=look_at_point_in_global_frame[2];

    ChppGikGazeConstraint lc(*attRobot,look_point);

    ////MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    
    ////********** AKP : Here we add the constraints in the task stack in the desired order of priority **********//
   /* state_constraint_tasks.push_back(&nsfc);
    state_constraint_tasks.push_back(&comc);
    state_constraint_tasks.push_back(&pc);

    //tasks.push_back(&rotc); 
    //rotation_constraint_push=1;

    //tasks.push_back(&prlc);
    state_constraint_tasks.push_back(&lc);
    ////tasks.push_back ( new ChppGikPositionConstraint ( * ( attStandingRobot->robot() ),*joint,lpoint,waist_inReachTarget ) );
    */
    
    vectorN activated;
    ////activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    ////activated =  attStandingRobot->maskFactory()->upperBodyMask();
    
    //// if(use_body_part==0) //means use only head
    //// {
   //// activated =  attStandingRobot->maskFactory()->rightArmMask();
   ////  }
     ////else
     ////{ 
      if(use_body_part==1) //means use upper body 
      {
    activated =  attStandingRobot->maskFactory()->upperBodyMask();
      }
      else
      {
       if(use_body_part==2) //means use whole body
       {
       activated =  attStandingRobot->maskFactory()->wholeBodyMask();
       }
      }
     ////}
    
    
       
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for ( unsigned int i=0;i<combined.size();i++ )
        combined ( i ) *= activated ( i );
    for ( unsigned int i=0;i<6;i++ )
        combined ( i ) = 0.0;

   
    gikSolver.weights ( combined );

//     Attempt solve with a single one step: Prepare the constraints (jacobian and value computation)

        

//Solve
//Create a ChppRobotMotion (where the successive configurations will be stored)
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    vector3d dummy;
    std::vector<double> prevVals, curVals;

    ////std::vector<double> dampers ( state_constraint_tasks.size(),0.2 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.


    //dampers[0] = dampers[1] = 0.0;
    
//Creating interpolated points, for this we need to populate vectorizable_tasks
    //vectorizable_tasks.push_back(&nsfc);
    vectorizable_tasks.clear();
    vectorizable_tasks.push_back(&lc);
    state_constraint_final_tasks.clear();
    state_constraint_final_tasks.push_back(&lc);//Will be used for checking the norm and convergence etc
    
    //vectorizable_tasks.push_back(&rotc); 
    //rotation_constraint_push=1;
    
  
    //tasks.push_back(&prlc);
    ////vectorizable_tasks.push_back(&lc);
    
    std::vector<ChppGikInterpolatedElement*> Elements;
    double startTime = 0.0;
    ////double attTasksDuration=3; //in sec
    double attTasksDuration=task_duration; //in sec
    //printf("Before Interpolating vectorizable_tasks\n");
    for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
    {
     // printf("Interpolating the vectorizable_tasks %d \n",i); 
     Elements.push_back ( new ChppGikInterpolatedElement ( attStandingRobot->robot(),vectorizable_tasks[i],i,startTime,attTasksDuration,attSamplingPeriod ) );
    }
    
  // printf(" After Interpolating vectorizable_tasks\n");
    double time = startTime+attSamplingPeriod;
    double endtime = startTime+attTasksDuration+attSamplingPeriod/2.0;
    bool gikStepSuccess = true;
    //vector3d dummy;
 int while_ctr=0;
 int breakLoop=0;
 RS_robotq = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 int tar_unreachable=false;

configPt rob_actual_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&rob_actual_pos);

    curVals.clear();
    prevVals.clear();

    while (!breakLoop)
    {
      ////printf(" Inside while with time =%lf, while_ctr=%d\n",time, while_ctr);
      state_constraint_tasks.clear();  
      ////state_constraint_final_tasks.clear();  
      if(state==2) //Robot is standing
      { 
      state_constraint_tasks.push_back(&nsfc); //Putting foot on ground constraint
      state_constraint_tasks.push_back(&comc); // Putting COM constraint
      ////state_constraint_final_tasks.push_back(&nsfc); //Putting foot on ground constraint
      ////state_constraint_final_tasks.push_back(&comc); // Putting COM constraint
      }
        //Now putting the element of vectorizable constraints 
       // printf(" Before pushing vectorizable_tasks\n");
        for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
         {
         ////printf(" Pushing vectorizable_tasks %d\n",i); 
         state_constraint_tasks.push_back ( Elements[i]->stateConstraintAtTime ( time ) );
         ////state_constraint_final_tasks.push_back ( Elements[i]->stateConstraintAtTime ( endtime-attSamplingPeriod/2.0 ) );
         }
         std::vector<double> dampers ( state_constraint_tasks.size(),0.8 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.
         if(state==2) //Robot is standing in half sitting position
         {
         dampers[0] = dampers[1] = 0.0; // We strictly want to maintain the foot on ground and com constraints, regardless of whatever big the velocity is 
         }
         ////dampers[3] = 1.5; //For gaze constraint use big values like this 
      //   printf(" After pushing vectorizable_tasks\n");   
   
         
        // vector3d p=Elements[2]->stateConstraintAtTime ( time )->worldTarget();
        time += attSamplingPeriod;
        //pc=Elements[i]->stateConstraintAtTime ( time );
        lc.computeValue(); // Computes the difference between the pc.worldTarget() and the current position of the hand
        vectorN p = lc.value();
        ////robot_hand_reach_goal[while_ctr][0]=p[0];
        ////robot_hand_reach_goal[while_ctr][1]=p[1];
        ////robot_hand_reach_goal[while_ctr][2]=p[2];
        
       
         
       //  printf(" Before gikSolver.prepare()\n");
         gikSolver.prepare(state_constraint_tasks);
       //  printf(" After gikSolver.prepare() and before gikSolver.solve()\n");
  
         ///***///// curVals.clear();
         ///***///// prevVals.clear();
      /*
       for ( unsigned int i = 0; i<state_constraint_tasks.size();i++ )
        {
            prevVals.push_back ( norm_2 ( state_constraint_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }*/
        for ( unsigned int i = 0; i<state_constraint_final_tasks.size()&&while_ctr==0;i++ )
        {
            state_constraint_final_tasks[i]->computeValue(); // It will compute the difference between the current position of the robot and the goal position
            prevVals.push_back ( norm_2 ( state_constraint_final_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }
         //state_constraint_final_tasks.push_back(&pc);//Will be used for checking the norm and convergence etc
         double prev_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
        //attGikSolver->solve( inConstraints );
        gikSolver.solve ( state_constraint_tasks, dampers );
      //  printf(" After gikSolver.solve() and before attStandingRobot->updateRobot()\n");
        attStandingRobot->updateRobot ( gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod );
      //  printf("After attStandingRobot->updateRobot() and before gMotion.appendSample ()\n");


       //********** AKP: Part added by Oussama getting a valid and stable ZMP
    	matrix4d tempInv,tempM4;
	tempM4 =  attStandingRobot->robot()->waist()->currentTransformation();
	MAL_S4x4_INVERSE(tempM4,tempInv,double);
	vector3d zmpWorObs = attStandingRobot->robot()->zeroMomentumPoint();
        vector3d zmpwstObs;
	MAL_S4x4_C_eq_A_by_B(zmpwstObs,tempInv,zmpWorObs);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,zmpwstObs,dummy,zmpWorObs );
        //************

        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
      //  printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
        return false;

    //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point


    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }

        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      


//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
      //  printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
        p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        //RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
    // printf(" Before calling hrp2_to_M3D_ConfigPt()\n");
     hrp2_to_M3D_ConfigPt(&gik_sol,RS_robotq);
   //  printf(" After calling hrp2_to_M3D_ConfigPt()\n");
//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     ////curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
    
      
          p3d_set_and_update_this_robot_conf(ACBTSET->robot,RS_robotq);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, res=%d \n", res);
 
         //Restore the actual position of the robot
       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 

        MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
       //attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, res=%d \n", res);
       
       //attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
 //Restore the actual position of the robot
       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
         MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
       return 0;
       }

   
  
      //***** Storing for executing real HRP2
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        }
        else
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=0;//attSamplingPeriod;
        } 

/*        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        } 
*/
        ////printf("******* Incrementing cur_gik_sol_configs.no_configs=%d \n",cur_gik_sol_configs.no_configs);
        cur_gik_sol_configs.no_configs++;

//curr_gik_sol.

//g3d_draw_env();

      
        
        int i_t;
        curVals.clear();
        /*for ( i_t = 0; i_t<state_constraint_tasks.size();i_t++ )
        {
          //  printf(" Inside for with i_t=%d, state_constraint_tasks.size() =%d \n",i_t,state_constraint_tasks.size()); 
            state_constraint_tasks[i_t]->computeValue();
          //  printf(" after tasks[i_t]->computeValue(); \n");
            state_constraint_tasks[i_t]->value();
          //  printf(" after tasks[i_t]->value(); \n");
            curVals.push_back(norm_2 ( state_constraint_tasks[i_t]->value() ));
            //curVals[i_t] = norm_2 ( state_constraint_tasks[i]->value() );
         //   printf(" after curVals[i_t] \n");
        }*/

        for (  i_t = 0; i_t<state_constraint_final_tasks.size();i_t++ )
        {
            state_constraint_final_tasks[i_t]->computeValue(); // It will compute the difference between the current position of the robot and the goal position
            curVals.push_back ( norm_2 ( state_constraint_final_tasks[i_t]->value() ) );
            
        }
        //std::cout<< " Before calculating norm_2 ( attStandingRobot->robot()->currentVelocity())" << std::endl;
        
        double curr_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
      //  printf(" curr_norm =%lf \n",curr_norm);
        //  Uncomment it for real test, there is some problem with norm_2
        if (  curr_norm < 0.00001 )
        {
            printf(" AKP WARNING : curr_norm =%lf So stop.\n",curr_norm);
            breakLoop = true;
            tar_unreachable=true;
        }
        /*else
        {
         breakLoop = false;
        }
        */
         
      //  printf(" prev_norm = %lf, curr_norm = %lf\n",prev_norm, curr_norm);
        /*
        if (  curr_norm - prev_norm > 0.5 )
        {
            printf(" AKP WARNING : curr_norm - prev_norm = %lf , So stop.\n",curr_norm - prev_norm);
            breakLoop = true;
            tar_unreachable=true;
        }
       */
      //   printf(" After if else norm_2 \n");
         
         //breakLoop = true;
        int i=0;
        //if(state==2) //Robot is standing in half sitting position
        //i=2; //Skip the COM and foot on ground constraint

        ////for (; i<state_constraint_tasks.size();i++)
        for (; i<state_constraint_final_tasks.size();i++)
        {
           // printf(" curVals[%d]=%lf, prevVals[%d] = %lf , \n",i,curVals[i],i , prevVals[i]);
            /*if(curVals[i]<0.0000005)
            {   printf(" AKP WARNING : Target unreachable as curVals[%d]=%lf\n",i,curVals[i]);
                breakLoop = true;
                tar_unreachable=true;
            } */
            double abs_diff=fabs(curVals[i] - prevVals[i]);
            /*if (abs_diff < 0.000005)//No significant progress
            {
                //printf(" state_constraint_final_tasks.size() = %d \n", state_constraint_final_tasks.size());
                printf(" AKP WARNING : fabs(curVals[%d] - prevVals[%d]) = %lf , No singificant progress So stop.\n",i,i,abs_diff);
                //printf(" AKP WARNING :  \n");
                breakLoop = true;
                tar_unreachable=true;
                //break;
            }*/
            if (curVals[i] - prevVals[i] > 0.005)//if (curVals[i] - prevVals[i] > 0.00005)//diverges//if (curVals[i] - prevVals[i] > 0.005)//diverges////if (curVals[i] - prevVals[i] > 0.00005)//diverges
            {
                //printf(" state_constraint_final_tasks.size() = %d \n", state_constraint_final_tasks.size());
                printf(" **** For while_ctr=%d\n",while_ctr);
                printf(" ***** AKP WARNING : curVals[%d] - prevVals[%d] = %lf , Diverging So stop.\n",i,i,curVals[i] - prevVals[i]);
                //printf(" AKP WARNING :  \n");
                breakLoop = true;
                tar_unreachable=true;
                //break;
            }
            else 
            {
            if (curVals[i] < prevVals[i] - 1e-4) //significant progress
             {
             //   printf(" Progress \n")  ;
                //breakLoop = false;
                //break;
             }
            else
             {
              /*if(fabs(prevVals[i] - curVals[i]) < 1e-5&&while_ctr>5)
              { 
               printf(" No progress so stop\n");
              //Might have stucked 
               breakLoop = true;
               tar_unreachable=true;
              }*/
             
             }
           }
        ////prevVals.push_back ( curVals[i] );
        }
        prevVals.clear();
        for (i=0; i<state_constraint_final_tasks.size();i++)
        {
         prevVals.push_back ( curVals[i] );
        }
        //curVals.push_back ( prevVals[i] );
        //breakLoop = true;
 
         // AKP : Tmp attempt to stop the loop, just for test purpose
         if(while_ctr==950)
         { 
         breakLoop=true;
         tar_unreachable=true;
         }
         if( time >= endtime )
         breakLoop=true;

while_ctr++;


    }//END  while ( time < endtime )

 //Restore the actual position of the robot
       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 

  MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
  

//AKP WARNING: We should delete the vectorizable_tasks but it is giving segmentation fault. Need to Debug it
/*
   for ( unsigned int i=0;i<vectorizable_tasks.size();i++ )
     {
        printf(" deleting vectorizable_tasks[%d]\n", i);
        delete vectorizable_tasks[i];
     }
*/
  // printf(" Outside while \n"); 
 /*  
   if(tar_unreachable==true)
   {
   if(hand_by_reach==2)// reach by right hand  
   printf("******** target unreachable by RIGHT hand\n");
   else
   printf("******** target unreachable by LEFT hand\n");
   }
   else
   {
   if(hand_by_reach==2)// reach by right hand  
   printf("<<<<<<< target reachable by RIGHT hand\n");
   else
   printf("<<<<<<< target reachable by LEFT hand\n");
   }
*/
   /*show_gik_sol();
   printf(" After show_gik_sol()\n");

  i=0;
  for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  MY_FREE(curr_gik_sol.configs[i], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure
  } 
  */

/////********* Uncomment below to display the current solution ************/////
/*
 int i=0;
 for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[i]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  }
  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/

/*  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
*/
   if(tar_unreachable==true)
   return 0;
   else
   return 1;
 
}



int HRP2_look_at_old(p3d_vector3 look_at_point_in_global_frame, double task_duration, int state, int use_body_part)//use_body_part=0 means use only the head, 1 means use upper body, 2 means use whole body. NOTE: Option 0 for use body part is not implemented currently
{
  printf(" Inside HRP2_look_at with look_at_point_in_global_frame=(%lf,%lf,%lf)\n",look_at_point_in_global_frame[0],look_at_point_in_global_frame[1],look_at_point_in_global_frame[2]);
  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


	curr_gik_sol.no_configs=0;


	ChppGikSolver gikSolver(*attRobot);

//Select the root of the robot at the right foot (reduces computation complexity)
    gikSolver.rootJoint(*attRobot->rightFoot());

    vector3d targetPoint, localPoint, p, look_point, req_hand_orient;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

//Create the constraints defing the tasks.

    std::vector<CjrlGikStateConstraint*> state_constraint_tasks;
    std::vector<CjrlGikStateConstraint*> state_constraint_final_tasks; //It will store the final positions of the task not the incremental positions, and will be used for checking the convergence and stopping conditions
   // printf(" After declearing   state_constraint_tasks \n");

    std::vector<ChppGikVectorizableConstraint*> vectorizable_tasks;
  //  printf(" After declearing   vectorizable_tasks \n");

   
   
   //// First priority: Foot on the ground
   //// AKP : The foot which will be used for foot on ground constraint should be opposite to the foot which has been selected as the root of the robot. Because the root of the robot will always be fixed on the ground and the another foot could move which we want to be fixed on the ground also.
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    ////Second priority: static Center of Mass
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;
   


    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d gloabl_inReachTarget, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

   
    ////configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    ////p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
    
     look_point[0]=look_at_point_in_global_frame[0];
     look_point[1]=look_at_point_in_global_frame[1];
     look_point[2]=look_at_point_in_global_frame[2];

    ChppGikGazeConstraint lc(*attRobot,look_point);

    ////MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    
    ////********** AKP : Here we add the constraints in the task stack in the desired order of priority **********//
   /* state_constraint_tasks.push_back(&nsfc);
    state_constraint_tasks.push_back(&comc);
    state_constraint_tasks.push_back(&pc);

    //tasks.push_back(&rotc); 
    //rotation_constraint_push=1;

    //tasks.push_back(&prlc);
    state_constraint_tasks.push_back(&lc);
    ////tasks.push_back ( new ChppGikPositionConstraint ( * ( attStandingRobot->robot() ),*joint,lpoint,waist_inReachTarget ) );
    */
    
    vectorN activated;
    ////activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    ////activated =  attStandingRobot->maskFactory()->upperBodyMask();
    
    //// if(use_body_part==0) //means use only head
    //// {
   //// activated =  attStandingRobot->maskFactory()->rightArmMask();
   ////  }
     ////else
     ////{ 
      if(use_body_part==1) //means use upper body 
      {
    activated =  attStandingRobot->maskFactory()->upperBodyMask();
      }
      else
      {
       if(use_body_part==2) //means use whole body
       {
       activated =  attStandingRobot->maskFactory()->wholeBodyMask();
       }
      }
     ////}
    
    
       
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for ( unsigned int i=0;i<combined.size();i++ )
        combined ( i ) *= activated ( i );
    for ( unsigned int i=0;i<6;i++ )
        combined ( i ) = 0.0;

   
    gikSolver.weights ( combined );

//     Attempt solve with a single one step: Prepare the constraints (jacobian and value computation)

        

//Solve
//Create a ChppRobotMotion (where the successive configurations will be stored)
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    vector3d dummy;
    std::vector<double> prevVals, curVals;

    ////std::vector<double> dampers ( state_constraint_tasks.size(),0.2 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.


    //dampers[0] = dampers[1] = 0.0;
    
//Creating interpolated points, for this we need to populate vectorizable_tasks
    //vectorizable_tasks.push_back(&nsfc);
    vectorizable_tasks.push_back(&lc);
    state_constraint_final_tasks.push_back(&lc);//Will be used for checking the norm and convergence etc
    
    //vectorizable_tasks.push_back(&rotc); 
    //rotation_constraint_push=1;
    
  
    //tasks.push_back(&prlc);
    ////vectorizable_tasks.push_back(&lc);
    
    std::vector<ChppGikInterpolatedElement*> Elements;
    double startTime = 0.0;
    ////double attTasksDuration=3; //in sec
    double attTasksDuration=task_duration; //in sec
    //printf("Before Interpolating vectorizable_tasks\n");
    for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
    {
     // printf("Interpolating the vectorizable_tasks %d \n",i); 
     Elements.push_back ( new ChppGikInterpolatedElement ( attStandingRobot->robot(),vectorizable_tasks[i],i,startTime,attTasksDuration,attSamplingPeriod ) );
    }
    
  // printf(" After Interpolating vectorizable_tasks\n");
    double time = startTime+attSamplingPeriod;
    double endtime = startTime+attTasksDuration+attSamplingPeriod/2.0;
    bool gikStepSuccess = true;
    //vector3d dummy;
 int while_ctr=0;
 int breakLoop=0;
 RS_robotq = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 int tar_unreachable=false;


    curVals.clear();
    prevVals.clear();

    while (!breakLoop)
    {
      ////printf(" Inside while with time =%lf, while_ctr=%d\n",time, while_ctr);
      state_constraint_tasks.clear();  
      ////state_constraint_final_tasks.clear();  
      if(state==2) //Robot is standing
      { 
      state_constraint_tasks.push_back(&nsfc); //Putting foot on ground constraint
      state_constraint_tasks.push_back(&comc); // Putting COM constraint
      ////state_constraint_final_tasks.push_back(&nsfc); //Putting foot on ground constraint
      ////state_constraint_final_tasks.push_back(&comc); // Putting COM constraint
      }
        //Now putting the element of vectorizable constraints 
       // printf(" Before pushing vectorizable_tasks\n");
        for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
         {
         ////printf(" Pushing vectorizable_tasks %d\n",i); 
         state_constraint_tasks.push_back ( Elements[i]->stateConstraintAtTime ( time ) );
         ////state_constraint_final_tasks.push_back ( Elements[i]->stateConstraintAtTime ( endtime-attSamplingPeriod/2.0 ) );
         }
         std::vector<double> dampers ( state_constraint_tasks.size(),0.8 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.
         if(state==2) //Robot is standing in half sitting position
         {
         dampers[0] = dampers[1] = 0.0; // We strictly want to maintain the foot on ground and com constraints, regardless of whatever big the velocity is 
         }
         ////dampers[3] = 1.5; //For gaze constraint use big values like this 
      //   printf(" After pushing vectorizable_tasks\n");   
   
         
        // vector3d p=Elements[2]->stateConstraintAtTime ( time )->worldTarget();
        time += attSamplingPeriod;
        //pc=Elements[i]->stateConstraintAtTime ( time );
        lc.computeValue(); // Computes the difference between the pc.worldTarget() and the current position of the hand
        vectorN p = lc.value();
        ////robot_hand_reach_goal[while_ctr][0]=p[0];
        ////robot_hand_reach_goal[while_ctr][1]=p[1];
        ////robot_hand_reach_goal[while_ctr][2]=p[2];
        
       
         
       //  printf(" Before gikSolver.prepare()\n");
         gikSolver.prepare(state_constraint_tasks);
       //  printf(" After gikSolver.prepare() and before gikSolver.solve()\n");
  
         ///***///// curVals.clear();
         ///***///// prevVals.clear();
      /*
       for ( unsigned int i = 0; i<state_constraint_tasks.size();i++ )
        {
            prevVals.push_back ( norm_2 ( state_constraint_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }*/
        for ( unsigned int i = 0; i<state_constraint_final_tasks.size()&&while_ctr==0;i++ )
        {
            state_constraint_final_tasks[i]->computeValue(); // It will compute the difference between the current position of the robot and the goal position
            prevVals.push_back ( norm_2 ( state_constraint_final_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }
         //state_constraint_final_tasks.push_back(&pc);//Will be used for checking the norm and convergence etc
         double prev_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
        //attGikSolver->solve( inConstraints );
        gikSolver.solve ( state_constraint_tasks, dampers );
      //  printf(" After gikSolver.solve() and before attStandingRobot->updateRobot()\n");
        attStandingRobot->updateRobot ( gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod );
      //  printf("After attStandingRobot->updateRobot() and before gMotion.appendSample ()\n");


       //********** AKP: Part added by Oussama getting a valid and stable ZMP
    	matrix4d tempInv,tempM4;
	tempM4 =  attStandingRobot->robot()->waist()->currentTransformation();
	MAL_S4x4_INVERSE(tempM4,tempInv,double);
	vector3d zmpWorObs = attStandingRobot->robot()->zeroMomentumPoint();
        vector3d zmpwstObs;
	MAL_S4x4_C_eq_A_by_B(zmpwstObs,tempInv,zmpWorObs);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,zmpwstObs,dummy,zmpWorObs );
        //************

        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
      //  printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
        return false;

    //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point


    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }

        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      


//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
      //  printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
        p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        //RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
    // printf(" Before calling hrp2_to_M3D_ConfigPt()\n");
     hrp2_to_M3D_ConfigPt(&gik_sol,RS_robotq);
   //  printf(" After calling hrp2_to_M3D_ConfigPt()\n");
//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     ////curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
     
     curr_gik_sol.configs[curr_gik_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

     p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     hrp2_to_M3D_ConfigPt(&gik_sol,curr_gik_sol.configs[curr_gik_sol.no_configs]);
    // printf(" Returned from hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     curr_gik_sol.configs[curr_gik_sol.no_configs][6]=motionConfig[0];
     curr_gik_sol.configs[curr_gik_sol.no_configs][7]=motionConfig[1];
     
    //////// if(state==1)//Means sitting
    //////// curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_sitting_Z_shift;
    //////// if(state==2)//Means half sitting
    //////// curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_half_sitting_Z_shift;

    if(state==1)//Means sitting
    curr_gik_sol.configs[curr_gik_sol.no_configs][8]=M3D_to_HRP2_GIK_sitting_Z_shift;
    if(state==2)//Means half sitting
    curr_gik_sol.configs[curr_gik_sol.no_configs][8]=M3D_to_HRP2_GIK_half_sitting_Z_shift;
     
     //curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-0.6487;
     curr_gik_sol.configs[curr_gik_sol.no_configs][9]=motionConfig[3];
     curr_gik_sol.configs[curr_gik_sol.no_configs][10]=motionConfig[4];
     curr_gik_sol.configs[curr_gik_sol.no_configs][11]=motionConfig[5];
////p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);
      
          p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, res=%d \n", res);
        MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
       //attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, res=%d \n", res);
       
       //attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
         MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
       return 0;
       }

     curr_gik_sol.no_configs++;
   //  printf("curr_gik_sol.no_configs = %d \n",curr_gik_sol.no_configs);
  
      //***** Storing for executing real HRP2
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        }
        else
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=0;//attSamplingPeriod;
        } 

/*        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        } 
*/
        ////printf("******* Incrementing cur_gik_sol_configs.no_configs=%d \n",cur_gik_sol_configs.no_configs);
        cur_gik_sol_configs.no_configs++;

//curr_gik_sol.

//g3d_draw_env();

      
        
        int i_t;
        curVals.clear();
        /*for ( i_t = 0; i_t<state_constraint_tasks.size();i_t++ )
        {
          //  printf(" Inside for with i_t=%d, state_constraint_tasks.size() =%d \n",i_t,state_constraint_tasks.size()); 
            state_constraint_tasks[i_t]->computeValue();
          //  printf(" after tasks[i_t]->computeValue(); \n");
            state_constraint_tasks[i_t]->value();
          //  printf(" after tasks[i_t]->value(); \n");
            curVals.push_back(norm_2 ( state_constraint_tasks[i_t]->value() ));
            //curVals[i_t] = norm_2 ( state_constraint_tasks[i]->value() );
         //   printf(" after curVals[i_t] \n");
        }*/

        for (  i_t = 0; i_t<state_constraint_final_tasks.size();i_t++ )
        {
            state_constraint_final_tasks[i_t]->computeValue(); // It will compute the difference between the current position of the robot and the goal position
            curVals.push_back ( norm_2 ( state_constraint_final_tasks[i_t]->value() ) );
            
        }
        //std::cout<< " Before calculating norm_2 ( attStandingRobot->robot()->currentVelocity())" << std::endl;
        
        double curr_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
      //  printf(" curr_norm =%lf \n",curr_norm);
        //  Uncomment it for real test, there is some problem with norm_2
        if (  curr_norm < 0.00001 )
        {
            printf(" AKP WARNING : curr_norm =%lf So stop.\n",curr_norm);
            breakLoop = true;
            tar_unreachable=true;
        }
        /*else
        {
         breakLoop = false;
        }
        */
         
      //  printf(" prev_norm = %lf, curr_norm = %lf\n",prev_norm, curr_norm);
        /*
        if (  curr_norm - prev_norm > 0.5 )
        {
            printf(" AKP WARNING : curr_norm - prev_norm = %lf , So stop.\n",curr_norm - prev_norm);
            breakLoop = true;
            tar_unreachable=true;
        }
       */
      //   printf(" After if else norm_2 \n");
         
         //breakLoop = true;
        int i=0;
        //if(state==2) //Robot is standing in half sitting position
        //i=2; //Skip the COM and foot on ground constraint

        ////for (; i<state_constraint_tasks.size();i++)
        for (; i<state_constraint_final_tasks.size();i++)
        {
           // printf(" curVals[%d]=%lf, prevVals[%d] = %lf , \n",i,curVals[i],i , prevVals[i]);
            /*if(curVals[i]<0.0000005)
            {   printf(" AKP WARNING : Target unreachable as curVals[%d]=%lf\n",i,curVals[i]);
                breakLoop = true;
                tar_unreachable=true;
            } */
            double abs_diff=fabs(curVals[i] - prevVals[i]);
            /*if (abs_diff < 0.000005)//No significant progress
            {
                //printf(" state_constraint_final_tasks.size() = %d \n", state_constraint_final_tasks.size());
                printf(" AKP WARNING : fabs(curVals[%d] - prevVals[%d]) = %lf , No singificant progress So stop.\n",i,i,abs_diff);
                //printf(" AKP WARNING :  \n");
                breakLoop = true;
                tar_unreachable=true;
                //break;
            }*/
            if (curVals[i] - prevVals[i] > 0.005)//if (curVals[i] - prevVals[i] > 0.00005)//diverges//if (curVals[i] - prevVals[i] > 0.005)//diverges////if (curVals[i] - prevVals[i] > 0.00005)//diverges
            {
                //printf(" state_constraint_final_tasks.size() = %d \n", state_constraint_final_tasks.size());
                printf(" **** For while_ctr=%d\n",while_ctr);
                printf(" ***** AKP WARNING : curVals[%d] - prevVals[%d] = %lf , Diverging So stop.\n",i,i,curVals[i] - prevVals[i]);
                //printf(" AKP WARNING :  \n");
                breakLoop = true;
                tar_unreachable=true;
                //break;
            }
            else 
            {
            if (curVals[i] < prevVals[i] - 1e-4) //significant progress
             {
             //   printf(" Progress \n")  ;
                //breakLoop = false;
                //break;
             }
            else
             {
              /*if(fabs(prevVals[i] - curVals[i]) < 1e-5&&while_ctr>5)
              { 
               printf(" No progress so stop\n");
              //Might have stucked 
               breakLoop = true;
               tar_unreachable=true;
              }*/
             
             }
           }
        ////prevVals.push_back ( curVals[i] );
        }
        prevVals.clear();
        for (i=0; i<state_constraint_final_tasks.size();i++)
        {
         prevVals.push_back ( curVals[i] );
        }
        //curVals.push_back ( prevVals[i] );
        //breakLoop = true;
 
         // AKP : Tmp attempt to stop the loop, just for test purpose
         if(while_ctr==950)
         { 
         breakLoop=true;
         tar_unreachable=true;
         }
         if( time >= endtime )
         breakLoop=true;

while_ctr++;


    }//END  while ( time < endtime )

  MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
  

//AKP WARNING: We should delete the vectorizable_tasks but it is giving segmentation fault. Need to Debug it
/*
   for ( unsigned int i=0;i<vectorizable_tasks.size();i++ )
     {
        printf(" deleting vectorizable_tasks[%d]\n", i);
        delete vectorizable_tasks[i];
     }
*/
  // printf(" Outside while \n"); 
 /*  
   if(tar_unreachable==true)
   {
   if(hand_by_reach==2)// reach by right hand  
   printf("******** target unreachable by RIGHT hand\n");
   else
   printf("******** target unreachable by LEFT hand\n");
   }
   else
   {
   if(hand_by_reach==2)// reach by right hand  
   printf("<<<<<<< target reachable by RIGHT hand\n");
   else
   printf("<<<<<<< target reachable by LEFT hand\n");
   }
*/
   /*show_gik_sol();
   printf(" After show_gik_sol()\n");

  i=0;
  for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  MY_FREE(curr_gik_sol.configs[i], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure
  } 
  */

/////********* Uncomment below to display the current solution ************/////
/*
 int i=0;
 for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[i]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  }
  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/

/*  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
*/
   if(tar_unreachable==true)
   return 0;
   else
   return 1;
 
}

int initialize_constraints_stack_for_standing_HRP2()
{

    

    vector3d targetPoint, localPoint, p, look_point, req_hand_orient;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

    //// First priority: Foot on the ground
   //// AKP : The foot which will be used for foot on ground constraint should be opposite to the foot which has been selected as the root of the robot. Because the root of the robot will always be fixed on the ground and the another foot could move which we want to be fixed on the ground also.
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    ////Second priority: static Center of Mass
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;
  
     state_constraint_tasks.push_back(&nsfc); //Putting foot on ground constraint
     state_constraint_tasks.push_back(&comc); // Putting COM constraint

     
}



int get_HRP2_GIK_sol()
{

//printf(" Inside get_HRP2_GIK_sol()\n");
    ChppGikSolver gikSolver(*attRobot);
//printf(" gikSolver created\n");
    gikSolver.weights ( combined_weights );
//printf(" weight assigend to gikSolver \n");
    //Select the root of the robot at the right foot (reduces computation complexity)
    gikSolver.rootJoint(*attRobot->rightFoot()); 

  // printf(" root joint assigned to gikSolver\n");
    
    
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    //printf(" gMotion created \n");
    vector3d dummy;
    std::vector<double> prevVals, curVals;

   //// printf(" state_constraint_task.size() = %d \n", state_constraint_tasks.size());

    std::vector<double> dampers ( state_constraint_tasks.size(),0.8 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.
         if(HRP2_CURRENT_STATE==2) //Robot is standing in half sitting position
         {
      //   printf(" Making damper[0] and [1] values as 0\n");
         dampers[0] = dampers[1] = 0.0; // We strictly want to maintain the foot on ground and com constraints, regardless of whatever big the velocity is 
         }
     
   //// printf(" Before gikSolver.prepare(state_constraint_tasks)\n");
 
    gikSolver.prepare(state_constraint_tasks);
   //// printf(" After gikSolver.prepare() and before gikSolver.solve()\n");
  
        //attGikSolver->solve( inConstraints );
     gikSolver.solve ( state_constraint_tasks, dampers );
      ////  printf(" After gikSolver.solve() and before attStandingRobot->updateRobot()\n");
     attStandingRobot->updateRobot ( gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod );
      ////  printf("After attStandingRobot->updateRobot() and before gMotion.appendSample ()\n");


       //********** AKP: Part added by Oussama getting a valid and stable ZMP
    	matrix4d tempInv,tempM4;
	tempM4 =  attStandingRobot->robot()->waist()->currentTransformation();
	MAL_S4x4_INVERSE(tempM4,tempInv,double);
	vector3d zmpWorObs = attStandingRobot->robot()->zeroMomentumPoint();
        vector3d zmpwstObs;
	MAL_S4x4_C_eq_A_by_B(zmpwstObs,tempInv,zmpWorObs);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,zmpwstObs,dummy,zmpWorObs );
        //************

        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
      //  printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
        return false;

    //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point


    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }

        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      
//***** Storing for executing real HRP2
        //printf(" Before Storing in cur_gik_sol_configs for executing real HRP2\n");
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        }
        else
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=0;//attSamplingPeriod;
        } 
        ////printf("******* Added cur_gik_sol_configs.no_configs=%d with time = %lf\n",cur_gik_sol_configs.no_configs, cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time);

        cur_gik_sol_configs.no_configs++;
     
    ////////delete state_constraint_tasks[0];
    
}

int HRP2_get_joint_mask(int hand_by_reach, int state, int use_body_part) // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body 
{
 
     vectorN activated; 
    ////activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    ////activated =  attStandingRobot->maskFactory()->upperBodyMask();
    if(hand_by_reach==2)// reach by right hand   
    {
     if(use_body_part==0) //means use only hand
     {
    activated =  attStandingRobot->maskFactory()->rightArmMask();
     }
     else
     { 
      if(use_body_part==1) //means use upper body 
      {
    activated =  attStandingRobot->maskFactory()->upperBodyMask();
      }
      else
      {
       if(use_body_part==2) //means use whole body
       {
       activated =  attStandingRobot->maskFactory()->wholeBodyMask();
       }
      }
     }
    }
    
    if(hand_by_reach==1)// reach by left hand   
    {
    ////activated =  attStandingRobot->maskFactory()->leftArmMask();
     if(use_body_part==0) //means use only hand
     {
    activated =  attStandingRobot->maskFactory()->leftArmMask();
     }
     else
     { 
      if(use_body_part==1) //means use upper body 
      {
    activated =  attStandingRobot->maskFactory()->upperBodyMask();
      }
      else
      {
       if(use_body_part==2) //means use whole body
       {
       activated =  attStandingRobot->maskFactory()->wholeBodyMask();
       }
      }
     }
    }
    
   //For any case activate the neak joints also
   activated(6+GHRP2_HEAD_JOINT0)= 1;
   activated(6+GHRP2_HEAD_JOINT1)= 1;

   for ( unsigned int i=0;i<activated.size();i++ )
   {
    printf("Joint i=%d, activated=%lf\n", i, activated(i) );
   } 
   
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();// How much weight each joint should be given
    vectorN combined = weights;
    for ( unsigned int i=0;i<combined.size();i++ )
        combined ( i ) *= activated ( i );
    for ( unsigned int i=0;i<6;i++ )
        combined ( i ) = 0.0; //for the freeflyer

    combined_weights=combined;

   for ( unsigned int i=0;i<combined_weights.size();i++ )
   {
    printf("Joint i=%d, combined_weights=%lf\n", i, combined_weights(i) );
   } 
   
   
    
   //// gikSolver.weights ( combined );
}

int create_HRP2_look_at_constraint()
{
     vector3d look_point;
     look_point[0]=0;
     look_point[1]=0;
     look_point[2]=0;
     
     lc=new ChppGikGazeConstraint (*attRobot,look_point);
    
}

int delete_HRP2_look_at_constraint()
{
 delete lc;
 ////delete state_constraint_tasks[1];
}

int create_HRP2_hand_pos_constraint(int hand_by_reach)
{
 CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d gloabl_inReachTarget, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    matrix4d curT; //attRobot->rightWrist()->currentTransformation();

    if(hand_by_reach==1)// reach by left hand   
    {
    curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
    curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }

    lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

    


     vector4d hand_center_in_global_frame = curT*lpoint; //Getting the current position of hand in the global frame
     //p3d_vector3 hand_center;
    
 
     gloabl_inReachTarget[0]=hand_center_in_global_frame[0];
     gloabl_inReachTarget[1]=hand_center_in_global_frame[1];
     gloabl_inReachTarget[2]=hand_center_in_global_frame[2];

     pc=new ChppGikPositionConstraint (*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget);
}

int delete_HRP2_hand_pos_constraint()
{
 delete pc;
 ////delete state_constraint_tasks[1];
}

int push_HRP2_look_at_constraint(p3d_vector3 look_at_point_in_global_frame)
{
     vector3d look_point;
     look_point[0]=look_at_point_in_global_frame[0];
     look_point[1]=look_at_point_in_global_frame[1];
     look_point[2]=look_at_point_in_global_frame[2];
     lc->worldTarget(look_point);
     state_constraint_tasks.push_back(lc);
     ////state_constraint_tasks.push_back(new ChppGikGazeConstraint (*attRobot,look_point));
     
}

int get_HRP2_look_at_constraint_interpolation(p3d_vector3 look_at_point_in_global_frame,  double task_duration, double start_time, double sampling_period, int priority)
{
     
     
    vector3d look_point_global;
    look_point_global[0]=look_at_point_in_global_frame[0];
    look_point_global[1]=look_at_point_in_global_frame[1];
    look_point_global[2]=look_at_point_in_global_frame[2];

    lc->worldTarget(look_point_global);
    
    vectorizable_tasks.clear();
    vectorizable_tasks.push_back(lc);
    ////vectorizable_tasks.push_back(new ChppGikParallelConstraint (*( attStandingRobot->robot() ),*joint,lpoint,req_hand_orient_global ));
    look_at_constraint_interpolated_elements.clear(); 

    look_at_constraint_interpolated_elements.push_back(new ChppGikInterpolatedElement( attStandingRobot->robot(), vectorizable_tasks[0],priority,start_time,task_duration,sampling_period ));
  
    printf(" vectorizable_tasks.size=%d \n",vectorizable_tasks.size());
}

int push_HRP2_look_at_constraint_interpolated_element(double at_time)
{
 ////vector3d req_hand_orient_global=parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time );
  //CjrlGikStateConstraint * ptr= parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time );
  ////prlc=(ChppGikParallelConstraint*) parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time );
//// printf("Inside push_parallel_constraint_interpolated_element with value=(%lf, %lf, %lf)\n",req_hand_orient_global[0],req_hand_orient_global[1],req_hand_orient_global[2]);
 
 ////prlc=dynamic_cast<ChppGikParallelConstraint*>(parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time ));
 ////vector3d req_hand_orient_global=prlc->targetVector();
 ////printf("pushing = (%lf, %lf, %lf)\n",req_hand_orient_global[0],req_hand_orient_global[1],req_hand_orient_global[2]);
 state_constraint_tasks.push_back(look_at_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time ));
 
}


int push_HRP2_hand_pos_constraint(p3d_vector3 target_in_global_frame, int hand_by_reach) // hand_by_reach =1 for left hand, 2 for right hand, 
{

   CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d gloabl_inReachTarget, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

    if(hand_by_reach==1)// reach by left hand   
    {
   //// curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
   //// curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }

    lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y, z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
  

     to_reach_target[0]=target_in_global_frame[0];
     to_reach_target[1]=target_in_global_frame[1];   
     to_reach_target[2]=target_in_global_frame[2];
 
     gloabl_inReachTarget[0]=target_in_global_frame[0];
     gloabl_inReachTarget[1]=target_in_global_frame[1];
     gloabl_inReachTarget[2]=target_in_global_frame[2];

     //printf("to_reach_target=%lf, %lf, %lf\n",to_reach_target[0],to_reach_target[1],to_reach_target[2]);  

    //////ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget);    

   
   

    
    ////////state_constraint_tasks.push_back(new ChppGikPositionConstraint (*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget));

    pc->worldTarget(gloabl_inReachTarget);
    pc->localPoint(lpoint);
    pc->joint(joint);
     ////vectorizable_tasks.push_back(&pc);
    ////state_constraint_final_tasks.push_back(&pc);//Will be used for checking the norm and convergence etc
    state_constraint_tasks.push_back(pc);
    

}

////CjrlJoint* joint_for_prl_const;
int create_HRP2_hand_parallel_constraint(int hand_by_reach)
{
    CjrlJoint* joint;
    vector3d wrist_x_axis;
    CjrlHand* hand;
    vector3d req_hand_orient_global, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

    if(hand_by_reach==1)// reach by left hand   
    {
   //// curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
   //// curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }

    wrist_x_axis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    ////wrist_x_axis = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
  
     
    ////////state_constraint_tasks.push_back(new ChppGikPositionConstraint (*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget));

     req_hand_orient_global[0]=1;
     req_hand_orient_global[1]=0;
     req_hand_orient_global[2]=0;

     prlc=new ChppGikParallelConstraint (*( attStandingRobot->robot() ),*joint,wrist_x_axis,req_hand_orient_global );

}

int delete_HRP2_hand_parallel_constraint()
{
 delete prlc;
}


int push_HRP2_hand_parallel_constraint(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach)
{
    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d req_hand_orient_global;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

    if(hand_by_reach==1)// reach by left hand   
    {
   //// curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
   //// curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }

    //lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure
    vector3d laxis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
     
     req_hand_orient_global[0]=req_hand_orientation_in_global_frame[0];
     req_hand_orient_global[1]=req_hand_orientation_in_global_frame[1];
     req_hand_orient_global[2]=req_hand_orientation_in_global_frame[2];
    ////////state_constraint_tasks.push_back(new ChppGikPositionConstraint (*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget));

    prlc->targetVector(req_hand_orient_global);
    prlc->localVector(laxis);
    prlc->joint(joint);
    state_constraint_tasks.push_back(prlc);
    ////state_constraint_tasks.push_back(new ChppGikParallelConstraint (*( attStandingRobot->robot() ),*joint,lpoint,req_hand_orient_global ));
    
}

int push_parallel_constraint_interpolated_element(double at_time)
{
 ////vector3d req_hand_orient_global=parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time );
  //CjrlGikStateConstraint * ptr= parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time );
  ////prlc=(ChppGikParallelConstraint*) parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time );
//// printf("Inside push_parallel_constraint_interpolated_element with value=(%lf, %lf, %lf)\n",req_hand_orient_global[0],req_hand_orient_global[1],req_hand_orient_global[2]);
 
//AKP NOTE: Uncomment following to see the interpolated values
 ////////prlc=dynamic_cast<ChppGikParallelConstraint*>(parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time ));
 ////////vector3d req_hand_orient_global=prlc->targetVector();
 ////////printf("pushing = (%lf, %lf, %lf)\n",req_hand_orient_global[0],req_hand_orient_global[1],req_hand_orient_global[2]);
 state_constraint_tasks.push_back(parallel_constraint_interpolated_elements[0]->stateConstraintAtTime ( at_time ));
 
}


int get_parallel_constraint_interpolation(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach, double task_duration, double start_time, double sampling_period, int priority)
{
    
   CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d req_hand_orient_global;

    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

    if(hand_by_reach==1)// reach by left hand   
    {
   //// curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
   //// curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }

    lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
   vector3d laxis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
   matrix4d cur_wrist_transformation= joint->currentTransformation();
   vector4d localaxis;

   localaxis[0]=laxis[0];
   localaxis[1]=laxis[1];
   localaxis[2]=laxis[2];
   localaxis[3]=1;

   matrix3d R_comp_cur_wrist_transformation;
   int i=0;
   for(i=0;i<3;i++)
   {
    int j=0;
    for(j=0;j<3;j++)
    {
     M3_IJ(R_comp_cur_wrist_transformation,i,j)=M4_IJ(cur_wrist_transformation,i,j);
    }
   }
   //vector4d wrst_vect_in_global_frame=cur_wrist_transformation*localaxis;
   vector3d wrist_vect_in_global_frame=R_comp_cur_wrist_transformation*laxis;
   //wrist_vect_in_global_frame[0]=wrst_vect_in_global_frame[0];
   //wrist_vect_in_global_frame[1]=wrst_vect_in_global_frame[1];
   //wrist_vect_in_global_frame[2]=wrst_vect_in_global_frame[2];

   printf(" ******* Current wrist_vect_in_global_frame : %lf, %lf, %lf ",wrist_vect_in_global_frame[0],wrist_vect_in_global_frame[1],wrist_vect_in_global_frame[2]); 



     
     req_hand_orient_global[0]=req_hand_orientation_in_global_frame[0];
     req_hand_orient_global[1]=req_hand_orientation_in_global_frame[1];
     req_hand_orient_global[2]=req_hand_orientation_in_global_frame[2];
    ////////state_constraint_tasks.push_back(new ChppGikPositionConstraint (*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget));

    

    printf("*******req_hand_orient_global= (%lf, %lf, %lf)\n",req_hand_orient_global[0],req_hand_orient_global[1],req_hand_orient_global[2]);

    prlc->targetVector(req_hand_orient_global);
    prlc->localVector(laxis);
    prlc->joint(joint);
    
    vectorizable_tasks.clear();
    vectorizable_tasks.push_back(prlc);
    ////vectorizable_tasks.push_back(new ChppGikParallelConstraint (*( attStandingRobot->robot() ),*joint,lpoint,req_hand_orient_global ));
    parallel_constraint_interpolated_elements.clear(); 

    parallel_constraint_interpolated_elements.push_back(new ChppGikInterpolatedElement( attStandingRobot->robot(), vectorizable_tasks[0],priority,start_time,task_duration,sampling_period ));
  
    printf(" vectorizable_tasks.size=%d \n",vectorizable_tasks.size());
    ////// return parallel_constraint_interpolated_elements[0].size();
}

int delete_parallel_constraint_interpolation()
{
 delete parallel_constraint_interpolated_elements[0];
}

int HRP2_hand_reach(p3d_vector3 target_in_global_frame, int hand_by_reach, double task_duration, int state, int thumb_up_constraint, int use_body_part) // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. thumb_up_constraint =1 means the x axis of the hand should be parallel to the global z axis, use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body 
{
////int state=1; //AKP : 1 is sitting, 2 is standing 
////create_HRP2_robot(state);

 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


	curr_gik_sol.no_configs=0;


	ChppGikSolver gikSolver(*attRobot);

//Select the root of the robot at the right foot (reduces computation complexity)
    gikSolver.rootJoint(*attRobot->rightFoot());

    vector3d targetPoint, localPoint, p, look_point, req_hand_orient;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

//Create the constraints defining the tasks.

    std::vector<CjrlGikStateConstraint*> state_constraint_tasks;
     std::vector<CjrlGikStateConstraint*> state_constraint_final_tasks; //It will store the final positions of the task not the incremental positions, and will be used for checking the convergence and stopping conditions
   // printf(" After declearing   state_constraint_tasks \n");

    std::vector<ChppGikVectorizableConstraint*> vectorizable_tasks;
  //  printf(" After declearing   vectorizable_tasks \n");

   
   
   //// First priority: Foot on the ground
   //// AKP : The foot which will be used for foot on ground constraint should be opposite to the foot which has been selected as the root of the robot. Because the root of the robot will always be fixed on the ground and the another foot could move which we want to be fixed on the ground also.
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    ////Second priority: static Center of Mass
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;
   


    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d gloabl_inReachTarget, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    ////matrix4d curT; //attRobot->rightWrist()->currentTransformation();

    if(hand_by_reach==1)// reach by left hand   
    {
   //// curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
   //// curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }

    lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
  

     to_reach_target[0]=target_in_global_frame[0];
     to_reach_target[1]=target_in_global_frame[1];   
     to_reach_target[2]=target_in_global_frame[2];
 
     gloabl_inReachTarget[0]=target_in_global_frame[0];
     gloabl_inReachTarget[1]=target_in_global_frame[1];
     gloabl_inReachTarget[2]=target_in_global_frame[2];

     //printf("to_reach_target=%lf, %lf, %lf\n",to_reach_target[0],to_reach_target[1],to_reach_target[2]);  

     ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, gloabl_inReachTarget);    

        
    ////********* AKP : For parallel constraint of the wrist ***********/////
    
     vector3d wrist_x_axis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    //printf(" okay axis in wrist frame : %lf, %lf, %lf ",wrist_x_axis[0],wrist_x_axis[1],wrist_x_axis[2]);
  
    vector3d wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    //printf(" palm axis in wrist frame : %lf, %lf, %lf ",wrist_y_axis[0],wrist_y_axis[1],wrist_y_axis[2]);

    vector3d wrist_z_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    //printf(" showing axis in wrist frame : %lf, %lf, %lf ",wrist_z_axis[0],wrist_z_axis[1],wrist_z_axis[2]);

    //preffered orientation of x axis of the wrist frame in the world coordinate. 
    req_hand_orient[0]=0;   
    req_hand_orient[1]=0;
    req_hand_orient[2]=1;
    ChppGikParallelConstraint prlc( * ( attStandingRobot->robot() ),*joint,wrist_x_axis,req_hand_orient );
    int orientation_constraint_push=0;
    

    ////******** AKP : For 3D rotation constraint of the wrist *************////
    matrix3d desired_wrist_rotation; 
    //AKP : 1st coloumn of the 3D matrix should contain the preffered orientation of the x axis of the wrist frame in the world coordinate, similarly 2nd and 3rd coloumn should contain the y and z axes of the wrist frame in the world coordinate   
    M3_IJ ( desired_wrist_rotation,0,0 ) = 0; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ ( desired_wrist_rotation,1,0 ) = 0;
    M3_IJ ( desired_wrist_rotation,2,0 ) = 1;
    M3_IJ ( desired_wrist_rotation,0,1 ) = -1;
    M3_IJ ( desired_wrist_rotation,1,1 ) = 1;
    M3_IJ ( desired_wrist_rotation,2,1 ) = 0;
    M3_IJ ( desired_wrist_rotation,0,2 ) = 0;
    M3_IJ ( desired_wrist_rotation,1,2 ) = -1;
    M3_IJ ( desired_wrist_rotation,2,2 ) = 0;
  
    ChppGikRotationConstraint rotc(* ( attStandingRobot->robot() ),*joint, desired_wrist_rotation);
    int rotation_constraint_push=0;
     	
    //***********If want to give current orientation of the wrist as the constraint uncommet below part *******//
    /*
      vector3d wrist_x_axis = hand->okayAxisInWristFrame();
        //matrix4d cur_wrist_transformation= attStandingRobot->robot()->rightWrist()->currentTransformation();
        vector3d wrist_x_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_x_axis;
   
        printf(" wrist_x_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_x_axis_in_global_frame[0],wrist_x_axis_in_global_frame[1],wrist_x_axis_in_global_frame[2]); 

        wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        vector3d wrist_y_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_y_axis;    
        
         printf(" wrist_y_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_y_axis_in_global_frame[0],wrist_y_axis_in_global_frame[1],wrist_y_axis_in_global_frame[2]); 

        vector3d wrist_show_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        //vector3d wrist_z_axis;
        wrist_z_axis[0]=wrist_show_axis[0];
        wrist_z_axis[1]=wrist_show_axis[1];
        wrist_z_axis[2]=-wrist_show_axis[2]; //AKP : Because in the robot kinematics chain z axis of wrist frame is opposite to the showing axis

        vector3d wrist_z_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_z_axis;    
        
        printf(" wrist_z_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_z_axis_in_global_frame[0],wrist_z_axis_in_global_frame[1],wrist_z_axis_in_global_frame[2]); 
    //AKP : 1st coloumn of the 3D matrix should contain the preffered orientation of the x axis of the wrist frame in the world coordinate, similarly 2nd and 3rd coloumn should contain the y and z axes of the wrist frame in the world coordinate   
    matrix3d cur_rot;// = rotc.targetOrientation();
    // 1st column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ( cur_rot,0,0 ) = wrist_x_axis_in_global_frame[0]; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ( cur_rot,1,0 ) = wrist_x_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,0 ) = wrist_x_axis_in_global_frame[2];
    
    // 2nd column : Preffered y axis orientation of the wrist frame in the world frame 
    M3_IJ( cur_rot,0,1 ) = wrist_y_axis_in_global_frame[0];
    M3_IJ( cur_rot,1,1 ) = wrist_y_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,1 ) = wrist_y_axis_in_global_frame[2];
 
    // 3rd column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ( cur_rot,0,2 ) = wrist_z_axis_in_global_frame[0];
    M3_IJ( cur_rot,1,2 ) = wrist_z_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,2 ) = wrist_z_axis_in_global_frame[2];
   
    //rotc.targetOrientation(cur_rot);
    ChppGikRotationConstraint rotc(* ( attStandingRobot->robot() ),*joint, cur_rot);
     */
    //***********END If want to give current orientation of the wrist as the constraint *******//


    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
    
     look_point[0]=to_reach_target[0];
     look_point[1]=to_reach_target[1];
     look_point[2]=to_reach_target[2];

    ChppGikGazeConstraint lc(*attRobot,look_point);

    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    
    ////********** AKP : Here we add the constraints in the task stack in the desired order of priority **********//
   /* state_constraint_tasks.push_back(&nsfc);
    state_constraint_tasks.push_back(&comc);
    state_constraint_tasks.push_back(&pc);

    //tasks.push_back(&rotc); 
    //rotation_constraint_push=1;

    //tasks.push_back(&prlc);
    state_constraint_tasks.push_back(&lc);
    ////tasks.push_back ( new ChppGikPositionConstraint ( * ( attStandingRobot->robot() ),*joint,lpoint,waist_inReachTarget ) );
    */
    
    vectorN activated;
    ////activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    ////activated =  attStandingRobot->maskFactory()->upperBodyMask();
    if(hand_by_reach==2)// reach by right hand   
    {
     if(use_body_part==0) //means use only hand
     {
    activated =  attStandingRobot->maskFactory()->rightArmMask();
     }
     else
     { 
      if(use_body_part==1) //means use upper body 
      {
    activated =  attStandingRobot->maskFactory()->upperBodyMask();
      }
      else
      {
       if(use_body_part==2) //means use whole body
       {
       activated =  attStandingRobot->maskFactory()->wholeBodyMask();
       }
      }
     }
    }
    
    if(hand_by_reach==1)// reach by left hand   
    {
    ////activated =  attStandingRobot->maskFactory()->leftArmMask();
     if(use_body_part==0) //means use only hand
     {
    activated =  attStandingRobot->maskFactory()->leftArmMask();
     }
     else
     { 
      if(use_body_part==1) //means use upper body 
      {
    activated =  attStandingRobot->maskFactory()->upperBodyMask();
      }
      else
      {
       if(use_body_part==2) //means use whole body
       {
       activated =  attStandingRobot->maskFactory()->wholeBodyMask();
       }
      }
     }
    }
    
    
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for ( unsigned int i=0;i<combined.size();i++ )
        combined ( i ) *= activated ( i );
    for ( unsigned int i=0;i<6;i++ )
        combined ( i ) = 0.0;

   
    gikSolver.weights ( combined );

//     Attempt solve with a single one step: Prepare the constraints (jacobian and value computation)

        

//Solve
//Create a ChppRobotMotion (where the successive configurations will be stored)
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    vector3d dummy;
    std::vector<double> prevVals, curVals;

    ////std::vector<double> dampers ( state_constraint_tasks.size(),0.2 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.


    //dampers[0] = dampers[1] = 0.0;
    
//Creating interpolated points, for this we need to populate vectorizable_tasks
    //vectorizable_tasks.push_back(&nsfc);
    vectorizable_tasks.push_back(&pc);
    state_constraint_final_tasks.push_back(&pc);//Will be used for checking the norm and convergence etc
    
    //vectorizable_tasks.push_back(&rotc); 
    //rotation_constraint_push=1;
    
    if(thumb_up_constraint==1)
    {
    vectorizable_tasks.push_back(&prlc);
    state_constraint_final_tasks.push_back(&prlc);//Will be used for checking the norm and convergence etc
    }
    //tasks.push_back(&prlc);
    ////vectorizable_tasks.push_back(&lc);
    
    std::vector<ChppGikInterpolatedElement*> Elements;
    double startTime = 0.0;
    ////double attTasksDuration=3; //in sec
    double attTasksDuration=task_duration; //in sec
    //printf("Before Interpolating vectorizable_tasks\n");
    for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
    {
     // printf("Interpolating the vectorizable_tasks %d \n",i); 
     Elements.push_back ( new ChppGikInterpolatedElement ( attStandingRobot->robot(),vectorizable_tasks[i],i,startTime,attTasksDuration,attSamplingPeriod ) );
    }
    
  // printf(" After Interpolating vectorizable_tasks\n");
    double time = startTime+attSamplingPeriod;
    double endtime = startTime+attTasksDuration+attSamplingPeriod/2.0;
    bool gikStepSuccess = true;
    //vector3d dummy;
 int while_ctr=0;
 int breakLoop=0;
 RS_robotq = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 int tar_unreachable=false;


    curVals.clear();
    prevVals.clear();

    while (!breakLoop)
    {
     // printf(" Inside while with time =%lf, while_ctr=%d\n",time, while_ctr);
      state_constraint_tasks.clear();  
      ////state_constraint_final_tasks.clear();  
      if(state==2) //Robot is standing
      { 
      state_constraint_tasks.push_back(&nsfc); //Putting foot on ground constraint
      state_constraint_tasks.push_back(&comc); // Putting COM constraint
      ////state_constraint_final_tasks.push_back(&nsfc); //Putting foot on ground constraint
      ////state_constraint_final_tasks.push_back(&comc); // Putting COM constraint
      }
        //Now putting the element of vectorizable constraints 
       // printf(" Before pushing vectorizable_tasks\n");
        for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
         {
       //  printf(" Pushing vectorizable_tasks %d\n",i); 
         state_constraint_tasks.push_back ( Elements[i]->stateConstraintAtTime ( time ) );
         ////state_constraint_final_tasks.push_back ( Elements[i]->stateConstraintAtTime ( endtime-attSamplingPeriod/2.0 ) );
         }
         std::vector<double> dampers ( state_constraint_tasks.size(),0.8 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.
         if(state==2) //Robot is standing in half sitting position
         {
         dampers[0] = dampers[1] = 0.0; // We strictly want to maintain the foot on ground and com constraints, regardless of whatever big the velocity is 
         }
         ////dampers[3] = 1.5; //For gaze constraint use big values like this 
      //   printf(" After pushing vectorizable_tasks\n");   
   
         
        // vector3d p=Elements[2]->stateConstraintAtTime ( time )->worldTarget();
        time += attSamplingPeriod;
        //pc=Elements[i]->stateConstraintAtTime ( time );
        pc.computeValue(); // Computes the difference between the pc.worldTarget() and the current position of the hand
        vectorN p = pc.value();
        ////robot_hand_reach_goal[while_ctr][0]=p[0];
        ////robot_hand_reach_goal[while_ctr][1]=p[1];
        ////robot_hand_reach_goal[while_ctr][2]=p[2];
        
       
         
       //  printf(" Before gikSolver.prepare()\n");
         gikSolver.prepare(state_constraint_tasks);
       //  printf(" After gikSolver.prepare() and before gikSolver.solve()\n");
  
         ///***///// curVals.clear();
         ///***///// prevVals.clear();
      /*
       for ( unsigned int i = 0; i<state_constraint_tasks.size();i++ )
        {
            prevVals.push_back ( norm_2 ( state_constraint_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }*/
        for ( unsigned int i = 0; i<state_constraint_final_tasks.size()&&while_ctr==0;i++ )
        {
            state_constraint_final_tasks[i]->computeValue(); // It will compute the difference between the current position of the robot and the goal position
            prevVals.push_back ( norm_2 ( state_constraint_final_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }
         //state_constraint_final_tasks.push_back(&pc);//Will be used for checking the norm and convergence etc
         double prev_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
        //attGikSolver->solve( inConstraints );
        gikSolver.solve ( state_constraint_tasks, dampers );
      //  printf(" After gikSolver.solve() and before attStandingRobot->updateRobot()\n");
        attStandingRobot->updateRobot ( gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod );
      //  printf("After attStandingRobot->updateRobot() and before gMotion.appendSample ()\n");


       //********** AKP: Part added by Oussama getting a valid and stable ZMP
    	matrix4d tempInv,tempM4;
	tempM4 =  attStandingRobot->robot()->waist()->currentTransformation();
	MAL_S4x4_INVERSE(tempM4,tempInv,double);
	vector3d zmpWorObs = attStandingRobot->robot()->zeroMomentumPoint();
        vector3d zmpwstObs;
	MAL_S4x4_C_eq_A_by_B(zmpwstObs,tempInv,zmpWorObs);
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,zmpwstObs,dummy,zmpWorObs );
        //************

        ////gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
      //  printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
     //  printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
        return false;

    //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point


    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (unsigned int i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }

        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      
//***** Storing for executing real HRP2
       if((SKIP_FIRST_CONFIG==1&&while_ctr>=1)||(SKIP_FIRST_CONFIG==0))
       { 
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs]=gik_sol;
        if(cur_gik_sol_configs.no_configs>0)
        {
        cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs].time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time+attSamplingPeriod;
        } 
        ////printf("******* Incrementing cur_gik_sol_configs.no_configs=%d \n",cur_gik_sol_configs.no_configs);
        cur_gik_sol_configs.no_configs++;
       }
//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
      //  printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
        p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        //RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
    // printf(" Before calling hrp2_to_M3D_ConfigPt()\n");
     hrp2_to_M3D_ConfigPt(&gik_sol,RS_robotq);
   //  printf(" After calling hrp2_to_M3D_ConfigPt()\n");
//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     ////curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
     
     curr_gik_sol.configs[curr_gik_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

     p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     hrp2_to_M3D_ConfigPt(&gik_sol,curr_gik_sol.configs[curr_gik_sol.no_configs]);
    // printf(" Returned from hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     curr_gik_sol.configs[curr_gik_sol.no_configs][6]=motionConfig[0];
     curr_gik_sol.configs[curr_gik_sol.no_configs][7]=motionConfig[1];
     
     if(state==1)//Means sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_sitting_Z_shift;
     if(state==2)//Means half sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_half_sitting_Z_shift;
     
     //curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-0.6487;
     curr_gik_sol.configs[curr_gik_sol.no_configs][9]=motionConfig[3];
     curr_gik_sol.configs[curr_gik_sol.no_configs][10]=motionConfig[4];
     curr_gik_sol.configs[curr_gik_sol.no_configs][11]=motionConfig[5];
////p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);

     curr_gik_sol.no_configs++;
   //  printf("curr_gik_sol.no_configs = %d \n",curr_gik_sol.no_configs);
  


//curr_gik_sol.

//g3d_draw_env();

      
        
        int i_t;
        curVals.clear();
        /*for ( i_t = 0; i_t<state_constraint_tasks.size();i_t++ )
        {
          //  printf(" Inside for with i_t=%d, state_constraint_tasks.size() =%d \n",i_t,state_constraint_tasks.size()); 
            state_constraint_tasks[i_t]->computeValue();
          //  printf(" after tasks[i_t]->computeValue(); \n");
            state_constraint_tasks[i_t]->value();
          //  printf(" after tasks[i_t]->value(); \n");
            curVals.push_back(norm_2 ( state_constraint_tasks[i_t]->value() ));
            //curVals[i_t] = norm_2 ( state_constraint_tasks[i]->value() );
         //   printf(" after curVals[i_t] \n");
        }*/

        for (  i_t = 0; i_t<state_constraint_final_tasks.size();i_t++ )
        {
            state_constraint_final_tasks[i_t]->computeValue(); // It will compute the difference between the current position of the robot and the goal position
            curVals.push_back ( norm_2 ( state_constraint_final_tasks[i_t]->value() ) );
            
        }
        //std::cout<< " Before calculating norm_2 ( attStandingRobot->robot()->currentVelocity())" << std::endl;
        
        double curr_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
      //  printf(" curr_norm =%lf \n",curr_norm);
        //  Uncomment it for real test, there is some problem with norm_2
        if (  curr_norm < 0.00001 )
        {
            printf(" AKP WARNING : curr_norm =%lf So stop.\n",curr_norm);
            breakLoop = true;
            tar_unreachable=true;
        }
        /*else
        {
         breakLoop = false;
        }
        */
         
      //  printf(" prev_norm = %lf, curr_norm = %lf\n",prev_norm, curr_norm);
        /*
        if (  curr_norm - prev_norm > 0.5 )
        {
            printf(" AKP WARNING : curr_norm - prev_norm = %lf , So stop.\n",curr_norm - prev_norm);
            breakLoop = true;
            tar_unreachable=true;
        }
       */
      //   printf(" After if else norm_2 \n");
         
         //breakLoop = true;
        int i=0;
        //if(state==2) //Robot is standing in half sitting position
        //i=2; //Skip the COM and foot on ground constraint

        ////for (; i<state_constraint_tasks.size();i++)
        for (; i<state_constraint_final_tasks.size();i++)
        {
           // printf(" curVals[%d]=%lf, prevVals[%d] = %lf , \n",i,curVals[i],i , prevVals[i]);
            /*if(curVals[i]<0.0000005)
            {   printf(" AKP WARNING : Target unreachable as curVals[%d]=%lf\n",i,curVals[i]);
                breakLoop = true;
                tar_unreachable=true;
            } */
            double abs_diff=fabs(curVals[i] - prevVals[i]);
            /*if (abs_diff < 0.000005)//No significant progress
            {
                //printf(" state_constraint_final_tasks.size() = %d \n", state_constraint_final_tasks.size());
                printf(" AKP WARNING : fabs(curVals[%d] - prevVals[%d]) = %lf , No singificant progress So stop.\n",i,i,abs_diff);
                //printf(" AKP WARNING :  \n");
                breakLoop = true;
                tar_unreachable=true;
                //break;
            }*/
            if (curVals[i] - prevVals[i] > 0.005)//if (curVals[i] - prevVals[i] > 0.00005)//diverges//if (curVals[i] - prevVals[i] > 0.005)//diverges////if (curVals[i] - prevVals[i] > 0.00005)//diverges
            {
                //printf(" state_constraint_final_tasks.size() = %d \n", state_constraint_final_tasks.size());
                printf(" **** For while_ctr=%d\n",while_ctr);
                printf(" ***** AKP WARNING : curVals[%d] - prevVals[%d] = %lf , Diverging So stop.\n",i,i,curVals[i] - prevVals[i]);
                //printf(" AKP WARNING :  \n");
                breakLoop = true;
                tar_unreachable=true;
                //break;
            }
            else 
            {
            if (curVals[i] < prevVals[i] - 1e-4) //significant progress
             {
             //   printf(" Progress \n")  ;
                //breakLoop = false;
                //break;
             }
            else
             {
              /*if(fabs(prevVals[i] - curVals[i]) < 1e-5&&while_ctr>5)
              { 
               printf(" No progress so stop\n");
              //Might have stucked 
               breakLoop = true;
               tar_unreachable=true;
              }*/
             
             }
           }
        ////prevVals.push_back ( curVals[i] );
        }
        prevVals.clear();
        for (i=0; i<state_constraint_final_tasks.size();i++)
        {
         prevVals.push_back ( curVals[i] );
        }
        //curVals.push_back ( prevVals[i] );
        //breakLoop = true;
 
         // AKP : Tmp attempt to stop the loop, just for test purpose
         if(while_ctr==950)
         { 
         breakLoop=true;
         tar_unreachable=true;
         }
         if( time >= endtime )
         breakLoop=true;

while_ctr++;


    }//END  while ( time < endtime )

  MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
  

//AKP WARNING: We should delete the vectorizable_tasks but it is giving segmentation fault. Need to Debug it
/*
   for ( unsigned int i=0;i<vectorizable_tasks.size();i++ )
     {
        printf(" deleting vectorizable_tasks[%d]\n", i);
        delete vectorizable_tasks[i];
     }
*/
  // printf(" Outside while \n"); 
 /*  
   if(tar_unreachable==true)
   {
   if(hand_by_reach==2)// reach by right hand  
   printf("******** target unreachable by RIGHT hand\n");
   else
   printf("******** target unreachable by LEFT hand\n");
   }
   else
   {
   if(hand_by_reach==2)// reach by right hand  
   printf("<<<<<<< target reachable by RIGHT hand\n");
   else
   printf("<<<<<<< target reachable by LEFT hand\n");
   }
*/
   /*show_gik_sol();
   printf(" After show_gik_sol()\n");

  i=0;
  for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  MY_FREE(curr_gik_sol.configs[i], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure
  } 
  */

/////********* Uncomment below to display the current solution ************/////
/*
 int i=0;
 for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[i]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  }
  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/

/*  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
*/
   if(tar_unreachable==true)
   return 0;
   else
   return 1;

}

double* get_HRP2_hand_x_axis_orientation_in_global_frame(int for_hand)//1 for left, 2 for right hand
{
     CjrlJoint* joint;
     CjrlHand* hand;
     matrix4d curT;//=  attRobot->rightWrist()->currentTransformation();
     if(for_hand==1)// reach by left hand   
    {
    joint = attStandingRobot->robot()->leftWrist();
    //curT = attRobot->leftWrist()->currentTransformation();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(for_hand==2)// reach by right hand   
    {
     joint = attStandingRobot->robot()->rightWrist();
    ////curT = attRobot->rightWrist()->currentTransformation();
    hand =  attStandingRobot->robot()->rightHand();
    }
  
    vector3d laxis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
   matrix4d cur_wrist_transformation= joint->currentTransformation();
   

   matrix3d R_comp_cur_wrist_transformation;
   int i=0;
   for(i=0;i<3;i++)
   {
    int j=0;
    for(j=0;j<3;j++)
    {
     M3_IJ(R_comp_cur_wrist_transformation,i,j)=M4_IJ(cur_wrist_transformation,i,j);
    }
   }
   //vector4d wrst_vect_in_global_frame=cur_wrist_transformation*localaxis;
   vector3d wrist_vect_in_global_frame=R_comp_cur_wrist_transformation*laxis;
   //wrist_vect_in_global_frame[0]=wrst_vect_in_global_frame[0];
   //wrist_vect_in_global_frame[1]=wrst_vect_in_global_frame[1];
   //wrist_vect_in_global_frame[2]=wrst_vect_in_global_frame[2];

   printf(" ******* Current wrist_vect_in_global_frame : %lf, %lf, %lf ",wrist_vect_in_global_frame[0],wrist_vect_in_global_frame[1],wrist_vect_in_global_frame[2]); 


     double hand_x_axis_orientation[3];
     hand_x_axis_orientation[0]= wrist_vect_in_global_frame[0];
     hand_x_axis_orientation[1]= wrist_vect_in_global_frame[1];
     hand_x_axis_orientation[2]= wrist_vect_in_global_frame[2];
     
     return hand_x_axis_orientation;
}

double* get_HRP2_hand_center_in_global_frame(int for_hand)//1 for left, 2 for right hand
{
     CjrlHand* hand;
     matrix4d curT;//=  attRobot->rightWrist()->currentTransformation();
     if(for_hand==1)// reach by left hand   
    {
    curT = attRobot->leftWrist()->currentTransformation();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(for_hand==2)// reach by right hand   
    {
    curT = attRobot->rightWrist()->currentTransformation();
    hand =  attStandingRobot->robot()->rightHand();
    }
  
   

    vector3d hand_center_in_wrist_frame = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
     vector4d hand_center_in_global_frame = curT*hand_center_in_wrist_frame; //Getting the current position of hand in the global frame
     //p3d_vector3 hand_center;
     double *hand_center;
     hand_center=MY_ALLOC(double,3); 
     hand_center[0]= hand_center_in_global_frame[0];
     hand_center[1]= hand_center_in_global_frame[1];
     hand_center[2]= hand_center_in_global_frame[2];
     
     return hand_center;
}

//To reach the bottle and look at it
int M3D_GIK_TEST(int hand_by_reach) //1 for left, 2 for right
{
int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
create_HRP2_robot(state);

 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


	curr_gik_sol.no_configs=0;


	ChppGikSolver gikSolver(*attRobot);

//Select the root of the robot at the right foot (reduces computation complexity)
    gikSolver.rootJoint(*attRobot->rightFoot());

    vector3d targetPoint, localPoint, p, look_point, req_hand_orient;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

//Create the constraints defing the tasks.

    std::vector<CjrlGikStateConstraint*> state_constraint_tasks;
    printf(" After declearing   state_constraint_tasks \n");

    std::vector<ChppGikVectorizableConstraint*> vectorizable_tasks;
    printf(" After declearing   vectorizable_tasks \n");

   
   
   //// First priority: Foot on the ground
   //// AKP : The foot which will be used for foot on ground constraint should be opposite to the foot which has been selected as the root of the robot. Because thr root of the robot will always be fixed on the ground and the another foot could move which we want to be fixed on the ground also.
    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    ////Second priority: static Center of Mass
    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;
   


    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d gloabl_inReachTarget, tmp_target;
   
    //AKP NOTE: We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
   
    matrix4d curT;//=  attRobot->rightWrist()->currentTransformation();
     if(hand_by_reach==1)// reach by left hand   
    {
    curT = attRobot->leftWrist()->currentTransformation();
    joint = attStandingRobot->robot()->leftWrist();
    hand =  attStandingRobot->robot()->leftHand();
    }
 
    if(hand_by_reach==2)// reach by right hand   
    {
    curT = attRobot->rightWrist()->currentTransformation();
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();
    }
  
   

    lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

  
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
     vector4d tar_in_global_frame = curT*lpoint; //Getting the current position of hand in the global frame

     vector3d target_in_global_frame;
     target_in_global_frame[0]=tar_in_global_frame[0]+0.2;
     target_in_global_frame[1]=tar_in_global_frame[1]+0.2;
     target_in_global_frame[2]=tar_in_global_frame[2]+0.5;
     
     to_reach_target[0]=target_in_global_frame[0];
     to_reach_target[1]=target_in_global_frame[1];   
     to_reach_target[2]=target_in_global_frame[2];

     configPt visball_pos = MY_ALLOC(double,ACBTSET->visball->nb_dof); /* Allocation of temporary robot configuration */
     p3d_get_robot_config_into(ACBTSET->visball,&visball_pos);  
     target_in_global_frame[0]=visball_pos[6];
     target_in_global_frame[1]=visball_pos[7];
     target_in_global_frame[2]=visball_pos[8];
     MY_FREE(visball_pos, double,ACBTSET->visball->nb_dof);  /* Freeing temporary robot config structure */
    
     int bottle_indx=get_index_of_robot_by_name("bottle");
     if(bottle_indx==NULL)
     bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
    printf(" bottle_indx = %d \n",bottle_indx);
     
 
     p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
     target_in_global_frame[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
     target_in_global_frame[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
     target_in_global_frame[2]=envPt->robot[bottle_indx]->ROBOT_POS[8];

    

     to_reach_target[0]=target_in_global_frame[0];
     to_reach_target[1]=target_in_global_frame[1];   
     to_reach_target[2]=target_in_global_frame[2];
    
     printf("to_reach_target=%lf, %lf, %lf\n",to_reach_target[0],to_reach_target[1],to_reach_target[2]);  

     ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, target_in_global_frame);    

        
    ////********* AKP : For parallel constraint of the wrist ***********/////
    
     vector3d wrist_x_axis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    printf(" okay axis in wrist frame : %lf, %lf, %lf ",wrist_x_axis[0],wrist_x_axis[1],wrist_x_axis[2]);
  
    vector3d wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    printf(" palm axis in wrist frame : %lf, %lf, %lf ",wrist_y_axis[0],wrist_y_axis[1],wrist_y_axis[2]);

    vector3d wrist_z_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    printf(" showing axis in wrist frame : %lf, %lf, %lf ",wrist_z_axis[0],wrist_z_axis[1],wrist_z_axis[2]);


   matrix4d cur_wrist_transformation= attStandingRobot->robot()->rightWrist()->currentTransformation();
   vector4d localaxis;

   localaxis[0]=wrist_x_axis[0];
   localaxis[1]=wrist_x_axis[1];
   localaxis[2]=wrist_x_axis[2];
   localaxis[3]=0;

   matrix3d R_comp_cur_wrist_transformation;
   int i=0;
   for(i=0;i<3;i++)
   {
    int j=0;
    for(j=0;j<3;j++)
    {
     M3_IJ(R_comp_cur_wrist_transformation,i,j)=M4_IJ(cur_wrist_transformation,i,j);
    }
   }
   //vector4d wrst_vect_in_global_frame=cur_wrist_transformation*localaxis;
   vector3d wrist_vect_in_global_frame=R_comp_cur_wrist_transformation*wrist_x_axis;
   //wrist_vect_in_global_frame[0]=wrst_vect_in_global_frame[0];
   //wrist_vect_in_global_frame[1]=wrst_vect_in_global_frame[1];
   //wrist_vect_in_global_frame[2]=wrst_vect_in_global_frame[2];

   printf(" wrist_vect_in_global_frame : %lf, %lf, %lf ",wrist_vect_in_global_frame[0],wrist_vect_in_global_frame[1],wrist_vect_in_global_frame[2]); 
     
    req_hand_orient[0]=0;   
    req_hand_orient[1]=0;
    req_hand_orient[2]=1;
    ChppGikParallelConstraint prlc( * ( attStandingRobot->robot() ),*joint,wrist_x_axis,req_hand_orient );
    int orientation_constraint_push=0;
    

    ////******** AKP : For 3D rotation constraint of the wrist *************////
    matrix3d desired_wrist_rotation; 
    //AKP : 1st coloumn of the 3D matrix should contain the preffered orientation of the x axis of the wrist frame in the world coordinate, similarly 2nd and 3rd coloumn should contain the y and z axes of the wrist frame in the world coordinate   
    M3_IJ ( desired_wrist_rotation,0,0 ) = 0; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ ( desired_wrist_rotation,1,0 ) = 0;
    M3_IJ ( desired_wrist_rotation,2,0 ) = 1;
    M3_IJ ( desired_wrist_rotation,0,1 ) = -1;
    M3_IJ ( desired_wrist_rotation,1,1 ) = 1;
    M3_IJ ( desired_wrist_rotation,2,1 ) = 0;
    M3_IJ ( desired_wrist_rotation,0,2 ) = 0;
    M3_IJ ( desired_wrist_rotation,1,2 ) = -1;
    M3_IJ ( desired_wrist_rotation,2,2 ) = 0;
  
    ChppGikRotationConstraint rotc(* ( attStandingRobot->robot() ),*joint, desired_wrist_rotation);
    int rotation_constraint_push=0;
     	
    //***********If want to give current orientation of the wrist as the constraint uncommet below part *******//
    /*
      vector3d wrist_x_axis = hand->okayAxisInWristFrame();
        //matrix4d cur_wrist_transformation= attStandingRobot->robot()->rightWrist()->currentTransformation();
        vector3d wrist_x_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_x_axis;
   
        printf(" wrist_x_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_x_axis_in_global_frame[0],wrist_x_axis_in_global_frame[1],wrist_x_axis_in_global_frame[2]); 

        wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        vector3d wrist_y_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_y_axis;    
        
         printf(" wrist_y_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_y_axis_in_global_frame[0],wrist_y_axis_in_global_frame[1],wrist_y_axis_in_global_frame[2]); 

        vector3d wrist_show_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        //vector3d wrist_z_axis;
        wrist_z_axis[0]=wrist_show_axis[0];
        wrist_z_axis[1]=wrist_show_axis[1];
        wrist_z_axis[2]=-wrist_show_axis[2]; //AKP : Because in the robot kinematics chain z axis of wrist frame is opposite to the showing axis

        vector3d wrist_z_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_z_axis;    
        
        printf(" wrist_z_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_z_axis_in_global_frame[0],wrist_z_axis_in_global_frame[1],wrist_z_axis_in_global_frame[2]); 
    //AKP : 1st coloumn of the 3D matrix should contain the preffered orientation of the x axis of the wrist frame in the world coordinate, similarly 2nd and 3rd coloumn should contain the y and z axes of the wrist frame in the world coordinate   
    matrix3d cur_rot;// = rotc.targetOrientation();
    // 1st column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ( cur_rot,0,0 ) = wrist_x_axis_in_global_frame[0]; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ( cur_rot,1,0 ) = wrist_x_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,0 ) = wrist_x_axis_in_global_frame[2];
    
    // 2nd column : Preffered y axis orientation of the wrist frame in the world frame 
    M3_IJ( cur_rot,0,1 ) = wrist_y_axis_in_global_frame[0];
    M3_IJ( cur_rot,1,1 ) = wrist_y_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,1 ) = wrist_y_axis_in_global_frame[2];
 
    // 3rd column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ( cur_rot,0,2 ) = wrist_z_axis_in_global_frame[0];
    M3_IJ( cur_rot,1,2 ) = wrist_z_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,2 ) = wrist_z_axis_in_global_frame[2];
   
    //rotc.targetOrientation(cur_rot);
    ChppGikRotationConstraint rotc(* ( attStandingRobot->robot() ),*joint, cur_rot);
     */
    //***********END If want to give current orientation of the wrist as the constraint *******//


    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
 
    look_point[0]=rob_cur_pos[6]+1;
    look_point[1]=rob_cur_pos[7]-1;
    look_point[2]=rob_cur_pos[8]+0.5;

     look_point[0]=to_reach_target[0];
     look_point[1]=to_reach_target[1];
     look_point[2]=to_reach_target[2];

    ChppGikGazeConstraint lc(*attRobot,look_point);

    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    
    ////********** AKP : Here we add the constraints in the task stack in the desired order of priority **********//
   /* state_constraint_tasks.push_back(&nsfc);
    state_constraint_tasks.push_back(&comc);
    state_constraint_tasks.push_back(&pc);

    //tasks.push_back(&rotc); 
    //rotation_constraint_push=1;

    //tasks.push_back(&prlc);
    state_constraint_tasks.push_back(&lc);
    ////tasks.push_back ( new ChppGikPositionConstraint ( * ( attStandingRobot->robot() ),*joint,lpoint,waist_inReachTarget ) );
    */
    


    ////vectorN activated =  attStandingRobot->maskFactory()->upperBodyMask();
    ////vectorN activated =  attStandingRobot->maskFactory()->rightArmMask();
    vectorN activated =  attStandingRobot->maskFactory()->leftArmMask();
    ////vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for ( unsigned int i=0;i<combined.size();i++ )
        combined ( i ) *= activated ( i );
    for ( unsigned int i=0;i<6;i++ )
        combined ( i ) = 0.0;

   
    gikSolver.weights ( combined );

//     Attempt solve with a single one step: Prepare the constraints (jacobian and value computation)

        

//Solve
//Create a ChppRobotMotion (where the successive configurations will be stored)
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    vector3d dummy;
    std::vector<double> prevVals, curVals;

    ////std::vector<double> dampers ( state_constraint_tasks.size(),0.2 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.


    //dampers[0] = dampers[1] = 0.0;
    
//Creating interpolated points, for this we need to populate vectorizable_tasks
    //vectorizable_tasks.push_back(&nsfc);
    vectorizable_tasks.push_back(&pc);
    
    //vectorizable_tasks.push_back(&rotc); 
    //rotation_constraint_push=1;
    vectorizable_tasks.push_back(&prlc);
    //tasks.push_back(&prlc);
    vectorizable_tasks.push_back(&lc);

 std::vector<ChppGikInterpolatedElement*> Elements;
    double startTime = 0.0;
    double attTasksDuration=3; //in sec
printf("Before Interpolating vectorizable_tasks\n");
    for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
    {
     printf("Interpolating the vectorizable_tasks %d \n",i); 
     Elements.push_back ( new ChppGikInterpolatedElement ( attStandingRobot->robot(),vectorizable_tasks[i],i,startTime,attTasksDuration,attSamplingPeriod ) );
    }
    
   printf(" After Interpolating vectorizable_tasks\n");
    double time = startTime+attSamplingPeriod;
    double endtime = startTime+attTasksDuration+attSamplingPeriod/2.0;
    bool gikStepSuccess = true;
    //vector3d dummy;
 int while_ctr=0;
 int breakLoop=0;
 RS_robotq = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 int big_norm_diff_ctr=0;
 int diverge_ctr=0;
    while (!breakLoop)
    {
      printf(" Inside while with time =%lf, while_ctr=%d\n",time, while_ctr);
      state_constraint_tasks.clear();  
      if(state==2) //Robot is standing
      { 
      state_constraint_tasks.push_back(&nsfc); //Putting foot on ground constraint
      state_constraint_tasks.push_back(&comc); // Putting COM constraint
      }
        //Now putting the element of vectorizable constraints 
        printf(" Before pushing vectorizable_tasks\n");
        for ( unsigned int i = 0;i<vectorizable_tasks.size();i++ )
         {
         printf(" Pushing vectorizable_tasks %d\n",i); 
         state_constraint_tasks.push_back ( Elements[i]->stateConstraintAtTime ( time ) );
         }
         std::vector<double> dampers ( state_constraint_tasks.size(),0.8 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.
         dampers[0] = dampers[1] = 0.0; 
         ////dampers[3] = 1.5; //For gaze constraint use big values like this 
         printf(" After pushing vectorizable_tasks\n");   
   
         
        // vector3d p=Elements[2]->stateConstraintAtTime ( time )->worldTarget();
        time += attSamplingPeriod;
        //pc=Elements[i]->stateConstraintAtTime ( time );
        pc.computeValue(); // Computes the difference between the pc.worldTarget() and the current position of the hand
        vectorN p = pc.value();
        ////robot_hand_reach_goal[while_ctr][0]=p[0];
        ////robot_hand_reach_goal[while_ctr][1]=p[1];
        ////robot_hand_reach_goal[while_ctr][2]=p[2];
        
       
         
         printf(" Before gikSolver.prepare()\n");
         gikSolver.prepare(state_constraint_tasks);
         printf(" After gikSolver.prepare() and before gikSolver.solve()\n");
  
             curVals.clear();
          prevVals.clear();
       for ( unsigned int i = 0; i<state_constraint_tasks.size();i++ )
        {
            prevVals.push_back ( norm_2 ( state_constraint_tasks[i]->value() ) );
            curVals.push_back ( prevVals[i] );
        }
        
         double prev_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
        //attGikSolver->solve( inConstraints );
        gikSolver.solve ( state_constraint_tasks, dampers );
        printf(" After gikSolver.solve() and before attStandingRobot->updateRobot()\n");
        attStandingRobot->updateRobot ( gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod );
        printf("After attStandingRobot->updateRobot() and before gMotion.appendSample ()\n");
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
        printf(" After gMotion.appendSample ()\n");
        
       const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;
       printf(" After getting gMotion.nextSample\n");  
      
       if ( !s )
        return false;

    //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point
    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }
        
        
        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      
//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        printf(" Before p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);\n");
        p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        //RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
     printf(" Before calling hrp2_to_M3D_ConfigPt()\n");
     hrp2_to_M3D_ConfigPt(&gik_sol,RS_robotq);
     printf(" After calling hrp2_to_M3D_ConfigPt()\n");
//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     ////curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
     
     curr_gik_sol.configs[curr_gik_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */

     p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     hrp2_to_M3D_ConfigPt(&gik_sol,curr_gik_sol.configs[curr_gik_sol.no_configs]);
     printf(" Returned from hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     curr_gik_sol.configs[curr_gik_sol.no_configs][6]=motionConfig[0];
     curr_gik_sol.configs[curr_gik_sol.no_configs][7]=motionConfig[1];
     
     if(HRP2_CURRENT_STATE==1)//Means sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_sitting_Z_shift;
     if(HRP2_CURRENT_STATE==2)//Means half sitting
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-M3D_to_HRP2_GIK_half_sitting_Z_shift;
     
     //curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-0.6487;
      curr_gik_sol.configs[curr_gik_sol.no_configs][9]=motionConfig[3];
     curr_gik_sol.configs[curr_gik_sol.no_configs][10]=motionConfig[4];
     curr_gik_sol.configs[curr_gik_sol.no_configs][11]=motionConfig[5];
////p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);

     curr_gik_sol.no_configs++;
     printf("curr_gik_sol.no_configs = %d \n",curr_gik_sol.no_configs);
  


//curr_gik_sol.

//g3d_draw_env();

      
        
        int i_t;
        curVals.clear();
        for ( i_t = 0; i_t<state_constraint_tasks.size();i_t++ )
        {
            printf(" Inside for with i_t=%d, state_constraint_tasks.size() =%d \n",i_t,state_constraint_tasks.size()); 
            state_constraint_tasks[i_t]->computeValue();
            printf(" after tasks[i_t]->computeValue(); \n");
            state_constraint_tasks[i_t]->value();
            printf(" after tasks[i_t]->value(); \n");
            curVals.push_back(norm_2 ( state_constraint_tasks[i_t]->value() ));
            //curVals[i_t] = norm_2 ( state_constraint_tasks[i]->value() );
            printf(" after curVals[i_t] \n");
        }
        //std::cout<< " Before calculating norm_2 ( attStandingRobot->robot()->currentVelocity())" << std::endl;
        
        double curr_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
        printf(" curr_norm =%lf \n",curr_norm);
        //  Uncomment it for real test, there is some problem with norm_2
        if (  curr_norm < 0.00001 )
        {
            printf(" AKP WARNING : curr_norm =%lf So stop.\n",curr_norm);
            breakLoop = true;
        }
        /*else
        {
         breakLoop = false;
        }
        */
         
        printf(" prev_norm = %lf, curr_norm = %lf\n",prev_norm, curr_norm);

         if (  curr_norm - prev_norm > 0.05 )
        {
            printf(" AKP WARNING : curr_norm - prev_norm = %lf , So stop.\n",curr_norm - prev_norm);
            big_norm_diff_ctr++;
           //// breakLoop = true;
        }

         printf(" After if else norm_2 \n");
         
         //breakLoop = true;
        for (unsigned int i = 0; i<state_constraint_tasks.size();i++)
        {
            if (curVals[i] - prevVals[i] > 0.001)//diverges
            {
                printf(" AKP WARNING : Diverging so stop \n");
                diverge_ctr++;
                ////breakLoop = true;
                
                //break;
            }
            else if (curVals[i] < prevVals[i] - 1e-4) //significant progress
            {
                printf(" Progress \n")  ;
                //breakLoop = false;
                //break;
            }
        }

        //breakLoop = true;
 
         // AKP : Tmp attempt to stop the loop, just for test purpose
         if(while_ctr==950)
         breakLoop=true;
         
         if( time >= endtime )
         breakLoop=true;

while_ctr++;


    }//END  while ( time < endtime )

  MY_FREE(RS_robotq, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
  

//AKP WARNING: We should delete the vectorizable_tasks but it is giving segmentation fault. Need to Debug it
/*
   for ( unsigned int i=0;i<vectorizable_tasks.size();i++ )
     {
        printf(" deleting vectorizable_tasks[%d]\n", i);
        delete vectorizable_tasks[i];
     }
*/
   printf(" Outside while \n"); 
   printf(" big_norm_diff_ctr = %d, diverge_ctr = %d\n",big_norm_diff_ctr,diverge_ctr);
   /*show_gik_sol();
   printf(" After show_gik_sol()\n");

  i=0;
  for(i=0;i<curr_gik_sol.no_configs;i++)
  {
  MY_FREE(curr_gik_sol.configs[i], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure
  } 
  */
 

}

int M3D_GIK_TEST_without_interpolation()
{


//\page example1_page Example I
//section intro_sec Introduction
//In this basic example, a motion for the center of the right hand is planned while projection of the center of mass on the ground and the feet's positions are maintained static.

//section code_sec Code

//Set a sampling period: 5ms second is the value used on HRP2
 
//code
    double attSamplingPeriod = 5e-3; // It should be 5e-3

    CjrlRobotDynamicsObjectConstructor<
    dynamicsJRLJapan::DynamicMultiBody,
    //dynamicsJRLJapan::HumanoidDynamicMultiBody,
    Chrp2OptHumanoidDynamicRobot,
    dynamicsJRLJapan::JointFreeflyer,
    dynamicsJRLJapan::JointRotation,
    dynamicsJRLJapan::JointTranslation,
    dynamicsJRLJapan::Body> jrlRobotFactory;


   CjrlHumanoidDynamicRobot* attRobot;
   attRobot = jrlRobotFactory.createhumanoidDynamicRobot();

    dynamicsJRLJapan::HumanoidDynamicMultiBody *aHDMB;
    aHDMB = ( dynamicsJRLJapan::HumanoidDynamicMultiBody* ) attRobot;

    //std::string path = "./";
    std::string path = "/home/akpandey/HRP2models/";
    //std::string name = "HRP2.wrl";
    std::string name = "HRP2JRLmain.wrl";
    //aHDMB->parserVRML ( path,name,"./HRP2LinkJointRank.xml" );
    aHDMB->parserVRML ( path,name,"/home/akpandey/HRP2models/HRP2LinkJointRank.xml" );
    //std::string aName="./HRP2Specificities.xml";
    
    std::string aName="/home/akpandey/HRP2models/HRP2Specificities.xml";
    aHDMB->SetHumanoidSpecificitiesFile ( aName );
    aHDMB->SetTimeStep ( attSamplingPeriod );
    aHDMB->setComputeVelocity ( true );
    aHDMB->setComputeMomentum ( true );
    aHDMB->setComputeCoM ( true );
    aHDMB->setComputeAcceleration ( true );
    aHDMB->setComputeZMP ( true );

    aHDMB->setComputeSkewCoM ( false );
    aHDMB->setComputeAccelerationCoM ( false );
    aHDMB->setComputeBackwardDynamics ( false );



    unsigned int nDof = attRobot->numberDof();
    vectorN halfsittingConf ( nDof );

    //Half sitting
    double dInitPos[40] =
    {
        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // right leg

        0.0, 0.0, -26.0, 50.0, -24.0, 0.0, // left leg

        0.0, 0.0, // chest

        0.0, 0.0, // head

        15.0, -10.0, 0.0, -30.0, 0.0, 0.0, // right arm

        10.0, // right hand clench

        15.0,  10.0, 0.0, -30.0, 0.0, 0.0, // left arm

        10.0, // left hand clench

        -10.0, 10.0, -10.0, 10.0, -10.0, // right hand parallel mechanism
        -10.0, 10.0, -10.0, 10.0, -10.0  // left hand parallel mechanism
    };


    configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
    p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);
    //rob_cur_pos[6]
    
    //AKP : Using hrp2 position in move3D to localize the robot, because HRP2 can not localize itself in the environment
    //waist x y z// Robot waist position and orientation in global system and every task and target are expressed in global frame
    halfsittingConf ( 0 ) = rob_cur_pos[6];
    halfsittingConf ( 1 ) = rob_cur_pos[7];
    halfsittingConf ( 2 ) = rob_cur_pos[8]+0.6487;   
 
    //MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    ////halfsittingConf ( 0 ) = 0.0;
    ////halfsittingConf ( 1 ) = 0.0;
    ////halfsittingConf ( 2 ) = 0.6487;
     
    
    //waist roll pitch yaw
    halfsittingConf ( 3 ) = 0.0;
    halfsittingConf ( 4 ) = 0.0;
    halfsittingConf ( 5 ) = 0.0;

    // AKP : Using halfsitting configuration to set roll, pitch, yaw of hrp2 model in move3d
    rob_cur_pos[9]=halfsittingConf(3);
    rob_cur_pos[10]=halfsittingConf(4);
    rob_cur_pos[11]=halfsittingConf(5);

    p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos);

    
    
    //joints
    for ( unsigned int i=6;i<nDof;i++ )
        halfsittingConf ( i ) = dInitPos[i-6]*M_PI/180;

    zero_vector<double> zeros ( attRobot->numberDof() );
    attRobot->currentConfiguration ( halfsittingConf );
    attRobot->currentVelocity ( zeros );
    attRobot->currentAcceleration ( zeros );
    attRobot->computeForwardKinematics();

    //set gaze origin and direction
    vector3d gazeDir,gazeOrigin;
    gazeDir[0] = cos(10*M_PI/180);
    gazeDir[1] = 0;
    gazeDir[2] = -sin(10*M_PI/180);

    gazeOrigin[0] = 0;
    gazeOrigin[1] = 0;
    gazeOrigin[2] = 0.118;

    aHDMB->gaze ( ( const vector3d& ) gazeDir, ( const vector3d& ) gazeOrigin );

    ChppGikStandingRobot* attStandingRobot = new ChppGikStandingRobot ( attRobot );
    attStandingRobot->staticState ( halfsittingConf );


        ChppGikSolver gikSolver(*attRobot);

//Select the root of the robot at the right foot (reduces computation complexity)

    gikSolver.rootJoint(*attRobot->rightFoot());


//Create a ChppRobotMotion (where the successive configurations will be stored)

    ////ChppRobotMotion attSolutionMotion(attRobot, 0.0 , attSamplingPeriod);

//Miscelleaenous variables

    vector3d targetPoint, localPoint, p, look_point, req_hand_orient;
    vector3d absZMPPla, absZMPObs, relZMPObs, relZMPPla;

//Create the constraints defing the tasks.

    std::vector<CjrlGikStateConstraint*> tasks;

 ////std::vector<ChppGikVectorizableConstraint*> tasks;

    ////vector3d com = attRobot->positionCenterOfMass();
    ////ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    ////absZMPPla = com;

   //// First priority: Foot on the ground

    localPoint[0] = 0.0;
    localPoint[1] = 0.0;
    localPoint[2] = 0.0;
    CjrlJoint& nsfJoint = *(attRobot->leftFoot());
    matrix4d nsfTransform = nsfJoint.currentTransformation();
    ChppGikTransformationConstraint nsfc(*attRobot, nsfJoint, localPoint, nsfTransform);

    ////Second priority: static Center of Mass

    vector3d com = attRobot->positionCenterOfMass();
    ChppGikComConstraint comc(*attRobot, com[0], com[1]);
    absZMPPla = com;



    CjrlJoint* joint;
    vector3d lpoint;
    CjrlHand* hand;
    vector3d waist_inReachTarget, gloabl_inReachTarget, tmp_target;
   
    waist_inReachTarget[0]=0.5;
    waist_inReachTarget[1]=-0.2;
    waist_inReachTarget[2]=1.0;
    

    //AKP : We have to give targets in increments. The sample should be such that it is reachable in the set attSamplingPeriod from the current position of the hand
    //matrix4d curT= attRobot->rightWrist()->currentTransformation();
    tmp_target[0]= 0.01;
    tmp_target[1]= -0.25;
    tmp_target[2]= 0.5;
 
    matrix4d curT=  attRobot->rightWrist()->currentTransformation();
   
    //ChppGikPositionConstraint pc(*attRobot,rwJoint,localPoint, curT*localPoint);
    ////waist_to_world_Coordinates(waist_inReachTarget,gloabl_inReachTarget);
   // if ( taskIsForRightHand )
  //  {
    joint = attStandingRobot->robot()->rightWrist();
    hand =  attStandingRobot->robot()->rightHand();

    lpoint = hand->centerInWristFrame(); //AKP: This vector will be always constant because it is in wrist frame, it is basically part of the robot structure

   
    ////ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, waist_inReachTarget);
   //// ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, tmp_target); //AKP : Instead of giving the final target position, we have to give the next close sample position towards the final target position. Then it is upto us to verify that it is converging towards the final target and it has reached there then shop the while loop. So, in the while loop below we are modifying pc for next sample

//**********AKP : For transfering a point in the joint frame to the global frame, multiply it by transformation matrix obtained by joint->currentTransformation, whereas for transfering a vector in joint frame to the global frame multiply it by the R component of transformation matrix, eg. for converting the x, y , z axes of wrist joint, we need to multiply it by extracting the 3x3 R matrix. *********//
 
     vector4d tar_in_global_frame = curT*lpoint; //For starting it is the current position of hand itself

     vector3d target_in_global_frame;
     target_in_global_frame[0]=tar_in_global_frame[0];
     target_in_global_frame[1]=tar_in_global_frame[1];
     target_in_global_frame[2]=tar_in_global_frame[2];

     ChppGikPositionConstraint pc(*( attStandingRobot->robot() ),*joint,lpoint, target_in_global_frame);    

    req_hand_orient[0]=1.0;
    req_hand_orient[1]=0.0;
    req_hand_orient[2]=0.0;
    
    ////********* AKP : For parallel constraint of the wrist ***********/////
    
    vector3d laxis = hand->okayAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    printf(" okay axis in wrist frame : %lf, %lf, %lf ",laxis[0],laxis[1],laxis[2]);
  
    vector3d wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    printf(" palm axis in wrist frame : %lf, %lf, %lf ",wrist_y_axis[0],wrist_y_axis[1],wrist_y_axis[2]);

    vector3d wrist_z_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
    
    printf(" showing axis in wrist frame : %lf, %lf, %lf ",wrist_z_axis[0],wrist_z_axis[1],wrist_z_axis[2]);


   matrix4d cur_wrist_transformation= attStandingRobot->robot()->rightWrist()->currentTransformation();
   vector4d localaxis;

   localaxis[0]=laxis[0];
   localaxis[1]=laxis[1];
   localaxis[2]=laxis[2];
   localaxis[3]=0;

   matrix3d R_comp_cur_wrist_transformation;
   int i=0;
   for(i=0;i<3;i++)
   {
    int j=0;
    for(j=0;j<3;j++)
    {
     M3_IJ(R_comp_cur_wrist_transformation,i,j)=M4_IJ(cur_wrist_transformation,i,j);
    }
   }
   //vector4d wrst_vect_in_global_frame=cur_wrist_transformation*localaxis;
   vector3d wrist_vect_in_global_frame=R_comp_cur_wrist_transformation*laxis;
   //wrist_vect_in_global_frame[0]=wrst_vect_in_global_frame[0];
   //wrist_vect_in_global_frame[1]=wrst_vect_in_global_frame[1];
   //wrist_vect_in_global_frame[2]=wrst_vect_in_global_frame[2];

   printf(" wrist_vect_in_global_frame : %lf, %lf, %lf ",wrist_vect_in_global_frame[0],wrist_vect_in_global_frame[1],wrist_vect_in_global_frame[2]); 
   
    ChppGikParallelConstraint prlc( * ( attStandingRobot->robot() ),*joint,laxis,req_hand_orient );
    int orientation_constraint_push=0;
    

    ////******** AKP : For 3D rotation constraint of the wrist *************////
    matrix3d desired_wrist_rotation; 
    /*M3_IJ ( desired_wrist_rotation,0,0 ) = 0; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ ( desired_wrist_rotation,1,0 ) = 0;
    M3_IJ ( desired_wrist_rotation,2,0 ) = 1;
    M3_IJ ( desired_wrist_rotation,0,1 ) = -1;
    M3_IJ ( desired_wrist_rotation,1,1 ) = 0;
    M3_IJ ( desired_wrist_rotation,2,1 ) = 0;
    M3_IJ ( desired_wrist_rotation,0,2 ) = 0;
    M3_IJ ( desired_wrist_rotation,1,2 ) = -1;
    M3_IJ ( desired_wrist_rotation,2,2 ) = 0;
  */
    //ChppGikRotationConstraint rotc(* ( attStandingRobot->robot() ),*joint, desired_wrist_rotation);
    int rotation_constraint_push=0;
     	

    
   

      vector3d wrist_x_axis = hand->okayAxisInWristFrame();
        //matrix4d cur_wrist_transformation= attStandingRobot->robot()->rightWrist()->currentTransformation();
        vector3d wrist_x_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_x_axis;
   
        printf(" wrist_x_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_x_axis_in_global_frame[0],wrist_x_axis_in_global_frame[1],wrist_x_axis_in_global_frame[2]); 

        wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        vector3d wrist_y_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_y_axis;    
        
         printf(" wrist_y_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_y_axis_in_global_frame[0],wrist_y_axis_in_global_frame[1],wrist_y_axis_in_global_frame[2]); 

        vector3d wrist_show_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        //vector3d wrist_z_axis;
        wrist_z_axis[0]=wrist_show_axis[0];
        wrist_z_axis[1]=wrist_show_axis[1];
        wrist_z_axis[2]=-wrist_show_axis[2]; //AKP : Because in the robot kinematics chain z axis of wrist frame is opposite to the showing axis

        vector3d wrist_z_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_z_axis;    
        
        printf(" wrist_z_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_z_axis_in_global_frame[0],wrist_z_axis_in_global_frame[1],wrist_z_axis_in_global_frame[2]); 
    //AKP : 1st coloumn of the 3D matrix should contain the preffered orientation of the x axis of the wrist frame in the world coordinate, similarly 2nd and 3rd coloumn should contain the y and z axes of the wrist frame in the world coordinate   
    matrix3d cur_rot;// = rotc.targetOrientation();
    // 1st column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ( cur_rot,0,0 ) = wrist_x_axis_in_global_frame[0]; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ( cur_rot,1,0 ) = wrist_x_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,0 ) = wrist_x_axis_in_global_frame[2];
    
    // 2nd column : Preffered y axis orientation of the wrist frame in the world frame 
    M3_IJ( cur_rot,0,1 ) = wrist_y_axis_in_global_frame[0];
    M3_IJ( cur_rot,1,1 ) = wrist_y_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,1 ) = wrist_y_axis_in_global_frame[2];
 
    // 3rd column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ( cur_rot,0,2 ) = wrist_z_axis_in_global_frame[0];
    M3_IJ( cur_rot,1,2 ) = wrist_z_axis_in_global_frame[1];
    M3_IJ( cur_rot,2,2 ) = wrist_z_axis_in_global_frame[2];
   
    //rotc.targetOrientation(cur_rot);
    ChppGikRotationConstraint rotc(* ( attStandingRobot->robot() ),*joint, cur_rot);


    look_point[0]=rob_cur_pos[6]+1;
    look_point[1]=rob_cur_pos[7];
    look_point[2]=rob_cur_pos[8]+0.5;

    ChppGikGazeConstraint lc(*attRobot,look_point);

    MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof);  /* Freeing temporary robot config structure */
    
    ////********** AKP : Here we add the constraints in the task stack in the desired order of priority **********//
    tasks.push_back(&nsfc);
    tasks.push_back(&comc);
    tasks.push_back(&pc);

    //tasks.push_back(&rotc); 
    //rotation_constraint_push=1;

    //tasks.push_back(&prlc);
    tasks.push_back(&lc);
    ////tasks.push_back ( new ChppGikPositionConstraint ( * ( attStandingRobot->robot() ),*joint,lpoint,waist_inReachTarget ) );



   ////vectorN activated =  attStandingRobot->maskFactory()->upperBodyMask();
    ////vectorN activated =  attStandingRobot->maskFactory()->rightArmMask();
    vectorN activated =  attStandingRobot->maskFactory()->wholeBodyMask();
    vectorN weights = attStandingRobot->maskFactory()->weightsDoubleSupport();
    vectorN combined = weights;
    for ( unsigned int i=0;i<combined.size();i++ )
        combined ( i ) *= activated ( i );
    for ( unsigned int i=0;i<6;i++ )
        combined ( i ) = 0.0;

   
    gikSolver.weights ( combined );

//     Attempt solve with a single one step: Prepare the constraints (jacobian and value computation)

        

//Solve
//Create a ChppRobotMotion (where the successive configurations will be stored)
    ChppRobotMotion gMotion ( attStandingRobot->robot(),0.0,attSamplingPeriod );
    vector3d dummy;
    std::vector<double> prevVals, curVals;
    std::vector<double> dampers ( tasks.size(),0.2 ); //It works as a filter for unwanted velocity. Smaller the value, bigger velocities will be allowed. 0 is minimum.
    //dampers[0] = dampers[1] = 0.0;
    
/*//Creating interpolated points
 std::vector<ChppGikInterpolatedElement*> Elements;
    double startTime = 0.0;
    double attSittingTasksDuration=2.0; //in sec
    for ( unsigned int i = 0;i<inTasks.size();i++ )
    {
     Elements.push_back ( new ChppGikInterpolatedElement ( attStandingRobot->robot(),tasks[i],i,startTime,attSittingTasksDuration,attSamplingPeriod ) );
    }
*/

 bool breakLoop = false;
    double minProgress = 1e-4;
    int while_ctr=0;
    while ( !breakLoop )
    {   
       //tasks.clear();
       printf(" Inside while ( !breakLoop ) with while_ctr =%d\n", while_ctr);
       while_ctr++;
        int i;
        for (  i = 0; i<tasks.size();i++ )
        {
             printf(" Inside for i =%d, tasks.size() =%d\n",i, tasks.size());
             prevVals.push_back ( norm_2 ( tasks[i]->value() ) );
             curVals.push_back ( prevVals[i] );
        }
        
        if(while_ctr<200)
        { 
        vector3d p = pc.worldTarget();
        p[0] += 0.002;
        //p[1] -= 0.0001;
        p[2] += 0.002;
        pc.worldTarget(p);
        

robot_hand_reach_goal[while_ctr-1][0]=p[0];
robot_hand_reach_goal[while_ctr-1][1]=p[1];
robot_hand_reach_goal[while_ctr-1][2]=p[2];
         }

 //G3D_Window *win = g3d_get_win_by_name((char*)"Move3D");
//g3d_drawDisc(2, -2, 1, 0.5, 4, NULL);
         //g3d_refresh_win(win);

        vector3d pl = lc.worldTarget();
        //pl[0] += 0.001;
        pl[1] -= 0.001;
        pl[2] -= 0.001;
        lc.worldTarget(pl);

        vector3d cur_axis = hand->okayAxisInWristFrame();
        matrix4d cur_wrist_transformation= attStandingRobot->robot()->rightWrist()->currentTransformation();
         matrix3d R_comp_cur_wrist_transformation;
   i=0;
   for(i=0;i<3;i++)
   {
    int j=0;
    for(j=0;j<3;j++)
    {
     M3_IJ(R_comp_cur_wrist_transformation,i,j)=M4_IJ(cur_wrist_transformation,i,j);
    }
   }
        
        vector3d wrist_x_axis_in_global_frame=R_comp_cur_wrist_transformation*cur_axis;
   
        printf(" wrist_x_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_x_axis_in_global_frame[0],wrist_x_axis_in_global_frame[1],wrist_x_axis_in_global_frame[2]); 

        vector3d wrist_y_axis = hand->palmAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        vector3d wrist_y_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_y_axis;    
        
         printf(" wrist_y_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_y_axis_in_global_frame[0],wrist_y_axis_in_global_frame[1],wrist_y_axis_in_global_frame[2]); 

        vector3d wrist_show_axis = hand->showingAxisInWristFrame(); //AKP : This vector will be always constant because it is in wrist frame, it is basically part of the robot structure. This is the local wrist vector 
        vector3d wrist_z_axis;
        wrist_z_axis[0]=wrist_show_axis[0];
        wrist_z_axis[1]=wrist_show_axis[1];
        wrist_z_axis[2]=-wrist_show_axis[2]; //AKP : Because in the robot kinematics chain z axis of wrist frame is opposite to the showing axis

        vector3d wrist_z_axis_in_global_frame=R_comp_cur_wrist_transformation*wrist_z_axis;    
        
        printf(" wrist_z_axis_orientation_in_global_frame : %lf, %lf, %lf ",wrist_z_axis_in_global_frame[0],wrist_z_axis_in_global_frame[1],wrist_z_axis_in_global_frame[2]); 
 
         //printf("*************** Current orientation of hand is %lf, %lf, %lf ************\n",cur_axis[0], cur_axis[1],cur_axis[2]);
        if(while_ctr>100)
        {
        /* /// For parallel constraint
         if(orientation_constraint_push==0)
         {
         //vector3d cur_axis = hand->okayAxisInWristFrame();
         //printf("*************** Current orientation of hand is %lf, %lf, %lf ************\n",cur_axis[0], cur_axis[1],cur_axis[2]);
        vector3d prl = prlc.targetVector();
        prl[0] = wrist_vect_in_global_frame[0];
        prl[1] = wrist_vect_in_global_frame[1];
        prl[2] = wrist_vect_in_global_frame[2];
        prlc.targetVector(prl);
         
         //laxis[0];
         tasks.clear();
         tasks.push_back(&nsfc);
         tasks.push_back(&comc);
         tasks.push_back(&pc);
         tasks.push_back(&prlc);
         tasks.push_back(&lc);
         //tasks.push_back(&prlc);
         orientation_constraint_push=1;
         }
         else
         {
        vector3d prl = prlc.targetVector();
        //if(wrist_vect_in_global_frame
        prl[0] = 0;
        prl[1] = 0;
        prl[2] = 1;
        prlc.targetVector(prl);
         }
        */
         
         ////// For rotation constraint of all the 3 axes of the wrist frame
         if(rotation_constraint_push==0)
         {
         //vector3d cur_axis = hand->okayAxisInWristFrame();
         //printf("*************** Current orientation of hand is %lf, %lf, %lf ************\n",cur_axis[0], cur_axis[1],cur_axis[2]);
        matrix3d cur_rot = rotc.targetOrientation();
   
    //AKP : 1st coloumn of the 3D matrix should contain the preffered orientation of the x axis of the wrist frame in the world coordinate, similarly 2nd and 3rd coloumn should contain the y and z axes of the wrist frame in the world coordinate   

    // 1st column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ ( cur_rot,0,0 ) = wrist_x_axis_in_global_frame[0]; //to access 3D matrix where i is row and j is column in M3_IJ(Matrix, i, j) 
    M3_IJ ( cur_rot,1,0 ) = wrist_x_axis_in_global_frame[1];
    M3_IJ ( cur_rot,2,0 ) = wrist_x_axis_in_global_frame[2];
    
    // 2nd column : Preffered y axis orientation of the wrist frame in the world frame 
    M3_IJ ( cur_rot,0,1 ) = wrist_y_axis_in_global_frame[0];
    M3_IJ ( cur_rot,1,1 ) = wrist_y_axis_in_global_frame[1];
    M3_IJ ( cur_rot,2,1 ) = wrist_y_axis_in_global_frame[2];
 
    // 3rd column : Preffered x axis orientation of the wrist frame in the world frame
    M3_IJ ( cur_rot,0,2 ) = wrist_z_axis_in_global_frame[0];
    M3_IJ ( cur_rot,1,2 ) = wrist_z_axis_in_global_frame[1];
    M3_IJ ( cur_rot,2,2 ) = wrist_z_axis_in_global_frame[2];
   
         rotc.targetOrientation(cur_rot);
         //laxis[0];
         tasks.clear();
         tasks.push_back(&nsfc);
         tasks.push_back(&comc);
         tasks.push_back(&pc);
         tasks.push_back(&rotc);
         tasks.push_back(&lc);
         //tasks.push_back(&prlc);
         rotation_constraint_push=1;

         }
         else
         {
          int changed_val[10]={0,0,0,0,0,0,0,0,0,0};
          matrix3d cur_rot = rotc.targetOrientation();
        printf("Inside else with while_ctr = %d, M3_IJ ( cur_rot,2,0 ) = %lf \n", while_ctr, M3_IJ(cur_rot,2,0));
        //if(wrist_vect_in_global_frame
            // 1st column : Preffered x axis orientation of the wrist frame in the world frame
        
        //Setting desired orientation of x axis of wrist frame as 0 0 1
        double tmp=M3_IJ ( cur_rot,0,0 );
        //tmp-=0.01;
        double ang_step=0.005;        
        double tolerance=0.01;
 
        if(tmp>tolerance)
        {
        M3_IJ ( cur_rot,0,0 )-=ang_step;
        changed_val[0]=1;
        }
        if(tmp<-tolerance)
        {
        M3_IJ ( cur_rot,0,0 )+=ang_step;
         changed_val[0]=1;
        }

        tmp=M3_IJ ( cur_rot,1,0 );
        if(tmp>tolerance)
        { 
        M3_IJ ( cur_rot,1,0 )-=ang_step;
        changed_val[1]=1;
        }
        if(tmp<-tolerance)
        {
        M3_IJ ( cur_rot,1,0 )+=ang_step;
        changed_val[1]=1;
        }
        
        tmp=M3_IJ ( cur_rot,2,0 );
        if(tmp>(1+tolerance))
        {
        M3_IJ ( cur_rot,2,0 )-=ang_step;
        changed_val[2]=1;
        } 
        if(tmp<(1-tolerance))
        {
        M3_IJ ( cur_rot,2,0 )+=ang_step;
        changed_val[2]=1;
        }
        

        //Setting desired orientation of y axis of wrist frame as -1 0 0
        tmp=M3_IJ ( cur_rot,0,1 );
        //tmp-=0.01;
        if(tmp>-(1-tolerance))
        {
        M3_IJ ( cur_rot,0,1 )-=ang_step;
        changed_val[3]=1;
        }
        if(tmp<-(1+tolerance))
        {
        M3_IJ ( cur_rot,0,1 )+=ang_step;
        changed_val[3]=1;
        }

        tmp=M3_IJ ( cur_rot,1,1 );
        if(tmp>tolerance)
        {
        M3_IJ ( cur_rot,1,1 )-=ang_step;
        changed_val[4]=1;
        }
        if(tmp<-tolerance)
        { 
        M3_IJ ( cur_rot,1,1 )+=ang_step;
        changed_val[4]=1;
        }
        
        tmp=M3_IJ ( cur_rot,2,1 );
        if(tmp>tolerance)
        {
        M3_IJ ( cur_rot,2,1 )-=ang_step;
        changed_val[5]=1;
        }
        if(tmp<-tolerance)
        {
        M3_IJ ( cur_rot,2,1 )+=ang_step;
        changed_val[5]=1;
        }
        
 
        //Setting desired orientation of z axis of wrist frame as 0 -1 0
        tmp=M3_IJ ( cur_rot,0,2 );
        if(tmp>tolerance)
        {
        M3_IJ ( cur_rot,0,2 )-=ang_step;
        changed_val[6]=1;
        }
        if(tmp<-tolerance)
        {
        M3_IJ ( cur_rot,0,2 )+=ang_step;
        changed_val[6]=1;
        }

        tmp=M3_IJ ( cur_rot,1,2 );
        //tmp-=0.01;
        if(tmp>-(1-tolerance))
        {
        M3_IJ ( cur_rot,1,2 )-=ang_step;
        changed_val[7]=1;
        }
        if(tmp<-(1+tolerance))
        {
        M3_IJ ( cur_rot,1,2 )+=ang_step;
        changed_val[7]=1;
        }

        tmp=M3_IJ ( cur_rot,2,2 );
        if(tmp>tolerance)
        {
        M3_IJ ( cur_rot,2,2 )-=ang_step;
        changed_val[8]=1;
        }
        if(tmp<-tolerance)
        { 
        M3_IJ ( cur_rot,2,2 )+=ang_step;
        changed_val[8]=1;
        }
        

         printf("\nChanged indices are : ");
         int tmp_i=0;
        int no_change=1;
        for(tmp_i=0;tmp_i<9;tmp_i++)
        {
        printf("%d ",changed_val[tmp_i]);
        if(changed_val[tmp_i]==1)
        no_change=0;
        
        }
        printf("no_change=%d\n",no_change);
        if(no_change==1)
        breakLoop = true;
       //// M3_IJ ( cur_rot,2,0 ) -= 0.01;
       //// printf(" After changing M3_IJ ( cur_rot,2,0 ) = %lf \n", M3_IJ(cur_rot,2,0));
    
        // 2nd column : Preffered y axis orientation of the wrist frame in the world frame 
        ////M3_IJ ( cur_rot,0,1 ) += 0.01;
    
    // 3rd column : Preffered x axis orientation of the wrist frame in the world frame
    //M3_IJ ( cur_rot,1,2 ) -= 0.05;
          rotc.targetOrientation(cur_rot);
         
         }
        }
                

       gikSolver.prepare(tasks);

        //attGikSolver->solve( inConstraints );
        gikSolver.solve ( tasks, dampers );
        
        attStandingRobot->updateRobot ( gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod );
        gMotion.appendSample ( attStandingRobot->robot()->currentConfiguration() ,dummy,dummy,dummy,dummy );
        
        
       const ChppRobotMotionSample *s = gMotion.nextSample();
       ////attLastSampleTime += attSamplingPeriod;

       if ( !s )
        return false;

    //vectorN config = s->configuration;
    vectorN motionConfig = s->configuration;
    vector3d motionZMPwstObs = s->ZMPwstObs; //AKP : ZMP : Zero Momentum Point
    ////ZMPwstObs = s->ZMPwstObs;
    ////timeRef = attLastSampleTime;
    ghrp2_config_t gik_sol;
      
        for (i = 0; i<GHRP2_MAXDOF; i++)
        {
        gik_sol.angles[i] = motionConfig[i+6];
        }
        
        
        gik_sol.zmp[0] = motionZMPwstObs[0];
        gik_sol.zmp[1] = motionZMPwstObs[1];
        gik_sol.zmp[2] = motionZMPwstObs[2];

        gik_sol.waistRpy[0] = motionConfig[3];
        gik_sol.waistRpy[1] = motionConfig[4];
        gik_sol.waistRpy[2] = motionConfig[5];
      
//p3d_get_robot_config_into(ACBTSET->robot,&RS_robotq);
        RS_robotq = p3d_get_robot_config(ACBTSET->robot);
 
     ////p3d_copy_config(ACBTSET->robot, ACBTSET->robot->ROBOT_POS);
    
     //mhpghrp2ConfigPt(&gik_sol,RS_robotq);
 
     hrp2_to_M3D_ConfigPt(&gik_sol,RS_robotq);

//p3d_get_robot_config_into(ACBTSET->robot,&curr_gik_sol.configs[curr_gik_sol.no_configs]);
     curr_gik_sol.configs[curr_gik_sol.no_configs] = p3d_get_robot_config(ACBTSET->robot);
     hrp2_to_M3D_ConfigPt(&gik_sol,curr_gik_sol.configs[curr_gik_sol.no_configs]);
     curr_gik_sol.configs[curr_gik_sol.no_configs][6]=motionConfig[0];
     curr_gik_sol.configs[curr_gik_sol.no_configs][7]=motionConfig[1];
     curr_gik_sol.configs[curr_gik_sol.no_configs][8]=motionConfig[2]-0.6487;;
   
////p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[curr_gik_sol.no_configs]);

     curr_gik_sol.no_configs++;
     printf("curr_gik_sol.no_configs = %d \n",curr_gik_sol.no_configs);
  


//curr_gik_sol.

//g3d_draw_env();




        int i_t;
        for ( i_t = 0; i_t<tasks.size();i_t++ )
        {
            printf(" Inside for with i_t=%d, tasks.size() =%d \n",i_t,tasks.size()); 
            tasks[i_t]->computeValue();
            printf(" after tasks[i_t]->computeValue(); \n");
            tasks[i_t]->value();
            printf(" after tasks[i_t]->value(); \n");
            curVals[i_t] = norm_2 ( tasks[i_t]->value() );
            printf(" after curVals[i_t] \n");
        }
       
        
        double curr_norm=norm_2 ( attStandingRobot->robot()->currentVelocity() );
        printf(" curr_norm =%lf \n",curr_norm);
        //  Uncomment it for real test, there is some problem with norm_2
        if (  curr_norm < 1e-4 )
        {
            //printf(" curr_norm =%lf \n",curr_norm);
            breakLoop = true;
        }
        /*else
        {
         breakLoop = false;
        }
        */
         printf(" After if else norm_2 \n");
         
         // AKP : Tmp attempt to stop the loop, just for test purpose
         if(while_ctr==450)
         breakLoop=true;
         
        /*
        breakLoop = true;
        for (unsigned int i = 0; i<inConstraints.size();i++)
        {
            if (curVals[i] > prevVals[i])//diverges
            {
                break;
            }
            else if (curVals[i] < prevVals[i] - minProgress) //significant progress
            {
                breakLoop = false;
                break;
            }
        }
        if (breakLoop)
        {
            std::cout << "Diverged." << std::endl;
            break;
        }

        breakLoop = true;
        */
    }

        //gikSolver.solve( tasks );

       
//Apply solution to robot

       //// attStandingRobot->updateRobot(gikSolver.solutionRootPose(), gikSolver.solutionJointConfiguration(), attSamplingPeriod);
/*
std::vector<double> dampers ( tasks.size(),1.0 );
    dampers[0] = dampers[1] = 0.0;

    solveStack ( tasks, dampers );
*/


  
   //AKP TO DO : WARNING : We are not deleting the tasks because we can't use delete in C, and I am not sure has the tasks been assigned by new in C++ or malloc in C 
 
/*   for ( unsigned int i=0;i<tasks.size();i++ )
     {
        printf(" deleting task[%d]\n", i);
        delete tasks[i];
     }
  */ 

 printf(" Outside while \n"); 
   show_gik_sol();
   printf(" After show_gik_sol()\n");
//solveStack(
//endcode
//Supposing we have already constructed a CjrlHumanoidDynamicRobot object in halfsitting configuration and for which we have stored a pointer \c attRobot , create a standing robot:
//\code

    
//    ChppGikStandingRobot attStandingRobot(attRobot);
////ChppGikStandingRobot attStandingRobot(attRobot);

}
/*
int get_HRP2_gik_sol()
{
  std::vector<CjrlGikStateConstraint*> gikconstraints;
    std::vector<ChppGikInterpolatedElement*> Elements;
    std::vector<double> dampers;
    for ( unsigned int i = 0;i<inTasks.size();i++ )
    {
        Elements.push_back ( new ChppGikInterpolatedElement ( attStandingRobot->robot(),inTasks[i],i,startTime,attSittingTasksDuration,attSamplingPeriod ) );
        dampers.push_back ( 0.8 );
    }

    double time = startTime+attSamplingPeriod;
    double endtime = startTime+attSittingTasksDuration+attSamplingPeriod/2;
    bool gikStepSuccess = true;
    vector3d dummy;

    while ( time < endtime )
    {
        gikconstraints.clear();
        for ( unsigned int i = 0;i<inTasks.size();i++ )
            gikconstraints.push_back ( Elements[i]->stateConstraintAtTime ( time ) );

        gikStepSuccess = solveConstraintStack ( gikconstraints, dampers );

        if ( gikStepSuccess )
            attGikMotion->appendSample ( attStandingRobot->robot()->currentConfiguration(), dummy,dummy,dummy,dummy );
        else
            break;
        time += attSamplingPeriod;
    }

    for ( unsigned int i = 0; i<Elements.size(); i++ )
        delete Elements[i];

}*/

/////////////AKP: Cut from Mightability_Maps.c

point_co_ordi spline_path_btwn_prev_n_curr_exec[3000];
int no_spline_points_btwn_prev_n_curr_exec=0;

point_co_ordi resultant_spline[5000];
int no_spline_points=0;

int show_spline_path_for_HRP2_hand()
{
 int i=0;
 for(i=0;i<no_spline_points_btwn_prev_n_curr_exec-1;i++)
 {
 ////////g3d_drawOneLine(resultant_spline[i].x,resultant_spline[i].y, resultant_spline[i].z, resultant_spline[i+1].x,resultant_spline[i+1].y, resultant_spline[i+1].z, Red, NULL);
 g3d_drawOneLine(spline_path_btwn_prev_n_curr_exec[i].x,spline_path_btwn_prev_n_curr_exec[i].y, spline_path_btwn_prev_n_curr_exec[i].z, spline_path_btwn_prev_n_curr_exec[i+1].x,spline_path_btwn_prev_n_curr_exec[i+1].y, spline_path_btwn_prev_n_curr_exec[i+1].z, Red, NULL);
  
 }

 
 for(i=0;i<no_HRP2_hand_pos-1;i++)
 {
  g3d_drawOneLine(HRP2_hand_pos_sequence[i][0],HRP2_hand_pos_sequence[i][1], HRP2_hand_pos_sequence[i][2], HRP2_hand_pos_sequence[i+1][0],HRP2_hand_pos_sequence[i+1][1], HRP2_hand_pos_sequence[i+1][2], Blue, NULL);
  
 }
}

double parallel_const_time=0.0;
double parallel_const_task_duration=1.5;//in s
int parallel_constraint_interpolation_created=0;
double parallel_const_sampling_period=10e-3;

int find_HRP2_GIK_sol_for_hand_orientation(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach, int state, int use_body_part)
{
   
   printf(" Inside find_HRP2_GIK_sol_for_hand_orientation() \n");
   configPt cur_rob_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&cur_rob_pos);

  if(state==2)
  {
   initialize_constraints_stack_for_standing_HRP2();// AKP WARNING TODO : Need to make the nsfc and com constraint variables global as done for pc otherwise it will give segmentation fault
  } 

  create_HRP2_hand_pos_constraint(hand_by_reach); 
  create_HRP2_hand_parallel_constraint(hand_by_reach);
  ////create_HRP2_look_at_constraint();

  HRP2_get_joint_mask(hand_by_reach, state, use_body_part); // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body 
 int i=0;
  
 

  state_constraint_tasks.clear();  
   
     
      double start_time=0.0;
 
      int priority=1;// expecting at priority 0 the hand position constraint will be there in the stack
      get_parallel_constraint_interpolation(req_hand_orientation_in_global_frame, hand_by_reach, parallel_const_task_duration, start_time, parallel_const_sampling_period, priority);
      parallel_constraint_interpolation_created=1;
      
    double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(hand_by_reach);//1 for left, 2 for right hand;
 p3d_vector3 target_in_global_frame;

 target_in_global_frame[0]=hand_pos[0];
 target_in_global_frame[1]=hand_pos[1];
 target_in_global_frame[2]=hand_pos[2];

    // if( parallel_constraint_interpolation_created==1)
    // {
      while(parallel_const_time<(parallel_const_task_duration-parallel_const_sampling_period))
      {
      parallel_const_time+=parallel_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_parallel_constraint_interpolated_element(parallel_const_time);

       push_HRP2_hand_pos_constraint(target_in_global_frame, hand_by_reach); // hand_by_reach =1 for left hand, 2 for
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      get_HRP2_GIK_sol();
          
   int check_collision=1;
  if(check_collision==1)
    {
   hrp2_to_M3D_ConfigPt(&cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1],cur_rob_pos);

  p3d_set_and_update_this_robot_conf(ACBTSET->robot,cur_rob_pos);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, res=%d \n", res);
       ////point_of_curr_collision.x=target_in_global_frame[0];
      //// point_of_curr_collision.y=target_in_global_frame[1];
      //// point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_hand_parallel_constraint();
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       cur_gik_sol_configs.no_configs--;
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, res=%d \n", res);
       ////point_of_curr_collision.x=target_in_global_frame[0];
       ////point_of_curr_collision.y=target_in_global_frame[1];
      //// point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_hand_parallel_constraint();
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       cur_gik_sol_configs.no_configs--;
       return 0;
       }
     }
 
    }//End while()
  MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
  delete_HRP2_hand_pos_constraint(); 
  ////if(HRP2_CURRENT_TASK==2&&impose_look_at_constraint==1)// for put object, 
  ////delete_HRP2_look_at_constraint();
  delete_HRP2_hand_parallel_constraint();

  return 1;
} 


int find_HRP2_GIK_sol_for_spline_path(int hand_by_reach, int state, int use_body_part, int maintain_hand_orientation)
{
   int impose_look_at_constraint=0;
   int impose_look_at_human_constraint=0;
   int impose_look_at_hand_path_constraint=0;

   printf(" Inside find_HRP2_GIK_sol_for_spline_path() with no_spline_points=%d\n",no_spline_points);
  configPt cur_rob_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&cur_rob_pos);
   
configPt rob_actual_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); 
   p3d_get_robot_config_into(ACBTSET->robot,&rob_actual_pos);

  if(state==2)
  {
   initialize_constraints_stack_for_standing_HRP2();// AKP WARNING TODO : Need to make the nsfc and com constraint variables global as done for pc otherwise it will give segmentation fault
  } 

  create_HRP2_hand_pos_constraint(hand_by_reach); 
  create_HRP2_hand_parallel_constraint(hand_by_reach);
  create_HRP2_look_at_constraint();

  HRP2_get_joint_mask(hand_by_reach, state, use_body_part); // hand_by_reach =1 for left hand, 2 for right hand, state=1 for sitting HRP2, 2 for standing HRP2. use_body_part=0 means use only the hand, 1 means use upper body, 2 means use whole body 
 int i=0;
 
 double look_at_const_timer=0.0;
  
   double look_at_const_sampling_period=10e-3;
 
 int look_at_constraint_interpolation_created=0;
 int skip_look_const_ctr=0;
 int look_at_human_executing=0;
 int look_at_hand_executing=0;
  
 double *hand_curr_x_orientation;
 p3d_vector3 req_hand_orientation_in_global_frame;

   if(maintain_hand_orientation==1)
      {
       hand_curr_x_orientation=get_HRP2_hand_x_axis_orientation_in_global_frame(hand_by_reach);//1 for left, 2 for right hand
       req_hand_orientation_in_global_frame[0]=hand_curr_x_orientation[0];
       req_hand_orientation_in_global_frame[1]=hand_curr_x_orientation[1];
       req_hand_orientation_in_global_frame[2]=hand_curr_x_orientation[2];
      } 

 int tmp_config_ctr=0;
 int prev_collision_free_config=0;
 double wait_ctr=0.0;
 double wait_cur_state=4;//in s

 vectorN collision_free_config = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model

 

 for(i=0;i<no_spline_points-1;i++)
 {
  p3d_vector3 target_in_global_frame;
  target_in_global_frame[0]=resultant_spline[i].x;
  target_in_global_frame[1]=resultant_spline[i].y;
  target_in_global_frame[2]=resultant_spline[i].z;

  state_constraint_tasks.clear();  

////  printf(" Pushing hand_pos_constraint %d \n",i);
    //if(HRP2_CURRENT_TASK!=2) //For put object
   //////// int hand_pos_push_res=push_HRP2_hand_pos_constraint(target_in_global_frame, hand_by_reach); // hand_by_reach =1 for left hand, 2 for right hand,
     /*if(HRP2_CURRENT_TASK==2&&impose_look_at_constraint==1)// for put object, 
     {
     ////skip_look_const_ctr++;
     ////if(skip_look_const_ctr>500)
      ////{
     printf(" Pushing look_at_constraint for path point %d \n",i);
     int look_at_push_res= push_HRP2_look_at_constraint(target_in_global_frame);
      ////}
     } 
     */

     

     /*
     ////if(HRP2_CURRENT_TASK==10) 
    
     ////if(i>=no_spline_points/2)
     {
      
      if(parallel_constraint_interpolation_created==0)
      {
      p3d_vector3 req_hand_orientation_in_global_frame;
  //The desired orientation of the x axis of wrist frame in the global frame, we want it to be parallel to the z axis of the global frame, i.e. in thumb up orientation
      ////req_hand_orientation_in_global_frame[0]=-0.205737;
      ////req_hand_orientation_in_global_frame[1]=0.321115;
      ////req_hand_orientation_in_global_frame[2]=0.924423 ;
      
      req_hand_orientation_in_global_frame[0]=0;
      req_hand_orientation_in_global_frame[1]=0;
      req_hand_orientation_in_global_frame[2]=1;
   
     
      double start_time=0.0;
 
      int priority=0;
      get_parallel_constraint_interpolation(req_hand_orientation_in_global_frame, hand_by_reach, parallel_const_task_duration, start_time, parallel_const_sampling_period, priority);
      parallel_constraint_interpolation_created=1;
      }
     }  
    
     if( parallel_constraint_interpolation_created==1)
     {
      if(parallel_const_time<(parallel_const_task_duration-parallel_const_sampling_period))
      {
      parallel_const_time+=parallel_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_parallel_constraint_interpolated_element(parallel_const_time);
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      }
      else
      {
       ////printf("Maintaining the last parallel constraint by pushing it with parallel_const_time =%lf\n",parallel_const_time); 
       push_parallel_constraint_interpolated_element(parallel_const_time);

      }
     }
      */
      if(maintain_hand_orientation==1)
      {
      ////printf(" Maintaining the current hand orientation by pushing the parellel constraint (%lf, %lf, %lf)\n",req_hand_orientation_in_global_frame[0],req_hand_orientation_in_global_frame[1],req_hand_orientation_in_global_frame[2]);
      push_HRP2_hand_parallel_constraint(req_hand_orientation_in_global_frame, hand_by_reach);
      }

     int hand_pos_push_res=push_HRP2_hand_pos_constraint(target_in_global_frame, hand_by_reach); // hand_by_reach =1 for left hand, 2 for right hand,
     
     if(HRP2_CURRENT_TASK==2&&wait_ctr>wait_cur_state)
     {
      
      ////printf("impose_look_at_human_constraint=%d, impose_look_at_hand_path_constraint=%d, look_at_human_executing=%d, look_at_hand_executing=%d\n",impose_look_at_human_constraint, impose_look_at_hand_path_constraint, look_at_human_executing, look_at_hand_executing);
      if(impose_look_at_human_constraint==0&&look_at_hand_executing==0&&impose_look_at_hand_path_constraint==0)
      {
      impose_look_at_human_constraint=1;
      look_at_human_executing=1;
      printf(" Look at Human ...\n");
      ////wait_cur_state=0.5;
      }
      else
      {
      if(impose_look_at_human_constraint==1&&look_at_human_executing==0&&impose_look_at_hand_path_constraint==0)
       {
      impose_look_at_human_constraint=0;
      impose_look_at_hand_path_constraint=1;
      look_at_hand_executing=1;
      look_at_constraint_interpolation_created=0;
      printf(" Look at Hand ...\n");
      ////wait_cur_state=2.0;
       }
      }
      wait_ctr=0; 
     }

     if(impose_look_at_human_constraint==1)
     {
      double look_at_const_task_duration=1.5;//in s
      if(look_at_constraint_interpolation_created==0)
      {
      p3d_vector3 look_at_point_in_global_frame;
      double start_time=0;
      double hum_head_pos[3];
      int priority=2;//Assuming 0 as hand pos constraint, 1 for hand parallel constraint
      hum_head_pos[0] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[0][3];//hum_cur_pos[6];
      hum_head_pos[1] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[1][3];//hum_cur_pos[7];
      hum_head_pos[2] = ACBTSET->human[ACBTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->abs_pos[2][3];
      
      look_at_point_in_global_frame[0]=hum_head_pos[0];
      look_at_point_in_global_frame[1]=hum_head_pos[1];
      look_at_point_in_global_frame[2]=hum_head_pos[2];

      get_HRP2_look_at_constraint_interpolation(look_at_point_in_global_frame,  look_at_const_task_duration, start_time, look_at_const_sampling_period, priority);
      look_at_constraint_interpolation_created=1;
      look_at_const_timer=0;
      }

      if(look_at_const_timer<(look_at_const_task_duration-look_at_const_sampling_period))
      {
      look_at_const_timer+=look_at_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_HRP2_look_at_constraint_interpolated_element(look_at_const_timer);
      look_at_human_executing=1;
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      }
      else
      {
       //impose_look_at_human_constraint=0;
       look_at_human_executing=0;
      }
      
     }

     if(impose_look_at_hand_path_constraint==1)
     {
      double look_at_const_task_duration=1.5;//in s
      if(look_at_constraint_interpolation_created==0)
      {
      p3d_vector3 look_at_point_in_global_frame;
      double start_time=0;
      
      int priority=2;//Assuming 0 as hand pos constraint, 1 for hand parallel constraint
     
      look_at_point_in_global_frame[0]=target_in_global_frame[0];
      look_at_point_in_global_frame[1]=target_in_global_frame[1];
      look_at_point_in_global_frame[2]=target_in_global_frame[2];

      get_HRP2_look_at_constraint_interpolation(look_at_point_in_global_frame,  look_at_const_task_duration, start_time, look_at_const_sampling_period, priority);
      look_at_constraint_interpolation_created=1;
      look_at_const_timer=0;
      }

      if(look_at_const_timer<(look_at_const_task_duration-look_at_const_sampling_period))
      {
      look_at_const_timer+=look_at_const_sampling_period;
      ////printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      push_HRP2_look_at_constraint_interpolated_element(look_at_const_timer);
      look_at_hand_executing=1;
      ////printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
      }
      else
      {
       impose_look_at_hand_path_constraint=0;
       look_at_constraint_interpolation_created=0;
       look_at_hand_executing=0;
      }
      
     }
     collision_free_config = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
     wait_ctr+=look_at_const_sampling_period;
    
   ////if((no_spline_points-i)<10)
  ////{
 //// printf(" Pushing look_at_constraint for path point %d \n",i);
////  int look_at_push_res= push_HRP2_look_at_constraint(target_in_global_frame);
////  }
  ////printf(" Calling get_HRP2_GIK_sol()\n");
  /*
  double parallel_const_task_duration=2.0;
   if(i>=no_spline_points/2&&parallel_const_time<parallel_const_task_duration)
   {
    if(parallel_constraint_interpolation_created==0)
    {
     p3d_vector3 req_hand_orientation_in_global_frame;
  //The desired orientation of the x axis of wrist frame in the global frame, we want it to be parallel to the z axis of the global frame, i.e. in thumb up orientation
      req_hand_orientation_in_global_frame[0]=0;
      req_hand_orientation_in_global_frame[1]=0;
      req_hand_orientation_in_global_frame[2]=1;
   
     
      double start_time=0.0;
 
      int priority=1;// expecting at priority 0 the hand position constraint will be there in the stack
      get_parallel_constraint_interpolation(req_hand_orientation_in_global_frame, hand_by_reach, parallel_const_task_duration, start_time, parallel_const_sampling_period, priority);
      parallel_constraint_interpolation_created=1;
    } 

     parallel_const_time+=parallel_const_sampling_period;

    printf("calling push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);
    push_parallel_constraint_interpolated_element(parallel_const_time);
    printf("after push_parallel_constraint_interpolated_element() with parallel_const_time =%lf\n",parallel_const_time);

    ////parallel_const_time+=parallel_const_sampling_period;

   } 
  */
   get_HRP2_GIK_sol();
   
   ////////double *hand_pos;
   
   ////////hand_pos=get_HRP2_hand_center_in_global_frame(hand_by_reach);//1 for left, 2 for right hand;
   ////////printf(" hand_pos=(%lf, %lf, %lf)\n", hand_pos[0], hand_pos[1], hand_pos[2]);
   ////////MY_FREE(hand_pos, double, 3);
   
   int check_collision=1;
  if(check_collision==1)
    {
   if(tmp_config_ctr%100==0||tmp_config_ctr>=no_spline_points-5)
     {
   hrp2_to_M3D_ConfigPt(&cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1],cur_rob_pos);

 
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,cur_rob_pos);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, for the configuration no. %d, res=%d \n", tmp_config_ctr, res);
       printf("with look_at_hand_executing = %d, May be this collision is due to look at hand constraint, so trying to remove this constraint and recalculating. \n",look_at_hand_executing);
        if(look_at_hand_executing==1) //May be this collision is due to look at hand constraint, so try to remove this constraint and recalculate
        {
         impose_look_at_hand_path_constraint=0;
       look_at_constraint_interpolation_created=0;
       look_at_hand_executing=0;
        
        attStandingRobot->staticState ( collision_free_config );//Restoring the previous collision free configuration configuration
         i=prev_collision_free_config-1;
        }
        else
        {
       point_of_curr_collision.x=target_in_global_frame[0];
       point_of_curr_collision.y=target_in_global_frame[1];
       point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_look_at_constraint();
       delete_HRP2_hand_parallel_constraint();
       delete_HRP2_hand_pos_constraint(); 

       //Restore the actual position of the robot
       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       ////////cur_gik_sol_configs.no_configs--;
       //TODO : Delete the configurations till current config to the prev_collision_free_config for freeing the memory
       cur_gik_sol_configs.no_configs=prev_collision_free_config;
       return 0;
        }
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, for the configuration no. %d, res=%d \n", tmp_config_ctr, res);
       point_of_curr_collision.x=target_in_global_frame[0];
       point_of_curr_collision.y=target_in_global_frame[1];
       point_of_curr_collision.z=target_in_global_frame[2]; 
       ////attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
       delete_HRP2_hand_pos_constraint(); 
       delete_HRP2_look_at_constraint();
       delete_HRP2_hand_parallel_constraint();

       p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
       MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
       MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
       ////////cur_gik_sol_configs.no_configs--;
        //TODO : Delete the configurations till current config to the prev_collision_free_config for freeing the memory
       cur_gik_sol_configs.no_configs=prev_collision_free_config;
       return 0;
       }
       prev_collision_free_config=cur_gik_sol_configs.no_configs;
       
      }
/////AKP NOTE : this is tmp storage of hand pos as every configuration, comment this part for making the syatem fast
     double *hand_pos;
     hand_pos=get_HRP2_hand_center_in_global_frame(hand_by_reach);//1 for left, 2 for right hand;
     HRP2_hand_pos_sequence[no_HRP2_hand_pos][0]=hand_pos[0];
     HRP2_hand_pos_sequence[no_HRP2_hand_pos][1]=hand_pos[1]; 
     HRP2_hand_pos_sequence[no_HRP2_hand_pos][2]=hand_pos[2];
     no_HRP2_hand_pos++;
     MY_FREE(hand_pos,double, 3);
//////////////////////      


     }
 tmp_config_ctr++;
 }

  //Restoring the actual position of the robot
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_actual_pos);
  
  MY_FREE(rob_actual_pos, double,ACBTSET->robot->nb_dof); 
  MY_FREE(cur_rob_pos, double,ACBTSET->robot->nb_dof); 
  delete_HRP2_hand_pos_constraint(); 
  ////if(HRP2_CURRENT_TASK==2&&impose_look_at_constraint==1)// for put object, 
  delete_HRP2_look_at_constraint();
  delete_HRP2_hand_parallel_constraint();

  return 1;
} 

point_co_ordi start_hand_pos;

int find_spline_path_for_via_points(point_co_ordi via_points[500], int no_via_points)
{
  

 int continuity_constraint_type=2;//1 means first derivative, i.e. velocity should be continuous everywhere, 2 means second derivative i.e. acceleration should also be continuous everywhere 
 point_co_ordi init_vel, final_vel;
  init_vel.x=0;
  init_vel.y=0;
  init_vel.z=0;
  final_vel.x=0;
  final_vel.y=0;
  final_vel.z=0;

  double total_time=4.0;//in s Total time for start to goal point 
 
   
  ////double sampling_period=attSamplingPeriod; //Set as sampling period used on HRP2, It should be 5e-3
  
  double sampling_period=10e-3;

  int no_spline_pts=get_cubic_spline_by_Hermite_polynomial(via_points, no_via_points, init_vel, final_vel, continuity_constraint_type, sampling_period, total_time, resultant_spline);

  no_spline_points+=no_spline_pts;//Adding in the global counter
  return no_spline_pts; //But returning the no. of pts in call of this function
  

 //return 0;
}

int find_spline_path_for_HRP2_hand(hri_bitmapset * btset, hri_bitmap* bitmap, int hand_by_reach)
{
  point_co_ordi curr_path[500];
  int no_pts=0;
  hri_bitmap_cell* current;
  //int i,j,color;
  //double task_duration=1.0;//1.5;
  if(bitmap == NULL)
    return 0;
  printf(" Inside move_HRP2_hand_with_GIK_on_path with bitmap->searched = %d \n",bitmap->searched);
  if(bitmap->searched) {

    // vertical lines on start and goal
    //g3d_drawOneLine(
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    //g3d_drawOneLine(
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    while(current != bitmap->search_start && current != NULL)
    {
     
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
     current = current->parent;
    }

     //Storing the start cell
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;

     //Storing the actual starting position of the hand, which might be different from the position of the starting cell due to discreatization 
     curr_path[no_pts].x=start_hand_pos.x;
     curr_path[no_pts].y=start_hand_pos.y;
     curr_path[no_pts].z=start_hand_pos.z;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
   


    printf(" No. of cells in the path = %d \n",no_pts);

    
    
    int i=0;
    int j=no_pts-1;
    int i_ctr=0;
    for(i=0;i<=j;i++)
    {
     double x=curr_path[i].x;
     double y=curr_path[i].y;
     double z=curr_path[i].z;
     curr_path[i].x=curr_path[j].x;
     curr_path[i].y=curr_path[j].y;
     curr_path[i].z=curr_path[j].z;
     curr_path[j].x=x;
     curr_path[j].y=y;
     curr_path[j].z=z;
     j--;
    }
    

    /*int i=0;
    int j=no_pts-1;
    for(i=0;i<no_pts;i++)
    {
        
    }*/

 int continuity_constraint_type=2;//1 means first derivative, i.e. velocity should be continuous everywhere, 2 means second derivative i.e. acceleration should also be continuous everywhere 
 point_co_ordi init_vel, final_vel;
  init_vel.x=0;
  init_vel.y=0;
  init_vel.z=0;
  final_vel.x=0;
  final_vel.y=0;
  final_vel.z=0;

  double total_time=4.0;//in s Total time for start to goal point 
 
   
  ////double sampling_period=attSamplingPeriod; //Set as sampling period used on HRP2, It should be 5e-3
  
  double sampling_period=10e-3;
  int no_spline_pts=get_cubic_spline_by_Hermite_polynomial(curr_path, no_pts, init_vel, final_vel, continuity_constraint_type, sampling_period, total_time, resultant_spline);
  no_spline_points=no_spline_pts;
  return 1;
  }//END if(bitmap->searched)
 return 0;
}

int move_HRP2_hand_with_GIK_on_path(hri_bitmapset * btset, hri_bitmap* bitmap, int hand_by_reach)
{
SKIP_FIRST_CONFIG=0;
   vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
 

        int j=0;

        printf(" Inside move_HRP2_hand_with_GIK_on_path(), NO_DOF_HRP2=%d and their valuse are : \n",NO_DOF_HRP2);
        for (  j=6;j<NO_DOF_HRP2;j++ )
        {
        
        
        printf(" backupConfig(%d)=%lf\n",j,backupConfig(j));
        
        }
  //int hand_by_reach=1;//1 for left hand, 2 for right hand
  //double maxi_reach_dist=0.5;
  p3d_vector3 target_in_global_frame;
  p3d_vector3 curr_path[1000];
  int no_pts=0;
  hri_bitmap_cell* current;
  //int i,j,color;
  double task_duration=1.0;//1.5;
  if(bitmap == NULL)
    return 0;
  printf(" Inside move_HRP2_hand_with_GIK_on_path with bitmap->searched = %d \n",bitmap->searched);
  if(bitmap->searched) {

    // vertical lines on start and goal
    //g3d_drawOneLine(
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    //g3d_drawOneLine(
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    while(current != bitmap->search_start && current != NULL)
    {
     
     curr_path[no_pts][0]=current->x*btset->pace+btset->realx;
     curr_path[no_pts][1]=current->y*btset->pace+btset->realy;
     curr_path[no_pts][2]=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
     current = current->parent;
    }
    
    printf(" No. of cells in the path = %d \n",no_pts);
    int i=no_pts-1;
    int i_ctr=0;
    for(;i>=0;i--)
    {
     printf(" Calling HRP2_hand_reach() for %dth path point\n",i); 
     int HRP2_state=HRP2_CURRENT_STATE; //1 for sitting on the chair, 2 for standing
     
    //// if(fabs(curr_path[i][2]-curr_path[0][2])<0.1&&HRP2_CURRENT_TASK!=3)
    ////  THUMB_UP_CONSTRAINT=1;

     int thumb_up_constraint=THUMB_UP_CONSTRAINT;//1 Means the x axis of the hand will be made parallel to the z axis of global frame
    //if(i<=5)
    //thumb_up_constraint=1;// Put the parallel constraint at the few last segments of the path
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
   
    
    int reachable=HRP2_hand_reach(curr_path[i], hand_by_reach, task_duration, HRP2_state, thumb_up_constraint, use_body_part);
    
    
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
    printf(" reachable = %d \n",reachable);
    if(reachable==0)
     {
     printf(" ****** AKP Warning: The GIK solution to reach next cell could not be found, so returning\n");
     attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
     
     return 0;
     }
       //i_ctr++;

     int j=0;
     if(i_ctr>0)
     {
      j=1; //Because the 0th configuration will be the end configuration of previous motion
      SKIP_FIRST_CONFIG=1; //Will be used in HRP2_hand_reach for skipping first configuration which will be the end configuration of previous motion
     }

     for(;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
   printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);

    int kcd_with_report=0;
 //int res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      ////set_kcd_which_test(P3D_KCD_ROB_AUTO);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision with robot, res=%d \n", res);
       point_of_curr_collision.x=curr_path[i][0];
       point_of_curr_collision.y=curr_path[i][1];
       point_of_curr_collision.z=curr_path[i][2]; 
       attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot, res=%d \n", res);
       point_of_curr_collision.x=curr_path[i][0];
       point_of_curr_collision.y=curr_path[i][1];
       point_of_curr_collision.z=curr_path[i][2]; 
       attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
       return 0;
       }
 
   

    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
    printf(" Updated robot with total configuration j =%d \n",j);
    i_ctr++;
    if(i_ctr>=1)
     {
     task_duration=0.5;//task_duration=0.25;
     }
    //return 1; 
    }
    
printf(" Restoring the original configuration of HRP2 GIK model in BioMove3d\n");
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
        
    return 1;
   }

}


int move_HRP2_one_cell_in_curr_path()
{
 
}

int synchronize_HRP2_GIK_model(ghrp2_config_t *hrp2_config)
{

 vectorN HRP2_conf(NO_DOF_HRP2);

        HRP2_conf(0)=hrp2_config->zmp[0];
        HRP2_conf(1)=hrp2_config->zmp[1];
        HRP2_conf(2)=hrp2_config->zmp[2];

        HRP2_conf(3)=hrp2_config->waistRpy[0];
        HRP2_conf(4)=hrp2_config->waistRpy[1];
        HRP2_conf(5)=hrp2_config->waistRpy[2];
        int i=0;

        for (  i=6;i<NO_DOF_HRP2;i++ )
        HRP2_conf(i)=hrp2_config->angles[i];

attStandingRobot->staticState ( HRP2_conf );

   
}

int execute_current_HRP2_GIK_solution(int with_bottle)
{
 printf(" Inside execute_current_HRP2_GIK_solution()\n");
 int i=0;
 configPt bottle_cur_pos;
 configPt rob_cur_pos;
 rob_cur_pos= MY_ALLOC(double,ACBTSET->robot->nb_dof);  

 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

 if(with_bottle==1)
 {
 bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);  

 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 }
/*
 for(i=0;i<HRP2_GIK_sol.no_configs;i++)
   {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  if(with_bottle==1)
  {
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);
  }
  
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
 }*/

 printf(" Before executing the configs, cur_gik_sol_configs.no_configs=%d, rob_cur_pos=(%lf, %lf, %lf)\n",cur_gik_sol_configs.no_configs,rob_cur_pos[6],rob_cur_pos[7],rob_cur_pos[8]);
 for(i=0;i<cur_gik_sol_configs.no_configs;i++)
 {
  ////printf("Executing for i=%d\n",i);
  hrp2_to_M3D_ConfigPt(&cur_gik_sol_configs.gik_sol[i],rob_cur_pos);
  ////printf("After hrp2_to_M3D_ConfigPt()\n");
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,rob_cur_pos);
  if(with_bottle==1)
  {
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]-0.06; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);
  
  }
 NEED_HRP2_VISIBILITY_UPDATE=1;
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 }
 ////printf(" Before synchronize_HRP2_GIK_model()\n");
 ////synchronize_HRP2_GIK_model(&cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1]);
 ////printf(" After synchronize_HRP2_GIK_model()\n");

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

vectorN first_config(NO_DOF_HRP2);   
vectorN HRP2_conf(NO_DOF_HRP2);

        HRP2_conf(0)=rob_cur_pos[6];
        HRP2_conf(1)=rob_cur_pos[7];
        ////////if(HRP2_CURRENT_STATE==1)//for sitting
        ////////HRP2_conf(2)=rob_cur_pos[8]+ M3D_to_HRP2_GIK_sitting_Z_shift;
        if(HRP2_CURRENT_STATE==1)//for sitting
        HRP2_conf(2)=M3D_to_HRP2_GIK_sitting_Z_shift;
        if(HRP2_CURRENT_STATE==2)//for half sitting
        HRP2_conf(2)=0.6487;

first_config(0)=HRP2_conf(0);
first_config(1)=HRP2_conf(1);
first_config(2)=HRP2_conf(2);


        HRP2_conf(3)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].waistRpy[0];
        HRP2_conf(4)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].waistRpy[1];
        HRP2_conf(5)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].waistRpy[2];
 
first_config(3)=cur_gik_sol_configs.gik_sol[0].waistRpy[0]; 
first_config(4)=cur_gik_sol_configs.gik_sol[0].waistRpy[1];
first_config(5)=cur_gik_sol_configs.gik_sol[0].waistRpy[2];
       
        int j=0;

        for (  j=6;j<NO_DOF_HRP2;j++ )
        {
        HRP2_conf(j)=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].angles[j-6];
        first_config(j)=cur_gik_sol_configs.gik_sol[0].angles[j-6];
        }

        printf(" After executing, the current state of HRP2 is :\n");
        
        for(j=0;j< NO_DOF_HRP2;j++)
        {
        
        printf(" first_config(%d)=%lf, backupConfig(%d)=%lf, HRP2_conf(%d) = %lf\n",j,first_config(j),j,backupConfig(j),j,HRP2_conf(j));
        
        }

attStandingRobot->staticState ( HRP2_conf );
////attStandingRobot->staticState ( first_config );

 if(with_bottle==1)
 {
  MY_FREE(bottle_cur_pos, double,ACBTSET->object->nb_dof); 
 }

  MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 
 
printf(" After executing the configs, rob_cur_pos=(%lf, %lf, %lf)\n",rob_cur_pos[6],rob_cur_pos[7],rob_cur_pos[8]);

printf(" Returning from execute_current_HRP2_GIK_solution()\n");

return 1;
  
}

int HRP2_grasp_object(int for_hand,double hand_clench_val)
{

////curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

 ////////double clench=0.4;
double clench=hand_clench_val;


//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
Hand_Clench ( for_hand, clench );
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
//Hand_Clench_without_GIK ( for_hand, clench );

/* AKP NOTE: WARNING Below is bug don't uncomment

 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
   //Restore the actual robot configuration
   p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/

}

int HRP2_release_object(int for_hand, double hand_clench_val)
{
HRP2_GIK_sol.no_configs=0;
 ////////double clench=0.3;
double clench=hand_clench_val;
//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
Hand_Clench ( for_hand, clench );
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration

//Hand_Clench_without_GIK ( for_hand, clench );

/* AKP NOTE: WARNING Below is bug don't uncomment

     int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
//Restore the actual robot configuration
   p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
*/
}

p3d_vector3 point_to_look;

int HRP2_look_at_bottle()
{
    ////configPt bottle_pos   = MY_ALLOC(double,ACBTSET->object->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    ////p3d_get_robot_config_into(ACBTSET->visball,&bottle_pos );
    
  
    
    point_to_look[0] = ACBTSET->object->joints[1]->abs_pos[0][3];
    point_to_look[1] = ACBTSET->object->joints[1]->abs_pos[1][3];
    point_to_look[2] = ACBTSET->object->joints[1]->abs_pos[2][3];
    printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",point_to_look[0],point_to_look[1],point_to_look[2]);

    ////MY_FREE(bottle_pos, double,ACBTSET->object->nb_dof); 
 
    double task_duration=2.0; //in s
    int state=HRP2_CURRENT_STATE;
    int use_body_part=1;//0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet

  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


    HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
HRP2_look_at(point_to_look, task_duration, state, use_body_part);
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    

return 1;    


   
}


int HRP2_look_at_point(p3d_vector3 point_to_look, int use_body_part)//use_body_part=0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet
{
    ////configPt bottle_pos   = MY_ALLOC(double,ACBTSET->object->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    ////p3d_get_robot_config_into(ACBTSET->visball,&bottle_pos );
    
  
    
    
    printf("Point to look is = (%lf, %lf, %lf)\n",point_to_look[0],point_to_look[1],point_to_look[2]);

    ////MY_FREE(bottle_pos, double,ACBTSET->object->nb_dof); 
 
    double task_duration=2.0; //in s
    int state=HRP2_CURRENT_STATE;
    

  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


    HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
HRP2_look_at(point_to_look, task_duration, state, use_body_part);
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    

return 1;    


   
}


int HRP2_look_at_bottle_old()
{
    ////configPt bottle_pos   = MY_ALLOC(double,ACBTSET->object->nb_dof); // We need to use a visball as the robot for finding the collosion free straight line path otherwise if we will use human it might return collision
    ////p3d_get_robot_config_into(ACBTSET->visball,&bottle_pos );
    
  
    
    point_to_look[0] = ACBTSET->object->joints[1]->abs_pos[0][3];
    point_to_look[1] = ACBTSET->object->joints[1]->abs_pos[1][3];
    point_to_look[2] = ACBTSET->object->joints[1]->abs_pos[2][3];
    printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",point_to_look[0],point_to_look[1],point_to_look[2]);

    ////MY_FREE(bottle_pos, double,ACBTSET->object->nb_dof); 
 
    double task_duration=2.0; //in s
    int state=HRP2_CURRENT_STATE;
    int use_body_part=1;//0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet

  int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


    HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
HRP2_look_at(point_to_look, task_duration, state, use_body_part);
attStandingRobot->staticState ( backupConfig );//Restoring the original configuration
    
//AKP NOTE: Warning code below is bug, don't uncomment it

printf("HRP2_GIK_sol.no_configs=%d\n",HRP2_GIK_sol.no_configs);

int j=0;

 for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
////  printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
/*
    int kcd_with_report=0;
 //res = p3d_col_test_self_collision(cur_rob,kcd_with_report);
    int res = p3d_col_test_self_collision(ACBTSET->robot,2);
 ////res = p3d_col_test_robot(cur_rob,2);
		//printf("collision 2:   %i \n",res2);
      set_kcd_which_test(P3D_KCD_ROB_ALL);
      res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is self collision detected with robot at configuration %d, res=%d \n",j, res);
       int pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        int step_back_sol=10;
        int ctr=0;
        for(;pr_sol_ctr>=0&&ctr<step_back_sol;ctr++)
        {
         ////p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[pr_sol_ctr]);
         //// g3d_draw_env();
         //// fl_check_forms();
         //// g3d_draw_allwin_active();
         MY_FREE(HRP2_GIK_sol.configs[pr_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
         HRP2_GIK_sol.no_configs--;
         pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        } 
        //Restoring the actual configuration
        p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
	printf(" Updated robot with total configuration =%d \n",HRP2_GIK_sol.no_configs);
       return 0;
       }

       kcd_with_report=0;
       res = p3d_col_test_robot(ACBTSET->robot,2);
		//printf("collision 2:   %i \n",res2);
       set_kcd_which_test(P3D_KCD_ROB_ALL);
       res = p3d_col_does_robot_collide(ACBTSET->robot->num, p3d_numcoll);
 //// res = p3d_col_test_self_collision(cur_rob->num, p3d_numcoll);
       if(res>0)
       {
       printf(" ***** AKP WARNING : There is collision with robot at configuration %d, res=%d \n", j, res);
       int pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        int step_back_sol=2;
        int ctr=0;
        for(;pr_sol_ctr>=0&&ctr<step_back_sol;ctr++)
        {
         ////p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[pr_sol_ctr]);
         MY_FREE(HRP2_GIK_sol.configs[pr_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
         HRP2_GIK_sol.no_configs--;
         pr_sol_ctr=HRP2_GIK_sol.no_configs-1;
        } 
         //Restoring the actual configuration
        p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
        printf(" Updated robot with total configuration=%d \n",HRP2_GIK_sol.no_configs);
       return 0;
       }
 */

    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  ////g3d_draw_env();
  ////fl_check_forms();
  ////g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
      //Restoring the actual configuration
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[0]);
    printf(" Updated robot with total configuration j =%d \n",j);
return 1;    


   
}


int make_cells_around_HRP2_RHNAD_as_non_obstacle(hri_bitmapset * bitmapset,  int bt_type, double expansion) //TODO : the argument expansion is in m not in terms of no. of cell, and is used as the expansion around BB coordinates, write code to use it
{
  double increment=3.0/4.0*bitmapset->pace;
  envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr = envPt->nr;
  p3d_obj *o;
  p3d_rob *r;
  int x,y,z;
  hri_bitmap * bitmap;

  bitmap = bitmapset->bitmap[bt_type];

  int r_ctr=0;
  
  for(r_ctr=0;r_ctr<nr;r_ctr++)
  {
  
  r = envPt->robot[r_ctr];
  ////printf(" r->name = %s ,r->no=%d \n",r->name,r->no);
  int r_o_ctr=0; 
  for(r_o_ctr=0;r_o_ctr<r->no;r_o_ctr++)
   {
   o = r->o[r_o_ctr];
   ////printf(" o->name = %s \n",o->name);
   if(strcasecmp(o->name,"HRP2ROBOT.RHAND")>=15)
    {
    printf(" o->name = %s \n",o->name);
    printf(" Hand part found ... making the cell values around this hand part= 0\n");
   double BBx;
   for(BBx=o->BB.xmin-expansion;BBx<o->BB.xmax+expansion;BBx+=increment)
     {
    
    x=(BBx- bitmapset->realx)/bitmapset->pace;  
    double BBy;
    for(BBy=o->BB.ymin-expansion;BBy<o->BB.ymax+expansion&&x>0&&x<bitmap->nx;BBy+=increment)
     {
     
     y=(BBy- bitmapset->realy)/bitmapset->pace;  
     double BBz;
     for(BBz=o->BB.zmin-expansion;BBz<o->BB.zmax+expansion&&y>0&&y<bitmap->ny;BBz+=increment)
      {
     
      z=(BBz- bitmapset->realz)/bitmapset->pace;  
      if(z>0&&z<bitmap->nz)
       {
        printf(" Making cell (%d,%d,%d) as value 0 \n",x,y,z); 
        bitmapset->bitmap[bt_type]->data[x][y][z].val = 0;  
       }
      }
     }
    }
   }
  }
 } 
}

int HRP2_find_collision_free_path_to_take_object_new()//In this version, the entire path is divided into three phases, first reach near to the bottle, then orient the hand then again plan a path from the new hand position to the bottle while maintaining the orientation
{
/*
find_candidate_points_on_plane_to_put_obj_new();
assign_weights_on_candidte_points_to_put_obj();
reverse_sort_weighted_candidate_points_to_put_obj();
CANDIDATE_POINTS_FOR_TASK_FOUND=1;
return 0;
*/

/*
find_candidate_points_to_show_obj_new();
assign_weights_on_candidte_points_to_show_obj();
reverse_sort_weighted_candidate_points_to_show_obj();
CANDIDATE_POINTS_FOR_TASK_FOUND=1;
return 0;
*/

/*
find_candidate_points_to_hide_obj_new();
assign_weights_on_candidte_points_to_hide_obj();
reverse_sort_weighted_candidate_points_to_hide_obj();
CANDIDATE_POINTS_FOR_TASK_FOUND=1;
return 0;
*/

 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_find_collision_free_path_to_take_object(), before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 ////create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 

 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 //As now the robot is in rest pos, the hand position will be stored for returning to rest pos
 right_hand_rest_pos.x=hand_pos[0];
 right_hand_rest_pos.y=hand_pos[1];
 right_hand_rest_pos.z=hand_pos[2];

//****This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 printf(" **** Storing right_hand_rest_pos: (%lf, %lf,%lf) \n",right_hand_rest_pos.x,right_hand_rest_pos.y,right_hand_rest_pos.z);
 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3]+0.1;
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

double expansion=0.05;//in m
double cur_hand_pos[3];
cur_hand_pos[0]=qs[0];
cur_hand_pos[1]=qs[1];
cur_hand_pos[2]=qs[2];
printf(" Calling  make_cells_around_HRP2_RHNAD_as_non_obstacle() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

expansion=1;//no of cells around the cell corresponding to the point qf
make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 

expansion=1;//no of cells around the cell corresponding to the point qf
 make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

//qf[2]+=0.1;
printf(" Finding path from hand to bottle \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to bottle could not be found \n");
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;

    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_path_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path-3); //-3 is Just to reach near the bottle, not exactly at bottle before that hand has to be properly oriented
    printf(" After find_spline_path_for_via_points() to reach near the bottle, no_path_pts=%d, no_spline_points=%d\n",no_path_pts, no_spline_points);
    //Storing the spline points for showing the path
    no_spline_points_btwn_prev_n_curr_exec=0;
 
    for(int tmp_i=0;tmp_i<no_path_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(no_path_pts==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to reach near the bottle \n");
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
    }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=0;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to reach near the bottle \n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
    
 
 printf("****Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
/////////***********///////////
     
     p3d_vector3 req_hand_orientation_in_global_frame;
     req_hand_orientation_in_global_frame[0]=0;
     req_hand_orientation_in_global_frame[1]=0;
     req_hand_orientation_in_global_frame[2]=1;
     
     GIK_path_res=find_HRP2_GIK_sol_for_hand_orientation(req_hand_orientation_in_global_frame,for_hand, state, use_body_part);
     
     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to orient the HRP2 hand for grasping the bottle \n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
     

    printf("NOW finding the path from current correctly oriented hand to the bottle\n");
    //NOW finding the path from current correctly oriented hand to the bottle
     hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 
     qs[0]=hand_pos[0]; 
     qs[1]=hand_pos[1];
     qs[2]=hand_pos[2];

 //This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3]+0.1;
  ////////z_val_of_grasp_point=qf[2];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */



////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

   
  cur_hand_pos[0]=qs[0];
  cur_hand_pos[1]=qs[1];
  cur_hand_pos[2]=qs[2];
  ////printf(" Calling make_cells_around_point_obstacle_free() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 ////make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

////expansion=1;
//// make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

printf(" Finding path from new correctly oriented hand position to bottle \n");
 val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from new correctly oriented hand position to bottle could not be found \n");
  attStandingRobot->staticState ( backupConfig );
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

/*
//Add the exact position of the bottle as the end point as the above path is based on the nearest cell
     curr_AStar_path[no_pts_curr_AStar_path].x=qf[0];
     curr_AStar_path[no_pts_curr_AStar_path].y=qf[1];
     curr_AStar_path[no_pts_curr_AStar_path].z=qf[2];
     no_pts_curr_AStar_path++;
*/

    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
     no_path_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path-1);//Find path one cell before the bottle
    
   //Storing the spline points for showing the path
    
   
    for(int tmp_i=0;tmp_i<no_path_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  ////////no_spline_points++;
    }
    if(no_path_pts==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to reach the bottle for grasping \n");
    attStandingRobot->staticState ( backupConfig );
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
    }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to reach the bottle for grasping.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
    

/////****** Now finding the st. line interpolated path between the cell very near to the bottle to the bottle

hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right h

point_co_ordi tmp_via_pts[2];
tmp_via_pts[0].x=hand_pos[0];
tmp_via_pts[0].y=hand_pos[1];
tmp_via_pts[0].z=hand_pos[2];



tmp_via_pts[1].x=ACBTSET->object->joints[1]->abs_pos[0][3];
tmp_via_pts[1].y=ACBTSET->object->joints[1]->abs_pos[1][3];
tmp_via_pts[1].z=ACBTSET->object->joints[1]->abs_pos[2][3]+0.1;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
z_val_of_grasp_point=qf[2];

double sampling_period=5e-3; 
no_spline_points=0;
double t2=0.0;
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;



   
    resultant_spline[no_spline_points].x=x2;
    resultant_spline[no_spline_points].y=y2;
    resultant_spline[no_spline_points].z=z2;
    no_spline_points++;

    spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
    spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
    spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
    no_spline_points_btwn_prev_n_curr_exec++;
     }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to reach the bottle for grasping.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
 printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

/////////***********///////////

attStandingRobot->staticState ( backupConfig );
p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

if(cur_gik_sol_configs.no_configs>0)
return cur_gik_sol_configs.no_configs;
else
return 0;

////show_HRP2_gik_sol();
 /*
int i=0;
//while(1)
//{
        

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);

  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
*/

//store_cur_GIK_sol_for_real_HRP2();
 
}

int HRP2_find_collision_free_path_to_take_object()
{
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_find_collision_free_path_to_take_object(), before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 ////create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 

 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 //As now the robot is in rest pos, the hand position will be stored for returning to rest pos
 right_hand_rest_pos.x=hand_pos[0];
 right_hand_rest_pos.y=hand_pos[1];
 right_hand_rest_pos.z=hand_pos[2];

  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 printf(" **** Storing right_hand_rest_pos: (%lf, %lf,%lf) \n",right_hand_rest_pos.x,right_hand_rest_pos.y,right_hand_rest_pos.z);
 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

double expansion=0.025;//in m
double cur_hand_pos[3];
cur_hand_pos[0]=qs[0];
cur_hand_pos[1]=qs[1];
cur_hand_pos[2]=qs[2];
printf(" Calling make_cells_around_point_obstacle_free() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

printf(" Finding path from hand to bottle \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to bottle could not be found \n");
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to take the bottle \n");
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
    }

    printf("****Inside HRP2_find_collision_free_path_to_take_object(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     int maintain_hand_orientation=0;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     return 0;
     }
    
 printf("****Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
/////////***********///////////


     

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
 /*
int i=0;
//while(1)
//{
        

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);

  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
*/

//store_cur_GIK_sol_for_real_HRP2();
 
}



int HRP2_return_hand_to_rest_position()
{
 HRP2_CURRENT_TASK=3;
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];

 qf[0]=right_hand_rest_pos.x;
 qf[1]=right_hand_rest_pos.y;
 qf[2]=right_hand_rest_pos.z;

 int no_trials=0;
 int path_found=0;
 int break_loop=0;
 while(no_trials<10&&break_loop==0)
 {

 printf(" **** Using right_hand_rest_pos: (%lf, %lf,%lf) \n",right_hand_rest_pos.x,right_hand_rest_pos.y,right_hand_rest_pos.z);
  
int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);

  if(path_res>0)
  {
printf("**** Path found for returning to rest position for the right hand of HRP2\n");
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;

 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();
 
//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

////AKP NOTE : Following function call is for straight line interpolation
////   int col_free_GIK_path=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);


//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
    no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to return to rest position \n");
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }
    
     int maintain_hand_orientation=0;
    int col_free_GIK_path=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
attStandingRobot->staticState ( backupConfig );

     if(col_free_GIK_path==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to return to rest position\n");
     return 0;
     }
    
/////////////////END for using spline



p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
   if(col_free_GIK_path==0)
   {
   printf(" no_trials=%d\n",no_trials);
   printf(" *** Could not found collision free path for returning the right hand to the rest position, trying another path\n");
   int extension=1;
   make_cells_around_point_as_obstacle(grid_around_HRP2.GRID_SET, HRP2_GIK_MANIP, point_of_curr_collision,extension);
   
   }
   else
   {
   path_found=1;
   break_loop=1;
   }
  }
 no_trials++;
 }
 if(path_found==1)
 {
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];
////show_HRP2_gik_sol();

printf(" HRP2_GIK_sol.no_configs=%d\n",HRP2_GIK_sol.no_configs);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
   {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
    }
 */
 return 1;

 }
 else
 {
  printf(" AKP WARNING : Could not found collision free path for returning the right hand to the rest position\n");
  return 0;
 }

 
 
  
 

}



int HRP2_put_object_for_human_to_take_new()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_put_object_for_human_to_take_new() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

MY_FREE(hand_pos, double, 3);




  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 


ChronoOff();

ChronoOn();



//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
no_pts_curr_AStar_path=0;
no_spline_points=0;
 vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;
no_spline_points_btwn_prev_n_curr_exec=0;


find_candidate_points_on_plane_to_put_obj_new();
printf("***candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);
if(candidate_points_to_put.no_points==0)
{
 ChronoOff();
printf("AKP WARNING: candidate_points_to_put.no_points=%d, so not trying to put the bottle.\n",candidate_points_to_put.no_points);
return 0;
}

assign_weights_on_candidte_points_to_put_obj();
reverse_sort_weighted_candidate_points_to_put_obj();


 //////***** Finding a path to first lift the hand with bottle

printf(" Finding path to lift the hand with bottle \n");

point_co_ordi tmp_via_pts[2];
tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];



tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z+0.06;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];

double sampling_period=5e-3; 
no_spline_points=0;
double t2=0.0;
printf("  qs = (%lf, %lf, %lf), qf = (%lf, %lf, %lf)  \n", qs[0], qs[1], qs[2], qf[0], qf[1], qf[2]);
    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
  if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }

   int maintain_hand_orientation=1;
   int  GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lift the bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lift the hand with bottle


  double *new_hand_pos;
new_hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
printf(" Earlier qs was (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);
 qs[0]=new_hand_pos[0];
 qs[1]=new_hand_pos[1];
 qs[2]=new_hand_pos[2];
MY_FREE(new_hand_pos, double, 3);
printf(" After lifting hand new qs is (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);

//////// find_candidate_points_on_plane_to_put_obj_new();
////////assign_weights_on_candidte_points_to_put_obj();
////////reverse_sort_weighted_candidate_points_to_put_obj();

//This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=qs[0];
  start_hand_pos.y=qs[1];
  start_hand_pos.z=qs[2];
////////////////////////////
 configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 
 p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;





///////////////////////
vectorN backupConfig_for_next_iteration  = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model after lifting the bottle, it will be used to restore the configuration, if the path to next iteration is not found, so that testing for the next candidate point could be done
printf("*****candidate_points_to_put.no_points=%d\n",candidate_points_to_put.no_points);

 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  

  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  ////////qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  qf[2]=qs[2]; //To maintain the z value 
printf(" Testing for candidate_points_to_put %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
   
    bottle_cur_pos[6]=qf[0];
  bottle_cur_pos[7]=qf[1];
  bottle_cur_pos[8]=candidate_points_to_put.point[test_p_ctr].z-0.06;
  
   int OK_to_Show=0;
   p3d_set_and_update_this_robot_conf(ACBTSET->object, bottle_cur_pos);
   if(psp_is_object_visible(currRob, ACBTSET->object, 95, FALSE))
   {
   OK_to_Show=0;
   }
   else
   {
   OK_to_Show=1;
   }

   if(OK_to_Show==1)
   {
 int expansion=0;
//AKP NOTE: TODO Undo the cells arround obstacle free if need to test for next point
 make_cells_around_point_obstacle_free(qf,expansion); 


  printf(" Finding path from hand to cnadidate point \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to candidate point %d could not be found \n", test_p_ctr);
 ////return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
      ////////  cur_gik_sol_configs.no_configs=0;
      ////////  cur_gik_sol_configs.gik_sol[0].time=0.0; 
////////p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

////////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path); 

    //Storing the spline points for showing the path
    ////////no_spline_points_btwn_prev_n_curr_exec=0;
    int prev_no_spline_points_btwn_prev_n_curr_exec=no_spline_points_btwn_prev_n_curr_exec;

    for(int tmp_i=0;tmp_i<no_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(no_pts>0)
    {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_put_found=1;
  HRP2_HAND_spline_path_calculated=1;
   printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=1;

    double prev_no_conf=cur_gik_sol_configs.no_configs;
    double prev_time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time; 

    int prev_no_HRP2_hand_pos=no_HRP2_hand_pos;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );
printf("****Inside HRP2_put_object_for_human_to_take_new(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to put the bottle for candidate point %d\n",test_p_ctr);
    no_spline_points_btwn_prev_n_curr_exec=prev_no_spline_points_btwn_prev_n_curr_exec;
    no_HRP2_hand_pos=prev_no_HRP2_hand_pos;
    cur_gik_sol_configs.no_configs=prev_no_conf;
    cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time=prev_time; 
  
    printf(" So Resetting the valid config and time as : cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf \n",cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    
      attStandingRobot->staticState ( backupConfig_for_next_iteration );
     ////attStandingRobot->staticState ( backupConfig );
     ////return 0;
     }
     else
     {

   

   point_to_put_found=1;
  ////////attStandingRobot->staticState ( backupConfig );
  break;
  
     }
    }
  else
    {
   printf("Fail to find path for candidate_points_to_put.point[%d] \n",test_p_ctr);
    }
    
  }//End if(OK_to_Show)
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n"); 
 attStandingRobot->staticState ( backupConfig );
 return 0;
 }


 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];



no_spline_points=0;
 


 //////***** Finding a path to lower the hand with bottle

printf(" Finding path to lower the hand with bottle \n");

tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];

tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z-0.04;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];


no_spline_points=0;
 t2=0.0;
printf("  qs = (%lf, %lf, %lf), qf = (%lf, %lf, %lf) \n", qs[0], qs[1], qs[2], qf[0], qf[1], qf[2]);
    //Storing the spline points for showing the path
    
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
   
    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lower the hand with bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lower the hand with bottle
 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;

    
   ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

   printf(" Returning from HRP2_put_object_for_human_to_take_new();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
attStandingRobot->staticState ( backupConfig );

return 1;
////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}


int HRP2_hide_object_from_human_new()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_hide_object_from_human_new() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

MY_FREE(hand_pos, double, 3);




  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 


ChronoOff();

ChronoOn();

find_candidate_points_to_hide_obj_new();
assign_weights_on_candidte_points_to_hide_obj();
reverse_sort_weighted_candidate_points_to_hide_obj();

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0.0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
no_pts_curr_AStar_path=0;
no_spline_points=0;
 vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;
no_spline_points_btwn_prev_n_curr_exec=0;
 //////***** Finding a path to first lift the hand with bottle

printf(" Finding path to lift the hand with bottle \n");

point_co_ordi tmp_via_pts[2];
tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];



tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z+0.06;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];

double sampling_period=5e-3; 
no_spline_points=0;
double t2=0.0;
printf("  qs (%lf, %lf, %lf) = \n", qs[0], qs[1], qs[2]);
    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_hide_object_from_human_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
  if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }

   int maintain_hand_orientation=1;
   int  GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lift the bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("**** after calling find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lift the hand with bottle


  double *new_hand_pos;
new_hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
printf(" Earlier qs was (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);
 qs[0]=new_hand_pos[0];
 qs[1]=new_hand_pos[1];
 qs[2]=new_hand_pos[2];
MY_FREE(new_hand_pos, double, 3);
printf(" After lifting hand new qs is (%lf, %lf, %lf) \n",qs[0],qs[1],qs[2]);

//////// find_candidate_points_on_plane_to_put_obj_new();
////////assign_weights_on_candidte_points_to_put_obj();
////////reverse_sort_weighted_candidate_points_to_put_obj();

//This is important, as it will be used in get_AStar_path() to store the first point as the current position of hand
  start_hand_pos.x=qs[0];
  start_hand_pos.y=qs[1];
  start_hand_pos.z=qs[2];

 
 int test_p_ctr=0;
///////////////////////////


 int point_to_hide_found=0;
 

 configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 
 p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

vectorN backupConfig_for_next_iteration  = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model after lifting the bottle, it will be used to restore the configuration, if the path to next iteration is not found, so that testing for the next candidate point could be done
printf(" ****candidate_points_to_hide.no_points=%d\n",candidate_points_to_hide.no_points);
 for(;test_p_ctr<candidate_points_to_hide.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_hide %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);

  
  qf[0]=candidate_points_to_hide.point[test_p_ctr].x;
  qf[1]=candidate_points_to_hide.point[test_p_ctr].y;
  qf[2]=candidate_points_to_hide.point[test_p_ctr].z;
  bottle_cur_pos[6]=qf[0];
  bottle_cur_pos[7]=qf[1];
  bottle_cur_pos[8]=qf[2];

  qf[2]=qs[2];
  ////qf[2]=qs[2]; //To maintain the z value 
  
   int OK_to_Hide=0;
   p3d_set_and_update_this_robot_conf(ACBTSET->object, bottle_cur_pos);
   if(psp_is_object_visible(currRob, ACBTSET->object, 5, FALSE))
   {
   OK_to_Hide=0;
   }
   else
   {
   OK_to_Hide=1;
   }

   point_to_hide_found=0;

   if(OK_to_Hide==1)
   { 
 int expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

point_to_hide_found=0;

 int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to candidate point %d could not be found \n", test_p_ctr);
 ////return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }

no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
      ////////  cur_gik_sol_configs.no_configs=0;
      ////////  cur_gik_sol_configs.gik_sol[0].time=0.0; 
////////p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

////////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path); //-3 is Just to reach near the bottle, not exactly at bottle before that hand has to be properly oriented

    //Storing the spline points for showing the path
    ////////no_spline_points_btwn_prev_n_curr_exec=0;
    int prev_no_spline_points_btwn_prev_n_curr_exec=no_spline_points_btwn_prev_n_curr_exec;

    for(int tmp_i=0;tmp_i<no_pts-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(no_pts>0)
    {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_hide_found=1;
  HRP2_HAND_spline_path_calculated=1;
   printf("****Inside HRP2_hide_object_from_human(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=1;

    double prev_no_conf=cur_gik_sol_configs.no_configs;
    double prev_time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time; 
    int prev_no_HRP2_hand_pos=no_HRP2_hand_pos;

    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );
printf("****Inside HRP2_hide_object_from_human(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to hide the bottle for candidate point %d\n",test_p_ctr);
    cur_gik_sol_configs.no_configs=prev_no_conf;
    cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time=prev_time; 
    no_spline_points_btwn_prev_n_curr_exec=prev_no_spline_points_btwn_prev_n_curr_exec;
     no_HRP2_hand_pos=prev_no_HRP2_hand_pos;

     attStandingRobot->staticState ( backupConfig_for_next_iteration );
     ////return 0;
     }
     else
     {

   

   point_to_hide_found=1;
  ////////attStandingRobot->staticState ( backupConfig );
  break;
  
     }
    }
  else
    {
   printf("Fail to find path for candidate_points_to_put.point[%d] \n",test_p_ctr);
    }
  }//END if(OK_to_Hide==1)
 }




///////////////////

 

if(point_to_hide_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n"); 
 attStandingRobot->staticState ( backupConfig );
 return 0;
 }


 ////hand_pos=MY_ALLOC(double,3); 
hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;

  ////hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

MY_FREE(hand_pos, double, 3);

no_spline_points=0;
 


 //////***** Finding a path to lower the hand with bottle

printf(" Finding path to lower the hand with bottle \n");

tmp_via_pts[0].x=qs[0];
tmp_via_pts[0].y=qs[1];
tmp_via_pts[0].z=qs[2];

tmp_via_pts[1].x=tmp_via_pts[0].x;
tmp_via_pts[1].y=tmp_via_pts[0].y;
tmp_via_pts[1].z=tmp_via_pts[0].z-0.04;

qf[0]=tmp_via_pts[1].x;
qf[1]=tmp_via_pts[1].y;
qf[2]=tmp_via_pts[1].z;
////z_val_of_grasp_point=qf[2];


no_spline_points=0;
 t2=0.0;
printf("  qs (%lf, %lf, %lf) = \n", qs[0], qs[1], qs[2]);
    //Storing the spline points for showing the path
    
      for(;t2<1.0;t2+=sampling_period) 
       { 
      
      //g3d_drawDisc(x,y,z,grid_around_HRP2.GRID_SET->pace/4.0,2,NULL);

      double x2=(1-t2)*tmp_via_pts[0].x+t2*tmp_via_pts[1].x;
      double y2=(1-t2)*tmp_via_pts[0].y+t2*tmp_via_pts[1].y;
      double z2=(1-t2)*tmp_via_pts[0].z+t2*tmp_via_pts[1].z;

      resultant_spline[no_spline_points].x=x2;
      resultant_spline[no_spline_points].y=y2;
      resultant_spline[no_spline_points].z=z2;
      no_spline_points++;

      //////printf(" point (%lf, %lf, %lf)\n",x2,y2,z2);
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=x2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=y2;
      spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=z2;
      no_spline_points_btwn_prev_n_curr_exec++;
       }

    printf("****Inside HRP2_put_object_for_human_to_take_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);


   ////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
   
    maintain_hand_orientation=1;
    GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not find a collision free path to lower the hand with bottle.\n");
     attStandingRobot->staticState ( backupConfig );
     return 0;
     }
 
     printf("****At the end of Inside HRP2_find_collision_free_path_to_take_object(), after find_HRP2_GIK_sol_for_spline_path(),  cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

 ////////******** End finding path to lower the hand with bottle
 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;

    
   ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

   printf(" Returning from HRP2_put_object_for_human_to_take_new();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
attStandingRobot->staticState ( backupConfig );
return 1;


}


int HRP2_put_object_for_human_to_take()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

  start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 


ChronoOff();

ChronoOn();

 find_candidate_points_on_plane_to_put_obj_new();
assign_weights_on_candidte_points_to_put_obj();
reverse_sort_weighted_candidate_points_to_put_obj();

 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_put %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  ////////qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  qf[2]=qs[2]; //To maintain the z value 

   
 int expansion=0;
 make_cells_around_point_obstacle_free(qf,expansion); 



  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_put_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_put.point[%d] \n",test_p_ctr);
  }
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n");
 return 0;
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to put the bottle \n");
    attStandingRobot->staticState ( backupConfig );
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }

    int maintain_hand_orientation=1;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);


attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to put the bottle \n");
      ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

     return 0;
     }
    
   ChronoPrint("<<<<<<>>>>>>> TIME of Finding the solution for put : ");
   ////printf(" %lf, %lf \n",tu,ts);
   ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

printf(" Returning from HRP2_put_object_for_human_to_take();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}

int HRP2_show_object_to_human_new()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
ChronoOff();
ChronoOn();
 find_candidate_points_to_show_obj_new();
assign_weights_on_candidte_points_to_show_obj();
reverse_sort_weighted_candidate_points_to_show_obj();
vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
no_HRP2_hand_pos=0;

vectorN backupConfig_for_next_iteration  = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model after lifting the bottle, it will be used to restore the configuration, if the path to next iteration is not found, so that testing for the next candidate point could be done

 int point_to_show_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_show.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_show %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
  qf[0]=candidate_points_to_show.point[test_p_ctr].x;
  qf[1]=candidate_points_to_show.point[test_p_ctr].y;
  qf[2]=candidate_points_to_show.point[test_p_ctr].z;
  ////qf[2]=qs[2]; //To maintain the z value 

   
 int expansion=0;
 make_cells_around_point_obstacle_free(qf,expansion); 

no_spline_points=0;

  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_show.point[%d] \n",test_p_ctr);
  
  

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
       cur_gik_sol_configs.no_configs=0;
       cur_gik_sol_configs.gik_sol[0].time=0; 
       p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation


    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }

////////////////////////////////////////////////
no_pts_curr_AStar_path=0;
int no_pts_AStar_path= get_AStar_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);//It will store the result in global variable curr_AStar_path

//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

  curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
      ////////  cur_gik_sol_configs.no_configs=0;
      ////////  cur_gik_sol_configs.gik_sol[0].time=0.0; 
////////p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);
//AKP NOTE: Uncomment following if required
////disactivate_collision_among_parts_of_HRP2_RHAND();

//****AKP NOTE: Following function call is for using liner interpolation 
////////int path_res=move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
///////if(path_res==0)
     ///////{
     ////////printf(" **** AKP WARNING : Could not found a collision free path to take the bottle \n");
     ///////return 0;
     ///////}

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation


    ////int path_res=find_spline_path_for_HRP2_hand(for_hand);
    no_spline_points=0;
    int no_pts=find_spline_path_for_via_points(curr_AStar_path, no_pts_curr_AStar_path); //-3 is Just to reach near the bottle, not exactly at bottle before that hand has to be properly oriented

      no_spline_points_btwn_prev_n_curr_exec=0;
      int prev_no_spline_points_btwn_prev_n_curr_exec=no_spline_points_btwn_prev_n_curr_exec;

    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }
    if(no_pts>0)
    {
  printf("Path found for candidate_points_to_show.point[%d] \n",test_p_ctr);
  point_to_show_found=1;
  HRP2_HAND_spline_path_calculated=1;
   printf("****Inside HRP2_show_object_to_human_new(), before calling find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);
    int maintain_hand_orientation=1;

    double prev_no_conf=cur_gik_sol_configs.no_configs;
    double prev_time=cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time; 
     int prev_no_HRP2_hand_pos=no_HRP2_hand_pos;
    
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part, maintain_hand_orientation);
////attStandingRobot->staticState ( backupConfig );
printf("****Inside HRP2_put_object_for_human_to_take_new(), after find_HRP2_GIK_sol_for_spline_path(), cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to put the bottle for candidate point %d\n",test_p_ctr);
    cur_gik_sol_configs.no_configs=prev_no_conf;
    cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time=prev_time; 
    no_spline_points_btwn_prev_n_curr_exec=prev_no_spline_points_btwn_prev_n_curr_exec;
     prev_no_HRP2_hand_pos=prev_no_HRP2_hand_pos;

     attStandingRobot->staticState ( backupConfig_for_next_iteration );
     ////return 0;
     }
      else
     {

   point_to_show_found=1;
  ////////attStandingRobot->staticState ( backupConfig );
  break;
  
     }
    }
  else
    {
   printf("Fail to find path for candidate_points_to_show.point[%d] \n",test_p_ctr);
    }
  }//End if(path_res>0)
 }//End for(;test_p_ctr<candidate_points_to_show.no_points;test_p_ctr++)




///////////////////

 

if(point_to_show_found==0)
 {
 printf(" **** AKP Warning : No feasible point to show the object has been found.\n"); 
 attStandingRobot->staticState ( backupConfig );
 return 0;
 }
else
 {
  printf(" **** Feasible point to show the object has been found.\n"); 
  attStandingRobot->staticState ( backupConfig );
  return 1;
 } 

 

}

int HRP2_show_object_to_human()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
ChronoOff();
ChronoOn();
 find_candidate_points_to_show_obj_new();
assign_weights_on_candidte_points_to_show_obj();
reverse_sort_weighted_candidate_points_to_show_obj();

 int point_to_show_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_show.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_show %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);
  qf[0]=candidate_points_to_show.point[test_p_ctr].x;
  qf[1]=candidate_points_to_show.point[test_p_ctr].y;
  qf[2]=candidate_points_to_show.point[test_p_ctr].z;
  ////qf[2]=qs[2]; //To maintain the z value 

   
 int expansion=0;
 make_cells_around_point_obstacle_free(qf,expansion); 



  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_show.point[%d] \n",test_p_ctr);
  point_to_show_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_show.point[%d] \n",test_p_ctr);
  }
 }

if(point_to_show_found==0)
 {
 printf(" **** AKP Warning : No feasible point to show the object has been found.\n");
 return 0;
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to show the bottle \n");
    attStandingRobot->staticState ( backupConfig );
    return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }

    int maintain_hand_orientation=1;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);

attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to show the bottle \n");
     return 0;
     }
    
   ChronoPrint("\n<<<<<< Time for finding final solution for show ");
ChronoOff();

p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

printf(" Returning from HRP2_show_object_to_human();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}

int HRP2_hide_object_from_human()
{
 HRP2_CURRENT_TASK=2;//To put object
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside HRP2_hide_object_from_human(), before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 //create_HRP2_robot(state);
 double *hand_pos;
 

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 start_hand_pos.x=hand_pos[0];
  start_hand_pos.y=hand_pos[1];
  start_hand_pos.z=hand_pos[2];
  
/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 ChronoOff();
ChronoOn();
 find_candidate_points_to_hide_obj_new();
assign_weights_on_candidte_points_to_hide_obj();
reverse_sort_weighted_candidate_points_to_hide_obj();

 int point_to_hide_found=0;
 int test_p_ctr=0;

 configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
 
 p3d_rob * currRob=ACBTSET->human[ACBTSET->actual_human]->HumanPt;

 for(;test_p_ctr<candidate_points_to_hide.no_points;test_p_ctr++)
 {
  printf(" Testing for candidate_points_to_hide %d with qs=(%lf, %lf,%lf) and qf=(%lf, %lf, %lf)\n",test_p_ctr, qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);

  
  qf[0]=candidate_points_to_hide.point[test_p_ctr].x;
  qf[1]=candidate_points_to_hide.point[test_p_ctr].y;
  qf[2]=candidate_points_to_hide.point[test_p_ctr].z;
  bottle_cur_pos[6]=qf[0];
  bottle_cur_pos[7]=qf[1];
  bottle_cur_pos[8]=qf[2];

  qf[2]=qs[2];
  ////qf[2]=qs[2]; //To maintain the z value 
  
   int OK_to_Hide=0;
   p3d_set_and_update_this_robot_conf(ACBTSET->object, bottle_cur_pos);
   if(psp_is_object_visible(currRob, ACBTSET->object, 5, FALSE))
   {
   OK_to_Hide=0;
   }
   else
   {
   OK_to_Hide=1;
   }

   point_to_hide_found=0;

   if(OK_to_Hide==1)
   { 
 int expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

point_to_hide_found=0;

  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_hide.point[%d] \n",test_p_ctr);
  point_to_hide_found=1;
  ////break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_hide.point[%d] \n",test_p_ctr);
   point_to_hide_found=0;
  }
  }
 //////}

//////if(point_to_show_found==0)
////// {
////// printf(" **** AKP Warning : No feasible point to hide the object has been found.\n");
////// return 0;
////// } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
  if(point_to_hide_found==1)
  {
 int prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<curr_gik_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(curr_gik_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 

curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach

  prev_sol_ctr=0;
  for(prev_sol_ctr=0;prev_sol_ctr<HRP2_GIK_sol.no_configs;prev_sol_ctr++)
  {
  MY_FREE(HRP2_GIK_sol.configs[prev_sol_ctr], double,ACBTSET->robot->nb_dof);  // Freeing temporary robot config structure 
  } 


HRP2_GIK_sol.no_configs=0;

////vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

//**** Initializing the time and no. of configurations which will be populated in HRP2_hand_reach called iteratively in the following function
        cur_gik_sol_configs.no_configs=0;
        cur_gik_sol_configs.gik_sol[0].time=0; 
p3d_col_deactivate_rob_rob(ACBTSET->robot,ACBTSET->object);

//AKP NOTE: Following function call is for using linear interpolation
////THUMB_UP_CONSTRAINT=0;
////   move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);
////THUMB_UP_CONSTRAINT=0;

//****AKP NOTE: Following part replaces the above function call of straight line interpolation and uses spline interpolation

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();//Taking the current configuration of HRP2 GIK model
    int use_body_part=0;//0 : use hand only, 1: use upper body, 2: use whole body
    if(HRP2_CURRENT_STATE==2) //Robot is in half sitting position so we could allow upper body motion without worrying about collision of opposite hand with the chair
    {
    use_body_part=1;//0 : use hand only, 1: use upper body, 2: use whole body
    }
    int path_res=find_spline_path_for_HRP2_hand(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

    //Storing the spline points for showing the path
     no_spline_points_btwn_prev_n_curr_exec=0;
    for(int tmp_i=0;tmp_i<no_spline_points-1;tmp_i++)
    {
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].x=resultant_spline[tmp_i].x;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].y=resultant_spline[tmp_i].y;
  spline_path_btwn_prev_n_curr_exec[no_spline_points_btwn_prev_n_curr_exec].z=resultant_spline[tmp_i].z;
  no_spline_points_btwn_prev_n_curr_exec++;
  
    }

    if(path_res==0)
    {
    printf(" **** AKP WARNING : Could not found a spline path to hide the bottle \n");
    attStandingRobot->staticState ( backupConfig );
    point_to_hide_found=0;
    //////return 0;
    }
    else
    {
     HRP2_HAND_spline_path_calculated=1;
 
     }

    int maintain_hand_orientation=1;
    int GIK_path_res=find_HRP2_GIK_sol_for_spline_path(for_hand, state, use_body_part,maintain_hand_orientation);

attStandingRobot->staticState ( backupConfig );

     if(GIK_path_res==0)
     {
     printf(" **** AKP WARNING : Could not found a collision free path to hide the bottle \n");
     point_to_hide_found=0;
     //////return 0;
     }
     else
     {
      point_to_hide_found=1;
      break;
     }
   }
 }

 if(point_to_hide_found==0)
 {
 printf(" **** AKP Warning : No feasible point to hide the object has been found.\n");
 return 0;
 } 
    
   
ChronoPrint("\n <<<<<<<<>>>>>>>>>>>Time for getting the solution to hide object\n");
ChronoOff();
p3d_col_activate_rob_rob(ACBTSET->robot,ACBTSET->object);
////attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

printf(" Returning from HRP2_hide_object_from_human();()\n");
   printf("cur_gik_sol_configs.no_configs=%d,start time=%lf, end time=%lf\n", cur_gik_sol_configs.no_configs,cur_gik_sol_configs.gik_sol[0].time,cur_gik_sol_configs.gik_sol[cur_gik_sol_configs.no_configs-1].time);

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
////configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
//// p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);
/*
int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  //envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  //envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  //envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  //bottle_cur_pos[6]=hand_pos[0];
  //bottle_cur_pos[7]=hand_pos[1];
  //bottle_cur_pos[8]=hand_pos[2];

  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

*/
 

}

int put_object_for_human_to_take_old() //Which is actually a combination of first grasp object by HRP2 then it will put the object
{
 
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);


  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" put_object_for_human_to_take_old, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 

 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
////p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

////create_3d_grid_for_HRP2_GIK();

int expansion=1;
double cur_hand_pos[3];
cur_hand_pos[0]=qs[0];
cur_hand_pos[1]=qs[1];
cur_hand_pos[2]=qs[2];
printf(" Calling make_cells_around_point_obstacle_free() with cur_hand_pos=(%lf,%lf,%lf)\n",cur_hand_pos[0],cur_hand_pos[1],cur_hand_pos[2]);
 ////make_cells_around_point_obstacle_free(cur_hand_pos,expansion); 
 make_cells_around_HRP2_RHNAD_as_non_obstacle(grid_around_HRP2.GRID_SET,HRP2_GIK_MANIP, expansion);

expansion=1;
 make_cells_around_point_obstacle_free(qf,expansion); 

////create_3d_grid_for_HRP2_GIK();

printf(" Finding path from hand to bottle \n");
int val=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
if(val<0)
 {
 printf("AKP Warning: path from hand to bottle could not be found \n");
 return 0;
 }
else
 {
 HRP2_GIK_path_calculated=1;
 }
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
int i=0;
//while(1)
//{

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

//}  

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 find_candidate_points_on_plane_to_put_obj();
 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  printf(" Testing for %d \n",test_p_ctr);
  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_put[%d] \n",test_p_ctr);
  point_to_put_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_put[%d] \n",test_p_ctr);
  }
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n");
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);

i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  /*envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  bottle_cur_pos[6]=hand_pos[0];
  bottle_cur_pos[7]=hand_pos[1];
  bottle_cur_pos[8]=hand_pos[2];
*/
  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }


 

}



int grid_created=0;
int show_3d_grid_for_HRP2_GIK()
{
if(grid_created==0)
 {
 grid_created=1;
 double qs[3];
 double qf[3];
 configPt rob_cur_pos = MY_ALLOC(double,ACBTSET->robot->nb_dof); /* Allocation of temporary robot configuration */
 p3d_get_robot_config_into(ACBTSET->robot,&rob_cur_pos);

 qs[0]= rob_cur_pos[6]+0.1;
 qs[1]= rob_cur_pos[7]-0.9;
 qs[2]= rob_cur_pos[8]+0.8;
 
 qf[0]=qs[0]+0.5;
 qf[1]=qs[1]+1.75;
 qf[2]=qs[2]+0.35;
  
 int for_hand=2;//1 for left, 2 for right hand;
 //p3d_vector3 hand_pos;
 printf(" Inside show_3d_grid_for_HRP2_GIK, before calling get_HRP2_hand_center_in_global_frame() \n");
 int state=HRP2_CURRENT_STATE; //AKP : 1 is sitting, 2 is half standing 
 create_HRP2_robot(state);
 double *hand_pos;
 hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

 /*
 int bottle_indx=get_index_of_robot_by_name("bottle");
 if(bottle_indx==NULL)
 bottle_indx=get_index_of_robot_by_name("BOTTLE");
     
 printf(" bottle_indx = %d \n",bottle_indx);
     
 
 envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
 qf[0]=envPt->robot[bottle_indx]->ROBOT_POS[6];
 qf[1]=envPt->robot[bottle_indx]->ROBOT_POS[7];
 qf[2]=envPt->robot[bottle_indx]->ROBOT_POS[8]+0.05;
 
  printf("bottle pos from envPt = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);
 */ 
 
  qf[0]=ACBTSET->object->joints[1]->abs_pos[0][3];
  qf[1]=ACBTSET->object->joints[1]->abs_pos[1][3];
  qf[2]=ACBTSET->object->joints[1]->abs_pos[2][3];
  printf("bottle pos from ACBTSET->object = (%lf, %lf, %lf)\n",qf[0],qf[1],qf[2]);

 /*qf[0]=qs[0]+0.25;
 qf[1]=qs[1]+0.5;
 qf[2]=qs[2]+0.45;
  */

MY_FREE(rob_cur_pos, double,ACBTSET->robot->nb_dof); 

////p3d_col_deactivate_rob_rob(ACBTSET->visball,envPt->robot[bottle_indx]);
p3d_col_deactivate_rob_rob(ACBTSET->visball,ACBTSET->object);

create_3d_grid_for_HRP2_GIK();

 
//printf("
Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;
move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
int i=0;
//while(1)
//{

 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
 
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

//}  

  hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
 
 /*qs[0]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[1]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 qs[2]=ACBTSET->robot->joints[ROBOTj_RHAND]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 */
 qs[0]=hand_pos[0]; 
 qs[1]=hand_pos[1];
 qs[2]=hand_pos[2];

/* qf[0]=qs[0]+0.1;
 qf[1]=qs[1]+0.3;
 qf[2]=qs[2]+0.1;
*/
 
 int point_to_put_found=0;
 int test_p_ctr=0;
 for(;test_p_ctr<candidate_points_to_put.no_points;test_p_ctr++)
 {
  printf(" Testing for %d \n",test_p_ctr);
  qf[0]=candidate_points_to_put.point[test_p_ctr].x;
  qf[1]=candidate_points_to_put.point[test_p_ctr].y;
  qf[2]=candidate_points_to_put.point[test_p_ctr].z+0.05;
  int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
  if(path_res>0)
  {
  printf("Path found for candidate_points_to_put.point[%d] \n",test_p_ctr);
  point_to_put_found=1;
  break;
  }
  else
  {
   printf("Fail to find path for candidate_points_to_put[%d] \n",test_p_ctr);
  }
 }

if(point_to_put_found==0)
 {
 printf(" **** AKP Warning : No feasible point to put the object has been found.\n");
 } 
////int path_res=Find_AStar_Path(qs, qf, grid_around_HRP2.GRID_SET, 1);
//// grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->searched=1;
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;

vectorN backupConfig = attStandingRobot->robot()->currentConfiguration();

/*
double clench=1;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );



 int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/

move_HRP2_hand_with_GIK_on_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP],for_hand);

attStandingRobot->staticState ( backupConfig );
 
qs_tmp[0]=qs[0];
qs_tmp[1]=qs[1];
qs_tmp[2]=qs[2];

qf_tmp[0]=qf[0];
qf_tmp[1]=qf[1];
qf_tmp[2]=qf[2];

////show_HRP2_gik_sol();
////configPt bottle_cur_pos = MY_ALLOC(double,envPt->robot[bottle_indx]->nb_dof); /* Allocation of temporary robot configuration */
configPt bottle_cur_pos = MY_ALLOC(double,ACBTSET->object->nb_dof);
//// p3d_get_robot_config_into(envPt->robot[bottle_indx],&bottle_cur_pos);
 p3d_get_robot_config_into(ACBTSET->object,&bottle_cur_pos);

i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
  /*envPt->robot[bottle_indx]->ROBOT_POS[6]=hand_pos[0];
  envPt->robot[bottle_indx]->ROBOT_POS[7]=hand_pos[1];
  envPt->robot[bottle_indx]->ROBOT_POS[8]=hand_pos[2];
  bottle_cur_pos[6]=hand_pos[0];
  bottle_cur_pos[7]=hand_pos[1];
  bottle_cur_pos[8]=hand_pos[2];
*/
  ////p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);
  p3d_set_and_update_this_robot_conf(ACBTSET->object,bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }

/* 
curr_gik_sol.no_configs=0; // It will be reset to 0 in every call to HRP2_hand_reach
HRP2_GIK_sol.no_configs=0;
    clench=5;
Hand_Clench ( for_hand, clench );
//Hand_Clench_without_GIK ( for_hand, clench );


  j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  ////printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  //HRP2_GIK_sol
    p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
    HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs] = MY_ALLOC(double,ACBTSET->robot->nb_dof); 

    p3d_get_robot_config_into(ACBTSET->robot,&HRP2_GIK_sol.configs[HRP2_GIK_sol.no_configs]);
   //  printf(" Before calling hrp2_to_M3D_ConfigPt() for curr_gik_sol.configs[%d]\n",curr_gik_sol.no_configs);
     
     
    HRP2_GIK_sol.no_configs++;
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  //g3d_draw_env();
  //fl_check_forms();
  //g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }

i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  //hand_pos=get_HRP2_hand_center_in_global_frame(for_hand);//1 for left, 2 for right hand;
  
  //printf(" ROBOTj_GRIP=%d\n",ROBOTj_GRIP);
  bottle_cur_pos[6]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[0][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[7]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[1][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
  bottle_cur_pos[8]=ACBTSET->robot->joints[ROBOTj_GRIP]->abs_pos[2][3]; // AKP: In the abs_pos[4][4] matrix the x,y,z are stored at indices [0][3], [1][3], [2][3] respectively
 
 
  p3d_set_and_update_this_robot_conf(envPt->robot[bottle_indx],bottle_cur_pos);

  //printf(" bottle_cur_pos = (%lf, %lf, %lf)\n",bottle_cur_pos[6],bottle_cur_pos[7],bottle_cur_pos[8]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
 
*/

 }

 /*show_3d_grid_Bounding_box_for_HRP2_GIK();
 g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 show_HRP2_gik_sol();
 */
 ////find_HRP2_GIK_collision_free_path(qs, qf, grid_around_HRP2.GRID_SET, 1);
 
 /*
     int j=0;
     for(j=0;j<curr_gik_sol.no_configs;j++)
     {
   //cur_i=i;
  printf(" Updating robot with configuration j =%d \n",j);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,curr_gik_sol.configs[j]);
  
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
  g3d_draw_env();
  fl_check_forms();
  g3d_draw_allwin_active();
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
     }
*/
 //show_gik_sol();

 int show_obstacle_cells=1;
 if(show_obstacle_cells==1)
 {
 int x=0;
 for(x=0;x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;x++)
 {
 int y=0;
 for(y=0;y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;y++)
  {
  int z=0;
  for(z=0;z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz;z++)
   {
    if(grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->data[x][y][z].val<0)
    {
       double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
       double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
       double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
       g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, grid_around_HRP2.GRID_SET->pace/4.0, Red, NULL);
        //fl_check_forms();
       //g3d_draw_allwin_active();
    }
    ////else
    ////{
    ////  double cell_x_world = grid_around_HRP2.GRID_SET->realx + (x * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_y_world = grid_around_HRP2.GRID_SET->realy + (y * grid_around_HRP2.GRID_SET->pace);
    ////  double cell_z_world = grid_around_HRP2.GRID_SET->realz + (z * grid_around_HRP2.GRID_SET->pace);
    ////  g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, 0.005, Green, NULL);
      
    ////} 
    //  fl_check_forms();
   }
  } 
 }
 }//End if(show_obstacle_cells==1)
}


int show_HRP2_gik_sol()
{
 int ctr=100;
 while(SHOW_HRP2_ENTIRE_GIK_SOL==1)
 {
 int i=0;
 for(i=0;i<HRP2_GIK_sol.no_configs&&SHOW_HRP2_ENTIRE_GIK_SOL==1;i++)
  {
  //cur_i=i;
  ////printf(" Drawing with i =%d \n",i);
  //g3d_drawDisc(5,-2, 2, 1, 3, NULL);
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[i]);
  // show_3d_grid_Bounding_box_for_HRP2_GIK();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  //printf("robot_hand_reach_goal[%d]=%lf, %lf, %lf\n",i,robot_hand_reach_goal[i][0],robot_hand_reach_goal[i][1],robot_hand_reach_goal[i][2]);
  //g3d_drawDisc(to_reach_target[0], to_reach_target[1], to_reach_target[2], 0.5, 4, NULL);
 g3d_draw_env();
 fl_check_forms();
 g3d_draw_allwin_active();
 //g3d_drawDisc(qs_tmp[0], qs_tmp[1],qs_tmp[2], 0.05, Red, NULL);
 //g3d_drawDisc(qf_tmp[0], qf_tmp[1],qf_tmp[2], 0.05, Green, NULL);
 
 //hri_bt_show_path(grid_around_HRP2.GRID_SET, grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]);
 
  //g3d_drawDisc(robot_hand_reach_goal[i][0], robot_hand_reach_goal[i][1], robot_hand_reach_goal[i][2], 0.5, 4, NULL);
  
  }
  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[0]);

 }

  //Resetting the robot to its first configuration
  p3d_set_and_update_this_robot_conf(ACBTSET->robot,HRP2_GIK_sol.configs[0]);

}




int get_AStar_path(hri_bitmapset * btset, hri_bitmap* bitmap)
{
  point_co_ordi curr_path[500];
  int no_pts=0;
  hri_bitmap_cell* current;
  //int i,j,color;
  //double task_duration=1.0;//1.5;
  if(bitmap == NULL)
    return 0;
  printf(" Inside move_HRP2_hand_with_GIK_on_path with bitmap->searched = %d \n",bitmap->searched);
  if(bitmap->searched) {

    // vertical lines on start and goal
    //g3d_drawOneLine(
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
      //  bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    //g3d_drawOneLine(
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
      //  bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    while(current != bitmap->search_start && current != NULL)
    {
     
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
     current = current->parent;
    }

     //Storing the start cell
     curr_path[no_pts].x=current->x*btset->pace+btset->realx;
     curr_path[no_pts].y=current->y*btset->pace+btset->realy;
     curr_path[no_pts].z=current->z*btset->pace+btset->realz;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;

    /* //Storing the actual starting position of the hand, which might be different from the position of the starting cell due to discreatization 
     curr_path[no_pts].x=start_hand_pos.x;
     curr_path[no_pts].y=start_hand_pos.y;
     curr_path[no_pts].z=start_hand_pos.z;
     ////printf(" Storing in curr_path[%d] = (%lf, %lf, %lf) \n",no_pts, curr_path[no_pts][0], curr_path[no_pts][1], curr_path[no_pts][2]);
     no_pts++;
   */


    printf(" No. of cells in the path = %d \n",no_pts);
 
   //Storing the hand pos as first point
   //////// curr_AStar_path[no_pts_curr_AStar_path].x=start_hand_pos.x;
   ////////  curr_AStar_path[no_pts_curr_AStar_path].y=start_hand_pos.y;
   ////////  curr_AStar_path[no_pts_curr_AStar_path].z=start_hand_pos.z;
   ////////  no_pts_curr_AStar_path++;

    //Storing the hand pos as first point
    curr_AStar_path[no_pts_curr_AStar_path].x=start_hand_pos.x;
     curr_AStar_path[no_pts_curr_AStar_path].y=start_hand_pos.y;
     curr_AStar_path[no_pts_curr_AStar_path].z=start_hand_pos.z;
     no_pts_curr_AStar_path++;


    int j=no_pts-1;
    
   
    for(;j>=0;j--)
    {
     curr_AStar_path[no_pts_curr_AStar_path].x=curr_path[j].x;
     curr_AStar_path[no_pts_curr_AStar_path].y=curr_path[j].y;
     curr_AStar_path[no_pts_curr_AStar_path].z=curr_path[j].z;
     no_pts_curr_AStar_path++;
    } 
   return no_pts_curr_AStar_path;
  }
  else
  return 0;
   
}




/*********************ASTAR**************************************/
/*!
 * \brief Calculate the cost of a cell when reached from a different cell
 * sets cell-> val unless for collision in soft obstacle
 * \param cell the cell
 *
 * \return FALSE and sets cell value to a negative number in case of a collision
 */
/****************************************************************/
int Calculate_AStar_Cell_Value(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell )
{
  configPt qc,q_o;
  double saved[3];

 
      cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);
      cell->q = qc;
   


  return cell->val >= 0; // non-negative means no collision
}

/*********************ASTAR***************************************/
/*!
 * \brief A* search: calculate neighbours
 *
 * all neighbors not opened yet will be opened, all openedneighbors will
 * all opened neighbors will be updated if they are cheaper to reach by the center cell
 *
 * \param bitmap the bitmap
 * \param center_cell whose neighbours
 * \param final_cell  goal cell
 * \param reached TRUE if we reach to the goal
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int Calculate_AStar_neigh_costs(hri_bitmapset* btset, hri_bitmap* bitmap, hri_bitmap_cell* center_cell, hri_bitmap_cell* final_cell, int bitmap_index)
{
 //// printf("Inside Calculate_AStar_neigh_costs\n");
  int i,j,k; // loop xyz coordinates
  int x, y, z; // center cell coordinates
  int refx1, refy1, refx2, refy2; // 2step reference cell xy coordinates
  hri_bitmap_cell *current_cell, *overstepped1, *overstepped2;
  double new_cell_g, overstep_val, distance;

  if(center_cell == NULL)
  { 
    printf(" AKP Warning: center_cell = NULL, so returning false \n");
    return FALSE;
  }
  else{
    x=center_cell->x; y=center_cell->y; z=center_cell->z;
  }

  if( ! on_map(center_cell->x, center_cell->y, center_cell->z, bitmap) ) {
    //PrintError(("cant get cell\n"));
    return FALSE;
  }

  /* iterate over all direct non-closed neighbors in bitmap */
  for(i=-1; i<2; i++){ // -1 to 1
    for(j=-1; j<2; j++){ // -1 to 1
      for(k=-1; k<2; k++){// -1 to 1
     // printf(" Checking cell (%d, %d, %d) \n",x+i, y+j, z+k);  
        if(i==0 && j==0 && k==0) continue; // center cell
        if (! on_map(x+i, y+j, z+k, bitmap)) {
          //printf(" Cell is not in the map \n");
          continue;
        }

        if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
          //PrintError(("Can't get cell\n"));
          return FALSE;
        }

        if(btset->bitmap[bitmap_index]->data[x+i][y+j][z+k].val < 0 )
        {
       // printf(" cell (%d, %d, %d) is at obstacle\n",x+i, y+j, z+k);  
        ////printf(" cell is at obstacle \n");
        continue; /* Is the cell in obstacle? */
        }

        /* closed cells already have a minimum path to start, and all neighbors opened */
        if(current_cell->closed) 
        {
       // printf(" Current cell is already closed \n");
        continue;
        }
        /* open cells might have less g cost for going through current node */
        int manhattan_distance = ABS(i) + ABS(j) + ABS(k);
         if (manhattan_distance == 1) {
           distance= 1; // normal grid step
         } else if(manhattan_distance == 2) {
           distance = M_SQRT2; // 2d diagonal step
         } else if(manhattan_distance == 3) {
           distance = M_SQRT3; // 3d diagonal step
         }
        if (current_cell->open) {
        // printf(" current cell is open \n");
          new_cell_g = hri_bt_A_CalculateCellG(btset, current_cell, center_cell, distance);

          if(current_cell->g > new_cell_g){
            current_cell->g =  new_cell_g;
            current_cell->parent = center_cell;
            hri_bt_A_update_cell_OL(current_cell);
          } else {
            continue;
          }

        } else { // cell was neither open nor closed
          //printf(" current cell is neither open nor closed \n");
          if (Calculate_AStar_Cell_Value(btset, bitmap, current_cell, center_cell) == FALSE)
            continue;// leave untouched
          current_cell->h = hri_bt_dist_heuristic(bitmap,current_cell->x,current_cell->y,current_cell->z);
          // TK:dead code never used because of if before
//          if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].val == BT_OBST_POTENTIAL_COLLISION
//              && btset->manip == BT_MANIP_NAVIGATION) {
//            fromcellno = get_direction(current_cell, center_cell);
//            if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].obstacle[fromcellno]==TRUE) // it was !=, PRAGUE
//              continue;
//          }

          current_cell->g = hri_bt_A_CalculateCellG(btset, current_cell, center_cell, distance);
          current_cell->parent = center_cell;

           //printf("Calling hri_bt_A_insert_OL(), g=%f now\n",current_cell->g); 
          hri_bt_A_insert_OL(current_cell);
          current_cell->open = TRUE;
        }
      }
    }
  } // end for immediate neighbors

  /**
   * as an optimisation in 2D to basic A* over the grid, we also search the 2 step
   * diagonals that would be reached by going 1 step horizontal or vertical, and one step diagonal.
   *
   * We use the value and open properties use for single grid steps calculated earlier.
   */
  if (bitmap->nz == 1) {
    k = 0;
    for(i=-2; i<3; i++){ // -2 to 2
      for(j=-2; j<3; j++){ // -2 to 2
        //        for(k=-2; k<3; j++){ // -2 to 2
        if (abs(i)!=2 && abs(j)!=2 && abs(k)!=2) continue; // not a 2 step border cell
        if (i==0 || j==0 || k==0) {
          // in plane on center cell, target cells are 3 manhattan steps away
          if (abs(i) + abs(j) + abs(k) != 3) {
            continue;
          }
        } else { // 3D case
          if (abs(i) + abs(j) + abs(k) != 4) {
            continue;
          }
        }
        if (! on_map(x+i, y+j, z+k, bitmap)) {
          continue;
        }


        if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
          PrintError(("Can't get cell\n"));
          return FALSE;
        }

        if(btset->bitmap[bitmap_index]->data[x+i][y+j][z+k].val < 0) continue; /* Is the cell in obstacle? */

        /* closed cells already have a minimum path to start, and all neighbors opened */
        if(current_cell->closed) continue;

        // 2D code, 2 reference points stepped over
        //ref1 is horizontal / vertical stepped over cell, ref2 diagonally stepped over cell
        if (i == -2) {
          refx1 = -1;
        } else if (i == 2) {
          refx1 = 1;
        } else {
          refx1 = 0;
        }
        if (j == -2) {
          refy1 = -1;
        } else if (j == 2) {
          refy1 = 1;
        } else {
          refy1 = 0;
        }
        if (i < 0) {
          refx2 = -1;
        } else  {
          refx2 = 1;
        }
        if (j < 0) {
          refy2 = -1;
        } else  {
          refy2 = 1;
        }
        overstepped1 = hri_bt_get_cell(bitmap, x+refx1, y+refy1, z+k);
        overstepped2 = hri_bt_get_cell(bitmap, x+refx2, y+refy2, z+k);

        // only continue if both overstepped cells have been opened, and add the max val to g later
        if (overstepped1->open && overstepped2->open) {
          overstep_val =  (overstepped1->val + overstepped2->val) / 2 ;
        } else {
          /* TODO: For more completeness, check the 2 step localpath for collisions,
           * and calculate equivalent cost for a double step */
          continue;
        }

        /* open cells might have less g cost for going through current node */
        if (current_cell->open) {
          new_cell_g = overstep_val + hri_bt_A_CalculateCellG(btset, current_cell, center_cell, M_SQRT5);

          if(current_cell->g > new_cell_g){
            current_cell->g =  new_cell_g;
            current_cell->parent = center_cell;
            hri_bt_A_update_cell_OL(current_cell);
          } else {
            continue;
          }

        } else { // cell was neither open nor closed
          if (Calculate_AStar_Cell_Value(btset, bitmap, current_cell, center_cell) == FALSE)
            continue;// leave untouched
          current_cell->h = hri_bt_dist_heuristic(bitmap,current_cell->x,current_cell->y,current_cell->z);

          current_cell->g = overstep_val + hri_bt_A_CalculateCellG(btset, current_cell, center_cell, M_SQRT5);
          current_cell->parent = center_cell;

          /*  printf("It is g=%f now\n",current_cell->g); */
          ////printf("Calling hri_bt_A_insert_OL(), g=%f now\n",current_cell->g); 
          hri_bt_A_insert_OL(current_cell);
          current_cell->open = TRUE;
        }
        //        }
      }
    } // end for distant diagonal neighbors
  } // end if 2d then smooth
////printf(" Returning true\n");
  return TRUE;
}

double calculate_AStar_path(hri_bitmapset *btset, hri_bitmap* bitmap, int bitmap_index)
{

  hri_bitmap_cell * current_cell;;
  int reached = FALSE;

  

  if(bitmap->search_start == NULL || bitmap->search_goal == NULL){
    printf("AKP Warning Inside calculate_AStar_path(), but start/final cell is NULL\n");
    return -1;
  }
  if(bitmap->search_start == bitmap->search_goal) {
  printf("AKP Warning Inside calculate_AStar_path(), but bitmap->search_start == bitmap->search_goal\n");
    return 0;
  }

  current_cell = bitmap->search_start;

  hri_bt_init_BinaryHeap(bitmap); // ALLOC 

  if(!hri_bt_close_cell(bitmap,current_cell)){
    PrintError(("cant close start cell!\n"));
    return -1;
  }

  printf(" Calling Calculate_AStar_neigh_costs for current_cell (%d,%d,%d)\n",current_cell->x,current_cell->y,current_cell->z);
  Calculate_AStar_neigh_costs(btset, bitmap, current_cell, bitmap->search_goal,bitmap_index);
  int ctr=0;

  while(!reached) {
   ////printf(" ctr = %d\n", ctr);
   ctr++;
    if( hri_bt_A_Heap_size()==0){
      PrintError(("hri_bt_A_Heap_size()=0, A*:no path found!"));
      hri_bt_destroy_BinaryHeap();
      return -1;
    }
    current_cell = hri_bt_A_remove_OL();
    if(current_cell == bitmap->search_goal) {
      reached = TRUE;
      break;
    }
    hri_bt_close_cell(bitmap, current_cell);
    Calculate_AStar_neigh_costs(btset, bitmap, current_cell, bitmap->search_goal,bitmap_index);
  }
  bitmap->searched = TRUE;

  printf("\ncost: %f \n", hri_bt_A_CalculateCellG(btset, bitmap->search_goal, bitmap->search_goal->parent, getCellDistance(bitmap->search_goal, bitmap->search_goal->parent)));


  // TK: This line looks like a bug, as bitmapset definitions do not allow bitmaps to change type
  //  bitmap->type = BT_PATH;

  hri_bt_destroy_BinaryHeap();


  return hri_bt_A_CalculateCellG(btset, bitmap->search_goal, bitmap->search_goal->parent, getCellDistance(bitmap->search_goal, bitmap->search_goal->parent));
}

void  hri_bt_reset_GIK_path(hri_bitmapset * btset)
{
  hri_bitmap* bitmap;

  bitmap = btset->bitmap[HRP2_GIK_MANIP];

  if(bitmap == NULL){
    PrintError(("Try to erase a path from non path bitmap\n"));
    return;
  }

  bitmap->search_start = NULL;
  bitmap->search_goal = NULL;
  bitmap->current_search_node = NULL;
  bitmap->searched = FALSE;

  int x,y,z;

  if(bitmap == NULL)
    return;

  for(x=0; x<bitmap->nx; x++){
    for(y=0; y<bitmap->ny; y++){
      for(z=0; z<bitmap->nz; z++){
        ////bitmap->data[x][y][z].val = 0;
        bitmap->data[x][y][z].h = -1;
        bitmap->data[x][y][z].g = 0;
        bitmap->data[x][y][z].parent = NULL;
        bitmap->data[x][y][z].closed = FALSE;
        bitmap->data[x][y][z].open   = FALSE;
        bitmap->data[x][y][z].x = x;
        bitmap->data[x][y][z].y = y;
        bitmap->data[x][y][z].z = z;
        bitmap->data[x][y][z].locked = FALSE;
      }
    }
  }
  ////hri_bt_reset_bitmap_data(bitmap);

  btset->pathexist = FALSE;

}

int Find_AStar_Path(double qs[3], double qf[3], hri_bitmapset* bitmapset, int manip)
{
  ////p3d_graph *G;
  ////p3d_list_node *graph_node;
  printf(" Inside Find_AStar_Path with qs=(%lf,%lf,%lf) and qf=(%lf,%lf,%lf) \n",qs[0],qs[1],qs[2],qf[0],qf[1],qf[2]);

  hri_bitmap* bitmap;
  bitmap = bitmapset->bitmap[HRP2_GIK_MANIP];
 
  //// if(!BTGRAPH)     G = hri_bt_create_graph(bitmapset->robot);
  ////else             G = BTGRAPH;

  ////bitmapset->robot->GRAPH = G;
  ////graph_node = G->nodes; 
   
  ////configPt q_s = p3d_get_robot_config(bitmapset->robot);
  ////G->search_start  = p3d_APInode_make(G,q_s);
  ////p3d_insert_node(G,G->search_start);
  ////p3d_create_compco(G,G->search_start);
  ////G->search_start->type = ISOLATED;

  ////configPt  q_g = p3d_get_robot_config(G->rob);

  ////G->search_goal  = p3d_APInode_make(G,q_g);
  ////p3d_insert_node(G,G->search_goal);
  ////p3d_create_compco(G,G->search_goal);
  ////G->search_goal->type = ISOLATED;

  hri_bitmap_cell* new_search_start=NULL;
  hri_bitmap_cell* new_search_goal=NULL;
  
  new_search_start  =  hri_bt_get_closest_cell(bitmapset, bitmap, qs[0], qs[1], qs[2]);
  new_search_goal  =   hri_bt_get_closest_cell(bitmapset, bitmap, qf[0], qf[1], qf[2]);

  if(new_search_start == NULL) {
    printf("Search start cell does not exist\n");
    bitmapset->pathexist = FALSE;
    return -4;
  }
  if(new_search_goal == NULL ){
    printf("Search goal cell does not exist\n");
    bitmapset->pathexist = FALSE;
    return -4;
  }  

// reset the path, also clears data from earlier failed attempts when !pathexist
  hri_bt_reset_GIK_path(bitmapset);
 

  printf(" search_start cell (%d, %d, %d) \n",new_search_start->x,new_search_start->y,new_search_start->z);
  printf(" search_goal cell (%d, %d, %d) \n",new_search_goal->x,new_search_goal->y,new_search_goal->z);
  bitmap->search_start = new_search_start;
  bitmap->search_goal  = new_search_goal;

  printf("Values at start and goal cells are %lf, %lf\n",bitmap->data[new_search_start->x][new_search_start->y][new_search_start->z].val,bitmap->data[new_search_goal->x][new_search_goal->y][new_search_goal->z].val);

 if( bitmap->data[new_search_start->x][new_search_start->y][new_search_start->z].val<0)
  {
  printf(" AKP Warning : Start cell is at obstacle, so can't find path \n");
  return -1;
  }
 
 if( bitmap->data[new_search_goal->x][new_search_goal->y][new_search_goal->z].val<0)
  {
  printf(" AKP Warning : Goal cell is at obstacle, so can't find path \n");
  return -1;
  }
  

  /******** calculating the path costs *************/
  double result = calculate_AStar_path(bitmapset, bitmap, HRP2_GIK_MANIP);

 


  if (result > -1) {
    bitmapset->pathexist = TRUE;
    return result;
  } else {
    bitmapset->pathexist = FALSE;
    return -5;
  }
}


int get_Hermite_polynomial_points(point_co_ordi st_point, point_co_ordi end_point, point_co_ordi st_vel, point_co_ordi end_vel, double sampling_rate, point_co_ordi *resultant_points)
{


 double t1;
 int no_pts=0;
 double t;
 for(t=0;t<=1;t+=sampling_rate)
 {
 t1=t;

 resultant_points[no_pts].x=st_point.x*(t1-1)*(t1-1)*(2*t1+1)+end_point.x*t1*t1*(3-2*t1)+st_vel.x*t1*(t1-1)*(t1-1)-end_vel.x*t1*t1*(1-t1);
 resultant_points[no_pts].y=st_point.y*(t1-1)*(t1-1)*(2*t1+1)+end_point.y*t1*t1*(3-2*t1)+st_vel.y*t1*(t1-1)*(t1-1)-end_vel.y*t1*t1*(1-t1);
 resultant_points[no_pts].z=st_point.z*(t1-1)*(t1-1)*(2*t1+1)+end_point.z*t1*t1*(3-2*t1)+st_vel.z*t1*(t1-1)*(t1-1)-end_vel.z*t1*t1*(1-t1);
 no_pts++;
 }
return no_pts;
/* for( i=0;i<=2;i+=.005)
 {
 float x=(1-i)*(1-i)*(1-i)*x1+3*(1-i)*(1-i)*i*x2+3*(1-i)*i*i*x3+i*i*i*x4;
 float y=(1-i)*(1-i)*(1-i)*y1+3*(1-i)*(1-i)*i*y2+3*(1-i)*i*i*y3+i*i*i*y4;
 putpixel(x,y,GREEN);
 delay(10);
 }  */
/*
 for(i=0;i<=1;i+=.005)
 {
 float x=(1-i)*(1-i)*(1-i)*x1+8*(1-i)*(1-i)*i*x2+3*(1-i)*i*i*x3+i*i*i*x4;
 float y=(1-i)*(1-i)*(1-i)*y1+8*(1-i)*(1-i)*i*y2+8*(1-i)*i*i*y3+i*i*i*y4;
 putpixel(x,y,GREEN);
 delay(20);
 }
*/

}

int get_cubic_spline_by_Hermite_polynomial(point_co_ordi point[100], int n, point_co_ordi init_vel, point_co_ordi final_vel, int continuity_constraint_type, double sampling_period, double total_time, point_co_ordi *resultant_spline)
{
 FILE *vel_profile=fopen("/home/akpandey/velocity_profile.txt","w");
 //float
 printf(" Inside get_cubic_spline_by_Hermite_polynomial() with n=%d \n",n);
 
 point_co_ordi tmp_resultant_spline[500];
 
 point_co_ordi st_point, end_point;
 point_co_ordi st_vel, end_vel;
 st_vel.x=init_vel.x;
 st_vel.y=init_vel.y;
 st_vel.z=init_vel.z;
 
 double speed=sqrt((st_vel.x*st_vel.x)+(st_vel.y*st_vel.y)+(st_vel.z*st_vel.z));
 fprintf(vel_profile, "st_vel_x=%lf\nst_vel_y=%lf\nst_vel_z=%lf\nspeed=%lf\n", st_vel.x, st_vel.y, st_vel.z, speed);
 
 
 if(continuity_constraint_type==2) // means the second derivative i.e. the accelaration should also be continuous everywhere
 {
 end_vel.x=3.0/4.0*(point[2].x-point[0].x);
 end_vel.y=3.0/4.0*(point[2].y-point[0].y);
 end_vel.z=3.0/4.0*(point[2].z-point[0].z);
 }
 else
 {
 if(continuity_constraint_type==1) // means the first derivative i.e. the velocity should be continuous everywhere
  {
  end_vel.x=0.5*(point[2].x-point[0].x);
  end_vel.y=0.5*(point[2].y-point[0].y);
  end_vel.z=0.5*(point[2].z-point[0].z);
  }
 }
 //end_vel.x=0.0;
 //end_vel.y=0.0;

 //double t_scaling=0.5;

int total_no_of_curves=n-1;//total no of curves will be 1 less than the total no of points

////double time_for_single_segment=total_time/total_no_of_curves;
////printf(" time_for_single_segment=%lf\n",time_for_single_segment);

double sampling_interval=sampling_period;
printf(" sampling_interval for cubic spline = %lf \n", sampling_interval); 
 //float sampling_interval=0.1;
 int i=0;
 int total_no_pts=no_spline_points;//This counter will set to be the already stored points in the resultant_spline
 for(i=0;i<n-1;i++)
 {
 st_point.x=point[i].x;
 st_point.y=point[i].y;
 st_point.z=point[i].z;
 end_point.x=point[i+1].x;
 end_point.y=point[i+1].y;
 end_point.z=point[i+1].z;

  //co_ordi tmp_st_vel, tmp_end_vel;
 //draw_Hermite_polynomial(st_point, end_point, st_vel, end_vel, t_scaling, GREEN);
 printf(" For segment %d, Calling get_Hermite_polynomial_points() \n", i);

  if(i==0)
  sampling_interval=sampling_period/2.0;
  else
  sampling_interval=sampling_period;

  int no_pts=get_Hermite_polynomial_points(st_point, end_point, st_vel, end_vel, sampling_interval, tmp_resultant_spline);
  printf(" Got no_pts=%d for segment %d \n",no_pts, i);
 int j=0; 
 for(;j<no_pts;j++)
  {
  resultant_spline[total_no_pts].x=tmp_resultant_spline[j].x; 
  resultant_spline[total_no_pts].y=tmp_resultant_spline[j].y;
  resultant_spline[total_no_pts].z=tmp_resultant_spline[j].z;
  total_no_pts++;
  } 
 
 
 st_vel.x=end_vel.x;
 st_vel.y=end_vel.y;
 st_vel.z=end_vel.z;

 double speed=sqrt((st_vel.x*st_vel.x)+(st_vel.y*st_vel.y)+(st_vel.z*st_vel.z));
  fprintf(vel_profile, "st_vel_x=%lf\nst_vel_y=%lf\nst_vel_z=%lf\nspeed=%lf\n", st_vel.x, st_vel.y, st_vel.z, speed);
  
 if(i==n-3)
 {
 end_vel.x=final_vel.x;
 end_vel.y=final_vel.y;
 end_vel.z=final_vel.z;
 }
 else
 {
 
 if(continuity_constraint_type==2) // means the second derivative i.e. the accelaration should also be continuous everywhere
 {
 end_vel.x=3.0/4.0*(point[i+3].x-point[i+1].x);
 end_vel.y=3.0/4.0*(point[i+3].y-point[i+1].y);
 end_vel.z=3.0/4.0*(point[i+3].z-point[i+1].z);
 }
 else
 {
 if(continuity_constraint_type==1) // means the first derivative i.e. the velocity should be continuous everywhere
  {
  end_vel.x=0.5*(point[i+3].x-point[i+1].x);
  end_vel.y=0.5*(point[i+3].y-point[i+1].y);
  end_vel.z=0.5*(point[i+3].z-point[i+1].z);
  }
 } 
 // end_vel.x=0.0;
 //end_vel.y=0.0;

 }

 }
 
 fclose(vel_profile);
 printf(" Returning total_no_pts = %d, from get_cubic_spline_by_Hermite_polynomial() \n",total_no_pts); 
return total_no_pts;

}
/////////////// END AKP: Cut from Mightability_Maps.c
