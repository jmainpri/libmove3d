#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
//#include "hrp2_gik.h"




#define M3_IJ MAL_S3x3_MATRIX_ACCESS_I_J
#define M4_IJ MAL_S4x4_MATRIX_ACCESS_I_J
#define V3_I  MAL_S3_VECTOR_ACCESS

double M3D_to_HRP2_GIK_sitting_Z_shift=0.475;//+0.16;//0.510+0.16;//0.622+0.16;//0.510-0.112=0.398//This is the value which needs to be added to the M3D z value of robot to synchronize with the z value of HRP2_GIK because for M3D the z is the height of foot whereas for HRP2_GIK z is the height of the waist. 
double M3D_to_HRP2_GIK_half_sitting_Z_shift=0.6487;
////double M3D_to_HRP2_GIK_half_sitting_Z_shift=0.8487;
int HRP2_CURRENT_STATE=1;//1 for sitting, 2 for half sitting

struct gik_solution curr_gik_sol;

struct SOLUTION_CONFIGS_FOR_HRP2 cur_gik_sol_configs;

extern hri_bitmapset * ACBTSET;

extern int SHOW_HRP2_GIK_SOL;

extern int SKIP_FIRST_CONFIG;

static configPt RS_robotq = NULL;    

int M3D_GIK_TEST();

p3d_vector3 robot_hand_reach_goal[1000];
int cur_i;
p3d_vector3 to_reach_target;

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

int get_index_of_robot_by_name(char *rob_name)
{
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr,i;
 
  nr = envPt->nr;
	
  for(i=0;i<nr;i++)
	{
		
		if (strcmp(envPt->robot[i]->name,rob_name)==0)
		{
			return i;
		}
	}
    return NULL;
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
