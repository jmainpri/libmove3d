#ifndef __MANIPULATIONPLANNER_HPP__
#define __MANIPULATIONPLANNER_HPP__


#include "ManipulationStruct.h"
//! @ingroup manipulation
class  ManipulationPlanner {
  private :
    /**************/
    /* The robots */
    /**************/
    /*!< pointer to the robot */
    p3d_rob * _robot;

    /*!< pointer to the hand robot (a freeflyer robot with the same structure as the robot's hand) */
    std::vector<p3d_rob *> _hand_robotPt;

    /******************************************/
    /* The attributes of the manipulator robot */
    /*******************************************/
    int _BaseMLP; /*!< Multilocalpath Id for the base*/
    int _HeadMLP; /*!< Multilocalpath Id for the head*/
    int _UpBodyMLP; /*!< Multilocalpath Id for the upper body (arms + torso) with linear localplanner*/
    int _UpBodySmMLP; /*!< Multilocalpath Id for the upper body (arms + torso) with softMotion localplanner*/
    //     int _ObjectMLP; /*!< Multilocalpath Id for the virtual object with linear localplanner*/
    //     int _ObjectSmMLP; /*!< Multilocalpath Id for the virtual object with softMotion localplanner*/
    //     int _nbArms;

    /********************************/
    /* The attributes of the grasp  */
    /********************************/
    //     std::vector<gpHand_properties>  _handProp;  /*!< information about the used hand */
    //     std::vector<std::list<gpGrasp> > _graspList;
    //     std::vector<gpGrasp>            _grasp;   /*!< the current grasp */
    //     std::vector<unsigned int>       _graspID; /*!< the current grasp ID */
    int _nbGraspsToTestForPickGoto; /*!< the  _nbGraspsToTestForPickGoto first grasps of the grasp list will be tested for the ARM_PICK_GOTO task planning */

    std::map < int, std::map<int, ManipulationData*, std::less<int> >, std::less<int> > _handsGraspsConfig;
    std::list<DoubleGraspData*> _handsDoubleGraspsConfigs;

    p3d_graph * _offlineGraph;
    static const int _maxColGrasps = 10;
    p3d_matrix4 _exchangeMat;
    std::vector<std::vector<double> > _statDatas;
    /** Number of steps for the optimisation*/
    int _optimizeSteps;
    /** Time limit for the optimisation*/
    double _optimizeTime;
    /********************************/
    /* The attributes of the motion */
    /********************************/
    //      configPt _configRest;
    //      configPt _configStart;
    //      configPt _configGoto;
    //      configPt _configApproach;

    std::vector<configPt> _configTraj; /*!< this array stores the key configurations that will be used to compute a sequence of trajectory*/
    //     bool _cartesian; /*!< choose to plan the arm motion in cartesian space (for the end effector) or joint space  */
    double _liftUpDistance;  /*!< the distance the object is lifted up after it is grasped, before any other movement */

    /***********************************/
    /* The attributes of the placement */
    /***********************************/
    gpPlacement _placement;   /*!< the current or target object placement */
    std::list<gpPlacement> _placementList; /*!< stable placements of the object (context independent) */
    std::list<gpPlacement> _placementOnSupportList; /*!< stable placements of the object on the current support (context dependent) */
    //! for stable placement computation, the space of possible poses on the support is sampled with the following steps:
    double _placementTranslationStep; /*!< the translation step of the discretization of the horizontal faces of the support */
    double _placementNbOrientations; /*!<  the number of orientations (around vertical axis) that will be tested to place the object on the support*/
    double _placementOffset; /*!< the arm configuration used for placement object will be slightly vertically shifted in order to be sure that the object will contact the support in real life (set the value with caution)*/

    /***********************************/
    /* The attributes of the camera    */
    /***********************************/
    p3d_jnt *_cameraJnt; /*!< the robot's joint that gives the pose of the pan/tilt camera */
    double _cameraFOV; /*!< robot's camera field of view angle (IN DEGREES) used for grasp visibility score computation */
    int _cameraImageWidth, _cameraImageHeight; /*!< dimensions of the synthetic images of what the robot views that will be used to compute the visibility score. Small values gives faster computations. */
    bool _capture;

  public:

     ManipulationPlanner(p3d_rob * robotPt);
     virtual ~ManipulationPlanner();

     void clear();

     int cleanRoadmap();
     int cleanTraj();

     int setObjectToManipulate(char *objectName);
     int setSupport(char *supportName);
     int setHuman(char *humanName);

     /**********/
     /* Motion */
     /**********/
     MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId,
             configPt qStart, configPt qGoal,
             char* objectName, std::vector <int> &lp,
             std::vector < std::vector <double> > &positions,
             MANPIPULATION_TRAJECTORY_STR &segments);
	     
     int computeSoftMotion(p3d_traj* traj, std::vector <int> &lp, std::vector < std::vector <double> > &positions, MANPIPULATION_TRAJECTORY_STR &segments);

     std::vector < std::vector <double> > positions;
     MANPIPULATION_TRAJECTORY_STR segments;
     std::vector <int> lp;

     int setLiftUpDistance(double dist) {
       _liftUpDistance= fabs(dist);
       return 0;
     }
     double liftUpDistance() {
       return _liftUpDistance;
     }

     int setArmQ(int armId, std::vector<double> q);
     int getArmQ(int armId, std::vector<double> &q);
     int setArmX(int armId, double x, double y, double z, double rx, double ry, double rz);
     int setArmX(int armId, double x, double y, double z, unsigned int nbTries= 30);
     int getArmX(int armId, double* x, double* y, double* z, double* rx, double* ry, double* rz);

          /*Functions relative to JIDO */
//      int setArmQ(double q1, double q2, double q3, double q4, double q5, double q6);
//      int getArmQ(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
//      int setArmX(double x, double y, double z, double rx, double ry, double rz);
//      int setArmX(double x, double y, double z, unsigned int nbTries= 30);
//      int getArmX(double* x, double* y, double* z, double* rx, double* ry, double* rz);

    void checkConfigForCartesianMode(configPt q);
     void setArmCartesian(int armId, bool cartesian);
     bool getArmCartesian(int armId);

     int setArmTask(MANIPULATION_TASK_TYPE_STR t);

     int armComputePRM(double computeTime);

     int grabObject(int armId, char* objectName);
     int releaseObject();

     int addConfigTraj(configPt config);
     int clearConfigTraj();
     int copyConfigTrajToFORM();
     int destroyTrajectories();
     int checkCollisionOnTraj();
     int checkCollisionOnTraj(int currentLpId);
     int replanCollidingTraj(int currentLpId, std::vector <int> &lp, std::vector < std::vector <double> > &positions, MANPIPULATION_TRAJECTORY_STR &segments);

     /*******************/
     /* grasp's members */
     /*******************/
    /** Fix the sampling of all the robots hands, desactivate hands self collisions and set the to rest configuration*/
     void fixAllHands(configPt q);
    /** UnFix the sampling of all the robots hands, activate hands self collision*/
     void unFixAllHands();

     bool displayGrasps; /*!< boolean to enable/disable the display of the grasps of the current grasp list */
     bool displayPlacements; /*!<  boolean to enable/disable the display of the placements of the current object pose list */
     int setNbGraspsToTestForPickGoto(int n);
     int reduceGraspList(int armId, int maxSize);
     int dynamicGrasping(int armId, char *robot_name, char *hand_robot_name, char *object_name);
     int robotBaseGraspConfig(int armId, char *objectName, double *x, double *y, double *theta);

     void draw(int armId);
     /* Functions relative to object grasping */
     int findPregraspAndGraspConfiguration(int armId, double distance, configPt* qPreGrasp, configPt* qGrasp);
     bool isObjectGraspable(int armId, char *objectName);


     /* Functions relative to object placement */
     int findPlacementConfigurations(int armId, double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     //     int findPlacementConfigurations(int armId, double distance, configPt qPrePlacement, configPt qPlacement);

     /******************/
     /* Camera Members */
     /******************/
     void setCapture(bool v);
     bool getCapture();
     int centerCamera();
     int setCameraJnt(char *cameraJntName);
     int setCameraFOV(double fov);
     int setCameraImageSize(int width, int height);

     /***************/
     /* The getters */
     /***************/
     p3d_rob* robot() {
      return _robot;
     }
     configPt robotStart();
     configPt robotGoto();
    int getOptimizeSteps(void);
    double getOptimizeTime(void);
    void setOptimizeSteps(int nbSteps);
    void setOptimizeTime(double time);


     /********/
     /* Util */
     /********/
     int printConstraintInfo();

     /**********/
     /* Object */
     /**********/
     /* Functions relative to other robots */
     int setPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz, configPt q);
     int setObjectPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz);
     int setFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
     int getObjectPose(p3d_matrix4 pose);
     int getObjectPose(double *x, double *y, double *z, double *rx, double *ry, double *rz);
     int setFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz);
     p3d_obj * getObjectByName(char *object_name);



  private:

    // Moky imported functions
    void computeOfflineRoadmap();
    p3d_traj* computeRegraspTask(configPt startConfig, configPt gotoConfig, std::string offlineFile);
    p3d_traj* computeRegraspTask(configPt startConfig, configPt gotoConfig, std::string offlineFile, int whichTest);

    int findAllArmsGraspsConfigs(p3d_matrix4 objectStartPos, p3d_matrix4 objectEndPos);
    int findAllSpecificArmGraspsConfigs(int armId, p3d_matrix4 objectPos);
    double getCollisionFreeGraspAndApproach(p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig);
    void computeExchangeMat(configPt startConfig, configPt gotoConfig);
    void computeDoubleGraspConfigList();
    inline void setExchangeMat(p3d_matrix4 exchangeMat){
      p3d_mat4Copy(exchangeMat, _exchangeMat);
    }
    inline void getExchangeMat(p3d_matrix4 exchangeMat){
      p3d_mat4Copy(_exchangeMat, exchangeMat);
    }

    void drawSimpleGraspConfigs();
    void drawDoubleGraspConfigs();
    void printStatDatas();



  protected:

     p3d_traj* computeTrajBetweenTwoConfigs(configPt qi, configPt qf);
     int computeGraspList(int armId);
     int findSimpleGraspConfiguration(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
//      int findSimpleGraspConfiguration(int armId, configPt* qGrasp);
     int computePlacementList();
     int markGraspAsTested(int armId, int id);
     int computeRRT(int smoothingSteps, double smootingTime, bool biDir);
     int computeOptimTraj();

     // Moky imported functions
    double getRobotGraspArmCost(gpGrasp grasp, configPt q);
    int getCollisionFreeDoubleGraspAndApproach(p3d_matrix4 objectPos, std::vector<gpHand_properties> armsProp, gpDoubleGrasp doubleGrasp, configPt* doubleGraspConfig);
    std::vector<gpHand_properties> InitHandProp(int armId);
    std::list<gpGrasp>* getGraspListFromMap(int armId);
    int checkTraj(p3d_traj * traj, p3d_graph* graph);

};

#endif