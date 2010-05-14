#ifndef __MANIPULATION_H__
#define __MANIPULATION_H__
#include <vector>
#include <map>
#include "GraspPlanning-pkg.h"

#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include <list>

typedef enum MANIPULATION_TASK_TYPE_STR {
  ARM_FREE,
  ARM_PICK_GOTO,
  ARM_PICK_TAKE_TO_FREE,
  ARM_PICK_TAKE_TO_PLACE,
  ARM_PLACE_FROM_FREE
} MANIPULATION_TASK_TYPE_STR;

// typedef enum MANIPULATION_TASK_CNTRT_ENUM {
// 
// 
// } MANIPULATION_TASK_CNTRT_ENUM;

class ManipulationData{
  public:
    ManipulationData(p3d_rob* robot){
      _robot = robot;
      _graspConfig = p3d_alloc_config(robot);
      _openConfig = p3d_alloc_config(robot);
      _approachConfig = p3d_alloc_config(robot);
      p3d_mat4Copy(p3d_mat4IDENTITY ,_graspAttachFrame);
    };
    ManipulationData(p3d_rob* robot, gpGrasp* grasp, configPt graspConfig, configPt openConfig, configPt approachConfig, p3d_matrix4 graspAttachFrame){
      _robot = robot;
      _grasp = grasp;
      _graspConfig = graspConfig;
      _openConfig = openConfig;
      _approachConfig = approachConfig;
      p3d_mat4Copy(graspAttachFrame ,_graspAttachFrame);
    };
    virtual ~ManipulationData(){
      if(_graspConfig){
        p3d_destroy_config(_robot, _graspConfig);
      }
      if(_openConfig){
        p3d_destroy_config(_robot, _openConfig);
      }
      if(_approachConfig){
        p3d_destroy_config(_robot, _approachConfig);
      }
    }
    //Getters
    inline gpGrasp* getGrasp(){
      return _grasp;
    }
    inline configPt getGraspConfig(){
      return _graspConfig;
    }
    inline configPt getOpenConfig(){
      return _openConfig;
    }
    inline configPt getApproachConfig(){
      return _approachConfig;
    }
    inline void getAttachFrame(p3d_matrix4 graspAttachFrame){
      p3d_mat4Copy(_graspAttachFrame, graspAttachFrame);
    }
    inline double getGraspConfigCost(){
      return _graspConfigCost;
    }
    //Setters
    inline void setGrasp(gpGrasp* grasp){
      _grasp = grasp;
    }
    inline void setGraspConfig(configPt graspConfig){
      _graspConfig = graspConfig;
    }
    inline void setOpenConfig(configPt openConfig){
      _openConfig = openConfig;
    }
    inline void setApproachConfig(configPt approachConfig){
      _approachConfig = approachConfig;
    }
    inline void setAttachFrame(p3d_matrix4 graspAttachFrame){
      p3d_mat4Copy(graspAttachFrame, _graspAttachFrame);
    }
    inline void setGraspConfigCost(double graspConfigCost){
      _graspConfigCost = graspConfigCost;
    }
  private:
    p3d_rob* _robot;
    gpGrasp* _grasp;
    configPt _graspConfig;
    configPt _openConfig;
    configPt _approachConfig;
    double _graspConfigCost;
    p3d_matrix4 _graspAttachFrame;
};


class DoubleGraspData{
  public:
  DoubleGraspData(p3d_rob* robot){
    _robot = robot;
    _config = p3d_alloc_config(robot);
  }
  DoubleGraspData(p3d_rob* robot, gpDoubleGrasp dGrasp, configPt cCConfig){
    _robot = robot;
    _doubleGrasp = dGrasp;
    _config = cCConfig;
  }
  virtual ~DoubleGraspData(){
    p3d_destroy_config(_robot, _config);
  }
  inline gpDoubleGrasp getDoubleGrasp(){
    return _doubleGrasp;
  }
  inline void setDoubleGrasp(gpDoubleGrasp doubleGrasp){
    _doubleGrasp = doubleGrasp;
  }
  inline configPt getConfig(){
    return _config;
  }
  inline void setConfig(configPt cCConfig){
    _config = cCConfig;
  }
  inline void setObjectExchangeMat(p3d_matrix4 objectExchangeMat){
    p3d_mat4Copy(objectExchangeMat, _objectExchangeMat);
  }
  inline void getObjectExchangeMat(p3d_matrix4 objectExchangeMat){
    p3d_mat4Copy(_objectExchangeMat, objectExchangeMat);
  }
  private:
  p3d_rob* _robot;
  gpDoubleGrasp _doubleGrasp;
  p3d_matrix4 _objectExchangeMat;
  configPt _config;
};


class Manipulation{
  public :
    Manipulation(p3d_rob *robot);
    virtual ~Manipulation();
    void clear();
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
    double getRobotGraspArmCost(gpGrasp grasp, configPt q);
    int getCollisionFreeDoubleGraspAndApproach(p3d_matrix4 objectPos, std::vector<gpHand_properties> armsProp, gpDoubleGrasp doubleGrasp, configPt* doubleGraspConfig);
    std::vector<gpHand_properties> InitHandProp(int armId);
    std::list<gpGrasp>* getGraspListFromMap(int armId);
    int checkTraj(p3d_traj * traj, p3d_graph* graph);
  
  private :
    std::map < int, std::map<int, ManipulationData*, std::less<int> >, std::less<int> > _handsGraspsConfig;
    std::list<DoubleGraspData*> _handsDoubleGraspsConfigs;
    p3d_rob * _robot;
    p3d_graph * _offlineGraph;
    static const int _maxColGrasps = 10;
    p3d_matrix4 _exchangeMat;
    std::vector<std::vector<double> > _statDatas;
};

class  Manipulation_JIDO {
  private :
     p3d_rob * _robotPt; /*!< pointer to the robot (Jido) */
     p3d_rob * _hand_robotPt;  /*!< pointer to the hand robot (a freeflyer robot with the same structure as the robot's hand) */
     //gpHand_type _handType;
     gpHand_properties _handProp;  /*!< information about the used hand */
     std::list<gpGrasp> _graspList; 
     gpGrasp _grasp;   /*!< the current grasp */
     
     configPt _configStart;
     configPt _configGoto;

  
     double _liftUpDistance;  /*!< the distance the object is lifted up after it is grasped, before any other movement */

     //! for stable pose computation, the space of possible poses on the support is sampled with the following steps:
     double _poseTranslationStep; /*!< the translation step of the discretization of the horizontal faces of the support */
     double _poseNbOrientations; /*!<  the number of orientations (around vertical axis) that will be tested to place the object on the support*/

     double _QCUR[6];
     double _QGOAL[6];
     double _XCUR[6];
     double _XGOAL[6];
     double _qrest[6];  /*!< rest configuration of the arm */

     p3d_rob *_object; /*!< the object to work with (a freeflyer robot) */
     p3d_rob *_support; /*!< the support to possibly place the object on (a freeflyer robot) */

     std::list<gpPose> _poseList; /*!< stable poses of the object (context independent) */
     std::list<gpPose> _poseOnSupportList; /*!< stable poses of the object on the current support (context dependent) */
     p3d_matrix4 _EEFRAME, _GFRAME;
     bool _capture;
     bool _cartesian; /*!< choose to plan the arm motion in cartesian space (for the end effector) or joint space  */
     bool _objectGrabed;

 public :
     Manipulation_JIDO(p3d_rob * robotPt, gpHand_type handType);
     virtual ~Manipulation_JIDO();
     void clear();

     void setCapture(bool v);
     bool getCapture();
     /*Functions relative to JIDO */
     int setArmQ(double q1, double q2, double q3, double q4, double q5, double q6);
     int getArmQ(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     int setArmX(double x, double y, double z, double rx, double ry, double rz);
     int getArmX(double* x, double* y, double* z, double* rx, double* ry, double* rz);
     void setArmCartesian(bool v);
     bool getArmCartesian();
     int setArmTask(MANIPULATION_TASK_TYPE_STR t);
     int setObjectToManipulate(char *objectName);
     int setSupport(char *supportName);
     int printConstraintInfo();
     int setPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz, configPt q);
     int dynamicGrasping(char *robot_name, char *hand_robot_name, char *object_name);
     int robotBaseGraspConfig(char *objectName, double *x, double *y, double *theta);
//      int armPlanGoto(int lp[], Gb_q6 positions[],  int *nbPositions);
     int armPlanTask(MANIPULATION_TASK_TYPE_STR task,char* objectName, int lp[], Gb_q6 positions[],  int *nbPositions);
     
     int armComputePRM();
     
     int cleanRoadmap();
     int cleanTraj();

     int grabObject(char* objectName);
     int releaseObject();

     void draw();
 
     int setLiftUpDistance(double dist) {
       _liftUpDistance= fabs(dist); 
       return 0;
     }
     double liftUpDistance() {
       return _liftUpDistance;
     }
     
     /* Functions relative to other robots */
     int setFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
     int getObjectPose(p3d_matrix4 pose);
     int getObjectPose(double *x, double *y, double *z, double *rx, double *ry, double *rz);
     int setFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz);
     p3d_obj * getObjectByName(char *object_name);
     int setObjectPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz);

     /* Functions relative to obstacles */
     int setObjectPose(char *object_name, double x, double y, double z, double rx, double ry, double rz);


     /* Functions relative to object grasping */
    int findPregraspAndGraspConfiguration(double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
    bool isObjectGraspable(char *objectName);

     /* Functions relative to object placement */
    int findPlacementConfigurations(double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);

  protected:
     /*Functions relative to JIDO */
     int computeTrajBetweenTwoConfigs(bool cartesian, configPt qi, configPt qf);
     int computeGraspList();
     int findSimpleGraspConfiguration(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     int computePoseList();
     

     int computeRRT();
     int computeOptimTraj();
};

int getFreeflyerPose(char *name, p3d_matrix4 pose);

#endif
