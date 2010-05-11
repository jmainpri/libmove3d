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
 public :
     Manipulation_JIDO(p3d_rob *robot);
     virtual ~Manipulation_JIDO();
     void clear();

     /*Functions relative to JIDO */
     int setArmQ(double q1, double q2, double q3, double q4, double q5, double q6);
     int getArmQ(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     int setArmX(double x, double y, double z, double rx, double ry, double rz);
     int getArmX(double* x, double* y, double* z, double* rx, double* ry, double* rz);
     void setArmCartesian(bool v);
     bool getArmCartesian();
     int setArmTask(MANIPULATION_TASK_TYPE_STR t);
     int setObjectToManipulate();
     int printConstraintInfo();
     int setPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz, configPt q);

//      int armPlanGoto(int lp[], Gb_q6 positions[],  int *nbPositions);
     int armPlanTask(MANIPULATION_TASK_TYPE_STR task,char* objectName, int lp[], Gb_q6 positions[],  int *nbPositions);
     
     int armComputePRM();
     
     int cleanRoadmap();
     int cleanTraj();

     int grabObject(configPt qGrab, char* objectName);
     int releaseObject();
     
     /* Function relative to other robots */
     int setFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
     int getFreeflyerPose(char *name, p3d_matrix4 pose);
     int setFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz);
     p3d_obj * getObjectByName(char *object_name);
     int setObjectPoseWrtEndEffector(p3d_rob *object, double x, double y, double z, double rx, double ry, double rz);

     /* Function relative to obstacles */
     int setObjectPose(char *object_name, double x, double y, double z, double rx, double ry, double rz);

  protected:
     /*Functions relative to JIDO */
     int computeTrajBetweenTwoConfigs(bool cartesian, configPt qi, configPt qf);
     int computeGraspList(p3d_rob *hand_robotPt, char *object_name);
     int findPregraspAndGraspConfiguration(p3d_rob *hand_robotPt, char *object_name, double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
     int computeRRT();
     int computeOptimTraj();

  private :
     p3d_rob * _robotPt;
     configPt _configStart;
     configPt _configGoto;

     double _QCUR[6];
     double _QGOAL[6];
     double _XCUR[6];
     double _XGOAL[6];

     p3d_rob *_object;
     gpHand_properties _HAND;  // information about the used hand

     std::list<gpGrasp> _GRASPLIST;
     gpGrasp _GRASP;   // the current grasp
     std::list<gpPose> _POSELIST;
     p3d_matrix4 _EEFRAME, _GFRAME;
     bool _capture;
     bool _cartesian;
     bool _objectGrabed;
};

#endif
