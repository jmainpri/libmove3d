#ifndef __MANIPULATIONCONFIGS_HPP__
#define __MANIPULATIONCONFIGS_HPP__

#if defined (LIGHT_PLANNER) && defined (GRASP_PLANNING)

#include "P3d-pkg.h"
#include "GraspPlanning-pkg.h"

#include "ManipulationStruct.hpp"
#include "ManipulationUtils.hpp"
#include "ManipulationData.hpp"

#include <vector>

configPt manipulation_get_free_holding_config();

//! @ingroup manipulation
//! Free Holding Config Data
class ManipIKConfigData
{
public: 
  p3d_rob* object;
  int armId;
  gpGrasp grasp;
  p3d_matrix4 tAtt;
  double confCost;
  std::vector<double> objGoto;
  p3d_rob* support;
};

//! @ingroup manipulation
class  ManipulationConfigs 
{
  public:
    ManipulationConfigs(p3d_rob * robot);
    virtual ~ManipulationConfigs();
  
    void setDebugMode(bool value);
    void setMobileBaseMode(bool value);
  
    void setOptimizeRedundentSteps(int nbSteps);
    int getOptimizeRedundentSteps(void) const;

    void setApproachFreeOffset(double offset);
    double getApproachFreeOffset(void) const;

    void setApproachGraspOffset(double offset);
    double getApproachGraspOffset(void) const;
  
    void setSafetyDistanceValue(double value);
    double getSafetyDistanceValue(void) const ;

    inline p3d_rob* robot()  const{return _robot;}
  
    // Primitive generating configurations
    configPt getGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost) const;
    configPt getOpenGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf) const;
    configPt getApproachFreeConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const;
    configPt getApproachGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const;
  configPt getFreeHoldingConf() const;
    configPt getFreeHoldingConf( p3d_rob* obj, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost, std::vector<double> &objGoto, p3d_rob* support = NULL ) const;
    configPt getExtractConf(int armId, configPt currentConf, p3d_matrix4 tAtt) const;
  
    // Compound of the function above
    MANIPULATION_TASK_MESSAGE getGraspOpenApproachExtractConfs(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, ManipulationData& configs) const;
    MANIPULATION_TASK_MESSAGE getHoldingOpenApproachExtractConfs(p3d_rob* object, std::vector<double> &objGoto, p3d_rob* placement, int armId, gpGrasp& grasp, p3d_matrix4 tAtt,  ManipulationData& configs) const;
    MANIPULATION_TASK_MESSAGE findArmGraspsConfigs(int armId, p3d_rob* object, gpGrasp& grasp, ManipulationData& configs) const;
    
  private:
    /*!< pointer to the robot */
    p3d_rob * _robot;
    /** uses the base degrees of freedom for generatiing configuration **/
    bool _useMobileBase;
    /** Number of steps when optimizing the redundent joint*/
    int _optimizeRedundentSteps;
    /** Offset to generate the approach configuration of a grasp (not carrying an object)*/
    double _approachFreeOffset;
    /** Offset to generate the approach configuration of a grasp (carrying an object)*/
    double _approachGraspOffset;
    double _safetyDistanceValue;
    /** Manipulation Data IK **/
    ManipIKConfigData* _IKData;
};

#endif
#endif
