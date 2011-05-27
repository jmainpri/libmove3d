#ifndef __MANIPULATIONCONFIGS_HPP__
#define __MANIPULATIONCONFIGS_HPP__

#if defined (LIGHT_PLANNER) && defined (GRASP_PLANNING)

#include "P3d-pkg.h"
#include "GraspPlanning-pkg.h"

#include "ManipulationStruct.h"
#include "ManipulationUtils.hpp"

#include <vector>

//! @ingroup manipulation
class  ManipulationConfigs {
  public:
  /* ******************************* */
  /* ******* (Con)Destructor ******* */
  /* ******************************* */
    ManipulationConfigs(p3d_rob * robot);
    virtual ~ManipulationConfigs();
  /* ******************************* */
  /* ******* (Ge)Setters *********** */
  /* ******************************* */
    void setDebugMode(bool value);
  
    void setOptimizeRedundentSteps(int nbSteps);
    int getOptimizeRedundentSteps(void) const;

    void setApproachFreeOffset(double offset);
    double getApproachFreeOffset(void) const;

    void setApproachGraspOffset(double offset);
    double getApproachGraspOffset(void) const;

    inline p3d_rob* robot()  const{return _robot;}
  /* ******************************* */
  /* *********** Methods *********** */
  /* ******************************* */
    /** Generate the grasp configuration given the grasp the arm and the object.
    @return the attach matrix computed given the grasp and Tatt2 from the p3d file
    @return the configuration cost
    @return the grasp configuration */
    configPt getGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost) const;
    /** Generate the open configuration given the grasp configuration, the grasp, the arm and the object.*/
    configPt getOpenGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf) const;
    /** Generate the open approach configuration given the grasp configuration, the grasp, the arm, the attach matrix and the object.*/
    configPt getApproachFreeConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const;
    /** Generate the grasp approach configuration given the grasp configuration, the grasp, the arm, the attach matrix and the object.*/
    configPt getApproachGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const;
    /** Generates a free configuration from a worspace point and a grasp*/
    configPt getFreeHoldingConf( p3d_rob* obj, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost, std::vector<double> &objGoto, p3d_rob* support = NULL ) const;
    /** Generate the extract configuration by moving the arm over Z axis until we have a collision free or passing 5 * offset */
    configPt getExtractConf(int armId, configPt currentConf, p3d_matrix4 tAtt) const;
    
  private:
    /*!< pointer to the robot */
    p3d_rob * _robot;
    /** Number of steps when optimizing the redundent joint*/
    int _optimizeRedundentSteps;
    /** Offset to generate the approach configuration of a grasp (not carrying an object)*/
    double _approachFreeOffset;
    /** Offset to generate the approach configuration of a grasp (carrying an object)*/
    double _approachGraspOffset;
};

#endif
#endif
