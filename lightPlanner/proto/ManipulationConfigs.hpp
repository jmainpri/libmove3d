/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
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
  
    void setDebugMode(bool value) const;
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
