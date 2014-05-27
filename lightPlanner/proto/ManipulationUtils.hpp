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
#ifndef __MANIPULATIONUTILS_HPP__
#define __MANIPULATIONUTILS_HPP__

#include "ManipulationStruct.hpp"
#ifdef GRASP_PLANNING
  #include "GraspPlanning-pkg.h"
#endif
#include "P3d-pkg.h"
#include <vector>

/** @defgroup manipulation
* The manipulation classes are dedicated to
* the planning of manipulation tasks.
* They encapsulates variables and functions that allow to easily (!) plan the motion
* required for basic manipulation tasks (pick-and-place, object transfer from one hand to the other).
 */

//! @ingroup manipulation
/** Different general utils for manipulation*/
class  ManipulationUtils 
{
  public:
    /*Constructors and Destructors*/

    ManipulationUtils(){};
    virtual ~ManipulationUtils(){};


    /*############# Static Functions ############*/

    /* Message gestion */
    static void undefinedRobotMessage();
    static void undefinedObjectMessage();
    static void undefinedSupportMessage();
    static void undefinedCameraMessage();
    static void printManipulationMessage(MANIPULATION_TASK_MESSAGE message);
    static int printConstraintInfo(p3d_rob* robot);
    
    /* UI gestion */
    //! Forbids all the interaction (keyboard and mouse) with the current window.
    //! \return 0 in case of success, 1 otherwise
    static int forbidWindowEvents();
    
    //! Allows the interaction (keyboard and mouse) with the current window.
    //! \return 0 in case of success, 1 otherwise
    static int allowWindowEvents();

    //! Copy the given configuration to the robot XForm window
    //! \return 0 in case of success, 1 otherwise
    static int copyConfigToFORM(p3d_rob* robot, configPt q);

    //! Check if the given vector is valid or not
    //! \return true if the vector is valid false otherwise
    static bool isValidVector(std::vector<double> ObjectPos);

    /** Fix the sampling of all the robots hands, desactivate hands self collisions and set the to rest configuration */
    static void fixAllHands(p3d_rob* robot, configPt q, bool rest);
    /** UnFix the sampling of all the robots hands, activate hands self collision */
    static void unFixAllHands(p3d_rob* robot);
    /** Fix the free flyer on the object pos. TODO: This function have to be changed to deal with cartesian mode (fix on the arm not on the object)*/
    static void fixManipulationJoints(p3d_rob* robot, int armId, configPt q, p3d_rob* object);
    /** UnFix the free flyers*/
    static void unfixManipulationJoints(p3d_rob* robot, int armId);
};
#endif
