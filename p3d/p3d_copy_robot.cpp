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
/**
  * To have a stable muti-treaded system, objects need a clear life cycle definition,
  * this file should contain methods that permit copying and deleting the p3d_rob strucure
  * and its dependant structures such as matrixes and joints
  */

#include "P3d-pkg.h"

#include <iostream>

using namespace std;

/**
  * This function creates a Joint structure
  */
p3d_jnt* copyJointStructure(p3d_jnt* jntPt)
{
    return jntPt;
}

/**
 * This functions copies all fields
 * with the dependent arrays
 */
p3d_rob* copyRobStructure(p3d_rob* robotPt)
{
    cout << " New Robot "  << endl;

    p3d_rob* newRobPt = new p3d_rob(*robotPt);

    //----------- Basics --------------------------
    newRobPt->name = new char[256];
    strcpy(newRobPt->name,robotPt->name);

    //----------- Bodies --------------------------
    newRobPt->o = new pp3d_obj[newRobPt->no];

    for(int i=0;i<newRobPt->no;i++)
    {
        newRobPt->o[i] = robotPt->o[i];
    }

    //----------- Joints --------------------------
    newRobPt->joints = new pp3d_jnt[newRobPt->njoints+1];
    for(int i=0;i<=newRobPt->njoints;i++)
    {
        newRobPt->joints[i] = copyJointStructure(robotPt->joints[i]);
    }

    if(robotPt->first_abs_pos)
    {
        newRobPt->first_abs_pos = new p3d_matrix4[1];
        p3d_mat4Copy( *(robotPt->first_abs_pos), *(newRobPt->first_abs_pos) );
    }


    //----------- Trajectories --------------------
    newRobPt->t = new pp3d_traj[newRobPt->nt];

    for(int i=0;i<newRobPt->nt;i++)
    {
        newRobPt->t[i] = robotPt->t[i];
    }

    //----------- BB ------------------------------
    for(int i=0;i<NDOF_BASE_ROTATE;i++)
    {
        newRobPt->vmin_rot[i] = robotPt->vmin_rot[i];
    }

    for(int i=0;i<NDOF_BASE_ROTATE;i++)
    {
        newRobPt->vmax_rot[i] = robotPt->vmax_rot[i];
    }


    //----------- Configuration -------------------
    newRobPt->ROBOT_POS = p3d_copy_config(robotPt,robotPt->ROBOT_POS);
    newRobPt->ROBOT_GOTO = p3d_copy_config(robotPt,robotPt->ROBOT_GOTO);

    //----------- Transitions ---------------------
    for(int i=0;i<newRobPt->nTransition;i++)
    {
        newRobPt->transitionConfigs[i] = p3d_copy_config(robotPt,robotPt->transitionConfigs[i]);
    }

    return newRobPt;
}

/**
  * This function deletes the values
  * with the dependent
  */
void deleteRobStructure(p3d_rob* robotPt)
{
    delete[] robotPt->name;
    delete[] robotPt->o;

//    for(int i=0;i<=robotPt->njoints;i++)
//    {
//        p3d_jnt_destroy(robotPt->joints[i]);
//    }

    delete[] robotPt->joints;

    if(robotPt->first_abs_pos)
    {
        delete robotPt->first_abs_pos;
    }
    delete[] robotPt->t;
    delete[] robotPt->ROBOT_POS;
    delete[] robotPt->ROBOT_GOTO;

//    cout << "Delete Robot"  << endl;

    delete robotPt;
}
