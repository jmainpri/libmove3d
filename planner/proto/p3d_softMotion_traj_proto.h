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
#ifndef P3D_SOFTMOTION_TRAJ_PROTO_H
#define P3D_SOFTMOTION_TRAJ_PROTO_H

#include "../lightPlanner/proto/ManipulationStruct.hpp"
#include "Localpath-pkg.h"

#include <utility>

typedef struct midCVSParams {
  double s;
  double tau;
} midCVSParams;

typedef std::pair< int, midCVSParams > middleOfCVS;

int p3d_multilocalpath_switch_to_linear_groups (p3d_rob * robotPt);

p3d_traj* p3d_get_last_linear_traj();

extern int p3d_optim_traj_softMotion(p3d_traj *trajPt, bool param_write_file, double *gain, int *ntest, std::vector <int> &lp, std::vector < std::vector <double> > &positions, SM_TRAJ &smTraj);
extern int p3d_convert_traj_to_softMotion(p3d_traj *trajPt, bool smooth, bool param_write_file, bool approximate, std::vector <int> &lp, std::vector < std::vector <double> > &positions, SM_TRAJ &smTraj);

extern void draw_trajectory_ptp();

extern int p3d_getQSwitchIDFromMidCVS(double tau, double t_rep, int* id);
extern bool p3d_getMidCVSTimeOnTraj(int id, double& time);

#endif
