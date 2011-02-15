#ifndef P3D_SOFTMOTION_TRAJ_PROTO_H
#define P3D_SOFTMOTION_TRAJ_PROTO_H

#include "../lightPlanner/proto/ManipulationStruct.h"
#include "Localpath-pkg.h"

#include <utility>
typedef std::pair<double,double> middleOfCVS;

int p3d_multilocalpath_switch_to_linear_groups (p3d_rob * robotPt);

p3d_traj* p3d_get_last_linear_traj();

extern int p3d_optim_traj_softMotion(p3d_traj *trajPt, bool param_write_file, double *gain, int *ntest, std::vector <int> &lp, std::vector < std::vector <double> > &positions, SM_TRAJ &smTraj);
extern int p3d_convert_traj_to_softMotion(p3d_traj *trajPt, bool param_write_file, std::vector <int> &lp, std::vector < std::vector <double> > &positions, SM_TRAJ &smTraj);

extern void draw_trajectory_ptp();
#endif
