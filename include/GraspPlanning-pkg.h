/*
 *  This file includes the prototypes of the functions defined in the
 *  directory graspPlanning and the files defining the structure appearing in
 *  these prototypes.
 */
#ifdef GRASP_PLANNING
#ifndef GRASP_PLANNING_PKG_H
#define GRASP_PLANNING_PKG_H

#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "p3d.h"

#include "glpk.h"
#include "../graspPlanning/include/gp_volInt.h"
#include "../graspPlanning/include/graspPlanning.h"
#include "../graspPlanning/include/gpKdTree.h"
#include "../graspPlanning/include/gpConvexHull.h"
#include "../graspPlanning/include/gp_grasp_io.h"
#include "../graspPlanning/include/gpWorkspace.h"
#include "../graspPlanning/include/gpPose.h"

#include "../graspPlanning/proto/gp_grasping_utils_proto.h"
#include "../graspPlanning/proto/gp_grasp_generation_proto.h"
#include "../graspPlanning/proto/gp_extensionsM3D_proto.h"
#include "../graspPlanning/proto/gp_force_closure_proto.h"
#include "../graspPlanning/proto/gp_geometry_proto.h"
#include "../graspPlanning/proto/gp_inertia_axes_proto.h"
#include "../graspPlanning/proto/gp_volInt_proto.h"
#include "../graspPlanning/proto/FormgraspPlanning_proto.h"

extern "C" {
#include "gbM/gbStruct.h"
#include "gbM/gb.h"
#include "gbM/Proto_gb.h"
#include "gbM/Proto_gbModeles.h"
}

#endif
#endif
