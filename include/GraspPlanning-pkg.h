/*
 *  This file includes the prototypes of the functions defined in the
 *  directory graspPlanning and the files defining the structure appearing in
 *  these prototypes.
 */

#ifndef GRASP_PLANNING_PKG_H
#define GRASP_PLANNING_PKG_H

#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "p3d.h"

#include "../other_libraries/glpk-4.31/installDir/include/glpk.h"
#include "../graspPlanning/include/volInt.h"
#include "../graspPlanning/include/graspPlanning.h"
#include "../graspPlanning/include/grasp_io.h"


#include "../graspPlanning/proto/grasping_utils_proto.h"
#include "../graspPlanning/proto/grasp_generation_proto.h"
#include "../graspPlanning/proto/extensionsM3D_proto.h"
#include "../graspPlanning/proto/force_closure_proto.h"
#include "../graspPlanning/proto/geometry_proto.h"
#include "../graspPlanning/proto/inertia_axes_proto.h"
#include "../graspPlanning/proto/volInt_proto.h"
#include "../graspPlanning/proto/FormgraspPlanning_proto.h"
#include "../graspPlanning/proto/grasp_io_proto.h"

#include "../other_libraries/gbM/src/gbStruct.h"
#include "../other_libraries/gbM/src/gb.h"

#include "../other_libraries/gbM/src/Proto_gb.h"
#include "../other_libraries/gbM/src/Proto_gbModeles.h"


#endif
