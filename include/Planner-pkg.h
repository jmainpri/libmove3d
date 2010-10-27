/* 
 *  This file includes the prototypes of the functions defined in the
 *  directory planner and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef PLANNER_PKG_H
#define PLANNER_PKG_H

/* struct */
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "p3d.h"

#include "elastic.h"
#include "traj.h"
#include "localpath.h"
#include "device.h"

#include "environment.h"

//start path deform
#include "dyna_list.h"
//end path deform
#include "roadmap.h"
/* globals */
extern pp3d_graph  XYZ_TAB_GRAPH[];

extern int (*ext_p3d_run_rrt)(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void));


/* proto */
#include "../planner/proto/planner_proto.h"
#include "../planner/Diffusion/proto/Diffusion_proto.h"
#include "../planner/astar/proto/astar_proto.h"
#include "../planner/dfs/proto/dfs_proto.h"
#include "../planner/proto/p3d_softMotion_traj_proto.h"

#endif	/* #ifndef PLANNER_PKG_H */
