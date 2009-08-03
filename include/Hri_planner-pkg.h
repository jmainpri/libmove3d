/*
 *  This file includes the prototypes of the functions defined in the
 *  directory localpath and the files defining the structure appearing in
 *  these prototypes.
 */


#ifdef UNIX
#ifdef __cplusplus
extern "C" {
#endif
#endif

#define HRI_HRP2 /* This is where we define the robot we use for HRI planner */

#include "p3d_sys.h"
#include "p3d_matrix.h"
#include "Rrt-pkg.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "environment.h"
#include "p3d.h"

#include "elastic.h"
#include "traj.h"
#include "localpath.h"
#include "device.h"

#include "roadmap.h"
#include "arm_models/pa10Const.h"

#include "../hri_planner/include/hri_bitmap.h"
#include "../hri_planner/include/hri_manip.h"
#include "../hri_planner/include/perspective.h"

  extern hri_bitmapset* BTSET;
  extern hri_bitmapset* BTSET_HUMAN;
  extern hri_bitmapset * INTERPOINT;
  extern hri_bitmapset * OBJSET;
  extern pp3d_graph BTGRAPH;
  extern hri_gik * HRI_GIK;
  extern double HRI_WEIGHTS[5];
  extern int * orderedpointsx;
  extern int * orderedpointsy;
  extern int * orderedpointsz;
  extern int orderedlength;
  extern int ordereddrawno;
  extern int HRI_GIK_CONTINUOUS;
  extern p3d_rob * PSP_ROBOT;

  extern int HRI_DRAW_TRAJ;



/* struct */



/* proto */

#include "../hri_planner/proto/hri_planner_proto.h"
#include "../hri_planner/graphic/proto/hri_graphic_proto.h"

#ifdef UNIX
#ifdef __cplusplus
}
#endif
#endif
