/*
 *  This file includes the prototypes of the functions defined in the
 *  directory localpath and the files defining the structure appearing in
 *  these prototypes.
 */


#ifdef UNIX
#ifdef __cplusplus
//extern "C" {
#endif
#endif

#define HRI_HRP2 /* This is where we define the robot we use for HRI planner */
////#define HRI_JIDO /* This is where we define the robot we use for HRI planner */

//AKP:Define which human model to use
//#define HRI_HUMAN_SUPERMAN
#define HRI_HUMAN_ACHILE


//#define HRI_JIDO

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

#include "dyna_list.h"
#include "roadmap.h"
#include "arm_models/pa10Const.h"
#include "forms.h"

#include "../hri_planner/include/hri_agent.h"
#include "../hri_planner/include/hri_bitmap.h"
#include "../hri_planner/include/hri_manip.h"
#include "../hri_planner/include/perspective.h"
#ifdef USE_MIGHTABILITY_MAPS
#include "../hri_planner/include/hri_affordance.h"
#include "../hri_planner/include/hrp2_gik.h"
#endif
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

#ifdef USE_MIGHTABILITY_MAPS
extern p3d_vector3 to_reach_target;
extern struct grid_3D grid_around_HRP2;
extern int HRP2_GIK_MANIP;// Just to set the type of the bitmap
extern int HRP2_GIK_path_calculated;
extern int Affordances_Found; 
extern hri_bitmapset * ACBTSET;
extern int grid_3d_affordance_calculated;
extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting
extern int SHOW_OBSTACLE_CELLS;
extern int CANDIDATE_POINTS_FOR_TASK_FOUND;

extern p3d_vector3 point_to_look;

extern int HRP2_HAND_spline_path_calculated;

extern struct SOLUTION_CONFIGS_FOR_HRP2 cur_gik_sol_configs;//It will store the final set of configurations to be executed on HRP2 

#endif




/* struct */



/* proto */

#include "../util/proto/gnuplot_proto.h"
#include "../hri_planner/proto/hri_planner_proto.h"
#include "../hri_planner/graphic/proto/hri_graphic_proto.h"
#ifdef USE_MIGHTABILITY_MAPS
#include "../hri_planner/proto/hri_affordance_include_proto.h"
#include "../hri_planner/proto/HRP2_gik_proto.h"
#endif

#ifdef UNIX
#ifdef __cplusplus
//}
#endif
#endif
