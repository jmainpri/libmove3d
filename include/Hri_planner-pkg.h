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

//////////#define HRI_HRP2 /* This is where we define the robot we use for HRI planner */
#define HRI_JIDO /* This is where we define the robot we use for HRI planner */

//AKP:Define which human model to use
//#define HRI_HUMAN_SUPERMAN
#define HRI_HUMAN_ACHILE

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

#ifdef WITH_XFORMS
#include "forms.h"
#endif

// Problem when using the minimal HRI module
// Without XFORMS (only hri_agent.c and hri_gik.c)
// The dependencies in files are not clear we should prevent
// using pkg.h usage and specify more precisly the interfaces 
// in source files

#if defined( HRI_GENERALIZED_IK ) && !defined( HRI_PLANNER )
#include "../hri_planner/include/hri_agent.h"
#include "../hri_planner/include/hri_manip.h"
#include "../util/proto/gnuplot_proto.h"
#include "../hri_planner/proto/hri_bitmap_proto.h"
#include "../hri_planner/proto/hri_agent_proto.h"
#endif

#ifdef HRI_PLANNER
#include "../hri_planner/include/hri_agent.h"
#include "../hri_planner/include/hri_manip.h"
#include "../hri_planner/include/perspective.h"
#include "../hri_planner/include/hri_bitmap.h"
#endif

#ifdef USE_MIGHTABILITY_MAPS
#include "../hri_planner/include/Mightability_Maps.h"
#endif

#ifdef USE_HRP2_GIK
#include "../hri_planner/include/hrp2_gik.h"
#include "../hri_planner/include/Geo_Sym_Sys.h"
#endif

extern hri_gik * HRI_GIK;
extern hri_bitmapset* BTSET;
extern hri_bitmapset* BTSET_HUMAN;
extern hri_bitmapset * INTERPOINT;
extern hri_bitmapset * OBJSET;
extern pp3d_graph BTGRAPH;
extern double HRI_WEIGHTS[5];
extern int * orderedpointsx;
extern int * orderedpointsy;
extern int * orderedpointsz;
extern int orderedlength;
extern int ordereddrawno;
extern int HRI_GIK_CONTINUOUS;
extern p3d_rob * PSP_ROBOT;

extern int HRI_DRAW_TRAJ;
extern HRI_AGENTS * GLOBAL_AGENTS;

#ifdef USE_MIGHTABILITY_MAPS
extern int SHOW_MM_BASED_OBJECT_REACHABLE;
extern int SHOW_MM_BASED_OBJECT_VISIBLE;
extern hri_bitmapset * ACBTSET;
extern robots_status robots_status_for_Mightability_Maps[100];
#endif

#ifdef USE_HRP2_GIK
extern int HRP2_HAND_spline_path_calculated;
extern struct SOLUTION_CONFIGS_FOR_HRP2 cur_gik_sol_configs;//It will store the final set of configurations to be executed on HRP2 
#endif

/* proto */
#ifdef HRI_PLANNER
#include "../hri_planner/proto/hri_planner_proto.h"
#include "../hri_planner/proto/hri_agent_proto.h"
#include "../hri_planner/graphic/proto/hri_graphic_proto.h"
#endif
#ifdef USE_MIGHTABILITY_MAPS
#include "../hri_planner/proto/hri_affordance_include_proto.h"
#endif

#ifdef USE_HRP2_GIK
#include "../hri_planner/proto/HRP2_gik_proto.h"
#endif


#ifdef UNIX
#ifdef __cplusplus
//}
#endif
#endif
