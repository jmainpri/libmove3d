/* 
 *  This file includes the prototypes of the functions defined in the
 *  directory p3d and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef SDK_PKG_H
#define SDK_PKG_H

#ifdef UNIX
#endif


/* struct */
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "p3d.h"

#include "traj.h"
#include "localpath.h"
#include "device.h"

#include "environment.h"
#include "roadmap.h"

#include "../sdk/sdk_types.h"

#include "../sdk/cfg.h"
#include "../sdk/kin.h"
#include "../sdk/env.h"
#include "../sdk/rdm.h"
#include "../sdk/geo.h"
#include "../sdk/ctr.h"

/* #include "scenario.h" */

#include "../sdk/kmp.h"
#include "../sdk/lph.h"
#include "../sdk/trj.h"
#include "../sdk/data.h"


/* globals */
/*
extern pp3d_env  XYZ_ENV;
extern pp3d_env  XYZ_TAB_ENV[];
extern int       XYZ_NUM_ENV;
extern pp3d_rob  XYZ_ROBOT;
extern pp3d_obj  XYZ_OBSTACLES;
*/
  /* pointer to function to choose the type of bounding box computation */
/* extern void (*p3d_BB_update_BB_obj)(p3d_obj *obj, p3d_matrix4 mat);
*/

  /* pointer to function to choose the type of bounding box computation */
/* extern void (*p3d_BB_get_BB_poly)(p3d_poly *p,double *x1,double *x2,
				  double *y1,double *y2,double *z1,double *z2);
*/


/* proto */
#include "../sdk/proto/sdk_proto.h"

#ifdef UNIX
#endif

#endif /* #ifndef SDK_PKG_H */
