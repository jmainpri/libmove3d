/* 
 *  This file includes the prototypes of the functions defined in the
 *  directory p3d and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef P3D_PKG_H
#define P3D_PKG_H

#ifdef UNIX
#endif


/* struct */
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "dyna_list.h"              // modif Juan
#include "p3d.h"

#include "traj.h"
#include "localpath.h"
#include "device.h"
#include "cntrt.h"                  // modif Juan

#include "environment.h"

#include "roadmap.h"

#include "collision_context.h"


/* globals */
extern pp3d_env  XYZ_ENV;
extern pp3d_env  *XYZ_TAB_ENV;
extern int       XYZ_NUM_ENV;
extern int       XYZ_MAX_NUM_ENV;
extern pp3d_rob  XYZ_ROBOT;
extern pp3d_obj  XYZ_OBSTACLES;
  
  /* pointer to function to choose the type of bounding box computation */
typedef void (*p3d_BB_update_BB_fct_type)(p3d_obj *obj, p3d_matrix4 mat);
extern p3d_BB_update_BB_fct_type p3d_BB_update_BB_obj;

  /* pointer to function to choose the type of bounding box computation */
typedef void (*p3d_BB_get_BB_poly_fct_type)(p3d_poly *p,double *x1,double *x2,
			    double *y1,double *y2,double *z1,double *z2);
extern p3d_BB_get_BB_poly_fct_type p3d_BB_get_BB_poly;


/* proto */
#include "../p3d/proto/p3d_proto.h"

#ifdef UNIX
#endif

#endif /* #ifndef P3D_PKG_H */
