/* 
 *  This file includes the prototypes of the functions defined in the
 *  directory localpath and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef LOCALPATH_PKG_H
#define LOCALPATH_PKG_H

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

#include "rs.h"
#include "../localpath/flat/include/struct_flat.h"

/* globals */


/* proto */
#include "../localpath/flat/include/init_flat_proto.h"
#include "../localpath/flat/include/general_flat_proto.h"
#include "../localpath/flat/include/kinematic_flat_proto.h"
#include "../localpath/proto/localpath_proto.h"


#ifdef UNIX
#endif

#endif /* #ifndef LOCALPATH_PKG_H */
