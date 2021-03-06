/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#ifndef KCD_SYS_H
#define KCD_SYS_H

#include "triangles.h"
#include "kcd_aabb_tree.h"
#include "../proto/kcd_proto.h"
#include "math.h"

#ifdef KCD_MOVE3D
#include "p3d_sys.h"
  /* #define MY_REALLOC(ptr,type,oldn,newn) (type *)basic_realloc((void *)ptr,oldn,newn,sizeof(type)) */
#else

#include <stdlib.h>
#include <malloc.h>
#ifndef NULL
#define NULL 0
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef MY_ALLOC
#define MY_ALLOC(type,n) (type *) malloc((unsigned)(sizeof(type) * n))
#endif
#ifndef MY_REALLOC
#define MY_REALLOC(ptr,type,oldn,newn) (type *)realloc((void *)ptr,(unsigned)(sizeof(type) * newn))
#endif
#ifndef MY_FREE
#define MY_FREE(ptr,type,n) free(ptr)
#endif

#ifndef FUZZ
#define FUZZ 0.00001
#endif
#ifndef P3D_HUGE
#define P3D_HUGE 1E10
#endif
#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

#ifndef SQR
#define SQR(x) ((x)*(x))
#endif
#ifndef MAX
#define MAX(x,y)     (((x) > (y)) ? (x) : (y))
#endif
#ifndef MIN
#define MIN(x,y)     (((x) < (y)) ? (x) : (y))
#endif
#ifndef MOD
#define MOD(k,n)     ((k+n)%n)
#endif
#ifndef ABS
#define ABS(x)       (((x) >= 0.0) ? (x) : -(x))
#endif
#ifndef EQ
#define EQ(x,y)      (ABS((x)-(y)) < FUZZ)
#endif
#ifndef LEQ
#define LEQ(x,y)     ((x)<(y) || EQ(x,y))
#endif
#ifndef GEQ
#define GEQ(x,y)     ((x)>(y) || EQ(x,y))
#endif
#ifndef LNEQ
#define LNEQ(x,y)    (!GEQ(x,y))
#endif
#ifndef GNEQ
#define GNEQ(x,y)    (!LEQ(x,y))
#endif
#ifndef SIGN
#define SIGN(f)      (((f) < 0) ? (-1) : (1))
#endif

#endif /* KCD_MOVE3D */



#ifndef POLYHEDRON_ENTITY
#define POLYHEDRON_ENTITY 0
#endif
#ifndef CONVEX_POLYHEDRON
#define CONVEX_POLYHEDRON 1
#endif
#ifndef CONCAVE_POLYHEDRON
#define CONCAVE_POLYHEDRON 2
#endif
#ifndef SPHERE_ENTITY
#define SPHERE_ENTITY 3
#endif
#ifndef CUBE_ENTITY
#define CUBE_ENTITY 4
#endif
#ifndef BOX_ENTITY
#define BOX_ENTITY 5
#endif
#ifndef CYLINDER_ENTITY
#define CYLINDER_ENTITY 6
#endif
#ifndef CONE_ENTITY
#define CONE_ENTITY 7
#endif

/* Modif Pepijn june 2001 */ 

#ifndef KCD_SMALL_VOLUME_BOX
#define KCD_SMALL_VOLUME_BOX 8
#endif

#endif /* KCD_SYS_H */


