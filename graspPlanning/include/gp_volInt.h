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

#ifndef VOLINT_H
#define VOLINT_H

/*
   ============================================================================
   macros
   ============================================================================
*/

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

/*
   ============================================================================
   constants
   ============================================================================
*/

#define MAX_VERTS 10000     /* maximum number of polyhedral vertices */
#define MAX_FACES 20000     /* maximum number of polyhedral faces */
#define MAX_POLYGON_SZ 3 /* maximum number of verts per polygonal face */

//#define X 0
//#define Y 1
//#define Z 2


/*
   ============================================================================
   data structures
   ============================================================================
*/

typedef struct {
  int numVerts;
  double norm[3];
  double w;
  int verts[MAX_POLYGON_SZ];
  struct VOLINT_POLYHEDRON *poly;
} VOLINT_FACE;

typedef struct VOLINT_POLYHEDRON {
  int numVerts, numFaces;
  double verts[MAX_VERTS][3];
  VOLINT_FACE faces[MAX_FACES];
} VOLINT_POLYHEDRON;


typedef struct Mass_properties{
  double mass, volume, density;
  double r[3];            /* center of mass */
  double J[3][3];         /* inertia tensor */
} Mass_properties;


#endif
