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
#include "groundWrapper.h"
#include "groundHeight.h"

int GHaddVertex(void* Obj, double  inX,  double  inY,  double  inZ) { 
  return ((CgroundHeight*)Obj)->addVertex( inX,   inY,   inZ);
}

int GHaddTriangle(void* Obj,unsigned int inPt1, unsigned int inPt2, unsigned int inPt3)
{
  return ((CgroundHeight*)Obj)->addTriangle(inPt1,  inPt2, inPt3);
}

int GHintersectionVerticalLineWithGround(void* Obj,double inX, double inY, double* groundHeight)
{
  return ((CgroundHeight*)Obj)->intersectionVerticalLineWithGround(inX, inY, *groundHeight);
}

void GHinitializeGrid(void* Obj) {
  ((CgroundHeight*)Obj)->initializeGrid();
}


void * GHinitCgroundHeight(unsigned int inNbTrianglesPerSquare) {
  return new CgroundHeight(inNbTrianglesPerSquare);
}
