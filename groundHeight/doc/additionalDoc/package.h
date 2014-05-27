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
/**
\mainpage

\section intro Introduction

This package implements ground height computation.  A ground is
modeled by a set of triangles in \f$\textbf{R}^3\f$. The main fonction
of the library is the computation of the intersection of a vertical
straight line with the ground and particularly of the z-coordinate of 
this intersection called height.

\section filtering Speeding up computation

To speed up the computation of the ground height for a given vertical
straight line, an initialization step builds filtering
data-structure. The bounding horizontal rectangle of the ground is
partitioned into contiguous squares aligned with the x and y axes. In
each square, the set of triangles of the ground the projection of
which onto the horizontal plane have non-empty intersection with the
square is stored in a vector.

To compute the ground height for a given (x,y), the square containing (x,y) is determined and only 
the triangles associated with this square are tested.

\image html partition.png "A ground is modeled as a set of triangles in 3d-space. The (x,y) plane is partitioned into a set of contiguous squares. In each square, the set of triangles the projection of which onto the horizontal plane have non-empty intersection with the square are stored."

\section how-to Using the ground height library

To use the functions implemented in this library and described above, the following steps
have to be followed.

\li Construct an CgroundHeight object. The constructor takes as input
the desired average number of triangles per square and computes the corresponding number of squares in the partition.

\li Insert the vertices and triangles of the ground by calling
functions CgroundHeight::addVertex and CgroundHeight::addTriangle.

\li Call CgroundHeight::initializeGrid() to  build the filtering structure


Then, function CgroundHeight::intersectionVerticalLineWithGround
computes whether a vertical straight line intersects the ground and if
yes returns the z-coordinate of the intersection.

*/
