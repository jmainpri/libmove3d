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
