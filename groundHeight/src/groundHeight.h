/*
 *  Copyright (c) 2007 Florent Lamiraux
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef GROUND_HEIGHT_H
#define GROUND_HEIGHT_H
#ifdef __cplusplus

#include <stddef.h>
#include <vector>

/*! \brief Computation of the intersection of a vertical straight line with the ground.
 
  The ground is represented by a list of triangular facets.
  To speed-up computation of ground elevation, the bounding rectangle of the ground is partitioned into squares.
  To each square is associated a list of triangles that have a non-empty intersection with the square.
  
  The desired average number of triangle per square in the above partition is given to the connstructor of the object.
 */
  


class CgroundHeight 
{
public:
  /**
     \name Initialization
     @{
  */

  /**
     \brief Constructor
     \param inNbTrianglesPerSquare average number of triangles per square of the partition of the ground bounding rectangle.

  */
  CgroundHeight(unsigned int inNbTrianglesPerSquare):
    attInitialized(false), attFiltering(true), attGroundXMin(1e30), attGroundYMin(1e30), attGroundXMax(-1e30), 
    attGroundYMax(-1e30), attNbSquaresAlongX(0), attNbSquaresAlongY(0), attSquareSize(0), 
      attTriangleIndexVectorArray(NULL), attAverageNbTrianglesPerSquare(inNbTrianglesPerSquare) {}
  /**
     \brief Constructor
  */
  CgroundHeight():
    attInitialized(false), attFiltering(false), attGroundXMin(1e30), attGroundYMin(1e30), attGroundXMax(-1e30), 
    attGroundYMax(-1e30), attNbSquaresAlongX(0), attNbSquaresAlongY(0), attSquareSize(0), 
      attTriangleIndexVectorArray(NULL), attAverageNbTrianglesPerSquare(0) {}
  /**
     \brief Destructor
     Deallocate CgroundHeight::attTriangleIndexVectorArray
  */
  ~CgroundHeight();

  /**
     \brief Get average number of triangles per square in partition of ground area.
  */
  unsigned int averageNbTrianglesPerSquare() {
    return attAverageNbTrianglesPerSquare;
  }

  /**
     \brief Initialization of filtering data-structure
  */
  void initializeGrid();

  /**
     @}
  */

  /**
     \name Insertion of vectices and triangles.

     @{
  */

  /**
     \brief Add a vertex
     \param inX x-coordinate of point
     \param inY y-coordinate of point
     \param inZ z-coordinate of point

     \return rank of point in vector.
  */
  unsigned int addVertex(const double & inX = 0., const double & inY = 0., const double & inZ = 0.);

  /**
     \brief Add a triangle.
     The vertices of the triangle are the points at given rank in vertex vector.

     \return false if one point index is out of range, true otherwise.
  */
  bool addTriangle(unsigned int inPt1, unsigned int inPt2, unsigned int inPt3);

  /**
     @}
  */

  /**
     \name Ground height computation

     @{
  */
  /**
     \brief Compute whether a vertical straight line intersects the ground and compute height of intersection.

     \param inX, inY coordinate of the vectical straigtht line.
     \retval groundHeight z-coordinate of intersection if any.
     \return true if line intersects ground, false otherwise.
  */
  bool intersectionVerticalLineWithGround(double inX, double inY, double& groundHeight);

  /**
     @}
  */

  /**
     \name Access to triangles

     @{
  */
  
  /**
     \brief Return the number of triangles defining the ground.
  */
  unsigned int countTriangles() { return attTriangleVector.size();};

  /**
     \brief Get triangle at given rank in vector.
  */
  bool triangleAtRank(unsigned int inRank, double& outX1, double& outY1, double& outZ1, 
		      double& outX2, double& outY2, double& outZ2, 
		      double& outX3, double& outY3, double& outZ3);
  
  /**
     @}
  */
private:
  
  /*
   
                  P R I V A T E   T Y P E S

  */
  /**
     \brief Triangle defined by three indices in CgroundHeight::attVertexVector
  */
  typedef struct Ttriangle {
    unsigned int pt1;
    unsigned int pt2;
    unsigned int pt3;
  } Ttriangle;

  /**
     \brief 3D point.
  */
  typedef struct T3dPoint {
    double x;
    double y;
    double z;
  } T3dPoint;

  /*
   
                  P R I V A T E   M E T H O D S

  */

  /**
     \brief Compute bounding rectangle of a triangle.
  */
  void boundingRectangleOfTriangle(const Ttriangle& triangle,
				   double& xminTriangle,
				   double &yminTriangle,
				   double& xmaxTriangle,
				   double& ymaxTriangle);
  
  /**
     \brief Compute the minimal and maximal x and y coordinates of all triangles in ground.
     \return false if triangle vector is empty, true otherwise.

  */
  bool computeGroundBoundingRectangle(); 

  /* 
     \brief Test whether one of the vertices of a triangle is inside a square.

     Square edges are aligned with coordinate axis.
     
     \param x1,y1 coordinates of first vectex of the triangle.
     \param x2,y2 coordinates of second vectex of the triangle.
     \param x3,y3 coordinates of third vectex of the triangle.

     \param xMinSquare, yMinSquare, xMaxSquare, yMaxSquare two opposite vertices of the square.
 */

  bool isTriangleInsideSquare(double x1, double y1, 
			      double x2, double y2, 
			      double x3, double y3, 
			      double xMinSquare, double yMinSquare, 
			      double xMaxSquare, double yMaxSquare);

/*
  \brief Test whether a point lies inside a triangle.
  
  \param x,y Coordinates of the point.
  \param x1,y1,x2,y2,x3,y3 Vectices of the triangle.

 */

  bool isPointInsideTriangle(double x, double y,
			     double x1, double y1, 
			     double x2, double y2, 
			     double x3, double y3);
  
  
  /* 
     \brief Test whether one of the vertices of a square is inside a triangle
     
     Square edges are aligned with coordinate axis.
     
     \param x1,y1,x2,y2,x3,y3  Vertices of the triangle.
     \param xMinSquare, yMinSquare,xMaxSquare,yMaxSquare Two opposite vertices of the square.
  */

  bool isSquareInsideTriangle(double x1, double y1, 
			      double x2, double y2, 
			      double x3, double y3, 
			      double xMinSquare, double yMinSquare, 
			      double xMaxSquare, double yMaxSquare);
  

  /*
    \brief Test whether two segments intersect.

    \param x1,y1,x2,y2 Extremities of first line-segment.
    \param x3,y3,x4,y4 Extremities of second line-segment.

 */
  
  bool doSegmentsIntersect(double x1, double y1, 
			   double x2, double y2, 
			   double x3, double y3, 
			   double x4, double y4);
  

  /* 
     \brief Test whether the edges of a triangle and of a square intersect.

     Square edges are aligned with coordinate axis.

     \param x1,y1,x2,y2,x3,y3  Vertices of the triangle.
     \param xMinSquare, yMinSquare,xMaxSquare,yMaxSquare Two opposite vertices of the square.

  */
  bool doEdgesIntersect(double x1, double y1, 
			double x2, double y2, 
			double x3, double y3, 
			double xMinSquare, double yMinSquare, 
			double xMaxSquare, double yMaxSquare);
  /**
     \brief Test whether a triangle and a square areas have non empty intersection.
     
     \param triangle a triangle.
     \param i,j indices of squares in array CgroundHeight::attTriangleIndexVectorArray.
  */
  bool doesTriangleIntersectSquare(const Ttriangle& triangle, 
				   unsigned int i, unsigned int j);



  /**
     \brief Compute size and number of squares in partition
  */
  void computeSizeAndNumberOfSquares();

  /**
     \brief Compute whether a vertical straight line and a triangle intersect.

     \param triangle the triangle.
     \param inX, inY coordinate of the vectical straight line.
     \retval groundHeight z-coordinate of the intersection if any.
     \return true if intersection exists, false otherwise.
  */

  bool intersectionVerticalLineWithTriangle(const Ttriangle& triangle, double inX, 
					    double inY, double& groundHeight);

  /*
   
                  P R I V A T E   A T T R I B U T E S

  */

  /**
     \brief Whether object has been initialized.
  */
  bool attInitialized;
  /**
     \brief Whether object needs filtering
  */
  bool attFiltering;
  /**
     \brief Vector of vectices.
  */
  std::vector<T3dPoint> attVertexVector;

  /**
     \brief Vector of facets.
  */
  std::vector<Ttriangle> attTriangleVector;

  /**
     \brief Bounding rectangle.
  */
  double attGroundXMin;
  /**
     \brief Bounding rectangle.
  */
  double attGroundYMin;
  /**
     \brief Bounding rectangle.
  */
  double attGroundXMax;
  /**
     \brief Bounding rectangle.
  */
  double attGroundYMax;
  /**
     \brief Number of squares in x direction 
  */
  unsigned int attNbSquaresAlongX;         
  /**
     \brief Number of squares in y direction 
  */
  unsigned int attNbSquaresAlongY;         
  /* 
     \brief Size of squares 
  */
  double attSquareSize;             
  /* 
     Number of polyhedra for this floor 
     int nb_polyhedron;       
  */
  /* array of polyhedra for this floor 
     poly_polyhedre** polyhedron_array; */
  
  /* 
     \brief Vector of facet indices for each square
     
     Index correspond to vector attTriangleVector.
  */
  std::vector<unsigned int>** attTriangleIndexVectorArray; 
  
  /**
    \brief Average number of triangles per square in partition of ground area.
  */
  unsigned int attAverageNbTrianglesPerSquare;
};

#endif /* __cplusplus */

#endif
