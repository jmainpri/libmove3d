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

#include <math.h>
#include <iostream>
#include "groundHeight.h"

unsigned int CgroundHeight::addVertex(const double & inX, const double & inY, const double & inZ)
{
  T3dPoint pt;
  pt.x = inX;
  pt.y = inY;
  pt.z = inZ;

  attVertexVector.push_back(pt);
  
  return attVertexVector.size()-1;
}

bool CgroundHeight::addTriangle(unsigned int inPt1, unsigned int inPt2, unsigned int inPt3)
{
  Ttriangle triangle;

  /*
    Check that vertex indices do not exceed vector length.
  */
  if (inPt1 >= attVertexVector.size()) {
    std::cerr << "CgroundHeight::addTriangle: index of vectex 1 exceeds vector length." << std::endl;
    return false;
  }

  if (inPt2 >= attVertexVector.size()) {
    std::cerr << "CgroundHeight::addTriangle: index of vectex 2 exceeds vector length." << std::endl;
    return false;
  }

  if (inPt3 >= attVertexVector.size()) {
    std::cerr << "CgroundHeight::addTriangle: index of vectex 3 exceeds vector length." << std::endl;
    return false;
  }

  triangle.pt1 = inPt1;
  triangle.pt2 = inPt2;
  triangle.pt3 = inPt3;

  /* Check that triangle is not degenerated */
  double x1 = attVertexVector[inPt1].x;
  double y1 = attVertexVector[inPt1].y;
  //  double z1 = attVertexVector[inPt1].z;

  double x2 = attVertexVector[inPt2].x;
  double y2 = attVertexVector[inPt2].y;
  //  double z2 = attVertexVector[inPt2].z;

  double x3 = attVertexVector[inPt3].x;
  double y3 = attVertexVector[inPt3].y;
  //  double z3 = attVertexVector[inPt3].z;

  if ( fabs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) < 1e-10) {
    std::cerr << "CgroundHeight::addTriangle: triangle is degenerated" << std::endl;
    std::cerr << "  x1=" << x1 << " y1=" << y1 << std::endl;
    std::cerr << "  x2=" << x2 << " y2=" << y2 << std::endl;
    std::cerr << "  x3=" << x3 << " y3=" << y3 << std::endl;
    return false;
  }

  attTriangleVector.push_back(triangle);
  return  true;
}

bool CgroundHeight::intersectionVerticalLineWithTriangle(const Ttriangle& triangle, double inX, 
							 double inY, double& groundHeight)
{
  double lambda, mu, nu;
  unsigned int vertexId;

  /* get x,y,z coordinates of facet vertices */
  vertexId = triangle.pt1;
  double x1 = attVertexVector[vertexId].x;
  double y1 = attVertexVector[vertexId].y;
  double z1 = attVertexVector[vertexId].z;

  vertexId = triangle.pt2;
  double x2 = attVertexVector[vertexId].x;
  double y2 = attVertexVector[vertexId].y;
  double z2 = attVertexVector[vertexId].z;

  vertexId = triangle.pt3;
  double x3 = attVertexVector[vertexId].x;
  double y3 = attVertexVector[vertexId].y;
  double z3 = attVertexVector[vertexId].z;

  lambda = (x3*y2-y2*inX+inX*y3-x2*y3+inY*x2-x3*inY)/
    (-x1*y2+x1*y3+x3*y2+x2*y1-x2*y3-x3*y1);
  mu = (x1*y3-x1*inY+x3*inY-x3*y1+inX*y1-inX*y3)/
    (-x1*y2+x1*y3+x3*y2+x2*y1-x2*y3-x3*y1);
  nu = 1 - lambda - mu;

  if ((lambda >= 0) && (mu >= 0) && (nu >= 0)) {
    groundHeight = lambda*z1 + mu*z2 + nu*z3;
    return true;
  }
  return false;
}

bool CgroundHeight::intersectionVerticalLineWithGround(double inX, double inY, double& groundHeight)
{

  if (!attInitialized) {
    initializeGrid();
  }
  
  /* Check that ground bounding rectangle is not restricted to a point*/
  if (attSquareSize == 0) {
    return false;
  }
   
  /* find the square containing point (x,y) */
  unsigned int i = (unsigned int)floor((inX-attGroundXMin)/attSquareSize);
  unsigned int j = (unsigned int)floor((inY-attGroundYMin)/attSquareSize);

  /* Test that point lies in ground bounding rectangle*/
  if ((i<0) || (i>=attNbSquaresAlongX) || (j<0) || (j>=attNbSquaresAlongY)) {
    return false;
  }

  /* Get vector of indices of triangle corresponding to square (i,j)*/
  std::vector<unsigned int> triangleIndexVector = attTriangleIndexVectorArray[i][j];

  for (unsigned int trId = 0; trId < triangleIndexVector.size(); trId++) {
    unsigned int triangleIndex = triangleIndexVector[trId];
    Ttriangle triangle = attTriangleVector[triangleIndex];
    if (intersectionVerticalLineWithTriangle(triangle, inX, inY, groundHeight))
      return true;
  }
  return false;
}



bool CgroundHeight::triangleAtRank(unsigned int inRank, double& outX1, double& outY1, double& outZ1, 
				   double& outX2, double& outY2, double& outZ2, 
				   double& outX3, double& outY3, double& outZ3)
{
  if (inRank >= attTriangleVector.size()) {
    std::cerr << "CgroundHeight::triangleAtRank: inRank=" << inRank  
	      << " exceeds number of triangles: " << attTriangleVector.size() << std::endl;
    return false;
  }

  const T3dPoint& point1 = attVertexVector[attTriangleVector[inRank].pt1];
  outX1 = point1.x;
  outY1 = point1.y;
  outZ1 = point1.z;

  const T3dPoint& point2 = attVertexVector[attTriangleVector[inRank].pt2];
  outX2 = point2.x;
  outY2 = point2.y;
  outZ2 = point2.z;

  const T3dPoint& point3 = attVertexVector[attTriangleVector[inRank].pt3];
  outX3 = point3.x;
  outY3 = point3.y;
  outZ3 = point3.z;

  return true;
}






void CgroundHeight::boundingRectangleOfTriangle(const Ttriangle& triangle,
						double& xminTriangle,
						double& yminTriangle,
						double& xmaxTriangle,
						double& ymaxTriangle)
{
  /* 
     Get 3 points of triangle 
  */
  const T3dPoint& point1 = attVertexVector[triangle.pt1];
  const T3dPoint& point2 = attVertexVector[triangle.pt2];
  const T3dPoint& point3 = attVertexVector[triangle.pt3];
  
  /* Get x and y coordinate of each point of triangle. */
  double x1 = point1.x;
  double y1 = point1.y;
  
  double x2 = point2.x;
  double y2 = point2.y;
  
  double x3 = point3.x;
  double y3 = point3.y;
  
  /* 
     Compute bounding rectangle of triangle 
  */
  xminTriangle = x1;
  xminTriangle = (x2 < xminTriangle ? x2 : xminTriangle);
  xminTriangle = (x3 < xminTriangle ? x3 : xminTriangle);
  
  yminTriangle = y1;
  yminTriangle = (y2 < yminTriangle ? y2 : yminTriangle);
  yminTriangle = (y3 < yminTriangle ? y3 : yminTriangle);
  
  xmaxTriangle = x1;
  xmaxTriangle = (x2 > xmaxTriangle ? x2 : xmaxTriangle);
  xmaxTriangle = (x3 > xmaxTriangle ? x3 : xmaxTriangle);
  
  ymaxTriangle = y1;
  ymaxTriangle = (y2 > ymaxTriangle ? y2 : ymaxTriangle);
  ymaxTriangle = (y3 > ymaxTriangle ? y3 : ymaxTriangle);
}


bool CgroundHeight::computeGroundBoundingRectangle()
{
  if (attTriangleVector.size() == 0) {
    return false;
  }

  attGroundXMin = 1e30;
  attGroundYMin = 1e30;
  attGroundXMax = -1e30;
  attGroundYMax = -1e30;

  double xminTriangle;
  double yminTriangle;
  double xmaxTriangle;
  double ymaxTriangle;

  /* 
     Loop over triangles of the ground.
  */
  for (unsigned int trId=0; trId < attTriangleVector.size(); trId++) {
 
    const Ttriangle& triangle = attTriangleVector[trId];
  
    boundingRectangleOfTriangle(triangle, xminTriangle, yminTriangle, 
				xmaxTriangle, ymaxTriangle);

    attGroundXMin = (attGroundXMin <= xminTriangle ? attGroundXMin : xminTriangle);
    attGroundYMin = (attGroundYMin <= yminTriangle ? attGroundYMin : yminTriangle);
    attGroundXMax = (attGroundXMax >= xmaxTriangle ? attGroundXMax : xmaxTriangle);
    attGroundYMax = (attGroundYMax >= ymaxTriangle ? attGroundYMax : ymaxTriangle);
  }
  
  return true;
}


bool CgroundHeight::isTriangleInsideSquare(double x1, double y1, 
					   double x2, double y2, 
					   double x3, double y3, 
					   double xMinSquare, double yMinSquare, 
					   double xMaxSquare, double yMaxSquare)
{
  if ((x1>=xMinSquare) && (x1<=xMaxSquare) && (y1>=yMinSquare) && (y1<=yMaxSquare)) {
    return true;
  }
  if ((x2>=xMinSquare) && (x2<=xMaxSquare) && (y2>=yMinSquare) && (y2<=yMaxSquare)) {
    return true;
  }
  if ((x3>=xMinSquare) && (x3<=xMaxSquare) && (y3>=yMinSquare) && (y3<=yMaxSquare)) {
    return true;
  }
  return false;
}


bool CgroundHeight::isPointInsideTriangle(double x, double y,
					  double x1, double y1, 
					  double x2, double y2, 
					  double x3, double y3)
{
  double vector_prod;
  int sign1, sign2, sign3;

  vector_prod = (x-x1)*(y2-y1) - (y-y1)*(x2-x1);
  sign1 = (vector_prod >= 0) ? 1 : 0;
  
  vector_prod = (x-x2)*(y3-y2) - (y-y2)*(x3-x2);
  sign2 = (vector_prod >= 0) ? 1 : 0;

  vector_prod = (x-x3)*(y1-y3) - (y-y3)*(x1-x3);
  sign3 = (vector_prod >= 0) ? 1 : 0;
  
  if ((sign1 == sign2) && (sign1 == sign3)) {
    return true;
  }
  
  return false;
}


bool CgroundHeight::isSquareInsideTriangle(double x1, double y1, 
					   double x2, double y2, 
					   double x3, double y3, 
					   double xMinSquare, double yMinSquare, 
					   double xMaxSquare, double yMaxSquare)
{
  if (isPointInsideTriangle(xMinSquare, yMinSquare, x1, y1, x2, y2, x3, y3)) {
    return true;
  }

  if (isPointInsideTriangle(xMaxSquare, yMinSquare, x1, y1, x2, y2, x3, y3)) {
    return true;
  }

  if (isPointInsideTriangle(xMinSquare, yMaxSquare, x1, y1, x2, y2, x3, y3)) {
    return true;
  }

  if (isPointInsideTriangle(xMaxSquare, yMaxSquare, x1, y1, x2, y2, x3, y3)) {
    return true;
  }

  return false;
}


bool CgroundHeight::doSegmentsIntersect(double x1, double y1, 
					double x2, double y2, 
					double x3, double y3, 
					double x4, double y4)
{
  double determinant = (x2-x1)*(y3-y4) - (y2-y1)*(x3-x4);
  double lambda, mu;

  if (determinant < 1e-10) {
    return false;
  }
  lambda = (x4*y1-x3*y1-x1*y4-y3*x4+x1*y3+y4*x3)/
    (x2*y4-x2*y3-x1*y4+x1*y3-x4*y2+x4*y1+x3*y2-x3*y1);
  mu = -(x1*y2-x2*y1+x2*y3-x1*y3-x3*y2+x3*y1)/
    (x2*y4-x2*y3-x1*y4+x1*y3-x4*y2+x4*y1+x3*y2-x3*y1);

  if ((lambda >= 0) && (lambda <= 1) && (mu >= 0) && (mu <= 1)) {
    return true;
  }
  return false;
}


bool CgroundHeight::doEdgesIntersect(double x1, double y1, 
				     double x2, double y2, 
				     double x3, double y3, 
				     double xMinSquare, double yMinSquare, 
				     double xMaxSquare, double yMaxSquare)
{
  if (doSegmentsIntersect(x1, y1, x2, y2, xMinSquare, yMinSquare, xMaxSquare, yMinSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x2, y2, x3, y3, xMinSquare, yMinSquare, xMaxSquare, yMinSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x3, y3, x1, y1, xMinSquare, yMinSquare, xMaxSquare, yMinSquare)) {
    return true;
  }

  if (doSegmentsIntersect(x1, y1, x2, y2, xMaxSquare, yMinSquare, xMaxSquare, yMaxSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x2, y2, x3, y3, xMaxSquare, yMinSquare, xMaxSquare, yMaxSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x3, y3, x1, y1, xMaxSquare, yMinSquare, xMaxSquare, yMaxSquare)) {
    return true;
  }

  if (doSegmentsIntersect(x1, y1, x2, y2, xMaxSquare, yMaxSquare, xMinSquare, yMaxSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x2, y2, x3, y3, xMaxSquare, yMaxSquare, xMinSquare, yMaxSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x3, y3, x1, y1, xMaxSquare, yMaxSquare, xMinSquare, yMaxSquare)) {
    return true;
  }

  if (doSegmentsIntersect(x1, y1, x2, y2, xMinSquare, yMaxSquare, xMinSquare, yMinSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x2, y2, x3, y3, xMinSquare, yMaxSquare, xMinSquare, yMinSquare)) {
    return true;
  }
  if (doSegmentsIntersect(x3, y3, x1, y1, xMinSquare, yMaxSquare, xMinSquare, yMinSquare)) {
    return true;
  }
  return false;
}


bool CgroundHeight::doesTriangleIntersectSquare(const Ttriangle& triangle, 
						unsigned int i, unsigned int j)
{
  unsigned int vectexId;
  double x1, y1, x2, y2, x3, y3, xMinSquare, xMaxSquare, yMinSquare, yMaxSquare;

  /* get x,y coordinates of facet vertices */
  vectexId = triangle.pt1;
  x1 = attVertexVector[vectexId].x;
  y1 = attVertexVector[vectexId].y;

  vectexId = triangle.pt2;
  x2 = attVertexVector[vectexId].x;
  y2 = attVertexVector[vectexId].y;

  vectexId = triangle.pt3;
  x3 = attVertexVector[vectexId].x;
  y3 = attVertexVector[vectexId].y;

  /* get bounds of square */
  xMinSquare = attGroundXMin + i*attSquareSize;
  xMaxSquare = attGroundXMin + (i+1)*attSquareSize;
  yMinSquare = attGroundYMin + j*attSquareSize;
  yMaxSquare = attGroundYMin + (j+1)*attSquareSize;

  if (isTriangleInsideSquare(x1, y1, x2, y2, x3, y3, 
			     xMinSquare, yMinSquare, xMaxSquare, yMaxSquare)) {
    return true;
  }

  if (isSquareInsideTriangle(x1, y1, x2, y2, x3, y3, 
			     xMinSquare, yMinSquare, xMaxSquare, yMaxSquare)) {
    return true;
  }
  
  if (doEdgesIntersect(x1, y1, x2, y2, x3, y3, 
		       xMinSquare, yMinSquare, xMaxSquare, yMaxSquare)) {
    return true;
  }
  return false;

}


CgroundHeight::~CgroundHeight()
{
  if(!attFiltering)
    {
      for (unsigned int i=0; i<attNbSquaresAlongX; i++) {
	delete attTriangleIndexVectorArray[i];
	attTriangleIndexVectorArray[i] = NULL;
      }
      delete attTriangleIndexVectorArray;
    }
}


void CgroundHeight::computeSizeAndNumberOfSquares()
{
  if(attFiltering)
    {
      unsigned int nbTriangles = attTriangleVector.size();
      double area = (attGroundXMax-attGroundXMin)*(attGroundYMax-attGroundYMin);
      unsigned int nbSquares = (unsigned int)ceil((double)nbTriangles/attAverageNbTrianglesPerSquare);
      
      attSquareSize = sqrt(area/nbSquares);
      
      attNbSquaresAlongX = (unsigned int)ceil((attGroundXMax-attGroundXMin)/attSquareSize);
      attNbSquaresAlongY = (unsigned int)ceil((attGroundYMax-attGroundYMin)/attSquareSize);
    }
}






void CgroundHeight::initializeGrid()
{
  attInitialized=true;
  /*
    Compute bounding rectangle of ground.
  */
  if (!computeGroundBoundingRectangle()) {
    return;
  }

  /*
    Compute size and number of squares in partition
  */
  computeSizeAndNumberOfSquares();

  /*
    Allocate array of vectors of triangle.
  */
  attTriangleIndexVectorArray = new std::vector<unsigned int>*[attNbSquaresAlongX];
  for (unsigned int i=0; i<attNbSquaresAlongX; i++) {
    attTriangleIndexVectorArray[i] = new std::vector<unsigned int>[attNbSquaresAlongY];
  }

  /*
    Fill array of vectors of triangles.
  */

  /* Loop over triangles of the ground. */
  for (unsigned int trId=0; trId < attTriangleVector.size(); trId++) {

    double xminTriangle;
    double yminTriangle;
    double xmaxTriangle;
    double ymaxTriangle;

    Ttriangle triangle = attTriangleVector[trId];

    /* 
       Compute bounding rectangle of triangle.
    */
    boundingRectangleOfTriangle(triangle, xminTriangle, yminTriangle, 
				xmaxTriangle, ymaxTriangle);

    /*
      Get minimum and maximum indices of squares in partition.
    */
    unsigned int imin = (unsigned int)floor((xminTriangle-attGroundXMin)/attSquareSize);
    unsigned int jmin = (unsigned int)floor((yminTriangle-attGroundYMin)/attSquareSize);

    unsigned int imax = (unsigned int)floor((xmaxTriangle-attGroundXMin)/attSquareSize);
    unsigned int jmax = (unsigned int)floor((ymaxTriangle-attGroundYMin)/attSquareSize);

    if (imin <0) {
      imin = 0;
    }
    if (imax>=attNbSquaresAlongX) {
      imax = attNbSquaresAlongX-1;
    }
    if (jmin <0) {
      jmin = 0;
    }
    if (jmax>=attNbSquaresAlongY) {
      jmax = attNbSquaresAlongY-1;
    }
    
    /* loop over ground squares */
    for (unsigned int i=(unsigned int)imin; i<=(unsigned int)imax; i++) {
      for (unsigned int j=(unsigned int)jmin; j<=(unsigned int)jmax; j++) {
	if (doesTriangleIntersectSquare(triangle, i, j)) {
	  attTriangleIndexVectorArray[i][j].push_back(trId);
	}
      }
    }
  }
}


