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
#ifndef GP_CONVEXHULL3D_H
#define GP_CONVEXHULL3D_H


//! @defgroup convexHull
//! @ingroup graspPlanning 
//! This module implements some classes to compute the convex hull of a point set (in arbitrary dimension).
//! It is based on qhull( http://www.qhull.org/).

class gpConvexHull;


//! @ingroup convexHull 
//! Class to store a ridge of a face.
class gpRidge
{
  friend class gpConvexHull;

  private:
   unsigned int dimension_; /*!< space dimension */
   std::vector<unsigned int> vertices_; /*!< indices of the ridge's vertices in a point array */
   unsigned int id_; /*!< ridge ID */
   bool toporient_; /*!< gives the orientation of the face associated to the ridge (used by the function orderFromRidges())*/
   int setDimension(unsigned int dimension);
   int resize(unsigned int size);
  public:
   gpRidge();
   ~gpRidge();
   gpRidge(unsigned int dimension, unsigned int vertex_number);
   unsigned int operator [] (const unsigned int i) const;
   unsigned int& operator [] (const unsigned int i);
   unsigned int nbVertices() { return vertices_.size(); }
   unsigned int id() { return id_; }
   unsigned int toporient() { return toporient_; }
};


//! @ingroup convexHull 
//! Class to store a face of a point set convex hull.
class gpFace
{
  friend class gpConvexHull;

  private:
   unsigned int dimension_; /*!< space dimension */
   std::vector<unsigned int> vertices_; /*!< indices of the face's vertices in a point array */
   unsigned int id_; /*!< face ID */
   //! face hyperplane equation (offset + normal)
   double offset_; /*!< offset (to the origin) of the face's hyperplane */
   std::vector<double> normal_; /*!< face's normal vector*/
   //! NB: if P is a point on the face's hyperplane, we have dot_product(normal, P) + offset = 0
   std::vector<double> center_; /*!< face's centrum */
   std::vector<gpRidge> ridges_;

   int setDimension(unsigned int dimension);
   int resize(unsigned int size);
   int resizeRidgeNumber(unsigned int size);
  public:
   gpFace();
   ~gpFace();
   gpFace(unsigned int dimension, unsigned int vertex_number);
   unsigned int operator [] (const unsigned int i) const;
   unsigned int& operator [] (const unsigned int i);
   unsigned int nbVertices() { return vertices_.size(); }
   unsigned int nbRidges() { return ridges_.size(); }
   std::vector<double> normal() { return normal_; }
   std::vector<double> center() { return center_; }
   unsigned int id() { return id_; }
   double offset() { return offset_; }
   std::vector<unsigned int> vertices() { return vertices_; }
   int print();
   int reverseVertexOrder();
   int orderFromRidges();
};


//! @ingroup convexHull 
//! One of classes used to store the Voronoi diagram of a point set.
class gpVoronoiRidge
{
  friend class gpConvexHull;
  friend class gpConvexHull2D;
  friend class gpConvexHull3D;

  private:
   //!Indices of the two points (in the "points_" array of ConvexHull) that are separated by this ridge.
   //! If the ridge has only one adjacent voronoi cell, one of the two sites id will be set to UINT_MAX.
   //! These indices can be used to find the neighbors of a given voronoi cell.
   unsigned int site1_id_, site2_id_; 

   //! Indices of the ridge vertices (referring to elements in the "voronoi_vertices_" array of ConvexHull).
   std::vector<unsigned int> vertices_;

   std::vector<double> center_;  /*!< ridge's centrum */
   std::vector<double> normal_;  /*!< ridge's normal vector: only used in 3D */
};


//! @ingroup convexHull 
//! One of classes used to store the Voronoi diagram of a point set.
class gpVoronoiCell
{
  friend class gpConvexHull;
  friend class gpConvexHull2D;
  friend class gpConvexHull3D;

  private:
   //!Index of the point (in the "points_" array of ConvexHull) the cell corresponds.
   unsigned int site_id_;

   //!Indices of the ridges (in the "voronoi_ridges_" array of ConvexHull) that defines the cell boundaries.
   std::vector< unsigned int > ridges_;

   std::vector<bool> ccw_; /*!< for each ridge, tells if it is counter-clockwise oriented: only used in 3D */
};

//! @ingroup convexHull 
//! This class is used to compute the convex hull (or the voronoi regions) of a set of points in arbitrary dimension (via Qhull library).
//! Special 3D and 6D case has a specific class (see below).
//! Set the point set at creation or with setPoints() and then call compute() to compute the convex hull
//! or voronoi() to compute the voronoi regions.
class gpConvexHull
{
  protected:
   unsigned int dimension_; //! space dimension (size of the point vectors)

   //! flag to tell wether or not the current content of hull_vertices and hull_faces corresponds
   //! to the actual convex hull of the current point set
   bool up_to_date_;

   std::vector< std::vector<double> > points_;  /*!<  the input set of points */

   //! the radius of the largest sphere centered on the origin and fully contained in the convex hull (it is null if the
   //! hull does not contain the origin):
   double largest_ball_radius_;

   int reset();

  public:
   std::vector<unsigned int> hull_vertices; /*!<  the hull vertices (indices to elements of the point set) */

   //! the hull faces (arrays containing indices to elements of the point set).
   std::vector<gpFace> hull_faces;

   //! for voronoi regions computation:
   std::vector< std::vector< double > > voronoi_vertices_;
   std::vector<gpVoronoiRidge> voronoi_ridges_;
   std::vector<gpVoronoiCell> voronoi_cells_;

   gpConvexHull();
   gpConvexHull(const std::vector< std::vector<double> > &points);
   virtual ~gpConvexHull();
   int setPoints(const std::vector< std::vector<double> > &points);
   unsigned int nbVertices() {  return hull_vertices.size(); }
   unsigned int nbFaces()    {  return hull_faces.size(); }
   int pointCoordinates(unsigned int i, std::vector<double> &coord);

   int compute(bool simplicial_facets, double postMergingCentrumRadius, bool verbose= true);
   virtual int voronoi(bool verbose= true);
   int draw();
   int drawFace(unsigned int face_index);
   int print();
   double largest_ball_radius();
   int isPointInside(std::vector<double> point, bool &inside, double &distance);
};


//! @ingroup convexHull 
//! Derives from gpConvexHull class to deal with 3D points.
//! Adds a new constructor and display functions.
class gpConvexHull3D: public gpConvexHull
{
  public:
   gpConvexHull3D();
   gpConvexHull3D(p3d_vector3 *point_array, unsigned int nbpoints);
   int setPoints(p3d_vector3 *point_array, unsigned int nbpoints);
   int voronoi(bool verbose= true);
   int draw(bool wireframe= false);
   int drawFace(unsigned int face_index);
};


//! @ingroup convexHull 
//! Derives from gpConvexHull class to deal with 6D points.
//! Adds a new constructor.
class gpConvexHull6D: public gpConvexHull
{
  public:
   gpConvexHull6D(double (*point_array)[6], unsigned int nbpoints_);
};

int gpSample_polyhedron_convex_hull(struct p3d_polyhedre *polyhedron, double step, std::vector<gpVector3D> &samples);

#endif
