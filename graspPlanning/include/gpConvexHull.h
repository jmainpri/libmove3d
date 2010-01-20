#ifndef GPCONVEXHULL3D_H
#define GPCONVEXHULL3D_H


//! see qhull_interface.cpp for more information
extern "C"
{
  #include "qhull/qhull_a.h"
}


class gpConvexHull;


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
   int print();
   int reverseVertexOrder();
   int orderFromRidges();
};

//! This class is used to compute the convex hull of a set of points in arbitrary dimension (via Qhull library).
//! Special 3D and 6D cases have specific classes (see below).
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

  public:
   std::vector<unsigned int> hull_vertices; /*!<  the hull vertices (indices to elements of the point set) */

   //! the hull faces (arrays containing indices to elements of the point set).
   std::vector<gpFace> hull_faces;

   gpConvexHull();
   ~gpConvexHull();
   unsigned int nbVertices() {  return hull_vertices.size(); }
   unsigned int nbFaces()    {  return hull_faces.size(); }
   int pointCoordinates(unsigned int i, std::vector<double> &coord);

   int compute(bool simplicial_facets, double postMergingCentrumRadius, bool verbose= true);
   int draw();
   int drawFace(unsigned int face_index);
   int print();
   double largest_ball_radius();
   int isPointInside(std::vector<double> point, bool &inside, double &distance);
};


//! Derives from gpConvexHull class to deal with 3D points.
//! Adds a new constructor and display functions.
class gpConvexHull3D: public gpConvexHull
{
  public:
   gpConvexHull3D(p3d_vector3 *point_array, unsigned int nbpoints_);
   int draw(bool wireframe= false);
   int drawFace(unsigned int face_index);
};


//! Derives from gpConvexHull class to deal with 6D points.
//! Adds a new constructor.
class gpConvexHull6D: public gpConvexHull
{
  public:
   gpConvexHull6D(double (*point_array)[6], unsigned int nbpoints_);
};

int gpSample_polyhedron_convex_hull(p3d_polyhedre *polyhedron, double step, std::vector<gpVector3D> &samples);

#endif
