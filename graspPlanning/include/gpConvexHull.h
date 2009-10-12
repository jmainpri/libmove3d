#ifndef GPCONVEXHULL3D_H
#define GPCONVEXHULL3D_H


//! see qhull_interface.cpp for more information
extern "C"
{
  #include "../other_libraries/qhull-install/include/qhull/qhull_a.h"
}


class gpConvexHull;


//! Class to store a ridge of a face.
class gpRidge
{
  friend class gpConvexHull;

  private:
   unsigned int _dimension; /*!< space dimension */
   std::vector<unsigned int> _v; /*!< indices of the ridge's vertices in a point array */
   unsigned int _id; /*!< ridge ID */
   bool _toporient; /*!< gives the orientation of the face associated to the ridge */
   int setDimension(unsigned int dimension);
   int resize(unsigned int size);
  public:
   gpRidge();
   gpRidge(unsigned int dimension, unsigned int vertex_number);
   unsigned int operator [] (const unsigned int i) const;
   unsigned int& operator [] (const unsigned int i);
   unsigned int nbVertices() { return _v.size(); }
   unsigned int id() { return _id; }
   unsigned int toporient() { return _toporient; }
};


//! Class to store a face of a point set convex hull.
class gpFace
{
  friend class gpConvexHull;

  private:
   unsigned int _dimension; /*!< space dimension */
   std::vector<unsigned int> _v; /*!< indices of the face's vertices in a point array */
   unsigned int _id; /*!< face ID */
   //! face hyperplane equation (offset + normal)
   double _offset; /*!< offset (to the origin) of the face's hyperplane */
   std::vector<double> _normal; /*!< face's normal vector*/
   //! NB: if P is a point on the face's hyperplane, we have dot_product(normal, P) + offset = 0
   std::vector<double> _center; /*!< face's centrum */
   bool _toporient; /*!<  true if facet has top-orientation (CCW order),  false if it has bottom-orientation (clockwise order) */
   std::vector<gpRidge> _ridges;

   int setDimension(unsigned int dimension);
   int resize(unsigned int size);
   int resizeRidgeNumber(unsigned int size);
  public:
   gpFace();
   gpFace(unsigned int dimension, unsigned int vertex_number);
   gpFace(unsigned int i1, unsigned int i2, unsigned int i3);
   unsigned int operator [] (const unsigned int i) const;
   unsigned int& operator [] (const unsigned int i);
   unsigned int nbVertices() { return _v.size(); }
   unsigned int nbRidges() { return _ridges.size(); }
   std::vector<double> normal() { return _normal; }
   std::vector<double> center() { return _center; }
   unsigned int id() { return _id; }
   double offset() { return _offset; }
   bool toporient() { return _toporient; }
   int print();
   int orderFromRidges();
};

//! This class is used to compute the convex hull of a set of points in arbitrary dimension (via Qhull library).
//! Special 3D and 6D cases have specific classes (see below).
class gpConvexHull
{
  protected:
   unsigned int _dimension; //! space dimension (size of the point vectors)

   //! flag to tell wether or not the current content of hull_vertices and hull_faces corresponds
   //! to the actual convex hull of the current point set
   bool _up_to_date;
   //! flag to tell wether or not the facet of the computed convex hull are simplicial
   bool _simplicial_facets;
   std::vector< std::vector<double> > _points;  /*!<  the input set of points */

   //! the radius of the largest sphere centered on the origin and fully contained in the convex hull (it is null if the
   //! hull does not contain the origin):
   double _largest_ball_radius;


  public:
   std::vector<unsigned int> hull_vertices; /*!<  the hull vertices (indices to elements of the point set) */

   //! the hull faces (arrays containing indices to elements of the point set).
   std::vector<gpFace> hull_faces;

   gpConvexHull();
   ~gpConvexHull();
   unsigned int nbVertices() {  return hull_vertices.size(); }
   unsigned int nbFaces()    {  return hull_faces.size(); }

   int compute(bool simplicial_facets, bool verbose= true);
   int draw();
   int drawFace(unsigned int face_index);
   int print();
   double largest_ball_radius();
};


//! Derives from gpConvexHull class to deal with 3D points.
//! Adds a new constructor and display functions.
class gpConvexHull3D: public gpConvexHull
{
  public:
   gpConvexHull3D(p3d_vector3 *point_array, unsigned int nb_points);
   int draw();
   int drawFace(unsigned int face_index);
   int getFacePoints(unsigned int face_index, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3);
};


//! Derives from gpConvexHull class to deal with 6D points.
//! Adds a new constructor.
class gpConvexHull6D: public gpConvexHull
{
  public:
   gpConvexHull6D(double (*point_array)[6], unsigned int nb_points);
};

#endif
