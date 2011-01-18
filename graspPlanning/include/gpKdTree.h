
#ifndef GP_KDTREE_H
#define GP_KDTREE_H


//! @defgroup kdTree
//! @ingroup graspPlanning 
//! This module implements some classes to compute the Kd tree of a point set 
//! or of a triangle set.

//! @ingroup kdTree 
//! A class for Axis-Aligned Bounding Box of a point set.
//! It is used in an associated Kd tree class.
class gpAABB
{
  private:
    class gpKdTree *tree_; /*!< the tree the AABB belongs to */
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_; /*!< AABB extreme coordinates */
    bool root_; /*!< boolean to know if the AABB is the root of the tree */
    bool leaf_; /*!< boolean to know if the AABB is a leaf of the tree  */
    unsigned int level_;  /*!< depth level of the AABB */
    class gpAABB *children_[2];  /*!< pointer to the children AABB if not a leaf */
    class gpAABB *brother_;  /*!< pointer to the AABB that has the same parent */
    std::list<unsigned int> inner_points_; /*!< list of the points that are inside the AABB */
  public:
    gpAABB(gpKdTree *tree, std::list<unsigned int> &inner_points); /*!< constructor for root AABB */
    gpAABB(gpAABB *previous, std::list<unsigned int> &inner_points); /*!< constructor for non root AABB */
    ~gpAABB();
    int divide();
    int draw(unsigned int level);
    int sphereIntersection(p3d_vector3 center, double radius, std::list<class gpContact> &points);
};

//! @ingroup kdTree 
//! A class for the Kd tree of a point set.
class gpKdTree
{
 friend class gpAABB; 
 private:
  class gpAABB *root_;
  std::vector<class gpContact> points;
  unsigned int depth_;
 public:
  gpKdTree();
  gpKdTree(std::list<class gpContact> &contactList);
  ~gpKdTree();
  int build(std::list<class gpContact> &contactList);
  int draw(unsigned int level);
  int sphereIntersection(p3d_vector3 center, double radius, std::list<class gpContact> &points);
  unsigned int depth()
  {  return depth_;  }
};

//! @ingroup kdTree 
//! A class for Axis-Aligned Bounding Box of a p3d_polyhedre (a set of triangles).
//! It is used in an associated Kd tree class.
class gpAABBTris
{
  private:
    class gpKdTreeTris *tree_; /*!< the tree the AABB belongs to */
    double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_; /*!< AABB extreme coordinates */
    bool root_; /*!< boolean to know if the AABB is the root of the tree */
    bool leaf_; /*!< boolean to know if the AABB is a leaf of the tree  */
    bool inside_; /*!< boolean to know if the AABB is entirely inside the polyhedron */
    unsigned int level_;  /*!< depth level of the AABB */
    class gpAABBTris *children_[2];  /*!< pointer to the children AABB if not a leaf */
    class gpAABBTris *brother_;  /*!< pointer to the AABB that has the same parent */
    std::list<unsigned int> inner_triangles_; /*!< list of the triangles that are (at least partly) inside the AABB */
  public:
    //! constructor for root AABB
    gpAABBTris(gpKdTreeTris *tree, std::list<unsigned int> &inner_triangles);
    //! constructor for non root AABB
    gpAABBTris(gpAABBTris *previous, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
    bool isTriangleOutside(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3);
    bool isInsidePolyhedre();
    int divide();
    int draw(unsigned int level);
    int sample(double step, std::list<gpVector3D> &points);
};

//! @ingroup kdTree 
//! A class for the Kd tree of a p3d_polyhedre.
class gpKdTreeTris
{
 friend class gpAABBTris; 
 private:
  class gpAABBTris *root_;
  p3d_polyhedre *polyhedron_;
  std::list<unsigned int> inner_triangles_;
  unsigned int depth_;
 public:
  gpKdTreeTris(p3d_polyhedre *polyhedron);
  int draw(unsigned int level);
  unsigned int depth()
  {  return depth_;  }
  int pointCloud(double step, std::list<gpVector3D> &points);
};

#endif

