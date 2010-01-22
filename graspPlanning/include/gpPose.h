#ifndef GP_POSE_H
#define GP_POSE_H


//! @defgroup stablePoseComputation
//! @ingroup graspPlanning 
//! This module implements some classes to compute a set of stable poses for a body.


//! @ingroup graspPlanning 
//! The following enum is used by the gpTriangle class to know
//! how it is described.
typedef enum gpTriangle_description
{
  GP_DESCRIPTION_INDICES, /*!< the triangle is described by the indices of its vertices in a vertex array */
  GP_DESCRIPTION_POINTS,/*!< the triangle is described by the coordinates of its vertices */
  GP_DESCRIPTION_BOTH /*!< the triangle is described by both data types */
} gpTriangle_description;


//! @ingroup stablePoseComputation 
//! A basic class to store triangles in STL containers.
class gpTriangle
{
  public:
   //! a triangle can be described:
   //!  -by indices in a point array (the user must know which one it is):
   unsigned int i1, i2, i3;
   //!  -or directly by the coordinates of its vertices:
   p3d_vector3 p1, p2, p3;
   gpTriangle_description description;

   gpTriangle();
   unsigned int operator [] (unsigned int i) const;
   unsigned int & operator [] (unsigned int i);
   gpTriangle(const gpTriangle &triangle);
   gpTriangle& operator=(const gpTriangle &triangle);
};



//! @ingroup stablePoseComputation 
//! Class containing information about a stable pose of an object (plane of the pose, stability criterion,
//! contact points on the plane).
class gpPose
{
 public:
  int ID;  /*!< ID number */
  p3d_plane plane; /*!< plane of the contact points of the pose */
  p3d_vector3 center; /*!< center of the orthogonal projection of the object's center of mass onto the pose plane */
  double stability;
  p3d_matrix4 T; /*!< transformation to apply to the object (wrt its default configuration) to place its suppport plane horizontally. It will be placed so that the pose center is at position (0,0,0) and the support plane normal is equal too -Z-axis */
  double theta; /*!< rotation angle around the support plane normal (vertical axis once the object has been applied T transformation) */
  p3d_polyhedre *polyhedron;  /*!< surface of the grasped object (must be consistent with the field  "surface" of the contacts)*/
  p3d_obj *object;  /*!< the object */
  std::string object_name;  /*!< name of the object */
  std::vector<gpContact> contacts; 

  gpPose();
  gpPose(const gpPose &pose);
  ~gpPose();
  gpPose & operator=(const gpPose &pose);
  bool operator < (const gpPose &pose);
  bool operator > (const gpPose &pose);
  int print();
  int draw(double length);
  void setPosition(double x, double y, double z);
};

extern int gpCompute_stable_poses(p3d_obj *object, p3d_vector3 cmass, std::list<gpPose> &poseList);

extern int gpFind_poses_on_object(p3d_rob *object, p3d_obj *support, std::list<gpPose> &poseListIn, double translationStep, unsigned int nbDirections, std::list<gpPose> &poseListOut);

extern int gpSample_horizontal_faces(p3d_obj *object, double step, std::list<gpVector3D> &sampleList);


#endif



