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
//! A pose contains a "absolute" transform matrix that is completely object-centered. It is computed once for all.
//! The transform of a pose is the transformation that must be applied to the object (from its default configuration)
//! to place it on the horizontal plane with the face contacting the plane centered on (0,0,0).
//! A pose also contains information about where is placed the object i.e. a position and a angle around the vertical axis.
class gpPose
{
 public:
  int ID;  /*!< ID number */
  p3d_plane plane; /*!< plane of the contact points of the pose */
  p3d_vector3 center; /*!< center of the orthogonal projection of the object's center of mass onto the pose plane */
  double stability;

   //! object-centered transformation of the pose: the transformation to apply to the object (wrt its default configuration) to place its suppport plane horizontally. It will be placed so that the pose center is at position (0,0,0) and the support plane normal is equal to -Z-axis 
  p3d_matrix4 T;

  //! position of the placement
  p3d_vector3 position;
  //! rotation angle around the support plane normal 
  double theta; 


  //! transformation to apply to 
  p3d_matrix4 Trel;

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

extern int gpCompute_stable_poses(p3d_obj *object, std::list<gpPose> &poseList);

extern int gpFind_poses_on_object(p3d_rob *object, p3d_rob *support, std::list<gpPose> &poseListIn, double translationStep, unsigned int nbDirections, std::list<gpPose> &poseListOut);

extern int gpFind_placement_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpPose> &poseList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &hand, double distance, configPt qpreplacement, configPt qplacement);

#endif



