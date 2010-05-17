#ifndef GP_GRASP_H
#define GP_GRASP_H

#include "GraspPlanning-pkg.h"

//! @ingroup graspPlanning 
//! This class is used to describe the characteristics of the contact points of a grasp.
//! It is also used to describe the contact points of an object pose (class gpPose).
class gpContact
{
 public:
  unsigned int ID; /*!< ID of the contact */
  p3d_polyhedre *surface; /*!<  surface (object) of the contact */
  unsigned int face;    /*!< index of the face (that must be a triangle), in the structure p3d_polyhedre, where the contact is located (starts from 0) */
  unsigned int fingerID;  /*!< ID (starting from 1) of the finger that realizes the contact (finger 1, finger 2, etc.)*/
  p3d_vector3 position; /*!<  contact position given in the object's frame */
  p3d_vector3 normal; /*!< surface normal at the contact point (directed outside the object) */
  double mu;         /*!<  friction coefficient of the contact */
  p3d_vector3 baryCoords; /*!< barycentric coordinates (defined by the vertices of the triangle) of the contact */
  double curvature; /*!<  curvature of the object surface at the contact point */
 
  gpContact();
  gpContact(const gpContact &contact);
  gpContact & operator=(const gpContact &contact);
  int draw(double cone_length, int cone_nb_slices= 10);
  int computeBarycentricCoordinates();
  int computeCurvature();
};



//! @ingroup graspPlanning 
//! This is the class used to describe all the characteristics of a grasp.
class gpGrasp
{
 public:
  int ID;  /*!< ID number */
  double stability;   /*!< stability score of the grasp (unstable if < 0) */
  double IKscore; /*!< IK score of the grasp (optional) */
  double visibility; /*!< visibility score of the grasp (optional) */
  double quality;   /*!< overall quality score of the grasp */
  p3d_matrix4 frame;  /*!< grasp frame */
  std::vector<gpContact> contacts; /*!< vector of contacts of the grasp */
  int handID; /*!< in case there are several hand, this stores the hand used by the grasp. If there is one hand= 0, two hands= 0 and 1 */
  p3d_rob *object;  /*!< the grasped object */
  int body_index;  /*!< index of the grasped body in the p3d_obj array of the robot */
  std::string object_name;  /*!< name of the grasped object */
  double finger_opening;  /*!< gripper opening (distance between the jaws)
                          corresponding to the grasp (for GP_GRIPPER hand) */
  enum gpHand_type hand_type;  /*!< type of the hand realizing the grasp */
  std::vector<double> config; /*!< configuration vector of the hand for the associated grasp */
  std::vector<double> openConfig; /*!< configuration vector of the hand slightly open from its grasp configuration (is used for the hand approach phase) */
  enum gpGrasp_collision_state collision_state; 

  gpGrasp();
  gpGrasp(const gpGrasp &grasp);
  ~gpGrasp();
  gpGrasp & operator = (const gpGrasp &grasp);
  bool operator < (const gpGrasp &grasp);
  bool operator > (const gpGrasp &grasp);
  int print();
  int printInFile(const char *filename);
  int draw(double cone_length, int cone_nb_slices= 10);
  int computeStability();
  int computeQuality();
  double configCost();
  double distance(const gpGrasp &grasp);
  int contactCentroid(p3d_vector3 centroid);
  int direction(p3d_vector3 direction);
  double similarity(const gpGrasp &grasp);
};

bool gpCompareVisibility(const gpGrasp &grasp1, const gpGrasp &grasp2); 

//! @ingroup graspPlanning
//! This class is used to describe all the characteristics of a double grasp
//! (the object is grasped simultaneously with the two hands).
class gpDoubleGrasp
{
  public:
  int ID;  /*!< ID number */
  gpGrasp grasp1, grasp2;  /*!< the grasps of each hand */
  double  distanceScore; /*!< distance score between the two hands */

  double stability;   /*!< stability score of the double grasp */
  double quality;   /*!< quality score of the double grasp */

  gpDoubleGrasp();
  gpDoubleGrasp(const gpGrasp &graspA, const gpGrasp &graspB);
  gpDoubleGrasp(const gpDoubleGrasp &dgrasp);
  ~gpDoubleGrasp();
  int setFromSingleGrasps(const gpGrasp &graspA, const gpGrasp &graspB);
  gpDoubleGrasp & operator = (const gpDoubleGrasp &dgrasp);
  bool operator < (const gpDoubleGrasp &grasp);
  bool operator > (const gpDoubleGrasp &grasp);
  int print();
  int draw(double cone_length, int cone_nb_slices= 10);
  int computeStability();
  int computeQuality();
  int computeBestObjectOrientation(p3d_matrix4 torsoPose, p3d_matrix4 objectPose);
};

int gpNormalize_distance_score(std::list<gpDoubleGrasp> &list);
int gpNormalize_stability(std::list<gpDoubleGrasp> &list);


#endif

