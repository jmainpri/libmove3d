//! This file describes all the classes used by the module graspPlanning, dedicated to
//! automatic computation of grasp configurations.
//! It also contains numerous symbolic names of joints or bodies.
//! The names contained in .p3d or .macro files must be adapted according to those defined
//! in graspPlanning.h or modify the '#define's in graspPlanning.h.


#ifndef GRASP_PLANNING_H
#define GRASP_PLANNING_H


#include <time.h>
#include <sys/times.h>
#include <stdio.h>

#include <vector>
#include <list>
#include <libxml2/libxml/xmlreader.h>
#include "Graphic-pkg.h"

//debug mode
#ifndef DEBUG
#define DEBUG
#endif

#define RADTODEG (180.0f/M_PI)
#define DEGTORAD (M_PI/180.0f)

// Distance en dessous de laquelle on considèrera deux points (3D) comme confondus:
//! Two 3D points whose relative distance is under this value will be regarded as identical:
#define GP_EPSILON 0.00001


//! Contact friction coefficient:
#define GP_FRICTION_COEFFICIENT 0.5



//! The symbolic names of the joints that play a particular role in some computations.
//! The .macro file of the robot must contain joints with these names.
//! In the case of GP_FINGERJOINT, it is the prefix of all the finger joints (e.g.: fingerJoint1, fingerJoint2,
//! etc. or fingerJoint_thumbBase, fingerJoint_forefingerDist, etc.).
#define GP_PLATFORMJOINT    "platformJoint"
#define GP_ARMBASEJOINT      "armBaseJoint"
#define GP_ARMJOINT1            "armJoint1"
#define GP_ARMJOINT2            "armJoint2"
#define GP_ARMJOINT3            "armJoint3"
#define GP_ARMJOINT4            "armJoint4"
#define GP_ARMJOINT5            "armJoint5"
#define GP_WRISTJOINT          "wristJoint"
#define GP_FINGERJOINT        "fingerJoint"
#define GP_FREEFLYERJOINT  "freeflyerJoint"
#define GP_VIRTUAL_OBJECT   "virtual_object"

//! Name of the gripper's joint:
#define GP_GRIPPERJOINT      "fingerJointGripper"


//! Names of the SAHand's finger joints:
#define GP_THUMBJOINT1       "fingerJointThumbRotation"
#define GP_THUMBJOINT2           "fingerJointThumbBase"
#define GP_THUMBJOINT3           "fingerJointThumbProx"
#define GP_THUMBJOINT4            "fingerJointThumbMid"
#define GP_THUMBJOINT5           "fingerJointThumbDist"
#define GP_FOREFINGERJOINT1       "fingerJointForeBase"
#define GP_FOREFINGERJOINT2       "fingerJointForeProx"
#define GP_FOREFINGERJOINT3        "fingerJointForeMid"
#define GP_FOREFINGERJOINT4       "fingerJointForeDist"
#define GP_MIDDLEFINGERJOINT1   "fingerJointMiddleBase"
#define GP_MIDDLEFINGERJOINT2   "fingerJointMiddleProx"
#define GP_MIDDLEFINGERJOINT3    "fingerJointMiddleMid"
#define GP_MIDDLEFINGERJOINT4   "fingerJointMiddleDist"
#define GP_RINGFINGERJOINT1       "fingerJointRingBase"
#define GP_RINGFINGERJOINT2       "fingerJointRingProx"
#define GP_RINGFINGERJOINT3        "fingerJointRingMid"
#define GP_RINGFINGERJOINT4       "fingerJointRingDist"



//! Symbolic name of the object to grasp:
#define GP_OBJECT_NAME "object"
#define GP_OBJECT_NAME_DEFAULT "object"


//! Symbolic name of the robot (mobile base + arm + gripper/hand)
//! there must be a robot with this name:
#define GP_ROBOT_NAME "robot"

//! Symbolic name of the special hand-robot used for the grasp computations;
//! there must be a robot with one of these names:
#define GP_GRIPPER_ROBOT_NAME "gripper_robot"
#define GP_SAHAND_RIGHT_ROBOT_NAME "SAHandRight_robot"
#define GP_SAHAND_LEFT_ROBOT_NAME   "SAHandLeft_robot"


//! Symbolic names of the PA10 arm bodies:
#define GP_PA1  "PA1"
#define GP_PA2  "PA2"
#define GP_PA3  "PA3"
#define GP_PA4  "PA4"
#define GP_PA5  "PA5"
#define GP_PA6  "PA6"
#define GP_PA7  "PA7"


//! The names of the robot's bodies must begin with one of these prefixes in order
//! to know to which part they belong (mobile platform, arm, hand, finger):
#define GP_PLATFORM_BODY_PREFIX "platform"
#define GP_ARM_BODY_PREFIX           "arm"
#define GP_HAND_BODY_PREFIX         "hand"
#define GP_FINGER_BODY_PREFIX     "finger"


//! Prefix of the symbolic names of the fingertip bodies:
#define GP_FINGERTIP "fingertip"



//! Inner and outer radii of the ring zone, centered on the object to grasp, where
//! the configurations of the robot's base will be chosen to compute grasp configurations:
#define GP_INNER_RADIUS 0.5
#define GP_OUTER_RADIUS 1.2


#define DBG printf("%s: %d \n", __FILE__, __LINE__)


//! The different hand types:
typedef enum gpHand_type
{
  GP_GRIPPER,
  GP_SAHAND_RIGHT,
  GP_SAHAND_LEFT,
  GP_HAND_NONE
} gpHand_type;

//! The different arm types:
typedef enum gpArm_type
{
  GP_PA10,
  GP_ARM_NONE,
} gpArm_type;


//Pour une prise, sert à marquer le résultat du calcul de collision.
//Suivant le contexte (obstacles, position du bras par rapport à l'objet) ce résultat
//pourra changer et une prise rejetée pourra devenir valide.
typedef	enum gpGrasp_collision_state
{
  NOT_TESTED,
  COLLIDING,
  COLLISION_FREE
} gpGrasp_collision_state;



//! Class containing information about a given hand (type, dimensions).
//! Some fields will be used or not depending on the kind of hand.
class gpHand_properties
{
 public:
  gpHand_type type;
  unsigned int nb_fingers;

  //! number of active DOFs of the hand
  unsigned int nb_dofs;

  //! radius of the fingertips or approximative half-width of the distal phalanges
  double fingertip_radius;


  //! matrix that will give the hand position from a given grasp frame.
  //! It depends on the convention used for the hand robot model.
  //! grasp_frame*Tgrasp_frame_hand  --> hand_frame
  p3d_matrix4 Tgrasp_frame_hand;

  //! matrix that will give the arm wrist position from a given hand frame.
  //! It depends on how the hand is linked to the arm and on the convention used for the arm's kinematics.
  //! hand_frame*Thand_wrist  -->  arm's wrist frame
  p3d_matrix4 Thand_wrist;


  //! discretization parameters that will be given to the grasp generation function:
  double translation_step, rotation_step;
  unsigned int nb_directions, max_nb_grasp_frames;


  /////////////////////////////////3-fingered gripper (JIDO)//////////////////////////////////
  double fingertip_distance;   /*!< distance between the two first fingers (the ones on the same U-shaped body) */
  double min_opening;          /*!< minimal distance between the gripper's jaws (minimal opening)*/
  double max_opening;          /*!< minimal distance between the gripper's jaws (maximal opening)*/
  double min_opening_jnt_value;    /*!< the value of the gripper joint's DOF when the gripper opening is minimal */
  double max_opening_jnt_value;   /*!< the value of the gripper joint's DOF when the gripper opening is maximal */
  /////////////////////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////SAHAND///////////////////////////////////////////////////
  //! lengths of the thumb's first phalanx and of the proximal, middle et distal phalanges:
  double length_thumbBase, length_proxPha, length_midPha, length_distPha;


  //! transformation matrices wrist frame (= hand's reference frame) -> finger base frame
  p3d_matrix4 Twrist_thumb, Twrist_forefinger, Twrist_middlefinger, Twrist_ringfinger;


  //! joint limits of the four fingers:
  double q0min[4], q0max[4];  /*!< thumb's first joint */
  double q1min[4], q1max[4]; /*!< abduction */
  double q2min[4], q2max[4]; /*!< subduction */
  double q3min[4], q3max[4]; /*!< proximal phalanx/middle phalanx joint */


  /////////////////////////////////////////////////////////////////////////////////////////////////
  gpHand_properties();
  p3d_rob* initialize();
  int draw(p3d_matrix4 pose);
};

//! This class is used to describe the characteristics of the contact points of a grasp.
//! It is also used to describe the contact points of an object pose (class gpPose).
class gpContact
{
 public:
  p3d_polyhedre *surface; /*!<  surface (object) of the contact */
  unsigned int face;    /*!< index of the face (that must be a triangle), in the structure p3d_polyhedre, where the contact is located (starts from 0) */
  unsigned int fingerID;  /*!< ID of the finger that realizes the contact (finger 1, finger 2, etc.)*/
  p3d_vector3 position; /*!<  contact position given in the object's frame */
  p3d_vector3 normal; /*!< surface normal at the contact point (directed outside the object) */
  double mu;         /*!<  friction coefficient of the contact */

  gpContact();
  gpContact(const gpContact &contact);
  gpContact & operator=(const gpContact &contact);
  int draw(double cone_length, int cone_nb_slices= 10);
};

class gpGrasp
{
 public:
  int ID;  /*!< ID number */
  double quality;   /*!< quality score of the grasp */
  p3d_matrix4 frame;  /*!< grasp frame */
  std::vector<gpContact> contacts; /*!< vector of contacts of the grasp */
  p3d_polyhedre *polyhedron;  /*!< surface of the grasped object (must be consistent with the field  "surface" of the contacts)*/
  p3d_obj *object;  /*!< the grasped object */
  std::string object_name;  /*!< name of the grasped object */
  double finger_opening;  /*!< gripper opening (distance between the jaws)
                          corresponding to the grasp (for GP_GRIPPER hand) */
  gpHand_type hand_type;
  std::vector<double> config; /*!< configuration vector of the hand for the associated grasp */
  gpGrasp_collision_state collision_state;

  gpGrasp();
  gpGrasp(const gpGrasp &grasp);
  ~gpGrasp();
  gpGrasp & operator=(const gpGrasp &grasp);
  bool operator < (const gpGrasp &grasp);
  bool operator > (const gpGrasp &grasp);
  void print();
  int printInFile(const char *filename);
  void draw(double cone_length, int cone_nb_slices= 10);
  double computeQuality();
};


//! WIP
typedef enum gpFeature_type
{
  GP_VERTEX,
  GP_EDGE,
  GP_FACE
} gpFeature_type;


//! WIP
class gpPolyhedronFeature
{
 public:
  gpFeature_type type;    /*!< type de primitive (vertex, edge or triangle) */
  p3d_polyhedre *polyhedron;   /*!< pointeur vers le p3d_polyhedre */ 
  std::vector<unsigned int> vertex_indices;   /*!< indices des sommets 
                         dans le tableau de sommets du polyèdre (les indices commencent à 0) */
  p3d_vector3 normals[3];  /*!<  normale(s) de la primitive */

  gpPolyhedronFeature();
  gpPolyhedronFeature(const gpPolyhedronFeature &pf);
  gpPolyhedronFeature & operator=(const gpPolyhedronFeature &pf);
};


typedef enum gpTriangle_description
{
  GP_DESCRIPTION_INDICES,
  GP_DESCRIPTION_POINTS,
  GP_DESCRIPTION_BOTH
} gpTriangle_description;


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

//! A basic class of 3D vectors (that can be used in STL containers unlike p3d_vector3).
class gpVector3D
{
  public:
   double x, y, z;
   double operator [] (unsigned int i) const
   {
      switch(i)
      {
        case 0:
          return x;
        break;
        case 1:
          return y;
        break;
        case 2:
          return z;
        break;
        default:
          printf("gpVector3D::operator []: index exceeds vector dimensions.\n");
          return 0;
        break;
      }
   }

   double& operator [] (unsigned int i)
   {
      switch(i)
      {
        case 0:
          return x;
        break;
        case 1:
          return y;
        break;
        case 2:
          return z;
        break;
        default:
          printf("gpVector3D::operator []: index exceeds vector dimensions.\n");
          return x;
        break;
      }
   }
   void draw(double red, double green, double blue)
   {
     glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POINT_BIT);
     glPointSize(4);
     glDisable(GL_LIGHTING);
     glColor3f(red, green, blue);
     glBegin(GL_POINTS);
      glVertex3f(x, y, z);
     glEnd();
     glPopAttrib();
   }
};

//! WIP
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
  p3d_obj *object;  /*!< the grasped object */
  std::string object_name;  /*!< name of the grasped object */
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

#endif

