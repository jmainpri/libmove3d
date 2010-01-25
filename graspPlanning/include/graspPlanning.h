#ifndef GRASP_PLANNING_H
#define GRASP_PLANNING_H


//! This file describes some classes used by the module graspPlanning, dedicated to
//! automatic computation of grasp configurations.
//! It also contains numerous symbolic names of joints or bodies.
//! The names contained in .p3d or .macro files must be adapted according to those defined
//! in graspPlanning.h or modify the '#define's in graspPlanning.h.
//! In case the robot has several parts of the same kinds (e.g. two hands, each one having a joints
//! with the symbolic name defined in GP_FOREFINGERJOINT1 i.e. fingerJointForeBase), the names
//! must be suffixed with the number of the part (e.g. there will be fingerJointForeBase_1 and
//! fingerJointForeBase_2).

//./bin/i386-linux/move3d -f ~/BioMove3DDemos/Bauzil/gsSoftMotionDynamicSAHand.p3d

/** @defgroup graspPlanning 
* The grasp planning module is dedicated to
* automatic computation of grasp configurations.
* It can compute a grasp list for a given object (represented by a freeflyer robot with a single body)
* for a given hand type (among the defined ones).
 */


#include <time.h>
#include <sys/times.h>
#include <stdio.h>

#include <vector>
#include <list>
#include <string>
#include <sstream>
#include <libxml2/libxml/xmlreader.h>
#include "P3d-pkg.h"
#include "Graphic-pkg.h"

//debug mode
#ifndef GP_DEBUG
#define GP_DEBUG
#endif

#define GP_OK  0
#define GP_ERROR 1


#define RADTODEG (180.0f/M_PI)
#define DEGTORAD (M_PI/180.0f)

//! Two 3D points whose relative distance is under this value will be regarded as identical:
#define GP_EPSILON 0.00001

//! Contact friction coefficient:
#define GP_FRICTION_COEFFICIENT 0.5


//! Converts a variable to a std::string.
template<class T> std::string convertToString(const T& t)
{
  std::ostringstream stream;
  stream << t;
  return stream.str();
}

//! Names of the gripper's finger joints:
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
// #define GP_OBJECT_NAME "DuploObject"
// #define GP_OBJECT_NAME_DEFAULT "DuploObject"
#define GP_OBJECT_NAME_DEFAULT "Horse"
// #define GP_OBJECT_NAME_DEFAULT "Mug"

//! Symbolic name of the robot (mobile base + arm + gripper/hand)
//! there must be a robot with this name:
#define GP_ROBOT_NAME "ROBOT"

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
#define GP_FINGERTIP_BODY_NAME "fingertip"



//! Inner and outer radii of the ring zone, centered on the object to grasp, where
//! the configurations of the robot's base will be chosen to compute grasp configurations:
#define GP_INNER_RADIUS 0.5
#define GP_OUTER_RADIUS 1.1


#define DBG printf("%s: %d \n", __FILE__, __LINE__)

//! @ingroup graspPlanning 
//! The different hand types:
typedef enum gpHand_type
{
  GP_GRIPPER,
  GP_SAHAND_RIGHT,
  GP_SAHAND_LEFT,
  GP_HAND_NONE
} gpHand_type;

//! @ingroup graspPlanning 
//! The different arm types:
typedef enum gpArm_type
{
  GP_PA10,
  GP_ARM_NONE,
} gpArm_type;


//! @ingroup graspPlanning 
//! This is used to store the result of a collision state for a grasp.
//! Depending on the context, this result can change and a rejected grasp may become valid.
typedef	enum gpGrasp_collision_state
{
  NOT_TESTED,
  COLLIDING,
  COLLISION_FREE
} gpGrasp_collision_state;


//! @ingroup graspPlanning 
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
  unsigned int nb_positions, nb_directions, nb_rotations, max_nb_grasp_frames;


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
  p3d_matrix4 Twrist_finger[4];

  //! joint limits of the four fingers:
  double q0min[4], q0max[4];  /*!< thumb's first joint */
  double q1min[4], q1max[4]; /*!< abduction */
  double q2min[4], q2max[4]; /*!< subduction */
  double q3min[4], q3max[4]; /*!< proximal phalanx/middle phalanx joint */
 
  //! approximation of the finger workspace by a set of spheres:
  std::vector<class gpSphere> workspace; 

  //! "rest" configuration
  double q0rest;  /*!< thumb's first joint */
  double q1rest[4]; /*!< abduction */
  double q2rest[4]; /*!< subduction */
  double q3rest[4]; /*!< proximal phalanx/middle phalanx joint */
  /////////////////////////////////////////////////////////////////////////////////////////////////
  gpHand_properties();
  int initialize(gpHand_type hand_type);
  p3d_rob* initialize();
  int draw(p3d_matrix4 pose);
};

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

  gpContact();
  gpContact(const gpContact &contact);
  gpContact & operator=(const gpContact &contact);
  int draw(double cone_length, int cone_nb_slices= 10);
};


//! @ingroup graspPlanning 
//! This is the class used to describe all the characteristics of a grasp.
class gpGrasp
{
 public:
  int ID;  /*!< ID number */
  double quality;   /*!< quality score of the grasp */
  p3d_matrix4 frame;  /*!< grasp frame */
  std::vector<gpContact> contacts; /*!< vector of contacts of the grasp */
  int handID; /*!< in case there are several hand, this stores the hand used by the grasp. If there is one hand= 0, two hands= 0 and 1 */
  p3d_polyhedre *polyhedron;  /*!< surface of the grasped object (must be consistent with the field  "surface" of the contacts)*/
  p3d_rob *object;  /*!< the grasped object */
  int body_index;  /*!< index of the grasped body in the p3d_obj array of the robot */
  std::string object_name;  /*!< name of the grasped object */
  double finger_opening;  /*!< gripper opening (distance between the jaws)
                          corresponding to the grasp (for GP_GRIPPER hand) */
  gpHand_type hand_type;  /*!< type of the hand realizing the grasp */
  std::vector<double> config; /*!< configuration vector of the hand for the associated grasp */
  gpGrasp_collision_state collision_state; 

  gpGrasp();
  gpGrasp(const gpGrasp &grasp);
  ~gpGrasp();
  gpGrasp & operator = (const gpGrasp &grasp);
  bool operator == (const gpGrasp &grasp);
  bool operator < (const gpGrasp &grasp);
  bool operator > (const gpGrasp &grasp);
  int print();
  int printInFile(const char *filename);
  int draw(double cone_length, int cone_nb_slices= 10);
  double computeQuality();
  double configCost();
  double distance(const gpGrasp &grasp);
};

//! @ingroup graspPlanning
//! This class is used to describe all the characteristics of a double grasp (the object is grasped simultaneously
//! with the two hands).
class gpDoubleGrasp
{
  public:
  int ID;  /*!< ID number */
  gpGrasp grasp1, grasp2;  /*!< the grasps of each hand */
  double quality;   /*!< quality score of the double grasp */

  gpDoubleGrasp();
  gpDoubleGrasp(const gpGrasp &graspA, const gpGrasp &graspB);
  gpDoubleGrasp(const gpDoubleGrasp &dgrasp);
  ~gpDoubleGrasp();
  gpDoubleGrasp & operator = (const gpDoubleGrasp &dgrasp);
  bool operator < (const gpDoubleGrasp &grasp);
  bool operator > (const gpDoubleGrasp &grasp);
  int print();
  int printInFile(const char *filename);
  int draw(double cone_length, int cone_nb_slices= 10);
};

//! @ingroup graspPlanning 
//! A basic class of 3D vectors (that can be used in STL containers unlike p3d_vector3).
class gpVector3D
{
  public:
   double x, y, z;  /*!< point coordinates */
   double cost; /*!< a cost can be used to sort a list of gpVector3Ds */

   gpVector3D()
   {
     x= y= z= 0.0;
   }

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

   bool operator < (const gpVector3D &vector3D)
   {   return (cost < vector3D.cost) ? true : false;   }

   bool operator > (const gpVector3D &vector3D)
   {   return (cost > vector3D.cost) ? true : false;   }

   int set(double x0, double y0, double z0)
   {
      if(this==NULL)
      {
        printf("%s: %d: gpSphere::setCenter(): the calling instance is NULL.\n",__FILE__,__LINE__);
        return GP_ERROR;
      }
      x= x0;
      y= y0;
      z= z0;

      return GP_OK;
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

   void print()
   {
     printf("v= [  %f  %f  %f  ]\n", x, y, z);
   }
};


//! @ingroup graspPlanning 
//! A simple class to store the parameters of a sphere.
class gpSphere
{
 public:
  p3d_vector3 center;
  double radius;

  gpSphere()
  {
    center[0]= center[1]= center[2]= 0.0;
    radius= 0.0;
  }

  gpSphere(const gpSphere &sphere)
  {
    center[0]= sphere.center[0];
    center[1]= sphere.center[1];
    center[2]= sphere.center[2];
    radius   = sphere.radius;
  }

  gpSphere & operator = (const gpSphere &sphere)
  {
    if(this==&sphere) 
    {  return *this;  }
    center[0]= sphere.center[0];
    center[1]= sphere.center[1];
    center[2]= sphere.center[2];
    radius   = sphere.radius;
    return *this;
  }
 
  int setCenter(double x, double y, double z)
  {
    if(this==NULL)
    {
      printf("%s: %d: gpSphere::setCenter(): the calling instance is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
    }
    center[0]= x;
    center[1]= y;
    center[2]= z;
    return GP_OK;
  }
};


//! @ingroup graspPlanning 
//! A basic class to store homogeneous transform matrix (to use in STL containers).
//! Homogeneous transform matrix class:
class gpHTMatrix
{
  public: 
    float m11, m12, m13, m14;
    float m21, m22, m23, m24;
    float m31, m32, m33, m34;

    gpHTMatrix() 
    { 
      m11= m22= m33= 1.0;
      m12= m13= m14= 0.0;
      m21= m23= m24= 0.0;
      m31= m32= m34= 0.0;
    }
    
    void setRotation(p3d_matrix3 R)
    {
      m11= R[0][0];  m12= R[0][1];  m13= R[0][2];
      m21= R[1][0];  m22= R[1][1];  m23= R[1][2];
      m31= R[2][0];  m32= R[2][1];  m33= R[2][2];
    }

    void set(p3d_matrix4 M)
    {
      m11= M[0][0];  m12= M[0][1];  m13= M[0][2];  m14= M[0][3]; 
      m21= M[1][0];  m22= M[1][1];  m23= M[1][2];  m24= M[1][3]; 
      m31= M[2][0];  m32= M[2][1];  m33= M[2][2];  m34= M[2][3]; 
    }

    void copyIn_p3d_matrix4(p3d_matrix4 M)
    {
      M[0][0]= m11;  M[0][1]= m12;  M[0][2]= m13;  M[0][3]= m14; 
      M[1][0]= m21;  M[1][1]= m22;  M[1][2]= m23;  M[1][3]= m24; 
      M[2][0]= m31;  M[2][1]= m32;  M[2][2]= m33;  M[2][3]= m34; 
      M[3][0]= 0.0;  M[3][1]= 0.0;  M[3][2]= 0.0;  M[3][3]= 1.0; 
    }

    void draw()
    { 
      p3d_matrix4 M;
      copyIn_p3d_matrix4(M);
      g3d_draw_frame(M, 0.05);
    }

   void print()
   {
     printf("M= \n");
     printf("[  %f  %f  %f  %f  ]\n", m11, m12, m13, m14);
     printf("[  %f  %f  %f  %f  ]\n", m21, m22, m23, m24);
     printf("[  %f  %f  %f  %f  ]\n", m31, m32, m33, m34);
     printf("[  0    0   0   1  ]\n");
   }
};

//! @ingroup graspPlanning
class gpIndex
{
  public:
   unsigned int index;
   double cost;

   gpIndex()
   {
     index= 0;
     cost= 0;
   }
   bool operator < (const gpIndex &gpIndex)
   {   return (cost < gpIndex.cost) ? true : false;   }

   bool operator > (const gpIndex &gpIndex)
   {   return (cost > gpIndex.cost) ? true : false;   }
};


#endif

