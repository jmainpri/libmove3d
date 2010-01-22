
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"

#include <math.h>
#include <string>
#include <sstream>

//! @ingroup graspPlanning 
//! Converts a gpHand_type to a std::string.
std::string gpHand_type_to_string(gpHand_type hand_type)
{
  switch(hand_type)
  {
    case GP_GRIPPER:
      return "GP_GRIPPER";
    break;
    case GP_SAHAND_RIGHT:
      return "GP_SAHAND_RIGHT";
    break;
    case GP_SAHAND_LEFT:
      return "GP_SAHAND_LEFT";
    break;
    case GP_HAND_NONE:
      return "GP_HAND_NONE";
    break;
  }

  return "UNDEFINED";
}

//! @ingroup graspPlanning 
//! Converts a gpHand_type to a std::string.
std::string gpHand_type_to_folder_name(gpHand_type hand_type)
{
  switch(hand_type)
  {
    case GP_GRIPPER:
      return "gripper";
    break;
    case GP_SAHAND_RIGHT:
      return "SAHandRight";
    break;
    case GP_SAHAND_LEFT:
      return "SAHandLeft";
    break;
    case GP_HAND_NONE:
      return "undefinedHand";
    break;
  }

  return "undefinedHand";
}

//! @ingroup graspPlanning 
//! Gets the arm base frame of a robot as a 4x4 matrix.
//! The frame is the one of the (fixed) joint that links the arm base body to the mobile base main body.
//! The function finds the joint by its name, that must be the one defined by GP_ARMBASEJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param frame the ouput matrix
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_arm_base_frame(p3d_rob *robot, p3d_matrix4 frame)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_arm_base_frame(): robot is NULL.\n", __FILE__, __LINE__);
    return GP_ERROR;
  }
  #endif

  int i, j;
  p3d_jnt *jnt= NULL;

  jnt= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMBASEJOINT);

  if(jnt==NULL)
  {
    printf("%s: %d: gpGet_arm_base_frame(): robot \"%s\" should have a joint named \"%s\".\n",__FILE__,__LINE__,robot->name, (char*)GP_ARMBASEJOINT);
    return GP_ERROR;
  }

  for(i=0; i<4; i++)
  {
    for(j=0; j<4; j++)
    {
      frame[i][j]= jnt->abs_pos[i][j];
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Gets the platform frame of a robot as a 4x4 matrix.
//! This frame is chosen to be the same as the frame
//! of the joint linking the mobile platform to the environment.
//! This joint is found by its name defined by GP_PLATFORMJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param frame the ouput matrix
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_platform_frame(p3d_rob *robot, p3d_matrix4 frame)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_platform_frame(): robot is NULL.\n", __FILE__, __LINE__);
    return GP_ERROR;
  }
  #endif

  int i, j;
  p3d_jnt *jnt= NULL;

  jnt= p3d_get_robot_jnt_by_name(robot, (char*)GP_PLATFORMJOINT);

  if(jnt==NULL)
  {
    printf("%s: %d: gpGet_platform_frame(): robot \"%s\" should have a joint named \"%s\".\n",__FILE__,__LINE__,robot->name, (char*)GP_PLATFORMJOINT);
    return GP_ERROR;
  }

  for(i=0; i<4; i++)
  {
    for(j=0; j<4; j++)
    {
      frame[i][j]= jnt->abs_pos[i][j];
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Gets the wrist frame of a robot as a 4x4 matrix.
//! This frame is chosen to be the same as the frame
//! of the robot's wrist joint.
//! This joint is found by its name defined by GP_WRISTJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param frame the ouput matrix
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_wrist_frame(p3d_rob *robot, p3d_matrix4 frame)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_wrist_frame(): robot is NULL.\n", __FILE__, __LINE__);
    return GP_ERROR;
  }
  #endif

  int i, j;
  p3d_jnt *jnt= NULL;

  jnt= p3d_get_robot_jnt_by_name(robot, (char*)GP_WRISTJOINT);

  if(jnt==NULL)
  {
    printf("%s: %d: gpGet_wrist_frame(): robot \"%s\" should have a joint named \"%s\".\n",__FILE__,__LINE__,robot->name, (char*)GP_WRISTJOINT);
    return GP_ERROR;
  }

  for(i=0; i<4; i++)
  {
    for(j=0; j<4; j++)
    {
      frame[i][j]= jnt->abs_pos[i][j];
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Gets the hand frame that is associated to a grasp frame for the given hand.
//! \param grasp_frame desired grasp frame
//! \param hand_frame computed hand frame
//! \param hand_properties parameters of the hand
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpHand_frame_from_grasp_frame(p3d_matrix4 grasp_frame, p3d_matrix4 hand_frame, gpHand_properties &hand_properties)
{
  p3d_mat4Mult(grasp_frame, hand_properties.Tgrasp_frame_hand, hand_frame);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Gets the grasp frame that must be associated to a given hand pose (robot hand pose).
//! \param hand_frame desired hand frame
//! \param grasp_frame computed grasp frame
//! \param hand_properties parameters of the hand
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_frame_from_hand_frame(p3d_matrix4 hand_frame, p3d_matrix4 grasp_frame, gpHand_properties &hand_properties)
{
  p3d_matrix4 Tgrasp_frame_hand_inv;

  p3d_matInvertXform(hand_properties.Tgrasp_frame_hand, Tgrasp_frame_hand_inv);

  p3d_mat4Mult(hand_frame, Tgrasp_frame_hand_inv, grasp_frame);

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Gets the grasp frame that must be associated to a given robot's end effector pose.
//! \param end_effector_frame desired end effector frame
//! \param grasp_frame computed grasp frame
//! \param hand_properties parameters of the hand
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_frame_from_end_effector_frame(p3d_matrix4 end_effector_frame, p3d_matrix4 grasp_frame, gpHand_properties &hand_properties)
{
  p3d_matrix4 Tgrasp_frame_hand_inv;
  p3d_matrix4 Thand_wrist_inv, tmp;

  p3d_matInvertXform(hand_properties.Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
  p3d_matInvertXform(hand_properties.Thand_wrist, Thand_wrist_inv);

  p3d_mat4Mult(end_effector_frame, Thand_wrist_inv, tmp);
  p3d_mat4Mult(tmp, Tgrasp_frame_hand_inv, grasp_frame);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Draws a wireframe friction cone with OpenGL functions.
//! \param c cone vertex
//! \param normal cone normal (directed from the vertex to the cone's inside)
//! \param mu friction coefficient
//! \param nb_slices number of segments of the cone discretization
//! \param length length of the cone
void gpDraw_friction_cone(p3d_vector3 c, p3d_vector3 normal, double mu, int nb_slices, double length)
{
  int i;
  p3d_matrix3 R, Ri, Rtmp;
  p3d_vector3 axis, u;
  gpOrthogonal_vector(normal, axis);

  p3d_mat3Rot(R, axis, atan(mu));
  p3d_vec3Mat3Mult(R, normal, u);
  p3d_mat3Rot(Ri, normal, 2*M_PI/nb_slices);

  glLineWidth(1.5);
  glBegin(GL_LINE_LOOP);
   for(i=0; i<nb_slices;i++)
   {
     glVertex3d(c[0] + length*u[0], c[1] + length*u[1], c[2] + length*u[2]);
     p3d_mat3Mult( Ri, R, Rtmp );
     p3d_mat3Copy ( Rtmp, R );
     p3d_vec3Mat3Mult(R, normal, u);
   }
  glEnd();

  glBegin(GL_LINE_STRIP);
   glVertex3d(c[0],c[1],c[2]);
   for(i=0; i<nb_slices;i++)
   {
     glVertex3d(c[0],c[1],c[2]);
     glVertex3d(c[0] + length*u[0], c[1] + length*u[1], c[2] + length*u[2]);
     p3d_mat3Mult( Ri, R, Rtmp );
     p3d_mat3Copy ( Rtmp, R );
     p3d_vec3Mat3Mult(R, normal, u);
   }
  glEnd();
}


//! @ingroup graspPlanning 
//! A different version, that also works.
void gpDraw_friction_cone2(p3d_vector3 c, p3d_vector3 normal, double mu, int nb_slices, double length)
{
  int i;
  p3d_vector3 v, w;
  gpOrthonormal_basis(normal, v, w);

  glLineWidth(1.5);
  double x, y, z;
  double cost, sint;
  glBegin(GL_LINE_LOOP);
   for(i=0; i<nb_slices;i++)
   {
     cost= cos(i*2*M_PI/nb_slices);
     sint= sin(i*2*M_PI/nb_slices);
     x= c[0] + length*normal[0] +  length*(mu*cost*v[0] + mu*sint*w[0]);
     y= c[1] + length*normal[1] +  length*(mu*cost*v[1] + mu*sint*w[1]);
     z= c[2] + length*normal[2] +  length*(mu*cost*v[2] + mu*sint*w[2]);
     glVertex3d(x, y, z);
   }
  glEnd();
  glBegin(GL_LINE_STRIP);
   glVertex3d(c[0],c[1],c[2]);
   for(i=0; i<nb_slices;i++)
   {
     cost= cos(i*2*M_PI/nb_slices);
     sint= sin(i*2*M_PI/nb_slices);
     x= c[0] + length*normal[0] +  length*(mu*cost*v[0] + mu*sint*w[0]);
     y= c[1] + length*normal[1] +  length*(mu*cost*v[1] + mu*sint*w[1]);
     z= c[2] + length*normal[2] +  length*(mu*cost*v[2] + mu*sint*w[2]);
     glVertex3d(c[0],c[1],c[2]);
     glVertex3d(x, y, z);
   }
  glEnd();
}

const int X = 0;
const int Y = 1;
const int Z = 2;

//! @ingroup graspPlanning 
//! Finds a collision-free configuration for the mobile base of a robot in a ring centered on a specified position.
//! The collisions are avoided for the base only. Some of the robot's joints or bodies must have specific names
//! (see graspPlanning.h).
//! NB: modification: the random base configurations are now tested with the arm in folded configuration
//! \param robot pointer to the robot
//! \param innerRadius inner radius of the ring
//! \param outerRadius outer radius of the ring
//! \param objLoc desired center of the ring (world coordinates)
//! \param arm_type type of the robot's arm
//! \return a pointer to the computed robot configuration
configPt gpRandom_robot_base(p3d_rob *robot, double innerRadius, double outerRadius, p3d_vector3 objLoc, gpArm_type arm_type)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpRandom_robot_base(): robot is NULL.\n",__FILE__,__LINE__);
   }
  #endif

  int nb_iter= 0, nb_iter_max= 100;
  int solution_found= 0;
  double x, y, theta, radius;

  configPt result=  p3d_alloc_config(robot);
  configPt q0    =  p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0); //store the current configuration

  gpDeactivate_arm_collisions(robot);
  gpDeactivate_hand_collisions(robot);

  while(nb_iter < nb_iter_max)
  {
    radius= p3d_random(innerRadius, outerRadius);
    theta= p3d_random(0.0, 2*M_PI);

    x= objLoc[X] + radius*cos(theta);
    y= objLoc[Y] + radius*sin(theta);
    // the robot base is rotated in direction of the object plus an angle between -Pi/2 and Pi/2
    theta+= M_PI + p3d_random(-M_PI/2.0, M_PI/2.0);

    gpSet_platform_configuration(robot, x, y, theta);
//     gpFold_arm(robot, arm_type);

    p3d_get_robot_config_into(robot, &result);

    if( !p3d_col_test() )
    {
      solution_found= 1;
      break;
    }
    else
    {  nb_iter++;  }
  }

  gpActivate_arm_collisions(robot);
  gpActivate_hand_collisions(robot);

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);

  if(solution_found==0)
  {
    p3d_destroy_config(robot, result);
    result= NULL;
    printf("%s: %d: gpRandom_robot_base(): no valid configuration was found for the robot mobile base.\n",__FILE__,__LINE__);
  }

  return result;
}


//! @ingroup graspPlanning 
//! Gets the joint angles of the SAHand fingers in its current configuration.
//! \param robot the robot (that must have joint with the appropriate names (see graspPlanning.h))
//! \param hand structure containing information about the hand geometry
//! \param q array that will be filled with the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand, double q[4], int finger_index)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGet_SAHfinger_joint_angles(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpGet_SAHfinger_joint_angles(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return GP_ERROR;
   }
  #endif

  p3d_jnt *joint= NULL;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:

     switch(finger_index)
     {
       case 1: //thumb
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT1);
          q[0]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT2);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT3);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT4);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
       case 2: //forefinger
          q[0]= 0.0;

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT1);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT2);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT3);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
       case 3: //middle finger
          q[0]= 0.0;

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT1);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT2);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT3);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
       case 4: //ring finger
          q[0]= 0.0;

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT1);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT2);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT3);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
     }
    break;
    default:
      printf("%s: %d: gpGet_SAHfinger_joint_angles(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
    break;
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Sets the joint angles of the SAHand fingers and update its current configuration.
//! \param robot the robot (that must have joint with the appropriate names (see graspPlanning.h))
//! \param hand structure containing information about the hand geometry
//! \param q array containing the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand, double q[4], int finger_index)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpSet_SAHfinger_joint_angles(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpSet_SAHfinger_joint_angles(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return GP_ERROR;
   }
  #endif

  p3d_jnt *joint= NULL;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:

     switch(finger_index)
     {
       case 1: //thumb
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT1);
          p3d_jnt_set_dof(joint, 0, q[0]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT2);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT3);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT4);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
       case 2: //forefinger
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT1);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT2);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT3);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
       case 3: //middle finger
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT1);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT2);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT3);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
       case 4: //ring finger
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT1);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT2);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT3);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
     }
    break;
    default:
      printf("%s: %d: gpSet_SAHfinger_joint_angles(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
    break;
  }

  p3d_update_this_robot_pos(robot);

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Computes the forward kinematics of the SAHand fingers.
//! \param Twrist hand pose (frame of the wrist center)
//! \param hand structure containing information about the hand geometry
//! \param q the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param p the computed position of the fingertip center
//! \param fingerpad_normal a vector giving the direction of the fingertip contact surface (orthogonal to the medial axis of the distal phalanx and directed towards the inside of the hand). It is computed for the given finger configuration.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return GP_OK in case of success, GP_ERROR otherwise
//! NB: the first joint of the thumb is not taken into account: it is supposed to be at its maximum value (90 degrees)
//! in opposition to the other fingers.
int gpSAHfinger_forward_kinematics(p3d_matrix4 Twrist, gpHand_properties &hand, double q[4], p3d_vector3 p, p3d_vector3 fingerpad_normal, int finger_index)
{
  #ifdef GP_DEBUG
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpSAHfinger_forward_kinematics(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return GP_ERROR;
   }
  #endif

  double l0, l1, l2, l3;
  double x, y, z;
  p3d_vector3 p_finger, fingerpad_normal_relative;
  p3d_matrix4 Tfinger_world;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
     l0= hand.length_thumbBase;
     l1= hand.length_proxPha;
     l2= hand.length_midPha;
     l3= hand.length_distPha;

     p3d_mat4Mult(Twrist, hand.Twrist_finger[finger_index-1], Tfinger_world);

     x=  -sin(q[1])*(  l1*cos(q[2]) + l2*cos(q[2]+q[3]) + l3*cos(q[2]+2*q[3])  );
     y=   cos(q[1])*(  l1*cos(q[2]) + l2*cos(q[2]+q[3]) + l3*cos(q[2]+2*q[3])  );
     z=  -l1*sin(q[2]) - l2*sin(q[2]+q[3]) - l3*sin(q[2]+2*q[3]);

     if(finger_index!=1)
     {
       p_finger[0]= x;
       p_finger[1]= y;
       p_finger[2]= z;
     }
     else
     {
       //for now, we do not care about the thumb first joint
       p_finger[0]= x;
       p_finger[1]= y;
       p_finger[2]= z;
     }

     p3d_xformPoint(Tfinger_world, p_finger, p);

     fingerpad_normal_relative[0]=   sin(q[1])*(sin(q[2]+2*q[3]));
     fingerpad_normal_relative[1]=  -cos(q[1])*(sin(q[2]+2*q[3]));
     fingerpad_normal_relative[2]=  -cos(q[2]+2*q[3]);

     p3d_xformVect(Tfinger_world, fingerpad_normal_relative, fingerpad_normal);

     g3d_drawSphere( p[0], p[1], p[2],  0.005, Yellow, NULL);
     glLineWidth(3);
     g3d_drawOneLine( p[0], p[1], p[2], p[0]+0.03*fingerpad_normal[0], p[1]+0.03*fingerpad_normal[1], p[2]+0.03*fingerpad_normal[2], Red, NULL);
    break;
    default:
       printf("%s: %d: gpSAHfinger_forward_kinematics(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
    break;
  }

  return GP_OK;
}



//! @ingroup graspPlanning 
//! Computes the inverse kinematics of the SAHand fingers.
//! \param Twrist hand pose (frame of the wrist center) in the world frame
//! \param hand structure containing information about the hand geometry
//! \param p the desired position of the fingertip in the world frame
//! \param q the computed finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param fingerpad_normal a vector giving the direction of the fingertip contact surface (orthogonal to the medial axis of the distal phalanx and directed towards the inside of the hand (outside of the fingertip surface)). It is computed for the computed finger joint angles (if a solution of the inverse kinematics problem exists) and
//! given in the world frame
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return GP_OK in case of success, GP_ERROR otherwise
//! NB: the first joint of the thumb is not taken into account: it is supposed to be at its maximum value (90 degrees)
//! in opposition to the other fingers.
int gpSAHfinger_inverse_kinematics(p3d_matrix4 Twrist, gpHand_properties &hand, p3d_vector3 p, double q[4], p3d_vector3 fingerpad_normal, int finger_index)
{
  #ifdef GP_DEBUG
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpSAHfinger_inverse_kinematics(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return GP_ERROR;
   }
  #endif

  int i, j, nb_solutions;
  double epsilon= 10e-9;
  double epsilonQ= 10e-6;
  double l0, l1, l2, l3;
  double x, y, z, a, b, c, delta, r1, r2;
  p3d_vector3 p_finger, p2;
  p3d_matrix4 Tfinger_world, Tworld_finger;
  double q0min, q0max, q1min, q1max, q2min, q2max, q3min, q3max;
  double q1[2], q2[8], q3[4];
  bool q1_found[2], q2_found[8], q3_found[4];

  q1_found[0]= q1_found[1]= false;
  q2_found[0]= q2_found[1]= q2_found[2]= q2_found[3]= false;
  q2_found[4]= q2_found[5]= q2_found[6]= q2_found[7]= false;
  q3_found[0]= q3_found[1]= q3_found[2]= q3_found[3]= false;

  q[0]= 90; // for the thumb
  q[1]= q[2]= q[3]= 0;
  fingerpad_normal[0]= fingerpad_normal[1]= fingerpad_normal[2]= 0;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
     l0= hand.length_thumbBase;
     l1= hand.length_proxPha;
     l2= hand.length_midPha;
     l3= hand.length_distPha;

     p3d_mat4Mult(Twrist, hand.Twrist_finger[finger_index-1], Tfinger_world);
    break;
    default:
       printf("%s: %d: gpSAHfinger_inverse_kinematics(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
       return GP_ERROR;
    break;
  }

  p3d_matInvertXform(Tfinger_world, Tworld_finger);

  p3d_xformPoint(Tworld_finger, p, p_finger);

  x= p_finger[0];
  y= p_finger[1];
  z= p_finger[2];

  q0min= hand.q0min[finger_index-1];
  q0max= hand.q0max[finger_index-1];
  q1min= hand.q1min[finger_index-1];
  q1max= hand.q1max[finger_index-1];
  q2min= hand.q2min[finger_index-1];
  q2max= hand.q2max[finger_index-1];
  q3min= hand.q3min[finger_index-1];
  q3max= hand.q3max[finger_index-1];

  if( fabs(y) < epsilon )
  {
    if(fabs(x) < epsilon)
    { q1[0]= 0; }
    else
    {
      q1[0]= 0;
      q1_found[0]= true;
    }
  }
  else
  {
    q1[0]= atan(-x/y);
    q1[1]= q1[0] + M_PI;

    if( !isnan(q1[0]) && !isinf(q1[0]) && q1[0]>=(q1min-epsilonQ) && q1[0]<=(q1max+epsilonQ) )
    {
      q1_found[0]= true;
    }

    if( !isnan(q1[1]) && !isinf(q1[1]) && q1[1]>=(q1min-epsilonQ) && q1[1]<=(q1max+epsilonQ) )
    {
      q1_found[1]= true;
    }
  }

  if( !q1_found[0] && !q1_found[1] )
  {  return GP_ERROR;  }

  a= 4*l1*l3;
  b= 2*l1*l2 + 2*l2*l3;
  c= -x*x - y*y - z*z + l1*l1 + l2*l2 + l3*l3 - 2*l1*l3;

  delta= b*b - 4*a*c;

  if(delta < 0)
  {  return GP_ERROR;  }

  r1= ( -b + sqrt(delta) ) / (2*a);
  r2= ( -b - sqrt(delta) ) / (2*a);

  if( fabs(r1) > 1 && fabs(r2) > 1 )
  {  return GP_ERROR;  }

  if( fabs(r1) < 1 )
  {
    q3[0]= acos(r1);
    q3[1]= 2*M_PI - q3[0];

    if( !isnan(q3[0]) && !isinf(q3[0]) && q3[0]>=(q3min-epsilonQ) && q3[0]<=(q3max+epsilonQ) )
    {   q3_found[0]= true;   }

    if( !isnan(q3[1]) && !isinf(q3[1]) && q3[1]>=(q3min-epsilonQ) && q3[1]<=(q3max+epsilonQ) )
    {   q3_found[1]= true;   }

  }
  else
  {
    if(r1==1)
    {   q3[0]= 0;   }

    if(r1==-1)
    {   q3[0]= M_PI;  }

    if( !isnan(q3[0]) && !isinf(q3[0]) && q3[0]>=(q3min-epsilonQ) && q3[0]<=(q3max+epsilonQ) )
    {  q3_found[0]= true;  }

  }

  if( fabs(r2) < 1 )
  {
    q3[2]= acos(r2);
    q3[3]= 2*M_PI - q3[2];

    if( !isnan(q3[2]) && !isinf(q3[2]) && q3[2]>=(q3min-epsilonQ) && q3[2]<=(q3max+epsilonQ) )
    {   q3_found[2]= true;   }

    if( !isnan(q3[3]) && !isinf(q3[3]) && q3[3]>=(q3min-epsilonQ) && q3[3]<=(q3max+epsilonQ) )
    {   q3_found[3]= true;   }

  }
  else
  {
    if(r2==1)
    {   q3[2]= 0;   }

    if(r2==-1)
    {   q3[2]= M_PI;  }

    if( !isnan(q3[2]) && !isinf(q3[2]) && q3[2]>=(q3min-epsilonQ) && q3[2]<=(q3max+epsilonQ) )
    {  q3_found[2]= true;  }

  }


  if( !q3_found[0] && !q3_found[1] && !q3_found[2] && !q3_found[3] )
  {  return GP_ERROR;  }

  for(i=0; i<4; i++)
  {
    if(q3_found[i]==false)
    {   continue;    }

    a= l2*sin(q3[i]) + l3*sin(2*q3[i]);
    b= l1 + l2*cos(q3[i]) + l3*cos(2*q3[i]);
    c= -z;

    nb_solutions= solve_trigonometric_equation(a, b, c, &q2[2*i], &q2[2*i+1]);

    switch(nb_solutions)
    {
      case 1:
        if( !isnan(q2[2*i]) && !isinf(q2[2*i]) && q2[2*i]>=(q2min-epsilonQ) && q2[2*i]<=(q2max+epsilonQ) )
        {  q2_found[2*i]= true;  }
      break;
      case 2:
        if( !isnan(q2[2*i]) && !isinf(q2[2*i]) && q2[2*i]>=(q2min-epsilonQ) && q2[2*i]<=(q2max+epsilonQ) )
        {  q2_found[2*i]= true;  }

        if( !isnan(q2[2*i+1]) && !isinf(q2[2*i+1]) && q2[2*i+1]>=(q2min-epsilonQ) && q2[2*i+1]<=(q2max+epsilonQ) )
        {  q2_found[2*i+1]= true;  }
      break;
      default:
      break;
    }
  }


  for(i=0; i<2; i++)
  {
    for(j=0; j<4; j++)
    {
      if( q1_found[i] && q3_found[j] && q2_found[2*j] )
      {
        q[0]= q0max;
        q[1]= q1[i];
        q[2]= q2[2*j];
        q[3]= q3[j];

        gpSAHfinger_forward_kinematics(Twrist, hand, q, p2, fingerpad_normal, finger_index);
        if( sqrt( SQR(p2[0]-p[0]) + SQR(p2[1]-p[1]) + SQR(p2[2]-p[2]) ) < 1e-3)
        { return GP_OK; }

      }
      if( q1_found[i] && q3_found[j] && q2_found[2*j+1] )
      {
        q[0]= q0max;
        q[1]= q1[i];
        q[2]= q2[2*j+1];
        q[3]= q3[j];

        gpSAHfinger_forward_kinematics(Twrist, hand, q, p2, fingerpad_normal, finger_index);
        if( sqrt( SQR(p2[0]-p[0]) + SQR(p2[1]-p[1]) + SQR(p2[2]-p[2]) ) < 1e-3)
        { return GP_OK; }
      }
    }
  }

  return GP_ERROR;
}


//! @ingroup graspPlanning 
//! Deactivates the collision tests between an object and the fingertips of a robot (that has some).
//! \param robot the robot (its fingertip bodies must have specific names, defined in graspPlanning.h)
//! \param object the object
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDeactivate_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_object_fingertips_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
   if(object==NULL)
   {
      printf("%s: %d: gpDeactivate_object_fingertips_collisions(): object is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  static bool already_warned= false;
  unsigned int i;
  std::string base_name, body_name;
  std::stringstream out;
  p3d_obj *fingertip;

  base_name= std::string(GP_HAND_BODY_PREFIX) + std::string(".") +std::string( GP_FINGER_BODY_PREFIX);

  for(i=1; i<=hand.nb_fingers; i++)
  {
     body_name= base_name;
     out << i;
     body_name+= out.str();
     body_name+= std::string(".") + GP_FINGERTIP_BODY_NAME;
     out.seekp(std::ios::beg);

     fingertip= NULL;
     fingertip= p3d_get_robot_body_by_name(robot, (char *) body_name.c_str());
     if(fingertip==NULL)
     {
       if(already_warned==false)
       {
         already_warned= true;
         printf("%s: %d: gpDeactivate_object_fingertips_collisions(): robot \"%s\" should have a body named \"%s\".\n",__FILE__, __LINE__, robot->name, body_name.c_str());
         printf(" More problems of this kind may have occured.\n");
       }
     }
     else
     { 
       p3d_col_deactivate_pair_of_objects(fingertip, object); 
     }
  }

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Activates the collision tests between an object and the fingertips of a robot (that has some).
//! \param robot the robot (its fingertip bodies must have specific names, defined in graspPlanning.h)
//! \param object the object
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpActivate_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpAactivate_object_fingertips_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
   if(object==NULL)
   {
      printf("%s: %d: gpAactivate_object_fingertips_collisions(): object is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  unsigned int i;
  std::string base_name, body_name;
  std::stringstream out;
  p3d_obj *fingertip;

  base_name= std::string(robot->name) + std::string(".") + std::string(GP_HAND_BODY_PREFIX) + std::string(".") + std::string(GP_FINGER_BODY_PREFIX);

  for(i=1; i<=hand.nb_fingers; i++)
  {
     body_name= base_name;
     out << i;
     body_name+= out.str();
     body_name+= std::string(".") + std::string(GP_FINGERTIP_BODY_NAME);
     out.seekp(std::ios::beg);

     fingertip= NULL;
     fingertip= p3d_get_body_by_name((char *) body_name.c_str());
     if(fingertip==NULL)
     {
       printf("%s: %d: gpActivate_object_fingertips_collisions(): robot \"%s\" should have a body named \"%s\".\n",__FILE__,__LINE__, robot->name, body_name.c_str());
     }
     p3d_col_activate_pair_of_objects(fingertip, object);
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Checks if the fingertips of the robot (that has some) are in collision with the object.
//! \param robot the robot (its fingertip bodies must have specific names, defined in graspPlanning.h)
//! \param object the object
//! \param hand structure containing information about the hand geometry
//! \return the number of fingertips in contact with the object, 0 in case of errror
int gpCount_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpCheck_object_fingertips_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
   if(object==NULL)
   {
      printf("%s: %d: gpCheck_object_fingertips_collisions(): object is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  unsigned int i;
  int nb_contacts;
  std::string base_name, body_name;
  std::stringstream out;
  p3d_obj *fingertip;

  base_name= std::string(robot->name) + std::string(".") + GP_HAND_BODY_PREFIX + std::string(".") + GP_FINGER_BODY_PREFIX;

  nb_contacts= 0;
  for(i=1; i<=hand.nb_fingers; i++)
  {
     body_name= base_name;
     out << i;
     body_name+= out.str();
     body_name+= std::string(".") + std::string(GP_FINGERTIP_BODY_NAME);
     out.seekp(std::ios::beg);

     fingertip= NULL;
     fingertip= p3d_get_body_by_name((char *) body_name.c_str());
     if(fingertip==NULL)
     {
       printf("%s: %d: gpCheck_object_fingertips_collisions(): robot \"%s\" should have a body named \"%s\".\n",__FILE__,__LINE__, robot->name, body_name.c_str());
       continue;
     }
printf("test %s vs %s \n",fingertip->name, object->name);
//      if(p3d_col_test_obj_obj(fingertip, object)) MODIF XAV
			 if(p3d_col_test_pair(fingertip, object))
		 {
       nb_contacts++;
     }
  }

  return nb_contacts;
}

//! @ingroup graspPlanning 
//! Opens the gripper or hand at its maximum.
//! \param robot the robot (its joints must have specific names, defined in graspPlanning.h)
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpOpen_hand(p3d_rob *robot, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpOpen_gripper(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  std::vector<double> q;

  q.resize(hand.nb_dofs);

  switch(hand.type)
  {
    case GP_GRIPPER:
      q[0]= hand.max_opening_jnt_value;
    break;
//! warning: in the following the SAHand joint values should not be their maximal bounds:
    case  GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      q[0]= hand.q0max[0];
      q[1]= hand.q1max[0];
      q[2]= hand.q2max[0];
      q[3]= hand.q3max[0];

      q[4]= hand.q1max[1];
      q[5]= hand.q2max[1];
      q[6]= hand.q3max[1];

      q[7]= hand.q1max[2];
      q[8]= hand.q2max[2];
      q[9]= hand.q3max[2];

      q[10]= hand.q1max[3];
      q[11]= hand.q2max[3];
      q[12]= hand.q3max[3];
    break;
    default:
     printf("%s: %d: gpOpen_hand(): unsupported hand type.\n",__FILE__,__LINE__);
     return GP_ERROR;
    break;
  }

  gpSet_hand_configuration(robot, hand , q);

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Closes the gripper or hand at its maximum.
//! \param robot the robot (its joints must have specific names, defined in graspPlanning.h)
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpClose_hand(p3d_rob *robot, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpClose_hand(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  std::vector<double> q;

  q.resize(hand.nb_dofs);

  switch(hand.type)
  {
    case GP_GRIPPER:
      q[0]= hand.min_opening_jnt_value;
    break;
//! warning: in the following the SAHand joint values should not be their minimal bounds:
    case  GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      q[0]= hand.q0min[0];
      q[1]= hand.q1min[0];
      q[2]= hand.q2min[0];
      q[3]= hand.q3min[0];

      q[4]= hand.q1min[1];
      q[5]= hand.q2min[1];
      q[6]= hand.q3min[1];

      q[7]= hand.q1min[2];
      q[8]= hand.q2min[2];
      q[9]= hand.q3min[2];

      q[10]= hand.q1min[3];
      q[11]= hand.q2min[3];
      q[12]= hand.q3min[3];
    break;
    default:
     printf("%s: %d: gpClose_hand(): unsupported hand type.\n",__FILE__,__LINE__);
     return GP_ERROR;
    break;
  }

  gpSet_hand_configuration(robot, hand , q);

  return GP_OK;
}

int gpClose_gripper_until_collision(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpClose_gripper_until_collision(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpClose_gripper_until_collision(): object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(hand.type!=GP_GRIPPER)
   {
     printf("%s: %d: gpClose_gripper_until_collision(): this function only applies to GP_GRIPPER.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  unsigned int i, n= 30, nb_contacts;
  double alpha;
  std::vector<double> q0;
  std::vector<double> q;
  std::vector<double> qprev;

  q0.resize(hand.nb_dofs);
  q.resize(hand.nb_dofs);
  qprev.resize(hand.nb_dofs);

  gpGet_hand_configuration(robot, hand, q0);

  if(p3d_col_test_robot_obj(robot, object))
  {
    return GP_ERROR;
  }

  nb_contacts= 0;
  for(i=0; i<n; ++i)
  {
    alpha= ((double) i)/((double) n);
    q[0]= (1-alpha)*q0[0] + alpha*hand.min_opening_jnt_value;

    gpSet_hand_configuration(robot, hand , q);

    if(p3d_col_test_robot_obj(robot, object))
    {
      nb_contacts= gpCount_object_fingertips_collisions(robot, object, hand);
      gpSet_hand_configuration(robot, hand , qprev);
      return nb_contacts;
    }
  }

   gpSet_hand_configuration(robot, hand , q0);

  return GP_ERROR;
}

//! @ingroup graspPlanning 
//! Gets the robot's platform configuration (x,y,theta).
//! The robot must have a joint of type P3D_FREEFLYER or P3D_PLAN with
//! the name defined in GP_PLATFORMJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param x where to copy the current x position
//! \param y where to copy the current y position
//! \param theta where to copy the current theta position
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_platform_configuration(p3d_rob *robot, double &x, double &y, double &theta)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGet_platform_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  int indexX, indexY, indexTheta;
  p3d_jnt *platformJoint= NULL;

  platformJoint= p3d_get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  if(platformJoint==NULL)
  {  return GP_ERROR; }


  switch(platformJoint->type)
  {
    case P3D_FREEFLYER:
        indexX    =  0;
        indexY    =  1;
        indexTheta=  5;
    break;
    case P3D_PLAN:
        indexX    =  0;
        indexY    =  1;
        indexTheta=  2;
    break;
    default:
      printf("%s: %d: gpGet_platform_configuration(): the platform joint must be of type P3D_FREEFLYER or P3D_PLAN.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  x= platformJoint->dof_data[indexX].v;
  y= platformJoint->dof_data[indexY].v;
  theta= platformJoint->dof_data[indexTheta].v;

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Sets the robot's platform configuration to the given values.
//! The robot must have a joint of type P3D_FREEFLYER or P3D_PLAN with
//! the name defined in GP_PLATFORMJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param x desired X position
//! \param y desired Y position
//! \param theta desired orientation around Z-axis
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_platform_configuration(p3d_rob *robot, double x, double y, double theta)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpSet_platform_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  int indexX, indexY, indexTheta;
  double x_min, x_max, y_min, y_max, theta_min, theta_max;
  configPt q= NULL;
  p3d_jnt *platformJoint= NULL;

  platformJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_PLATFORMJOINT);

  if(platformJoint==NULL)
  {  return GP_ERROR; }

  switch(platformJoint->type)
  {
    case P3D_FREEFLYER:
        indexX    =  0;
        indexY    =  1;
        indexTheta=  5;
    break;
    case P3D_PLAN:
        indexX    =  0;
        indexY    =  1;
        indexTheta=  2;
    break;
    default:
      printf("%s: %d: gpSet_platform_configuration(): the platform joint must be of type P3D_FREEFLYER or P3D_PLAN.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  x_min= platformJoint->dof_data[indexX].vmin;
  x_max= platformJoint->dof_data[indexX].vmax;
  y_min= platformJoint->dof_data[indexY].vmin;
  y_max= platformJoint->dof_data[indexY].vmax;
  theta_min= platformJoint->dof_data[indexTheta].vmin;
  theta_max= platformJoint->dof_data[indexTheta].vmax;

  if( (x < x_min) || (x > x_max) )
  {
    printf("%s: %d: gpSet_platform_configuration(): desired x value (%f) is out of bounds (%f %f).\n",__FILE__,__LINE__, x, x_min, x_max);
    printf("Its value will not be changed.\n");
    x= platformJoint->dof_data[indexX].v;
  }
  if( (y < y_min) || (y > y_max) )
  {
    printf("%s: %d: gpSet_platform_configuration(): desired y value (%f) is out of bounds (%f %f).\n",__FILE__,__LINE__, y, y_min, y_max);
    printf("Its value will not be changed.\n");
    y= platformJoint->dof_data[indexY].v;
  }

  theta= fmod(theta, 2*M_PI);
  if(theta < theta_min)
  {
    theta+= 2*M_PI;
  }
  if(theta > theta_max)
  {
    theta-= 2*M_PI;
  }

  if( (theta < theta_min) || (theta > theta_max) )
  {
    printf("%s: %d: gpSet_platform_configuration(): desired theta value (%f) is out of bounds (%f %f).\n",__FILE__,__LINE__, theta, theta_min, theta_max);
    printf("Its value will not be changed.\n");
    theta= platformJoint->dof_data[indexTheta].v;
  }

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);

  q[platformJoint->index_dof + indexX]    = x;
  q[platformJoint->index_dof + indexY]    = y;
  q[platformJoint->index_dof + indexTheta]= theta;
  p3d_set_and_update_this_robot_conf(robot, q);

  p3d_destroy_config(robot, q);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Gets the robot's arm configuration (in radians).
//! \param robot pointer to the robot
//! \param arm_type arm type (for now, only PA10 is supported)
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, double &q1, double &q2, double &q3, double &q4, double &q5, double &q6)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpGet_arm_configuration(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  p3d_jnt *armJoint= NULL;

  switch(arm_type)
  {
    case GP_PA10:
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT1);
        if(armJoint==NULL)
        {  return GP_ERROR; }
        q1= armJoint->dof_data[0].v;

        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT2);
        if(armJoint==NULL)
        {  return GP_ERROR; }
        q2= armJoint->dof_data[0].v;

        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT3);
        if(armJoint==NULL)
        {  return GP_ERROR; }
        q3= armJoint->dof_data[0].v;

        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT4);
        if(armJoint==NULL)
        {  return GP_ERROR; }
        q4= armJoint->dof_data[0].v;

        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT5);
        if(armJoint==NULL)
        {  return GP_ERROR; }
        q5= armJoint->dof_data[0].v;

        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_WRISTJOINT);
        if(armJoint==NULL)
        {  return GP_ERROR; }
        q6= armJoint->dof_data[0].v;
    break;
    default:
      printf("%s: %d: gpGet_arm_configuration(): unsupported arm type.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Sets the robot's arm configuration with the given values (in radians).
//! NB: The respect of joint limits is verified.
//! \param robot pointer to the robot
//! \param arm_type arm type (for now, only PA10 is supported)
//! \param q1 value of joint #1
//! \param q2 value of joint #2
//! \param q3 value of joint #3
//! \param q4 value of joint #4
//! \param q5 value of joint #5
//! \param q6 value of joint #6
//! \param verbose enable/disable error message display
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, double q1, double q2, double q3, double q4, double q5, double q6, bool verbose)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpSet_arm_configuration(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  bool isValid;
  double qmin, qmax;
  p3d_jnt *armJoint= NULL;
  configPt q= NULL;

  #ifdef LIGHT_PLANNER
  deactivateCcCntrts(robot, -1);
  #endif

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);

  switch(arm_type)
  {
    case GP_PA10:
        ////////////////////////q1////////////////////////////
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT1);
        if(armJoint==NULL)
        {
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        isValid= true;
        qmin= armJoint->dof_data[0].vmin;
        qmax= armJoint->dof_data[0].vmax;
        if(q1 > qmax)
        {
          q1-= 2*M_PI;
          if( (q1 < qmin) || (q1 > qmax) )
          {  isValid= false; }
        }
        if(q1 < qmin)
        {
          q1+= 2*M_PI;
          if( (q1 < qmin) || (q1 > qmax) )
          {  isValid= false; }
        }
        if(isValid)
        { q[armJoint->index_dof]=  q1;   }
        else
        {
          if(verbose)
          {
             printf("%s: %d: gpSet_arm_configuration(): q1 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q1,qmin,qmax);
          }
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        /////////////////////////////////////////////////////


        ////////////////////////q2////////////////////////////
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT2);
        if(armJoint==NULL)
        {
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        isValid= true;
        qmin= armJoint->dof_data[0].vmin;
        qmax= armJoint->dof_data[0].vmax;
        if(q2 > qmax)
        {
          q2-= 2*M_PI;
          if( (q2 < qmin) || (q2 > qmax) )
          {  isValid= false; }
        }
        if(q2 < qmin)
        {
          q2+= 2*M_PI;
          if( (q2 < qmin) || (q2 > qmax) )
          {  isValid= false; }        }
        if(isValid)
        { q[armJoint->index_dof]=  q2;   }
        else
        {
          if(verbose)
          {
             printf("%s: %d: gpSet_arm_configuration(): q2 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q2,qmin,qmax);
          }
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        /////////////////////////////////////////////////////


        ////////////////////////q3////////////////////////////
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT3);
        if(armJoint==NULL)
        {
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        isValid= true;
        qmin= armJoint->dof_data[0].vmin;
        qmax= armJoint->dof_data[0].vmax;
        if(q3 > qmax)
        {
          q3-= 2*M_PI;
          if( (q3 < qmin) || (q3 > qmax) )
          {  isValid= false; }
        }
        if(q3 < qmin)
        {
          q3+= 2*M_PI;
          if( (q3 < qmin) || (q3 > qmax) )
          {  isValid= false; }
        }
        if(isValid)
        { q[armJoint->index_dof]=  q3;   }
        else
        {
          if(verbose)
          {
             printf("%s: %d: gpSet_arm_configuration(): q3 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q3,qmin,qmax);
          }
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        /////////////////////////////////////////////////////


        ////////////////////////q4////////////////////////////
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT4);
        if(armJoint==NULL)
        {
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        isValid= true;
        qmin= armJoint->dof_data[0].vmin;
        qmax= armJoint->dof_data[0].vmax;
        if(q4 > qmax)
        {
          q4-= 2*M_PI;
          if( (q4 < qmin) || (q4 > qmax) )
          {  isValid= false; }
        }
        if(q4 < qmin)
        {
          q4+= 2*M_PI;
          if( (q4 < qmin) || (q4 > qmax) )
          {  isValid= false; }
        }
        if(isValid)
        { q[armJoint->index_dof]=  q4;   }
        else
        {
          if(verbose)
          {
             printf("%s: %d: gpSet_arm_configuration(): q4 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q4,qmin,qmax);
          }
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        /////////////////////////////////////////////////////


        ////////////////////////q5////////////////////////////
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT5);
        if(armJoint==NULL)
        {
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        isValid= true;
        qmin= armJoint->dof_data[0].vmin;
        qmax= armJoint->dof_data[0].vmax;
        if(q5 > qmax)
        {
          q5-= 2*M_PI;
          if( (q5 < qmin) || (q5 > qmax) )
          {  isValid= false; }
        }
        if(q5 < qmin)
        {
          q5+= 2*M_PI;
          if( (q5 < qmin) || (q5 > qmax) )
          {  isValid= false; }
        }
        if(isValid)
        { q[armJoint->index_dof]=  q5;   }
        else
        {
          if(verbose)
          {
             printf("%s: %d: gpSet_arm_configuration(): q5 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q5,qmin,qmax);
          }
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        /////////////////////////////////////////////////////


        ////////////////////////q6////////////////////////////
        armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_WRISTJOINT);
        if(armJoint==NULL)
        {
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
        isValid= true;
        qmin= armJoint->dof_data[0].vmin;
        qmax= armJoint->dof_data[0].vmax;
        if(q6 > qmax)
        {
          q6-= 2*M_PI;
          if( (q6 < qmin) || (q6 > qmax) )
          {  isValid= false; }
        }
        if(q6 < qmin)
        {
          q6+= 2*M_PI;
          if( (q6 < qmin) || (q6 > qmax) )
          {  isValid= false; }
        }
        if(isValid)
        { q[armJoint->index_dof]=  q6;   }
        else
        {
          if(verbose)
          {
             printf("%s: %d: gpSet_arm_configuration(): q6 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q6,qmin,qmax);
          }
          p3d_destroy_config(robot, q);
          return GP_ERROR;
        }
    break;
    default:
      printf("%s: %d: gpSet_arm_configuration(): unsupported arm type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot, q);
      return GP_ERROR;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, q);
  p3d_destroy_config(robot, q);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot with the configuration contained in a gpGrasp variable.
//! It only modifies the parameters of the hand.
//! \param robot pointer to the robot
//! \param hand information concerning the hand
//! \param grasp the grasp to set
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_grasp_configuration(p3d_rob *robot, gpHand_properties &hand, const gpGrasp &grasp)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_grasp_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  p3d_jnt *fingerJoint= NULL;

  if(grasp.config.size()!=hand.nb_dofs)
  {
    printf("%s: %d: gpSet_grasp_configuration(): the configuration vector of the input grasp has a bad size (%d instead of %d).\n",__FILE__,__LINE__,grasp.config.size(), hand.nb_dofs);
    return GP_ERROR;
  }

  configPt q= NULL;
  q= p3d_get_robot_config(robot);
  switch(hand.type)
  {
    case GP_GRIPPER:
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[0];
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[0];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[1];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[2];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT4);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[3];

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[4];

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[5];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[6];

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[7];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[8];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[9];

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[10];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[11];
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR; }
      q[fingerJoint->index_dof]= grasp.config[12];
    break;
    default:
       printf("%s: %d: gpSet_grasp_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, q);
  p3d_copy_config_into(robot, q, &robot->ROBOT_POS);
  p3d_destroy_config(robot, q);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Gets the hand/gripper's configuration of a robot and copies it in a std::vector.
//! \param robot pointer to the robot
//! \param hand information about the hand
//! \param q a std::vector that will be filled with the current joint parameters of the hand
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_hand_configuration(p3d_rob *robot, gpHand_properties &hand, std::vector<double> q)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_hand_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  p3d_jnt *fingerJoint= NULL;

  q.resize(hand.nb_dofs);

  switch(hand.type)
  {
    case GP_GRIPPER:
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[0]= fingerJoint->dof_data[0].v;
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[0]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[1]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[2]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT4);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[3]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[4]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[5]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[6]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[7]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[8]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[9]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[10]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[11]= fingerJoint->dof_data[0].v;

      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[12]= fingerJoint->dof_data[0].v;
    break;
    default:
      printf("%s: %d: gpGet_hand_configuration(): unsupported hand type.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Sets the hand/gripper's configuration of a robot with the configuration contained in a std::vector.
//! It only modifies the parameters of the hand.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_hand_configuration(p3d_rob *robot, gpHand_properties &hand, std::vector<double> q, bool verbose)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_grasp_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  unsigned int i;
  bool isValid;
  double qmin, qmax;
  p3d_jnt *fingerJoint= NULL;

  if(q.size()!=hand.nb_dofs)
  {
    printf("%s: %d: gpSet_hand_configuration(): the input configuration vector has a bad size (%d instead of %d).\n",__FILE__,__LINE__,q.size(), hand.nb_dofs);
    return GP_ERROR;
  }

  configPt qcur= NULL;
  qcur= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &qcur);

  switch(hand.type)
  {
    case GP_GRIPPER:
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {
        p3d_destroy_config(robot, qcur);
        return GP_ERROR;
      }
      qmin= fingerJoint->dof_data[0].vmin;
      qmax= fingerJoint->dof_data[0].vmax;
      if( q[0]<qmin || q[0]>qmax )
      {
         if(verbose)
         {
            printf("%s: %d: gpSet_hand_configuration(): q[0] value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q[0],qmin,qmax);
         }
         p3d_destroy_config(robot, qcur);
         return GP_ERROR;
      }
      qcur[fingerJoint->index_dof]= q[0];
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      for(i=0; i<12; i++)
      {
        switch(i)
        {
          case 0: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT1); break;
          case 1: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT2); break;
          case 2: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT3); break;
          case 3: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_THUMBJOINT4); break;
          case 4: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT1); break;
          case 5: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT2); break;
          case 6: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_FOREFINGERJOINT3); break;
          case 7: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT1); break;
          case 8: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT2); break;
          case 9: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_MIDDLEFINGERJOINT3); break;
          case 10: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT1); break;
          case 11: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT2); break;
          case 12: fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_RINGFINGERJOINT3); break;
        }
        if(fingerJoint==NULL)
        {
          p3d_destroy_config(robot, qcur);
          return GP_ERROR;
        }
        isValid= true;
        qmin= fingerJoint->dof_data[0].vmin;
        qmax= fingerJoint->dof_data[0].vmax;
        if(q[i] > qmax)
        {
          q[i]-= 2*M_PI;
          if( (q[i] < qmin) || (q[i] > qmax) )
          {  isValid= false; }
        }
        if(q[i] < qmin)
        {
          q[i]+= 2*M_PI;
          if( (q[i] < qmin) || (q[i] > qmax) )
          {  isValid= false; }
        }
        if(!isValid)
        {
          if(verbose)
          {
            printf("%s: %d: gpSet_hand_configuration(): q[%d] value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,i,q[i],qmin,qmax);
          }
          p3d_destroy_config(robot, qcur);
          return GP_ERROR;
        }
        qcur[fingerJoint->index_dof]= q[i];
      }
    break;
    default:
       printf("%s: %d: gpSet_grasp_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, qcur);
  p3d_destroy_config(robot, qcur);

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Sets the robot's arm to a "folded" configuration so that it takes the less room
//! (when the base is moving for instance).
//! \param robot pointer to the robot
//! \param arm_type arm type (for now, only PA10 is supported)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFold_arm(p3d_rob *robot, gpArm_type arm_type)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpFold_arm(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int result;
  double q1, q2, q3, q4, q5, q6;
  configPt q0= NULL;

  q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0);

  #ifdef LIGHT_PLANNER
  if(robot->openChainConf!=NULL)
  {
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, robot->openChainConf);
    p3d_set_and_update_this_robot_conf(robot, robot->openChainConf);
    gpGet_arm_configuration(robot, arm_type, q1, q2, q3, q4, q5, q6);
    p3d_set_and_update_this_robot_conf(robot, q0);
  }
  #else
    //for horizontal jido:
    q1= DEGTORAD*(-90);
    q2= DEGTORAD*(90);
    q3= DEGTORAD*(45);
    q4= DEGTORAD*(0);
    q5= DEGTORAD*(-45);
    q6= DEGTORAD*(0);

    //for vertical jido:
    q1= DEGTORAD*(77.15);
    q2= DEGTORAD*(-8.99);
    q3= DEGTORAD*(148.39);
    q4= DEGTORAD*(80.17);
    q5= DEGTORAD*(-81.68);
    q6= DEGTORAD*(-18.51);
  #endif


  switch(arm_type)
  {
    case GP_PA10:
      result= gpSet_arm_configuration(robot, GP_PA10, q1, q2, q3, q4, q5, q6);
    break;
    default:
      printf("%s: %d: gpFold_arm(): unsupported arm type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot, q0);
      return GP_ERROR;
    break;
  }

  if(p3d_col_test())
  {
    p3d_set_and_update_this_robot_conf(robot, q0);
    result= GP_ERROR;
  }

//   if(result==0)
//   {   printf("%s: %d: gpFold_arm(): the arm could not be folded.\n",__FILE__,__LINE__);   }

  p3d_destroy_config(robot, q0);

  return result;
}

//! @ingroup graspPlanning 
//! Deactivates all the collision tests for the arm bodies of the specified robot.
//! \param robot the robot (its arm bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDeactivate_arm_collisions(p3d_rob *robot)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_arm_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i;
  std::string arm_body_base_name, body_name;

  arm_body_base_name= std::string(robot->name) + "." + std::string(GP_ARM_BODY_PREFIX) + std::string(".");

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of arm_body_base_name,
    //to arm_body_base_name:
    if(body_name.compare(0, arm_body_base_name.length(), arm_body_base_name)==0)
    {
       p3d_col_deactivate_obj(robot->o[i]);
       continue;
    }

  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Activates all the collision tests for the arm bodies of the specified robot.
//! \param robot the robot (its arm bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpActivate_arm_collisions(p3d_rob *robot)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_arm_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif
  int i;
  std::string arm_body_base_name, body_name;

  arm_body_base_name= std::string(robot->name) + "." + std::string(GP_ARM_BODY_PREFIX) + std::string(".");

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of arm_body_base_name,
    //to arm_body_base_name:
    if(body_name.compare(0, arm_body_base_name.length(), arm_body_base_name)==0)
    {
       p3d_col_activate_obj(robot->o[i]);
       continue;
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Deactivates all the collision tests for the hand bodies of the specified robot.
//! \param robot the robot (its hand bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDeactivate_hand_collisions(p3d_rob *robot)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_hand_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i;
  std::string hand_body_base_name, body_name;

  hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".");

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of hand_body_base_name,
    //to hand_body_base_name:
    if(body_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
    {
       p3d_col_deactivate_obj(robot->o[i]);
       continue;
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Activates all the collision tests for the hand bodies of the specified robot.
//! \param robot the robot (its hand bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpActivate_hand_collisions(p3d_rob *robot)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_hand_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i;
  std::string hand_body_base_name, body_name;

  hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".");

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of hand_body_base_name,
    //to hand_body_base_name:
    if(body_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
    {
       p3d_col_activate_obj(robot->o[i]);
       continue;
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Deactivates all the collision tests for the specified finger of the specified robot.
//! \param robot the robot (its finger bodies must have specific names, defined in graspPlanning.h)
//! \param finger_index the number of the finger ( 1 <= finger_index <= hand number of fingers)
//! \param hand a gpHand_properties variable filled with information concerning the chosen hand characteristics
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDeactivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_finger_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
   if( finger_index < 1 || finger_index > hand.nb_fingers )
   {
      printf("%s: %d: gpDeactivate_finger_collisions(): the finger index exceeds the hand number of fingers.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i;
  std::string finger_body_base_name, body_name;
  std::stringstream out;

  finger_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".") + GP_FINGER_BODY_PREFIX;
  out << finger_index;
  finger_body_base_name+= out.str();

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of finger_body_base_name,
    //to finger_body_base_name:
    if(body_name.compare(0, finger_body_base_name.length(), finger_body_base_name)==0)
    {
       p3d_col_deactivate_obj(robot->o[i]);
       continue;
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Activates all the collision tests for the specified finger of the specified robot.
//! \param robot the robot (its finger bodies must have specific names, defined in graspPlanning.h)
//! \param finger_index the number of the finger ( 1 <= finger_index <= hand number of fingers)
//! \param hand a gpHand_properties variable filled with information concerning the chosen hand characteristics
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpActivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_finger_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
   if( finger_index < 1 || finger_index > hand.nb_fingers )
   {
      printf("%s: %d: gpActivate_finger_collisions(): the finger index exceeds the hand number of fingers.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i;
  std::string finger_body_base_name, body_name;
  std::stringstream out;

  finger_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".") + GP_FINGER_BODY_PREFIX;

  out << finger_index;
  finger_body_base_name+= out.str();

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of finger_body_base_name,
    //to finger_body_base_name:
    if(body_name.compare(0, finger_body_base_name.length(), finger_body_base_name)==0)
    {
       p3d_col_activate_obj(robot->o[i]);
       continue;
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Computes a set of contact points on the surface of an object mesh.
//! \param object the object
//! \param step the discretization step of the sampling (if it is bigger than the triangle dimensions, there will be only one sample generated, positioned at the triangle center)
//! \param shift the point will be shifted in the direction of the surface normal of a distance 'shift'
//! \param contactList a contactList list the computed set of contacts will be added to
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSample_obj_surface(p3d_obj *object, double step, double shift, std::list<gpContact> &contactList)
{
  unsigned int nb_samples= 0, nb_faces= 0;
  unsigned int i, j;
  p3d_index *indices= NULL;
  p3d_vector3 *points= NULL, *surf_points= NULL;
  p3d_face *faces= NULL;
  p3d_polyhedre *poly= NULL;
  gpContact contact;

  poly= object->pol[0]->poly;
  points= poly->the_points;
  nb_faces= poly->nb_faces;
  faces= poly->the_faces;

  for(i=0; i<nb_faces; ++i)
  {
    if(faces[i].plane==NULL)
    {
      printf("%s: %d: gpSample_obj_surface(): a plane of a face has not been computed -> call p3d_build_planes() first.\n",__FILE__,__LINE__);
      continue;
    }

    indices= faces[i].the_indexs_points;

    surf_points= gpSample_triangle_surface(points[indices[0]-1], points[indices[1]-1], points[indices[2]-1], step, &nb_samples);
 
    for(j=0; j<nb_samples; ++j)
    {
      p3d_vectCopy(surf_points[j], contact.position);
      p3d_vectCopy(faces[i].plane->normale, contact.normal);
      contact.position[0]+= shift*contact.normal[0];
      contact.position[1]+= shift*contact.normal[1];
      contact.position[2]+= shift*contact.normal[2];
      contact.surface= poly;
      contactList.push_back(contact);
    }
    nb_samples= 0;
    free(surf_points);
    surf_points= NULL;
  }

  return GP_OK;
}
