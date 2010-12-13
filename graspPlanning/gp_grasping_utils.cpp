
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
    case GP_PR2_GRIPPER:
      return "GP_PR2_GRIPPER";
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
    case GP_PR2_GRIPPER:
      return "pr2_gripper";
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
//! From an hand ID (as used in class gpGrasp), returns the string that must be used
//! in a .p3d file as a suffix to the name of a part of the hand.
std::string gpHand_suffix_from_ID(int id)
{
  std::string suffix;

  if(id < 0)
  {
    printf("%s: %d: gpHand_suffix_from_ID(): input ID must be >= 0.\n",__FILE__,__LINE__);
    return suffix;
  }

//   if(id==0)
//   {  return suffix;  }

  suffix= std::string("_") + convertToString(id);
  return suffix;
}

//! @ingroup graspPlanning
//! From an arm ID, returns the string that must be used
//! in a .p3d file as a suffix to the name of a part of the arm.
std::string gpArm_suffix_from_ID(int id)
{
  std::string suffix;

  if(id < 0)
  {
    printf("%s: %d: gpArm_suffix_from_ID(): input ID must be >= 0.\n",__FILE__,__LINE__);
    return suffix;
  }

  if(id==0)
  {  return suffix;  }

  suffix= std::string("_") + convertToString(id);
  return suffix;
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

  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
  glDisable(GL_LIGHTING);
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

  glPopAttrib();
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
configPt gpRandom_robot_base(p3d_rob *robot, double innerRadius, double outerRadius, p3d_vector3 objLoc)
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
//! \param handProp structure containing information about the hand geometry
//! \param q array that will be filled with the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &handProp, double q[4], int finger_index, int handID)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGet_SAHfinger_joint_angles(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpGet_SAHfinger_joint_angles(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return GP_ERROR;
   }
  #endif

  std::string jointName, suffix;
  p3d_jnt *joint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  switch(handProp.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:

     switch(finger_index)
     {
       case 1: //thumb
          jointName= std::string(GP_THUMBJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[0]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_THUMBJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[1]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_THUMBJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[2]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_THUMBJOINT4) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[3]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");
       break;
       case 2: //forefinger
          q[0]= 0.0;

          jointName= std::string(GP_FOREFINGERJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[1]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_FOREFINGERJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[2]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_FOREFINGERJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[3]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");
       break;
       case 3: //middle finger
          q[0]= 0.0;

          jointName= std::string(GP_MIDDLEFINGERJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[1]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_MIDDLEFINGERJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[2]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_MIDDLEFINGERJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[3]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");
       break;
       case 4: //ring finger
          q[0]= 0.0;

          jointName= std::string(GP_RINGFINGERJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[1]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_RINGFINGERJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[2]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");

          jointName= std::string(GP_RINGFINGERJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  q[3]= p3d_jnt_get_dof(joint, 0);  } else printf("problem\n");
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
int gpSet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand, double q[4], int finger_index, int handID)
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

  std::string jointName, suffix;
  p3d_jnt *joint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:

     switch(finger_index)
     {
       case 1: //thumb
          jointName= std::string(GP_THUMBJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[0]);  }

          jointName= std::string(GP_THUMBJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[1]);  }

          jointName= std::string(GP_THUMBJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[2]);  }

          jointName= std::string(GP_THUMBJOINT4) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[3]);  }
       break;
       case 2: //forefinger
          jointName= std::string(GP_FOREFINGERJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[1]);  }

          jointName= std::string(GP_FOREFINGERJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[2]);  }

          jointName= std::string(GP_FOREFINGERJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[3]);  }
       break;
       case 3: //middle finger
          jointName= std::string(GP_MIDDLEFINGERJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[1]);  }

          jointName= std::string(GP_MIDDLEFINGERJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[2]);  }

          jointName= std::string(GP_MIDDLEFINGERJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[3]);  }
       break;
       case 4: //ring finger
          jointName= std::string(GP_RINGFINGERJOINT1) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[1]);  }

          jointName= std::string(GP_RINGFINGERJOINT2) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[2]);  }

          jointName= std::string(GP_RINGFINGERJOINT3) + suffix;
          joint=  p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          if(joint!=NULL)
          {  p3d_jnt_set_dof(joint, 0, q[3]);  }
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
//! \param p the computed position of the fingertip center wrt to the wrist frame
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
//      g3d_drawColorSphere( p[0], p[1], p[2],  0.005, Yellow, NULL);
//      glLineWidth(3);
//      g3d_drawOneLine( p[0], p[1], p[2], p[0]+0.03*fingerpad_normal[0], p[1]+0.03*fingerpad_normal[1], p[2]+0.03*fingerpad_normal[2], Red, NULL);
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

  q0min= hand.qmin.at(0);
  q0max= hand.qmax.at(0);
  q1min= hand.qmin.at(3*(finger_index-1) + 1);
  q1max= hand.qmax.at(3*(finger_index-1) + 1);
  q2min= hand.qmin.at(3*(finger_index-1) + 2);
  q2max= hand.qmax.at(3*(finger_index-1) + 2);
  q3min= hand.qmin.at(3*(finger_index-1) + 3);
  q3max= hand.qmax.at(3*(finger_index-1) + 3);

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
  

  base_name= std::string(GP_HAND_BODY_PREFIX) + "." +std::string(GP_FINGER_BODY_PREFIX);
  

//  base_name= std::string(GP_HAND_BODY_PREFIX) + std::string(".") +std::string( GP_FINGER_BODY_PREFIX);

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
    case GP_GRIPPER: case GP_PR2_GRIPPER:
      q[0]= hand.max_opening_jnt_value;
    break;
//! warning: in the following the SAHand joint values should not be their maximal bounds:
    case  GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      q= hand.qmax;
    break;
    default:
     printf("%s: %d: gpOpen_hand(): unsupported hand type.\n",__FILE__,__LINE__);
     return GP_ERROR;
    break;
  }

  gpSet_hand_configuration(robot, hand , q, false);

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
    case GP_GRIPPER: case GP_PR2_GRIPPER:
//       q[0]= hand.min_opening_jnt_value;
      q[0]= hand.qmin.at(0);
    break;
//! warning: in the following the SAHand joint values should not be their minimal bounds:
    case  GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      q[0]= hand.qmin[0];
      q[1]= hand.qmin[1];
      q[2]= hand.qmin[2];
      q[3]= hand.qmin[3];

      q[4]= hand.qmin[4];
      q[5]= hand.qmin[5];
      q[6]= hand.qmin[6];

      q[7]= hand.qmin[7];
      q[8]= hand.qmin[8];
      q[9]= hand.qmin[9];

      q[10]= hand.qmin[10];
      q[11]= hand.qmin[11];
      q[12]= hand.qmin[12];
//       q[0]= hand.q0min[0];
//       q[1]= hand.q1min[0];
//       q[2]= hand.q2min[0];
//       q[3]= hand.q3min[0];
// 
//       q[4]= hand.q1min[1];
//       q[5]= hand.q2min[1];
//       q[6]= hand.q3min[1];
// 
//       q[7]= hand.q1min[2];
//       q[8]= hand.q2min[2];
//       q[9]= hand.q3min[2];
// 
//       q[10]= hand.q1min[3];
//       q[11]= hand.q2min[3];
//       q[12]= hand.q3min[3];
    break;
    default:
     printf("%s: %d: gpClose_hand(): unsupported hand type.\n",__FILE__,__LINE__);
     return GP_ERROR;
    break;
  }

  gpSet_hand_configuration(robot, hand , q, false);

  return GP_OK;
}

int gpClose_gripper_until_contact(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpClose_gripper_until_contact(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpClose_gripper_until_contact(): object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(hand.type!=GP_GRIPPER && hand.type!=GP_PR2_GRIPPER)
   {
     printf("%s: %d: gpClose_gripper_until_contact(): this function only applies to GP_GRIPPER.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  unsigned int i, n= 30, nb_contacts;
  double alpha;
  std::vector<double> q0;
  std::vector<double> q;
  std::vector<double> qprev;
  std::vector<p3d_obj*> fingertipBodies, handBodies;

  q0.resize(hand.nb_dofs);
  q.resize(hand.nb_dofs);
  qprev.resize(hand.nb_dofs);

  gpGet_hand_configuration(robot, hand, 0, q0);


  gpGet_fingertip_bodies(robot, hand, fingertipBodies);

  gpGet_non_fingertip_bodies(robot, hand, handBodies);



  if(p3d_col_test_robot_obj(robot, object))
  {
    return GP_ERROR;
  }

  nb_contacts= 0;
  for(i=0; i<n; ++i)
  {
    alpha= ((double) i)/((double) n);
//     q[0]= (1-alpha)*q0[0] + alpha*hand.min_opening_jnt_value;
    q[0]= (1-alpha)*q0[0] + alpha*hand.qmin.at(0);

    gpSet_hand_configuration(robot, hand , q, false);

    if(p3d_col_test_robot_obj(robot, object))
    {
      nb_contacts= gpCount_object_fingertips_collisions(robot, object, hand);
      gpSet_hand_configuration(robot, hand , qprev, false);
      return nb_contacts;
    }
  }

   gpSet_hand_configuration(robot, hand , q0, false);

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
//! \param arm_type arm type
//! \param q vector of the joint angles
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, std::vector<double> &q)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGet_arm_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  unsigned int i, nbArmJoints;
  std::vector< std::string > jointNames;
  p3d_jnt *armJoint= NULL;

  switch(arm_type)
  {
    case GP_PA10: 
      nbArmJoints= 6;
      jointNames.resize(6);
      jointNames.at(0)= GP_ARMJOINT1;
      jointNames.at(1)= GP_ARMJOINT2;
      jointNames.at(2)= GP_ARMJOINT3;
      jointNames.at(3)= GP_ARMJOINT4;
      jointNames.at(4)= GP_ARMJOINT5;
      jointNames.at(5)= GP_ARMJOINT6;
    break;
    case GP_LWR: 
      nbArmJoints= 7;
      jointNames.resize(7);
      jointNames.at(0)= GP_ARMJOINT1;
      jointNames.at(1)= GP_ARMJOINT2;
      jointNames.at(2)= GP_ARMJOINT3;
      jointNames.at(3)= GP_ARMJOINT4;
      jointNames.at(4)= GP_ARMJOINT5;
      jointNames.at(5)= GP_ARMJOINT6;
      jointNames.at(6)= GP_ARMJOINT7;
    break;
    default:
      printf("%s: %d: gpGet_arm_configuration(): unsupported arm type.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  q.resize(nbArmJoints);

  for(i=0; i<nbArmJoints; ++i)
  {
    armJoint= p3d_get_robot_jnt_by_name(robot, (char*)GP_ARMJOINT1);
    if(armJoint==NULL)
    {  return GP_ERROR; }
    q.at(i)= armJoint->dof_data[i].v;
  }
/*
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
  }*/

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Sets the robot's arm configuration with the given values (in radians).
//! NB: The respect of joint limits is verified.
//! \param robot pointer to the robot
//! \param arm_type arm type
//! \param q vector of the joint angles
//! \param verbose enable/disable error message display
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, std::vector<double> q, bool verbose)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpSet_arm_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  bool isValid;
  unsigned int i, nbArmJoints;
  double qmin, qmax;
  p3d_jnt *armJoint= NULL;
  configPt cfg= NULL;
  std::vector< std::string > jointNames;

  #ifdef LIGHT_PLANNER
  deactivateCcCntrts(robot, -1);
  #endif

  cfg= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &cfg);


  switch(arm_type)
  {
    case GP_PA10: 
      nbArmJoints= 6;
      jointNames.resize(6);
      jointNames.at(0)= GP_ARMJOINT1;
      jointNames.at(1)= GP_ARMJOINT2;
      jointNames.at(2)= GP_ARMJOINT3;
      jointNames.at(3)= GP_ARMJOINT4;
      jointNames.at(4)= GP_ARMJOINT5;
      jointNames.at(5)= GP_ARMJOINT6;
    break;
    case GP_LWR: 
      nbArmJoints= 7;
      jointNames.resize(7);
      jointNames.at(0)= GP_ARMJOINT1;
      jointNames.at(1)= GP_ARMJOINT2;
      jointNames.at(2)= GP_ARMJOINT3;
      jointNames.at(3)= GP_ARMJOINT4;
      jointNames.at(4)= GP_ARMJOINT5;
      jointNames.at(5)= GP_ARMJOINT6;
      jointNames.at(6)= GP_ARMJOINT7;
    break;
    default:
      printf("%s: %d: gpSet_arm_configuration(): unsupported arm type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot, cfg);
      return GP_ERROR;
    break;
  }

  for(i=0; i<nbArmJoints; ++i)
  {
      ////////////////////////q1////////////////////////////
      armJoint= p3d_get_robot_jnt_by_name(robot, (char*)(jointNames.at(i)).c_str() );
      if(armJoint==NULL)
      {
        p3d_destroy_config(robot, cfg);
        return GP_ERROR;
      }
      isValid= true;
      qmin= armJoint->dof_data[0].vmin;
      qmax= armJoint->dof_data[0].vmax;
      if(q.at(i) > qmax)
      {
        q.at(i)-= 2*M_PI;
        if( (q.at(i) < qmin) || (q.at(i) > qmax) )
        {  isValid= false; }
      }
      if(q.at(i) < qmin)
      {
        q.at(i)+= 2*M_PI;
        if( (q.at(i) < qmin) || (q.at(i) > qmax) )
        {  isValid= false; }
      }
      if(isValid)
      {  cfg[armJoint->index_dof]=  q.at(i);   }
      else
      {
        if(verbose)
        {
            printf("%s: %d: gpSet_arm_configuration(): q[%d] value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,i,q.at(i),qmin,qmax);
        }
        p3d_destroy_config(robot, cfg);
        return GP_ERROR;
      }
        /////////////////////////////////////////////////////
  }

  p3d_set_and_update_this_robot_conf(robot, cfg);
  p3d_destroy_config(robot, cfg);

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Gets the hand/gripper's configuration of a robot and copies it in a std::vector.
//! \param robot pointer to the robot
//! \param handProp geometric information about the hand
//! \param handID ID of the hand in case the robot has several hand (see graspPlanning.h)
//! \param q a std::vector that will be filled with the current joint parameters of the hand
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_hand_configuration(p3d_rob *robot, gpHand_properties &handProp, int handID, std::vector<double> &q)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_hand_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  std::string jointName, suffix;
  p3d_jnt *fingerJoint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  q.resize(handProp.nb_dofs);

  switch(handProp.type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER:
      jointName= std::string(GP_GRIPPERJOINT) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[0]= fingerJoint->dof_data[0].v;
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      jointName= std::string(GP_THUMBJOINT1) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[0]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_THUMBJOINT2) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[1]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_THUMBJOINT3) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[2]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_THUMBJOINT4) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[3]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_FOREFINGERJOINT1) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[4]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_FOREFINGERJOINT2) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[5]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_FOREFINGERJOINT3) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[6]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_MIDDLEFINGERJOINT1) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[7]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_MIDDLEFINGERJOINT2) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[8]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_MIDDLEFINGERJOINT3) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[9]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_RINGFINGERJOINT1) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[10]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_RINGFINGERJOINT2) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {  return GP_ERROR;  }
      q[11]= fingerJoint->dof_data[0].v;

      jointName= std::string(GP_RINGFINGERJOINT3) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
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
//! \param robot pointer to the robot
//! \param handProp geometric information about the hand
//! \param config a std::vector containing the finger joint parameters associated to the grasp
//! \param verbose flag to print information in case of error or not
//! \param handID ID of the hand used by the grasp (see gpHand_suffix_from_ID)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_hand_configuration(p3d_rob *robot, gpHand_properties &handProp, std::vector<double> config, bool verbose, int handID)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_hand_configuration(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  unsigned int i;
  bool isValid;
  double eps= 1e-4;
  double qmin, qmax, q;
  std::string jointName, suffix;
  p3d_jnt *fingerJoint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  if(config.size()!=handProp.nb_dofs)
  {
    printf("%s: %d: gpSet_hand_configuration(): the input configuration vector has a bad size (%d instead of %d).\n",__FILE__,__LINE__,config.size(), handProp.nb_dofs);
    return GP_ERROR;
  }
//   verbose= true;
  configPt qcur= NULL;
  qcur= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &qcur);

  switch(handProp.type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER:
      jointName= std::string(GP_GRIPPERJOINT) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {
        p3d_destroy_config(robot, qcur);
        return GP_ERROR;
      }
      qmin= fingerJoint->dof_data[0].vmin;
      qmax= fingerJoint->dof_data[0].vmax;
      q= config[0];
      if( q<qmin || q>qmax )
      {
         if(verbose)
         {
            printf("%s: %d: gpSet_hand_configuration(): q[0] value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,config[0],qmin,qmax);
         }
         p3d_destroy_config(robot, qcur);
         return GP_ERROR;
      }
      qcur[fingerJoint->index_dof]= config[0];
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      for(i=0; i<13; i++)
      {
        switch(i)
        {
          case 0:
             jointName= std::string(GP_THUMBJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 1:
             jointName= std::string(GP_THUMBJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 2:
             jointName= std::string(GP_THUMBJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 3:
             jointName= std::string(GP_THUMBJOINT4) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 4:
             jointName= std::string(GP_FOREFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 5:
             jointName= std::string(GP_FOREFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 6:
             jointName= std::string(GP_FOREFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 7:
             jointName= std::string(GP_MIDDLEFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 8:
             jointName= std::string(GP_MIDDLEFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 9:
             jointName= std::string(GP_MIDDLEFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 10:
             jointName= std::string(GP_RINGFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 11:
             jointName= std::string(GP_RINGFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 12:
             jointName= std::string(GP_RINGFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
        }
        if(fingerJoint==NULL)
        {
          p3d_destroy_config(robot, qcur);
          return GP_ERROR;
        }
        isValid= true;
        qmin= fingerJoint->dof_data[0].vmin;
        qmax= fingerJoint->dof_data[0].vmax;
        q= config[i];
        if(q  > qmax )
        {
          if( q < qmax + eps)
          {   q = qmax;   } 
          else
          {
            q-= 2*M_PI;
            if( (q < qmin-eps) || (q > qmax+eps) )
            {  isValid= false; }
          }
        }
        if(q < qmin)
        {
          if( q > qmin - eps)
          {   q = qmin + eps;   } 
          else
          {
            q+= 2*M_PI;
            if( (q < qmin-eps) || (q > qmax+eps) )
            {  isValid= false; }
          }
        }
        if(!isValid)
        {
          if(verbose)
          {
            printf("%s: %d: gpSet_hand_configuration(): q[%d] value (%f°) is out of range (%f° %f°).\n",__FILE__,__LINE__,i,config[i]*RADTODEG,qmin*RADTODEG,qmax*RADTODEG);
          }
          p3d_destroy_config(robot, qcur);
          return GP_ERROR;
        }
        qcur[fingerJoint->index_dof]= q;
      }
    break;
    default:
       printf("%s: %d: gpSet_hand_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }

//  p3d_copy_config_into(robot, qcur, &robot->ROBOT_POS);

  p3d_set_and_update_this_robot_conf(robot, qcur);
  p3d_destroy_config(robot, qcur);

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Sets the hand/gripper's configuration of a robot with the configuration contained in a std::vector.
//! The rest of the robot configuration is set from a configPt.
//! \param robot pointer to the robot
//! \param handProp geometric information about the hand
//! \param config a std::vector containing the finger joint parameters associated to the grasp
//! \param qr desired complete configuration vector of the robot (only the non-hand part will be used)
//! \param handID ID of the hand used by the grasp
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_hand_configuration(p3d_rob *robot, gpHand_properties &hand, std::vector<double> config, configPt qr, int handID)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_hand_configuration(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(qr==NULL)
  {
    printf("%s: %d: gpSet_hand_configuration(): input configPt is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  unsigned int i;
  bool isValid;
  double eps= 1e-4;
  double qmin, qmax, q;
  std::string jointName, suffix;
  p3d_jnt *fingerJoint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  if(config.size()!=hand.nb_dofs)
  {
    printf("%s: %d: gpSet_hand_configuration(): the input configuration vector has a bad size (%d instead of %d).\n",__FILE__,__LINE__,config.size(), hand.nb_dofs);
    return GP_ERROR;
  }

  switch(hand.type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER:
      jointName= std::string(GP_GRIPPERJOINT) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {
        return GP_ERROR;
      }
      qmin= fingerJoint->dof_data[0].vmin;
      qmax= fingerJoint->dof_data[0].vmax;
      q= config[0];
      if( q<qmin || q>qmax )
      {
         return GP_ERROR;
      }
      qr[fingerJoint->index_dof]= config[0];
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      for(i=0; i<13; i++)
      {
        switch(i)
        {
          case 0:  
             jointName= std::string(GP_THUMBJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 1:  
             jointName= std::string(GP_THUMBJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 2:  
             jointName= std::string(GP_THUMBJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 3:  
             jointName= std::string(GP_THUMBJOINT4) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 4:  
             jointName= std::string(GP_FOREFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 5:  
             jointName= std::string(GP_FOREFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 6:  
             jointName= std::string(GP_FOREFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 7:  
             jointName= std::string(GP_MIDDLEFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 8:  
             jointName= std::string(GP_MIDDLEFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 9:  
             jointName= std::string(GP_MIDDLEFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 10:  
             jointName= std::string(GP_RINGFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 11:  
             jointName= std::string(GP_RINGFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 12:  
             jointName= std::string(GP_RINGFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
        }
        if(fingerJoint==NULL)
        {
          return GP_ERROR;
        }
        isValid= true;
        qmin= fingerJoint->dof_data[0].vmin;
        qmax= fingerJoint->dof_data[0].vmax;
        q= config[i];
        if(q  > qmax )
        {
          if( q < qmax + eps)
          {   q = qmax;   } 
          else
          {
            q-= 2*M_PI;
            if( (q < qmin-eps) || (q > qmax+eps) )
            {  isValid= false; }
          }
        }
        if(q < qmin)
        {
          if( q > qmin - eps)
          {   q = qmin + eps;   } 
          else
          {
            q+= 2*M_PI;
            if( (q < qmin-eps) || (q > qmax+eps) )
            {  isValid= false; }
          }
        }
        if(!isValid)
        {
          return GP_ERROR;
        }
        qr[fingerJoint->index_dof]= q;
      }
    break;
    default:
       printf("%s: %d: gpSet_hand_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, qr);

  return GP_OK;
}

//! Sets the configuration of a robot hand from a grasp.
//! \param robot pointer to the robot hand_type
//! \param object pointer to the object (freeflyer robot)
//! \param grasp the grasp to set
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_robot_hand_grasp_configuration(p3d_rob *robot, p3d_rob *object, const gpGrasp &grasp)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_robot_hand_grasp_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(object==NULL)
  {
    printf("%s: %d: gpSet_robot_hand_grasp_configuration(): object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  gpHand_properties handProp;
  p3d_jnt *objectJnt= NULL;
  configPt q= NULL;

  handProp.initialize(grasp.hand_type);

  q= p3d_alloc_config(robot);

  objectJnt= object->o[grasp.body_index]->jnt;

  if(objectJnt==NULL)
  {
    printf("%s: %d: gpSet_robot_hand_grasp_configuration(): robot object has no valid joint.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  gpInverse_geometric_model_freeflying_hand(robot, objectJnt->abs_pos, (p3d_matrix_type(*)[4])grasp.frame, handProp, q);

  p3d_set_and_update_this_robot_conf(robot, q);

  gpSet_grasp_configuration(robot, grasp, 0);

  p3d_destroy_config(robot, q);

  return GP_OK;
}

//! Sets the open configuration of a robot hand from a grasp.
//! \param robot pointer to the robot hand_type
//! \param object pointer to the object (freeflyer robot)
//! \param grasp the grasp to set
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_robot_hand_grasp_open_configuration(p3d_rob *robot, p3d_rob *object, const gpGrasp &grasp)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_robot_hand_grasp_open_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(object==NULL)
  {
    printf("%s: %d: gpSet_robot_hand_grasp_open_configuration(): object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  gpHand_properties handProp;
  p3d_jnt *objectJnt= NULL;
  configPt q= NULL;

  handProp.initialize(grasp.hand_type);

  q= p3d_alloc_config(robot);

  objectJnt= object->o[grasp.body_index]->jnt;

  if(objectJnt==NULL)
  {
    printf("%s: %d: gpSet_robot_hand_grasp_configuration(): robot object has no valid joint.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  gpInverse_geometric_model_freeflying_hand(robot, objectJnt->abs_pos, (p3d_matrix_type(*)[4])grasp.frame, handProp, q);

  p3d_set_and_update_this_robot_conf(robot, q);

  gpSet_grasp_open_configuration(robot, grasp, 0);

  p3d_destroy_config(robot, q);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot with the grasping configuration contained in a gpGrasp variable.
//! It only modifies the parameters of the hand.
//! NB: The finger joints require to have specific names (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param grasp the grasp to set
//! \param handID the hand to set (in case there are several): 0 by default
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_grasp_configuration(p3d_rob *robot, const gpGrasp &grasp, int handID)
{
  gpHand_properties handProp;

  handProp.initialize(grasp.hand_type);

  return  gpSet_hand_configuration(robot, handProp, grasp.config, true, handID);
}

//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot with the open configuration contained in a gpGrasp variable.
//! It only modifies the parameters of the hand.
//! NB: The finger joints require to have specific names (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param grasp the grasp to set
//! \param handID the hand to set (in case there are several): 0 by default
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_grasp_open_configuration(p3d_rob *robot, const gpGrasp &grasp, int handID)
{
  gpHand_properties handProp;

  handProp.initialize(grasp.hand_type);

  return  gpSet_hand_configuration(robot, handProp, grasp.openConfig, true, handID);
}

//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot to a "rest" configuration.
//! It only modifies the parameters of the hand.
//! NB: The finger joints require to have specific names (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param hand information concerning the hand
//! \param handID the hand to set (in case there are several): 0 by default
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_hand_rest_configuration(p3d_rob *robot, gpHand_properties &handProp, int handID)
{
  return  gpSet_hand_configuration(robot, handProp, handProp.qrest, true, handID);
}



//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot with the grasping configuration contained in a gpGrasp variable.
//! The rest of the robot configuration is set from a configPt.
//! NB: The finger joints require to have specific names (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param grasp the grasp to set
//! \param q desired complete configuration vector of the robot (only the non-hand part will be used)
//! \param handID the hand to set (in case there are several): 0 by default
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_grasp_configuration(p3d_rob *robot, const gpGrasp &grasp, configPt q, int handID)
{
  gpHand_properties handProp;

  handProp.initialize(grasp.hand_type);

  return  gpSet_hand_configuration(robot, handProp, grasp.config, q, handID);
}

//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot with the open configuration contained in a gpGrasp variable.
//! The rest of the robot configuration is set from a configPt.
//! NB: The finger joints require to have specific names (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param grasp the grasp to set
//! \param q desired complete configuration vector of the robot (only the non-hand part will be used)
//! \param handID the hand to set (in case there are several): 0 by default
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_grasp_open_configuration(p3d_rob *robot, const gpGrasp &grasp, configPt q, int handID)
{
  gpHand_properties handProp;

  handProp.initialize(grasp.hand_type);

  return  gpSet_hand_configuration(robot, handProp, grasp.openConfig, q, handID);
}

//! @ingroup graspPlanning 
//! Sets the hand/gripper configuration of a robot to a "rest" configuration.
//! The rest of the robot configuration is set from a configPt.
//! NB: The finger joints require to have specific names (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param hand information concerning the hand
//! \param q desired complete configuration vector of the robot (only the non-hand part will be used)
//! \param handID the hand to set (in case there are several): 0 by default
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSet_hand_rest_configuration(p3d_rob *robot, gpHand_properties &handProp, configPt q, int handID)
{
  return  gpSet_hand_configuration(robot, handProp, handProp.qrest, q, handID);
}


#ifdef LIGHT_PLANNER
//! @ingroup graspPlanning
//! Fix the hand/gripper's configuration of a robot
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFix_hand_configuration(p3d_rob *robot, gpHand_properties &hand, int handID)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpFix_grasp_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  unsigned int i;
  std::string jointName, suffix;
  p3d_jnt *fingerJoint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  switch(hand.type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER:
      jointName= std::string(GP_GRIPPERJOINT) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {
        return GP_ERROR;
      }
      fixJoint(robot, fingerJoint, fingerJoint->jnt_mat);
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:{
      std::string jntNames[17] = {std::string(GP_THUMBJOINT1), std::string(GP_THUMBJOINT2), std::string(GP_THUMBJOINT3), std::string(GP_THUMBJOINT4), std::string(GP_THUMBJOINT5), std::string(GP_FOREFINGERJOINT1), std::string(GP_FOREFINGERJOINT2), std::string(GP_FOREFINGERJOINT3), std::string(GP_FOREFINGERJOINT4), std::string(GP_MIDDLEFINGERJOINT1), std::string(GP_MIDDLEFINGERJOINT2), std::string(GP_MIDDLEFINGERJOINT3), std::string(GP_MIDDLEFINGERJOINT4), std::string(GP_RINGFINGERJOINT1), std::string(GP_RINGFINGERJOINT2), std::string(GP_RINGFINGERJOINT3), std::string(GP_RINGFINGERJOINT4)};
      for(i=0; i<17; i++)
      {
        jointName= jntNames[i] + suffix;
        fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
        if(fingerJoint==NULL)
        {
          return GP_ERROR;
        }
        fixJoint(robot, fingerJoint, fingerJoint->jnt_mat);
      }
    break;
    }
    default:
       printf("%s: %d: gpFix_grasp_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }
  return GP_OK;
}

//! @ingroup graspPlanning
//! Unfix the hand/gripper's configuration of a robot
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpUnFix_hand_configuration(p3d_rob *robot, gpHand_properties &hand, int handID)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpUnFix_hand_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  unsigned int i;
  std::string jointName, suffix;
  p3d_jnt *fingerJoint= NULL;

  suffix= gpHand_suffix_from_ID(handID);

  switch(hand.type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER:
      jointName= std::string(GP_GRIPPERJOINT) + suffix;
      fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
      if(fingerJoint==NULL)
      {
        return GP_ERROR;
      }
      unFixJoint(robot, fingerJoint);
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      for(i=0; i<12; i++)
      {
        switch(i)
        {
          case 0:
             jointName= std::string(GP_THUMBJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 1:
             jointName= std::string(GP_THUMBJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 2:
             jointName= std::string(GP_THUMBJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 3:
             jointName= std::string(GP_THUMBJOINT4) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 4:
             jointName= std::string(GP_FOREFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 5:
             jointName= std::string(GP_FOREFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 6:
             jointName= std::string(GP_FOREFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 7:
             jointName= std::string(GP_MIDDLEFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 8:
             jointName= std::string(GP_MIDDLEFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 9:
             jointName= std::string(GP_MIDDLEFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 10:
             jointName= std::string(GP_RINGFINGERJOINT1) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 11:
             jointName= std::string(GP_RINGFINGERJOINT2) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
          case 12:
             jointName= std::string(GP_RINGFINGERJOINT3) + suffix;
             fingerJoint= p3d_get_robot_jnt_by_name(robot, (char*)jointName.c_str());
          break;
        }
        if(fingerJoint==NULL)
        {
          return GP_ERROR;
        }
        unFixJoint(robot, fingerJoint);
      }
    break;
    default:
       printf("%s: %d: gpUnFix_hand_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }
  return GP_OK;
}
#endif


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
  std::vector<double> q(6);
  configPt cfg0= NULL;

  cfg0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &cfg0);

  #ifdef LIGHT_PLANNER
  if(robot->openChainConf!=NULL)
  {
    p3d_update_virtual_object_config_for_arm_ik_constraint(robot, 0, robot->openChainConf);
    //p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, robot->openChainConf);
    p3d_set_and_update_this_robot_conf(robot, robot->openChainConf);
    gpGet_arm_configuration(robot, arm_type, q);
    p3d_set_and_update_this_robot_conf(robot, cfg0);
  }
  #else
    //for horizontal jido:
    q.at(0)= DEGTORAD*(-90);
    q.at(1)= DEGTORAD*(90);
    q.at(2)= DEGTORAD*(45);
    q.at(3)= DEGTORAD*(0);
    q.at(4)= DEGTORAD*(-45);
    q.at(5)= DEGTORAD*(0);

    //for vertical jido:
    q.at(0)= DEGTORAD*(77.15);
    q.at(1)= DEGTORAD*(-8.99);
    q.at(2)= DEGTORAD*(148.39);
    q.at(3)= DEGTORAD*(80.17);
    q.at(4)= DEGTORAD*(-81.68);
    q.at(5)= DEGTORAD*(-18.51);
  #endif


  switch(arm_type)
  {
    case GP_PA10:
      result= gpSet_arm_configuration(robot, GP_PA10, q);
    break;
    default:
      printf("%s: %d: gpFold_arm(): unsupported arm type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot, cfg0);
      return GP_ERROR;
    break;
  }

  if(p3d_col_test())
  {
    p3d_set_and_update_this_robot_conf(robot, cfg0);
    result= GP_ERROR;
  }

//   if(result==0)
//   {   printf("%s: %d: gpFold_arm(): the arm could not be folded.\n",__FILE__,__LINE__);   }

  p3d_destroy_config(robot, cfg0);

  return result;
}

//! @ingroup graspPlanning 
//! Deactivates all the collision tests for the arm bodies of the specified robot.
//! \param robot the robot (its arm bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDeactivate_arm_collisions(p3d_rob *robot, int armID)
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
int gpActivate_arm_collisions(p3d_rob *robot, int armID)
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
int gpDeactivate_hand_collisions(p3d_rob *robot, int handID)
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
int gpActivate_hand_collisions(p3d_rob *robot, int handID)
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

  if(handID==0)
  {
    hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".");
  }
  else
  {
    hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + convertToString(handID) + std::string(".");
  }


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
//! Deactivates all the selfcollision tests for the hand bodies of the specified robot.
//! \param robot the robot (its hand bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDeactivate_hand_selfcollisions(p3d_rob *robot, int handID)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_hand_selfcollisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i, j;
  std::string hand_body_base_name, body1_name, body2_name;

  if(handID==0)
  {
    hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".");
  }
  else
  {
    hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + convertToString(handID) + std::string(".");
  } 

  for(i=0; i<robot->no; i++)
  {
    body1_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of hand_body_base_name,
    //to hand_body_base_name:
    if(body1_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
    {
      for(j=0; j<robot->no; j++)
      {
        if(j==i)
        { continue;  }

        body2_name= robot->o[j]->name;
        if(body2_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
        {
          //pqp_deactivate_object_object_collision(robot->o[i], robot->o[j]);
          p3d_col_deactivate_obj_obj(robot->o[i], robot->o[j]);
        }
      }
    }
  }

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Activates all the selfcollision tests for the hand bodies of the specified robot.
//! \param robot the robot (its hand bodies must have specific names, defined in graspPlanning.h)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpActivate_hand_selfcollisions(p3d_rob *robot, int handID)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_hand_selfcollisions(): robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
   }
  #endif

  int i, j;
  std::string hand_body_base_name, body1_name, body2_name;

  if(handID==0)
  {
    hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + std::string(".");
  }
  else
  {
    hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX) + convertToString(handID) + std::string(".");
  } 

  for(i=0; i<robot->no; i++)
  {
    body1_name= robot->o[i]->name;

    //compare a substring of body_name, with a length equal to the length of hand_body_base_name,
    //to hand_body_base_name:
    if(body1_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
    {
      for(j=0; j<robot->no; j++)
      {
        if(j==i)
        { continue;  }

        body2_name= robot->o[j]->name;
        if(body2_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
        {
          if(p3d_isMarkedForautocol(robot->num, robot->o[i]->num, robot->o[j]->num) == 1){
            p3d_col_activate_obj_obj(robot->o[i], robot->o[j]);
          }
        }
      }
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
int gpDeactivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand, int handID)
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
int gpActivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand, int handID)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_finger_collisions(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
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
//! Gets all the bodies corresponding of the fingertips.
//! \param robot the robot (its finger bodies must have specific names, defined in graspPlanning.h)
//! \param hand a gpHand_properties variable filled with information concerning the chosen hand characteristics
//! \param bodies a vector of pointers to the bodies
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_fingertip_bodies(p3d_rob *robot, gpHand_properties &hand, std::vector<p3d_obj*> &bodies)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_fingertip_bodies(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  int i;
  std::string fingertip_suffix, body_name;
  std::stringstream out;

  fingertip_suffix= GP_FINGERTIP_BODY_NAME;

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    if(body_name.compare(body_name.size() - fingertip_suffix.length(), fingertip_suffix.length(), fingertip_suffix)==0)
    { 
      bodies.push_back(robot->o[i]);
      continue;
    }
  }

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Gets all the bodies of the hand except for the fingertip bodies.
//! \param robot the robot (its finger bodies must have specific names, defined in graspPlanning.h)
//! \param hand a gpHand_properties variable filled with information concerning the chosen hand characteristics
//! \param bodies a vector of pointers to the bodies
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_non_fingertip_bodies(p3d_rob *robot, gpHand_properties &hand, std::vector<p3d_obj*> &bodies)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_non_fingertip_bodies(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  int i;
  std::string hand_body_base_name, fingertip_suffix, body_name;
  std::stringstream out;

  hand_body_base_name= std::string(robot->name) + "." + std::string(GP_HAND_BODY_PREFIX);
  fingertip_suffix= GP_FINGERTIP_BODY_NAME;

  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    if(body_name.compare(0, hand_body_base_name.length(), hand_body_base_name)==0)
    { 
      if(body_name.compare(body_name.size() - fingertip_suffix.length(), fingertip_suffix.length(), fingertip_suffix)!=0)
      {
        bodies.push_back(robot->o[i]);
        continue;
      }
    }
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Computes a set of contact points on the surface of a mesh.
//! \param poly the p3d_polyhedre
//! \param step the discretization step of the sampling (if it is bigger than the triangle dimensions, there will be only one sample generated, positioned at the triangle center)
//! \param shift the point will be shifted in the direction of the surface normal of a distance 'shift'
//! \param contactList a contactList list the computed set of contacts will be added to
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSample_poly_surface(p3d_polyhedre *poly, double step, double shift, std::list<gpContact> &contactList)
{
  if(poly==NULL)
  {
    printf("%s: %d: gpSample_poly_surface(): input p3d_polyhedre is NULL.\n",__FILE__,__LINE__); 
    return GP_ERROR;
  }

  unsigned int nb_samples= 0, nb_faces= 0;
  unsigned int i, j;
  p3d_index *indices= NULL;
  p3d_vector3 *points= NULL, *surf_points= NULL;
  p3d_face *faces= NULL;
  gpContact contact;

  points= poly->the_points;
  nb_faces= poly->nb_faces;
  faces= poly->the_faces;

  if(faces[0].plane==NULL)
  {
    p3d_build_planes(poly);
  }

  for(i=0; i<nb_faces; ++i)
  {
    if(faces[i].plane==NULL)
    {
      printf("%s: %d: gpSample_poly_surface(): a plane of a face has not been computed -> call p3d_build_planes() first.\n",__FILE__,__LINE__);
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
      contact.face= i;
      contactList.push_back(contact);
    }
    nb_samples= 0;
    free(surf_points);
    surf_points= NULL;
  }

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Computes a set of contact points on the surface of a mesh.
//! \param poly the p3d_polyhedre
//! \param nb_samples the desired number of samples
//! \param shift the point will be shifted in the direction of the surface normal of a distance 'shift'
//! \param contactList a contactList list the computed set of contacts will be added to
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSample_poly_surface_random(p3d_polyhedre *poly, unsigned int nb_samples, double shift, std::list<gpContact> &contactList)
{
  if(poly==NULL)
  {
    printf("%s: %d: gpSample_poly_surface(): input p3d_polyhedre is NULL.\n",__FILE__,__LINE__); 
    return GP_ERROR;
  }

  unsigned int nb_faces= 0;
  unsigned int i, irand;
  double areaMax, arand, alpha, beta;
  p3d_vector3 p;
  std::vector<double> areas;
  p3d_index *indices= NULL;
  p3d_vector3 *points= NULL;
  p3d_face *faces= NULL;
  gpContact contact;

  points= poly->the_points;
  nb_faces= poly->nb_faces;
  faces= poly->the_faces;

  if(faces[0].plane==NULL)
  {
    p3d_build_planes(poly);
  }

  areas.resize(nb_faces);
  for(i=0; i<nb_faces; ++i)
  {
    indices= faces[i].the_indexs_points;
    areas[i]= gpTriangle_area(points[indices[0]-1], points[indices[1]-1], points[indices[2]-1]);
    if( (i==0) || (areas[i]>areaMax) )
    {
      areaMax= areas[i];
    }
  }

  for(i=0; i<nb_samples; ++i)
  {
    while(true)
    {
      irand= (unsigned int) p3d_random(0.0, nb_faces);
      arand= p3d_random(0.0, areaMax);
      if(areas[irand] > arand)
      {  break; }
    }
    indices= faces[irand].the_indexs_points;
    alpha= p3d_random(0.0, 1.0);
    beta= p3d_random(0.0, 1.0);
    gpPoint_in_triangle_from_parameters(alpha, beta, points[indices[0]-1], points[indices[1]-1], points[indices[2]-1], p);
    p3d_vectCopy(p, contact.position);
    p3d_vectCopy(faces[irand].plane->normale, contact.normal);
    contact.position[0]+= shift*contact.normal[0];
    contact.position[1]+= shift*contact.normal[1];
    contact.position[2]+= shift*contact.normal[2];
    contact.surface= poly;
    contact.face= irand;
    contactList.push_back(contact);
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
  if(object==NULL)
  {
    printf("%s: %d: gpSample_obj_surface(): input p3d_obj is NULL.\n",__FILE__,__LINE__); 
    return GP_ERROR;
  }

  return gpSample_poly_surface(object->pol[0]->poly, step, shift, contactList);
}

//! does not work
//! Converts the bodies of a robot from ghost to graphic and vice-versa then restarts the collision checker (PQP).
//! \param robot pointer to the robot
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSwap_ghost_and_graphic_bodies(p3d_rob *robot)
{
  if(robot==NULL)
  {
     printf("%s: %d: gpSwitch_ghost_and_graphic_bodies(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
  }

  int i, j;

  for(i=0; i<robot->no; ++i)
  {
    for(j=0; j<robot->o[i]->np; ++j)
    {
//        if(robot->o[i]->pol[j]->p3d_objPt!=robot->o[i])
//        {  continue; }

       if(robot->o[i]->pol[j]->TYPE==P3D_GRAPHIC)
       {
         robot->o[i]->pol[j]->TYPE= P3D_GHOST;
       }
       else if(robot->o[i]->pol[j]->TYPE==P3D_GHOST)
       {
         robot->o[i]->pol[j]->TYPE= P3D_GRAPHIC;
       }
    }
  }

  p3d_col_stop();
  p3d_col_start(p3d_col_mode_pqp);

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Draws the intersection of the kdtree of the object surface sample points
//! and the hand workspace sphere approximation.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDraw_workspace_object_intersection(p3d_rob *object, p3d_rob *hand, gpHand_properties &handData)
{
  unsigned int i, j;
  GLfloat mat[16];
  GLfloat colors[4][3]= {{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0},{1.0,0.0,1.0}};
  p3d_vector3 center;
  p3d_matrix4 Twrist, Twrist_obj, Tobj, Tobj_inv, T[4];
  std::list<gpContact> contactList, points;
  std::list<gpContact>::iterator iter;
  static p3d_rob *obj= NULL;
  static gpKdTree kdtree;

  if(obj!=object)
  {
//     gpSample_obj_surface(object->o[0], 0.005, handData.fingertip_radius, contactList);
    gpSample_obj_surface(object->o[0], 0.005, 0, contactList);
    kdtree.build(contactList);
  }
  

  obj= object;

  p3d_get_freeflyer_pose(hand, Twrist);
  p3d_get_freeflyer_pose(object, Tobj);
  p3d_to_gl_matrix(Tobj, mat);
  p3d_matInvertXform(Tobj, Tobj_inv);
  p3d_mat4Mult(Tobj_inv, Twrist, Twrist_obj);

  p3d_mat4Mult(Twrist_obj, handData.Twrist_finger[0], T[0]);
  p3d_mat4Mult(Twrist_obj, handData.Twrist_finger[1], T[1]);
  p3d_mat4Mult(Twrist_obj, handData.Twrist_finger[2], T[2]);
  p3d_mat4Mult(Twrist_obj, handData.Twrist_finger[3], T[3]);


//   kdtree.draw(5);

  glPushAttrib(GL_LIGHTING_BIT | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glPointSize(6);
  glColor3f(1.0, 0.0, 0.0);

  glPushMatrix();
  glMultMatrixf(mat);

  for(j=0; j<4; ++j)
  {
//     g3d_rgb_from_hue(0.25+j*0.25, color);
// g3d_set_color(Any, color);
    glColor3f(colors[j][0], colors[j][1], colors[j][2]);
    glBegin(GL_POINTS);
    for(i=0; i<handData.workspace.size(); ++i)
    {
      p3d_xformPoint(T[j], handData.workspace[i].center, center);
      points.clear();
      kdtree.sphereIntersection(center, handData.workspace[i].radius, points);
  
      for(iter=points.begin(); iter!=points.end(); iter++)
      { 
  //       p3d_xformPoint(Tobj_inv, iter->position, p); //object frame -> world frame
  //       glVertex3dv(p);
        glVertex3dv(iter->position);
      }
    }
    glEnd();
  }



  glPopMatrix();
 
  glPopAttrib();

  return GP_OK;
}

