
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "GraspPlanning-pkg.h"

#include <time.h>
#include <sys/times.h>


#include <math.h>
#include <stdio.h>
#include <string>
#include <sstream>


// 10% ouverture pince pour test collision
//#define GRIP_OPEN_PERCENT 1.1
// 100% ouverture pince pour test collision
#define GRIP_OPEN_PERCENT 1.1


gpHand_properties::gpHand_properties()
{
  type= GP_HAND_NONE;
  nb_fingers= 0;
  nb_dofs= 0;
}


//! Initializes a gpHand_properties variable.
//! The function explores all the existing robots to find those with the specific
//! names defined in graspPlanning.h
//! The hand type is deduced from these names.
p3d_rob* gpHand_properties::initialize()
{
  int i;
  p3d_vector3 t, axis;
  p3d_matrix4 R, T, Trinit, Tt1, Tt2, Tr1, Tr2, Tr3, Tr4, Tint1, Tint2;
  p3d_rob *hand_robot= NULL;

  hand_robot= p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME);
  if(hand_robot!=NULL)
  {
    type= GP_GRIPPER;
  }
  else
  {
    hand_robot= p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
    if(hand_robot!=NULL)
    {
       type= GP_SAHAND_RIGHT;
    }
    else
    {
      hand_robot= p3d_get_robot_by_name(GP_SAHAND_LEFT_ROBOT_NAME);
      if(hand_robot!=NULL)
      {
       type= GP_SAHAND_LEFT;
      }
      else
      {
        printf("There must be a robot named \"%s\" or \"%s\" or \"%s\".\n", GP_GRIPPER_ROBOT_NAME, GP_SAHAND_RIGHT_ROBOT_NAME, GP_SAHAND_LEFT_ROBOT_NAME);
        return NULL;
      }
    }
  }



  switch(type)
  {
    case GP_GRIPPER:
       nb_fingers= 3;
       nb_dofs= 1;
       fingertip_distance =   0.04;
       fingertip_radius   =   0.0042;
       min_opening        =   0.01005;
       max_opening        =  0.075007;
       min_opening_jnt_value =   0;
       max_opening_jnt_value =   0.0325;

       p3d_mat4Copy(p3d_mat4IDENTITY, Tgrasp_frame_hand);
       Tgrasp_frame_hand[2][3]= 0.0;//0.007;

      //transformation grasp frame -> arm's wrist frame:
      /*
        x= z
        y= -x
        z= -y
      */
       T[0][0]=  0.0;  T[0][1]= -1.0;  T[0][2]=  0.0;  T[0][3]=  0.0;
       T[1][0]=  0.0;  T[1][1]=  0.0;  T[1][2]= -1.0;  T[1][3]=  0.0;
       T[2][0]=  1.0;  T[2][1]=  0.0;  T[2][2]=  0.0;  T[2][3]=  0;
       T[3][0]=  0.0;  T[3][1]=  0.0;  T[3][2]=  0.0;  T[3][3]=  1.0;

       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(R, axis, M_PI/8.0);
       p3d_matMultXform(R, T, Thand_wrist);

       translation_step= 0.01;
       rotation_step= 2*M_PI/5;
       nb_directions= 12;
       max_nb_grasp_frames= 1000;
    break;
    case GP_SAHAND_RIGHT:
       nb_fingers= 4;
       nb_dofs= 13;
       length_thumbBase =   0.0855;
       length_proxPha   =   0.067816;
       length_midPha    =   0.029980;
       length_distPha   =   0.029;
       fingertip_radius =   0.013;

       //compute the frames at the base of each finger:
       axis[0]=   0;
       axis[1]=   1;
       axis[2]=   0;
       p3d_mat4Copy(p3d_mat4IDENTITY, T);
       p3d_mat4Rot(T, axis, 20*DEGTORAD);
       T[0][3]= 0.07;
       T[1][3]= 0.02;
       T[2][3]= 0.22;

       p3d_matInvertXform(T, Tgrasp_frame_hand);

       //initial axis swap (x'= -y, y'= z, z'= -x)
       axis[0]=  1;
       axis[1]= -1;
       axis[2]= -1;
       p3d_mat4Rot(Trinit, axis, 120*DEGTORAD);

       ////////////////////////////thumb///////////////////////////////////
       t[0]=  -0.0271;
       t[1]=        0;
       t[2]=    0.003;
       p3d_mat4Trans(Tt1, t);

       axis[0]=  0;
       axis[1]= -1;
       axis[2]=  0;
       p3d_mat4Rot(Tr1, axis, 90*DEGTORAD);

       t[0]=  -0.1126 - t[0];
       t[1]=  0.10282 - t[1];
       t[2]=    0.003 - t[2];
       p3d_mat4Trans(Tt2, t);

       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(Tr2, axis, 35.2*DEGTORAD);

       axis[0]= 1;
       axis[1]= 0;
       axis[2]= 0;
       p3d_mat4Rot(Tr3, axis, -0.7*DEGTORAD);

       axis[0]= 0;
       axis[1]= 1;
       axis[2]= 0;
       p3d_mat4Rot(Tr4, axis, 270*DEGTORAD);

       p3d_matMultXform(Trinit, Tt1, Tint1);
       p3d_matMultXform(Tint1, Tr1, Tint2);
       p3d_matMultXform(Tint2, Tt2, Tint1);
       p3d_matMultXform(Tint1, Tr2, Tint2);
       p3d_matMultXform(Tint2, Tr3, Tint1);
       p3d_matMultXform(Tint1, Tr4, Twrist_thumb);

       ///////////////////////////forefinger///////////////////////////////////
       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(Tr1, axis, 2.2*DEGTORAD);
       t[0]= -0.04025;
       t[1]=  0.15584;
       t[2]=    0.003;
       p3d_mat4Trans(Tt1, t);

       p3d_matMultXform(Trinit, Tt1, Tint1);
       p3d_matMultXform(Tint1, Tr1, Twrist_forefinger);


       ///////////////////////////middle finger///////////////////////////////////
       t[0]=        0;
       t[1]=  0.16056;
       t[2]=    0.003;
       p3d_mat4Trans(Tt1, t);
       p3d_matMultXform(Trinit, Tt1, Twrist_middlefinger);

       ///////////////////////////ring finger///////////////////////////////////
       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(Tr1, axis, -1.76*DEGTORAD);
       t[0]= 0.04025;
       t[1]= 0.15584;
       t[2]=   0.003;
       p3d_mat4Trans(Tt1, t);

       p3d_mat4Mult(Trinit, Tt1, Tint1);
       p3d_mat4Mult(Tint1, Tr1, Twrist_ringfinger);


       // joint bounds
       for(i=0; i<4; i++)
       {
         q0min[i]=            0;
         q0max[i]=  90*DEGTORAD;
         q1min[i]= -20*DEGTORAD;
         q1max[i]=  20*DEGTORAD;
         q2min[i]= -19*DEGTORAD;
         q2max[i]=  90*DEGTORAD;
         q3min[i]=            0;
         q3max[i]=  90*DEGTORAD;
       }
       //for the thumb:
       q2min[0]= -19*DEGTORAD;
       q2max[0]=  90*DEGTORAD;

       T[0][0]=  0.0;   T[0][1]=  1.0;   T[0][2]=  0.0;   T[0][3]=  0.0;
       T[1][0]=  0.0;   T[1][1]=  0.0;   T[1][2]=  1.0;   T[1][3]=  0.0;
       T[2][0]=  1.0;   T[2][1]=  0.0;   T[2][2]=  0.0;   T[2][3]=  0.14;
       T[3][0]=  0.0;   T[3][1]=  0.0;   T[3][2]=  0.0;   T[3][3]=  1.0;

       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(R, axis, M_PI_2);
       p3d_mat4Mult(R, T, Thand_wrist);

       translation_step= 0.02;
       rotation_step= 2*M_PI/3;
       nb_directions= 6;
       max_nb_grasp_frames= 500;
    break;
    default:
       printf("%s: %d: gpHand_properties::initalize(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return NULL;
    break;
  }

  return hand_robot;
}


//! Draws different things that allow to visualize the dimension settings in a gpHand_properties class
//! by comparing the displayed things to the display of the Move3D model. Must be used in an OpenGL context.
//! \param pose the frame the things will be drawn in relatively to. Use the robot's wrist frame for instance.
//! \return 1 in case of success, 0 otherwise
int gpHand_properties::draw(p3d_matrix4 pose)
{
  GLboolean lighting_enable;
  int result= 1;
  GLint line_width;
  float matGL[16];
  p3d_matrix4 Tgrasp_frame_hand_inv;

  glGetIntegerv(GL_LINE_WIDTH, &line_width);
  glGetBooleanv(GL_LIGHTING, &lighting_enable);
  p3d_matrix4_to_OpenGL_format(pose, matGL);

  glPushMatrix();
   glMultMatrixf(matGL);

    switch(type)
    {
      case GP_GRIPPER:
        p3d_matInvertXform(Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
        draw_frame(Tgrasp_frame_hand_inv, 0.1);

        g3d_set_color_mat(Red, NULL);
        g3d_draw_solid_sphere(0.5*min_opening,-0.5*fingertip_distance,  0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(0.5*max_opening,-0.5*fingertip_distance,  0.065, fingertip_radius, 20);
        g3d_set_color_mat(Green, NULL);
        g3d_draw_solid_sphere(0.5*min_opening,0.5*fingertip_distance, 0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(0.5*max_opening,0.5*fingertip_distance, 0.065, fingertip_radius, 20);
        g3d_set_color_mat(Blue, NULL);
        g3d_draw_solid_sphere(-0.5*min_opening,0.0,  0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(-0.5*max_opening,0.0,  0.065, fingertip_radius, 20);
      break;
      case GP_SAHAND_RIGHT:
        p3d_matInvertXform(Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
        draw_frame(Tgrasp_frame_hand_inv, 0.1);
        draw_frame(Twrist_thumb, 0.05);
        draw_frame(Twrist_forefinger, 0.05);
        draw_frame(Twrist_middlefinger, 0.05);
        draw_frame(Twrist_ringfinger, 0.05);

        p3d_matrix4_to_OpenGL_format(Twrist_thumb, matGL);
        glPushMatrix();
         glMultMatrixf(matGL);
         glRotatef(-90, 1.0, 0.0, 0.0);
         g3d_set_color_mat(Red, NULL);
         glTranslatef(0, 0, 0.5*length_proxPha);
         g3d_draw_solid_cylinder(fingertip_radius, length_proxPha, 10);
         g3d_set_color_mat(Green, NULL);
         glTranslatef(0, 0, 0.5*(length_proxPha + length_midPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_midPha, 10);
         g3d_set_color_mat(Blue, NULL);
         glTranslatef(0, 0, 0.5*(length_midPha + length_distPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_distPha, 10);
        glPopMatrix();

        p3d_matrix4_to_OpenGL_format(Twrist_forefinger, matGL);
        glPushMatrix();
         glMultMatrixf(matGL);
         glRotatef(-90, 1.0, 0.0, 0.0);
         g3d_set_color_mat(Red, NULL);
         glTranslatef(0, 0, 0.5*length_proxPha);
         g3d_draw_solid_cylinder(fingertip_radius, length_proxPha, 10);
         g3d_set_color_mat(Green, NULL);
         glTranslatef(0, 0, 0.5*(length_proxPha + length_midPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_midPha, 10);
         g3d_set_color_mat(Blue, NULL);
         glTranslatef(0, 0, 0.5*(length_midPha + length_distPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_distPha, 10);
        glPopMatrix();


        p3d_matrix4_to_OpenGL_format(Twrist_middlefinger, matGL);
        glPushMatrix();
         glMultMatrixf(matGL);
         glRotatef(-90, 1.0, 0.0, 0.0);
         g3d_set_color_mat(Red, NULL);
         glTranslatef(0, 0, 0.5*length_proxPha);
         g3d_draw_solid_cylinder(fingertip_radius, length_proxPha, 10);
         g3d_set_color_mat(Green, NULL);
         glTranslatef(0, 0, 0.5*(length_proxPha + length_midPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_midPha, 10);
         g3d_set_color_mat(Blue, NULL);
         glTranslatef(0, 0, 0.5*(length_midPha + length_distPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_distPha, 10);
        glPopMatrix();

        p3d_matrix4_to_OpenGL_format(Twrist_ringfinger, matGL);
        glPushMatrix();
         glMultMatrixf(matGL);
         glRotatef(-90, 1.0, 0.0, 0.0);
         g3d_set_color_mat(Red, NULL);
         glTranslatef(0, 0, 0.5*length_proxPha);
         g3d_draw_solid_cylinder(fingertip_radius, length_proxPha, 10);
         g3d_set_color_mat(Green, NULL);
         glTranslatef(0, 0, 0.5*(length_proxPha + length_midPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_midPha, 10);
         g3d_set_color_mat(Blue, NULL);
         glTranslatef(0, 0, 0.5*(length_midPha + length_distPha));
         g3d_draw_solid_cylinder(fingertip_radius, length_distPha, 10);
        glPopMatrix();
      break;
      default:
       printf("%s: %d: gpHand_properties::draw(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       result= 0;
      break;
    }

  glPopMatrix();

  glLineWidth(line_width);
  if(lighting_enable)
  {  glEnable(GL_LIGHTING);  }
  else
  {  glDisable(GL_LIGHTING);  }


  return result;
}


//! Computes a set of grasps from a grasp frame. Generic function.
//! \param robot the hand robot (a freeflying robot composed of hand bodies only)
//! \param object the object to grasp
//! \param part the object part to grasp (all the object mesh triangles that have the same value in their "part" field). Set to 0 if unused (all the triangles will be considered).
//! \param gframe the grasp frame (a 4x4 homogeneous transform matrix)
//! \param hand variable containing information about the hand geometry
//! \param graspList a grasp list the computed set of grasps will be added to
//! \return 1 in case of success, 0 otherwise
int gpGrasps_from_grasp_frame(p3d_rob *robot, p3d_obj *object, int part, p3d_matrix4 gFrame, gpHand_properties &hand, std::list<class gpGrasp> &graspList)
{
   #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpGrasps_from_grasp_frame(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
   if(object==NULL)
   {
      printf("%s: %d: gpGrasps_from_grasp_frame(): object is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
   #endif

   switch( hand.type )
   {
      case GP_GRIPPER:
         return gpGrasps_from_grasp_frame_gripper(object->pol[0]->poly, part, gFrame, hand, graspList);
      break;
      case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
         return gpGrasps_from_grasp_frame_SAHand(robot, object, part, gFrame, hand, graspList);
      break;
      default:
         printf("%s: %d: gpGrasps_from_grasp_frame(): this hand model is not defined.\n",__FILE__,__LINE__);
         return 0;
      break;
   }

   return 0;
}


//! Computes a set of grasps from a grasp frame for the SAHand.
//! \param robot the hand robot (a freeflying robot composed of hand bodies only)
//! \param object the object to grasp
//! \param part the object part to grasp (all the object mesh triangles that have the same value in their "part" field). Set to 0 if unused (all the triangles will be considered).
//! \param gframe the grasp frame (a 4x4 homogeneous transform matrix)
//! \param hand variable containing information about the hand geometry
//! \param graspList a grasp list the computed set of grasps will be added to
//! \return 1 in case of success, 0 otherwise
int gpGrasps_from_grasp_frame_SAHand(p3d_rob *robot, p3d_obj *object, unsigned int part, p3d_matrix4 gFrame, gpHand_properties &hand, std::list<class gpGrasp> &graspList)
{
  #ifdef DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: gpGrasps_from_grasp_frame_SAHand(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  if(object==NULL)
  {
     printf("%s: %d: gpGrasps_from_grasp_frame_SAHand(): object is NULL.\n",__FILE__,__LINE__);
     return 0;
  }
  #endif

  //printf("gpGrasps_from_grasp_frame_SAHand()\n");

  unsigned int i, nb_samples;
  int j, k, nb_faces;
  p3d_index *indices;
  double fingertip_radius;
  p3d_polyhedre *poly= NULL;
  p3d_vector3 *points= NULL, *surf_points= NULL;
  p3d_face *faces;
  p3d_vector3 p, contact_normal, fingerpad_normal;
  double q[4][4];
  p3d_matrix4 objectFrame, Twrist, Twrist_grasp_frame_inv;
  configPt config0= NULL, config= NULL;

  std::vector<gpContact> contacts;
  gpGrasp grasp;

  p3d_matInvertXform(hand.Tgrasp_frame_hand, Twrist_grasp_frame_inv);
 // p3d_mat4Mult(gFrame, Twrist_grasp_frame_inv, Twrist);
  p3d_mat4Mult(gFrame, hand.Tgrasp_frame_hand, Twrist);
 // p3d_mat4Copy(gFrame, Twrist);


  poly= object->pol[0]->poly;
  points= poly->the_points;
  nb_faces=  poly->nb_faces;
  faces= poly->the_faces;
  fingertip_radius= hand.fingertip_radius;
  contacts.reserve(4);


  //memorize current robot configuration:
  config0= p3d_alloc_config(robot);

  config0= p3d_get_robot_config(robot);


  p3d_get_obj_pos(object, objectFrame);
  config= p3d_alloc_config(robot);
  gpInverse_geometric_model_freeflying_hand(robot, objectFrame, Twrist, hand, config);
  p3d_set_and_update_this_robot_conf(robot, config);



  // Deactivate the collision for all fingers
  gpDeactivate_finger_collisions(robot, 1, hand);
  gpDeactivate_finger_collisions(robot, 2, hand);
  gpDeactivate_finger_collisions(robot, 3, hand);
  gpDeactivate_finger_collisions(robot, 4, hand);


  // If the hand robot is already colliding with the object, it's the palm and there is no way
  // to find a collision-free configuration:
  if(p3d_col_test_robot_obj(robot, object))
  {
//p3d_obj *obj1, *obj2;
//pqp_colliding_pair(&obj1, &obj2);
//printf("colliding pair= %s %s \n", obj1->name, obj2->name);

    gpActivate_finger_collisions(robot, 1, hand);
    gpActivate_finger_collisions(robot, 2, hand);
    gpActivate_finger_collisions(robot, 3, hand);
    gpActivate_finger_collisions(robot, 4, hand);
    p3d_set_and_update_this_robot_conf(robot, config0);
    p3d_destroy_config(robot, config0);
    p3d_destroy_config(robot, config); //printf("palm colliding\n");
    return 0;
  }


  for(i=0; i<4; i++) //for each finger:
  {
     gpActivate_finger_collisions(robot, i+1, hand);

     for(j=0; j<nb_faces; j++) //for each triangle:
     {
        if(part!=0 && faces[j].part!=part)
        {  continue;  }


        indices= faces[j].the_indexs_points;


        surf_points= gpSample_triangle_surface(points[indices[0]-1], points[indices[1]-1], points[indices[2]-1], fingertip_radius/5.0, &nb_samples);


        p3d_vectScale(faces[j].plane->normale, contact_normal, fingertip_radius);

        glDisable(GL_LIGHTING);
        glPointSize(5);
        glColor3f(1, 0, 0);
/*
        for(k=0; k<nb_samples; k++) //for each surface point:
        {
          glBegin(GL_POINTS);
           glVertex3dv(surf_points[k]);
          glEnd();
        }*/
        glEnable(GL_LIGHTING);

        for(k=0; k<nb_samples; k++) //for each surface point:
        {
          p3d_vectAdd(surf_points[k], contact_normal, p);
   // printf("sample %f %f %f\n", surf_points[k][0], surf_points[k][1], surf_points[k][2]);
          glColor3f(1, 0, 0);
          glBegin(GL_POINTS);
           glVertex3dv(p);
          glEnd();


          if(gpSAHfinger_inverse_kinematics(Twrist, hand, p, q[i], fingerpad_normal, i+1)==1)
          {
//printf("IK doigt %d OK\n", i+1);
            // contact normal and fingerpad normal must be in opposite directions:
            if( p3d_vectDotProd(contact_normal, fingerpad_normal) > 0 )
            {  continue;  }

            gpSet_SAHfinger_joint_angles(robot, hand, q[i], i+1);

 //printf("col test %d %d\n", p3d_col_test_robot_obj(robot, object), p3d_col_test_self_collision(robot, 0));
            if( p3d_col_test_robot_obj(robot, object) )
            {  continue;  }

            if( p3d_col_test_self_collision(robot, 0) )
            {  continue;  }


            if( (!p3d_col_test_robot_obj(robot, object)) && (!p3d_col_test_self_collision(robot, 0)) )
            {
//printf("contact found\n");
              contacts.resize(contacts.size()+1);

              contacts.back().position[0]= surf_points[k][0];
              contacts.back().position[1]= surf_points[k][1];
              contacts.back().position[2]= surf_points[k][2];

              contacts.back().normal[0]= faces[j].plane->normale[0];
              contacts.back().normal[1]= faces[j].plane->normale[1];
              contacts.back().normal[2]= faces[j].plane->normale[2];

              contacts.back().surface= poly;
              contacts.back().face= j;
              contacts.back().fingerID= i+1;

              j= nb_faces;
              break;
            }
          }
        }

        free(surf_points);
        surf_points= NULL;
     }

     if(i==0 && contacts.size()==0)
     {
      // printf("no contact found for the thumb\n");
       gpActivate_finger_collisions(robot, 1, hand);
       gpActivate_finger_collisions(robot, 2, hand);
       gpActivate_finger_collisions(robot, 3, hand);
       gpActivate_finger_collisions(robot, 4, hand);
       p3d_set_and_update_this_robot_conf(robot, config0);
       p3d_destroy_config(robot, config0);
       p3d_destroy_config(robot, config);
       return 0;
     }

  }


  if(contacts.size() < 3)
  {
    printf("only %d contact(s) were found\n", contacts.size());
    gpActivate_finger_collisions(robot, 1, hand);
    gpActivate_finger_collisions(robot, 2, hand);
    gpActivate_finger_collisions(robot, 3, hand);
    gpActivate_finger_collisions(robot, 4, hand);
    p3d_set_and_update_this_robot_conf(robot, config0);
    p3d_destroy_config(robot, config0);
    p3d_destroy_config(robot, config);
    return 0;
  }


  //test collision for the whole hand configuration:
  gpActivate_finger_collisions(robot, 1, hand);
  gpActivate_finger_collisions(robot, 2, hand);
  gpActivate_finger_collisions(robot, 3, hand);
  gpActivate_finger_collisions(robot, 4, hand);
  if( p3d_col_test_robot_obj(robot, object) || p3d_col_test_self_collision(robot, 0) )
  {
    p3d_set_and_update_this_robot_conf(robot, config0);
    p3d_destroy_config(robot, config0);
    p3d_destroy_config(robot, config);
    return 0;
  }

/*
  for(i=0; i<contacts.size(); i++)
  {
    printf("\t contact[%d]= finger %d \n", i, contacts[i].fingerID);
  }
*/

  grasp.hand_type= hand.type;
  grasp.ID= graspList.size() + 1;
  grasp.object= object;
  p3d_mat4Copy(gFrame, grasp.frame);
  grasp.config.resize(13);

  grasp.contacts.resize(contacts.size());

  for(i=0; i<grasp.contacts.size(); i++)
  {
    grasp.contacts[i]   = contacts[i];
    grasp.contacts[i].mu= GP_FRICTION_COEFFICIENT;
  }


  grasp.config[0]= M_PI_2; // thumb's first DOF
  for(i=0; i<4; i++)
  {
    gpGet_SAHfinger_joint_angles(robot, hand, q[i], i+1);
    grasp.config[3*i+1]= q[i][1];
    grasp.config[3*i+2]= q[i][2];
    grasp.config[3*i+3]= q[i][3];
  }


  p3d_set_and_update_this_robot_conf(robot, config0);
  p3d_destroy_config(robot, config0);
  p3d_destroy_config(robot, config);

  graspList.push_back(grasp);

  return 1;
}


//! Cette fonction calcule, pour la pince à 3 doigts, les trois positions des doigts obtenus
//! à partir d'un repère de prise ainsi qu'un repère lié aux points de contact.
//! La fonction retourne 0 au cas où il n'y a pas de solution, 1 sinon.
//! L'argument 'part' sert à sélectionner une partie de l'objet (ensemble de facettes)
//! dans le cas où il a été segmenté par ailleurs. Par défaut, on la laisse à 0 et toutes les facettes
//! seront considérées.
//! Principe général (plus de détails et des figures dans la thèse d'Efrain Lopez Damian:
//! "Grasp planning for object manipulation by an autonomous robot"):
//! On part de la donnée d'un repère de prise (Oxyz).
//! On définit le plan de prise par (Oxy).
//! Le premier point de contact (p1) est donné par l'intersection de l'axe Ox avec la surface de l'objet
//! (intersection entre une demi-droite et un des triangles du polyèdre). Ce point est décalé
//! dans la direction de la normale à la surface d'une distance
//! égale au rayon Rf des doigts (les doigts sont hémisphériques) pour donner p1',
//! la position du centre du doigt 1.
//! Pour le second point (p2), on cherche d'abord l'intersection entre le plan de prise et
//! les facettes déplacées d'une distance R dans la direction de la normale à la surface.
//! Les segments obtenus doivent intersecter un cercle de centre p1 et de rayon R où R est le rayon entre
//! les deux doigts du même côté de la paume (ceux des contacts 1 et 2). Le point d'intersection est p2'.
//! Comme il y a deux possibilités, on prend p2' tel que p1'p2' soit dans le même sens que l'axe (Oy).
//! On prend alors p1'p2' comme nouvel axe Oy du repère de prise (après normalisation).
//! Le nouvel axe Z est la normale au plan formé par les point (O, p1',p2').
//! p3 est alors l'intersection entre le rayon partant du milieu de p1'p2'
//! et de direction égale à celle de l'axe Ox du nouveau repère.
//! p3' est obtenu en décalant p3 de Rf dans la direction de la normale à la surface.
//! On choisit alors une nouvelle origine pour le nouveau repère de prise:
//! le milieu du segment formé par le milieu de p1'p2' et p3'.
//! NOTE: le plan défini par les trois points de contact n'est pas forcément le plan de prise initiale.
//! Il va dépendre des normales des faces intersectées par le plan de prise initial.
//! Plusieurs prises peuvent être obtenue à partir d'un même repère de prise.
//! Compute a set of grasps for a given grasp frame for the gripper.
//! \param polyhedron the polyhedral mesh of the object surface
//! \param part if the object has been previously segmented, "part" is used to select a part of the object (a set of triangles). Set to 0 to select all the triangles.
//! \param gFrame the grasp frame (a 4x4 homogeneous transform matrix)
//! \param hand structure containing information about the hand geometry
//! \param graspList a grasp list the computed set of grasps will be added to
//! \return 1 if grasps were found and added to the list, 0 otherwise
int gpGrasps_from_grasp_frame_gripper(p3d_polyhedre *polyhedron, unsigned int part, p3d_matrix4 gFrame, gpHand_properties &hand, std::list<class gpGrasp> &graspList)
{
    #ifdef DEBUG
    if(polyhedron==NULL)
    {
      printf("%s: %d: gpGrasp_from_grasp_frame_gripper(): polyhedron=NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(hand.type != GP_GRIPPER)
    {
      printf("%s: %d: gpGrasp_from_grasp_frame_gripper(): this function can not be applied to this hand model.\n",__FILE__,__LINE__);
     return 0;
    }
    #endif

    unsigned int i, j, k;
    p3d_vector3 origin, new_origin, xAxis, yAxis, zAxis, new_xAxis, new_xAxis_neg, new_yAxis, new_zAxis;
    p3d_vector3 px, py, shift, pinter1, pinter2, result2, middle_point;
    p3d_plane gPlane;

    p3d_vector3 p1, p2, p3; //Les positions des doigts (de leur centre)
    p3d_vector3 p1_s, p2_s, p3_s; //points de contacts sur la surface de l'objet;
                                  //par rapport à l'explication: p_s= p   et p= p'
    p3d_vector3 p1p3_s;
    double distance_p1p2 = hand.fingertip_distance; //distance entre les deux premiers doigts
                                                     //(ceux du même côté de la paume)
    double fingertip_radius= hand.fingertip_radius; //rayon des doigts
    double max_opening= hand.max_opening;            //ouverture maximale de la pince
                                                      //(mouvement de translation)

    //distance maximale possible entre les contacts 1 et 3:
    double max_distance_p1p3_s= sqrt( SQR(0.5*distance_p1p2) + SQR(max_opening) );

    int nbinter= 0;
    poly_index *ind;

    for(i=0; i<3; i++)
    {
      origin[i]= gFrame[i][3];
      xAxis[i] = gFrame[i][0];
      yAxis[i] = gFrame[i][1];
      zAxis[i] = gFrame[i][2];
    }

    p3d_vectAdd(origin, xAxis, px);
    p3d_vectAdd(origin, yAxis, py);

    //le plan de prise:
    gPlane= gpPlane_from_points(origin, px, py);

    gpGrasp grasp;
    //la liste des prises qui sera retournée:
    //LList *graspList= create_LList(DATA_LIST, (delete_function) gpDestroy_grasp);
  //  std::list<gpGrasp> *graspList= NULL;
   // graspList= new std::list<gpGrasp>;

    gpContact contact;
    //tableaux contenant les contacts trouvés pour les doigts 1 et 2 :
    //gpContact *contacts1, *contacts2;
    std::vector<gpContact> contacts1;
    contacts1.reserve(5);
    std::vector<gpContact> contacts2;
    contacts2.reserve(5);

   // int max_nb_contacts= 5; //nombre maximal de contacts alloués pour les doigts 1 et 2 (ils sont réalloués si besoin est)
    unsigned int nb_contacts12= 0; //nombre actuel de paires de contacts trouvées pour les doigts 1 et 2

    int nb_grasps= 0; //nombre actuel de prises trouvées (contacts des doigts 1, 2 et 3)
    bool isNeighbourIntersected= false;

   // contacts1= new gpContact[max_nb_contacts];
   // contacts2= new gpContact[max_nb_contacts];

    p3d_vector3 *points= polyhedron->the_points;
    unsigned int nb_faces=  (unsigned int) polyhedron->nb_faces;
    p3d_face *faces= polyhedron->the_faces;


    ///////////////////////////premier point de contact///////////////////////////
    for(i=0; i<nb_faces; i++)
    {
        if(part!=0 && faces[i].part!=part)
           continue;

	ind= faces[i].the_indexs_points;


        // On cherche des faces dont la normale est orientée dans le même sens que l'axe X:
        if( p3d_vectDotProd(faces[i].plane->normale, xAxis) < 0 )
            continue;

        // Pour éviter de prendre en compte plusieurs fois le même point de contact si l'axe X coupe des triangles différents
        // en un même point (sur un de leurs sommets ou arêtes communs), on regarde parmi les points de contacts trouvés
        // si l'un d'entre eux n'est pas sur un des voisins du triangle courant:
        isNeighbourIntersected= false;
        for(j=0; j<nb_contacts12; j++)
        {
          for(k=0; k<3; k++)
          {
             if(faces[i].neighbours[k]!=-1 && contacts1[j].face==(unsigned int) faces[i].neighbours[k])
             //if(contacts1[j].face==faces[i].neighbours[k])
             {
               isNeighbourIntersected= true;
               break;
             }
          }
          if(isNeighbourIntersected)
            break;
         }

        if(isNeighbourIntersected)
          continue;

        // On teste maintenant l'intersection triangle courant axe X:
        nbinter= gpLine_triangle_intersection(origin, px, points[ind[0]-1], points[ind[1]-1], points[ind[2]-1], p1_s);


        if(nbinter!=0)
        {
         // shift = fingertip_radius*normale:
         p3d_vectScale(faces[i].plane->normale, shift, fingertip_radius);

         // Le centre du premier doigt:
         p3d_vectAdd(p1_s, shift, p1);

          ///////////////////////////recherche d'un deuxième point de contact///////////////////////////
          for(j=0; j<nb_faces; j++)
          {
	      ind= faces[j].the_indexs_points;

              // Les deux premiers contacts doivent avoir des normales dans des directions non
              // opposées:
              if( p3d_vectDotProd(faces[i].plane->normale, faces[j].plane->normale) < 0 )
                continue;

              nbinter= gpTriangle_plane_intersection(points[ind[0]-1], points[ind[1]-1], points[ind[2]-1], gPlane, pinter1, pinter2);
              if( nbinter != 2 )
                continue;

              // shift = fingertip_radius*normale:
              p3d_vectScale(faces[j].plane->normale, shift, fingertip_radius);


              // décalage selon la normale à la surface:
              p3d_vectAdd(pinter1, shift, pinter1);
              p3d_vectAdd(pinter2, shift, pinter2);

              // Le deuxième point de contact est dans p2:
              nbinter= gpLine_segment_sphere_intersection(pinter1, pinter2, p1_s, distance_p1p2, p2, result2);

              if(nbinter==0)
                  continue;
              else
              {
                  // calcul du point de contact sur la face:
                  p3d_vectSub(p2, shift, p2_s);

                  // calcul du nouvel axe Y (axe p1-p2):
                  p3d_vectSub(p2, p1, new_yAxis);
                  p3d_vectNormalize(new_yAxis, new_yAxis);

                  if( p3d_vectDotProd(new_yAxis, yAxis) > 0 )
                      break;
                  else
                  {
                    if(nbinter==2) // s'il y avait un deuxième point d'intersection
                    {
                        p3d_vectCopy(result2, p2);
                        p3d_vectSub(p2, shift, p2_s);

                        // calcul du nouvel axe Y (axe p1-p2):
                        p3d_vectSub(p2, p1, new_yAxis);
                        p3d_vectNormalize(new_yAxis, new_yAxis);
                        if( p3d_vectDotProd(new_yAxis, yAxis) > 0 )
                          break;
                    }
                  }
              }
          }

          if(j<nb_faces)
          {
            // on a trouvé une paire p1p2:
            contact.surface= polyhedron;
            contact.face= i;
            p3d_vectCopy(p1_s, contact.position);
            p3d_vectCopy(faces[i].plane->normale, contact.normal);
            contact.mu= GP_FRICTION_COEFFICIENT;
            contacts1.push_back(contact);

            contact.surface= polyhedron;
            contact.face= j;
            p3d_vectCopy(p2_s, contact.position);
            p3d_vectCopy(faces[j].plane->normale, contact.normal);
            contact.mu= GP_FRICTION_COEFFICIENT;
            contacts2.push_back(contact);

            nb_contacts12++;
          }

        }

    }


    if( nb_contacts12==0 ) //pas d'intersection (le repère de saisie est hors du volume de l'objet)
    {
       contacts1.clear();
       contacts2.clear();
       return 0;
    }


    ///////////////////////////troisième point de contact///////////////////////////
    for(i=0; i<nb_contacts12; i++)
    {
      p3d_vectCopy(contacts1[i].position, p1_s);
      p3d_vectScale(contacts1[i].normal, shift, fingertip_radius);
      p3d_vectAdd(p1_s, shift, p1);

      p3d_vectCopy(contacts2[i].position, p2_s);
      p3d_vectScale(contacts2[i].normal, shift, fingertip_radius);
      p3d_vectAdd(p2_s, shift, p2);

      //  Calcul des nouveaux axes:
      //  nouvel axe Y
      p3d_vectSub(p2, p1, new_yAxis);
      p3d_vectNormalize(new_yAxis, new_yAxis);

      //  nouvel axe Z (normale au plan formé par les points (origine du repère initial, p1, p2))
      //  NB: on doit changer d'axe Z car le nouvel axe Y calculé plus haut n'est pas forcément orthogonal à l'ancien axe Z.
      p3d_plane plane= gpPlane_from_points(origin, contacts1[i].position, contacts2[i].position);
      p3d_vectCopy(plane.normale, new_zAxis);
      p3d_vectNormalize(new_zAxis, new_zAxis);
      if(p3d_vectDotProd(zAxis, new_zAxis) < 0.0)
      { p3d_vectNeg(new_zAxis, new_zAxis);  }

      //calcul du nouvel axe X
      p3d_vectXprod(new_yAxis, new_zAxis, new_xAxis);   //p3d_vectXprod(new_yAxis, zAxis, new_xAxis);
      p3d_vectNeg(new_xAxis, new_xAxis_neg); //On va chercher l'intersection avec l'axe -(Ox)


      middle_point[0]= ( p1[0] + p2[0] )/2;
      middle_point[1]= ( p1[1] + p2[1] )/2;
      middle_point[2]= ( p1[2] + p2[2] )/2;


      for(j=0; j<nb_faces; j++)
      {
        //il ne faut pas réintersecter la face du point p1 ni celle du point p2
        if( j==contacts1[i].face || j==contacts2[i].face )
          continue;

        //Plus généralement, comme le point de départ du rayon est hors de la surface,
        // il faut s'assurer que l'intersection se fait
        //du bon côté de la surface de l'objet (c'est-à-dire pas du même côté que p1 et p2).
        //La face doit être intersectée par le rayon du côté intérieur de l'objet:
        if(  p3d_vectDotProd(faces[j].plane->normale, new_xAxis_neg) < 0 )
          continue;

        ind= faces[j].the_indexs_points;
        nbinter= gpRay_triangle_intersection(middle_point, new_xAxis_neg, points[ind[0]-1], points[ind[1]-1], points[ind[2]-1], p3_s);

        if(nbinter==1)
        {
          // shift = fingertip_radius*normale:
          p3d_vectScale(faces[j].plane->normale, shift, fingertip_radius);

          // décalage selon la normale à la surface:
          p3d_vectAdd(p3_s, shift, p3);

          p3d_vectSub(p3_s, p1_s, p1p3_s);

          if( p3d_vectNorm(p1p3_s)  > max_distance_p1p3_s ) //les contacts sont trop éloignés pour la pince
          {
            continue;
          }
          else
          { // une prise a été trouvée:

            // calcul de la nouvelle origine:
            new_origin[0]= ( middle_point[0] + p3[0] )/2;
            new_origin[1]= ( middle_point[1] + p3[1] )/2;
            new_origin[2]= ( middle_point[2] + p3[2] )/2;

            nb_grasps++;

            grasp.hand_type= GP_GRIPPER;
            grasp.ID= graspList.size() + 1;
            grasp.config.resize(1);

            //grasp.finger_opening= sqrt( pow(p3d_vectNorm(p1p3_s), 2) - pow(distance_p1p2/2.0,2) );

            //middle of contact1-contact2:
            middle_point[0]= 0.5*( contacts1[i].position[0] + contacts2[i].position[0] );
            middle_point[1]= 0.5*( contacts1[i].position[1] + contacts2[i].position[1] );
            middle_point[2]= 0.5*( contacts1[i].position[2] + contacts2[i].position[2] );

            grasp.finger_opening= sqrt( SQR(p3_s[0]-middle_point[0]) + SQR(p3_s[1]-middle_point[1]) + SQR(p3_s[2]-middle_point[2]) )+2*hand.fingertip_radius;

            //La configuration de la pince est calculée telle qu'elle se ferme sur l'objet. Pour les tests
            //de collision, il faudra l'ouvrir un peu plus.
            grasp.config[0]= hand.min_opening_jnt_value + ( (grasp.finger_opening - hand.min_opening)/(hand.max_opening - hand.min_opening) )*(hand.max_opening_jnt_value - hand.min_opening_jnt_value);

            if(isnan(grasp.finger_opening))
            {
              grasp.finger_opening= hand.min_opening;
              grasp.config[0]= hand.min_opening_jnt_value;
            }

            for(k=0; k<3; k++)
            {
              grasp.frame[k][0]= new_xAxis[k];
              grasp.frame[k][1]= new_yAxis[k];
              grasp.frame[k][2]= new_zAxis[k];
              grasp.frame[k][3]= new_origin[k];
              grasp.frame[3][k]=   0;
            }
            grasp.frame[3][3]=  1;

            //grasp.contacts= (gpContact *) malloc(3*sizeof(gpContact));
            grasp.contacts.resize(3);
            grasp.polyhedron= polyhedron;
            grasp.object= NULL;

            grasp.contacts[0]= contacts1[i];
            grasp.contacts[0].fingerID= 1;

            grasp.contacts[1]= contacts2[i];
            grasp.contacts[1].fingerID= 2;


            grasp.contacts[2].surface= polyhedron;
            grasp.contacts[2].face= j;
            grasp.contacts[2].fingerID= 3;

            p3d_vectCopy(p3_s, grasp.contacts[2].position);
            p3d_vectCopy(faces[j].plane->normale, grasp.contacts[2].normal);
            grasp.contacts[2].mu= GP_FRICTION_COEFFICIENT;

            graspList.push_back(grasp);
          }

        }

      }


    }


    contacts1.clear();
    contacts2.clear();

    if(nb_grasps==0)
    {  return 0;  }
    else
    { return 1; }

}


//! \deprecated
// Cette fonction calcule un repère de prise (matrice 4x4).
// Elle part du repère formé par les axes principaux d'inertie (iaxes),
// centré sur le centre de gravité de l'objet (cmass)
// Une translation est appliquée à ce repère dans une des 6 directions possibles
// (on en choisit une avec "direction",
// un entier entre 1 et 6) ainsi qu'une rotation d'axe donné par l'entier "axis" (1,2 ou 3 pour x,y ou z).
// Le déplacement de la translation est "displacement" et l'angle de la rotation "angle".
// Le résultat est recopié dans gframe.
// NOTE: pour assurer la cohérence avec la façon dont on calcule une prise à partir d'un repère de prise,
// la fonction fait coïncider l'axe z du repère de prise avec la direction
// dans laquelle s'effectue le mouvement de translation du centre du repère.
int gpGrasp_frame_from_inertia_axes(p3d_matrix3 iaxes, p3d_vector3 cmass, int direction, double displacementX, double displacementY, int axis, double angle, p3d_matrix4 gframe)
{
   int i;
   p3d_matrix4 frame, Mtransf;
   p3d_vector3 rotAxis;

   //repère initial:
   frame[0][3]= cmass[0];
   frame[1][3]= cmass[1];
   frame[2][3]= cmass[2];
   frame[3][0]=        0;  frame[3][1]=  0; frame[3][2]=   0; frame[3][3]=   1;

   switch(direction)
   {
     case 1:  // x'= x      y'= y     z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]= iaxes[i][0];
            frame[i][1]= iaxes[i][1];
            frame[i][2]= iaxes[i][2];    }
     break;
     case 2:  // x'= -x     y'= -y      z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]=  -iaxes[i][0];
            frame[i][1]=  -iaxes[i][1];
            frame[i][2]=   iaxes[i][2];  }
     break;

     case 3: // x'= y       y'= -x     z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]=   iaxes[i][1];
            frame[i][1]=  -iaxes[i][0];
            frame[i][2]=   iaxes[i][2];  }
     break;
     case 4: // x'= -y     y'= x       z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]=   -iaxes[i][1];
            frame[i][1]=    iaxes[i][0];
            frame[i][2]=    iaxes[i][2];  }

     break;

     case 5: // x'= z      y'= y       z'= -x
        for(i=0; i<3;i++)
        {   frame[i][0]=    iaxes[i][2];
            frame[i][1]=    iaxes[i][1];
            frame[i][2]=   -iaxes[i][0];  }

     break;
     case 6: // x'= -z     y'= y       z'= x
        for(i=0; i<3;i++)
        {   frame[i][0]=   -iaxes[i][2];
            frame[i][1]=    iaxes[i][1];
            frame[i][2]=    iaxes[i][0];  }

     break;
     default:
       printf("%s: %d: grasp_frame_from_inertia_axes: one of the input parameters is not valid.\n",__FILE__,__LINE__);
       return 0;
     break;
   }

   switch(axis)
   {
      case 1:
         rotAxis[0]=      1;    rotAxis[1]=         0;    rotAxis[2]=           0;
      break;
      case 2:
         rotAxis[0]=      0;    rotAxis[1]=         1;    rotAxis[2]=           0;
      break;
      case 3:
         rotAxis[0]=      0;    rotAxis[1]=         0;    rotAxis[2]=           1;
      break;
      default:
        printf("%s: %d: grasp_frame_from_inertia_axes: one of the input parameters is not valid.\n",__FILE__,__LINE__);
        return 0;
      break;
   }


   //matrice de transformation (rotation autour de l'axe (orienté) choisi et translation selon cet axe (orienté))
   p3d_mat4Rot(Mtransf, rotAxis, angle); //construit la matrice de transformation avec une rotation selon l'axe choisi
   Mtransf[0][3]= displacementX;
   Mtransf[1][3]= 0;
   Mtransf[2][3]= 0;

   p3d_matMultXform ( frame, Mtransf, gframe );


   //une rotation pour faire coïncider l'axe d'inertie avec l'axe z du repère de prise pour assurer la cohérence
   //avec la façon dont on calcule une prise à partir d'un repère de prise.
   p3d_matrix4 tmp;
   p3d_mat4Copy(gframe, tmp);

   for(i=0; i<3;i++)
   {    gframe[i][0]=    tmp[i][1];  // Y --> X
        gframe[i][1]=    tmp[i][2];  // Z --> Y
        gframe[i][2]=    tmp[i][0];  // X --> Z
   }

  return 1;
}


//! \deprecated
int gpGrasp_frame_from_inertia_axes(p3d_matrix3 iaxes, p3d_vector3 cmass, int direction, double displacement, int axis, double angle, p3d_matrix4 gframe)
{
   int i;
   p3d_matrix4 frame, Mtransf;
   p3d_vector3 rotAxis;

   //repère initial:
   frame[0][3]= cmass[0];
   frame[1][3]= cmass[1];
   frame[2][3]= cmass[2];
   frame[3][0]=        0;  frame[3][1]=  0; frame[3][2]=   0; frame[3][3]=   1;


   switch(direction)
   {
     case 1:  // x'= x      y'= y     z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]= iaxes[i][0];
            frame[i][1]= iaxes[i][1];
            frame[i][2]= iaxes[i][2];    }
     break;
     case 2:  // x'= -x     y'= -y      z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]=  -iaxes[i][0];
            frame[i][1]=  -iaxes[i][1];
            frame[i][2]=   iaxes[i][2];  }
     break;

     case 3: // x'= y       y'= -x     z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]=   iaxes[i][1];
            frame[i][1]=  -iaxes[i][0];
            frame[i][2]=   iaxes[i][2];  }
     break;
     case 4: // x'= -y     y'= x       z'= z
        for(i=0; i<3;i++)
        {   frame[i][0]=   -iaxes[i][1];
            frame[i][1]=    iaxes[i][0];
            frame[i][2]=    iaxes[i][2];  }

     break;

     case 5: // x'= z      y'= y       z'= -x
        for(i=0; i<3;i++)
        {   frame[i][0]=    iaxes[i][2];
            frame[i][1]=    iaxes[i][1];
            frame[i][2]=   -iaxes[i][0];  }

     break;
     case 6: // x'= -z     y'= y       z'= x
        for(i=0; i<3;i++)
        {   frame[i][0]=   -iaxes[i][2];
            frame[i][1]=    iaxes[i][1];
            frame[i][2]=    iaxes[i][0];  }

     break;
     default:
       printf("%s: %d: grasp_frame_from_inertia_axes: one of the input parameters is not valid.\n",__FILE__,__LINE__);
       return 0;
     break;
   }

   switch(axis)
   {
      case 1:
         rotAxis[0]=      1;    rotAxis[1]=         0;    rotAxis[2]=           0;
      break;
      case 2:
         rotAxis[0]=      0;    rotAxis[1]=         1;    rotAxis[2]=           0;
      break;
      case 3:
         rotAxis[0]=      0;    rotAxis[1]=         0;    rotAxis[2]=           1;
      break;
      default:
        printf("%s: %d: grasp_frame_from_inertia_axes: one of the input parameters is not valid.\n",__FILE__,__LINE__);
        return 0;
      break;
   }


   //matrice de transformation (rotation autour de l'axe (orienté) choisi et translation selon cet axe (orienté))
   p3d_mat4Rot(Mtransf, rotAxis, angle); //construit la matrice de transformation avec une rotation selon l'axe choisi
   Mtransf[0][3]= displacement;
   Mtransf[1][3]= 0;
   Mtransf[2][3]= 0;

   p3d_matMultXform ( frame, Mtransf, gframe );


   //une rotation pour faire coïncider l'axe d'inertie avec l'axe z du repère de prise pour assurer la cohérence
   //avec la façon dont on calcule une prise à partir d'un repère de prise.
   p3d_matrix4 tmp;
   p3d_mat4Copy(gframe, tmp);

   for(i=0; i<3;i++)
   {    gframe[i][0]=    tmp[i][1];  // Y --> X
        gframe[i][1]=    tmp[i][2];  // Z --> Y
        gframe[i][2]=    tmp[i][0];  // X --> Z
   }

  return 1;
}


//! Fonction d'echantillonnage des reperes de prise.
//! Elle reçoit en entree le centre de masse de l'objet, ses axes principaux d'inertie de l'objet et ses dimensions maximales
//! dans les directions de ces axes par rapport au centre de masse, ce qui donne un volume parallelepipedique.
//! Celui-ci est discrétisé avec un pas en translation donne en argument. Pour chaque position, nbDirections differentes
//! sont calculees pour l'axe z du repere de prise puis on fait effectuer des rotations du repere selon z avec un pas en rotation
//! de rotationStep (en radians).
//! La fonction retourne un tableau de repères (matrices 4x4) de dimension nbSamples.
//! Computes a set of grasp frames.
//! \param cmass object center of mass
//! \param iaxes object principal inertia axes (stored in column in a 3x3 matrix)
//! \param iaabb the extremal coordinates of the object mesh along the inertia axes
//! (xmin, xmax, ymin, ymax, zmin, zmax) -> (iaxes+iaab) ->inertia axes aligned bounding box
//! \param translationStep translation discretization step for hand/object pose sampling
//! \param nbDirections number of sampled directions for hand/object pose sampling
//! \param rotationStep rotation discretization step around each sampled direction for hand/object pose sampling
//! \param nbSamples will be filled with the number of generated grasp frames
//! \return a pointer to an array of grasp frames (4x4 matrices) in case of success, NULL otherwise
p3d_matrix4 *gpSample_grasp_frames(p3d_vector3 cmass, p3d_matrix3 iaxes, double iaabb[6], double translationStep, unsigned int nbDirections, double rotationStep, unsigned int *nbSamples)
{
  #ifdef DEBUG
   if(iaabb==NULL || nbSamples==NULL)
   { printf("%s: %d: gpSample_grasp_frames(): one input or more is NULL.\n",__FILE__,__LINE__);
     return NULL; }
  #endif

  unsigned int i, j, k, id, it, count= 0;
  unsigned int nbStepsX, nbStepsY, nbStepsZ, nbPositions;
  unsigned int nbStepsTheta, nbOrientations;
  double dX, dY, dZ, dTheta;
  double lengthX, lengthY, lengthZ;
  p3d_vector3 v, w;
  p3d_vector3 *directions= NULL;
  p3d_vector3 *positions= NULL;
  p3d_matrix3 *orientations= NULL;
  p3d_matrix3 R1, R2;
  p3d_matrix4 *result= NULL;


  lengthX= fabs(iaabb[0]) + fabs(iaabb[1]);
  lengthY= fabs(iaabb[2]) + fabs(iaabb[3]);
  lengthZ= fabs(iaabb[4]) + fabs(iaabb[5]);

  nbStepsX= (int) (lengthX/translationStep);
  if(nbStepsX==0) nbStepsX= 1;
  dX= lengthX/(nbStepsX+1);

  nbStepsY= (int) (lengthY/translationStep);
  if(nbStepsY==0) nbStepsY= 1;
  dY= lengthY/(nbStepsY+1);

  nbStepsZ= (int) (lengthZ/translationStep);
  if(nbStepsZ==0) nbStepsZ= 1;
  dZ= lengthZ/(nbStepsZ+1);

  nbStepsTheta= (int) (2*M_PI/rotationStep);
  if(nbStepsTheta==0) nbStepsTheta= 1;
  dTheta= 2*M_PI/(nbStepsTheta+1);

  nbPositions= nbStepsX*nbStepsY*nbStepsZ;
  positions= (p3d_vector3 *) malloc(nbPositions*sizeof(p3d_vector3));

  nbOrientations= nbDirections*nbStepsTheta;
  orientations= (p3d_matrix3 *) malloc(nbOrientations*sizeof(p3d_matrix3));

  *nbSamples= nbPositions*nbOrientations;
  result= (p3d_matrix4 *) malloc((*nbSamples)*sizeof(p3d_matrix4));

  count= 0;
  for(i=1; i<=nbStepsX; i++)
  {
    for(j=1; j<=nbStepsY; j++)
    {
      for(k=1; k<=nbStepsZ; k++)
      {
        positions[count][0]= cmass[0] + (iaabb[0] + i*dX)*iaxes[0][0] + (iaabb[2] + j*dY)*iaxes[0][1] + (iaabb[4] + k*dZ)*iaxes[0][2];

        positions[count][1]= cmass[1] + (iaabb[0] + i*dX)*iaxes[1][0] + (iaabb[2] + j*dY)*iaxes[1][1] + (iaabb[4] + k*dZ)*iaxes[1][2];

        positions[count][2]= cmass[2] + (iaabb[0] + i*dX)*iaxes[2][0] + (iaabb[2] + j*dY)*iaxes[2][1] + (iaabb[4] + k*dZ)*iaxes[2][2];

        count++;
      }
    }
  }

  directions= gpSample_sphere_surface(nbDirections, 1.0);
  count= 0;
  for(id=0; id<nbDirections; id++)
  {
    gpOrthonormal_basis(directions[id], v, w);

    R1[0][0]= v[0];
    R1[1][0]= v[1];
    R1[2][0]= v[2];

    R1[0][1]= w[0];
    R1[1][1]= w[1];
    R1[2][1]= w[2];

    R1[0][2]= directions[id][0];
    R1[1][2]= directions[id][1];
    R1[2][2]= directions[id][2];

    for(it=0; it<nbStepsTheta; it++)
    {
      R2[0][0]= cos(it*rotationStep); R2[0][1]=-sin(it*rotationStep); R2[0][2]= 0;
      R2[1][0]= sin(it*rotationStep); R2[1][1]= cos(it*rotationStep); R2[1][2]= 0;
      R2[2][0]=                    0; R2[2][1]=                    0; R2[2][2]= 1;

      p3d_mat3Mult(R1, R2, orientations[count]);
      count++;
    }
  }

  count= 0;
  for(i=0; i<nbPositions; i++)
  {
    for(j=0; j<nbOrientations; j++)
    {
      result[count][0][0]= orientations[j][0][0];
      result[count][1][0]= orientations[j][1][0];
      result[count][2][0]= orientations[j][2][0];
      result[count][3][0]= 0;

      result[count][0][1]= orientations[j][0][1];
      result[count][1][1]= orientations[j][1][1];
      result[count][2][1]= orientations[j][2][1];
      result[count][3][1]= 0;

      result[count][0][2]= orientations[j][0][2];
      result[count][1][2]= orientations[j][1][2];
      result[count][2][2]= orientations[j][2][2];
      result[count][3][2]= 0;

      result[count][0][3]= positions[i][0];
      result[count][1][3]= positions[i][1];
      result[count][2][3]= positions[i][2];
      result[count][3][3]= 1;
      count++;
    }
  }


  free(directions);
  free(positions);
  free(orientations);

  return result;
}

//! Generates a list of grasp for the given robot hand and object.
//! \param robot the hand robot (a freeflying robot composed of the hand/gripper bodies only)
//! \param object the object to be grasped
//! \param part the object part to grasp (all the object mesh triangles that have the same value in their "part" field). Set to 0 if unused (all the triangles will be considered).
//! \param cmass the object center of mass (written in the object frame)
//! \param iaxes the object principal inertia axes (written in the object frame)
//! \param iaabb the extremal coordinates of the object mesh along the inertia axes
//! (xmin, xmax, ymin, ymax, zmin, zmax) -> (iaxes+iaab) ->inertia axes aligned bounding box
//! \param hand structure containing information about the hand geometry
//! \param translationStep translation discretization step for hand/object pose sampling
//! \param nbDirections number of sampled directions for hand/object pose sampling
//! \param rotationStep rotation discretization step around each sampled direction for hand/object pose sampling
//! \param graspList the computed grasp list
//! \return 1 in case of success, 0 otherwise
int gpGrasp_generation(p3d_rob *robot, p3d_obj *object, int part, p3d_vector3 cmass, p3d_matrix3 iaxes, double iaabb[6], gpHand_properties &hand, double translationStep, unsigned int nbDirections, double rotationStep, std::list<class gpGrasp> &graspList)
{
  #ifdef DEBUG
   if(robot==NULL || object==NULL)
   {
     printf("%s: %d: gpGrasp_generation(): one input or more is NULL (%p %p).\n",__FILE__,__LINE__, robot, object);
     return 0;
   }
  #endif

  unsigned int i;
  unsigned int nbGraspFrames= 0;
  unsigned int nbGraspFramesMax= hand.max_nb_grasp_frames; //to avoid excessive computations if the input parameters were not properly chosen
  p3d_matrix4 *gframes= NULL;
  std::list<gpGrasp>::iterator igrasp;

  gframes= gpSample_grasp_frames(cmass, iaxes, iaabb, translationStep, nbDirections, rotationStep,  &nbGraspFrames);

  if(nbGraspFrames > nbGraspFramesMax)
  {
    printf("%s: %d: gpGrasp_generation(): number of sampled grasp frames exceeds %d. It will be reduced.\n",__FILE__,__LINE__,nbGraspFramesMax);
  }

  while(nbGraspFrames > nbGraspFramesMax)
  {
    free(gframes);
    gframes= NULL;
    translationStep= 1.1*translationStep;
    rotationStep   = 1.1*rotationStep;
    gframes= gpSample_grasp_frames(cmass, iaxes, iaabb, translationStep, nbDirections, rotationStep,  &nbGraspFrames);
  }

  printf("%d grasp frames will be used.\n", nbGraspFrames);

  for(i=0; i<nbGraspFrames; i++)
  {
    gpGrasps_from_grasp_frame(robot, object, part, gframes[i], hand, graspList);
  }

  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    if(igrasp->object==NULL)
    {
      igrasp->object= object;
      igrasp->object_name= object->name;
    }
  }

  free(gframes);
  return 1;
}


//!  Context independent collision test: removes from a grasp list all the grasps causing a collision between the robot hand and the grasped object.
//! \param graspList the original grasp list
//! \param robot the hand robot (a freeflying robot only composed of the hand/gripper bodies)
//! \param object the grasped object
//! \param hand structure containing information about the hand geometry
//! \return 1 in case of success, 0 otherwise
int gpGrasp_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGrasp_collision_filter(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpGrasp_collision_filter(): object is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_collision_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif

  p3d_matrix4 objectFrame;
  configPt q= p3d_alloc_config(robot);
  std::list<gpGrasp>::iterator igrasp;

  p3d_get_obj_pos(object, objectFrame);

  p3d_get_robot_config_into(robot, &q);

  gpDeactivate_object_fingertips_collisions(robot, object, hand);

  igrasp= graspList.begin();
  while(igrasp!=graspList.end())
  {
     gpInverse_geometric_model_freeflying_hand(robot, objectFrame, igrasp->frame, hand, q);

     p3d_set_and_update_this_robot_conf(robot, q);

     gpSet_grasp_configuration(robot, hand, *igrasp);

     if(!p3d_col_test_robot_obj(robot, object))//pas de collision
     {
        igrasp++;
        continue;
     }
      else
      {
         igrasp= graspList.erase(igrasp);
         continue;
      }

     switch(hand.type)
     {
        case GP_GRIPPER:
        //On ouvre la pince légèrement  plus (10%) que dans la position de prise:
         igrasp->config[0]*= 1.1;
         if(igrasp->config[0] > hand.max_opening_jnt_value)
         {  igrasp->config[0]= hand.max_opening_jnt_value;  }
         gpSet_grasp_configuration(robot, hand, *igrasp);
        break;
        case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
         //erase_current_LList(graspList);
        continue;
        break;
        default:
          printf("%s: %d: gpGrasp_collision_filter(): undefined or unimplemented hand type.\n", __FILE__, __LINE__);
          p3d_destroy_config(robot, q);
          return 0;
        break;
      }

      p3d_set_and_update_this_robot_conf(robot, q);

      if( !p3d_col_test_robot_obj(robot, object) )//pas de collision
      {
        igrasp++;
        continue;
      }
      else
      {
         igrasp= graspList.erase(igrasp);
      }
  }


  p3d_destroy_config(robot, q);


  return 1;
}

//!  Context dependent collision test: removes from a grasp list all the grasps causing a collision between the robot hand and the environment.
//! \param graspList the original grasp list
//! \param robot the hand robot (a freeflying robot only composed of the hand/gripper bodies)
//! \param object the grasped object
//! \param hand structure containing information about the hand geometry
//! \return 1 in case of success, 0 otherwise
int gpGrasp_context_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGrasp_context_collision_filter(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_context_collision_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif

  p3d_matrix4 objectFrame;
  configPt q= p3d_alloc_config(robot);
  std::list<gpGrasp>::iterator igrasp;

  p3d_get_obj_pos(object, objectFrame);

  p3d_get_robot_config_into(robot, &q);

  igrasp= graspList.begin();
  while(igrasp!=graspList.end())
  {
     gpInverse_geometric_model_freeflying_hand(robot, objectFrame, igrasp->frame, hand, q);

     p3d_set_and_update_this_robot_conf(robot, q);

     gpSet_grasp_configuration(robot, hand, *igrasp);

     if( !p3d_col_test_robot_statics(robot, 1) )
     {
        igrasp++;
        continue;
     }
     else
     {
        igrasp= graspList.erase(igrasp);
        continue;
     }
  }


  p3d_destroy_config(robot, q);


  return 1;
}


//! Eliminates all the unstable grasps from a list and sorts the remaining list from the grasp with the biggest
//! stability score to the one with the smallest score.
//! \param graspList a list of grasps
//! \return 1 in case of success, 0 otherwise
int gpGrasp_stability_filter(std::list<gpGrasp> &graspList)
{
  #ifdef DEBUG
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_stability_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif

  double quality;
  std::list<gpGrasp>::iterator igrasp;

  igrasp= graspList.begin();
  while(igrasp!=graspList.end())
  {
    quality= igrasp->computeQuality();
    if(quality==0)
    {
       igrasp= graspList.erase(igrasp);
       continue;
    }
    igrasp++;
  }

  graspList.sort(); //sort from the smallest to the biggest quality
  graspList.reverse(); //reverse the order of the elements in the list

  return 1;
}

//! Computes the hand (wrist) pose corresponding to a given grasp frame.
//! \param robot pointer to the hand robot (its first joint must be a P3D_FREEFLYER)
//! \param objectFrame frame representing the object pose (in world frame)
//! \param graspFrame grasp frame (in object frame)
//! \param hand structure containing information about the hand geometry
//! \param q the computed hand configuration (must have been allocated before calling the function). Only the part corresponding to the hand pose is modified. The finger configurations are not modified.
int gpInverse_geometric_model_freeflying_hand(p3d_rob *robot, p3d_matrix4 objectFrame, p3d_matrix4 graspFrame, gpHand_properties &hand, configPt q)
{
   #ifdef DEBUG
    if(robot==NULL)
    {
      printf("%s: %d: gpInverse_geometric_model_freeflying_hand(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
    if(q==NULL)
    {
      printf("%s: %d: gpInverse_geometric_model_freeflying_hand(): q is NULL.\n",__FILE__,__LINE__);
      return 0;
    }
   #endif

   p3d_matrix4 graspFrame_world, Twrist;

   p3d_get_robot_config_into(robot, &q);

   p3d_mat4Mult(objectFrame, graspFrame, graspFrame_world ); //passage repère objet -> repère monde

   p3d_mat4Mult(graspFrame_world, hand.Tgrasp_frame_hand, Twrist);

   p3d_mat4ExtractPosReverseOrder2(Twrist, &q[6], &q[7], &q[8], &q[9], &q[10], &q[11]);

   return 1;
}


//! Computes the forward kinematics model of the PA-10 arm for the robot's current configuration.
//! \param robot the robot (that must have joints with specific names (see graspPlanning.h))
//! \param Tend_eff the computed end effector pose matrix (in the world frame)
//! \param display if true, the frame of each body will be displayed
//! \return 1 in case of success, 0 otherwise
extern int gpForward_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, bool display)
{
  float mat[16];
  p3d_matrix4 armBaseFrame, TH01, TH02, TH03, TH04, TH05, Tend_effb;;
  p3d_jnt *armJoint= NULL;
  Gb_q6 q;
  Gb_6rParameters arm_parameters;
  Gb_th thMGD, th01, th02, th03, th04, th05, R6RT, thMatPA10;
  Gb_dataMGD d;
  Gb_dep dep1;

  gpGet_arm_base_frame(robot, armBaseFrame);

  armJoint= p3d_get_robot_jnt_by_name(robot,  GP_ARMBASEJOINT);
  if(armJoint==NULL)
  {  return 0; }
  q.q1= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  GP_ARMJOINT2);
  if(armJoint==NULL)
  {  return 0; }
  q.q2= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  GP_ARMJOINT3);
  if(armJoint==NULL)
  {  return 0; }
  q.q3= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  GP_ARMJOINT4);
  if(armJoint==NULL)
  {  return 0; }
  q.q4= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  GP_ARMJOINT5);
  if(armJoint==NULL)
  {  return 0; }
  q.q5= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  GP_WRISTJOINT);
  if(armJoint==NULL)
  {  return 0; }
  q.q6= robot->ROBOT_POS[armJoint->index_dof];


  arm_parameters.a2 = PA10_ARM_A2;
  arm_parameters.r4 = PA10_ARM_R4;
  arm_parameters.epsilon = PA10_ARM_EPSILON;
  arm_parameters.of1 = PA10_ARM_OF1;
  arm_parameters.of2 = PA10_ARM_OF2;
  arm_parameters.of3 = PA10_ARM_OF3;
  arm_parameters.of4 = PA10_ARM_OF4;
  arm_parameters.of5 = PA10_ARM_OF5;
  arm_parameters.of6 = PA10_ARM_OF6;

  Gb_MGD6r_6Th(&arm_parameters, &q, &d, &th01, &th02, &th03, &th04, &th05, &thMGD);

  Gb_th_matrix4(&th01, TH01);
  Gb_th_matrix4(&th02, TH02);
  Gb_th_matrix4(&th03, TH03);
  Gb_th_matrix4(&th04, TH04);
  Gb_th_matrix4(&th05, TH05);
  Gb_th_matrix4(&thMGD, Tend_eff);

  Gb_dep_set(&dep1, 0.0, 0.0, (PA10_6ARM_LENGTH + PA10_TOOL_LENGTH), 0.0, 1.0, 0.0, -(M_PI/2.0));

  Gb_dep_th(&dep1, &R6RT);
  Gb_th_produit(&thMGD, &R6RT, &thMatPA10);
  Gb_th_matrix4(&thMatPA10, Tend_effb);

  p3d_matrix4_to_OpenGL_format(armBaseFrame, mat);
  p3d_matMultXform(armBaseFrame, Tend_effb, Tend_eff);

  if(display)
 {
   glPushMatrix();
   glMultMatrixf(mat);
    draw_frame(TH01, 0.2);
    draw_frame(TH02, 0.2);
    draw_frame(TH03, 0.2);
    draw_frame(TH04, 0.2);
    draw_frame(TH05, 0.2);
    draw_frame(Tend_effb, 0.3);
   glPopMatrix();
 }

 return 1;
}


//! Computes the inverse kinematics of the PA-10 arm.
//! \param robot the robot (that must have joints with specific names (see graspPlanning.h))
//! \param Tend_eff the desired end effector pose matrix (given in the arm base frame)
//! \param q the solution joint parameter vector (that must be allocated before calling the function)
//! \return 1 in case of success, 0 otherwise
int gpInverse_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt q)
{
  int result;
  Gb_6rParameters arm_parameters;
  Gb_th eth;
  Gb_q6 qcurrent, qgoal;
  Gb_dataMGD d;
  Gb_th thdep1, thdep2, R6RT, invR6RT, thMatPA10;
  Gb_dep dep1, dep2;

  arm_parameters.a2 = PA10_ARM_A2;
  arm_parameters.r4 = PA10_ARM_R4;
  arm_parameters.epsilon = PA10_ARM_EPSILON;
  arm_parameters.of1 = PA10_ARM_OF1;
  arm_parameters.of2 = PA10_ARM_OF2;
  arm_parameters.of3 = PA10_ARM_OF3;
  arm_parameters.of4 = PA10_ARM_OF4;
  arm_parameters.of5 = PA10_ARM_OF5;
  arm_parameters.of6 = PA10_ARM_OF6;

  Gb_matrix4_th(Tend_eff, &eth);

  //PA10_TOOL_LENGTH+0.041= distance à ajouter pour que le repère terminal soit à l'extrémité
  //du dernier corps du bras (celui sur lequel est montée la pince)
  //On rajoute 0.0685 pour que le repère soit au niveau des "doigts" (hémisphères) de la pince.
  //Il faut aussi se décaler de O.OOO5 selon x pour que le plan oxy du repère soit bien dans le plan
  //des trois doigts.
  Gb_dep_set(&dep1, 0, 0, PA10_TOOL_LENGTH + PA10_6ARM_LENGTH, 0.0, 1.0, 0.0, -M_PI_2);
//   Gb_dep_set(&dep2, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -(M_PI/8.0));
  Gb_dep_set(&dep2, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  Gb_dep_th(&dep1, &thdep1);
  Gb_dep_th(&dep2, &thdep2);
  Gb_th_produit(&thdep1, &thdep2, &R6RT);
  Gb_th_inverse(&R6RT, &invR6RT);

  Gb_th_produit(&eth, &invR6RT, &thMatPA10);

  p3d_get_robot_config_into(robot, &q);

  qcurrent.q1 = DEGTORAD * PA10_Q1_INIT;
  qcurrent.q2 = DEGTORAD * PA10_Q2_INIT;
  qcurrent.q3 = DEGTORAD * PA10_Q3_INIT;
  qcurrent.q4 = DEGTORAD * PA10_Q4_INIT;
  qcurrent.q5 = DEGTORAD * PA10_Q5_INIT;
  qcurrent.q6 = DEGTORAD * PA10_Q6_INIT;

  result= Gb_MGI6rTh_O(&arm_parameters, &thMatPA10, &qcurrent, &d, &qgoal);

  switch(result)
  {
    case MGI_OK:
        //printf("MGI_OK\n");
    break;
    case MGI_ERROR:
        //printf("MGI_ERROR\n");
        return 0;
    break;
    case MGI_APPROXIMATE:
         //printf("MGI_APPROXIMATE\n");
    break;
    case MGI_SINGULAR:
         //printf("MGI_SINGULAR\n");
    break;
  }

  configPt q0= NULL;

  q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0);

  // tests if the joint parameters are within the bounds with gpSet_arm_configuration() function:
  if(gpSet_arm_configuration(robot, GP_PA10, qgoal.q1, qgoal.q2, qgoal.q3, qgoal.q4, qgoal.q5, qgoal.q6)==1)
  {
    p3d_get_robot_config_into(robot, &q);
    p3d_set_and_update_this_robot_conf(robot, q0);
    p3d_destroy_config(robot, q0);
    return 1;
  }
  else
  {
    p3d_set_and_update_this_robot_conf(robot, q0);
    p3d_destroy_config(robot, q0);
    return 0;
  }
}


//! Finds, for a given mobile base configuration of the robot, a grasp from the given grasp list, that is
//! reachable by the arm and hand.
//! \param robot the robot
//! \param object the object to grasp
//! \param graspList a list of grasps
//! \param arm_type the robot arm type
//! \param qbase a configuration of the robot (only the part corresponding to the mobile base will be used)
//! \param grasp a copy of the grasp that has been found, in case of success
//! \param hand structure containing information about the hand geometry
//! \return a pointer to the computed grasping configuration of the whole robot in case of success, NULL otherwise
configPt gpFind_grasp_from_base_configuration(p3d_rob *robot, p3d_obj *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpFind_robot_config_from_grasp(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpFind_robot_config_from_grasp(): object is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif

  std::list<gpGrasp>::iterator igrasp;

  p3d_matrix4 object_frame, base_frame, inv_base_frame, gframe_object, gframe_world, gframe_robot, gframe_robot2;
  configPt q0= NULL; //pour mémoriser la configuration courante du robot
  configPt result= NULL;


  q0= p3d_get_robot_config(robot);

  //On met à jour la configuration du robot pour que sa base soit dans la configuration
  //souhaitée:
  p3d_set_and_update_this_robot_conf(robot, qbase);
  result= p3d_alloc_config(robot);

  gpGet_arm_base_frame(robot, base_frame); //on récupère le repère de la base du bras
  p3d_matInvertXform(base_frame, inv_base_frame);


  //pour chaque prise de la liste:
  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    p3d_mat4Copy(igrasp->frame, gframe_object);
    p3d_get_obj_pos(object, object_frame);
    p3d_mat4Mult(object_frame, gframe_object, gframe_world ); //passage repère objet -> repère monde

    p3d_mat4Mult(inv_base_frame, gframe_world, gframe_robot); //passage repère monde -> repère robot

    gpDeactivate_object_fingertips_collisions(robot, object, hand);
    switch(arm_type)
    {
      case GP_PA10:
        p3d_mat4Mult(gframe_robot, hand.Tgrasp_frame_hand, gframe_robot2);
        p3d_mat4Mult(gframe_robot2, hand.Thand_wrist, gframe_robot);

        p3d_copy_config_into(robot, qbase, &result);

        if( gpInverse_geometric_model_PA10(robot, gframe_robot, result)==1 )
        {
					p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, result);
           p3d_set_and_update_this_robot_conf(robot, result);
           gpSet_grasp_configuration(robot, hand, *igrasp);

           if(!p3d_col_test()) //if no collision
          // if(!p3d_col_test_robot_statics(robot, 0) && !p3d_col_test_self_collision(robot, 0)) //if no collision
           {
              p3d_get_robot_config_into(robot, &result);
              igrasp->collision_state= COLLISION_FREE;
              grasp= *igrasp;

              p3d_set_and_update_this_robot_conf(robot, q0);
              p3d_destroy_config(robot, q0);

              return result;
           }
        }
      break;
      default:
          printf("%s: %d: gpFind_grasp_from_base_configuration(): undefined or unimplemented arm type.\n",__FILE__,__LINE__);
          p3d_set_and_update_this_robot_conf(robot, q0);
          p3d_destroy_config(robot, q0);
          return NULL;
      break;
    }

  }

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);

  return NULL;
}


