
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include <math.h>
#include <string>
#include <sstream>

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


//! Gets the arm base frame of a robot as a 4x4 matrix.
//! The frame is the one of the (fixed) joint that links the arm base body to the mobile base main body.
//! The function finds the joint by its name, that must be the one defined by GP_ARMBASEJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param frame the ouput matrix
//! \return 1 in case of success, 0 otherwise
int gpGet_arm_base_frame(p3d_rob *robot, p3d_matrix4 frame)
{
  #ifdef DEBUG
  if(robot==NULL)
  { 
    printf("%s: %d: gpGet_arm_base_frame(): robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
  #endif

  int i, j;
  p3d_jnt *jnt= NULL;

  jnt= get_robot_jnt_by_name(robot, GP_ARMBASEJOINT);

  if(jnt==NULL)
  {
    printf("%s: %d: gpGet_arm_base_frame(): robot \"%s\" should have a joint named \"%s\".\n",__FILE__,__LINE__,robot->name, GP_ARMBASEJOINT);
    return 0;
  }

  for(i=0; i<4; i++)
  {
    for(j=0; j<4; j++)
    {
      frame[i][j]= jnt->abs_pos[i][j];
    }
  }

  return 1;
}


//! Gets the platform frame of a robot as a 4x4 matrix.
//! This frame is chosen to be the same as the frame
//! of the joint linking the mobile platform to the environment.
//! This joint is found by its name defined by GP_PLATFORMJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param frame the ouput matrix
//! \return 1 in case of success, 0 otherwise
int gpGet_platform_frame(p3d_rob *robot, p3d_matrix4 frame)
{
  #ifdef DEBUG
  if(robot==NULL)
  { 
    printf("%s: %d: gpGet_platform_frame(): robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
  #endif

  int i, j;
  p3d_jnt *jnt= NULL;

  jnt= get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  if(jnt==NULL)
  {
    printf("%s: %d: gpGet_platform_frame(): robot \"%s\" should have a joint named \"%s\".\n",__FILE__,__LINE__,robot->name, GP_PLATFORMJOINT);
    return 0;
  }

  for(i=0; i<4; i++)
  {
    for(j=0; j<4; j++)
    {
      frame[i][j]= jnt->abs_pos[i][j];
    }
  }

  return 1;
}


//! Gets the wrist frame of a robot as a 4x4 matrix.
//! This frame is chosen to be the same as the frame
//! of the robot's wrist joint.
//! This joint is found by its name defined by GP_WRISTJOINT (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param frame the ouput matrix
//! \return 1 in case of success, 0 otherwise
int gpGet_wrist_frame(p3d_rob *robot, p3d_matrix4 frame)
{
  #ifdef DEBUG
  if(robot==NULL)
  { 
    printf("%s: %d: gpGet_wrist_frame(): robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
  #endif

  int i, j;
  p3d_jnt *jnt= NULL;

  jnt= get_robot_jnt_by_name(robot, GP_WRISTJOINT);

  if(jnt==NULL)
  {
    printf("%s: %d: gpGet_wrist_frame(): robot \"%s\" should have a joint named \"%s\".\n",__FILE__,__LINE__,robot->name, GP_WRISTJOINT);
    return 0;
  }

  for(i=0; i<4; i++)
  {
    for(j=0; j<4; j++)
    {
      frame[i][j]= jnt->abs_pos[i][j];
    }
  }

  return 1;
}



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



//! Finds a collision-free configuration for the mobile base of a robot in a ring centered on a specified position.
//! The collisions are avoided for the base only. Some of the robot's joints or bodies must have specific names
//! (see graspPlanning.h).
//! \param robot pointer to the robot
//! \param innerRadius inner radius of the ring
//! \param outerRadius outer radius of the ring
//! \param objLoc desired center of the ring (world coordinates)
//! \return a pointer to the computed robot configuration
configPt gpRandom_robot_base(p3d_rob *robot, double innerRadius, double outerRadius, p3d_vector3 objLoc)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpRandom_robot_base(): robot is NULL.\n",__FILE__,__LINE__);
   }
  #endif

  int i, nb_iter= 0, nb_iter_max= 100; 
  int solution_found= 0;

  double theta, radius;
 
  configPt q= p3d_alloc_config(robot);
  configPt q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0); //pour mémoriser la configuration courante

  int k, jnt_index;
  p3d_jnt *jntPt= NULL;
  jnt_index= get_robot_jnt_index_by_name(robot, GP_PLATFORMJOINT);

  jntPt = robot->joints[jnt_index];
  jntPt= get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  //On veut juste tester que la configuration de la base mobile est sans collision
  //car la configuration du bras dépendra de la prise souhaitée. On désactive 
  //donc les collisions entre les corps du bras et de la main et ceux de l'environnement 
  //ainsi que les collisions internes du robot:
  XYZ_ENV->cur_robot= robot;
  std::string arm_body_base_name, hand_body_base_name, body_name;
  std::stringstream out;

  arm_body_base_name= std::string(robot->name) + std::string(".") + GP_ARM_BODY_PREFIX + std::string(".");
  hand_body_base_name= std::string(robot->name) + std::string(".") + GP_HAND_BODY_PREFIX + std::string(".");

  //Deactivate
  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;

    if(arm_body_base_name.compare(0, arm_body_base_name.length(), body_name, 0, arm_body_base_name.length())==0)
    { 
       p3d_col_deactivate_obj_env(robot->o[i]);
       continue;
    }
    if(hand_body_base_name.compare(0, hand_body_base_name.length(), body_name, 0, hand_body_base_name.length())==0)
    { 
       p3d_col_deactivate_obj_env(robot->o[i]);
       continue;
    }
  }

  //On désactive les collisions internes:
  p3d_col_deactivate_rob(robot);

  while(nb_iter < nb_iter_max)
  {
    radius= p3d_random(innerRadius, outerRadius);
    theta= p3d_random(0.0, 2*M_PI); 

    k= jntPt->index_dof;
    q[k]= objLoc[X] + radius*cos(theta);
    k= jntPt->index_dof + 1;
    q[k]= objLoc[Y] + radius*sin(theta);
    q[jntPt->index_dof + 2]= q0[jntPt->index_dof + 2];

    k= jntPt->index_dof + 5;
    //La base du robot va être tournée vers l'objet avec un angle compris entre -Pi/2 et Pi/2 de façon
    //à ce que l'objet soit devant le robot mais que ce dernier puisse quand même être un peu tourné.
    //Ça peut être utile s'il y a des obstacles à éviter.
    q[k] = theta + M_PI + p3d_random(-M_PI/2.0, M_PI/2.0);
    p3d_set_and_update_this_robot_conf(robot, q);

    if( !p3d_col_test_robot_statics(robot, 0) )
    {
      solution_found= 1;
      break;
    }
    else  
      nb_iter++;
  }

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);


  // reactivate the previously decactivated collisions:
  for(i=0; i<robot->no; i++)
  {
    body_name= robot->o[i]->name;
    if(arm_body_base_name.compare(0, arm_body_base_name.length(), body_name, 0, arm_body_base_name.length())==0)
    { 
       p3d_col_activate_obj_env(robot->o[i]);
       continue;
    }
    if(hand_body_base_name.compare(0, hand_body_base_name.length(), body_name, 0, hand_body_base_name.length())==0)
    { 
       p3d_col_activate_obj_env(robot->o[i]);
       continue;
    }
  }
  p3d_col_activate_rob(robot);


  if(solution_found==0)
  {
    p3d_destroy_config(robot, q);
    q= NULL;
    printf("%s: %d: gpRandom_robot_base(): no valid configuration was found for the robot mobile base.\n",__FILE__,__LINE__);
  } 

  return q;
}



// WIP
// Calcule un ensemble de poses stables pour le polyèdre reçu en argument.
// Une pose est un plan sur lequel on pourra poser l'objet.
/*
int gpFind_stable_poses(p3d_polyhedre *polyhedron, int nb_directions)
{
   #ifdef DEBUG 
   if(polyhedron==NULL)
   {
     printf("%s: %d: gpFind_stable_poses(): input is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   #endif

   int i, j;
   double d1, d2, d3, dmin;
   tChull chull= NULL;
   chull= compute_convex_hull(polyhedron);
   tFace f;
   p3d_vector3 center_of_mass, p1, p2, p3, pn, intersection;
   p3d_plane triangle_plane, plane1, plane2, plane3;

   center_of_mass[0]= chull->center[0];
   center_of_mass[1]= chull->center[1];
   center_of_mass[2]= chull->center[2];

   p3d_vector3 *samples= sample_sphere_surface(nb_directions, 1.0);


   f= chull->faces;
   do
   {
     f->visible= BOOL_FALSE; 
     f = f->next;
   } while ( f != chull->faces );


   for(i=0; i<nb_directions; i++)
   {
     f= chull->faces;
     do
     {
       if(f->visible==BOOL_TRUE) //pour marquer les faces déjà testées
         continue;

       for(j=0; j<3; j++)
       {
          p1[j]= f->vertex[0]->v[j];
          p2[j]= f->vertex[1]->v[j];
          p3[j]= f->vertex[2]->v[j];
       }  

       if(ray_triangle_intersection(center_of_mass, samples[i], p1, p2, p3, intersection)!=0)
       {
         f->visible=BOOL_TRUE;
         triangle_plane= plane_from_points(p1, p2, p3);
        // orthogonal_projection_point_onto_plane(center_of_mass, triangle_plane, projection);

         p3d_vectAdd(p2, triangle_plane.normale, pn);
         plane1= plane_from_points(p1, p2, pn);
         d1= p3d_vectDotProd(center_of_mass, plane1.normale);
         if(d1-plane1.d < 0)
            continue;

         p3d_vectAdd(p3, triangle_plane.normale, pn);
         plane2= plane_from_points(p2, p3, pn);
         d2= p3d_vectDotProd(center_of_mass, plane2.normale);
         if(d2-plane2.d < 0)
            continue;

         p3d_vectAdd(p1, triangle_plane.normale, pn);
         plane3= plane_from_points(p3, p1, pn);  
         d3= p3d_vectDotProd(center_of_mass, plane3.normale);
         if(d3-plane3.d < 0)
            continue;       

         dmin= (d1<d2) ? d1 : d2;
         dmin= (d3<dmin) ? d3 : dmin;
       }


       f = f->next;
     } while ( f != chull->faces );

   }

   free(samples);

//  v= chull->vertices;
//  do
//  {
//    num_vertices++;
//    if(v->vnum > max_index)
//      max_index= v->vnum;
//    v = v->next;
//  } while ( v != chull->vertices );

//   f= chull->faces;
//   do
//   {
//     num_faces++;    
//     f = f->next;
//   } while ( f != chull->faces );


  return 1;
}*/



//! Gets the joint angles of the SAHand fingers in its current configuration.
//! \param robot the robot (that must have joint with the appropriate names (see graspPlanning.h))
//! \param hand structure containing information about the hand geometry
//! \param q array that will be filled with the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return 1 in case of succcess, 0 otherwise
int gpGet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand, double q[4], int finger_index)
{
  #ifdef DEBUG 
   if(robot==NULL)
   {
     printf("%s: %d: gpGet_SAHfinger_joint_angles(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpGet_SAHfinger_joint_angles(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return 0;
   }
  #endif

  p3d_jnt *joint= NULL;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:

     switch(finger_index)
     {
       case 1: //thumb
          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT1);
          q[0]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT2);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT3);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT4);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
       case 2: //forefinger
          q[0]= 0.0;

          joint=  get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
       case 3: //middle finger
          q[0]= 0.0;

          joint=  get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
       case 4: //ring finger
          q[0]= 0.0;

          joint=  get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1);
          q[1]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2);
          q[2]= p3d_jnt_get_dof(joint, 0);

          joint=  get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3);
          q[3]= p3d_jnt_get_dof(joint, 0);
       break;
     }
    break;
    default:
      printf("%s: %d: gpGet_SAHfinger_joint_angles(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
    break;
  }

  return 1;
}

//! Sets the joint angles of the SAHand fingers and update its current configuration.
//! \param robot the robot (that must have joint with the appropriate names (see graspPlanning.h))
//! \param hand structure containing information about the hand geometry
//! \param q array containing the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return 1 in case of succcess, 0 otherwise
int gpSet_SAHfinger_joint_angles(p3d_rob *robot, gpHand_properties &hand, double q[4], int finger_index)
{
  #ifdef DEBUG 
   if(robot==NULL)
   {
     printf("%s: %d: gpSet_SAHfinger_joint_angles(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpSet_SAHfinger_joint_angles(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return 0;
   }
  #endif

  p3d_jnt *joint= NULL;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:

     switch(finger_index)
     {
       case 1: //thumb
          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT1);
          p3d_jnt_set_dof(joint, 0, q[0]);

          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT2);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT3);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  get_robot_jnt_by_name(robot, GP_THUMBJOINT4);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
       case 2: //forefinger
          joint=  get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
       case 3: //middle finger
          joint=  get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
       case 4: //ring finger
          joint=  get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1);
          p3d_jnt_set_dof(joint, 0, q[1]);

          joint=  get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2);
          p3d_jnt_set_dof(joint, 0, q[2]);

          joint=  get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3);
          p3d_jnt_set_dof(joint, 0, q[3]);
       break;
     }
    break;
    default:
      printf("%s: %d: gpSet_SAHfinger_joint_angles(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
    break;
  }

  p3d_update_this_robot_pos(robot);

  return 1;
}


// Calcule le modèle géométrique direct d'un doigt de la main SAHand (de type (gauche/droite) et dimensions définis
// dans hand_properties). Le doigt est celui désigné par finger_index.
// La fonction reçoit en argument la pose de la main (repère Twrist du centre du poignet) et ses dimensions
// via une structure gpHand_properties.
// Le point retourné (p) est le centre (à l'intérieur) du volume du bout du doigt (phalange distale).
// La fonction retourne aussi la normale à la surface du bout du doigt en son centre.
// Cette normale permet de savoir dans quel demi-plan est la partie 'avant' du doigt (celle opposée
// à l'ongle).
// On a thumb= 1, forefinger= 2, middle finger= 3, ring finger= 4.
// Les paramètres articulaires sont dans q (q[0] est inutile pour les doigts autres que le pouce).
// Le point p est le point atteint par le centre du doigt.
// Il est donné dans le repère global (monde).
//! Computes the forward kinematics of the SAHand fingers.
//! \param Twrist hand pose (frame of the wrist center)
//! \param hand structure containing information about the hand geometry
//! \param q the finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param p the computed position of the fingertip center
//! \param fingerpad_normal a vector giving the direction of the fingertip contact surface (orthogonal to the medial axis of the distal phalanx and directed towards the inside of the hand). It is computed for the given finger configuration.
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return 1 in case of succcess, 0 otherwise
//! NB: the first joint of the thumb is not taken into account: it is supposed to be at its maximum value (90 degrees)
//! in opposition to the other fingers.
int gpSAHfinger_forward_kinematics(p3d_matrix4 Twrist, gpHand_properties &hand, double q[4], p3d_vector3 p, p3d_vector3 fingerpad_normal, int finger_index)
{
  #ifdef DEBUG 
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpSAHfinger_forward_kinematics(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return 0;
   }
  #endif

  double l0, l1, l2, l3;
  double x, y, z;
  p3d_vector3 p_finger, fingerpad_normal_relative;
  p3d_matrix4 Tfinger_world;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT:
     l0= hand.length_thumbBase;
     l1= hand.length_proxPha;
     l2= hand.length_midPha;
     l3= hand.length_distPha;

     switch(finger_index)
     {
       case 1: //thumb
          p3d_mat4Mult(Twrist, hand.Twrist_thumb, Tfinger_world);
       break;
       case 2: //forefinger
          p3d_mat4Mult(Twrist, hand.Twrist_forefinger, Tfinger_world);
       break;
       case 3: //middle finger
          p3d_mat4Mult(Twrist, hand.Twrist_middlefinger, Tfinger_world);
       break;
       case 4: //ring finger
          p3d_mat4Mult(Twrist, hand.Twrist_ringfinger, Tfinger_world);
       break;
     }

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


     //g3d_drawSphere( p[0], p[1], p[2],  0.005, Yellow, NULL);
     //glLineWidth(5);
     //g3d_drawOneLine( p[0], p[1], p[2], p[0]+0.03*normal[0], p[1]+0.03*normal[1], p[2]+0.03*normal[2], Red, NULL);
    break;
    default:
       printf("%s: %d: gpSAHfinger_forward_kinematics(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
    break;
  }

  return 1;
}



// Calcule le modèle géométrique inverse d'un doigt de la main SAHand (de type (gauche/droite) et dimensions définis
// dans hand_properties). Le doigt est celui désigné par finger_index.
// On a thumb= 1, forefinger= 2, middle finger= 3, ring finger= 4.
// Le point p_world, donné en argument, est le point que doit atteindre le centre du doigt.
// Il est donné dans le repère global (monde) et la position de la main est donnée
// via le repère de transformation du poignet Twrist.
// q sera rempli avec la solution. Pour les doigts autres que le pouce, q[0] est inutile (ils n'ont que trois
// liaisons indépendantes).
// La fonction retourne 1 si elle trouve une solution, 0 sinon.
//! Computes the inverse kinematics of the SAHand fingers.
//! \param Twrist hand pose (frame of the wrist center) in the "world" frame
//! \param hand structure containing information about the hand geometry
//! \param p the desired position of the fingertip in the "world" frame
//! \param q the computed finger joint parameters (angles in radians). Except for the thumb, only the three last elements are used.
//! \param fingerpad_normal a vector giving the direction of the fingertip contact surface (orthogonal to the medial axis of the distal phalanx and directed towards the inside of the hand). It is computed for the computed finger joint angles (if a solution of the inverse kinematics problem exists).
//! \param finger_index index of the chosen finger (1= thumb, 2= forefinger, 3= middlefinger, 4= ringfinger)
//! \return 1 in case of succcess, 0 otherwise
//! NB: the first joint of the thumb is not taken into account: it is supposed to be at its maximum value (90 degrees)
//! in opposition to the other fingers.
int gpSAHfinger_inverse_kinematics(p3d_matrix4 Twrist, gpHand_properties &hand, p3d_vector3 p, double q[4], p3d_vector3 fingerpad_normal, int finger_index)
{
  #ifdef DEBUG 
   if(finger_index<1 || finger_index>4 )
   {
     printf("%s: %d: gpSAHfinger_inverse_kinematics(): finger_index must be >= 1 and <=4 (finger_index= %d).\n",__FILE__,__LINE__, finger_index);
     return 0;
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

  q[0]= q[1]= q[2]= q[3]= 0;
  fingerpad_normal[0]= fingerpad_normal[1]= fingerpad_normal[2]= 0;

  switch(hand.type)
  {
    case GP_SAHAND_RIGHT:
     l0= hand.length_thumbBase;
     l1= hand.length_proxPha;
     l2= hand.length_midPha;
     l3= hand.length_distPha;

     switch(finger_index)
     {
       case 1: //thumb
          p3d_mat4Mult(Twrist, hand.Twrist_thumb, Tfinger_world);
          q[0]= 90*DEGTORAD;
       break;
       case 2: //forefinger
          p3d_mat4Mult(Twrist, hand.Twrist_forefinger, Tfinger_world);
       break;
       case 3: //middle finger
          p3d_mat4Mult(Twrist, hand.Twrist_middlefinger, Tfinger_world);
       break;
       case 4: //ring finger
          p3d_mat4Mult(Twrist, hand.Twrist_ringfinger, Tfinger_world);
       break;
     }
    break;
    default:
       printf("%s: %d: gpSAHfinger_inverse_kinematics(): this function only applies to GP_SAHAND_RIGHT et GP_SAHAND_LEFT hands.\n", __FILE__, __LINE__);
       return 0;
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
  {  return 0;  }

  a= 4*l1*l3;
  b= 2*l1*l2 + 2*l2*l3;
  c= -x*x - y*y - z*z + l1*l1 + l2*l2 + l3*l3 - 2*l1*l3;

  delta= b*b - 4*a*c;

  if(delta < 0)
  {  return 0;  }

  r1= ( -b + sqrt(delta) ) / (2*a);
  r2= ( -b - sqrt(delta) ) / (2*a);
  
  if( fabs(r1) > 1 && fabs(r2) > 1 )
  {  return 0;  } 

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
  {
    //printf("no valid  q3\n");   
    return 0;
  }

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
/*
        x2=  -sin(q[1])*(  l1*cos(q[2]) + l2*cos(q[2]+q[3]) + l3*cos(q[2]+2*q[3])  );
        y2=   cos(q[1])*(  l1*cos(q[2]) + l2*cos(q[2]+q[3]) + l3*cos(q[2]+2*q[3])  );
        z2=  -l1*sin(q[2]) - l2*sin(q[2]+q[3]) - l3*sin(q[2]+2*q[3]);*/

        if( sqrt( SQR(p2[0]-p[0]) + SQR(p2[1]-p[1]) + SQR(p2[2]-p[2]) ) < 10e-6)
        { return 1; }

      }
      if( q1_found[i] && q3_found[j] && q2_found[2*j+1] )
      { 
        q[0]= q0max;
        q[1]= q1[i];
        q[2]= q2[2*j+1];
        q[3]= q3[j];

        gpSAHfinger_forward_kinematics(Twrist, hand, q, p2, fingerpad_normal, finger_index);
/*
        x2=  -sin(q[1])*(  l1*cos(q[2]) + l2*cos(q[2]+q[3]) + l3*cos(q[2]+2*q[3])  );
        y2=   cos(q[1])*(  l1*cos(q[2]) + l2*cos(q[2]+q[3]) + l3*cos(q[2]+2*q[3])  );
        z2=  -l1*sin(q[2]) - l2*sin(q[2]+q[3]) - l3*sin(q[2]+2*q[3]);
*/
        //if( sqrt( SQR(x2-x) + SQR(y2-y) + SQR(z2-z) ) < 10e-6)
        if( sqrt( SQR(p2[0]-p[0]) + SQR(p2[1]-p[1]) + SQR(p2[2]-p[2]) ) < 10e-6)
        { return 1; }
      }

    }
  }


  return 0;
}



//! Deactivates the collision tests between an object and the fingertips of a robot (that has some).
//! \param robot the robot (its fingertip bodies must have specific names, defined in graspPlanning.h)
//! \param object the object
//! \param hand structure containing information about the hand geometry
//! \return 1 in case of success, 0 otherwise
int gpDeactivate_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_object_fingertips_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
   if(object==NULL)
   {
      printf("%s: %d: gpDeactivate_object_fingertips_collisions(): object is NULL.\n",__FILE__,__LINE__);
      return 0;
   }  
  #endif

  static bool already_warned= false;
  unsigned int i;
  std::string base_name, body_name;
  std::stringstream out;
  p3d_obj *fingertip;

  base_name+= GP_HAND_BODY_PREFIX + std::string(".") + GP_FINGER_BODY_PREFIX;

  for(i=1; i<=hand.nb_fingers; i++)
  {
     body_name= base_name;
     out << i;
     body_name+= out.str();
     body_name+= std::string(".") + GP_FINGERTIP;
     out.seekp(std::ios::beg); 

     fingertip= NULL;
     fingertip= get_robot_body_by_name(robot, (char *) body_name.c_str());
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
     { p3d_col_deactivate_pair_of_objects(fingertip, object);  }
  }

  return 1;
}


//! Activates the collision tests between an object and the fingertips of a robot (that has some).
//! \param robot the robot (its fingertip bodies must have specific names, defined in graspPlanning.h)
//! \param object the object
//! \param hand structure containing information about the hand geometry
//! \return 1 in case of success, 0 otherwise
int gpActivate_object_fingertips_collisions(p3d_rob *robot, p3d_obj *object, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpAactivate_object_fingertips_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
   if(object==NULL)
   {
      printf("%s: %d: gpAactivate_object_fingertips_collisions(): object is NULL.\n",__FILE__,__LINE__);
      return 0;
   }  
  #endif

  unsigned int i;
  std::string base_name, body_name;
  std::stringstream out;
  p3d_obj *fingertip;

  base_name= std::string(robot->name) + std::string(".") + GP_FINGERTIP;

  for(i=1; i<=hand.nb_fingers; i++)
  {
     body_name= base_name;
     out << i;
     body_name+= out.str();
     body_name+= std::string(".") + GP_FINGERTIP;;
     out.seekp(std::ios::beg); 

     fingertip= NULL;
     fingertip= p3d_get_body_by_name((char *) body_name.c_str());
     if(fingertip==NULL)
     {
       printf("%s: %d: gpActivate_object_fingertips_collisions(): robot \"%s\" should have a body named \"%s\".\n",__FILE__,__LINE__, robot->name, body_name.c_str());
     }
     p3d_col_activate_pair_of_objects(fingertip, object);
  }

  return 1;
}

//! Opens the gripper or hand at its maximum.
//! \param robot the robot (its joints must have specific names, defined in graspPlanning.h)
//! \param hand structure containing information about the hand geometry
//! \return 1 in case of success, 0 otherwise
int gpOpen_hand(p3d_rob *robot, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpOpen_gripper(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif  

  std::vector<double> q;

  q.resize(hand.nb_dofs);

  switch(hand.type)
  {
    case GP_GRIPPER:
      q[0]= hand.max_opening_jnt_value;
    break;
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
     return 0;
    break;
  }

  gpSet_hand_configuration(robot, hand , q);

  return 1;
}

//! Locks the DOFs of the robot's platform for future planning.
//! \param robot pointer to the robot
//! \return 1 in case of success, 0 otherwise
int gpLock_platform(p3d_rob *robot)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpLock_platform(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *platformJoint= NULL;

  platformJoint= get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  if(platformJoint==NULL)
  {  return 0;   }

  p3d_jnt_set_dof_is_user(platformJoint, 0, FALSE);
  p3d_jnt_set_dof_is_user(platformJoint, 1, FALSE);
  p3d_jnt_set_dof_is_user(platformJoint, 2, FALSE);
  p3d_jnt_set_dof_is_user(platformJoint, 3, FALSE);
  p3d_jnt_set_dof_is_user(platformJoint, 4, FALSE);
  p3d_jnt_set_dof_is_user(platformJoint, 5, FALSE);

  p3d_jnt_set_is_active_for_planner(platformJoint, FALSE);

  return 1;
}

//! Unlocks the DOFs of the robot's platform for future planning.
//! \param robot pointer to the robot
//! \return 1 in case of success, 0 otherwise
int gpUnlock_platform(p3d_rob *robot)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpUnlock_platform(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *platformJoint= NULL;

  platformJoint= get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  if(platformJoint==NULL)
  {  return 0;   }

  p3d_jnt_set_dof_is_user(platformJoint, 0, TRUE);
  p3d_jnt_set_dof_is_user(platformJoint, 1, TRUE);
  p3d_jnt_set_dof_is_user(platformJoint, 2, TRUE);
  p3d_jnt_set_dof_is_user(platformJoint, 3, TRUE);
  p3d_jnt_set_dof_is_user(platformJoint, 4, TRUE);
  p3d_jnt_set_dof_is_user(platformJoint, 5, TRUE);

  p3d_jnt_set_is_active_for_planner(platformJoint, TRUE);

  return 1;
}


//! Locks the DOFs of the robot's arm for future planning.
//! \param robot pointer to the robot
//! \param arm_type type of the arm
//! \return 1 in case of success, 0 otherwise
int gpLock_arm(p3d_rob *robot, gpArm_type arm_type)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpLock_arm(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *armJoint= NULL;

  switch(arm_type)
  {
    case GP_PA10:
      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT1);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(armJoint, FALSE);

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT2);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(armJoint, FALSE);

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT3);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(armJoint, FALSE);

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT4);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(armJoint, FALSE);
        
      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT5);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(armJoint, FALSE);

      armJoint= get_robot_jnt_by_name(robot, GP_WRISTJOINT);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(armJoint, FALSE);
    break;
    default:
      printf("%s: %d: gpLock_arm(): unsupported arm type.\n",__FILE__,__LINE__);
      return 0;
    break;
  }


  return 1;
}



//! Unlocks the DOFs of the robot's arm for future planning.
//! \param robot pointer to the robot
//! \param arm_type type of the arm
//! \return 1 in case of success, 0 otherwise
int gpUnlock_arm(p3d_rob *robot, gpArm_type arm_type)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpUnlock_arm(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *armJoint= NULL;

  switch(arm_type)
  {
    case GP_PA10:
      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT1);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(armJoint, TRUE); 

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT2);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(armJoint, TRUE); 

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT3);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(armJoint, TRUE); 

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT4);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(armJoint, TRUE); 

      armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT5);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(armJoint, TRUE); 

      armJoint= get_robot_jnt_by_name(robot, GP_WRISTJOINT);
      if(armJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(armJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(armJoint, TRUE); 
    break;
    default:
      printf("%s: %d: gpUnlock_arm(): unsupported arm type.\n",__FILE__,__LINE__);
      return 0;
    break;
  }

  return 1;
}

//! Locks the DOFs of the robot's hand for future planning.
//! \param robot pointer to the robot
//! \param hand_type type of the hand
//! \return 1 in case of success, 0 otherwise
int gpLock_hand(p3d_rob *robot, gpHand_type hand_type)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpLock_hand(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *fingerJoint= NULL;

  switch(hand_type)
  {
    case GP_GRIPPER:
      fingerJoint= get_robot_jnt_by_name(robot, GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 
    break;
    case GP_SAHAND_RIGHT:
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT5);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, FALSE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, FALSE); 
    break;
    default:
      printf("%s: %d: gpLock_hand(): unsupported hand type.\n",__FILE__,__LINE__);
      return 0;
    break;
  }

  return 1;
}


//! Unlocks the DOFs of the robot's hand for future planning.
//! \param robot pointer to the robot
//! \param hand_type type of the hand
//! \return 1 in case of success, 0 otherwise
int gpUnlock_hand(p3d_rob *robot, gpHand_type hand_type)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpUnlock_hand(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *fingerJoint= NULL;

  switch(hand_type)
  {
    case GP_GRIPPER:
      fingerJoint= get_robot_jnt_by_name(robot, GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 
    break;
    case GP_SAHAND_RIGHT:
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT5);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT4);
      if(fingerJoint==NULL)
      {  return 0;   }
      p3d_jnt_set_dof_is_user(fingerJoint, 0, TRUE);
      p3d_jnt_set_is_active_for_planner(fingerJoint, TRUE); 
    break;
    default:
      printf("%s: %d: gpUnlock_hand(): unsupported hand type.\n",__FILE__,__LINE__);
      return 0;
    break;
  }

  return 1;
}


//! Gets the robot's platform configuration (x,y,theta).
//! \param robot pointer to the robot
//! \param x where to copy the current x position
//! \param y where to copy the current y position
//! \param theta where to copy the current theta position
//! \return 1 in case of success, 0 otherwise
int gpGet_platform_configuration(p3d_rob *robot, double &x, double &y, double &theta)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGet_platform_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif

  p3d_jnt *platformJoint= NULL;

  platformJoint= get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  if(platformJoint==NULL)
  {  return 0; }

  x= platformJoint->dof_data[0].v;
  y= platformJoint->dof_data[1].v;
  theta= platformJoint->dof_data[5].v;

  return 1;
}


//! Sets the robot's platform configuration to the given values.
//! \param robot pointer to the robot
//! \param x desired X position
//! \param y desired Y position
//! \param theta desired orientation around Z-axis
//! \return 1 in case of success, 0 otherwise
int gpSet_platform_configuration(p3d_rob *robot, double x, double y, double theta)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpSet_platform_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return 0;
   }
  #endif

  double x_min, x_max, y_min, y_max, theta_min, theta_max;
  configPt q= NULL;
  p3d_jnt *platformJoint= NULL;

  platformJoint= get_robot_jnt_by_name(robot, GP_PLATFORMJOINT);

  if(platformJoint==NULL)
  {  return 0; }

  x_min= platformJoint->dof_data[0].vmin;
  x_max= platformJoint->dof_data[0].vmax;
  y_min= platformJoint->dof_data[1].vmin;
  y_max= platformJoint->dof_data[1].vmax;
  theta_min= platformJoint->dof_data[5].vmin;
  theta_max= platformJoint->dof_data[5].vmax;

  if( (x<x_min) || (x>x_max) )
  {
    printf("%s: %d: gpSet_platform_configuration(): desired x value (%f) is out of bounds (%f %f).\n",__FILE__,__LINE__,x, x_min,x_max);
    printf("Its value will not be changed.\n");
    x= platformJoint->dof_data[0].v;
  }
  if( (y<y_min) || (y>y_max) )
  {
    printf("%s: %d: gpSet_platform_configuration(): desired y value (%f) is out of bounds (%f %f).\n",__FILE__,__LINE__,y, y_min,y_max);
    printf("Its value will not be changed.\n");
    y= platformJoint->dof_data[1].v;
  }
//   if( (theta<theta_min) || (theta>theta_max) )
//   {
//     printf("%s: %d: gpSet_platform_configuration(): desired theta value (%f) is out of bounds (%f %f).\n",__FILE__,__LINE__theta, theta_min,theta_max);
//     printf("Its value will not be changed.\n");
//     theta= platformJoint->dof_data[5].v;
//   }

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);
print_config(robot, q);
  q[platformJoint->index_dof]= x;
  q[platformJoint->index_dof+1]= y;
  q[platformJoint->index_dof+5]= theta;
  p3d_set_and_update_this_robot_conf(robot, q);
print_config(robot, q);
  p3d_destroy_config(robot, q);

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);
print_config(robot, q);
  p3d_destroy_config(robot, q);

  return 1;
}


//! Gets the robot's arm configuration.
//! \param robot pointer to the robot
//! \param arm_type arm type (for now, only PA10 is supported)
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 1 in case of success, 0 otherwise
int gpGet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, double &q1, double &q2, double &q3, double &q4, double &q5, double &q6)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpGet_arm_configuration(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  p3d_jnt *armJoint= NULL;

  switch(arm_type)
  {
    case GP_PA10:
        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT1);
        if(armJoint==NULL)
        {  return 0; }
        q1= armJoint->dof_data[0].v;

        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT2);
        if(armJoint==NULL)
        {  return 0; }
        q2= armJoint->dof_data[0].v;

        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT3);
        if(armJoint==NULL)
        {  return 0; }
        q3= armJoint->dof_data[0].v;

        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT4);
        if(armJoint==NULL)
        {  return 0; }
        q4= armJoint->dof_data[0].v;

        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT5);
        if(armJoint==NULL)
        {  return 0; }
        q5= armJoint->dof_data[0].v;

        armJoint= get_robot_jnt_by_name(robot, GP_WRISTJOINT);
        if(armJoint==NULL)
        {  return 0; }
        q6= armJoint->dof_data[0].v;
    break;
    default:
      printf("%s: %d: gpGet_arm_configuration(): unsupported arm type.\n",__FILE__,__LINE__);
      return 0;
    break;
  }
  
  return 1;
}



//! Sets the robot's arm configuration with the given values.
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
//! \return 1 in case of success, 0 otherwise
int gpSet_arm_configuration(p3d_rob *robot, gpArm_type arm_type, double q1, double q2, double q3, double q4, double q5, double q6, bool verbose)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpSet_arm_configuration(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  bool isValid;
  double qmin, qmax;
  p3d_jnt *armJoint= NULL;
  configPt q= NULL;

  q= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q);

  switch(arm_type)
  {
    case GP_PA10:
        ////////////////////////q1////////////////////////////
        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT1);
        if(armJoint==NULL)
        {  
          p3d_destroy_config(robot, q);
          return 0; 
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
          return 0;
        }
        /////////////////////////////////////////////////////
      
      
        ////////////////////////q2////////////////////////////
        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT2); 
        if(armJoint==NULL)
        {  
          p3d_destroy_config(robot, q);
          return 0; 
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
          return 0;
        }
        /////////////////////////////////////////////////////
      
      
        ////////////////////////q3////////////////////////////
        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT3);
        if(armJoint==NULL)
        {  
          p3d_destroy_config(robot, q);
          return 0; 
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
          return 0;
        }
        /////////////////////////////////////////////////////
      
      
        ////////////////////////q4////////////////////////////
        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT4);
        if(armJoint==NULL)
        {  
          p3d_destroy_config(robot, q);
          return 0; 
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
          return 0;
        }
        /////////////////////////////////////////////////////
      
      
        ////////////////////////q5////////////////////////////
        armJoint= get_robot_jnt_by_name(robot, GP_ARMJOINT5);
        if(armJoint==NULL)
        {  
          p3d_destroy_config(robot, q);
          return 0; 
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
          return 0;
        }
        /////////////////////////////////////////////////////
      
      
        ////////////////////////q6////////////////////////////
        armJoint= get_robot_jnt_by_name(robot, GP_WRISTJOINT);
        if(armJoint==NULL)
        {  
          p3d_destroy_config(robot, q);
          return 0; 
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
          return 0;
        }
    break;
    default:
      printf("%s: %d: gpSet_arm_configuration(): unsupported arm type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot, q);
      return 0;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, q);
  p3d_destroy_config(robot, q);
  
  return 1;
}


//! Sets the hand/gripper configuration of a robot with the configuration contained in a gpGrasp variable.
//! It only modifies the parameters of the hand.
//! \param robot pointer to the robot
//! \param hand information concerning the hand
//! \param grasp the grasp to set
//! \return 1 in case of success, 0 otherwise
int gpSet_grasp_configuration(p3d_rob *robot, gpHand_properties &hand, const gpGrasp &grasp)
{
  #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_grasp_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return 0;
  }
  #endif

  p3d_jnt *fingerJoint= NULL;

  if(grasp.config.size()!=hand.nb_dofs)
  {
    printf("%s: %d: gpSet_grasp_configuration(): the configuration vector of the input grasp has a bad size (%d instead of %d).\n",__FILE__,__LINE__,grasp.config.size(), hand.nb_dofs);
    return 0;
  }

  configPt q= NULL;
  q= p3d_get_robot_config(robot);
  switch(hand.type)
  {
    case GP_GRIPPER:
      fingerJoint= get_robot_jnt_by_name(robot, GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[0];
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT1);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[0];
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT2);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[1];
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT3);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[2];
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT4);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[3];

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[4];

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[5];
      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[6];

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[7];
      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[8];
      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[9];

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[10];
      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[11];
      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0; }
      q[fingerJoint->index_dof]= grasp.config[12];
    break;
    default:
       printf("%s: %d: gpSet_grasp_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return 0;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, q);
  p3d_destroy_config(robot, q);

  return 1;
}


//! Gets the hand/gripper's configuration of a robot and copies it in a std::vector.
//! \param robot pointer to the robot
//! \param hand information about the hand
//! \param q a std::vector that will be filled with the current joint parameters of the hand
//! \return 1 in case of success, 0 otherwise
int gpGet_hand_configuration(p3d_rob *robot, gpHand_properties &hand, std::vector<double> q)
{
  #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGet_hand_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return 0;
  }
  #endif

  p3d_jnt *fingerJoint= NULL;

  q.resize(hand.nb_dofs);

  switch(hand.type)
  {
    case GP_GRIPPER:
      fingerJoint= get_robot_jnt_by_name(robot, GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[0]= fingerJoint->dof_data[0].v;
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT1);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[0]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT2);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[1]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT3);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[2]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT4);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[3]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[4]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[5]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[6]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[7]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[8]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[9]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[10]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[11]= fingerJoint->dof_data[0].v;

      fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3);
      if(fingerJoint==NULL)
      {  return 0;  }
      q[12]= fingerJoint->dof_data[0].v;
    break;
    default:
      printf("%s: %d: gpGet_hand_configuration(): unsupported hand type.\n",__FILE__,__LINE__);
      return 0;
    break;
  }
  
  return 1;
}

//! Sets the hand/gripper's configuration of a robot with the configuration contained in a std::vector.
//! It only modifies the parameters of the hand.
//! \return 1 in case of success, 0 otherwise
int gpSet_hand_configuration(p3d_rob *robot, gpHand_properties &hand, std::vector<double> q, bool verbose)
{
  #ifdef DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpSet_grasp_configuration(): robot is NULL.\n",__FILE__,__LINE__);
    return 0;
  }
  #endif

  unsigned int i;
  bool isValid;
  double qmin, qmax;
  p3d_jnt *fingerJoint= NULL;

  if(q.size()!=hand.nb_dofs)
  {
    printf("%s: %d: gpSet_hand_configuration(): the input configuration vector has a bad size (%d instead of %d).\n",__FILE__,__LINE__,q.size(), hand.nb_dofs);
    return 0;
  }

  configPt qcur= NULL;
  qcur= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &qcur);

  switch(hand.type)
  {
    case GP_GRIPPER:
      fingerJoint= get_robot_jnt_by_name(robot, GP_GRIPPERJOINT);
      if(fingerJoint==NULL)
      {  
        p3d_destroy_config(robot, qcur);
        return 0;
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
         return 0;
      }
      qcur[fingerJoint->index_dof]= q[0];
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      for(i=0; i<12; i++)
      {
        switch(i)
        {
          case 0: fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT1); break;
          case 1: fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT2); break;
          case 2: fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT3); break;
          case 3: fingerJoint= get_robot_jnt_by_name(robot, GP_THUMBJOINT4); break; 
          case 4: fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT1); break;
          case 5: fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT2); break;
          case 6: fingerJoint= get_robot_jnt_by_name(robot, GP_FOREFINGERJOINT3); break;
          case 7: fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT1); break;
          case 8: fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT2); break;
          case 9: fingerJoint= get_robot_jnt_by_name(robot, GP_MIDDLEFINGERJOINT3); break;
          case 10: fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT1); break;
          case 11: fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT2); break;
          case 12: fingerJoint= get_robot_jnt_by_name(robot, GP_RINGFINGERJOINT3); break;
        }
        if(fingerJoint==NULL)
        {  
          p3d_destroy_config(robot, qcur);
          return 0;
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
          return 0;
        }
        qcur[fingerJoint->index_dof]= q[i];
      }
    break;
    default:
       printf("%s: %d: gpSet_grasp_configuration(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return 0;
    break;
  }

  p3d_set_and_update_this_robot_conf(robot, qcur);
  p3d_destroy_config(robot, qcur);

  return 1;
}



//! Sets the robot's arm to a "folded" configuration so that it takes the less room
//! (when the base is moving for instance).
//! \param robot pointer to the robot
//! \param arm_type arm type (for now, only PA10 is supported)
//! \return 1 in case of success, 0 otherwise
int gpFold_arm(p3d_rob *robot, gpArm_type arm_type)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpFold_arm(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  int result;
  double q1, q2, q3, q4, q5, q6;
  configPt q0= NULL;

  q0= p3d_alloc_config(robot);

  //for vertical jido:
  q1= DEGTORAD*(-90);
  q2= DEGTORAD*(90);
  q3= DEGTORAD*(45);
  q4= DEGTORAD*(0);
  q5= DEGTORAD*(-45);
  q6= DEGTORAD*(0);

  switch(arm_type)
  {
    case GP_PA10:
      result= gpSet_arm_configuration(robot, GP_PA10, q1, q2, q3, q4, q5, q6); 
    break;
    default:
      printf("%s: %d: gpFold_arm(): unsupported arm type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot, q0);
      return 0;
    break;
  }

  if(p3d_col_test())
  {
    p3d_set_and_update_this_robot_conf(robot, q0);
    result= 0;
  }

//   if(result==0)
//   {   printf("%s: %d: gpFold_arm(): the arm could not be folded.\n",__FILE__,__LINE__);   }

  p3d_destroy_config(robot, q0);

  return result;
}


//! Deactivates all the collision tests for the arm bodies of the specified robot.
//! \param robot the robot (its arm bodies must have specific names, defined in graspPlanning.h)
//! \return 1 in case of success, 0 otherwise
int gpDeactivate_arm_collisions(p3d_rob *robot)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_arm_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  int i;
  std::string arm_body_base_name, body_name;

  arm_body_base_name= std::string(robot->name) + "." + GP_ARM_BODY_PREFIX + std::string(".");

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

  return 1;
}

//! Activates all the collision tests for the arm bodies of the specified robot.
//! \param robot the robot (its arm bodies must have specific names, defined in graspPlanning.h)
//! \return 1 in case of success, 0 otherwise
int gpActivate_arm_collisions(p3d_rob *robot)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_arm_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif
  int i;
  std::string arm_body_base_name, body_name;

  arm_body_base_name= std::string(robot->name) + "." + GP_ARM_BODY_PREFIX + std::string(".");

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

  return 1;
}

//! Deactivates all the collision tests for the hand bodies of the specified robot.
//! \param robot the robot (its hand bodies must have specific names, defined in graspPlanning.h)
//! \return 1 in case of success, 0 otherwise
int gpDeactivate_hand_collisions(p3d_rob *robot)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_hand_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  int i;
  std::string hand_body_base_name, body_name;

  hand_body_base_name= std::string(robot->name) + "." + GP_HAND_BODY_PREFIX + std::string(".");

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

  return 1;
}

//! Activates all the collision tests for the hand bodies of the specified robot.
//! \param robot the robot (its hand bodies must have specific names, defined in graspPlanning.h)
//! \return 1 in case of success, 0 otherwise
int gpActivate_hand_collisions(p3d_rob *robot)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_hand_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  int i;
  std::string hand_body_base_name, body_name;

  hand_body_base_name= std::string(robot->name) + "." + GP_HAND_BODY_PREFIX + std::string(".");

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

  return 1;
}

//! Deactivates all the collision tests for the specified finger of the specified robot.
//! \param robot the robot (its finger bodies must have specific names, defined in graspPlanning.h)
//! \param finger_index the number of the finger ( 1 <= finger_index <= hand number of fingers)
//! \param hand a gpHand_properties variable filled with information concerning the chosen hand characteristics
//! \return 1 in case of success, 0 otherwise
int gpDeactivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpDeactivate_finger_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
   if( finger_index < 1 || finger_index > hand.nb_fingers )
   {
      printf("%s: %d: gpDeactivate_finger_collisions(): the finger index exceeds the hand number of fingers.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  int i;
  std::string finger_body_base_name, body_name;
  std::stringstream out;

  finger_body_base_name= std::string(robot->name) + "." + GP_HAND_BODY_PREFIX + std::string(".") + GP_FINGER_BODY_PREFIX;
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

  return 1;
}

//! Activates all the collision tests for the specified finger of the specified robot.
//! \param robot the robot (its finger bodies must have specific names, defined in graspPlanning.h)
//! \param finger_index the number of the finger ( 1 <= finger_index <= hand number of fingers)
//! \param hand a gpHand_properties variable filled with information concerning the chosen hand characteristics
//! \return 1 in case of success, 0 otherwise
int gpActivate_finger_collisions(p3d_rob *robot, unsigned int finger_index, gpHand_properties &hand)
{
  #ifdef DEBUG
   if(robot==NULL)
   {
      printf("%s: %d: gpActivate_finger_collisions(): robot is NULL.\n",__FILE__,__LINE__);
      return 0;
   }
   if( finger_index < 1 || finger_index > hand.nb_fingers )
   {
      printf("%s: %d: gpActivate_finger_collisions(): the finger index exceeds the hand number of fingers.\n",__FILE__,__LINE__);
      return 0;
   }
  #endif

  int i;
  std::string finger_body_base_name, body_name;
  std::stringstream out;

  finger_body_base_name= std::string(robot->name) + "." + GP_HAND_BODY_PREFIX + std::string(".") + GP_FINGER_BODY_PREFIX;

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

  return 1;
}
