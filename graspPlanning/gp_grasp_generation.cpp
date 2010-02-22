
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "GraspPlanning-pkg.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/lightPlanner.h"

#include <time.h>
#include <dirent.h>
#include <errno.h>
#include <sys/times.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <math.h>
#include <stdio.h>
#include <string>
#include <sstream>


// 10% ouverture pince pour test collision
//#define GRIP_OPEN_PERCENT 1.1
// 100% ouverture pince pour test collision
#define GRIP_OPEN_PERCENT 1.1

//! @ingroup graspPlanning 
//! Computes a set of grasps from a grasp frame for the SAHand.
//! \param robot the hand robot (a freeflying robot composed of hand bodies only)
//! \param object the object to grasp
//! \param body_index the object poly to grasp (set to 0 if you do not know what to choose).
//! \param gframe the grasp frame (a 4x4 homogeneous transform matrix)
//! \param hand variable containing information about the hand geometry
//! \param graspList a grasp list the computed set of grasps will be added to
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasps_from_grasp_frame_SAHand(p3d_rob *robot, p3d_rob *object, int body_index, p3d_matrix4 gFrame, gpHand_properties &hand, gpKdTree &kdtree, std::list<class gpGrasp> &graspList)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
     printf("%s: %d: gpGrasps_from_grasp_frame_SAHand(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
  }
  if(object==NULL)
  {
     printf("%s: %d: gpGrasps_from_grasp_frame_SAHand(): object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
  }
  #endif

  unsigned int i, j;
  double fingertip_radius;
  p3d_vector3 p, center, contact_normal, fingerpad_normal;
  double q[4][4], q_default[4]= {M_PI_2, 0.0, 0.0, 0.0};
  double qik[4];
  p3d_matrix4 objectFrame, objectFrame_inv, Twrist, Twrist_world, T;
  configPt config0= NULL, config= NULL;

  std::vector<gpContact> contacts;
  std::list<gpContact> points;
  std::list<gpContact>::iterator iter;
  gpGrasp grasp;

  gpActivate_hand_collisions(robot);
  XYZ_ENV->cur_robot= robot;

  p3d_mat4Mult(gFrame, hand.Tgrasp_frame_hand, Twrist);

  fingertip_radius= hand.fingertip_radius;
  contacts.reserve(6);

  //memorize current robot configuration:
  config= p3d_alloc_config(robot);

  config0= p3d_get_robot_config(robot);

  p3d_get_body_pose(object, body_index, objectFrame);
  p3d_matInvertXform(objectFrame, objectFrame_inv);

  p3d_mat4Mult(objectFrame_inv, Twrist, Twrist_world);

  gpInverse_geometric_model_freeflying_hand(robot, objectFrame, gFrame, hand, config);
  p3d_set_and_update_this_robot_conf(robot, config);

  p3d_copy_config_into(robot, config, &robot->ROBOT_POS);

  // Deactivate the collision for all fingers
  gpDeactivate_finger_collisions(robot, 1, hand);
  gpDeactivate_finger_collisions(robot, 2, hand);
  gpDeactivate_finger_collisions(robot, 3, hand);
  gpDeactivate_finger_collisions(robot, 4, hand);

  // If the hand robot is already colliding with the object, it's the palm and there is no way
  // to find a collision-free configuration:
  if(p3d_col_test_robot_other(robot, object, 0))
  {
    gpActivate_finger_collisions(robot, 1, hand);
    gpActivate_finger_collisions(robot, 2, hand);
    gpActivate_finger_collisions(robot, 3, hand);
    gpActivate_finger_collisions(robot, 4, hand);
    p3d_set_and_update_this_robot_conf(robot, config0);
    p3d_destroy_config(robot, config0);
    p3d_destroy_config(robot, config); 
    return GP_ERROR;
  }

  for(i=0; i<4; ++i) //for each finger:
  {
     q[i][0]= q_default[0];
     q[i][1]= q_default[1];
     q[i][2]= q_default[2];
     q[i][3]= q_default[3];
     gpSet_SAHfinger_joint_angles(robot, hand, q_default, i+1, 0);

     gpActivate_finger_collisions(robot, i+1, hand);
     p3d_mat4Mult(Twrist, hand.Twrist_finger[i], T);

     for(j=0; j<hand.workspace.size(); ++j)
     {
       p3d_xformPoint(T, hand.workspace[j].center, center);
       points.clear();
       kdtree.sphereIntersection(center, hand.workspace[j].radius, points);
       for(iter=points.begin(); iter!=points.end(); iter++)
       { 
          p3d_xformPoint(objectFrame_inv, iter->position, p); //object frame -> world frame

          p3d_xformVect(objectFrame_inv, iter->normal, contact_normal); //object frame -> world frame
          if(gpSAHfinger_inverse_kinematics(Twrist_world, hand, p, qik, fingerpad_normal, i+1)==GP_OK)
          {
//             printf("can reach point: finger %d (%f %f %f)\n", i, q[i][1]*RADTODEG, q[i][2]*RADTODEG, q[i][3]*RADTODEG);

            // contact normal and fingerpad normal must be in opposite directions:
            if( p3d_vectDotProd(contact_normal, fingerpad_normal) > -0.3 )
            {  continue;  }

            gpSet_SAHfinger_joint_angles(robot, hand, qik, i+1, 0);

            // if collision, bring the finger back to its default configuration
            if(p3d_col_test_self_collision(robot, 0) || p3d_col_test_robot_other(robot, object, 0))
            {  
               gpSet_SAHfinger_joint_angles(robot, hand, q_default, i+1, 0);
               continue;
            }
            q[i][0]= qik[0];
            q[i][1]= qik[1];
            q[i][2]= qik[2];
            q[i][3]= qik[3];

            contacts.resize(contacts.size()+1);

            contacts.back().position[0]= iter->position[0] - hand.fingertip_radius*iter->normal[0];
            contacts.back().position[1]= iter->position[1] - hand.fingertip_radius*iter->normal[1];
            contacts.back().position[2]= iter->position[2] - hand.fingertip_radius*iter->normal[2];

            contacts.back().normal[0]= iter->normal[0];
            contacts.back().normal[1]= iter->normal[1];
            contacts.back().normal[2]= iter->normal[2];

            contacts.back().surface= object->o[body_index]->pol[0]->poly;

            contacts.back().face= iter->face;
            contacts.back().fingerID= i+1;
            contacts.back().computeBarycentricCoordinates();
            contacts.back().computeCurvature();
            goto Next;
          }
          else {  /*printf("%s: %d: can't reach point\n",__FILE__,__LINE__); printf("finger %d\n",i);*/ }

       }

     }
Next:;
     if(i==0 && contacts.empty())
     {
       gpActivate_finger_collisions(robot, 1, hand);
       gpActivate_finger_collisions(robot, 2, hand);
       gpActivate_finger_collisions(robot, 3, hand);
       gpActivate_finger_collisions(robot, 4, hand);
       p3d_set_and_update_this_robot_conf(robot, config0);
       p3d_destroy_config(robot, config0);
       p3d_destroy_config(robot, config);
       return GP_ERROR;
     }
  }

  if(contacts.size() < 3)
  {
    gpActivate_finger_collisions(robot, 1, hand);
    gpActivate_finger_collisions(robot, 2, hand);
    gpActivate_finger_collisions(robot, 3, hand);
    gpActivate_finger_collisions(robot, 4, hand);
    p3d_set_and_update_this_robot_conf(robot, config0);
    p3d_destroy_config(robot, config0);
    p3d_destroy_config(robot, config);
    return GP_ERROR;
  }

  //test collision for the whole hand configuration:
  gpActivate_finger_collisions(robot, 1, hand);
  gpActivate_finger_collisions(robot, 2, hand);
  gpActivate_finger_collisions(robot, 3, hand);
  gpActivate_finger_collisions(robot, 4, hand);

  if( p3d_col_test_robot_other(robot, object, 0) || p3d_col_test_self_collision(robot, 0) )
  {
    p3d_set_and_update_this_robot_conf(robot, config0);
    p3d_destroy_config(robot, config0);
    p3d_destroy_config(robot, config);
    return GP_ERROR;
  }

  grasp.hand_type= hand.type;
  grasp.ID= graspList.size() + 1;
  grasp.object= object;
  grasp.object_name= object->name;
  grasp.body_index= body_index;
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
    grasp.config[3*i+1]= q[i][1];
    grasp.config[3*i+2]= q[i][2];
    grasp.config[3*i+3]= q[i][3];
  }

// grasp.print();
  p3d_set_and_update_this_robot_conf(robot, config0);
  p3d_destroy_config(robot, config0);
  p3d_destroy_config(robot, config);

  graspList.push_back(grasp);

  return GP_OK;
}

//! @ingroup graspPlanning 
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
int gpGrasps_from_grasp_frame_gripper(p3d_polyhedre *polyhedron, p3d_matrix4 gFrame, gpHand_properties &hand, std::list<class gpGrasp> &graspList)
{
    #ifdef GP_DEBUG
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


    gpContact contact;

    //vector contenant les contacts trouvés pour les doigts 1 et 2 :
    std::vector<gpContact> contacts1;
    contacts1.reserve(10);
    std::vector<gpContact> contacts2;
    contacts2.reserve(10);


    unsigned int nb_contacts12= 0; //nombre actuel de paires de contacts trouvées pour les doigts 1 et 2

    int nb_grasps= 0; //nombre actuel de prises trouvées (contacts des doigts 1, 2 et 3)
    bool isNeighbourIntersected= false;

    p3d_vector3 *points= polyhedron->the_points;
    unsigned int nb_faces=  (unsigned int) polyhedron->nb_faces;
    p3d_face *faces= polyhedron->the_faces;


    ///////////////////////////premier point de contact///////////////////////////
    for(i=0; i<nb_faces; i++)
    {
//         if(part!=0 && faces[i].part!=part)
//            continue;

	ind= faces[i].the_indexs_points;

        if(faces[i].plane==NULL)
        {
          printf("%s: %d: gpGrasp_from_grasp_frame_gripper(): a plane of a face has not been computed -> call p3d_build_planes() first.\n",__FILE__,__LINE__);
          continue;
        }

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

// printf("i= %d (%d %d %d) nbinter= %d\n",i, ind[0],ind[1],ind[2],nbinter);
//  printf("t (%f %f %f) (%f %f %f) (%f %f %f)\n",points[ind[0]-1][0],points[ind[0]-1][1],points[ind[0]-1][2],points[ind[1]-1][0],points[ind[1]-1][1],points[ind[1]-1][2],points[ind[2]-1][0],points[ind[2]-1][1],points[ind[2]-1][2]);
// printf("px (%f %f %f) \n",px[0],px[1],px[2]);
// printf("p1_s (%f %f %f) \n",p1_s[0],p1_s[1],p1_s[2]);

        if(nbinter!=0)
        {
         // shift = fingertip_radius*normale:
         p3d_vectScale(faces[i].plane->normale, shift, fingertip_radius);

         // Le centre du premier doigt:
         p3d_vectAdd(p1_s, shift, p1);

          ///////////////////////////recherche d'un deuxieme point de contact///////////////////////////
          for(j=0; j<nb_faces; j++)
          {
	      ind= faces[j].the_indexs_points;

              // Les deux premiers contacts doivent avoir des normales dans des directions non
              // opposees:
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
                    if(nbinter==2) // s'il y avait un deuxieme point d'intersection
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
            // on a trouve une paire p1p2:
            contact.surface= polyhedron;
            contact.face= i;
            p3d_vectCopy(p1_s, contact.position);
            p3d_vectCopy(faces[i].plane->normale, contact.normal);
            contact.computeBarycentricCoordinates();
            contact.computeCurvature();
            contact.mu= GP_FRICTION_COEFFICIENT;
            contacts1.push_back(contact);

            contact.surface= polyhedron;
            contact.face= j;
            p3d_vectCopy(p2_s, contact.position);
            p3d_vectCopy(faces[j].plane->normale, contact.normal);
            contact.computeBarycentricCoordinates();
            contact.computeCurvature();
            contact.mu= GP_FRICTION_COEFFICIENT;
            contacts2.push_back(contact);

            nb_contacts12++;
          }

        }

    }
printf("contacts1: %d\n",contacts1.size());
printf("contacts2: %d\n",contacts2.size());

    if( nb_contacts12==0 ) //pas d'intersection (le repere de saisie est hors du volume de l'objet)
    {
       contacts1.clear();
       contacts2.clear();
       return 0;
    }

printf("nb_contacts12= %d\n", nb_contacts12);
    ///////////////////////////troisieme point de contact///////////////////////////
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

      //  nouvel axe Z (normale au plan forme par les points (origine du repère initial, p1, p2))
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
//             grasp.config[0]= hand.min_opening_jnt_value + ( (grasp.finger_opening - hand.min_opening)/(hand.max_opening - hand.min_opening) )*(hand.max_opening_jnt_value - hand.min_opening_jnt_value);
            grasp.config[0]= hand.qmin.at(0) + ( (grasp.finger_opening - hand.min_opening)/(hand.max_opening - hand.min_opening) )*(hand.qmax.at(0) - hand.qmin.at(0));

            if(grasp.config[0] <= hand.min_opening_jnt_value || grasp.config[0] >= hand.max_opening_jnt_value)
            {   continue;  }

            if(isnan(grasp.finger_opening))
            {
              grasp.finger_opening= hand.min_opening;
//               grasp.config[0]= hand.min_opening_jnt_value;
              grasp.config[0]= hand.qmin.at(0);
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

            grasp.contacts.resize(3);
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
            grasp.contacts[2].computeBarycentricCoordinates();
            grasp.contacts[2].computeCurvature();
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


//! From each grasp configuration of a list, computes a hand configuration the most open possible starting 
//! from the grasping configuration.
//! The finger joint angles are incrementally modified to open the hand until there is a collision
//! with the object. If a collision occurs, the finger joint angle is chosen at the middle
//! of its initial value and the last value.
//! \param graspList the original grasp list
//! \param robot the hand robot (a freeflying robot only composed of the hand/gripper bodies)
//! \param object the grasped object
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_compute_open_configs(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp)
{
  #ifdef GP_DEBUG
  if(robot==NULL)
  {
    printf("%s: %d: gpGrasp_compute_open_configs(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(object==NULL)
  {
    printf("%s: %d: gpGrasp_compute_open_configs(): object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  unsigned int i, j, k, nbSteps;
  double qnew[4];
  p3d_matrix4 objectFrame;
  configPt config0, config;
  std::vector<bool> blocked;
  std::vector<double> q, qstart, qstop, delta;
  std::list<gpGrasp>::iterator igrasp;

  //memorize the robot current configuration:
  config0= p3d_get_robot_config(robot);
  config= p3d_alloc_config(robot);

  nbSteps= 10;

  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    if(igrasp->hand_type!=handProp.type) 
    {
      printf("%s: %d: gpGrasp_compute_open_configs(): the gpHand_properties of a grasp mismatches the input gpHand_properties.\n",__FILE__,__LINE__);
      continue;
    }

    p3d_get_body_pose(object, igrasp->body_index, objectFrame);

    gpInverse_geometric_model_freeflying_hand(robot, objectFrame, igrasp->frame, handProp, config);
    p3d_set_and_update_this_robot_conf(robot, config);

    igrasp->openConfig= igrasp->config;

    switch(igrasp->hand_type)
    {
      case GP_GRIPPER:
        igrasp->openConfig.at(0)= handProp.qmax.at(0);
      break;
      case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
        if(igrasp->config.size()!=13 || igrasp->openConfig.size()!=13)
        {
          printf("%s: %d: gpGrasp_compute_open_configs(): config vector has a bad size.\n",__FILE__,__LINE__);
          continue;
        }
        q.resize(13);
        qstart.resize(13);
        qstop.resize(13);
        blocked.resize(13);
        delta.resize(13);

        qstart= igrasp->config;
        qstop= handProp.qmin;

        for(i=0; i<blocked.size(); ++i)
        {   blocked[i]= false;     }

        qstop[0]= qstart[0]; // for thumb
        blocked[0]= true;

        //abduction joint
        for(i=0; i<4; ++i)
        {  blocked[3*i+1]= true;    }
      
        for(i=0; i<delta.size(); ++i)
        {   delta[i]= (qstop[i] - qstart[i])/((double)(nbSteps));      }

        q= qstart;

        gpSet_hand_configuration(robot, handProp, igrasp->config, false, 0);

        for(j=1; j<=nbSteps; ++j)
        {
          for(i=0; i<4; ++i)
          {
            qnew[0]= qstart[0]; //for thumb only
            if(!blocked[3*i+1])
            {  qnew[1]=  q[3*i+1] + delta[3*i+1]; }
            else
            {  qnew[1]=  q[3*i+1]; }

            if(!blocked[3*i+2])
            {  qnew[2]=  q[3*i+2] + delta[3*i+2]; }
            else
            {  qnew[2]=  q[3*i+2]; }

            if(!blocked[3*i+3])
            {  qnew[3]=  q[3*i+3] + delta[3*i+3]; }
            else
            {  qnew[3]=  q[3*i+3]; }

            if(blocked[3*i+1] && blocked[3*i+2] && blocked[3*i+3])
            {  continue; }

            gpSet_SAHfinger_joint_angles(robot, handProp, qnew, i+1); 
            if( p3d_col_test_robot_other(robot, object, 0) || p3d_col_test_self_collision(robot, 0) )
            { 
              blocked[3*i+1]= true;
              blocked[3*i+2]= true;
              blocked[3*i+3]= true;

              qnew[0]= qstart[0];
              qnew[1]= q[3*i+1];
              qnew[2]= q[3*i+2];
              qnew[3]= q[3*i+3];

              qnew[1]= 0.5*(qstart[3*i+1] + q[3*i+1]);
              qnew[2]= 0.5*(qstart[3*i+2] + q[3*i+2]);
              qnew[3]= 0.5*(qstart[3*i+3] + q[3*i+3]);
//               q[3*i+1]= qnew[1];
//               q[3*i+2]= qnew[2];
//               q[3*i+3]= qnew[3];

              gpSet_SAHfinger_joint_angles(robot, handProp, qnew, i+1);
            }
            else
            {
              q[3*i+1]= qnew[1];
              q[3*i+2]= qnew[2];
              q[3*i+3]= qnew[3];
            }
            for(k=0; k<blocked.size(); ++k)
            { 
              if(blocked[k]==false)
              {  break; }
            }
            if(k==blocked.size()) // all fingers are blocked
            {  break; }
          }
        }
        igrasp->openConfig= q;
//         gpSet_grasp_open_configuration(robot, handProp, *igrasp, 0);
// 
//         if(p3d_col_test_robot_other(robot, object, 0))
//         {
//             printf("problem collision %d\n",igrasp->ID);
//             pqp_print_colliding_pair();
//         }
      break;
      default:
          printf("%s: %d: gpCompute_grasp_open_config(): this hand model is not defined.\n",__FILE__,__LINE__);
          return GP_ERROR;
      break;
    }
  }

  p3d_set_and_update_this_robot_conf(robot, config0);

  return GP_OK;
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

//! @ingroup graspPlanning 
//! Computes a set of grasp frames by building a grid.
//! The number of computed frames depends on the given discretization steps and
//! on a threshold on the maximal number of frames.
//! The positions are computed in a grid with given resolution.and 
//! are then filtered to remove all the points that are outside the convex hull
//! of the polyhedron's points.
//! NB: if nbSamples is too big (in regard of a realistic memory allocation),
//! the function returns NULL and set nbSamples to a value !=0 whereas
//! if their is a memory allocation error, it returns NULL and sets nbSamples to 0.
//! \param polyhedron pointer to the p3d_polyhedre to sample point inside its convex hull
//! \param translationStep translation discretization step for hand/object pose sampling
//! \param nbDirections number of sampled directions for hand/object pose sampling
//! \param rotationStep rotation discretization step around each sampled direction for hand/object pose sampling
//! \param nbFramesMax maximal number of frames (the resolution will be adapted to reduce the number
//! of frames under this threshold)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSample_grasp_frames(p3d_polyhedre *polyhedron, unsigned int nbPositions, unsigned int nbDirections, unsigned int nbRotations, unsigned int nbFramesMax, std::vector<gpHTMatrix> &frames)
{
  #ifdef GP_DEBUG
   if(polyhedron==NULL)
   { 
     printf("%s: %d: gpSample_grasp_frames(): input p3d_polyhedre* is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  unsigned int i, j, id, it, count= 0;
  unsigned int nbSamples, nbPositionsMin, nbDirectionsMin, nbRotationsMin;
  int result;
  double translationStep, rotationStep;
  double xmin, xmax, ymin, ymax, zmin, zmax, dimX, dimY, dimZ;
  p3d_vector3 u, v, w;
  p3d_matrix3 R1, R2, R3;
  std::vector<gpVector3D> positions, directions;
  gpHTMatrix M;
  std::vector<gpHTMatrix> orientations;

  gpPolyhedron_AABB(polyhedron, xmin, xmax, ymin, ymax, zmin, zmax);

  dimX= 1.1*(xmax - xmin);
  dimY= 1.1*(ymax - ymin);
  dimZ= 1.1*(zmax - zmin);
//   printf("dim %f %f %f\n",dimX,dimY,dimZ);
//   result= gpSample_polyhedron_convex_hull(polyhedron, translationStep, positions);
  nbPositionsMin= 30;
  nbDirectionsMin= 6;
  nbRotationsMin= 3;

  nbSamples= (unsigned int) (nbPositions*nbDirections*nbRotations);

  while(nbSamples > nbFramesMax)
  {
    if(nbPositions > nbPositionsMin)
    {  nbPositions--;   }
    else if(nbRotations > nbRotationsMin)
    {  nbRotations--;   }
    else if(nbDirections > nbDirectionsMin)
    {  nbDirections--;   }
    else
    {  break; }
    nbSamples= (unsigned int) (2*nbPositions*nbDirections*nbRotations);
  }
  printf("nbPositions= %d nbDirections= %d nbRotations= %d \n", nbPositions, nbDirections, nbRotations);
  printf("nbSamples= %d nbFramesMax= %d \n", nbSamples, nbFramesMax);


  translationStep= pow((dimX*dimY*dimZ)/((double) (nbPositions)), (1.0/3.0));
  rotationStep= 2*M_PI/((double) (nbRotations));
//  printf("translationStep= %f %d %f\n",translationStep, nbPositions, ((double) (nbPositions)));
  result= gpSample_polyhedron_AABB(polyhedron, translationStep, positions);
//   printf("positions %d\n",positions.size());

  if(result==GP_ERROR)
  {
    printf("%s: %d: gpSample_grasp_frames(): position sampling error.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  result= gpSample_sphere_surface(1.0, nbDirections, directions);
  if(result==GP_ERROR)
  {
    printf("%s: %d: gpSample_grasp_frames(): problem with sphere surface sampling.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
   
  orientations.resize(nbRotations*directions.size());
  count= 0;
  for(id=0; id<directions.size(); ++id)
  {
    u[0]= directions[id].x;
    u[1]= directions[id].y;
    u[2]= directions[id].z;
    gpOrthonormal_basis(u, v, w);

    R1[0][0]= v[0];
    R1[1][0]= v[1];
    R1[2][0]= v[2];

    R1[0][1]= w[0];
    R1[1][1]= w[1];
    R1[2][1]= w[2];

    R1[0][2]= u[0];
    R1[1][2]= u[1];
    R1[2][2]= u[2];

    for(it=0; it<nbRotations; it++)
    {
      R2[0][0]= cos(it*rotationStep); R2[0][1]=-sin(it*rotationStep); R2[0][2]= 0;
      R2[1][0]= sin(it*rotationStep); R2[1][1]= cos(it*rotationStep); R2[1][2]= 0;
      R2[2][0]=                    0; R2[2][1]=                    0; R2[2][2]= 1;

      p3d_mat3Mult(R1, R2, R3);
      orientations.at(count).setRotation(R3);
      count++;
    }
  }

  frames.resize(positions.size()*orientations.size());

  count= 0;
  for(i=0; i<positions.size(); i++)
  {
    for(j=0; j<orientations.size(); j++)
    {
      frames.at(count)= orientations[j];

      frames.at(count).m14= positions[i].x;
      frames.at(count).m24= positions[i].y;
      frames.at(count).m34= positions[i].z;
      count++;
    }
  }


  return GP_OK;
}

//! @ingroup graspPlanning 
//! Generates a list of grasp for the given robot hand and object.
//! \param robot the hand robot (a freeflying robot composed of the hand/gripper bodies only)
//! \param object the object to be grasped
//! \param part the object part to grasp (all the object mesh triangles that have the same value in their "part" field). Set to 0 if unused (all the triangles will be considered).
//! \param hand structure containing information about the hand geometry
//! \param translationStep translation discretization step for hand/object pose sampling
//! \param nbDirections number of sampled directions for hand/object pose sampling
//! \param rotationStep rotation discretization step around each sampled direction for hand/object pose sampling
//! \param graspList the computed grasp list
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_generation(p3d_rob *robot, p3d_rob *object, int body_index, gpHand_properties &handProp, unsigned int nbPositions, unsigned int nbDirections, unsigned int nbRotations, std::list<class gpGrasp> &graspList)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGrasp_generation(): input robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpGrasp_generation(): input object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if( (body_index < 0) || (body_index > object->no-1) )
   {
     printf("%s: %d: gpGrasp_generation(): the index of selected body is not consistent with the number of bodies of the object.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  unsigned int i;
  unsigned int nbGraspFramesMax= handProp.max_nb_grasp_frames; //to avoid excessive computations if the input parameters were not properly chosen
  int result;
  std::vector<gpHTMatrix> gframes;
  p3d_matrix4 frame;
  p3d_polyhedre *polyhedron= NULL;
  std::list<gpGrasp>::iterator igrasp;
  std::list<gpContact> contactList;
  gpKdTree kdtree;

  polyhedron= object->o[body_index]->pol[0]->poly;

  p3d_compute_edges_and_face_neighbours(polyhedron);

  result= gpSample_grasp_frames(polyhedron, nbPositions, nbDirections, nbRotations, nbGraspFramesMax, gframes);

  if(result==GP_ERROR || gframes.empty())
  {
    printf("%s: %d: gpGrasp_generation(): grasp frames were not correctly generated.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  printf("Grasp computation for object \"%s\": %d grasp frames will be used.\n", object->name, gframes.size());

  switch(handProp.type)
  {
    case GP_GRIPPER:
      for(i=0; i<gframes.size(); i++)
      {
        gframes[i].copyIn_p3d_matrix4(frame);
        gpGrasps_from_grasp_frame_gripper(polyhedron, frame, handProp, graspList);
      }
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
     gpSample_obj_surface(object->o[body_index], 0.005, handProp.fingertip_radius, contactList);
     kdtree.build(contactList);
     printf("%d samples on object surface \n",contactList.size());
     for(i=0; i<gframes.size(); i++)
     {
       gframes[i].copyIn_p3d_matrix4(frame);
       gpGrasps_from_grasp_frame_SAHand(robot, object, body_index, frame, handProp, kdtree, graspList);
     }
    break;
    default:
      printf("%s: %d: gpGrasp_generation(): undefined hand type.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }


  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    if(igrasp->object==NULL)
    {
      igrasp->object= object;
      igrasp->body_index= body_index;
      igrasp->object_name= object->name;
    }
    igrasp->openConfig= igrasp->config;
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//!  Context independent collision test: removes from a grasp list all the grasps causing a collision between the robot hand and the grasped object.
//! \param graspList the original grasp list
//! \param robot the hand robot (a freeflying robot only composed of the hand/gripper bodies)
//! \param object the grasped object
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGrasp_collision_filter(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpGrasp_collision_filter(): object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_collision_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  bool collision;
  int body_index;
  p3d_matrix4 objectFrame;
  configPt q= p3d_alloc_config(robot);
  std::list<gpGrasp>::iterator igrasp;

  body_index= graspList.front().body_index;
  p3d_get_robot_config_into(robot, &q);

  gpDeactivate_object_fingertips_collisions(robot, object->o[body_index], hand);

  igrasp= graspList.begin();
  while(igrasp!=graspList.end())
  {
     p3d_get_body_pose(object, igrasp->body_index, objectFrame);

     collision= false;
     gpInverse_geometric_model_freeflying_hand(robot, objectFrame, igrasp->frame, hand, q);

     p3d_set_and_update_this_robot_conf(robot, q);

     gpSet_grasp_configuration(robot, hand, *igrasp);

     if( p3d_col_test_robot_other(robot, object, 0) ) //collision
     {
       igrasp= graspList.erase(igrasp);
       continue;
     }
     igrasp++;
  }


  p3d_destroy_config(robot, q);


  return GP_OK;
}

//! @ingroup graspPlanning 
//!  Context dependent collision test: removes from a grasp list all the grasps causing a collision between the robot hand and the environment.
//! \param graspList the original grasp list
//! \param robot the hand robot (a freeflying robot only composed of the hand/gripper bodies)
//! \param object the grasped object
//! \param hand structure containing information about the hand geometry
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_context_collision_filter(std::list<gpGrasp> &graspList, p3d_rob *robot, p3d_rob *object, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpGrasp_context_collision_filter(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_context_collision_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  p3d_matrix4 objectFrame;
  configPt q= p3d_alloc_config(robot);
  std::list<gpGrasp>::iterator igrasp;

//   p3d_get_obj_pos(object, objectFrame);
//   pqp_get_obj_pos(object, objectFrame);
//   p3d_get_first_joint_pose(object, objectFrame);

//   p3d_mat4Print(objectFrame, object->name);

  p3d_get_robot_config_into(robot, &q);

  igrasp= graspList.begin();
  while(igrasp!=graspList.end())
  {
     p3d_get_body_pose(object, igrasp->body_index, objectFrame);

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


  return GP_OK;
}

//! @ingroup graspPlanning 
//! Eliminates all the unstable grasps from a list and sorts the remaining list from the grasp with the biggest
//! stability score to the one with the smallest score.
//! \param graspList a list of grasps
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_stability_filter(std::list<gpGrasp> &graspList)
{
  #ifdef GP_DEBUG
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_stability_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  std::list<gpGrasp>::iterator igrasp;

  igrasp= graspList.begin();
  while(igrasp!=graspList.end())
  {
    igrasp->computeStability();
    if(igrasp->stability <= 0.0)
    {
       igrasp= graspList.erase(igrasp);
       continue;
    }
    igrasp++;
  }

  return GP_OK;
}

//! @ingroup graspPlanning 
//! Eliminates all the unstable grasps from a list and sorts the remaining list from the grasp with the biggest
//! stability score to the one with the smallest score.
//! \param graspList a list of grasps
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp_quality_filter(std::list<gpGrasp> &graspList)
{
  #ifdef GP_DEBUG
   if(graspList.empty())
   {
     printf("%s: %d: gpGrasp_quality_filter(): the grasp list is empty.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  double quality;
  std::list<gpGrasp>::iterator igrasp;

  igrasp= graspList.begin();
  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    quality= igrasp->computeQuality();
  }

  graspList.sort(); //sort from the smallest to the biggest stability
  graspList.reverse(); //reverse the order of the elements in the list

  return GP_OK;
}


//! @ingroup graspPlanning 
//! Computes the hand (wrist) pose corresponding to a given grasp frame.
//! \param robot pointer to the hand robot (its first joint must be a P3D_FREEFLYER)
//! \param objectFrame frame representing the object pose (in world frame)
//! \param graspFrame grasp frame (in object frame)
//! \param hand structure containing information about the hand geometry
//! \param q the computed hand configuration (must have been allocated before calling the function). Only the part corresponding to the hand pose is modified. The finger configurations are not modified.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpInverse_geometric_model_freeflying_hand(p3d_rob *robot, p3d_matrix4 objectFrame, p3d_matrix4 graspFrame, gpHand_properties &hand, configPt q)
{
   #ifdef GP_DEBUG
    if(robot==NULL)
    {
      printf("%s: %d: gpInverse_geometric_model_freeflying_hand(): input hand robot is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
    }
    if(q==NULL)
    {
      printf("%s: %d: gpInverse_geometric_model_freeflying_hand(): q is NULL.\n",__FILE__,__LINE__);
      return GP_ERROR;
    }
   if(robot->joints[1]->type!=P3D_FREEFLYER)
   {
      printf("%s: %d: gpInverse_geometric_model_freeflying_hand(): the first joint (\"%s\") of the hand robot (\"%s\") must  be of type P3D_FREEFLYER.\n",__FILE__,__LINE__,robot->joints[1]->name,robot->name);
      return GP_ERROR;
   }
   #endif

   p3d_matrix4 graspFrame_world, Twrist;

   p3d_get_robot_config_into(robot, &q);

   p3d_mat4Mult(objectFrame, graspFrame, graspFrame_world ); // object frame -> world frame

   p3d_mat4Mult(graspFrame_world, hand.Tgrasp_frame_hand, Twrist);

   p3d_mat4ExtractPosReverseOrder2(Twrist, &q[6], &q[7], &q[8], &q[9], &q[10], &q[11]);

   return GP_OK;
}

//! @ingroup graspPlanning 
//! Computes the forward kinematics model of the PA-10 arm for the robot's current configuration.
//! \param robot the robot (that must have joints with specific names (see graspPlanning.h))
//! \param Tend_eff the computed end effector pose matrix (in the world frame)
//! \param display if true, the frame of each body will be displayed
//! \return GP_OK in case of success, GP_ERROR otherwise
extern int gpForward_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, bool display)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpForward_geometric_model_PA10(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  float mat[16];
  p3d_matrix4 armBaseFrame, TH01, TH02, TH03, TH04, TH05, Tend_effb;;
  p3d_jnt *armJoint= NULL;
  Gb_q6 q;
  Gb_6rParameters arm_parameters;
  Gb_th thMGD, th01, th02, th03, th04, th05, R6RT, thMatPA10;
  Gb_dataMGD d;
  Gb_dep dep1;

  gpGet_arm_base_frame(robot, armBaseFrame);

  armJoint= p3d_get_robot_jnt_by_name(robot,  (char*)GP_ARMBASEJOINT);
  if(armJoint==NULL)
  {  return GP_ERROR; }
  q.q1= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  (char*)GP_ARMJOINT2);
  if(armJoint==NULL)
  {  return GP_ERROR; }
  q.q2= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  (char*)GP_ARMJOINT3);
  if(armJoint==NULL)
  {  return GP_ERROR; }
  q.q3= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  (char*)GP_ARMJOINT4);
  if(armJoint==NULL)
  {  return GP_ERROR; }
  q.q4= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  (char*)GP_ARMJOINT5);
  if(armJoint==NULL)
  {  return GP_ERROR; }
  q.q5= robot->ROBOT_POS[armJoint->index_dof];

  armJoint= p3d_get_robot_jnt_by_name(robot,  (char*)GP_WRISTJOINT);
  if(armJoint==NULL)
  {  return GP_ERROR; }
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
    g3d_draw_frame(TH01, 0.2);
    g3d_draw_frame(TH02, 0.2);
    g3d_draw_frame(TH03, 0.2);
    g3d_draw_frame(TH04, 0.2);
    g3d_draw_frame(TH05, 0.2);
    g3d_draw_frame(Tend_effb, 0.3);
   glPopMatrix();
 }

 return GP_OK;
}


//! @ingroup graspPlanning 
//! Computes the inverse kinematics of the PA-10 arm.
//! \param robot the robot (that must have joints with specific names (see graspPlanning.h))
//! \param Tend_eff the desired end effector pose matrix (given in the arm base frame)
//! \param q the solution joint parameter vector (that must be allocated before calling the function)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpInverse_geometric_model_PA10(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt q)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpInverse_geometric_model_PA10(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

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
        return GP_ERROR;
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
  if(gpSet_arm_configuration(robot, GP_PA10, qgoal.q1, qgoal.q2, qgoal.q3, qgoal.q4, qgoal.q5, qgoal.q6)==GP_OK)
  {
    p3d_get_robot_config_into(robot, &q);
    p3d_set_and_update_this_robot_conf(robot, q0);
    p3d_destroy_config(robot, q0);
    return GP_OK;
  }
  else
  {
    p3d_set_and_update_this_robot_conf(robot, q0);
    p3d_destroy_config(robot, q0);
    return GP_ERROR;
  }
}


#ifdef LIGHT_PLANNER
//! @ingroup graspPlanning 
int gpInverse_geometric_model(p3d_rob *robot, p3d_matrix4 Tend_eff, configPt q)
{
  if(robot==NULL)
  {
    printf("%s: %d: gpInverse_geometric_model(): robot is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  int result;
  configPt q0= NULL;

  q0= p3d_alloc_config(robot);
  p3d_get_robot_config_into(robot, &q0);

  if(p3d_set_virtual_object_pose(robot, Tend_eff)!=0)
  {
    p3d_destroy_config(robot, q0);
    return GP_ERROR;
  }

 // activateCcCntrts(robot, -1);
 // result= p3d_update_this_robot_pos(robot);
  p3d_get_robot_config_into(robot, &q);
// print_config(robot, q0);
// print_config(robot, q);
  deactivateCcCntrts(robot, -1);
  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);


  return 1;
  if(result==FALSE) 
  {
    return GP_ERROR;
  }
  else
  {
    return GP_OK;
  }

}
#endif

//! @ingroup graspPlanning 
//! Finds, for a given mobile base configuration of the robot, a grasp from the given grasp list, that is
//! reachable by the arm and hand, and computes for the grasp a grasping configuration for the whole robot.
//! \param robot the robot
//! \param object the object to grasp (a freeflyer robot)
//! \param graspList a list of grasps
//! \param arm_type the robot arm type
//! \param qbase a configuration of the robot (only the part corresponding to the mobile base will be used)
//! \param grasp a copy of the grasp that has been found, in case of success
//! \param hand parameters of the hand
//! \return a pointer to the computed grasping configuration of the whole robot in case of success, NULL otherwise
configPt gpFind_grasp_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &hand)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpFind_robot_config_from_grasp(): robot is NULL.\n",__FILE__,__LINE__);
     return NULL;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpFind_robot_config_from_grasp(): object is NULL.\n",__FILE__,__LINE__);
     return NULL;
   }
  #endif

  std::list<gpGrasp>::iterator igrasp;

  p3d_matrix4 object_frame, base_frame, inv_base_frame, gframe_object, gframe_world, gframe_robot, gframe_robot2;
  p3d_matrix4 T, Thand, Twrist;
  configPt q0= NULL; //pour mémoriser la configuration courante du robot
  configPt result= NULL;


  q0= p3d_get_robot_config(robot);

  XYZ_ENV->cur_robot= robot;

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
//gframe_object[0][3]+= 0.05;
//     p3d_get_obj_pos(object, object_frame);
//     p3d_get_first_joint_pose(object, object_frame);
    p3d_get_body_pose(object, igrasp->body_index, object_frame);
// p3d_mat4Print(object_frame, "object_frame");

    p3d_mat4Mult(object_frame, gframe_object, gframe_world ); //passage repère objet -> repère monde
    p3d_mat4Mult(gframe_world, hand.Tgrasp_frame_hand, Thand);
    p3d_mat4Mult(Thand, hand.Thand_wrist, Twrist);

    p3d_mat4Mult(inv_base_frame, gframe_world, gframe_robot); //passage repère monde -> repère robot
    p3d_mat4Mult(inv_base_frame, T, Twrist);

//     gpDeactivate_object_fingertips_collisions(robot, object, hand);
    switch(arm_type)
    {
      case GP_PA10:
        p3d_mat4Mult(gframe_robot, hand.Tgrasp_frame_hand, gframe_robot2);
        p3d_mat4Mult(gframe_robot2, hand.Thand_wrist, gframe_robot);
// p3d_mat4Print(gframe_robot, "gframe_robot");

        p3d_copy_config_into(robot, qbase, &result);

        if( gpInverse_geometric_model_PA10(robot, gframe_robot, result)==GP_OK )
//         if( gpInverse_geometric_model_PA10(robot, Twrist, result)==GP_OK )
       // if( gpInverse_geometric_model(robot, gframe_robot, result)==GP_OK )
        {
           #ifdef LIGHT_PLANNER
// 	   p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, result);
//            p3d_set_and_update_this_robot_conf(robot, result);
           #endif
           p3d_set_and_update_this_robot_conf(robot, result);
// printf("IK success\n");
           gpSet_grasp_configuration(robot, hand, *igrasp);

           if(!p3d_col_test()) //if no collision
          // if(!p3d_col_test_robot_statics(robot, 0) && !p3d_col_test_self_collision(robot, 0)) //if no collision
           { 
// print_config(robot, result);
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


//! @ingroup graspPlanning 
//! Finds, for a given mobile base configuration of the robot, a grasp from the given grasp list, that is
//! reachable by the arm and hand, and computes for the grasp a grasping configuration for the whole robot.
//! It also computes  an intermediate configuration (a configuration slightly before grasping the object)
//! \param robot the robot
//! \param object the object to grasp
//! \param graspList a list of grasps
//! \param arm_type the robot arm type
//! \param qbase a configuration of the robot (only the part corresponding to the mobile base will be used)
//! \param grasp a copy of the grasp that has been found, in case of success
//! \param hand parameters of the hand
//! \param distance distance between the grasp and pregrasp configurations (meters)
//! \param qpregrasp the pregrasp configuration (must have been allocated before)
//! \param qgrasp the grasp configuration (must have been allocated before)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFind_grasp_and_pregrasp_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpGrasp> &graspList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &hand, double distance, configPt qpregrasp, configPt qgrasp)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpFind_grasp_and_pregrasp_from_base_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpFind_grasp_and_pregrasp_from_base_configuration(): object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  std::list<gpGrasp>::iterator igrasp;

  p3d_matrix4 object_frame, base_frame, inv_base_frame, gframe_object1, gframe_object2;
  p3d_matrix4 gframe_world1, gframe_world2, gframe_robot1, gframe_robot2, tmp;
  configPt q0= NULL; //pour mémoriser la configuration courante du robot
  configPt result1= NULL, result2= NULL;


  q0= p3d_get_robot_config(robot);

  //On met à jour la configuration du robot pour que sa base soit dans la configuration
  //souhaitée:
  p3d_set_and_update_this_robot_conf(robot, qbase);
  result1= p3d_alloc_config(robot);
  result2= p3d_alloc_config(robot);


  gpGet_arm_base_frame(robot, base_frame); //on récupère le repère de la base du bras
  p3d_matInvertXform(base_frame, inv_base_frame);

  //pour chaque prise de la liste:
  for(igrasp=graspList.begin(); igrasp!=graspList.end(); igrasp++)
  {
    p3d_mat4Copy(igrasp->frame, gframe_object1); //for grasp config test
    p3d_mat4Copy(igrasp->frame, gframe_object2); //for pre-grasp config test
    gframe_object2[0][3]-= distance*gframe_object2[0][2];
    gframe_object2[1][3]-= distance*gframe_object2[1][2];
    gframe_object2[2][3]-= distance*gframe_object2[2][2];

    p3d_get_body_pose(object, igrasp->body_index, object_frame);

    p3d_mat4Mult(object_frame, gframe_object1, gframe_world1); //passage repère objet -> repère monde
    p3d_mat4Mult(object_frame, gframe_object2, gframe_world2); //passage repère objet -> repère monde

    p3d_mat4Mult(inv_base_frame, gframe_world1, gframe_robot1); //passage repère monde -> repère robot
    p3d_mat4Mult(inv_base_frame, gframe_world2, gframe_robot2); //passage repère monde -> repère robot

//     gpDeactivate_object_fingertips_collisions(robot, object, hand);
    switch(arm_type)
    {
      case GP_PA10:
        p3d_mat4Mult(gframe_robot1, hand.Tgrasp_frame_hand, tmp);
        p3d_mat4Mult(tmp, hand.Thand_wrist, gframe_robot1);

        p3d_mat4Mult(gframe_robot2, hand.Tgrasp_frame_hand, tmp);
        p3d_mat4Mult(tmp, hand.Thand_wrist, gframe_robot2);

        p3d_copy_config_into(robot, qbase, &result1);
        p3d_copy_config_into(robot, qbase, &result2);

        if( gpInverse_geometric_model_PA10(robot, gframe_robot1, result1)==GP_OK && gpInverse_geometric_model_PA10(robot, gframe_robot2, result2)==GP_OK )
        {
           #ifdef LIGHT_PLANNER
// 	   p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robot, result);
//            p3d_set_and_update_this_robot_conf(robot, result);
           #endif
           p3d_set_and_update_this_robot_conf(robot, result1);
           gpSet_grasp_configuration(robot, hand, *igrasp, 0);
//            if(p3d_col_test())
//            {  continue;  }
           gpOpen_hand(robot, hand);
//            if(p3d_col_test())
//            {  continue;  }

           p3d_set_and_update_this_robot_conf(robot, result2);
           gpOpen_hand(robot, hand);
            if(p3d_col_test())
            {  continue;  }

           igrasp->collision_state= COLLISION_FREE;
           grasp= *igrasp;

           p3d_set_and_update_this_robot_conf(robot, q0);
           p3d_destroy_config(robot, q0);

           p3d_copy_config_into(robot, result1, &qgrasp);
           p3d_copy_config_into(robot, result2, &qpregrasp);
           return GP_OK;
        }
      break;
      default:
          printf("%s: %d: gpFind_grasp_and_pregrasp_from_base_configuration(): undefined or unimplemented arm type.\n",__FILE__,__LINE__);
          p3d_set_and_update_this_robot_conf(robot, q0);
          p3d_destroy_config(robot, q0);
          return GP_ERROR;
      break;
    }

  }

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);

  return GP_ERROR;
}


//! @ingroup graspPlanning
//! Computes (or loads if it has been previously computed) a grasp list for a given object with the gripper 
//! The computed list will be saved.
//! NB: The world needs to have a robot corresponding to the chosen hand (see graspPlanning.h).
//! The grasps are tested for "internal" collisions (hand self collisions and hand vs object collisions ) and stability.
//! Collision against environment depends on the context and must be tested separately.
//! The grasp list file is searched for in a directory graspPlanning/graspList/"hand name"
//! inside the directory $HOME_MOVE3D. If it does not exist, it will be created by the function.
//! \param object_to_grasp the name of the object to grasp (a freeflyer robot)
//! \param graspList the computed grasp list
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_grasp_list_gripper(std::string object_to_grasp, std::list<gpGrasp> &graspList)
{
  float clock0, elapsedTime;
  gpHand_properties handProp;
  p3d_rob *hand_robot= NULL;
  p3d_rob *object= NULL;
  p3d_polyhedre *poly= NULL;
  std::string pathName, handFolderName, graspListFile, graspListFileOld;
  DIR *directory= NULL;
  std::list<gpGrasp>::iterator iter;

  if(getenv("HOME_MOVE3D")==NULL)
  {
    printf("%s: %d: gpGet_grasp_list_gripper(): the environment variable \"HOME_MOVE3D\" must have been defined .\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  handProp.initialize(GP_GRIPPER);
  hand_robot= p3d_get_robot_by_name((char*)GP_GRIPPER_ROBOT_NAME);
  if(hand_robot==NULL)
  {
    printf("%s: %d: gpGet_grasp_list_gripper(): a robot \"%s\" is required.\n",__FILE__,__LINE__,(char*)GP_GRIPPER_ROBOT_NAME);
    return GP_ERROR;
  }

  object= p3d_get_robot_by_name((char *)object_to_grasp.c_str());

  if(object==NULL)
  {
    printf("%s: %d: gpGet_grasp_list_gripper(): there is no robot (the object to grasp) named \"%s\".\n", __FILE__, __LINE__, object_to_grasp.c_str());
    return GP_ERROR;
  }

  poly= object->o[0]->pol[0]->poly;
  poly_build_planes(poly);



  pathName= std::string(getenv("HOME_MOVE3D")) + std::string("/graspPlanning/graspLists/");
  handFolderName= pathName + gpHand_type_to_folder_name(handProp.type);
  
  // look for a directory for the chosen hand:
  directory= opendir(handFolderName.c_str());
  if(directory==NULL)
  {
    // directory needs to be created:
    if(mkdir(handFolderName.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH)==-1)
    {
      printf("%s: %d: gpGet_grasp_list_gripper(): failed to create directory \"%s\".\n", __FILE__, __LINE__, handFolderName.c_str());
      return GP_ERROR;
    }
  }
  else
  { 
    closedir(directory);
  } 

  graspListFile= handFolderName  + std::string("/") + std::string(object_to_grasp) + std::string("Grasps.xml");
  graspListFileOld= handFolderName  + std::string("/") + std::string(object_to_grasp) + std::string("Grasps_old.xml");

  graspList.clear();

  if(gpLoad_grasp_list(graspListFile, graspList)==GP_ERROR) //grasp list needs to be computed
  {
    if(p3d_col_get_mode()!=p3d_col_mode_pqp)
    {
      printf("%s: %d: gpGet_grasp_list_gripper(): The collision detector must be PQP to use graspPlanning module.\n",__FILE__,__LINE__);
      printf("The graspPlanning module will not work.\n");
      return GP_ERROR;
    }

    clock0= clock();
    rename(graspListFile.c_str(), graspListFileOld.c_str()); //store the current grasp file (if it exists)

    gpGrasp_generation(hand_robot, object, 0, handProp, handProp.nb_positions, handProp.nb_directions, handProp.nb_rotations, graspList);

    gpGrasp_collision_filter(graspList, hand_robot, object, handProp);

    gpGrasp_stability_filter(graspList);

    gpGrasp_quality_filter(graspList);

//     gpGrasp_compute_open_configs(graspList, hand_robot, object, handProp);

    elapsedTime= (clock()-clock0)/CLOCKS_PER_SEC;
    printf("Computation time: %2.1fs= %dmin%ds, %d grasps computed\n",elapsedTime, (int)(elapsedTime/60.0), (int)(elapsedTime - 60*((int)(elapsedTime/60.0))), graspList.size() );

    gpSave_grasp_list(graspList, graspListFile);
  } 
  else
  {
    printf("%s: %d: gpGet_grasp_list_gripper(): file \"%s\" has been loaded successfully.\n", __FILE__, __LINE__,graspListFile.c_str()); 
  }


  return GP_OK;
}


//! @ingroup graspPlanning
//! Computes (or loads if it has been previously computed) a grasp list for a given object with the right 
//! or left SAHand. The computed list will be saved.
//! NB: The world needs to have a robot corresponding to the chosen hand (see graspPlanning.h).
//! The grasps are tested for "internal" collisions (hand self collisions and hand vs object collisions ) and stability.
//! Collision against environment depends on the context and must be tested separately.
//! The grasp list file is searched for in a directory graspPlanning/graspList/"hand name"
//! inside the directory $HOME_MOVE3D. If it does not exist, it will be created by the function.
//! \param object_to_grasp the name of the object to grasp (a freeflyer robot)
//! \param hand_to_use an integar defining the hand to use (1=right hand, 2= left hand)
//! \param graspList the computed grasp list
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGet_grasp_list_SAHand(std::string object_to_grasp, int hand_to_use, std::list<gpGrasp> &graspList)
{
  float clock0, elapsedTime;
  gpHand_properties handProp;
  p3d_rob *hand_robot= NULL;
  p3d_rob *object= NULL;
  p3d_polyhedre *poly= NULL;
  std::string pathName, handFolderName, graspListFile, graspListFileOld;
  DIR *directory= NULL;
  std::list<gpGrasp>::iterator iter;

  if(getenv("HOME_MOVE3D")==NULL)
  {
    printf("%s: %d: gpGet_grasp_list_SAHand(): the environment variable \"HOME_MOVE3D\" must have been defined .\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  switch(hand_to_use)
  {
    case 1:
       handProp.initialize(GP_SAHAND_RIGHT);
       hand_robot= p3d_get_robot_by_name((char*)GP_SAHAND_RIGHT_ROBOT_NAME);
       if(hand_robot==NULL)
       {
         printf("%s: %d: gpGet_grasp_list_SAHand(): a robot \"%s\" is required.\n",__FILE__,__LINE__,(char*)GP_SAHAND_RIGHT_ROBOT_NAME);
         return GP_ERROR;
       }
    break;
    case 2:
       handProp.initialize(GP_SAHAND_LEFT);
       hand_robot= p3d_get_robot_by_name((char*)GP_SAHAND_LEFT_ROBOT_NAME);
       if(hand_robot==NULL)
       {
         printf("%s: %d: gpGet_grasp_list_SAHand(): a robot \"%s\" is required.\n",__FILE__,__LINE__,(char*)GP_SAHAND_LEFT_ROBOT_NAME);
         return GP_ERROR;
       }
    break;
    default:
      printf("%s: %d: gpGet_grasp_list_SAHand(): \"hand_to_use\" parameter must be 1 (right hand) or 2 (left hand).\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  object= p3d_get_robot_by_name((char *)object_to_grasp.c_str());

  if(object==NULL)
  {
    printf("%s: %d: gpGet_grasp_list_SAHand(): there is no robot (the object to grasp) named \"%s\".\n", __FILE__, __LINE__, object_to_grasp.c_str());
    return GP_ERROR;
  }

  poly= object->o[0]->pol[0]->poly;
  poly_build_planes(poly);

  pathName= std::string(getenv("HOME_MOVE3D")) + std::string("/graspPlanning/graspLists/");
  handFolderName= pathName + gpHand_type_to_folder_name(handProp.type);
  
  // look for a directory for the chosen hand:
  directory= opendir(handFolderName.c_str());
  if(directory==NULL)
  {
    // directory needs to be created:
    if(mkdir(handFolderName.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH)==-1)
    {
      printf("%s: %d: gpGet_grasp_list_SAHand(): failed to create directory \"%s\".\n", __FILE__, __LINE__, handFolderName.c_str());
      return GP_ERROR;
    }
  }
  else
  { 
    closedir(directory);
  } 


  graspListFile= handFolderName  + std::string("/") + std::string(object_to_grasp) + std::string("Grasps.xml");
  graspListFileOld= handFolderName  + std::string("/") + std::string(object_to_grasp) + std::string("Grasps_old.xml");

  graspList.clear();

  if(gpLoad_grasp_list(graspListFile, graspList)==GP_ERROR) //grasp list needs to be computed
  {
    if(p3d_col_get_mode()!=p3d_col_mode_pqp)
    {
      printf("%s: %d: gpGet_grasp_list_SAHand(): The collision detector must be PQP to use compute a grasp list with the graspPlanning module.\n",__FILE__,__LINE__);
      printf("The graspPlanning module will not work.\n");
      return GP_ERROR;
    }

    clock0= clock();
    rename(graspListFile.c_str(), graspListFileOld.c_str()); //store the current grasp file (if it exists)


    gpGrasp_generation(hand_robot, object, 0, handProp, handProp.nb_positions, handProp.nb_directions, handProp.nb_rotations, graspList);
printf("before %d\n",graspList.size());
    gpGrasp_stability_filter(graspList);
printf("after %d\n",graspList.size());
    gpGrasp_quality_filter(graspList);
printf("after %d\n",graspList.size());

    gpGrasp_compute_open_configs(graspList, hand_robot, object, handProp);

    elapsedTime= (clock()-clock0)/CLOCKS_PER_SEC;

    printf("%d grasps were computed.\n",graspList.size());
    printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, (int)(elapsedTime/60.0), (int)(elapsedTime - 60*((int)(elapsedTime/60.0))) );

    gpSave_grasp_list(graspList, graspListFile);
  } 
  else
  {
    printf("%s: %d: gpGet_grasp_list_SAHand(): file \"%s\" has been loaded successfully.\n", __FILE__, __LINE__,graspListFile.c_str()); 
    printf("It contains %d grasps.\n",graspList.size());
  }

  for(iter=graspList.begin(); iter!=graspList.end(); iter++)
  {
    iter->handID= hand_to_use;
  }
 
  return GP_OK;
}



//! Computes grasps and adds them to an input grasp list.
//! The grasp list must not be empty because it is used to know some information
//! (what is the object to grasp, the hand to use, etc.).
//! The new grasp frames that will be tested are chosen at random.
//! \param robot the hand robot (a freeflying robot composed of the hand/gripper bodies only)
//! \param graspList a non-empty grasp list
//! \param nbTries the number of grasp frames that will be tested
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpExpand_grasp_list(p3d_rob *robot, std::list<class gpGrasp> &graspList, int nbTries)
{
  if(graspList.empty())
  {
    printf("%s: %d: gpExpand_grasp_list(): the input grasp list is empty. First computes a valid one.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  int i, body_index;
  double xmin, xmax, ymin, ymax, zmin, zmax;
  p3d_vector4 quat;
  p3d_matrix4 gframe;
  gpHand_type hand_type;
  gpHand_properties handProp;
  p3d_polyhedre *polyhedron= NULL;
  p3d_rob *object= NULL;
  std::list<gpGrasp> newGraspList;
  std::list<gpGrasp>::iterator igrasp;
  std::list<gpContact> contactList;
  gpKdTree kdtree;

  object= graspList.front().object;

  if(object==NULL)
  {
    printf("%s: %d: gpExpand_grasp_list(): the pointer to the object, contained in the input grasp list elements, is not valid .\n",__FILE__,__LINE__);
    return GP_ERROR; 
  }

  hand_type= graspList.front().hand_type;
 
  if(hand_type==GP_HAND_NONE)
  {
    printf("%s: %d: gpExpand_grasp_list(): the \"hand_type\" field, contained in the input grasp list elements, is not valid .\n",__FILE__,__LINE__);
    return GP_ERROR; 
  }

  body_index= graspList.front().body_index;
  if( (body_index < 0) || (body_index > object->no) )
  {
    printf("%s: %d: gpExpand_grasp_list(): the \"body_index\" field, contained in the input grasp list elements, is not valid .\n",__FILE__,__LINE__);
    return GP_ERROR; 
  }


  handProp.initialize(hand_type);

  polyhedron= object->o[body_index]->pol[0]->poly;

  gpPolyhedron_AABB(polyhedron, xmin, xmax, ymin, ymax, zmin, zmax);

  switch(handProp.type)
  {
    case GP_GRIPPER:
      for(i=0; i<nbTries; ++i)
      {
        p3d_random_quaternion(quat);
        p3d_quaternion_to_matrix4(quat, gframe);
        gframe[0][3]= p3d_random(xmin, xmax);
        gframe[1][3]= p3d_random(ymin, ymax);
        gframe[2][3]= p3d_random(zmin, zmax);

        gpGrasps_from_grasp_frame_gripper(polyhedron, gframe, handProp, newGraspList);
      }
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
     gpSample_obj_surface(object->o[body_index], 0.005, handProp.fingertip_radius, contactList);
     kdtree.build(contactList);
     printf("%d samples on object surface \n", contactList.size());
     for(i=0; i<nbTries; ++i)
     {
       p3d_random_quaternion(quat);
       p3d_quaternion_to_matrix4(quat, gframe);
       gframe[0][3]= p3d_random(xmin, xmax);
       gframe[1][3]= p3d_random(ymin, ymax);
       gframe[2][3]= p3d_random(zmin, zmax);

       gpGrasps_from_grasp_frame_SAHand(robot, object, body_index, gframe, handProp, kdtree, newGraspList);
     }
    break;
    default:
      printf("%s: %d: gpExpand_grasp_list(): undefined hand type.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  for(igrasp=newGraspList.begin(); igrasp!=newGraspList.end(); igrasp++)
  {
    if(igrasp->object==NULL)
    {
      igrasp->object= object;
      igrasp->body_index= body_index;
      igrasp->object_name= object->name;
    }
    igrasp->openConfig= igrasp->config;
  }

  switch(hand_type)
  {
    case GP_GRIPPER:
      gpGrasp_collision_filter(newGraspList, robot, object, handProp);
    break;
    default:
    break;
  }
    
  gpGrasp_stability_filter(newGraspList);

  gpGrasp_quality_filter(newGraspList);
  
  graspList.merge(newGraspList);

  return GP_OK;
}

//! Generates a list of double grasps from two lists of simple grasps.
//! \param robot1 pointer to the first robot hand
//! \param robot1 pointer to the second robot hand
//! \param robot1 pointer to the first robot hand
//! \param graspList1 previously computed grasp list for the first robot hand
//! \param graspList2 previously computed grasp list for the second robot hand
//! \param doubleGraspList the double grasp list that will be computed
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDouble_grasp_generation(p3d_rob *robot1, p3d_rob *robot2, p3d_rob *object, std::list<class gpGrasp> &graspList1, std::list<class gpGrasp> &graspList2, std::list<class gpDoubleGrasp> &doubleGraspList)
{
  #ifdef GP_DEBUG
  if(robot1==NULL)
  {
    printf("%s: %d: gpDouble_grasp_generation(): input robot 1 is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(robot2==NULL)
  {
    printf("%s: %d: gpDouble_grasp_generation(): input robot 2 is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(object==NULL)
  {
    printf("%s: %d: gpDouble_grasp_generation(): input object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  double distance;
  p3d_matrix4 objectPose1, objectPose2;
  configPt config1_0, config2_0;
  configPt config1, config2;
  gpHand_type handType1, handType2;
  gpHand_properties handProp1, handProp2;
  gpDoubleGrasp doubleGrasp;
  std::list<gpGrasp>::iterator iter1, iter2;

  config1_0= p3d_alloc_config(robot1);
  config2_0= p3d_alloc_config(robot2);
  p3d_get_robot_config_into(robot1, &config1_0);
  p3d_get_robot_config_into(robot2, &config2_0);

  config1= p3d_alloc_config(robot1);
  config2= p3d_alloc_config(robot2);


  handType1= graspList1.front().hand_type;
  handType2= graspList2.front().hand_type;
  handProp1.initialize(handType1);
  handProp2.initialize(handType2);

  doubleGraspList.clear();

  for(iter1=graspList1.begin(); iter1!=graspList1.end(); iter1++)
  {
    if(iter1->hand_type!=handType1)
    {
      printf("%s: %d: gpDouble_grasp_generation(): the initial simple grasps do not have the same hand type.\n",__FILE__,__LINE__);
      p3d_destroy_config(robot1, config1);
      p3d_destroy_config(robot2, config2);
      p3d_set_and_update_this_robot_conf(robot1, config1_0);
      p3d_set_and_update_this_robot_conf(robot2, config2_0);
      p3d_destroy_config(robot1, config1_0);
      p3d_destroy_config(robot2, config2_0);
      return GP_ERROR; 
    }
    p3d_get_body_pose(object, iter1->body_index, objectPose1);
    gpInverse_geometric_model_freeflying_hand(robot1, objectPose1, iter1->frame, handProp1, config1);
    p3d_set_and_update_this_robot_conf(robot1, config1);
    for(iter2=graspList2.begin(); iter2!=graspList2.end(); iter2++)
    {
      if(iter2->hand_type!=handType2)
      {
        printf("%s: %d: gpDouble_grasp_generation(): the initial simple grasps do not have the same hand type.\n",__FILE__,__LINE__);
        p3d_destroy_config(robot1, config1);
        p3d_destroy_config(robot2, config2);
        p3d_set_and_update_this_robot_conf(robot1, config1_0);
        p3d_set_and_update_this_robot_conf(robot2, config2_0);
        p3d_destroy_config(robot1, config1_0);
        p3d_destroy_config(robot2, config2_0);
        return GP_ERROR; 
      }

      p3d_get_body_pose(object, iter2->body_index, objectPose2);
      gpInverse_geometric_model_freeflying_hand(robot2, objectPose2, iter2->frame, handProp2, config2);
      p3d_set_and_update_this_robot_conf(robot2, config2);

      if(!p3d_col_test_robot_other(robot1, robot2, 0))
      {
        doubleGrasp.setFromSingleGrasps(*iter1, *iter2);
        doubleGrasp.distance= p3d_col_robot_robot_weighted_distance(robot1, robot2);
//         doubleGrasp.direction=
        doubleGrasp.computeDirection();

        doubleGraspList.push_back(doubleGrasp);
        doubleGraspList.back().ID= doubleGraspList.size();
//         p3d_mat4Distance(doubleGraspList.back().grasp1.frame, doubleGraspList.back().grasp2.frame, double weightR, double weightT)
      }
    }
  }

  doubleGraspList.sort();
  doubleGraspList.reverse();

  p3d_set_and_update_this_robot_conf(robot1, config1_0);
  p3d_set_and_update_this_robot_conf(robot2, config2_0);
  p3d_destroy_config(robot1, config1_0);
  p3d_destroy_config(robot2, config2_0);

  return GP_OK;
}


