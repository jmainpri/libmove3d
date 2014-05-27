/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
/**********************************************************************/
/*! 
  \file kcd_dist.c
  \brief  Functions concerning exact distance calculation
  
  This file contains a set of functions and static variables used to
  obtain the exact distances
  written by Pepijn Raeymaekers (KINEO) july 2001
*/
/* ********************************************************************/



/**********************************************************************
 * Functions concerning exact distance calculation
 * written by Pepijn Raeymaekers (KINEO)
 *********************************************************************/

#include "Util-pkg.h"
#include "Collision-pkg.h"


/* necessary variables */
extern double kcd_tolerance;


/* just for analysing */
static int counter_filter = 0;
static int count_test = 0;
static int gjk_calls = 0;
static int min_gjk = 0;
static int min_max_obb = 0;
/*static int match_min = 0;
static int match_max = 0;
static int max_not_min = 0;
static int total = 0;
*/


/* this list will contain the id_in_all_bbs of the AABB, for all
   AABB that intersects with the given mo
*/
static int *list_intersected_aabb = NULL;
void kcd_dist_init_list_aabb()
{
  list_intersected_aabb = NULL;
}

/**********************************************************************
 *                 CLOSEST POINTS                                     *
 **********************************************************************/

static kcd_vector3 closest_pointA;
static kcd_vector3 closest_pointB;

static double current_min_dist;
void kcd_dist_set_min_dist(double d)  
{
  current_min_dist=d;
}
double kcd_dist_get_min_dist()  
{
  return (current_min_dist);
}
void kcd_dist_init_min_dist()
{
  current_min_dist=P3D_HUGE; 
}

void kcd_dist_init_closest_points()
{
  closest_pointA[0] = 0.0;closest_pointA[1] = 0.0;closest_pointA[2] = 0.0;
  closest_pointB[0] = 0.0;closest_pointB[1] = 0.0;closest_pointB[2] = 0.0;
}
void kcd_dist_get_closest_points(kcd_vector3 pointA, kcd_vector3 pointB)
{
  pointA[0] = closest_pointA[0]; 
  pointA[1] = closest_pointA[1]; 
  pointA[2] = closest_pointA[2]; 
  pointB[0] = closest_pointB[0]; 
  pointB[1] = closest_pointB[1]; 
  pointB[2] = closest_pointB[2];  
}
void kcd_dist_set_closest_points(kcd_vector3 pointA, kcd_vector3 pointB)
{
  closest_pointA[0] = pointA[0];  
  closest_pointA[1] = pointA[1]; 
  closest_pointA[2] = pointA[2]; 
  closest_pointB[0] = pointB[0];  
  closest_pointB[1] = pointB[1]; 
  closest_pointB[2] = pointB[2];
}

/**********************************************************************
 *              EPSILON FILTERING                                     *                                 
 **********************************************************************/

/*! \var kcd_exact_epsilon
\brief this variable represents a percentage(and is always between 0 and 1), and is used to 
have a good estimation of the exact distances, and to accelerate the tests
If this variable hass a value different of zero then the returned closest points and
distances are not garantueed the exact ones but are a good approximation
*/
static double kcd_exact_epsilon = 0.0;

void kcd_set_kcd_exact_epsilon(double value)
{
  kcd_exact_epsilon = value;  
  //  PrintInfo(("KCD: kcd_exact_epsilon %lf\n", kcd_exact_epsilon));
}
double kcd_get_kcd_exact_epsilon()
{
  return (kcd_exact_epsilon);
}



/**********************************************************************
 *                       OBB                                          *
 **********************************************************************/
/**********************************************************************/
/*! \fn int kcd_gjk_between_2_OBB(kcd_bb *obbox1, kcd_bb *obbox2, 
                                  kcd_matrix4 *BwrtA, double *min_distance)
 \brief exact distance between two OBB's, used for min_max filtering
 when testintg two OBB-trees


 \param  *obbox1 pointers to OBB 1
 \param  *obbox2 pointers to OBB 2
 \param  BwrtA B with respect to A (placement matrix)
 \return min_distance, exact distance between two OBB's
 
 \note  BwrtA is used together with AwrtA, we will retrieve the 
        exact distance but the closest points will not be valid
*/
/*********************************************************************/
int kcd_gjk_between_2_OBB(kcd_bb *obbox1, kcd_bb *obbox2, 
			  kcd_matrix4 *BwrtA, double *min_distance)
{
  int in_collision = FALSE;
  kcd_vector3 zpa,zpb;
  kcd_matrix4 AwrtA =  {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  /* box 1 */
  int nof_vertices1=0;
  int *facet_ids1=NULL;
  int kcd_gjk_id1;
  void *polyh1=NULL; 
  /* box 2 */  
  int nof_vertices2=0;
  int *facet_ids2=NULL;
  int kcd_gjk_id2;
  void *polyh2=NULL;
  double gjk_tolerance = 0.0;
 

  /* we are going to simulate that the two OBBS are two BOXES */

  /* set the id's */
  kcd_gjk_id1 = obbox1->bb_id_in_all_bbs;
  kcd_gjk_id2 = obbox2->bb_id_in_all_bbs;
  /* set vertices a zero 
     nof_vertices1 = 0; 
     nof_vertices2 = 0;*/
  /* initialisation of the zpi's */
  zpa[0]=0.0;zpa[1]=0.0;zpa[2]=0.0;
  zpb[0]=0.0;zpb[1]=0.0;zpb[2]=0.0;
  /*set facet_ids to NULL 
    facet_ids1 = NULL; 
    facet_ids2 = NULL; */
  /* set polyh to NULL 
     polyh1 = NULL; 
     polyh2 = NULL;*/
  /* set tolerance to zero we want the exact distance */
  gjk_tolerance = 0.0;
  *min_distance = P3D_HUGE;
  
  /* p3d_mat4Print(AwrtA,"AwrtA");*/
  /* p3d_mat4Print(BwrtA,"BwrtA");*/
  in_collision = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
				      kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
				      AwrtA, *BwrtA, &zpa, &zpb, min_distance, gjk_tolerance);  
  /* return result */
  return (in_collision);
}  


/**********************************************************************/
/*! \fn void kcd_min_dist_est_between_2_OBB(kcd_bb *obbox1, kcd_bb *obbox2, 
  kcd_matrix4 BwrtA, double *min_distance)
  
  \brief Minimum distance estimate between two OBB (underestimation)
  \param *obbox pointers to the two OBB
  \param  BwrtA B with respect to A (placement matrix)
  \return minimum distance estimate ( >= 0 )
  \note is a better minimum estimate then obb_overlap but not always.
  Migth be usefull for min_max filtering.
  \note  not used
*/
/*********************************************************************/
void kcd_min_dist_est_between_2_OBB(kcd_bb *obbox1, kcd_bb *obbox2, 
				    kcd_matrix4 BwrtA, double *min_distance)
{
  double dist_estimate = 0.0;
  /* Minimum distance estimate is: 
   * the distance between the 2 centers
   * minus the distances from the centers till the corners
   */
  
  /* distance center <-> corner */
  dist_estimate -= sqrt((obbox1->d[0]*obbox1->d[0]) + (obbox1->d[1]*obbox1->d[1]) + (obbox1->d[2]*obbox1->d[2]));
  dist_estimate -= sqrt((obbox2->d[0]*obbox2->d[0]) + (obbox2->d[1]*obbox2->d[1]) + (obbox2->d[2]*obbox2->d[2]));
  /* distance between the centers */
  dist_estimate += sqrt((BwrtA[0][3]*BwrtA[0][3])   + (BwrtA[1][3]*BwrtA[1][3])   + (BwrtA[2][3]*BwrtA[2][3]));
  
  /* Arg OUT */
  if(dist_estimate < 0.0)
    {
      dist_estimate = 0.0;
    }
  *min_distance = dist_estimate;
}

/**********************************************************************/
/*! \fn void kcd_max_dist_est_between_2_OBB(kcd_bb *obbox1, kcd_bb *obbox2, 
  kcd_matrix4 BwrtA, double *max_distance)
  \brief Maximum distance estimate between two OBB (overestimation)
  \param *obbox pointers to the two OBB
  \param  BwrtA B with respect to A (placement matrix)
  \return max-distance,  maximum distance estimate 
  \note used for min max filtering, this function is slightly modified
  to filter more efficient, the maximum distance is actually the distance 
  between the two centers of the OBB
*/
/**********************************************************************/


void kcd_max_dist_est_between_2_OBB(kcd_bb *obbox1, kcd_bb *obbox2, 
				    kcd_matrix4 BwrtA, double *max_distance)
{
  double dist_estimate = 0.0;
  /* Minimum distance estimate is: 
   * the distance between the 2 centers
   * minus the distances from the centers till the corners
   */
  
  /* distance center <-> corner */
  /* dist_estimate += sqrt((obbox1->d[0]*obbox1->d[0]) + (obbox1->d[1]*obbox1->d[1]) + (obbox1->d[2]*obbox1->d[2]));
     dist_estimate += sqrt((obbox2->d[0]*obbox2->d[0]) + (obbox2->d[1]*obbox2->d[1]) + (obbox2->d[2]*obbox2->d[2]));*/
  /* distance between the centers */
  dist_estimate += sqrt((BwrtA[0][3]*BwrtA[0][3])   + (BwrtA[1][3]*BwrtA[1][3])   + (BwrtA[2][3]*BwrtA[2][3]));  
  *max_distance = dist_estimate;
}

/**********************************************************************/
/*! \fn int kcd_dist_exact_between_2_obbtrees(
  kcd_bb *rob_box, kcd_matrix4 invM2,kcd_matrix4 mat2, kcd_matrix4 a_box_pos, 
  kcd_bb *second_box, kcd_matrix4 *THISmat9, kcd_matrix4 *THISmat8,
  double *distance_exact, int *nearest_obstacle, int which_trees) 

  \brief calculate the exact distance and the correponding closest points 
  between the objects inside two given OBB-trees.

  \param *rob_box pointer to the BB around a movable object
  \param invM2 information position wrt joint
  \param mat2 information concerning position of joint
  \param a_box_pos could be second_box wrt W
  \param *second_box pointer to the BB, this migth be the BB around a static
  or a movable object
  \param *THISmat9  could be W wrt Rob_box
  \param *THISmat8 also a sort of second_box wrt W
  \param *distance_exact

  \return The exact distances and the corresponding closest points
  will be written in  the report
  \note The placement matrices contain different information depending 
  on the utilisation, the information inside could be what is written in 
  the explication of the parameters
  
  \todo remove the nearest_obstacle parameter
*/
/*********************************************************************/

int kcd_dist_exact_between_2_obbtrees(kcd_bb *rob_box, kcd_matrix4 invM2,kcd_matrix4 mat2, kcd_matrix4 a_box_pos, 
				      kcd_bb *second_box, kcd_matrix4 *THISmat9, kcd_matrix4 *THISmat8,
				      double *distance_estimate, int *nearest_obstacle, int which_trees) 
{
  int collision;
  double min_param, max_param;

  if(second_box == NULL)
    return FALSE;

  min_param = *distance_estimate;
  max_param = P3D_HUGE;

  counter_filter = 0;
  count_test = 0;
  gjk_calls = 0;
  min_gjk = 0;
  min_max_obb = 0;  

  switch (which_trees)
    {
    case GO_DOWN_ROBOT_TREE:
      {
	/* case if we only have to test the robot_kids */
	collision = kcd_visit_obbtree_robot(rob_box, invM2, mat2, second_box,
					    distance_estimate,&max_param, &min_param);     
	break;
      }
    case GO_DOWN_BOTH_TREES: 
      {
	/* we go down both_trees */
	collision = kcd_visit_2_obbtrees( rob_box, invM2, mat2, a_box_pos, second_box, THISmat9, THISmat8, 
					  distance_estimate,  &max_param, &min_param);
	break;
      }
    case GO_DOWN_2_ROBOT_TREES:
      {
	/* robot versus robot tree*/
	//PrintInfo(("we test\n"));
	collision = kcd_visit_2_robot_obbtrees(rob_box, invM2, mat2, a_box_pos, *THISmat9,
					       second_box, NULL,NULL,distance_estimate, &max_param, &min_param);
	
	break;
      }
    default:
      {
	PrintInfo(("KCD WARNING: collision test not valid\n"));
	collision = TRUE;
      }
    }

  if(distance_estimate)
    {
      *distance_estimate = min_param;  
    }
 
/*    PrintInfo(("We filtered %d times and tested %d OBB's and %d GJK calls,  collision ? %d with robottree %d\n",
      counter_filter,count_test,gjk_calls,collision,which_trees));  */
/*      PrintInfo(("we filtered min_max_obb %d, min_gjk %d\n", min_max_obb,min_gjk));*/

  return collision;
}
/**********************************************************************
 * kcd_visit_2_obbtrees
 * is a recursive function that will visit two given OBB trees
 * IN:
 * OUT: minimal max_dist
 *      minimal gjk_dist that is calculated if we arrive at GJK level
 * USE: obtain exact distances
 *********************************************************************/
int kcd_visit_2_obbtrees(kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat2, kcd_matrix4 a_box_pos, 
			 kcd_bb *second_box, kcd_matrix4 *THISmat9, kcd_matrix4 *THISmat8,
			 double *distance_estimate,
			 double *max_param, double *min_param)
{
  int collision_exists=FALSE;
  double the_distance_estimate = P3D_HUGE, kids_distance_estimate = P3D_HUGE, new_distance_estimate = P3D_HUGE;
  int i,j,nof_kids;
  kcd_matrix4 NEWinvM2, MOwrtSO;
  kcd_matrix4 M1,mat,mat9,mat8;
  kcd_bb *a_box = NULL;
  kcd_bb *a_bb = NULL;
  kcd_vector3 gjk_sep_axis ;
  kcd_vector3 *a_center = NULL;
  int volume_is_small,robot_volume_is_small,must_test_volume_for_robots = FALSE; 
  double the_max_estimate;
 

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  /* MAYBE DO THIS CALCUL LATER IF WE ARE SURE that we are going to test */
  robot_volume_is_small = must_test_volume_for_robots ? kcd_volume_smaller_than_user_defined_small_volume(rob_box->volume) : FALSE;
 
  if(THISmat9)
    {
      kcd_matMultTransfo(*THISmat9,a_box_pos,mat );
    }
  else if(THISmat8)
    {
      kcd_matMultTransfo(invM2,*THISmat8,mat );
    }
  else
    {
      kcd_matMultTransfo(invM2,mat2,mat9);
      kcd_matMultTransfo(mat9,a_box_pos,mat );
    }
  
 
  kcd_gjk_between_2_OBB(rob_box, second_box, &mat,&the_distance_estimate);    
  if(the_distance_estimate <= 0.0)
    collision_exists = TRUE;
  second_box->min_estimated_distance = the_distance_estimate; 
  kcd_max_dist_est_between_2_OBB(rob_box, second_box, mat,&the_max_estimate);     
  rob_box->max_estimated_distance = *max_param; 
  /* if new minimal max_distance we don't filter this solution  */
  if (*max_param > the_max_estimate)
    {
      *max_param = the_max_estimate; 
    }
//  else if( (!collision_exists) && ((filter_min_max_OBB(rob_box,second_box)) || filter_min_gjk_OBB(second_box,*min_param) )) 
  else if( (!collision_exists) && ((filter_min_max_OBB(rob_box,second_box)) || filter_min_gjk_OBB_with_eps(second_box,*min_param) )) 
    { /*       we filtered the solution so we don't have to continue  */
      counter_filter++;
      return FALSE;
    }
  count_test++;

  volume_is_small = kcd_volume_smaller_than_user_defined_small_volume(second_box->volume);
  nof_kids = second_box->nof_children_bb;
  if(  (rob_box->d[0] < second_box->d[0]) && (nof_kids > 0) && (!volume_is_small))
    {     
      /* recurse with kids of obstacle */
      collision_exists=FALSE;
      for(i=0;(i<nof_kids)&&(!collision_exists);i++)
	{
	  new_distance_estimate = P3D_HUGE;
	  a_bb = (second_box->children_bb)[i];
	      
	  /* change Place to matrix of eigenvalues of the child box */
	  M1[0][0] = a_bb->eigenv[0][0];	  M1[1][0] = a_bb->eigenv[0][1];
	  M1[2][0] = a_bb->eigenv[0][2];	  M1[0][3] = a_bb->center[0];
	  M1[0][1] = a_bb->eigenv[1][0];	  M1[1][1] = a_bb->eigenv[1][1];
	  M1[2][1] = a_bb->eigenv[1][2];	  M1[1][3] = a_bb->center[1];
	  M1[0][2] = a_bb->eigenv[2][0];	  M1[1][2] = a_bb->eigenv[2][1];
	  M1[2][2] = a_bb->eigenv[2][2];	  M1[2][3] = a_bb->center[2];
	  M1[3][0] = 0.0;	                  M1[3][1] = 0.0;
	  M1[3][2] = 0.0;	                  M1[3][3] = 1.0;
	      
	  /* only M1 is new => pass on mat3 */
	  if(THISmat9)
	    {
	      collision_exists = 
		kcd_visit_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,THISmat9,NULL,&new_distance_estimate,max_param,min_param);
	    }
	  else
	    {
	      kcd_matMultTransfo(invM2,mat2,mat9);
	      collision_exists = 
		kcd_visit_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,&mat9,NULL,&new_distance_estimate,max_param,min_param);
	    }
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;
	    }
	  /*done: recurse with kids of obstacle */
	}
    }
  else if((rob_box->nof_children_bb > 0) && (!robot_volume_is_small))
    {
      /* recurse with kids of robot link */      
      collision_exists=FALSE;
      for(j=0;(j<rob_box->nof_children_bb)&&(!collision_exists);j++) 
	{
	  new_distance_estimate = P3D_HUGE;
	  a_box = rob_box->children_bb[j];	      
	  NEWinvM2[0][0] = a_box->eigenv[0][0];	  NEWinvM2[0][1] = a_box->eigenv[0][1];
	  NEWinvM2[0][2] = a_box->eigenv[0][2];	  NEWinvM2[1][0] = a_box->eigenv[1][0];
	  NEWinvM2[1][1] = a_box->eigenv[1][1];	  NEWinvM2[1][2] = a_box->eigenv[1][2];
	  NEWinvM2[2][0] = a_box->eigenv[2][0];	  NEWinvM2[2][1] = a_box->eigenv[2][1];
	  NEWinvM2[2][2] = a_box->eigenv[2][2];	      
	  a_center = &(a_box->center);
	  NEWinvM2[0][3] = -NEWinvM2[0][0]*(*a_center)[0] -NEWinvM2[0][1]*(*a_center)[1] -NEWinvM2[0][2]*(*a_center)[2];
	  NEWinvM2[1][3] = -NEWinvM2[1][0]*(*a_center)[0] -NEWinvM2[1][1]*(*a_center)[1] -NEWinvM2[1][2]*(*a_center)[2];
	  NEWinvM2[2][3] = -NEWinvM2[2][0]*(*a_center)[0] -NEWinvM2[2][1]*(*a_center)[1] -NEWinvM2[2][2]*(*a_center)[2];	      
	  NEWinvM2[3][0] = 0.0;	  NEWinvM2[3][1] = 0.0;	  NEWinvM2[3][2] = 0.0;	  NEWinvM2[3][3] = 1.0;	      		  
	  /* only invM2 changed => do 1 multiplication here and pass on result */
	  if(THISmat8)
	    {
	      collision_exists = 
		kcd_visit_2_obbtrees(a_box,NEWinvM2,mat2,a_box_pos,second_box,NULL,THISmat8,&new_distance_estimate,max_param,min_param); 
	    }
	  else
	    {
	      /* compute mat8 */
	      kcd_matMultTransfo(mat2,a_box_pos,mat8 );
		  
	      collision_exists = 
		kcd_visit_2_obbtrees(a_box,NEWinvM2,mat2,a_box_pos,second_box,NULL,&mat8,&new_distance_estimate,max_param,min_param);    
	    }
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;
	    }
	}
    }
  else if( (nof_kids > 0) && (!volume_is_small) )
    {
      /* go down the kids of the obstacle anyway (since the robot doesn't have any other children */
      collision_exists = FALSE;     
      for(i=0 ; (i<nof_kids)&&(!collision_exists) ;i++)
	{
	  /* change Place to matrix of eigenvalues of the child box */
	  new_distance_estimate = P3D_HUGE;
	  a_bb = (second_box->children_bb)[i];

	  M1[0][0] = a_bb->eigenv[0][0];	  M1[1][0] = a_bb->eigenv[0][1];
	  M1[2][0] = a_bb->eigenv[0][2];	  M1[0][3] = a_bb->center[0];
	  M1[0][1] = a_bb->eigenv[1][0];	  M1[1][1] = a_bb->eigenv[1][1];
	  M1[2][1] = a_bb->eigenv[1][2];	  M1[1][3] = a_bb->center[1];
	  M1[0][2] = a_bb->eigenv[2][0];	  M1[1][2] = a_bb->eigenv[2][1];
	  M1[2][2] = a_bb->eigenv[2][2];	  M1[2][3] = a_bb->center[2];
	  M1[3][0] = 0.0;	  M1[3][1] = 0.0;	  M1[3][2] = 0.0;	  M1[3][3] = 1.0;
	      
	  /* only M1 is new => pass on mat3 */
	  if(THISmat9)
	    {
	      collision_exists = 
		kcd_visit_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,THISmat9,NULL,&new_distance_estimate,max_param,min_param);
	    }
	  else
	    {
	      /* compute mat9 */
	      kcd_matMultTransfo(invM2,mat2,mat9);
	      collision_exists = 
		kcd_visit_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,&mat9,NULL,&new_distance_estimate,max_param,min_param);
	    }
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;
	    }
	}
    }
  else
    {
      if((get_entity_type(rob_box->bb_id_in_all_bbs) == KCD_SMALL_VOLUME_BOX) || (get_entity_type(second_box->bb_id_in_all_bbs) == KCD_SMALL_VOLUME_BOX))
	{
	  /*modif 7 august for volume*/	  
	  kids_distance_estimate = *min_param;
	  kcd_matInvertTransfo(mat, MOwrtSO);
	  collision_exists = kcd_gjk_between_2_OBB_volume_boxes(rob_box, second_box, MOwrtSO,  &kids_distance_estimate);
	  if(kids_distance_estimate < *min_param)
	    {		   
	      *min_param =  kids_distance_estimate;		   
	    }
	}
      else if(robot_volume_is_small ||  volume_is_small)     
	{
	  //PrintInfo(("les volumes sont trop petites\n"));
	  kids_distance_estimate = *min_param;
	  kcd_matInvertTransfo(mat, MOwrtSO);
	  collision_exists = kcd_gjk_between_2_OBB_volume_boxes(rob_box, second_box, MOwrtSO,  &kids_distance_estimate);
	  if(kids_distance_estimate < *min_param)
	    {		   
	      *min_param =  kids_distance_estimate;		   
	    }
	}
      else
	{	 	  
	  gjk_sep_axis[0] = 0.0; gjk_sep_axis[1] = 0.0; gjk_sep_axis[2] = 0.0;	 
	  kids_distance_estimate = *min_param;		 
	  collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,DISTANCE_EXACT,&kids_distance_estimate,kcd_tolerance);	 	  	 	  
	  if(kids_distance_estimate < *min_param)
	    {     
	      *min_param =  kids_distance_estimate;	     
	    }	 		 
	}         
      gjk_calls++;     
    }

  /* fill in report anyway */
  if(collision_exists)
    {
      (*distance_estimate) = 0.0;
    }
  else
    {
      (*distance_estimate) = kids_distance_estimate;
    }

  return collision_exists;
}


/**********************************************************************
 * kcd_visit_obbtree_robot()
 * is a recursive function that will test the robottree vs objectOBB
 * IN:
 * OUT: minimal max_dist
 *      minimal gjk_dist that is calculated if we arrive at GJK level
 * USE: obtain exact distances
 *********************************************************************/
/* second_box is a leaf of an OBB-tree around an obstacle, rob_box is a OBB-(sub)tree around a robot link */
/* returns TRUE as soon as a collision is found between one of the leafs of rob_box and second_box */

int kcd_visit_obbtree_robot(kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat3,  
			    kcd_bb *second_box, double *distance_estimate, 
			    double *max_param, double *min_param)
{
  double the_distance_estimate = P3D_HUGE, kids_distance_estimate = P3D_HUGE, new_distance_estimate = P3D_HUGE;
  int collision_exists=FALSE;
  int j;
  kcd_matrix4 mat,NEWinvM2, MOwrtSO;
  kcd_bb *a_box = NULL;
  kcd_vector3 gjk_sep_axis;
  int volume_is_small, must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */
  double the_max_estimate;
#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  kcd_matMultTransfo(invM2,mat3,mat );
 
 // collision_exists = !( obb_disjoint_and_distance(mat, rob_box->d, second_box->d, &the_distance_estimate) );
  //  PrintInfo(("we tested the two boxes with overlap collision %d\n",collision_exists));
 kcd_gjk_between_2_OBB(rob_box, second_box, &mat,&the_distance_estimate);    
  if(the_distance_estimate <= 0.0)
    collision_exists = TRUE;

  /* filtering */
  second_box->min_estimated_distance = the_distance_estimate;
  kcd_max_dist_est_between_2_OBB(rob_box, second_box, mat,&the_max_estimate);     
  rob_box->max_estimated_distance = *max_param;
  /* if new minimal max_distance we don't filter this solution */
  if (*max_param > the_max_estimate)
    {     
      *max_param = the_max_estimate;    
    } 
  else if( (!collision_exists) ) 
    {  
      if(  ((filter_min_max_OBB(rob_box,second_box)) || filter_min_gjk_OBB_with_eps(second_box,*min_param) ) ) 
	{	 
	  counter_filter++;
	  return FALSE; 
	}
    } 
  count_test++;

  volume_is_small = (must_test_volume_for_robots) ? kcd_volume_smaller_than_user_defined_small_volume(rob_box->volume) : FALSE;
  if((rob_box->nof_children_bb > 0) && ( !volume_is_small) ) /* Modif. Carl on volume */
    {    
      collision_exists=FALSE;
      for(j=0;(j<rob_box->nof_children_bb)&&(!collision_exists);j++) 
	{
	  new_distance_estimate = P3D_HUGE;
	  a_box = rob_box->children_bb[j];

	  NEWinvM2[0][0] = a_box->eigenv[0][0];	  NEWinvM2[0][1] = a_box->eigenv[0][1];	  NEWinvM2[0][2] = a_box->eigenv[0][2];
	  NEWinvM2[1][0] = a_box->eigenv[1][0];	  NEWinvM2[1][1] = a_box->eigenv[1][1];	  NEWinvM2[1][2] = a_box->eigenv[1][2];
	  NEWinvM2[2][0] = a_box->eigenv[2][0];	  NEWinvM2[2][1] = a_box->eigenv[2][1];	  NEWinvM2[2][2] = a_box->eigenv[2][2];
	  NEWinvM2[0][3] = -NEWinvM2[0][0]*a_box->center[0] -NEWinvM2[0][1]*a_box->center[1] -NEWinvM2[0][2]*a_box->center[2];
	  NEWinvM2[1][3] = -NEWinvM2[1][0]*a_box->center[0] -NEWinvM2[1][1]*a_box->center[1] -NEWinvM2[1][2]*a_box->center[2];
	  NEWinvM2[2][3] = -NEWinvM2[2][0]*a_box->center[0] -NEWinvM2[2][1]*a_box->center[1] -NEWinvM2[2][2]*a_box->center[2];
	  NEWinvM2[3][0] = 0.0;   NEWinvM2[3][1] = 0.0;	  NEWinvM2[3][2] = 0.0;	  NEWinvM2[3][3] = 1.0;

	  /* only invM2 changed  */
	  collision_exists = kcd_visit_obbtree_robot(a_box,NEWinvM2,mat3,second_box,&new_distance_estimate,max_param,min_param);
		 
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;	 
	    }
	}
    }
  else
    {                 
      if((get_entity_type(rob_box->bb_id_in_all_bbs) == KCD_SMALL_VOLUME_BOX) || (get_entity_type(second_box->bb_id_in_all_bbs) == KCD_SMALL_VOLUME_BOX))
	{
	  kids_distance_estimate = *min_param;
	  kcd_matInvertTransfo(mat, MOwrtSO);
	  collision_exists = kcd_gjk_between_2_OBB_volume_boxes(rob_box, second_box, MOwrtSO,  &kids_distance_estimate);
	  //	  kcd_dist_get_closest_points(pa,pb);
	  //  PrintInfo(("collision exists %d :  %.0lf\n\n\n",  collision_exists, kids_distance_estimate));
	  if(kids_distance_estimate < *min_param)
	    {		   
	      *min_param =  kids_distance_estimate;		   
	    }
	}
      else if(volume_is_small)
	{	 
	  kids_distance_estimate = *min_param;
	  kcd_matInvertTransfo(mat, MOwrtSO);
	  collision_exists = kcd_gjk_between_2_OBB_volume_boxes(rob_box, second_box, MOwrtSO,  &kids_distance_estimate);	
	  //  PrintInfo(("collision exists %d :  %.0lf\n\n\n",  collision_exists, kids_distance_estimate));
	  if(kids_distance_estimate < *min_param)
	    {		   
	      *min_param =  kids_distance_estimate;		   
	    }
	}
      else
	{	 	  
	  gjk_sep_axis[0] = 0.0; gjk_sep_axis[1] = 0.0; gjk_sep_axis[2] = 0.0;	 
	  kids_distance_estimate = *min_param;	
	 
	  collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,DISTANCE_EXACT,&kids_distance_estimate,kcd_tolerance);	 	  	 	  
	  if(kids_distance_estimate < *min_param)
	    {     
	      *min_param =  kids_distance_estimate;	     
	    }	 		 
	}    
      
      gjk_calls++;
    }  	
  
  if(collision_exists)
    {
      (*distance_estimate) = 0.0;     
    }
  else
    {      
      (*distance_estimate) = kids_distance_estimate;   
    }     
  return collision_exists;
}



/* similar to collision_exists_between_2_obbtrees, but for 2 robots */
int kcd_visit_2_robot_obbtrees(kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat2, kcd_matrix4 mat1, 
			       kcd_matrix4 thisM1, kcd_bb *second_box, kcd_matrix4 *THISmat8, kcd_matrix4 *THISmat9,
			       double *distance_estimate, 
			       double *max_param, double *min_param) 
{
  int collision_exists=FALSE;
  double the_distance_estimate = P3D_HUGE, kids_distance_estimate = P3D_HUGE, new_distance_estimate = P3D_HUGE;
  int i,j,nof_kids;
  kcd_matrix4 NEWinvM2,M1,  MO1wrtMO2;
  kcd_matrix4 mat9,mat,mat8;
  kcd_bb *a_box = NULL;
  kcd_bb *a_bb = NULL;
  kcd_vector3 gjk_sep_axis ;
  kcd_vector3 *a_center = NULL;
  int must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */
  int volume_is_small; /* Modif. Carl on volume */
  double the_max_estimate;

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  if(THISmat8)
    {
      kcd_matMultTransfo(*THISmat8,thisM1,mat);
    }
  else if(THISmat9)
    {
      kcd_matMultTransfo(invM2,*THISmat9,mat8);
      kcd_matMultTransfo(mat8,thisM1,mat);      
    }
  else
    {
      kcd_matMultTransfo(mat2,mat1,mat9);
      kcd_matMultTransfo(invM2,mat9,mat8);
      kcd_matMultTransfo(mat8,thisM1,mat);
    }
  
  //  collision_exists = !( obb_disjoint_and_distance(mat, rob_box->d, second_box->d, &the_distance_estimate) );   
  kcd_gjk_between_2_OBB(rob_box, second_box, &mat,&the_distance_estimate);    
  if(the_distance_estimate <= 0.0)
    collision_exists = TRUE;

  
  //kcd_gjk_between_2_OBB(rob_box, second_box, &mat,&the_distance_estimate);    
  second_box->min_estimated_distance = the_distance_estimate;
  kcd_max_dist_est_between_2_OBB(rob_box, second_box, mat,&the_max_estimate);   
  // rob_box->max_estimated_distance = the_max_estimate;
  rob_box->max_estimated_distance = *max_param;
  /* if new minimal max_distance we don't filter this solution */
  if (*max_param > the_max_estimate)
    {
      *max_param = the_max_estimate;
      rob_box->max_estimated_distance = the_max_estimate;
    }
  else if( (!collision_exists) && ((filter_min_max_OBB(rob_box,second_box)) || filter_min_gjk_OBB_with_eps(second_box,*min_param) ))
    {
      /* we filtered the solution so we don't have to continue */
      counter_filter++;
      return FALSE;
    }
  count_test++;
 
  nof_kids = second_box->nof_children_bb;
  volume_is_small = (must_test_volume_for_robots ? kcd_volume_smaller_than_user_defined_small_volume( second_box->volume) : FALSE);
  if((rob_box->d[0] < second_box->d[0]) && (nof_kids > 0) && ( !volume_is_small ))
    {
      /* recurse with kids of second robot */
      collision_exists=FALSE;
      for(i=0;(i<nof_kids)&&(!collision_exists);i++)
	{
	  new_distance_estimate = P3D_HUGE;
	  /* change Place to matrix of eigenvalues of the child box */
	  a_bb = (second_box->children_bb)[i];

	  M1[0][0] = a_bb->eigenv[0][0];	  M1[1][0] = a_bb->eigenv[0][1];
	  M1[2][0] = a_bb->eigenv[0][2];	  M1[0][3] = a_bb->center[0];
	  M1[0][1] = a_bb->eigenv[1][0];	  M1[1][1] = a_bb->eigenv[1][1];
	  M1[2][1] = a_bb->eigenv[1][2];	  M1[1][3] = a_bb->center[1];
	  M1[0][2] = a_bb->eigenv[2][0];	  M1[1][2] = a_bb->eigenv[2][1];
	  M1[2][2] = a_bb->eigenv[2][2];	  M1[2][3] = a_bb->center[2];
	  M1[3][0] = 0.0; M1[3][1] = 0.0; M1[3][2] = 0.0; M1[3][3] = 1.0;
	  /* only M1 changed => pass on mat8 */
	  if(THISmat8)
	    {
	      /*		collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,THISmat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle)*/
	      collision_exists =    kcd_visit_2_robot_obbtrees(rob_box, invM2,mat2,mat1, 
							       M1, a_bb, THISmat8, NULL,
							       &new_distance_estimate,
							       max_param, min_param); 
	    }
	  else
	    {
	      /*	      collision_exists = 
			      collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,&mat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);*/
	      collision_exists =    kcd_visit_2_robot_obbtrees(rob_box, invM2,mat2,mat1, 
							       M1, a_bb, &mat8, NULL,
							       &new_distance_estimate, 
							       max_param, min_param); 
	    }
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;
//	      kids_nearest_obstacle = new_nearest_obstacle;
	    }
	}
    }
  else if( (rob_box->nof_children_bb > 0) && (must_test_volume_for_robots ? !kcd_volume_smaller_than_user_defined_small_volume( rob_box->volume) : TRUE) )
    {
      /* recurse with kids of robot link */
      collision_exists=FALSE;
      for(j=0;(j<rob_box->nof_children_bb)&&(!collision_exists);j++) 
	{
	  new_distance_estimate = P3D_HUGE;
	  a_box = rob_box->children_bb[j];

	  NEWinvM2[0][0] = a_box->eigenv[0][0];	  NEWinvM2[0][1] = a_box->eigenv[0][1];
	  NEWinvM2[0][2] = a_box->eigenv[0][2];	  NEWinvM2[1][0] = a_box->eigenv[1][0];
	  NEWinvM2[1][1] = a_box->eigenv[1][1];	  NEWinvM2[1][2] = a_box->eigenv[1][2];
	  NEWinvM2[2][0] = a_box->eigenv[2][0];	  NEWinvM2[2][1] = a_box->eigenv[2][1];
	  NEWinvM2[2][2] = a_box->eigenv[2][2];	  
   	  a_center = &(a_box->center);
	  NEWinvM2[0][3] = -NEWinvM2[0][0]*(*a_center)[0] -NEWinvM2[0][1]*(*a_center)[1] -NEWinvM2[0][2]*(*a_center)[2];
	  NEWinvM2[1][3] = -NEWinvM2[1][0]*(*a_center)[0] -NEWinvM2[1][1]*(*a_center)[1] -NEWinvM2[1][2]*(*a_center)[2];
	  NEWinvM2[2][3] = -NEWinvM2[2][0]*(*a_center)[0] -NEWinvM2[2][1]*(*a_center)[1] -NEWinvM2[2][2]*(*a_center)[2];
	      
	  NEWinvM2[3][0] = 0.0;	  NEWinvM2[3][1] = 0.0;	  NEWinvM2[3][2] = 0.0;	  NEWinvM2[3][3] = 1.0;
	      
	  /* only invM2 changed => pass on mat9 and thisM1 */
	  if(THISmat9)
	    {
	      /*      collision_exists = collision_exists_between_2_robot_obbtrees(a_box,NEWinvM2,mat2,mat1,thisM1,second_box,
		      NULL,THISmat9,with_report,&new_distance_estimate,&new_nearest_obstacle);*/
	      collision_exists =    kcd_visit_2_robot_obbtrees(a_box, NEWinvM2,mat2,mat1, 
							       thisM1, second_box, NULL,THISmat9,
							       &new_distance_estimate,
							       max_param, min_param); 
	    }
	  else
	    {
	      if(THISmat8)
		kcd_matMultTransfo(mat2,mat1,mat9);
	      /*	      collision_exists = collision_exists_between_2_robot_obbtrees(a_box,NEWinvM2,mat2,mat1,thisM1,second_box,
			      NULL,&mat9,with_report,&new_distance_estimate,&new_nearest_obstacle);*/
	      collision_exists =    kcd_visit_2_robot_obbtrees(a_box, NEWinvM2,mat2,mat1, 
							       thisM1, second_box, NULL,&mat9,
							       &new_distance_estimate, 
							       max_param, min_param); 
	    }
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;
//	      kids_nearest_obstacle = new_nearest_obstacle;
	    }		
	}
    }
  else if( (nof_kids > 0) && (must_test_volume_for_robots ?  !volume_is_small : TRUE) )
    {
      /* no kids in rob_box, but we can still go down in second_box */
      /* recurse with kids of second robot */
      collision_exists=FALSE;
      for(i=0;(i<nof_kids)&&(!collision_exists);i++)
	{
	  /* change Place to matrix of eigenvalues of the child box */
	  new_distance_estimate = P3D_HUGE;
	  a_bb = (second_box->children_bb)[i];
	  M1[0][0] = a_bb->eigenv[0][0];	  M1[1][0] = a_bb->eigenv[0][1];
	  M1[2][0] = a_bb->eigenv[0][2];	  M1[0][3] = a_bb->center[0];
	  M1[0][1] = a_bb->eigenv[1][0];	  M1[1][1] = a_bb->eigenv[1][1];
	  M1[2][1] = a_bb->eigenv[1][2];	  M1[1][3] = a_bb->center[1];
	  M1[0][2] = a_bb->eigenv[2][0];	  M1[1][2] = a_bb->eigenv[2][1];
	  M1[2][2] = a_bb->eigenv[2][2];	  M1[2][3] = a_bb->center[2];
	  M1[3][0] = 0.0;	  M1[3][1] = 0.0;	  M1[3][2] = 0.0;	  M1[3][3] = 1.0;
	  /* only M1 changed => pass on mat8 */
	  if(THISmat8)
	    {
	      /*	      collision_exists = 
			      collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,THISmat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);*/
	      collision_exists =    kcd_visit_2_robot_obbtrees(rob_box, invM2,mat2,mat1, 
							       M1, a_bb, THISmat8, NULL,
							       &new_distance_estimate, 
							       max_param, min_param); 
	    }
	  else
	    {
	      /*	      collision_exists = 
			      collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,&mat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);*/
	      collision_exists =    kcd_visit_2_robot_obbtrees(rob_box, invM2,mat2,mat1, 
							       M1, a_bb, &mat8, NULL,
							       &new_distance_estimate,
							       max_param, min_param); 
	    }
	  if(kids_distance_estimate > new_distance_estimate)
	    {
	      kids_distance_estimate = new_distance_estimate;

	    }
	}
    }
  else
    {
 
      if((get_entity_type(rob_box->bb_id_in_all_bbs) == KCD_SMALL_VOLUME_BOX) || (get_entity_type(second_box->bb_id_in_all_bbs) == KCD_SMALL_VOLUME_BOX))
	{  
	  kids_distance_estimate = *min_param;
	  kcd_matInvertTransfo(mat, MO1wrtMO2);
	  collision_exists = kcd_gjk_between_2_robot_OBB_volume_boxes(rob_box, second_box, MO1wrtMO2,mat1 , &kids_distance_estimate);
	  //collision_exists = kcd_gjk_between_2_robot_OBB_volume_boxes(second_box, rob_box,  MO1wrtMO2,mat1 , &kids_distance_estimate);

	  if(kids_distance_estimate < *min_param)
	    {		   
	      *min_param =  kids_distance_estimate;		   
	    }
	}
      else if( (must_test_volume_for_robots ? ((volume_is_small) || (kcd_volume_smaller_than_user_defined_small_volume( rob_box->volume))) : FALSE) )
	{
	  //PrintInfo(("descente dans 2 robottrees\n"));
	  kids_distance_estimate = *min_param;
	  kcd_matInvertTransfo(mat, MO1wrtMO2);
	  collision_exists = kcd_gjk_between_2_robot_OBB_volume_boxes(rob_box, second_box, MO1wrtMO2,mat1 , &kids_distance_estimate);
	  //collision_exists = kcd_gjk_between_2_robot_OBB_volume_boxes(second_box, rob_box,  MO1wrtMO2,mat1 , &kids_distance_estimate);	  
	  if(kids_distance_estimate < *min_param)
	    {		   
	      *min_param =  kids_distance_estimate;		   
	    } 	    
	}
      else
	{
	  gjk_sep_axis[0] = 0.0; gjk_sep_axis[1] = 0.0; gjk_sep_axis[2] = 0.0;	 
	  kids_distance_estimate = *min_param;		  
	  kcd_dist_set_min_dist(*min_param); 
	  //collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,DISTANCE_EXACT,&kids_distance_estimate,kcd_tolerance);	 	  	 	  
	  /* sinon les points sont inversees */
	  collision_exists = kcd_gjk_intersect(second_box,rob_box,&gjk_sep_axis,DISTANCE_EXACT,&kids_distance_estimate,kcd_tolerance);	 	  	 	  

	  if(kids_distance_estimate < *min_param)
	    {
 	      *min_param =  kids_distance_estimate;
	    }
	 
	}     
      gjk_calls++;
    }
 
  if(distance_estimate)
    {
      if(collision_exists)
	{
	  (*distance_estimate) = 0.0;
	}
      else
	{
	  (*distance_estimate) = kids_distance_estimate;
	}
    }
    	    
  return collision_exists;
}













/**********************************************************************
 * Min_max filtering
 * IN: TWO BB and if max_dist of BB1 < min_dist of BB2 then
 *     TRUE is returned
 * OUT: returns if BB2 can be filtered or not
 * USE: to avoid testing some trees
 *********************************************************************/
int filter_min_max_OBB(kcd_bb *bb1,kcd_bb *bb2)
{
  /* if(bb1->max_estimated_distance < bb2->min_estimated_distance)
     min_max_obb++;*/
  return(bb1->max_estimated_distance < bb2->min_estimated_distance);
}
int filter_min_gjk_OBB(kcd_bb *obb, double min_gjk_dist)
{
  /*  if (min_gjk_dist < obb->min_estimated_distance )
      min_gjk++;*/
  return(min_gjk_dist < obb->min_estimated_distance );
}

int filter_min_gjk_OBB_with_eps(kcd_bb *obb, double min_gjk_dist)
{
  if( kcd_exact_epsilon == 0.0 )
    {
      return(min_gjk_dist < obb->min_estimated_distance );
    }
  else if (min_gjk_dist == P3D_HUGE)
    {
      return FALSE;
    }
  else
    {
      return (( min_gjk_dist - (min_gjk_dist * kcd_exact_epsilon)) < obb->min_estimated_distance );
    }
}


int filter_min_max_OBB_2(double max, double min)
{
  return(max<min);
}
int filter_min_gjk_OBB_2(double min_obb, double min_gjk_dist)
{
  /* if the distance between two OBB's is bigger then an already
   *  calculated gjk distance then we can filter this obb */
  return(min_gjk_dist < min_obb );
}


/**********************************************************************
 *                      AABB                                          *
 **********************************************************************/

/**********************************************************************
 * Maximum distance between two AABB's
 * IN: pointers to the two AABB
 * OUT : maximum distance between two AABB even if they intersect

 * USE: min_max filtering to obtain exact distances
 *********************************************************************/
double aabbs_maximum_distance(int b1_nr, int b2_nr)
{
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];
  double lx=P3D_HUGE;
  double ly=P3D_HUGE;
  double lz=P3D_HUGE;
  double x_12 = ABS(aabb1->x1 - aabb2->x2);
  double x_21 = ABS(aabb1->x2 - aabb2->x1); 
  double y_12 = ABS(aabb1->y1 - aabb2->y2);
  double y_21 = ABS(aabb1->y2 - aabb2->y1);
  double z_12 = ABS(aabb1->z1 - aabb2->z2);
  double z_21 = ABS(aabb1->z2 - aabb2->z1);

  lx = MAX(x_12,x_21);
  ly = MAX(y_12,y_21);
  lz = MAX(z_12,z_21);
 
  return (sqrt( lx*lx + ly*ly + lz*lz ));
}
/**********************************************************************
 * Maximum SQUARE distance between two AABB's
 * IN: pointers to the two AABB
 * OUT : maximum square distance between two AABB even if they intersect
 * USE: min_max filtering to obtain exact distances
 *********************************************************************/
double aabbs_maximum_square_distance(int b1_nr, int b2_nr)
{
  kcd_bb *aabb1 = all_bbs[b1_nr];
  kcd_bb *aabb2 = all_bbs[b2_nr];
  double lx=P3D_HUGE;
  double ly=P3D_HUGE;
  double lz=P3D_HUGE;
  double x_12 = ABS(aabb1->x1 - aabb2->x2);
  double x_21 = ABS(aabb1->x2 - aabb2->x1); 
  double y_12 = ABS(aabb1->y1 - aabb2->y2);
  double y_21 = ABS(aabb1->y2 - aabb2->y1);
  double z_12 = ABS(aabb1->z1 - aabb2->z2);
  double z_21 = ABS(aabb1->z2 - aabb2->z1);

  lx = MAX(x_12,x_21);
  ly = MAX(y_12,y_21);
  lz = MAX(z_12,z_21);
  
  return ( (lx*lx) + (ly*ly) + (lz*lz) );
}


/**********************************************************************
 *                           API                                      *
 **********************************************************************/



/*\fn int kcd_dist_mo_in_collision(int mobj_id,int bb_id_of_body, double *min_distance)
\brief test a given mo against the environment to obtain the exact distances
\param mobj_id movable object id
\param bb_id_of_body id in all_bbs
   */
int kcd_dist_mo_in_collision(int mobj_id,int bb_id_of_body, double *min_distance)
{
  int collision = FALSE;
  int nof_el_level,level;
  double smallest_max_dist = P3D_HUGE;  
  double smallest_min_dist = P3D_HUGE;
  double gjk_min_dist;
  int gjk_nearest_obstacle;
  int i, current_bb_id;
  double current_square_min_dist, current_dist;
  kcd_vector3 pa,pb;
  int aabb_filtered;
  int closest_i = 0,closest_min=0; // closest_max;
  int total_gjk_calls = 0; 
  int nr_list=0;
  /* B Kineo Carl 27.02.2002 */
  int ext_obj_id;
  /* E Kineo Carl 27.02.2002 */

  /* this test is done to avoid testing when there are no static objects
   * test comes too late
   */
  if(kcd_get_total_height() == 1)
    return FALSE;

  kcd_dist_init_closest_points(); 
 
  level = 0;
  nof_el_level = kcd_get_nofels_on_level(level);
 
  if(list_intersected_aabb == NULL)
    {
      list_intersected_aabb = MY_ALLOC(int, nof_el_level);
    }

  /* run through the lowest level of the AABB tree and filter */
  for(i=0; i<nof_el_level; i++)
    { 
      current_bb_id = AABB_TREE[level][i].the_bb; 
      /* B modif. Kineo Carl 27.02.2002 */
      ext_obj_id = all_bbs[current_bb_id]->ext_obj_id;
      if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE)
	{
	  /* static object inside AABB is to be taken into account (not deactivated) */
	  all_bbs[current_bb_id]->min_estimated_distance = aabbs_square_distance(bb_id_of_body,current_bb_id); 
	  all_bbs[current_bb_id]->max_estimated_distance = aabbs_maximum_square_distance( bb_id_of_body,current_bb_id);    
	  if(all_bbs[current_bb_id]->max_estimated_distance < smallest_max_dist) 
	    {
	      smallest_max_dist = all_bbs[current_bb_id]->max_estimated_distance; 
	    }       
	  if(all_bbs[current_bb_id]->min_estimated_distance == 0.0)	    
	  {
	     list_intersected_aabb[nr_list] = i;
	     nr_list++;
	   }
	  else if(all_bbs[current_bb_id]->min_estimated_distance < smallest_min_dist) 
	    {
	      smallest_min_dist = all_bbs[current_bb_id]->min_estimated_distance; 	  
	      closest_i = i;	  
	    }  
	}
      /* E modif. Kineo Carl 27.02.2002 - intro. du if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE) */
    }
  /* closest AABB with distance not zero */
  closest_min = closest_i;


  /* necessary initialisations before descend in the OBB-tree */
  kcd_dist_init_min_dist();
  current_square_min_dist = P3D_HUGE;
  current_dist = P3D_HUGE;
  gjk_min_dist = P3D_HUGE;
 
  
  /* test all the AABB that are in collision */
  for( i = 0; (i< nr_list) && (!collision); i++)
    {
      current_bb_id = AABB_TREE[level][list_intersected_aabb[i]].the_bb;
      collision = kcd_dist_test_robot_vs_static_obbs(mobj_id, all_bbs[ bb_id_of_body]->x1,all_bbs[ bb_id_of_body]->x2,
						     all_bbs[ bb_id_of_body]->y1,all_bbs[ bb_id_of_body]->y2, 
						     all_bbs[ bb_id_of_body]->z1,all_bbs[ bb_id_of_body]->z2,	
						     DISTANCE_EXACT,&gjk_min_dist ,&gjk_nearest_obstacle,current_bb_id);      	       
      if(current_dist > gjk_min_dist)
	{		    
	  kcd_dist_get_closest_points(pa,pb);	     
	  current_dist = gjk_min_dist;	  
	 // current_square_min_dist  = (gjk_min_dist * gjk_min_dist);
	  closest_i = list_intersected_aabb[i];	  
	}    
      total_gjk_calls += gjk_calls; 
      all_bbs[current_bb_id]->min_estimated_distance = P3D_HUGE; 
    }
  /* save the closest square min_dist */
  current_square_min_dist  = (current_dist * current_dist);


  /* filter out possible candidates */
  for(i=0; i<nof_el_level; i++) 
    { 
      current_bb_id = AABB_TREE[level][i].the_bb;        
      if(all_bbs[current_bb_id]->min_estimated_distance > smallest_max_dist)
	{	
	  all_bbs[current_bb_id]->min_estimated_distance = P3D_HUGE; 	
	} 
    }
  
  aabb_filtered = 0;

  /* start with the closest one  */
  current_bb_id = AABB_TREE[level][closest_min].the_bb;
  if((all_bbs[current_bb_id]->min_estimated_distance < current_square_min_dist) && (!collision))
    {      
      collision = kcd_dist_test_robot_vs_static_obbs(mobj_id, all_bbs[ bb_id_of_body]->x1,all_bbs[ bb_id_of_body]->x2,
						     all_bbs[ bb_id_of_body]->y1,all_bbs[ bb_id_of_body]->y2, 
						     all_bbs[ bb_id_of_body]->z1,all_bbs[ bb_id_of_body]->z2,	
						     DISTANCE_EXACT,&gjk_min_dist ,&gjk_nearest_obstacle,current_bb_id);      	       
      if(current_dist > gjk_min_dist)
	{		    
	  kcd_dist_get_closest_points(pa,pb);	     
	  current_dist = gjk_min_dist;	  
	  current_square_min_dist  = (gjk_min_dist * gjk_min_dist);
	  closest_i = closest_min;
	}
      else
	closest_i =0;

      aabb_filtered++;	  
      total_gjk_calls += gjk_calls; 
      /* to avoid testing it again */
      all_bbs[current_bb_id]->min_estimated_distance = P3D_HUGE;       
    }
  
  /* test all other possible candidates */
  for(i=0; (i<nof_el_level && (!collision)); i++)
    {           
      current_bb_id = AABB_TREE[level][i].the_bb;
      /* B modif Jim hri Mainprice 25.10.2008 */
      ext_obj_id = all_bbs[current_bb_id]->ext_obj_id;
      if(kcd_get_active_kcd_o_prm_table(ext_obj_id) == TRUE)
      	{
		      if(all_bbs[current_bb_id]->min_estimated_distance < current_square_min_dist )
			{	  	  	 
			  collision = kcd_dist_test_robot_vs_static_obbs(mobj_id, all_bbs[ bb_id_of_body]->x1,all_bbs[ bb_id_of_body]->x2,
									 all_bbs[ bb_id_of_body]->y1,all_bbs[ bb_id_of_body]->y2, 
									 all_bbs[ bb_id_of_body]->z1,all_bbs[ bb_id_of_body]->z2,	
									 DISTANCE_EXACT,&gjk_min_dist ,&gjk_nearest_obstacle,current_bb_id);      	   
			  if(current_dist > gjk_min_dist)
			    {		    	  
			      kcd_dist_get_closest_points(pa,pb);	     
			      current_dist = gjk_min_dist;	  
			      current_square_min_dist  = (gjk_min_dist * gjk_min_dist);
			      closest_i = i;
			  
			    }	 
			  aabb_filtered++;	  
			  total_gjk_calls += gjk_calls;
			}
      	}

    }    

  /* this warning appears if GJK isn't called */
  if((!collision) && (total_gjk_calls == 0))
    {
      /* This warning should not appear */
      PrintInfo(("KCD WARNING: DISTANCE IS WRONG FOR mo %d\n",mobj_id));
    }
 
  /* for the report mecanism */
  *min_distance = current_min_dist;  
  kcd_dist_set_closest_points(pa,pb); 

  return collision;
}

/* USE: testing in depth */

int kcd_dist_test_robot_vs_static_obbs(int mobj_id, double xmin, double xmax, double ymin, 
				       double ymax, double zmin, double zmax, 
				       int with_report, double *minimal_distance_estimate, int *nearest_obstacle, int current_bb_id)
{
  int no_clash_found,the_nearest_obstacle;
  double a[4],center_a[3],the_distance_estimate=P3D_HUGE;

  /* AABB around movable object: */
  a[0] = xmax - xmin;
  a[1] = ymax - ymin;
  a[2] = zmax - zmin;
  /* a[3] contains largest value of a[0], a[1], a[2] */
  if(a[0] > a[1])
    if(a[0] > a[2])
      a[3] = a[0];
    else 
      a[3] = a[2];
  else
    if(a[1] > a[2])
      a[3] = a[1];
    else
      a[3] = a[2];

  center_a[0] = (xmax + xmin)/2.0;
  center_a[1] = (ymax + ymin)/2.0;
  center_a[2] = (zmax + zmin)/2.0;

  no_clash_found = TRUE;
  /* look at each of the AABBs in collision with this link */
  the_distance_estimate=P3D_HUGE;
  no_clash_found = check_robot_vs_obbs_of_this_aabb(mobj_id,a,center_a,
						    all_bbs[current_bb_id],with_report,&the_distance_estimate,&the_nearest_obstacle);
  if(*minimal_distance_estimate > the_distance_estimate) 
    {
      *minimal_distance_estimate = the_distance_estimate;
    }
 
  return !no_clash_found;
}



/**********************************************************************/
/*!\fn int kcd_gjk_between_2_OBB_volume_boxes(kcd_bb *obbox1, kcd_bb *obbox2, 
	   			       kcd_matrix4 MOwrtSO,
				       double *min_distance)
 \brief exact distance between two OBB's and the corresponding closest points,
 this function is called to test two volume boxes against eachother


 \param  *obbox1 pointer to OBB 1 (Movable object)
 \param  *obbox2 pointer to OBB 2 (Static object)
 \param  MOwrtSO movable object with respect to static object     (placement matrix)
 \return min_distance, exact distance between two OBB's

 \warning the placement matrix for the static object will be retrieved in the 
 kcd_bb *obbox2 
 \note  The closest points (in kcd_dist.c) will be filled in and will
 be valid, these are the points filled in the report afterwards
*/
/*********************************************************************/
int kcd_gjk_between_2_OBB_volume_boxes(kcd_bb *obbox1, kcd_bb *obbox2, 
				       kcd_matrix4 MOwrtSO,
				       double *min_distance)
{
  int in_collision = FALSE;
  kcd_vector3 zpa,zpb,tzpa,tzpb;
  /* box 1 */
  int nof_vertices1=0;
  int *facet_ids1=NULL;
  int kcd_gjk_id1;
  void *polyh1=NULL; 
  /* box 2 */  
  int nof_vertices2=0;
  int *facet_ids2=NULL;
  int kcd_gjk_id2;
  void *polyh2=NULL;
  /*tolerance has to be taken into account*/
  double gjk_tolerance = kcd_tolerance;
  kcd_matrix4 MOwrtW, SOwrtW;

  *min_distance = P3D_HUGE;

  SOwrtW[0][0] = obbox2->eigenv[0][0];	  SOwrtW[1][0] = obbox2->eigenv[0][1];
  SOwrtW[2][0] = obbox2->eigenv[0][2];	  SOwrtW[0][3] = obbox2->center[0];
  SOwrtW[0][1] = obbox2->eigenv[1][0];	  SOwrtW[1][1] = obbox2->eigenv[1][1];
  SOwrtW[2][1] = obbox2->eigenv[1][2];	  SOwrtW[1][3] = obbox2->center[1];
  SOwrtW[0][2] = obbox2->eigenv[2][0];	  SOwrtW[1][2] = obbox2->eigenv[2][1];
  SOwrtW[2][2] = obbox2->eigenv[2][2];	  SOwrtW[2][3] = obbox2->center[2];
  SOwrtW[3][0] = 0.0;	  SOwrtW[3][1] = 0.0;	  SOwrtW[3][2] = 0.0;	  SOwrtW[3][3] = 1.0;

  kcd_matMultTransfo( SOwrtW, MOwrtSO, MOwrtW);  

  zpa[0] = 0.0; zpa[1] = 0.0; zpa[2] = 0.0;			  				
  kcd_gjk_id1 = obbox1->bb_id_in_all_bbs;

  zpb[0] = 0.0;zpb[1] = 0.0;zpb[2] = 0.0;		
  kcd_gjk_id2 = obbox2->bb_id_in_all_bbs; 

  
  in_collision = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
				      kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
				      MOwrtW, SOwrtW, &zpa, &zpb, min_distance, gjk_tolerance); 

  if(*min_distance <=  kcd_dist_get_min_dist())
    {  
      //   PrintInfo(("%.0lf\n", *min_distance));
      kcd_dist_set_min_dist(*min_distance); 
      kcd_TransfoPoint(SOwrtW, zpb, tzpb);  
      kcd_TransfoPoint(MOwrtW, zpa, tzpa);   
      kcd_dist_set_closest_points(tzpa,tzpb);
    }

  /* return result */
  return (in_collision);
}  



	
/**********************************************************************/
/*!\fn int kcd_gjk_between_2_robot_OBB_volume_boxes(kcd_bb *obbox1, kcd_bb *obbox2, 
	   			       kcd_matrix4 MO1wrtMO2, kcd_matrix4 MO2wrtW,
				       double *min_distance)
 \brief exact distance between two OBB's and the corresponding closest points,
 this function is called to test two volume boxes against eachother.
 This function is specificly for testing two OBB's around moving objects


 \param  *obbox1 pointer to OBB 1 (Movable object 1)
 \param  *obbox2 pointer to OBB 2 (Movable object 2)
 \param  MO1wrtMO2 movable object with respect to static object(placement matrix)
 \param  MO2wrtW movable object wrt to Workspace(placement matrix)
 \return min_distance, exact distance between two OBB's

 \note  The closest points (in kcd_dist.c) will be filled in and will
 be valid, these are the points filled in the report afterwards
*/
/*********************************************************************/
int kcd_gjk_between_2_robot_OBB_volume_boxes(kcd_bb *obbox1, kcd_bb *obbox2, 
					     kcd_matrix4 MO1wrtMO2, kcd_matrix4 MO2wrtW,
					     double *min_distance)
{
  int in_collision = FALSE;
  kcd_vector3 zpa,zpb,tzpa,tzpb;
  /* box 1 */
  int nof_vertices1=0;
  int *facet_ids1=NULL;
  int kcd_gjk_id1;
  void *polyh1=NULL; 
  /* box 2 */  
  int nof_vertices2=0;
  int *facet_ids2=NULL;
  int kcd_gjk_id2;
  void *polyh2=NULL;
  /*tolerance has to be taken into account*/
  double gjk_tolerance = kcd_tolerance;
  kcd_matrix4 MO1wrtW;

/*  char c;

  p3d_mat4Print(MO1wrtMO2,"mo1wrtmo2");
  p3d_mat4Print(MO2wrtW,"mo2wrtW");
  scanf("%c",&c);*/

 // kcd_matMultTransfo( MO1wrtMO2, MO2wrtW, MO1wrtW);  
  kcd_matMultTransfo( MO2wrtW, MO1wrtMO2, MO1wrtW);  

  zpa[0] = 0.0; zpa[1] = 0.0; zpa[2] = 0.0;			  				
  kcd_gjk_id1 = obbox1->bb_id_in_all_bbs;

  zpb[0] = 0.0;zpb[1] = 0.0;zpb[2] = 0.0;		
  kcd_gjk_id2 = obbox2->bb_id_in_all_bbs; 

  
  in_collision = kcd_gjk_closest_pair(kcd_gjk_id1,nof_vertices1,facet_ids1,polyh1,
				      kcd_gjk_id2,nof_vertices2,facet_ids2,polyh2,
				      MO1wrtW, MO2wrtW, &zpa, &zpb, min_distance, gjk_tolerance); 

  kcd_dist_set_min_dist(*min_distance); 

//  kcd_TransfoPoint(MO2wrtW, zpa, tzpa);  
//  kcd_TransfoPoint(MO1wrtW, zpb, tzpb);

  /*original*/
  kcd_TransfoPoint(MO2wrtW, zpb, tzpb);  
  kcd_TransfoPoint(MO1wrtW, zpa, tzpa);
  /* points are changed */
  kcd_dist_set_closest_points(tzpb,tzpa);


  /* return result */
    return (in_collision);
}  



/* **********************************************************************************************************/
/* INIT AND CLEAN UP */
/* **********************************************************************************************************/


/*\fn void kcd_init_distance_exact()
\brief functions that need to be called in the initialisation phase of KCD
They initalise the internal structures for exact distance calculation
\note These are completly separated from the report functions
*/
void kcd_init_distance_exact()
{
  kcd_dist_init_min_dist();
  kcd_dist_init_closest_points();
  kcd_dist_init_list_aabb();
}
/*\fn void  kcd_clean_up_dist()
\brief clean up of the internal structures
*/
void  kcd_clean_up_dist()
{
  if(list_intersected_aabb != NULL)
    {     
      MY_FREE(list_intersected_aabb, int,  kcd_get_nofels_on_level(0));
    }
}
