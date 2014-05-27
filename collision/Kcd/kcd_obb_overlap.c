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
#include "Collision-pkg.h"

int DO_KCD_GJK=TRUE;  /* is TRUE when user want GJK to be executed, FALSE otherwise */

#define rfabs(x) ((x < 0) ? -x : x)  
const double r_eps = 1e-12;

double kcd_tolerance = 0.0;

void set_DO_KCD_GJK(int val)
{
  DO_KCD_GJK = val;
}

void kcd_assign_tolerance(double val)
{
  kcd_tolerance = val;
}

void kcd_get_tolerance(double *tol)
{
  *tol = kcd_tolerance;
}

int kcd_tolerance_is_zero()
{
  return (kcd_tolerance == 0.0);
}

/* params: p3d_matrix4 Place  (   orientation of box b   w.r.t. box a
                              and translation of box b   w.r.t. box a),
           double a[3]        (size of box a), 
	   double b[3]        (size of box b).  
   out: returns distance in kcd_obb_distance in case of disjointness */
int obb_disjoint_and_distance(kcd_matrix4 Place, double a[3], double b[3], double *kcd_obb_distance)
{
  register double val1, val;
  register int res;
  double fabs_of_Place[3][3];

  /* fabs_of_Place = fabs(Place), compute when needed */
  fabs_of_Place[0][0] = rfabs(Place[0][0]);  fabs_of_Place[0][0] += r_eps;
  fabs_of_Place[0][1] = rfabs(Place[0][1]);  fabs_of_Place[0][1] += r_eps;
  fabs_of_Place[0][2] = rfabs(Place[0][2]);  fabs_of_Place[0][2] += r_eps;

  /* if any of these tests are one-sided, then the polyhedra are disjoint */
  res = 1;

  /* A1 x A2 = A0 */
  val1 = rfabs(Place[0][3]) - (a[0] + b[0] * fabs_of_Place[0][0] + b[1] * fabs_of_Place[0][1] + b[2] * fabs_of_Place[0][2]);
  res &= (val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 1;
    }
  
  fabs_of_Place[1][0] = rfabs(Place[1][0]);  fabs_of_Place[1][0] += r_eps;
  fabs_of_Place[2][0] = rfabs(Place[2][0]);  fabs_of_Place[2][0] += r_eps;

  /* B1 x B2 = B0*/
  val = Place[0][3]*Place[0][0] + Place[1][3]*Place[1][0] + Place[2][3]*Place[2][0];
  val1 = rfabs(val) - (b[0] + a[0] * fabs_of_Place[0][0] + a[1] * fabs_of_Place[1][0] + a[2] * fabs_of_Place[2][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 2;
    }
    
  fabs_of_Place[1][1] = rfabs(Place[1][1]);  fabs_of_Place[1][1] += r_eps;
  fabs_of_Place[1][2] = rfabs(Place[1][2]);  fabs_of_Place[1][2] += r_eps;

  /* A2 x A0 = A1 */
  val1 = rfabs(Place[1][3]) - (a[1] + b[0] * fabs_of_Place[1][0] + b[1] * fabs_of_Place[1][1] + b[2] * fabs_of_Place[1][2]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 3;
    }

  fabs_of_Place[2][1] = rfabs(Place[2][1]);  fabs_of_Place[2][1] += r_eps;
  fabs_of_Place[2][2] = rfabs(Place[2][2]);  fabs_of_Place[2][2] += r_eps;

  /* A0 x A1 = A2 */
  val1 = rfabs(Place[2][3]) - (a[2] + b[0] * fabs_of_Place[2][0] + b[1] * fabs_of_Place[2][1] + b[2] * fabs_of_Place[2][2]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 4;
    }

  /* B2 x B0 = B1 */
  val = Place[0][3]*Place[0][1] + Place[1][3]*Place[1][1] + Place[2][3]*Place[2][1];
  val1 = rfabs(val) - (b[1] + a[0] * fabs_of_Place[0][1] + a[1] * fabs_of_Place[1][1] + a[2] * fabs_of_Place[2][1]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 5;
    }

  /* B0 x B1 = B2 */
  val = Place[0][3]*Place[0][2] + Place[1][3]*Place[1][2] + Place[2][3]*Place[2][2];
  val1 = rfabs(val) - (b[2] + a[0] * fabs_of_Place[0][2] + a[1] * fabs_of_Place[1][2] + a[2] * fabs_of_Place[2][2]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 6;
    }

  /* A0 x B0 */
  val = Place[2][3] * Place[1][0] - Place[1][3] * Place[2][0];
  val1 = rfabs(val) - (a[1] * fabs_of_Place[2][0] + a[2] * fabs_of_Place[1][0] +
		       b[1] * fabs_of_Place[0][2] + b[2] * fabs_of_Place[0][1]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 7;
    }
  
  /* A0 x B1 */
  val = Place[2][3] * Place[1][1] - Place[1][3] * Place[2][1];
  val1 = rfabs(val) - (a[1] * fabs_of_Place[2][1] + a[2] * fabs_of_Place[1][1] +
		       b[0] * fabs_of_Place[0][2] + b[2] * fabs_of_Place[0][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 8;
    }

  /* A0 x B2 */
  val = Place[2][3] * Place[1][2] - Place[1][3] * Place[2][2];
  val1 = rfabs(val) - (a[1] * fabs_of_Place[2][2] + a[2] * fabs_of_Place[1][2] +
		       b[0] * fabs_of_Place[0][1] + b[1] * fabs_of_Place[0][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 9;
    }

  /* A1 x B0 */
  val = Place[0][3] * Place[2][0] - Place[2][3] * Place[0][0];
  val1 = rfabs(val) - (a[0] * fabs_of_Place[2][0] + a[2] * fabs_of_Place[0][0] +
		       b[1] * fabs_of_Place[1][2] + b[2] * fabs_of_Place[1][1]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 10;
    }

  /* A1 x B1 */
  val = Place[0][3] * Place[2][1] - Place[2][3] * Place[0][1];
  val1 = rfabs(val) - (a[0] * fabs_of_Place[2][1] + a[2] * fabs_of_Place[0][1] +
		       b[0] * fabs_of_Place[1][2] + b[2] * fabs_of_Place[1][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 11;
    }

  /* A1 x B2 */
  val = Place[0][3] * Place[2][2] - Place[2][3] * Place[0][2];
  val1 = rfabs(val) - (a[0] * fabs_of_Place[2][2] + a[2] * fabs_of_Place[0][2] +
		       b[0] * fabs_of_Place[1][1] + b[1] * fabs_of_Place[1][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 12;
    }

  /* A2 x B0 */
  val = Place[1][3] * Place[0][0] - Place[0][3] * Place[1][0];
  val1 = rfabs(val) - (a[0] * fabs_of_Place[1][0] + a[1] * fabs_of_Place[0][0] +
		       b[1] * fabs_of_Place[2][2] + b[2] * fabs_of_Place[2][1]);
  res &= (val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 13;
    }

  /* A2 x B1 */
  val = Place[1][3] * Place[0][1] - Place[0][3] * Place[1][1];
  val1 = rfabs(val) - (a[0] * fabs_of_Place[1][1] + a[1] * fabs_of_Place[0][1] +
		       b[0] * fabs_of_Place[2][2] + b[2] * fabs_of_Place[2][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 14;
    }

  /* A2 x B2 */
  val = Place[1][3] * Place[0][2] - Place[0][3] * Place[1][2];
  val1 = rfabs(val) - (a[0] * fabs_of_Place[1][2] + a[1] * fabs_of_Place[0][2] +
	   b[0] * fabs_of_Place[2][1] + b[1] * fabs_of_Place[2][0]);
  res &= ( val1 <= kcd_tolerance );
  if (!res) 
    {
      if(kcd_obb_distance != NULL) *kcd_obb_distance = val1;
      return 15;
    }

  if(kcd_obb_distance != NULL) *kcd_obb_distance = 0.0;
  return 0;   /* OBBs are not disjoint, they collide */
}


/* params: p3d_matrix4 Place  (   orientation of box b   w.r.t. box a
                              and translation of box b   w.r.t. box a),
           double a[3]        (size of box a), 
	   double b[3]        (size of box b).  */
int obb_disjoint(kcd_matrix4 Place, double a[3], double b[3])
{
  register double val1, val;
  register int res;
  double fabs_of_Place[3][3];

  fabs_of_Place[0][0] = rfabs(Place[0][0]);  fabs_of_Place[0][0] += r_eps;
  fabs_of_Place[0][1] = rfabs(Place[0][1]);  fabs_of_Place[0][1] += r_eps;
  fabs_of_Place[0][2] = rfabs(Place[0][2]);  fabs_of_Place[0][2] += r_eps;

  
  /* if any of these tests are one-sided, then the polyhedra are disjoint */
  res = 1;

  /* A1 x A2 = A0 */
  val1 = rfabs(Place[0][3]);
  res &= (val1 <= 
	  (a[0] + b[0] * fabs_of_Place[0][0] + b[1] * fabs_of_Place[0][1] + b[2] * fabs_of_Place[0][2]));
  if (!res) return 1;
  
  fabs_of_Place[1][0] = rfabs(Place[1][0]);  fabs_of_Place[1][0] += r_eps;
  fabs_of_Place[2][0] = rfabs(Place[2][0]);  fabs_of_Place[2][0] += r_eps;

  /* B1 x B2 = B0 */
  val = Place[0][3]*Place[0][0] + Place[1][3]*Place[1][0] + Place[2][3]*Place[2][0];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (b[0] + a[0] * fabs_of_Place[0][0] + a[1] * fabs_of_Place[1][0] + a[2] * fabs_of_Place[2][0]));
  if (!res) return 2;
    
  fabs_of_Place[1][1] = rfabs(Place[1][1]);  fabs_of_Place[1][1] += r_eps;
  fabs_of_Place[1][2] = rfabs(Place[1][2]);  fabs_of_Place[1][2] += r_eps;

  /* A2 x A0 = A1 */
  val1 = rfabs(Place[1][3]);
  res &= ( val1 <= 
	  (a[1] + b[0] * fabs_of_Place[1][0] + b[1] * fabs_of_Place[1][1] + b[2] * fabs_of_Place[1][2]));
  if (!res) return 3;

  fabs_of_Place[2][1] = rfabs(Place[2][1]);  fabs_of_Place[2][1] += r_eps;
  fabs_of_Place[2][2] = rfabs(Place[2][2]);  fabs_of_Place[2][2] += r_eps;

  /* A0 x A1 = A2 */
  val1 = rfabs(Place[2][3]);

  res &= ( val1 <= 
	  (a[2] + b[0] * fabs_of_Place[2][0] + b[1] * fabs_of_Place[2][1] + b[2] * fabs_of_Place[2][2]));
  if (!res) return 4;

  /* B2 x B0 = B1 */
  val = Place[0][3]*Place[0][1] + Place[1][3]*Place[1][1] + Place[2][3]*Place[2][1];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (b[1] + a[0] * fabs_of_Place[0][1] + a[1] * fabs_of_Place[1][1] + a[2] * fabs_of_Place[2][1]));
  if (!res) return 5;

  /* B0 x B1 = B2 */
  val = Place[0][3]*Place[0][2] + Place[1][3]*Place[1][2] + Place[2][3]*Place[2][2];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (b[2] + a[0] * fabs_of_Place[0][2] + a[1] * fabs_of_Place[1][2] + a[2] * fabs_of_Place[2][2]));
  if (!res) return 6;

  /* A0 x B0 */
  val = Place[2][3] * Place[1][0] - Place[1][3] * Place[2][0];
  val1 = rfabs(val);
  res &= ( val1 <= 
	(a[1] * fabs_of_Place[2][0] + a[2] * fabs_of_Place[1][0] +
	 b[1] * fabs_of_Place[0][2] + b[2] * fabs_of_Place[0][1]));
  if (!res) return 7;
  
  /* A0 x B1 */
  val = Place[2][3] * Place[1][1] - Place[1][3] * Place[2][1];
  val1 = rfabs(val);
  res &= ( val1 <=
	(a[1] * fabs_of_Place[2][1] + a[2] * fabs_of_Place[1][1] +
	 b[0] * fabs_of_Place[0][2] + b[2] * fabs_of_Place[0][0]));
  if (!res) return 8;

  /* A0 x B2 */
  val = Place[2][3] * Place[1][2] - Place[1][3] * Place[2][2];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (a[1] * fabs_of_Place[2][2] + a[2] * fabs_of_Place[1][2] +
	   b[0] * fabs_of_Place[0][1] + b[1] * fabs_of_Place[0][0]));
  if (!res) return 9;

  /* A1 x B0 */
  val = Place[0][3] * Place[2][0] - Place[2][3] * Place[0][0];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (a[0] * fabs_of_Place[2][0] + a[2] * fabs_of_Place[0][0] +
	   b[1] * fabs_of_Place[1][2] + b[2] * fabs_of_Place[1][1]));
  if (!res) return 10;

  /* A1 x B1 */
  val = Place[0][3] * Place[2][1] - Place[2][3] * Place[0][1];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (a[0] * fabs_of_Place[2][1] + a[2] * fabs_of_Place[0][1] +
	   b[0] * fabs_of_Place[1][2] + b[2] * fabs_of_Place[1][0]));
  if (!res) return 11;

  /* A1 x B2 */
  val = Place[0][3] * Place[2][2] - Place[2][3] * Place[0][2];
  val1 = rfabs(val);
  res &= (val1 <=
	  (a[0] * fabs_of_Place[2][2] + a[2] * fabs_of_Place[0][2] +
	   b[0] * fabs_of_Place[1][1] + b[1] * fabs_of_Place[1][0]));
  if (!res) return 12;

  /* A2 x B0 */
  val = Place[1][3] * Place[0][0] - Place[0][3] * Place[1][0];
  val1 = rfabs(val);
  res &= (val1 <=
	  (a[0] * fabs_of_Place[1][0] + a[1] * fabs_of_Place[0][0] +
	   b[1] * fabs_of_Place[2][2] + b[2] * fabs_of_Place[2][1]));
  if (!res) return 13;

  /* A2 x B1 */
  val = Place[1][3] * Place[0][1] - Place[0][3] * Place[1][1];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (a[0] * fabs_of_Place[1][1] + a[1] * fabs_of_Place[0][1] +
	   b[0] * fabs_of_Place[2][2] + b[2] * fabs_of_Place[2][0]));
  if (!res) return 14;

  /* A2 x B2 */
  val = Place[1][3] * Place[0][2] - Place[0][3] * Place[1][2];
  val1 = rfabs(val);
  res &= ( val1 <=
	  (a[0] * fabs_of_Place[1][2] + a[1] * fabs_of_Place[0][2] +
	   b[0] * fabs_of_Place[2][1] + b[1] * fabs_of_Place[2][0]));
  if (!res) return 15;

  return 0;   /* OBBs are not disjoint, they collide */
}

/* second_box is a leaf of an OBB-tree around an obstacle, rob_box is a OBB-(sub)tree around a robot link */
/* returns TRUE as soon as a collision is found between one of the leafs of rob_box and second_box */
/* returns FALSE if rob_box does not collide with second_box */
/* calls itself recursively and returns FALSE if all recursive calls returned FALSE */
static int collision_exists_with_robot_kids(kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat3,  
					    kcd_bb *second_box,int with_report, double *distance_estimate, int *nearest_obstacle)
{
  double the_distance_estimate = P3D_HUGE, kids_distance_estimate = P3D_HUGE, new_distance_estimate = P3D_HUGE;
  int collision_exists=FALSE,new_nearest_obstacle,kids_nearest_obstacle = 0; //avoid warning
  int j;
  kcd_matrix4 mat,NEWinvM2; 
  kcd_bb *a_box = NULL;
  kcd_vector3 gjk_sep_axis ;
  int volume_is_small, must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  kcd_matMultTransfo(invM2,mat3,mat );
  if( (with_report == DISTANCE_ESTIMATE))
    {
      collision_exists = !( obb_disjoint_and_distance(mat, rob_box->d, second_box->d, &the_distance_estimate) );
    }
  else
    {
      collision_exists = !( obb_disjoint(mat, rob_box->d, second_box->d) ); 
    }
  if(collision_exists)
    {  
      volume_is_small = (must_test_volume_for_robots) ? kcd_volume_smaller_than_user_defined_small_volume(rob_box->volume) : FALSE;
      if((rob_box->nof_children_bb > 0) && ( !volume_is_small) ) /* Modif. Carl on volume */
	{
	  collision_exists=FALSE;
	  for(j=0;(j<rob_box->nof_children_bb)&&(!collision_exists);j++) 
	    {
	      new_distance_estimate = P3D_HUGE;
	      a_box = rob_box->children_bb[j];

	      NEWinvM2[0][0] = a_box->eigenv[0][0];
	      NEWinvM2[0][1] = a_box->eigenv[0][1];
	      NEWinvM2[0][2] = a_box->eigenv[0][2];
	      NEWinvM2[1][0] = a_box->eigenv[1][0];
	      NEWinvM2[1][1] = a_box->eigenv[1][1];
	      NEWinvM2[1][2] = a_box->eigenv[1][2];
	      NEWinvM2[2][0] = a_box->eigenv[2][0];
	      NEWinvM2[2][1] = a_box->eigenv[2][1];
	      NEWinvM2[2][2] = a_box->eigenv[2][2];
		  
	      NEWinvM2[0][3] = -NEWinvM2[0][0]*a_box->center[0] -NEWinvM2[0][1]*a_box->center[1] -NEWinvM2[0][2]*a_box->center[2];
	      NEWinvM2[1][3] = -NEWinvM2[1][0]*a_box->center[0] -NEWinvM2[1][1]*a_box->center[1] -NEWinvM2[1][2]*a_box->center[2];
	      NEWinvM2[2][3] = -NEWinvM2[2][0]*a_box->center[0] -NEWinvM2[2][1]*a_box->center[1] -NEWinvM2[2][2]*a_box->center[2];
		  
	      NEWinvM2[3][0] = 0.0;
	      NEWinvM2[3][1] = 0.0;
	      NEWinvM2[3][2] = 0.0;
	      NEWinvM2[3][3] = 1.0;

	      /* only invM2 changed  */
	      collision_exists = collision_exists_with_robot_kids(a_box,NEWinvM2,mat3,second_box,with_report,&new_distance_estimate,&new_nearest_obstacle);
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
		}
	    }
	}
      else
	{
	  collision_exists=TRUE; 
	  if(( DO_KCD_GJK) && ( !volume_is_small ) && (!kcd_volume_smaller_than_user_defined_small_volume(second_box->volume)) ) 
	    {
	      gjk_sep_axis[0] = 0.0;
	      gjk_sep_axis[1] = 0.0;
	      gjk_sep_axis[2] = 0.0;
	      /* call GJK intersection detection */
	      collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,with_report,&kids_distance_estimate,kcd_tolerance); 
	      if(collision_exists)
		{
		  /* kcd_modify_nearest_obstacle(rob_id,second_box->bb_id_in_all_bbs); */	
		  kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		  kids_distance_estimate = 0.0;
		}
	      else
		{
		  kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		}
	    }
	  else
	    {
	      kids_distance_estimate = 0.0;
	      /* kcd_modify_nearest_obstacle(rob_id,second_box->bb_id_in_all_bbs);	  */     
	      kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
	      /* B Kineo Carl 22.02.2002 */
	      kcd_set_pairInCollision(rob_box->pol,rob_box->ext_pol_id,second_box->pol,second_box->ext_pol_id);
	      /* E Kineo Carl 22.02.2002 */
	    }
	}
      if(distance_estimate)
	{	
	  if(collision_exists)
	    {
	      (*distance_estimate) = 0.0;
	      *nearest_obstacle = second_box->bb_id_in_all_bbs;
	    }
	  else
	    {
	      (*distance_estimate) = kids_distance_estimate;
	      *nearest_obstacle = kids_nearest_obstacle;
	    }
	}
    }
  else
    {
      /* PrintInfo(("collision_exists_with_robot_kids: no collision on first level\n")); */
      if(distance_estimate)
	{
	  (*distance_estimate) = the_distance_estimate;
	  *nearest_obstacle = second_box->bb_id_in_all_bbs;
	}
    }

  return collision_exists;
}


static int collision_exists_between_2_obbtrees(kcd_bb *rob_box, kcd_matrix4 invM2, 
					       kcd_matrix4 mat2, kcd_matrix4 a_box_pos, kcd_bb *second_box,
					       kcd_matrix4 *THISmat9, kcd_matrix4 *THISmat8,int with_report, 
					       double *distance_estimate, int *nearest_obstacle) 
{
  int collision_exists=FALSE;
  double the_distance_estimate = P3D_HUGE, kids_distance_estimate = P3D_HUGE, new_distance_estimate = P3D_HUGE;
  int i,j,nof_kids,new_nearest_obstacle,kids_nearest_obstacle = 0; //avoid warning
  kcd_matrix4 NEWinvM2;
  kcd_matrix4 M1,mat,mat9,mat8;
  kcd_bb *a_box = NULL;
  kcd_bb *a_bb = NULL;
  kcd_vector3 gjk_sep_axis ;
  kcd_vector3 *a_center = NULL;
  int volume_is_small,robot_volume_is_small,must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

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

  if(with_report == DISTANCE_ESTIMATE)
    {
      collision_exists = !( obb_disjoint_and_distance(mat, rob_box->d, second_box->d, &the_distance_estimate) );
    }
  else
    {
      collision_exists = !( obb_disjoint(mat, rob_box->d, second_box->d) ); 
    }
  if(collision_exists)
    {  
      volume_is_small = kcd_volume_smaller_than_user_defined_small_volume(second_box->volume);
      nof_kids = second_box->nof_children_bb;
      if((rob_box->d[0] < second_box->d[0]) && (nof_kids > 0) && (!volume_is_small))
	{
	  /* recurse with kids of obstacle */
	  collision_exists = FALSE;
	  for(i=0;(i<nof_kids)&&(!collision_exists);i++)
	    {
	      new_distance_estimate = P3D_HUGE;
	      a_bb = (second_box->children_bb)[i];
	      
	      /* change Place to matrix of eigenvalues of the child box */
	      M1[0][0] = a_bb->eigenv[0][0];
	      M1[1][0] = a_bb->eigenv[0][1];
	      M1[2][0] = a_bb->eigenv[0][2];
	      M1[0][3] = a_bb->center[0];
	      M1[0][1] = a_bb->eigenv[1][0];
	      M1[1][1] = a_bb->eigenv[1][1];
	      M1[2][1] = a_bb->eigenv[1][2];
	      M1[1][3] = a_bb->center[1];
	      M1[0][2] = a_bb->eigenv[2][0];
	      M1[1][2] = a_bb->eigenv[2][1];
	      M1[2][2] = a_bb->eigenv[2][2];
	      M1[2][3] = a_bb->center[2];
	      M1[3][0] = 0.0;
	      M1[3][1] = 0.0;
	      M1[3][2] = 0.0;
	      M1[3][3] = 1.0;
	      
	      /* only M1 is new => pass on mat3 */
	      if(THISmat9)
		{
		  collision_exists = 
		    collision_exists_between_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,THISmat9,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      else
		{
		  kcd_matMultTransfo(invM2,mat2,mat9);
		  collision_exists = 
		    collision_exists_between_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,&mat9,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
		}
	    }
	  /*done: recurse with kids of obstacle */
	}
      else if((rob_box->nof_children_bb > 0) && (!robot_volume_is_small))
	{
	  /* recurse with kids of robot link */
	  collision_exists=FALSE;
	  for(j=0;(j<rob_box->nof_children_bb)&&(!collision_exists);j++) 
	    {
	      new_distance_estimate = P3D_HUGE;
	      a_box = rob_box->children_bb[j];
	      
	      NEWinvM2[0][0] = a_box->eigenv[0][0];
	      NEWinvM2[0][1] = a_box->eigenv[0][1];
	      NEWinvM2[0][2] = a_box->eigenv[0][2];
	      NEWinvM2[1][0] = a_box->eigenv[1][0];
	      NEWinvM2[1][1] = a_box->eigenv[1][1];
	      NEWinvM2[1][2] = a_box->eigenv[1][2];
	      NEWinvM2[2][0] = a_box->eigenv[2][0];
	      NEWinvM2[2][1] = a_box->eigenv[2][1];
	      NEWinvM2[2][2] = a_box->eigenv[2][2];
	      
	      a_center = &(a_box->center);
	      NEWinvM2[0][3] = -NEWinvM2[0][0]*(*a_center)[0] -NEWinvM2[0][1]*(*a_center)[1] -NEWinvM2[0][2]*(*a_center)[2];
	      NEWinvM2[1][3] = -NEWinvM2[1][0]*(*a_center)[0] -NEWinvM2[1][1]*(*a_center)[1] -NEWinvM2[1][2]*(*a_center)[2];
	      NEWinvM2[2][3] = -NEWinvM2[2][0]*(*a_center)[0] -NEWinvM2[2][1]*(*a_center)[1] -NEWinvM2[2][2]*(*a_center)[2];
	      
	      NEWinvM2[3][0] = 0.0;
	      NEWinvM2[3][1] = 0.0;
	      NEWinvM2[3][2] = 0.0;
	      NEWinvM2[3][3] = 1.0;
	      
		  
	      /* only invM2 changed => do 1 multiplication here and pass on result */
	      if(THISmat8)
		{
		  collision_exists = 
		    collision_exists_between_2_obbtrees(a_box,NEWinvM2,mat2,a_box_pos,second_box,NULL,THISmat8,with_report,&new_distance_estimate,&new_nearest_obstacle); 
		}
	      else
		{
		  /* compute mat8 */
		  kcd_matMultTransfo(mat2,a_box_pos,mat8 );
		  
		  collision_exists = 
		    collision_exists_between_2_obbtrees(a_box,NEWinvM2,mat2,a_box_pos,second_box,NULL,&mat8,with_report,&new_distance_estimate,&new_nearest_obstacle);    
		}
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
		}
	    }
	}
      else if( (nof_kids > 0) && (!volume_is_small) )
	{
	  /* go down the kids of the obstacle anyway (since the robot doesn't have any other children */
	  collision_exists = FALSE;
	  for(i=0;(i<nof_kids)&&(!collision_exists);i++)
	    {
	      /* change Place to matrix of eigenvalues of the child box */
	      new_distance_estimate = P3D_HUGE;
	      a_bb = (second_box->children_bb)[i];

	      M1[0][0] = a_bb->eigenv[0][0];
	      M1[1][0] = a_bb->eigenv[0][1];
	      M1[2][0] = a_bb->eigenv[0][2];
	      M1[0][3] = a_bb->center[0];
	      M1[0][1] = a_bb->eigenv[1][0];
	      M1[1][1] = a_bb->eigenv[1][1];
	      M1[2][1] = a_bb->eigenv[1][2];
	      M1[1][3] = a_bb->center[1];
	      M1[0][2] = a_bb->eigenv[2][0];
	      M1[1][2] = a_bb->eigenv[2][1];
	      M1[2][2] = a_bb->eigenv[2][2];
	      M1[2][3] = a_bb->center[2];
	      M1[3][0] = 0.0;
	      M1[3][1] = 0.0;
	      M1[3][2] = 0.0;
	      M1[3][3] = 1.0;
	      
	      /* only M1 is new => pass on mat3 */
	      if(THISmat9)
		{
		  collision_exists = 
		    collision_exists_between_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,THISmat9,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      else
		{
		  /* compute mat9 */
		  kcd_matMultTransfo(invM2,mat2,mat9);
		  collision_exists = 
		    collision_exists_between_2_obbtrees(rob_box,invM2,mat2,M1,a_bb,&mat9,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
		}
	    }
	}
      else
	{
	  collision_exists=TRUE; 	  
	  if((DO_KCD_GJK) && (!volume_is_small) && (!robot_volume_is_small) )
	    {
	      gjk_sep_axis[0] = 0.0;
	      gjk_sep_axis[1] = 0.0;
	      gjk_sep_axis[2] = 0.0;
	      /* call GJK intersection detection */
	      collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,with_report,&kids_distance_estimate,kcd_tolerance);
	      if(collision_exists)
		{
		  kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		  kids_distance_estimate = 0.0;
		}
	      else
		{
		  kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		}
	    }
	  else
	    {
	      (*distance_estimate) = 0.0;
	      /* since OBB collides */
	      kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
	      /* B Kineo Carl 22.02.2002 */
	      kcd_set_pairInCollision(rob_box->pol,rob_box->ext_pol_id,second_box->pol,second_box->ext_pol_id);
	      /* E Kineo Carl 22.02.2002 */
	    }
	}
      if(distance_estimate)
	{
	  if(collision_exists)
	    {
	      (*distance_estimate) = 0.0;
	      *nearest_obstacle = kids_nearest_obstacle; /* second_box->bb_id_in_all_bbs; */
	    }
	  else
	    {
	      (*distance_estimate) = kids_distance_estimate;
	      *nearest_obstacle = kids_nearest_obstacle;
	    }
	}
    }
  else
    {
      if(distance_estimate)
	{
	  (*distance_estimate) = the_distance_estimate;
	  *nearest_obstacle = second_box->bb_id_in_all_bbs;
	}
    }
  return collision_exists;
}

/* precond: second_box is a obb-tree */
int obb_disjoint_trees(int mobj_id, kcd_matrix4 a_box_pos,kcd_matrix4 inv_a_pos, double a[4], kcd_bb *second_box,
		       int with_report,double *minimal_distance_estimate,int *nearest_obstacle)
{
  double the_distance_estimate=P3D_HUGE,kids_distance_estimate=P3D_HUGE,new_distance_estimate=P3D_HUGE;
  int i,nof_kids,kids_nearest_obstacle,new_nearest_obstacle;
  int obb_collision_exists = TRUE, collision_exists = FALSE,guess_of_poly_id=0;
  kcd_matrix4 M1,invM2,mat2,mat3,Place,mat9;
  /* p3d_poly *pol_it = NULL; */
  kcd_bb *a_bb = NULL;
  kcd_vector3 *a_center = NULL;
  int volume_is_small,robot_volume_is_small,must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */
  double a_vol;        /* Modif. Carl on volume */
  kcd_matrix4 *jnt_pos;
  int kcd_ext_o;

#ifdef APPROX_ROBOT
  must_test_volume_for_robots = TRUE;
#endif

  /* begin modif. Carl on volume */
  volume_is_small = kcd_volume_smaller_than_user_defined_small_volume(second_box->volume);
  a_vol = 8.0 * a[0] * a[1] * a[2];  /* end modif. Carl on volume */
  robot_volume_is_small = must_test_volume_for_robots ? kcd_volume_smaller_than_user_defined_small_volume(a_vol) : FALSE;

  /* test the robot link aabb */

#ifdef USE_P3D_BODY_AABB
  kcd_matMultTransfo(inv_a_pos,a_box_pos,Place);

  if((with_report == DISTANCE_ESTIMATE)/* || (with_report == DISTANCE_EXACT)*/ )
    {
      obb_collision_exists = !( obb_disjoint_and_distance(Place, a, second_box->d, &the_distance_estimate) );
    }
  else
    {
      obb_collision_exists = !obb_disjoint(Place,a,second_box->d);
    }
#else
  obb_collision_exists = TRUE; 
#endif

  if(with_report == DISTANCE_EXACT)
    {
     // PrintInfo(("distance EXACT\n"));
      obb_collision_exists = TRUE;
    }

  if( obb_collision_exists )
    {
      /* check children of obstacle, if any */
      nof_kids = second_box->nof_children_bb;
      if((nof_kids > 0) && ( !volume_is_small ))
	{	
	  if( a[3] < second_box->d[0] )
	    {	   
		  /* go on testing robot link AABB against obstacle OBB-tree */
		  for(i=0;(i<nof_kids)&&(!collision_exists);i++)
		    {
		      new_distance_estimate = P3D_HUGE;
		      /* change Place to matrix of eigenvalues of the child box */
		      a_bb = (second_box->children_bb)[i];
		      /* begin modif. Carl on volume */

		      M1[0][0] = a_bb->eigenv[0][0];
		      M1[1][0] = a_bb->eigenv[0][1];
		      M1[2][0] = a_bb->eigenv[0][2];
		      M1[0][3] = a_bb->center[0];
		      M1[0][1] = a_bb->eigenv[1][0];
		      M1[1][1] = a_bb->eigenv[1][1];
		      M1[2][1] = a_bb->eigenv[1][2];
		      M1[1][3] = a_bb->center[1];
		      M1[0][2] = a_bb->eigenv[2][0];
		      M1[1][2] = a_bb->eigenv[2][1];
		      M1[2][2] = a_bb->eigenv[2][2];
		      M1[2][3] = a_bb->center[2];
		      M1[3][0] = 0.0;
		      M1[3][1] = 0.0;
		      M1[3][2] = 0.0;
		      M1[3][3] = 1.0;
		      
		      collision_exists = /* collision_exists ||  */
			( !obb_disjoint_trees(mobj_id,M1,inv_a_pos,a,a_bb,with_report,&new_distance_estimate,&new_nearest_obstacle ) );
		      if(new_distance_estimate < kids_distance_estimate)
			{
			  kids_distance_estimate = new_distance_estimate;
			  kids_nearest_obstacle = new_nearest_obstacle;
			}
		    }	  
	    }
	  else if( !robot_volume_is_small ) 
	    {
	      //PrintInfo(("test_two_trees\n"));
	      /* go on testing this small obstacle OBB against the OBB-tree of this link */
	      guess_of_poly_id = kcd_mo_bb_table[mobj_id].bb_id;
	      kcd_ext_o = kcd_mo_bb_table[mobj_id].ext_o_id;
	      kcd_get_obj_abs_pos(kcd_ext_o,&jnt_pos);

	      kcd_matInvertTransfo(*jnt_pos,mat2);
	      invM2[0][0] = all_bbs[guess_of_poly_id]->eigenv[0][0];
	      invM2[0][1] = all_bbs[guess_of_poly_id]->eigenv[0][1];
	      invM2[0][2] = all_bbs[guess_of_poly_id]->eigenv[0][2];
	      invM2[1][0] = all_bbs[guess_of_poly_id]->eigenv[1][0];
	      invM2[1][1] = all_bbs[guess_of_poly_id]->eigenv[1][1];
	      invM2[1][2] = all_bbs[guess_of_poly_id]->eigenv[1][2];
	      invM2[2][0] = all_bbs[guess_of_poly_id]->eigenv[2][0];
	      invM2[2][1] = all_bbs[guess_of_poly_id]->eigenv[2][1];
	      invM2[2][2] = all_bbs[guess_of_poly_id]->eigenv[2][2];
	      a_center = &(all_bbs[guess_of_poly_id]->center);
	      invM2[0][3] = -invM2[0][0]*(*a_center)[0] -invM2[0][1]*(*a_center)[1] -invM2[0][2]*(*a_center)[2];
	      invM2[1][3] = -invM2[1][0]*(*a_center)[0] -invM2[1][1]*(*a_center)[1] -invM2[1][2]*(*a_center)[2];
	      invM2[2][3] = -invM2[2][0]*(*a_center)[0] -invM2[2][1]*(*a_center)[1] -invM2[2][2]*(*a_center)[2];
	      
	      invM2[3][0] = 0.0;
	      invM2[3][1] = 0.0;
	      invM2[3][2] = 0.0;
	      invM2[3][3] = 1.0;
	      
	      kcd_matMultTransfo(invM2,mat2,mat9);
	   
	      /* collision_exists = collision_exists_between_2_obbtrees(all_bbs[guess_of_poly_id],invM2,mat2,a_box_pos,
				  second_box,&mat9,NULL,with_report,&kids_distance_estimate,&kids_nearest_obstacle); */

	      if(with_report == DISTANCE_EXACT )
		{
//		    PrintInfo(("distance exact two\n"));
		  collision_exists =  kcd_dist_exact_between_2_obbtrees(all_bbs[guess_of_poly_id],invM2,mat2,a_box_pos,
									second_box,&mat9,NULL,&kids_distance_estimate,&kids_nearest_obstacle,GO_DOWN_BOTH_TREES); 
		}
	      else
		{
		  collision_exists = collision_exists_between_2_obbtrees(all_bbs[guess_of_poly_id],invM2,mat2,a_box_pos,
									 second_box,&mat9,NULL,with_report,&kids_distance_estimate,&kids_nearest_obstacle); 
		}
	      /* if(collision_exists)
		 PrintInfo(("collision with 2 trees\n"));*/


	      /*(kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat2, kcd_matrix4 a_box_pos, kcd_bb *second_box,
		kcd_matrix4 *THISmat9, kcd_matrix4 *THISmat8,double *distance_estimate, int *nearest_obstacle) */


	    }
	    
	}
      else if( !robot_volume_is_small )
	{
	  /* we got a leaf of an OBB(-Tree) around an obstacle: */

	  /* we'll go on checking the obb(-tree)s of the polyhedrons and solids of the link */
	  /* get joint matrix for robot and body */

	  guess_of_poly_id = kcd_mo_bb_table[mobj_id].bb_id;

	  /* begin modif. Carl on volume */
	  kcd_ext_o = kcd_mo_bb_table[mobj_id].ext_o_id;
	  kcd_get_obj_abs_pos(kcd_ext_o,&jnt_pos);

	  kcd_matInvertTransfo(*jnt_pos,mat2);	  
	  
	  invM2[0][0] = all_bbs[guess_of_poly_id]->eigenv[0][0];
	  invM2[0][1] = all_bbs[guess_of_poly_id]->eigenv[0][1];
	  invM2[0][2] = all_bbs[guess_of_poly_id]->eigenv[0][2];
	  invM2[1][0] = all_bbs[guess_of_poly_id]->eigenv[1][0];
	  invM2[1][1] = all_bbs[guess_of_poly_id]->eigenv[1][1];
	  invM2[1][2] = all_bbs[guess_of_poly_id]->eigenv[1][2];
	  invM2[2][0] = all_bbs[guess_of_poly_id]->eigenv[2][0];
	  invM2[2][1] = all_bbs[guess_of_poly_id]->eigenv[2][1];
	  invM2[2][2] = all_bbs[guess_of_poly_id]->eigenv[2][2];
	  a_center = &(all_bbs[guess_of_poly_id]->center);
	  invM2[0][3] = -invM2[0][0]*(*a_center)[0] -invM2[0][1]*(*a_center)[1] -invM2[0][2]*(*a_center)[2];
	  invM2[1][3] = -invM2[1][0]*(*a_center)[0] -invM2[1][1]*(*a_center)[1] -invM2[1][2]*(*a_center)[2];
	  invM2[2][3] = -invM2[2][0]*(*a_center)[0] -invM2[2][1]*(*a_center)[1] -invM2[2][2]*(*a_center)[2];
	  
	  invM2[3][0] = 0.0;
	  invM2[3][1] = 0.0;
	  invM2[3][2] = 0.0;
	  invM2[3][3] = 1.0;
	  /* call for collision (recursively) */
	  
	  kcd_matMultTransfo(mat2,a_box_pos,mat3 ); 
	  /* we rather pass on mat3 */
	  /*
	  collision_exists =   collision_exists_with_robot_kids(all_bbs[guess_of_poly_id],invM2,mat3,
								second_box,with_report,&kids_distance_estimate,&kids_nearest_obstacle); */

	  /* if DISTANCE_EXACT */
	  if(with_report == DISTANCE_EXACT )
	    {
//	      PrintInfo(("distance exact robot  mobj %d   second_box_id %d\n",mobj_id, second_box->bb_id_in_all_bbs));

	      collision_exists =  kcd_dist_exact_between_2_obbtrees(all_bbs[guess_of_poly_id],invM2,mat3,NULL,
								    second_box,NULL,NULL,&kids_distance_estimate,&kids_nearest_obstacle,GO_DOWN_ROBOT_TREE); 
	    }
	  else
	    {
	      collision_exists =   collision_exists_with_robot_kids(all_bbs[guess_of_poly_id],invM2,mat3,
								    second_box,with_report,&kids_distance_estimate,&kids_nearest_obstacle);
	    }
	 /* if(collision_exists)
	    PrintInfo(("collision with robot_kids\n"));*/
	}
      else
	{
	  collision_exists = TRUE;
	  /* B Kineo Carl 22.02.2002 */
	  kcd_set_pairInCollision(all_bbs[guess_of_poly_id]->pol,all_bbs[guess_of_poly_id]->ext_pol_id,second_box->pol,second_box->ext_pol_id);
	  /* E Kineo Carl 22.02.2002 */
	}
   //   PrintInfo((" %lf = kids\n", kids_distance_estimate));
      if(minimal_distance_estimate)
	{
	  if(collision_exists)
	    {
	      *minimal_distance_estimate = 0.0;
	      *nearest_obstacle = kids_nearest_obstacle;
	    }
	  else
	    {
	      if(*minimal_distance_estimate > kids_distance_estimate)
		{ 
		//  PrintInfo(("we set in trees %lf = kids2\n", kids_distance_estimate));
		  *minimal_distance_estimate = kids_distance_estimate;
		  *nearest_obstacle = kids_nearest_obstacle;
		}
	    }
	}
    }
  else
    {
      collision_exists = FALSE;
      if(minimal_distance_estimate)
	{
	  *minimal_distance_estimate = the_distance_estimate;
	  *nearest_obstacle = second_box->bb_id_in_all_bbs;
	}
    }

  
  return !collision_exists;
}



/* similar to collision_exists_between_2_obbtrees, but for 2 robots */
int collision_exists_between_2_robot_obbtrees(kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat2, kcd_matrix4 mat1, 
					      kcd_matrix4 thisM1, kcd_bb *second_box, kcd_matrix4 *THISmat8, kcd_matrix4 *THISmat9,int with_report, 
					       double *distance_estimate, int *nearest_obstacle) 
{
  int collision_exists=FALSE;
  double the_distance_estimate = P3D_HUGE, kids_distance_estimate = P3D_HUGE, new_distance_estimate = P3D_HUGE;
  int i,j,nof_kids,new_nearest_obstacle,kids_nearest_obstacle = 0; // avoid warning
  kcd_matrix4 NEWinvM2,M1;
  kcd_matrix4 mat9,mat,mat8;
  kcd_bb *a_box = NULL;
  kcd_bb *a_bb = NULL;
  kcd_vector3 gjk_sep_axis ;
  kcd_vector3 *a_center = NULL;
  int must_test_volume_for_robots = FALSE; /* Modif. Carl on volume */
  int volume_is_small; /* Modif. Carl on volume */

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

  if((with_report == DISTANCE_ESTIMATE) )
    {
      collision_exists = !( obb_disjoint_and_distance(mat, rob_box->d, second_box->d, &the_distance_estimate) );
 /*PrintInfo(("overlap  %.0lf \t",the_distance_estimate));
      kcd_max_dist_est_between_2_OBB(rob_box, second_box, mat,&the_distance_estimate);   
      PrintInfo(("max  %.0lf \t",the_distance_estimate)); 
      kcd_min_dist_est_between_2_OBB(rob_box, second_box, mat,&the_distance_estimate);   
      PrintInfo(("min  %.0lf \t",the_distance_estimate));
      kcd_gjk_between_2_OBB(rob_box, second_box, &mat,&the_distance_estimate);    
      PrintInfo(("gjk  %.0lf \n",the_distance_estimate));*/
/*      collision_exists = TRUE;
      the_distance_estimate = 111111111111;*/
   
    }
  else
    {
      collision_exists = !( obb_disjoint(mat, rob_box->d, second_box->d) ); 
    }

  if(collision_exists)
    {  
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

	      M1[0][0] = a_bb->eigenv[0][0];
	      M1[1][0] = a_bb->eigenv[0][1];
	      M1[2][0] = a_bb->eigenv[0][2];
	      M1[0][3] = a_bb->center[0];
	      M1[0][1] = a_bb->eigenv[1][0];
	      M1[1][1] = a_bb->eigenv[1][1];
	      M1[2][1] = a_bb->eigenv[1][2];
	      M1[1][3] = a_bb->center[1];
	      M1[0][2] = a_bb->eigenv[2][0];
	      M1[1][2] = a_bb->eigenv[2][1];
	      M1[2][2] = a_bb->eigenv[2][2];
	      M1[2][3] = a_bb->center[2];
	      M1[3][0] = 0.0;
	      M1[3][1] = 0.0;
	      M1[3][2] = 0.0;
	      M1[3][3] = 1.0;
	      /* only M1 changed => pass on mat8 */
	      if(THISmat8)
		{
		  collision_exists = 
		    collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,THISmat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      else
		{
		  collision_exists = 
		    collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,&mat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
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

	      NEWinvM2[0][0] = a_box->eigenv[0][0];
	      NEWinvM2[0][1] = a_box->eigenv[0][1];
	      NEWinvM2[0][2] = a_box->eigenv[0][2];
	      NEWinvM2[1][0] = a_box->eigenv[1][0];
	      NEWinvM2[1][1] = a_box->eigenv[1][1];
	      NEWinvM2[1][2] = a_box->eigenv[1][2];
	      NEWinvM2[2][0] = a_box->eigenv[2][0];
	      NEWinvM2[2][1] = a_box->eigenv[2][1];
	      NEWinvM2[2][2] = a_box->eigenv[2][2];
	      
	      a_center = &(a_box->center);
	      NEWinvM2[0][3] = -NEWinvM2[0][0]*(*a_center)[0] -NEWinvM2[0][1]*(*a_center)[1] -NEWinvM2[0][2]*(*a_center)[2];
	      NEWinvM2[1][3] = -NEWinvM2[1][0]*(*a_center)[0] -NEWinvM2[1][1]*(*a_center)[1] -NEWinvM2[1][2]*(*a_center)[2];
	      NEWinvM2[2][3] = -NEWinvM2[2][0]*(*a_center)[0] -NEWinvM2[2][1]*(*a_center)[1] -NEWinvM2[2][2]*(*a_center)[2];
	      
	      NEWinvM2[3][0] = 0.0;
	      NEWinvM2[3][1] = 0.0;
	      NEWinvM2[3][2] = 0.0;
	      NEWinvM2[3][3] = 1.0;
	      
	      /* only invM2 changed => pass on mat9 and thisM1 */
	      if(THISmat9)
		{
		  collision_exists = collision_exists_between_2_robot_obbtrees(a_box,NEWinvM2,mat2,mat1,thisM1,second_box,
									       NULL,THISmat9,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      else
		{
		  if(THISmat8)
		    kcd_matMultTransfo(mat2,mat1,mat9);
		  collision_exists = collision_exists_between_2_robot_obbtrees(a_box,NEWinvM2,mat2,mat1,thisM1,second_box,
									       NULL,&mat9,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
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
	      M1[0][0] = a_bb->eigenv[0][0];
	      M1[1][0] = a_bb->eigenv[0][1];
	      M1[2][0] = a_bb->eigenv[0][2];
	      M1[0][3] = a_bb->center[0];
	      M1[0][1] = a_bb->eigenv[1][0];
	      M1[1][1] = a_bb->eigenv[1][1];
	      M1[2][1] = a_bb->eigenv[1][2];
	      M1[1][3] = a_bb->center[1];
	      M1[0][2] = a_bb->eigenv[2][0];
	      M1[1][2] = a_bb->eigenv[2][1];
	      M1[2][2] = a_bb->eigenv[2][2];
	      M1[2][3] = a_bb->center[2];
	      M1[3][0] = 0.0;
	      M1[3][1] = 0.0;
	      M1[3][2] = 0.0;
	      M1[3][3] = 1.0;
	      /* only M1 changed => pass on mat8 */
	      if(THISmat8)
		{
		  collision_exists = 
		    collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,THISmat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      else
		{
		  collision_exists = 
		    collision_exists_between_2_robot_obbtrees(rob_box,invM2,mat2,mat1,M1,a_bb,&mat8,NULL,with_report,&new_distance_estimate,&new_nearest_obstacle);
		}
	      if(kids_distance_estimate > new_distance_estimate)
		{
		  kids_distance_estimate = new_distance_estimate;
		  kids_nearest_obstacle = new_nearest_obstacle;
		}
	    }
	}
      else
	{
	  /* PrintInfo(("   no kids, but collision\n")); */
	  collision_exists=TRUE; 

	  if((must_test_volume_for_robots ? (!volume_is_small) && (!kcd_volume_smaller_than_user_defined_small_volume( rob_box->volume)) : TRUE))
	    {
	      gjk_sep_axis[0] = 0.0;
	      gjk_sep_axis[1] = 0.0;
	      gjk_sep_axis[2] = 0.0;
	      /* call GJK intersection detection */
	      if(DO_KCD_GJK)
		{		  
		  /* collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,FALSE,NULL,0.0); */
		  collision_exists = kcd_gjk_intersect(rob_box,second_box,&gjk_sep_axis,
						       with_report,&kids_distance_estimate,kcd_tolerance);
		  if(collision_exists)
		    {
		      kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		      kids_distance_estimate = 0.0;
		    }
		  else
		    {
		      kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		    }
		}
	      else
		{
		  (*distance_estimate) = 0.0;
		  /* since OBB collides */
		  kids_nearest_obstacle = second_box->bb_id_in_all_bbs;
		  /* B Kineo Carl 22.02.2002 */
		  kcd_set_pairInCollision(rob_box->pol,rob_box->ext_pol_id,second_box->pol,second_box->ext_pol_id);
		  /* E Kineo Carl 22.02.2002 */
		}
	    }
	}
      if(distance_estimate)
	{
	  if(collision_exists)
	    {
	      (*distance_estimate) = 0.0;
	      *nearest_obstacle = kids_nearest_obstacle; /* second_box->bb_id_in_all_bbs; */
	    }
	  else
	    {
	      (*distance_estimate) = kids_distance_estimate;
	      *nearest_obstacle = kids_nearest_obstacle;
	    }
	}
    }
  else
    {
      /* no collision on first level */
      if(distance_estimate)
	{
	  (*distance_estimate) = the_distance_estimate;
	  *nearest_obstacle = second_box->bb_id_in_all_bbs;
	}
    }
  return collision_exists;
}
