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
/****************************************************************************/
/*!
 *  \file p3d_jnt_fixed.c
 *
 *  \brief Description of the ::P3D_FIXED joint.
 *
 *       This joint is fixed, it has no degree of freedom. It is used to
 * attach kinematic chains.
 */

#include "P3d-pkg.h"
#include "Util-pkg.h"


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  From the configuration q_init of the joint to q_max_param, 
 *  this function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_dist() for the joint
 *  ::P3D_FIXED.
 *
 *  Note: The joint looks directly in the robot configurations 
 *        the degree of freedom that it needs.
 *
 * \param  prev_data:   speed of the previous joint
 * \param  jntPt:       the joint
 * \param  distance:    the maximal distance
 * \param  qinit:       the initial configuration
 * \param  q_max_param: the final configuration
 * \param  max_param:   the value of the delta parameter in the final
 *                      configuration (it gives the range param needed
 *                                     to reach the final configuration)
 * \param  reach_param: the actual maximal parameter that could be reach
 *                      (previous joint can limits the range parameter)
 *
 * \retval data:        speed of the joint
 *         distance:    the distance that the joint couldn't cross
 *         reach_param: the new maximal range parameter 
 *                      that could be reach
 *
 *  \internal
 */
void p3d_jnt_fixed_stay_within_dist(p3d_stay_within_dist_data * prev_data,
				    p3d_jnt * jntPt,
				    p3d_stay_within_dist_data * data,
				    double * distance, configPt q_init, 
				    configPt q_max_param, 
				    double max_param, double * reach_param)
{
  double dist;         /* distance between joints */
  double velocity_max; /* maximum of the absolute speed */
  double range;        /* delta parameter that cross the distance */
  p3d_point p_min, p_max;

  p3d_jnt_get_point(jntPt, &(data->p));
  
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    if (prev_data->wmax < EPS6)
      { dist = 0; } /* We don't need to compute this distance */
    else {
      p_min.x = data->p.x +q_init[jntPt->index_dof]*jntPt->dof_data[0].axis[0];
      p_min.y = data->p.y +q_init[jntPt->index_dof]*jntPt->dof_data[0].axis[1];
      p_min.z = data->p.z +q_init[jntPt->index_dof]*jntPt->dof_data[0].axis[2];

      p_max.x = data->p.x + (q_init[jntPt->index_dof] + 
	     (q_max_param[jntPt->index_dof] - q_init[jntPt->index_dof]) *
	     (*reach_param) / max_param) * jntPt->dof_data[0].axis[0];
      p_max.y = data->p.y + (q_init[jntPt->index_dof] +
	     (q_max_param[jntPt->index_dof] - q_init[jntPt->index_dof]) *
	     (*reach_param) / max_param) * jntPt->dof_data[0].axis[1];
      p_max.z = data->p.z + (q_init[jntPt->index_dof] +
	     (q_max_param[jntPt->index_dof] - q_init[jntPt->index_dof]) *
	     (*reach_param) / max_param) * jntPt->dof_data[0].axis[2];
      dist = MAX(p3d_point_dist(prev_data->p, p_min),
		 p3d_point_dist(prev_data->p, p_max));
    }    
    data->wmax = prev_data->wmax;
    data->vmax = prev_data->vmax + dist*(prev_data->wmax);
    
    /* maximal velocity of all the points of the current body */
    if (p3d_jnt_is_with_object(jntPt))
      { velocity_max = data->vmax + (jntPt->dist)*(data->wmax); }
    else
      { velocity_max = 0.0; }
    
    if (velocity_max > EPS6)
      { range = (*distance)/velocity_max; }
    else
      { range = P3D_HUGE; }
    if (range<*reach_param)
      { *reach_param = range; }
    *distance -= velocity_max*(*reach_param);
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_FIXED
 *
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint
 * \param  vmin & vmax:  the bounds values of 
 *                       the degree of freedom for the joint
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of 
 *                                the degree of freedom for the joint
 * \param  param: the array of the parameters for the joint (NULL here)
 *
 * \return the new joint.
 *
 *  \internal
 */
p3d_jnt * p3d_jnt_fixed_create(p3d_matrix4 pos, double * v, 
			       double * vmin, double * vmax, 
			       double * vmin_rand, double * vmax_rand,
			       double * param) 
{
  p3d_jnt * jntPt;

  jntPt = p3d_jnt_create_common(pos);
  if(jntPt == NULL)
    { return NULL; }

  jntPt->type        = P3D_FIXED;

  jntPt->dof_equiv_nbr =  0;

  p3d_mat4Copy(p3d_mat4IDENTITY, jntPt->jnt_mat);
  p3d_mat4Copy(jntPt->abs_pos_before_jnt, jntPt->abs_pos);

  return jntPt;
}

//start path defrom
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 * 
 *  This function computes an interval of parameter for which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_dist() for the joint
 *  ::P3D_FIXED.
 *
 *  Note: The joint looks directly in the robot configurations 
 *        the degree of freedom that it needs.
 *
 * \param  prev_data:   speed of the previous joint
 * \param  jntPt:       the joint
 * \param  distance:    the maximal distance
 *
 * \retval data:        speed of the joint
 *         distance:    the distance that the joint couldn't cross
 *         reach_param: the new maximal range parameter 
 *                      that could be reach
 *
 *  \internal
 */
void p3d_jnt_fixed_stay_within_sphere(p3d_stay_within_dist_data * prev_data,
            p3d_jnt * jntPt,
            p3d_stay_within_dist_data * data,
            double * distance, double * reach_param){
  double dist;         /* distance between joints */
  double velocity_max; /* maximum of the absolute speed */
  double range;        /* delta parameter that cross the distance */
  p3d_point p_min, p_max;
  double vmin, vmax;

  p3d_jnt_get_point(jntPt, &(data->p));

  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  }else{
    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    if (prev_data->wmax < EPS6){
      dist = 0; /* We don't need to compute this distance */
    }else{
      p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vmin, &vmax);

      p_min.x = data->p.x + vmin* jntPt->dof_data[0].axis[0];
      p_min.y = data->p.y + vmin* jntPt->dof_data[0].axis[1];
      p_min.z = data->p.z + vmin* jntPt->dof_data[0].axis[2];

      p_max.x = data->p.x + vmax*jntPt->dof_data[0].axis[0];
      p_max.y = data->p.y + vmax*jntPt->dof_data[0].axis[1];
      p_max.z = data->p.z + vmax*jntPt->dof_data[0].axis[2];

      /*Warning :we should have p_min = p_max (to confirm) */
      dist = MAX(p3d_point_dist(prev_data->p, p_min), p3d_point_dist(prev_data->p, p_max));
    }
    data->wmax = prev_data->wmax;
    data->vmax = prev_data->vmax + dist*(prev_data->wmax);
    /* maximal velocity of all the points of the current body */
    if (p3d_jnt_is_with_object(jntPt)){
      velocity_max = data->vmax + (jntPt->dist)*(data->wmax);
    }else{
      velocity_max = 0.0;
    }
    if (velocity_max > EPS6){
      range = (*distance)/velocity_max;
    }else{
      range = P3D_HUGE;
    }
    if (range<*reach_param){
      *reach_param = range;
    }
    *distance -= velocity_max*(*reach_param);
  }
}
//end path deform
