/****************************************************************************/
/*!
 *  \file p3d_jnt_rotate.c
 *
 *  \brief Description of the ::P3D_ROTATE joint.
 *
 *       It's one of the basic joint in Move3D. This is a single
 * rotation joint. It's default orientation is the z axis
 * (modified by \a pos in p3d_jnt_rotate_create()).
 */

#include "P3d-pkg.h"
#include "Util-pkg.h"

/*! \brief Default name for a joint rotate.
 *  \internal
 */
static const char * rotate_name_dof = "Rot.";

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the name of a degree of freedom for a rotate joint
 *
 *  This is the implementation of p3d_jnt_get_dof_name() for the joint
 *  ::P3D_ROTATE.
 *
 *  \param i_dof:  the index of the degree of freedom 
 *
 *  \return The name of the degree of freedom
 *
 *  \internal
 *    The \a i_dof is not used here because there is only one degree 
 *    of freedom.
 */
const char * p3d_jnt_rotate_get_dof_name(int i_dof)
{ return rotate_name_dof; }


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  From the configuration q_init of the joint to q_max_param, 
 *  this function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_dist() for the joint
 *  ::P3D_ROTATE.
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
void p3d_jnt_rotate_stay_within_dist(p3d_stay_within_dist_data * prev_data, 
				     p3d_jnt * jntPt,
				     p3d_stay_within_dist_data * data, 
				     double * distance, configPt q_init,
				     configPt q_max_param, 
				     double max_param, double * reach_param)
{
  double wmax_rel;     /* relative speed */
  double dist;         /* distance between joints */
  double velocity_max; /* maximum of the absolute speed */
  double range;        /* delta parameter that cross the distance */

  p3d_jnt_get_point(jntPt, &(data->p));//jntPt->pos0
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    //distance between the current config of the robot (qinit) and the qmax_param for this joint.
    if (p3d_jnt_is_dof_circular(jntPt, 0)) {
      /* the joint can rotate freely */
      wmax_rel = fabs(dist_circle(q_max_param[jntPt->index_dof],
				  q_init[jntPt->index_dof])/max_param);
    } else {
      wmax_rel = fabs((q_max_param[jntPt->index_dof] - 
		       q_init[jntPt->index_dof])/max_param);
    }

    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    dist = p3d_point_dist(prev_data->p, data->p);
    
    data->wmax = prev_data->wmax + wmax_rel;
    data->vmax = prev_data->vmax + dist*(prev_data->wmax);
    
    /* maximal velocity of all the points of the current body */
    if (p3d_jnt_is_with_object(jntPt))
      {  velocity_max = data->vmax + (jntPt->dist)*(data->wmax); }
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
 *  \brief Compute the transformation matrix of the joint
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof(). This is a rotation on z axis.
 *  This is used in p3d_jnt_calc_mat_pos() for ::P3D_ROTATE
 *  joint.
 *
 * \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_rotate_calc_jnt_mat(p3d_jnt * jntPt)
{
  double cos_v, sin_v;

  cos_v = cos(jntPt->dof_data[0].v);
  sin_v = sin(jntPt->dof_data[0].v);

  jntPt->jnt_mat[0][0] = cos_v;    
  jntPt->jnt_mat[0][1] = - sin_v;
  jntPt->jnt_mat[0][2] = 0.0;
  jntPt->jnt_mat[0][3] = 0.0;
  jntPt->jnt_mat[1][0] = sin_v;
  jntPt->jnt_mat[1][1] = cos_v;
  jntPt->jnt_mat[1][2] = 0.0;
  jntPt->jnt_mat[1][3] = 0.0;
  jntPt->jnt_mat[2][0] = 0.0;
  jntPt->jnt_mat[2][1] = 0.0;
  jntPt->jnt_mat[2][2] = 1.0;
  jntPt->jnt_mat[2][3] = 0.0;
  jntPt->jnt_mat[3][0] = 0.0;
  jntPt->jnt_mat[3][1] = 0.0;
  jntPt->jnt_mat[3][2] = 0.0;
  jntPt->jnt_mat[3][3] = 1.0;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the jacobian matrix of the joint
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof(), the position matrix and the prejacobian
 *  matrix of the previous joint.
 *  This is used in p3d_jnt_calc_mat_pos() for ::P3D_ROTATE
 *  joint.
 *
 * \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_rotate_calc_mat_jac(p3d_jnt * jntPt)
{
  p3d_jnt * jnt_prevPt;
  p3d_link_between_joint * linkPt = NULL;
  p3d_matrix4 m_jac_j, tmp_mat;
  p3d_prejac *prevprejac, *prejac;
  double cos_v, sin_v;

  /* update prejacobian matrix */

  jnt_prevPt = jntPt->prev_jnt;

  prejac = jntPt->prejac;
  if (!prejac) {
    PrintError(("error jacobian : jacobian not initialized\n"));
    return;
  }

  p3d_mat4Copy(p3d_mat4NULL, m_jac_j);
  if (jntPt->dof_data[0].vmin<jntPt->dof_data[0].vmax) {
    cos_v = cos(jntPt->dof_data[0].v);
    sin_v = sin(jntPt->dof_data[0].v);

    m_jac_j[0][0] = - sin_v;
    m_jac_j[0][1] = - cos_v;
    m_jac_j[1][0] =   cos_v;
    m_jac_j[1][1] = - sin_v;
  }
  p3d_matMultXform(jntPt->abs_pos_before_jnt, m_jac_j, prejac->J);
  prejac = prejac->prev_prejac;

  /* update the other prejacobian matrix */
  if (jnt_prevPt!=NULL) {
    linkPt = jntPt->link_jnt_arr[0];
    prevprejac = jnt_prevPt->prejac;
    p3d_matMultXform(linkPt->rel_pos, jntPt->jnt_mat, tmp_mat);
    while (prejac!=NULL) {
      p3d_mat4Mult(prevprejac->J, tmp_mat, prejac->J);
      prejac = prejac->prev_prejac;
      prevprejac = prevprejac->prev_prejac;
    }
  }  
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_ROTATE with angle in radian
 *
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint in radian
 * \param  vmin & vmax:  the bounds values of 
 *                       the degree of freedom for the joint in radian
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of 
 *                                the degree of freedom for the joint in radian
 * \param  param: the array of the parameters for the joint (NULL here)
 *
 * \return the new joint.
 *
 *  \internal
 */
p3d_jnt * p3d_jnt_rotate_create(p3d_matrix4 pos, double * v, 
		double * vmin, double * vmax, 
		double * vmin_rand, double * vmax_rand, double * param)
{
  p3d_jnt * jntPt;

  if((*v<*vmin)||(*v>*vmax)) {
    PrintError(("p3d_jnt_rotate_create: initial value is out of range"));
    return NULL;
  }
  jntPt = p3d_jnt_create_common(pos);
  if(jntPt == NULL)
    { return NULL; }

  jntPt->type        = P3D_ROTATE;

  jntPt->dof_equiv_nbr =  1;

  /* The rotation of the z axis is the third columns of pos matrix */
  jntPt->dof_data[0].axis[0] = jntPt->axe[0] = pos[0][2]; 
  jntPt->dof_data[0].axis[1] = jntPt->axe[1] = pos[1][2]; 
  jntPt->dof_data[0].axis[2] = jntPt->axe[2] = pos[2][2];

  jntPt->dof_data[0].old_v = P3D_HUGE;
  p3d_jnt_set_dof(jntPt, 0, v[0]); 
  p3d_jnt_set_dof_v0(jntPt, 0, 0); 
  p3d_jnt_set_dof_bounds(jntPt, 0, vmin[0], vmax[0]); 
  p3d_jnt_set_dof_rand_bounds(jntPt, 0, vmin_rand[0], vmax_rand[0]); 
  if (LEQ(vmax[0], vmin[0]))
    { jntPt->dof_data[0].is_user = FALSE; }
  else {
    jntPt->dof_data[0].is_user = TRUE;
    jntPt->user_dof_equiv_nbr =1;
  }
  p3d_jnt_rotate_calc_jnt_mat(jntPt);
  p3d_matMultXform(jntPt->abs_pos_before_jnt, jntPt->jnt_mat, jntPt->abs_pos);

  return jntPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_ROTATE with angle in degree
 *
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint in degree
 * \param  vmin & vmax:  the bounds values of 
 *                       the degree of freedom for the joint in degree
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of 
 *                                the degree of freedom for the joint in degree
 * \param  param: the array of the parameters for the joint (NULL here)
 *
 * \return the new joint.
 *
 *  \internal
 */
p3d_jnt * p3d_jnt_rotate_create_deg(p3d_matrix4 pos, double * v, 
		double * vmin, double * vmax, 
		double * vmin_rand, double * vmax_rand, double * param)
{
  v[0] = DTOR(v[0]);
  vmin[0] = DTOR(vmin[0]);
  vmax[0] = DTOR(vmax[0]);
  vmin_rand[0] = DTOR(vmin_rand[0]);
  vmax_rand[0] = DTOR(vmax_rand[0]); 
  return p3d_jnt_rotate_create(pos, v, vmin, vmax,  
			       vmin_rand, vmax_rand, param);
}

//start path deform
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_sphere() for the joint
 *  ::P3D_ROTATE.
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
void p3d_jnt_rotate_stay_within_sphere(p3d_stay_within_dist_data * prev_data, 
             p3d_jnt * jntPt,
             p3d_stay_within_dist_data * data, 
             double * distance, 
             double * reach_param){
  double wmax_rel;     /* relative speed */
  double dist;         /* distance between joints */
  double velocity_max; /* maximum of the absolute speed */
  double range;        /* delta parameter that cross the distance */
//  double length;        /* used to homogenize lengths and angles */
  p3d_jnt_get_point(jntPt, &(data->p));
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    //    wmax_rel= 1.0/length;
    wmax_rel= 1.0/jntPt->dist;

    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    dist = p3d_point_dist(prev_data->p, data->p);

    data->wmax = prev_data->wmax + wmax_rel;
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
//end path defrom
