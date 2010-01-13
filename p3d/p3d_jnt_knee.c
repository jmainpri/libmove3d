/****************************************************************************/
/*!
 *  \file p3d_jnt_knee.c
 *
 *  \brief Description of the ::P3D_KNEE joint.
 *
 *       It's a complex joint. It owns:
 * 3 rotation (Rx, Ry, Rz).
 * It's default orientation could be modify by \a pos in
 * p3d_jnt_knee_create().
 */

#include "P3d-pkg.h"
#include "Util-pkg.h"

/*! \brief Default name for a knee joint
 *  \internal
 */
static const char * name_dof_knee[] = { "Rx", "Ry", "Rz" };

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the name of a degree of freedom for a knee joint
 *
 *  This is the implementation of p3d_jnt_get_dof_name() for the joint
 *  ::P3D_KNEE.
 *
 *  \param i_dof:  the index of the degree of freedom 
 *
 *  \return The name of the degree of freedom
 *
 *  \internal
 */
const char * p3d_jnt_knee_get_dof_name(int i_dof)
{
  if ((i_dof>=0) && (i_dof<3))
    { return name_dof_knee[i_dof]; }
  return NULL;
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  From the configuration q_init of the joint to q_max_param, 
 *  this function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_dist() for the joint
 *  ::P3D_KNEE.
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
void p3d_jnt_knee_stay_within_dist(p3d_stay_within_dist_data * prev_data,
				   p3d_jnt * jntPt,
				   p3d_stay_within_dist_data * data,
				   double * distance, configPt q_init, 
				   configPt q_max_param, 
				   double max_param, double * reach_param)
{
  double vmax_rel, wmax_rel;   /* relative speed */
  double dist;                 /* distance between joints */
  double velocity_max;         /* maximum of the absolute speed */
  double range;                /* delta parameter that cross the distance */
  int index_dof, i;

  p3d_jnt_get_point(jntPt, &(data->p));
  
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    index_dof = jntPt->index_dof;
    /* the joint is constrained in an interval */
    vmax_rel = 0.;
    wmax_rel = 0.;
    for(i=0; i<3; i++) {
      if (p3d_jnt_is_dof_circular(jntPt, i)) {
	/* the joint can rotate freely */
	wmax_rel += fabs(dist_circle(q_max_param[index_dof+i],
				     q_init[index_dof+i]) / max_param);
      } else {
	wmax_rel += fabs((q_max_param[index_dof+i] - 
			  q_init[index_dof+i]) / max_param);
      }
    }
    
    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    dist = p3d_point_dist(prev_data->p, data->p);
    
    data->wmax = prev_data->wmax + wmax_rel;
    data->vmax = prev_data->vmax + dist*(prev_data->wmax) + vmax_rel;
    
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
 *  \brief Compute the transformation matrix of the knee joint
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof().
 *  This is used in p3d_jnt_calc_mat_pos() for ::P3D_KNEE
 *  joint.
 *
 *  \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_knee_calc_jnt_mat(p3d_jnt * jntPt)
{
  p3d_mat4PosReverseOrder(jntPt->jnt_mat, 0., 0., 0.,
			  jntPt->dof_data[0].v, jntPt->dof_data[1].v,
			  jntPt->dof_data[2].v); 
}




/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the jacobian matrix of the joint
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof(), the position matrix and the prejacobian
 *  matrix of the previous joint.
 *  This is used in p3d_jnt_calc_mat_pos() for ::P3D_KNEE
 *  joint.
 *
 * \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_knee_calc_mat_jac(p3d_jnt * jntPt)
{
  p3d_jnt * jnt_prevPt;
  p3d_link_between_joint * linkPt = NULL;
  p3d_matrix4 m_jac_j, tmp_mat;
  p3d_prejac *prevprejac, *prejac;
  double rx, ry, rz;

  /* update prejacobian matrix */

  jnt_prevPt = jntPt->prev_jnt;

  prejac = jntPt->prejac;
  if (!prejac) {
    PrintError(("error jacobian : jacobian not initialized\n"));
    return;
  }

  rx = jntPt->dof_data[0].v;
  ry = jntPt->dof_data[1].v;
  rz = jntPt->dof_data[2].v;

  if (jntPt->dof_data[2].vmin<jntPt->dof_data[2].vmax)
    { jac_rot_trans4_rz(m_jac_j, rx, ry, rz); }
  else
    { p3d_mat4Copy(p3d_mat4NULL, m_jac_j); }
  p3d_matMultXform(jntPt->abs_pos_before_jnt, m_jac_j, prejac->J);
  prejac = prejac->prev_prejac;

  if (jntPt->dof_data[1].vmin<jntPt->dof_data[1].vmax)
    { jac_rot_trans4_ry(m_jac_j, rx, ry, rz); }
  else
    { p3d_mat4Copy(p3d_mat4NULL, m_jac_j); }
  p3d_matMultXform(jntPt->abs_pos_before_jnt, m_jac_j, prejac->J);
  prejac = prejac->prev_prejac;

  if (jntPt->dof_data[0].vmin<jntPt->dof_data[0].vmax)
    { jac_rot_trans4_rx(m_jac_j, rx, ry, rz); }
  else
    { p3d_mat4Copy(p3d_mat4NULL, m_jac_j); }
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
 * \brief Create a new joint ::P3D_KNEE with angle in radian
 *
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint 
 *               (with angle in radian)
 * \param  vmin & vmax:  the bounds values of 
 *                       the degree of freedom for the joint
 *                       (with angle in radian)
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of 
 *                                the degree of freedom for the joint
 *                                (with angle in radian)
 * \param  param: the array of the parameters for the joint (NULL here)
 *
 * \return the new joint.
 *
 *  \internal
 */
p3d_jnt * p3d_jnt_knee_create(p3d_matrix4 pos, double * v, 
		double * vmin, double * vmax, 
		double * vmin_rand, double * vmax_rand, double * param)
{
  p3d_jnt * jntPt;
  int i;

  for(i=0; i<3; i++) {
    if((v[i]<vmin[i])||(v[i]>vmax[i])) {
      PrintError(("p3d_jnt_plan_create: initial value is out of range !!!\n"));
      return NULL;
    }
  }
  jntPt = p3d_jnt_create_common(pos);
  if(jntPt == NULL)
    { return NULL; }

  jntPt->type        = P3D_KNEE;

  jntPt->dof_equiv_nbr =  3;

  /* The rotation of the x axis is the first columns of pos matrix */
  jntPt->dof_data[0].axis[0] = pos[0][0]; 
  jntPt->dof_data[0].axis[1] = pos[1][0]; 
  jntPt->dof_data[0].axis[2] = pos[2][0];

  /* The rotation of the y axis is the second columns of pos matrix */
  jntPt->dof_data[1].axis[0] = pos[0][1]; 
  jntPt->dof_data[1].axis[1] = pos[1][1]; 
  jntPt->dof_data[1].axis[2] = pos[2][1];

  /* The rotation of the z axis is the third columns of pos matrix */
  jntPt->dof_data[2].axis[0] = pos[0][2]; 
  jntPt->dof_data[2].axis[1] = pos[1][2]; 
  jntPt->dof_data[2].axis[2] = pos[2][2];

  for(i=0; i<3; i++) {
    jntPt->dof_data[i].old_v = P3D_HUGE;
    p3d_jnt_set_dof(jntPt, i, v[i]); 
    p3d_jnt_set_dof_v0(jntPt, i, 0); 
    p3d_jnt_set_dof_bounds(jntPt, i, vmin[i], vmax[i]); 
    p3d_jnt_set_dof_rand_bounds(jntPt, i, vmin_rand[i], vmax_rand[i]); 
    if (LEQ(vmax[i], vmin[i]))
      { jntPt->dof_data[i].is_user = FALSE; }
    else {
      jntPt->dof_data[i].is_user = TRUE;
      jntPt->user_dof_equiv_nbr ++;
    }
  }
  p3d_jnt_knee_calc_jnt_mat(jntPt);
  p3d_matMultXform(jntPt->abs_pos_before_jnt, jntPt->jnt_mat, jntPt->abs_pos);

  return jntPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_KNEE with angle in degree
 *
 * \param  pos:  the position matrix of the joint
 * \param  v:    the values of the degree of freedom for the joint 
 *               (with angle in degree)
 * \param  vmin & vmax:  the bounds values of 
 *                       the degree of freedom for the joint
 *                       (with angle in degree)
 * \param  vmin_rand & vmax_rand: the random (or user) bounds values of 
 *                                the degree of freedom for the joint
 *                                (with angle in degree)
 * \param  param: the array of the parameters for the joint (NULL here)
 *
 * \return the new joint.
 *
 *  \internal
 */
p3d_jnt * p3d_jnt_knee_create_deg(p3d_matrix4 pos, double * v, 
		double * vmin, double * vmax, 
		double * vmin_rand, double * vmax_rand, double * param) 
{
  int i;
  
  for(i=0; i<3; i++) {
    v[i] = DTOR(v[i]);
    vmin[i] = DTOR(vmin[i]);
    vmax[i] = DTOR(vmax[i]);
    vmin_rand[i] = DTOR(vmin_rand[i]);
    vmax_rand[i] = DTOR(vmax_rand[i]); 
  }
  return p3d_jnt_knee_create(pos, v, vmin, vmax, vmin_rand, vmax_rand, param);
}
//start path deform
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_sphere() for the joint
 *  ::P3D_KNEE.
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
void p3d_jnt_knee_stay_within_sphere(p3d_stay_within_dist_data * prev_data,
           p3d_jnt * jntPt,
           p3d_stay_within_dist_data * data,
           double * distance,
           double * reach_param){
  double vmax_rel, wmax_rel;   /* relative speed */
  double dist;                 /* distance between joints */
  double velocity_max;         /* maximum of the absolute speed */
  double range;                /* delta parameter that cross the distance */
//  int i;

  p3d_jnt_get_point(jntPt, &(data->p));

  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  }else{
    vmax_rel = 0.0;
    wmax_rel = 1.0/(jntPt->dist);
    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    dist = p3d_point_dist(prev_data->p, data->p);
    data->wmax = prev_data->wmax + wmax_rel;
    data->vmax = prev_data->vmax + dist*(prev_data->wmax) + vmax_rel;
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

