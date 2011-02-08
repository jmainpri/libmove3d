/****************************************************************************/
/*!
 *  \file p3d_jnt_plan.c
 *
 *  \brief Description of the ::P3D_PLAN joint.
 *
 *       It's a complex joint. It owns:
 * 2 translations (x, y), 1 rotation (Rz). It is ofen use for robot base.
 * It's default orientation could be modify by \a pos in
 * p3d_jnt_plan_create().
 */

#include "P3d-pkg.h"
#include "Util-pkg.h"

/*! \brief Default name for a joint plan.
 *  \internal
 */
static const char * name_dof_plan[] = { "X", "Y", "Rz" };

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the name of a degree of freedom for a plan joint
 *
 *  This is the implementation of p3d_jnt_get_dof_name() for the joint
 *  ::P3D_PLAN.
 *
 *  \param i_dof:  the index of the degree of freedom 
 *
 *  \return The name of the degree of freedom
 *
 *  \internal
 */
const char * p3d_jnt_plan_get_dof_name(int i_dof)
{
  if ((i_dof>=0) && (i_dof<3))
    { return name_dof_plan[i_dof]; }
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
 *  ::P3D_PLAN.
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
void p3d_jnt_plan_stay_within_dist(p3d_stay_within_dist_data * prev_data,
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
  p3d_point p_min, p_max;
  int index_dof;

  p3d_jnt_get_point(jntPt, &(data->p));
  
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    index_dof = jntPt->index_dof;
    /* the joint is constrained in an interval */
    vmax_rel = sqrt(SQR(q_max_param[index_dof]   - q_init[index_dof])+
		    SQR(q_max_param[index_dof+1] - q_init[index_dof+1])
		    ) / max_param;
    wmax_rel = 0.;
    if (p3d_jnt_is_dof_circular(jntPt, 2)) {
      /* the joint can rotate freely */
      wmax_rel += fabs(dist_circle(q_max_param[index_dof+2],
				   q_init[index_dof+2]) / max_param);
    } else {
      wmax_rel += fabs((q_max_param[index_dof+2] - 
			q_init[index_dof+2]) / max_param);
    }
    
    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    if (prev_data->wmax < EPS6)
      { dist = 0; }/* We don't need to compute this distance */
    else {
      p_min.x = data->p.x + q_init[index_dof] * jntPt->dof_data[0].axis[0] +
	q_init[index_dof+1] * jntPt->dof_data[1].axis[0];
      p_min.y = data->p.y + q_init[index_dof] * jntPt->dof_data[0].axis[1] +
	q_init[index_dof+1] * jntPt->dof_data[1].axis[1];
      p_min.z = data->p.z + q_init[index_dof] * jntPt->dof_data[0].axis[2] +
	q_init[index_dof+1] * jntPt->dof_data[1].axis[2];

      p_max.x = data->p.x + 
	(q_init[index_dof] + (q_max_param[index_dof] - q_init[index_dof]) *
	 (*reach_param) / max_param) * jntPt->dof_data[0].axis[0] +
	(q_init[index_dof+1] + (q_max_param[index_dof+1]-q_init[index_dof+1]) *
	 (*reach_param) / max_param) * jntPt->dof_data[1].axis[0];
      p_max.y = data->p.y +
	(q_init[index_dof] + (q_max_param[index_dof] - q_init[index_dof]) *
	 (*reach_param) / max_param) * jntPt->dof_data[0].axis[1] +
	(q_init[index_dof+1] + (q_max_param[index_dof+1]-q_init[index_dof+1]) *
	 (*reach_param) / max_param) * jntPt->dof_data[1].axis[1];
      p_max.z = data->p.z + 
	(q_init[index_dof] + (q_max_param[index_dof] - q_init[index_dof]) *
	 (*reach_param) / max_param) * jntPt->dof_data[0].axis[2] +
	(q_init[index_dof+1] + (q_max_param[index_dof+1]-q_init[index_dof+1]) *
	 (*reach_param) / max_param) * jntPt->dof_data[1].axis[2];
      dist = MAX(p3d_point_dist(prev_data->p, p_min),
		 p3d_point_dist(prev_data->p, p_max));
    }
    
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
 *  \brief Compute the position matrix of the joint
 *
 *  This function use the value for the degree of freedom set in the joint
 *  by p3d_jnt_set_dof().
 *  This is used in p3d_jnt_calc_mat_pos() for ::P3D_PLAN
 *  joint.
 *
 * \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_plan_calc_jnt_mat(p3d_jnt * jntPt)
{
  double cos_v, sin_v;

  cos_v = cos(jntPt->dof_data[2].v);
  sin_v = sin(jntPt->dof_data[2].v);

  jntPt->jnt_mat[0][0] = cos_v;    
  jntPt->jnt_mat[0][1] = - sin_v;
  jntPt->jnt_mat[0][2] = 0.0;
  jntPt->jnt_mat[0][3] = jntPt->dof_data[0].v;
  jntPt->jnt_mat[1][0] = sin_v;
  jntPt->jnt_mat[1][1] = cos_v;
  jntPt->jnt_mat[1][2] = 0.0;
  jntPt->jnt_mat[1][3] = jntPt->dof_data[1].v;
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
 *  This is used in p3d_jnt_calc_mat_pos() for ::P3D_PLAN
 *  joint.
 *
 *  \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_plan_calc_mat_jac(p3d_jnt * jntPt)
{
  p3d_jnt * jnt_prevPt;
  p3d_link_between_joint * linkPt = NULL;
  p3d_matrix4 m_jac_j, tmp_mat;
  p3d_prejac *prevprejac, *prejac;
  double cos_v, sin_v;

  /* update prejacobian matrix */

  prejac = jntPt->prejac;
  if (!prejac) {
    PrintError(("error jacobian : jacobian not initialized\n"));
    return;
  }

  jnt_prevPt = jntPt->prev_jnt;

  p3d_mat4Copy(p3d_mat4NULL, m_jac_j);
  if (jntPt->dof_data[2].vmin<jntPt->dof_data[2].vmax) {
    cos_v = cos(jntPt->dof_data[2].v);
    sin_v = sin(jntPt->dof_data[2].v);

    m_jac_j[0][0] = - sin_v;
    m_jac_j[0][1] = - cos_v;
    m_jac_j[1][0] =   cos_v;
    m_jac_j[1][1] = - sin_v;
  }
  p3d_matMultXform(jntPt->abs_pos_before_jnt, m_jac_j, prejac->J);
  prejac = prejac->prev_prejac;

  p3d_mat4Copy(p3d_mat4NULL, m_jac_j);
  if (jntPt->dof_data[1].vmin<jntPt->dof_data[1].vmax)
    { m_jac_j[1][3] = 1.0; }
  p3d_matMultXform(jntPt->abs_pos_before_jnt, m_jac_j, prejac->J);
  prejac = prejac->prev_prejac;

  p3d_mat4Copy(p3d_mat4NULL, m_jac_j);
  if (jntPt->dof_data[0].vmin<jntPt->dof_data[0].vmax)
    { m_jac_j[0][3] = 1.0; }
  p3d_matMultXform(jntPt->abs_pos_before_jnt, m_jac_j, prejac->J);
  prejac = prejac->prev_prejac;

  /* update the other prejacobian matrix */
  if (jnt_prevPt!=NULL) {
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
 * \brief Create a new joint ::P3D_PLAN with angle in radian
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
 * \internal
 */
p3d_jnt * p3d_jnt_plan_create(p3d_matrix4 pos, double * v, 
		double * vmin, double * vmax, 
		double * vmin_rand, double * vmax_rand, double *velocity_max, double *acceleration_max, double *jerk_max, double * param)
{
  p3d_jnt * jntPt;
  int i;

  for(i=0; i<3; i++) {
    if((v[i]<vmin[i])||(v[i]>vmax[i])) {
      PrintWarning(("p3d_jnt_plan_create: initial value is out of range"));
      return NULL;
    }
  }
  jntPt = p3d_jnt_create_common(pos);
  if(jntPt == NULL)
    { return NULL; }

  jntPt->type        = P3D_PLAN;

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
    jntPt->dof_data[i].velocity_max = velocity_max[i];
    jntPt->dof_data[i].acceleration_max = acceleration_max[i];
    jntPt->dof_data[i].jerk_max = jerk_max[i];
  }
  p3d_jnt_plan_calc_jnt_mat(jntPt);
  p3d_matMultXform(jntPt->abs_pos_before_jnt, jntPt->jnt_mat, jntPt->abs_pos);

  return jntPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_PLAN with angle in degree
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
 * \internal
 */
p3d_jnt * p3d_jnt_plan_create_deg(p3d_matrix4 pos, double * v, 
		double * vmin, double * vmax, 
		double * vmin_rand, double * vmax_rand, double *velocity_max, double *acceleration_max, double *jerk_max, double * param)
{
  v[2] = DTOR(v[2]);
  vmin[2] = DTOR(vmin[2]);
  vmax[2] = DTOR(vmax[2]);
  vmin_rand[2] = DTOR(vmin_rand[2]);
  vmax_rand[2] = DTOR(vmax_rand[2]); 

  return p3d_jnt_plan_create(pos, v, vmin, vmax,  
			     vmin_rand, vmax_rand,  velocity_max, acceleration_max, jerk_max, param);
}

//start path deform
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_sphere() for the joint
 *  ::P3D_PLAN.
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
void p3d_jnt_plan_stay_within_sphere(p3d_stay_within_dist_data * prev_data,
           p3d_jnt * jntPt,
           p3d_stay_within_dist_data * data,
           double * distance,
           double * reach_param){
  double vmax_rel, wmax_rel;   /* relative speed */
  double dist;                 /* distance between joints */
  double velocity_max;         /* maximum of the absolute speed */
  double range;                /* delta parameter that cross the distance */
  p3d_point p_min, p_max;
  double vmin0, vmax0, vmin1, vmax1;
  double vmin, vmax;

  p3d_jnt_get_point(jntPt, &(data->p));
  
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  }else{
    vmax_rel = sqrt(1.0);
    p3d_jnt_get_dof_rand_bounds(jntPt, 2, &vmin, &vmax);
    if(vmin == vmax){
      wmax_rel = 0.0;   
    }else{
      wmax_rel = 1.0/jntPt->dist;
    }

    /* distance between the reference point of the previous body
       and the point the current joint is attached to */
    if (prev_data->wmax < EPS6){
      dist = 0; /* We don't need to compute this distance */
    }else{
      p3d_jnt_get_dof_rand_bounds(jntPt, 0, &vmin0, &vmax0);
      p3d_jnt_get_dof_rand_bounds(jntPt, 1, &vmin1, &vmax1);

      p_min.x = data->p.x + vmin0* jntPt->dof_data[0].axis[0] + vmin1 * jntPt->dof_data[1].axis[0];
      p_min.y = data->p.y + vmin0 * jntPt->dof_data[0].axis[1] + vmin1 * jntPt->dof_data[1].axis[1];
      p_min.z = data->p.z + vmin0 * jntPt->dof_data[0].axis[2] + vmin1 * jntPt->dof_data[1].axis[2];

      p_max.x = data->p.x + vmax0 * jntPt->dof_data[0].axis[0] + vmax1 * jntPt->dof_data[1].axis[0];
      p_max.y = data->p.y + vmax0 * jntPt->dof_data[0].axis[1] + vmax1 * jntPt->dof_data[1].axis[1];
      p_max.z = data->p.z + vmax0 * jntPt->dof_data[0].axis[2] + vmax1 * jntPt->dof_data[1].axis[2];

      dist = MAX(p3d_point_dist(prev_data->p, p_min), p3d_point_dist(prev_data->p, p_max));
    }

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
