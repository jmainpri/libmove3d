/****************************************************************************/
/*!
 *  \file p3d_jnt_base.c
 *
 *  \brief Description of the ::P3D_BASE joint.
 *
 *       It's the first complex joint in Move3D. It's a freeflyer joint:
 * 3 translations (x, y, z), 3 rotations (Rx, Ry, Rz). It is used for 
 * compatibility resons. With it the joint 0 could be used as a classical
 * complex joint and doesn't need specific treatement. This joint is bound
 * to become a placement joint. It's default orientation couldn't be change
 * (\a pos in p3d_jnt_base_create() as no effect on the orrientation).
 */

#include "P3d-pkg.h"
#include "Util-pkg.h"

/*! \brief Default name for a joint base.
 *  \internal
 */
static const char * name_dof_base[] = { "X", "Y", "Z", "Rx", "Ry", "Rz" };

/*--------------------------------------------------------------------------*/
/*!
 * \brief Get the name of a degree of freedom for a base joint
 *
 *  This is the implementation of p3d_jnt_get_dof_name() for the joint
 *  ::P3D_BASE.
 *
 *  \param i_dof:  the index of the degree of freedom 
 *
 *  \return The name of the degree of freedom
 *
 *  \internal
 */
const char * p3d_jnt_base_get_dof_name(int i_dof)
{
  if ((i_dof>=0) && (i_dof<6))
    { return name_dof_base[i_dof]; }
  return NULL;
}


/***************************************************************************
 * Function to change the parameters of the degree of freedom
 */

/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of a degree of freedom for the given joint
 *
 * Note: this function doesn't check if the value is valid
 *       (between the bounds).
 *
 *  This is the implementation of p3d_jnt_set_dof() for the joint
 *  ::P3D_BASE. It allows compatibilities with previous definition of joint 0.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param val:    the new value for this degree of freedom
 *                 (expressed in radian for angles)
 *
 *  \internal
 */
void p3d_jnt_base_set_dof(p3d_jnt * jntPt, int i_dof, double val)
{
  jntPt->dof_data[i_dof].v = val;
  /* compatibilite*/
  if (jntPt->rob != NULL) {
    switch(i_dof) {
    case 0:
      jntPt->p0.x = val;
      break;
    case 1:
      jntPt->p0.y = val;
      break;
    case 2:
      jntPt->p0.z = val;
      break;
    case 3:
      jntPt->rob->rx = val;
      break;
    case 4:
      jntPt->rob->ry = val;
      break;
    case 5:
      jntPt->rob->rz = jntPt->v = val;
      break;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Change the value of the bounds for a given joint and
 *        degree of freedom
 *
 * Note: this function doesn't check if the values are valid
 *       (vmin<vmax).
 *
 *  This is the implementation of p3d_jnt_set_dof_bounds() for the joint
 *  ::P3D_BASE. This allows compatibility with previous definition of joint 0.
 *
 *  \param jntPt:  the joint
 *  \param i_dof:  the index of the degree of freedom
 *  \param vmin:   the new minimum bound for this degree of freedom
 *                 (expressed in radian for angles)
 *  \param vmax:   the new maximum bound for this degree of freedom
 *                 (expressed in radian for angles)
 *
 *  \internal
 */
void p3d_jnt_base_set_dof_bounds(p3d_jnt * jntPt, int i_dof, 
				 double vmin, double vmax)
{
  jntPt->dof_data[i_dof].vmin = vmin;
  jntPt->dof_data[i_dof].vmax = vmax;
  /* compatibilite*/
  if (jntPt->rob!=NULL) {
    switch(i_dof) {
    case 0:
      jntPt->rob->box.x1 = vmin;
      jntPt->rob->box.x2 = vmax;
      break;
    case 1:
      jntPt->rob->box.y1 = vmin;
      jntPt->rob->box.y2 = vmax;
      break;
    case 2:
      jntPt->rob->box.z1 = vmin;
      jntPt->rob->box.z2 = vmax;
      break;
    case 3:
      jntPt->rob->vmin_rot[0] = vmin;
      jntPt->rob->vmax_rot[0] = vmax;
      break;
    case 4:
      jntPt->rob->vmin_rot[1] = vmin;
      jntPt->rob->vmax_rot[1] = vmax;
      break;
    case 5:
      jntPt->rob->vmin_rot[2] = jntPt->vmin = vmin;
      jntPt->rob->vmax_rot[2] = jntPt->vmax = vmax;
      break;
    }
  }
}


/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *
 *  From the configuration q_init of the joint to q_max_param, 
 *  this function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_dist() for the joint
 *  ::P3D_BASE.
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
void p3d_jnt_base_stay_within_dist(p3d_stay_within_dist_data * prev_data,
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
  int i, index_dof;

  p3d_jnt_get_point(jntPt, &(data->p));
  
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    index_dof = jntPt->index_dof;
    /* the joint is constrained in an interval */
    vmax_rel = sqrt(SQR(q_max_param[index_dof]   - q_init[index_dof])+
		    SQR(q_max_param[index_dof+1] - q_init[index_dof+1])+
		    SQR(q_max_param[index_dof+2] - q_init[index_dof+2])
		    ) / max_param;
    wmax_rel = 0.;
    for(i=3; i<6; i++) {
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
    if (prev_data->wmax < EPS6)
      { dist = 0; }/* We don't need to compute this distance */
    else {
      p_min.x = data->p.x + q_init[index_dof];
      p_min.y = data->p.y + q_init[index_dof+1];
      p_min.z = data->p.z + q_init[index_dof+2];

      p_max.x = data->p.x + q_init[index_dof] + (*reach_param) *
	(q_max_param[index_dof] - q_init[index_dof]) / max_param;
      p_max.y = data->p.y + q_init[index_dof+1] + (*reach_param) *
	(q_max_param[index_dof+1] - q_init[index_dof+1]) / max_param;
      p_max.z = data->p.z + q_init[index_dof+2] + (*reach_param) *
	(q_max_param[index_dof+2] - q_init[index_dof+2]) / max_param;
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
 *  by p3d_jnt_set_dof() and the position matrix of the previous joint.
 *  This is the implementation of p3d_jnt_calc_mat_pos() for ::P3D_BASE
 *  joint.
 *
 * \param jntPt:  the joint
 *
 *  \internal
 */
void p3d_jnt_base_calc_dof(p3d_jnt * jntPt)
{
  int i;
  double dof[6], vmin, vmax;

  p3d_mat4ExtractPosReverseOrder(jntPt->abs_pos, &(dof[0]), &(dof[1]),
				 &(dof[2]), &(dof[3]), &(dof[4]), &(dof[5]));
  p3d_mat4Copy(jntPt->abs_pos, jntPt->jnt_mat);
  for(i=0; i<6; i++) {
    p3d_jnt_get_dof_bounds(jntPt, i, &vmin, &vmax);
    if (vmin == vmax) 
      { p3d_jnt_set_dof_bounds(jntPt, i, dof[i], dof[i]); }
    p3d_jnt_set_dof(jntPt, i, dof[i]); 
  }
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_BASE with angle in radian
 *
 * Note: For compatibility reason the orientation of the joint 
 *       couldn't be change by \a pos.
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
p3d_jnt * p3d_jnt_base_create(p3d_matrix4 pos, double * v, 
			      double * vmin, double * vmax, 
			      double * vmin_rand, double * vmax_rand,
			      double * param)
{
  p3d_jnt * jntPt;

  p3d_mat4Copy(p3d_mat4IDENTITY, pos);
  jntPt = p3d_jnt_freeflyer_create(pos, v, vmin, vmax, 
				   vmin_rand, vmax_rand, param);
  if (jntPt == NULL) {
    PrintError(("p3d_jnt_base_create: Cannot create the joint !!!\n"));
    return NULL;
  }
  jntPt->type               = P3D_BASE;

  return jntPt;
}


/*--------------------------------------------------------------------------*/
/*!
 * \brief Create a new joint ::P3D_BASE with angle in degree
 *
 * Note: For compatibility reason the orientation of the joint 
 *       couldn't be change by \a pos.
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
p3d_jnt * p3d_jnt_base_create_deg(p3d_matrix4 pos, double * v, 
		  double * vmin, double * vmax, 
		  double * vmin_rand, double * vmax_rand, double * param)
{
  int i;

  for(i=3; i<6; i++) {
    v[i] = DTOR(v[i]);
    vmin[i] = DTOR(vmin[i]);
    vmax[i] = DTOR(vmax[i]);
    vmin_rand[i] = DTOR(vmin_rand[i]);
    vmax_rand[i] = DTOR(vmax_rand[i]); 
  }
  return p3d_jnt_base_create(pos, v, vmin, vmax, vmin_rand, vmax_rand,
			     param);
}

//start path deform
/*--------------------------------------------------------------------------*/
/*!
 *  \brief Compute the distance and the speed that the joint could reach
 *  
 *
 *  This function computes an interval of parameter which all the points
 *  of the joint move by less than the distance given as input.
 *  This is the implementation of p3d_jnt_stay_within_sphere() for the joint
 *  ::P3D_BASE.
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
void p3d_jnt_base_stay_within_sphere(p3d_stay_within_dist_data * prev_data,
             p3d_jnt * jntPt,
             p3d_stay_within_dist_data * data,
             double * distance, 
             double * reach_param){
//  double velocity_max;         /* maximum of the absolute speed */
//  double range;                /* delta parameter that cross the distance */
//  double length;               /* used to homogenize lengths and angles */
  p3d_jnt_get_point(jntPt, &(data->p));
  
  if (*reach_param<EPS6) {
    data->vmax = prev_data->vmax;
    data->wmax = prev_data->wmax;
  } else {
    data->vmax = 0.;
    data->wmax = 0.0;
     }
}
//end path deform
