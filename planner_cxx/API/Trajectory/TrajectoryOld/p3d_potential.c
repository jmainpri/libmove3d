/* ---------------------------------------------------------------------*/
/*!\file p3d_potential.c
 * \brief   gradient descent algorithm
 * \ingroup planner
 *
 * \author E.Ferre
 * \date   Aug 2001
 *
 * Set of functions for gradient calculation, projection on 
 * the configuration space and gradient descent
 *
 */
/* ---------------------------------------------------------------------*/

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Planner-pkg.h"

/************************************************************************/
/*!\fn static void gradientProject(p3d_rob *rob, double d0, configPt Gt)
 * \brief project the gradient on the free configuration space
 *
 * \param rob the robot
 * \param d0  security distance from the obstacle
 *
 * \retval Gt  the gradient
 *
 * \note It is not an exact projection. Projection is done for one body.
 * The result gradient is projected for the next body and so on.\n
 * the result q + Gt could be in collision.
 *
 * \warning position and jacobian must be up-to-date
 *
 */
/************************************************************************/

static void gradientProject(p3d_rob *rob, double d0, configPt Gt)
{
  p3d_vector3 *r = MY_ALLOC(p3d_vector3, rob->no);
  p3d_vector3 *s = MY_ALLOC(p3d_vector3, rob->no);
  double *distances = MY_ALLOC(double, rob->no);
  p3d_matrix4 M;
  p3d_vector3 F, P;
  p3d_jnt *jnt;
  configPt G = p3d_alloc_config(rob);
  int i, j, i_dof, k;
  double alpha, normG, d, dmin = P3D_HUGE;
  p3d_jnt * jntPt;

  // calculate exact distances and contact points
  
  p3d_col_test_robot(rob, DISTANCE_EXACT);
  p3d_col_report_closest_points(rob, r, s, distances);

  for (i=0;i<rob->no;i++)
    {
      // project Gt on the tangent plane of the nearest obstacle
      if (distances[i]<P3D_HUGE)
	{
	  d = distances[i];
	  if (d<dmin)
	    dmin = d;
	  jnt = rob->o[i]->jnt;
	  p3d_matInvertXform(jnt->abs_pos, M);
	  p3d_xformPoint(M, r[i], P);
	  p3d_vectSub(s[i], r[i], F);
	  p3d_vectNormalize(F, F);
	  p3d_jacTransposeMult(P, jnt, F, G);
	  alpha = 0;
	  normG = 0;
	  for (j=0;j<=rob->njoints;j++) {
	    jntPt =rob->joints[j];
	    for(i_dof = 0; i_dof<jntPt->dof_equiv_nbr; i_dof++) {
	      k = jntPt->index_dof+i_dof;
	      /* Normalize to avoid the disparity between parameters
		 with angle and parameters with length (change with
		 the environment) */
	      if (p3d_jnt_is_dof_angular(jntPt, i)) {
		alpha += Gt[k]*G[k] * SQR(jntPt->dist);
		normG +=  G[k]*G[k] * SQR(jntPt->dist);
	      } else {
		alpha += Gt[k]*G[k];
		normG +=  G[k]*G[k];
	      }
	    }
	  }
	  if ((alpha > d-d0)&&(normG>EPS6))
	    {
	      for (k=0;k<rob->nb_dof;k++)
		Gt[k] -= ((alpha - d + d0)/normG) * G[k];
	    }			
    	}
    }
  p3d_destroy_config(rob, G);
  MY_FREE(r, p3d_vector3, rob->no);
  MY_FREE(s, p3d_vector3, rob->no);
  MY_FREE(distances, double, rob->no);
}

/************************************************************************/
/*!\fn static void gradientCalculate(p3d_rob *rob, p3d_strippoint *sp, configPt G)
 * \brief calculate the gradient for the elastic strip optimization
 *
 * \param rob the robot
 * \param sp  the strippoint
 *
 * \retval G   the resulting gradient
 *
 * the gradient is calculated to transform the configuration to another places
 * on the local path linking sp->prev and sp->next at the distance 
 * sp->Kes * pathlength
 * 
 * \warning G must be allocated
 *
 */
/************************************************************************/

static void gradientCalculate(p3d_rob *rob, p3d_strippoint *sp, configPt G)
{
  p3d_localpath *path = p3d_local_planner(rob, sp->prev->q, sp->next->q);
  configPt q = path->config_at_distance(rob, path, sp->Kes * path->length_lp);
  if (!q)
    q = p3d_copy_config(rob, sp->q);
  p3d_subConfig(rob, sp->q, q, G);
  path->destroy(rob,path);
  p3d_destroy_config(rob, q);
}

/************************************************************************/
/*!\fn int p3d_projectedGradientDescent(p3d_rob *rob,  p3d_strippoint *sp, double d0, configPt q, int *ntest) 
 * \brief calculate the new strippoint configuration with a gradient descent method
 *
 * \param rob the robot
 * \param sp the strippoint
 * \param d0 minimal distance from obstacles
 *
 * \retval q     the new config.(must be allocated)
 * \retval ntest number of collision test is added
 *
 * \return  TRUE if q is different from sp->q
 * 
 */
/************************************************************************/

int p3d_projectedGradientDescent(p3d_rob *rob,  p3d_strippoint *sp, double d0, configPt q, int *ntest)
{
  configPt Gt = p3d_alloc_config(rob);
  configPt qg = p3d_alloc_config(rob);
  double Kpath = 0, dmax;
  int microcol = p3d_col_get_microcollision(); 
  p3d_localplanner_type planner = p3d_local_get_planner();
  p3d_localpath *path = NULL;
  

  p3d_jacSetAndUpdatePosition(rob, sp->q);
  
  gradientCalculate(rob, sp, Gt);
  gradientProject(rob, d0, Gt);
    
  p3d_addConfig(rob, sp->q, Gt, qg);
  p3d_destroy_config (rob, Gt);
  p3d_set_robot_in_joint_limits(rob, qg);
  if (rob->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(rob, qg);
    p3d_get_robot_config_into(rob, &qg);
  }
  if (p3d_equal_config(rob, qg, sp->q))
    {
      p3d_destroy_config(rob,qg);
      p3d_copy_config_into(rob, sp->q, &q);
      return FALSE;
    }
  p3d_local_set_planner(P3D_LINEAR_PLANNER);
  if (!(path = p3d_local_planner(rob, sp->q, qg)))
    {
      p3d_local_set_planner(planner);
      p3d_destroy_config(rob,qg);
      p3d_copy_config_into(rob, sp->q, &q);
      return FALSE;
    }
  
  p3d_col_get_dmax(&dmax);
  p3d_col_set_dmax(d0*0.5);
  p3d_col_set_microcollision(TRUE);
  p3d_unvalid_localpath_classic_test(rob, path, ntest, &Kpath, &qg);  // <- modif Juan
  p3d_col_set_microcollision(microcol);
  p3d_col_set_dmax(dmax);

  p3d_destroy_config(rob,qg);  
  qg = path->config_at_param(rob, path, Kpath*path->range_param);
  if (rob->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(rob, qg);
    p3d_get_robot_config_into(rob, &qg);
  }
  path->destroy(rob,path);
  p3d_local_set_planner(planner);
  
  p3d_copy_config_into(rob, qg, &q);
  p3d_destroy_config(rob, qg);


  return (Kpath > 0);
  //return (!p3d_equal_config(rob, q, sp->q));
}

/************************************************************************/
/*!\fn p3d_quickGradientDescent(p3d_rob *rob,  p3d_strippoint *sp, double d0, configPt q, int *ntest)
 * \brief descend the gradient without projection
 * 
 * \param rob   the robot
 * \param sp    the strippoint
 * \param d0    minimal distance from obstacles
 *
 * \retval q     the new config.(must be allocated)
 * \retval ntest number of collision test is added
 *
 * \return TRUE if q is different from sp->q
 *
 */
/************************************************************************/

int p3d_quickGradientDescent(p3d_rob *rob,  p3d_strippoint *sp, double d0, configPt q, int *ntest)
{
  double Kpath = 0; 
  double dmax;
  int microcol = p3d_col_get_microcollision();
  p3d_localplanner_type planner = p3d_local_get_planner();
  p3d_localpath *path = p3d_local_planner(rob, sp->prev->q, sp->next->q);
  configPt qg = path->config_at_distance(rob, path, sp->Kes * path->length_lp);
  path->destroy(rob, path); 
  if (!qg)
    {
      p3d_copy_config_into(rob, sp->q, &q);
      return FALSE;
    }
  if (p3d_equal_config(rob, qg, sp->q))
    {
      p3d_destroy_config(rob,qg);
      p3d_copy_config_into(rob, sp->q, &q);
      return FALSE;
    }
  p3d_local_set_planner(P3D_LINEAR_PLANNER);
  if (!(path = p3d_local_planner(rob, sp->q, qg)))
    {
      p3d_local_set_planner(planner);
      p3d_destroy_config(rob,qg);
      p3d_copy_config_into(rob, sp->q, &q);
      return FALSE;
    }
  
  p3d_col_get_dmax(&dmax);
  p3d_col_set_dmax(d0);
  p3d_col_set_microcollision(TRUE);
  p3d_unvalid_localpath_classic_test(rob, path, ntest, &Kpath, &qg);  // <- modif Juan
  p3d_col_set_microcollision(microcol);
  p3d_col_set_dmax(dmax);

  p3d_destroy_config(rob,qg);
  qg = path->config_at_param(rob, path, Kpath*path->range_param);
  if (rob->cntrt_manager->cntrts != NULL) {
    p3d_set_and_update_this_robot_conf(rob, qg);
    p3d_get_robot_config_into(rob, &qg);
  }
  path->destroy(rob,path);
  p3d_local_set_planner(planner);
  p3d_copy_config_into(rob, qg, &q);
  p3d_destroy_config(rob, qg);
  return (!p3d_equal_config(rob, q, sp->q));
}
  
