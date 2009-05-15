// ---------------------------------------------------------------------
/*!\file p3d_jacobian.c
  \brief jacobian matrix
  \ingroup p3d_utils
  \author Etienne Ferre
  \date may 2001

  functions to calculate jacobian matrix

*/
// ---------------------------------------------------------------------

#include "Util-pkg.h"
#include "P3d-pkg.h"


/*****************************************************************************\
 @ p3d_jacInitialization()
 ----------------------------------------------------------------------------*/
/*!
  \brief       initialization of the pre-jacobian matrix
  \param rob   robot
*/
/*****************************************************************************/
void p3d_jacInitialization(p3d_rob *rob)
{
  int i, j, njnt;
  p3d_jnt *jnt, *prevjnt;
  p3d_prejac *prejac, **prev_prejac;
  njnt = rob->njoints;

  for(i=0; i<njnt + 1; i++) {
    jnt = rob->joints[i];
    prevjnt = jnt;
    prev_prejac = &(jnt->prejac);
    while (prevjnt!=NULL) {
      for(j=prevjnt->dof_equiv_nbr-1; j>=0; j--) {
	prejac = MY_ALLOC(p3d_prejac, 1);
	*prev_prejac = prejac;
	prejac->numder = prevjnt->index_dof+j;
	/* Normalize to avoid the disparity between parameters with angle
	   and parameters with length (change with the environment) */
	if (p3d_jnt_is_dof_angular(prevjnt, j))
	  { prejac->alpha = SQR(prevjnt->dist); }
	else
	  { prejac->alpha = 1.0; }
	prev_prejac = &(prejac->prev_prejac);
      }
      prevjnt = prevjnt->prev_jnt;
    }
    *prev_prejac = NULL;
  }
}

/*****************************************************************************\
 @ p3d_jacDelete()
 ----------------------------------------------------------------------------*/
/*! 
  \brief       destruction of the pre-jacobian matrix
  \param rob   robot
*/ 
/*****************************************************************************/

void p3d_jacDelete(p3d_rob *rob)
{
  int i, njnt;
  p3d_jnt *jnt;
  p3d_prejac *nextprejac, *prejac;
  njnt = rob->njoints;
  for (i=0;i<njnt + 1;i++)
    {
      jnt = rob->joints[i];
      prejac = jnt->prejac->prev_prejac;
      MY_FREE(jnt->prejac, p3d_prejac, 1);
      jnt->prejac = NULL;
      nextprejac = prejac;
      while (prejac)
	{
	  prejac = nextprejac->prev_prejac;
	  MY_FREE(nextprejac, p3d_prejac, 1);
	  nextprejac = prejac;
	}
    }
}


/*****************************************************************************\
 @ p3d_jacUpdatePosition()
 ----------------------------------------------------------------------------*/
/*!
  \brief          update the position of the robot and its prejacobian matrix
  \param robotPt  robot
*/ 
/*****************************************************************************/

int p3d_jacUpdatePosition(pp3d_rob robotPt)
{
  pp3d_jnt j;
  p3d_cntrt *ct;
  int I_can, i;

  /* change la configuration s'il y a des contraintes cinematiques */
  I_can = 1;
  // modif Juan
  if(robotPt->cntrt_manager->cntrts != NULL) {
    for (dbl_list_goto_first(robotPt->cntrt_manager->cntrt_call_list);
	 dbl_list_more(robotPt->cntrt_manager->cntrt_call_list); 
	 dbl_list_next(robotPt->cntrt_manager->cntrt_call_list)) {
      ct = (p3d_cntrt *) DBL_LIST_DATA(void *, robotPt->cntrt_manager->cntrt_call_list);
      if(ct->active)
	// WARNING : don't handle multisol 
	I_can = (*ct->fct_cntrt)(ct,-1,NULL,0.0);
      if(!I_can) return(FALSE);
    }
  }
  // fmodif Juan

  j = robotPt->j_modif;

  if ((j == NULL) || !(j->pos_updated)) { /* Start with the joint 0 */
    j = robotPt->joints[0];
    p3d_jnt_set_mat_pos(j, NULL); 
  }

  p3d_jacUpdateJoint(j);
  
  p3d_BB_update_BB_rob(robotPt);
  
  robotPt->j_modif = NULL;
  for(i=0; i<robotPt->njoints; i++) {
    j = robotPt->joints[i];
    p3d_jnt_clean_update(j);
  }

  return(TRUE);

}

/*****************************************************************************\
 @ p3d_jacSetAndUpdatePosition()
 ----------------------------------------------------------------------------*/
/*!
  \brief       set and update the position of the robot and its prejacobian matrix
  \param robotPt robot 
  \param q       new configuration
 
  \note        angles in radians
*/
/*****************************************************************************/

int  p3d_jacSetAndUpdatePosition(p3d_rob *robotPt, configPt q)
{
  int I_can;
  p3d_set_robot_config(robotPt, q);
  I_can = p3d_jacUpdatePosition(robotPt);
  return I_can;
}

/*****************************************************************************\
 @ p3d_jacUpdateJoint()
 ----------------------------------------------------------------------------*/
/*!
  \brief       update position and prejacobien of a joint and its children
  \param j     joint
 
  \note        angles in radians
*/
/*****************************************************************************/

void p3d_jacUpdateJoint( p3d_jnt *j)
{
  int      i;
  
  if ((j->o!=NULL) && (j->abs_pos_modified))
    { update_robot_obj_pos(j->o); }

  for(i=0;i<j->n_link_jnt;i++) {
    if(p3d_jnt_calc_mat_pos_and_jac(j->link_jnt_arr[i])) {
      if (j == j->link_jnt_arr[i]->prev_jnt)
	{ p3d_jacUpdateJoint(j->link_jnt_arr[i]->next_jnt); }
      else
	{ p3d_jacUpdateJoint(j->link_jnt_arr[i]->prev_jnt); }
    }
  }
}

/*****************************************************************************/

/*****************************************************************************\
 @ p3d_jacMult()
 ----------------------------------------------------------------------------*/
/*!
  \brief jacobian multiplied by a vector (configPt)
  \param P    the point (vector 3x1) where the jacobian is calculated
  \param jnt  the joint where this point is attached
  \param G    configuration vector entry
  \retval F   3x1 vector : F = J x G
*/
/*****************************************************************************/

void  p3d_jacMult(p3d_vector3 P, p3d_jnt *jnt, configPt G, p3d_vector3 F)
{
  p3d_vector3 Jp;
  p3d_prejac *prejac;

  F[0] = F[1] = F[2] = 0.;

  prejac = jnt->prejac;
  while (prejac) {
    p3d_xformPoint(prejac->J, P, Jp);
    F[0] += Jp[0]*G[prejac->numder];
    F[1] += Jp[1]*G[prejac->numder];
    F[2] += Jp[2]*G[prejac->numder];
    prejac = prejac->prev_prejac;
  }
}


/*****************************************************************************\
 @ p3d_jacTransposeMult()
 ----------------------------------------------------------------------------*/
/*!
  \brief jacobian transpose multiplied by a vector (3x3)
  \param P   the point (vector 3x1) where the jacobian is calculated
  \param jnt the joint where this point is attached
  \param F   3x1 vector entry
  \retval G  configuration vector G = Jt x F
*/ 
/*****************************************************************************/

void  p3d_jacTransposeMult(p3d_vector3 P, p3d_jnt *jnt, p3d_vector3 F, configPt G)
{
  p3d_vector3 Jp;
  int i;
  p3d_prejac *prejac;

  for (i=0;i<jnt->rob->nb_dof;i++)
    { G[i] = 0; }

  prejac = jnt->prejac;
  while (prejac)
    {
      p3d_xformPoint(prejac->J, P, Jp);
      G[prejac->numder] = p3d_vectDotProd(Jp, F) / prejac->alpha;
      prejac = prejac->prev_prejac;
    }
}

/*****************************************************************************\
 @ p3d_jacPseudoInvMult()
 ----------------------------------------------------------------------------*/
/*!
  \brief jacobian pseudoinverse multiplied by a vector (3x1)
  \param P   the point (vector 3x1) where the jacobian is calculated
  \param jnt the joint where this point is attached
  \param F   3x1 vector entry
  \retval G  configuration vector G = Jt x F
  \return -1 if pseudoinverse doesn't exist else 0
*/ 
/*****************************************************************************/ 

int  p3d_jacPseudoInvMult(p3d_vector3 P, p3d_jnt *jnt, p3d_vector3 F, configPt G)
{
  p3d_matrix3 M;
  p3d_vector3 Jp, F2;
  p3d_prejac *prejac;
  int i, j;

  p3d_mat3Copy(p3d_mat3NULL, M);
  
  prejac = jnt->prejac;
  while (prejac) {
    p3d_xformPoint(prejac->J, P, Jp);
    for (i=0;i<3;i++) {
      for (j=0;j<3;j++)
	{ M[i][j] += Jp[i]*Jp[j] / prejac->alpha; }
    }
    prejac = prejac->prev_prejac;
  }
  if (p3d_mat3Invert(M, M)) {
    PrintInfo(("matrice non-inversible\n"));
    return -1;
  }
  p3d_vec3Mat3Mult(M, F, F2);
  p3d_jacTransposeMult(P, jnt, F2, G);
  return 0;
}


/***********************************************/


//------------------------------------
//! Jacobienne d'une Matrice de rotation d'angle t autour de l'axe p
//------------------------------------
void jac_rot(p3d_matrix4 M, p3d_vector3 p, double t)
{double norm, dc, ds, dv;
 double x, y, z;

 x = p[0];
 y = p[1];
 z = p[2];

 p3d_mat4Copy(p3d_mat4NULL,M);

 norm = p3d_vectNorm(p);
 if (norm == 0.0) {
   return;
 }
	
 x /= norm;
 y /= norm;
 z /= norm;

 dc = -sin(t);
 ds = cos(t);
 dv = - dc;

 M[0][0] = x*x*dv + dc;
 M[1][0] = x*y*dv + z*ds;
 M[2][0] = x*z*dv - y*ds;
 M[0][1] = x*y*dv - z*ds;
 M[1][1] = y*y*dv + dc;
 M[2][1] = y*z*dv + x*ds;
 M[0][2] = x*z*dv + y*ds;
 M[1][2] = y*z*dv - x*ds;
 M[2][2] = z*z*dv + dc;
	
}

/*****************************************************************************\
 @ static void jac_rot_trans4_rx(p3d_matrix4 M, double rx, double ry, double rz)
 ----------------------------------------------------------------------------*/
/*!
  \brief derivate along rx rotation translation matrix calculated by rot_trans4()

  see rot_trans4() documentation for parameters description

  \sa jac_rot_trans4_ry(),jac_rot_trans4_rz()

*/ 
/*****************************************************************************/ 


void jac_rot_trans4_rx(p3d_matrix4 M, double rx, double ry, double rz)
{
  double t1 = cos(rz);
  double t2 = cos(ry);
  double t4 = sin(rz);
  double dt5 = -sin(rx);
  double t7 = sin(ry);
  double t8 = t1*t7;
  double dt9 = cos(rx);
  double t17 = t4*t7;

  M[0][0] = 0;
  M[0][1] = -t4*dt5+t8*dt9;
  M[0][2] = t4*dt9+t8*dt5;
  M[0][3] = 0;

  M[1][0] = 0;
  M[1][1] = t1*dt5+t17*dt9;
  M[1][2] = -t1*dt9+t17*dt5;
  M[1][3] = 0;

  M[2][0] = 0;
  M[2][1] = t2*dt9;
  M[2][2] = t2*dt5;
  M[2][3] = 0;

  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 0;
}

/*****************************************************************************\
 @ static void jac_rot_trans4_ry(p3d_matrix4 M, double rx, double ry, double rz)
 ----------------------------------------------------------------------------*/
/*!
  \brief derivate along ry rotation translation matrix calculated by rot_trans4()

  see rot_trans4() documentation for parameters description

  \sa jac_rot_trans4_rx(),jac_rot_trans4_rz()

*/ 
/*****************************************************************************/ 


void jac_rot_trans4_ry(p3d_matrix4 M, double rx, double ry, double rz)
{
  double t1 = cos(rz);
  double dt2 = -sin(ry);
  double t4 = sin(rz);
  double t5 = cos(rx);
  double dt7 = cos(ry);
  double dt8 = t1*dt7;
  double t9 = sin(rx);
  double dt17 = t4*dt7;

  M[0][0] = t1*dt2;
  M[0][1] = dt8*t9;
  M[0][2] = dt8*t5;
  M[0][3] = 0;

  M[1][0] = t4*dt2;
  M[1][1] = dt17*t9;
  M[1][2] = dt17*t5;
  M[1][3] = 0;

  M[2][0] = -dt7;
  M[2][1] = dt2*t9;
  M[2][2] = dt2*t5;
  M[2][3] = 0;

  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 0;
}

/*****************************************************************************\
 @ static void jac_rot_trans4_rz(p3d_matrix4 M, double rx, double ry, double rz)
 ----------------------------------------------------------------------------*/
/*!
  \brief derivate along rz rotation translation matrix calculated by rot_trans4()

  see rot_trans4() documentation for parameters description

  \sa jac_rot_trans4_rx(),jac_rot_trans4_ry()

*/ 
/*****************************************************************************/ 


void jac_rot_trans4_rz(p3d_matrix4 M, double rx, double ry, double rz)
{
  double dt1 = -sin(rz);
  double t2 = cos(ry);
  double dt4 = cos(rz);
  double t5 = cos(rx);
  double t7 = sin(ry);
  double dt8 = dt1*t7;
  double t9 = sin(rx);
  double dt17 = dt4*t7;

  M[0][0] = dt1*t2;
  M[0][1] = -dt4*t5+dt8*t9;
  M[0][2] = dt4*t9+dt8*t5;
  M[0][3] = 0;

  M[1][0] = dt4*t2;
  M[1][1] = dt1*t5+dt17*t9;
  M[1][2] = -dt1*t9+dt17*t5;
  M[1][3] = 0;

  M[2][0] = 0;
  M[2][1] = 0;
  M[2][2] = 0;
  M[2][3] = 0;

  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 0;
}

