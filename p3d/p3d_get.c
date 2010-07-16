/*****************************************************************/
/* Fichier d'utilitaires : fonctions utilisateur                 */
/*****************************************************************/

#include "Util-pkg.h"
#include "P3d-pkg.h"

//#ifdef P3D_PLANNER
//#include "Planner-pkg.h"
//#endif

#ifdef P3D_LOCALPATH
#include "Localpath-pkg.h"
#endif


/**************************************************/
/* Fonction recuperant une donne d'un polyhedre   */
/* In : pointer vers polyhedre                    */
/* Out :    type d'entite (polyh, sphere, box,    */
/*                         cube, cylindre, cone)  */
/**************************************************/
int p3d_get_poly_entity_type(p3d_poly *p)
{
  return p->entity_type;
}

/**************************************************/
/* Fonction recuperant une donne d'un polyhedre   */
/* In : pointer vers polyhedre                    */
/* Out :   pointer vers representation            */
/* exacte (primitive)                             */
/**************************************************/
p3d_primitive *p3d_get_poly_primitive_data(p3d_poly *p)
{
  return p->primitive_data;
}

/**************************************************/
/* Fonction recuperant une donne d'un polyhedre   */
/* In : pointer vers polyhedre                    */
/* Out :  couleur                                 */
/**************************************************/
int p3d_get_poly_color(p3d_poly *p)
{
  return p->color;
}

/***************************************************/
/* Fonction recuperant le ieme sommet du polyhedre */
/* In : le poly                                */
/* Out : le nombre de points                       */
/***************************************************/
void p3d_get_poly_pt(p3d_poly *p,int i, double *x, double *y, double *z)
{
  p3d_get_point_2_d(p->poly,i,x,y,z);
}

/********************************************/
/* Fonction recuperant la box d'un objet     */
/* In : l'objet                             */
/* Out :                                    */
/********************************************/
void p3d_get_box_obj(p3d_obj *o,double *x1,double *x2,double *y1,double *y2,double *z1,double *z2)
{
     *x1 = o->box.x1;
     *x2 = o->box.x2;
     *y1 = o->box.y1;
     *y2 = o->box.y2;
     *z1 = o->box.z1;
     *z2 = o->box.z2;
}

/********************************************/
/* Fonction recuperant la BB d'un objet     */
/* In : l'objet                             */
/* Out :                                    */
/********************************************/
void p3d_get_BB_obj(p3d_obj *o,double *x1,double *x2,double *y1,double *y2,double *z1,double *z2)
{
     *x1 = o->BB.xmin;
     *x2 = o->BB.xmax;
     *y1 = o->BB.ymin;
     *y2 = o->BB.ymax;
     *z1 = o->BB.zmin;
     *z2 = o->BB.zmax;
}

/********************************************/
/* Fonction recuperant la BB d'un objet     */
/* In : l'objet                             */
/* Out :                                    */
/********************************************/
void p3d_get_BB_rob(p3d_rob *r,double *x1,double *x2,double *y1,double *y2,double *z1,double *z2)
{
     *x1 = r->BB.xmin;
     *x2 = r->BB.xmax;
     *y1 = r->BB.ymin;
     *y2 = r->BB.ymax;
     *z1 = r->BB.zmin;
     *z2 = r->BB.zmax;
}

/* new function: Carl 23052001 */
/* ****************************************************** *
 * p3d_get_BB_rob_max_size(p3d_rob *r,double *maxsize)
 * ARGS IN  : r        pointer to robot
 * ARGS OUT : maxsize  maximal value of the lengths
 *                     of the sizes of the BB stored
 *                     with the device
 * ****************************************************** */
void p3d_get_BB_rob_max_size(p3d_rob *r,double *maxsize)
{
  *maxsize = MAX( r->BB.xmax - r->BB.xmin, 
		   MAX( r->BB.ymax - r->BB.ymin,
			r->BB.zmax - r->BB.zmin) );
}

/**************************************************************/
/* Fonction recuperant la taille de la boite englobante       */
/* de l'environnement courant en directions X, Y et Z         */
/* In :                                                       */
/* Out : la taille de la boite                                */
/**************************************************************/
void p3d_get_env_size(double *x,double *y,double *z)
{
  *x = XYZ_ENV->box.x2 - XYZ_ENV->box.x1;
  *y = XYZ_ENV->box.y2 - XYZ_ENV->box.y1;
  *z = XYZ_ENV->box.z2 - XYZ_ENV->box.z1;
}


/**************************************************************/
/* Fonction recuperant la boite englobante de l'environnement */
/* courant                                                    */
/* In :                                                       */
/* Out : les coordonnees de la boite                          */
/**************************************************************/
void p3d_get_env_box(double *x1, double *x2, double *y1, double *y2, double *z1, double *z2)
{pp3d_box b= &XYZ_ENV->box;
  *x1 = b->x1;
  *x2 = b->x2;
  *y1 = b->y1;
  *y2 = b->y2;
  *z1 = b->z1;
  *z2 = b->z2;
}

/* begin modif Carl */
/**********************************************************/
/* Fonction recuperant le pas de discretisation graphique */
/* des trajectoires de l'environnement courant            */
/* In :                                                   */
/* Out : le rayon                                         */
/**********************************************************/
double p3d_get_env_graphic_dmax(void)
{pp3d_env e = (pp3d_env)p3d_get_desc_curid(P3D_ENV);
  return(e->graphic_dmin);
}
/* end modif Carl */

/* begin modif Pepijn */
/*******************************************************/
/* Function who retrieves the general object tolerance */
/* from the current environment                         */
/* In :                                                */
/* Out : le rayon                                      */
/*******************************************************/
double p3d_get_env_object_tolerance(void)
{
 pp3d_env e = (pp3d_env)p3d_get_desc_curid(P3D_ENV);
 return(e->object_tolerance);
}
/* end modif Pepijn */



/*******************************************************/
/* Fonction recuperant le pas de discretisation des    */
/* trajectoires de l'environnement courant             */
/* In :                                                */
/* Out : le rayon                                      */
/*******************************************************/
double p3d_get_env_dmax(void)
{pp3d_env e = (pp3d_env)p3d_get_desc_curid(P3D_ENV);
 return(e->dmax);
}

/************************************************/
/************************************************/
/************************************************/


/**************************************************************/
/* Fonction recuperant le nombre de sommets du ieme polyhedre */
/* de l'obstacle courant                                       */
/* In : le numero                                             */
/* Out : le nombre de points                                  */
/**************************************************************/
int p3d_get_obstacle_npt(int i) 
{
  return(p3d_get_nb_points(XYZ_ENV->ocur->pol[i]->poly));
}

/**************************************************************/
/* Fonction recuperant le nombre de polyhedre de l'obstacle   */
/* courant                                                    */
/* In :                                                       */
/* Out : le nombre de points                                  */
/**************************************************************/
int p3d_get_obstacle_npoly(void) 
{
  return(XYZ_ENV->ocur->np);
}

/***************************************************/
/* Fonction recuperant un sommet du ieme polyhedre */
/*  l'obstacle courant                             */
/* In : les numeros                                */
/* Out : le nombre de points                       */
/***************************************************/
void p3d_get_obstacle_pt(int num,int i, double *x, double *y, double *z)
{
  /*tsiano
 p3d_get_vert_poly(((p3d_poly *) (&XYZ_ENV->ocur->pol[num])),i,x,y,z);*/
  p3d_get_point_2_d(((p3d_poly *) (XYZ_ENV->ocur->pol[num-1]))->poly,i,x,y,z);
}

/**************************************************************/
/* Fonction recuperant la boite englobante de l'obstacle      */
/* courant                                                    */
/* In :                                                       */
/* Out : les coordonnees de la boite                          */
/**************************************************************/
void p3d_get_obstacle_box(double *x1,double *x2,double *y1,double *y2,double *z1,double *z2)
{pp3d_box b = &XYZ_ENV->ocur->box;

  *x1 = b->x1;*x2 = b->x2;*y1 = b->y1;*y2 = b->y2;*z1 = b->z1;*z2 = b->z2;
}

/***********************************************/
/* Fonction recuperant un obstacle par son nom */
/* In :                                        */
/* Out : son nom                               */
/***********************************************/
p3d_obj *p3d_get_obst_by_name(char *name)
{int o,no,i;
 
   o = p3d_get_desc_curnum(P3D_OBSTACLE);
   no= p3d_get_desc_number(P3D_OBSTACLE);
   for(i=0;i<no;i++){
     p3d_sel_desc_num(P3D_OBSTACLE,i);
     if(strcmp(name,XYZ_ENV->ocur->name) == 0){return(XYZ_ENV->ocur);}
   }
   p3d_sel_desc_num(P3D_OBSTACLE,o);
   return(FALSE);
}


/************************************************/
/************************************************/
/************************************************/

/******************************************************/
/* Fonction recuperant le rayon du robot courant      */
/* In :                                               */
/* Out : le rayon                                     */
/******************************************************/
double p3d_get_this_robot_radius(pp3d_rob r)
{
#ifdef P3D_LOCALPATH
 lm_reeds_shepp_str *rs_paramPt=lm_get_reeds_shepp_lm_param(r);
 if (rs_paramPt == NULL){
   return -1;
 }
 return(rs_paramPt->radius);
#endif
}

/******************************************************/
/* Fonction recuperant le rayon du robot courant      */
/* In :                                               */
/* Out : le rayon                                     */
/******************************************************/
double p3d_get_robot_radius(void)
{pp3d_rob r = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
#ifdef P3D_LOCALPATH
 lm_reeds_shepp_str *rs_paramPt=lm_get_reeds_shepp_lm_param(r);
 if (rs_paramPt == NULL){
   return -1;
 }
 return(rs_paramPt->radius);
#endif
}





/*
 * get the current robot box. theta is in radians
 */
void p3d_get_robot_box(double *x1, double *x2, double *y1, double *y2,
		       double *z1,double *z2,double *t1,double *t2,
		       double *u1, double *u2, double *v1, double *v2)
{
  pp3d_box b = &(XYZ_ENV->cur_robot->box);
  pp3d_rob r = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);
  
  *x1 = b->x1;
  *x2 = b->x2;
  *y1 = b->y1;
  *y2 = b->y2;
  *z1 = b->z1;
  *z2 = b->z2;
  
  *t1 = r->vmin_rot[0]; 
  *t2 = r->vmax_rot[0];
  *u1 = r->vmin_rot[1]; 
  *u2 = r->vmax_rot[1];
  *v1 = r->vmin_rot[2]; 
  *v2 = r->vmax_rot[2];
}

/*
 * get the current robot box. theta is converted into degrees
 */

void p3d_get_robot_box_deg(double *x1, double *x2, double *y1, double *y2,
			   double *z1,double *z2,double *t1,double *t2,
			   double *u1, double *u2, double *v1, double *v2)
{
  pp3d_box b = &(XYZ_ENV->cur_robot->box);
  pp3d_rob r = (pp3d_rob)p3d_get_desc_curid(P3D_ROBOT);

  *x1 = b->x1;
  *x2 = b->x2;
  *y1 = b->y1;
  *y2 = b->y2;
  *z1 = b->z1;
  *z2 = b->z2;
  
  *t1 = RTOD(r->vmin_rot[0]); 
  *t2 = RTOD(r->vmax_rot[0]);
  *u1 = RTOD(r->vmin_rot[1]); 
  *u2 = RTOD(r->vmax_rot[1]);
  *v1 = RTOD(r->vmin_rot[2]); 
  *v2 = RTOD(r->vmax_rot[2]);

}


/*--------------------------------------------------------------------------*/
/*! 
 * \brief Get position of current robot
 *
 *  This function use the joint0 as a placement joint.
 *
 *  Note:
 *     - angles are in radian.
 *
 *  \retval q:  the position of the first joint (we use only 6 dof: 
 *              x, y, z, Rx, Ry, Rz).
 */
void p3d_get_robot_pos(double *q)
{
  pp3d_rob r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  p3d_jnt *j = r->joints[0];
  int i;

  for(i=0; i<j->dof_equiv_nbr; i++)
    { q[i] = p3d_jnt_get_dof(j, i); }
}

/*--------------------------------------------------------------------------*/
/*! 
 * \brief Get position of current robot
 *
 *  This function use the joint0 as a placement joint.
 *
 *  Note:
 *     - angles are in degree.
 *
 *  \retval q:  the position of the first joint (we use only 6 dof: 
 *              x, y, z, Rx, Ry, Rz).
 */
void p3d_get_robot_pos_deg(double *q)
{
  pp3d_rob r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  p3d_jnt *j = r->joints[0];
  int i;

  for(i=0; i<j->dof_equiv_nbr; i++)
    { q[i] = p3d_jnt_get_dof_deg(j, i); }
}

/*--------------------------------------------------------------------------*/
/*! 
 * \brief Get position of a given robot
 *
 *  This function use the joint0 as a placement joint.
 *
 *  Note:
 *     - angles are in radian.
 *
 *  \retval q:  the position of the first joint (we use only 6 dof: 
 *              x, y, z, Rx, Ry, Rz).
 */
void p3d_get_robot_pos(p3d_rob * r, double *q)
{
  p3d_jnt *j = r->joints[0];
  int i;
	
  for(i=0; i<j->dof_equiv_nbr; i++)
	{ q[i] = p3d_jnt_get_dof(j, i); }
}


/**********************************************************/
/* Fonction recuperant le nombre d'articulations du robot */
/* courant                                                */
/* In :                                                   */
/* Out : le nombre d'articulations                        */
/**********************************************************/
int p3d_get_robot_njnt(void) 
{
  return(XYZ_ENV->cur_robot->njoints);
}

/**********************************************************/
/* Fonction recuperant le nom       du robot  courant     */
/* In :                                                   */
/* Out : le nombre d'articulations                        */
/**********************************************************/
char *p3d_get_robot_name(void) 
{
  return(XYZ_ENV->cur_robot->name);
}

/***********************************************************/
/* Fonction recuperant le type (rotoide/prismatique) d'une */
/* articulation du robot courant                           */
/* In : le numero de l'articulation                        */
/* Out : le type de l'articulation                         */
/***********************************************************/
int p3d_get_robot_jnt_type(int i) 
{
  return(XYZ_ENV->cur_robot->joints[i]->type);
}

/*
 *  Get the value of a joint of the current robot
 *
 *  Note: Don't use any more this function (use p3d_get_robot_dof or
 *        functions in p3d_joints.c).
 *
 *  Input:  the index of the joint
 *
 *  Output: the value (expressed in radian for angles)
 */

void p3d_get_robot_jnt(int i, double *val)
{
  p3d_jnt *j = XYZ_ENV->cur_robot->joints[i];
  *val = j->dof_data[0].v;
}

/*
 *  Get the value of a joint of the current robot
 *
 *  Note: Don't use any more this function (use p3d_get_robot_dof or
 *        functions in p3d_joints.c).
 *
 *  Input:  the index of the joint
 *
 *  Output: the value (expressed in degree for angles)
 */

void p3d_get_robot_jnt_deg(int i, double *val)
{
  p3d_jnt *j = XYZ_ENV->cur_robot->joints[i];

  if(p3d_jnt_is_dof_angular(j, 0))
    *val = RTOD(j->dof_data[0].v);
  else                      
    *val = j->dof_data[0].v;
}

/* -------- Ajout Fabien --------- */
/************************************************************
 ************************************************************
 * Function that access to the robot's degrees of freedom
 */

/*
 *  Get the number of a degree of freedom for the current robot
 *
 *  Input:  the robot
 *
 *  Output: the number of degree of freedom
 */
int p3d_get_robot_ndof(void) 
{
  return(XYZ_ENV->cur_robot->nb_dof);
}


/*
 *  Get the value of a degree of freedom for the given robot
 *
 *  Input:  the robot, the index of the degree of freedom
 *
 *  Output: the value (expressed in radian for angles)
 */
void p3d_get_robot_dof(p3d_rob * robotPt, int i, double *val)
{
  int j;
  p3d_jnt * jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &j);
  *val = p3d_jnt_get_dof(jntPt, j);
}


/*
 *  Get the value of a degree of freedom for the given robot
 *
 *  Input:  the robot, the index of the degree of freedom
 *
 *  Output: the value (expressed in degree for angles)
 */
void p3d_get_robot_dof_deg(pp3d_rob robotPt, int i, double *val)
{
  int j;
  p3d_jnt * jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &j);
  *val = p3d_jnt_get_dof_deg(jntPt, j);
}


/*
 *  Get the value of the bounds for a given robot and degree of freedom
 *
 *  Input:  the robot, the index of the degree of freedom
 *
 *  Output: the minimum and maximum values (expressed in radian for angles)
 */
void p3d_get_robot_dof_bounds(p3d_rob * robotPt, int i,
			      double *vmin, double *vmax)
{
  int j;
  p3d_jnt * jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &j);
  p3d_jnt_get_dof_bounds(jntPt, j, vmin, vmax);
}


/*
 *  Get the value of the bounds for random shoot for a
 *      given robot and degree of freedom
 *
 *  Input:  the robot, the index of the degree of freedom
 *
 *  Output: the minimum and maximum values (expressed in radian for angles)
 */
void p3d_get_robot_dof_rand_bounds(p3d_rob * robotPt, int i, 
				   double *vmin, double *vmax)
{
  int j;
  p3d_jnt * jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &j);
  p3d_jnt_get_dof_rand_bounds(jntPt, j, vmin, vmax);
}


/*
 *  Get the value of the bounds for a given robot and degree of freedom
 *
 *  Input:  the robot, the index of the degree of freedom
 *
 *  Output: the minimum and maximum values (expressed in degree for angles)
 */
void p3d_get_robot_dof_bounds_deg(p3d_rob * robotPt, int i,
				  double *vmin, double *vmax)
{
  int j;
  p3d_jnt * jntPt;

  jntPt = p3d_robot_dof_to_jnt(robotPt, i, &j);
  p3d_jnt_get_dof_bounds_deg(jntPt, j, vmin, vmax);
}


/* 
 *  Get the configuration of the robot
 *
 *  Input:  the robot
 *
 *  Output: the configuration
 *
 *  Description: 
 */
void p3d_get_robot_config_into(p3d_rob* robotPt, configPt * config)
{
  int njnt = robotPt->njoints;
  int i, j;
  p3d_jnt * jntPt;

  if (*config == NULL){
    PrintError(("p3d_get_robot_config_into: failed\n"));
  } else {
    for (i=0; i<=njnt; i++){
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	(*config)[jntPt->index_dof+j] = p3d_jnt_get_dof(jntPt, j);
      }
    }
  }
}


/* 
 *  Get the configuration of the robot
 *
 *  Input:  the robot
 *
 *  Output: the configuration(expressed in degree for angles)
 *
 *  Description: 
 */
void p3d_get_robot_config_deg_into(p3d_rob* robotPt, configPt * config)
{
  int njnt = robotPt->njoints;
  int i, j;
  p3d_jnt * jntPt;

  if (*config == NULL){
    printf("p3d_get_robot_config_deg_into: failed\n");
  } else {
    for (i=0; i<=njnt; i++){
      jntPt = robotPt->joints[i];
      for(j=0; j<jntPt->dof_equiv_nbr; j++) {
	(*config)[jntPt->index_dof+j] = p3d_jnt_get_dof_deg(jntPt, j);
      }
    }
  }
}


/* 
 *  Get the configuration of the robot
 *
 *  Input:  the robot
 *
 *  Output: the configuration
 *
 *  Description: 
 */
configPt p3d_get_robot_config(p3d_rob* robotPt)
{
  configPt config;

  config = p3d_alloc_config(robotPt);
  if (config == NULL){
    PrintInfo(("p3d_get_robot_config: allocation failed\n"));
    return NULL;
  }
  p3d_get_robot_config_into(robotPt, &config);
  return config;
}


/* 
 *  Get the configuration of the robot
 *
 *  Input:  the robot
 *
 *  Output: the configuration(expressed in degree for angles)
 *
 *  Description: 
 */
configPt p3d_get_robot_config_deg(p3d_rob* robotPt)
{
  configPt config;

  config = p3d_alloc_config(robotPt);
  if (config == NULL){
    printf("p3d_get_robot_config_deg: allocation failed\n");
    return NULL;
  } 
  p3d_get_robot_config_deg_into(robotPt, &config);
  return config;
}


/* 
 *  Get the configuration min of the robot
 *
 *  Input:  the robot
 *
 *  Output: the configuration
 *
 *  Description: 
 */
configPt p3d_get_robot_min_config(p3d_rob* robotPt)
{
  int njnt = robotPt->njoints;
  configPt config;
  int i, j;
  double vmax;
  p3d_jnt * jntPt;

  config = p3d_alloc_config(robotPt);
  if (config == NULL){
    PrintInfo(("p3d_get_robot_config: allocation failed\n"));
    return NULL;
  }
  for (i=0; i<=njnt; i++){
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &(config[j+jntPt->index_dof]), &vmax);
    }
  }
  return config;
}


/* 
 *  Get the configuration max of the robot
 *
 *  Input:  the robot
 *
 *  Output: the configuration
 *
 *  Description: 
 */
configPt p3d_get_robot_max_config(p3d_rob* robotPt)
{
  int njnt = robotPt->njoints;
  configPt config;
  int i, j;
  double vmin;
  p3d_jnt * jntPt;

  config = p3d_alloc_config(robotPt);
  if (config == NULL){
    printf("p3d_get_robot_config: allocation failed\n");
    return NULL;
  }
  for (i=0; i<=njnt; i++){
    jntPt = robotPt->joints[i];
    for(j=0; j<jntPt->dof_equiv_nbr; j++) {
      p3d_jnt_get_dof_bounds(jntPt, j, &vmin, &(config[j+jntPt->index_dof]));
    }
  }
  return config;
}

/*-------- Fin ajout Fabien ---------*/


/****************************************************/
/* Fonction recuperant la valeur d'une articulation */
/* du robot courant     (angle en rad)              */
/* In : le numero de l'articulation                 */
/* Out : la valeur de l'articulation                */
/****************************************************/
void p3d_get_robot_jnt_rad(int i, double *val)
{p3d_jnt *j = XYZ_ENV->cur_robot->joints[i];

 *val = j->dof_data[0].v;
}

/* 
 *  limit values of a joint of the current robot
 *
 *  Input:  the index of the joint
 *
 *  Output: the minimum and maximum values (expressed in radian for angles)
 */

void p3d_get_robot_jnt_bounds(int i, double *vmin, double *vmax)
{p3d_jnt *j = XYZ_ENV->cur_robot->joints[i];

  *vmin = j->dof_data[0].vmin;
  *vmax = j->dof_data[0].vmax;
}


/* 
 *  limit values of a joint of the current robot
 *
 *  Input:  the index of the joint
 *
 *  Output: the minimum and maximum values (expressed in radian for angles)
 */

void p3d_get_robot_jnt_rand_bounds(int i, double *vmin, double *vmax)
{
  p3d_jnt *j = XYZ_ENV->cur_robot->joints[i];

  *vmin = j->dof_data[0].vmin_r;
  *vmax = j->dof_data[0].vmax_r;
}


/* 
 *  limit values of a joint of the current robot
 *
 *  Input:  the index of the joint
 *
 *  Output: the minimum and maximum values (expressed in degree for angles)
 */

void p3d_get_robot_jnt_bounds_deg(int i, double *vmin, double *vmax)
{
  p3d_jnt *j = XYZ_ENV->cur_robot->joints[i];

  if(p3d_jnt_is_dof_angular(j, 0)) {
    *vmin = RTOD(j->dof_data[0].vmin);
    *vmax = RTOD(j->dof_data[0].vmax);
  }
  else {
    *vmin = j->dof_data[0].vmin;
    *vmax = j->dof_data[0].vmax;
  }
}


/************************************************/
/************************************************/
/************************************************/

/****************************************************/
/* Fonction recuperant le nombre de sommets du      */
/* ieme polyhedre du body courant                   */
/* In : le numero du polyhedre                      */
/* Out : le nombre de sommets                       */
/****************************************************/
int p3d_get_body_npt(int i)
{
  return(p3d_get_nb_points(XYZ_ENV->cur_robot->ocur->pol[i]->poly));
}

/****************************************************/
/* Fonction recuperant le nombre de polyhedres      */
/* du body courant                                  */
/* In :                                             */
/* Out : le nombre de sommets                       */
/****************************************************/
int p3d_get_body_npoly(void)
{
  return(XYZ_ENV->cur_robot->ocur->np);
}

/****************************************************/
/* Fonction recuperant un sommet du body courant    */
/* In :                                             */
/* Out : le nombre de sommets                       */
/****************************************************/
void p3d_get_body_pt(int num, int i, double *x, double *y, double *z)
{

  /*tsiano p3d_get_vert_poly(((p3d_poly *) (&XYZ_ENV->cur_robot->ocur->pol)),i,x,y,z);*/
  p3d_get_point_2_d(((p3d_poly *) (XYZ_ENV->cur_robot->ocur->pol[num-1]))->poly,i,x,y,z);
}

/***************************************************/
/* Fonction recuperant la boite englobante du body */
/* courant                                         */
/* In :                                            */
/* Out : les coordonnees de la boite               */
/***************************************************/
void p3d_get_body_box(double *x1, double *x2, double *y1, double *y2,double *z1,double *z2)
{pp3d_box b = &XYZ_ENV->cur_robot->ocur->box;

  *x1 = b->x1;*x2 = b->x2;*y1 = b->y1;*y2 = b->y2;*z1 = b->z1;*z2 = b->z2;
}

/********************************************/
/* Fonction recuperant un body par son nom  */
/* In :                                     */
/* Out : son p3d_obj                            */
/********************************************/
p3d_obj *p3d_get_body_by_name(char *name)
{int b,nb,i;
 
   b = p3d_get_desc_curnum(P3D_BODY);
   nb= p3d_get_desc_number(P3D_BODY);
   for(i=0;i<nb;i++){
     p3d_sel_desc_num(P3D_BODY,i);
     if(strcmp(name,XYZ_ENV->cur_robot->ocur->name) == 0){
       return(XYZ_ENV->cur_robot->ocur);
     }
   }
   p3d_sel_desc_num(P3D_BODY,b);
   return(FALSE);
}

/********************************************/
/* Function returns joint number of joint   */
/* occuring before body with name "name"    */
/* In :       body name                     */
/* Out :number of joint preceeding the body */
/********************************************/
int p3d_get_parent_jnt_by_name(char *name)
{
  int jnt_nr = 0, nof_bodys, i;
  p3d_obj    *ith_body=NULL;

  if(XYZ_ROBOT)
    {
      nof_bodys = XYZ_ROBOT->no;
      for(i=0;i<nof_bodys;i++)
	{
	  ith_body = XYZ_ROBOT->o[i];
	  if(strcmp(ith_body->name,name)==0)
	    {
	      jnt_nr = ith_body->num;
	      i=nof_bodys; /* quit loop */
	    }
	}
    }
  return jnt_nr;
}

/********************************************/
/* Function returns robot number id by name */
/*                                          */
/* In :      robot name                     */
/* Out : robot number id/-1 if doesn't exist*/
/* J. PETTRE 30/05/01                       */
/********************************************/
int p3d_get_rob_nid_by_name (char *name)
{
  int i;
  for (i=0;i<XYZ_ENV->nr;i++) 
    if (XYZ_ENV->robot[i]!=NULL) 
      if(strcmp(name,XYZ_ENV->robot[i]->name)==0) 
	return i;
  return -1;
}

/********************************************/
/* Function returns robot number id by name */
/*                                          */
/* In :      robot name                     */
/* Out : robot number id/-1 if doesn't exist*/
/* J. PETTRE 30/05/01                       */
/********************************************/
int p3d_get_body_nid_by_name (char *name)
{
  int i;
  for (i=0;i<XYZ_ENV->cur_robot->no;i++) 
    if (XYZ_ENV->cur_robot->o[i]!=NULL) 
      if(strcmp(name,XYZ_ENV->cur_robot->o[i]->name)==0) 
	return i;
  return -1;
}

/************************************************/
/************************************************/
/************************************************/

/*
 *  Description: Return the number of Reeds and Shepp segment of the 
 *       current trajectory of the current robot. If there is no current 
 *       trajectory return 0.
 */
int p3d_get_traj_ncourbes(void) 
{
  pp3d_traj t = XYZ_ENV->cur_robot->tcur;
  if(t) return(t->nlp);
  else  return(0);
}

/*
 *  Description: Return the parameter range of the current trajectory 
 *       of the current robot. If there is no current 
 *       trajectory return 0.
 */
double p3d_get_cur_traj_rangeparam(void) 
{
  pp3d_traj t = XYZ_ENV->cur_robot->tcur;
  double range_param = 0;
  p3d_localpath *localpathPt;

  if(t){
    localpathPt = t->courbePt;
    while (localpathPt != NULL){
      range_param += localpathPt->range_param;
      localpathPt = localpathPt->next_lp;
    }
    return(range_param);
  }
  else  return(0);
}


/* Debut Modification Thibaut */

/****************************************************/
/* Fonction recuperant la normale d'un polyhedre    */
/* placee dans le repere global par sa matrice pos0 */
/****************************************************/

void p3d_get_plane_normalv_in_world_pos(p3d_poly *p, p3d_index face_index, p3d_vector3 normv)
{
  int i;
  p3d_vector3 u10,u11,u20,u21,p0,p1;

  p3d_get_point_2_v3(p->poly,(int)p3d_get_index_point_in_face(p->poly,face_index,1),&u10);
  p3d_get_point_2_v3(p->poly,(int)p3d_get_index_point_in_face(p->poly,face_index,2),&p0);
  p3d_get_point_2_v3(p->poly,(int)p3d_get_index_point_in_face(p->poly,face_index,3),&u20);

  p3d_xformVect(p->pos_rel_jnt,u10,u11);
  p3d_xformVect(p->pos_rel_jnt,u20,u21);
  p3d_xformVect(p->pos_rel_jnt,p0,p1); 
  
  for(i=0;i<3;i++)
    {
      u11[i]=p1[i]-u11[i];	
      u21[i]=u21[i]-p1[i];
    }      
  normv[0]=u11[1]*u21[2]-u11[2]*u21[1];
  normv[1]=u11[2]*u21[0]-u11[0]*u21[2];
  normv[2]=u11[0]*u21[1]-u11[1]*u21[0];
}

//! Retourne un pointeur sur le robot dont le nom est donne en parametre.
//! \param name the name of the robot
//! \return a pointer to the robot with the given name
p3d_rob* p3d_get_robot_by_name(const char *name)
{
  #ifdef DEBUG
   if(name==NULL)
   {
     printf("%s: %d: p3d_get_robot_by_name(): name is NULL.\n", __FILE__, __LINE__);
     return NULL;
   }
  #endif

   int i;

   for(i=0; i<XYZ_ENV->nr; i++)
   {
     if(strcmp(name, XYZ_ENV->robot[i]->name)==0)
     {  return(XYZ_ENV->robot[i]);  }
   }

//    printf("%s: %d: p3d_get_robot_by_name(): there is no robot named \"%s\".\n", __FILE__, __LINE__, name);

   return NULL;
}

//! Retourne un pointeur sur le robot dont le nom contient donne en parametre.
//! \param name the name of the robot
//! \return a pointer to the robot with the given name
p3d_rob* p3d_get_robot_by_name_containing(const char *name)
{
#ifdef DEBUG
	if(name==NULL)
	{
		printf("%s: %d: p3d_get_robot_by_name(): name is NULL.\n", __FILE__, __LINE__);
		return NULL;
	}
#endif
	
	for(int i=0; i<XYZ_ENV->nr; i++)
	{
		printf("Robot[%d] is %s\n",i,XYZ_ENV->robot[i]->name);
		if( strcasestr(XYZ_ENV->robot[i]->name,name) != NULL )
		{  return(XYZ_ENV->robot[i]);  }
	}
	
	//    printf("%s: %d: p3d_get_robot_by_name(): there is no robot named \"%s\".\n", __FILE__, __LINE__, name);
	
	return NULL;
}

//! Cette fonction permet de retrouver l'indice d'une liaison dans le tableau des liaisons d'un robot,
//! a partir de son nom.
//! Find the index of a robot joint from its name.
//! It is the index in the robot's joint array and it starts from 0.
//! \param robot pointer to the robot
//! \param name name of the searched joint
//! \return the index of the joint if it is found, 0 otherwise
int get_robot_jnt_index_by_name(p3d_rob* robot, char *name)
{
  if(robot==NULL)
  {
    printf("%s: %d: get_robot_jnt_index_by_name(): robot is NULL.\n", __FILE__, __LINE__);
    return 0;
  }
  if(name==NULL)
  {
    printf("%s: %d: p3d_get_robot_jnt_index_by_name(): name is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  int i;
  for(i=0; i<=robot->njoints; i++)
  {
     if(robot->joints[i]->name==NULL)
       continue;

     if( strcmp(robot->joints[i]->name, name) == 0 )
     {
        return i;
     }
  }

  printf("%s: %d: p3d_get_robot_jnt_index_by_name(): robot \"%s\" has no joint named \"%s\".\n", __FILE__, __LINE__, robot->name, name);
  return 0;

}

//! Finds a robot's joint from its name.
//! \param robot pointer to the robot
//! \param name name of the searched joint
//! \return pointer to the joint if it is found, NULL otherwise
p3d_jnt * p3d_get_robot_jnt_by_name(p3d_rob* robot, char *name)
{
  if(robot==NULL)
  {
    printf("%s: %d: p3d_get_robot_jnt_by_name(): robot is NULL (while looking for \"%s\" joint).\n", __FILE__, __LINE__,name);
    return NULL;
  }
  if(name==NULL)
  {
    printf("%s: %d: p3d_get_robot_jnt_by_name(): joint name is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }

  int i;
  for(i=0; i<=robot->njoints; i++)
  {
     if(robot->joints[i]->name==NULL)
       continue;

     if( strcmp(robot->joints[i]->name, name) == 0 )
     {
        return robot->joints[i];
     }
  }

  printf("%s: %d: p3d_get_robot_jnt_by_name(): robot \"%s\" has no joint named \"%s\".\n", __FILE__, __LINE__, robot->name, name);

  return NULL;
}



//! Finds a robot body from its name.
//! \param robot pointer to the robot
//! \param name name of the searched body (without the prefix "robot_name.")
//! \return pointer to the joint if it is found, NULL otherwise
p3d_obj * p3d_get_robot_body_by_name(p3d_rob* robot, char *name)
{
  if(robot==NULL)
  {
    printf("%s: %d: p3d_get_robot_body_by_name(): robot is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }
  if(name==NULL)
  {
    printf("%s: %d: p3d_get_robot_body_by_name(): name is NULL.\n", __FILE__, __LINE__);
    return NULL;
  }

  int i;
  std::string body_name;
  body_name= std::string(robot->name) + "." + std::string(name);

  for(i=0; i<robot->no; i++)
  {
     if(robot->o[i]->name==NULL)
     {  continue;  }

     if( strcmp(robot->o[i]->name, body_name.c_str()) == 0 )
     {
        return robot->o[i];
     }
  }

//   printf("%s: %d: get_robot_body_by_name(): robot \"%s\" has no body named \"%s\".\n", __FILE__, __LINE__, robot->name, name);

  return NULL;
}

//! Gets the pose of the firts joint of the robot.
//! \return 0 in case of success, 1 otherwise
int p3d_get_first_joint_pose(p3d_rob *robotPt, p3d_matrix4 pose)
{
  if(robotPt==NULL) {
    printf("%s: %d: p3d_get_first_joint_pose(): input robot is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  p3d_jnt *joint= NULL;

  joint= robotPt->joints[1];

  p3d_mat4Copy(joint->abs_pos, pose);

  return 0;
}

int p3d_get_body_pose(p3d_rob *robotPt, int index, p3d_matrix4 pose)
{
  if(robotPt==NULL) {
    printf("%s: %d: p3d_get_body_pose(): input robot is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  if( (index < 0) || (index > robotPt->no-1) ) {
    printf("%s: %d: p3d_get_body_pose(): wrong body index.\n", __FILE__, __LINE__);
    return 1;
  }


  p3d_mat4Copy(robotPt->o[index]->jnt->abs_pos, pose);

//   p3d_matMultXform(robotPt->o[index]->jnt, robotPt->o[index]->pol[0]->pos_rel_jnt, pose);

  return 0;
}

/* Fin Modification Thibaut */

//! Gets the current pose of a freeflyer robot.
//! \param robotPt pointer to the robot
//! \param pose pose matrix
//! \return 0 in case of success, 1 otherwise
int p3d_get_freeflyer_pose(p3d_rob *robotPt, p3d_matrix4 pose)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: p3d_set_freeflyer_pose(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  p3d_jnt *firstJoint= NULL;

  firstJoint= robotPt->joints[1];

  if(firstJoint->type!=P3D_FREEFLYER)
  {
    printf("%s: %d: p3d_get_freeflyer_pose(): the first joint of robot \"%s\" is not of type P3D_FREEFLYER.\n",__FILE__,__LINE__,robotPt->name);
    return 1;
  }

  p3d_mat4Copy(firstJoint->abs_pos, pose);

  return 0;
}


//! Gets the current configuration of a freeflyer robot.
//! \param robotPt pointer to the robot
//! \param x coordinate along X axis
//! \param y coordinate along Y axis
//! \param z coordinate along Z axis
//! \param rx first Euler angle (in rads)
//! \param ry second Euler angle (in rads)
//! \param rz third Euler angle (in rads)
//! \return 0 in case of success, 1 otherwise
int p3d_get_freeflyer_pose2(p3d_rob *robotPt, double *x, double *y, double *z, double *rx, double *ry, double *rz)
{
  if(robotPt==NULL)
  {
    printf("%s: %d: p3d_get_freeflyer_pose2(): input p3d_rob* is NULL.\n",__FILE__,__LINE__);
    return 1;
  }

  configPt q= NULL;
  p3d_jnt *firstJoint= NULL;

  firstJoint= robotPt->joints[1];

  if(firstJoint->type!=P3D_FREEFLYER)
  {
    printf("%s: %d: p3d_get_freeflyer_pose2(): the first joint of robot \"%s\" is not of type P3D_FREEFLYER.\n",__FILE__,__LINE__,robotPt->name);
    return 1;
  }

  q= p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &q);

  *x= q[firstJoint->index_dof + 0];
  *y= q[firstJoint->index_dof + 1];
  *z= q[firstJoint->index_dof + 2];
  *rx= q[firstJoint->index_dof + 3];
  *ry= q[firstJoint->index_dof + 4];
  *rz= q[firstJoint->index_dof + 5];

  p3d_destroy_config(robotPt, q);

  return 0;
}
